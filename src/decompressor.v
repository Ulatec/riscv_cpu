// RISC-V C Extension (RV32C) Decompressor
// Converts 16-bit compressed instructions to 32-bit RV32I/M equivalents
// Reference: RISC-V ISA Specification, Chapter 16 (RVC)

module decompressor (
    input  [31:0] fetched_word,    // Raw 32-bit word from memory
    input         pc_bit1,          // PC[1] - which half to use
    input  [15:0] saved_half,       // Previously saved upper half (for spanning)
    input         have_saved,       // Whether saved_half is valid
    output [31:0] instruction,      // Decompressed 32-bit instruction
    output        is_compressed,    // 1 = was 16-bit instruction
    output        need_next_half    // 1 = 32-bit spans word boundary
);

    // Select which 16-bit half based on PC[1]
    // PC[1]=0: use lower half (bits 15:0)
    // PC[1]=1: use upper half (bits 31:16)
    wire [15:0] current_half = pc_bit1 ? fetched_word[31:16] : fetched_word[15:0];

    // For spanning instructions: combine saved upper half with current lower half
    wire [31:0] spanning_word = {fetched_word[15:0], saved_half};

    // Check if current half is compressed (bits [1:0] != 2'b11)
    wire is_compressed_half = (current_half[1:0] != 2'b11);

    // Check if we need next half (32-bit instruction at PC[1]=1)
    // This happens when PC[1]=1 and instruction is not compressed
    assign need_next_half = pc_bit1 && !is_compressed_half && !have_saved;

    // Output is_compressed flag
    assign is_compressed = is_compressed_half && !have_saved;

    // Extract compressed instruction fields
    wire [1:0]  c_op     = current_half[1:0];     // Quadrant (00, 01, 10)
    wire [2:0]  c_funct3 = current_half[15:13];   // Function field
    wire [4:0]  c_rd_rs1 = current_half[11:7];    // rd/rs1 for some formats
    wire [4:0]  c_rs2    = current_half[6:2];     // rs2 for some formats

    // Compressed register fields (3-bit maps to x8-x15)
    wire [4:0]  c_rd_prime  = {2'b01, current_half[4:2]};   // rd' = x8-x15
    wire [4:0]  c_rs1_prime = {2'b01, current_half[9:7]};   // rs1' = x8-x15
    wire [4:0]  c_rs2_prime = {2'b01, current_half[4:2]};   // rs2' = x8-x15

    // Pre-compute immediates outside the always block
    // C.ADDI4SPN: nzuimm[9:2]
    wire [9:0] c_addi4spn_imm = {current_half[10:7], current_half[12:11], current_half[5], current_half[6], 2'b00};

    // C.LW/C.SW offset
    wire [6:0] c_lw_sw_offset = {current_half[5], current_half[12:10], current_half[6], 2'b00};

    // C.ADDI/C.LI signed immediate
    wire [5:0] c_imm6 = {current_half[12], current_half[6:2]};
    wire [11:0] c_imm6_sext = {{6{c_imm6[5]}}, c_imm6};

    // C.JAL/C.J offset (11 bits, sign-extended)
    wire [11:0] c_j_offset = {current_half[12], current_half[8], current_half[10:9], current_half[6],
                              current_half[7], current_half[2], current_half[11], current_half[5:3], 1'b0};
    wire [20:0] c_j_offset_sext = {{9{c_j_offset[11]}}, c_j_offset};

    // C.BEQZ/C.BNEZ offset (8 bits, sign-extended)
    wire [8:0] c_b_offset = {current_half[12], current_half[6:5], current_half[2],
                             current_half[11:10], current_half[4:3], 1'b0};
    wire [12:0] c_b_offset_sext = {{4{c_b_offset[8]}}, c_b_offset};

    // C.ADDI16SP immediate
    wire [9:0] c_addi16sp_imm = {current_half[12], current_half[4:3], current_half[5],
                                  current_half[2], current_half[6], 4'b0000};
    wire [11:0] c_addi16sp_sext = {{2{c_addi16sp_imm[9]}}, c_addi16sp_imm};

    // C.LUI immediate (17:12)
    wire [5:0] c_lui_imm6 = {current_half[12], current_half[6:2]};

    // Shift amount for C.SLLI/C.SRLI/C.SRAI
    wire [5:0] c_shamt = {current_half[12], current_half[6:2]};

    // C.LWSP offset
    wire [7:0] c_lwsp_offset = {current_half[3:2], current_half[12], current_half[6:4], 2'b00};

    // C.SWSP offset
    wire [7:0] c_swsp_offset = {current_half[8:7], current_half[12:9], 2'b00};

    // Sub-function bits for C1.100
    wire [1:0] c_funct2_hi = current_half[11:10];
    wire [1:0] c_funct2_lo = current_half[6:5];

    // Decompressed instruction (default to NOP for illegal/reserved)
    reg [31:0] decompressed;

    always @(*) begin
        // Default: NOP (ADDI x0, x0, 0)
        decompressed = 32'h00000013;

        case (c_op)
            // =====================================================================
            // Quadrant 0 (C0): c_op = 2'b00
            // =====================================================================
            2'b00: begin
                case (c_funct3)
                    // C.ADDI4SPN: addi rd', x2, nzuimm
                    3'b000: begin
                        if (c_addi4spn_imm == 10'b0) begin
                            // Illegal (nzuimm=0 is reserved)
                            decompressed = 32'h00000013; // NOP
                        end else begin
                            // addi rd', x2, nzuimm
                            decompressed = {2'b00, c_addi4spn_imm, 5'd2, 3'b000, c_rd_prime, 7'b0010011};
                        end
                    end

                    // C.LW: lw rd', offset(rs1')
                    3'b010: begin
                        // lw rd', offset(rs1')
                        decompressed = {5'b00000, c_lw_sw_offset, c_rs1_prime, 3'b010, c_rd_prime, 7'b0000011};
                    end

                    // C.SW: sw rs2', offset(rs1')
                    3'b110: begin
                        // sw rs2', offset(rs1') -> imm[11:5], rs2, rs1, funct3, imm[4:0], opcode
                        decompressed = {5'b00000, c_lw_sw_offset[6:5], c_rs2_prime, c_rs1_prime, 3'b010, c_lw_sw_offset[4:0], 7'b0100011};
                    end

                    default: begin
                        // Reserved/FLD/FSD (floating point) - treat as NOP
                        decompressed = 32'h00000013;
                    end
                endcase
            end

            // =====================================================================
            // Quadrant 1 (C1): c_op = 2'b01
            // =====================================================================
            2'b01: begin
                case (c_funct3)
                    // C.NOP / C.ADDI: addi rd, rd, imm
                    3'b000: begin
                        if (c_rd_rs1 == 5'b0) begin
                            // C.NOP
                            decompressed = 32'h00000013;
                        end else begin
                            // C.ADDI rd, rd, imm
                            decompressed = {c_imm6_sext, c_rd_rs1, 3'b000, c_rd_rs1, 7'b0010011};
                        end
                    end

                    // C.JAL: jal x1, offset (RV32 only)
                    3'b001: begin
                        // jal x1, offset
                        // JAL encoding: imm[20|10:1|11|19:12], rd, opcode
                        decompressed = {c_j_offset_sext[20], c_j_offset_sext[10:1], c_j_offset_sext[11], c_j_offset_sext[19:12], 5'd1, 7'b1101111};
                    end

                    // C.LI: addi rd, x0, imm
                    3'b010: begin
                        if (c_rd_rs1 == 5'b0) begin
                            // rd=0 is HINT, treat as NOP
                            decompressed = 32'h00000013;
                        end else begin
                            // addi rd, x0, imm
                            decompressed = {c_imm6_sext, 5'd0, 3'b000, c_rd_rs1, 7'b0010011};
                        end
                    end

                    // C.LUI / C.ADDI16SP
                    3'b011: begin
                        if (c_rd_rs1 == 5'd2) begin
                            // C.ADDI16SP: addi x2, x2, nzimm
                            if (c_addi16sp_imm == 10'b0) begin
                                // Reserved (nzimm=0)
                                decompressed = 32'h00000013;
                            end else begin
                                // addi x2, x2, nzimm
                                decompressed = {c_addi16sp_sext, 5'd2, 3'b000, 5'd2, 7'b0010011};
                            end
                        end else if (c_rd_rs1 == 5'b0) begin
                            // rd=0 is reserved
                            decompressed = 32'h00000013;
                        end else begin
                            // C.LUI: lui rd, nzimm
                            if (c_lui_imm6 == 6'b0) begin
                                // Reserved (nzimm=0)
                                decompressed = 32'h00000013;
                            end else begin
                                // lui rd, nzimm[17:12] (sign-extended to 20 bits)
                                decompressed = {{14{c_lui_imm6[5]}}, c_lui_imm6, c_rd_rs1, 7'b0110111};
                            end
                        end
                    end

                    // C.SRLI, C.SRAI, C.ANDI, C.SUB, C.XOR, C.OR, C.AND
                    3'b100: begin
                        case (c_funct2_hi)
                            // C.SRLI: srli rd', rd', shamt
                            2'b00: begin
                                if (c_shamt[5:0] == 6'b0 || c_shamt[5] == 1'b1) begin
                                    // shamt=0 is HINT for RV32, shamt[5]=1 reserved
                                    decompressed = 32'h00000013;
                                end else begin
                                    // srli rd', rd', shamt
                                    decompressed = {7'b0000000, c_shamt[4:0], c_rs1_prime, 3'b101, c_rs1_prime, 7'b0010011};
                                end
                            end

                            // C.SRAI: srai rd', rd', shamt
                            2'b01: begin
                                if (c_shamt[5:0] == 6'b0 || c_shamt[5] == 1'b1) begin
                                    // shamt=0 is HINT for RV32, shamt[5]=1 reserved
                                    decompressed = 32'h00000013;
                                end else begin
                                    // srai rd', rd', shamt
                                    decompressed = {7'b0100000, c_shamt[4:0], c_rs1_prime, 3'b101, c_rs1_prime, 7'b0010011};
                                end
                            end

                            // C.ANDI: andi rd', rd', imm
                            2'b10: begin
                                // andi rd', rd', imm
                                decompressed = {c_imm6_sext, c_rs1_prime, 3'b111, c_rs1_prime, 7'b0010011};
                            end

                            // C.SUB, C.XOR, C.OR, C.AND
                            2'b11: begin
                                if (current_half[12] == 1'b0) begin
                                    case (c_funct2_lo)
                                        // C.SUB: sub rd', rd', rs2'
                                        2'b00: decompressed = {7'b0100000, c_rs2_prime, c_rs1_prime, 3'b000, c_rs1_prime, 7'b0110011};
                                        // C.XOR: xor rd', rd', rs2'
                                        2'b01: decompressed = {7'b0000000, c_rs2_prime, c_rs1_prime, 3'b100, c_rs1_prime, 7'b0110011};
                                        // C.OR: or rd', rd', rs2'
                                        2'b10: decompressed = {7'b0000000, c_rs2_prime, c_rs1_prime, 3'b110, c_rs1_prime, 7'b0110011};
                                        // C.AND: and rd', rd', rs2'
                                        2'b11: decompressed = {7'b0000000, c_rs2_prime, c_rs1_prime, 3'b111, c_rs1_prime, 7'b0110011};
                                    endcase
                                end else begin
                                    // Reserved (bit 12 = 1 for this encoding in RV32C)
                                    decompressed = 32'h00000013;
                                end
                            end
                        endcase
                    end

                    // C.J: jal x0, offset
                    3'b101: begin
                        // jal x0, offset
                        decompressed = {c_j_offset_sext[20], c_j_offset_sext[10:1], c_j_offset_sext[11], c_j_offset_sext[19:12], 5'd0, 7'b1101111};
                    end

                    // C.BEQZ: beq rs1', x0, offset
                    3'b110: begin
                        // beq rs1', x0, offset
                        // B-type: imm[12|10:5], rs2, rs1, funct3, imm[4:1|11], opcode
                        decompressed = {c_b_offset_sext[12], c_b_offset_sext[10:5], 5'd0, c_rs1_prime, 3'b000, c_b_offset_sext[4:1], c_b_offset_sext[11], 7'b1100011};
                    end

                    // C.BNEZ: bne rs1', x0, offset
                    3'b111: begin
                        // bne rs1', x0, offset
                        decompressed = {c_b_offset_sext[12], c_b_offset_sext[10:5], 5'd0, c_rs1_prime, 3'b001, c_b_offset_sext[4:1], c_b_offset_sext[11], 7'b1100011};
                    end
                endcase
            end

            // =====================================================================
            // Quadrant 2 (C2): c_op = 2'b10
            // =====================================================================
            2'b10: begin
                case (c_funct3)
                    // C.SLLI: slli rd, rd, shamt
                    3'b000: begin
                        if (c_rd_rs1 == 5'b0 || c_shamt[5] == 1'b1) begin
                            // rd=0 is reserved, shamt[5]=1 is reserved for RV32
                            decompressed = 32'h00000013;
                        end else if (c_shamt[4:0] == 5'b0) begin
                            // shamt=0 is HINT
                            decompressed = 32'h00000013;
                        end else begin
                            // slli rd, rd, shamt
                            decompressed = {7'b0000000, c_shamt[4:0], c_rd_rs1, 3'b001, c_rd_rs1, 7'b0010011};
                        end
                    end

                    // C.LWSP: lw rd, offset(x2)
                    3'b010: begin
                        if (c_rd_rs1 == 5'b0) begin
                            // rd=0 is reserved
                            decompressed = 32'h00000013;
                        end else begin
                            // lw rd, offset(x2)
                            decompressed = {4'b0000, c_lwsp_offset, 5'd2, 3'b010, c_rd_rs1, 7'b0000011};
                        end
                    end

                    // C.JR, C.MV, C.EBREAK, C.JALR, C.ADD
                    3'b100: begin
                        if (current_half[12] == 1'b0) begin
                            if (c_rs2 == 5'b0) begin
                                // C.JR: jalr x0, rs1, 0
                                if (c_rd_rs1 == 5'b0) begin
                                    // Reserved
                                    decompressed = 32'h00000013;
                                end else begin
                                    // jalr x0, rs1, 0
                                    decompressed = {12'b0, c_rd_rs1, 3'b000, 5'd0, 7'b1100111};
                                end
                            end else begin
                                // C.MV: add rd, x0, rs2
                                if (c_rd_rs1 == 5'b0) begin
                                    // HINT
                                    decompressed = 32'h00000013;
                                end else begin
                                    // add rd, x0, rs2
                                    decompressed = {7'b0000000, c_rs2, 5'd0, 3'b000, c_rd_rs1, 7'b0110011};
                                end
                            end
                        end else begin
                            if (c_rs2 == 5'b0) begin
                                if (c_rd_rs1 == 5'b0) begin
                                    // C.EBREAK: ebreak
                                    decompressed = 32'h00100073;
                                end else begin
                                    // C.JALR: jalr x1, rs1, 0
                                    decompressed = {12'b0, c_rd_rs1, 3'b000, 5'd1, 7'b1100111};
                                end
                            end else begin
                                // C.ADD: add rd, rd, rs2
                                if (c_rd_rs1 == 5'b0) begin
                                    // HINT
                                    decompressed = 32'h00000013;
                                end else begin
                                    // add rd, rd, rs2
                                    decompressed = {7'b0000000, c_rs2, c_rd_rs1, 3'b000, c_rd_rs1, 7'b0110011};
                                end
                            end
                        end
                    end

                    // C.SWSP: sw rs2, offset(x2)
                    3'b110: begin
                        // sw rs2, offset(x2)
                        // S-type: imm[11:5], rs2, rs1, funct3, imm[4:0], opcode
                        decompressed = {4'b0000, c_swsp_offset[7:5], c_rs2, 5'd2, 3'b010, c_swsp_offset[4:0], 7'b0100011};
                    end

                    default: begin
                        // Reserved (FLDSP, FSDSP - floating point)
                        decompressed = 32'h00000013;
                    end
                endcase
            end

            // =====================================================================
            // Quadrant 3: c_op = 2'b11 (Not compressed - shouldn't reach here)
            // =====================================================================
            2'b11: begin
                // This is a 32-bit instruction, pass through
                decompressed = 32'h00000013;
            end
        endcase
    end

    // Output selection:
    // - If we have a saved half, we're completing a spanning 32-bit instruction
    // - If current is compressed, output decompressed
    // - If current is 32-bit at PC[1]=0, pass through the word directly
    // - If current is 32-bit at PC[1]=1 and we have saved half, use spanning_word
    assign instruction = have_saved ? spanning_word :
                         is_compressed_half ? decompressed :
                         pc_bit1 ? 32'h00000013 :  // Waiting for next half
                         fetched_word;             // 32-bit at PC[1]=0

endmodule
