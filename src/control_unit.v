`include "../src/definitions.v"
module control_unit(
    // Inputs - Instruction fields
    input [4:0]  opcode,      // instruction[6:2]
    input [2:0]  funct3,      // instruction[14:12]
    input [6:0]  funct7,      // instruction[31:25]
    input [4:0]  rd,          // instruction[11:7] (for CSR, checking x0)
    input [4:0]  rs1,         // instruction[19:15] (for CSR immediate)
    input [4:0]  rs2,
    // Outputs - Control signals
    output reg [4:0]  alu_op,
    output reg [1:0]  alu_in1_src,
    output reg        alusrc,
    output reg        mem_read,
    output reg        mem_write,
    output reg        reg_write,
    output reg        mem_to_reg,
    
    // Outputs - Instruction type flags
    output reg        is_branch,
    output reg        is_jal,
    output reg        is_jalr,
    output reg        is_csr,
    output reg        is_ecall,
    output reg        is_ebreak,
    output reg        is_mret,
    // Atomic (A extension)
    output wire       is_lr,
    output wire       is_sc,
    output wire       is_amo,
    output wire [4:0] amo_funct5
);
wire is_mext = (funct7 == 7'b0000001);

// Atomic instruction detection
wire is_atomic = (opcode == 5'b01011);        // AMO opcode
assign amo_funct5 = funct7[6:2];              // instruction[31:27]
assign is_lr  = is_atomic && (amo_funct5 == 5'b00010);
assign is_sc  = is_atomic && (amo_funct5 == 5'b00011);
assign is_amo = is_atomic && !is_lr && !is_sc;
// Control Unit Logic (Combinational)
    always @(*) begin
        // Default values
        alu_op = `ALU_ADD;
        alusrc = 1'b0;
        alu_in1_src = 2'b00;
        mem_read = 1'b0;
        mem_write = 1'b0;
        reg_write = 1'b0;
        mem_to_reg = 1'b0;
        is_branch = 1'b0;
        is_jal = 1'b0;
        is_jalr = 1'b0;
        is_csr = 1'b0;
        is_ecall = 1'b0;
        is_ebreak = 1'b0;
        is_mret = 1'b0;
        case (opcode)
            5'b01100: begin // R-type (Base + M extension)
                reg_write = 1'b1;
                
                if (is_mext) begin
                    // M-extension: funct3 directly maps to operation
                    case (funct3)
                        3'b000: alu_op = `ALU_MUL;
                        3'b001: alu_op = `ALU_MULH;
                        3'b010: alu_op = `ALU_MULHSU;
                        3'b011: alu_op = `ALU_MULHU;
                        3'b100: alu_op = `ALU_DIV;
                        3'b101: alu_op = `ALU_DIVU;
                        3'b110: alu_op = `ALU_REM;
                        3'b111: alu_op = `ALU_REMU;
                    endcase
                end else begin
                    // Base R-type instructions
                    case (funct3)
                        3'b000: alu_op = funct7[5] ? `ALU_SUB : `ALU_ADD;
                        3'b001: alu_op = `ALU_SLL;
                        3'b010: alu_op = `ALU_SLT;
                        3'b011: alu_op = `ALU_SLTU;
                        3'b100: alu_op = `ALU_XOR;
                        3'b101: alu_op = funct7[5] ? `ALU_SRA : `ALU_SRL;
                        3'b110: alu_op = `ALU_OR;
                        3'b111: alu_op = `ALU_AND;
                    endcase
                end
            end
            
            5'b00000: begin // Load
                reg_write = 1'b1;
                alusrc = 1'b1;
                mem_read = 1'b1;
                mem_to_reg = 1'b1;
                alu_op = `ALU_ADD;
            end
            
            5'b01000: begin // Store
                alusrc = 1'b1;
                mem_write = 1'b1;
                alu_op = `ALU_ADD;
            end
            
            5'b11000: begin // Branch
                is_branch = 1'b1;
                alu_op = `ALU_SUB;
            end
            
            5'b11011: begin // JAL
                is_jal = 1'b1;
                reg_write = 1'b1;
                alu_in1_src = 2'b11; // PC+4
            end
            
            5'b11001: begin // JALR
                is_jalr = 1'b1;
                reg_write = 1'b1;
                alusrc = 1'b1;
                alu_op = `ALU_ADD;
            end
            
            5'b01101: begin // LUI
                reg_write = 1'b1;
                alusrc = 1'b1;
                alu_in1_src = 2'b10; // Zero
                alu_op = `ALU_ADD;
            end
            
            5'b00101: begin // AUIPC
                reg_write = 1'b1;
                alusrc = 1'b1;
                alu_in1_src = 2'b01; // PC
                alu_op = `ALU_ADD;
            end
            
            5'b11100: begin // System
                if (funct3 == 3'b000) begin
                    if (funct7 == 7'b0011000 && rs2 == 5'b00010) begin
                        is_mret = 1'b1;           // MRET: funct7=0x18, rs2=2
                    end
                    else if (rs2[0]) begin
                        is_ebreak = 1'b1;         // EBREAK: rs2=1
                    end
                    else begin
                        is_ecall = 1'b1;          // ECALL: rs2=0
                    end
                end else begin
                    is_csr = 1'b1;
                    reg_write = 1'b1;
                    alusrc = (funct3[2]);
                end
            end
            5'b01011: begin // Atomic (LR.W, SC.W, AMO*.W)
                reg_write  = 1'b1;      // All atomic ops write to rd
                alusrc     = 1'b1;      // Immediate (0) for address passthrough
                mem_read   = 1'b1;      // Read memory (LR loads, AMO reads before modify)
                mem_to_reg = 1'b1;      // Result comes through memory/atomic path
                alu_op     = `ALU_ADD;  // Address = rs1 + 0
            end

            5'b00100: begin // I-type arithmetic (ADDI, XORI, SLLI, etc.)
                reg_write = 1'b1;
                alusrc = 1'b1;
                case (funct3)
                    3'b000: alu_op = `ALU_ADD;
                    3'b001: alu_op = `ALU_SLL;
                    3'b010: alu_op = `ALU_SLT;
                    3'b011: alu_op = `ALU_SLTU;
                    3'b100: alu_op = `ALU_XOR;
                    3'b101: alu_op = funct7[5] ? `ALU_SRA : `ALU_SRL;
                    3'b110: alu_op = `ALU_OR;
                    3'b111: alu_op = `ALU_AND;
                endcase
            end
        endcase
    end
endmodule