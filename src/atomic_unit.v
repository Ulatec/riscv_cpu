// Atomic Operations Unit for RISC-V A Extension
// Implements LR.W, SC.W, and AMO instructions
//
// For a single-core processor, atomicity is straightforward since there's
// no other core that could interfere. The reservation mechanism for LR/SC
// still needs to track the reserved address and validity.
//
// Instruction encodings (all have opcode 0101111, funct3=010 for .W):
//   funct7[31:27]  Instruction
//   00010          LR.W
//   00011          SC.W
//   00001          AMOSWAP.W
//   00000          AMOADD.W
//   00100          AMOXOR.W
//   01100          AMOAND.W
//   01000          AMOOR.W
//   10000          AMOMIN.W
//   10100          AMOMAX.W
//   11000          AMOMINU.W
//   11100          AMOMAXU.W

module atomic_unit (
    input         clk,
    input         rst,
    
    // Operation selection
    input  [4:0]  amo_op,        // funct7[31:27] - operation type
    input         is_lr,         // LR.W instruction
    input         is_sc,         // SC.W instruction
    input         is_amo,        // AMO instruction (not LR/SC)
    
    // Operands
    input  [31:0] addr,          // Memory address (from rs1)
    input  [31:0] rs2_data,      // Source data (for SC and AMO)
    input  [31:0] mem_rdata,     // Data read from memory
    
    // Outputs
    output [31:0] mem_wdata,     // Data to write to memory
    output [31:0] rd_data,       // Data to write to rd
    output        sc_success,    // SC succeeded (reservation valid)
    output        do_mem_write,  // Should perform memory write
    
    // Reservation management
    input         clear_reservation,  // External clear (e.g., context switch)
    input         enable,             // Gate reservation updates (deassert during MMU stall)
    output        reservation_valid,  // Current reservation status
    output [31:0] reservation_addr    // Current reserved address
);

    // =========================================================================
    // AMO Operation Codes (funct7[31:27])
    // =========================================================================
    localparam AMO_LR      = 5'b00010;
    localparam AMO_SC      = 5'b00011;
    localparam AMO_SWAP    = 5'b00001;
    localparam AMO_ADD     = 5'b00000;
    localparam AMO_XOR     = 5'b00100;
    localparam AMO_AND     = 5'b01100;
    localparam AMO_OR      = 5'b01000;
    localparam AMO_MIN     = 5'b10000;
    localparam AMO_MAX     = 5'b10100;
    localparam AMO_MINU    = 5'b11000;
    localparam AMO_MAXU    = 5'b11100;

    // =========================================================================
    // Reservation Register (for LR/SC)
    // =========================================================================
    reg        resv_valid;
    reg [31:0] resv_addr;
    
    assign reservation_valid = resv_valid;
    assign reservation_addr = resv_addr;
    
    // SC succeeds if reservation is valid and address matches
    wire sc_match = resv_valid && (resv_addr == addr);
    assign sc_success = is_sc && sc_match;
    
    // =========================================================================
    // AMO Computation
    // =========================================================================
    
    // Signed comparison for MIN/MAX
    wire signed [31:0] mem_signed = $signed(mem_rdata);
    wire signed [31:0] rs2_signed = $signed(rs2_data);
    
    // AMO result computation
    reg [31:0] amo_result;
    
    always @(*) begin
        case (amo_op)
            AMO_SWAP: amo_result = rs2_data;
            AMO_ADD:  amo_result = mem_rdata + rs2_data;
            AMO_XOR:  amo_result = mem_rdata ^ rs2_data;
            AMO_AND:  amo_result = mem_rdata & rs2_data;
            AMO_OR:   amo_result = mem_rdata | rs2_data;
            AMO_MIN:  amo_result = (mem_signed < rs2_signed) ? mem_rdata : rs2_data;
            AMO_MAX:  amo_result = (mem_signed > rs2_signed) ? mem_rdata : rs2_data;
            AMO_MINU: amo_result = (mem_rdata < rs2_data) ? mem_rdata : rs2_data;
            AMO_MAXU: amo_result = (mem_rdata > rs2_data) ? mem_rdata : rs2_data;
            default:  amo_result = rs2_data;
        endcase
    end
    
    // =========================================================================
    // Output Muxes
    // =========================================================================
    
    // Data to write to memory:
    // - SC: rs2_data (if successful)
    // - AMO: computed result
    assign mem_wdata = is_sc ? rs2_data : amo_result;
    
    // Data to write to rd:
    // - LR: mem_rdata (loaded value)
    // - SC: 0 if success, 1 if failure
    // - AMO (including SWAP): mem_rdata (old value before operation)
    assign rd_data = is_lr       ? mem_rdata :
                     is_sc       ? (sc_success ? 32'h0 : 32'h1) :
                     is_amo      ? mem_rdata :
                     32'h0;

    // Should we write to memory?
    // - SC: only if reservation valid
    // - AMO (including SWAP): always write (needed for xchg, atomic counters, etc.)
    // - LR: never (just a load)
    assign do_mem_write = (is_sc && sc_success) || is_amo;
    
    // =========================================================================
    // Reservation Register Update
    // =========================================================================
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            resv_valid <= 1'b0;
            resv_addr  <= 32'h0;
        end else if (enable) begin
            // Gate updates on 'enable' to prevent spurious state changes during
            // MMU stalls. Without this, SC.W in MEM during a multi-cycle stall
            // (e.g., D-bit PTW walk) would clear the reservation on the first
            // cycle but then report failure on subsequent cycles because
            // resv_valid=0. When the stall clears, the pipeline captures the
            // stale failure result, causing SC.W to always fail when stalled.
            if (clear_reservation) begin
                // External clear (context switch, etc.)
                resv_valid <= 1'b0;
            end else if (is_lr) begin
                // LR sets reservation
                resv_valid <= 1'b1;
                resv_addr  <= addr;
            end else if (is_sc) begin
                // SC always clears reservation (success or failure)
                resv_valid <= 1'b0;
            end
            // Note: In a multi-core system, stores to the reserved address
            // from other cores would also clear the reservation. For single-core,
            // this isn't needed, but could be added for correctness:
            // else if (external_store && store_addr == resv_addr) begin
            //     resv_valid <= 1'b0;
            // end
        end
    end

    // =========================================================================
    // Initialization
    // =========================================================================
    
    `ifdef SIMULATION
    initial begin
        resv_valid = 1'b0;
        resv_addr  = 32'h0;
    end
    `endif

endmodule