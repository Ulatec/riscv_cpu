`include "../src/definitions.v"

module csr_file(
    input clk,
    input rst,
    //Read
    input [11:0] csr_addr,
    output reg [31:0] csr_rdata,
    //Write
    input csr_write,
    input [11:0] csr_waddr,
    input [31:0] csr_wdata, 
    // Trap handling
    input trap_taken,
    input [31:0] trap_cause,
    input [31:0] trap_pc,
    //counters from CPU
    input [31:0] cycle_count,
    input retire_inst,
    input mret_taken,
    // Interrupt inputs
    input         timer_irq,       // Timer interrupt pending from CLINT
    input         external_irq,    // External interrupt (for future use)
    input         software_irq,    // Software interrupt (for future use)
    
    // Interrupt output (active when interrupt should be taken)
    output        interrupt_pending,  // An enabled interrupt is pending
    output [31:0] interrupt_cause    // Cause code for pending interrupt
);

//Hardware Info Registers
reg[31:0] mvendorid; // 0xF11 - 0 (non-commercial)
reg[31:0] marchid;   // 0xF12 - Architecture
reg[31:0] mimpid;    // 0xF13 - Implementation
reg[31:0] mhartid;   // 0xF14 - Hardware Thread

// Trap Setup Registers
reg[31:0] mstatus;  
reg[31:0] misa;     // 0x301 - ISA and Extensions
reg[31:0] mie;      // 0x304 - Machine interrupt enable
reg[31:0] mtvec;    // 0x305 - Machine trap-handler base address

// Trap Handling Registers
reg[31:0] mscratch; // 0x340 - Scratch register
reg[31:0] mepc;     // 0x341 - Exception counter
reg[31:0] mcause;   // 0x342 - Trap cause
reg[31:0] mtval;    // 0x343 - Value of bad address or instruction
reg[31:0] mip;      // 0x344 - Interrupt pending

// Counters/Timers
reg[63:0] cycle_counter;    // 0xC00/0xC80 - Cycle counter
reg[63:0] instr_ret_counter;// 0xC02/0xC82 - Instructions retired counter

// mip reflects actual interrupt sources
// MTIP (bit 7) comes from CLINT timer_irq
// MSIP (bit 3) and MEIP (bit 11) for future use
wire [31:0] mip_actual;
localparam MIE_BIT  = 3;   // Machine Interrupt Enable
localparam MPIE_BIT = 7;   // Machine Previous Interrupt Enable
localparam MPP_LO   = 11;  // Machine Previous Privilege (low bit)
localparam MPP_HI   = 12;  // Machine Previous Privilege (high bit)
localparam MSIP_BIT = 3;   // Machine Software Interrupt
localparam MTIP_BIT = 7;   // Machine Timer Interrupt  
localparam MEIP_BIT = 11;  // Machine External Interrupt
assign mip_actual[MSIP_BIT] = software_irq;
assign mip_actual[MTIP_BIT] = timer_irq;
assign mip_actual[MEIP_BIT] = external_irq;
// All other bits from software-written mip
assign mip_actual[2:0]   = mip[2:0];
assign mip_actual[6:4]   = mip[6:4];
assign mip_actual[10:8]  = mip[10:8];
assign mip_actual[31:12] = mip[31:12];
localparam MCAUSE_MSI = 32'h80000003;  // Machine Software Interrupt
localparam MCAUSE_MTI = 32'h80000007;  // Machine Timer Interrupt
localparam MCAUSE_MEI = 32'h8000000B;  // Machine External Interrupt
wire msi_pending = mip_actual[MSIP_BIT] & mie[MSIP_BIT] & mstatus[MIE_BIT];
wire mti_pending = mip_actual[MTIP_BIT] & mie[MTIP_BIT] & mstatus[MIE_BIT];
wire mei_pending = mip_actual[MEIP_BIT] & mie[MEIP_BIT] & mstatus[MIE_BIT];
assign interrupt_pending = msi_pending | mti_pending | mei_pending;    
assign interrupt_cause = mei_pending ? MCAUSE_MEI :
                             msi_pending ? MCAUSE_MSI :
                             mti_pending ? MCAUSE_MTI :
                             32'h0;
//Initialize CSRs
initial begin
   mvendorid = 32'h0;
   marchid = 32'h0; 
   mimpid = 32'h0;
   mhartid = 32'h0;
   mstatus = 32'h0;
   //[31:30] = 01, [25:0] bit 8 set (I-extension)
   misa = 32'h40000100;

   //Trap Setup
   mie = 32'h0;
   mtvec = 32'h0;
   mscratch = 32'h0;
   mepc = 32'h0;
   mcause = 32'h0;
   mtval = 32'h0;
   mip = 32'h0;
   //Counters
   cycle_counter = 64'h0;
   instr_ret_counter = 64'h0;
end

// ========== UNIFIED CSR UPDATE BLOCK ==========
// Handles counters, trap handling, and CSR writes in priority order
always @(posedge clk or posedge rst) begin
    if (rst) begin
        // Reset counters
        cycle_counter <= 64'h0;
        instr_ret_counter <= 64'h0;
        
        // Reset all writable CSRs
        mstatus <= 32'h0;
        mie <= 32'h0;
        mtvec <= 32'h0;
        mscratch <= 32'h0;
        mepc <= 32'h0;
        mcause <= 32'h0;
        mtval <= 32'h0;
        mip <= 32'h0;
    end else begin
        // Increment cycle counter every clock (always happens)
        cycle_counter <= cycle_counter + 1;
        
        // Increment instruction counter when instruction retires (always happens when retire_inst)
        if (retire_inst) begin
            instr_ret_counter <= instr_ret_counter + 1;
        end
        
        // ===== TRAP HANDLING HAS PRIORITY OVER CSR WRITES =====
        if (trap_taken) begin
            mepc    <= trap_pc;      // Save PC of trapping instruction
            mcause  <= trap_cause;   // Save trap cause
            mstatus[7] <= mstatus[3];  // MPIE <= MIE
            mstatus[3] <= 1'b0;        // MIE <= 0
            mstatus[12:11] <= 2'b11;   // MPP <= M-mode (3)
            // Note: Other CSRs (mtvec, etc.) are NOT modified by trap
        end
        else if (mret_taken) begin
            // Restore MIE from MPIE
            mstatus[3] <= mstatus[7];  // MIE <= MPIE
            mstatus[7] <= 1'b1;        // MPIE <= 1
            mstatus[12:11] <= 2'b00;   // MPP <= U-mode (or lowest)
        end
        // ===== NORMAL CSR WRITES (only if no trap this cycle) =====
        else if (csr_write) begin
            case (csr_waddr)
                // Machine Trap Setup
                12'h300: mstatus <= csr_wdata & 32'h00001888;  // Only MIE, MPIE, MPP bits writable
                12'h304: mie <= csr_wdata & 32'h00000888;      // Only MEIE, MTIE, MSIE writable
                12'h305: mtvec <= csr_wdata;                   // Full 32-bit writable
                
                // Machine Trap Handling
                12'h340: mscratch <= csr_wdata;  // Scratch register
                12'h341: mepc <= csr_wdata;      // Exception PC
                12'h342: mcause <= csr_wdata;    // Trap cause
                12'h343: mtval <= csr_wdata;     // Trap value
                12'h344: mip <= csr_wdata & 32'h00000888;  // Only MEIP, MTIP, MSIP writable
                
                // Note: Counters are read-only from software perspective
                // Machine info registers are read-only
                // misa is read-only
                
                default: ; // Ignore writes to read-only or unimplemented CSRs
            endcase
        end
    end
end

// ========== CSR READ LOGIC (Combinational) ==========
always @(*) begin
    case (csr_addr)
        // Machine Information Registers
        12'hF11: csr_rdata = mvendorid;
        12'hF12: csr_rdata = marchid;
        12'hF13: csr_rdata = mimpid;
        12'hF14: csr_rdata = mhartid;
        
        // Machine Trap Setup
        12'h300: csr_rdata = mstatus;
        12'h301: csr_rdata = misa;
        12'h304: csr_rdata = mie;
        12'h305: csr_rdata = mtvec;
        
        // Machine Trap Handling
        12'h340: csr_rdata = mscratch;
        12'h341: csr_rdata = mepc;
        12'h342: csr_rdata = mcause;
        12'h343: csr_rdata = mtval;
        12'h344: csr_rdata = mip;
        
        // Machine Counter/Timers (lower 32 bits)
        12'hC00: csr_rdata = cycle_counter[31:0];     // cycle
        12'hC01: csr_rdata = cycle_counter[31:0];     // time (same as cycle for now)
        12'hC02: csr_rdata = instr_ret_counter[31:0];   // instret
        
        // Machine Counter/Timers (upper 32 bits for RV32)
        12'hC80: csr_rdata = cycle_counter[63:32];    // cycleh
        12'hC81: csr_rdata = cycle_counter[63:32];    // timeh
        12'hC82: csr_rdata = instr_ret_counter[63:32];  // instreth
        
        default: csr_rdata = 32'h0;  // Return 0 for unimplemented CSRs
    endcase
end

endmodule