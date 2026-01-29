// CLINT - Core Local Interruptor
// Implements RISC-V standard timer functionality
//
// Memory Map (active when address in CLINT region):
//   0x0200BFF8 - mtime (low 32 bits) - read/write
//   0x0200BFFC - mtimeh (high 32 bits) - read/write  
//   0x02004000 - mtimecmp (low 32 bits) - read/write
//   0x02004004 - mtimecmph (high 32 bits) - read/write
//
// Timer interrupt fires when mtime >= mtimecmp

module clint (
    input         clk,
    input         rst,
    
    // Memory interface
    input  [31:0] addr,        // Full address from CPU
    input  [31:0] wdata,       // Write data
    input  [3:0]  wstrb,       // Write strobe (byte enables)
    input         read_en,     // Read enable
    output reg [31:0] rdata,   // Read data
    output reg    addr_valid,  // High if address is in CLINT region
    
    // Interrupt output
    output        timer_irq,   // Timer interrupt pending

    // Timer value output (for CSR TIME register)
    output [63:0] mtime_out
);

    // ============================================================
    // CLINT Registers
    // ============================================================
    
    // mtime - 64-bit free-running timer
    // Increments every clock cycle
    reg [63:0] mtime;
    
    // mtimecmp - 64-bit compare register
    // Interrupt fires when mtime >= mtimecmp
    reg [63:0] mtimecmp;
    
    // ============================================================
    // Address Decoding
    // ============================================================
    
    // CLINT region: 0x02000000 - 0x0200FFFF
    wire clint_region = (addr[31:16] == 16'h0200);
    
    // Specific register addresses
    wire sel_mtime_lo    = clint_region && (addr[15:0] == 16'hBFF8);
    wire sel_mtime_hi    = clint_region && (addr[15:0] == 16'hBFFC);
    wire sel_mtimecmp_lo = clint_region && (addr[15:0] == 16'h4000);
    wire sel_mtimecmp_hi = clint_region && (addr[15:0] == 16'h4004);
    
    // Address is valid if it's any CLINT register
    always @(*) begin
        addr_valid = sel_mtime_lo | sel_mtime_hi | sel_mtimecmp_lo | sel_mtimecmp_hi;
    end
    
    // ============================================================
    // Timer Interrupt Logic
    // ============================================================
    
    // Interrupt fires when mtime >= mtimecmp
    // Note: This is a level-sensitive signal, not edge-triggered
    // Software clears it by writing a new (larger) value to mtimecmp
    assign timer_irq = (mtime >= mtimecmp);
    assign mtime_out = mtime;
    
    // ============================================================
    // mtime Counter (always incrementing)
    // ============================================================
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            mtime <= 64'h0;
        end else begin
            // mtime always increments, but writes take precedence
            if (sel_mtime_lo && (wstrb != 4'b0)) begin
                // Write to low 32 bits
                if (wstrb[0]) mtime[7:0]   <= wdata[7:0];
                if (wstrb[1]) mtime[15:8]  <= wdata[15:8];
                if (wstrb[2]) mtime[23:16] <= wdata[23:16];
                if (wstrb[3]) mtime[31:24] <= wdata[31:24];
            end else if (sel_mtime_hi && (wstrb != 4'b0)) begin
                // Write to high 32 bits
                if (wstrb[0]) mtime[39:32] <= wdata[7:0];
                if (wstrb[1]) mtime[47:40] <= wdata[15:8];
                if (wstrb[2]) mtime[55:48] <= wdata[23:16];
                if (wstrb[3]) mtime[63:56] <= wdata[31:24];
            end else begin
                // Normal increment
                mtime <= mtime + 1;
            end
        end
    end
    
    // ============================================================
    // mtimecmp Register (software writable)
    // ============================================================
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // Initialize to max value so interrupt doesn't fire immediately
            mtimecmp <= 64'hFFFFFFFF_FFFFFFFF;
        end else begin
            if (sel_mtimecmp_lo && (wstrb != 4'b0)) begin
                // Write to low 32 bits
                if (wstrb[0]) mtimecmp[7:0]   <= wdata[7:0];
                if (wstrb[1]) mtimecmp[15:8]  <= wdata[15:8];
                if (wstrb[2]) mtimecmp[23:16] <= wdata[23:16];
                if (wstrb[3]) mtimecmp[31:24] <= wdata[31:24];
            end
            if (sel_mtimecmp_hi && (wstrb != 4'b0)) begin
                // Write to high 32 bits
                if (wstrb[0]) mtimecmp[39:32] <= wdata[7:0];
                if (wstrb[1]) mtimecmp[47:40] <= wdata[15:8];
                if (wstrb[2]) mtimecmp[55:48] <= wdata[23:16];
                if (wstrb[3]) mtimecmp[63:56] <= wdata[31:24];
            end
        end
    end
    
    // ============================================================
    // Read Logic (combinational)
    // ============================================================
    
    always @(*) begin
        rdata = 32'h0;
        if (read_en) begin
            if (sel_mtime_lo)
                rdata = mtime[31:0];
            else if (sel_mtime_hi)
                rdata = mtime[63:32];
            else if (sel_mtimecmp_lo)
                rdata = mtimecmp[31:0];
            else if (sel_mtimecmp_hi)
                rdata = mtimecmp[63:32];
        end
    end

endmodule