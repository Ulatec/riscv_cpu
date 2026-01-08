// PLIC - Platform-Level Interrupt Controller
// Implements RISC-V PLIC specification for external interrupts
//
// Features:
// - 32 interrupt sources (source 0 reserved, 1-31 usable)
// - 7 priority levels (0=disabled, 1-7 active, 7 highest)
// - Priority threshold (only IRQs with priority > threshold fire)
// - Claim/complete handshake mechanism
//
// Memory Map:
//   0x0C000004 - 0x0C00007C: priority[1..31] (32-bit each, only [2:0] used)
//   0x0C001000:              pending[31:0]   (read-only, 1 bit per source)
//   0x0C002000:              enable[31:0]    (read/write, 1 bit per source)
//   0x0C200000:              threshold       (32-bit, only [2:0] used)
//   0x0C200004:              claim/complete  (read=claim, write=complete)

module plic (
    input         clk,
    input         rst,
    
    // Memory interface
    input  [31:0] addr,
    input  [31:0] wdata,
    input  [3:0]  wstrb,
    input         read_en,
    output reg [31:0] rdata,
    output reg    addr_valid,
    
    // Interrupt sources (directly active level inputs from peripherals)
    input  [31:0] irq_sources,    // Directly from peripheral IRQ lines
    
    // Interrupt output to CPU
    output        external_irq    // Active when claimable interrupt exists
);

    // ============================================================
    // Parameters
    // ============================================================
    localparam NUM_SOURCES = 32;     // Source 0 reserved, 1-31 usable
    localparam PRIORITY_BITS = 3;    // 0-7 priority levels
    
    // ============================================================
    // Registers
    // ============================================================
    
    // Priority for each source (0 = disabled, 1-7 = active priority)
    reg [PRIORITY_BITS-1:0] priority_reg [1:NUM_SOURCES-1];
    
    // Pending bits - set by hardware when IRQ fires, cleared by claim
    reg [NUM_SOURCES-1:0] pending;
    
    // Enable bits - software controlled
    reg [NUM_SOURCES-1:0] enable;
    
    // Priority threshold - only IRQs with priority > threshold can fire
    reg [PRIORITY_BITS-1:0] threshold;
    
    // Currently claimed interrupt (0 = none)
    reg [4:0] claimed_id;
    reg       claim_active;
    
    // ============================================================
    // Address Decoding
    // ============================================================
    
    // PLIC region: 0x0C000000 - 0x0CFFFFFF
    wire plic_region = (addr[31:24] == 8'h0C);
    
    // Priority registers: 0x0C000004 - 0x0C00007C (source 1-31)
    wire sel_priority = plic_region && (addr[23:12] == 12'h000) && 
                        (addr[11:2] >= 10'd1) && (addr[11:2] <= 10'd31);
    wire [4:0] priority_idx = addr[6:2];  // Source index from address
    
    // Pending register: 0x0C001000
    wire sel_pending = plic_region && (addr[23:0] == 24'h001000);
    
    // Enable register: 0x0C002000
    wire sel_enable = plic_region && (addr[23:0] == 24'h002000);
    
    // Threshold register: 0x0C200000
    wire sel_threshold = plic_region && (addr[23:0] == 24'h200000);
    
    // Claim/Complete register: 0x0C200004
    wire sel_claim = plic_region && (addr[23:0] == 24'h200004);
    
    // Address valid
    always @(*) begin
        addr_valid = sel_priority | sel_pending | sel_enable | 
                     sel_threshold | sel_claim;
    end
    
    // ============================================================
    // Interrupt Detection - Gateway Logic
    // ============================================================
    
    // Track previous state of IRQ sources to detect edges
    reg [NUM_SOURCES-1:0] irq_sources_prev;
    
    // Edge detection: rising edge sets pending
    wire [NUM_SOURCES-1:0] irq_rising_edge = irq_sources & ~irq_sources_prev;
    
    // ============================================================
    // Priority Comparison - Find highest priority pending & enabled
    // ============================================================
    
    // Effective pending: pending AND enabled AND (priority > threshold)
    reg [NUM_SOURCES-1:0] can_fire;
    reg [4:0] highest_id;
    reg [PRIORITY_BITS-1:0] highest_priority;
    
    integer i;
    always @(*) begin
        highest_id = 5'd0;
        highest_priority = 3'd0;
        
        for (i = 1; i < NUM_SOURCES; i = i + 1) begin
            can_fire[i] = pending[i] && enable[i] && 
                          (priority_reg[i] > threshold) &&
                          !(claim_active && (claimed_id == i));
            
            // Find highest priority (in case of tie, lower ID wins)
            if (can_fire[i] && (priority_reg[i] > highest_priority)) begin
                highest_priority = priority_reg[i];
                highest_id = i[4:0];
            end
        end
        can_fire[0] = 1'b0;  // Source 0 is reserved
    end
    
    // External IRQ output - active when there's a claimable interrupt
    assign external_irq = (highest_id != 5'd0);
    
    // ============================================================
    // Register Write Logic
    // ============================================================
    
    integer j;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // Reset all priorities to 0 (disabled)
            for (j = 1; j < NUM_SOURCES; j = j + 1) begin
                priority_reg[j] <= 3'd0;
            end
            pending <= 32'b0;
            enable <= 32'b0;
            threshold <= 3'd0;
            claimed_id <= 5'd0;
            claim_active <= 1'b0;
            irq_sources_prev <= 32'b0;
        end else begin
            // ===== Pending bit updates =====
            // Set pending on rising edge of IRQ source
            for (j = 1; j < NUM_SOURCES; j = j + 1) begin
                if (irq_rising_edge[j]) begin
                    pending[j] <= 1'b1;
                end
            end
            
            // ===== Priority register writes =====
            if (sel_priority && (wstrb != 4'b0)) begin
                if (priority_idx >= 1 && priority_idx <= 31) begin
                    priority_reg[priority_idx] <= wdata[PRIORITY_BITS-1:0];
                end
            end
            
            // ===== Enable register writes =====
            if (sel_enable && (wstrb != 4'b0)) begin
                enable <= wdata;
            end
            
            // ===== Threshold register writes =====
            if (sel_threshold && (wstrb != 4'b0)) begin
                threshold <= wdata[PRIORITY_BITS-1:0];
            end
            
            // ===== Claim (read) handling =====
            if (sel_claim && read_en && !claim_active && (highest_id != 5'd0)) begin
                // Claim the highest priority interrupt
                claimed_id <= highest_id;
                claim_active <= 1'b1;
                pending[highest_id] <= 1'b0;  // Clear pending on claim
            end
            
            // ===== Complete (write) handling =====
            if (sel_claim && (wstrb != 4'b0)) begin
                // Complete the interrupt
                if (wdata[4:0] == claimed_id && claim_active) begin
                    claim_active <= 1'b0;
                    claimed_id <= 5'd0;
                end
                // Note: Writing wrong ID is ignored (per spec)
            end
            
            // ===== Update previous IRQ state for edge detection =====
            irq_sources_prev <= irq_sources;
        end
    end
    
    // ============================================================
    // Register Read Logic
    // ============================================================
    
    always @(*) begin
        rdata = 32'h0;
        
        if (read_en) begin
            if (sel_priority) begin
                if (priority_idx >= 1 && priority_idx <= 31) begin
                    rdata = {29'b0, priority_reg[priority_idx]};
                end
            end
            else if (sel_pending) begin
                rdata = pending;
            end
            else if (sel_enable) begin
                rdata = enable;
            end
            else if (sel_threshold) begin
                rdata = {29'b0, threshold};
            end
            else if (sel_claim) begin
                // Return highest priority pending interrupt ID (or 0 if none)
                rdata = {27'b0, highest_id};
            end
        end
    end
    
    // ============================================================
    // Initialization (for simulation)
    // ============================================================
    
    initial begin
        for (j = 1; j < NUM_SOURCES; j = j + 1) begin
            priority_reg[j] = 3'd0;
        end
        pending = 32'b0;
        enable = 32'b0;
        threshold = 3'd0;
        claimed_id = 5'd0;
        claim_active = 1'b0;
        irq_sources_prev = 32'b0;
    end

endmodule