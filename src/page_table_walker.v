// =============================================================================
// Sv32 Page Table Walker
// =============================================================================
// Hardware state machine that walks the two-level Sv32 page table on TLB miss.
//
// Walk process:
// 1. Read PTE from level 1 (using satp.PPN and VPN[1])
// 2. If leaf PTE (R|W|X != 0), it's a 4MB superpage - done
// 3. If pointer PTE, read level 0 (using PTE.PPN and VPN[0])
// 4. Check permissions and return translation or fault
// 5. If A/D bits need updating, write back PTE with updated bits (Svadu)
//
// Page faults generated:
// - Instruction page fault (cause 12)
// - Load page fault (cause 13)
// - Store/AMO page fault (cause 15)
// =============================================================================

module page_table_walker (
    input         clk,
    input         rst,

    // Walk request interface
    input         walk_request,     // Start a page table walk
    input  [31:0] vaddr,            // Virtual address to translate
    input  [1:0]  access_type,      // 00=fetch, 01=load, 10=store
    input  [1:0]  priv_mode,        // Current privilege mode
    output reg    walk_done,        // Walk complete
    output reg    walk_fault,       // Page fault occurred
    output reg [3:0] fault_cause,   // Fault cause code

    // Translation result
    output reg [21:0] result_ppn,   // Physical page number
    output reg [9:0]  result_flags, // PTE flags
    output reg        result_superpage, // Is superpage?

    // satp CSR input
    input  [31:0] satp,             // satp CSR value

    // Permission control bits
    input         mstatus_sum,      // Supervisor User Memory access
    input         mstatus_mxr,      // Make eXecutable Readable

    // Memory interface (directly to physical memory)
    output reg        mem_req,      // Memory request
    output reg [33:0] mem_addr,     // Physical address (34-bit)
    output reg        mem_write,    // Memory write (for A/D bit updates)
    output reg [31:0] mem_wdata,    // Memory write data
    input      [31:0] mem_rdata,    // Memory read data
    input             mem_ready     // Memory ready
);

    // =========================================================================
    // Access Type Encoding
    // =========================================================================
    localparam ACCESS_FETCH = 2'b00;
    localparam ACCESS_LOAD  = 2'b01;
    localparam ACCESS_STORE = 2'b10;

    // Privilege levels
    localparam PRIV_U = 2'b00;
    localparam PRIV_S = 2'b01;
    localparam PRIV_M = 2'b11;

    // Fault causes
    localparam FAULT_INST_PAGE  = 4'd12;
    localparam FAULT_LOAD_PAGE  = 4'd13;
    localparam FAULT_STORE_PAGE = 4'd15;

    // =========================================================================
    // satp Decoding
    // =========================================================================
    wire        satp_mode = satp[31];           // 0=Bare, 1=Sv32
    wire [8:0]  satp_asid = satp[30:22];        // ASID
    wire [21:0] satp_ppn  = satp[21:0];         // Root page table PPN

    // =========================================================================
    // Virtual Address Decoding
    // =========================================================================
    wire [9:0]  vpn1 = vaddr[31:22];
    wire [9:0]  vpn0 = vaddr[21:12];
    wire [11:0] offset = vaddr[11:0];

    // =========================================================================
    // PTE Decoding
    // =========================================================================
    wire [21:0] pte_ppn  = mem_rdata[31:10];
    wire [1:0]  pte_rsw  = mem_rdata[9:8];
    wire        pte_d    = mem_rdata[7];  // Dirty
    wire        pte_a    = mem_rdata[6];  // Accessed
    wire        pte_g    = mem_rdata[5];  // Global
    wire        pte_u    = mem_rdata[4];  // User
    wire        pte_x    = mem_rdata[3];  // Execute
    wire        pte_w    = mem_rdata[2];  // Write
    wire        pte_r    = mem_rdata[1];  // Read
    wire        pte_v    = mem_rdata[0];  // Valid

    // Is this a leaf PTE? (has R, W, or X permission)
    wire pte_is_leaf = pte_r || pte_x;  // Note: W without R is reserved

    // Is this a valid pointer PTE?
    wire pte_is_pointer = pte_v && !pte_is_leaf;

    // =========================================================================
    // Permission Checking (without A/D bits - hardware manages A/D via Svadu)
    // =========================================================================
    wire perm_ok;

    // Check based on access type
    wire fetch_ok = pte_x;
    wire load_ok  = pte_r || (mstatus_mxr && pte_x);
    wire store_ok = pte_w;

    // Check user/supervisor access
    // U=1 page: accessible in U-mode, accessible in S-mode only if sstatus.SUM=1
    // U=0 page: not accessible in U-mode, accessible in S-mode
    wire priv_ok = (priv_mode == PRIV_M) ||
                   (priv_mode == PRIV_S && (!pte_u || mstatus_sum)) ||
                   (priv_mode == PRIV_U && pte_u);

    // Base permission check (A/D bits handled by hardware)
    assign perm_ok = pte_v && priv_ok &&
                     ((access_type == ACCESS_FETCH && fetch_ok) ||
                      (access_type == ACCESS_LOAD  && load_ok) ||
                      (access_type == ACCESS_STORE && store_ok));

    // Check if A/D bits need hardware update
    wire needs_ad_update = !pte_a || (access_type == ACCESS_STORE && !pte_d);

    // =========================================================================
    // State Machine
    // =========================================================================
    localparam IDLE           = 4'd0;
    localparam LEVEL1_REQ     = 4'd1;
    localparam LEVEL1_WAIT    = 4'd2;
    localparam LEVEL0_REQ     = 4'd3;
    localparam LEVEL0_WAIT    = 4'd4;
    localparam DONE           = 4'd5;
    localparam FAULT          = 4'd6;
    localparam AD_UPDATE_REQ  = 4'd7;
    localparam AD_UPDATE_WAIT = 4'd8;

    reg [3:0] state;
    reg [3:0] next_state;

    // Saved values during walk
    reg [31:0] saved_vaddr;
    reg [1:0]  saved_access_type;
    reg [1:0]  saved_priv_mode;
    reg [21:0] level1_ppn;       // PPN from level 1 PTE (for level 0 lookup)
    reg [33:0] saved_pte_addr;   // Address of leaf PTE (for A/D write-back)
    reg [31:0] saved_pte_data;   // Updated PTE data (with A/D bits set)

    // State machine - combinational
    always @(*) begin
        next_state = state;

        case (state)
            IDLE: begin
                if (walk_request)
                    next_state = LEVEL1_REQ;
            end

            LEVEL1_REQ: begin
                next_state = LEVEL1_WAIT;
            end

            LEVEL1_WAIT: begin
                if (mem_ready) begin
                    if (!pte_v) begin
                        // Invalid PTE - page fault
                        next_state = FAULT;
                    end else if (pte_is_leaf) begin
                        // Leaf PTE at level 1 = superpage
                        // Check for misaligned superpage (PPN[9:0] must be 0)
                        if (pte_ppn[9:0] != 10'd0) begin
                            next_state = FAULT;  // Misaligned superpage
                        end else if (perm_ok) begin
                            next_state = needs_ad_update ? AD_UPDATE_REQ : DONE;
                        end else begin
                            next_state = FAULT;
                        end
                    end else begin
                        // Pointer PTE - continue to level 0
                        next_state = LEVEL0_REQ;
                    end
                end
            end

            LEVEL0_REQ: begin
                next_state = LEVEL0_WAIT;
            end

            LEVEL0_WAIT: begin
                if (mem_ready) begin
                    if (!pte_v || !pte_is_leaf) begin
                        // Invalid or not a leaf at level 0 - fault
                        next_state = FAULT;
                    end else if (perm_ok) begin
                        next_state = needs_ad_update ? AD_UPDATE_REQ : DONE;
                    end else begin
                        next_state = FAULT;
                    end
                end
            end

            AD_UPDATE_REQ: begin
                next_state = AD_UPDATE_WAIT;
            end

            AD_UPDATE_WAIT: begin
                if (mem_ready)
                    next_state = DONE;
            end

            DONE: begin
                next_state = IDLE;
            end

            FAULT: begin
                next_state = IDLE;
            end

            default: next_state = IDLE;
        endcase
    end

    // State machine - sequential
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= IDLE;
            saved_vaddr <= 32'd0;
            saved_access_type <= 2'd0;
            saved_priv_mode <= PRIV_M;
            level1_ppn <= 22'd0;
            saved_pte_addr <= 34'd0;
            saved_pte_data <= 32'd0;

            walk_done <= 1'b0;
            walk_fault <= 1'b0;
            fault_cause <= 4'd0;
            result_ppn <= 22'd0;
            result_flags <= 10'd0;
            result_superpage <= 1'b0;
            mem_req <= 1'b0;
            mem_addr <= 34'd0;
            mem_write <= 1'b0;
            mem_wdata <= 32'd0;
        end else begin
            state <= next_state;

            // Default outputs
            walk_done <= 1'b0;
            walk_fault <= 1'b0;
            mem_req <= 1'b0;
            mem_write <= 1'b0;

            case (state)
                IDLE: begin
                    if (walk_request) begin
                        saved_vaddr <= vaddr;
                        saved_access_type <= access_type;
                        saved_priv_mode <= priv_mode;
                    end
                end

                LEVEL1_REQ: begin
                    // Calculate level 1 PTE address: satp.PPN * 4096 + VPN[1] * 4
                    mem_req <= 1'b1;
                    mem_addr <= {satp_ppn, 12'd0} + {22'd0, saved_vaddr[31:22], 2'b00};
                    // Save PTE address for potential A/D write-back
                    saved_pte_addr <= {satp_ppn, 12'd0} + {22'd0, saved_vaddr[31:22], 2'b00};
                end

                LEVEL1_WAIT: begin
                    if (mem_ready) begin
                        // Save PPN for potential level 0 lookup
                        level1_ppn <= pte_ppn;

                        if (pte_v && pte_is_leaf && pte_ppn[9:0] == 10'd0 && perm_ok) begin
                            // Valid superpage
                            result_ppn <= pte_ppn;
                            result_superpage <= 1'b1;
                            if (needs_ad_update) begin
                                // Prepare updated PTE with A (and D for stores) bits
                                saved_pte_data <= mem_rdata |
                                    ((access_type == ACCESS_STORE) ? 32'h000000C0 : 32'h00000040);
                                result_flags <= mem_rdata[9:0] |
                                    ((access_type == ACCESS_STORE) ? 10'h0C0 : 10'h040);
                            end else begin
                                result_flags <= mem_rdata[9:0];
                            end
                        end
                    end
                end

                LEVEL0_REQ: begin
                    // Calculate level 0 PTE address: level1_ppn * 4096 + VPN[0] * 4
                    mem_req <= 1'b1;
                    mem_addr <= {level1_ppn, 12'd0} + {22'd0, saved_vaddr[21:12], 2'b00};
                    // Save PTE address for potential A/D write-back
                    saved_pte_addr <= {level1_ppn, 12'd0} + {22'd0, saved_vaddr[21:12], 2'b00};
                end

                LEVEL0_WAIT: begin
                    if (mem_ready && pte_v && pte_is_leaf && perm_ok) begin
                        result_ppn <= pte_ppn;
                        result_superpage <= 1'b0;
                        if (needs_ad_update) begin
                            saved_pte_data <= mem_rdata |
                                ((access_type == ACCESS_STORE) ? 32'h000000C0 : 32'h00000040);
                            result_flags <= mem_rdata[9:0] |
                                ((access_type == ACCESS_STORE) ? 10'h0C0 : 10'h040);
                        end else begin
                            result_flags <= mem_rdata[9:0];
                        end
                    end
                end

                AD_UPDATE_REQ: begin
                    // Write updated PTE with A/D bits to memory
                    mem_req <= 1'b1;
                    mem_write <= 1'b1;
                    mem_addr <= saved_pte_addr;
                    mem_wdata <= saved_pte_data;
                end

                AD_UPDATE_WAIT: begin
                    // Write complete, proceed to DONE on next cycle
                end

                DONE: begin
                    walk_done <= 1'b1;
                    walk_fault <= 1'b0;
                end

                FAULT: begin
                    walk_done <= 1'b1;
                    walk_fault <= 1'b1;
                    // Set appropriate fault cause
                    case (saved_access_type)
                        ACCESS_FETCH: fault_cause <= FAULT_INST_PAGE;
                        ACCESS_LOAD:  fault_cause <= FAULT_LOAD_PAGE;
                        ACCESS_STORE: fault_cause <= FAULT_STORE_PAGE;
                        default:      fault_cause <= FAULT_LOAD_PAGE;
                    endcase
                end
            endcase
        end
    end

    // =========================================================================
    // Initialization
    // =========================================================================
    `ifdef SIMULATION
    initial begin
        state = IDLE;
        saved_vaddr = 32'd0;
        saved_access_type = 2'd0;
        saved_priv_mode = PRIV_M;
        level1_ppn = 22'd0;
        saved_pte_addr = 34'd0;
        saved_pte_data = 32'd0;
        walk_done = 1'b0;
        walk_fault = 1'b0;
        fault_cause = 4'd0;
        result_ppn = 22'd0;
        result_flags = 10'd0;
        result_superpage = 1'b0;
        mem_req = 1'b0;
        mem_addr = 34'd0;
        mem_write = 1'b0;
        mem_wdata = 32'd0;
    end
    `endif

endmodule
