// =============================================================================
// Sv32 Memory Management Unit (MMU)
// =============================================================================
// Top-level MMU module integrating TLB and Page Table Walker.
//
// Features:
// - Sv32 virtual memory support (32-bit VA -> 34-bit PA)
// - Separate instruction and data TLBs (optional, currently unified)
// - Hardware page table walker
// - SFENCE.VMA support for TLB invalidation
// - Bypass mode when satp.MODE=0 (Bare)
//
// Interface:
// - CPU side: virtual addresses
// - Memory side: physical addresses
// =============================================================================

`include "../src/tlb.v"
`include "../src/page_table_walker.v"

module mmu (
    input         clk,
    input         rst,
    
    // =========================================================================
    // CPU Interface (Virtual Addresses)
    // =========================================================================
    // Instruction fetch
    input  [31:0] ifetch_vaddr,     // Virtual address for instruction fetch
    input         ifetch_req,       // Fetch request
    output [33:0] ifetch_paddr,     // Physical address
    output        ifetch_ready,     // Translation complete
    output        ifetch_fault,     // Page fault
    
    // Data access (load/store)
    input  [31:0] data_vaddr,       // Virtual address for data access
    input         data_req,         // Data request
    input         data_write,       // 1=store, 0=load
    output [33:0] data_paddr,       // Physical address
    output        data_ready,       // Translation complete
    output        data_fault,       // Page fault
    
    // =========================================================================
    // CSR Interface
    // =========================================================================
    input  [31:0] satp,             // satp CSR
    input  [1:0]  priv_mode,        // Current privilege mode (for instruction fetch)
    input  [1:0]  data_priv_mode,   // Effective privilege for data access (MPRV-aware)
    input         mstatus_mxr,      // Make eXecutable Readable
    input         mstatus_sum,      // Supervisor User Memory access
    
    // =========================================================================
    // SFENCE.VMA Interface
    // =========================================================================
    input         sfence_vma,       // SFENCE.VMA instruction
    input         sfence_asid_valid,
    input  [8:0]  sfence_asid,
    input         sfence_vaddr_valid,
    input  [31:0] sfence_vaddr,
    
    // =========================================================================
    // Page Table Walker Memory Interface
    // =========================================================================
    output        ptw_mem_req,      // Memory request from PTW
    output [33:0] ptw_mem_addr,     // Physical address
    output        ptw_mem_write,    // Memory write from PTW (A/D updates)
    output [31:0] ptw_mem_wdata,    // Memory write data from PTW
    input  [31:0] ptw_mem_rdata,    // Read data
    input         ptw_mem_ready,    // Memory ready
    
    // =========================================================================
    // Fault Information
    // =========================================================================
    output [3:0]  fault_cause,      // Page fault cause code
    output [31:0] fault_vaddr       // Faulting virtual address
);

    // =========================================================================
    // satp Decoding
    // =========================================================================
    wire        vm_enabled = satp[31];  // MODE: 0=Bare, 1=Sv32
    wire [8:0]  current_asid = satp[30:22];
    
    // =========================================================================
    // Privilege Levels
    // =========================================================================
    localparam PRIV_U = 2'b00;
    localparam PRIV_S = 2'b01;
    localparam PRIV_M = 2'b11;
    
    // VM is only active in S-mode and U-mode (M-mode always uses physical)
    // Instruction fetch always uses the actual privilege level
    wire vm_active_ifetch = vm_enabled && (priv_mode != PRIV_M);
    // Data access uses MPRV-aware effective privilege (allows M-mode to
    // access memory as S/U-mode when MPRV=1, needed for OpenSBI)
    wire vm_active_data = vm_enabled && (data_priv_mode != PRIV_M);
    // Combined for backward compatibility (used where both paths share logic)
    wire vm_active = vm_active_ifetch;
    
    // =========================================================================
    // Access Type for TLB/PTW
    // =========================================================================
    localparam ACCESS_FETCH = 2'b00;
    localparam ACCESS_LOAD  = 2'b01;
    localparam ACCESS_STORE = 2'b10;
    
    // =========================================================================
    // Notes on Arbitration
    // =========================================================================
    // The TLB has two independent lookup ports - one for ifetch, one for data.
    // Each can hit independently, allowing both accesses to proceed in parallel.
    // When a TLB miss occurs, the PTW handles one walk at a time (data priority).
    // Ready signals are asserted only for the access that triggered the PTW.

    // =========================================================================
    // TLB Instance
    // =========================================================================
    // Primary lookup (for data access)
    wire        tlb_data_hit;
    wire [33:0] tlb_data_paddr;
    wire [9:0]  tlb_data_flags;

    // Second lookup (for instruction fetch - independent)
    wire        tlb_ifetch_hit;
    wire [33:0] tlb_ifetch_paddr;
    wire [9:0]  tlb_ifetch_flags;

    // TLB refill signals (from PTW)
    wire        tlb_refill_valid;
    wire [31:0] tlb_refill_vaddr;
    wire [21:0] tlb_refill_ppn;
    wire [9:0]  tlb_refill_flags;
    wire        tlb_refill_superpage;

    tlb #(
        .NUM_ENTRIES(16)
    ) tlb_inst (
        .clk(clk),
        .rst(rst),

        // Primary lookup (data access)
        .vaddr(data_vaddr),
        .asid(current_asid),
        .lookup_valid(data_req && vm_active_data),
        .hit(tlb_data_hit),
        .paddr(tlb_data_paddr),
        .pte_flags(tlb_data_flags),

        // Second lookup (instruction fetch)
        .ifetch_vaddr(ifetch_vaddr),
        .ifetch_lookup_valid(ifetch_req && vm_active_ifetch),
        .ifetch_hit(tlb_ifetch_hit),
        .ifetch_paddr(tlb_ifetch_paddr),
        .ifetch_pte_flags(tlb_ifetch_flags),

        // Refill
        .refill_valid(tlb_refill_valid),
        .refill_vaddr(tlb_refill_vaddr),
        .refill_asid(current_asid),
        .refill_ppn(tlb_refill_ppn),
        .refill_flags(tlb_refill_flags),
        .refill_superpage(tlb_refill_superpage),

        // SFENCE.VMA
        .sfence_vma(sfence_vma),
        .sfence_asid_valid(sfence_asid_valid),
        .sfence_asid(sfence_asid),
        .sfence_vaddr_valid(sfence_vaddr_valid),
        .sfence_vaddr(sfence_vaddr)
    );
    
    // =========================================================================
    // Permission Check on TLB Hit - Instruction Fetch
    // =========================================================================
    wire ifetch_pte_v = tlb_ifetch_flags[0];
    wire ifetch_pte_x = tlb_ifetch_flags[3];
    wire ifetch_pte_u = tlb_ifetch_flags[4];
    wire ifetch_pte_a = tlb_ifetch_flags[6];

    wire ifetch_fetch_ok = ifetch_pte_x;
    wire ifetch_priv_ok = (priv_mode == PRIV_M) ||
                          (priv_mode == PRIV_S && (!ifetch_pte_u || mstatus_sum)) ||
                          (priv_mode == PRIV_U && ifetch_pte_u);
    wire ifetch_perm_ok = ifetch_pte_v && ifetch_pte_a && ifetch_priv_ok && ifetch_fetch_ok;

    // Detect A/D miss: base permissions OK but A bit not set (hardware will update)
    wire ifetch_base_perm_ok = ifetch_pte_v && ifetch_priv_ok && ifetch_fetch_ok;
    wire ifetch_ad_miss = tlb_ifetch_hit && ifetch_base_perm_ok && !ifetch_pte_a;

    wire ifetch_tlb_perm_fault = tlb_ifetch_hit && !ifetch_perm_ok && !ifetch_ad_miss;

    // =========================================================================
    // Permission Check on TLB Hit - Data Access
    // =========================================================================
    wire data_pte_v = tlb_data_flags[0];
    wire data_pte_r = tlb_data_flags[1];
    wire data_pte_w = tlb_data_flags[2];
    wire data_pte_x = tlb_data_flags[3];
    wire data_pte_u = tlb_data_flags[4];
    wire data_pte_a = tlb_data_flags[6];
    wire data_pte_d = tlb_data_flags[7];

    wire data_load_ok  = data_pte_r || (mstatus_mxr && data_pte_x);
    wire data_store_ok = data_pte_w && data_pte_d;
    wire data_priv_ok = (data_priv_mode == PRIV_M) ||
                        (data_priv_mode == PRIV_S && (!data_pte_u || mstatus_sum)) ||
                        (data_priv_mode == PRIV_U && data_pte_u);
    wire data_perm_ok = data_pte_v && data_pte_a && data_priv_ok &&
                        (data_write ? data_store_ok : data_load_ok);

    // Detect A/D miss: base permissions OK but A or D bits need updating (hardware will update)
    wire data_base_perm_ok = data_pte_v && data_priv_ok &&
                              (data_write ? data_pte_w : data_load_ok);
    wire data_ad_miss = tlb_data_hit && data_base_perm_ok &&
                        (!data_pte_a || (data_write && !data_pte_d));

    wire data_tlb_perm_fault = tlb_data_hit && !data_perm_ok && !data_ad_miss;
    
    // =========================================================================
    // Page Table Walker Instance
    // =========================================================================
    wire        ptw_request;
    wire        ptw_done;
    wire        ptw_fault;
    wire [3:0]  ptw_fault_cause;
    wire [21:0] ptw_result_ppn;
    wire [9:0]  ptw_result_flags;
    wire        ptw_result_superpage;

    // Determine which access needs PTW (priority: data > ifetch)
    // A/D misses are treated as TLB misses - PTW will set A/D bits in memory
    wire data_needs_ptw = data_req && vm_active_data && (!tlb_data_hit || data_ad_miss) && !data_tlb_perm_fault;
    wire ifetch_needs_ptw = ifetch_req && vm_active_ifetch && (!tlb_ifetch_hit || ifetch_ad_miss) && !ifetch_tlb_perm_fault;

    // PTW request when either needs it and PTW is available
    // Priority: data over ifetch (data access can stall pipeline)
    wire ptw_for_data_next = data_needs_ptw;
    wire [31:0] ptw_vaddr = ptw_for_data_next ? data_vaddr : ifetch_vaddr;
    wire [1:0]  ptw_access_type = ptw_for_data_next ? (data_write ? ACCESS_STORE : ACCESS_LOAD) : ACCESS_FETCH;

    // Start PTW on TLB miss (when VM is active, and no latched fault for that address)
    wire ifetch_fault_latched = ptw_fault_latched && !ptw_for_data && (fault_vaddr_latched == ifetch_vaddr);
    wire data_fault_latched = ptw_fault_latched && ptw_for_data && (fault_vaddr_latched == data_vaddr);
    wire data_ptw_eligible = data_needs_ptw && !data_fault_latched;
    assign ptw_request = !ptw_busy &&
                         (data_ptw_eligible ||
                          (ifetch_needs_ptw && !data_ptw_eligible && !ifetch_fault_latched));

    // PTW privilege: use data_priv_mode for data walks, priv_mode for ifetch walks
    wire [1:0] ptw_eff_priv = ptw_for_data ? data_priv_mode : priv_mode;

    page_table_walker ptw_inst (
        .clk(clk),
        .rst(rst),

        // Walk request
        .walk_request(ptw_request),
        .vaddr(ptw_vaddr),
        .access_type(ptw_access_type),
        .priv_mode(ptw_eff_priv),
        .walk_done(ptw_done),
        .walk_fault(ptw_fault),
        .fault_cause(ptw_fault_cause),

        // Result
        .result_ppn(ptw_result_ppn),
        .result_flags(ptw_result_flags),
        .result_superpage(ptw_result_superpage),

        // satp
        .satp(satp),

        // Permission control bits
        .mstatus_sum(mstatus_sum),
        .mstatus_mxr(mstatus_mxr),

        // Memory interface
        .mem_req(ptw_mem_req),
        .mem_addr(ptw_mem_addr),
        .mem_write(ptw_mem_write),
        .mem_wdata(ptw_mem_wdata),
        .mem_rdata(ptw_mem_rdata),
        .mem_ready(ptw_mem_ready)
    );
    
    // PTW busy tracking
    reg ptw_busy;
    reg ptw_fault_latched;
    reg [3:0] ptw_fault_cause_latched;
    reg [31:0] fault_vaddr_latched;  // Track which VA faulted
    reg ptw_for_data;  // Track if current PTW is for data (vs ifetch)

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            ptw_busy <= 1'b0;
            ptw_fault_latched <= 1'b0;
            ptw_fault_cause_latched <= 4'd0;
            fault_vaddr_latched <= 32'h0;
            ptw_for_data <= 1'b0;
        end else if (ptw_request) begin
            ptw_busy <= 1'b1;
            ptw_for_data <= ptw_for_data_next;  // Remember which access triggered PTW
        end else if (ptw_done) begin
            ptw_busy <= 1'b0;
            if (ptw_fault) begin
                ptw_fault_latched <= 1'b1;
                ptw_fault_cause_latched <= ptw_fault_cause;
                fault_vaddr_latched <= ptw_inst.saved_vaddr;  // Remember which VA faulted
            end
        end else begin
            // Clear fault latch when the faulting address is no longer being accessed
            if (ptw_fault_latched) begin
                if (ptw_for_data && (!data_req || (data_vaddr != fault_vaddr_latched))) begin
                    ptw_fault_latched <= 1'b0;
                end else if (!ptw_for_data && (!ifetch_req || (ifetch_vaddr != fault_vaddr_latched))) begin
                    ptw_fault_latched <= 1'b0;
                end
            end
        end
    end

    // TLB refill on successful PTW
    assign tlb_refill_valid = ptw_done && !ptw_fault;
    assign tlb_refill_vaddr = ptw_inst.saved_vaddr;
    assign tlb_refill_ppn = ptw_result_ppn;
    assign tlb_refill_flags = ptw_result_flags;
    assign tlb_refill_superpage = ptw_result_superpage;

    // =========================================================================
    // Output Generation
    // =========================================================================

    // Bare-mode paths (zero-extend VA to 34-bit PA)
    wire [33:0] ifetch_bare_paddr = {2'b00, ifetch_vaddr};
    wire [33:0] data_bare_paddr = {2'b00, data_vaddr};

    // PTW result physical addresses
    // For superpages: PPN[21:10] from PTW, VPN[0] + offset from VA (22 bits)
    // For regular pages: full PPN from PTW, page offset from VA (12 bits)
    wire [33:0] ifetch_ptw_paddr = ptw_result_superpage ?
        {ptw_result_ppn[21:10], ifetch_vaddr[21:0]} :
        {ptw_result_ppn, ifetch_vaddr[11:0]};
    wire [33:0] data_ptw_paddr = ptw_result_superpage ?
        {ptw_result_ppn[21:10], data_vaddr[21:0]} :
        {ptw_result_ppn, data_vaddr[11:0]};

    // -------------------------------------------------------------------------
    // Instruction Fetch Outputs
    // -------------------------------------------------------------------------
    // ifetch can proceed if:
    // - VM not active (bare mode): always ready
    // - TLB hit with valid permissions: ready with TLB address
    // - PTW completed for ifetch (!ptw_for_data): ready with PTW result
    wire ifetch_tlb_ready = tlb_ifetch_hit && ifetch_perm_ok;
    wire ifetch_ptw_ready = ptw_done && !ptw_for_data && !ptw_fault;
    wire ifetch_has_fault = ifetch_tlb_perm_fault ||
                            (ptw_done && !ptw_for_data && ptw_fault) ||
                            (ifetch_fault_latched);

    assign ifetch_paddr = !vm_active_ifetch ? ifetch_bare_paddr :
                          ifetch_tlb_ready ? tlb_ifetch_paddr :
                          ifetch_ptw_paddr;
    assign ifetch_ready = !vm_active_ifetch || ifetch_tlb_ready || ifetch_ptw_ready;
    assign ifetch_fault = vm_active_ifetch && ifetch_req && ifetch_has_fault;

    // -------------------------------------------------------------------------
    // Data Access Outputs
    // -------------------------------------------------------------------------
    // data can proceed if:
    // - VM not active (bare mode): always ready
    // - TLB hit with valid permissions: ready with TLB address
    // - PTW completed for data (ptw_for_data): ready with PTW result
    wire data_tlb_ready = tlb_data_hit && data_perm_ok;
    wire data_ptw_ready = ptw_done && ptw_for_data && !ptw_fault;
    wire data_has_fault = data_tlb_perm_fault ||
                          (ptw_done && ptw_for_data && ptw_fault) ||
                          (data_fault_latched);

    assign data_paddr = !vm_active_data ? data_bare_paddr :
                        data_tlb_ready ? tlb_data_paddr :
                        data_ptw_paddr;
    assign data_ready = !vm_active_data || data_tlb_ready || data_ptw_ready;
    assign data_fault = vm_active_data && data_req && data_has_fault;

    // -------------------------------------------------------------------------
    // Fault Information
    // -------------------------------------------------------------------------
    // Determine which fault to report (priority: data > ifetch)
    wire reporting_data_fault = data_fault;
    wire reporting_ifetch_fault = ifetch_fault && !data_fault;

    assign fault_cause = data_tlb_perm_fault ? (data_write ? 4'd15 : 4'd13) :
                         ifetch_tlb_perm_fault ? 4'd12 :
                         (ptw_fault ? ptw_fault_cause : ptw_fault_cause_latched);
    assign fault_vaddr = reporting_data_fault ? data_vaddr : ifetch_vaddr;
    
    // =========================================================================
    // Initialization
    // =========================================================================
    `ifdef SIMULATION
    initial begin
        ptw_busy = 1'b0;
        ptw_fault_latched = 1'b0;
        ptw_fault_cause_latched = 4'd0;
        fault_vaddr_latched = 32'h0;
        ptw_for_data = 1'b0;
    end
    `endif

endmodule
