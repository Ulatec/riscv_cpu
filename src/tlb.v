// =============================================================================
// Sv32 Translation Lookaside Buffer (TLB)
// =============================================================================
// Caches virtual-to-physical address translations to avoid page table walks.
// Fully associative, 16 entries.
//
// Features:
// - ASID support for process isolation
// - Superpage (4MB) support
// - Global page support (shared across ASIDs)
// - LRU replacement policy (simplified)
// =============================================================================

module tlb #(
    parameter NUM_ENTRIES = 16
) (
    input         clk,
    input         rst,

    // Lookup interface (primary - used for data access)
    input  [31:0] vaddr,          // Virtual address to translate
    input  [8:0]  asid,           // Address Space ID
    input         lookup_valid,   // Lookup request valid
    output        hit,            // TLB hit
    output [33:0] paddr,          // Physical address (34-bit for Sv32)
    output [9:0]  pte_flags,      // PTE flags (D,A,G,U,X,W,R,V + 2 RSW)

    // Second lookup interface (for instruction fetch - independent)
    input  [31:0] ifetch_vaddr,   // Instruction fetch virtual address
    input         ifetch_lookup_valid, // Instruction fetch lookup request
    output        ifetch_hit,     // Instruction fetch TLB hit
    output [33:0] ifetch_paddr,   // Instruction fetch physical address
    output [9:0]  ifetch_pte_flags, // Instruction fetch PTE flags

    // Refill interface (from page table walker)
    input         refill_valid,   // Refill request
    input  [31:0] refill_vaddr,   // Virtual address for refill
    input  [8:0]  refill_asid,    // ASID for refill
    input  [21:0] refill_ppn,     // Physical page number
    input  [9:0]  refill_flags,   // PTE flags
    input         refill_superpage, // Is this a 4MB superpage?
    
    // Invalidation interface
    input         sfence_vma,     // SFENCE.VMA instruction
    input         sfence_asid_valid, // Invalidate specific ASID
    input  [8:0]  sfence_asid,    // ASID to invalidate
    input         sfence_vaddr_valid, // Invalidate specific address
    input  [31:0] sfence_vaddr    // Address to invalidate
);

    // =========================================================================
    // TLB Entry Structure
    // =========================================================================
    // Each entry stores:
    // - Valid bit
    // - VPN (Virtual Page Number) - 20 bits for regular, 10 bits used for superpage
    // - ASID (9 bits)
    // - PPN (Physical Page Number) - 22 bits
    // - Flags (10 bits: RSW, D, A, G, U, X, W, R, V)
    // - Superpage flag
    
    reg [NUM_ENTRIES-1:0] entry_valid;
    reg [19:0]            entry_vpn     [NUM_ENTRIES-1:0];
    reg [8:0]             entry_asid    [NUM_ENTRIES-1:0];
    reg [21:0]            entry_ppn     [NUM_ENTRIES-1:0];
    reg [9:0]             entry_flags   [NUM_ENTRIES-1:0];
    reg [NUM_ENTRIES-1:0] entry_superpage;
    reg [NUM_ENTRIES-1:0] entry_global;  // Global bit cached separately
    
    // LRU tracking (simplified - just track most recently used)
    reg [3:0] lru_counter [NUM_ENTRIES-1:0];
    reg [3:0] lru_max;
    
    // =========================================================================
    // VPN Extraction (Primary - data access)
    // =========================================================================
    wire [9:0]  vpn1 = vaddr[31:22];  // First level (4MB granularity)
    wire [9:0]  vpn0 = vaddr[21:12];  // Second level (4KB granularity)
    wire [19:0] vpn  = vaddr[31:12];  // Full VPN
    wire [11:0] page_offset = vaddr[11:0];

    // VPN Extraction (Instruction fetch)
    wire [9:0]  ifetch_vpn1 = ifetch_vaddr[31:22];
    wire [9:0]  ifetch_vpn0 = ifetch_vaddr[21:12];
    wire [19:0] ifetch_vpn  = ifetch_vaddr[31:12];
    wire [11:0] ifetch_page_offset = ifetch_vaddr[11:0];
    
    // =========================================================================
    // Lookup Logic
    // =========================================================================
    reg [NUM_ENTRIES-1:0] match;
    reg [3:0] hit_index;
    reg       hit_found;
    
    integer i;
    
    // Match detection (combinational)
    always @(*) begin
        hit_found = 1'b0;
        hit_index = 4'd0;
        
        for (i = 0; i < NUM_ENTRIES; i = i + 1) begin
            // Check if entry is valid
            if (entry_valid[i]) begin
                // Check ASID match (or global page)
                if (entry_global[i] || (entry_asid[i] == asid)) begin
                    // Check VPN match
                    if (entry_superpage[i]) begin
                        // Superpage: only compare VPN[1] (upper 10 bits)
                        if (entry_vpn[i][19:10] == vpn1) begin
                            match[i] = 1'b1;
                            if (!hit_found) begin
                                hit_found = 1'b1;
                                hit_index = i[3:0];
                            end
                        end else begin
                            match[i] = 1'b0;
                        end
                    end else begin
                        // Regular page: compare full VPN
                        if (entry_vpn[i] == vpn) begin
                            match[i] = 1'b1;
                            if (!hit_found) begin
                                hit_found = 1'b1;
                                hit_index = i[3:0];
                            end
                        end else begin
                            match[i] = 1'b0;
                        end
                    end
                end else begin
                    match[i] = 1'b0;
                end
            end else begin
                match[i] = 1'b0;
            end
        end
    end
    
    // Output signals (primary lookup)
    assign hit = lookup_valid && hit_found;

    // Physical address construction
    wire [21:0] hit_ppn = entry_ppn[hit_index];
    wire        hit_super = entry_superpage[hit_index];

    // For superpage: PPN[21:10] from TLB, PPN[9:0] from VA's VPN[0]
    // For regular:   PPN[21:0] from TLB
    wire [21:0] final_ppn = hit_super ? {hit_ppn[21:10], vpn0} : hit_ppn;

    assign paddr = {final_ppn, page_offset};
    assign pte_flags = entry_flags[hit_index];

    // =========================================================================
    // Second Lookup Logic (Instruction Fetch)
    // =========================================================================
    reg [NUM_ENTRIES-1:0] ifetch_match;
    reg [3:0] ifetch_hit_index;
    reg       ifetch_hit_found;

    always @(*) begin
        ifetch_hit_found = 1'b0;
        ifetch_hit_index = 4'd0;

        for (i = 0; i < NUM_ENTRIES; i = i + 1) begin
            if (entry_valid[i]) begin
                if (entry_global[i] || (entry_asid[i] == asid)) begin
                    if (entry_superpage[i]) begin
                        if (entry_vpn[i][19:10] == ifetch_vpn1) begin
                            ifetch_match[i] = 1'b1;
                            if (!ifetch_hit_found) begin
                                ifetch_hit_found = 1'b1;
                                ifetch_hit_index = i[3:0];
                            end
                        end else begin
                            ifetch_match[i] = 1'b0;
                        end
                    end else begin
                        if (entry_vpn[i] == ifetch_vpn) begin
                            ifetch_match[i] = 1'b1;
                            if (!ifetch_hit_found) begin
                                ifetch_hit_found = 1'b1;
                                ifetch_hit_index = i[3:0];
                            end
                        end else begin
                            ifetch_match[i] = 1'b0;
                        end
                    end
                end else begin
                    ifetch_match[i] = 1'b0;
                end
            end else begin
                ifetch_match[i] = 1'b0;
            end
        end
    end

    // Output signals (instruction fetch lookup)
    assign ifetch_hit = ifetch_lookup_valid && ifetch_hit_found;

    wire [21:0] ifetch_hit_ppn = entry_ppn[ifetch_hit_index];
    wire        ifetch_hit_super = entry_superpage[ifetch_hit_index];
    wire [21:0] ifetch_final_ppn = ifetch_hit_super ? {ifetch_hit_ppn[21:10], ifetch_vpn0} : ifetch_hit_ppn;

    assign ifetch_paddr = {ifetch_final_ppn, ifetch_page_offset};
    assign ifetch_pte_flags = entry_flags[ifetch_hit_index];
    
    // =========================================================================
    // Refill Logic
    // =========================================================================
    reg [3:0] refill_index;
    reg       found_invalid;
    reg       found_existing;

    // Find replacement entry (existing match > invalid > LRU)
    // Checking for existing entries prevents stale TLB entries (e.g., D=0)
    // from persisting after PTW updates A/D bits and refills.
    always @(*) begin
        found_existing = 1'b0;
        found_invalid = 1'b0;
        refill_index = 4'd0;

        // First, check if an entry already exists for this VPN (replace it)
        for (i = 0; i < NUM_ENTRIES; i = i + 1) begin
            if (entry_valid[i] && !found_existing) begin
                if (entry_global[i] || (entry_asid[i] == refill_asid)) begin
                    if (entry_superpage[i] && refill_superpage) begin
                        if (entry_vpn[i][19:10] == refill_vaddr[31:22]) begin
                            found_existing = 1'b1;
                            refill_index = i[3:0];
                        end
                    end else if (!entry_superpage[i] && !refill_superpage) begin
                        if (entry_vpn[i] == refill_vaddr[31:12]) begin
                            found_existing = 1'b1;
                            refill_index = i[3:0];
                        end
                    end
                end
            end
        end

        // If no existing entry, look for an invalid entry
        if (!found_existing) begin
            for (i = 0; i < NUM_ENTRIES; i = i + 1) begin
                if (!entry_valid[i] && !found_invalid) begin
                    found_invalid = 1'b1;
                    refill_index = i[3:0];
                end
            end

            // If all valid, use LRU (find entry with lowest counter)
            if (!found_invalid) begin
                refill_index = 4'd0;
                for (i = 1; i < NUM_ENTRIES; i = i + 1) begin
                    if (lru_counter[i] < lru_counter[refill_index]) begin
                        refill_index = i[3:0];
                    end
                end
            end
        end
    end
    
    // =========================================================================
    // SFENCE.VMA Invalidation Logic
    // =========================================================================
    wire [NUM_ENTRIES-1:0] invalidate_mask;
    
    genvar g;
    generate
        for (g = 0; g < NUM_ENTRIES; g = g + 1) begin : gen_invalidate
            wire asid_match = !sfence_asid_valid || (entry_asid[g] == sfence_asid) || entry_global[g];
            wire vaddr_match;
            
            // Address match depends on superpage
            assign vaddr_match = !sfence_vaddr_valid || 
                (entry_superpage[g] ? (entry_vpn[g][19:10] == sfence_vaddr[31:22]) :
                                      (entry_vpn[g] == sfence_vaddr[31:12]));
            
            assign invalidate_mask[g] = sfence_vma && asid_match && vaddr_match;
        end
    endgenerate
    
    // =========================================================================
    // Sequential Logic
    // =========================================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            entry_valid <= {NUM_ENTRIES{1'b0}};
            entry_superpage <= {NUM_ENTRIES{1'b0}};
            entry_global <= {NUM_ENTRIES{1'b0}};
            lru_max <= 4'd0;
            
            for (i = 0; i < NUM_ENTRIES; i = i + 1) begin
                entry_vpn[i] <= 20'd0;
                entry_asid[i] <= 9'd0;
                entry_ppn[i] <= 22'd0;
                entry_flags[i] <= 10'd0;
                lru_counter[i] <= 4'd0;
            end
        end else begin
            // Handle SFENCE.VMA invalidation
            if (sfence_vma) begin
                entry_valid <= entry_valid & ~invalidate_mask;
            end
            
            // Handle refill
            if (refill_valid && !sfence_vma) begin
                entry_valid[refill_index] <= 1'b1;
                entry_vpn[refill_index] <= refill_vaddr[31:12];
                entry_asid[refill_index] <= refill_asid;
                entry_ppn[refill_index] <= refill_ppn;
                entry_flags[refill_index] <= refill_flags;
                entry_superpage[refill_index] <= refill_superpage;
                entry_global[refill_index] <= refill_flags[5]; // G bit
                
                // Update LRU
                lru_max <= lru_max + 1;
                lru_counter[refill_index] <= lru_max + 1;
            end
            
            // Update LRU on hit
            if (hit && lookup_valid && !refill_valid) begin
                lru_max <= lru_max + 1;
                lru_counter[hit_index] <= lru_max + 1;
            end
        end
    end
    
    // =========================================================================
    // Initialization
    // =========================================================================
    `ifdef SIMULATION
    initial begin
        entry_valid = {NUM_ENTRIES{1'b0}};
        entry_superpage = {NUM_ENTRIES{1'b0}};
        entry_global = {NUM_ENTRIES{1'b0}};
        lru_max = 4'd0;

        for (i = 0; i < NUM_ENTRIES; i = i + 1) begin
            entry_vpn[i] = 20'd0;
            entry_asid[i] = 9'd0;
            entry_ppn[i] = 22'd0;
            entry_flags[i] = 10'd0;
            lru_counter[i] = 4'd0;
        end
    end
    `endif

endmodule