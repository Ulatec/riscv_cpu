// CSR Register File with S-Mode Support
// Implements Machine and Supervisor mode CSRs for RISC-V
//
// Privilege Levels:
//   2'b00 = U-mode (User)
//   2'b01 = S-mode (Supervisor)
//   2'b11 = M-mode (Machine)
//
// New features:
//   - Privilege level tracking
//   - Trap delegation (medeleg/mideleg)
//   - S-mode CSRs (sstatus, stvec, sepc, scause, etc.)
//   - SRET support

module csr_file (
    input         clk,
    input         rst,

    // CSR read port (EX stage)
    input  [11:0] csr_addr,
    output reg [31:0] csr_rdata,

    // CSR write port (WB stage)
    input         csr_write,
    input  [11:0] csr_waddr,
    input  [31:0] csr_wdata,

    // Trap interface (directly from MEM stage)
    input         trap_taken,      // Combined exception OR interrupt
    input  [31:0] trap_cause,      // Exception/interrupt cause
    input  [31:0] trap_pc,         // PC to save (varies for exc vs int)
    input  [31:0] trap_val,        // Trap value (bad address, etc.)

    // Interrupt inputs
    input         timer_irq,       // From CLINT
    input         external_irq,    // From PLIC
    input         software_irq,    // Software interrupt

    // Interrupt outputs
    output        interrupt_pending,  // Enabled interrupt waiting
    output [31:0] interrupt_cause,    // Cause code if interrupt taken

    // Return instructions
    input         mret_taken,      // MRET executing in MEM stage
    input         sret_taken,      // SRET executing in MEM stage

    // Current privilege level
    output reg [1:0] priv_level,   // Current privilege (00=U, 01=S, 11=M)

    // Trap destination outputs
    output [31:0] trap_vector,     // Where to jump on trap (mtvec or stvec)
    output [31:0] trap_return_pc,  // Where to return (mepc or sepc)
    output        trap_to_s_mode,  // Trap goes to S-mode (vs M-mode)

    // MMU-related outputs
    output [31:0] satp_out,        // SATP register for MMU
    output        mstatus_mxr,     // Make eXecutable Readable
    output        mstatus_sum,     // Supervisor User Memory access

    // Performance counters
    input  [31:0] cycle_count,
    input         retire_inst,

    // Timer from CLINT (for TIME CSR)
    input  [63:0] mtime           // Current mtime value from CLINT
);

    // =========================================================================
    // Privilege Level Encoding
    // =========================================================================
    localparam PRIV_U = 2'b00;
    localparam PRIV_S = 2'b01;
    localparam PRIV_M = 2'b11;

    // =========================================================================
    // CSR Address Definitions
    // =========================================================================

    // Machine-level CSRs
    localparam MSTATUS    = 12'h300;
    localparam MISA       = 12'h301;
    localparam MEDELEG    = 12'h302;
    localparam MIDELEG    = 12'h303;
    localparam MIE        = 12'h304;
    localparam MTVEC      = 12'h305;
    localparam MCOUNTEREN = 12'h306;
    localparam MSCRATCH   = 12'h340;
    localparam MEPC       = 12'h341;
    localparam MCAUSE     = 12'h342;
    localparam MTVAL      = 12'h343;
    localparam MIP        = 12'h344;

    // Machine info
    localparam MVENDORID  = 12'hF11;
    localparam MARCHID    = 12'hF12;
    localparam MIMPID     = 12'hF13;
    localparam MHARTID    = 12'hF14;

    // Supervisor-level CSRs
    localparam SSTATUS    = 12'h100;
    localparam SIE        = 12'h104;
    localparam STVEC      = 12'h105;
    localparam SCOUNTEREN = 12'h106;
    localparam SSCRATCH   = 12'h140;
    localparam SEPC       = 12'h141;
    localparam SCAUSE     = 12'h142;
    localparam STVAL      = 12'h143;
    localparam SIP        = 12'h144;
    localparam SATP       = 12'h180;

    // Performance counters
    localparam CYCLE      = 12'hC00;
    localparam TIME       = 12'hC01;
    localparam INSTRET    = 12'hC02;
    localparam CYCLEH     = 12'hC80;
    localparam TIMEH      = 12'hC81;
    localparam INSTRETH   = 12'hC82;

    // =========================================================================
    // Machine-Mode Registers
    // =========================================================================

    // mstatus fields (directly stored)
    reg        mstatus_mie;   // Machine interrupt enable
    reg        mstatus_mpie;  // Previous MIE
    reg [1:0]  mstatus_mpp;   // Previous privilege (for MRET)
    reg        mstatus_sie;   // Supervisor interrupt enable
    reg        mstatus_spie;  // Previous SIE
    reg        mstatus_spp;   // Previous privilege for S-mode (1 bit: 0=U, 1=S)
    reg        mstatus_mxr_reg;  // Make eXecutable Readable
    reg        mstatus_sum_reg;  // Supervisor User Memory access
    reg        mstatus_mprv;     // Modify PRiVilege

    // Compose full mstatus
    wire [31:0] mstatus = {
        1'b0,              // SD (31) - no dirty state
        8'b0,              // Reserved (30:23)
        1'b0,              // TSR (22) - trap SRET
        1'b0,              // TW (21) - timeout wait
        1'b0,              // TVM (20) - trap virtual memory
        mstatus_mxr_reg,   // MXR (19) - make executable readable
        mstatus_sum_reg,   // SUM (18) - supervisor user memory
        mstatus_mprv,      // MPRV (17) - modify privilege
        2'b0,              // XS (16:15) - extension state
        2'b0,              // FS (14:13) - FPU state
        mstatus_mpp,       // MPP (12:11) - previous privilege for M-mode
        2'b0,              // Reserved (10:9)
        mstatus_spp,       // SPP (8) - previous privilege for S-mode
        mstatus_mpie,      // MPIE (7)
        1'b0,              // Reserved (6)
        mstatus_spie,      // SPIE (5)
        1'b0,              // Reserved (4)
        mstatus_mie,       // MIE (3)
        1'b0,              // Reserved (2)
        mstatus_sie,       // SIE (1)
        1'b0               // Reserved (0)
    };

    // sstatus is a restricted view of mstatus
    wire [31:0] sstatus = {
        1'b0,              // SD (31)
        11'b0,             // Reserved (30:20)
        mstatus_mxr_reg,   // MXR (19) - visible in S-mode
        mstatus_sum_reg,   // SUM (18) - visible in S-mode
        1'b0,              // Reserved (17)
        2'b0,              // XS (16:15)
        2'b0,              // FS (14:13)
        4'b0,              // Reserved (12:9)
        mstatus_spp,       // SPP (8)
        1'b0,              // Reserved (7)
        1'b0,              // Reserved (6)
        mstatus_spie,      // SPIE (5)
        3'b0,              // Reserved (4:2)
        mstatus_sie,       // SIE (1)
        1'b0               // Reserved (0)
    };

    reg [31:0] mtvec;
    reg [31:0] mscratch;
    reg [31:0] mepc;
    reg [31:0] mcause;
    reg [31:0] mtval;

    // Delegation registers
    reg [31:0] medeleg;   // Exception delegation
    reg [31:0] mideleg;   // Interrupt delegation

    // Interrupt enable/pending
    reg [31:0] mie;       // Machine interrupt enable bits
    // mip is mostly read-only, directly reflects interrupt lines

    // misa - hardcoded for RV32IMA_S
    // Bits: 0=A, 8=I, 12=M, 18=S
    wire [31:0] misa = {
        2'b01,          // MXL = 32-bit (bits 31:30)
        4'b0,           // Reserved (bits 29:26)
        26'b00000001000001000100000001
        //        ^     ^   ^       ^
        //       18    12   8       0
        //        S     M   I       A
    };

    // =========================================================================
    // Supervisor-Mode Registers
    // =========================================================================

    reg [31:0] stvec;
    reg [31:0] sscratch;
    reg [31:0] sepc;
    reg [31:0] scause;
    reg [31:0] stval;
    reg [31:0] satp;       // Address translation (for MMU later)
    reg [31:0] scounteren; // Counter access for U-mode
    reg [31:0] mcounteren; // Counter access for S-mode (M-mode enables)

    // =========================================================================
    // Interrupt Logic
    // =========================================================================

    // mip - Machine Interrupt Pending (directly reflects hardware state)
    // Bits: 11=MEIP, 9=SEIP, 7=MTIP, 5=STIP, 3=MSIP, 1=SSIP
    reg        mip_seip_sw;  // Software-writable SEIP
    reg        mip_stip_sw;  // Software-writable STIP
    reg        mip_ssip;     // Software interrupt pending (S-mode)

    wire [31:0] mip = {
        20'b0,
        external_irq,    // MEIP (11) - from PLIC
        1'b0,
        mip_seip_sw,     // SEIP (9) - can be set by software or external
        1'b0,
        timer_irq,       // MTIP (7) - from CLINT
        1'b0,
        mip_stip_sw,     // STIP (5) - software only for S-mode timer
        1'b0,
        software_irq,    // MSIP (3) - from CLINT (if implemented)
        1'b0,
        mip_ssip,        // SSIP (1) - S-mode software interrupt
        1'b0
    };

    // sip is restricted view of mip (only S-mode visible bits)
    wire [31:0] sip = mip & 32'h222;  // Bits 9, 5, 1

    // sie is restricted view of mie
    wire [31:0] sie = mie & 32'h222;  // Bits 9, 5, 1

    // Determine which interrupts can fire based on privilege and enables
    // M-mode interrupts: always go to M-mode
    // S-mode interrupts: go to S-mode if delegated, else M-mode

    wire [31:0] m_interrupts = mip & mie & ~mideleg;  // Non-delegated
    wire [31:0] s_interrupts = mip & mie & mideleg;   // Delegated to S-mode

    // Can take M-mode interrupt if MIE=1 or in lower priv
    wire can_take_m_int = (priv_level != PRIV_M) || mstatus_mie;

    // Can take S-mode interrupt if in U-mode, or (S-mode and SIE=1)
    wire can_take_s_int = (priv_level == PRIV_U) ||
                          ((priv_level == PRIV_S) && mstatus_sie);

    // Final pending signals
    wire m_int_pending = (m_interrupts != 0) && can_take_m_int;
    wire s_int_pending = (s_interrupts != 0) && can_take_s_int && (priv_level != PRIV_M);

    // M-mode interrupts have higher priority
    assign interrupt_pending = m_int_pending || s_int_pending;

    // Determine interrupt cause (priority: MEI > MSI > MTI > SEI > SSI > STI)
    reg [31:0] int_cause_m;
    reg [31:0] int_cause_s;

    always @(*) begin
        // M-mode interrupt cause
        if (m_interrupts[11])      int_cause_m = 32'h8000000B;  // MEI
        else if (m_interrupts[3])  int_cause_m = 32'h80000003;  // MSI
        else if (m_interrupts[7])  int_cause_m = 32'h80000007;  // MTI
        else                       int_cause_m = 32'h0;

        // S-mode interrupt cause
        if (s_interrupts[9])       int_cause_s = 32'h80000009;  // SEI
        else if (s_interrupts[1])  int_cause_s = 32'h80000001;  // SSI
        else if (s_interrupts[5])  int_cause_s = 32'h80000005;  // STI
        else                       int_cause_s = 32'h0;
    end

    assign interrupt_cause = m_int_pending ? int_cause_m : int_cause_s;

    // =========================================================================
    // Trap Delegation Logic
    // =========================================================================

    // Determine if trap should go to S-mode
    // Trap goes to S-mode if:
    //   1. Not currently in M-mode AND
    //   2. The trap type is delegated in medeleg/mideleg

    wire is_interrupt = trap_cause[31];
    wire [4:0] trap_code = trap_cause[4:0];

    wire trap_delegated = is_interrupt ? mideleg[trap_code] : medeleg[trap_code];

    assign trap_to_s_mode = (priv_level != PRIV_M) && trap_delegated;

    // Select trap vector based on destination mode
    assign trap_vector = trap_to_s_mode ? stvec : mtvec;

    // Select return PC based on which return instruction
    assign trap_return_pc = sret_taken ? sepc : mepc;

    // MMU outputs
    assign satp_out = satp;
    assign mstatus_mxr = mstatus[19];  // MXR bit
    assign mstatus_sum = mstatus[18];  // SUM bit

    // =========================================================================
    // Performance Counters
    // =========================================================================

    reg [63:0] cycle_counter;
    reg [63:0] instret_counter;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cycle_counter <= 64'b0;
            instret_counter <= 64'b0;
        end else begin
            cycle_counter <= cycle_counter + 1;
            if (retire_inst)
                instret_counter <= instret_counter + 1;
        end
    end

    // =========================================================================
    // CSR Read Logic
    // =========================================================================

    always @(*) begin
        csr_rdata = 32'h0;

        case (csr_addr)
            // Machine-mode CSRs
            MSTATUS:    csr_rdata = mstatus;
            MISA:       csr_rdata = misa;
            MEDELEG:    csr_rdata = medeleg;
            MIDELEG:    csr_rdata = mideleg;
            MIE:        csr_rdata = mie;
            MTVEC:      csr_rdata = mtvec;
            MSCRATCH:   csr_rdata = mscratch;
            MEPC:       csr_rdata = mepc;
            MCAUSE:     csr_rdata = mcause;
            MTVAL:      csr_rdata = mtval;
            MIP:        csr_rdata = mip;
            MCOUNTEREN: csr_rdata = mcounteren;

            // Machine info (read-only)
            MVENDORID:  csr_rdata = 32'h0;
            MARCHID:    csr_rdata = 32'h0;
            MIMPID:     csr_rdata = 32'h0;
            MHARTID:    csr_rdata = 32'h0;

            // Supervisor-mode CSRs
            SSTATUS:    csr_rdata = sstatus;
            SIE:        csr_rdata = sie;
            STVEC:      csr_rdata = stvec;
            SCOUNTEREN: csr_rdata = scounteren;
            SSCRATCH:   csr_rdata = sscratch;
            SEPC:       csr_rdata = sepc;
            SCAUSE:     csr_rdata = scause;
            STVAL:      csr_rdata = stval;
            SIP:        csr_rdata = sip;
            SATP:       csr_rdata = satp;

            // Performance counters
            CYCLE:      csr_rdata = cycle_counter[31:0];
            CYCLEH:     csr_rdata = cycle_counter[63:32];
            TIME:       csr_rdata = mtime[31:0];   // From CLINT
            TIMEH:      csr_rdata = mtime[63:32];  // From CLINT
            INSTRET:    csr_rdata = instret_counter[31:0];
            INSTRETH:   csr_rdata = instret_counter[63:32];

            default:    csr_rdata = 32'h0;
        endcase
    end

    // =========================================================================
    // CSR Write and Trap Logic
    // =========================================================================

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // Machine-mode resets
            mstatus_mie  <= 1'b0;
            mstatus_mpie <= 1'b0;
            mstatus_mpp  <= PRIV_M;
            mstatus_sie  <= 1'b0;
            mstatus_spie <= 1'b0;
            mstatus_spp  <= 1'b0;
            mstatus_mxr_reg <= 1'b0;
            mstatus_sum_reg <= 1'b0;
            mstatus_mprv <= 1'b0;

            mtvec    <= 32'h0;
            mscratch <= 32'h0;
            mepc     <= 32'h0;
            mcause   <= 32'h0;
            mtval    <= 32'h0;
            mie      <= 32'h0;
            medeleg  <= 32'h0;
            mideleg  <= 32'h0;

            // Supervisor-mode resets
            stvec      <= 32'h0;
            sscratch   <= 32'h0;
            sepc       <= 32'h0;
            scause     <= 32'h0;
            stval      <= 32'h0;
            satp       <= 32'h0;
            scounteren <= 32'h0;
            mcounteren <= 32'h7;  // Enable CY, TM, IR for S-mode by default

            // Software-writable interrupt bits
            mip_seip_sw <= 1'b0;
            mip_stip_sw <= 1'b0;
            mip_ssip    <= 1'b0;

            // Start in M-mode
            priv_level <= PRIV_M;

        end else begin
            // =================================================================
            // Trap Entry (highest priority)
            // =================================================================
            if (trap_taken) begin
                if (trap_to_s_mode) begin
                    // Trap to S-mode
                    sepc   <= trap_pc;
                    scause <= trap_cause;
                    stval  <= trap_val;

                    // Save current interrupt enable and privilege
                    mstatus_spie <= mstatus_sie;
                    mstatus_spp  <= (priv_level == PRIV_S) ? 1'b1 : 1'b0;

                    // Disable interrupts
                    mstatus_sie <= 1'b0;

                    // Enter S-mode
                    priv_level <= PRIV_S;
                end else begin
                    // Trap to M-mode
                    mepc   <= trap_pc;
                    mcause <= trap_cause;
                    mtval  <= trap_val;

                    // Save current interrupt enable and privilege
                    mstatus_mpie <= mstatus_mie;
                    mstatus_mpp  <= priv_level;

                    // Disable interrupts
                    mstatus_mie <= 1'b0;

                    // Enter M-mode
                    priv_level <= PRIV_M;
                end
            end

            // =================================================================
            // MRET - Return from M-mode trap
            // =================================================================
            else if (mret_taken) begin
                // Restore interrupt enable
                mstatus_mie <= mstatus_mpie;
                mstatus_mpie <= 1'b1;

                // Return to previous privilege level
                priv_level <= mstatus_mpp;

                // Set MPP to U-mode (or S if no U-mode)
                mstatus_mpp <= PRIV_U;
            end

            // =================================================================
            // SRET - Return from S-mode trap
            // =================================================================
            else if (sret_taken) begin
                // Restore interrupt enable
                mstatus_sie <= mstatus_spie;
                mstatus_spie <= 1'b1;

                // Return to previous privilege level
                priv_level <= mstatus_spp ? PRIV_S : PRIV_U;

                // Set SPP to U-mode
                mstatus_spp <= 1'b0;
            end

            // =================================================================
            // CSR Writes (lower priority than traps/returns)
            // =================================================================
            else if (csr_write) begin
                case (csr_waddr)
                    // Machine-mode CSRs
                    MSTATUS: begin
                        mstatus_mie  <= csr_wdata[3];
                        mstatus_mpie <= csr_wdata[7];
                        mstatus_mpp  <= csr_wdata[12:11];
                        mstatus_sie  <= csr_wdata[1];
                        mstatus_spie <= csr_wdata[5];
                        mstatus_spp  <= csr_wdata[8];
                        mstatus_mxr_reg <= csr_wdata[19];
                        mstatus_sum_reg <= csr_wdata[18];
                        mstatus_mprv <= csr_wdata[17];
                    end
                    MIE:      mie <= csr_wdata & 32'hAAA;  // Only valid bits
                    MTVEC:    mtvec <= {csr_wdata[31:2], 2'b0};  // Align to 4
                    MSCRATCH: mscratch <= csr_wdata;
                    MEPC:     mepc <= {csr_wdata[31:2], 2'b0};  // Align to 4
                    MCAUSE:   mcause <= csr_wdata;
                    MTVAL:    mtval <= csr_wdata;
                    MEDELEG:  medeleg <= csr_wdata & 32'h0000B3FF;  // Valid exception bits
                    MIDELEG:  mideleg <= csr_wdata & 32'h222;       // Only S-mode ints delegatable
                    MIP: begin
                        // Only some bits are writable
                        mip_seip_sw <= csr_wdata[9];
                        mip_stip_sw <= csr_wdata[5];
                        mip_ssip    <= csr_wdata[1];
                    end
                    MCOUNTEREN: mcounteren <= csr_wdata & 32'h7;  // Only CY, TM, IR bits

                    // Supervisor-mode CSRs
                    SSTATUS: begin
                        // sstatus writes affect mstatus S-mode bits
                        mstatus_sie  <= csr_wdata[1];
                        mstatus_spie <= csr_wdata[5];
                        mstatus_spp  <= csr_wdata[8];
                        mstatus_mxr_reg <= csr_wdata[19];
                        mstatus_sum_reg <= csr_wdata[18];
                    end
                    SIE:       mie <= (mie & ~32'h222) | (csr_wdata & 32'h222);
                    STVEC:     stvec <= {csr_wdata[31:2], 2'b0};
                    SCOUNTEREN: scounteren <= csr_wdata;
                    SSCRATCH:  sscratch <= csr_wdata;
                    SEPC:      sepc <= {csr_wdata[31:2], 2'b0};
                    SCAUSE:    scause <= csr_wdata;
                    STVAL:     stval <= csr_wdata;
                    SIP: begin
                        // Only SSIP is writable via SIP
                        mip_ssip <= csr_wdata[1];
                    end
                    SATP:      satp <= csr_wdata;

                    default: ;  // Ignore writes to unknown/read-only CSRs
                endcase
            end
        end
    end

    // =========================================================================
    // Initialization (simulation only)
    // =========================================================================

    initial begin
        mstatus_mie  = 1'b0;
        mstatus_mpie = 1'b0;
        mstatus_mpp  = PRIV_M;
        mstatus_sie  = 1'b0;
        mstatus_spie = 1'b0;
        mstatus_spp  = 1'b0;
        mstatus_mxr_reg = 1'b0;
        mstatus_sum_reg = 1'b0;
        mstatus_mprv = 1'b0;

        mtvec    = 32'h0;
        mscratch = 32'h0;
        mepc     = 32'h0;
        mcause   = 32'h0;
        mtval    = 32'h0;
        mie      = 32'h0;
        medeleg  = 32'h0;
        mideleg  = 32'h0;

        stvec      = 32'h0;
        sscratch   = 32'h0;
        sepc       = 32'h0;
        scause     = 32'h0;
        stval      = 32'h0;
        satp       = 32'h0;
        scounteren = 32'h0;
        mcounteren = 32'h7;  // Enable CY, TM, IR for S-mode

        mip_seip_sw = 1'b0;
        mip_stip_sw = 1'b0;
        mip_ssip    = 1'b0;

        priv_level = PRIV_M;
    end

endmodule
