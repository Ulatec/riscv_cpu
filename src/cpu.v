`ifndef SYNTHESIS
`include "../src/reg_file.v"
`include "../src/ALU.v"
`include "../src/csr_file.v"
`include "../src/definitions.v"
`include "../src/control_unit.v"
`include "../src/clint.v"
`include "../src/plic.v"
`include "../src/atomic_unit.v"
`include "../src/mmu.v"
`include "../src/decompressor.v"
`include "../src/div_unit.v"
`endif

module cpu(
    input rst, clk,
    // Instruction memory (read-only)
    output wire [31:0] imem_addr,
    input  wire [31:0] imem_rdata,
    output wire        imem_rstrb,

    input [31:0] mem_rdata,
    output [31:0] mem_addr,
    output [31:0] mem_wdata,
    output mem_rstrb,
    output reg [31:0] cycle,
    output [3:0] mem_wstrb,

    // External interrupt sources for PLIC
    input [31:0] external_irq_sources
);

    // =========================================================================
    // Memory Ready Signal (for PTW)
    // =========================================================================
    wire mem_ready = 1'b1;  // Memory always ready (single-cycle memory)

    // =========================================================================
    // Debug Registers
    // =========================================================================
    reg [31:0] debug_id_instruction;
    reg [31:0] debug_ex_instruction;
    reg [31:0] debug_mem_instruction;
    reg [31:0] debug_wb_instruction;

    `ifdef SIMULATION
    initial begin
        cycle = 0;
        debug_id_instruction  = 32'h00000013;
        debug_ex_instruction  = 32'h00000013;
        debug_mem_instruction = 32'h00000013;
        debug_wb_instruction  = 32'h00000013;
        saved_half   = 16'b0;
        have_saved   = 1'b0;
        spanning_pc  = 32'b0;
    end
    `endif

    // =========================================================================
    // Program Counter
    // =========================================================================
    reg [31:0] pc_reg;

    // =========================================================================
    // RVC (Compressed Instruction) State
    // =========================================================================
    reg [15:0] saved_half;       // Saved upper half for spanning instructions
    reg        have_saved;       // Whether saved_half is valid
    reg [31:0] spanning_pc;      // PC where spanning instruction started

    // =========================================================================
    // IF/ID Pipeline Registers
    // =========================================================================
    reg [31:0] if_id_instruction;
    reg [31:0] if_id_pcplus4;
    reg [31:0] if_id_pc;         // PC of instruction in IF/ID

    // =========================================================================
    // ID/EX Pipeline Registers
    // =========================================================================
    // Data values
    reg [31:0] id_ex_pcplus4;
    reg [31:0] id_ex_pc;         // PC of instruction in ID/EX
    reg [31:0] id_ex_rs1_data;
    reg [31:0] id_ex_rs2_data;
    reg [31:0] id_ex_immediate;
    // Register addresses
    reg [4:0]  id_ex_rs1_addr;
    reg [4:0]  id_ex_rs2_addr;
    reg [4:0]  id_ex_rd_addr;
    // Control signals
    reg [4:0]  id_ex_alu_op;
    reg [1:0]  id_ex_alu_in1_src;
    reg        id_ex_alusrc;
    reg        id_ex_mem_read;
    reg        id_ex_mem_write;
    reg        id_ex_reg_write;
    reg        id_ex_mem_to_reg;
    reg        id_ex_isBtype_reg;
    reg        id_ex_isJAL;
    reg        id_ex_isJALR;
    reg [2:0]  id_ex_funct3;
    // CSR
    reg [11:0] id_ex_csr_addr;
    reg        id_ex_is_csr;
    reg        id_ex_is_ecall;
    reg        id_ex_is_ebreak;
    reg        id_ex_is_csr_imm;
    reg        id_ex_is_mret;
    reg        id_ex_is_sret;
    // Atomic
    reg        id_ex_is_lr;
    reg        id_ex_is_sc;
    reg        id_ex_is_amo;
    reg [4:0]  id_ex_amo_funct5;

    // =========================================================================
    // EX/MEM Pipeline Registers
    // =========================================================================
    reg [31:0] ex_mem_alu_result;
    reg [31:0] ex_mem_pc;          // PC of instruction in EX/MEM
    reg [31:0] ex_mem_rs1_data;    // For SFENCE.VMA vaddr
    reg [31:0] ex_mem_rs2_data;
    reg [4:0]  ex_mem_rs1_addr;    // For SFENCE.VMA vaddr_valid check
    reg [4:0]  ex_mem_rs2_addr;    // For SFENCE.VMA asid_valid check
    reg [4:0]  ex_mem_rd_addr;
    reg [2:0]  ex_mem_funct3;
    reg        ex_mem_zero_flag;
    reg        ex_mem_alu_lt;
    reg        ex_mem_alu_ltu;
    reg        ex_mem_mem_read;
    reg        ex_mem_mem_write;
    reg        ex_mem_reg_write;
    reg        ex_mem_mem_to_reg;
    reg        ex_mem_is_mret;
    reg        ex_mem_is_sret;
    reg [31:0] ex_mem_pcplus4;
    reg        ex_mem_isJAL;
    reg        ex_mem_isJALR;
    reg [31:0] ex_mem_branch_target;
    reg        ex_mem_isBtype;
    // CSR
    reg [31:0] ex_mem_csr_wdata;
    reg [11:0] ex_mem_csr_addr;
    reg        ex_mem_is_csr;
    reg        ex_mem_csr_write;
    reg [31:0] ex_mem_csr_rdata;
    reg        ex_mem_is_ecall;
    reg        ex_mem_is_ebreak;
    // Atomic
    reg        ex_mem_is_lr;
    reg        ex_mem_is_sc;
    reg        ex_mem_is_amo;
    reg [4:0]  ex_mem_amo_funct5;
    // MMU
    reg        ex_mem_is_sfence_vma;

    // =========================================================================
    // MEM/WB Pipeline Registers
    // =========================================================================
    reg [31:0] mem_wb_mem_data;
    reg [31:0] mem_wb_alu_result;
    reg [4:0]  mem_wb_rd_addr;
    reg        mem_wb_reg_write;
    reg        mem_wb_mem_to_reg;
    reg [31:0] mem_wb_pcplus4;
    reg        mem_wb_isJAL;
    reg        mem_wb_isJALR;
    // CSR
    reg [31:0] mem_wb_csr_rdata;
    reg [11:0] mem_wb_csr_addr;
    reg        mem_wb_is_csr;
    reg [31:0] mem_wb_csr_wdata;
    reg        mem_wb_csr_write;
    // Atomic
    reg [31:0] mem_wb_atomic_data;
    reg        mem_wb_is_atomic;

    // =========================================================================
    // Wires for Module Interfaces
    // =========================================================================
    wire [31:0] rs1_data;
    wire [31:0] rs2_data;
    wire        alu_lt;
    wire        alu_ltu;
    wire [31:0] alu_result;
    wire        zero_flag;
    wire [31:0] write_data_to_reg;

    // =========================================================================
    // RVC Decompressor
    // =========================================================================
    wire [31:0] decompressed_instruction;
    wire        rvc_is_compressed;
    wire        rvc_need_next_half;

    decompressor decomp_inst (
        .fetched_word(imem_rdata),
        .pc_bit1(pc_reg[1]),
        .saved_half(saved_half),
        .have_saved(have_saved),
        .instruction(decompressed_instruction),
        .is_compressed(rvc_is_compressed),
        .need_next_half(rvc_need_next_half)
    );

    // IF Stage - PC increment: +2 for compressed/spanning, +4 for normal 32-bit
    wire [31:0] pcplus_if = rvc_need_next_half  ? (pc_reg + 32'd2) :
                            (rvc_is_compressed || have_saved) ? (pc_reg + 32'd2) :
                            (pc_reg + 32'd4);
    wire [31:0] next_pc;

    // =========================================================================
    // ID Stage - Field Extraction from Instruction
    // =========================================================================
    wire [4:0] opcode_id = if_id_instruction[6:2];
    wire [4:0] rd_id     = if_id_instruction[11:7];
    wire [4:0] rs1_id    = if_id_instruction[19:15];
    wire [4:0] rs2_id    = if_id_instruction[24:20];
    wire [2:0] funct3_id = if_id_instruction[14:12];
    wire [6:0] funct7_id = if_id_instruction[31:25];
    wire [11:0] csr_addr = if_id_instruction[31:20];

    // =========================================================================
    // Immediate Generation (ID Stage)
    // =========================================================================
    wire [31:0] imm_i = {{21{if_id_instruction[31]}}, if_id_instruction[30:20]};
    wire [31:0] imm_b = {{20{if_id_instruction[31]}}, if_id_instruction[7], if_id_instruction[30:25], if_id_instruction[11:8], 1'b0};
    wire [31:0] imm_s = {{21{if_id_instruction[31]}}, if_id_instruction[30:25], if_id_instruction[11:7]};
    wire [31:0] imm_j = {{12{if_id_instruction[31]}}, if_id_instruction[19:12], if_id_instruction[20], if_id_instruction[30:21], 1'b0};
    wire [31:0] imm_u = {if_id_instruction[31], if_id_instruction[30:12], 12'h000};

    reg [31:0] immediate_id;
    always @(*) begin
        immediate_id = imm_i;  // Default I-type
        if (opcode_id == 5'b01000)      immediate_id = imm_s;  // S-type
        else if (opcode_id == 5'b11000) immediate_id = imm_b;  // B-type
        else if (opcode_id == 5'b11011) immediate_id = imm_j;  // JAL
        else if (opcode_id == 5'b01101 || opcode_id == 5'b00101) immediate_id = imm_u;  // LUI, AUIPC
        else if (opcode_id == 5'b01011) immediate_id = 32'b0;  // Atomic (addr = rs1 + 0)
    end

    // =========================================================================
    // Control Unit
    // =========================================================================
    wire [4:0] alu_op_ctrl;
    wire [1:0] ctrl_alu_in1_src;
    wire alusrc_ctrl, mem_read_ctrl, mem_write_ctrl;
    wire reg_write_ctrl, mem_to_reg_ctrl;
    wire is_branch_ctrl, is_jal_ctrl, is_jalr_ctrl;
    wire is_csr_ctrl, is_ecall_ctrl, is_ebreak_ctrl;
    wire is_mret_ctrl, is_sret_ctrl;
    wire is_sfence_vma_ctrl;
    wire is_lr_ctrl, is_sc_ctrl, is_amo_ctrl;
    wire [4:0] amo_funct5_ctrl;
    wire retire_inst;

    control_unit control_inst(
        .opcode(opcode_id),
        .funct3(funct3_id),
        .funct7(funct7_id),
        .rd(rd_id),
        .rs1(rs1_id),
        .rs2(rs2_id),
        .alu_op(alu_op_ctrl),
        .alu_in1_src(ctrl_alu_in1_src),
        .alusrc(alusrc_ctrl),
        .mem_read(mem_read_ctrl),
        .mem_write(mem_write_ctrl),
        .reg_write(reg_write_ctrl),
        .mem_to_reg(mem_to_reg_ctrl),
        .is_branch(is_branch_ctrl),
        .is_jal(is_jal_ctrl),
        .is_jalr(is_jalr_ctrl),
        .is_csr(is_csr_ctrl),
        .is_ecall(is_ecall_ctrl),
        .is_ebreak(is_ebreak_ctrl),
        .is_mret(is_mret_ctrl),
        .is_sret(is_sret_ctrl),
        .is_sfence_vma(is_sfence_vma_ctrl),
        .is_lr(is_lr_ctrl),
        .is_sc(is_sc_ctrl),
        .is_amo(is_amo_ctrl),
        .amo_funct5(amo_funct5_ctrl)
    );

    // =========================================================================
    // Hazard Detection Unit
    // =========================================================================
    // Detect load-use hazard: Load in pipeline and ID stage reads that register
    // Detect CSR-use hazard: CSR read in pipeline and ID stage reads that register
    //
    // When hazard detected:
    //   - Stall PC (don't increment)
    //   - Stall IF/ID (keep same instruction)
    //   - Insert bubble in ID/EX (NOP - clear control signals)
    //   - Let EX/MEM and MEM/WB continue normally

    // Check if ID stage instruction uses rs1 or rs2
    wire id_uses_rs1 = (rs1_id != 5'b0) && (
        mem_read_ctrl ||        // Load uses rs1
        mem_write_ctrl ||       // Store uses rs1
        !alusrc_ctrl ||         // R-type uses rs1
        alusrc_ctrl ||          // I-type uses rs1 (for ALU op)
        is_branch_ctrl ||       // Branch uses rs1
        is_jalr_ctrl ||         // JALR uses rs1
        is_csr_ctrl             // CSR may use rs1
    );

    wire id_uses_rs2 = (rs2_id != 5'b0) && (
        mem_write_ctrl ||       // Store uses rs2
        !alusrc_ctrl ||         // R-type uses rs2 (when alusrc=0, we use rs2)
        is_branch_ctrl          // Branch uses rs2
    );

    // Load-use hazard check against ID/EX stage (load in EX)
    wire load_use_hazard_ex = id_ex_mem_read && (id_ex_rd_addr != 5'b0) && (
        (id_uses_rs1 && (id_ex_rd_addr == rs1_id)) ||
        (id_uses_rs2 && (id_ex_rd_addr == rs2_id))
    );

    // Load-use hazard check against EX/MEM stage (load in MEM)
    wire load_use_hazard_mem = ex_mem_mem_read && (ex_mem_rd_addr != 5'b0) && (
        (id_uses_rs1 && (ex_mem_rd_addr == rs1_id)) ||
        (id_uses_rs2 && (ex_mem_rd_addr == rs2_id))
    );

    // Load-use hazard check against MEM/WB stage (load result being written this cycle)
    wire load_use_hazard_wb = mem_wb_mem_to_reg && mem_wb_reg_write && (mem_wb_rd_addr != 5'b0) && (
        (id_uses_rs1 && (mem_wb_rd_addr == rs1_id)) ||
        (id_uses_rs2 && (mem_wb_rd_addr == rs2_id))
    );

    // CSR-use hazard: EX stage has a CSR read, and ID stage reads the CSR destination
    wire csr_use_hazard_ex = id_ex_is_csr && (id_ex_rd_addr != 5'b0) && (
        (id_uses_rs1 && (id_ex_rd_addr == rs1_id)) ||
        (id_uses_rs2 && (id_ex_rd_addr == rs2_id))
    );

    // CSR result in MEM stage
    wire csr_use_hazard_mem = ex_mem_is_csr && (ex_mem_rd_addr != 5'b0) && (
        (id_uses_rs1 && (ex_mem_rd_addr == rs1_id)) ||
        (id_uses_rs2 && (ex_mem_rd_addr == rs2_id))
    );

    // CSR result being written in WB stage
    wire csr_use_hazard_wb = mem_wb_is_csr && mem_wb_reg_write && (mem_wb_rd_addr != 5'b0) && (
        (id_uses_rs1 && (mem_wb_rd_addr == rs1_id)) ||
        (id_uses_rs2 && (mem_wb_rd_addr == rs2_id))
    );

    // Combined hazard signal
    wire data_hazard_stall = load_use_hazard_ex || load_use_hazard_mem || load_use_hazard_wb ||
                             csr_use_hazard_ex || csr_use_hazard_mem || csr_use_hazard_wb;

    // =========================================================================
    // Forwarding Logic
    // =========================================================================
    reg [2:0] ForwardA;
    reg [2:0] ForwardB;
    reg [1:0] ForwardStore;

    wire [31:0] forward_data_mem = ex_mem_alu_result;
    wire [31:0] forward_data_wb = write_data_to_reg;

    // =========================================================================
    // CLINT (Timer)
    // =========================================================================
    wire        clint_timer_irq;
    wire [31:0] clint_rdata;
    wire        clint_addr_valid;
    wire [63:0] clint_mtime;

    clint clint_inst (
        .clk(clk),
        .rst(rst),
        .addr(ex_mem_alu_result),
        .wdata(ex_mem_rs2_data),
        .wstrb(mem_wstrb),
        .read_en(ex_mem_mem_read),
        .rdata(clint_rdata),
        .addr_valid(clint_addr_valid),
        .timer_irq(clint_timer_irq),
        .mtime_out(clint_mtime)
    );

    // =========================================================================
    // PLIC (External Interrupts)
    // =========================================================================
    wire        plic_external_irq;
    wire [31:0] plic_rdata;
    wire        plic_addr_valid;

    plic plic_inst (
        .clk(clk),
        .rst(rst),
        .addr(ex_mem_alu_result),
        .wdata(ex_mem_rs2_data),
        .wstrb(mem_wstrb),
        .read_en(ex_mem_mem_read),
        .rdata(plic_rdata),
        .addr_valid(plic_addr_valid),
        .irq_sources(external_irq_sources),
        .external_irq(plic_external_irq)
    );

    // =========================================================================
    // Atomic Unit
    // =========================================================================
    wire [31:0] atomic_mem_wdata;
    wire [31:0] atomic_rd_data;
    wire        atomic_sc_success;
    wire        atomic_do_mem_write;
    wire        atomic_reservation_valid;
    wire [31:0] atomic_reservation_addr;

    wire is_atomic_mem = ex_mem_is_lr || ex_mem_is_sc || ex_mem_is_amo;
    wire clear_atomic_reservation = take_trap;

    atomic_unit atomic_unit_inst (
        .clk(clk),
        .rst(rst),
        .amo_op(ex_mem_amo_funct5),
        .is_lr(ex_mem_is_lr),
        .is_sc(ex_mem_is_sc),
        .is_amo(ex_mem_is_amo),
        .addr(ex_mem_alu_result),
        .rs2_data(ex_mem_rs2_data),
        .mem_rdata(mem_rdata),
        .mem_wdata(atomic_mem_wdata),
        .rd_data(atomic_rd_data),
        .sc_success(atomic_sc_success),
        .do_mem_write(atomic_do_mem_write),
        .clear_reservation(clear_atomic_reservation),
        .enable(!mmu_stall),
        .reservation_valid(atomic_reservation_valid),
        .reservation_addr(atomic_reservation_addr)
    );

    // =========================================================================
    // Memory Management Unit (Sv32 MMU)
    // =========================================================================
    wire [33:0] ifetch_paddr;
    wire        ifetch_ready;
    wire        ifetch_fault;
    wire [33:0] data_paddr;
    wire        data_ready;
    wire        data_fault;
    wire        ptw_mem_req;
    wire [33:0] ptw_mem_addr;
    wire        ptw_mem_write;
    wire [31:0] ptw_mem_wdata;
    wire [31:0] ptw_mem_rdata;
    wire        ptw_mem_ready;
    wire [3:0]  mmu_fault_cause;
    wire [31:0] mmu_fault_vaddr;

    wire sfence_in_mem = ex_mem_is_sfence_vma;
    wire [8:0] sfence_asid = ex_mem_rs2_data[8:0];
    wire [31:0] sfence_vaddr_val = ex_mem_rs1_data;

    mmu mmu_inst (
        .clk(clk),
        .rst(rst),

        // Instruction fetch (from IF stage)
        .ifetch_vaddr(pc_reg),
        .ifetch_req(1'b1),
        .ifetch_paddr(ifetch_paddr),
        .ifetch_ready(ifetch_ready),
        .ifetch_fault(ifetch_fault),

        // Data access (from MEM stage)
        .data_vaddr(ex_mem_alu_result),
        .data_req(ex_mem_mem_read || ex_mem_mem_write),
        .data_write(ex_mem_mem_write),
        .data_paddr(data_paddr),
        .data_ready(data_ready),
        .data_fault(data_fault),

        // CSR interface
        .satp(satp_value),
        .priv_mode(current_priv),
        .data_priv_mode(effective_data_priv),
        .mstatus_mxr(fwd_mstatus_mxr),
        .mstatus_sum(fwd_mstatus_sum),

        // SFENCE.VMA
        .sfence_vma(sfence_in_mem),
        .sfence_asid_valid(ex_mem_rs2_addr != 5'b0),
        .sfence_asid(sfence_asid),
        .sfence_vaddr_valid(ex_mem_rs1_addr != 5'b0),
        .sfence_vaddr(sfence_vaddr_val),

        // PTW memory interface
        .ptw_mem_req(ptw_mem_req),
        .ptw_mem_addr(ptw_mem_addr),
        .ptw_mem_write(ptw_mem_write),
        .ptw_mem_wdata(ptw_mem_wdata),
        .ptw_mem_rdata(ptw_mem_rdata),
        .ptw_mem_ready(ptw_mem_ready),

        // Fault information
        .fault_cause(mmu_fault_cause),
        .fault_vaddr(mmu_fault_vaddr)
    );

    // PTW memory port sharing - PTW takes over data memory port during walks
    assign ptw_mem_rdata = mem_rdata;
    assign ptw_mem_ready = ptw_mem_req && mem_ready;

    // Page fault signals
    wire page_fault_ifetch = ifetch_fault;
    wire page_fault_data = data_fault;

    // MMU stall signals
    wire mmu_stall_ifetch = !ifetch_ready && !ifetch_fault;
    wire mmu_stall_data = (ex_mem_mem_read || ex_mem_mem_write) &&
                          !data_ready && !data_fault;
    wire mmu_stall = mmu_stall_ifetch || mmu_stall_data || div_stall;
    wire pipeline_stall = mmu_stall || data_hazard_stall;

    // =========================================================================
    // CSR File
    // =========================================================================
    wire [31:0] csr_rdata;
    wire        interrupt_pending;
    wire [31:0] interrupt_cause;
    wire [1:0]  priv_level;
    wire [31:0] trap_vector;
    wire [31:0] trap_return_pc;
    wire        trap_to_s_mode;
    wire [31:0] satp_value;
    wire [1:0]  current_priv;
    wire        mstatus_mxr;
    wire        mstatus_sum;
    wire        mstatus_mprv;
    wire [1:0]  mstatus_mpp;

    // CSR-to-MMU forwarding: when WB stage writes mstatus/sstatus, forward
    // new SUM/MXR/MPRV/MPP to MMU so MEM stage sees updated values immediately
    wire csr_fwd_active = mem_wb_csr_write &&
        (mem_wb_csr_addr == 12'h300 || mem_wb_csr_addr == 12'h100);
    wire csr_fwd_mstatus = mem_wb_csr_write && (mem_wb_csr_addr == 12'h300);
    wire fwd_mstatus_sum = csr_fwd_active ? mem_wb_csr_wdata[18] : mstatus_sum;
    wire fwd_mstatus_mxr = csr_fwd_active ? mem_wb_csr_wdata[19] : mstatus_mxr;
    // MPRV/MPP only writable via mstatus (not sstatus)
    wire fwd_mstatus_mprv = csr_fwd_mstatus ? mem_wb_csr_wdata[17] : mstatus_mprv;
    wire [1:0] fwd_mstatus_mpp = csr_fwd_mstatus ? mem_wb_csr_wdata[12:11] : mstatus_mpp;

    // MPRV: M-mode data accesses use MPP privilege when MPRV=1
    wire [1:0] effective_data_priv = (current_priv == 2'b11 && fwd_mstatus_mprv) ?
                                      fwd_mstatus_mpp : current_priv;

    // Suppress wrong-path ifetch page faults when MEM stage has a redirect
    wire ifetch_fault_suppress = take_branch_condition || ex_mem_isJAL || ex_mem_isJALR ||
                                  ex_mem_is_mret || ex_mem_is_sret || ex_mem_is_sfence_vma ||
                                  ex_mem_is_ecall || ex_mem_is_ebreak;
    wire effective_ifetch_fault = page_fault_ifetch && !ifetch_fault_suppress;

    // Exception handling - includes page faults
    wire exception_taken = ex_mem_is_ecall || ex_mem_is_ebreak ||
                           effective_ifetch_fault || page_fault_data;

    wire [31:0] exception_cause =
        ex_mem_is_ebreak ? 32'd3 :
        ex_mem_is_ecall  ? (priv_level == 2'b11 ? 32'd11 :
                            priv_level == 2'b01 ? 32'd9  : 32'd8) :
        page_fault_data && ex_mem_mem_write  ? 32'd15 :   // Store/AMO page fault
        page_fault_data && !ex_mem_mem_write ? 32'd13 :   // Load page fault
        effective_ifetch_fault               ? 32'd12 :   // Instruction page fault
        32'd0;

    // Exception PC: faulting instruction address
    // For ifetch page faults, the faulting PC is the current pc_reg
    // For data faults/ecall/ebreak, it's the instruction in MEM stage
    wire [31:0] exception_pc = effective_ifetch_fault ? pc_reg : ex_mem_pc;

    // Trap value: faulting address for page faults
    wire [31:0] trap_val = page_fault_data    ? mmu_fault_vaddr :
                           effective_ifetch_fault ? pc_reg :
                           32'd0;

    wire can_take_interrupt = interrupt_pending && !exception_taken &&
                              !ex_mem_is_mret && !ex_mem_is_sret;
    wire [31:0] interrupt_pc = pc_reg;  // Save current PC for interrupt resume
    wire take_trap = exception_taken || can_take_interrupt;

    // Gate trap taken to CSR file by !mmu_stall to prevent CSR corruption during stalls
    wire take_trap_effective = take_trap && !mmu_stall;

    wire [31:0] trap_cause_final = exception_taken ? exception_cause : interrupt_cause;
    wire [31:0] trap_pc_final = exception_taken ? exception_pc : interrupt_pc;

    csr_file csr_file_inst(
        .clk(clk),
        .rst(rst),
        .csr_addr(id_ex_csr_addr),
        .csr_rdata(csr_rdata),
        .csr_write(mem_wb_csr_write),
        .csr_waddr(mem_wb_csr_addr),
        .csr_wdata(mem_wb_csr_wdata),
        // Trap handling - gated by !mmu_stall
        .trap_taken(take_trap_effective),
        .trap_cause(trap_cause_final),
        .trap_pc(trap_pc_final),
        .trap_val(trap_val),
        // Interrupt inputs
        .timer_irq(clint_timer_irq),
        .external_irq(plic_external_irq),
        .software_irq(1'b0),
        // Interrupt outputs
        .interrupt_pending(interrupt_pending),
        .interrupt_cause(interrupt_cause),
        // Return instructions - gated by !mmu_stall
        .mret_taken(ex_mem_is_mret && !mmu_stall),
        .sret_taken(ex_mem_is_sret && !mmu_stall),
        // Privilege and trap routing
        .priv_level(priv_level),
        .trap_vector(trap_vector),
        .trap_return_pc(trap_return_pc),
        .trap_to_s_mode(trap_to_s_mode),
        // MMU-related outputs
        .satp_out(satp_value),
        .mstatus_mxr(mstatus_mxr),
        .mstatus_sum(mstatus_sum),
        .mstatus_mprv_out(mstatus_mprv),
        .mstatus_mpp_out(mstatus_mpp),
        // Performance counters
        .cycle_count(cycle),
        .retire_inst(retire_inst),
        // Timer from CLINT
        .mtime(clint_mtime)
    );

    // Current privilege level from CSR file
    assign current_priv = priv_level;

    // Instruction retire (for minstret counter)
    assign retire_inst = !pipeline_stall && !rst;

    // =========================================================================
    // ALU Input Muxes (EX Stage)
    // =========================================================================
    wire [31:0] pc_ex = id_ex_pc;

    wire [31:0] alu_in1_source_select =
        (id_ex_alu_in1_src == 2'b01) ? pc_ex :
        (id_ex_alu_in1_src == 2'b10) ? 32'b0 :
        (id_ex_alu_in1_src == 2'b11) ? id_ex_pcplus4 :
        id_ex_rs1_data;

    // Only apply forwarding when actually using rs1 (alu_in1_src == 00)
    wire [31:0] forwarded_alu_in1 =
        (id_ex_alu_in1_src != 2'b00) ? alu_in1_source_select :
        (ForwardA == 2'b00) ? alu_in1_source_select :
        (ForwardA == 2'b01) ? forward_data_wb :
        (ForwardA == 2'b10) ? forward_data_mem :
        alu_in1_source_select;

    wire [31:0] alu_in2_source_select = id_ex_alusrc ? id_ex_immediate : id_ex_rs2_data;

    wire [31:0] forwarded_alu_in2 =
        (ForwardB == 2'b00) ? alu_in2_source_select :
        (ForwardB == 2'b01) ? forward_data_wb :
        (ForwardB == 2'b10) ? forward_data_mem :
        alu_in2_source_select;

    wire [31:0] store_rs2_forwarded =
        (ForwardStore == 2'b01) ? forward_data_wb :
        (ForwardStore == 2'b10) ? forward_data_mem :
        id_ex_rs2_data;

    // rs1 forwarding for SFENCE.VMA (needs rs1 value in MEM stage)
    wire [31:0] sfence_rs1_forwarded =
        (ForwardA == 2'b01) ? forward_data_wb :
        (ForwardA == 2'b10) ? forward_data_mem :
        id_ex_rs1_data;

    // =========================================================================
    // CSR Operations (EX Stage)
    // =========================================================================
    wire [31:0] csr_rdata_ex = csr_rdata;
    wire [31:0] csr_operand = id_ex_is_csr_imm ? {27'b0, id_ex_rs1_addr} : forwarded_alu_in1;

    reg [31:0] csr_wdata_computed;
    always @(*) begin
        case (id_ex_funct3)
            3'b001, 3'b101: csr_wdata_computed = csr_operand;
            3'b010, 3'b110: csr_wdata_computed = csr_rdata_ex | csr_operand;
            3'b011, 3'b111: csr_wdata_computed = csr_rdata_ex & ~csr_operand;
            default:        csr_wdata_computed = 32'b0;
        endcase
    end

    wire csr_write_cond = (id_ex_funct3[1:0] == 2'b01) || (id_ex_rs1_addr != 5'b0);
    wire csr_should_write_ex = id_ex_is_csr && csr_write_cond;

    // =========================================================================
    // Branch Logic (MEM Stage)
    // =========================================================================
    wire is_beq  = (ex_mem_funct3 == 3'b000);
    wire is_bne  = (ex_mem_funct3 == 3'b001);
    wire is_blt  = (ex_mem_funct3 == 3'b100);
    wire is_bge  = (ex_mem_funct3 == 3'b101);
    wire is_bltu = (ex_mem_funct3 == 3'b110);
    wire is_bgeu = (ex_mem_funct3 == 3'b111);

    wire branch_taken = (is_beq  && ex_mem_zero_flag) ||
                        (is_bne  && !ex_mem_zero_flag) ||
                        (is_blt  && ex_mem_alu_lt) ||
                        (is_bge  && !ex_mem_alu_lt) ||
                        (is_bltu && ex_mem_alu_ltu) ||
                        (is_bgeu && !ex_mem_alu_ltu);

    wire take_branch_condition = ex_mem_isBtype && branch_taken;

    // Branch target calculation (EX stage)
    wire [31:0] branch_target_ex = pc_ex + id_ex_immediate;

    // =========================================================================
    // Memory Interface (MEM Stage)
    // =========================================================================
    wire [31:0] mem_rdata_muxed = clint_addr_valid ? clint_rdata :
                                  plic_addr_valid  ? plic_rdata  :
                                  mem_rdata;

    // Load data formatting
    wire mem_byteAccess_mem     = (ex_mem_funct3[1:0] == 2'b00);
    wire mem_halfwordAccess_mem = (ex_mem_funct3[1:0] == 2'b01);
    wire [15:0] LOAD_halfword_mem = ex_mem_alu_result[1] ? mem_rdata[31:16] : mem_rdata[15:0];
    wire [7:0]  LOAD_byte_mem     = ex_mem_alu_result[0] ? LOAD_halfword_mem[15:8] : LOAD_halfword_mem[7:0];
    wire LOAD_sign_mem = !ex_mem_funct3[2];

    wire [31:0] load_data_formatted =
        mem_byteAccess_mem     ? {{24{LOAD_sign_mem & LOAD_byte_mem[7]}}, LOAD_byte_mem} :
        mem_halfwordAccess_mem ? {{16{LOAD_sign_mem & LOAD_halfword_mem[15]}}, LOAD_halfword_mem} :
        mem_rdata_muxed;

    // Store data alignment
    wire [31:0] aligned_store_data =
        (ex_mem_funct3 == 3'b010) ? ex_mem_rs2_data :
        (ex_mem_funct3 == 3'b001) ? {2{ex_mem_rs2_data[15:0]}} :
        {4{ex_mem_rs2_data[7:0]}};

    // Store byte enables
    wire [3:0] store_byte_enables =
        (ex_mem_funct3 == 3'b010) ? 4'b1111 :
        (ex_mem_funct3 == 3'b001) ? (ex_mem_alu_result[1] ? 4'b1100 : 4'b0011) :
        (ex_mem_funct3 == 3'b000) ? (ex_mem_alu_result[1] ?
            (ex_mem_alu_result[0] ? 4'b1000 : 4'b0100) :
            (ex_mem_alu_result[0] ? 4'b0010 : 4'b0001)) :
        4'b0000;

    // Memory interface outputs - Physical addresses from MMU
    assign imem_addr = ifetch_paddr[31:0];    // Instruction fetch physical address
    assign imem_rstrb = 1'b1;

    // Data memory: PTW takes over during page table walks
    assign mem_addr = ptw_mem_req ? ptw_mem_addr[31:0] : data_paddr[31:0];
    assign mem_rstrb = ptw_mem_req || ex_mem_mem_read ||
                       ex_mem_is_lr || ex_mem_is_sc || ex_mem_is_amo;

    // Memory write data/strobe - handle PTW writes, atomic operations, and normal stores
    wire normal_mem_write = ex_mem_mem_write && !is_atomic_mem;
    assign mem_wdata = ptw_mem_req   ? ptw_mem_wdata :
                       is_atomic_mem ? atomic_mem_wdata : aligned_store_data;
    assign mem_wstrb = ptw_mem_req ? (ptw_mem_write ? 4'b1111 : 4'b0000) :
                       (normal_mem_write || atomic_do_mem_write) ?
                       (is_atomic_mem ? 4'b1111 : store_byte_enables) : 4'b0000;

    wire [31:0] instruction_from_mem = decompressed_instruction;

    // =========================================================================
    // Register File
    // =========================================================================
    wire [4:0]  rd_wb = mem_wb_rd_addr;
    wire        reg_write_wb = mem_wb_reg_write;

    reg_file reg_file_inst (
        .clk(clk),
        .rst(rst),
        .rs1(rs1_id),
        .rs2(rs2_id),
        .rd(rd_wb),
        .write_data(write_data_to_reg),
        .write_en(reg_write_wb),
        .rs1_data(rs1_data),
        .rs2_data(rs2_data)
    );

    // =========================================================================
    // ALU
    // =========================================================================
    alu alu_inst (
        .alu_in1(forwarded_alu_in1),
        .alu_in2(forwarded_alu_in2),
        .ALUOp(id_ex_alu_op),
        .alu_out(alu_result),
        .zero_flag(zero_flag),
        .alu_lt(alu_lt),
        .alu_ltu(alu_ltu)
    );

    // =========================================================================
    // Iterative Divider (replaces combinational DIV/REM in ALU)
    // =========================================================================
    wire is_div_op = (id_ex_alu_op == `ALU_DIV)  || (id_ex_alu_op == `ALU_DIVU) ||
                     (id_ex_alu_op == `ALU_REM)  || (id_ex_alu_op == `ALU_REMU);
    wire div_is_unsigned = (id_ex_alu_op == `ALU_DIVU) || (id_ex_alu_op == `ALU_REMU);
    wire div_is_rem = (id_ex_alu_op == `ALU_REM) || (id_ex_alu_op == `ALU_REMU);

    wire [31:0] div_result;
    wire        div_busy;
    wire        div_done;
    wire        div_start = is_div_op && !div_busy && !div_done;
    wire        div_stall = div_busy || (is_div_op && !div_done);

    div_unit div_unit_inst (
        .clk(clk),
        .rst(rst),
        .start(div_start),
        .is_unsigned(div_is_unsigned),
        .is_rem(div_is_rem),
        .dividend(forwarded_alu_in1),
        .divisor(forwarded_alu_in2),
        .result(div_result),
        .busy(div_busy),
        .done(div_done)
    );

    // Mux div result into ALU result for div operations
    wire [31:0] effective_alu_result = is_div_op ? div_result : alu_result;

    // =========================================================================
    // Next PC Logic
    // =========================================================================
    assign next_pc =
        take_trap ? trap_vector :
        (ex_mem_is_mret || ex_mem_is_sret) ? trap_return_pc :
        ex_mem_isJALR ? (ex_mem_alu_result & 32'hFFFFFFFE) :
        take_branch_condition ? ex_mem_branch_target :
        ex_mem_isJAL ? ex_mem_branch_target :
        pcplus_if;

    wire pipeline_flush = (take_trap || take_branch_condition ||
                          ex_mem_isJAL || ex_mem_isJALR ||
                          ex_mem_is_mret || ex_mem_is_sret ||
                          ex_mem_is_sfence_vma) && !mmu_stall;

    // =========================================================================
    // Writeback Data Mux
    // =========================================================================
    wire [31:0] wb_data =
        (mem_wb_isJAL | mem_wb_isJALR) ? mem_wb_pcplus4 :
        mem_wb_mem_to_reg ? mem_wb_mem_data :
        mem_wb_alu_result;

    assign write_data_to_reg =
        mem_wb_is_atomic ? mem_wb_atomic_data :
        mem_wb_is_csr ? mem_wb_csr_rdata :
        mem_wb_mem_to_reg ? mem_wb_mem_data :
        wb_data;

    // =========================================================================
    // Pipeline Registers (Sequential Logic)
    // =========================================================================
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // =================================================================
            // Full Reset
            // =================================================================
            pc_reg <= 32'h80000000;  // Start at RAM base
            // RVC state
            saved_half   <= 16'b0;
            have_saved   <= 1'b0;
            spanning_pc  <= 32'b0;
            // IF/ID Reset
            if_id_instruction <= 32'h00000013;
            if_id_pcplus4     <= 32'b0;
            if_id_pc          <= 32'b0;
            // ID/EX Reset
            id_ex_pcplus4     <= 32'b0;
            id_ex_pc          <= 32'b0;
            id_ex_rs1_data    <= 32'b0;
            id_ex_rs2_data    <= 32'b0;
            id_ex_immediate   <= 32'b0;
            id_ex_rs1_addr    <= 5'b0;
            id_ex_rs2_addr    <= 5'b0;
            id_ex_rd_addr     <= 5'b0;
            id_ex_alu_op      <= `ALU_ADD;
            id_ex_alusrc      <= 1'b0;
            id_ex_mem_read    <= 1'b0;
            id_ex_mem_write   <= 1'b0;
            id_ex_reg_write   <= 1'b0;
            id_ex_mem_to_reg  <= 1'b0;
            id_ex_funct3      <= 3'b0;
            id_ex_isBtype_reg <= 1'b0;
            id_ex_isJAL       <= 1'b0;
            id_ex_isJALR      <= 1'b0;
            id_ex_alu_in1_src <= 2'b00;
            id_ex_csr_addr    <= 12'b0;
            id_ex_is_csr      <= 1'b0;
            id_ex_is_csr_imm  <= 1'b0;
            id_ex_is_ecall    <= 1'b0;
            id_ex_is_ebreak   <= 1'b0;
            id_ex_is_mret     <= 1'b0;
            id_ex_is_sret     <= 1'b0;
            id_ex_is_lr       <= 1'b0;
            id_ex_is_sc       <= 1'b0;
            id_ex_is_amo      <= 1'b0;
            id_ex_amo_funct5  <= 5'b0;
            // EX/MEM Reset
            ex_mem_alu_result <= 32'b0;
            ex_mem_pc         <= 32'b0;
            ex_mem_rs1_data   <= 32'b0;
            ex_mem_rs2_data   <= 32'b0;
            ex_mem_rs1_addr   <= 5'b0;
            ex_mem_rs2_addr   <= 5'b0;
            ex_mem_rd_addr    <= 5'b0;
            ex_mem_zero_flag  <= 1'b0;
            ex_mem_alu_lt     <= 1'b0;
            ex_mem_alu_ltu    <= 1'b0;
            ex_mem_isJAL      <= 1'b0;
            ex_mem_isJALR     <= 1'b0;
            ex_mem_pcplus4    <= 32'b0;
            ex_mem_branch_target <= 32'b0;
            ex_mem_mem_to_reg <= 1'b0;
            ex_mem_mem_read   <= 1'b0;
            ex_mem_mem_write  <= 1'b0;
            ex_mem_reg_write  <= 1'b0;
            ex_mem_is_mret    <= 1'b0;
            ex_mem_is_sret    <= 1'b0;
            ex_mem_isBtype    <= 1'b0;
            ex_mem_funct3     <= 3'b0;
            ex_mem_csr_wdata  <= 32'b0;
            ex_mem_csr_addr   <= 12'b0;
            ex_mem_is_csr     <= 1'b0;
            ex_mem_csr_write  <= 1'b0;
            ex_mem_csr_rdata  <= 32'b0;
            ex_mem_is_ecall   <= 1'b0;
            ex_mem_is_ebreak  <= 1'b0;
            ex_mem_is_lr      <= 1'b0;
            ex_mem_is_sc      <= 1'b0;
            ex_mem_is_amo     <= 1'b0;
            ex_mem_amo_funct5 <= 5'b0;
            ex_mem_is_sfence_vma <= 1'b0;
            // MEM/WB Reset
            mem_wb_mem_data   <= 32'b0;
            mem_wb_alu_result <= 32'b0;
            mem_wb_rd_addr    <= 5'b0;
            mem_wb_reg_write  <= 1'b0;
            mem_wb_mem_to_reg <= 1'b0;
            mem_wb_pcplus4    <= 32'b0;
            mem_wb_isJAL      <= 1'b0;
            mem_wb_isJALR     <= 1'b0;
            mem_wb_csr_rdata  <= 32'b0;
            mem_wb_csr_wdata  <= 32'b0;
            mem_wb_csr_addr   <= 12'b0;
            mem_wb_csr_write  <= 1'b0;
            mem_wb_is_csr     <= 1'b0;
            mem_wb_atomic_data <= 32'b0;
            mem_wb_is_atomic  <= 1'b0;
            // Debug
            debug_id_instruction  <= 32'h00000013;
            debug_ex_instruction  <= 32'h00000013;
            debug_mem_instruction <= 32'h00000013;
            debug_wb_instruction  <= 32'h00000013;

        end else if (pipeline_flush) begin
            // =================================================================
            // Pipeline Flush - update PC, flush IF/ID, ID/EX, EX/MEM
            // MEM/WB continues for writeback of the flushing instruction
            // =================================================================
            pc_reg <= next_pc;

            // Clear RVC spanning state
            have_saved <= 1'b0;

            // Flush IF/ID
            if_id_instruction <= 32'h00000013;
            if_id_pcplus4     <= 32'b0;
            if_id_pc          <= 32'b0;

            // Flush ID/EX
            id_ex_pcplus4     <= 32'b0;
            id_ex_pc          <= 32'b0;
            id_ex_rs1_data    <= 32'b0;
            id_ex_rs2_data    <= 32'b0;
            id_ex_immediate   <= 32'b0;
            id_ex_rs1_addr    <= 5'b0;
            id_ex_rs2_addr    <= 5'b0;
            id_ex_rd_addr     <= 5'b0;
            id_ex_alu_op      <= `ALU_ADD;
            id_ex_alusrc      <= 1'b0;
            id_ex_mem_read    <= 1'b0;
            id_ex_mem_write   <= 1'b0;
            id_ex_reg_write   <= 1'b0;
            id_ex_mem_to_reg  <= 1'b0;
            id_ex_funct3      <= 3'b0;
            id_ex_isBtype_reg <= 1'b0;
            id_ex_isJAL       <= 1'b0;
            id_ex_isJALR      <= 1'b0;
            id_ex_alu_in1_src <= 2'b00;
            id_ex_csr_addr    <= 12'b0;
            id_ex_is_csr      <= 1'b0;
            id_ex_is_csr_imm  <= 1'b0;
            id_ex_is_ecall    <= 1'b0;
            id_ex_is_ebreak   <= 1'b0;
            id_ex_is_mret     <= 1'b0;
            id_ex_is_sret     <= 1'b0;
            id_ex_is_lr       <= 1'b0;
            id_ex_is_sc       <= 1'b0;
            id_ex_is_amo      <= 1'b0;
            id_ex_amo_funct5  <= 5'b0;

            // Flush EX/MEM
            ex_mem_alu_result <= 32'b0;
            ex_mem_pc         <= 32'b0;
            ex_mem_rs1_data   <= 32'b0;
            ex_mem_rs2_data   <= 32'b0;
            ex_mem_rs1_addr   <= 5'b0;
            ex_mem_rs2_addr   <= 5'b0;
            ex_mem_rd_addr    <= 5'b0;
            ex_mem_zero_flag  <= 1'b0;
            ex_mem_alu_lt     <= 1'b0;
            ex_mem_alu_ltu    <= 1'b0;
            ex_mem_pcplus4    <= 32'b0;
            ex_mem_branch_target <= 32'b0;
            ex_mem_mem_to_reg <= 1'b0;
            ex_mem_mem_read   <= 1'b0;
            ex_mem_mem_write  <= 1'b0;
            ex_mem_reg_write  <= 1'b0;
            ex_mem_is_mret    <= 1'b0;
            ex_mem_is_sret    <= 1'b0;
            ex_mem_isBtype    <= 1'b0;
            ex_mem_isJAL      <= 1'b0;
            ex_mem_isJALR     <= 1'b0;
            ex_mem_funct3     <= 3'b0;
            ex_mem_csr_wdata  <= 32'b0;
            ex_mem_csr_addr   <= 12'b0;
            ex_mem_is_csr     <= 1'b0;
            ex_mem_csr_write  <= 1'b0;
            ex_mem_csr_rdata  <= 32'b0;
            ex_mem_is_ecall   <= 1'b0;
            ex_mem_is_ebreak  <= 1'b0;
            ex_mem_is_lr      <= 1'b0;
            ex_mem_is_sc      <= 1'b0;
            ex_mem_is_amo     <= 1'b0;
            ex_mem_amo_funct5 <= 5'b0;
            ex_mem_is_sfence_vma <= 1'b0;

            // MEM/WB continues - let the flushing instruction complete writeback
            mem_wb_mem_data   <= mem_rdata;
            mem_wb_alu_result <= ex_mem_alu_result;
            mem_wb_rd_addr    <= ex_mem_rd_addr;
            mem_wb_reg_write  <= ex_mem_reg_write;
            mem_wb_mem_to_reg <= ex_mem_mem_to_reg;
            mem_wb_pcplus4    <= ex_mem_pcplus4;
            mem_wb_isJAL      <= ex_mem_isJAL;
            mem_wb_isJALR     <= ex_mem_isJALR;
            mem_wb_csr_rdata  <= ex_mem_csr_rdata;
            mem_wb_csr_wdata  <= ex_mem_csr_wdata;
            mem_wb_csr_addr   <= ex_mem_csr_addr;
            mem_wb_csr_write  <= ex_mem_csr_write;
            mem_wb_is_csr     <= ex_mem_is_csr;
            mem_wb_atomic_data <= atomic_rd_data;
            mem_wb_is_atomic  <= is_atomic_mem;

            // Debug
            debug_id_instruction  <= 32'h00000013;
            debug_ex_instruction  <= 32'h00000013;
            debug_mem_instruction <= 32'h00000013;
            debug_wb_instruction  <= debug_mem_instruction;

        end else if (mmu_stall) begin
            // =================================================================
            // MMU / Divider Stall - entire pipeline frozen
            // All registers retain their values (no assignments needed)
            // =================================================================

        end else if (data_hazard_stall) begin
            // =================================================================
            // Data Hazard Stall (load-use or CSR-use hazard)
            // - Keep PC unchanged (re-fetch same instruction next cycle)
            // - Keep IF/ID unchanged (re-decode same instruction)
            // - Insert bubble (NOP) into ID/EX
            // - Let EX/MEM and MEM/WB continue normally
            // =================================================================

            // ID/EX gets a bubble - clear all control signals
            id_ex_mem_read    <= 1'b0;
            id_ex_mem_write   <= 1'b0;
            id_ex_reg_write   <= 1'b0;
            id_ex_isBtype_reg <= 1'b0;
            id_ex_isJAL       <= 1'b0;
            id_ex_isJALR      <= 1'b0;
            id_ex_is_csr      <= 1'b0;
            id_ex_is_ecall    <= 1'b0;
            id_ex_is_ebreak   <= 1'b0;
            id_ex_is_mret     <= 1'b0;
            id_ex_is_sret     <= 1'b0;
            id_ex_is_lr       <= 1'b0;
            id_ex_is_sc       <= 1'b0;
            id_ex_is_amo      <= 1'b0;
            id_ex_rd_addr     <= 5'b0;

            // EX/MEM continues normally
            ex_mem_alu_result <= effective_alu_result;
            ex_mem_pc         <= id_ex_pc;
            ex_mem_rs1_data   <= sfence_rs1_forwarded;
            ex_mem_rs2_data   <= store_rs2_forwarded;
            ex_mem_rs1_addr   <= id_ex_rs1_addr;
            ex_mem_rs2_addr   <= id_ex_rs2_addr;
            ex_mem_rd_addr    <= id_ex_rd_addr;
            ex_mem_zero_flag  <= zero_flag;
            ex_mem_alu_lt     <= alu_lt;
            ex_mem_alu_ltu    <= alu_ltu;
            ex_mem_mem_read   <= id_ex_mem_read;
            ex_mem_mem_write  <= id_ex_mem_write;
            ex_mem_reg_write  <= id_ex_reg_write;
            ex_mem_mem_to_reg <= id_ex_mem_to_reg;
            ex_mem_funct3     <= id_ex_funct3;
            ex_mem_isBtype    <= id_ex_isBtype_reg;
            ex_mem_isJAL      <= id_ex_isJAL;
            ex_mem_isJALR     <= id_ex_isJALR;
            ex_mem_pcplus4    <= id_ex_pcplus4;
            ex_mem_branch_target <= branch_target_ex;
            ex_mem_csr_wdata  <= csr_wdata_computed;
            ex_mem_csr_addr   <= id_ex_csr_addr;
            ex_mem_is_csr     <= id_ex_is_csr;
            ex_mem_csr_write  <= csr_should_write_ex;
            ex_mem_csr_rdata  <= csr_rdata;
            ex_mem_is_ecall   <= id_ex_is_ecall;
            ex_mem_is_ebreak  <= id_ex_is_ebreak;
            ex_mem_is_mret    <= id_ex_is_mret;
            ex_mem_is_sret    <= id_ex_is_sret;
            ex_mem_is_lr      <= id_ex_is_lr;
            ex_mem_is_sc      <= id_ex_is_sc;
            ex_mem_is_amo     <= id_ex_is_amo;
            ex_mem_amo_funct5 <= id_ex_amo_funct5;
            ex_mem_is_sfence_vma <= is_sfence_vma_ctrl;

            // MEM/WB continues normally
            mem_wb_mem_data   <= load_data_formatted;
            mem_wb_alu_result <= ex_mem_alu_result;
            mem_wb_rd_addr    <= ex_mem_rd_addr;
            mem_wb_reg_write  <= ex_mem_reg_write;
            mem_wb_mem_to_reg <= ex_mem_mem_to_reg;
            mem_wb_pcplus4    <= ex_mem_pcplus4;
            mem_wb_isJAL      <= ex_mem_isJAL;
            mem_wb_isJALR     <= ex_mem_isJALR;
            mem_wb_csr_rdata  <= ex_mem_csr_rdata;
            mem_wb_csr_wdata  <= ex_mem_csr_wdata;
            mem_wb_csr_addr   <= ex_mem_csr_addr;
            mem_wb_csr_write  <= ex_mem_csr_write;
            mem_wb_is_csr     <= ex_mem_is_csr;
            mem_wb_atomic_data <= atomic_rd_data;
            mem_wb_is_atomic  <= is_atomic_mem;

            // Debug
            debug_id_instruction  <= if_id_instruction;
            debug_ex_instruction  <= 32'h00000013;
            debug_mem_instruction <= debug_ex_instruction;
            debug_wb_instruction  <= debug_mem_instruction;

        end else begin
            // =================================================================
            // Normal Operation (no stall, no flush)
            // =================================================================

                // --- Clock PC ---
                pc_reg <= next_pc;

                // --- RVC spanning state ---
                if (rvc_need_next_half) begin
                    saved_half  <= imem_rdata[31:16];
                    have_saved  <= 1'b1;
                    spanning_pc <= pc_reg;
                end else begin
                    have_saved  <= 1'b0;
                end

                // --- IF/ID ---
                if_id_instruction <= instruction_from_mem;
                if_id_pcplus4     <= pcplus_if;
                if_id_pc          <= have_saved ? spanning_pc : pc_reg;

                // --- ID/EX ---
                id_ex_pcplus4     <= if_id_pcplus4;
                id_ex_pc          <= if_id_pc;
                id_ex_rs1_data    <= rs1_data;
                id_ex_rs2_data    <= rs2_data;
                id_ex_immediate   <= immediate_id;
                id_ex_rs1_addr    <= rs1_id;
                id_ex_rs2_addr    <= rs2_id;
                id_ex_rd_addr     <= rd_id;
                id_ex_alu_op      <= alu_op_ctrl;
                id_ex_alusrc      <= alusrc_ctrl;
                id_ex_mem_read    <= mem_read_ctrl;
                id_ex_mem_write   <= mem_write_ctrl;
                id_ex_reg_write   <= reg_write_ctrl;
                id_ex_mem_to_reg  <= mem_to_reg_ctrl;
                id_ex_funct3      <= funct3_id;
                id_ex_alu_in1_src <= ctrl_alu_in1_src;
                id_ex_isBtype_reg <= is_branch_ctrl;
                id_ex_isJAL       <= is_jal_ctrl;
                id_ex_isJALR      <= is_jalr_ctrl;
                id_ex_csr_addr    <= csr_addr;
                id_ex_is_csr      <= is_csr_ctrl;
                id_ex_is_csr_imm  <= funct3_id[2];
                id_ex_is_ecall    <= is_ecall_ctrl;
                id_ex_is_ebreak   <= is_ebreak_ctrl;
                id_ex_is_mret     <= is_mret_ctrl;
                id_ex_is_sret     <= is_sret_ctrl;
                id_ex_is_lr       <= is_lr_ctrl;
                id_ex_is_sc       <= is_sc_ctrl;
                id_ex_is_amo      <= is_amo_ctrl;
                id_ex_amo_funct5  <= amo_funct5_ctrl;

                // --- EX/MEM ---
                ex_mem_alu_result <= effective_alu_result;
                ex_mem_pc         <= id_ex_pc;
                ex_mem_rs1_data   <= sfence_rs1_forwarded;
                ex_mem_rs2_data   <= store_rs2_forwarded;
                ex_mem_rs1_addr   <= id_ex_rs1_addr;
                ex_mem_rs2_addr   <= id_ex_rs2_addr;
                ex_mem_rd_addr    <= id_ex_rd_addr;
                ex_mem_zero_flag  <= zero_flag;
                ex_mem_alu_lt     <= alu_lt;
                ex_mem_alu_ltu    <= alu_ltu;
                ex_mem_mem_read   <= id_ex_mem_read;
                ex_mem_mem_write  <= id_ex_mem_write;
                ex_mem_reg_write  <= id_ex_reg_write;
                ex_mem_mem_to_reg <= id_ex_mem_to_reg;
                ex_mem_funct3     <= id_ex_funct3;
                ex_mem_isBtype    <= id_ex_isBtype_reg;
                ex_mem_isJAL      <= id_ex_isJAL;
                ex_mem_isJALR     <= id_ex_isJALR;
                ex_mem_pcplus4    <= id_ex_pcplus4;
                ex_mem_branch_target <= branch_target_ex;
                ex_mem_csr_wdata  <= csr_wdata_computed;
                ex_mem_csr_addr   <= id_ex_csr_addr;
                ex_mem_is_csr     <= id_ex_is_csr;
                ex_mem_csr_write  <= csr_should_write_ex;
                ex_mem_csr_rdata  <= csr_rdata;
                ex_mem_is_ecall   <= id_ex_is_ecall;
                ex_mem_is_ebreak  <= id_ex_is_ebreak;
                ex_mem_is_mret    <= id_ex_is_mret;
                ex_mem_is_sret    <= id_ex_is_sret;
                ex_mem_is_lr      <= id_ex_is_lr;
                ex_mem_is_sc      <= id_ex_is_sc;
                ex_mem_is_amo     <= id_ex_is_amo;
                ex_mem_amo_funct5 <= id_ex_amo_funct5;
                ex_mem_is_sfence_vma <= is_sfence_vma_ctrl;

                // --- MEM/WB ---
                mem_wb_mem_data   <= load_data_formatted;
                mem_wb_alu_result <= ex_mem_alu_result;
                mem_wb_rd_addr    <= ex_mem_rd_addr;
                mem_wb_reg_write  <= ex_mem_reg_write;
                mem_wb_mem_to_reg <= ex_mem_mem_to_reg;
                mem_wb_pcplus4    <= ex_mem_pcplus4;
                mem_wb_isJAL      <= ex_mem_isJAL;
                mem_wb_isJALR     <= ex_mem_isJALR;
                mem_wb_csr_rdata  <= ex_mem_csr_rdata;
                mem_wb_csr_wdata  <= ex_mem_csr_wdata;
                mem_wb_csr_addr   <= ex_mem_csr_addr;
                mem_wb_csr_write  <= ex_mem_csr_write;
                mem_wb_is_csr     <= ex_mem_is_csr;
                mem_wb_atomic_data <= atomic_rd_data;
                mem_wb_is_atomic  <= is_atomic_mem;

                // Debug
                debug_id_instruction  <= if_id_instruction;
                debug_ex_instruction  <= if_id_instruction;
                debug_mem_instruction <= debug_ex_instruction;
                debug_wb_instruction  <= debug_mem_instruction;
        end
    end

    // =========================================================================
    // Forwarding Unit
    // =========================================================================
    always @(*) begin
        ForwardA = 2'b00;
        ForwardB = 2'b00;

        // EX/MEM Hazard
        if (ex_mem_reg_write && (ex_mem_rd_addr != 5'b0)) begin
            if (ex_mem_rd_addr == id_ex_rs1_addr)
                ForwardA = 2'b10;
            if ((ex_mem_rd_addr == id_ex_rs2_addr) && !id_ex_alusrc)
                ForwardB = 2'b10;
        end

        // MEM/WB Hazard (lower priority)
        if (mem_wb_reg_write && (mem_wb_rd_addr != 5'b0)) begin
            if ((mem_wb_rd_addr == id_ex_rs1_addr) &&
                !(ex_mem_reg_write && (ex_mem_rd_addr != 5'b0) && (ex_mem_rd_addr == id_ex_rs1_addr)))
                ForwardA = 2'b01;
            if ((mem_wb_rd_addr == id_ex_rs2_addr) && !id_ex_alusrc &&
                !(ex_mem_reg_write && (ex_mem_rd_addr != 5'b0) && (ex_mem_rd_addr == id_ex_rs2_addr)))
                ForwardB = 2'b01;
        end
    end

    // Store forwarding
    always @(*) begin
        ForwardStore = 2'b00;
        if (ex_mem_reg_write && (ex_mem_rd_addr != 5'b0) && (ex_mem_rd_addr == id_ex_rs2_addr))
            ForwardStore = 2'b10;
        else if (mem_wb_reg_write && (mem_wb_rd_addr != 5'b0) && (mem_wb_rd_addr == id_ex_rs2_addr))
            ForwardStore = 2'b01;
    end

    // =========================================================================
    // Cycle Counter
    // =========================================================================
    always @(posedge clk or posedge rst) begin
        if (rst)
            cycle <= 0;
        else
            cycle <= cycle + 1;
    end

    // =========================================================================
    // Debug Monitoring (disabled - set cycle range to enable)
    // =========================================================================
    `ifdef SIMULATION
    always @(posedge clk) begin
        if (!rst && cycle >= 1 && cycle <= 0) begin  // Disabled (cycle <= 0)
            $display("\n========== Cycle %0d ==========", cycle);
            $display("PC=0x%08x  next_pc=0x%08x", pc_reg, next_pc);
            $display("STALL: pipeline=%b mmu_ifetch=%b mmu_data=%b hazard=%b",
                     pipeline_stall, mmu_stall_ifetch, mmu_stall_data, data_hazard_stall);
            $display("MMU: ifetch_ready=%b data_ready=%b ptw_req=%b",
                     ifetch_ready, data_ready, ptw_mem_req);
            $display("IF/ID: inst=0x%08x", if_id_instruction);
            $display("EX/MEM: alu=0x%08x mem_r=%b mem_w=%b",
                     ex_mem_alu_result, ex_mem_mem_read, ex_mem_mem_write);
            $display("MEM: addr=0x%08x wdata=0x%08x wstrb=%b",
                     mem_addr, mem_wdata, mem_wstrb);
            $display("WB: rd=%d reg_w=%b data=0x%08x",
                     mem_wb_rd_addr, mem_wb_reg_write, write_data_to_reg);
        end
    end
    `endif

endmodule
