`include "../src/reg_file.v" // Include the register file module
 `include "../src/ALU.v"      // Include the ALU module
 `include "../src/csr_file.v"
`include "../src/definitions.v" 
`include "../src/control_unit.v"
module cpu(
    input rst, clk,
      // Instruction memory (read-only)
  output wire [31:0] imem_addr,     // usually PC
  input  wire [31:0] imem_rdata,
  output wire        imem_rstrb,    // fetch enable (often 1'b1)

    input [31:0] mem_rdata, // Data read from memory (Instruction or Load data)
    output [31:0] mem_addr,  // Address to memory (PC for fetch, or Load/Store address)
    output [31:0] mem_wdata, // Data to write to memory (Store data)
    output mem_rstrb,        // Memory read strobe/enable
    output reg [31:0] cycle, // Keep for testbench for now
    output [3:0] mem_wstrb   // Memory write strobe (byte enables)
  );

  reg [31:0] debug_id_instruction;  // Instruction leaving ID stage (entering EX)
  reg [31:0] debug_ex_instruction;  // Instruction leaving EX stage (entering MEM)
  reg [31:0] debug_mem_instruction; // Instruction leaving MEM stage (entering WB)
  reg [31:0] debug_wb_instruction;  // Instruction leaving WB stage
initial begin
      cycle = 0; // Initialize cycle counter 
      // Initialize debug registers to NOP to avoid unknown state at start
      debug_id_instruction  = 32'h00000013;
      debug_ex_instruction  = 32'h00000013;
      debug_mem_instruction = 32'h00000013;
      debug_wb_instruction  = 32'h00000013;
  end
  reg [31:0] pc_reg; // Program Counter register

  //***** IF/ID PIPELINE REGISTERS ******//
  reg [31:0] if_id_instruction;
  reg [31:0] if_id_pcplus4;

  //***** ID/EX PIPELINE REGISTERS ******//
  // Data values
  reg [31:0] id_ex_pcplus4;     // PC+4 from IF/ID
  reg [31:0] id_ex_rs1_data;    // Data read from register file
  reg [31:0] id_ex_rs2_data;    // Data read from register file
  reg [31:0] id_ex_immediate;   // Sign-extended immediate
  // Register addresses (needed for forwarding/debugging)
  reg [4:0]  id_ex_rs1_addr;    // rs1 address
  reg [4:0]  id_ex_rs2_addr;    // rs2 address
  reg [4:0]  id_ex_rd_addr;     // Destination register address
  // Control Signals (Generated in ID, used in EX/MEM/WB)
  reg [3:0]  id_ex_alu_op;      // ALU operation code
  reg [1:0] id_ex_alu_in1_src;
  reg        id_ex_alusrc;      // Mux select for ALU input 2 (0=rs2_data, 1=immediate)
  reg        id_ex_mem_read;    // Enable memory read in MEM stage
  reg        id_ex_mem_write;   // Enable memory write in MEM stage
  reg        id_ex_reg_write;   // Enable register write in WB stage
  reg        id_ex_mem_to_reg;  // Mux select for writeback data (0=ALU result, 1=Memory data)
  reg id_ex_isBtype_reg;
  reg id_ex_isJAL; 
  reg id_ex_isJALR;

reg [11:0] id_ex_csr_addr;   // CSR address (imm[11:0])
reg        id_ex_is_csr;     // Is this a CSR instruction?
reg        id_ex_is_ecall;   // Is this ECALL?
reg        id_ex_is_ebreak;  // Is this EBREAK?
reg        id_ex_is_csr_imm; // 
  // Store instruction type information if needed later (optional but helpful)
  reg [2:0]  id_ex_funct3;      // Pass funct3 for Load/Store byte/halfword handling
reg id_ex_is_mret;

  //********* Wires for interfacing with Reg File and ALU *******//
  wire [31:0] rs1_data;      // Output from Reg File (ID stage)
  wire [31:0] rs2_data;      // Output from Reg File (ID stage)
    wire        alu_lt;  
  wire        alu_ltu;    
  wire [31:0] alu_result;    // Output from ALU (EX stage)
  wire        zero_flag;     // Output from ALU (EX stage)
  wire [31:0] write_data_to_reg; // Data input to Reg File write port (WB stage)
  // wire        write_reg_en;      // Replaced by id_ex_reg_write passed down pipeline
  // wire [3:0]  alu_op_signal;     // Replaced by id_ex_alu_op passed down pipeline

  // --- IF Stage ---
  wire [31:0] pcplus4_if = pc_reg + 4;         // PC + 4 calculated this cycle
  wire [31:0] next_pc; // Mux to select the actual next PC value

  // --- ID Stage (uses outputs from IF/ID registers) ---
  wire [4:0] opcode_id = if_id_instruction[6:2]; // Use ID suffix for clarity
  wire [4:0] rd_id     = if_id_instruction[11:7];
  wire [4:0] rs1_id    = if_id_instruction[19:15];
  wire [4:0] rs2_id    = if_id_instruction[24:20];
  wire [2:0] funct3_id = if_id_instruction[14:12];
  wire [6:0] funct7_id = if_id_instruction[31:25];
  // Immediate values generated from registered instruction
  wire [31:0] imm_i = {{21{if_id_instruction[31]}},if_id_instruction[30:20]};
  wire [31:0] imm_b = {{20{if_id_instruction[31]}},if_id_instruction[7],if_id_instruction[30:25],if_id_instruction[11:8],1'b0};
  wire [31:0] imm_s = {{21{if_id_instruction[31]}},if_id_instruction[30:25],if_id_instruction[11:7]};
  wire [31:0] imm_j = {{12{if_id_instruction[31]}},if_id_instruction[19:12],if_id_instruction[20],if_id_instruction[30:21],1'b0};
  wire [31:0] imm_u = {if_id_instruction[31],if_id_instruction[30:12],12'h000};
    wire [11:0] csr_addr = if_id_instruction[31:20];  // CSR address is in imm field
  wire [31:0] alu_in1_mux;
  reg [2:0] ForwardA;
    reg [2:0] ForwardB;
  // Immediate Mux (Select correct immediate based on type) - Combinational in ID
  reg [31:0] immediate_id; // Use reg because assigned in always block
  always @(*) begin
      // Default to I-type, modify as needed
      immediate_id = imm_i; // Common case
      if (opcode_id == 5'b01000) // S-type (Store)
          immediate_id = imm_s;
      else if (opcode_id == 5'b11000) // B-type (Branch)
          immediate_id = imm_b;
      else if (opcode_id == 5'b11011) // J-type (JAL)
          immediate_id = imm_j;
      else if (opcode_id == 5'b01101 || opcode_id == 5'b00101) // U-type (LUI, AUIPC)
          immediate_id = imm_u;
      // Default covers I-type (Load, JALR, Imm Arith)
  end



// Control signals
wire [3:0] alu_op_ctrl;
wire [1:0] ctrl_alu_in1_src;
wire alusrc_ctrl, mem_read_ctrl, mem_write_ctrl;
wire reg_write_ctrl, mem_to_reg_ctrl;
wire is_branch_ctrl, is_jal_ctrl, is_jalr_ctrl;
wire is_csr_ctrl, is_ecall_ctrl, is_ebreak_ctrl;  
wire retire_inst;
wire is_mret_ctrl;
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
    .is_mret(is_mret_ctrl)
);
wire [31:0] csr_rdata; 
csr_file csr_file_inst(
    .clk(clk),
    .rst(rst),
    .csr_addr(id_ex_csr_addr),
    .csr_rdata(csr_rdata),
    .csr_write(mem_wb_csr_write),
    .csr_waddr(mem_wb_csr_addr),
    .csr_wdata(mem_wb_csr_wdata),
        // Trap handling (MEM stage)
    .trap_taken(trap_taken),
    .trap_cause(trap_cause),
    .trap_pc(trap_pc),
    .cycle_count(cycle),
    .retire_inst(retire_inst),
    .mret_taken(ex_mem_is_mret)
);

wire [31:0] forward_data_mem = ex_mem_alu_result;     // Data source from EX/MEM stage
 wire [31:0] forward_data_wb = write_data_to_reg; // Data source from MEM/WB stage (result of WB mux)
 // ALU Input A Mux (Handles special cases AND Forwarding)
 wire [31:0] pc_ex = id_ex_pcplus4 - 4; // Reconstruct PC if needed for AUIPC
 wire [31:0] alu_in1_source_select; // Output of the special source mux
 assign alu_in1_source_select = (id_ex_alu_in1_src == 2'b01) ? pc_ex :
                                (id_ex_alu_in1_src == 2'b10) ? 32'b0 :
                                (id_ex_alu_in1_src == 2'b11) ? id_ex_pcplus4 :
                                id_ex_rs1_data; // Default uses rs1_data from ID/EX
// Forwarding Mux for ALU Input A (NEW)
 wire [31:0] forwarded_alu_in1;
 assign forwarded_alu_in1 = (ForwardA == 2'b00) ? alu_in1_source_select : // Use value from ID/EX (potentially muxed for LUI etc.)
                            (ForwardA == 2'b01) ? forward_data_wb :     // Forward from WB stage
                            (ForwardA == 2'b10) ? forward_data_mem :     // Forward from MEM stage
                            alu_in1_source_select; // Default (shouldn't happen with 2 bits)
// ALU Input B Mux (Handles Immediate vs Reg AND Forwarding)
 wire [31:0] alu_in2_source_select = id_ex_alusrc ? id_ex_immediate : id_ex_rs2_data; // Output of Imm/Reg mux

 // Forwarding Mux for ALU Input B (NEW)
 wire [31:0] forwarded_alu_in2;
 assign forwarded_alu_in2 = (ForwardB == 2'b00) ? alu_in2_source_select : // Use value from ID/EX (Imm or rs2_data)
                            (ForwardB == 2'b01) ? forward_data_wb :     // Forward from WB stage
                            (ForwardB == 2'b10) ? forward_data_mem :     // Forward from MEM stage
                            alu_in2_source_select; // Default
  // --- EX Stage Logic (uses outputs from ID/EX registers) ---
  // ALU Inputs Selection (Mux driven by registered alusrc_ctrl)
  wire [31:0] alu_input_b = id_ex_alusrc ? id_ex_immediate : id_ex_rs2_data;
  // Refinements needed here for LUI/AUIPC/JAL/JALR overriding alu_in1/alu_in2
wire [31:0] store_rs2_forwarded;
assign store_rs2_forwarded =
    (ForwardStore == 2'b01) ? forward_data_wb :
    (ForwardStore == 2'b10) ? forward_data_mem :
                          id_ex_rs2_data;

wire [31:0] csr_rdata_ex = csr_rdata;
wire [31:0] csr_operand = id_ex_is_csr_imm ? 
                          {27'b0, id_ex_rs1_addr} : 
                          forwarded_alu_in1;
                          reg [31:0] csr_wdata_computed;
always @(*) begin
    case (id_ex_funct3)
        3'b001, 3'b101: csr_wdata_computed = csr_operand;              // CSRRW/CSRRWI
        3'b010, 3'b110: csr_wdata_computed = csr_rdata_ex | csr_operand;  // CSRRS/CSRRSI
        3'b011, 3'b111: csr_wdata_computed = csr_rdata_ex & ~csr_operand; // CSRRC/CSRRCI
        default:        csr_wdata_computed = 32'b0;
    endcase
end
wire csr_write_cond = (id_ex_funct3[1:0] == 2'b01) || (id_ex_rs1_addr != 5'b0);
wire csr_should_write_ex = id_ex_is_csr && csr_write_cond;

    reg [31:0] ex_mem_alu_result;
    reg [31:0] ex_mem_rs2_data;
    reg [31:0] ex_mem_rd_addr;
    reg [2:0] ex_mem_funct3;
    reg ex_mem_zero_flag;
    reg ex_mem_alu_lt;              
reg ex_mem_alu_ltu;             
    reg ex_mem_mem_read;
    reg ex_mem_mem_write;
    reg ex_mem_reg_write;
    reg ex_mem_mem_to_reg;
    reg ex_mem_is_mret;
    reg [31:0] ex_mem_pcplus4;
    reg ex_mem_isJAL;
    reg ex_mem_isJALR;
    reg [31:0] ex_mem_branch_target;
    reg ex_mem_isBtype;
    reg [31:0] ex_mem_csr_wdata;     // Computed CSR write data
reg [11:0] ex_mem_csr_addr;      // CSR address
reg        ex_mem_is_csr;        // Is this a CSR instruction?
reg        ex_mem_csr_write;     // Should CSR be written?
reg [31:0] ex_mem_csr_rdata;     // CSR read data (for writeback)
reg        ex_mem_is_ecall;   
reg        ex_mem_is_ebreak;   
    wire is_beq  = (ex_mem_funct3 == 3'b000);  // Branch if equal
    wire is_bne  = (ex_mem_funct3 == 3'b001);  // Branch if not equal
    wire is_blt  = (ex_mem_funct3 == 3'b100);  // Branch if less than (signed)
    wire is_bge  = (ex_mem_funct3 == 3'b101);  // Branch if greater/equal (signed)
    wire is_bltu = (ex_mem_funct3 == 3'b110);  // Branch if less than (unsigned)
    wire is_bgeu = (ex_mem_funct3 == 3'b111);  // Branch if greater/equal (unsigned)

wire branch_taken = (is_beq  && ex_mem_zero_flag) ||   // Take if equal (zero)
                        (is_bne  && !ex_mem_zero_flag) ||  // Take if not equal
                        (is_blt  && ex_mem_alu_lt) ||       // Take if less than (signed)
                        (is_bge  && !ex_mem_alu_lt) ||      // Take if greater/equal (signed)
                        (is_bltu && ex_mem_alu_ltu) ||      // Take if less than (unsigned)
                        (is_bgeu && !ex_mem_alu_ltu);       // Take if greater/equal (unsigned)
reg [31:0] mem_wb_csr_rdata;     // CSR read data
reg [11:0] mem_wb_csr_addr;      // CSR address
reg        mem_wb_is_csr;        // Is this a CSR instruction?
reg [31:0] mem_wb_csr_wdata;     // CSR write data
reg        mem_wb_csr_write;     // CSR write enable
reg [31:0] mem_wb_pcplus4;
reg        mem_wb_isJAL;
reg        mem_wb_isJALR;
    reg [31:0] mem_wb_mem_data;    // Data read from memory (if Load)
  reg [31:0] mem_wb_alu_result;  // ALU result (passed from EX/MEM)
  reg [4:0]  mem_wb_rd_addr;     // Destination register address (passed from EX/MEM)
  // Control Signals passed from EX/MEM
  reg        mem_wb_reg_write;   // Register write enable
  reg        mem_wb_mem_to_reg;  // Writeback data mux select

  // Branching Logic (will use ALU result/flags in EX/MEM stage)
assign take_branch_condition = ex_mem_isBtype && branch_taken;
wire trap_taken = ex_mem_is_ecall || ex_mem_is_ebreak;
// Trap cause codes (defined in RISC-V spec)
wire [31:0] trap_cause = ex_mem_is_ebreak ? 32'd3 :   // Breakpoint
                         ex_mem_is_ecall  ? 32'd11 :  // Environment call from M-mode
                         32'd0;
wire [31:0] trap_pc = ex_mem_pcplus4 - 4;  // Reconstruct PC of trapping instruction

  reg [31:0] branch_addr;
  // PC Calculation Logic for branches/jumps (Part of EX stage)
  wire [31:0] jalr_target_calc = alu_result;

// --- MEM Stage Logic ---

  // 1. Determine access type based on funct3 from EX/MEM stage
  wire mem_byteAccess_mem     = (ex_mem_funct3[1:0] == 2'b00); // lb, lbu
  wire mem_halfwordAccess_mem = (ex_mem_funct3[1:0] == 2'b01); // lh, lhu

  // 2. Extract Halfword and Byte based on address LSBs from EX/MEM stage
  // If address[1] is 1, we want the upper 16 bits, else lower 16 bits.
  wire [15:0] LOAD_halfword_mem = ex_mem_alu_result[1] ? mem_rdata[31:16] : mem_rdata[15:0];
  // If address[0] is 1, we want the upper 8 bits of the selected halfword.
  wire [7:0]  LOAD_byte_mem     = ex_mem_alu_result[0] ? LOAD_halfword_mem[15:8] : LOAD_halfword_mem[7:0];

  // 3. Determine if we need sign extension.
  // RISC-V: funct3[2] is 1 for unsigned loads (LBU, LHU), 0 for signed (LB, LH).
  wire LOAD_sign_mem = !ex_mem_funct3[2];

  // 4. Format the final data
  wire [31:0] load_data_formatted =
      mem_byteAccess_mem     ? {{24{LOAD_sign_mem & LOAD_byte_mem[7]}}, LOAD_byte_mem} :
      mem_halfwordAccess_mem ? {{16{LOAD_sign_mem & LOAD_halfword_mem[15]}}, LOAD_halfword_mem} :
      mem_rdata; // Default to full word (LW); // Needs registered address LSBs from EX/MEM
wire is_instruction_fetch = (mem_addr == pc_reg);
  // Memory Interface signals (Control needed based on EX/MEM register values)
wire mem_access_in_mem_stage = ex_mem_mem_read || ex_mem_mem_write;

// INSTRUCTION MEMORY INTERFACE (for fetch)
assign imem_addr = pc_reg;  // Always fetch from PC
assign imem_rstrb = 1'b1;   // Always fetching

// DATA MEMORY INTERFACE (for loads/stores only)
assign mem_addr = ex_mem_alu_result;  // Data address from ALU
assign mem_rstrb = ex_mem_mem_read;   // Read strobe for loads
  
// Use instruction memory for fetches
wire [31:0] instruction_from_mem = imem_rdata; // Get instruction from imem interface
wire [31:0] aligned_store_data;
assign aligned_store_data = 
    (ex_mem_funct3 == 3'b010) ? ex_mem_rs2_data :              // SW
    (ex_mem_funct3 == 3'b001) ? {2{ex_mem_rs2_data[15:0]}} :   // SH
    {4{ex_mem_rs2_data[7:0]}};       
  assign mem_wdata = aligned_store_data;
  // Store byte enables based on funct3 and address
wire [3:0] store_byte_enables;
assign store_byte_enables = 
    (ex_mem_funct3 == 3'b010) ? 4'b1111 :                      // SW
    (ex_mem_funct3 == 3'b001) ?                                // SH
        (ex_mem_alu_result[1] ? 4'b1100 : 4'b0011) :
    (ex_mem_funct3 == 3'b000) ?                                // SB
        (ex_mem_alu_result[1] ? 
            (ex_mem_alu_result[0] ? 4'b1000 : 4'b0100) :
            (ex_mem_alu_result[0] ? 4'b0010 : 4'b0001)
        ) :
    4'b0000;
assign mem_wstrb = ex_mem_mem_write ? store_byte_enables : 4'b0000;
// CSR write signals (from WB stage)
assign csr_write_wb = mem_wb_csr_write;
assign csr_waddr_wb = mem_wb_csr_addr;
assign csr_wdata_wb = mem_wb_csr_wdata;
// Branch target calculation
 wire [31:0] branch_target_ex = pc_ex + id_ex_immediate;

// Update mem_wstrb assignment
//assign mem_wstrb = ex_mem_mem_write ? STORE_wmask_mem : 4'b0; // Use EX/MEM control signal
  // ******** Instantiate Modules ********
  reg_file reg_file_inst (
      .clk(clk),
      .rst(rst),
      .rs1(rs1_id), // Read address 1 from ID stage (registered instruction bits)
      .rs2(rs2_id), // Read address 2 from ID stage (registered instruction bits)
      .rd(rd_wb),   // Write addr comes from MEM/WB register output
      .write_data(write_data_to_reg), // Data comes from WB stage mux
      .write_en(reg_write_wb),       // Write enable comes from MEM/WB register output
      .rs1_data(rs1_data),   // Output read data 1 (ID stage)
      .rs2_data(rs2_data)    // Output read data 2 (ID stage)
  );

  alu alu_inst (
      .alu_in1(forwarded_alu_in1),   // Connect ALU input 1 (from ID/EX reg output)
      .alu_in2(forwarded_alu_in2),      // Connect ALU input 2 (from Mux using ID/EX reg outputs)
      .ALUOp(id_ex_alu_op),       // Connect ALU control signal (from ID/EX reg output)
      .alu_out(alu_result),       // Output wire for ALU result (EX stage)
      .zero_flag(zero_flag),      // Output wire for zero flag (EX stage)
        .alu_lt(alu_lt),               
        .alu_ltu(alu_ltu)              
  );

  initial begin
      cycle = 0; // Initialize cycle counter
  end
  wire [31:0] mtvec_value;
  wire [31:0] mepc_value;
  assign mtvec_value = csr_file_inst.mtvec;  // Direct access to CSR
  assign mepc_value = csr_file_inst.mepc;
  assign next_pc = 
  (trap_taken) ? mtvec_value :  
  (ex_mem_is_mret)  ? mepc_value :     // MRET returns to mepc
  (ex_mem_isJALR) ? (ex_mem_alu_result & 32'hFFFFFFFE) : // JALR target (mask LSB)
                 (take_branch_condition) ? ex_mem_branch_target :      // Taken Branch target
                 (ex_mem_isJAL) ? ex_mem_branch_target :          // JAL target
                 pcplus4_if;


  // Sequential Logic (Clocking PC, IF/ID, ID/EX Registers)
  always @(posedge clk or posedge rst) begin
      if (rst) begin
          // PC Reset
          pc_reg            <= 32'b0;

          // IF/ID Reset
          if_id_instruction <= 32'h00000013; // Reset to NOP
          if_id_pcplus4     <= 32'b0;
          // ID/EX Reset (Reset all fields)
          id_ex_pcplus4     <= 32'b0;
          id_ex_rs1_data    <= 32'b0;
          id_ex_rs2_data    <= 32'b0;
          id_ex_immediate   <= 32'b0;
          id_ex_rs1_addr    <= 5'b0;
          id_ex_rs2_addr    <= 5'b0;
          id_ex_rd_addr     <= 5'b0;
          id_ex_alu_op      <= `ALU_ADD; // Default/NOP op
          id_ex_alusrc      <= 1'b0;
          id_ex_mem_read    <= 1'b0;
          id_ex_mem_write   <= 1'b0;
          id_ex_reg_write   <= 1'b0;
          id_ex_mem_to_reg  <= 1'b0;
          id_ex_funct3      <= 3'b0;
          ex_mem_csr_wdata  <= 32'b0;
ex_mem_csr_addr   <= 12'b0;
ex_mem_is_csr     <= 1'b0;
ex_mem_csr_write  <= 1'b0;
ex_mem_csr_rdata  <= 32'b0;
        ex_mem_alu_lt     <= 1'b0;          
        ex_mem_alu_ltu    <= 1'b0;          
          ex_mem_alu_result <= 32'b0;
          ex_mem_rs2_data <= 32'b0;
          ex_mem_rd_addr <= 32'b0;
          ex_mem_zero_flag  <= 1'b0;
          ex_mem_isJAL <= 1'b0;
          ex_mem_isJALR <= 1'b0;
          ex_mem_pcplus4   <= 32'b0;
          mem_wb_mem_data   <= 32'b0;
          ex_mem_branch_target <= 32'b0;
          ex_mem_mem_to_reg <= 1'b0;
          ex_mem_mem_read <= 1'b0;
          ex_mem_mem_write <= 1'b0;
          ex_mem_reg_write <= 1'b0;
          ex_mem_is_mret <= 1'b0;
        mem_wb_alu_result <= 32'b0;
        mem_wb_rd_addr    <= 5'b0;
        mem_wb_reg_write  <= 1'b0;
        mem_wb_mem_to_reg <= 1'b0;
        mem_wb_pcplus4   <= 32'b0;
        mem_wb_csr_rdata <= 32'b0;
        mem_wb_csr_wdata <= 32'b0;
mem_wb_csr_addr  <= 12'b0;
mem_wb_csr_write <= 1'b0;
mem_wb_is_csr    <= 1'b0;
        id_ex_isJAL <= 1'b0;
        id_ex_isJALR <= 1'b0;
        ex_mem_isBtype <= 1'b0;
        id_ex_isBtype_reg <= 1'b0;
        id_ex_alu_in1_src <= 2'b00;
        debug_id_instruction  <= 32'h00000013;
          debug_ex_instruction  <= 32'h00000013;
          debug_mem_instruction <= 32'h00000013;
          debug_wb_instruction  <= 32'h00000013;
          id_ex_csr_addr    <= 12'b0;
          ex_mem_is_ecall   <= 1'b0;              
      ex_mem_is_ebreak  <= 1'b0;   
      id_ex_is_csr      <= 1'b0;
      id_ex_is_csr_imm  <= 1'b0;
      id_ex_is_ecall    <= 1'b0;
      id_ex_is_ebreak   <= 1'b0;
      id_ex_is_mret  <= 1'b0;
      end else begin
          // --- Clock PC ---
          pc_reg            <= next_pc;

          // --- Clock IF/ID Register ---
          if_id_instruction <= instruction_from_mem;
          if_id_pcplus4     <= pcplus4_if;

          // --- Clock ID/EX Register ---
          id_ex_pcplus4     <= if_id_pcplus4;     // Pass PC+4
          id_ex_rs1_data    <= rs1_data;          // Latch data read from reg file
          id_ex_rs2_data    <= rs2_data;          // Latch data read from reg file
          id_ex_immediate   <= immediate_id;      // Latch calculated immediate
          id_ex_rs1_addr    <= rs1_id;            // Pass rs1 address
          id_ex_rs2_addr    <= rs2_id;            // Pass rs2 address
          id_ex_rd_addr     <= rd_id;             // Pass destination register address
          id_ex_alu_op      <= alu_op_ctrl;       // Latch control signal
          id_ex_alusrc      <= alusrc_ctrl;       // Latch control signal
          id_ex_mem_read    <= mem_read_ctrl;     // Latch control signal
          id_ex_mem_write   <= mem_write_ctrl;    // Latch control signal
          id_ex_reg_write   <= reg_write_ctrl;    // Latch control signal
          id_ex_mem_to_reg  <= mem_to_reg_ctrl;   // Latch control signal
          id_ex_funct3      <= funct3_id;         // Pass funct3

          // -- Clock EX/MEM Register ---
          ex_mem_alu_result <= alu_result;
          ex_mem_rs2_data <= store_rs2_forwarded;
          ex_mem_rd_addr <= id_ex_rd_addr;
          ex_mem_zero_flag  <= zero_flag;
          ex_mem_csr_wdata  <= csr_wdata_computed;
          id_ex_csr_addr    <= csr_addr;           // Latch CSR address
id_ex_is_csr      <= is_csr_ctrl;        // Latch CSR flag
id_ex_is_csr_imm  <= funct3_id[2];       // Immediate variant flag
id_ex_is_ecall    <= is_ecall_ctrl;      // Latch ECALL flag
id_ex_is_ebreak   <= is_ebreak_ctrl;     // Latch EBREAK flag
ex_mem_csr_addr   <= id_ex_csr_addr;
ex_mem_is_csr     <= id_ex_is_csr;
ex_mem_csr_write  <= csr_should_write_ex;
ex_mem_csr_rdata  <= csr_rdata;
ex_mem_is_ecall   <= id_ex_is_ecall;    
ex_mem_is_ebreak  <= id_ex_is_ebreak;   
          // Pass control signals through
          ex_mem_alu_lt     <= alu_lt;            
ex_mem_alu_ltu    <= alu_ltu;           
          ex_mem_mem_read   <= id_ex_mem_read;
          ex_mem_mem_write  <= id_ex_mem_write;
          ex_mem_reg_write  <= id_ex_reg_write;
          ex_mem_mem_to_reg <= id_ex_mem_to_reg;
          ex_mem_funct3 <= id_ex_funct3;
          ex_mem_isBtype <= id_ex_isBtype_reg;
          id_ex_isBtype_reg <= is_branch_ctrl;
          id_ex_isJAL   <= is_jal_ctrl;
          id_ex_isJALR  <= is_jalr_ctrl;
          ex_mem_isJALR <= id_ex_isJALR;
          ex_mem_isJAL <= id_ex_isJAL;
          ex_mem_branch_target <= branch_target_ex;
          mem_wb_mem_data   <= mem_rdata;
id_ex_is_mret <= is_mret_ctrl;
ex_mem_is_mret <= id_ex_is_mret;
        // Pass through values from previous stage (EX/MEM)
        mem_wb_alu_result <= ex_mem_alu_result;
        mem_wb_rd_addr    <= ex_mem_rd_addr;
        mem_wb_reg_write  <= ex_mem_reg_write;
        mem_wb_mem_to_reg <= ex_mem_mem_to_reg;
mem_wb_csr_rdata  <= ex_mem_csr_rdata;
mem_wb_csr_wdata  <= ex_mem_csr_wdata;
mem_wb_csr_addr   <= ex_mem_csr_addr;
mem_wb_csr_write  <= ex_mem_csr_write;
mem_wb_is_csr     <= ex_mem_is_csr;
          // MEM/WB latches
          ex_mem_pcplus4 <= id_ex_pcplus4;
            mem_wb_pcplus4 <= ex_mem_pcplus4;
            mem_wb_isJAL   <= ex_mem_isJAL;
            mem_wb_isJALR  <= ex_mem_isJALR;
        
        id_ex_alu_in1_src <= ctrl_alu_in1_src;

        // DEBUG CLOCKING
        debug_id_instruction <= if_id_instruction; // Latch instruction leaving ID

          // --- Clock EX/MEM Register ---
          // ... (existing EX/MEM register clocking) ...
          debug_ex_instruction <= debug_id_instruction; // Pass instruction leaving EX

          // --- Clock MEM/WB Register ---
          // ... (existing MEM/WB register clocking) ...
          debug_mem_instruction <= debug_ex_instruction; // Pass instruction leaving MEM

          // --- Update WB Debug Register ---
          debug_wb_instruction <= debug_mem_instruction; // Pass instruction leaving WB
      end
  end
// --- Forwarding Unit (Combinational Logic) ---
 always @(*) begin
    // Default: No forwarding
    ForwardA = 2'b00;
    ForwardB = 2'b00;

    // EX/MEM Hazard Check (Check if MEM stage is writing to a register needed by EX stage)
    // Check if RegWrite is enabled for the instruction in MEM stage,
    // if the destination register (rd) is not x0,
    // and if it matches the source register (rs1 or rs2) needed by the instruction in EX stage.
    if (ex_mem_reg_write && (ex_mem_rd_addr != 5'b0)) begin
        if (ex_mem_rd_addr == id_ex_rs1_addr) begin
            ForwardA = 2'b10; // Forward ALU result from MEM stage to ALU input A
        end
        // Only forward to ALU input B if the instruction uses rs2 for ALU (not immediate)
        if ((ex_mem_rd_addr == id_ex_rs2_addr) && !id_ex_alusrc) begin
            ForwardB = 2'b10; // Forward ALU result from MEM stage to ALU input B
        end
    end

    // MEM/WB Hazard Check (Check if WB stage is writing to a register needed by EX stage)
    // Check if RegWrite is enabled for the instruction in WB stage,
    // if the destination register (rd) is not x0,
    // and if it matches the source register (rs1 or rs2) needed by the instruction in EX stage.
    // **Crucially, only forward from WB if MEM didn't already forward for the same register**
    //    (MEM stage result is newer/more relevant than WB stage result for EX stage).
    if (mem_wb_reg_write && (mem_wb_rd_addr != 5'b0)) begin
        if ((mem_wb_rd_addr == id_ex_rs1_addr) && !(ex_mem_reg_write && (ex_mem_rd_addr != 5'b0) && (ex_mem_rd_addr == id_ex_rs1_addr))) begin
            // Forward only if MEM stage isn't already writing to the same rs1
            ForwardA = 2'b01; // Forward Writeback data from WB stage to ALU input A
        end
        // Only forward to ALU input B if the instruction uses rs2 for ALU (not immediate)
        if ((mem_wb_rd_addr == id_ex_rs2_addr) && !id_ex_alusrc && !(ex_mem_reg_write && (ex_mem_rd_addr != 5'b0) && (ex_mem_rd_addr == id_ex_rs2_addr))) begin
             // Forward only if MEM stage isn't already writing to the same rs2
            ForwardB = 2'b01; // Forward Writeback data from WB stage to ALU input B
        end
    end
 end
 
 // Separate forwarding for store data (always check rs2 for stores, regardless of alusrc)
 reg [1:0] ForwardStore;
 always @(*) begin
    ForwardStore = 2'b00; // Default: no forwarding
    
    // Check if EX/MEM stage is writing to rs2
    if (ex_mem_reg_write && (ex_mem_rd_addr != 5'b0) && (ex_mem_rd_addr == id_ex_rs2_addr)) begin
        ForwardStore = 2'b10; // Forward from MEM stage
    end
    // Check if MEM/WB stage is writing to rs2 (lower priority than EX/MEM)
    else if (mem_wb_reg_write && (mem_wb_rd_addr != 5'b0) && (mem_wb_rd_addr == id_ex_rs2_addr)) begin
        ForwardStore = 2'b01; // Forward from WB stage
    end
 end
  // Cycle counter
  always @(posedge clk or posedge rst) begin
    if(rst)
      cycle <= 0;
    else begin
        cycle <= cycle + 1;
    end
  end
    wire [31:0] wb_data =
    (mem_wb_isJAL | mem_wb_isJALR) ? mem_wb_pcplus4 :
    (mem_wb_mem_to_reg           ) ? mem_wb_mem_data :
                                    mem_wb_alu_result;
    // --- WB Stage Logic ---
    // Placeholder wires for MEM/WB outputs
    wire [4:0]  rd_wb = mem_wb_rd_addr;         // rd addr from MEM/WB reg
    wire        reg_write_wb = mem_wb_reg_write;  // RegWrite signal from MEM/WB reg
    wire        mem_to_reg_wb = mem_wb_mem_to_reg; // MemToReg signal from MEM/WB reg

    // Register file write back data mux (Combinational - WB stage)
assign write_data_to_reg = mem_wb_is_csr ? mem_wb_csr_rdata :
                          (mem_wb_mem_to_reg ? mem_wb_mem_data : wb_data);
// Extended debug monitoring
always @(posedge clk) begin
  if (!rst && cycle >= 1 && cycle <= 9) begin
    $display("\n========== Cycle %0d ==========", cycle);
    $display("PC=0x%08x  next_pc=0x%08x", pc_reg, next_pc);
    $display("\nIF/ID:");
    $display("  instruction=0x%08x  pcplus4=0x%08x", if_id_instruction, if_id_pcplus4);
    
    $display("\nID (Decode):");
    $display("  opcode=%b rd=%d rs1=%d rs2=%d funct3=%b", opcode_id, rd_id, rs1_id, rs2_id, funct3_id);
    $display("  rs1_data=0x%08x rs2_data=0x%08x (from reg file)", rs1_data, rs2_data);
    $display("  immediate=0x%08x", immediate_id);
    $display("  ctrl: alu_op=%d alusrc=%b mem_read=%b mem_write=%b reg_write=%b",
             alu_op_ctrl, alusrc_ctrl, mem_read_ctrl, mem_write_ctrl, reg_write_ctrl);
    
    $display("\nID/EX:");
    $display("  rs1_data=0x%08x rs2_data=0x%08x immediate=0x%08x", 
             id_ex_rs1_data, id_ex_rs2_data, id_ex_immediate);
    $display("  rs1_addr=%d rs2_addr=%d rd_addr=%d", id_ex_rs1_addr, id_ex_rs2_addr, id_ex_rd_addr);
    
    $display("\nEX (Execute):");
    $display("  ForwardA=%b ForwardB=%b", ForwardA, ForwardB);
    $display("  alu_in1=0x%08x alu_in2=0x%08x", forwarded_alu_in1, forwarded_alu_in2);
    $display("  alu_result=0x%08x zero=%b", alu_result, zero_flag);
    
    $display("\nEX/MEM:");
    $display("  alu_result=0x%08x rs2_data=0x%08x rd_addr=%d", 
             ex_mem_alu_result, ex_mem_rs2_data, ex_mem_rd_addr);
    $display("  mem_read=%b mem_write=%b reg_write=%b mem_to_reg=%b", 
             ex_mem_mem_read, ex_mem_mem_write, ex_mem_reg_write, ex_mem_mem_to_reg);
    
    $display("\nMEM:");
    $display("  mem_addr=0x%08x mem_wdata=0x%08x mem_wstrb=%b mem_rstrb=%b",
             mem_addr, mem_wdata, mem_wstrb, mem_rstrb);
    
    $display("\nMEM/WB:");
    $display("  alu_result=0x%08x mem_data=0x%08x", mem_wb_alu_result, mem_wb_mem_data);
    $display("  rd_addr=%d reg_write=%b mem_to_reg=%b", mem_wb_rd_addr, mem_wb_reg_write, mem_wb_mem_to_reg);
    $display("  write_data_to_reg=0x%08x", write_data_to_reg);
    
    // Show register file state for x1 and x2
    $display("\nRegister File: x0=0x%08x x1=0x%08x x2=0x%08x", 
             reg_file_inst.regfile[0], reg_file_inst.regfile[1], reg_file_inst.regfile[2]);
  end
end

endmodule