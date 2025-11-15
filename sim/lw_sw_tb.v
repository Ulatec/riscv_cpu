`timescale 1ns/1ps
`include "cpu.v"
module lw_sw_tb;

  reg clk = 0;
  reg rst = 1;

  // clock: 100 MHz
  always #5 clk = ~clk;

  // CPU <-> instruction memory
  wire [31:0] imem_addr;
  reg  [31:0] imem_rdata;
  wire        imem_rstrb;

  // CPU <-> data memory
  wire [31:0] mem_addr;
  wire [31:0] mem_wdata;
  wire        mem_rstrb;
  wire [3:0]  mem_wstrb;
  reg  [31:0] mem_rdata;

  // Debug
  wire [31:0] cycle;

  // DUT
  cpu dut (
    .clk        (clk),
    .rst        (rst),
    // Instruction memory
    .imem_addr  (imem_addr),
    .imem_rdata (imem_rdata),
    .imem_rstrb (imem_rstrb),
    // Data memory
    .mem_rdata  (mem_rdata),
    .mem_addr   (mem_addr),
    .mem_wdata  (mem_wdata),
    .mem_rstrb  (mem_rstrb),
    .mem_wstrb  (mem_wstrb),
    .cycle      (cycle)
  );

  // Unified memory (256 words = 1 KB)
  reg [31:0] mem [0:255];

  localparam DATA_WORD = 16;  // word index for "data" label

  // Initialize memory
  integer i;
  integer idx;
  initial begin
    // Default: NOPs everywhere
    for (i = 0; i < 256; i = i + 1)
      mem[i] = 32'h00000013;

    // Program:
    mem[0] = 32'h04000093; // addi x1, x0, 64      ; x1 = &data (64 = 0x40)
    mem[1] = 32'h0000A103; // lw   x2, 0(x1)       ; x2 = mem[data]  
    mem[2] = 32'h00000013; // nop  (addi x0,x0,0)  ; bubble
    mem[3] = 32'h00110113; // addi x2, x2, 1       ; x2++
    mem[4] = 32'h0020A023; // sw   x2, 0(x1)       ; mem[data] = x2
    mem[5] = 32'h0000006F; // jal  x0, 0           ; infinite loop
    
    // Data
    mem[DATA_WORD] = 32'h12345678;  // At byte address 0x40 (word 16)
    
    $display("Program loaded:");
    $display("  mem[0]  = 0x%08x (addi x1, x0, 64)", mem[0]);
    $display("  mem[1]  = 0x%08x (lw x2, 0(x1))", mem[1]);
    $display("  mem[16] = 0x%08x (data at addr 0x40)", mem[16]);
  end

  // Combinational read for instruction memory (no conflicts)
  always @(*) begin
    imem_rdata = mem[imem_addr[9:2]];
  end

  // Combinational read for data memory
  always @(*) begin
    if (mem_rstrb)
      mem_rdata = mem[mem_addr[9:2]];
    else
      mem_rdata = 32'h00000000;
  end

  // Synchronous write for data memory
  always @(posedge clk) begin
    if (|mem_wstrb) begin
      idx = mem_addr[9:2];
      $display("Cycle %0d: WRITE addr=0x%08x [%0d] data=0x%08x wstrb=%b", 
               cycle, mem_addr, idx, mem_wdata, mem_wstrb);

      if (mem_wstrb[0]) mem[idx][7:0]   <= mem_wdata[7:0];
      if (mem_wstrb[1]) mem[idx][15:8]  <= mem_wdata[15:8];
      if (mem_wstrb[2]) mem[idx][23:16] <= mem_wdata[23:16];
      if (mem_wstrb[3]) mem[idx][31:24] <= mem_wdata[31:24];
    end
  end

  // Test control
  initial begin
    $dumpfile("test.vcd");
    $dumpvars;
    
    #100 rst = 0;  // Release reset
    #500;          // Run for a while
    
    $display("\n=== TEST RESULTS ===");
    $display("Final mem[16] = 0x%08x (expected 0x12345679)", mem[DATA_WORD]);
    
    if (mem[DATA_WORD] == 32'h12345679) begin
      $display("*** LW/SW TEST PASS ***");
    end else begin
      $display("*** LW/SW TEST FAIL ***");
    end
    
    $finish;
  end

endmodule