// Minimal FPGA top for a unified I+D memory bus CPU
// Exposes only clock, reset button, and 4 LEDs.
// - Instantiates your cpu.v (unmodified)
// - Instantiates a unified BRAM (byte-write) preloaded with fw/firmware.mem
// - Heartbeat LED proves clock/reset are good

module top_fpga (
    input  wire CLK100MHZ,       // board clock (e.g., 100 MHz)
    input  wire rst_n,     // active-low reset button
//    output wire uart_rxd_out,  // FPGA -> PC (TX)
//    input  wire uart_txd_in,    // PC  -> FPGA (RX)
    output wire [3:0] led // simple heartbeat/debug
);

  // --------------------------------------------------------------------------
  // Reset: synchronize and invert active-low button
  // --------------------------------------------------------------------------
  reg [1:0] rst_sync;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) rst_sync <= 2'b11;
    else        rst_sync <= {1'b0, rst_sync[1]};
  end
  wire rst = rst_sync[0];
  // Clock
  wire clk = CLK100MHZ;
  // --------------------------------------------------------------------------
  // Wires between CPU and unified memory
  // Your cpu.v prototype (from your earlier snippet) looked like:
  //   input  [31:0] mem_rdata
  //   output [31:0] mem_addr
  //   output [31:0] mem_wdata
  //   output        mem_rstrb
  //   output [3:0]  mem_wstrb
  // Adjust names below if they differ.
  // --------------------------------------------------------------------------
  wire [31:0] mem_rdata;
  wire [31:0] mem_addr;
  wire [31:0] mem_wdata;
  wire        mem_rstrb;
  wire [3:0]  mem_wstrb;
  wire        mem_en = mem_rstrb | (|mem_wstrb);   // simple enable
  // --------------------------------------------------------------------------
  // CPU instance
  // Make sure your cpu.v has clk/rst ports named like this; rename if needed.
  // --------------------------------------------------------------------------
  cpu u_cpu (
        .rst(rst), .clk(clk),
        .mem_rdata(mem_rdata),
        .mem_addr(mem_addr),
        .mem_rstrb(mem_rstrb),
        .mem_wdata(mem_wdata),
        .mem_wstrb(mem_wstrb)
  );

  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // IMPORTANT: The above port list must match your actual cpu.v exactly.
  // Replace with your real port names and remove duplicates.
  // If your cpu already has a mem_wstrb output we wired above, don't declare it twice.
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  // --------------------------------------------------------------------------
  // Unified memory (initialized from fw/firmware.mem)
  // Convert byte address to word address for BRAM by dropping low 2 bits.
  // Depth: 16 KB default (AW=12 -> 4K words * 4 bytes)
  // --------------------------------------------------------------------------
  localparam AW = 12; // 16 KB = 2^12 words * 4 bytes
  wire [AW-1:0] addr_word = mem_addr[AW+1:2];

  progmem mem0(
            .rst(rst), .clk(clk),
            .addr(addr_word),
            .data_in(mem_wdata),
            .rd_strobe(mem_rstrb),
            .wr_strobe(mem_wstrb),
            .data_out(mem_rdata)
          );

  // --------------------------------------------------------------------------
  // Heartbeat + simple debug LEDs
  // --------------------------------------------------------------------------
  reg [25:0] hb;
  always @(posedge clk) begin
    if (rst) hb <= 0;
    else     hb <= hb + 1'b1;
  end

  assign led[0] = hb[23];          // slow blink ~1.5 Hz @ 100 MHz
  assign led[1] = |mem_wstrb;      // lights on any store
  assign led[2] = mem_rstrb;       // lights when CPU requests a read
  assign led[3] = ~rst;            // on when out of reset

endmodule
`default_nettype wire