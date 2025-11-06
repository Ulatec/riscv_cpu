module fpga_top (
  input  wire clk_100mhz,      // board clock
  input  wire btn_reset_n,     // active-low pushbutton
  output wire [3:0] led        // simple heartbeat & status
);
  wire rst = ~btn_reset_n;

  // CPU <-> memory bus
  wire [31:0] mem_addr;
  wire [31:0] mem_wdata;
  wire [31:0] mem_rdata;
  wire        mem_rstrb;       // read enable
  wire [3:0]  mem_wstrb;       // byte write enables

  // optional: cycle counter out of the CPU is internal, so we show a heartbeat here
  reg [25:0] hb;
  always @(posedge clk_100mhz) begin
    if (rst) hb <= 0;
    else     hb <= hb + 1;
  end
  assign led = hb[25:22];

  // ---- your CPU ----
  cpu u_cpu (
    .rst(rst),
    .clk(clk_100mhz),
    .mem_rdata(mem_rdata),
    .mem_addr(mem_addr),
    .mem_wdata(mem_wdata),
    .mem_rstrb(mem_rstrb),
    .cycle(),               // unused here
    .mem_wstrb(mem_wstrb)
  );

  // ---- unified I+D simple memory ----
  // Depth = 16 KB (4096 words) — adjust as needed
  bram_32k u_mem (
    .clk(clk_100mhz),
    .rst(rst),
    .raddr(mem_addr[15:2]),   // word address
    .rdata(mem_rdata),
    .re(mem_rstrb),
    .waddr(mem_addr[15:2]),
    .wdata(mem_wdata),
    .we(mem_wstrb)
  );

endmodule