// =============================================================================
// Simple AXI4 RAM Slave — for bridge testing in simulation
// =============================================================================
// Single-beat AXI4 slave with configurable read latency.
// No bursts, no caching. Matches the Zynq HP port behavior.
// =============================================================================

module axi_ram_model #(
    parameter RAM_SIZE = 32'h2400000,  // 36MB
    parameter READ_LATENCY = 3        // cycles from ARVALID+ARREADY to RVALID
) (
    input  wire        clk,
    input  wire        rst,

    // AXI4 Slave Interface
    input  wire [3:0]  S_AXI_AWID,
    input  wire [31:0] S_AXI_AWADDR,
    input  wire        S_AXI_AWVALID,
    output reg         S_AXI_AWREADY,

    input  wire [31:0] S_AXI_WDATA,
    input  wire [3:0]  S_AXI_WSTRB,
    input  wire        S_AXI_WVALID,
    output reg         S_AXI_WREADY,

    output reg  [3:0]  S_AXI_BID,
    output wire [1:0]  S_AXI_BRESP,
    output reg         S_AXI_BVALID,
    input  wire        S_AXI_BREADY,

    input  wire [3:0]  S_AXI_ARID,
    input  wire [31:0] S_AXI_ARADDR,
    input  wire        S_AXI_ARVALID,
    output reg         S_AXI_ARREADY,

    output reg  [3:0]  S_AXI_RID,
    output reg  [31:0] S_AXI_RDATA,
    output wire [1:0]  S_AXI_RRESP,
    output wire        S_AXI_RLAST,
    output reg         S_AXI_RVALID,
    input  wire        S_AXI_RREADY
);

    assign S_AXI_BRESP = 2'b00;  // OKAY
    assign S_AXI_RRESP = 2'b00;  // OKAY
    assign S_AXI_RLAST = 1'b1;   // Always last (single beat)

    // RAM array
    localparam RAM_WORDS = RAM_SIZE / 4;
    reg [31:0] ram [0:RAM_WORDS-1];

    integer i;
    initial begin
        for (i = 0; i < RAM_WORDS; i = i + 1)
            ram[i] = 32'h0;
    end

    // Read pipeline
    reg [7:0]  rd_delay_cnt;
    reg        rd_pending;
    reg [3:0]  rd_id;
    reg [31:0] rd_data;

    // Write state
    reg        aw_received;
    reg        w_received;
    reg [3:0]  wr_id;
    reg [31:0] wr_addr;
    reg [31:0] wr_data;
    reg [3:0]  wr_strb;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            S_AXI_AWREADY <= 1'b1;
            S_AXI_WREADY  <= 1'b1;
            S_AXI_BVALID  <= 1'b0;
            S_AXI_ARREADY <= 1'b1;
            S_AXI_RVALID  <= 1'b0;
            S_AXI_RDATA   <= 32'h0;
            S_AXI_RID     <= 4'h0;
            S_AXI_BID     <= 4'h0;
            rd_delay_cnt  <= 0;
            rd_pending    <= 1'b0;
            aw_received   <= 1'b0;
            w_received    <= 1'b0;
        end else begin
            // ---- Read path ----
            if (S_AXI_RVALID && S_AXI_RREADY) begin
                S_AXI_RVALID <= 1'b0;
                S_AXI_ARREADY <= 1'b1;
            end

            if (S_AXI_ARVALID && S_AXI_ARREADY && !rd_pending) begin
                // Accept read request
                S_AXI_ARREADY <= 1'b0;
                rd_pending    <= 1'b1;
                rd_delay_cnt  <= READ_LATENCY;
                rd_id         <= S_AXI_ARID;
                // Read data immediately (RAM is synchronous but we delay the response)
                if (S_AXI_ARADDR[31:28] == 4'h8 && ((S_AXI_ARADDR - 32'h80000000) >> 2) < RAM_WORDS)
                    rd_data <= ram[(S_AXI_ARADDR - 32'h80000000) >> 2];
                else
                    rd_data <= 32'hDEADBEEF;
            end

            if (rd_pending) begin
                if (rd_delay_cnt > 0) begin
                    rd_delay_cnt <= rd_delay_cnt - 8'd1;
                end else begin
                    S_AXI_RVALID <= 1'b1;
                    S_AXI_RDATA  <= rd_data;
                    S_AXI_RID    <= rd_id;
                    rd_pending   <= 1'b0;
                end
            end

            // ---- Write path ----
            if (S_AXI_BVALID && S_AXI_BREADY) begin
                S_AXI_BVALID  <= 1'b0;
                S_AXI_AWREADY <= 1'b1;
                S_AXI_WREADY  <= 1'b1;
                aw_received   <= 1'b0;
                w_received    <= 1'b0;
            end

            if (S_AXI_AWVALID && S_AXI_AWREADY) begin
                aw_received <= 1'b1;
                wr_id       <= S_AXI_AWID;
                wr_addr     <= S_AXI_AWADDR;
                S_AXI_AWREADY <= 1'b0;
            end

            if (S_AXI_WVALID && S_AXI_WREADY) begin
                w_received <= 1'b1;
                wr_data    <= S_AXI_WDATA;
                wr_strb    <= S_AXI_WSTRB;
                S_AXI_WREADY <= 1'b0;
            end

            // Both address and data received — commit write and respond
            if ((aw_received || (S_AXI_AWVALID && S_AXI_AWREADY)) &&
                (w_received  || (S_AXI_WVALID  && S_AXI_WREADY))) begin : wr_commit
                reg [31:0] wa, wd;
                reg [3:0]  ws;
                integer widx;
                wa = aw_received ? wr_addr : S_AXI_AWADDR;
                wd = w_received  ? wr_data : S_AXI_WDATA;
                ws = w_received  ? wr_strb : S_AXI_WSTRB;
                if (wa[31:28] == 4'h8) begin
                    widx = (wa - 32'h80000000) >> 2;
                    if (widx < RAM_WORDS) begin
                        if (ws[0]) ram[widx][7:0]   <= wd[7:0];
                        if (ws[1]) ram[widx][15:8]  <= wd[15:8];
                        if (ws[2]) ram[widx][23:16] <= wd[23:16];
                        if (ws[3]) ram[widx][31:24] <= wd[31:24];
                    end
                end
                S_AXI_BVALID <= 1'b1;
                S_AXI_BID    <= aw_received ? wr_id : S_AXI_AWID;
            end
        end
    end
endmodule
