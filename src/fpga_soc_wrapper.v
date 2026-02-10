// =============================================================================
// FPGA SoC Wrapper for Digilent Arty S7-25
// =============================================================================
// Top-level module for FPGA synthesis. Contains:
//   - MMCM: 100MHz input -> 50MHz system clock
//   - Reset synchronizer with MMCM lock
//   - SoC instantiation with 8KB RAM
//   - LED status indicators
// =============================================================================

module fpga_soc_wrapper (
    input         CLK100MHZ,   // 100MHz SSTL135 from board oscillator
    input         rst_n,       // Active-low reset button (btn[0], G15)
    output        uart_tx,     // FPGA -> USB-UART bridge (V12)
    input         uart_rx,     // USB-UART bridge -> FPGA (R12)
    output [3:0]  led          // Status LEDs
);

    // =========================================================================
    // Clock Generation (MMCM: 100MHz -> 20MHz)
    // =========================================================================
    wire clk_out;
    wire clk_fb;
    wire mmcm_locked;

    MMCME2_BASE #(
        .CLKIN1_PERIOD  (10.0),    // 100MHz input -> 10ns period
        .CLKFBOUT_MULT_F(10.0),   // VCO = 100 * 10 = 1000MHz
        .CLKOUT0_DIVIDE_F(50.0),  // 1000 / 50 = 20MHz
        .STARTUP_WAIT   ("FALSE")
    ) mmcm_inst (
        .CLKIN1   (CLK100MHZ),
        .CLKFBIN  (clk_fb),
        .CLKFBOUT (clk_fb),
        .CLKOUT0  (clk_out),
        .LOCKED   (mmcm_locked),
        .PWRDWN   (1'b0),
        .RST      (1'b0)
    );

    // Buffer the output clock
    wire sys_clk;
    BUFG clk_buf (
        .I(clk_out),
        .O(sys_clk)
    );

    // =========================================================================
    // Reset Synchronizer
    // =========================================================================
    // 4-stage shift register, waits for MMCM lock before releasing reset.
    // Active-high reset internally.
    reg [3:0] rst_sync = 4'b1111;  // Start in reset
    wire sys_rst = rst_sync[3];    // Synchronized active-high reset

    always @(posedge sys_clk) begin
        if (!rst_n || !mmcm_locked)
            rst_sync <= 4'b1111;
        else
            rst_sync <= {rst_sync[2:0], 1'b0};
    end

    // =========================================================================
    // SoC Instance
    // =========================================================================
    wire [31:0] debug_pc;
    wire [31:0] debug_cycle;

    soc #(
        .RAM_SIZE      (32'h1000),        // 4KB for FPGA
        .RAM_INIT_FILE ("firmware.hex")   // Bare-metal firmware
    ) soc_inst (
        .clk         (sys_clk),
        .rst         (sys_rst),
        .uart_tx     (uart_tx),
        .uart_rx     (uart_rx),
        .debug_pc    (debug_pc),
        .debug_cycle (debug_cycle)
    );

    // =========================================================================
    // LED Status Indicators
    // =========================================================================

    // led[0]: Heartbeat (~0.6Hz blink from 20MHz clock)
    reg [24:0] heartbeat_cnt;
    always @(posedge sys_clk or posedge sys_rst) begin
        if (sys_rst)
            heartbeat_cnt <= 0;
        else
            heartbeat_cnt <= heartbeat_cnt + 1;
    end
    assign led[0] = heartbeat_cnt[24];  // ~1.49Hz toggle

    // led[1]: Out of reset (lit when running)
    assign led[1] = !sys_rst;

    // led[2]: PC has advanced beyond reset vector (0x80000000)
    reg pc_advanced;
    always @(posedge sys_clk or posedge sys_rst) begin
        if (sys_rst)
            pc_advanced <= 1'b0;
        else if (debug_pc != 32'h80000000 && debug_pc != 32'h0)
            pc_advanced <= 1'b1;
    end
    assign led[2] = pc_advanced;

    // led[3]: Cycle counter activity (blinks at ~cycle[22] rate)
    assign led[3] = debug_cycle[22];

endmodule
