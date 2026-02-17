// =============================================================================
// RISC-V SoC Top Level
// =============================================================================
// Integrates:
//   - RV32IMA CPU with MMU
//   - UART (16550-compatible) at 0x10000000
//   - CLINT at 0x02000000 (inside CPU)
//   - PLIC at 0x0C000000 (inside CPU)
//   - RAM at 0x80000000 (8MB default)
//
// Memory Map:
//   0x02000000 - 0x0200FFFF : CLINT (timer) - handled in CPU
//   0x0C000000 - 0x0FFFFFFF : PLIC - handled in CPU
//   0x10000000 - 0x10000007 : UART
//   0x80000000 - 0x807FFFFF : RAM (8MB default, configurable)
// =============================================================================

`ifndef SYNTHESIS
`include "../src/cpu.v"
`include "../src/uart.v"
`endif

module soc #(
    parameter RAM_SIZE = 32'h800000,      // 8MB default for Linux boot
    parameter RAM_INIT_FILE = ""          // Optional memory init file
) (
    input         clk,
    input         rst,
    
    // UART external pins
    output        uart_tx,
    input         uart_rx,
    
    // Debug outputs
    output [31:0] debug_pc,
    output [31:0] debug_cycle
);

    // =========================================================================
    // CPU Interface Wires
    // =========================================================================
    
    // Instruction memory interface
    wire [31:0] imem_addr;
    wire [31:0] imem_rdata;
    wire        imem_rstrb;
    
    // Data memory interface
    wire [31:0] mem_addr;
    wire [31:0] mem_rdata;
    wire [31:0] mem_wdata;
    wire        mem_rstrb;
    wire [3:0]  mem_wstrb;
    
    wire [31:0] cycle;
    
    // =========================================================================
    // Address Decoding
    // =========================================================================
    
    // UART: 0x10000000 - 0x1000001F (8 registers * 4 bytes = 32 bytes)
    wire uart_select = (mem_addr[31:5] == 27'h0800000);
    
    // RAM: 0x80000000+
    wire ram_select = (mem_addr[31:28] == 4'h8);
    
    // Instruction fetch is always from RAM (0x80000000+)
    wire imem_ram_select = (imem_addr[31:28] == 4'h8);
    
    // =========================================================================
    // UART Instance
    // =========================================================================
    
    wire [31:0] uart_rdata;
    wire        uart_addr_valid;
    wire        uart_irq;
    
    // Simulation interface
    wire [7:0]  uart_sim_tx_data;
    wire        uart_sim_tx_valid;
    reg  [7:0]  uart_sim_rx_data;
    reg         uart_sim_rx_valid;
    
    uart #(
        .BASE_ADDR(32'h10000000),
        .FIFO_DEPTH(16)
    ) uart_inst (
        .clk(clk),
        .rst(rst),
        
        // Memory interface
        .addr(mem_addr),
        .wdata(mem_wdata),
        .wstrb(uart_select ? mem_wstrb : 4'b0),
        .read_en(uart_select && mem_rstrb),
        .rdata(uart_rdata),
        .addr_valid(uart_addr_valid),
        
        // Interrupt
        .uart_irq(uart_irq),
        
        // External pins
        .tx_out(uart_tx),
        .rx_in(uart_rx),
        
        // Simulation interface
        .sim_tx_data(uart_sim_tx_data),
        .sim_tx_valid(uart_sim_tx_valid),
        .sim_rx_data(uart_sim_rx_data),
        .sim_rx_valid(uart_sim_rx_valid)
    );
    
    // UART TX console output disabled - testbench handles output capture
    // reg uart_tx_valid_prev;
    // always @(posedge clk or posedge rst) begin ... end
    
    // Initialize simulation RX
    `ifdef SIMULATION
    initial begin
        uart_sim_rx_data = 8'h0;
        uart_sim_rx_valid = 1'b0;
    end
    `endif
    
    // =========================================================================
    // RAM Instance (Dual-port: instruction + data)
    // =========================================================================
    
    wire [31:0] ram_imem_rdata;
    wire [31:0] ram_dmem_rdata;
    
    // RAM address width (number of word address bits)
    // For 8KB: RAM_AW = 11 (2048 words), for 8MB: RAM_AW = 21
    localparam RAM_AW = $clog2(RAM_SIZE/4);

    // Convert byte address to word address for RAM
    // Bit 31 is always set for RAM addresses (0x80000000+), so just use lower bits
    wire [RAM_AW-1:0] imem_word_addr = imem_addr[RAM_AW+1:2];
    wire [RAM_AW-1:0] dmem_word_addr = mem_addr[RAM_AW+1:2];
    
    // RAM array
    // Distributed RAM (LUT RAM) — combinational reads are incompatible with BRAM
    // In simulation, pad to power-of-2 to prevent out-of-bounds array access.
    // RAM_AW may address more entries than RAM_SIZE/4 for non-power-of-2 sizes;
    // 2-state simulators compile reg arrays to C++ arrays where OOB is UB.
    `ifdef SIMULATION
    localparam RAM_ALLOC = (1 << RAM_AW);
    `else
    localparam RAM_ALLOC = (RAM_SIZE / 4);
    `endif
    (* ram_style = "distributed" *) reg [31:0] ram [0:RAM_ALLOC-1];

    // Initialize RAM
    integer i;
    initial begin
        `ifdef SIMULATION
        for (i = 0; i < RAM_ALLOC; i = i + 1) begin
            ram[i] = 32'h00000000;  // Zero - required for .bss
        end
        if (RAM_INIT_FILE != "") begin
            $readmemh(RAM_INIT_FILE, ram);
        end
        `else
        // Synthesis: provide a firmware_init.vh with ram[N] = 32'hXXXXXXXX assignments
        // or set RAM_INIT_FILE parameter for $readmemh
        if (RAM_INIT_FILE != "") begin
            $readmemh(RAM_INIT_FILE, ram);
        end
        `endif
    end
    
    // Instruction read (port A) - always read
    // Address masking via truncated wire width ensures we stay in bounds
    assign ram_imem_rdata = ram[imem_word_addr];

    // Data read (port B)
    assign ram_dmem_rdata = ram[dmem_word_addr];

    // Data write with byte enables
    always @(posedge clk) begin
        if (ram_select && (mem_wstrb != 4'b0)) begin
            if (mem_wstrb[0]) ram[dmem_word_addr][7:0]   <= mem_wdata[7:0];
            if (mem_wstrb[1]) ram[dmem_word_addr][15:8]  <= mem_wdata[15:8];
            if (mem_wstrb[2]) ram[dmem_word_addr][23:16] <= mem_wdata[23:16];
            if (mem_wstrb[3]) ram[dmem_word_addr][31:24] <= mem_wdata[31:24];
        end
    end
    
    // =========================================================================
    // Memory Read Mux
    // =========================================================================
    
    // Instruction memory always from RAM
    assign imem_rdata = ram_imem_rdata;
    
    // Data memory mux based on address
    assign mem_rdata = uart_select ? uart_rdata :
                       ram_select  ? ram_dmem_rdata :
                       32'h0;
    
    // =========================================================================
    // CPU Instance
    // =========================================================================
    
    cpu cpu_inst (
        .clk(clk),
        .rst(rst),

        // Instruction memory
        .imem_addr(imem_addr),
        .imem_rdata(imem_rdata),
        .imem_rstrb(imem_rstrb),

        // Data memory
        .mem_addr(mem_addr),
        .mem_rdata(mem_rdata),
        .mem_wdata(mem_wdata),
        .mem_rstrb(mem_rstrb),
        .mem_wstrb(mem_wstrb),

        .cycle(cycle),

        // External interrupts: UART is IRQ source 1 (per DTS)
        .external_irq_sources({30'b0, uart_irq, 1'b0})
    );
    
    // =========================================================================
    // Debug Outputs
    // =========================================================================
    
    assign debug_pc = imem_addr;
    assign debug_cycle = cycle;

endmodule