`timescale 1ns/1ps

// Include the CPU and UART (fpga_soc.v expects them pre-included)
`include "../src/cpu.v"
`include "../src/uart.v"
`include "../fpga/fpga_soc.v"
`include "../fpga/axi_mem_bridge.v"
`include "axi_ram_model.v"

// ============================================================================
// Linux Boot with ACTUAL AXI Bridge + AXI RAM Model
// Tests bridge logic in simulation — if doubled chars appear here,
// it's a bridge LOGIC bug (not timing). If clean, it's a timing issue.
// ============================================================================
module linux_bridge_tb;
    parameter RAM_SIZE = 32'h2400000;  // 36MB
    parameter DTB_ADDR = 32'h82200000;
    parameter MAX_CYCLES = 300_000_000;
    parameter AXI_READ_LATENCY = 1;   // Simulated DDR read latency (low for speed)

    reg clk, rst;
    wire uart_tx;
    wire [31:0] debug_pc, debug_cycle;
    wire [1:0]  debug_priv;
    wire [2:0]  debug_axi_state;

    // AXI wires between fpga_soc and axi_ram_model
    wire [3:0]  axi_awid, axi_arid, axi_bid, axi_rid;
    wire [31:0] axi_awaddr, axi_araddr, axi_wdata, axi_rdata;
    wire [7:0]  axi_awlen, axi_arlen;
    wire [2:0]  axi_awsize, axi_arsize;
    wire [1:0]  axi_awburst, axi_arburst, axi_bresp, axi_rresp;
    wire        axi_awlock, axi_arlock;
    wire [3:0]  axi_awcache, axi_arcache, axi_wstrb, axi_awqos, axi_arqos;
    wire [2:0]  axi_awprot, axi_arprot;
    wire        axi_awvalid, axi_awready;
    wire        axi_wvalid, axi_wready, axi_wlast;
    wire        axi_bvalid, axi_bready;
    wire        axi_arvalid, axi_arready;
    wire        axi_rvalid, axi_rready, axi_rlast;

    fpga_soc dut (
        .clk(clk), .rst(rst),
        .uart_tx(uart_tx), .uart_rx(1'b1),
        .debug_pc(debug_pc), .debug_cycle(debug_cycle),
        .debug_priv(debug_priv),
        .debug_axi_state(debug_axi_state),
        .debug_axi_arvalid_ever(), .debug_axi_arready_ever(), .debug_axi_rd_complete(),
        .debug_first_trap_cause(), .debug_trap_captured(), .debug_first_trap_pc(),
        .M_AXI_AWID(axi_awid), .M_AXI_AWADDR(axi_awaddr), .M_AXI_AWLEN(axi_awlen),
        .M_AXI_AWSIZE(axi_awsize), .M_AXI_AWBURST(axi_awburst), .M_AXI_AWLOCK(axi_awlock),
        .M_AXI_AWCACHE(axi_awcache), .M_AXI_AWPROT(axi_awprot), .M_AXI_AWQOS(axi_awqos),
        .M_AXI_AWVALID(axi_awvalid), .M_AXI_AWREADY(axi_awready),
        .M_AXI_WDATA(axi_wdata), .M_AXI_WSTRB(axi_wstrb), .M_AXI_WLAST(axi_wlast),
        .M_AXI_WVALID(axi_wvalid), .M_AXI_WREADY(axi_wready),
        .M_AXI_BID(axi_bid), .M_AXI_BRESP(axi_bresp),
        .M_AXI_BVALID(axi_bvalid), .M_AXI_BREADY(axi_bready),
        .M_AXI_ARID(axi_arid), .M_AXI_ARADDR(axi_araddr), .M_AXI_ARLEN(axi_arlen),
        .M_AXI_ARSIZE(axi_arsize), .M_AXI_ARBURST(axi_arburst), .M_AXI_ARLOCK(axi_arlock),
        .M_AXI_ARCACHE(axi_arcache), .M_AXI_ARPROT(axi_arprot), .M_AXI_ARQOS(axi_arqos),
        .M_AXI_ARVALID(axi_arvalid), .M_AXI_ARREADY(axi_arready),
        .M_AXI_RID(axi_rid), .M_AXI_RDATA(axi_rdata), .M_AXI_RRESP(axi_rresp),
        .M_AXI_RLAST(axi_rlast), .M_AXI_RVALID(axi_rvalid), .M_AXI_RREADY(axi_rready)
    );

    axi_ram_model #(
        .RAM_SIZE(RAM_SIZE),
        .READ_LATENCY(AXI_READ_LATENCY)
    ) ram_model (
        .clk(clk), .rst(rst),
        .S_AXI_AWID(axi_awid), .S_AXI_AWADDR(axi_awaddr),
        .S_AXI_AWVALID(axi_awvalid), .S_AXI_AWREADY(axi_awready),
        .S_AXI_WDATA(axi_wdata), .S_AXI_WSTRB(axi_wstrb),
        .S_AXI_WVALID(axi_wvalid), .S_AXI_WREADY(axi_wready),
        .S_AXI_BID(axi_bid), .S_AXI_BRESP(axi_bresp),
        .S_AXI_BVALID(axi_bvalid), .S_AXI_BREADY(axi_bready),
        .S_AXI_ARID(axi_arid), .S_AXI_ARADDR(axi_araddr),
        .S_AXI_ARVALID(axi_arvalid), .S_AXI_ARREADY(axi_arready),
        .S_AXI_RID(axi_rid), .S_AXI_RDATA(axi_rdata), .S_AXI_RRESP(axi_rresp),
        .S_AXI_RLAST(axi_rlast), .S_AXI_RVALID(axi_rvalid), .S_AXI_RREADY(axi_rready)
    );

    initial clk = 0;
    always #5 clk = ~clk;

    wire [31:0] pc = dut.cpu_inst.pc_reg;
    wire [1:0]  priv = dut.cpu_inst.csr_file_inst.priv_level;
    wire [31:0] satp_val = dut.cpu_inst.csr_file_inst.satp;
    wire uart_tx_valid = dut.uart_inst.sim_tx_valid;
    wire [7:0] uart_tx_data = dut.uart_inst.sim_tx_data;
    wire trap_taken = dut.cpu_inst.csr_file_inst.trap_taken;
    wire [31:0] trap_cause = dut.cpu_inst.csr_file_inst.trap_cause;

    integer i, uart_count, page_faults;
    reg [31:0] prev_satp, pt_base_word;
    reg entered_smode, saw_kernel_virt;
    reg [7:0] prev_char;
    reg saw_shell_prompt;
    integer shell_cycle;
    integer rela_idx, rela_applied, scan;
    reg [31:0] r_offset, r_info, r_addend, rela_start, rela_end;
    reg found_rela;

    initial begin
        rst = 1; uart_count = 0; page_faults = 0;
        entered_smode = 0; prev_satp = 0; saw_kernel_virt = 0;
        prev_char = 0; saw_shell_prompt = 0; shell_cycle = 0;

        $display("=== Linux Boot with ACTUAL AXI Bridge (latency=%0d) ===", AXI_READ_LATENCY);
        $display("RAM: %0d MB", RAM_SIZE/(1024*1024));

        // Load firmware into AXI RAM model
        for (i = 0; i < RAM_SIZE/4; i = i + 1)
            ram_model.ram[i] = 32'h0;
        $readmemh("../dts/fw_payload_earlycon.hex", ram_model.ram, 0);
        $readmemh("../dts/riscv_soc_dtb_le.hex", ram_model.ram, (DTB_ADDR - 32'h80000000) / 4);

        // Auto-detect and apply relocations
        found_rela = 0;
        for (scan = 32'h8000; scan < 32'h20000 && !found_rela; scan = scan + 1) begin
            if ((ram_model.ram[scan+1] & 32'hFF) == 32'h03 &&
                (ram_model.ram[scan+4] & 32'hFF) == 32'h03 &&
                (ram_model.ram[scan+7] & 32'hFF) == 32'h03 &&
                ram_model.ram[scan] < RAM_SIZE) begin
                rela_start = scan; found_rela = 1;
            end
        end
        if (found_rela) begin
            rela_end = rela_start;
            for (scan = rela_start; scan < rela_start + 32'h3000; scan = scan + 3)
                if ((ram_model.ram[scan+1] & 32'hFF) == 32'h03 && ram_model.ram[scan] < RAM_SIZE)
                    rela_end = scan + 3;
                else scan = rela_start + 32'h3000;
            rela_applied = 0;
            for (rela_idx = rela_start; rela_idx < rela_end; rela_idx = rela_idx + 3) begin
                r_offset = ram_model.ram[rela_idx];
                r_info = ram_model.ram[rela_idx+1];
                r_addend = ram_model.ram[rela_idx+2];
                if ((r_info & 32'hFF) == 32'h03) begin
                    ram_model.ram[r_offset >> 2] = 32'h80000000 + r_addend;
                    rela_applied = rela_applied + 1;
                end
            end
            $display("Relocations: %0d applied", rela_applied);
        end
        $fflush();

        #1000; rst = 0;
        dut.cpu_inst.reg_file_inst.regfile[10] = 0;
        dut.cpu_inst.reg_file_inst.regfile[11] = DTB_ADDR;

        for (i = 0; i < MAX_CYCLES; i = i + 1) begin
            @(posedge clk);

            // PTE injection
            if (satp_val != prev_satp && satp_val[31]) begin
                pt_base_word = ({satp_val[21:0], 12'b0} - 32'h80000000) >> 2;
                ram_model.ram[pt_base_word + 32'h040] = 32'h040000C7;
                ram_model.ram[pt_base_word + 32'h008] = 32'h008000C7;
                ram_model.ram[pt_base_word + 32'h030] = 32'h030000C7;
                prev_satp = satp_val;
            end else if (satp_val != prev_satp) prev_satp = satp_val;

            if (priv == 2'b01 && !entered_smode) begin
                entered_smode = 1;
                $display("[%0d] ENTERED S-MODE", i); $fflush();
            end
            if (!saw_kernel_virt && pc >= 32'hC0000000) begin
                saw_kernel_virt = 1;
                $display("[%0d] KERNEL VIRTUAL pc=0x%08x", i, pc); $fflush();
            end
            if (trap_taken && (trap_cause == 12 || trap_cause == 13 || trap_cause == 15))
                page_faults = page_faults + 1;

            if (uart_tx_valid) begin
                uart_count = uart_count + 1;
                $write("%c", uart_tx_data);
                if (uart_tx_data == 8'h0A || uart_count % 64 == 0) $fflush();
                if ((prev_char == 8'h23 || prev_char == 8'h24) && uart_tx_data == 8'h20) begin
                    if (!saw_shell_prompt) begin
                        saw_shell_prompt = 1; shell_cycle = i;
                        $display("\n[%0d] *** SHELL PROMPT ***", i); $fflush();
                    end
                end
                prev_char = uart_tx_data;
            end

            if (saw_shell_prompt && (i - shell_cycle) > 2_000_000) begin
                $display("\n[%0d] Shell detected, stopping", i); i = MAX_CYCLES;
            end
            if (i % 5_000_000 == 0 && i > 0) begin
                $display("\n[%0dM] pc=0x%08x priv=%0d uart=%0d pgf=%0d",
                    i/1000000, pc, priv, uart_count, page_faults); $fflush();
            end
        end

        $display("\n=== RESULTS (AXI bridge, latency=%0d) ===", AXI_READ_LATENCY);
        $display("UART: %0d chars  PGF: %0d  Shell: %s",
            uart_count, page_faults, saw_shell_prompt ? "YES" : "NO");
        $fflush(); $finish;
    end
endmodule
