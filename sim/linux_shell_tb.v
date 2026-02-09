`timescale 1ns/1ps
`include "../src/soc.v"

// ============================================================================
// Linux Shell Boot Testbench
// Boots OpenSBI + Linux kernel and waits for shell prompt
// Auto-detects relocation table, doesn't depend on specific kernel symbols
// ============================================================================
module linux_shell_tb;
    parameter RAM_SIZE = 32'h2400000;  // 36MB
    parameter DTB_ADDR = 32'h82200000;
    parameter MAX_CYCLES = 300_000_000;
    parameter MAX_UART = 100000;

    reg clk, rst;
    wire uart_tx;
    reg uart_rx;
    wire [31:0] debug_pc, debug_cycle;

    soc #(.RAM_SIZE(RAM_SIZE)) dut (
        .clk(clk), .rst(rst),
        .uart_tx(uart_tx), .uart_rx(uart_rx),
        .debug_pc(debug_pc), .debug_cycle(debug_cycle)
    );

    initial clk = 0;
    always #5 clk = ~clk;  // 100MHz for faster simulation

    wire [31:0] pc = dut.cpu_inst.pc_reg;
    wire [1:0]  priv = dut.cpu_inst.csr_file_inst.priv_level;
    wire [31:0] satp_val = dut.cpu_inst.csr_file_inst.satp;

    // UART output
    wire uart_tx_valid = dut.uart_inst.sim_tx_valid;
    wire [7:0] uart_tx_data = dut.uart_inst.sim_tx_data;

    // Trap tracking
    wire trap_taken = dut.cpu_inst.csr_file_inst.trap_taken;
    wire [31:0] trap_cause = dut.cpu_inst.csr_file_inst.trap_cause;

    integer i, uart_count, page_faults;
    reg [31:0] prev_satp, pt_base_word;
    reg entered_smode;
    reg saw_kernel_virt;  // PC in 0xC0xxxxxx range (kernel virtual)

    // Shell detection: look for "# " prompt
    reg [7:0] prev_char;
    reg saw_shell_prompt;
    integer shell_cycle;

    // Relocation
    integer rela_idx, rela_applied, scan;
    reg [31:0] r_offset, r_info, r_addend;
    reg [31:0] rela_start, rela_end;
    reg found_rela;

    initial begin
        rst = 1; uart_rx = 1;
        uart_count = 0; page_faults = 0;
        entered_smode = 0; prev_satp = 0;
        saw_kernel_virt = 0;
        prev_char = 0;
        saw_shell_prompt = 0;
        shell_cycle = 0;

        $display("=== Linux Shell Boot Test ===");
        $display("RAM: %0d MB  Max cycles: %0dM", RAM_SIZE/(1024*1024), MAX_CYCLES/1000000);

        // Clear RAM
        for (i = 0; i < RAM_SIZE/4; i = i + 1)
            dut.ram[i] = 32'h0;

        // Load firmware and DTB
        $readmemh("../dts/fw_payload_earlycon.hex", dut.ram, 0);
        $readmemh("../dts/riscv_soc_dtb_le.hex", dut.ram, (DTB_ADDR - 32'h80000000) / 4);

        // Auto-detect relocation table
        // Scan for 3+ consecutive R_RISCV_RELATIVE entries (r_info & 0xFF == 0x03)
        found_rela = 0;
        rela_start = 0;
        rela_end = 0;
        for (scan = 32'h8000; scan < 32'h20000 && !found_rela; scan = scan + 1) begin
            if ((dut.ram[scan + 1] & 32'hFF) == 32'h03 &&
                (dut.ram[scan + 4] & 32'hFF) == 32'h03 &&
                (dut.ram[scan + 7] & 32'hFF) == 32'h03 &&
                dut.ram[scan] < RAM_SIZE &&
                dut.ram[scan + 3] < RAM_SIZE) begin
                rela_start = scan;
                found_rela = 1;
            end
        end

        if (found_rela) begin
            // Find end of relocation table
            rela_end = rela_start;
            for (scan = rela_start; scan < rela_start + 32'h3000; scan = scan + 3) begin
                if ((dut.ram[scan + 1] & 32'hFF) == 32'h03 && dut.ram[scan] < RAM_SIZE)
                    rela_end = scan + 3;
                else
                    scan = rela_start + 32'h3000; // break
            end

            // Apply relocations
            rela_applied = 0;
            for (rela_idx = rela_start; rela_idx < rela_end; rela_idx = rela_idx + 3) begin
                r_offset = dut.ram[rela_idx];
                r_info   = dut.ram[rela_idx + 1];
                r_addend = dut.ram[rela_idx + 2];
                if ((r_info & 32'hFF) == 32'h03) begin
                    dut.ram[r_offset >> 2] = 32'h80000000 + r_addend;
                    rela_applied = rela_applied + 1;
                end
            end
            $display("Relocation table at word 0x%x, applied %0d relocations", rela_start, rela_applied);
        end else begin
            $display("WARNING: No relocation table found (scanning 0x8000-0x20000)");
        end

        $fflush();

        #1000; rst = 0;
        dut.cpu_inst.reg_file_inst.regfile[10] = 0;       // a0 = hart id
        dut.cpu_inst.reg_file_inst.regfile[11] = DTB_ADDR; // a1 = DTB address

        for (i = 0; i < MAX_CYCLES; i = i + 1) begin
            @(posedge clk);

            // ======== PTE injection for identity-mapped MMIO ========
            if (satp_val != prev_satp && satp_val[31]) begin
                pt_base_word = ({satp_val[21:0], 12'b0} - 32'h80000000) >> 2;
                // UART: 0x10000000 -> PTE index 0x040 (VPN[1] = 0x040)
                dut.ram[pt_base_word + 32'h040] = 32'h040000C7;
                // CLINT: 0x02000000 -> PTE index 0x008
                dut.ram[pt_base_word + 32'h008] = 32'h008000C7;
                // PLIC: 0x0C000000 -> PTE index 0x030
                dut.ram[pt_base_word + 32'h030] = 32'h030000C7;
                prev_satp = satp_val;
            end else if (satp_val != prev_satp) begin
                prev_satp = satp_val;
            end

            // ======== S-mode entry ========
            if (priv == 2'b01 && !entered_smode) begin
                entered_smode = 1;
                $display("[%0d] ENTERED S-MODE pc=0x%08x", i, pc);
                $fflush();
            end

            // ======== Kernel virtual address space ========
            if (!saw_kernel_virt && pc >= 32'hC0000000) begin
                saw_kernel_virt = 1;
                $display("[%0d] KERNEL VIRTUAL pc=0x%08x (MMU active)", i, pc);
                $fflush();
            end

            // ======== Page faults (just count, don't spam) ========
            if (trap_taken && (trap_cause == 12 || trap_cause == 13 || trap_cause == 15)) begin
                page_faults = page_faults + 1;
                if (page_faults <= 3)
                    $display("[%0d] PGF #%0d cause=%0d pc=0x%08x stval=0x%08x",
                        i, page_faults, trap_cause,
                        dut.cpu_inst.csr_file_inst.trap_pc,
                        dut.cpu_inst.csr_file_inst.trap_val);
            end

            // ======== UART output ========
            if (uart_tx_valid) begin
                uart_count = uart_count + 1;
                $write("%c", uart_tx_data);
                if (uart_tx_data == 8'h0A || uart_count % 64 == 0) $fflush();

                // Detect shell prompt "# " or "$ "
                if ((prev_char == 8'h23 || prev_char == 8'h24) && uart_tx_data == 8'h20) begin
                    if (!saw_shell_prompt) begin
                        saw_shell_prompt = 1;
                        shell_cycle = i;
                        $display("\n[%0d] *** SHELL PROMPT DETECTED ***", i);
                        $fflush();
                    end
                end
                prev_char = uart_tx_data;

                if (uart_count >= MAX_UART) begin
                    $display("\n[%0d] UART limit reached (%0d chars)", i, uart_count);
                    i = MAX_CYCLES; // force exit
                end
            end

            // ======== Exit after shell prompt + 2M more cycles ========
            if (saw_shell_prompt && (i - shell_cycle) > 2_000_000) begin
                $display("\n[%0d] Shell detected, stopping after grace period", i);
                i = MAX_CYCLES;
            end

            // ======== Progress report every 5M cycles ========
            if (i % 5_000_000 == 0 && i > 0) begin
                $display("\n[%0dM] pc=0x%08x priv=%0d uart=%0d pgf=%0d",
                    i/1000000, pc, priv, uart_count, page_faults);
                $fflush();
            end
        end

        $display("\n\n=== RESULTS ===");
        $display("Total cycles:    %0d", i);
        $display("UART chars:      %0d", uart_count);
        $display("Page faults:     %0d", page_faults);
        $display("Entered S-mode:  %s", entered_smode ? "YES" : "NO");
        $display("Kernel virtual:  %s", saw_kernel_virt ? "YES" : "NO");
        $display("Shell prompt:    %s", saw_shell_prompt ? "YES" : "NO");
        $fflush();
        $finish;
    end
endmodule
