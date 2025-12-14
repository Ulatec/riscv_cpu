`timescale 1ns / 1ps
`include "../src/cpu.v"

// Testbench for MRET instruction
module mret_testbench;

    // Clock and reset
    reg clk;
    reg rst;
    
    // Instruction memory interface
    wire [31:0] imem_addr;
    reg  [31:0] imem_rdata;
    wire        imem_rstrb;
    
    // Data memory interface
    reg  [31:0] mem_rdata;
    wire [31:0] mem_addr;
    wire [31:0] mem_wdata;
    wire        mem_rstrb;
    wire [3:0]  mem_wstrb;
    
    // Debug outputs
    wire [31:0] cycle;

    // Instruction memory array
    reg [31:0] imem [0:1023];
    
    // Data memory array
    reg [31:0] dmem [0:255];
    
    // Instantiate CPU
    cpu dut (
        .clk(clk),
        .rst(rst),
        .imem_addr(imem_addr),
        .imem_rdata(imem_rdata),
        .imem_rstrb(imem_rstrb),
        .mem_rdata(mem_rdata),
        .mem_addr(mem_addr),
        .mem_wdata(mem_wdata),
        .mem_rstrb(mem_rstrb),
        .mem_wstrb(mem_wstrb),
        .cycle(cycle)
    );
    
    // Clock generation - 10ns period (100MHz)
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end
    
    // Instruction memory read
    always @(*) begin
        imem_rdata = imem[imem_addr[31:2]];
    end
    
    // Data memory read/write
    always @(posedge clk) begin
        if (mem_wstrb[0]) dmem[mem_addr[31:2]][7:0]   <= mem_wdata[7:0];
        if (mem_wstrb[1]) dmem[mem_addr[31:2]][15:8]  <= mem_wdata[15:8];
        if (mem_wstrb[2]) dmem[mem_addr[31:2]][23:16] <= mem_wdata[23:16];
        if (mem_wstrb[3]) dmem[mem_addr[31:2]][31:24] <= mem_wdata[31:24];
    end
    
    always @(*) begin
        mem_rdata = dmem[mem_addr[31:2]];
    end
    
    // Test program
    integer i;
    initial begin
        // Initialize ALL memory to NOPs first
        for (i = 0; i < 1024; i = i + 1) begin
            imem[i] = 32'h00000013;  // NOP
        end
        
        // ========== Main Program ==========
        // Address 0x00: Set up trap handler
        imem[0]  = 32'h10000F93;  // addi t6, x0, 0x100  (t6 = 0x100)
        imem[1]  = 32'h305F9073;  // csrw mtvec, t6      (mtvec = 0x100)
        
        // Address 0x08: Set a marker register before trap
        imem[2]  = 32'h00A00093;  // addi x1, x0, 10     (x1 = 10, marker before trap)
        
        // Address 0x0C: NOPs for pipeline
        imem[3]  = 32'h00000013;  // nop
        imem[4]  = 32'h00000013;  // nop
        
        // Address 0x14: Trigger ECALL (this should trap to 0x100)
        imem[5]  = 32'h00000073;  // ecall               (PC = 0x14)
        
        // Address 0x18: This executes AFTER returning from trap handler
        imem[6]  = 32'h01400113;  // addi x2, x0, 20     (x2 = 20, marker after return)
        
        // Address 0x1C: Another marker to confirm continued execution
        imem[7]  = 32'h01E00193;  // addi x3, x0, 30     (x3 = 30)
        
        // Address 0x20: End marker
        imem[8]  = 32'h02800213;  // addi x4, x0, 40     (x4 = 40)
        
        // ========== Trap Handler (at 0x100) ==========
        // Address 0x100: Save context / do trap work
        imem[64] = 32'h06400293;  // addi x5, x0, 100    (x5 = 100, trap handler marker)
        
        // Address 0x104: Read mepc to verify it was saved
        imem[65] = 32'h34102373;  // csrr x6, mepc       (x6 = mepc, should be 0x14)
        
        // Address 0x108: Read mcause to verify
        imem[66] = 32'h342023F3;  // csrr x7, mcause     (x7 = mcause, should be 11)
        
        // Address 0x10C: Increment mepc by 4 to skip ecall on return
        imem[67] = 32'h00430313;  // addi x6, x6, 4      (x6 = mepc + 4)
        imem[68] = 32'h34131073;  // csrw mepc, x6       (mepc = 0x18)
        
        // Address 0x114: NOPs before MRET
        imem[69] = 32'h00000013;  // nop
        imem[70] = 32'h00000013;  // nop
        imem[71] = 32'h00000013;  // nop
        
        // Address 0x120: Execute MRET to return from trap
        imem[72] = 32'h30200073;  // mret                (PC = mepc = 0x18)
        
        // Address 0x124: Should never execute (MRET jumps away)
        imem[73] = 32'hFF000413;  // addi x8, x0, -16    (x8 = -16, error marker)
        
        // Initialize data memory
        for (i = 0; i < 256; i = i + 1) begin
            dmem[i] = 32'h00000000;
        end
    end
    
    // Test monitoring
    integer pass_count;
    integer fail_count;
    
    initial begin
        pass_count = 0;
        fail_count = 0;
        
        // Reset
        rst = 1;
        #20;
        rst = 0;
        
        $display("\n╔══════════════════════════════════════════════════════╗");
        $display("║           MRET INSTRUCTION TESTBENCH                 ║");
        $display("╚══════════════════════════════════════════════════════╝\n");
        
        // ===== Test 1: Verify trap is taken =====
        $display("Test 1: ECALL triggers trap");
        $display("─────────────────────────────────────────────");
        
        wait_for_pc(32'h00000100);
        $display("  ✓ PC jumped to trap handler (0x100)");
        pass_count = pass_count + 1;
        
        #20;
        check_csr(12'h341, 32'h00000014, "mepc = 0x14 (PC of ECALL)");
        check_csr(12'h342, 32'd11, "mcause = 11 (ECALL)");
        
        $display("");
        
        // ===== Test 2: Verify MRET returns correctly =====
        $display("Test 2: MRET returns to mepc");
        $display("─────────────────────────────────────────────");
        
        // Wait for MRET to execute and return to 0x18
        wait_for_pc(32'h00000018);
        $display("  ✓ PC returned to 0x18 (mepc after increment)");
        pass_count = pass_count + 1;
        
        $display("");
        
        // ===== Test 3: Verify execution continues after return =====
        $display("Test 3: Execution continues after MRET");
        $display("─────────────────────────────────────────────");
        
        // Let a few more instructions execute
        #100;
        
        check_register(5, 32'd100, "x5 = 100 (trap handler executed)");
        check_register(2, 32'd20, "x2 = 20 (instruction after ECALL executed)");
        check_register(3, 32'd30, "x3 = 30 (continued execution)");
        check_register(8, 32'h00000000, "x8 = 0 (instruction after MRET NOT executed)");
        
        $display("");
        
        // ===== Test 4: Verify mstatus was updated =====
        $display("Test 4: mstatus updated by trap/MRET");
        $display("─────────────────────────────────────────────");
        
        // After MRET: MIE should be restored, MPIE should be 1
        check_mstatus_mie(1'b0, "MIE restored (was 0 before trap)");
        check_mstatus_mpie(1'b1, "MPIE set to 1 after MRET");
        
        $display("");
        
        // ===== Summary =====
        $display("╔══════════════════════════════════════════════════════╗");
        $display("║              TEST SUMMARY                            ║");
        $display("╚══════════════════════════════════════════════════════╝");
        $display("");
        $display("Tests Passed: %0d", pass_count);
        $display("Tests Failed: %0d", fail_count);
        $display("");
        
        if (fail_count == 0) begin
            $display("  ╔═══════════════════════════════════════════════════╗");
            $display("  ║  ✓ MRET IMPLEMENTATION VERIFIED!                  ║");
            $display("  ║    Trap handlers can now return to normal code.   ║");
            $display("  ╚═══════════════════════════════════════════════════╝");
            $display("");
        end else begin
            $display("  ⚠️  SOME TESTS FAILED - Review above for details");
            $display("");
        end
        
        $finish;
    end
    
    // Helper task to wait for specific PC
    task wait_for_pc;
        input [31:0] target_pc;
        integer timeout;
        begin
            timeout = 0;
            while (dut.pc_reg != target_pc && timeout < 200) begin
                @(posedge clk);
                timeout = timeout + 1;
            end
            if (timeout >= 200) begin
                $display("  ✗ TIMEOUT waiting for PC=0x%08h (stuck at 0x%08h)", target_pc, dut.pc_reg);
                fail_count = fail_count + 1;
            end
        end
    endtask
    
    // Helper task to check register value
    task check_register;
        input [4:0] reg_num;
        input [31:0] expected;
        input [200*8:0] test_name;
        reg [31:0] actual;
        begin
            actual = dut.reg_file_inst.regfile[reg_num];
            $display("  %s", test_name);
            if (actual == expected) begin
                $display("    Expected: 0x%08h, Got: 0x%08h ✓ PASS", expected, actual);
                pass_count = pass_count + 1;
            end else begin
                $display("    Expected: 0x%08h, Got: 0x%08h ✗ FAIL", expected, actual);
                fail_count = fail_count + 1;
            end
        end
    endtask
    
    // Helper task to check CSR value
    task check_csr;
        input [11:0] csr_addr;
        input [31:0] expected;
        input [200*8:0] test_name;
        reg [31:0] actual;
        begin
            case (csr_addr)
                12'h300: actual = dut.csr_file_inst.mstatus;
                12'h341: actual = dut.csr_file_inst.mepc;
                12'h342: actual = dut.csr_file_inst.mcause;
                12'h340: actual = dut.csr_file_inst.mscratch;
                12'h305: actual = dut.csr_file_inst.mtvec;
                default: actual = 32'hDEADBEEF;
            endcase
            
            $display("  %s", test_name);
            if (actual == expected) begin
                $display("    Expected: 0x%08h, Got: 0x%08h ✓ PASS", expected, actual);
                pass_count = pass_count + 1;
            end else begin
                $display("    Expected: 0x%08h, Got: 0x%08h ✗ FAIL", expected, actual);
                fail_count = fail_count + 1;
            end
        end
    endtask
    
    // Helper task to check mstatus.MIE bit
    task check_mstatus_mie;
        input expected;
        input [200*8:0] test_name;
        reg actual;
        begin
            actual = dut.csr_file_inst.mstatus[3];
            $display("  %s", test_name);
            if (actual == expected) begin
                $display("    Expected: %b, Got: %b ✓ PASS", expected, actual);
                pass_count = pass_count + 1;
            end else begin
                $display("    Expected: %b, Got: %b ✗ FAIL", expected, actual);
                fail_count = fail_count + 1;
            end
        end
    endtask
    
    // Helper task to check mstatus.MPIE bit
    task check_mstatus_mpie;
        input expected;
        input [200*8:0] test_name;
        reg actual;
        begin
            actual = dut.csr_file_inst.mstatus[7];
            $display("  %s", test_name);
            if (actual == expected) begin
                $display("    Expected: %b, Got: %b ✓ PASS", expected, actual);
                pass_count = pass_count + 1;
            end else begin
                $display("    Expected: %b, Got: %b ✗ FAIL", expected, actual);
                fail_count = fail_count + 1;
            end
        end
    endtask
    
    // Waveform dump
    initial begin
        $dumpfile("mret_test.vcd");
        $dumpvars(0, mret_testbench);
    end
    
    // Timeout
    initial begin
        #10000;
        $display("\n⚠️  ERROR: Testbench timeout!");
        $display("  PC stuck at: 0x%08h", dut.pc_reg);
        $finish;
    end

endmodule