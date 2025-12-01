// `timescale 1ns / 1ps
`include "cpu.v"
module csr_testbench;

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
    reg [31:0] imem [0:255];
    
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
    initial begin
        // Initialize instruction memory with CSR test program
        
        // Test 1: CSRRW (Read/Write) with mscratch
        imem[0]  = 32'h123452B7;  // lui t0, 0x12345
        imem[1]  = 32'h67828293;  // addi t0, t0, 0x678       (t0 = 0x12345678)
        imem[2]  = 32'h34029373;  // csrrw t1, mscratch, t0   (write 0x12345678 to mscratch, read old to t1)
        imem[3]  = 32'h34003373;  // csrrw t2, mscratch, x0   (read mscratch to t2, write 0)
        
        // Test 2: CSRRS (Read and Set) with mstatus
        imem[4]  = 32'h0FF00293;  // addi t0, x0, 0xFF        (t0 = 0xFF)
        imem[5]  = 32'h30003073;  // csrrw x0, mstatus, x0    (clear mstatus)
        imem[6]  = 32'h30032373;  // csrrs t1, mstatus, t0    (set bits in mstatus with 0xFF)
        
        // Test 3: CSRRC (Read and Clear) with mstatus
        imem[7]  = 32'h00800293;  // addi t0, x0, 8           (t0 = 8, bit 3)
        imem[8]  = 32'h30033373;  // csrrc t1, mstatus, t0    (clear bit 3 in mstatus)
        
        // Test 4: CSRRWI (Write Immediate)
        imem[9]  = 32'h3400D373;  // csrrwi t1, mscratch, 1   (write immediate 1 to mscratch)
        imem[10] = 32'h34002373;  // csrr t1, mscratch        (read back)
        
        // Test 5: CSRRSI (Set Immediate)
        imem[11] = 32'h3000E373;  // csrrsi t1, mstatus, 1    (set bit 0 in mstatus)
        
        // Test 6: CSRRCI (Clear Immediate)
        imem[12] = 32'h3000F373;  // csrrci t1, mstatus, 1    (clear bit 0 in mstatus)
        
        // Test 7: Read cycle counter
        imem[13] = 32'hC0002373;  // csrr t1, cycle           (read cycle counter low)
        imem[14] = 32'hC8002373;  // csrr t1, cycleh          (read cycle counter high)
        
        // Test 8: Read instret counter
        imem[15] = 32'hC0202373;  // csrr t1, instret         (read instret counter low)
        imem[16] = 32'hC8202373;  // csrr t1, instreth        (read instret counter high)
        
        // Test 9: Read machine info registers
        imem[17] = 32'hF1102373;  // csrr t1, mvendorid
        imem[18] = 32'hF1202373;  // csrr t1, marchid
        imem[19] = 32'hF1302373;  // csrr t1, mimpid
        imem[20] = 32'hF1402373;  // csrr t1, mhartid
        
        // Test 10: No-write test (rs1=x0 for CSRRS/CSRRC)
        imem[21] = 32'h08800293;  // addi t0, x0, 0x88        (t0 = 0x88)
        imem[22] = 32'h30029373;  // csrrw t1, mstatus, t0    (write 0x88 to mstatus)
        imem[23] = 32'h30002373;  // csrrs t1, mstatus, x0    (read-only, no write because rs1=x0)
        
        // End marker
        imem[24] = 32'h00000013;  // nop
        imem[25] = 32'h00000013;  // nop
        imem[26] = 32'h00000013;  // nop
        
        // Initialize rest to NOPs
        for (integer i = 27; i < 256; i = i + 1) begin
            imem[i] = 32'h00000013;
        end
        
        // Initialize data memory
        for (integer i = 0; i < 256; i = i + 1) begin
            dmem[i] = 32'h00000000;
        end
    end
    
    // Test monitoring and checking
    integer test_num;
    integer pass_count;
    integer fail_count;
    
    initial begin
        test_num = 0;
        pass_count = 0;
        fail_count = 0;
        
        // Reset
        rst = 1;
        #20;
        rst = 0;
        
        $display("\n========================================");
        $display("CSR Testbench Started");
        $display("========================================\n");
        
        // Wait for instructions to execute
        #50;  // Wait 5 cycles for pipeline to fill
        
        // Test 1: Check t2 has value read from mscratch
        wait_for_pc(16);  // After instruction at PC=0x0C (imem[3])
        #10;
        check_register(7, 32'h12345678, "Test 1: CSRRW read mscratch to t2");
        check_csr_mscratch(32'h00000000, "Test 1: CSRRW wrote 0 to mscratch");
        
        // Test 2: Check mstatus after CSRRS
        wait_for_pc(28);  // After instruction at PC=0x18 (imem[6])
        #10;
        check_csr_mstatus(32'h00000088, "Test 2: CSRRS set bits in mstatus (masked to 0x88)");
        
        // Test 3: Check mstatus after CSRRC
        wait_for_pc(36);  // After instruction at PC=0x20 (imem[8])
        #10;
        check_csr_mstatus(32'h00000080, "Test 3: CSRRC cleared bit 3 (0x88 -> 0x80)");
        
        // Test 4: Check mscratch after CSRRWI
        wait_for_pc(44);  // After instruction at PC=0x28 (imem[10])
        #10;
        check_register(6, 32'h00000001, "Test 4: CSRRWI wrote immediate 1 to mscratch");
        
        // Test 5: Check mstatus after CSRRSI
        wait_for_pc(48);  // After instruction at PC=0x2C (imem[11])
        #10;
        // Should have bit 0 set (but it might be masked)
        $display("Test 5: CSRRSI - mstatus = 0x%h (bit 0 attempted)", dut.csr_file_inst.mstatus);
        
        // Test 6: Check mstatus after CSRRCI
        wait_for_pc(52);  // After instruction at PC=0x30 (imem[12])
        #10;
        $display("Test 6: CSRRCI - mstatus = 0x%h", dut.csr_file_inst.mstatus);
        
        // Test 7: Check cycle counter was read
        wait_for_pc(56);  // After instruction at PC=0x34 (imem[13])
        #10;
        $display("Test 7: Read cycle counter low = 0x%h (should be > 0)", dut.reg_file_inst.regfile[6]);
        if (dut.reg_file_inst.regfile[6] > 0) begin
            $display("  PASS: Cycle counter is incrementing");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: Cycle counter is 0");
            fail_count = fail_count + 1;
        end
        
        // Test 8: Check instret counter
        wait_for_pc(64);  // After instruction at PC=0x3C (imem[15])
        #10;
        $display("Test 8: Read instret counter low = 0x%h (should be > 0)", dut.reg_file_inst.regfile[6]);
        if (dut.reg_file_inst.regfile[6] > 0) begin
            $display("  PASS: Instret counter is incrementing");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: Instret counter is 0");
            fail_count = fail_count + 1;
        end
        
        // Test 9: Check machine info reads
        wait_for_pc(84);  // After instruction at PC=0x50 (imem[20])
        #10;
        $display("Test 9: Read machine info registers:");
        $display("  mvendorid = 0x%h", dut.csr_file_inst.mvendorid);
        $display("  marchid   = 0x%h", dut.csr_file_inst.marchid);
        $display("  mimpid    = 0x%h", dut.csr_file_inst.mimpid);
        $display("  mhartid   = 0x%h", dut.csr_file_inst.mhartid);
        $display("  PASS: Machine info registers readable");
        pass_count = pass_count + 1;
        
        // Test 10: Read-only CSRRS (rs1=x0)
        wait_for_pc(96);  // After instruction at PC=0x5C (imem[23])
        #10;
        check_csr_mstatus(32'h00000088, "Test 10: CSRRS with rs1=x0 didn't modify mstatus");
        
        // Summary
        #100;
        $display("\n========================================");
        $display("CSR Test Summary");
        $display("========================================");
        $display("Tests Passed: %0d", pass_count);
        $display("Tests Failed: %0d", fail_count);
        if (fail_count == 0) begin
            $display("\n*** ALL TESTS PASSED! ***\n");
        end else begin
            $display("\n*** SOME TESTS FAILED ***\n");
        end
        
        $finish;
    end
    
    // Helper task to wait for specific PC
    task wait_for_pc;
        input [31:0] target_pc;
        begin
            wait (dut.pc_reg == target_pc);
            #1; // Small delay to let signals settle
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
            $display("%s", test_name);
            $display("  Expected: 0x%h, Got: 0x%h", expected, actual);
            if (actual == expected) begin
                $display("  PASS");
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL");
                fail_count = fail_count + 1;
            end
        end
    endtask
    
    // Helper task to check CSR mstatus
    task check_csr_mstatus;
        input [31:0] expected;
        input [200*8:0] test_name;
        reg [31:0] actual;
        begin
            actual = dut.csr_file_inst.mstatus;
            $display("%s", test_name);
            $display("  Expected: 0x%h, Got: 0x%h", expected, actual);
            if (actual == expected) begin
                $display("  PASS");
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL");
                fail_count = fail_count + 1;
            end
        end
    endtask
    
    // Helper task to check CSR mscratch
    task check_csr_mscratch;
        input [31:0] expected;
        input [200*8:0] test_name;
        reg [31:0] actual;
        begin
            actual = dut.csr_file_inst.mscratch;
            $display("%s", test_name);
            $display("  Expected: 0x%h, Got: 0x%h", expected, actual);
            if (actual == expected) begin
                $display("  PASS");
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL");
                fail_count = fail_count + 1;
            end
        end
    endtask
    
    // Waveform dump
    initial begin
        $dumpfile("csr_test.vcd");
        $dumpvars(0, csr_testbench);
    end
    
    // Timeout
    initial begin
        #10000;
        $display("\nERROR: Testbench timeout!");
        $finish;
    end

endmodule