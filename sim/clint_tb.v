// CLINT Testbench
// Tests timer functionality in isolation before CPU integration

`include "../src/clint.v"

module clint_tb;
    
    reg         clk;
    reg         rst;
    reg  [31:0] addr;
    reg  [31:0] wdata;
    reg  [3:0]  wstrb;
    reg         read_en;
    wire [31:0] rdata;
    wire        addr_valid;
    wire        timer_irq;
    
    // Instantiate CLINT
    clint uut (
        .clk(clk),
        .rst(rst),
        .addr(addr),
        .wdata(wdata),
        .wstrb(wstrb),
        .read_en(read_en),
        .rdata(rdata),
        .addr_valid(addr_valid),
        .timer_irq(timer_irq)
    );
    
    // Clock generation - 10ns period
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end
    
    // Test counters
    integer pass_count = 0;
    integer fail_count = 0;
    
    // Helper task: Write to CLINT register
    task clint_write;
        input [31:0] address;
        input [31:0] data;
        begin
            @(posedge clk);
            addr  <= address;
            wdata <= data;
            wstrb <= 4'b1111;
            read_en <= 0;
            @(posedge clk);
            wstrb <= 4'b0000;  // Clear write
        end
    endtask
    
    // Helper task: Read from CLINT register
    task clint_read;
        input  [31:0] address;
        output [31:0] data;
        begin
            @(posedge clk);
            addr    <= address;
            wstrb   <= 4'b0000;
            read_en <= 1;
            @(posedge clk);
            data = rdata;
            read_en <= 0;
        end
    endtask
    
    // Helper task: Check value
    task check;
        input [31:0] expected;
        input [31:0] actual;
        input [255:0] test_name;
        begin
            if (expected === actual) begin
                $display("PASS: %s (expected=0x%08x, got=0x%08x)", test_name, expected, actual);
                pass_count = pass_count + 1;
            end else begin
                $display("FAIL: %s (expected=0x%08x, got=0x%08x)", test_name, expected, actual);
                fail_count = fail_count + 1;
            end
        end
    endtask
    
    // Main test sequence
    reg [31:0] read_data;
    reg [63:0] mtime_snapshot;
    
    initial begin
        $display("\n========================================");
        $display("CLINT Testbench");
        $display("========================================\n");
        
        // Initialize
        rst     = 1;
        addr    = 0;
        wdata   = 0;
        wstrb   = 0;
        read_en = 0;
        
        // Reset
        #20;
        rst = 0;
        #10;
        
        // ========================================
        // Test 1: mtime is incrementing
        // ========================================
        $display("--- Test 1: mtime incrementing ---");
        
        clint_read(32'h0200BFF8, read_data);  // Read mtime low
        mtime_snapshot = read_data;
        
        #100;  // Wait 10 cycles
        
        clint_read(32'h0200BFF8, read_data);
        if (read_data > mtime_snapshot) begin
            $display("PASS: mtime is incrementing (was 0x%08x, now 0x%08x)", 
                     mtime_snapshot[31:0], read_data);
            pass_count = pass_count + 1;
        end else begin
            $display("FAIL: mtime not incrementing");
            fail_count = fail_count + 1;
        end
        
        // ========================================
        // Test 2: mtimecmp defaults to max (no interrupt)
        // ========================================
        $display("\n--- Test 2: Initial state (no interrupt) ---");
        
        clint_read(32'h02004000, read_data);  // mtimecmp low
        check(32'hFFFFFFFF, read_data, "mtimecmp_lo initial");
        
        clint_read(32'h02004004, read_data);  // mtimecmp high
        check(32'hFFFFFFFF, read_data, "mtimecmp_hi initial");
        
        check(1'b0, timer_irq, "timer_irq initially low");
        
        // ========================================
        // Test 3: Write mtimecmp and trigger interrupt
        // ========================================
        $display("\n--- Test 3: Trigger timer interrupt ---");
        
        // Set mtimecmp to a small value (mtime should already be past it)
        clint_write(32'h02004000, 32'h00000010);  // mtimecmp_lo = 16
        clint_write(32'h02004004, 32'h00000000);  // mtimecmp_hi = 0
        
        #20;  // Wait a couple cycles
        
        // Verify mtimecmp was written
        clint_read(32'h02004000, read_data);
        check(32'h00000010, read_data, "mtimecmp_lo after write");
        
        // Check interrupt is now pending (mtime > mtimecmp)
        @(posedge clk);
        check(1'b1, timer_irq, "timer_irq high after mtimecmp set low");
        
        // ========================================
        // Test 4: Clear interrupt by setting mtimecmp high
        // ========================================
        $display("\n--- Test 4: Clear interrupt ---");
        
        // Set mtimecmp to a large value
        clint_write(32'h02004000, 32'hFFFFFFFF);
        clint_write(32'h02004004, 32'hFFFFFFFF);
        
        #20;
        
        check(1'b0, timer_irq, "timer_irq low after mtimecmp set high");
        
        // ========================================
        // Test 5: Write to mtime
        // ========================================
        $display("\n--- Test 5: Write to mtime ---");
        
        // Write a specific value to mtime
        clint_write(32'h0200BFF8, 32'h12345678);
        clint_write(32'h0200BFFC, 32'h00000000);
        
        #10;
        
        clint_read(32'h0200BFF8, read_data);
        // Should be close to what we wrote (plus a few increments)
        if (read_data >= 32'h12345678 && read_data < 32'h12345700) begin
            $display("PASS: mtime writable (read back 0x%08x)", read_data);
            pass_count = pass_count + 1;
        end else begin
            $display("FAIL: mtime write failed (expected ~0x12345678, got 0x%08x)", read_data);
            fail_count = fail_count + 1;
        end
        
        // ========================================
        // Test 6: Address validity
        // ========================================
        $display("\n--- Test 6: Address decoding ---");
        
        addr = 32'h0200BFF8;
        @(posedge clk);
        check(1'b1, addr_valid, "addr_valid for mtime_lo");
        
        addr = 32'h02004000;
        @(posedge clk);
        check(1'b1, addr_valid, "addr_valid for mtimecmp_lo");
        
        addr = 32'h00001000;  // Not CLINT
        @(posedge clk);
        check(1'b0, addr_valid, "addr_valid low for non-CLINT addr");
        
        addr = 32'h02001000;  // CLINT region but not valid register
        @(posedge clk);
        check(1'b0, addr_valid, "addr_valid low for invalid CLINT offset");
        
        // ========================================
        // Test 7: 64-bit mtimecmp boundary test
        // ========================================
        $display("\n--- Test 7: 64-bit comparison ---");
        
        // Reset mtime to 0
        clint_write(32'h0200BFF8, 32'h00000000);
        clint_write(32'h0200BFFC, 32'h00000000);
        
        // Set mtimecmp to require high word comparison
        clint_write(32'h02004000, 32'h00000000);  // mtimecmp_lo = 0
        clint_write(32'h02004004, 32'h00000001);  // mtimecmp_hi = 1
        
        #10;
        
        // mtime (0x00000000_0000000X) < mtimecmp (0x00000001_00000000)
        check(1'b0, timer_irq, "No IRQ when mtime_hi < mtimecmp_hi");
        
        // Now set mtime_hi = 1, mtime_lo = 0 (exactly equal)
        clint_write(32'h0200BFF8, 32'h00000000);
        clint_write(32'h0200BFFC, 32'h00000001);
        
        #10;
        
        check(1'b1, timer_irq, "IRQ when mtime >= mtimecmp (equal)");
        
        // ========================================
        // Summary
        // ========================================
        #50;
        $display("\n========================================");
        $display("Test Summary: %0d passed, %0d failed", pass_count, fail_count);
        $display("========================================");
        
        if (fail_count == 0)
            $display("ALL TESTS PASSED!\n");
        else
            $display("SOME TESTS FAILED!\n");
        
        $finish;
    end
    
    // Timeout
    initial begin
        #10000;
        $display("ERROR: Timeout!");
        $finish;
    end
    
    // Waveform dump
    initial begin
        $dumpfile("clint_test.vcd");
        $dumpvars(0, clint_tb);
    end

endmodule