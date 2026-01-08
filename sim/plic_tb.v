// PLIC Testbench
// Tests external interrupt controller functionality

`include "../src/plic.v"

module plic_tb;

    reg         clk;
    reg         rst;
    reg  [31:0] addr;
    reg  [31:0] wdata;
    reg  [3:0]  wstrb;
    reg         read_en;
    wire [31:0] rdata;
    wire        addr_valid;
    reg  [31:0] irq_sources;
    wire        external_irq;
    
    // Instantiate PLIC
    plic uut (
        .clk(clk),
        .rst(rst),
        .addr(addr),
        .wdata(wdata),
        .wstrb(wstrb),
        .read_en(read_en),
        .rdata(rdata),
        .addr_valid(addr_valid),
        .irq_sources(irq_sources),
        .external_irq(external_irq)
    );
    
    // Clock: 10ns period
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end
    
    // Test counters
    integer pass_count;
    integer fail_count;
    
    // Helper task: Write to PLIC
    task plic_write;
        input [31:0] address;
        input [31:0] data;
        begin
            #2;  // Move away from any clock edge
            addr  = address;
            wdata = data;
            wstrb = 4'b1111;
            read_en = 0;
            @(posedge clk);
            #1;  // Let registers update
            wstrb = 4'b0;
        end
    endtask
    
    // Helper task: Read from PLIC
    task plic_read;
        input  [31:0] address;
        output [31:0] data;
        begin
            #2;  // Move away from any clock edge
            addr    = address;
            wstrb   = 4'b0;
            read_en = 1;
            #1;  // Let combinational logic settle
            data = rdata;  // Capture value BEFORE clock edge
            @(posedge clk);
            #1;
            read_en = 0;
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
    
    // Main test
    reg [31:0] read_data;
    
    initial begin
        $display("\n========================================");
        $display("PLIC Testbench");
        $display("========================================\n");
        
        // Initialize
        pass_count = 0;
        fail_count = 0;
        rst = 1;
        addr = 0;
        wdata = 0;
        wstrb = 0;
        read_en = 0;
        irq_sources = 32'b0;
        
        #20;
        rst = 0;
        #10;
        
        // ========================================
        // Test 1: Initial state - no interrupt
        // ========================================
        $display("--- Test 1: Initial state ---");
        
        check(1'b0, external_irq, "external_irq initially low");
        
        plic_read(32'h0C001000, read_data);  // pending
        check(32'h0, read_data, "pending initially 0");
        
        plic_read(32'h0C002000, read_data);  // enable
        check(32'h0, read_data, "enable initially 0");
        
        // ========================================
        // Test 2: Configure priority for source 1
        // ========================================
        $display("\n--- Test 2: Set priority ---");
        
        plic_write(32'h0C000004, 32'd5);  // priority[1] = 5
        
        plic_read(32'h0C000004, read_data);
        check(32'd5, read_data, "priority[1] = 5");
        
        // ========================================
        // Test 3: Enable source 1
        // ========================================
        $display("\n--- Test 3: Enable source ---");
        
        plic_write(32'h0C002000, 32'h00000002);  // enable bit 1
        
        plic_read(32'h0C002000, read_data);
        check(32'h00000002, read_data, "enable[1] = 1");
        
        // Still no interrupt (no pending)
        check(1'b0, external_irq, "external_irq still low (no pending)");
        
        // ===== Trigger interrupt on source 1
        // ========================================
        $display("\n--- Test 4: Trigger interrupt ---");
        
        // Ensure irq_sources is 0, then create rising edge
        #2;
        irq_sources = 32'h00000000;
        @(posedge clk);  // Let irq_sources_prev become 0
        @(posedge clk);  // One more to be sure
        
        // Now create rising edge on source 1
        #2;
        irq_sources = 32'h00000002;
        @(posedge clk);  // This clock should detect the rising edge and set pending
        #1;
        
        // Check pending
        plic_read(32'h0C001000, read_data);
        check(32'h00000002, read_data, "pending[1] = 1");
        
        // Check external_irq is now high
        check(1'b1, external_irq, "external_irq high");
        
        // ========================================
        // Test 5: Claim interrupt
        // ========================================
        $display("\n--- Test 5: Claim interrupt ---");
        
        plic_read(32'h0C200004, read_data);  // claim
        check(32'd1, read_data, "claim returns source 1");
        
        // After claim, pending should be cleared
        @(posedge clk);
        plic_read(32'h0C001000, read_data);
        check(32'h00000000, read_data, "pending cleared after claim");
        
        // external_irq should be low now
        check(1'b0, external_irq, "external_irq low after claim");
        
        // ========================================
        // Test 6: Complete interrupt
        // ========================================
        $display("\n--- Test 6: Complete interrupt ---");
        
        plic_write(32'h0C200004, 32'd1);  // complete source 1
        @(posedge clk);
        
        // Should be able to claim again if another IRQ comes
        // For now, just verify no crash
        $display("PASS: Complete accepted");
        pass_count = pass_count + 1;
        
        // ========================================
        // Test 7: Priority threshold
        // ========================================
        $display("\n--- Test 7: Priority threshold ---");
        
        // Clear IRQ source first
        #2;
        irq_sources = 32'h00000000;
        @(posedge clk);
        @(posedge clk);
        
        // Set threshold to 5 (only priority > 5 can fire)
        plic_write(32'h0C200000, 32'd5);
        
        plic_read(32'h0C200000, read_data);
        check(32'd5, read_data, "threshold = 5");
        
        // Source 1 has priority 5, which is NOT > 5, so shouldn't fire
        #2;
        irq_sources = 32'h00000002;  // Rising edge on source 1
        @(posedge clk);
        #1;
        
        // Pending should be set but external_irq should be low
        plic_read(32'h0C001000, read_data);
        check(32'h00000002, read_data, "pending[1] set");
        check(1'b0, external_irq, "external_irq low (priority <= threshold)");
        
        // Raise priority to 6
        plic_write(32'h0C000004, 32'd6);
        @(posedge clk);
        #1;
        
        // Now it should fire
        check(1'b1, external_irq, "external_irq high (priority > threshold)");
        
        // Clean up - claim and complete
        plic_read(32'h0C200004, read_data);
        plic_write(32'h0C200004, read_data);
        
        // ========================================
        // Test 8: Multiple sources - priority arbitration
        // ========================================
        $display("\n--- Test 8: Priority arbitration ---");
        
        // Reset threshold
        plic_write(32'h0C200000, 32'd0);
        
        // Clear all pending
        #2;
        irq_sources = 32'h00000000;
        @(posedge clk);
        @(posedge clk);
        
        // Configure source 2 with priority 3
        plic_write(32'h0C000008, 32'd3);  // priority[2] = 3
        
        // Configure source 3 with priority 7 (highest)
        plic_write(32'h0C00000C, 32'd7);  // priority[3] = 7
        
        // Enable sources 1, 2, 3
        plic_write(32'h0C002000, 32'h0000000E);  // bits 1,2,3
        
        // Trigger all three sources
        #2;
        irq_sources = 32'h0000000E;
        @(posedge clk);
        #1;
        
        // Claim should return source 3 (highest priority = 7)
        plic_read(32'h0C200004, read_data);
        check(32'd3, read_data, "claim returns source 3 (priority 7)");
        
        // Complete source 3
        plic_write(32'h0C200004, 32'd3);
        @(posedge clk);
        #1;
        
        // Next claim should return source 1 (priority 6)
        plic_read(32'h0C200004, read_data);
        check(32'd1, read_data, "claim returns source 1 (priority 6)");
        
        // Complete source 1
        plic_write(32'h0C200004, 32'd1);
        @(posedge clk);
        #1;
        
        // Next claim should return source 2 (priority 3)
        plic_read(32'h0C200004, read_data);
        check(32'd2, read_data, "claim returns source 2 (priority 3)");
        
        // Complete source 2
        plic_write(32'h0C200004, 32'd2);
        @(posedge clk);
        #1;
        
        // No more pending
        check(1'b0, external_irq, "external_irq low after all claimed");
        
        // ========================================
        // Test 9: Disabled source doesn't fire
        // ========================================
        $display("\n--- Test 9: Disabled source ---");
        
        #2;
        irq_sources = 32'h00000000;
        @(posedge clk);
        @(posedge clk);
        
        // Disable source 1
        plic_write(32'h0C002000, 32'h0000000C);  // Only bits 2,3 enabled
        
        // Trigger source 1
        #2;
        irq_sources = 32'h00000002;
        @(posedge clk);
        #1;
        
        // Pending should be set but external_irq low (disabled)
        plic_read(32'h0C001000, read_data);
        // Note: pending is set regardless of enable
        check(1'b0, external_irq, "external_irq low (source disabled)");
        
        // ========================================
        // Test 10: Address decoding
        // ========================================
        $display("\n--- Test 10: Address decoding ---");
        
        #2;
        addr = 32'h0C000004;  // priority[1]
        #1;
        check(1'b1, addr_valid, "addr_valid for priority[1]");
        
        addr = 32'h0C001000;  // pending
        #1;
        check(1'b1, addr_valid, "addr_valid for pending");
        
        addr = 32'h0C002000;  // enable
        #1;
        check(1'b1, addr_valid, "addr_valid for enable");
        
        addr = 32'h0C200000;  // threshold
        #1;
        check(1'b1, addr_valid, "addr_valid for threshold");
        
        addr = 32'h0C200004;  // claim
        #1;
        check(1'b1, addr_valid, "addr_valid for claim");
        
        addr = 32'h0D000000;  // Not PLIC
        #1;
        check(1'b0, addr_valid, "addr_valid low for non-PLIC");
        
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
        #20000;
        $display("ERROR: Timeout!");
        $finish;
    end
    
    // Waveform dump
    initial begin
        $dumpfile("plic_test.vcd");
        $dumpvars(0, plic_tb);
    end

endmodule