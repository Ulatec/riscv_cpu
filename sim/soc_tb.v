// =============================================================================
// SoC Testbench
// =============================================================================
// Tests UART output with a simple "Hello World" program
// =============================================================================

`timescale 1ns/1ps

`include "../src/soc.v"

module soc_tb;

    reg clk;
    reg rst;
    
    wire uart_tx;
    reg  uart_rx;
    
    wire [31:0] debug_pc;
    wire [31:0] debug_cycle;
    
    // Instantiate SoC
    soc #(
        .RAM_SIZE(32'h10000),  // 64KB
        .RAM_INIT_FILE("")     // We'll load manually
    ) dut (
        .clk(clk),
        .rst(rst),
        .uart_tx(uart_tx),
        .uart_rx(uart_rx),
        .debug_pc(debug_pc),
        .debug_cycle(debug_cycle)
    );
    
    // Clock generation: 10ns period (100MHz)
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end
    
    // =========================================================================
    // Load Test Program
    // =========================================================================
    // 
    // Simple program that writes "Hello\n" to UART at 0x10000000
    //
    // Assembly:
    //   _start:
    //       li   t0, 0x10000000    # UART base address
    //       li   t1, 'H'
    //       sb   t1, 0(t0)         # Write 'H'
    //       li   t1, 'e'
    //       sb   t1, 0(t0)         # Write 'e'
    //       li   t1, 'l'
    //       sb   t1, 0(t0)         # Write 'l'
    //       li   t1, 'l'
    //       sb   t1, 0(t0)         # Write 'l'
    //       li   t1, 'o'
    //       sb   t1, 0(t0)         # Write 'o'
    //       li   t1, '\n'
    //       sb   t1, 0(t0)         # Write '\n'
    //   done:
    //       j    done              # Loop forever
    //
    // =========================================================================
    
    initial begin
        // Wait for arrays to be ready
        #1;
        
        // lui t0, 0x10000          # t0 = 0x10000000
        dut.ram[0] = 32'h100002b7;
        
        // addi t1, x0, 'H' (0x48)
        dut.ram[1] = 32'h04800313;
        
        // sb t1, 0(t0)
        dut.ram[2] = 32'h00628023;
        
        // addi t1, x0, 'e' (0x65)
        dut.ram[3] = 32'h06500313;
        
        // sb t1, 0(t0)
        dut.ram[4] = 32'h00628023;
        
        // addi t1, x0, 'l' (0x6C)
        dut.ram[5] = 32'h06c00313;
        
        // sb t1, 0(t0)
        dut.ram[6] = 32'h00628023;
        
        // addi t1, x0, 'l' (0x6C)
        dut.ram[7] = 32'h06c00313;
        
        // sb t1, 0(t0)
        dut.ram[8] = 32'h00628023;
        
        // addi t1, x0, 'o' (0x6F)
        dut.ram[9] = 32'h06f00313;
        
        // sb t1, 0(t0)
        dut.ram[10] = 32'h00628023;
        
        // addi t1, x0, '\n' (0x0A)
        dut.ram[11] = 32'h00a00313;
        
        // sb t1, 0(t0)
        dut.ram[12] = 32'h00628023;
        
        // j done (offset = 0, infinite loop)
        dut.ram[13] = 32'h0000006f;
        
        // Debug: print RAM contents (commented out)
        // $display("RAM loaded:");
        // $display("  ram[0]=0x%08x (lui)", dut.ram[0]);
        // ...
    end
    
    // =========================================================================
    // Test Sequence
    // =========================================================================
    
    integer char_count;
    reg [7:0] received_chars [0:15];
    
    initial begin
        $display("\n===========================================");
        $display("SoC UART Test - Hello World");
        $display("===========================================\n");
        
        char_count = 0;
        rst = 1;
        uart_rx = 1;  // Idle high
        
        #100;
        rst = 0;
        
        $display("Reset released, starting execution...\n");
        $display("UART Output:");
        $display("------------");
        
        // Wait for program to complete (should print "Hello\n")
        // The UART outputs characters each cycle when data is written
        #5000;
        
        $display("\n------------");
        $display("\nTest completed after %0d cycles", debug_cycle);
        
        // Check results
        if (char_count >= 6) begin
            $display("SUCCESS: Received %0d characters", char_count);
        end else begin
            $display("WARNING: Only received %0d characters (expected 6)", char_count);
        end
        
        $display("\n===========================================\n");
        $finish;
    end
    
    // Capture UART output
    always @(posedge clk) begin
        if (dut.uart_sim_tx_valid) begin
            received_chars[char_count] = dut.uart_sim_tx_data;
            char_count = char_count + 1;
        end
    end
    
    // Timeout
    initial begin
        #100000;
        $display("ERROR: Timeout!");
        $finish;
    end
    
    // Waveform dump
    initial begin
        $dumpfile("soc_test.vcd");
        $dumpvars(0, soc_tb);
    end
    
    // Debug: show PC progression (disabled for clean output)
    reg [31:0] last_pc;
    always @(posedge clk) begin
        if (!rst && debug_pc !== last_pc) begin
            // if (debug_cycle <= 50) begin
            //     $display("  Cycle %0d: PC = 0x%08x", debug_cycle, debug_pc);
            // end
            last_pc <= debug_pc;
        end
    end

endmodule