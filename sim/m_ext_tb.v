`include "../src/definitions.v"
`include "../src/ALU.v"

module m_ext_tb;
    reg [31:0] alu_in1, alu_in2;
    reg [4:0] ALUOp;
    wire [31:0] alu_out;
    wire alu_lt, alu_ltu, zero_flag;
    
    integer pass_count = 0;
    integer fail_count = 0;
    
    alu uut (
        .alu_in1(alu_in1),
        .alu_in2(alu_in2),
        .ALUOp(ALUOp),
        .alu_out(alu_out),
        .alu_lt(alu_lt),
        .alu_ltu(alu_ltu),
        .zero_flag(zero_flag)
    );
    
    task check_result;
        input [31:0] expected;
        input [127:0] test_name;  // String for test name
        begin
            #1;
            if (alu_out === expected) begin
                $display("PASS: %s = 0x%08x", test_name, alu_out);
                pass_count = pass_count + 1;
            end else begin
                $display("FAIL: %s expected 0x%08x, got 0x%08x", test_name, expected, alu_out);
                fail_count = fail_count + 1;
            end
        end
    endtask
    
    initial begin
        $display("========================================");
        $display("M-Extension ALU Testbench");
        $display("========================================\n");
        
        // ========== MUL Tests ==========
        $display("--- MUL (Lower 32 bits) ---");
        
        // Basic multiplication
        alu_in1 = 32'd7; alu_in2 = 32'd6; ALUOp = `ALU_MUL;
        check_result(32'd42, "7 * 6");
        
        // Larger numbers
        alu_in1 = 32'd1000; alu_in2 = 32'd2000; ALUOp = `ALU_MUL;
        check_result(32'd2000000, "1000 * 2000");
        
        // Negative × Positive (lower 32 bits same for signed)
        alu_in1 = 32'hFFFFFFFF; alu_in2 = 32'd5; ALUOp = `ALU_MUL; // -1 * 5
        check_result(32'hFFFFFFFB, "-1 * 5 (lower bits)");
        
        // ========== MULH Tests (signed × signed, upper 32 bits) ==========
        $display("\n--- MULH (Upper 32 bits, signed) ---");
        
        // Small positive × positive → upper = 0
        alu_in1 = 32'd100; alu_in2 = 32'd200; ALUOp = `ALU_MULH;
        check_result(32'd0, "100 * 200 upper");
        
        // Large positive × positive
        alu_in1 = 32'h10000000; alu_in2 = 32'h10; ALUOp = `ALU_MULH;
        check_result(32'h00000001, "0x10000000 * 0x10 upper");
        
        // Negative × Positive → upper should be negative (sign extended)
        alu_in1 = 32'hFFFFFFFF; alu_in2 = 32'd1; ALUOp = `ALU_MULH; // -1 * 1
        check_result(32'hFFFFFFFF, "-1 * 1 upper (signed)");
        
        // Negative × Negative → upper positive
        alu_in1 = 32'hFFFFFFFF; alu_in2 = 32'hFFFFFFFF; ALUOp = `ALU_MULH; // -1 * -1
        check_result(32'd0, "-1 * -1 upper");
        
        // ========== MULHU Tests (unsigned × unsigned, upper 32 bits) ==========
        $display("\n--- MULHU (Upper 32 bits, unsigned) ---");
        
        // 0xFFFFFFFF treated as large unsigned
        alu_in1 = 32'hFFFFFFFF; alu_in2 = 32'd2; ALUOp = `ALU_MULHU;
        check_result(32'd1, "0xFFFFFFFF * 2 upper (unsigned)");
        
        // Max unsigned × 2
        alu_in1 = 32'hFFFFFFFF; alu_in2 = 32'hFFFFFFFF; ALUOp = `ALU_MULHU;
        check_result(32'hFFFFFFFE, "0xFFFFFFFF * 0xFFFFFFFF upper");
        
        // ========== MULHSU Tests (signed × unsigned, upper 32 bits) ==========
        $display("\n--- MULHSU (signed × unsigned, upper) ---");
        
        // Positive signed × unsigned
        alu_in1 = 32'd100; alu_in2 = 32'hFFFFFFFF; ALUOp = `ALU_MULHSU;
        check_result(32'd99, "100(s) * 0xFFFFFFFF(u) upper");
        
        // Negative signed × unsigned
        alu_in1 = 32'hFFFFFFFF; alu_in2 = 32'd2; ALUOp = `ALU_MULHSU; // -1(s) * 2(u)
        check_result(32'hFFFFFFFF, "-1(s) * 2(u) upper");
        
        // ========== DIV Tests ==========
        $display("\n--- DIV (Signed Division) ---");
        
        // Basic positive division
        alu_in1 = 32'd20; alu_in2 = 32'd6; ALUOp = `ALU_DIV;
        check_result(32'd3, "20 / 6");
        
        // Negative dividend
        alu_in1 = 32'hFFFFFFEC; alu_in2 = 32'd5; ALUOp = `ALU_DIV; // -20 / 5
        check_result(32'hFFFFFFFC, "-20 / 5 = -4");
        
        // Both negative
        alu_in1 = 32'hFFFFFFEC; alu_in2 = 32'hFFFFFFFB; ALUOp = `ALU_DIV; // -20 / -5
        check_result(32'd4, "-20 / -5 = 4");
        
        // Divide by zero → -1 (RISC-V spec)
        alu_in1 = 32'd100; alu_in2 = 32'd0; ALUOp = `ALU_DIV;
        check_result(32'hFFFFFFFF, "100 / 0 = -1 (spec)");
        
        // Signed overflow: MIN_INT / -1 → MIN_INT (RISC-V spec)
        alu_in1 = 32'h80000000; alu_in2 = 32'hFFFFFFFF; ALUOp = `ALU_DIV;
        check_result(32'h80000000, "MIN_INT / -1 = MIN_INT (spec)");
        
        // ========== DIVU Tests ==========
        $display("\n--- DIVU (Unsigned Division) ---");
        
        alu_in1 = 32'd100; alu_in2 = 32'd7; ALUOp = `ALU_DIVU;
        check_result(32'd14, "100 / 7 (unsigned)");
        
        // Large unsigned value
        alu_in1 = 32'hFFFFFFFF; alu_in2 = 32'd2; ALUOp = `ALU_DIVU;
        check_result(32'h7FFFFFFF, "0xFFFFFFFF / 2 (unsigned)");
        
        // Divide by zero → 0xFFFFFFFF (RISC-V spec)
        alu_in1 = 32'd100; alu_in2 = 32'd0; ALUOp = `ALU_DIVU;
        check_result(32'hFFFFFFFF, "100 / 0 = 0xFFFFFFFF (spec)");
        
        // ========== REM Tests ==========
        $display("\n--- REM (Signed Remainder) ---");
        
        alu_in1 = 32'd20; alu_in2 = 32'd6; ALUOp = `ALU_REM;
        check_result(32'd2, "20 %% 6");
        
        // Negative dividend
        alu_in1 = 32'hFFFFFFEC; alu_in2 = 32'd6; ALUOp = `ALU_REM; // -20 % 6
        check_result(32'hFFFFFFFE, "-20 %% 6 = -2");
        
        // Divide by zero → dividend (RISC-V spec)
        alu_in1 = 32'd123; alu_in2 = 32'd0; ALUOp = `ALU_REM;
        check_result(32'd123, "123 %% 0 = 123 (spec)");
        
        // Signed overflow: MIN_INT % -1 → 0 (RISC-V spec)
        alu_in1 = 32'h80000000; alu_in2 = 32'hFFFFFFFF; ALUOp = `ALU_REM;
        check_result(32'd0, "MIN_INT %% -1 = 0 (spec)");
        
        // ========== REMU Tests ==========
        $display("\n--- REMU (Unsigned Remainder) ---");
        
        alu_in1 = 32'd100; alu_in2 = 32'd7; ALUOp = `ALU_REMU;
        check_result(32'd2, "100 %% 7 (unsigned)");
        
        // Divide by zero → dividend (RISC-V spec)
        alu_in1 = 32'd456; alu_in2 = 32'd0; ALUOp = `ALU_REMU;
        check_result(32'd456, "456 %% 0 = 456 (spec)");
        
        // ========== Verify Base Operations Still Work ==========
        $display("\n--- Base RV32I Operations (Regression) ---");
        
        alu_in1 = 32'd10; alu_in2 = 32'd3; ALUOp = `ALU_ADD;
        check_result(32'd13, "ADD: 10 + 3");
        
        alu_in1 = 32'd10; alu_in2 = 32'd3; ALUOp = `ALU_SUB;
        check_result(32'd7, "SUB: 10 - 3");
        
        alu_in1 = 32'hF0F0F0F0; alu_in2 = 32'h0F0F0F0F; ALUOp = `ALU_XOR;
        check_result(32'hFFFFFFFF, "XOR");
        
        alu_in1 = 32'd8; alu_in2 = 32'd2; ALUOp = `ALU_SLL;
        check_result(32'd32, "SLL: 8 << 2");
        
        // ========== Summary ==========
        $display("\n========================================");
        $display("Test Summary: %0d passed, %0d failed", pass_count, fail_count);
        $display("========================================");
        
        if (fail_count == 0)
            $display("ALL TESTS PASSED!");
        else
            $display("SOME TESTS FAILED!");
            
        $finish;
    end
endmodule