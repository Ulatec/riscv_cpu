// =============================================================================
// Iterative Divider Unit for RISC-V M Extension
// =============================================================================
// 32-cycle restoring division algorithm. Handles DIV, DIVU, REM, REMU.
// Special cases (div-by-zero, signed overflow) resolve in 1 cycle.
//
// Interface:
//   start pulse -> busy asserted -> 32 cycles -> done pulse + result valid
//   Pipeline should stall EX stage while busy || (is_div && !done)
// =============================================================================

module div_unit (
    input         clk,
    input         rst,
    input         start,       // Pulse to begin division
    input         is_unsigned, // 1 for DIVU/REMU
    input         is_rem,      // 1 for REM/REMU
    input  [31:0] dividend,
    input  [31:0] divisor,
    output [31:0] result,
    output        busy,
    output reg    done
);

    reg        active;
    reg [4:0]  step;       // 0 to 31 (32 iterations)
    reg [32:0] remainder;  // 33-bit partial remainder
    reg [31:0] quotient;   // Quotient accumulator
    reg [31:0] dvsor;      // Saved unsigned divisor
    reg [31:0] dvnd;       // Dividend bits (shifted out MSB first)
    reg        neg_quo;    // Negate quotient at end
    reg        neg_rem;    // Negate remainder at end
    reg        rem_op;     // Is this a remainder operation

    assign busy = active;

    reg [31:0] result_reg;
    assign result = result_reg;

    // Trial subtraction: shift remainder left, bring in MSB of dividend
    wire [32:0] shifted = {remainder[31:0], dvnd[31]};
    wire [32:0] trial   = shifted - {1'b0, dvsor};

    always @(posedge clk) begin
        if (rst) begin
            active     <= 0;
            done       <= 0;
            result_reg <= 0;
            step       <= 0;
            remainder  <= 0;
            quotient   <= 0;
            dvsor      <= 0;
            dvnd       <= 0;
            neg_quo    <= 0;
            neg_rem    <= 0;
            rem_op     <= 0;
        end else begin
            done <= 0;

            if (start && !active) begin
                // === Special cases (1-cycle result) ===
                if (divisor == 32'b0) begin
                    // Divide by zero (RISC-V spec)
                    result_reg <= is_rem ? dividend : 32'hFFFFFFFF;
                    done <= 1;
                end else if (!is_unsigned && dividend == 32'h80000000 && divisor == 32'hFFFFFFFF) begin
                    // Signed overflow: INT_MIN / -1
                    result_reg <= is_rem ? 32'b0 : 32'h80000000;
                    done <= 1;
                end else begin
                    // === Start iterative division ===
                    active    <= 1;
                    step      <= 0;
                    rem_op    <= is_rem;
                    remainder <= 33'b0;
                    quotient  <= 32'b0;

                    // Convert signed operands to unsigned magnitude
                    dvnd  <= (!is_unsigned && dividend[31]) ? (~dividend + 1) : dividend;
                    dvsor <= (!is_unsigned && divisor[31])  ? (~divisor + 1)  : divisor;

                    // Remember sign corrections for final result
                    neg_quo <= !is_unsigned && (dividend[31] ^ divisor[31]);
                    neg_rem <= !is_unsigned && dividend[31];
                end
            end else if (active) begin
                // === One iteration of restoring division ===
                // shifted = {remainder[31:0], dvnd[31]}  (bring in next dividend bit)
                // trial   = shifted - divisor

                if (!trial[32]) begin
                    // Subtraction succeeded (trial >= 0)
                    remainder <= trial;
                    quotient  <= {quotient[30:0], 1'b1};
                end else begin
                    // Subtraction failed (trial < 0), keep shifted value
                    remainder <= shifted;
                    quotient  <= {quotient[30:0], 1'b0};
                end

                // Shift dividend left (next MSB becomes available)
                dvnd <= {dvnd[30:0], 1'b0};

                if (step == 31) begin
                    // === Division complete ===
                    active <= 0;
                    done   <= 1;

                    // Compute final result using THIS iteration's combinational values
                    // (registered quotient/remainder are 1 cycle behind)
                    if (rem_op) begin
                        if (neg_rem)
                            result_reg <= ~(!trial[32] ? trial[31:0] : shifted[31:0]) + 1;
                        else
                            result_reg <= !trial[32] ? trial[31:0] : shifted[31:0];
                    end else begin
                        if (neg_quo)
                            result_reg <= ~{quotient[30:0], !trial[32]} + 1;
                        else
                            result_reg <= {quotient[30:0], !trial[32]};
                    end
                end else begin
                    step <= step + 1;
                end
            end
        end
    end

endmodule
