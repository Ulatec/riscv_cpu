module alu(
    input [31:0] alu_in1,
    input [31:0] alu_in2,
    input [4:0] ALUOp,
    output reg [31:0] alu_out,
    output alu_lt,      // Signed less than flag for BLT/BGE
    output alu_ltu,      // Unsigned less than flag for BLTU/BGEU
    output zero_flag
);
parameter ALU_ADD  = 5'b00000;
parameter ALU_SUB  = 5'b00001;
parameter ALU_XOR  = 5'b00010;
parameter ALU_OR   = 5'b00011;
parameter ALU_AND  = 5'b00100;
parameter ALU_SLL  = 5'b00101;
parameter ALU_SRL  = 5'b00110;
parameter ALU_SRA  = 5'b00111;
parameter ALU_SLT  = 5'b01000;
parameter ALU_SLTU = 5'b01001;
parameter ALU_MUL = 5'b01010;
parameter ALU_MULH = 5'b01011;
parameter ALU_MULHSU = 5'b01100;
parameter ALU_MULHU = 5'b01101;
parameter ALU_DIV = 5'b01110;
parameter ALU_DIVU = 5'b01111;
parameter ALU_REM = 5'b10000;
parameter ALU_REMU = 5'b10001;
// Comparison flags for branch instructions
assign zero_flag = (alu_out == 32'b0);
assign alu_lt = $signed(alu_in1) < $signed(alu_in2);    // Signed comparison
assign alu_ltu = alu_in1 < alu_in2;                     // Unsigned comparison
wire [63:0] mul_ss;   // signed × signed
wire [63:0] mul_uu;   // unsigned × unsigned
wire [63:0] mul_su;   // signed × unsigned
assign mul_ss = $signed(alu_in1) * $signed(alu_in2);
assign mul_uu = alu_in1 * alu_in2;  // Both treated as unsigned
assign mul_su = $signed(alu_in1) * $signed({1'b0, alu_in2}); // rs1 signed, rs2 signed
always @(*) begin
    case (ALUOp)
        ALU_ADD:  alu_out = alu_in1 + alu_in2;
        ALU_SUB:  alu_out = alu_in1 - alu_in2; 
        ALU_XOR:  alu_out = alu_in1 ^ alu_in2;
        ALU_OR:   alu_out = alu_in1 | alu_in2;
        ALU_AND: alu_out = alu_in1 & alu_in2;
        ALU_SLL: alu_out = alu_in1 << alu_in2[4:0];
        ALU_SRL: alu_out = alu_in1 >> alu_in2[4:0];
        ALU_SLT: alu_out = ($signed(alu_in1) < $signed(alu_in2)) ? 32'b1 : 32'b0;
        ALU_SLTU: alu_out = (alu_in1 < alu_in2) ? 32'b1 : 32'b0;
        ALU_SRA: alu_out = $signed(alu_in1) >>> alu_in2[4:0];
        ALU_MUL: alu_out = mul_ss[31:0];    // Lower 32 bits
        ALU_MULH: alu_out = mul_ss[63:32];   // Upper 32 bits (signed × signed)
        ALU_MULHSU: alu_out = mul_su[63:32];   // Upper 32 bits (signed × unsigned)
        ALU_MULHU:  alu_out = mul_uu[63:32];   // Upper 32 bits (unsigned × unsigned)
        ALU_DIV:    alu_out = 32'b0;  // Handled by div_unit
        ALU_DIVU:   alu_out = 32'b0;  // Handled by div_unit
        ALU_REM:    alu_out = 32'b0;  // Handled by div_unit
        ALU_REMU:   alu_out = 32'b0;  // Handled by div_unit
        default: alu_out = 32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx;
    endcase
end



endmodule