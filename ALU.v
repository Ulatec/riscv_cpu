module alu(
    input [31:0] alu_in1,
    input [31:0] alu_in2,
    input [3:0] ALUOp,
    output reg [31:0] alu_out,
    output zero_flag
);
parameter ALU_ADD  = 4'b0000;
parameter ALU_SUB  = 4'b0001;
parameter ALU_XOR  = 4'b0010;
parameter ALU_OR   = 4'b0011;
parameter ALU_AND  = 4'b0100;
parameter ALU_SLL  = 4'b0101;
parameter ALU_SRL  = 4'b0110;
parameter ALU_SRA  = 4'b0111;
parameter ALU_SLT  = 4'b1000;
parameter ALU_SLTU = 4'b1001;
assign zero_flag = (alu_out == 32'b0);
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
        default: alu_out = 32'bxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx;
    endcase
end



endmodule