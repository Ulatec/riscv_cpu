module reg_file(
    input [4:0] rs1 ,rs2,
    input clk, rst,
    input [4:0] rd, 
    input [31:0] write_data,
    input write_en,
    output [31:0] rs1_data,
    output [31:0] rs2_data
);

reg [31:0] regfile[0:31];//Register file with X0 to X31;
integer i;
assign rs1_data = (rs1 == 5'b0) ? 32'b0 :  regfile[rs1];
assign rs2_data = (rs2 == 5'b0) ? 32'b0 :  regfile[rs2];
always @(posedge clk)
begin
    if(rst)begin
        for (i = 1; i < 32; i = i + 1) begin // Skip x0, reset all regs to 0
            regfile[i] <= 32'b0;
        end
    end
    else if(write_en && rd !=5'b0)
    begin
        regfile[rd] <= write_data;
end
end
endmodule