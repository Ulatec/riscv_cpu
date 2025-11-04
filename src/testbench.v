`include "top.v"
module top_test;
  reg rst, clk;
  wire [31:0] cycle;
  integer i;
  reg [10:0] loc = 1000;//location
  //instantiate DUT
  top dut (rst, clk, cycle);
  initial
  begin
        $dumpfile("test.vcd");
    $dumpvars;
    rst=1;
    clk=0;
    #50;
    rst=0;
    #5000;

    //Print register content
    $display("*** Printing register content ***");
    for(i=0; i<9; i=i+1)
    // $display("Time=%0t PC=0x%h Instr (IF/ID)=0x%h",
    //            $time, dut.cpu0.pc_reg, dut.cpu0.if_id_instruction);31
      $display("X[%0d] = %0d ",i,$signed(dut.cpu0.reg_file_inst.regfile[i]));
    $display("Clock cycle=%0d", cycle);
    $display("Data at location %d = %d",loc, dut.mem0.PROGMEM[loc[10:2]]);

    $finish;
  end
  always @(posedge clk) begin
    // Only display after reset is finished
    if (!rst) begin
      // Display PC (address of fetched instruction) and the instruction now in IF/ID reg
      $display("Time=%0t PC=0x%h Instr (IF/ID)=0x%h",
               $time, dut.cpu0.pc_reg, dut.cpu0.if_id_instruction);
    end
  end
  initial begin
  $monitor("Time=%0d, X[0]=%0d, X[1]=%0d, X[2]=%0d, X[3]=%0d, X[4]=%0d,  X[5]=%0d, Cycle=%0d, PC=%0d",$time, dut.cpu0.reg_file_inst.regfile[0],dut.cpu0.reg_file_inst.regfile[1],dut.cpu0.reg_file_inst.regfile[2],dut.cpu0.reg_file_inst.regfile[3],dut.cpu0.reg_file_inst.regfile[4],dut.cpu0.reg_file_inst.regfile[5], cycle,dut.cpu0.pc_reg);
  end
  
  always #5 clk=~clk; //clock generator
endmodule
