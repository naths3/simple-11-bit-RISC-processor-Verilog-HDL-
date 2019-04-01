
//`timescale 1ns / 1ps  
module testbench(); 
reg clk; 
reg reset; 
  wire [3:0] WriteData, DataAdr;
  wire [7:0] PC; 
wire MemWrite,BeqFlag, Beq;
  wire [1:0] ALUControl;
  wire[1:0] AdrRd;
  wire [1:0] AdrRa;
                 wire [1:0] AdrRb;

// instantiate device to be tested
  top dut( .clk(clk), .reset(reset), .WriteData(WriteData), .DataAdr(DataAdr), .MemWrite(MemWrite) ,.PC(PC), .ALUControl(ALUControl), .AdrRd(AdrRd), .AdrRa(AdrRa), .AdrRb(AdrRb), .BeqFlag(BeqFlag) , .Beq(Beq) );

// initialize test
initial
begin
	reset <= 1; # 22; reset <= 0;
end

// generate clock to sequence tests
always
begin
	clk <= 1; # 10; clk <= 0; # 10;
end


// check that 7 gets written to address 0x64
// at end of program
always @(negedge clk)
begin
  $display("---------");
  $display("PC is ", PC);
  $display("MemWrite is ",MemWrite);
  $display("ALUControl is ", ALUControl);
  $display(" AdrRd is ",  AdrRd);
  $display(" AdrRa is ",  AdrRa);
  $display(" AdrRb is ",  AdrRb);
  $display(" BeqFlag is ",  BeqFlag);
  $display(" Beq is ",  Beq);
      $display("DataAdr=",DataAdr);
     $display("WriteData=",WriteData);
	if(MemWrite) begin
      
      if(DataAdr == 7 & WriteData == 6) begin
			$display("Simulation succeeded");
			$stop;
     // end else if(DataAdr !=96) begin
       // $display("Simulation failed");
			//$stop;
      end
    end //if MemWrite
end //always block
  
endmodule


//Top module

module top(input clk, reset, 
           output [3:0] WriteData, DataAdr,
           output [7:0] PC, 
output MemWrite,BeqFlag,Beq,
           output [1:0] ALUControl,
            output  [1:0] AdrRd,
                 output [1:0] AdrRa,
                 output  [1:0] AdrRb);

  
  wire  [3:0] ReadData;
  wire [10:0] Instr;
// instantiate processor and memories
  processor proc( .clk(clk), .reset(reset), .PC(PC), .Instr(Instr), .MemWrite(MemWrite), .ALUResult(DataAdr), .WriteData(WriteData), .ReadData(ReadData), .ALUControl(ALUControl), .AdrRd(AdrRd), .AdrRa(AdrRa), .AdrRb(AdrRb) ,.BeqFlag(BeqFlag) , .Beq(Beq) );
  imem imem( .a(PC), .rd(Instr) );
  dmem dmem( .clk(clk), .we(MemWrite), .a(DataAdr), .wd(WriteData), .rd(ReadData) );

endmodule

//Dmem module
module dmem(input clk, we, 
            input [3:0] a, wd, 
            output [3:0] rd); 

  reg [3:0] RAM[15:0];

  assign rd = RAM[a]; // no need for word aligned in  4 bit case

  always @(posedge clk) 
    begin
      if (we) RAM[a] <= wd;
    end
endmodule

//Imem module

module imem(input [7:0] a, 
            output [10:0] rd); //11 bit instr

  reg [10:0] RAM[127:0];

initial
  $readmemb("memfile.txt",RAM);

  assign rd = RAM[a]; // no need for word aligned in 8 bit
endmodule






