
//control unit
module controlUnit(input [10:0] Instr,
                   input BeqFlag,
                   output  [1:0] AdrRd,
                   output  [1:0] AdrRa,
                   output  reg [1:0] AdrRb,
                   output  [3:0] imm,
                   output [6:0] Adr,
                   output reg[1:0] ALUControl,
                   output reg MemToReg, MemWrite, RegWrite, ALUSrc, J,initRd,
                   output Beq);
  
  reg Beq_;
  always @(*) begin
    case(Instr[10:8])//opcode
      //register operations
      3'b000: begin
        	MemToReg=1'b0;MemWrite=1'b0;RegWrite=1'b1; ALUSrc=1'b0; Beq_=1'b0; J=1'b0; ALUControl = Instr[1:0]; AdrRb = Instr[3:2];initRd=1'b0;
      end
      //addi
      3'b001: begin
       	 MemToReg=1'b0;MemWrite=1'b0;RegWrite=1'b1; ALUSrc=1'b1; Beq_=1'b0; J=1'b0; ALUControl=2'b00;  AdrRb = Instr[3:2];initRd=1'b0;
      end
      // lw (load into rd fromm dmem)
      3'b010: begin
        MemToReg=1'b1;MemWrite=1'b0;RegWrite=1'b1; ALUSrc=1'b1; Beq_=1'b0; J=1'b0; ALUControl=2'b00;  AdrRb = Instr[3:2];initRd=1'b0;
      end
      // sw (store into dmem from rd)
      3'b011: begin
        MemToReg=1'b0;MemWrite=1'b1;RegWrite=1'b0; ALUSrc=1'b1; Beq_=1'b0; J=1'b0; ALUControl=2'b00;  AdrRb = Instr[7:6];initRd=1'b0;
      end
      //beq 
      3'b100: begin
        MemToReg=1'b0;MemWrite=1'b0;RegWrite=1'b0; ALUSrc=1'b1; Beq_=1'b1; J=1'b0; ALUControl=2'b00;  AdrRb = Instr[3:2];initRd=1'b0;
      end
      //j
      3'b101: begin
         MemToReg=1'b0;MemWrite=1'b0;RegWrite=1'b0; ALUSrc=1'b0; Beq_=1'b0; J=1'b1; ALUControl=2'b00;  AdrRb = Instr[3:2];initRd=1'b0;
      end
      //initRd initialise Rd in regfile to imm8
      3'b110: begin
        MemToReg=1'b0;MemWrite=1'b0;RegWrite=1'b1; ALUSrc=1'b0; Beq_=1'b0; J=1'b0; ALUControl=2'b00;  AdrRb = Instr[3:2]; initRd=1'b1;initRd=1'b1;
      end 
      default:  begin
        MemToReg=1'b0;MemWrite=1'b0;RegWrite=1'b0; ALUSrc=1'b0; Beq_=1'b0; J=1'b0; ALUControl=2'b00;  AdrRb = Instr[3:2];initRd=1'b0;
      end
    endcase
    
  end// always @(*)
   assign Beq = Beq_ &  BeqFlag;//pc should branch only if branch flag is 
  //set in previous clock cycle
  assign AdrRd = Instr[7:6];
    assign AdrRa = Instr[5:4];
    
  assign imm = Instr[3:0];
    assign Adr = Instr[6:0];
    
    endmodule

//ALU
module ALU(input [3:0] SrcA, SrcB,
           input [1:0] ALUControl,
           output reg [3:0] ALUResult,
           output reg BeqFlag);
  
  wire [3:0] condinvb;
  wire [4:0] sum;
// wire ofw1;
 //wire ofw0;
 //wire Cout;
  
 assign condinvb = ALUControl[0] ? ~{1'b0,SrcB} :{1'b0,SrcB};
  assign sum = {1'b0,SrcA} + condinvb + ALUControl[0];
 //assign Cout = sum[8];
//assign ofw1 = ~(SrcA[7] ^ SrcB[7] ^ ALUControl[0]);
 // assign ofw0 = SrcA[7] ^ sum[7] ;
 
  
  always @(*)
  begin 
    
    case(ALUControl)
      2'b01: ALUResult = sum[3:0];
      2'b00: ALUResult = sum[3:0];//add
     2'b10: ALUResult = SrcA & SrcB;
     2'b11: ALUResult = SrcA | SrcB;
    endcase
  
  //BeqFlag
    case(ALUResult)
      8'h00: BeqFlag = 1'b1;//flag set is result is zero
      default: BeqFlag = 1'b0;
    endcase
  end//always@(*)
  
endmodule      

//adder
module adder #(parameter WIDTH=8)
  (input [WIDTH-1:0] a,b,
   output [WIDTH-1:0] y);
  assign y=a+b;
endmodule

// regfile
module regfile( input clk,
		input RegWrite,
               input  [3:0] Rd,
               output [3:0] Ra, Rb,
               input  [1:0] AdrRd,
                   input  [1:0] AdrRa,
                   input  [1:0] AdrRb);

  reg [3:0] rf[3:0];//total of four 4bit registers
  //initial rf[1]=8'b1111;
//3 ported register file
//read 2 ports combinationally
//write 3rd port on rising clock edge
//register 15 reads PC+8 instead

always @(posedge clk)
  begin
    if (RegWrite) rf[AdrRd]<=Rd;
  end
  assign Ra=rf[AdrRa];
  assign Rb=rf[AdrRb];
  
endmodule

//extendJ
module extendJ(input [6:0] Adr,
             
               output [7:0] Adrx2);
  
  assign Adrx2={Adr,1'b0};
endmodule

//extend 4 it imm to 8 bit imm8
module extendImm(input [3:0] imm,
             
                 output [7:0] imm8);
  
  assign imm8={4'b0,imm};
endmodule


//pcMux
module pcmux(input [7:0] PCplusImm,Adrx2,PCplus1,
             input [1:0] BJ,
             output reg[7:0] PC);
  always @(*) begin
    case(BJ)
      2'b10: PC=PCplusImm;
      2'b01: PC=Adrx2;
      default: PC=PCplus1;
    endcase
  end
endmodule
                
//memor elements
//flopenr
module flopenr #(parameter WIDTH = 4)
  (input clk,reset,en,
   input [WIDTH-1:0] d,
   output reg [WIDTH -1:0] q);
  always @(posedge clk, posedge reset)
    begin
    if (reset) q<=0;
  else if (en) q<=d;
    end
endmodule

//flopr
module flopr #(parameter WIDTH = 4)
  (input clk,reset,
   input [WIDTH-1:0] d,
   output reg [WIDTH -1:0] q);
  always @(posedge clk, posedge reset)
    begin
    if (reset) q<=0;
  else  q<=d;
    end
endmodule

//datapath
module datapath(   input clk,reset,
  				   output BeqFlag,
                   input  [1:0] AdrRd,
                   input  [1:0] AdrRa,
                   input  [1:0] AdrRb,
                   input  [3:0] imm,
                   input [6:0] Adr,
                   input [1:0] ALUControl,
                   input MemToReg, MemWrite, RegWrite, ALUSrc, J,
                   input Beq,
                output [7:0] PC, 
                output [3:0] ALUResult, WriteData,
                input [3:0] ReadData,
               input initRd);
  
  wire BeqFlag_preclk;
  wire [7:0] PCplusImm,Adrx2,PCplus1,PCpre,imm8;
  wire [1:0] BJ;
  wire [3:0] Result,Ra;
  wire [3:0] SrcB;
  wire [3:0] Result_;
  assign BJ={Beq,J};
  
  //PC logic
  pcmux Pcmux (.PCplusImm(PCplusImm),.Adrx2(Adrx2),.PCplus1(PCplus1),.BJ(BJ),.PC(PCpre));
  flopr #(8) PCreg (.clk(clk),.reset(reset), .d(PCpre),.q(PC) );
  adder adder1(.a(PC),.b(imm8),.y(PCplusImm));
  adder adder2(.a(PC), .b(8'b1),.y(PCplus1));
  extendImm extendImm(.imm(imm), .imm8(imm8));
  extendJ extendJ(.Adr(Adr), .Adrx2(Adrx2));
  
  //regfile logic
  regfile regfile( .clk(clk),.RegWrite(RegWrite),.Rd(Result),.Ra(Ra), .Rb(WriteData), .AdrRd(AdrRd),.AdrRa(AdrRa), .AdrRb(AdrRb) );
 
  //register or immediate vALUe for ALU?
  assign SrcB=(ALUSrc==1'b1) ? imm :WriteData;
  
  //ALU logic
  ALU  ALU(.SrcA(Ra), .SrcB(SrcB), .ALUControl(ALUControl), .ALUResult (ALUResult),.BeqFlag(BeqFlag_preclk) );
  
  flopr #(1) flagreg(.clk(clk), .reset(reset), .d(BeqFlag_preclk), .q(BeqFlag) );
   
  //generate reg Result_
  assign Result_ = (MemToReg==1'b1) ? ReadData : ALUResult;
  
  //take Result_ value or initialise Rd to imm8?
  assign Result = (initRd==1'b1) ? imm : Result_;
  
endmodule
  
module processor(input clk, reset,
                 output [7:0] PC,
                 input [10:0] Instr,
                 output MemWrite,BeqFlag,
                 output [3:0] ALUResult,WriteData,
                 input [3:0] ReadData,
                 output [1:0] ALUControl,
                 output  [1:0] AdrRd,
                 output [1:0] AdrRa,
                 output  [1:0] AdrRb,
                output Beq);
  
 // wire MemWrite,BeqFlag;
    wire MemToReg, MemWrite, RegWrite, ALUSrc, J, initRd;
 // wire  [1:0] AdrRd;
 // wire  [1:0] AdrRa;
//  wire  [1:0] AdrRb;
  wire  [3:0] imm;
  wire [6:0] Adr;
  //wire[1:0] ALUControl;
  
  controlUnit cu(.Instr(Instr), .BeqFlag(BeqFlag),.AdrRd(AdrRd),.AdrRa(AdrRa), .AdrRb(AdrRb),.imm(imm),.Adr(Adr),.ALUControl(ALUControl),.MemToReg(MemToReg), .MemWrite(MemWrite), .RegWrite(RegWrite), .ALUSrc(ALUSrc), .J(J), .Beq(Beq), .initRd(initRd) );
    
  datapath dp( .clk(clk),.reset(reset),.BeqFlag(BeqFlag),.AdrRd(AdrRd), .AdrRa(AdrRa), .AdrRb(AdrRb), .imm(imm), .Adr(Adr), .ALUControl(ALUControl), .MemToReg(MemToReg), .MemWrite(MemWrite), .RegWrite(RegWrite), .ALUSrc(ALUSrc), .J(J), .Beq(Beq), .PC(PC), .ALUResult(ALUResult), .WriteData(WriteData), .ReadData(ReadData) ,.initRd(initRd));
    
  endmodule
          
