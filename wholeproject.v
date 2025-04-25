module Adder (
    input wire signed [31:0]input1, 
    input wire signed [31:0]input2,
    output wire [31:0] result
);
assign result = input1 + input2 ;
endmodule
module ALU32Bit(
    input wire signed [31:0] data1, 
    input wire signed [31:0] data2, 
    input wire [3:0] ALUControl, 
    input wire [4:0] shiftAmount,
    input wire reset,
    output reg overflow, 
    output reg zero, 
    output reg signed [31:0] result
);
wire [31:0] neg_data2;
assign neg_data2 = -data2;
always @(posedge reset)begin 
    zero <= 1'b0;//reser zero flag
end
always @(ALUControl, data1, data2) begin
    case (ALUControl)
        4'b0000: begin // add
            result <= data1 + data2;
            if (data1[31] == data2[31] && result[31] == ~data1[31]) begin
                overflow <= 1'b1;
            end
            else begin
                overflow <= 1'b0;
            end
        end
        4'b0001: begin // sub also used for branch
            result <= data1 + neg_data2;
            #50
            if (result == 0 )begin 
            end
            else begin
            end
        end
        4'b1000: begin // less
            if (data1 < data2)begin
                result <= 1;
            end
            else begin
                result <= 0;
            end
        end
    endcase
    if (data1 == data2) begin
        zero <= 1'b1;
    end
    else
        zero <= 1'b0;
end
endmodule
module ALUControl(
    input [3:0] ALUOp,
    input [5:0] funct,
	output reg [3:0] ALUControl
);
always @(*)begin
        case (ALUOp)
            4'b0000: begin
				ALUControl = 4'b0000;//set for addition
			end
            4'b0001: begin 
				ALUControl = 4'b0001;//set for subtraction
			end
            4'b0101: begin 
				ALUControl = 4'b1000;//set for less than
			end
            4'b0010: begin
                case (funct)
                    6'b100000: begin 
						ALUControl = 4'b0000;//R type add
					end
                    6'b100010: begin 
						ALUControl = 4'b0001; //R type subtract
					end
                    6'b101010: begin 
						ALUControl = 4'b1000;//R type less than
					end
                    default: ALUControl = 4'bxxxx;
                endcase
            end
            default: ALUControl = 4'bxxxx;
        endcase
    end
endmodule
module Comparator(
    input [31:0]input1 ,
    input [31:0]input2 ,
    output result
);
assign result = (input1 == input2) ? 1'b1 : 1'b0;
endmodule

module ControlUnit (
  input [5:0] opCode ,
  output reg registerDestination,
  output reg branch,
  output reg memoryRead,
  output reg memoryToRegister,
  output reg [3:0] ALUop,
  output reg memoryWrite,
  output reg AluSrc,
  output reg registerWrite,
  input reset
); 
  always @(posedge reset)begin
   registerDestination <= 1'b0;
   branch <= 1'b0;
   memoryRead <= 1'b0;
   memoryToRegister <= 1'b0;
   ALUop <= 4'b0000;
   memoryWrite <= 1'b0;
   AluSrc <= 1'b0;
   registerWrite <= 1'b0;
  end
  always@(opCode) begin
      case (opCode)
        6'b000000:    // R type       
          begin
          registerDestination<=1 ;
          branch<=0 ;
          memoryRead<=0 ;
          memoryToRegister<=0 ;
          memoryWrite<=0 ;
          AluSrc<=0 ;
          registerWrite<=1 ;
          ALUop<=4'b0010 ;
          end
        6'b001010: //slti
        begin
          registerDestination<=0 ;
          branch<=0 ;
          memoryRead<=0 ;
          memoryToRegister<=0 ;
          memoryWrite<=0 ;
          AluSrc<=1 ;
          registerWrite<=1 ;
          ALUop<=4'b0101 ;
        end
        6'b100011:  begin // lw
          registerDestination<=0 ;
          branch<=0 ;
          memoryRead<=1 ;
          memoryToRegister<=1 ;
          memoryWrite<=0 ;
          AluSrc<=1 ;
          registerWrite<=1 ;
          ALUop<=4'b0000 ;
          end
        6'b101011:  begin   //sw      
          branch<=0 ;
          memoryRead<=0 ;
          memoryToRegister<=0 ;
          memoryWrite<=1 ;
          AluSrc<=1 ;
          registerWrite<=0 ;
          ALUop<=4'b0000 ;
          end
        6'b000100:     //beq      
          begin
          branch<= 1;
          memoryRead<=0 ;
          memoryToRegister<=0 ;
          memoryWrite<=0 ;
          AluSrc<=0 ;
          registerWrite<=0 ;
          ALUop<=4'b0001 ;
          end
      endcase
  end 
endmodule

module DataMemory (
  input clock,
  input memoryWrite,
  input memoryRead,
  input [31:0] address,
  input [31:0] writeData,
  output reg [31:0]readData
);
reg[31:0] memory [0:1023];
initial begin
// f(0)
memory[0] = 32'h00000000;
// f(1)
memory[1] = 32'h00000001;
end
always@(negedge clock) begin
    if(memoryWrite==1) begin 
        memory[address]<=writeData;
    end
end
always@(address or memoryRead)begin
    if(memoryRead==1) begin 
        readData=memory[address]; 
    end
end
endmodule

module DataMemory2 (
  input memoryWrite1,
  input memoryWrite2,
  input memoryRead1,
  input memoryRead2,
  input [31:0] address1,
  input [31:0] address2,
  input [15:0] writeData1,
  input [15:0] writeData2,
  input clock,
  output reg [15:0] readData1,
  output reg [15:0] readData2
);
  reg[31:0] memory [0:31];
  always@(negedge clock) begin
      if(memoryWrite1==1)begin
        memory[address1][31:16]<=writeData1;//write upper half of memory
      end
      if(memoryWrite2==1)begin
        memory[address2][15:0]<=writeData2;
      end
  end
  always@(address1 or address2 or memoryRead1 or memoryRead2)begin
      if(memoryRead1==1)begin
        readData1=memory[address1][31:16]; //read upper half of memory
      end
      if(memoryRead2==1)begin
        readData2=memory[address2][15:0];
      end
  end
endmodule

module EX_MemReg (
  input clock,
  input registerWrite,
  input memoryToRegister,
  input memoryWrite,
  input memoryRead,
  input [31:0] ALUresult,
  input [31:0] writeData,
  input [4:0] writeRegister,
  output reg registerWriteOut,
  output reg memoryToRegisterOut,
  output reg memoryWriteOut,
	output reg memoryReadOut,
  output reg [31:0] ALUresultOut,
  output reg [31:0] writeDataOut,
  output reg [4:0] writeRegisterOut
);
always@(posedge clock)begin
  //pass elements like writedata or memory to output
      writeDataOut <= writeData;
      memoryToRegisterOut <= memoryToRegister;
      writeRegisterOut <= writeRegister;    
      registerWriteOut <= registerWrite;
      memoryWriteOut <= memoryWrite;
      memoryReadOut <= memoryRead;
      ALUresultOut <= ALUresult;
end
endmodule

module ForwardingUnit (
  input EX_MemRegwrite,
  input [4:0] EX_MemWriteReg,
  input Mem_WbRegwrite,
  input [4:0] Mem_WbWriteReg,
  input [4:0] ID_Ex_Rs,
  input [4:0] ID_Ex_Rt,
  output reg [1:0] upperMux_sel,
  output reg [1:0] lowerMux_sel,
  output reg [1:0] comparatorMux1Selector,
  output reg [1:0] comparatorMux2Selector
);
  always@(EX_MemRegwrite or EX_MemWriteReg or Mem_WbRegwrite or Mem_WbWriteReg or ID_Ex_Rs or ID_Ex_Rt)begin
    //forwarding from ALU to ALU & from ALU to ID stage
      if(EX_MemRegwrite && EX_MemWriteReg)  begin
          if (EX_MemWriteReg==ID_Ex_Rs)begin
            upperMux_sel<=2'b10;//forward from ex to upper mux
            comparatorMux1Selector<=2'b01;//forward to comparator
          end
          else //no forwarding
          begin
          upperMux_sel<=2'b00;//no forwarding
          comparatorMux1Selector<=2'b00;
          end  
          if(EX_MemWriteReg==ID_Ex_Rt)begin
            lowerMux_sel<=2'b10;
            comparatorMux2Selector<=2'b01;
          end
          else //no forwarding
          begin
          lowerMux_sel<=2'b00;//forward from ex to lower mux
          comparatorMux2Selector<=2'b00;
          end
      end
      else if (Mem_WbRegwrite && Mem_WbWriteReg)//forwarding from Memorystage to ALU ID stage
        begin
          if ((Mem_WbWriteReg==ID_Ex_Rs) && (EX_MemWriteReg!=ID_Ex_Rs))
            begin
            upperMux_sel<=2'b01;//forward from mem stage to upper mux
            comparatorMux1Selector<=2'b10;
            end
          else //no forwarding
          begin
          upperMux_sel<=2'b00;
          comparatorMux1Selector<=2'b00;
          end 
          if((Mem_WbWriteReg==ID_Ex_Rt) && (EX_MemWriteReg==ID_Ex_Rt) )
          begin
            lowerMux_sel<=2'b01;//forward from mem to lower mux
            comparatorMux2Selector<=2'b10;
          end
          else //no forwarding
          begin
          lowerMux_sel<=2'b00;
          comparatorMux2Selector<=2'b00;
          end
        end
      else begin
         //No forwarding             
          upperMux_sel<=2'b00;
          lowerMux_sel<=2'b00;
          comparatorMux1Selector<=2'b00;
          comparatorMux2Selector<=2'b00;
          
       end
        
  end
endmodule
module HazardDetectionUnit(
  input ID_ExMemRead,
  input EX_MemMemRead,
  input [4:0] ID_Ex_Rt,
  input [31:0] IF_ID_Instr,
  output reg holdPC,
  output reg holdIF_ID,
  output reg muxSelector
);
  initial begin
	holdPC <= 0;
	holdIF_ID <= 0;
	muxSelector <= 0;
	end
  always@(ID_ExMemRead or ID_Ex_Rt or IF_ID_Instr) begin
      if (ID_ExMemRead && (holdPC == 1'b0) && (holdIF_ID == 1'b0)) begin
          if(ID_Ex_Rt==IF_ID_Instr[25:21] || ID_Ex_Rt==IF_ID_Instr[20:15] )begin
              holdPC<=1;
              holdIF_ID<=1;
              muxSelector<=1;
          end
      end
        // beq opcode 
      else if((IF_ID_Instr [31:26]==6'b000100) && (holdPC == 1'b0) && (holdIF_ID == 1'b0)) begin
          holdPC<=1;
          holdIF_ID<=1;
          muxSelector<=1;
      end
      else begin
          holdPC<=0;
          holdIF_ID<=0;
          muxSelector<=0;     
      end    
    end
endmodule

module ID_EX_reg (
  input wire clock , 
  input wire registerWrite,
  input wire memoryToRegister,
  input wire memoryWrite,
  input wire memoryRead,
  input wire ALUSrc,
  input wire[3:0] ALUOp,
  input wire registerDestination,
  input wire [31:0] PCplus4 ,
  input wire [31:0] data1Input ,
  input wire [31:0] data2Input,
  input wire [31:0] signExtendResultInput ,
  input wire[14:0] registerAddressInput ,
  output reg [31:0] PCplus4out ,
  output reg [31:0] data1Output ,
  output reg [31:0] data2Output ,
  output reg [31:0] signExtendResultOutput ,
  output reg [4:0]rsOut ,
  output reg [4:0]rtOut ,
  output reg [4:0]rdOut,
  output reg registerWriteOutput,
  output reg memoryToRegisterOutput,
  output reg memoryWriteOutput,
  output reg memoryReadOutput,
  output reg ALUSrcOut,
  output reg [3:0]ALUOpOut,
  output reg registerDestinationOut
);
  always @(posedge clock)
  //pass inputs to outputs on clk
    begin
      PCplus4out <= PCplus4;
      data1Output <= data1Input;
      data2Output <= data2Input;
      signExtendResultOutput <= signExtendResultInput;
      rsOut <= registerAddressInput[14:10];
      rtOut <= registerAddressInput[9:5];
      rdOut <= registerAddressInput[4:0];
      registerWriteOutput <= registerWrite;
      memoryToRegisterOutput <= memoryToRegister;
      memoryWriteOutput <= memoryWrite;
      memoryReadOutput <= memoryRead;
      ALUSrcOut <= ALUSrc;
      ALUOpOut <= ALUOp;
      registerDestinationOut <= registerDestination;
    end
endmodule

module IF_ID_reg(
  input clk ,
  input wire[31:0] PCplus4 ,
  input wire[31:0] instrIn ,
  output reg [31:0] instrOut ,
  input hold,
  output reg[31:0] PCplus4Out,
  input IF_flush
);
  always @(posedge clk)
    begin
      if (hold==1'b0) begin   
      PCplus4Out<=PCplus4;
      instrOut <= instrIn;   
      end
      else if (IF_flush==1'b1) begin
          PCplus4Out<=PCplus4; 
          instrOut<=32'b0;
      end
    end
endmodule

module IF_PR_reg(
    input clk, hold, IF_flush,
    input wire [31:0] PCplus4, instrIn,
    output reg [31:0] instrOut, PCplus4Out
);

    always @(posedge clk) begin
        if(hold == 1'b0) begin
            PCplus4Out <= PCplus4;
            instrOut <= instrIn;
        end else if (IF_flush == 1'b1) begin
            PCplus4Out <= PCplus4;
            
        end
    end

endmodule


module PR_ID_Reg(
    input clk, stall, IF_flush,
    input wire [31:0] PC, Instruction, Read_Data_1, Read_Data_2,
    output reg [31:0] PC_out, Instruction_out, Read_Data_1_out, Read_Data_2_out
);

    always @(posedge clk) begin
        if(stall == 1'b0) begin
            PC_out <= PC;
            Instruction_out <= Instruction;
            Read_Data_1_out <= Read_Data_1;
            Read_Data_2_out <= Read_Data_2;
        end else if (IF_flush == 1'b1) begin
            PC_out <= PC;
            Instruction_out <= 32'd0;//flush instructions
            Read_Data_1_out <= 32'd0;
            Read_Data_2_out <= 32'd0;
        end
    end


endmodule





module InstructionMemory(
	input clock,
	input [31:0] pc ,
	output reg [31:0] readData
);
reg [31:0] instructionMemory [0:1060];
initial begin
	instructionMemory[0] = 32'h8C000000 ; //lw 
	instructionMemory[4] = 32'h8C010001 ; //lw 
	instructionMemory[8] = 32'h03FEF020 ; // i++
	instructionMemory[12] = 32'h00011020 ; // add 2
	instructionMemory[16] = 32'h00221820 ; // add 3 
	instructionMemory[20] = 32'h03FEF020 ; // i++
	instructionMemory[24] = 32'h00622020; // add 4
	instructionMemory[28] = 32'h03FEF020 ; // i++
	instructionMemory[32] = 32'h00832820; // add 5
	instructionMemory[36] = 32'h03FEF020 ; // i++
	instructionMemory[40] = 32'h28AA0006 ; //lsti
	// 0010 1000 1010 1010 0000 0000 0000 0110
	instructionMemory[44] = 32'h13C50002 ; // beq
	instructionMemory[48] = 32'hAC210000 ; // sw
	// 1010 1100 0010 0001 0000 0000 0000 0000
end
always @ (pc)begin	
	readData <= instructionMemory[pc];
end				
endmodule	

module InstructionMemory2(
	input clk,
	input [31:0] pc,
	output reg [31:0] readdata
);
reg [31:0] instructionMemory [0:1023];

initial begin
		instructionMemory[0] = 32'h01095020 ;
		instructionMemory[1] = 32'hAC0A0000 ;
		instructionMemory[2] = 32'h01495822 ;
		instructionMemory[3] = 32'h1168FFFC ;  
		instructionMemory[4] = 32'hAC0A0000 ;
end


always @ (pc)begin	 
	readdata <= instructionMemory[pc>>2];
end			
		
endmodule	


module Mem_WbReg(
  input clock,
  input registerWrite, 
  input memoryToRegister,
  input [31:0] ALUresult,
  input [31:0] readData,
  input [4:0] writeRegister,
  output reg registerWriteOut,
  output reg memoryToRegisterOut,
  output reg[31:0] readDataOut,
  output reg [31:0] ALUresultOut,
  output reg [4:0] writeRegisterOut
);
always@(posedge clock)begin
      registerWriteOut<=registerWrite;
      memoryToRegisterOut<=memoryToRegister;
      readDataOut<=readData;
      ALUresultOut<=ALUresult;
      writeRegisterOut<=writeRegister;
end  
endmodule


module Mem1_Mem2_Reg (
  MemWriteMEM, MemWrite2, MemReadMEM, MemRead2, ALUResultMEM, address2, 
memoryWriteDataMEM, memoryWriteData2, clk, memoryReadData1, memoryReadDataMEM, 
RegWriteMEM, RegWriteMEM2, MemtoRegMEM, MemtoRegMEM2,
writeRegMEM, writeRegMEM2, RegWriteWB, RegWriteWB2, writeRegWB, writeRegWB2);
  input clk;
  input MemWriteMEM, MemReadMEM, RegWriteMEM, MemtoRegMEM, RegWriteWB;
  input [4:0] writeRegMEM, writeRegWB;
  input [15:0] memoryWriteDataMEM, memoryReadData1;
  input [31:0] ALUResultMEM;

  output reg MemWrite2, MemRead2, RegWriteMEM2, MemtoRegMEM2, RegWriteWB2;
  output reg [4:0] writeRegMEM2, writeRegWB2;
  output reg [15:0] memoryWriteData2, memoryReadDataMEM;
  output reg [31:0] address2;

  
  always@(posedge clk)
    begin
      
        MemWrite2 <= MemWriteMEM;
        MemRead2 <= MemReadMEM; 
        RegWriteMEM2 <= RegWriteMEM; 
        MemtoRegMEM2 <= MemtoRegMEM;
        RegWriteWB2 <= RegWriteWB;
        writeRegMEM2 <= writeRegMEM;
        writeRegWB2 <= writeRegWB;
        memoryWriteData2 <= memoryWriteDataMEM;
        memoryReadDataMEM <= memoryReadData1;
        address2 <= ALUResultMEM;
      
    end
  
  
endmodule

module Mux2x1_5Bits(
    output reg [4:0] result,
    input [4:0] input1,
    input [4:0] input2,
    input select
);
always @(input1, input2, select) begin
    case(select)
        1'b0: begin 
			result = input1;
		end 
		1'b1: begin 
			result = input2;
		end
		default: result = 5'bx;
    endcase
end
endmodule
module Mux2x1_10Bits(
    output reg [9:0] result,
    input [9:0] input1,
    input [9:0] input2,
    input select
);
always @(input1, input2, select) begin
    case(select)
        1'b0: begin 
			result = input1; 
		end
        1'b1: begin 
			result = input2;
		end
        default: result = 10'bx;
    endcase
end
endmodule
module Mux2x1_32Bits(
    output reg [31:0] result,
    input [31:0] input1,
    input [31:0] input2,
    input select
);
always @(input1, input2, select) begin
    case(select)
        1'b0: begin 
			result = input1; 
		end
        1'b1: begin 
			result = input2; 
		end
        default: result = 32'bx;
    endcase
end

endmodule
module Mux3x1_32Bits(
    output reg [31:0] result,
    input [31:0] input1, 
	input [31:0] input2, 
	input [31:0] input3,
    input [1:0] select
);
always @(input1, input2, input3, select) begin
    case(select)
        2'b00: begin 
			result = input1;
		end
        2'b01: begin 
			result = input2;
		end
        2'b10: begin
			 result = input3;
		end
        default: result = 32'bx;
    endcase
end

endmodule
module PC (
  input clock , 
  input wire [31:0] nextPC ,
  output reg [31:0] outPC ,
  input reset ,
  input holdPC
);
always@(posedge reset) begin
  outPC <= 32'hFFFFFFFC;
end
always @(posedge clock)begin
  //to support stalls from hazard detection unit
	if (holdPC==0) begin
    outPC <= nextPC;
	end
end
endmodule


module RegisterFile(
	input clock,
	input [4:0] readRegister1, 
	input [4:0] readRegister2, 
	input [4:0] RegisterAddress, 
	input [31:0] WriteData,
	input writeSignal, 
	output reg [31:0] ReadData1, 
	output reg [31:0] ReadData2, 
	input reset
);
    reg [31:0] registers[0:31];
	always @(posedge reset) begin
		registers[0] <= 32'h00000000;
		registers[1] <= 32'h00000000;
		// saving i 
		registers[30] <= 32'h00000001;
		// saving constant 1
		registers[31] <= 32'h00000001 ;
	end
	always @(readRegister1, readRegister2) begin
		ReadData1 <= registers[readRegister1];
  		ReadData2 <= registers[readRegister2];
	end
	always @(negedge clock) begin
  		if (writeSignal == 1)begin
			registers[RegisterAddress] <= WriteData;
      	end
  	end

endmodule
module ShiftLeft2(
    output [31:0] result,
    input [31:0] input1
);
assign result = input1 << 2;
endmodule
module SignExtend (
    input [15:0] input1,
    output reg [31:0] result
);
always @(input1) begin
    if (input1[15] == 1)  begin     
        result = {16'hffff, input1};
    end
    else if (input1[15] == 0) begin
         result = {16'h0000, input1};
    end
    else begin
        result = 32'hxxxx_xxxx;
    end
end
endmodule
