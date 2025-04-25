module Summation (
    input wire signed [31:0] operand1, 
    input wire signed [31:0] operand2,
    output wire [31:0] outputSum
);
assign outputSum = operand1 + operand2;
endmodule

module ArithmeticLogicUnit(
    input wire signed [31:0] input1, 
    input wire signed [31:0] input2, 
    input wire [3:0] operationCode, 
    input wire [4:0] shiftBits,
    input wire resetSignal,
    output reg carryOut, 
    output reg isZero, 
    output reg signed [31:0] outputResult
);
wire [31:0] invertedInput2;
assign invertedInput2 = -input2;//invert for subtraction 

always @(posedge resetSignal) begin 
    isZero <= 1'b0;//reset flag on reset signal
end

always @(operationCode, input1, input2) begin
    case (operationCode)
        4'b0000: begin // addition
            outputResult <= input1 + input2;
            if (input1[31] == input2[31] && outputResult[31] == ~input1[31]) begin
                carryOut <= 1'b1;//set carry out
            end else begin
                carryOut <= 1'b0;//not have carry out
            end
        end
        4'b0001: begin // subtraction
            outputResult <= input1 + invertedInput2;
        end
        4'b1000: begin // less than
            outputResult <= (input1 < input2) ? 1 : 0;
        end
    endcase
    isZero <= (input1 == input2) ? 1'b1 : 1'b0;
end
endmodule

module OperationDecoder(
    input [3:0] controlSignal,
    input [5:0] functionCode,
    output reg [3:0] operationOutput
);
always @(*) begin
    case (controlSignal)
        4'b0000: operationOutput = 4'b0000; // add
        4'b0001: operationOutput = 4'b0001; // sub
        4'b0101: operationOutput = 4'b1000; // less
        4'b0010: begin
            case (functionCode)
                6'b100000: operationOutput = 4'b0000; // add
                6'b100010: operationOutput = 4'b0001; // sub
                6'b101010: operationOutput = 4'b1000; // less
                default: operationOutput = 4'bxxxx;
            endcase
        end
        default: operationOutput = 4'bxxxx;
    endcase
end
endmodule

module BranchPredictor (
    input     clockSignal,
    input     resetSignal,
    input    isBranchValid,
    input   isBranchTaken,
    output   branchPrediction
);

    localparam HISTORY_SIZE = 7;//size of histroy register
    localparam TABLE_SIZE = (1 << HISTORY_SIZE);//size of predictor table
//based on previous experiences predict
    reg [1:0] predictorTable [0:TABLE_SIZE-1];
    reg [HISTORY_SIZE-1:0] globalHistoryRegister;

    wire [1:0] currentPrediction;
    assign currentPrediction = predictorTable[globalHistoryRegister];

    assign branchPrediction = (currentPrediction >= 2);//prediction will be taken or not taken
//we predict based on strentgh of previous predictions
    integer idx;
    always @(posedge clockSignal) begin
        if (resetSignal) begin
            globalHistoryRegister <= {HISTORY_SIZE{1'b0}};//reset history register
            for (idx = 0; idx < TABLE_SIZE; idx = idx + 1) begin
                predictorTable[idx] <= 2;//initialize predictor table
            end
        end else if (isBranchValid) begin
            if (isBranchTaken) begin
                if (predictorTable[globalHistoryRegister] < 2'b11)
                    predictorTable[globalHistoryRegister] <= predictorTable[globalHistoryRegister] + 1;//update prediction
            end else begin
                if (predictorTable[globalHistoryRegister] > 2'b00)
                    predictorTable[globalHistoryRegister] <= predictorTable[globalHistoryRegister] - 1;
            end

            globalHistoryRegister <= { globalHistoryRegister[HISTORY_SIZE-2:0], isBranchTaken };//update history
        end
    end

endmodule

module EqualityChecker(
    input [31:0] firstInput,
    input [31:0] secondInput,
    output comparisonResult
);
assign comparisonResult = (firstInput == secondInput) ? 1'b1 : 1'b0;
endmodule

module ControlSignalGenerator (
    input [5:0] opcode,
    output reg destReg,
    output reg isBranch,
    output reg readMemory,
    output reg writeToReg,
    output reg [3:0] ALUOperation,
    output reg writeMemorySignal,
    output reg sourceSelect,
    output reg writeRegSignal,
    input resetSignal
); 
always @(posedge resetSignal) begin
    destReg <= 1'b0; //reset signals
    isBranch <= 1'b0;
    readMemory <= 1'b0;
    writeToReg <= 1'b0;
    ALUOperation <= 4'b0000;
    writeMemorySignal <= 1'b0;
    sourceSelect <= 1'b0;
    writeRegSignal <= 1'b0;
end

always @(opcode) begin
    case (opcode)
        6'b000000: begin // R-type
            destReg <= 1;
            isBranch <= 0;
            readMemory <= 0;
            writeToReg <= 0;
            writeMemorySignal <= 0;
            sourceSelect <= 0;//select immediate source
            writeRegSignal <= 1;
            ALUOperation <= 4'b0010;
        end
        6'b001010: begin // slti
            destReg <= 0;
            isBranch <= 0;
            readMemory <= 0;
            writeToReg <= 0;
            writeMemorySignal <= 0;
            sourceSelect <= 1;
            writeRegSignal <= 1;
            ALUOperation <= 4'b0101;
        end
        6'b100011: begin // lw
            destReg <= 0;
            isBranch <= 0;
            readMemory <= 1;
            writeToReg <= 1;
            writeMemorySignal <= 0;
            sourceSelect <= 1;
            writeRegSignal <= 1;
            ALUOperation <= 4'b0000;
        end
        6'b101011: begin // sw
            isBranch <= 0;
            readMemory <= 0;
            writeToReg <= 0;
            writeMemorySignal <= 1;
            sourceSelect <= 1;
            writeRegSignal <= 0;
            ALUOperation <= 4'b0000;
        end
        6'b000100: begin // beq
            isBranch <= 1;
            readMemory <= 0;
            writeToReg <= 0;
            writeMemorySignal <= 0;
            sourceSelect <= 0;
            writeRegSignal <= 0;
            ALUOperation <= 4'b0001;
        end
    endcase
end 
endmodule

module MemoryUnit (
    input clockSignal,
    input writeMemorySignal,
    input readMemorySignal,
    input [31:0] address,
    input [31:0] dataToWrite,
    output reg [31:0] dataRead
);
reg[31:0] memoryArray [0:1023];
initial begin
    memoryArray[0] = 32'h00000000;//initialize memory
    memoryArray[1] = 32'h00000001;
end

always @(negedge clockSignal) begin
    if(writeMemorySignal) begin 
        memoryArray[address] <= dataToWrite;//write data to memory
    end
end

always @(address or readMemorySignal) begin
    if(readMemorySignal) begin 
        dataRead = memoryArray[address]; //read data from memory
    end
end
endmodule

module DualDataMemory (
    input writeMemory1,
    input writeMemory2,
    input readMemory1,
    input readMemory2,
    input [31:0] address1,
    input [31:0] address2,
    input [15:0] dataToWrite1,
    input [15:0] dataToWrite2,
    input clockSignal,
    output reg [15:0] dataRead1,
    output reg [15:0] dataRead2
);
reg[31:0] memoryArray [0:31];

always @(negedge clockSignal) begin
    if(writeMemory1) begin
        memoryArray[address1][31:16] <= dataToWrite1;//write upper half to memory
    end
    if(writeMemory2) begin
        memoryArray[address2][15:0] <= dataToWrite2;//write lower half to memory
    end
end

always @(address1 or address2 or readMemory1 or readMemory2) begin
    if(readMemory1) begin
        dataRead1 = memoryArray[address1][31:16]; //read upper half of memory
    end
    if(readMemory2) begin
        dataRead2 = memoryArray[address2][15:0];//read lower half of memory
    end
end
endmodule

module ExecutionMemoryRegister (
    //temp storage holds data between exe and mem stages
    input clockSignal,
    input writeRegSignal,
    input writeToRegSignal,
    input writeMemorySignal,
    input readMemorySignal,
    input [31:0] ALUOutput,
    input [31:0] dataToWrite,
    input [4:0] regAddress,
    output reg writeRegOut,
    output reg writeToRegOut,
    output reg writeMemoryOut,
    output reg readMemoryOut,
    output reg [31:0] ALUOutputOut,
    output reg [31:0] dataToWriteOut,
    output reg [4:0] regAddressOut
);
always @(posedge clockSignal) begin
    dataToWriteOut <= dataToWrite;//store data to write
    writeToRegOut <= writeToRegSignal;//store write to register signals
    regAddressOut <= regAddress;  //store register addresses  
    writeRegOut <= writeRegSignal;
    writeMemoryOut <= writeMemorySignal;
    readMemoryOut <= readMemorySignal;
    ALUOutputOut <= ALUOutput;
end
endmodule

module ForwardingControlUnit (
    input EX_MemRegWrite,
    input [4:0] EX_MemWriteReg,
    input Mem_WbRegWrite,
    input [4:0] Mem_WbWriteReg,
    input [4:0] ID_Ex_Rs,
    input [4:0] ID_Ex_Rt,
    output reg [1:0] upperMuxSelect,
    output reg [1:0] lowerMuxSelect,
    output reg [1:0] comparatorMux1Select,
    output reg [1:0] comparatorMux2Select
);
always @(EX_MemRegWrite or EX_MemWriteReg or Mem_WbRegWrite or Mem_WbWriteReg or ID_Ex_Rs or ID_Ex_Rt) begin
    if(EX_MemRegWrite && EX_MemWriteReg) begin
        if (EX_MemWriteReg == ID_Ex_Rs) begin
            upperMuxSelect <= 2'b10;//forwarding to upper mux
            comparatorMux1Select <= 2'b01;//forwarding for comparator
        end else begin
            upperMuxSelect <= 2'b00;//no forwarding
            comparatorMux1Select <= 2'b00;//no forwarding
        end  
        if(EX_MemWriteReg == ID_Ex_Rt) begin
            lowerMuxSelect <= 2'b10;
            comparatorMux2Select <= 2'b01;
        end else begin
            lowerMuxSelect <= 2'b00;
            comparatorMux2Select <= 2'b00;
        end
    end else if (Mem_WbRegWrite && Mem_WbWriteReg) begin
        if ((Mem_WbWriteReg == ID_Ex_Rs) && (EX_MemWriteReg != ID_Ex_Rs)) begin
            upperMuxSelect <= 2'b01;
            comparatorMux1Select <= 2'b10;
        end else begin
            upperMuxSelect <= 2'b00;
            comparatorMux1Select <= 2'b00;
        end 
        if((Mem_WbWriteReg == ID_Ex_Rt) && (EX_MemWriteReg == ID_Ex_Rt)) begin
            lowerMuxSelect <= 2'b01;
            comparatorMux2Select <= 2'b10;
        end else begin
            lowerMuxSelect <= 2'b00;
            comparatorMux2Select <= 2'b00;
        end
    end else begin
        upperMuxSelect <= 2'b00;//no forwarding
        lowerMuxSelect <= 2'b00;
        comparatorMux1Select <= 2'b00;
        comparatorMux2Select <= 2'b00;
    end
end
endmodule

module HazardControlUnit(
    input ID_ExMemRead,
    input EX_MemMemRead,
    input [4:0] ID_Ex_Rt,
    input [31:0] IF_ID_Instr,
    output reg stallPC,
    output reg stallIF_ID,
    output reg muxSelect
);
initial begin
    stallPC <= 0;
    stallIF_ID <= 0;
    muxSelect <= 0;
end

always @(ID_ExMemRead or ID_Ex_Rt or IF_ID_Instr) begin
    if (ID_ExMemRead && (stallPC == 1'b0) && (stallIF_ID == 1'b0)) begin
        if(ID_Ex_Rt == IF_ID_Instr[25:21] || ID_Ex_Rt == IF_ID_Instr[20:15]) begin
            stallPC <= 1;//stall because hazard detected
            stallIF_ID <= 1;
            muxSelect <= 1;
        end
    end else if ((IF_ID_Instr[31:26] == 6'b000100) && (stallPC == 1'b0) && (stallIF_ID == 1'b0)) begin
        stallPC <= 1;//stall because branch instruction
        stallIF_ID <= 1;
        muxSelect <= 1;
    end else begin
        stallPC <= 0;
        stallIF_ID <= 0;
        muxSelect <= 0;     
    end    
end
endmodule

module InstructionDecode_Execute_Register (
    input wire clockSignal, 
    input wire writeRegSignal,
    input wire writeToRegSignal,
    input wire writeMemorySignal,
    input wire readMemorySignal,
    input wire ALUSrcSignal,
    input wire [3:0] ALUOpSignal,
    input wire destRegSignal,
    input wire [31:0] PCPlus4Signal,
    input wire [31:0] data1InputSignal,
    input wire [31:0] data2InputSignal,
    input wire [31:0] signExtendInputSignal,
    input wire [14:0] registerAddressInputSignal,
    output reg [31:0] PCPlus4OutSignal,
    output reg [31:0] data1OutputSignal,
    output reg [31:0] data2OutputSignal,
    output reg [31:0] signExtendOutputSignal,
    output reg [4:0] rsOutSignal,
    output reg [4:0] rtOutSignal,
    output reg [4:0] rdOutSignal,
    output reg writeRegOutputSignal,
    output reg writeToRegOutputSignal,
    output reg writeMemoryOutputSignal,
    output reg readMemoryOutputSignal,
    output reg ALUSrcOutputSignal,
    output reg [3:0] ALUOpOutputSignal,
    output reg destRegOutputSignal
);
always @(posedge clockSignal) begin
    PCPlus4OutSignal <= PCPlus4Signal;//store pc+4
    data1OutputSignal <= data1InputSignal;//store data 1
    data2OutputSignal <= data2InputSignal;//store data2
    signExtendOutputSignal <= signExtendInputSignal;
    rsOutSignal <= registerAddressInputSignal[14:10];//store source register
    rtOutSignal <= registerAddressInputSignal[9:5];//store target register
    rdOutSignal <= registerAddressInputSignal[4:0];//store destination register
    writeRegOutputSignal <= writeRegSignal;
    writeToRegOutputSignal <= writeToRegSignal;
    writeMemoryOutputSignal <= writeMemorySignal;
    readMemoryOutputSignal <= readMemorySignal;
    ALUSrcOutputSignal <= ALUSrcSignal;
    ALUOpOutputSignal <= ALUOpSignal;
    destRegOutputSignal <= destRegSignal;
end
endmodule

module InstructionFetch_Decode_Register (
    input clockSignal,
    input wire [31:0] PCPlus4Signal,
    input wire [31:0] instructionInputSignal,
    output reg [31:0] instructionOutputSignal,
    input holdSignal,
    output reg [31:0] PCPlus4OutputSignal,
    input flushSignal
);
always @(posedge clockSignal) begin
    if (holdSignal == 1'b0) begin   
        PCPlus4OutputSignal <= PCPlus4Signal;
        instructionOutputSignal <= instructionInputSignal;   
    end else if (flushSignal == 1'b1) begin
        PCPlus4OutputSignal <= PCPlus4Signal; 
        instructionOutputSignal <= 32'b0;//clear output
    end
end
endmodule

module InstructionMemoryUnit(
    input clockSignal,
    input [31:0] programCounter,
    output reg [31:0] instructionData
);
reg [31:0] instructionMemoryArray [0:1060];
initial begin
    instructionMemoryArray[0] = 32'h8C000000; // lw 
    instructionMemoryArray[4] = 32'h8C010001; // lw 
    instructionMemoryArray[8] = 32'h03FEF020; // i++
    instructionMemoryArray[12] = 32'h00011020; // add 2
    instructionMemoryArray[16] = 32'h00221820; // add 3 
    instructionMemoryArray[20] = 32'h03FEF020; // i++
    instructionMemoryArray[24] = 32'h00622020; // add 4
    instructionMemoryArray[28] = 32'h03FEF020; // i++
    instructionMemoryArray[32] = 32'h00832820; // add 5
    instructionMemoryArray[36] = 32'h03FEF020; // i++
    instructionMemoryArray[40] = 32'h28AA0006; // lsti
    instructionMemoryArray[44] = 32'h13C50002; // beq
    instructionMemoryArray[48] = 32'hAC210000; // sw
end

always @(programCounter) begin	
    instructionData <= instructionMemoryArray[programCounter];//read instruction from instruction memory
end				
endmodule	

module InstructionMemoryUnit2(
    //provide larger instruction memory space
    input clockSignal,
    input [31:0] programCounter,
    output reg [31:0] instructionData
);
reg [31:0] instructionMemoryArray [0:1023];

initial begin
    instructionMemoryArray[0] = 32'h01095020;
    instructionMemoryArray[1] = 32'hAC0A0000;
    instructionMemoryArray[2] = 32'h01495822;
    instructionMemoryArray[3] = 32'h1168FFFC;  
    instructionMemoryArray[4] = 32'hAC0A0000;
end

always @(programCounter) begin	 
    instructionData <= instructionMemoryArray[programCounter >> 2];
end			
		
endmodule	

module WriteBackRegister (
    input clockSignal,
    input writeRegSignal, 
    input writeToRegSignal,
    input [31:0] ALUOutput,
    input [31:0] readData,
    input [4:0] regAddress,
    output reg writeRegOut,
    output reg writeToRegOut,
    output reg [31:0] readDataOut,
    output reg [31:0] ALUOutputOut,
    output reg [4:0] regAddressOut
);
always @(posedge clockSignal) begin
    writeRegOut <= writeRegSignal;
    writeToRegOut <= writeToRegSignal;
    readDataOut <= readData;
    ALUOutputOut <= ALUOutput;
    regAddressOut <= regAddress;
end  
endmodule

module MemoryRegisterTransfer (
    //transfer data between memory and register
    input writeMemorySignal,
    input writeMemory2Signal,
    input readMemorySignal,
    input readMemory2Signal,
    input [31:0] ALUResult,
    input [31:0] address2,
    input [15:0] memoryWriteData,
    input [15:0] memoryWriteData2,
    input clockSignal,
    output reg [15:0] readData1, // Changed to reg
    output reg [15:0] readDataMemory, // Changed to reg
    output reg writeRegSignal, // Changed to reg
    output reg writeRegSignal2, // Changed to reg
    output reg [4:0] writeReg, // Changed to reg
    output reg [4:0] writeReg2 // Changed to reg
);
always @(posedge clockSignal) begin
    writeRegSignal2 <= writeRegSignal; 
    writeReg2 <= writeReg;
    readDataMemory <= readData1; // Ensure this is the correct assignment
end
endmodule

module Multiplexer2x1_5Bits(
    output reg [4:0] result,
    input [4:0] inputA,
    input [4:0] inputB,
    input selectSignal
);
always @(inputA, inputB, selectSignal) begin
    case(selectSignal)
        1'b0: result = inputA;
        1'b1: result = inputB;
        default: result = 5'bx;
    endcase
end
endmodule

module Multiplexer2x1_10Bits(
    output reg [9:0] result,
    input [9:0] inputA,
    input [9:0] inputB,
    input selectSignal
);
always @(inputA, inputB, selectSignal) begin
    case(selectSignal)
        1'b0: result = inputA; 
        1'b1: result = inputB;
        default: result = 10'bx;
    endcase
end
endmodule

module Multiplexer2x1_32Bits(
    output reg [31:0] result,
    input [31:0] inputA,
    input [31:0] inputB,
    input selectSignal
);
always @(inputA, inputB, selectSignal) begin
    case(selectSignal)
        1'b0: result = inputA; 
        1'b1: result = inputB; 
        default: result = 32'bx;
    endcase
end
endmodule

module Multiplexer3x1_32Bits(
    output reg [31:0] result,
    input [31:0] inputA, 
    input [31:0] inputB, 
    input [31:0] inputC,
    input [1:0] selectSignal
);
always @(inputA, inputB, inputC, selectSignal) begin
    case(selectSignal)
        2'b00: result = inputA;
        2'b01: result = inputB;
        2'b10: result = inputC;
        default: result = 32'bx;
    endcase
end
endmodule

module ProgramCounter (
    input clockSignal, 
    input wire [31:0] nextPCSignal,
    output reg [31:0] currentPC,
    input resetSignal,
    input holdSignal
);
always @(posedge resetSignal) begin
    currentPC <= 32'hFFFFFFFC;//reset pc to specific value
end

always @(posedge clockSignal) begin
    if (holdSignal == 0) begin
        currentPC <= nextPCSignal;//update pc
    end
end
endmodule

module RegisterFileUnit(
//a drawer included many reg and access to reg using control signals 
    input clockSignal,
    input [4:0] readReg1, 
    input [4:0] readReg2, 
    input [4:0] writeReg, 
    input [31:0] writeData,
    input writeEnable, 
    output reg [31:0] readData1, 
    output reg [31:0] readData2, 
    input resetSignal
);
reg [31:0] registers[0:31];

always @(posedge resetSignal) begin
    registers[0] <= 32'h00000000;
    registers[1] <= 32'h00000000;
    registers[30] <= 32'h00000001; // saving constant 1
    registers[31] <= 32'h00000001; // saving constant 1
end

always @(readReg1, readReg2) begin
    readData1 <= registers[readReg1];
    readData2 <= registers[readReg2];
end

always @(negedge clockSignal) begin
    if (writeEnable) begin
        registers[writeReg] <= writeData;
    end
end
endmodule

module ShiftLeftBy2(
    output [31:0] shiftedResult,
    input [31:0] inputData
);
assign shiftedResult = inputData << 2;
endmodule

module SignExtender (
    input [15:0] inputData,
    output reg [31:0] extendedResult
);
always @(inputData) begin
    if (inputData[15] == 1) begin     
        extendedResult = {16'hFFFF, inputData};
    end else begin
        extendedResult = {16'h0000, inputData};
    end
end
endmodule