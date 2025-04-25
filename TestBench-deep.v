`timescale 1ns / 1ps
`include "wholeproject.v"
module Testbench;

    // Signals for Summation
    reg signed [31:0] operand1;
    reg signed [31:0] operand2;
    wire [31:0] outputSum;

    // Signals for ArithmeticLogicUnit
    reg signed [31:0] input1;
    reg signed [31:0] input2;
    reg [3:0] operationCode;
    reg [4:0] shiftBits;
    reg resetSignal;
    wire carryOut;
    wire isZero;
    wire signed [31:0] outputResult;

    // Signals for ControlSignalGenerator
    reg [5:0] opcode;
    wire destReg;
    wire isBranch;
    wire readMemory;
    wire writeToReg;
    wire [3:0] ALUOperation;
    wire writeMemorySignal;
    wire sourceSelect;
    wire writeRegSignal;

    // Signals for MemoryUnit
    reg clockSignal;
    reg writeMemorySignal_tb; // Changed to reg
    reg readMemory_tb; // Changed to reg
    reg [31:0] address;
    reg [31:0] dataToWrite;
    wire [31:0] dataRead;

    // Signals for RegisterFileUnit
    reg [4:0] readReg1;
    reg [4:0] readReg2;
    reg [4:0] writeReg;
    reg [31:0] writeData;
    reg writeEnable;
    wire [31:0] readData1;
    wire [31:0] readData2;

    // Instantiate the modules
    Summation summation_inst (
        .operand1(operand1),
        .operand2(operand2),
        .outputSum(outputSum)
    );

    ArithmeticLogicUnit alu_inst (
        .input1(input1),
        .input2(input2),
        .operationCode(operationCode),
        .shiftBits(shiftBits),
        .resetSignal(resetSignal),
        .carryOut(carryOut),
        .isZero(isZero),
        .outputResult(outputResult)
    );

    ControlSignalGenerator control_gen_inst (
        .opcode(opcode),
        .destReg(destReg),
        .isBranch(isBranch),
        .readMemory(readMemory),
        .writeToReg(writeToReg),
        .ALUOperation(ALUOperation),
        .writeMemorySignal(writeMemorySignal),
        .sourceSelect(sourceSelect),
        .writeRegSignal(writeRegSignal),
        .resetSignal(resetSignal)
    );

    MemoryUnit mem_unit_inst (
        .clockSignal(clockSignal),
        .writeMemorySignal(writeMemorySignal_tb), // Use the reg
        .readMemorySignal(readMemory_tb), // Use the reg
        .address(address),
        .dataToWrite(dataToWrite),
        .dataRead(dataRead)
    );

    RegisterFileUnit reg_file_inst (
        .clockSignal(clockSignal),
        .readReg1(readReg1),
        .readReg2(readReg2),
        .writeReg(writeReg),
        .writeData(writeData),
        .writeEnable(writeEnable),
        .readData1(readData1),
        .readData2(readData2),
        .resetSignal(resetSignal)
    );

    // Clock generation
    initial begin
        clockSignal = 0;
        forever #5 clockSignal = ~clockSignal; // 10ns clock period
    end

    // Test procedure
    initial begin
        // Initialize signals
        resetSignal = 1;
        #10; // Wait for a clock cycle
        resetSignal = 0;

        // Initialize waveform dumping
        $dumpfile("waveform.vcd"); // Specify the name of the VCD file
        $dumpvars(0, Testbench); // Dump all variables in the Testbench module

        // Read the first two Fibonacci numbers from memory
        readMemory_tb = 1; // Enable memory read
        address = 0; // Address of Fibonacci(0)
        #10;
        // Expect 0

        address = 1; // Address of Fibonacci(1)
        #10;
        // Expect 1

        // Calculate and store the next 10 Fibonacci numbers
        for (integer i = 2; i < 12; i = i + 1) begin
            // Read Fibonacci(i-2)
            address = i - 2;
            #10;
            operand1 = dataRead;

            // Read Fibonacci(i-1)
            address = i - 1;
            #10;
            operand2 = dataRead;

            #10;

            writeMemorySignal_tb = 1; // Enable memory write
            address = i;
            dataToWrite = outputSum;
            #10;
            writeMemorySignal_tb = 0; // Disable memory write
        end

        for (integer i = 0; i < 12; i = i + 1) begin
            address = i;
            readMemory_tb = 1;
            #10;
            $display("Fibonacci(%0d): %d", i, dataRead);
        end

        $finish;
    end
endmodule