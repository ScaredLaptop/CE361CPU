`define OPCODE_COMPUTE    7'b0110011
`define OPCODE_COMPUTE_IMM 7'b0010011
`define OPCODE_BRANCH     7'b1100011
`define OPCODE_LOAD       7'b0000011
`define OPCODE_JMP        7'b1101111
`define OPCODE_JMP_LINK   7'b1100111
`define OPCODE_STORE      7'b0100011 
`define OPCODE_LUI       7'b0110111
`define OPCODE_AUIPC     7'b0010111
`define FUNC_ADD      3'b000
`define AUX_FUNC_ADD  7'b0000000
`define AUX_FUNC_SUB  7'b0100000
`define SIZE_BYTE  2'b00
`define SIZE_HWORD 2'b01
`define SIZE_WORD  2'b10
`define PCSel_4 1'b0
`define PCSel_ALU 1'b1
`define ImmSel_I 3'b000
`define ImmSel_B 3'b001
`define ImmSel_J 3'b010
`define ImmSel_U 3'b011
`define ImmSel_S 3'b100
`define ASel_Reg 1'b0
`define ASel_PC 1'b1
`define BSel_Reg 1'b0
`define BSel_IMM 1'b1
`define MemRW_Write 1'b0
`define MemRW_Read 1'b1
`define RWrEn_Enable 1'b0
`define RWrEn_Disable 1'b1
`define WBSel_ALU 2'b00
`define WBSel_PC4 2'b01
`define WBSel_Mem 2'b10
`define WBSel_Imm 2'b11
`define BEQ 3'b000
`define BNE 3'b001
`define BLT 3'b100
`define BGE 3'b101
`define BLTU 3'b110
`define BGEU 3'b111
// Template for Northwestern - CompEng 361 - Lab2
// Groupname: SingleRussleCPU
// NetIDs: ljp0624, swp8132

module SingleCycleCPU(halt, clk, rst);
   output halt;
   input clk, rst;

   wire [31:0] PC, InstWord;
   wire [31:0] DataAddr, StoreData, DataWord;
   wire [1:0]  MemSize;
   wire        MemWrEn;
   
   wire [4:0]  Rsrc1, Rsrc2, Rdst;
   wire [31:0] Rdata1, Rdata2;

   wire BrEq, BrLT, PCSel, ASel, BSel, MemRW, RWrEn;
   wire [2:0] ImmSel;
   wire [1:0] WBSel;
   wire [31:0] NPC;
   wire [6:0]  opcode;
   wire [6:0]  funct7;
   wire [2:0]  funct3;

   // ALU Inputs
   wire [31:0] ALUOutput;
   wire [31:0] ALU_A;
   wire [31:0] ALU_B;

   wire [31:0] RWrData;

   wire [31:0] LoadExtended;

   SizeModule SM(.funct3(funct3),
                .DataWord(DataWord),
                .MemSize(MemSize),
                .LoadExtended(LoadExtended)
                );
   // System State (everything is neg assert)
   InstMem IMEM(.Addr(PC), .Size(`SIZE_WORD), .DataOut(InstWord), .CLK(clk));
   DataMem DMEM(.Addr(ALUOutput), .Size(MemSize), .DataIn(Rdata2), .DataOut(DataWord), .WEN(MemRW), .CLK(clk));

   RegFile RF(.AddrA(Rsrc1), .DataOutA(Rdata1), 
          .AddrB(Rsrc2), .DataOutB(Rdata2), 
          .AddrW(Rdst), .DataInW(RWrData), .WenW(RWrEn), .CLK(clk));

   Reg PC_REG(.Din(NPC), .Qout(PC), .WEN(1'b0), .CLK(clk), .RST(rst));

   // Instruction Decode
   assign opcode = InstWord[6:0];   
   assign Rdst = InstWord[11:7]; 
   assign Rsrc1 = InstWord[19:15]; 
   assign Rsrc2 = InstWord[24:20];
   assign funct3 = InstWord[14:12];  // R-Type, I-Type, S-Type
   assign funct7 = InstWord[31:25];  // R-Type

   assign ALU_A = (ASel == 1'b0) ? Rdata1 : PC; 
   assign ALU_B = (BSel == 1'b0) ? Rdata2 : Imm;
   ExecutionUnit EU(.out(ALUOutput), .opA(ALU_A), .opB(ALU_B), .func(funct3), .auxFunc(funct7), .opcode(opcode));


   // Write Back signal generation
   assign RWrData = (WBSel == `WBSel_ALU) ? ALUOutput : (WBSel == `WBSel_PC4) ? PC + 4 : (WBSel == `WBSel_Mem) ? LoadExtended : Imm;

   // Immediate generation
   wire [31:0] Imm;
   ImmGen IG(.InstWord(InstWord), .ImmSel(ImmSel), .Imm(Imm));   

   // Branch Comparison TODO: support unsigned
   wire BR;
   BranchComparison BC(
    .Rdata1(Rdata1),
    .Rdata2(Rdata2),
    .funct3(funct3),
    .BR(BR)
    );
   // Fetch Address Datapath
   PCUpdate PCU(.PCSel(PCSel), .PC(PC), .PC_Jump(ALUOutput), .NPC(NPC)); 
   
   // Opcode decoder
   OpDecoder OD(.op(opcode),
    .funct3(funct3),
    .funct7(funct7),
    .BR(BR),
    .PCSel(PCSel),
    .ImmSel(ImmSel), 
    .ASel(ASel), 
    .BSel(BSel), 
    .MemRW(MemRW), 
    .RWrEn(RWrEn), 
    .WBSel(WBSel),
    .halt(halt));
endmodule // SingleCycleCPU

module SizeModule(input [2:0] funct3,
                input [31:0] DataWord,
                output reg [1:0] MemSize,
                output reg [31:0] LoadExtended
                );
always @(*) begin
    case (funct3)
        000: MemSize = `SIZE_BYTE;
        001: MemSize = `SIZE_HWORD;
        010: MemSize = `SIZE_WORD;
        default: MemSize = 2'bxx;
    endcase
   end

always @(*) begin
    case (funct3)
        000: LoadExtended = {{24{DataWord[7]}}, DataWord[7:0]};
        001: LoadExtended = {{16{DataWord[15]}}, {DataWord[15:0]} };
        010: LoadExtended = DataWord;
        100: LoadExtended = {{24{1'b0}}, {DataWord[7:0]} };
        101: LoadExtended = {{16{1'b0}}, {DataWord[15:0]} };
    endcase
   end
endmodule

module PCUpdate(
    input PCSel,
    input [31:0] PC,
    input [31:0] PC_Jump,
    output reg [31:0] NPC
);
    always @(*) begin
        case(PCSel)
            `PCSel_4: NPC <= PC + 4;
            `PCSel_ALU: NPC <= PC_Jump;
        endcase
    end
endmodule
module ImmGen(
   input [31:0] InstWord,
   input [2:0]  ImmSel, 
   output reg [31:0] Imm);
   always @(*) 
   case(ImmSel)
        `ImmSel_I: Imm = { {21{InstWord[31]}}, InstWord[30:25], InstWord[24:21], InstWord[20]};
        `ImmSel_S: Imm = { {21{InstWord[31]}}, InstWord[30:25], InstWord[11:8], InstWord[7]};
        `ImmSel_U: Imm = { InstWord[31], InstWord[30:20], InstWord[19:12], {12{1'b0}} };
        `ImmSel_I: Imm = { {21{InstWord[31]}}, InstWord[30:25], InstWord[24:21], InstWord[20]};
        `ImmSel_J: Imm = { {12{InstWord[31]}}, InstWord[19:12], InstWord[20], InstWord[30:25], InstWord[24:21], {1{1'b0}}};
        `ImmSel_B: Imm = { {20{InstWord[31]}}, InstWord[7], InstWord[30:25], InstWord[11:8], {1{1'b0}}};
        default: Imm = 32'bx;
    endcase
endmodule
// Module which determines whether a branch should be taken
module BranchComparison(
    input [31:0] Rdata1,
    input [31:0] Rdata2,
    input [2:0] funct3,
    output reg BR
);

always @(*) begin
    case(funct3)
        `BEQ: BR = (Rdata1 == Rdata2)? 1 : 0;
        `BNE: BR = (Rdata1 != Rdata2)? 1 : 0;
        `BLT: BR = (Rdata1 < Rdata2)? 1 : 0;
        `BGE: BR = (Rdata1 >= Rdata2)? 1 : 0;
        `BLTU: BR = (Rdata1 < Rdata2)? 1 : 0;
        `BGEU: BR = (Rdata1 >= Rdata2)? 1 : 0;
        default: BR = 1'bx;
    endcase
end
endmodule

module OpDecoder(
   input [6:0] op,
   input [2:0] funct3,
   input [6:0] funct7,
   input BR,
   output reg PCSel,
   output reg [2:0] ImmSel, 
   output reg ASel, 
   output reg BSel, 
   output reg MemRW, 
   output reg RWrEn, 
   output reg [1:0] WBSel,
   output reg halt
);
   always @(*) begin
      case (op)
            `OPCODE_COMPUTE: // R-Type
                begin
                PCSel <= `PCSel_4;
                ImmSel <= 3'bx;
                ASel <= `ASel_Reg;
                BSel <= `BSel_Reg;
                MemRW <= `MemRW_Read;
                RWrEn <= `RWrEn_Enable;
                WBSel <= `WBSel_ALU;
                end
            `OPCODE_COMPUTE_IMM: // I-Type
                begin
                PCSel <= `PCSel_4;
                ImmSel <= `ImmSel_I;
                ASel <= `ASel_Reg;
                BSel <= `BSel_IMM;
                MemRW <= `MemRW_Read;
                RWrEn <= `RWrEn_Enable;
                WBSel <= `WBSel_ALU;
                end
            `OPCODE_BRANCH: 
               begin
                PCSel <= (BR) ? `PCSel_ALU : `PCSel_4;
                ImmSel <= `ImmSel_B;
                ASel <= `ASel_PC;
                BSel <= `BSel_IMM;
                MemRW <= `MemRW_Read;
                RWrEn <= `RWrEn_Disable;
                WBSel <= 2'bxx;
               end
            `OPCODE_LOAD:
               begin
               PCSel <= `PCSel_4;
               ImmSel <= `ImmSel_I;
               ASel <= `ASel_Reg;
               BSel <= `BSel_IMM;
               MemRW <= `MemRW_Read;
               RWrEn <= `RWrEn_Enable;
               WBSel <= `WBSel_Mem;
               end
            `OPCODE_STORE:
               begin
               PCSel <= `PCSel_4;
               ImmSel <= `ImmSel_S;
               ASel <= `ASel_Reg;
               BSel <= `BSel_IMM;
               MemRW <= `MemRW_Write;
               RWrEn <= `RWrEn_Disable;
               WBSel <= 2'bxx;
               end
            `OPCODE_JMP:
                begin
                    PCSel <= `PCSel_ALU;
                    ImmSel <= `ImmSel_J;
                    ASel <= `ASel_PC;
                    BSel <= `BSel_IMM;
                    MemRW <= `MemRW_Read;
                    RWrEn <= `RWrEn_Enable;
                    WBSel <= `WBSel_PC4;
                end
            `OPCODE_JMP_LINK:
                begin
                    PCSel <= `PCSel_ALU;
                    ImmSel <= `ImmSel_I;
                    ASel <= `ASel_Reg;
                    BSel <= `BSel_IMM;
                    MemRW <= `MemRW_Read;
                    RWrEn <= `RWrEn_Enable;
                    WBSel <= `WBSel_PC4;
                end
            `OPCODE_AUIPC:
                begin
                    PCSel <= `PCSel_4;
                    ImmSel <= `ImmSel_U;
                    ASel <= `ASel_PC;
                    BSel <= `BSel_IMM;
                    MemRW <= `MemRW_Read;
                    RWrEn <= `RWrEn_Enable;
                    WBSel <= `WBSel_ALU;
                end
            `OPCODE_LUI:
                begin
                    PCSel <= `PCSel_4;
                    ImmSel <= `ImmSel_U;
                    ASel <= `ASel_Reg;
                    BSel <= `BSel_IMM;
                    MemRW <= `MemRW_Read;
                    RWrEn <= `RWrEn_Enable;
                    WBSel <= `WBSel_Imm;
                end
            default:
            begin
               halt = 1'b1;
            end
        endcase
   end
endmodule

// ExecutionUnit
module ExecutionUnit(out, opA, opB, func, auxFunc, opcode);
output reg [31:0] out;
input [31:0] opA, opB;
input [2:0] func;
input [6:0] auxFunc;
input [6:0] opcode;
// Place your code here
wire signed [31:0] s_opA, s_opB;
assign s_opA = opA;
assign s_opB = opB;
always @(*) begin
    if(opcode == `OPCODE_COMPUTE || opcode == `OPCODE_COMPUTE_IMM) begin
        case (func)
            3'b000: 
                if(auxFunc == 7'b0000000 || opcode == `OPCODE_COMPUTE_IMM)
                    out = opA + opB;
                else
                    out = opA - opB;
            3'b001: out = opA << opB;
            3'b010: out = (s_opA < s_opB)? 1 : 0;
            3'b011: out = (opA < opB)? 1 : 0;
            3'b100: out = opA ^ opB;
            3'b101: 
                if(auxFunc == 7'b0000000)
                    out = opA >> opB;
                else
                    out = s_opA >>> opB;
            3'b110: out = opA | opB;
            3'b111: out = opA & opB;
            default: out = 32'b0;
        endcase
    end
    else
        out = opA + opB;
end
endmodule // ExecutionUnit

