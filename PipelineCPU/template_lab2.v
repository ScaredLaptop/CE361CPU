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
// Datapath used from https://inst.eecs.berkeley.edu/~cs61c/su21/pdfs/lectures/fa20-trimmed/lec20_lec21.pdf
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
   wire [4:0] instr;
   // ALU Inputs
   wire [31:0] ALUOutput;
   wire [31:0] ALU_A;
   wire [31:0] ALU_B;

   wire [31:0] RWrData;

   wire [31:0] LoadExtended;

    wire invalidOpcode;
    wire unalignedPC;
    wire unalignedAccess;
    wire invalidSize;
   SizeModule SM(.funct3(funct3),
                .DataWord(DataWord),
                .MemSize(MemSize),
                .LoadExtended(LoadExtended),
                .halt(invalidSize)
                );
    assign invalidLoadStore = (opcode == `OPCODE_LOAD || opcode == `OPCODE_STORE)? invalidSize : 0;
   assign halt = invalidOpcode | unalignedPC | unalignedAccess | invalidBranch | invalidALUOp | invalidLoadStore;
   // System State (everything is neg assert)
   
   assign unalignedAccess = (opcode == `OPCODE_LOAD || opcode == `OPCODE_STORE)? 
                    (((MemSize == `SIZE_HWORD && ALUOutput[0] != 1'b0) || (MemSize == `SIZE_WORD && (ALUOutput[0] != 1'b0 || ALUOutput[0] != 1'b0)))?
                        1: 0) :0;
   assign invalidBranch = (opcode == `OPCODE_BRANCH & invalidBranchOp) ? 1 : 0;
   wire MemRW_Halt_Gated;
   assign MemRW_Halt_Gated = (halt | MemRW);
   DataMem DMEM(.Addr(ALUOutput), .Size(MemSize), .DataIn(Rdata2), .DataOut(DataWord), .WEN(MemRW_Halt_Gated), .CLK(clk));


   // Write Back signal generation
   assign RWrData = (WBSel == `WBSel_ALU) ? ALUOutput : (WBSel == `WBSel_PC4) ? PC + 4 : (WBSel == `WBSel_Mem) ? LoadExtended : Imm;


endmodule // SingleCycleCPU

module SizeModule(input [2:0] funct3,
                input [31:0] DataWord,
                output reg [1:0] MemSize,
                output reg [31:0] LoadExtended,
                output reg halt
                );

always @(*) begin
    halt <= 1'b0;
    case (funct3)
        3'b000: LoadExtended = {{24{DataWord[7]}}, DataWord[7:0]};
        3'b001: LoadExtended = {{16{DataWord[15]}}, {DataWord[15:0]} };
        3'b010: LoadExtended = DataWord;
        3'b100: LoadExtended = {{24{1'b0}}, {DataWord[7:0]} };
        3'b101: LoadExtended = {{16{1'b0}}, {DataWord[15:0]} };
        default: begin
            halt <= 1'b1;
            LoadExtended = {32{1'b0}};
        end

    endcase
   end
endmodule



