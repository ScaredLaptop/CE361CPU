module IDecode(
    input halt_in_id,
    input [31:0] instr_in_id,
    input [31:0] pc_in_id,
    output reg halt_out_id,
    output reg [31:0] pc_out_id,
    output reg MemRW_out_id,
    output reg RWrEn_out_id,
    output reg MemToReg_out_id,
    output reg [1:0] ALUOp_out_id,
    output reg [1:0] ALUSrc_out_id,
    output reg [1:0] RegDst_out_id,
    output reg [2:0] ImmSel_out_id,
    output reg ASel_out_id,
    output reg BSel_out_id,
    output reg [2:0] BranchType_out_id,
    output reg JMP_out_id,
    output reg BR_out_id,
    output reg [2:0] MemSize_out_id,
    output reg [31:0] Immediate_out_id,
    output reg halt_out_id,
    input clk,
    input rst);

// Decode instruction
wire [6:0]  opcode;
wire [6:0]  funct7;
wire [2:0]  funct3;
wire [4:0]  Rsrc1, Rsrc2, Rdst;
assign opcode = instr_in_id[6:0];   
assign Rdst = instr_in_id[11:7]; 
assign Rsrc1 = instr_in_id[19:15]; 
assign Rsrc2 = instr_in_id[24:20];
assign funct3 = instr_in_id[14:12];
assign funct7 = instr_in_id[31:25];  

// Generate signals
wire invalidOpcode;
OpDecoder decoder(
    .op(opcode),
    .funct3(funct3),
    .funct7(funct7),
    .ImmSel(ImmSel_out_id),
    .ASel(ASel_out_id),
    .BSel(BSel_out_id),
    .MemRW(MemRW_out_id),
    .RWrEn(RWrEn_out_id),
    .WBSel(WBSel_out_id),
    .halt(invalidOpcode),
);

// Get size of instr
wire invalidOpSize;
SizeModule size(
    .funct3(funct3),
    .MemSize(MemSize_out_id)
    .halt(invalidOpSize),
);

// Generate immediate
ImmGen immGenerator(
    .InstWord(instr_in_id),
    .ImmSel(ImmSel_out_id),
    .Imm(Imm_out_id)
);

// Propogate and assign halt
assign halt_out_id = halt_in_id | invalidOpcode | invalidOpSize;
endmodule // IDecode

module OpDecoder(
   input [6:0] op,
   input [2:0] funct3,
   input [6:0] funct7,
   output reg [2:0] ImmSel, 
   output reg ASel, 
   output reg BSel, 
   output reg MemRW, 
   output reg RWrEn, 
   output reg [1:0] WBSel,
   output reg halt
);
   always @(*) begin
      halt <= 1'b0;
      case (op)
            `OPCODE_COMPUTE: // R-Type
                begin
                ImmSel <= 3'bx;
                ASel <= `ASel_Reg;
                BSel <= `BSel_Reg;
                MemRW <= `MemRW_Read;
                RWrEn <= `RWrEn_Enable;
                WBSel <= `WBSel_ALU;
                end
            `OPCODE_COMPUTE_IMM: // I-Type
                begin
                ImmSel <= `ImmSel_I;
                ASel <= `ASel_Reg;
                BSel <= `BSel_IMM;
                MemRW <= `MemRW_Read;
                RWrEn <= `RWrEn_Enable;
                WBSel <= `WBSel_ALU;
                end
            `OPCODE_BRANCH: 
               begin
                ImmSel <= `ImmSel_B;
                ASel <= `ASel_PC;
                BSel <= `BSel_IMM;
                MemRW <= `MemRW_Read;
                RWrEn <= `RWrEn_Disable;
                WBSel <= 2'bxx;
               end
            `OPCODE_LOAD:
               begin
               ImmSel <= `ImmSel_I;
               ASel <= `ASel_Reg;
               BSel <= `BSel_IMM;
               MemRW <= `MemRW_Read;
               RWrEn <= `RWrEn_Enable;
               WBSel <= `WBSel_Mem;
               end
            `OPCODE_STORE:
               begin
               ImmSel <= `ImmSel_S;
               ASel <= `ASel_Reg;
               BSel <= `BSel_IMM;
               MemRW <= `MemRW_Write;
               RWrEn <= `RWrEn_Disable;
               WBSel <= 2'bxx;
               end
            `OPCODE_JMP:
                begin
                    ImmSel <= `ImmSel_J;
                    ASel <= `ASel_PC;
                    BSel <= `BSel_IMM;
                    MemRW <= `MemRW_Read;
                    RWrEn <= `RWrEn_Enable;
                    WBSel <= `WBSel_PC4;
                end
            `OPCODE_JMP_LINK:
                begin
                    ImmSel <= `ImmSel_I;
                    ASel <= `ASel_Reg;
                    BSel <= `BSel_IMM;
                    MemRW <= `MemRW_Read;
                    RWrEn <= `RWrEn_Enable;
                    WBSel <= `WBSel_PC4;
                end
            `OPCODE_AUIPC:
                begin
                    ImmSel <= `ImmSel_U;
                    ASel <= `ASel_PC;
                    BSel <= `BSel_IMM;
                    MemRW <= `MemRW_Read;
                    RWrEn <= `RWrEn_Enable;
                    WBSel <= `WBSel_ALU;
                end
            `OPCODE_LUI:
                begin
                    ImmSel <= `ImmSel_U;
                    ASel <= `ASel_Reg;
                    BSel <= `BSel_IMM;
                    MemRW <= `MemRW_Read;
                    RWrEn <= `RWrEn_Enable;
                    WBSel <= `WBSel_Imm;
                end
            default:
            begin
               halt <= 1'b1;
            end
        endcase
   end
endmodule

module SizeModule(input [2:0] funct3,
                input [31:0] DataWord,
                output reg [1:0] MemSize,
                output reg halt
                );
always @(*) begin
    halt <= 1'b0;
    case (funct3)
        3'b000: MemSize = `SIZE_BYTE;
        3'b001: MemSize = `SIZE_HWORD;
        3'b010: MemSize = `SIZE_WORD;
        default: begin 
            halt <= 1'b1;
            MemSize = 3'bxxx;
        end
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
        `ImmSel_J: Imm = { {12{InstWord[31]}}, InstWord[19:12], InstWord[20], InstWord[30:25], InstWord[24:21], {1{1'b0}}};
        `ImmSel_B: Imm = { {20{InstWord[31]}}, InstWord[7], InstWord[30:25], InstWord[11:8], {1{1'b0}}};
        default: Imm = 32'bx;
    endcase
endmodule