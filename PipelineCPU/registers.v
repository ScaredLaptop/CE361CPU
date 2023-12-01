module IF_ID_Register(
    input [31:0] PC_if,
    input [31:0] Inst_if,
    output reg [31:0] PC_id,
    output reg [31:0] Inst_id,
    input WEN, 
    input CLK, 
    input RST);
    always @ (negedge CLK or negedge RST)
        if (!RST) begin
            PC_id <= 0;
            Inst_id <= 0;
        end else if (!WEN) begin
            PC_id <= PC_if;
            Inst_id <= Inst_if;
        end
endmodule // IF_ID_Register

module ID_EX_Register(
    input [31:0] PC_id,
    input [31:0] Inst_id,
    input MemRW_id, 
    input RWrEn_id,
    input MemToReg_id,
    input [1:0] ALUOp_id,
    input [1:0] ALUSrc_id,
    input [1:0] RegDst_id,
    input [2:0] ImmSel_id,
    input ASel_id,
    input BSel_id,
    input [2:0] BranchType_id,
    input JMP_id,
    input BR_id,
    input halt_id,
    output reg [31:0] PC_ex,
    output reg [31:0] Inst_ex,
    output reg MemRW_ex,
    output reg RWrEn_ex,
    output reg MemToReg_ex,
    output reg [1:0] ALUOp_ex,
    output reg [1:0] ALUSrc_ex,
    output reg [1:0] RegDst_ex,
    output reg [2:0] ImmSel_ex,
    output reg ASel_ex,
    output reg BSel_ex,
    output reg [2:0] BranchType_ex,
    output reg JMP_ex,
    output reg BR_ex,
    output reg halt_ex,
    input WEN, 
    input CLK, 
    input RST);
    always @ (negedge CLK or negedge RST)
        if (!RST) begin
            PC_ex <= 0;
            Inst_ex <= 0;
            MemRW_ex <= 0;
            RWrEn_ex <= 0;
            MemToReg_ex <= 0;
            ALUOp_ex <= 0;
            ALUSrc_ex <= 0;
            RegDst_ex <= 0;
            ImmSel_ex <= 0;
            ASel_ex <= 0;
            BSel_ex <= 0;
            BranchType_ex <= 0;
            JMP_ex <= 0;
            BR_ex <= 0;
            halt_ex <= 0;
        end else if (!WEN) begin
            PC_ex <= PC_id;
            Inst_ex <= Inst_id;
            MemRW_ex <= MemRW_id;
            RWrEn_ex <= RWrEn_id;
            MemToReg_ex <= MemToReg_id;
            ALUOp_ex <= ALUOp_id;
            ALUSrc_ex <= ALUSrc_id;
            RegDst_ex <= RegDst_id;
            ImmSel_ex <= ImmSel_id;
            ASel_ex <= ASel_id;
            BSel_ex <= BSel_id;
            BranchType_ex <= BranchType_id;
            JMP_ex <= JMP_id;
            BR_ex <= BR_id;
            halt_ex <= halt_id;
        end
endmodule // ID_EX_Register

module EX_MEM_Register(
    input [31:0] PC_ex,
    input [31:0] Inst_ex,
    input MemRW_ex,
    input RWrEn_ex,
    input MemToReg_ex,
    input JMP_ex,
    input BR_ex,
    input BranchCondTrue_ex,  
    input halt_ex,
    output reg [31:0] PC_mem,
    output reg [31:0] Inst_mem,
    output reg MemRW_mem,
    output reg RWrEn_mem,
    output reg MemToReg_mem,
    output reg JMP_mem,
    output reg BR_mem,
    output reg BranchCondTrue_mem,
    output reg halt_mem,
    input WEN, 
    input CLK, 
    input RST);
    always @ (negedge CLK or negedge RST)
        if (!RST) begin
            PC_mem <= 0;
            Inst_mem <= 0;
            MemRW_mem <= 0;
            RWrEn_mem <= 0;
            MemToReg_mem <= 0;
            JMP_mem <= 0;
            BR_mem <= 0;
            BranchCondTrue_mem <= 0;
            halt_mem <= 0;
        end else if (!WEN) begin
            PC_mem <= PC_ex;
            Inst_mem <= Inst_ex;
            MemRW_mem <= MemRW_ex;
            RWrEn_mem <= RWrEn_ex;
            MemToReg_mem <= MemToReg_ex;
            JMP_mem <= JMP_ex;
            BR_mem <= BR_ex;
            BranchCondTrue_mem <= BranchCondTrue_ex;
            halt_mem <= halt_ex;
        end
endmodule // EX_MEM_Register

module MEM_WB_Register(
    input [31:0] PC_mem,
    input [31:0] Inst_mem,
    input RegWrite_mem,
    input MemToReg_mem,
    input halt_mem,
    output reg [31:0] PC_wb,
    output reg [31:0] Inst_wb,
    output reg RegWrite_wb,
    output reg MemToReg_wb,
    output reg halt_wb,
    input WEN, 
    input CLK, 
    input RST);
    always @ (negedge CLK or negedge RST)
        if (!RST) begin
            PC_wb <= 0;
            Inst_wb <= 0;
            RegWrite_wb <= 0;
            MemToReg_wb <= 0;
            halt_wb <= 0;
        end else if (!WEN) begin
            PC_wb <= PC_mem;
            Inst_wb <= Inst_mem;
            RegWrite_wb <= RegWrite_mem;
            MemToReg_wb <= MemToReg_mem;
            halt_wb <= halt_mem;
        end
endmodule // MEM_WB_Register