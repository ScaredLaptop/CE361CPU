module IFetch(
    input halt_in_if,
    input jump_in_if,
    input branch_in_if,
    input branch_taken_in_if,
    input [31:0] pc_br_jmp_target_in_if,
    output reg halt_out_if,
    output reg [31:0] pc_out_if,
    output reg [31:0] pc4_out_if,
    output reg [31:0] instr_out_if,
    input clk,
    input rst);

wire [31:0] NPC;

Reg PC_REG(.Din(NPC), .Qout(PC), .WEN(1'b0), .CLK(clk), .RST(rst));

assign pc_out_if = PC;
assign pc4_out_if = PC + 4;
InstMem IMEM(.Addr(PC), .Size(`SIZE_WORD), .DataOut(InstWord), .CLK(clk));

// Check PC Alignment
wire unalignedPC;
assign unalignedPC = (~(PC[0] == 1'b0)) | (~(PC[1] == 1'b0));
assign halt_out_if = halt_in_if | unalignedPC;

// Set next PC
always @(*) begin
        if(rst == 1'b1) begin
            PC = 32'h0;
        end
        else if(halt_out_if == 1'b1) begin
            NPC = PC;
        end
        else if(branch_taken_in_if == 1'b1 && branch_in_if == 1'b1) begin
            NPC = pc_br_jmp_target_in_if;
        end
        else if(jump_in_if == 1'b1) begin
            NPC = pc_br_jmp_target_in_if;
        end
        else begin
            NPC = PC + 4;
        end
end

endmodule // IFetch