/*
 * Copyright (c) 2015-2016 The Ultiparc Project. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * Instruction decode pipeline stage
 */

`include "cpu_common.vh"
`include "cpu_const.vh"


/* Decode stage */
module decode(
	clk,
	nrst,
	/* CU signals */
	i_pc,
	i_exec_stall,
	i_mem_stall,
	i_fetch_stall,
	i_drop,
	/* Coprocessor 0 */
	i_cop0_cop,
	i_cop0_reg_no,
	i_cop0_rt_no,
	/* Fetched instruction */
	i_instr,
	/* Decoded instr */
	o_rd_no,
	o_rs_no,
	o_rt_no,
	o_imm,
	o_alu_op,
	o_alu_inpt,
	o_alu_ovf_ex,
	o_jump,
	o_jump_link,
	o_imuldiv_op,
	o_lsu_op,
	o_lsu_lns,
	o_lsu_ext
);
`include "reg_names.vh"
`include "decode_const.vh"
localparam [`CPU_INSTR_WIDTH-1:0] NOP = 32'h0000_0000;
/* Inputs */
input wire				clk;
input wire				nrst;
/* CU signals */
input wire [`CPU_ADDR_WIDTH-1:0]	i_pc;
input wire				i_exec_stall;
input wire				i_mem_stall;
input wire				i_fetch_stall;
input wire				i_drop;
/* Coprocessor 0 */
input wire [`CPU_REGNO_WIDTH-1:0]	i_cop0_cop;
input wire [`CPU_REGNO_WIDTH-1:0]	i_cop0_reg_no;
input wire [`CPU_REGNO_WIDTH-1:0]	i_cop0_rt_no;
/* Fetched instruction */
input wire [`CPU_INSTR_WIDTH-1:0]	i_instr;
/* Decoded instr */
output reg [`CPU_REGNO_WIDTH-1:0]	o_rd_no;
output reg [`CPU_REGNO_WIDTH-1:0]	o_rs_no;
output reg [`CPU_REGNO_WIDTH-1:0]	o_rt_no;
output reg [`CPU_DATA_WIDTH-1:0]	o_imm;
output reg [`CPU_ALUOP_WIDTH-1:0]	o_alu_op;
output reg [4:0]			o_alu_inpt;
output wire				o_alu_ovf_ex;
output reg [4:0]			o_jump;
output wire				o_jump_link;
output reg [`CPU_IMDOP_WIDTH-1:0]	o_imuldiv_op;
output reg [`CPU_LSUOP_WIDTH-1:0]	o_lsu_op;
output reg				o_lsu_lns;
output reg				o_lsu_ext;


wire core_stall;
assign core_stall = i_exec_stall || i_mem_stall || i_fetch_stall;


reg [`CPU_INSTR_WIDTH-1:0] instr;	/* Instruction word */
reg [3:0] pc_high;			/* High four bits of PC */


/* Instruction fields */
wire [5:0]			op;	/* Opcode */
wire [`CPU_REGNO_WIDTH-1:0]	rd;	/* Destination register */
wire [`CPU_REGNO_WIDTH-1:0]	rs;	/* Source register 1 */
wire [`CPU_REGNO_WIDTH-1:0]	rt;	/* Source register 2 */
wire [`CPU_REGNO_WIDTH-1:0]	regimm;	/* REGIMM function */
wire [4:0]			shamt;	/* Shift amount */
wire [5:0]			func;	/* Function */

assign op	= instr[31:26];
assign rd	= instr[15:11];
assign rs	= instr[25:21];
assign rt	= instr[20:16];
assign regimm	= instr[20:16];
assign shamt	= instr[10:6];
assign func	= instr[5:0];


/* Immediate values */
wire [`CPU_DATA_WIDTH-1:0]	sign_ext;	/* Sign extended immediate */
wire [`CPU_DATA_WIDTH-1:0]	zero_ext;	/* Zero extended immediate */
wire [`CPU_DATA_WIDTH-1:0]	upper_imm;	/* Upper immediate (for LUI) */
wire [`CPU_DATA_WIDTH-1:0]	jump_target;	/* Unconditional jump target address */
wire [`CPU_DATA_WIDTH-1:0]	branch_offset;	/* Conditional branch offset */
wire [`CPU_DATA_WIDTH-1:0]	shift_amount;	/* Shift amount (for shift operations) */

assign sign_ext		= { {16{instr[15]}}, instr[15:0] };
assign zero_ext		= { 16'b0, instr[15:0] };
assign upper_imm	= { instr[15:0], 16'b0 };
assign jump_target	= { pc_high, instr[25:0], 2'b0 };
assign branch_offset	= { {14{instr[15]}}, instr[15:0], 2'b0 };
assign shift_amount	= { 27'b0, shamt };


/* RS source register */
always @(*)
begin
	case(op)
	`CPU_OP_J, `CPU_OP_JAL, `CPU_OP_LUI, `CPU_OP_COP0: o_rs_no = R0;
	default: o_rs_no = rs;
	endcase
end


/* RT source register */
always @(*)
begin
	case(op)
	`CPU_OP_SPECIAL: o_rt_no = (func == `CPU_FUNC_JR || func == `CPU_FUNC_JALR) ? R0 : rt;
	`CPU_OP_COP0: o_rt_no = i_cop0_rt_no;
	default: o_rt_no = rt;
	endcase
end


/* Decode destination register */
always @(*)
begin
	case(op)
	`CPU_OP_SPECIAL: begin
		case(func)
		`CPU_FUNC_JR, `CPU_FUNC_MTHI, `CPU_FUNC_MTLO, `CPU_FUNC_MULT,
		`CPU_FUNC_MULTU, `CPU_FUNC_DIV, `CPU_FUNC_DIVU: o_rd_no = R0;
		default: o_rd_no = rd;
		endcase
	end
	/***/
	`CPU_OP_REGIMM: begin
		case(regimm)
		`CPU_REGIMM_BLTZAL, `CPU_REGIMM_BGEZAL: o_rd_no = R31;
		default:  o_rd_no = R0;
		endcase
	end
	/***/
	`CPU_OP_J, `CPU_OP_BEQ, `CPU_OP_BNE, `CPU_OP_BLEZ,
	`CPU_OP_BGTZ, `CPU_OP_SB, `CPU_OP_SH, `CPU_OP_SW: o_rd_no = R0;
	/***/
	`CPU_OP_JAL: o_rd_no = R31;
	/***/
	`CPU_OP_ADDI, `CPU_OP_ADDIU, `CPU_OP_SLTI, `CPU_OP_SLTIU, `CPU_OP_ANDI,
	`CPU_OP_ORI, `CPU_OP_XORI, `CPU_OP_LUI, `CPU_OP_LB, `CPU_OP_LH, `CPU_OP_LW,
	`CPU_OP_LBU, `CPU_OP_LHU: o_rd_no = rt;
	/***/
	`CPU_OP_COP0: o_rd_no = (i_cop0_cop == `CPU_COP0_MF ? i_cop0_rt_no : R0);
	/***/
	default: o_rd_no = rd;
	endcase
end


/* Decode immediate value type to use */
always @(*)
begin
	case(op)
	`CPU_OP_SPECIAL: o_imm = shift_amount;
	/***/
	`CPU_OP_LUI: o_imm = upper_imm;
	/***/
	`CPU_OP_J, `CPU_OP_JAL: o_imm = jump_target;
	/***/
	`CPU_OP_ANDI, `CPU_OP_ORI, `CPU_OP_XORI: o_imm = zero_ext;
	/***/
	`CPU_OP_REGIMM, `CPU_OP_BEQ, `CPU_OP_BNE, `CPU_OP_BLEZ,
	`CPU_OP_BGTZ: o_imm = branch_offset;
	/***/
	default: o_imm = sign_ext;
	endcase
end


/* Decode ALU operation */
always @(*)
begin
	if(op == `CPU_OP_SPECIAL && (func == `CPU_FUNC_SLL || func == `CPU_FUNC_SLLV))
		o_alu_op = `CPU_ALUOP_SLL;
	else if(op == `CPU_OP_SPECIAL && (func == `CPU_FUNC_SRL || func == `CPU_FUNC_SRLV))
		o_alu_op = `CPU_ALUOP_SRL;
	else if(op == `CPU_OP_SPECIAL && (func == `CPU_FUNC_SRA || func == `CPU_FUNC_SRAV))
		o_alu_op = `CPU_ALUOP_SRA;
	else if(op == `CPU_OP_SPECIAL && (func == `CPU_FUNC_SUB || func == `CPU_FUNC_SUBU))
		o_alu_op = `CPU_ALUOP_SUB;
	else if((op == `CPU_OP_SPECIAL && func == `CPU_FUNC_AND) || (op == `CPU_OP_ANDI))
		o_alu_op = `CPU_ALUOP_AND;
	else if((op == `CPU_OP_SPECIAL && func == `CPU_FUNC_OR) || (op == `CPU_OP_ORI))
		o_alu_op = `CPU_ALUOP_OR;
	else if((op == `CPU_OP_SPECIAL && func == `CPU_FUNC_XOR) || (op == `CPU_OP_XORI))
		o_alu_op = `CPU_ALUOP_XOR;
	else if((op == `CPU_OP_SPECIAL && func == `CPU_FUNC_SLT) || (op == `CPU_OP_SLTI))
		o_alu_op = `CPU_ALUOP_SLT;
	else if((op == `CPU_OP_SPECIAL && func == `CPU_FUNC_SLTU) || (op == `CPU_OP_SLTIU))
		o_alu_op = `CPU_ALUOP_SLTU;
	else if(op == `CPU_OP_SPECIAL && func == `CPU_FUNC_NOR)
		o_alu_op = `CPU_ALUOP_NOR;
	else
		o_alu_op = `CPU_ALUOP_ADD;
end


/* Decode ALU inputs type */
always @(*)
begin
	case(op)
	`CPU_OP_SPECIAL: begin
		case(func)
		`CPU_FUNC_SLL, `CPU_FUNC_SRL, `CPU_FUNC_SRA: o_alu_inpt = DECODE_ALU_INPT_RTIMM;
		`CPU_FUNC_SLLV, `CPU_FUNC_SRLV, `CPU_FUNC_SRAV: o_alu_inpt = DECODE_ALU_INPT_RTRS;
		default: o_alu_inpt = DECODE_ALU_INPT_RSRT;
		endcase
	end
	/***/
	`CPU_OP_REGIMM, `CPU_OP_BEQ, `CPU_OP_BNE, `CPU_OP_BLEZ,
	`CPU_OP_BGTZ: o_alu_inpt = DECODE_ALU_INPT_PCIMM;
	/***/
	`CPU_OP_J, `CPU_OP_JAL, `CPU_OP_LUI, `CPU_OP_ADDI, `CPU_OP_ADDIU,
	`CPU_OP_SLTI, `CPU_OP_SLTIU, `CPU_OP_ANDI, `CPU_OP_ORI, `CPU_OP_XORI,
	`CPU_OP_LB, `CPU_OP_LH, `CPU_OP_LW, `CPU_OP_LBU, `CPU_OP_LHU,
	`CPU_OP_SB, `CPU_OP_SH, `CPU_OP_SW: o_alu_inpt = DECODE_ALU_INPT_RSIMM;
	/***/
	default: o_alu_inpt = DECODE_ALU_INPT_RSRT;
	endcase
end


/* Determine if integer overflow exception can be generated */
assign o_alu_ovf_ex = (op == `CPU_OP_SPECIAL && (func == `CPU_FUNC_ADD || func == `CPU_FUNC_SUB) ||
	op == `CPU_OP_ADDI) ? 1'b1 : 1'b0;


/* Decode jump type */
always @(*)
begin
	if(op == `CPU_OP_SPECIAL && (func == `CPU_FUNC_JR || func == `CPU_FUNC_JALR))
		o_jump = DECODE_JUMPT_JR;
	else if(op == `CPU_OP_REGIMM && (regimm == `CPU_REGIMM_BLTZ || regimm == `CPU_REGIMM_BLTZAL))
		o_jump = DECODE_JUMPT_BLTZ;
	else if(op == `CPU_OP_REGIMM && (regimm == `CPU_REGIMM_BGEZ || regimm == `CPU_REGIMM_BGEZAL))
		o_jump = DECODE_JUMPT_BGEZ;
	else if(op == `CPU_OP_J || op == `CPU_OP_JAL)
		o_jump = DECODE_JUMPT_J;
	else if(op == `CPU_OP_BEQ)
		o_jump = DECODE_JUMPT_BEQ;
	else if(op == `CPU_OP_BNE)
		o_jump = DECODE_JUMPT_BNE;
	else if(op == `CPU_OP_BLEZ)
		o_jump = DECODE_JUMPT_BLEZ;
	else if(op == `CPU_OP_BGTZ)
		o_jump = DECODE_JUMPT_BGTZ;
	else
		o_jump = DECODE_JUMPT_NONE;
end


/* Determine if jump and link required */
assign o_jump_link = (op == `CPU_OP_SPECIAL && func == `CPU_FUNC_JALR) ||
	(op == `CPU_OP_REGIMM && regimm == `CPU_REGIMM_BLTZAL) ||
	(op == `CPU_OP_REGIMM && regimm == `CPU_REGIMM_BGEZAL) ||
	(op == `CPU_OP_JAL) ? 1'b1 : 1'b0;


/* Decode integer multiplication and division unit operation */
always @(*)
begin
	if(op == `CPU_OP_SPECIAL && func == `CPU_FUNC_MFLO)
		o_imuldiv_op = `CPU_IMDOP_MFLO;
	else if(op == `CPU_OP_SPECIAL && func == `CPU_FUNC_MFHI)
		o_imuldiv_op = `CPU_IMDOP_MFHI;
	else if(op == `CPU_OP_SPECIAL && func == `CPU_FUNC_MTLO)
		o_imuldiv_op = `CPU_IMDOP_MTLO;
	else if(op == `CPU_OP_SPECIAL && func == `CPU_FUNC_MTHI)
		o_imuldiv_op = `CPU_IMDOP_MTHI;
	else if(op == `CPU_OP_SPECIAL && func == `CPU_FUNC_MULT)
		o_imuldiv_op = `CPU_IMDOP_MUL;
	else if(op == `CPU_OP_SPECIAL && func == `CPU_FUNC_MULTU)
		o_imuldiv_op = `CPU_IMDOP_MULU;
	else if(op == `CPU_OP_SPECIAL && func == `CPU_FUNC_DIV)
		o_imuldiv_op = `CPU_IMDOP_DIV;
	else if(op == `CPU_OP_SPECIAL && func == `CPU_FUNC_DIVU)
		o_imuldiv_op = `CPU_IMDOP_DIVU;
	else
		o_imuldiv_op = `CPU_IMDOP_IDLE;
end


/* Decode LSU operation */
always @(*)
begin
	o_lsu_op = `CPU_LSU_IDLE;
	o_lsu_lns = 1'b0;	/* LnW */
	o_lsu_ext = 1'b0;	/* Sign/zero extension (1/0) */

	if(op == `CPU_OP_LB)
	begin
		o_lsu_op = `CPU_LSU_BYTE;
		o_lsu_lns = 1'b1;
		o_lsu_ext = 1'b1;
	end
	else if(op == `CPU_OP_LH)
	begin
		o_lsu_op = `CPU_LSU_HWORD;
		o_lsu_lns = 1'b1;
		o_lsu_ext = 1'b1;
	end
	else if(op == `CPU_OP_LW)
	begin
		o_lsu_op = `CPU_LSU_WORD;
		o_lsu_lns = 1'b1;
		o_lsu_ext = 1'b1;
	end
	else if(op == `CPU_OP_LBU)
	begin
		o_lsu_op = `CPU_LSU_BYTE;
		o_lsu_lns = 1'b1;
	end
	else if(op == `CPU_OP_LHU)
	begin
		o_lsu_op = `CPU_LSU_HWORD;
		o_lsu_lns = 1'b1;
	end
	else if(op == `CPU_OP_SB)
	begin
		o_lsu_op = `CPU_LSU_BYTE;
		o_lsu_lns = 1'b0;
	end
	else if(op == `CPU_OP_SH)
	begin
		o_lsu_op = `CPU_LSU_HWORD;
		o_lsu_lns = 1'b0;
	end
	else if(op == `CPU_OP_SW)
	begin
		o_lsu_op = `CPU_LSU_WORD;
		o_lsu_lns = 1'b0;
	end
end


/* Sequential logic part */
always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		instr <= NOP;
		pc_high <= 4'b0;
	end
	else if(i_drop)
	begin
		instr <= NOP;
		pc_high <= 4'b0;
	end
	else if(!core_stall)
	begin
		instr <= i_instr;
		pc_high <= i_pc[31:28];
	end
end


endmodule /* decode */
