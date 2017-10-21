/*
 * Copyright (c) 2015-2017 The Ultiparc Project. All rights reserved.
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

`include "uparc_cpu_common.vh"
`include "uparc_cpu_const.vh"


/* Decode stage */
module uparc_decode(
	clk,
	nrst,
	/* CU signals */
	i_pc,
	i_exec_stall,
	i_mem_stall,
	i_fetch_stall,
	o_decode_error,
	i_nullify,
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
	o_sw_trap,
	o_lsu_op,
	o_lsu_lns,
	o_lsu_ext
);
`include "uparc_reg_names.vh"
`include "uparc_decode_const.vh"
localparam [`UPARC_INSTR_WIDTH-1:0] NOP = 32'h0000_0000;
/* Inputs */
input wire				clk;
input wire				nrst;
/* CU signals */
input wire [`UPARC_ADDR_WIDTH-1:0]	i_pc;
input wire				i_exec_stall;
input wire				i_mem_stall;
input wire				i_fetch_stall;
output reg				o_decode_error;
input wire				i_nullify;
/* Coprocessor 0 */
input wire [`UPARC_REGNO_WIDTH-1:0]	i_cop0_cop;
input wire [`UPARC_REGNO_WIDTH-1:0]	i_cop0_reg_no;
input wire [`UPARC_REGNO_WIDTH-1:0]	i_cop0_rt_no;
/* Fetched instruction */
input wire [`UPARC_INSTR_WIDTH-1:0]	i_instr;
/* Decoded instr */
output reg [`UPARC_REGNO_WIDTH-1:0]	o_rd_no;
output reg [`UPARC_REGNO_WIDTH-1:0]	o_rs_no;
output reg [`UPARC_REGNO_WIDTH-1:0]	o_rt_no;
output reg [`UPARC_DATA_WIDTH-1:0]	o_imm;
output reg [`UPARC_ALUOP_WIDTH-1:0]	o_alu_op;
output reg [4:0]			o_alu_inpt;
output wire				o_alu_ovf_ex;
output reg [4:0]			o_jump;
output wire				o_jump_link;
output reg [`UPARC_IMDOP_WIDTH-1:0]	o_imuldiv_op;
output reg [`UPARC_SWTRP_WIDTH-1:0]	o_sw_trap;
output reg [`UPARC_LSUOP_WIDTH-1:0]	o_lsu_op;
output reg				o_lsu_lns;
output reg				o_lsu_ext;


wire core_stall = i_exec_stall || i_mem_stall || i_fetch_stall;


reg [`UPARC_INSTR_WIDTH-1:0] instr;	/* Instruction word */
reg [3:0] pc_high;			/* High four bits of PC */


/* Instruction fields */
wire [5:0]			op;	/* Opcode */
wire [`UPARC_REGNO_WIDTH-1:0]	rd;	/* Destination register */
wire [`UPARC_REGNO_WIDTH-1:0]	rs;	/* Source register 1 */
wire [`UPARC_REGNO_WIDTH-1:0]	rt;	/* Source register 2 */
wire [`UPARC_REGNO_WIDTH-1:0]	regimm;	/* REGIMM function */
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
wire [`UPARC_DATA_WIDTH-1:0]	sign_ext;	/* Sign extended immediate */
wire [`UPARC_DATA_WIDTH-1:0]	zero_ext;	/* Zero extended immediate */
wire [`UPARC_DATA_WIDTH-1:0]	upper_imm;	/* Upper immediate (for LUI) */
wire [`UPARC_DATA_WIDTH-1:0]	jump_target;	/* Unconditional jump target address */
wire [`UPARC_DATA_WIDTH-1:0]	branch_offset;	/* Conditional branch offset */
wire [`UPARC_DATA_WIDTH-1:0]	shift_amount;	/* Shift amount (for shift operations) */

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
	`UPARC_OP_J, `UPARC_OP_JAL, `UPARC_OP_LUI, `UPARC_OP_COP0: o_rs_no = R0;
	default: o_rs_no = rs;
	endcase
end


/* RT source register */
always @(*)
begin
	case(op)
	`UPARC_OP_SPECIAL: o_rt_no = (func == `UPARC_FUNC_JR || func == `UPARC_FUNC_JALR) ? R0 : rt;
	`UPARC_OP_COP0: o_rt_no = i_cop0_rt_no;
	default: o_rt_no = rt;
	endcase
end


/* Decode destination register */
always @(*)
begin
	case(op)
	`UPARC_OP_SPECIAL: begin
		case(func)
		`UPARC_FUNC_JR, `UPARC_FUNC_MTHI, `UPARC_FUNC_MTLO, `UPARC_FUNC_MULT,
		`UPARC_FUNC_MULTU, `UPARC_FUNC_DIV, `UPARC_FUNC_DIVU: o_rd_no = R0;
		default: o_rd_no = rd;
		endcase
	end
	/***/
	`UPARC_OP_REGIMM: begin
		case(regimm)
		`UPARC_REGIMM_BLTZAL, `UPARC_REGIMM_BGEZAL: o_rd_no = R31;
		default:  o_rd_no = R0;
		endcase
	end
	/***/
	`UPARC_OP_J, `UPARC_OP_BEQ, `UPARC_OP_BNE, `UPARC_OP_BLEZ,
	`UPARC_OP_BGTZ, `UPARC_OP_SB, `UPARC_OP_SH, `UPARC_OP_SW: o_rd_no = R0;
	/***/
	`UPARC_OP_JAL: o_rd_no = R31;
	/***/
	`UPARC_OP_ADDI, `UPARC_OP_ADDIU, `UPARC_OP_SLTI, `UPARC_OP_SLTIU, `UPARC_OP_ANDI,
	`UPARC_OP_ORI, `UPARC_OP_XORI, `UPARC_OP_LUI, `UPARC_OP_LB, `UPARC_OP_LH, `UPARC_OP_LW,
	`UPARC_OP_LBU, `UPARC_OP_LHU: o_rd_no = rt;
	/***/
	`UPARC_OP_COP0: o_rd_no = (i_cop0_cop == `UPARC_COP0_MF ? i_cop0_rt_no : R0);
	/***/
	default: o_rd_no = rd;
	endcase
end


/* Decode immediate value type to use */
always @(*)
begin
	case(op)
	`UPARC_OP_SPECIAL: o_imm = shift_amount;
	/***/
	`UPARC_OP_LUI: o_imm = upper_imm;
	/***/
	`UPARC_OP_J, `UPARC_OP_JAL: o_imm = jump_target;
	/***/
	`UPARC_OP_ANDI, `UPARC_OP_ORI, `UPARC_OP_XORI: o_imm = zero_ext;
	/***/
	`UPARC_OP_REGIMM, `UPARC_OP_BEQ, `UPARC_OP_BNE, `UPARC_OP_BLEZ,
	`UPARC_OP_BGTZ: o_imm = branch_offset;
	/***/
	default: o_imm = sign_ext;
	endcase
end


/* Decode ALU operation */
always @(*)
begin
	if(op == `UPARC_OP_SPECIAL && (func == `UPARC_FUNC_SLL || func == `UPARC_FUNC_SLLV))
		o_alu_op = `UPARC_ALUOP_SLL;
	else if(op == `UPARC_OP_SPECIAL && (func == `UPARC_FUNC_SRL || func == `UPARC_FUNC_SRLV))
		o_alu_op = `UPARC_ALUOP_SRL;
	else if(op == `UPARC_OP_SPECIAL && (func == `UPARC_FUNC_SRA || func == `UPARC_FUNC_SRAV))
		o_alu_op = `UPARC_ALUOP_SRA;
	else if(op == `UPARC_OP_SPECIAL && (func == `UPARC_FUNC_SUB || func == `UPARC_FUNC_SUBU))
		o_alu_op = `UPARC_ALUOP_SUB;
	else if((op == `UPARC_OP_SPECIAL && func == `UPARC_FUNC_AND) || (op == `UPARC_OP_ANDI))
		o_alu_op = `UPARC_ALUOP_AND;
	else if((op == `UPARC_OP_SPECIAL && func == `UPARC_FUNC_OR) || (op == `UPARC_OP_ORI))
		o_alu_op = `UPARC_ALUOP_OR;
	else if((op == `UPARC_OP_SPECIAL && func == `UPARC_FUNC_XOR) || (op == `UPARC_OP_XORI))
		o_alu_op = `UPARC_ALUOP_XOR;
	else if((op == `UPARC_OP_SPECIAL && func == `UPARC_FUNC_SLT) || (op == `UPARC_OP_SLTI))
		o_alu_op = `UPARC_ALUOP_SLT;
	else if((op == `UPARC_OP_SPECIAL && func == `UPARC_FUNC_SLTU) || (op == `UPARC_OP_SLTIU))
		o_alu_op = `UPARC_ALUOP_SLTU;
	else if(op == `UPARC_OP_SPECIAL && func == `UPARC_FUNC_NOR)
		o_alu_op = `UPARC_ALUOP_NOR;
	else
		o_alu_op = `UPARC_ALUOP_ADD;
end


/* Decode ALU inputs type */
always @(*)
begin
	case(op)
	`UPARC_OP_SPECIAL: begin
		case(func)
		`UPARC_FUNC_SLL, `UPARC_FUNC_SRL, `UPARC_FUNC_SRA: o_alu_inpt = DECODE_ALU_INPT_RTIMM;
		`UPARC_FUNC_SLLV, `UPARC_FUNC_SRLV, `UPARC_FUNC_SRAV: o_alu_inpt = DECODE_ALU_INPT_RTRS;
		default: o_alu_inpt = DECODE_ALU_INPT_RSRT;
		endcase
	end
	/***/
	`UPARC_OP_REGIMM, `UPARC_OP_BEQ, `UPARC_OP_BNE, `UPARC_OP_BLEZ,
	`UPARC_OP_BGTZ: o_alu_inpt = DECODE_ALU_INPT_PCIMM;
	/***/
	`UPARC_OP_J, `UPARC_OP_JAL, `UPARC_OP_LUI, `UPARC_OP_ADDI, `UPARC_OP_ADDIU,
	`UPARC_OP_SLTI, `UPARC_OP_SLTIU, `UPARC_OP_ANDI, `UPARC_OP_ORI, `UPARC_OP_XORI,
	`UPARC_OP_LB, `UPARC_OP_LH, `UPARC_OP_LW, `UPARC_OP_LBU, `UPARC_OP_LHU,
	`UPARC_OP_SB, `UPARC_OP_SH, `UPARC_OP_SW: o_alu_inpt = DECODE_ALU_INPT_RSIMM;
	/***/
	default: o_alu_inpt = DECODE_ALU_INPT_RSRT;
	endcase
end


/* Determine if integer overflow exception can be generated */
assign o_alu_ovf_ex = (op == `UPARC_OP_SPECIAL && (func == `UPARC_FUNC_ADD || func == `UPARC_FUNC_SUB) ||
	op == `UPARC_OP_ADDI) ? 1'b1 : 1'b0;


/* Decode jump type */
always @(*)
begin
	if(op == `UPARC_OP_SPECIAL && (func == `UPARC_FUNC_JR || func == `UPARC_FUNC_JALR))
		o_jump = DECODE_JUMPT_JR;
	else if(op == `UPARC_OP_REGIMM && (regimm == `UPARC_REGIMM_BLTZ || regimm == `UPARC_REGIMM_BLTZAL))
		o_jump = DECODE_JUMPT_BLTZ;
	else if(op == `UPARC_OP_REGIMM && (regimm == `UPARC_REGIMM_BGEZ || regimm == `UPARC_REGIMM_BGEZAL))
		o_jump = DECODE_JUMPT_BGEZ;
	else if(op == `UPARC_OP_J || op == `UPARC_OP_JAL)
		o_jump = DECODE_JUMPT_J;
	else if(op == `UPARC_OP_BEQ)
		o_jump = DECODE_JUMPT_BEQ;
	else if(op == `UPARC_OP_BNE)
		o_jump = DECODE_JUMPT_BNE;
	else if(op == `UPARC_OP_BLEZ)
		o_jump = DECODE_JUMPT_BLEZ;
	else if(op == `UPARC_OP_BGTZ)
		o_jump = DECODE_JUMPT_BGTZ;
	else
		o_jump = DECODE_JUMPT_NONE;
end


/* Determine if jump and link required */
assign o_jump_link = (op == `UPARC_OP_SPECIAL && func == `UPARC_FUNC_JALR) ||
	(op == `UPARC_OP_REGIMM && regimm == `UPARC_REGIMM_BLTZAL) ||
	(op == `UPARC_OP_REGIMM && regimm == `UPARC_REGIMM_BGEZAL) ||
	(op == `UPARC_OP_JAL) ? 1'b1 : 1'b0;


/* Decode integer multiplication and division unit operation */
always @(*)
begin
	if(op == `UPARC_OP_SPECIAL && func == `UPARC_FUNC_MFLO)
		o_imuldiv_op = `UPARC_IMDOP_MFLO;
	else if(op == `UPARC_OP_SPECIAL && func == `UPARC_FUNC_MFHI)
		o_imuldiv_op = `UPARC_IMDOP_MFHI;
	else if(op == `UPARC_OP_SPECIAL && func == `UPARC_FUNC_MTLO)
		o_imuldiv_op = `UPARC_IMDOP_MTLO;
	else if(op == `UPARC_OP_SPECIAL && func == `UPARC_FUNC_MTHI)
		o_imuldiv_op = `UPARC_IMDOP_MTHI;
	else if(op == `UPARC_OP_SPECIAL && func == `UPARC_FUNC_MULT)
		o_imuldiv_op = `UPARC_IMDOP_MUL;
	else if(op == `UPARC_OP_SPECIAL && func == `UPARC_FUNC_MULTU)
		o_imuldiv_op = `UPARC_IMDOP_MULU;
	else if(op == `UPARC_OP_SPECIAL && func == `UPARC_FUNC_DIV)
		o_imuldiv_op = `UPARC_IMDOP_DIV;
	else if(op == `UPARC_OP_SPECIAL && func == `UPARC_FUNC_DIVU)
		o_imuldiv_op = `UPARC_IMDOP_DIVU;
	else
		o_imuldiv_op = `UPARC_IMDOP_IDLE;
end


/* Decode software trap */
always @(*)
begin
	if(op == `UPARC_OP_SPECIAL && func == `UPARC_FUNC_SYSCALL)
		o_sw_trap = `UPARC_SWTRP_SYSCALL;
	else if(op == `UPARC_OP_SPECIAL && func == `UPARC_FUNC_BREAK)
		o_sw_trap = `UPARC_SWTRP_BREAK;
	else
		o_sw_trap = `UPARC_SWTRP_NONE;
end


/* Decode LSU operation */
always @(*)
begin
	o_lsu_op = `UPARC_LSU_IDLE;
	o_lsu_lns = 1'b0;	/* LnW */
	o_lsu_ext = 1'b0;	/* Sign/zero extension (1/0) */

	if(op == `UPARC_OP_LB)
	begin
		o_lsu_op = `UPARC_LSU_BYTE;
		o_lsu_lns = 1'b1;
		o_lsu_ext = 1'b1;
	end
	else if(op == `UPARC_OP_LH)
	begin
		o_lsu_op = `UPARC_LSU_HWORD;
		o_lsu_lns = 1'b1;
		o_lsu_ext = 1'b1;
	end
	else if(op == `UPARC_OP_LW)
	begin
		o_lsu_op = `UPARC_LSU_WORD;
		o_lsu_lns = 1'b1;
		o_lsu_ext = 1'b1;
	end
	else if(op == `UPARC_OP_LBU)
	begin
		o_lsu_op = `UPARC_LSU_BYTE;
		o_lsu_lns = 1'b1;
	end
	else if(op == `UPARC_OP_LHU)
	begin
		o_lsu_op = `UPARC_LSU_HWORD;
		o_lsu_lns = 1'b1;
	end
	else if(op == `UPARC_OP_SB)
	begin
		o_lsu_op = `UPARC_LSU_BYTE;
		o_lsu_lns = 1'b0;
	end
	else if(op == `UPARC_OP_SH)
	begin
		o_lsu_op = `UPARC_LSU_HWORD;
		o_lsu_lns = 1'b0;
	end
	else if(op == `UPARC_OP_SW)
	begin
		o_lsu_op = `UPARC_LSU_WORD;
		o_lsu_lns = 1'b0;
	end
end


/* Detect instruction format errors */
always @(*)
begin
	case(op)
	/** SPECIAL instruction class **/
	`UPARC_OP_SPECIAL: begin
		case(func)
		`UPARC_FUNC_SLL, `UPARC_FUNC_SRL,
		`UPARC_FUNC_SRA: o_decode_error = |rs ? 1'b1 : 1'b0;
		/****/
		`UPARC_FUNC_SLLV, `UPARC_FUNC_SRLV, `UPARC_FUNC_SRAV, `UPARC_FUNC_ADD,
		`UPARC_FUNC_ADDU, `UPARC_FUNC_SUB, `UPARC_FUNC_SUBU, `UPARC_FUNC_AND,
		`UPARC_FUNC_OR, `UPARC_FUNC_XOR, `UPARC_FUNC_NOR, `UPARC_FUNC_SLT,
		`UPARC_FUNC_SLTU: o_decode_error = |shamt ? 1'b1 : 1'b0;
		/****/
		`UPARC_FUNC_JR, `UPARC_FUNC_MTHI,
		`UPARC_FUNC_MTLO: o_decode_error = |{ rt, rd, shamt } ? 1'b1 : 1'b0;
		/****/
		`UPARC_FUNC_JALR: o_decode_error = |{ rt, shamt } ? 1'b1 : 1'b0;
		/****/
		`UPARC_FUNC_MFHI,
		`UPARC_FUNC_MFLO: o_decode_error = |{ rs, rt, shamt } ? 1'b1 : 1'b0;
		/****/
		`UPARC_FUNC_MULT, `UPARC_FUNC_MULTU, `UPARC_FUNC_DIV,
		`UPARC_FUNC_DIVU: o_decode_error = |{ rd, shamt } ? 1'b1 : 1'b0;
		/****/
		`UPARC_FUNC_SYSCALL, `UPARC_FUNC_BREAK: o_decode_error = 1'b0;
		/****/
		default:  o_decode_error = 1'b1;
		endcase
	end
	/** REGIMM instruction class **/
	`UPARC_OP_REGIMM: begin
		case(regimm)
		`UPARC_REGIMM_BLTZ, `UPARC_REGIMM_BGEZ,
		`UPARC_REGIMM_BLTZAL, `UPARC_REGIMM_BGEZAL: o_decode_error = 1'b0;
		/****/
		default: o_decode_error = 1'b1;
		endcase
	end
	/** Conditional branch instructions **/
	`UPARC_OP_BLEZ, `UPARC_OP_BGTZ: o_decode_error = |rt ? 1'b1 : 1'b0;
	/** Load upper immediate **/
	`UPARC_OP_LUI: o_decode_error = |rs ? 1'b1 : 1'b0;
	/** Unconditional and conditional branch instructions **/
	`UPARC_OP_J, `UPARC_OP_JAL, `UPARC_OP_BEQ, `UPARC_OP_BNE: o_decode_error = 1'b0;
	/** Instructions with immediate operand **/
	`UPARC_OP_ADDI, `UPARC_OP_ADDIU, `UPARC_OP_SLTI, `UPARC_OP_SLTIU, `UPARC_OP_ANDI,
	`UPARC_OP_ORI, `UPARC_OP_XORI: o_decode_error = 1'b0;
	/** Load/store instructions **/
	`UPARC_OP_LB, `UPARC_OP_LH, `UPARC_OP_LW, `UPARC_OP_LBU, `UPARC_OP_LHU,
	`UPARC_OP_SB, `UPARC_OP_SH, `UPARC_OP_SW: o_decode_error = 1'b0;
	/** Coprocessor 0 instructions **/
	`UPARC_OP_COP0: o_decode_error = 1'b0;	/* Other fields are checked by Cop0 */
	/****/
	default: o_decode_error = 1'b1;
	endcase
end


/* Sequential logic part */
always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		instr <= NOP;
		pc_high <= 4'b0;
	end
	else if(!core_stall)
	begin
		instr <= !i_nullify ? i_instr : NOP;
		pc_high <= i_pc[31:28];
	end
end


endmodule /* uparc_decode */
