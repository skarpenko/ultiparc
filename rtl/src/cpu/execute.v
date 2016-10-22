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
 * Execute pipeline stage
 */

`include "cpu_common.vh"
`include "cpu_const.vh"


/* Execute stage */
module execute(
	clk,
	nrst,
	/* CU signals */
	i_pc_p0,
	i_pc_p1,
	o_exec_stall,
	i_mem_stall,
	i_fetch_stall,
	i_drop,
	/* Coprocessor 0 */
	i_cop0_op,
	i_cop0_cop,
	i_cop0_reg_val,
	/* Decoded instr */
	i_rd_no,
	i_rs_val,
	i_rt_val,
	i_imm,
	i_alu_op,
	i_alu_inpt,
	i_alu_ovf_ex,
	i_jump,
	i_jump_link,
	i_imuldiv_op,
	i_lsu_op,
	i_lsu_lns,
	i_lsu_ext,
	/* Stage output */
	o_rd_no,
	o_alu_result,
	o_jump_addr,
	o_jump_valid,
	o_lsu_op,
	o_lsu_lns,
	o_lsu_ext,
	o_mem_data
);
`include "reg_names.vh"
`include "decode_const.vh"
/* Inputs */
input wire				clk;
input wire				nrst;
/* CU signals */
input wire [`CPU_ADDR_WIDTH-1:0]	i_pc_p0;
input wire [`CPU_ADDR_WIDTH-1:0]	i_pc_p1;
output wire				o_exec_stall;
input wire				i_mem_stall;
input wire				i_fetch_stall;
input wire				i_drop;
/* Coprocessor 0 */
input wire				i_cop0_op;
input wire [`CPU_REGNO_WIDTH-1:0]	i_cop0_cop;
input wire [`CPU_REG_WIDTH-1:0]		i_cop0_reg_val;
/* Decoded instr */
input wire [`CPU_REGNO_WIDTH-1:0]	i_rd_no;
input wire [`CPU_REG_WIDTH-1:0]		i_rs_val;
input wire [`CPU_REG_WIDTH-1:0]		i_rt_val;
input wire [`CPU_DATA_WIDTH-1:0]	i_imm;
input wire [`CPU_ALUOP_WIDTH-1:0]	i_alu_op;
input wire [4:0]			i_alu_inpt;
input wire				i_alu_ovf_ex;
input wire [4:0]			i_jump;
input wire				i_jump_link;
input wire [1:0]			i_imuldiv_op;
input wire [`CPU_LSUOP_WIDTH-1:0]	i_lsu_op;
input wire				i_lsu_lns;
input wire				i_lsu_ext;
/* Stage output */
output wire [`CPU_REGNO_WIDTH-1:0]	o_rd_no;
output wire [`CPU_REG_WIDTH-1:0]	o_alu_result;
output wire [`CPU_ADDR_WIDTH-1:0]	o_jump_addr;
output wire				o_jump_valid;
output reg [`CPU_LSUOP_WIDTH-1:0]	o_lsu_op;
output reg				o_lsu_lns;
output reg				o_lsu_ext;
output reg [`CPU_DATA_WIDTH-1:0]	o_mem_data;


wire core_stall;
assign core_stall = o_exec_stall || i_mem_stall || i_fetch_stall;

assign o_exec_stall = 1'b0;

assign o_jump_addr = alu_result;


reg jump_instr;				/* Current operation is jump */
reg branch_taken;			/* Set if branch taken */
reg branch_link;			/* Branch requires link */
reg [`CPU_REGNO_WIDTH-1:0] rd_no;	/* Captured destination register number */
reg [`CPU_ADDR_WIDTH-1:0] pc_p0;	/* Captured PC of instruction after delay slot */


/* ALU interconnect */
reg [`CPU_ALUOP_WIDTH-1:0]	alu_op;
reg [`CPU_REG_WIDTH-1:0]	a;
reg [`CPU_REG_WIDTH-1:0]	b;
wire [`CPU_REG_WIDTH-1:0]	alu_result;
wire				ovflow;
wire				zero;
wire				neg;


/* Capture destination register */
always @(posedge clk or negedge nrst)
begin
	if(!nrst)
		rd_no <= {(`CPU_REGNO_WIDTH){1'b0}};
	else if(i_drop)
		rd_no <= {(`CPU_REGNO_WIDTH){1'b0}};
	else if(!core_stall)
		rd_no <= i_rd_no;
end


/* ALU operation */
always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		alu_op <= `CPU_ALUOP_ADD;
		a <= {(`CPU_REG_WIDTH){1'b0}};
		b <= {(`CPU_REG_WIDTH){1'b0}};
	end
	else if(!core_stall)
	begin
		alu_op <= i_alu_op;
		case(i_alu_inpt)
		DECODE_ALU_INPT_RTRS: begin
			a <= i_rt_val;
			b <= i_rs_val;
		end
		DECODE_ALU_INPT_RSIMM: begin
			a <= i_rs_val;
			b <= i_imm;
		end
		DECODE_ALU_INPT_RTIMM: begin
			a <= i_rt_val;
			b <= i_imm;
		end
		DECODE_ALU_INPT_PCIMM: begin
			a <= i_pc_p1;
			b <= i_imm;
		end
		default: begin /* DECODE_ALU_INPT_RSRT */
			a <= i_rs_val;
			b <= (i_cop0_op && i_cop0_cop == `CPU_COP0_MF ? i_cop0_reg_val : i_rt_val);
		end
		endcase
	end
end


/* LSU operation */
always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		o_lsu_op <= `CPU_LSU_IDLE;
		o_lsu_lns <= 1'b0;
		o_lsu_ext <= 1'b0;
		o_mem_data <= {(`CPU_DATA_WIDTH){1'b0}};
	end
	else if(i_drop)
	begin
		o_lsu_op <= `CPU_LSU_IDLE;
	end
	else if(!core_stall)
	begin
		o_lsu_op <= i_lsu_op;
		o_lsu_lns <= i_lsu_lns;
		o_lsu_ext <= i_lsu_ext;
		if(i_lsu_op != `CPU_LSU_IDLE) /* Pass only when needed */
			o_mem_data <= i_rt_val;
	end
end


/* Evaluate branch */
always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		branch_taken <= 1'b0;
		branch_link <= 1'b0;
		jump_instr <= 1'b0;
		pc_p0 <= {(`CPU_ADDR_WIDTH){1'b0}};
	end
	else if(i_drop)
	begin
		branch_taken <= 1'b0;
		jump_instr <= 1'b0;
	end
	else if(!core_stall)
	begin
		branch_link <= i_jump_link;
		jump_instr <= i_jump != DECODE_JUMPT_NONE ? 1'b1 : 1'b0;
		pc_p0 <= i_pc_p0;

		case(i_jump)
		DECODE_JUMPT_J, DECODE_JUMPT_JR: branch_taken <= 1'b1;
		DECODE_JUMPT_BLTZ: branch_taken <= i_rs_val[31];
		DECODE_JUMPT_BGEZ: branch_taken <= ~i_rs_val[31];
		DECODE_JUMPT_BEQ: branch_taken <= i_rs_val == i_rt_val;
		DECODE_JUMPT_BNE: branch_taken <= i_rs_val != i_rt_val;
		DECODE_JUMPT_BLEZ: branch_taken <= i_rs_val[31] || ~|i_rs_val;
		DECODE_JUMPT_BGTZ: branch_taken <= !i_rs_val[31] && |i_rs_val;
		default: branch_taken <= 1'b0;
		endcase
	end
end


/* Set outputs */
assign o_rd_no = jump_instr && !branch_taken ? R0 : rd_no;
assign o_jump_valid = branch_taken;
assign o_alu_result = jump_instr && branch_link ? pc_p0 : alu_result;


/* ALU instance */
alu alu(
	.alu_op(alu_op),
	.a(a),
	.b(b),
	.result(alu_result),
	.ovflow(ovflow),
	.zero(zero),
	.neg(neg)
);


endmodule /* execute */
