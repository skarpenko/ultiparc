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
	o_drop,
	/* Decoded instr */
	i_op,
	i_dst_gpr,
	i_src1_gpr_v,
	i_src2_gpr_v,
	i_src3_se_v,
	i_src3_ze_v,
	i_src3_sh16_v,
	i_src3_j_v,
	i_src3_broff_v,
	i_shamt,
	i_func,
	i_regimm,
	/* Stage output */
	o_op,
	o_dst_gpr,
	o_result,
	o_mem_data,
	o_j_addr_valid,
	o_j_addr
);
/* Inputs */
input wire				clk;
input wire				nrst;
/* CU signals */
input wire [`CPU_ADDR_WIDTH-1:0]	i_pc_p0;
input wire [`CPU_ADDR_WIDTH-1:0]	i_pc_p1;
output wire				o_exec_stall;
input wire				i_mem_stall;
input wire				i_fetch_stall;
output reg				o_drop;
/* Decoded instr */
input wire [5:0]			i_op;
input wire [`CPU_REGNO_WIDTH-1:0]	i_dst_gpr;
input wire [`CPU_REG_WIDTH-1:0]		i_src1_gpr_v;
input wire [`CPU_REG_WIDTH-1:0]		i_src2_gpr_v;
input wire [`CPU_DATA_WIDTH-1:0]	i_src3_se_v;
input wire [`CPU_DATA_WIDTH-1:0]	i_src3_ze_v;
input wire [`CPU_DATA_WIDTH-1:0]	i_src3_sh16_v;
input wire [`CPU_ADDR_WIDTH-1:0]	i_src3_j_v;
input wire [`CPU_ADDR_WIDTH-1:0]	i_src3_broff_v;
input wire [4:0]			i_shamt;
input wire [5:0]			i_func;
input wire [`CPU_REGNO_WIDTH-1:0]	i_regimm;
/* Stage output */
output reg [5:0]			o_op;
output reg [`CPU_REGNO_WIDTH-1:0]	o_dst_gpr;
output reg [`CPU_DATA_WIDTH-1:0]	o_result;
output reg [`CPU_DATA_WIDTH-1:0]	o_mem_data;
output reg				o_j_addr_valid;
output reg [`CPU_ADDR_WIDTH-1:0]	o_j_addr;


wire core_stall;
assign core_stall = o_exec_stall || i_mem_stall || i_fetch_stall;

assign o_exec_stall = 1'b0;


reg [`CPU_ALUOP_WIDTH-1:0] alu_op;
reg [`CPU_REG_WIDTH-1:0] a;
reg [`CPU_REG_WIDTH-1:0] b;
wire [`CPU_REG_WIDTH-1:0] alu_result;
wire ovflow;
wire zero;
wire neg;

reg j_mux;
reg [`CPU_ADDR_WIDTH-1:0]	j_addr;

reg r_mux;
reg [`CPU_REG_WIDTH-1:0]	result;


always @(*)
	o_j_addr = !j_mux ? j_addr : alu_result;

always @(*)
	o_result = !r_mux ? alu_result : result;



always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		o_op <= 6'b0;
		o_dst_gpr <= {(`CPU_REGNO_WIDTH){1'b0}};
//		o_result <= {(`CPU_DATA_WIDTH){1'b0}};
		o_mem_data <= {(`CPU_DATA_WIDTH){1'b0}};
		o_j_addr_valid <= 1'b0;
//		o_j_addr <= {(`CPU_ADDR_WIDTH){1'b0}};
		o_drop <= 1'b0;
		j_mux <= 1'b0;
		j_addr <= {(`CPU_ADDR_WIDTH){1'b0}};
		r_mux <= 1'b0;
		result <= {(`CPU_REG_WIDTH){1'b0}};
		a <= {(`CPU_REG_WIDTH){1'b0}};
		b <= {(`CPU_REG_WIDTH){1'b0}};
	end
	else if(!core_stall)
	begin
		o_j_addr_valid <= 1'b0;
		o_drop <= 1'b0;
		j_mux <= 1'b0;
		r_mux <= 1'b0;

		o_op <= i_op;
		o_dst_gpr <= i_dst_gpr;

		if(i_op == `CPU_OP_SPECIAL && i_func == `CPU_FUNC_SLL)
		begin
			a <= i_src2_gpr_v;
			b <= { 27'b0, i_shamt };
			alu_op <= `CPU_ALUOP_SLL;
		end
		else if(i_op == `CPU_OP_SPECIAL && i_func == `CPU_FUNC_SRL)
		begin
			a <= i_src2_gpr_v;
			b <= { 27'b0, i_shamt };
			alu_op <= `CPU_ALUOP_SRL;
		end
		else if(i_op == `CPU_OP_SPECIAL && i_func == `CPU_FUNC_SRA)
		begin
			a <= i_src2_gpr_v;
			b <= { 27'b0, i_shamt };
			alu_op <= `CPU_ALUOP_SRA;
		end
		else if(i_op == `CPU_OP_SPECIAL && i_func == `CPU_FUNC_SLLV)
		begin
			a <= i_src2_gpr_v;
			b <= { 27'b0, i_src1_gpr_v[4:0] };
			alu_op <= `CPU_ALUOP_SLL;
		end
		else if(i_op == `CPU_OP_SPECIAL && i_func == `CPU_FUNC_SRLV)
		begin
			a <= i_src2_gpr_v;
			b <= { 27'b0, i_src1_gpr_v[4:0] };
			alu_op <= `CPU_ALUOP_SRL;
		end
		else if(i_op == `CPU_OP_SPECIAL && i_func == `CPU_FUNC_SRAV)
		begin
			a <= i_src2_gpr_v;
			b <= { 27'b0, i_src1_gpr_v[4:0] };
			alu_op <= `CPU_ALUOP_SRA;
		end
		else if(i_op == `CPU_OP_SPECIAL && i_func == `CPU_FUNC_JR)
		begin
			o_dst_gpr <= {(`CPU_REGNO_WIDTH){1'b0}};
			o_j_addr_valid <= 1'b1;
			j_addr <= i_src1_gpr_v;
		end
		else if(i_op == `CPU_OP_SPECIAL && i_func == `CPU_FUNC_JALR)
		begin
			o_j_addr_valid <= 1'b1;
			j_addr <= i_src1_gpr_v;
			result <= i_pc_p0;
			r_mux <= 1'b1;
		end
		else if(i_op == `CPU_OP_SPECIAL && i_func == `CPU_FUNC_SYSCALL)
		begin
		end
		else if(i_op == `CPU_OP_SPECIAL && i_func == `CPU_FUNC_BREAK)
		begin
		end
		else if(i_op == `CPU_OP_SPECIAL && i_func == `CPU_FUNC_MFHI)
		begin
		end
		else if(i_op == `CPU_OP_SPECIAL && i_func == `CPU_FUNC_MTHI)
		begin
		end
		else if(i_op == `CPU_OP_SPECIAL && i_func == `CPU_FUNC_MFLO)
		begin
		end
		else if(i_op == `CPU_OP_SPECIAL && i_func == `CPU_FUNC_MTLO)
		begin
		end
		else if(i_op == `CPU_OP_SPECIAL && i_func == `CPU_FUNC_MULT)
		begin
		end
		else if(i_op == `CPU_OP_SPECIAL && i_func == `CPU_FUNC_MULTU)
		begin
		end
		else if(i_op == `CPU_OP_SPECIAL && i_func == `CPU_FUNC_DIV)
		begin
		end
		else if(i_op == `CPU_OP_SPECIAL && i_func == `CPU_FUNC_DIVU)
		begin
		end
		else if(i_op == `CPU_OP_SPECIAL && i_func == `CPU_FUNC_ADD)
		begin
			a <= i_src1_gpr_v;
			b <= i_src2_gpr_v;
			alu_op <= `CPU_ALUOP_ADD;
		end
		else if(i_op == `CPU_OP_SPECIAL && i_func == `CPU_FUNC_ADDU)
		begin
			a <= i_src1_gpr_v;
			b <= i_src2_gpr_v;
			alu_op <= `CPU_ALUOP_ADD;
		end
		else if(i_op == `CPU_OP_SPECIAL && i_func == `CPU_FUNC_SUB)
		begin
			a <= i_src1_gpr_v;
			b <= i_src2_gpr_v;
			alu_op <= `CPU_ALUOP_SUB;
		end
		else if(i_op == `CPU_OP_SPECIAL && i_func == `CPU_FUNC_SUBU)
		begin
			a <= i_src1_gpr_v;
			b <= i_src2_gpr_v;
			alu_op <= `CPU_ALUOP_SUB;
		end
		else if(i_op == `CPU_OP_SPECIAL && i_func == `CPU_FUNC_AND)
		begin
			a <= i_src1_gpr_v;
			b <= i_src2_gpr_v;
			alu_op <= `CPU_ALUOP_AND;
		end
		else if(i_op == `CPU_OP_SPECIAL && i_func == `CPU_FUNC_OR)
		begin
			a <= i_src1_gpr_v;
			b <= i_src2_gpr_v;
			alu_op <= `CPU_ALUOP_OR;
		end
		else if(i_op == `CPU_OP_SPECIAL && i_func == `CPU_FUNC_XOR)
		begin
			a <= i_src1_gpr_v;
			b <= i_src2_gpr_v;
			alu_op <= `CPU_ALUOP_XOR;
		end
		else if(i_op == `CPU_OP_SPECIAL && i_func == `CPU_FUNC_NOR)
		begin
			a <= i_src1_gpr_v;
			b <= i_src2_gpr_v;
			alu_op <= `CPU_ALUOP_NOR;
		end
		else if(i_op == `CPU_OP_SPECIAL && i_func == `CPU_FUNC_SLT)
		begin
			a <= i_src1_gpr_v;
			b <= i_src2_gpr_v;
			alu_op <= `CPU_ALUOP_SLT;
		end
		else if(i_op == `CPU_OP_SPECIAL && i_func == `CPU_FUNC_SLTU)
		begin
			a <= i_src1_gpr_v;
			b <= i_src2_gpr_v;
			alu_op <= `CPU_ALUOP_SLTU;
		end
		else if(i_op == `CPU_OP_REGIMM && i_regimm == `CPU_REGIMM_BLTZ)
		begin
			a <= i_pc_p1;
			b <= i_src3_broff_v;
			alu_op <= `CPU_ALUOP_ADD;
			j_mux <= 1'b1;
			o_j_addr_valid <= i_src1_gpr_v[31];
			o_drop <= ~i_src1_gpr_v[31];
			o_dst_gpr <= {(`CPU_REGNO_WIDTH){1'b0}};
		end
		else if(i_op == `CPU_OP_REGIMM && i_regimm == `CPU_REGIMM_BGEZ)
		begin
			a <= i_pc_p1;
			b <= i_src3_broff_v;
			alu_op <= `CPU_ALUOP_ADD;
			j_mux <= 1'b1;
			o_j_addr_valid <= ~i_src1_gpr_v[31];
			o_drop <= i_src1_gpr_v[31];
			o_dst_gpr <= {(`CPU_REGNO_WIDTH){1'b0}};

		end
		else if(i_op == `CPU_OP_REGIMM && i_regimm == `CPU_REGIMM_BLTZAL)
		begin
			a <= i_pc_p1;
			b <= i_src3_broff_v;
			alu_op <= `CPU_ALUOP_ADD;
			j_mux <= 1'b1;
			o_j_addr_valid <= i_src1_gpr_v[31];
			o_drop <= ~i_src1_gpr_v[31];
			o_dst_gpr <= i_src1_gpr_v[31] ? 31 : {(`CPU_REGNO_WIDTH){1'b0}};
			result <= i_pc_p0;
			r_mux <= i_src1_gpr_v[31];
		end
		else if(i_op == `CPU_OP_REGIMM && i_regimm == `CPU_REGIMM_BGEZAL)
		begin
			a <= i_pc_p1;
			b <= i_src3_broff_v;
			alu_op <= `CPU_ALUOP_ADD;
			j_mux <= 1'b1;
			o_j_addr_valid <= ~i_src1_gpr_v[31];
			o_drop <= i_src1_gpr_v[31];
			o_dst_gpr <= ~i_src1_gpr_v[31] ? 31 : {(`CPU_REGNO_WIDTH){1'b0}};
			result <= i_pc_p0;
			r_mux <= ~i_src1_gpr_v[31];
		end
		else if(i_op == `CPU_OP_J)
		begin
			j_addr <= i_src3_j_v;
			o_j_addr_valid <= 1'b1;
			o_dst_gpr <= {(`CPU_REGNO_WIDTH){1'b0}};
		end
		else if(i_op == `CPU_OP_JAL)
		begin
			j_addr <= i_src3_j_v;
			o_j_addr_valid <= 1'b1;
			o_dst_gpr <= 31;
			result <= i_pc_p0;
			r_mux <= 1'b1;
		end
		else if(i_op == `CPU_OP_BEQ)
		begin
			a <= i_pc_p1;
			b <= i_src3_broff_v;
			alu_op <= `CPU_ALUOP_ADD;
			j_mux <= 1'b1;
			o_j_addr_valid <= i_src1_gpr_v == i_src2_gpr_v;
			o_drop <= i_src1_gpr_v != i_src2_gpr_v;
			o_dst_gpr <= {(`CPU_REGNO_WIDTH){1'b0}};
		end
		else if(i_op == `CPU_OP_BNE)
		begin
			a <= i_pc_p1;
			b <= i_src3_broff_v;
			alu_op <= `CPU_ALUOP_ADD;
			j_mux <= 1'b1;
			o_j_addr_valid <= i_src1_gpr_v != i_src2_gpr_v;
			o_drop <= i_src1_gpr_v == i_src2_gpr_v;
			o_dst_gpr <= {(`CPU_REGNO_WIDTH){1'b0}};
		end
		else if(i_op == `CPU_OP_BLEZ)
		begin
			a <= i_pc_p1;
			b <= i_src3_broff_v;
			alu_op <= `CPU_ALUOP_ADD;
			j_mux <= 1'b1;
			o_j_addr_valid <= i_src1_gpr_v[31] || ~|i_src1_gpr_v;
			o_drop <= !i_src1_gpr_v[31] && |i_src1_gpr_v;
			o_dst_gpr <= {(`CPU_REGNO_WIDTH){1'b0}};
		end
		else if(i_op == `CPU_OP_BGTZ)
		begin
			a <= i_pc_p1;
			b <= i_src3_broff_v;
			alu_op <= `CPU_ALUOP_ADD;
			j_mux <= 1'b1;
			o_j_addr_valid <= !i_src1_gpr_v[31] && |i_src1_gpr_v;
			o_drop <= i_src1_gpr_v[31] || ~|i_src1_gpr_v;
			o_dst_gpr <= {(`CPU_REGNO_WIDTH){1'b0}};
		end
		else if(i_op == `CPU_OP_ADDI)
		begin
			o_dst_gpr <= i_regimm; //XXX:
			a <= i_src1_gpr_v;
			b <= i_src3_se_v;
			alu_op <= `CPU_ALUOP_ADD;
		end
		else if(i_op == `CPU_OP_ADDIU)
		begin
			o_dst_gpr <= i_regimm; //XXX:
			a <= i_src1_gpr_v;
			b <= i_src3_se_v;
			alu_op <= `CPU_ALUOP_ADD;
		end
		else if(i_op == `CPU_OP_SLTI)
		begin
			o_dst_gpr <= i_regimm; //XXX:
			a <= i_src1_gpr_v;
			b <= i_src3_se_v;
			alu_op <= `CPU_ALUOP_SLT;
		end
		else if(i_op == `CPU_OP_SLTIU)
		begin
			o_dst_gpr <= i_regimm; //XXX:
			a <= i_src1_gpr_v;
			b <= i_src3_se_v;
			alu_op <= `CPU_ALUOP_SLTU;
		end
		else if(i_op == `CPU_OP_ANDI)
		begin
			o_dst_gpr <= i_regimm; //XXX:
			a <= i_src1_gpr_v;
			b <= i_src3_ze_v;
			alu_op <= `CPU_ALUOP_AND;
		end
		else if(i_op == `CPU_OP_ORI)
		begin
			o_dst_gpr <= i_regimm; //XXX:
			a <= i_src1_gpr_v;
			b <= i_src3_ze_v;
			alu_op <= `CPU_ALUOP_OR;
		end
		else if(i_op == `CPU_OP_XORI)
		begin
			o_dst_gpr <= i_regimm; //XXX:
			a <= i_src1_gpr_v;
			b <= i_src3_ze_v;
			alu_op <= `CPU_ALUOP_XOR;
		end
		else if(i_op == `CPU_OP_LUI)
		begin
			o_dst_gpr <= i_regimm; //XXX:
			r_mux <= 1'b1;
			result <= { i_src3_ze_v[15:0], 16'b0 };
		end
		else if(i_op == `CPU_OP_LB)
		begin
			o_dst_gpr <= i_regimm; //XXX:
			a <= i_src1_gpr_v;
			b <= i_src3_se_v;
			alu_op <= `CPU_ALUOP_ADD;
		end
		else if(i_op == `CPU_OP_LH)
		begin
			o_dst_gpr <= i_regimm; //XXX:
			a <= i_src1_gpr_v;
			b <= i_src3_se_v;
			alu_op <= `CPU_ALUOP_ADD;
		end
		else if(i_op == `CPU_OP_LW)
		begin
			o_dst_gpr <= i_regimm; //XXX:
			a <= i_src1_gpr_v;
			b <= i_src3_se_v;
			alu_op <= `CPU_ALUOP_ADD;
		end
		else if(i_op == `CPU_OP_LBU)
		begin
			o_dst_gpr <= i_regimm; //XXX:
			a <= i_src1_gpr_v;
			b <= i_src3_se_v;
			alu_op <= `CPU_ALUOP_ADD;
		end
		else if(i_op == `CPU_OP_LHU)
		begin
			o_dst_gpr <= i_regimm; //XXX:
			a <= i_src1_gpr_v;
			b <= i_src3_se_v;
			alu_op <= `CPU_ALUOP_ADD;
		end
		else if(i_op == `CPU_OP_SB)
		begin
			o_dst_gpr <= 0; //XXX:
			o_mem_data <= i_src2_gpr_v;
			a <= i_src1_gpr_v;
			b <= i_src3_se_v;
			alu_op <= `CPU_ALUOP_ADD;
		end
		else if(i_op == `CPU_OP_SH)
		begin
			o_dst_gpr <= 0; //XXX:
			o_mem_data <= i_src2_gpr_v;
			a <= i_src1_gpr_v;
			b <= i_src3_se_v;
			alu_op <= `CPU_ALUOP_ADD;
		end
		else if(i_op == `CPU_OP_SW)
		begin
			o_dst_gpr <= 0; //XXX:
			o_mem_data <= i_src2_gpr_v;
			a <= i_src1_gpr_v;
			b <= i_src3_se_v;
			alu_op <= `CPU_ALUOP_ADD;
		end
	end
end


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
