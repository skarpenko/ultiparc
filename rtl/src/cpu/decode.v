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
	i_instr,
	i_exec_stall,
	i_mem_stall,
	i_fetch_stall,
	i_drop,
	/* Decoded instr */
	o_op,
	o_dst_gpr,
	o_src1_gpr,
	o_src2_gpr,
	o_src3_se_v,
	o_src3_ze_v,
	o_src3_sh16_v,
	o_src3_j_v,
	o_src3_broff_v,
	o_shamt,
	o_func
);
/* Inputs */
input wire				clk;
input wire				nrst;
/* CU signals */
input wire [`CPU_ADDR_WIDTH-1:0]	i_pc;
input wire [`CPU_INSTR_WIDTH-1:0]	i_instr;
input wire				i_exec_stall;
input wire				i_mem_stall;
input wire				i_fetch_stall;
input wire				i_drop;
/* Decoded instr */
output wire [5:0] o_op;
output wire [`CPU_REGNO_WIDTH-1:0]	o_dst_gpr;
output wire [`CPU_REGNO_WIDTH-1:0]	o_src1_gpr;
output wire [`CPU_REGNO_WIDTH-1:0]	o_src2_gpr;
output wire [`CPU_DATA_WIDTH-1:0]	o_src3_se_v;
output wire [`CPU_DATA_WIDTH-1:0]	o_src3_ze_v;
output wire [`CPU_DATA_WIDTH-1:0]	o_src3_sh16_v;
output wire [`CPU_ADDR_WIDTH-1:0]	o_src3_j_v;
output wire [`CPU_ADDR_WIDTH-1:0]	o_src3_broff_v;
output wire [4:0]			o_shamt;
output wire [5:0]			o_func;


wire core_stall;
assign core_stall = i_exec_stall || i_mem_stall || i_fetch_stall;


reg [`CPU_INSTR_WIDTH-1:0] instr;
reg [3:0] pc_high;

always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		instr <= 32'b0;
		pc_high <= 4'b0;
	end
	else if(i_drop)
	begin
		instr <= 32'b0;
		pc_high <= 4'b0;
	end
	if(!core_stall)
	begin
		instr <= i_instr;
		pc_high <= i_pc[31:28];
	end
end


assign o_op		= instr[31:26];	/* opcode */
assign o_dst_gpr	= instr[15:11];	/* rd */
assign o_src1_gpr	= instr[25:21];	/* rs */
assign o_src2_gpr	= instr[20:16];	/* rt */
assign o_shamt		= instr[10:6];	/* shift amount */
assign o_func		= instr[5:0];	/* function */


assign o_src3_se_v	= { {16{instr[15]}}, instr[15:0] };		/* Sign extended immediate */
assign o_src3_ze_v	= { 16'b0, instr[15:0] };			/* Zero extended immediate */
assign o_src3_sh16_v	= { instr[15:0], 16'b0 };			/* Shifted left immediate */
assign o_src3_j_v	= { pc_high, instr[25:0], 2'b0 };		/* Jump target address */
assign o_src3_broff_v	= { {14{instr[15]}}, instr[15:0], 2'b0 };	/* Branch offset */


endmodule /* decode */
