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
 * Instruction fetch pipeline stage
 */

`include "cpu_common.vh"
`include "cpu_const.vh"


/* Fetch stage */
module fetch(
	clk,
	nrst,
	/* CU signals */
	i_pc,
	i_j_valid,
	o_instr,
	i_exec_stall,
	i_mem_stall,
	o_fetch_stall,
	/* IFU signals */
	o_addr,
	i_instr_dat,
	o_rd_cmd,
	i_busy,
	i_err_align,
	i_err_bus
);
localparam [`CPU_INSTR_WIDTH-1:0] NOP = 32'h0000_0000;
/* Inputs */
input wire				clk;
input wire				nrst;
/* CU signals */
input wire [`CPU_ADDR_WIDTH-1:0]	i_pc;
input wire				i_j_valid;
output wire [`CPU_INSTR_WIDTH-1:0]	o_instr;
input wire				i_exec_stall;
input wire				i_mem_stall;
output wire				o_fetch_stall;
output wire				o_fetch_except;
/* IFU interface */
output reg [`CPU_ADDR_WIDTH-1:0]	o_addr;
input wire [`CPU_INSTR_WIDTH-1:0]	i_instr_dat;
output reg				o_rd_cmd;
input wire				i_busy;
input wire				i_err_align;
input wire				i_err_bus;


wire core_stall;

assign core_stall = i_exec_stall || i_mem_stall || o_fetch_stall;
assign o_fetch_stall = i_busy;

reg nullify;
reg [`CPU_INSTR_WIDTH-1:0] instr;

assign o_instr = nullify ? NOP : instr;


always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		o_addr <= {(`CPU_ADDR_WIDTH){1'b0}};
		o_rd_cmd <= 1'b0;
	end
	else
	begin
		o_rd_cmd <= 1'b0;
		if(!core_stall)
		begin
			o_addr <= i_pc;
			o_rd_cmd <= 1'b1;
		end
	end
end


always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		instr <= NOP;
		nullify <= 1'b0;
	end
	else if(i_j_valid)
	begin
		nullify <= 1'b1;
	end
	else if(!core_stall)
	begin
		instr <= !nullify ? i_instr_dat : NOP;
		nullify <= 1'b0;
	end
end


endmodule /* fetch */
