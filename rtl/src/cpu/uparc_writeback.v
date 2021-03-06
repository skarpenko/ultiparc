/*
 * Copyright (c) 2015-2018 The Ultiparc Project. All rights reserved.
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
 * Writeback pipeline stage
 */

`include "uparc_cpu_config.vh"
`include "uparc_cpu_common.vh"
`include "uparc_cpu_const.vh"


/* Writeback stage */
module uparc_writeback(
	clk,
	nrst,
	/* CU signals */
	i_exec_stall,
	i_mem_stall,
	i_fetch_stall,
	i_wait_stall,
	i_nullify,
	/* Data for writeback */
	i_rd_no,
	i_rd_val,
	o_rd_no,
	o_rd_val
);
/* Inputs */
input wire				clk;
input wire				nrst;
/* CU signals */
input wire				i_exec_stall;
input wire				i_mem_stall;
input wire				i_fetch_stall;
input wire				i_wait_stall;
input wire				i_nullify;
/* Input from memory access stage */
input wire [`UPARC_REGNO_WIDTH-1:0]	i_rd_no;
input wire [`UPARC_REG_WIDTH-1:0]	i_rd_val;
/* Output for write to register file */
output reg [`UPARC_REGNO_WIDTH-1:0]	o_rd_no;
output reg [`UPARC_REG_WIDTH-1:0]	o_rd_val;


wire core_stall = i_exec_stall || i_mem_stall || i_fetch_stall || i_wait_stall;


always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		o_rd_no <= {(`UPARC_REGNO_WIDTH){1'b0}};
		o_rd_val <= {(`UPARC_REG_WIDTH){1'b0}};
	end
	else if(!core_stall)
	begin
		o_rd_no <= !i_nullify ? i_rd_no : {(`UPARC_REGNO_WIDTH){1'b0}};
		o_rd_val <= i_rd_val;
	end
end


endmodule /* uparc_writeback */
