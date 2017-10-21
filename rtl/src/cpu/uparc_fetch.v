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
 * Instruction fetch pipeline stage
 */

`include "uparc_cpu_common.vh"
`include "uparc_cpu_const.vh"


/* Fetch stage */
module uparc_fetch(
	clk,
	nrst,
	/* CU signals */
	i_pc,
	i_jump_addr,
	i_jump_valid,
	i_except_valid,
	i_except_haddr,
	i_exec_stall,
	i_mem_stall,
	o_fetch_stall,
	o_bus_error,
	o_addr_error,
	i_nullify,
	/* IFU signals */
	o_addr,
	i_instr_dat,
	o_rd_cmd,
	i_busy,
	i_err_align,
	i_err_bus,
	/* Fetched instruction */
	o_instr,
	o_pc
);
localparam [`UPARC_INSTR_WIDTH-1:0] NOP = 32'h0000_0000;
/* Inputs */
input wire				clk;
input wire				nrst;
/* CU signals */
input wire [`UPARC_ADDR_WIDTH-1:0]	i_pc;
input wire [`UPARC_ADDR_WIDTH-1:0]	i_jump_addr;
input wire				i_jump_valid;
input wire				i_except_valid;
input wire [`UPARC_ADDR_WIDTH-1:0]	i_except_haddr;
input wire				i_exec_stall;
input wire				i_mem_stall;
output wire				o_fetch_stall;
output wire				o_bus_error;
output wire				o_addr_error;
input wire				i_nullify;
/* IFU interface */
output reg [`UPARC_ADDR_WIDTH-1:0]	o_addr;
input wire [`UPARC_INSTR_WIDTH-1:0]	i_instr_dat;
output reg				o_rd_cmd;
input wire				i_busy;
input wire				i_err_align;
input wire				i_err_bus;
/* Fetched instruction */
output wire [`UPARC_INSTR_WIDTH-1:0]	o_instr;
output reg [`UPARC_ADDR_WIDTH-1:0]	o_pc;


wire core_stall = i_exec_stall || i_mem_stall || o_fetch_stall;


assign o_fetch_stall = i_busy;
assign o_bus_error = i_err_bus | err_bus_r;
assign o_addr_error = i_err_align | err_align_r;
assign o_instr = (!i_jump_valid && !i_nullify && !i_except_valid ? i_instr_dat : NOP);


/** Local wires and registers **/
reg				err_bus_r;
reg				err_align_r;
wire [`UPARC_ADDR_WIDTH-1:0]	new_pc = !i_except_valid ?
					(!i_jump_valid ? i_pc : i_jump_addr) : i_except_haddr;


/* IFU operation */
always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		o_addr <= {(`UPARC_ADDR_WIDTH){1'b0}};
		o_pc <= {(`UPARC_ADDR_WIDTH){1'b0}};
		o_rd_cmd <= 1'b0;
		err_align_r <= 1'b0;
		err_bus_r <= 1'b0;
	end
	else
	begin
		o_rd_cmd <= 1'b0;
		err_bus_r <= err_bus_r | i_err_bus;
		err_align_r <= err_align_r | i_err_align;
		if(!core_stall)
		begin
			err_align_r <= 1'b0;
			err_bus_r <= 1'b0;
			o_addr <= new_pc;
			o_pc <= new_pc;
			o_rd_cmd <= !i_nullify ? 1'b1 : 1'b0;
		end
	end
end


endmodule /* uparc_fetch */
