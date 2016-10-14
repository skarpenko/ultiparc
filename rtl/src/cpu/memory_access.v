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
 * Memory access pipeline stage
 */

`include "cpu_common.vh"
`include "cpu_const.vh"


/* Memory access stage */
module memory_access(
	clk,
	nrst,
	/* CU signals */
	i_exec_stall,
	o_mem_stall,
	i_fetch_stall,
	/* LSU interface */
	lsu_addr,
	lsu_wdata,
	lsu_rdata,
	lsu_cmd,
	lsu_rnw,
	lsu_busy,
	lsu_err_align,
	lsu_err_bus,
	/* Result of execute stage */
	i_op,
	i_dst_gpr,
	i_result,
	i_mem_data,
	/* Data for writeback */
	o_dst_gpr,
	o_dst_gpr_v
);
localparam [4:0] LSU_OTHER	= 5'b00000;
localparam [4:0] LSU_LD		= 5'b00001;
localparam [4:0] LSU_LD_BZE	= 5'b00010;
localparam [4:0] LSU_LD_HZE	= 5'b00100;
localparam [4:0] LSU_LD_BSE	= 5'b01000;
localparam [4:0] LSU_LD_HSE	= 5'b10000;
/* Inputs */
input wire				clk;
input wire				nrst;
/* CU signals */
input wire				i_exec_stall;
output wire				o_mem_stall;
input wire				i_fetch_stall;
/* LSU interface */
output reg [`CPU_ADDR_WIDTH-1:0]	lsu_addr;
output reg [`CPU_DATA_WIDTH-1:0]	lsu_wdata;
input wire [`CPU_DATA_WIDTH-1:0]	lsu_rdata;
output reg [1:0]			lsu_cmd;
output reg				lsu_rnw;
input wire				lsu_busy;
input wire				lsu_err_align;
input wire				lsu_err_bus;
/* Input from execute stage */
input wire [5:0]			i_op;
input wire [`CPU_REGNO_WIDTH-1:0]	i_dst_gpr;
input wire [`CPU_DATA_WIDTH-1:0]	i_result;
input wire [`CPU_DATA_WIDTH-1:0]	i_mem_data;
/* Output for writeback */
output reg [`CPU_REGNO_WIDTH-1:0]	o_dst_gpr;
output reg [`CPU_DATA_WIDTH-1:0]	o_dst_gpr_v;



wire core_stall;
assign core_stall = i_exec_stall || o_mem_stall || i_fetch_stall;

assign o_mem_stall = lsu_busy;

reg [4:0] lsu_op;
reg [`CPU_DATA_WIDTH-1:0] result;


always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		o_dst_gpr <= 0;
		lsu_op <= LSU_OTHER;

		lsu_addr <= 0;
		lsu_wdata <= 0;
		lsu_cmd <= 0;
		lsu_rnw <= 0;
		result <= 0;
	end
	else
	begin
		lsu_cmd <= `CPU_LSU_IDLE;
//		lsu_op <= LSU_OTHER;
		if(!core_stall)
		begin
//			lsu_cmd <= `CPU_LSU_IDLE;
			lsu_op <= LSU_OTHER;
			case(i_op)
			`CPU_OP_LB: begin
				lsu_addr <= i_result;
				lsu_cmd <= `CPU_LSU_BYTE;
				lsu_rnw <= 1'b1;
				lsu_op <= LSU_LD_BSE;
				o_dst_gpr <= i_dst_gpr;
			end
			`CPU_OP_LH: begin
				lsu_addr <= i_result;
				lsu_cmd <= `CPU_LSU_HWORD;
				lsu_rnw <= 1'b1;
				lsu_op <= LSU_LD_HSE;
				o_dst_gpr <= i_dst_gpr;
			end
			`CPU_OP_LW: begin
				lsu_addr <= i_result;
				lsu_cmd <= `CPU_LSU_WORD;
				lsu_rnw <= 1'b1;
				lsu_op <= LSU_LD;
				o_dst_gpr <= i_dst_gpr;
			end
			`CPU_OP_LBU: begin
				lsu_addr <= i_result;
				lsu_cmd <= `CPU_LSU_BYTE;
				lsu_rnw <= 1'b1;
				lsu_op <= LSU_LD_BZE;
				o_dst_gpr <= i_dst_gpr;
			end
			`CPU_OP_LHU: begin
				lsu_addr <= i_result;
				lsu_cmd <= `CPU_LSU_HWORD;
				lsu_rnw <= 1'b1;
				lsu_op <= LSU_LD_HZE;
				o_dst_gpr <= i_dst_gpr;
			end
			`CPU_OP_SB: begin
				lsu_addr <= i_result;
				lsu_wdata <= i_mem_data;
				lsu_cmd <= `CPU_LSU_BYTE;
				lsu_rnw <= 1'b0;
				o_dst_gpr <= 0; //XXX: ?
			end
			`CPU_OP_SH: begin
				lsu_addr <= i_result;
				lsu_wdata <= i_mem_data;
				lsu_cmd <= `CPU_LSU_HWORD;
				lsu_rnw <= 1'b0;
				o_dst_gpr <= 0; //XXX: ?
			end
			`CPU_OP_SW: begin
				lsu_addr <= i_result;
				lsu_wdata <= i_mem_data;
				lsu_cmd <= `CPU_LSU_WORD;
				lsu_rnw <= 1'b0;
				o_dst_gpr <= 0; //XXX: ?
			end
			default: begin
				o_dst_gpr <= i_dst_gpr;
				result <= i_result;
			end
			endcase
		end
	end
end


always @(*)
begin
	if(lsu_op)
	begin
		case(lsu_op)
		LSU_LD_BZE: o_dst_gpr_v = { 24'b0, lsu_rdata[7:0] };
		LSU_LD_HZE: o_dst_gpr_v = { 16'b0, lsu_rdata[15:0] };
		LSU_LD_BSE: o_dst_gpr_v = { {24{lsu_rdata[7]}}, lsu_rdata[7:0] };
		LSU_LD_HSE: o_dst_gpr_v = { {16{lsu_rdata[15]}}, lsu_rdata[15:0] };
		default: o_dst_gpr_v = lsu_rdata;
		endcase
	end
	else
		o_dst_gpr_v = result;
end


endmodule /* memory_access */
