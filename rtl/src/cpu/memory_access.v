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
	o_bus_error,
	o_addr_error,
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
	i_rd_no,
	i_alu_result,
	i_lsu_op,
	i_lsu_lns,
	i_lsu_ext,
	i_mem_data,
	/* Data for writeback */
	o_rd_no,
	o_rd_val
);
`include "reg_names.vh"
/* Destination result */
localparam [3:0] MUX_RD_ALU		= 4'b0000;	/* ALU result */
localparam [3:0] MUX_RD_BYTE_SE		= 4'b1000;	/* Sign-extended byte */
localparam [3:0] MUX_RD_BYTE_ZE		= 4'b1001;	/* Zero-extended byte */
localparam [3:0] MUX_RD_HWORD_SE	= 4'b1010;	/* Sign-extended halfword */
localparam [3:0] MUX_RD_HWORD_ZE	= 4'b1011;	/* Zero-extended halfword */
localparam [3:0] MUX_RD_WORD		= 4'b1100;	/* Word */
/* Inputs */
input wire				clk;
input wire				nrst;
/* CU signals */
input wire				i_exec_stall;
output wire				o_mem_stall;
input wire				i_fetch_stall;
output wire				o_bus_error;
output wire				o_addr_error;
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
input wire [`CPU_REGNO_WIDTH-1:0]	i_rd_no;
input wire [`CPU_REG_WIDTH-1:0]		i_alu_result;
input wire [`CPU_LSUOP_WIDTH-1:0]	i_lsu_op;
input wire				i_lsu_lns;
input wire				i_lsu_ext;
input wire [`CPU_DATA_WIDTH-1:0]	i_mem_data;
/* Output for writeback */
output reg [`CPU_REGNO_WIDTH-1:0]	o_rd_no;
output reg [`CPU_REG_WIDTH-1:0]		o_rd_val;



wire core_stall = i_exec_stall || o_mem_stall || i_fetch_stall;

assign o_mem_stall = lsu_busy;

assign o_bus_error = lsu_err_bus;
assign o_addr_error = lsu_err_align;


reg [3:0] lsu_mux;	/* Destination result MUX */

reg [`CPU_REG_WIDTH-1:0] alu_result;


/* LSU operation */
always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		lsu_addr <= {(`CPU_ADDR_WIDTH){1'b0}};
		lsu_wdata <= {(`CPU_DATA_WIDTH){1'b0}};
		lsu_cmd <= `CPU_LSU_IDLE;
		lsu_rnw <= 1'b0;
		lsu_mux <= MUX_RD_ALU;
		alu_result <= {(`CPU_REG_WIDTH){1'b0}};
	end
	else
	begin
		lsu_cmd <= `CPU_LSU_IDLE;
		if(!core_stall)
		begin
			o_rd_no <= i_rd_no;

			alu_result <= i_alu_result;

			lsu_addr <= i_alu_result;
			lsu_wdata <= i_mem_data;
			lsu_cmd <= i_lsu_op;
			lsu_rnw <= i_lsu_lns;

			if(i_lsu_op == `CPU_LSU_BYTE && i_lsu_ext == 1'b1)
				lsu_mux <= MUX_RD_BYTE_SE;
			else if(i_lsu_op == `CPU_LSU_BYTE && i_lsu_ext == 1'b0)
				lsu_mux <= MUX_RD_BYTE_ZE;
			else if(i_lsu_op == `CPU_LSU_HWORD && i_lsu_ext == 1'b1)
				lsu_mux <= MUX_RD_HWORD_SE;
			else if(i_lsu_op == `CPU_LSU_HWORD && i_lsu_ext == 1'b0)
				lsu_mux <= MUX_RD_HWORD_ZE;
			else if(i_lsu_op == `CPU_LSU_WORD)
				lsu_mux <= MUX_RD_WORD;
			else
				lsu_mux <= MUX_RD_ALU;
		end
	end
end


/* Set outputs */
always @(*)
begin
	case(lsu_mux)
	MUX_RD_BYTE_SE: o_rd_val = { {24{lsu_rdata[7]}}, lsu_rdata[7:0] };
	MUX_RD_BYTE_ZE: o_rd_val = { 24'b0, lsu_rdata[7:0] };
	MUX_RD_HWORD_SE: o_rd_val = { {16{lsu_rdata[15]}}, lsu_rdata[15:0] };
	MUX_RD_HWORD_ZE: o_rd_val = { 16'b0, lsu_rdata[15:0] };
	MUX_RD_WORD: o_rd_val = lsu_rdata;
	default: o_rd_val = alu_result;
	endcase
end

endmodule /* memory_access */
