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
 * Testbench for programmable interval timer
 */

`timescale 1us/100ns

`include "common.vh"
`include "ocp_const.vh"


module tb_interval_timer();
	localparam HCLK = 5;
	localparam PCLK = 2*HCLK;	/* Clock period */

	/* Timer Register */
	localparam CTRLREG = 32'h000;	/* Control register */
	localparam CNTRREG = 32'h004;	/* Counter register */
	localparam CURRREG = 32'h008;	/* Current counter */

	reg clk;
	reg nrst;
	reg [`ADDR_WIDTH-1:0]	i_addr;
	reg [2:0]		i_cmd;
	reg [`DATA_WIDTH-1:0]	i_data;
	reg [`BEN_WIDTH-1:0]	i_ben;
	wire			o_cmd_acc;
	wire [`DATA_WIDTH-1:0]	o_data;
	wire [1:0]		o_resp;
	wire			o_intr;

	always
		#HCLK clk = !clk;

	initial
	begin
		/* Set tracing */
		$dumpfile("trace.vcd");
		$dumpvars(0, tb_interval_timer);

		clk = 1;
		nrst = 0;
		i_addr = 0;
		i_data = 0;
		i_ben = 0;
		i_cmd = 0;
		#(10*PCLK) nrst = 1;

		#(2*PCLK)
		@(posedge clk)
		begin
			/* Set counter value */
			i_addr = CNTRREG;
			i_data = 32'h10;
			i_ben = 4'hf;
			i_cmd = `OCP_CMD_WRITE;
		end

		@(posedge clk)
		begin
			i_cmd = `OCP_CMD_IDLE;
		end

		@(posedge clk)
		begin
			/* Start: enable = 1, reload = 1, imask = 1 */
			i_addr = CTRLREG;
			i_data = 32'h7;
			i_ben = 4'hf;
			i_cmd = `OCP_CMD_WRITE;
		end

		@(posedge clk)
		begin
			i_cmd = `OCP_CMD_IDLE;
		end

		@(posedge clk)
		begin
			/* Read control register */
			i_addr = CTRLREG;
			i_ben = 4'hf;
			i_cmd = `OCP_CMD_READ;
		end

		@(posedge clk)
		begin
			i_cmd = `OCP_CMD_IDLE;
		end

		@(posedge clk)
		begin
			/* Read counter register */
			i_addr = CNTRREG;
			i_ben = 4'hf;
			i_cmd = `OCP_CMD_READ;
		end

		@(posedge clk)
		begin
			i_cmd = `OCP_CMD_IDLE;
		end

		@(posedge clk)
		begin
			/* Read current count (1) */
			i_addr = CURRREG;
			i_ben = 4'hf;
			i_cmd = `OCP_CMD_READ;
		end

		@(posedge clk)
		begin
			i_cmd = `OCP_CMD_IDLE;
		end

		@(posedge clk)
		begin
			/* Read current count (2) */
			i_addr = CURRREG;
			i_ben = 4'hf;
			i_cmd = `OCP_CMD_READ;
		end

		@(posedge clk)
		begin
			i_cmd = `OCP_CMD_IDLE;
		end

		#(40*PCLK)
		@(posedge clk)
		begin
			/* Update counter value */
			i_addr = CNTRREG;
			i_data = 32'h4;
			i_ben = 4'hf;
			i_cmd = `OCP_CMD_WRITE;
		end

		@(posedge clk)
		begin
			i_cmd = `OCP_CMD_IDLE;
		end

		#(20*PCLK)

		@(posedge clk)
		begin
			/* Start: enable = 1, reload = 0, imask = 0 */
			i_addr = CTRLREG;
			i_data = 32'h1;
			i_ben = 4'hf;
			i_cmd = `OCP_CMD_WRITE;
		end

		@(posedge clk)
		begin
			i_cmd = `OCP_CMD_IDLE;
		end

		#500 $finish;
	end


	/* Instantiate timer */
	interval_timer timer(
		.clk(clk),
		.nrst(nrst),
		.i_MAddr(i_addr),
		.i_MCmd(i_cmd),
		.i_MData(i_data),
		.i_MByteEn(i_ben),
		.o_SCmdAccept(o_cmd_acc),
		.o_SData(o_data),
		.o_SResp(o_resp),
		.o_intr(o_intr)
	);

endmodule /* tb_interval_timer */
