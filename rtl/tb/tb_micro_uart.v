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
 * Testbench for micro UART
 */

`include "common.vh"
`include "ocp_const.vh"


`ifndef TRACE_FILE
`define TRACE_FILE "trace.vcd"
`endif


module tb_micro_uart();
	localparam HCLK = 5;
	localparam PCLK = 2*HCLK;	/* Clock period */

	/* Micro UART register offsets */
	localparam [`ADDR_WIDTH-1:0] CHARREG = 32'h000;	/* Character register */

	reg clk;
	reg nrst;
	reg [`ADDR_WIDTH-1:0]	MAddr;
	reg [2:0]		MCmd;
	reg [`DATA_WIDTH-1:0]	MData;
	reg [`BEN_WIDTH-1:0]	MByteEn;
	wire			SCmdAccept;
	wire [`DATA_WIDTH-1:0]	SData;
	wire [1:0]		SResp;

	always
		#HCLK clk = !clk;

	initial
	begin
		/* Set tracing */
		$dumpfile(`TRACE_FILE);
		$dumpvars(0, tb_micro_uart);

		clk = 1;
		nrst = 0;
		MAddr = 0;
		MData = 0;
		MByteEn = 0;
		MCmd = 0;
		#(10*PCLK) nrst = 1;

		#(2*PCLK)
		@(posedge clk)
		begin
			/* Read char register */
			MAddr = CHARREG;
			MByteEn = 4'hf;
			MCmd = `OCP_CMD_READ;
		end

		@(posedge clk)
		begin
			MCmd = `OCP_CMD_IDLE;
		end

		@(posedge clk)
		begin
			/* Write char register */
			MAddr = CHARREG;
			MData = "H";
			MByteEn = 4'hf;
			MCmd = `OCP_CMD_WRITE;
		end

		@(posedge clk)
		begin
			MCmd = `OCP_CMD_IDLE;
		end

		@(posedge clk)
		begin
			/* Write char register */
			MAddr = CHARREG;
			MData = "e";
			MByteEn = 4'hf;
			MCmd = `OCP_CMD_WRITE;
		end

		@(posedge clk)
		begin
			MCmd = `OCP_CMD_IDLE;
		end

		@(posedge clk)
		begin
			/* Write char register */
			MAddr = CHARREG;
			MData = "l";
			MByteEn = 4'hf;
			MCmd = `OCP_CMD_WRITE;
		end

		@(posedge clk)
		begin
			MCmd = `OCP_CMD_IDLE;
		end

		@(posedge clk)
		begin
			/* Write char register */
			MAddr = CHARREG;
			MData = "l";
			MByteEn = 4'hf;
			MCmd = `OCP_CMD_WRITE;
		end

		@(posedge clk)
		begin
			MCmd = `OCP_CMD_IDLE;
		end

		@(posedge clk)
		begin
			/* Write char register */
			MAddr = CHARREG;
			MData = "o";
			MByteEn = 4'hf;
			MCmd = `OCP_CMD_WRITE;
		end

		@(posedge clk)
		begin
			MCmd = `OCP_CMD_IDLE;
		end

		@(posedge clk)
		begin
			/* Write char register */
			MAddr = CHARREG;
			MData = "\n";
			MByteEn = 4'hf;
			MCmd = `OCP_CMD_WRITE;
		end

		@(posedge clk)
		begin
			MCmd = `OCP_CMD_IDLE;
		end


		#500 $finish;
	end


	/* Instantiate micro UART */
	micro_uart uart(
		.clk(clk),
		.nrst(nrst),
		.i_MAddr(MAddr),
		.i_MCmd(MCmd),
		.i_MData(MData),
		.i_MByteEn(MByteEn),
		.o_SCmdAccept(SCmdAccept),
		.o_SData(SData),
		.o_SResp(SResp)
	);

endmodule /* tb_micro_uart */
