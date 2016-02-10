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
 * Testbench for programmable interrupt controller
 */

`include "common.vh"
`include "ocp_const.vh"


`ifndef TRACE_FILE
`define TRACE_FILE "trace.vcd"
`endif

module tb_intr_controller();
	localparam HCLK = 5;
	localparam PCLK = 2*HCLK;	/* Clock period */

	/* Interrupt controller registers */
	localparam [`ADDR_WIDTH-1:0] ISTATREG = 32'h000;	/* Interrupts status register */
	localparam [`ADDR_WIDTH-1:0] IMASKREG = 32'h004;	/* Interrupts mask register */
	localparam [`ADDR_WIDTH-1:0] IRAWREG  = 32'h008;	/* Raw interrupts register */

	reg clk;
	reg nrst;
	reg [`ADDR_WIDTH-1:0]	MAddr;
	reg [2:0]		MCmd;
	reg [`DATA_WIDTH-1:0]	MData;
	reg [`BEN_WIDTH-1:0]	MByteEn;
	wire			SCmdAccept;
	wire [`DATA_WIDTH-1:0]	SData;
	wire [1:0]		SResp;
	reg [31:0]		intr_vec;
	wire			intr;

	always
		#HCLK clk = !clk;

	initial
	begin
		/* Set tracing */
		$dumpfile(`TRACE_FILE);
		$dumpvars(0, tb_intr_controller);

		clk = 1;
		nrst = 0;
		MAddr = 0;
		MData = 0;
		MByteEn = 0;
		MCmd = 0;
		intr_vec = 0;
		#(10*PCLK) nrst = 1;

		#(2*PCLK)

		@(posedge clk)
		begin
			/* Unmask line 0 */
			MAddr = IMASKREG;
			MData = 32'h1;
			MByteEn = 4'hf;
			MCmd = `OCP_CMD_WRITE;
		end

		@(posedge clk)
		begin
			MCmd = `OCP_CMD_IDLE;
		end

		#(2*PCLK)

		/* Generate interrupt on line 0 */
		@(posedge clk)
		begin
			intr_vec[0] = 1;
		end
		@(posedge clk)
		begin
			intr_vec[0] = 0;
		end

		#(2*PCLK)

		@(posedge clk)
		begin
			/* Acknowledge */
			MAddr = ISTATREG;
			MData = 32'h1;
			MByteEn = 4'hf;
			MCmd = `OCP_CMD_WRITE;
		end

		@(posedge clk)
		begin
			MCmd = `OCP_CMD_IDLE;
		end

		#(2*PCLK)

		/* Generate interrupt on line 1 */
		@(posedge clk)
		begin
			intr_vec[1] = 1;
		end
		@(posedge clk)
		begin
			intr_vec[1] = 0;
		end

		#(2*PCLK)

		@(posedge clk)
		begin
			/* Read raw status */
			MAddr = IRAWREG;
			MByteEn = 4'hf;
			MCmd = `OCP_CMD_READ;
		end

		@(posedge clk)
		begin
			MCmd = `OCP_CMD_IDLE;
		end

		#500 $finish;
	end


	/* Instantiate interrupt controller */
	intr_controller intr_ctrl(
		.clk(clk),
		.nrst(nrst),
		.i_MAddr(MAddr),
		.i_MCmd(MCmd),
		.i_MData(MData),
		.i_MByteEn(MByteEn),
		.o_SCmdAccept(SCmdAccept),
		.o_SData(SData),
		.o_SResp(SResp),
		.o_intr(intr),
		.i_intr_vec(intr_vec)
	);

endmodule /* tb_intr_controller */
