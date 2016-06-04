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

`include "common.vh"
`include "ocp_const.vh"


`ifndef TRACE_FILE
`define TRACE_FILE "trace.vcd"
`endif


module tb_interval_timer();
	localparam HCLK = 5;
	localparam PCLK = 2*HCLK;	/* Clock period */

	/* Timer Registers */
	localparam [`ADDR_WIDTH-1:0] CTRLREG = 32'h000;	/* Control register */
	localparam [`ADDR_WIDTH-1:0] CNTRREG = 32'h004;	/* Counter register */
	localparam [`ADDR_WIDTH-1:0] CURRREG = 32'h008;	/* Current counter */

	reg clk;
	reg nrst;
	reg [`ADDR_WIDTH-1:0]	MAddr;
	reg [2:0]		MCmd;
	reg [`DATA_WIDTH-1:0]	MData;
	reg [`BEN_WIDTH-1:0]	MByteEn;
	wire			SCmdAccept;
	wire [`DATA_WIDTH-1:0]	SData;
	wire [1:0]		SResp;
	wire			intr;

	always
		#HCLK clk = !clk;

	/* Issue bus read transaction */
	task bus_read;
	input [`ADDR_WIDTH-1:0] addr;
	begin
		@(posedge clk)
		begin
			MAddr <= addr;
			MByteEn <= 4'hf;
			MCmd <= `OCP_CMD_READ;
		end

		@(posedge clk)
		begin
			MAddr <= 0;
			MByteEn <= 4'h0;
			MCmd <= `OCP_CMD_IDLE;
		end
	end
	endtask

	/* Issue bus write transaction */
	task bus_write;
	input [`ADDR_WIDTH-1:0] addr;
	input [`DATA_WIDTH-1:0] data;
	begin
		@(posedge clk)
		begin
			MAddr <= addr;
			MData <= data;
			MByteEn <= 4'hf;
			MCmd <= `OCP_CMD_WRITE;
		end

		@(posedge clk)
		begin
			MAddr <= 0;
			MData <= 0;
			MByteEn <= 4'h0;
			MCmd <= `OCP_CMD_IDLE;
		end
	end
	endtask

	initial
	begin
		/* Set tracing */
		$dumpfile(`TRACE_FILE);
		$dumpvars(0, tb_interval_timer);

		clk = 1;
		nrst = 0;
		MAddr = 0;
		MData = 0;
		MByteEn = 0;
		MCmd = 0;
		#(10*PCLK) nrst = 1;

		#(2*PCLK)

		/* Set counter value */
		bus_write(CNTRREG, 32'h10);

		/* Start: enable = 1, reload = 1, imask = 1 */
		bus_write(CTRLREG, 32'h7);

		/* Read control register */
		bus_read(CTRLREG);

		/* Read counter register */
		bus_read(CNTRREG);

		/* Read current count (1) */
		bus_read(CURRREG);

		/* Read current count (2) */
		bus_read(CURRREG);

		#(40*PCLK)

		/* Update counter value */
		bus_write(CNTRREG, 32'h4);

		#(20*PCLK)

		/* Start: enable = 1, reload = 0, imask = 0 */
		bus_write(CTRLREG, 32'h1);

		#500 $finish;
	end


	/* Instantiate timer */
	interval_timer timer(
		.clk(clk),
		.nrst(nrst),
		.i_MAddr(MAddr),
		.i_MCmd(MCmd),
		.i_MData(MData),
		.i_MByteEn(MByteEn),
		.o_SCmdAccept(SCmdAccept),
		.o_SData(SData),
		.o_SResp(SResp),
		.o_intr(intr)
	);

endmodule /* tb_interval_timer */
