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
 * Testbench for fabric
 */

`include "common.vh"
`include "ocp_const.vh"


`ifndef TRACE_FILE
`define TRACE_FILE "trace.vcd"
`endif


module tb_fabric();
	localparam HCLK = 5;
	localparam PCLK = 2*HCLK;	/* Clock period */

	/* Micro UART register offsets */
	localparam [`ADDR_WIDTH-1:0] UART_CHARREG = 32'h8000_0000;	/* Character register */

	/* Control device registers */
	localparam [`ADDR_WIDTH-1:0] SIM_CTRLREG = 32'h8010_0000;	/* Control register */

	/* Interrupt controller registers */
	localparam [`ADDR_WIDTH-1:0] IC_ISTATREG = 32'h8020_0000;	/* Interrupts status register */
	localparam [`ADDR_WIDTH-1:0] IC_IMASKREG = 32'h8020_0004;	/* Interrupts mask register */
	localparam [`ADDR_WIDTH-1:0] IC_IRAWREG  = 32'h8020_0008;	/* Raw interrupts register */

	/* Timer Registers */
	localparam [`ADDR_WIDTH-1:0] TMR_CTRLREG = 32'h8030_0000;	/* Control register */
	localparam [`ADDR_WIDTH-1:0] TMR_CNTRREG = 32'h8030_0004;	/* Counter register */
	localparam [`ADDR_WIDTH-1:0] TMR_CURRREG = 32'h8030_0008;	/* Current counter */

	reg clk;
	reg nrst;
	reg [`ADDR_WIDTH-1:0]	I_MAddr;
	reg [2:0]		I_MCmd;
	reg [`DATA_WIDTH-1:0]	I_MData;
	reg [`BEN_WIDTH-1:0]	I_MByteEn;
	wire			I_SCmdAccept;
	wire [`DATA_WIDTH-1:0]	I_SData;
	wire [1:0]		I_SResp;

	reg [`ADDR_WIDTH-1:0]	D_MAddr;
	reg [2:0]		D_MCmd;
	reg [`DATA_WIDTH-1:0]	D_MData;
	reg [`BEN_WIDTH-1:0]	D_MByteEn;
	wire			D_SCmdAccept;
	wire [`DATA_WIDTH-1:0]	D_SData;
	wire [1:0]		D_SResp;

	wire [`ADDR_WIDTH-1:0]	P_MAddr[0:4];
	wire [2:0]		P_MCmd[0:4];
	wire [`DATA_WIDTH-1:0]	P_MData[0:4];
	wire [`BEN_WIDTH-1:0]	P_MByteEn[0:4];
	wire			P_SCmdAccept[0:4];
	wire [`DATA_WIDTH-1:0]	P_SData[0:4];
	wire [1:0]		P_SResp[0:4];

	wire intr;
	wire timer_intr;

	always
		#HCLK clk = !clk;

	/* Issue instruction port read */
	task i_bus_read;
	input [`ADDR_WIDTH-1:0] addr;
	begin
		@(posedge clk)
		begin
			I_MAddr <= addr;
			I_MByteEn <= 4'hf;
			I_MCmd <= `OCP_CMD_READ;
		end

		@(posedge clk)
		begin
			I_MAddr <= 0;
			I_MByteEn <= 4'h0;
			I_MCmd <= `OCP_CMD_IDLE;
		end
	end
	endtask

	/* Issue instruction port write */
	task i_bus_write;
	input [`ADDR_WIDTH-1:0] addr;
	input [`DATA_WIDTH-1:0] data;
	input [`BEN_WIDTH-1:0] ben;
	begin
		@(posedge clk)
		begin
			I_MAddr <= addr;
			I_MData <= data;
			I_MByteEn <= ben;
			I_MCmd <= `OCP_CMD_WRITE;
		end

		@(posedge clk)
		begin
			I_MAddr <= 0;
			I_MData <= 0;
			I_MByteEn <= 4'h0;
			I_MCmd <= `OCP_CMD_IDLE;
		end
	end
	endtask

	/* Issue data port read */
	task d_bus_read;
	input [`ADDR_WIDTH-1:0] addr;
	begin
		@(posedge clk)
		begin
			D_MAddr <= addr;
			D_MByteEn <= 4'hf;
			D_MCmd <= `OCP_CMD_READ;
		end

		@(posedge clk)
		begin
			D_MAddr <= 0;
			D_MByteEn <= 4'h0;
			D_MCmd <= `OCP_CMD_IDLE;
		end
	end
	endtask

	/* Issue data port write */
	task d_bus_write;
	input [`ADDR_WIDTH-1:0] addr;
	input [`DATA_WIDTH-1:0] data;
	input [`BEN_WIDTH-1:0] ben;
	begin
		@(posedge clk)
		begin
			D_MAddr <= addr;
			D_MData <= data;
			D_MByteEn <= ben;
			D_MCmd <= `OCP_CMD_WRITE;
		end

		@(posedge clk)
		begin
			D_MAddr <= 0;
			D_MData <= 0;
			D_MByteEn <= 4'h0;
			D_MCmd <= `OCP_CMD_IDLE;
		end
	end
	endtask

	/* Issue instructions and data port read */
	task id_bus_read;
	input [`ADDR_WIDTH-1:0] i_addr;
	input [`ADDR_WIDTH-1:0] d_addr;
	begin
		@(posedge clk)
		begin
			I_MAddr <= i_addr;
			I_MByteEn <= 4'hf;
			I_MCmd <= `OCP_CMD_READ;
			D_MAddr <= d_addr;
			D_MByteEn <= 4'hf;
			D_MCmd <= `OCP_CMD_READ;
		end

		@(posedge clk)
		begin
			I_MAddr <= 0;
			I_MByteEn <= 4'h0;
			I_MCmd <= `OCP_CMD_IDLE;
			D_MAddr <= 0;
			D_MByteEn <= 4'h0;
			D_MCmd <= `OCP_CMD_IDLE;
		end
	end
	endtask

	/* Issue instructions and data port write */
	task id_bus_write;
	input [`ADDR_WIDTH-1:0] i_addr;
	input [`DATA_WIDTH-1:0] i_data;
	input [`BEN_WIDTH-1:0] i_ben;
	input [`ADDR_WIDTH-1:0] d_addr;
	input [`DATA_WIDTH-1:0] d_data;
	input [`BEN_WIDTH-1:0] d_ben;
	begin
		@(posedge clk)
		begin
			I_MAddr <= i_addr;
			I_MData <= i_data;
			I_MByteEn <= i_ben;
			I_MCmd <= `OCP_CMD_WRITE;
			D_MAddr <= d_addr;
			D_MData <= d_data;
			D_MByteEn <= d_ben;
			D_MCmd <= `OCP_CMD_WRITE;
		end

		@(posedge clk)
		begin
			I_MAddr <= 0;
			I_MData <= 0;
			I_MByteEn <= 4'h0;
			I_MCmd <= `OCP_CMD_IDLE;
			D_MAddr <= 0;
			D_MData <= 0;
			D_MByteEn <= 4'h0;
			D_MCmd <= `OCP_CMD_IDLE;
		end
	end
	endtask

	task wait_pos_clk;
		@(posedge clk);
	endtask


	initial
	begin
		/* Set tracing */
		$dumpfile(`TRACE_FILE);
		$dumpvars(0, tb_fabric);

		clk = 1;
		nrst = 0;
		I_MAddr = 0;
		I_MData = 0;
		I_MByteEn = 0;
		I_MCmd = 0;
		D_MAddr = 0;
		D_MData = 0;
		D_MByteEn = 0;
		D_MCmd = 0;
		#(10*PCLK) nrst = 1;

		#(2*PCLK)

		/* Overlapped access conflict */
		i_bus_read(32'h0000_0000);
		d_bus_write(32'h0000_0000, 32'hdead_beef, 4'hf);
		i_bus_read(32'h0000_0000);
		d_bus_write(32'h0000_0004, 32'hcafe_babe, 4'hf);

		wait_pos_clk();
		wait_pos_clk();
		wait_pos_clk();
		wait_pos_clk();

		/* Simultaneous access conflict */
		id_bus_read(32'h0000_0000, 32'h0000_0004);

		wait_pos_clk();
		wait_pos_clk();
		wait_pos_clk();
		wait_pos_clk();
		wait_pos_clk();

		/* No conflict access. (Memory + UART) */
		id_bus_write(32'h0000_0008, 32'h5a5a_5a5a, 4'hf, UART_CHARREG, "!", 4'hf);

		wait_pos_clk();
		wait_pos_clk();

		/* Overlapped access to different ports */
		i_bus_read(IC_IRAWREG);
		d_bus_write(UART_CHARREG, "\n", 4'hf);

		wait_pos_clk();
		wait_pos_clk();

		/* Set timer counter value */
		d_bus_write(TMR_CNTRREG, 32'h10, 4'hf);

		wait_pos_clk();

		/* Start timer and unmask interrupt line 0 */
		id_bus_write(TMR_CTRLREG, 32'h7, 4'hf, IC_IMASKREG, 32'h1, 4'hf);

		#(40*PCLK)

		/* Read memory and interrupt controller status */
		id_bus_read(32'h0000_0008, IC_ISTATREG);

		wait_pos_clk();

		/* Acknowledge interrupt */
		d_bus_write(IC_ISTATREG, 32'h1, 4'hf);

		#(40*PCLK)

		/* Overlapped conflict (2) */
		@(posedge clk)
		begin
			I_MAddr <= TMR_CNTRREG;
			I_MByteEn <= 4'hf;
			I_MCmd <= `OCP_CMD_READ;
		end
		@(posedge clk)
		begin
			I_MAddr <= 0;
			I_MByteEn <= 4'h0;
			I_MCmd <= `OCP_CMD_IDLE;
			D_MAddr <= TMR_CNTRREG;
			D_MByteEn <= 4'hf;
			D_MCmd <= `OCP_CMD_READ;
		end
		@(posedge clk)
		begin
			D_MAddr <= 0;
			D_MByteEn <= 4'h0;
			D_MCmd <= `OCP_CMD_IDLE;
		end

		wait_pos_clk();
		wait_pos_clk();

		/* Overlapped conflict (3) */
		@(posedge clk)
		begin
			D_MAddr <= TMR_CNTRREG;
			D_MByteEn <= 4'hf;
			D_MCmd <= `OCP_CMD_READ;
		end
		@(posedge clk)
		begin
			D_MAddr <= 0;
			D_MByteEn <= 4'h0;
			D_MCmd <= `OCP_CMD_IDLE;
			I_MAddr <= TMR_CNTRREG;
			I_MByteEn <= 4'hf;
			I_MCmd <= `OCP_CMD_READ;
		end
		@(posedge clk)
		begin
			I_MAddr <= 0;
			I_MByteEn <= 4'h0;
			I_MCmd <= `OCP_CMD_IDLE;
		end

		wait_pos_clk();
		wait_pos_clk();


		/* Overlapped no conflict */
		@(posedge clk)
		begin
			D_MAddr <= TMR_CNTRREG;
			D_MByteEn <= 4'hf;
			D_MCmd <= `OCP_CMD_READ;
		end
		@(posedge clk)
		begin
			D_MAddr <= 0;
			D_MByteEn <= 4'h0;
			D_MCmd <= `OCP_CMD_IDLE;
			I_MAddr <= IC_IRAWREG;
			I_MByteEn <= 4'hf;
			I_MCmd <= `OCP_CMD_READ;
		end
		@(posedge clk)
		begin
			I_MAddr <= 0;
			I_MByteEn <= 4'h0;
			I_MCmd <= `OCP_CMD_IDLE;
		end

		wait_pos_clk();
		wait_pos_clk();

		/* Stop simulation */
		i_bus_write(SIM_CTRLREG, 32'h1, 4'hf);


		#500 $finish;
	end


	/* Instantiate memory */
	memory mem(
		.clk(clk),
		.nrst(nrst),
		.i_MAddr(P_MAddr[0]),
		.i_MCmd(P_MCmd[0]),
		.i_MData(P_MData[0]),
		.i_MByteEn(P_MByteEn[0]),
		.o_SCmdAccept(P_SCmdAccept[0]),
		.o_SData(P_SData[0]),
		.o_SResp(P_SResp[0])
	);


	/* Instantiate micro UART */
	micro_uart uart(
		.clk(clk),
		.nrst(nrst),
		.i_MAddr(P_MAddr[1]),
		.i_MCmd(P_MCmd[1]),
		.i_MData(P_MData[1]),
		.i_MByteEn(P_MByteEn[1]),
		.o_SCmdAccept(P_SCmdAccept[1]),
		.o_SData(P_SData[1]),
		.o_SResp(P_SResp[1])
	);


	/* Instantiate control device */
	sim_control sim_ctl(
		.clk(clk),
		.nrst(nrst),
		.i_MAddr(P_MAddr[2]),
		.i_MCmd(P_MCmd[2]),
		.i_MData(P_MData[2]),
		.i_MByteEn(P_MByteEn[2]),
		.o_SCmdAccept(P_SCmdAccept[2]),
		.o_SData(P_SData[2]),
		.o_SResp(P_SResp[2])
	);


	/* Instantiate interrupt controller */
	intr_controller intr_ctrl(
		.clk(clk),
		.nrst(nrst),
		.i_MAddr(P_MAddr[3]),
		.i_MCmd(P_MCmd[3]),
		.i_MData(P_MData[3]),
		.i_MByteEn(P_MByteEn[3]),
		.o_SCmdAccept(P_SCmdAccept[3]),
		.o_SData(P_SData[3]),
		.o_SResp(P_SResp[3]),
		.o_intr(intr),
		.i_intr_vec({31'b0, timer_intr})
	);


	/* Instantiate timer */
	interval_timer timer(
		.clk(clk),
		.nrst(nrst),
		.i_MAddr(P_MAddr[4]),
		.i_MCmd(P_MCmd[4]),
		.i_MData(P_MData[4]),
		.i_MByteEn(P_MByteEn[4]),
		.o_SCmdAccept(P_SCmdAccept[4]),
		.o_SData(P_SData[4]),
		.o_SResp(P_SResp[4]),
		.o_intr(timer_intr)
	);


	/* Instantiate fabric */
	fabric fab(
		.clk(clk),
		.nrst(nrst),
		/* OCP interface: instructions (master) */
		.i_I_MAddr(I_MAddr), .i_I_MCmd(I_MCmd),
		.i_I_MData(I_MData), .i_I_MByteEn(I_MByteEn),
		.o_I_SCmdAccept(I_SCmdAccept), .o_I_SData(I_SData),
		.o_I_SResp(I_SResp),
		/* OCP interface: data (master) */
		.i_D_MAddr(D_MAddr), .i_D_MCmd(D_MCmd),
		.i_D_MData(D_MData), .i_D_MByteEn(D_MByteEn),
		.o_D_SCmdAccept(D_SCmdAccept), .o_D_SData(D_SData),
		.o_D_SResp(D_SResp),
		/* OCP interface: Port 0 (slave) */
		.o_P0_MAddr(P_MAddr[0]), .o_P0_MCmd(P_MCmd[0]),
		.o_P0_MData(P_MData[0]), .o_P0_MByteEn(P_MByteEn[0]),
		.i_P0_SCmdAccept(P_SCmdAccept[0]), .i_P0_SData(P_SData[0]),
		.i_P0_SResp(P_SResp[0]),
		/* OCP interface: Port 1 (slave) */
		.o_P1_MAddr(P_MAddr[1]), .o_P1_MCmd(P_MCmd[1]),
		.o_P1_MData(P_MData[1]), .o_P1_MByteEn(P_MByteEn[1]),
		.i_P1_SCmdAccept(P_SCmdAccept[1]), .i_P1_SData(P_SData[1]),
		.i_P1_SResp(P_SResp[1]),
		/* OCP interface: Port 2 (slave) */
		.o_P2_MAddr(P_MAddr[2]), .o_P2_MCmd(P_MCmd[2]),
		.o_P2_MData(P_MData[2]), .o_P2_MByteEn(P_MByteEn[2]),
		.i_P2_SCmdAccept(P_SCmdAccept[2]), .i_P2_SData(P_SData[2]),
		.i_P2_SResp(P_SResp[2]),
		/* OCP interface: Port 3 (slave) */
		.o_P3_MAddr(P_MAddr[3]), .o_P3_MCmd(P_MCmd[3]),
		.o_P3_MData(P_MData[3]), .o_P3_MByteEn(P_MByteEn[3]),
		.i_P3_SCmdAccept(P_SCmdAccept[3]), .i_P3_SData(P_SData[3]),
		.i_P3_SResp(P_SResp[3]),
		/* OCP interface: Port 4 (slave) */
		.o_P4_MAddr(P_MAddr[4]), .o_P4_MCmd(P_MCmd[4]),
		.o_P4_MData(P_MData[4]), .o_P4_MByteEn(P_MByteEn[4]),
		.i_P4_SCmdAccept(P_SCmdAccept[4]), .i_P4_SData(P_SData[4]),
		.i_P4_SResp(P_SResp[4])
	);


endmodule /* tb_fabric */
