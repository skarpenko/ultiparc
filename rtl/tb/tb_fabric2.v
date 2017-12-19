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
 * Testbench for fabric (version 2)
 */

`include "common.vh"
`include "ocp_const.vh"


`ifndef TRACE_FILE
`define TRACE_FILE "trace.vcd"
`endif


module tb_fabric2();
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

	reg [`ADDR_WIDTH-1:0]	I_DAddr;
	reg			I_DCmd;
	reg			I_DRnW;
	reg [`BEN_WIDTH-1:0]	I_DBen;
	reg [`DATA_WIDTH-1:0]	I_MDData;
	wire [`DATA_WIDTH-1:0]	I_SDData;
	wire			I_DRdy;
	wire			I_DErr;

	reg [`ADDR_WIDTH-1:0]	D_DAddr;
	reg			D_DCmd;
	reg			D_DRnW;
	reg [`BEN_WIDTH-1:0]	D_DBen;
	reg [`DATA_WIDTH-1:0]	D_MDData;
	wire [`DATA_WIDTH-1:0]	D_SDData;
	wire			D_DRdy;
	wire			D_DErr;

	wire [`ADDR_WIDTH-1:0]	I_MAddr;
	wire [2:0]		I_MCmd;
	wire [`DATA_WIDTH-1:0]	I_MData;
	wire [`BEN_WIDTH-1:0]	I_MByteEn;
	wire			I_SCmdAccept;
	wire [`DATA_WIDTH-1:0]	I_SData;
	wire [1:0]		I_SResp;

	wire [`ADDR_WIDTH-1:0]	D_MAddr;
	wire [2:0]		D_MCmd;
	wire [`DATA_WIDTH-1:0]	D_MData;
	wire [`BEN_WIDTH-1:0]	D_MByteEn;
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
			I_DAddr <= addr;
			I_DCmd <= 1'b1;
			I_DRnW <= 1'b1;
			I_DBen <= 4'hf;
		end

		@(posedge clk)
		begin
			I_DAddr <= 'd0;
			I_DCmd <= 1'b0;
			I_DRnW <= 1'b0;
			I_DBen <= 4'h0;
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
			I_DAddr <= addr;
			I_DCmd <= 1'b1;
			I_DRnW <= 1'b0;
			I_DBen <= ben;
			I_MDData <= data;
		end

		@(posedge clk)
		begin
			I_DAddr <= 'd0;
			I_DCmd <= 1'b0;
			I_DRnW <= 1'b0;
			I_DBen <= 4'h0;
			I_MDData <= 'd0;
		end
	end
	endtask

	/* Issue data port read */
	task d_bus_read;
	input [`ADDR_WIDTH-1:0] addr;
	begin
		@(posedge clk)
		begin
			D_DAddr <= addr;
			D_DCmd <= 1'b1;
			D_DRnW <= 1'b1;
			D_DBen <= 4'hf;
		end

		@(posedge clk)
		begin
			D_DAddr <= 'd0;
			D_DCmd <= 1'b0;
			D_DRnW <= 1'b0;
			D_DBen <= 4'h0;
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
			D_DAddr <= addr;
			D_DCmd <= 1'b1;
			D_DRnW <= 1'b0;
			D_DBen <= ben;
			D_MDData <= data;
		end

		@(posedge clk)
		begin
			D_DAddr <= 'd0;
			D_DCmd <= 1'b0;
			D_DRnW <= 1'b0;
			D_DBen <= 4'h0;
			D_MDData <= 'd0;
		end
	end
	endtask

	/* Issue instructions and data port read/write */
	task id_bus_rdwr;
	input i_rnw;
	input [`ADDR_WIDTH-1:0] i_addr;
	input [`DATA_WIDTH-1:0] i_data;
	input [`BEN_WIDTH-1:0] i_ben;
	input d_rnw;
	input [`ADDR_WIDTH-1:0] d_addr;
	input [`DATA_WIDTH-1:0] d_data;
	input [`BEN_WIDTH-1:0] d_ben;
	begin
		@(posedge clk)
		begin
			I_DAddr <= i_addr;
			I_DCmd <= 1'b1;
			I_DRnW <= i_rnw;
			I_DBen <= i_ben;
			I_MDData <= i_data;
			D_DAddr <= d_addr;
			D_DCmd <= 1'b1;
			D_DRnW <= d_rnw;
			D_DBen <= d_ben;
			D_MDData <= d_data;
		end

		@(posedge clk)
		begin
			I_DAddr <= 'd0;
			I_DCmd <= 1'b0;
			I_DRnW <= 1'b0;
			I_DBen <= 4'h0;
			I_MDData <= 'd0;
			D_DAddr <= 'd0;
			D_DCmd <= 1'b0;
			D_DRnW <= 1'b0;
			D_DBen <= 4'h0;
			D_MDData <= 'd0;
		end
	end
	endtask

	/* Issue instructions and data port read/write (data 1 clock delay) */
	task id_bus_rdwr_d1;
	input i_rnw;
	input [`ADDR_WIDTH-1:0] i_addr;
	input [`DATA_WIDTH-1:0] i_data;
	input [`BEN_WIDTH-1:0] i_ben;
	input d_rnw;
	input [`ADDR_WIDTH-1:0] d_addr;
	input [`DATA_WIDTH-1:0] d_data;
	input [`BEN_WIDTH-1:0] d_ben;
	begin
		@(posedge clk)
		begin
			I_DAddr <= i_addr;
			I_DCmd <= 1'b1;
			I_DRnW <= i_rnw;
			I_DBen <= i_ben;
			I_MDData <= i_data;
		end

		@(posedge clk)
		begin
			I_DAddr <= 'd0;
			I_DCmd <= 1'b0;
			I_DRnW <= 1'b0;
			I_DBen <= 4'h0;
			I_MDData <= 'd0;
			D_DAddr <= d_addr;
			D_DCmd <= 1'b1;
			D_DRnW <= d_rnw;
			D_DBen <= d_ben;
			D_MDData <= d_data;
		end

		@(posedge clk)
		begin
			D_DAddr <= 'd0;
			D_DCmd <= 1'b0;
			D_DRnW <= 1'b0;
			D_DBen <= 4'h0;
			D_MDData <= 'd0;
		end
	end
	endtask

	/* Issue instructions and data port read/write (instructions 1 clock delay) */
	task id_bus_rdwr_i1;
	input i_rnw;
	input [`ADDR_WIDTH-1:0] i_addr;
	input [`DATA_WIDTH-1:0] i_data;
	input [`BEN_WIDTH-1:0] i_ben;
	input d_rnw;
	input [`ADDR_WIDTH-1:0] d_addr;
	input [`DATA_WIDTH-1:0] d_data;
	input [`BEN_WIDTH-1:0] d_ben;
	begin
		@(posedge clk)
		begin
			D_DAddr <= d_addr;
			D_DCmd <= 1'b1;
			D_DRnW <= d_rnw;
			D_DBen <= d_ben;
			D_MDData <= d_data;
		end

		@(posedge clk)
		begin
			I_DAddr <= i_addr;
			I_DCmd <= 1'b1;
			I_DRnW <= i_rnw;
			I_DBen <= i_ben;
			I_MDData <= i_data;
			D_DAddr <= 'd0;
			D_DCmd <= 1'b0;
			D_DRnW <= 1'b0;
			D_DBen <= 4'h0;
			D_MDData <= 'd0;
		end

		@(posedge clk)
		begin
			I_DAddr <= 'd0;
			I_DCmd <= 1'b0;
			I_DRnW <= 1'b0;
			I_DBen <= 4'h0;
			I_MDData <= 'd0;
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
		$dumpvars(0, tb_fabric2);

		clk = 1;
		nrst = 0;

		I_DAddr = 'd0;
		I_DCmd = 'd0;
		I_DRnW = 'd0;
		I_DBen = 'd0;
		I_MDData = 'd0;
		D_DAddr = 'd0;
		D_DCmd = 'd0;
		D_DRnW = 'd0;
		D_DBen = 'd0;
		D_MDData = 'd0;

		#(10*PCLK) nrst = 1;

		#(2*PCLK)


		/* Non overlapped access */
		i_bus_write(32'h0000_0004, 32'hf1f2_f3f4, 4'hf);

//		wait_pos_clk();
//		wait_pos_clk();

		d_bus_write(32'h0000_0008, 32'hf5f6_f7f8, 4'hf);

		wait_pos_clk();
		wait_pos_clk();

		/* Overlapped access (conflict) */
		id_bus_rdwr_d1(1'b0, 32'h0000_0004, 32'hf1f2_f3f4, 4'hf,
			1'b0, 32'h0000_0008, 32'hf5f6_f7f8, 4'hf);

		wait_pos_clk();
		wait_pos_clk();

		id_bus_rdwr_i1(1'b0, 32'h0000_0004, 32'hf1f2_f3f4, 4'hf,
			1'b0, 32'h0000_0008, 32'hf5f6_f7f8, 4'hf);

		wait_pos_clk();
		wait_pos_clk();

		/* Simultaneous access (conflict) */
		id_bus_rdwr(1'b0, UART_CHARREG, "!", 4'hf,
			1'b0, UART_CHARREG, "\n", 4'hf);

		wait_pos_clk();
		wait_pos_clk();

		/* Simultaneous access */
		id_bus_rdwr(1'b1, IC_IMASKREG, 32'h0, 4'hf,
			1'b1, TMR_CURRREG, 32'h0, 4'hf);

		wait_pos_clk();
		wait_pos_clk();

		/* Stop simulation */
		d_bus_write(SIM_CTRLREG, 32'h1, 4'hf);


		#500 $finish;
	end


	/* CPU DBus-to-OCP (instructions) */
	dbus2ocp2 i_dbus2ocp(
		.clk(clk),
		.nrst(nrst),
		/* CPU port */
		.i_DAddr(I_DAddr),
		.i_DCmd(I_DCmd),
		.i_DRnW(I_DRnW),
		.i_DBen(I_DBen),
		.i_DData(I_MDData),
		.o_DData(I_SDData),
		.o_DRdy(I_DRdy),
		.o_DErr(I_DErr),
		/* OCP port */
		.o_MAddr(I_MAddr),
		.o_MCmd(I_MCmd),
		.o_MData(I_MData),
		.o_MByteEn(I_MByteEn),
		.i_SCmdAccept(I_SCmdAccept),
		.i_SData(I_SData),
		.i_SResp(I_SResp)
	);


	/* CPU DBus-to-OCP (data) */
	dbus2ocp2 d_dbus2ocp(
		.clk(clk),
		.nrst(nrst),
		/* CPU port */
		.i_DAddr(D_DAddr),
		.i_DCmd(D_DCmd),
		.i_DRnW(D_DRnW),
		.i_DBen(D_DBen),
		.i_DData(D_MDData),
		.o_DData(D_SDData),
		.o_DRdy(D_DRdy),
		.o_DErr(D_DErr),
		/* OCP port */
		.o_MAddr(D_MAddr),
		.o_MCmd(D_MCmd),
		.o_MData(D_MData),
		.o_MByteEn(D_MByteEn),
		.i_SCmdAccept(D_SCmdAccept),
		.i_SData(D_SData),
		.i_SResp(D_SResp)
	);


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
	fabric2 fab(
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


endmodule /* tb_fabric2 */
