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
 * System top level
 */

`include "common.vh"
`include "ocp_const.vh"


/*
 * System top
 */
module sys_top (
	clk,
	nrst
);
input wire clk;
input wire nrst;

/** Local interconnect **/

/* CPU I-Bus */
wire [`ADDR_WIDTH-1:0]	C_IAddr;
wire			C_IRdC;
wire [`DATA_WIDTH-1:0]	C_IData;
wire			C_IRdy;
wire			C_IErr;

/* CPU D-Bus */
wire [`ADDR_WIDTH-1:0]	C_DAddr;
wire			C_DCmd;
wire			C_DRnW;
wire [`BEN_WIDTH-1:0]	C_DBen;
wire [`DATA_WIDTH-1:0]	C_DDataM;
wire [`DATA_WIDTH-1:0]	C_DDataS;
wire			C_DRdy;
wire			C_DErr;

/* OCP I-Port */
wire [`ADDR_WIDTH-1:0]	I_MAddr;
wire [2:0]		I_MCmd;
wire [`DATA_WIDTH-1:0]	I_MData;
wire [`BEN_WIDTH-1:0]	I_MByteEn;
wire			I_SCmdAccept;
wire [`DATA_WIDTH-1:0]	I_SData;
wire [1:0]		I_SResp;

/* OCP D-Port */
wire [`ADDR_WIDTH-1:0]	D_MAddr;
wire [2:0]		D_MCmd;
wire [`DATA_WIDTH-1:0]	D_MData;
wire [`BEN_WIDTH-1:0]	D_MByteEn;
wire			D_SCmdAccept;
wire [`DATA_WIDTH-1:0]	D_SData;
wire [1:0]		D_SResp;

/* Slave ports */
wire [`ADDR_WIDTH-1:0]	P_MAddr[0:4];
wire [2:0]		P_MCmd[0:4];
wire [`DATA_WIDTH-1:0]	P_MData[0:4];
wire [`BEN_WIDTH-1:0]	P_MByteEn[0:4];
wire			P_SCmdAccept[0:4];
wire [`DATA_WIDTH-1:0]	P_SData[0:4];
wire [1:0]		P_SResp[0:4];

/* CPU interrupt */
wire intr;

/* Timer interrupt */
wire timer_intr;


/* IBus-to-OCP */
ibus2ocp ibus_ocp(
	.i_IAddr(C_IAddr),
	.i_IRdC(C_IRdC),
	.o_IData(C_IData),
	.o_IRdy(C_IRdy),
	.o_IErr(C_IErr),
	.o_MAddr(I_MAddr),
	.o_MCmd(I_MCmd),
	.o_MData(I_MData),
	.o_MByteEn(I_MByteEn),
	.i_SCmdAccept(I_SCmdAccept),
	.i_SData(I_SData),
	.i_SResp(I_SResp)
);


/* DBus-to-OCP */
dbus2ocp dbus_ocp(
	.i_DAddr(C_DAddr),
	.i_DCmd(C_DCmd),
	.i_DRnW(C_DRnW),
	.i_DBen(C_DBen),
	.i_DData(C_DDataM),
	.o_DData(C_DDataS),
	.o_DRdy(C_DRdy),
	.o_DErr(C_DErr),
	.o_MAddr(D_MAddr),
	.o_MCmd(D_MCmd),
	.o_MData(D_MData),
	.o_MByteEn(D_MByteEn),
	.i_SCmdAccept(D_SCmdAccept),
	.i_SData(D_SData),
	.i_SResp(D_SResp)
);


/* CPU */
uparc_cpu_top cpu(
	.clk(clk),
	.nrst(nrst),
	.i_intr(intr),
	.o_IAddr(C_IAddr),
	.o_IRdC(C_IRdC),
	.i_IData(C_IData),
	.i_IRdy(C_IRdy),
	.i_IErr(C_IErr),
	.o_DAddr(C_DAddr),
	.o_DCmd(C_DCmd),
	.o_DRnW(C_DRnW),
	.o_DBen(C_DBen),
	.o_DData(C_DDataM),
	.i_DData(C_DDataS),
	.i_DRdy(C_DRdy),
	.i_DErr(C_DErr)
);


/* Memory */
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


/* micro UART */
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


/* Control device */
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


/* Interrupt controller */
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


/* Interval timer */
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


/* Fabric */
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


endmodule /* sys_top */
