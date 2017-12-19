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
 * System fabric (version 2)
 */

`include "common.vh"
`include "ocp_const.vh"


/* Fabric top level */
module fabric2(
	clk,
	nrst,
	/* OCP interface: instructions (master) */
	i_I_MAddr, i_I_MCmd, i_I_MData, i_I_MByteEn, o_I_SCmdAccept, o_I_SData, o_I_SResp,
	/* OCP interface: data (master) */
	i_D_MAddr, i_D_MCmd, i_D_MData, i_D_MByteEn, o_D_SCmdAccept, o_D_SData, o_D_SResp,
	/* OCP interface: Port 0 (slave) */
	o_P0_MAddr, o_P0_MCmd, o_P0_MData, o_P0_MByteEn, i_P0_SCmdAccept, i_P0_SData, i_P0_SResp,
	/* OCP interface: Port 1 (slave) */
	o_P1_MAddr, o_P1_MCmd, o_P1_MData, o_P1_MByteEn, i_P1_SCmdAccept, i_P1_SData, i_P1_SResp,
	/* OCP interface: Port 2 (slave) */
	o_P2_MAddr, o_P2_MCmd, o_P2_MData, o_P2_MByteEn, i_P2_SCmdAccept, i_P2_SData, i_P2_SResp,
	/* OCP interface: Port 3 (slave) */
	o_P3_MAddr, o_P3_MCmd, o_P3_MData, o_P3_MByteEn, i_P3_SCmdAccept, i_P3_SData, i_P3_SResp,
	/* OCP interface: Port 4 (slave) */
	o_P4_MAddr, o_P4_MCmd, o_P4_MData, o_P4_MByteEn, i_P4_SCmdAccept, i_P4_SData, i_P4_SResp
);
localparam PORTNO_WIDTH = 11;
localparam NPORTS = 5;

input wire			clk;
input wire			nrst;
/* OCP interface: instructions (master) */
input wire [`ADDR_WIDTH-1:0]	i_I_MAddr;
input wire [2:0]		i_I_MCmd;
input wire [`DATA_WIDTH-1:0]	i_I_MData;
input wire [`BEN_WIDTH-1:0]	i_I_MByteEn;
output wire			o_I_SCmdAccept;
output wire [`DATA_WIDTH-1:0]	o_I_SData;
output wire [1:0]		o_I_SResp;
/* OCP interface: data (master) */
input wire [`ADDR_WIDTH-1:0]	i_D_MAddr;
input wire [2:0]		i_D_MCmd;
input wire [`DATA_WIDTH-1:0]	i_D_MData;
input wire [`BEN_WIDTH-1:0]	i_D_MByteEn;
output wire			o_D_SCmdAccept;
output wire [`DATA_WIDTH-1:0]	o_D_SData;
output wire [1:0]		o_D_SResp;
/* OCP interface: Port 0 (slave) */
output wire [`ADDR_WIDTH-1:0]	o_P0_MAddr;
output wire [2:0]		o_P0_MCmd;
output wire [`DATA_WIDTH-1:0]	o_P0_MData;
output wire [`BEN_WIDTH-1:0]	o_P0_MByteEn;
input wire			i_P0_SCmdAccept;
input wire [`DATA_WIDTH-1:0]	i_P0_SData;
input wire [1:0]		i_P0_SResp;
/* OCP interface: Port 1 (slave) */
output wire [`ADDR_WIDTH-1:0]	o_P1_MAddr;
output wire [2:0]		o_P1_MCmd;
output wire [`DATA_WIDTH-1:0]	o_P1_MData;
output wire [`BEN_WIDTH-1:0]	o_P1_MByteEn;
input wire			i_P1_SCmdAccept;
input wire [`DATA_WIDTH-1:0]	i_P1_SData;
input wire [1:0]		i_P1_SResp;
/* OCP interface: Port 2 (slave) */
output wire [`ADDR_WIDTH-1:0]	o_P2_MAddr;
output wire [2:0]		o_P2_MCmd;
output wire [`DATA_WIDTH-1:0]	o_P2_MData;
output wire [`BEN_WIDTH-1:0]	o_P2_MByteEn;
input wire			i_P2_SCmdAccept;
input wire [`DATA_WIDTH-1:0]	i_P2_SData;
input wire [1:0]		i_P2_SResp;
/* OCP interface: Port 3 (slave) */
output wire [`ADDR_WIDTH-1:0]	o_P3_MAddr;
output wire [2:0]		o_P3_MCmd;
output wire [`DATA_WIDTH-1:0]	o_P3_MData;
output wire [`BEN_WIDTH-1:0]	o_P3_MByteEn;
input wire			i_P3_SCmdAccept;
input wire [`DATA_WIDTH-1:0]	i_P3_SData;
input wire [1:0]		i_P3_SResp;
/* OCP interface: Port 4 (slave) */
output wire [`ADDR_WIDTH-1:0]	o_P4_MAddr;
output wire [2:0]		o_P4_MCmd;
output wire [`DATA_WIDTH-1:0]	o_P4_MData;
output wire [`BEN_WIDTH-1:0]	o_P4_MByteEn;
input wire			i_P4_SCmdAccept;
input wire [`DATA_WIDTH-1:0]	i_P4_SData;
input wire [1:0]		i_P4_SResp;


/** Internal interconnect **/

wire [`ADDR_WIDTH-1:0]	P_MAddr[0:NPORTS-1];
wire [2:0]		P_MCmd[0:NPORTS-1];
wire [`DATA_WIDTH-1:0]	P_MData[0:NPORTS-1];
wire [`BEN_WIDTH-1:0]	P_MByteEn[0:NPORTS-1];
wire			P_SCmdAccept[0:NPORTS-1];
wire [`DATA_WIDTH-1:0]	P_SData[0:NPORTS-1];
wire [1:0]		P_SResp[0:NPORTS-1];

/* Port 0 (slave) */
assign o_P0_MAddr = P_MAddr[0];
assign o_P0_MCmd = P_MCmd[0];
assign o_P0_MData = P_MData[0];
assign o_P0_MByteEn = P_MByteEn[0];
assign P_SCmdAccept[0] = i_P0_SCmdAccept;
assign P_SData[0] = i_P0_SData;
assign P_SResp[0] = i_P0_SResp;
/* Port 1 (slave) */
assign o_P1_MAddr = P_MAddr[1];
assign o_P1_MCmd = P_MCmd[1];
assign o_P1_MData = P_MData[1];
assign o_P1_MByteEn = P_MByteEn[1];
assign P_SCmdAccept[1] = i_P1_SCmdAccept;
assign P_SData[1] = i_P1_SData;
assign P_SResp[1] = i_P1_SResp;
/* Port 2 (slave) */
assign o_P2_MAddr = P_MAddr[2];
assign o_P2_MCmd = P_MCmd[2];
assign o_P2_MData = P_MData[2];
assign o_P2_MByteEn = P_MByteEn[2];
assign P_SCmdAccept[2] = i_P2_SCmdAccept;
assign P_SData[2] = i_P2_SData;
assign P_SResp[2] = i_P2_SResp;
/* Port 3 (slave) */
assign o_P3_MAddr = P_MAddr[3];
assign o_P3_MCmd = P_MCmd[3];
assign o_P3_MData = P_MData[3];
assign o_P3_MByteEn = P_MByteEn[3];
assign P_SCmdAccept[3] = i_P3_SCmdAccept;
assign P_SData[3] = i_P3_SData;
assign P_SResp[3] = i_P3_SResp;
/* Port 4 (slave) */
assign o_P4_MAddr = P_MAddr[4];
assign o_P4_MCmd = P_MCmd[4];
assign o_P4_MData = P_MData[4];
assign o_P4_MByteEn = P_MByteEn[4];
assign P_SCmdAccept[4] = i_P4_SCmdAccept;
assign P_SData[4] = i_P4_SData;
assign P_SResp[4] = i_P4_SResp;

/* Instructions master port destinations */
wire [`ADDR_WIDTH-1:0]	instr_P_MAddr[0:NPORTS-1];
wire [2:0]		instr_P_MCmd[0:NPORTS-1];
wire [`DATA_WIDTH-1:0]	instr_P_MData[0:NPORTS-1];
wire [`BEN_WIDTH-1:0]	instr_P_MByteEn[0:NPORTS-1];
wire			instr_P_SCmdAccept[0:NPORTS-1];
wire [`DATA_WIDTH-1:0]	instr_P_SData[0:NPORTS-1];
wire [1:0]		instr_P_SResp[0:NPORTS-1];

/* Data master port destinations */
wire [`ADDR_WIDTH-1:0]	data_P_MAddr[0:NPORTS-1];
wire [2:0]		data_P_MCmd[0:NPORTS-1];
wire [`DATA_WIDTH-1:0]	data_P_MData[0:NPORTS-1];
wire [`BEN_WIDTH-1:0]	data_P_MByteEn[0:NPORTS-1];
wire			data_P_SCmdAccept[0:NPORTS-1];
wire [`DATA_WIDTH-1:0]	data_P_SData[0:NPORTS-1];
wire [1:0]		data_P_SResp[0:NPORTS-1];

/* Decoded addresses and port numbers */
wire [`ADDR_WIDTH-1:0]	instr_decoded_addr;
wire [PORTNO_WIDTH-1:0]	instr_portno;
wire [`ADDR_WIDTH-1:0]	data_decoded_addr;
wire [PORTNO_WIDTH-1:0]	data_portno;

/* Master port switches control */
wire [PORTNO_WIDTH-1:0]	i_mswitch;
wire [PORTNO_WIDTH-1:0]	d_mswitch;
/* Slave port switches control */
wire			p_sswitch[0:NPORTS-1];

/* Active and completed transactions */
wire instr_act = (i_I_MCmd != `OCP_CMD_IDLE ? 1'b1 : 1'b0);
wire instr_done = (o_I_SResp != `OCP_RESP_NULL ? 1'b1 : 1'b0);
wire data_act = (i_D_MCmd != `OCP_CMD_IDLE ? 1'b1 : 1'b0);
wire data_done = (o_D_SResp != `OCP_RESP_NULL ? 1'b1 : 1'b0);


/* Instructions master port address decoder */
fabric2_decoder #(.PORTNO_WIDTH(PORTNO_WIDTH)) instr_addr_decoder(
	.i_addr(i_I_MAddr),
	.o_addr(instr_decoded_addr),
	.o_portno(instr_portno)
);

/* Data master port address decoder */
fabric2_decoder #(.PORTNO_WIDTH(PORTNO_WIDTH)) data_addr_decoder(
	.i_addr(i_D_MAddr),
	.o_addr(data_decoded_addr),
	.o_portno(data_portno)
);


/* Instructions master port switch */
fabric2_mswitch #(.PORTNO_WIDTH(PORTNO_WIDTH)) instr_mswitch(
	.i_portno(i_mswitch),
	/* OCP interface: instructions/data (master) */
	.i_ID_MAddr(instr_decoded_addr),
	.i_ID_MCmd(i_I_MCmd),
	.i_ID_MData(i_I_MData),
	.i_ID_MByteEn(i_I_MByteEn),
	.o_ID_SCmdAccept(o_I_SCmdAccept),
	.o_ID_SData(o_I_SData),
	.o_ID_SResp(o_I_SResp),
	/* OCP interface: Port 0 (slave) */
	.o_P0_MAddr(instr_P_MAddr[0]),
	.o_P0_MCmd(instr_P_MCmd[0]),
	.o_P0_MData(instr_P_MData[0]),
	.o_P0_MByteEn(instr_P_MByteEn[0]),
	.i_P0_SCmdAccept(instr_P_SCmdAccept[0]),
	.i_P0_SData(instr_P_SData[0]),
	.i_P0_SResp(instr_P_SResp[0]),
	/* OCP interface: Port 1 (slave) */
	.o_P1_MAddr(instr_P_MAddr[1]),
	.o_P1_MCmd(instr_P_MCmd[1]),
	.o_P1_MData(instr_P_MData[1]),
	.o_P1_MByteEn(instr_P_MByteEn[1]),
	.i_P1_SCmdAccept(instr_P_SCmdAccept[1]),
	.i_P1_SData(instr_P_SData[1]),
	.i_P1_SResp(instr_P_SResp[1]),
	/* OCP interface: Port 2 (slave) */
	.o_P2_MAddr(instr_P_MAddr[2]),
	.o_P2_MCmd(instr_P_MCmd[2]),
	.o_P2_MData(instr_P_MData[2]),
	.o_P2_MByteEn(instr_P_MByteEn[2]),
	.i_P2_SCmdAccept(instr_P_SCmdAccept[2]),
	.i_P2_SData(instr_P_SData[2]),
	.i_P2_SResp(instr_P_SResp[2]),
	/* OCP interface: Port 3 (slave) */
	.o_P3_MAddr(instr_P_MAddr[3]),
	.o_P3_MCmd(instr_P_MCmd[3]),
	.o_P3_MData(instr_P_MData[3]),
	.o_P3_MByteEn(instr_P_MByteEn[3]),
	.i_P3_SCmdAccept(instr_P_SCmdAccept[3]),
	.i_P3_SData(instr_P_SData[3]),
	.i_P3_SResp(instr_P_SResp[3]),
	/* OCP interface: Port 4 (slave) */
	.o_P4_MAddr(instr_P_MAddr[4]),
	.o_P4_MCmd(instr_P_MCmd[4]),
	.o_P4_MData(instr_P_MData[4]),
	.o_P4_MByteEn(instr_P_MByteEn[4]),
	.i_P4_SCmdAccept(instr_P_SCmdAccept[4]),
	.i_P4_SData(instr_P_SData[4]),
	.i_P4_SResp(instr_P_SResp[4])
);

/* Data master port switch */
fabric2_mswitch #(.PORTNO_WIDTH(PORTNO_WIDTH)) data_mswitch(
	.i_portno(d_mswitch),
	/* OCP interface: instructions/data (master) */
	.i_ID_MAddr(data_decoded_addr),
	.i_ID_MCmd(i_D_MCmd),
	.i_ID_MData(i_D_MData),
	.i_ID_MByteEn(i_D_MByteEn),
	.o_ID_SCmdAccept(o_D_SCmdAccept),
	.o_ID_SData(o_D_SData),
	.o_ID_SResp(o_D_SResp),
	/* OCP interface: Port 0 (slave) */
	.o_P0_MAddr(data_P_MAddr[0]),
	.o_P0_MCmd(data_P_MCmd[0]),
	.o_P0_MData(data_P_MData[0]),
	.o_P0_MByteEn(data_P_MByteEn[0]),
	.i_P0_SCmdAccept(data_P_SCmdAccept[0]),
	.i_P0_SData(data_P_SData[0]),
	.i_P0_SResp(data_P_SResp[0]),
	/* OCP interface: Port 1 (slave) */
	.o_P1_MAddr(data_P_MAddr[1]),
	.o_P1_MCmd(data_P_MCmd[1]),
	.o_P1_MData(data_P_MData[1]),
	.o_P1_MByteEn(data_P_MByteEn[1]),
	.i_P1_SCmdAccept(data_P_SCmdAccept[1]),
	.i_P1_SData(data_P_SData[1]),
	.i_P1_SResp(data_P_SResp[1]),
	/* OCP interface: Port 2 (slave) */
	.o_P2_MAddr(data_P_MAddr[2]),
	.o_P2_MCmd(data_P_MCmd[2]),
	.o_P2_MData(data_P_MData[2]),
	.o_P2_MByteEn(data_P_MByteEn[2]),
	.i_P2_SCmdAccept(data_P_SCmdAccept[2]),
	.i_P2_SData(data_P_SData[2]),
	.i_P2_SResp(data_P_SResp[2]),
	/* OCP interface: Port 3 (slave) */
	.o_P3_MAddr(data_P_MAddr[3]),
	.o_P3_MCmd(data_P_MCmd[3]),
	.o_P3_MData(data_P_MData[3]),
	.o_P3_MByteEn(data_P_MByteEn[3]),
	.i_P3_SCmdAccept(data_P_SCmdAccept[3]),
	.i_P3_SData(data_P_SData[3]),
	.i_P3_SResp(data_P_SResp[3]),
	/* OCP interface: Port 4 (slave) */
	.o_P4_MAddr(data_P_MAddr[4]),
	.o_P4_MCmd(data_P_MCmd[4]),
	.o_P4_MData(data_P_MData[4]),
	.o_P4_MByteEn(data_P_MByteEn[4]),
	.i_P4_SCmdAccept(data_P_SCmdAccept[4]),
	.i_P4_SData(data_P_SData[4]),
	.i_P4_SResp(data_P_SResp[4])
);


/* Generate slave port switches */
genvar i;
generate
for (i=0; i<NPORTS; i=i+1)
begin : port_sswitch
	fabric2_sswitch sswitch(
		.i_port_sel(p_sswitch[i]),
		/* OCP interface: instructions (master) */
		.i_I_MAddr(instr_P_MAddr[i]),
		.i_I_MCmd(instr_P_MCmd[i]),
		.i_I_MData(instr_P_MData[i]),
		.i_I_MByteEn(instr_P_MByteEn[i]),
		.o_I_SCmdAccept(instr_P_SCmdAccept[i]),
		.o_I_SData(instr_P_SData[i]),
		.o_I_SResp(instr_P_SResp[i]),
		/* OCP interface: data (master) */
		.i_D_MAddr(data_P_MAddr[i]),
		.i_D_MCmd(data_P_MCmd[i]),
		.i_D_MData(data_P_MData[i]),
		.i_D_MByteEn(data_P_MByteEn[i]),
		.o_D_SCmdAccept(data_P_SCmdAccept[i]),
		.o_D_SData(data_P_SData[i]),
		.o_D_SResp(data_P_SResp[i]),
		/* OCP interface: Port (slave) */
		.o_P_MAddr(P_MAddr[i]),
		.o_P_MCmd(P_MCmd[i]),
		.o_P_MData(P_MData[i]),
		.o_P_MByteEn(P_MByteEn[i]),
		.i_P_SCmdAccept(P_SCmdAccept[i]),
		.i_P_SData(P_SData[i]),
		.i_P_SResp(P_SResp[i])
	);
end
endgenerate


/* Control and arbitration */
fabric2_control #(.PORTNO_WIDTH(PORTNO_WIDTH)) control(
	.clk(clk),
	.nrst(nrst),
	/* Active and completed transactions */
	.i_I_act(instr_act),
	.i_I_done(instr_done),
	.i_D_act(data_act),
	.i_D_done(data_done),
	/* Port numbers */
	.i_I_portno(instr_portno),
	.i_D_portno(data_portno),
	/* Master port switches control */
	.o_I_mswitch(i_mswitch),
	.o_D_mswitch(d_mswitch),
	/* Slave port switches control */
	.o_p0_sswitch(p_sswitch[0]),
	.o_p1_sswitch(p_sswitch[1]),
	.o_p2_sswitch(p_sswitch[2]),
	.o_p3_sswitch(p_sswitch[3]),
	.o_p4_sswitch(p_sswitch[4])
);


endmodule /* fabric2 */
