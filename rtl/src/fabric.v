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
 * System fabric
 */

`include "common.vh"
`include "ocp_const.vh"


/*
 * Fabric
 */
module fabric(
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
wire				I_P_SCmdAccept[0:NPORTS-1];
wire [`DATA_WIDTH-1:0]		I_P_SData[0:NPORTS-1];
wire [1:0]			I_P_SResp[0:NPORTS-1];

wire				D_P_SCmdAccept[0:NPORTS-1];
wire [`DATA_WIDTH-1:0]		D_P_SData[0:NPORTS-1];
wire [1:0]			D_P_SResp[0:NPORTS-1];

wire [`ADDR_WIDTH-1:0]	o_P_MAddr[0:NPORTS-1];
wire [2:0]		o_P_MCmd[0:NPORTS-1];
wire [`DATA_WIDTH-1:0]	o_P_MData[0:NPORTS-1];
wire [`BEN_WIDTH-1:0]	o_P_MByteEn[0:NPORTS-1];
wire			i_P_SCmdAccept[0:NPORTS-1];
wire [`DATA_WIDTH-1:0]	i_P_SData[0:NPORTS-1];
wire [1:0]		i_P_SResp[0:NPORTS-1];

assign o_P0_MAddr = o_P_MAddr[0];
assign o_P0_MCmd = o_P_MCmd[0];
assign o_P0_MData = o_P_MData[0];
assign o_P0_MByteEn = o_P_MByteEn[0];
assign i_P_SCmdAccept[0] = i_P0_SCmdAccept;
assign i_P_SData[0] = i_P0_SData;
assign i_P_SResp[0] = i_P0_SResp;

assign o_P1_MAddr = o_P_MAddr[1];
assign o_P1_MCmd = o_P_MCmd[1];
assign o_P1_MData = o_P_MData[1];
assign o_P1_MByteEn = o_P_MByteEn[1];
assign i_P_SCmdAccept[1] = i_P1_SCmdAccept;
assign i_P_SData[1] = i_P1_SData;
assign i_P_SResp[1] = i_P1_SResp;

assign o_P2_MAddr = o_P_MAddr[2];
assign o_P2_MCmd = o_P_MCmd[2];
assign o_P2_MData = o_P_MData[2];
assign o_P2_MByteEn = o_P_MByteEn[2];
assign i_P_SCmdAccept[2] = i_P2_SCmdAccept;
assign i_P_SData[2] = i_P2_SData;
assign i_P_SResp[2] = i_P2_SResp;

assign o_P3_MAddr = o_P_MAddr[3];
assign o_P3_MCmd = o_P_MCmd[3];
assign o_P3_MData = o_P_MData[3];
assign o_P3_MByteEn = o_P_MByteEn[3];
assign i_P_SCmdAccept[3] = i_P3_SCmdAccept;
assign i_P_SData[3] = i_P3_SData;
assign i_P_SResp[3] = i_P3_SResp;

assign o_P4_MAddr = o_P_MAddr[4];
assign o_P4_MCmd = o_P_MCmd[4];
assign o_P4_MData = o_P_MData[4];
assign o_P4_MByteEn = o_P_MByteEn[4];
assign i_P_SCmdAccept[4] = i_P4_SCmdAccept;
assign i_P_SData[4] = i_P4_SData;
assign i_P_SResp[4] = i_P4_SResp;


assign o_I_SCmdAccept = I_P_SCmdAccept[0] & I_P_SCmdAccept[1] & I_P_SCmdAccept[2] &
	I_P_SCmdAccept[3] & I_P_SCmdAccept[4];
assign o_I_SData = I_P_SData[0] | I_P_SData[1] | I_P_SData[2] | I_P_SData[3] | I_P_SData[4];
assign o_I_SResp = I_P_SResp[0] | I_P_SResp[1] | I_P_SResp[2] | I_P_SResp[3] | I_P_SResp[4];

assign o_D_SCmdAccept = D_P_SCmdAccept[0] & D_P_SCmdAccept[1] & D_P_SCmdAccept[2] &
	D_P_SCmdAccept[3] & D_P_SCmdAccept[4];
assign o_D_SData = D_P_SData[0] | D_P_SData[1] | D_P_SData[2] | D_P_SData[3] | D_P_SData[4];
assign o_D_SResp = D_P_SResp[0] | D_P_SResp[1] | D_P_SResp[2] | D_P_SResp[3] | D_P_SResp[4];


/* Ports FSMs */
genvar i;
generate
for (i=0; i<NPORTS; i=i+1)
begin : ports_inst
	fabric_port #(.PORT(i)) port(
		.clk(clk),
		.nrst(nrst),

		.i_I_MAddr(i_I_MAddr),
		.i_I_MCmd(i_I_MCmd),
		.i_I_MData(i_I_MData),
		.i_I_MByteEn(i_I_MByteEn),

		.o_I_SCmdAccept(I_P_SCmdAccept[i]),
		.o_I_SData(I_P_SData[i]),
		.o_I_SResp(I_P_SResp[i]),

		.i_D_MAddr(i_D_MAddr),
		.i_D_MCmd(i_D_MCmd),
		.i_D_MData(i_D_MData),
		.i_D_MByteEn(i_D_MByteEn),

		.o_D_SCmdAccept(D_P_SCmdAccept[i]),
		.o_D_SData(D_P_SData[i]),
		.o_D_SResp(D_P_SResp[i]),

		.o_P_MAddr(o_P_MAddr[i]),
		.o_P_MCmd(o_P_MCmd[i]),
		.o_P_MData(o_P_MData[i]),
		.o_P_MByteEn(o_P_MByteEn[i]),
		.i_P_SCmdAccept(i_P_SCmdAccept[i]),
		.i_P_SData(i_P_SData[i]),
		.i_P_SResp(i_P_SResp[i])
	);
end
endgenerate

endmodule /* fabric */
