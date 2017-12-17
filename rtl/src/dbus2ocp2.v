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
 * CPU data bus to OCP bus converter (version 2)
 */

`include "common.vh"
`include "ocp_const.vh"


/* CPU DBus-to-OCP */
module dbus2ocp2(
	clk,
	nrst,
	/* CPU port */
	i_DAddr,
	i_DCmd,
	i_DRnW,
	i_DBen,
	i_DData,
	o_DData,
	o_DRdy,
	o_DErr,
	/* OCP port */
	o_MAddr,
	o_MCmd,
	o_MData,
	o_MByteEn,
	i_SCmdAccept,
	i_SData,
	i_SResp
);
input wire			clk;
input wire			nrst;
/* CPU port */
input wire [`ADDR_WIDTH-1:0]	i_DAddr;
input wire			i_DCmd;
input wire			i_DRnW;
input wire [`BEN_WIDTH-1:0]	i_DBen;
input wire [`DATA_WIDTH-1:0]	i_DData;
output reg [`DATA_WIDTH-1:0]	o_DData;
output reg			o_DRdy;
output reg			o_DErr;
/* OCP port */
output reg [`ADDR_WIDTH-1:0]	o_MAddr;
output reg [2:0]		o_MCmd;
output reg [`DATA_WIDTH-1:0]	o_MData;
output reg [`BEN_WIDTH-1:0]	o_MByteEn;
input wire			i_SCmdAccept;
input wire [`DATA_WIDTH-1:0]	i_SData;
input wire [1:0]		i_SResp;


/** Internal wires and registers **/

reg [`ADDR_WIDTH-1:0]	maddr_r;
reg [2:0]		mcmd_r;
reg [`DATA_WIDTH-1:0]	mdata_r;
reg [`BEN_WIDTH-1:0]	mben_r;
reg			naccept_r;
wire [2:0]		ocp_cmd = (i_DCmd ? (i_DRnW ? `OCP_CMD_READ : `OCP_CMD_WRITE) :
				`OCP_CMD_IDLE);


/* FSM */
always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		maddr_r <= { (`ADDR_WIDTH){1'b0} };
		mcmd_r <= `OCP_CMD_IDLE;
		mdata_r <= { (`DATA_WIDTH){1'b0} };
		mben_r <= { (`BEN_WIDTH){1'b0} };
		naccept_r <= 1'b0;
	end
	else if(i_DCmd && !i_SCmdAccept)
	begin
		maddr_r <= i_DAddr;
		mcmd_r <= ocp_cmd;
		mben_r <= i_DBen;
		naccept_r <= 1'b1;
		if(!i_DRnW)
			mdata_r <= i_DData;
	end
	else if(i_SCmdAccept)
	begin
		naccept_r <= 1'b0;
	end
end


/* Output logic */
always @(*)
begin
	o_MAddr = (!naccept_r ? i_DAddr : maddr_r);
	o_MCmd = (!naccept_r ? ocp_cmd : mcmd_r);
	o_MByteEn = (!naccept_r ? i_DBen : mben_r);
	o_MData = (!naccept_r ? i_DData : mdata_r);
	o_DData = i_SData;
	o_DRdy = (i_SResp != `OCP_RESP_NULL ? 1'b1 : 1'b0);
	o_DErr = (i_SResp == `OCP_RESP_ERR || i_SResp == `OCP_RESP_FAIL ? 1'b1 : 1'b0);
end


endmodule /* dbus2ocp2 */
