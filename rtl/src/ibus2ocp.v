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
 * CPU instructions bus to OCP bus wrapper
 */

`include "common.vh"
`include "ocp_const.vh"


/* CPU IBus-to-OCP */
module ibus2ocp(
	/* CPU port */
	i_IAddr,
	i_IRdC,
	o_IData,
	o_IRdy,
	o_IErr,
	/* OCP port */
	o_MAddr,
	o_MCmd,
	o_MData,
	o_MByteEn,
	i_SCmdAccept,
	i_SData,
	i_SResp
);
/* CPU port */
input wire [`ADDR_WIDTH-1:0]	i_IAddr;
input wire			i_IRdC;
output reg [`DATA_WIDTH-1:0]	o_IData;
output reg			o_IRdy;
output reg			o_IErr;
/* OCP port */
output reg [`ADDR_WIDTH-1:0]	o_MAddr;
output reg [2:0]		o_MCmd;
output reg [`DATA_WIDTH-1:0]	o_MData;	/* Not used */
output reg [`BEN_WIDTH-1:0]	o_MByteEn;
input wire			i_SCmdAccept;	/* Not used */
input wire [`DATA_WIDTH-1:0]	i_SData;
input wire [1:0]		i_SResp;


always @(*)
begin
	o_MAddr = i_IAddr;
	o_MCmd = (i_IRdC ? `OCP_CMD_READ : `OCP_CMD_IDLE);
	o_MData = {(`DATA_WIDTH){1'b0}};
	o_MByteEn = {(`BEN_WIDTH){1'b1}};
	o_IData = i_SData;
	o_IRdy = (i_SResp != `OCP_RESP_NULL ? 1'b1 : 1'b0);
	o_IErr = (i_SResp == `OCP_RESP_ERR || i_SResp == `OCP_RESP_FAIL ? 1'b1 : 1'b0);
end


endmodule /* ibus2ocp */
