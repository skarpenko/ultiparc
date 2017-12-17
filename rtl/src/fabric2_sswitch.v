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


/* Slave ports switch */
module fabric2_sswitch(
	/* Source port select */
	i_port_sel,
	/* OCP interface: instructions (master) */
	i_I_MAddr, i_I_MCmd, i_I_MData, i_I_MByteEn, o_I_SCmdAccept, o_I_SData, o_I_SResp,
	/* OCP interface: data (master) */
	i_D_MAddr, i_D_MCmd, i_D_MData, i_D_MByteEn, o_D_SCmdAccept, o_D_SData, o_D_SResp,
	/* OCP interface: Port 0 (slave) */
	o_P_MAddr, o_P_MCmd, o_P_MData, o_P_MByteEn, i_P_SCmdAccept, i_P_SData, i_P_SResp
);
/* Source port select */
input wire			i_port_sel;
/* OCP interface: instructions (master) */
input wire [`ADDR_WIDTH-1:0]	i_I_MAddr;
input wire [2:0]		i_I_MCmd;
input wire [`DATA_WIDTH-1:0]	i_I_MData;
input wire [`BEN_WIDTH-1:0]	i_I_MByteEn;
output reg			o_I_SCmdAccept;
output reg [`DATA_WIDTH-1:0]	o_I_SData;
output reg [1:0]		o_I_SResp;
/* OCP interface: data (master) */
input wire [`ADDR_WIDTH-1:0]	i_D_MAddr;
input wire [2:0]		i_D_MCmd;
input wire [`DATA_WIDTH-1:0]	i_D_MData;
input wire [`BEN_WIDTH-1:0]	i_D_MByteEn;
output reg			o_D_SCmdAccept;
output reg [`DATA_WIDTH-1:0]	o_D_SData;
output reg [1:0]		o_D_SResp;
/* OCP interface: Port (slave) */
output reg [`ADDR_WIDTH-1:0]	o_P_MAddr;
output reg [2:0]		o_P_MCmd;
output reg [`DATA_WIDTH-1:0]	o_P_MData;
output reg [`BEN_WIDTH-1:0]	o_P_MByteEn;
input wire			i_P_SCmdAccept;
input wire [`DATA_WIDTH-1:0]	i_P_SData;
input wire [1:0]		i_P_SResp;



always @(*)
begin
/*
	o_I_SCmdAccept = 1'b0;
	o_I_SData = { (`DATA_WIDTH){1'b0} };
	o_I_SResp = 2'b00;
	o_D_SCmdAccept = 1'b0;
	o_D_SData = { (`DATA_WIDTH){1'b0} };
	o_D_SResp = 2'b00;
	o_P_MAddr = { (`ADDR_WIDTH){1'b0} };
	o_P_MCmd = 3'b000;
	o_P_MData = { (`DATA_WIDTH){1'b0} };
	o_P_MByteEn = { (`BEN_WIDTH){1'b0} };
*/

	if(!i_port_sel)
	begin
		o_P_MAddr = i_I_MAddr;
		o_P_MCmd = i_I_MCmd;
		o_P_MData = i_I_MData;
		o_P_MByteEn = i_I_MByteEn;
		o_I_SCmdAccept = i_P_SCmdAccept;
		o_I_SData = i_P_SData;
		o_I_SResp = i_P_SResp;
			o_D_SCmdAccept = 1'b0;
			o_D_SData = { (`DATA_WIDTH){1'b0} };
			o_D_SResp = 2'b00;
	end
	else
	begin
		o_P_MAddr = i_D_MAddr;
		o_P_MCmd = i_D_MCmd;
		o_P_MData = i_D_MData;
		o_P_MByteEn = i_D_MByteEn;
			o_I_SCmdAccept = 1'b0;
			o_I_SData = { (`DATA_WIDTH){1'b0} };
			o_I_SResp = 2'b00;
		o_D_SCmdAccept = i_P_SCmdAccept;
		o_D_SData = i_P_SData;
		o_D_SResp = i_P_SResp;
	end
end


endmodule /* fabric2_sswitch */
