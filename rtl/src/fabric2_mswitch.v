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


/* Master ports switch */
module fabric2_mswitch #(
	parameter PORTNO_WIDTH = 11
)
(
	/* Destination port select */
	i_portno,
	/* OCP interface: instructions/data (master) */
	i_ID_MAddr, i_ID_MCmd, i_ID_MData, i_ID_MByteEn, o_ID_SCmdAccept, o_ID_SData, o_ID_SResp,
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
/* Destination port select */
input wire [PORTNO_WIDTH-1:0]	i_portno;
/* OCP interface: instructions/data (master) */
input wire [`ADDR_WIDTH-1:0]	i_ID_MAddr;
input wire [2:0]		i_ID_MCmd;
input wire [`DATA_WIDTH-1:0]	i_ID_MData;
input wire [`BEN_WIDTH-1:0]	i_ID_MByteEn;
output reg			o_ID_SCmdAccept;
output reg [`DATA_WIDTH-1:0]	o_ID_SData;
output reg [1:0]		o_ID_SResp;
/* OCP interface: Port 0 (slave) */
output reg [`ADDR_WIDTH-1:0]	o_P0_MAddr;
output reg [2:0]		o_P0_MCmd;
output reg [`DATA_WIDTH-1:0]	o_P0_MData;
output reg [`BEN_WIDTH-1:0]	o_P0_MByteEn;
input wire			i_P0_SCmdAccept;
input wire [`DATA_WIDTH-1:0]	i_P0_SData;
input wire [1:0]		i_P0_SResp;
/* OCP interface: Port 1 (slave) */
output reg [`ADDR_WIDTH-1:0]	o_P1_MAddr;
output reg [2:0]		o_P1_MCmd;
output reg [`DATA_WIDTH-1:0]	o_P1_MData;
output reg [`BEN_WIDTH-1:0]	o_P1_MByteEn;
input wire			i_P1_SCmdAccept;
input wire [`DATA_WIDTH-1:0]	i_P1_SData;
input wire [1:0]		i_P1_SResp;
/* OCP interface: Port 2 (slave) */
output reg [`ADDR_WIDTH-1:0]	o_P2_MAddr;
output reg [2:0]		o_P2_MCmd;
output reg [`DATA_WIDTH-1:0]	o_P2_MData;
output reg [`BEN_WIDTH-1:0]	o_P2_MByteEn;
input wire			i_P2_SCmdAccept;
input wire [`DATA_WIDTH-1:0]	i_P2_SData;
input wire [1:0]		i_P2_SResp;
/* OCP interface: Port 3 (slave) */
output reg [`ADDR_WIDTH-1:0]	o_P3_MAddr;
output reg [2:0]		o_P3_MCmd;
output reg [`DATA_WIDTH-1:0]	o_P3_MData;
output reg [`BEN_WIDTH-1:0]	o_P3_MByteEn;
input wire			i_P3_SCmdAccept;
input wire [`DATA_WIDTH-1:0]	i_P3_SData;
input wire [1:0]		i_P3_SResp;
/* OCP interface: Port 4 (slave) */
output reg [`ADDR_WIDTH-1:0]	o_P4_MAddr;
output reg [2:0]		o_P4_MCmd;
output reg [`DATA_WIDTH-1:0]	o_P4_MData;
output reg [`BEN_WIDTH-1:0]	o_P4_MByteEn;
input wire			i_P4_SCmdAccept;
input wire [`DATA_WIDTH-1:0]	i_P4_SData;
input wire [1:0]		i_P4_SResp;



always @(*)
begin
	o_ID_SCmdAccept = 1'b0;
	o_ID_SData = { (`DATA_WIDTH){1'b0} };
	o_ID_SResp = 2'b00;
	o_P0_MAddr = { (`ADDR_WIDTH){1'b0} };
	o_P0_MCmd = 3'b000;
	o_P0_MData = { (`DATA_WIDTH){1'b0} };
	o_P0_MByteEn = { (`BEN_WIDTH){1'b0} };
	o_P1_MAddr = { (`ADDR_WIDTH){1'b0} };
	o_P1_MCmd = 3'b000;
	o_P1_MData = { (`DATA_WIDTH){1'b0} };
	o_P1_MByteEn = { (`BEN_WIDTH){1'b0} };
	o_P2_MAddr = { (`ADDR_WIDTH){1'b0} };
	o_P2_MCmd = 3'b000;
	o_P2_MData = { (`DATA_WIDTH){1'b0} };
	o_P2_MByteEn = { (`BEN_WIDTH){1'b0} };
	o_P3_MAddr = { (`ADDR_WIDTH){1'b0} };
	o_P3_MCmd = 3'b000;
	o_P3_MData = { (`DATA_WIDTH){1'b0} };
	o_P3_MByteEn = { (`BEN_WIDTH){1'b0} };
	o_P4_MAddr = { (`ADDR_WIDTH){1'b0} };
	o_P4_MCmd = 3'b000;
	o_P4_MData = { (`DATA_WIDTH){1'b0} };
	o_P4_MByteEn = { (`BEN_WIDTH){1'b0} };


	case(i_portno)
	'd1: begin
		o_P1_MAddr = i_ID_MAddr;
		o_P1_MCmd = i_ID_MCmd;
		o_P1_MData = i_ID_MData;
		o_P1_MByteEn = i_ID_MByteEn;
		o_ID_SCmdAccept = i_P1_SCmdAccept;
		o_ID_SData = i_P1_SData;
		o_ID_SResp = i_P1_SResp;
	end
	'd2: begin
		o_P2_MAddr = i_ID_MAddr;
		o_P2_MCmd = i_ID_MCmd;
		o_P2_MData = i_ID_MData;
		o_P2_MByteEn = i_ID_MByteEn;
		o_ID_SCmdAccept = i_P2_SCmdAccept;
		o_ID_SData = i_P2_SData;
		o_ID_SResp = i_P2_SResp;
	end
	'd3: begin
		o_P3_MAddr = i_ID_MAddr;
		o_P3_MCmd = i_ID_MCmd;
		o_P3_MData = i_ID_MData;
		o_P3_MByteEn = i_ID_MByteEn;
		o_ID_SCmdAccept = i_P3_SCmdAccept;
		o_ID_SData = i_P3_SData;
		o_ID_SResp = i_P3_SResp;
	end
	'd4: begin
		o_P4_MAddr = i_ID_MAddr;
		o_P4_MCmd = i_ID_MCmd;
		o_P4_MData = i_ID_MData;
		o_P4_MByteEn = i_ID_MByteEn;
		o_ID_SCmdAccept = i_P4_SCmdAccept;
		o_ID_SData = i_P4_SData;
		o_ID_SResp = i_P4_SResp;
	end
	default: begin
		o_P0_MAddr = i_ID_MAddr;
		o_P0_MCmd = i_ID_MCmd;
		o_P0_MData = i_ID_MData;
		o_P0_MByteEn = i_ID_MByteEn;
		o_ID_SCmdAccept = i_P0_SCmdAccept;
		o_ID_SData = i_P0_SData;
		o_ID_SResp = i_P0_SResp;
	end
	endcase
end


endmodule /* fabric2_mswitch */
