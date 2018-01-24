/*
 * Copyright (c) 2015-2018 The Ultiparc Project. All rights reserved.
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
 * Micro UART
 */

`include "common.vh"
`include "ocp_const.vh"


/*
 * Micro UART
 *  Registers:
 *    0x00  -  character register;
 *             Bits:
 *              [31:8] - ignored;
 *              [7:0]  - character value to print.
 */
module micro_uart(
	clk,
	nrst,
	/* OCP interface */
	i_MAddr,
	i_MCmd,
	i_MData,
	i_MByteEn,
	o_SCmdAccept,
	o_SData,
	o_SResp
);
/* Register offsets */
localparam [`ADDR_WIDTH-1:0] CHARREG = 32'h000;	/* Character register */

/* Inputs and outputs */
input wire			clk;
input wire			nrst;
input wire [`ADDR_WIDTH-1:0]	i_MAddr;
input wire [2:0]		i_MCmd;
input wire [`DATA_WIDTH-1:0]	i_MData;
input wire [`BEN_WIDTH-1:0]	i_MByteEn;
output wire			o_SCmdAccept;
output reg [`DATA_WIDTH-1:0]	o_SData;
output reg [1:0]		o_SResp;


assign o_SCmdAccept = 1'b1;	/* Always ready to accept command */


/* Bus logic */
always @(*)
begin
	case(i_MCmd)
	`OCP_CMD_WRITE: begin
		o_SData = { (`DATA_WIDTH){1'b0} };
		o_SResp = `OCP_RESP_DVA;
	end
	`OCP_CMD_READ: begin
		if(i_MAddr == CHARREG)
		begin
			/* Ignored. Always 0. */
			o_SData = { (`DATA_WIDTH){1'b0} };
		end
		else
			o_SData = 32'hDEADDEAD;
		o_SResp = `OCP_RESP_DVA;
	end
	default: begin
		o_SData = { (`DATA_WIDTH){1'b0} };
		o_SResp = `OCP_RESP_NULL;
	end
	endcase
end


/* Print character */
always @(posedge clk)
begin
	if(i_MCmd == `OCP_CMD_WRITE && i_MAddr == CHARREG)
	begin
		$write("%c", i_MData[7:0]);
`ifndef VERILATOR
		$fflush();	/* Verilator doesn't support it */
`endif
	end
end

endmodule /* micro_uart */
