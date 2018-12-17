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
 * Behavioral memory model
 */

`include "common.vh"
`include "ocp_const.vh"


/*
 * RAM
 */
module memory #(
	parameter MEMWORDS = 1048576 /* Memory size (number of data words) */
)
(
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

/* RAM */
reg [`DATA_WIDTH-1:0] mem[0:MEMWORDS-1];


integer i;
/* Preinit memory */
initial
begin : memory_init
`ifndef VERILATOR
	reg [65536*8-1:0] filepath;
`else
	reg [256*8-1:0] filepath;
	/** Verilator limits string length to 64 words (256*8/32 = 64) **/
`endif

	for(i=0; i<MEMWORDS; i=i+1) begin
		mem[i] = 0;
	end

	if($value$plusargs("MEMORY_FILE=%s", filepath))
	begin
		$readmemh(filepath, mem);
	end
	else
	begin
`ifdef MEMORY_IMAGE
		$readmemh(`MEMORY_IMAGE, mem);
`endif
	end
end


assign o_SCmdAccept = 1'b1;	/* Always accept command */


/* Bus logic */
always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		o_SData <= { (`DATA_WIDTH){1'b0} };
		o_SResp <= `OCP_RESP_NULL;
	end
	else
	begin
	/* verilator lint_off WIDTH */
		case(i_MCmd)
		`OCP_CMD_WRITE: begin
			if(i_MAddr[`ADDR_WIDTH-1:2] < MEMWORDS)
			begin
				if(i_MByteEn[0]) mem[i_MAddr[`ADDR_WIDTH-1:2]][7:0]   <= i_MData[7:0];
				if(i_MByteEn[1]) mem[i_MAddr[`ADDR_WIDTH-1:2]][15:8]  <= i_MData[15:8];
				if(i_MByteEn[2]) mem[i_MAddr[`ADDR_WIDTH-1:2]][23:16] <= i_MData[23:16];
				if(i_MByteEn[3]) mem[i_MAddr[`ADDR_WIDTH-1:2]][31:24] <= i_MData[31:24];
				/* Note: Need to be modified if DATA_WIDTH/BEN_WIDTH changed. */
			end
			o_SResp <= `OCP_RESP_DVA;
		end
		`OCP_CMD_READ: begin
			if(i_MAddr[`ADDR_WIDTH-1:2] < MEMWORDS)
			begin
				o_SData <= mem[i_MAddr[`ADDR_WIDTH-1:2]];
			end
			else
				o_SData <= 32'hDEADDEAD;
			o_SResp <= `OCP_RESP_DVA;
		end
		default: begin
			o_SResp <= `OCP_RESP_NULL;
		end
		endcase
	/* verilator lint_on WIDTH */
	end
end

endmodule /* memory */
