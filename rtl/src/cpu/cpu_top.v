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
 * CPU top level
 */

`include "cpu_common.vh"
`include "cpu_const.vh"


/* CPU */
module cpu_top(
	clk,
	nrst,
	/* Interrupt input */
	i_intr,
	/* I-Port */
	o_IAddr,
	o_IRdC,
	i_IData,
	i_IRdy,
	i_IErr,
	/* D-Port */
	o_DAddr,
	o_DCmd,
	o_DRnW,
	o_DBen,
	o_DData,
	i_DData,
	i_DRdy,
	i_DErr
);
input wire				clk;
input wire				nrst;
/* Interrupt input */
input wire				i_intr;
/* I-Port */
output reg [`CPU_ADDR_WIDTH-1:0]	o_IAddr;
output reg				o_IRdC;
input wire [`CPU_DATA_WIDTH-1:0]	i_IData;
input wire				i_IRdy;
input wire				i_IErr;
/* D-Port */
output reg [`CPU_ADDR_WIDTH-1:0]	o_DAddr;
output reg				o_DCmd;
output reg				o_DRnW;
output reg [`CPU_BEN_WIDTH-1:0]		o_DBen;
output reg [`CPU_DATA_WIDTH-1:0]	o_DData;
input wire [`CPU_DATA_WIDTH-1:0]	i_DData;
input wire				i_DRdy;
input wire				i_DErr;


/* tbd */
always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		o_IAddr <= 0;
		o_IRdC <= 0;
		o_DAddr <= 0;
		o_DCmd <= 0;
		o_DRnW <= 0;
		o_DBen <= 0;
		o_DData <= 0;
	end
	else if(o_IRdC == 1'b0)
	begin
		o_IRdC <= 1'b1;
		o_DBen <= 4'hf;
		o_DCmd <= 1'b1;
	end
	else
	begin
		o_IRdC <= 1'b0;
		o_DBen <= 4'hf;
		o_DCmd <= 1'b0;
	end
end




endmodule /* cpu_top */
