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
 * Long integer division
 */

`include "uparc_cpu_config.vh"
`include "uparc_cpu_common.vh"
`include "uparc_cpu_const.vh"


/* Division */
module uparc_long_idiv(
	clk,
	nrst,
	dividend,
	divider,
	start,
	signd,
	ready,
	remquot
);
input wire				clk;
input wire				nrst;
input wire [`UPARC_REG_WIDTH-1:0]	dividend;
input wire [`UPARC_REG_WIDTH-1:0]	divider;
input wire				start;
input wire				signd;
output wire				ready;
output wire [2*`UPARC_REG_WIDTH-1:0]	remquot;


/* Local registers and wires */
reg [2*`UPARC_REG_WIDTH-1:0]	qr;
reg [5:0]			nbit;
wire [`UPARC_REG_WIDTH:0]	diff;
reg [`UPARC_REG_WIDTH-1:0]	abs_divider;


assign ready	= !(|nbit) && !start;


/* Result: remainder and quotient */
assign remquot[2*`UPARC_REG_WIDTH-1:`UPARC_REG_WIDTH]	=
			(signd && dividend[`UPARC_REG_WIDTH-1]) ?
				-qr[2*`UPARC_REG_WIDTH-1:`UPARC_REG_WIDTH] :
				qr[2*`UPARC_REG_WIDTH-1:`UPARC_REG_WIDTH];
assign remquot[`UPARC_REG_WIDTH-1:0]			=
			(signd && (dividend[`UPARC_REG_WIDTH-1] ^ divider[`UPARC_REG_WIDTH-1])) ?
				-qr[`UPARC_REG_WIDTH-1:0] : qr[`UPARC_REG_WIDTH-1:0];


/* Difference between divider and working portion of dividend */
assign diff	= qr[2*`UPARC_REG_WIDTH-1:`UPARC_REG_WIDTH-1] - { 1'b0, abs_divider };


always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		qr <= {(2*`UPARC_REG_WIDTH){1'b0}};
		nbit <= 6'b0;
		abs_divider <= {(`UPARC_REG_WIDTH){1'b0}};
	end
	else if(start)
	begin
		if(!(|dividend) || !(|divider))
		begin
			qr <= {(2*`UPARC_REG_WIDTH){1'b0}};
			nbit <= 6'b0;
		end
		else
		begin
			nbit <= 6'd`UPARC_REG_WIDTH;
			qr <= { {(`UPARC_REG_WIDTH){1'b0}}, signd && dividend[`UPARC_REG_WIDTH-1] ?
					-dividend : dividend };
			abs_divider <= signd && divider[`UPARC_REG_WIDTH-1] ? -divider : divider;
		end
	end
	else if(|nbit)
	begin
		nbit <= nbit - 1'b1;
		if(diff[`UPARC_REG_WIDTH])
			qr <= { qr[2*`UPARC_REG_WIDTH-2:0], 1'b0 };
		else
			qr <= { diff[`UPARC_REG_WIDTH-1:0], qr[`UPARC_REG_WIDTH-2:0], 1'b1 };
	end
end


endmodule /* uparc_long_idiv */
