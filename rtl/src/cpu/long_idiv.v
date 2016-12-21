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
 * Long integer division
 */

`include "cpu_common.vh"
`include "cpu_const.vh"


/* Division */
module long_idiv(
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
input wire [`CPU_REG_WIDTH-1:0]		dividend;
input wire [`CPU_REG_WIDTH-1:0]		divider;
input wire				start;
input wire				signd;
output wire				ready;
output wire [2*`CPU_REG_WIDTH-1:0]	remquot;


/* Local registers and wires */
reg [2*`CPU_REG_WIDTH-1:0]	qr;
reg [5:0]			bit;
wire [`CPU_REG_WIDTH:0]		diff;
reg [`CPU_REG_WIDTH-1:0]	abs_divider;


assign ready	= !bit && !start;


/* Result: remainder and quotient */
assign remquot[2*`CPU_REG_WIDTH-1:`CPU_REG_WIDTH]	=
			(signd && dividend[`CPU_REG_WIDTH-1]) ?
				-qr[2*`CPU_REG_WIDTH-1:`CPU_REG_WIDTH] :
				qr[2*`CPU_REG_WIDTH-1:`CPU_REG_WIDTH];
assign remquot[`CPU_REG_WIDTH-1:0]			=
			(signd && (dividend[`CPU_REG_WIDTH-1] ^ divider[`CPU_REG_WIDTH-1])) ?
				-qr[`CPU_REG_WIDTH-1:0] : qr[`CPU_REG_WIDTH-1:0];


/* Difference between divider and working portion of dividend */
assign diff	= qr[2*`CPU_REG_WIDTH-1:`CPU_REG_WIDTH-1] - { 1'b0, abs_divider };


always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		qr <= {(2*`CPU_REG_WIDTH){1'b0}};
		bit <= 6'b0;
		abs_divider <= {(`CPU_REG_WIDTH){1'b0}};
	end
	else if(start)
	begin
		if(!dividend || !divider)
		begin
			qr <= {(2*`CPU_REG_WIDTH){1'b0}};
			bit <= 6'b0;
		end
		else
		begin
			bit <= 6'd`CPU_REG_WIDTH;
			qr <= { {(`CPU_REG_WIDTH){1'b0}}, signd && dividend[`CPU_REG_WIDTH-1] ?
					-dividend : dividend };
			abs_divider <= signd && divider[`CPU_REG_WIDTH-1] ? -divider : divider;
		end
	end
	else if(bit)
	begin
		bit <= bit - 1;
		if(diff[`CPU_REG_WIDTH])
			qr <= { qr[2*`CPU_REG_WIDTH-2:0], 1'b0 };
		else
			qr <= { diff[`CPU_REG_WIDTH-1:0], qr[`CPU_REG_WIDTH-2:0], 1'b1 };
	end
end


endmodule /* long_idiv */
