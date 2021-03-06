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
 * Long integer multiplication
 */

`include "uparc_cpu_config.vh"
`include "uparc_cpu_common.vh"
`include "uparc_cpu_const.vh"


/* Multiplication */
module uparc_long_imul(
	clk,
	nrst,
	multiplicand,
	multiplier,
	start,
	signd,
	ready,
	product
);
input wire				clk;
input wire				nrst;
input wire [`UPARC_REG_WIDTH-1:0]	multiplicand;
input wire [`UPARC_REG_WIDTH-1:0]	multiplier;
input wire				start;
input wire				signd;
output wire				ready;
output wire [2*`UPARC_REG_WIDTH-1:0]	product;


/* Local registers */
reg [5:0]			nbit;
reg [2*`UPARC_REG_WIDTH-1:0]	prod;
reg [`UPARC_REG_WIDTH-1:0]	abs_multiplicand;


assign ready	= !nbit && !start;
assign product	= (signd && (multiplicand[`UPARC_REG_WIDTH-1] ^ multiplier[`UPARC_REG_WIDTH-1])) ?
			-prod : prod;


always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		nbit <= 6'b0;
		prod <= {(2*`UPARC_REG_WIDTH){1'b0}};
		abs_multiplicand <= {(`UPARC_REG_WIDTH){1'b0}};
	end
	else if(start)
	begin
		if(!multiplicand || !multiplier)
		begin
			nbit <= 6'b0;
			prod <= {(2*`UPARC_REG_WIDTH){1'b0}};
		end
		else
		begin
			nbit <= 6'd`UPARC_REG_WIDTH;
			prod <= { {(`UPARC_REG_WIDTH){1'b0}}, signd && multiplier[`UPARC_REG_WIDTH-1] ?
					-multiplier : multiplier };
			abs_multiplicand <= signd && multiplicand[`UPARC_REG_WIDTH-1] ?
					-multiplicand : multiplicand;
		end
	end
	else if(nbit)
	begin
		nbit <= nbit - 1'b1;
		if(prod[0])
		begin
			prod[2*`UPARC_REG_WIDTH-1:`UPARC_REG_WIDTH-1] <=
					prod[2*`UPARC_REG_WIDTH-1:`UPARC_REG_WIDTH] + abs_multiplicand;
			prod[`UPARC_REG_WIDTH-2:0] <= prod[`UPARC_REG_WIDTH-1:1];
		end
		else
			prod <= { 1'b0, prod[2*`UPARC_REG_WIDTH-1:1] };
	end
end


endmodule /* uparc_long_imul */
