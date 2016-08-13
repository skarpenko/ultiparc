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
 * Arithmetic logic unit
 */

`include "cpu_common.vh"
`include "cpu_const.vh"


/* ALU */
module alu(
	alu_op,
	a,
	b,
	result,
	ovflow,
	zero,
	neg
);
/* Inpus */
input wire [`CPU_ALUOP_WIDTH-1:0] alu_op;
input wire [`CPU_REG_WIDTH-1:0] a;
input wire [`CPU_REG_WIDTH-1:0] b;
/* Outputs */
output reg [`CPU_REG_WIDTH-1:0] result;
output wire ovflow;	/* Integer overflow */
output wire zero;	/* Result is zero */
output wire neg;	/* Result is negative */


/** Internal wires **/

wire [`CPU_REG_WIDTH-1:0] b_mux;
wire [`CPU_REG_WIDTH-1:0] a_plus_b;
wire [`CPU_REG_WIDTH-1:0] a_or_b;

assign b_mux = (alu_op != `CPU_ALUOP_SUB ? b : (~b) + 1);
assign a_plus_b = a + b_mux;
assign a_or_b = a | b;


/* Result is negative */
assign neg = result[`CPU_REG_WIDTH-1];
/* Result is zero */
assign zero = (result ? 1'b0 : 1'b1);
/* Integer overflow */
assign ovflow =  (a[`CPU_REG_WIDTH-1] & b_mux[`CPU_REG_WIDTH-1]) ^
			(a[`CPU_REG_WIDTH-2] & b_mux[`CPU_REG_WIDTH-2]);


always @(*)
begin
	result = {(`CPU_REG_WIDTH){1'b0}};

	case(alu_op)
	`CPU_ALUOP_ADD:  result = a_plus_b;
	`CPU_ALUOP_SUB:  result = a_plus_b;
	`CPU_ALUOP_SLL:  result = a << b[4:0];
	`CPU_ALUOP_SRL:  result = $unsigned(a) >> b[4:0];
	`CPU_ALUOP_SRA:  result = $signed(a) >>> b[4:0];
	`CPU_ALUOP_AND:  result = a & b;
	`CPU_ALUOP_OR:   result = a_or_b;
	`CPU_ALUOP_XOR:  result = a ^ b;
	`CPU_ALUOP_NOR:  result = !a_or_b;
	`CPU_ALUOP_SLT:  result = ($signed(a) < $signed(b) ?
		{{(`CPU_REG_WIDTH-1){1'b0}}, 1'b1} : {(`CPU_REG_WIDTH){1'b0}});
	`CPU_ALUOP_SLTU: result = ($unsigned(a) < $unsigned(b) ?
		{{(`CPU_REG_WIDTH-1){1'b0}}, 1'b1} : {(`CPU_REG_WIDTH){1'b0}});
	endcase
end


endmodule /* alu */
