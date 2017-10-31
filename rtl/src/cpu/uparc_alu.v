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
 * Arithmetic logic unit
 */

`include "uparc_cpu_config.vh"
`include "uparc_cpu_common.vh"
`include "uparc_cpu_const.vh"


/* ALU */
module uparc_alu(
	alu_op,
	a,
	b,
	result,
	ovflow,
	zero,
	neg
);
/* Inputs */
input wire [`UPARC_ALUOP_WIDTH-1:0] alu_op;
input wire [`UPARC_REG_WIDTH-1:0] a;
input wire [`UPARC_REG_WIDTH-1:0] b;
/* Outputs */
output reg [`UPARC_REG_WIDTH-1:0] result;
output wire ovflow;	/* Integer overflow */
output wire zero;	/* Result is zero */
output wire neg;	/* Result is negative */


/** Internal wires **/

wire [`UPARC_REG_WIDTH-1:0] b_mux;
wire [`UPARC_REG_WIDTH-1:0] a_plus_b;
wire [`UPARC_REG_WIDTH-1:0] a_or_b;

assign b_mux = (alu_op != `UPARC_ALUOP_SUB ? b : (~b) + 1);
assign a_plus_b = a + b_mux;
assign a_or_b = a | b;


/* Result is negative */
assign neg = result[`UPARC_REG_WIDTH-1];
/* Result is zero */
assign zero = (result ? 1'b0 : 1'b1);
/* Integer overflow */
assign ovflow = ~(a[`UPARC_REG_WIDTH-1] ^ b_mux[`UPARC_REG_WIDTH-1]) &
	(a[`UPARC_REG_WIDTH-1] ^ result[`UPARC_REG_WIDTH-1]);


always @(*)
begin
	case(alu_op)
	`UPARC_ALUOP_ADD:  result = a_plus_b;
	`UPARC_ALUOP_SUB:  result = a_plus_b;
	`UPARC_ALUOP_SLL:  result = a << b[4:0];
	`UPARC_ALUOP_SRL:  result = $unsigned(a) >> b[4:0];
	`UPARC_ALUOP_SRA:  result = $signed(a) >>> b[4:0];
	`UPARC_ALUOP_AND:  result = a & b;
	`UPARC_ALUOP_OR:   result = a_or_b;
	`UPARC_ALUOP_XOR:  result = a ^ b;
	`UPARC_ALUOP_NOR:  result = !a_or_b;
	`UPARC_ALUOP_SLT:  result = ($signed(a) < $signed(b) ?
		{{(`UPARC_REG_WIDTH-1){1'b0}}, 1'b1} : {(`UPARC_REG_WIDTH){1'b0}});
	`UPARC_ALUOP_SLTU: result = ($unsigned(a) < $unsigned(b) ?
		{{(`UPARC_REG_WIDTH-1){1'b0}}, 1'b1} : {(`UPARC_REG_WIDTH){1'b0}});
	default: result = {(`UPARC_REG_WIDTH){1'b0}};
	endcase
end


endmodule /* uparc_alu */
