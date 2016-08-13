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
 * CPU constants
 */

`ifndef _CPU_CONST_VH_
`define _CPU_CONST_VH_


/* Load-store unit commands */
`define CPU_LSU_IDLE		2'b00	/* Idle state */
`define CPU_LSU_BYTE		2'b01	/* Load/store byte */
`define CPU_LSU_HWORD		2'b10	/* Load/store halfword */
`define CPU_LSU_WORD		2'b11	/* Load/store word */


/* ALU operations */
`define CPU_ALUOP_ADD		4'b0000	/* Addition  */
`define CPU_ALUOP_SUB		4'b0001	/* Subtraction */
`define CPU_ALUOP_SLL		4'b0010	/* Shift Left Logical */
`define CPU_ALUOP_SRL		4'b0011	/* Shift Right Logical */
`define CPU_ALUOP_SRA		4'b0100	/* Shift Right Arithmetic */
`define CPU_ALUOP_AND		4'b0101	/* And */
`define CPU_ALUOP_OR		4'b0110	/* Or */
`define CPU_ALUOP_XOR		4'b0111	/* Xor */
`define CPU_ALUOP_NOR		4'b1000	/* Nor */
`define CPU_ALUOP_SLT		4'b1001	/* Set Less Than (signed) */
`define CPU_ALUOP_SLTU		4'b1010	/* Set Less Than (unsigned) */
`define CPU_ALUOP_RESVD1	4'b1011	/* Reserved */
`define CPU_ALUOP_RESVD2	4'b1100	/* Reserved */
`define CPU_ALUOP_RESVD3	4'b1101	/* Reserved */
`define CPU_ALUOP_RESVD4	4'b1110	/* Reserved */
`define CPU_ALUOP_RESVD5	4'b1111	/* Reserved */


`endif /* _CPU_CONST_VH_ */
