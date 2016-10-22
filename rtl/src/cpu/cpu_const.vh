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


/* Operation codes */
`define CPU_OP_SPECIAL		6'b000000
`define CPU_OP_COP0		6'b010000
`define CPU_OP_REGIMM		6'b000001
`define CPU_OP_J		6'b000010
`define CPU_OP_JAL		6'b000011
`define CPU_OP_BEQ		6'b000100
`define CPU_OP_BNE		6'b000101
`define CPU_OP_BLEZ		6'b000110
`define CPU_OP_BGTZ		6'b000111
`define CPU_OP_ADDI		6'b001000
`define CPU_OP_ADDIU		6'b001001
`define CPU_OP_SLTI		6'b001010
`define CPU_OP_SLTIU		6'b001011
`define CPU_OP_ANDI		6'b001100
`define CPU_OP_ORI		6'b001101
`define CPU_OP_XORI		6'b001110
`define CPU_OP_LUI		6'b001111
`define CPU_OP_LB		6'b100000
`define CPU_OP_LH		6'b100001
`define CPU_OP_LW		6'b100011
`define CPU_OP_LBU		6'b100100
`define CPU_OP_LHU		6'b100101
`define CPU_OP_SB		6'b101000
`define CPU_OP_SH		6'b101001
`define CPU_OP_SW		6'b101011


/* Function codes */
`define CPU_FUNC_SLL		6'b000000
`define CPU_FUNC_SRL		6'b000010
`define CPU_FUNC_SRA		6'b000011
`define CPU_FUNC_SLLV		6'b000100
`define CPU_FUNC_SRLV		6'b000110
`define CPU_FUNC_SRAV		6'b000111
`define CPU_FUNC_JR		6'b001000
`define CPU_FUNC_JALR		6'b001001
`define CPU_FUNC_SYSCALL	6'b001100
`define CPU_FUNC_BREAK		6'b001101
`define CPU_FUNC_MFHI		6'b010000
`define CPU_FUNC_MTHI		6'b010001
`define CPU_FUNC_MFLO		6'b010010
`define CPU_FUNC_MTLO		6'b010011
`define CPU_FUNC_MULT		6'b011000
`define CPU_FUNC_MULTU		6'b011001
`define CPU_FUNC_DIV		6'b011010
`define CPU_FUNC_DIVU		6'b011011
`define CPU_FUNC_ADD		6'b100000
`define CPU_FUNC_ADDU		6'b100001
`define CPU_FUNC_SUB		6'b100010
`define CPU_FUNC_SUBU		6'b100011
`define CPU_FUNC_AND		6'b100100
`define CPU_FUNC_OR		6'b100101
`define CPU_FUNC_XOR		6'b100110
`define CPU_FUNC_NOR		6'b100111
`define CPU_FUNC_SLT		6'b101010
`define CPU_FUNC_SLTU		6'b101011


/* REGIMM function codes */
`define CPU_REGIMM_BLTZ		5'b00000
`define CPU_REGIMM_BGEZ		5'b00001
`define CPU_REGIMM_BLTZAL	5'b10000
`define CPU_REGIMM_BGEZAL	5'b10001


/* Coprocessor 0 operation codes */
`define CPU_COP0_MF		5'b00000
`define CPU_COP0_MT		5'b00100
`define CPU_COP0_CO		5'b10000


/* Coprocessor 0 function codes */
`define CPU_COP0_FUNC_RFE	6'b010000


`endif /* _CPU_CONST_VH_ */
