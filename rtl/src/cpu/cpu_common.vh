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
 * CPU common defines
 */

`ifndef _CPU_COMMON_VH_
`define _CPU_COMMON_VH_


`define CPU_ADDR_WIDTH	32			/* Address width */
`define CPU_DATA_WIDTH	32			/* Data width */
`define CPU_BEN_WIDTH	(`CPU_DATA_WIDTH/8)	/* Byte enable width */
`define CPU_INSTR_WIDTH	32			/* Instruction width */
`define CPU_REG_WIDTH	32			/* Registers width */
`define CPU_REGNO_WIDTH	5			/* Register number width (0-31) */
`define CPU_ADDR_SIZE	(`CPU_ADDR_WIDTH/8)	/* Address size */
`define CPU_INSTR_SIZE	(`CPU_INSTR_WIDTH/8)	/* Instruction size */
`define CPU_LSUOP_WIDTH	2			/* LSU operation width */
`define CPU_ALUOP_WIDTH	4			/* ALU operation width */
`define CPU_IMDOP_WIDTH	4			/* Multiplication and division unit operation width */

`define CPU_RESET_ADDR	32'h0000_0000		/* Reset vector address */
`define CPU_PROCID_CODE	32'h001A_8100		/* CPU Identification */
/*
 * PROCID fields
 *
 * 0xSSCCPPRR
 *
 * SS - Company options;
 * CC - Company ID;
 * PP - CPU ID;
 * RR - Revision.
 *
 */


`endif /* _CPU_COMMON_VH_ */
