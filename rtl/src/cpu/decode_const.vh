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
 * Decode stage constants
 */

`ifndef _DECODE_CONST_VH_
`define _DECODE_CONST_VH_


/* ALU inputs type */
localparam [4:0] DECODE_ALU_INPT_RSRT	= 5'b00001;
localparam [4:0] DECODE_ALU_INPT_RTRS	= 5'b00010;
localparam [4:0] DECODE_ALU_INPT_RSIMM	= 5'b00100;
localparam [4:0] DECODE_ALU_INPT_RTIMM	= 5'b01000;
localparam [4:0] DECODE_ALU_INPT_PCIMM	= 5'b10000;


/* Jump type */
localparam [4:0] DECODE_JUMPT_NONE	= 5'b00000;
localparam [4:0] DECODE_JUMPT_J		= 5'b10000;
localparam [4:0] DECODE_JUMPT_JR	= 5'b10001;
localparam [4:0] DECODE_JUMPT_BLTZ	= 5'b11000;
localparam [4:0] DECODE_JUMPT_BGEZ	= 5'b11001;
localparam [4:0] DECODE_JUMPT_BEQ	= 5'b11010;
localparam [4:0] DECODE_JUMPT_BNE	= 5'b11011;
localparam [4:0] DECODE_JUMPT_BLEZ	= 5'b11100;
localparam [4:0] DECODE_JUMPT_BGTZ	= 5'b11101;


`endif /* _DECODE_CONST_VH_ */
