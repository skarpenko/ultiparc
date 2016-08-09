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
 * Register file
 */

`include "cpu_common.vh"
`include "cpu_const.vh"


/* RF */
module reg_file(
	clk,
	nrst,
	rs,
	rs_data,
	rt,
	rt_data,
	rd,
	rd_data
);
`include "reg_names.vh"

input wire clk;
input wire nrst;
/* First source register */
input wire [`CPU_REGNO_WIDTH-1:0] rs;
output reg [`CPU_REG_WIDTH-1:0]   rs_data;
/* Second source register */
input wire [`CPU_REGNO_WIDTH-1:0] rt;
output reg [`CPU_REG_WIDTH-1:0]   rt_data;
/* Destination register */
input wire [`CPU_REGNO_WIDTH-1:0] rd;
input wire [`CPU_REG_WIDTH-1:0]   rd_data;


/* Registers */
reg [`CPU_REG_WIDTH-1:0] r1;
reg [`CPU_REG_WIDTH-1:0] r2;
reg [`CPU_REG_WIDTH-1:0] r3;
reg [`CPU_REG_WIDTH-1:0] r4;
reg [`CPU_REG_WIDTH-1:0] r5;
reg [`CPU_REG_WIDTH-1:0] r6;
reg [`CPU_REG_WIDTH-1:0] r7;
reg [`CPU_REG_WIDTH-1:0] r8;
reg [`CPU_REG_WIDTH-1:0] r9;
reg [`CPU_REG_WIDTH-1:0] r10;
reg [`CPU_REG_WIDTH-1:0] r11;
reg [`CPU_REG_WIDTH-1:0] r12;
reg [`CPU_REG_WIDTH-1:0] r13;
reg [`CPU_REG_WIDTH-1:0] r14;
reg [`CPU_REG_WIDTH-1:0] r15;
reg [`CPU_REG_WIDTH-1:0] r16;
reg [`CPU_REG_WIDTH-1:0] r17;
reg [`CPU_REG_WIDTH-1:0] r18;
reg [`CPU_REG_WIDTH-1:0] r19;
reg [`CPU_REG_WIDTH-1:0] r20;
reg [`CPU_REG_WIDTH-1:0] r21;
reg [`CPU_REG_WIDTH-1:0] r22;
reg [`CPU_REG_WIDTH-1:0] r23;
reg [`CPU_REG_WIDTH-1:0] r24;
reg [`CPU_REG_WIDTH-1:0] r25;
reg [`CPU_REG_WIDTH-1:0] r26;
reg [`CPU_REG_WIDTH-1:0] r27;
reg [`CPU_REG_WIDTH-1:0] r28;
reg [`CPU_REG_WIDTH-1:0] r29;
reg [`CPU_REG_WIDTH-1:0] r30;
reg [`CPU_REG_WIDTH-1:0] r31;


wire [`CPU_REG_WIDTH-1:0] r0_wire;
wire [`CPU_REG_WIDTH-1:0] r1_wire;
wire [`CPU_REG_WIDTH-1:0] r2_wire;
wire [`CPU_REG_WIDTH-1:0] r3_wire;
wire [`CPU_REG_WIDTH-1:0] r4_wire;
wire [`CPU_REG_WIDTH-1:0] r5_wire;
wire [`CPU_REG_WIDTH-1:0] r6_wire;
wire [`CPU_REG_WIDTH-1:0] r7_wire;
wire [`CPU_REG_WIDTH-1:0] r8_wire;
wire [`CPU_REG_WIDTH-1:0] r9_wire;
wire [`CPU_REG_WIDTH-1:0] r10_wire;
wire [`CPU_REG_WIDTH-1:0] r11_wire;
wire [`CPU_REG_WIDTH-1:0] r12_wire;
wire [`CPU_REG_WIDTH-1:0] r13_wire;
wire [`CPU_REG_WIDTH-1:0] r14_wire;
wire [`CPU_REG_WIDTH-1:0] r15_wire;
wire [`CPU_REG_WIDTH-1:0] r16_wire;
wire [`CPU_REG_WIDTH-1:0] r17_wire;
wire [`CPU_REG_WIDTH-1:0] r18_wire;
wire [`CPU_REG_WIDTH-1:0] r19_wire;
wire [`CPU_REG_WIDTH-1:0] r20_wire;
wire [`CPU_REG_WIDTH-1:0] r21_wire;
wire [`CPU_REG_WIDTH-1:0] r22_wire;
wire [`CPU_REG_WIDTH-1:0] r23_wire;
wire [`CPU_REG_WIDTH-1:0] r24_wire;
wire [`CPU_REG_WIDTH-1:0] r25_wire;
wire [`CPU_REG_WIDTH-1:0] r26_wire;
wire [`CPU_REG_WIDTH-1:0] r27_wire;
wire [`CPU_REG_WIDTH-1:0] r28_wire;
wire [`CPU_REG_WIDTH-1:0] r29_wire;
wire [`CPU_REG_WIDTH-1:0] r30_wire;
wire [`CPU_REG_WIDTH-1:0] r31_wire;


assign r0_wire  = {(`CPU_REG_WIDTH){1'b0}};	/* Always 0 */
assign r1_wire  = (rd == R1 ?  rd_data : r1);
assign r2_wire  = (rd == R2 ?  rd_data : r2);
assign r3_wire  = (rd == R3 ?  rd_data : r3);
assign r4_wire  = (rd == R4 ?  rd_data : r4);
assign r5_wire  = (rd == R5 ?  rd_data : r5);
assign r6_wire  = (rd == R6 ?  rd_data : r6);
assign r7_wire  = (rd == R7 ?  rd_data : r7);
assign r8_wire  = (rd == R8 ?  rd_data : r8);
assign r9_wire  = (rd == R9 ?  rd_data : r9);
assign r10_wire = (rd == R10 ? rd_data : r10);
assign r11_wire = (rd == R11 ? rd_data : r11);
assign r12_wire = (rd == R12 ? rd_data : r12);
assign r13_wire = (rd == R13 ? rd_data : r13);
assign r14_wire = (rd == R14 ? rd_data : r14);
assign r15_wire = (rd == R15 ? rd_data : r15);
assign r16_wire = (rd == R16 ? rd_data : r16);
assign r17_wire = (rd == R17 ? rd_data : r17);
assign r18_wire = (rd == R18 ? rd_data : r18);
assign r19_wire = (rd == R19 ? rd_data : r19);
assign r20_wire = (rd == R20 ? rd_data : r20);
assign r21_wire = (rd == R21 ? rd_data : r21);
assign r22_wire = (rd == R22 ? rd_data : r22);
assign r23_wire = (rd == R23 ? rd_data : r23);
assign r24_wire = (rd == R24 ? rd_data : r24);
assign r25_wire = (rd == R25 ? rd_data : r25);
assign r26_wire = (rd == R26 ? rd_data : r26);
assign r27_wire = (rd == R27 ? rd_data : r27);
assign r28_wire = (rd == R28 ? rd_data : r28);
assign r29_wire = (rd == R29 ? rd_data : r29);
assign r30_wire = (rd == R30 ? rd_data : r30);
assign r31_wire = (rd == R31 ? rd_data : r31);


always @(*)
begin
	case(rs)
	R0:  rs_data = r0_wire;
	R1:  rs_data = r1_wire;
	R2:  rs_data = r2_wire;
	R3:  rs_data = r3_wire;
	R4:  rs_data = r4_wire;
	R5:  rs_data = r5_wire;
	R6:  rs_data = r6_wire;
	R7:  rs_data = r7_wire;
	R8:  rs_data = r8_wire;
	R9:  rs_data = r9_wire;
	R10: rs_data = r10_wire;
	R11: rs_data = r11_wire;
	R12: rs_data = r12_wire;
	R13: rs_data = r13_wire;
	R14: rs_data = r14_wire;
	R15: rs_data = r15_wire;
	R16: rs_data = r16_wire;
	R17: rs_data = r17_wire;
	R18: rs_data = r18_wire;
	R19: rs_data = r19_wire;
	R20: rs_data = r20_wire;
	R21: rs_data = r21_wire;
	R22: rs_data = r22_wire;
	R23: rs_data = r23_wire;
	R24: rs_data = r24_wire;
	R25: rs_data = r25_wire;
	R26: rs_data = r26_wire;
	R27: rs_data = r27_wire;
	R28: rs_data = r28_wire;
	R29: rs_data = r29_wire;
	R30: rs_data = r30_wire;
	R31: rs_data = r31_wire;
	endcase
end


always @(*)
begin
	case(rt)
	R0:  rt_data = r0_wire;
	R1:  rt_data = r1_wire;
	R2:  rt_data = r2_wire;
	R3:  rt_data = r3_wire;
	R4:  rt_data = r4_wire;
	R5:  rt_data = r5_wire;
	R6:  rt_data = r6_wire;
	R7:  rt_data = r7_wire;
	R8:  rt_data = r8_wire;
	R9:  rt_data = r9_wire;
	R10: rt_data = r10_wire;
	R11: rt_data = r11_wire;
	R12: rt_data = r12_wire;
	R13: rt_data = r13_wire;
	R14: rt_data = r14_wire;
	R15: rt_data = r15_wire;
	R16: rt_data = r16_wire;
	R17: rt_data = r17_wire;
	R18: rt_data = r18_wire;
	R19: rt_data = r19_wire;
	R20: rt_data = r20_wire;
	R21: rt_data = r21_wire;
	R22: rt_data = r22_wire;
	R23: rt_data = r23_wire;
	R24: rt_data = r24_wire;
	R25: rt_data = r25_wire;
	R26: rt_data = r26_wire;
	R27: rt_data = r27_wire;
	R28: rt_data = r28_wire;
	R29: rt_data = r29_wire;
	R30: rt_data = r30_wire;
	R31: rt_data = r31_wire;
	endcase
end


always @(posedge clk or negedge nrst)
begin
	if (!nrst)
	begin
		r1 <= {(`CPU_REG_WIDTH){1'b0}};
		r2 <= {(`CPU_REG_WIDTH){1'b0}};
		r3 <= {(`CPU_REG_WIDTH){1'b0}};
		r4 <= {(`CPU_REG_WIDTH){1'b0}};
		r5 <= {(`CPU_REG_WIDTH){1'b0}};
		r6 <= {(`CPU_REG_WIDTH){1'b0}};
		r7 <= {(`CPU_REG_WIDTH){1'b0}};
		r8 <= {(`CPU_REG_WIDTH){1'b0}};
		r9 <= {(`CPU_REG_WIDTH){1'b0}};
		r10 <= {(`CPU_REG_WIDTH){1'b0}};
		r11 <= {(`CPU_REG_WIDTH){1'b0}};
		r12 <= {(`CPU_REG_WIDTH){1'b0}};
		r13 <= {(`CPU_REG_WIDTH){1'b0}};
		r14 <= {(`CPU_REG_WIDTH){1'b0}};
		r15 <= {(`CPU_REG_WIDTH){1'b0}};
		r16 <= {(`CPU_REG_WIDTH){1'b0}};
		r17 <= {(`CPU_REG_WIDTH){1'b0}};
		r18 <= {(`CPU_REG_WIDTH){1'b0}};
		r19 <= {(`CPU_REG_WIDTH){1'b0}};
		r20 <= {(`CPU_REG_WIDTH){1'b0}};
		r21 <= {(`CPU_REG_WIDTH){1'b0}};
		r22 <= {(`CPU_REG_WIDTH){1'b0}};
		r23 <= {(`CPU_REG_WIDTH){1'b0}};
		r24 <= {(`CPU_REG_WIDTH){1'b0}};
		r25 <= {(`CPU_REG_WIDTH){1'b0}};
		r26 <= {(`CPU_REG_WIDTH){1'b0}};
		r27 <= {(`CPU_REG_WIDTH){1'b0}};
		r28 <= {(`CPU_REG_WIDTH){1'b0}};
		r29 <= {(`CPU_REG_WIDTH){1'b0}};
		r30 <= {(`CPU_REG_WIDTH){1'b0}};
		r31 <= {(`CPU_REG_WIDTH){1'b0}};
	end
	else
	begin
		case(rd)
		R1:  r1 <= rd_data;
		R2:  r2 <= rd_data;
		R3:  r3 <= rd_data;
		R4:  r4 <= rd_data;
		R5:  r5 <= rd_data;
		R6:  r6 <= rd_data;
		R7:  r7 <= rd_data;
		R8:  r8 <= rd_data;
		R9:  r9 <= rd_data;
		R10: r10 <= rd_data;
		R11: r11 <= rd_data;
		R12: r12 <= rd_data;
		R13: r13 <= rd_data;
		R14: r14 <= rd_data;
		R15: r15 <= rd_data;
		R16: r16 <= rd_data;
		R17: r17 <= rd_data;
		R18: r18 <= rd_data;
		R19: r19 <= rd_data;
		R20: r20 <= rd_data;
		R21: r21 <= rd_data;
		R22: r22 <= rd_data;
		R23: r23 <= rd_data;
		R24: r24 <= rd_data;
		R25: r25 <= rd_data;
		R26: r26 <= rd_data;
		R27: r27 <= rd_data;
		R28: r28 <= rd_data;
		R29: r29 <= rd_data;
		R30: r30 <= rd_data;
		R31: r31 <= rd_data;
		endcase
	end
end


endmodule /* reg_file */
