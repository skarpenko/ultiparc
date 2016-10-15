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
 * Forwarding unit
 */

`include "cpu_common.vh"
`include "cpu_const.vh"


/* Forwarding unit */
module fwdu(
	rs,
	rs_data,
	rt,
	rt_data,
	rd_p3,
	rd_data_p3,
	rd_p4,
	rd_data_p4,
	rs_data_p2,
	rt_data_p2
);
/* rs and rt read at decode stage */
input wire [`CPU_REGNO_WIDTH-1:0]	rs;
input wire [`CPU_REG_WIDTH-1:0]		rs_data;
input wire [`CPU_REGNO_WIDTH-1:0]	rt;
input wire [`CPU_REG_WIDTH-1:0]		rt_data;
/* Destination at memory stage */
input wire [`CPU_REGNO_WIDTH-1:0]	rd_p3;
input wire [`CPU_REG_WIDTH-1:0]		rd_data_p3;
/* Destination at writeback stage */
input wire [`CPU_REGNO_WIDTH-1:0]	rd_p4;
input wire [`CPU_REG_WIDTH-1:0]		rd_data_p4;
/* Forwarded values of rs and rt */
output reg [`CPU_REG_WIDTH-1:0]		rs_data_p2;
output reg [`CPU_REG_WIDTH-1:0]		rt_data_p2;


always @(*)
begin
	if(rs && rs == rd_p3)
		rs_data_p2 = rd_data_p3;
	else if(rs && rs == rd_p4)
		rs_data_p2 = rd_data_p4;
	else
		rs_data_p2 = rs_data;
end


always @(*)
begin
	if(rt && rt == rd_p3)
		rt_data_p2 = rd_data_p3;
	else if(rt && rt == rd_p4)
		rt_data_p2 = rd_data_p4;
	else
		rt_data_p2 = rt_data;
end


endmodule /* fwdu */