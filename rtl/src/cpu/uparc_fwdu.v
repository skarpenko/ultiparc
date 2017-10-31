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
 * Forwarding unit
 */

`include "uparc_cpu_config.vh"
`include "uparc_cpu_common.vh"
`include "uparc_cpu_const.vh"


/* Forwarding unit */
module uparc_fwdu(
	rs,
	rs_data,
	rt,
	rt_data,
	rd_p2,
	rd_data_p2,
	pend_mem_load_p2,
	rd_p3,
	rd_data_p3,
	rs_data_p1,
	rt_data_p1
);
/* rs and rt read at decode stage */
input wire [`UPARC_REGNO_WIDTH-1:0]	rs;
input wire [`UPARC_REG_WIDTH-1:0]	rs_data;
input wire [`UPARC_REGNO_WIDTH-1:0]	rt;
input wire [`UPARC_REG_WIDTH-1:0]	rt_data;
/* Destination at execute stage */
input wire [`UPARC_REGNO_WIDTH-1:0]	rd_p2;
input wire [`UPARC_REG_WIDTH-1:0]	rd_data_p2;
/* Pending memory load */
input wire				pend_mem_load_p2;
/* Destination at memory stage */
input wire [`UPARC_REGNO_WIDTH-1:0]	rd_p3;
input wire [`UPARC_REG_WIDTH-1:0]	rd_data_p3;
/* Forwarded values of rs and rt */
output reg [`UPARC_REG_WIDTH-1:0]	rs_data_p1;
output reg [`UPARC_REG_WIDTH-1:0]	rt_data_p1;


always @(*)
begin
	if(rs && rs == rd_p2)
		rs_data_p1 = rd_data_p2;
	else if(rs && rs == rd_p3)
		rs_data_p1 = rd_data_p3;
	else
		rs_data_p1 = rs_data;
end


always @(*)
begin
	if(rt && rt == rd_p2 && !pend_mem_load_p2)
		rt_data_p1 = rd_data_p2;
	else if(rt && rt == rd_p3)
		rt_data_p1 = rd_data_p3;
	else
		rt_data_p1 = rt_data;
end

/*
 * Note: We cannot forward value to RT if there is pending memory load.
 * This means that the value is not valid yet and we just use old value of RT.
 * (Load delay slot)
 */

endmodule /* uparc_fwdu */
