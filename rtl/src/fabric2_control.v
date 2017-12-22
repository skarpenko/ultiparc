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
 * System fabric (version 2)
 */

`include "common.vh"
`include "ocp_const.vh"


/* Arbitration and control */
module fabric2_control #(
	parameter PORTNO_WIDTH = 11
)
(
	clk,
	nrst,
	/* Active and completed transactions */
	i_I_act,
	i_I_done,
	i_D_act,
	i_D_done,
	/* Destination ports */
	i_I_portno,
	i_D_portno,
	/* Master ports switch control */
	o_I_mswitch,
	o_D_mswitch,
	/* Slave ports switch control */
	o_p0_sswitch,
	o_p1_sswitch,
	o_p2_sswitch,
	o_p3_sswitch,
	o_p4_sswitch
);
input wire			clk;
input wire			nrst;
/* Active transactions */
input wire			i_I_act;
input wire			i_I_done;
input wire			i_D_act;
input wire			i_D_done;
/* Destination ports */
input wire [PORTNO_WIDTH-1:0]	i_I_portno;
input wire [PORTNO_WIDTH-1:0]	i_D_portno;
/* Master ports switch control */
output wire [PORTNO_WIDTH-1:0]	o_I_mswitch;
output wire [PORTNO_WIDTH-1:0]	o_D_mswitch;
/* Slave ports switch control */
output wire			o_p0_sswitch;
output wire			o_p1_sswitch;
output wire			o_p2_sswitch;
output wire			o_p3_sswitch;
output wire			o_p4_sswitch;


/** Internal wires and registers **/

reg			instr_act_r;
reg			data_act_r;
reg [PORTNO_WIDTH-1:0]	i_portno_r;
reg [PORTNO_WIDTH-1:0]	d_portno_r;
wire			instr_act = i_I_act || instr_act_r;
wire			data_act = i_D_act || data_act_r;
wire [PORTNO_WIDTH-1:0]	i_portno = i_I_act ? i_I_portno : i_portno_r;
wire [PORTNO_WIDTH-1:0]	d_portno = i_D_act ? i_D_portno : d_portno_r;

reg sw;


always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		instr_act_r <= 1'b0;
		data_act_r <= 1'b0;
		i_portno_r <= { (PORTNO_WIDTH){1'b0} };
		d_portno_r <= { (PORTNO_WIDTH){1'b0} };
		sw <= 1'b0;
	end
	else
	begin
		instr_act_r <= ((i_I_act || instr_act_r) && !i_I_done ? 1'b1 : 1'b0);
		data_act_r <= ((i_D_act || data_act_r) && !i_D_done ? 1'b1 : 1'b0);

		if(i_I_act)
			i_portno_r <= i_I_portno;

		if(i_D_act)
			d_portno_r <= i_D_portno;

		if(instr_act && !confl)
			sw <= 1'b0;

		if(data_act && !confl)
			sw <= 1'b1;
	end
end


/* Conflict */
wire confl = (i_portno == d_portno && instr_act && data_act);


/* Slave ports switch control logic */

assign o_p0_sswitch = !confl ? ((instr_act && i_portno == 'd0) ? 1'b0 : 1'b1) : sw;
assign o_p1_sswitch = !confl ? ((instr_act && i_portno == 'd1) ? 1'b0 : 1'b1) : sw;
assign o_p2_sswitch = !confl ? ((instr_act && i_portno == 'd2) ? 1'b0 : 1'b1) : sw;
assign o_p3_sswitch = !confl ? ((instr_act && i_portno == 'd3) ? 1'b0 : 1'b1) : sw;
assign o_p4_sswitch = !confl ? ((instr_act && i_portno == 'd4) ? 1'b0 : 1'b1) : sw;

/*
assign o_p0_sswitch = (!data_act_r && (confl || (instr_act && i_portno == 'd0)) ? 1'b0 : 1'b1);
assign o_p1_sswitch = (!data_act_r && (confl || (instr_act && i_portno == 'd1)) ? 1'b0 : 1'b1);
assign o_p2_sswitch = (!data_act_r && (confl || (instr_act && i_portno == 'd2)) ? 1'b0 : 1'b1);
assign o_p3_sswitch = (!data_act_r && (confl || (instr_act && i_portno == 'd3)) ? 1'b0 : 1'b1);
assign o_p4_sswitch = (!data_act_r && (confl || (instr_act && i_portno == 'd4)) ? 1'b0 : 1'b1);
*/

/*
assign o_p0_sswitch = ((confl || (instr_act && i_portno == 'd0)) ? 1'b0 : 1'b1);
assign o_p1_sswitch = ((confl || (instr_act && i_portno == 'd1)) ? 1'b0 : 1'b1);
assign o_p2_sswitch = ((confl || (instr_act && i_portno == 'd2)) ? 1'b0 : 1'b1);
assign o_p3_sswitch = ((confl || (instr_act && i_portno == 'd3)) ? 1'b0 : 1'b1);
assign o_p4_sswitch = ((confl || (instr_act && i_portno == 'd4)) ? 1'b0 : 1'b1);
*/


/* Master ports switch control logic */
assign o_I_mswitch = i_portno;
assign o_D_mswitch = d_portno;


endmodule /* fabric2_control */
