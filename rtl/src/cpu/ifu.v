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
 * Instruction fetch unit
 */

`include "cpu_common.vh"
`include "cpu_const.vh"


/* IFU */
module ifu(
	clk,
	nrst,
	/* Internal signals */
	addr,
	instr_dat,
	rd_cmd,
	busy,
	err_align,
	err_bus,
	/* I-Bus */
	o_IAddr,
	o_IRdC,
	i_IData,
	i_IRdy,
	i_IErr
);
/* I-Bus FSM states */
localparam READY = 1'b0;	/* Ready to accept response */
localparam WAIT  = 1'b1;	/* Wait for response */

/* Inputs */
input wire				clk;
input wire				nrst;
/* Internal CPU interface */
input wire [`CPU_ADDR_WIDTH-1:0]	addr;
output reg [`CPU_INSTR_WIDTH-1:0]	instr_dat;
input wire				rd_cmd;
output wire				busy;
output wire				err_align;
output reg				err_bus;
/* I-Bus interface */
output reg [`CPU_ADDR_WIDTH-1:0]	o_IAddr;
output reg				o_IRdC;
input wire [`CPU_INSTR_WIDTH-1:0]	i_IData;
input wire				i_IRdy;
input wire				i_IErr;


assign err_align = (rd_cmd == 1'b1 && addr[1:0] != 2'b0);
assign busy = (!err_align && rd_cmd) || state;


reg state;	/* Instruction fetch FSM state */


always @(*)
begin
	o_IAddr = 32'b0;
	o_IRdC = 1'b0;

	/* Issue command to I-Bus if no error */
	if(!err_align && rd_cmd == 1'b1)
	begin
		o_IAddr = addr;
		o_IRdC = 1'b1;
	end
end


/* I-Bus response wait FSM */
always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		state <= READY;
		instr_dat <= 32'b0;
		err_bus <= 1'b0;
	end
	else
	begin
		err_bus <= 1'b0;

		/*
		 * Set response if available on current clock,
		 * otherwise switch to wait state.
		 */
		if(state == READY && !err_align && rd_cmd)
		begin
			if(i_IErr)
				err_bus <= 1'b1;
			else if(i_IRdy)
				instr_dat <= i_IData;
			else
				state <= WAIT;
		end
		else if(state == WAIT)
		begin
			if(i_IErr)
			begin
				err_bus <= 1'b1;
				state <= 1'b0;
			end
			else if(i_IRdy)
			begin
				instr_dat <= i_IData;
				state <= READY;
			end
		end
	end
end

endmodule /* ifu */
