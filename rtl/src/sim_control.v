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
 * RTL Simulation Control Device
 */

`include "common.vh"
`include "ocp_const.vh"


/*
 * Simulation control device
 *  Registers:
 *    0x00  -  control register;
 *             Bits:
 *              [31]   - used to indicate error condition on model termination;
 *              [30:1] - ignored;
 *              [0]    - stop simulation if 1 written.
 *    0x04  -  response delay register;
 *             Bits:
 *              [31:8] - ignored;
 *              [7:0]  - cycles to delay.
 *    0x08  -  bus error register (access generates bus error response).
 */
module sim_control(
	clk,
	nrst,
	/* OCP interface */
	i_MAddr,
	i_MCmd,
	i_MData,
	i_MByteEn,
	o_SCmdAccept,
	o_SData,
	o_SResp
);
/* Register offsets */
localparam [`ADDR_WIDTH-1:0] CTRLREG = 32'h000;	/* Control register */
localparam [`ADDR_WIDTH-1:0] DELYREG = 32'h004;	/* Delay register */
localparam [`ADDR_WIDTH-1:0] BUSEREG = 32'h008;	/* Bus error register */

/* Inputs and outputs */
input wire			clk;
input wire			nrst;
input wire [`ADDR_WIDTH-1:0]	i_MAddr;
input wire [2:0]		i_MCmd;
input wire [`DATA_WIDTH-1:0]	i_MData;
input wire [`BEN_WIDTH-1:0]	i_MByteEn;
output wire			o_SCmdAccept;
output reg [`DATA_WIDTH-1:0]	o_SData;
output reg [1:0]		o_SResp;


reg [`DATA_WIDTH-1:0] last_value;	/* Last written value */
reg [7:0] delay;			/* Amount of cycles to delay */
reg [7:0] counter;			/* Current delay counter */


assign o_SCmdAccept = (i_MCmd == `OCP_CMD_IDLE || counter == 8'h00) ? 1'b1 : 1'b0;


/* Ctl device logic */
always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		o_SData <= { (`DATA_WIDTH){1'b0} };
		o_SResp <= `OCP_RESP_NULL;
		delay <= 8'h00;
		counter <= 8'h00;
		last_value <= 32'h00;
	end
	else if(counter == 8'h00)
	begin
		case(i_MCmd)
		`OCP_CMD_WRITE: begin
			if(i_MAddr == CTRLREG)
			begin
				last_value <= i_MData;
				if(i_MData[0])
				begin
					if(i_MData[31])
						$display("SIMULATION TERMINATED WITH ERRORS!");
					else
						$display("SIMULATION SUCCESSFULLY TERMINATED!");
					$finish;
				end
			end
			else if(i_MAddr == DELYREG)
			begin
				delay <= i_MData[7:0];
			end

			/* Respond if no delay needed or bus error response needed */
			if(i_MAddr == BUSEREG)
				o_SResp <= `OCP_RESP_ERR;
			else if(delay == 8'h00)
				o_SResp <= `OCP_RESP_DVA;
			else
				counter <= delay;
		end
		`OCP_CMD_READ: begin
			if(i_MAddr == CTRLREG)
			begin
				o_SData <= last_value;
			end
			else if(i_MAddr == DELYREG)
			begin
				o_SData <= { 24'h0, delay };
			end
			else
				o_SData <= 32'hDEADDEAD;

			/* Respond if no delay needed or bus error response needed */
			if(i_MAddr == BUSEREG)
				o_SResp <= `OCP_RESP_ERR;
			else if(delay == 8'h00)
				o_SResp <= `OCP_RESP_DVA;
			else
				counter <= delay;
		end
		default: begin
			o_SData <= { (`DATA_WIDTH){1'b0} };
			o_SResp <= `OCP_RESP_NULL;
		end
		endcase
	end
	else if(counter == 8'h01)
	begin
		o_SResp <= `OCP_RESP_DVA;
		counter <= 8'h00;
	end
	else
		counter <= counter - 1;
end

endmodule /* sim_control */
