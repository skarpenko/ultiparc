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
 * Micro UART
 */

`include "common.vh"
`include "ocp_const.vh"


/*
 * Micro UART
 *  Registers:
 *    0x00  -  character register;
 *             Bits:
 *              [31:8] - ignored;
 *              [7:0]  - character value to print.
 */
module micro_uart(
	input				clk,
	input				nrst,
	/* OCP interface */
	input [`ADDR_WIDTH-1:0]		i_MAddr,
	input [2:0]			i_MCmd,
	input [`DATA_WIDTH-1:0]		i_MData,
	input [`BEN_WIDTH-1:0]		i_MByteEn,
	output				o_SCmdAccept,
	output [`DATA_WIDTH-1:0]	o_SData,
	output [1:0]			o_SResp
);
/* Bus interface FSM states */
localparam [2:0] IDLE  = 3'b001;
localparam [2:0] WRITE = 3'b010;
localparam [2:0] READ  = 3'b100;

/* Register offsets */
localparam [`ADDR_WIDTH-1:0] CHARREG = 32'h000;	/* Character register */

/* Inputs and outputs */
wire			clk;
wire			nrst;
wire [`ADDR_WIDTH-1:0]	i_MAddr;
wire [2:0]		i_MCmd;
wire [`DATA_WIDTH-1:0]	i_MData;
wire [`BEN_WIDTH-1:0]	i_MByteEn;
reg			o_SCmdAccept;
reg [`DATA_WIDTH-1:0]	o_SData;
reg [1:0]		o_SResp;


/* Latched address */
reg [`ADDR_WIDTH-1:0] l_addr;
/* Latched write data */
reg [`DATA_WIDTH-1:0] l_wdata;

/* Bus FSM state */
reg [2:0] bus_state;
reg [2:0] bus_next_state;


/* Seq logic */
always @(posedge clk or negedge nrst)
	bus_state <= nrst ? bus_next_state : IDLE;

/* Next state logic */
always @(bus_state or i_MCmd)
begin
	if(bus_state == IDLE)
	begin
		o_SCmdAccept <= 1'b1;
		case(i_MCmd)
		`OCP_CMD_WRITE: begin
			l_addr <= i_MAddr;
			l_wdata <= i_MData;
			bus_next_state <= WRITE;
		end
		`OCP_CMD_READ: begin
			l_addr <= i_MAddr;
			bus_next_state <= READ;
		end
		default: begin
			bus_next_state <= IDLE;
		end
		endcase
	end
	else
	begin
		o_SCmdAccept <= (i_MCmd == `OCP_CMD_IDLE) ? 1'b1 : 1'b0;
		bus_next_state <= IDLE;
	end
end

/* Output logic */
always @(bus_state or negedge nrst)
begin
	if(nrst)
	begin
		case(bus_state)
		WRITE: begin
			if(l_addr == CHARREG)
			begin
				$write("%c", l_wdata[7:0]);
			end
			o_SResp <= `OCP_RESP_DVA;
		end
		READ: begin
			if(l_addr == CHARREG)
			begin
				/* Ignored. Always 0. */
				o_SData <= { (`DATA_WIDTH){1'b0} };
			end
			else
				o_SData <= 32'hDEADDEAD;
			o_SResp <= `OCP_RESP_DVA;
		end
		default: begin
			o_SResp <= `OCP_RESP_NULL;
		end
		endcase
	end
	else
	begin
		o_SData <= { (`DATA_WIDTH){1'b0} };
		o_SResp <= `OCP_RESP_NULL;
	end
end

endmodule /* micro_uart */
