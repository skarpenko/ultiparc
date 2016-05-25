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
 * Programmable interval timer
 */

`include "common.vh"
`include "ocp_const.vh"


/*
 * Programmable interval timer
 *  Registers:
 *    0x00  -  control register;
 *             Bits:
 *              [31:3] - ignored;
 *              [2]    - reload value (1);
 *              [1]    - mask (0) or unmask (1) interrupt;
 *              [0]    - enable (1) and disable (0) timer.
 *    0x04  -  counter register;
 *    0x08  -  current count register (read only).
 */
module interval_timer(
	input				clk,
	input				nrst,
	/* Interrupt output */
	output				o_intr,
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

/* Timer counter states */
localparam [2:0] STOP   = 3'b001;
localparam [2:0] RELOAD = 3'b010;
localparam [2:0] COUNT  = 3'b100;

/* Register offsets */
localparam [`ADDR_WIDTH-1:0] CTRLREG = 32'h000;	/* Control register */
localparam [`ADDR_WIDTH-1:0] CNTRREG = 32'h004;	/* Counter register */
localparam [`ADDR_WIDTH-1:0] CURRREG = 32'h008;	/* Current counter */

/* Inputs and outputs */
wire			clk;
wire			nrst;
reg			o_intr;
wire [`ADDR_WIDTH-1:0]	i_MAddr;
wire [2:0]		i_MCmd;
wire [`DATA_WIDTH-1:0]	i_MData;
wire [`BEN_WIDTH-1:0]	i_MByteEn;
wire			o_SCmdAccept;
reg [`DATA_WIDTH-1:0]	o_SData;
reg [1:0]		o_SResp;

/* Internal registers */
reg [`DATA_WIDTH-1:0] initval;	/* Initial value */
reg [`DATA_WIDTH-1:0] currval;	/* Current value */
reg enable;			/* Timer enabled */
reg imask;			/* Interrupt mask */
reg reload;			/* Reload counter automatically */

/* Latched address and data */
reg [`ADDR_WIDTH-1:0] addr;
reg [`DATA_WIDTH-1:0] wdata;

/* Bus FSM state */
reg [2:0] bus_state;
reg [2:0] bus_next_state;

/* Counter FSM state */
reg [2:0] ctr_state;
reg reload_en;		/* Force counter reload */


assign o_SCmdAccept = (i_MCmd == `OCP_CMD_IDLE || bus_state == IDLE) ? 1'b1 : 1'b0;


/* Latch address and data */
always @(posedge clk)
begin
	addr <= i_MAddr;
	wdata <= i_MData;
end


/* Seq logic */
always @(posedge clk or negedge nrst)
	bus_state <= nrst ? bus_next_state : IDLE;


/* Next state logic */
always @(*)
begin
	bus_next_state = IDLE;

	if(bus_state == IDLE)
	begin
		case(i_MCmd)
		`OCP_CMD_WRITE: bus_next_state = WRITE;
		`OCP_CMD_READ: bus_next_state = READ;
		default: bus_next_state = IDLE;
		endcase
	end
end


/* Output logic */
always @(bus_state or negedge nrst)
begin
	if(!nrst)
	begin
		o_SData <= {(`DATA_WIDTH){1'b0}};
		o_SResp <= `OCP_RESP_NULL;
		enable <= 1'b0;
		imask <= 1'b0;
		initval <= {(`DATA_WIDTH){1'b0}};
		reload <= 1'b0;
		reload_en <= 1'b0;
	end
	else
	begin
		/* Force reload if updating initial count value */
		reload_en <= (bus_state == WRITE && addr == CNTRREG) ? 1'b1 : 1'b0;

		case(bus_state)
		WRITE: begin
			if(addr == CTRLREG)
			begin
				enable <= wdata[0];
				imask <= wdata[1];
				reload <= wdata[2];
			end
			else if(addr == CNTRREG)
			begin
				initval <= wdata;
			end
			o_SResp <= `OCP_RESP_DVA;
		end
		READ: begin
			if(addr == CTRLREG)
			begin
				o_SData <= { {(`DATA_WIDTH-3){1'b0}},
					reload, imask, enable };
			end
			else if(addr == CNTRREG)
			begin
				o_SData <= initval;
			end
			else if(addr == CURRREG)
			begin
				o_SData <= currval;
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
end


/* Counter FSM */
always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		currval <= 0;
		o_intr <= 1'b0;
		ctr_state <= STOP;
	end
	else if(reload_en)
	begin
		/* Update current value and restart timer */
		currval <= initval;
		ctr_state <= STOP;
	end
	else
	begin
		case(ctr_state)
		STOP: begin
			o_intr <= 1'b0;
			if(enable)
				ctr_state <= COUNT;
		end
		COUNT: begin
			if(currval == 0)
				o_intr <= 1'b1 & imask; /* Set interrupt */
			else
				currval <= currval - 1;

			/* After zero reached next state depends on reload flag */
			ctr_state <= enable ?
				(currval == 0 ?
				(reload ? RELOAD : STOP) : COUNT) : STOP;
		end
		RELOAD: begin
			o_intr <= 1'b0;
			currval <= initval - 1;
			ctr_state <= enable ? COUNT : STOP;
		end
		default: ctr_state <= STOP;
		endcase
	end
end

endmodule /* interval_timer */
