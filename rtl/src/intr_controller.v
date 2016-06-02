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
 * Programmable interrupt controller
 */

`include "common.vh"
`include "ocp_const.vh"


/*
 * Programmable interrupt controller
 *  Registers:
 *    0x00  -  interrupt status register;
 *             Bits:
 *              [31:0] - set bits denotes active unmasked interrupt lines.
 *             Write to this register acknowledges specified interrupts.
 *    0x04  -  interrupt mask register;
 *             Bits:
 *              [31:0] - set bits correspond to unmasked interrupts.
 *    0x08  -  raw interrupts register (read only register);
 *             Bits:
 *              [31:0] - set bits correspond to active interrupt lines.
 */
module intr_controller(
	clk,
	nrst,
	/* Interrupts input */
	i_intr_vec,
	/* Interrupt output */
	o_intr,
	/* OCP interface */
	i_MAddr,
	i_MCmd,
	i_MData,
	i_MByteEn,
	o_SCmdAccept,
	o_SData,
	o_SResp
);
/* Bus interface FSM states */
localparam [2:0] IDLE  = 3'b001;
localparam [2:0] WRITE = 3'b010;
localparam [2:0] READ  = 3'b100;

/* Register offsets */
localparam [`ADDR_WIDTH-1:0] ISTATREG = 32'h000;	/* Interrupts status register */
localparam [`ADDR_WIDTH-1:0] IMASKREG = 32'h004;	/* Interrupts mask register */
localparam [`ADDR_WIDTH-1:0] IRAWREG  = 32'h008;	/* Raw interrupts register */

/* Inputs and outputs */
input wire			clk;
input wire			nrst;
input wire [31:0]		i_intr_vec;
output wire			o_intr;
input wire [`ADDR_WIDTH-1:0]	i_MAddr;
input wire [2:0]		i_MCmd;
input wire [`DATA_WIDTH-1:0]	i_MData;
input wire [`BEN_WIDTH-1:0]	i_MByteEn;
output wire			o_SCmdAccept;
output reg [`DATA_WIDTH-1:0]	o_SData;
output reg [1:0]		o_SResp;

/* Internal registers */
reg [31:0] int_mask;	/* Interrupt mask */
reg [31:0] raw_int;	/* Raw interrupts */

/* Latched address and data */
reg [`ADDR_WIDTH-1:0] addr;
reg [`DATA_WIDTH-1:0] wdata;

/* Bus FSM state */
reg [2:0] bus_state;
reg [2:0] bus_next_state;

reg iack;	/* Interrupt acknowledge */


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
		int_mask <= 32'b0;
		iack <= 1'b0;
	end
	else
	begin
		/* Interrut acknowledge happens on write to interrupt status  */
		iack <= (bus_state == WRITE && addr == ISTATREG) ? 1'b1 : 1'b0;

		case(bus_state)
		WRITE: begin
			if(addr == IMASKREG)
			begin
				int_mask <= wdata;
			end
			o_SResp <= `OCP_RESP_DVA;
		end
		READ: begin
			if(addr == ISTATREG)
			begin
				o_SData <= { {(`DATA_WIDTH-32){1'b0}},
					int_mask & raw_int };
			end
			else if(addr == IMASKREG)
			begin
				o_SData <= int_mask;
			end
			else if(addr == IRAWREG)
			begin
				o_SData <= raw_int;
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

assign o_intr = |(int_mask & raw_int);

/* Accept and acknowledge interrupts */
always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		raw_int <= 32'b0;
	end
	else
	begin
		raw_int <= (iack ? raw_int & ~wdata : raw_int) | i_intr_vec;
	end
end

endmodule /* intr_controller */
