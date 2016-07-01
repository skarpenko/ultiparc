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
 * Fabric port FSM
 */

`include "common.vh"
`include "ocp_const.vh"


/*
 * Fabric port FSM
 */
module fabric_port #(
	parameter [10:0] PORT = 0	/* Port number */
)
(
	clk,
	nrst,
	/* OCP interface: instructions (master) */
	i_I_MAddr,
	i_I_MCmd,
	i_I_MData,
	i_I_MByteEn,
	o_I_SCmdAccept,
	o_I_SData,
	o_I_SResp,
	/* OCP interface: data (master) */
	i_D_MAddr,
	i_D_MCmd,
	i_D_MData,
	i_D_MByteEn,
	o_D_SCmdAccept,
	o_D_SData,
	o_D_SResp,
	/* OCP interface: Port (slave) */
	o_P_MAddr,
	o_P_MCmd,
	o_P_MData,
	o_P_MByteEn,
	i_P_SCmdAccept,
	i_P_SData,
	i_P_SResp
);
localparam PORT_BITS = 12;

input wire			clk;
input wire			nrst;
/* OCP interface: instructions */
input wire [`ADDR_WIDTH-1:0]	i_I_MAddr;
input wire [2:0]		i_I_MCmd;
input wire [`DATA_WIDTH-1:0]	i_I_MData;
input wire [`BEN_WIDTH-1:0]	i_I_MByteEn;
output wire			o_I_SCmdAccept;
output wire [`DATA_WIDTH-1:0]	o_I_SData;
output wire [1:0]		o_I_SResp;
/* OCP interface: data */
input wire [`ADDR_WIDTH-1:0]	i_D_MAddr;
input wire [2:0]		i_D_MCmd;
input wire [`DATA_WIDTH-1:0]	i_D_MData;
input wire [`BEN_WIDTH-1:0]	i_D_MByteEn;
output wire			o_D_SCmdAccept;
output wire [`DATA_WIDTH-1:0]	o_D_SData;
output wire [1:0]		o_D_SResp;
/* OCP interface: Port */
output wire [`ADDR_WIDTH-1:0]	o_P_MAddr;
output wire [2:0]		o_P_MCmd;
output wire [`DATA_WIDTH-1:0]	o_P_MData;
output wire [`BEN_WIDTH-1:0]	o_P_MByteEn;
input wire			i_P_SCmdAccept;
input wire [`DATA_WIDTH-1:0]	i_P_SData;
input wire [1:0]		i_P_SResp;


/* Bus interface FSM states */
localparam [2:0] BUS_IDLE = 3'b001;	/* Idle state */
localparam [2:0] BUS_WAIT = 3'b010;	/* Wait for slave response */
localparam [2:0] BUS_CONF = 3'b100;	/* Wait for conflict resolution */


/* Bus FSM state registers */
reg [2:0] i_bus_state;
reg [2:0] d_bus_state;


/* Command accept signals */
assign o_I_SCmdAccept = ((i_bus_state == BUS_IDLE || i_I_MCmd == `OCP_CMD_IDLE) ? 1'b1 : 1'b0);
assign o_D_SCmdAccept = ((d_bus_state == BUS_IDLE || i_D_MCmd == `OCP_CMD_IDLE) ? 1'b1 : 1'b0);


/* Return port number */
function [PORT_BITS-2:0] port_no;
	input [`ADDR_WIDTH-1:0] addr;
	port_no = (addr[`ADDR_WIDTH-1] ? addr[`ADDR_WIDTH-2:`ADDR_WIDTH-PORT_BITS] + 1 : 0);
endfunction


/* Decode address */
function [`ADDR_WIDTH-1:0] decode_addr;
	input [`ADDR_WIDTH-1:0] addr;
	decode_addr = (!addr[`ADDR_WIDTH-1] ? { 1'b0, addr[`ADDR_WIDTH-2:0] } :
		{ {(PORT_BITS){1'b0}}, addr[`ADDR_WIDTH-PORT_BITS-1:0] });
endfunction


/* Check for address conflict */
function confl;
	input [`ADDR_WIDTH-1:0] i_addr;
	input [2:0] i_cmd;
	input [`ADDR_WIDTH-1:0] d_addr;
	input [2:0] d_cmd;
	confl = (port_no(i_addr) == port_no(d_addr) &&
		i_cmd != `OCP_CMD_IDLE && d_cmd != `OCP_CMD_IDLE);
endfunction


/* Internal instructions port connections */
reg [`ADDR_WIDTH-1:0]		I_MAddr;
reg [2:0]			I_MCmd;
reg [`DATA_WIDTH-1:0]		I_MData;
reg [`BEN_WIDTH-1:0]		I_MByteEn;
wire [`DATA_WIDTH-1:0]		I_SData;
wire [1:0]			I_SResp;
wire				I_SCmdAccept;


/* Internal data port connections */
reg [`ADDR_WIDTH-1:0]		D_MAddr;
reg [2:0]			D_MCmd;
reg [`DATA_WIDTH-1:0]		D_MData;
reg [`BEN_WIDTH-1:0]		D_MByteEn;
wire [`DATA_WIDTH-1:0]		D_SData;
wire [1:0]			D_SResp;
wire				D_SCmdAccept;


/* Output */
assign o_I_SData = I_SData;
assign o_I_SResp = I_SResp;
assign o_D_SData = D_SData;
assign o_D_SResp = D_SResp;


reg i_busy;	/* Instruction fetch on the fly */
reg d_busy;	/* Data read/write on the fly */


wire select;
assign select = (d_busy ? 1'b0 : 1'b1);	/* Select port */


/* Latched instruction request */
reg [`ADDR_WIDTH-1:0]		i_addr;
reg [2:0]			i_cmd;
reg [`DATA_WIDTH-1:0]		i_data;
reg [`BEN_WIDTH-1:0]		i_ben;


/* Latched data request */
reg [`ADDR_WIDTH-1:0]		d_addr;
reg [2:0]			d_cmd;
reg [`DATA_WIDTH-1:0]		d_data;
reg [`BEN_WIDTH-1:0]		d_ben;


/* Instructions port FSM  */
always @(posedge clk or negedge nrst)
begin : instructions_port_fsm
	if(!nrst)
	begin
		I_MAddr <= {(`ADDR_WIDTH){1'b0}};
		I_MCmd <= 3'b0;
		I_MData <= {(`DATA_WIDTH){1'b0}};
		I_MByteEn <= {(`BEN_WIDTH){1'b0}};
		i_addr <= {(`ADDR_WIDTH){1'b0}};
		i_cmd <= 3'b0;
		i_data <= {(`DATA_WIDTH){1'b0}};
		i_ben <= {(`BEN_WIDTH){1'b0}};
		i_busy <= 1'b0;
		i_bus_state <= BUS_IDLE;
	end
	else
	begin
		case(i_bus_state)
		BUS_IDLE: begin
			if(i_I_MCmd != `OCP_CMD_IDLE && port_no(i_I_MAddr) == PORT)
			begin
				if(confl(i_I_MAddr, i_I_MCmd, i_D_MAddr, i_D_MCmd) || d_busy)
				begin
					i_addr <= i_I_MAddr;
					i_cmd <= i_I_MCmd;
					i_data <= i_I_MData;
					i_ben <= i_I_MByteEn;
					i_bus_state <= BUS_CONF;
				end
				else
				begin
					I_MAddr <= decode_addr(i_I_MAddr);
					I_MCmd <= i_I_MCmd;
					I_MData <= i_I_MData;
					I_MByteEn <= i_I_MByteEn;
					i_busy <= 1'b1;
					i_bus_state <= BUS_WAIT;
				end
			end
		end
		BUS_WAIT: begin
			if(I_SCmdAccept != 1'b0)
			begin
				I_MCmd <= `OCP_CMD_IDLE;
				if(I_SResp != `OCP_RESP_NULL)
				begin
					i_busy <= 1'b0;
					i_bus_state <= BUS_IDLE;
				end
			end
		end
		BUS_CONF: begin
			if(!confl(i_addr, i_cmd, i_D_MAddr, i_D_MCmd) && !d_busy)
			begin
				I_MAddr <= decode_addr(i_addr);
				I_MCmd <= i_cmd;
				I_MData <= i_data;
				I_MByteEn <= i_ben;
				i_busy <= 1'b1;
				i_bus_state <= BUS_WAIT;
			end
		end
		endcase
	end
end


/* Data port FSM  */
always @(posedge clk or negedge nrst)
begin : data_port_fsm
	if(!nrst)
	begin
		D_MAddr <= {(`ADDR_WIDTH){1'b0}};
		D_MCmd <= 3'b0;
		D_MData <= {(`DATA_WIDTH){1'b0}};
		D_MByteEn <= {(`BEN_WIDTH){1'b0}};
		d_addr <= {(`ADDR_WIDTH){1'b0}};
		d_cmd <= 3'b0;
		d_data <= {(`DATA_WIDTH){1'b0}};
		d_ben <= {(`BEN_WIDTH){1'b0}};
		d_busy <= 1'b0;
		d_bus_state <= BUS_IDLE;
	end
	else
	begin
		case(d_bus_state)
		BUS_IDLE: begin
			if(i_D_MCmd != `OCP_CMD_IDLE && port_no(i_D_MAddr) == PORT)
			begin
				if(i_busy)
				begin
					d_addr <= i_D_MAddr;
					d_cmd <= i_D_MCmd;
					d_data <= i_D_MData;
					d_ben <= i_D_MByteEn;
					d_bus_state <= BUS_CONF;
				end
				else
				begin
					D_MAddr <= decode_addr(i_D_MAddr);
					D_MCmd <= i_D_MCmd;
					D_MData <= i_D_MData;
					D_MByteEn <= i_D_MByteEn;
					d_busy <= 1'b1;
					d_bus_state <= BUS_WAIT;
				end
			end
		end
		BUS_WAIT: begin
			if(D_SCmdAccept != 1'b0)
			begin
				D_MCmd <= `OCP_CMD_IDLE;
				if(D_SResp != `OCP_RESP_NULL)
				begin
					d_busy <= 1'b0;
					d_bus_state <= BUS_IDLE;
				end
			end
		end
		BUS_CONF: begin
			if(!i_busy)
			begin
				D_MAddr <= decode_addr(d_addr);
				D_MCmd <= d_cmd;
				D_MData <= d_data;
				D_MByteEn <= d_ben;
				d_busy <= 1'b1;
				d_bus_state <= BUS_WAIT;
			end
		end
		endcase
	end
end


/* Ports demultiplexer */
fabric_port_demux demux(
	.i_select(select),
	.i_I_MAddr(I_MAddr),
	.i_I_MCmd(I_MCmd),
	.i_I_MData(I_MData),
	.i_I_MByteEn(I_MByteEn),
	.o_I_SCmdAccept(I_SCmdAccept),
	.o_I_SData(I_SData),
	.o_I_SResp(I_SResp),
	.i_D_MAddr(D_MAddr),
	.i_D_MCmd(D_MCmd),
	.i_D_MData(D_MData),
	.i_D_MByteEn(D_MByteEn),
	.o_D_SCmdAccept(D_SCmdAccept),
	.o_D_SData(D_SData),
	.o_D_SResp(D_SResp),
	.o_P_MAddr(o_P_MAddr),
	.o_P_MCmd(o_P_MCmd),
	.o_P_MData(o_P_MData),
	.o_P_MByteEn(o_P_MByteEn),
	.i_P_SCmdAccept(i_P_SCmdAccept),
	.i_P_SData(i_P_SData),
	.i_P_SResp(i_P_SResp)
);


endmodule /* fabric_port */
