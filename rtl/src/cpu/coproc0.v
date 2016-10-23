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
 * Coprocessor 0
 */

`include "cpu_common.vh"
`include "cpu_const.vh"


/* Coprocessor 0 */
module coproc0(
	clk,
	nrst,
	/* CU signals */
	i_exec_stall,
	i_mem_stall,
	i_fetch_stall,
	i_drop_p1,
	i_drop_p2,
	/* Fetched instruction */
	i_instr,
	/* Decoded instr */
	o_cop0_op_p1,
	o_cop0_cop_p1,
	o_cop0_reg_no_p1,
	o_cop0_reg_val_p1,
	o_cop0_rt_no_p1,
	/* Execute stage signals */
	i_cop0_alu_result_p2
);
`include "reg_names.vh"
`include "decode_const.vh"
localparam [`CPU_INSTR_WIDTH-1:0] NOP = 32'h0000_0000;
/* Coprocessor 0 register numbers */
localparam [`CPU_REGNO_WIDTH-1:0] IVT	= 5'h0A;
localparam [`CPU_REGNO_WIDTH-1:0] PSR	= 5'h0B;
localparam [`CPU_REGNO_WIDTH-1:0] SR	= 5'h0C;
localparam [`CPU_REGNO_WIDTH-1:0] EPC	= 5'h0E;
localparam [`CPU_REGNO_WIDTH-1:0] PRID	= 5'h0F;
/* Inputs */
input wire				clk;
input wire				nrst;
/* CU signals */
input wire				i_exec_stall;
input wire				i_mem_stall;
input wire				i_fetch_stall;
input wire				i_drop_p1;
input wire				i_drop_p2;
/* Fetched instruction */
input wire [`CPU_INSTR_WIDTH-1:0]	i_instr;
/* Decoded instr */
output wire				o_cop0_op_p1;
output wire [`CPU_REGNO_WIDTH-1:0]	o_cop0_cop_p1;
output wire [`CPU_REGNO_WIDTH-1:0]	o_cop0_reg_no_p1;
output reg [`CPU_REG_WIDTH-1:0]		o_cop0_reg_val_p1;
output wire [`CPU_REGNO_WIDTH-1:0]	o_cop0_rt_no_p1;
/* Execute stage signals */
input wire [`CPU_REG_WIDTH-1:0]		i_cop0_alu_result_p2;



wire core_stall;
assign core_stall = i_exec_stall || i_mem_stall || i_fetch_stall;


/* Coprocessor 0 registers */
reg [`CPU_ADDR_WIDTH-11:0]	reg_ivt;	/* High 22 bits of IVT base (reg 0xA) */
reg				reg_psr_ie;	/* Copy of IE flag from Status register (reg 0xB) */
reg				reg_sr_ie;	/* IE flag from Status register (reg 0xC) */
reg [`CPU_ADDR_WIDTH-1:0]	reg_epc;	/* Program counter on exception entrance (reg 0xE) */
wire [`CPU_DATA_WIDTH-1:0]	reg_prid;	/* Processor ID R/O register (reg 0xF) */
assign reg_prid = `CPU_PROCID_CODE;


reg [`CPU_INSTR_WIDTH-1:0] instr;	/* Instruction word */


/******************************* DECODE STAGE *********************************/

/* Instruction fields */
wire [5:0]			op;	/* Opcode */
wire [`CPU_REGNO_WIDTH-1:0]	cop;	/* Coprocessor opcode */
wire [`CPU_REGNO_WIDTH-1:0]	rt;	/* Source register 2 */
wire [`CPU_REGNO_WIDTH-1:0]	rd;	/* Destination register */
wire [5:0]			func;	/* Function */

assign op	= instr[31:26];
assign cop	= instr[25:21];
assign rt	= instr[20:16];
assign rd	= instr[15:11];
assign func	= instr[5:0];


/* Decoded fields */
wire				cop_instr_p1;
wire [`CPU_REGNO_WIDTH-1:0]	cop_p1;
wire [`CPU_REGNO_WIDTH-1:0]	cop_rt_no_p1;
wire [`CPU_REGNO_WIDTH-1:0]	cop_reg_no_p1;
wire [5:0]			cop_func_p1;

assign cop_instr_p1 = op == `CPU_OP_COP0 ? 1'b1 : 1'b0;
assign cop_p1 = op == `CPU_OP_COP0 ? cop : {(`CPU_REGNO_WIDTH){1'b0}};
assign cop_rt_no_p1 = op == `CPU_OP_COP0 ? rt : {(`CPU_REGNO_WIDTH){1'b0}};
assign cop_reg_no_p1 = op == `CPU_OP_COP0 ? rd : {(`CPU_REGNO_WIDTH){1'b0}};
assign cop_func_p1 = op == `CPU_OP_COP0 ? func : 6'b0;


/* Decode stage outputs */
assign o_cop0_op_p1 = cop_instr_p1;
assign o_cop0_cop_p1 = cop_p1 == `CPU_COP0_MF || cop_p1 == `CPU_COP0_MT ? cop_p1 :
	{(`CPU_REGNO_WIDTH){1'b0}};
assign o_cop0_rt_no_p1 = cop_rt_no_p1;
assign o_cop0_reg_no_p1 = cop_reg_no_p1;

/* Coprocessor 0 register value */
always @(*)
begin
	o_cop0_reg_val_p1 = {(`CPU_REG_WIDTH){1'b0}};

	if(cop_instr_p1 && cop_p1 == `CPU_COP0_MF)
	begin
		case(cop_reg_no_p1)
		IVT: o_cop0_reg_val_p1 = { reg_ivt, 10'b0 };
		PSR: o_cop0_reg_val_p1 = { {(`CPU_REG_WIDTH-1){1'b0}}, reg_psr_ie };
		SR: o_cop0_reg_val_p1 = { {(`CPU_REG_WIDTH-1){1'b0}}, reg_sr_ie };
		EPC: o_cop0_reg_val_p1 = reg_epc;
		PRID: o_cop0_reg_val_p1 = reg_prid;
		default: o_cop0_reg_val_p1 = {(`CPU_REG_WIDTH){1'b0}};
		endcase
	end
end


/* Decode stage */
always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		instr <= NOP;
	end
	else if(i_drop_p1)
	begin
		instr <= NOP;
	end
	else if(!core_stall)
	begin
		instr <= i_instr;
	end
end


/****************************** EXECUTE STAGE *********************************/


reg				cop_instr_p2;
reg [`CPU_REGNO_WIDTH-1:0]	cop_p2;
reg [`CPU_REGNO_WIDTH-1:0]	cop_reg_no_p2;
reg [5:0]			cop_func_p2;


/* Execute stage */
always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		cop_instr_p2 <= 1'b0;
		cop_p2 <= {(`CPU_REGNO_WIDTH){1'b0}};
		cop_reg_no_p2 <= {(`CPU_REGNO_WIDTH){1'b0}};
		cop_func_p2 <= 6'b0;
	end
	else if(i_drop_p2)
	begin
		cop_instr_p2 <= 1'b0;
		cop_p2 <= {(`CPU_REGNO_WIDTH){1'b0}};
		cop_reg_no_p2 <= {(`CPU_REGNO_WIDTH){1'b0}};
		cop_func_p2 <= 6'b0;
	end
	else if(!core_stall)
	begin
		cop_instr_p2 <= cop_instr_p1;
		cop_p2 <= cop_p1;
		cop_reg_no_p2 <= cop_reg_no_p1;
		cop_func_p2 <= cop_func_p1;
	end
end


/******************************* MEMORY STAGE *********************************/


reg				cop_instr_p3;
reg [`CPU_REGNO_WIDTH-1:0]	cop_p3;
reg [`CPU_REGNO_WIDTH-1:0]	cop_reg_no_p3;
reg [`CPU_REG_WIDTH-1:0]	cop_reg_val_p3;
reg [5:0]			cop_func_p3;


/* Memory stage */
always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		cop_instr_p3 <= 1'b0;
		cop_p3 <= {(`CPU_REGNO_WIDTH){1'b0}};
		cop_reg_no_p3 <= {(`CPU_REGNO_WIDTH){1'b0}};
		cop_reg_val_p3 <= {(`CPU_REG_WIDTH){1'b0}};
		cop_func_p3 <= 6'b0;
	end
	else if(!core_stall)
	begin
		cop_instr_p3 <= cop_instr_p2;
		cop_p3 <= cop_p2;
		cop_reg_no_p3 <= cop_reg_no_p2;
		cop_reg_val_p3 <= i_cop0_alu_result_p2;
		cop_func_p3 <= cop_func_p2;
	end
end


/***************************** WRITEBACK STAGE ********************************/


/* Writeback stage */
always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		reg_ivt <= {(`CPU_ADDR_WIDTH-10){1'b0}};
		reg_psr_ie <= 1'b0;
		reg_sr_ie <= 1'b0;
		reg_epc <= {(`CPU_ADDR_WIDTH){1'b0}};
	end
	else if(!core_stall && cop_instr_p3)
	begin
		if(cop_p3 == `CPU_COP0_CO)
		begin
			case(cop_func_p3)
			`CPU_COP0_FUNC_RFE: begin
				reg_sr_ie <= reg_psr_ie;
				reg_psr_ie <= 1'b0;
			end
			endcase
		end
		else if(cop_p3 == `CPU_COP0_MT)
		begin
			case(cop_reg_no_p3)
			IVT: reg_ivt <= cop_reg_val_p3[`CPU_REG_WIDTH-1:10];
			PSR: reg_psr_ie <= cop_reg_val_p3[0];
			SR: reg_sr_ie <= cop_reg_val_p3[0];
			EPC: reg_epc <= cop_reg_val_p3;
			endcase
		end
	end
end


endmodule /* coproc0 */
