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
 * Coprocessor 0
 */

`include "uparc_cpu_common.vh"
`include "uparc_cpu_const.vh"


/* Coprocessor 0 */
module uparc_coproc0(
	clk,
	nrst,
	/* CU signals */
	i_exec_stall,
	i_mem_stall,
	i_fetch_stall,
	o_decode_error,
	i_except_start,
	i_except_dly_slt,
	i_except_raddr,
	i_except_raddr_dly,
	i_nullify_decode,
	i_nullify_execute,
	i_nullify_mem,
	i_nullify_wb,
	/* COP0 signals */
	o_cop0_ivtbase,
	o_cop0_ie,
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
`include "uparc_reg_names.vh"
`include "uparc_decode_const.vh"
localparam [`UPARC_INSTR_WIDTH-1:0] NOP		= 32'h0000_0000;
/* Coprocessor 0 register numbers */
localparam [`UPARC_REGNO_WIDTH-1:0] TSCLO	= 5'h08;
localparam [`UPARC_REGNO_WIDTH-1:0] TSCHI	= 5'h09;
localparam [`UPARC_REGNO_WIDTH-1:0] IVT		= 5'h0A;
localparam [`UPARC_REGNO_WIDTH-1:0] PSR		= 5'h0B;
localparam [`UPARC_REGNO_WIDTH-1:0] SR		= 5'h0C;
localparam [`UPARC_REGNO_WIDTH-1:0] CAUSE	= 5'h0D;
localparam [`UPARC_REGNO_WIDTH-1:0] EPC		= 5'h0E;
localparam [`UPARC_REGNO_WIDTH-1:0] PRID	= 5'h0F;
/* Inputs */
input wire				clk;
input wire				nrst;
/* CU signals */
input wire				i_exec_stall;
input wire				i_mem_stall;
input wire				i_fetch_stall;
output reg				o_decode_error;
input wire				i_except_start;
input wire				i_except_dly_slt;
input wire [`UPARC_ADDR_WIDTH-1:0]	i_except_raddr;
input wire [`UPARC_ADDR_WIDTH-1:0]	i_except_raddr_dly;
input wire				i_nullify_decode;
input wire				i_nullify_execute;
input wire				i_nullify_mem;
input wire				i_nullify_wb;
/* COP0 signals */
output wire [`UPARC_ADDR_WIDTH-11:0]	o_cop0_ivtbase;
output wire				o_cop0_ie;
/* Fetched instruction */
input wire [`UPARC_INSTR_WIDTH-1:0]	i_instr;
/* Decoded instr */
output wire				o_cop0_op_p1;
output wire [`UPARC_REGNO_WIDTH-1:0]	o_cop0_cop_p1;
output wire [`UPARC_REGNO_WIDTH-1:0]	o_cop0_reg_no_p1;
output reg [`UPARC_REG_WIDTH-1:0]	o_cop0_reg_val_p1;
output wire [`UPARC_REGNO_WIDTH-1:0]	o_cop0_rt_no_p1;
/* Execute stage signals */
input wire [`UPARC_REG_WIDTH-1:0]	i_cop0_alu_result_p2;


wire core_stall = i_exec_stall || i_mem_stall || i_fetch_stall;


assign o_cop0_ivtbase = reg_ivt;
assign o_cop0_ie = reg_sr_ie;


/* Coprocessor 0 registers */
reg [`UPARC_ADDR_WIDTH-11:0]	reg_ivt;	/* High 22 bits of IVT base (reg 0xA) */
reg				reg_psr_ie;	/* Copy of IE flag from Status register (reg 0xB) */
reg				reg_sr_ie;	/* IE flag from Status register (reg 0xC) */
reg				reg_cause_bd;	/* BD flag from Cause register (reg 0xD) */
reg [`UPARC_ADDR_WIDTH-1:0]	reg_epc;	/* Program counter on exception entrance (reg 0xE) */
wire [`UPARC_DATA_WIDTH-1:0]	reg_prid;	/* Processor ID R/O register (reg 0xF) */
assign reg_prid = `UPARC_PROCID_CODE;


reg [`UPARC_INSTR_WIDTH-1:0] instr;	/* Instruction word */


/******************************* DECODE STAGE *********************************/

/* Instruction fields */
wire [5:0]			op;	/* Opcode */
wire [`UPARC_REGNO_WIDTH-1:0]	cop;	/* Coprocessor opcode */
wire [`UPARC_REGNO_WIDTH-1:0]	rt;	/* Source register 2 */
wire [`UPARC_REGNO_WIDTH-1:0]	rd;	/* Destination register */
wire [4:0]			rsvd;	/* Reserved field */
wire [5:0]			func;	/* Function */

assign op	= instr[31:26];
assign cop	= instr[25:21];
assign rt	= instr[20:16];
assign rd	= instr[15:11];
assign rsvd	= instr[10:6];
assign func	= instr[5:0];


/* Decoded fields */
wire				cop_instr_p1;
wire [`UPARC_REGNO_WIDTH-1:0]	cop_p1;
wire [`UPARC_REGNO_WIDTH-1:0]	cop_rt_no_p1;
wire [`UPARC_REGNO_WIDTH-1:0]	cop_reg_no_p1;
wire [5:0]			cop_func_p1;

assign cop_instr_p1 = op == `UPARC_OP_COP0 ? 1'b1 : 1'b0;
assign cop_p1 = op == `UPARC_OP_COP0 ? cop : {(`UPARC_REGNO_WIDTH){1'b0}};
assign cop_rt_no_p1 = op == `UPARC_OP_COP0 ? rt : {(`UPARC_REGNO_WIDTH){1'b0}};
assign cop_reg_no_p1 = op == `UPARC_OP_COP0 ? rd : {(`UPARC_REGNO_WIDTH){1'b0}};
assign cop_func_p1 = op == `UPARC_OP_COP0 ? func : 6'b0;


/* Decode stage outputs */
assign o_cop0_op_p1 = cop_instr_p1;
assign o_cop0_cop_p1 = cop_p1 == `UPARC_COP0_MF || cop_p1 == `UPARC_COP0_MT ? cop_p1 :
	{(`UPARC_REGNO_WIDTH){1'b0}};
assign o_cop0_rt_no_p1 = cop_rt_no_p1;
assign o_cop0_reg_no_p1 = cop_reg_no_p1;

/* Coprocessor 0 register value */
always @(*)
begin
	o_cop0_reg_val_p1 = {(`UPARC_REG_WIDTH){1'b0}};

	if(cop_instr_p1 && cop_p1 == `UPARC_COP0_MF)
	begin
		case(cop_reg_no_p1)
		TSCLO: o_cop0_reg_val_p1 = tsc_latched_lo;
		TSCHI: o_cop0_reg_val_p1 = tsc_latched_hi;
		IVT: o_cop0_reg_val_p1 = { reg_ivt, 10'b0 };
		PSR: o_cop0_reg_val_p1 = { {(`UPARC_REG_WIDTH-1){1'b0}}, reg_psr_ie };
		SR: o_cop0_reg_val_p1 = { {(`UPARC_REG_WIDTH-1){1'b0}}, reg_sr_ie };
		CAUSE: o_cop0_reg_val_p1 = { reg_cause_bd, {(`UPARC_REG_WIDTH-1){1'b0}} };
		EPC: o_cop0_reg_val_p1 = reg_epc;
		PRID: o_cop0_reg_val_p1 = reg_prid;
		default: o_cop0_reg_val_p1 = {(`UPARC_REG_WIDTH){1'b0}};
		endcase
	end
end


/* Detect instruction format errors */
always @(*)
begin
	o_decode_error = 1'b0;

	if(cop_instr_p1)
	begin
		case(cop_p1)
		`UPARC_COP0_MF,
		`UPARC_COP0_MT: o_decode_error = |{ rsvd, func } ? 1'b1 : 1'b0;
		`UPARC_COP0_CO: o_decode_error =
			|{ rt, rd, rsvd } || func != `UPARC_COP0_FUNC_RFE ? 1'b1 : 1'b0;
		default: o_decode_error = 1'b1;
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
	else if(!core_stall)
	begin
		instr <= !i_nullify_decode ? i_instr : NOP;
	end
end


/****************************** EXECUTE STAGE *********************************/


reg				cop_instr_p2;
reg [`UPARC_REGNO_WIDTH-1:0]	cop_p2;
reg [`UPARC_REGNO_WIDTH-1:0]	cop_reg_no_p2;
reg [5:0]			cop_func_p2;


/* Execute stage */
always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		cop_instr_p2 <= 1'b0;
		cop_p2 <= {(`UPARC_REGNO_WIDTH){1'b0}};
		cop_reg_no_p2 <= {(`UPARC_REGNO_WIDTH){1'b0}};
		cop_func_p2 <= 6'b0;
	end
	else if(!core_stall && !i_nullify_execute)
	begin
		cop_instr_p2 <= cop_instr_p1;
		cop_p2 <= cop_p1;
		cop_reg_no_p2 <= cop_reg_no_p1;
		cop_func_p2 <= cop_func_p1;
	end
	else if(!core_stall)
	begin
		cop_instr_p2 <= 1'b0;
		cop_p2 <= {(`UPARC_REGNO_WIDTH){1'b0}};
		cop_reg_no_p2 <= {(`UPARC_REGNO_WIDTH){1'b0}};
		cop_func_p2 <= 6'b0;
	end
end


/******************************* MEMORY STAGE *********************************/


reg				cop_instr_p3;
reg [`UPARC_REGNO_WIDTH-1:0]	cop_p3;
reg [`UPARC_REGNO_WIDTH-1:0]	cop_reg_no_p3;
reg [`UPARC_REG_WIDTH-1:0]	cop_reg_val_p3;
reg [5:0]			cop_func_p3;


/* Memory stage */
always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		cop_instr_p3 <= 1'b0;
		cop_p3 <= {(`UPARC_REGNO_WIDTH){1'b0}};
		cop_reg_no_p3 <= {(`UPARC_REGNO_WIDTH){1'b0}};
		cop_reg_val_p3 <= {(`UPARC_REG_WIDTH){1'b0}};
		cop_func_p3 <= 6'b0;
	end
	else if(!core_stall && !i_nullify_mem)
	begin
		cop_instr_p3 <= cop_instr_p2;
		cop_p3 <= cop_p2;
		cop_reg_no_p3 <= cop_reg_no_p2;
		cop_reg_val_p3 <= i_cop0_alu_result_p2;
		cop_func_p3 <= cop_func_p2;
	end
	else if(!core_stall)
	begin
		cop_instr_p3 <= 1'b0;
		cop_p3 <= {(`UPARC_REGNO_WIDTH){1'b0}};
		cop_reg_no_p3 <= {(`UPARC_REGNO_WIDTH){1'b0}};
		cop_reg_val_p3 <= {(`UPARC_REG_WIDTH){1'b0}};
		cop_func_p3 <= 6'b0;
	end
end


/***************************** WRITEBACK STAGE ********************************/


/* Writeback stage */
always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		reg_ivt <= {(`UPARC_ADDR_WIDTH-10){1'b0}};
		reg_psr_ie <= 1'b0;
		reg_sr_ie <= 1'b0;
		reg_cause_bd <= 1'b0;
		reg_epc <= {(`UPARC_ADDR_WIDTH){1'b0}};
	end
	else if(!core_stall && !i_nullify_wb)
	begin
		if(cop_instr_p3 && cop_p3 == `UPARC_COP0_CO)
		begin
			case(cop_func_p3)
			`UPARC_COP0_FUNC_RFE: begin
				reg_sr_ie <= reg_psr_ie;
				reg_psr_ie <= 1'b0;
			end
			endcase
		end
		else if(cop_instr_p3 && cop_p3 == `UPARC_COP0_MT)
		begin
			case(cop_reg_no_p3)
			IVT: reg_ivt <= cop_reg_val_p3[`UPARC_REG_WIDTH-1:10];
			PSR: reg_psr_ie <= cop_reg_val_p3[0];
			SR: reg_sr_ie <= cop_reg_val_p3[0];
			CAUSE: reg_cause_bd <= cop_reg_val_p3[31];
			EPC: reg_epc <= cop_reg_val_p3;
			endcase
		end
	end
	else if(!core_stall && i_except_start)
	begin
		reg_psr_ie <= reg_sr_ie;
		reg_sr_ie <= 1'b0;
		reg_cause_bd <= !i_except_dly_slt ? 1'b0 : 1'b1;
		reg_epc <= !i_except_dly_slt ? i_except_raddr : i_except_raddr_dly;
	end
end


/*************************** TIME STAMP COUNTER *******************************/


reg [2*`UPARC_REG_WIDTH-1:0] tsc_reg;		/* Counter register */
reg [`UPARC_REG_WIDTH-1:0] tsc_latched_lo;	/* Latched lower half */
reg [`UPARC_REG_WIDTH-1:0] tsc_latched_hi;	/* Latched upper half */

/* Latch counter on read of lower half */
wire tsc_latch = (i_instr[31:26] == `UPARC_OP_COP0) &&
			(i_instr[25:21] == `UPARC_COP0_MF) &&
			(i_instr[15:11] == TSCLO) ? 1'b1 : 1'b0;


/* Counter */
always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		tsc_reg <= {(2*`UPARC_ADDR_WIDTH){1'b0}};
	end
	else
	begin
		tsc_reg <= tsc_reg + 1'b1;
	end
end


/* Counter latch logic */
always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		tsc_latched_lo <= {(`UPARC_ADDR_WIDTH){1'b0}};
		tsc_latched_hi <= {(`UPARC_ADDR_WIDTH){1'b0}};
	end
	else if(!core_stall && !i_nullify_decode && tsc_latch)
	begin
		tsc_latched_lo <= tsc_reg[`UPARC_REG_WIDTH-1:0];
		tsc_latched_hi <= tsc_reg[2*`UPARC_REG_WIDTH-1:`UPARC_REG_WIDTH];
	end
end


endmodule /* uparc_coproc0 */
