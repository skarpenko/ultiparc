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
 * Coprocessor 0. Exceptions and Interrupts Unit.
 */

`include "uparc_cpu_config.vh"
`include "uparc_cpu_common.vh"
`include "uparc_cpu_const.vh"


/* Coprocessor 0 EIU */
module uparc_coproc0_eiu(
	clk,
	nrst,
	/* External interrupt */
	i_intr,
	/* CU signals */
	i_exec_stall,
	i_mem_stall,
	i_fetch_stall,
	i_jump_valid,
	/* COP0 signals */
	i_cop0_ivtbase,
	i_cop0_ie,
	/* Exception signals */
	o_except_start,
	o_except_dly_slt,
	o_except_valid,
	o_except_haddr,
	/* Error signals from stages */
	i_bus_error_p0,
	i_addr_error_p0,
	i_decode_error_p1,
	i_overfl_error_p2,
	i_addr_error_p2,
	i_syscall_trap_p2,
	i_break_trap_p2,
	i_bus_error_p3,
	i_addr_error_p3,
	/* Result nullify signals */
	o_nullify_fetch,
	o_nullify_decode,
	o_nullify_execute,
	o_nullify_mem,
	o_nullify_wb
);
/* Exceptions */
localparam [2:0] EX_NONE	= 3'b000;
localparam [2:0] EX_BUSERR	= 3'b001;
localparam [2:0] EX_OVERFL	= 3'b010;
localparam [2:0] EX_ADDRERR	= 3'b011;
localparam [2:0] EX_RESVDI	= 3'b100;
localparam [2:0] EX_BREAK	= 3'b101;
localparam [2:0] EX_SYSCALL	= 3'b110;
localparam [2:0] EX_HWINTR	= 3'b111;
/* Inputs */
input wire				clk;
input wire				nrst;
/* External interrupt */
input wire 				i_intr;
/* CU signals */
input wire				i_exec_stall;
input wire				i_mem_stall;
input wire				i_fetch_stall;
input wire				i_jump_valid;
/* COP0 signals */
input wire [`UPARC_ADDR_WIDTH-11:0]	i_cop0_ivtbase;
input wire				i_cop0_ie;
/* Exception signals */
output wire				o_except_start;
output wire				o_except_dly_slt;
output reg				o_except_valid;
output reg [`UPARC_ADDR_WIDTH-1:0]	o_except_haddr;
/* Error signals from stages */
input wire				i_bus_error_p0;
input wire				i_addr_error_p0;
input wire				i_decode_error_p1;
input wire				i_overfl_error_p2;
input wire				i_addr_error_p2;
input wire				i_syscall_trap_p2;
input wire				i_break_trap_p2;
input wire				i_bus_error_p3;
input wire				i_addr_error_p3;
/* Result nullify signals */
output wire				o_nullify_fetch;
output wire				o_nullify_decode;
output wire				o_nullify_execute;
output wire				o_nullify_mem;
output wire				o_nullify_wb;


wire core_stall = i_exec_stall || i_mem_stall || i_fetch_stall;


assign o_nullify_fetch = ex_state_p0 || ex_state_p1 || ex_state_p2 || ex_state_p3;
assign o_nullify_decode = ex_state_p0 || ex_state_p1 || ex_state_p2 || ex_state_p3;
assign o_nullify_execute = ex_state_p1 || ex_state_p2 || ex_state_p3;
assign o_nullify_mem = ex_state_p2 || ex_state_p3;
assign o_nullify_wb =  ex_state_p3;
assign o_except_start = ex_state_p3;
assign o_except_dly_slt = dly_p3;


/* External interrupt registered flag */
wire intr_reg = intr_reg_p3 | intr_reg_p4;


/* Exception state at stages */
wire ex_state_p0 = i_bus_error_p0 || i_addr_error_p0;
wire ex_state_p1 = |ex_p1 || i_decode_error_p1;
wire ex_state_p2 = |ex_p2 || i_overfl_error_p2 || i_addr_error_p2 ||
		i_syscall_trap_p2 || i_break_trap_p2 ||
		(intr_valid && !bubble_p2 && !intr_reg);
wire ex_state_p3 = |ex_p3 || i_bus_error_p3 || i_addr_error_p3;



/******************************* DECODE STAGE *********************************/

reg [2:0] ex_p1;
reg bubble_p1;

always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		ex_p1 <= EX_NONE;
		bubble_p1 <= 1'b0;
	end
	else if(!core_stall)
	begin
		if(i_bus_error_p0)
			ex_p1 <= EX_BUSERR;
		else if(i_addr_error_p0)
			ex_p1 <= EX_ADDRERR;
		else
			ex_p1 <= EX_NONE;

		/* Mark bubble instructions (nullified instructions) */
		bubble_p1 <= (o_nullify_fetch || i_jump_valid || o_except_valid) ? 1'b1 : 1'b0;
	end
end


/****************************** EXECUTE STAGE *********************************/

reg [2:0] ex_p2;
reg dly_p2;
reg bubble_p2;

always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		ex_p2 <= EX_NONE;
		dly_p2 <= 1'b0;
		bubble_p2 <= 1'b0;
	end
	else if(!core_stall)
	begin
		if(i_decode_error_p1)
			ex_p2 <= EX_RESVDI;
		else
			ex_p2 <= ex_p1;

		dly_p2 <= i_jump_valid;
		bubble_p2 <= bubble_p1;
	end
end


/******************************* MEMORY STAGE *********************************/

reg [2:0] ex_p3;
reg dly_p3;
reg intr_reg_p3;

always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		ex_p3 <= EX_NONE;
		dly_p3 <= 1'b0;
		intr_reg_p3 <= 1'b0;
	end
	else if(!core_stall)
	begin
		if(intr_valid && !bubble_p2 && !intr_reg)
		begin
			ex_p3 <= EX_HWINTR;
			intr_reg_p3 <= 1'b1;
		end
		else if(i_overfl_error_p2)
			ex_p3 <= EX_OVERFL;
		else if(i_addr_error_p2)
			ex_p3 <= EX_ADDRERR;
		else if(i_syscall_trap_p2)
			ex_p3 <= EX_SYSCALL;
		else if(i_break_trap_p2)
			ex_p3 <= EX_BREAK;
		else
		begin
			ex_p3 <= ex_p2;
			intr_reg_p3 <= 1'b0;
		end

		dly_p3 <= dly_p2;
	end
end


/***************************** WRITEBACK STAGE ********************************/

reg intr_reg_p4;

always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		o_except_valid <= 1'b0;
		o_except_haddr <= {(`UPARC_ADDR_WIDTH){1'b0}};
		intr_reg_p4 <= 1'b0;
	end
	else if(!core_stall)
	begin
		intr_reg_p4 <= intr_reg_p3;

		if(i_bus_error_p3)
		begin
			o_except_valid <= 1'b1;
			o_except_haddr <= { i_cop0_ivtbase, `UPARC_EXVECT_BUSERR };
		end
		else if(i_addr_error_p3)
		begin
			o_except_valid <= 1'b1;
			o_except_haddr <= { i_cop0_ivtbase, `UPARC_EXVECT_ADDRERR };
		end
		else if(ex_p3)
		begin
			o_except_valid <= 1'b1;
			case(ex_p3)
			EX_BUSERR:	o_except_haddr <= { i_cop0_ivtbase, `UPARC_EXVECT_BUSERR };
			EX_OVERFL:	o_except_haddr <= { i_cop0_ivtbase, `UPARC_EXVECT_OVERFL };
			EX_ADDRERR:	o_except_haddr <= { i_cop0_ivtbase, `UPARC_EXVECT_ADDRERR };
			EX_RESVDI:	o_except_haddr <= { i_cop0_ivtbase, `UPARC_EXVECT_RESVDI };
			EX_BREAK:	o_except_haddr <= { i_cop0_ivtbase, `UPARC_EXVECT_BREAK };
			EX_SYSCALL:	o_except_haddr <= { i_cop0_ivtbase, `UPARC_EXVECT_SYSCALL };
			EX_HWINTR:	o_except_haddr <= { i_cop0_ivtbase, `UPARC_EXVECT_HWINTR };
			default:	o_except_haddr <= { i_cop0_ivtbase, `UPARC_EXVECT_RESET };
			endcase
		end
		else
		begin
			o_except_valid <= 1'b0;
		end
	end
end


/************************* EXTERNAL INTERRUPT CAPTURE *************************/

reg intr_valid;
reg intr_latch;

always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		intr_valid <= 1'b0;
		intr_latch <= 1'b0;
	end
	else if(!core_stall)
	begin
		intr_valid <= (intr_latch | ((i_intr && i_cop0_ie && !intr_valid) ? 1'b1 : 1'b0)) &
			~intr_reg;
		intr_latch <= 1'b0;
	end
	else
		intr_latch <= intr_latch | ((i_intr && i_cop0_ie && !intr_valid) ? 1'b1 : 1'b0);
end


endmodule /* uparc_coproc0_eiu */
