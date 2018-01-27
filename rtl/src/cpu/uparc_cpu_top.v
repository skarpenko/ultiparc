/*
 * Copyright (c) 2015-2018 The Ultiparc Project. All rights reserved.
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
 * CPU top level
 */

`include "uparc_cpu_config.vh"
`include "uparc_cpu_common.vh"
`include "uparc_cpu_const.vh"


/* CPU */
module uparc_cpu_top(
	clk,
	nrst,
	/* Interrupt input */
	i_intr,
	/* I-Port */
	o_IAddr,
	o_IRdC,
	i_IData,
	i_IRdy,
	i_IErr,
	/* D-Port */
	o_DAddr,
	o_DCmd,
	o_DRnW,
	o_DBen,
	o_DData,
	i_DData,
	i_DRdy,
	i_DErr
);
input wire				clk;
input wire				nrst;
/* Interrupt input */
input wire				i_intr;
/* I-Port */
output wire [`UPARC_ADDR_WIDTH-1:0]	o_IAddr;
output wire				o_IRdC;
input wire [`UPARC_DATA_WIDTH-1:0]	i_IData;
input wire				i_IRdy;
input wire				i_IErr;
/* D-Port */
output wire [`UPARC_ADDR_WIDTH-1:0]	o_DAddr;
output wire				o_DCmd;
output wire				o_DRnW;
output wire [`UPARC_BEN_WIDTH-1:0]	o_DBen;
output wire [`UPARC_DATA_WIDTH-1:0]	o_DData;
input wire [`UPARC_DATA_WIDTH-1:0]	i_DData;
input wire				i_DRdy;
input wire				i_DErr;


/************************ INTERNAL INTERCONNECT *******************************/

/* Program counter */
reg [`UPARC_ADDR_WIDTH-1:0]	pc_next;
/* pc_p0 is the output from fetch stage */
reg [`UPARC_ADDR_WIDTH-1:0]	pc_p1;
reg [`UPARC_ADDR_WIDTH-1:0]	pc_p2;
reg [`UPARC_ADDR_WIDTH-1:0]	pc_p3;
reg [`UPARC_ADDR_WIDTH-1:0]	pc_p4;


/* Register file wires */
wire [`UPARC_REGNO_WIDTH-1:0]	rf_rs;
wire [`UPARC_REG_WIDTH-1:0]	rf_rs_data;
wire [`UPARC_REGNO_WIDTH-1:0]	rf_rt;
wire [`UPARC_REG_WIDTH-1:0]	rf_rt_data;
wire [`UPARC_REGNO_WIDTH-1:0]	rf_rd;
wire [`UPARC_REG_WIDTH-1:0]	rf_rd_data;


/* Instruction fetch unit wires */
wire [`UPARC_ADDR_WIDTH-1:0]	ifu_addr;
wire [`UPARC_INSTR_WIDTH-1:0]	ifu_instr_dat;
wire				ifu_rd_cmd;
wire				ifu_busy;
wire				ifu_err_align;
wire				ifu_err_bus;


/* Load-store unit wires */
wire [`UPARC_ADDR_WIDTH-1:0]	lsu_addr;
wire [`UPARC_DATA_WIDTH-1:0]	lsu_wdata;
wire [`UPARC_DATA_WIDTH-1:0]	lsu_rdata;
wire [1:0]			lsu_cmd;
wire				lsu_rnw;
wire				lsu_busy;
wire				lsu_err_align;
wire				lsu_err_bus;


/* Control signals */
wire				exec_stall;
wire				mem_stall;
wire				fetch_stall;
wire				wait_stall;
wire				core_stall = fetch_stall || exec_stall || mem_stall || wait_stall;
wire				bus_error_p0;
wire				addr_error_p0;
wire				base_decode_error_p1;
wire				cop0_decode_error_p1;
wire				decode_error_p1 = base_decode_error_p1 || cop0_decode_error_p1;
wire				overfl_error_p2;
wire				addr_error_p2;
wire				syscall_trap_p2;
wire				break_trap_p2;
wire				bus_error_p3;
wire				addr_error_p3;


/* Exception handling signals */
wire				except_start_p3;
wire				except_dly_slt_p3;
wire				except_valid_p4;
wire [`UPARC_ADDR_WIDTH-1:0]	except_haddr_p4;
wire				nullify_fetch;
wire				nullify_decode;
wire				nullify_execute;
wire				nullify_mem;
wire				nullify_wb;


/* Coprocessor 0 wires */
wire				cop0_op_p1;
wire [`UPARC_REGNO_WIDTH-1:0]	cop0_cop_p1;
wire [`UPARC_REGNO_WIDTH-1:0]	cop0_reg_no_p1;
wire [`UPARC_REG_WIDTH-1:0]	cop0_reg_val_p1;
wire [`UPARC_REGNO_WIDTH-1:0]	cop0_rt_no_p1;
wire [`UPARC_ADDR_WIDTH-11:0]	cop0_ivtbase;		/* IVT base */
wire				cop0_ie;		/* IE status */
wire				cop0_intr_wait;		/* Interrupt wait state */


/* Integer multiplication and division unit wires */
wire [`UPARC_REG_WIDTH-1:0]	imuldiv_rd_val;
wire				imuldiv_rd_valid;


/* Fetch stage output */
wire [`UPARC_INSTR_WIDTH-1:0]	instr_p0;
wire [`UPARC_ADDR_WIDTH-1:0]	pc_p0;


/* Decode stage output */
wire [`UPARC_REGNO_WIDTH-1:0]	rd_no_p1;
wire [`UPARC_REGNO_WIDTH-1:0]	rs_no_p1;
wire [`UPARC_REGNO_WIDTH-1:0]	rt_no_p1;
wire [`UPARC_DATA_WIDTH-1:0]	imm_p1;
wire [`UPARC_ALUOP_WIDTH-1:0]	alu_op_p1;
wire [4:0]			alu_inpt_p1;
wire				alu_ovf_ex_p1;
wire [4:0]			jump_p1;
wire				jump_link_p1;
wire [`UPARC_IMDOP_WIDTH-1:0]	imuldiv_op_p1;
wire [`UPARC_SWTRP_WIDTH-1:0]	sw_trap_p1;
wire [`UPARC_LSUOP_WIDTH-1:0]	lsu_op_p1;
wire				lsu_lns_p1;
wire				lsu_ext_p1;

assign rf_rs = rs_no_p1;
assign rf_rt = rt_no_p1;

wire [`UPARC_REG_WIDTH-1:0]	rs_val_p1;
wire [`UPARC_REG_WIDTH-1:0]	rt_val_p1;


/* Execute stage output */
wire [`UPARC_REGNO_WIDTH-1:0]	rd_no_p2;
wire [`UPARC_REG_WIDTH-1:0]	alu_result_p2;
wire [`UPARC_ADDR_WIDTH-1:0]	jump_addr_p2;
wire				jump_valid_p2;
wire [`UPARC_LSUOP_WIDTH-1:0]	lsu_op_p2;
wire				lsu_lns_p2;
wire				lsu_ext_p2;
wire [`UPARC_DATA_WIDTH-1:0]	mem_data_p2;
wire				pend_mem_load_p2 = |lsu_op_p2 & lsu_lns_p2;


/* Memory access stage output */
wire [`UPARC_REGNO_WIDTH-1:0]	rd_no_p3;
wire [`UPARC_REG_WIDTH-1:0]	rd_val_p3;



/************************** UNITS INSTANCE  ***********************************/

/** Register File **/
uparc_reg_file rf(
	.clk(clk),
	.nrst(nrst),
	.rs(rf_rs),
	.rs_data(rf_rs_data),
	.rt(rf_rt),
	.rt_data(rf_rt_data),
	.rd(rf_rd),
	.rd_data(rf_rd_data)
);


/** Instruction Fetch Unit **/
uparc_ifu ifu(
	.clk(clk),
	.nrst(nrst),
	.addr(ifu_addr),
	.instr_dat(ifu_instr_dat),
	.rd_cmd(ifu_rd_cmd),
	.busy(ifu_busy),
	.err_align(ifu_err_align),
	.err_bus(ifu_err_bus),
	.o_IAddr(o_IAddr),
	.o_IRdC(o_IRdC),
	.i_IData(i_IData),
	.i_IRdy(i_IRdy),
	.i_IErr(i_IErr)
);


/** Load-Store Unit **/
uparc_lsu lsu(
	.clk(clk),
	.nrst(nrst),
	.addr(lsu_addr),
	.wdata(lsu_wdata),
	.rdata(lsu_rdata),
	.cmd(lsu_cmd),
	.rnw(lsu_rnw),
	.busy(lsu_busy),
	.err_align(lsu_err_align),
	.err_bus(lsu_err_bus),
	.o_DAddr(o_DAddr),
	.o_DCmd(o_DCmd),
	.o_DRnW(o_DRnW),
	.o_DBen(o_DBen),
	.o_DData(o_DData),
	.i_DData(i_DData),
	.i_DRdy(i_DRdy),
	.i_DErr(i_DErr)
);


/** Forwarding Unit **/
uparc_fwdu fwdu(
	.rs(rs_no_p1),
	.rs_data(rf_rs_data),
	.rt(rt_no_p1),
	.rt_data(rf_rt_data),
	.rd_p2(rd_no_p2),
	.rd_data_p2(alu_result_p2),
	.pend_mem_load_p2(pend_mem_load_p2),
	.rd_p3(rd_no_p3),
	.rd_data_p3(rd_val_p3),
	.rs_data_p1(rs_val_p1),
	.rt_data_p1(rt_val_p1)
);


/** Coprocessor 0 **/
uparc_coproc0 coproc0(
	.clk(clk),
	.nrst(nrst),
	/* Control signals */
	.i_exec_stall(exec_stall),
	.i_mem_stall(mem_stall),
	.i_fetch_stall(fetch_stall),
	.i_wait_stall(wait_stall),
	.o_decode_error(cop0_decode_error_p1),
	.i_except_start(except_start_p3),
	.i_except_dly_slt(except_dly_slt_p3),
	.i_except_raddr(pc_p3),
	.i_except_raddr_dly(pc_p4),
	.i_nullify_decode(nullify_decode),
	.i_nullify_execute(nullify_execute),
	.i_nullify_mem(nullify_mem),
	.i_nullify_wb(nullify_wb),
	/* COP0 signals */
	.o_cop0_ivtbase(cop0_ivtbase),
	.o_cop0_ie(cop0_ie),
	.o_cop0_intr_wait(cop0_intr_wait),
	/* Fetched instruction */
	.i_instr(instr_p0),
	/* Decoded coprocessor instruction */
	.o_cop0_op_p1(cop0_op_p1),
	.o_cop0_cop_p1(cop0_cop_p1),
	.o_cop0_reg_no_p1(cop0_reg_no_p1),
	.o_cop0_reg_val_p1(cop0_reg_val_p1),
	.o_cop0_rt_no_p1(cop0_rt_no_p1),
	/* Execute stage signals */
	.i_cop0_alu_result_p2(alu_result_p2)
);


/** Coprocessor 0. Exceptions and Interrupts Unit. **/
uparc_coproc0_eiu coproc0_eiu(
	.clk(clk),
	.nrst(nrst),
	/* External interrupt */
	.i_intr(i_intr),
	/* CU signals */
	.i_exec_stall(exec_stall),
	.i_mem_stall(mem_stall),
	.i_fetch_stall(fetch_stall),
	.o_wait_stall(wait_stall),
	.i_jump_valid(jump_valid_p2),
	/* COP0 signals */
	.i_cop0_ivtbase(cop0_ivtbase),
	.i_cop0_ie(cop0_ie),
	.i_cop0_intr_wait(cop0_intr_wait),
	/* Exception signals */
	.o_except_start(except_start_p3),
	.o_except_dly_slt(except_dly_slt_p3),
	.o_except_valid(except_valid_p4),
	.o_except_haddr(except_haddr_p4),
	/* Error signals from stages */
	.i_bus_error_p0(bus_error_p0),
	.i_addr_error_p0(addr_error_p0),
	.i_decode_error_p1(decode_error_p1),
	.i_overfl_error_p2(overfl_error_p2),
	.i_addr_error_p2(addr_error_p2),
	.i_syscall_trap_p2(syscall_trap_p2),
	.i_break_trap_p2(break_trap_p2),
	.i_bus_error_p3(bus_error_p3),
	.i_addr_error_p3(addr_error_p3),
	/* Result nullify signals */
	.o_nullify_fetch(nullify_fetch),
	.o_nullify_decode(nullify_decode),
	.o_nullify_execute(nullify_execute),
	.o_nullify_mem(nullify_mem),
	.o_nullify_wb(nullify_wb)
);


/** Integer multiplication and division unit **/
uparc_imuldivu imuldivu(
	.clk(clk),
	.nrst(nrst),
	/* Control signals */
	.o_exec_stall(exec_stall),
	.i_mem_stall(mem_stall),
	.i_fetch_stall(fetch_stall),
	.i_wait_stall(wait_stall),
	.i_nullify_execute(nullify_execute),
	.i_nullify_mem(nullify_mem),
	.i_nullify_wb(nullify_wb),
	/* Decoded operation */
	.i_imuldiv_op(imuldiv_op_p1),
	/* Operands */
	.i_rs_val(rs_val_p1),
	.i_rt_val(rt_val_p1),
	/* Result of MFHI and MFLO */
	.o_imuldiv_rd_val(imuldiv_rd_val),
	.o_imuldiv_rd_valid(imuldiv_rd_valid)
);




/*************************** PIPELINE STAGES **********************************/

/** Fetch stage **/
uparc_fetch fetch(
	.clk(clk),
	.nrst(nrst),
	/* Control signals */
	.i_pc(pc_next),
	.i_jump_addr(jump_addr_p2),
	.i_jump_valid(jump_valid_p2),
	.i_except_valid(except_valid_p4),
	.i_except_haddr(except_haddr_p4),
	.i_exec_stall(exec_stall),
	.i_mem_stall(mem_stall),
	.o_fetch_stall(fetch_stall),
	.i_wait_stall(wait_stall),
	.o_bus_error(bus_error_p0),
	.o_addr_error(addr_error_p0),
	.i_nullify(nullify_fetch),
	/* IFU signals */
	.o_addr(ifu_addr),
	.i_instr_dat(ifu_instr_dat),
	.o_rd_cmd(ifu_rd_cmd),
	.i_busy(ifu_busy),
	.i_err_align(ifu_err_align),
	.i_err_bus(ifu_err_bus),
	/* Fetched instruction */
	.o_instr(instr_p0),
	.o_pc(pc_p0)
);


/** Decode stage **/
uparc_decode decode(
	.clk(clk),
	.nrst(nrst),
	/* Control signals */
	.i_pc(pc_p0),
	.i_exec_stall(exec_stall),
	.i_mem_stall(mem_stall),
	.i_fetch_stall(fetch_stall),
	.i_wait_stall(wait_stall),
	.o_decode_error(base_decode_error_p1),
	.i_nullify(nullify_decode),
	/* Coprocessor 0 */
	.i_cop0_cop(cop0_cop_p1),
	.i_cop0_reg_no(cop0_reg_no_p1),
	.i_cop0_rt_no(cop0_rt_no_p1),
	/* Fetched instruction */
	.i_instr(instr_p0),
	/* Decoded instruction */
	.o_rd_no(rd_no_p1),
	.o_rs_no(rs_no_p1),
	.o_rt_no(rt_no_p1),
	.o_imm(imm_p1),
	.o_alu_op(alu_op_p1),
	.o_alu_inpt(alu_inpt_p1),
	.o_alu_ovf_ex(alu_ovf_ex_p1),
	.o_jump(jump_p1),
	.o_jump_link(jump_link_p1),
	.o_imuldiv_op(imuldiv_op_p1),
	.o_sw_trap(sw_trap_p1),
	.o_lsu_op(lsu_op_p1),
	.o_lsu_lns(lsu_lns_p1),
	.o_lsu_ext(lsu_ext_p1)
);


/** Execute stage **/
uparc_execute execute(
	.clk(clk),
	.nrst(nrst),
	/* Control signals */
	.i_pc_p0(pc_next),
	.i_pc_p1(pc_p0),
	.i_exec_stall(exec_stall),
	.i_mem_stall(mem_stall),
	.i_fetch_stall(fetch_stall),
	.i_wait_stall(wait_stall),
	.o_overfl_error(overfl_error_p2),
	.o_addr_error(addr_error_p2),
	.o_syscall_trap(syscall_trap_p2),
	.o_break_trap(break_trap_p2),
	.i_nullify(nullify_execute),
	/* Coprocessor 0 */
	.i_cop0_op(cop0_op_p1),
	.i_cop0_cop(cop0_cop_p1),
	.i_cop0_reg_val(cop0_reg_val_p1),
	/* Integer multiplication and division unit */
	.i_imuldiv_rd_val(imuldiv_rd_val),
	.i_imuldiv_rd_valid(imuldiv_rd_valid),
	/* Decoded instruction */
	.i_rd_no(rd_no_p1),
	.i_rs_val(rs_val_p1),
	.i_rt_val(rt_val_p1),
	.i_imm(imm_p1),
	.i_alu_op(alu_op_p1),
	.i_alu_inpt(alu_inpt_p1),
	.i_alu_ovf_ex(alu_ovf_ex_p1),
	.i_jump(jump_p1),
	.i_jump_link(jump_link_p1),
	.i_sw_trap(sw_trap_p1),
	.i_lsu_op(lsu_op_p1),
	.i_lsu_lns(lsu_lns_p1),
	.i_lsu_ext(lsu_ext_p1),
	/* Stage output */
	.o_rd_no(rd_no_p2),
	.o_alu_result(alu_result_p2),
	.o_jump_addr(jump_addr_p2),
	.o_jump_valid(jump_valid_p2),
	.o_lsu_op(lsu_op_p2),
	.o_lsu_lns(lsu_lns_p2),
	.o_lsu_ext(lsu_ext_p2),
	.o_mem_data(mem_data_p2)
);


/** Memory access stage **/
uparc_memory_access memory_access(
	.clk(clk),
	.nrst(nrst),
	/* Control signals */
	.i_exec_stall(exec_stall),
	.o_mem_stall(mem_stall),
	.i_fetch_stall(fetch_stall),
	.i_wait_stall(wait_stall),
	.o_bus_error(bus_error_p3),
	.o_addr_error(addr_error_p3),
	.i_nullify(nullify_mem),
	/* LSU interface */
	.lsu_addr(lsu_addr),
	.lsu_wdata(lsu_wdata),
	.lsu_rdata(lsu_rdata),
	.lsu_cmd(lsu_cmd),
	.lsu_rnw(lsu_rnw),
	.lsu_busy(lsu_busy),
	.lsu_err_align(lsu_err_align),
	.lsu_err_bus(lsu_err_bus),
	/* Result of execute stage */
	.i_rd_no(rd_no_p2),
	.i_alu_result(alu_result_p2),
	.i_lsu_op(lsu_op_p2),
	.i_lsu_lns(lsu_lns_p2),
	.i_lsu_ext(lsu_ext_p2),
	.i_mem_data(mem_data_p2),
	/* Data for writeback */
	.o_rd_no(rd_no_p3),
	.o_rd_val(rd_val_p3)
);


/** Writeback stage **/
uparc_writeback writeback(
	.clk(clk),
	.nrst(nrst),
	/* Control signals */
	.i_exec_stall(exec_stall),
	.i_mem_stall(mem_stall),
	.i_fetch_stall(fetch_stall),
	.i_wait_stall(wait_stall),
	.i_nullify(nullify_wb),
	/* Data for writeback */
	.i_rd_no(rd_no_p3),
	.i_rd_val(rd_val_p3),
	.o_rd_no(rf_rd),
	.o_rd_val(rf_rd_data)
);


/* Program counter */
always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		pc_next <= `UPARC_RESET_ADDR;
		pc_p1 <= `UPARC_RESET_ADDR + 1*`UPARC_INSTR_SIZE;
		pc_p2 <= `UPARC_RESET_ADDR + 2*`UPARC_INSTR_SIZE;
		pc_p3 <= `UPARC_RESET_ADDR + 3*`UPARC_INSTR_SIZE;
		pc_p4 <= `UPARC_RESET_ADDR + 4*`UPARC_INSTR_SIZE;
	end
	else if(!core_stall)
	begin
		pc_next <= ( !except_valid_p4 ?
			(!jump_valid_p2 ? pc_next : jump_addr_p2) : except_haddr_p4 ) +
			`UPARC_INSTR_SIZE;
		pc_p1 <= pc_p0;
		pc_p2 <= pc_p1;
		pc_p3 <= pc_p2;
		pc_p4 <= pc_p3;
	end
end


endmodule /* uparc_cpu_top */
