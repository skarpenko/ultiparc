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
 * CPU top level
 */

`include "cpu_common.vh"
`include "cpu_const.vh"


/* CPU */
module cpu_top(
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
output wire [`CPU_ADDR_WIDTH-1:0]	o_IAddr;
output wire				o_IRdC;
input wire [`CPU_DATA_WIDTH-1:0]	i_IData;
input wire				i_IRdy;
input wire				i_IErr;
/* D-Port */
output wire [`CPU_ADDR_WIDTH-1:0]	o_DAddr;
output wire				o_DCmd;
output wire				o_DRnW;
output wire [`CPU_BEN_WIDTH-1:0]	o_DBen;
output wire [`CPU_DATA_WIDTH-1:0]	o_DData;
input wire [`CPU_DATA_WIDTH-1:0]	i_DData;
input wire				i_DRdy;
input wire				i_DErr;


/************************ INTERNAL INTERCONNECT *******************************/

/* Program counter */
reg [`CPU_ADDR_WIDTH-1:0]	pc_next;
reg [`CPU_ADDR_WIDTH-1:0]	pc_prev;

/* Register file wires */
wire [`CPU_REGNO_WIDTH-1:0]	rf_rs;
wire [`CPU_REG_WIDTH-1:0]	rf_rs_data;
wire [`CPU_REGNO_WIDTH-1:0]	rf_rt;
wire [`CPU_REG_WIDTH-1:0]	rf_rt_data;
wire [`CPU_REGNO_WIDTH-1:0]	rf_rd;
wire [`CPU_REG_WIDTH-1:0]	rf_rd_data;

/* Instruction fetch unit wires */
wire [`CPU_ADDR_WIDTH-1:0]	ifu_addr;
wire [`CPU_INSTR_WIDTH-1:0]	ifu_instr_dat;
wire				ifu_rd_cmd;
wire				ifu_busy;
wire				ifu_err_align;
wire				ifu_err_bus;

/* Load-store unit wires */
wire [`CPU_ADDR_WIDTH-1:0]	lsu_addr;
wire [`CPU_DATA_WIDTH-1:0]	lsu_wdata;
wire [`CPU_DATA_WIDTH-1:0]	lsu_rdata;
wire [1:0]			lsu_cmd;
wire				lsu_rnw;
wire				lsu_busy;
wire				lsu_err_align;
wire				lsu_err_bus;

/* Control signals */
wire				exec_stall;
wire				mem_stall;
wire				fetch_stall;
wire				core_stall = fetch_stall || exec_stall || mem_stall;
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


/* Coprocessor 0 wires */
wire				cop0_op_p1;
wire [`CPU_REGNO_WIDTH-1:0]	cop0_cop_p1;
wire [`CPU_REGNO_WIDTH-1:0]	cop0_reg_no_p1;
wire [`CPU_REG_WIDTH-1:0]	cop0_reg_val_p1;
wire [`CPU_REGNO_WIDTH-1:0]	cop0_rt_no_p1;


/* Integer multiplication and division unit wires */
wire [`CPU_REG_WIDTH-1:0]	imuldiv_rd_val;
wire				imuldiv_rd_valid;


/* Fetch stage output */
wire [`CPU_INSTR_WIDTH-1:0]	instr_p0;


/* Decode stage output */
wire [`CPU_REGNO_WIDTH-1:0]	rd_no_p1;
wire [`CPU_REGNO_WIDTH-1:0]	rs_no_p1;
wire [`CPU_REGNO_WIDTH-1:0]	rt_no_p1;
wire [`CPU_DATA_WIDTH-1:0]	imm_p1;
wire [`CPU_ALUOP_WIDTH-1:0]	alu_op_p1;
wire [4:0]			alu_inpt_p1;
wire				alu_ovf_ex_p1;
wire [4:0]			jump_p1;
wire				jump_link_p1;
wire [`CPU_IMDOP_WIDTH-1:0]	imuldiv_op_p1;
wire [`CPU_SWTRP_WIDTH-1:0]	sw_trap_p1;
wire [`CPU_LSUOP_WIDTH-1:0]	lsu_op_p1;
wire				lsu_lns_p1;
wire				lsu_ext_p1;

assign rf_rs = rs_no_p1;
assign rf_rt = rt_no_p1;

wire [`CPU_REG_WIDTH-1:0]	rs_val_p1;
wire [`CPU_REG_WIDTH-1:0]	rt_val_p1;


/* Execute stage output */
wire [`CPU_REGNO_WIDTH-1:0]	rd_no_p2;
wire [`CPU_REG_WIDTH-1:0]	alu_result_p2;
wire [`CPU_ADDR_WIDTH-1:0]	jump_addr_p2;
wire				jump_valid_p2;
wire [`CPU_LSUOP_WIDTH-1:0]	lsu_op_p2;
wire				lsu_lns_p2;
wire				lsu_ext_p2;
wire [`CPU_DATA_WIDTH-1:0]	mem_data_p2;


/* Memory access stage output */
wire [`CPU_REGNO_WIDTH-1:0]	rd_no_p3;
wire [`CPU_REG_WIDTH-1:0]	rd_val_p3;



/************************** UNITS INSTANCE  ***********************************/

/** Register File **/
reg_file rf(
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
ifu ifu(
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
lsu lsu(
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
fwdu fwdu(
	.rs(rs_no_p1),
	.rs_data(rf_rs_data),
	.rt(rt_no_p1),
	.rt_data(rf_rt_data),
	.rd_p2(rd_no_p2),
	.rd_data_p2(alu_result_p2),
	.rd_p3(rd_no_p3),
	.rd_data_p3(rd_val_p3),
	.rs_data_p1(rs_val_p1),
	.rt_data_p1(rt_val_p1)
);


/** Coprocessor 0 **/
coproc0 coproc0(
	.clk(clk),
	.nrst(nrst),
	/* Control signals */
	.i_exec_stall(exec_stall),
	.i_mem_stall(mem_stall),
	.i_fetch_stall(fetch_stall),
	.o_decode_error(cop0_decode_error_p1),
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


/** Integer multiplication and division unit **/
imuldivu imuldivu(
	.clk(clk),
	.nrst(nrst),
	/* Control signals */
	.o_exec_stall(exec_stall),
	.i_mem_stall(mem_stall),
	.i_fetch_stall(fetch_stall),
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
fetch fetch(
	.clk(clk),
	.nrst(nrst),
	/* Control signals */
	.i_pc(pc_next),
	.i_jump_addr(jump_addr_p2),
	.i_jump_valid(jump_valid_p2),
	.i_exec_stall(exec_stall),
	.i_mem_stall(mem_stall),
	.o_fetch_stall(fetch_stall),
	.o_bus_error(bus_error_p0),
	.o_addr_error(addr_error_p0),
	/* IFU signals */
	.o_addr(ifu_addr),
	.i_instr_dat(ifu_instr_dat),
	.o_rd_cmd(ifu_rd_cmd),
	.i_busy(ifu_busy),
	.i_err_align(ifu_err_align),
	.i_err_bus(ifu_err_bus),
	/* Fetched instruction */
	.o_instr(instr_p0)
);


/** Decode stage **/
decode decode(
	.clk(clk),
	.nrst(nrst),
	/* Control signals */
	.i_pc(pc_prev),
	.i_exec_stall(exec_stall),
	.i_mem_stall(mem_stall),
	.i_fetch_stall(fetch_stall),
	.o_decode_error(base_decode_error_p1),
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
execute execute(
	.clk(clk),
	.nrst(nrst),
	/* Control signals */
	.i_pc_p0(pc_next),
	.i_pc_p1(pc_prev),
	.i_exec_stall(exec_stall),
	.i_mem_stall(mem_stall),
	.i_fetch_stall(fetch_stall),
	.o_overfl_error(overfl_error_p2),
	.o_addr_error(addr_error_p2),
	.o_syscall_trap(syscall_trap_p2),
	.o_break_trap(break_trap_p2),
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
memory_access memory_access(
	.clk(clk),
	.nrst(nrst),
	/* Control signals */
	.i_exec_stall(exec_stall),
	.o_mem_stall(mem_stall),
	.i_fetch_stall(fetch_stall),
	.o_bus_error(bus_error_p3),
	.o_addr_error(addr_error_p3),
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
writeback writeback(
	.clk(clk),
	.nrst(nrst),
	/* Control signals */
	.i_exec_stall(exec_stall),
	.i_mem_stall(mem_stall),
	.i_fetch_stall(fetch_stall),
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
		pc_next <= `CPU_RESET_ADDR;
		pc_prev <= `CPU_RESET_ADDR;
	end
	else if(!core_stall)
	begin
		pc_next <= (!jump_valid_p2 ? pc_next : jump_addr_p2) + `CPU_INSTR_SIZE;
		pc_prev <= pc_next;
	end
end


endmodule /* cpu_top */
