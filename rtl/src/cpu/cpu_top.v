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
reg [`CPU_ADDR_WIDTH-1:0]	pc_p0;
reg [`CPU_ADDR_WIDTH-1:0]	pc_p1;
reg [`CPU_ADDR_WIDTH-1:0]	pc_p2;
reg [`CPU_ADDR_WIDTH-1:0]	pc_p3;
reg [`CPU_ADDR_WIDTH-1:0]	pc_p4;


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
wire				core_stall;
wire				exec_stall;
wire				mem_stall;
wire				fetch_stall;
assign core_stall = fetch_stall || exec_stall || mem_stall;

wire drop_p2;


wire [`CPU_INSTR_WIDTH-1:0]	instr_p0;


wire [5:0]			op_p1;
wire [`CPU_REGNO_WIDTH-1:0]	dst_gpr_p1;
wire [`CPU_REGNO_WIDTH-1:0]	src1_gpr_p1;
wire [`CPU_REGNO_WIDTH-1:0]	src2_gpr_p1;
wire [`CPU_DATA_WIDTH-1:0]	src3_se_v_p1;
wire [`CPU_DATA_WIDTH-1:0]	src3_ze_v_p1;
wire [`CPU_DATA_WIDTH-1:0]	src3_sh16_v_p1;
wire [`CPU_ADDR_WIDTH-1:0]	src3_j_v_p1;
wire [`CPU_ADDR_WIDTH-1:0]	src3_broff_v_p1;
wire [4:0]			shamt_p1;
wire [5:0]			func_p1;


assign rf_rs = src1_gpr_p1;
assign rf_rt = src2_gpr_p1;


wire [`CPU_REG_WIDTH-1:0]	src1_gpr_v_p2;
wire [`CPU_REG_WIDTH-1:0]	src2_gpr_v_p2;

wire [5:0]			op_p2;
wire [`CPU_REGNO_WIDTH-1:0]	dst_gpr_p2;
wire [`CPU_DATA_WIDTH-1:0]	result_p2;
wire [`CPU_DATA_WIDTH-1:0]	mem_data_p2;
wire				j_addr_valid_p2;
wire [`CPU_ADDR_WIDTH-1:0]	j_addr_p2;


wire [`CPU_REGNO_WIDTH-1:0]	dst_gpr_p3;
wire [`CPU_DATA_WIDTH-1:0]	dst_gpr_v_p3;



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
	.rs(src1_gpr_p1),
	.rs_data(rf_rs_data),
	.rt(src2_gpr_p1),
	.rt_data(rf_rt_data),
//	.rd_p3(dst_gpr_p3),
//	.rd_data_p3(dst_gpr_v_p3),
	.rd_p3(dst_gpr_p2),
	.rd_data_p3(result_p2),
//	.rd_p4(rf_rd),
//	.rd_data_p4(rf_rd_data),
	.rd_p4(dst_gpr_p3),
	.rd_data_p4(dst_gpr_v_p3),
	.rs_data_p2(src1_gpr_v_p2),
	.rt_data_p2(src2_gpr_v_p2)
);



/*************************** PIPELINE STAGES **********************************/

/** Fetch stage **/
fetch fetch(
	.clk(clk),
	.nrst(nrst),
	/* Control signals */
	.i_pc(pc_next),
	.i_j_valid(j_addr_valid_p2),
	.o_instr(instr_p0),
	.i_exec_stall(exec_stall),
	.i_mem_stall(mem_stall),
	.o_fetch_stall(fetch_stall),
	/* IFU signals */
	.o_addr(ifu_addr),
	.i_instr_dat(ifu_instr_dat),
	.o_rd_cmd(ifu_rd_cmd),
	.i_busy(ifu_busy),
	.i_err_align(ifu_err_align),
	.i_err_bus(ifu_err_bus)
);


/** Decode stage **/
decode decode(
	.clk(clk),
	.nrst(nrst),
	/* Control signals */
	.i_pc(pc_p1),
	.i_instr(instr_p0),
	.i_exec_stall(exec_stall),
	.i_mem_stall(mem_stall),
	.i_fetch_stall(fetch_stall),
	.i_drop(drop_p2),
	/* Decoded instr */
	.o_op(op_p1),
	.o_dst_gpr(dst_gpr_p1),
	.o_src1_gpr(src1_gpr_p1),
	.o_src2_gpr(src2_gpr_p1),
	.o_src3_se_v(src3_se_v_p1),
	.o_src3_ze_v(src3_ze_v_p1),
	.o_src3_sh16_v(src3_sh16_v_p1),
	.o_src3_j_v(src3_j_v_p1),
	.o_src3_broff_v(src3_broff_v_p1),
	.o_shamt(shamt_p1),
	.o_func(func_p1)
);


/** Execute stage **/
execute execute(
	.clk(clk),
	.nrst(nrst),
	/* Control signals */
	.i_pc_p0(pc_p0),
	.i_pc_p1(pc_p1),
	.o_exec_stall(exec_stall),
	.i_mem_stall(mem_stall),
	.i_fetch_stall(fetch_stall),
	.o_drop(drop_p2),
	/* Decoded instr */
	.i_op(op_p1),
	.i_dst_gpr(dst_gpr_p1),
	.i_src1_gpr_v(src1_gpr_v_p2),
	.i_src2_gpr_v(src2_gpr_v_p2),
	.i_src3_se_v(src3_se_v_p1),
	.i_src3_ze_v(src3_ze_v_p1),
	.i_src3_sh16_v(src3_sh16_v_p1),
	.i_src3_j_v(src3_j_v_p1),
	.i_src3_broff_v(src3_broff_v_p1),
	.i_shamt(shamt_p1),
	.i_func(func_p1),
	.i_regimm(src2_gpr_p1),
	/* Stage output */
	.o_op(op_p2),
	.o_dst_gpr(dst_gpr_p2),
	.o_result(result_p2),
	.o_mem_data(mem_data_p2),
	.o_j_addr_valid(j_addr_valid_p2),
	.o_j_addr(j_addr_p2)
);


/** Memory access stage **/
memory_access memory_access(
	.clk(clk),
	.nrst(nrst),
	/* CU signals */
	.i_exec_stall(exec_stall),
	.o_mem_stall(mem_stall),
	.i_fetch_stall(fetch_stall),
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
	.i_op(op_p2),
	.i_dst_gpr(dst_gpr_p2),
	.i_result(result_p2),
	.i_mem_data(mem_data_p2),
	/* Data for writeback */
	.o_dst_gpr(dst_gpr_p3),
	.o_dst_gpr_v(dst_gpr_v_p3)
);


/** Writeback stage **/
writeback writeback(
	.clk(clk),
	.nrst(nrst),
	/* CU signals */
	.i_exec_stall(exec_stall),
	.i_mem_stall(mem_stall),
	.i_fetch_stall(fetch_stall),
	/* Data for writeback */
	.i_dst_gpr(dst_gpr_p3),
	.i_dst_gpr_v(dst_gpr_v_p3),
	.o_rd(rf_rd),
	.o_rd_data(rf_rd_data)
);


always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		pc_next <= 0;
		pc_p0 <= 0;	/* TODO: default addr */
		pc_p1 <= 0;
		pc_p2 <= 0;
		pc_p3 <= 0;
		pc_p4 <= 0;
	end
	else if(!core_stall)
	begin
		pc_next <= j_addr_valid_p2 ? j_addr_p2 : pc_next + 4;
		pc_p0 <= pc_next;
		pc_p1 <= pc_p0;
		pc_p2 <= pc_p1;
		pc_p3 <= pc_p2;
		pc_p4 <= pc_p3;
	end
end


endmodule /* cpu_top */
