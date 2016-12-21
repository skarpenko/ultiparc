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
 * Integer multiplication and division unit
 */

`include "cpu_common.vh"
`include "cpu_const.vh"


/* IMulDivU */
module imuldivu(
	clk,
	nrst,
	/* CU signals */
	o_imuldiv_stall,
	i_exec_stall,
	i_mem_stall,
	i_fetch_stall,
	i_drop_p2,
	/* Decoded operation */
	i_imuldiv_op,
	/* Operands */
	i_rs_val,
	i_rt_val,
	/* Result */
	o_imuldiv_rd_val,
	o_imuldiv_rd_valid
);
/* Inputs */
input wire				clk;
input wire				nrst;
/* CU signals */
output wire				o_imuldiv_stall;
input wire				i_exec_stall;
input wire				i_mem_stall;
input wire				i_fetch_stall;
input wire				i_drop_p2;
/* Decoded operation */
input wire [`CPU_IMDOP_WIDTH-1:0]	i_imuldiv_op;
/* Operands */
input wire [`CPU_REG_WIDTH-1:0]		i_rs_val;
input wire [`CPU_REG_WIDTH-1:0]		i_rt_val;
/* Result */
output reg [`CPU_REG_WIDTH-1:0]		o_imuldiv_rd_val;
output reg				o_imuldiv_rd_valid;

wire core_stall;
assign core_stall	= i_exec_stall || i_mem_stall || i_fetch_stall;


reg [2*`CPU_REG_WIDTH-1:0] hilor;	/* HI and LO registers */

assign o_imuldiv_stall = !i_mem_stall && !i_fetch_stall && !muldiv_ready && interlock_instr;

wire interlock_instr;
assign interlock_instr = (i_imuldiv_op == `CPU_IMDOP_MFLO) || (i_imuldiv_op == `CPU_IMDOP_MFHI) ||
			(i_imuldiv_op == `CPU_IMDOP_MUL) || (i_imuldiv_op == `CPU_IMDOP_MULU) ||
			(i_imuldiv_op == `CPU_IMDOP_DIV) || (i_imuldiv_op == `CPU_IMDOP_DIVU);


wire muldiv_ready;
assign muldiv_ready = !div_running && !mul_running;

reg mul_running;
reg div_running;


reg [`CPU_REG_WIDTH-1:0]	rs_reg;
reg [`CPU_REG_WIDTH-1:0]	rt_reg;

reg				div_start;
reg				div_signd;
wire				div_ready;
wire [2*`CPU_REG_WIDTH-1:0]	div_remquot;

reg				mul_start;
reg				mul_signd;
wire				mul_ready;
wire [2*`CPU_REG_WIDTH-1:0]	mul_product;


/****************************** EXECUTE STAGE *********************************/


reg				imd_mthi_p2;
reg				imd_mtlo_p2;
reg [`CPU_REG_WIDTH-1:0]	imd_rval_p2;



/* Execute stage */
always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		imd_mthi_p2 <= 1'b0;
		imd_mtlo_p2 <= 1'b0;
		imd_rval_p2 <= {(`CPU_REG_WIDTH){1'b0}};
		rs_reg <= {(`CPU_REG_WIDTH){1'b0}};
		rt_reg <= {(`CPU_REG_WIDTH){1'b0}};
		div_running <= 1'b0;
		div_start <= 1'b0;
		div_signd <= 1'b0;
		mul_running <= 1'b0;
		mul_start <= 1'b0;
		mul_signd <= 1'b0;
		o_imuldiv_rd_valid <= 1'b0;
		o_imuldiv_rd_val <= {(`CPU_REG_WIDTH){1'b0}};
	end
	else
	begin
		div_start <= 1'b0;
		mul_start <= 1'b0;
		mul_running <= !mul_ready;
		div_running <= !div_ready;

		if(i_drop_p2)
		begin
			imd_mthi_p2 <= 1'b0;
			imd_mtlo_p2 <= 1'b0;
		end
		else if(!core_stall)
		begin
			imd_mthi_p2 <= 1'b0;
			imd_mtlo_p2 <= 1'b0;
			o_imuldiv_rd_valid <= 1'b0;

			if(i_imuldiv_op == `CPU_IMDOP_MFLO)
			begin
				o_imuldiv_rd_valid <= 1'b1;
				o_imuldiv_rd_val <= hilor[`CPU_REG_WIDTH-1:0];
			end
			else if(i_imuldiv_op == `CPU_IMDOP_MFHI)
			begin
				o_imuldiv_rd_valid <= 1'b1;
				o_imuldiv_rd_val <= hilor[2*`CPU_REG_WIDTH-1:`CPU_REG_WIDTH];
			end
			else if(i_imuldiv_op == `CPU_IMDOP_MTLO)
			begin
				imd_mtlo_p2 <= 1'b1;
				imd_rval_p2 <= i_rs_val;
			end
			else if(i_imuldiv_op == `CPU_IMDOP_MTHI)
			begin
				imd_mthi_p2 <= 1'b1;
				imd_rval_p2 <= i_rs_val;
			end
			else if(i_imuldiv_op == `CPU_IMDOP_MUL)
			begin
				rs_reg <= i_rs_val;
				rt_reg <= i_rt_val;
				mul_signd <= 1'b1;
				mul_start <= 1'b1;
				mul_running <= 1'b1;
			end
			else if(i_imuldiv_op == `CPU_IMDOP_MULU)
			begin
				rs_reg <= i_rs_val;
				rt_reg <= i_rt_val;
				mul_signd <= 1'b0;
				mul_start <= 1'b1;
				mul_running <= 1'b1;
			end
			else if(i_imuldiv_op == `CPU_IMDOP_DIV)
			begin
				rs_reg <= i_rs_val;
				rt_reg <= i_rt_val;
				div_signd <= 1'b1;
				div_start <= 1'b1;
				div_running <= 1'b1;
			end
			else if(i_imuldiv_op == `CPU_IMDOP_DIVU)
			begin
				rs_reg <= i_rs_val;
				rt_reg <= i_rt_val;
				div_signd <= 1'b0;
				div_start <= 1'b1;
				div_running <= 1'b1;
			end
		end
	end
end


/******************************* MEMORY STAGE *********************************/


reg				imd_mthi_p3;
reg				imd_mtlo_p3;
reg [`CPU_REG_WIDTH-1:0]	imd_rval_p3;


/* Memory stage */
always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		imd_mthi_p3 <= 1'b0;
		imd_mtlo_p3 <= 1'b0;
		imd_rval_p3 <= {(`CPU_REG_WIDTH){1'b0}};
	end
	else if(!core_stall)
	begin
		imd_mthi_p3 <= imd_mthi_p2;
		imd_mtlo_p3 <= imd_mtlo_p2;
		imd_rval_p3 <= imd_rval_p2;
	end
end


/***************************** WRITEBACK STAGE ********************************/


/* Writeback stage */
always @(posedge clk or negedge nrst)
begin
	if(!nrst)
	begin
		hilor <= {(2*`CPU_REG_WIDTH){1'b0}};
	end
	else if(mul_running && mul_ready && (!imd_mthi_p3 || !imd_mtlo_p3))
	begin
		hilor <= mul_product;
	end
	else if(div_running && div_ready && (!imd_mthi_p3 || !imd_mtlo_p3))
	begin
		hilor <= div_remquot;
	end
	else if(!core_stall)
	begin
		if(imd_mthi_p3) hilor[2*`CPU_REG_WIDTH-1:`CPU_REG_WIDTH] <= imd_rval_p3;
		if(imd_mtlo_p3) hilor[`CPU_REG_WIDTH-1:0] <= imd_rval_p3;
	end
end



/**************** DIVISION AND MULTIPLICATION BLOCKS INSTANCES ****************/


long_idiv idiv(
	.clk(clk),
	.nrst(nrst),
	.dividend(rs_reg),
	.divider(rt_reg),
	.start(div_start),
	.signd(div_signd),
	.ready(div_ready),
	.remquot(div_remquot)
);


long_imul imul(
	.clk(clk),
	.nrst(nrst),
	.multiplicand(rs_reg),
	.multiplier(rt_reg),
	.start(mul_start),
	.signd(mul_signd),
	.ready(mul_ready),
	.product(mul_product)
);


endmodule /* imuldivu */
