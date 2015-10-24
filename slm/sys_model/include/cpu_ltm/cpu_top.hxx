/*
 * Copyright (c) 2015 The Ultiparc Project. All rights reserved.
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
 * CPU top module (Loosely timed model. Executable specification.)
 */

#include <systemc.h>
#include <stdint.h>
#include <iostream>
#include <string>
#pragma once


// CPU top
SC_MODULE(cpu_top) {
	sc_in<bool>          clk;
	sc_in<bool>          nrst;

	// I-Bus port
	sc_out<sc_uint<32> > ib_addr_o;
	sc_out<bool>         ib_rdc_o;
	sc_in<sc_uint<32> >  ib_data_i;
	sc_in<bool>          ib_rdy_i;
	sc_in<bool>          ib_err_i;

	// D-Bus port
	sc_out<sc_uint<32> > db_addr_o;
	sc_out<bool>         db_cmd_o;
	sc_out<bool>         db_rnw_o;
	sc_out<sc_uint<4> >  db_ben_o;
	sc_out<sc_uint<32> > db_data_o;
	sc_in<sc_uint<32> >  db_data_i;
	sc_in<bool>          db_rdy_i;
	sc_in<bool>          db_err_i;


	SC_CTOR(cpu_top)
		: m_prid(0)
	{
		SC_THREAD(cpu_thread);
			sensitive << clk.pos();

		// Program counter
		m_pc = 0x00000000;  // Reset vector address
		m_next_pc = 0x00000004;

		// Special registers
		m_sr = 0x00000000;
		m_psr = 0x00000000;
		m_ivtb = 0x00000000;

		// Internal state
		m_except = EX_NONE;
		m_delay_slot = false;
		m_interrupt = false;
	}

private:
	static const unsigned IVT_ENTRY_SZ = 8;

	enum cpu_exception {
		EX_NONE = 0,		// No exception
		EX_RESET,		// Reset
		EX_BUS_ERR,		// Bus Error
		EX_INT_OVERFLOW,	// Integer Overflow
		EX_ADDR_ERR,		// Address Error
		EX_RSVD_INSTR,		// Reserved Instruction
		EX_BREAKPOINT,		// Breakpoint
		EX_SYSCALL,		// System call
		EX_HW_INTR		// Hardware Interrupt
	};

	struct rtype_instr {
		uint32_t func  : 6;
		uint32_t shamt : 5;
		uint32_t rd    : 5;
		uint32_t rt    : 5;
		uint32_t rs    : 5;
		uint32_t op    : 6;
	};

	struct itype_instr {
		uint32_t imm : 16;
		uint32_t rt  : 5;
		uint32_t rs  : 5;
		uint32_t op  : 6;
	};

	struct jtype_instr {
		uint32_t target : 26;
		uint32_t op     : 6;
	};

	union instruction {
		uint32_t word;
		rtype_instr r;
		itype_instr i;
		jtype_instr j;
	};

private:
	int32_t sign_extend8(uint32_t imm);
	int32_t sign_extend16(uint32_t imm);

	uint32_t zero_extend8(uint32_t imm);
	uint32_t zero_extend16(uint32_t imm);

	uint32_t rd_gpreg(uint32_t r);
	void wr_gpreg(uint32_t r, uint32_t v);

	uint32_t fetch_instr(uint32_t pc);

	uint32_t load_byte(uint32_t addr);
	uint32_t load_halfword(uint32_t addr);
	uint32_t load_word(uint32_t addr);

	void store_byte(uint32_t addr, uint32_t v);
	void store_halfword(uint32_t addr, uint32_t v);
	void store_word(uint32_t addr, uint32_t v);

	bool check_except(void);
	bool check_intr(void);

	// TODO: put description here
	void instr_mfc0(uint32_t instr);
	void instr_mtc0(uint32_t instr);
	void instr_rfe(uint32_t instr);

	void instr_sll(uint32_t instr);
	void instr_srl(uint32_t instr);
	void instr_sra(uint32_t instr);
	void instr_sllv(uint32_t instr);
	void instr_srlv(uint32_t instr);
	void instr_srav(uint32_t instr);
	void instr_jr(uint32_t instr);
	void instr_jalr(uint32_t instr);
	void instr_syscall(uint32_t instr);
	void instr_break(uint32_t instr);
	void instr_mfhi(uint32_t instr);
	void instr_mthi(uint32_t instr);
	void instr_mflo(uint32_t instr);
	void instr_mtlo(uint32_t instr);
	void instr_mult(uint32_t instr);
	void instr_multu(uint32_t instr);
	void instr_div(uint32_t instr);
	void instr_divu(uint32_t instr);
	void instr_add(uint32_t instr);
	void instr_addu(uint32_t instr);
	void instr_sub(uint32_t instr);
	void instr_subu(uint32_t instr);
	void instr_and(uint32_t instr);
	void instr_or(uint32_t instr);
	void instr_xor(uint32_t instr);
	void instr_nor(uint32_t instr);
	void instr_slt(uint32_t instr);
	void instr_sltu(uint32_t instr);

	void instr_bltz(uint32_t instr);
	void instr_bgez(uint32_t instr);
	void instr_bltzal(uint32_t instr);
	void instr_bgezal(uint32_t instr);

	void instr_j(uint32_t instr);
	void instr_jal(uint32_t instr);
	void instr_beq(uint32_t instr);
	void instr_bne(uint32_t instr);
	void instr_blez(uint32_t instr);
	void instr_bgtz(uint32_t instr);
	void instr_addi(uint32_t instr);
	void instr_addiu(uint32_t instr);
	void instr_slti(uint32_t instr);
	void instr_sltiu(uint32_t instr);
	void instr_andi(uint32_t instr);
	void instr_ori(uint32_t instr);
	void instr_xori(uint32_t instr);
	void instr_lui(uint32_t instr);

	void instr_lb(uint32_t instr);
	void instr_lh(uint32_t instr);
	void instr_lw(uint32_t instr);
	void instr_lbu(uint32_t instr);
	void instr_lhu(uint32_t instr);
	void instr_sb(uint32_t instr);
	void instr_sh(uint32_t instr);
	void instr_sw(uint32_t instr);

	void exec_special(uint32_t instr, bool dslot);
	void exec_cop0(uint32_t instr, bool dslot);
	void exec_load_store(uint32_t instr, bool dslot);
	void exec_regimm(uint32_t instr, bool dslot);
	void exec_other(uint32_t instr, bool dslot);

	void execute(uint32_t instr, bool dslot);

	void cpu_thread(void);

private:
	uint32_t m_gp_regs[32];		// General purpose registers
	uint32_t m_pc, m_next_pc;	// Program Counters

	uint32_t m_hi, m_lo;		// Hi and Lo registers for Div and Mul instructions

	const uint32_t m_prid;		// Processor Id
	uint32_t m_epc;			// Return address from exception
	uint32_t m_sr;			// Status register
	uint32_t m_psr;			// Previous value of Status Register stored on exception
	uint32_t m_ivtb;		// Base address of interrupt vectors table

	cpu_exception m_except;		// Current exceptions state

	bool m_delay_slot;		// Instruction in delay slot need to be executed
	bool m_interrupt;		// Hardware interrupt occurred
};


inline int32_t cpu_top::sign_extend8(uint32_t imm)
{
	return (imm & (1<<7)) ? static_cast<int32_t>(imm | (0xFFFFFF00)) : imm;
}


inline int32_t cpu_top::sign_extend16(uint32_t imm)
{
	return (imm & (1<<15)) ? static_cast<int32_t>(imm | (0xFFFF0000)) : imm;
}


inline uint32_t cpu_top::zero_extend8(uint32_t imm)
{
	return imm;
}


inline uint32_t cpu_top::zero_extend16(uint32_t imm)
{
	return imm;
}


inline uint32_t cpu_top::rd_gpreg(uint32_t r)
{
	r &= 0x1F;
	return (r ? m_gp_regs[r] : 0);
}

inline void cpu_top::wr_gpreg(uint32_t r, uint32_t v)
{
	r &= 0x1F;
	m_gp_regs[r] = (r ? v : m_gp_regs[r]);
}


inline uint32_t cpu_top::fetch_instr(uint32_t pc)
{
	if(pc & 0x3) {
		std::cerr << __PRETTY_FUNCTION__
			<< ": Unaligned PC!"
			<< std::endl;
		sc_stop();
	}

	wait();
	ib_addr_o.write(pc);
	ib_rdc_o.write(true);
	do {
		wait(clk.posedge_event() | ib_rdy_i.value_changed_event() | ib_err_i.value_changed_event());
	} while(ib_rdy_i.read() != true && ib_err_i.read() != true);

	uint32_t instr = ib_data_i.read();
	if(ib_err_i.read() == true)
		m_except = EX_BUS_ERR;
	ib_rdc_o.write(false);

	return instr;
}


inline uint32_t cpu_top::load_byte(uint32_t addr)
{
	uint32_t sht = addr & 0x3;
	uint32_t be = 1 << sht;
	addr &= ~0x3;

	wait();
	db_addr_o.write(addr);
	db_ben_o.write(be);
	db_rnw_o.write(true);
	db_cmd_o.write(true);

	do {
		wait(clk.posedge_event() | db_rdy_i.value_changed_event() | db_err_i.value_changed_event());
	} while(db_rdy_i.read() != true && db_err_i.read() != true);

	uint32_t data = db_data_i.read();
	if(db_err_i.read() == true)
		m_except = EX_BUS_ERR;
	db_cmd_o.write(false);

	data >>= (sht<<3);
	data &= 0xff;

	return data;
}


inline uint32_t cpu_top::load_halfword(uint32_t addr)
{
	if(addr & 0x1) {
		std::cerr << __PRETTY_FUNCTION__
			<< ": Unaligned address!"
			<< std::endl;
		sc_stop();
	}

	uint32_t sht = addr & 0x3;
	uint32_t be = 3 << sht;
	addr &= ~0x3;

	wait();
	db_addr_o.write(addr);
	db_ben_o.write(be);
	db_rnw_o.write(true);
	db_cmd_o.write(true);

	do {
		wait(clk.posedge_event() | db_rdy_i.value_changed_event() | db_err_i.value_changed_event());
	} while(db_rdy_i.read() != true && db_err_i.read() != true);

	uint32_t data = db_data_i.read();
	if(db_err_i.read() == true)
		m_except = EX_BUS_ERR;
	db_cmd_o.write(false);

	data >>= (sht<<3);
	data &= 0xffff;

	return data;
}


inline uint32_t cpu_top::load_word(uint32_t addr)
{
	if(addr & 0x3) {
		std::cerr << __PRETTY_FUNCTION__
			<< ": Unaligned address!"
			<< std::endl;
		sc_stop();
	}

	wait();
	db_addr_o.write(addr);
	db_ben_o.write(0xf);
	db_rnw_o.write(true);
	db_cmd_o.write(true);

	do {
		wait(clk.posedge_event() | db_rdy_i.value_changed_event() | db_err_i.value_changed_event());
	} while(db_rdy_i.read() != true && db_err_i.read() != true);

	uint32_t data = db_data_i.read();
	if(db_err_i.read() == true)
		m_except = EX_BUS_ERR;
	db_cmd_o.write(false);

	return data;
}


inline void cpu_top::store_byte(uint32_t addr, uint32_t v)
{
	uint32_t sht = addr & 0x3;
	uint32_t be = 1 << sht;
	addr &= ~0x3;
	v <<= (sht<<3);

	wait();
	db_data_o.write(v);
	db_addr_o.write(addr);
	db_ben_o.write(be);
	db_rnw_o.write(false);
	db_cmd_o.write(true);

	do {
		wait(clk.posedge_event() | db_rdy_i.value_changed_event() | db_err_i.value_changed_event());
	} while(db_rdy_i.read() != true && db_err_i.read() != true);

	if(db_err_i.read() == true)
		m_except = EX_BUS_ERR;
	db_cmd_o.write(false);
}


inline void cpu_top::store_halfword(uint32_t addr, uint32_t v)
{
	if(addr & 0x1) {
		std::cerr << __PRETTY_FUNCTION__
			<< ": Unaligned address!"
			<< std::endl;
		sc_stop();
	}

	uint32_t sht = addr & 0x3;
	uint32_t be = 3 << sht;
	addr &= ~0x3;
	v <<= (sht<<3);

	wait();
	db_data_o.write(v);
	db_addr_o.write(addr);
	db_ben_o.write(be);
	db_rnw_o.write(false);
	db_cmd_o.write(true);

	do {
		wait(clk.posedge_event() | db_rdy_i.value_changed_event() | db_err_i.value_changed_event());
	} while(db_rdy_i.read() != true && db_err_i.read() != true);

	if(db_err_i.read() == true)
		m_except = EX_BUS_ERR;
	db_cmd_o.write(false);
}


inline void cpu_top::store_word(uint32_t addr, uint32_t v)
{
	if(addr & 0x3) {
		std::cerr << __PRETTY_FUNCTION__
			<< ": Unaligned address!"
			<< std::endl;
		sc_stop();
	}

	wait();
	db_data_o.write(v);
	db_addr_o.write(addr);
	db_ben_o.write(0xf);
	db_rnw_o.write(false);
	db_cmd_o.write(true);

	do {
		wait(clk.posedge_event() | db_rdy_i.value_changed_event() | db_err_i.value_changed_event());
	} while(db_rdy_i.read() != true && db_err_i.read() != true);

	if(db_err_i.read() == true)
		m_except = EX_BUS_ERR;
	db_cmd_o.write(false);
}


inline bool cpu_top::check_except(void)
{
	if(m_except) {
		m_epc = m_pc;
		m_psr = m_sr;
		m_sr = 0;
		m_pc = m_ivtb + (m_except-1)*IVT_ENTRY_SZ;
		m_next_pc = m_pc + 4;
		m_except = EX_NONE;
		return true;
	} else
		return false;
}


inline bool cpu_top::check_intr(void)
{
	if(m_interrupt && (m_sr & 0x1)) {
		m_epc = m_next_pc;
		m_psr = m_sr;
		m_sr = 0;
		m_pc = m_ivtb + (EX_HW_INTR-1)*IVT_ENTRY_SZ;
		m_next_pc = m_pc + 4;
		return true;
	} else
		return false;
}


inline void cpu_top::instr_mfc0(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	if(iw.r.func || iw.r.shamt) {
		m_except = EX_RSVD_INSTR;
		return;
	}

	switch(iw.r.rd) {
		case 10: // IVTB register
			m_gp_regs[iw.r.rt] = m_ivtb;
			break;
		case 11: // PSR Register
			m_gp_regs[iw.r.rt] = m_psr;
			break;
		case 12: // SR Register
			m_gp_regs[iw.r.rt] = m_sr;
			break;
		case 14: // EPC Register
			m_gp_regs[iw.r.rt] = m_epc;
			break;
		case 15: // PRId Register
			m_gp_regs[iw.r.rt] = m_prid;
			break;
		default:
			break;
	}
}


inline void cpu_top::instr_mtc0(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	if(iw.r.func || iw.r.shamt) {
		m_except = EX_RSVD_INSTR;
		return;
	}

	switch(iw.r.rd) {
		case 10: // IVTB register
			m_ivtb = m_gp_regs[iw.r.rt] & (~0x3);
			break;
		case 11: // PSR Register
			m_psr = m_gp_regs[iw.r.rt] & (~0x1);
			break;
		case 12: // SR Register
			m_sr = m_gp_regs[iw.r.rt] & (~0x1);
			break;
		case 14: // EPC Register
			m_epc = m_gp_regs[iw.r.rt];
			break;
		case 15: // PRId Register
		default:
			break;
	}
}


inline void cpu_top::instr_rfe(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	if(iw.r.func == 16 && !iw.r.rd && !iw.r.rt && !iw.r.shamt) {
		m_sr = m_psr;
	} else {
		m_except = EX_RSVD_INSTR;
	}
}


inline void cpu_top::instr_sll(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	if(iw.r.rs) {
		m_except = EX_RSVD_INSTR;
		return;
	}

	uint32_t rt = rd_gpreg(iw.r.rt);
	wr_gpreg(iw.r.rd, (rt << iw.r.shamt));
}


inline void cpu_top::instr_srl(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	if(iw.r.rs) {
		m_except = EX_RSVD_INSTR;
		return;
	}

	uint32_t rt = rd_gpreg(iw.r.rt);
	wr_gpreg(iw.r.rd, (rt >> iw.r.shamt));
}


inline void cpu_top::instr_sra(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	if(iw.r.rs) {
		m_except = EX_RSVD_INSTR;
		return;
	}

	uint32_t rt = rd_gpreg(iw.r.rt);
	wr_gpreg(iw.r.rd, ((int32_t)rt >> iw.r.shamt));
}


inline void cpu_top::instr_sllv(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t rt = rd_gpreg(iw.r.rt);
	uint32_t rs = rd_gpreg(iw.r.rs);
	wr_gpreg(iw.r.rd, (rt << (rs & 0x1F)));
}


inline void cpu_top::instr_srlv(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t rt = rd_gpreg(iw.r.rt);
	uint32_t rs = rd_gpreg(iw.r.rs);
	wr_gpreg(iw.r.rd, (rt >> (rs & 0x1F)));
}


inline void cpu_top::instr_srav(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t rt = rd_gpreg(iw.r.rt);
	uint32_t rs = rd_gpreg(iw.r.rs);
	wr_gpreg(iw.r.rd, ((int32_t)rt >> (rs & 0x1F)));
}


inline void cpu_top::instr_jr(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	if(iw.r.rt || iw.r.rd) {
		m_except = EX_RSVD_INSTR;
		return;
	}

	uint32_t target = rd_gpreg(iw.r.rs);

	if(target & 0x3) {
		m_except = EX_ADDR_ERR;
		return;
	}

	m_next_pc = target;
}


inline void cpu_top::instr_jalr(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	if(iw.r.rt) {
		m_except = EX_RSVD_INSTR;
		return;
	}

	uint32_t target = rd_gpreg(iw.r.rs);

	if(target & 0x3) {
		m_except = EX_ADDR_ERR;
		return;
	}

	wr_gpreg(iw.r.rd, m_pc + 4 + 4);

	m_next_pc = target;
}


inline void cpu_top::instr_syscall(uint32_t instr)
{
	(void)instr;
	m_except = EX_SYSCALL;
}


inline void cpu_top::instr_break(uint32_t instr)
{
	(void)instr;
	m_except = EX_BREAKPOINT;
}


inline void cpu_top::instr_mfhi(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	if(iw.r.rs || iw.r.rt) {
		m_except = EX_RSVD_INSTR;
		return;
	}

	wr_gpreg(iw.r.rd, m_hi);
}


inline void cpu_top::instr_mthi(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	if(iw.r.rd || iw.r.rt) {
		m_except = EX_RSVD_INSTR;
		return;
	}

	m_hi = rd_gpreg(iw.r.rs);
}


inline void cpu_top::instr_mflo(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	if(iw.r.rs || iw.r.rt) {
		m_except = EX_RSVD_INSTR;
		return;
	}

	wr_gpreg(iw.r.rd, m_lo);
}


inline void cpu_top::instr_mtlo(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	if(iw.r.rd || iw.r.rt) {
		m_except = EX_RSVD_INSTR;
		return;
	}

	m_lo = rd_gpreg(iw.r.rs);
}


inline void cpu_top::instr_mult(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	if(iw.r.rd) {
		m_except = EX_RSVD_INSTR;
		return;
	}

	int32_t rs = rd_gpreg(iw.r.rs);
	int32_t rt = rd_gpreg(iw.r.rt);
	int64_t t = rs * rt;
	m_lo = (uint32_t)(t & 0xFFFFFFFF);
	m_hi = (uint32_t)((t>>32) & 0xFFFFFFFF);
}


inline void cpu_top::instr_multu(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	if(iw.r.rd) {
		m_except = EX_RSVD_INSTR;
		return;
	}

	uint32_t rs = rd_gpreg(iw.r.rs);
	uint32_t rt = rd_gpreg(iw.r.rt);
	uint64_t t = rs * rt;
	m_lo = (uint32_t)(t & 0xFFFFFFFF);
	m_hi = (uint32_t)((t>>32) & 0xFFFFFFFF);
}


inline void cpu_top::instr_div(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	if(iw.r.rd) {
		m_except = EX_RSVD_INSTR;
		return;
	}

	int32_t rs = rd_gpreg(iw.r.rs);
	int32_t rt = rd_gpreg(iw.r.rt);
	if(rt == 0)
		return;

	m_lo = (uint32_t)(rs / rt);
	m_hi = (uint32_t)(rs % rt);
}


inline void cpu_top::instr_divu(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	if(iw.r.rd) {
		m_except = EX_RSVD_INSTR;
		return;
	}

	uint32_t rs = rd_gpreg(iw.r.rs);
	uint32_t rt = rd_gpreg(iw.r.rt);
	if(rt == 0)
		return;

	m_lo = (uint32_t)(rs / rt);
	m_hi = (uint32_t)(rs % rt);
}


inline void cpu_top::instr_add(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t rs = rd_gpreg(iw.r.rs);
	uint32_t rt = rd_gpreg(iw.r.rt);
	uint32_t rd = rs + rt;
	if(!((rs ^ rt) & (1<<31)) && ((rd ^ rs) & (1<<31))) {
		m_except = EX_INT_OVERFLOW;
		return;
	}

	wr_gpreg(iw.r.rd, rd);
}


inline void cpu_top::instr_addu(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t rs = rd_gpreg(iw.r.rs);
	uint32_t rt = rd_gpreg(iw.r.rt);
	uint32_t rd = rs + rt;

	wr_gpreg(iw.r.rd, rd);
}


inline void cpu_top::instr_sub(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t rs = rd_gpreg(iw.r.rs);
	uint32_t rt = rd_gpreg(iw.r.rt);
	uint32_t rd = rs - rt;
	if(!((rs ^ ~rt) & (1<<31)) && ((rd ^ rs) & (1<<31))) {
		m_except = EX_INT_OVERFLOW;
		return;
	}

	wr_gpreg(iw.r.rd, rd);
}


inline void cpu_top::instr_subu(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t rs = rd_gpreg(iw.r.rs);
	uint32_t rt = rd_gpreg(iw.r.rt);
	uint32_t rd = rs - rt;

	wr_gpreg(iw.r.rd, rd);
}


inline void cpu_top::instr_and(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t rs = rd_gpreg(iw.r.rs);
	uint32_t rt = rd_gpreg(iw.r.rt);
	uint32_t rd = rs & rt;

	wr_gpreg(iw.r.rd, rd);
}


inline void cpu_top::instr_or(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t rs = rd_gpreg(iw.r.rs);
	uint32_t rt = rd_gpreg(iw.r.rt);
	uint32_t rd = rs | rt;

	wr_gpreg(iw.r.rd, rd);
}


inline void cpu_top::instr_xor(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t rs = rd_gpreg(iw.r.rs);
	uint32_t rt = rd_gpreg(iw.r.rt);
	uint32_t rd = rs ^ rt;

	wr_gpreg(iw.r.rd, rd);
}


inline void cpu_top::instr_nor(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t rs = rd_gpreg(iw.r.rs);
	uint32_t rt = rd_gpreg(iw.r.rt);
	uint32_t rd = !(rs | rt);

	wr_gpreg(iw.r.rd, rd);
}


inline void cpu_top::instr_slt(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	int32_t rs = rd_gpreg(iw.r.rs);
	int32_t rt = rd_gpreg(iw.r.rt);
	uint32_t rd = (rs < rt ? 1 : 0);

	wr_gpreg(iw.r.rd, rd);
}


inline void cpu_top::instr_sltu(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t rs = rd_gpreg(iw.r.rs);
	uint32_t rt = rd_gpreg(iw.r.rt);
	uint32_t rd = (rs < rt ? 1 : 0);

	wr_gpreg(iw.r.rd, rd);
}


inline void cpu_top::instr_bltz(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t target = sign_extend16(iw.i.imm) << 2;
	uint32_t rs = rd_gpreg(iw.i.rs);
	bool cond = ((rs & (1<<31)) != 0);
	if(cond) {
		m_next_pc = m_pc + 4 + target;
	} else {
		m_next_pc = m_pc + 4 + 4;
	}
}


inline void cpu_top::instr_bgez(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t target = sign_extend16(iw.i.imm) << 2;
	uint32_t rs = rd_gpreg(iw.i.rs);
	bool cond = ((rs & (1<<31)) == 0);
	if(cond) {
		m_next_pc = m_pc + 4 + target;
	} else {
		m_next_pc = m_pc + 4 + 4;
	}
}


inline void cpu_top::instr_bltzal(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t target = sign_extend16(iw.i.imm) << 2;
	uint32_t rs = rd_gpreg(iw.i.rs);
	bool cond = ((rs & (1<<31)) != 0);
	m_gp_regs[31] = m_pc + 8;
	if(cond) {
		m_next_pc = m_pc + 4 + target;
	} else {
		m_next_pc = m_pc + 4 + 4;
	}
}


inline void cpu_top::instr_bgezal(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t target = sign_extend16(iw.i.imm) << 2;
	uint32_t rs = rd_gpreg(iw.i.rs);
	bool cond = ((rs & (1<<31)) == 0);
	m_gp_regs[31] = m_pc + 8;
	if(cond) {
		m_next_pc = m_pc + 4 + target;
	} else {
		m_next_pc = m_pc + 4 + 4;
	}
}


inline void cpu_top::instr_j(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t target = iw.j.target;
	m_next_pc = ((m_pc+4) & 0xF0000000) | target << 2;
}


inline void cpu_top::instr_jal(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t target = iw.j.target;
	m_next_pc = ((m_pc+4) & 0xF0000000) | target << 2;
	m_gp_regs[31] = m_pc + 4 + 4;
}


inline void cpu_top::instr_beq(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t target = sign_extend16(iw.i.imm) << 2;
	uint32_t rs = rd_gpreg(iw.i.rs);
	uint32_t rt = rd_gpreg(iw.i.rt);
	bool cond = (rs == rt);
	if(cond) {
		m_next_pc = m_pc + 4 + target;
	} else {
		m_next_pc = m_pc + 4 + 4;
	}
}


inline void cpu_top::instr_bne(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t target = sign_extend16(iw.i.imm) << 2;
	uint32_t rs = rd_gpreg(iw.i.rs);
	uint32_t rt = rd_gpreg(iw.i.rt);
	bool cond = (rs != rt);
	if(cond) {
		m_next_pc = m_pc + 4 + target;
	} else {
		m_next_pc = m_pc + 4 + 4;
	}
}


inline void cpu_top::instr_blez(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	if(iw.i.rt) {
		m_except = EX_RSVD_INSTR;
		return;
	}

	uint32_t target = sign_extend16(iw.i.imm) << 2;
	uint32_t rs = rd_gpreg(iw.i.rs);
	bool cond = ((rs & (1<<31)) != 0 || rs == 0);
	if(cond) {
		m_next_pc = m_pc + 4 + target;
	} else {
		m_next_pc = m_pc + 4 + 4;
	}
}


inline void cpu_top::instr_bgtz(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	if(iw.i.rt) {
		m_except = EX_RSVD_INSTR;
		return;
	}

	uint32_t target = sign_extend16(iw.i.imm) << 2;
	uint32_t rs = rd_gpreg(iw.i.rs);
	bool cond = ((rs & (1<<31)) == 0 || rs != 0);
	if(cond) {
		m_next_pc = m_pc + 4 + target;
	} else {
		m_next_pc = m_pc + 4 + 4;
	}
}


inline void cpu_top::instr_addi(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t rs = rd_gpreg(iw.i.rs);
	uint32_t imm = sign_extend16(iw.i.imm);
	uint32_t rt = rs+imm;
	if(!((rs ^ imm) & (1<<31)) && ((rt ^ rs) & (1<<31))) {
		m_except = EX_INT_OVERFLOW;
		return;
	}

	wr_gpreg(iw.i.rt, rt);
}


inline void cpu_top::instr_addiu(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t rs = rd_gpreg(iw.i.rs);
	uint32_t imm = sign_extend16(iw.i.imm);
	uint32_t rt = rs+imm;

	wr_gpreg(iw.i.rt, rt);
}


inline void cpu_top::instr_slti(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	int32_t rs = rd_gpreg(iw.i.rs);
	int32_t imm = sign_extend16(iw.i.imm);
	uint32_t rt = (rs < imm ? 1 : 0);

	wr_gpreg(iw.i.rt, rt);
}


inline void cpu_top::instr_sltiu(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t rs = rd_gpreg(iw.i.rs);
	uint32_t imm = sign_extend16(iw.i.imm);
	uint32_t rt = (rs < imm ? 1 : 0);

	wr_gpreg(iw.i.rt, rt);
}


inline void cpu_top::instr_andi(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t rs = rd_gpreg(iw.i.rs);
	uint32_t imm = zero_extend16(iw.i.imm);
	uint32_t rt = rs & imm;

	wr_gpreg(iw.i.rt, rt);
}


inline void cpu_top::instr_ori(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t rs = rd_gpreg(iw.i.rs);
	uint32_t imm = zero_extend16(iw.i.imm);
	uint32_t rt = rs | imm;

	wr_gpreg(iw.i.rt, rt);
}


inline void cpu_top::instr_xori(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t rs = rd_gpreg(iw.i.rs);
	uint32_t imm = zero_extend16(iw.i.imm);
	uint32_t rt = rs ^ imm;

	wr_gpreg(iw.i.rt, rt);
}


inline void cpu_top::instr_lui(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	if(iw.i.rs) {
		m_except = EX_RSVD_INSTR;
		return;
	}

	wr_gpreg(iw.i.rt, (iw.i.imm << 16));
}


inline void cpu_top::instr_lb(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t addr = rd_gpreg(iw.i.rs) + sign_extend16(iw.i.imm);
	uint32_t data = sign_extend8( load_byte(addr) );
	wr_gpreg(iw.i.rt, data);
}


inline void cpu_top::instr_lh(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t addr = rd_gpreg(iw.i.rs) + sign_extend16(iw.i.imm);
	if(addr & 0x1) {
		m_except = EX_ADDR_ERR;
		return;
	}

	uint32_t data = sign_extend16( load_halfword(addr) );
	wr_gpreg(iw.i.rt, data);
}


inline void cpu_top::instr_lw(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t addr = rd_gpreg(iw.i.rs) + sign_extend16(iw.i.imm);
	if(addr & 0x3) {
		m_except = EX_ADDR_ERR;
		return;
	}

	uint32_t data = load_word(addr);
	wr_gpreg(iw.i.rt, data);
}


inline void cpu_top::instr_lbu(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t addr = rd_gpreg(iw.i.rs) + sign_extend16(iw.i.imm);
	uint32_t data = zero_extend8( load_byte(addr) );
	wr_gpreg(iw.i.rt, data);
}


inline void cpu_top::instr_lhu(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t addr = rd_gpreg(iw.i.rs) + sign_extend16(iw.i.imm);
	if(addr & 0x1) {
		m_except = EX_ADDR_ERR;
		return;
	}

	uint32_t data = zero_extend16( load_halfword(addr) );
	wr_gpreg(iw.i.rt, data);
}


inline void cpu_top::instr_sb(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t addr = rd_gpreg(iw.i.rs) + sign_extend16(iw.i.imm);
	uint32_t data = rd_gpreg(iw.i.rt) & 0xFF;
	store_byte(addr, data);
}


inline void cpu_top::instr_sh(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t addr = rd_gpreg(iw.i.rs) + sign_extend16(iw.i.imm);
	if(addr & 0x1) {
		m_except = EX_ADDR_ERR;
		return;
	}

	uint32_t data = rd_gpreg(iw.i.rt) & 0xFFFF;
	store_halfword(addr, data);
}


inline void cpu_top::instr_sw(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t addr = rd_gpreg(iw.i.rs) + sign_extend16(iw.i.imm);
	if(addr & 0x3) {
		m_except = EX_ADDR_ERR;
		return;
	}

	uint32_t data = rd_gpreg(iw.i.rt);
	store_word(addr, data);
}


inline void cpu_top::exec_special(uint32_t instr, bool dslot)
{
	instruction iw;
	iw.word = instr;

	if(iw.r.shamt &&
		(iw.r.func != 0 && iw.r.func != 2 && iw.r.func != 3 &&
			iw.r.func != 12 && iw.r.func != 13)) {
		m_except = EX_RSVD_INSTR;
		return;
	}

	switch(iw.r.func) {
		case 0:
			instr_sll(instr);
			break;
		case 2:
			instr_srl(instr);
			break;
		case 3:
			instr_sra(instr);
			break;
		case 4:
			instr_sllv(instr);
			break;
		case 6:
			instr_srlv(instr);
			break;
		case 7:
			instr_srav(instr);
			break;
		case 8:
			if(!dslot) instr_jr(instr);
			break;
		case 9:
			if(!dslot) instr_jalr(instr);
			break;
		case 12:
			instr_syscall(instr);
			break;
		case 13:
			instr_break(instr);
			break;
		case 16:
			instr_mfhi(instr);
			break;
		case 17:
			instr_mthi(instr);
			break;
		case 18:
			instr_mflo(instr);
			break;
		case 19:
			instr_mtlo(instr);
			break;
		case 24:
			instr_mult(instr);
			break;
		case 25:
			instr_multu(instr);
			break;
		case 26:
			instr_div(instr);
			break;
		case 27:
			instr_divu(instr);
			break;
		case 32:
			instr_add(instr);
			break;
		case 33:
			instr_addu(instr);
			break;
		case 34:
			instr_sub(instr);
			break;
		case 35:
			instr_subu(instr);
			break;
		case 36:
			instr_and(instr);
			break;
		case 37:
			instr_or(instr);
			break;
		case 38:
			instr_xor(instr);
			break;
		case 39:
			instr_nor(instr);
			break;
		case 42:
			instr_slt(instr);
			break;
		case 43:
			instr_sltu(instr);
			break;
		default:
			m_except = EX_RSVD_INSTR;
			break;
	}
}


//
// Execute Coprocessor 0 instructions
// Registers supported:
//   0xA  - IVTB (Interrupt Vector Table Base, two LSB bits are ignored)
//   0xB  - PSR  (Copy of SR register on interrupt/exception entrance)
//   0xC  - SR   (Status Register)
//             Bit 0 - IE (Interrupt enable)
//   0xE  - EPC  (Program counter saved on interrupt/exception entrance)
//   0xF  - PRId (Processor Id)
//
inline void cpu_top::exec_cop0(uint32_t instr, bool dslot)
{
	(void)dslot;

	instruction iw;
	iw.word = instr;

	switch(iw.r.rs) {
		case 0:
			instr_mfc0(instr);
			break;
		case 4:
			instr_mtc0(instr);
			break;
		case 16:
			instr_rfe(instr);
			break;
		default:
			m_except = EX_RSVD_INSTR;
			break;
	}
}


inline void cpu_top::exec_load_store(uint32_t instr, bool dslot)
{
	(void)dslot;

	instruction iw;
	iw.word = instr;

	switch(iw.r.op) {
		case 32:
			instr_lb(instr);
			break;
		case 33:
			instr_lh(instr);
			break;
		case 35:
			instr_lw(instr);
			break;
		case 36:
			instr_lbu(instr);
			break;
		case 37:
			instr_lhu(instr);
			break;
		case 40:
			instr_sb(instr);
			break;
		case 41:
			instr_sh(instr);
			break;
		case 43:
			instr_sw(instr);
			break;
		default:
			m_except = EX_RSVD_INSTR;
			break;
	}
}


inline void cpu_top::exec_regimm(uint32_t instr, bool dslot)
{
	(void)dslot;

	instruction iw;
	iw.word = instr;

	switch(iw.r.rt) {
		case 0:
			if(!dslot) instr_bltz(instr);
			break;
		case 1:
			if(!dslot) instr_bgez(instr);
			break;
		case 16:
			if(!dslot) instr_bltzal(instr);
			break;
		case 17:
			if(!dslot) instr_bgezal(instr);
			break;
		default:
			m_except = EX_RSVD_INSTR;
			break;
	}
}


inline void cpu_top::exec_other(uint32_t instr, bool dslot)
{
	instruction iw;
	iw.word = instr;

	switch(iw.r.op) {
		case 2:
			instr_j(instr);
			break;
		case 3:
			instr_jal(instr);
			break;
		case 4:
			instr_beq(instr);
			break;
		case 5:
			instr_bne(instr);
			break;
		case 6:
			instr_blez(instr);
			break;
		case 7:
			instr_bgtz(instr);
			break;
		case 8:
			instr_addi(instr);
			break;
		case 9:
			instr_addiu(instr);
			break;
		case 10:
			instr_slti(instr);
			break;
		case 11:
			instr_sltiu(instr);
			break;
		case 12:
			instr_andi(instr);
			break;
		case 13:
			instr_ori(instr);
			break;
		case 14:
			instr_xori(instr);
			break;
		case 15:
			instr_lui(instr);
			break;
		default:
			m_except = EX_RSVD_INSTR;
			break;
	}
}


inline void cpu_top::execute(uint32_t instr, bool dslot)
{
	instruction iw;
	iw.word = instr;

	switch(iw.r.op) {
		case 0:
			exec_special(instr, dslot);
			break;
		case 1:
			exec_regimm(instr, dslot);
			break;
		case 16:
			exec_cop0(instr, dslot);
			break;
		case 32:
		case 33:
		case 35:
		case 36:
		case 37:
		case 40:
		case 41:
		case 43:
			exec_load_store(instr, dslot);
			break;
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
		case 9:
		case 10:
		case 11:
		case 12:
		case 13:
		case 14:
		case 15:
			exec_other(instr, dslot);
			break;
		default:
			std::cout << "invalid opcode: " << iw.r.op << std::endl;
			m_except = EX_RSVD_INSTR;
			break;
	}
}


inline void cpu_top::cpu_thread(void) {
	wait(nrst.posedge_event());

	while(true) {
		uint32_t instr = fetch_instr(m_pc);
		if(check_except())
			continue;

		execute(instr, false);
		if(check_except())
			continue;

		if(m_delay_slot) {
			m_delay_slot = false;
			instr = fetch_instr(m_pc+4);
			if(check_except())
				continue;
			execute(instr, true);
			if(check_except())
				continue;
		}

		if(check_intr())
			continue;

		m_pc = m_next_pc;
		m_next_pc += 4;
	}

}
