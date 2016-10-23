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

	sc_in<bool>          intr_i;

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
		: m_prid(0x001A8100)
	{
		SC_CTHREAD(cpu_intr_thread, clk.pos());
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
		m_ld_delay_slot = false;
		m_ld_dst_reg = 0;
		m_ld_data = 0;
	}

public:
	// Exported CPU register values for software debug
	sc_signal<sc_uint<32> > cpu_reg_r0_zero;
	sc_signal<sc_uint<32> > cpu_reg_r1_at;
	sc_signal<sc_uint<32> > cpu_reg_r2_v0;
	sc_signal<sc_uint<32> > cpu_reg_r3_v1;
	sc_signal<sc_uint<32> > cpu_reg_r4_a0;
	sc_signal<sc_uint<32> > cpu_reg_r5_a1;
	sc_signal<sc_uint<32> > cpu_reg_r6_a2;
	sc_signal<sc_uint<32> > cpu_reg_r7_a3;
	sc_signal<sc_uint<32> > cpu_reg_r8_t0;
	sc_signal<sc_uint<32> > cpu_reg_r9_t1;
	sc_signal<sc_uint<32> > cpu_reg_r10_t2;
	sc_signal<sc_uint<32> > cpu_reg_r11_t3;
	sc_signal<sc_uint<32> > cpu_reg_r12_t4;
	sc_signal<sc_uint<32> > cpu_reg_r13_t5;
	sc_signal<sc_uint<32> > cpu_reg_r14_t6;
	sc_signal<sc_uint<32> > cpu_reg_r15_t7;
	sc_signal<sc_uint<32> > cpu_reg_r16_s0;
	sc_signal<sc_uint<32> > cpu_reg_r17_s1;
	sc_signal<sc_uint<32> > cpu_reg_r18_s2;
	sc_signal<sc_uint<32> > cpu_reg_r19_s3;
	sc_signal<sc_uint<32> > cpu_reg_r20_s4;
	sc_signal<sc_uint<32> > cpu_reg_r21_s5;
	sc_signal<sc_uint<32> > cpu_reg_r22_s6;
	sc_signal<sc_uint<32> > cpu_reg_r23_s7;
	sc_signal<sc_uint<32> > cpu_reg_r24_t8;
	sc_signal<sc_uint<32> > cpu_reg_r25_t9;
	sc_signal<sc_uint<32> > cpu_reg_r26_k0;
	sc_signal<sc_uint<32> > cpu_reg_r27_k1;
	sc_signal<sc_uint<32> > cpu_reg_r28_gp;
	sc_signal<sc_uint<32> > cpu_reg_r29_sp;
	sc_signal<sc_uint<32> > cpu_reg_r30_s8_fp;
	sc_signal<sc_uint<32> > cpu_reg_r31_ra;
	sc_signal<sc_uint<32> > cpu_reg_pc;
	sc_signal<sc_uint<32> > cpu_reg_hi;
	sc_signal<sc_uint<32> > cpu_reg_lo;
	sc_signal<sc_uint<32> > cpu_reg_prid;
	sc_signal<sc_uint<32> > cpu_reg_epc;
	sc_signal<sc_uint<32> > cpu_reg_sr;
	sc_signal<sc_uint<32> > cpu_reg_psr;
	sc_signal<sc_uint<32> > cpu_reg_ivtb;

private:
	static const unsigned IVT_ENTRY_SZ = 8;	// IVT entry size

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

	//
	// R-Type instruction (Register)
	//
	// 31          26 25       21 20       16 15       11 10        6 5            0
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	// |     op      |    rs     |    rt     |    rd     |   shamt   |    funct    |
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	//
	struct rtype_instr {
		uint32_t func  : 6;
		uint32_t shamt : 5;
		uint32_t rd    : 5;
		uint32_t rt    : 5;
		uint32_t rs    : 5;
		uint32_t op    : 6;
	};

	//
	// I-Type instruction (Immediate)
	//
	// 31          26 25       21 20       16 15                                   0
	// +-------------+-----------+-----------+-------------------------------------+
	// |     op      |    rs     |    rt     |              immediate              |
	// +-------------+-----------+-----------+-------------------------------------+
	//
	struct itype_instr {
		uint32_t imm : 16;
		uint32_t rt  : 5;
		uint32_t rs  : 5;
		uint32_t op  : 6;
	};

	//
	// J-Type instruction (Jump)
	//
	// 31          26 25                                                           0
	// +-------------+-------------------------------------------------------------+
	// |     op      |                         target                              |
	// +-------------+-------------------------------------------------------------+
	//
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
	// Sign extension
	uint32_t sign_extend8(uint32_t imm);
	uint32_t sign_extend16(uint32_t imm);

	// Zero extension
	uint32_t zero_extend8(uint32_t imm);
	uint32_t zero_extend16(uint32_t imm);

	// Read/write general purpose register (GPR)
	uint32_t rd_gpreg(uint32_t r);
	void wr_gpreg(uint32_t r, uint32_t v);

	// Bus operation - fetch instruction
	uint32_t fetch_instr(uint32_t pc);

	// Bus operations - load byte, halfword and word
	uint32_t load_byte(uint32_t addr);
	uint32_t load_halfword(uint32_t addr);
	uint32_t load_word(uint32_t addr);

	// Bus operations - store byte, halfword and word
	void store_byte(uint32_t addr, uint32_t v);
	void store_halfword(uint32_t addr, uint32_t v);
	void store_word(uint32_t addr, uint32_t v);

	// Check exception state
	bool check_except(void);
	// Check external interrupt active state
	bool check_intr(void);

	//// INSTRUCTIONS

	//
	// MFC0  Move From Coprocessor 0
	//
	// 31          26 25       21 20       16 15       11 10        6 5            0
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	// |     COP0    |     MF    |           |           |           |             |
	// | 0 1 0 0 0 0 | 0 0 0 0 0 |    rt     |    rd     | 0 0 0 0 0 | 0 0 0 0 0 0 |
	// |             |           |           |           |           |             |
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	//         6            5          5           5            5           6
	//
	// Format:
	//	MFC0 rt, rd
	// Description: rt <- rd
	//	The contents of coprocessor register rd of coprocessor 0 are loaded
	//	into general register rt.
	// Exceptions:
	//	None
	//
	void instr_mfc0(uint32_t instr);

	//
	// MTC0  Move To Coprocessor 0
	//
	// 31          26 25       21 20       16 15       11 10        6 5            0
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	// |     COP0    |     MT    |           |           |           |             |
	// | 0 1 0 0 0 0 | 0 0 1 0 0 |    rt     |    rd     | 0 0 0 0 0 | 0 0 0 0 0 0 |
	// |             |           |           |           |           |             |
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	//         6            5          5           5            5           6
	//
	// Format:
	//	MTC0 rt, rd
	// Description: rd <- rt
	//	The contents of general register rt are loaded into coprocessor
	//	register rd of coprocessor 0.
	// Exceptions:
	//	None
	//
	void instr_mtc0(uint32_t instr);

	//
	// RFE  Restore From Exception
	//
	// 31          26 25       21 20       16 15       11 10        6 5            0
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	// |     COP0    |     CO    |           |           |           |    RFE      |
	// | 0 1 0 0 0 0 | 1 0 0 0 0 | 0 0 0 0 0 | 0 0 0 0 0 | 0 0 0 0 0 | 0 1 0 0 0 0 |
	// |             |           |           |           |           |             |
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	//         6            5          5           5            5           6
	//
	// Format:
	//	RFE
	// Description: sr <- psr
	//	RFE restores the previous Status Register bits including interrupt
	//	enable mask bit. The MIPS architecture does not specify the operation
	//	of memory references associated with load/store instructions immediately
	//	prior to an RFE instruction. Normally, the RFE instruction follows
	//	in the delay slot of a JR instruction to restore the PC.
	// Exceptions:
	//	None
	//
	void instr_rfe(uint32_t instr);

	//
	// SLL  Shift Word Left Logical
	//
	// 31          26 25       21 20       16 15       11 10        6 5            0
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	// |   SPECIAL   |     0     |           |           |           |    SLL      |
	// | 0 0 0 0 0 0 | 0 0 0 0 0 |    rt     |    rd     |    sa     | 0 0 0 0 0 0 |
	// |             |           |           |           |           |             |
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	//         6            5          5           5            5           6
	//
	// Format:
	//	SLL rd, rt, sa
	// Description: rd <- rt << sa  (logical)
	//	The contents of the low-order word of general register rt are shifted
	//	left by sa bits, inserting zeros into the low-order bits. The word
	//	result is placed in register rd.
	// Exceptions:
	//	None
	//
	void instr_sll(uint32_t instr);

	//
	// SRL  Shift Word Right Logical
	//
	// 31          26 25       21 20       16 15       11 10        6 5            0
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	// |   SPECIAL   |     0     |           |           |           |    SRL      |
	// | 0 0 0 0 0 0 | 0 0 0 0 0 |    rt     |    rd     |    sa     | 0 0 0 0 1 0 |
	// |             |           |           |           |           |             |
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	//         6            5          5           5            5           6
	//
	// Format:
	//	SRL rd, rt, sa
	// Description: rd <- rt >> sa  (logical)
	//	The contents of the low-order word of general register rt are shifted
	//	right by sa bits, inserting zeros into the high-order bits. The word
	//	result is placed in register rd.
	// Exceptions:
	//	None
	//
	void instr_srl(uint32_t instr);

	//
	// SRA  Shift Word Right Arithmetic
	//
	// 31          26 25       21 20       16 15       11 10        6 5            0
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	// |   SPECIAL   |     0     |           |           |           |    SRA      |
	// | 0 0 0 0 0 0 | 0 0 0 0 0 |    rt     |    rd     |    sa     | 0 0 0 0 1 1 |
	// |             |           |           |           |           |             |
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	//         6            5          5           5            5           6
	//
	// Format:
	//	SRA rd, rt, sa
	// Description: rd <- rt >> sa  (arithmetic)
	//	The contents of the low-order word of general register rt are shifted
	//	right by sa bits, sign-extending the high-order bits. The word
	//	result is placed in register rd.
	// Exceptions:
	//	None
	//
	void instr_sra(uint32_t instr);

	//
	// SLLV  Shift Word Left Logical Variable
	//
	// 31          26 25       21 20       16 15       11 10        6 5            0
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	// |   SPECIAL   |           |           |           |     0     |    SLLV     |
	// | 0 0 0 0 0 0 |    rs     |    rt     |    rd     | 0 0 0 0 0 | 0 0 0 1 0 0 |
	// |             |           |           |           |           |             |
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	//         6            5          5           5            5           6
	//
	// Format:
	//	SLLV rd, rt, rs
	// Description: rd <- rt << (rs & 0x1f)  (logical)
	//	The contents of the low-order word of general register rt are shifted
	//	left the number of bits specified by the low-order five bits contained
	//	in general register rs, inserting zeros into the low-order bits.
	//	The word-value result is placed in register rd.
	// Exceptions:
	//	None
	//
	void instr_sllv(uint32_t instr);

	//
	// SRLV  Shift Word Right Logical Variable
	//
	// 31          26 25       21 20       16 15       11 10        6 5            0
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	// |   SPECIAL   |           |           |           |     0     |    SRLV     |
	// | 0 0 0 0 0 0 |    rs     |    rt     |    rd     | 0 0 0 0 0 | 0 0 0 1 1 0 |
	// |             |           |           |           |           |             |
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	//         6            5          5           5            5           6
	//
	// Format:
	//	SRLV rd, rt, rs
	// Description: rd <- rt >> (rs & 0x1f)  (logical)
	//	The low-order word of general register rt are shifted right by the number
	//	of bits specified by the low-order five bits of general register rs,
	//	inserting zeros into the high-order bits. The result is placed in register rd.
	// Exceptions:
	//	None
	//
	void instr_srlv(uint32_t instr);

	//
	// SRAV  Shift Word Right Arithmetic Variable
	//
	// 31          26 25       21 20       16 15       11 10        6 5            0
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	// |   SPECIAL   |           |           |           |     0     |    SRAV     |
	// | 0 0 0 0 0 0 |    rs     |    rt     |    rd     | 0 0 0 0 0 | 0 0 0 1 1 1 |
	// |             |           |           |           |           |             |
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	//         6            5          5           5            5           6
	//
	// Format:
	//	SRAV rd, rt, rs
	// Description: rd <- rt >> (rs & 0x1f)  (arithmetic)
	//	The contents of general register rt are shifted right by the number
	//	of bits specified by the low-order five bits of general register rs,
	//	sign-extending the high-order bits. The result is placed in register rd.
	// Exceptions:
	//	None
	//
	void instr_srav(uint32_t instr);

	//
	// JR  Jump Register
	//
	// 31          26 25       21 20       16 15       11 10        6 5            0
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	// |   SPECIAL   |           |     0     |     0     |     0     |     JR      |
	// | 0 0 0 0 0 0 |    rs     | 0 0 0 0 0 | 0 0 0 0 0 | 0 0 0 0 0 | 0 0 1 0 0 0 |
	// |             |           |           |           |           |             |
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	//         6            5          5           5            5           6
	//
	// Format:
	//	JR rs
	// Description: PC <- rs
	//	The program unconditionally jumps to the address contained in general
	//	register rs, with a delay of one instruction. Since instructions must
	//	be word-aligned, a Jump Register instruction must specify a target
	//	register rs whose two low-order bits are zero. If these low-order bits
	//	are not zero, an address exception will occur when the jump target
	//	instruction is subsequently fetched.
	// Exceptions:
	//	Address error exception
	//
	void instr_jr(uint32_t instr);

	//
	// JALR  Jump And Link Register
	//
	// 31          26 25       21 20       16 15       11 10        6 5            0
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	// |   SPECIAL   |           |     0     |           |     0     |    JALR     |
	// | 0 0 0 0 0 0 |    rs     | 0 0 0 0 0 |    rd     | 0 0 0 0 0 | 0 0 1 0 0 1 |
	// |             |           |           |           |           |             |
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	//         6            5          5           5            5           6
	//
	// Format:
	//	JALR rs    (rd=31 implied)
	//	JALR rd, rs
	// Description: rd = PC+8, PC <- rs
	//	The program unconditionally jumps to the address contained in general
	//	register rs, with a delay of one instruction. The address of the
	//	instruction after the delay slot is placed in general register rd. The
	//	default value of rd, if omitted in the assembly language instruction, is 31.
	//	Register specifiers rs and rd may not be equal, because such an instruction
	//	does not have the same effect when re-executed. However, an attempt to
	//	execute this instruction is not trapped, and the result of executing such
	//	an instruction is undefined.
	//	A Jump and Link Register instruction that uses a register whose low-order
	//	2 bits are non-zero, or specifies an address outside of the accessible
	//	address space, causes an Address Error Exception when the jump is executed.
	//	The Exception PC points to the location of the Jump instruction causing
	//	the error, and the instruction in the delay slot is not executed.
	//	If desired, system software can emulate the delay instruction and advance
	//	the PC to the target of the jump before delivering the exception to the
	//	user program.
	// Exceptions:
	//	Address error exception
	//
	void instr_jalr(uint32_t instr);

	//
	// SYSCALL  System Call
	//
	// 31          26 25                                            6 5            0
	// +-------------+-----------------------------------------------+-------------+
	// |   SPECIAL   |                                               |   SYSCALL   |
	// | 0 0 0 0 0 0 |                     code                      | 0 0 1 1 0 0 |
	// |             |                                               |             |
	// +-------------+-----------------------------------------------+-------------+
	//         6                            20                              6
	//
	// Format:
	//	SYSCALL
	// Description:
	//	A system call exception occurs, immediately and unconditionally
	//	transferring control to the exception handler.
	//	The code field is available for use as software parameters,
	//	but is retrieved by the exception handler only by loading the
	//	contents of the memory word containing the instruction.
	// Exceptions:
	//	System Call exception
	//
	void instr_syscall(uint32_t instr);

	//
	// BREAK  Breakpoint
	//
	// 31          26 25                                            6 5            0
	// +-------------+-----------------------------------------------+-------------+
	// |   SPECIAL   |                                               |    BREAK    |
	// | 0 0 0 0 0 0 |                     code                      | 0 0 1 1 0 1 |
	// |             |                                               |             |
	// +-------------+-----------------------------------------------+-------------+
	//         6                            20                              6
	//
	// Format:
	//	BREAK
	// Description:
	//	A breakpoint trap occurs, immediately and unconditionally transferring
	//	control to the exception handler. The code field is available for use
	//	as software parameters, but is retrieved by the exception handler only
	//	by loading the contents of the memory word containing the instruction.
	// Exceptions:
	//	Breakpoint exception
	//
	void instr_break(uint32_t instr);

	//
	// MFHI  Move From HI
	//
	// 31          26 25       21 20       16 15       11 10        6 5            0
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	// |   SPECIAL   |     0     |     0     |           |     0     |    MFHI     |
	// | 0 0 0 0 0 0 | 0 0 0 0 0 | 0 0 0 0 0 |    rd     | 0 0 0 0 0 | 0 1 0 0 0 0 |
	// |             |           |           |           |           |             |
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	//         6            5          5           5            5           6
	//
	// Format:
	//	MFHI rd
	// Description: rd <- HI
	//	The contents of special register HI are loaded into general register rd.
	//	To ensure proper operation in the event of interruptions, the two
	//	instructions which follow a MFHI instruction may not be any of the
	//	instructions which modify the HI register: MULT, MULTU, DIV,
	//	DIVU, MTHI.
	// Exceptions:
	//	None
	//
	void instr_mfhi(uint32_t instr);

	//
	// MTHI  Move To HI
	//
	// 31          26 25       21 20       16 15       11 10        6 5            0
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	// |   SPECIAL   |           |     0     |     0     |     0     |    MTHI     |
	// | 0 0 0 0 0 0 |    rs     | 0 0 0 0 0 | 0 0 0 0 0 | 0 0 0 0 0 | 0 1 0 0 0 1 |
	// |             |           |           |           |           |             |
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	//         6            5          5           5            5           6
	//
	// Format:
	//	MTHI rs
	// Description: HI <- rs
	//	The contents of general register rs are loaded into special register HI.
	//	Instructions that write to the HI and LO registers are not interlocked
	//	and serialized; a result written to the HI/LO pair must be read before
	//	another result is written. If a MTHI operation is executed following a
	//	MULT, MULTU, DIV, or DIVU instruction, but before any MFLO, MFHI, MTLO,
	//	or MTHI instructions, the contents of the companion special register LO
	//	are undefined.
	// Exceptions:
	//	None
	//
	void instr_mthi(uint32_t instr);

	//
	// MFLO  Move From LO
	//
	// 31          26 25       21 20       16 15       11 10        6 5            0
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	// |   SPECIAL   |     0     |     0     |           |     0     |    MFLO     |
	// | 0 0 0 0 0 0 | 0 0 0 0 0 | 0 0 0 0 0 |    rd     | 0 0 0 0 0 | 0 1 0 0 1 0 |
	// |             |           |           |           |           |             |
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	//         6            5          5           5            5           6
	//
	// Format:
	//	MFLO rd
	// Description: rd <- LO
	//	The contents of special register LO are loaded into general register rd.
	//	To ensure proper operation in the event of interruptions, the two
	//	instructions which follow a MFLO instruction may not be any of the
	//	instructions which modify the LO register: MULT, MULTU, DIV, DIVU, MTLO.
	// Exceptions:
	//	None
	//
	void instr_mflo(uint32_t instr);

	//
	// MTLO  Move To LO
	//
	// 31          26 25       21 20       16 15       11 10        6 5            0
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	// |   SPECIAL   |           |     0     |     0     |     0     |    MTLO     |
	// | 0 0 0 0 0 0 |    rs     | 0 0 0 0 0 | 0 0 0 0 0 | 0 0 0 0 0 | 0 1 0 0 1 1 |
	// |             |           |           |           |           |             |
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	//         6            5          5           5            5           6
	//
	// Format:
	//	MTLO rs
	// Description: LO <- rs
	//	The contents of general register rs are loaded into special register LO.
	//	Instructions that write to the HI and LO registers are not interlocked
	//	and serialized; a result written to the HI/LO pair must be read before
	//	another result is written. If a MTLO operation is executed following a
	//	MULT, MULTU, DIV, or DIVU instruction, but before any MFLO, MFHI, MTLO,
	//	or MTHI instructions, the contents of the companion special register HI
	//	are undefined.
	// Exceptions:
	//	None
	//
	void instr_mtlo(uint32_t instr);

	//
	// MULT  Multiply Word
	//
	// 31          26 25       21 20       16 15       11 10        6 5            0
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	// |   SPECIAL   |           |           |     0     |     0     |    MULT     |
	// | 0 0 0 0 0 0 |    rs     |    rt     | 0 0 0 0 0 | 0 0 0 0 0 | 0 1 1 0 0 0 |
	// |             |           |           |           |           |             |
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	//         6            5          5           5            5           6
	//
	// Format:
	//	MULT rs, rt
	// Description: (LO, HI) <- rs * rt  (signed)
	//	The contents of general registers rs and rt are multiplied, treating
	//	both operands as 32-bit 2’s complement values. No integer overflow
	//	exception occurs under any circumstances. When the operation completes,
	//	the low-order word of the double result is loaded into special register LO,
	//	and the high-order word of the double result is loaded into special
	//	register HI. If either of the two preceding instructions is MFHI or MFLO,
	//	the results of these instructions are undefined. Correct operation
	//	requires separating reads of HI or LO from writes by a minimum of
	//	two other instructions.
	// Exceptions:
	//	None
	//
	void instr_mult(uint32_t instr);

	//
	// MULTU  Multiply Unsigned Word
	//
	// 31          26 25       21 20       16 15       11 10        6 5            0
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	// |   SPECIAL   |           |           |     0     |     0     |    MULTU    |
	// | 0 0 0 0 0 0 |    rs     |    rt     | 0 0 0 0 0 | 0 0 0 0 0 | 0 1 1 0 0 1 |
	// |             |           |           |           |           |             |
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	//         6            5          5           5            5           6
	//
	// Format:
	//	MULTU rs, rt
	// Description: (LO, HI) <- rs * rt  (unsigned)
	//	The contents of general register rs and the contents of general register rt
	//	are multiplied, treating both operands as unsigned values. No overflow
	//	exception occurs under any circumstances. When the operation completes,
	//	the low-order word of the double result is loaded into special register LO,
	//	and the high-order word of the double result is loaded into special
	//	register HI. If either of the two preceding instructions is MFHI or MFLO,
	//	the results of these instructions are undefined. Correct operation requires
	//	separating reads of HI or LO from writes by a minimum of two instructions.
	// Exceptions:
	//	None
	//
	void instr_multu(uint32_t instr);

	//
	// DIV  Divide Word
	//
	// 31          26 25       21 20       16 15       11 10        6 5            0
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	// |   SPECIAL   |           |           |     0     |     0     |     DIV     |
	// | 0 0 0 0 0 0 |    rs     |    rt     | 0 0 0 0 0 | 0 0 0 0 0 | 0 1 1 0 1 0 |
	// |             |           |           |           |           |             |
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	//         6            5          5           5            5           6
	//
	// Format:
	//	DIV rs, rt
	// Description: LO <- rs / rt,  HI <- rs % rt  (signed)
	//	The contents of general register rs are divided by the contents of
	//	general register rt, treating both operands as 2’s complement values.
	//	No overflow exception occurs under any circumstances, and the result
	//	of this operation is undefined when the divisor is zero. This instruction
	//	is typically followed by additional instructions to check for a zero
	//	divisor and for overflow.
	//	When the operation completes, the quotient word of the double result
	//	is loaded into special register LO, and the remainder word of the double
	//	result is loaded into special register HI. If either of the two preceding
	//	instructions is MFHI or MFLO, the results of those instructions are
	//	undefined. Correct operation requires separating reads of HI or LO
	//	from writes by two or more instructions.
	// Exceptions:
	//	None
	//
	void instr_div(uint32_t instr);

	//
	// DIVU  Divide Unsigned Word
	//
	// 31          26 25       21 20       16 15       11 10        6 5            0
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	// |   SPECIAL   |           |           |     0     |     0     |    DIVU     |
	// | 0 0 0 0 0 0 |    rs     |    rt     | 0 0 0 0 0 | 0 0 0 0 0 | 0 1 1 0 1 1 |
	// |             |           |           |           |           |             |
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	//         6            5          5           5            5           6
	//
	// Format:
	//	DIVU rs, rt
	// Description: LO <- rs / rt,  HI <- rs % rt  (unsigned)
	//	The contents of general register rs are divided by the contents of
	//	general register rt, treating both operands as unsigned values. No
	//	integer overflow exception occurs under any circumstances, and the
	//	result of this operation is undefined when the divisor is zero.
	//	This instruction is typically followed by additional instructions to
	//	check for a zero divisor. When the operation completes, the quotient
	//	word of the double result is loaded into special register LO, and the
	//	remainder word of the double result is loaded into special register HI.
	//	If either of the two preceding instructions is MFHI or MFLO, the results
	//	of those instructions are undefined. Correct operation requires
	//	separating reads of HI or LO from writes by two or more
	//	instructions.
	// Exceptions:
	//	None
	//
	void instr_divu(uint32_t instr);

	//
	// ADD  Add Word
	//
	// 31          26 25       21 20       16 15       11 10        6 5            0
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	// |   SPECIAL   |           |           |           |     0     |     ADD     |
	// | 0 0 0 0 0 0 |    rs     |    rt     |    rd     | 0 0 0 0 0 | 1 0 0 0 0 0 |
	// |             |           |           |           |           |             |
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	//         6            5          5           5            5           6
	//
	// Format:
	//	ADD rd, rs, rt
	// Description: rd <- rs + rt
	//	The word value in general register rt is added to the word value in
	//	general register rs and the result word value is placed into general
	//	register rd.If the addition results in 32-bit 2’s complement arithmetic
	//	overflow (carries out of bits 30 and 31 differ) then the destination
	//	register rd is not modified and an integer overflow exception occurs.
	// Exceptions:
	//	Integer overflow exception
	//
	void instr_add(uint32_t instr);

	//
	// ADDU  Add Unsigned Word
	//
	// 31          26 25       21 20       16 15       11 10        6 5            0
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	// |   SPECIAL   |           |           |           |     0     |    ADDU     |
	// | 0 0 0 0 0 0 |    rs     |    rt     |    rd     | 0 0 0 0 0 | 1 0 0 0 0 1 |
	// |             |           |           |           |           |             |
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	//         6            5          5           5            5           6
	//
	// Format:
	//	ADDU rd, rs, rt
	// Description: rd <- rs + rt
	//	Add two 32-bit values and produce a 32-bit result; arithmetic overflow
	//	is ignored (does not cause an exception). The word value in general
	//	register rt is added to the word value in general register rs and the
	//	result word value is placed into general register rd. ADDU differs
	//	from ADD only when an arithmetic overflow occurs. If the addition
	//	results in 32-bit 2’s complement overflow (carries out of bits 30 and
	//	31 differ), the result word value is placed into register rd and no
	//	exception occurs.
	// Exceptions:
	//	None
	//
	void instr_addu(uint32_t instr);

	//
	// SUB  Subtract Word
	//
	// 31          26 25       21 20       16 15       11 10        6 5            0
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	// |   SPECIAL   |           |           |           |     0     |     SUB     |
	// | 0 0 0 0 0 0 |    rs     |    rt     |    rd     | 0 0 0 0 0 | 1 0 0 0 1 0 |
	// |             |           |           |           |           |             |
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	//         6            5          5           5            5           6
	//
	// Format:
	//	SUB rd, rs, rt
	// Description: rd <- rs - rt
	//	The contents of general register rt are subtracted from the contents of
	//	general register rs to form a result. The result is placed into general
	//	register rd. The only difference between this instruction and the SUBU
	//	instruction is that SUBU never traps on overflow. An integer overflow
	//	exception takes place if the carries out of bits 30 and 31 differ
	//	(2’s complement overflow). The destination register rd is not modified
	//	when an integer overflow exception occurs.
	// Exceptions:
	//	Integer overflow exception
	//
	void instr_sub(uint32_t instr);

	//
	// SUBU  Subtract Unsigned Word
	//
	// 31          26 25       21 20       16 15       11 10        6 5            0
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	// |   SPECIAL   |           |           |           |     0     |    SUBU     |
	// | 0 0 0 0 0 0 |    rs     |    rt     |    rd     | 0 0 0 0 0 | 1 0 0 0 1 1 |
	// |             |           |           |           |           |             |
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	//         6            5          5           5            5           6
	//
	// Format:
	//	SUBU rd, rs, rt
	// Description: rd <- rs - rt
	//	The contents of general register rt are subtracted from the contents
	//	of general register rs to form a result. The result is placed into
	//	general register rd. The only difference between this instruction
	//	and the SUB instruction is that SUBU never traps on overflow. No
	//	integer overflow exception occurs under any circumstances.
	// Exceptions:
	//	None
	//
	void instr_subu(uint32_t instr);

	//
	// AND  And
	//
	// 31          26 25       21 20       16 15       11 10        6 5            0
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	// |   SPECIAL   |           |           |           |     0     |     AND     |
	// | 0 0 0 0 0 0 |    rs     |    rt     |    rd     | 0 0 0 0 0 | 1 0 0 1 0 0 |
	// |             |           |           |           |           |             |
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	//         6            5          5           5            5           6
	//
	// Format:
	//	AND rd, rs, rt
	// Description: rd <- rs & rt
	//	The contents of general register rs are combined with the contents
	//	of general register rt in a bit-wise logical AND operation. The
	//	result is placed into general register rd.
	// Exceptions:
	//	None
	//
	void instr_and(uint32_t instr);

	//
	// OR  Or
	//
	// 31          26 25       21 20       16 15       11 10        6 5            0
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	// |   SPECIAL   |           |           |           |     0     |     OR      |
	// | 0 0 0 0 0 0 |    rs     |    rt     |    rd     | 0 0 0 0 0 | 1 0 0 1 0 1 |
	// |             |           |           |           |           |             |
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	//         6            5          5           5            5           6
	//
	// Format:
	//	OR rd, rs, rt
	// Description: rd <- rs | rt
	//	The contents of general register rs are combined with the contents
	//	of general register rt in a bit-wise logical OR operation. The result
	//	is placed into general register rd.
	// Exceptions:
	//	None
	//
	void instr_or(uint32_t instr);

	//
	// XOR  Exclusive Or
	//
	// 31          26 25       21 20       16 15       11 10        6 5            0
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	// |   SPECIAL   |           |           |           |     0     |     XOR     |
	// | 0 0 0 0 0 0 |    rs     |    rt     |    rd     | 0 0 0 0 0 | 1 0 0 1 1 0 |
	// |             |           |           |           |           |             |
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	//         6            5          5           5            5           6
	//
	// Format:
	//	XOR rd, rs, rt
	// Description: rd <- rs ^ rt
	//	The contents of general register rs are combined with the contents
	//	of general register rt in a bit-wise logical exclusive OR operation.
	//	The result is placed into general register rd.
	// Exceptions:
	//	None
	//
	void instr_xor(uint32_t instr);

	//
	// NOR  Not Or
	//
	// 31          26 25       21 20       16 15       11 10        6 5            0
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	// |   SPECIAL   |           |           |           |     0     |     NOR     |
	// | 0 0 0 0 0 0 |    rs     |    rt     |    rd     | 0 0 0 0 0 | 1 0 0 1 1 1 |
	// |             |           |           |           |           |             |
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	//         6            5          5           5            5           6
	//
	// Format:
	//	NOR rd, rs, rt
	// Description: rd <- !(rs | rt)
	//	The contents of general register rs are combined with the contents
	//	of general register rt in a bit-wise logical NOR operation. The result
	//	is placed into general register rd.
	// Exceptions:
	//	None
	//
	void instr_nor(uint32_t instr);

	//
	// SLT  Set On Less Than
	//
	// 31          26 25       21 20       16 15       11 10        6 5            0
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	// |   SPECIAL   |           |           |           |     0     |     SLT     |
	// | 0 0 0 0 0 0 |    rs     |    rt     |    rd     | 0 0 0 0 0 | 1 0 1 0 1 0 |
	// |             |           |           |           |           |             |
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	//         6            5          5           5            5           6
	//
	// Format:
	//	SLT rd, rs, rt
	// Description: rd <- (rs < rt)   (signed)
	//	The contents of general register rt are subtracted from the contents
	//	of general register rs. Considering both quantities as signed integers,
	//	if the contents of general register rs are less than the contents of
	//	general register rt, the result is set to one; otherwise the result
	//	is set to zero. The result is placed into general register rd.
	//	No integer overflow exception occurs under any circumstances. The
	//	comparison is valid even if the subtraction used during the comparison
	//	overflows.
	// Exceptions:
	//	None
	//
	void instr_slt(uint32_t instr);

	//
	// SLTU  Set On Less Than Unsigned
	//
	// 31          26 25       21 20       16 15       11 10        6 5            0
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	// |   SPECIAL   |           |           |           |     0     |     SLTU    |
	// | 0 0 0 0 0 0 |    rs     |    rt     |    rd     | 0 0 0 0 0 | 1 0 1 0 1 1 |
	// |             |           |           |           |           |             |
	// +-------------+-----------+-----------+-----------+-----------+-------------+
	//         6            5          5           5            5           6
	//
	// Format:
	//	SLTU rd, rs, rt
	// Description: rd <- (rs < rt)   (unsigned)
	//	The contents of general register rt are subtracted from the contents
	//	of general register rs. Considering both quantities as unsigned
	//	integers, if the contents of general register rs are less than the
	//	contents of general register rt, the result is set to one; otherwise
	//	the result is set to zero. The result is placed into general register rd.
	//	No integer overflow exception occurs under any circumstances. The
	//	comparison is valid even if the subtraction used during the
	//	comparison overflows.
	// Exceptions:
	//	None
	//
	void instr_sltu(uint32_t instr);

	//
	// BLTZ  Branch On Less Than Zero
	//
	// 31          26 25       21 20       16 15                                   0
	// +-------------+-----------+-----------+-------------------------------------+
	// |    REGIMM   |           |   BLTZ    |                                     |
	// | 0 0 0 0 0 1 |    rs     | 0 0 0 0 0 |              offset                 |
	// |             |           |           |                                     |
	// +-------------+-----------+-----------+-------------------------------------+
	//         6            5          5                         16
	//
	// Format:
	//	BLTZ rs, offset
	// Description: if (rs<0) then branch
	//	A branch target address is computed from the sum of the address of the
	//	instruction in the delay slot and the 16-bit offset, shifted left two
	//	bits and sign-extended. If the contents of general register rs have the
	//	sign bit set, then the program branches to the target address, with a
	//	delay of one instruction.
	// Exceptions:
	//	None
	//
	void instr_bltz(uint32_t instr);

	//
	// BGEZ  Branch On Greater Than Or Equal To Zero
	//
	// 31          26 25       21 20       16 15                                   0
	// +-------------+-----------+-----------+-------------------------------------+
	// |    REGIMM   |           |   BGEZ    |                                     |
	// | 0 0 0 0 0 1 |    rs     | 0 0 0 0 1 |              offset                 |
	// |             |           |           |                                     |
	// +-------------+-----------+-----------+-------------------------------------+
	//         6            5          5                         16
	//
	// Format:
	//	BGEZ rs, offset
	// Description: if (rs>=0) then branch
	//	A branch target address is computed from the sum of the address
	//	of the instruction in the delay slot and the 16-bit offset, shifted
	//	left two bits and sign-extended. If the contents of general register rs
	//	have the sign bit cleared, then the program branches to the target
	//	address, with a delay of one instruction.
	// Exceptions:
	//	None
	//
	void instr_bgez(uint32_t instr);

	//
	// BLTZAL  Branch On Less Than Zero And Link
	//
	// 31          26 25       21 20       16 15                                   0
	// +-------------+-----------+-----------+-------------------------------------+
	// |    REGIMM   |           |  BLTZAL   |                                     |
	// | 0 0 0 0 0 1 |    rs     | 1 0 0 0 0 |              offset                 |
	// |             |           |           |                                     |
	// +-------------+-----------+-----------+-------------------------------------+
	//         6            5          5                         16
	//
	// Format:
	//	BLTZAL rs, offset
	// Description: r31 <- PC + 8, if (rs<0) then branch
	//	A branch target address is computed from the sum of the address of
	//	the instruction in the delay slot and the 16-bit offset, shifted
	//	left two bits and sign-extended. Unconditionally, the address of the
	//	instruction after the delay slot is placed in the link register, r31.
	//	If the contents of general register rs have the sign bit set, then
	//	the program branches to the target address, with a delay of one
	//	instruction. General register rs may not be general register 31,
	//	because such an instruction is not restartable. An attempt to execute
	//	this instruction with register 31 specified as rs is not trapped,
	//	however.
	// Exceptions:
	//	None
	//
	void instr_bltzal(uint32_t instr);

	//
	// BGEZAL  Branch On Greater Than Or Equal To Zero And Link
	//
	// 31          26 25       21 20       16 15                                   0
	// +-------------+-----------+-----------+-------------------------------------+
	// |    REGIMM   |           |  BGEZAL   |                                     |
	// | 0 0 0 0 0 1 |    rs     | 1 0 0 0 1 |              offset                 |
	// |             |           |           |                                     |
	// +-------------+-----------+-----------+-------------------------------------+
	//         6            5          5                         16
	//
	// Format:
	//	BGEZAL rs, offset
	// Description: r31 <- PC + 8, if (rs>=0) then branch
	//	A branch target address is computed from the sum of the address of
	//	the instruction in the delay slot and the 16-bit offset, shifted
	//	left two bits and sign-extended. Unconditionally, the address of the
	//	instruction after the delay slot is placed in the link register, r31.
	//	If the contents of general register rs have the sign bit cleared,
	//	then the program branches to the target address, with a delay of one
	//	instruction. General register rs may not be general register 31,
	//	because such an instruction is not restartable. An attempt to execute
	//	this instruction is not trapped, however.
	// Exceptions:
	//	None
	//
	void instr_bgezal(uint32_t instr);

	//
	// J  Jump
	//
	// 31          26 25                                                           0
	// +-------------+-------------------------------------------------------------+
	// |      J      |                                                             |
	// | 0 0 0 0 1 0 |                       target                                |
	// |             |                                                             |
	// +-------------+-------------------------------------------------------------+
	//         6                                   26
	//
	// Format:
	//	J target
	// Description:
	//	The 26-bit target address is shifted left two bits and combined
	//	with the high-order bits of the address of the delay slot. The
	//	program unconditionally jumps to this calculated address with a
	//	delay of one instruction.
	// Exceptions:
	//	None
	//
	void instr_j(uint32_t instr);

	//
	// JAL  Jump And Link
	//
	// 31          26 25                                                           0
	// +-------------+-------------------------------------------------------------+
	// |     JAL     |                                                             |
	// | 0 0 0 0 1 1 |                       target                                |
	// |             |                                                             |
	// +-------------+-------------------------------------------------------------+
	//         6                                   26
	//
	// Format:
	//	JAL target
	// Description:
	//	The 26-bit target address is shifted left two bits and combined
	//	with the high-order bits of the address of the delay slot. The
	//	program unconditionally jumps to this calculated address with a
	//	delay of one instruction. The address of the instruction after the
	//	delay slot is placed in the link register, r31.
	// Exceptions:
	//	None
	//
	void instr_jal(uint32_t instr);

	//
	// BEQ  Branch On Equal
	//
	// 31          26 25       21 20       16 15                                   0
	// +-------------+-----------+-----------+-------------------------------------+
	// |     BEQ     |           |           |                                     |
	// | 0 0 0 1 0 0 |    rs     |    rt     |              offset                 |
	// |             |           |           |                                     |
	// +-------------+-----------+-----------+-------------------------------------+
	//         6            5          5                         16
	//
	// Format:
	//	BEQ rs, rt, offset
	// Description: if (rs==rt) then branch
	//	A branch target address is computed from the sum of the address of
	//	the instruction in the delay slot and the 16-bit offset, shifted
	//	left two bits and sign-extended. The contents of general register rs
	//	and the contents of general register rt are compared. If the two
	//	registers are equal, then the program branches to the target address,
	//	with a delay of one instruction.
	// Exceptions:
	//	None
	//
	void instr_beq(uint32_t instr);

	//
	// BNE  Branch On Not Equal
	//
	// 31          26 25       21 20       16 15                                   0
	// +-------------+-----------+-----------+-------------------------------------+
	// |     BNE     |           |           |                                     |
	// | 0 0 0 1 0 1 |    rs     |    rt     |              offset                 |
	// |             |           |           |                                     |
	// +-------------+-----------+-----------+-------------------------------------+
	//         6            5          5                         16
	//
	// Format:
	//	BNE rs, rt, offset
	// Description: if (rs!=rt) then branch
	//	A branch target address is computed from the sum of the address of
	//	the instruction in the delay slot and the 16-bit offset, shifted left
	//	two bits and sign-extended. The contents of general register rs and
	//	the contents of general register rt are compared. If the two registers
	//	are not equal, then the program branches to the target address, with
	//	a delay of one instruction.
	// Exceptions:
	//	None
	//
	void instr_bne(uint32_t instr);

	//
	// BLEZ  Branch on Less Than Or Equal To Zero
	//
	// 31          26 25       21 20       16 15                                   0
	// +-------------+-----------+-----------+-------------------------------------+
	// |     BLEZ    |           |     0     |                                     |
	// | 0 0 0 1 1 0 |    rs     | 0 0 0 0 0 |              offset                 |
	// |             |           |           |                                     |
	// +-------------+-----------+-----------+-------------------------------------+
	//         6            5          5                         16
	//
	// Format:
	//	BLEZ rs, offset
	// Description: if (rs<=0) then branch
	//	A branch target address is computed from the sum of the address of
	//	the instruction in the delay slot and the 16-bit offset, shifted left
	//	two bits and sign-extended. The contents of general register rs are
	//	compared to zero. If the contents of general register rs have the sign
	//	bit set, or are equal to zero, then the program branches to the target
	//	address, with a delay of one instruction.
	// Exceptions:
	//	None
	//
	void instr_blez(uint32_t instr);

	//
	// BGTZ  Branch On Greater Than Zero
	//
	// 31          26 25       21 20       16 15                                   0
	// +-------------+-----------+-----------+-------------------------------------+
	// |     BGTZ    |           |     0     |                                     |
	// | 0 0 0 1 1 1 |    rs     | 0 0 0 0 0 |              offset                 |
	// |             |           |           |                                     |
	// +-------------+-----------+-----------+-------------------------------------+
	//         6            5          5                         16
	//
	// Format:
	//	BGTZ rs, offset
	// Description: if (rs>0) then branch
	//	A branch target address is computed from the sum of the address of
	//	the instruction in the delay slot and the 16-bit offset, shifted left
	//	two bits and sign-extended. The contents of general register rs are
	//	compared to zero. If the contents of general register rs have the sign
	//	bit cleared and are not equal to zero, then the program branches to
	//	the target address, with a delay of one instruction.
	// Exceptions:
	//	None
	//
	void instr_bgtz(uint32_t instr);

	//
	// ADDI  Add Immediate Word
	//
	// 31          26 25       21 20       16 15                                   0
	// +-------------+-----------+-----------+-------------------------------------+
	// |     ADDI    |           |           |                                     |
	// | 0 0 1 0 0 0 |    rs     |    rt     |              immediate              |
	// |             |           |           |                                     |
	// +-------------+-----------+-----------+-------------------------------------+
	//         6            5          5                         16
	//
	// Format:
	//	ADDI rt, rs, immediate
	// Description: rt <- rs + immediate
	//	The 16-bit immediate is sign-extended and added to the contents
	//	of general register rs to form the result. The result is placed
	//	into general register rt. An overflow exception occurs if carries
	//	out of bits 30 and 31 differ (2’s complement overflow). The
	//	destination register rt is not modified when an integer overflow
	//	exception occurs.
	// Exceptions:
	//	Integer overflow exception
	//
	void instr_addi(uint32_t instr);

	//
	// ADDIU  Add Immediate Unsigned Word
	//
	// 31          26 25       21 20       16 15                                   0
	// +-------------+-----------+-----------+-------------------------------------+
	// |    ADDIU    |           |           |                                     |
	// | 0 0 1 0 0 1 |    rs     |    rt     |              immediate              |
	// |             |           |           |                                     |
	// +-------------+-----------+-----------+-------------------------------------+
	//         6            5          5                         16
	//
	// Format:
	//	ADDIU rt, rs, immediate
	// Description: rt <- rs + immediate
	//	The 16-bit immediate is sign-extended and added to the contents of
	//	general register rs to form the result. The result is placed into
	//	general register rt. No integer overflow exception occurs under any
	//	circumstances. The only difference between this instruction and the
	//	ADDI instruction is that ADDIU never causes an overflow exception.
	// Exceptions:
	//	None
	//
	void instr_addiu(uint32_t instr);

	//
	// SLTI  Set On Less Than Immediate
	//
	// 31          26 25       21 20       16 15                                   0
	// +-------------+-----------+-----------+-------------------------------------+
	// |     SLTI    |           |           |                                     |
	// | 0 0 1 0 1 0 |    rs     |    rt     |              immediate              |
	// |             |           |           |                                     |
	// +-------------+-----------+-----------+-------------------------------------+
	//         6            5          5                         16
	//
	// Format:
	//	SLTI rt, rs, immediate
	// Description: rt <- (rs < immediate)  (signed)
	//	The 16-bit immediate is sign-extended and subtracted from the contents
	//	of general register rs. Considering both quantities as signed integers,
	//	if rs is less than the sign-extended immediate, the result is set to
	//	one; otherwise the result is set to zero. The result is placed into
	//	general register rt. No integer overflow exception occurs under any
	//	circumstances. The comparison is valid even if the subtraction used
	//	during the comparison overflows.
	// Exceptions:
	//	None
	//
	void instr_slti(uint32_t instr);

	//
	// SLTIU  Set On Less Than Immediate Unsigned
	//
	// 31          26 25       21 20       16 15                                   0
	// +-------------+-----------+-----------+-------------------------------------+
	// |    SLTIU    |           |           |                                     |
	// | 0 0 1 0 1 1 |    rs     |    rt     |              immediate              |
	// |             |           |           |                                     |
	// +-------------+-----------+-----------+-------------------------------------+
	//         6            5          5                         16
	//
	// Format:
	//	SLTIU rt, rs, immediate
	// Description: rt <- (rs < immediate)  (unsigned)
	//	The 16-bit immediate is sign-extended and subtracted from the contents
	//	of general register rs. Considering both quantities as unsigned integers,
	//	if rs is less than the sign-extended immediate, the result is set to one;
	//	otherwise the result is set to zero. The result is placed into general
	//	register rt. No integer overflow exception occurs under any circumstances.
	//	The comparison is valid even if the subtraction used during the comparison
	//	overflows.
	// Exceptions:
	//	None
	//
	void instr_sltiu(uint32_t instr);

	//
	// ANDI  And Immediate
	//
	// 31          26 25       21 20       16 15                                   0
	// +-------------+-----------+-----------+-------------------------------------+
	// |     ANDI    |           |           |                                     |
	// | 0 0 1 1 0 0 |    rs     |    rt     |              immediate              |
	// |             |           |           |                                     |
	// +-------------+-----------+-----------+-------------------------------------+
	//         6            5          5                         16
	//
	// Format:
	//	ANDI rt, rs, immediate
	// Description: rt <- rs & immediate
	//	The 16-bit immediate is zero-extended and combined with the contents
	//	of general register rs in a bit-wise logical AND operation. The result
	//	is placed into general register rt.
	// Exceptions:
	//	None
	//
	void instr_andi(uint32_t instr);

	//
	// ORI  Or Immediate
	//
	// 31          26 25       21 20       16 15                                   0
	// +-------------+-----------+-----------+-------------------------------------+
	// |      ORI    |           |           |                                     |
	// | 0 0 1 1 0 1 |    rs     |    rt     |              immediate              |
	// |             |           |           |                                     |
	// +-------------+-----------+-----------+-------------------------------------+
	//         6            5          5                         16
	//
	// Format:
	//	ORI rt, rs, immediate
	// Description: rt <- rs | immediate
	//	The 16-bit immediate is zero-extended and combined with the contents of
	//	general register rs in a bit-wise logical OR operation. The result is
	//	placed into general register rt.
	// Exceptions:
	//	None
	//
	void instr_ori(uint32_t instr);

	//
	// XORI  Exclusive Or Immediate
	//
	// 31          26 25       21 20       16 15                                   0
	// +-------------+-----------+-----------+-------------------------------------+
	// |     XORI    |           |           |                                     |
	// | 0 0 1 1 1 0 |    rs     |    rt     |              immediate              |
	// |             |           |           |                                     |
	// +-------------+-----------+-----------+-------------------------------------+
	//         6            5          5                         16
	//
	// Format:
	//	XORI rt, rs, immediate
	// Description: rt <- rs ^ immediate
	//	The 16-bit immediate is zero-extended and combined with the contents of
	//	general register rs in a bit-wise logical exclusive OR operation.
	//	The result is placed into general register rt.
	// Exceptions:
	//	None
	//
	void instr_xori(uint32_t instr);

	//
	// LUI  Load Upper Immediate
	//
	// 31          26 25       21 20       16 15                                   0
	// +-------------+-----------+-----------+-------------------------------------+
	// |     LUI     |     0     |           |                                     |
	// | 0 0 1 1 1 1 | 0 0 0 0 0 |    rt     |              immediate              |
	// |             |           |           |                                     |
	// +-------------+-----------+-----------+-------------------------------------+
	//         6            5          5                         16
	//
	// Format:
	//	LUI rt, immediate
	// Description: rt <- immediate << 16
	//	The 16-bit immediate is shifted left 16 bits and concatenated with
	//	16 bits of low-order zeros. The 32-bit result is then placed into
	//	general register rt.
	// Exceptions:
	//	None
	//
	void instr_lui(uint32_t instr);

	//
	// LB  Load Byte
	//
	// 31          26 25       21 20       16 15                                   0
	// +-------------+-----------+-----------+-------------------------------------+
	// |      LB     |           |           |                                     |
	// | 1 0 0 0 0 0 |    base   |    rt     |              immediate              |
	// |             |           |           |                                     |
	// +-------------+-----------+-----------+-------------------------------------+
	//         6            5          5                         16
	//
	// Format:
	//	LB rt, offset(base)
	// Description: rt <- offset(base)
	//	The 16-bit offset is sign-extended and added to the contents of general
	//	register base to form an address. The contents of the byte at the
	//	memory location specified by the effective address are sign-extended and
	//	loaded into general register rt.
	// Exceptions:
	//	Bus error exception
	//
	void instr_lb(uint32_t instr);

	//
	// LH  Load Halfword
	//
	// 31          26 25       21 20       16 15                                   0
	// +-------------+-----------+-----------+-------------------------------------+
	// |      LH     |           |           |                                     |
	// | 1 0 0 0 0 1 |    base   |    rt     |              immediate              |
	// |             |           |           |                                     |
	// +-------------+-----------+-----------+-------------------------------------+
	//         6            5          5                         16
	//
	// Format:
	//	LH rt, offset(base)
	// Description: rt <- offset(base)
	//	The 16-bit offset is sign-extended and added to the contents of general
	//	register base to form an address. The contents of the halfword at the
	//	memory location specified by the effective address are sign-extended and
	//	loaded into general register rt. If the least-significant bit of the effective
	//	address is non-zero, an address error exception occurs.
	// Exceptions:
	//	Bus error exception
	//	Address error exception
	//
	void instr_lh(uint32_t instr);

	//
	// LW  Load Word
	//
	// 31          26 25       21 20       16 15                                   0
	// +-------------+-----------+-----------+-------------------------------------+
	// |      LW     |           |           |                                     |
	// | 1 0 0 0 1 1 |    base   |    rt     |              immediate              |
	// |             |           |           |                                     |
	// +-------------+-----------+-----------+-------------------------------------+
	//         6            5          5                         16
	//
	// Format:
	//	LW rt, offset(base)
	// Description: rt <- offset(base)
	//	The 16-bit offset is sign-extended and added to the contents of general
	//	register base to form an address. The contents of the word at the memory
	//	location specified by the effective address are loaded into general
	//	register rt. If either of the two least-significant bits of the effective
	//	address is non-zero, an address error exception occurs.
	// Exceptions:
	//	Bus error exception
	//	Address error exception
	//
	void instr_lw(uint32_t instr);

	//
	// LBU  Load Byte Unsigned
	//
	// 31          26 25       21 20       16 15                                   0
	// +-------------+-----------+-----------+-------------------------------------+
	// |     LBU     |           |           |                                     |
	// | 1 0 0 1 0 0 |    base   |    rt     |              immediate              |
	// |             |           |           |                                     |
	// +-------------+-----------+-----------+-------------------------------------+
	//         6            5          5                         16
	//
	// Format:
	//	LBU rt, offset(base)
	// Description: rt <- offset(base)
	//	The 16-bit offset is sign-extended and added to the contents of general
	//	register base to form an address. The contents of the byte at the memory
	//	location specified by the effective address are zero-extended and loaded
	//	into general register rt.
	// Exceptions:
	//	Bus error exception
	//
	void instr_lbu(uint32_t instr);

	//
	// LHU  Load Halfword Unsigned
	//
	// 31          26 25       21 20       16 15                                   0
	// +-------------+-----------+-----------+-------------------------------------+
	// |     LHU     |           |           |                                     |
	// | 1 0 0 1 0 1 |    base   |    rt     |              immediate              |
	// |             |           |           |                                     |
	// +-------------+-----------+-----------+-------------------------------------+
	//         6            5          5                         16
	//
	// Format:
	//	LHU rt, offset(base)
	// Description: rt <- offset(base)
	//	The 16-bit offset is sign-extended and added to the contents of general
	//	register base to form an address. The contents of the halfword at the
	//	memory location specified by the effective address are zero-extended
	//	and loaded into general register rt. If the least-significant bit of
	//	the effective address is non-zero, an address error exception occurs.
	// Exceptions:
	//	Bus error exception
	//	Address error exception
	//
	void instr_lhu(uint32_t instr);

	//
	// SB  Store Byte
	//
	// 31          26 25       21 20       16 15                                   0
	// +-------------+-----------+-----------+-------------------------------------+
	// |     SB      |           |           |                                     |
	// | 1 0 1 0 0 0 |    base   |    rt     |              immediate              |
	// |             |           |           |                                     |
	// +-------------+-----------+-----------+-------------------------------------+
	//         6            5          5                         16
	//
	// Format:
	//	SB rt, offset(base)
	// Description: offset(base) <- rt
	//	The 16-bit offset is sign-extended and added to the contents of general
	//	register base to form an address. The least-significant byte of register
	//	rt is stored at the effective address.
	// Exceptions:
	//	Bus error exception
	//
	void instr_sb(uint32_t instr);

	//
	// SH  Store Halfword
	//
	// 31          26 25       21 20       16 15                                   0
	// +-------------+-----------+-----------+-------------------------------------+
	// |      SH     |           |           |                                     |
	// | 1 0 1 0 0 1 |    base   |    rt     |              immediate              |
	// |             |           |           |                                     |
	// +-------------+-----------+-----------+-------------------------------------+
	//         6            5          5                         16
	//
	// Format:
	//	SH rt, offset(base)
	// Description: offset(base) <- rt
	//	The 16-bit offset is sign-extended and added to the contents of general
	//	register base to form an unsigned effective address. The least-significant
	//	halfword of register rt is stored at the effective address. If the
	//	least-significant bit of the effective address is non-zero, an address
	//	error exception occurs.
	// Exceptions:
	//	Bus error exception
	//	Address error exception
	//
	void instr_sh(uint32_t instr);

	//
	// SW  Store Word
	//
	// 31          26 25       21 20       16 15                                   0
	// +-------------+-----------+-----------+-------------------------------------+
	// |      SW     |           |           |                                     |
	// | 1 0 1 0 1 1 |    base   |    rt     |              immediate              |
	// |             |           |           |                                     |
	// +-------------+-----------+-----------+-------------------------------------+
	//         6            5          5                         16
	//
	// Format:
	//	SW rt, offset(base)
	// Description: offset(base) <- rt
	//	The 16-bit offset is sign-extended and added to the contents of general
	//	register base to form an address. The contents of general register rt
	//	are stored at the memory location specified by the effective address.
	//	If either of the two least-significant bits of the effective address
	//	are non-zero, an address error exception occurs.
	// Exceptions:
	//	Bus error exception
	//	Address error exception
	//
	void instr_sw(uint32_t instr);

	//
	// Execute SPECIAL class instructions
	//
	void exec_special(uint32_t instr, bool dslot);

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
	void exec_cop0(uint32_t instr, bool dslot);

	//
	// Execute Load/Store instructions
	//
	void exec_load_store(uint32_t instr, bool dslot);

	//
	// Execute REGIMM class instructions
	//
	void exec_regimm(uint32_t instr, bool dslot);

	//
	// Execute other instructions
	//
	void exec_other(uint32_t instr, bool dslot);

	//
	// Execute instruction (main entry point for execute stage)
	//
	void execute(uint32_t instr, bool dslot);

	//
	// Main CPU thread
	//
	void cpu_thread(void);

	//
	// Interrupt delivery thread
	//
	void cpu_intr_thread(void);

	//
	// Export CPU register values
	//
	void export_register_values(uint32_t pc);

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

	bool m_ld_delay_slot;		// Load delay slot (data isn't available for next instruction)
	uint32_t m_ld_dst_reg;		// Load destination register
	uint32_t m_ld_data;		// Loaded data
};


inline uint32_t cpu_top::sign_extend8(uint32_t imm)
{
	return (imm & (1<<7)) ? imm | 0xFFFFFF00 : imm;
}


inline uint32_t cpu_top::sign_extend16(uint32_t imm)
{
	return (imm & (1<<15)) ? imm | 0xFFFF0000 : imm;
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
	wait(clk.posedge_event());
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
	wait(clk.posedge_event());
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
	wait(clk.posedge_event());
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
	wait(clk.posedge_event());
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
	wait(clk.posedge_event());
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
	wait(clk.posedge_event());
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
	wait(clk.posedge_event());
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
			m_ivtb = m_gp_regs[iw.r.rt] & (~0x3FF);
			break;
		case 11: // PSR Register
			m_psr = m_gp_regs[iw.r.rt] & (0x1);
			break;
		case 12: // SR Register
			m_sr = m_gp_regs[iw.r.rt] & (0x1);
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
	m_delay_slot = true;
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
	m_delay_slot = true;
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
	int64_t t = (int64_t)rs * (int64_t)rt;
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
	uint64_t t = (uint64_t)rs * (uint64_t)rt;
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
	m_delay_slot = true;
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
	m_delay_slot = true;
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
	m_delay_slot = true;
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
	m_delay_slot = true;
}


inline void cpu_top::instr_j(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t target = iw.j.target;
	m_next_pc = ((m_pc+4) & 0xF0000000) | target << 2;
	m_delay_slot = true;
}


inline void cpu_top::instr_jal(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t target = iw.j.target;
	m_next_pc = ((m_pc+4) & 0xF0000000) | target << 2;
	m_gp_regs[31] = m_pc + 4 + 4;
	m_delay_slot = true;
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
	m_delay_slot = true;
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
	m_delay_slot = true;
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
	m_delay_slot = true;
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
	m_delay_slot = true;
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

	m_ld_delay_slot = true;
	m_ld_dst_reg = iw.i.rt;
	m_ld_data = data;
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

	m_ld_delay_slot = true;
	m_ld_dst_reg = iw.i.rt;
	m_ld_data = data;
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

	m_ld_delay_slot = true;
	m_ld_dst_reg = iw.i.rt;
	m_ld_data = data;
}


inline void cpu_top::instr_lbu(uint32_t instr)
{
	instruction iw;
	iw.word = instr;

	uint32_t addr = rd_gpreg(iw.i.rs) + sign_extend16(iw.i.imm);
	uint32_t data = zero_extend8( load_byte(addr) );

	m_ld_delay_slot = true;
	m_ld_dst_reg = iw.i.rt;
	m_ld_data = data;
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

	m_ld_delay_slot = true;
	m_ld_dst_reg = iw.i.rt;
	m_ld_data = data;
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

	// Get load delay slot state
	bool ld_delay_slot = m_ld_delay_slot;
	uint32_t ld_dst_reg = m_ld_dst_reg;
	uint32_t ld_data = m_ld_data;
	m_ld_delay_slot = false; // clear

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
			m_except = EX_RSVD_INSTR;
			break;
	}

	// Write register if there is pending update
	if(ld_delay_slot) {
		wr_gpreg(ld_dst_reg, ld_data);
	}
}


inline void cpu_top::cpu_thread(void) {
	wait(nrst.posedge_event());

	while(true) {
		export_register_values(m_pc);

		uint32_t instr = fetch_instr(m_pc);
		if(check_except())
			continue;

		execute(instr, false);
		if(check_except())
			continue;

		export_register_values(m_pc);

		if(m_delay_slot) {
			m_delay_slot = false;
			instr = fetch_instr(m_pc+4);
			if(check_except())
				continue;
			execute(instr, true);
			if(check_except())
				continue;

			export_register_values(m_pc+4);
		}

		if(check_intr())
			continue;

		m_pc = m_next_pc;
		m_next_pc += 4;
	}
}


inline void cpu_top::cpu_intr_thread(void)
{
	while(true) {
		wait();
		if(!nrst.read())
			continue;
		m_interrupt = intr_i.read();
	}
}


inline void cpu_top::export_register_values(uint32_t pc)
{
	cpu_reg_pc.write(pc);

	cpu_reg_r0_zero.write(rd_gpreg(0));
	cpu_reg_r1_at.write(rd_gpreg(1));
	cpu_reg_r2_v0.write(rd_gpreg(2));
	cpu_reg_r3_v1.write(rd_gpreg(3));
	cpu_reg_r4_a0.write(rd_gpreg(4));
	cpu_reg_r5_a1.write(rd_gpreg(5));
	cpu_reg_r6_a2.write(rd_gpreg(6));
	cpu_reg_r7_a3.write(rd_gpreg(7));
	cpu_reg_r8_t0.write(rd_gpreg(8));
	cpu_reg_r9_t1.write(rd_gpreg(9));
	cpu_reg_r10_t2.write(rd_gpreg(10));
	cpu_reg_r11_t3.write(rd_gpreg(11));
	cpu_reg_r12_t4.write(rd_gpreg(12));
	cpu_reg_r13_t5.write(rd_gpreg(13));
	cpu_reg_r14_t6.write(rd_gpreg(14));
	cpu_reg_r15_t7.write(rd_gpreg(15));
	cpu_reg_r16_s0.write(rd_gpreg(16));
	cpu_reg_r17_s1.write(rd_gpreg(17));
	cpu_reg_r18_s2.write(rd_gpreg(18));
	cpu_reg_r19_s3.write(rd_gpreg(19));
	cpu_reg_r20_s4.write(rd_gpreg(20));
	cpu_reg_r21_s5.write(rd_gpreg(21));
	cpu_reg_r22_s6.write(rd_gpreg(22));
	cpu_reg_r23_s7.write(rd_gpreg(23));
	cpu_reg_r24_t8.write(rd_gpreg(24));
	cpu_reg_r25_t9.write(rd_gpreg(25));
	cpu_reg_r26_k0.write(rd_gpreg(26));
	cpu_reg_r27_k1.write(rd_gpreg(27));
	cpu_reg_r28_gp.write(rd_gpreg(28));
	cpu_reg_r29_sp.write(rd_gpreg(29));
	cpu_reg_r30_s8_fp.write(rd_gpreg(30));
	cpu_reg_r31_ra.write(rd_gpreg(31));

	cpu_reg_hi.write(m_hi);
	cpu_reg_lo.write(m_lo);

	cpu_reg_prid.write(m_prid);
	cpu_reg_epc.write(m_epc);
	cpu_reg_sr.write(m_sr);
	cpu_reg_psr.write(m_psr);
	cpu_reg_ivtb.write(m_ivtb);
}
