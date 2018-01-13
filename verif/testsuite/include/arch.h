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
 * CPU architecture specific defines
 */

#ifndef _CPU_ARCH_H_
#define _CPU_ARCH_H_

#include "arch_types.h"


#define CPU_REG_SIZE	4	/* CPU register size */
#define INTR_FRAME_LEN	36	/* Interrupt stack frame length */

#define CPU_ID		0x001A8100	/* Processor Id */

/* Coprocessor 0 registers */
#define TSCLO	8	/* Low half of timestamp counter */
#define TSCHI	9	/* Upper half of timestamp counter */
#define IVTB	10	/* IVT Base */
#define PSR	11	/* Prev. Status Register  */
#define SR	12	/* Status Register */
#define CAUSE	13	/* Cause Register */
#define EPC	14	/* Exception PC */
#define PRID	15	/* Processor Id */


#ifndef __ASSEMBLY__

/* Interrupt stack frame */
struct interrupt_frame {
	u32 vec;	/* Vector number */
	u32 psr;	/* Previous status */
	u32 sr;		/* Status register */
	u32 cause;	/* Cause register */
	u32 epc;	/* EPC */
	u32 lo;		/* LO special register */
	u32 hi;		/* HI special register */
	/* General purpose registers follow */
	u32 ra;
	u32 fp;
	u32 sp;
	u32 gp;
	u32 t9;
	u32 t8;
	u32 s7;
	u32 s6;
	u32 s5;
	u32 s4;
	u32 s3;
	u32 s2;
	u32 s1;
	u32 s0;
	u32 t7;
	u32 t6;
	u32 t5;
	u32 t4;
	u32 t3;
	u32 t2;
	u32 t1;
	u32 t0;
	u32 a3;
	u32 a2;
	u32 a1;
	u32 a0;
	u32 v1;
	u32 v0;
	u32 at;
};


/* Task state */
struct arch_task {
	u32 sp;
};


/* Read word */
static inline u32 readl(addr_t addr)
{
	return *((volatile addr_t*)addr);
}


/* Write word */
static inline void writel(u32 v, addr_t addr)
{
	*((volatile addr_t*)addr) = v;
}


/* Enable interrupts */
static inline void interrupts_enable(void)
{
	__asm__ __volatile__ (
		"li $t0, 1        ;"
		"mfc0 $t1, $12    ;"
		"or $t1, $t1, $t0 ;"
		"mtc0 $t1, $12    ;"
		:
		:
		: "$t0", "$t1"
	);
}


/* Disable interrupts */
static inline unsigned long interrupts_disable(void)
{
	u32 v;
	__asm__ __volatile__ (
		"li $t0, ~1        ;"
		"mfc0 $t1, $12     ;"
		"and $t0, $t1, $t0 ;"
		"mtc0 $t1, $12     ;"
		"move %0, $t1      ;"
		: "=r" (v)
		:
		: "$t0", "$t1"
	);

	return v & 0x01;
}


/* Restore interrupts */
static inline void interrupts_restore(unsigned long state)
{
	__asm__ __volatile__ (
		"mfc0 $t0, $12   ;"
		"or $t0, $t0, %0 ;"
		"mtc0 $t1, $12   ;"
		:
		: "r" (state)
		: "$t0"
	);
}


/* Read lower half of timestamp counter */
static inline u32 rdtsc_lo(void)
{
	u32 v;
	__asm__ __volatile__ (
		"mfc0 %0, $8     ;"
		: "=r" (v)
		:
		:
	);

	return v;
}


/* Read upper half of timestamp counter */
static inline u32 rdtsc_hi(void)
{
	u32 v;
	__asm__ __volatile__ (
		"mfc0 %0, $9     ;"
		: "=r" (v)
		:
		:
	);

	return v;
}


/* Read timestamp counter */
static inline u64 rdtsc(void)
{
	u32 lo = rdtsc_lo();	/* Read lower half first to latch upper half */
	u32 hi = rdtsc_hi();
	return ((u64)hi << 32) | lo;
}


/* Switch task context macro */
void arch_switch_task_context(struct arch_task *from, struct arch_task *to);
#define switch_task_context(from, to)				\
	do {							\
		arch_switch_task_context(from, to);		\
		__asm__ __volatile__("" : : : "memory");	\
	} while(0)


/* Initialize task stack */
static inline u32* init_task_stack(u32 *stack, u32 entry)
{
	extern u32 _gp;
	int i;
	struct interrupt_frame *iframe;
	u32 top = (u32)stack;
	for(i = 0; i < INTR_FRAME_LEN; ++i) *--stack = 0;
	iframe = (struct interrupt_frame *)stack;
	iframe->sr = 0x01;	/* Interrupts enable */
	iframe->ra = entry;
	iframe->sp = top;
	iframe->gp = (u32)&_gp;
	return stack;
}


#endif /* __ASSEMBLY__ */


#endif /* _CPU_ARCH_H_ */
