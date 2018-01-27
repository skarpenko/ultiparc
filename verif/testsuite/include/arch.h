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


#if !defined(__ASSEMBLY__)

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


/*
 * NOTE: According to current microarchitecture on interrupts enable first
 * interrupt may happen on third instruction following write of "1" to SR.IE and
 * two "nop" instructions might need to be inserted after write, on interrupts
 * disable instruction following write of "0" to SR.IE can be interrupted and
 * one "nop" instruction must be inserted after write to avoid races.
 */


/* Enable interrupts */
static inline void interrupts_enable(void)
{
	__asm__ __volatile__ (
		".set push        ;"
		".set noreorder   ;"
		"mfc0 $t0, $12    ;"
		"ori $t0, $t0, 1  ;"
		"mtc0 $t0, $12    ;"
		"nop              ;"
		"nop              ;"
		".set pop         ;"
		:
		:
		: "$t0"
	);
}


/* Disable interrupts */
static inline unsigned long interrupts_disable(void)
{
	u32 v;
	__asm__ __volatile__ (
		".set push         ;"
		".set noreorder    ;"
		"mfc0 %0, $12      ;"
		"li $t0, ~1        ;"
		"and $t0, %0, $t0  ;"
		"mtc0 $t0, $12     ;"
		"nop               ;"
		".set pop          ;"
		: "=r" (v)
		:
		: "$t0"
	);

	return v & 0x01;
}


/* Restore interrupts */
static inline void interrupts_restore(unsigned long state)
{
	__asm__ __volatile__ (
		".set push             ;"
		".set noreorder        ;"
		"mfc0 $t0, $12         ;"
		"li $t1, ~1            ;"
		"and $t0, $t0, $t1     ;"
		"or $t0, $t0, %0       ;"
		"mtc0 $t0, $12         ;"
		"nop                   ;"
		"nop                   ;"
		".set pop              ;"
		:
		: "r" (state)
		: "$t0", "$t1"
	);
}


/* Wait for interrupt */
static inline void waiti(void)
{
	__asm__ __volatile__ (
		".set push        ;"
		".set noreorder   ;"
		".long 0x42000020 ;"
		"nop              ;"
		"nop              ;"
		".set pop         ;"
		:
		:
		:
	);
}


/* Enable interrupts and wait for interrupt */
static inline void waiti_safe(void)
{
	__asm__ __volatile__ (
		".set push        ;"
		".set noreorder   ;"
		"mfc0 $t0, $12    ;"
		"ori $t0, $t0, 1  ;"
		"mtc0 $t0, $12    ;"
		".long 0x42000020 ;"
		"nop              ;"
		"nop              ;"
		".set pop         ;"
		:
		:
		: "$t0"
	);
}


/* Read lower half of timestamp counter */
static inline u32 rdtsc_lo(void)
{
	u32 v;
	__asm__ __volatile__ (
		".set push       ;"
		".set noreorder  ;"
		"mfc0 %0, $8     ;"
		".set pop        ;"
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
		".set push       ;"
		".set noreorder  ;"
		"mfc0 %0, $9     ;"
		".set pop        ;"
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


/* Read timestamp counter (interrupt safe) */
static inline u64 rdtsc_safe(void)
{
	unsigned long intstate = interrupts_disable();
	u32 lo = rdtsc_lo();
	u32 hi = rdtsc_hi();
	interrupts_restore(intstate);
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


#elif defined(__ASSEMBLY__)


/* WAIT instruction code */
#define WAITI	.long 0x42000020


#endif /* __ASSEMBLY__ */


#endif /* _CPU_ARCH_H_ */
