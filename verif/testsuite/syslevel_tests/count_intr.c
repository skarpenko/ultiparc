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
 * Counting interrupts test
 */

#include <arch.h>
#include <test_defines.h>


#define NEXPECT	20	/* Number of interrupts to expect */
#define COUNT	2000	/* Timer count value */


volatile unsigned intr_count = 0;	/* Interrupts count */


/* Interrupt handler */
void interrupt_entry(struct interrupt_frame *p)
{
	u32 status;

	/* Test failed if system exception occurred (7 - HW interrupt) */
	if(p->vec != 7)
		test_failed();

	++intr_count;	/* count timer interrupt */

	/* Acknowledge interrupt */
	status = readl(INTCTL_STATUS);
	writel(status, INTCTL_STATUS);
}


/* Test start */
void user_entry()
{
	unsigned count = 0;

	writel(COUNT, ITIMER_COUNT);	/* Set timer counter */
	writel(7, ITIMER_CTLREG);	/* Enable timer (+reload and interrupt) */

	/* Unmask interrupt controller line */
	writel(1, INTCTL_MASK);

	interrupts_enable();

	/* Wait for expected timer interrupts count */
	while(intr_count < NEXPECT) {
		waiti();
		++count;
	}

	/* Disable interrupts and recheck the result */
	interrupts_disable();
	if(intr_count != NEXPECT || intr_count != count)
		test_failed();

	test_passed();
}
