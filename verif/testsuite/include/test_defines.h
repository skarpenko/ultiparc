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
 * Common definitions used by test programs
 */

#ifndef _CPU_TEST_DEFINES_H_
#define _CPU_TEST_DEFINES_H_


#include "arch_types.h"


/* Control device */
#define SIM_CTRL_IOBASE	0x80100000	/* Simulation control device I/O base */
#define SIM_CTRL_CTLREG	(SIM_CTRL_IOBASE + 0x00)	/* Control register */
#define TEST_PASSED	0x00000001	/* Test passed command */
#define TEST_FAILED	0x80000001	/* Test failed command */


/* Micro UART */
#define MUART_IOBASE	0x80000000	/* Micro UART I/O base */
#define MUART_CHREG	(MUART_IOBASE + 0x00)	/* Character register */


/* Interrupt controller */
#define INTCTL_IOBASE	0x80200000	/* Interrupt controller I/O base */
#define INTCTL_STATUS	(INTCTL_IOBASE + 0x00)	/* Status register */
#define INTCTL_MASK	(INTCTL_IOBASE + 0x04)	/* Mask register */
#define INTCTL_RAW	(INTCTL_IOBASE + 0x08)	/* Raw interrupts */


/* Interval timer */
#define ITIMER_IOBASE	0x80300000	/* Interval timer I/O base */
#define ITIMER_CTLREG	(ITIMER_IOBASE + 0x00)	/* Control register */
#define ITIMER_COUNT	(ITIMER_IOBASE + 0x04)	/* Counter register */
#define ITIMER_CURRENT	(ITIMER_IOBASE + 0x08)	/* Current count value */


#ifndef __ASSEMBLY__

/* Report success to simulator */
static inline void test_passed(void)
{
	*((volatile addr_t *)SIM_CTRL_CTLREG) = TEST_PASSED;
	while(1)
		;
}


/* Report failure to simulator */
static inline void test_failed(void)
{
	*((volatile addr_t *)SIM_CTRL_CTLREG) = TEST_FAILED;
	while(1)
		;
}

#endif /* __ASSEMBLY__ */


#endif /* _CPU_TEST_DEFINES_H_ */
