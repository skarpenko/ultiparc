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
 * Fibonacci numbers, simple workqueue test
 */

#include <arch.h>
#include <test_defines.h>
#include <compiler.h>
#include "common.h"


#define MAKE_NOISE	0
#define TMRCOUNT	5000	/* Timer counter value */
#define NR_WORKS	4	/* Number of works to allocate */
#define NR_FIB		90	/* Length of Fibonacci sequence */


/* Work data */
struct work {
	struct work *next;		/* Pointer to next work */
	void (*work_func)(void);	/* Work function */
};


/* Works */
struct work works[NR_WORKS];
struct work *free_works = NULL;		/* List of free works */
struct work *running_works = NULL;	/* List of ready to run works */


/* Fibonacci sequence */
unsigned last_fib;	/* Last computed index */
u64 fibonacci[NR_FIB];


/* Fibonacci step */
void fib_work(void)
{
	unsigned n = last_fib + 1;
	fibonacci[n] = fibonacci[n-1] + fibonacci[n-2];
	last_fib = n;
#if MAKE_NOISE == 1
	print_str("F"); print_unsigned(n); print_str(" = ");
	print_unsigned64(fibonacci[n]); print_str("\n");
#endif
}


/* Init work lists */
static inline
void init_works(void)
{
	int i;
	for(i = 0; i < NR_WORKS; ++i) {
		works[i].next = free_works;
		works[i].work_func = fib_work;
		free_works = &works[i];
	}
}


/* Get work from free list */
static inline
struct work *get_free(void)
{
	struct work *w = free_works;
	if(w)
		free_works = free_works->next;
	return w;
}


/* Put work to free list */
static inline
void put_free(struct work *w)
{
	w->next = free_works;
	free_works = w;
}


/* Get work from running list */
static inline
struct work *get_running(void)
{
	struct work *w = running_works;
	if(w)
		running_works = running_works->next;
	return w;
}


/* Put work to running list */
static inline
void put_running(struct work *w)
{
	w->next = running_works;
	running_works = w;
}


/* Interrupt entry point  */
void interrupt_entry(struct interrupt_frame *p)
{
	u32 status;

	/* Test failed if system exception occurred (7 - HW interrupt) */
	if(p->vec != 7) {
#if MAKE_NOISE == 1
		print_str("Exception #");
		print_integer(p->vec);
		print_str("!\n");
#endif
		test_failed();
	}

	/* Acknowledge interrupt */
	status = readl(INTCTL_STATUS);
	writel(status, INTCTL_STATUS);

	/* Push new works to run */
	if(last_fib < NR_FIB - 1) {
		unsigned nw;

		nw = (fibonacci[last_fib] & 0x3) + 1;	/* Number of works to run */

		while(nw) {
			struct work *w = get_free();
			if(!w)
				break;
			put_running(w);
			--nw;
		}
	}
}


/* Test start */
void user_entry()
{
	unsigned i;

#if MAKE_NOISE == 1
	print_str("Setting data structures\n");
#endif

	/* Init Fibonacci sequence */
	fibonacci[0] = 0;
	fibonacci[1] = 1;
	last_fib = 1;

	/* Setup work lists */
	init_works();


#if MAKE_NOISE == 1
	print_str("Setting timer\n");
#endif

	writel(TMRCOUNT, ITIMER_COUNT);	/* Set timer counter */
	writel(7, ITIMER_CTLREG);	/* Enable timer (+reload and interrupt) */

	/* Unmask interrupt controller line */
	writel(1, INTCTL_MASK);


#if MAKE_NOISE == 1
	print_str("Starting\n");
#endif

	/* Start! */
	interrupts_enable();

	/* Run works from queue */
	while(last_fib < NR_FIB - 1) {
		struct work *w;
		void (*work_func)(void);

		interrupts_disable();
		w = get_running();
		if(w) {
			work_func = w->work_func;
			put_free(w);
			interrupts_enable();
		} else {
			waiti_safe();	/* Wait if nothing to do */
			barrier();	/* Memory contents might have changed */
			continue;
		}

		/* Run work */
		work_func();
	}

	/* Stop! */
	interrupts_disable();


#if MAKE_NOISE == 1
	print_str("Verifying result\n");
#endif

	/* Verify Fibonacci sequence */
	for(i = 2; i < NR_FIB; ++i) {
		u64 v = fibonacci[i-1] + fibonacci[i-2];
		if(v != fibonacci[i])
			test_failed();
	}

	/* Successful termination */
	test_passed();
}
