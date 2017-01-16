/*
 * Copyright (c) 2017 The Ultiparc Project. All rights reserved.
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
 * Simple preemptive multitasking test
 */

#include <arch.h>
#include <test_defines.h>
#include <math.h>
#include "common.h"
#include "fxp24.h"


#define MAKE_NOISE	0
#define TMRCOUNT	5000	/* Timer counter value */
#define DEFINE_STACK(name, size)	\
	u8 name[size] __attribute__((aligned(4)));
#define STACK_SIZE	4096	/* Default stack size */


/* Task data */
struct task {
	struct arch_task arch;	/* Architecture specific data */
	unsigned time2run;	/* Time to run (priority) */
	struct task *next;	/* Pointer to next task */
};


struct task *current = NULL;		/* Current task */
struct task *task_list_head = NULL;	/* Run queue head */
struct task *task_list_tail = NULL;	/* Run queue tail */


/* Pop task from run queue head */
static inline
struct task *task_pop_head(void)
{
	struct task *p = task_list_head;
	if(p) {
		task_list_head = p->next;
		task_list_tail =
			task_list_head ? task_list_tail : NULL;
	}
	return p;
}


/* Put task to the end of run queue */
static inline
void task_push_tail(struct task *t)
{
	if(task_list_tail) {
		task_list_tail = task_list_tail->next = t;
	} else {
		task_list_head = task_list_tail = t;
	}
	t->next = NULL;
}


/* Interrupt entry point  */
void interrupt_entry(struct interrupt_frame *p)
{
	static unsigned task_time = 0;
	u32 status;
	int reschedule = 0;

	/* Test failed if system exception occurred (7 - HW interrupt) */
	if(p->vec != 7) {
#if MAKE_NOISE == 1
		print_str("Exception #");
		print_integer(p->vec);
		print_str("!\n");
#endif
		test_failed();
	}

	/* Count down task run time */
	if(task_time)
		--task_time;
	else
		reschedule = 1;

	/* Acknowledge interrupt */
	status = readl(INTCTL_STATUS);
	writel(status, INTCTL_STATUS);

	/* Reschedule */
	if(reschedule) {
		struct task *next = task_pop_head();
		struct task *curr = current;
		if(next) {
			task_push_tail(current);
			current = next;
			task_time = current->time2run;
			switch_task_context(&curr->arch, &next->arch);
		}
	}
}


/* Create new task and add it to run queue */
void create_new_task(struct task *t, unsigned ttr, u8 *stack, unsigned stack_sz,
	void (*entry)(void))
{
	u32* stack_top = (u32*)&stack[stack_sz];
	t->arch.sp = (u32)init_task_stack(stack_top, (u32)entry);
	t->time2run = ttr;
	t->next = NULL;
	task_push_tail(t);
}


#define NTASKS	3

/* Floating-point FFT task */
DEFINE_STACK(flp_fft_task_stack, STACK_SIZE);
struct task flp_fft_task;
void flp_fft_task_func(void);
volatile int flp_fft_task_compl = 0;

/* Fixed-point FFT task */
DEFINE_STACK(fxp_fft_task_stack, STACK_SIZE);
struct task fxp_fft_task;
void fxp_fft_task_func(void);
volatile int fxp_fft_task_compl = 0;

/* Bubble sort task */
DEFINE_STACK(bubble_sort_task_stack, STACK_SIZE);
struct task bubble_sort_task;
void bubble_sort_task_func(void);
volatile int bubble_sort_task_compl = 0;


/* Test start */
void user_entry()
{
	unsigned nf;

	/* Set current task */
	struct task main = {
		.time2run = 3,
		.next = NULL
	};
	current = &main;

#if MAKE_NOISE == 1
	print_str("Setting timer\n");
#endif

	writel(TMRCOUNT, ITIMER_COUNT);	/* Set timer counter */
	writel(7, ITIMER_CTLREG);	/* Enable timer (+reload and interrupt) */

	/* Unmask interrupt controller line */
	writel(1, INTCTL_MASK);

#if MAKE_NOISE == 1
	print_str("Creating tasks\n");
#endif
	create_new_task(&flp_fft_task, 20, flp_fft_task_stack, sizeof(flp_fft_task_stack),
		flp_fft_task_func);
	create_new_task(&fxp_fft_task, 15, fxp_fft_task_stack, sizeof(fxp_fft_task_stack),
		fxp_fft_task_func);
	create_new_task(&bubble_sort_task, 10, bubble_sort_task_stack, sizeof(bubble_sort_task_stack),
		bubble_sort_task_func);

#if MAKE_NOISE == 1
	print_str("Starting\n");
#endif

	/* Begin the magic */
	interrupts_enable();

	/* Check tasks completion */
	while(1) {
		nf = 0;
		nf += flp_fft_task_compl;
		nf += fxp_fft_task_compl;
		nf += bubble_sort_task_compl;
		if(nf == NTASKS)
			break;
	}

	/* Successful termination */
	test_passed();
}


/****************** Floating-point FFT task ***********************************/

#define ROUND_ERROR	1E-10

/* Floating-point fast Fourier transform */
static void fft(double *re, double *im, unsigned long n, int isign)
{
	unsigned long i, j, k, l, le, le1, ip, n2;
	double wpr, wpi, wr, wi, wtr, wti;

	n2 = n>>1;
	j = 1;
	for(i=0; i<n-1; i++) {
		if(i<j) {
			wtr     = re[j-1];
			wti     = im[j-1];
			re[j-1] = re[i];
			im[j-1] = im[i];
			re[i]   = wtr;
			im[i]   = wti;
		}
		k = n2;
		while(k<j) {
			j -= k;
			k >>= 1;
		}
		j += k;
	}
	l=1;
	k=n;
	while(k>>=1) {
		le1 = (le=1<<l++) >> 1;
		wtr = M_PI / (double)le1;
		wpr = cos(wtr); wpi = -isign*sin(wtr);
		wr = 1.0;       wi = 0.0;
		for(j=0; j<le1; j++) {
			for(i=j; i<n; i+=le) {
				ip = i + le1;
				wtr    = wr*re[ip] - wi*im[ip];
				wti    = wi*re[ip] + wr*im[ip];
				re[ip] = re[i] - wtr;
				im[ip] = im[i] - wti;
				re[i]  = re[i] + wtr;
				im[i]  = im[i] + wti;
			}
			wr = (wtr=wr)*wpr - wi*wpi;
			wi = wi*wpr + wtr*wpi;
		}
	}
}


/* Source data */
#define FLP_FFT_SIZE	64
double flp_f_r[FLP_FFT_SIZE], flp_f_i[FLP_FFT_SIZE];
double flp_g_r[FLP_FFT_SIZE], flp_g_i[FLP_FFT_SIZE];


/* FLP FFT task entry */
void flp_fft_task_func(void)
{
	unsigned i;

#if MAKE_NOISE == 1
	print_str("Floating-point FFT task started.\n");
#endif

	/**** Prepare data for test ****/

#if MAKE_NOISE == 1
	print_str("FLP FFT: Generating test data\n");
#endif

	for(i=0; i<FLP_FFT_SIZE; ++i) {
		flp_f_r[i] = flp_g_r[i] = 3.0 * cos(2.0 * M_PI * 0.08 * i) +
			2.0 * cos(2.0 * M_PI * 0.05 * i);
		flp_f_i[i] = flp_g_i[i] = 3.0 * sin(2.0 * M_PI * 0.08 * i) +
			2.0 * sin(2.0 * M_PI * 0.05 * i);
	}

	/**** Forward transform ****/

#if MAKE_NOISE == 1
	print_str("FLP FFT: FFT\n");
#endif

	fft(flp_f_r, flp_f_i, FLP_FFT_SIZE, 1);

	/**** Inverse transform ****/

#if MAKE_NOISE == 1
	print_str("FLP FFT: IFFT\n");
#endif

	fft(flp_f_r, flp_f_i, FLP_FFT_SIZE, -1);

	/**** Normalization ****/

#if MAKE_NOISE == 1
	print_str("FLP FFT: Normalization\n");
#endif

	for(i=0; i<FLP_FFT_SIZE; ++i) {
		flp_f_r[i] /= (double)FLP_FFT_SIZE;
		flp_f_i[i] /= (double)FLP_FFT_SIZE;
	}

	/**** Result verification ****/

#if MAKE_NOISE == 1
	print_str("FLP FFT: Verifying result\n");
#endif

	for(i=0; i<FLP_FFT_SIZE; ++i) {
		if(fabs(flp_f_r[i] - flp_g_r[i]) > ROUND_ERROR)
			test_failed();
		if(fabs(flp_f_i[i] - flp_g_i[i]) > ROUND_ERROR)
			test_failed();
	}

#if MAKE_NOISE == 1
	print_str("FLP FFT: Done\n");
#endif

	flp_fft_task_compl = 1;	/* Mark as completed */

	/* No termination support. Spin forever. */
	while(1)
		;
}


/******************** Fixed-point FFT task ************************************/

/* Fixed-point fast Fourier transform */
static void fxp_fft(fixed_t *re, fixed_t *im, unsigned n, int isign)
{
	/* Sine values table */
	static const fixed_t sin_tbl[] = {
		0, 16777216, 11863283, 6420362, 3273072, 1644454, 823218,
		411733, 205882, 102943, 51471, 25735, 12867, 6433, 3216, 1608
	};
	/* Cosine values table */
	static const fixed_t cos_tbl[] = {
		-16777216, 0, 11863283, 15500126, 16454846, 16696429, 16757007,
		16772163, 16775952, 16776900, 16777137, 16777196, 16777211,
		16777214, 16777215, 16777215
	};
	unsigned i, j, k, l, le, le1, ip, n2;
	fixed_t wpr, wpi, wr, wi, wtr, wti;

	/* Maximum data length */
	if(n>65536) n = 65536;

	n2 = n>>1;
	j = 1;
	for(i=0; i<n-1; i++) {
		if(i<j) {
			wtr     = re[j-1];
			wti     = im[j-1];
			re[j-1] = re[i];
			im[j-1] = im[i];
			re[i]   = wtr;
			im[i]   = wti;
		}
		k = n2;
		while(k<j) {
			j -= k;
			k >>= 1;
		}
		j += k;
	}
	l=1;
	k=n;
	while(k>>=1) {
		le1 = (le=1<<l++) >> 1;
		wpr = cos_tbl[l-2]; wpi = (isign < 0 ? sin_tbl[l-2] : FXP_NEG(sin_tbl[l-2]));
		wr = FXP_ONE;       wi = FXP_ZERO;
		for(j=0; j<le1; j++) {
			for(i=j; i<n; i+=le) {
				ip = i + le1;
				wtr    = FXP_SUB( FXP_MUL(wr, re[ip]), FXP_MUL(wi, im[ip]) );
				wti    = FXP_ADD( FXP_MUL(wi, re[ip]), FXP_MUL(wr, im[ip]) );
				re[ip] = FXP_SUB( re[i], wtr );
				im[ip] = FXP_SUB( im[i], wti );
				re[i]  = FXP_ADD( re[i], wtr );
				im[i]  = FXP_ADD( im[i], wti );
			}
			wtr = wr;
			wr = FXP_SUB( FXP_MUL(wr, wpr), FXP_MUL(wi, wpi) );
			wi = FXP_ADD( FXP_MUL(wi, wpr), FXP_MUL(wtr, wpi) );
		}
	}
}


/* Source data */
#define FFT_SIZE	256
#include "fxp_fft_data.h"


/* FXP FFT task entry */
void fxp_fft_task_func(void)
{
	unsigned i;

#if MAKE_NOISE == 1
	print_str("Fixed-point FFT task started.\n");
#endif

#if MAKE_NOISE == 1
	print_str("FXP FFT: FFT\n");
#endif

	/* Forward transform */
	fxp_fft(f_r, f_i, N, 1);

#if MAKE_NOISE == 1
	print_str("FXP FFT: Verifying result\n");
#endif

	for(i=0; i<N; ++i)
		if(f_r[i] != F_r[i] || f_i[i] != F_i[i])
			test_failed();

#if MAKE_NOISE == 1
	print_str("FXP FFT: IFFT\n");
#endif

	/* Inverse transform */

	fxp_fft(f_r, f_i, N, -1);

#if MAKE_NOISE == 1
	print_str("FXP FFT: Verifying result\n");
#endif

	/*
	 * We don't do FFT/IFFT scaling as a result amplitude of
	 * reconstructed signal differs from source signal.
	 */
	for(i=0; i<N; ++i)
		if(f_r[i] != f2_r[i] || f_i[i] != f2_i[i])
			test_failed();

#if MAKE_NOISE == 1
	print_str("FXP FFT: Done\n");
#endif

	fxp_fft_task_compl = 1;	/* Mark as completed */

	/* No task termination support. Just spin forever */
	while(1)
		;
}


/**************************** Bubble sort task ********************************/

#define SORT_DATA_LEN	256
int b_sort[SORT_DATA_LEN];


/* Bubble sort task entry */
void bubble_sort_task_func(void)
{
	int a = 16807;
	int m = 2147483647;
	int seed = SORT_DATA_LEN >> 1;
	int i, j, tmp;

#if MAKE_NOISE == 1
	print_str("Bubble sort task started.\n");
#endif

#if MAKE_NOISE == 1
	print_str("Bubble: Generating test data\n");
#endif
	for(i = 0; i < SORT_DATA_LEN; ++i) {
		seed = (a * seed) % m;
		b_sort[i] = seed % (SORT_DATA_LEN << 12);
	}

#if MAKE_NOISE == 1
	print_str("Bubble: Sorting\n");
#endif

	for(j = 0 ; j < SORT_DATA_LEN - 1; ++j) {
		for(i = 0; i < SORT_DATA_LEN - j - 1; ++i) {
			if (b_sort[i] > b_sort[i + 1]) {
				tmp = b_sort[i];
				b_sort[i] = b_sort[i + 1];
				b_sort[i + 1] = tmp;
			}
		}
	}

#if MAKE_NOISE == 1
	print_str("Bubble: Verifying result\n");
#endif

	for(i = 1; i < SORT_DATA_LEN; ++i)
		if(b_sort[i-1] > b_sort[i])
			test_failed();

#if MAKE_NOISE == 1
	print_str("Bubble: Done\n");
#endif

	bubble_sort_task_compl = 1;	/* Mark as completed */

	/* Spin forever */
	while(1)
		;
}
