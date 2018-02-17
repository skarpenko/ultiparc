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
 * Wavelet transform. Multitasking test.
 */

#include <stddef.h>
#include <arch.h>
#include <test_defines.h>
#include <compiler.h>
#include <math.h>
#include "common.h"
#include "fxp24.h"
#include "wavelet_filt.h"
#include "wavelet_data.h"


#define MAKE_NOISE	0	/* Produce debug output */
#define INTERM_OUTPUT	0	/* Produce output of processed data */
#define SMALL_DATASET	1	/* Use smaller dataset */
#define TMRCOUNT	5000	/* Timer counter value */
#define DEFINE_STACK(name, size)	\
	u8 name[size] __attribute__((aligned(4)));
#define STACK_SIZE	4096	/* Default stack size */


/* Rounding error tolerance */
#define ROUND_ERROR_F	1E-7	/* For floating-point computations */
#define ROUND_ERROR_B	1	/* For floating-point to byte conversions */


/* Data dimensions */
#define DLENGTH		(WAVELET_DATA_LENGTH)
#define DWIDTH		(WAVELET_DATA_WIDTH)
#if SMALL_DATASET == 0
# define DHEIGHT	(WAVELET_DATA_HEIGHT)
#else
# define DHEIGHT	(1)
#endif


/* Debug macros */
#if MAKE_NOISE == 1
# define PRINTS(a)	print_str(a)
# define PRINTI(a)	print_integer(a)
# define PRINTU(a)	print_unsigned(a)
# define PRINTU64(a)	print_unsigned64(a)
# define INTEN()	interrupts_enable()
# define INTDIS()	interrupts_disable()
#else
# define PRINTS(a)
# define PRINTI(a)
# define PRINTU(a)
# define PRINTU64(a)
# define INTEN()
# define INTDIS()
#endif


/* Queue node element */
struct queue_node {
	struct queue_node *next;
	struct queue_node *prev;
};


/* Queue */
struct queue {
	struct queue_node *head;
	struct queue_node *tail;
};
#define DEFINE_QUEUE(_name)	\
	struct queue _name = { NULL, NULL };


/* Task data */
struct task {
	struct queue_node qn;	/* Queue node */
	struct arch_task arch;	/* Architecture specific data */
	unsigned time2run;	/* Time to run (priority) */
};


/* Work data */
struct work {
	struct queue_node qn;			/* Queue node */
	const char *name;			/* Work name */
	void (*func)(const struct work*);	/* Work function */
	const void *uptr1;			/* User data 1 */
	const void *uptr2;			/* User data 2 */
};
#define DEFINE_WORK(_name, _func, _uptr1, _uptr2)			\
	struct work _name = { .qn = { NULL, NULL }, .name = #_name,	\
				.func = _func, .uptr1 = _uptr1,		\
				.uptr2 = _uptr2 }


struct task *current = NULL;			/* Current task */
volatile unsigned current_task_time = 0;	/* Current task time */

DEFINE_QUEUE(runqueue);		/* Run queue for preemptive scheduler */
DEFINE_QUEUE(filtwqueue);	/* Wavelet filtering work queue */
DEFINE_QUEUE(reconwqueue);	/* Wavelet function reconstruction work queue */


/*** Tasks ***/

/* Wavelet filtering task */
DEFINE_STACK(filtering_task_stack, STACK_SIZE);
struct task filtering_task;
void filtering_task_func(void);

/* Wavelet function reconstruction */
DEFINE_STACK(reconstr_task_stack, STACK_SIZE);
struct task reconstr_task;
void reconstr_task_func(void);


/*** Works ***/

#define NWORKS	8

volatile int ncompl_works = 0;	/* Number of completed works */

void flp_filter_work_func(const struct work *w);
void fxp_filter_work_func(const struct work *w);
void flp_reconstr_work_func(const struct work *w);
void fxp_reconstr_work_func(const struct work *w);

/* Define works */
DEFINE_WORK(flp_filter_data1, flp_filter_work_func, &data1[0][0],
	&flp_filt_data1[0][0]);
DEFINE_WORK(flp_filter_data2, flp_filter_work_func, &data2[0][0],
	&flp_filt_data2[0][0]);
DEFINE_WORK(fxp_filter_data1, fxp_filter_work_func, &data1[0][0],
	&fxp_filt_data1[0][0]);
DEFINE_WORK(fxp_filter_data2, fxp_filter_work_func, &data2[0][0],
	&fxp_filt_data2[0][0]);
DEFINE_WORK(flp_reconstr_db2, flp_reconstr_work_func, &flp_db2_func[0],
	NULL);
DEFINE_WORK(flp_reconstr_rbio22, flp_reconstr_work_func, &flp_rbio22_func[0],
	NULL);
DEFINE_WORK(fxp_reconstr_db2, fxp_reconstr_work_func, &fxp_db2_func[0],
	NULL);
DEFINE_WORK(fxp_reconstr_rbio22, fxp_reconstr_work_func, &fxp_rbio22_func[0],
	NULL);


/* Add an element to queue */
static inline
void queue_push_tail(struct queue *q, struct queue_node *n)
{
	if(q->tail) {
		q->tail = q->tail->next = n;
	} else {
		q->head = q->tail = n;
	}
	n->next = NULL;
}


/* Get an element from queue */
static inline
struct queue_node *queue_pop_head(struct queue *q)
{
	struct queue_node *p = q->head;
	if(p) {
		q->head = p->next;
		q->tail =
			q->head ? q->tail : NULL;
	}
	return p;
}


/* Get task from run queue head */
static inline
struct task *task_pop_head(void)
{
	return (struct task *)queue_pop_head(&runqueue);
}


/* Put task to the end of run queue */
static inline
void task_push_tail(struct task *t)
{
	queue_push_tail(&runqueue, &t->qn);
}


/* Create new task and add it to run queue */
void create_new_task(struct task *t, unsigned ttr, u8 *stack, unsigned stack_sz,
	void (*entry)(void))
{
	u32* stack_top = (u32*)&stack[stack_sz];
	t->arch.sp = (u32)init_task_stack(stack_top, (u32)entry);
	t->time2run = ttr;
	task_push_tail(t);
}


/******************* Timer interrupt handler and scheduler ********************/

/* Interrupt entry point */
void interrupt_entry(struct interrupt_frame *p)
{
	u32 status;
	int reschedule = 0;

	/* Test failed if system exception occurred (7 - HW interrupt) */
	if(p->vec != 7) {
		PRINTS("Exception #");
		PRINTI(p->vec);
		PRINTS("!\n");
		test_failed();
	}

	/* Count down task run time */
	if(current_task_time)
		--current_task_time;
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
			current_task_time = current->time2run;
			switch_task_context(&curr->arch, &next->arch);
		}
	}
}


/************************ Main entry point ************************************/

/* Test start */
void user_entry()
{
	/* Set current task */
	struct task main = {
		.qn = { NULL, NULL},
		.time2run = 10
	};
	unsigned ctswitch_counter = 0;	/* Context switches counter */
	/* Works schedule */
	DEFINE_QUEUE(filter_sched);
	DEFINE_QUEUE(reconst_sched);

	current = &main;	/* Set current task */

	PRINTS("Setting timer\n");

	writel(TMRCOUNT, ITIMER_COUNT);	/* Set timer counter */
	writel(7, ITIMER_CTLREG);	/* Enable timer (+reload and interrupt) */

	/* Unmask interrupt controller line */
	writel(1, INTCTL_MASK);

	/* Create tasks for preemptive scheduling */
	PRINTS("Creating tasks\n");
	create_new_task(&filtering_task, 25, filtering_task_stack,
		sizeof(filtering_task_stack), filtering_task_func);
	create_new_task(&reconstr_task, 15, reconstr_task_stack,
		sizeof(reconstr_task_stack), reconstr_task_func);

	/* Prepare filter works schedule */
	queue_push_tail(&filter_sched, &flp_filter_data1.qn);
	queue_push_tail(&filter_sched, &fxp_filter_data1.qn);
	queue_push_tail(&filter_sched, &flp_filter_data2.qn);
	queue_push_tail(&filter_sched, &fxp_filter_data2.qn);
	/* Prepare reconstruction works schedule */
	queue_push_tail(&reconst_sched, &flp_reconstr_db2.qn);
	queue_push_tail(&reconst_sched, &fxp_reconstr_db2.qn);
	queue_push_tail(&reconst_sched, &flp_reconstr_rbio22.qn);
	queue_push_tail(&reconst_sched, &fxp_reconstr_rbio22.qn);

	PRINTS("Starting\n");

	/* Enable interrupts to start preemptive scheduling */
	interrupts_enable();

	/* Check tasks completion */
	while(ncompl_works != NWORKS) {
		/* Update local counter of context switches */
		if(current_task_time == main.time2run)
			++ctswitch_counter;

		/* Schedule new computation works */
		interrupts_disable();
		if((ctswitch_counter % 2) == 0) {
			struct queue_node *qn = queue_pop_head(&filter_sched);
			if(qn)
				queue_push_tail(&filtwqueue, qn);
		} else if((ctswitch_counter % 3) == 0) {
			struct queue_node *qn = queue_pop_head(&reconst_sched);
			if(qn)
				queue_push_tail(&reconwqueue, qn);
		}
		waiti_safe();	/* Wait for interrupt */
		barrier();	/* Compiler optimization barrier */
	}

	PRINTS("All done!\n");

	/* Successful termination */
	test_passed();
}


/********************** Wavelet transform routines ****************************/

void flp_dwt_fwd(const float *restrict in, float *restrict out, size_t n,
	const float *restrict h, const float *restrict g, size_t ncoef)
{
	size_t i, ii;
	size_t jf;
	size_t k;
	size_t n1, ni, nh, nmod;

	nmod = ncoef * n;
	nmod -= ncoef >> 1;	/* center support of wavelet */

	n1 = n - 1;
	nh = n >> 1;

	for(i = 0; i < n; ++i) out[i] = 0.0;

	for (ii = 0, i = 0; i < n; i += 2, ii++) {
		ni = i + nmod;
		for (k = 0; k < ncoef; k++) {
			jf = n1 & (ni + k);
			out[ii]		+= h[k] * in[jf];
			out[ii + nh]	+= g[k] * in[jf];
		}
	}
}


void flp_dwt_inv(const float *restrict in, float *restrict out, size_t n,
	const float *restrict h, const float *restrict g, size_t ncoef)
{
	size_t i, ii;
	size_t jf;
	size_t k;
	size_t n1, ni, nh, nmod;
	float ai, ai1;

	nmod = ncoef * n;
	nmod -= ncoef >> 1;	/* center support of wavelet */

	n1 = n - 1;
	nh = n >> 1;

	for(i = 0; i < n; ++i) out[i] = 0.0;

	for (ii = 0, i = 0; i < n; i += 2, ii++) {
		ai = in[ii];
		ai1 = in[ii + nh];
		ni = i + nmod;
		for (k = 0; k < ncoef; k++) {
			jf = (n1 & (ni + k));
			out[jf] += h[k] * ai + g[k] * ai1;
		}
	}
}


void fxp_dwt_fwd(const fixed_t *restrict in, fixed_t *restrict out, size_t n,
	const fixed_t *restrict h, const fixed_t *restrict g, size_t ncoef)
{
	size_t i, ii;
	size_t jf;
	size_t k;
	size_t n1, ni, nh, nmod;

	nmod = ncoef * n;
	nmod -= ncoef >> 1;	/* center support of wavelet */

	n1 = n - 1;
	nh = n >> 1;

	for(i = 0; i < n; ++i) out[i] = 0;

	for (ii = 0, i = 0; i < n; i += 2, ii++) {
		ni = i + nmod;
		for (k = 0; k < ncoef; k++) {
			jf = n1 & (ni + k);
			out[ii]		= FXP_ADD(out[ii], FXP_MUL(h[k], in[jf]));
			out[ii + nh]	= FXP_ADD(out[ii + nh], FXP_MUL(g[k], in[jf]));
		}
	}
}


void fxp_dwt_inv(const fixed_t *restrict in, fixed_t *restrict out, size_t n,
	const fixed_t *restrict h, const fixed_t *restrict g, size_t ncoef)
{
	size_t i, ii;
	size_t jf;
	size_t k;
	size_t n1, ni, nh, nmod;
	fixed_t ai, ai1;

	nmod = ncoef * n;
	nmod -= ncoef >> 1;	/* center support of wavelet */

	n1 = n - 1;
	nh = n >> 1;

	for(i = 0; i < n; ++i) out[i] = 0;

	for (ii = 0, i = 0; i < n; i += 2, ii++) {
		ai = in[ii];
		ai1 = in[ii + nh];
		ni = i + nmod;
		for (k = 0; k < ncoef; k++) {
			jf = (n1 & (ni + k));
			out[jf] = FXP_ADD(out[jf], FXP_ADD(FXP_MUL(h[k], ai),
				FXP_MUL(g[k], ai1)));
		}
	}
}


/************************** Utility routines **********************************/

/* Print data as Portable Graymap format (PGM) image */
void print_picture(const char *name, unsigned width, unsigned height, const void *data)
{
	size_t i;
	u8 *d = (u8*)data;

	print_str("Picture "); print_str(name); print_str(":\n");

	print_str("P2\n");

	print_unsigned(width); print_str(" "); print_unsigned(height); print_str("\n");

	print_str("255");
	for(i = 0; i < width * height; ++i) {
		if(!(i % 17))	/* Line should not be more than 70 chars */
			print_str("\n");
		if(d[i] < 100) print_str(" ");
		if(d[i] < 10) print_str(" ");
		print_str(" "); print_unsigned(d[i]);
	}
	print_str("\n");
}


/* Print fixed-point wavelet function values */
void print_fxp_wavelet(const char *name, const fixed_t *wavelet, size_t n)
{
	size_t i;

	print_str("Wavelet function "); print_str(name); print_str(":\n");

	for(i = 0; i < n; ++i) {
		print_integer(wavelet[i]);
		print_str("\n");
	}
}


/* Print floating-point wavelet function values */
void print_flp_wavelet(const char *name, const float *wavelet, size_t n)
{
	size_t i;

	print_str("Wavelet function "); print_str(name); print_str(":\n");

	for(i = 0; i < n; ++i) {
		/* Just multiply to some big number and print as an integer value */
		print_integer((int)(wavelet[i] * 1E5));
		print_str("\n");
	}
}


/* Print filtering task progress */
void print_filt_progress(const char *task_name, int data_row, const char *msg)
{
#if MAKE_NOISE == 1
	unsigned long intstat = interrupts_disable();
	print_str(task_name);
	if(data_row >= 0) {
		print_str(": row = ");
		print_integer(data_row);
	}
	print_str(": ");
	print_str(msg);
	print_str("\n");
	interrupts_restore(intstat);
#endif
}


/* Get passed cycles count */
u64 passed_cycles(u64 start, u64 end)
{
	/* Deal with wraparound */
	return end < start ? -start + end : end - start;
}


/******************* Floating-point wavelet filtering *************************/

/* Compute threshold for filtering */
float flp_threshold(float *data, size_t n)
{
	float thr = 0.0;
	size_t i;
	for(i = 0; i < n; ++i) {
		float v = (data[i] >= 0.0 ? data[i] : -data[i]);
		if(v > thr)
			thr = v;
	}
	return thr / 2.0;
}


/* Byte to floating-point */
static inline
float byte2float(u8 b)
{
	return (float)b / 255.0;
}


/* Floating-point to byte */
static inline
u8 float2byte(float v)
{
	v *= 255.0;
	return (u8)(v < 0.0 ? 0.0 : (v > 255.0 ? 255.0 : v));
}


/* Returns absolute value of byte difference */
static inline
unsigned byte_diff(u8 a, u8 b)
{
	int v = (int)a - (int)b;
	return (v >= 0 ? v : -v);
}


/* Filter data using floating-point wavelet filters */
void flp_filter_data(const char *name, const u8 *src_data, const u8 *ref_data)
{
	size_t l;
	size_t nn;
	static float dat[DWIDTH];
	static float tmp[DWIDTH];
	static u8 filtered[DHEIGHT][DWIDTH];

	for(l = 0; l < DHEIGHT; ++l) {
		float thr;

		print_filt_progress(name, l, "converting bytes to floating-point");

		/* Convert data to floating-point values */
		for(nn = 0; nn < DWIDTH; ++nn)
			dat[nn] = byte2float(src_data[l*DWIDTH + nn]);

		print_filt_progress(name, l, "forward wavelet transform");

		/* Apply forward wavelet transform using Coiflet5 wavelet */
		for(nn = DWIDTH; nn >= 4; nn >>= 1) {
			flp_dwt_fwd(dat, tmp, nn, flp_coif5_h, flp_coif5_g, WLTLEN(flp_coif5_h));
			memcpy(dat, tmp, sizeof(float) * nn);
		}

		print_filt_progress(name, l, "thresholding level 1 coefficients");

		/* Threshold wavelet coefficients of level 1 */
		thr = flp_threshold(&dat[DWIDTH>>1], DWIDTH>>1);
		for(nn = DWIDTH>>1; nn < DWIDTH; ++nn) {
			float v = (dat[nn] >= 0.0 ? dat[nn] : -dat[nn]);
			dat[nn] = (v >= thr ? dat[nn] : 0.0);
		}

		print_filt_progress(name, l, "thresholding level 2 coefficients");

		/* Threshold wavelet coefficients of level 2 */
		thr = flp_threshold(&dat[DWIDTH>>2], DWIDTH>>2);
		for(nn = DWIDTH>>2; nn < DWIDTH>>1; ++nn) {
			float v = (dat[nn] >= 0.0 ? dat[nn] : -dat[nn]);
			dat[nn] = (v >= thr ? dat[nn] : 0.0);
		}

		print_filt_progress(name, l, "inverse wavelet transform");

		/* Apply inverse wavelet transform */
		for(nn = 4; nn <= DWIDTH; nn <<= 1) {
			flp_dwt_inv(dat, tmp, nn, flp_coif5_h, flp_coif5_g, WLTLEN(flp_coif5_h));
			memcpy(dat, tmp, sizeof(float) * nn);
		}

		print_filt_progress(name, l, "converting floating-point to bytes");

		/* Convert floating-point data back to bytes */
		for(nn = 0; nn < DWIDTH; ++nn)
			filtered[l][nn] = float2byte(dat[nn]);
	}

#if INTERM_OUTPUT == 1
	/* Output filtered data as PGM picture */
	interrupts_disable();
	print_picture(name, DWIDTH, DHEIGHT, &filtered[0][0]);
	interrupts_enable();
#endif

	print_filt_progress(name, -1, "verifying result");

	/* Verify filtered data */
	for(l = 0; l < DHEIGHT; ++l)
		for(nn = 0; nn < DWIDTH; ++nn)
			if(byte_diff(ref_data[l*DWIDTH + nn], filtered[l][nn]) > ROUND_ERROR_B)
				test_failed();
}


/* Filtering work function */
void flp_filter_work_func(const struct work *w)
{
	u64 start, end, cycles;
	(void)cycles;	/* Keep compiler happy */

	INTDIS();
	PRINTS("Work "); PRINTS(w->name); PRINTS(" started\n");
	INTEN();

	start = rdtsc_safe();

	/* Filter data using floating-point computations */
	flp_filter_data(w->name, w->uptr1, w->uptr2);

	++ncompl_works;	/* Count completed work */

	end = rdtsc_safe();

	cycles = passed_cycles(start, end);

	INTDIS();
	PRINTS("Work "); PRINTS(w->name); PRINTS(" done!");
	PRINTS(" ("); PRINTU64(cycles); PRINTS(" cycles passed)\n");
	INTEN();
}


/********************* Fixed-point wavelet filtering **************************/

/* Compute threshold for filtering */
fixed_t fxp_threshold(fixed_t *data, size_t n)
{
	fixed_t thr = 0;
	size_t i;
	for(i = 0; i < n; ++i) {
		fixed_t v = (data[i] >= 0 ? data[i] : FXP_NEG(data[i]));
		if(v > thr)
			thr = v;
	}
	return FXP_DIV(thr, FXP_INTEGER(2));
}


/* Byte to fixed-point */
static inline
fixed_t byte2fixed(u8 b)
{
	return (fixed_t)b << (FXP_FRAC_BITS - 8);
}


/* Fixed-point to byte */
static inline
u8 fixed2byte(fixed_t v)
{
	v >>= (FXP_FRAC_BITS - 8);
	return (v < 0 ? 0 : (v > 255 ? 255 : v));
}


/* Filter data using fixed-point wavelet filters */
void fxp_filter_data(const char *name, const u8 *src_data, const u8 *ref_data)
{
	size_t l;
	size_t nn;
	static fixed_t dat[DWIDTH];
	static fixed_t tmp[DWIDTH];
	static u8 filtered[DHEIGHT][DWIDTH];


	for(l = 0; l < DHEIGHT; ++l) {
		fixed_t thr;

		print_filt_progress(name, l, "converting bytes to fixed-point");

		/* Convert data to fixed-point values */
		for(nn = 0; nn < DWIDTH; ++nn)
			dat[nn] = byte2fixed(src_data[l*DWIDTH + nn]);

		print_filt_progress(name, l, "forward wavelet transform");

		/* Apply forward wavelet transform using Coiflet5 wavelet */
		for(nn = DWIDTH; nn >= 4; nn >>= 1) {
			fxp_dwt_fwd(dat, tmp, nn, fxp_coif5_h, fxp_coif5_g, WLTLEN(fxp_coif5_h));
			memcpy(dat, tmp, sizeof(fixed_t) * nn);
		}

		print_filt_progress(name, l, "thresholding level 1 coefficients");

		/* Threshold wavelet coefficients of level 1 */
		thr = fxp_threshold(&dat[DWIDTH>>1], DWIDTH>>1);
		for(nn = DWIDTH>>1; nn < DWIDTH; ++nn) {
			fixed_t v = (dat[nn] >= 0 ? dat[nn] : FXP_NEG(dat[nn]));
			dat[nn] = (v >= thr ? dat[nn] : 0);
		}

		print_filt_progress(name, l, "thresholding level 2 coefficients");

		/* Threshold wavelet coefficients of level 2 */
		thr = fxp_threshold(&dat[DWIDTH>>2], DWIDTH>>2);
		for(nn = DWIDTH>>2; nn < DWIDTH>>1; ++nn) {
			fixed_t v = (dat[nn] >= 0 ? dat[nn] : FXP_NEG(dat[nn]));
			dat[nn] = (v >= thr ? dat[nn] : 0);
		}

		print_filt_progress(name, l, "inverse wavelet transform");

		/* Apply inverse wavelet transform */
		for(nn = 4; nn <= DWIDTH; nn <<= 1) {
			fxp_dwt_inv(dat, tmp, nn, fxp_coif5_h, fxp_coif5_g, WLTLEN(fxp_coif5_h));
			memcpy(dat, tmp, sizeof(fixed_t) * nn);
		}

		print_filt_progress(name, l, "converting fixed-point to bytes");

		/* Convert fixed-point data back to bytes */
		for(nn = 0; nn < DWIDTH; ++nn)
			filtered[l][nn] = fixed2byte(dat[nn]);
	}

#if INTERM_OUTPUT == 1
	/* Output filtered data as PGM picture */
	interrupts_disable();
	print_picture(name, DWIDTH, DHEIGHT, &filtered[0][0]);
	interrupts_enable();
#endif

	print_filt_progress(name, -1, "verifying result");

	/* Verify filtered data */
	for(l = 0; l < DHEIGHT; ++l)
		for(nn = 0; nn < DWIDTH; ++nn)
			if(ref_data[l*DWIDTH + nn] != filtered[l][nn])
				test_failed();
}


/* Filtering work function */
void fxp_filter_work_func(const struct work *w)
{
	u64 start, end, cycles;
	(void)cycles;	/* Keep compiler happy */

	INTDIS();
	PRINTS("Work "); PRINTS(w->name); PRINTS(" started\n");
	INTEN();

	start = rdtsc_safe();

	/* Filter data using fixed-point computations */
	fxp_filter_data(w->name, w->uptr1, w->uptr2);

	++ncompl_works;	/* Count completed work */

	end = rdtsc_safe();

	cycles = passed_cycles(start, end);

	INTDIS();
	PRINTS("Work "); PRINTS(w->name); PRINTS(" done!");
	PRINTS(" ("); PRINTU64(cycles); PRINTS(" cycles passed)\n");
	INTEN();
}


/************** Floating-point wavelet function reconstruction ****************/

/* Reconstruct wavelet function shape using floating-point filters */
void flp_reconst(float *out, const float *h, const float *g, size_t ncoef)
{
	size_t nn;
	static float tmp[DLENGTH];

	for(nn = 0; nn < DLENGTH; ++nn) out[nn] = 0.0;

	out[8] = 1.0;	/* Scaling function */
	out[56] = 1.0;	/* Wavelet function */

	/* Inverse wavelet transform */
	for(nn = 64; nn <= DLENGTH; nn <<= 1) {
		flp_dwt_inv(out, tmp, nn, h, g, ncoef);
		memcpy(out, tmp, sizeof(float) * nn);
	}
}


/* Wavelet shape reconstruction work function */
void flp_reconstr_work_func(const struct work *w)
{
	static float s[DLENGTH];
	const float *h;
	const float *g;
	size_t n;
	u64 start, end, cycles;
	const char *name;
	(void)name;	/* Keep compiler happy */
	(void)cycles;	/* same */

	const float *ref = (float*)w->uptr1;

	INTDIS();
	PRINTS("Work "); PRINTS(w->name); PRINTS(" started\n");
	INTEN();

	start = rdtsc_safe();

	/* Choose wavelet function for reconstruction */
	if(w->uptr1 == flp_db2_func) {
		h = flp_db2_h;
		g = flp_db2_g;
		n = WLTLEN(flp_db2_h);
		name = "flp_db2";
	} else if(w->uptr1 == flp_rbio22_func) {
		h = flp_rbio22_h;
		g = flp_rbio22_g;
		n = WLTLEN(flp_rbio22_h);
		name = "flp_rbio2.2";
	} else {
		test_failed();
		return;
	}

	/* Reconstruct wavelet function shape using floating-point computations */
	flp_reconst(s, h, g, n);

#if INTERM_OUTPUT == 1
	interrupts_disable();
	print_flp_wavelet(name, s, DLENGTH);
	interrupts_enable();
#endif

	/* Verify result */
	for(n = 0; n < DLENGTH; ++n)
		if(fabs(s[n] - ref[n]) > ROUND_ERROR_F)
			test_failed();

	++ncompl_works;	/* Count completed work */

	end = rdtsc_safe();

	cycles = passed_cycles(start, end);

	INTDIS();
	PRINTS("Work "); PRINTS(w->name); PRINTS(" done!");
	PRINTS(" ("); PRINTU64(cycles); PRINTS(" cycles passed)\n");
	INTEN();
}


/**************** Fixed-point wavelet function reconstruction *****************/

/* Reconstruct wavelet function shape using fixed-point filters */
void fxp_reconst(fixed_t *out, const fixed_t *h, const fixed_t *g, size_t ncoef)
{
	size_t nn;
	static fixed_t tmp[DLENGTH];

	for(nn = 0; nn < DLENGTH; ++nn) out[nn] = 0.0;

	out[8] = FXP_ONE;	/* Scaling function */
	out[56] = FXP_ONE;	/* Wavelet function */

	/* Inverse wavelet transform */
	for(nn = 64; nn <= DLENGTH; nn <<= 1) {
		fxp_dwt_inv(out, tmp, nn, h, g, ncoef);
		memcpy(out, tmp, sizeof(fixed_t) * nn);
	}
}


/* Wavelet shape reconstruction work function */
void fxp_reconstr_work_func(const struct work *w)
{
	static fixed_t s[DLENGTH];
	const fixed_t *h;
	const fixed_t *g;
	size_t n;
	u64 start, end, cycles;
	const char *name;
	(void)name;	/* Keep compiler happy */
	(void)cycles;	/* same */

	const fixed_t *ref = (fixed_t*)w->uptr1;

	INTDIS();
	PRINTS("Work "); PRINTS(w->name); PRINTS(" started\n");
	INTEN();

	start = rdtsc_safe();

	/* Choose wavelet function for reconstruction */
	if(w->uptr1 == fxp_db2_func) {
		h = fxp_db2_h;
		g = fxp_db2_g;
		n = WLTLEN(fxp_db2_h);
		name = "fxp_db2";
	} else if(w->uptr1 == fxp_rbio22_func) {
		h = fxp_rbio22_h;
		g = fxp_rbio22_g;
		n = WLTLEN(fxp_rbio22_h);
		name = "fxp_rbio2.2";
	} else {
		test_failed();
		return;
	}

	/* Reconstruct wavelet function shape using fixed-point computations */
	fxp_reconst(s, h, g, n);

#if INTERM_OUTPUT == 1
	interrupts_disable();
	print_fxp_wavelet(name, s, DLENGTH);
	interrupts_enable();
#endif

	/* Verify result */
	for(n = 0; n < DLENGTH; ++n)
		if(s[n] != ref[n])
			test_failed();

	++ncompl_works;	/* Count completed work */

	end = rdtsc_safe();

	cycles = passed_cycles(start, end);

	INTDIS();
	PRINTS("Work "); PRINTS(w->name); PRINTS(" done!");
	PRINTS(" ("); PRINTU64(cycles); PRINTS(" cycles passed)\n");
	INTEN();
}


/************************** Task functions ************************************/

/* Filtering task */
void filtering_task_func(void)
{
	INTDIS();
	PRINTS("Filter task started\n");
	INTEN();

	while(1) {
		struct work *w;

		/* Check queue for available works */
		interrupts_disable();
		w = (struct work *)queue_pop_head(&filtwqueue);
		if(!w) {
			waiti_safe();	/* Wait if nothing to do */
			barrier();	/* Invalidate all previously loaded data */
			continue;
		} else
			interrupts_enable();

		/* Run work */
		w->func(w);
	}
}


/* Wavelet function shape reconstruction task */
void reconstr_task_func(void)
{
	INTDIS();
	PRINTS("Reconstruction task started\n");
	INTEN();

	while(1) {
		struct work *w;

		/* Check queue for available works */
		interrupts_disable();
		w = (struct work *)queue_pop_head(&reconwqueue);
		if(!w) {
			waiti_safe();	/* Wait if nothing to do */
			barrier();	/* Invalidate all previously loaded data */
			continue;
		} else
			interrupts_enable();

		/* Run work */
		w->func(w);
	}
}
