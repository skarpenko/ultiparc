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
 * Floating-point Fast Fourier Transform with time measurements
 */

#include <arch.h>
#include <test_defines.h>
#include <math.h>
#include "common.h"


#define MAKE_NOISE	0
#define ROUND_ERROR	1E-10
#define TMRCOUNT	2000	/* Timer counter value */


volatile unsigned int_count = 0;	/* Timer ticks count */


/* Any exception fails the test */
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

	++int_count;	/* count tick */

	/* Acknowledge interrupt */
	status = readl(INTCTL_STATUS);
	writel(status, INTCTL_STATUS);
}


/* fast Fourier transform */
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
#define FFT_SIZE	64/*256*/
double f_r[FFT_SIZE], f_i[FFT_SIZE];
double g_r[FFT_SIZE], g_i[FFT_SIZE];


/* Test start */
void user_entry()
{
	unsigned i;
	unsigned start_time, duration;

#if MAKE_NOISE == 1
	print_str("Setting timer\n\n");
#endif

	writel(TMRCOUNT, ITIMER_COUNT);	/* Set timer counter */
	writel(7, ITIMER_CTLREG);	/* Enable timer (+reload and interrupt) */

	/* Unmask interrupt controller line */
	writel(1, INTCTL_MASK);

	interrupts_enable();


	/**** Prepare data for test ****/


#if MAKE_NOISE == 1
	print_str("Generating test data\n");
#endif

	start_time = int_count;
	for(i=0; i<FFT_SIZE; ++i) {
		f_r[i] = g_r[i] = 3.0 * cos(2.0 * M_PI * 0.08 * i) +
			2.0 * cos(2.0 * M_PI * 0.05 * i);
		f_i[i] = g_i[i] = 3.0 * sin(2.0 * M_PI * 0.08 * i) +
			2.0 * sin(2.0 * M_PI * 0.05 * i);
	}
	duration = int_count - start_time;

#if MAKE_NOISE == 1
	print_str("Timer ticks: "); print_unsigned(duration); print_str("\n\n");
#endif
	if(!duration)
		test_failed();


	/**** Forward transform ****/


#if MAKE_NOISE == 1
	print_str("FFT\n");
#endif

	start_time = int_count;
	fft(f_r, f_i, FFT_SIZE, 1);
	duration = int_count - start_time;

#if MAKE_NOISE == 1
	print_str("Timer ticks: "); print_unsigned(duration); print_str("\n\n");
#endif
	if(!duration)
		test_failed();


	/**** Inverse transform ****/


#if MAKE_NOISE == 1
	print_str("IFFT\n");
#endif

	start_time = int_count;
	fft(f_r, f_i, FFT_SIZE, -1);
	duration = int_count - start_time;

#if MAKE_NOISE == 1
	print_str("Timer ticks: "); print_unsigned(duration); print_str("\n\n");
#endif
	if(!duration)
		test_failed();


	/**** Normalization ****/


#if MAKE_NOISE == 1
	print_str("Normalization\n");
#endif

	start_time = int_count;
	for(i=0; i<FFT_SIZE; ++i) {
		f_r[i] /= (double)FFT_SIZE;
		f_i[i] /= (double)FFT_SIZE;
	}
	duration = int_count - start_time;

#if MAKE_NOISE == 1
	print_str("Timer ticks: "); print_unsigned(duration); print_str("\n\n");
#endif
	if(!duration)
		test_failed();


	/**** Result verification ****/


#if MAKE_NOISE == 1
	print_str("Verifying result\n");
#endif

	start_time = int_count;
	for(i=0; i<FFT_SIZE; ++i) {
		if(fabs(f_r[i] - g_r[i]) > ROUND_ERROR)
			test_failed();
		if(fabs(f_i[i] - g_i[i]) > ROUND_ERROR)
			test_failed();
	}
	duration = int_count - start_time;

#if MAKE_NOISE == 1
	print_str("Timer ticks: "); print_unsigned(duration); print_str("\n\n");
#endif
	if(!duration)
		test_failed();


	/* Successful termination */
	test_passed();
}
