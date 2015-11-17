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
 * Fixed-point Fast Fourier Transform (Q8.24 fixed-point format)
 */

#include <arch.h>
#include <test_defines.h>
#include "fxp24.h"


/* Any exception fails the test */
void interrupt_entry(struct interrupt_frame *p)
{
	test_failed();
}


/* compute fixed-point fast Fourier transform */
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


/* Test start */
void user_entry()
{
	unsigned i;

	/* Forward transform */
	fxp_fft(f_r, f_i, N, 1);

	for(i=0; i<N; ++i)
		if(f_r[i] != F_r[i] || f_i[i] != F_i[i])
			test_failed();

	/* Inverse transform */
	fxp_fft(f_r, f_i, N, -1);

	/*
	 * We don't do FFT/IFFT scaling as a result amplitude of
	 * reconstructed signal differs from source signal.
	 */
	for(i=0; i<N; ++i)
		if(f_r[i] != f2_r[i] || f_i[i] != f2_i[i])
			test_failed();

	test_passed();
}
