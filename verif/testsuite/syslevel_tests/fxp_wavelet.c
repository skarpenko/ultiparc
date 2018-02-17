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
 * Fixed-point wavelet transform (Q8.24 fixed-point format)
 */

#include <stddef.h>
#include <arch.h>
#include <test_defines.h>
#include "common.h"
#include "fxp24.h"
#include "wavelet_filt.h"
#include "wavelet_data.h"


#define MAKE_NOISE	0	/* Produce debug output */
#define INTERM_OUTPUT	0	/* Produce output of processed data */
#define SMALL_DATASET	1	/* Use smaller dataset */
#define ONE_DATASET	0	/* Use only one dataset */
#define PRINT_PROGRESS	0	/* Print filtering progress */


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
#else
# define PRINTS(a)
# define PRINTI(a)
# define PRINTU(a)
# define PRINTU64(a)
#endif


/********************** Wavelet transform routines ****************************/

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


/* Print filtering progress */
void print_filt_progress(const char *name, int data_row, const char *msg)
{
#if MAKE_NOISE == 1 && PRINT_PROGRESS == 1
	print_str(name);
	if(data_row >= 0) {
		print_str(": row = ");
		print_integer(data_row);
	}
	print_str(": ");
	print_str(msg);
	print_str("\n");
#endif
}


/* Get passed cycles count */
u64 passed_cycles(u64 start, u64 end)
{
	/* Deal with wraparound */
	return end < start ? -start + end : end - start;
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
	print_picture(name, DWIDTH, DHEIGHT, &filtered[0][0]);
#endif

	print_filt_progress(name, -1, "verifying result");

	/* Verify filtered data */
	for(l = 0; l < DHEIGHT; ++l)
		for(nn = 0; nn < DWIDTH; ++nn)
			if(ref_data[l*DWIDTH + nn] != filtered[l][nn])
				test_failed();
}


/************************ Main entry point ************************************/

/* Test start */
void user_entry()
{
	u64 cycles;


	/* Filter dataset 1 */

	PRINTS("Filtering data1 ...\n");

	cycles = rdtsc();

	fxp_filter_data("data1", &data1[0][0], &fxp_filt_data1[0][0]);

	cycles = passed_cycles(cycles, rdtsc());

	PRINTS("... done!");
	PRINTS(" ("); PRINTU64(cycles); PRINTS(" cycles passed)\n");


	/* Filter dataset 2 */

#if ONE_DATASET == 0
	PRINTS("Filtering data2 ...\n");

	cycles = rdtsc();

	fxp_filter_data("data2", &data2[0][0], &fxp_filt_data2[0][0]);

	cycles = passed_cycles(cycles, rdtsc());

	PRINTS("... done!");
	PRINTS(" ("); PRINTU64(cycles); PRINTS(" cycles passed)\n");
#endif	/* ONE_DATASET */


	/* Successful termination */
	test_passed();
}
