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
 * Fixed-point Taylor series (Q20.12 fixed-point format)
 */

#include <arch.h>
#include <test_defines.h>
#include "fxp12.h"


#define ARRAY_LENGTH(a) \
	(sizeof(a) / sizeof(a[0]))


/* Values for interval -1..1 */
const fixed_t ten_terms[] = {
	-4096, -3686, -3276, -2867, -2457, -2048, -1638, -1228, -819, -409, 0,
	409, 819, 1228, 1638, 2047, 2457, 2867, 3276, 3686, 4095
};

/* Sine values computed using 10 terms of Taylor series */
const fixed_t sin10[] = {
	-3448, -3209, -2938, -2638, -2312, -1964, -1595, -1210, -814, -409, 0,
	409, 814, 1210, 1595, 1963, 2312, 2638, 2938, 3209, 3447
};

/* Cosine values computed using 10 terms of Taylor series */
const fixed_t cos10[] = {
	2213, 2546, 2854, 3133, 3382, 3594, 3773, 3913, 4015, 4076, 4096, 4076,
	4015, 3913, 3773, 3595, 3382, 3133, 2854, 2546, 2214
};

/* Values for interval -PI..PI */
const fixed_t four_terms[] = {
	-12867, -12458, -12048, -11639, -11229, -10819, -10410, -10000, -9591,
	-9181, -8771, -8362, -7952, -7543, -7133, -6723, -6314, -5904, -5495,
	-5085, -4675, -4266, -3856, -3447, -3037, -2627, -2218, -1808, -1399,
	-989, -579, -170, 239, 648, 1058, 1468, 1877, 2287, 2696, 3106, 3516,
	3925, 4335, 4744, 5154, 5564, 5973, 6383, 6792, 7202, 7612, 8021, 8431,
	8840, 9250, 9660, 10069, 10479, 10888, 11298, 11708, 12117, 12527
};

/* Sine values computed using 4 terms of Taylor series */
const fixed_t sin4[] = {
	-30, -430, -828, -1221, -1602, -1969, -2316, -2640, -2939, -3211, -3448,
	-3650, -3818, -3946, -4036, -4085, -4095, -4062, -3989, -3876, -3724,
	-3535, -3312, -3055, -2766, -2450, -2111, -1750, -1372, -980, -577,
	-170, 239, 646, 1047, 1437, 1812, 2170, 2506, 2817, 3100, 3352, 3570,
	3753, 3897, 4004, 4071, 4095, 4080, 4024, 3927, 3792, 3619, 3410, 3166,
	2892, 2589, 2259, 1908, 1538, 1154, 761, 362
};

/* Cosine values computed using 4 terms of Taylor series */
const fixed_t cos4[] = {
	-3998, -4005, -3963, -3877, -3746, -3577, -3369, -3124, -2848, -2543,
	-2210, -1856, -1483, -1095, -695, -289, 120, 527, 931, 1325, 1706, 2068,
	2412, 2729, 3022, 3282, 3510, 3703, 3860, 3977, 4056, 4093, 4090, 4045,
	3960, 3835, 3673, 3474, 3241, 2974, 2677, 2355, 2010, 1643, 1259, 864,
	460, 51, -358, -763, -1162, -1547, -1917, -2268, -2596, -2897, -3168,
	-3406, -3608, -3772, -3895, -3973, -4006
};


/* Any exception fails the test */
void interrupt_entry(struct interrupt_frame *p)
{
	test_failed();
}


/* Fixed-point sine */
fixed_t sine_fxp(fixed_t x, unsigned n)
{
	const fixed_t one = FXP_INTEGER(1);
	const fixed_t two = FXP_INTEGER(2);
	fixed_t s = x;
	fixed_t xm = x;
	fixed_t x2 = FXP_MUL(x, x);
	fixed_t f = one;
	fixed_t fm = one;
	int i;

	for(i=1; i<=n; ++i) {
		xm = FXP_MUL( xm, x2 );
		fm = FXP_MUL(fm, FXP_MUL(FXP_ADD(f, one), FXP_ADD(f, two)));
		f = FXP_ADD(f, two);
		if(i&1)
			s = FXP_SUB(s, FXP_DIV(xm, fm));
		else
			s = FXP_ADD(s, FXP_DIV(xm, fm));
	}
	return s;
}


/* Fixed-point cosine */
fixed_t cosine_fxp(fixed_t x, unsigned n)
{
	const fixed_t one = FXP_INTEGER(1);
	const fixed_t two = FXP_INTEGER(2);
	fixed_t s = one;
	fixed_t xm = one;
	fixed_t x2 = FXP_MUL(x, x);
	fixed_t f = FXP_ZERO;
	fixed_t fm = one;
	int i;

	for(i=1; i<=n; ++i) {
		xm = FXP_MUL(xm, x2);
		fm = FXP_MUL(fm, FXP_MUL(FXP_ADD(f, one), FXP_ADD(f, two)));
		f = FXP_ADD(f, two);
		if(i&1)
			s = FXP_SUB(s, FXP_DIV(xm, fm));
		else
			s = FXP_ADD(s, FXP_DIV(xm, fm));
	}
	return s;
}


/* Test start */
void user_entry()
{
	unsigned i;

	/*
	 * Sine/cosine values on interval -1..1 computed using 10 terms of
	 * Taylor series.
	 */
	for(i = 0; i<ARRAY_LENGTH(ten_terms); ++i) {
		fixed_t sin = sine_fxp(ten_terms[i], 10);
		fixed_t cos = cosine_fxp(ten_terms[i], 10);
		if(sin != sin10[i] || cos != cos10[i])
			test_failed();
	}

	/*
	 * Sine/cosine values on interval -PI..PI computed using 4 terms of
	 * Taylor series.
	 */
	for(i = 0; i<ARRAY_LENGTH(four_terms); ++i) {
		fixed_t sin = sine_fxp(four_terms[i], 4);
		fixed_t cos = cosine_fxp(four_terms[i], 4);
		if(sin != sin4[i] || cos != cos4[i])
			test_failed();
	}

	test_passed();
}
