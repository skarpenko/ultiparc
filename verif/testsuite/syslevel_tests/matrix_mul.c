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
 * Matrix multiply
 */

#include <arch.h>
#include <test_defines.h>
#include "common.h"


#define N	10	/* Matrix dimensions */


static int A[N][N] = {
	{  0,  1,  2,  3,  4,  5,  6,  7,  8,  9 },
	{  9,  8,  7,  6,  5,  4,  3,  2,  1,  0 },
	{  0, -1, -2, -3, -4, -5, -6, -7, -8, -9 },
	{  9,  8,  7,  6,  5,  4,  3,  2,  1,  0 },
	{  0,  1,  2,  3,  4,  5,  6,  7,  8,  9 },
	{ -9, -8, -7, -6, -5, -4, -3, -2, -1,  0 },
	{  0,  1,  2,  3,  4,  5,  6,  7,  8,  9 },
	{  9,  8,  7,  6,  5,  4,  3,  2,  1,  0 },
	{  0, -1, -2, -3, -4, -5, -6, -7, -8, -9 },
	{  9,  8,  7,  6,  5,  4,  3,  2,  1,  0 }
};


static int B[N][N] = {
	{  9,  8,  7,  6,  5,  4,  3,  2,  1,  0 },
	{  0, -1, -2, -3, -4, -5, -6, -7, -8, -9 },
	{  9,  8,  7,  6,  5,  4,  3,  2,  1,  0 },
	{  0,  1,  2,  3,  4,  5,  6,  7,  8,  9 },
	{ -9, -8, -7, -6, -5, -4, -3, -2, -1,  0 },
	{  0,  1,  2,  3,  4,  5,  6,  7,  8,  9 },
	{  9,  8,  7,  6,  5,  4,  3,  2,  1,  0 },
	{  0, -1, -2, -3, -4, -5, -6, -7, -8, -9 },
	{  9,  8,  7,  6,  5,  4,  3,  2,  1,  0 },
	{  0,  1,  2,  3,  4,  5,  6,  7,  8,  9 }
};


static int R[N][N] = { 0 };


static const int GOLD[N][N] = {
	{  108,  105,  102,  99,  96,  93,  90,  87,  84,  81 },
	{  135,  120,  105,  90,  75,  60,  45,  30,  15,   0 },
	{ -108, -105, -102, -99, -96, -93, -90, -87, -84, -81 },
	{  135,  120,  105,  90,  75,  60,  45,  30,  15,   0 },
	{  108,  105,  102,  99,  96,  93,  90,  87,  84,  81 },
	{ -135, -120, -105, -90, -75, -60, -45, -30, -15,   0 },
	{  108,  105,  102,  99,  96,  93,  90,  87,  84,  81 },
	{  135,  120,  105,  90,  75,  60,  45,  30,  15,   0 },
	{ -108, -105, -102, -99, -96, -93, -90, -87, -84, -81 },
	{  135,  120,  105,  90,  75,  60,  45,  30,  15,   0 }
};


/* Test start */
void user_entry()
{
	int i, j, k;

	/* Multiply */
	for(i=0; i<N; ++i) {
		for(j=0; j<N; ++j) {
			R[i][j] = 0;
			for(k=0; k<N; ++k)
				R[i][j] += A[i][k] * B[k][j];
		}
	}

	/* Verify result */
	for(i=0; i<N; ++i) {
		for(j=0; j<N; ++j) {
			if(R[i][j] != GOLD[i][j])
				test_failed();
		}
	}

	test_passed();
}
