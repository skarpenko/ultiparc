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
 * Arithmetic operations on Q8.24 fixed-point values
 */

#ifndef _SYSLEVEL_TESTS_FXP24_H_
#define _SYSLEVEL_TESTS_FXP24_H_


/* Fixed-point types */
typedef signed long fixed_t;
typedef signed long long __extended_t;


/* Size of fractional part */
#define FXP_FRAC_BITS	24

/* Fixed-point 1.0 value */
#define FXP_ONE		(fixed_t)((fixed_t)1<<FXP_FRAC_BITS)
/* Fixed-point 0.0 value */
#define FXP_ZERO	(fixed_t)(0)

/* Convertion to fixed-point for integer values */
#define FXP_INTEGER(a)	(fixed_t)((a)<<FXP_FRAC_BITS)

/* Addition */
#define FXP_ADD(a,b)	\
	(fixed_t)((a)+(b))
/* Subtraction */
#define FXP_SUB(a,b)	\
	(fixed_t)((a)-(b))
/* Multiply */
#define FXP_MUL(a,b)	\
	(fixed_t)(((__extended_t)(a)*(__extended_t)(b))>>FXP_FRAC_BITS)
/* Divide */
#define FXP_DIV(a,b)	\
	(fixed_t)(((__extended_t)(a)<<FXP_FRAC_BITS)/(__extended_t)(b))
/* Negation */
#define FXP_NEG(a)	\
	(fixed_t)(-(a))


#endif /* _SYSLEVEL_TESTS_FXP24_H_ */
