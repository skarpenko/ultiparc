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
 * Common routines used by system-level tests
 */

#include <arch.h>
#include <test_defines.h>


void *memcpy(void *dst, const void *src, unsigned int count)
{
/* Word size and mask */
#define wsize	sizeof(word_t)
#define wmask	(wsize - 1)

	char *d = (char *)dst;
	const char *s = (const char *)src;
	int len;

	if(count == 0 || dst == src)
		return dst;

	if(((word_t)d | (word_t)s) & wmask) {
		/* src and/or dst is not word aligned */
		if((((word_t)d ^ (word_t)s) & wmask) || (count < wsize))
			len = count; /* copy leftover using using byte access */
		else
			len = wsize - ((word_t)d & wmask); /* word align pointers */

		count -= len;
		for(; len > 0; len--)
			*d++ = *s++;
	}
	/* words copy */
	for(len = count / wsize; len > 0; len--) {
		*(word_t *)d = *(word_t *)s;
		d += wsize;
		s += wsize;
	}
	/* bytes copy */
	for(len = count & wmask; len > 0; len--)
		*d++ = *s++;

	return dst;
#undef wsize
#undef wmask
}


void print_char(char ch)
{
	writel(ch, MUART_CHREG);
}


void print_str(const char *str)
{
	if(str) {
		while(*str) {
			writel(*str, MUART_CHREG);
			++str;
		}
	} else
		print_str("<NULL>");
}
