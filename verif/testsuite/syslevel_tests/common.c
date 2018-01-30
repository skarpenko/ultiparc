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
			print_char(*str);
			++str;
		}
	} else
		print_str("<NULL>");
}


void print_hex(unsigned hex)
{
	const static char h[] = "0123456789ABCDEF";
	print_char(h[(hex & 0xF0000000)>>28]);
	print_char(h[(hex & 0x0F000000)>>24]);
	print_char(h[(hex & 0x00F00000)>>20]);
	print_char(h[(hex & 0x000F0000)>>16]);
	print_char(h[(hex & 0x0000F000)>>12]);
	print_char(h[(hex & 0x00000F00)>>8]);
	print_char(h[(hex & 0x000000F0)>>4]);
	print_char(h[(hex & 0x0000000F)>>0]);
}


void print_integer(int v)
{
	int d;
	int o = 1000000000;
	int f = 0;

	if(v<0)
		print_char('-');

	while(o != 1) {
		d = v / o;
		v = v % o;
		o = o / 10;
		d = (d<0 ? -d : d);
		v = (v<0 ? -v : v);
		f = f | d;
		if(f) print_char('0' + d);
	}
	print_char('0' + v);
}


void print_unsigned(unsigned v)
{
	unsigned d;
	unsigned o = 1000000000;
	unsigned f = 0;

	while(o != 1) {
		d = v / o;
		v = v % o;
		o = o / 10;
		f = f | d;
		if(f) print_char('0' + d);
	}
	print_char('0' + v);
}


void print_integer64(long long v)
{
	long long d;
	long long o = 1000000000000000000;
	long long f = 0;

	if(v<0)
		print_char('-');

	while(o != 1) {
		d = v / o;
		v = v % o;
		o = o / 10;
		d = (d<0 ? -d : d);
		v = (v<0 ? -v : v);
		f = f | d;
		if(f) print_char('0' + d);
	}
	print_char('0' + v);
}


void print_unsigned64(unsigned long long v)
{
	unsigned long long d;
	unsigned long long o = 10000000000000000000;
	unsigned long long f = 0;

	while(o != 1) {
		d = v / o;
		v = v % o;
		o = o / 10;
		f = f | d;
		if(f) print_char('0' + d);
	}
	print_char('0' + v);
}
