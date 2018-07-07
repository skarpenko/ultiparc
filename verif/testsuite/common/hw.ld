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
 * Linker script
 */


OUTPUT_FORMAT("elf32-littlemips", "elf32-bigmips",
		"elf32-littlemips")
OUTPUT_ARCH(mips)
ENTRY(__reset)


MEMORY
{
	ram(rwx): ORIGIN = 0x00000000, LENGTH = 256K
}


SECTIONS
{
	.startup	0x00000000 : { *(.startup .startup.*) } > ram
	.text		ALIGN(0x4) : { *(.text .text.*) } > ram
	.rodata		ALIGN(0x4) : { *(.rodata .rodata.*) } > ram
	.rodata1	ALIGN(0x4) : { *(.rodata1) } > ram
	.sdata2		ALIGN(0x4) : { *(.sdata2 .sdata2.*) } > ram
	.sbss2		ALIGN(0x4) : {
		__sbss2_start = ABSOLUTE(.);
		*(.sbss2 .sbss2.*);
		. = ALIGN(0x4);
		__sbss2_end = ABSOLUTE(.);
	} > ram
	.data		ALIGN(0x4) : { *(.data .data.*) } > ram
	.data1		ALIGN(0x4) : { *(.data1) } > ram
	HIDDEN(_gp = ALIGN(16) + 0x7ff0);
	.got		ALIGN(0x4) : { *(.got .got.*) } > ram
	.sdata		ALIGN(0x4) : { *(.sdata .sdata.*) } > ram
	.sbss		ALIGN(0x4) : {
		__sbss_start = ABSOLUTE(.);
		*(.sbss .sbss.*);
		. = ALIGN(0x4);
		__sbss_end = ABSOLUTE(.);
	} > ram
	.bss		ALIGN(0x4) : {
		__bss_start = ABSOLUTE(.);
		*(.bss .bss.*);
		. = ALIGN(0x4);
		__bss_end = ABSOLUTE(.);
	} > ram
	/DISCARD/ : { *(.note.GNU-stack) *(.gnu_debuglink) *(.reginfo) }

	PROVIDE(__stack_top = ABSOLUTE(ORIGIN("ram") + LENGTH("ram")));
}
