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
 * Load / Store instructions test
 */

#include <arch.h>
#include <test_defines.h>

/*
 * Notes: Any exception fails the test.
 */

.text
.set noreorder
.globl __reset
/********************************* Reset Vector *******************************/
__reset:
	j test_entry	/* Jump to program start */
	nop
/*********************************** Bus Error ********************************/
bus_error_exception:
	j test_failed
	nop
/******************************* Integer Overflow *****************************/
overflow_exception:
	j test_failed
	nop
/********************************* Address Error ******************************/
address_error_exception:
	j test_failed
	nop
/***************************** Reserved Instruction ***************************/
resvd_instr_exception:
	j test_failed
	nop
/********************************** Breakpoint ********************************/
break_exception:
	j test_failed
	nop
/********************************** System Call *******************************/
syscall_exception:
	j test_failed
	nop
/****************************** Hardware Interrupt ****************************/
hardware_interrupt:
	j test_failed
	nop

/******************************* TEST ENTRY POINT *****************************/
test_entry:
	la $t0, load_pattern	/* Pattern for loads test */

	/* Load word */
	lw $t1, 0($t0)
	li $t2, 0xF3F2F1F0
	bne $t1, $t2, test_failed
	nop

	/* Load half word */
	lh $t1, 0($t0)
	li $t2, 0xFFFFF1F0
	bne $t1, $t2, test_failed
	nop
	lh $t1, 2($t0)
	li $t2, 0xFFFFF3F2
	bne $t1, $t2, test_failed
	nop

	/* Load half word unsigned */
	lhu $t1, 0($t0)
	li $t2, 0xF1F0
	bne $t1, $t2, test_failed
	nop
	lhu $t1, 2($t0)
	li $t2, 0xF3F2
	bne $t1, $t2, test_failed
	nop

	/* Load byte */
	lb $t1, 0($t0)
	li $t2, 0xFFFFFFF0
	bne $t1, $t2, test_failed
	nop
	lb $t1, 1($t0)
	li $t2, 0xFFFFFFF1
	bne $t1, $t2, test_failed
	nop
	lb $t1, 2($t0)
	li $t2, 0xFFFFFFF2
	bne $t1, $t2, test_failed
	nop
	lb $t1, 3($t0)
	li $t2, 0xFFFFFFF3
	bne $t1, $t2, test_failed
	nop

	/* Load byte unsigned */
	lbu $t1, 0($t0)
	li $t2, 0xF0
	bne $t1, $t2, test_failed
	nop
	lbu $t1, 1($t0)
	li $t2, 0xF1
	bne $t1, $t2, test_failed
	nop
	lbu $t1, 2($t0)
	li $t2, 0xF2
	bne $t1, $t2, test_failed
	nop
	lbu $t1, 3($t0)
	li $t2, 0xF3
	bne $t1, $t2, test_failed
	nop

	/* Store word */
	la $t0, sw_data
	li $t1, 0xE3E2E1E0
	sw $t1, 0($t0)
	lw $t2, 0($t0)
	bne $t1, $t2, test_failed
	nop

	/* Store half word */
	la $t0, sh_data
	li $t1, 0xE1E0
	sh $t1, 0($t0)
	lw $t2, 0($t0)
	bne $t1, $t2, test_failed
	nop
	li $t1, 0xE3E2
	sh $t1, 2($t0)
	lw $t2, 0($t0)
	li $t1, 0xE3E2E1E0
	bne $t1, $t2, test_failed
	nop

	/* Store byte */
	la $t0, sb_data
	li $t1, 0xE0
	sb $t1, 0($t0)
	lw $t2, 0($t0)
	bne $t1, $t2, test_failed
	nop
	li $t1, 0xE1
	sb $t1, 1($t0)
	lw $t2, 0($t0)
	li $t1, 0xE1E0
	bne $t1, $t2, test_failed
	nop
	li $t1, 0xE2
	sb $t1, 2($t0)
	lw $t2, 0($t0)
	li $t1, 0xE2E1E0
	bne $t1, $t2, test_failed
	nop
	li $t1, 0xE3
	sb $t1, 3($t0)
	lw $t2, 0($t0)
	li $t1, 0xE3E2E1E0
	bne $t1, $t2, test_failed
	nop

test_passed:
	/* Report success to simulator */
	li $t0, SIM_CTRL_CTLREG
	li $t1, TEST_PASSED
	sw $t1, 0($t0)
	j test_passed
	nop

test_failed:
	/* Report failure to simulator */
	li $t0, SIM_CTRL_CTLREG
	li $t1, TEST_FAILED
	sw $t1, 0($t0)
	j test_failed
	nop

	/*** Data section ***/
.data

load_pattern:
	.long 0xF3F2F1F0
sw_data:
	.long 0
sh_data:
	.long 0
sb_data:
	.long 0
