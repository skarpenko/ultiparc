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
 * Multiply and divide instructions test
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
	/* Load test values */
	li $t7, 0xA		/*  10 */
	li $t6, 0x3		/*   3  */
	li $t5, 0xFFFFFFF6	/* -10 */
	li $t4, 0xFFFFFFFD	/*  -3 */

	/** Signed multiply **/

	/* A mult B */
	mult $t7, $t6
	nop
	nop
	mfhi $t0
	bne $t0, $zero, test_failed
	nop
	mflo $t0
	li $t1, 0x1E
	bne $t0, $t1, test_failed
	nop

	/* -A mult -B */
	mult $t5, $t4
	nop
	nop
	mfhi $t0
	bne $t0, $zero, test_failed
	nop
	mflo $t0
	li $t1, 0x1E
	bne $t0, $t1, test_failed
	nop

	/* A mult -B */
	mult $t7, $t4
	nop
	nop
	mfhi $t0
	li $t1, 0xFFFFFFFF
	bne $t0, $t1, test_failed
	nop
	mflo $t0
	li $t1, 0xFFFFFFE2
	bne $t0, $t1, test_failed
	nop

	/* -A mult B */
	mult $t5, $t6
	nop
	nop
	mfhi $t0
	li $t1, 0xFFFFFFFF
	bne $t0, $t1, test_failed
	nop
	mflo $t0
	li $t1, 0xFFFFFFE2
	bne $t0, $t1, test_failed
	nop

	/** Unsigned multiply **/

	/* A multu B */
	multu $t7, $t6
	nop
	nop
	mfhi $t0
	bne $t0, $zero, test_failed
	nop
	mflo $t0
	li $t1, 0x1E
	bne $t0, $t1, test_failed
	nop

	/* -A multu -B */
	multu $t5, $t4
	nop
	nop
	mfhi $t0
	li $t1, 0xFFFFFFF3
	bne $t0, $t1, test_failed
	nop
	mflo $t0
	li $t1, 0x1E
	bne $t0, $t1, test_failed
	nop

	/* A multu -B */
	multu $t7, $t4
	nop
	nop
	mfhi $t0
	li $t1, 9
	bne $t0, $t1, test_failed
	nop
	mflo $t0
	li $t1, 0xFFFFFFE2
	bne $t0, $t1, test_failed
	nop

	/* -A multu B */
	multu $t5, $t6
	nop
	nop
	mfhi $t0
	li $t1, 2
	bne $t0, $t1, test_failed
	nop
	mflo $t0
	li $t1, 0xFFFFFFE2
	bne $t0, $t1, test_failed
	nop

	/** Signed divide **/

	/* A div B, A mod B */
	div $zero, $t7, $t6
	nop
	nop
	mfhi $t0
	li $t1, 1
	bne $t0, $t1, test_failed
	nop
	mflo $t0
	li $t1, 3
	bne $t0, $t1, test_failed
	nop

	/* -A div -B, -A mod -B */
	div $zero, $t5, $t4
	nop
	nop
	mfhi $t0
	li $t1, 0xFFFFFFFF
	bne $t0, $t1, test_failed
	nop
	mflo $t0
	li $t1, 3
	bne $t0, $t1, test_failed
	nop

	/* A div -B, A mod -B */
	div $zero, $t7, $t4
	nop
	nop
	mfhi $t0
	li $t1, 1
	bne $t0, $t1, test_failed
	nop
	mflo $t0
	li $t1, 0xFFFFFFFD
	bne $t0, $t1, test_failed
	nop

	/* -A div B, -A mod B */
	div $zero, $t5, $t6
	nop
	nop
	mfhi $t0
	li $t1, 0xFFFFFFFF
	bne $t0, $t1, test_failed
	nop
	mflo $t0
	li $t1, 0xFFFFFFFD
	bne $t0, $t1, test_failed
	nop

	/** Unsigned divide **/

	/* A divu B, A modu B */
	divu $zero, $t7, $t6
	nop
	nop
	mfhi $t0
	li $t1, 1
	bne $t0, $t1, test_failed
	nop
	mflo $t0
	li $t1, 3
	bne $t0, $t1, test_failed
	nop

	/* -A divu -B, -A modu -B */
	divu $zero, $t5, $t4
	nop
	nop
	mfhi $t0
	li $t1, 0xFFFFFFF6
	bne $t0, $t1, test_failed
	nop
	mflo $t0
	bne $t0, $zero, test_failed
	nop

	/* A divu -B, A modu -B */
	divu $zero, $t7, $t4
	nop
	nop
	mfhi $t0
	li $t1, 0xA
	bne $t0, $t1, test_failed
	nop
	mflo $t0
	bne $t0, $zero, test_failed
	nop

	/* -A divu B, -A modu B */
	divu $zero, $t5, $t6
	nop
	nop
	mfhi $t0
	bne $t0, $zero, test_failed
	nop
	mflo $t0
	li $t1, 0x55555552
	bne $t0, $t1, test_failed
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
