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
 * Pipeline hazards test
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
	/* Load values for testing */
	li $a3, 16
	li $a2, 8
	li $a1, 4
	li $a0, 2
	nop
	nop
	nop
	nop
	nop

	/** Structural hazards **/

	/* Div case */
	div $zero, $a2, $a0
	nop
	nop
	div $zero, $a2, $a1
	nop
	nop
	mflo $t0
	bne $t0, $a0, test_failed
	nop

	/* Mult case */
	mult $a1, $a0
	nop
	nop
	mult $a2, $a0
	nop
	nop
	mflo $t0
	bne $t0, $a3, test_failed
	nop

	/** Read after write hazards **/

	/* Write and following reads */
	addu $t0, $a0, $a0
	addu $t1, $t0, $a1
	sll $t2, $t0, 1
	sll $t3, $t0, 2
	bne $t1, $a2, test_failed
	nop
	bne $t2, $a2, test_failed
	nop
	bne $t3, $a3, test_failed
	nop

	/* Two writes and following reads */
	addu $t0, $a0, $a1
	addu $t0, $a0, $a0
	addu $t1, $t0, $a1
	sll $t2, $t0, 1
	sll $t3, $t0, 2
	bne $t1, $a2, test_failed
	nop
	bne $t2, $a2, test_failed
	nop
	bne $t3, $a3, test_failed
	nop

	/* Load to register with following reads */
	la $t1, test_data
	lw $t0, 0($t1)
	nop
	addu $t1, $t0, $a1
	sll $t2, $t0, 1
	sll $t3, $t0, 2
	bne $t1, $a2, test_failed
	nop
	bne $t2, $a2, test_failed
	nop
	bne $t3, $a3, test_failed
	nop

	/* Two loads to register with following reads */
	la $t1, test_data
	lw $t0, 4($t1)
	lw $t0, 0($t1)
	nop
	addu $t1, $t0, $a1
	sll $t2, $t0, 1
	sll $t3, $t0, 2
	bne $t1, $a2, test_failed
	nop
	bne $t2, $a2, test_failed
	nop
	bne $t3, $a3, test_failed
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

test_data:
	.long 4
	.long 8
