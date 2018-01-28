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
 * WAIT instruction test.
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
	j hw_intr_handler
	nop

/******************************* TEST ENTRY POINT *****************************/
test_entry:
	/*** Interrupts should be disabled at this point. ***/

	/* Unmask timer interrupt */
	li $t0, INTCTL_MASK
	li $t1, 0x1
	sw $t1, 0($t0)

	/* Set timer count */
	li $t0, ITIMER_COUNT
	li $t1, 5000
	sw $t1, 0($t0)

	/* Start timer */
	li $t0, ITIMER_CTLREG
	li $t1, 0x7
	sw $t1, 0($t0)

	/* Read timestamp */
	mfc0 $t5, $TSCLO

	/* Enable interrupts */
	li $t0, 1
	mfc0 $t1, $SR
	or $t1, $t1, $t0
	mtc0 $t1, $SR

	/* Wait for interrupt */
	WAITI
	nop
	nop

	/* Read timestamp */
	mfc0 $t6, $TSCLO

	/* Check passed cycles count */
	sub $t5, $t6, $t5
	li $t6, 5000
	/* $t5 should be greater than $t6 */
	sltu $t2, $t6, $t5
	beq $t2, $zero, test_failed
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

	/*** Hardware interrupt handler ***/
hw_intr_handler:
	/* Get return address */
	mfc0 $ra, $EPC

	/* Mask all interrupts */
	li $t0, INTCTL_MASK
	li $t1, 0x0
	sw $t1, 0($t0)

	/* Return from exception */
	jr $ra
	rfe
