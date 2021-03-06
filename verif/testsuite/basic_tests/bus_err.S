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
 * Bus Error exception test
 */

#include <arch.h>
#include <test_defines.h>

/*
 * Notes: Any exception other than Bus Error fails the test.
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
	j bus_error_handler	/* Jump to exception handler */
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
	xor $k0, $k0, $k0	/* Use $k0 for counting successful steps */
	li $k1, 2		/* Step number for faulty jump (see handler) */

	/* Wrong bus address */
	li $t0, SIM_CTRL_BERREG

	/* Constants to check */
	li $t1, 0xDEADBEEF
	li $t2, 0xDEADBEEF

	/* Load word from wrong address */
	lw $t1, 0($t0)
	nop
	/* Destination register should keep previous value */
	bne $t1, $t2, test_failed
	nop

	/* Jump to wrong address */
faulty_jump:
	jr $t0
	nop

	/* Load word from wrong address at delay slot */
	j test_failed
	lw $t1, 0($t0)
	nop
	/* Destination register should keep previous value */
	bne $t1, $t2, test_failed
	nop

	/* Store word to wrong address */
	sw $t1, 0($t0)
	nop

check_result:
	/* Should get 4 Bus Error exceptions */
	li $t0, 4
	bne $t0, $k0, test_failed
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

	/*** Bus Error Exception handler ***/
bus_error_handler:
	/* Increment counter */
	addiu $k0, $k0, 1
	/* Get return address */
	mfc0 $ra, $EPC
	/* Faulty jump causes bus error at fetch from
	 * wrong address. Return back to valid location.
	 */
	bne $k0, $k1, ret_from_handler
	nop
	la $ra, faulty_jump

ret_from_handler:
	/* Advance return address */
	addiu $ra, $ra, 8
	/* Return */
	jr $ra
	rfe
