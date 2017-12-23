/*
 * Copyright (c) 2015-2017 The Ultiparc Project. All rights reserved.
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
 * Integer Overflow exception test
 */

#include <arch.h>
#include <test_defines.h>

/*
 * Notes: Any exception other than Integer Overflow fails the test.
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
	j overflow_handler	/* Jump to exception handler */
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

	li $t0, 0xCAFEBABE	/* Value of destination register */

	/* Add two negative values which cause result overflow */
	li $t1, 0x80000000
	li $t2, 0xFFFF8000
	add $t0, $t1, $t2
	nop
	addi $t0, $t1, 0x8000
	nop
	/* Add two positive values which cause result overflow */
	li $t1, 0x7FFFFFFF
	li $t2, 0x7FFF
	add $t0, $t1, $t2
	nop
	addi $t0, $t1, 0x7FFF
	nop
	/* Subtract values which cause result overflow */
	li $t1, 0x7FFFFFFF
	li $t2, 0xFFFF8000
	sub $t0, $t1, $t2
	nop
	li $t1, 0x80000000
	li $t2, 0x7FFF
	sub $t0, $t1, $t2
	nop

	/** Now repeat tests for delay slots **/

	/* Add two negative values with overflow */
	li $t1, 0x80000000
	li $t2, 0xFFFF8000
	j test_failed
	add $t0, $t1, $t2
	j test_failed
	addi $t0, $t1, 0x8000
	/* Add two positive values with overflow */
	li $t1, 0x7FFFFFFF
	li $t2, 0x7FFF
	j test_failed
	add $t0, $t1, $t2
	j test_failed
	addi $t0, $t1, 0x7FFF
	/* Subtract values with overflow */
	li $t1, 0x7FFFFFFF
	li $t2, 0xFFFF8000
	j test_failed
	sub $t0, $t1, $t2
	li $t1, 0x80000000
	li $t2, 0x7FFF
	j test_failed
	sub $t0, $t1, $t2

	/* Check value of destination (should be unchanged) */
	li $t1, 0xCAFEBABE
	bne $t0, $t1, test_failed
	nop

check_result:
	/* Should get 12 Integer Overflow exceptions */
	li $t0, 12
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

	/*** Overflow Exception handler ***/
overflow_handler:
	/* Increment counter */
	addiu $k0, $k0, 1
	/* Get return address */
	mfc0 $ra, $EPC
	/* Advance return address */
	addiu $ra, $ra, 8
	/* Return from exception */
	jr $ra
	rfe
