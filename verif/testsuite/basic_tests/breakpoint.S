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
 * Breakpoint exception test
 */

#include <test_defines.h>

/*
 * Notes: Any exception other than Breakpoint fails the test.
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
/******************************* Address Overflow *****************************/
address_error_exception:
	j test_failed
	nop
/***************************** Reserved Instruction ***************************/
resvd_instr_exception:
	j test_failed
	nop
/********************************** Breakpoint ********************************/
break_exception:
	j breakpoint_handler	/* Jump to exception handler */
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
	break 0x0		/* Breakpoint */
	j check_result		/* Breakpoint in delay slot */
	break 0x0

check_result:
	/* $k0 should contain two successful tries, otherwise test failed */
	li $t0, 2
	bne $t0, $k0, test_failed
	nop /* delay slot */

test_passed:
	/* Report success to simulator */
	li $t0, CTRL_DEV_IOBASE
	li $t1, TEST_PASSED
	sw $t1, 0($t0)
	j test_passed
	nop

test_failed:
	/* Report failure to simulator */
	li $t0, CTRL_DEV_IOBASE
	li $t1, TEST_FAILED
	sw $t1, 0($t0)
	j test_failed
	nop

	/*** Breakpoint Exception handler ***/
breakpoint_handler:
	/* Increment counter */
	addiu $k0, $k0, 1
	/* Get return address */
	mfc0 $ra, $EPC
	/* Load faulted instruction */
	lw $t0, 0($ra)
	/* If instruction is break then increment $ra by 4 otherwise increment
	 * by 8 because it is in delay slot.
	 */
	li $t1, 0xd
	beq $t0, $t1, exception_return
	addiu $ra, $ra, 4
	addiu $ra, $ra, 4
exception_return:
	jr $ra
	rfe
