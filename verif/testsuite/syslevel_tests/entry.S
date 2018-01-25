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
 * Firmware startup code
 */

#include <arch.h>


.text
.set noreorder
.globl __reset
/********************************* Reset Vector *******************************/
__reset:
	j __entry	/* Jump to program entry point */
	nop
/*********************************** Bus Error ********************************/
__bus_error_exception:
	j __exception_entry
	addiu $k0, $zero, 1
/******************************* Integer Overflow *****************************/
__overflow_exception:
	j __exception_entry
	addiu $k0, $zero, 2
/********************************* Address Error ******************************/
__address_error_exception:
	j __exception_entry
	addiu $k0, $zero, 3
/***************************** Reserved Instruction ***************************/
__resvd_instr_exception:
	j __exception_entry
	addiu $k0, $zero, 4
/********************************** Breakpoint ********************************/
__break_exception:
	j __exception_entry
	addiu $k0, $zero, 5
/********************************** System Call *******************************/
__syscall_exception:
	j __exception_entry
	addiu $k0, $zero, 6
/****************************** Hardware Interrupt ****************************/
__hardware_interrupt:
	j __exception_entry
	addiu $k0, $zero, 7

/********************************** ENTRY POINT *******************************/
__entry:
	/* Set stack and global pointer */
	la $sp, __stack_top
	la $gp, _gp

	/* Zero .bss section */
	la $t0, __bss_start
	la $t1, __bss_end
__zero_bss:
	beq $t0, $t1, __zero_bss_end
	nop
	sw $zero, 0($t0)
	j __zero_bss
	addiu $t0, $t0, 4
__zero_bss_end:

	/* Zero .sbss section */
	la $t0, __sbss_start
	la $t1, __sbss_end
__zero_sbss:
	beq $t0, $t1, __zero_sbss_end
	nop
	sw $zero, 0($t0)
	j __zero_sbss
	addiu $t0, $t0, 4
__zero_sbss_end:

	/* Zero .sbss2 section */
	la $t0, __sbss2_start
	la $t1, __sbss2_end
__zero_sbss2:
	beq $t0, $t1, __zero_sbss2_end
	nop
	sw $zero, 0($t0)
	j __zero_sbss
	addiu $t0, $t0, 4
__zero_sbss2_end:

	/* Pass control to user code */
	.extern user_entry
	jal user_entry
	nop

	/* Return is not expected. Hang the system. */
	li $t0, ~1
	mfc0 $t1, $SR
	and $t1, $t1, $t0
	mtc0 $t1, $SR	/* Disable interrupts */
	nop
__hang:
	j __hang
	nop


/************************ Exception handler entry *****************************/

.set noat
__exception_entry:
	/* Registers $k0 and $k1 are ours! */

	move $k1, $sp	/* Save stack pointer */
	/* Adjust stack to store interrupt frame */
	addiu $sp, $sp, -INTR_FRAME_LEN*CPU_REG_SIZE

	/* Save vector number */
	sw $k0, 0*CPU_REG_SIZE($sp)

	/* Save Previous Status register (PSR) */
	mfc0 $k0, $PSR
	sw $k0, 1*CPU_REG_SIZE($sp)

	/* Save Status register (SR) */
	mfc0 $k0, $SR
	sw $k0, 2*CPU_REG_SIZE($sp)

	/* Save Cause register */
	mfc0 $k0, $CAUSE
	sw $k0, 3*CPU_REG_SIZE($sp)

	/* Save return address */
	mfc0 $k0, $EPC
	sw $k0, 4*CPU_REG_SIZE($sp)

	/* Save HI/LO pair */
	mflo $k0
	sw $k0, 5*CPU_REG_SIZE($sp)
	mfhi $k0
	sw $k0, 6*CPU_REG_SIZE($sp)

	/* Save general purpose registers */
	sw $ra, 7*CPU_REG_SIZE($sp)
	sw $fp, 8*CPU_REG_SIZE($sp)
	sw $k1, 9*CPU_REG_SIZE($sp)	/* Save original $sp */
	sw $gp, 10*CPU_REG_SIZE($sp)
	sw $t9, 11*CPU_REG_SIZE($sp)
	sw $t8, 12*CPU_REG_SIZE($sp)
	sw $s7, 13*CPU_REG_SIZE($sp)
	sw $s6, 14*CPU_REG_SIZE($sp)
	sw $s5, 15*CPU_REG_SIZE($sp)
	sw $s4, 16*CPU_REG_SIZE($sp)
	sw $s3, 17*CPU_REG_SIZE($sp)
	sw $s2, 18*CPU_REG_SIZE($sp)
	sw $s1, 19*CPU_REG_SIZE($sp)
	sw $s0, 20*CPU_REG_SIZE($sp)
	sw $t7, 21*CPU_REG_SIZE($sp)
	sw $t6, 22*CPU_REG_SIZE($sp)
	sw $t5, 23*CPU_REG_SIZE($sp)
	sw $t4, 24*CPU_REG_SIZE($sp)
	sw $t3, 25*CPU_REG_SIZE($sp)
	sw $t2, 26*CPU_REG_SIZE($sp)
	sw $t1, 27*CPU_REG_SIZE($sp)
	sw $t0, 28*CPU_REG_SIZE($sp)
	sw $a3, 29*CPU_REG_SIZE($sp)
	sw $a2, 30*CPU_REG_SIZE($sp)
	sw $a1, 31*CPU_REG_SIZE($sp)
	sw $a0, 32*CPU_REG_SIZE($sp)
	sw $v1, 33*CPU_REG_SIZE($sp)
	sw $v0, 34*CPU_REG_SIZE($sp)
	sw $at, 35*CPU_REG_SIZE($sp)

	/* Pass control to user exception handler */
	move $a0, $sp
	/* void interrupt_entry(struct interrupt_frame*) */
	.extern interrupt_entry
	jal interrupt_entry
	nop

	/* Disable interrupts before continue */
	li $t0, ~1
	mfc0 $t1, $SR
	and $t1, $t1, $t0
	mtc0 $t1, $SR
	nop

	/* Restore general purpose registers */
	lw $at, 35*CPU_REG_SIZE($sp)
	lw $v0, 34*CPU_REG_SIZE($sp)
	lw $v1, 33*CPU_REG_SIZE($sp)
	lw $a0, 32*CPU_REG_SIZE($sp)
	lw $a1, 31*CPU_REG_SIZE($sp)
	lw $a2, 30*CPU_REG_SIZE($sp)
	lw $a3, 29*CPU_REG_SIZE($sp)
	lw $t0, 28*CPU_REG_SIZE($sp)
	lw $t1, 27*CPU_REG_SIZE($sp)
	lw $t2, 26*CPU_REG_SIZE($sp)
	lw $t3, 25*CPU_REG_SIZE($sp)
	lw $t4, 24*CPU_REG_SIZE($sp)
	lw $t5, 23*CPU_REG_SIZE($sp)
	lw $t6, 22*CPU_REG_SIZE($sp)
	lw $t7, 21*CPU_REG_SIZE($sp)
	lw $s0, 20*CPU_REG_SIZE($sp)
	lw $s1, 19*CPU_REG_SIZE($sp)
	lw $s2, 18*CPU_REG_SIZE($sp)
	lw $s3, 17*CPU_REG_SIZE($sp)
	lw $s4, 16*CPU_REG_SIZE($sp)
	lw $s5, 15*CPU_REG_SIZE($sp)
	lw $s6, 14*CPU_REG_SIZE($sp)
	lw $s7, 13*CPU_REG_SIZE($sp)
	lw $t8, 12*CPU_REG_SIZE($sp)
	lw $t9, 11*CPU_REG_SIZE($sp)
	lw $gp, 10*CPU_REG_SIZE($sp)
	lw $k1, 9*CPU_REG_SIZE($sp)	/* $sp value goes to $k1 */
	lw $fp, 8*CPU_REG_SIZE($sp)
	lw $ra, 7*CPU_REG_SIZE($sp)

	/* Restore HI/LO pair */
	lw $k0, 6*CPU_REG_SIZE($sp)
	nop
	mthi $k0
	lw $k0, 5*CPU_REG_SIZE($sp)
	nop
	mtlo $k0

	/* Restore Previous Status register  */
	lw $k0, 1*CPU_REG_SIZE($sp)
	nop
	mtc0 $k0, $PSR

	/* Restore Status register  */
	lw $k0, 2*CPU_REG_SIZE($sp)
	nop
	mtc0 $k0, $SR
	/* This assumes that SR.IE = 0. Otherwise we are in big trouble since
	 * nested interrupt can clobber $k0 and $k1.
	 */

	/* Restore Cause register  */
	lw $k0, 3*CPU_REG_SIZE($sp)
	nop
	mtc0 $k0, $CAUSE

	/* Load return address */
	lw $k0, 4*CPU_REG_SIZE($sp)

	move $sp, $k1	/* Restore original $sp */

	/* Return from exception */
	jr $k0
	rfe
.set at

/************************* Default interrupt handler **************************/
.weak interrupt_entry
interrupt_entry:
	jr $ra
	nop

/*************************** Default user entry *******************************/
.weak user_entry
user_entry:
	jr $ra
	nop
