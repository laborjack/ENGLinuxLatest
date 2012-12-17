/*
 * Copyright (C) 2012 - ARM Ltd
 * Author: Marc Zyngier <marc.zyngier@arm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/linkage.h>

#include <asm/assembler.h>
#include <asm/kvm_arm.h>
#include <asm/kvm_mmu.h>

	.text
	.pushsection	.hyp.idmap.text, "ax"

	.align	11

__kvm_hyp_init:
	.global	__kvm_hyp_init

ENTRY(__kvm_hyp_init_vector)
	ventry	__invalid		// Synchronous EL2t
	ventry	__invalid		// IRQ EL2t
	ventry	__invalid		// FIQ EL2t
	ventry	__invalid		// Error EL2t

	ventry	__invalid		// Synchronous EL2h
	ventry	__invalid		// IRQ EL2h
	ventry	__invalid		// FIQ EL2h
	ventry	__invalid		// Error EL2h

	ventry	__do_hyp_init		// Synchronous 64-bit EL1
	ventry	__invalid		// IRQ 64-bit EL1
	ventry	__invalid		// FIQ 64-bit EL1
	ventry	__invalid		// Error 64-bit EL1

	ventry	__invalid		// Synchronous 32-bit EL1
	ventry	__invalid		// IRQ 32-bit EL1
	ventry	__invalid		// FIQ 32-bit EL1
	ventry	__invalid		// Error 32-bit EL1
ENDPROC(__kvm_hyp_init_vector)

__invalid:
	b	.

	/*
	 * x0: HYP pgd
	 * x1: HYP stack
	 * x2: HYP vectors
	 */
__do_hyp_init:

	msr	ttbr0_el2, x0
	kern_hyp_va	x1
	mov	sp, x1
	kern_hyp_va	x2
	msr	vbar_el2, x2

	mrs	x0, tcr_el1
	ldr	x1, =TCR_EL2_MASK
	and	x0, x0, x1
	ldr	x1, =TCR_EL2_FLAGS
	orr	x0, x0, x1
	msr	tcr_el2, x0

	ldr	x0, =VTCR_EL2_FLAGS
	msr	vtcr_el2, x0

	mrs	x0, mair_el1
	msr	mair_el2, x0
	isb

	mov	x0, #SCTLR_EL2_FLAGS
	msr	sctlr_el2, x0

	eret

	.popsection
