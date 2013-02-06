/*
 * Fault injection for both 32 and 64bit guests.
 *
 * Copyright (C) 2012 - ARM Ltd
 * Author: Marc Zyngier <marc.zyngier@arm.com>
 *
 * Based on arch/arm/kvm/emulate.c
 * Copyright (C) 2012 - Virtual Open Systems and Columbia University
 * Author: Christoffer Dall <c.dall@virtualopensystems.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
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

#include <linux/kvm_host.h>
#include <asm/kvm_emulate.h>

static void prepare_fault32(struct kvm_vcpu *vcpu, u32 mode, u32 vect_offset)
{
	unsigned long cpsr;
	unsigned long new_spsr_value = *vcpu_cpsr(vcpu);
	bool is_thumb = (new_spsr_value & COMPAT_PSR_T_BIT);
	u32 return_offset = (is_thumb) ? 4 : 0;
	u32 sctlr = vcpu->arch.cp15[c1_SCTLR];

	cpsr = mode | COMPAT_PSR_I_BIT;

	if (sctlr & (1 << 30))
		cpsr |= COMPAT_PSR_T_BIT;
	if (sctlr & (1 << 25))
		cpsr |= COMPAT_PSR_E_BIT;

	*vcpu_cpsr(vcpu) = cpsr;

	/* Note: These now point to the banked copies */
	*vcpu_spsr(vcpu) = new_spsr_value;
	*vcpu_reg(vcpu, 14) = *vcpu_pc(vcpu) + return_offset;

	/* Branch to exception vector */
	if (sctlr & (1 << 13))
		vect_offset += 0xffff0000;
	else /* always have security exceptions */
		vect_offset += vcpu->arch.cp15[c12_VBAR];

	*vcpu_pc(vcpu) = vect_offset;
}

static void inject_undef32(struct kvm_vcpu *vcpu)
{
	prepare_fault32(vcpu, COMPAT_PSR_MODE_UND, 4);
}

/*
 * Modelled after TakeDataAbortException() and TakePrefetchAbortException
 * pseudocode.
 */
static void inject_abt32(struct kvm_vcpu *vcpu, bool is_pabt,
			 unsigned long addr)
{
	u32 vect_offset;
	u32 *far, *fsr;
	bool is_lpae;

	if (is_pabt) {
		vect_offset = 12;
		far = &vcpu->arch.cp15[c6_IFAR];
		fsr = &vcpu->arch.cp15[c5_IFSR];
	} else { /* !iabt */
		vect_offset = 16;
		far = &vcpu->arch.cp15[c6_DFAR];
		fsr = &vcpu->arch.cp15[c5_DFSR];
	}

	prepare_fault32(vcpu, COMPAT_PSR_MODE_ABT | COMPAT_PSR_A_BIT, vect_offset);

	*far = addr;

	/* Always give debug fault for now - should give guest a clue */
	is_lpae = (vcpu->arch.cp15[c2_TTBCR] >> 31);
	if (is_lpae)
		*fsr = 1 << 9 | 0x22;
	else
		*fsr = 2;
}

static void inject_abt64(struct kvm_vcpu *vcpu, bool is_iabt, unsigned long addr)
{
	unsigned long cpsr = *vcpu_cpsr(vcpu);
	int is_aarch32;
	u32 esr = 0;

	is_aarch32 = vcpu_mode_is_32bit(vcpu);

	*vcpu_spsr(vcpu) = cpsr;
	vcpu->arch.regs.elr_el1 = *vcpu_pc(vcpu);

	*vcpu_cpsr(vcpu) = PSR_MODE_EL1h | PSR_A_BIT | PSR_F_BIT | PSR_I_BIT;
	*vcpu_pc(vcpu) = vcpu->arch.sys_regs[VBAR_EL1] + 0x200;

	vcpu->arch.sys_regs[FAR_EL1] = addr;

	/*
	 * Build an {i,d}abort, depending on the level and the
	 * instruction set. Report an external synchronous abort.
	 */
	if (kvm_vcpu_trap_il_is32bit(vcpu))
		esr |= (1 << 25);

	if (is_aarch32 || (cpsr & PSR_MODE_MASK) == PSR_MODE_EL0t)
		esr |= (0x20 << 26);
	else
		esr |= (0x21 << 26);

	if (!is_iabt)
		esr |= (1 << 28);

	vcpu->arch.sys_regs[ESR_EL1] = esr | 0x10;
}

static void inject_undef64(struct kvm_vcpu *vcpu)
{
	unsigned long cpsr = *vcpu_cpsr(vcpu);
	u32 esr = 0;

	*vcpu_spsr(vcpu) = cpsr;
	vcpu->arch.regs.elr_el1 = *vcpu_pc(vcpu);

	*vcpu_cpsr(vcpu) = PSR_MODE_EL1h | PSR_F_BIT | PSR_I_BIT;
	*vcpu_pc(vcpu) = vcpu->arch.sys_regs[VBAR_EL1] + 0x200;

	/*
	 * Build an unknown exception, depending on the instruction
	 * set.
	 */
	if (kvm_vcpu_trap_il_is32bit(vcpu))
		esr |= (1 << 25);

	vcpu->arch.sys_regs[ESR_EL1] = esr;
}

/**
 * kvm_inject_dabt - inject a data abort into the guest
 * @vcpu: The VCPU to receive the undefined exception
 * @addr: The address to report in the DFAR
 *
 * It is assumed that this code is called from the VCPU thread and that the
 * VCPU therefore is not currently executing guest code.
 */
void kvm_inject_dabt(struct kvm_vcpu *vcpu, unsigned long addr)
{
	if (!(vcpu->arch.hcr_el2 & HCR_RW))
		inject_abt32(vcpu, false, addr);

	inject_abt64(vcpu, false, addr);
}

/**
 * kvm_inject_pabt - inject a prefetch abort into the guest
 * @vcpu: The VCPU to receive the undefined exception
 * @addr: The address to report in the DFAR
 *
 * It is assumed that this code is called from the VCPU thread and that the
 * VCPU therefore is not currently executing guest code.
 */
void kvm_inject_pabt(struct kvm_vcpu *vcpu, unsigned long addr)
{
	if (!(vcpu->arch.hcr_el2 & HCR_RW))
		inject_abt32(vcpu, true, addr);

	inject_abt64(vcpu, true, addr);
}

/**
 * kvm_inject_undefined - inject a undefined instruction into the guest
 *
 * It is assumed that this code is called from the VCPU thread and that the
 * VCPU therefore is not currently executing guest code.
 */
void kvm_inject_undefined(struct kvm_vcpu *vcpu)
{
	if (!(vcpu->arch.hcr_el2 & HCR_RW))
		inject_undef32(vcpu);

	inject_undef64(vcpu);
}
