/*
 * Copyright (C) 2012 - ARM Ltd
 * Author: Marc Zyngier <marc.zyngier@arm.com>
 *
 * Based on arch/arm/kvm/coproc_a15.c:
 * Copyright (C) 2012 - Virtual Open Systems and Columbia University
 * Authors: Rusty Russell <rusty@rustcorp.au>
 *          Christoffer Dall <c.dall@virtualopensystems.com>
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
#include <linux/kvm_host.h>
#include <asm/cputype.h>
#include <asm/kvm_arm.h>
#include <asm/kvm_asm.h>
#include <asm/kvm_host.h>
#include <asm/kvm_emulate.h>
#include <asm/kvm_coproc.h>
#include <linux/init.h>

#include "sys_regs.h"

#define MPIDR_EL1_AFF0_MASK	0xff

static void reset_mpidr(struct kvm_vcpu *vcpu, const struct sys_reg_desc *r)
{
	/*
	 * Simply map the vcpu_id into the Aff0 field of the MPIDR.
	 */
	vcpu->arch.sys_regs[MPIDR_EL1] = (1 << 31) | (vcpu->vcpu_id & MPIDR_EL1_AFF0_MASK);
}

static bool access_actlr(struct kvm_vcpu *vcpu,
			 const struct sys_reg_params *p,
			 const struct sys_reg_desc *r)
{
	if (p->is_write)
		return ignore_write(vcpu, p);

	*vcpu_reg(vcpu, p->Rt) = vcpu->arch.sys_regs[ACTLR_EL1];
	return true;
}

static void reset_actlr(struct kvm_vcpu *vcpu, const struct sys_reg_desc *r)
{
	u64 actlr;

	asm volatile("mrs %0, actlr_el1\n" : "=r" (actlr));
	vcpu->arch.sys_regs[ACTLR_EL1] = actlr;
}

/*
 * A57-specific sys-reg registers.
 * Important: Must be sorted ascending by Op0, Op1, CRn, CRm, Op2
 */
static const struct sys_reg_desc a57_sys_regs[] = {
	{ Op0(0b11), Op1(0b000), CRn(0b0000), CRm(0b0000), Op2(0b101), /* MPIDR_EL1 */
	  NULL, reset_mpidr, MPIDR_EL1 },
	{ Op0(0b11), Op1(0b000), CRn(0b0001), CRm(0b0000), Op2(0b000), /* SCTLR_EL1 */
	  NULL, reset_val, SCTLR_EL1, 0x00C50078 },
	{ Op0(0b11), Op1(0b000), CRn(0b0001), CRm(0b0000), Op2(0b001), /* ACTLR_EL1 */
	  access_actlr, reset_actlr, ACTLR_EL1 },
	{ Op0(0b11), Op1(0b000), CRn(0b0001), CRm(0b0000), Op2(0b010), /* CPACR_EL1 */
	  NULL, reset_val, CPACR_EL1, 0 },
};

static struct kvm_sys_reg_target_table a57_target_table = {
	.target = KVM_ARM_TARGET_CORTEX_A57,
	.table64 = {
		.table = a57_sys_regs,
		.num = ARRAY_SIZE(a57_sys_regs),
	},
};

static int __init sys_reg_a57_init(void)
{
	unsigned int i;

	for (i = 1; i < ARRAY_SIZE(a57_sys_regs); i++)
		BUG_ON(cmp_sys_reg(&a57_sys_regs[i-1],
			       &a57_sys_regs[i]) >= 0);

	kvm_register_target_sys_reg_table(&a57_target_table);
	return 0;
}
late_initcall(sys_reg_a57_init);
