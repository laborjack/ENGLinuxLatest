/*
 * Copyright (C) 2012 - ARM Ltd
 * Author: Marc Zyngier <marc.zyngier@arm.com>
 *
 * This program is free software; you can redistribute it and/or modify
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

#include <linux/smp.h>
#include <linux/kvm_host.h>
#include <linux/wait.h>

#include <asm/cputype.h>
#include <asm/kvm_emulate.h>
#include <asm/kvm_psci.h>

/*
 * This is an implementation of the Power State Coordination Interface
 * as described in ARM document number ARM DEN 0022A.
 */

struct psci_suspend_info {
	struct kvm_vcpu *vcpu;
	unsigned long saved_entry;
	unsigned long saved_context_id;
};

static void psci_do_suspend(void *context)
{
	struct psci_suspend_info *sinfo = context;

	sinfo->vcpu->arch.pause = true;
	sinfo->vcpu->arch.suspend = true;
	sinfo->vcpu->arch.suspend_entry = sinfo->saved_entry;
	sinfo->vcpu->arch.suspend_context_id = sinfo->saved_context_id;
}

static unsigned long kvm_psci_vcpu_suspend(struct kvm_vcpu *vcpu)
{
	int i;
	unsigned long mpidr;
	unsigned long target_affinity;
	unsigned long target_affinity_mask;
	unsigned long lowest_affinity_level;
	struct kvm *kvm = vcpu->kvm;
	struct kvm_vcpu *tmp;
	struct psci_suspend_info sinfo;

	target_affinity = kvm_vcpu_get_mpidr(vcpu);
	lowest_affinity_level = (*vcpu_reg(vcpu, 1) >> 24) & 0x3;

	/* Determine target affinity mask */
	target_affinity_mask = MPIDR_HWID_BITMASK;
	switch (lowest_affinity_level) {
	case 0: /* All affinity levels are valid */
		target_affinity_mask &= ~0x0UL;
		break;
	case 1: /* Aff0 ignored */
		target_affinity_mask &= ~0xFFUL;
		break;
	case 2: /* Aff0 and Aff1 ignored */
		target_affinity_mask &= ~0xFFFFUL;
		break;
	case 3: /* Aff0, Aff1, and Aff2 ignored */
		target_affinity_mask &= ~0xFFFFFFUL;
		break;
	default:
		return KVM_PSCI_RET_INVAL;
	};

	/* Ignore other bits of target affinity */
	target_affinity &= target_affinity_mask;

	/* Prepare suspend info */
	sinfo.vcpu = NULL;
	sinfo.saved_entry = *vcpu_reg(vcpu, 2);
	sinfo.saved_context_id = *vcpu_reg(vcpu, 3);

	/* Suspend all VCPUs within target affinity */
	kvm_for_each_vcpu(i, tmp, kvm) {
		mpidr = kvm_vcpu_get_mpidr(tmp);
		if (((mpidr & target_affinity_mask) == target_affinity) &&
		    !tmp->arch.suspend) {
			sinfo.vcpu = tmp;
			smp_call_function_single(tmp->cpu,
						 psci_do_suspend, &sinfo, 1);
		}
	}

	return KVM_PSCI_RET_SUCCESS;
}

static void kvm_psci_vcpu_off(struct kvm_vcpu *vcpu)
{
	vcpu->arch.pause = true;
	vcpu->arch.suspend = false;
}

static unsigned long kvm_psci_vcpu_on(struct kvm_vcpu *source_vcpu,
				      int psci_version)
{
	struct kvm *kvm = source_vcpu->kvm;
	struct kvm_vcpu *vcpu = NULL, *tmp;
	wait_queue_head_t *wq;
	unsigned long cpu_id;
	unsigned long context_id;
	unsigned long mpidr;
	phys_addr_t target_pc;
	int i;

	cpu_id = *vcpu_reg(source_vcpu, 1);
	if (vcpu_mode_is_32bit(source_vcpu))
		cpu_id &= ~((u32) 0);

	kvm_for_each_vcpu(i, tmp, kvm) {
		mpidr = kvm_vcpu_get_mpidr(tmp);
		if ((mpidr & MPIDR_HWID_BITMASK) == (cpu_id & MPIDR_HWID_BITMASK)) {
			vcpu = tmp;
			break;
		}
	}

	/*
	 * Make sure the caller requested a valid CPU and that the CPU is
	 * turned off.
	 */
	if (!vcpu || !vcpu->arch.pause)
		return KVM_PSCI_RET_INVAL;

	target_pc = *vcpu_reg(source_vcpu, 2);
	context_id = *vcpu_reg(source_vcpu, 3);

	kvm_reset_vcpu(vcpu);

	/* Gracefully handle Thumb2 entry point */
	if (vcpu_mode_is_32bit(vcpu) && (target_pc & 1)) {
		target_pc &= ~((phys_addr_t) 1);
		vcpu_set_thumb(vcpu);
	}

	/* Propagate caller endianness */
	if (kvm_vcpu_is_be(source_vcpu))
		kvm_vcpu_set_be(vcpu);

	*vcpu_pc(vcpu) = target_pc;
	if (psci_version != KVM_ARM_PSCI_0_1)
		*vcpu_reg(vcpu, 0) = context_id;
	vcpu->arch.pause = false;
	smp_mb();		/* Make sure the above is visible */

	wq = kvm_arch_vcpu_wq(vcpu);
	wake_up_interruptible(wq);

	return KVM_PSCI_RET_SUCCESS;
}

static unsigned long kvm_psci_vcpu_affinity_info(struct kvm_vcpu *vcpu)
{
	int i;
	unsigned long mpidr;
	unsigned long target_affinity;
	unsigned long target_affinity_mask;
	unsigned long lowest_affinity_level;
	struct kvm *kvm = vcpu->kvm;
	struct kvm_vcpu *tmp;

	target_affinity = *vcpu_reg(vcpu, 1);
	lowest_affinity_level = *vcpu_reg(vcpu, 2);

	/* Determine target affinity mask */
	target_affinity_mask = MPIDR_HWID_BITMASK;
	switch (lowest_affinity_level) {
	case 0: /* All affinity levels are valid */
		target_affinity_mask &= ~0x0UL;
		break;
	case 1: /* Aff0 ignored */
		target_affinity_mask &= ~0xFFUL;
		break;
	case 2: /* Aff0 and Aff1 ignored */
		target_affinity_mask &= ~0xFFFFUL;
		break;
	case 3: /* Aff0, Aff1, and Aff2 ignored */
		target_affinity_mask &= ~0xFFFFFFUL;
		break;
	default:
		return KVM_PSCI_RET_INVAL;
	};

	/* Ignore other bits of target affinity */
	target_affinity &= target_affinity_mask;

	/*
	 * If one or more VCPU matching target affinity are running
	 * then return 0 (ON) else return 1 (OFF)
	 */
	kvm_for_each_vcpu(i, tmp, kvm) {
		mpidr = kvm_vcpu_get_mpidr(tmp);
		if (((mpidr & target_affinity_mask) == target_affinity) &&
		    !tmp->arch.pause) {
			return 0;
		}
	}

	return 1;
}

static inline void kvm_prepare_system_event(struct kvm_vcpu *vcpu, u32 type)
{
	memset(&vcpu->run->system_event, 0, sizeof(vcpu->run->system_event));
	vcpu->run->system_event.type = type;
	vcpu->run->exit_reason = KVM_EXIT_SYSTEM_EVENT;
}

static void kvm_psci_system_off(struct kvm_vcpu *vcpu)
{
	kvm_prepare_system_event(vcpu, KVM_SYSTEM_EVENT_SHUTDOWN);
}

static void kvm_psci_system_reset(struct kvm_vcpu *vcpu)
{
	kvm_prepare_system_event(vcpu, KVM_SYSTEM_EVENT_RESET);
}

int kvm_psci_version(struct kvm_vcpu *vcpu)
{
	if (test_bit(KVM_ARM_VCPU_PSCI_0_2, vcpu->arch.features))
		return KVM_ARM_PSCI_0_2;

	return KVM_ARM_PSCI_0_1;
}

static int kvm_psci_0_2_call(struct kvm_vcpu *vcpu)
{
	int ret = 1;
	unsigned long psci_fn = *vcpu_reg(vcpu, 0) & ~((u32) 0);
	unsigned long val;

	switch (psci_fn) {
	case KVM_PSCI_0_2_FN_PSCI_VERSION:
		/*
		 * Bits[31:16] = Major Version = 0
		 * Bits[15:0] = Minor Version = 2
		 */
		val = 2;
		break;
	case KVM_PSCI_0_2_FN_CPU_SUSPEND:
	case KVM_PSCI_0_2_FN64_CPU_SUSPEND:
		val = kvm_psci_vcpu_suspend(vcpu);
		break;
	case KVM_PSCI_0_2_FN_CPU_OFF:
		kvm_psci_vcpu_off(vcpu);
		val = KVM_PSCI_RET_SUCCESS;
		break;
	case KVM_PSCI_0_2_FN_CPU_ON:
	case KVM_PSCI_0_2_FN64_CPU_ON:
		val = kvm_psci_vcpu_on(vcpu, KVM_ARM_PSCI_0_2);
		break;
	case KVM_PSCI_0_2_FN_AFFINITY_INFO:
	case KVM_PSCI_0_2_FN64_AFFINITY_INFO:
		val = kvm_psci_vcpu_affinity_info(vcpu);
		break;
	case KVM_PSCI_0_2_FN_MIGRATE:
	case KVM_PSCI_0_2_FN64_MIGRATE:
		val = KVM_PSCI_RET_NI;
		break;
	case KVM_PSCI_0_2_FN_MIGRATE_INFO_TYPE:
		/*
		 * Trusted OS is either not present or
		 * does not require migration
		 */
		val = 2;
		break;
	case KVM_PSCI_0_2_FN_MIGRATE_INFO_UP_CPU:
	case KVM_PSCI_0_2_FN64_MIGRATE_INFO_UP_CPU:
		val = KVM_PSCI_RET_NI;
		break;
	case KVM_PSCI_0_2_FN_SYSTEM_OFF:
		kvm_psci_system_off(vcpu);
		val = KVM_PSCI_RET_SUCCESS;
		ret = 0;
		break;
	case KVM_PSCI_0_2_FN_SYSTEM_RESET:
		kvm_psci_system_reset(vcpu);
		val = KVM_PSCI_RET_SUCCESS;
		ret = 0;
		break;
	default:
		return -EINVAL;
	}

	*vcpu_reg(vcpu, 0) = val;
	return ret;
}

static int kvm_psci_0_1_call(struct kvm_vcpu *vcpu)
{
	unsigned long psci_fn = *vcpu_reg(vcpu, 0) & ~((u32) 0);
	unsigned long val;

	switch (psci_fn) {
	case KVM_PSCI_FN_CPU_OFF:
		kvm_psci_vcpu_off(vcpu);
		val = KVM_PSCI_RET_SUCCESS;
		break;
	case KVM_PSCI_FN_CPU_ON:
		val = kvm_psci_vcpu_on(vcpu, KVM_ARM_PSCI_0_1);
		break;
	case KVM_PSCI_FN_CPU_SUSPEND:
	case KVM_PSCI_FN_MIGRATE:
		val = KVM_PSCI_RET_NI;
		break;
	default:
		return -EINVAL;
	}

	*vcpu_reg(vcpu, 0) = val;
	return 1;
}

void kvm_psci_reset(struct kvm_vcpu *vcpu)
{
	vcpu->arch.suspend = false;
	vcpu->arch.suspend_entry = 0;
	vcpu->arch.suspend_context_id = 0;
}

/**
 * kvm_psci_call - handle PSCI call if r0 value is in range
 * @vcpu: Pointer to the VCPU struct
 *
 * Handle PSCI calls from guests through traps from HVC instructions.
 * The calling convention is similar to SMC calls to the secure world
 * where the function number is placed in r0.
 *
 * This function returns: > 0 (success), 0 (success but exit to user
 * space), and < 0 (errors)
 *
 * Errors:
 * -EINVAL: Unrecognized PSCI function
 */
int kvm_psci_call(struct kvm_vcpu *vcpu)
{
	switch (kvm_psci_version(vcpu)) {
	case KVM_ARM_PSCI_0_2:
		return kvm_psci_0_2_call(vcpu);
	case KVM_ARM_PSCI_0_1:
		return kvm_psci_0_1_call(vcpu);
	default:
		return -EINVAL;
	};
}
