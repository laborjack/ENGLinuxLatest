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

#include <linux/kvm_host.h>
#include <linux/wait.h>

#include <asm/kvm_emulate.h>
#include <asm/kvm_psci.h>

/*
 * This is an implementation of the Power State Coordination Interface
 * as described in ARM document number ARM DEN 0022A.
 */

static void kvm_psci_vcpu_off(struct kvm_vcpu *vcpu)
{
	vcpu->arch.pause = true;
}

static unsigned long kvm_psci_vcpu_on(struct kvm_vcpu *source_vcpu)
{
	struct kvm *kvm = source_vcpu->kvm;
	struct kvm_vcpu *vcpu;
	wait_queue_head_t *wq;
	unsigned long cpu_id;
	phys_addr_t target_pc;

	cpu_id = *vcpu_reg(source_vcpu, 1);
	if (vcpu_mode_is_32bit(source_vcpu))
		cpu_id &= ~((u32) 0);

	if (cpu_id >= atomic_read(&kvm->online_vcpus))
		return KVM_PSCI_RET_INVAL;

	target_pc = *vcpu_reg(source_vcpu, 2);

	vcpu = kvm_get_vcpu(kvm, cpu_id);

	wq = kvm_arch_vcpu_wq(vcpu);
	if (!waitqueue_active(wq))
		return KVM_PSCI_RET_INVAL;

	kvm_reset_vcpu(vcpu);

	/* Gracefully handle Thumb2 entry point */
	if (vcpu_mode_is_32bit(vcpu) && (target_pc & 1)) {
		target_pc &= ~((phys_addr_t) 1);
		vcpu_set_thumb(vcpu);
	}

	*vcpu_pc(vcpu) = target_pc;
	vcpu->arch.pause = false;
	smp_mb();		/* Make sure the above is visible */

	wake_up_interruptible(wq);

	return KVM_PSCI_RET_SUCCESS;
}

static void kvm_psci_system_off(struct kvm_vcpu *vcpu, struct kvm_run *run)
{
	struct kvm_exit_psci psci;

	psci.fn = KVM_PSCI_FN_SYSTEM_OFF;
	memset(&psci.args, 0, sizeof(psci.args));
	kvm_prepare_psci(run, &psci);
}

static void kvm_psci_system_reset(struct kvm_vcpu *vcpu, struct kvm_run *run)
{
	struct kvm_exit_psci psci;

	psci.fn = KVM_PSCI_FN_SYSTEM_RESET;
	memset(&psci.args, 0, sizeof(psci.args));
	kvm_prepare_psci(run, &psci);
}

/**
 * kvm_handle_psci_return -- Handle PSCI after user space emulation
 * @vcpu: The VCPU pointer
 * @run:  The VCPU run struct containing the psci data
 *
 * This should only be called after returning from userspace for
 * PSCI emulation.
 */
int kvm_handle_psci_return(struct kvm_vcpu *vcpu, struct kvm_run *run)
{
	/*
	 * Currently, the PSCI functions passed to user space for emulation
	 * are SYSTEM_OFF and SYSTEM_RESET. These PSCI functions are not
	 * expected to return back after emulating in user space hence by
	 * default we return -EINVAL to avoid user space from doing RUN ioctl
	 * after handling KVM_EXIT_PSCI.
	 */

	return -EINVAL;
}

/**
 * kvm_psci_call - handle PSCI call if r0 value is in range
 * @vcpu: Pointer to the VCPU struct
 *
 * Handle PSCI calls from guests through traps from HVC instructions.
 * The calling convention is similar to SMC calls to the secure world where
 * the function number is placed in r0 and this function returns true if the
 * function number specified in r0 is withing the PSCI range, and false
 * otherwise.
 */
int kvm_psci_call(struct kvm_vcpu *vcpu, struct kvm_run *run)
{
	int ret = 0;
	unsigned long psci_fn = *vcpu_reg(vcpu, 0) & ~((u32) 0);
	unsigned long val;

	switch (psci_fn) {
	case KVM_PSCI_FN_CPU_OFF:
		kvm_psci_vcpu_off(vcpu);
		val = KVM_PSCI_RET_SUCCESS;
		break;
	case KVM_PSCI_FN_CPU_ON:
		val = kvm_psci_vcpu_on(vcpu);
		break;
	case KVM_PSCI_FN_CPU_SUSPEND:
	case KVM_PSCI_FN_MIGRATE:
		val = KVM_PSCI_RET_NI;
		break;
	case KVM_PSCI_FN_SYSTEM_OFF:
		kvm_psci_system_off(vcpu, run);
		val = KVM_PSCI_RET_SUCCESS;
		ret = -EINTR;
		break;
	case KVM_PSCI_FN_SYSTEM_RESET:
		kvm_psci_system_reset(vcpu, run);
		val = KVM_PSCI_RET_SUCCESS;
		ret = -EINTR;
		break;
	default:
		return -EINVAL;
	}

	*vcpu_reg(vcpu, 0) = val;
	return ret;
}
