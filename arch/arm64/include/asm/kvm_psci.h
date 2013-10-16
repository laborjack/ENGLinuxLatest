/*
 * Copyright (C) 2012,2013 - ARM Ltd
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

#ifndef __ARM64_KVM_PSCI_H__
#define __ARM64_KVM_PSCI_H__

#include <linux/kvm_host.h>
#include <asm/kvm_asm.h>
#include <asm/kvm_arm.h>

/*
 * The in-kernel PSCI emulation code wants to use a copy of run->psci,
 * which is an anonymous type. Use our own type instead.
 */
struct kvm_exit_psci {
	u32		fn;
	u64		args[7];
};

static inline void kvm_prepare_psci(struct kvm_run *run,
				    struct kvm_exit_psci *psci)
{
	run->psci.fn = psci->fn;
	memcpy(&run->psci.args, &psci->args, sizeof(run->psci.args));
	memset(&run->psci.ret, 0, sizeof(run->psci.ret));
	run->exit_reason = KVM_EXIT_PSCI;
}

int kvm_handle_psci_return(struct kvm_vcpu *vcpu, struct kvm_run *run);
int kvm_psci_call(struct kvm_vcpu *vcpu, struct kvm_run *run);

#endif /* __ARM64_KVM_PSCI_H__ */
