/*
 * Copyright (C) 2012 ARM Ltd.
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

#ifndef __ARM64_KVM_ARCH_TIMER_H
#define __ARM64_KVM_ARCH_TIMER_H

#include <linux/clocksource.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>

struct arch_timer_kvm {
	/* Is the timer enabled */
	bool			enabled;

	/* Virtual offset, restored only */
	cycle_t			cntvoff;
};

struct arch_timer_cpu {
	/* Background timer used when the guest is not running */
	struct hrtimer			timer;

	/* Work queued with the above timer expires */
	struct work_struct		expired;

	/* Background timer active */
	bool				armed;

	/* Timer IRQ */
	const struct kvm_irq_level	*irq;

	/* Registers: control register, timer value */
	u32				cntv_ctl;	/* Saved/restored */
	cycle_t				cntv_cval;	/* Saved/restored */
};

int kvm_timer_hyp_init(void);
int kvm_timer_init(struct kvm *kvm);
void kvm_timer_vcpu_reset(struct kvm_vcpu *vcpu, 
			  const struct kvm_irq_level *irq);
void kvm_timer_vcpu_init(struct kvm_vcpu *vcpu);
void kvm_timer_flush_hwstate(struct kvm_vcpu *vcpu);
void kvm_timer_sync_hwstate(struct kvm_vcpu *vcpu);
void kvm_timer_vcpu_terminate(struct kvm_vcpu *vcpu);

#endif
