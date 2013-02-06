#
# Makefile for Kernel-based Virtual Machine module
#

ccflags-y += -Ivirt/kvm -Iarch/arm64/kvm
CFLAGS_arm.o := -I.
CFLAGS_mmu.o := -I.

obj-$(CONFIG_KVM_ARM_HOST) += $(addprefix ../../../virt/kvm/, kvm_main.o coalesced_mmio.o)

obj-$(CONFIG_KVM_ARM_HOST) += $(addprefix ../../../arch/arm/kvm/, arm.o mmu.o mmio.o psci.o perf.o)

obj-$(CONFIG_KVM_ARM_HOST) += emulate.o inject_fault.o regmap.o
obj-$(CONFIG_KVM_ARM_HOST) += hyp.o hyp-init.o handle_exit.o idmap.o
obj-$(CONFIG_KVM_ARM_HOST) += guest.o reset.o sys_regs.o sys_regs_a57.o

obj-$(CONFIG_KVM_ARM_VGIC) += $(addprefix ../../../arch/arm/kvm/, vgic.o)
obj-$(CONFIG_KVM_ARM_TIMER) += $(addprefix ../../../arch/arm/kvm/, arch_timer.o)
