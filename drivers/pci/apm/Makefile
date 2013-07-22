#
# Makefile for the linux kernel.
#
apm-msi-obj-$(CONFIG_PCI_MSI)		:= apm88xxxx_msi.o
obj-$(CONFIG_APM88XXXX_PCI)		+= apm88xxxx_pcie.o apm88xxxx_pcie_common.o apm88xxxx_pcie_serdes.o apm88xxxx_serdes.o $(apm-msi-obj-y)

