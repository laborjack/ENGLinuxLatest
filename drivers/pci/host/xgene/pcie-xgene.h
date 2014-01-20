/**
 * APM X-Gene PCIe Header File
 *
 * Copyright (c) 2013 Applied Micro Circuits Corporation.
 *
 * Author: Tanmay Inamdar <tinamdar@apm.com>.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __XGENE_PCIE_H__
#define __XGENE_PCIE_H__

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/memblock.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/gpio.h>
#include <asm/pcibios.h>
#include "pcie-xgene-core.h"

#define MAX_BOARD_GPIOS		32
#define GPIO_INDEX(index)	(ARCH_NR_GPIOS - MAX_BOARD_GPIOS + index)
#define XGENE_PCIE_MAX_PORTS	5
#define XGENE_PCIE_EP_MEM_SIZE	0x100000

enum {
	XGENE_MEM,
	XGENE_MSI,
	XGENE_IO,
	XGENE_RES	/* termination */
};

struct xgene_pcie_ep_info {
	void		*reg_virt;	/* maps to outbound space of RC */
	dma_addr_t	reg_phys;	/* Physical address of reg space */
};

struct xgene_pcie_port {
	struct device_node		*node;
	struct resource			res[XGENE_RES];
	u8				index;
	u8				type;
	u8				link_up;
	u8				link_width;
	u8				link_speed;
	u8				cur_link_width;
	u8				cur_link_speed;
	u8				serdes_clk;
	u32				first_busno;
	u32				last_busno;
	u32				reset_gpio;
	struct xgene_pcie_map_tbl	*map_tbl;
	void				*csr_base;
	void				*cfg_base;
	struct device			*dev;
	struct xgene_pcie_ep_info	ep_info;
};

struct xgene_pcie_info {
	struct xgene_pcie_port	port[XGENE_PCIE_MAX_PORTS];
	int			nr_ports;
};

int xgene_pcie_setup_core(struct xgene_pcie_port *port);

#ifdef CONFIG_ARCH_MSLIM
#include <asm/hardware/mslim-iof-map.h>
static inline u64 xgene_pcie_get_iof_addr(u64 addr)
{
	return mslim_pa_to_iof_axi(lower_32_bits(addr));
}
#define pci_io_offset(s)		(s & 0x00000000)
#else /* X-GENE */
#define xgene_pcie_get_iof_addr(addr)	addr;
#define pci_io_offset(s)		(s & 0xff00000000)
#endif /* CONFIG_ARCH_MSLIM */
#endif /* __XGENE_PCIE_H__ */
