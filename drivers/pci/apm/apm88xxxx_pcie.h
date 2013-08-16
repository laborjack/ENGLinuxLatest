/**
 * APM APM86xxx PCIe Header File
 *
 * Copyright (c) 2010 Applied Micro Circuits Corporation.
 * All rights reserved. Loc Ho <lho@apm.com>, Feng Kan <fkan@apm.com>.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * This module defines the CSR register, header types, and function
 * for PCIE module under Linux. It supports setup and configure the
 * PCIE controllers to work as root complex or endpoint.
 *
 */
#ifndef __APM_PCIE_H__
#define __APM_PCIE_H__

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/bootmem.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/slab.h>
#include <linux/clk.h>

#include <asm/sizes.h>
#include <asm/pci.h>
#include <asm/io.h>
#include "apm88xxxx_serdes.h"
#include "apm88xxxx_pcie_csr.h"

#if 0
#define  APM_PCIE_DEBUG
#define  APM_PCIE_VDEBUG
#define  APM_PCIE_CDEBUG
#endif
#define CONFIG_RESOURCES_64BIT

#ifdef APM_PCIE_CDEBUG
# define PCIE_CSR_DEBUG(fmt, ...)	printk(KERN_INFO fmt, ##__VA_ARGS__); 
#else
# define PCIE_CSR_DEBUG(fmt, ...)
#endif

#ifdef APM_PCIE_VDEBUG
# define PCIE_VDEBUG(fmt, ...)		printk(KERN_INFO fmt, ##__VA_ARGS__);
#else
# define PCIE_VDEBUG(fmt, ...)
#endif

#ifdef APM_PCIE_DEBUG
# define PCIE_DEBUG(fmt, ...)		printk(KERN_INFO fmt, ##__VA_ARGS__); 
#else
# define PCIE_DEBUG(x, ...)
#endif
#define PCIE_WARN(fmt, ...)		printk(KERN_WARNING fmt, ##__VA_ARGS__);
#define PCIE_ERR(fmt, ...)		printk(KERN_ERR fmt, ##__VA_ARGS__);
#define PCIE_INFO(fmt, ...)		printk(KERN_INFO fmt, ##__VA_ARGS__);

#define APM_PCIE_MAX_PORTS	5

#define APM_EP_REG_SPACE_SIZE	0x100000
#define APM_EP_MEM_SPACE_SIZE	0x100000

struct apm_pcie_ep_info {
	void		*reg_space;	/* maps to outbound space of RC */
	dma_addr_t	reg_addr_phys;	/* Physical address of reg space */
	void		*mem_space;	/* own system memory of EP */
	dma_addr_t	mem_addr_phys;	/* Physical address of mem space */
};

struct apm_pcie_port {
	struct device_node	*node;
#ifdef CONFIG_PM
	u32			pwred_off_flags;
#endif
	u64			ob_pci_mem_offset[3];
	u64			ib_pci_mem_offset[3];
	struct resource		res[3];
	u8			dma;
	u8			index;
	u8			type;
	u8			link_up;
	u8			link_width;
	u8			link_speed;
	u8			serdes_clk;
	u32			first_busno;
	u32			last_busno;
	struct apm_pcie_map_tbl *pcie_map_tbl;
	void			*csr_base;
	void			*cfg_base;
	struct clk		*clk;
	struct device		*dev;
	struct apm_pcie_ep_info	ep_info;
};

struct apm_pcie_info {
	struct apm_pcie_port	port[APM_PCIE_MAX_PORTS];
	int			nr_ports;
};

#endif
