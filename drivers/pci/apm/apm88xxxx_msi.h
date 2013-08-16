/*
 * APM MSI Driver Header file
 *
 * Copyright (c) 2010, Applied Micro Circuits Corporation
 * Author: Tanmay Inamdar <tinamdar@apm.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 */

#ifndef _APM_MSI_H
#define _APM_MSI_H

#include <linux/of.h>
#include <asm/msi_bitmap.h>

#ifdef APM_MSI_DEBUG
# define msi_dbg(fmt, ...)	printk(KERN_DEBUG "%s: "fmt, __func__,##__VA_ARGS__); 
#else
# define msi_dbg(x, ...)
#endif

#define NR_MSI_REG		16
#define IRQS_PER_MSI_REG	256
#define NR_MSI_IRQS		(NR_MSI_REG * IRQS_PER_MSI_REG)

#define APM_PIC_IP_MASK		0x0000000F
#define APM_PIC_IP_GIC		0x00000001

/* PCIe MSI Index Registers */
#define MSI0IR0		0x000000
#define MSIFIR7		0x7F0000

/* PCIe MSI Interrupt Register */
#define MSI1INT0	0x800000
#define MSI1INTF	0x8F0000

struct apm_msi {
	struct irq_domain *irqhost;
	unsigned long cascade_irq;
	u32 msiir_offset;
	u32 msi_addr_lo;
	u32 msi_addr_hi;
	void __iomem *msi_regs;
	u32 feature;
	int msi_virqs[NR_MSI_REG];
	struct msi_bitmap bitmap;
	struct list_head list;		/* support multiple MSI banks */
	phandle phandle;
};

#endif /* _APM_MSI_H */

