/**
 * APM STORM PCIe Header File
 *
 * Copyright (c) 2010 Applied Micro Circuits Corporation.
 * All rights reserved. Tanmay Inamdar <tinamdar@apm.com>.
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
 * This module implements PCIE functionality. This setups and
 * configures PCIE controllers as either root complex or endpoint.
 *
 */

#include <linux/kernel.h>
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
#include <linux/memblock.h>
#include <linux/platform_device.h>
#include <asm/sizes.h>
#include <asm/pci.h>
#include <asm/io.h>

#include "apm88xxxx_pcie_csr.h"
#include "apm88xxxx_pcie.h"
#include "apm88xxxx_pcie_common.h"

#define PCIE_DRIVER_VERSION "0.1"

static int probe_cnt;
static struct apm_pcie_info pcie_ports;
extern int of_irq_map_pci(struct pci_dev *pdev, struct of_irq *out_irq);

static int apm_pcie_validate_bdf(struct pci_bus *bus, unsigned int devfn)
{
	struct pci_sys_data *sys = bus->sysdata;
	struct apm_pcie_port *port = sys->private_data;
	static int message;

	BUG_ON(port == NULL);

	if(!port->link_up)
		return PCIBIOS_DEVICE_NOT_FOUND;
		
	/* Endpoint can not generate upstream(remote) config cycles */
	if (!port->cfg_base && (bus->number != port->first_busno)) {
		printk("%s:%d Error \n",__func__,__LINE__);
		return PCIBIOS_DEVICE_NOT_FOUND;
	}
	/* Check we are within the mapped range */
	if (bus->number > port->last_busno) {
		printk("%s:%d Error \n",__func__,__LINE__);
		if (!message) {
			pr_info("Probing bus %u out of range!\n",
				bus->number);
			message++;
		}
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	/* The root complex has only one device / function */
	if (bus->number == port->first_busno && devfn != 0)
		return PCIBIOS_DEVICE_NOT_FOUND;

	/* The other side of the RC has only one device as well */
	if (bus->number == (port->first_busno + 1) &&
	    PCI_SLOT(devfn) != 0) {
		printk("%s:%d Error \n",__func__,__LINE__);
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	/* Check if we have a link */
	if ((bus->number != port->first_busno) && !port->link_up) {
		printk("%s:%d Error \n",__func__,__LINE__);
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	return 0;
}

static void __iomem *apm_pcie_get_cfg_base(struct pci_bus *bus)
{
	struct pci_sys_data *sys = bus->sysdata;
	struct apm_pcie_port *port = sys->private_data;
	u64 addr;

	if (bus->number >= (port->first_busno + 1))
		addr = (u64)port->cfg_base | 0x10000;
	else
		addr = (u64)port->cfg_base;

	if(port->dma)
		addr |= 0x20000;

	return (void *)addr;
}

static void apm_pcie_set_rtdid_reg(struct pci_bus *bus, uint devfn)
{
	struct pci_sys_data *sys = bus->sysdata;
	struct apm_pcie_port *port = sys->private_data;
	unsigned int b, d, f;
	u32 rtdid_val = 0;

	b = bus->number;
	d = PCI_SLOT(devfn);
	f = PCI_FUNC(devfn);

	if (bus->number == port->first_busno)
		rtdid_val = 0;
	else if (bus->number >= (port->first_busno + 1))
		rtdid_val = (b << 8) | (d << 3) | (f << 0);

	apm_out32(port->csr_base + SM_PCIE_CSR_REGS_RTDID__ADDR, rtdid_val);
}

/**
 * @brief Read from PCIe configuration space
 * @param bus		PCIE bus
 * @param devfn		Device/function number to read from
 * @param offset	Offset CSR
 * @param len		# of byte (1, 2, or 4)
 * @param val		A pointer to location to store value
 * @return PCIBIOS_SUCCESSFUL is returned.
 *
 */
static int pcie_read_config(struct pci_bus *bus, unsigned int devfn,
			    int offset, int len, u32 *val)
{
	void __iomem *addr;
	u8 val8;
	u16 val16;

	if (apm_pcie_validate_bdf(bus, devfn))
		return PCIBIOS_DEVICE_NOT_FOUND;

	apm_pcie_set_rtdid_reg(bus, devfn);
	addr = apm_pcie_get_cfg_base(bus);
	switch (len) {
	case 1:
		pcie_cfg_in8(addr + offset, &val8);
		*val = val8;
		break;
	case 2:
		pcie_cfg_in16(addr + offset, &val16);
		*val = val16;
		break;
	default:
		pcie_cfg_in32(addr + offset, val);
		break;
	}
	return PCIBIOS_SUCCESSFUL;
}

/**
 * @brief Write to PCIe configuration space
 * @param bus		PCIE bus
 * @param devfn		Device/function number to read from
 * @param offset	Offset CSR
 * @param len		# of byte (1, 2, or 4)
 * @param val		A value to write
 * @return PCIBIOS_SUCCESSFUL is returned.
 *
 */
static int pcie_write_config(struct pci_bus *bus, unsigned int devfn,
			     int offset, int len, u32 val)
{
	void __iomem *addr;

	if (apm_pcie_validate_bdf(bus, devfn))
		return PCIBIOS_DEVICE_NOT_FOUND;

	apm_pcie_set_rtdid_reg(bus, devfn);
	addr = apm_pcie_get_cfg_base(bus);
	switch (len) {
	case 1:
		pcie_cfg_out8(addr + offset, (u8)val);
		break;
	case 2:
		pcie_cfg_out16(addr + offset, (u16)val);
		break;
	default:
		pcie_cfg_out32(addr + offset, val);
		break;
	}
	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops apm_pcie_ops = {
	.read  = pcie_read_config,
	.write = pcie_write_config
};


int pcibios_prep_pcie_dma(struct pci_bus *bus)
{
	struct pci_sys_data *sys = bus->sysdata;
	struct apm_pcie_port *port = sys->private_data;

	if(port->dma)
		return -1;
	port->dma = 1;
	return 0;
}
EXPORT_SYMBOL(pcibios_prep_pcie_dma);

void pcibios_cleanup_pcie_dma(struct pci_bus *bus)
{
	struct pci_sys_data *sys = bus->sysdata;
	struct apm_pcie_port *port = sys->private_data;
	port->dma = 0;
}
EXPORT_SYMBOL(pcibios_cleanup_pcie_dma);

struct device_node *pcibios_get_phb_of_node(struct pci_bus *bus)
{
	struct pci_sys_data *sys = bus->sysdata;
	struct apm_pcie_port *port = sys->private_data;

	return of_node_get(port->node);
}

static void fixup_apm_pcie_bridge(struct pci_dev *dev)
{
	struct pci_bus *bus = dev->bus;
	struct pci_sys_data *sys = bus->sysdata;
	struct apm_pcie_port *port = sys->private_data;
	int i;

	BUG_ON(port == NULL);

	if (dev->devfn != 0 || dev->bus->self != NULL)
		return;

	/* Hide the PCI host BARs from the kernel as their content doesn't
	 * fit well in the resource management
	 */
	for (i = 0; i < 3; i++) {
		dev->resource[i].start = dev->resource[i].end = 0;
		dev->resource[i].flags = 0;
	}

	printk(KERN_INFO "PCI: Hiding APM host bridge resources %s\n",
	       pci_name(dev));
}
DECLARE_PCI_FIXUP_HEADER(0x19AA, 0xE004, fixup_apm_pcie_bridge);

void apm_pcie_setup_primary_bus(struct apm_pcie_port *port)
{
        u32 val;
	void *cfg_addr = port->cfg_base;

	if(port->type == PTYPE_ENDPOINT)
		return;
	apm_in32(cfg_addr + PCI_PRIMARY_BUS, &val);
	val &= 0xFF000000;
	val |=  port->last_busno << 16 |
		(port->first_busno + 1) << 8 |
		port->first_busno;
	apm_out32(cfg_addr + PCI_PRIMARY_BUS, val);
}

static void apm_pcie_adjust_ib_region(struct apm_pcie_port *port,
				   struct resource *ib_res)
{
	phys_addr_t ddr_base = memblock_start_of_DRAM();
	phys_addr_t ddr_size = ALIGN(memblock_phys_mem_size(), 0x80000000);

	if (ddr_base != ib_res->start)
		ib_res->start = ddr_base;

	if (ddr_size != RES_SIZE(ib_res))
		ib_res->end = ib_res->start + ddr_size - 1;
}

static u64 parse_memory_ranges( struct resource *res,
				struct apm_pcie_port *port,
				int region, const u32 *ranges,
				int rlen)
{
	struct device_node *dn = port->node;
	int pna = of_n_addr_cells(dn);
	int np = pna + 5;
	u32 pci_space;
	u64 pci_addr = 0;
	u64 cpu_addr = 0;
	u64 size = 0;

	while ((rlen -= np * 4) >= 0) {
		pci_space = ranges[0];
		pci_addr = of_read_number(ranges + 1, 2);
		cpu_addr = of_translate_address(dn, ranges + 3);
		size = of_read_number(ranges + pna + 3, 2);
		ranges += np;
		if (cpu_addr == OF_BAD_ADDR || size == 0)
			continue;

		/* We only care about memory */
		if ((pci_space & 0x00000003) == 0x00000001)
			continue;

		/* Check if not prefetchable */
		if (!(pci_space & 0x00000040))
			res->flags &= ~IORESOURCE_PREFETCH;

		res->start = cpu_addr;
		res->end = res->start + size - 1;

		/* Remove bootloader dependency */
		if (region == 0)
			apm_pcie_adjust_ib_region(port, res);
		size = RES_SIZE(res);
		break;
	}
	port->ib_pci_mem_offset[region] = (res->start & ~(size - 1));
	PCIE_DEBUG("%s 0x%016llx..0x%016llx -> 0x%016llx\n",
		  (region == 0)?"DMA":"MSI", res->start, res->end, pci_addr);
	return size;
}

#ifdef CONFIG_PCI_MSI
static int apm_pcie_parse_map_msi_ranges(struct apm_pcie_port *port)
{
	struct device_node *dn = port->node;
	struct resource res;
	struct apm_pcie_map_tbl *hb = port->pcie_map_tbl;
	const u32 *ranges;
	int rlen;
	u64 size;

	res.start = res.end = 0;
	res.flags = IORESOURCE_MEM | IORESOURCE_PREFETCH | IORESOURCE_MEM_64;

	/* Get msi-ranges property */
	ranges = of_get_property(dn, "msi-ranges", &rlen);
	if (ranges == NULL)
		return -EINVAL;

	size = parse_memory_ranges(&res, port, 1, ranges, rlen);

	/* Lets map the PCIe Interrupt Handler for this PIM, one for one.
	 * Use 2nd inbound memory region to map MSI ranges.
	 */
	hb->ib_mem_addr[1].pcie_hi = HIDWORD(res.start);
	hb->ib_mem_addr[1].pcie_lo = LODWORD(res.start);
	hb->ib_mem_addr[1].pcie_size = RES_SIZE(&res);
	hb->ib_mem_addr[1].pim1_hi = HIDWORD(port->ib_pci_mem_offset[1]);
	hb->ib_mem_addr[1].pim1_lo = LODWORD(port->ib_pci_mem_offset[1]);
	hb->ib_mem_addr[1].pim2_hi = 0;
	hb->ib_mem_addr[1].pim2_lo = 0;
	hb->ib_mem_addr[1].pim_size = PIM_SIZE;

	PCIE_DEBUG("PCIe MSI offset set to 0x%08lx\n",
		   (long unsigned int) res.start);

	return 0;
}
#endif /* CONFIG_MSI */

/**
 * apm_pci_process_bridge_OF_ranges - Parse PCI bridge resources from device tree
 * @port: newly allocated pci_port to be setup
 *
 * This function will parse the "ranges" property of a PCI host bridge device
 * node and setup the resource mapping of a pci controller based on its
 * content.
 *
 */
static void apm_pci_process_bridge_OF_ranges(struct apm_pcie_port *port)
{
	struct device_node *dev = port->node;
	const u32 *ranges;
	int rlen;
	int pna = of_n_addr_cells(dev);
	int np = pna + 5;
	int memno = 0;
	u32 pci_space;
	u64 pci_addr, cpu_addr, pci_next, cpu_next, size;
	struct resource *res;

	PCIE_DEBUG("PCI host bridge %s ranges:\n", dev->full_name);

	/* Get ranges property */
	ranges = of_get_property(dev, "ranges", &rlen);
	if (ranges == NULL) {
		printk("Missing 'ranges' property in device-tree node %s\n",
			dev->full_name);
		return;
	}

	/* Parse it */
	while ((rlen -= np * 4) >= 0) {
		/* Read next ranges element */
		pci_space = ranges[0];
		pci_addr = of_read_number(ranges + 1, 2);
		cpu_addr = of_translate_address(dev, ranges + 3);
		size = of_read_number(ranges + pna + 3, 2);
		ranges += np;

		/* If we failed translation or got a zero-sized region */
		if (cpu_addr == OF_BAD_ADDR || size == 0)
			continue;

		/* Now consume following elements while they are contiguous */
		for (; rlen >= np * sizeof(u32);
				ranges += np, rlen -= np * 4) {
			if (ranges[0] != pci_space)
				break;
			pci_next = of_read_number(ranges + 1, 2);
			cpu_next = of_translate_address(dev, ranges + 3);
			if (pci_next != pci_addr + size ||
					cpu_next != cpu_addr + size)
				break;
			size += of_read_number(ranges + pna + 3, 2);
		}

		/* Act based on address space type */
		res = NULL;
		switch (pci_space & 0x3) {
			case 1:		/* PCI IO space */
				PCIE_DEBUG(
					"  IO 0x%016llx..0x%016llx -> 0x%016llx\n",
					cpu_addr, cpu_addr + size - 1, pci_addr);
				/* pci_io_size and io_base_phys always represent IO
				 * space starting at 0 so we factor in pci_addr
				 */
				port->ob_pci_mem_offset[0] = 0;

				/* Build resource */
				res = &port->res[0];
				res->flags = IORESOURCE_IO | IORESOURCE_MEM_64;
				res->start = cpu_addr;
				break;
			case 2:		/* PCI Memory space */
			case 3:		/* PCI 64 bits Memory space */
				PCIE_DEBUG(
					" MEM 0x%016llx..0x%016llx -> 0x%016llx %s\n",
					cpu_addr, cpu_addr + size - 1, pci_addr,
					(pci_space & 0x40000000) ? "Prefetch" : "");

				/* We support only 3 memory ranges */
				if (memno >= 3) {
					printk("\\--> Skipped (too many)!\n");
					continue;
				}
				port->ob_pci_mem_offset[memno + 1] = 0;
				/* Build resource */
				res = &port->res[memno + 1];
				res->flags = IORESOURCE_MEM | IORESOURCE_MEM_64;
				if (pci_space & 0x40000000)
					res->flags |= IORESOURCE_PREFETCH;
				res->start = cpu_addr;
				memno++;
				break;
		}
		if (res != NULL) {
			res->name = dev->full_name;
			res->end = res->start + size - 1;
			res->parent = NULL;
			res->sibling = NULL;
			res->child = NULL;
		}
	}
}

static int __init apm_pcie_parse_dma_ranges(struct apm_pcie_port *port,
					     struct resource *res)
{
	struct device_node *dn = port->node;
	const u32 *ranges;
	u64 size = 0;
	int rlen;

	/* Default 0 to 4GB */
	res->flags = IORESOURCE_MEM | IORESOURCE_PREFETCH | IORESOURCE_MEM_64;
	res->start = res->end = 0;

	/* Get dma-ranges property */
	ranges = of_get_property(dn, "dma-ranges", &rlen);
	if (ranges == NULL)
		goto out;

	/* Walk it */
	size = parse_memory_ranges(res, port, 0, ranges, rlen);

	/* Check we are a power of 2 size and that base is a multiple of size*/
	if (!is_power_of_2(size) ||
	    (res->start & (size - 1)) != 0) {
		printk(KERN_ERR "%s: dma-ranges unaligned size 0x%llx\n",
		       dn->full_name, size);
		return -ENXIO;
	}

out:
	return 0;
}

/*
 * map the CSR and CFG regions.
 */
static int apm_pcie_map_regions(struct apm_pcie_port *port,
				struct resource csr_res,
				struct resource cfg_res)
{

	struct apm_pcie_map_tbl *hb = port->pcie_map_tbl;

	if (port->type == PTYPE_ROOT_PORT) {
		/* Only map the external config space in cfg_data for
		 * PCIe root-complexes.
		 */
		hb->cfg_addr_hi = HIDWORD(cfg_res.start);
		hb->cfg_addr_lo = LODWORD(cfg_res.start);
		hb->cfg_addr_size = RES_SIZE(&cfg_res);
		port->cfg_base = ioremap_nocache(cfg_res.start, hb->cfg_addr_size);
		if (port->cfg_base == NULL)
			return -ENOMEM;
	}

	/* Map CSR space */
	hb->csr_addr = csr_res.start;
	hb->csr_addr_size = RES_SIZE(&csr_res);
	port->csr_base = ioremap_nocache(csr_res.start, hb->csr_addr_size);
	if (port->csr_base == NULL)
		return -ENOMEM;

	return 0;
}

/*
 * read configuration values from DTS
 */
static int apm_pcie_read_config(struct apm_pcie_port *port,
			        struct resource *csr_res,
			        struct resource *cfg_res)
{
	struct device_node *np = port->node;
	u32 val32;
	u32 bus_range[2] = { 0x0 , 0x1f };
	const u8 *val;

	val = of_get_property(np, "device_type", NULL);
	if((val != NULL) && !strcmp(val, "ep")) {
		port->type = PTYPE_ENDPOINT;
		PCIE_DEBUG("PCIE%d: setup as endpoint\n", port->index);
	} else {
		port->type = PTYPE_ROOT_PORT;
		PCIE_DEBUG("PCIE%d: setup as root-complex\n", port->index);
	}

	of_property_read_u32(np, "link_width", &val32);
	port->link_width = val32;

	of_property_read_u32(np, "link_speed", &val32);
	port->link_speed = val32;

	of_property_read_u32(np, "serdes-diff-clk", &val32);
	port->serdes_clk = val32;

	of_property_read_u32_array(np, "bus-range", bus_range, 2);
	port->first_busno = bus_range[0];
	port->last_busno = bus_range[1];

	/* Get configured CSR space registers address */
	if (of_address_to_resource(np, 0, csr_res))
		return -EINVAL;

	/* Get configured config space registers address */
	if (of_address_to_resource(np, 1, cfg_res))
		return -EINVAL;

	return 0;
}

/*
 * Allocate reg space and system mem space for EP
 */
static int apm_pcie_alloc_ep_mem(struct apm_pcie_port *port)
{
	struct apm_pcie_ep_info *ep = &port->ep_info;

	ep->reg_space = dma_alloc_coherent(port->dev,
			APM_EP_REG_SPACE_SIZE, &ep->reg_addr_phys, GFP_KERNEL);
	if(!ep->reg_space) {
		printk(KERN_ERR "APM PCIe EP reg mem alloc fail\n");
		return -ENOMEM;
	}

	printk("EP: Reg Space - %p Phys - 0x%llx Size - 0x%x\n",
			ep->reg_space, (u64) ep->reg_addr_phys,
			APM_EP_REG_SPACE_SIZE);

	ep->mem_space = dma_alloc_coherent(port->dev,
			APM_EP_MEM_SPACE_SIZE, &ep->mem_addr_phys, GFP_KERNEL);

	if(!ep->mem_space) {
		printk(KERN_ERR "APM PCIe EP sys mem alloc fail\n");
		goto fail_ep;
	}

	PCIE_DEBUG("EP: Mem Space - %p Phys - 0x%llx Size - 0x%x\n",
			ep->mem_space, (u64) ep->mem_addr_phys,
			APM_EP_MEM_SPACE_SIZE);

	return 0;

fail_ep:
	dma_free_coherent(port->dev, APM_EP_REG_SPACE_SIZE,
			   ep->reg_space, ep->reg_addr_phys);
	return -ENOMEM;
}

/* Load inbound configuration address into table -
 * Currently supports single inbound memory region
 */
static int apm_pcie_populate_inbound_regions(struct apm_pcie_port *port,
					      struct resource ib_res)
{
	struct apm_pcie_map_tbl *hb = port->pcie_map_tbl;

	if (port->type == PTYPE_ROOT_PORT) {
		hb->ib_mem_addr[0].pcie_hi = HIDWORD(ib_res.start);
		hb->ib_mem_addr[0].pcie_lo = LODWORD(ib_res.start);
		hb->ib_mem_addr[0].pcie_size = RES_SIZE(&ib_res);
		hb->ib_mem_addr[0].pim1_hi = HIDWORD(port->ib_pci_mem_offset[0]);
		hb->ib_mem_addr[0].pim1_lo = LODWORD(port->ib_pci_mem_offset[0]);
		hb->ib_mem_addr[0].pim2_hi = 0;
		hb->ib_mem_addr[0].pim2_lo = 0;
	} else {
		/* Allocate space for EP */
		struct apm_pcie_ep_info *ep = &port->ep_info;
		if (apm_pcie_alloc_ep_mem(port))
			return -ENOMEM;
		hb->ib_mem_addr[0].pcie_size = APM_EP_REG_SPACE_SIZE;
		hb->ib_mem_addr[0].pim1_hi = HIDWORD(ep->reg_addr_phys);
		hb->ib_mem_addr[0].pim1_lo = LODWORD(ep->reg_addr_phys);
		hb->ib_mem_addr[0].pim2_hi = 0;
		hb->ib_mem_addr[0].pim2_lo = 0;
	}
	hb->ib_mem_addr[0].pim_size = (u64) ~(hb->ib_mem_addr[0].pcie_size - 1);
	return 0;
}

/* Load outbound configuration address into table -
 * Usually 3 memory region or 2 memory + 1 I/O regions
 */
static void apm_pcie_populate_outbound_regions(struct apm_pcie_port *port)
{
	struct apm_pcie_map_tbl *hb = port->pcie_map_tbl;
	phys_addr_t ddr_base = memblock_start_of_DRAM();
	int i = 0;
	int j = 2;

	if (RES_SIZE(&port->res[0]) > 0) {
		hb->ob_mem_addr[j].hi = HIDWORD(port->res[i].start);
		hb->ob_mem_addr[j].lo = LODWORD(port->res[i].start);
		hb->ob_mem_addr[j].size = RES_SIZE(&port->res[i]);
		if (port->type == PTYPE_ROOT_PORT) {
			hb->ob_mem_addr[j].pcie_hi = LODWORD(port->ob_pci_mem_offset[i]);
			hb->ob_mem_addr[j].pcie_lo = HIDWORD(port->ob_pci_mem_offset[i]);
		} else {
			hb->ob_mem_addr[j].pcie_hi = LODWORD(port->ob_pci_mem_offset[i] | ddr_base);
			hb->ob_mem_addr[j].pcie_lo = HIDWORD(port->ob_pci_mem_offset[i] | ddr_base);
		}
	}

	for (i = 1, j = 0; i < 3; i++, j++) {
		hb->ob_mem_addr[j].hi = HIDWORD(port->res[i].start);
		hb->ob_mem_addr[j].lo = LODWORD(port->res[i].start);
		hb->ob_mem_addr[j].size = RES_SIZE(&port->res[i]);
		if (port->type == PTYPE_ROOT_PORT) {
			hb->ob_mem_addr[j].pcie_hi = HIDWORD(port->ob_pci_mem_offset[i]);
			hb->ob_mem_addr[j].pcie_lo = LODWORD(port->ob_pci_mem_offset[i]);
		} else {
			hb->ob_mem_addr[j].pcie_hi = HIDWORD(port->ob_pci_mem_offset[i] | ddr_base);
			hb->ob_mem_addr[j].pcie_lo = LODWORD(port->ob_pci_mem_offset[i] | ddr_base);
		}
	}
}
	
static int apm_pcie_setup(int nr, struct pci_sys_data *sys)
{
	struct apm_pcie_port *pp;

	if (nr >= pcie_ports.nr_ports)
		return 0;

	pp = &pcie_ports.port[nr];
	if(pp->type == PTYPE_ENDPOINT)
		return 0;

	sys->io_offset = pp->res[0].start & 0xff00000000;
	sys->mem_offset = pp->res[1].start & 0xff00000000;

	/*
	 * IORESOURCE_MEM
	 */
	if (request_resource(&iomem_resource, &pp->res[1]))
		panic("Request PCIe Memory resource failed\n");

	/*
	 * IORESOURCE_MEM | IORESOURCE_PREFETCH
	 */
	if (request_resource(&iomem_resource, &pp->res[2]))
		panic("Request PCIe Prefetch Memory resource failed\n");

	/*
	 * IORESOURCE_IO
	 */
	if (request_resource(&iomem_resource, &pp->res[0]))
		panic("Request PCIe IO resource failed\n");

	pci_add_resource_offset(&sys->resources, &pp->res[1], sys->mem_offset);
	pci_add_resource_offset(&sys->resources, &pp->res[2], sys->mem_offset);
	pci_add_resource_offset(&sys->resources, &pp->res[0], sys->io_offset);

	sys->private_data = pp;
	return 1;
}

static int apm_pcie_map_irq(const struct pci_dev *pci_dev, u8 slot, u8 pin)
{
	struct of_irq oirq;
	unsigned int virq = 0;

//	if (pci_is_root_bus(pci_dev->bus))
//		return -1;

	PCIE_VDEBUG("PCI: Try to map irq for dev: %p name: %s...\n",
		    pci_dev, pci_name(pci_dev));

	if (of_irq_map_pci((struct pci_dev *) pci_dev, &oirq)) {
		PCIE_DEBUG("No map ! Using slot %d (pin %d)\n", slot, pin);
		return -1;
	} else {
		PCIE_VDEBUG(" Got one, spec %d cells (0x%08x 0x%08x...) on %s\n",
				oirq.size, oirq.specifier[0], oirq.specifier[1],
				oirq.controller ? oirq.controller->full_name :
				"<default>");

		virq = irq_create_of_mapping(oirq.controller, oirq.specifier,
				      oirq.size);
		if (virq == NO_IRQ) {
			PCIE_ERR("Failed to map PCI legacy interrupt\n");
			return -1;
		}
	}

	PCIE_DEBUG("Mapped to linux irq %d\n", virq);
	return virq;
}

static struct pci_bus __init *apm_pcie_scan_bus(int nr,
						struct pci_sys_data *sys)
{
	struct apm_pcie_port *pp;

	if (nr >= pcie_ports.nr_ports)
		return NULL;

	pp = &pcie_ports.port[nr];
	if(pp->type == PTYPE_ENDPOINT)
		return NULL;
	
	PCIE_DEBUG("%s: Scanning PCIe Port:%d Bus:%d\n", __func__,
			nr, sys->busnr);

	pp->last_busno = sys->busnr + (pp->last_busno - pp->first_busno);
	pp->first_busno = sys->busnr;

	PCIE_DEBUG("%s: pp->first_busno: %d, pp->last_busno: %d\n", __func__,
			pp->first_busno, pp->last_busno);

	return pci_scan_root_bus(NULL, sys->busnr, &apm_pcie_ops,
				 sys, &sys->resources);
}

static void __init apm_pcie_preinit(void)
{
#ifdef APM_PCIE_DEBUG
	pcibios_setup("debug");
#endif
}

static struct hw_pci apm_pcie_hw __initdata = {
	.nr_controllers = APM_PCIE_MAX_PORTS,
	.setup          = apm_pcie_setup,
	.preinit	= apm_pcie_preinit,
	.scan           = apm_pcie_scan_bus,
//	.swizzle        = pci_std_swizzle,
	.map_irq        = apm_pcie_map_irq,
};

static int __init apm_pcie_probe_bridge(struct platform_device *pdev)
{
	struct device_node *np = of_node_get(pdev->dev.of_node);
	struct apm_pcie_port *port = NULL;
	struct resource csr_res, cfg_res, ib_res;
	struct apm_pcie_map_tbl *hb;
	u32 val32;
	int retval = 0;

	/* Get the port number from the device-tree */
	of_property_read_u32(np, "port", &val32);
	if (val32 < 0 || val32 >= pcie_ports.nr_ports)
		return -EINVAL;

	port = &pcie_ports.port[val32];
	BUG_ON(port == NULL); 
	port->index = val32;
	port->node = np;

	if (apm_pcie_read_config(port, &csr_res, &cfg_res))
		return -EINVAL;
	PCIE_DEBUG("Probing APM88xxxx PCIE Port %d X%d, Gen-%d\n", val32,
				port->link_width, port->link_speed + 1);
	port->dev = &pdev->dev;

	hb = kzalloc(sizeof(struct apm_pcie_map_tbl), GFP_KERNEL);
	if(!hb)
		goto fail;
	port->pcie_map_tbl = hb;

	/* Map CFG and CSR regions */
	retval = apm_pcie_map_regions(port, csr_res, cfg_res);
	if (retval)
		goto fail;

	PCIE_DEBUG("PCIE%d: CSR 0x%02X_%08X (0x%p) size 0x%X "
		   "CFG 0x%02X_%08X (0x%p) size 0x%X\n",
		   port->index, HIDWORD(hb->csr_addr), LODWORD(hb->csr_addr),
		   port->csr_base, hb->csr_addr_size, hb->cfg_addr_hi,
		   hb->cfg_addr_lo, port->cfg_base, hb->cfg_addr_size);

	apm_pci_process_bridge_OF_ranges(port);
	apm_pcie_populate_outbound_regions(port);
	retval = apm_pcie_parse_dma_ranges(port, &ib_res);
	if (retval) {
		printk(KERN_ERR "PCIe%d DMA range error!\n", port->index);
		goto fail;
	}
#ifdef CONFIG_PCI_MSI
	retval = apm_pcie_parse_map_msi_ranges(port);
	if (retval) {
		printk(KERN_ERR "PCIe%d MSI range error!\n", port->index);
		goto fail;
	}
#endif

#if 0
	port->clk = clk_get(port->dev, NULL);
	if(!port->clk)
		printk(KERN_ERR "Can't get PCIE clock\n");
	else if(clk_prepare_enable(port->clk))
		printk(KERN_ERR "PCIE clock prepare enable failed\n");
#endif

	if (apm_pcie_populate_inbound_regions(port, ib_res))
		goto fail;

	retval = apm_init_pcie(port);
	if (retval)
		goto fail;

	apm_pcie_setup_port(port);
	apm_pcie_setup_primary_bus(port);

	if (++probe_cnt == pcie_ports.nr_ports) {
		apm_pcie_hw.nr_controllers = pcie_ports.nr_ports;
		pci_common_init(&apm_pcie_hw);
	}

	return retval;
fail:
	iounmap(port->csr_base);
	iounmap(port->cfg_base);
	kfree(hb);
	return retval;
}




#ifdef CONFIG_PM
static int apm_pcie_hwresume(struct apm_pcie_port *port)
{
	void *cfg_base = port->cfg_base;
	u32 flags;

	/* Check if we actually went into deep sleep */
	apm_in32(cfg_base + PCI_VENDOR_ID, &flags);
	if (flags == port->pwred_off_flags)
		return 0;

	/* Re-initilize hardware */
	apm_init_pcie(port);
	printk(KERN_INFO "PCIE%d: re-configured as %s\n", 
		port->index,(port->type == PTYPE_ROOT_PORT) ? 
				"root-complex" :"endpoint");
	/* Re-setup Root/Endpoint */
	if (apm_pcie_setup_port(port)) {
		printk(KERN_ERR "PCIE%d: Failed to resume\n");
		return -1;
	}
	return 0;
}

static int apm_pcie_suspend(void)
{
	int i;
	
	for (i = 0; i < pcie_ports.nr_ports; i++) {
		struct apm_pcie_port *port = &pcie_ports.port[i]; 
		if (port &&  port->cfg_base) {
			void *cfg_base = port->cfg_base;
			apm_in32(cfg_base + PCI_VENDOR_ID,
				&port->pwred_off_flags);
		}
	} 

	return 0;
}

static int apm_pcie_resume(void)
{
	int i;
	for (i = 0; i < pcie_ports.nr_ports; i++) {
		struct apm_pcie_port *port = &pcie_ports.port[i]; 
		if (port &&  port->cfg_base) {
			/* Re-program the bridge again */
			apm_pcie_hwresume(port);
		}
	} 
	return 0;
}

static struct syscore_ops apm_pcie_pm_syscore_ops = {
	.suspend = apm_pcie_suspend,
	.resume = apm_pcie_resume,
};

static int __init apm_pcie_pm_init(void)
{
	register_syscore_ops(&apm_pcie_pm_syscore_ops);

	return 0;
}
#endif /* CONFIG_PM */

static const __initconst struct of_device_id apm_pcie_match[] = {
        { .compatible     = "apm88xxxx,pcie", },
        { },
};

static struct  platform_driver apm_pcie_driver = {
        .driver = {
                .name = "apm-pcie",
                .owner = THIS_MODULE,
                .of_match_table = apm_pcie_match,
        },
        .probe          = apm_pcie_probe_bridge,
#ifdef CONFIG_PM
        .suspend = apm_pcie_suspend,
        .resume = apm_pcie_resume,
#endif
};

static int __init apm_pcie_init(void)
{
	int rc;
        struct device_node *np;

	printk(KERN_ERR "APM88xxxx: PCIe driver v%s\n", PCIE_DRIVER_VERSION);

	/* Count total number of pcie nodes */
        pcie_ports.nr_ports = 0;
        for_each_compatible_node(np, NULL, "apm88xxxx,pcie") {
		if (of_device_is_available(np))
                        pcie_ports.nr_ports++;
	}

	rc = platform_driver_register(&apm_pcie_driver);
	if (rc) {
		printk(KERN_ERR "APM88xxxx PCIe Driver registration failed\n");
		return rc;
	}
#ifdef CONFIG_PM
	apm_pcie_pm_init();
#endif
	return 0;
}
subsys_initcall(apm_pcie_init);
