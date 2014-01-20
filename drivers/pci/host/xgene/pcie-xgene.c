/**
 * APM X-Gene PCIe Driver
 *
 * Copyright (c) 2013 Applied Micro Circuits Corporation.
 *
 * Author: Tanmay Inamdar <tinamdar@apm.com>.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include "pcie-xgene.h"
#include "pcie-xgene-core.h"

static struct xgene_pcie_info pcie_ports;

static struct xgene_pcie_port *xgene_pcie_bus_to_port(struct pci_bus *bus)
{
	struct pci_sys_data *sys = bus->sysdata;
	struct xgene_pcie_port *port = sys->private_data;

	return port;
}

/* IO ports are memory mapped */
void __iomem *__pci_ioport_map(struct pci_dev *dev, unsigned long port,
			       unsigned int nr)
{
	return ioremap_nocache(port, nr);
}

static void __iomem *xgene_pcie_get_cfg_base(struct pci_bus *bus)
{
	struct xgene_pcie_port *port = xgene_pcie_bus_to_port(bus);
	phys_addr_t addr = (phys_addr_t) port->cfg_base;

	if (bus->number >= (port->first_busno + 1))
		addr |= AXI_EP_CFG_ACCESS;

	return (void *)addr;
}

static void xgene_pcie_set_rtdid_reg(struct pci_bus *bus, uint devfn)
{
	struct xgene_pcie_port *port = xgene_pcie_bus_to_port(bus);
	unsigned int b, d, f;
	u32 rtdid_val = 0;

	b = bus->number;
	d = PCI_SLOT(devfn);
	f = PCI_FUNC(devfn);

	if (bus->number == port->first_busno)
		rtdid_val = (b << 24) | (d << 19) | (f << 16);
	else if (bus->number >= (port->first_busno + 1))
		rtdid_val = (port->first_busno << 24) |
			    (b << 8)  | (d << 3)  | (f << 0);

	xgene_pcie_out32(port->csr_base + RTDID, rtdid_val);
}

static int xgene_pcie_read_config(struct pci_bus *bus, unsigned int devfn,
			    int offset, int len, u32 *val)
{
	void __iomem *addr;
	u8 val8;
	u16 val16;

	xgene_pcie_set_rtdid_reg(bus, devfn);
	addr = xgene_pcie_get_cfg_base(bus);
	switch (len) {
	case 1:
		xgene_pcie_cfg_in8(addr + offset, &val8);
		*val = val8;
		break;
	case 2:
		xgene_pcie_cfg_in16(addr + offset, &val16);
		*val = val16;
		break;
	default:
		xgene_pcie_cfg_in32(addr + offset, val);
		break;
	}
	return PCIBIOS_SUCCESSFUL;
}

static int xgene_pcie_write_config(struct pci_bus *bus, unsigned int devfn,
			     int offset, int len, u32 val)
{
	void __iomem *addr;

	xgene_pcie_set_rtdid_reg(bus, devfn);
	addr = xgene_pcie_get_cfg_base(bus);
	switch (len) {
	case 1:
		xgene_pcie_cfg_out8(addr + offset, (u8)val);
		break;
	case 2:
		xgene_pcie_cfg_out16(addr + offset, (u16)val);
		break;
	default:
		xgene_pcie_cfg_out32(addr + offset, val);
		break;
	}
	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops xgene_pcie_ops = {
	.read  = xgene_pcie_read_config,
	.write = xgene_pcie_write_config
};

struct device_node *pcibios_get_phb_of_node(struct pci_bus *bus)
{
	struct xgene_pcie_port *port = xgene_pcie_bus_to_port(bus);

	return of_node_get(port->node);
}

static void xgene_pcie_fixup_bridge(struct pci_dev *dev)
{
	int i;

	for (i = 0; i < DEVICE_COUNT_RESOURCE; i++) {
		dev->resource[i].start = dev->resource[i].end = 0;
		dev->resource[i].flags = 0;
	}
}
DECLARE_PCI_FIXUP_HEADER(XGENE_PCIE_VENDORID, XGENE_PCIE_BRIDGE_DEVICEID,
			xgene_pcie_fixup_bridge);

void xgene_pcie_setup_primary_bus(struct xgene_pcie_port *port)
{
	u32 val;
	void *cfg_addr = port->cfg_base;

	xgene_pcie_in32(cfg_addr + PCI_PRIMARY_BUS, &val);
	val &= ~PCI_PRIMARY_BUS_MASK;
	val |=  (port->last_busno << 16)	|
		((port->first_busno + 1) << 8)	|
		(port->first_busno);
	xgene_pcie_out32(cfg_addr + PCI_PRIMARY_BUS, val);
}

/*
 * read configuration values from DTS
 */
static int xgene_pcie_read_dts_config(struct xgene_pcie_port *port)
{
	struct xgene_pcie_map_tbl *tbl = port->map_tbl;
	struct device_node *np = port->node;
	struct resource csr_res;
	u32 val32, ret;
	const u8 *val;

	val = of_get_property(np, "device_type", NULL);
	if ((val != NULL) && !strcmp(val, "ep"))
		port->type = PTYPE_ENDPOINT;
	else
		port->type = PTYPE_ROOT_PORT;

	of_property_read_u32(np, "link_width", &val32);
	port->link_width = val32;

	of_property_read_u32(np, "link_speed", &val32);
	port->link_speed = val32;

	of_property_read_u32(np, "serdes-diff-clk", &val32);
	port->serdes_clk = val32;

	port->reset_gpio = -1;
	ret = of_property_read_u32(np, "reset_gpio", &val32);
	if (ret == 0)
		port->reset_gpio = val32;

	/* Get configured CSR space registers address */
	if (of_address_to_resource(np, 0, &csr_res))
		return -EINVAL;

	tbl->csr_addr = csr_res.start;
	tbl->csr_addr_size = resource_size(&csr_res);
	port->csr_base = ioremap_nocache(csr_res.start, tbl->csr_addr_size);
	if (port->csr_base == NULL)
		return -ENOMEM;

	return 0;
}

static int xgene_pcie_alloc_ep_mem(struct xgene_pcie_port *port)
{
	struct xgene_pcie_ep_info *ep = &port->ep_info;

	ep->reg_virt = dma_alloc_coherent(port->dev, XGENE_PCIE_EP_MEM_SIZE,
					   &ep->reg_phys, GFP_KERNEL);
	if (ep->reg_virt == NULL)
		return -ENOMEM;

	dev_info(port->dev, "EP: Virt - %p Phys - 0x%llx Size - 0x%x\n",
		 ep->reg_virt, (u64) ep->reg_phys,
		 XGENE_PCIE_EP_MEM_SIZE);
	return 0;
}

static int xgene_pcie_populate_inbound_regions(struct xgene_pcie_port *port)
{
	struct xgene_pcie_map_tbl *hb = port->map_tbl;
	struct resource *msi_res = &port->res[XGENE_MSI];
	struct ib_mem_addr *mem = &hb->ib_mem_addr[XGENE_MEM];
	struct ib_mem_addr *msi = &hb->ib_mem_addr[XGENE_MSI];
	u64 ddr_size = memblock_phys_mem_size();
	phys_addr_t ddr_base = memblock_start_of_DRAM();

	if (port->type == PTYPE_ROOT_PORT) {
		mem->pcie_hi = upper_32_bits(ddr_base);
		mem->pcie_lo = lower_32_bits(ddr_base);
		mem->pcie_size = ddr_size;
		mem->pim1_hi = upper_32_bits(ddr_base);
		mem->pim1_lo = lower_32_bits(ddr_base);
		mem->pim_size = (u64) ~(ddr_size - 1);
	} else {
		struct xgene_pcie_ep_info *ep = &port->ep_info;
		if (xgene_pcie_alloc_ep_mem(port))
			return -ENOMEM;
		mem->pcie_size = XGENE_PCIE_EP_MEM_SIZE;
		mem->pim1_hi = upper_32_bits(ep->reg_phys);
		mem->pim1_lo = lower_32_bits(ep->reg_phys);
		mem->pim_size = (u64) ~(mem->pcie_size - 1);
	}

	msi->pcie_hi = upper_32_bits(msi_res->start);
	msi->pcie_lo = lower_32_bits(msi_res->start);
	msi->pcie_size = resource_size(msi_res);
	msi->pim1_hi = upper_32_bits(msi_res->start);
	msi->pim1_lo = lower_32_bits(msi_res->start);
	msi->pim_size = (u64) ~(resource_size(msi_res) - 1);
	return 0;
}

static void xgene_pcie_res_to_map_tbl(struct xgene_pcie_map_tbl *tbl,
				      u32 index, struct resource *res)
{
	u64 axi_addr = xgene_pcie_get_iof_addr(res->start);
	if (axi_addr != res->start) {
		tbl->ob_mem_addr[index].pcie_hi = upper_32_bits(res->start);
		tbl->ob_mem_addr[index].pcie_lo = lower_32_bits(res->start);
	}
	tbl->ob_mem_addr[index].hi = upper_32_bits(axi_addr);
	tbl->ob_mem_addr[index].lo = lower_32_bits(axi_addr);
	tbl->ob_mem_addr[index].size = resource_size(res);
}

static int xgene_pcie_map_cfg(struct xgene_pcie_port *port,
			      struct of_pci_range *range)
{
	struct xgene_pcie_map_tbl *tbl = port->map_tbl;
	struct device *dev = port->dev;
	u64 addr = xgene_pcie_get_iof_addr(range->cpu_addr);

	tbl->cfg_addr_hi = upper_32_bits(addr);
	tbl->cfg_addr_lo = lower_32_bits(addr);
	tbl->cfg_addr_size = range->size;
	port->cfg_base = ioremap_nocache(addr, range->size);
	if (port->cfg_base == NULL) {
		dev_err(dev, "failed to map cfg region!");
		return -ENOMEM;
	}
	return 0;
}

static int xgene_pcie_parse_map_ranges(struct xgene_pcie_port *port)
{
	struct device_node *np = port->node;
	struct of_pci_range range;
	struct of_pci_range_parser parser;
	struct xgene_pcie_map_tbl *tbl = port->map_tbl;
	struct device *dev = port->dev;
	u32 cfg_map_done = 0;
	int ret;

	if (of_pci_range_parser_init(&parser, np)) {
		dev_err(dev, "missing ranges property\n");
		return -EINVAL;
	}

	/* Get the I/O, memory, config ranges from DT */
	for_each_of_pci_range(&parser, &range) {
		struct resource *res = NULL;
		u64 restype = range.flags & IORESOURCE_TYPE_BITS;
		u64 end = range.cpu_addr + range.size - 1;
		dev_dbg(port->dev, "0x%08x 0x%016llx..0x%016llx -> 0x%016llx\n",
			range.flags, range.cpu_addr, end, range.pci_addr);

		switch (restype) {
		case IORESOURCE_IO:
			res = &port->res[XGENE_IO];
			of_pci_range_to_resource(&range, np, res);
			xgene_pcie_res_to_map_tbl(tbl, XGENE_IO, res);
			break;
		case IORESOURCE_MEM:
			res = &port->res[XGENE_MEM];
			of_pci_range_to_resource(&range, np, res);
			xgene_pcie_res_to_map_tbl(tbl, XGENE_MEM, res);
			break;
		case 0:
			if (!cfg_map_done) {
				/* config region */
				ret = xgene_pcie_map_cfg(port, &range);
				if (ret)
					return ret;
				cfg_map_done = 1;
			} else {
				/* msi region */
				res = &port->res[XGENE_MSI];
				of_pci_range_to_resource(&range, np, res);
			}
			break;
		default:
			dev_err(dev, "invalid io resource!");
			return -EINVAL;
		}
	}
	ret = xgene_pcie_populate_inbound_regions(port);
	if (ret)
		return ret;
	return 0;
}

static int xgene_pcie_setup(int nr, struct pci_sys_data *sys)
{
	struct xgene_pcie_port *pp = &pcie_ports.port[nr];

	if (pp->type == PTYPE_ENDPOINT)
		return 0;

	sys->io_offset  = pci_io_offset(pp->res[XGENE_IO].start);
	sys->mem_offset = pci_io_offset(pp->res[XGENE_MEM].start);

	BUG_ON(request_resource(&iomem_resource, &pp->res[XGENE_IO]) ||
		request_resource(&iomem_resource, &pp->res[XGENE_MEM]));

	pci_add_resource_offset(&sys->resources, &pp->res[XGENE_MEM],
				sys->mem_offset);
	pci_add_resource_offset(&sys->resources, &pp->res[XGENE_IO],
				sys->io_offset);
	sys->private_data = pp;
	return 1;
}

static int xgene_pcie_map_irq(const struct pci_dev *pci_dev, u8 slot, u8 pin)
{
	return of_irq_parse_and_map_pci(pci_dev, slot, pin);
}

static struct pci_bus __init *xgene_pcie_scan_bus(int nr,
						struct pci_sys_data *sys)
{
	struct xgene_pcie_port *pp =  &pcie_ports.port[nr];
	struct pci_bus *bus;

	if (pp->type == PTYPE_ENDPOINT)
		return NULL;

	pp->first_busno = sys->busnr;
	pp->last_busno  = 0xff;
	xgene_pcie_setup_primary_bus(pp);
	bus =  pci_scan_root_bus(NULL, sys->busnr, &xgene_pcie_ops,
				 sys, &sys->resources);
	if (bus)
		pp->last_busno  = bus->busn_res.end;

	return bus;
}

static struct hw_pci xgene_pcie_hw __initdata = {
	.nr_controllers = XGENE_PCIE_MAX_PORTS,
	.setup          = xgene_pcie_setup,
	.scan           = xgene_pcie_scan_bus,
	.map_irq        = xgene_pcie_map_irq,
};

static int __init xgene_pcie_probe_bridge(struct platform_device *pdev)
{
	struct device_node *np = of_node_get(pdev->dev.of_node);
	struct xgene_pcie_port *port = NULL;
	struct xgene_pcie_map_tbl *tbl;
	static u32 pcount;
	u32 val32;
	int ret;

	tbl = kzalloc(sizeof(*tbl), GFP_KERNEL);
	if (tbl == NULL)
		return -ENOMEM;

	of_property_read_u32(np, "port", &val32);
	port = &pcie_ports.port[val32];
	port->index = val32;
	port->node = np;
	port->map_tbl = tbl;
	port->dev = &pdev->dev;

	ret = xgene_pcie_read_dts_config(port);
	if (ret)
		goto fail;

	ret = xgene_pcie_parse_map_ranges(port);
	if (ret)
		goto fail;

	xgene_pcie_setup_core(port);

	if (port->type == PTYPE_ROOT_PORT) {
		if (!port->link_up)
			dev_info(port->dev,
				 "pcie%d: (RC) link down\n", port->index);
		else
			dev_info(port->dev,
				"port%d: (RC) X%d GEN-%d link up\n",
				port->index, port->cur_link_width,
				port->cur_link_speed + 1);
	} else
		dev_info(port->dev, "port%d: (EP)\n", port->index);

	if (++pcount == pcie_ports.nr_ports)
		pci_common_init(&xgene_pcie_hw);

	platform_set_drvdata(pdev, port);
	return 0;
fail:
	iounmap(port->csr_base);
	iounmap(port->cfg_base);
	kfree(tbl);
	return ret;
}

static const struct of_device_id xgene_pcie_match_table[] __initconst = {
	{ .compatible     = "xgene,pcie", },
	{ },
};

static struct  platform_driver xgene_pcie_driver = {
	.driver = {
		.name = "xgene-pcie",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(xgene_pcie_match_table),
	},
};

static int __init xgene_pcie_init(void)
{
	struct device_node *np;
	int nr_ports = 0;

	pr_info("X-Gene: PCIe driver\n");

	/* Count total number of pcie nodes */
	pcie_ports.nr_ports = 0;
	for_each_compatible_node(np, NULL, "xgene,pcie") {
		nr_ports++;
		if (of_device_is_available(np) == 1)
			pcie_ports.nr_ports++;
	}
	xgene_pcie_hw.nr_controllers = nr_ports;
	return platform_driver_probe(&xgene_pcie_driver,
			xgene_pcie_probe_bridge);
}
device_initcall(xgene_pcie_init);

MODULE_AUTHOR("Tanmay Inamdar <tinamdar@apm.com>");
MODULE_DESCRIPTION("APM X-Gene PCIe driver");
MODULE_LICENSE("GPL v2");
