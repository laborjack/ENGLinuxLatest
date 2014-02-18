/**
 * Applied Micro X-Gene PCIe core configuration file
 *
 * Copyright (c) 2013 Applied Micro Circuits Corporation.
 *
 * Author: Tanmay Inamdar <tinamdar@apm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "pcie-xgene-core.h"
#include "pcie-xgene-serdes.h"

/* PCIE Out/In to CSR */
int xgene_pcie_out32(void *addr, u32 val)
{
	writel(val, addr);
	return 0;
}

int xgene_pcie_in32(void *addr, u32 *val)
{
	*val = readl(addr);
	return 0;
}

/* PCIE Configuration Out/In */
int xgene_pcie_cfg_out32(void *addr, u32 val)
{
	writel(val, addr);
	return 0;
}

int xgene_pcie_cfg_out16(void *addr, u16 val)
{
	phys_addr_t temp_addr = (phys_addr_t) addr & ~0x3;
	u32 val32 = readl((void *)temp_addr);

	switch ((phys_addr_t) addr & 0x3) {
	case 2:
		val32 &= ~0xFFFF0000;
		val32 |= (u32) val << 16;
		break;
	case 0:
	default:
		val32 &= ~0xFFFF;
		val32 |= val;
		break;
	}
	writel(val32, (void *)temp_addr);
	return 0;
}

int xgene_pcie_cfg_out8(void *addr, u8 val)
{
	phys_addr_t temp_addr = (phys_addr_t) addr & ~0x3;
	u32 val32 = readl((void *)temp_addr);

	switch ((phys_addr_t) addr & 0x3) {
	case 0:
		val32 &= ~0xFF;
		val32 |= val;
		break;
	case 1:
		val32 &= ~0xFF00;
		val32 |= (u32) val << 8;
		break;
	case 2:
		val32 &= ~0xFF0000;
		val32 |= (u32) val << 16;
		break;
	case 3:
	default:
		val32 &= ~0xFF000000;
		val32 |= (u32) val << 24;
		break;
	}
	writel(val32, (void *)temp_addr);
	return 0;
}

int xgene_pcie_cfg_in32(void *addr, u32 *val)
{
	*val = readl(addr);
	return 0;
}

int xgene_pcie_cfg_in16(void *addr, u16 *val)
{
	phys_addr_t temp_addr = (phys_addr_t) addr & ~0x3;
	u32 val32;

	val32 = readl((void *)temp_addr);

	switch ((phys_addr_t) addr & 0x3) {
	case 2:
		*val = val32 >> 16;
		break;
	case 0:
	default:
		*val = val32;
		break;
	}
	return 0;
}

int xgene_pcie_cfg_in8(void *addr, u8 *val)
{
	phys_addr_t temp_addr = (phys_addr_t) addr & ~0x3;
	u32 val32;

	val32 = readl((void *)temp_addr);

	switch ((phys_addr_t) addr & 0x3) {
	case 3:
		*val = val32 >> 24;
		break;
	case 2:
		*val = val32 >> 16;
		break;
	case 1:
		*val = val32 >> 8;
		break;
	case 0:
	default:
		*val = val32;
		break;
	}
	return 0;
}

static void xgene_pcie_setup_link(struct xgene_pcie_port *port)
{
	void *csr_base = port->csr_base;
	u32 val;

	xgene_pcie_in32(csr_base + CFG_CONSTANTS_479_448, &val);
	switch (port->link_speed) {
	case PCIE_GEN1:
		val &= ~SUPPORT_5GTS_MASK;
		val &= ~SUPPORT_8GTS_MASK;
		break;
	case PCIE_GEN2:
		val |= SUPPORT_5GTS_MASK;
		val |= DIRECT_TO_5GTS_MASK;
		val &= ~SUPPORT_8GTS_MASK;
		val &= ~DIRECT_TO_8GTS_MASK;
		break;
	case PCIE_GEN3:
		val |= DIRECT_TO_8GTS_MASK;
		val |= SUPPORT_5GTS_MASK;
		val |= SUPPORT_8GTS_MASK;
		val |= DIRECT_TO_5GTS_MASK;
		break;
	}
	xgene_pcie_out32(csr_base + CFG_CONSTANTS_479_448, val);

	xgene_pcie_in32(csr_base + CFG_CONSTANTS_479_448, &val);
	val &= ~ADVT_INFINITE_CREDITS;
	xgene_pcie_out32(csr_base + CFG_CONSTANTS_479_448, val);

	xgene_pcie_in32(csr_base + CFG_8G_CONSTANTS_31_0, &val);
	val |= MGMT_DS_PORT_TX_PRESET_SET(val, 0x7);
	val |= MGMT_US_PORT_TX_PRESET_SET(val, 0x7);
	xgene_pcie_out32(csr_base + CFG_8G_CONSTANTS_31_0, val);

	if (port->link_speed == PCIE_GEN3) {
		xgene_pcie_in32(csr_base + CFG_8G_CONSTANTS_31_0, &val);
		val |= DWNSTRM_EQ_SKP_PHS_2_3;
		xgene_pcie_out32(csr_base + CFG_8G_CONSTANTS_31_0, val);
	}
}

static void xgene_pcie_program_core(void *csr_base)
{
	u32 val;

	xgene_pcie_in32(csr_base + CFG_CONSTANTS_31_00, &val);
	val |= AER_OPTIONAL_ERROR_EN;
	xgene_pcie_out32(csr_base + CFG_CONSTANTS_31_00, val);
	xgene_pcie_out32(csr_base + INTXSTATUSMASK, 0x0);
	xgene_pcie_in32(csr_base + CFG_CONTROL_63_32, &val);
	val = (val & ~0xffff) | XGENE_PCIE_DEV_CTRL;
	xgene_pcie_out32(csr_base + CFG_CONTROL_63_32, val);
}

static void xgene_pcie_setup_outbound_regions(struct xgene_pcie_port *port)
{
	struct xgene_pcie_map_tbl *hb = port->map_tbl;
	int type = port->type;
	void *csr_base = port->csr_base;
	u32 addr = OMR1BARL;
	u32 flag = EN_REG;
	u32 i;
	u64 val64;

	for (i = 0; i < XGENE_PCIE_MAX_REGIONS; i++) {
		if (hb->ob_mem_addr[i].size == 0)
			continue;
		if (i == XGENE_IO)
			flag |= OB_LO_IO;
		xgene_pcie_out32(csr_base + addr, hb->ob_mem_addr[i].lo);
		xgene_pcie_out32(csr_base + addr + 0x04, hb->ob_mem_addr[i].hi);
		val64 = ~(hb->ob_mem_addr[i].size - 1) | flag;
		xgene_pcie_out32(csr_base + addr + 0x08, lower_32_bits(val64));
		xgene_pcie_out32(csr_base + addr + 0x0c, upper_32_bits(val64));
		xgene_pcie_out32(csr_base + addr + 0x10,
				 hb->ob_mem_addr[i].pcie_lo);
		xgene_pcie_out32(csr_base + addr + 0x14,
				 hb->ob_mem_addr[i].pcie_hi);
		addr += 0x18;
	}

	if (type == PTYPE_ROOT_PORT) {
		xgene_pcie_out32(csr_base + CFGBARL, hb->cfg_addr_lo);
		xgene_pcie_out32(csr_base + CFGBARH, hb->cfg_addr_hi);
		xgene_pcie_out32(csr_base + CFGCTL, EN_REG);
	}
}

static u64 xgene_pcie_set_ib_mask(struct xgene_pcie_port *port,
				    u32 addr, u32 flags)
{
	struct xgene_pcie_map_tbl *hb = port->map_tbl;
	void *csr_base = port->csr_base;
	u64 val64 = 0;
	u32 val32 = 0;
	u32 val;

	if (hb->ib_mem_addr[XGENE_MEM].pcie_size >= 1024)
		val64 = (~(hb->ib_mem_addr[XGENE_MEM].pcie_size - 1) &
			 PCI_BASE_ADDRESS_MEM_MASK) | flags;

	xgene_pcie_in32(csr_base + addr, &val32);
	val = (val32 & 0x0000ffff) | (lower_32_bits(val64) << 16);
	xgene_pcie_out32(csr_base + addr, val);

	xgene_pcie_in32(csr_base + addr + 0x04, &val32);
	val = (val32 & 0xffff0000) | (lower_32_bits(val64) >> 16);
	xgene_pcie_out32(csr_base + addr + 0x04, val);

	xgene_pcie_in32(csr_base + addr + 0x04, &val32);
	val = (val32 & 0x0000ffff) | (upper_32_bits(val64) << 16);
	xgene_pcie_out32(csr_base + addr + 0x04, val);

	xgene_pcie_in32(csr_base + addr + 0x08, &val32);
	val = (val32 & 0xffff0000) | (upper_32_bits(val64) >> 16);
	xgene_pcie_out32(csr_base + addr + 0x08, val);

	return val64;
}

static void xgene_pcie_configure_pim_settings(struct xgene_pcie_port *port,
					      u32 addr, u32 i)
{
	struct xgene_pcie_map_tbl *hb = port->map_tbl;
	void *csr_base = port->csr_base;
	u32 val;

	xgene_pcie_out32(csr_base + addr, hb->ib_mem_addr[i].pim1_lo);
	hb->ib_mem_addr[i].pim1_hi |= EN_COHERENCY;
	xgene_pcie_out32(csr_base + addr + 0x04, hb->ib_mem_addr[i].pim1_hi);
	xgene_pcie_out32(csr_base + addr + 0x08, hb->ib_mem_addr[i].pim2_lo);
	xgene_pcie_out32(csr_base + addr + 0x0c, hb->ib_mem_addr[i].pim2_hi);
	val = lower_32_bits(hb->ib_mem_addr[i].pim_size);
	xgene_pcie_out32(csr_base + addr + 0x10, val);
	val = upper_32_bits(hb->ib_mem_addr[i].pim_size);
	xgene_pcie_out32(csr_base + addr + 0x14, val);
}

static void xgene_pcie_setup_ep_inbound_regions(struct xgene_pcie_port *port)
{
	struct xgene_pcie_map_tbl *hb = port->map_tbl;
	u32 i, mask_addr;
	u32 addr = PIM1_1L;
	u32 flags = PCI_BASE_ADDRESS_MEM_PREFETCH |
		    PCI_BASE_ADDRESS_MEM_TYPE_64;

	for (i = 0; i < XGENE_PCIE_MAX_REGIONS; i++) {
		if (hb->ib_mem_addr[i].pcie_size == 0)
			continue;
		if (i == 1)
			addr = PIM2_1L;
		else if (i == 2)
			addr = PIM3_1L;

		mask_addr = CFG_CONSTANTS_159_128 + (i * 0x10);
		xgene_pcie_set_ib_mask(port, mask_addr, flags);
		xgene_pcie_configure_pim_settings(port, addr, i);
	}
}

static void xgene_pcie_setup_rc_inbound_regions(struct xgene_pcie_port *port)
{
	struct xgene_pcie_map_tbl *hb = port->map_tbl;
	void *csr_base = port->csr_base;
	void *cfg_addr = port->cfg_base;
	u32 i, addr, val64;
	u32 flags = PCI_BASE_ADDRESS_MEM_PREFETCH |
		    PCI_BASE_ADDRESS_MEM_TYPE_64;

	for (i = 0; i < XGENE_PCIE_MAX_REGIONS; i++) {
		addr = PIM1_1L + (i * 0x18);
		if (hb->ib_mem_addr[i].pcie_size == 0)
			continue;
		if (i == 0) {
			u32 mask_addr = CFG_CONSTANTS_159_128;
			u32 val;
			val64 = xgene_pcie_set_ib_mask(port, mask_addr, flags);
			val = hb->ib_mem_addr[i].pcie_lo;
			val &= PCI_BASE_ADDRESS_MEM_MASK;
			val |= flags;
			xgene_pcie_out32(cfg_addr + PCI_BASE_ADDRESS_0, val);
			val = hb->ib_mem_addr[i].pcie_hi;
			xgene_pcie_out32(cfg_addr + PCI_BASE_ADDRESS_1, val);
		} else {
			xgene_pcie_out32(csr_base + addr,
				    hb->ib_mem_addr[i].pcie_lo);
			val64 = ~(hb->ib_mem_addr[i].pcie_size - 1) | EN_REG;
			xgene_pcie_out32(csr_base + addr + 0x04,
				    lower_32_bits(val64));
			addr += 0x08;
		}
		xgene_pcie_configure_pim_settings(port, addr, i);
	}

	if (hb->ib_rom_addr_size == 0)
		return;
	xgene_pcie_out32(csr_base + PIM4L, hb->ib_rom_addr_lo);
	xgene_pcie_out32(csr_base + PIM4L + 0x4, hb->ib_rom_addr_hi);
	xgene_pcie_out32(cfg_addr + PCI_ROM_ADDRESS1, hb->ib_rom_addr_lo);
	xgene_pcie_set_ib_mask(port, CFG_CONSTANTS_351_320, 0);
}

static u8 xgene_pcie_poll_pcie_linkup(struct xgene_pcie_port *port)
{
	void *csr_base = port->csr_base;
	u32 val32;
	u64 wait;

	port->link_up = 0;
	for (wait = XGENE_PCIE_LINK_WAIT; wait > 0; wait--) {
		xgene_pcie_in32(csr_base + PCIECORE_CTLANDSTATUS, &val32);
		if (val32 & LINK_UP_MASK) {
			port->link_up = 1;
			port->cur_link_speed = PIPE_PHY_RATE_RD(val32);
			xgene_pcie_in32(csr_base + PCIE_STATUS_31_0, &val32);
			port->cur_link_width = val32 >> 26;
			break;
		}
	}

	return !port->link_up;
}

static void xgene_pcie_setup_root_complex(struct xgene_pcie_port *port)
{
	void *csr_base = port->csr_base;
	u32 val;

	val = (XGENE_PCIE_BRIDGE_DEVICEID << 16) | XGENE_PCIE_VENDORID;
	xgene_pcie_out32(csr_base + CFG_CONSTANTS_31_00, val);

	xgene_pcie_in32(csr_base + CFG_CONSTANTS_63_32, &val);
	val &= ~CLASS_CODE_MASK;
	val |= PCI_CLASS_BRIDGE_PCI << 16;
	xgene_pcie_out32(csr_base + CFG_CONSTANTS_63_32, val);

	xgene_pcie_in32(csr_base + CFG_CONSTANTS_479_448, &val);
	val |= SWITCH_PORT_MODE_MASK;
	val &= ~PM_FORCE_RP_MODE_MASK;
	val &= ~ADVT_INFINITE_CREDITS;
	xgene_pcie_out32(csr_base + CFG_CONSTANTS_479_448, val);

	xgene_pcie_setup_link(port);

	xgene_pcie_in32(csr_base + CFG_8G_CONSTANTS_287_256, &val);
	val = EQ_PRE_CURSOR_LANE0_SET(val, 0x7);
	val = EQ_PRE_CURSOR_LANE1_SET(val, 0x7);
	xgene_pcie_out32(csr_base + CFG_8G_CONSTANTS_287_256, val);

	xgene_pcie_in32(csr_base + CFG_8G_CONSTANTS_319_288, &val);
	val = EQ_PRE_CURSOR_LANE2_SET(val, 0x7);
	val = EQ_PRE_CURSOR_LANE3_SET(val, 0x7);
	xgene_pcie_out32(csr_base + CFG_8G_CONSTANTS_319_288, val);

	xgene_pcie_in32(csr_base + CFG_8G_CONSTANTS_351_320, &val);
	val = EQ_PRE_CURSOR_LANE4_SET(val, 0x7);
	val = EQ_PRE_CURSOR_LANE5_SET(val, 0x7);
	xgene_pcie_out32(csr_base + CFG_8G_CONSTANTS_351_320, val);

	xgene_pcie_in32(csr_base + CFG_8G_CONSTANTS_383_352, &val);
	val = EQ_PRE_CURSOR_LANE6_SET(val, 0x7);
	val = EQ_PRE_CURSOR_LANE7_SET(val, 0x7);
	xgene_pcie_out32(csr_base + CFG_8G_CONSTANTS_383_352, val);

	xgene_pcie_in32(csr_base + CFG_8G_CONSTANTS_159_128, &val);
	val = EQ_UPDN_POST_STEP_SET(val, 0x1);
	val = EQ_UPDN_POST_STEP_SET(val, 0x1);
	xgene_pcie_out32(csr_base + CFG_8G_CONSTANTS_159_128, val);

	xgene_pcie_in32(csr_base + CFG_CONTROL_191_160, &val);
	val &= ~DEVICE_PORT_TYPE_MASK;
	val |= XGENE_PORT_TYPE_RC;
	xgene_pcie_out32(csr_base + CFG_CONTROL_191_160, val);

	xgene_pcie_in32(csr_base + CFG_CONTROL_95_64, &val);
	val |= 2 << 26;
	xgene_pcie_out32(csr_base + CFG_CONTROL_95_64, val);

	xgene_pcie_in32(csr_base + CFG_CONSTANTS_415_384, &val);
	val = ENABLE_L1S_POWER_MGMT_SET(val, 1);
	xgene_pcie_out32(csr_base + CFG_CONSTANTS_415_384, val);
}

static void xgene_pcie_setup_endpoint(struct xgene_pcie_port *port)
{
	void *csr_base = port->csr_base;
	u32 val;

	xgene_pcie_in32(csr_base + CFG_CONSTANTS_479_448, &val);
	val &= ~SWITCH_PORT_MODE_MASK;
	val &= ~PM_FORCE_RP_MODE_MASK;
	xgene_pcie_out32(csr_base + CFG_CONSTANTS_479_448, val);

	xgene_pcie_in32(csr_base + CFG_CONTROL_191_160, &val);
	val &= ~DEVICE_PORT_TYPE_MASK;
	val &= ~SLOT_IMPLEMENTED_MASK;
	xgene_pcie_out32(csr_base + CFG_CONTROL_191_160, val);

	xgene_pcie_in32(csr_base + CFG_CONSTANTS_31_00, &val);
	val = (XGENE_PCIE_DEVICEID << 16) | XGENE_PCIE_VENDORID;
	xgene_pcie_out32(csr_base + CFG_CONSTANTS_31_00, val);

	xgene_pcie_in32(csr_base + CFG_CONSTANTS_63_32, &val);
	val &= REVISION_ID_MASK;
	val |= PCI_CLASS_BRIDGE_OTHER << 16;
	xgene_pcie_out32(csr_base + CFG_CONSTANTS_63_32, val);

	xgene_pcie_setup_link(port);
}

static int xgene_pcie_setup_port(struct xgene_pcie_port *port)
{
	int type = port->type;

	xgene_pcie_program_core(port->csr_base);
	if (type == PTYPE_ROOT_PORT)
		xgene_pcie_setup_root_complex(port);
	else
		xgene_pcie_setup_endpoint(port);

	xgene_pcie_reset_pcie_core_clk(port);
	xgene_pcie_setup_outbound_regions(port);
	if (type == PTYPE_ROOT_PORT)
		xgene_pcie_setup_rc_inbound_regions(port);
	else
		xgene_pcie_setup_ep_inbound_regions(port);
/*
 * A component must enter the LTSSM Detect state within 20 ms of the end of
 * Fundamental Reset.
 */
	mdelay(20);
	if (type == PTYPE_ROOT_PORT) {
		if (xgene_pcie_poll_pcie_linkup(port))
			if (port->link_speed > PCIE_GEN1)
				port->link_speed--;

		return !port->link_up;
	}
	return 0;
}

/*
 * Setup and initialize the APM X-Gene PCIe controller
 */
int xgene_pcie_setup_core(struct xgene_pcie_port *port)
{
	int ret = 0;

	/* X-Gene RC always starts at GEN-3. However few low gen PCIe cards
	 * don't get link up. So when X-Gene PCIe port is RC and there is no
	 * link up, then reduce the GEN speed on X-Gene side and retry for
	 * link up.
	 */
	while (1) {
		ret = xgene_pcie_serdes_init(port);
		if (ret)
			break;
		ret = xgene_pcie_setup_port(port);
		if (port->type == PTYPE_ROOT_PORT) {
			if (port->link_up || (port->link_speed == PCIE_GEN1))
				break;
		} else
			break;
	}
	/*
	 * With a Downstream Port that supports Link speeds greater
	 * than 5.0 GT/s, software must wait a minimum of 200 ms after
	 * link training completes before sending a configuration
	 * request to the device immediately below that Port
	 */
	mdelay(200);
	return ret;
}
