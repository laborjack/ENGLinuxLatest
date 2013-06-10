/**
 * APM STORM NWL PCIe Driver
 *
 * Copyright (c) 2009 Applied Micro Circuits Corporation.
 * All rights reserved. Tanmay Inamdar <tinamdar@apm.com>
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
 * @file   apm88xxxx_pcie_common.c
 *
 * This module implements PCIe functionality for APM STORM SoC NWL.
 * This setups and configures PCIe controllers as either root complex
 * or endpoint.
 */
#include "apm88xxxx_pcie_common.h"

/* PCIE Out/In to CSR */
int apm_out32(void *addr, u32 val)
{
	u32 reg = ((u64)addr & 0xff00) >> 8;
	writel(val, addr);
	if ((reg != 0xa0) && (reg != 0xc0))
		PCIE_CSR_DEBUG("PCIE CSR WR: 0x%p value: 0x%08X\n", addr, val);
	return 0;
}

int apm_in32(void *addr, u32 *val)
{
	u32 reg = ((u64)addr & 0xff00) >> 8;
	*val = readl(addr);
	if ((reg != 0xa0) && (reg != 0xc0))
		PCIE_CSR_DEBUG("PCIE CSR RD: 0x%p value: 0x%08X\n", addr, *val);
	return 0;
}

/* PCIE Configuration Out/In */
int pcie_cfg_out32(void *addr, u32 val)
{
	writel(val, addr);
	PCIE_CSR_DEBUG("PCIE CFG WR: 0x%p value: 0x%08X (0x%08X)\n", 
			addr, val, readl(addr));
	return 0;
}

int pcie_cfg_out16(void *addr, u16 val)
{
	u64 temp_addr = (u64) addr & ~0x3;
	u32 val32  = readl((void *) temp_addr);
	
	switch ((u64)addr & 0x3) {
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
	PCIE_CSR_DEBUG("PCIE CFG WR16: 0x%p value: 0x%04X (0x%08llX 0x%08X)\n", 
			addr, val, temp_addr, val32); 
	writel(val32, (void *)temp_addr);
	return 0;
}

int pcie_cfg_out8(void *addr, u8 val)
{
	u64 temp_addr = (u64) addr & ~0x3;
	u32 val32  = readl((void *)temp_addr);
	
	switch ((u64)addr & 0x3) {
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
	PCIE_CSR_DEBUG("PCIE CFG WR8: 0x%p value: 0x%04X (0x%08llX 0x%08X)\n", 
			addr, val, temp_addr, val32); 
	writel(val32, (void *)temp_addr);
	return 0;
}

int pcie_cfg_in32(void *addr, u32 *val)
{
	*val = readl(addr);
	PCIE_CSR_DEBUG("PCIE CFG RD: 0x%p value: 0x%08X\n", addr, *val);
	return 0;
}

int pcie_cfg_in16(void *addr, u16 *val)
{
	u64 temp_addr = (u64) addr & ~0x3;
	u32 val32  = readl((void *)temp_addr);
	
	switch ((u64)addr & 0x3) {
	case 2:
		*val = val32 >> 16;
		break;
	case 0:
	default:
		*val = val32;
		break;
	}
	PCIE_CSR_DEBUG("PCIE CFG RD16: 0x%p value: 0x%04X (0x%08llX 0x%08X)\n", 
			addr, *val, temp_addr, val32); 
	return 0;
}

int pcie_cfg_in8(void *addr, u8 *val)
{
	u64 temp_addr = (u64) addr & ~0x3;
	u32 val32  = readl((void *)temp_addr);
	
	switch ((u64)addr & 0x3) {
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
	PCIE_CSR_DEBUG("PCIE CFG RD8: 0x%p value: 0x%02X (0x%08llX 0x%08X)\n", 
			addr, *val, temp_addr, val32);
	return 0;
}

static void apm_pcie_setup_link(struct apm_pcie_port *port) 
{
	void *csr_base = port->csr_base;
	u32 val;
	u32 mps = 1; /* max_payload_size: 0 = 128, 1 = 256, 2 = 512 */

	if (port->link_width == LNKW_X8) {
		PCIE_VDEBUG("Override max link width to 8 lanes\n");
		apm_in32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_607_576__ADDR, &val);
		val &= ~FIELD_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_607_576_CFG_CONSTANTS_MAX_LINK_WIDTH_OVERRIDE_MASK;
		val |= 0x4;	/* max link width is 8 lanes */
		apm_out32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_607_576__ADDR, val);
		mps = 2;
	}

	PCIE_VDEBUG("Programming Port %d for only GEN%d\n", port->index,
		    port->link_speed + 1);
	apm_in32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_479_448__ADDR, &val);
	switch (port->link_speed)
	{
		case PCIE_GEN1:
			val &= ~FIELD_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_479_448_CFG_CONSTANTS_SUPPORT_5GTS_MASK;
			val &= ~FIELD_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_479_448_CFG_CONSTANTS_SUPPORT_8GTS_MASK;
			break;
		case PCIE_GEN2:
			val |= FIELD_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_479_448_CFG_CONSTANTS_DIRECT_TO_5GTS_MASK;
			val |= FIELD_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_479_448_CFG_CONSTANTS_SUPPORT_5GTS_MASK;
			val &= ~FIELD_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_479_448_CFG_CONSTANTS_SUPPORT_8GTS_MASK;
			break;
		case PCIE_GEN3:
			val |= FIELD_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_479_448_CFG_CONSTANTS_DIRECT_TO_8GTS_MASK;
			val |= FIELD_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_479_448_CFG_CONSTANTS_SUPPORT_5GTS_MASK;
			val |= FIELD_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_479_448_CFG_CONSTANTS_SUPPORT_8GTS_MASK;
			val &= ~FIELD_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_479_448_CFG_CONSTANTS_DIRECT_TO_5GTS_MASK;
			
			break;
		default:
			PCIE_ERR("Unsupported port link speed %d\n", port->link_speed);
	}
	apm_out32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_479_448__ADDR, val);

	if (port->link_speed != PCIE_GEN3) {
		PCIE_VDEBUG("Skipping Downstream equalization phase\n");
		apm_in32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_8G_CONSTANTS_CFG_8G_CONSTANTS_31_0__ADDR, &val);
		val |= FIELD_EXPRESSO_CFG_8G_CONSTANTS_CFG_8G_CONSTANTS_31_0_CFG_8G_CONSTANTS_DOWNSTREAM_EQ_SKIP_PHASE_2_3_MASK;
		apm_out32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_8G_CONSTANTS_CFG_8G_CONSTANTS_31_0__ADDR, val);
	}

	PCIE_VDEBUG("Initial max payload size : %d\n", mps);
	apm_in32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONTROL_CFG_CONTROL_63_32__ADDR, &val);
	val &= ~FIELD_EXPRESSO_CFG_CONTROL_CFG_CONTROL_63_32_CFG_CONTROL_PCIE_DEV_CAP_MAX_PAYLOAD_SIZE_SUPPORTED_MASK;
	val |= mps;
	apm_out32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONTROL_CFG_CONTROL_63_32__ADDR, val);
}

static void apm_pcie_program_core(void *csr_base)
{
	u32 val;

	PCIE_VDEBUG("Enable ECRC generation and checking\n");
	apm_in32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONTROL_CFG_CONTROL_31_0__ADDR, &val);
	val |= FIELD_EXPRESSO_CFG_CONTROL_CFG_CONTROL_31_0_CFG_CONTROL_AER_OPTIONAL_ERROR_EN_MASK;
	apm_out32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONTROL_CFG_CONTROL_31_0__ADDR, val);

	PCIE_VDEBUG("Enable all PCIE event, error generation "
		   "and interrupt status\n");
	apm_out32(csr_base + SM_PCIE_CSR_REGS_EVENTINTERRUPTSTATUSMASK__ADDR, 0xFFFFFEFF);
	apm_out32(csr_base + SM_PCIE_CSR_REGS_ERRORINTERRUPTSTATUSMASK__ADDR, 0x00000000);
	apm_out32(csr_base + SM_PCIE_CSR_REGS_INTXSTATUSMASK__ADDR, 0x00000000);

	apm_in32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONTROL_CFG_CONTROL_63_32__ADDR, &val);
	val &= ~0xffff;
	val |= 0x2F0F;
	PCIE_VDEBUG("Setting PCIE_DEV_CAP to 0x%08x\n", val);
	apm_out32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONTROL_CFG_CONTROL_63_32__ADDR, val);
}

/* Setup outbound memory and IO regions and AXI -> PCIe address translation */
static void apm_setup_outbound_regions(struct apm_pcie_port *port)
{
	struct apm_pcie_map_tbl *hb = port->pcie_map_tbl;
	int type = port->type;
	void *csr_base = port->csr_base;
	u64 val64;

	PCIE_DEBUG("***Setting up outbound regions for PCIe%d\n", port->index);
	/* Set up outbound PCIE translation Memory Region #1:
	 * Translate from host memory address to PCIE address
	 */
	val64 = 0;
	apm_out32(csr_base + SM_PCIE_CSR_REGS_OMR1BARL__ADDR, hb->ob_mem_addr[0].lo);
	apm_out32(csr_base + SM_PCIE_CSR_REGS_OMR1BARH__ADDR, hb->ob_mem_addr[0].hi);
	if (hb->ob_mem_addr[0].size)
		val64 = ~(hb->ob_mem_addr[0].size - 1) | PCI_ENABLE_REGION;
	apm_out32(csr_base + SM_PCIE_CSR_REGS_OMR1MSKL__ADDR, LODWORD(val64));
	apm_out32(csr_base + SM_PCIE_CSR_REGS_OMR1MSKH__ADDR, HIDWORD(val64));
	apm_out32(csr_base + SM_PCIE_CSR_REGS_POM1L__ADDR, hb->ob_mem_addr[0].pcie_lo);
	apm_out32(csr_base + SM_PCIE_CSR_REGS_POM1H__ADDR, hb->ob_mem_addr[0].pcie_hi);

	if (hb->ob_mem_addr[0].size) {
		PCIE_DEBUG("PCIE%d: Outbound 0 :\n"
			   "\tBAR	0x%08X_%08X ->\n"
			   "\tPOM	0x%08X_%08X\n"
			   "\tMask	0x%08X_%08X\n",
			   port->index,
			   (unsigned int)hb->ob_mem_addr[0].hi,
			   (unsigned int)hb->ob_mem_addr[0].lo,
			   (unsigned int)hb->ob_mem_addr[0].pcie_hi,
			   (unsigned int)hb->ob_mem_addr[0].pcie_lo,
			   (unsigned int)HIDWORD(val64),
			   (unsigned int)LODWORD(val64));
	}

	/* Set up outbound PCIE translation memory Region #2:
	 * Translate from host memory address (ie. 0xXXXX_XXXX) to PCIE
	 * address at 0x0000_0000 + size of 1st memory map.
	 */
	val64 = 0;
	apm_out32(csr_base + SM_PCIE_CSR_REGS_OMR2BARL__ADDR, hb->ob_mem_addr[1].lo);
	apm_out32(csr_base + SM_PCIE_CSR_REGS_OMR2BARH__ADDR, hb->ob_mem_addr[1].hi);
	if (hb->ob_mem_addr[1].size)
		val64 = ~(hb->ob_mem_addr[1].size - 1) | PCI_ENABLE_REGION;
	apm_out32(csr_base + SM_PCIE_CSR_REGS_OMR2MSKL__ADDR, LODWORD(val64));
	apm_out32(csr_base + SM_PCIE_CSR_REGS_OMR2MSKH__ADDR, HIDWORD(val64));
	apm_out32(csr_base + SM_PCIE_CSR_REGS_POM2L__ADDR, hb->ob_mem_addr[1].pcie_lo);
	apm_out32(csr_base + SM_PCIE_CSR_REGS_POM2H__ADDR, hb->ob_mem_addr[1].pcie_hi);

	if (hb->ob_mem_addr[1].size) {
		PCIE_DEBUG("PCIE%d: Outbound 1 :\n"
			   "\tBAR	0x%08X_%08X ->\n"
			   "\tPOM	0x%08X_%08X\n"
			   "\tMask	0x%08X_%08X\n",
			   port->index,
			   (unsigned int)hb->ob_mem_addr[1].hi,
			   (unsigned int)hb->ob_mem_addr[1].lo,
			   (unsigned int)hb->ob_mem_addr[1].pcie_hi,
			   (unsigned int)hb->ob_mem_addr[1].pcie_lo,
			   (unsigned int)HIDWORD(val64),
			   (unsigned int)LODWORD(val64));
	}

	if (type == PTYPE_ROOT_PORT) {
		/* Set up outbound PCIE translation I/O Region #3:
		 * Translate from host memory address (ie. 0xXXXX_XXXX) to PCIE
		 * address at 0x0000_0000 + size of 1st & 2nd memory map.
		 */
		val64 = 0;
		apm_out32(csr_base + SM_PCIE_CSR_REGS_OMR3BARL__ADDR, hb->ob_mem_addr[2].lo);
		apm_out32(csr_base + SM_PCIE_CSR_REGS_OMR3BARH__ADDR, hb->ob_mem_addr[2].hi);
		if (hb->ob_mem_addr[2].size)
			val64 = ~(hb->ob_mem_addr[2].size - 1) |
			    PCI_ENABLE_REGION | PCIE_OB_MASK_LO_IO;
		apm_out32(csr_base + SM_PCIE_CSR_REGS_OMR3MSKL__ADDR, LODWORD(val64));
		apm_out32(csr_base + SM_PCIE_CSR_REGS_OMR3MSKH__ADDR, HIDWORD(val64));
		apm_out32(csr_base + SM_PCIE_CSR_REGS_POM3L__ADDR, hb->ob_mem_addr[2].pcie_lo);
		apm_out32(csr_base + SM_PCIE_CSR_REGS_POM3H__ADDR, hb->ob_mem_addr[2].pcie_hi);

		if (hb->ob_mem_addr[2].size) {
			PCIE_DEBUG("PCIE%d: Outbound 2 :\n"
				   "\tBAR	0x%08X_%08X ->\n"
				   "\tPOM	0x%08X_%08X\n"
				   "\tMask	0x%08X_%08X\n",
				   port->index,
				   (unsigned int)hb->ob_mem_addr[2].hi,
				   (unsigned int)hb->ob_mem_addr[2].lo,
				   (unsigned int)hb->ob_mem_addr[2].pcie_lo,
				   (unsigned int)hb->ob_mem_addr[2].pcie_hi,
				   (unsigned int)HIDWORD(val64),
				   (unsigned int)LODWORD(val64));
		}
	} else {
		/* NO outbound PCIE translation I/O Region #3 for Endpoint */
		apm_out32(csr_base + SM_PCIE_CSR_REGS_OMR3BARL__ADDR, 0x000000);
		apm_out32(csr_base + SM_PCIE_CSR_REGS_OMR3BARH__ADDR, 0x000000);
		apm_out32(csr_base + SM_PCIE_CSR_REGS_OMR3MSKL__ADDR, 0x000000);
		apm_out32(csr_base + SM_PCIE_CSR_REGS_OMR3MSKH__ADDR, 0x000000);
		apm_out32(csr_base + SM_PCIE_CSR_REGS_POM3L__ADDR, 0x000000);
		apm_out32(csr_base + SM_PCIE_CSR_REGS_POM3H__ADDR, 0x000000);
	}

	/* No POM (or POM value is hardcoded to 0)
	 *      64’h0000_0000_0000_0000 –
	 *      64’h0000_0000_0000_3FFF
	 *      size: 2**14 (16 Kilo Byte)
	 */
	apm_out32(csr_base + SM_PCIE_CSR_REGS_MSGBARL__ADDR, hb->ob_msg_addr_lo);
	apm_out32(csr_base + SM_PCIE_CSR_REGS_MSGBARH__ADDR, hb->ob_msg_addr_hi);
	apm_out32(csr_base + SM_PCIE_CSR_REGS_MSGCTL__ADDR, 0x0);
#if 0
	PCIE_DEBUG("PCIE%d: Outbound Msg AHB 0x%08X_%08X\n",
		   portno,
		   (unsigned int)hb->ob_msg_addr_lo,
		   (unsigned int)hb->ob_msg_addr_hi);
#endif

	if (type == PTYPE_ROOT_PORT) {
		/* No POM (or POM value is hardcoded to 0)
		 *      64’h0000_0000_0000_0000 –
		 *      64’h0000_0000_0001_FFFF
		 *      size: 2**18 (256 Kilo Byte)
		 */
		val64 = 0;
		apm_out32(csr_base + SM_PCIE_CSR_REGS_CFGBARL__ADDR, hb->cfg_addr_lo);
		apm_out32(csr_base + SM_PCIE_CSR_REGS_CFGBARH__ADDR, hb->cfg_addr_hi);
		apm_out32(csr_base + SM_PCIE_CSR_REGS_CFGCTL__ADDR, 0x1);
		PCIE_DEBUG("PCIE%d: Outbound Config 0x%08X_%08X\n",
			   port->index,
			   (unsigned int)hb->cfg_addr_hi,
			   (unsigned int)hb->cfg_addr_lo);
	}
}

static u64 apm_pcie_setup_ib_mask(struct apm_pcie_port *port)
{
	struct apm_pcie_map_tbl *hb = port->pcie_map_tbl;
	void *csr_base = port->csr_base;
	u64 val64 = 0;
	u32 val32 = 0;

	if (hb->ib_mem_addr[0].pcie_size >= 1024)
		val64 = (~(hb->ib_mem_addr[0].pcie_size - 1) &
			PCI_BASE_ADDRESS_MEM_MASK) |
			PCI_BASE_ADDRESS_MEM_PREFETCH |
			PCI_BASE_ADDRESS_MEM_TYPE_64;	/* min is 1 KB */

	apm_in32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_159_128__ADDR, &val32);
	apm_out32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_159_128__ADDR,
		  (val32 & 0x0000ffff) | (LODWORD(val64) << 16));
	apm_in32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_191_160__ADDR, &val32);
	apm_out32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_191_160__ADDR,
		  (val32 & 0xffff0000) | (LODWORD(val64) >> 16));

	apm_in32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_191_160__ADDR, &val32);
	apm_out32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_191_160__ADDR,
		  (val32 & 0x0000ffff) | (HIDWORD(val64) << 16));
	apm_in32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_223_192__ADDR, &val32);
	apm_out32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_223_192__ADDR,
		  (val32 & 0xffff0000) | (HIDWORD(val64) >> 16));
	return val64;
}

/* Setup RC's inbound PCIe BARs and PCIe -> AXI address translation */
static void apm_setup_inbound_regions(struct apm_pcie_port *port)
{
	struct apm_pcie_map_tbl *hb = port->pcie_map_tbl;
	void *csr_base = port->csr_base;
	void *cfg_addr = port->cfg_base;
	int type = port->type;
	u64 val64 = 0;
	u32 val32 = 0;
	u32 val = 0;

	PCIE_DEBUG("***Setting up inbound regions for PCIe%d\n", port->index);

	val64 = apm_pcie_setup_ib_mask(port);
	/* Set up in bound PCIE Memory Region #1 */
	if (type == PTYPE_ROOT_PORT) {
		apm_out32(cfg_addr + PCI_BASE_ADDRESS_0,
				  (hb->ib_mem_addr[0].pcie_lo &
				   PCI_BASE_ADDRESS_MEM_MASK)		|
				   PCI_BASE_ADDRESS_MEM_PREFETCH	|
				   PCI_BASE_ADDRESS_MEM_TYPE_64);
		apm_out32(cfg_addr + PCI_BASE_ADDRESS_1,
			  hb->ib_mem_addr[0].pcie_hi);
		apm_in32(cfg_addr + PCI_BASE_ADDRESS_1, &val32);
		val32 = 0;
	}

	apm_out32(csr_base + SM_PCIE_CSR_REGS_PIM1_1L__ADDR, hb->ib_mem_addr[0].pim1_lo);

	/* Enable PCIe Cache Coherency */
	hb->ib_mem_addr[0].pim1_hi |=   FIELD_PIM1_1H_WR_PIM1_1_CACHE_WR(1) 		|
					FIELD_PIM1_1H_WR_PIM1_1_CACHE_OVERRIDE_WR(1)	|
					FIELD_PIM1_1H_RD_PIM1_1_CACHE_WR(1)		|
					FIELD_PIM1_1H_RD_PIM1_1_CACHE_OVERRIDE_WR(1);

	apm_out32(csr_base + SM_PCIE_CSR_REGS_PIM1_1H__ADDR, hb->ib_mem_addr[0].pim1_hi);
	apm_out32(csr_base + SM_PCIE_CSR_REGS_PIM1_2L__ADDR, hb->ib_mem_addr[0].pim2_lo);
	apm_out32(csr_base + SM_PCIE_CSR_REGS_PIM1_2H__ADDR, hb->ib_mem_addr[0].pim2_hi);
	apm_out32(csr_base + SM_PCIE_CSR_REGS_PIM1SL__ADDR, LODWORD(hb->ib_mem_addr[0].pim_size));
	apm_out32(csr_base + SM_PCIE_CSR_REGS_PIM1SH__ADDR, HIDWORD(hb->ib_mem_addr[0].pim_size));

	if (hb->ib_mem_addr[0].pcie_size >= 1024) {
		PCIE_DEBUG("PCIE%d: Inbound 0 : \n"
			   "\tBAR	0x%08X_%08X ->\n"
			   "\tPIM1_1	0x%08X_%08X\n"
			   "\tMask	0x%08X_%08X ->\n"
			   "\tPIM1_2	0x%08X_%08X\n"
			   "\tPIM Sz	0x%08X_%08X\n",
			   port->index, 
			   hb->ib_mem_addr[0].pcie_hi,
			   hb->ib_mem_addr[0].pcie_lo,
			   hb->ib_mem_addr[0].pim1_hi,
			   hb->ib_mem_addr[0].pim1_lo,
			   HIDWORD(val64),
			   LODWORD(val64),
			   hb->ib_mem_addr[0].pim2_hi,
			   hb->ib_mem_addr[0].pim2_lo,
			   HIDWORD(hb->ib_mem_addr[0].pim_size),
			   LODWORD(hb->ib_mem_addr[0].pim_size));
	}

	/* Set up in bound PCIE Memory Region #2 */
	val64 = 0;
	if (type == PTYPE_ROOT_PORT) {
		apm_out32(csr_base + SM_PCIE_CSR_REGS_IBAR3L__ADDR, hb->ib_mem_addr[1].pcie_lo);
		apm_out32(csr_base + SM_PCIE_CSR_REGS_IBAR3H__ADDR, hb->ib_mem_addr[1].pcie_hi);
		if (hb->ib_mem_addr[1].pcie_size >= (1024 * 1024))
			val64 = ~(hb->ib_mem_addr[1].pcie_size - 1)
				| PCI_ENABLE_REGION;	/* min is 1 MB */

		apm_out32(csr_base + SM_PCIE_CSR_REGS_IR3MSKL__ADDR, LODWORD(val64));
		apm_out32(csr_base + SM_PCIE_CSR_REGS_IR3MSKH__ADDR, HIDWORD(val64));
	} else {
		if (hb->ib_mem_addr[1].pcie_size >= (1024 * 1024))
			val64 = (~(hb->ib_mem_addr[1].pcie_size - 1) &
				PCI_BASE_ADDRESS_MEM_MASK)	|
				PCI_BASE_ADDRESS_MEM_PREFETCH	|
				PCI_BASE_ADDRESS_MEM_TYPE_64; /* min is 1 MB */
		apm_in32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_287_256__ADDR, &val32);
		apm_out32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_287_256__ADDR,
				(val32 & 0x0000ffff) | (LODWORD(val64) << 16));
		apm_in32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_319_288__ADDR, &val32);
		apm_out32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_319_288__ADDR,
				(val32 & 0xffff0000) | (LODWORD(val64) >> 16));

		apm_in32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_319_288__ADDR, &val32);
		apm_out32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_319_288__ADDR,
				(val32 & 0x0000ffff) | (HIDWORD(val64) << 16));
		apm_in32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_351_320__ADDR, &val32);
		apm_out32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_351_320__ADDR,
				(val32 & 0xffff0000) | (HIDWORD(val64) >> 16));

	}
	apm_out32(csr_base + SM_PCIE_CSR_REGS_PIM3_1L__ADDR, hb->ib_mem_addr[1].pim1_lo);
	apm_out32(csr_base + SM_PCIE_CSR_REGS_PIM3_1H__ADDR, hb->ib_mem_addr[1].pim1_hi);
	apm_out32(csr_base + SM_PCIE_CSR_REGS_PIM3_2L__ADDR, hb->ib_mem_addr[1].pim2_lo);
	apm_out32(csr_base + SM_PCIE_CSR_REGS_PIM3_2H__ADDR, hb->ib_mem_addr[1].pim2_hi);
	apm_out32(csr_base + SM_PCIE_CSR_REGS_PIM3SL__ADDR, LODWORD(hb->ib_mem_addr[1].pim_size));
	apm_out32(csr_base + SM_PCIE_CSR_REGS_PIM3SH__ADDR, HIDWORD(hb->ib_mem_addr[1].pim_size));
	if (hb->ib_mem_addr[1].pcie_size >= (1024 * 1024)) {
		PCIE_DEBUG("PCIE%d: Inbound 1 :\n"
			   "\tBAR	0x%08X_0x%08X ->\n"
			   "\tPIM3_1	0x%08X_0x%08X\n"
			   "\tMask	0x%08X\n"
			   "\tPIM3_2	0x%08X\n",
			   port->index,
			   hb->ib_mem_addr[1].pcie_hi,
			   hb->ib_mem_addr[1].pcie_lo,
			   hb->ib_mem_addr[1].pim1_hi,
			   hb->ib_mem_addr[1].pim1_lo,
			   LODWORD(val64), hb->ib_mem_addr[1].pim2_lo);
	}

	/* Set up inbound PCIE I/O Region: Not used */
	if (type == PTYPE_ROOT_PORT)
		apm_out32(csr_base + SM_PCIE_CSR_REGS_IR2MSK__ADDR, 0);
	else {
		apm_in32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_223_192__ADDR, &val32);
		apm_out32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_223_192__ADDR,
				val32 & 0x0000ffff);
		apm_in32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_255_224__ADDR, &val32);
		apm_out32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_255_224__ADDR,
				val32 & 0xffff0000);
		apm_out32(csr_base + SM_PCIE_CSR_REGS_PIM2S__ADDR, 0x0);
	}
	
	/* Expansion ROM Region #4 */
	apm_out32(csr_base + SM_PCIE_CSR_REGS_PIM4L__ADDR, hb->ib_rom_addr_lo);
	apm_out32(csr_base + SM_PCIE_CSR_REGS_PIM4H__ADDR, hb->ib_rom_addr_hi);
	val32 = 0;
	if (hb->ib_rom_addr_size >= (2 * 1024))	/* min 2KB */
		val32 = ~(hb->ib_rom_addr_size - 1);
	if (type == PTYPE_ROOT_PORT)
		apm_out32(cfg_addr + PCI_ROM_ADDRESS1, hb->ib_rom_addr_lo);
	apm_in32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_351_320__ADDR, &val);
	apm_out32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_351_320__ADDR,
		  (val & 0x0000ffff) | (val32 << 16));
	apm_in32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_383_352__ADDR, &val);
	apm_out32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_383_352__ADDR,
		  (val & 0xffff0000) | (val32 >> 16));
}

static u8 apm_poll_pcie_linkup(struct apm_pcie_port *port)
{
	void *csr_base = port->csr_base;
	int type = port->type;
	u32 val32, link_width, link_speed;
	u64 link_wait;

	PCIE_VDEBUG("Polling for Link with gen %d\n", port->link_speed + 1);
	/* Software is to poll dl_link_up which indicate that the LTSSM
	 * is ready to transmit or receive TLPs and enumeration can start.
	 */
	port->link_up = 0;
	if(type == PTYPE_ROOT_PORT) {
		for (link_wait = APM_PCIE_LINK_WAIT_US; link_wait > 0; link_wait--) {
			apm_in32(csr_base + SM_PCIE_CSR_REGS_PCIECORE_CTLANDSTATUS__ADDR, &val32);
			if (val32 & FIELD_PCIECORE_CTLANDSTATUS_S_LINK_UP_MASK) {
				port->link_up = 1;
				link_speed = FIELD_PCIECORE_CTLANDSTATUS_PIPE_PHY_RATE_RD(val32);
				apm_in32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_PCIE_STATUS_PCIE_STATUS_31_0__ADDR, &val32);
				link_width = val32 >> 26;
				PCIE_INFO("PCIE%d: (RC) X%d GEN-%d link up\n", port->index,
							link_width, link_speed + 1);
				break;
			}
		}
	}
	return !port->link_up;
}

/* Setup PCIe Port as Root Complex*/
static void apm_pcie_setup_root_complex(struct apm_pcie_port *port)
{
	void *csr_base = port->csr_base;
	u32 val = 0;

	PCIE_VDEBUG("Configuring class code\n");
	apm_in32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_63_32__ADDR, &val);
	val &= ~FIELD_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_63_32_CFG_CONSTANTS_CLASS_CODE_MASK;
	/* Class Code for PCIe Host Bridge [31:9] = 24'h06_0400 */
	val |= 0x06040000;
	apm_out32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_63_32__ADDR, val);

	PCIE_VDEBUG("Configuring switch mode and RP mode\n");
	apm_in32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_479_448__ADDR, &val);
	val |= FIELD_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_479_448_CFG_CONSTANTS_SWITCH_PORT_MODE_MASK;
	val &= ~FIELD_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_479_448_CFG_CONSTANTS_PM_FORCE_RP_MODE_MASK;
	val &= ~FIELD_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_479_448_CFG_CONSTANTS_ADVERTISE_INFINITE_CH_CD_CREDITS_AS_ROOT_PORT_MASK;
	apm_out32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_479_448__ADDR, val);

	apm_pcie_setup_link(port);

	PCIE_VDEBUG("Setting Device port type\n");
	apm_in32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONTROL_CFG_CONTROL_191_160__ADDR, &val);
	val &= ~FIELD_EXPRESSO_CFG_CONTROL_CFG_CONTROL_191_160_CFG_CONTROL_PCIE_CAP_DEVICE_PORT_TYPE_MASK;
	val = val | 0x05000000;
	apm_out32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONTROL_CFG_CONTROL_191_160__ADDR, val);

#ifdef CONFIG_PCI_MSI
	PCIE_VDEBUG("Enabling the msi capability\n");
	apm_in32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONTROL_CFG_CONTROL_447_416__ADDR, &val);
	val &= ~FIELD_EXPRESSO_CFG_CONTROL_CFG_CONTROL_447_416_CFG_CONTROL_MSI_CAPABILITY_DISABLE_MASK;
	val |= FIELD_EXPRESSO_CFG_CONTROL_CFG_CONTROL_447_416_CFG_CONTROL_MSI_MULTIPLE_MESSAGE_CAPABLE_MASK;
	apm_out32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONTROL_CFG_CONTROL_447_416__ADDR, val);
#endif
}

/* Setup PCIe Port as Endpoint */
static void apm_pcie_setup_endpoint(struct apm_pcie_port *port)
{
	void *csr_base = port->csr_base;
	u32 val = 0;

	apm_in32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_479_448__ADDR, &val);
	val &= ~FIELD_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_479_448_CFG_CONSTANTS_SWITCH_PORT_MODE_MASK;
	val &= ~FIELD_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_479_448_CFG_CONSTANTS_PM_FORCE_RP_MODE_MASK;
	val |= FIELD_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_479_448_CFG_CONSTANTS_ADVERTISE_INFINITE_CH_CD_CREDITS_AS_ROOT_PORT_MASK;
	apm_out32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_479_448__ADDR, val);

	PCIE_VDEBUG("Setting Device port type\n");
	apm_in32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONTROL_CFG_CONTROL_191_160__ADDR, &val);
	val &= ~FIELD_EXPRESSO_CFG_CONTROL_CFG_CONTROL_191_160_CFG_CONTROL_PCIE_CAP_DEVICE_PORT_TYPE_MASK;
	val &= ~FIELD_EXPRESSO_CFG_CONTROL_CFG_CONTROL_191_160_CFG_CONTROL_PCIE_CAP_SLOT_IMPLEMENTED_MASK;
	apm_out32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONTROL_CFG_CONTROL_191_160__ADDR, val);

	PCIE_VDEBUG("Configuring Vedor and Device IDs\n");
	val =  (CONFIG_SYS_PCI_SUBSYS_DEVICEID << 16) |
		CONFIG_SYS_PCI_SUBSYS_VENDORID;
	apm_out32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_31_0__ADDR, val);

	PCIE_VDEBUG("Configuring class code\n");
	apm_in32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_63_32__ADDR, &val);
	val &= FIELD_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_63_32_CFG_CONSTANTS_REVISION_ID_MASK;
	/* Class Code for Host Bridge [31:9] = 24'h0680_0000 */
	val |= 0x06800000;
	apm_out32(csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_63_32__ADDR, val);

	apm_pcie_setup_link(port);
}

/**
 * @brief  This function setups PCIE controller Root/End point for operation.
 * @param  port   PCIE controller port
 * @return 0 if successfull, < 0 if failed
 */
int apm_pcie_setup_port(struct apm_pcie_port *port)
{
	int type = port->type;
	u32 val32;

	apm_pcie_program_core(port->csr_base);
	/* Set bus numbers on our root port */
	if (type == PTYPE_ROOT_PORT)
		apm_pcie_setup_root_complex(port);
	else
		apm_pcie_setup_endpoint(port);
	
	apm_reset_pcie_core_clk(port);
	apm_setup_outbound_regions(port);
	apm_setup_inbound_regions(port);

/*
 * A component must enter the LTSSM Detect state within 20 ms of the end of
 * Fundamental Reset (Link Training is described in Section 4.2.4).
 * Note: In some systems, it is possible that the two components on a Link may
 *	 exit 10 Fundamental Reset at different times. Each component must
 *	 observe the requirement to enter the initial active Link Training state
 *	 within 20 ms of the end of Fundamental Reset from its own point of view.
 */
	mdelay(20);
	/* Enable I/O, Mem, and Busmaster cycles */
	if (type == PTYPE_ROOT_PORT) {
		apm_in32(port->cfg_base + PCI_COMMAND, &val32);
		apm_out32(port->cfg_base + PCI_COMMAND, val32	|
				PCI_COMMAND_IO		|
				PCI_COMMAND_MEMORY	|
				PCI_COMMAND_MASTER);
retry_link:
		/* Wait until PHY and Data Link are up */
		if (apm_poll_pcie_linkup(port)) {
			if (port->link_speed == PCIE_GEN1)
				goto give_up;
			port->link_speed--;
			apm_disable_pcie_core_clk(port);
			apm_pcie_setup_link(port);
			apm_reset_pcie_core_clk(port);
			mdelay(20);
			goto retry_link;
		}

give_up:
		if (!port->link_up) {
			PCIE_INFO("PCIE%d: (RC) link down\n", port->index);
		} else {
/* 
 * With a Downstream Port that supports Link speeds greater than 5.0 GT/s,
 * software must wait a minimum of 100 ms after Link training completes
 * before sending a Configuration Request to the device immediately below
 * that Port
 */
			mdelay(100);
		}
		return !port->link_up;
	}
	PCIE_INFO("PCIE%d: (EP)\n", port->index);
 	return 0;
}

#if 0
static void apm_pcie_hack_for_kc(void *base)
{
	u32 data;

	PCIE_VDEBUG("Bypassing the receiver detection and disabling L2 power mgmt - KC testchip bug.\n");
	apm_in32(base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_383_352__ADDR, &data);
	data |= FIELD_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_383_352_CFG_CONSTANTS_BYPASS_RECEIVER_DETECTION_MASK;
	data &= ~FIELD_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_383_352_CFG_CONSTANTS_ENABLE_L2_POWER_MGMT_MASK;
	apm_out32(base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_383_352__ADDR, data);
}
#endif

/* Initialize APM PCIe core and phy */
int apm_init_pcie(struct apm_pcie_port *port)
{
	u32 data;
	PCIE_DEBUG("***Initializing PCIe port = %d\n", port->index);

	if(is_apm_pcie_reset_done(port)) {
		PCIE_DEBUG("PCIE reset already done\n");
		return 0;
	}

	/* Reset and enable PCIE clock */
	if(apm_reset_pcie_clk(port))
		return -1;
	apm_pcie_init_phy(port);
	apm_pcie_init_ecc(port);
	apm_pcie_release_phy_reset(port->csr_base);
	apm_pcie_wait_pll_lock(port);
	apm_pcie_wait_phy_rdy(port->csr_base);
	apm_pcie_manual_calib(port->csr_base, port->link_width);
	
	apm_in32(port->csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_383_352__ADDR, &data);
	data = FIELD_NWL_PCIE_DMA_CFG_CONSTANTS_383_352_BYPASS_RECEIVER_DETECTION_SET(data, 0x1); 
	apm_out32(port->csr_base + NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_383_352__ADDR, data);
	return 0;
}
