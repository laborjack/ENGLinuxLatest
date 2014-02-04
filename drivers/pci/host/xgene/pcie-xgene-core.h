/**
 * Appled Micro X-Gene PCIe core header file
 *
 * Copyright (c) 2013 Applied Micro Circuits Corporation.
 *
 * Author: Tanmay Inamdar <tinamdar@apm.com>.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#if !defined(__XGENE_PCIE_CORE_H__)
#define __XGENE_PCIE_CORE_H__
#include "pcie-xgene.h"

#define PCIECORE_LTSSM			0x4c
#define PCIECORE_CTLANDSTATUS		0x50
#define POWERMANAGEMENTREG		0x54
#define EVENTINTERRUPTSTATUS		0x58
#define EVENTINTERRUPTSTATUSMASK	0x5c
#define ERRORINTERRUPTSTATUS		0x60
#define ERRORINTERRUPTSTATUSMASK	0x64
#define INTXSTATUSMASK			0x6c
#define PCIECORE_CTLANDSTATUS		0x50
#define PIM1_1L				0x80
#define PIM2_1L				0xa0
#define PIM3_1L				0xc4
#define PIM4L				0xdc
#define OMR1BARL			0x100
#define CFGBARL				0x154
#define CFGBARH				0x158
#define CFGCTL				0x15c
#define RTDID				0x160
#define CFG_CONSTANTS_31_00		0x2000
#define CFG_CONSTANTS_63_32		0x2004
#define CFG_CONSTANTS_159_128		0x2010
#define CFG_CONSTANTS_351_320		0x2028
#define CFG_CONSTANTS_479_448		0x2038
#define CFG_CONSTANTS_607_576		0x2048
#define CFG_8G_CONSTANTS_31_0		0x2100
#define CFG_CONTROL_31_00		0x2200
#define CFG_CONTROL_63_32		0x2204
#define CFG_CONTROL_191_160		0x2214
#define CFG_CONTROL_447_416		0x2234
#define PCIE_STATUS_31_0		0x2600

#define PCI_PRIMARY_BUS_MASK		0x00ffffff
#define REVISION_ID_MASK		0x000000ff
#define SLOT_IMPLEMENTED_MASK		0x04000000
#define DEVICE_PORT_TYPE_MASK		0x03c00000
#define ADVT_INFINITE_CREDITS		0x00000200
#define PM_FORCE_RP_MODE_MASK		0x00000400
#define SWITCH_PORT_MODE_MASK		0x00000800
#define MSI_CAPABLE_MASK		0x001c0000
#define MSIX_CAP_DISABLE_MASK		0x00020000
#define MSI_CAP_DISABLE_MASK		0x00010000
#define CLASS_CODE_MASK			0xffffff00
#define LINK_UP_MASK			0x00000100
#define AER_OPTIONAL_ERROR_EN		0xffc00000
#define MAX_PAYLD_SZ_SUP_MASK		0x00000007
#define DWNSTRM_EQ_SKP_PHS_2_3		0x00010000
#define DIRECT_TO_5GTS_MASK		0x00020000
#define SUPPORT_5GTS_MASK		0x00010000
#define DIRECT_TO_8GTS_MASK		0x00008000
#define SUPPORT_8GTS_MASK		0x00004000
#define MAX_LNK_WDT_OVRRD_MASK		0x00000007
#define XGENE_PCIE_DEV_CTRL		0x2f0f
#define AXI_EP_CFG_ACCESS		0x10000
#define AXI_EP_DMA_ACCESS		0x20000
#define XGENE_PORT_TYPE_RC		0x05000000

#define EN_COHERENCY			0xF0000000
#define EN_REG				0x00000001
#define OB_LO_IO			0x00000002
#define XGENE_PCIE_LINK_WAIT		(500*1000)
#define XGENE_PCIE_MAX_REGIONS		3
#define XGENE_PCIE_VENDORID		0x19AA
#define XGENE_PCIE_BRIDGE_DEVICEID	0xE008
#define XGENE_PCIE_DEVICEID		0xCAFE
#define PIM_SIZE			0xffffffff00000000ULL

enum {
	PTYPE_ENDPOINT = 0x0,
	PTYPE_LEGACY_ENDPOINT = 0x1,
	PTYPE_ROOT_PORT = 0x4,

	LNKW_X1 = 0x1,
	LNKW_X2 = 0x2,
	LNKW_X4 = 0x4,
	LNKW_X8 = 0x8,
	LNKW_X16 = 0x10,

	PCIE_GEN1 = 0x0,	/* 2.5G */
	PCIE_GEN2 = 0x1,	/* 5.0G */
	PCIE_GEN3 = 0x2,	/* 8.0G */
};

/* Host to PCIE translation address map */
struct ob_mem_addr {
	u32 hi;
	u32 lo;
	u64 size;
	u32 pcie_hi;
	u32 pcie_lo;
};

struct ib_mem_addr {
	u32 pcie_hi;
	u32 pcie_lo;
	u64 pcie_size;
	u32 pim1_hi;
	u32 pim1_lo;
	u32 pim2_hi;
	u32 pim2_lo;
	u64 pim_size;
};

struct xgene_pcie_map_tbl {
	u64 csr_addr;
	u32 csr_addr_size;
	u32 cfg_addr_hi;
	u32 cfg_addr_lo;
	u32 cfg_addr_size;
	u64 cfg_vaddr;
	struct ob_mem_addr ob_mem_addr[XGENE_PCIE_MAX_REGIONS];
	u32 ob_msg_addr_hi;
	u32 ob_msg_addr_lo;
	u64 ob_msg_addr_size;
	struct ib_mem_addr ib_mem_addr[XGENE_PCIE_MAX_REGIONS];
	u32 ib_rom_addr_hi;
	u32 ib_rom_addr_lo;
	u32 ib_rom_addr_size;
};

int xgene_pcie_out32(void *addr, u32 val);
int xgene_pcie_in32(void *addr, u32 *val);
int xgene_pcie_cfg_out32(void *addr, u32 val);
int xgene_pcie_cfg_out16(void *addr, u16 val);
int xgene_pcie_cfg_out8(void *addr, u8 val);
int xgene_pcie_cfg_in32(void *addr, u32 *val);
int xgene_pcie_cfg_in16(void *addr, u16 *val);
int xgene_pcie_cfg_in8(void *addr, u8 *val);
#endif /* __XGENE_PCIE_CORE_H__ */
