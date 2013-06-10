/**
 * APM APM86xxx PCIe Header File
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
 * This module defines the CSR register, header types, and function
 * for PCIE module under Linux. It supports setup and configure the
 * PCIE controllers to work as root complex or endpoint.
 *
 */

#if !defined(__APM_PCIE_COMMON_H__)
#define __APM_PCIE_COMMON_H_
#include "apm88xxxx_pcie.h"
#include "apm88xxxx_pcie_serdes.h"

#define PCI_ENABLE_REGION	0x00000001
#define PCIE_OB_MASK_LO_IO	0x00000002
#define APM_PCIE_LINK_WAIT_US	(500*1000)

#define CONFIG_SYS_PCI_SUBSYS_VENDORID		0x19AA
#define CONFIG_SYS_PCI_SUBSYS_DEVICEID		0xCAFE
#define PIM_SIZE				0xffffffff00000000ULL

#ifndef HIDWORD
#	define MAKE_U64(h, l)		((((u64) (h)) << 32) | (l))
#	define HIDWORD(x)		((u32) (((u64)(x)) >> 32))
#	define LODWORD(x)		((u32) ((x) & 0xFFFFFFFF))
#endif

#define RES_SIZE(res)			((res)->end - (res)->start + 1)
#define PCI_ADDR_LO(h, l, O)	LODWORD((O) - MAKE_U64((h), (l)))
#define PCI_ADDR_HI(h, l, O)	HIDWORD((O) - MAKE_U64((h), (l)))

enum
{
	PTYPE_ENDPOINT		    	= 0x0,
	PTYPE_LEGACY_ENDPOINT		= 0x1,
	PTYPE_ROOT_PORT		    	= 0x4,

	LNKW_X1			        = 0x1,
	LNKW_X2			        = 0x2,
	LNKW_X4			        = 0x4,
	LNKW_X8			        = 0x8,
	LNKW_X16			= 0x10,

	PCIE_GEN1			= 0x0, /* 2.5G */
	PCIE_GEN2			= 0x1, /* 5.0G */
	PCIE_GEN3			= 0x2, /* 8.0G */
};

/**
 * @struct apm_pcie_map_tbl
 * @brief Host to PCIE translation address map
 * @note Although, pci_controller structure stores some of these values,
 *       it isn't complete. Therefore, we will keep a complete map here.
 *
 */
struct apm_pcie_map_tbl {
	u64 csr_addr;    		/* Host bridge CSR address */
	u32 csr_addr_size;

	/* Outbound AXI address for translation to PCIE */
	u32 cfg_addr_hi;    		/* Configuration AXI address for all */
	u32 cfg_addr_lo;		/* PCI devices */
	u32 cfg_addr_size;		/* use 0 for not used */
	u64 cfg_vaddr;			/* Virtual address */
	struct {
		u32 hi;   		/* Host address map and size */
		u32 lo;
		u64 size;
		u32 pcie_hi;   		/* PCIE address map  and size */
		u32 pcie_lo;
	} ob_mem_addr[3];		/* Inbound two memory and one IO regions */
	u32 ob_msg_addr_hi;		/* Message region address map */
	u32 ob_msg_addr_lo;
	u64 ob_msg_addr_size;		/* use 0 for not used */

	/* Inbound AXI address for translation from PCIE */
	struct {
		u32 pcie_hi;   		/* PCIE address map */
		u32 pcie_lo;
		u64 pcie_size; 		/* use 0 for not used */
		u32 pim1_hi;   		/* Host address map first half */
		u32 pim1_lo;
		u32 pim2_hi;   		/* To set R/W attribute, OR's them
					   with address */
		u32 pim2_lo;
		u64 pim_size;  		/* Host address map second half */
	} ib_mem_addr[2];		/* Inbound two memory regions */
	u32 ib_rom_addr_hi;		/* Inbound expansion rom map address */
	u32 ib_rom_addr_lo;
	u32 ib_rom_addr_size;  		/* use 0 for not used */
};

int apm_out32(void *addr, u32 val);
int apm_in32(void *addr, u32 *val);
int pcie_cfg_out32(void *addr, u32 val);
int pcie_cfg_out16(void *addr, u16 val);
int pcie_cfg_out8(void *addr, u8 val);
int pcie_cfg_in32(void *addr, u32 *val);
int pcie_cfg_in16(void *addr, u16 *val);
int pcie_cfg_in8(void *addr, u8 *val);
int apm_pcie_setup_port(struct apm_pcie_port *port);
int apm_init_pcie(struct apm_pcie_port *port);
void apm_pcie_setup_primary_bus(struct apm_pcie_port *port);
#endif /* __APM_PCIE_COMMON_H__ */
