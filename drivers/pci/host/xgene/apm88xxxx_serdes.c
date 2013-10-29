/**
 * Interface for APM88XXXX Koolchip Serdes configuration
 *
 * Copyright (c) 2013, Applied Micro Circuits Corporation
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

#if defined(__U_BOOT__)
/* U-Boot */
#include <common.h>
#else
/* LINUX */
#include <linux/delay.h>
#endif

#include <asm/io.h>
#include "apm88xxxx_serdes.h"

#undef DEBUG_APM88XXX_SERDES

#ifdef DEBUG_APM88XXX_SERDES
#ifndef __U_BOOT__
# define SERDES_CSR_DEBUG(fmt, ...)	printk(KERN_INFO fmt, ##__VA_ARGS__);
#else
# define SERDES_CSR_DEBUG(fmt, ...)	printf(fmt, ##__VA_ARGS__);
#endif
#else
# define SERDES_CSR_DEBUG(fmt, ...)
#endif

static int apm_out32(void *addr, u32 val)
{
	writel(val, addr);
	return 0;
}

static int apm_in32(void *addr, u32 *val)
{
	*val = readl(addr);
	return 0;
}

void apm88xxxx_sds_wr_op(void *csr_base, uint32_t ind_cmd_reg_addr,
			uint32_t ind_wdata_reg_addr, uint32_t offset,
			uint32_t data)
{

	uint32_t cap_value;
	unsigned int ind_addr_cmd;

	ind_addr_cmd = (offset << 4) | 0x00000005;

	apm_out32(csr_base + ind_wdata_reg_addr, data);
	apm_out32(csr_base + ind_cmd_reg_addr, ind_addr_cmd);	// Write CMU_reg0 register
        SERDES_CSR_DEBUG("SERDES SDS INDIRECT WR: 0x%x value: 0x%08X\n", offset, data);
	cap_value = 0;
        udelay(1000);
	while (cap_value != 0x00000004) {
		apm_in32(csr_base + ind_cmd_reg_addr, &cap_value);
		cap_value = cap_value & 0x00000004;
	}


}

void apm88xxxx_sds_rd_op(void *csr_base, uint32_t ind_cmd_reg_addr,
			uint32_t ind_rdata_reg_addr, uint32_t offset,
			uint32_t * data)
{

	uint32_t cap_value;
	unsigned int ind_addr_cmd;

	ind_addr_cmd = (offset << 4) | 0x000000006;

	apm_out32(csr_base + ind_cmd_reg_addr, ind_addr_cmd);	// Read CMU_reg0 register
        udelay(1000);
	cap_value = 0;
	while (cap_value != 0x00000004) {
		apm_in32(csr_base + ind_cmd_reg_addr, &cap_value);
		cap_value = cap_value & 0x00000004;
	}

	apm_in32(csr_base + ind_rdata_reg_addr, data);
        SERDES_CSR_DEBUG("SERDES SDS INDIRECT RD: 0x%x value: 0x%08X\n", offset, *data);
}

void apm88xxxx_pcs_wr_op(void *csr_base, uint32_t ind_cmd_reg_addr,
			uint32_t ind_wdata_reg_addr, uint32_t offset,
			uint32_t data)
{

	uint32_t cap_value;
	unsigned int ind_addr_cmd;

	ind_addr_cmd = (offset << 4) | 0x00000005;
	ind_addr_cmd = ind_addr_cmd | ((1 << 16) << 4);	// Setting the 20th bit of ind_addr[21:4].

	apm_out32(csr_base + ind_wdata_reg_addr, data);
	apm_out32(csr_base + ind_cmd_reg_addr, ind_addr_cmd);	// Write CMU_reg0 register
        udelay(1000);
        SERDES_CSR_DEBUG("SERDES PCS INDIRECT WR: 0x%x value: 0x%08X\n", offset, data);
	cap_value = 0;
	while (cap_value != 0x00000004) {
		apm_in32(csr_base + ind_cmd_reg_addr, &cap_value);
		cap_value = cap_value & 0x00000004;
	}

}

void apm88xxxx_pcs_rd_op(void *csr_base, uint32_t ind_cmd_reg_addr,
			uint32_t ind_rdata_reg_addr, uint32_t offset,
			uint32_t * data)
{

	uint32_t cap_value;
	unsigned int ind_addr_cmd;

	ind_addr_cmd = (offset << 4) | 0x000000006;
	ind_addr_cmd = ind_addr_cmd | ((1 << 16) << 4);	// Setting the 20th bit of ind_addr[21:4].

	apm_out32(csr_base + ind_cmd_reg_addr, ind_addr_cmd);	// Read CMU_reg0 register
        udelay(1000);
	cap_value = 0;
	while (cap_value != 0x00000004) {
		apm_in32(csr_base + ind_cmd_reg_addr, &cap_value);
		cap_value = cap_value & 0x00000004;
	}

	apm_in32(csr_base + ind_rdata_reg_addr, data);
        SERDES_CSR_DEBUG("SERDES PCS INDIRECT RD: 0x%x value: 0x%08X\n", offset, *data);
}

void apm88xxxx_cm_wr_op(void *csr_base, uint32_t ind_cmd_reg_addr,
		       uint32_t ind_wdata_reg_addr, uint32_t offset,
		       uint32_t data)
{

	uint32_t cap_value;
	unsigned int ind_addr_cmd;

	ind_addr_cmd = (offset << 4) | 0x00000005;
	ind_addr_cmd = ind_addr_cmd | ((1 << 17) << 4);	// Setting the 21st bit of ind_addr[21:4].

	apm_out32(csr_base + ind_wdata_reg_addr, data);
	apm_out32(csr_base + ind_cmd_reg_addr, ind_addr_cmd);	// Write CMU_reg0 register
        udelay(1000);
        SERDES_CSR_DEBUG("SERDES CM INDIRECT WR: 0x%x value: 0x%08X\n", offset, data);
	cap_value = 0;
	while (cap_value != 0x00000004) {
		apm_in32(csr_base + ind_cmd_reg_addr, &cap_value);
		cap_value = cap_value & 0x00000004;
	}

	udelay(10);
}

void apm88xxxx_cm_rd_op(void *csr_base, uint32_t ind_cmd_reg_addr,
		       uint32_t ind_rdata_reg_addr, uint32_t offset,
		       uint32_t * data)
{

	uint32_t cap_value;
	unsigned int ind_addr_cmd;

	ind_addr_cmd = (offset << 4) | 0x000000006;
	ind_addr_cmd = ind_addr_cmd | ((1 << 17) << 4);	// Setting the 21st bit of ind_addr[21:4].

	apm_out32(csr_base + ind_cmd_reg_addr, ind_addr_cmd);	// Read CMU_reg0 register
        udelay(1000);
	cap_value = 0;
	while (cap_value != 0x00000004) {
		apm_in32(csr_base + ind_cmd_reg_addr, &cap_value);
		cap_value = cap_value & 0x00000004;
	}

	apm_in32(csr_base + ind_rdata_reg_addr, data);
        SERDES_CSR_DEBUG("SERDES CM INDIRECT RD: 0x%x value: 0x%08X\n", offset, *data);
}

void apm88xxxx_sds2_wr_op(void *csr_base, uint32_t ind_cmd_reg_addr,
			 uint32_t ind_wdata_reg_addr, uint32_t offset,
			 uint32_t data)
{

	uint32_t cap_value;
	unsigned int ind_addr_cmd;

	ind_addr_cmd = (offset << 4) | 0x00000005;
	ind_addr_cmd = ind_addr_cmd | (3 * ((1 << 16) << 4));	// Setting the 20th n 21st bit of ind_addr[21:4].

	apm_out32(csr_base + ind_wdata_reg_addr, data);
	apm_out32(csr_base + ind_cmd_reg_addr, ind_addr_cmd);	// Write CMU_reg0 register
        SERDES_CSR_DEBUG("SERDES SDS2 INDIRECT WR: 0x%x value: 0x%08X\n", offset, data);
	cap_value = 0;
        udelay(1000);
	while (cap_value != 0x00000004) {
		apm_in32(csr_base + ind_cmd_reg_addr, &cap_value);
		cap_value = cap_value & 0x00000004;
	}

}

void apm88xxxx_sds2_rd_op(void *csr_base, uint32_t ind_cmd_reg_addr,
			 uint32_t ind_rdata_reg_addr, uint32_t offset,
			 uint32_t * data)
{

	uint32_t cap_value;
	unsigned int ind_addr_cmd;

	ind_addr_cmd = (offset << 4) | 0x000000006;
	ind_addr_cmd = ind_addr_cmd | (3 * ((1 << 16) << 4));	// Setting the 20th n 21st bit of ind_addr[21:4].

	apm_out32(csr_base + ind_cmd_reg_addr, ind_addr_cmd);	// Read CMU_reg0 register
        udelay(1000);
	cap_value = 0;
	while (cap_value != 0x00000004) {
		apm_in32(csr_base + ind_cmd_reg_addr, &cap_value);
		cap_value = cap_value & 0x00000004;
	}

	apm_in32(csr_base + ind_rdata_reg_addr, data);
        SERDES_CSR_DEBUG("SERDES SDS2 INDIRECT RD: 0x%x value: 0x%08X\n", offset, *data);
}
