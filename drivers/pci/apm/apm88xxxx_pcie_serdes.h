/**
 * APM APM86xxx PCIe Serdes Header File
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

#if !defined(__APM_PCIE_SERDES_H__)
#define __APM_PCIE_SERDES_H_
#include "apm88xxxx_pcie.h"

#define APM_PCIE_TIMEOUT        (500*1000)

enum {
	EXTERNAL_DIFFERENTIAL_CLK       = 0x0,
	INTERNAL_DIFFERENTIAL_CLK       = 0x1,
	INTERNAL_SINGLE_ENDED_CLK       = 0x2,
};

int apm_reset_pcie_clk(struct apm_pcie_port *port);
int apm_reset_pcie_core_clk(struct apm_pcie_port *port);
int apm_disable_pcie_core_clk(struct apm_pcie_port *port);
int is_apm_pcie_reset_done(struct apm_pcie_port *port);
void apm_pcie_init_ecc(struct apm_pcie_port *port);
void apm_pcie_wait_pll_lock(struct apm_pcie_port *port);
void apm_pcie_init_phy(struct apm_pcie_port *port);
void apm_pcie_release_phy_reset(void *base);
void apm_pcie_wait_phy_rdy(void *base);
void apm_pcie_manual_calib(void *base, u32 linkup_width);
#endif /* __APM_PCIE_SERDES_H__ */
