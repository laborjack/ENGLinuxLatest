/**
 * APM STORM NWL PCIe PHY / Serdes initialization
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
 * @file   apm88xxxx_pcie_serdes.c
 *
 * This module implements PCIe functionality for APM STORM SoC NWL.
 * This setups and configures PCIe controllers as either root complex
 * or endpoint.
 */
#include "apm88xxxx_pcie_serdes.h"

extern int apm_out32(void *addr, u32 val);
extern int apm_in32(void *addr, u32 *val);

static void pcie_phy_csr_write(void *base, u32 addr, u32 data)
{
	u32 cap_value;
	u32 ind_addr_cmd;

	ind_addr_cmd = (addr << FIELD_PCIE_SDS_IND_CMD_REG_CFG_IND_ADDR_SHIFT_MASK) |
			FIELD_PCIE_SDS_IND_CMD_REG_CFG_IND_WR_CMD_MASK;
	ind_addr_cmd |= FIELD_PCIE_SDS_IND_CMD_REG_CFG_IND_CMD_DONE_MASK;

	apm_out32(base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_IND_WDATA_REG__ADDR, data);
	apm_out32(base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_IND_CMD_REG__ADDR, ind_addr_cmd);

	do {
		apm_in32(base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_IND_CMD_REG__ADDR, &cap_value);
	} while ((cap_value & FIELD_PCIE_SDS_IND_CMD_REG_CFG_IND_CMD_DONE_MASK)
		 != FIELD_PCIE_SDS_IND_CMD_REG_CFG_IND_CMD_DONE_MASK);
//	PCIE_VDEBUG("PHY CSR WRITE ADDR 0x%x DATA is 0x%x \n", addr, data);
}

static void pcie_phy_csr_read(void *base, u32 addr, u32 *data)
{
	u32 cap_value;
	u32 ind_addr_cmd;

	ind_addr_cmd = (addr << FIELD_PCIE_SDS_IND_CMD_REG_CFG_IND_ADDR_SHIFT_MASK) |
			FIELD_PCIE_SDS_IND_CMD_REG_CFG_IND_RD_CMD_MASK;
	ind_addr_cmd |= FIELD_PCIE_SDS_IND_CMD_REG_CFG_IND_CMD_DONE_MASK;
	apm_out32(base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_IND_CMD_REG__ADDR, ind_addr_cmd);

	do {
		apm_in32(base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_IND_CMD_REG__ADDR, &cap_value);
	} while ((cap_value & FIELD_PCIE_SDS_IND_CMD_REG_CFG_IND_CMD_DONE_MASK)
		 != FIELD_PCIE_SDS_IND_CMD_REG_CFG_IND_CMD_DONE_MASK);

	apm_in32(base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_IND_RDATA_REG__ADDR, data);
//	PCIE_VDEBUG("PHY CSR READ ADDR 0x%x DATA is 0x%x \n", addr, *data);
}

static void apm_init_serdes_refclk(struct apm_pcie_port *port)
{
	void *csr_base = port->csr_base;
	int type = port->type;
	int pll_rdy =	FIELD_PCIE_CLK_MACRO_REG_O_PLL_LOCK_MASK |
			FIELD_PCIE_CLK_MACRO_REG_O_PLL_READY_MASK;
	int timeout = APM_PCIE_TIMEOUT;
	u32 val;

	switch (port->serdes_clk) {
	case EXTERNAL_DIFFERENTIAL_CLK:
		PCIE_VDEBUG("Using EXTERNAL_DIFFERENTIAL_CLK\n");
		if (port->link_width == 8) {
		/* The external differential clks will be passed through the clktree,
		 * before feeding the second x4 SERDES, serdes_x4_2i for the x8 mode.
		 * For that module, this clk source will appear as refclk2
		 */
			/* Method 2 */
			apm_in32(csr_base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_CTL0__ADDR, &val);
			val &= ~0x1000;
			apm_out32(csr_base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_CTL0__ADDR, val);
			pcie_phy_csr_read(csr_base, KC_SERDES2_CMU_REGS_CMU_REG0__ADDR, &val);
			val = FIELD_CMU_REG0_PLL_REF_SEL_SET(val, 0x1);
			pcie_phy_csr_write(csr_base, KC_SERDES2_CMU_REGS_CMU_REG0__ADDR, val);
		}
		break;

	case INTERNAL_SINGLE_ENDED_CLK:
		PCIE_VDEBUG("Using INTERNAL_SINGLE_ENDED_CLK\n");
		/* Take the pciemacro out of reset */
		apm_in32(csr_base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_CLK_MACRO_REG__ADDR, &val);
		val |= FIELD_PCIE_CLK_MACRO_REG_I_RESET_B_MASK;
		if (type == 0x4) /* ROOT PORT */
			val |= FIELD_PCIE_CLK_MACRO_REG_I_OE_MASK;
		apm_out32(csr_base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_CLK_MACRO_REG__ADDR, val);
	
		PCIE_VDEBUG("Enabling the SSC for the CLKMACRO\n");
		pcie_phy_csr_read(csr_base, KC_CLKMACRO_CMU_REGS_CMU_REG36__ADDR, &val);
		val |= FIELD_CMU_REG36_PLL_SSC_EN_MASK;
		pcie_phy_csr_write(csr_base, KC_CLKMACRO_CMU_REGS_CMU_REG36__ADDR, val);

		/* Poll to ensure that pciemacro is ready */
		PCIE_VDEBUG("Poll to ensure that pciemacro is ready\n");
		do {
			apm_in32(csr_base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_CLK_MACRO_REG__ADDR, &val);
			timeout--;
		} while (((val & pll_rdy) != pll_rdy) && timeout);

		if(!timeout)
			PCIE_VDEBUG("poll pciemacro ready - Timeout\n");

		/* Select internal cmos clk */
		apm_in32(csr_base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_CTL0__ADDR, &val);
		val |= FIELD_PCIE_SDS_CTL0_CFG_I_REFCLK_CMOS_SEL_MASK | 0x1000;
		apm_out32(csr_base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_CTL0__ADDR, val);
		break;
	default:
		PCIE_ERR("Unsupported SERDES CLOCK for PCIe\n");
	}

#if 0
	PCIE_VDEBUG("Enabling the SSC for the SERDES\n");
	pcie_phy_csr_read(csr_base, KC_SERDES_CMU_REGS_CMU_REG36__ADDR, &val);
	val |= FIELD_CMU_REG36_PLL_SSC_EN_MASK;
	pcie_phy_csr_write(csr_base, KC_SERDES_CMU_REGS_CMU_REG36__ADDR, val);

	if(port->link_width == 8) {
		PCIE_VDEBUG("Enabling the SSC for the SERDES2\n");
		pcie_phy_csr_read(csr_base, KC_SERDES2_CMU_REGS_CMU_REG36__ADDR, &val);
		val |= FIELD_CMU_REG36_PLL_SSC_EN_MASK;
		pcie_phy_csr_write(csr_base, KC_SERDES2_CMU_REGS_CMU_REG36__ADDR, val);
	}
#endif
}

void apm_pcie_serdes_pipe_config(void *base)
{
	u32 data, i;

	//increase receiver detect timeout between serdes and PIPE
	pcie_phy_csr_read(base, KC_PIPE_REGS_RECEIVE_DETECT_CONTROL__ADDR, &data);
	data = FIELD_RECEIVE_DETECT_CONTROL_RX_PRES_ONE_CNT_TH_SET(data, 0x1388);
	pcie_phy_csr_write(base, KC_PIPE_REGS_RECEIVE_DETECT_CONTROL__ADDR, data);

	//increase receiver detect timeout window between serdes and PIPE
	pcie_phy_csr_read(base, KC_PIPE_REGS_RECEIVE_DETECT_CONTROL__ADDR, &data);
	data = FIELD_RECEIVE_DETECT_CONTROL_RX_PRES_ONE_CNT_TH_SET(data, 0x1388); //10us max
	pcie_phy_csr_write(base, KC_PIPE_REGS_RECEIVE_DETECT_CONTROL__ADDR, data);

	pcie_phy_csr_read(base, KC_PIPE_REGS_USB_PCIE_CTRL__ADDR, &data);
	data = FIELD_USB_PCIE_CTRL_RX_PRES_ONE_CNT_CMP_TH_SET(data, 0x64); //200ns min
	pcie_phy_csr_write(base, KC_PIPE_REGS_USB_PCIE_CTRL__ADDR, data);

	for (i = 0; i < 8; i++) {
		//CDR SEL override
		pcie_phy_csr_read(base, KC_PIPE_REGS_SERDES_DFE_CONTROL0__ADDR + (i *4), &data);
		data = FIELD_SERDES_DFE_CONTROL0_P2S_I_SPD_SEL_CDR_OVR_LN_SET(data, 0x7);
		pcie_phy_csr_write(base, KC_PIPE_REGS_SERDES_DFE_CONTROL0__ADDR + (i *4), data);
	}

	//increase tx amp, channel 0 bit location is different from rest of channels
	pcie_phy_csr_read(base, KC_PIPE_REGS_SERDES_CONTROL3__ADDR, &data);
	data = FIELD_SERDES_CONTROL3_P2S_I_TX_AMP_EN_LN0_SET(data, 0x1);
	data = FIELD_SERDES_CONTROL3_P2S_I_TX_AMP_LN0_SET(data, 0xf);
	pcie_phy_csr_write(base, KC_PIPE_REGS_SERDES_CONTROL3__ADDR, data);

	for (i = 0; i < 7; i++) {
		//increase tx amp
		pcie_phy_csr_read(base, KC_PIPE_REGS_SERDES_CONTROL4__ADDR + (i *4), &data);
		data = FIELD_SERDES_CONTROL4_P2S_I_TX_AMP_EN_LN1_SET(data, 0x1);
		data = FIELD_SERDES_CONTROL4_P2S_I_TX_AMP_LN1_SET(data, 0xf);
		pcie_phy_csr_write(base, KC_PIPE_REGS_SERDES_CONTROL4__ADDR + (i *4) , data);
	}
}

void apm_pcie_init_ecc(struct apm_pcie_port *port)
{
	void *csr_base = port->csr_base;
	int timeout = APM_PCIE_TIMEOUT;
	u32 val;

	PCIE_VDEBUG("Intialize PCIe ECC\n");
	apm_out32(csr_base + SM_GLBL_DIAG_CSR_CFG_MEM_RAM_SHUTDOWN__ADDR, 0x0);
	apm_in32(csr_base + SM_GLBL_DIAG_CSR_CFG_MEM_RAM_SHUTDOWN__ADDR, &val);
	PCIE_VDEBUG("MEM_RAM_SHUTDOWN Data : 0x%08x\n", val);

	apm_in32(csr_base + SM_GLBL_DIAG_CSR_BLOCK_MEM_RDY__ADDR, &val);
	while ((val != 0xFFFFFFFF) && timeout) {
		timeout--;
		apm_in32(csr_base + SM_GLBL_DIAG_CSR_BLOCK_MEM_RDY__ADDR, &val);
	}
	if (!timeout)
		PCIE_VDEBUG("PCIe ECC init timeout\n");
}

/* Reset PCIe Core clock */
int apm_reset_pcie_core_clk(struct apm_pcie_port *port)
{	
	void *csr_base = port->csr_base;
	u32 val;

	PCIE_VDEBUG("Enable Clock for CORE to enable link up\n");
	apm_in32(csr_base + SM_PCIE_CLKRST_CSR_PCIE_CLKEN__ADDR, &val);
	if (!(val & FIELD_PCIE_CLKEN_CORE_CLKEN_MASK)) {
		val |= FIELD_PCIE_CLKEN_CORE_CLKEN_MASK;
		apm_out32(csr_base + SM_PCIE_CLKRST_CSR_PCIE_CLKEN__ADDR, val);
	}

	PCIE_VDEBUG("Deasserting the core reset\n");
	apm_in32(csr_base + SM_PCIE_CLKRST_CSR_PCIE_SRST__ADDR, &val);
	if(val & FIELD_PCIE_SRST_CORE_RESET_MASK) {
		val &= ~FIELD_PCIE_SRST_CORE_RESET_MASK;
		apm_out32(csr_base + SM_PCIE_CLKRST_CSR_PCIE_SRST__ADDR, val);
	}

	return 0;
}

int apm_disable_pcie_core_clk(struct apm_pcie_port *port)
{	
	void *csr_base = port->csr_base;
	u32 val;

	PCIE_VDEBUG("Disabled Clock for CORE to enable link up\n");
	apm_in32(csr_base + SM_PCIE_CLKRST_CSR_PCIE_CLKEN__ADDR, &val);
	val &= ~FIELD_PCIE_CLKEN_CORE_CLKEN_MASK;
	apm_out32(csr_base + SM_PCIE_CLKRST_CSR_PCIE_CLKEN__ADDR, val);

	PCIE_VDEBUG("asserting the core reset\n");
	apm_in32(csr_base + SM_PCIE_CLKRST_CSR_PCIE_SRST__ADDR, &val);
	val |= FIELD_PCIE_SRST_CORE_RESET_MASK;
	apm_out32(csr_base + SM_PCIE_CLKRST_CSR_PCIE_SRST__ADDR, val);

	return 0;
}

int is_apm_pcie_reset_done(struct apm_pcie_port *port)
{
	void *csr_base = port->csr_base;
	u32 val;

	apm_in32(csr_base + SM_PCIE_CLKRST_CSR_PCIE_CLKEN__ADDR, &val);
	PCIE_VDEBUG("%s: CLKRST_CSR_PCIE_CLKEN - 0x%08x\n", __func__, val);
	if (val == 0xf)
		return 1;
	return 0;
}

/* Reset PCIe clock */
int apm_reset_pcie_clk(struct apm_pcie_port *port) 
{	
	void *csr_base = port->csr_base;
	int portnum = port->index;
	u32 val;

	apm_out32(csr_base + SM_PCIE_CLKRST_CSR_PCIE_CLKEN__ADDR, 0x00000000);
	apm_out32(csr_base + SM_PCIE_CLKRST_CSR_PCIE_SRST__ADDR, 0xFFFFFFFF);

	PCIE_VDEBUG("Enable Clock for CSR, AXI, APB\n");
	apm_in32(csr_base + SM_PCIE_CLKRST_CSR_PCIE_CLKEN__ADDR, &val);
	val |=  FIELD_PCIE_CLKEN_APB_CORE_CLKEN_MASK |
		FIELD_PCIE_CLKEN_AXI_CORE_CLKEN_MASK |
		FIELD_PCIE_CLKEN_CSR_CLKEN_MASK;
	apm_out32(csr_base + SM_PCIE_CLKRST_CSR_PCIE_CLKEN__ADDR, val);

	apm_in32(csr_base + SM_PCIE_CLKRST_CSR_PCIE_SRST__ADDR, &val);

	val &=  ~FIELD_PCIE_SRST_CSR_RESET_MASK		&
		~FIELD_PCIE_SRST_AXI_CORE_RESET_MASK	&
		~FIELD_PCIE_SRST_APB_CORE_RESET_MASK;

	apm_out32(csr_base + SM_PCIE_CLKRST_CSR_PCIE_SRST__ADDR, val);

	if (port->link_width == 8) {
		if (!((portnum == 0) || (portnum == 3))) {
			PCIE_WARN("incorrect link width for port %d\n",
				port->index);
			return -1;
		}
		PCIE_VDEBUG("Enabling x8 for controller %d\n", portnum);
		/* enable x8 .. default is x4 */
		apm_in32(csr_base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_MUX__ADDR, &val);
		val = FIELD_PCIE_SDS_MUX_SEL_PCIE_FULL_WIDTH_SET(val, 0x1);
		apm_out32(csr_base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_MUX__ADDR, val);
	}

	return 0;
}

void apm_pcie_release_phy_reset(void *base)
{
	u32 data;

	PCIE_VDEBUG("Deasserting the PHY reset now.\n");
	apm_in32(base + SM_PCIE_CLKRST_CSR_PCIE_SRST__ADDR, &data);
	data &= ~FIELD_PCIE_SRST_PHY_SDS_RESET_MASK;
	apm_out32(base + SM_PCIE_CLKRST_CSR_PCIE_SRST__ADDR, data);
}

void apm_pcie_wait_phy_rdy(void *base)
{
	u32 data;

	PCIE_VDEBUG("Waiting for the PHY to be ready\n");
	do {
		apm_in32(base + SM_PCIE_CSR_REGS_PCIECORE_CTLANDSTATUS__ADDR, &data);
		data &= FIELD_PCIECORE_CTLANDSTATUS_PIPE_PHY_STATUS_MASK;
		PCIE_VDEBUG("Waiting for the PHY to be ready\n");
	} while(data != 0); 
	PCIE_VDEBUG("PHY_STATUS deasserted. PHY is ready now.\n");
}; 

void apm_pcie_wait_pll_lock(struct apm_pcie_port *port)
{
	void *base = port->csr_base;
	u32 linkwidth = port->link_width;
	u32 data;
	u32 lock = 0;
	u32 lock1 = 0;

	if (linkwidth != 8) {
		do {
			apm_in32(base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS0_CMU_STATUS0__ADDR, &data);
			lock = data & 0x7;
			PCIE_VDEBUG("Poll PLL lock, reg read data 0x%x \n", data);
		} while (lock == 0);
	} else {
		do {
			apm_in32(base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS0_CMU_STATUS0__ADDR, &data);
			lock = data & 0x7;
			PCIE_VDEBUG("Poll PLL lock on Serdes 0, reg read data 0x%x \n", data);

			apm_in32(base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS1_CMU_STATUS0__ADDR, &data);
			lock1 = data & 0x7;
			PCIE_VDEBUG("Poll PLL lock on Serdes 1, reg read data 0x%x \n", data);
		} while (lock == 0 && lock1 == 0);
	}
}

//TX control
void serdes_config_tx_control(void *base, u32 ch, u32 sds2)
{
	u32 data;
	int sds2_offset = (sds2 > 0) ? 0x30000 : 0x0;

	PCIE_VDEBUG("Serdes TX config, channel# 0x%x, serdes# 0x%x sds2_offset# 0x%x\n",
			ch, sds2, sds2_offset);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG2__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG2_TX_FIFO_ENA_SET(data, 0x1);
	data = FIELD_CH0_RXTX_REG2_RESETB_TXD_SET(data, 0x1);
	data = FIELD_CH0_RXTX_REG2_BIST_ENA_TX_SET(data, 0x0);
	data = FIELD_CH0_RXTX_REG2_TX_INV_SET(data, 0x0);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG2__ADDR + (0x200 * ch) + sds2_offset, data);

	//130212
	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG6__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG6_TXAMP_ENA_SET(data, 0x1);
	data = FIELD_CH0_RXTX_REG6_TXAMP_CNTL_SET(data, 0xf);
	data = FIELD_CH0_RXTX_REG6_TX_IDLE_SET(data, 0x0);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG6__ADDR + (0x200 * ch) + sds2_offset, data);

	//130212
	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG5__ADDR + (0x200 * ch) + sds2_offset, &data);
	//data = FIELD_CH0_RXTX_REG5_TX_CN2_SET(data, 0x5);
	data = FIELD_CH0_RXTX_REG5_TX_CN2_SET(data, 0x0);
	data = FIELD_CH0_RXTX_REG5_TX_CP1_SET(data, 0xf);
	data = FIELD_CH0_RXTX_REG5_TX_CN1_SET(data, 0x2);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG5__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG4__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG4_TX_LOOPBACK_BUF_EN_SET(data, 0x0);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG4__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG145__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG145_TX_IDLE_SATA_SET(data, 0x0);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG145__ADDR + (0x200 * ch) + sds2_offset, data);
}

//RX control
void serdes_config_rx_control(void *base, u32 ch, u32 sds2)
{
	u32 data;
	int sds2_offset = (sds2 > 0) ? 0x30000 : 0x0;

	PCIE_VDEBUG("Serdes RX config, channel# 0x%x, serdes# 0x%x sds2_offset# 0x%x\n",
			ch, sds2, sds2_offset);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG2__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG2_RESETB_TERM_SET(data, 0x0);  // is this correct ???
	data = FIELD_CH0_RXTX_REG2_VTT_ENA_SET(data, 0x1);
	data = FIELD_CH0_RXTX_REG2_VTT_SEL_SET(data, 0x1);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG2__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG1__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG1_RXACVCM_SET(data, 0x7);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG1__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG12__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG12_RX_DET_TERM_ENABLE_SET(data, 0x0);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG12__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG148__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = 0xffff; //rx bist word count 0 
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG148__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG149__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = 0xffff; //rx bist word count 1 
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG149__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG150__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = 0xffff; //rx bist word count 2 
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG150__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG151__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = 0xffff; //rx bist word count 3 
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG151__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG147__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG147_STMC_OVERRIDE_SET(data, 0x6); 
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG147__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG1__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG1_CTLE_EQ_SET(data, 0x1c);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG1__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG0__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG0_CTLE_EQ_FR_SET(data, 0x4);
	data = FIELD_CH0_RXTX_REG0_CTLE_EQ_QR_SET(data, 0x4);
	data = FIELD_CH0_RXTX_REG0_CTLE_EQ_HR_SET(data, 0x4);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG0__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG12__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG12_LATCH_OFF_ENA_SET(data, 0x1);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG12__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG128__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG128_LATCH_CAL_WAIT_SEL_SET(data, 0x3);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG128__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG8__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG8_CDR_LOOP_ENA_SET(data, 0x1);
	data = FIELD_CH0_RXTX_REG8_CDR_BYPASS_RXLOS_SET(data, 0x0);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG8__ADDR + (0x200 * ch) + sds2_offset, data);

	//controlled by PIPE
	//pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG61__ADDR + (0x200 * ch) + sds2_offset, &data);
	//data = FIELD_CH0_RXTX_REG61_SPD_SEL_CDR_SET(data, 0x7);
	//pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG61__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG125__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG125_PQ_REG_SET(data, 0xa);
	data = FIELD_CH0_RXTX_REG125_PHZ_MANUAL_SET(data, 0x1);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG125__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG11__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG11_PHASE_ADJUST_LIMIT_SET(data, 0x0);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG11__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG61__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG61_LOADFREQ_SHIFT_SET(data, 0x1);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG61__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG102__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG102_FREQLOOP_LIMIT_SET(data, 0x3);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG102__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG8__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG8_SSC_ENABLE_SET(data, 0x0);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG8__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG96__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG96_MU_FREQ1_SET(data, 0x10);
	data = FIELD_CH0_RXTX_REG96_MU_FREQ2_SET(data, 0x10);
	data = FIELD_CH0_RXTX_REG96_MU_FREQ3_SET(data, 0x10);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG96__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG97__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG97_MU_FREQ4_SET(data, 0x10);
	data = FIELD_CH0_RXTX_REG97_MU_FREQ5_SET(data, 0x10);
	data = FIELD_CH0_RXTX_REG97_MU_FREQ6_SET(data, 0x10);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG97__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG98__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG98_MU_FREQ7_SET(data, 0x10);
	data = FIELD_CH0_RXTX_REG98_MU_FREQ8_SET(data, 0x10);
	data = FIELD_CH0_RXTX_REG98_MU_FREQ9_SET(data, 0x10);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG98__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG99__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG99_MU_PHASE1_SET(data, 0x7);
	data = FIELD_CH0_RXTX_REG99_MU_PHASE2_SET(data, 0x7);
	data = FIELD_CH0_RXTX_REG99_MU_PHASE3_SET(data, 0x7);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG99__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG100__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG100_MU_PHASE4_SET(data, 0x7);
	data = FIELD_CH0_RXTX_REG100_MU_PHASE5_SET(data, 0x7);
	data = FIELD_CH0_RXTX_REG100_MU_PHASE6_SET(data, 0x7);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG100__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG101__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG101_MU_PHASE7_SET(data, 0x7);
	data = FIELD_CH0_RXTX_REG101_MU_PHASE8_SET(data, 0x7);
	data = FIELD_CH0_RXTX_REG101_MU_PHASE9_SET(data, 0x7);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG101__ADDR + (0x200 * ch) + sds2_offset, data);

	//Daniel: just a test
	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG8__ADDR + (0x200 * ch) + sds2_offset, &data);
	//data = FIELD_CH0_RXTX_REG8_SD_DISABLE_SET(data, 0x1);
	data = FIELD_CH0_RXTX_REG8_SD_DISABLE_SET(data, 0x0);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG8__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG26__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG26_BLWC_ENA_SET(data, 0x1); 
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG26__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG81__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG81_MU_DFE1_SET(data, 0xe);  //these are 0x6 for gen3
	data = FIELD_CH0_RXTX_REG81_MU_DFE2_SET(data, 0xe);
	data = FIELD_CH0_RXTX_REG81_MU_DFE3_SET(data, 0xe);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG81__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG82__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG82_MU_DFE4_SET(data, 0xe);  //these are 0x6 for gen3
	data = FIELD_CH0_RXTX_REG82_MU_DFE5_SET(data, 0xe);
	data = FIELD_CH0_RXTX_REG82_MU_DFE6_SET(data, 0xe);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG82__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG83__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG83_MU_DFE7_SET(data, 0xe); //these are 0x6 for gen3
	data = FIELD_CH0_RXTX_REG83_MU_DFE8_SET(data, 0xe);
	data = FIELD_CH0_RXTX_REG83_MU_DFE9_SET(data, 0xe);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG83__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG84__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG84_MU_PH1_SET(data, 0xe); //these are 0x6 for gen3
	data = FIELD_CH0_RXTX_REG84_MU_PH2_SET(data, 0xe);
	data = FIELD_CH0_RXTX_REG84_MU_PH3_SET(data, 0xe);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG84__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG85__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG85_MU_PH4_SET(data, 0xe);//these are 0x6 for gen3
	data = FIELD_CH0_RXTX_REG85_MU_PH5_SET(data, 0xe);
	data = FIELD_CH0_RXTX_REG85_MU_PH6_SET(data, 0xe);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG85__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG86__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG86_MU_PH7_SET(data, 0xe);//these are 0x6 for gen3
	data = FIELD_CH0_RXTX_REG86_MU_PH8_SET(data, 0xe);
	data = FIELD_CH0_RXTX_REG86_MU_PH9_SET(data, 0xe);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG86__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG87__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG87_MU_TH1_SET(data, 0xe);//these are 0x6 for gen3
	data = FIELD_CH0_RXTX_REG87_MU_TH2_SET(data, 0xe);
	data = FIELD_CH0_RXTX_REG87_MU_TH3_SET(data, 0xe);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG87__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG88__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG88_MU_TH4_SET(data, 0xe);//these are 0x6 for gen3
	data = FIELD_CH0_RXTX_REG88_MU_TH5_SET(data, 0xe);
	data = FIELD_CH0_RXTX_REG88_MU_TH6_SET(data, 0xe);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG88__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG89__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG89_MU_TH7_SET(data, 0xe);//these are 0x6 for gen3
	data = FIELD_CH0_RXTX_REG89_MU_TH8_SET(data, 0xe);
	data = FIELD_CH0_RXTX_REG89_MU_TH9_SET(data, 0xe);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG89__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG145__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG145_RXDFE_CONFIG_SET(data, 0x3);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG145__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG28__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = 0x7; //DFE tab enables
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG28__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG7__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG7_RESETB_RXD_SET(data, 0x1);
	data = FIELD_CH0_RXTX_REG7_LOOP_BACK_ENA_CTLE_SET(data, 0x0);
	data = FIELD_CH0_RXTX_REG7_BIST_ENA_RX_SET(data, 0x0);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG7__ADDR + (0x200 * ch) + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG12__ADDR + (0x200 * ch) + sds2_offset, &data);
	data = FIELD_CH0_RXTX_REG12_RX_INV_SET(data, 0x0);
	pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG12__ADDR + (0x200 * ch) + sds2_offset, data);
}

void apm_pcie_manual_calib(void *base, u32 link_width)
{
	u32 data, i;

	//PLL manual calibration
	pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG4__ADDR, &data);
	data = FIELD_CMU_REG4_VCO_MANMOMSEL_PCIE_SET(data, 0x19);
	pcie_phy_csr_write(base, KC_SERDES_CMU_REGS_CMU_REG4__ADDR, data);

	pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG3__ADDR, &data);
	data = FIELD_CMU_REG3_VCO_MANMOMSEL_SET(data, 0xa);
	pcie_phy_csr_write(base, KC_SERDES_CMU_REGS_CMU_REG3__ADDR, data);

	pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG1__ADDR, &data);
	data = FIELD_CMU_REG1_PLL_MANUALCAL_SET(data, 0x1);
	pcie_phy_csr_write(base, KC_SERDES_CMU_REGS_CMU_REG1__ADDR, data);

	if (link_width == 8) {
		pcie_phy_csr_read(base, KC_SERDES2_CMU_REGS_CMU_REG4__ADDR, &data);
		data = FIELD_CMU_REG4_VCO_MANMOMSEL_PCIE_SET(data, 0x19);
		pcie_phy_csr_write(base, KC_SERDES2_CMU_REGS_CMU_REG4__ADDR, data);

		pcie_phy_csr_read(base, KC_SERDES2_CMU_REGS_CMU_REG3__ADDR, &data);
		data = FIELD_CMU_REG3_VCO_MANMOMSEL_SET(data, 0xa);
		pcie_phy_csr_write(base, KC_SERDES2_CMU_REGS_CMU_REG3__ADDR, data);

		pcie_phy_csr_read(base, KC_SERDES2_CMU_REGS_CMU_REG1__ADDR, &data);
		data = FIELD_CMU_REG1_PLL_MANUALCAL_SET(data, 0x1);
		pcie_phy_csr_write(base, KC_SERDES2_CMU_REGS_CMU_REG1__ADDR, data);

	}

	//pvt manual calibration
	pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG6__ADDR, &data);
	data = FIELD_CMU_REG6_PLL_VREGTRIM_SET(data, 0x0);
	data = FIELD_CMU_REG6_MAN_PVT_CAL_SET(data, 0x1);  //130207
	pcie_phy_csr_write(base, KC_SERDES_CMU_REGS_CMU_REG6__ADDR, data);

	if (link_width == 8) {
		//pvt manual calibration
		pcie_phy_csr_read(base, KC_SERDES2_CMU_REGS_CMU_REG6__ADDR, &data);
		data = FIELD_CMU_REG6_PLL_VREGTRIM_SET(data, 0x0);
		data = FIELD_CMU_REG6_MAN_PVT_CAL_SET(data, 0x1);  //130207
		pcie_phy_csr_write(base, KC_SERDES2_CMU_REGS_CMU_REG6__ADDR, data);
	}

	//130207
	for (i = 0; i < 4; i++) {
		//termination calibration
		pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG17__ADDR, &data);
		data = FIELD_CMU_REG17_RESERVED_7_SET(data, i);  //need to get updated spec from KC
		pcie_phy_csr_write(base,  KC_SERDES_CMU_REGS_CMU_REG17__ADDR, data);

		pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG17__ADDR, &data);
		data = FIELD_CMU_REG17_PVT_CODE_R2A_SET(data, 0xd);
		data = FIELD_CMU_REG17_PVT_TERM_MAN_ENA_SET(data, 0x1);
		pcie_phy_csr_write(base,  KC_SERDES_CMU_REGS_CMU_REG17__ADDR, data);

		pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG17__ADDR, &data);
		data = FIELD_CMU_REG17_PVT_TERM_MAN_ENA_SET(data, 0x0);
		pcie_phy_csr_write(base,  KC_SERDES_CMU_REGS_CMU_REG17__ADDR, data);
		//up calibration cause receiver detect problem
		/*
		   pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG17__ADDR);
		   data = FIELD_CMU_REG17_RESERVED_7_SET(data, 0);
		   pcie_phy_csr_write(base,  KC_SERDES_CMU_REGS_CMU_REG17__ADDR, data);

		   pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG17__ADDR);
		   data = FIELD_CMU_REG17_PVT_CODE_R2A_SET(data, 0x26);
		   pcie_phy_csr_write(base,  KC_SERDES_CMU_REGS_CMU_REG17__ADDR , data);

		   pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG16__ADDR );
		   data = FIELD_CMU_REG16_PVT_UP_MAN_ENA_SET (data, 0x1);
		   pcie_phy_csr_write(base,  KC_SERDES_CMU_REGS_CMU_REG16__ADDR , data);

		   pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG16__ADDR );
		   data = FIELD_CMU_REG16_PVT_UP_MAN_ENA_SET (data, 0x0);
		   pcie_phy_csr_write(base,  KC_SERDES_CMU_REGS_CMU_REG16__ADDR , data);
		 */
		//down calibration
		pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG17__ADDR, &data);
		data = FIELD_CMU_REG17_RESERVED_7_SET(data, i);
		pcie_phy_csr_write(base,  KC_SERDES_CMU_REGS_CMU_REG17__ADDR , data);

		pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG17__ADDR, &data);
		data = FIELD_CMU_REG17_PVT_CODE_R2A_SET(data, 0x23);
		pcie_phy_csr_write(base,  KC_SERDES_CMU_REGS_CMU_REG17__ADDR , data);

		pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG16__ADDR, &data);
		data = FIELD_CMU_REG16_PVT_DN_MAN_ENA_SET(data, 0x1);
		pcie_phy_csr_write(base,  KC_SERDES_CMU_REGS_CMU_REG16__ADDR , data);

		pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG16__ADDR, &data);
		data = FIELD_CMU_REG16_PVT_DN_MAN_ENA_SET(data, 0x0);
		pcie_phy_csr_write(base,  KC_SERDES_CMU_REGS_CMU_REG16__ADDR , data);
	}

	if (link_width == 8) {
		for (i = 0; i < 4; i++) {
			//termination calibration
			pcie_phy_csr_read(base, KC_SERDES2_CMU_REGS_CMU_REG17__ADDR, &data);
			data = FIELD_CMU_REG17_RESERVED_7_SET(data, i);  //need to get updated spec from KC
			pcie_phy_csr_write(base,  KC_SERDES2_CMU_REGS_CMU_REG17__ADDR, data);

			pcie_phy_csr_read(base, KC_SERDES2_CMU_REGS_CMU_REG17__ADDR, &data);
			data = FIELD_CMU_REG17_PVT_CODE_R2A_SET(data, 0xd);
			data = FIELD_CMU_REG17_PVT_TERM_MAN_ENA_SET(data, 0x1);
			pcie_phy_csr_write(base,  KC_SERDES2_CMU_REGS_CMU_REG17__ADDR, data);

			pcie_phy_csr_read(base, KC_SERDES2_CMU_REGS_CMU_REG17__ADDR, &data);
			data = FIELD_CMU_REG17_PVT_TERM_MAN_ENA_SET(data, 0x0);
			pcie_phy_csr_write(base,  KC_SERDES2_CMU_REGS_CMU_REG17__ADDR, data);
			//up calibration cause receiver detect problem
			/*
			   pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG17__ADDR);
			   data = FIELD_CMU_REG17_RESERVED_7_SET(data, 0);
			   pcie_phy_csr_write(base,  KC_SERDES_CMU_REGS_CMU_REG17__ADDR, data);

			   pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG17__ADDR);
			   data = FIELD_CMU_REG17_PVT_CODE_R2A_SET(data, 0x26);
			   pcie_phy_csr_write(base,  KC_SERDES_CMU_REGS_CMU_REG17__ADDR , data);

			   pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG16__ADDR );
			   data = FIELD_CMU_REG16_PVT_UP_MAN_ENA_SET (data, 0x1);
			   pcie_phy_csr_write(base,  KC_SERDES_CMU_REGS_CMU_REG16__ADDR , data);

			   pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG16__ADDR );
			   data = FIELD_CMU_REG16_PVT_UP_MAN_ENA_SET (data, 0x0);
			   pcie_phy_csr_write(base,  KC_SERDES_CMU_REGS_CMU_REG16__ADDR , data);
			 */
			//down calibration
			pcie_phy_csr_read(base, KC_SERDES2_CMU_REGS_CMU_REG17__ADDR, &data);
			data = FIELD_CMU_REG17_RESERVED_7_SET(data, i);
			pcie_phy_csr_write(base,  KC_SERDES2_CMU_REGS_CMU_REG17__ADDR , data);

			pcie_phy_csr_read(base, KC_SERDES2_CMU_REGS_CMU_REG17__ADDR, &data);
			data = FIELD_CMU_REG17_PVT_CODE_R2A_SET(data, 0x23);
			pcie_phy_csr_write(base,  KC_SERDES2_CMU_REGS_CMU_REG17__ADDR , data);

			pcie_phy_csr_read(base, KC_SERDES2_CMU_REGS_CMU_REG16__ADDR, &data);
			data = FIELD_CMU_REG16_PVT_DN_MAN_ENA_SET(data, 0x1);
			pcie_phy_csr_write(base,  KC_SERDES2_CMU_REGS_CMU_REG16__ADDR , data);

			pcie_phy_csr_read(base, KC_SERDES2_CMU_REGS_CMU_REG16__ADDR, &data);
			data = FIELD_CMU_REG16_PVT_DN_MAN_ENA_SET(data, 0x0);
			pcie_phy_csr_write(base,  KC_SERDES2_CMU_REGS_CMU_REG16__ADDR , data);
		}
	}
}

void serdes_config_LSPLL(void *base, u32 sds2)
{
	u32 data;
	int sds2_offset = (sds2 > 0) ? 0x30000 : 0x0;

	PCIE_VDEBUG("Serdes PLL config, serdes# 0x%x \n", sds2);
	//pciegen3 control from PIPE is turned off
	pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG0__ADDR + sds2_offset, &data);
	data = FIELD_CMU_REG0_PCIEGEN3_SET(data, 0x1);
	pcie_phy_csr_write(base, KC_SERDES_CMU_REGS_CMU_REG0__ADDR + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG1__ADDR + sds2_offset, &data);
	data = FIELD_CMU_REG1_PLL_CP_SET(data, 0xf);
	data = FIELD_CMU_REG1_PLL_CP_SEL_SET(data, 0xc);
	pcie_phy_csr_write(base, KC_SERDES_CMU_REGS_CMU_REG1__ADDR + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG3__ADDR + sds2_offset, &data);
	data = FIELD_CMU_REG3_VCOVARSEL_SET(data, 0x1);
	pcie_phy_csr_write(base, KC_SERDES_CMU_REGS_CMU_REG3__ADDR + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG2__ADDR + sds2_offset, &data);
	data = FIELD_CMU_REG2_PLL_LFRES_SET(data, 0x2);
	data = FIELD_CMU_REG2_PLL_FBDIV_SET(data, 0x31); //control off from PIPE
	pcie_phy_csr_write(base, KC_SERDES_CMU_REGS_CMU_REG2__ADDR + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG5__ADDR + sds2_offset, &data);
	data = FIELD_CMU_REG5_PLL_LFCAP_SET(data, 0);
	data = FIELD_CMU_REG5_PLL_LFSMCAP_SET(data, 0);
	pcie_phy_csr_write(base, KC_SERDES_CMU_REGS_CMU_REG5__ADDR + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG4__ADDR + sds2_offset, &data);
	data = FIELD_CMU_REG4_VCOVARSEL_PCIE_SET(data, 0x1);
	pcie_phy_csr_write(base, KC_SERDES_CMU_REGS_CMU_REG4__ADDR + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG32__ADDR + sds2_offset, &data);
	data = FIELD_CMU_REG32_IREF_ADJ_SET(data, 0x3);
	pcie_phy_csr_write(base, KC_SERDES_CMU_REGS_CMU_REG32__ADDR + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG1__ADDR + sds2_offset, &data);
	data = FIELD_CMU_REG1_PLL_MANUALCAL_SET(data, 0x0);
	pcie_phy_csr_write(base, KC_SERDES_CMU_REGS_CMU_REG1__ADDR + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG3__ADDR + sds2_offset, &data);
	data = FIELD_CMU_REG3_VCO_MOMSEL_INIT_SET(data, 0x10);
	pcie_phy_csr_write(base, KC_SERDES_CMU_REGS_CMU_REG3__ADDR + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG4__ADDR + sds2_offset, &data);
	data = FIELD_CMU_REG4_VCO_MOMSEL_INIT_PCIE_SET(data, 0x10);
	pcie_phy_csr_write(base, KC_SERDES_CMU_REGS_CMU_REG4__ADDR + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG34__ADDR + sds2_offset, &data);
	data = FIELD_CMU_REG34_VCO_CAL_VTH_HI_MIN_SET(data, 0x2);
	data = FIELD_CMU_REG34_VCO_CAL_VTH_HI_MAX_SET(data, 0xa);
	data = FIELD_CMU_REG34_VCO_CAL_VTH_LO_MIN_SET(data, 0x2);
	data = FIELD_CMU_REG34_VCO_CAL_VTH_LO_MAX_SET(data, 0xa);
	pcie_phy_csr_write(base, KC_SERDES_CMU_REGS_CMU_REG34__ADDR + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG0__ADDR + sds2_offset, &data);
	data = FIELD_CMU_REG0_CAL_COUNT_RESOL_SET(data, 0x7); //pll lock calibration
	pcie_phy_csr_write(base, KC_SERDES_CMU_REGS_CMU_REG0__ADDR + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG16__ADDR + sds2_offset, &data);
	data = FIELD_CMU_REG16_VCOCAL_WAIT_BTW_CODE_SET(data, 0x7); //VCO Calb wait time
	pcie_phy_csr_write(base, KC_SERDES_CMU_REGS_CMU_REG16__ADDR + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG30__ADDR + sds2_offset, &data);
	data = FIELD_CMU_REG30_LOCK_COUNT_SET(data, 0x3); //Lock count wait time
	data = FIELD_CMU_REG30_PLL_FBDIV_GEN3_SET(data, 0x27);  //control turn off from PIPE
	pcie_phy_csr_write(base, KC_SERDES_CMU_REGS_CMU_REG30__ADDR + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG13__ADDR + sds2_offset, &data);
	data = FIELD_CMU_REG13_STATE_DELAY1_SET(data, 0xf);
	data = FIELD_CMU_REG13_STATE_DELAY2_SET(data, 0xf);
	data = FIELD_CMU_REG13_STATE_DELAY3_SET(data, 0xf);
	data = FIELD_CMU_REG13_STATE_DELAY4_SET(data, 0xf);
	pcie_phy_csr_write(base, KC_SERDES_CMU_REGS_CMU_REG13__ADDR + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG14__ADDR + sds2_offset, &data);
	data = FIELD_CMU_REG14_STATE_DELAY5_SET(data, 0xf);
	data = FIELD_CMU_REG14_STATE_DELAY6_SET(data, 0xf);
	data = FIELD_CMU_REG14_STATE_DELAY7_SET(data, 0xf);
	data = FIELD_CMU_REG14_STATE_DELAY8_SET(data, 0xf);
	pcie_phy_csr_write(base, KC_SERDES_CMU_REGS_CMU_REG14__ADDR + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG32__ADDR + sds2_offset, &data);
	data = FIELD_CMU_REG32_PVT_CAL_WAIT_SEL_SET(data, 0x3);
	pcie_phy_csr_write(base, KC_SERDES_CMU_REGS_CMU_REG32__ADDR + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG31__ADDR + sds2_offset, &data);
	data = FIELD_CMU_REG31_LOS_OVERRIDE_CH3_SET(data, 0x1);
	data = FIELD_CMU_REG31_LOS_OVERRIDE_CH2_SET(data, 0x1);
	data = FIELD_CMU_REG31_LOS_OVERRIDE_CH1_SET(data, 0x1);
	data = FIELD_CMU_REG31_LOS_OVERRIDE_CH0_SET(data, 0x1);
	pcie_phy_csr_write(base, KC_SERDES_CMU_REGS_CMU_REG31__ADDR + sds2_offset, data);

	pcie_phy_csr_read(base, KC_SERDES_CMU_REGS_CMU_REG37__ADDR + sds2_offset, &data);
	data = FIELD_CMU_REG37_CTLE_CAL_DONE_OVR_SET(data, 0xf);
	data = FIELD_CMU_REG37_FT_SEARCH_DONE_OVR_SET(data, 0xf);
	pcie_phy_csr_write(base, KC_SERDES_CMU_REGS_CMU_REG37__ADDR + sds2_offset, data);
}

static void sds_rx_invert(void *base, int link_width)
{
	u32 data, i;

	PCIE_VDEBUG("Invert serdes rx clock ..... \n\r");
	if (link_width < 4) {
		pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG13__ADDR, &data);
		data = data | 0x2000;
		pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG13__ADDR, data);
	} else if (link_width == 4){
		for (i = 0 ; i < 4; i++) {
			pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG13__ADDR + (0x200 * i), &data);
			data = data | 0x2000;
			pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG13__ADDR + (0x200 * i), data);
		}
	} else if (link_width == 8) {
		for (i = 0 ; i < 4; i++) {
			pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG13__ADDR + (0x200 * i), &data);
			data = data | 0x2000;
			pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG13__ADDR + (0x200 * i), data);
		}
		for (i = 0 ; i < 4; i++) {
			pcie_phy_csr_read(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG13__ADDR + (0x200 * i) + 0x30000, &data);
			data = data | 0x2000;
			pcie_phy_csr_write(base, KC_SERDES_X4_RXTX_REGS_CH0_RXTX_REG13__ADDR + (0x200 * i) + 0x30000, data);
		}
	}
}

void apm_pcie_init_phy(struct apm_pcie_port *port)
{
	void *base = port->csr_base;
	u32 link_width = port->link_width;
	u32 data;

	apm_init_serdes_refclk(port);

	/* FS/LF adjustment from PHY */
	apm_in32(base + SM_PCIE_CSR_REGS_PIPECTLREG__ADDR, &data);
	data = FIELD_PIPECTLREG_PHY_EQ_TX_FS_SET(data, 0x2c);
	data = FIELD_PIPECTLREG_PHY_EQ_TX_LF_SET(data, 0xc);
	data = FIELD_PIPECTLREG_PHY_EQ_TX_MAX_PRE_SET(data, 0xB);
	data = FIELD_PIPECTLREG_PHY_EQ_TX_MAX_POST_SET(data, 0x16);
	apm_out32(base + SM_PCIE_CSR_REGS_PIPECTLREG__ADDR, data);

	/* customer pin mode setting */
	apm_in32(base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_CTL0__ADDR, &data);
	data = FIELD_PCIE_SDS_CTL0_CFG_I_CUSTOMER_PIN_MODE_SET(data, 0x0d21);
	apm_out32(base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_CTL0__ADDR, data);

	apm_pcie_serdes_pipe_config(base);

	switch (link_width) {
		/* X8 port, second X4 serdes is on PCIE2 */
		case 8:
			serdes_config_tx_control(base, 0, 0);
			serdes_config_tx_control(base, 1, 0);
			serdes_config_tx_control(base, 2, 0);
			serdes_config_tx_control(base, 3, 0);
			serdes_config_rx_control(base, 0, 0);
			serdes_config_rx_control(base, 1, 0);
			serdes_config_rx_control(base, 2, 0);
			serdes_config_rx_control(base, 3, 0);
			serdes_config_LSPLL(base, 0);

			serdes_config_tx_control(base, 0, 1);
			serdes_config_tx_control(base, 1, 1);
			serdes_config_tx_control(base, 2, 1);
			serdes_config_tx_control(base, 3, 1);
			serdes_config_rx_control(base, 0, 1);
			serdes_config_rx_control(base, 1, 1);
			serdes_config_rx_control(base, 2, 1);
			serdes_config_rx_control(base, 3, 1);
			serdes_config_LSPLL(base, 1);
			break;
		/* X4 port, uses 1 X4 serdes */
		case 4:
			serdes_config_tx_control(base, 0, 0);
			serdes_config_tx_control(base, 1, 0);
			serdes_config_tx_control(base, 2, 0);
			serdes_config_tx_control(base, 3, 0);
			serdes_config_rx_control(base, 0, 0);
			serdes_config_rx_control(base, 1, 0);
			serdes_config_rx_control(base, 2, 0);
			serdes_config_rx_control(base, 3, 0);
			serdes_config_LSPLL(base, 0);
			break;
		/* x1 port, use 1 x1 serdes */
		case 1: 
			// x1 port, use 1 x1 serdes 
			serdes_config_tx_control(base, 0, 0);
			serdes_config_rx_control(base, 0, 0);
			serdes_config_LSPLL(base, 0);
			break;
	}

	sds_rx_invert(base, link_width);
}

#if 0
void apm_pcie_rxdetect_mask(void *base, u32 link_width)
{
	u32 data, mask;

	switch (link_width)
	{
		case (1): mask = 0x1; break;
		case (2): mask = 0x3; break;
		case (4): mask = 0xf; break;
		case (8): mask = 0xff; break;
		default: mask = 0xffff; break;
	}

	apm_in32(base +  NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_543_512__ADDR, &data);
	data = FIELD_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_543_512_CFG_CONSTANTS_RX_DETECT_OVERRIDE_SET(data, 0x1);
	apm_out32(base +  NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_543_512__ADDR, data);

	apm_in32(base +  NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_575_544__ADDR, &data);
	data = FIELD_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_575_544_CFG_CONSTANTS_RX_DETECT_MASK_SET(data, mask);
	apm_out32(base +  NWL_PCIE_APB_REGS_EXPRESSO_CFG_CONSTANTS_CFG_CONSTANTS_575_544__ADDR, data);
}
#endif
