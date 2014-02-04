/**
 * APM X-Gene PCIe PHY / Serdes initialization
 *
 * Copyright (c) 2013 Applied Micro Circuits Corporation.
 *
 * Author: Tanmay Inamdar <tinamdar@apm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include "pcie-xgene-serdes.h"

static void xgene_pcie_phy_wr(void *base, u32 addr, u32 dt)
{
	u32 val;
	u32 cmd = IND_WR_CMD_MASK | IND_CMD_DONE_MASK;

	cmd |= addr << 0x4;
	xgene_pcie_out32(base + IND_WDATA_REG, dt);
	xgene_pcie_out32(base + IND_CMD_REG, cmd);
	do {
		xgene_pcie_in32(base + IND_CMD_REG, &val);
		val &= IND_CMD_DONE_MASK;
	} while (val != IND_CMD_DONE_MASK);
}

static void xgene_pcie_phy_rd(void *base, u32 addr, u32 *dt)
{
	u32 val;
	u32 cmd = IND_RD_CMD_MASK | IND_CMD_DONE_MASK;

	cmd |= addr << 0x4;
	xgene_pcie_out32(base + IND_CMD_REG, cmd);
	do {
		xgene_pcie_in32(base + IND_CMD_REG, &val);
		val &= IND_CMD_DONE_MASK;
	} while (val != IND_CMD_DONE_MASK);

	xgene_pcie_in32(base + IND_RDATA_REG, dt);
}

static void xgene_pcie_init_serdes_refclk(struct xgene_pcie_port *port)
{
	void *csr_base = port->csr_base;
	u32 val;

	xgene_pcie_in32(csr_base + PCIE_SDS_CTL0, &val);
	val &= ~0x1000;
	xgene_pcie_out32(csr_base + PCIE_SDS_CTL0, val);

	switch (port->serdes_clk) {
	case EXTERNAL_DIFFERENTIAL_CLK:
		if (port->link_width == 8) {
			xgene_pcie_phy_rd(csr_base, CMU_REG0 + SDS1_OFF, &val);
			val = PLL_REF_SEL_SET(val, 0x1);
			xgene_pcie_phy_wr(csr_base, CMU_REG0 + SDS1_OFF, val);
		}
		break;
	}
}

static void xgene_pcie_serdes_pipe_config(void *base)
{
	u32 dt, i;

	xgene_pcie_phy_rd(base, DETECT_CONTROL, &dt);
	dt = ONE_CNT_TH_SET(dt, 0xffff);
	xgene_pcie_phy_wr(base, DETECT_CONTROL, dt);

	xgene_pcie_phy_rd(base, USB_PCIE_CTRL, &dt);
	dt = ONE_CNT_CMP_TH_SET(dt, 0x50);
	xgene_pcie_phy_wr(base, USB_PCIE_CTRL, dt);

	for (i = 0; i < 8; i++) {
		xgene_pcie_phy_rd(base, DFE_CONTROL0 + (i * 4), &dt);
		dt = SEL_CDR_OVR_LN_SET(dt, 0x7);
		xgene_pcie_phy_wr(base, DFE_CONTROL0 + (i * 4), dt);
	}

	xgene_pcie_phy_rd(base, SERDES_CONTROL3, &dt);
	dt = TX_AMP_EN_LN0_SET(dt, 0x1);
	dt = TX_AMP_LN0_SET(dt, 0xf);
	xgene_pcie_phy_wr(base, SERDES_CONTROL3, dt);

	for (i = 0; i < 7; i++) {
		xgene_pcie_phy_rd(base, SERDES_CONTROL4 + (i * 4), &dt);
		dt = TX_AMP_EN_LN1_SET(dt, 0x1);
		dt = TX_AMP_LN1_SET(dt, 0xf);
		xgene_pcie_phy_wr(base, SERDES_CONTROL4 + (i * 4), dt);
	}
}

static int xgene_pcie_init_ecc(struct xgene_pcie_port *port)
{
	void *csr_base = port->csr_base;
	int timeout = XGENE_PCIE_TIMEOUT;
	u32 val;

	xgene_pcie_out32(csr_base + MEM_RAM_SHUTDOWN, 0x0);
	xgene_pcie_in32(csr_base + MEM_RAM_SHUTDOWN, &val);
	xgene_pcie_in32(csr_base + BLOCK_MEM_RDY, &val);
	while ((val != BLOCK_MEM_RDY_VAL) && timeout) {
		timeout--;
		xgene_pcie_in32(csr_base + BLOCK_MEM_RDY, &val);
	}

	return timeout > 0;
}

void xgene_pcie_reset_pcie_core_clk(struct xgene_pcie_port *port)
{
	void *csr_base = port->csr_base;
	u32 val;

	xgene_pcie_in32(csr_base + PCIE_CLKEN, &val);
	if (!(val & CORE_CLKEN_MASK)) {
		val |= CORE_CLKEN_MASK;
		xgene_pcie_out32(csr_base + PCIE_CLKEN, val);
	}

	xgene_pcie_in32(csr_base + PCIE_SRST, &val);
	if (val & CORE_RESET_MASK) {
		val &= ~CORE_RESET_MASK;
		xgene_pcie_out32(csr_base + PCIE_SRST, val);
	}
}

static int xgene_pcie_reset_done(struct xgene_pcie_port *port)
{
	void *csr_base = port->csr_base;
	u32 val;

	xgene_pcie_in32(csr_base + PCIE_SRST, &val);
	if ((val & 0xfff) == 0x0)
		return 1;
	return 0;
}

static void xgene_pcie_reset_clk(struct xgene_pcie_port *port)
{
	void *csr_base = port->csr_base;
	u32 val;

	xgene_pcie_out32(csr_base + PCIE_CLKEN, 0x00000000);
	xgene_pcie_out32(csr_base + PCIE_SRST, 0xFFFFFFFF);

	xgene_pcie_in32(csr_base + PCIE_CLKEN, &val);
	xgene_pcie_out32(csr_base + PCIE_CLKEN, val | 0xD);

	xgene_pcie_in32(csr_base + PCIE_SRST, &val);
	xgene_pcie_out32(csr_base + PCIE_SRST, val & ~0xD);

	if (port->link_width == 8) {
		xgene_pcie_in32(csr_base + SDS_MUX, &val);
		xgene_pcie_out32(csr_base + SDS_MUX, val | 0x1);
	}
}

static void xgene_pcie_release_phy_reset(void *base)
{
	u32 dt;

	xgene_pcie_in32(base + PCIE_SRST, &dt);
	xgene_pcie_out32(base + PCIE_SRST, dt & ~0x100);
}

static int xgene_pcie_wait_phy_rdy(void *base)
{
	u32 dt;
	int timeout = XGENE_PCIE_TIMEOUT;

	do {
		xgene_pcie_in32(base + PCIECORE_CTLANDSTATUS, &dt);
		dt &= 0xff;
	} while (dt != 0 && timeout--);
	return timeout > 0;
}

static int xgene_pcie_wait_pll_lock(struct xgene_pcie_port *port)
{
	void *base = port->csr_base;
	u32 linkwidth = port->link_width;
	u32 dt;
	u32 lock = 0;
	u32 lock1 = 0;
	int timeout = XGENE_PCIE_TIMEOUT;

	if (linkwidth != 8) {
		do {
			xgene_pcie_in32(base + SDS0_CMU_STATUS0, &dt);
			lock = dt & 0x7;
		} while (lock == 0 && timeout--);
	} else {
		do {
			xgene_pcie_in32(base + SDS0_CMU_STATUS0, &dt);
			lock = dt & 0x7;

			xgene_pcie_in32(base + SDS1_CMU_STATUS0, &dt);
			lock1 = dt & 0x7;
		} while (lock == 0 && lock1 == 0 && timeout--);
	}
	return timeout > 0;
}

static void xgene_pcie_serdes_cfg_tx_ctrl(void *base, u32 ch, u32 sds2)
{
	u32 offset = (sds2 > 0) ? SDS1_OFF : 0x0;
	u32 dt;

	xgene_pcie_phy_rd(base, CH0_RXTX_REG2 + (CH_OFF * ch) + offset, &dt);
	dt = TX_FIFO_ENA_SET(dt, 0x1);
	dt = RESETB_TXD_SET(dt, 0x1);
	dt = BIST_ENA_TX_SET(dt, 0x0);
	dt = TX_INV_SET(dt, 0x0);
	dt = TX_RCVDET_SEL_SET(dt, 3);
	xgene_pcie_phy_wr(base, CH0_RXTX_REG2 + (CH_OFF * ch) + offset, dt);

	xgene_pcie_phy_rd(base, CH0_RXTX_REG6 + (CH_OFF * ch) + offset, &dt);
	dt = TXAMP_ENA_SET(dt, 0x1);
	dt = TXAMP_CNTL_SET(dt, 0xf);
	dt = TX_IDLE_SET(dt, 0x0);
	xgene_pcie_phy_wr(base, CH0_RXTX_REG6 + (CH_OFF * ch) + offset, dt);

	xgene_pcie_phy_rd(base, CH0_RXTX_REG5 + (CH_OFF * ch) + offset, &dt);
	dt = TX_CN2_SET(dt, 0x2);
	dt = TX_CP1_SET(dt, 0xf);
	dt = TX_CN1_SET(dt, 0x2);
	xgene_pcie_phy_wr(base, CH0_RXTX_REG5 + (CH_OFF * ch) + offset, dt);

	xgene_pcie_phy_rd(base, CH0_RXTX_REG4 + (CH_OFF * ch) + offset, &dt);
	dt = TX_LOOPBACK_BUF_EN_SET(dt, 0x0);
	xgene_pcie_phy_wr(base, CH0_RXTX_REG4 + (CH_OFF * ch) + offset, dt);

	xgene_pcie_phy_rd(base, CH0_RXTX_REG145 + (CH_OFF * ch) + offset, &dt);
	dt = TX_IDLE_SATA_SET(dt, 0x0);
	xgene_pcie_phy_wr(base, CH0_RXTX_REG145 + (CH_OFF * ch) + offset, dt);
}

static void xgene_pcie_serdes_cfg_rx_ctrl(void *base, u32 ch, u32 sds2)
{
	u32 offset = (sds2 > 0) ? SDS1_OFF : 0x0;
	u32 dt, addr, i;

	xgene_pcie_phy_rd(base, CH0_RXTX_REG147 + (CH_OFF * ch) + offset, &dt);
	dt = STMC_OVERRIDE_SET(dt, 0x6);
	xgene_pcie_phy_wr(base, CH0_RXTX_REG147 + (CH_OFF * ch) + offset, dt);

	xgene_pcie_phy_rd(base, CH0_RXTX_REG27 + (CH_OFF * ch) + offset, &dt);
	dt = RXPD_CONFIG_SET(dt, 0x1);
	xgene_pcie_phy_wr(base, CH0_RXTX_REG27 + (CH_OFF * ch) + offset, dt);

	xgene_pcie_phy_rd(base, CH0_RXTX_REG2 + (CH_OFF * ch) + offset, &dt);
	dt = RESETB_TERM_SET(dt, 0x0);
	dt = VTT_ENA_SET(dt, 0x1);
	dt = VTT_SEL_SET(dt, 0x1);
	xgene_pcie_phy_wr(base, CH0_RXTX_REG2 + (CH_OFF * ch) + offset, dt);

	xgene_pcie_phy_rd(base, CH0_RXTX_REG1 + (CH_OFF * ch) + offset, &dt);
	dt = RXACVCM_SET(dt, 0x7);
	xgene_pcie_phy_wr(base, CH0_RXTX_REG1 + (CH_OFF * ch) + offset, dt);

	xgene_pcie_phy_rd(base, CH0_RXTX_REG12 + (CH_OFF * ch) + offset, &dt);
	dt = RX_DET_TERM_ENABLE_SET(dt, 0x0);
	xgene_pcie_phy_wr(base, CH0_RXTX_REG12 + (CH_OFF * ch) + offset, dt);

	addr = CH0_RXTX_REG148;
	for (i = 0; i < 4; i++) {
		xgene_pcie_phy_rd(base, addr + (CH_OFF * ch) + offset, &dt);
		dt = 0xffff;
		xgene_pcie_phy_wr(base, addr + (CH_OFF * ch) + offset, dt);
		addr += 2;
	}

	xgene_pcie_phy_rd(base, CH0_RXTX_REG147 + (CH_OFF * ch) + offset, &dt);
	dt = STMC_OVERRIDE_SET(dt, 0x6);
	xgene_pcie_phy_wr(base, CH0_RXTX_REG147 + (CH_OFF * ch) + offset, dt);

	xgene_pcie_phy_rd(base, CH0_RXTX_REG1 + (CH_OFF * ch) + offset, &dt);
	dt = CTLE_EQ_SET(dt, 0x1c);
	xgene_pcie_phy_wr(base, CH0_RXTX_REG1 + (CH_OFF * ch) + offset, dt);

	xgene_pcie_phy_rd(base, CH0_RXTX_REG0 + (CH_OFF * ch) + offset, &dt);
	dt = CTLE_EQ_FR_SET(dt, 0x1C);
	dt = CTLE_EQ_QR_SET(dt, 0x1C);
	dt = CTLE_EQ_HR_SET(dt, 0x1C);
	xgene_pcie_phy_wr(base, CH0_RXTX_REG0 + (CH_OFF * ch) + offset, dt);

	xgene_pcie_phy_rd(base, CH0_RXTX_REG12 + (CH_OFF * ch) + offset, &dt);
	dt = LATCH_OFF_ENA_SET(dt, 0x1);
	xgene_pcie_phy_wr(base, CH0_RXTX_REG12 + (CH_OFF * ch) + offset, dt);

	xgene_pcie_phy_rd(base, CH0_RXTX_REG128 + (CH_OFF * ch) + offset, &dt);
	dt = LATCH_CAL_WAIT_SEL_SET(dt, 0x3);
	xgene_pcie_phy_wr(base, CH0_RXTX_REG128 + (CH_OFF * ch) + offset, dt);

	xgene_pcie_phy_rd(base, CH0_RXTX_REG8 + (CH_OFF * ch) + offset, &dt);
	dt = CDR_LOOP_ENA_SET(dt, 0x1);
	dt = CDR_BYPASS_RXLOS_SET(dt, 0x0);
	dt = SD_VREF_SET(dt, 0x4);
	xgene_pcie_phy_wr(base, CH0_RXTX_REG8 + (CH_OFF * ch) + offset, dt);

	xgene_pcie_phy_rd(base, CH0_RXTX_REG125 + (CH_OFF * ch) + offset, &dt);
	dt = PQ_REG_SET(dt, 0xa);
	dt = PHZ_MANUAL_SET(dt, 0x1);
	xgene_pcie_phy_wr(base, CH0_RXTX_REG125 + (CH_OFF * ch) + offset, dt);

	xgene_pcie_phy_rd(base, CH0_RXTX_REG11 + (CH_OFF * ch) + offset, &dt);
	dt = PHASE_ADJUST_LIMIT_SET(dt, 0x0);
	xgene_pcie_phy_wr(base, CH0_RXTX_REG11 + (CH_OFF * ch) + offset, dt);

	xgene_pcie_phy_rd(base, CH0_RXTX_REG61 + (CH_OFF * ch) + offset, &dt);
	dt = LOADFREQ_SHIFT_SET(dt, 0x1);
	xgene_pcie_phy_wr(base, CH0_RXTX_REG61 + (CH_OFF * ch) + offset, dt);

	xgene_pcie_phy_rd(base, CH0_RXTX_REG102 + (CH_OFF * ch) + offset, &dt);
	dt = FREQLOOP_LIMIT_SET(dt, 0x3);
	xgene_pcie_phy_wr(base, CH0_RXTX_REG102 + (CH_OFF * ch) + offset, dt);

	xgene_pcie_phy_rd(base, CH0_RXTX_REG8 + (CH_OFF * ch) + offset, &dt);
	dt = SSC_ENABLE_SET(dt, 0x0);
	xgene_pcie_phy_wr(base, CH0_RXTX_REG8 + (CH_OFF * ch) + offset, dt);

	addr = CH0_RXTX_REG96;
	for (i = 0; i < 3; i++) {
		xgene_pcie_phy_rd(base, addr + (CH_OFF * ch) + offset, &dt);
		dt = MU_FREQ1_SET(dt, 0x10);
		dt = MU_FREQ2_SET(dt, 0x10);
		dt = MU_FREQ3_SET(dt, 0x10);
		xgene_pcie_phy_wr(base, addr + (CH_OFF * ch) + offset, dt);
		addr += 2;
	}

	addr = CH0_RXTX_REG99;
	for (i = 0; i < 3; i++) {
		xgene_pcie_phy_rd(base, addr + (CH_OFF * ch) + offset, &dt);
		dt = MU_PHASE1_SET(dt, 0x7);
		dt = MU_PHASE2_SET(dt, 0x7);
		dt = MU_PHASE3_SET(dt, 0x7);
		xgene_pcie_phy_wr(base, addr + (CH_OFF * ch) + offset, dt);
		addr += 2;
	}

	xgene_pcie_phy_rd(base, CH0_RXTX_REG8 + (CH_OFF * ch) + offset, &dt);
	dt = SD_DISABLE_SET(dt, 0x0);
	xgene_pcie_phy_wr(base, CH0_RXTX_REG8 + (CH_OFF * ch) + offset, dt);

	xgene_pcie_phy_rd(base, CH0_RXTX_REG26 + (CH_OFF * ch) + offset, &dt);
	dt = BLWC_ENA_SET(dt, 0x1);
	xgene_pcie_phy_wr(base, CH0_RXTX_REG26 + (CH_OFF * ch) + offset, dt);

	addr = CH0_RXTX_REG81;
	for (i = 0; i < 9; i++) {
		xgene_pcie_phy_rd(base, addr + (CH_OFF * ch) + offset, &dt);
		dt = MU_DFE1_SET(dt, 0xe);
		dt = MU_DFE2_SET(dt, 0xe);
		dt = MU_DFE3_SET(dt, 0xe);
		xgene_pcie_phy_wr(base, addr + (CH_OFF * ch) + offset, dt);
		addr += 2;
	}

	xgene_pcie_phy_rd(base, CH0_RXTX_REG145 + (CH_OFF * ch) + offset, &dt);
	dt = RXDFE_CONFIG_SET(dt, 0x3);
	xgene_pcie_phy_wr(base, CH0_RXTX_REG145 + (CH_OFF * ch) + offset, dt);

	xgene_pcie_phy_wr(base, CH0_RXTX_REG28 + (CH_OFF * ch) + offset, 0x7);

	xgene_pcie_phy_rd(base, CH0_RXTX_REG7 + (CH_OFF * ch) + offset, &dt);
	dt = RESETB_RXD_SET(dt, 0x1);
	dt = LP_ENA_CTLE_SET(dt, 0x0);
	dt = BIST_ENA_RX_SET(dt, 0x0);
	xgene_pcie_phy_wr(base, CH0_RXTX_REG7 + (CH_OFF * ch) + offset, dt);

	xgene_pcie_phy_rd(base, CH0_RXTX_REG12 + (CH_OFF * ch) + offset, &dt);
	dt = RX_INV_SET(dt, 0x0);
	xgene_pcie_phy_wr(base, CH0_RXTX_REG12 + (CH_OFF * ch) + offset, dt);
}

static void xgene_pcie_get_momsel(void *base, u32 sds,
				  u32 *momsel, u32 *momsel_pcie)
{
	int cal_done = 0;
	int timeout = XGENE_PCIE_TIMEOUT;
	u32 offset = (sds > 0) ? SDS1_OFF : 0x0;
	u32 dt;

	xgene_pcie_phy_rd(base, CMU_REG33 + offset, &dt);
	dt = CUST_MODE_INV_SET(dt, 0xffff);
	xgene_pcie_phy_wr(base, CMU_REG33 + offset, dt);

	xgene_pcie_phy_rd(base, CMU_REG30 + offset, &dt);
	dt = PCIE_MODE_SET(dt, 0x0);
	xgene_pcie_phy_wr(base, CMU_REG30 + offset, dt);

	xgene_pcie_phy_rd(base, CMU_REG1 + offset, &dt);
	dt = PLL_MANUALCAL_SET(dt, 0x0);
	xgene_pcie_phy_wr(base, CMU_REG1 + offset, dt);

	xgene_pcie_phy_rd(base, CMU_REG2 + offset, &dt);
	dt = PLL_REFDIV_SET(dt, 0x1);
	dt = PLL_LFRES_SET(dt, 0x2);
	dt = PLL_FBDIV_SET(dt, 0x4f);
	xgene_pcie_phy_wr(base, CMU_REG2 + offset, dt);

	xgene_pcie_phy_rd(base, CMU_REG9 + offset, &dt);
	dt = POST_DIVBY2_SET(dt, 0x1);
	xgene_pcie_phy_wr(base, CMU_REG9 + offset, dt);

	xgene_pcie_phy_rd(base, CMU_REG34 + offset, &dt);
	dt = CAL_VTH_LO_MAX_SET(dt, 0x8);
	dt = CAL_VTH_HI_MAX_SET(dt, 0x8);
	dt = CAL_VTH_LO_MIN_SET(dt, 0x5);
	dt = CAL_VTH_HI_MIN_SET(dt, 0x5);
	xgene_pcie_phy_wr(base, CMU_REG34 + offset, dt);

	xgene_pcie_phy_rd(base, CMU_REG0 + offset, &dt);
	dt = CAL_COUNT_RESOL_SET(dt, 0x7);
	xgene_pcie_phy_wr(base, CMU_REG0 + offset, dt);

	xgene_pcie_phy_rd(base, CMU_REG32 + offset, &dt);
	dt = VCOCAL_START_SET(dt, 0x1);
	xgene_pcie_phy_wr(base, CMU_REG32 + offset, dt);

	xgene_pcie_phy_rd(base, CMU_REG32 + offset, &dt);
	dt = VCOCAL_START_SET(dt, 0x0);
	xgene_pcie_phy_wr(base, CMU_REG32 + offset, dt);

	do {
		timeout--;
		xgene_pcie_phy_rd(base, CMU_REG7 + offset, &dt);
		cal_done = PLL_CALIB_DONE_RD(dt);
	} while (!cal_done && timeout);

	xgene_pcie_phy_rd(base, CMU_REG19 + offset, &dt);
	*momsel_pcie = PLL_VCOMOMSEL_RD(dt);

	/* The delay is required for Serdes Calibration
	 * to complete successfully.
	 */
	mdelay(20);
	xgene_pcie_phy_rd(base, CMU_REG2 + offset, &dt);
	dt = PLL_REFDIV_SET(dt, 0x0);
	dt = PLL_FBDIV_SET(dt, 0x31);
	xgene_pcie_phy_wr(base, CMU_REG2 + offset, dt);

	xgene_pcie_phy_rd(base, CMU_REG9 + offset, &dt);
	dt = POST_DIVBY2_SET(dt, 0x1);
	xgene_pcie_phy_wr(base, CMU_REG9 + offset, dt);

	xgene_pcie_phy_rd(base, CMU_REG32 + offset, &dt);
	dt = VCOCAL_START_SET(dt, 0x1);
	xgene_pcie_phy_wr(base, CMU_REG32 + offset, dt);

	xgene_pcie_phy_rd(base, CMU_REG32 + offset, &dt);
	dt = VCOCAL_START_SET(dt, 0x0);
	xgene_pcie_phy_wr(base, CMU_REG32 + offset, dt);

	timeout = XGENE_PCIE_TIMEOUT;
	cal_done = 0;
	do {
		timeout--;
		xgene_pcie_phy_rd(base, CMU_REG7 + offset, &dt);
		cal_done = PLL_CALIB_DONE_RD(dt);
	} while (!cal_done && timeout);

	xgene_pcie_phy_rd(base, CMU_REG19 + offset, &dt);
	*momsel = PLL_VCOMOMSEL_RD(dt);
	xgene_pcie_phy_rd(base, CMU_REG2 + offset, &dt);
	dt = PLL_FBDIV_SET(dt, 0x31);
	xgene_pcie_phy_wr(base, CMU_REG2 + offset, dt);

	xgene_pcie_phy_rd(base, CMU_REG9 + offset, &dt);
	dt = POST_DIVBY2_SET(dt, 0x1);
	xgene_pcie_phy_wr(base, CMU_REG9 + offset, dt);

	xgene_pcie_phy_rd(base, CMU_REG33 + offset, &dt);
	dt = CUST_MODE_INV_SET(dt, 0x0);
	xgene_pcie_phy_wr(base, CMU_REG33 + offset, dt);
}

static void xgene_pcie_pll_calib(void *base, u32 link_width)
{
	u32 dt, j, addr;
	u32 momsel, momsel_pcie;

	for (j = 0; j < (link_width / 4); j++) {
		xgene_pcie_get_momsel(base, j, &momsel, &momsel_pcie);
		addr = j * SDS1_OFF;
		xgene_pcie_phy_rd(base, CMU_REG4 + addr, &dt);
		dt = MANMOMSEL_PCIE_SET(dt, momsel_pcie);
		xgene_pcie_phy_wr(base, CMU_REG4 + addr, dt);

		xgene_pcie_phy_rd(base, CMU_REG3 + addr, &dt);
		dt = VCO_MANMOMSEL_SET(dt, momsel);
		xgene_pcie_phy_wr(base, CMU_REG3 + addr, dt);

		xgene_pcie_phy_rd(base, CMU_REG1 + addr, &dt);
		dt = PLL_MANUALCAL_SET(dt, 0x1);
		xgene_pcie_phy_wr(base, CMU_REG1 + addr, dt);
	}
	for (j = 0; j < (link_width / 4); j++) {
		addr = j * SDS1_OFF;
		xgene_pcie_phy_rd(base, CMU_REG6 + addr, &dt);
		dt = PLL_VREGTRIM_SET(dt, 0x0);
		dt = MAN_PVT_CAL_SET(dt, 0x1);
		xgene_pcie_phy_wr(base, CMU_REG6 + addr, dt);
	}
}

static void xgene_pcie_rcvr_detect(void *base, u32 addr, int index)
{
	u32 dt;

	xgene_pcie_phy_rd(base, CMU_REG17 + addr, &dt);
	dt = RESERVED_7_SET(dt, index);
	xgene_pcie_phy_wr(base, CMU_REG17 + addr, dt);

	xgene_pcie_phy_rd(base, CMU_REG17 + addr, &dt);
	dt = PVT_CODE_R2A_SET(dt, 0x29);
	xgene_pcie_phy_wr(base, CMU_REG17 + addr, dt);

	xgene_pcie_phy_rd(base, CMU_REG16 + addr, &dt);
	dt = UP_MAN_ENA_SET(dt, 0x1);
	xgene_pcie_phy_wr(base, CMU_REG16 + addr, dt);

	xgene_pcie_phy_rd(base, CMU_REG16 + addr, &dt);
	dt = UP_MAN_ENA_SET(dt, 0x0);
	xgene_pcie_phy_wr(base, CMU_REG16 + addr, dt);
}

static void xgene_pcie_manual_calib(void *base, u32 link_width)
{
	u32 dt, addr, i, j;

	if (link_width == 1)
		link_width = 4;

	xgene_pcie_pll_calib(base, link_width);

	for (j = 0; j < (link_width / 4); j++) {
		addr = j * SDS1_OFF;
		xgene_pcie_phy_rd(base, CMU_REG6 + addr, &dt);
		dt = PLL_VREGTRIM_SET(dt, 0x0);
		dt = MAN_PVT_CAL_SET(dt, 0x1);
		xgene_pcie_phy_wr(base, CMU_REG6 + addr, dt);
	}

	for (j = 0; j < (link_width / 4); j++) {
		addr = j * SDS1_OFF;
		for (i = 0; i < 4; i++) {
			xgene_pcie_phy_rd(base, CMU_REG17 + addr, &dt);
			dt = RESERVED_7_SET(dt, i);
			xgene_pcie_phy_wr(base, CMU_REG17 + addr, dt);

			xgene_pcie_phy_rd(base, CMU_REG17 + addr, &dt);
			dt = PVT_CODE_R2A_SET(dt, 0x12);
			dt = TERM_MAN_ENA_SET(dt, 0x1);
			xgene_pcie_phy_wr(base, CMU_REG17 + addr, dt);

			xgene_pcie_phy_rd(base, CMU_REG17 + addr, &dt);
			dt = TERM_MAN_ENA_SET(dt, 0x0);
			xgene_pcie_phy_wr(base, CMU_REG17 + addr, dt);

			xgene_pcie_rcvr_detect(base, addr, i);

			xgene_pcie_phy_rd(base, CMU_REG17 + addr, &dt);
			dt = RESERVED_7_SET(dt, i);
			xgene_pcie_phy_wr(base, CMU_REG17 + addr, dt);

			xgene_pcie_phy_rd(base, CMU_REG17 + addr, &dt);
			dt = PVT_CODE_R2A_SET(dt, 0x29);
			xgene_pcie_phy_wr(base, CMU_REG17 + addr, dt);

			xgene_pcie_phy_rd(base, CMU_REG16 + addr, &dt);
			dt = DN_MAN_ENA_SET(dt, 0x1);
			xgene_pcie_phy_wr(base, CMU_REG16 + addr, dt);

			xgene_pcie_phy_rd(base, CMU_REG16 + addr, &dt);
			dt = DN_MAN_ENA_SET(dt, 0x0);
			xgene_pcie_phy_wr(base, CMU_REG16 + addr, dt);
		}
	}
}

static void xgene_pcie_serdes_cfg_LSPLL(void *base, u32 sds2)
{
	int offset = (sds2 > 0) ? SDS1_OFF : 0x0;
	u32 dt;

	xgene_pcie_phy_rd(base, CMU_REG0 + offset, &dt);
	dt = PCIEGEN3_SET(dt, 0x1);
	xgene_pcie_phy_wr(base, CMU_REG0 + offset, dt);

	xgene_pcie_phy_rd(base, CMU_REG1 + offset, &dt);
	dt = PLL_CP_SET(dt, 0xf);
	dt = PLL_CP_SEL_SET(dt, 0xc);
	xgene_pcie_phy_wr(base, CMU_REG1 + offset, dt);

	xgene_pcie_phy_rd(base, CMU_REG3 + offset, &dt);
	dt = VCOVARSEL_SET(dt, 0x1);
	xgene_pcie_phy_wr(base, CMU_REG3 + offset, dt);

	xgene_pcie_phy_rd(base, CMU_REG2 + offset, &dt);
	dt = PLL_LFRES_SET(dt, 0x2);
	dt = PLL_FBDIV_SET(dt, 0x31);
	xgene_pcie_phy_wr(base, CMU_REG2 + offset, dt);

	xgene_pcie_phy_rd(base, CMU_REG5 + offset, &dt);
	dt = PLL_LFCAP_SET(dt, 0);
	dt = PLL_LFSMCAP_SET(dt, 0);
	xgene_pcie_phy_wr(base, CMU_REG5 + offset, dt);

	xgene_pcie_phy_rd(base, CMU_REG4 + offset, &dt);
	dt = VCOVARSEL_PCIE_SET(dt, 0x1);
	xgene_pcie_phy_wr(base, CMU_REG4 + offset, dt);

	xgene_pcie_phy_rd(base, CMU_REG32 + offset, &dt);
	dt = IREF_ADJ_SET(dt, 0x3);
	xgene_pcie_phy_wr(base, CMU_REG32 + offset, dt);

	xgene_pcie_phy_rd(base, CMU_REG1 + offset, &dt);
	dt = PLL_MANUALCAL_SET(dt, 0x0);
	xgene_pcie_phy_wr(base, CMU_REG1 + offset, dt);

	xgene_pcie_phy_rd(base, CMU_REG3 + offset, &dt);
	dt = MOMSEL_INIT_SET(dt, 0x10);
	xgene_pcie_phy_wr(base, CMU_REG3 + offset, dt);

	xgene_pcie_phy_rd(base, CMU_REG4 + offset, &dt);
	dt = MOMSEL_INIT_PCIE_SET(dt, 0x10);
	xgene_pcie_phy_wr(base, CMU_REG4 + offset, dt);

	xgene_pcie_phy_rd(base, CMU_REG34 + offset, &dt);
	dt = CAL_VTH_HI_MIN_SET(dt, 0x2);
	dt = CAL_VTH_HI_MAX_SET(dt, 0xa);
	dt = CAL_VTH_LO_MIN_SET(dt, 0x2);
	dt = CAL_VTH_LO_MAX_SET(dt, 0xa);
	xgene_pcie_phy_wr(base, CMU_REG34 + offset, dt);

	xgene_pcie_phy_rd(base, CMU_REG0 + offset, &dt);
	dt = CAL_COUNT_RESOL_SET(dt, 0x7);
	xgene_pcie_phy_wr(base, CMU_REG0 + offset, dt);

	xgene_pcie_phy_rd(base, CMU_REG16 + offset, &dt);
	dt = WAIT_BTW_CODE_SET(dt, 0x7);
	xgene_pcie_phy_wr(base, CMU_REG16 + offset, dt);

	xgene_pcie_phy_rd(base, CMU_REG30 + offset, &dt);
	dt = REFDIV_GEN3_SET(dt, 0x1);
	dt = LOCK_COUNT_SET(dt, 0x3);
	dt = FBDIV_GEN3_SET(dt, 0x4f);
	xgene_pcie_phy_wr(base, CMU_REG30 + offset, dt);

	xgene_pcie_phy_rd(base, CMU_REG12 + offset, &dt);
	dt = STATE_DELAY9_SET(dt, 0x2);
	xgene_pcie_phy_wr(base, CMU_REG12 + offset, dt);

	xgene_pcie_phy_rd(base, CMU_REG13 + offset, &dt);
	dt = STATE_DELAY1_SET(dt, 0xf);
	dt = STATE_DELAY2_SET(dt, 0x2);
	dt = STATE_DELAY3_SET(dt, 0xd);
	dt = STATE_DELAY4_SET(dt, 0xb);
	xgene_pcie_phy_wr(base, CMU_REG13 + offset, dt);

	xgene_pcie_phy_rd(base, CMU_REG14 + offset, &dt);
	dt = STATE_DELAY5_SET(dt, 0x2);
	dt = STATE_DELAY6_SET(dt, 0x2);
	dt = STATE_DELAY7_SET(dt, 0x7);
	dt = STATE_DELAY8_SET(dt, 0xa);
	xgene_pcie_phy_wr(base, CMU_REG14 + offset, dt);

	xgene_pcie_phy_rd(base, CMU_REG32 + offset, &dt);
	dt = CAL_WAIT_SEL_SET(dt, 0x3);
	xgene_pcie_phy_wr(base, CMU_REG32 + offset, dt);

	xgene_pcie_phy_rd(base, CMU_REG31 + offset, &dt);
	dt = OVERRIDE_CH3_SET(dt, 0x1);
	dt = OVERRIDE_CH2_SET(dt, 0x1);
	dt = OVERRIDE_CH1_SET(dt, 0x1);
	dt = OVERRIDE_CH0_SET(dt, 0x1);
	xgene_pcie_phy_wr(base, CMU_REG31 + offset, dt);

	xgene_pcie_phy_rd(base, CMU_REG37 + offset, &dt);
	dt = CAL_DONE_OVR_SET(dt, 0xf);
	dt = SEARCH_DONE_OVR_SET(dt, 0xf);
	xgene_pcie_phy_wr(base, CMU_REG37 + offset, dt);

	xgene_pcie_phy_rd(base, CMU_REG27 + offset, &dt);
	dt = VOLT_SEL_CH0_SET(dt, 0x2);
	dt = VOLT_SEL_CH1_SET(dt, 0x2);
	dt = VOLT_SEL_CH2_SET(dt, 0x2);
	dt = VOLT_SEL_CH3_SET(dt, 0x2);
	xgene_pcie_phy_wr(base, CMU_REG27 + offset, dt);
}

static void xgene_pcie_adjust_phy_param(struct xgene_pcie_port *port)
{
	void *base = port->csr_base;
	u32 dt;

	xgene_pcie_in32(base + PIPECTLREG, &dt);
	dt = EQ_TX_PARAMS_VALID_SET(dt, 0);
	xgene_pcie_out32(base + PIPECTLREG, dt);

	xgene_pcie_in32(base + PIPECTLREG, &dt);
	dt = PIPECTLREG_PHY_EQ_TX_FS_SET(dt, 0x36);
	dt = PIPECTLREG_PHY_EQ_TX_LF_SET(dt, 0x10);
	dt = PIPECTLREG_PHY_EQ_TX_MAX_PRE_SET(dt, 0x6);
	dt = PIPECTLREG_PHY_EQ_TX_MAX_POST_SET(dt, 0xf);
	dt = EQ_TX_PARAMS_VALID_SET(dt, 1);
	xgene_pcie_out32(base + PIPECTLREG, dt);

	xgene_pcie_in32(base + PCIE_SDS_CTL0, &dt);
	dt = CUSTOMER_PIN_MODE_SET(dt, 0x0d25);
	xgene_pcie_out32(base + PCIE_SDS_CTL0, dt);
}

static void xgene_pcie_setup_serdes_control(void *base, u32 link_width)
{
	u32 i, j;

	if (link_width < 4) {
		xgene_pcie_serdes_cfg_LSPLL(base, 0);
		xgene_pcie_serdes_cfg_rx_ctrl(base, 0, 0);
		xgene_pcie_serdes_cfg_tx_ctrl(base, 0, 0);
		return;
	}

	for (i = 0; i < (link_width / 4); i++) {
		xgene_pcie_serdes_cfg_LSPLL(base, i);
		for (j = 0; j < 4; j++) {
			xgene_pcie_serdes_cfg_rx_ctrl(base, j, i);
			xgene_pcie_serdes_cfg_tx_ctrl(base, j, i);
		}
	}
}

static void xgene_pcie_init_phy(struct xgene_pcie_port *port)
{
	void *base = port->csr_base;
	u32 link_width = port->link_width;

	xgene_pcie_init_serdes_refclk(port);
	xgene_pcie_adjust_phy_param(port);
	xgene_pcie_serdes_pipe_config(base);
	xgene_pcie_setup_serdes_control(base, link_width);
}

static void xgene_pcie_bypass_rcvr_detect(void *base)
{
	u32 dt;

	xgene_pcie_in32(base + CFG_CONSTANTS_383_352, &dt);
	dt = BYPASS_RECEIVER_DETECTION_SET(dt, 0x1);
	xgene_pcie_out32(base + CFG_CONSTANTS_383_352, dt);
}

static void set_sds_rxd_reset(void *base, int ch, int sds2)
{

	u32 data;
	int offset = (sds2 > 0) ? SDS1_OFF : 0x0;

	xgene_pcie_phy_rd(base, CH0_RXTX_REG7 + (CH_OFF * ch) + offset, &data);
	data = RESETB_RXD_SET(data, 0x0);
	xgene_pcie_phy_wr(base, CH0_RXTX_REG7 + (CH_OFF * ch) + offset, data);

	mdelay(10);

	xgene_pcie_phy_rd(base, CH0_RXTX_REG7 + (CH_OFF * ch) + offset, &data);
	data = RESETB_RXD_SET(data, 0x1);
	xgene_pcie_phy_wr(base, CH0_RXTX_REG7 + (CH_OFF * ch) + offset, data);

}

static void lat_summer_cal(void *base, int ch, int sds2)
{

	int timeout = 0;
	int loop = 10;
	int fail_odd = 0;
	int fail_even = 0;
	int lat_do = 0;
	int lat_xo = 0;
	int lat_eo = 0;
	int lat_so = 0;
	int lat_de = 0;
	int lat_xe = 0;
	int lat_ee = 0;
	int lat_se = 0;
	int sum_cal = 0;
	int lat_do_tmp;
	int lat_xo_tmp;
	int lat_eo_tmp;
	int lat_so_tmp;
	int lat_de_tmp;
	int lat_xe_tmp;
	int lat_ee_tmp;
	int lat_se_tmp;
	int sum_cal_tmp;
	int offset = (sds2 > 0) ? SDS1_OFF : 0x0;
	u32 data;
	u32 dfe_preset;

	xgene_pcie_phy_rd(base, CH0_RXTX_REG12 + (CH_OFF * ch) + offset, &data);
	data = RX_DET_TERM_ENABLE_SET(data, 0x1);
	xgene_pcie_phy_wr(base, CH0_RXTX_REG12 + (CH_OFF * ch) + offset, data);

	xgene_pcie_phy_rd(base, CH0_RXTX_REG8 + (CH_OFF * ch) + offset, &data);
	data = CDR_LOOP_ENA_SET(data, 0x0);
	xgene_pcie_phy_wr(base, CH0_RXTX_REG8 + (CH_OFF * ch) + offset, data);

	xgene_pcie_phy_rd(base, CH0_RXTX_REG28 + (CH_OFF * ch) + offset, &data);
	data = 0;
	xgene_pcie_phy_wr(base, CH0_RXTX_REG28 + (CH_OFF * ch) + offset, data);

	xgene_pcie_phy_rd(base, CH0_RXTX_REG31 + (CH_OFF * ch) + offset,
			  &dfe_preset);
	data = 0;
	xgene_pcie_phy_wr(base, CH0_RXTX_REG31 + (CH_OFF * ch) + offset, data);

	while (loop > 0) {
		udelay(100);
		xgene_pcie_phy_rd(base,
				  CH0_RXTX_REG127 + (CH_OFF * ch) + offset,
				  &data);
		data = FORCE_SUM_CAL_START_SET(data, 0x1);
		xgene_pcie_phy_wr(base,
				  CH0_RXTX_REG127 + (CH_OFF * ch) + offset,
				  data);

		udelay(100);
		xgene_pcie_phy_rd(base,
				  CH0_RXTX_REG127 + (CH_OFF * ch) + offset,
				  &data);
		data = FORCE_SUM_CAL_START_SET(data, 0x0);
		xgene_pcie_phy_wr(base,
				  CH0_RXTX_REG127 + (CH_OFF * ch) + offset,
				  data);

		udelay(100);
		xgene_pcie_phy_rd(base,
				  CH0_RXTX_REG127 + (CH_OFF * ch) + offset,
				  &data);
		data = FORCE_LAT_CAL_START_SET(data, 0x1);
		xgene_pcie_phy_wr(base,
				  CH0_RXTX_REG127 + (CH_OFF * ch) + offset,
				  data);

		udelay(100);
		xgene_pcie_phy_rd(base,
				  CH0_RXTX_REG127 + (CH_OFF * ch) + offset,
				  &data);
		data = FORCE_LAT_CAL_START_SET(data, 0x0);
		xgene_pcie_phy_wr(base,
				  CH0_RXTX_REG127 + (CH_OFF * ch) + offset,
				  data);

		udelay(100);

		xgene_pcie_phy_rd(base, CH0_RXTX_REG21 + (CH_OFF * ch) + offset,
				  &data);
		lat_do_tmp = DO_LATCH_CALOUT_RD(data);
		lat_xo_tmp = XO_LATCH_CALOUT_RD(data);
		fail_odd = LATCH_CAL_FAIL_ODD_RD(data);

		xgene_pcie_phy_rd(base, CH0_RXTX_REG22 + (CH_OFF * ch) + offset,
				  &data);
		lat_eo_tmp = EO_LATCH_CALOUT_RD(data);
		lat_so_tmp = SO_LATCH_CALOUT_RD(data);
		fail_even = LATCH_CAL_FAIL_EVEN_RD(data);

		xgene_pcie_phy_rd(base, CH0_RXTX_REG23 + (CH_OFF * ch) + offset,
				  &data);
		lat_de_tmp = DE_LATCH_CALOUT_RD(data);
		lat_xe_tmp = XE_LATCH_CALOUT_RD(data);

		xgene_pcie_phy_rd(base, CH0_RXTX_REG24 + (CH_OFF * ch) + offset,
				  &data);
		lat_ee_tmp = EE_LATCH_CALOUT_RD(data);
		lat_se_tmp = SE_LATCH_CALOUT_RD(data);

		xgene_pcie_phy_rd(base,
				  CH0_RXTX_REG121 + (CH_OFF * ch) + offset,
				  &data);
		sum_cal_tmp = SUMOS_CAL_CODE_RD(data);

		if ((fail_even == 0 || fail_even == 1)
		    && (fail_odd == 0 || fail_odd == 1)) {
			lat_do += lat_do_tmp;
			lat_xo += lat_xo_tmp;
			lat_eo += lat_eo_tmp;
			lat_so += lat_so_tmp;
			lat_de += lat_de_tmp;
			lat_xe += lat_xe_tmp;
			lat_ee += lat_ee_tmp;
			lat_se += lat_se_tmp;
			sum_cal += sum_cal_tmp;
			loop--;
		}

		set_sds_rxd_reset(base, ch, sds2);
	}

	xgene_pcie_phy_rd(base, CH0_RXTX_REG127 + (CH_OFF * ch) + offset,
			  &data);
	data = DO_LATCH_MANCAL_SET(data, (lat_do / 10));
	data = XO_LATCH_MANCAL_SET(data, (lat_xo / 10));
	xgene_pcie_phy_wr(base, CH0_RXTX_REG127 + (CH_OFF * ch) + offset, data);

	xgene_pcie_phy_rd(base, CH0_RXTX_REG128 + (CH_OFF * ch) + offset,
			  &data);
	data = EO_LATCH_MANCAL_SET(data, (lat_eo / 10));
	data = SO_LATCH_MANCAL_SET(data, (lat_so / 10));
	xgene_pcie_phy_wr(base, CH0_RXTX_REG128 + (CH_OFF * ch) + offset, data);

	xgene_pcie_phy_rd(base, CH0_RXTX_REG129 + (CH_OFF * ch) + offset,
			  &data);
	data = DE_LATCH_MANCAL_SET(data, (lat_de / 10));
	data = XE_LATCH_MANCAL_SET(data, (lat_xe / 10));
	xgene_pcie_phy_wr(base, CH0_RXTX_REG129 + (CH_OFF * ch) + offset, data);

	xgene_pcie_phy_rd(base, CH0_RXTX_REG130 + (CH_OFF * ch) + offset,
			  &data);
	data = EE_LATCH_MANCAL_SET(data, (lat_ee / 10));
	data = SE_LATCH_MANCAL_SET(data, (lat_se / 10));
	xgene_pcie_phy_wr(base, CH0_RXTX_REG130 + (CH_OFF * ch) + offset, data);

	xgene_pcie_phy_rd(base, CH0_RXTX_REG14 + (CH_OFF * ch) + offset, &data);
	data = CLTE_LATCAL_MAN_PROG_SET(data, (sum_cal / 10));
	xgene_pcie_phy_wr(base, CH0_RXTX_REG14 + (CH_OFF * ch) + offset, data);

	xgene_pcie_phy_rd(base, CH0_RXTX_REG14 + (CH_OFF * ch) + offset, &data);
	data = CTLE_LATCAL_MAN_ENA_SET(data, 1);
	xgene_pcie_phy_wr(base, CH0_RXTX_REG14 + (CH_OFF * ch) + offset, data);

	xgene_pcie_phy_rd(base, CH0_RXTX_REG127 + (CH_OFF * ch) + offset,
			  &data);
	data = LATCH_MAN_CAL_ENA_SET(data, 1);
	xgene_pcie_phy_wr(base, CH0_RXTX_REG127 + (CH_OFF * ch) + offset, data);

	xgene_pcie_phy_rd(base, CH0_RXTX_REG12 + (CH_OFF * ch) + offset, &data);
	data = RX_DET_TERM_ENABLE_SET(data, 0x0);
	xgene_pcie_phy_wr(base, CH0_RXTX_REG12 + (CH_OFF * ch) + offset, data);

	xgene_pcie_phy_rd(base, CH0_RXTX_REG28 + (CH_OFF * ch) + offset, &data);
	data = 7;
	xgene_pcie_phy_wr(base, CH0_RXTX_REG28 + (CH_OFF * ch) + offset, data);

	xgene_pcie_phy_rd(base, CH0_RXTX_REG8 + (CH_OFF * ch) + offset, &data);
	data = CDR_LOOP_ENA_SET(data, 0x1);
	xgene_pcie_phy_wr(base, CH0_RXTX_REG8 + (CH_OFF * ch) + offset, data);

	xgene_pcie_phy_wr(base, CH0_RXTX_REG31 + (CH_OFF * ch) + offset,
			  dfe_preset);
}

void lat_summer_average_cal(void *base, int link)
{

	if (link > LNKW_X2) {
		lat_summer_cal(base, 0, 0);
		lat_summer_cal(base, 1, 0);
		lat_summer_cal(base, 2, 0);
		lat_summer_cal(base, 3, 0);
		lat_summer_cal(base, 0, 1);
		lat_summer_cal(base, 1, 1);
		lat_summer_cal(base, 2, 1);
		lat_summer_cal(base, 3, 1);
	} else {
		lat_summer_cal(base, 0, 0);
	}
}

/* Initialize APM X-Gene PCIe core serdes and phy */
int xgene_pcie_serdes_init(struct xgene_pcie_port *port)
{
	if (xgene_pcie_reset_done(port))
		return 0;

	xgene_pcie_reset_clk(port);
	xgene_pcie_init_phy(port);
	if (xgene_pcie_init_ecc(port) == 0)
		return -1;
	xgene_pcie_release_phy_reset(port->csr_base);
	if (xgene_pcie_wait_pll_lock(port) == 0)
		return -1;
	if (xgene_pcie_wait_phy_rdy(port->csr_base) == 0)
		return -1;
	xgene_pcie_manual_calib(port->csr_base, port->link_width);
	lat_summer_average_cal(port->csr_base, port->link_width);
	xgene_pcie_bypass_rcvr_detect(port->csr_base);
	return 0;
}
