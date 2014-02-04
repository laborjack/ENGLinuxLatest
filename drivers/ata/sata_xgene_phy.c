/*
 * AppliedMicro X-Gene SATA PHY driver
 *
 * Copyright (c) 2013, Applied Micro Circuits Corporation
 * Author: Loc Ho <lho@apm.com>
 *         Tuan Phan <tphan@apm.com>
 *         Suman Tripathi <stripathi@apm.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/phy/phy.h>

#undef XGENE_DBG1_CSR	/* Dump all CSR access */
#undef XGENE_DBG2_CSR	/* Dump all PHY access */

#define XGENE_PHY_DTS		"apm,xgene-ahci-phy"
#define XGENE_PHY2_DTS		"apm,xgene-ahci-phy2"

/* Max 2 channel per a PHY */
#define MAX_CHANNEL			2

/*
 * Configure Reference clock (clock type):
 *  External differential	0
 *  Internal differential	1
 *  Internal single ended	2
 */
#define SATA_CLK_EXT_DIFF		0
#define SATA_CLK_INT_DIFF		1
#define SATA_CLK_INT_SING		2

/* SATA PHY CSR block offset */
#define SATA_ETH_MUX_OFFSET		0x00007000
#define SATA_SERDES_OFFSET		0x0000A000
#define SATA_CLK_OFFSET			0x0000C000

/* SATA PHY common tunning parameters.
 *
 * These are the common tunning PHY parameter. This are here to quick
 * reference. They can be override from the control override registers.
 */
#define FBDIV_VAL_50M			0x77
#define REFDIV_VAL_50M			0x1
#define FBDIV_VAL_100M			0x3B
#define REFDIV_VAL_100M			0x0

#define DEFAULT_TXBOOST_GAIN		0x3
#define DEFAULT_TXEYEDIRECTION		0x0
#define DEFAULT_TXEYETUNING		0xa
#define DEFAULT_SPD_SEL			0x7

#define SPD_SEL_GEN3			0x7
#define SPD_SEL_GEN2			0x3
#define SPD_SEL_GEN1			0x1

/* SATA Clock/Reset CSR */
#define SATACLKENREG_ADDR		0x00000000
#define SATASRESETREG_ADDR		0x00000004
#define  SATA_MEM_RESET_MASK		0x00000020
#define  SATA_MEM_RESET_RD(src)		(((src) & 0x00000020)>>5)
#define  SATA_SDS_RESET_MASK		0x00000004
#define  SATA_CSR_RESET_MASK		0x00000001
#define  SATA_CORE_RESET_MASK		0x00000002
#define  SATA_PMCLK_RESET_MASK		0x00000010
#define  SATA_PCLK_RESET_MASK		0x00000008

/* SATA SDS CSR */
#define SATA_ENET_SDS_PCS_CTL0_ADDR	0x00000000
#define  REGSPEC_CFG_I_TX_WORDMODE0_SET(dst, src) \
		(((dst) & ~0x00070000) | (((u32)(src)<<16) & 0x00070000))
#define  REGSPEC_CFG_I_RX_WORDMODE0_SET(dst, src) \
		(((dst) & ~0x00e00000) | (((u32)(src)<<21) & 0x00e00000))
#define SATA_ENET_SDS_CTL1_ADDR		0x00000010
#define  CFG_I_SPD_SEL_CDR_OVR1_SET(dst, src) \
		(((dst) & ~0x0000000f) | (((u32)(src)) & 0x0000000f))
#define SATA_ENET_SDS_CTL0_ADDR		0x0000000c
#define  REGSPEC_CFG_I_CUSTOMER_PIN_MODE0_SET(dst, src) \
		(((dst) & ~0x00007fff) | (((u32)(src)) & 0x00007fff))
#define SATA_ENET_SDS_RST_CTL_ADDR	0x00000024
#define SATA_ENET_SDS_IND_CMD_REG_ADDR	0x0000003c
#define  CFG_IND_WR_CMD_MASK		0x00000001
#define  CFG_IND_RD_CMD_MASK		0x00000002
#define  CFG_IND_CMD_DONE_MASK		0x00000004
#define  CFG_IND_ADDR_SET(dst, src) \
		(((dst) & ~0x003ffff0) | (((u32)(src)<<4) & 0x003ffff0))
#define SATA_ENET_SDS_IND_RDATA_REG_ADDR	0x00000040
#define SATA_ENET_SDS_IND_WDATA_REG_ADDR	0x00000044
#define SATA_ENET_CLK_MACRO_REG_ADDR		0x0000004c
#define  I_RESET_B_SET(dst, src) \
		(((dst) & ~0x00000001) | (((u32)(src)) & 0x00000001))
#define  I_PLL_FBDIV_SET(dst, src) \
		(((dst) & ~0x001ff000) | (((u32)(src)<<12) & 0x001ff000))
#define  I_CUSTOMEROV_SET(dst, src) \
		(((dst) & ~0x00000f80) | (((u32)(src)<<7) & 0x00000f80))
#define  O_PLL_LOCK_RD(src)		(((src) & 0x40000000)>>30)
#define  O_PLL_READY_RD(src)		(((src) & 0x80000000)>>31)

/* SATA PHY clock CSR */
#define KC_CLKMACRO_CMU_REGS_CMU_REG0_ADDR	0x20000
#define  CMU_REG0_PDOWN_MASK			0x00004000
#define  CMU_REG0_CAL_COUNT_RESOL_SET(dst, src) \
		(((dst) & ~0x000000e0) | (((u32)(src) << 0x5) & 0x000000e0))
#define KC_CLKMACRO_CMU_REGS_CMU_REG1_ADDR	0x20002
#define  CMU_REG1_PLL_CP_SET(dst, src) \
		(((dst) & ~0x00003c00) | (((u32)(src) << 0xa) & 0x00003c00))
#define  CMU_REG1_PLL_MANUALCAL_SET(dst, src) \
		(((dst) & ~0x00000008) | (((u32)(src) << 0x3) & 0x00000008))
#define  CMU_REG1_PLL_CP_SEL_SET(dst, src) \
		(((dst) & ~0x000003e0) | (((u32)(src) << 0x5) & 0x000003e0))
#define KC_CLKMACRO_CMU_REGS_CMU_REG2_ADDR	0x20004
#define  CMU_REG2_PLL_LFRES_SET(dst, src) \
		(((dst) & ~0x0000001e) | (((u32)(src) << 0x1) & 0x0000001e))
#define  CMU_REG2_PLL_FBDIV_SET(dst, src) \
		(((dst) & ~0x00003fe0) | (((u32)(src) << 0x5) & 0x00003fe0))
#define KC_CLKMACRO_CMU_REGS_CMU_REG3_ADDR	0x20006
#define  CMU_REG3_VCOVARSEL_SET(dst, src) \
		(((dst) & ~0x0000000f) | (((u32)(src) << 0x0) & 0x0000000f))
#define  CMU_REG3_VCO_MOMSEL_INIT_SET(dst, src) \
		(((dst) & ~0x000003f0) | (((u32)(src) << 0x4) & 0x000003f0))
#define KC_CLKMACRO_CMU_REGS_CMU_REG4_ADDR	0x20008
#define KC_CLKMACRO_CMU_REGS_CMU_REG5_ADDR	0x2000a
#define  CMU_REG5_PLL_LFSMCAP_SET(dst, src) \
		(((dst) & ~0x0000c000) | (((u32)(src) << 0xe) & 0x0000c000))
#define  CMU_REG5_PLL_LOCK_RESOLUTION_SET(dst, src) \
		(((dst) & ~0x0000000e) | (((u32)(src) << 0x1) & 0x0000000e))
#define  CMU_REG5_PLL_LFCAP_SET(dst, src) \
		(((dst) & ~0x00003000) | (((u32)(src) << 0xc) & 0x00003000))
#define KC_CLKMACRO_CMU_REGS_CMU_REG6_ADDR	0x2000c
#define  CMU_REG6_PLL_VREGTRIM_SET(dst, src) \
		(((dst) & ~0x00000600) | (((u32)(src) << 0x9) & 0x00000600))
#define  CMU_REG6_MAN_PVT_CAL_SET(dst, src) \
		(((dst) & ~0x00000004) | (((u32)(src) << 0x2) & 0x00000004))
#define KC_CLKMACRO_CMU_REGS_CMU_REG7_ADDR	0x2000e
#define  CMU_REG7_PLL_CALIB_DONE_RD(src) \
		((0x00004000 & (u32)(src)) >> 0xe)
#define  CMU_REG7_VCO_CAL_FAIL_RD(src) \
		((0x00000c00 & (u32)(src)) >> 0xa)
#define KC_CLKMACRO_CMU_REGS_CMU_REG8_ADDR	0x20010
#define KC_CLKMACRO_CMU_REGS_CMU_REG9_ADDR	0x20012
#define KC_CLKMACRO_CMU_REGS_CMU_REG10_ADDR	0x20014
#define KC_CLKMACRO_CMU_REGS_CMU_REG11_ADDR	0x20016
#define KC_CLKMACRO_CMU_REGS_CMU_REG12_ADDR	0x20018
#define KC_CLKMACRO_CMU_REGS_CMU_REG13_ADDR	0x2001a
#define KC_CLKMACRO_CMU_REGS_CMU_REG14_ADDR	0x2001c
#define KC_CLKMACRO_CMU_REGS_CMU_REG15_ADDR	0x2001e
#define KC_CLKMACRO_CMU_REGS_CMU_REG16_ADDR	0x20020
#define  CMU_REG16_PVT_DN_MAN_ENA_MASK		0x00000001
#define  CMU_REG16_PVT_UP_MAN_ENA_MASK		0x00000002
#define  CMU_REG16_VCOCAL_WAIT_BTW_CODE_SET(dst, src) \
		(((dst) & ~0x0000001c) | (((u32)(src) << 0x2) & 0x0000001c))
#define  CMU_REG16_CALIBRATION_DONE_OVERRIDE_SET(dst, src) \
		(((dst) & ~0x00000040) | (((u32)(src) << 0x6) & 0x00000040))
#define  CMU_REG16_BYPASS_PLL_LOCK_SET(dst, src) \
		(((dst) & ~0x00000020) | (((u32)(src) << 0x5) & 0x00000020))
#define KC_CLKMACRO_CMU_REGS_CMU_REG17_ADDR               0x20022
#define  CMU_REG17_PVT_CODE_R2A_SET(dst, src) \
		(((dst) & ~0x00007f00) | (((u32)(src) << 0x8) & 0x00007f00))
#define  CMU_REG17_RESERVED_7_SET(dst, src) \
		(((dst) & ~0x000000e0) | (((u32)(src) << 0x5) & 0x000000e0))
#define  CMU_REG17_PVT_TERM_MAN_ENA_MASK			0x00008000
#define KC_CLKMACRO_CMU_REGS_CMU_REG18_ADDR	0x20024
#define KC_CLKMACRO_CMU_REGS_CMU_REG19_ADDR	0x20026
#define KC_CLKMACRO_CMU_REGS_CMU_REG20_ADDR	0x20028
#define KC_CLKMACRO_CMU_REGS_CMU_REG21_ADDR	0x2002a
#define KC_CLKMACRO_CMU_REGS_CMU_REG22_ADDR	0x2002c
#define KC_CLKMACRO_CMU_REGS_CMU_REG23_ADDR	0x2002e
#define KC_CLKMACRO_CMU_REGS_CMU_REG24_ADDR	0x20030
#define KC_CLKMACRO_CMU_REGS_CMU_REG25_ADDR	0x20032
#define KC_CLKMACRO_CMU_REGS_CMU_REG26_ADDR	0x20034
#define  CMU_REG26_FORCE_PLL_LOCK_SET(dst, src) \
		(((dst) & ~0x00000001) | (((u32)(src) << 0x0) & 0x00000001))
#define KC_CLKMACRO_CMU_REGS_CMU_REG27_ADDR	0x20036
#define KC_CLKMACRO_CMU_REGS_CMU_REG28_ADDR	0x20038
#define KC_CLKMACRO_CMU_REGS_CMU_REG29_ADDR	0x2003a
#define KC_CLKMACRO_CMU_REGS_CMU_REG30_ADDR	0x2003c
#define  CMU_REG30_LOCK_COUNT_SET(dst, src) \
		(((dst) & ~0x00000006) | (((u32)(src) << 0x1) & 0x00000006))
#define  CMU_REG30_PCIE_MODE_SET(dst, src) \
		(((dst) & ~0x00000008) | (((u32)(src) << 0x3) & 0x00000008))
#define KC_CLKMACRO_CMU_REGS_CMU_REG31_ADDR	0x2003e
#define KC_CLKMACRO_CMU_REGS_CMU_REG32_ADDR	0x20040
#define  CMU_REG32_FORCE_VCOCAL_START_MASK	0x00004000
#define  CMU_REG32_PVT_CAL_WAIT_SEL_SET(dst, src) \
		(((dst) & ~0x00000006) | (((u32)(src) << 0x1) & 0x00000006))
#define  CMU_REG32_IREF_ADJ_SET(dst, src) \
		(((dst) & ~0x00000180) | (((u32)(src) << 0x7) & 0x00000180))
#define KC_CLKMACRO_CMU_REGS_CMU_REG33_ADDR	0x20042
#define KC_CLKMACRO_CMU_REGS_CMU_REG34_ADDR	0x20044
#define  CMU_REG34_VCO_CAL_VTH_LO_MAX_SET(dst, src) \
		(((dst) & ~0x0000000f) | (((u32)(src) << 0x0) & 0x0000000f))
#define  CMU_REG34_VCO_CAL_VTH_HI_MAX_SET(dst, src) \
		(((dst) & ~0x00000f00) | (((u32)(src) << 0x8) & 0x00000f00))
#define  CMU_REG34_VCO_CAL_VTH_LO_MIN_SET(dst, src) \
		(((dst) & ~0x000000f0) | (((u32)(src) << 0x4) & 0x000000f0))
#define  CMU_REG34_VCO_CAL_VTH_HI_MIN_SET(dst, src) \
		(((dst) & ~0x0000f000) | (((u32)(src) << 0xc) & 0x0000f000))
#define KC_SERDES_CMU_REGS_CMU_REG35_ADDR	0x46
#define  CMU_REG35_PLL_SSC_MOD_SET(dst, src) \
		(((dst) & ~0x0000fe00) | (((u32)(src) << 0x9) & 0x0000fe00))
#define KC_SERDES_CMU_REGS_CMU_REG36_ADDR	0x48
#define  CMU_REG36_PLL_SSC_EN_SET(dst, src) \
		(((dst) & ~0x00000010) | (((u32)(src) << 0x4) & 0x00000010))
#define  CMU_REG36_PLL_SSC_VSTEP_SET(dst, src) \
		(((dst) & ~0x0000ffc0) | (((u32)(src) << 0x6) & 0x0000ffc0))
#define  CMU_REG36_PLL_SSC_DSMSEL_SET(dst, src) \
		(((dst) & ~0x00000020) | (((u32)(src) << 0x5) & 0x00000020))
#define KC_CLKMACRO_CMU_REGS_CMU_REG35_ADDR	0x20046
#define KC_CLKMACRO_CMU_REGS_CMU_REG36_ADDR	0x20048
#define KC_CLKMACRO_CMU_REGS_CMU_REG37_ADDR	0x2004a
#define KC_CLKMACRO_CMU_REGS_CMU_REG38_ADDR	0x2004c
#define KC_CLKMACRO_CMU_REGS_CMU_REG39_ADDR	0x2004e

/* SATA PHY RXTX CSR */
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG0_ADDR	0x400
#define  CH0_RXTX_REG0_CTLE_EQ_HR_SET(dst, src) \
		(((dst) & ~0x0000f800) | (((u32)(src) << 0xb) & 0x0000f800))
#define  CH0_RXTX_REG0_CTLE_EQ_QR_SET(dst, src) \
		(((dst) & ~0x000007c0) | (((u32)(src) << 0x6) & 0x000007c0))
#define  CH0_RXTX_REG0_CTLE_EQ_FR_SET(dst, src) \
		(((dst) & ~0x0000003e) | (((u32)(src) << 0x1) & 0x0000003e))
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG1_ADDR	0x402
#define  CH0_RXTX_REG1_RXACVCM_SET(dst, src) \
		(((dst) & ~0x0000f000) | (((u32)(src) << 0xc) & 0x0000f000))
#define  CH0_RXTX_REG1_CTLE_EQ_SET(dst, src) \
		(((dst) & ~0x00000f80) | (((u32)(src) << 0x7) & 0x00000f80))
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG2_ADDR	0x404
#define  CH0_RXTX_REG2_VTT_ENA_SET(dst, src) \
		(((dst) & ~0x00000100) | (((u32)(src) << 0x8) & 0x00000100))
#define  CH0_RXTX_REG2_TX_FIFO_ENA_SET(dst, src) \
		(((dst) & ~0x00000020) | (((u32)(src) << 0x5) & 0x00000020))
#define  CH0_RXTX_REG2_VTT_SEL_SET(dst, src) \
		(((dst) & ~0x000000c0) | (((u32)(src) << 0x6) & 0x000000c0))
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG4_ADDR	0x408
#define  CH0_RXTX_REG4_TX_LOOPBACK_BUF_EN_MASK			0x00000040
#define  CH0_RXTX_REG4_TX_DATA_RATE_SET(dst, src) \
		(((dst) & ~0x0000c000) | (((u32)(src) << 0xe) & 0x0000c000))
#define  CH0_RXTX_REG4_TX_WORD_MODE_SET(dst, src) \
		(((dst) & ~0x00003800) | (((u32)(src) << 0xb) & 0x00003800))
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG5_ADDR	0x40a
#define  CH0_RXTX_REG5_TX_CN1_SET(dst, src) \
		(((dst) & ~0x0000f800) | (((u32)(src) << 0xb) & 0x0000f800))
#define  CH0_RXTX_REG5_TX_CP1_SET(dst, src) \
		(((dst) & ~0x000007e0) | (((u32)(src) << 0x5) & 0x000007e0))
#define  CH0_RXTX_REG5_TX_CN2_SET(dst, src) \
		(((dst) & ~0x0000001f) | (((u32)(src) << 0x0) & 0x0000001f))
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG6_ADDR	0x40c
#define  CH0_RXTX_REG6_TXAMP_CNTL_SET(dst, src) \
		(((dst) & ~0x00000780) | (((u32)(src) << 0x7) & 0x00000780))
#define  CH0_RXTX_REG6_TXAMP_ENA_SET(dst, src) \
		(((dst) & ~0x00000040) | (((u32)(src) << 0x6) & 0x00000040))
#define  CH0_RXTX_REG6_RX_BIST_ERRCNT_RD_SET(dst, src) \
		(((dst) & ~0x00000001) | (((u32)(src) << 0x0) & 0x00000001))
#define  CH0_RXTX_REG6_TX_IDLE_SET(dst, src) \
		(((dst) & ~0x00000008) | (((u32)(src) << 0x3) & 0x00000008))
#define  CH0_RXTX_REG6_RX_BIST_RESYNC_SET(dst, src) \
		(((dst) & ~0x00000002) | (((u32)(src) << 0x1) & 0x00000002))
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG7_ADDR	0x40e
#define  CH0_RXTX_REG7_RESETB_RXD_MASK			0x00000100
#define  CH0_RXTX_REG7_RESETB_RXA_MASK			0x00000080
#define  CH0_RXTX_REG7_BIST_ENA_RX_SET(dst, src) \
		(((dst) & ~0x00000040) | (((u32)(src) << 0x6) & 0x00000040))
#define  CH0_RXTX_REG7_RX_WORD_MODE_SET(dst, src) \
		(((dst) & ~0x00003800) | (((u32)(src) << 0xb) & 0x00003800))
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG8_ADDR	0x410
#define  CH0_RXTX_REG8_CDR_LOOP_ENA_SET(dst, src) \
		(((dst) & ~0x00004000) | (((u32)(src) << 0xe) & 0x00004000))
#define  CH0_RXTX_REG8_CDR_BYPASS_RXLOS_SET(dst, src) \
		(((dst) & ~0x00000800) | (((u32)(src) << 0xb) & 0x00000800))
#define  CH0_RXTX_REG8_SSC_ENABLE_SET(dst, src) \
		(((dst) & ~0x00000200) | (((u32)(src) << 0x9) & 0x00000200))
#define  CH0_RXTX_REG8_SD_VREF_SET(dst, src) \
		(((dst) & ~0x000000f0) | (((u32)(src) << 0x4) & 0x000000f0))
#define  CH0_RXTX_REG8_SD_DISABLE_SET(dst, src) \
		(((dst) & ~0x00000100) | (((u32)(src) << 0x8) & 0x00000100))
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG7_ADDR	0x40e
#define  CH0_RXTX_REG7_RESETB_RXD_SET(dst, src) \
		(((dst) & ~0x00000100) | (((u32)(src) << 0x8) & 0x00000100))
#define  CH0_RXTX_REG7_RESETB_RXA_SET(dst, src) \
		(((dst) & ~0x00000080) | (((u32)(src) << 0x7) & 0x00000080))
#define  CH0_RXTX_REG7_LOOP_BACK_ENA_CTLE_MASK			0x00004000
#define  CH0_RXTX_REG7_LOOP_BACK_ENA_CTLE_SET(dst, src) \
		(((dst) & ~0x00004000) | (((u32)(src) << 0xe) & 0x00004000))
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG11_ADDR	0x416
#define  CH0_RXTX_REG11_PHASE_ADJUST_LIMIT_SET(dst, src) \
		(((dst) & ~0x0000f800) | (((u32)(src) << 0xb) & 0x0000f800))
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG12_ADDR	0x418
#define  CH0_RXTX_REG12_LATCH_OFF_ENA_SET(dst, src) \
		(((dst) & ~0x00002000) | (((u32)(src) << 0xd) & 0x00002000))
#define  CH0_RXTX_REG12_SUMOS_ENABLE_SET(dst, src) \
		(((dst) & ~0x00000004) | (((u32)(src) << 0x2) & 0x00000004))
#define  CH0_RXTX_REG12_RX_DET_TERM_ENABLE_MASK		0x00000002
#define  CH0_RXTX_REG12_RX_DET_TERM_ENABLE_SET(dst, src) \
		(((dst) & ~0x00000002) | (((u32)(src) << 0x1) & 0x00000002))
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG13_ADDR	0x41a
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG14_ADDR	0x41c
#define CH0_RXTX_REG14_CLTE_LATCAL_MAN_PROG_SET(dst, src) \
		(((dst) & ~0x0000003f) | (((u32)(src) << 0x0) & 0x0000003f))
#define CH0_RXTX_REG14_CTLE_LATCAL_MAN_ENA_SET(dst, src) \
		(((dst) & ~0x00000040) | (((u32)(src) << 0x6) & 0x00000040))
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG26_ADDR	0x434
#define  CH0_RXTX_REG26_PERIOD_ERROR_LATCH_SET(dst, src) \
		(((dst) & ~0x00003800) | (((u32)(src) << 0xb) & 0x00003800))
#define  CH0_RXTX_REG26_BLWC_ENA_SET(dst, src) \
		(((dst) & ~0x00000008) | (((u32)(src) << 0x3) & 0x00000008))
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG21_ADDR	0x42a
#define  CH0_RXTX_REG21_DO_LATCH_CALOUT_RD(src) \
		((0x0000fc00 & (u32)(src)) >> 0xa)
#define  CH0_RXTX_REG21_XO_LATCH_CALOUT_RD(src) \
		((0x000003f0 & (u32)(src)) >> 0x4)
#define  CH0_RXTX_REG21_LATCH_CAL_FAIL_ODD_RD(src) \
		((0x0000000f & (u32)(src)))
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG22_ADDR	0x42c
#define  CH0_RXTX_REG22_SO_LATCH_CALOUT_RD(src) \
		((0x000003f0 & (u32)(src)) >> 0x4)
#define  CH0_RXTX_REG22_EO_LATCH_CALOUT_RD(src) \
		((0x0000fc00 & (u32)(src)) >> 0xa)
#define  CH0_RXTX_REG22_LATCH_CAL_FAIL_EVEN_RD(src) \
		((0x0000000f & (u32)(src)))
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG23_ADDR	0x42e
#define  CH0_RXTX_REG23_DE_LATCH_CALOUT_RD(src) \
		((0x0000fc00 & (u32)(src)) >> 0xa)
#define  CH0_RXTX_REG23_XE_LATCH_CALOUT_RD(src) \
		((0x000003f0 & (u32)(src)) >> 0x4)
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG24_ADDR	0x430
#define  CH0_RXTX_REG24_EE_LATCH_CALOUT_RD(src) \
		((0x0000fc00 & (u32)(src)) >> 0xa)
#define  CH0_RXTX_REG24_SE_LATCH_CALOUT_RD(src) \
		((0x000003f0 & (u32)(src)) >> 0x4)
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG27_ADDR	0x436
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG28_ADDR	0x438
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG31_ADDR	0x43e
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG38_ADDR	0x44c
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG39_ADDR	0x44e
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG40_ADDR	0x450
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG41_ADDR	0x452
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG42_ADDR	0x454
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG43_ADDR	0x456
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG44_ADDR	0x458
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG45_ADDR	0x45a
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG46_ADDR	0x45c
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG47_ADDR	0x45e
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG48_ADDR	0x460
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG49_ADDR	0x462
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG50_ADDR	0x464
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG51_ADDR	0x466
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG52_ADDR	0x468
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG53_ADDR	0x46a
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG54_ADDR	0x46c
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG55_ADDR	0x46e
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG61_ADDR	0x47a
#define  CH0_RXTX_REG61_ISCAN_INBERT_SET(dst, src) \
		(((dst) & ~0x00000010) | (((u32)(src) << 0x4) & 0x00000010))
#define  CH0_RXTX_REG61_LOADFREQ_SHIFT_SET(dst, src) \
		(((dst) & ~0x00000008) | (((u32)(src) << 0x3) & 0x00000008))
#define  CH0_RXTX_REG61_EYE_COUNT_WIDTH_SEL_SET(dst, src) \
		(((dst) & ~0x000000c0) | (((u32)(src) << 0x6) & 0x000000c0))
#define  CH0_RXTX_REG61_SPD_SEL_CDR_SET(dst, src) \
		(((dst) & ~0x00003c00) | (((u32)(src) << 0xa) & 0x00003c00))
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG62_ADDR	0x47c
#define  CH0_RXTX_REG62_PERIOD_H1_QLATCH_SET(dst, src) \
		(((dst) & ~0x00003800) | (((u32)(src) << 0xb) & 0x00003800))
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG81_ADDR	0x4a2

#define  CH0_RXTX_REG89_MU_TH7_SET(dst, src) \
		(((dst) & ~0x0000f800) | (((u32)(src) << 0xb) & 0x0000f800))
#define  CH0_RXTX_REG89_MU_TH8_SET(dst, src) \
		(((dst) & ~0x000007c0) | (((u32)(src) << 0x6) & 0x000007c0))
#define  CH0_RXTX_REG89_MU_TH9_SET(dst, src) \
		(((dst) & ~0x0000003e) | (((u32)(src) << 0x1) & 0x0000003e))
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG96_ADDR	0x4c0
#define  CH0_RXTX_REG96_MU_FREQ1_SET(dst, src) \
		(((dst) & ~0x0000f800) | (((u32)(src) << 0xb) & 0x0000f800))
#define  CH0_RXTX_REG96_MU_FREQ2_SET(dst, src) \
		(((dst) & ~0x000007c0) | (((u32)(src) << 0x6) & 0x000007c0))
#define  CH0_RXTX_REG96_MU_FREQ3_SET(dst, src) \
		(((dst) & ~0x0000003e) | (((u32)(src) << 0x1) & 0x0000003e))
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG99_ADDR	0x4c6
#define  CH0_RXTX_REG99_MU_PHASE1_SET(dst, src) \
		(((dst) & ~0x0000f800) | (((u32)(src) << 0xb) & 0x0000f800))
#define  CH0_RXTX_REG99_MU_PHASE2_SET(dst, src) \
		(((dst) & ~0x000007c0) | (((u32)(src) << 0x6) & 0x000007c0))
#define  CH0_RXTX_REG99_MU_PHASE3_SET(dst, src) \
		(((dst) & ~0x0000003e) | (((u32)(src) << 0x1) & 0x0000003e))
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG102_ADDR	0x4cc
#define  CH0_RXTX_REG102_FREQLOOP_LIMIT_SET(dst, src) \
		(((dst) & ~0x00000060) | (((u32)(src) << 0x5) & 0x00000060))
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG114_ADDR	0x4e4
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG121_ADDR	0x4f2
#define  CH0_RXTX_REG121_SUMOS_CAL_CODE_RD(src) \
		((0x0000003e & (u32)(src)) >> 0x1)
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG125_ADDR	0x4fa
#define  CH0_RXTX_REG125_PQ_REG_SET(dst, src) \
		(((dst) & ~0x0000fe00) | (((u32)(src) << 0x9) & 0x0000fe00))
#define  CH0_RXTX_REG125_SIGN_PQ_SET(dst, src) \
		(((dst) & ~0x00000100) | (((u32)(src) << 0x8) & 0x00000100))
#define  CH0_RXTX_REG125_SIGN_PQ_2C_SET(dst, src) \
		(((dst) & ~0x00000080) | (((u32)(src) << 0x7) & 0x00000080))
#define  CH0_RXTX_REG125_PHZ_MANUALCODE_SET(dst, src) \
		(((dst) & ~0x0000007c) | (((u32)(src) << 0x2) & 0x0000007c))
#define  CH0_RXTX_REG125_PHZ_MANUAL_SET(dst, src) \
		(((dst) & ~0x00000002) | (((u32)(src) << 0x1) & 0x00000002))
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG127_ADDR	0x4fe
#define  CH0_RXTX_REG127_FORCE_SUM_CAL_START_MASK	0x00000002
#define  CH0_RXTX_REG127_FORCE_LAT_CAL_START_MASK	0x00000004
#define KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG127_ADDR	0x6fe
#define  CH1_RXTX_REG127_FORCE_SUM_CAL_START_SET(dst, src) \
		(((dst) & ~0x00000002) | (((u32)(src) << 0x1) & 0x00000002))
#define  CH1_RXTX_REG127_FORCE_LAT_CAL_START_SET(dst, src) \
		(((dst) & ~0x00000004) | (((u32)(src) << 0x2) & 0x00000004))
#define  CH0_RXTX_REG127_LATCH_MAN_CAL_ENA_SET(dst, src) \
		(((dst) & ~0x00000008) | (((u32)(src) << 0x3) & 0x00000008))
#define  CH0_RXTX_REG127_DO_LATCH_MANCAL_SET(dst, src) \
		(((dst) & ~0x0000fc00) | (((u32)(src) << 0xa) & 0x0000fc00))
#define  CH0_RXTX_REG127_XO_LATCH_MANCAL_SET(dst, src) \
		(((dst) & ~0x000003f0) | (((u32)(src) << 0x4) & 0x000003f0))
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG128_ADDR	0x500
#define  CH0_RXTX_REG128_LATCH_CAL_WAIT_SEL_SET(dst, src) \
		(((dst) & ~0x0000000c) | (((u32)(src) << 0x2) & 0x0000000c))
#define  CH0_RXTX_REG128_EO_LATCH_MANCAL_SET(dst, src) \
		(((dst) & ~0x0000fc00) | (((u32)(src) << 0xa) & 0x0000fc00))
#define CH0_RXTX_REG128_SO_LATCH_MANCAL_SET(dst, src) \
		(((dst) & ~0x000003f0) | (((u32)(src) << 0x4) & 0x000003f0))
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG129_ADDR	0x502
#define CH0_RXTX_REG129_DE_LATCH_MANCAL_SET(dst, src) \
		(((dst) & ~0x0000fc00) | (((u32)(src) << 0xa) & 0x0000fc00))
#define CH0_RXTX_REG129_XE_LATCH_MANCAL_SET(dst, src) \
		(((dst) & ~0x000003f0) | (((u32)(src) << 0x4) & 0x000003f0))
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG130_ADDR	0x504
#define  CH0_RXTX_REG130_EE_LATCH_MANCAL_SET(dst, src) \
		(((dst) & ~0x0000fc00) | (((u32)(src) << 0xa) & 0x0000fc00))
#define  CH0_RXTX_REG130_SE_LATCH_MANCAL_SET(dst, src) \
		(((dst) & ~0x000003f0) | (((u32)(src) << 0x4) & 0x000003f0))
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG145_ADDR	0x522
#define  CH0_RXTX_REG145_TX_IDLE_SATA_SET(dst, src) \
		(((dst) & ~0x00000001) | (((u32)(src) << 0x0) & 0x00000001))
#define  CH0_RXTX_REG145_RXES_ENA_SET(dst, src) \
		(((dst) & ~0x00000002) | (((u32)(src) << 0x1) & 0x00000002))
#define  CH0_RXTX_REG145_RXDFE_CONFIG_SET(dst, src) \
		(((dst) & ~0x0000c000) | (((u32)(src) << 0xe) & 0x0000c000))
#define  CH0_RXTX_REG145_RXVWES_LATENA_SET(dst, src) \
		(((dst) & ~0x00000004) | (((u32)(src) << 0x2) & 0x00000004))
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG147_ADDR	0x526
#define KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG148_ADDR	0x528

#define KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG4_ADDR	0x608
#define  CH1_RXTX_REG4_TX_LOOPBACK_BUF_EN_SET(dst, src) \
		(((dst) & ~0x00000040) | (((u32)(src) << 0x6) & 0x00000040))
#define KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG7_ADDR	0x60e
#define KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG13_ADDR	0x61a
#define KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG38_ADDR	0x64c
#define KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG39_ADDR	0x64e
#define KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG40_ADDR	0x650
#define KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG41_ADDR	0x652
#define KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG42_ADDR	0x654
#define KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG43_ADDR	0x656
#define KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG44_ADDR	0x658
#define KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG45_ADDR	0x65a
#define KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG46_ADDR	0x65c
#define KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG47_ADDR	0x65e
#define KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG48_ADDR	0x660
#define KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG49_ADDR	0x662
#define KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG50_ADDR	0x664
#define KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG51_ADDR	0x666
#define KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG52_ADDR	0x668
#define KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG53_ADDR	0x66a
#define KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG54_ADDR	0x66c
#define KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG55_ADDR	0x66e
#define KC_SERDES_X2_RXTX_REGS_CH1_RXTX_REG121_ADDR	0x6f2

/* SATA/ENET Shared CSR */
#define SATA_ENET_CONFIG_REG_ADDR		0x00000000
#define  CFG_SATA_ENET_SELECT_MASK		0x00000001

/* SATA SERDES CMU CSR */
#define KC_SERDES_CMU_REGS_CMU_REG0_ADDR	0x0
#define  CMU_REG0_PLL_REF_SEL_MASK		0x00002000
#define CMU_REG0_PLL_REF_SEL_SHIFT_MASK		0xd
#define CMU_REG0_PLL_REF_SEL_SET(dst, src)	\
		(((dst) & ~0x00002000) | (((u32)(src) << 0xd) & 0x00002000))
#define KC_SERDES_CMU_REGS_CMU_REG1_ADDR	0x2
#define  CMU_REG1_REFCLK_CMOS_SEL_MASK		0x00000001
#define CMU_REG1_REFCLK_CMOS_SEL_SHIFT_MASK	0x0
#define CMU_REG1_REFCLK_CMOS_SEL_SET(dst, src)	\
		(((dst) & ~0x00000001) | (((u32)(src) << 0x0) & 0x00000001))
#define KC_SERDES_CMU_REGS_CMU_REG2_ADDR	0x4
#define  CMU_REG2_PLL_REFDIV_SET(dst, src) \
		(((dst) & ~0x0000c000) | (((u32)(src) << 0xe) & 0x0000c000))
#define KC_SERDES_CMU_REGS_CMU_REG3_ADDR	0x6
#define  CMU_REG3_VCO_MANMOMSEL_SET(dst, src) \
		(((dst) & ~0x0000fc00) | (((u32)(src) << 0xa) & 0x0000fc00))
#define KC_SERDES_CMU_REGS_CMU_REG5_ADDR	0xa
#define  CMU_REG5_PLL_RESETB_MASK		0x00000001
#define KC_SERDES_CMU_REGS_CMU_REG6_ADDR	0xc
#define KC_SERDES_CMU_REGS_CMU_REG7_ADDR	0xe
#define KC_SERDES_CMU_REGS_CMU_REG9_ADDR	0x12
#define  CMU_REG9_TX_WORD_MODE_CH1_SET(dst, src) \
		(((dst) & ~0x00000380) | (((u32)(src) << 0x7) & 0x00000380))
#define  CMU_REG9_TX_WORD_MODE_CH0_SET(dst, src) \
		(((dst) & ~0x00000070) | (((u32)(src) << 0x4) & 0x00000070))
#define  CMU_REG9_PLL_POST_DIVBY2_SET(dst, src) \
		(((dst) & ~0x00000008) | (((u32)(src) << 0x3) & 0x00000008))
#define KC_SERDES_CMU_REGS_CMU_REG12_ADDR	0x18
#define  CMU_REG12_STATE_DELAY9_SET(dst, src) \
		(((dst) & ~0x000000f0) | (((u32)(src) << 0x4) & 0x000000f0))
#define KC_SERDES_CMU_REGS_CMU_REG13_ADDR	0x1a
#define KC_SERDES_CMU_REGS_CMU_REG14_ADDR	0x1c
#define KC_SERDES_CMU_REGS_CMU_REG15_ADDR	0x1e
#define KC_SERDES_CMU_REGS_CMU_REG16_ADDR	0x20
#define KC_SERDES_CMU_REGS_CMU_REG17_ADDR	0x22
#define KC_SERDES_CMU_REGS_CMU_REG26_ADDR	0x34
#define KC_SERDES_CMU_REGS_CMU_REG30_ADDR	0x3c
#define KC_SERDES_CMU_REGS_CMU_REG31_ADDR	0x3e
#define KC_SERDES_CMU_REGS_CMU_REG32_ADDR	0x40
#define  CMU_REG32_FORCE_VCOCAL_START_MASK	0x00004000
#define KC_SERDES_CMU_REGS_CMU_REG34_ADDR	0x44
#define KC_SERDES_CMU_REGS_CMU_REG37_ADDR	0x4a

/* SATA Serdes CSR using PCIE clock macro */
#define SM_PCIE_CLKRST_CSR_PCIE_SRST_ADDR	0xc000
#define SM_PCIE_CLKRST_CSR_PCIE_CLKEN_ADDR	0xc008
#define SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_IND_WDATA_REG_ADDR	0xa01c
#define SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_IND_CMD_REG_ADDR	0xa014
#define  PCIE_SDS_IND_CMD_REG_CFG_IND_ADDR_SET(dst, src) \
		(((dst) & ~0x003ffff0) | (((u32)(src) << 0x4) & 0x003ffff0))
#define  PCIE_SDS_IND_CMD_REG_CFG_IND_WR_CMD_SET(dst, src) \
		(((dst) & ~0x00000001) | (((u32)(src) << 0x0) & 0x00000001))
#define SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_IND_RDATA_REG_ADDR	0xa018
#define SM_PCIE_X8_SDS_CSR_REGS_PCIE_CLK_MACRO_REG_ADDR		0xa094
#define  PCIE_SDS_IND_CMD_REG_CFG_IND_CMD_DONE_RD(src) \
		((0x00000004 & (uint32_t)(src)) >> 0x2)
#define  PCIE_SDS_IND_CMD_REG_CFG_IND_CMD_DONE_SET(dst, src) \
		(((dst) & ~0x00000004) | (((u32)(src) << 0x2) & 0x00000004))
#define  PCIE_SDS_IND_CMD_REG_CFG_IND_RD_CMD_SET(dst, src) \
		(((dst) & ~0x00000002) | (((u32)(src) << 0x1) & 0x00000002))
#define PCIE_CLK_MACRO_REG_I_RESET_B_SET(dst, src) \
		(((dst) & ~0x00000001) | (((u32)(src) << 0x0) & 0x00000001))
#define PCIE_CLK_MACRO_REG_I_CUSTOMEROV_SET(dst, src) \
		(((dst) & ~0x00000f80) | (((u32)(src) << 0x7) & 0x00000f80))
#define PCIE_CLK_MACRO_REG_O_PLL_READY_RD(src) \
		((0x80000000 & (u32)(src)) >> 0x1f)
#define PCIE_CLK_MACRO_REG_I_PLL_FBDIV_SET(dst, src) \
		(((dst) & ~0x001ff000) | (((u32)(src) << 0xc) & 0x001ff000))
#define PCIE_CLK_MACRO_REG_O_PLL_LOCK_RD(src)	\
		((0x40000000 & (u32)(src)) >> 0x1e)

struct xgene_ahci_phy_ctx {
	struct device *dev;
	struct phy *phy;
	u64 csr_phys;		/* Physical address of PHY CSR base addr */
	void *csr_base;		/* PHY CSR base addr */
	void *pcie_base;	/* PHY CSR base addr with PCIE clock macro */

	/* Override Serdes parameters */
	u32 speed[MAX_CHANNEL]; /* Index for override parameter per channel */
	u32 txspeed[3]; 	/* Tx speed */
	u32 txboostgain[MAX_CHANNEL*3];	/* Tx freq boost and gain control */
	u32 txeyetuning[MAX_CHANNEL*3]; /* Tx eye tuning */
	u32 txeyedirection[MAX_CHANNEL*3]; /* Tx eye tuning direction */
};

/* Manual calibration is required for chip that is earlier than A3.
   To enable, pass boot argument sata_xgene_phy.manual=1 */
static int enable_manual_cal;
MODULE_PARM_DESC(manual, "Enable manual calibration (1=enable 0=disable)");
module_param_named(manual, enable_manual_cal, int, 0444);

static void phy_rd(void *addr, u32 *val)
{
	*val = readl(addr);
#if defined(XGENE_DBG1_CSR)
	pr_debug("SATAPHY CSR RD: 0x%p value: 0x%08x\n", addr, *val);
#endif
}

static void phy_wr(void *addr, u32 val)
{
	writel(val, addr);
#if defined(XGENE_DBG1_CSR)
	pr_debug("SATAPHY CSR WR: 0x%p value: 0x%08x\n", addr, val);
#endif
}

static void phy_wr_flush(void *addr, u32 val)
{
	writel(val, addr);
#if defined(XGENE_DBG1_CSR)
	pr_debug("SATAPHY CSR WR: 0x%p value: 0x%08x\n", addr, val);
#endif
	val = readl(addr);	/* Force an barrier */
}

static void serdes_wr(void *csr_base, u32 indirect_cmd_reg,
		      u32 indirect_data_reg, u32 addr, u32 data)
{
	u32 val;
	u32 cmd;

	cmd = CFG_IND_WR_CMD_MASK | CFG_IND_CMD_DONE_MASK;
	cmd = (addr << 4) | cmd;
	phy_wr(csr_base + indirect_data_reg, data);
	phy_rd(csr_base + indirect_data_reg, &val); /* Force a barrier */
	phy_wr(csr_base + indirect_cmd_reg, cmd);
	phy_rd(csr_base + indirect_cmd_reg, &val); /* Force a barrier */
	/* Add an barrier - very important please don't remove */
	mb();
	/* Ignore first read */
	phy_rd(csr_base + indirect_cmd_reg, &val);
	/* Allow Serdes to get reflected. This is required! */
	udelay(100);
	do {
		phy_rd(csr_base + indirect_cmd_reg, &val);
	} while (!(val & CFG_IND_CMD_DONE_MASK));
}

static void serdes_rd(void *csr_base, u32 indirect_cmd_reg,
		      u32 indirect_data_reg, u32 addr, u32 *data)
{
	u32 val;
	u32 cmd;

	cmd = CFG_IND_RD_CMD_MASK | CFG_IND_CMD_DONE_MASK;
	cmd = (addr << 4) | cmd;
	phy_wr(csr_base + indirect_cmd_reg, cmd);
	phy_rd(csr_base + indirect_cmd_reg, &val); /* Force a barrier */
	/* Add an barrier - very important please don't remove */
	mb();
	/* Ignore one read */
	phy_rd(csr_base + indirect_cmd_reg, &val);
	do {
		phy_rd(csr_base + indirect_cmd_reg, &val);
	} while (!(val & CFG_IND_CMD_DONE_MASK));
	phy_rd(csr_base + indirect_data_reg, data);
}

/* X-Gene Serdes write helper */
static void sds_wr(void *csr_base, u32 addr, u32 data)
{
	u32 val;
	serdes_wr(csr_base, SATA_ENET_SDS_IND_CMD_REG_ADDR,
		  SATA_ENET_SDS_IND_WDATA_REG_ADDR, addr, data);
	serdes_rd(csr_base, SATA_ENET_SDS_IND_CMD_REG_ADDR,
		  SATA_ENET_SDS_IND_RDATA_REG_ADDR, addr, &val);
#if defined(XGENE_DBG2_CSR)
	pr_debug("SDS WR addr 0x%X value 0x%08X <-> 0x%08X\n", addr, data,
		 val);
#endif
}

/* X-Gene Serdes read helper */
static void sds_rd(void *csr_base, u32 addr, u32 *data)
{
	serdes_rd(csr_base, SATA_ENET_SDS_IND_CMD_REG_ADDR,
		  SATA_ENET_SDS_IND_RDATA_REG_ADDR, addr, data);
#if defined(XGENE_DBG2_CSR)
	pr_debug("SDS RD addr 0x%X value 0x%08X\n", addr, *data);
#endif
}

/* X-Gene Serdes toggle helper */
static void sds_toggle1to0(void *csr_base, u32 addr, u32 bits)
{
	u32 val;
	sds_rd(csr_base, addr, &val);
	val |= bits;
	sds_wr(csr_base, addr, val);
	sds_rd(csr_base, addr, &val);
	val &= ~bits;
	sds_wr(csr_base, addr, val);
}

/* X-Gene Serdes clear bits helper */
static void sds_clrbits(void *csr_base, u32 addr, u32 bits)
{
	u32 val;
	sds_rd(csr_base, addr, &val);
	val &= ~bits;
	sds_wr(csr_base, addr, val);
}

/* X-Gene Serdes set bits helper */
static void sds_setbits(void *csr_base, u32 addr, u32 bits)
{
	u32 val;
	sds_rd(csr_base, addr, &val);
	val |= bits;
	sds_wr(csr_base, addr, val);
}

/* X-Gene Serdes write helper using PCIe clock macro */
static void sds_pcie_wr(void *csr_base, u32 addr, u32 data)
{
	u32 val;
	serdes_wr(csr_base, SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_IND_CMD_REG_ADDR,
		  SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_IND_WDATA_REG_ADDR, addr,
		  data);
	serdes_rd(csr_base, SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_IND_CMD_REG_ADDR,
		  SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_IND_RDATA_REG_ADDR,
		  addr, &val);
#if defined(XGENE_DBG2_CSR)
	pr_debug("PCIE SDS WR addr 0x%X value 0x%08X <-> 0x%08X\n", addr,
		 data, val);
#endif
}

/* X-Gene Serdes read helper using PCIe clock macro */
static void sds_pcie_rd(void *csr_base, u32 addr, u32 *data)
{
	serdes_rd(csr_base, SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_IND_CMD_REG_ADDR,
		  SM_PCIE_X8_SDS_CSR_REGS_PCIE_SDS_IND_RDATA_REG_ADDR,
		  addr, data);
#if defined(XGENE_DBG2_CSR)
	pr_debug("PCIE SDS RD addr 0x%X value 0x%08X\n", addr, *data);
#endif
}

static void sds_pcie_toggle1to0(void *csr_base, u32 addr, u32 bits)
{
	u32 val;
	sds_pcie_rd(csr_base, addr, &val);
	val |= bits;
	sds_pcie_wr(csr_base, addr, val);
	sds_pcie_rd(csr_base, addr, &val);
	val &= ~bits;
	sds_pcie_wr(csr_base, addr, val);
}

static int xgene_phy_cal_rdy_check(void *csr_serdes,
				   void (*serdes_rd) (void *, u32, u32 *),
				   void (*serdes_wr) (void *, u32, u32),
				   void (*serdes_toggle1to0) (void *, u32, u32))
{
	int loopcount;
	u32 val;

	if (!enable_manual_cal)
		goto skip_manual_cal;

	/* TERM CALIBRATION CH0 */
	serdes_rd(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG17_ADDR, &val);
	val = CMU_REG17_PVT_CODE_R2A_SET(val, 0x12);
	val = CMU_REG17_RESERVED_7_SET(val, 0x0);
	serdes_wr(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG17_ADDR, val);
	serdes_toggle1to0(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG17_ADDR,
			CMU_REG17_PVT_TERM_MAN_ENA_MASK);
	/* DOWN CALIBRATION CH0 */
	serdes_rd(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG17_ADDR, &val);
	val = CMU_REG17_PVT_CODE_R2A_SET(val, 0x26);
	val = CMU_REG17_RESERVED_7_SET(val, 0x0);
	serdes_wr(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG17_ADDR, val);
	serdes_toggle1to0(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG16_ADDR,
			CMU_REG16_PVT_DN_MAN_ENA_MASK);
	/* UP CALIBRATION CH0 */
	serdes_rd(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG17_ADDR, &val);
	val = CMU_REG17_PVT_CODE_R2A_SET(val, 0x28);
	val = CMU_REG17_RESERVED_7_SET(val, 0x0);
	serdes_wr(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG17_ADDR, val);
	serdes_toggle1to0(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG16_ADDR,
			CMU_REG16_PVT_UP_MAN_ENA_MASK);

skip_manual_cal:
	/* Check PLL calibration for 10ms */
	loopcount = 50;
	do {
		serdes_rd(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG7_ADDR, &val);
		if (CMU_REG7_PLL_CALIB_DONE_RD(val))
			return 0;
		udelay(200); /* No need to poll faster than 200 us */
	} while (!CMU_REG7_PLL_CALIB_DONE_RD(val) && --loopcount > 0);

	return -1;
}

/* SATA port PLL initialization */
static int xgene_phy_macro_cal_rdy_chk(struct xgene_ahci_phy_ctx *ctx)
{
	void *csr_serdes = ctx->csr_base + SATA_SERDES_OFFSET;
	u32 val;

	xgene_phy_cal_rdy_check(csr_serdes, sds_rd, sds_wr, sds_toggle1to0);

	/* Check if PLL calibration complete sucessfully */
	sds_rd(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG7_ADDR, &val);
	if (CMU_REG7_PLL_CALIB_DONE_RD(val) == 0x1)
		dev_dbg(ctx->dev, "Clock macro PLL calibration done\n");
	/* Check for VCO FAIL */
	if (CMU_REG7_VCO_CAL_FAIL_RD(val) == 0x0) {
		dev_dbg(ctx->dev, "Clock macro VCO calibration successful\n");
		return 0;
	}
	dev_err(ctx->dev,
		"Clock macro calibration failed due to VCO failure\n");
	return -1;
}

/* SATA port PLL initialization (mux'ed with PCIe) */
static int xgene_phy_pcie_macro_cal_rdy_chk(struct xgene_ahci_phy_ctx *ctx)
{
	void *pcie_base = ctx->pcie_base;
	u32 val;

	xgene_phy_cal_rdy_check(pcie_base, sds_pcie_rd, sds_pcie_wr,
				sds_pcie_toggle1to0);

	/* PLL Calibration DONE */
	sds_pcie_rd(pcie_base, KC_CLKMACRO_CMU_REGS_CMU_REG7_ADDR, &val);
	if (CMU_REG7_PLL_CALIB_DONE_RD(val) == 0x1)
		dev_dbg(ctx->dev, "Clock macro PLL calibration done\n");
	/* Check for VCO FAIL */
	if (CMU_REG7_VCO_CAL_FAIL_RD(val) == 0x0) {
		dev_dbg(ctx->dev, "Clock macro VCO calibration successful\n");
		return 0;
	}
	dev_err(ctx->dev,
		"Clock macro calibration failed due to VCO failure\n");
	return -1;
}

/* SATA port force power down VCO */
static void xgene_phy_macro_pdwn_force_vco(struct xgene_ahci_phy_ctx *ctx)
{
	sds_toggle1to0(ctx->csr_base + SATA_SERDES_OFFSET,
		       KC_CLKMACRO_CMU_REGS_CMU_REG0_ADDR,
		       CMU_REG0_PDOWN_MASK);
	sds_toggle1to0(ctx->csr_base + SATA_SERDES_OFFSET,
		       KC_CLKMACRO_CMU_REGS_CMU_REG32_ADDR,
		       CMU_REG32_FORCE_VCOCAL_START_MASK);
}

/* SATA port force power down VCO (mux'ed with PCIe) */
static void xgene_phy_pcie_macro_pdwn_force_vco(struct xgene_ahci_phy_ctx *ctx)
{
	sds_pcie_toggle1to0(ctx->pcie_base, KC_CLKMACRO_CMU_REGS_CMU_REG0_ADDR,
			    CMU_REG0_PDOWN_MASK);
	sds_pcie_toggle1to0(ctx->pcie_base,
			    KC_CLKMACRO_CMU_REGS_CMU_REG32_ADDR,
			    CMU_REG32_FORCE_VCOCAL_START_MASK);
}

static int xgene_phy_macro_cfg(void *csr_serdes,
			       void (*serdes_rd) (void *, u32, u32 *),
			       void (*serdes_wr) (void *, u32, u32),
			       int ref_100MHz)
{
	u32 val;

	serdes_rd(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG34_ADDR, &val);
	val = CMU_REG34_VCO_CAL_VTH_LO_MAX_SET(val, 0x7);
	val = CMU_REG34_VCO_CAL_VTH_HI_MAX_SET(val, 0xd);
	val = CMU_REG34_VCO_CAL_VTH_LO_MIN_SET(val, 0x2);
	val = CMU_REG34_VCO_CAL_VTH_HI_MIN_SET(val, 0x8);
	serdes_wr(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG34_ADDR, val);
	serdes_rd(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG0_ADDR, &val);
	val = CMU_REG0_CAL_COUNT_RESOL_SET(val, 0x4);
	serdes_wr(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG0_ADDR, val);
	/* CMU_REG1 */
	serdes_rd(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG1_ADDR, &val);
	val = CMU_REG1_PLL_CP_SET(val, 0x1);
	val = CMU_REG1_PLL_CP_SEL_SET(val, 0x5);
	val = CMU_REG1_PLL_MANUALCAL_SET(val, 0x0);
	serdes_wr(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG1_ADDR, val);
	/* CMU_REG2 */
	serdes_rd(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG2_ADDR, &val);
	val = CMU_REG2_PLL_LFRES_SET(val, 0xa);
	if (ref_100MHz) {
		val = CMU_REG2_PLL_FBDIV_SET(val, 0x27); /* 100Mhz refclk */
		val = CMU_REG2_PLL_REFDIV_SET(val, 0x0);
	} else {
		val = CMU_REG2_PLL_FBDIV_SET(val, 0x4f); /* 50Mhz refclk */
		val = CMU_REG2_PLL_REFDIV_SET(val, 0x1);
	}
	serdes_wr(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG2_ADDR, val);
	/* CMU_REG3 */
	serdes_rd(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG3_ADDR, &val);
	val = CMU_REG3_VCOVARSEL_SET(val, 0x3);
	val = CMU_REG3_VCO_MOMSEL_INIT_SET(val, 0x10);
	serdes_wr(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG3_ADDR, val);
	/* CMU_REG26  */
	serdes_rd(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG26_ADDR, &val);
	val = CMU_REG26_FORCE_PLL_LOCK_SET(val, 0x0);
	serdes_wr(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG26_ADDR, val);
	/* CMU_REG5 */
	serdes_rd(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG5_ADDR, &val);
	val = CMU_REG5_PLL_LFSMCAP_SET(val, 0x3);
	val = CMU_REG5_PLL_LFCAP_SET(val, 0x3);
	val = CMU_REG5_PLL_LOCK_RESOLUTION_SET(val, 0x7);
	serdes_wr(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG5_ADDR, val);
	/* CMU_reg6 */
	serdes_rd(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG6_ADDR, &val);
	val = CMU_REG6_PLL_VREGTRIM_SET(val, 0x0);
	val = CMU_REG6_MAN_PVT_CAL_SET(val, enable_manual_cal ? 0x1 : 0x0);
	serdes_wr(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG6_ADDR, val);
	/* CMU_reg16 */
	serdes_rd(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG16_ADDR, &val);
	val = CMU_REG16_CALIBRATION_DONE_OVERRIDE_SET(val, 0x1);
	val = CMU_REG16_BYPASS_PLL_LOCK_SET(val, 0x1);
	val = CMU_REG16_VCOCAL_WAIT_BTW_CODE_SET(val, 0x4);
	serdes_wr(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG16_ADDR, val);
	/* CMU_reg30 */
	serdes_rd(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG30_ADDR, &val);
	val = CMU_REG30_PCIE_MODE_SET(val, 0x0);
	val = CMU_REG30_LOCK_COUNT_SET(val, 0x3);
	serdes_wr(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG30_ADDR, val);
	/* CMU reg31 */
	serdes_wr(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG31_ADDR, 0xF);
	/* CMU_reg32 */
	serdes_rd(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG32_ADDR, &val);
	val = CMU_REG32_PVT_CAL_WAIT_SEL_SET(val, 0x3);
	val = CMU_REG32_IREF_ADJ_SET(val, 0x3);
	serdes_wr(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG32_ADDR, val);
	/* CMU_reg34 */
	serdes_wr(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG34_ADDR, 0x8d27);
	/* CMU_reg37 */
	serdes_rd(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG37_ADDR, &val);
	serdes_wr(csr_serdes, KC_CLKMACRO_CMU_REGS_CMU_REG37_ADDR, 0xF00F);

	return 0;
}

/* SATA port macro configuration */
static int xgene_phy_sata_macro_cfg(struct xgene_ahci_phy_ctx *ctx)
{
	void *csr_serdes = ctx->csr_base + SATA_SERDES_OFFSET;
	int calib_loop_count = 0;
	u32 val;

	phy_rd(csr_serdes + SATA_ENET_CLK_MACRO_REG_ADDR, &val);
	val = I_RESET_B_SET(val, 0x0);
	val = I_PLL_FBDIV_SET(val, 0x27);
	val = I_CUSTOMEROV_SET(val, 0x0);
	phy_wr(csr_serdes + SATA_ENET_CLK_MACRO_REG_ADDR, val);

	xgene_phy_macro_cfg(csr_serdes, sds_rd, sds_wr, 1 /* 100MHz ref */);

	phy_rd(csr_serdes + SATA_ENET_CLK_MACRO_REG_ADDR, &val);
	val = I_RESET_B_SET(val, 0x1);
	val = I_CUSTOMEROV_SET(val, 0x0);
	phy_wr(csr_serdes + SATA_ENET_CLK_MACRO_REG_ADDR, val);

	mb();

	while (++calib_loop_count <= 5) {
		if (xgene_phy_macro_cal_rdy_chk(ctx) == 0)
			break;
		xgene_phy_macro_pdwn_force_vco(ctx);
	}
	if (calib_loop_count > 5) {
		dev_err(ctx->dev, "Clock macro PLL not ready...\n");
		return -1;
	}
	phy_rd(csr_serdes + SATA_ENET_CLK_MACRO_REG_ADDR, &val);
	dev_dbg(ctx->dev, "Clock macro PLL %sLOOKED...\n",
		O_PLL_LOCK_RD(val) ? "" : "UN");
	dev_dbg(ctx->dev, "Clock macro PLL %sREADY...\n",
		O_PLL_READY_RD(val) ? "" : "NOT");

	return 0;
}

static void xgene_phy_pcie_enable_clk(struct xgene_ahci_phy_ctx *ctx)
{
	void *pcie_base = ctx->pcie_base;

	phy_wr_flush(pcie_base + SM_PCIE_CLKRST_CSR_PCIE_CLKEN_ADDR, 0xff);
	phy_wr_flush(pcie_base + SM_PCIE_CLKRST_CSR_PCIE_SRST_ADDR, 0x00);
}

/* SATA port macro configuration using the PCIe clock macro */
static int xgene_phy_pcie_macro_cfg(struct xgene_ahci_phy_ctx *ctx)
{
	void *pcie_base = ctx->pcie_base;
	int calib_loop_count;
	u32 val;

	xgene_phy_pcie_enable_clk(ctx);

	phy_rd(pcie_base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_CLK_MACRO_REG_ADDR,
	       &val);
	val = PCIE_CLK_MACRO_REG_I_RESET_B_SET(val, 0x0);
	val = PCIE_CLK_MACRO_REG_I_CUSTOMEROV_SET(val, 0x0);
	phy_wr(pcie_base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_CLK_MACRO_REG_ADDR,
	       val);
	phy_rd(pcie_base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_CLK_MACRO_REG_ADDR,
	       &val);

	xgene_phy_macro_cfg(pcie_base, sds_pcie_rd, sds_pcie_wr,
			    0 /* 50Mhz clock */);

	phy_rd(pcie_base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_CLK_MACRO_REG_ADDR,
	       &val);
	val = PCIE_CLK_MACRO_REG_I_RESET_B_SET(val, 0x1);
	val = PCIE_CLK_MACRO_REG_I_CUSTOMEROV_SET(val, 0x0);
	phy_wr(pcie_base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_CLK_MACRO_REG_ADDR,
	       val);

	mb();

	/* Check for 5 times */
	calib_loop_count = 5;
	do {
		if (xgene_phy_pcie_macro_cal_rdy_chk(ctx) == 0)
			break;
		xgene_phy_pcie_macro_pdwn_force_vco(ctx);
	} while (--calib_loop_count > 0);
	if (calib_loop_count <= 0) {
		dev_err(ctx->dev, "Clock macro PLL failed\n");
		return -1;
	}
	phy_rd(pcie_base + SM_PCIE_X8_SDS_CSR_REGS_PCIE_CLK_MACRO_REG_ADDR,
	       &val);
	dev_dbg(ctx->dev, "Clock macro PLL %sLOOKED...\n",
		 PCIE_CLK_MACRO_REG_O_PLL_LOCK_RD(val) ? "" : "UN");
	dev_dbg(ctx->dev, "Clock macro PLL %sREADY...\n",
		 PCIE_CLK_MACRO_REG_O_PLL_READY_RD(val) ? "" : "NOT ");

	return 0;
}

static void xgene_phy_clk_rst_pre(struct xgene_ahci_phy_ctx *ctx)
{
	void *clkcsr_base = ctx->csr_base + SATA_CLK_OFFSET;
	u32 val;

	dev_dbg(ctx->dev, "enable controller clock\n");
	/* disable all reset */
	phy_wr_flush(clkcsr_base + SATASRESETREG_ADDR, 0x00);

	/* Enable all resets */
	phy_wr_flush(clkcsr_base + SATASRESETREG_ADDR, 0xff);

	/* Disable all clks */
	phy_wr_flush(clkcsr_base + SATACLKENREG_ADDR, 0x00);

	/* Enable all clks */
	phy_wr_flush(clkcsr_base + SATACLKENREG_ADDR, 0xf9);

	/* Get out of reset for:
	 *  SDS, CSR
	 *  CORE & MEM are still reset
	 */
	phy_rd(clkcsr_base + SATASRESETREG_ADDR, &val);
	if (SATA_MEM_RESET_RD(val) == 1) {
		val &= ~(SATA_CSR_RESET_MASK | SATA_SDS_RESET_MASK);
		val |= SATA_CORE_RESET_MASK | SATA_PCLK_RESET_MASK |
			SATA_PMCLK_RESET_MASK | SATA_MEM_RESET_MASK;
	}
	phy_wr_flush(clkcsr_base + SATASRESETREG_ADDR, val);
}

static void xgene_phy_reset_pclk(struct xgene_ahci_phy_ctx *ctx)
{
	void *clkcsr_base = ctx->csr_base + SATA_CLK_OFFSET;
	u32 val;

	phy_rd(clkcsr_base + SATASRESETREG_ADDR, &val);
	val &= ~SATA_PCLK_RESET_MASK;
	phy_wr_flush(clkcsr_base + SATASRESETREG_ADDR, val);
}

static void xgene_phy_reset_sds_pmclk_core(struct xgene_ahci_phy_ctx *ctx)
{
	void *clkcsr_base = ctx->csr_base + SATA_CLK_OFFSET;
	u32 val;

	phy_rd(clkcsr_base + SATASRESETREG_ADDR, &val);
	val &= ~(SATA_CORE_RESET_MASK |
		 SATA_PMCLK_RESET_MASK | SATA_SDS_RESET_MASK);
	phy_wr_flush(clkcsr_base + SATASRESETREG_ADDR, val);
}

static void xgene_phy_force_lat_summer_cal(struct xgene_ahci_phy_ctx *ctx,
					   int channel)
{
	void *csr_base = ctx->csr_base + SATA_SERDES_OFFSET;
	u32 os = channel * 0x200;
	int i;
	struct {
		u32 reg;
		u32 val;
	} serdes_reg[] = {
		{KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG38_ADDR, 0x0},
		{KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG39_ADDR, 0xff00},
		{KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG40_ADDR, 0xffff},
		{KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG41_ADDR, 0xffff},
		{KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG42_ADDR, 0xffff},
		{KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG43_ADDR, 0xffff},
		{KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG44_ADDR, 0xffff},
		{KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG45_ADDR, 0xffff},
		{KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG46_ADDR, 0xffff},
		{KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG47_ADDR, 0xfffc},
		{KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG48_ADDR, 0x0},
		{KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG49_ADDR, 0x0},
		{KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG50_ADDR, 0x0},
		{KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG51_ADDR, 0x0},
		{KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG52_ADDR, 0x0},
		{KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG53_ADDR, 0x0},
		{KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG54_ADDR, 0x0},
		{KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG55_ADDR, 0x0},
		{0, 0x0},
	};

	/* SUMMER CALIBRATION CH0/CH1 */
	/* SUMMER calib toggle CHX */
	sds_toggle1to0(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG127_ADDR + os,
		       CH0_RXTX_REG127_FORCE_SUM_CAL_START_MASK);
	/* latch calib toggle CHX */
	sds_toggle1to0(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG127_ADDR + os,
		       CH0_RXTX_REG127_FORCE_LAT_CAL_START_MASK);
	/* CHX */
	sds_wr(csr_base, KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG28_ADDR + os, 0x7);
	sds_wr(csr_base, KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG31_ADDR + os,
	       0x7e00);

	sds_clrbits(csr_base, KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG4_ADDR + os,
		    CH0_RXTX_REG4_TX_LOOPBACK_BUF_EN_MASK);
	sds_clrbits(csr_base, KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG7_ADDR + os,
		    CH0_RXTX_REG7_LOOP_BACK_ENA_CTLE_MASK);

	/* RXTX_REG38-55 */
	for (i = 0; serdes_reg[i].reg != 0; i++)
		sds_wr(csr_base, serdes_reg[i].reg + os, serdes_reg[i].val);
}

static int xgene_phy_get_avg(int accum, int samples)
{
	return (accum + (samples / 2)) / samples;
}

static void xgene_phy_reset_rxd(struct xgene_ahci_phy_ctx *ctx, int channel)
{
	void *csr_base = ctx->csr_base + SATA_SERDES_OFFSET;

	sds_clrbits(csr_base, KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG7_ADDR +
		    channel * 0x200, CH0_RXTX_REG7_RESETB_RXD_MASK);
	sds_setbits(csr_base, KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG7_ADDR +
		    channel * 0x200, CH0_RXTX_REG7_RESETB_RXD_MASK);
}

static void xgene_phy_gen_avg_val(struct xgene_ahci_phy_ctx *ctx, int channel)
{
	void *csr_serdes_base = ctx->csr_base + SATA_SERDES_OFFSET;
	int avg_loop = 10;
	int MAX_LOOP = 10;
	int lat_do = 0, lat_xo = 0, lat_eo = 0, lat_so = 0;
	int lat_de = 0, lat_xe = 0, lat_ee = 0, lat_se = 0;
	int sum_cal = 0;
	int lat_do_itr = 0, lat_xo_itr = 0, lat_eo_itr = 0, lat_so_itr = 0;
	int lat_de_itr = 0, lat_xe_itr = 0, lat_ee_itr = 0, lat_se_itr = 0;
	int sum_cal_itr = 0;
	int fail_even = 0;
	int fail_odd = 0;
	u32 val;
	u32 os;

	dev_dbg(ctx->dev, "Generating average calibration value for port %d\n",
		channel);

	os = channel * 0x200;

	/* Enable RX Hi-Z termination enable */
	sds_setbits(csr_serdes_base,
		    KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG12_ADDR + os,
		    CH0_RXTX_REG12_RX_DET_TERM_ENABLE_MASK);
	/* Turn off DFE */
	sds_wr(csr_serdes_base,
	       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG28_ADDR + os, 0x0000);
	/* DFE Presets to zero */
	sds_wr(csr_serdes_base,
	       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG31_ADDR + os, 0x0000);

	while (avg_loop > 0) {
		xgene_phy_force_lat_summer_cal(ctx, channel);

		sds_rd(csr_serdes_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG21_ADDR + os, &val);
		lat_do_itr = CH0_RXTX_REG21_DO_LATCH_CALOUT_RD(val);
		lat_xo_itr = CH0_RXTX_REG21_XO_LATCH_CALOUT_RD(val);
		fail_odd = CH0_RXTX_REG21_LATCH_CAL_FAIL_ODD_RD(val);

		sds_rd(csr_serdes_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG22_ADDR + os, &val);
		lat_eo_itr = CH0_RXTX_REG22_EO_LATCH_CALOUT_RD(val);
		lat_so_itr = CH0_RXTX_REG22_SO_LATCH_CALOUT_RD(val);
		fail_even = CH0_RXTX_REG22_LATCH_CAL_FAIL_EVEN_RD(val);

		sds_rd(csr_serdes_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG23_ADDR + os, &val);
		lat_de_itr = CH0_RXTX_REG23_DE_LATCH_CALOUT_RD(val);
		lat_xe_itr = CH0_RXTX_REG23_XE_LATCH_CALOUT_RD(val);
		sds_rd(csr_serdes_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG24_ADDR + os, &val);
		lat_ee_itr = CH0_RXTX_REG24_EE_LATCH_CALOUT_RD(val);
		lat_se_itr = CH0_RXTX_REG24_SE_LATCH_CALOUT_RD(val);

		sds_rd(csr_serdes_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG121_ADDR + os, &val);
		sum_cal_itr = CH0_RXTX_REG121_SUMOS_CAL_CODE_RD(val);

		if ((fail_even == 0 || fail_even == 1) &&
		    (fail_odd == 0 || fail_odd == 1)) {
			lat_do += lat_do_itr;
			lat_xo += lat_xo_itr;
			lat_eo += lat_eo_itr;
			lat_so += lat_so_itr;
			lat_de += lat_de_itr;
			lat_xe += lat_xe_itr;
			lat_ee += lat_ee_itr;
			lat_se += lat_se_itr;
			sum_cal += sum_cal_itr;

			dev_dbg(ctx->dev, "Interation Value: %d\n", avg_loop);
			dev_dbg(ctx->dev, "DO 0x%x XO 0x%x EO 0x%x SO 0x%x\n",
				lat_do_itr, lat_xo_itr, lat_eo_itr,
				lat_so_itr);
			dev_dbg(ctx->dev, "DE 0x%x XE 0x%x EE 0x%x SE 0x%x\n",
				lat_de_itr, lat_xe_itr, lat_ee_itr,
				lat_se_itr);
			dev_dbg(ctx->dev, "sum_cal 0x%x", sum_cal_itr);
			avg_loop--;
		} else {
			dev_err(ctx->dev, "Interation failed %d\n", avg_loop);
		}
		xgene_phy_reset_rxd(ctx, channel);
	}

	/* Update with Average Value */
	sds_rd(csr_serdes_base,
	       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG127_ADDR + os, &val);
	val = CH0_RXTX_REG127_DO_LATCH_MANCAL_SET(val,
		xgene_phy_get_avg(lat_do, MAX_LOOP));
	val = CH0_RXTX_REG127_XO_LATCH_MANCAL_SET(val,
		xgene_phy_get_avg(lat_xo, MAX_LOOP));
	sds_wr(csr_serdes_base,
	       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG127_ADDR + os, val);

	sds_rd(csr_serdes_base,
	       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG128_ADDR + os, &val);
	val = CH0_RXTX_REG128_EO_LATCH_MANCAL_SET(val,
		xgene_phy_get_avg(lat_eo, MAX_LOOP));
	val = CH0_RXTX_REG128_SO_LATCH_MANCAL_SET(val,
		xgene_phy_get_avg(lat_so, MAX_LOOP));
	sds_wr(csr_serdes_base,
	       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG128_ADDR + os, val);
	sds_rd(csr_serdes_base,
	       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG129_ADDR + os, &val);
	val = CH0_RXTX_REG129_DE_LATCH_MANCAL_SET(val,
		xgene_phy_get_avg(lat_de, MAX_LOOP));
	val = CH0_RXTX_REG129_XE_LATCH_MANCAL_SET(val,
		xgene_phy_get_avg(lat_xe, MAX_LOOP));
	sds_wr(csr_serdes_base,
	       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG129_ADDR + os, val);

	sds_rd(csr_serdes_base,
	       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG130_ADDR + os, &val);
	val = CH0_RXTX_REG130_EE_LATCH_MANCAL_SET(val,
		xgene_phy_get_avg(lat_ee, MAX_LOOP));
	val = CH0_RXTX_REG130_SE_LATCH_MANCAL_SET(val,
		xgene_phy_get_avg(lat_se, MAX_LOOP));
	sds_wr(csr_serdes_base,
	       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG130_ADDR + os, val);
	/* Summer Calibration Value */
	sds_rd(csr_serdes_base,
	       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG14_ADDR + os, &val);
	val = CH0_RXTX_REG14_CLTE_LATCAL_MAN_PROG_SET(val,
		xgene_phy_get_avg(sum_cal, MAX_LOOP));
	sds_wr(csr_serdes_base,
	       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG14_ADDR + os, val);

	dev_dbg(ctx->dev, "Average Value:\n");
	dev_dbg(ctx->dev, "DO 0x%x XO 0x%x EO 0x%x SO 0x%x\n",
		 xgene_phy_get_avg(lat_do, MAX_LOOP),
		 xgene_phy_get_avg(lat_xo, MAX_LOOP),
		 xgene_phy_get_avg(lat_eo, MAX_LOOP),
		 xgene_phy_get_avg(lat_so, MAX_LOOP));
	dev_dbg(ctx->dev, "DE 0x%x XE 0x%x EE 0x%x SE 0x%x\n",
		 xgene_phy_get_avg(lat_de, MAX_LOOP),
		 xgene_phy_get_avg(lat_xe, MAX_LOOP),
		 xgene_phy_get_avg(lat_ee, MAX_LOOP),
		 xgene_phy_get_avg(lat_se, MAX_LOOP));
	dev_dbg(ctx->dev, "sum_cal 0x%x\n",
		xgene_phy_get_avg(sum_cal, MAX_LOOP));

	/* Manual Summer Calibration */
	sds_rd(csr_serdes_base,
	       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG14_ADDR + os, &val);
	val = CH0_RXTX_REG14_CTLE_LATCAL_MAN_ENA_SET(val, 0x1);
	sds_wr(csr_serdes_base,
	       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG14_ADDR + os, val);
	dev_dbg(ctx->dev, "Manual Summer calibration enabled\n");

	/* Manual Latch Calibration */
	sds_rd(csr_serdes_base,
	       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG127_ADDR + os, &val);
	val = CH0_RXTX_REG127_LATCH_MAN_CAL_ENA_SET(val, 0x1);
	dev_dbg(ctx->dev, "Manual Latch Calibration enabled\n");
	sds_wr(csr_serdes_base,
	       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG127_ADDR + os, val);

	/* Disable RX Hi-Z termination enable */
	sds_rd(csr_serdes_base,
	       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG12_ADDR + os, &val);
	val = CH0_RXTX_REG12_RX_DET_TERM_ENABLE_SET(val, 0);
	sds_wr(csr_serdes_base,
	       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG12_ADDR + os, val);

	/* Turn on DFE */
	sds_wr(csr_serdes_base,
	       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG28_ADDR + os, 0x0007);

	/* DFE Presets to 0 */
	sds_wr(csr_serdes_base,
	       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG31_ADDR + os, 0x7e00);
}

static int xgene_phy_host_sata_select(struct xgene_ahci_phy_ctx *ctx)
{
	void *muxcsr_base = ctx->csr_base + SATA_ETH_MUX_OFFSET;
	u32 val;

	dev_dbg(ctx->dev, "select SATA mux\n");
	phy_rd(muxcsr_base + SATA_ENET_CONFIG_REG_ADDR, &val);
	val &= ~CFG_SATA_ENET_SELECT_MASK;
	phy_wr(muxcsr_base + SATA_ENET_CONFIG_REG_ADDR, val);
	phy_rd(muxcsr_base + SATA_ENET_CONFIG_REG_ADDR, &val);
	return val & CFG_SATA_ENET_SELECT_MASK ? -1 : 0;
}

static void xgene_phy_validation_CMU_cfg(struct xgene_ahci_phy_ctx *ctx,
					 int clk_type)
{
	void *csr_base = ctx->csr_base + SATA_SERDES_OFFSET;
	u32 val;

	sds_rd(csr_base, KC_SERDES_CMU_REGS_CMU_REG0_ADDR, &val);
	val = CMU_REG0_CAL_COUNT_RESOL_SET(val, 0x4);
	sds_wr(csr_base, KC_SERDES_CMU_REGS_CMU_REG0_ADDR, val);

	sds_rd(csr_base, KC_SERDES_CMU_REGS_CMU_REG1_ADDR, &val);
	val = CMU_REG1_PLL_CP_SET(val, 0x1);
	val = CMU_REG1_PLL_CP_SEL_SET(val, 0x5);
	val = CMU_REG1_PLL_MANUALCAL_SET(val, 0x0);
	sds_wr(csr_base, KC_SERDES_CMU_REGS_CMU_REG1_ADDR, val);

	sds_rd(csr_base, KC_SERDES_CMU_REGS_CMU_REG2_ADDR, &val);
	val = CMU_REG2_PLL_LFRES_SET(val, 0xa);
	if (clk_type == SATA_CLK_INT_SING) {
		val = CMU_REG2_PLL_FBDIV_SET(val, FBDIV_VAL_100M);
		val = CMU_REG2_PLL_REFDIV_SET(val, REFDIV_VAL_100M);
	} else {
		val = CMU_REG2_PLL_FBDIV_SET(val, FBDIV_VAL_50M);
		val = CMU_REG2_PLL_REFDIV_SET(val, REFDIV_VAL_50M);
	}
	sds_wr(csr_base, KC_SERDES_CMU_REGS_CMU_REG2_ADDR, val);

	sds_rd(csr_base, KC_SERDES_CMU_REGS_CMU_REG3_ADDR, &val);
	val = CMU_REG3_VCOVARSEL_SET(val, 0xF);
	val = CMU_REG3_VCO_MOMSEL_INIT_SET(val, 0x15);
	val = CMU_REG3_VCO_MANMOMSEL_SET(val, 0x15);
	sds_wr(csr_base, KC_SERDES_CMU_REGS_CMU_REG3_ADDR, val);

	sds_rd(csr_base, KC_SERDES_CMU_REGS_CMU_REG26_ADDR, &val);
	val = CMU_REG26_FORCE_PLL_LOCK_SET(val, 0x0);
	sds_wr(csr_base, KC_SERDES_CMU_REGS_CMU_REG26_ADDR, val);

	sds_rd(csr_base, KC_SERDES_CMU_REGS_CMU_REG5_ADDR, &val);
	val = CMU_REG5_PLL_LFSMCAP_SET(val, 0x3);
	val = CMU_REG5_PLL_LFCAP_SET(val, 0x3);
	val = CMU_REG5_PLL_LOCK_RESOLUTION_SET(val, 0x4);
	sds_wr(csr_base, KC_SERDES_CMU_REGS_CMU_REG5_ADDR, val);

	sds_rd(csr_base, KC_SERDES_CMU_REGS_CMU_REG6_ADDR, &val);
	val = CMU_REG6_PLL_VREGTRIM_SET(val, 0x0);
	val = CMU_REG6_MAN_PVT_CAL_SET(val, enable_manual_cal ? 0x1 : 0x0);
	sds_wr(csr_base, KC_SERDES_CMU_REGS_CMU_REG6_ADDR, val);

	sds_rd(csr_base, KC_SERDES_CMU_REGS_CMU_REG9_ADDR, &val);
	val = CMU_REG9_TX_WORD_MODE_CH1_SET(val, 0x3);
	val = CMU_REG9_TX_WORD_MODE_CH0_SET(val, 0x3);
	val = CMU_REG9_PLL_POST_DIVBY2_SET(val, 0x1);
	sds_wr(csr_base, KC_SERDES_CMU_REGS_CMU_REG9_ADDR, val);

	sds_rd(csr_base, KC_SERDES_CMU_REGS_CMU_REG16_ADDR, &val);
	val = CMU_REG16_CALIBRATION_DONE_OVERRIDE_SET(val, 0x1);
	val = CMU_REG16_BYPASS_PLL_LOCK_SET(val, 0x1);
	val = CMU_REG16_VCOCAL_WAIT_BTW_CODE_SET(val, 0x4);
	sds_wr(csr_base, KC_SERDES_CMU_REGS_CMU_REG16_ADDR, val);

	sds_rd(csr_base, KC_SERDES_CMU_REGS_CMU_REG30_ADDR, &val);
	val = CMU_REG30_PCIE_MODE_SET(val, 0x0);
	val = CMU_REG30_LOCK_COUNT_SET(val, 0x3);
	sds_wr(csr_base, KC_SERDES_CMU_REGS_CMU_REG30_ADDR, val);

	sds_wr(csr_base, KC_SERDES_CMU_REGS_CMU_REG31_ADDR, 0xF);

	sds_rd(csr_base, KC_SERDES_CMU_REGS_CMU_REG32_ADDR, &val);
	val = CMU_REG32_PVT_CAL_WAIT_SEL_SET(val, 0x3);
	val = CMU_REG32_IREF_ADJ_SET(val, 0x3);
	sds_wr(csr_base, KC_SERDES_CMU_REGS_CMU_REG32_ADDR, val);

	sds_wr(csr_base, KC_SERDES_CMU_REGS_CMU_REG34_ADDR, 0x8d27);

	sds_wr(csr_base, KC_SERDES_CMU_REGS_CMU_REG37_ADDR, 0xF00F);
}

static void xgene_phy_validation_rxtx_cfg(struct xgene_ahci_phy_ctx *ctx)
{
	void *csr_base = ctx->csr_base + SATA_SERDES_OFFSET;
	u32 val;
	u32 reg;
	int i;
	int chan;

	for (chan = 0; chan < MAX_CHANNEL; chan++) {
		u32 os = chan * 0x200;

		sds_wr(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG147_ADDR + os, 0x6);

		sds_rd(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG0_ADDR + os, &val);
		val = CH0_RXTX_REG0_CTLE_EQ_HR_SET(val, 0x10);
		val = CH0_RXTX_REG0_CTLE_EQ_QR_SET(val, 0x10);
		val = CH0_RXTX_REG0_CTLE_EQ_FR_SET(val, 0x10);
		sds_wr(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG0_ADDR + os, val);

		sds_rd(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG1_ADDR + os, &val);
		val = CH0_RXTX_REG1_RXACVCM_SET(val, 0x7);
		val = CH0_RXTX_REG1_CTLE_EQ_SET(val,
			ctx->txboostgain[chan * 3 + ctx->speed[chan]]);
		sds_wr(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG1_ADDR + os, val);

		sds_rd(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG2_ADDR + os, &val);
		val = CH0_RXTX_REG2_VTT_ENA_SET(val, 0x1);
		val = CH0_RXTX_REG2_VTT_SEL_SET(val, 0x1);
		val = CH0_RXTX_REG2_TX_FIFO_ENA_SET(val, 0x1);
		sds_wr(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG2_ADDR + os, val);

		sds_rd(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG4_ADDR + os, &val);
		val = CH0_RXTX_REG4_TX_WORD_MODE_SET(val, 0x3);
		sds_wr(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG4_ADDR + os, val);

		sds_rd(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG5_ADDR + os, &val);
		val = CH0_RXTX_REG5_TX_CN1_SET(val, 0x0);
		val = CH0_RXTX_REG5_TX_CP1_SET(val, 0xF);
		val = CH0_RXTX_REG5_TX_CN2_SET(val, 0x0);
		sds_wr(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG5_ADDR + os, val);

		sds_rd(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG6_ADDR + os, &val);
		val = CH0_RXTX_REG6_TXAMP_CNTL_SET(val, 0xf);
		val = CH0_RXTX_REG6_TXAMP_ENA_SET(val, 0x1);
		val = CH0_RXTX_REG6_TX_IDLE_SET(val, 0x0);
		val = CH0_RXTX_REG6_RX_BIST_RESYNC_SET(val, 0x0);
		val = CH0_RXTX_REG6_RX_BIST_ERRCNT_RD_SET(val, 0x0);
		sds_wr(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG6_ADDR + os, val);

		sds_rd(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG7_ADDR + os, &val);
		val = CH0_RXTX_REG7_BIST_ENA_RX_SET(val, 0x0);
		val = CH0_RXTX_REG7_RX_WORD_MODE_SET(val, 0x3);
		sds_wr(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG7_ADDR + os, val);

		sds_rd(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG8_ADDR + os, &val);
		val = CH0_RXTX_REG8_CDR_LOOP_ENA_SET(val, 0x1);
		val = CH0_RXTX_REG8_CDR_BYPASS_RXLOS_SET(val, 0x0);
		val = CH0_RXTX_REG8_SSC_ENABLE_SET(val, 0x1);
		val = CH0_RXTX_REG8_SD_DISABLE_SET(val, 0x0);
		val = CH0_RXTX_REG8_SD_VREF_SET(val, 0x4);
		sds_wr(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG8_ADDR + os, val);

		sds_rd(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG11_ADDR + os, &val);
		val = CH0_RXTX_REG11_PHASE_ADJUST_LIMIT_SET(val, 0x0);
		sds_wr(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG11_ADDR + os, val);

		sds_rd(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG12_ADDR + os, &val);
		val = CH0_RXTX_REG12_LATCH_OFF_ENA_SET(val, 0x1);
		val = CH0_RXTX_REG12_SUMOS_ENABLE_SET(val, 0x0);
		val = CH0_RXTX_REG12_RX_DET_TERM_ENABLE_SET(val, 0x0);
		sds_wr(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG12_ADDR + os, val);

		sds_rd(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG26_ADDR + os, &val);
		val = CH0_RXTX_REG26_PERIOD_ERROR_LATCH_SET(val, 0x0);
		val = CH0_RXTX_REG26_BLWC_ENA_SET(val, 0x1);
		sds_wr(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG26_ADDR + os, val);

		sds_wr(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG28_ADDR + os, 0x0);
		sds_wr(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG31_ADDR + os, 0x0);

		sds_rd(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG61_ADDR + os, &val);
		val = CH0_RXTX_REG61_ISCAN_INBERT_SET(val, 0x1);
		val = CH0_RXTX_REG61_LOADFREQ_SHIFT_SET(val, 0x0);
		val = CH0_RXTX_REG61_EYE_COUNT_WIDTH_SEL_SET(val, 0x0);
		sds_wr(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG61_ADDR + os, val);

		sds_rd(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG62_ADDR + os, &val);
		val = CH0_RXTX_REG62_PERIOD_H1_QLATCH_SET(val, 0x0);
		sds_wr(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG62_ADDR + os, val);

		for (i = 0; i < 9; i++) {
			reg = KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG81_ADDR +
				os + i * 2;
			sds_rd(csr_base, reg, &val);
			val = CH0_RXTX_REG89_MU_TH7_SET(val, 0xe);
			val = CH0_RXTX_REG89_MU_TH8_SET(val, 0xe);
			val = CH0_RXTX_REG89_MU_TH9_SET(val, 0xe);
			sds_wr(csr_base, reg, val);
		}

		for (i = 0; i < 3; i++) {
			reg = KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG96_ADDR +
				os + i * 2;
			sds_rd(csr_base, reg, &val);
			val = CH0_RXTX_REG96_MU_FREQ1_SET(val, 0x10);
			val = CH0_RXTX_REG96_MU_FREQ2_SET(val, 0x10);
			val = CH0_RXTX_REG96_MU_FREQ3_SET(val, 0x10);
			sds_wr(csr_base, reg, val);
		}

		for (i = 0; i < 3; i++) {
			reg = KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG99_ADDR +
				os + i * 2;
			sds_rd(csr_base, reg, &val);
			val = CH0_RXTX_REG99_MU_PHASE1_SET(val, 0x7);
			val = CH0_RXTX_REG99_MU_PHASE2_SET(val, 0x7);
			val = CH0_RXTX_REG99_MU_PHASE3_SET(val, 0x7);
			sds_wr(csr_base, reg, val);
		}

		sds_rd(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG102_ADDR + os, &val);
		val = CH0_RXTX_REG102_FREQLOOP_LIMIT_SET(val, 0x0);
		sds_wr(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG102_ADDR + os, val);

		sds_wr(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG114_ADDR + os,
		       0xffe0);

		sds_rd(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG125_ADDR + os, &val);
		val = CH0_RXTX_REG125_SIGN_PQ_SET(val,
			ctx->txeyedirection[chan * 3 + ctx->speed[chan]]);
		val = CH0_RXTX_REG125_PQ_REG_SET(val,
			ctx->txeyetuning[chan * 3 + ctx->speed[chan]]);
		val = CH0_RXTX_REG125_PHZ_MANUAL_SET(val, 0x1);
		sds_wr(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG125_ADDR + os, val);

		sds_rd(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG127_ADDR + os, &val);
		val = CH0_RXTX_REG127_LATCH_MAN_CAL_ENA_SET(val, 0x0);
		sds_wr(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG127_ADDR + os, val);

		sds_rd(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG128_ADDR + os, &val);
		val = CH0_RXTX_REG128_LATCH_CAL_WAIT_SEL_SET(val, 0x3);
		sds_wr(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG128_ADDR + os, val);

		sds_rd(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG145_ADDR + os, &val);
		val = CH0_RXTX_REG145_RXDFE_CONFIG_SET(val, 0x3);
		val = CH0_RXTX_REG145_TX_IDLE_SATA_SET(val, 0x0);
		val = CH0_RXTX_REG145_RXES_ENA_SET(val, 0x1);
		val = CH0_RXTX_REG145_RXVWES_LATENA_SET(val, 0x1);
		sds_wr(csr_base,
		       KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG145_ADDR + os, val);

		for (i = 0; i < 4; i++) {
			reg = KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG148_ADDR +
			    os + i * 2;
			sds_wr(csr_base, reg, 0xFFFF);
		}
	}
}

static int xgene_phy_cal_rdy_chk(struct xgene_ahci_phy_ctx *ctx)
{
	void *csr_serdes = ctx->csr_base + SATA_SERDES_OFFSET;
	int loopcount;
	u32 val;

	/* Relasase serdes main reset */
	phy_wr_flush(csr_serdes + SATA_ENET_SDS_RST_CTL_ADDR, 0x000000DF);

	if (!enable_manual_cal)
		goto skip_manual_cal;

	/* TERM CALIBRATION KC_SERDES_CMU_REGS_CMU_REG17__ADDR */
	/* TERM calibration for channel 0 */
	sds_rd(csr_serdes, KC_SERDES_CMU_REGS_CMU_REG17_ADDR, &val);
	val = CMU_REG17_PVT_CODE_R2A_SET(val, 0x12);
	val = CMU_REG17_RESERVED_7_SET(val, 0x0);
	sds_wr(csr_serdes, KC_SERDES_CMU_REGS_CMU_REG17_ADDR, val);
	sds_toggle1to0(csr_serdes, KC_SERDES_CMU_REGS_CMU_REG17_ADDR,
		       CMU_REG17_PVT_TERM_MAN_ENA_MASK);
	/* DOWN CALIBRATION for channel zero */
	sds_rd(csr_serdes, KC_SERDES_CMU_REGS_CMU_REG17_ADDR, &val);
	val = CMU_REG17_PVT_CODE_R2A_SET(val, 0x29);
	val = CMU_REG17_RESERVED_7_SET(val, 0x0);
	sds_wr(csr_serdes, KC_SERDES_CMU_REGS_CMU_REG17_ADDR, val);
	sds_toggle1to0(csr_serdes, KC_SERDES_CMU_REGS_CMU_REG16_ADDR,
		       CMU_REG16_PVT_DN_MAN_ENA_MASK);
	/* UP CALIBRATION for channel 0 */
	sds_rd(csr_serdes, KC_SERDES_CMU_REGS_CMU_REG17_ADDR, &val);
	val = CMU_REG17_PVT_CODE_R2A_SET(val, 0x28);
	val = CMU_REG17_RESERVED_7_SET(val, 0x0);
	sds_wr(csr_serdes, KC_SERDES_CMU_REGS_CMU_REG17_ADDR, val);
	sds_toggle1to0(csr_serdes, KC_SERDES_CMU_REGS_CMU_REG16_ADDR,
		       CMU_REG16_PVT_UP_MAN_ENA_MASK);

skip_manual_cal:
	/* Check for 10 ms */
	loopcount = 50;
	do {
		sds_rd(csr_serdes, KC_SERDES_CMU_REGS_CMU_REG7_ADDR, &val);
		if (CMU_REG7_PLL_CALIB_DONE_RD(val))
			break;
		udelay(200);	/* No need to poll more than 200 us */
	} while (--loopcount > 0);

	sds_rd(csr_serdes, KC_SERDES_CMU_REGS_CMU_REG7_ADDR, &val);
	if (CMU_REG7_PLL_CALIB_DONE_RD(val) == 1)
		dev_dbg(ctx->dev, "PLL calibration done\n");
	if (CMU_REG7_VCO_CAL_FAIL_RD(val) == 0x0) {
		dev_dbg(ctx->dev, "PLL calibration successful\n");
	} else {
		/* Assert SDS reset and recall calib function */
		dev_err(ctx->dev,
			"PLL calibration failed due to VCO failure\n");
		return -1;
	}

	dev_dbg(ctx->dev, "Checking TX ready\n");
	sds_rd(csr_serdes, KC_SERDES_CMU_REGS_CMU_REG15_ADDR, &val);
	dev_dbg(ctx->dev, "PHY TX is %sready\n", val & 0x300 ? "" : "NOT ");
	return 0;
}

static void xgene_phy_pdwn_force_vco(struct xgene_ahci_phy_ctx *ctx)
{
	void *csr_serdes = ctx->csr_base + SATA_SERDES_OFFSET;
	u32 val;

	dev_dbg(ctx->dev, "power down VCO\n");
	sds_rd(csr_serdes, KC_SERDES_CMU_REGS_CMU_REG16_ADDR, &val);
	val = CMU_REG16_VCOCAL_WAIT_BTW_CODE_SET(val, 0x5);
	sds_wr(csr_serdes, KC_SERDES_CMU_REGS_CMU_REG16_ADDR, val);

	sds_toggle1to0(csr_serdes, KC_SERDES_CMU_REGS_CMU_REG0_ADDR,
		       CMU_REG0_PDOWN_MASK);
	sds_toggle1to0(csr_serdes, KC_SERDES_CMU_REGS_CMU_REG32_ADDR,
		       CMU_REG32_FORCE_VCOCAL_START_MASK);
}

static void xgene_phy_tx_ssc_enable(struct xgene_ahci_phy_ctx *ctx)
{
	void *csr_serdes = ctx->csr_base + SATA_SERDES_OFFSET;
	u32 val;

	sds_rd(csr_serdes, KC_SERDES_CMU_REGS_CMU_REG35_ADDR, &val);
	val = CMU_REG35_PLL_SSC_MOD_SET(val, 98);
	sds_wr(csr_serdes, KC_SERDES_CMU_REGS_CMU_REG35_ADDR, val);

	sds_rd(csr_serdes, KC_SERDES_CMU_REGS_CMU_REG36_ADDR, &val);
	val = CMU_REG36_PLL_SSC_VSTEP_SET(val, 30);
	val = CMU_REG36_PLL_SSC_EN_SET(val, 1);
	val = CMU_REG36_PLL_SSC_DSMSEL_SET(val, 1);
	sds_wr(csr_serdes, KC_SERDES_CMU_REGS_CMU_REG36_ADDR, val);

	sds_clrbits(csr_serdes, KC_SERDES_CMU_REGS_CMU_REG5_ADDR,
		    CMU_REG5_PLL_RESETB_MASK);
	sds_setbits(csr_serdes, KC_SERDES_CMU_REGS_CMU_REG5_ADDR,
		    CMU_REG5_PLL_RESETB_MASK);
	sds_toggle1to0(csr_serdes, KC_SERDES_CMU_REGS_CMU_REG32_ADDR,
		       CMU_REG32_FORCE_VCOCAL_START_MASK);
}

static int xgene_phy_init(struct xgene_ahci_phy_ctx *ctx, int clk_type,
			  int ssc_enable)
{
	void *csr_base = ctx->csr_base;
	void *csr_serdes_base = csr_base + SATA_SERDES_OFFSET;
	void *clkcsr_base = ctx->csr_base + SATA_CLK_OFFSET;
	u32 val;
	int calib_loop_count;
	int rc;

	dev_dbg(ctx->dev, "PHY init clk type %d\n", clk_type);

	if (ctx->pcie_base &&
	    (clk_type == SATA_CLK_INT_DIFF || clk_type == SATA_CLK_INT_SING)) {
		if (xgene_phy_pcie_macro_cfg(ctx))
			return -ENODEV;
	}

	/* Select SATA mux if shared with SGMII ETH */
	if (!ctx->pcie_base && xgene_phy_host_sata_select(ctx) != 0) {
		dev_err(ctx->dev, "can not select SATA MUX\n");
		return -ENODEV;
	}

	/* Clock reset must before after select the MUX */
	xgene_phy_clk_rst_pre(ctx);

	if (!ctx->pcie_base &&
	    (clk_type == SATA_CLK_INT_DIFF || clk_type == SATA_CLK_INT_SING)) {
		if (xgene_phy_sata_macro_cfg(ctx))
			return -ENODEV;
	}

	dev_dbg(ctx->dev, "Reset PHY\n");
	phy_wr(csr_serdes_base + SATA_ENET_SDS_RST_CTL_ADDR, 0x0);
	/* Serdes main reset and Controller also under reset */
	phy_wr(csr_serdes_base + SATA_ENET_SDS_RST_CTL_ADDR, 0x20);

	/* Release all resets except  main reset */
	phy_wr(csr_serdes_base + SATA_ENET_SDS_RST_CTL_ADDR, 0xde);

	phy_rd(csr_serdes_base + SATA_ENET_SDS_CTL1_ADDR, &val);
	val = CFG_I_SPD_SEL_CDR_OVR1_SET(val, ctx->txspeed[ctx->speed[0]]);
	phy_wr(csr_serdes_base + SATA_ENET_SDS_CTL1_ADDR, val);

	dev_dbg(ctx->dev, "Setting the customer pin mode\n");
	/*
	 * Clear customer pins mode[13:0] = 0
	 * Set customer pins mode[14] = 1
	 */
	phy_rd(csr_serdes_base + SATA_ENET_SDS_CTL0_ADDR, &val);
	val = REGSPEC_CFG_I_CUSTOMER_PIN_MODE0_SET(val, 0x4421);
	phy_wr(csr_serdes_base + SATA_ENET_SDS_CTL0_ADDR, val);

	/* CMU_REG12 tx ready delay 0x2 */
	sds_rd(csr_serdes_base, KC_SERDES_CMU_REGS_CMU_REG12_ADDR, &val);
	val = CMU_REG12_STATE_DELAY9_SET(val, 0x1);
	sds_wr(csr_serdes_base, KC_SERDES_CMU_REGS_CMU_REG12_ADDR, val);
	sds_wr(csr_serdes_base, KC_SERDES_CMU_REGS_CMU_REG13_ADDR, 0xF222);
	sds_wr(csr_serdes_base,
		     KC_SERDES_CMU_REGS_CMU_REG14_ADDR, 0x2225);
	if (clk_type == SATA_CLK_EXT_DIFF) {
		sds_rd(csr_serdes_base, KC_SERDES_CMU_REGS_CMU_REG0_ADDR, &val);
		val = CMU_REG0_PLL_REF_SEL_SET(val, 0x0);
		sds_wr(csr_serdes_base, KC_SERDES_CMU_REGS_CMU_REG0_ADDR, val);
		sds_rd(csr_serdes_base, KC_SERDES_CMU_REGS_CMU_REG1_ADDR, &val);
		val = CMU_REG1_REFCLK_CMOS_SEL_SET(val, 0x0);
		sds_wr(csr_serdes_base, KC_SERDES_CMU_REGS_CMU_REG1_ADDR, val);
		dev_dbg(ctx->dev, "Setting external reference clock\n");
	} else if (clk_type == SATA_CLK_INT_DIFF) {
		sds_rd(csr_serdes_base, KC_SERDES_CMU_REGS_CMU_REG0_ADDR, &val);
		val = CMU_REG0_PLL_REF_SEL_SET(val, 0x1);
		sds_wr(csr_serdes_base, KC_SERDES_CMU_REGS_CMU_REG0_ADDR, val);
		sds_rd(csr_serdes_base, KC_SERDES_CMU_REGS_CMU_REG1_ADDR, &val);
		val = CMU_REG1_REFCLK_CMOS_SEL_SET(val, 0x1);
		sds_wr(csr_serdes_base, KC_SERDES_CMU_REGS_CMU_REG1_ADDR, val);

		dev_dbg(ctx->dev, "Setting internal reference clock\n");
	} else if (clk_type == SATA_CLK_INT_SING) {
		sds_rd(csr_serdes_base, KC_SERDES_CMU_REGS_CMU_REG1_ADDR, &val);
		val = CMU_REG1_REFCLK_CMOS_SEL_SET(val, 0x1);
		sds_wr(csr_serdes_base, KC_SERDES_CMU_REGS_CMU_REG1_ADDR, val);
		dev_dbg(ctx->dev,
			"Setting internal single ended reference clock\n");
	}
	/* PCIe clock macro has no support for CML1 */
	if (ctx->pcie_base && clk_type == SATA_CLK_INT_DIFF)
		sds_setbits(csr_serdes_base, KC_SERDES_CMU_REGS_CMU_REG1_ADDR,
			    CMU_REG1_REFCLK_CMOS_SEL_MASK);

	/* Program serdes registers */
	xgene_phy_validation_CMU_cfg(ctx, clk_type);
	if (ssc_enable)
		xgene_phy_tx_ssc_enable(ctx);
	xgene_phy_validation_rxtx_cfg(ctx);

	phy_rd(csr_serdes_base + SATA_ENET_SDS_PCS_CTL0_ADDR, &val);
	val = REGSPEC_CFG_I_RX_WORDMODE0_SET(val, 0x3);
	val = REGSPEC_CFG_I_TX_WORDMODE0_SET(val, 0x3);
	phy_wr(csr_serdes_base + SATA_ENET_SDS_PCS_CTL0_ADDR, val);

	mb();

	calib_loop_count = 10;
	do {
		rc = xgene_phy_cal_rdy_chk(ctx);
		if (rc == 0)
			break;
		xgene_phy_pdwn_force_vco(ctx);
	} while (++calib_loop_count > 0);
	if (calib_loop_count <= 0) {
		dev_err(ctx->dev, "PLL calibration failed\n");
		return -ENODEV;
	}

	/* Enable remaining clock and CSR reset */
	phy_wr_flush(clkcsr_base + SATACLKENREG_ADDR, 0xff);
	xgene_phy_reset_sds_pmclk_core(ctx);
	xgene_phy_reset_pclk(ctx);

	return 0;
}

static void xgene_ahci_serdes_force_gen(struct xgene_ahci_phy_ctx *ctx,
					int chan, int gen)
{
	void *csr_base = ctx->csr_base;
	void *csr_serdes = csr_base + SATA_SERDES_OFFSET;
	u32 val;

	phy_rd(csr_serdes + SATA_ENET_SDS_CTL1_ADDR, &val);
	val = CFG_I_SPD_SEL_CDR_OVR1_SET(val, gen);
	phy_wr(csr_serdes + SATA_ENET_SDS_CTL1_ADDR, val);

	sds_rd(csr_serdes, KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG0_ADDR +
	       chan * 0x200, &val);
	val = CH0_RXTX_REG0_CTLE_EQ_HR_SET(val, 0x1c);
	val = CH0_RXTX_REG0_CTLE_EQ_QR_SET(val, 0x1c);
	val = CH0_RXTX_REG0_CTLE_EQ_FR_SET(val, 0x1c);
	sds_wr(csr_serdes, KC_SERDES_X2_RXTX_REGS_CH0_RXTX_REG0_ADDR +
	       chan * 0x200, val);
}

static int xgene_ahci_phy_hw_init(struct phy *phy)
{
	struct xgene_ahci_phy_ctx *ctx = phy_get_drvdata(phy);
	int rc;

	rc = xgene_phy_init(ctx, SATA_CLK_EXT_DIFF, 0 /* SSC */);
	if (rc != 0) {
		dev_err(ctx->dev, "PHY initialize failed %d\n", rc);
		return rc;
	}

	xgene_phy_gen_avg_val(ctx, 1);
	xgene_phy_gen_avg_val(ctx, 0);

	dev_dbg(ctx->dev, "PHY initialized\n");
	return 0;
}

static int xgene_ahci_phy_set_speed(struct phy *phy, int lane, u64 speed)
{
	struct xgene_ahci_phy_ctx *ctx = phy_get_drvdata(phy);

	if (lane >= MAX_CHANNEL)
		return -EINVAL;
	if (speed >= 6000000000 /* 6Gbps */) {
		ctx->speed[lane] = 2;
		xgene_ahci_serdes_force_gen(ctx, lane, SPD_SEL_GEN3);
	} else if (speed >= 3000000000 /* 3Gbps */) {
		ctx->speed[lane] = 1;
		xgene_ahci_serdes_force_gen(ctx, lane, SPD_SEL_GEN2);
	} else /* 1.5Gbps */ {
		ctx->speed[lane] = 0;
		xgene_ahci_serdes_force_gen(ctx, lane, SPD_SEL_GEN1);
	}
	return 0;
}

static int xgene_ahci_phy_power_on(struct phy *phy)
{
	return 0;
}

static int xgene_ahci_phy_power_off(struct phy *phy)
{
	return 0;
}

static const struct phy_ops xgene_phy_ops = {
	.init		= xgene_ahci_phy_hw_init,
	.power_on	= xgene_ahci_phy_power_on,
	.power_off	= xgene_ahci_phy_power_off,
	.set_speed	= xgene_ahci_phy_set_speed,
	.owner		= THIS_MODULE,
};

static struct phy *xgene_ahci_phy_xlate(struct device *dev,
	struct of_phandle_args *args)
{
	struct xgene_ahci_phy_ctx *ctx = dev_get_drvdata(dev);

	return ctx->phy;
}

static void xgene_ahci_phy_get_param(struct platform_device *pdev,
	const char *name, u32 *buffer, int count, u32 default_val)
{
	int rc;
	int i;
	rc = of_property_read_u32_array(pdev->dev.of_node, name, buffer,
					count);
	if (rc) {
		for (i = 0; i < count; i++)
			buffer[i] = default_val;
	}
}

static int xgene_ahci_phy_probe(struct platform_device *pdev)
{
	struct phy_provider *phy_provider;
	struct xgene_ahci_phy_ctx *ctx;
	struct resource *res;
	int rc = 0;

	if (!of_device_is_available(pdev->dev.of_node))
		return -ENODEV;

	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		dev_err(&pdev->dev, "can't allocate PHY context\n");
		return -ENOMEM;
	}
	ctx->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no PHY resource address\n");
		goto error;
	}
	ctx->csr_phys = res->start;
	ctx->csr_base = devm_ioremap(&pdev->dev, res->start,
				     resource_size(res));
	if (!ctx->csr_base) {
		dev_err(&pdev->dev, "can't map PHY resource\n");
		rc = -ENOMEM;
		goto error;
	}

	if (of_device_is_compatible(pdev->dev.of_node, XGENE_PHY2_DTS)) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		if (!res) {
			dev_err(&pdev->dev,
				"no SATA/PCIE PHY resource address\n");
			goto error;
		}
		ctx->pcie_base = devm_ioremap(&pdev->dev, res->start,
					      resource_size(res));
		if (!ctx->pcie_base) {
			dev_err(&pdev->dev,
				"can't map SATA/PCIe PHY resource\n");
			rc = -ENOMEM;
			goto error;
		}
	}

	/* Load override paramaters */
	xgene_ahci_phy_get_param(pdev, "txeyetuning", ctx->txeyetuning, 6,
				 DEFAULT_TXEYETUNING);
	xgene_ahci_phy_get_param(pdev, "txeyedirection", ctx->txeyedirection,
				 6, DEFAULT_TXEYEDIRECTION);
	xgene_ahci_phy_get_param(pdev, "txboostgain", ctx->txboostgain, 6,
				 DEFAULT_TXBOOST_GAIN);
	xgene_ahci_phy_get_param(pdev, "txspeed", ctx->txspeed, 3,
				 DEFAULT_SPD_SEL);
	ctx->speed[0] = 2;	/* Default to Gen3 for channel 0 */
	ctx->speed[1] = 2;	/* Default to Gen3 for channel 1 */

	ctx->dev = &pdev->dev;
	platform_set_drvdata(pdev, ctx);

	phy_provider = devm_of_phy_provider_register(ctx->dev,
						     xgene_ahci_phy_xlate);
	if (IS_ERR(phy_provider)) {
		rc = PTR_ERR(phy_provider);
		goto error;
	}

	ctx->phy = devm_phy_create(ctx->dev, &xgene_phy_ops, NULL);
	if (IS_ERR(ctx->phy)) {
		dev_dbg(&pdev->dev, "Failed to create PHY\n");
		return PTR_ERR(ctx->phy);
	}

	phy_set_drvdata(ctx->phy, ctx);

	dev_info(&pdev->dev, "X-Gene PHY @0x%llX %sregistered\n",
		 ctx->csr_phys, ctx->pcie_base ? "(PCIe) " : "");
	return 0;

error:
	return rc;
}

static int xgene_ahci_phy_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id xgene_ahci_phy_of_match[] = {
	{.compatible = XGENE_PHY_DTS,},
	{.compatible = XGENE_PHY2_DTS, },
	{},
};
MODULE_DEVICE_TABLE(of, xgene_ahci_phy_of_match);

static struct platform_driver xgene_ahci_phy_driver = {
	.probe = xgene_ahci_phy_probe,
	.remove = xgene_ahci_phy_remove,
	.driver = {
		   .name = "xgene-ahci-phy",
		   .owner = THIS_MODULE,
		   .of_match_table = xgene_ahci_phy_of_match,
	},
};

static int __init xgene_ahci_phy_init(void)
{
	return platform_driver_register(&xgene_ahci_phy_driver);
}
subsys_initcall(xgene_ahci_phy_init);

static void __exit xgene_ahci_phy_exit(void)
{
	platform_driver_unregister(&xgene_ahci_phy_driver);
}
module_exit(xgene_ahci_phy_exit);

MODULE_DESCRIPTION("APM X-Gene AHCI PHY driver");
MODULE_AUTHOR("Loc Ho <lho@apm.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
