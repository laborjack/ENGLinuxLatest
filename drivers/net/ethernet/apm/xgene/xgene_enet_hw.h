/* Applied Micro X-Gene SoC Ethernet Driver
 *
 * Copyright (c) 2014, Applied Micro Circuits Corporation
 * Authors: Iyappan Subramanian <isubramanian@apm.com>
 *	    Ravi Patel <rapatel@apm.com>
 *	    Keyur Chudgar <kchudgar@apm.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __XGENE_ENET_HW_H__
#define __XGENE_ENET_HW_H__

#include "xgene_enet_main.h"

struct xgene_enet_pdata;
struct xgene_enet_detailed_stats;

#define CSR_RING_ID		0x00000008
#define OVERWRITE		BIT(31)
#define CSR_RING_ID_BUF		0x0000000c
#define CSR_RING_NE_INT_MODE	0x0000017c
#define CSR_RING_CONFIG		0x0000006c
#define CSR_RING_WR_BASE	0x00000070
#define NUM_RING_CONFIG		5
#define BUFPOOL_MODE		3

/* Empty slot soft signature */
#define EMPTY_SLOT_INDEX	1
#define EMPTY_SLOT		~(u64)0

#define RING_BUFNUM(q)		(q->id & 0x003F)
#define RING_OWNER(q)		((q->id & 0x03C0) >> 6)
#define BUF_LEN_CODE_2K		0x5000

#define SELTHRSH_POS		3
#define SELTHRSH_LEN		3
#define ACCEPTLERR_POS		19
#define ACCEPTLERR_LEN		1
#define QCOHERENT_POS		4
#define QCOHERENT_LEN		1
#define RINGADDRL_POS		5
#define RINGADDRL_LEN		27
#define RINGADDRH_POS		0
#define RINGADDRH_LEN		6
#define RINGSIZE_POS		23
#define RINGSIZE_LEN		3
#define RINGTYPE_POS		19
#define RINGTYPE_LEN		2
#define RINGMODE_POS		20
#define RINGMODE_LEN		3
#define RECOMBBUF_POS		27
#define RECOMBBUF_LEN		1
#define RECOMTIMEOUTL_POS	28
#define RECOMTIMEOUTL_LEN	3
#define RECOMTIMEOUTH_POS	0
#define RECOMTIMEOUTH_LEN	2
#define NUMMSGSINQ_POS		1
#define NUMMSGSINQ_LEN		16

#define CREATE_MASK(pos, len)		GENMASK(pos+len-1, pos)
#define CREATE_MASK_ULL(pos, len)	GENMASK_ULL(pos+len-1, pos)

#define IS_FP(x) ((x & 0x0020) ? 1 : 0)

#ifndef UDP_HDR_SIZE
#define UDP_HDR_SIZE		2
#endif

/* Direct Address mode */
#define BLOCK_ETH_CSR_OFFSET		0x2000
#define BLOCK_ETH_RING_IF_OFFSET	0x9000
#define BLOCK_ETH_CLKRST_CSR_OFFSET	0xC000
#define BLOCK_ETH_DIAG_CSR_OFFSET	0xD000

/* Indirect & Direct  Address mode for MCX_MAC and AXG_MAC */
#define BLOCK_ETH_MAC_OFFSET		0x0000
#define BLOCK_ETH_STATS_OFFSET		0x0014
#define BLOCK_ETH_MAC_CSR_OFFSET	0x2800

/* Constants for indirect registers */
#define MAC_ADDR_REG_OFFSET		0
#define MAC_COMMAND_REG_OFFSET		4
#define MAC_WRITE_REG_OFFSET		8
#define MAC_READ_REG_OFFSET		12
#define MAC_COMMAND_DONE_REG_OFFSET	16

#define STAT_ADDR_REG_OFFSET		0
#define STAT_COMMAND_REG_OFFSET		4
#define STAT_WRITE_REG_OFFSET		8
#define STAT_READ_REG_OFFSET		12
#define STAT_COMMAND_DONE_REG_OFFSET	16

/* Address PE_MCXMAC  Registers */
#define MII_MGMT_CONFIG_ADDR		0x00000020
#define MII_MGMT_COMMAND_ADDR		0x00000024
#define MII_MGMT_ADDRESS_ADDR		0x00000028
#define MII_MGMT_CONTROL_ADDR		0x0000002c
#define MII_MGMT_STATUS_ADDR		0x00000030
#define MII_MGMT_INDICATORS_ADDR	0x00000034

#define BUSY_MASK			BIT(0)
#define READ_CYCLE_MASK			BIT(0)
#define PHY_CONTROL_WR(src)		(((u32)(src)) & GENMASK(15, 0))

#define ENET_SPARE_CFG_REG_ADDR		0x00000750
#define RSIF_CONFIG_REG_ADDR		0x00000010
#define RSIF_RAM_DBG_REG0_ADDR		0x00000048
#define RGMII_REG_0_ADDR		0x000007e0
#define CFG_LINK_AGGR_RESUME_0_ADDR	0x000007c8
#define DEBUG_REG_ADDR			0x00000700
#define CFG_BYPASS_ADDR			0x00000294
#define CLE_BYPASS_REG0_0_ADDR		0x00000490
#define CLE_BYPASS_REG1_0_ADDR		0x00000494
#define CFG_RSIF_FPBUFF_TIMEOUT_EN	BIT(31)
#define RESUME_TX			BIT(0)
#define CFG_SPEED_1250			BIT(24)
#define TX_PORT0			BIT(0)
#define CFG_BYPASS_UNISEC_TX		BIT(2)
#define CFG_BYPASS_UNISEC_RX		BIT(1)
#define CFG_TXCLK_MUXSEL0_WR(src)	(((u32) (src) << 29) & GENMASK(31, 20))
#define CFG_CLE_BYPASS_EN0		BIT(31)
#define CFG_CLE_IP_PROTOCOL0_SET(dst, src) \
	(((dst) & ~GENMASK(17, 16)) | (((u32) (src) << 16) & GENMASK(17, 16)))
#define CFG_CLE_DSTQID0_SET(dst, src) \
	(((dst) & ~GENMASK(11, 0)) | (((u32) (src)) & GENMASK(11, 0)))
#define CFG_CLE_FPSEL0_SET(dst, src) \
	(((dst) & ~GENMASK(19, 16)) | (((u32) (src) << 16) & GENMASK(19, 16)))
#define CFG_CLE_NXTFPSEL0_SET(dst, src) \
	(((dst) & ~GENMASK(23, 20)) | (((u32) (src) << 20) & GENMASK(23, 20)))
#define CFG_MACMODE_SET(dst, src) \
	(((dst) & ~GENMASK(19, 18)) | (((u32) (src) << 18) & GENMASK(19, 18)))
#define CFG_WAITASYNCRD_SET(dst, src) \
	(((dst) & ~GENMASK(15, 0)) | (((u32) (src) << 15) & GENMASK(15, 0)))
#define ICM_CONFIG0_REG_0_ADDR		0x00000400
#define ICM_CONFIG2_REG_0_ADDR		0x00000410
#define RX_DV_GATE_REG_0_ADDR		0x000005fc
#define TX_DV_GATE_EN0_SET(dst, src) \
	(((dst) & ~BIT(2)) | (((u32) (src) << 2) & BIT(2)))
#define RX_DV_GATE_EN0_SET(dst, src) \
	(((dst) & ~BIT(1)) | (((u32) (src) << 1) & BIT(1)))
#define RESUME_RX0_SET(dst, src) \
	(((dst) & ~BIT(0)) | (((u32) (src)) & BIT(0)))
#define ENET_CFGSSQMIWQASSOC_ADDR		0x000000e0
#define ENET_CFGSSQMIFPQASSOC_ADDR		0x000000dc
#define ENET_CFGSSQMIQMLITEFPQASSOC_ADDR	0x000000f0
#define ENET_CFGSSQMIQMLITEWQASSOC_ADDR		0x000000f4

#define ENET_CFG_MEM_RAM_SHUTDOWN_ADDR		0x00000070
#define ENET_BLOCK_MEM_RDY_ADDR			0x00000074
#define MAC_CONFIG_1_ADDR			0x00000000
#define MAC_CONFIG_2_ADDR			0x00000004
#define MAX_FRAME_LEN_ADDR			0x00000010
#define INTERFACE_CONTROL_ADDR			0x00000038
#define STATION_ADDR0_ADDR			0x00000040
#define STATION_ADDR1_ADDR			0x00000044
#define SCAN_CYCLE_MASK_SET(dst, src) \
	(((dst) & ~BIT(1)) | (((u32)(src)) & BIT(1)))
#define SOFT_RESET1_MASK		BIT(31)
#define PHY_ADDR_WR(src)		(((u32) (src) < 8) & GENMASK(12, 8))
#define PHY_ADDR_SET(dst, src) \
	(((dst) & ~GENMASK(12, 8)) | (((u32) (src) << 8) & GENMASK(12, 8)))
#define REG_ADDR_WR(src)		(((u32) (src)) & GENMASK(4, 0))
#define REG_ADDR_SET(dst, src) \
	(((dst) & ~GENMASK(4, 0)) | (((u32)(src)) & GENMASK(4, 0)))
#define RESET_TX_FUN1_WR(src)		BIT(16)
#define RESET_RX_FUN1_WR(src)		BIT(17)
#define RESET_TX_MC1_WR(src)		BIT(18)
#define RESET_RX_MC1_WR(src)		BIT(19)
#define SIM_RESET1_WR(src)		BIT(30)
#define SOFT_RESET1_WR(src)		BIT(31)
#define TX_EN1_WR(src)			BIT(0)
#define RX_EN1_WR(src)			BIT(2)
#define ENET_LHD_MODE_WR(src)		BIT(25)
#define ENET_GHD_MODE_WR(src)		BIT(26)
#define FULL_DUPLEX2_WR(src)		BIT(0)
#define ENET_INTERFACE_MODE2_WR(src)	(((u32) (src) << 8) & GENMASK(9, 8))
#define PREAMBLE_LENGTH2_WR(src)	(((u32) (src) << 12) & GENMASK(15, 12))
#define MAX_FRAME_LEN_WR(src)		(((u32) (src)) & GENMASK(15, 0))
#define MGMT_CLOCK_SEL_SET(dst, src) \
	(((dst) & ~GENMASK(2, 0)) | (((u32) (src)) & GENMASK(2, 0)))
#define SCAN_AUTO_INCR_MASK		0x00000020
#define RBYT_ADDR			0x00000027
#define RPKT_ADDR			0x00000028
#define RFCS_ADDR			0x00000029
#define RALN_ADDR			0x0000002f
#define RFLR_ADDR			0x00000030
#define RUND_ADDR			0x00000033
#define ROVR_ADDR			0x00000034
#define RDRP_ADDR			0x00000037
#define TBYT_ADDR			0x00000038
#define TPKT_ADDR			0x00000039
#define TDRP_ADDR			0x00000045
#define TFCS_ADDR			0x00000047
#define TUND_ADDR			0x0000004a
#define RX_BYTE_CNTR_MASK		0x7fffffff
#define RX_PKT_CNTR_MASK		0x7fffffff
#define RX_FCS_ERROR_CNTR_MASK		0x0000ffff
#define RX_ALIGN_ERR_CNTR_MASK		0x0000ffff
#define RX_LEN_ERR_CNTR_MASK		0x0000ffff
#define RX_UNDRSIZE_PKT_CNTR_MASK	0x0000ffff
#define RX_OVRSIZE_PKT_CNTR_MASK	0x0000ffff
#define RX_DROPPED_PKT_CNTR_MASK	0x0000ffff
#define TX_BYTE_CNTR_MASK		0x7fffffff
#define TX_PKT_CNTR_MASK		0x7fffffff
#define TX_DROP_FRAME_CNTR_MASK		0x0000ffff
#define TX_FCS_ERROR_CNTR_MASK		0x00000fff
#define TX_UNDSIZE_FRAME_CNTR_MASK	0x00000fff

#define TSO_IPPROTO_TCP			1
#define TSO_IPPROTO_UDP			0
#define	FULL_DUPLEX			2

#define USERINFO_POS			0
#define USERINFO_LEN			32
#define FPQNUM_POS			32
#define FPQNUM_LEN			12
#define STASH_POS			53
#define STASH_LEN			2
#define BUFDATALEN_POS			48
#define BUFDATALEN_LEN			12
#define BUFLEN_POS			60
#define BUFLEN_LEN			3
#define DATAADDR_POS			0
#define DATAADDR_LEN			42
#define COHERENT_POS			63
#define COHERENT_LEN			1
#define HENQNUM_POS			48
#define HENQNUM_LEN			12
#define TYPESEL_POS			44
#define TYPESEL_LEN			4
#define ETHHDR_POS			12
#define ETHHDR_LEN			8
#define IC_POS				35	/* Insert CRC */
#define IC_LEN				1
#define TCPHDR_POS			0
#define TCPHDR_LEN			6
#define IPHDR_POS			6
#define IPHDR_LEN			5
#define EC_POS				22	/* Enable checksum */
#define EC_LEN				1
#define IS_POS				24	/* IP protocol select */
#define IS_LEN				1

struct xgene_enet_desc {
	u64 m0;
	u64 m1;
	u64 m2;
	u64 m3;
};

struct xgene_enet_desc16 {
	u64 m0;
	u64 m1;
};

static inline void xgene_enet_cpu_to_le64(void *desc_ptr, int count)
{
	u64 *desc = desc_ptr;
	int i;

	for (i = 0; i < count; i++)
		desc[i] = cpu_to_le64(desc[i]);
}

static inline void xgene_enet_le64_to_cpu(void *desc_ptr, int count)
{
	u64 *desc = desc_ptr;
	int i;

	for (i = 0; i < count; i++)
		desc[i] = le64_to_cpu(desc[i]);
}

static inline void xgene_enet_desc16_to_le64(void *desc_ptr)
{
	u64 *desc = desc_ptr;

	desc[1] = cpu_to_le64(desc[1]);
}

static inline void xgene_enet_le64_to_desc16(void *desc_ptr)
{
	u64 *desc = desc_ptr;

	desc[1] = le64_to_cpu(desc[1]);
}

enum xgene_enet_ring_cfgsize {
	RING_CFGSIZE_512B,
	RING_CFGSIZE_2KB,
	RING_CFGSIZE_16KB,
	RING_CFGSIZE_64KB,
	RING_CFGSIZE_512KB,
	RING_CFGSIZE_INVALID
};

enum xgene_enet_ring_type {
	RING_DISABLED,
	RING_REGULAR,
	RING_BUFPOOL
};

enum xgene_enet_ring_owner {
	RING_OWNER_ETH0,
	RING_OWNER_CPU = 15,
	RING_OWNER_INVALID
};

enum xgene_enet_ring_bufnum {
	RING_BUFNUM_REGULAR = 0x0,
	RING_BUFNUM_BUFPOOL = 0x20,
	RING_BUFNUM_INVALID
};

struct xgene_enet_desc_ring *xgene_enet_setup_ring(
		struct xgene_enet_desc_ring *ring);
void xgene_enet_clear_ring(struct xgene_enet_desc_ring *ring);

enum desc_info_index {
	USERINFO,
	FPQNUM,
	STASH,
	DATAADDR,
	BUFDATALEN,
	BUFLEN,
	COHERENT,
	TCPHDR,
	IPHDR,
	ETHHDR,
	EC,
	IS,
	IC,
	TYPESEL,
	HENQNUM,
	MAX_DESC_INFO_INDEX
};

struct xgene_enet_desc_info {
	u8 word_index;
	u8 start_bit;
	u8 len;
};

enum xgene_enet_cmd {
	XGENE_ENET_WR_CMD = BIT(31),
	XGENE_ENET_RD_CMD = BIT(30)
};

void set_desc(void *desc_ptr, enum desc_info_index index, u64 val);
u64 get_desc(void *desc_ptr, enum desc_info_index index);

void xgene_enet_reset(struct xgene_enet_pdata *priv);
void xgene_gmac_reset(struct xgene_enet_pdata *priv);
void xgene_gmac_init(struct xgene_enet_pdata *priv, unsigned char *dev_addr,
		     int speed);
void xgene_gmac_tx_enable(struct xgene_enet_pdata *priv);
void xgene_gmac_rx_enable(struct xgene_enet_pdata *priv);
void xgene_gmac_tx_disable(struct xgene_enet_pdata *priv);
void xgene_gmac_rx_disable(struct xgene_enet_pdata *priv);
void xgene_gmac_set_mac_addr(struct xgene_enet_pdata *pdata,
			     unsigned char *dev_addr);
void xgene_enet_cle_bypass_mode_cfg(struct xgene_enet_pdata *priv,
				    u32 dst_ring_num, u32 fpsel, u32 nxtfpsel);
void xgene_gport_shutdown(struct xgene_enet_pdata *priv);
void xgene_gmac_get_detailed_stats(struct xgene_enet_pdata *priv,
				   struct xgene_enet_detailed_stats
				   *detailed_stats);
#endif /* __XGENE_ENET_HW_H__ */
