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
struct xgene_enet_stats;

/* clears and then set bits */
static inline void set_bits(u32 *dst, u32 val, u32 start, u32 len)
{
	u32 end = start + len - 1;
	u32 mask = GENMASK(end, start);

	*dst &= ~mask;
	*dst |= (val << start) & mask;
}

static inline u32 get_bits(u32 val, u32 start, u32 end)
{
	return (val & GENMASK(end, start)) >> start;
}

#define CSR_RING_ID		0x00000008
#define OVERWRITE		BIT(31)
#define IS_BUFFER_POOL		BIT(20)
#define PREFETCH_BUF_EN		BIT(21)
#define CSR_RING_ID_BUF		0x0000000c
#define CSR_RING_NE_INT_MODE	0x0000017c
#define CSR_RING_CONFIG		0x0000006c
#define CSR_RING_WR_BASE	0x00000070
#define NUM_RING_CONFIG		5
#define BUFPOOL_MODE		3
#define RM3			3
#define INC_DEC_CMD_ADDR	0x2c
#define IS_FP(x) ((x & 0x0020) ? 1 : 0)
#define UDP_HDR_SIZE		2

#define CREATE_MASK(pos, len)		GENMASK(pos+len-1, pos)
#define CREATE_MASK_ULL(pos, len)	GENMASK_ULL(pos+len-1, pos)

/* Empty slot soft signature */
#define EMPTY_SLOT_INDEX	1
#define EMPTY_SLOT		~0ULL

#define RING_BUFNUM(q)		(q->id & 0x003F)
#define RING_OWNER(q)		((q->id & 0x03C0) >> 6)
#define BUF_LEN_CODE_2K		0x5000

#define SELTHRSH_POS		3
#define SELTHRSH_LEN		3
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
#define RECOMTIMEOUTL_POS	28
#define RECOMTIMEOUTL_LEN	3
#define RECOMTIMEOUTH_POS	0
#define RECOMTIMEOUTH_LEN	2
#define NUMMSGSINQ_POS		1
#define NUMMSGSINQ_LEN		16
#define ACCEPTLERR		BIT(19)
#define QCOHERENT		BIT(4)
#define RECOMBBUF		BIT(27)

#define BLOCK_ETH_CSR_OFFSET		0x2000
#define BLOCK_ETH_RING_IF_OFFSET	0x9000
#define BLOCK_ETH_CLKRST_CSR_OFFSET	0xC000
#define BLOCK_ETH_DIAG_CSR_OFFSET	0xD000

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
#define MII_MGMT_CONFIG_ADDR		0x20
#define MII_MGMT_COMMAND_ADDR		0x24
#define MII_MGMT_ADDRESS_ADDR		0x28
#define MII_MGMT_CONTROL_ADDR		0x2c
#define MII_MGMT_STATUS_ADDR		0x30
#define MII_MGMT_INDICATORS_ADDR	0x34

#define BUSY_MASK			BIT(0)
#define READ_CYCLE_MASK			BIT(0)
#define PHY_CONTROL_SET(dst, val)	set_bits(dst, val, 0, 16)

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
#define CFG_TXCLK_MUXSEL0_SET(dst, val)	set_bits(dst, val, 29, 3)
#define CFG_CLE_BYPASS_EN0		BIT(31)

#define CFG_CLE_IP_PROTOCOL0_SET(dst, val)	set_bits(dst, val, 16, 2)
#define CFG_CLE_DSTQID0_SET(dst, val)		set_bits(dst, val, 0, 12)
#define CFG_CLE_FPSEL0_SET(dst, val)		set_bits(dst, val, 16, 4)
#define CFG_MACMODE_SET(dst, val)		set_bits(dst, val, 18, 2)
#define CFG_WAITASYNCRD_SET(dst, val)		set_bits(dst, val, 0, 16)
#define ICM_CONFIG0_REG_0_ADDR		0x00000400
#define ICM_CONFIG2_REG_0_ADDR		0x00000410
#define RX_DV_GATE_REG_0_ADDR		0x000005fc
#define TX_DV_GATE_EN0			BIT(2)
#define RX_DV_GATE_EN0			BIT(1)
#define RESUME_RX0			BIT(0)
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
#define SCAN_CYCLE_MASK_SET(dst, src)		set_bits(dst, val, 0, 1)
#define PHY_ADDR_SET(dst, val)			set_bits(dst, val, 8, 5)
#define REG_ADDR_SET(dst, val)			set_bits(dst, val, 0, 5)
#define ENET_INTERFACE_MODE2_SET(dst, val)	set_bits(dst, val, 8, 2)
#define MGMT_CLOCK_SEL_SET(dst, val)		set_bits(dst, val, 0, 3)
#define SOFT_RESET1			BIT(31)
#define TX_EN				BIT(0)
#define RX_EN				BIT(2)
#define ENET_LHD_MODE			BIT(25)
#define ENET_GHD_MODE			BIT(26)
#define FULL_DUPLEX2			BIT(0)
#define SCAN_AUTO_INCR			BIT(5)
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
	u64 *desc;

	desc = desc_ptr;
	desc[1] = cpu_to_le64(desc[1]);
}

static inline void xgene_enet_le64_to_desc16(void *desc_ptr)
{
	u64 *desc;

	desc = desc_ptr;
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
void xgene_gmac_init(struct xgene_enet_pdata *priv, int speed);
void xgene_gmac_tx_enable(struct xgene_enet_pdata *priv);
void xgene_gmac_rx_enable(struct xgene_enet_pdata *priv);
void xgene_gmac_tx_disable(struct xgene_enet_pdata *priv);
void xgene_gmac_rx_disable(struct xgene_enet_pdata *priv);
void xgene_gmac_set_mac_addr(struct xgene_enet_pdata *pdata);
void xgene_enet_cle_bypass(struct xgene_enet_pdata *priv,
			   u32 dst_ring_num, u32 fpsel, u32 nxtfpsel);
void xgene_gport_shutdown(struct xgene_enet_pdata *priv);
void xgene_gmac_get_stats(struct xgene_enet_pdata *priv,
			  struct xgene_enet_stats *stats);
#endif /* __XGENE_ENET_HW_H__ */
