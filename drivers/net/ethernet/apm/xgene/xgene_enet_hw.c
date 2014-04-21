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

#include <linux/of_platform.h>
#include "xgene_enet_main.h"
#include "xgene_enet_hw.h"

struct xgene_enet_desc_info desc_info[MAX_DESC_INFO_INDEX] = {
	[USERINFO] = {0, USERINFO_POS, USERINFO_LEN},
	[FPQNUM] = {0, FPQNUM_POS, FPQNUM_LEN},
	[STASH] = {0, STASH_POS, STASH_LEN},
	[DATAADDR] = {1, DATAADDR_POS, DATAADDR_LEN},
	[BUFDATALEN] = {1, BUFDATALEN_POS, BUFDATALEN_LEN},
	[BUFLEN] = {1, BUFLEN_POS, BUFLEN_LEN},
	[COHERENT] = {1, COHERENT_POS, COHERENT_LEN},
	[TCPHDR] = {3, TCPHDR_POS, TCPHDR_LEN},
	[IPHDR] = {3, IPHDR_POS, IPHDR_LEN},
	[ETHHDR] = {3, ETHHDR_POS, ETHHDR_LEN},
	[EC] = {3, EC_POS, EC_LEN},
	[IS] = {3, IS_POS, IS_LEN},
	[IC] = {3, IC_POS, IC_LEN},
	[TYPESEL] = {3, TYPESEL_POS, TYPESEL_LEN},
	[HENQNUM] = {3, HENQNUM_POS, HENQNUM_LEN},
};

void set_desc(void *desc_ptr, enum desc_info_index index, u64 val)
{
	u64 *desc;
	u8 i, start_bit, len;
	u64 mask;

	desc = desc_ptr;
	i = desc_info[index].word_index;
	start_bit = desc_info[index].start_bit;
	len = desc_info[index].len;
	mask = GENMASK_ULL((start_bit + len - 1), start_bit);
	desc[i] = (desc[i] & ~mask) | ((val << start_bit) & mask);
}

u64 get_desc(void *desc_ptr, enum desc_info_index index)
{
	u64 *desc;
	u8 i, start_bit, len;
	u64 mask;

	desc = desc_ptr;
	i = desc_info[index].word_index;
	start_bit = desc_info[index].start_bit;
	len = desc_info[index].len;
	mask = GENMASK_ULL((start_bit + len - 1), start_bit);

	return (desc[i] & mask) >> start_bit;
}

static void xgene_enet_ring_init(u32 *ring_cfg, u64 addr,
				 enum xgene_enet_ring_cfgsize cfgsize)
{
	ring_cfg[4] |= ((u32) 1 << SELTHRSH_POS)
	    & CREATE_MASK(SELTHRSH_POS, SELTHRSH_LEN);
	ring_cfg[3] |= ((u32) 1 << ACCEPTLERR_POS)
	    & CREATE_MASK(ACCEPTLERR_POS, ACCEPTLERR_LEN);
	ring_cfg[2] |= ((u32) 1 << QCOHERENT_POS)
	    & CREATE_MASK(QCOHERENT_POS, QCOHERENT_LEN);

	addr >>= 8;
	ring_cfg[2] |= (addr << RINGADDRL_POS)
	    & CREATE_MASK_ULL(RINGADDRL_POS, RINGADDRL_LEN);
	addr >>= RINGADDRL_LEN;
	ring_cfg[3] |= addr & CREATE_MASK_ULL(RINGADDRH_POS, RINGADDRH_LEN);
	ring_cfg[3] |= ((u32) cfgsize << RINGSIZE_POS)
	    & CREATE_MASK(RINGSIZE_POS, RINGSIZE_LEN);
}

static void xgene_enet_ring_set_type(u32 *ring_cfg, u8 is_bufpool)
{
	u8 val = is_bufpool ? RING_BUFPOOL : RING_REGULAR;

	ring_cfg[4] |= ((u32) val << RINGTYPE_POS)
	    & CREATE_MASK(RINGTYPE_POS, RINGTYPE_LEN);

	if (is_bufpool) {
		ring_cfg[3] |= ((u32) BUFPOOL_MODE << RINGMODE_POS)
		    & CREATE_MASK(RINGMODE_POS, RINGMODE_LEN);
	}
}

static void xgene_enet_ring_set_recombbuf(u32 *ring_cfg)
{
	ring_cfg[3] |= ((u32) 1 << RECOMBBUF_POS)
	    & CREATE_MASK(RECOMBBUF_POS, RECOMBBUF_LEN);
	ring_cfg[3] |= ((u32) 0xf << RECOMTIMEOUTL_POS)
	    & CREATE_MASK(RECOMTIMEOUTL_POS, RECOMTIMEOUTL_LEN);
	ring_cfg[4] |= (u32) 0x7
	    & CREATE_MASK(RECOMTIMEOUTH_POS, RECOMTIMEOUTH_LEN);
}

static void xgene_enet_ring_wr32(struct xgene_enet_desc_ring *ring,
					u32 offset, u32 data)
{
	struct xgene_enet_pdata *pdata = netdev_priv(ring->ndev);

	iowrite32(data, pdata->ring_csr_addr + offset);
}

static void xgene_enet_ring_rd32(struct xgene_enet_desc_ring *ring,
					u32 offset, u32 *data)
{
	struct xgene_enet_pdata *pdata = netdev_priv(ring->ndev);

	*data = ioread32(pdata->ring_csr_addr + offset);
}

static void xgene_enet_write_ring_state(struct xgene_enet_desc_ring *ring)
{
	int i;

	xgene_enet_ring_wr32(ring, CSR_RING_CONFIG, ring->num);
	for (i = 0; i < NUM_RING_CONFIG; i++) {
		xgene_enet_ring_wr32(ring, CSR_RING_WR_BASE + (i * 4),
				     ring->state[i]);
	}
}

static void xgene_enet_clr_ring_state(struct xgene_enet_desc_ring *ring)
{
	struct xgene_enet_desc_ring clr_ring;

	memset(clr_ring.state, 0, sizeof(u32) * NUM_RING_CONFIG);
	clr_ring.num = ring->num;
	clr_ring.ndev = ring->ndev;

	xgene_enet_write_ring_state(&clr_ring);
}

static void xgene_enet_set_ring_state(struct xgene_enet_desc_ring *ring)
{
	xgene_enet_ring_set_type(ring->state, IS_FP(ring->id));

	if (RING_OWNER(ring) == RING_OWNER_ETH0)
		xgene_enet_ring_set_recombbuf(ring->state);

	xgene_enet_ring_init(ring->state, ring->dma, ring->cfgsize);
	xgene_enet_write_ring_state(ring);
}

static void xgene_enet_set_ring_id(struct xgene_enet_desc_ring *ring)
{
	u32 ring_id_val;
	u32 ring_id_buf;
	u8 is_bufpool = IS_FP(ring->id);

	ring_id_val = ring->id & GENMASK(9, 0);
	ring_id_val |= (1 << 31) & GENMASK(31, 31);

	ring_id_buf = (ring->num << 9) & GENMASK(18, 9);
	ring_id_buf |= ((u32) is_bufpool << 20) & GENMASK(20, 20);
	ring_id_buf |= (1U << 21) & GENMASK(21, 21);

	xgene_enet_ring_wr32(ring, CSR_RING_ID, ring_id_val);
	xgene_enet_ring_wr32(ring, CSR_RING_ID_BUF, ring_id_buf);
}

static void xgene_enet_clr_desc_ring_id(struct xgene_enet_desc_ring *ring)
{
	u32 ring_id = ring->id | OVERWRITE;

	xgene_enet_ring_wr32(ring, CSR_RING_ID, ring_id);
	xgene_enet_ring_wr32(ring, CSR_RING_ID_BUF, 0);
}

struct xgene_enet_desc_ring *xgene_enet_setup_ring(
					struct xgene_enet_desc_ring *ring)
{
	u32 size = ring->size;
	u32 i, data;

	xgene_enet_clr_ring_state(ring);
	xgene_enet_set_ring_state(ring);
	xgene_enet_set_ring_id(ring);

	ring->slots = IS_FP(ring->id) ? size / 16 : size / 32;

	if (IS_FP(ring->id) || RING_OWNER(ring) != RING_OWNER_CPU)
		goto out;

	for (i = 0; i < ring->slots; i++) {
		u64 *desc = (u64 *)&ring->desc[i];
		desc[EMPTY_SLOT_INDEX] = EMPTY_SLOT;
	}

	xgene_enet_ring_rd32(ring, CSR_RING_NE_INT_MODE, &data);
	data |= (u32) (1 << (31 - RING_BUFNUM(ring)));
	xgene_enet_ring_wr32(ring, CSR_RING_NE_INT_MODE, data);

out:
	return ring;
}

void xgene_enet_clear_ring(struct xgene_enet_desc_ring *ring)
{
	u32 data;

	if (IS_FP(ring->id) || RING_OWNER(ring) != RING_OWNER_CPU)
		goto out;

	xgene_enet_ring_rd32(ring, CSR_RING_NE_INT_MODE, &data);
	data &= ~(u32) (1 << (31 - RING_BUFNUM(ring)));
	xgene_enet_ring_wr32(ring, CSR_RING_NE_INT_MODE, data);

out:
	xgene_enet_clr_desc_ring_id(ring);
	xgene_enet_clr_ring_state(ring);
}

static void xgene_enet_wr_csr(struct xgene_enet_pdata *pdata,
			      u32 offset, u32 val)
{
	void *addr = pdata->eth_csr_addr + offset;

	iowrite32(val, addr);
}

static void xgene_enet_wr_ring_if(struct xgene_enet_pdata *pdata,
				  u32 offset, u32 val)
{
	void *addr = pdata->eth_ring_if_addr + offset;

	iowrite32(val, addr);
}

static void xgene_enet_wr_diag_csr(struct xgene_enet_pdata *pdata,
				   u32 offset, u32 val)
{
	void *addr = pdata->eth_diag_csr_addr + offset;

	iowrite32(val, addr);
}

static void xgene_enet_wr_mcx_csr(struct xgene_enet_pdata *pdata,
				  u32 offset, u32 val)
{
	void *addr = pdata->mcx_mac_csr_addr + offset;

	iowrite32(val, addr);
}

static u32 xgene_enet_wr_indirect(void *addr, void *wr, void *cmd,
				  void *cmd_done, u32 wr_addr, u32 wr_data)
{
	u32 cmd_done_val;

	iowrite32(wr_addr, addr);
	iowrite32(wr_data, wr);
	iowrite32(XGENE_ENET_WR_CMD, cmd);
	udelay(5);		/* wait 5 us for completion */
	cmd_done_val = ioread32(cmd_done);
	iowrite32(0, cmd);
	return cmd_done_val;
}

static void xgene_enet_wr_mcx_mac(struct xgene_enet_pdata *pdata,
				  u32 wr_addr, u32 wr_data)
{
	void *addr, *wr, *cmd, *cmd_done;
	int ret;

	addr = pdata->mcx_mac_addr + MAC_ADDR_REG_OFFSET;
	wr = pdata->mcx_mac_addr + MAC_WRITE_REG_OFFSET;
	cmd = pdata->mcx_mac_addr + MAC_COMMAND_REG_OFFSET;
	cmd_done = pdata->mcx_mac_addr + MAC_COMMAND_DONE_REG_OFFSET;

	ret = xgene_enet_wr_indirect(addr, wr, cmd, cmd_done, wr_addr, wr_data);
	if (!ret)
		netdev_err(pdata->ndev, "MCX mac write failed, addr: %04x",
			   wr_addr);
}

static void xgene_enet_rd_csr(struct xgene_enet_pdata *pdata,
			      u32 offset, u32 *val)
{
	void *addr = pdata->eth_csr_addr + offset;

	*val = ioread32(addr);
}

static void xgene_enet_rd_diag_csr(struct xgene_enet_pdata *pdata,
				   u32 offset, u32 *val)
{
	void *addr = pdata->eth_diag_csr_addr + offset;

	*val = ioread32(addr);
}

static void xgene_enet_rd_mcx_csr(struct xgene_enet_pdata *pdata,
				  u32 offset, u32 *val)
{
	void *addr = pdata->mcx_mac_csr_addr + offset;

	*val = ioread32(addr);
}

static u32 xgene_enet_rd_indirect(void *addr, void *rd, void *cmd,
				  void *cmd_done, u32 rd_addr, u32 *rd_data)
{
	u32 cmd_done_val;

	iowrite32(rd_addr, addr);
	iowrite32(XGENE_ENET_RD_CMD, cmd);
	udelay(5);		/* wait 5 us for completion */
	cmd_done_val = ioread32(cmd_done);
	*rd_data = ioread32(rd);
	iowrite32(0, cmd);

	return cmd_done_val;
}

static void xgene_enet_rd_mcx_mac(struct xgene_enet_pdata *pdata,
				  u32 rd_addr, u32 *rd_data)
{
	void *addr, *rd, *cmd, *cmd_done;
	int ret;

	addr = pdata->mcx_mac_addr + MAC_ADDR_REG_OFFSET;
	rd = pdata->mcx_mac_addr + MAC_READ_REG_OFFSET;
	cmd = pdata->mcx_mac_addr + MAC_COMMAND_REG_OFFSET;
	cmd_done = pdata->mcx_mac_addr + MAC_COMMAND_DONE_REG_OFFSET;

	ret = xgene_enet_rd_indirect(addr, rd, cmd, cmd_done, rd_addr, rd_data);
	if (!ret)
		netdev_err(pdata->ndev, "MCX mac read failed, addr: %04x",
			   rd_addr);
}

static void xgene_enet_rd_mcx_stats(struct xgene_enet_pdata *pdata,
				    u32 rd_addr, u32 *rd_data)
{
	void *addr, *rd, *cmd, *cmd_done;
	int ret;

	addr = pdata->mcx_stats_addr + STAT_ADDR_REG_OFFSET;
	rd = pdata->mcx_stats_addr + STAT_READ_REG_OFFSET;
	cmd = pdata->mcx_stats_addr + STAT_COMMAND_REG_OFFSET;
	cmd_done = pdata->mcx_stats_addr + STAT_COMMAND_DONE_REG_OFFSET;

	ret = xgene_enet_rd_indirect(addr, rd, cmd, cmd_done, rd_addr, rd_data);
	if (!ret)
		netdev_err(pdata->ndev, "MCX stats read failed, addr: %04x",
			   rd_addr);
}

void xgene_genericmiiphy_write(struct xgene_enet_pdata *pdata, int phy_id,
			       u32 reg, u16 data)
{
	u32 addr, wr_data, done;

	addr = PHY_ADDR_WR(phy_id) | REG_ADDR_WR(reg);
	xgene_enet_wr_mcx_mac(pdata, MII_MGMT_ADDRESS_ADDR, addr);

	wr_data = PHY_CONTROL_WR(data);
	xgene_enet_wr_mcx_mac(pdata, MII_MGMT_CONTROL_ADDR, wr_data);

	usleep_range(20, 30);		/* wait 20 us for completion */
	xgene_enet_rd_mcx_mac(pdata, MII_MGMT_INDICATORS_ADDR, &done);
	if (done & BUSY_MASK)
		netdev_err(pdata->ndev, "MII_MGMT write failed\n");
}

void xgene_genericmiiphy_read(struct xgene_enet_pdata *pdata, u8 phy_id,
			      u32 reg, u32 *data)
{
	u32 addr, done;

	addr = PHY_ADDR_WR(phy_id) | REG_ADDR_WR(reg);
	xgene_enet_wr_mcx_mac(pdata, MII_MGMT_ADDRESS_ADDR, addr);
	xgene_enet_wr_mcx_mac(pdata, MII_MGMT_COMMAND_ADDR, READ_CYCLE_MASK);

	usleep_range(20, 30);		/* wait 20 us for completion */
	xgene_enet_rd_mcx_mac(pdata, MII_MGMT_INDICATORS_ADDR, &done);
	if (done & BUSY_MASK)
		netdev_err(pdata->ndev, "MII_MGMT read failed\n");

	xgene_enet_rd_mcx_mac(pdata, MII_MGMT_STATUS_ADDR, data);
	xgene_enet_wr_mcx_mac(pdata, MII_MGMT_COMMAND_ADDR, 0);
}

void xgene_gmac_set_mac_addr(struct xgene_enet_pdata *pdata,
			     unsigned char *dev_addr)
{
	u32 addr0, addr1;

	addr0 = (dev_addr[3] << 24) | (dev_addr[2] << 16) |
		(dev_addr[1] << 8) | dev_addr[0];
	addr1 = (dev_addr[5] << 24) | (dev_addr[4] << 16);
	addr1 |= pdata->phy_addr & 0xFFFF;

	xgene_enet_wr_mcx_mac(pdata, STATION_ADDR0_ADDR, addr0);
	xgene_enet_wr_mcx_mac(pdata, STATION_ADDR1_ADDR, addr1);
}

static int xgene_enet_ecc_init(struct xgene_enet_pdata *pdata)
{
	struct net_device *ndev = pdata->ndev;
	u32 data;

	xgene_enet_wr_diag_csr(pdata, ENET_CFG_MEM_RAM_SHUTDOWN_ADDR, 0x0);
	usleep_range(1000, 1100);		/* wait 1 ms for completion */
	xgene_enet_rd_diag_csr(pdata, ENET_BLOCK_MEM_RDY_ADDR, &data);
	if (data != 0xffffffff) {
		netdev_err(ndev, "Failed to release memory from shutdown\n");
		return -ENODEV;
	}

	return 0;
}

static void xgene_gmac_phy_enable_scan_cycle(struct xgene_enet_pdata *pdata,
					     int enable)
{
	u32 val;

	xgene_enet_rd_mcx_mac(pdata, MII_MGMT_COMMAND_ADDR, &val);
	val = SCAN_CYCLE_MASK_SET(val, enable);
	xgene_enet_wr_mcx_mac(pdata, MII_MGMT_COMMAND_ADDR, val);

	/* Program phy address start scan from 0 and register at address 0x1 */
	xgene_enet_rd_mcx_mac(pdata, MII_MGMT_ADDRESS_ADDR, &val);
	val = PHY_ADDR_SET(val, 0);
	val = REG_ADDR_SET(val, 1);
	xgene_enet_wr_mcx_mac(pdata, MII_MGMT_ADDRESS_ADDR, val);
}

void xgene_gmac_reset(struct xgene_enet_pdata *pdata)
{
	u32 value;

	xgene_enet_rd_mcx_mac(pdata, MAC_CONFIG_1_ADDR, &value);
	if (!(value & SOFT_RESET1_MASK))
		return;

	value = RESET_TX_FUN1_WR(1)
	    | RESET_RX_FUN1_WR(1)
	    | RESET_TX_MC1_WR(1)
	    | RESET_RX_MC1_WR(1)
	    | SIM_RESET1_WR(1)
	    | SOFT_RESET1_WR(1);

	xgene_enet_wr_mcx_mac(pdata, MAC_CONFIG_1_ADDR, value);
	xgene_enet_rd_mcx_mac(pdata, MAC_CONFIG_1_ADDR, &value);
	xgene_enet_wr_mcx_mac(pdata, MAC_CONFIG_1_ADDR, 0);
}

void xgene_gmac_init(struct xgene_enet_pdata *pdata, unsigned char *dev_addr,
		     int speed)
{
	u32 value;
	u32 mc2;
	u32 intf_ctl = ENET_GHD_MODE_WR(1);
	u32 rgmii = 0;
	u32 icm0 = 0x0008503f;
	u32 icm2 = 0x0001000f;

	xgene_gmac_reset(pdata);

	xgene_enet_rd_mcx_mac(pdata, ICM_CONFIG2_REG_0_ADDR, &icm2);
	xgene_enet_rd_mcx_mac(pdata, MAC_CONFIG_2_ADDR, &mc2);
	mc2 |= FULL_DUPLEX2_WR(1);

	switch (speed) {
	case SPEED_10:
		intf_ctl = ENET_LHD_MODE_WR(0) | ENET_GHD_MODE_WR(0);
		mc2 |= ENET_INTERFACE_MODE2_WR(1);
		icm0 = CFG_MACMODE_SET(icm0, 0);
		icm2 = CFG_WAITASYNCRD_SET(icm2, 500);
		break;
	case SPEED_100:
		intf_ctl = ENET_LHD_MODE_WR(1);
		mc2 |= ENET_INTERFACE_MODE2_WR(1);
		icm0 = CFG_MACMODE_SET(icm0, 1);
		icm2 = CFG_WAITASYNCRD_SET(icm2, 80);
		break;
	default:
		mc2 |= ENET_INTERFACE_MODE2_WR(2);
		rgmii = CFG_SPEED_1250 | CFG_TXCLK_MUXSEL0_WR(4);
		break;
	}

	xgene_enet_wr_mcx_mac(pdata, MAC_CONFIG_2_ADDR, mc2);
	xgene_enet_wr_mcx_mac(pdata, INTERFACE_CONTROL_ADDR, intf_ctl);

	value = MAX_FRAME_LEN_WR(XGENE_ENET_MAX_MTU);
	xgene_enet_wr_mcx_mac(pdata, MAX_FRAME_LEN_ADDR, value);

	/* Program the station MAC address */
	xgene_gmac_set_mac_addr(pdata, dev_addr);

	/* Adjust MDC clock frequency */
	xgene_enet_rd_mcx_mac(pdata, MII_MGMT_CONFIG_ADDR, &value);
	value = MGMT_CLOCK_SEL_SET(value, 7);
	xgene_enet_wr_mcx_mac(pdata, MII_MGMT_CONFIG_ADDR, value);

	/* Enable drop if FP not available */
	xgene_enet_rd_csr(pdata, RSIF_CONFIG_REG_ADDR, &value);
	value |= CFG_RSIF_FPBUFF_TIMEOUT_EN;
	xgene_enet_wr_csr(pdata, RSIF_CONFIG_REG_ADDR, value);

	/* Rtype should be copied from FP */
	xgene_enet_wr_csr(pdata, RSIF_RAM_DBG_REG0_ADDR, 0);

	/* Initialize RGMII PHY */
	if (pdata->phy_mode == PHY_INTERFACE_MODE_RGMII)
		xgene_enet_wr_csr(pdata, RGMII_REG_0_ADDR, rgmii);

	xgene_enet_wr_mcx_csr(pdata, ICM_CONFIG0_REG_0_ADDR, icm0);
	xgene_enet_wr_mcx_csr(pdata, ICM_CONFIG2_REG_0_ADDR, icm2);

	/* Rx-Tx traffic resume */
	xgene_enet_wr_csr(pdata, CFG_LINK_AGGR_RESUME_0_ADDR, TX_PORT0);

	if (speed == SPEED_1000) {
		xgene_enet_rd_csr(pdata, DEBUG_REG_ADDR, &value);
		value |= CFG_BYPASS_UNISEC_TX
		    | CFG_BYPASS_UNISEC_RX;
		xgene_enet_wr_csr(pdata, DEBUG_REG_ADDR, value);
	}

	xgene_enet_rd_mcx_csr(pdata, RX_DV_GATE_REG_0_ADDR, &value);
	value = TX_DV_GATE_EN0_SET(value, 0);
	value = RX_DV_GATE_EN0_SET(value, 0);
	value = RESUME_RX0_SET(value, 1);
	xgene_enet_wr_mcx_csr(pdata, RX_DV_GATE_REG_0_ADDR, value);

	xgene_enet_wr_csr(pdata, CFG_BYPASS_ADDR, RESUME_TX);
}

/* Start Statistics related functions */
static void xgene_gmac_get_rx_stats(struct xgene_enet_pdata *pdata,
				    struct xgene_enet_rx_stats *rx_stat)
{
	xgene_enet_rd_mcx_stats(pdata, RBYT_ADDR, &rx_stat->rx_byte_count);
	xgene_enet_rd_mcx_stats(pdata, RPKT_ADDR, &rx_stat->rx_packet_count);
	xgene_enet_rd_mcx_stats(pdata, RDRP_ADDR, &rx_stat->rx_drop_pkt_count);
	xgene_enet_rd_mcx_stats(pdata, RFCS_ADDR, &rx_stat->rx_fcs_err_count);
	xgene_enet_rd_mcx_stats(pdata, RFLR_ADDR,
				&rx_stat->rx_frm_len_err_pkt_count);
	xgene_enet_rd_mcx_stats(pdata, RALN_ADDR,
				&rx_stat->rx_alignment_err_pkt_count);
	xgene_enet_rd_mcx_stats(pdata, ROVR_ADDR,
				&rx_stat->rx_oversize_pkt_count);
	xgene_enet_rd_mcx_stats(pdata, RUND_ADDR,
				&rx_stat->rx_undersize_pkt_count);

	rx_stat->rx_byte_count &= RX_BYTE_CNTR_MASK;
	rx_stat->rx_packet_count &= RX_PKT_CNTR_MASK;
	rx_stat->rx_drop_pkt_count &= RX_DROPPED_PKT_CNTR_MASK;
	rx_stat->rx_fcs_err_count &= RX_FCS_ERROR_CNTR_MASK;
	rx_stat->rx_frm_len_err_pkt_count &= RX_LEN_ERR_CNTR_MASK;
	rx_stat->rx_alignment_err_pkt_count &= RX_ALIGN_ERR_CNTR_MASK;
	rx_stat->rx_oversize_pkt_count &= RX_OVRSIZE_PKT_CNTR_MASK;
	rx_stat->rx_undersize_pkt_count &= RX_UNDRSIZE_PKT_CNTR_MASK;
}

static void xgene_gmac_get_tx_stats(struct xgene_enet_pdata *pdata,
				    struct xgene_enet_tx_stats *tx_stats)
{
	xgene_enet_rd_mcx_stats(pdata, TBYT_ADDR, &tx_stats->tx_byte_count);
	xgene_enet_rd_mcx_stats(pdata, TPKT_ADDR, &tx_stats->tx_pkt_count);
	xgene_enet_rd_mcx_stats(pdata, TDRP_ADDR, &tx_stats->tx_drop_frm_count);
	xgene_enet_rd_mcx_stats(pdata, TFCS_ADDR,
				&tx_stats->tx_fcs_err_frm_count);
	xgene_enet_rd_mcx_stats(pdata, TUND_ADDR,
				&tx_stats->tx_undersize_frm_count);

	tx_stats->tx_byte_count &= TX_BYTE_CNTR_MASK;
	tx_stats->tx_pkt_count &= TX_PKT_CNTR_MASK;
	tx_stats->tx_drop_frm_count &= TX_DROP_FRAME_CNTR_MASK;
	tx_stats->tx_fcs_err_frm_count &= TX_FCS_ERROR_CNTR_MASK;
	tx_stats->tx_undersize_frm_count &= TX_UNDSIZE_FRAME_CNTR_MASK;
}

void xgene_gmac_get_detailed_stats(struct xgene_enet_pdata *pdata,
				   struct xgene_enet_detailed_stats *stats)
{
	xgene_gmac_get_rx_stats(pdata, &stats->rx_stats);
	xgene_gmac_get_tx_stats(pdata, &stats->tx_stats);
}

static void xgene_enet_config_ring_if_assoc(struct xgene_enet_pdata *pdata)
{
	u32 val = 0xffffffff;

	xgene_enet_wr_ring_if(pdata, ENET_CFGSSQMIWQASSOC_ADDR, val);
	xgene_enet_wr_ring_if(pdata, ENET_CFGSSQMIFPQASSOC_ADDR, val);
	xgene_enet_wr_ring_if(pdata, ENET_CFGSSQMIQMLITEWQASSOC_ADDR, val);
	xgene_enet_wr_ring_if(pdata, ENET_CFGSSQMIQMLITEFPQASSOC_ADDR, val);
}

void xgene_enet_cle_bypass_mode_cfg(struct xgene_enet_pdata *pdata,
				    u32 dst_ring_num, u32 fpsel, u32 nxtfpsel)
{
	u32 cb;

	xgene_enet_rd_csr(pdata, CLE_BYPASS_REG0_0_ADDR, &cb);
	cb |= CFG_CLE_BYPASS_EN0;
	cb = CFG_CLE_IP_PROTOCOL0_SET(cb, 3);
	xgene_enet_wr_csr(pdata, CLE_BYPASS_REG0_0_ADDR, cb);

	xgene_enet_rd_csr(pdata, CLE_BYPASS_REG1_0_ADDR, &cb);
	cb = CFG_CLE_DSTQID0_SET(cb, dst_ring_num);
	cb = CFG_CLE_FPSEL0_SET(cb, fpsel);
	cb = CFG_CLE_NXTFPSEL0_SET(cb, nxtfpsel);
	xgene_enet_wr_csr(pdata, CLE_BYPASS_REG1_0_ADDR, cb);
}

void xgene_gmac_rx_enable(struct xgene_enet_pdata *pdata)
{
	u32 data;

	xgene_enet_rd_mcx_mac(pdata, MAC_CONFIG_1_ADDR, &data);
	xgene_enet_wr_mcx_mac(pdata, MAC_CONFIG_1_ADDR, data | GENMASK(2, 2));
}

void xgene_gmac_tx_enable(struct xgene_enet_pdata *pdata)
{
	u32 data;

	xgene_enet_rd_mcx_mac(pdata, MAC_CONFIG_1_ADDR, &data);
	xgene_enet_wr_mcx_mac(pdata, MAC_CONFIG_1_ADDR, data | GENMASK(0, 0));
}

void xgene_gmac_rx_disable(struct xgene_enet_pdata *pdata)
{
	u32 data;

	xgene_enet_rd_mcx_mac(pdata, MAC_CONFIG_1_ADDR, &data);
	xgene_enet_wr_mcx_mac(pdata, MAC_CONFIG_1_ADDR, data & ~GENMASK(2, 2));
}

void xgene_gmac_tx_disable(struct xgene_enet_pdata *pdata)
{
	u32 data;

	xgene_enet_rd_mcx_mac(pdata, MAC_CONFIG_1_ADDR, &data);
	xgene_enet_wr_mcx_mac(pdata, MAC_CONFIG_1_ADDR, data & ~GENMASK(0, 0));
}

void xgene_enet_reset(struct xgene_enet_pdata *pdata)
{
	u32 val;

	clk_prepare_enable(pdata->clk);
	clk_disable_unprepare(pdata->clk);
	clk_prepare_enable(pdata->clk);
	xgene_enet_ecc_init(pdata);
	xgene_enet_config_ring_if_assoc(pdata);

	/* Enable auto-incr for scanning */
	xgene_enet_rd_mcx_mac(pdata, MII_MGMT_CONFIG_ADDR, &val);
	val |= SCAN_AUTO_INCR_MASK;
	val = MGMT_CLOCK_SEL_SET(val, 1);
	xgene_enet_wr_mcx_mac(pdata, MII_MGMT_CONFIG_ADDR, val);
	xgene_gmac_phy_enable_scan_cycle(pdata, 1);
}

void xgene_gport_shutdown(struct xgene_enet_pdata *pdata)
{
	clk_disable_unprepare(pdata->clk);
}

static int xgene_enet_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
	struct xgene_enet_pdata *pdata = bus->priv;
	u32 val;

	xgene_genericmiiphy_read(pdata, mii_id, regnum, &val);
	netdev_dbg(pdata->ndev, "mdio_rd: bus=%d reg=%d val=%x\n",
		   mii_id, regnum, val);

	return val;
}

static int xgene_enet_mdio_write(struct mii_bus *bus, int mii_id, int regnum,
				 u16 val)
{
	struct xgene_enet_pdata *pdata = bus->priv;

	netdev_dbg(pdata->ndev, "mdio_wr: bus=%d reg=%d val=%x\n",
		   mii_id, regnum, val);
	xgene_genericmiiphy_write(pdata, mii_id, regnum, val);

	return 0;
}

static void xgene_enet_adjust_link(struct net_device *ndev)
{
	struct xgene_enet_pdata *pdata = netdev_priv(ndev);
	struct phy_device *phydev = pdata->phy_dev;
	int status_change = 0;

	if (phydev->link && pdata->phy_speed != phydev->speed) {
		xgene_gmac_init(pdata, ndev->dev_addr, phydev->speed);
		pdata->phy_speed = phydev->speed;
		status_change = 1;
	}

	if (pdata->phy_link != phydev->link) {
		if (!phydev->link)
			pdata->phy_speed = 0;
		pdata->phy_link = phydev->link;
		status_change = 1;
	}

	if (status_change) {
		if (phydev->link) {
			xgene_gmac_rx_enable(pdata);
			xgene_gmac_tx_enable(pdata);
		} else {
			xgene_gmac_rx_disable(pdata);
			xgene_gmac_tx_disable(pdata);
		}
		phy_print_status(phydev);
	}
}

static int xgene_enet_phy_connect(struct net_device *ndev)
{
	struct xgene_enet_pdata *pdata = netdev_priv(ndev);
	struct device_node *phy_np;
	struct phy_device *phy_dev;
	int ret = 0;

	struct device *dev = &pdata->pdev->dev;

	phy_np = of_parse_phandle(dev->of_node, "phy-handle", 0);

	if (!phy_np) {
		netdev_dbg(ndev, "No phy-handle found\n");
		ret = -ENODEV;
	}

	phy_dev = of_phy_connect(ndev, phy_np, &xgene_enet_adjust_link,
				 0, pdata->phy_mode);
	if (!phy_dev) {
		netdev_err(ndev, "Could not connect to PHY\n");
		ret = -ENODEV;
		goto out;
	}

out:
	pdata->phy_link = 0;
	pdata->phy_speed = 0;
	pdata->phy_dev = phy_dev;

	return ret;
}

int xgene_enet_mdio_config(struct xgene_enet_pdata *pdata)
{
	struct net_device *ndev = pdata->ndev;
	struct device *dev = &pdata->pdev->dev;
	struct device_node *child_np = NULL;
	struct device_node *mdio_np = NULL;
	struct mii_bus *mdio_bus = NULL;
	int ret;

	for_each_child_of_node(dev->of_node, child_np) {
		if (of_device_is_compatible(child_np, "apm,xgene-mdio")) {
			mdio_np = child_np;
			break;
		}
	}

	if (!mdio_np) {
		netdev_dbg(ndev, "No mdio node in the dts\n");
		ret = -1;
		goto err;
	}

	mdio_bus = mdiobus_alloc();
	if (!mdio_bus) {
		netdev_err(ndev, "Could not allocate MDIO bus\n");
		ret = -ENOMEM;
		goto err;
	}

	mdio_bus->name = "APM X-Gene MDIO bus";
	mdio_bus->read = xgene_enet_mdio_read;
	mdio_bus->write = xgene_enet_mdio_write;
	snprintf(mdio_bus->id, MII_BUS_ID_SIZE, "%s", "xgene-enet-mii");

	mdio_bus->irq = devm_kcalloc(dev, PHY_MAX_ADDR, sizeof(int),
				     GFP_KERNEL);
	if (mdio_bus->irq == NULL) {
		ret = -ENOMEM;
		goto err;
	}

	mdio_bus->priv = pdata;
	mdio_bus->parent = &ndev->dev;

	ret = of_mdiobus_register(mdio_bus, mdio_np);
	if (ret) {
		netdev_err(ndev, "Failed to register MDIO bus\n");
		goto err;
	}
	pdata->mdio_bus = mdio_bus;
	ret = xgene_enet_phy_connect(ndev);

	return ret;
err:
	if (mdio_bus) {
		if (mdio_bus->irq)
			devm_kfree(dev, mdio_bus->irq);
		mdiobus_free(mdio_bus);
	}
	return ret;
}

int xgene_enet_mdio_remove(struct xgene_enet_pdata *pdata)
{
	struct mii_bus *mdio_bus;

	mdio_bus = pdata->mdio_bus;
	mdiobus_unregister(mdio_bus);
	mdiobus_free(mdio_bus);
	pdata->mdio_bus = NULL;

	return 0;
}
