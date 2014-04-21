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

#ifndef __XGENE_ENET_MAIN_H__
#define __XGENE_ENET_MAIN_H__

#include <linux/clk.h>
#include <linux/of_platform.h>
#include <linux/of_net.h>
#include <linux/of_mdio.h>
#include <linux/module.h>
#include <net/ip.h>
#include <linux/if_vlan.h>
#include <linux/phy.h>
#include "xgene_enet_hw.h"

#define XGENE_ENET_MAX_MTU	1536
#define SKB_BUFFER_SIZE		(XGENE_ENET_MAX_MTU - NET_IP_ALIGN)

#define XGENE_NUM_PKT_BUF	64
#define XGENE_ENET_FP_NBUF	32

#define RM3			3

#define TX_RING_CFGSIZE		RING_CFGSIZE_2KB
#define RX_RING_CFGSIZE		RING_CFGSIZE_16KB
#define BUFPOOL_CFGSIZE		RING_CFGSIZE_2KB

/* software context of a descriptor ring */
struct xgene_enet_desc_ring {
	struct net_device *ndev;
	u16 id;
	u16 num;
	u16 head;
	u16 tail;
	u16 slots;
	u16 irq;
	u32 size;
	u32 state[NUM_RING_CONFIG];
	void __iomem *cmd_base;
	void __iomem *cmd;
	dma_addr_t dma;
	u16 dst_ring_num;
	u8 nbufpool;
	struct sk_buff *(*rx_skb);
	struct sk_buff *(*cp_skb);
	enum xgene_enet_ring_cfgsize cfgsize;
	struct xgene_enet_desc_ring *cp_ring;
	struct xgene_enet_desc_ring *buf_pool;
	struct napi_struct napi;
	union {
		void *desc_addr;
		struct xgene_enet_desc *desc;
		struct xgene_enet_desc16 *desc16;
	};
};

struct xgene_enet_rx_stats {
	u32 rx_byte_count;
	u32 rx_packet_count;
	u32 rx_fcs_err_count;
	u32 rx_alignment_err_pkt_count;
	u32 rx_frm_len_err_pkt_count;
	u32 rx_undersize_pkt_count;
	u32 rx_oversize_pkt_count;
	u32 rx_drop_pkt_count;
};

struct xgene_enet_tx_stats {
	u32 tx_byte_count;
	u32 tx_pkt_count;
	u32 tx_drop_frm_count;
	u32 tx_fcs_err_frm_count;
	u32 tx_undersize_frm_count;
};

struct xgene_enet_detailed_stats {
	struct xgene_enet_rx_stats rx_stats;
	struct xgene_enet_tx_stats tx_stats;
};

/* ethernet private data */
struct xgene_enet_pdata {
	struct net_device *ndev;
	struct mii_bus *mdio_bus;
	struct phy_device *phy_dev;
	int phy_link;
	int phy_speed;
	struct clk *clk;
	struct platform_device *pdev;
	struct xgene_enet_desc_ring *tx_ring;
	struct xgene_enet_desc_ring *rx_ring;
	struct net_device_stats nstats;
	char *dev_name;
	u32 rx_buff_cnt;
	u32 tx_qcnt_hi;
	u32 cp_qcnt_hi;
	u32 cp_qcnt_low;
	u32 rx_irq;
	void __iomem *eth_csr_addr;
	void __iomem *eth_ring_if_addr;
	void __iomem *eth_diag_csr_addr;
	void __iomem *mcx_mac_addr;
	void __iomem *mcx_stats_addr;
	void __iomem *mcx_mac_csr_addr;
	void __iomem *base_addr;
	void __iomem *ring_csr_addr;
	void __iomem *ring_cmd_addr;
	u32 phy_addr;
	int phy_mode;
	u32 speed;
	u16 rm;
};

int xgene_enet_mdio_config(struct xgene_enet_pdata *pdata);
int xgene_enet_mdio_remove(struct xgene_enet_pdata *pdata);

#endif /* __XGENE_ENET_MAIN_H__ */
