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

#include "xgene_enet_main.h"
#include "xgene_enet_hw.h"

static void xgene_enet_init_bufpool(struct xgene_enet_desc_ring *buf_pool)
{
	struct xgene_enet_desc16 *desc;
	int i;

	for (i = 0; i < buf_pool->slots; i++) {
		desc = &buf_pool->desc16[i];

		set_desc(desc, USERINFO, i);
		set_desc(desc, FPQNUM, buf_pool->dst_ring_num);
		set_desc(desc, STASH, 1);

		/* Hardware expects descriptor in little endian format */
		xgene_enet_cpu_to_le64(desc, 4);
	}
}

static struct device *ndev_to_dev(struct net_device *ndev)
{
	struct xgene_enet_pdata *pdata = netdev_priv(ndev);

	return &pdata->pdev->dev;
}

static int xgene_enet_refill_bufpool(struct xgene_enet_desc_ring *buf_pool,
				     u32 nbuf)
{
	struct sk_buff *skb;
	struct xgene_enet_desc16 *desc;
	struct net_device *ndev;
	struct device *dev;
	dma_addr_t dma_addr;
	u32 tail = buf_pool->tail;
	u32 slots = buf_pool->slots - 1;
	int i, ret = 0;
	u16 bufdatalen = BUF_LEN_CODE_2K | (SKB_BUFFER_SIZE & GENMASK(11, 0));

	ndev = buf_pool->ndev;
	dev = ndev_to_dev(buf_pool->ndev);

	for (i = 0; i < nbuf; i++) {
		desc = &buf_pool->desc16[tail];

		skb = netdev_alloc_skb_ip_align(ndev, XGENE_ENET_MAX_MTU);
		if (unlikely(!skb)) {
			netdev_err(ndev, "Could not allocate skb");
			ret = -ENOMEM;
			goto out;
		}
		buf_pool->rx_skb[tail] = skb;

		dma_addr = dma_map_single(dev, skb->data, skb->len,
					  DMA_TO_DEVICE);
		if (dma_mapping_error(dev, dma_addr)) {
			netdev_err(ndev, "DMA mapping error\n");
			dev_kfree_skb_any(skb);
			ret = -EINVAL;
			goto out;
		}
		set_desc(desc, DATAADDR, dma_addr);
		set_desc(desc, BUFDATALEN, bufdatalen);
		set_desc(desc, COHERENT, 1);

		xgene_enet_desc16_to_le64(desc);
		tail = (tail + 1) & slots;
	}

	iowrite32(nbuf, buf_pool->cmd);
	buf_pool->tail = tail;

out:
	return ret;
}

static u16 xgene_enet_dst_ring_num(struct xgene_enet_desc_ring *ring)
{
	struct xgene_enet_pdata *pdata = netdev_priv(ring->ndev);

	return ((u16)pdata->rm << 10) | ring->num;
}

static u8 xgene_enet_hdr_len(const void *data)
{
	const struct ethhdr *eth = data;

	return (eth->h_proto == htons(ETH_P_8021Q)) ? VLAN_ETH_HLEN : ETH_HLEN;
}

static u32 xgene_enet_ring_len(struct xgene_enet_desc_ring *ring)
{
	u32 *cmd_base = ring->cmd_base;
	u32 ring_state, num_msgs;

	ring_state = ioread32(&cmd_base[1]);
	num_msgs = ring_state & CREATE_MASK(NUMMSGSINQ_POS, NUMMSGSINQ_LEN);
	return num_msgs >> NUMMSGSINQ_POS;
}

static void xgene_enet_delete_bufpool(struct xgene_enet_desc_ring *buf_pool)
{
	u32 tail = buf_pool->tail;
	u32 slots = buf_pool->slots - 1;
	int len = xgene_enet_ring_len(buf_pool);
	struct xgene_enet_desc16 *desc;
	u32 userinfo;
	int i;

	for (i = 0; i < len; i++) {
		tail = (tail - 1) & slots;
		desc = &buf_pool->desc16[tail];

		/* Hardware stores descriptor in little endian format */
		xgene_enet_le64_to_desc16(desc);
		userinfo = get_desc(desc, USERINFO);
		dev_kfree_skb_any(buf_pool->rx_skb[userinfo]);
	}

	iowrite32(-len, buf_pool->cmd);
	buf_pool->tail = tail;
}

irqreturn_t xgene_enet_rx_irq(const int irq, void *data)
{
	struct xgene_enet_desc_ring *rx_ring = data;

	if (napi_schedule_prep(&rx_ring->napi)) {
		disable_irq_nosync(irq);
		__napi_schedule(&rx_ring->napi);
	}

	return IRQ_HANDLED;
}

static int xgene_enet_tx_completion(struct xgene_enet_desc_ring *cp_ring,
				    struct xgene_enet_desc *desc)
{
	struct sk_buff *skb;
	dma_addr_t pa;
	size_t len;
	struct device *dev;
	u16 skb_index;
	int ret = 0;

	skb_index = get_desc(desc, USERINFO);
	skb = cp_ring->cp_skb[skb_index];

	dev = ndev_to_dev(cp_ring->ndev);
	pa = (dma_addr_t)get_desc(desc, DATAADDR);
	len = get_desc(desc, BUFDATALEN);
	dma_unmap_single(dev, pa, len, DMA_TO_DEVICE);

	if (likely(skb)) {
		dev_kfree_skb_any(skb);
	} else {
		netdev_err(cp_ring->ndev, "completion skb is NULL\n");
		ret = -1;
	}

	return ret;
}

static void xgene_enet_checksum_offload(struct xgene_enet_desc *desc,
					struct sk_buff *skb)
{
	u32 maclen, nr_frags;
	struct iphdr *iph;
	u8 l4hlen = 0;
	u8 l3hlen = 0;
	u8 csum_enable = 0;
	u8 proto = 0;
	struct net_device *ndev = skb->dev;

	if (unlikely(!(ndev->features & NETIF_F_IP_CSUM)))
		goto out;
	if (unlikely(skb->protocol != htons(ETH_P_IP)) &&
	    unlikely(skb->protocol != htons(ETH_P_8021Q)))
		goto out;

	nr_frags = skb_shinfo(skb)->nr_frags;
	maclen = xgene_enet_hdr_len(skb->data);
	iph = ip_hdr(skb);
	l3hlen = ip_hdrlen(skb) >> 2;

	if (unlikely(iph->frag_off & htons(IP_MF | IP_OFFSET)))
		goto out;
	if (likely(iph->protocol == IPPROTO_TCP)) {
		l4hlen = tcp_hdrlen(skb) / 4;
		csum_enable = 1;
		proto = TSO_IPPROTO_TCP;
	} else if (iph->protocol == IPPROTO_UDP) {
		l4hlen = UDP_HDR_SIZE;
		csum_enable = 1;
		proto = TSO_IPPROTO_UDP;
	}

	set_desc(desc, TCPHDR, l4hlen);
	set_desc(desc, IPHDR, l3hlen);
	set_desc(desc, EC, csum_enable);
	set_desc(desc, IS, proto);
out:
	return;
}

static int xgene_enet_setup_tx_desc(struct xgene_enet_desc_ring *tx_ring,
				     struct sk_buff *skb)
{
	struct xgene_enet_desc *desc;
	dma_addr_t dma_addr;
	u8 ethhdr;
	u16 tail = tx_ring->tail;
	struct device *dev = ndev_to_dev(tx_ring->ndev);

	desc = &tx_ring->desc[tail];
	memset(desc, 0, sizeof(struct xgene_enet_desc));

	dma_addr = dma_map_single(dev, skb->data, skb->len, DMA_TO_DEVICE);
	if (dma_mapping_error(dev, dma_addr)) {
		netdev_err(tx_ring->ndev, "DMA mapping error\n");
		return -EINVAL;
	}
	set_desc(desc, DATAADDR, dma_addr);

	set_desc(desc, BUFDATALEN, skb->len);
	set_desc(desc, COHERENT, 1);
	tx_ring->cp_ring->cp_skb[tail] = skb;
	set_desc(desc, USERINFO, tail);
	set_desc(desc, HENQNUM, tx_ring->dst_ring_num);
	set_desc(desc, TYPESEL, 1);
	ethhdr = xgene_enet_hdr_len(skb->data);
	set_desc(desc, ETHHDR, ethhdr);
	set_desc(desc, IC, 1);

	xgene_enet_checksum_offload(desc, skb);
	/* Hardware expects descriptor in little endian format */
	xgene_enet_cpu_to_le64(desc, 4);

	return 0;
}

static netdev_tx_t xgene_enet_start_xmit(struct sk_buff *skb,
					 struct net_device *ndev)
{
	struct xgene_enet_pdata *pdata = netdev_priv(ndev);
	struct xgene_enet_desc_ring *tx_ring = pdata->tx_ring;
	struct xgene_enet_desc_ring *cp_ring = tx_ring->cp_ring;
	u32 tx_level, cq_level;
	u32 pkt_count = 1;

	tx_level = xgene_enet_ring_len(tx_ring);
	cq_level = xgene_enet_ring_len(cp_ring);
	if (tx_level > pdata->tx_qcnt_hi || cq_level > pdata->cp_qcnt_hi) {
		netif_stop_queue(ndev);
		goto out;
	}

	if (xgene_enet_setup_tx_desc(tx_ring, skb))
		goto out;

	skb_tx_timestamp(skb);

	tx_ring->tail = (tx_ring->tail + 1) & (tx_ring->slots - 1);
	iowrite32(pkt_count, tx_ring->cmd);
	ndev->trans_start = jiffies;
out:
	return NETDEV_TX_OK;
}

void xgene_enet_skip_csum(struct sk_buff *skb)
{
	struct iphdr *iph = (struct iphdr *)skb->data;

	if (!(iph->frag_off & htons(IP_MF | IP_OFFSET)) ||
	    (iph->protocol != IPPROTO_TCP && iph->protocol != IPPROTO_UDP)) {
		skb->ip_summed = CHECKSUM_UNNECESSARY;
	}
}

static int xgene_enet_rx_frame(struct xgene_enet_desc_ring *rx_ring,
				struct xgene_enet_desc *desc)
{
	struct net_device *ndev = rx_ring->ndev;
	struct device *dev = ndev_to_dev(rx_ring->ndev);
	struct xgene_enet_desc_ring *buf_pool = rx_ring->buf_pool;
	u32 datalen, skb_index;
	struct sk_buff *skb;
	dma_addr_t pa;
	size_t len;
	int ret = 0;

	skb_index = get_desc(desc, USERINFO);
	skb = buf_pool->rx_skb[skb_index];
	prefetch(skb->data - NET_IP_ALIGN);

	/* Strip off CRC as HW isn't doing this */
	datalen = get_desc(desc, BUFDATALEN);
	datalen -= 4;
	skb_put(skb, datalen);

	pa = (dma_addr_t)get_desc(desc, DATAADDR);
	len = get_desc(desc, BUFDATALEN);
	dma_unmap_single(dev, pa, len, DMA_TO_DEVICE);

	if (--rx_ring->nbufpool == 0) {
		ret = xgene_enet_refill_bufpool(buf_pool, XGENE_ENET_FP_NBUF);
		rx_ring->nbufpool = XGENE_ENET_FP_NBUF;
	}

	skb_checksum_none_assert(skb);
	skb->protocol = eth_type_trans(skb, ndev);
	if (likely((ndev->features & NETIF_F_IP_CSUM) &&
		   skb->protocol == htons(ETH_P_IP))) {
		xgene_enet_skip_csum(skb);
	}

	napi_gro_receive(&rx_ring->napi, skb);

	return ret;
}

static int xgene_enet_process_ring(struct xgene_enet_desc_ring *ring,
				   int budget)
{
	struct net_device *ndev = ring->ndev;
	struct xgene_enet_pdata *pdata = netdev_priv(ring->ndev);
	struct xgene_enet_desc *desc;
	int napi_budget = budget;
	int cmd = 0, ret = 0;
	u16 head = ring->head;
	u16 slots = ring->slots - 1;

	do {
		desc = &ring->desc[head];
		if (unlikely(((u64 *)desc)[EMPTY_SLOT_INDEX] == EMPTY_SLOT))
			break;

		/* Hardware stores descriptor in little endian format */
		xgene_enet_le64_to_cpu(desc, 4);
		if (get_desc(desc, FPQNUM))
			ret = xgene_enet_rx_frame(ring, desc);
		else
			ret = xgene_enet_tx_completion(ring, desc);
		((u64 *)desc)[EMPTY_SLOT_INDEX] = EMPTY_SLOT;

		head = (head + 1) & slots;
		cmd++;

		if (ret)
			goto out;
	} while (--budget);

	if (likely(cmd)) {
		iowrite32(-cmd, ring->cmd);
		ring->head = head;

		if (netif_queue_stopped(ndev)) {
			if (xgene_enet_ring_len(ring) < pdata->cp_qcnt_low)
				netif_wake_queue(ndev);
		}
	}

out:
	return napi_budget - budget;
}

static int xgene_enet_napi(struct napi_struct *napi, const int budget)
{
	struct xgene_enet_desc_ring *ring =
	    container_of(napi, struct xgene_enet_desc_ring, napi);
	int processed = xgene_enet_process_ring(ring, budget);

	if (processed != budget) {
		napi_complete(napi);
		enable_irq(ring->irq);
	}

	return processed;
}

static void xgene_enet_timeout(struct net_device *ndev)
{
	struct xgene_enet_pdata *pdata = netdev_priv(ndev);

	xgene_gmac_reset(pdata);
}

static int xgene_enet_register_irq(struct net_device *ndev)
{
	struct xgene_enet_pdata *pdata = netdev_priv(ndev);
	struct device *dev = &pdata->pdev->dev;
	char irq_name[16];
	int ret;


	snprintf(irq_name, sizeof(irq_name), "%s-tx-rx", ndev->name);
	ret = devm_request_irq(dev, pdata->rx_ring->irq, xgene_enet_rx_irq,
			      IRQF_SHARED, irq_name, pdata->rx_ring);
	if (ret) {
		netdev_err(ndev, "rx%d interrupt request failed\n",
			   pdata->rx_ring->irq);
	}

	return ret;
}

static void xgene_enet_free_irq(struct net_device *ndev)
{
	struct xgene_enet_pdata *pdata = netdev_priv(ndev);
	struct device *dev = &pdata->pdev->dev;

	devm_free_irq(dev, pdata->rx_ring->irq, pdata->rx_ring);
}

static int xgene_enet_open(struct net_device *ndev)
{
	int ret = 0;
	struct xgene_enet_pdata *pdata = netdev_priv(ndev);

	xgene_gmac_tx_enable(pdata);
	xgene_gmac_rx_enable(pdata);

	ret = xgene_enet_register_irq(ndev);
	if (ret)
		goto out;
	napi_enable(&pdata->rx_ring->napi);

	if (pdata->phy_dev)
		phy_start(pdata->phy_dev);

	netif_start_queue(ndev);
out:
	return ret;
}

static int xgene_enet_close(struct net_device *ndev)
{
	struct xgene_enet_pdata *pdata = netdev_priv(ndev);

	netif_stop_queue(ndev);

	if (pdata->phy_dev)
		phy_stop(pdata->phy_dev);

	napi_disable(&pdata->rx_ring->napi);
	xgene_enet_free_irq(ndev);
	xgene_enet_process_ring(pdata->rx_ring, -1);

	xgene_gmac_tx_disable(pdata);
	xgene_gmac_rx_disable(pdata);

	return 0;
}

static void xgene_enet_delete_ring(struct xgene_enet_desc_ring *ring)
{
	struct xgene_enet_pdata *pdata = netdev_priv(ring->ndev);
	struct device *dev = &pdata->pdev->dev;

	xgene_enet_clear_ring(ring);
	dma_free_coherent(dev, ring->size, ring->desc_addr, ring->dma);
	devm_kfree(dev, ring);
}

static void xgene_enet_delete_desc_rings(struct xgene_enet_pdata *pdata)
{
	struct device *dev = &pdata->pdev->dev;
	struct xgene_enet_desc_ring *buf_pool;

	if (pdata->tx_ring) {
		xgene_enet_delete_ring(pdata->tx_ring);
		pdata->tx_ring = NULL;
	}

	if (pdata->rx_ring) {
		buf_pool = pdata->rx_ring->buf_pool;
		xgene_enet_delete_bufpool(buf_pool);
		xgene_enet_delete_ring(buf_pool);
		devm_kfree(dev, buf_pool->rx_skb);

		xgene_enet_delete_ring(pdata->rx_ring);
		pdata->rx_ring = NULL;
	}
}

static int xgene_enet_get_ring_size(struct device *dev,
				    enum xgene_enet_ring_cfgsize cfgsize)
{
	int size = -EINVAL;

	switch (cfgsize) {
	case RING_CFGSIZE_512B:
		size = 0x200;
		break;
	case RING_CFGSIZE_2KB:
		size = 0x800;
		break;
	case RING_CFGSIZE_16KB:
		size = 0x4000;
		break;
	case RING_CFGSIZE_64KB:
		size = 0x10000;
		break;
	case RING_CFGSIZE_512KB:
		size = 0x80000;
		break;
	default:
		dev_err(dev, "Unsupported cfg ring size %d\n", cfgsize);
		break;
	}

	return size;
}

static struct xgene_enet_desc_ring *xgene_enet_create_desc_ring(
			struct net_device *ndev, u32 ring_num,
			enum xgene_enet_ring_cfgsize cfgsize, u32 ring_id)
{
	struct xgene_enet_desc_ring *ring;
	struct xgene_enet_pdata *pdata = netdev_priv(ndev);
	struct device *dev = &pdata->pdev->dev;
	u32 size;

	ring = devm_kzalloc(dev, sizeof(struct xgene_enet_desc_ring),
			    GFP_KERNEL);
	if (!ring) {
		netdev_err(ndev, "Could not allocate ring\n");
		goto err;
	}

	ring->ndev = ndev;
	ring->num = ring_num;
	ring->cfgsize = cfgsize;
	ring->id = ring_id;

	size = xgene_enet_get_ring_size(dev, cfgsize);
	ring->desc_addr = dma_zalloc_coherent(dev, size, &ring->dma,
					      GFP_KERNEL);
	if (!ring->desc_addr) {
		netdev_err(ndev, "Could not allocate desc_addr\n");
		goto err;
	}
	ring->size = size;

	ring->cmd_base = pdata->ring_cmd_addr + (ring->num << 6);
	ring->cmd = ring->cmd_base + 0x2C;
	pdata->rm = RM3;
	ring = xgene_enet_setup_ring(ring);
	netdev_dbg(ndev, "ring info: num=%d  size=%d  id=%d  slots=%d\n",
		   ring->num, ring->size, ring->id, ring->slots);

	return ring;
err:
	if (ring && ring->desc_addr) {
		dma_free_coherent(dev, size, ring->desc_addr, ring->dma);
		devm_kfree(dev, ring);
	}
	if (ring)
		devm_kfree(dev, ring);
	return NULL;
}

static int xgene_enet_create_desc_rings(struct net_device *ndev)
{
	struct xgene_enet_pdata *pdata = netdev_priv(ndev);
	struct device *dev = &pdata->pdev->dev;
	struct xgene_enet_desc_ring *rx_ring, *tx_ring, *cp_ring;
	struct xgene_enet_desc_ring *buf_pool = NULL;
	u32 ring_num = 0;
	u32 ring_id;
	int ret = 0;

	/* allocate rx descriptor ring */
	ring_id = (RING_OWNER_CPU << 6) | RING_BUFNUM_REGULAR;
	rx_ring = xgene_enet_create_desc_ring(ndev, ring_num++,
					      RING_CFGSIZE_16KB, ring_id);
	if (IS_ERR_OR_NULL(rx_ring)) {
		ret = PTR_ERR(rx_ring);
		goto err;
	}

	/* allocate buffer pool for receiving packets */
	ring_id = (RING_OWNER_ETH0 << 6) | RING_BUFNUM_BUFPOOL;
	buf_pool = xgene_enet_create_desc_ring(ndev, ring_num++,
					       RING_CFGSIZE_2KB, ring_id);
	if (IS_ERR_OR_NULL(buf_pool)) {
		ret = PTR_ERR(buf_pool);
		goto err;
	}

	rx_ring->nbufpool = XGENE_ENET_FP_NBUF;
	rx_ring->buf_pool = buf_pool;
	rx_ring->irq = pdata->rx_irq;
	buf_pool->rx_skb = devm_kcalloc(dev, buf_pool->slots,
				     sizeof(struct sk_buff *), GFP_KERNEL);
	if (!buf_pool->rx_skb) {
		netdev_err(ndev, "Could not allocate rx_skb pointers\n");
		ret = -ENOMEM;
		goto err;
	}

	buf_pool->dst_ring_num = xgene_enet_dst_ring_num(buf_pool);
	rx_ring->buf_pool = buf_pool;
	pdata->rx_ring = rx_ring;

	/* allocate tx descriptor ring */
	ring_id = (RING_OWNER_ETH0 << 6) | RING_BUFNUM_REGULAR;
	tx_ring = xgene_enet_create_desc_ring(ndev, ring_num++,
					      RING_CFGSIZE_2KB, ring_id);
	if (IS_ERR_OR_NULL(tx_ring)) {
		ret = PTR_ERR(tx_ring);
		goto err;
	}
	pdata->tx_ring = tx_ring;

	cp_ring = pdata->rx_ring;
	cp_ring->cp_skb = devm_kcalloc(dev, tx_ring->slots,
				     sizeof(struct sk_buff *), GFP_KERNEL);
	if (!cp_ring->cp_skb) {
		netdev_err(ndev, "Could not allocate cp_skb pointers\n");
		ret = -ENOMEM;
		goto err;
	}
	pdata->tx_ring->cp_ring = cp_ring;
	pdata->tx_ring->dst_ring_num = xgene_enet_dst_ring_num(cp_ring);

	pdata->tx_qcnt_hi = pdata->tx_ring->slots / 2;
	pdata->cp_qcnt_hi = pdata->rx_ring->slots / 2;
	pdata->cp_qcnt_low = pdata->cp_qcnt_hi / 2;

	return 0;

err:
	xgene_enet_delete_desc_rings(pdata);
	return ret;
}

static struct net_device_stats *xgene_enet_stats(struct net_device *ndev)
{
	struct xgene_enet_pdata *pdata = netdev_priv(ndev);
	struct net_device_stats *nst = &pdata->nstats;
	struct xgene_enet_detailed_stats detailed_stats;
	struct xgene_enet_rx_stats *rx_stats;
	struct xgene_enet_tx_stats *tx_stats;
	u32 pkt_bytes, crc_bytes = 4;

	memset(&detailed_stats, 0, sizeof(struct xgene_enet_detailed_stats));

	rx_stats = &detailed_stats.rx_stats;
	tx_stats = &detailed_stats.tx_stats;

	local_irq_disable();
	xgene_gmac_get_detailed_stats(pdata, &detailed_stats);

	pkt_bytes = rx_stats->rx_byte_count;
	pkt_bytes -= rx_stats->rx_packet_count * crc_bytes;
	nst->rx_packets += rx_stats->rx_packet_count;
	nst->rx_bytes += pkt_bytes;

	pkt_bytes = tx_stats->tx_byte_count;
	pkt_bytes -= tx_stats->tx_pkt_count * crc_bytes;
	nst->tx_packets += tx_stats->tx_pkt_count;
	nst->tx_bytes += pkt_bytes;

	nst->rx_dropped += rx_stats->rx_drop_pkt_count;
	nst->tx_dropped += tx_stats->tx_drop_frm_count;

	nst->rx_crc_errors += rx_stats->rx_fcs_err_count;
	nst->rx_length_errors += rx_stats->rx_frm_len_err_pkt_count;
	nst->rx_frame_errors += rx_stats->rx_alignment_err_pkt_count;
	nst->rx_over_errors += rx_stats->rx_oversize_pkt_count;

	nst->rx_errors += rx_stats->rx_fcs_err_count
	    + rx_stats->rx_frm_len_err_pkt_count
	    + rx_stats->rx_oversize_pkt_count
	    + rx_stats->rx_undersize_pkt_count;

	nst->tx_errors += tx_stats->tx_fcs_err_frm_count +
	    tx_stats->tx_undersize_frm_count;

	local_irq_enable();

	return nst;
}

static int xgene_enet_set_mac_address(struct net_device *ndev, void *addr)
{
	struct xgene_enet_pdata *pdata = netdev_priv(ndev);
	int ret;

	ret = eth_mac_addr(ndev, addr);
	if (ret)
		goto out;

	xgene_gmac_set_mac_addr(pdata, (unsigned char *)(ndev->dev_addr));
out:
	return ret;
}

static const struct net_device_ops xgene_ndev_ops = {
	.ndo_open = xgene_enet_open,
	.ndo_stop = xgene_enet_close,
	.ndo_start_xmit = xgene_enet_start_xmit,
	.ndo_tx_timeout = xgene_enet_timeout,
	.ndo_get_stats = xgene_enet_stats,
	.ndo_change_mtu = eth_change_mtu,
	.ndo_set_mac_address = xgene_enet_set_mac_address,
};

static int xgene_enet_get_resources(struct xgene_enet_pdata *pdata)
{
	struct platform_device *pdev;
	struct net_device *ndev;
	struct device *dev;
	struct resource *res;
	void *base_addr;
	const char *mac;
	int ret = 0;

	pdev = pdata->pdev;
	dev = &pdev->dev;
	ndev = pdata->ndev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "Resource IORESOURCE_MEM 0 not defined\n");
		ret = -ENODEV;
		goto out;
	}
	pdata->base_addr = devm_ioremap_resource(dev, res);
	if (IS_ERR(pdata->base_addr)) {
		dev_err(dev, "Unable to retrieve ENET Port CSR region\n");
		return PTR_ERR(pdata->base_addr);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(dev, "Resource IORESOURCE_MEM 1 not defined\n");
		ret = -ENODEV;
		goto out;
	}
	pdata->ring_csr_addr = devm_ioremap_resource(dev, res);
	if (IS_ERR(pdata->ring_csr_addr)) {
		dev_err(dev, "Unable to retrieve ENET Ring CSR region\n");
		return PTR_ERR(pdata->ring_csr_addr);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!res) {
		dev_err(dev, "Resource IORESOURCE_MEM 2 not defined\n");
		ret = -ENODEV;
		goto out;
	}
	pdata->ring_cmd_addr = devm_ioremap_resource(dev, res);
	if (IS_ERR(pdata->ring_cmd_addr)) {
		dev_err(dev, "Unable to retrieve ENET Ring command region\n");
		return PTR_ERR(pdata->ring_cmd_addr);
	}

	ret = platform_get_irq(pdev, 0);
	if (ret <= 0) {
		dev_err(dev, "Unable to get ENET Rx IRQ\n");
		goto out;
	}
	pdata->rx_irq = ret;

	mac = of_get_mac_address(dev->of_node);
	if (mac)
		memcpy(ndev->dev_addr, mac, ndev->addr_len);
	else
		eth_hw_addr_random(ndev);
	memcpy(ndev->perm_addr, ndev->dev_addr, ndev->addr_len);

	pdata->phy_mode = of_get_phy_mode(pdev->dev.of_node);
	if (pdata->phy_mode < 0) {
		dev_err(dev, "Incorrect phy-connection-type in DTS\n");
		ret = -EINVAL;
		goto out;
	}

	pdata->clk = devm_clk_get(&pdev->dev, NULL);
	ret = IS_ERR(pdata->clk);
	if (ret) {
		dev_err(&pdev->dev, "can't get clock\n");
		goto out;
	}

	base_addr = pdata->base_addr;
	pdata->eth_csr_addr = base_addr + BLOCK_ETH_CSR_OFFSET;
	pdata->eth_ring_if_addr = base_addr + BLOCK_ETH_RING_IF_OFFSET;
	pdata->eth_diag_csr_addr = base_addr + BLOCK_ETH_DIAG_CSR_OFFSET;
	pdata->mcx_mac_addr = base_addr + BLOCK_ETH_MAC_OFFSET;
	pdata->mcx_stats_addr = base_addr + BLOCK_ETH_STATS_OFFSET;
	pdata->mcx_mac_csr_addr = base_addr + BLOCK_ETH_MAC_CSR_OFFSET;
	pdata->rx_buff_cnt = XGENE_NUM_PKT_BUF;
out:
	return ret;
}

static int xgene_enet_init_hw(struct xgene_enet_pdata *pdata)
{
	struct net_device *ndev = pdata->ndev;
	struct xgene_enet_desc_ring *buf_pool;
	int ret = 0;

	xgene_enet_reset(pdata);

	xgene_gmac_tx_disable(pdata);
	xgene_gmac_rx_disable(pdata);

	ret = xgene_enet_create_desc_rings(ndev);
	if (ret) {
		netdev_err(ndev, "Error in ring configuration\n");
		goto out;
	}

	/* setup buffer pool */
	buf_pool = pdata->rx_ring->buf_pool;
	xgene_enet_init_bufpool(buf_pool);
	ret = xgene_enet_refill_bufpool(buf_pool, pdata->rx_buff_cnt);
	if (ret)
		goto out;

	xgene_enet_cle_bypass_mode_cfg(pdata,
				       xgene_enet_dst_ring_num(pdata->rx_ring),
				       RING_BUFNUM(buf_pool) - 0x20, 0);
	xgene_gmac_init(pdata, ndev->dev_addr, SPEED_1000);
out:
	return ret;
}

static int xgene_enet_probe(struct platform_device *pdev)
{
	struct net_device *ndev;
	struct xgene_enet_pdata *pdata;
	struct device *dev = &pdev->dev;
	struct napi_struct *napi;
	int ret = 0;

	ndev = alloc_etherdev(sizeof(struct xgene_enet_pdata));
	if (!ndev) {
		dev_err(dev, "Could not allocate netdev\n");
		return -ENOMEM;
	}

	pdata = netdev_priv(ndev);

	pdata->pdev = pdev;
	pdata->ndev = ndev;
	SET_NETDEV_DEV(ndev, dev);
	platform_set_drvdata(pdev, pdata);
	ndev->netdev_ops = &xgene_ndev_ops;
	ndev->features |= NETIF_F_IP_CSUM;
	ndev->features |= NETIF_F_GSO;
	ndev->features |= NETIF_F_GRO;

	ret = xgene_enet_get_resources(pdata);
	if (ret)
		goto err;

	ret = register_netdev(ndev);
	if (ret) {
		netdev_err(ndev, "Failed to register net dev!\n");
		goto err;
	}

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret) {
		netdev_err(ndev, "No usable DMA configuration\n");
		goto err;
	}

	ret = xgene_enet_init_hw(pdata);
	if (ret)
		goto err;

	napi = &pdata->rx_ring->napi;
	netif_napi_add(ndev, napi, xgene_enet_napi, NAPI_POLL_WEIGHT);
	ret = xgene_enet_mdio_config(pdata);

	return ret;
err:
	free_netdev(ndev);
	return ret;
}

static int xgene_enet_remove(struct platform_device *pdev)
{
	struct xgene_enet_pdata *pdata;
	struct net_device *ndev;

	pdata = platform_get_drvdata(pdev);
	ndev = pdata->ndev;

	xgene_gmac_rx_disable(pdata);
	xgene_gmac_tx_disable(pdata);

	netif_napi_del(&pdata->rx_ring->napi);
	xgene_enet_mdio_remove(pdata);
	xgene_enet_delete_desc_rings(pdata);
	unregister_netdev(ndev);
	xgene_gport_shutdown(pdata);
	free_netdev(ndev);

	return 0;
}

static struct of_device_id xgene_enet_match[] = {
	{.compatible = "apm,xgene-enet",},
	{},
};

MODULE_DEVICE_TABLE(of, xgene_enet_match);

static struct platform_driver xgene_enet_driver = {
	.driver = {
		   .name = "xgene-enet",
		   .owner = THIS_MODULE,
		   .of_match_table = xgene_enet_match,
		   },
	.probe = xgene_enet_probe,
	.remove = xgene_enet_remove,
};

module_platform_driver(xgene_enet_driver);

MODULE_DESCRIPTION("APM X-Gene SoC Ethernet driver");
MODULE_VERSION("1.0");
MODULE_AUTHOR("Keyur Chudgar <kchudgar@apm.com>");
MODULE_LICENSE("GPL");
