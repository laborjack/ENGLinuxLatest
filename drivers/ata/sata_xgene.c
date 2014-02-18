/*
 * AppliedMicro X-Gene SoC SATA Host Controller Driver
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
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/acpi.h>
#include <linux/efi.h>
#include <linux/phy/phy.h>
#include "ahci.h"

/* Enable for dumping CSR read/write access */
#undef XGENE_DBG_CSR

/* Max # of disk per a controller */
#define MAX_AHCI_CHN_PERCTR		2

#define SATA_DIAG_OFFSET		0x0000D000
#define SATA_GLB_OFFSET			0x0000D850
#define SATA_SHIM_OFFSET		0x0000E000
#define SATA_MASTER_OFFSET		0x0000F000
#define SATA_PORT0_OFFSET		0x00000100
#define SATA_PORT1_OFFSET		0x00000180

/* SATA host controller CSR */
#define SLVRDERRATTRIBUTES_ADDR		0x00000000
#define SLVWRERRATTRIBUTES_ADDR		0x00000004
#define MSTRDERRATTRIBUTES_ADDR		0x00000008
#define MSTWRERRATTRIBUTES_ADDR		0x0000000c
#define BUSCTLREG_ADDR			0x00000014
#define  MSTAWAUX_COHERENT_BYPASS_SET(dst, src) \
		(((dst) & ~0x00000002) | (((u32)(src)<<1) & 0x00000002))
#define  MSTARAUX_COHERENT_BYPASS_SET(dst, src) \
		(((dst) & ~0x00000001) | (((u32)(src)) & 0x00000001))
#define IOFMSTRWAUX_ADDR		0x00000018
#define INTSTATUSMASK_ADDR		0x0000002c
#define ERRINTSTATUS_ADDR		0x00000030
#define ERRINTSTATUSMASK_ADDR		0x00000034

/* SATA host AHCI CSR */
#define PORTCFG_ADDR			0x000000a4
#define  PORTADDR_SET(dst, src) \
		(((dst) & ~0x0000003f) | (((u32)(src)) & 0x0000003f))
#define PORTPHY1CFG_ADDR		0x000000a8
#define PORTPHY1CFG_FRCPHYRDY_SET(dst, src) \
		(((dst) & ~0x00100000) | (((u32)(src) << 0x14) & 0x00100000))
#define PORTPHY2CFG_ADDR		0x000000ac
#define PORTPHY3CFG_ADDR		0x000000b0
#define PORTPHY4CFG_ADDR		0x000000b4
#define PORTPHY5CFG_ADDR		0x000000b8
#define SCTL0_ADDR			0x0000012C
#define PORTPHY5CFG_RTCHG_SET(dst, src) \
		(((dst) & ~0xfff00000) | (((u32)(src) << 0x14) & 0xfff00000))
#define PORTAXICFG_EN_CONTEXT_SET(dst, src) \
		(((dst) & ~0x01000000) | (((u32)(src) << 0x18) & 0x01000000))
#define PORTAXICFG_ADDR			0x000000bc
#define PORTAXICFG_OUTTRANS_SET(dst, src) \
		(((dst) & ~0x00f00000) | (((u32)(src) << 0x14) & 0x00f00000))

/* SATA host controller slave CSR */
#define INT_SLV_TMOMASK_ADDR		0x00000010

/* SATA global diagnostic CSR */
#define REGSPEC_CFG_MEM_RAM_SHUTDOWN_ADDR	0x00000070
#define REGSPEC_BLOCK_MEM_RDY_ADDR		0x00000074

/* AHBC IOB flush CSR */
#define CFG_AMA_MODE_ADDR		0x0000e014
#define  CFG_RD2WR_EN			0x00000002

#define MAX_RETRY_COUNT			3
#define SATA_RESET_MEM_RAM_TO		100000

struct xgene_ahci_context {
	struct ahci_host_priv  hpriv;
	struct device *dev;
	int irq;		/* IRQ */
	void *csr_base;		/* CSR base address of IP */
	u64 csr_phys;		/* Physical address of CSR base address */
	void *mmio_base;	/* AHCI I/O base address */
	u64 mmio_phys;		/* Physical address of MMIO base address */
	void *ahbc_csr_base;	/* Used for IOB flushing if non-zero */
	void *ahbc_io_base;	/* Used for IOB flushing if non-zero */

	struct phy *phy;
};

/* These wrapper existed for future expansion to support running on MSLIM
   cores. */
#define xgene_ahci_fill_cmd_slot	ahci_fill_cmd_slot
#define xgene_ahci_exec_polled_cmd	ahci_exec_polled_cmd
#define xgene_ahci_to_axi(x)		(x)
#define xgene_ahci_dflush(x, ...)

static void xgene_rd(void *addr, u32 *val)
{
	*val = readl(addr);
#if defined(XGENE_DBG_CSR)
	pr_debug("X-Gene SATA CSR RD: 0x%p value: 0x%08x\n", addr, *val);
#endif
}

static void xgene_wr(void *addr, u32 val)
{
	writel(val, addr);
#if defined(XGENE_DBG_CSR)
	pr_debug("X-Gene SATA CSR WR: 0x%p value: 0x%08x\n", addr, val);
#endif
}

static void xgene_wr_flush(void *addr, u32 val)
{
	writel(val, addr);
#if defined(XGENE_DBG_CSR)
	pr_debug("X-Gene SATA CSR WR: 0x%p value: 0x%08x\n", addr, val);
#endif
	val = readl(addr);
}

static int xgene_ahci_get_channel(struct ata_host *host, struct ata_port *port)
{
	int i;
	for (i = 0; i < host->n_ports; i++)
		if (host->ports[i] == port)
			return i;
	return -1;
}

static int xgene_ahci_init_memram(struct xgene_ahci_context *ctx)
{
	void *diagcsr = ctx->csr_base + SATA_DIAG_OFFSET;
	int timeout;
	u32 val;

	xgene_rd(diagcsr + REGSPEC_CFG_MEM_RAM_SHUTDOWN_ADDR, &val);
	if (val == 0) {
		dev_dbg(ctx->dev, "already clear memory shutdown\n");
		return 0;
	}
	dev_dbg(ctx->dev, "clear controller memory shutdown\n");
	/* SATA controller memory in shutdown. Remove from shutdown. */
	xgene_wr_flush(diagcsr + REGSPEC_CFG_MEM_RAM_SHUTDOWN_ADDR, 0x00);
	timeout = SATA_RESET_MEM_RAM_TO;
	do {
		xgene_rd(diagcsr + REGSPEC_BLOCK_MEM_RDY_ADDR, &val);
		if (val != 0xFFFFFFFF)
			udelay(1);
	} while (val != 0xFFFFFFFF && timeout-- > 0);
	if (timeout <= 0) {
		dev_err(ctx->dev, "failed to remove memory from shutdown\n");
		return -ENODEV;
	}
	return 0;
}

/*
 * Custom Query ID command
 *
 * Due to HW errata, we must stop and re-start the port state machine after
 * read ID command.
 */
static unsigned int xgene_ahci_read_id(struct ata_device *dev,
				       struct ata_taskfile *tf, u16 *id)
{
	u32 err_mask;
	struct ata_port *ap = dev->link->ap;
	void *port_mmio = ahci_port_base(ap);
	u32 data32;

	err_mask = ata_do_dev_read_id(dev, tf, id);
	if (err_mask)
		return err_mask;

	/* Mask reserved area. Bit78 spec of Link Power Management
	 * bit15-8: reserved
	 * bit7: NCQ autosence
	 * bit6: Software settings preservation supported
	 * bit5: reserved
	 * bit4: In-order sata delivery supported
	 * bit3: DIPM requests supported
	 * bit2: DMA Setup FIS Auto-Activate optimization supported
	 * bit1: DMA Setup FIX non-Zero buffer offsets supported
	 * bit0: Reserved
	 *
	 * Clear reserved bit (DEVSLP bit) as we don't support DEVSLP
	 */
	id[78] &= 0x00FF;

	/* Restart the port if requred due to HW errata */
	data32 = readl(port_mmio + PORT_CMD_ISSUE);
	if (data32 == 0x00000000) {
		writel(PORT_CMD_FIS_RX, port_mmio + PORT_CMD);
		readl(port_mmio + PORT_CMD);	/* flush */
		writel(PORT_CMD_FIS_RX | PORT_CMD_START, port_mmio + PORT_CMD);
		readl(port_mmio + PORT_CMD);	/* flush */
	}
	return 0;
}

/*
 * Custom QC issue
 *
 * Due to HW errata, we must stop and re-start the port state machine after
 * read ID command.
 */
static unsigned int xgene_ahci_qc_issue(struct ata_queued_cmd *qc)
{
	struct ata_port *ap = qc->ap;
	void __iomem *port_mmio = ahci_port_base(ap);
	struct ahci_port_priv *pp = ap->private_data;

	/* Keep track of the currently active link.  It will be used
	 * in completion path to determine whether NCQ phase is in
	 * progress.
	 */
	pp->active_link = qc->dev->link;

	if (qc->tf.protocol == ATA_PROT_NCQ)
		writel(1 << qc->tag, port_mmio + PORT_SCR_ACT);

	if (pp->fbs_enabled && pp->fbs_last_dev != qc->dev->link->pmp) {
		u32 fbs = readl(port_mmio + PORT_FBS);
		fbs &= ~(PORT_FBS_DEV_MASK | PORT_FBS_DEC);
		fbs |= qc->dev->link->pmp << PORT_FBS_DEV_OFFSET;
		writel(fbs, port_mmio + PORT_FBS);
		pp->fbs_last_dev = qc->dev->link->pmp;
	}

	writel(1 << qc->tag, port_mmio + PORT_CMD_ISSUE);

	ahci_sw_activity(qc->dev->link);

	/* For query ID command, restart the port if requred due to HW errata.
	   This is needed when PMP is attached. */
	if (qc->dev->link->pmp && qc->tf.command == ATA_CMD_ID_ATA &&
	    readl(port_mmio + PORT_CMD_ISSUE) == 0x0) {
		writel(PORT_CMD_FIS_RX, port_mmio + PORT_CMD);
		readl(port_mmio + PORT_CMD);	/* flush */
		writel(PORT_CMD_FIS_RX | PORT_CMD_START, port_mmio + PORT_CMD);
		readl(port_mmio + PORT_CMD);	/* flush */
	}

	return 0;
}

static void xgene_ahci_enable_phy(struct xgene_ahci_context *ctx,
				  int channel, int enable)
{
	void *mmio = ctx->mmio_base;
	u32 val;

	xgene_rd(mmio + PORTCFG_ADDR, &val);
	val = PORTADDR_SET(val, channel == 0 ? 2 : 3);
	xgene_wr_flush(mmio + PORTCFG_ADDR, val);
	xgene_rd(mmio + PORTPHY1CFG_ADDR, &val);
	val = PORTPHY1CFG_FRCPHYRDY_SET(val, enable);
	xgene_wr(mmio + PORTPHY1CFG_ADDR, val);
}

static void xgene_ahci_set_phy_cfg(struct xgene_ahci_context *ctx, int channel)
{
	void *mmio = ctx->mmio_base;
	u32 val;

	dev_dbg(ctx->dev, "port configure mmio 0x%p channel %d\n",
		mmio, channel);
	xgene_rd(mmio + PORTCFG_ADDR, &val);
	val = PORTADDR_SET(val, channel == 0 ? 2 : 3);
	xgene_wr_flush(mmio + PORTCFG_ADDR, val);
	/* Disable fix rate */
	xgene_wr_flush(mmio + PORTPHY1CFG_ADDR, 0x0001fffe);
	xgene_wr_flush(mmio + PORTPHY2CFG_ADDR, 0x5018461c);
	xgene_wr_flush(mmio + PORTPHY3CFG_ADDR, 0x1c081907);
	xgene_wr_flush(mmio + PORTPHY4CFG_ADDR, 0x1c080815);
	xgene_rd(mmio + PORTPHY5CFG_ADDR, &val);
	/* Window negotiation 0x800 to 0x400 */
	val = PORTPHY5CFG_RTCHG_SET(val, 0x300);
	xgene_wr_flush(mmio + PORTPHY5CFG_ADDR, val);
	xgene_rd(mmio + PORTAXICFG_ADDR, &val);
	val = PORTAXICFG_EN_CONTEXT_SET(val, 0x1); /* enable context mgmt */
	val = PORTAXICFG_OUTTRANS_SET(val, 0xe); /* Outstanding */
	xgene_wr_flush(mmio + PORTAXICFG_ADDR, val);
}

/* Restart the PHY in case of disparity error for Gen2/Gen1 disk only */
static int xgene_ahci_phy_restart(struct ata_link *link)
{
	struct ata_port *port = link->ap;
	struct ata_host *host = port->host;
	struct xgene_ahci_context *ctx = host->private_data;
	int channel;

	channel = xgene_ahci_get_channel(host, port);
	if (channel < 0 || channel >= MAX_AHCI_CHN_PERCTR)
		return -EINVAL;
	xgene_ahci_enable_phy(ctx, channel, 1);
	xgene_ahci_enable_phy(ctx, channel, 0);
	return 0;
}

static int xgene_ahci_do_hardreset(struct ata_link *link, int chan,
				   unsigned long deadline, bool *online)
{
	const unsigned long *timing = sata_ehc_deb_timing(&link->eh_context);
	struct ata_port *ap = link->ap;
	struct xgene_ahci_context *ctx = ap->host->private_data;
	struct ahci_port_priv *pp = ap->private_data;
	u8 *d2h_fis = pp->rx_fis + RX_FIS_D2H_REG;
	void __iomem *port_mmio = ahci_port_base(ap);
	struct ata_taskfile tf;
	int link_retry = 0;
	int retry = 0;
	int rc;
	u32 val;

hardreset_retry:
	/* clear D2H reception area to properly wait for D2H FIS */
	ata_tf_init(link->device, &tf);
	tf.command = 0x80;
	ata_tf_to_fis(&tf, 0, 0, d2h_fis);
	rc = sata_link_hardreset(link, timing, deadline, online,
				 ahci_check_ready);
	/* clear all errors */
	xgene_rd(port_mmio + PORT_SCR_ERR, &val);
	xgene_wr(port_mmio + PORT_SCR_ERR, val);

	/* Check to ensure that the disk comes up in match speed */
	if (*online) {
		u32 sstatus;
		sata_scr_read(link, SCR_STATUS, &sstatus);
		if (!retry) {
			if (((sstatus >> 4) & 0xf) == 2) {
				/* For Gen2 and first time, let's check again
				 * with Gen2 serdes to ensure actual Gen2 disk.
				 */
				phy_set_speed(ctx->phy, chan, 3000000000);
				xgene_ahci_phy_restart(link);
				++retry;
				goto hardreset_retry;
			} else if (((sstatus >> 4) & 0xf) == 1) {
				/* For Gen1 and first time, let's check again
				 * with Gen1 serdes to ensure actual Gen1 disk.
				 */
				phy_set_speed(ctx->phy, chan, 1500000000);
				xgene_ahci_phy_restart(link);
				++retry;
				goto hardreset_retry;
			}
		}
	} else if (link_retry < 4) {
		link_retry++;
		goto hardreset_retry;
	}
	ata_link_dbg(link, "channel %d post-hardrest PORT_CMD 0x%08X\n",
		     chan, readl(port_mmio + PORT_CMD));

	return rc;
}

static int xgene_ahci_hardreset(struct ata_link *link, unsigned int *class,
				unsigned long deadline)
{
	struct ata_port *ap = link->ap;
	void __iomem *port_mmio = ahci_port_base(ap);
	bool online;
	int chan;
	int rc;
	int portcmd_saved;
	u32 portclb_saved;
	u32 portclbhi_saved;
	u32 portrxfis_saved;
	u32 portrxfishi_saved;

	chan = xgene_ahci_get_channel(ap->host, ap);
	if (chan >= MAX_AHCI_CHN_PERCTR) {
		*class = ATA_DEV_NONE;
		return 0;
	}
	ata_link_dbg(link, "channel %d APM hardreset\n", chan);

	/* As hardreset reset these CSR, let save it to restore later */
	portcmd_saved = readl(port_mmio + PORT_CMD);
	portclb_saved = readl(port_mmio + PORT_LST_ADDR);
	portclbhi_saved = readl(port_mmio + PORT_LST_ADDR_HI);
	portrxfis_saved = readl(port_mmio + PORT_FIS_ADDR);
	portrxfishi_saved = readl(port_mmio + PORT_FIS_ADDR_HI);

	ahci_stop_engine(ap);

	rc = xgene_ahci_do_hardreset(link, chan, deadline, &online);

	/* As controller hardreset clear them, let restore them */
	writel(portcmd_saved, port_mmio + PORT_CMD);
	writel(portclb_saved, port_mmio + PORT_LST_ADDR);
	writel(portclbhi_saved, port_mmio + PORT_LST_ADDR_HI);
	writel(portrxfis_saved, port_mmio + PORT_FIS_ADDR);
	writel(portrxfishi_saved, port_mmio + PORT_FIS_ADDR_HI);

	ahci_start_engine(ap);

	if (online)
		*class = ahci_dev_classify(ap);

	ata_link_dbg(link, "channel %d APM hardreset EXIT class %u\n",
		     chan, *class);
	return rc;
}

/* Flush the IOB to ensure all SATA controller writes completed before
   servicing the completed command. This is needed due to the possibility
   that interrupt serviced before the data actually written to the cache/DDR.
   Writes from the IP to the CPU domain is not synchronized with the IRQ
   line or the IP core toggled the CI bits before the data write completed. */
static int xgene_ahci_iob_flush(struct xgene_ahci_context *ctx)
{
	if (ctx->ahbc_io_base)
		readl(ctx->ahbc_io_base);
	return 0;
}

static unsigned int xgene_ahci_fill_sg(struct ata_queued_cmd *qc,
				       void *cmd_tbl)
{
	struct scatterlist *sg;
	struct ahci_sg *ahci_sg = cmd_tbl + AHCI_CMD_TBL_HDR_SZ;
	unsigned int si;

	/*
	 * Next, the S/G list.
	 */
	for_each_sg(qc->sg, sg, qc->n_elem, si) {
		dma_addr_t addr = sg_dma_address(sg);
		u64 dma_addr = xgene_ahci_to_axi(addr);
		u32 sg_len = sg_dma_len(sg);
		ahci_sg[si].addr = cpu_to_le32(dma_addr & 0xffffffff);
		ahci_sg[si].addr_hi = cpu_to_le32((dma_addr >> 16) >> 16);
		ahci_sg[si].flags_size = cpu_to_le32(sg_len - 1);
		xgene_ahci_dflush((void *) __va(addr), sg_len);
	}
	return si;
}

static void xgene_ahci_qc_prep(struct ata_queued_cmd *qc)
{
	struct ata_port *ap = qc->ap;
	struct ahci_port_priv *pp = ap->private_data;
	int is_atapi = ata_is_atapi(qc->tf.protocol);
	void *cmd_tbl;
	u32 opts;
	const u32 cmd_fis_len = 5;	/* five dwords */
	unsigned int n_elem;
	void *port_mmio = ahci_port_base(ap);
	u32 fbs;

	/*
	 * Fill in command table information.  First, the header,
	 * a SATA Register - Host to Device command FIS.
	 */
	cmd_tbl = pp->cmd_tbl + qc->tag * AHCI_CMD_TBL_SZ;

	/* Due to hardware errata for port multipier CBS mode, enable DEV
	   field of PxFBS in order to clear the PxCI */
	fbs = readl(port_mmio + 0x40);
	if (qc->dev->link->pmp || ((fbs >> 8) & 0x0000000f)) {
		fbs &= 0xfffff0ff;
		fbs |= qc->dev->link->pmp << 8;
		writel(fbs, port_mmio + 0x40);
	}

	ata_tf_to_fis(&qc->tf, qc->dev->link->pmp, 1, cmd_tbl);
	if (is_atapi) {
		memset(cmd_tbl + AHCI_CMD_TBL_CDB, 0, 32);
		memcpy(cmd_tbl + AHCI_CMD_TBL_CDB, qc->cdb, qc->dev->cdb_len);
	}
	n_elem = 0;
	if (qc->flags & ATA_QCFLAG_DMAMAP)
		n_elem = xgene_ahci_fill_sg(qc, cmd_tbl);

	/*
	 * Fill in command slot information.
	 */
	opts = cmd_fis_len | n_elem << 16 | (qc->dev->link->pmp << 12);
	if (qc->tf.flags & ATA_TFLAG_WRITE)
		opts |= AHCI_CMD_WRITE;
	if (is_atapi)
		opts |= AHCI_CMD_ATAPI | AHCI_CMD_PREFETCH;

	xgene_ahci_fill_cmd_slot(pp, qc->tag, opts);
}

/* Due to HW BUG we are limited to single FIS receive area for FBS so
 * limiting the FBS FIS area from 16 to 0.
 */
static bool xgene_ahci_qc_fill_rtf(struct ata_queued_cmd *qc)
{
	struct ahci_port_priv *pp = qc->ap->private_data;
	u8 *rx_fis = pp->rx_fis;

	/*
	 * After a successful execution of an ATA PIO data-in command,
	 * the device doesn't send D2H Reg FIS to update the TF and
	 * the host should take TF and E_Status from the preceding PIO
	 * Setup FIS.
	 */
	if (qc->tf.protocol == ATA_PROT_PIO && qc->dma_dir == DMA_FROM_DEVICE &&
	    !(qc->flags & ATA_QCFLAG_FAILED)) {
		ata_tf_from_fis(rx_fis + RX_FIS_PIO_SETUP, &qc->result_tf);
		qc->result_tf.command = (rx_fis + RX_FIS_PIO_SETUP)[15];
	} else
		ata_tf_from_fis(rx_fis + RX_FIS_D2H_REG, &qc->result_tf);

	return true;
}

static int xgene_ahci_do_softreset(struct ata_link *link,
				   unsigned int *class, int pmp,
				   unsigned long deadline,
				   int (*check_ready) (struct ata_link *link))
{
	struct ata_port *ap = link->ap;
	struct ahci_host_priv *hpriv = ap->host->private_data;
	const char *reason = NULL;
	unsigned long now, msecs;
	struct ata_taskfile tf;
	int rc;

	ata_link_dbg(link, "ENTER\n");

	/* prepare for SRST (AHCI-1.1 10.4.1) */
	rc = ahci_kick_engine(ap);
	if (rc && rc != -EOPNOTSUPP)
		ata_link_warn(link, "failed to reset engine (errno=%d)\n", rc);

	ata_tf_init(link->device, &tf);
	/* issue the first D2H Register FIS */
	msecs = 0;
	now = jiffies;
	if (time_after(deadline, now))
		msecs = jiffies_to_msecs(deadline - now);

	tf.ctl |= ATA_SRST;
	/* Must call X-Gene version in case it needs to flush the cache for
	   MSLIM as well as AXI address translation */
	if (xgene_ahci_exec_polled_cmd(ap, pmp, &tf, 0,
				       AHCI_CMD_RESET | AHCI_CMD_CLR_BUSY,
				       msecs)) {
		rc = -EIO;
		reason = "1st FIS failed";
		goto fail;
	}

	/* spec says at least 5us, but be generous and sleep for 1ms */
	ata_msleep(ap, 1);

	/* issue the second D2H Register FIS */
	tf.ctl &= ~ATA_SRST;
	/* HW need AHCI_CMD_RESET and AHCI_CMD_CLR_BUSY */
	xgene_ahci_exec_polled_cmd(ap, pmp, &tf, 0,
				   AHCI_CMD_RESET | AHCI_CMD_CLR_BUSY, msecs);
	/* wait for link to become ready */
	rc = ata_wait_after_reset(link, deadline, check_ready);
	if (rc == -EBUSY && hpriv->flags & AHCI_HFLAG_SRST_TOUT_IS_OFFLINE) {
		/*
		 * Workaround for cases where link online status can't
		 * be trusted.  Treat device readiness timeout as link
		 * offline.
		 */
		ata_link_info(link, "device not ready, treating as offline\n");
		*class = ATA_DEV_NONE;
	} else if (rc) {
		/* link occupied, -ENODEV too is an error */
		reason = "device not ready";
		goto fail;
	} else {
		*class = ahci_dev_classify(ap);
	}

	ata_link_dbg(link, "EXIT, class=%u\n", *class);
	return 0;

fail:
	ata_link_err(link, "softreset failed (%s)\n", reason);
	return rc;
}

static int xgene_ahci_softreset(struct ata_link *link, unsigned int *class,
				unsigned long deadline)
{
	int pmp = sata_srst_pmp(link);
	return xgene_ahci_do_softreset(link, class, pmp, deadline,
				       ahci_check_ready);
}

static struct ata_port_operations xgene_ahci_ops = {
	.inherits = &ahci_ops,
	.hardreset = xgene_ahci_hardreset,
	.read_id = xgene_ahci_read_id,
	.qc_prep = xgene_ahci_qc_prep,
	.qc_issue = xgene_ahci_qc_issue,
	.softreset = xgene_ahci_softreset,
	.pmp_softreset = xgene_ahci_softreset,
	.qc_fill_rtf = xgene_ahci_qc_fill_rtf,
};

static const struct ata_port_info xgene_ahci_port_info[] = {
	{
	 .flags = AHCI_FLAG_COMMON,
	 .pio_mask = ATA_PIO4,
	 .udma_mask = ATA_UDMA6,
	 .port_ops = &xgene_ahci_ops,
	 },
};

static struct scsi_host_template xgene_ahci_sht = {
	AHCI_SHT("XGene-ahci"),
};

static void xgene_ahci_port_intr(struct ata_port *ap)
{
	void __iomem *port_mmio = ahci_port_base(ap);
	struct ata_eh_info *ehi = &ap->link.eh_info;
	struct ahci_port_priv *pp = ap->private_data;
	struct ahci_host_priv *hpriv = ap->host->private_data;
	int resetting = !!(ap->pflags & ATA_PFLAG_RESETTING);
	u32 status, qc_active = 0;
	int rc;

	status = readl(port_mmio + PORT_IRQ_STAT);
	writel(status, port_mmio + PORT_IRQ_STAT);

	/* ignore BAD_PMP while resetting */
	if (unlikely(resetting))
		status &= ~PORT_IRQ_BAD_PMP;

	/* if LPM is enabled, PHYRDY doesn't mean anything */
	if (ap->link.lpm_policy > ATA_LPM_MAX_POWER) {
		status &= ~PORT_IRQ_PHYRDY;
		ahci_scr_write(&ap->link, SCR_ERROR, SERR_PHYRDY_CHG);
	}

	if (unlikely(status & PORT_IRQ_ERROR)) {
		ahci_error_intr(ap, status);
		return;
	}

	if (status & PORT_IRQ_SDB_FIS) {
		/* If SNotification is available, leave notification
		 * handling to sata_async_notification().  If not,
		 * emulate it by snooping SDB FIS RX area.
		 *
		 * Snooping FIS RX area is probably cheaper than
		 * poking SNotification but some constrollers which
		 * implement SNotification, ICH9 for example, don't
		 * store AN SDB FIS into receive area.
		 */
		if (hpriv->cap & HOST_CAP_SNTF)
			sata_async_notification(ap);
		else {
			/* If the 'N' bit in word 0 of the FIS is set,
			 * we just received asynchronous notification.
			 * Tell libata about it.
			 *
			 * Lack of SNotification should not appear in
			 * ahci 1.2, so the workaround is unnecessary
			 * when FBS is enabled.
			 */
			if (pp->fbs_enabled)
				WARN_ON_ONCE(1);
			else {
				const __le32 *f = pp->rx_fis + RX_FIS_SDB;
				u32 f0 = le32_to_cpu(f[0]);
				if (f0 & (1 << 15))
					sata_async_notification(ap);
			}
		}
	}

	/* pp->active_link is not reliable once FBS is enabled, both
	 * PORT_SCR_ACT and PORT_CMD_ISSUE should be checked because
	 * NCQ and non-NCQ commands may be in flight at the same time.
	 */
	if (pp->fbs_enabled) {
		if (ap->qc_active) {
			qc_active = readl(port_mmio + PORT_SCR_ACT);
			qc_active |= readl(port_mmio + PORT_CMD_ISSUE);
		}
	} else {
		/* pp->active_link is valid iff any command is in flight */
		if (ap->qc_active && pp->active_link->sactive)
			qc_active = readl(port_mmio + PORT_SCR_ACT);
		else
			qc_active = readl(port_mmio + PORT_CMD_ISSUE);
	}

	/* Flush the IOB before servicing interrupt to ensure all data
	   written by the controller appears in DDR */
	xgene_ahci_iob_flush((struct xgene_ahci_context *) hpriv);

	rc = ata_qc_complete_multiple(ap, qc_active);

	/* while resetting, invalid completions are expected */
	if (unlikely(rc < 0 && !resetting)) {
		ehi->err_mask |= AC_ERR_HSM;
		ehi->action |= ATA_EH_RESET;
		ata_port_freeze(ap);
	}
}

static irqreturn_t xgene_ahci_interrupt(int irq, void *dev_instance)
{
	struct ata_host *host = dev_instance;
	struct ahci_host_priv *hpriv;
	unsigned int i, handled = 0;
	void __iomem *mmio;
	u32 irq_stat, irq_masked;

	VPRINTK("ENTER\n");

	hpriv = host->private_data;
	mmio = hpriv->mmio;

	/* sigh.  0xffffffff is a valid return from h/w */
	irq_stat = readl(mmio + HOST_IRQ_STAT);
	if (!irq_stat)
		return IRQ_NONE;

	irq_masked = irq_stat & hpriv->port_map;

	spin_lock(&host->lock);

	for (i = 0; i < host->n_ports; i++) {
		struct ata_port *ap;

		if (!(irq_masked & (1 << i)))
			continue;

		ap = host->ports[i];
		if (ap) {
			xgene_ahci_port_intr(ap);
			VPRINTK("port %u\n", i);
		} else {
			VPRINTK("port %u (no irq)\n", i);
			if (ata_ratelimit())
				dev_warn(host->dev,
					 "interrupt on disabled port %u\n", i);
		}

		handled = 1;
	}

	/* HOST_IRQ_STAT behaves as level triggered latch meaning that
	 * it should be cleared after all the port events are cleared;
	 * otherwise, it will raise a spurious interrupt after each
	 * valid one.  Please read section 10.6.2 of ahci 1.1 for more
	 * information.
	 *
	 * Also, use the unmasked value to clear interrupt as spurious
	 * pending event on a dummy port might cause screaming IRQ.
	 */
	writel(irq_stat, mmio + HOST_IRQ_STAT);

	spin_unlock(&host->lock);

	VPRINTK("EXIT\n");

	return IRQ_RETVAL(handled);
}

static int xgene_ahci_hw_init(struct xgene_ahci_context *hpriv)
{
	int i;
	int rc;
	u32 val;

	/* Remove IP RAM out of shutdown */
	rc = xgene_ahci_init_memram(hpriv);
	if (rc)
		return rc;

	for (i = 0; i < MAX_AHCI_CHN_PERCTR; i++)
		xgene_ahci_set_phy_cfg(hpriv, i);

	/* Map in the IOB register */
	rc = xgene_ahci_iob_flush(hpriv);

	/* Now enable top level interrupt. Otherwise, port interrupt will
	   not work. */
	/* AXI disable Mask */
	xgene_wr_flush(hpriv->mmio_base + HOST_IRQ_STAT, 0xffffffff);
	xgene_wr(hpriv->csr_base + INTSTATUSMASK_ADDR, 0);
	xgene_rd(hpriv->csr_base + INTSTATUSMASK_ADDR, &val);
	dev_dbg(hpriv->dev, "top level interrupt mask 0x%X value 0x%08X\n",
		INTSTATUSMASK_ADDR, val);
	xgene_wr_flush(hpriv->csr_base + ERRINTSTATUSMASK_ADDR, 0x0);
	xgene_wr_flush(hpriv->csr_base + SATA_SHIM_OFFSET +
			       INT_SLV_TMOMASK_ADDR, 0x0);
	/* Enable AXI Interrupt */
	xgene_wr(hpriv->csr_base + SLVRDERRATTRIBUTES_ADDR, 0xffffffff);
	xgene_wr(hpriv->csr_base + SLVWRERRATTRIBUTES_ADDR, 0xffffffff);
	xgene_wr(hpriv->csr_base + MSTRDERRATTRIBUTES_ADDR, 0xffffffff);
	xgene_wr(hpriv->csr_base + MSTWRERRATTRIBUTES_ADDR, 0xffffffff);

	/* Enable coherency unless explicit disabled */
	xgene_rd(hpriv->csr_base + BUSCTLREG_ADDR, &val);
	val = MSTAWAUX_COHERENT_BYPASS_SET(val, 0);
	val = MSTARAUX_COHERENT_BYPASS_SET(val, 0);
	xgene_wr(hpriv->csr_base + BUSCTLREG_ADDR, val);

	xgene_rd(hpriv->csr_base + IOFMSTRWAUX_ADDR, &val);
	val |= (1 << 3);	/* Enable read coherency */
	val |= (1 << 9);	/* Enable write coherency */
	xgene_wr_flush(hpriv->csr_base + IOFMSTRWAUX_ADDR, val);
	xgene_rd(hpriv->csr_base + IOFMSTRWAUX_ADDR, &val);
	dev_dbg(hpriv->dev, "coherency 0x%X value 0x%08X\n",
		IOFMSTRWAUX_ADDR, val);

	if (hpriv->ahbc_csr_base) {
		/* Enable IOB flush feature */
		val = readl(hpriv->ahbc_csr_base + CFG_AMA_MODE_ADDR);
		val |= CFG_RD2WR_EN;
		writel(val, hpriv->ahbc_csr_base + CFG_AMA_MODE_ADDR);
		dev_dbg(hpriv->dev, "enable IOB flush\n");
	}
	return rc;
}

static int xgene_ahci_probe(struct platform_device *pdev)
{
	struct xgene_ahci_context *hpriv;
	struct ata_port_info pi = xgene_ahci_port_info[0];
	const struct ata_port_info *ppi[] = { &pi, NULL };
	struct ata_host *host;
	struct resource *res;
	char res_name[30];
	int n_ports;
	int rc = 0;
	int i;

	/* When both ACPi and DTS are enabled, custom ACPI built-in ACPI
	   table, and booting via DTS, we need to skip the probe of the
	   built-in ACPI table probe. */
	if (!efi_enabled(EFI_BOOT) && pdev->dev.of_node == NULL)
		return -ENODEV;

	/* Check if the entry is disabled for OF only */
	if (!efi_enabled(EFI_BOOT) &&
	    !of_device_is_available(pdev->dev.of_node))
		return -ENODEV;
#if defined(CONFIG_ACPI)
	if (efi_enabled(EFI_BOOT)) {
		struct acpi_device *device;

		if (acpi_bus_get_device(ACPI_HANDLE(&pdev->dev), &device))
			return -ENODEV;

		if (acpi_bus_get_status(device) || !device->status.present)
			return -ENODEV;
	}
#endif

	hpriv = devm_kzalloc(&pdev->dev, sizeof(*hpriv), GFP_KERNEL);
	if (!hpriv) {
		dev_err(&pdev->dev, "can't allocate host context\n");
		return -ENOMEM;
	}
	hpriv->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no AHCI MMIO resource address\n");
		goto error;
	}
	hpriv->mmio_phys = res->start;
	hpriv->mmio_base = devm_ioremap(&pdev->dev, res->start,
					resource_size(res));
	if (!hpriv->mmio_base) {
		dev_err(&pdev->dev, "can't map AHCI MMIO resource\n");
		rc = -ENOMEM;
		goto error;
	}
	hpriv->hpriv.mmio = hpriv->mmio_base;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(&pdev->dev, "no host resource address\n");
		goto error;
	}
	hpriv->csr_phys = res->start;
	hpriv->csr_base = devm_ioremap(&pdev->dev, res->start,
				       resource_size(res));
	if (!hpriv->csr_base) {
		dev_err(&pdev->dev, "can't map host resource\n");
		rc = -ENOMEM;
		goto error;
	}

	/* Both the IOB CSR and IOB IO flush registers must be available
	   in order to enable the IOB flush feature. */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (res) {
		hpriv->ahbc_csr_base = devm_ioremap(&pdev->dev, res->start,
						    resource_size(res));
		if (!hpriv->ahbc_csr_base) {
			dev_err(&pdev->dev,
				"can't map IOB CSR flush resource\n");
			rc = -ENOMEM;
			goto error;
		}
		res = platform_get_resource(pdev, IORESOURCE_MEM, 3);
		if (!res) {
			dev_err(&pdev->dev, "no IOB IO CSR resource\n");
			rc = -ENOMEM;
			goto error;
		}
		hpriv->ahbc_io_base = devm_ioremap(&pdev->dev, res->start,
						   resource_size(res));
		if (!hpriv->ahbc_io_base) {
			dev_err(&pdev->dev, "can't map IOB IO CSR resource\n");
			rc = -ENOMEM;
			goto error;
		}
	}

	dev_dbg(&pdev->dev,
		"PHY PAddr 0x%016LX VAddr 0x%p Mmio PAddr 0x%016LX VAddr 0x%p\n",
		hpriv->csr_phys, hpriv->csr_base, hpriv->mmio_phys,
		hpriv->mmio_base);

	hpriv->irq = platform_get_irq(pdev, 0);
	if (hpriv->irq <= 0) {
		dev_err(&pdev->dev, "no IRQ resource\n");
		rc = -ENODEV;
		goto error;
	}

	/* Configure the PHY */
	sprintf(res_name, "sataphy%08x", (u32) hpriv->csr_phys);
	hpriv->phy = devm_phy_get(&pdev->dev, res_name);
	if (!hpriv->phy) {
		dev_err(&pdev->dev, "no PHY available\n");
		goto error;
	}

	rc = phy_init(hpriv->phy);
	if (rc) {
		dev_err(&pdev->dev, "PHY initialize failed %d\n", rc);
		goto error;
	}

	/* Configure the host controller */
	xgene_ahci_hw_init(hpriv);

	/* Setup AHCI host priv structure */
	ahci_save_initial_config(&pdev->dev, &hpriv->hpriv, 0, 0);

	/* prepare host */
	if (hpriv->hpriv.cap & HOST_CAP_NCQ)
		pi.flags |= ATA_FLAG_NCQ;
	if (hpriv->hpriv.cap & HOST_CAP_PMP) {
		pi.flags |= ATA_FLAG_PMP;
		if (hpriv->hpriv.cap & HOST_CAP_FBS)
			hpriv->hpriv.flags |= AHCI_HFLAG_YES_FBS;
	}
	ahci_set_em_messages(&hpriv->hpriv, &pi);

	/* CAP.NP sometimes indicate the index of the last enabled
	 * port, at other times, that of the last possible port, so
	 * determining the maximum port number requires looking at
	 * both CAP.NP and port_map.
	 */
	n_ports = max(ahci_nr_ports(hpriv->hpriv.cap),
		      fls(hpriv->hpriv.port_map));

	host = ata_host_alloc_pinfo(&pdev->dev, ppi, n_ports);
	if (!host) {
		dev_err(&pdev->dev, "can not allocate host pinfo\n");
		rc = -ENOMEM;
		goto error;
	}

	host->private_data = hpriv;

	if (!(hpriv->hpriv.cap & HOST_CAP_SSS) || ahci_ignore_sss)
		host->flags |= ATA_HOST_PARALLEL_SCAN;
	else
		dev_warn(&pdev->dev,
			 "ahci: SSS flag set, parallel bus scan disabled\n");

	if (pi.flags & ATA_FLAG_EM)
		ahci_reset_em(host);

	for (i = 0; i < host->n_ports; i++) {
		struct ata_port *ap = host->ports[i];

		ata_port_desc(ap, "mmio 0x%llX", hpriv->mmio_phys);
		ata_port_desc(ap, "port 0x%x", 0x100 + ap->port_no * 0x80);

		/* set enclosure management message type */
		if (ap->flags & ATA_FLAG_EM)
			ap->em_message_type = hpriv->hpriv.em_msg_type;

		/* disabled/not-implemented port */
		if (!(hpriv->hpriv.port_map & (1 << i)))
			ap->ops = &ata_dummy_port_ops;
	}

	rc = ahci_reset_controller(host);
	if (rc)
		goto error;

	ahci_init_controller(host);
	ahci_print_info(host, "XGene-AHCI\n");

	/* Setup DMA mask */
	dma_set_mask(&pdev->dev, DMA_BIT_MASK(64));
	dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(64));

	rc = ata_host_activate(host, hpriv->irq, xgene_ahci_interrupt,
			       IRQF_SHARED, &xgene_ahci_sht);
	if (rc)
		goto error;

	dev_dbg(&pdev->dev, "X-Gene SATA host controller initialized\n");
	return 0;

error:
	devm_kfree(&pdev->dev, hpriv);
	return rc;
}

static int xgene_ahci_remove(struct platform_device *pdev)
{
	dev_dbg(&pdev->dev, "remove\n");
	return 0;
}

#ifdef CONFIG_PM
static int xgene_ahci_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct ata_host *host = dev_get_drvdata(&pdev->dev);
	struct xgene_ahci_context *hpriv = host->private_data;
	void __iomem *mmio = hpriv->mmio_base;
	u32 ctl;
	int rc;

	dev_dbg(&pdev->dev, "suspend\n");

	/*
	 * AHCI spec rev1.1 section 8.3.3:
	 * Software must disable interrupts prior to requesting a
	 * transition of the HBA to D3 state.
	 */
	ctl = readl(mmio + HOST_CTL);
	ctl &= ~HOST_IRQ_EN;
	writel(ctl, mmio + HOST_CTL);
	readl(mmio + HOST_CTL);	/* flush */

	rc = ata_host_suspend(host, state);
	if (rc)
		return rc;

	return 0;
}

static int xgene_ahci_resume(struct platform_device *pdev)
{
	struct ata_host *host = dev_get_drvdata(&pdev->dev);
	struct xgene_ahci_context *hpriv = host->private_data;
	int rc;

	dev_dbg(&pdev->dev, "resume\n");

	if (pdev->dev.power.power_state.event == PM_EVENT_SUSPEND) {
		rc = ahci_reset_controller(host);
		if (rc)
			return rc;

		ahci_init_controller(host);
	}

	ata_host_resume(host);
	return 0;
}
#endif

static const struct acpi_device_id xgene_ahci_acpi_match[] = {
	{"APMC0D00", 0},
	{},
};

MODULE_DEVICE_TABLE(acpi, xgene_ahci_acpi_match);

static const struct of_device_id xgene_ahci_of_match[] = {
	{.compatible = "apm,xgene-ahci",},
	{},
};

MODULE_DEVICE_TABLE(of, xgene_ahci_of_match);

static struct platform_driver xgene_ahci_driver = {
	.driver = {
		   .name = "xgene-ahci",
		   .owner = THIS_MODULE,
		   .of_match_table = xgene_ahci_of_match,
		   .acpi_match_table = ACPI_PTR(xgene_ahci_acpi_match),
		   },
	.probe = xgene_ahci_probe,
	.remove = xgene_ahci_remove,
#ifdef CONFIG_PM
	.suspend = xgene_ahci_suspend,
	.resume = xgene_ahci_resume,
#endif
};

module_platform_driver(xgene_ahci_driver);

MODULE_DESCRIPTION("APM X-Gene AHCI SATA driver");
MODULE_AUTHOR("Loc Ho <lho@apm.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.3");
