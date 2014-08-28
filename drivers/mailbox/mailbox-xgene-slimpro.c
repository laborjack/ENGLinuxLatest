/*
 * APM X-Gene SLIMpro MailBox Driver
 *
 * Copyright (c) 2014, Applied Micro Circuits Corporation
 * Author: Feng Kan fkan@apm.com
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/interrupt.h>
#include <linux/acpi.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/mailbox_controller.h>

#define MBOX_CON_NAME			"slimpro-mbox"
#define MBOX_REG_SET_OFFSET		0x1000
#define MBOX_CNT			8
#define MBOX_STATUS_AVAIL_MASK		0x00010000
#define MBOX_STATUS_ACK_MASK		0x00000001

/* Configuration and Status Registers */
struct slimpro_mbox_reg {
	u32 in;
	u32 din0;
	u32 din1;
	u32 rsvd1;
	u32 out;
	u32 dout0;
	u32 dout1;
	u32 rsvd2;
	u32 status;
	u32 statusmask;
};

struct slimpro_mbox_chan {
	struct device *dev;
	struct mbox_chan *chan;
	struct slimpro_mbox_reg __iomem *reg;
	int id;
	int irq;
	u32 rx_msg[3];
};

struct slimpro_mbox {
	struct mbox_controller mb_ctrl;
	struct slimpro_mbox_chan mc[MBOX_CNT];
	struct mbox_chan chans[MBOX_CNT];
};

static struct slimpro_mbox_chan *to_slimpro_mbox_chan(struct mbox_chan *chan)
{
	if (!chan || !chan->con_priv)
		return NULL;

	return (struct slimpro_mbox_chan *)chan->con_priv;
}

static void mb_chan_send_msg(struct slimpro_mbox_chan *mb_chan, u32 *msg)
{
	writel(msg[1], &mb_chan->reg->dout0);
	writel(msg[2], &mb_chan->reg->dout1);
	writel(msg[0], &mb_chan->reg->out);
}

static void mb_chan_recv_msg(struct slimpro_mbox_chan *mb_chan)
{
	mb_chan->rx_msg[1] = readl(&mb_chan->reg->din0);
	mb_chan->rx_msg[2] = readl(&mb_chan->reg->din1);
	mb_chan->rx_msg[0] = readl(&mb_chan->reg->in);
}

static void mb_chan_enable_int(struct slimpro_mbox_chan *mb_chan, u32 mask)
{
	u32 val = readl(&mb_chan->reg->statusmask);

	val &= ~mask;

	writel(val, &mb_chan->reg->statusmask);
}

static void mb_chan_disable_int(struct slimpro_mbox_chan *mb_chan, u32 mask)
{
	u32 val = readl(&mb_chan->reg->statusmask);

	val |= mask;

	writel(val, &mb_chan->reg->statusmask);
}

static int mb_chan_status_ack(struct slimpro_mbox_chan *mb_chan)
{
	u32 val = readl(&mb_chan->reg->status);

	if (val & MBOX_STATUS_ACK_MASK) {
		writel(MBOX_STATUS_ACK_MASK, &mb_chan->reg->status);
		return 1;
	}
	return 0;
}

static int mb_chan_status_avail(struct slimpro_mbox_chan *mb_chan)
{
	u32 val = readl(&mb_chan->reg->status);

	if (val & MBOX_STATUS_AVAIL_MASK) {
		mb_chan_recv_msg(mb_chan);
		writel(MBOX_STATUS_AVAIL_MASK, &mb_chan->reg->status);
		return 1;
	}
	return 0;
}

static irqreturn_t slimpro_mbox_irq(int irq, void *id)
{
	struct slimpro_mbox_chan *mb_chan = id;

	if (mb_chan_status_ack(mb_chan))
		mbox_chan_txdone(mb_chan->chan, 0);

	if (mb_chan_status_avail(mb_chan)) {
		mb_chan_recv_msg(mb_chan);
		mbox_chan_received_data(mb_chan->chan, mb_chan->rx_msg);
	}

	return IRQ_HANDLED;
}

static int slimpro_mbox_send_data(struct mbox_chan *chan, void *msg)
{
	struct slimpro_mbox_chan *mb_chan = to_slimpro_mbox_chan(chan);

	mb_chan_send_msg(mb_chan, msg);
	return 0;
}

static int slimpro_mbox_startup(struct mbox_chan *chan)
{
	struct slimpro_mbox_chan *mb_chan = to_slimpro_mbox_chan(chan);
	int rc;

	rc = devm_request_irq(mb_chan->dev, mb_chan->irq, slimpro_mbox_irq, 0,
			      MBOX_CON_NAME, mb_chan);
	if (unlikely(rc)) {
		dev_err(mb_chan->dev, "failed to register mailbox interrupt %d\n",
			mb_chan->irq);
		return rc;
	}

	/* Enable HW interrupt */
	writel(MBOX_STATUS_ACK_MASK | MBOX_STATUS_AVAIL_MASK,
		&mb_chan->reg->status);
	mb_chan_enable_int(mb_chan, MBOX_STATUS_ACK_MASK |
					MBOX_STATUS_AVAIL_MASK);
	return 0;
}

static void slimpro_mbox_shutdown(struct mbox_chan *chan)
{
	struct slimpro_mbox_chan *mb_chan = to_slimpro_mbox_chan(chan);

	mb_chan_disable_int(mb_chan, MBOX_STATUS_ACK_MASK |
				  MBOX_STATUS_AVAIL_MASK);
	devm_free_irq(mb_chan->dev, mb_chan->irq, mb_chan);
}

static struct mbox_chan_ops slimpro_mbox_ops = {
	.send_data = slimpro_mbox_send_data,
	.startup = slimpro_mbox_startup,
	.shutdown = slimpro_mbox_shutdown,
};

static int __init slimpro_mbox_probe(struct platform_device *pdev)
{
	struct slimpro_mbox *ctx;
	struct resource *regs;
	void __iomem *mb_base;
	int rc;
	int i;

	ctx = devm_kzalloc(&pdev->dev, sizeof(struct slimpro_mbox), GFP_KERNEL);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);
	platform_set_drvdata(pdev, ctx);

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mb_base = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(mb_base))
		return PTR_ERR(mb_base);

	/* Setup mailbox links */
	for (i = 0; i < MBOX_CNT; i++) {
		ctx->mc[i].irq = platform_get_irq(pdev, i);
		if (ctx->mc[i].irq < 0) {
			dev_err(&pdev->dev, "no IRQ at index %d\n",
				ctx->mc[i].irq);
			return -ENODEV;
		}

		ctx->mc[i].dev = &pdev->dev;
		ctx->mc[i].reg = mb_base + i * MBOX_REG_SET_OFFSET;
		ctx->mc[i].id = i;
		ctx->mc[i].chan = &ctx->chans[i];
		ctx->chans[i].con_priv = &ctx->mc[i];
	}

	/* Setup mailbox controller */
	ctx->mb_ctrl.dev = &pdev->dev;
	ctx->mb_ctrl.chans = ctx->chans;
	ctx->mb_ctrl.txdone_irq = true;
	ctx->mb_ctrl.ops = &slimpro_mbox_ops;
	ctx->mb_ctrl.num_chans = MBOX_CNT;

	rc = mbox_controller_register(&ctx->mb_ctrl);
	if (rc) {
		dev_err(&pdev->dev,
			"APM X-Gene SLIMpro MailBox register failed:%d\n", rc);
		return rc;
	}

	dev_info(&pdev->dev, "APM X-Gene SLIMpro MailBox registered\n");
	return 0;
}

static int slimpro_mbox_remove(struct platform_device *pdev)
{
	struct slimpro_mbox *smb = platform_get_drvdata(pdev);

	mbox_controller_unregister(&smb->mb_ctrl);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id slimpro_of_match[] = {
	{.compatible = "apm,xgene-slimpro-mbox" },
	{ },
};
MODULE_DEVICE_TABLE(of, slimpro_of_match);
#endif

#ifdef CONFIG_ACPI
static const struct acpi_device_id slimpro_acpi_ids[] = {
	{"APMC0D01", 0},
	{}
};
MODULE_DEVICE_TABLE(acpi, slimpro_acpi_ids);
#endif

static struct platform_driver slimpro_mbox_driver = {
	.probe	= slimpro_mbox_probe,
	.remove = slimpro_mbox_remove,
	.driver	= {
		.name = "xgene-slimpro-mbox",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(slimpro_of_match),
		.acpi_match_table = ACPI_PTR(slimpro_acpi_ids)
	},
};

static int __init slimpro_mbox_init(void)
{
	return platform_driver_register(&slimpro_mbox_driver);
}

static void __exit slimpro_mbox_exit(void)
{
}

subsys_initcall(slimpro_mbox_init);
module_exit(slimpro_mbox_exit);

MODULE_DESCRIPTION("APM X-Gene SLIMpro Mailbox Driver");
MODULE_LICENSE("GPL");
