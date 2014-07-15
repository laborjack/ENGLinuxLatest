/*
 * Arasan Secure Digital Host Controller Interface.
 * Copyright (C) 2011 - 2012 Michal Simek <monstr@monstr.eu>
 * Copyright (c) 2012 Wind River Systems, Inc.
 * Copyright (C) 2013 Pengutronix e.K.
 * Copyright (C) 2013 Xilinx Inc.
 *
 * Based on sdhci-of-esdhc.c
 *
 * Copyright (c) 2007 Freescale Semiconductor, Inc.
 * Copyright (c) 2009 MontaVista Software, Inc.
 *
 * Authors: Xiaobo Xie <X.Xie@freescale.com>
 *	    Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/of_device.h>
#include "sdhci-pltfm.h"

#define SDHCI_ARASAN_CLK_CTRL_OFFSET	0x2c

#define CLK_CTRL_TIMEOUT_SHIFT		16
#define CLK_CTRL_TIMEOUT_MASK		(0xf << CLK_CTRL_TIMEOUT_SHIFT)
#define CLK_CTRL_TIMEOUT_MIN_EXP	13

/**
 * struct sdhci_arasan_data
 * @clk_ahb:	Pointer to the AHB clock
 */
struct sdhci_arasan_data {
	struct clk	*clk_ahb;
	struct platform_device *pdev;
	void __iomem	*ahb_aim_csr;
	const struct sdhci_arasan_ahb_ops *ahb_ops;
};

/**
 * struct sdhci_arasan_ahb_ops
 * @init_ahb	Initialize translation bus
 * @xlat_addr	Set up an 64-bit addressing translation
 */
struct sdhci_arasan_ahb_ops {
	int (*init_ahb)(struct sdhci_arasan_data *data);
	void (*xlat_addr)(struct sdhci_arasan_data *data, u64 dma_addr);
};

static unsigned int sdhci_arasan_get_timeout_clock(struct sdhci_host *host)
{
	u32 div;
	unsigned long freq;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);

	div = readl(host->ioaddr + SDHCI_ARASAN_CLK_CTRL_OFFSET);
	div = (div & CLK_CTRL_TIMEOUT_MASK) >> CLK_CTRL_TIMEOUT_SHIFT;

	freq = clk_get_rate(pltfm_host->clk);
	freq /= 1 << (CLK_CTRL_TIMEOUT_MIN_EXP + div);

	return freq;
}

static void sdhci_arasan_writel(struct sdhci_host *host, u32 val, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_arasan_data *sdhci_arasan = pltfm_host->priv;

	if (reg == SDHCI_DMA_ADDRESS) {
		if (sdhci_arasan->ahb_ops && sdhci_arasan->ahb_ops->xlat_addr)
			sdhci_arasan->ahb_ops->xlat_addr(sdhci_arasan,
				sg_dma_address(host->data->sg));
	}
	writel(val, host->ioaddr + reg);
}

static struct sdhci_ops sdhci_arasan_ops = {
	.write_l = sdhci_arasan_writel,
	.set_clock = sdhci_set_clock,
	.get_max_clock = sdhci_pltfm_clk_get_max_clock,
	.get_timeout_clock = sdhci_arasan_get_timeout_clock,
	.set_bus_width = sdhci_set_bus_width,
	.reset = sdhci_reset,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
};

static struct sdhci_pltfm_data sdhci_arasan_pdata = {
	.ops = &sdhci_arasan_ops,
};

#ifdef CONFIG_PM_SLEEP
/**
 * sdhci_arasan_suspend - Suspend method for the driver
 * @dev:	Address of the device structure
 * Returns 0 on success and error value on error
 *
 * Put the device in a low power state.
 */
static int sdhci_arasan_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_arasan_data *sdhci_arasan = pltfm_host->priv;
	int ret;

	ret = sdhci_suspend_host(host);
	if (ret)
		return ret;

	clk_disable(pltfm_host->clk);
	clk_disable(sdhci_arasan->clk_ahb);

	return 0;
}

/**
 * sdhci_arasan_resume - Resume method for the driver
 * @dev:	Address of the device structure
 * Returns 0 on success and error value on error
 *
 * Resume operation after suspend
 */
static int sdhci_arasan_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_arasan_data *sdhci_arasan = pltfm_host->priv;
	int ret;

	ret = clk_enable(sdhci_arasan->clk_ahb);
	if (ret) {
		dev_err(dev, "Cannot enable AHB clock.\n");
		return ret;
	}

	ret = clk_enable(pltfm_host->clk);
	if (ret) {
		dev_err(dev, "Cannot enable SD clock.\n");
		clk_disable(sdhci_arasan->clk_ahb);
		return ret;
	}

	return sdhci_resume_host(host);
}
#endif /* ! CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(sdhci_arasan_dev_pm_ops, sdhci_arasan_suspend,
			 sdhci_arasan_resume);

static int sdhci_arasan_xgene_init_ahb(struct sdhci_arasan_data *data)
{
	#define AIM_SIZE_CTL_OFFSET	0x00000004
	#define  AIM_EN_N_WR(src)	(((u32) (src) << 31) & 0x80000000)
	#define  ARSB_WR(src)		(((u32) (src) << 24) & 0x0f000000)
	#define  AWSB_WR(src)		(((u32) (src) << 20) & 0x00f00000)
	#define  AIM_MASK_N_WR(src)	(((u32) (src)) & 0x000fffff)

	struct sdhci_host *host = platform_get_drvdata(data->pdev);
	int ret;

	if (!data->ahb_aim_csr)
		return 0;

	/*
	 * Setup AHB AIM windows ctrl register. The lower 32-bit is left
	 * at 0 while the upper bit are programmed when the buffer address
	 ( is set from function sdhci_arasn_writel.
	 */
	writel(AIM_EN_N_WR(1) | ARSB_WR(1) | AWSB_WR(1) | AIM_MASK_N_WR(0),
	       data->ahb_aim_csr + AIM_SIZE_CTL_OFFSET);

	/* Set DMA mask */
	ret = dma_set_mask_and_coherent(&data->pdev->dev, DMA_BIT_MASK(64));
	if (ret) {
		dev_err(&data->pdev->dev, "Unable to set dma mask\n");
		return ret;
	}

	/*
	 * This shouldn't be necessary. Just in case the FW doesn't
	 * configure disable ADMA support as we can't support multiple
	 * DMA buffer whose address is 64-bit. The AHB translation bridge
	 * only has 8 entry max and that is required to be shared and
	 * upper layer can pass more than 8 buffer pointers.
	 */
	host->quirks |= SDHCI_QUIRK_BROKEN_ADMA;

	return 0;
}

static void sdhci_arasn_xgene_xlat_addr(struct sdhci_arasan_data *data,
				       u64 dma_addr)
{
	#define AIM_AXI_HI_OFFSET	0x0000000c
	#define  AIM_AXI_ADDRESS_HI_N_WR(src) \
					(((u32) (src) << 20) & 0xfff00000)

	if (!data->ahb_aim_csr)
		return;

	writel(AIM_AXI_ADDRESS_HI_N_WR(dma_addr >> 32),
		data->ahb_aim_csr + AIM_AXI_HI_OFFSET);
}

static const struct sdhci_arasan_ahb_ops xgene_ahb_ops = {
	.init_ahb = sdhci_arasan_xgene_init_ahb,
	.xlat_addr = sdhci_arasn_xgene_xlat_addr,
};

static const struct of_device_id sdhci_arasan_of_match[] = {
	{ .compatible = "arasan,sdhci-8.9a" },
	{ .compatible = "apm,arasan,sdhci-8.9a", .data = &xgene_ahb_ops },
	{ }
};
MODULE_DEVICE_TABLE(of, sdhci_arasan_of_match);

static int sdhci_arasan_probe(struct platform_device *pdev)
{
	int ret;
	struct clk *clk_xin = NULL;
	struct sdhci_host *host;
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_arasan_data *sdhci_arasan;
	const struct of_device_id *of_id =
			of_match_device(sdhci_arasan_of_match, &pdev->dev);
	struct resource *res;

	sdhci_arasan = devm_kzalloc(&pdev->dev, sizeof(*sdhci_arasan),
			GFP_KERNEL);
	if (!sdhci_arasan)
		return -ENOMEM;

	sdhci_arasan->clk_ahb = devm_clk_get(&pdev->dev, "clk_ahb");
	if (IS_ERR(sdhci_arasan->clk_ahb)) {
		/* Clock is optional */
		sdhci_arasan->clk_ahb = NULL;
		goto skip_clk;
	}

	clk_xin = devm_clk_get(&pdev->dev, "clk_xin");
	if (IS_ERR(clk_xin)) {
		dev_err(&pdev->dev, "clk_xin clock not found.\n");
		return PTR_ERR(clk_xin);
	}

	ret = clk_prepare_enable(sdhci_arasan->clk_ahb);
	if (ret) {
		dev_err(&pdev->dev, "Unable to enable AHB clock.\n");
		return ret;
	}

	ret = clk_prepare_enable(clk_xin);
	if (ret) {
		dev_err(&pdev->dev, "Unable to enable SD clock.\n");
		goto clk_dis_ahb;
	}
skip_clk:

	host = sdhci_pltfm_init(pdev, &sdhci_arasan_pdata, 0);
	if (IS_ERR(host)) {
		ret = PTR_ERR(host);
		dev_err(&pdev->dev, "platform init failed (%u)\n", ret);
		goto clk_disable_all;
	}

	sdhci_get_of_property(pdev);
	pltfm_host = sdhci_priv(host);
	pltfm_host->priv = sdhci_arasan;
	pltfm_host->clk = clk_xin;

	/* Retrieval optional AHB translation memory resource */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	sdhci_arasan->ahb_aim_csr = devm_ioremap(&pdev->dev, res->start,
						 resource_size(res));

	sdhci_arasan->pdev = pdev;
	sdhci_arasan->ahb_ops = of_id->data;
	if (sdhci_arasan->ahb_ops && sdhci_arasan->ahb_ops->init_ahb) {
		ret = sdhci_arasan->ahb_ops->init_ahb(sdhci_arasan);
		if (ret)
			goto err_pltfm_free;
	}

	ret = sdhci_add_host(host);
	if (ret) {
		dev_err(&pdev->dev, "platform register failed (%u)\n", ret);
		goto err_pltfm_free;
	}

	return 0;

err_pltfm_free:
	sdhci_pltfm_free(pdev);
clk_disable_all:
	clk_disable_unprepare(clk_xin);
clk_dis_ahb:
	clk_disable_unprepare(sdhci_arasan->clk_ahb);

	return ret;
}

static int sdhci_arasan_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_arasan_data *sdhci_arasan = pltfm_host->priv;

	clk_disable_unprepare(pltfm_host->clk);
	clk_disable_unprepare(sdhci_arasan->clk_ahb);

	return sdhci_pltfm_unregister(pdev);
}

static struct platform_driver sdhci_arasan_driver = {
	.driver = {
		.name = "sdhci-arasan",
		.owner = THIS_MODULE,
		.of_match_table = sdhci_arasan_of_match,
		.pm = &sdhci_arasan_dev_pm_ops,
	},
	.probe = sdhci_arasan_probe,
	.remove = sdhci_arasan_remove,
};

module_platform_driver(sdhci_arasan_driver);

MODULE_DESCRIPTION("Driver for the Arasan SDHCI Controller");
MODULE_AUTHOR("Soeren Brinkmann <soren.brinkmann@xilinx.com>");
MODULE_LICENSE("GPL");
