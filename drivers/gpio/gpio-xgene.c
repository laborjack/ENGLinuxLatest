/*
 * AppliedMicro X-Gene SoC GPIO Driver
 *
 * Copyright (c) 2014, Applied Micro Circuits Corporation
 * Author: Feng Kan <fkan@apm.com>.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/bitops.h>

#define GPIO_SET_MASK(x)	BIT(x + 16)

#define GPIO_SET_DR_OFFSET	0x00
#define GPIO_DATA_OFFSET	0x08

#define XGENE_MAX_GPIO_PER_BANK	16

struct xgene_gpio;

struct xgene_gpio {
	struct	device		*dev;
	struct xgene_gpio_bank	*banks;
	struct gpio_chip	chip;
	void __iomem		*base;
	spinlock_t		lock;
#ifdef CONFIG_PM
	u32			set_dr_val;
	u32			od_val;
#endif
};

static inline struct xgene_gpio *to_xgene_gpio(struct gpio_chip *chip)
{
	return container_of(chip, struct xgene_gpio, chip);
}

static int xgene_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct xgene_gpio *bank = to_xgene_gpio(gc);

	return !!(ioread32(bank->base + GPIO_DATA_OFFSET) & BIT(offset));
}

static void xgene_gpio_set(struct gpio_chip *gc, unsigned int offset, int val)
{
	struct xgene_gpio *bank = to_xgene_gpio(gc);
	unsigned long flags;
	u32 setval;

	spin_lock_irqsave(&bank->lock, flags);

	setval = ioread32(bank->base + GPIO_SET_DR_OFFSET);
	if (val)
		setval |= GPIO_SET_MASK(offset);
	else
		setval &= ~GPIO_SET_MASK(offset);
	iowrite32(setval, bank->base + GPIO_SET_DR_OFFSET);

	spin_unlock_irqrestore(&bank->lock, flags);
}

static int xgene_gpio_dir_in(struct gpio_chip *gc, unsigned int offset)
{
	struct xgene_gpio *bank = to_xgene_gpio(gc);
	unsigned long flags;
	u32 dirval;

	spin_lock_irqsave(&bank->lock, flags);

	dirval = ioread32(bank->base + GPIO_SET_DR_OFFSET);
	dirval |= BIT(offset);
	iowrite32(dirval, bank->base + GPIO_SET_DR_OFFSET);

	spin_unlock_irqrestore(&bank->lock, flags);

	return 0;
}

static int xgene_gpio_dir_out(struct gpio_chip *gc,
					unsigned int offset, int val)
{
	struct xgene_gpio *bank = to_xgene_gpio(gc);
	unsigned long flags;
	u32 dirval;

	spin_lock_irqsave(&bank->lock, flags);

	dirval = ioread32(bank->base + GPIO_SET_DR_OFFSET);
	dirval &= ~BIT(offset);
	iowrite32(dirval, bank->base + GPIO_SET_DR_OFFSET);

	spin_unlock_irqrestore(&bank->lock, flags);

	return 0;
}

#ifdef CONFIG_PM
static int xgene_gpio_suspend(struct device *dev)
{
	struct xgene_gpio *gpio = dev_get_drvdata(dev);

	gpio->set_dr_val = ioread32(gpio->base + GPIO_SET_DR_OFFSET);
	return 0;
}

static int xgene_gpio_resume(struct device *dev)
{
	struct xgene_gpio *gpio = dev_get_drvdata(dev);

	iowrite32(gpio->set_dr_val, gpio->base + GPIO_SET_DR_OFFSET);
	return 0;
}

static SIMPLE_DEV_PM_OPS(xgene_gpio_pm, xgene_gpio_suspend, xgene_gpio_resume);
#define XGENE_GPIO_PM_OPS	(&xgene_gpio_pm)
#else
#define XGENE_GPIO_PM_OPS	NULL
#endif

static int xgene_gpio_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct resource *res;
	struct xgene_gpio *gpio;
	int err = 0;
	unsigned int bank = 0, ngpio = 0;

	gpio = devm_kzalloc(&pdev->dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio) {
		err = -ENOMEM;
		goto err;
	}
	gpio->dev = &pdev->dev;

	gpio->banks = devm_kzalloc(&pdev->dev,
				   sizeof(struct xgene_gpio), GFP_KERNEL);
	if (!gpio->banks) {
		err = -ENOMEM;
		goto err;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	gpio->base = devm_ioremap_nocache(&pdev->dev, res->start,
							resource_size(res));
	if (IS_ERR(gpio->base)) {
		err = PTR_ERR(gpio->base);
		goto err;
	}

	/*
	 * Determine gpio bank using gpio resource address
	 */
	bank = (res->start & 0xff) / 0xc - 1;
	if (bank > 2) {
		dev_err(gpio->dev, "incorrect gpio base specified\n");
		goto err;
	}

	ngpio = XGENE_MAX_GPIO_PER_BANK;
	gpio->chip.ngpio = ngpio;
	gpio->chip.of_node = np;
	gpio->chip.base = bank * ngpio;
	gpio->chip.label = dev_name(&pdev->dev);

	gpio->chip.direction_input = xgene_gpio_dir_in;
	gpio->chip.direction_output = xgene_gpio_dir_out;
	gpio->chip.get = xgene_gpio_get;
	gpio->chip.set = xgene_gpio_set;

	platform_set_drvdata(pdev, gpio);

	err = gpiochip_add(&gpio->chip);
	if (err) {
		dev_err(gpio->dev,
			"failed to register gpiochip for bank%d\n", bank);
		goto err;
	}

	dev_info(&pdev->dev, "X-Gene GPIO driver registered\n");
	return 0;
err:
	dev_err(&pdev->dev, "X-Gene GPIO driver registration failed\n");
	return err;
}

#ifdef CONFIG_OF
static const struct of_device_id xgene_gpio_of_match[] = {
	{ .compatible = "apm,xgene-gpio", },
	{},
};
MODULE_DEVICE_TABLE(of, xgene_gpio_of_match);
#endif

static struct platform_driver xgene_gpio_driver = {
	.driver = {
		.name = "xgene-gpio",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(xgene_gpio_of_match),
		.pm     = XGENE_GPIO_PM_OPS,
	},
	.probe = xgene_gpio_probe,
};

module_platform_driver(xgene_gpio_driver);

MODULE_AUTHOR("Feng Kan <fkan@apm.com>");
MODULE_DESCRIPTION("APM X-Gene GPIO driver");
MODULE_LICENSE("GPL");
