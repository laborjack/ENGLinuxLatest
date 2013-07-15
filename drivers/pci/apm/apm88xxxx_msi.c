/*
 * APM MSI Driver
 *
 * Copyright (c) 2010, Applied Micro Circuits Corporation
 * Author: Tanmay Inamdar <tinamdar@apm.com>
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
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 */

#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/bootmem.h>
#include <linux/msi.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/export.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <asm/prom.h>
#include <asm/hw_irq.h>
#include <asm/io.h>
#include "apm88xxxx_msi.h"

#define MSI_DRIVER_VERSION "0.1"
LIST_HEAD(msi_head);

struct apm_msi_feature {
	u32 apm_pic_ip;
	u32 msiir_offset; /* Offset of MSIIR, relative to start of MSIR bank */
};

struct apm_msi_cascade_data {
	struct apm_msi *msi_data;
	int index;
};

irq_hw_number_t virq_to_hw(unsigned int virq)
{
	struct irq_data *irq_data = irq_get_irq_data(virq);
	return WARN_ON(!irq_data) ? 0 : irq_data->hwirq;
} 

static inline u32 apm_msi_intr_read(u64 __iomem *base, unsigned int reg)
{
	u32 irq_reg = MSI1INT0 + (reg << 4);
	msi_dbg("base = %p irq_reg = 0x%x , reg = 0x%x\n", base, irq_reg, reg);
	return readl((void *)((u64)base + irq_reg));
}

static inline u32 apm_msi_read(u64 __iomem *base, unsigned int reg)
{
	u32 irq_reg = MSI0IR0 + (reg << 4);
	msi_dbg("base = %p irq_reg = 0x%x , reg = 0x%x\n", base, irq_reg, reg);
	return readl(base + irq_reg);
}

/*
 * We do not need this actually. The MSIR register has been read once
 * in the cascade interrupt. So, this MSI interrupt has been acked
*/
static void apm_msi_end_irq(struct irq_data *d)
{
}

static struct irq_chip apm_msi_chip = {
	.irq_mask	= mask_msi_irq,
	.irq_unmask	= unmask_msi_irq,
	.irq_ack	= apm_msi_end_irq,
	.name		= "apm-msi",
};

static int apm_msi_host_map(struct irq_domain *h, unsigned int virq,
			    irq_hw_number_t hw)
{
	struct apm_msi *msi_data = h->host_data;
	struct irq_chip *chip = &apm_msi_chip;

	msi_dbg("ENTER\n");
	irq_set_status_flags(virq, IRQ_TYPE_EDGE_RISING);

	irq_set_chip_data(virq, msi_data);
	irq_set_chip_and_handler(virq, chip, handle_edge_irq);

	return 0;
}

static const struct irq_domain_ops apm_msi_host_ops = {
	.map = apm_msi_host_map,
};

static int apm_msi_init_allocator(struct apm_msi *msi_data)
{
	int rc;

	rc = msi_bitmap_alloc(&msi_data->bitmap, NR_MSI_IRQS,
			      msi_data->irqhost->of_node);
	if (rc)
		return rc;

#if 0
	/* FIXME */
	rc = msi_bitmap_reserve_dt_hwirqs(&msi_data->bitmap);
	if (rc < 0) {
		msi_bitmap_free(&msi_data->bitmap);
		return rc;
	}
#endif

	return 0;
}

int arch_msi_check_device(struct pci_dev *pdev, int nvec, int type)
{
	msi_dbg("ENTER\n");
	if (type == PCI_CAP_ID_MSIX) {
		msi_dbg("apmmsi: MSI-X Not Present\n");
		return -EINVAL;
	}

	return 0;
}

void arch_teardown_msi_irqs(struct pci_dev *pdev)
{
	struct msi_desc *entry;
	struct apm_msi *msi_data;

	msi_dbg("ENTER\n");
	list_for_each_entry(entry, &pdev->msi_list, list) {
		if (entry->irq == NO_IRQ)
			continue;
		msi_data = irq_get_chip_data(entry->irq);
		irq_set_msi_desc(entry->irq, NULL);
		msi_bitmap_free_hwirqs(&msi_data->bitmap,
				       virq_to_hw(entry->irq), 1);
		irq_dispose_mapping(entry->irq);
	}

	return;
}

static void apm_compose_msi_msg(struct pci_dev *pdev, int hwirq,
				struct msi_msg *msg,
				struct apm_msi *msi_data)
{
	int reg_set;

	reg_set = hwirq / IRQS_PER_MSI_REG;
	msi_dbg("reg_set : 0x%x\n", reg_set);
	msg->address_lo = msi_data->msi_addr_lo + (reg_set << 4);
	msg->address_hi = msi_data->msi_addr_hi;
	msg->data = hwirq;

	msi_dbg("allocated srs: %d, ibs: %d\n",
		hwirq / IRQS_PER_MSI_REG, hwirq % IRQS_PER_MSI_REG);
}

int arch_setup_msi_irqs(struct pci_dev *pdev, int nvec, int type)
{
	struct device_node *np = pci_device_to_OF_node(pdev);
	phandle phandle = 0;
	int rc, hwirq = -ENOMEM;
	unsigned int virq;
	struct msi_desc *entry;
	struct msi_msg msg;
	struct apm_msi *msi_data;

	msi_dbg("ENTER\n");
	/*
	 * If the PCI node has an apm,msi property, then we need to use it
	 * to find the specific MSI.
	 */
	np = of_parse_phandle(np, "apm,msi", 0);
	if (np) {
		if (of_device_is_compatible(np, "apm,gic-msi-cascade")) 
			phandle = np->phandle;
		else {
			dev_err(&pdev->dev,
				"node %s has an invalid apm,msi phandle %u\n",
				np->full_name, np->phandle);
			return -EINVAL;
		}
	}

	list_for_each_entry(entry, &pdev->msi_list, list) {
		msi_dbg("Loop over MSI devices\n");
		/*
		 * Loop over all the MSI devices until we find one that has an
		 * available interrupt.
		 */
		list_for_each_entry(msi_data, &msi_head, list) {
			if (phandle && (phandle != msi_data->phandle))
				continue;
			msi_dbg("apm_msi : %p\n", msi_data);
			hwirq = msi_bitmap_alloc_hwirqs(&msi_data->bitmap, 1);
			if (hwirq >= 0)
				break;
		}

		if (hwirq < 0) {
			rc = hwirq;
			dev_err(&pdev->dev, "could not allocate MSI interrupt\n");
			goto out_free;
		}

		virq = irq_create_mapping(msi_data->irqhost, hwirq);
		if (virq == NO_IRQ) {
			dev_err(&pdev->dev, "fail mapping hwirq %i\n", hwirq);
			msi_bitmap_free_hwirqs(&msi_data->bitmap, hwirq, 1);
			rc = -ENOSPC;
			goto out_free;
		}
		msi_dbg("Created Mapping HWIRQ %d TO VIRQ %d\n", hwirq, virq);
		/* chip_data is msi_data via host->hostdata in host->map() */
		irq_set_msi_desc(virq, entry);
		apm_compose_msi_msg(pdev, hwirq, &msg, msi_data);
		write_msi_msg(virq, &msg);
	}
	return 0;

out_free:
	/* free by the caller of this function */
	msi_dbg("EXIT with error %d\n", rc);
	return rc;
}

static void apm_msi_cascade(unsigned int irq, struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct irq_data *idata = irq_desc_get_irq_data(desc);
	unsigned int cascade_irq;
	struct apm_msi *msi_data;
	int msir_index = -1;
	u32 msir_value = 0;
	u32 intr_index = 0;
	u32 msi_intr_reg_value = 0;
	u32 msi_intr_reg;
	struct apm_msi_cascade_data *cascade_data;

	msi_dbg("ENTER\n");
	cascade_data = irq_get_handler_data(irq);
	msi_data = cascade_data->msi_data;
	msi_dbg("apm_msi : 0x%p\n", msi_data);

	raw_spin_lock(&desc->lock);
	if (unlikely(irqd_irq_inprogress(idata)))
		goto unlock;

	msi_intr_reg = cascade_data->index;
	msi_dbg("msi_intr_reg : %d\n", msi_intr_reg);

	if (msi_intr_reg >= NR_MSI_REG)
		cascade_irq = NO_IRQ;

	irqd_set_chained_irq_inprogress(idata);
	switch (msi_data->feature & APM_PIC_IP_MASK) {
	case APM_PIC_IP_GIC:
		msi_intr_reg_value = apm_msi_intr_read(msi_data->msi_regs,
						       msi_intr_reg);
		msi_dbg("msi_intr_reg_value : 0x%08x\n", msi_intr_reg_value);
		break;
	}

	while (msi_intr_reg_value) {
		msir_index = ffs(msi_intr_reg_value) - 1;
		msi_dbg("msir_index : %d\n", msir_index);
		msir_value = apm_msi_read(msi_data->msi_regs, intr_index);
		while (msir_value) {
			intr_index = ffs(msir_value) - 1;
			msi_dbg("intr_index : %d\n", intr_index);
			cascade_irq = irq_linear_revmap(msi_data->irqhost,
				msir_index * IRQS_PER_MSI_REG +	intr_index);
			msi_dbg("cascade_irq : %d\n", cascade_irq);
			if (cascade_irq != NO_IRQ)
				generic_handle_irq(cascade_irq);
			msir_value = msir_value >> (intr_index + 1);
			msi_dbg("msir_value : 0x%08x\n", msir_value);
		}
		msi_intr_reg_value = msi_intr_reg_value >> (msir_index + 1);
		msi_dbg("msi_intr_reg_value : 0x%08x\n", msi_intr_reg_value);
	}
	irqd_clr_chained_irq_inprogress(idata);

	switch (msi_data->feature & APM_PIC_IP_MASK) {
	case APM_PIC_IP_GIC:
		chip->irq_eoi(idata);
		break;
	}
unlock:
	raw_spin_unlock(&desc->lock);
	msi_dbg("EXIT\n");
}

static int apm_of_msi_remove(struct platform_device *ofdev)
{
	struct apm_msi *msi = platform_get_drvdata(ofdev);
	int virq, i;
	struct apm_msi_cascade_data *cascade_data;

	msi_dbg("ENTER\n");
	if (msi->list.prev != NULL)
		list_del(&msi->list);
	for (i = 0; i < NR_MSI_REG; i++) {
		virq = msi->msi_virqs[i];
		if (virq != NO_IRQ) {
			cascade_data = irq_get_handler_data(virq);
			kfree(cascade_data);
			irq_dispose_mapping(virq);
		}
	}
	if (msi->bitmap.bitmap)
		msi_bitmap_free(&msi->bitmap);

	iounmap(msi->msi_regs);
	kfree(msi);

	return 0;
}

static int apm_msi_setup_hwirq(struct apm_msi *msi,
				struct platform_device *dev,
				int offset, int irq_index)
{
	struct apm_msi_cascade_data *cascade_data = NULL;
	int virt_msir;

	virt_msir = irq_of_parse_and_map(dev->dev.of_node, irq_index);
	if (virt_msir == NO_IRQ) {
		dev_err(&dev->dev, "%s: Cannot translate IRQ index %d\n",
			__func__, irq_index);
		return 0;
	}
	msi_dbg("mapped virt irq %d\n", virt_msir);

	cascade_data = kzalloc(sizeof(struct apm_msi_cascade_data), GFP_KERNEL);
	if (!cascade_data) {
		dev_err(&dev->dev, "No memory for MSI cascade data\n");
		return -ENOMEM;
	}

	msi->msi_virqs[irq_index] = virt_msir;
	cascade_data->index = offset;
	cascade_data->msi_data = msi;
	irq_set_handler_data(virt_msir, cascade_data);
	irq_set_chained_handler(virt_msir, apm_msi_cascade);

	return 0;
}

static const struct of_device_id apm_of_msi_ids[];
static int apm_of_msi_probe(struct platform_device *dev)
{
	const struct of_device_id *match;
	struct apm_msi *msi;
	struct resource res;
	int err, j, irq_index, count;
	int rc;
	const struct apm_msi_feature *features;
	u32 offset;
	u32 p[] = { 0, NR_MSI_IRQS};

	match = of_match_device(apm_of_msi_ids, &dev->dev);
	if (!match)
		return -EINVAL;
	features = match->data;

	msi = kzalloc(sizeof(struct apm_msi), GFP_KERNEL);
	if (!msi) {
		dev_err(&dev->dev, "No memory for MSI structure\n");
		return -ENOMEM;
	}
	msi_dbg("apm_msi : %p\n", msi);
	platform_set_drvdata(dev, msi);

	msi->irqhost = irq_domain_add_linear(dev->dev.of_node,
				      NR_MSI_IRQS, &apm_msi_host_ops, msi);

	if (msi->irqhost == NULL) {
		dev_err(&dev->dev, "No memory for MSI irqhost\n");
		err = -ENOMEM;
		goto error_out;
	}

	err = of_address_to_resource(dev->dev.of_node, 0, &res);
	if (err) {
		dev_err(&dev->dev, "invalid resource for node %s\n",
				dev->dev.of_node->full_name);
		goto error_out;
	}

	msi->msi_regs = ioremap_nocache(res.start, resource_size(&res));
	if (!msi->msi_regs) {
		err = -ENOMEM;
		dev_err(&dev->dev, "could not map node %s\n",
				dev->dev.of_node->full_name);
		goto error_out;
	}
	msi_dbg("mapped 0x%08llx to 0x%p Size : 0x%08x\n", res.start,
		 msi->msi_regs, resource_size(&res));
	msi->msiir_offset = features->msiir_offset + (res.start & 0xfffff);
	msi->msi_addr_hi = res.start >> 32;
	msi->msi_addr_lo = features->msiir_offset + (res.start & 0xffffffff);
	msi->feature = features->apm_pic_ip;
	msi->phandle = dev->dev.of_node->phandle;

	rc = apm_msi_init_allocator(msi);
	if (rc) {
		dev_err(&dev->dev, "Error allocating MSI bitmap\n");
		goto error_out;
	}

	rc = of_property_read_u32_array(dev->dev.of_node,
					"msi-available-ranges", p, 2);
	if (rc) {
		dev_err(&dev->dev, "Error getting MSI ranges\n");
		goto error_out;
	}

	msi_dbg("p[0] = 0x%x p[1] = 0x%x\n", p[0], p[1]);
	if (p[0] % IRQS_PER_MSI_REG || p[1] % IRQS_PER_MSI_REG) {
		printk( KERN_WARNING "%s: %s: msi available range of"
				"%u at %u is not IRQ-aligned\n",
				__func__, dev->dev.of_node->full_name,
				p[1], p[0]);
		err = -EINVAL;
		goto error_out;
	}

	offset = p[0] / IRQS_PER_MSI_REG;
	count  = p[1] / IRQS_PER_MSI_REG;
	msi_dbg("offset = %d count = %d\n", offset, count);

	for (irq_index = 0, j = 0; j < count; j++, irq_index++) {
		err = apm_msi_setup_hwirq(msi, dev, offset + j, irq_index);
		if (err)
			goto error_out;
	}

	list_add_tail(&msi->list, &msi_head);
	printk(KERN_INFO "APM88xxxx: PCIe MSI driver v%s\n", MSI_DRIVER_VERSION);

	msi_dbg("EXIT\n");
	return 0;
error_out:
	apm_of_msi_remove(dev);
	return err;
}

static const struct apm_msi_feature gic_msi_feature = {
	.apm_pic_ip = APM_PIC_IP_GIC,
	.msiir_offset = 0,
};

static const struct of_device_id apm_of_msi_ids[] = {
	{
		.compatible = "apm,gic-msi",
		.data = (void *)&gic_msi_feature,
	},
	{}
};

static struct platform_driver apm_of_msi_driver = {
	.driver = {
		.name = "apm-msi",
		.owner = THIS_MODULE,
		.of_match_table = apm_of_msi_ids,
	},
	.probe = apm_of_msi_probe,
	.remove = apm_of_msi_remove,
};

static __init int apm_of_msi_init(void)
{
	return platform_driver_register(&apm_of_msi_driver);
}

subsys_initcall_sync(apm_of_msi_init);
