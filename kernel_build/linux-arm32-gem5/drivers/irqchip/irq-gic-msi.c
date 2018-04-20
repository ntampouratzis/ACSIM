/*
 * GIC MSI Extensions Driver.
 *
 * This driver provides support for up to 1 MSI per pci_dev. The functionality
 * is provided by an MSI widget which provides an MSI capture register which
 * fires SPIs. This driver hooks into the standard MSI arch callbacks and maps
 * MSI vectors to SPI interrupts.
 *
 * Copyright (C) 2012-2013 ARM Ltd.
 * Author: Andrew Murray <andrew.murray@arm.com>
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
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip/arm-gic-msi.h>
#include <linux/module.h>
#include <linux/msi.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/slab.h>

#define DEVICE_NAME "gic-msi"

#ifdef CONFIG_PCI_MSI

/* The kernel only supports a single MSI controller, thus our driver is limited
   to a single instance */
static struct gic_msi_data *gd;

int arch_setup_msi_irq(struct pci_dev *pdev, struct msi_desc *desc)
{
	int virt, vector;
	struct msi_msg msg;
	struct gic_msi_frame *frame;

	if (WARN(!gd, DEVICE_NAME ": No MSI handler"))
		return 1;

	/* Look in all the frames to find an available MSI vector/SPI */
	list_for_each_entry(frame, &gd->frames, list) {
again:
		vector = find_first_zero_bit(frame->msi_irq_in_use,
				frame->nr_msi);
		if (vector >= frame->nr_msi)
			continue;

		if (test_and_set_bit(vector, frame->msi_irq_in_use))
			goto again;

		virt = vector + frame->start_spi;

		irq_set_msi_desc(virt, desc);
		irq_set_irq_type(virt, IRQ_TYPE_EDGE_RISING);

#ifdef CONFIG_64BIT
		msg.address_hi = (phys_addr_t)frame->base_p >> 32;
#endif
		msg.address_lo = (phys_addr_t)frame->base_p + GIC_MSI_SETSPI_NSR;
		msg.data = virt;

		write_msi_msg(virt, &msg);

		return 0;
	}

	/* No free MSIs */
	WARN_ON(DEVICE_NAME ": Unable to assign another MSI");
	return 1;
}

void arch_teardown_msi_irq(unsigned int irq)
{
	struct gic_msi_frame *frame;
	int hwirq;

	if (WARN(!gd, DEVICE_NAME ": No MSI handler"))
		return;

	list_for_each_entry(frame, &gd->frames, list) {
		hwirq = irq - frame->start_spi;
		if (hwirq < frame->nr_msi) {
			clear_bit(hwirq, frame->msi_irq_in_use);
			return;
		}
	}
}
static int gic_msi_init_frame(struct resource *res)
{
	struct gic_msi_frame *frame;
	int err, x;

	frame = kzalloc(sizeof(struct gic_msi_frame), GFP_KERNEL);
	if (WARN(!frame, DEVICE_NAME ": Unable to allocate memory"))
		return -ENOMEM;

	frame->base_p = res->start;
	frame->base_sz = resource_size(res);

	if (!request_mem_region(res->start, resource_size(res), DEVICE_NAME)) {
		pr_err(DEVICE_NAME
			": Failed to request config registers resource\n");
		err = -EBUSY;
		goto err_request_mem;
	}

	frame->base_p= res->start;
	frame->base_sz= resource_size(res);
	frame->base = ioremap(res->start, resource_size(res));
	if (!frame->base) {
		pr_err(DEVICE_NAME
			": Failed to map configuration registers resource\n");
		err = -ENOMEM;
		goto err_ioremap;
	}

	x = readl(frame->base + GIC_MSI_TYPER);
	frame->start_spi = (x & GIC_MSI_TYPER_START_MASK)
				>> GIC_MSI_TYPER_START_SHIFT;
	frame->nr_msi = x & GIC_MSI_TYPER_NR_MASK;

	list_add_tail(&frame->list, &gd->frames);

	return 0;

err_ioremap:
	iounmap(frame->base);

err_request_mem:
	kfree(frame);

	return err;
}

static int gic_msi_remove_frames(void)
{
	struct gic_msi_frame *frame, *tmp;

	list_for_each_entry_safe(frame, tmp, &gd->frames, list) {
		iounmap(frame->base);
		release_mem_region(frame->base_p, frame->base_sz);
		list_del(&frame->list);
		kfree(frame);
	}
	return 0;
}

static int gic_msi_get_virt_offset(struct device_node *np, int *virt_offset)
{
	struct device_node *p;
	struct irq_domain *domain;
	unsigned int type, spec[3] = { 0, 0, 0 };
	irq_hw_number_t hwirq;
	int err;

	p = of_irq_find_parent(np);
	if (WARN(!p, DEVICE_NAME ": Unable to find interrupt parent"))
		return -EINVAL;

	domain = irq_find_host(p);
	if (WARN(!domain, DEVICE_NAME
			": Unable to find interrupt parent domain"))
		goto err;

	if (WARN(!domain->ops->xlate, DEVICE_NAME
		": Unable to translate interrupt parent domain"))
		goto err;

	err = domain->ops->xlate(domain, p, spec, sizeof(spec) / sizeof(int),
					&hwirq, &type);
	if (err) {
		pr_err(DEVICE_NAME
			": Unable to translate interrupt parent domain\n");
		goto err;

	}

	*virt_offset = irq_create_mapping(domain, hwirq);

	of_node_put(p);
	return 0;

err:
	of_node_put(p);
	return -EINVAL;
}

static int gic_msi_probe(struct platform_device *pdev)
{
	int err, x = 0;
	struct device_node *np;
	struct resource res;

	np = pdev->dev.of_node;

	if (WARN(gd, DEVICE_NAME ": MSI handler already installed"))
		return -EEXIST;

	if (WARN(!np, DEVICE_NAME ": No device tree node"))
		return -EINVAL;

	gd = kzalloc(sizeof(struct gic_msi_data), GFP_KERNEL);
	if (WARN(!gd, DEVICE_NAME ": Unable to allocate memory"))
		return -ENOMEM;

	INIT_LIST_HEAD(&gd->frames);

	err = gic_msi_get_virt_offset(np, &gd->virt_offset);
	if (err) {
		pr_err(DEVICE_NAME
			": Unable to get virtual offset\n");
		goto err_init_frame;
	}

	/* Iterate through each frame identified in the DT */
	while (!(err = of_address_to_resource(np, x++, &res))) {
		err = gic_msi_init_frame(&res);
		if (err)
			goto err_init_frame;

		break;
	}

	return 0;
err_init_frame:
	gic_msi_remove_frames();

	kfree(gd);
	gd = NULL;

	return -EINVAL;
}

/**
 * Remove is a bad idea. We cannot easily tell PCIe devices to stop
 * sending MSI's they've been allocated. In any case remove is unlikely
 * to be used.
 */
static int gic_msi_remove(struct platform_device *pdev)
{
	gic_msi_remove_frames();

	kfree(gd);
	gd = NULL;

	return 0;
}

static const struct of_device_id gic_msi_ids[] = {
	{ .compatible = "arm,gic-msi", },
};

static struct platform_driver gic_msi_driver = {
	.probe  = gic_msi_probe,
	.remove = gic_msi_remove,
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = gic_msi_ids,
	},
};

static int __init gic_msi_init(void)
{
	return platform_driver_register(&gic_msi_driver);
}

subsys_initcall(gic_msi_init);
#endif
