/*
 * include/linux/irqchip/arm-gic-msi.h
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
#ifndef __LINUX_IRQCHIP_ARM_GIC_MSI_H
#define __LINUX_IRQCHIP_ARM_GIC_MSI_H

/*
 * The hardware provides its functionality through multiple frames, each
 * frame contains an MSI capture register and maps to a specific range of
 * SPIs. The maximum number of SPIs per frame is 1024
 */
#define GIC_MSI_MAX_FRAME		1024

#define GIC_MSI_TYPER			0x8
#define GIC_MSI_SETSPI_NSR		0x40
#define GIC_MSI_TYPER_NR_MASK		0x3ff
#define GIC_MSI_TYPER_START_MASK	0x3ff0000
#define GIC_MSI_TYPER_START_SHIFT	16

struct gic_msi_frame {
	void __iomem *base;
	resource_size_t base_p;
	resource_size_t base_sz;
	int start_spi;
	int nr_msi;
	struct list_head list;
	DECLARE_BITMAP(msi_irq_in_use, GIC_MSI_MAX_FRAME);
};

struct gic_msi_data {
	struct list_head frames;
	int virt_offset;
};

#endif
