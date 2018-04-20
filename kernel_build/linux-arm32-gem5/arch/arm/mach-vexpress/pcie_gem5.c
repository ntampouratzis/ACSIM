/*
 *  linux/arch/arm/mach-versatile/pci.c
 *
 * (C) Copyright Koninklijke Philips Electronics NV 2004. All rights reserved.
 * You can redistribute and/or modify this software under the terms of version 2
 * of the GNU General Public License as published by the Free Software Foundation.
 * THIS SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY; WITHOUT EVEN THE IMPLIED
 * WARRANTY OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 * Koninklijke Philips Electronics nor its subsidiaries is obligated to provide any support for this software.
 *
 * ARM Versatile PCI driver.
 *
 * 14/04/2005 Initial version, colin.king@philips.com
 *
 */
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/init.h>
#include <linux/io.h>

#include "include/mach/hardware.h"
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/mach/pci.h>

static int versatile_read_config(struct pci_bus *pbus, unsigned int devfn, int where,
				 int size, u32 *val)
{
    u32 addr, bus, slot, function;
    bus = pbus->number;
    slot = PCI_SLOT(devfn);
    function = PCI_FUNC(devfn);

    if (bus != 0) {
        switch (size) {
          case 1:
            *val = 0xff;
            break;
          case 2:
            *val = 0xffff;
            break;
          case 4:
            *val = 0xffffffff;
            break;
        }
        return PCIBIOS_DEVICE_NOT_FOUND;
    }

    addr = dt_vexpress_pci_cfg_vbase | (bus << 24) | ((slot & 0x1f) << 19) |
        ((function & 7) << 16) | where;

    switch (size) {
      case 1:
        *val = readb(addr);
        break;
      case 2:
        *val = readw(addr);
        break;
      case 4:
        *val = readl(addr);
        break;
    }
    return PCIBIOS_SUCCESSFUL;
}


static int versatile_write_config(struct pci_bus *pbus, unsigned int devfn, int where,
				  int size, u32 val)
{
    u32 addr, bus, slot, function;
    bus = pbus->number;
    slot = PCI_SLOT(devfn);
    function = PCI_FUNC(devfn);

    if (bus != 0) {
        return PCIBIOS_DEVICE_NOT_FOUND;
    }

    addr = dt_vexpress_pci_cfg_vbase | (bus << 24) | ((slot & 0x1f) << 19) |
        ((function & 7) << 16) | where;

    switch (size) {
      case 1:
        writeb(val, addr);
        break;
      case 2:
        writew(val, addr);
        break;
      case 4:
        writel(val, addr);
        break;
    }
    return PCIBIOS_SUCCESSFUL;
}



static struct pci_ops pci_versatile_ops = {
	.read	= versatile_read_config,
	.write	= versatile_write_config,
};

static struct resource vexpress_io_res = {
	.name	= "PCI I/O space",
	.start	= 0xDEADBEEF,
	.end	= 0xDEADBEEF,
	.flags	= IORESOURCE_IO,
};

static struct resource vexpress_mem_res = {
	.name	= "PCI non-prefetchable",
	.start	= 0xDEADBEEF,
	.end	= 0xDEADBEEF,
	.flags	= IORESOURCE_MEM,
};

static struct resource vexpress_memp_res = {
	.name	= "PCI prefetchable",
	.start	= 0xDEADBEEF,
	.end	= 0xDEADBEEF,
	.flags	= IORESOURCE_MEM | IORESOURCE_PREFETCH,
};


int __init pci_versatile_setup(int nr, struct pci_sys_data *sys)
{
    sys->busnr = 0;
    sys->mem_offset = 0;
    sys->io_offset = 0;

    pci_add_resource(&sys->resources, &vexpress_io_res);
    pci_add_resource(&sys->resources, &vexpress_mem_res);
    pci_add_resource(&sys->resources, &vexpress_memp_res);

    return 1;
}

struct pci_bus * __init pci_versatile_scan_bus(int nr, struct pci_sys_data *sys)
{
    struct pci_bus *root_bus = NULL;

    root_bus = pci_scan_bus(sys->busnr, &pci_versatile_ops, sys);
    pci_assign_unassigned_resources();

    return root_bus;
}

void __init pci_versatile_preinit(void)
{
        vexpress_io_res.start = dt_vexpress_pci_io_base;
        vexpress_io_res.end = dt_vexpress_pci_io_limit;

        vexpress_mem_res.start = dt_vexpress_pci_mem_base;
        vexpress_mem_res.end = dt_vexpress_pci_mem_limit;

        vexpress_memp_res.start = dt_vexpress_pci_memp_base;
        vexpress_memp_res.end = dt_vexpress_pci_memp_limit;

	pcibios_min_io = dt_vexpress_pci_io_base;
	pcibios_min_mem = dt_vexpress_pci_mem_base;
}

/*
 * map the specified device/slot/pin to an IRQ.   Different backplanes may need to modify this.
 */
static int __init versatile_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	int irq;
	int devslot = PCI_SLOT(dev->devfn);

	irq = IRQ_V2M_PCIE + pin - 1;

	printk("PCI map irq: slot %d, pin %d, devslot %d, irq: %d\n",slot,pin,devslot,irq);

	return irq;
}

static struct hw_pci versatile_pci __initdata = {
	.swizzle		= NULL,
	.map_irq		= versatile_map_irq,
	.nr_controllers		= 1,
	.setup			= pci_versatile_setup,
	.scan			= pci_versatile_scan_bus,
	.preinit		= pci_versatile_preinit,
};

static int __init versatile_pci_init(void)
{
	if (dt_vexpress_pci_cfg_base == 0)
		pr_info("Skipping PCI init\n");
	else
		pci_common_init(&versatile_pci);
	return 0;
}

subsys_initcall(versatile_pci_init);
