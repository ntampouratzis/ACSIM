/*
 * Versatile Express V2M Motherboard Support
 */
#include <linux/device.h>
#include <linux/amba/bus.h>
#include <linux/amba/mmci.h>
#include <linux/io.h>
#include <linux/smp.h>
#include <linux/init.h>
#include <linux/memblock.h>
#include <linux/of_address.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/ata_platform.h>
#include <linux/smsc911x.h>
#include <linux/spinlock.h>
#include <linux/usb/isp1760.h>
#include <linux/mtd/physmap.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>
#include <linux/vexpress.h>
#include <linux/clkdev.h>

#include <asm/mach-types.h>
#include <asm/sizes.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include <asm/hardware/arm_timer.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/hardware/timer-sp.h>

#include <mach/ct-ca9x4.h>
#include <mach/motherboard.h>

#include <plat/sched_clock.h>
#include <plat/platsmp.h>

#include "core.h"

#define V2M_PA_CS0	0x40000000
#define V2M_PA_CS1	0x44000000
#define V2M_PA_CS2	0x48000000
#define V2M_PA_CS3	0x4c000000
#define V2M_PA_CS7	0x10000000

#define SPACE_CODE_BIT_LOCN 24
#define SPACE_CODE_WIDTH 0x3

u32 dt_vexpress_pci_cfg_base = 0;
u32 dt_vexpress_pci_cfg_size;
u32 dt_vexpress_pci_cfg_limit;
u32 dt_vexpress_pci_cfg_vbase;
u32 dt_vexpress_pci_mem_vbase;
u32 dt_vexpress_pci_memp_vbase;
u32 dt_vexpress_pci_io_vbase;
u32 dt_vexpress_pci_mem_base;
u32 dt_vexpress_pci_memp_base;
u32 dt_vexpress_pci_io_base;
u32 dt_vexpress_pci_mem_size;
u32 dt_vexpress_pci_memp_size;
u32 dt_vexpress_pci_io_size;
u32 dt_vexpress_pci_mem_limit;
u32 dt_vexpress_pci_memp_limit;
u32 dt_vexpress_pci_io_limit;

u32 pci_convert_base_to_vbase(u32 a) {
    return (a - dt_vexpress_pci_io_base + dt_vexpress_pci_io_vbase);
}

static struct map_desc v2m_io_desc[] __initdata = {
	{
		.virtual	= V2M_PERIPH,
		.pfn		= __phys_to_pfn(V2M_PA_CS7),
		.length		= SZ_128K,
		.type		= MT_DEVICE,
	},
};

static void __init v2m_sp804_init(void __iomem *base, unsigned int irq)
{
	if (WARN_ON(!base || irq == NO_IRQ))
		return;

	sp804_clocksource_init(base + TIMER_2_BASE, "v2m-timer1");
	sp804_clockevents_init(base + TIMER_1_BASE, irq, "v2m-timer0");
}


static struct resource v2m_pcie_i2c_resource = {
	.start	= V2M_SERIAL_BUS_PCI,
	.end	= V2M_SERIAL_BUS_PCI + SZ_4K - 1,
	.flags	= IORESOURCE_MEM,
};

static struct platform_device v2m_pcie_i2c_device = {
	.name		= "versatile-i2c",
	.id		= 0,
	.num_resources	= 1,
	.resource	= &v2m_pcie_i2c_resource,
};

static struct resource v2m_ddc_i2c_resource = {
	.start	= V2M_SERIAL_BUS_DVI,
	.end	= V2M_SERIAL_BUS_DVI + SZ_4K - 1,
	.flags	= IORESOURCE_MEM,
};

static struct platform_device v2m_ddc_i2c_device = {
	.name		= "versatile-i2c",
	.id		= 1,
	.num_resources	= 1,
	.resource	= &v2m_ddc_i2c_resource,
};

static struct resource v2m_eth_resources[] = {
	{
		.start	= V2M_LAN9118,
		.end	= V2M_LAN9118 + SZ_64K - 1,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_V2M_LAN9118,
		.end	= IRQ_V2M_LAN9118,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct smsc911x_platform_config v2m_eth_config = {
	.flags		= SMSC911X_USE_32BIT,
	.irq_polarity	= SMSC911X_IRQ_POLARITY_ACTIVE_HIGH,
	.irq_type	= SMSC911X_IRQ_TYPE_PUSH_PULL,
	.phy_interface	= PHY_INTERFACE_MODE_MII,
};

static struct platform_device v2m_eth_device = {
	.name		= "smsc911x",
	.id		= -1,
	.resource	= v2m_eth_resources,
	.num_resources	= ARRAY_SIZE(v2m_eth_resources),
	.dev.platform_data = &v2m_eth_config,
};

static struct regulator_consumer_supply v2m_eth_supplies[] = {
	REGULATOR_SUPPLY("vddvario", "smsc911x"),
	REGULATOR_SUPPLY("vdd33a", "smsc911x"),
};

static struct resource v2m_usb_resources[] = {
	{
		.start	= V2M_ISP1761,
		.end	= V2M_ISP1761 + SZ_128K - 1,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= IRQ_V2M_ISP1761,
		.end	= IRQ_V2M_ISP1761,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct isp1760_platform_data v2m_usb_config = {
	.is_isp1761		= true,
	.bus_width_16		= false,
	.port1_otg		= true,
	.analog_oc		= false,
	.dack_polarity_high	= false,
	.dreq_polarity_high	= false,
};

static struct platform_device v2m_usb_device = {
	.name		= "isp1760",
	.id		= -1,
	.resource	= v2m_usb_resources,
	.num_resources	= ARRAY_SIZE(v2m_usb_resources),
	.dev.platform_data = &v2m_usb_config,
};

static struct physmap_flash_data v2m_flash_data = {
	.width		= 4,
};

static struct resource v2m_flash_resources[] = {
	{
		.start	= V2M_NOR0,
		.end	= V2M_NOR0 + SZ_64M - 1,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= V2M_NOR1,
		.end	= V2M_NOR1 + SZ_64M - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device v2m_flash_device = {
	.name		= "physmap-flash",
	.id		= -1,
	.resource	= v2m_flash_resources,
	.num_resources	= ARRAY_SIZE(v2m_flash_resources),
	.dev.platform_data = &v2m_flash_data,
};

static struct pata_platform_info v2m_pata_data = {
	.ioport_shift	= 2,
};

static struct resource v2m_pata_resources[] = {
	{
		.start	= V2M_CF,
		.end	= V2M_CF + 0xff,
		.flags	= IORESOURCE_MEM,
	}, {
		.start	= V2M_CF + 0x100,
		.end	= V2M_CF + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device v2m_cf_device = {
	.name		= "pata_platform",
	.id		= -1,
	.resource	= v2m_pata_resources,
	.num_resources	= ARRAY_SIZE(v2m_pata_resources),
	.dev.platform_data = &v2m_pata_data,
};

static struct mmci_platform_data v2m_mmci_data = {
	.ocr_mask	= MMC_VDD_32_33|MMC_VDD_33_34,
	.gpio_wp	= VEXPRESS_GPIO_MMC_WPROT,
	.gpio_cd	= VEXPRESS_GPIO_MMC_CARDIN,
};

static struct resource v2m_sysreg_resources[] = {
	{
		.start	= V2M_SYSREGS,
		.end	= V2M_SYSREGS + 0xfff,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device v2m_sysreg_device = {
	.name		= "vexpress-sysreg",
	.id		= -1,
	.resource	= v2m_sysreg_resources,
	.num_resources	= ARRAY_SIZE(v2m_sysreg_resources),
};

static struct platform_device v2m_muxfpga_device = {
	.name		= "vexpress-muxfpga",
	.id		= 0,
	.num_resources	= 1,
	.resource	= (struct resource []) {
		VEXPRESS_RES_FUNC(0, 7),
	}
};

static struct platform_device v2m_shutdown_device = {
	.name		= "vexpress-shutdown",
	.id		= 0,
	.num_resources	= 1,
	.resource	= (struct resource []) {
		VEXPRESS_RES_FUNC(0, 8),
	}
};

static struct platform_device v2m_reboot_device = {
	.name		= "vexpress-reboot",
	.id		= 0,
	.num_resources	= 1,
	.resource	= (struct resource []) {
		VEXPRESS_RES_FUNC(0, 9),
	}
};

static struct platform_device v2m_dvimode_device = {
	.name		= "vexpress-dvimode",
	.id		= 0,
	.num_resources	= 1,
	.resource	= (struct resource []) {
		VEXPRESS_RES_FUNC(0, 11),
	}
};

static AMBA_APB_DEVICE(aaci,  "mb:aaci",  0, V2M_AACI, IRQ_V2M_AACI, NULL);
static AMBA_APB_DEVICE(mmci,  "mb:mmci",  0, V2M_MMCI, IRQ_V2M_MMCI, &v2m_mmci_data);
static AMBA_APB_DEVICE(kmi0,  "mb:kmi0",  0, V2M_KMI0, IRQ_V2M_KMI0, NULL);
static AMBA_APB_DEVICE(kmi1,  "mb:kmi1",  0, V2M_KMI1, IRQ_V2M_KMI1, NULL);
static AMBA_APB_DEVICE(uart0, "mb:uart0", 0, V2M_UART0, IRQ_V2M_UART0, NULL);
static AMBA_APB_DEVICE(uart1, "mb:uart1", 0, V2M_UART1, IRQ_V2M_UART1, NULL);
static AMBA_APB_DEVICE(uart2, "mb:uart2", 0, V2M_UART2, IRQ_V2M_UART2, NULL);
static AMBA_APB_DEVICE(uart3, "mb:uart3", 0, V2M_UART3, IRQ_V2M_UART3, NULL);
static AMBA_APB_DEVICE(wdt,   "mb:wdt",   0, V2M_WDT, IRQ_V2M_WDT, NULL);
static AMBA_APB_DEVICE(rtc,   "mb:rtc",   0, V2M_RTC, IRQ_V2M_RTC, NULL);

static struct amba_device *v2m_amba_devs[] __initdata = {
	&aaci_device,
	&mmci_device,
	&kmi0_device,
	&kmi1_device,
	&uart0_device,
	&uart1_device,
	&uart2_device,
	&uart3_device,
	&wdt_device,
	&rtc_device,
};

static struct map_desc v2m_rs1_io_desc_pci[] __initdata = {
	{
	    .virtual    = 0xDEADBEEF,
	    .pfn        = 0xDEADBEEF,
	    .length     = 0xDEADBEEF,
	    .type       = MT_DEVICE
	},
	{
	    .virtual    = 0xDEADBEEF,
	    .pfn        = 0xDEADBEEF,
	    .length     = 0xDEADBEEF,
	    .type       = MT_DEVICE
	},
	{
	    .virtual    = 0xDEADBEEF,
	    .pfn        = 0xDEADBEEF,
	    .length     = 0xDEADBEEF,
	    .type       = MT_DEVICE
	},
	{
	    .virtual    = 0xDEADBEEF,
	    .pfn        = 0xDEADBEEF,
	    .length     = 0xDEADBEEF,
	    .type       = MT_DEVICE
	},
};

static void __init v2m_timer_init(void)
{
	vexpress_clk_init(ioremap(V2M_SYSCTL, SZ_4K));
	v2m_sp804_init(ioremap(V2M_TIMER01, SZ_4K), IRQ_V2M_TIMER0);
}

static void __init v2m_init_early(void)
{
	if (ct_desc->init_early)
		ct_desc->init_early();
	versatile_sched_clock_init(vexpress_get_24mhz_clock_base(), 24000000);
}

struct ct_desc *ct_desc;

static struct ct_desc *ct_descs[] __initdata = {
#if defined(CONFIG_ARCH_VEXPRESS_CA9X4) || defined(CONFIG_ARCH_VEXPRESS_GEM5)
	&ct_ca9x4_desc,
#endif
};

static void __init v2m_populate_ct_desc(void)
{
	int i;
	u32 current_tile_id;

	ct_desc = NULL;
	current_tile_id = vexpress_get_procid(VEXPRESS_SITE_MASTER)
				& V2M_CT_ID_MASK;

	for (i = 0; i < ARRAY_SIZE(ct_descs) && !ct_desc; ++i)
		if (ct_descs[i]->id == current_tile_id)
			ct_desc = ct_descs[i];

	if (!ct_desc)
		panic("vexpress: this kernel does not support core tile ID 0x%08x when booting via ATAGs.\n"
		      "You may need a device tree blob or a different kernel to boot on this board.\n",
		      current_tile_id);
}

static void __init v2m_map_io(void)
{
	iotable_init(v2m_io_desc, ARRAY_SIZE(v2m_io_desc));
	vexpress_sysreg_early_init(ioremap(V2M_SYSREGS, SZ_4K));
	v2m_populate_ct_desc();
	ct_desc->map_io();
}

static void __init v2m_init_irq(void)
{
	ct_desc->init_irq();
}

static void __init v2m_init(void)
{
	int i;

	regulator_register_fixed(0, v2m_eth_supplies,
			ARRAY_SIZE(v2m_eth_supplies));

	platform_device_register(&v2m_muxfpga_device);
	platform_device_register(&v2m_shutdown_device);
	platform_device_register(&v2m_reboot_device);
	platform_device_register(&v2m_dvimode_device);

	platform_device_register(&v2m_sysreg_device);
	platform_device_register(&v2m_pcie_i2c_device);
	platform_device_register(&v2m_ddc_i2c_device);
	platform_device_register(&v2m_flash_device);
	platform_device_register(&v2m_cf_device);
	platform_device_register(&v2m_eth_device);
	platform_device_register(&v2m_usb_device);

	for (i = 0; i < ARRAY_SIZE(v2m_amba_devs); i++)
		amba_device_register(v2m_amba_devs[i], &iomem_resource);

	ct_desc->init_tile();
}

MACHINE_START(VEXPRESS, "ARM-Versatile Express")
	.atag_offset	= 0x100,
	.smp		= smp_ops(vexpress_smp_ops),
	.map_io		= v2m_map_io,
	.init_early	= v2m_init_early,
	.init_irq	= v2m_init_irq,
	.init_time	= v2m_timer_init,
	.init_machine	= v2m_init,
MACHINE_END

static void __init v2m_dt_hdlcd_init(void)
{
	struct device_node *node;
	int len, na, ns;
	const __be32 *prop;
	phys_addr_t fb_base, fb_size;

	node = of_find_compatible_node(NULL, NULL, "arm,hdlcd");
	if (!node)
		return;

	na = of_n_addr_cells(node);
	ns = of_n_size_cells(node);

	prop = of_get_property(node, "framebuffer", &len);
	if (WARN_ON(!prop || len < (na + ns) * sizeof(*prop)))
		return;

	fb_base = of_read_number(prop, na);
	fb_size = of_read_number(prop + na, ns);

	if (WARN_ON(memblock_remove(fb_base, fb_size)))
		return;
};

static struct map_desc v2m_rs1_io_desc __initdata = {
	.virtual	= V2M_PERIPH,
	.pfn		= __phys_to_pfn(0x1c000000),
	.length		= SZ_2M,
	.type		= MT_DEVICE,
};

static int __init v2m_dt_scan_memory_map(unsigned long node, const char *uname,
		int depth, void *data)
{
	const char **map = data;

	if (strcmp(uname, "motherboard") != 0)
		return 0;

	*map = of_get_flat_dt_prop(node, "arm,v2m-memory-map", NULL);

	return 1;
}

static int __init v2m_dt_scan_pcie_gem5_ranges(unsigned long node, const char *uname,
		int depth, void *data)
{
	const u32 **ranges = data;

	if (strncmp(uname, "gem5_pcie@", 10) != 0)
		return 0;

	*ranges = of_get_flat_dt_prop(node, "ranges", NULL);
	return 1;
}

static int __init v2m_dt_scan_pcie_gem5_reg(unsigned long node, const char *uname,
		int depth, void *data)
{
	const u32 **reg = data;

	if (strncmp(uname, "gem5_pcie@", 10) != 0)
		return 0;

	*reg = of_get_flat_dt_prop(node, "reg", NULL);
	return 1;
}

static int __init v2m_dt_scan_pcie_gem5_virtual_reg(unsigned long node, const char *uname,
		int depth, void *data)
{
	const u32 **reg = data;

	if (strncmp(uname, "gem5_pcie@", 10) != 0)
		return 0;

	*reg = of_get_flat_dt_prop(node, "virtual-reg", NULL);
	return 1;
}

static int __init v2m_dt_scan_pcie_gem5_addr_cells(unsigned long node, const char *uname,
		int depth, void *data)
{
	const u32 **addr_cells = data;

	if (strncmp(uname, "gem5_pcie@", 10) != 0)
		return 0;

	*addr_cells = of_get_flat_dt_prop(node, "#address-cells", NULL);
	return 1;
}

static int __init v2m_dt_scan_pcie_gem5_size_cells(unsigned long node, const char *uname,
		int depth, void *data)
{
	const u32 **size_cells = data;

	if (strncmp(uname, "gem5_pcie@", 10) != 0)
		return 0;

	*size_cells = of_get_flat_dt_prop(node, "#size-cells", NULL);
	return 1;
}

static int __init v2m_dt_scan_motherboard_addr_cells(unsigned long node, const char *uname,
		int depth, void *data)
{
	const u32 **addr_cells = data;

	if (strcmp(uname, "motherboard") != 0)
		return 0;

	*addr_cells = of_get_flat_dt_prop(node, "#address-cells", NULL);
	return 1;
}

static void __init v2m_dt_pci_init(void)
{
	const __be32 *ranges = NULL;
	const __be32 *reg = NULL;
	const __be32 *virtual_reg = NULL;
	const __be32 *addr_cells  = NULL;
	const __be32 *size_cells  = NULL;
	const __be32 *m_addr_cells  = NULL;

	/* variables to identify io and mem spaces */
	int i;
	int n_addr_cells, n_size_cells, m_n_addr_cells, range_set_w;
	u32 phys_hi, phys_addr, addr_space_size;

	/* Scan Flat device tree for pcie node and extract the necessary parameters */
	printk("kdebugv2m: Following are test values to confirm proper working\n");
	of_scan_flat_dt(v2m_dt_scan_pcie_gem5_ranges, &ranges);

	if (ranges == NULL) {
		pr_info("No gem5 PCI found in DT.\n");
		return;
	}

	printk("kdebugv2m: Ranges %x %x \n",
		be32_to_cpup(&ranges[0]), be32_to_cpup(&ranges[1]) );

	of_scan_flat_dt(v2m_dt_scan_pcie_gem5_reg, &reg);
	printk("kdebugv2m: Regs %x %x \n", be32_to_cpup(&reg[1]),
		be32_to_cpup(&reg[2]) );

	of_scan_flat_dt(v2m_dt_scan_pcie_gem5_virtual_reg, &virtual_reg);
	printk("kdebugv2m: Virtual-Reg %x \n", be32_to_cpup(&virtual_reg[0]));

	of_scan_flat_dt(v2m_dt_scan_pcie_gem5_addr_cells, &addr_cells);
	printk("kdebugv2m: pci node addr_cells %x \n",
		be32_to_cpup(&addr_cells[0]));

	of_scan_flat_dt(v2m_dt_scan_pcie_gem5_size_cells, &size_cells);
	printk("kdebugv2m: pci node size_cells %x \n",
		be32_to_cpup(&size_cells[0]));

	of_scan_flat_dt(v2m_dt_scan_motherboard_addr_cells, &m_addr_cells);
	printk("kdebugv2m: motherboard addr_cells %x \n",
		be32_to_cpup(&m_addr_cells[0]));

	n_addr_cells = be32_to_cpup(addr_cells);
	n_size_cells = be32_to_cpup(size_cells);
	m_n_addr_cells = be32_to_cpup(m_addr_cells);
	range_set_w = n_addr_cells + m_n_addr_cells + n_size_cells;

	/* Assign values to the extern variables */
	dt_vexpress_pci_cfg_base  = be32_to_cpup(&reg[1]);
	dt_vexpress_pci_cfg_size  = be32_to_cpup(&reg[2]);
	dt_vexpress_pci_cfg_vbase = be32_to_cpup(&virtual_reg[0]);

	/* interating to get range parameters from Flat DTB */
	for(i=0; i<3; i++) {
		phys_hi = be32_to_cpup(&ranges[i*range_set_w]);
		phys_addr = be32_to_cpup(&ranges[i*range_set_w + n_addr_cells - 1]);
		addr_space_size = be32_to_cpup(&ranges[i*range_set_w + range_set_w - 1]);
		switch( (phys_hi >> SPACE_CODE_BIT_LOCN) & SPACE_CODE_WIDTH) {
		case 1: /* IO Space */
			dt_vexpress_pci_io_base = phys_addr;
			dt_vexpress_pci_io_size = addr_space_size;
			break;
		case 2: /* Mem Space */
                	if( (phys_hi >> 30) & 0x1) { /* Prefetchable mem */
				dt_vexpress_pci_memp_base = phys_addr;
				dt_vexpress_pci_memp_size = addr_space_size;
			} else { /* non-prefetchable mem */
				dt_vexpress_pci_mem_base = phys_addr;
				dt_vexpress_pci_mem_size = addr_space_size;
			}
			break;
		}
	}

	dt_vexpress_pci_mem_vbase = dt_vexpress_pci_cfg_vbase +
		dt_vexpress_pci_cfg_size;
	dt_vexpress_pci_memp_vbase = dt_vexpress_pci_mem_vbase +
		dt_vexpress_pci_mem_size;
	dt_vexpress_pci_io_vbase = dt_vexpress_pci_memp_vbase +
		dt_vexpress_pci_memp_size;
	dt_vexpress_pci_cfg_limit  = (dt_vexpress_pci_cfg_size +
		dt_vexpress_pci_cfg_base -1);
	dt_vexpress_pci_mem_limit  = (dt_vexpress_pci_mem_base +
		dt_vexpress_pci_mem_size - 1);
	dt_vexpress_pci_memp_limit = (dt_vexpress_pci_memp_base +
		dt_vexpress_pci_memp_size - 1);
	dt_vexpress_pci_io_limit   = (dt_vexpress_pci_io_base +
		dt_vexpress_pci_io_size - 1);

	/* Update the map_desc structure */
	// CFG
	v2m_rs1_io_desc_pci[0].virtual    = dt_vexpress_pci_cfg_vbase;
	v2m_rs1_io_desc_pci[0].pfn        = __phys_to_pfn(dt_vexpress_pci_cfg_base);
	v2m_rs1_io_desc_pci[0].length     = dt_vexpress_pci_cfg_size;

	// MEMP
	v2m_rs1_io_desc_pci[1].virtual    = dt_vexpress_pci_memp_vbase;
	v2m_rs1_io_desc_pci[1].pfn        = __phys_to_pfn(dt_vexpress_pci_memp_base);
	v2m_rs1_io_desc_pci[1].length     = dt_vexpress_pci_memp_size;

	// MEM
	v2m_rs1_io_desc_pci[2].virtual    = dt_vexpress_pci_mem_vbase;
	v2m_rs1_io_desc_pci[2].pfn        = __phys_to_pfn(dt_vexpress_pci_mem_base);
	v2m_rs1_io_desc_pci[2].length     = dt_vexpress_pci_mem_size;

	// IO
	v2m_rs1_io_desc_pci[3].virtual    = dt_vexpress_pci_io_vbase;
	v2m_rs1_io_desc_pci[3].pfn        = __phys_to_pfn(dt_vexpress_pci_io_base);
	v2m_rs1_io_desc_pci[3].length     = dt_vexpress_pci_io_size;

	/* Initialize the updated mapping */
	iotable_init(v2m_rs1_io_desc_pci, ARRAY_SIZE(v2m_rs1_io_desc_pci));
}

void __init v2m_dt_map_io(void)
{
	const char *map = NULL;

	of_scan_flat_dt(v2m_dt_scan_memory_map, &map);

	if (map && strcmp(map, "rs1") == 0) {
		iotable_init(&v2m_rs1_io_desc, 1);
#ifdef CONFIG_PCI
		v2m_dt_pci_init();
#endif
	} else
		iotable_init(v2m_io_desc, ARRAY_SIZE(v2m_io_desc));

#if defined(CONFIG_SMP)
	vexpress_dt_smp_map_io();
#endif
}

void __init v2m_dt_init_early(void)
{
	u32 dt_hbi;

	vexpress_sysreg_of_early_init();

	/* Confirm board type against DT property, if available */
	if (of_property_read_u32(of_allnodes, "arm,hbi", &dt_hbi) == 0) {
		u32 hbi = vexpress_get_hbi(VEXPRESS_SITE_MASTER);

		if (WARN_ON(dt_hbi != hbi))
			pr_warning("vexpress: DT HBI (%x) is not matching "
					"hardware (%x)!\n", dt_hbi, hbi);
	}

	versatile_sched_clock_init(vexpress_get_24mhz_clock_base(), 24000000);

	v2m_dt_hdlcd_init();
}

static const struct of_device_id v2m_dt_bus_match[] __initconst = {
	{ .compatible = "simple-bus", },
	{ .compatible = "arm,amba-bus", },
	{ .compatible = "arm,vexpress,config-bus", },
	{}
};

static void __init v2m_dt_init(void)
{
	l2x0_of_init(0x00400000, 0xfe0fffff);
	of_platform_populate(NULL, v2m_dt_bus_match, NULL, NULL);
}

static const char * const v2m_dt_match[] __initconst = {
	"arm,vexpress",
	NULL,
};

DT_MACHINE_START(VEXPRESS_DT, "ARM-Versatile Express")
	.dt_compat	= v2m_dt_match,
	.smp		= smp_ops(vexpress_smp_ops),
	.smp_init	= smp_init_ops(vexpress_smp_init_ops),
	.map_io		= v2m_dt_map_io,
	.init_early	= v2m_dt_init_early,
	.init_machine	= v2m_dt_init,
MACHINE_END
