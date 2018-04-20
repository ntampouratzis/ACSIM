/*
 * Gem5 Energy Controller support
 * (code adapted from vexpress-spc)
 *
 * Copyright (C) 2013-2014 ARM Ltd.
 *
 * Authors: Akash Bagdia <akash.bagdia@arm.com>
 *          Vasileios Spiliopoulos <vasileios.spiliopoulos@arm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/gem5_energy_ctrl.h>

// Register addresses
#define DVFS_HANDLER_STATUS 0x00
#define DVFS_NUM_DOMAINS 0x04
#define DVFS_DOMAINID_AT_INDEX 0x08
#define DVFS_HANDLER_TRANS_LATENCY 0x0C
#define DOMAIN_ID  0x10
#define PERF_LEVEL 0x14
#define PERF_LEVEL_ACK 0x18
#define NUM_OF_PERF_LEVELS 0x1C
#define PERF_LEVEL_TO_READ 0x20
#define FREQ_AT_PERF_LEVEL 0x24
#define VOLT_AT_PERF_LEVEL 0x28

#define TIME_OUT	100
#define GEM5_MAX_NUM_DOMAINS	32

struct gem5_energy_ctrl_drvdata {
	void __iomem *baseaddr;
	bool dvfs_handler_status;
	u32 num_gem5_domains;
	u32 *freqs[GEM5_MAX_NUM_DOMAINS];
	u32 *voltages[GEM5_MAX_NUM_DOMAINS];
	int opp_cnt[GEM5_MAX_NUM_DOMAINS];
	u32 domain_ids[GEM5_MAX_NUM_DOMAINS];
	spinlock_t lock;
};

static struct gem5_energy_ctrl_drvdata *info;

static int gem5_energy_ctrl_load_result = -EAGAIN;

static bool gem5_energy_ctrl_initialized(void)
{
	return gem5_energy_ctrl_load_result == 0;
}

bool gem5_energy_ctrl_dvfs_enabled(void)
{
	return info->dvfs_handler_status;
}
EXPORT_SYMBOL_GPL(gem5_energy_ctrl_dvfs_enabled);

static u32 index_of_domain_id(u32 domain_id)
{
	u32 i;

	for(i = 0; i < info->num_gem5_domains; i++)
		if(domain_id == info->domain_ids[i])
			return i;
	return GEM5_MAX_NUM_DOMAINS;
}

u32 gem5_energy_ctrl_get_trans_latency(void)
{
	u32 data;

	if (!gem5_energy_ctrl_initialized() || !info->dvfs_handler_status)
		return -EINVAL;

	spin_lock(&info->lock);
	data = readl(info->baseaddr + DVFS_HANDLER_TRANS_LATENCY);
	spin_unlock(&info->lock);

	return data;
}
EXPORT_SYMBOL_GPL(gem5_energy_ctrl_get_trans_latency);

/**
 * gem5_energy_ctrl_get_performance - get current performance level of domain
 * @domain_id: mpidr[15:8] bitfield describing domain's affinity level
 * @freq: pointer to the performance level to be assigned
 *
 * Return: 0 on success
 *         < 0 on read error
 */
int gem5_energy_ctrl_get_performance(u32 domain_id, u32 *freq)
{
	int perf;
	u32 domain_index;

	if (!gem5_energy_ctrl_initialized() || !info->dvfs_handler_status)
		return -EINVAL;

	domain_index = index_of_domain_id(domain_id);
	if(domain_index >= info->num_gem5_domains)
		return -EINVAL;

	spin_lock(&info->lock);
	writel(domain_id, info->baseaddr + DOMAIN_ID);
	perf = readl(info->baseaddr + PERF_LEVEL);
	spin_unlock(&info->lock);
	*freq = info->freqs[domain_index][perf];

	return 0;
}
EXPORT_SYMBOL_GPL(gem5_energy_ctrl_get_performance);

static int gem5_energy_ctrl_find_perf_index(u32 domain_index, u32 freq)
{
	int idx;

	for (idx = 0; idx < info->opp_cnt[domain_index]; idx++)
		if (info->freqs[domain_index][idx] == freq)
			return idx;

	return -EINVAL;
}


static inline int read_wait_to(void __iomem *reg, int status, int timeout)
{
	while (timeout-- && readl(reg) != status) {
		cpu_relax();
		udelay(2);
	}
	if (!timeout)
		return -EAGAIN;
	else
		return 0;
}

/**
 * gem5_energy_ctrl_set_performance - set current performance level of domain
 *
 * @domain_id: mpidr[15:8] bitfield describing domain's affinity level
 * @freq: performance level to be programmed
 *
 * Returns: 0 on success
 *          < 0 on write error
 */
int gem5_energy_ctrl_set_performance(u32 domain_id, u32 freq)
{
	int ret, perf;
	u32 domain_index;

        ret = 0;

	if (!gem5_energy_ctrl_initialized() || !info->dvfs_handler_status)
		return -EINVAL;

	domain_index = index_of_domain_id(domain_id);
	if(domain_index >= info->num_gem5_domains)
		return -EINVAL;

	spin_lock(&info->lock);

	writel(domain_id, info->baseaddr + DOMAIN_ID);

	perf = gem5_energy_ctrl_find_perf_index(domain_index, freq);

	if (perf < 0)
		return -EINVAL;

	writel(perf, info->baseaddr + PERF_LEVEL);

	//Some logic to determine successful setting of perf level
	if (read_wait_to(info->baseaddr + PERF_LEVEL_ACK, 1, TIME_OUT))
		ret = -EAGAIN;

	spin_unlock(&info->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(gem5_energy_ctrl_set_performance);

/**
 * gem5_energy_ctrl_populate_opps() - initialize opp tables from energy ctrl
 *
 * @domain_id: mpidr[15:8] bitfield describing domain's affinity level
 *
 * Return: 0 on success
 *         < 0 on error
 */
static int gem5_energy_ctrl_populate_opps(u32 domain_id)
{
	u32 data = 0, i;
	u32 domain_index = index_of_domain_id(domain_id);

	if (!info->dvfs_handler_status ||
		WARN_ON_ONCE(domain_index >= info->num_gem5_domains))
		return -EINVAL;

	spin_lock(&info->lock);

	writel(domain_id, info->baseaddr + DOMAIN_ID);

	if (readl(info->baseaddr + DOMAIN_ID) != domain_id) {
		spin_unlock(&info->lock);
		return -EINVAL;
	}

	data = readl(info->baseaddr + NUM_OF_PERF_LEVELS);
	info->opp_cnt[domain_index] = data;

	info->freqs[domain_index] = kzalloc(sizeof(u32) *
				info->opp_cnt[domain_index], GFP_KERNEL);

	info->voltages[domain_index] = kzalloc(sizeof(u32) *
				info->opp_cnt[domain_index], GFP_KERNEL);

	for (i = 0; i < info->opp_cnt[domain_index] ; i++) {
		writel(i, info->baseaddr + PERF_LEVEL_TO_READ);
		data = readl(info->baseaddr + FREQ_AT_PERF_LEVEL);
		info->freqs[domain_index][i] = data;
		data = readl(info->baseaddr + VOLT_AT_PERF_LEVEL);
		info->voltages[domain_index][i] = data;
	}

	spin_unlock(&info->lock);

	return 0;
}

/**
 * gem5_energy_ctrl_get_opp_table() - Retrieve a pointer to the frequency,
 *                voltage tables for a given domain
 *
 * @domain_id: mpidr[15:8] bitfield describing domain's affinity level
 * @fptr: pointer to be initialized
 * Return: operating points count on success
 *         -EINVAL on pointer error
 */
int gem5_energy_ctrl_get_opp_table(u32 domain_id, u32 **fptr, u32 **vptr)
{
	u32 domain_index;

	if (!gem5_energy_ctrl_initialized() || !fptr || !vptr || !info ||
            !info->dvfs_handler_status)
		return -EINVAL;

	domain_index = index_of_domain_id(domain_id);
	if(domain_index >= info->num_gem5_domains)
		return -EINVAL;

	*fptr = info->freqs[domain_index];
	*vptr = info->voltages[domain_index];
	return info->opp_cnt[domain_index];
}
EXPORT_SYMBOL_GPL(gem5_energy_ctrl_get_opp_table);

static const struct of_device_id gem5_energy_ctrl_ids[] __initconst = {
	{ .compatible = "arm,gem5-energy-ctrl" },
	{},
};

static int __init gem5_energy_ctrl_init(void)
{
	int ret;
	u32 i, data;
	struct device_node *node = of_find_matching_node(NULL,
							 gem5_energy_ctrl_ids);

	if (!node)
		return -ENODEV;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		pr_err("%s: unable to allocate mem\n", __func__);
		return -ENOMEM;
	}

	info->dvfs_handler_status = 0;
	info->num_gem5_domains = 0;

	info->baseaddr = of_iomap(node, 0);
	if (WARN_ON(!info->baseaddr)) {
		ret = -ENXIO;
		goto mem_free;
	}

	spin_lock_init(&info->lock);

	info->dvfs_handler_status = readl(info->baseaddr + DVFS_HANDLER_STATUS);

	if (!info->dvfs_handler_status) {
		pr_info("gem5 DVFS handler is disabled\n");
	} else {
		info->num_gem5_domains = readl(info->baseaddr +
						DVFS_NUM_DOMAINS);
		if (info->num_gem5_domains > GEM5_MAX_NUM_DOMAINS) {
			pr_err("gem5 DVFS handler manages more domains than\
				supported by the gem5 energy controller driver\n");
			ret = -ENODEV;
			goto unmap;
		}
		else {
			/* Get domain ID information
			* Populate operation table for all the domains(clusters)
			* managed by the controller
			*/
			for(i = 0;i < info->num_gem5_domains; i++) {
				spin_lock(&info->lock);
				writel(i, info->baseaddr +
					DVFS_DOMAINID_AT_INDEX);
				data = readl(info->baseaddr +
					DVFS_DOMAINID_AT_INDEX);
				spin_unlock(&info->lock);
				info->domain_ids[i] = data;
				if(gem5_energy_ctrl_populate_opps(info->domain_ids[i]))
				{
					pr_err("failed to build OPP table for\
						%d domain\n",
					info->domain_ids[i]);
					ret = -ENODEV;
					goto unmap;
				}
			}
		}
	}
	pr_info("gem5-energy-ctrl loaded at %p\n", info->baseaddr);
	return 0;

unmap:
	pr_info("gem5-energy-ctrl unmapped at %p, possible error in syncing with "\
		"the device\n", info->baseaddr);
	iounmap(info->baseaddr);

mem_free:
	kfree(info);
	return ret;
}

static bool __init __gem5_energy_ctrl_check_loaded(void);
/*
 * Pointer spc_check_loaded is swapped after init hence it is safe
 * to initialize it to a function in the __init section
 */
static bool (*energy_ctrl_check_loaded)(void) __refdata = \
     &__gem5_energy_ctrl_check_loaded;

static bool __init __gem5_energy_ctrl_check_loaded(void)
{
	if (gem5_energy_ctrl_load_result == -EAGAIN)
		gem5_energy_ctrl_load_result = gem5_energy_ctrl_init();
	energy_ctrl_check_loaded = &gem5_energy_ctrl_initialized;
	return gem5_energy_ctrl_initialized();
}

/*
 * Function exported to manage early_initcall ordering.
 * SPC code is needed very early in the boot process
 * to bring CPUs out of reset and initialize power
 * management back-end. After boot swap pointers to
 * make the functionality check available to loadable
 * modules, when early boot init functions have been
 * already freed from kernel address space.
 */
bool gem5_energy_ctrl_check_loaded(void)
{
	return energy_ctrl_check_loaded();
}
EXPORT_SYMBOL_GPL(gem5_energy_ctrl_check_loaded);

//static int __init vexpress_spc_early_init(void)
//{
//	__vexpress_spc_check_loaded();
//	return vexpress_spc_load_result;
//}
//early_initcall(vexpress_spc_early_init);
MODULE_LICENSE("GPL");
