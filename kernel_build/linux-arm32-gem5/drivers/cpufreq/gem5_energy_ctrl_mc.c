/*
 * Gem5 Multi-cluster CPUFreq Interface driver
 * (adapted from vexpress_big_little.c)
 *
 * It provides necessary opp's to arm_gem5_mc.c cpufreq driver and gets
 * frequency information from gem5 energy controller device.
 *
 * Copyright (C) 2013 - 2014 ARM Ltd.
 * Authors: Akash Bagdia <Akash.bagdia@arm.com>
 *          Vasileios Spiliopoulos <vasileios.spiliopoulos@arm.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cpufreq.h>
#include <linux/export.h>
#include <linux/pm_opp.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/gem5_energy_ctrl.h>
#include "arm_gem5_mc.h"

static int gem5_init_opp_table(struct device *cpu_dev)
{
	int i = -1, count, cluster = cpu_to_cluster(cpu_dev->id);
	u32 *freq_table;
	u32 *volt_table; /* In micro volts */
	int ret;

	count = gem5_energy_ctrl_get_opp_table(cluster, &freq_table, &volt_table);
	if (!freq_table || !count) {
		pr_err("gem5 energy controller returned invalid freq table");
		return -EINVAL;
	}

	if (!volt_table || !count) {
		pr_err("gem5 energy controller returned invalid voltage table");
		return -EINVAL;
	}

	while (++i < count) {
		ret = dev_pm_opp_add(cpu_dev, freq_table[i] * 1000,
			volt_table[i]);
		if (ret) {
			dev_warn(cpu_dev,
				"%s: Failed to add OPP freq %d, u-voltage %d,\
				 err: %d\n",
				 __func__, freq_table[i] * 1000,
				 volt_table[i], ret);
			return ret;
		}
	}

	return 0;
}

static int gem5_get_transition_latency(struct device *cpu_dev)
{
	return gem5_energy_ctrl_get_trans_latency();
}

static struct cpufreq_arm_mc_ops gem5_mc_ops = {
	.name	= "gem5-mc",
	.get_transition_latency = gem5_get_transition_latency,
	.init_opp_table = gem5_init_opp_table,
};

static int gem5_mc_init(void)
{
	if (!gem5_energy_ctrl_check_loaded()) {
		pr_info("%s: No energy controller found\n", __func__);
		return -ENOENT;
	}

        if (!gem5_energy_ctrl_dvfs_enabled()) {
                pr_info("%s: DVFS handler in energy controller is disabled, \
                        ARM gem5 multi-cluster cpufreq driver \
                        will not be registered\n",
                        __func__);
                return -ENOENT;
        }

	return mc_cpufreq_register(&gem5_mc_ops);
}
module_init(gem5_mc_init);

static void gem5_mc_exit(void)
{
	return mc_cpufreq_unregister(&gem5_mc_ops);
}
module_exit(gem5_mc_exit);

MODULE_DESCRIPTION("ARM gem5 multi-cluster cpufreq driver");
MODULE_LICENSE("GPL");
