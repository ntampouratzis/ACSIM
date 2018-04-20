/*
 * ARM Multi-Cluster Platforms CPUFreq support for Gem5
 *
 * Copyright (C) 2013 -2014 ARM Ltd.
 *
 * Akash Bagdia <akash.bagdia@arm.com>
 * Vasileios Spiliopoulos <vasileios.spiliopoulos@arm.com>
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

#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/export.h>
#include <linux/mutex.h>
#include <linux/of_platform.h>
#include <linux/pm_opp.h>
#include <linux/slab.h>
#include <linux/topology.h>
#include <linux/types.h>

#include "arm_gem5_mc.h"

static struct cpufreq_arm_mc_ops *arm_mc_ops;
static struct clk *clk[MAX_CLUSTERS];
static struct cpufreq_frequency_table *freq_table[MAX_CLUSTERS + 1];
static atomic_t cluster_usage[MAX_CLUSTERS + 1] = {ATOMIC_INIT(0),
	ATOMIC_INIT(0)};

static DEFINE_PER_CPU(unsigned int, physical_cluster);
static DEFINE_PER_CPU(unsigned int, cpu_last_req_freq);

static struct mutex cluster_lock[MAX_CLUSTERS];

static unsigned int find_cluster_maxfreq(int cluster)
{
	int j;
	u32 max_freq = 0, cpu_freq;

	for_each_online_cpu(j) {
		cpu_freq = per_cpu(cpu_last_req_freq, j);

		if ((cluster == per_cpu(physical_cluster, j)) &&
				(max_freq < cpu_freq))
			max_freq = cpu_freq;
	}

	pr_debug("%s: cluster: %d, max freq: %d\n", __func__, cluster,
			max_freq);

	return max_freq;
}

static unsigned int clk_get_cpu_rate(unsigned int cpu)
{
	u32 cur_cluster = per_cpu(physical_cluster, cpu);
	u32 rate = clk_get_rate(clk[cur_cluster]) / 1000;

	pr_debug("%s: cpu: %d, cluster: %d, freq: %u\n", __func__, cpu,
			cur_cluster, rate);

	return rate;
}

static unsigned int mc_cpufreq_get_rate(unsigned int cpu)
{
	return clk_get_cpu_rate(cpu);
}

static unsigned int
mc_cpufreq_set_rate(u32 cpu, u32 new_cluster, u32 rate)
{
	u32 new_rate;
	int ret;

	mutex_lock(&cluster_lock[new_cluster]);

	new_rate = rate;

	pr_debug("%s: cpu: %d, new cluster: %d, freq: %d\n",
			__func__, cpu, new_cluster, new_rate);

	ret = clk_set_rate(clk[new_cluster], new_rate * 1000);
	if (WARN_ON(ret)) {
		pr_err("clk_set_rate failed: %d, new cluster: %d\n", ret,
				new_cluster);

		mutex_unlock(&cluster_lock[new_cluster]);

		return ret;
	}

	mutex_unlock(&cluster_lock[new_cluster]);

	return 0;
}

/* Validate policy frequency range */
static int mc_cpufreq_verify_policy(struct cpufreq_policy *policy)
{
	u32 cur_cluster = cpu_to_cluster(policy->cpu);

	return cpufreq_frequency_table_verify(policy, freq_table[cur_cluster]);
}

/* Set clock frequency */
static int mc_cpufreq_set_target(struct cpufreq_policy *policy,
		unsigned int target_freq, unsigned int relation)
{
	struct cpufreq_freqs freqs;
	u32 cpu = policy->cpu, freq_tab_idx, cur_cluster, new_cluster;
	int ret = 0;

	cur_cluster = cpu_to_cluster(cpu);
	new_cluster = per_cpu(physical_cluster, cpu);

	freqs.old = mc_cpufreq_get_rate(cpu);

	/* Determine valid target frequency using freq_table */
	cpufreq_frequency_table_target(policy, freq_table[cur_cluster],
			target_freq, relation, &freq_tab_idx);
	freqs.new = freq_table[cur_cluster][freq_tab_idx].frequency;

	pr_debug("%s: cpu: %d, cluster: %d, oldfreq: %d, target freq: %d, new freq: %d\n",
			__func__, cpu, cur_cluster, freqs.old, target_freq,
			freqs.new);

	if (freqs.old == freqs.new)
		return 0;

	cpufreq_notify_transition(policy, &freqs, CPUFREQ_PRECHANGE);

	ret = mc_cpufreq_set_rate(cpu, new_cluster, freqs.new);
	if (ret)
		freqs.new = freqs.old;

	cpufreq_notify_transition(policy, &freqs, CPUFREQ_POSTCHANGE);

	return ret;
}

static inline u32 get_table_count(struct cpufreq_frequency_table *table)
{
	int count;

	for (count = 0; table[count].frequency != CPUFREQ_TABLE_END; count++)
		;

	return count;
}

/* get the minimum frequency in the cpufreq_frequency_table */
static inline u32 get_table_min(struct cpufreq_frequency_table *table)
{
	int i;
	uint32_t min_freq = ~0;
	for (i = 0; (table[i].frequency != CPUFREQ_TABLE_END); i++)
		if (table[i].frequency < min_freq)
			min_freq = table[i].frequency;
	return min_freq;
}

/* get the maximum frequency in the cpufreq_frequency_table */
static inline u32 get_table_max(struct cpufreq_frequency_table *table)
{
	int i;
	uint32_t max_freq = 0;
	for (i = 0; (table[i].frequency != CPUFREQ_TABLE_END); i++)
		if (table[i].frequency > max_freq)
			max_freq = table[i].frequency;
	return max_freq;
}

static void _put_cluster_clk_and_freq_table(struct device *cpu_dev)
{
	u32 cluster = cpu_to_cluster(cpu_dev->id);

	if (!atomic_dec_return(&cluster_usage[cluster])) {
		clk_put(clk[cluster]);
		dev_pm_opp_free_cpufreq_table(cpu_dev, &freq_table[cluster]);
		dev_dbg(cpu_dev, "%s: cluster: %d\n", __func__, cluster);
	}
}

static void put_cluster_clk_and_freq_table(struct device *cpu_dev)
{
	u32 cluster = cpu_to_cluster(cpu_dev->id);

	if (cluster < MAX_CLUSTERS)
		return _put_cluster_clk_and_freq_table(cpu_dev);
}

static int _get_cluster_clk_and_freq_table(struct device *cpu_dev)
{
	u32 cluster = cpu_to_cluster(cpu_dev->id);
	char name[14] = "cpu-cluster.";
	int ret;

	if (atomic_inc_return(&cluster_usage[cluster]) != 1)
		return 0;

	ret = arm_mc_ops->init_opp_table(cpu_dev);
	if (ret) {
		dev_err(cpu_dev, "%s: init_opp_table failed, cpu: %d, err: %d\n",
				__func__, cpu_dev->id, ret);
		goto atomic_dec;
	}

	ret = dev_pm_opp_init_cpufreq_table(cpu_dev, &freq_table[cluster]);
	if (ret) {
		dev_err(cpu_dev, "%s: failed to init cpufreq table, cpu: %d, err: %d\n",
				__func__, cpu_dev->id, ret);
		goto atomic_dec;
	}

	name[12] = cluster + '0';
	clk[cluster] = clk_get_sys(name, NULL);
	if (!IS_ERR(clk[cluster])) {
		dev_dbg(cpu_dev, "%s: clk: %p & freq table: %p, cluster: %d\n",
				__func__, clk[cluster], freq_table[cluster],
				cluster);
		return 0;
	}

	dev_err(cpu_dev, "%s: Failed to get clk for cpu: %d, cluster: %d\n",
			__func__, cpu_dev->id, cluster);
	ret = PTR_ERR(clk[cluster]);
	dev_pm_opp_free_cpufreq_table(cpu_dev, &freq_table[cluster]);

atomic_dec:
	atomic_dec(&cluster_usage[cluster]);
	dev_err(cpu_dev, "%s: Failed to get data for cluster: %d\n", __func__,
			cluster);
	return ret;
}

static int get_cluster_clk_and_freq_table(struct device *cpu_dev)
{
	u32 cluster = cpu_to_cluster(cpu_dev->id);

	if (cluster < MAX_CLUSTERS)
		return _get_cluster_clk_and_freq_table(cpu_dev);

	return 0;
}

/* Per-CPU initialization */
static int mc_cpufreq_init(struct cpufreq_policy *policy)
{
	u32 cur_cluster = cpu_to_cluster(policy->cpu);
	struct device *cpu_dev;
	int ret;

	cpu_dev = get_cpu_device(policy->cpu);
	if (!cpu_dev) {
		pr_err("%s: failed to get cpu%d device\n", __func__,
				policy->cpu);
		return -ENODEV;
	}

	ret = get_cluster_clk_and_freq_table(cpu_dev);
	if (ret)
		return ret;

	ret = cpufreq_frequency_table_cpuinfo(policy, freq_table[cur_cluster]);
	if (ret) {
		dev_err(cpu_dev, "CPU %d, cluster: %d invalid freq table\n",
				policy->cpu, cur_cluster);
		put_cluster_clk_and_freq_table(cpu_dev);
		return ret;
	}

	cpufreq_frequency_table_get_attr(freq_table[cur_cluster], policy->cpu);

	if (cur_cluster < MAX_CLUSTERS) {
		cpumask_copy(policy->cpus, topology_core_cpumask(policy->cpu));

		per_cpu(physical_cluster, policy->cpu) = cur_cluster;
	} else {
		pr_err("Invalid current cluster %d\n", cur_cluster);
		return -ENODEV;
	}

	if (arm_mc_ops->get_transition_latency)
		policy->cpuinfo.transition_latency =
			arm_mc_ops->get_transition_latency(cpu_dev);
	else
		policy->cpuinfo.transition_latency = CPUFREQ_ETERNAL;

	policy->cur = clk_get_cpu_rate(policy->cpu);

	dev_info(cpu_dev, "%s: CPU %d initialized\n", __func__, policy->cpu);
	return 0;
}

/* Export freq_table to sysfs */
static struct freq_attr *mc_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver mc_cpufreq_driver = {
	.name			= "arm-gem5-mc",
	.flags			= CPUFREQ_STICKY |
					CPUFREQ_HAVE_GOVERNOR_PER_POLICY,
	.verify			= mc_cpufreq_verify_policy,
	.target			= mc_cpufreq_set_target,
	.get			= mc_cpufreq_get_rate,
	.init			= mc_cpufreq_init,
	.attr			= mc_cpufreq_attr,
};

int mc_cpufreq_register(struct cpufreq_arm_mc_ops *ops)
{
	int ret, i;

	if (arm_mc_ops) {
		pr_debug("%s: Already registered: %s, exiting\n", __func__,
				arm_mc_ops->name);
		return -EBUSY;
	}

	if (!ops || !strlen(ops->name) || !ops->init_opp_table) {
		pr_err("%s: Invalid arm_mc_ops, exiting\n", __func__);
		return -ENODEV;
	}

	arm_mc_ops = ops;

	for (i = 0; i < MAX_CLUSTERS; i++)
		mutex_init(&cluster_lock[i]);

	ret = cpufreq_register_driver(&mc_cpufreq_driver);
	if (ret) {
		pr_info("%s: Failed registering platform driver: %s, err: %d\n",
				__func__, ops->name, ret);
		arm_mc_ops = NULL;
	} else {
		pr_info("%s: Registered platform driver: %s\n",
				__func__, ops->name);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(mc_cpufreq_register);

void mc_cpufreq_unregister(struct cpufreq_arm_mc_ops *ops)
{
	if (arm_mc_ops != ops) {
		pr_err("%s: Registered with: %s, can't unregister, exiting\n",
				__func__, arm_mc_ops->name);
		return;
	}

	cpufreq_unregister_driver(&mc_cpufreq_driver);
	pr_info("%s: Un-registered platform driver: %s\n", __func__,
			arm_mc_ops->name);

	int i;

	for (i = 0; i < MAX_CLUSTERS; i++) {
		struct device *cdev = get_cpu_device(i);
		if (!cdev) {
			pr_err("%s: failed to get cpu%d device\n",
					__func__, i);
			return;
		}

		put_cluster_clk_and_freq_table(cdev);
	}

	arm_mc_ops = NULL;
}
EXPORT_SYMBOL_GPL(mc_cpufreq_unregister);
