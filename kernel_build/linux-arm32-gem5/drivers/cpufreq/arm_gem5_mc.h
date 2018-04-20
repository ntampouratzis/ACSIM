/*
 * ARM GEM5 Multi-cluster platform's CPUFreq header file
 *
 * Copyright (C) 2013 - 2014 ARM Ltd.
 *
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
#ifndef CPUFREQ_ARM_GEM5_MC_H
#define CPUFREQ_ARM_GEM5_MC_H

#include <linux/cpufreq.h>
#include <linux/device.h>
#include <linux/types.h>

/* Currently we support n=32 clusters */
#define MAX_CLUSTERS	32

struct cpufreq_arm_mc_ops {
	char name[CPUFREQ_NAME_LEN];
	int (*get_transition_latency)(struct device *cpu_dev);

	/*
	 * This must set opp table for cpu_dev in a similar way as done by
	 * of_init_opp_table().
	 */
	int (*init_opp_table)(struct device *cpu_dev);
};

static inline int cpu_to_cluster(int cpu)
{
	return topology_physical_package_id(cpu);
}

int mc_cpufreq_register(struct cpufreq_arm_mc_ops *ops);
void mc_cpufreq_unregister(struct cpufreq_arm_mc_ops *ops);

#endif /* CPUFREQ_ARM_GEM5_MC_H */
