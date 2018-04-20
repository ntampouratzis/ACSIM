/*
 * Copyright (C) 2013 -2014 ARM Limited
 * Copyright (C) 2013 Linaro
 *
 * Authors: Akash Bagdia <Akash.bagdia@arm.com>
 *          Vasileios Spiliopoulos <vasileios.spiliopoulos@arm.com>
 * (code adapted from clk-vexpress-spc.c)
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

/* PE Controller clock programming interface for Gem5 Platform cpus */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/gem5_energy_ctrl.h>

struct clk_energy_ctrl {
	struct clk_hw hw;
	spinlock_t *lock;
	int cluster;
};

#define to_clk_energy_ctrl(ec) container_of(ec, struct clk_energy_ctrl, hw)

static unsigned long energy_ctrl_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	struct clk_energy_ctrl *energy_ctrl = to_clk_energy_ctrl(hw);
	u32 freq;

	if (gem5_energy_ctrl_get_performance(energy_ctrl->cluster, &freq)) {
		return -EIO;
		pr_err("%s: Failed", __func__);
	}

	return freq * 1000;
}

static long energy_ctrl_round_rate(struct clk_hw *hw, unsigned long drate,
		unsigned long *parent_rate)
{
	return drate;
}

static int energy_ctrl_set_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long parent_rate)
{
	struct clk_energy_ctrl *energy_ctrl = to_clk_energy_ctrl(hw);

	return gem5_energy_ctrl_set_performance(energy_ctrl->cluster, rate / 1000);
}

static struct clk_ops clk_energy_ctrl_ops = {
	.recalc_rate = energy_ctrl_recalc_rate,
	.round_rate = energy_ctrl_round_rate,
	.set_rate = energy_ctrl_set_rate,
};

struct clk *gem5_clk_register_energy_ctrl(const char *name, int cluster_id)
{
	struct clk_init_data init;
	struct clk_energy_ctrl *energy_ctrl;
	struct clk *clk;

	if (!name) {
		pr_err("Invalid name passed");
		return ERR_PTR(-EINVAL);
	}

	energy_ctrl = kzalloc(sizeof(*energy_ctrl), GFP_KERNEL);
	if (!energy_ctrl) {
		pr_err("could not allocate energy_ctrl clk\n");
		return ERR_PTR(-ENOMEM);
	}

	energy_ctrl->hw.init = &init;
	energy_ctrl->cluster = cluster_id;

	init.name = name;
	init.ops = &clk_energy_ctrl_ops;
	init.flags = CLK_IS_ROOT | CLK_GET_RATE_NOCACHE;
	init.num_parents = 0;

	clk = clk_register(NULL, &energy_ctrl->hw);
	if (!IS_ERR_OR_NULL(clk))
		return clk;

	pr_err("clk register failed\n");
	kfree(energy_ctrl);

	return NULL;
}

void __init gem5_clk_of_register_energy_ctrl(void)
{
	char name[14] = "cpu-cluster.";
	struct device_node *node = NULL;
	struct clk *clk;
	const u32 *val;
	int cluster_id = 0, len;

	if (!of_find_compatible_node(NULL, NULL, "arm,gem5-energy-ctrl")) {
		pr_debug("%s: No EC found, Exiting!!\n", __func__);
		return;
	}

	while ((node = of_find_node_by_name(node, "cluster"))) {
		val = of_get_property(node, "reg", &len);
		if (val && len == 4)
			cluster_id = be32_to_cpup(val);

		name[12] = cluster_id + '0';
		clk = gem5_clk_register_energy_ctrl(name, cluster_id);
		if (IS_ERR(clk))
			return;

		pr_debug("Registered clock '%s'\n", name);
		clk_register_clkdev(clk, NULL, name);
	}
}
CLK_OF_DECLARE(energy_ctrl, "arm,gem5-energy-ctrl", gem5_clk_of_register_energy_ctrl);
