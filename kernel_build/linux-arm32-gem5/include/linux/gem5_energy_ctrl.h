/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Copyright (C) 2013 - 2014 ARM Limited
 * Authors: Akash Bagdia <Akash.bagdia@arm.com>
 *          Vasileios Spiliopoulos <vasileios.spiliopoulos@arm.com>
 */

#ifndef _LINUX_GEM5_ENERGY_CTRL_H
#define _LINUX_GEM5_ENERGY_CTRL_H

/* Energy Controller */

extern bool gem5_energy_ctrl_check_loaded(void);
extern bool gem5_energy_ctrl_dvfs_enabled(void);
extern u32 gem5_energy_ctrl_get_trans_latency(void);
extern int gem5_energy_ctrl_get_opp_table(u32, u32 **, u32 **);
extern int gem5_energy_ctrl_get_performance(u32, u32 *);
extern int gem5_energy_ctrl_set_performance(u32, u32);

#endif
