/*
 * Copyright (c) 2013, Francisco Franco <franciscofranco.1990@gmail.com>. 
 *
 * Small algorithm changes for more performance
 * Copyright (c) 2014, Alexander Christ <alex.christ@hotmail.de>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Simple no bullshit hot[un]plug driver for SMP
 */

/*
 * TODO   - Enable sysfs entries for better tuning
 *        - Add Thermal Throttle Driver
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cpu.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/notifier.h>
#include <linux/cpufreq.h>
#include <linux/module.h>
#include <linux/delay.h>

#define DEFAULT_FIRST_LEVEL	85
#define DEFAULT_SECOND_LEVEL	75
#define HIGH_LOAD_COUNTER	25
#define SAMPLING_RATE_MS	500

struct cpu_stats
{
	unsigned int default_first_level;
	unsigned int default_second_level;
	unsigned long timestamp;

	/* For the three hot-plug-able Cores */
	unsigned int counter[2];
	unsigned int cpu_load_stats[3];

};

struct cpu_load_data {
        u64 prev_cpu_idle;
        u64 prev_cpu_wall;
};

static DEFINE_PER_CPU(struct cpu_load_data, cpuload);

static struct cpu_stats stats;
static struct workqueue_struct *wq;
static struct delayed_work decide_hotplug;

static unsigned long queue_sampling;

static inline int get_cpu_load(unsigned int cpu)
{
	struct cpu_load_data *pcpu = &per_cpu(cpuload, cpu);
	struct cpufreq_policy policy;
	u64 cur_wall_time, cur_idle_time;
	unsigned int idle_time, wall_time;
	unsigned int cur_load;

	cpufreq_get_policy(&policy, cpu);

	cur_idle_time = get_cpu_idle_time(cpu, &cur_wall_time, true);

	wall_time = (unsigned int) (cur_wall_time - pcpu->prev_cpu_wall);
	pcpu->prev_cpu_wall = cur_wall_time;

	idle_time = (unsigned int) (cur_idle_time - pcpu->prev_cpu_idle);
	pcpu->prev_cpu_idle = cur_idle_time;

	if (unlikely(!wall_time || wall_time < idle_time))
		return 0;

	cur_load = 100 * (wall_time - idle_time) / wall_time;

	return (cur_load * policy.cur) / policy.max;
}

/*
 * Returns the average load for all currently onlined cpus
 */

static int get_load_for_all_cpu(void)
{
	int cpu;
	int load = 0;

	for_each_online_cpu(cpu) {

		stats.cpu_load_stats[cpu] = 0;
		stats.cpu_load_stats[cpu] = get_cpu_load(cpu);
		load = load + stats.cpu_load_stats[cpu];
	}
	
	load = (unsigned int) (load / num_online_cpus());	
	return load;
}

/*
 * Calculates the load for a given cpu
 */ 

static void calculate_load_for_cpu(int cpu) 
{
	struct cpufreq_policy policy;

	for_each_online_cpu(cpu) {
		cpufreq_get_policy(&policy, cpu);
		/*  
		 * We are above our threshold, so update our counter for cpu.
		 * Consider this only, if we are on our max frequency
		 */
		if (get_cpu_load(cpu) >= stats.default_first_level &&
			get_load_for_all_cpu() >= stats.default_second_level
			&& likely(stats.counter[cpu] < HIGH_LOAD_COUNTER)
			&& cpufreq_quick_get(cpu) == policy.max) {
				stats.counter[cpu] += 2;
		}

		else {
			if (stats.counter[cpu] > 0)
				stats.counter[cpu]--;
		}

		/* Reset CPU */
		if (cpu)
			break;
	}	

}

/**
 * Simple load based decision algorithm to determ
 * how many cores should be on- or offlined
 */

static void __ref decide_hotplug_func(struct work_struct *work)
{
	int i, j;
	int current_cpu = 0;

	/* Do load calculation for each cpu counter */

	for (i = 0, j = 2; i < 2; i++, j++) {
		calculate_load_for_cpu(i);

		if (stats.counter[i] >= 10 && get_load_for_all_cpu() >= stats.default_second_level
			&& (num_online_cpus() != num_possible_cpus())) {
			if (!cpu_online(j)) {
				printk("[Hot-Plug]: CPU%u ready for onlining\n", j);
				cpu_up(j);
				stats.timestamp = jiffies;
			}
		}
		else {
			calculate_load_for_cpu(i);

			/* Prevent fast on-/offlining */ 
			if (time_is_after_jiffies(stats.timestamp + (HZ * 3))) {
				/* Rearm you work_queue immediatly */
				queue_delayed_work_on(0, wq, &decide_hotplug, queue_sampling);
			}
			else {
				if (stats.counter[i] > 0 && cpu_online(j)) {
						
						/*
						 * Decide which core should be offlined
						 */

						if (get_cpu_load(j) > get_cpu_load(j+1) 
								&& cpu_online(j+1))
							current_cpu = j + 1;
						else if (get_cpu_load(j) > get_cpu_load(j-1) 
								&& cpu_online(j-1) && j-1 != 1)
							current_cpu = j - 1;
						else
							current_cpu = j;
						
						printk("[Hot-Plug]: CPU%u ready for offlining\n", current_cpu);	
						cpu_down(current_cpu);
						stats.timestamp = jiffies;
				}
			}
		}
	}
	
	/* Make a dedicated work_queue */
	queue_delayed_work_on(0, wq, &decide_hotplug, queue_sampling);
}

int __init falcon_hotplug_init(void)
{
	pr_info("Falcon Hotplug driver started.\n");
    
	/* init everything here */
	stats.default_first_level = DEFAULT_FIRST_LEVEL;
	stats.default_second_level = DEFAULT_SECOND_LEVEL;
	queue_sampling = msecs_to_jiffies(SAMPLING_RATE_MS);
	
	/* Resetting Counters */
	stats.counter[0] = 0;
	stats.counter[1] = 0;
	stats.timestamp = jiffies;

	wq = create_singlethread_workqueue("falcon_hotplug_workqueue");
    
	if (!wq)
		return -ENOMEM;
    
	INIT_DELAYED_WORK(&decide_hotplug, decide_hotplug_func);
	queue_delayed_work_on(0, wq, &decide_hotplug, queue_sampling);
    
	return 0;
}
MODULE_AUTHOR("Francisco Franco <franciscofranco.1990@gmail.com>, "
	      "Alexander Christ <alex.christ@hotmail.de");
MODULE_DESCRIPTION("Simple SMP hotplug driver");
MODULE_LICENSE("GPL");

late_initcall(falcon_hotplug_init);


