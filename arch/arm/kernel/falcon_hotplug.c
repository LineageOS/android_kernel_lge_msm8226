/*
 * Copyright (c) 2013, Francisco Franco <franciscofranco.1990@gmail.com>. 
 *
 * Small algorithm changes for more performance/battery life
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
 * TODO   - Add Thermal Throttle Driver (if needed)
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cpu.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/cpufreq.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#ifdef CONFIG_POWERSUSPEND
#include <linux/powersuspend.h>
#endif


#define DEFAULT_FIRST_LEVEL	80
#define DEFAULT_SECOND_LEVEL	60
#define HIGH_LOAD_COUNTER	25
#define SAMPLING_RATE		10
#define DEFAULT_MIN_ONLINE	4
// #define FALCON_DEBUG

struct hotplug_data {
	/* The threshold level for the average load of all onlined cpus */
	unsigned int all_cpus_threshold;

	/* The default sampling rate for the hotplug driver */
	unsigned int hotplug_sampling;

	/* The minimum amount of time for a cpu to stay online */
	unsigned int min_online_time;

	/* The first threshold level for a single cpu */
	unsigned int single_cpu_threshold;

	unsigned long timestamp;

	/* For the three hot-plug-able Cores */
	unsigned int counter[2];
	unsigned int cpu_load_stats[3];
} *hot_data;

struct cpu_load_data {
        u64 prev_cpu_idle;
        u64 prev_cpu_wall;
};

static DEFINE_PER_CPU(struct cpu_load_data, cpuload);
static struct workqueue_struct *wq;
static struct delayed_work decide_hotplug;
static struct work_struct resume;
static struct work_struct suspend;

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

		hot_data->cpu_load_stats[cpu] = 0;
		hot_data->cpu_load_stats[cpu] = get_cpu_load(cpu);
		load = load + hot_data->cpu_load_stats[cpu];
	}
	
	return (load / num_online_cpus());
}

/*
 * Calculates the load for a given cpu
 */ 

static void calculate_load_for_cpu(int cpu) 
{
	struct cpufreq_policy policy;
	int avg_load;

	avg_load = get_load_for_all_cpu();

	for_each_online_cpu(cpu) {
		cpufreq_get_policy(&policy, cpu);
		/*  
		 * We are above our threshold, so update our counter for cpu.
		 * Consider this only, if we are on our max frequency
		 */
		if (get_cpu_load(cpu) >= hot_data->single_cpu_threshold &&
			avg_load >= hot_data->all_cpus_threshold
			&& likely(hot_data->counter[cpu] < HIGH_LOAD_COUNTER)
			&& cpufreq_quick_get(cpu) == policy.max) {
				hot_data->counter[cpu] += 2;
		}

		else {
			if (hot_data->counter[cpu] > 0)
				hot_data->counter[cpu]--;
		}

		/* Reset CPU */
		if (cpu)
			break;
	}	

}
/*
 * Finds the lowest operation core to offline
 */

static void put_cpu_down(int cpu) 
{
	int current_cpu = 0;

	/* Prevent fast on-/offlining */ 
	if (time_is_after_jiffies(hot_data->timestamp + (HZ * hot_data->min_online_time)))	
		return;

	/*
	 * Decide which core should be offlined
	 */

	/* No core was online anyway */
	if (!cpu_online(cpu)) {
		if ((!cpu_online(cpu + 1)) || (!cpu_online(cpu - 1)))
			return;
	}
	
	switch(cpu) {
		case 2:
			if (cpu_online(cpu) && !cpu_online(cpu + 1)) {
				current_cpu = cpu;
				break;
			} else if (!cpu_online(cpu) && cpu_online(cpu + 1)) {
				current_cpu = cpu + 1;
				break;
			} else if (cpu_online(cpu) && cpu_online(cpu + 1)) {
				if (get_cpu_load(cpu) >= get_cpu_load(cpu + 1)) {
					current_cpu = cpu + 1;
					break;
				} else {
					current_cpu = cpu;
					break;
				}
			}
		break;
		case 3:
			if (cpu_online(cpu - 1) && !cpu_online(cpu)) {
				current_cpu = cpu - 1;
				break;
			} else if (cpu_online(cpu) && cpu_online(cpu - 1)) {
				current_cpu = cpu;
				break;
			} else if (cpu_online(cpu) && cpu_online(cpu - 1)
				&& (cpu - 1) != 1) {
				if (get_cpu_load(cpu - 1) >= get_cpu_load(cpu)) {
					current_cpu = cpu;
					break;
				} else {
					current_cpu = cpu - 1;	
					break;
				}
			}
		break;
	}

#ifdef FALCON_DEBUG						
	printk("[Hot-Plug]: CPU%u ready for offlining\n", current_cpu);
#endif	
	cpu_down(current_cpu);
	hot_data->timestamp = jiffies;

}

/**
 * Simple load based decision algorithm to determ
 * how many cores should be on- or offlined
 */

static void __ref decide_hotplug_func(struct work_struct *work)
{
	int i, j;
	unsigned int current_online_cpus = num_online_cpus();

	/* Reschedule early if we don't need to bother about calculations */
	if (unlikely(current_online_cpus == 1)
		queue_delayed_work(system_power_efficient_wq, &decide_hotplug, msecs_to_jiffies(hot_data->hotplug_sampling * HZ));


	/* Do load calculation for each cpu counter */
	calculate_load_for_cpu(i);

	for (i = 0, j = 2; i < 2; i++, j++) {

		if (hot_data->counter[i] >= 10) {
			if (!cpu_online(j)) {
#ifdef FALCON_DEBUG
				printk("[Hot-Plug]: CPU%u ready for onlining\n", j);
#endif
				cpu_up(j);
				hot_data->timestamp = jiffies;
			}
		}
		else {
			if (hot_data->counter[i] >= 0) {
				put_cpu_down(j);	
			}
		}
	}
	
	/* Make a dedicated work_queue */
	queue_delayed_work_on(0, wq, &decide_hotplug, msecs_to_jiffies(hot_data->hotplug_sampling * HZ));
}

#ifdef CONFIG_POWERSUSPEND
static void suspend_func(struct work_struct *work)
{	 
	int cpu;

	/* cancel the hotplug work when the screen is off and flush the WQ */
	flush_workqueue(wq);
	cancel_delayed_work_sync(&decide_hotplug);
	cancel_work_sync(&resume);

	for_each_online_cpu(cpu) 
		if (cpu)
			cpu_down(cpu);

#ifdef FALCON_DEBUG
	pr_info("[Hot-Plug]: Early Suspend stopping Hotplug work. CPUs online: %d\n", num_online_cpus());
#endif
    
}

static void __ref resume_func(struct work_struct *work)
{
	/* Online only the second core */
	cpu_up(1);

	cancel_work_sync(&suspend);

	/* Resetting Counters */
	hot_data->counter[0] = 0;
	hot_data->counter[1] = 0;
	hot_data->timestamp = jiffies;

#ifdef FALCON_DEBUG
	pr_info("[Hot-Plug]: Late Resume starting Hotplug work. CPUs online: %d\n", num_online_cpus());
#endif
	queue_delayed_work_on(0, wq, &decide_hotplug, msecs_to_jiffies(hot_data->hotplug_sampling * HZ));
}

static void falcon_hotplug_suspend(struct power_suspend *h)
{
	queue_work(system_power_efficient_wq, &suspend);
}


static void falcon_hotplug_resume(struct power_suspend *h)
{
	queue_work(system_power_efficient_wq, &resume);
}

static struct power_suspend falcon_hotplug_power_suspend = {
	.suspend = falcon_hotplug_suspend,
	.resume	 = falcon_hotplug_resume,
};
#endif

/* Start sysfs entries */
static ssize_t show_first_threshold(struct kobject *kobj,
					struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", hot_data->single_cpu_threshold);
}

static ssize_t store_first_threshold(struct kobject *kobj,
					 struct attribute *attr,
					 const char *buf, size_t count)
{
	unsigned int val;

	sscanf(buf, "%u", &val);

	hot_data->single_cpu_threshold = val;
	return count;
}

static struct global_attr single_core_threshold_attr = __ATTR(single_core_threshold, 0664,
					show_first_threshold, store_first_threshold);

static ssize_t show_sampling_rate(struct kobject *kobj,
					struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", hot_data->hotplug_sampling);
}

static ssize_t store_sampling_rate(struct kobject *kobj,
					 struct attribute *attr,
					 const char *buf, size_t count)
{
	unsigned int val;

	sscanf(buf, "%u", &val);

	hot_data->hotplug_sampling = val;
	return count;
}

static struct global_attr hotplug_sampling_rate_attr = __ATTR(hotplug_sampling, 0664,
					show_sampling_rate, store_sampling_rate);

static ssize_t show_min_online_time(struct kobject *kobj,
					struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", hot_data->min_online_time);
}

static ssize_t store_min_online_time(struct kobject *kobj,
					 struct attribute *attr,
					 const char *buf, size_t count)
{
	unsigned int val;

	sscanf(buf, "%u", &val);

	hot_data->min_online_time = val;
	return count;
}

static struct global_attr min_online_time_attr = __ATTR(min_online_time, 0664,
					show_min_online_time, store_min_online_time);


static ssize_t show_all_cpus_threshold(struct kobject *kobj,
					struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", hot_data->all_cpus_threshold);
}

static ssize_t store_all_cpus_threshold(struct kobject *kobj,
					 struct attribute *attr,
					 const char *buf, size_t count)
{
	unsigned int val;

	sscanf(buf, "%u", &val);

	hot_data->all_cpus_threshold = val;
	return count;
}

static struct global_attr all_cpus_threshold_attr = __ATTR(all_cpus_threshold, 0664,
					show_all_cpus_threshold, store_all_cpus_threshold);

static struct attribute *falcon_hotplug_attributes[] = 
{
	&hotplug_sampling_rate_attr.attr,
	&single_core_threshold_attr.attr,
	&min_online_time_attr.attr,
	&all_cpus_threshold_attr.attr,
	NULL
};

static struct attribute_group hotplug_attr_group = 
{
	.attrs  = falcon_hotplug_attributes,
};

static struct kobject *hotplug_control_kobj;

int __init falcon_hotplug_init(void)
{
	int ret;
#ifdef FALCON_DEBUG
	pr_info("[Hot-Plug]: Falcon Hotplug driver started.\n");
#endif
	hot_data = kzalloc(sizeof(*hot_data), GFP_KERNEL);
	if (!hot_data)
		return -ENOMEM;

	hot_data->hotplug_sampling = SAMPLING_RATE;
	hot_data->min_online_time = DEFAULT_MIN_ONLINE;
	hot_data->single_cpu_threshold = DEFAULT_FIRST_LEVEL;
	hot_data->all_cpus_threshold = DEFAULT_SECOND_LEVEL;

	hotplug_control_kobj = kobject_create_and_add("hotplug_control", kernel_kobj);
	if (!hotplug_control_kobj) {
		pr_err("%s hotplug_control kobject create failed!\n", __FUNCTION__);
		return -ENOMEM;
	}

	ret = sysfs_create_group(hotplug_control_kobj,
			&hotplug_attr_group);
        if (ret) {
		pr_info("%s hotplug_control sysfs create failed!\n", __FUNCTION__);
		kobject_put(hotplug_control_kobj);
		return ret;
	}

	/* Resetting Counters */
	hot_data->counter[0] = 0;
	hot_data->counter[1] = 0;
	hot_data->timestamp = jiffies;

	wq = create_singlethread_workqueue("falcon_hotplug_workqueue");
    
	if (!wq)
		return -ENOMEM;

#ifdef CONFIG_POWERSUSPEND
	register_power_suspend(&falcon_hotplug_power_suspend);
	INIT_WORK(&resume, resume_func);
	INIT_WORK(&suspend, suspend_func);
#endif

	INIT_DELAYED_WORK(&decide_hotplug, decide_hotplug_func);
	queue_delayed_work_on(0, wq, &decide_hotplug, msecs_to_jiffies(20000));
	
	return 0;
}
MODULE_AUTHOR("Francisco Franco <franciscofranco.1990@gmail.com>, "
	      "Alexander Christ <alex.christ@hotmail.de");
MODULE_DESCRIPTION("Simple SMP hotplug driver");
MODULE_LICENSE("GPL");

late_initcall(falcon_hotplug_init);
