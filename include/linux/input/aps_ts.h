
#ifndef _LINUX_APS_TOUCH_H
#define _LINUX_APS_TOUCH_H
#define LGD_TOUCH_NAME "lgd_melfas_incell_touch"

struct aps_ts_platform_data {
	int	max_x;
	int	max_y;

	int	gpio_irq;
	int gpio_reset;
	unsigned int 	irq_flags;
	unsigned int 	reset_flags;
	char	*name;				/* input drv name */
};

#endif /* _LINUX_APS_TOUCH_H */
