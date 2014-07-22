/*
 * mms_ts.h - Platform data for Melfas MMS-series touch driver
 *
 * Copyright (C) 2013 Melfas Inc.
 * Author: DVK team <dvk@melfas.com>
 *
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#ifndef _LINUX_MMS_TOUCH_H
#define _LINUX_MMS_TOUCH_H

#define DRIVER_NAME "touch_dev"

#define MAX_NUM_OF_KEY 8

struct mms_ts_platform_data {
	int	max_x;
	int	max_y;

	int	gpio_sda;
	int	gpio_scl;
	int	gpio_int;
	int	gpio_vdd_en;
	
	int	key_code[MAX_NUM_OF_KEY];

	int auto_fw_update;
	int use_isp_erase;
	int use_vdd;
	int vdd_voltage;
	int use_vdd_i2c;
	int use_vdd_int;

	int tx_num;
	int rx_num;
	int key_num;
	bool self_diagnostic[3];
				
	const char *fw_name;
	const char *panel_spec_name;
	const char *product;
	char force_fw_upgrade;
};

#define TOUCH_INFO_MSG(fmt, args...) 	printk(KERN_ERR "[Touch] " fmt, ##args)

#endif /* _LINUX_MMS_TOUCH_H */
