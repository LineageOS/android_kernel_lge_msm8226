/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include "msm_sensor.h"
#define HI544_SENSOR_NAME "hi544"
#include <mach/board_lge.h>		//to use lge_get_board_revno()
DEFINE_MSM_MUTEX(hi544_mut);

static struct msm_sensor_ctrl_t hi544_s_ctrl;

static struct msm_sensor_power_setting hi544_power_setting_rev_a[] = {
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VIO,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
#if defined(CONFIG_MACH_MSM8926_E2_MPCS_US) || defined(CONFIG_MACH_MSM8926_E2_VZW) || defined(CONFIG_MACH_MSM8926_VFP_KR) \
	|| defined(CONFIG_MACH_MSM8926_E2_BELL_CA) || defined(CONFIG_MACH_MSM8926_E2_RGS_CA) || defined(CONFIG_MACH_MSM8926_E2_VTR_CA) \
	|| defined(CONFIG_MACH_MSM8926_T8LTE_ATT_US) || defined(CONFIG_MACH_MSM8926_T8LTE_TMO_US) || defined(CONFIG_MACH_MSM8926_T8LTE_ACG_US) || defined(CONFIG_MACH_MSM8926_T8LTE_USC_US)
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
#else
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VDIG,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
#endif
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_AF_PWDM,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1, // >= 16MCLK
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 17, // >= 10msec
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},

};
#if defined(CONFIG_MACH_MSM8926_E2_SPR_US)

static struct msm_sensor_power_setting hi544_power_setting_rev_b[] = {
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VIO,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{	//Rev.B Specific Setting
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_AF_PWDM,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1, // >= 16MCLK
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 17, // >= 10msec
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},

};
#endif

static struct v4l2_subdev_info hi544_subdev_info[] = {
	{
		.code = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt = 1,
		.order = 0,
	},
};

static const struct i2c_device_id hi544_i2c_id[] = {
	{HI544_SENSOR_NAME, (kernel_ulong_t)&hi544_s_ctrl},
	{ }
};

static int32_t msm_hi544_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &hi544_s_ctrl);
}

static struct i2c_driver hi544_i2c_driver = {
	.id_table = hi544_i2c_id,
	.probe  = msm_hi544_i2c_probe,
	.driver = {
		.name = HI544_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client hi544_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id hi544_dt_match[] = {
	{.compatible = "qcom,hi544", .data = &hi544_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, hi544_dt_match);

static struct platform_driver hi544_platform_driver = {
	.driver = {
		.name = "qcom,hi544",
		.owner = THIS_MODULE,
		.of_match_table = hi544_dt_match,
	},
};

static int32_t hi544_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(hi544_dt_match, &pdev->dev);
/* LGE_CHANGE_S : WBT, 2013-5-31, jonghwan.ko@lge.com */
	if(!match)
	{
		  pr_err(" %s failed ",__func__);
		  return -ENODEV;
	 }
/* LGE_CHANGE_E : WBT, 2013-5-31, jonghwan.ko@lge.com */
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init hi544_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);

#if defined(CONFIG_MACH_MSM8926_E2_SPR_US)
	switch(lge_get_board_revno()) {
		case HW_REV_A:
			printk("%s: Sensor power is set as Rev.A, line:%d\n", __func__, __LINE__);
			hi544_s_ctrl.power_setting_array.power_setting = hi544_power_setting_rev_a;
			hi544_s_ctrl.power_setting_array.size = ARRAY_SIZE(hi544_power_setting_rev_a);
			break;
		case HW_REV_B:
			printk("%s: Sensor power is set as Rev.B, line:%d\n", __func__, __LINE__);
			hi544_s_ctrl.power_setting_array.power_setting = hi544_power_setting_rev_b;
			hi544_s_ctrl.power_setting_array.size = ARRAY_SIZE(hi544_power_setting_rev_b);
			break;
		default:
			printk("%s: Sensor power is set as Rev.10, line:%d\n", __func__, __LINE__);
			hi544_s_ctrl.power_setting_array.power_setting = hi544_power_setting_rev_b;
			hi544_s_ctrl.power_setting_array.size = ARRAY_SIZE(hi544_power_setting_rev_b);
			break;
	}
#else
	printk("%s: Sensor power is set, line:%d\n", __func__, __LINE__);
	hi544_s_ctrl.power_setting_array.power_setting = hi544_power_setting_rev_a;
	hi544_s_ctrl.power_setting_array.size = ARRAY_SIZE(hi544_power_setting_rev_a);
#endif

	rc = platform_driver_probe(&hi544_platform_driver,
		hi544_platform_probe);
	if (!rc)
		return rc;
	pr_info("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&hi544_i2c_driver);
}

static struct msm_sensor_fn_t hi544_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	//.sensor_match_id = hi544_sensor_match_id,
};

static void __exit hi544_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (hi544_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&hi544_s_ctrl);
		platform_driver_unregister(&hi544_platform_driver);
	} else
		i2c_del_driver(&hi544_i2c_driver);
	return;
}

static struct msm_sensor_ctrl_t hi544_s_ctrl = {
	.sensor_i2c_client = &hi544_sensor_i2c_client,
	//LGE_CHANGE_S, jongkwon.chae, 2014.06.20, To separate power settings depending on HW revisions.
	//.power_setting_array.power_setting = hi544_power_setting,
	//.power_setting_array.size = ARRAY_SIZE(hi544_power_setting),
	//LGE_CHANGE_E, jongkwon.chae, 2014.06.20, To separate power settings depending on HW revisions.
	.msm_sensor_mutex = &hi544_mut,
	.sensor_v4l2_subdev_info = hi544_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(hi544_subdev_info),
	.func_tbl = &hi544_sensor_func_tbl,
};

module_init(hi544_init_module);
module_exit(hi544_exit_module);
MODULE_DESCRIPTION("hi544");
MODULE_LICENSE("GPL v2");

