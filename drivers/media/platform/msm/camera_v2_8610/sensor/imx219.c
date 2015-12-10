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
#include <mach/board_lge.h>		//to use lge_get_board_revno()
#define IMX219_SENSOR_NAME "imx219"

#define CONFIG_IMX219_DEBUG
#undef CDBG
#ifdef CONFIG_IMX219_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

DEFINE_MSM_MUTEX(imx219_mut);

static struct msm_sensor_ctrl_t imx219_s_ctrl;
#if defined(CONFIG_MACH_MSM8926_B2L_ATT) || defined(CONFIG_MACH_MSM8926_X10_VZW) || defined(CONFIG_MACH_MSM8926_JAGNM_ATT)
static struct msm_sensor_power_setting imx219_power_setting[] = {
	 /* Set GPIO_RESET to low to disable power on reset*/
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VIO,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_AF_PWDM,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};
#endif
/*                                                                                                             */
static struct msm_sensor_power_setting imx219_power_setting_rev_0[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VAF,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 30,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

#if !defined(CONFIG_MACH_MSM8X10_W6)
static struct msm_sensor_power_setting imx219_power_setting_rev_a[] = {
	 /* Set GPIO_RESET to low to disable power on reset*/
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 0,
	},
	{								//VANA, GPIO 62
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{								//VIO, GPIO 113
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VIO,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{                               //MCLK order is changed. 20140103. younjung.park
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 30,
	},

	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};
#endif
#if defined(CONFIG_MACH_MSM8X10_W5_GLOBAL_COM) || defined(CONFIG_MACH_MSM8X10_W5DS_GLOBAL_COM)|| defined(CONFIG_MACH_MSM8X10_W5TS_GLOBAL_COM)|| defined(CONFIG_MACH_MSM8X10_W5N_GLOBAL_COM) || defined(CONFIG_MACH_MSM8X10_W6)
static struct msm_sensor_power_setting imx219_power_setting_on_8x10[] = {
	 /* Set GPIO_RESET to low to disable power on reset*/
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},

	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 1,
	},

	{										//VANA2v8, VDVDD1v2 LDO
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
#if 0
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VDIG,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
#endif
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 1, //TODO : >= 16MCLK ?
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};
#endif
/*                                                                                                            */

static struct v4l2_subdev_info imx219_subdev_info[] = {
	{
		.code = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt = 1,
		.order = 0,
	},
};

static const struct i2c_device_id imx219_i2c_id[] = {
	{IMX219_SENSOR_NAME, (kernel_ulong_t)&imx219_s_ctrl},
	{ }
};

static int32_t msm_imx219_i2c_probe(struct i2c_client *client,
       const struct i2c_device_id *id)
{
       return msm_sensor_i2c_probe(client, id, &imx219_s_ctrl);
}

static struct i2c_driver imx219_i2c_driver = {
	.id_table = imx219_i2c_id,
	.probe  = msm_imx219_i2c_probe,
	.driver = {
		.name = IMX219_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client imx219_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id imx219_dt_match[] = {
	{.compatible = "qcom,imx219", .data = &imx219_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, imx219_dt_match);

static struct platform_driver imx219_platform_driver = {
	.driver = {
		.name = "qcom,imx219",
		.owner = THIS_MODULE,
		.of_match_table = imx219_dt_match,
	},
};

static int32_t imx219_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;

	match = of_match_device(imx219_dt_match, &pdev->dev);

	if(!match)
		return -EINVAL;

	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init imx219_init_module(void)
{
	int32_t rc = 0;
	hw_rev_type rev_type = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
      rev_type = lge_get_board_revno();
#if 1//                         
	switch(rev_type) {
		case HW_REV_0:
			printk("%s: Sensor power is set as Rev.0\n", __func__);
			imx219_s_ctrl.power_setting_array.power_setting = imx219_power_setting_rev_0;
			imx219_s_ctrl.power_setting_array.size = ARRAY_SIZE(imx219_power_setting_rev_0);
			break;
		case HW_REV_A:
#if defined(CONFIG_MACH_MSM8X10_W5_GLOBAL_COM) || defined(CONFIG_MACH_MSM8X10_W5DS_GLOBAL_COM)|| defined(CONFIG_MACH_MSM8X10_W5TS_GLOBAL_COM)|| defined(CONFIG_MACH_MSM8X10_W5N_GLOBAL_COM)
			printk("%s: Sensor power is set as Rev. %d\n", __func__,rev_type);
			imx219_s_ctrl.power_setting_array.power_setting = imx219_power_setting_rev_a;
			imx219_s_ctrl.power_setting_array.size = ARRAY_SIZE(imx219_power_setting_rev_a);
#elif defined(CONFIG_MACH_MSM8X10_W6)
			printk("%s: HW rev is %d. But sensor power is set as Rev.B \n", __func__,rev_type);
			imx219_s_ctrl.power_setting_array.power_setting = imx219_power_setting_on_8x10;
			imx219_s_ctrl.power_setting_array.size = ARRAY_SIZE(imx219_power_setting_on_8x10);
#endif
		case HW_REV_B:
		default:
			printk("%s: Sensor power is set as Rev.%d(Line:%d)\n", __func__,rev_type, __LINE__);
#if defined(CONFIG_MACH_MSM8X10_W5_GLOBAL_COM) || defined(CONFIG_MACH_MSM8X10_W5DS_GLOBAL_COM)|| defined(CONFIG_MACH_MSM8X10_W5TS_GLOBAL_COM) || defined(CONFIG_MACH_MSM8X10_W5N_GLOBAL_COM) || defined(CONFIG_MACH_MSM8X10_W6)
			imx219_s_ctrl.power_setting_array.power_setting = imx219_power_setting_on_8x10;
			imx219_s_ctrl.power_setting_array.size = ARRAY_SIZE(imx219_power_setting_on_8x10);
#else
			imx219_s_ctrl.power_setting_array.power_setting = imx219_power_setting_rev_a;
			imx219_s_ctrl.power_setting_array.size = ARRAY_SIZE(imx219_power_setting_rev_a);
#endif
			break;
	}
#endif

	rc = platform_driver_probe(&imx219_platform_driver,
		imx219_platform_probe);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&imx219_i2c_driver);
}

static void __exit imx219_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);

	if (imx219_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&imx219_s_ctrl);
		platform_driver_unregister(&imx219_platform_driver);
	} else {
		i2c_del_driver(&imx219_i2c_driver);
	}
	return;
}

static struct msm_sensor_ctrl_t imx219_s_ctrl = {
	.sensor_i2c_client = &imx219_sensor_i2c_client,
#if defined(CONFIG_MACH_MSM8926_B2L_ATT) || defined(CONFIG_MACH_MSM8926_X10_VZW) || defined(CONFIG_MACH_MSM8926_JAGNM_ATT)
	.power_setting_array.power_setting = imx219_power_setting,
	.power_setting_array.size = ARRAY_SIZE(imx219_power_setting),
#endif
	.msm_sensor_mutex = &imx219_mut,
	.sensor_v4l2_subdev_info = imx219_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(imx219_subdev_info),
};

module_init(imx219_init_module);
module_exit(imx219_exit_module);
MODULE_DESCRIPTION("imx219");
MODULE_LICENSE("GPL v2");
