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
#define HI543_SENSOR_NAME "hi543"
#include <mach/board_lge.h>		//to use lge_get_board_revno()
DEFINE_MSM_MUTEX(hi543_mut);

static struct msm_sensor_ctrl_t hi543_s_ctrl;
#if !defined(CONFIG_MACH_MSM8X10_W6)
static struct msm_sensor_power_setting hi543_power_setting_rev_a[] = {

	{  /* Set GPIO_RESET to low to disable power on reset*/
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	#if defined(CONFIG_MACH_MSM8X10_W5)
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 0,
	},
	#else
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VIO,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
	#endif
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
#if defined(CONFIG_MACH_MSM8X10_W5)
#if defined(CONFIG_MACH_MSM8X10_W5C_VZW) || defined(CONFIG_MACH_MSM8X10_W5C_SPR_US) || defined(CONFIG_MACH_MSM8X10_W5C_TRF_US)
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
#endif
#else //temp, comment out for sleep current, will be used from revB, 2013-08-30, yt.jeon@lge.com
#if defined(CONFIG_MACH_MSM8226_E8WIFI) || defined(CONFIG_MACH_MSM8226_E9WIFI) || defined(CONFIG_MACH_MSM8226_E9WIFIN)
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
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
#if defined(CONFIG_MACH_MSM8226_E9WIFI) || defined(CONFIG_MACH_MSM8226_E9WIFIN) || defined(CONFIG_MACH_MSM8226_E8WIFI) || defined(CONFIG_MACH_MSM8926_E8LTE) || defined(CONFIG_MACH_MSM8926_E7LTE_ATT_US) || defined(CONFIG_MACH_MSM8926_E7LTE_VZW_US) || defined (CONFIG_MACH_MSM8926_E7LTE_USC_US) || defined(CONFIG_MACH_MSM8926_T8LTE_ATT_US) || defined(CONFIG_MACH_MSM8926_E9LTE_VZW_US)
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VAF,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
#else
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
#endif
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
		.delay = 1, // >= 16MCLK
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},

};
#endif
#if defined(CONFIG_MACH_MSM8X10_W5) || defined(CONFIG_MACH_MSM8X10_W6)
#if !defined(CONFIG_MACH_MSM8X10_W5C_VZW) && !defined(CONFIG_MACH_MSM8X10_W5C_SPR_US) && !defined(CONFIG_MACH_MSM8X10_W5C_TRF_US)
static struct msm_sensor_power_setting hi543_power_setting_rev_b[] = {
	 {	/* Set GPIO_RESET to low to disable power on reset*/
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
#if 0
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VIO,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
#endif
	{
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
#endif
/* LGE_CHANGE_S,  2014.02.27, Add power settings for E9 Rev A. sangwoo25.park@lge.com */
#if defined(CONFIG_MACH_MSM8226_E9WIFI) || defined(CONFIG_MACH_MSM8226_E9WIFIN)
static struct msm_sensor_power_setting hi543_power_setting_e9_rev_a[] = {
	{	//mt9m114 digital
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 15,
	},
	{  /* Set GPIO_RESET to low to disable power on reset*/
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{	//mt9m114 & hi543 vio
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VIO,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VAF,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
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
		.delay = 1, // >= 16MCLK
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};
#endif
/* LGE_CHANGE_E,  2014.02.27, Add power settings for E9 Rev A. sangwoo25.park@lge.com */

/* LGE_CHANGE_S,  2014.04.03, To separate power settings for E7LTE. sangwoo25.park@lge.com */
#if defined(CONFIG_MACH_MSM8926_E7LTE_ATT_US) || defined(CONFIG_MACH_MSM8926_E7LTE_VZW_US) || defined (CONFIG_MACH_MSM8926_E7LTE_USC_US) || defined (CONFIG_MACH_MSM8926_E8LTE) || defined(CONFIG_MACH_MSM8926_T8LTE_ATT_US) || defined(CONFIG_MACH_MSM8926_E9LTE_VZW_US)
static struct msm_sensor_power_setting hi543_power_setting_e7lte_rev_b[] = {
	{  /* Set GPIO_RESET to low to disable power on reset*/
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{	// DVDD& VIO
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VIO,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VAF,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
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
		.delay = 1, // >= 16MCLK
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};
#endif
/* LGE_CHANGE_E,  2014.04.03, To separate power settings for E7LTE. sangwoo25.park@lge.com */

#if defined(CONFIG_MACH_MSM8X10_W5C_VZW) || defined(CONFIG_MACH_MSM8X10_W5C_SPR_US) || defined(CONFIG_MACH_MSM8X10_W5C_TRF_US)
static struct msm_sensor_power_setting hi543_power_setting_w5c[] = {
	 {	/* Set GPIO_RESET to low to disable power on reset*/
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
#if 0
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VIO,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
#endif
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VAF,
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

static struct v4l2_subdev_info hi543_subdev_info[] = {
	{
		.code = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt = 1,
		.order = 0,
	},
};

static const struct i2c_device_id hi543_i2c_id[] = {
	{HI543_SENSOR_NAME, (kernel_ulong_t)&hi543_s_ctrl},
	{ }
};

static int32_t msm_hi543_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &hi543_s_ctrl);
}

static struct i2c_driver hi543_i2c_driver = {
	.id_table = hi543_i2c_id,
	.probe  = msm_hi543_i2c_probe,
	.driver = {
		.name = HI543_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client hi543_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id hi543_dt_match[] = {
	{.compatible = "qcom,hi543", .data = &hi543_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, hi543_dt_match);

static struct platform_driver hi543_platform_driver = {
	.driver = {
		.name = "qcom,hi543",
		.owner = THIS_MODULE,
		.of_match_table = hi543_dt_match,
	},
};

static int32_t hi543_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(hi543_dt_match, &pdev->dev);
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

static int __init hi543_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
/* LGE_CHANGE_S, yt.jeon, 2013.10.07, To separate power settings depending on HW revisions. */
#if defined(CONFIG_MACH_MSM8X10_W5)
	switch(lge_get_board_revno()) {
		case HW_REV_A:
			printk("%s: Sensor power is set as Rev.A, line(%d)\n", __func__, __LINE__);
			hi543_s_ctrl.power_setting_array.power_setting = hi543_power_setting_rev_a;
			hi543_s_ctrl.power_setting_array.size = ARRAY_SIZE(hi543_power_setting_rev_a);
			break;
		case HW_REV_B:
			#if defined(CONFIG_MACH_MSM8X10_W5C_VZW)
			printk("%s: (W5C_VZW)Sensor power is set as over Rev.A, line(%d)\n", __func__, __LINE__);
			hi543_s_ctrl.power_setting_array.power_setting = hi543_power_setting_rev_a;
			hi543_s_ctrl.power_setting_array.size = ARRAY_SIZE(hi543_power_setting_rev_a);
			break;
			#elif defined(CONFIG_MACH_MSM8X10_W5C_SPR_US) || defined(CONFIG_MACH_MSM8X10_W5C_TRF_US)
			printk("%s: Sensor power is set as Rev.B, line(%d)\n", __func__, __LINE__);
			hi543_s_ctrl.power_setting_array.power_setting = hi543_power_setting_w5c;
			hi543_s_ctrl.power_setting_array.size = ARRAY_SIZE(hi543_power_setting_w5c);
			break;
			#endif
		case HW_REV_C:
			#if defined(CONFIG_MACH_MSM8X10_W5C_VZW) || defined(CONFIG_MACH_MSM8X10_W5C_SPR_US) || defined(CONFIG_MACH_MSM8X10_W5C_TRF_US)
			printk("%s: (W5C_VZW)Sensor power is set as Rev.C, line(%d)\n", __func__, __LINE__);
			hi543_s_ctrl.power_setting_array.power_setting = hi543_power_setting_w5c;
			hi543_s_ctrl.power_setting_array.size = ARRAY_SIZE(hi543_power_setting_w5c);
			break;
			#else
			printk("%s: Sensor power is set as Rev.B, line(%d)\n", __func__, __LINE__);
			hi543_s_ctrl.power_setting_array.power_setting = hi543_power_setting_rev_b;
			hi543_s_ctrl.power_setting_array.size = ARRAY_SIZE(hi543_power_setting_rev_b);
			break;
			#endif
		default:
			#if defined(CONFIG_MACH_MSM8X10_W5C_VZW) || defined(CONFIG_MACH_MSM8X10_W5C_SPR_US) || defined(CONFIG_MACH_MSM8X10_W5C_TRF_US)
			printk("%s: (W5C_VZW)Sensor power is set as over Rev.C, line(%d)\n", __func__, __LINE__);
			hi543_s_ctrl.power_setting_array.power_setting = hi543_power_setting_w5c;
			hi543_s_ctrl.power_setting_array.size = ARRAY_SIZE(hi543_power_setting_w5c);
			break;
			#else
			printk("%s: Sensor power is set as Rev.B, line(%d)\n", __func__, __LINE__);
			hi543_s_ctrl.power_setting_array.power_setting = hi543_power_setting_rev_b;
			hi543_s_ctrl.power_setting_array.size = ARRAY_SIZE(hi543_power_setting_rev_b);
			break;
			#endif
	}
#elif defined(CONFIG_MACH_MSM8X10_W6)
	switch(lge_get_board_revno()) {
		case HW_REV_A:
		default:
			printk("%s: Sensor power is set as Rev.b(%d)\n", __func__, __LINE__);
			hi543_s_ctrl.power_setting_array.power_setting = hi543_power_setting_rev_b;
			hi543_s_ctrl.power_setting_array.size = ARRAY_SIZE(hi543_power_setting_rev_b);
			break;
	}
/* LGE_CHANGE_S,  2014.02.27, To separate power settings for E9. sangwoo25.park@lge.com */
#elif defined(CONFIG_MACH_MSM8226_E9WIFI) || defined(CONFIG_MACH_MSM8226_E9WIFIN)
	switch(lge_get_board_revno()) {
		case HW_REV_A:
		case HW_REV_B:
			printk("%s: E9 Sensor power is set as Rev.A\n", __func__);
			hi543_s_ctrl.power_setting_array.power_setting = hi543_power_setting_e9_rev_a;
			hi543_s_ctrl.power_setting_array.size = ARRAY_SIZE(hi543_power_setting_e9_rev_a);
			break;
		default:
			printk("%s: Sensor power is set as Rev.0 or Rev 1.0\n", __func__);
			hi543_s_ctrl.power_setting_array.power_setting = hi543_power_setting_rev_a;
			hi543_s_ctrl.power_setting_array.size = ARRAY_SIZE(hi543_power_setting_rev_a);
			break;
	}
/* LGE_CHANGE_E,  2014.02.27, To separate power settings for E9. sangwoo25.park@lge.com */
/* LGE_CHANGE_S,  2014.04.03, To separate power settings for E7LTE. sangwoo25.park@lge.com */
#elif defined(CONFIG_MACH_MSM8926_E7LTE_ATT_US) || defined(CONFIG_MACH_MSM8926_E7LTE_VZW_US) || defined (CONFIG_MACH_MSM8926_E7LTE_USC_US) || defined (CONFIG_MACH_MSM8926_E8LTE) || defined(CONFIG_MACH_MSM8926_T8LTE_ATT_US) || defined(CONFIG_MACH_MSM8926_E9LTE_VZW_US)
	switch(lge_get_board_revno()) {
		case HW_REV_0:
		case HW_REV_A:
			printk("%s: E7LTE Sensor power is set as Rev.A\n", __func__);
			hi543_s_ctrl.power_setting_array.power_setting = hi543_power_setting_rev_a;
			hi543_s_ctrl.power_setting_array.size = ARRAY_SIZE(hi543_power_setting_rev_a);
			break;
		default:
			printk("%s: E7LTE Sensor power is set as Rev.B\n", __func__);
			hi543_s_ctrl.power_setting_array.power_setting = hi543_power_setting_e7lte_rev_b;
			hi543_s_ctrl.power_setting_array.size = ARRAY_SIZE(hi543_power_setting_e7lte_rev_b);
		break;
	}
/* LGE_CHANGE_E,  2014.04.03, To separate power settings for E7LTE. sangwoo25.park@lge.com */
#else
	switch(lge_get_board_revno()) {
		case HW_REV_A:
			printk("%s: Sensor power is set as Rev.A\n", __func__);
			hi543_s_ctrl.power_setting_array.power_setting = hi543_power_setting_rev_a;
			hi543_s_ctrl.power_setting_array.size = ARRAY_SIZE(hi543_power_setting_rev_a);
			break;
		default:
			printk("%s: Sensor power is set as Rev.A\n", __func__);
			hi543_s_ctrl.power_setting_array.power_setting = hi543_power_setting_rev_a;
			hi543_s_ctrl.power_setting_array.size = ARRAY_SIZE(hi543_power_setting_rev_a);
			break;
	}
#endif
/* LGE_CHANGE_E, yt.jeon, 2013.10.07, To separate power settings depending on HW revisions. */
	rc = platform_driver_probe(&hi543_platform_driver,
		hi543_platform_probe);
	if (!rc)
		return rc;
	pr_info("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&hi543_i2c_driver);
}

int32_t hi543_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{

	int32_t rc = 0;
	uint16_t chipid[2];

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			s_ctrl->sensordata->slave_info->sensor_id_reg_addr,
			&chipid[0], MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: %d %s: read id failed\n", __func__,__LINE__,
			s_ctrl->sensordata->sensor_name);
		return rc;
	}
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			s_ctrl->sensordata->slave_info->sensor_id_reg_addr|0x0001,
			&chipid[1], MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: %d %s: read id failed\n", __func__,__LINE__,
			s_ctrl->sensordata->sensor_name);
		return rc;
	}

	chipid[1] = chipid[0] << 8 | chipid[1];

	if (chipid[1] != s_ctrl->sensordata->slave_info->sensor_id) {
		pr_err("%s read id 0x%x does not match with expected id 0x%x\n",__func__,
			chipid[1], s_ctrl->sensordata->slave_info->sensor_id);
		return -ENODEV;
	}

	return rc;

}

static struct msm_sensor_fn_t hi543_sensor_func_tbl = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = hi543_sensor_match_id,
};

static void __exit hi543_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (hi543_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&hi543_s_ctrl);
		platform_driver_unregister(&hi543_platform_driver);
	} else
		i2c_del_driver(&hi543_i2c_driver);
	return;
}

static struct msm_sensor_ctrl_t hi543_s_ctrl = {
	.sensor_i2c_client = &hi543_sensor_i2c_client,
/* LGE_CHANGE_S, yt.jeon, 2013.10.07, To separate power settings depending on HW revisions. */
	//.power_setting_array.power_setting = hi543_power_setting,
	//.power_setting_array.size = ARRAY_SIZE(hi543_power_setting),
/* LGE_CHANGE_E, yt.jeon, 2013.10.07, To separate power settings depending on HW revisions. */
	.msm_sensor_mutex = &hi543_mut,
	.sensor_v4l2_subdev_info = hi543_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(hi543_subdev_info),
	.func_tbl = &hi543_sensor_func_tbl,
};

module_init(hi543_init_module);
module_exit(hi543_exit_module);
MODULE_DESCRIPTION("hi543");
MODULE_LICENSE("GPL v2");

