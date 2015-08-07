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
#define IMX119_SENSOR_NAME "imx119"
DEFINE_MSM_MUTEX(imx119_mut);

static struct msm_sensor_ctrl_t imx119_s_ctrl;

/* LGE_CHANGE_S, jaehan.jeong, 2013.7.30, To separate power settings depending on HW revisions, [STARTS HERE] */
#if defined(CONFIG_MACH_MSM8226_G2MDS_OPEN_CIS) || defined(CONFIG_MACH_MSM8226_G2MDS_GLOBAL_COM) || defined(CONFIG_MACH_MSM8926_G2M_OPEN) || defined(CONFIG_MACH_MSM8926_G2M_VDF) || defined(CONFIG_MACH_MSM8926_G2M_GLOBAL) || defined(CONFIG_MACH_MSM8926_G2M_KR) || defined(CONFIG_MACH_MSM8226_G2MSS_GLOBAL_COM)
static struct msm_sensor_power_setting imx119_power_setting_rev_0[] = {
	 /* Set GPIO_RESET to low to disable power on reset*/
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
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
		.delay = 30,
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
		.delay = 1,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},

};

static struct msm_sensor_power_setting imx119_power_setting_rev_a[] = {
	 /* Set GPIO_RESET to low to disable power on reset*/
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
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
		.delay = 30,
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
		.delay = 1,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},

};

static struct msm_sensor_power_setting imx119_power_setting_rev_c[] = {
	 /* Set GPIO_RESET to low to disable power on reset*/
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 0,
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
		.delay = 30,
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
		.delay = 1,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},

};
#elif defined(CONFIG_MACH_MSM8926_B2L_ATT) || defined(CONFIG_MACH_MSM8926_B2LN_KR) || defined(CONFIG_MACH_MSM8926_X10_VZW) \
    || defined(CONFIG_MACH_MSM8926_JAGNM_ATT) || defined(CONFIG_MACH_MSM8926_JAGN_KR) || defined(CONFIG_MACH_MSM8926_JAGC_SPR) || defined(CONFIG_MACH_MSM8926_JAGNM_GLOBAL_COM) \
    || defined(CONFIG_MACH_MSM8926_JAGDSNM_CMCC_CN) || defined(CONFIG_MACH_MSM8926_JAGDSNM_CTC_CN) || defined(CONFIG_MACH_MSM8926_JAGDSNM_CUCC_CN) \
    || defined(CONFIG_MACH_MSM8226_JAG3GSS_GLOBAL_COM) || defined(CONFIG_MACH_MSM8226_JAG3GDS_GLOBAL_COM) || defined(CONFIG_MACH_MSM8926_VFP_KR) \
    || defined(CONFIG_MACH_MSM8926_JAGNM_RGS) || defined(CONFIG_MACH_MSM8926_JAGNM_TLS) || defined(CONFIG_MACH_MSM8926_JAGNM_VTR) || defined(CONFIG_MACH_MSM8926_JAGNM_BELL) || defined(CONFIG_MACH_MSM8926_JAGC_SPR)
static struct msm_sensor_power_setting imx119_power_setting_b2l_b2m[] = {
	 /* Set GPIO_RESET to low to disable power on reset*/
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 0,
	},
	{								//VIO, GPIO 113
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VIO,
		.config_val = GPIO_OUT_HIGH,
		.delay = 30,
	},
	{								//VANA, GPIO 62
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
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
		.delay = 1,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},

};
#elif defined(CONFIG_MACH_MSM8X10_W5_TRF_US)
static struct msm_sensor_power_setting imx119_power_setting_w5[] = {
	 /* Set GPIO_RESET to low to disable power on reset*/
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
	},
#if 0
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 0,
	},
#endif
	{								//VANA, GPIO 67
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 30,
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
		.delay = 1,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},

};

#else
static struct msm_sensor_power_setting imx119_power_setting_rev_0[] = {

	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
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
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 1,
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
		.delay = 1,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},

};

static struct msm_sensor_power_setting imx119_power_setting_rev_a[] = {
	 /* Set GPIO_RESET to low to disable power on reset*/
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
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
		.delay = 30,
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

/* LGE_CHANGE_E, jaehan.jeong, 2013.7.30, To separate power settings depending on HW revisions,  [ENDS HERE] */

static struct v4l2_subdev_info imx119_subdev_info[] = {
	{
		.code = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt = 1,
		.order = 0,
	},
};

static const struct i2c_device_id imx119_i2c_id[] = {
	{IMX119_SENSOR_NAME, (kernel_ulong_t)&imx119_s_ctrl},
	{ }
};

static int32_t msm_imx119_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &imx119_s_ctrl);
}

static struct i2c_driver imx119_i2c_driver = {
	.id_table = imx119_i2c_id,
	.probe  = msm_imx119_i2c_probe,
	.driver = {
		.name = IMX119_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client imx119_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id imx119_dt_match[] = {
	{.compatible = "qcom,imx119", .data = &imx119_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, imx119_dt_match);

static struct platform_driver imx119_platform_driver = {
	.driver = {
		.name = "qcom,imx119",
		.owner = THIS_MODULE,
		.of_match_table = imx119_dt_match,
	},
};

static int32_t imx119_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(imx119_dt_match, &pdev->dev);
	if(!match)
		return -EINVAL;
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

#if defined(CONFIG_MACH_MSM8226_G2MDS_OPEN_CIS) || defined(CONFIG_MACH_MSM8226_G2MDS_GLOBAL_COM) || defined(CONFIG_MACH_MSM8926_G2M_OPEN) || defined(CONFIG_MACH_MSM8926_G2M_VDF) || defined(CONFIG_MACH_MSM8926_G2M_GLOBAL) || defined(CONFIG_MACH_MSM8926_G2M_KR) || defined(CONFIG_MACH_MSM8226_G2MSS_GLOBAL_COM)
static void imx119_power_setting(void)
{
	hw_rev_type rev_type = 0;
	
      rev_type = lge_get_board_revno();

      pr_err("%s: Sensor power is set as Rev.0\n", __func__);
      
	switch(lge_get_board_revno()) {
		case HW_REV_0:
			imx119_s_ctrl.power_setting_array.power_setting = imx119_power_setting_rev_0;
			imx119_s_ctrl.power_setting_array.size = ARRAY_SIZE(imx119_power_setting_rev_0);
			break;
		case HW_REV_A:
		case HW_REV_B:
			imx119_s_ctrl.power_setting_array.power_setting = imx119_power_setting_rev_a;
			imx119_s_ctrl.power_setting_array.size = ARRAY_SIZE(imx119_power_setting_rev_a);
			break;
		case HW_REV_C:
			imx119_s_ctrl.power_setting_array.power_setting = imx119_power_setting_rev_c;
			imx119_s_ctrl.power_setting_array.size = ARRAY_SIZE(imx119_power_setting_rev_c);
			break;
		default:
			printk("%s: Sensor power is set as Default mode. Rev.%d\n", __func__,rev_type);
			imx119_s_ctrl.power_setting_array.power_setting = imx119_power_setting_rev_a;
			imx119_s_ctrl.power_setting_array.size = ARRAY_SIZE(imx119_power_setting_rev_a);
			break;
	}

}
#elif defined(CONFIG_MACH_MSM8926_B2L_ATT) || defined(CONFIG_MACH_MSM8926_B2LN_KR) || defined(CONFIG_MACH_MSM8926_X10_VZW) \
    || defined(CONFIG_MACH_MSM8926_JAGNM_ATT) || defined(CONFIG_MACH_MSM8926_JAGN_KR) || defined(CONFIG_MACH_MSM8926_JAGC_SPR) || defined(CONFIG_MACH_MSM8926_JAGNM_GLOBAL_COM) \
    || defined(CONFIG_MACH_MSM8926_JAGDSNM_CMCC_CN) || defined(CONFIG_MACH_MSM8926_JAGDSNM_CTC_CN) || defined(CONFIG_MACH_MSM8926_JAGDSNM_CUCC_CN) \
    || defined(CONFIG_MACH_MSM8226_JAG3GSS_GLOBAL_COM) || defined(CONFIG_MACH_MSM8226_JAG3GDS_GLOBAL_COM) || defined(CONFIG_MACH_MSM8926_VFP_KR) \
    || defined(CONFIG_MACH_MSM8926_JAGNM_RGS) || defined(CONFIG_MACH_MSM926_JAGNM_TLS) || defined(CONFIG_MACH_MSM8926_JAGNM_VTR) || defined(CONFIG_MACH_MSM8926_JAGNM_BELL) || defined(CONFIG_MACH_MSM8926_JAGC_SPR)
static void imx119_power_setting(void)
{
	pr_err("%s: Sensor power is set as imx119_power_setting_b2l_b2m[]\n", __func__);
	imx119_s_ctrl.power_setting_array.power_setting = imx119_power_setting_b2l_b2m;
	imx119_s_ctrl.power_setting_array.size = ARRAY_SIZE(imx119_power_setting_b2l_b2m);
}
#elif defined(CONFIG_MACH_MSM8X10_W5_TRF_US)
static void imx119_power_setting(void)
{
	pr_err("%s: Sensor power is set as imx119_power_setting_w5[]\n", __func__);
	imx119_s_ctrl.power_setting_array.power_setting = imx119_power_setting_w5;
	imx119_s_ctrl.power_setting_array.size = ARRAY_SIZE(imx119_power_setting_w5);
}
#else
static void imx119_power_setting(void)
{
	hw_rev_type rev_type = 0;
	
      rev_type = lge_get_board_revno();
	switch(lge_get_board_revno()) {
		case HW_REV_0:
			pr_err("%s: Sensor power is set as Rev.0\n", __func__);
			imx119_s_ctrl.power_setting_array.power_setting = imx119_power_setting_rev_0;
			imx119_s_ctrl.power_setting_array.size = ARRAY_SIZE(imx119_power_setting_rev_0);
			break;
		case HW_REV_A:
		case HW_REV_B:
		default:
			printk("%s: Sensor power is set as Rev. %d\n", __func__,rev_type);
			imx119_s_ctrl.power_setting_array.power_setting = imx119_power_setting_rev_a;
			imx119_s_ctrl.power_setting_array.size = ARRAY_SIZE(imx119_power_setting_rev_a);
			break;
	}

}
#endif

static int __init imx119_init_module(void)
{
	int32_t rc = 0;
	
	pr_err("%s:%d\n", __func__, __LINE__);

	imx119_power_setting();

	rc = platform_driver_probe(&imx119_platform_driver,
		imx119_platform_probe);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&imx119_i2c_driver);
}

static void __exit imx119_exit_module(void)
{
	pr_err("%s:%d\n", __func__, __LINE__);
	if (imx119_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&imx119_s_ctrl);
		platform_driver_unregister(&imx119_platform_driver);
	} else
		i2c_del_driver(&imx119_i2c_driver);
	return;
}

static struct msm_sensor_ctrl_t imx119_s_ctrl = {
	.sensor_i2c_client = &imx119_sensor_i2c_client,
/* LGE_CHANGE_S, jaehan.jeong, 2013.7.30, To separate power settings depending on HW revisions, [STARTS HERE] */
/*
	.power_setting_array.power_setting = imx119_power_setting,
	.power_setting_array.size = ARRAY_SIZE(imx119_power_setting),
 */
/* LGE_CHANGE_E, jaehan.jeong, 2013.7.30, To separate power settings depending on HW revisions,  [ENDS HERE] */
	.msm_sensor_mutex = &imx119_mut,
	.sensor_v4l2_subdev_info = imx119_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(imx119_subdev_info),
};

module_init(imx119_init_module);
module_exit(imx119_exit_module);
MODULE_DESCRIPTION("imx119");
MODULE_LICENSE("GPL v2");
