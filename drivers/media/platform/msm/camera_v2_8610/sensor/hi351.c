/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
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
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "hi351_reg.h"

#define CONFIG_MSMB_CAMERA_DEBUG 1

#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

#define HI351_SENSOR_NAME "hi351"
DEFINE_MSM_MUTEX(hi351_mut);

//static int PREV_EFFECT = -1;
//static int PREV_ISO = -1;
//static int PREV_WB = -1;
//static int PREV_FPS = -1;
static int INIT_DONE = 0;			//                                                                                                                           
static int PREV_BESTSHOT = -1;
static int DELAY_START = 0;

typedef enum {
  HI351_SUNNY,
  HI351_COWELL,
  HI351_MODULE_MAX,
} HI351ModuleType;

typedef enum {
  HI351_60HZ,
  HI351_50HZ,
  HI351_HZ_MAX_NUM,
} HI351AntibandingType;

static int main_cam_id_value = HI351_COWELL;
static int hi351_antibanding = HI351_50HZ;
static int hi351_ab_mod = 0;

static ssize_t hi351_antibanding_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t n)
{
       int val;
       sscanf(buf,"%d",&val);
       printk("hi351: antibanding type [0x%x] \n",val);

       /* 1 : Antibanding 60Hz        * 2 : Antibanding 50Hz */
       switch(val)
       {
			case 1:
				hi351_antibanding = HI351_60HZ;
				break;
			case 2:
				hi351_antibanding = HI351_50HZ;
				break;
			default:
			pr_err("hi351: invalid antibanding type[%d] \n",val);
			hi351_antibanding = HI351_50HZ;
			break;
		}
	return n;
}

static DEVICE_ATTR(antibanding, /*S_IRUGO|S_IWUGO*/ 0664, NULL, hi351_antibanding_store);

static struct attribute* hi351_sysfs_attrs[] = {
       &dev_attr_antibanding.attr,
};

static int hi351_sysfs_add(struct kobject* kobj)
{
	int i, n, ret;

	n = ARRAY_SIZE(hi351_sysfs_attrs);
	for(i = 0; i < n; i++){
		if(hi351_sysfs_attrs[i]){
			ret = sysfs_create_file(kobj, hi351_sysfs_attrs[i]);
				if(ret < 0){
					pr_err("hi351 sysfs is not created\n");
					}
			}
		}
	return 0;
};

static struct msm_sensor_ctrl_t hi351_s_ctrl;

static struct msm_sensor_power_setting hi351_power_setting[] = {
/*                                                                                  */
#if defined(CONFIG_MACH_MSM8X10_W3C_TRF_US) || \
	defined(CONFIG_MACH_MSM8X10_W3C_VZW)
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
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
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 4,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 4,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 2,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 20,
	},

/*                                                                                  */
#else
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VDIG,
		.config_val = GPIO_OUT_HIGH,
		.delay = 2,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 2,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 2,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 20,
	},

/*                                                                                  */
#endif
/*                                                                                  */
};

static struct msm_camera_i2c_conf_array hi351_init_conf[] = {
	{&hi351_recommend_settings_sunny[HI351_60HZ][0],
	ARRAY_SIZE(hi351_recommend_settings_sunny[HI351_60HZ]), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&hi351_recommend_settings_sunny[HI351_50HZ][0],
	ARRAY_SIZE(hi351_recommend_settings_sunny[HI351_50HZ]), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&hi351_recommend_settings_cowell[HI351_60HZ][0],
	ARRAY_SIZE(hi351_recommend_settings_cowell[HI351_60HZ]), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&hi351_recommend_settings_cowell[HI351_50HZ][0],
	ARRAY_SIZE(hi351_recommend_settings_cowell[HI351_50HZ]), 0, MSM_CAMERA_I2C_BYTE_DATA},
};

//                                                                                                                             
// conf0 is with recommend setting right after.
static struct msm_camera_i2c_conf_array hi351_prev_conf_in_case_of_init[] = {
	{&hi351_prev_settings_in_case_of_init[HI351_60HZ][0],
	ARRAY_SIZE(hi351_prev_settings_in_case_of_init[HI351_60HZ]), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&hi351_prev_settings_in_case_of_init[HI351_50HZ][0],
	ARRAY_SIZE(hi351_prev_settings_in_case_of_init[HI351_50HZ]), 0, MSM_CAMERA_I2C_BYTE_DATA},
};

// conf1 is for after snapshot or restart-previewing.
static struct msm_camera_i2c_conf_array hi351_prev_conf[] = {
	{&hi351_prev_settings[HI351_60HZ][0],
	ARRAY_SIZE(hi351_prev_settings[HI351_60HZ]), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&hi351_prev_settings[HI351_50HZ][0],
	ARRAY_SIZE(hi351_prev_settings[HI351_50HZ]), 0, MSM_CAMERA_I2C_BYTE_DATA},
};
//                                                                                                                             

static struct msm_camera_i2c_conf_array hi351_snap_conf[] = {
	{&hi351_snap_settings[HI351_60HZ][0],
	ARRAY_SIZE(hi351_snap_settings[HI351_60HZ]), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&hi351_snap_settings[HI351_50HZ][0],
	ARRAY_SIZE(hi351_snap_settings[HI351_50HZ]), 0, MSM_CAMERA_I2C_BYTE_DATA},
};

//                                                                                                                             
// conf0 is with recommend setting right after.
static struct msm_camera_i2c_conf_array hi351_attached_fps_conf_in_case_of_init[] = {
	{&hi351_attached_fps_settings_in_case_of_init[HI351_60HZ][0],
	ARRAY_SIZE(hi351_attached_fps_settings_in_case_of_init[HI351_60HZ]), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&hi351_attached_fps_settings_in_case_of_init[HI351_50HZ][0],
	ARRAY_SIZE(hi351_attached_fps_settings_in_case_of_init[HI351_50HZ]), 0, MSM_CAMERA_I2C_BYTE_DATA},
};

// conf1 is for after snapshot or restart-previewing.
static struct msm_camera_i2c_conf_array hi351_attached_fps_conf[] = {
	{&hi351_attached_fps_settings[HI351_60HZ][0],
	ARRAY_SIZE(hi351_attached_fps_settings[HI351_60HZ]), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&hi351_attached_fps_settings[HI351_50HZ][0],
	ARRAY_SIZE(hi351_attached_fps_settings[HI351_50HZ]), 0, MSM_CAMERA_I2C_BYTE_DATA},
};
//                                                                                                                             

//                                                                                                                             
// conf0 is with recommend setting right after.
static struct msm_camera_i2c_conf_array hi351_fixed_fps_conf_in_case_of_init[] = {
	{&hi351_fixed_fps_settings_in_case_of_init[HI351_60HZ][0],
	ARRAY_SIZE(hi351_fixed_fps_settings_in_case_of_init[HI351_60HZ]), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&hi351_fixed_fps_settings_in_case_of_init[HI351_50HZ][0],
	ARRAY_SIZE(hi351_fixed_fps_settings_in_case_of_init[HI351_50HZ]), 0, MSM_CAMERA_I2C_BYTE_DATA},
};

// conf1 is for after snapshot or restart-previewing.
static struct msm_camera_i2c_conf_array hi351_fixed_fps_conf[] = {
	{&hi351_fixed_fps_settings[HI351_60HZ][0],
	ARRAY_SIZE(hi351_fixed_fps_settings[HI351_60HZ]), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&hi351_fixed_fps_settings[HI351_50HZ][0],
	ARRAY_SIZE(hi351_fixed_fps_settings[HI351_50HZ]), 0, MSM_CAMERA_I2C_BYTE_DATA},
};
//                                                                                                                             

//                                                                                                                             
// conf0 is with recommend setting right after.
static struct msm_camera_i2c_conf_array hi351_auto_fps_conf_in_case_of_init[] = {
	{&hi351_auto_fps_settings_in_case_of_init[HI351_60HZ][0],
	ARRAY_SIZE(hi351_auto_fps_settings_in_case_of_init[HI351_60HZ]), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&hi351_auto_fps_settings_in_case_of_init[HI351_50HZ][0],
	ARRAY_SIZE(hi351_auto_fps_settings_in_case_of_init[HI351_50HZ]), 0, MSM_CAMERA_I2C_BYTE_DATA},
};

// conf1 is for after snapshot or restart-previewing.
static struct msm_camera_i2c_conf_array hi351_auto_fps_conf[] = {
	{&hi351_auto_fps_settings[HI351_60HZ][0],
	ARRAY_SIZE(hi351_auto_fps_settings[HI351_60HZ]), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&hi351_auto_fps_settings[HI351_50HZ][0],
	ARRAY_SIZE(hi351_auto_fps_settings[HI351_50HZ]), 0, MSM_CAMERA_I2C_BYTE_DATA},
};
//                                                                                                                             

static struct msm_camera_i2c_conf_array hi351_scene_auto_conf[] = {
	{&hi351_reg_scene_auto[HI351_60HZ][0],
	ARRAY_SIZE(hi351_reg_scene_auto[HI351_60HZ]), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&hi351_reg_scene_auto[HI351_50HZ][0],
	ARRAY_SIZE(hi351_reg_scene_auto[HI351_50HZ]), 0, MSM_CAMERA_I2C_BYTE_DATA},
};

static struct msm_camera_i2c_conf_array hi351_scene_night_conf[] = {
	{&hi351_reg_scene_night[HI351_60HZ][0],
	ARRAY_SIZE(hi351_reg_scene_night[HI351_60HZ]), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&hi351_reg_scene_night[HI351_50HZ][0],
	ARRAY_SIZE(hi351_reg_scene_night[HI351_50HZ]), 0, MSM_CAMERA_I2C_BYTE_DATA},
};

static struct msm_camera_i2c_conf_array hi351_scene_landscape_conf[] = {
	{&hi351_reg_scene_landscape[HI351_60HZ][0],
	ARRAY_SIZE(hi351_reg_scene_landscape[HI351_60HZ]), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&hi351_reg_scene_landscape[HI351_50HZ][0],
	ARRAY_SIZE(hi351_reg_scene_landscape[HI351_50HZ]), 0, MSM_CAMERA_I2C_BYTE_DATA},
};
static struct msm_camera_i2c_conf_array hi351_scene_portrait_conf[] = {
	{&hi351_reg_scene_portrait[HI351_60HZ][0],
	ARRAY_SIZE(hi351_reg_scene_portrait[HI351_60HZ]), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&hi351_reg_scene_portrait[HI351_50HZ][0],
	ARRAY_SIZE(hi351_reg_scene_portrait[HI351_50HZ]), 0, MSM_CAMERA_I2C_BYTE_DATA},
};

static struct msm_camera_i2c_conf_array hi351_scene_sport_conf[] = {
	{&hi351_reg_scene_sport[HI351_60HZ][0],
	ARRAY_SIZE(hi351_reg_scene_sport[HI351_60HZ]), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&hi351_reg_scene_sport[HI351_50HZ][0],
	ARRAY_SIZE(hi351_reg_scene_sport[HI351_50HZ]), 0, MSM_CAMERA_I2C_BYTE_DATA},
};

static struct msm_camera_i2c_conf_array hi351_scene_sunset_conf[] = {
	{&hi351_reg_scene_sunset[HI351_60HZ][0],
	ARRAY_SIZE(hi351_reg_scene_sunset[HI351_60HZ]), 0, MSM_CAMERA_I2C_BYTE_DATA},
	{&hi351_reg_scene_sunset[HI351_50HZ][0],
	ARRAY_SIZE(hi351_reg_scene_sunset[HI351_50HZ]), 0, MSM_CAMERA_I2C_BYTE_DATA},
};

static struct v4l2_subdev_info hi351_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order  = 0,
	},
};

static const struct i2c_device_id hi351_i2c_id[] = {
	{HI351_SENSOR_NAME, (kernel_ulong_t)&hi351_s_ctrl},
	{ }
};

static int32_t msm_hi351_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	pr_err("%s, E.\n", __func__);

	rc = msm_sensor_i2c_probe(client, id, &hi351_s_ctrl);
	if(rc == 0){
		if(hi351_sysfs_add(&client->dev.kobj) < 0)
			pr_err("hi351: failed hi351_sysfs_add\n");
	}

	if(gpio_is_valid(71)){
		if(gpio_request(71, "main_cam_id") == 0){
			if(gpio_direction_input(71) == 0){
				   main_cam_id_value = gpio_get_value(71);
				   pr_err("main_cam_id(gpio 71) is %d\n", main_cam_id_value);
			}else pr_err("unable to set direction for gpio 71\n");
		}else pr_err("gpio 71 request failed\n");
	}else pr_err("Invalid gpio 71\n");
	return 0;
}

static struct i2c_driver hi351_i2c_driver = {
	.id_table = hi351_i2c_id,
	.probe  = msm_hi351_i2c_probe,
	.driver = {
		.name = HI351_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client hi351_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static const struct of_device_id hi351_dt_match[] = {
	{.compatible = "qcom,hi351", .data = &hi351_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, hi351_dt_match);

static struct platform_driver hi351_platform_driver = {
	.driver = {
		.name = "qcom,hi351",
		.owner = THIS_MODULE,
		.of_match_table = hi351_dt_match,
	},
};

/*                                                                                            */

static int32_t msm_camera_qup_i2c_txdata(
	struct msm_camera_i2c_client *dev_client, unsigned char *txdata,
	int length)
{
	int32_t rc = 0;
	uint16_t saddr = dev_client->client->addr >> 1;
	struct i2c_msg msg[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		 },
	};
	rc = i2c_transfer(dev_client->client->adapter, msg, 1);
	if (rc < 0)
		CDBG("msm_camera_qup_i2c_txdata faild 0x%x\n", saddr);
	return 0;
}

static int32_t hi351_i2c_write_b_sensor(struct msm_camera_i2c_client *client, unsigned char baddr, unsigned char bdata)
{
	int32_t rc = -EIO;
	unsigned char buf[2];
	memset(buf, 0, sizeof(buf));

	if (DELAY_START == 1) {
		if (baddr == 0xFE) {
			msleep(bdata);
		}
		DELAY_START = 0;
		return 0;
	}
	else {
		if (baddr == 0x03 && bdata == 0xFE) {
			DELAY_START = 1;
			return 0;
		}
		else {

			buf[0] = baddr;
			buf[1] = bdata;

			//pr_err("hi351_i2c_write_b_sensor: 0x%x\n", baddr);
			rc = msm_camera_qup_i2c_txdata(client, buf, 2);

			if (rc < 0)
				pr_err("i2c_write_w failed, addr = 0x%x, val = 0x%x!\n", baddr, bdata);
		}
	}
	return rc;
}

static int32_t hi351_sensor_write_init_settings(struct msm_camera_i2c_client *client,
													struct msm_camera_i2c_reg_conf *conf, uint16_t size)
{
	//BURST MODE
	int32_t rc = 0;
	int i;
	u8 buf[301];
	int bufIndex = 0;

	memset(buf, 0, sizeof(buf));

	//for burst mode
	for (i = 0; i < size; i++) {
		if ( conf->dt == MSM_CAMERA_I2C_BURST_DATA && bufIndex < 301 ) {
			if(bufIndex == 0) {
				buf[bufIndex] = conf->reg_addr;
				bufIndex++;
				buf[bufIndex] = conf->reg_data;
				bufIndex++;
			}
			else {
				buf[bufIndex] = conf->reg_data;
				bufIndex++;
			}
		}
		else {
			if (bufIndex > 0) {
				//pr_err("hi351_sensor_write_init_settings: Burst Mode: bufIndex: %d\n", bufIndex);
				rc = msm_camera_qup_i2c_txdata(client, buf, bufIndex);
				//pr_err("%s: BurstMODE write bufIndex = %d \n",__func__, bufIndex);
				bufIndex = 0;
				memset(buf, 0, sizeof(buf));
				if (rc < 0) {
					pr_err("%s: %d  failed Exit \n",__func__, __LINE__);
					return rc;
				}
			}
			rc = hi351_i2c_write_b_sensor(client,
									conf->reg_addr,
									conf->reg_data);

			if (rc < 0) {
				pr_err("%s: %d  failed Exit \n",__func__, __LINE__);
				return rc;
			}
		}
		conf++;
	}
	return rc;

}


static void hi351_i2c_write_table(struct msm_sensor_ctrl_t *s_ctrl,
		struct msm_camera_i2c_reg_conf *table,
		int num)
{
	int i = 0;
	int rc = 0;
	for (i = 0; i < num; ++i) {
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write(
			s_ctrl->sensor_i2c_client, table->reg_addr,
			table->reg_data,
			MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0) {
			msleep(100);
			pr_err("%s: %d One more try \n",__func__, __LINE__);			
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write(
				s_ctrl->sensor_i2c_client, table->reg_addr,
				table->reg_data,
				MSM_CAMERA_I2C_BYTE_DATA);
		}
		table++;
	}
}

/*                                                                                            */

static int32_t hi351_platform_probe(struct platform_device *pdev)
{
	int32_t rc;
	const struct of_device_id *match;
	printk("%s, E.\n", __func__);
	match = of_match_device(hi351_dt_match, &pdev->dev);
/*                                                    */
		if(!match)
		{
			  pr_err(" %s failed\n",__func__);
			  return -ENODEV;
		 }
/*                                                    */
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init hi351_init_module(void)
{
	int32_t rc;
	
	printk("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&hi351_platform_driver, hi351_platform_probe);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&hi351_i2c_driver);
}

static void __exit hi351_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (hi351_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&hi351_s_ctrl);
		platform_driver_unregister(&hi351_platform_driver);
	} else
		i2c_del_driver(&hi351_i2c_driver);
	return;
}

/*                                                                                  */
static int32_t hi351_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint16_t chipid = 0;
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
		s_ctrl->sensor_i2c_client,
		s_ctrl->sensordata->slave_info->sensor_id_reg_addr,
		&chipid, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: %s: hi351 read id failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
		return rc;
	}

	CDBG("%s: read id: %x expected id %x:\n", __func__, chipid,
		s_ctrl->sensordata->slave_info->sensor_id);
	if (chipid != s_ctrl->sensordata->slave_info->sensor_id) {
		pr_err("msm_sensor_match_id chip id doesnot match\n");
		return -ENODEV;
	}
	return rc;
}
/*                                                                                  */
#if 0
static void hi351_set_stauration(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	pr_debug("%s %d", __func__, value);
	hi351_i2c_write_table(s_ctrl, &hi351_reg_saturation[value][0],
		ARRAY_SIZE(hi351_reg_saturation[value]));
}

static void hi351_set_contrast(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	pr_debug("%s %d", __func__, value);
	hi351_i2c_write_table(s_ctrl, &hi351_reg_contrast[value][0],
		ARRAY_SIZE(hi351_reg_contrast[value]));
}

static void hi351_set_sharpness(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	int val = value / 6;
	pr_debug("%s %d", __func__, value);
	hi351_i2c_write_table(s_ctrl, &hi351_reg_sharpness[val][0],
		ARRAY_SIZE(hi351_reg_sharpness[val]));
}
#endif

static void hi351_set_iso(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	pr_debug("%s %d\n", __func__, value);
	hi351_i2c_write_table(s_ctrl, &hi351_reg_iso[value][0],
		ARRAY_SIZE(hi351_reg_iso[value]));
}

static void hi351_set_exposure_compensation(struct msm_sensor_ctrl_t *s_ctrl,
	int value)
{
	int val = value + ((12 - value)/2); //                                                                                      
	pr_debug("%s %d\n", __func__, val);
	hi351_i2c_write_table(s_ctrl, &hi351_reg_exposure_compensation[val][0],
		ARRAY_SIZE(hi351_reg_exposure_compensation[val]));
}

static void hi351_set_effect(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	pr_debug("%s %d\n", __func__, value);
	switch (value) {
	case MSM_CAMERA_EFFECT_MODE_OFF: {
		hi351_i2c_write_table(s_ctrl, &hi351_reg_effect_normal[0],
			ARRAY_SIZE(hi351_reg_effect_normal));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_MONO: {
		hi351_i2c_write_table(s_ctrl, &hi351_reg_effect_black_white[0],
			ARRAY_SIZE(hi351_reg_effect_black_white));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_NEGATIVE: {
		hi351_i2c_write_table(s_ctrl, &hi351_reg_effect_negative[0],
			ARRAY_SIZE(hi351_reg_effect_negative));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_SEPIA: {
		hi351_i2c_write_table(s_ctrl, &hi351_reg_effect_old_movie[0],
			ARRAY_SIZE(hi351_reg_effect_old_movie));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_SOLARIZE: {
		pr_debug("%s: We do not support Saturation feature Value now\n", __func__);
#if 0
		hi351_i2c_write_table(s_ctrl, &hi351_reg_effect_solarize[0],
			ARRAY_SIZE(hi351_reg_effect_solarize));
#endif
		break;
	}
	default:
		hi351_i2c_write_table(s_ctrl, &hi351_reg_effect_normal[0],
			ARRAY_SIZE(hi351_reg_effect_normal));
	}
}

#if 0
static void hi351_set_antibanding(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	pr_debug("%s %d", __func__, value);
	hi351_i2c_write_table(s_ctrl, &hi351_reg_antibanding[value][0],
		ARRAY_SIZE(hi351_reg_antibanding[value]));
}
#endif

static void hi351_set_scene_mode(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	if(PREV_BESTSHOT == value)
		pr_err("%s duplicated %d\n", __func__, value);
	else{
		PREV_BESTSHOT = value;

	pr_debug("%s %d\n", __func__, value);
	switch (value) {
	case MSM_CAMERA_SCENE_MODE_OFF: {
		hi351_sensor_write_init_settings(s_ctrl->sensor_i2c_client,
		(struct msm_camera_i2c_reg_conf *) hi351_scene_auto_conf[hi351_antibanding].conf, hi351_scene_auto_conf[hi351_antibanding].size);
		break;
	}
	case MSM_CAMERA_SCENE_MODE_NIGHT: {
		hi351_sensor_write_init_settings(s_ctrl->sensor_i2c_client,
		(struct msm_camera_i2c_reg_conf *) hi351_scene_night_conf[hi351_antibanding].conf, hi351_scene_night_conf[hi351_antibanding].size);
		hi351_i2c_write_table(s_ctrl, &hi351_scene_active_settings[0],
			ARRAY_SIZE(hi351_scene_active_settings));
		break;
	}
	case MSM_CAMERA_SCENE_MODE_LANDSCAPE: {
		hi351_sensor_write_init_settings(s_ctrl->sensor_i2c_client,
		(struct msm_camera_i2c_reg_conf *) hi351_scene_landscape_conf[hi351_antibanding].conf, hi351_scene_landscape_conf[hi351_antibanding].size);
		hi351_i2c_write_table(s_ctrl, &hi351_scene_active_settings[0],
			ARRAY_SIZE(hi351_scene_active_settings));
		break;
	}
	case MSM_CAMERA_SCENE_MODE_PORTRAIT: {
		hi351_sensor_write_init_settings(s_ctrl->sensor_i2c_client,
		(struct msm_camera_i2c_reg_conf *) hi351_scene_portrait_conf[hi351_antibanding].conf, hi351_scene_portrait_conf[hi351_antibanding].size);
		hi351_i2c_write_table(s_ctrl, &hi351_scene_active_settings[0],
			ARRAY_SIZE(hi351_scene_active_settings));
		break;
	}
	case MSM_CAMERA_SCENE_MODE_SPORTS: {
		hi351_sensor_write_init_settings(s_ctrl->sensor_i2c_client,
		(struct msm_camera_i2c_reg_conf *) hi351_scene_sport_conf[hi351_antibanding].conf, hi351_scene_sport_conf[hi351_antibanding].size);
		hi351_i2c_write_table(s_ctrl, &hi351_scene_active_settings[0],
			ARRAY_SIZE(hi351_scene_active_settings));
		break;
	}
	case MSM_CAMERA_SCENE_MODE_SUNSET: {
		hi351_sensor_write_init_settings(s_ctrl->sensor_i2c_client,
		(struct msm_camera_i2c_reg_conf *) hi351_scene_sunset_conf[hi351_antibanding].conf, hi351_scene_sunset_conf[hi351_antibanding].size);
		hi351_i2c_write_table(s_ctrl, &hi351_scene_active_settings[0],
			ARRAY_SIZE(hi351_scene_active_settings));
		break;
	}
	default:
		hi351_sensor_write_init_settings(s_ctrl->sensor_i2c_client,
		(struct msm_camera_i2c_reg_conf *) hi351_scene_auto_conf[hi351_antibanding].conf, hi351_scene_auto_conf[hi351_antibanding].size);
		hi351_i2c_write_table(s_ctrl, &hi351_scene_active_settings[0],
			ARRAY_SIZE(hi351_scene_active_settings));
	}
	}
}

static void hi351_set_white_balance_mode(struct msm_sensor_ctrl_t *s_ctrl,
	int value)
{
	pr_debug("%s %d\n", __func__, value);
	switch (value) {
	case MSM_CAMERA_WB_MODE_AUTO: {
		hi351_i2c_write_table(s_ctrl, &hi351_reg_wb_auto[0],
			ARRAY_SIZE(hi351_reg_wb_auto));
		break;
	}
	case MSM_CAMERA_WB_MODE_INCANDESCENT: {
		hi351_i2c_write_table(s_ctrl, &hi351_reg_wb_home[0],
			ARRAY_SIZE(hi351_reg_wb_home));
		break;
	}
	case MSM_CAMERA_WB_MODE_DAYLIGHT: {
		hi351_i2c_write_table(s_ctrl, &hi351_reg_wb_sunny[0],
			ARRAY_SIZE(hi351_reg_wb_sunny));
		break;
	}
	case MSM_CAMERA_WB_MODE_FLUORESCENT: {
		hi351_i2c_write_table(s_ctrl, &hi351_reg_wb_office[0],
			ARRAY_SIZE(hi351_reg_wb_office));
		break;
	}
	case MSM_CAMERA_WB_MODE_CLOUDY_DAYLIGHT: {
		hi351_i2c_write_table(s_ctrl, &hi351_reg_wb_cloudy[0],
			ARRAY_SIZE(hi351_reg_wb_cloudy));
		break;
	}
	default:
		hi351_i2c_write_table(s_ctrl, &hi351_reg_wb_auto[0],
		ARRAY_SIZE(hi351_reg_wb_auto));
	}
}


//                                                                                                                                     
static void hi351_set_framerate_for_soc(struct msm_sensor_ctrl_t *s_ctrl, struct msm_fps_range_setting *framerate)
{
	int32_t value = 0;
	 //                                                                                                              
	if((framerate->min_fps == 1097859072) && (framerate->max_fps == 1097859072))
		value = 0;
	 //                                                                                                                                 
	else if((framerate->min_fps == 1103101952) && (framerate->max_fps == 1106247680))
		value = 1;
	else value = 2;

	pr_debug("%s %d\n", __func__, value);
	switch (value) {
		case 0: { //attached MMS VIDEO 177x144, 384x288
			pr_debug("%s %d\n", __func__, value);

			//                                                                                                                             
			// conf0 is with recommend setting right after. When INIT_DONE is 1, it means this is followed by right after Recommend Setting register set.
			if(INIT_DONE == 1){
				hi351_sensor_write_init_settings(s_ctrl->sensor_i2c_client,
				(struct msm_camera_i2c_reg_conf *) hi351_attached_fps_conf_in_case_of_init[hi351_antibanding].conf, hi351_attached_fps_conf_in_case_of_init[hi351_antibanding].size);
				}
			// conf1 is normal situation.
			else if(INIT_DONE == 0){
				hi351_sensor_write_init_settings(s_ctrl->sensor_i2c_client,
				(struct msm_camera_i2c_reg_conf *) hi351_attached_fps_conf[hi351_antibanding].conf, hi351_attached_fps_conf[hi351_antibanding].size);
				}
			//                                                                                                                             
			}
			break;

		case 1: { //fixed for VIDEO 640x480 and more over.
			pr_debug("%s %d\n", __func__, value);

			//                                                                                                                             
			// conf0 is with recommend setting right after. When INIT_DONE is 1, it means this is followed by right after Recommend Setting register set.
			if(INIT_DONE == 1){
				hi351_sensor_write_init_settings(s_ctrl->sensor_i2c_client,
				(struct msm_camera_i2c_reg_conf *) hi351_fixed_fps_conf_in_case_of_init[hi351_antibanding].conf, hi351_fixed_fps_conf_in_case_of_init[hi351_antibanding].size);
				}
			// conf1 is normal situation.
			else if(INIT_DONE == 0){
				hi351_sensor_write_init_settings(s_ctrl->sensor_i2c_client,
				(struct msm_camera_i2c_reg_conf *) hi351_fixed_fps_conf[hi351_antibanding].conf, hi351_fixed_fps_conf[hi351_antibanding].size);
				}
			//                                                                                                                             
			}
			break;

		default:{  //default
			pr_debug("%s %d\n", __func__, value);

			//                                                                                                                             
			// conf0 is with recommend setting right after. When INIT_DONE is 1, it means this is followed by right after Recommend Setting register set.
			if(INIT_DONE == 1){
				hi351_sensor_write_init_settings(s_ctrl->sensor_i2c_client,
				(struct msm_camera_i2c_reg_conf *) hi351_auto_fps_conf_in_case_of_init[hi351_antibanding].conf, hi351_auto_fps_conf_in_case_of_init[hi351_antibanding].size);
				}
			// conf1 is normal situation.
			else if(INIT_DONE == 0){
				hi351_sensor_write_init_settings(s_ctrl->sensor_i2c_client,
				(struct msm_camera_i2c_reg_conf *) hi351_auto_fps_conf[hi351_antibanding].conf, hi351_auto_fps_conf[hi351_antibanding].size);
				}
			//                                                                                                                             
			}
			break;
	}
}
//                                                                                                                                     

int32_t hi351_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	long rc = 0;
	int32_t i = 0;
	mutex_lock(s_ctrl->msm_sensor_mutex);
	CDBG("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
		s_ctrl->sensordata->sensor_name, cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_GET_SENSOR_INFO:
		pr_err("%s, CFG_GET_SENSOR_INFO!!\n", __func__);
		memcpy(cdata->cfg.sensor_info.sensor_name,
			s_ctrl->sensordata->sensor_name,
			sizeof(cdata->cfg.sensor_info.sensor_name));
		cdata->cfg.sensor_info.session_id =
			s_ctrl->sensordata->sensor_info->session_id;
		for (i = 0; i < SUB_MODULE_MAX; i++)
			cdata->cfg.sensor_info.subdev_id[i] =
				s_ctrl->sensordata->sensor_info->subdev_id[i];
		CDBG("%s:%d sensor name %s\n", __func__, __LINE__,
			cdata->cfg.sensor_info.sensor_name);
		CDBG("%s:%d session id %d\n", __func__, __LINE__,
			cdata->cfg.sensor_info.session_id);
		for (i = 0; i < SUB_MODULE_MAX; i++)
			CDBG("%s:%d subdev_id[%d] %d\n", __func__, __LINE__, i,
				cdata->cfg.sensor_info.subdev_id[i]);

		break;
	case CFG_SET_INIT_SETTING: {
		/* Write Recommend settings */
		int32_t rc = 0;
		int32_t retry;
		pr_err("%s, CFG_SET_INIT_SETTING!!\n", __func__);
/*                                                                                                    */
		if((main_cam_id_value == HI351_SUNNY)&&(hi351_antibanding == HI351_60HZ)) hi351_ab_mod = 0;
			else if((main_cam_id_value == HI351_SUNNY)&&(hi351_antibanding == HI351_50HZ)) hi351_ab_mod = 1;
				else if((main_cam_id_value == HI351_COWELL)&&(hi351_antibanding == HI351_60HZ)) hi351_ab_mod = 2;
					else if((main_cam_id_value == HI351_COWELL)&&(hi351_antibanding == HI351_50HZ)) hi351_ab_mod = 3;
		pr_err("%s, hi351_ab_mod vaule : 01Sun,23Cow %d\n", __func__, hi351_ab_mod);
/*                                                                                                    */

		for (retry = 0; retry < 3; ++retry) {
				rc = hi351_sensor_write_init_settings(s_ctrl->sensor_i2c_client,
							(struct msm_camera_i2c_reg_conf *) hi351_init_conf[hi351_ab_mod].conf, hi351_init_conf[hi351_ab_mod].size);
				if (rc < 0)
					pr_err(KERN_ERR "[ERROR]%s:Sensor Init Setting Fail\n", __func__);
				else break;
		}
//                                                                                                                             
// We do nothing here except setting the Static variable INIT_DONE, we set the variable into 1.
		if( INIT_DONE == 0 ) INIT_DONE = 1;
		pr_err("%s, CFG_SET_INIT_SETTING!! done\n", __func__);
//                                                                                                                             
		break;
		}
	case CFG_SET_RESOLUTION: {
		/*                                                                                                                          */
		int val = 0;
		pr_err("%s, CFG_SET_RESOLUTION!!\n", __func__);
		if (copy_from_user(&val,
			(void *)cdata->cfg.setting, sizeof(int))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		if (val == 0){
			hi351_sensor_write_init_settings(s_ctrl->sensor_i2c_client,
							(struct msm_camera_i2c_reg_conf *) hi351_snap_conf[hi351_antibanding].conf, hi351_snap_conf[hi351_antibanding].size);
			pr_err("%s, snapsettings!!\n", __func__);
		}
		else if (val == 1){
//                                                                                                                             
// conf0 is with recommend setting right after. When INIT_DONE is 1, it means this is followed by right after Recommend Setting register set.
			if(INIT_DONE == 1){
			hi351_sensor_write_init_settings(s_ctrl->sensor_i2c_client,
								(struct msm_camera_i2c_reg_conf *) hi351_prev_conf_in_case_of_init[hi351_antibanding].conf, hi351_prev_conf_in_case_of_init[hi351_antibanding].size);
			pr_err("%s, prevsettings followed by INIT!!\n", __func__);
				}
			// conf1 is normal situation.
			else if(INIT_DONE == 0){
			hi351_sensor_write_init_settings(s_ctrl->sensor_i2c_client,
								(struct msm_camera_i2c_reg_conf *) hi351_prev_conf[hi351_antibanding].conf, hi351_prev_conf[hi351_antibanding].size);
			pr_err("%s, prevsettings!!\n", __func__);
				}
//                                                                                                                              
		}
		break;
		/*                                                                                                                          */
		}
	case CFG_SET_STOP_STREAM:
		pr_err("%s, CFG_SET_STOP_STREAM!!\n", __func__);
		hi351_i2c_write_table(s_ctrl,
			&hi351_stop_settings[0],
			ARRAY_SIZE(hi351_stop_settings));
		break;

	case CFG_SET_START_STREAM:
		pr_err("%s, CFG_SET_START_STREAM!!\n", __func__);
//                                                                                                                             
// settings0 is with recommend setting right after. When INIT_DONE is 1, it means this is followed by right after Recommend Setting register set.
		if(INIT_DONE == 1){
			hi351_i2c_write_table(s_ctrl, &hi351_start_settings_in_case_of_init[0], ARRAY_SIZE(hi351_start_settings_in_case_of_init));
			INIT_DONE = 0; // this is very important spot.
			}
		// settings1 is with recommend setting right after. When INIT_DONE is 1, it means this is followed by right after Recommend Setting register set.
		else if(INIT_DONE == 0){
			hi351_i2c_write_table(s_ctrl, &hi351_start_settings[0], ARRAY_SIZE(hi351_start_settings));
			}
//                                                                                                                             
		break;
	case CFG_GET_SENSOR_INIT_PARAMS:
		pr_err("%s, CFG_GET_SENSOR_INIT_PARAMS!!\n", __func__);
		cdata->cfg.sensor_init_params =
			*s_ctrl->sensordata->sensor_init_params;
		CDBG("%s:%d init params mode %d pos %d mount %d\n", __func__,
			__LINE__,
			cdata->cfg.sensor_init_params.modes_supported,
			cdata->cfg.sensor_init_params.position,
			cdata->cfg.sensor_init_params.sensor_mount_angle);
		break;
	case CFG_SET_SLAVE_INFO: {
		struct msm_camera_sensor_slave_info sensor_slave_info;
		struct msm_sensor_power_setting_array *power_setting_array;
		int slave_index = 0;
		pr_err("%s, CFG_SET_SLAVE_INFO!!\n", __func__);
		if (copy_from_user(&sensor_slave_info,
		    (void *)cdata->cfg.setting,
		    sizeof(struct msm_camera_sensor_slave_info))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		/* Update sensor slave address */
		if (sensor_slave_info.slave_addr) {
			s_ctrl->sensor_i2c_client->cci_client->sid =
				sensor_slave_info.slave_addr >> 1;
		}

		/* Update sensor address type */
		s_ctrl->sensor_i2c_client->addr_type =
			sensor_slave_info.addr_type;

		/* Update power up / down sequence */
		s_ctrl->power_setting_array =
			sensor_slave_info.power_setting_array;
		power_setting_array = &s_ctrl->power_setting_array;
		power_setting_array->power_setting = kzalloc(
			power_setting_array->size *
			sizeof(struct msm_sensor_power_setting), GFP_KERNEL);
		if (!power_setting_array->power_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(power_setting_array->power_setting,
		    (void *)sensor_slave_info.power_setting_array.power_setting,
		    power_setting_array->size *
		    sizeof(struct msm_sensor_power_setting))) {
			kfree(power_setting_array->power_setting);
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		s_ctrl->free_power_setting = true;
		CDBG("%s sensor id %x\n", __func__,
			sensor_slave_info.slave_addr);
		CDBG("%s sensor addr type %d\n", __func__,
			sensor_slave_info.addr_type);
		CDBG("%s sensor reg %x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id_reg_addr);
		CDBG("%s sensor id %x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id);
		for (slave_index = 0; slave_index <
			power_setting_array->size; slave_index++) {
			CDBG("%s i %d power setting %d %d %ld %d\n", __func__,
				slave_index,
				power_setting_array->power_setting[slave_index].
				seq_type,
				power_setting_array->power_setting[slave_index].
				seq_val,
				power_setting_array->power_setting[slave_index].
				config_val,
				power_setting_array->power_setting[slave_index].
				delay);
		}
		kfree(power_setting_array->power_setting);
		break;
	}
	case CFG_WRITE_I2C_ARRAY: {
		struct msm_camera_i2c_reg_setting conf_array;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		pr_err("%s, CFG_WRITE_I2C_ARRAY!!\n", __func__);
		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
			s_ctrl->sensor_i2c_client, &conf_array);
		kfree(reg_setting);
		break;
	}
	case CFG_WRITE_I2C_SEQ_ARRAY: {
		struct msm_camera_i2c_seq_reg_setting conf_array;
		struct msm_camera_i2c_seq_reg_array *reg_setting = NULL;
		pr_err("%s, CFG_WRITE_I2C_SEQ_ARRAY!!\n", __func__);
		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_seq_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_seq_reg_array)),
			GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_seq_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_seq_table(s_ctrl->sensor_i2c_client,
			&conf_array);
		kfree(reg_setting);
		break;
	}

/*                                                              */
		case CFG_PAGE_MODE_READ_I2C_ARRAY:{
			int16_t size=0;
			uint16_t read_data_size = 0;
			uint16_t *read_data;
			uint16_t *read_data_head;
			struct msm_camera_i2c_reg_setting conf_array;
			struct msm_camera_i2c_reg_array *reg_setting = NULL;

	//		pr_err("[WX] %s CFG_PAGE_MODE_READ_I2C_ARRAY\n", __func__);

			if (copy_from_user(&conf_array,
				(void *)cdata->cfg.setting,
				sizeof(struct msm_camera_i2c_reg_setting))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}

			size = conf_array.size; 	//size for write(page_mode) and read
			read_data_size = size - 1;	//size for read

			pr_err("[WX] %s: size : %d rsize : %d\n", __func__, size, read_data_size);

			if (!size || !read_data_size) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}

			reg_setting = kzalloc(size *(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
			if (!reg_setting) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -ENOMEM;
				break;
			}
			if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
				size * sizeof(struct msm_camera_i2c_reg_array))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				kfree(reg_setting);
				rc = -EFAULT;
				break;
			}

			read_data = kzalloc(read_data_size * (sizeof(uint16_t)), GFP_KERNEL);
			if (!read_data) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -ENOMEM;
				break;
			}

			//check if this code is needed;;;
			if (copy_from_user(read_data, (void *)conf_array.value,
				read_data_size * sizeof(uint16_t))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				kfree(reg_setting);
				rc = -EFAULT;
				break;
			}
			//

			conf_array.reg_setting = reg_setting;
			read_data_head = read_data;

			for(i = 0; i < size; i++){
				if(i == 0){
					rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, conf_array.reg_setting->reg_addr, conf_array.reg_setting->reg_data, conf_array.data_type);
				}
				else{
					rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, conf_array.reg_setting->reg_addr, read_data, conf_array.data_type);
					pr_err("[WX] %s read_data : %d\n", __func__, *read_data);
					read_data++;
				}
				conf_array.reg_setting++;
			}

			read_data = read_data_head;

			if (copy_to_user((void *)conf_array.value, read_data, read_data_size * sizeof(uint16_t))) {
				pr_err("%s:%d copy failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}

			kfree(reg_setting);
			kfree(read_data);

			reg_setting = NULL;
			read_data = NULL;
			read_data_head = NULL;

			pr_err("[WX] %s done\n", __func__);

			break;
		}
/*                                                              */

/*                                                                                          */
	case CFG_POWER_UP:{

		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_DOWN) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}

		if (s_ctrl->func_tbl->sensor_power_up){
			rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);

		if (rc < 0) {
				pr_err("%s POWER_UP failed\n", __func__);
				break;
			}
			s_ctrl->sensor_state = MSM_SENSOR_POWER_UP;
			pr_err("%s:%d sensor state %d\n", __func__, __LINE__,
				s_ctrl->sensor_state);
		}else{
			rc = -EFAULT;
		}
		break;
	}
	case CFG_POWER_DOWN:{

		kfree(s_ctrl->stop_setting.reg_setting);
		s_ctrl->stop_setting.reg_setting = NULL;
		if (s_ctrl->sensor_state != MSM_SENSOR_POWER_UP) {
			pr_err("%s:%d failed: invalid state %d\n", __func__,
				__LINE__, s_ctrl->sensor_state);
			rc = -EFAULT;
			break;
		}

		if (s_ctrl->func_tbl->sensor_power_down){
			rc = s_ctrl->func_tbl->sensor_power_down(s_ctrl);

			if (rc < 0) {
				pr_err("%s POWER_DOWN failed\n", __func__);
				break;
			}
			s_ctrl->sensor_state = MSM_SENSOR_POWER_DOWN;
			pr_err("%s:%d sensor state %d\n", __func__, __LINE__,
				s_ctrl->sensor_state);

		}else{
			rc = -EFAULT;
		}
		break;
	}
/*                                                                                          */
	case CFG_SET_STOP_STREAM_SETTING: {
		struct msm_camera_i2c_reg_setting *stop_setting =
			&s_ctrl->stop_setting;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		pr_err("%s, CFG_SET_STOP_STREAM_SETTING!!\n", __func__);
		if (copy_from_user(stop_setting, (void *)cdata->cfg.setting,
		    sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = stop_setting->reg_setting;
		stop_setting->reg_setting = kzalloc(stop_setting->size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!stop_setting->reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(stop_setting->reg_setting,
		    (void *)reg_setting, stop_setting->size *
		    sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(stop_setting->reg_setting);
			stop_setting->reg_setting = NULL;
			stop_setting->size = 0;
			rc = -EFAULT;
			break;
		}
		break;
	}
	case CFG_SET_SATURATION: {
		pr_debug("%s: We do not support Saturation feature Value now\n", __func__);
#if 0
		int32_t sat_lev;
		if (copy_from_user(&sat_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: Saturation Value is %d", __func__, sat_lev);
		hi351_set_stauration(s_ctrl, sat_lev);
#endif
		break;
	}
	case CFG_SET_CONTRAST: {
		pr_debug("%s: We do not support Contrast feature Value now\n", __func__);
#if 0
		int32_t con_lev;
		if (copy_from_user(&con_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: Contrast Value is %d", __func__, con_lev);
		hi351_set_contrast(s_ctrl, con_lev);
#endif
		break;
	}
	case CFG_SET_SHARPNESS: {
		pr_debug("%s: We do not support Sharpness feature Value now\n", __func__);
#if 0
		int32_t shp_lev;
		if (copy_from_user(&shp_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: Sharpness Value is %d", __func__, shp_lev);
		hi351_set_sharpness(s_ctrl, shp_lev);
#endif
		break;
	}
	case CFG_SET_ISO: {
		int32_t iso_lev;
		if (copy_from_user(&iso_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: ISO Value is %d\n", __func__, iso_lev);
		hi351_set_iso(s_ctrl, iso_lev);
		break;
	}
	case CFG_SET_EXPOSURE_COMPENSATION: {
		int32_t ec_lev;
		if (copy_from_user(&ec_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: Exposure compensation Value is %d\n",
			__func__, ec_lev);
		hi351_set_exposure_compensation(s_ctrl, ec_lev);
		break;
	}
	case CFG_SET_EFFECT: {
		int32_t effect_mode;
		if (copy_from_user(&effect_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: Effect mode is %d\n", __func__, effect_mode);
		hi351_set_effect(s_ctrl, effect_mode);
		break;
	}
	case CFG_SET_ANTIBANDING: {
		pr_debug("%s: We do not support Antibanding feature Value now\n", __func__);
#if 0
		int32_t antibanding_mode;
		if (copy_from_user(&antibanding_mode,
			(void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: anti-banding mode is %d", __func__,
			antibanding_mode);
		hi351_set_antibanding(s_ctrl, antibanding_mode);
#endif
		break;
	}
	case CFG_SET_BESTSHOT_MODE: {
		int32_t bs_mode;
		if (copy_from_user(&bs_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: best shot mode is %d\n", __func__, bs_mode);
		hi351_set_scene_mode(s_ctrl, bs_mode);
		break;
	}
	case CFG_SET_WHITE_BALANCE: {
		int32_t wb_mode;
		if (copy_from_user(&wb_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: white balance is %d\n", __func__, wb_mode);
		hi351_set_white_balance_mode(s_ctrl, wb_mode);
		break;
	}
//                                                                                                          
	case CFG_SET_AEC_LOCK:
	case CFG_SET_AWB_LOCK:
	case CFG_SET_AEC_ROI:
		pr_debug("%s: We do not support features value related to LOCK now\n", __func__);
		break;
//                                                                                                          
//                                                                                                                                     
	case CFG_SET_FRAMERATE_FOR_SOC: {
		struct msm_fps_range_setting *framerate;
		if (copy_from_user(&framerate, (void *)cdata->cfg.setting, sizeof(struct msm_fps_range_setting))) {
			rc = -EFAULT;
			break;
		}
		hi351_set_framerate_for_soc(s_ctrl, framerate);
		break;
	}
//                                                                                                                                     
	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}

static struct msm_sensor_fn_t hi351_sensor_func_tbl = {
	.sensor_config = hi351_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = hi351_sensor_match_id,
};

static struct msm_sensor_ctrl_t hi351_s_ctrl = {
	.sensor_i2c_client = &hi351_sensor_i2c_client,
	.power_setting_array.power_setting = hi351_power_setting,
	.power_setting_array.size = ARRAY_SIZE(hi351_power_setting),
	.msm_sensor_mutex = &hi351_mut,
	.sensor_v4l2_subdev_info = hi351_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(hi351_subdev_info),
	.func_tbl = &hi351_sensor_func_tbl,
};

module_init(hi351_init_module);
module_exit(hi351_exit_module);
MODULE_DESCRIPTION("Hynix 3MP YUV sensor driver");
MODULE_LICENSE("GPL v2");
