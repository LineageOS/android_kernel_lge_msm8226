/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
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

/* LGE_CHANGE_S. To fast tune register. sujeong.kwon@lge.com 2014.03.22*/
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <mach/gpio.h>
#include <linux/kthread.h>
#include <linux/syscalls.h>
#include <linux/fs.h>
/* LGE_CHANGE_E. To fast tune register. sujeong.kwon@lge.com 2014.03.22*/
#include "../../../../../base/base.h" /* LGE_CHANGE . To read antibanding value. sujeong.kwon@lge.com 2014.04.23*/
#include "hi258_reg.h"

//#define CONFIG_MSMB_CAMERA_DEBUG
//#define CONFIG_FAST_TUNE_REGISTER  /* LGE_CHANGE . To fast tune register. sujeong.kwon@lge.com 2014.03.22*/

#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

#define HI258_SENSOR_NAME "hi258"
DEFINE_MSM_MUTEX(hi258_mut);
static struct msm_sensor_ctrl_t hi258_s_ctrl;

#if defined(CONFIG_FAST_TUNE_REGISTER)
static int TUNING_REGISTER = 0 ;		/* LGE_CHANGE . To fast tune register. sujeong.kwon@lge.com 2014.03.22*/
#define BUF_SIZE	(256 * 1024)
#endif

typedef enum {
  HI258_60HZ,
  HI258_50HZ,
  HI258_HZ_MAX_NUM,
} HI258AntibandingType;

//LGE_CHANGE_S sujeong.kwon@lge.com 2014-03-08. To know current resolution
typedef enum {
  HI258_SENSOR_RES_FULL,  // for capture
  HI258_SENSOR_RES_PREVIEW,   //for preview
  HI258_SENSOR_RES_HD,   // 16:9 size, HD, for HD recording.
  HI258_SENSOR_RES_SVGA,
  HI258_SENSOR_RES_MAX_NUM,
} HI258ResolutionType;
//LGE_CHANGE_E sujeong.kwon@lge.com 2014-03-08. To know current resolution

static int hi258_antibanding = HI258_60HZ;
static int hi258_cur_res = HI258_SENSOR_RES_PREVIEW;  //sujeong.kwon@lge.com 2014-03-08. To know current resolution
static int hi258_prev_res = HI258_SENSOR_RES_PREVIEW;  //sujeong.kwon@lge.com 2014-03-08. To know previous resolution. After 720P returns to QTR, you must set  hi258_recover_from720P_settings.

static ssize_t hi258_antibanding_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t n)
{
       int val;
       sscanf(buf,"%d",&val);
       printk("hi258: antibanding type [0x%x] \n",val);

       /* 1 : Antibanding 60Hz        * 2 : Antibanding 50Hz */
       switch(val)
       {
			case 1:
				hi258_antibanding = HI258_60HZ;
				break;
			case 2:
				hi258_antibanding = HI258_50HZ;
				break;
			default:
				pr_err("hi258: invalid antibanding type[%d] \n",val);
				hi258_antibanding = HI258_60HZ;
			break;
		}
	return n;
}

static DEVICE_ATTR(antibanding, /*S_IRUGO|S_IWUGO*/ 0664, NULL, hi258_antibanding_store);

static struct attribute* hi258_sysfs_attrs[] = {
       &dev_attr_antibanding.attr,
};

static struct device_attribute* hi258_sysfs_symlink[] = {
       &dev_attr_antibanding,
		NULL
};

static int hi258_sysfs_add(struct kobject* kobj)
{
	int i, n, ret;

	n = ARRAY_SIZE(hi258_sysfs_attrs);
	for(i = 0; i < n; i++){
		if(hi258_sysfs_attrs[i]){
			ret = sysfs_create_file(kobj, hi258_sysfs_attrs[i]);
				if(ret < 0){
					pr_err("hi258 sysfs is not created\n");
					}
			}
		}
	return 0;
};

static int hi258_sysfs_add_symlink(struct device *dev)
{
	 int i = 0;
	 int rc = 0;
	 int n =0;
	 struct bus_type *bus = dev->bus;

	n = ARRAY_SIZE(hi258_sysfs_symlink);
	for(i = 0; i < n; i++){
		if(hi258_sysfs_symlink[i]){
			rc = device_create_file(dev, hi258_sysfs_symlink[i]);
				if(rc < 0){
					pr_err("hi258_sysfs_add_symlink is not created\n");
					goto out_unreg;
				}
			}
		}

	if(bus){
//  PATH of bus->p->devices_kset = /sys/bus/platform/devices/
		rc = sysfs_create_link(&bus->p->devices_kset->kobj, &dev->kobj, "cam_sensor");
		if(rc)
			goto out_unlink;
	}

	CDBG("hi258_sysfs_add_symlink is created\n");
	return 0;

out_unreg:
	pr_err("fail to creat device file for antibanding\n");
	for (; i >= 0; i--)
		device_remove_file(dev, hi258_sysfs_symlink[i]);

	return rc;

out_unlink:
	pr_err("fail to creat sys link for antibanding\n");
	sysfs_remove_link(&bus->p->devices_kset->kobj, "cam_sensor");
	return rc;

};
static struct msm_sensor_power_setting hi258_power_setting[] = {
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
		.delay = 1,
	},
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
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VDIG,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 5,  //20,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 30, //15,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 2,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct v4l2_subdev_info hi258_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt	= 1,
		.order	= 0,
	},
};

static const struct i2c_device_id hi258_i2c_id[] = {
	{HI258_SENSOR_NAME, (kernel_ulong_t)&hi258_s_ctrl},
	{ }
};

static int32_t msm_hi258_i2c_probe(struct i2c_client *client,
	   const struct i2c_device_id *id)
{
#if 0 //QCT original
	return msm_sensor_i2c_probe(client, id, &hi258_s_ctrl);
#else
	int rc = 0;

	rc = msm_sensor_i2c_probe(client, id, &hi258_s_ctrl);
	if(rc == 0){
		if(hi258_sysfs_add(&client->dev.kobj) < 0)
			pr_err("hi258: failed hi258_sysfs_add\n");
	}

	return rc;
#endif
}

static struct i2c_driver hi258_i2c_driver = {
	.id_table = hi258_i2c_id,
	.probe  = msm_hi258_i2c_probe,
	.driver = {
		.name = HI258_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client hi258_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static const struct of_device_id hi258_dt_match[] = {
	{.compatible = "qcom,hi258", .data = &hi258_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, hi258_dt_match);

static struct platform_driver hi258_platform_driver = {
	.driver = {
		.name = "qcom,hi258",
		.owner = THIS_MODULE,
		.of_match_table = hi258_dt_match,
	},
};

static void hi258_i2c_write_table(struct msm_sensor_ctrl_t *s_ctrl,
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
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write(
				s_ctrl->sensor_i2c_client, table->reg_addr,
				table->reg_data,
				MSM_CAMERA_I2C_BYTE_DATA);
		}
		table++;
	}
}

static int32_t hi258_platform_probe(struct platform_device *pdev)
{
	int32_t rc;
	const struct of_device_id *match;
	match = of_match_device(hi258_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);

	if(rc < 0){
		pr_err("%s: %d failed\n",__func__,__LINE__);
		return -EIO;
	}

	rc = hi258_sysfs_add_symlink(&pdev->dev);

	return rc;
}

static int __init hi258_init_module(void)
{
	int32_t rc;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&hi258_platform_driver,
		hi258_platform_probe);
	if (!rc)
		return rc;
	return i2c_add_driver(&hi258_i2c_driver);
}

static void __exit hi258_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (hi258_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&hi258_s_ctrl);
		platform_driver_unregister(&hi258_platform_driver);
	} else
		i2c_del_driver(&hi258_i2c_driver);
	return;
}
#if 1
static int32_t hi258_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint16_t chipid = 0;
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
		s_ctrl->sensor_i2c_client,
		s_ctrl->sensordata->slave_info->sensor_id_reg_addr,
		&chipid, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: %s: hi258 read id failed\n", __func__,
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
#endif

/*-----------------------------------------------------------------------------------------------------------------
------------------------------------[START] To fast tune register of SOC type ----------------------------------------------
------------------------------------------------------------------------------------------------------------------*/
#if defined(CONFIG_FAST_TUNE_REGISTER)
	const static struct msm_camera_i2c_reg_conf *hi258_recommend_settings_tuning;
	static struct msm_camera_i2c_reg_conf temp_hi258_recommend_settings[1200];

	const static struct msm_camera_i2c_reg_conf *hi258_preview_settings_tuning;
	static struct msm_camera_i2c_reg_conf temp_hi258_preview_settings[150];

	const static struct msm_camera_i2c_reg_conf *hi258_capture_settings_tuning;
	static struct msm_camera_i2c_reg_conf temp_hi258_capture_settings[60];

	const static struct msm_camera_i2c_reg_conf *hi258_HDvideo_settings_tuning;
	static struct msm_camera_i2c_reg_conf temp_hi258_HDvideo_settings[1200];

	const static struct msm_camera_i2c_reg_conf *hi258_video_settings_tuning;
	static struct msm_camera_i2c_reg_conf temp_hi258_video_settings[1200];

	const static struct msm_camera_i2c_reg_conf *hi258_reg_scene_tuning;
	static struct msm_camera_i2c_reg_conf temp_hi258_reg_scene[50];

	const static struct msm_camera_i2c_reg_conf *hi258_reg_wb_tuning;
	static struct msm_camera_i2c_reg_conf temp_hi258_reg_wb[50];

static uint16_t recommend_size, recommend_cnt = 0;
static uint16_t prev_size, prev_cnt = 0;
static uint16_t snap_size, snap_cnt = 0;
static uint16_t hdvideo_size, hdvideo_cnt = 0;
static uint16_t video_size, video_cnt = 0;
static uint16_t scene_size, scene_cnt = 0;
static uint16_t wb_size, wb_cnt = 0;

static void hi258_parsing_register(char* buf, int buf_size)
{
	int i = 0;
	unsigned int addr = 0;
	unsigned int value = 0;
	//int type = 0;
	//char data_type[25];
	int rc = 0;

	char scan_buf[40];	//suejong.kwon@lge.com addr, value, data_type's total length
	int scan_buf_len = 0;
	char subject = 0;

	CDBG("%s:%d Enter \n", __func__, __LINE__);

	while (i < buf_size) {
	 // select subject
	 if (buf[i] == '<') {
	  subject = buf[++i];
	  while(buf[++i] != '>');
	 }

	 // code delude
		if (buf[i] == '{') {
			scan_buf_len = 0;
			while(buf[i] != '}') {
				if (buf[i] < 38 || 126 < buf[i]) {
					++i;
					continue;
				}else
					scan_buf[scan_buf_len++] = buf[i++];
			}

			scan_buf[scan_buf_len++] = buf[i];
			scan_buf[scan_buf_len] = 0;

			rc = sscanf(scan_buf, "{%x, %x}", &addr, &value);

			if (rc!=2) {
				pr_err("%s:%d file format error. rc = %d\n", __func__, __LINE__, rc);
				return;
			}

			//pr_err("%s:%d file format %x, %x,  \n", __func__, __LINE__,addr,value);

			switch (subject) {
			 case 'A' : {
			 	temp_hi258_recommend_settings[recommend_cnt].reg_addr= addr;
				temp_hi258_recommend_settings[recommend_cnt].reg_data= value;
				//temp_hi258_recommend_settings[recommend_cnt].dt = MSM_CAMERA_I2C_BYTE_DATA;

				++recommend_cnt;
				break;
			 }

			 case 'B' : {
			 	temp_hi258_preview_settings[prev_cnt].reg_addr = addr;
				temp_hi258_preview_settings[prev_cnt].reg_data = value;
				//temp_hi258_preview_settings[prev_cnt].dt  = MSM_CAMERA_I2C_BYTE_DATA;

				++prev_cnt;
				break;
			 }

			 case 'C' : {
				temp_hi258_capture_settings[snap_cnt].reg_addr = addr;
				temp_hi258_capture_settings[snap_cnt].reg_data = value;
				//temp_hi258_capture_settings[snap_cnt].dt  = MSM_CAMERA_I2C_BYTE_DATA;

				++snap_cnt;
				break;
			 }

			 case 'D' : {
			 	temp_hi258_HDvideo_settings[hdvideo_cnt].reg_addr = addr;
				temp_hi258_HDvideo_settings[hdvideo_cnt].reg_data = value;
				//temp_hi258_HDvideo_settings[hdvideo_cnt].dt  = MSM_CAMERA_I2C_BYTE_DATA;

				++hdvideo_cnt;
				break;
			 }

			 case 'E' : {
			 	temp_hi258_video_settings[video_cnt].reg_addr = addr;
				temp_hi258_video_settings[video_cnt].reg_data = value;
				//temp_hi258_video_settings[video_cnt].dt  = MSM_CAMERA_I2C_BYTE_DATA;

				++video_cnt;
				break;
			 }

			 case 'F' : {
			 	temp_hi258_reg_scene[scene_cnt].reg_addr = addr;
				temp_hi258_reg_scene[scene_cnt].reg_data = value;
				//temp_hi258_reg_scene[scene_cnt].dt  = MSM_CAMERA_I2C_BYTE_DATA;

				++scene_cnt;
				break;
			 }

			 case 'G' : {
				 temp_hi258_reg_wb[wb_cnt].reg_addr = addr;
				 temp_hi258_reg_wb[wb_cnt].reg_data = value;
				// temp_hi258_reg_wb[wb_cnt].dt	= MSM_CAMERA_I2C_BYTE_DATA;

				 ++wb_cnt;
				 break;
			  }

			  default :
			   break;
			 }
		 }
		 ++i;
	 }
}

static void hi258_release_parsing_register(void) {
	//Init - recommend
	hi258_recommend_settings_tuning = temp_hi258_recommend_settings;
	recommend_size = recommend_cnt;
	pr_err("%s:%d recommend_size = %d\n", __func__, __LINE__, recommend_size);

	// Preview
	hi258_preview_settings_tuning = temp_hi258_preview_settings;
	prev_size = prev_cnt;
	pr_err("%s:%d prev_size = %d\n", __func__, __LINE__, prev_size);

	// Capture
	hi258_capture_settings_tuning = temp_hi258_capture_settings;
	snap_size = snap_cnt;
	pr_err("%s:%d snap_size = %d\n", __func__, __LINE__, snap_size);

	// 720P recording
	hi258_HDvideo_settings_tuning = temp_hi258_HDvideo_settings;
	hdvideo_size = hdvideo_cnt;
	pr_err("%s:%d hp_size = %d\n", __func__, __LINE__, hdvideo_size);

	// Video recording
	hi258_video_settings_tuning = temp_hi258_video_settings;
	video_size = video_cnt;
	pr_err("%s:%d video_size = %d\n", __func__, __LINE__, video_size);

	// Scene
	hi258_reg_scene_tuning = temp_hi258_reg_scene;
	scene_size = scene_cnt;
	pr_err("%s:%d scene_size = %d\n", __func__, __LINE__, scene_size);

	// White Balance
	hi258_reg_wb_tuning = temp_hi258_reg_wb;
	wb_size = wb_cnt;
	pr_err("%s:%d wb_size = %d\n", __func__, __LINE__, wb_size);

	recommend_cnt = 0;
	prev_cnt = 0;
	snap_cnt = 0;
	hdvideo_cnt = 0;
	video_cnt = 0;
	scene_cnt = 0;
	wb_cnt = 0;
}

static void hi258_read_register_from_file1(void)
{
	int fd =0;
	mm_segment_t oldfs = get_fs();
	char* buf;
	int read_size;

	set_fs(KERNEL_DS);

	fd = sys_open("/data/hi258_reg1.txt", O_RDONLY|O_LARGEFILE, 0777);

	if (fd < 0) {
		pr_err("%s:%d File from Internal Memory open fail fd = %d\n", __func__, __LINE__, fd);
		fd = sys_open("/storage/external_SD/hi258_reg1.txt", O_RDONLY |O_LARGEFILE, 0777);
	}

	if (fd < 0) {
		pr_err("%s:%d File from Internal Memory & SD card open fail fd = %d\n", __func__, __LINE__, fd);
		goto err_open_file;
	}

	buf = kmalloc(BUF_SIZE, GFP_KERNEL);
	if (!buf) {
		pr_err("%s:%d Memory alloc fail\n", __func__, __LINE__);
		goto err_buf;
	}

 	pr_err("%s:%d memory allocation\n", __func__, __LINE__);
	read_size = sys_read(fd, buf, BUF_SIZE);

	if (read_size < 0) {
		pr_err("%s:%d File read fail: read_size = %d\n", __func__, __LINE__, read_size);
		goto err_read;
	}

	hi258_parsing_register(buf, read_size);
err_read:
	kfree(buf);
err_buf:
	sys_close(fd);
err_open_file:
	set_fs(oldfs);
}

static void hi258_read_register_from_file2(void)
{
	int fd;
	mm_segment_t oldfs;
	char* buf;
	int read_size;

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	fd = sys_open("/data/hi258_reg2.txt", O_RDONLY |O_LARGEFILE, 0777);

	if (fd < 0) {
		pr_err("%s:%d File from Internal Memory open fail\n", __func__, __LINE__);
		fd = sys_open("/storage/external_SD/hi258_reg2.txt", O_RDONLY |O_LARGEFILE, 0777);
	}

	if (fd < 0) {
		pr_err("%s:%d File from Internal Memory & SD card open fail\n", __func__, __LINE__);
		goto err_open_file;
	}

	buf = kmalloc(BUF_SIZE, GFP_KERNEL);
	if (!buf) {
		pr_err("%s:%d Memory alloc fail\n", __func__, __LINE__);
		goto err_buf;
	}

 	pr_err("%s:%d memory allocation\n", __func__, __LINE__);
	read_size = sys_read(fd, buf, BUF_SIZE);

	if (read_size < 0) {
		pr_err("%s:%d File read fail: read_size = %d\n", __func__, __LINE__, read_size);
		goto err_read;
	}

	hi258_parsing_register(buf, read_size);
err_read:
	kfree(buf);
err_buf:
	sys_close(fd);
err_open_file:
	set_fs(oldfs);
}

static void hi258_read_register_from_file3(void)
{
	int fd;
	mm_segment_t oldfs;
	char* buf;
	int read_size;

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	fd = sys_open("/data/hi258_reg3.txt", O_RDONLY |O_LARGEFILE, 0777);

	if (fd < 0) {
		pr_err("%s:%d File from Internal Memory open fail\n", __func__, __LINE__);
		fd = sys_open("/storage/external_SD/hi258_reg3.txt", O_RDONLY |O_LARGEFILE, 0777);
	}

	if (fd < 0) {
		pr_err("%s:%d File from Internal Memory & SD card open fail\n", __func__, __LINE__);
		goto err_open_file;
	}

	buf = kmalloc(BUF_SIZE, GFP_KERNEL);
	if (!buf) {
		pr_err("%s:%d Memory alloc fail\n", __func__, __LINE__);
		goto err_buf;
	}

 pr_err("%s:%d memory allocation\n", __func__, __LINE__);
	read_size = sys_read(fd, buf, BUF_SIZE);

	if (read_size < 0) {
		pr_err("%s:%d File read fail: read_size = %d\n", __func__, __LINE__, read_size);
		goto err_read;
	}

	hi258_parsing_register(buf, read_size);
	hi258_release_parsing_register();

	TUNING_REGISTER = 1;
err_read:
	kfree(buf);
err_buf:
	sys_close(fd);
err_open_file:
	set_fs(oldfs);
}

static void hi258_update_register(struct msm_sensor_ctrl_t *s_ctrl)
{
	pr_err("%s START to Read the file from internal or SDcard\n", __func__);
	hi258_read_register_from_file1();
	msleep(1);
	hi258_read_register_from_file2();
	msleep(1);
	hi258_read_register_from_file3();
	msleep(500);

	pr_err("%s END to Read the file from internal or SDcard\n", __func__);

}
#endif //CONFIG_FAST_TUNE_REGISTER
/*-----------------------------------------------------------------------------------------------------------------
------------------------------------[END] To fast tune register of SOC type ---------------------------------------------
------------------------------------------------------------------------------------------------------------------*/

static void hi258_set_exposure_compensation(struct msm_sensor_ctrl_t *s_ctrl,
	int value)
{
	int val = value; //(value + 12) / 6;
	CDBG("%s %d", __func__, val);
	hi258_i2c_write_table(s_ctrl, &HI258_reg_exposure_compensation[val][0],
		ARRAY_SIZE(HI258_reg_exposure_compensation[val]));
}
#if 0 //LGE_CHANGE_S. not support features. 2015-01-23. sujeong.kwon@lge.com
static void hi258_set_effect(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	pr_debug("%s %d", __func__, value);
	switch (value) {
	case MSM_CAMERA_EFFECT_MODE_OFF: {
		hi258_i2c_write_table(s_ctrl, &HI258_reg_effect_normal[0],
			ARRAY_SIZE(HI258_reg_effect_normal));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_MONO: {
		hi258_i2c_write_table(s_ctrl, &HI258_reg_effect_black_white[0],
			ARRAY_SIZE(HI258_reg_effect_black_white));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_NEGATIVE: {
		hi258_i2c_write_table(s_ctrl, &HI258_reg_effect_negative[0],
			ARRAY_SIZE(HI258_reg_effect_negative));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_SEPIA: {
		hi258_i2c_write_table(s_ctrl, &HI258_reg_effect_old_movie[0],
			ARRAY_SIZE(HI258_reg_effect_old_movie));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_SOLARIZE: {
		hi258_i2c_write_table(s_ctrl, &HI258_reg_effect_solarize[0],
			ARRAY_SIZE(HI258_reg_effect_solarize));
		break;
	}
	default:
		hi258_i2c_write_table(s_ctrl, &HI258_reg_effect_normal[0],
			ARRAY_SIZE(HI258_reg_effect_normal));
	}
}

static void hi258_set_antibanding(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	pr_debug("%s %d", __func__, value);
	hi258_i2c_write_table(s_ctrl, &HI258_reg_antibanding[value][0],
		ARRAY_SIZE(HI258_reg_antibanding[value]));
}

static void hi258_set_scene_mode(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	pr_debug("%s %d", __func__, value);
	switch (value) {
	case MSM_CAMERA_SCENE_MODE_OFF: {
		hi258_i2c_write_table(s_ctrl, &HI258_reg_scene_auto[0],
			ARRAY_SIZE(HI258_reg_scene_auto));
		break;
	}
	case MSM_CAMERA_SCENE_MODE_NIGHT: {
		hi258_i2c_write_table(s_ctrl, &HI258_reg_scene_night[0],
			ARRAY_SIZE(HI258_reg_scene_night));
					break;
	}
	case MSM_CAMERA_SCENE_MODE_LANDSCAPE: {
		hi258_i2c_write_table(s_ctrl, &HI258_reg_scene_landscape[0],
			ARRAY_SIZE(HI258_reg_scene_landscape));
		break;
	}
	case MSM_CAMERA_SCENE_MODE_PORTRAIT: {
		hi258_i2c_write_table(s_ctrl, &HI258_reg_scene_portrait[0],
			ARRAY_SIZE(HI258_reg_scene_portrait));
		break;
	}
	default:
		hi258_i2c_write_table(s_ctrl, &HI258_reg_scene_auto[0],
			ARRAY_SIZE(HI258_reg_scene_auto));
	}
}

static void hi258_set_white_balance_mode(struct msm_sensor_ctrl_t *s_ctrl,
	int value)
{
	pr_debug("%s %d", __func__, value);
	switch (value) {
	case MSM_CAMERA_WB_MODE_AUTO: {
		hi258_i2c_write_table(s_ctrl, &HI258_reg_wb_auto[0],
			ARRAY_SIZE(HI258_reg_wb_auto));
		break;
	}
	case MSM_CAMERA_WB_MODE_INCANDESCENT: {
		hi258_i2c_write_table(s_ctrl, &HI258_reg_wb_home[0],
			ARRAY_SIZE(HI258_reg_wb_home));
		break;
	}
	case MSM_CAMERA_WB_MODE_DAYLIGHT: {
		hi258_i2c_write_table(s_ctrl, &HI258_reg_wb_sunny[0],
			ARRAY_SIZE(HI258_reg_wb_sunny));
					break;
	}
	case MSM_CAMERA_WB_MODE_FLUORESCENT: {
		hi258_i2c_write_table(s_ctrl, &HI258_reg_wb_office[0],
			ARRAY_SIZE(HI258_reg_wb_office));
					break;
	}
	case MSM_CAMERA_WB_MODE_CLOUDY_DAYLIGHT: {
		hi258_i2c_write_table(s_ctrl, &HI258_reg_wb_cloudy[0],
			ARRAY_SIZE(HI258_reg_wb_cloudy));
					break;
	}
	default:
		hi258_i2c_write_table(s_ctrl, &HI258_reg_wb_auto[0],
		ARRAY_SIZE(HI258_reg_wb_auto));
	}
}
#endif //LGE_CHANGE_E. not support features. 2015-01-23. sujeong.kwon@lge.com

int32_t hi258_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	long rc = 0;
	int32_t i = 0;
	mutex_lock(s_ctrl->msm_sensor_mutex);
	pr_err("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
		s_ctrl->sensordata->sensor_name, cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_GET_SENSOR_INFO:
		CDBG("%s, CFG_GET_SENSOR_INFO!!\n", __func__);
		memcpy(cdata->cfg.sensor_info.sensor_name,
			s_ctrl->sensordata->sensor_name,
			sizeof(cdata->cfg.sensor_info.sensor_name));
		cdata->cfg.sensor_info.session_id =
			s_ctrl->sensordata->sensor_info->session_id;
		for (i = 0; i < SUB_MODULE_MAX; i++)
			cdata->cfg.sensor_info.subdev_id[i] =
				s_ctrl->sensordata->sensor_info->subdev_id[i];
		cdata->cfg.sensor_info.is_mount_angle_valid =
			s_ctrl->sensordata->sensor_info->is_mount_angle_valid;
		cdata->cfg.sensor_info.sensor_mount_angle =
			s_ctrl->sensordata->sensor_info->sensor_mount_angle;
		cdata->cfg.sensor_info.position =
			s_ctrl->sensordata->sensor_info->position;
		cdata->cfg.sensor_info.modes_supported =
			s_ctrl->sensordata->sensor_info->modes_supported;
		CDBG("%s:%d sensor name %s\n", __func__, __LINE__,
			cdata->cfg.sensor_info.sensor_name);
		CDBG("%s:%d session id %d\n", __func__, __LINE__,
			cdata->cfg.sensor_info.session_id);
		for (i = 0; i < SUB_MODULE_MAX; i++)
			CDBG("%s:%d subdev_id[%d] %d\n", __func__, __LINE__, i,
				cdata->cfg.sensor_info.subdev_id[i]);
		CDBG("%s:%d mount angle valid %d value %d\n", __func__,
			__LINE__, cdata->cfg.sensor_info.is_mount_angle_valid,
			cdata->cfg.sensor_info.sensor_mount_angle);

		break;
	case CFG_SET_INIT_SETTING:
		CDBG("CFG_SET_INIT_SETTING  enter\n");

		#if defined(CONFIG_FAST_TUNE_REGISTER)
			if(TUNING_REGISTER){
				hi258_i2c_write_table(s_ctrl,(struct msm_camera_i2c_reg_conf *)hi258_recommend_settings_tuning, recommend_size);
			}else{
				hi258_i2c_write_table(s_ctrl, &hi258_recommend_settings[0], ARRAY_SIZE(hi258_recommend_settings));
			}
		#else
			hi258_i2c_write_table(s_ctrl,&hi258_recommend_settings[0], ARRAY_SIZE(hi258_recommend_settings));
		#endif
		pr_err("CFG_SET_INIT_SETTING  end\n");
		break;
	case CFG_SET_RESOLUTION: {
		int val = 0;
		if (copy_from_user(&val,
			(void *)cdata->cfg.setting, sizeof(int))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		hi258_prev_res = hi258_cur_res;
		hi258_cur_res = val ;
		pr_err("CFG_SET_RESOLUTION val =%d hi258_prev_res =%d, hi258_cur_res =%d \n",val, hi258_prev_res, hi258_cur_res);

		switch(val){
			case 0:	{
				#if defined(CONFIG_FAST_TUNE_REGISTER)
					if(TUNING_REGISTER){
						hi258_i2c_write_table(s_ctrl,(struct msm_camera_i2c_reg_conf *)hi258_capture_settings_tuning, snap_size);
					}else{
						hi258_i2c_write_table(s_ctrl,&hi258_capture_settings[0],ARRAY_SIZE(hi258_capture_settings));
					}
				#else
					hi258_i2c_write_table(s_ctrl, &hi258_capture_settings[0],ARRAY_SIZE(hi258_capture_settings));
				#endif
				pr_err("cam resolution capture setting\n");
				break;
			}
			case 1:	{
				#if defined(CONFIG_FAST_TUNE_REGISTER)
					if(TUNING_REGISTER){
						if(hi258_prev_res ==2 || hi258_prev_res ==3){
							hi258_i2c_write_table(s_ctrl,(struct msm_camera_i2c_reg_conf *)hi258_recommend_settings_tuning, recommend_size);
							pr_err("cam resolution init end\n");
						}
						hi258_i2c_write_table(s_ctrl,(struct msm_camera_i2c_reg_conf *)hi258_preview_settings_tuning, prev_size);
					}else{
						if(hi258_prev_res ==2 || hi258_prev_res ==3){
							hi258_i2c_write_table(s_ctrl, &hi258_recommend_settings[0],ARRAY_SIZE(hi258_recommend_settings));
							pr_err("cam resolution init end\n");
						}
						hi258_i2c_write_table(s_ctrl,&hi258_preview_settings[0],ARRAY_SIZE(hi258_preview_settings));
					}
				#else
					if(hi258_prev_res ==2 || hi258_prev_res ==3){
						hi258_i2c_write_table(s_ctrl, &hi258_recommend_settings[0],ARRAY_SIZE(hi258_recommend_settings));
						pr_err("cam resolution init end\n");
					}
					hi258_i2c_write_table(s_ctrl, &hi258_preview_settings[0],ARRAY_SIZE(hi258_preview_settings));
				#endif
				pr_err("cam resolution preview setting\n");
				break;
			}
			case 2: {
				#if defined(CONFIG_FAST_TUNE_REGISTER)
					if(TUNING_REGISTER){
						hi258_i2c_write_table(s_ctrl,(struct msm_camera_i2c_reg_conf *)hi258_HDvideo_settings_tuning, hdvideo_size);
					}else{
						hi258_i2c_write_table(s_ctrl,&hi258_HDvideo_settings[0],ARRAY_SIZE(hi258_HDvideo_settings));
					}
				#else
					hi258_i2c_write_table(s_ctrl, &hi258_HDvideo_settings[0],ARRAY_SIZE(hi258_HDvideo_settings));
				#endif
				pr_err("cam resolution HD video setting\n");
				break;
			}
			case 3: {
				#if defined(CONFIG_FAST_TUNE_REGISTER)
					if(TUNING_REGISTER){
						hi258_i2c_write_table(s_ctrl,(struct msm_camera_i2c_reg_conf *)hi258_video_settings_tuning, video_size);
					}else{
						hi258_i2c_write_table(s_ctrl,&hi258_video_settings[0],ARRAY_SIZE(hi258_video_settings));
					}
				#else
					hi258_i2c_write_table(s_ctrl, &hi258_video_settings[0],ARRAY_SIZE(hi258_video_settings));
				#endif
				pr_err("cam resolution video setting\n");
				break;
			}
			default:
				break;
		}

		CDBG("CFG_SET_RESOLUTION  end\n");
		break;
	}
	case CFG_SET_STOP_STREAM:
		pr_err("CFG_SET_STOP_STREAM  \n");
		break;
	case CFG_SET_START_STREAM:
		pr_err("CFG_SET_START_STREAM  \n");
		break;
	case CFG_GET_SENSOR_INIT_PARAMS:
		cdata->cfg.sensor_init_params.modes_supported =
			s_ctrl->sensordata->sensor_info->modes_supported;
		cdata->cfg.sensor_init_params.position =
			s_ctrl->sensordata->sensor_info->position;
		cdata->cfg.sensor_init_params.sensor_mount_angle =
			s_ctrl->sensordata->sensor_info->sensor_mount_angle;
		CDBG("%s:%d CFG_GET_SENSOR_INIT_PARAMS %d pos %d mount %d\n", __func__,
			__LINE__,
			cdata->cfg.sensor_init_params.modes_supported,
			cdata->cfg.sensor_init_params.position,
			cdata->cfg.sensor_init_params.sensor_mount_angle);
		break;
	case CFG_SET_SLAVE_INFO: {
		struct msm_camera_sensor_slave_info sensor_slave_info;
		struct msm_sensor_power_setting_array *power_setting_array;
		int slave_index = 0;
		CDBG("%s:%d CFG_SET_SLAVE_INFO  enter\n", __func__, __LINE__);
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
		CDBG("%s:%d CFG_SET_SLAVE_INFO  end\n", __func__, __LINE__);
		break;
	}
	case CFG_WRITE_I2C_ARRAY: {
		struct msm_camera_i2c_reg_setting conf_array;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		CDBG("%s:%d CFG_WRITE_I2C_ARRAY	enter\n", __func__, __LINE__);

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

		CDBG("%s:%d CFG_WRITE_I2C_ARRAY	end\n", __func__, __LINE__);
		kfree(reg_setting);
		break;
	}
	case CFG_WRITE_I2C_SEQ_ARRAY: {
		struct msm_camera_i2c_seq_reg_setting conf_array;
		struct msm_camera_i2c_seq_reg_array *reg_setting = NULL;
		CDBG("%s:%d CFG_WRITE_I2C_SEQ_ARRAY enter\n", __func__, __LINE__);

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
		CDBG("%s:%d CFG_WRITE_I2C_SEQ_ARRAY end\n", __func__, __LINE__);
		kfree(reg_setting);
		break;
	}

	/*LGE_CHANGE_S, add soc exif, 2013-10-04, kwangsik83.kim@lge.com*/
			case CFG_PAGE_MODE_READ_I2C_ARRAY:{
				int16_t size=0;
				uint16_t read_data_size = 0;
				uint16_t *read_data;
				uint16_t *read_data_head;
				struct msm_camera_i2c_reg_setting conf_array;
				struct msm_camera_i2c_reg_array *reg_setting = NULL;

				//pr_err("[WX] %s CFG_PAGE_MODE_READ_I2C_ARRAY\n", __func__);

				if (copy_from_user(&conf_array,
					(void *)cdata->cfg.setting,
					sizeof(struct msm_camera_i2c_reg_setting))) {
					pr_err("%s:%d failed\n", __func__, __LINE__);
					rc = -EFAULT;
					break;
				}

				size = conf_array.size; 	//size for write(page_mode) and read
				read_data_size = size - 1;	//size for read

				CDBG("[WX] %s: size : %d rsize : %d\n", __func__, size, read_data_size);

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
						CDBG("[WX] %s read_addr : 0x%x data : 0x%x\n", __func__, conf_array.reg_setting->reg_addr, *read_data);
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

				CDBG("[WX] %s done\n", __func__);

				break;
			}
	/*LGE_CHANGE_E, add soc exif, 2013-10-04, kwangsik83.kim@lge.com*/

	case CFG_SET_REGISTER_UPDATE: {
#if defined(CONFIG_FAST_TUNE_REGISTER)
		pr_err("%s CFG_SET_REGISTER_UPDATE is enabled\n", __func__);
		hi258_update_register(s_ctrl);
#else
		CDBG("%s CFG_SET_REGISTER_UPDATE is disabled\n", __func__);
#endif
		break;
	}

/*LGE_CHANGE_S, modified power-up/down status for recovery, 2013-12-27, hyungtae.lee@lge.com*/
	case CFG_POWER_UP:{
		CDBG("%s:%d CFG_POWER_UP enter\n", __func__, __LINE__);

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
		pr_err("%s:%d CFG_POWER_UP end\n", __func__, __LINE__);
		break;
	}
	case CFG_POWER_DOWN:{
		CDBG("%s:%d CFG_POWER_DOWN enter\n", __func__, __LINE__);

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
		pr_err("%s:%d CFG_POWER_DOWN end\n", __func__, __LINE__);
		break;
	}
/*LGE_CHANGE_E, modified power-up/down status for recovery, 2013-12-27, hyungtae.lee@lge.com*/
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
		break;
	}
	case CFG_SET_CONTRAST: {
		break;
	}
	case CFG_SET_SHARPNESS: {
		break;
	}
	case CFG_SET_ISO: {
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
		pr_debug("%s: Exposure compensation Value is %d",
			__func__, ec_lev);
		hi258_set_exposure_compensation(s_ctrl, ec_lev);
		break;
	}
	case CFG_SET_EFFECT: {
		pr_err("%s: We do not support effect feature Value now\n", __func__);
#if 0
		int32_t effect_mode;
		if (copy_from_user(&effect_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: Effect mode is %d", __func__, effect_mode);
		hi258_set_effect(s_ctrl, effect_mode);
#endif
		break;
	}
	case CFG_SET_ANTIBANDING: {
		pr_err("%s: We do not support Antibanding feature Value now\n", __func__);
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
		hi258_set_antibanding(s_ctrl, antibanding_mode);
#endif
		break;
	}
	case CFG_SET_BESTSHOT_MODE: {
		pr_err("%s: We do not support scene feature Value now\n", __func__);
#if 0
		int32_t bs_mode;
		if (copy_from_user(&bs_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: best shot mode is %d", __func__, bs_mode);

		#if defined(CONFIG_FAST_TUNE_REGISTER)
			if(TUNING_REGISTER){
				hi258_i2c_write_table(s_ctrl,(struct msm_camera_i2c_reg_conf *)hi258_reg_scene_tuning, scene_size);
			}else{
				hi258_set_scene_mode(s_ctrl, bs_mode);
			}
		#else
			hi258_set_scene_mode(s_ctrl, bs_mode);
		#endif
#endif
		break;
	}
	case CFG_SET_WHITE_BALANCE: {
		pr_err("%s: We do not support white balance feature Value now\n", __func__);
#if 0
		int32_t wb_mode;
		if (copy_from_user(&wb_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: white balance is %d", __func__, wb_mode);

		#if defined(CONFIG_FAST_TUNE_REGISTER)
			if(TUNING_REGISTER){
				hi258_i2c_write_table(s_ctrl,(struct msm_camera_i2c_reg_conf *)hi258_reg_wb_tuning, wb_size);
			}else{
				hi258_set_white_balance_mode(s_ctrl, wb_mode);
			}
		#else
			hi258_set_white_balance_mode(s_ctrl, wb_mode);
		#endif
#endif
		break;
	}
	//LGE_CHANGE_E,  These options has beend added due to colour effect issue. youngwook.song@lge.com 2013-11-25
	case CFG_SET_AEC_LOCK:
	case CFG_SET_AWB_LOCK:
	case CFG_SET_AEC_ROI:
		pr_debug("%s: We do not support features value related to LOCK now\n", __func__);
		break;
	//LGE_CHANGE_X,  These options has beend added due to colour effect issue. youngwook.song@lge.com 2013-11-25:
	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}

static struct msm_sensor_fn_t hi258_sensor_func_tbl = {
	.sensor_config = hi258_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	//.sensor_match_id = msm_sensor_match_id,
	.sensor_match_id = hi258_sensor_match_id,
};

static struct msm_sensor_ctrl_t hi258_s_ctrl = {
	.sensor_i2c_client = &hi258_sensor_i2c_client,
	.power_setting_array.power_setting = hi258_power_setting,
	.power_setting_array.size = ARRAY_SIZE(hi258_power_setting),
	.msm_sensor_mutex = &hi258_mut,
	.sensor_v4l2_subdev_info = hi258_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(hi258_subdev_info),
	.func_tbl = &hi258_sensor_func_tbl,
};

module_init(hi258_init_module);
module_exit(hi258_exit_module);
MODULE_DESCRIPTION("Hi258 2MP YUV sensor driver");
MODULE_LICENSE("GPL v2");
