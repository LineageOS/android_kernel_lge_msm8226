/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
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
#include "../../../../../base/base.h" /* LGE_CHANGE . To read antibanding value. sujeong.kwon@lge.com 2014.04.11*/
#define MT9M114_SENSOR_NAME "mt9m114"
#define PLATFORM_DRIVER_NAME "msm_camera_mt9m114"
#define mt9m114_obj mt9m114_##obj

#define CONFIG_MSMB_CAMERA_DEBUG 1

#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

DEFINE_MSM_MUTEX(mt9m114_mut);

typedef enum {
  MT9M114_60HZ,
  MT9M114_50HZ,
  MT9M114_HZ_MAX_NUM,
}MT9M114AntibandingType;

static int mt9m114_antibanding = MT9M114_60HZ;  //sujeong.kwon@lge.com 2014-04-02.



/* LGE_CHANGE_S : To apply for  Manual Flicker: antibanding, 2014-04-11, sujeong.kwon@lge.com */
static ssize_t mt9m114_antibanding_store(struct device* dev, struct device_attribute* attr, const char* buf, size_t n)
{
       int val =0;

       sscanf(buf,"%d",&val);
       printk("mt9m114: antibanding type [0x%x] \n",val);

       /* 1 : Antibanding 60Hz        * 2 : Antibanding 50Hz */
       switch(val)
       {
			case 1:
				mt9m114_antibanding = MT9M114_60HZ;
				break;
			case 2:
				mt9m114_antibanding = MT9M114_50HZ;
				break;
			default:
			pr_err("mt9m114: invalid antibanding type[%d] \n",val);
			mt9m114_antibanding = MT9M114_50HZ;
			break;
		}

	return n;
}

static DEVICE_ATTR(antibanding, /*S_IRUGO|S_IWUGO*/ 0664, NULL, mt9m114_antibanding_store);


static struct device_attribute* mt9m114_sysfs_symlink[] = {
       &dev_attr_antibanding,
	   	NULL,
};

static int mt9m114_sysfs_add_symlink(struct device *dev)
{
	 int i = 0;
	 int rc = 0;
	 int n =0;
	 struct bus_type *bus = dev->bus;

	n = ARRAY_SIZE(mt9m114_sysfs_symlink);
	for(i = 0; i < n; i++){
		if(mt9m114_sysfs_symlink[i]){
			rc = device_create_file(dev, mt9m114_sysfs_symlink[i]);
				if(rc < 0){
					pr_err("mt9m114_sysfs_symlink is not created\n");
					goto out_unreg;
				}
			}
		}

	if(bus){
		#if 0//defined(CONFIG_MACH_MSM8226_E7WIFI)
		sysfs_remove_link(&bus->p->devices_kset->kobj, "cam_sensor_vt");
		msleep(10);
		#endif
		//  PATH of bus->p->devices_kset = /sys/bus/platform/devices/
		rc = sysfs_create_link(&bus->p->devices_kset->kobj, &dev->kobj, "cam_sensor_vt");

		if(rc)
			goto out_unlink;
	}

	pr_err("mt9m114_sysfs_add_symlink is created\n");
	return 0;

out_unreg:
	pr_err("fail to creat device file for antibanding");
	for (; i >= 0; i--)
		device_remove_file(dev, mt9m114_sysfs_symlink[i]);

	return rc;

out_unlink:
	pr_err("fail to creat sys link for antibanding");
	sysfs_remove_link(&bus->p->devices_kset->kobj, "cam_sensor_vt");
	return rc;

};
/* LGE_CHANGE_E : To apply for Manual Flicker: antibanding, 2014-04-11, sujeong.kwon@lge.com */

static struct msm_sensor_ctrl_t mt9m114_s_ctrl;

static struct msm_sensor_power_setting mt9m114_power_setting[] = {
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
/* LGE CHANGE S, Add E7 LTE define, 2014-03-06, sangwoo25.park@lge.com */
#if defined(CONFIG_MACH_MSM8926_E7LTE_ATT_US) || defined (CONFIG_MACH_MSM8926_E7LTE_VZW_US) || defined (CONFIG_MACH_MSM8926_E8LTE_KR) || defined (CONFIG_MACH_MSM8926_E8LTE) || defined (CONFIG_MACH_MSM8926_E7LTE_USC_US) || defined(CONFIG_MACH_MSM8926_T8LTE_ATT_US) || defined (CONFIG_MACH_MSM8926_E9LTE_VZW_US)
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VDIG,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},
#else
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 15,
	},
#endif
#if defined(CONFIG_MACH_MSM8226_E7WIFI) || defined(CONFIG_MACH_MSM8226_E8WIFI) || defined(CONFIG_MACH_MSM8226_E9WIFI) || defined(CONFIG_MACH_MSM8226_E9WIFIN)
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
#endif
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VIO,
		.config_val = GPIO_OUT_HIGH,
		.delay = 15,
	},
#if defined(CONFIG_MACH_MSM8926_E7LTE_ATT_US) || defined (CONFIG_MACH_MSM8926_E7LTE_VZW_US) || defined (CONFIG_MACH_MSM8926_E8LTE_KR) || defined (CONFIG_MACH_MSM8926_E8LTE) || defined (CONFIG_MACH_MSM8926_E7LTE_USC_US) || defined(CONFIG_MACH_MSM8926_T8LTE_ATT_US) || defined (CONFIG_MACH_MSM8926_E9LTE_VZW_US)
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 10,
	},
#else
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 10,
	},
#endif
/* LGE CHANGE E, Add E7 LTE define, 2014-03-06, sangwoo25.park@lge.com */
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 50,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};


static struct msm_camera_i2c_reg_conf mt9m114_recommend_settings[] = {
	{0xFFFD, 0x0564},	 /* [POLLING CMD(16)], [TIME(8) | COUNT(8)] */
	{0x0080, 0x0002},    /* [REG ADDR(16)], [MASK(16)] */

	/* BITFIELD= 0x301A, 0x0200(mask), 0x0001(value) */
	{0xFFFC, 0x0001},	 /* [BITFIELD CMD(16)], [VALUE(16)] */
	{0x301A, 0x0200},	 /* [REG ADDR(16)], [MASK(16)] */

	//{0x301A, 0x0230,}, 	// RESET_REGISTER
	//{0xFFFF, 0x0064,},	// DELAY=100

	//[Step2-PLL_Timing]
	//{0x098E, 0x1000,}, 	// LOGICAL_ADDRESS_ACCESS
	{0x098E, 0xC97E,}, 	// LOGICAL_ADDRESS_ACCESS
	{0x0990, 0x0100,},	// CAM_SYSCTL_PLL_ENABLE
	{0xC980, 0x0120,},	//cam_sysctl_pll_divider_m_n = 1632
	{0xC982, 0x0700,},	//cam_sysctl_pll_divider_p = 1792
	//Timing_settings
	{0xC800, 0x0004,},	//cam_sensor_cfg_y_addr_start = 4
	{0xC802, 0x0004,},	//cam_sensor_cfg_x_addr_start = 4
	{0xC804, 0x03CB,},	//cam_sensor_cfg_y_addr_end = 971
	{0xC806, 0x050B,},	//cam_sensor_cfg_x_addr_end = 1291
	{0x098E, 0x4808,}, 	// LOGICAL_ADDRESS_ACCESS		===== 32
	{0x0990, 0x02DC,},	// cam_sensor_cfg_pixclk
	{0x0992, 0x6C00,},	// cam_sensor_cfg_pixclk
	// 0xC808, 0x2DC1A9E	//cam_sensor_cfg_pixclk = 47979166
	{0xC80C, 0x0001,},	//cam_sensor_cfg_row_speed = 1
	{0xC80E, 0x00DB,},	//cam_sensor_cfg_fine_integ_time_min = 219
	{0xC810, 0x05B3,},	//cam_sensor_cfg_fine_integ_time_max = 1469 *****
	{0xC812, 0x03EE,},	//cam_sensor_cfg_frame_length_lines = 1500 *****
	{0xC814, 0x0636,},	//cam_sensor_cfg_line_length_pck = 1600 ******
	{0xC816, 0x0060,},	//cam_sensor_cfg_fine_correction = 96
	{0xC818, 0x03C3,},	//cam_sensor_cfg_cpipe_last_row = 963
	{0xC826, 0x0020,},	//cam_sensor_cfg_reg_0_data = 32
#if defined(CONFIG_MACH_MSM8226_E9WIFI) || defined(CONFIG_MACH_MSM8226_E9WIFIN) || defined(CONFIG_MACH_MSM8926_E9LTE_VZW_US)
	{0xC834, 0x0000,},/*sensor_control_read_mode = 0*/
#else
	{0xC834, 0x0003,},/*sensor_control_read_mode = 0*/
#endif
	{0xC854, 0x0000,},	//cam_crop_window_xoffset = 0
	{0xC856, 0x0000,},	//cam_crop_window_yoffset = 0
	{0xC858, 0x0500,},	//cam_crop_window_width = 1280
	{0xC85A, 0x03C0,},	//cam_crop_window_height = 960
	{0x098E, 0xC85C,},	//LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x0300,},	//cam_crop_cropmode = 3
	{0xC868, 0x0500,},	//cam_output_width = 1280
	{0xC86A, 0x03C0,},	//cam_output_height = 960
	{0x098E, 0xC878,},	//LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x0000,},	//cam_aet_aemode = 0
	{0xC88C, 0x1E02,},	//cam_aet_max_frame_rate = 5120 *****
	{0xC88E, 0x0800,},	//cam_aet_min_frame_rate = 2560 *****
	{0xC914, 0x0000,},	//cam_stat_awb_clip_window_xstart = 0
	{0xC916, 0x0000,},	//cam_stat_awb_clip_window_ystart = 0
	{0xC918, 0x04FF,},	//cam_stat_awb_clip_window_xend = 1279
	{0xC91A, 0x03BF,},	//cam_stat_awb_clip_window_yend = 959
	{0xC91C, 0x0000,},	//cam_stat_ae_initial_window_xstart = 0
	{0xC91E, 0x0000,},	//cam_stat_ae_initial_window_ystart = 0
	{0xC920, 0x00FF,},	//cam_stat_ae_initial_window_xend = 255
	{0xC922, 0x00BF,},	//cam_stat_ae_initial_window_yend = 191
	{0x098E, 0xE801,},	//LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x0000,},	//cam_aet_aemode = 0
	{0x098E, 0xCC03,},	// LOGICAL_ADDRESS_ACCESS [UVC_POWER_LINE_FREQUENCY_CONTROL]			===== 8
	{0x0990, 0x0200,},	// UVC_POWER_LINE_FREQUENCY_CONTROL  ==> 60Hz : 0x0200 , 50Hz ==>0x0100


	{0x098E, 0xDC00,}, 	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x2800,},	// AE_TRACK_AE_TRACKING_DAMPENING_SPEED
	{0x0080, 0x8002,}, 	// COMMAND_REGISTER

	/* POLLREG= 0x0080, 2, !=0, DELAY=1msec, TIMEOUT=100 */
	{0xFFFD, 0x0164},	 /* [POLLING CMD(16)], [TIME(8) | COUNT(8)] */
	{0x0080, 0x0002},	 /* [REG ADDR(16)], [MASK(16)] */

	//[Step3-Recommended] //Optimization
	{0x316A, 0x8270,},
	{0x316C, 0x8270,},
	{0x3ED0, 0x2305,},
	{0x3ED2, 0x77CF,},
	{0x316E, 0x8202,},
	{0x3180, 0x87FF,},
	{0x30D4, 0x6080,},
	{0xA802, 0x0008,},	// AE_TRACK_MODE
	{0x3E14, 0xFF39,},
	{0x301A, 0x0234,},
	{0x31E0, 0x0001,},  	//0x33F4, 0x0009

	//[Load Patch 1204]
	{0x0982, 0x0001,},	// ACCESS_CTL_STAT
	{0x098A, 0x60BC,},	// PHYSICAL_ADDRESS_ACCESS
	{0xE0BC, 0xC0F1,},
	{0xE0BE, 0x082A,},
	{0xE0C0, 0x05A0,},
	{0xE0C2, 0xD800,},
	{0xE0C4, 0x71CF,},
	{0xE0C6, 0xFFFF,},
	{0xE0C8, 0xC344,},
	{0xE0CA, 0x77CF,},
	{0xE0CC, 0xFFFF,},
	{0xE0CE, 0xC7C0,},
	{0xE0D0, 0xB104,},
	{0xE0D2, 0x8F1F,},
	{0xE0D4, 0x75CF,},
	{0xE0D6, 0xFFFF,},
	{0xE0D8, 0xC84C,},
	{0xE0DA, 0x0811,},
	{0xE0DC, 0x005E,},
	{0xE0DE, 0x70CF,},
	{0xE0E0, 0x0000,},
	{0xE0E2, 0x500E,},
	{0xE0E4, 0x7840,},
	{0xE0E6, 0xF019,},
	{0xE0E8, 0x0CC6,},
	{0xE0EA, 0x0340,},
	{0xE0EC, 0x0E26,},
	{0xE0EE, 0x0340,},
	{0xE0F0, 0x95C2,},
	{0xE0F2, 0x0E21,},
	{0xE0F4, 0x101E,},
	{0xE0F6, 0x0E0D,},
	{0xE0F8, 0x119E,},
	{0xE0FA, 0x0D56,},
	{0xE0FC, 0x0340,},
	{0xE0FE, 0xF008,},
	{0xE100, 0x2650,},
	{0xE102, 0x1040,},
	{0xE104, 0x0AA2,},
	{0xE106, 0x0360,},
	{0xE108, 0xB502,},
	{0xE10A, 0xB5C2,},
	{0xE10C, 0x0B22,},
	{0xE10E, 0x0400,},
	{0xE110, 0x0CCE,},
	{0xE112, 0x0320,},
	{0xE114, 0xD800,},
	{0xE116, 0x70CF,},
	{0xE118, 0xFFFF,},
	{0xE11A, 0xC5D4,},
	{0xE11C, 0x902C,},
	{0xE11E, 0x72CF,},
	{0xE120, 0xFFFF,},
	{0xE122, 0xE218,},
	{0xE124, 0x9009,},
	{0xE126, 0xE105,},
	{0xE128, 0x73CF,},
	{0xE12A, 0xFF00,},
	{0xE12C, 0x2FD0,},
	{0xE12E, 0x7822,},
	{0xE130, 0x7910,},
	{0xE132, 0xB202,},
	{0xE134, 0x1382,},
	{0xE136, 0x0700,},
	{0xE138, 0x0815,},
	{0xE13A, 0x03DE,},
	{0xE13C, 0x1387,},
	{0xE13E, 0x0700,},
	{0xE140, 0x2102,},
	{0xE142, 0x000A,},
	{0xE144, 0x212F,},
	{0xE146, 0x0288,},
	{0xE148, 0x1A04,},
	{0xE14A, 0x0284,},
	{0xE14C, 0x13B9,},
	{0xE14E, 0x0700,},
	{0xE150, 0xB8C1,},
	{0xE152, 0x0815,},
	{0xE154, 0x0052,},
	{0xE156, 0xDB00,},
	{0xE158, 0x230F,},
	{0xE15A, 0x0003,},
	{0xE15C, 0x2102,},
	{0xE15E, 0x00C0,},
	{0xE160, 0x7910,},
	{0xE162, 0xB202,},
	{0xE164, 0x9507,},
	{0xE166, 0x7822,},
	{0xE168, 0xE080,},
	{0xE16A, 0xD900,},
	{0xE16C, 0x20CA,},
	{0xE16E, 0x004B,},
	{0xE170, 0xB805,},
	{0xE172, 0x9533,},
	{0xE174, 0x7815,},
	{0xE176, 0x6038,},
	{0xE178, 0x0FB2,},
	{0xE17A, 0x0560,},
	{0xE17C, 0xB861,},
	{0xE17E, 0xB711,},
	{0xE180, 0x0775,},
	{0xE182, 0x0540,},
	{0xE184, 0xD900,},
	{0xE186, 0xF00A,},
	{0xE188, 0x70CF,},
	{0xE18A, 0xFFFF,},
	{0xE18C, 0xE210,},
	{0xE18E, 0x7835,},
	{0xE190, 0x8041,},
	{0xE192, 0x8000,},
	{0xE194, 0xE102,},
	{0xE196, 0xA040,},
	{0xE198, 0x09F1,},
	{0xE19A, 0x8094,},
	{0xE19C, 0x7FE0,},
	{0xE19E, 0xD800,},
	{0xE1A0, 0xC0F1,},
	{0xE1A2, 0xC5E1,},
	{0xE1A4, 0x71CF,},
	{0xE1A6, 0x0000,},
	{0xE1A8, 0x45E6,},
	{0xE1AA, 0x7960,},
	{0xE1AC, 0x7508,},
	{0xE1AE, 0x70CF,},
	{0xE1B0, 0xFFFF,},
	{0xE1B2, 0xC84C,},
	{0xE1B4, 0x9002,},
	{0xE1B6, 0x083D,},
	{0xE1B8, 0x021E,},
	{0xE1BA, 0x0D39,},
	{0xE1BC, 0x10D1,},
	{0xE1BE, 0x70CF,},
	{0xE1C0, 0xFF00,},
	{0xE1C2, 0x3354,},
	{0xE1C4, 0x9055,},
	{0xE1C6, 0x71CF,},
	{0xE1C8, 0xFFFF,},
	{0xE1CA, 0xC5D4,},
	{0xE1CC, 0x116C,},
	{0xE1CE, 0x0103,},
	{0xE1D0, 0x1170,},
	{0xE1D2, 0x00C1,},
	{0xE1D4, 0xE381,},
	{0xE1D6, 0x22C6,},
	{0xE1D8, 0x0F81,},
	{0xE1DA, 0x0000,},
	{0xE1DC, 0x00FF,},
	{0xE1DE, 0x22C4,},
	{0xE1E0, 0x0F82,},
	{0xE1E2, 0xFFFF,},
	{0xE1E4, 0x00FF,},
	{0xE1E6, 0x29C0,},
	{0xE1E8, 0x0222,},
	{0xE1EA, 0x7945,},
	{0xE1EC, 0x7930,},
	{0xE1EE, 0xB035,},
	{0xE1F0, 0x0715,},
	{0xE1F2, 0x0540,},
	{0xE1F4, 0xD900,},
	{0xE1F6, 0xF00A,},
	{0xE1F8, 0x70CF,},
	{0xE1FA, 0xFFFF,},
	{0xE1FC, 0xE224,},
	{0xE1FE, 0x7835,},
	{0xE200, 0x8041,},
	{0xE202, 0x8000,},
	{0xE204, 0xE102,},
	{0xE206, 0xA040,},
	{0xE208, 0x09F1,},
	{0xE20A, 0x8094,},
	{0xE20C, 0x7FE0,},
	{0xE20E, 0xD800,},
	{0xE210, 0xFFFF,},
	{0xE212, 0xCB40,},
	{0xE214, 0xFFFF,},
	{0xE216, 0xE0BC,},
	{0xE218, 0x0000,},
	{0xE21A, 0x0000,},
	{0xE21C, 0x0000,},
	{0xE21E, 0x0000,},
	{0xE220, 0x0000,},
	{0x098E, 0x0000,},      // LOGICAL_ADDRESS_ACCESS

	//[Apply Patch 1204]
	{0xE000, 0x1184,},      // PATCHLDR_LOADER_ADDRESS
	{0xE002, 0x1204,},      // PATCHLDR_PATCH_ID
	{0x098E, 0x6004,},      // PATCHLDR_FIRMWARE_ID
	{0x0990, 0x4103,},      // PATCHLDR_FIRMWARE_ID
	{0x0992, 0x0202,},      // PATCHLDR_FIRMWARE_ID

	{0x0080, 0xFFF0,},      // COMMAND_REGISTER

	/* POLLREG= 0x0080, 2, !=0, DELAY=1msec, TIMEOUT=100count */
	{0xFFFD, 0x0164},    /* [POLLING CMD(16)], [TIME(8) | COUNT(8)] */
	{0x0080, 0x0001},    /* [REG ADDR(16)], [MASK(16)] */

	{0x0080, 0xFFF1,},      // COMMAND_REGISTER

	/* POLLREG= 0x0080, 2, !=0, DELAY=1msec, TIMEOUT=100count */
	{0xFFFD, 0x0164},    /* [POLLING CMD(16)], [TIME(8) | COUNT(8)] */
	{0x0080, 0x0001},    /* [REG ADDR(16)], [MASK(16)] */

	{0xA804, 0x01BF,},

	//[Step4-APGA]
	//[APGA Settings 100% 2011/09/22 11:59:56]
	{0x3640, 0x0130,},
	{0x3642, 0x5CCC,},
	{0x3644, 0x7450,},
	{0x3646, 0xF06D,},
	{0x3648, 0xFC4F,},
	{0x364A, 0x0270,},
	{0x364C, 0x1FCC,},
	{0x364E, 0x7590,},
	{0x3650, 0x956D,},
	{0x3652, 0xBA4F,},
	{0x3654, 0x0330,},
	{0x3656, 0x2CCD,},
	{0x3658, 0x3630,},
	{0x365A, 0xC3CD,},
	{0x365C, 0xB92F,},
	{0x365E, 0x0130,},
	{0x3660, 0x5B8C,},
	{0x3662, 0x7410,},
	{0x3664, 0xEDED,},
	{0x3666, 0xF96F,},
	{0x3680, 0x102D,},
	{0x3682, 0x4ACC,},
	{0x3684, 0xEACA,},
	{0x3686, 0xD3EE,},
	{0x3688, 0xD16F,},
	{0x368A, 0x6B4C,},
	{0x368C, 0xD1AB,},
	{0x368E, 0x652D,},
	{0x3690, 0x5DCB,},
	{0x3692, 0xC80F,},
	{0x3694, 0x124D,},
	{0x3696, 0xC02D,},
	{0x3698, 0xD08C,},
	{0x369A, 0x584D,},
	{0x369C, 0xC40E,},
	{0x369E, 0x0F6D,},
	{0x36A0, 0x4A8C,},
	{0x36A2, 0x8D88,},
	{0x36A4, 0xD5AE,},
	{0x36A6, 0xE16F,},
	{0x36C0, 0x1091,},
	{0x36C2, 0x614B,},
	{0x36C4, 0x5EF1,},
	{0x36C6, 0xBBB0,},
	{0x36C8, 0x8074,},
	{0x36CA, 0x0DB1,},
	{0x36CC, 0x1ECD,},
	{0x36CE, 0x0312,},
	{0x36D0, 0xC1D0,},
	{0x36D2, 0x8FF4,},
	{0x36D4, 0x5E90,},
	{0x36D6, 0x226E,},
	{0x36D8, 0x74D1,},
	{0x36DA, 0x9DD1,},
	{0x36DC, 0x85F4,},
	{0x36DE, 0x10B1,},
	{0x36E0, 0x356C,},
	{0x36E2, 0x6611,},
	{0x36E4, 0xC890,},
	{0x36E6, 0x84B4,},
	{0x3700, 0x15CC,},
	{0x3702, 0xB40D,},
	{0x3704, 0xB4D1,},
	{0x3706, 0x5FD0,},
	{0x3708, 0x21B3,},
	{0x370A, 0xD60D,},
	{0x370C, 0xCD0E,},
	{0x370E, 0xDF10,},
	{0x3710, 0x53F0,},
	{0x3712, 0x3172,},
	{0x3714, 0xE34A,},
	{0x3716, 0x8BCC,},
	{0x3718, 0xBA31,},
	{0x371A, 0x09B0,},
	{0x371C, 0x0473,},
	{0x371E, 0x18AC,},
	{0x3720, 0xB90D,},
	{0x3722, 0xBE11,},
	{0x3724, 0x6610,},
	{0x3726, 0x27D3,},
	{0x3740, 0x8229,},
	{0x3742, 0xA450,},
	{0x3744, 0x88B5,},
	{0x3746, 0x4FF2,},
	{0x3748, 0x2DF6,},
	{0x374A, 0x50B0,},
	{0x374C, 0x9131,},
	{0x374E, 0x8C55,},
	{0x3750, 0x1C33,},
	{0x3752, 0x39F6,},
	{0x3754, 0x1030,},
	{0x3756, 0xB091,},
	{0x3758, 0x8795,},
	{0x375A, 0x7833,},
	{0x375C, 0x47D6,},
	{0x375E, 0xDBAA,},
	{0x3760, 0xBA10,},
	{0x3762, 0x8B95,},
	{0x3764, 0x6192,},
	{0x3766, 0x3456,},
	{0x3784, 0x0280,},
	{0x3782, 0x01E0,},
	{0x37C0, 0xF96A,},
	{0x37C2, 0x8C2B,},
	{0x37C4, 0xDE4B,},
	{0x37C6, 0x80AB,},
	{0x098E, 0x0000,},
	{0xC960, 0x0C80,},
	{0xC962, 0x79B6,},
	{0xC964, 0x5AA8,},
	{0xC966, 0x79BC,},
	{0xC968, 0x75A6,},
	{0xC96A, 0x1004,},
	{0xC96C, 0x98BD,},
	{0xC96E, 0x9249,},	//{0xC96E, 0x7DD0,} // for shading , 2014-04-09
	{0xC970, 0x8560,},
	{0xC972, 0x946F,},
	{0xC974, 0x1964,},
	{0xC976, 0x7BC0,},
	{0xC978, 0x8787,},
	{0xC97A, 0xA9F2,},
	{0xC97C, 0x89AE,},
	{0xC95E, 0x0003,},
	//[AE]
// start of test for AE window //2014-04-09
//[AE weight update 1]
/* LGE CHANGE S, add featuring E9, 2014-05-03, js.kim@lge.com */
#if defined(CONFIG_MACH_MSM8226_E9WIFI) || defined(CONFIG_MACH_MSM8226_E9WIFIN) || defined(CONFIG_MACH_MSM8926_E9LTE_VZW_US)
	{0x098E, 0x2404,},       // LOGICAL_ADDRESS_ACCESS [AE_RULE_ALGO]
	{0xA404, 0x0001,},      // AE_RULE_ALGO
	{0x098E, 0xA407,},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_0_0]
	{0x0990, 0x0400,},
	{0x098E, 0xA408,},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_0_1]
	{0x0990, 0x0400,},
	{0x098E, 0xA409,},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_0_2]
	{0x0990, 0x0400,},
	{0x098E, 0xA40A,},    // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_0_3]
	{0x0990, 0x0400,},
	{0x098E, 0xA40B,},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_0_4]
	{0x0990, 0x0400,},
	{0x098E, 0xA40C,},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_1_0]
	{0x0990, 0x0400,},
	{0x098E, 0xA40D,},    // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_1_1]
	{0x0990, 0x0F00,},//0x3200},       //0x4B00
	{0x098E, 0xA40E,},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_1_2]
	{0x0990, 0x4B00,},//0x3200},       //0x4B00
	{0x098E, 0xA40F,},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_1_3]
	{0x0990, 0x0F00,},//0x3200},       //0x4B00
	{0x098E, 0xA410,},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_1_4]
	{0x0990, 0x0400,},
	{0x098E, 0xA411,},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_2_0]
	{0x0990, 0x0400,},
	{0x098E, 0xA412,},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_2_1]
	{0x0990, 0x0F00,},//0x3200},       //0x4B00
	{0x098E, 0xA413,},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_2_2]
	{0x0990, 0x6300,},//0x4B00},     //0x6400
	{0x098E, 0xA414,},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_2_3]
	{0x0990, 0x0F00,},//0x3200},       //0x4B00
	{0x098E, 0xA415,},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_2_4]
	{0x0990, 0x0400,},
	{0x098E, 0xA416,},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_3_0]
	{0x0990, 0x0400,},
	{0x098E, 0xA417,},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_3_1]
	{0x0990, 0x0F00,},//0x3200},       //0x4B00
	{0x098E, 0xA418,},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_3_2]
	{0x0990, 0x4B00,},//0x3200},       //0x4B00
	{0x098E, 0xA419,},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_3_3]
	{0x0990, 0x0F00,},//0x3200},       //0x4B00
	{0x098E, 0xA41A,},    // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_3_4]
	{0x0990, 0x0400,},
	{0x098E, 0xA41B,},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_4_0]
	{0x0990, 0x0400,},
	{0x098E, 0xA41C,},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_4_1]
	{0x0990, 0x0400,},
	{0x098E, 0xA41D,},    // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_4_2]
	{0x0990, 0x0400,},
	{0x098E, 0xA41E,},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_4_3]
	{0x0990, 0x0400,},
	{0x098E, 0xA41F,},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_4_4]
	{0x0990, 0x0400,},
#else
	{0x098E, 0x2404},       // LOGICAL_ADDRESS_ACCESS [AE_RULE_ALGO]
	{0xA404, 0x0001},      // AE_RULE_ALGO
	{0x098E, 0xA407},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_0_0]
	{0x0990, 0x0000},
	{0x098E, 0xA408},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_0_1]
	{0x0990, 0x0000},
	{0x098E, 0xA409},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_0_2]
	{0x0990, 0x0000},
	{0x098E, 0xA40A},    // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_0_3]
	{0x0990, 0x0000},
	{0x098E, 0xA40B},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_0_4]
	{0x0990, 0x0000},
	{0x098E, 0xA40C},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_1_0]
	{0x0990, 0x0000},
	{0x098E, 0xA40D},    // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_1_1]
	{0x0990, 0x3200},       //0x4B00
	{0x098E, 0xA40E},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_1_2]
	{0x0990, 0x3200},       //0x4B00
	{0x098E, 0xA40F},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_1_3]
	{0x0990, 0x3200},       //0x4B00
	{0x098E, 0xA410},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_1_4]
	{0x0990, 0x0000},
	{0x098E, 0xA411},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_2_0]
	{0x0990, 0x0000},
	{0x098E, 0xA412},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_2_1]
	{0x0990, 0x3200},       //0x4B00
	{0x098E, 0xA413},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_2_2]
	{0x0990, 0x4B00},     //0x6400
	{0x098E, 0xA414},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_2_3]
	{0x0990, 0x3200},       //0x4B00
	{0x098E, 0xA415},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_2_4]
	{0x0990, 0x0000},
	{0x098E, 0xA416},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_3_0]
	{0x0990, 0x0000},
	{0x098E, 0xA417},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_3_1]
	{0x0990, 0x3200},       //0x4B00
	{0x098E, 0xA418},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_3_2]
	{0x0990, 0x3200},       //0x4B00
	{0x098E, 0xA419},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_3_3]
	{0x0990, 0x3200},       //0x4B00
	{0x098E, 0xA41A},    // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_3_4]
	{0x0990, 0x0000},
	{0x098E, 0xA41B},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_4_0]
	{0x0990, 0x0000},
	{0x098E, 0xA41C},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_4_1]
	{0x0990, 0x0000},
	{0x098E, 0xA41D},    // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_4_2]
	{0x0990, 0x0000},
	{0x098E, 0xA41E},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_4_3]
	{0x0990, 0x0000},
	{0x098E, 0xA41F},     // LOGICAL_ADDRESS_ACCESS [AE_RULE_AE_WEIGHT_TABLE_4_4]
	{0x0990, 0x0000},
	#endif
	/* LGE CHANGE E, add featuring E9, 2014-05-03, js.kim@lge.com */
// end of test for AE window //2014-04-09
	{0x098E, 0xC87A,},	//			===== 8
	{0x0990, 0x3000,}, 	// CAM_AET_TARGET_AVERAGE_LUMA
	{0x098E, 0xBC07,},		//			===== 8
	{0x0990, 0x0000,}, 	// LL_GAMMA_SELECT
	//[Gamma]
	{0x098E, 0xBC0A,},	//			===== 8
	{0x0990, 0x0000,},	// LL_GAMMA_CONTRAST_CURVE_0
	{0x098E, 0xBC0B,},	//			===== 8
	{0x0990, 0x0A00,},	// LL_GAMMA_CONTRAST_CURVE_1
	{0x098E, 0xBC0C,},	//			===== 8
	{0x0990, 0x1D00,},	// LL_GAMMA_CONTRAST_CURVE_2
	{0x098E, 0xBC0D,},	//			===== 8
	{0x0990, 0x3700,},	// LL_GAMMA_CONTRAST_CURVE_3
	{0x098E, 0xBC0E,},	//			===== 8
	{0x0990, 0x5800,},	// LL_GAMMA_CONTRAST_CURVE_4
	{0x098E, 0xBC0F,},	//			===== 8
	{0x0990, 0x7100,},	// LL_GAMMA_CONTRAST_CURVE_5
	{0x098E, 0xBC10,},	//			===== 8
	{0x0990, 0x8600,},	// LL_GAMMA_CONTRAST_CURVE_6
	{0x098E, 0xBC11,},	//			===== 8
	{0x0990, 0x9800,},	// LL_GAMMA_CONTRAST_CURVE_7
	{0x098E, 0xBC12,},	//			===== 8
	{0x0990, 0xA700,},	// LL_GAMMA_CONTRAST_CURVE_8
	{0x098E, 0xBC13,},	//			===== 8
	{0x0990, 0xAA00,},	// LL_GAMMA_CONTRAST_CURVE_9
	{0x098E, 0xBC14,},	//			===== 8
	{0x0990, 0xB400,},	// LL_GAMMA_CONTRAST_CURVE_10
	{0x098E, 0xBC15,},	//			===== 8
	{0x0990, 0xB900,},	// LL_GAMMA_CONTRAST_CURVE_11
	{0x098E, 0xBC16,},	//			===== 8
	{0x0990, 0xBE00,},	// LL_GAMMA_CONTRAST_CURVE_12
	{0x098E, 0xBC17,},	//			===== 8
	{0x0990, 0xC300,},	// LL_GAMMA_CONTRAST_CURVE_13
	{0x098E, 0xBC18,},	//			===== 8
	{0x0990, 0xC800,},	// LL_GAMMA_CONTRAST_CURVE_14
	{0x098E, 0xBC19,},	//			===== 8
	{0x0990, 0xCD00,},	// LL_GAMMA_CONTRAST_CURVE_15
	{0x098E, 0xBC1A,},	//			===== 8
	{0x0990, 0xD200,},	// LL_GAMMA_CONTRAST_CURVE_16
	{0x098E, 0xBC1B,},	//			===== 8
	{0x0990, 0xD700,},	// LL_GAMMA_CONTRAST_CURVE_17
	{0x098E, 0xBC1C,},	//			===== 8
	{0x0990, 0xDC00,},	// LL_GAMMA_CONTRAST_CURVE_18
	{0x098E, 0xBC1F,},	//			===== 8
	{0x0990, 0x1D00,}, 	// LL_GAMMA_NRCURVE_2
	{0x098E, 0xBC20,},	//			===== 8
	{0x0990, 0x3700,}, 	// LL_GAMMA_NRCURVE_3
	{0x098E, 0xBC21,},	//			===== 8
	{0x0990, 0x5800,}, 	// LL_GAMMA_NRCURVE_4
	{0x098E, 0xBC22,},	//			===== 8
	{0x0990, 0x7100,}, 	// LL_GAMMA_NRCURVE_5

	//[Step5-AWB_CCM]
	{0xC892, 0x020F,},
	{0xC894, 0xFF84,},
	{0xC896, 0xFFB7,},
	{0xC898, 0xFF3A,},
	{0xC89A, 0x0209,},
	{0xC89C, 0x0004,},
	{0xC89E, 0xFF8C,},
	{0xC8A0, 0xFF16,},
	{0xC8A2, 0x028A,},
	{0xC8A4, 0x01C5,},
	{0xC8A6, 0xFFA8,},
	{0xC8A8, 0xFFCE,},
	{0xC8AA, 0xFFAA,},
	{0xC8AC, 0x01B3,},
	{0xC8AE, 0xFFE6,},
	{0xC8B0, 0xFFD7,},
	{0xC8B2, 0xFEF5,},
	{0xC8B4, 0x0260,},
	{0xC8B6, 0x0211,},
	{0xC8B8, 0xFF59,},
	{0xC8BA, 0xFFC2,},
	{0xC8BC, 0xFF84,},
	{0xC8BE, 0x01CD,},
	{0xC8C0, 0xFFE8,},
	{0xC8C2, 0xFFD1,},
	{0xC8C4, 0xFF82,},
	{0xC8C6, 0x01DD,},
	{0xC8C8, 0x0075,},		// CAM_AWB_CCM_L_RG_GAIN
	{0xC8CA, 0x011C,},		// CAM_AWB_CCM_L_BG_GAIN
	{0xC8CC, 0x009A,},		// CAM_AWB_CCM_M_RG_GAIN
	{0xC8CE, 0x0105,},		// CAM_AWB_CCM_M_BG_GAIN
	{0xC8D0, 0x00A4,}, 		// CAM_AWB_CCM_R_RG_GAIN
	{0xC8D2, 0x00AC,},		// CAM_AWB_CCM_R_BG_GAIN
	{0xC8D4, 0x0A8C,},		// CAM_AWB_CCM_L_CTEMP
	{0xC8D6, 0x0F0A,},		// CAM_AWB_CCM_M_CTEMP
	{0xC8D8, 0x1964,},		// CAM_AWB_CCM_R_CTEMP
	{0xC914, 0x0000,},		// CAM_STAT_AWB_CLIP_WINDOW_XSTART
	{0xC916, 0x0000,},		// CAM_STAT_AWB_CLIP_WINDOW_YSTART
	{0xC918, 0x04FF,},		// CAM_STAT_AWB_CLIP_WINDOW_XEND
	{0xC91A, 0x03BF,},		// CAM_STAT_AWB_CLIP_WINDOW_YEND
    {0xC904, 0x002f,},      // for osram , 2014-04-10
	{0xC906, 0x0036,},      // CAM_AWB_AWB_YSHIFT_PRE_ADJ //2014-04-09
	{0x098E, 0xC8F2,},		// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x0400,},		// CAM_AWB_AWB_XSCALE
	{0x098E, 0xC8F3,},		// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x0200,},		// CAM_AWB_AWB_XSCALE
	{0xC8F4, 0x0000,},		// CAM_AWB_AWB_WEIGHTS_0
	{0xC8F6, 0x0000,},		// CAM_AWB_AWB_WEIGHTS_1
	{0xC8F8, 0x0000,},		// CAM_AWB_AWB_WEIGHTS_2
	{0xC8FA, 0xE724,},		// CAM_AWB_AWB_WEIGHTS_3
	{0xC8FC, 0x1583,},		// CAM_AWB_AWB_WEIGHTS_4
	{0xC8FE, 0x2045,},		// CAM_AWB_AWB_WEIGHTS_5
	{0xC900, 0x061C,},			// 0x05DC,		// CAM_AWB_AWB_WEIGHTS_6
	{0xC902, 0x007C,},		// CAM_AWB_AWB_WEIGHTS_7
	{0x098E, 0xC90C,},		// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x8000,},		// CAM_AWB_K_R_L //2014-04-09
	{0x098E, 0xC90D,},		// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x8000,},		// CAM_AWB_K_G_L //2014-04-09
	{0x098E, 0xC90E,},		// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x8000,},		// CAM_AWB_K_B_L
	{0x098E, 0xC90F,},		// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x7C00,},		// CAM_AWB_K_R_R //2014-04-29
	{0x098E, 0xC910,},		// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x8000,},		// CAM_AWB_K_G_R //2014-04-09
	{0x098E, 0xC911,},		// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x8000,},		// CAM_AWB_K_B_R //2014-04-09
	{0x098E, 0xC912,},		// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x0000,},		// CAM_STAT_LUMA_THRESH_LOW
	{0x098E, 0xAC0C,},         	// LOGICAL_ADDRESS_ACCESS [AWB_B_SCENE_RATIO_LOWER]
	{0x0990, 0x2F00,}, 		//0x3500,               // AWB_B_SCENE_RATIO_LOWER
	{0x098E, 0xAC16,},		// AWB_PRE_AWB_RATIOS_TRACKING_SPEED   // 2011.07.04
	{0x0990, 0x2000,},
	{0x098E, 0xAC17,},		// AWB_STATISTICS_TRACKING_SPEED  // 2011.07.04
	{0x0990, 0x2000,},
	{0x210A, 0x0010,},//js_test_Outdoor AWB hunting_0503
	//[Step7-CPIPE_Preference]
	{0x098E, 0x3C02,}, 	// LOGICAL_ADDRESS_ACCESS [LL_MODE]
	{0xBC02, 0x000D,}, 	// LL_MODE
	{0xC926, 0x0020,},	// CAM_LL_START_BRIGHTNESS
	{0xC928, 0x009A,},	// CAM_LL_STOP_BRIGHTNESS
	{0xC946, 0x0070,},	// CAM_LL_START_GAIN_METRIC
	{0xC948, 0x00F3,},	// CAM_LL_STOP_GAIN_METRIC
	{0xC952, 0x0020,},	// CAM_LL_START_TARGET_LUMA_BM
	{0xC954, 0x009A,},	// CAM_LL_STOP_TARGET_LUMA_BM
	{0x098E, 0xC92A,},	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x6800,},	// CAM_LL_START_SATURATION	// 2014-04-29
	{0x098E, 0xC92B,},	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x6000,},	// CAM_LL_END_SATURATION		// 2011.07.04
	{0x098E, 0xC92C,},	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x0000,},	// CAM_LL_START_DESATURATION
	{0x098E, 0xC92D,},	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x9F00,},	// CAM_LL_END_DESATURATION
	{0x098E, 0xC92E,},	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x6400,},	//0x0990, 0x7000,	// CAM_LL_START_DEMOSAIC		// 2011.07.04	// SKT IOT 2nd (2012-01-07) : Change Sharpness
	{0x098E, 0xC95B,},	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x0000,},	// CAM_SEQ_DARK_COLOR_KILL
	{0x098E, 0xC95C,},	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x6200,},	// CAM_SEQ_BRIGHT_COLOR_KILL
	{0x098E, 0x4948,},	// LOGICAL_ADDRESS_ACCESS
	{0xC948, 0x01A0,},	// CAM_LL_STOP_GAIN_METRIC
	{0x098E, 0xC92F,},	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x0500,},	//0x0990, 0x0500,	// CAM_LL_START_AP_GAIN							// SKT IOT 2nd (2012-01-07) : Change Sharpness
	{0x098E, 0xC930,},	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x0300,},	// CAM_LL_START_AP_THRESH   //for noise, 2014-04-10
	{0x098E, 0xC931,},	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x6400,},	// CAM_LL_STOP_DEMOSAIC
	{0x098E, 0xC932,},	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x0100,},	// CAM_LL_STOP_AP_GAIN		// 2011.07.04
	{0x098E, 0xC933,},	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x0C00,},	// CAM_LL_STOP_AP_THRESH
	{0x098E, 0xC934,},	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x3200,},	//0x0990, 0x3600,	// CAM_LL_START_NR_RED							// SKT IOT 2nd (2012-01-07) : Change Sharpness
	{0x098E, 0xC935,},	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x3200,},	//0x0990, 0x1800,	// CAM_LL_START_NR_GREEN							// SKT IOT 2nd (2012-01-07) : Change Sharpness
	{0x098E, 0xC936,},	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x3200,},	//0x0990, 0x3600,	// CAM_LL_START_NR_BLUE							// SKT IOT 2nd (2012-01-07) : Change Sharpness
	{0x098E, 0xC937,},	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x3200,},	//0x0990, 0x1800,	// CAM_LL_START_NR_THRESH						// SKT IOT 2nd (2012-01-07) : Change Sharpness
	{0x098E, 0xC938,},	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x5000,},	// CAM_LL_STOP_NR_RED
	{0x098E, 0xC939,},	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x5000,},	// CAM_LL_STOP_NR_GREEN
	{0x098E, 0xC93A,},	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x5000,},	// CAM_LL_STOP_NR_BLUE
	{0x098E, 0xC93B,},	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x5000,},	// CAM_LL_STOP_NR_THRESH
	{0xC93C, 0x0020,},	// CAM_LL_START_CONTRAST_BM
	{0xC93E, 0x009A,},	// CAM_LL_STOP_CONTRAST_BM
	{0xC940, 0x00Df,},	// CAM_LL_GAMMA
	{0x098E, 0xC942,},	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x4000,},	// CAM_LL_START_CONTRAST_GRADIENT
	{0x098E, 0xC943,}, 	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x3800,},	// CAM_LL_STOP_CONTRAST_GRADIENT
	{0x098E, 0xC944,}, 	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x6400,},	// CAM_LL_START_CONTRAST_LUMA_PERCENTAGE
	{0x098E, 0xC945,}, 	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x3F00,},	// CAM_LL_STOP_CONTRAST_LUMA_PERCENTAGE
	{0xC94A, 0x0230,}, 	// CAM_LL_START_FADE_TO_BLACK_LUMA
	{0xC94C, 0x0010,}, 	// CAM_LL_STOP_FADE_TO_BLACK_LUMA
	{0xC94E, 0x000E,},	//0xC94E, 0x01CD, 	// CAM_LL_CLUSTER_DC_TH_BM					// SKT IOT 2nd (2012-01-07) : Change Sharpness
	{0x098E, 0xC950,}, 	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x0500,},	// CAM_LL_CLUSTER_DC_GATE_PERCENTAGE
	{0x098E, 0xC951,}, 	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x4000,},	// CAM_LL_SUMMING_SENSITIVITY_FACTOR
	{0x098E, 0xC87B,}, 	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x1B00,},	// CAM_AET_TARGET_AVERAGE_LUMA_DARK
	{0x098E, 0xC878,}, 	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x0000,},	// CAM_AET_AEMODE = 0
	{0xC890, 0x0080,}, 	// CAM_AET_TARGET_GAIN
	{0xC882, 0x0084,}, 	//0x00E0 	//0x00AF//Dgain
	{0xC886, 0x01A8,}, 	// CAM_AET_AE_MAX_VIRT_AGAIN
	{0xC87C, 0x000a,},	//	5A 	// CAM_AET_BLACK_CLIPPING_TARGET
	{0x098E, 0xB00C,}, 	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x2500,},	// BLACKLEVEL_MAX_BLACK_LEVEL
	{0x098E, 0xB00D,}, 	// LOGICAL_ADDRESS_ACCESS [BLACKLEVEL_BLACK_LEVEL_DAMPENING]			===== 8
	{0x0990, 0x1000,}, 	// BLACKLEVEL_BLACK_LEVEL_DAMPENING
	{0x098E, 0xB42A,}, 	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x0500,},	// CCM_DELTA_GAIN
	{0x098E, 0xA80A,}, 	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x2000,},	// AE_TRACK_AE_TRACKING_DAMPENING_SPEED
	//[Step8-Features]

	{0x001E, 0x0777,}, 	// PAD_SLEW


	// MIPI non-continuous clock mode
	{0x3C40, 0x7830},  	// (enable non-continuous mode)
	//MIPI Timing
	{0xC984, 0x8001,},	//cam_port_output_control = 32833 // {0xC984, 0x8001}, for non-continuous mode
	{0xC988, 0x0F00,},	//cam_port_mipi_timing_t_hs_zero = 3840
	{0xC98A, 0x0B07,},	//cam_port_mipi_timing_t_hs_exit_hs_trail = 2823
	{0xC98C, 0x0D01,},	//cam_port_mipi_timing_t_clk_post_clk_pre = 3329
	{0xC98E, 0x071D,},	//cam_port_mipi_timing_t_clk_trail_clk_zero = 1821
	{0xC990, 0x0006,},	//cam_port_mipi_timing_t_lpx = 6
	{0xC992, 0x0A0C,},	//cam_port_mipi_timing_init_timing = 2572

#if 0  //sujeong.kwon@lge.com. 2014-04-02. seperated to mt9m114_config_change_settings[]
	//[Change-Config]
	{0x098E, 0xDC00,}, 	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x2800,},	// AE_TRACK_AE_TRACKING_DAMPENING_SPEED
	{0x0080, 0x8002,}, 	// COMMAND_REGISTER

	/* POLLREG= 0x0080, 2, !=0, DELAY=1msec, TIMEOUT=100 */
	{0xFFFD, 0x0164},	 /* [POLLING CMD(16)], [TIME(8) | COUNT(8)] */
	{0x0080, 0x0002},	 /* [REG ADDR(16)], [MASK(16)] */
#endif
};

static struct msm_camera_i2c_reg_conf mt9m114_config_change_settings[] = {
	//[Change-Config]
	{0x098E, 0xDC00,}, 	// LOGICAL_ADDRESS_ACCESS			===== 8
	{0x0990, 0x2800,},	// AE_TRACK_AE_TRACKING_DAMPENING_SPEED
	{0x0080, 0x8002,}, 	// COMMAND_REGISTER

	/* POLLREG= 0x0080, 2, !=0, DELAY=1msec, TIMEOUT=100 */
	{0xFFFD, 0x0164,},	 /* [POLLING CMD(16)], [TIME(8) | COUNT(8)] */
	{0x0080, 0x0002,},	 /* [REG ADDR(16)], [MASK(16)] */
};

static struct msm_camera_i2c_reg_conf mt9m114_flicker_50hz_settings[] = {
	{0x098E, 0xC000,}, 	 // LOGICAL_ADDRESS_ACCESS [FLICKER_DETECT_MODE]
	{0x0990, 0x0000,},	 // FLICKER_DETECT_MODE
	{0x098E, 0xC88B,}, 	 // LOGICAL_ADDRESS_ACCESS [CAM_AET_FLICKER_FREQ_HZ]
	{0x0990, 0x3200,},	 // CAM_AET_FLICKER_FREQ_HZ
};

static struct msm_camera_i2c_reg_conf mt9m114_flicker_60hz_settings[] = {
	{0x098E, 0xC000,}, 	 // LOGICAL_ADDRESS_ACCESS [FLICKER_DETECT_MODE]
	{0x0990, 0x0000,}, 	 // FLICKER_DETECT_MODE
	{0x098E, 0xC88B,}, 	 // LOGICAL_ADDRESS_ACCESS [CAM_AET_FLICKER_FREQ_HZ]
	{0x0990, 0x3C00,}, 	 // CAM_AET_FLICKER_FREQ_HZ
};



static struct v4l2_subdev_info mt9m114_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id mt9m114_i2c_id[] = {
	{MT9M114_SENSOR_NAME, (kernel_ulong_t)&mt9m114_s_ctrl},
	{ }
};

static int32_t msm_mt9m114_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &mt9m114_s_ctrl);
}

static struct i2c_driver mt9m114_i2c_driver = {
	.id_table = mt9m114_i2c_id,
	.probe  = msm_mt9m114_i2c_probe,
	.driver = {
		.name = MT9M114_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client mt9m114_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id mt9m114_dt_match[] = {
	{.compatible = "qcom,mt9m114", .data = &mt9m114_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, mt9m114_dt_match);

static struct platform_driver mt9m114_platform_driver = {
	.driver = {
		.name = "qcom,mt9m114",
		.owner = THIS_MODULE,
		.of_match_table = mt9m114_dt_match,
	},
};

static int32_t mt9m114_platform_probe(struct platform_device *pdev)
{
	int32_t rc;
	const struct of_device_id *match;
	printk("%s, E.\n", __func__);
	match = of_match_device(mt9m114_dt_match, &pdev->dev);
/* LGE_CHANGE_S : WBT, 2013-5-31, jonghwan.ko@lge.com */
		if(!match)
		{
			  pr_err(" %s failed\n",__func__);
			  return -ENODEV;
		 }
/* LGE_CHANGE_E : WBT, 2013-5-31, jonghwan.ko@lge.com */
	rc = msm_sensor_platform_probe(pdev, match->data);
	pr_err("mt9m114: mt9m114_platform_probe \n");

	if(rc < 0){
		pr_err("%s: %d failed\n",__func__,__LINE__);
		return -EIO;
	}

	rc = mt9m114_sysfs_add_symlink(&pdev->dev);

	return rc;
}

static int __init mt9m114_init_module(void)
{
	int32_t rc;
	pr_err("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&mt9m114_platform_driver,
		mt9m114_platform_probe);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&mt9m114_i2c_driver);
}

static void __exit mt9m114_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (mt9m114_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&mt9m114_s_ctrl);
		platform_driver_unregister(&mt9m114_platform_driver);
	} else
		i2c_del_driver(&mt9m114_i2c_driver);
	return;
}

static int mt9m114_sensor_config_control(struct msm_camera_i2c_client *client,
	struct msm_camera_i2c_reg_conf *reg_conf_tbl, uint16_t size,
	enum msm_camera_i2c_data_type data_type)
{
	#define CMD_SLEEP     0xFFFE
	#define CMD_POLLING   0xFFFD
	#define CMD_SET_MASK  0xFFFC
	int i = 0;
	long rc = 0;

	pr_err("%s:%d enter  \n", __func__, __LINE__);

	for (i = 0; i < size; i++) {
		if (reg_conf_tbl[i].reg_addr == CMD_POLLING) {
			/* [CMD_POLLING(16)], [TIME(8)|COUNT(8)]
				 [REG(16)], [MASK(16)] */
			uint8_t  poll_time =
				((reg_conf_tbl[i].reg_data) & 0xFF00) >> 8;
			uint8_t  poll_max_count =
				((reg_conf_tbl[i].reg_data) & 0xFF);
			uint16_t reg_addr = reg_conf_tbl[i+1].reg_addr;
			uint16_t reg_mask = reg_conf_tbl[i+1].reg_data;
			uint16_t data = 0;
			uint16_t j;

			for (j = 0; j < poll_max_count; j++) {
				rc = client->i2c_func_tbl->i2c_read(client,
					reg_addr, &data, MSM_CAMERA_I2C_WORD_DATA);
				if (rc < 0) {
					pr_err("%s:%d failed\n", __func__, __LINE__);
					rc = -EFAULT;
					break;
				}

				if ((data&reg_mask) == 0) {
					pr_err("%s:%d [CMD_POLLING] success (cnt=%d) \n",
							__func__, __LINE__, j);
					break;
				}
				msleep(poll_time);
			}
			i++;
		} else if (reg_conf_tbl[i].reg_addr == CMD_SET_MASK) {
			/* [CMD_SET_MASK(16)], [value(16)]
				 [REG(16)], [MASK(16)] */
			uint8_t  value = reg_conf_tbl[i].reg_data;
			uint16_t reg_addr = reg_conf_tbl[i+1].reg_addr;
			uint16_t reg_mask = reg_conf_tbl[i+1].reg_data;
			uint16_t data = 0;

			rc = client->i2c_func_tbl->i2c_read(client,
					reg_addr, &data, MSM_CAMERA_I2C_WORD_DATA);
			if (rc < 0) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}

			if (value)
				data |= reg_mask;
			else
				data &= (~reg_mask);

			rc = client->i2c_func_tbl->i2c_write(
					client,reg_addr, data, MSM_CAMERA_I2C_WORD_DATA);
			if (rc < 0) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
			pr_err("%s:%d [CMD_SET_MASK] success \n",__func__, __LINE__);

			i++;
		} else if (reg_conf_tbl[i].reg_addr == CMD_SLEEP) {
			/* [CMD_SLEEP(16)], [TIME(16)] */
			uint16_t time = reg_conf_tbl[i].reg_data;
			pr_err("%s:%d -- delay = 0x%x  \n", __func__, __LINE__,time);
			msleep(time);
		} else {
			rc = client->i2c_func_tbl->i2c_write(client,
					reg_conf_tbl[i].reg_addr,
					reg_conf_tbl[i].reg_data, data_type);
			if (rc < 0) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
		}
	}
	return rc;
}

int32_t mt9m114_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
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
		pr_err("%s:%d CFG_GET_SENSOR_INFO\n", __func__, __LINE__);
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
		pr_err("%s:%d CFG_SET_INIT_SETTING\n", __func__, __LINE__);
		/* 1. Write Recommend settings */
		/* 2. Set Manual Flicker */
		/* 3. Write change settings */
		rc = mt9m114_sensor_config_control(
			s_ctrl->sensor_i2c_client, mt9m114_recommend_settings,
			ARRAY_SIZE(mt9m114_recommend_settings), MSM_CAMERA_I2C_WORD_DATA);

		if(mt9m114_antibanding == MT9M114_50HZ){
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
				mt9m114_flicker_50hz_settings,ARRAY_SIZE(mt9m114_flicker_50hz_settings), MSM_CAMERA_I2C_WORD_DATA);
			CDBG("%s:%d CFG_SET_INIT_SETTING - 50hz\n", __func__, __LINE__);
		}else{
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(s_ctrl->sensor_i2c_client,
				mt9m114_flicker_60hz_settings, ARRAY_SIZE(mt9m114_flicker_60hz_settings), MSM_CAMERA_I2C_WORD_DATA);
			CDBG("%s:%d CFG_SET_INIT_SETTING - 60hz\n", __func__, __LINE__);
		}

		rc = mt9m114_sensor_config_control(
			s_ctrl->sensor_i2c_client, mt9m114_config_change_settings,
			ARRAY_SIZE(mt9m114_config_change_settings), MSM_CAMERA_I2C_WORD_DATA);

		pr_err("%s:%d CFG_SET_INIT_SETTING - done\n", __func__, __LINE__);
		break;
	case CFG_SET_RESOLUTION:
	case CFG_SET_STOP_STREAM:
		break;
	case CFG_SET_START_STREAM:
		break;
	case CFG_GET_SENSOR_INIT_PARAMS:
		cdata->cfg.sensor_init_params.modes_supported =
			s_ctrl->sensordata->sensor_info->modes_supported;
		cdata->cfg.sensor_init_params.position =
			s_ctrl->sensordata->sensor_info->position;
		cdata->cfg.sensor_init_params.sensor_mount_angle =
			s_ctrl->sensordata->sensor_info->sensor_mount_angle;
		CDBG("%s:%d init params mode %d pos %d mount %d\n", __func__,
			__LINE__,
			cdata->cfg.sensor_init_params.modes_supported,
			cdata->cfg.sensor_init_params.position,
			cdata->cfg.sensor_init_params.sensor_mount_angle);
		break;
	case CFG_SET_SLAVE_INFO: {
		struct msm_camera_sensor_slave_info sensor_slave_info;
		struct msm_camera_power_ctrl_t *p_ctrl;
		uint16_t size;
		int slave_index = 0;
		pr_err("%s:%d CFG_SET_SLAVE_INFO\n", __func__, __LINE__);
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
		p_ctrl = &s_ctrl->sensordata->power_info;
		size = sensor_slave_info.power_setting_array.size;
		if (p_ctrl->power_setting_size < size) {
			struct msm_sensor_power_setting *tmp;
			tmp = kmalloc(sizeof(struct msm_sensor_power_setting)
				      * size, GFP_KERNEL);
			if (!tmp) {
				pr_err("%s: failed to alloc mem\n", __func__);
				rc = -ENOMEM;
				break;
			}
			kfree(p_ctrl->power_setting);
			p_ctrl->power_setting = tmp;
		}
		p_ctrl->power_setting_size = size;

		rc = copy_from_user(p_ctrl->power_setting, (void *)
			sensor_slave_info.power_setting_array.power_setting,
			size * sizeof(struct msm_sensor_power_setting));
		if (rc) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s sensor id %x\n", __func__,
			sensor_slave_info.slave_addr);
		CDBG("%s sensor addr type %d\n", __func__,
			sensor_slave_info.addr_type);
		CDBG("%s sensor reg %x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id_reg_addr);
		CDBG("%s sensor id %x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id);
		for (slave_index = 0; slave_index <
			p_ctrl->power_setting_size; slave_index++) {
			CDBG("%s i %d power setting %d %d %ld %d\n", __func__,
				slave_index,
				p_ctrl->power_setting[slave_index].seq_type,
				p_ctrl->power_setting[slave_index].seq_val,
				p_ctrl->power_setting[slave_index].config_val,
				p_ctrl->power_setting[slave_index].delay);
		}
		break;
	}
	case CFG_WRITE_I2C_ARRAY: {
		struct msm_camera_i2c_reg_setting conf_array;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;

		pr_err("%s:%d CFG_WRITE_I2C_ARRAY\n", __func__, __LINE__);

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
		pr_err("%s:%d CFG_WRITE_I2C_SEQ_ARRAY\n", __func__, __LINE__);

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

	case CFG_POWER_UP:
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

	case CFG_POWER_DOWN:
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
/*LGE_CHANGE_S, add soc exif, 2013-10-04, kwangsik83.kim@lge.com*/
	case CFG_PAGE_MODE_READ_I2C_ARRAY:{
		int16_t size=0;
		uint16_t read_data_size = 0;
		uint16_t *read_data;
		uint16_t *read_data_head;
		struct msm_camera_i2c_reg_setting conf_array;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;

		pr_err("[WX] %s CFG_PAGE_MODE_READ_I2C_ARRAY\n", __func__);
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
/*LGE_CHANGE_E, add soc exif, 2013-10-04, kwangsik83.kim@lge.com*/

/*LGE_CHANGE_E, modified power-up/down status for recovery, 2013-12-27, hyungtae.lee@lge.com*/
	case CFG_SET_STOP_STREAM_SETTING: {
		struct msm_camera_i2c_reg_setting *stop_setting =
			&s_ctrl->stop_setting;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		pr_err("%s:%d CFG_SET_STOP_STREAM_SETTING\n", __func__, __LINE__);
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
			int32_t sat_lev;
			if (copy_from_user(&sat_lev, (void *)cdata->cfg.setting,
				sizeof(int32_t))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
		pr_debug("%s: Saturation Value is %d", __func__, sat_lev);
		break;
		}
		case CFG_SET_CONTRAST: {
			int32_t con_lev;
			if (copy_from_user(&con_lev, (void *)cdata->cfg.setting,
				sizeof(int32_t))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
		pr_debug("%s: Contrast Value is %d", __func__, con_lev);
		break;
		}
		case CFG_SET_SHARPNESS: {
			int32_t shp_lev;
			if (copy_from_user(&shp_lev, (void *)cdata->cfg.setting,
				sizeof(int32_t))) {
				pr_err("%s:%d failed\n", __func__, __LINE__);
				rc = -EFAULT;
				break;
			}
		pr_debug("%s: Sharpness Value is %d", __func__, shp_lev);
		break;
		}
		case CFG_SET_AUTOFOCUS: {
		/* TO-DO: set the Auto Focus */
		pr_debug("%s: Setting Auto Focus", __func__);
		break;
		}
		case CFG_CANCEL_AUTOFOCUS: {
		/* TO-DO: Cancel the Auto Focus */
		pr_debug("%s: Cancelling Auto Focus", __func__);
		break;
		}
//LGE_CHANGE_E,  These options has beend added due to colour effect issue. youngwook.song@lge.com 2013-11-25
	case CFG_SET_ISO:
	case CFG_SET_EXPOSURE_COMPENSATION:
	case CFG_SET_EFFECT:
	case CFG_SET_ANTIBANDING:
	case CFG_SET_BESTSHOT_MODE:
	case CFG_SET_WHITE_BALANCE:
	case CFG_SET_AEC_LOCK:
	case CFG_SET_AWB_LOCK:
	case CFG_SET_AEC_ROI:
		pr_debug("%s: We do not support features value related to LOCK now\n", __func__);
		break;
//LGE_CHANGE_X,  These options has beend added due to colour effect issue. youngwook.song@lge.com 2013-11-25
	case CFG_SET_FRAMERATE_FOR_SOC:
		break;
		default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}

static struct msm_sensor_fn_t mt9m114_sensor_func_tbl = {
	.sensor_config = mt9m114_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
};

static struct msm_sensor_ctrl_t mt9m114_s_ctrl = {
	.sensor_i2c_client = &mt9m114_sensor_i2c_client,
	.power_setting_array.power_setting = mt9m114_power_setting,
	.power_setting_array.size = ARRAY_SIZE(mt9m114_power_setting),
	.msm_sensor_mutex = &mt9m114_mut,
	.sensor_v4l2_subdev_info = mt9m114_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(mt9m114_subdev_info),
	.func_tbl = &mt9m114_sensor_func_tbl,
};

module_init(mt9m114_init_module);
module_exit(mt9m114_exit_module);
MODULE_DESCRIPTION("Aptina 1.26MP YUV sensor driver");
MODULE_LICENSE("GPL v2");
