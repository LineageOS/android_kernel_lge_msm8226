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
#include <mach/board_lge.h>		//to use lge_get_board_revno()
#define HI707_SENSOR_NAME "hi707"
#define PLATFORM_DRIVER_NAME "msm_camera_hi707"

#define CONFIG_MSMB_CAMERA_DEBUG
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif


#define SENSOR_REG_PAGE_ADDR 0x03
#define SENSOR_REG_PAGE_0 0x00
#define SENSOR_REG_PAGE_20 0x20

#define SENSOR_PREVIEW_WIDTH 640
#define SENSOR_PREVIEW_HEIGHT 480

#define AWB_LOCK_ON 1
#define AWB_LOCK_OFF 0
#define AEC_LOCK_ON 1
#define AEC_LOCK_OFF 0

#define AEC_ROI_DX (192) // (128)
#define AEC_ROI_DY (192) // (128) // (96)

static int PREV_SOC_AEC_LOCK = -1;	/*LGE_CHANGE, to prevent duplicated setting, 2013-01-07, kwangsik83.kim@lge.com*/
static int PREV_SOC_AWB_LOCK = -1;  /*LGE_CHANGE, to prevent duplicated setting, 2013-01-07, kwangsik83.kim@lge.com*/
static int mCurrentFpsMode = 4;  /* LGE_CHANGE, check current fps mode to avoid setting. , 2014-02-11, hyunuk.park@lge.com */

/* LGE_CHANGE_S, Fix for Dual Camera Module of HI707, 2014-02-28, dongsu.bag@lge.com */
typedef enum {
		HI707_LGIT,
		HI707_COWELL,
		HI707_MODULE_MAX,
}HI707ModuleType;

static int vt_cam_id_value = HI707_COWELL;
static int product_kor = 0;

/* LGE_CHANGE_E, Fix for Dual Camera Module of HI707, 2014-02-28, dongsu.bag@lge.com */

DEFINE_MSM_MUTEX(hi707_mut);
static struct msm_sensor_ctrl_t hi707_s_ctrl;

static struct msm_sensor_power_setting hi707_power_setting[] = {
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
	},
#if defined(CONFIG_MACH_MSM8X10_W5) || defined(CONFIG_MACH_MSM8X10_W6) || defined(CONFIG_MACH_MSM8X10_L70P)
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
		.delay = 1,
	},
#endif
#if defined(CONFIG_MACH_MSM8X10_W5C_SPR_US)
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 1,
	},
#else
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
#endif
#if defined(CONFIG_MACH_MSM8X10_W5) || defined(CONFIG_MACH_MSM8X10_W6) || defined(CONFIG_MACH_MSM8X10_L70P)
#elif defined(CONFIG_MACH_MSM8926_VFP_KR)
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
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 31,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 31,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

#if defined(CONFIG_MACH_MSM8926_E2_SPR_US)
static struct msm_sensor_power_setting hi707_power_setting_rev_b[] = {
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
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
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
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
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 31,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 31,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};
#endif

/*LGE_CHANGE_S, mipi end packet issue, 2013-10-15, kwangsik83.kim@lge.com*/
static struct msm_camera_i2c_reg_conf hi707_entrance_start_settings[] = {
	{0x03, 0x48},
	{0x16, 0x88},
	{0x03, 0x00},
	{0x01, 0x70},
	{0x09, 0x00},
	{0x03, 0x20},
	{0x18, 0x30},
};

static struct msm_camera_i2c_reg_conf hi707_start_settings[] = {
	{0x03, 0x48},
	{0x16, 0x80},
	{0x03, 0x00},
	{0x01, 0x70},
	{0x09, 0x00},
	{0x03, 0x20},
	{0x18, 0x30},
};
/*LGE_CHANGE_S, mipi end packet issue, 2013-10-15, kwangsik83.kim@lge.com*/


static struct msm_camera_i2c_reg_conf hi707_stop_settings[] = {
	{0x03, 0x00},
	{0x09, 0x01},
};
static struct msm_camera_i2c_reg_conf hi707_lgit_recommend_settings[] = {

	//2013.02.06 업데이트
	//V7 모델 셋팅 적용
	//Ae target 0x34 -> 3a
	//Video 30fps 모드 Max Exposure 1/40sec->1/30sec으로 변경함 -> 20130205
	//Video 모드 셋팅 : sleep on/off 추가 ->20130206
	//MIPI Error 개선 및 AE 오동작 개선
	//DV2차 실외 AWB 이슈 개선+Fixed 30fps video 추가
	//Manual 60Hz -> Auto flicker 변경
	//AG max 0xa0 -> 0xb0
	//BLC indoor 0x42 -> 0x44
	//최종 업데이트 : 20130315

		{0x03, 0x00},
		{0x01, 0x71},  // reset op.
		{0x01, 0x73},
		{0x01, 0x71},
		{0x03, 0x20},  //page 20
		{0x10, 0x1c},  //ae off
		{0x03, 0x22},  //page 22
		{0x10, 0x7b},  //awb off
		{0x03, 0x00},
		{0x08, 0x0f}, //Parallel NO Output_PAD Out
		{0x10, 0x00},	//VDOCTL1 [5:4]subsample:1,1/2,1/4, [0]preview_en
		{0x11, 0x90},	//VDOCTL2 , 90 : FFR off, 94 : FFR on
		{0x12, 0x00},	//CLK_CTL
		{0x14, 0x88},	//[7]fix_frm_spd:prevent hounting, [3]fix_frm_opt:inc. exp.time
		{0x0b, 0xaa},
		{0x0c, 0xaa},
		{0x0d, 0xaa},
		{0xc0, 0x95},
		{0xc1, 0x18},
		{0xc2, 0x91},
		{0xc3, 0x00},
		{0xc4, 0x01},

		{0x03, 0x00},
		{0x20, 0x00},	//WINROW
		{0x21, 0x04}, // - VGA:04, QVGA,QQVGA:02
		{0x22, 0x00},	//WINCOL
		{0x23, 0x04}, // - VGA,QVGA:04, QQVGA:02
		{0x40, 0x00},	//HBLANK
		{0x41, 0x90},	// - YUV422:0090, BAYER:0158
		{0x42, 0x00},	//VSYNCH
		{0x43, 0x04}, // - YUV422:0002, BAYER:0014

		{0x80, 0x2e}, //don't touch
		{0x81, 0x7e}, //don't touch
		{0x82, 0x90}, //don't touch
		{0x83, 0x30}, //don't touch
		{0x84, 0x2c}, //don't touch
		{0x85, 0x4b}, //don't touch
		{0x86, 0x01}, //don't touch
		{0x88, 0x47}, //don't touch
		{0x89, 0x48}, //BLC hold
		{0x90, 0x0c}, //BLC_TIME_TH_ON
		{0x91, 0x0c}, //BLC_TIME_TH_OFF
		{0x92, 0xa8}, //98}, //BLC_AG_TH_ON
		{0x93, 0xa0}, //90}, //BLC_AG_TH_OFF
		{0x98, 0x38},
		{0x99, 0x00}, //41}, //Out BLC LHC
		{0xa0, 0x02}, //00}, //Dark BLC
		{0xa8, 0x44}, //42}, //40}, //Normal BLC LHC

		{0xc0, 0x95},	//PLL Mode
		{0xc1, 0x18},
		{0xc2, 0x91},	//[4]plldiv_en, [3:2]mipi4xclkdiv:bypass,1/2,1/4,1/8, [0]ispclkdiv:1/2,1/4
		{0xc3, 0x00},
		{0xc4, 0x01},

		///////////////////////////// Page 2	-  Analog Circuit Control
		{0x03, 0x02},
		{0x10, 0x00},	//MODE_TEST
		{0x11, 0x00},	//MODE_DEAD_TEST
		{0x13, 0x40},	//MODE_ANA_TEST
		{0x14, 0x04},	//MODE_MEMORY
		{0x18, 0x1c},	//Analog mode
		{0x19, 0x00},	//[0]pmos_off
		{0x1a, 0x00},
		{0x1b, 0x08},
		{0x1c, 0x9c},	//DC-DC
		{0x1d, 0x03},
		{0x20, 0x33},	//PX bias
		{0x21, 0x77},	//ADC/ASP bias
		{0x22, 0xa7},	//Main bias
		{0x23, 0x32},	//Clamp
		{0x24, 0x33},
		{0x2b, 0x40},	//Fixed frame counter end
		{0x2d, 0x32},	//Fixed frame counter start
		{0x31, 0x99},	//shared control
		{0x32, 0x00},
		{0x33, 0x00},
		{0x34, 0x3c},
		{0x35, 0x0d},
		{0x3b, 0x80}, //SF 60
		{0x50, 0x21}, //timing control 1
		{0x51, 0x1C},
		{0x52, 0xaa},
		{0x53, 0x5a},
		{0x54, 0x30},
		{0x55, 0x10},
		{0x56, 0x0c},
		{0x58, 0x00},
		{0x59, 0x0F},
		{0x60, 0x34},	//addr_en - Exp. //Row Timing Control
		{0x61, 0x3a},
		{0x62, 0x34},	//rx1
		{0x63, 0x39},
		{0x64, 0x34},	//rx2
		{0x65, 0x39},
		{0x72, 0x35},	//tx1
		{0x73, 0x38},
		{0x74, 0x35},	//tx2
		{0x75, 0x38},
		{0x80, 0x02},	//addr_en - Read.
		{0x81, 0x2e},
		{0x82, 0x0d},	//rx1
		{0x83, 0x10},
		{0x84, 0x0d},	//rx2
		{0x85, 0x10},
		{0x92, 0x1d},	//tx1
		{0x93, 0x20},
		{0x94, 0x1d},	//tx2
		{0x95, 0x20},
		{0xa0, 0x03},	//sx
		{0xa1, 0x2d},
		{0xa4, 0x2d},	//sxb
		{0xa5, 0x03},
		{0xa8, 0x12},	//wrst
		{0xa9, 0x1b},
		{0xaa, 0x22},	//wsig
		{0xab, 0x2b},
		{0xac, 0x10},	//rx_off_rst
		{0xad, 0x0e},	//tx_off_rst
		{0xb8, 0x33},	//rx pwr - exp.
		{0xb9, 0x35},
		{0xbc, 0x0c},	//rx pwr - read
		{0xbd, 0x0e},
		{0xc0, 0x3a},	//addr_en1 - Fixed Exp.
		{0xc1, 0x3f},
		{0xc2, 0x3a},	//addr_en2
		{0xc3, 0x3f},
		{0xc4, 0x3a},	//sx1
		{0xc5, 0x3e},
		{0xc6, 0x3a},	//sx2
		{0xc7, 0x3e},
		{0xc8, 0x3a},	//rx1
		{0xc9, 0x3e},
		{0xca, 0x3a},	//rx2
		{0xcb, 0x3e},
		{0xcc, 0x3b},	//tx1
		{0xcd, 0x3d},
		{0xce, 0x3b},	//tx2
		{0xcf, 0x3d},
		{0xd0, 0x33},	//Exposure domain valid
		{0xd1, 0x3f},

		///////////////////////////// Page 10
		{0x03, 0x10}, //Page 10 - Format, Image Effect
		{0x10, 0x03}, //ISPCTL - [7:4]0:YUV322, 6:BAYER, [1:0]VYUY, UYVY, YVYU, YUYV
		{0x11, 0x43}, // - [0x1010:1011]YUV422:0343, BAYER:6000
		{0x12, 0x30}, //Y offet, dy offseet enable
		{0x40, 0x00},
		{0x41, 0x00}, //DYOFS  00->10-> 00	STEVE_130110(black scene face saturation)
		{0x48, 0x80}, //Contrast  88->84  _100318
		{0x50, 0xa0}, //e0}, //AGBRT
		{0x60, 0x0b},
		{0x61, 0x00}, //default
		{0x62, 0x78}, //SATB  (1.4x)
		{0x63, 0x78}, //SATR  (1.2x)
		{0x64, 0x80}, //a0}, //AGSAT 20130205
		{0x66, 0x90}, //wht_th2
		{0x67, 0x36}, //wht_gain  Dark (0.4x), Normal (0.75x)

		///////////////////////////// Page 11
		{0x03, 0x11},
		{0x10, 0x25}, //LPF_CTL1 //0x01
		{0x11, 0x07}, //1f},	//Test Setting
		{0x20, 0x00}, //LPF_AUTO_CTL
		{0x21, 0x60}, //38},	//LPF_PGA_TH
		{0x23, 0x0a}, //LPF_TIME_TH
		{0x60, 0x13}, //ZARA_SIGMA_TH //40->10
		{0x61, 0x85},
		{0x62, 0x00},	//ZARA_HLVL_CTL
		{0x63, 0x00}, //83},//ZARA_LLVL_CTL
		{0x64, 0x00}, //83},//ZARA_DY_CTL
		{0x67, 0x70}, //60},//70}, //F0},	//Dark
		{0x68, 0x24}, //24},//30},	//Middle
		{0x69, 0x04}, //10},//High

		///////////////////////////// Page 12
		{0x03, 0x12}, //Page 12 - 2D : YC1D,YC2D,DPC,Demosaic
		{0x40, 0xd3}, //d6}, //d7},//YC2D_LPF_CTL1 //bc
		{0x41, 0x09},	//YC2D_LPF_CTL2
		{0x50, 0x18}, //10}, //18}, //Test Setting
		{0x51, 0x24},	//Test Setting
		{0x70, 0x1f},	//GBGR_CTL1 //0x1f
		{0x71, 0x00},	//Test Setting
		{0x72, 0x00},	//Test Setting
		{0x73, 0x00},	//Test Setting
		{0x74, 0x12},	//GBGR_G_UNIT_TH//12
		{0x75, 0x12},	//GBGR_RB_UNIT_TH//12
		{0x76, 0x20},	//GBGR_EDGE_TH
		{0x77, 0x80},	//GBGR_HLVL_TH
		{0x78, 0x88},	//GBGR_HLVL_COMP
		{0x79, 0x18},	//Test Setting

		{0x90, 0x3d},
		{0x91, 0x34},
		{0x99, 0x28},
		{0x9c, 0x05}, //14 For defect
		{0x9d, 0x08}, //15 For defect
		{0x9e, 0x28},
		{0x9f, 0x28},

		{0xb0, 0x7d}, //75 White Defect
		{0xb5, 0x44},
		{0xb6, 0x82},
		{0xb7, 0x52},
		{0xb8, 0x44},
		{0xb9, 0x15},

		///////////////////////////// Page 13
		{0x03, 0x13}, //Page 13 - Sharpness
		{0x10, 0x01},
		{0x11, 0x8f}, //89}, //for resolution , 131119
		{0x12, 0x14},
		{0x13, 0x19},
		{0x14, 0x08},	//Test Setting
		{0x20, 0x05}, //03},	//SHARP_Negative //for resolution , 131119
		{0x21, 0x05}, //04}, //03},	//SHARP_Positive 131105
		{0x23, 0x25},	//SHARP_DY_CTL
		{0x24, 0x21},	//40->33
		{0x25, 0x08},	//SHARP_PGA_TH
		{0x26, 0x40},	//Test Setting
		{0x27, 0x00},	//Test Setting
		{0x28, 0x08},	//Test Setting
		{0x29, 0x50},	//AG_TH
		{0x2a, 0xe0},	//region ratio
		{0x2b, 0x10},	//Test Setting
		{0x2c, 0x28},	//Test Setting
		{0x2d, 0x40},	//Test Setting
		{0x2e, 0x00},	//Test Setting
		{0x2f, 0x00},	//Test Setting
		{0x30, 0x11},	//Test Setting
		{0x80, 0x05},	//SHARP2D_CTL
		{0x81, 0x07},	//Test Setting
		{0x90, 0x05}, //04},	//SHARP2D_SLOPE  //for resolution , 131119
		{0x91, 0x06}, //05},	//SHARP2D_DIFF_CTL //for resolution , 131119
		{0x92, 0x00},	//SHARP2D_HI_CLIP
		{0x93, 0x30},	//SHARP2D_DY_CTL
		{0x94, 0x30},	//Test Setting
		{0x95, 0x10},	//Test Setting

		///////////////////////////// Page 14
		{0x03, 0x14}, //Page 14 - Lens Shading Correction, 20140718 jinyoung.bae@lge.com
		{0x10, 0x01},
		{0x20, 0x80}, //60},   //XCEN LHC
		{0x21, 0x80}, //YCEN
		{0x22, 0x56}, //7d}, //_20131209 //88}, //7b}, //6a}, //50},
		{0x23, 0x40}, //5d}, //_20131209 //5c}, //50}, //44}, //40},
		{0x24, 0x38}, //48}, //_20131209 //49}, //44}, //32}, //3d},

		//////////////////////////// 15page
		{0x03, 0x15},
		{0x10, 0x03},
		{0x14, 0x52},	//CMCOFSGM
		{0x16, 0x3a},	//CMCOFSGL
		{0x17, 0x2f},	//CMC SIGN

		{0x30, 0xf1},
		{0x31, 0x71},
		{0x32, 0x00},
		{0x33, 0x1f},
		{0x34, 0xe1},
		{0x35, 0x42},
		{0x36, 0x01},
		{0x37, 0x31},
		{0x38, 0x72},

		{0x40, 0x90}, //CMC OFS
		{0x41, 0x82},
		{0x42, 0x12},
		{0x43, 0x86},
		{0x44, 0x92},
		{0x45, 0x18},
		{0x46, 0x84},
		{0x47, 0x02},
		{0x48, 0x02},
		//////////////////////////// 16page
		{0x03, 0x16}, //gamma, , 20140718 jinyoung.bae@lge.com
		{0x10, 0x01},
		{0x30, 0x00},
		{0x31, 0x0f},
		{0x32, 0x20},
		{0x33, 0x35},
		{0x34, 0x58},
		{0x35, 0x75},
		{0x36, 0x8e},
		{0x37, 0xa3},
		{0x38, 0xb4},
		{0x39, 0xc3},
		{0x3a, 0xcf},
		{0x3b, 0xe2},
		{0x3c, 0xf0},
		{0x3d, 0xf9},
		{0x3e, 0xff},
		//////////////////////////// 17page
		{0x03, 0x17},
		{0xc0, 0x01},
		{0xc4, 0x4b}, //3c},
		{0xc5, 0x3e}, //32},
		///////////////////////////// Page 20	- Auto Exposure
		{0x03, 0x20},
		{0x10, 0x0c},	//AECTL
		{0x11, 0x04},
		{0x18, 0x30}, //31}, // 130521, Flicker Test
		{0x20, 0x01},	//FrameCTL
		{0x28, 0x27},	//FineCTL
		{0x29, 0xa1},
		{0x2a, 0xf0},
		{0x2b, 0x34},
		{0x2c, 0x2b},	 // 130521, Flicker Test
		{0x39, 0x22},
		{0x3a, 0xde},
		{0x3b, 0x23},
		{0x3c, 0xde},

					 // 130521, Flicker Test
		{0x60, 0x71}, //70}, //0x71}, //AE weight, 20140718 jinyoung.bae@lge.com
		{0x61, 0x11}, //0x11},
		{0x62, 0x71}, //70}, //0x71},
		{0x63, 0x11}, //00}, //0x11},
		{0x68, 0x32}, //3c}, //30}, //AE_CEN
		{0x69, 0x6e}, //64}, //6a},
		{0x6A, 0x50}, //27}, //27},
		{0x6B, 0xa0}, //bb}, //bb},
#if defined(CONFIG_MACH_MSM8926_VFP_KR)
		{0x70, 0x38}, //36 //34, //lgit 20140328  //Y Targe 32
#else
		{0x70, 0x3a}, //38 //36 //34, //lgit 20140328  //Y Targe 32, 20140718 jinyoung.bae@lge.com
#endif
		{0x76, 0x88}, //22}, // Unlock bnd1
		{0x77, 0xfe}, //02}, // Unlock bnd2
		{0x78, 0x22}, //23 //22}, 20130524  //12}, // Yth 1
		{0x79, 0x26}, //Yth 2
		{0x7a, 0x23}, //Yth 3
		{0x7c, 0x1c}, //Yth 2
		{0x7d, 0x22}, //Yth 4

		{0x83, 0x01}, //EXP Normal 20.00 fps
		{0x84, 0x24},
		{0x85, 0xf8},
		{0x86, 0x00}, //EXPMin 7500.00 fps
		{0x87, 0xc8},
		{0x88, 0x02}, //EXP Max(120Hz) 10.00 fps
		{0x89, 0x49},
		{0x8a, 0xf0},
		{0xa0, 0x02}, //EXP Max(100Hz) 10.00 fps
		{0xa1, 0x49},
		{0xa2, 0xf0},
		{0x8B, 0x3a}, //EXP100
		{0x8C, 0x98},
		{0x8D, 0x30}, //EXP120
		{0x8E, 0xd4},
		{0x9c, 0x04}, //EXP Limit 1250.00 fps
		{0x9d, 0xc4}, //b0 //lgit 20140328
		{0x9e, 0x00}, //EXP Unit
		{0x9f, 0xc8},

		{0x98, 0x8c}, //Outdoor BRIGHT_MEASURE_TH
		{0x99, 0x23},

		{0xb0, 0x1d},
		{0xb1, 0x14}, //14
		{0xb2, 0xb0}, //a0}, //80
		{0xb3, 0x17}, //AGLVL //17
		{0xb4, 0x17},
		{0xb5, 0x3e},
		{0xb6, 0x2b},
		{0xb7, 0x24},
		{0xb8, 0x21},
		{0xb9, 0x1f},
		{0xba, 0x1e},
		{0xbb, 0x1d},
		{0xbc, 0x1c},
		{0xbd, 0x1b},

		{0xc0, 0x1a}, //PGA_sky
		{0xc3, 0x48}, //PGA_dark_on
		{0xc4, 0x48}, //PGA_dark_off

		{0x03, 0x22}, //Page 22 AWB
		{0x10, 0xe2},
		{0x11, 0x2E}, //26},
		{0x20, 0x41}, //01 //69
		{0x21, 0x40},
		{0x30, 0x80},
		{0x31, 0x80},
		{0x38, 0x12},
		{0x39, 0x33},
		{0x40, 0xf3}, //b8}, //93}, //f0},
					  // STEVE Yellowish
		{0x41, 0x54}, //0x44}, //0x54},
		{0x42, 0x33}, //0x22}, //0x33},
		{0x43, 0xf3}, //0xf8}, //0xf3},
		{0x44, 0x55}, //0x55}, //0x55},
		{0x45, 0x44}, //0x44}, //0x44},
		{0x46, 0x02}, //0x08}, //0x02},

		{0x80, 0x3b}, //_20131209 // //40}, // R
		{0x81, 0x20}, //20}, // G
		{0x82, 0x44}, //_20131209 // //38}, // B

		{0x83, 0x59}, //58}, //5a}, //52}, //RMAX
		{0x84, 0x20}, //1d}, //RMIN
		{0x85, 0x53}, //BMAX 5a
		{0x86, 0x24}, //BMIN

		{0x87, 0x49}, //48}, //4a}, //42
		{0x88, 0x3c},
		{0x89, 0x3e},
		{0x8a, 0x34},

		{0x8b, 0x00}, //02}, //0x08}, //02}, //OUT TH
		{0x8d, 0x24}, //3a}, //0x11}, //22},
		{0x8e, 0x61}, //b3}, //0x11}, //71},

		{0x8f, 0x63}, //65}, //0x63},
		{0x90, 0x60}, //61}, //0x60},
		{0x91, 0x5c}, //5C}, //0x5c},
		{0x92, 0x56}, //56}, //0x56},
		{0x93, 0x52}, //4E}, //0x52},
		{0x94, 0x4c}, //43}, //0x4c},
		{0x95, 0x3e}, //3A}, //0x3e},
		{0x96, 0x2f}, //32}, //0x2f},
		{0x97, 0x28}, //2A}, //0x28},
		{0x98, 0x23}, //24}, //0x23},
		{0x99, 0x21}, //20}, //0x21},
		{0x9a, 0x20}, //1C}, //0x20},
		{0x9b, 0x08}, //0a}, //0x07},

		/////////////////////////////
		// Page 48 MIPI /////////////
		/////////////////////////////
		{0x03, 0x48},
	  //{0x10, 0x05},			//05},
		{0x11, 0x0c}, //04}, //00}, con
		{0x16, 0x88}, //88}, //c8}, //c4},
		{0x1a, 0x00},				 //00},
		{0x1b, 0x35},				 //00},
		{0x1c, 0x03}, // HSP_LPX	 //02},
		{0x1d, 0x05}, // HSN_LPX	 //04},
		{0x1e, 0x07}, // HS_ZERO	 //07},
		{0x1f, 0x03}, // HS_TRAIL	 //05},
		{0x20, 0x00}, // HS_EXIT	 //00},
		{0x21, 0xb8}, // HS_SYNC
		{0x28, 0x00}, // ADD
		{0x30, 0x05},				 //05},
		{0x31, 0x00},				 //00},
		{0x32, 0x07}, // CLK_ZERO	 //06},
		{0x35, 0x01}, // CLK_TRAIL	 //03},
		{0x34, 0x01}, // CLK_PREPARE //02},
		{0x36, 0x01}, // CLKP_LPX	 //01},
		{0x37, 0x03}, // CLKN_LPX	 //03},
		{0x38, 0x02}, // CLK_EXIT	 //00},
		{0x39, 0xef},				 //4a},
		{0x3c, 0x00},				 //00},
		{0x3d, 0xfa},				 //fa},
		{0x3f, 0x10},				 //10},
		{0x40, 0x00},				 //00},
		{0x41, 0x20},				 //20},
		{0x42, 0x00},				 //00},
		{0x10, 0x05},				 //05},

		{0x03, 0x20},
		{0x10, 0xec}, //cc}, //120hz first 20130524
		{0x03, 0x22}, //Page 22 AWB
		{0x10, 0xfb},

};

static struct msm_camera_i2c_reg_conf hi707_cowell_recommend_settings[] = {

	//2013.02.06 업데이트
	//V7 모델 셋팅 적용
	//Ae target 0x34 -> 3a
	//Video 30fps 모드 Max Exposure 1/40sec->1/30sec으로 변경함 -> 20130205
	//Video 모드 셋팅 : sleep on/off 추가 ->20130206
	//MIPI Error 개선 및 AE 오동작 개선
	//DV2차 실외 AWB 이슈 개선+Fixed 30fps video 추가
	//Manual 60Hz -> Auto flicker 변경
	//AG max 0xa0 -> 0xb0
	//BLC indoor 0x42 -> 0x44
	//최종 업데이트 : 20130315

		{0x03, 0x00},
		{0x01, 0x71},  // reset op.
		{0x01, 0x73},
		{0x01, 0x71},
		{0x03, 0x20},  //page 20
		{0x10, 0x1c},  //ae off
		{0x03, 0x22},  //page 22
		{0x10, 0x7b},  //awb off
		{0x03, 0x00},
		{0x08, 0x0f}, //Parallel NO Output_PAD Out
		{0x10, 0x00},	//VDOCTL1 [5:4]subsample:1,1/2,1/4, [0]preview_en
		{0x11, 0x90},	//VDOCTL2 , 90 : FFR off, 94 : FFR on
		{0x12, 0x00},	//CLK_CTL
		{0x14, 0x88},	//[7]fix_frm_spd:prevent hounting, [3]fix_frm_opt:inc. exp.time
		{0x0b, 0xaa},
		{0x0c, 0xaa},
		{0x0d, 0xaa},
		{0xc0, 0x95},
		{0xc1, 0x18},
		{0xc2, 0x91},
		{0xc3, 0x00},
		{0xc4, 0x01},

		{0x03, 0x00},
		{0x20, 0x00},	//WINROW
		{0x21, 0x04}, // - VGA:04, QVGA,QQVGA:02
		{0x22, 0x00},	//WINCOL
		{0x23, 0x04}, // - VGA,QVGA:04, QQVGA:02
		{0x40, 0x00},	//HBLANK
		{0x41, 0x90},	// - YUV422:0090, BAYER:0158
		{0x42, 0x00},	//VSYNCH
		{0x43, 0x04}, // - YUV422:0002, BAYER:0014

		{0x80, 0x2e}, //don't touch
		{0x81, 0x7e}, //don't touch
		{0x82, 0x90}, //don't touch
		{0x83, 0x30}, //don't touch
		{0x84, 0x2c}, //don't touch
		{0x85, 0x4b}, //don't touch
		{0x86, 0x01}, //don't touch
		{0x88, 0x47}, //don't touch
		{0x89, 0x48}, //BLC hold
		{0x90, 0x0c}, //BLC_TIME_TH_ON
		{0x91, 0x0c}, //BLC_TIME_TH_OFF
		{0x92, 0xa8}, //98}, //BLC_AG_TH_ON
		{0x93, 0xa0}, //90}, //BLC_AG_TH_OFF
		{0x98, 0x38},
		{0x99, 0x00}, //41}, //Out BLC LHC
		{0xa0, 0x02}, //00}, //Dark BLC
		{0xa8, 0x44}, //42}, //40}, //Normal BLC LHC

		{0xc0, 0x95},	//PLL Mode
		{0xc1, 0x18},
		{0xc2, 0x91},	//[4]plldiv_en, [3:2]mipi4xclkdiv:bypass,1/2,1/4,1/8, [0]ispclkdiv:1/2,1/4
		{0xc3, 0x00},
		{0xc4, 0x01},

		///////////////////////////// Page 2	-  Analog Circuit Control
		{0x03, 0x02},
		{0x10, 0x00},	//MODE_TEST
		{0x11, 0x00},	//MODE_DEAD_TEST
		{0x13, 0x40},	//MODE_ANA_TEST
		{0x14, 0x04},	//MODE_MEMORY
		{0x18, 0x1c},	//Analog mode
		{0x19, 0x00},	//[0]pmos_off
		{0x1a, 0x00},
		{0x1b, 0x08},
		{0x1c, 0x9c},	//DC-DC
		{0x1d, 0x03},
		{0x20, 0x33},	//PX bias
		{0x21, 0x77},	//ADC/ASP bias
		{0x22, 0xa7},	//Main bias
		{0x23, 0x32},	//Clamp
		{0x24, 0x33},
		{0x2b, 0x40},	//Fixed frame counter end
		{0x2d, 0x32},	//Fixed frame counter start
		{0x31, 0x99},	//shared control
		{0x32, 0x00},
		{0x33, 0x00},
		{0x34, 0x3c},
		{0x35, 0x0d},
		{0x3b, 0x80}, //SF 60
		{0x50, 0x21}, //timing control 1
		{0x51, 0x1C},
		{0x52, 0xaa},
		{0x53, 0x5a},
		{0x54, 0x30},
		{0x55, 0x10},
		{0x56, 0x0c},
		{0x58, 0x00},
		{0x59, 0x0F},
		{0x60, 0x34},	//addr_en - Exp. //Row Timing Control
		{0x61, 0x3a},
		{0x62, 0x34},	//rx1
		{0x63, 0x39},
		{0x64, 0x34},	//rx2
		{0x65, 0x39},
		{0x72, 0x35},	//tx1
		{0x73, 0x38},
		{0x74, 0x35},	//tx2
		{0x75, 0x38},
		{0x80, 0x02},	//addr_en - Read.
		{0x81, 0x2e},
		{0x82, 0x0d},	//rx1
		{0x83, 0x10},
		{0x84, 0x0d},	//rx2
		{0x85, 0x10},
		{0x92, 0x1d},	//tx1
		{0x93, 0x20},
		{0x94, 0x1d},	//tx2
		{0x95, 0x20},
		{0xa0, 0x03},	//sx
		{0xa1, 0x2d},
		{0xa4, 0x2d},	//sxb
		{0xa5, 0x03},
		{0xa8, 0x12},	//wrst
		{0xa9, 0x1b},
		{0xaa, 0x22},	//wsig
		{0xab, 0x2b},
		{0xac, 0x10},	//rx_off_rst
		{0xad, 0x0e},	//tx_off_rst
		{0xb8, 0x33},	//rx pwr - exp.
		{0xb9, 0x35},
		{0xbc, 0x0c},	//rx pwr - read
		{0xbd, 0x0e},
		{0xc0, 0x3a},	//addr_en1 - Fixed Exp.
		{0xc1, 0x3f},
		{0xc2, 0x3a},	//addr_en2
		{0xc3, 0x3f},
		{0xc4, 0x3a},	//sx1
		{0xc5, 0x3e},
		{0xc6, 0x3a},	//sx2
		{0xc7, 0x3e},
		{0xc8, 0x3a},	//rx1
		{0xc9, 0x3e},
		{0xca, 0x3a},	//rx2
		{0xcb, 0x3e},
		{0xcc, 0x3b},	//tx1
		{0xcd, 0x3d},
		{0xce, 0x3b},	//tx2
		{0xcf, 0x3d},
		{0xd0, 0x33},	//Exposure domain valid
		{0xd1, 0x3f},

		///////////////////////////// Page 10
		{0x03, 0x10}, //Page 10 - Format, Image Effect
		{0x10, 0x03}, //ISPCTL - [7:4]0:YUV322, 6:BAYER, [1:0]VYUY, UYVY, YVYU, YUYV
		{0x11, 0x43}, // - [0x1010:1011]YUV422:0343, BAYER:6000
		{0x12, 0x30}, //Y offet, dy offseet enable
		{0x40, 0x00},
		{0x41, 0x00}, //DYOFS  00->10-> 00	STEVE_130110(black scene face saturation)
		{0x48, 0x80}, //Contrast  88->84  _100318
		{0x50, 0xa0}, //e0}, //AGBRT
		{0x60, 0x0b},
		{0x61, 0x00}, //default
		{0x62, 0x78}, //SATB  (1.4x)
		{0x63, 0x78}, //SATR  (1.2x)
		{0x64, 0x80}, //a0}, //AGSAT 20130205
		{0x66, 0x90}, //wht_th2
		{0x67, 0x36}, //wht_gain  Dark (0.4x), Normal (0.75x)

		///////////////////////////// Page 11
		{0x03, 0x11},
		{0x10, 0x25}, //LPF_CTL1 //0x01
		{0x11, 0x07}, //1f},	//Test Setting
		{0x20, 0x00}, //LPF_AUTO_CTL
		{0x21, 0x60}, //38},	//LPF_PGA_TH
		{0x23, 0x0a}, //LPF_TIME_TH
		{0x60, 0x13}, //ZARA_SIGMA_TH //40->10
		{0x61, 0x85},
		{0x62, 0x00},	//ZARA_HLVL_CTL
		{0x63, 0x00}, //83},//ZARA_LLVL_CTL
		{0x64, 0x00}, //83},//ZARA_DY_CTL
		{0x67, 0x70}, //60},//70}, //F0},	//Dark
		{0x68, 0x24}, //24},//30},	//Middle
		{0x69, 0x04}, //10},//High

		///////////////////////////// Page 12
		{0x03, 0x12}, //Page 12 - 2D : YC1D,YC2D,DPC,Demosaic
		{0x40, 0xd3}, //d6}, //d7},//YC2D_LPF_CTL1 //bc
		{0x41, 0x09},	//YC2D_LPF_CTL2
		{0x50, 0x18}, //10}, //18}, //Test Setting
		{0x51, 0x24},	//Test Setting
		{0x70, 0x1f},	//GBGR_CTL1 //0x1f
		{0x71, 0x00},	//Test Setting
		{0x72, 0x00},	//Test Setting
		{0x73, 0x00},	//Test Setting
		{0x74, 0x12},	//GBGR_G_UNIT_TH//12
		{0x75, 0x12},	//GBGR_RB_UNIT_TH//12
		{0x76, 0x20},	//GBGR_EDGE_TH
		{0x77, 0x80},	//GBGR_HLVL_TH
		{0x78, 0x88},	//GBGR_HLVL_COMP
		{0x79, 0x18},	//Test Setting

		{0x90, 0x3d},
		{0x91, 0x34},
		{0x99, 0x28},
		{0x9c, 0x05}, //14 For defect
		{0x9d, 0x08}, //15 For defect
		{0x9e, 0x28},
		{0x9f, 0x28},

		{0xb0, 0x7d}, //75 White Defect
		{0xb5, 0x44},
		{0xb6, 0x82},
		{0xb7, 0x52},
		{0xb8, 0x44},
		{0xb9, 0x15},

		///////////////////////////// Page 13
		{0x03, 0x13}, //Page 13 - Sharpness
		{0x10, 0x01},
		{0x11, 0x8f}, //89}, //for resolution , 131119
		{0x12, 0x14},
		{0x13, 0x19},
		{0x14, 0x08},	//Test Setting
		{0x20, 0x05}, //03},	//SHARP_Negative //for resolution , 131119
		{0x21, 0x05}, //04}, //03},	//SHARP_Positive 131105
		{0x23, 0x25},	//SHARP_DY_CTL
		{0x24, 0x21},	//40->33
		{0x25, 0x08},	//SHARP_PGA_TH
		{0x26, 0x40},	//Test Setting
		{0x27, 0x00},	//Test Setting
		{0x28, 0x08},	//Test Setting
		{0x29, 0x50},	//AG_TH
		{0x2a, 0xe0},	//region ratio
		{0x2b, 0x10},	//Test Setting
		{0x2c, 0x28},	//Test Setting
		{0x2d, 0x40},	//Test Setting
		{0x2e, 0x00},	//Test Setting
		{0x2f, 0x00},	//Test Setting
		{0x30, 0x11},	//Test Setting
		{0x80, 0x05},	//SHARP2D_CTL
		{0x81, 0x07},	//Test Setting
		{0x90, 0x05}, //04},	//SHARP2D_SLOPE  //for resolution , 131119
		{0x91, 0x06}, //05},	//SHARP2D_DIFF_CTL //for resolution , 131119
		{0x92, 0x00},	//SHARP2D_HI_CLIP
		{0x93, 0x30},	//SHARP2D_DY_CTL
		{0x94, 0x30},	//Test Setting
		{0x95, 0x10},	//Test Setting

		///////////////////////////// Page 14
		{0x03, 0x14}, //Page 14 - Lens Shading Correction
		{0x10, 0x01},
		{0x20, 0x80}, //60},   //XCEN LHC
		{0x21, 0x80}, //YCEN
		{0x22, 0x7d}, //_20131209 //88}, //7b}, //6a}, //50},
		{0x23, 0x5d}, //_20131209 //5c}, //50}, //44}, //40},
		{0x24, 0x48}, //_20131209 //49}, //44}, //32}, //3d},

		//////////////////////////// 15page
		{0x03, 0x15},
		{0x10, 0x03},
		{0x14, 0x52},	//CMCOFSGM
		{0x16, 0x3a},	//CMCOFSGL
		{0x17, 0x2f},	//CMC SIGN

		{0x30, 0xf1},
		{0x31, 0x71},
		{0x32, 0x00},
		{0x33, 0x1f},
		{0x34, 0xe1},
		{0x35, 0x42},
		{0x36, 0x01},
		{0x37, 0x31},
		{0x38, 0x72},

		{0x40, 0x90}, //CMC OFS
		{0x41, 0x82},
		{0x42, 0x12},
		{0x43, 0x86},
		{0x44, 0x92},
		{0x45, 0x18},
		{0x46, 0x84},
		{0x47, 0x02},
		{0x48, 0x02},
		//////////////////////////// 16page
		{0x03, 0x16}, //gamma
		{0x30, 0x00},
		{0x31, 0x08},
		{0x32, 0x1c},
		{0x33, 0x2f},
		{0x34, 0x53},
		{0x35, 0x76},
		{0x36, 0x93},
		{0x37, 0xac},
		{0x38, 0xc0},
		{0x39, 0xd0},
		{0x3a, 0xdc},
		{0x3b, 0xed},
		{0x3c, 0xf4}, //f7
		{0x3d, 0xf6}, //fc
		{0x3e, 0xfa}, //ff
		//////////////////////////// 17page
		{0x03, 0x17},
		{0xc0, 0x01},
		{0xc4, 0x4b}, //3c},
		{0xc5, 0x3e}, //32},
		///////////////////////////// Page 20	- Auto Exposure
		{0x03, 0x20},
		{0x10, 0x0c},	//AECTL
		{0x11, 0x04},
		{0x18, 0x30}, //31}, // 130521, Flicker Test
		{0x20, 0x01},	//FrameCTL
		{0x28, 0x27},	//FineCTL
		{0x29, 0xa1},
		{0x2a, 0xf0},
		{0x2b, 0x34},
		{0x2c, 0x2b},	 // 130521, Flicker Test
		{0x39, 0x22},
		{0x3a, 0xde},
		{0x3b, 0x23},
		{0x3c, 0xde},

					 // 130521, Flicker Test
		{0x60, 0x71}, //70}, //0x71}, //AE weight
		{0x61, 0x00}, //0x11},
		{0x62, 0x71}, //70}, //0x71},
		{0x63, 0x11}, //00}, //0x11},
		{0x68, 0x32}, //3c}, //30}, //AE_CEN
		{0x69, 0x6e}, //64}, //6a},
		{0x6A, 0x50}, //27}, //27},
		{0x6B, 0xa0}, //bb}, //bb},
		{0x70, 0x36}, //34}, 20130524  //Y Targe 32
		{0x76, 0x88}, //22}, // Unlock bnd1
		{0x77, 0xfe}, //02}, // Unlock bnd2
		{0x78, 0x22}, //23 //22}, 20130524  //12}, // Yth 1
		{0x79, 0x26}, //Yth 2
		{0x7a, 0x23}, //Yth 3
		{0x7c, 0x1c}, //Yth 2
		{0x7d, 0x22}, //Yth 4

		{0x83, 0x01}, //EXP Normal 20.00 fps
		{0x84, 0x24},
		{0x85, 0xf8},
		{0x86, 0x00}, //EXPMin 7500.00 fps
		{0x87, 0xc8},
		{0x88, 0x02}, //EXP Max(120Hz) 10.00 fps
		{0x89, 0x49},
		{0x8a, 0xf0},
		{0xa0, 0x02}, //EXP Max(100Hz) 10.00 fps
		{0xa1, 0x49},
		{0xa2, 0xf0},
		{0x8B, 0x3a}, //EXP100
		{0x8C, 0x98},
		{0x8D, 0x30}, //EXP120
		{0x8E, 0xd4},
		{0x9c, 0x04}, //EXP Limit 1250.00 fps
		{0x9d, 0xb0},
		{0x9e, 0x00}, //EXP Unit
		{0x9f, 0xc8},

		{0x98, 0x8c}, //Outdoor BRIGHT_MEASURE_TH
		{0x99, 0x23},

		{0xb0, 0x1d},
		{0xb1, 0x14}, //14
		{0xb2, 0xb0}, //a0}, //80
		{0xb3, 0x17}, //AGLVL //17
		{0xb4, 0x17},
		{0xb5, 0x3e},
		{0xb6, 0x2b},
		{0xb7, 0x24},
		{0xb8, 0x21},
		{0xb9, 0x1f},
		{0xba, 0x1e},
		{0xbb, 0x1d},
		{0xbc, 0x1c},
		{0xbd, 0x1b},

		{0xc0, 0x1a}, //PGA_sky
		{0xc3, 0x48}, //PGA_dark_on
		{0xc4, 0x48}, //PGA_dark_off

		{0x03, 0x22}, //Page 22 AWB
		{0x10, 0xe2},
		{0x11, 0x2E}, //26},
		{0x20, 0x41}, //01 //69
		{0x21, 0x40},
		{0x30, 0x80},
		{0x31, 0x80},
		{0x38, 0x12},
		{0x39, 0x33},
		{0x40, 0xf3}, //b8}, //93}, //f0},
					  // STEVE Yellowish
		{0x41, 0x54}, //0x44}, //0x54},
		{0x42, 0x33}, //0x22}, //0x33},
		{0x43, 0xf3}, //0xf8}, //0xf3},
		{0x44, 0x55}, //0x55}, //0x55},
		{0x45, 0x44}, //0x44}, //0x44},
		{0x46, 0x02}, //0x08}, //0x02},

		{0x80, 0x3b}, //_20131209 // //40}, // R
		{0x81, 0x20}, //20}, // G
		{0x82, 0x44}, //_20131209 // //38}, // B

		{0x83, 0x59}, //58}, //5a}, //52}, //RMAX
		{0x84, 0x20}, //1d}, //RMIN
		{0x85, 0x53}, //BMAX 5a
		{0x86, 0x24}, //BMIN

		{0x87, 0x49}, //48}, //4a}, //42
		{0x88, 0x3c},
		{0x89, 0x3e},
		{0x8a, 0x34},

		{0x8b, 0x00}, //02}, //0x08}, //02}, //OUT TH
		{0x8d, 0x24}, //3a}, //0x11}, //22},
		{0x8e, 0x61}, //b3}, //0x11}, //71},

		{0x8f, 0x63}, //65}, //0x63},
		{0x90, 0x60}, //61}, //0x60},
		{0x91, 0x5c}, //5C}, //0x5c},
		{0x92, 0x56}, //56}, //0x56},
		{0x93, 0x52}, //4E}, //0x52},
		{0x94, 0x4c}, //43}, //0x4c},
		{0x95, 0x3e}, //3A}, //0x3e},
		{0x96, 0x2f}, //32}, //0x2f},
		{0x97, 0x28}, //2A}, //0x28},
		{0x98, 0x23}, //24}, //0x23},
		{0x99, 0x21}, //20}, //0x21},
		{0x9a, 0x20}, //1C}, //0x20},
		{0x9b, 0x08}, //0a}, //0x07},

		/////////////////////////////
		// Page 48 MIPI /////////////
		/////////////////////////////
		{0x03, 0x48},
	  //{0x10, 0x05},			//05},
		{0x11, 0x0c}, //04}, //00}, con
		{0x16, 0x88}, //88}, //c8}, //c4},
		{0x1a, 0x00},				 //00},
		{0x1b, 0x35},				 //00},
		{0x1c, 0x03}, // HSP_LPX	 //02},
		{0x1d, 0x05}, // HSN_LPX	 //04},
		{0x1e, 0x07}, // HS_ZERO	 //07},
		{0x1f, 0x03}, // HS_TRAIL	 //05},
		{0x20, 0x00}, // HS_EXIT	 //00},
		{0x21, 0xb8}, // HS_SYNC
		{0x28, 0x00}, // ADD
		{0x30, 0x05},				 //05},
		{0x31, 0x00},				 //00},
		{0x32, 0x07}, // CLK_ZERO	 //06},
		{0x35, 0x01}, // CLK_TRAIL	 //03},
		{0x34, 0x01}, // CLK_PREPARE //02},
		{0x36, 0x01}, // CLKP_LPX	 //01},
		{0x37, 0x03}, // CLKN_LPX	 //03},
		{0x38, 0x02}, // CLK_EXIT	 //00},
		{0x39, 0xef},				 //4a},
		{0x3c, 0x00},				 //00},
		{0x3d, 0xfa},				 //fa},
		{0x3f, 0x10},				 //10},
		{0x40, 0x00},				 //00},
		{0x41, 0x20},				 //20},
		{0x42, 0x00},				 //00},
		{0x10, 0x05},				 //05},

		{0x03, 0x20},
		{0x10, 0xec}, //cc}, //120hz first 20130524
		{0x03, 0x22}, //Page 22 AWB
		{0x10, 0xfb},

};

/* LGE_CHANGE_S, Fix for Dual Camera Module of HI707, 2014-02-28, dongsu.bag@lge.com */
static struct msm_camera_i2c_reg_conf hi707_lgit_recommend_vt_settings[] = {
			
{0x03, 0x00},
{0x01, 0x71},
{0x01, 0x73},
{0x01, 0x71},
{0x03, 0x20},
{0x10, 0x1c},

#if defined(CONFIG_MACH_MSM8926_VFP_KR)
{0x18, 0x38},
#endif
{0x03, 0x22},
{0x10, 0x7b},
{0x03, 0x00},
{0x08, 0x0f},
{0x10, 0x00},

#if defined(CONFIG_MACH_MSM8926_VFP_KR)
{0x11, 0x94},
#else
{0x11, 0x90},
#endif
{0x12, 0x00},
{0x14, 0x88},
{0x0b, 0xaa},
{0x0c, 0xaa},
{0x0d, 0xaa},
{0xc0, 0x95},
{0xc1, 0x18},
{0xc2, 0x91},
{0xc3, 0x00},
{0xc4, 0x01},

{0x03, 0x00},
{0x20, 0x00},
{0x21, 0x04},
{0x22, 0x00},
{0x23, 0x04},
{0x40, 0x00},
{0x41, 0x90},
#if defined(CONFIG_MACH_MSM8926_VFP_KR)
{0x42, 0x05},
{0x43, 0x46},
#else
{0x42, 0x00},
{0x43, 0x14},
#endif
             //BLC
{0x80, 0x2e},
{0x81, 0x7e},
{0x82, 0x90},
{0x83, 0x30},
{0x84, 0x2c},
{0x85, 0x4b},
{0x86, 0x01},
{0x88, 0x47},
{0x89, 0x48}, 
#if defined(CONFIG_MACH_MSM8926_VFP_KR)
{0x90, 0x0e},
{0x91, 0x0e},
#else
{0x90, 0x11},
{0x91, 0x11},
#endif
{0x92, 0xf0},
{0x93, 0xe8},
{0x98, 0x38},

{0x99, 0x40},
{0xa0, 0x40},
{0xa8, 0x42},

{0xc0, 0x95},
{0xc1, 0x18},
{0xc2, 0x91},	
{0xc3, 0x00},
{0xc4, 0x01},

{0x03, 0x02},
{0x10, 0x00},
{0x11, 0x00},
{0x13, 0x40},
{0x14, 0x04},
{0x18, 0x1c},
{0x19, 0x00},
{0x1a, 0x00},
{0x1b, 0x08},
{0x1c, 0x9c},
{0x1d, 0x03},
{0x20, 0x33},
{0x21, 0x77},
{0x22, 0xa7},
{0x23, 0x32},
{0x24, 0x33},
{0x2b, 0x40},
{0x2d, 0x32},
{0x31, 0x99},
{0x32, 0x00},
{0x33, 0x00},
{0x34, 0x3c},
{0x35, 0x0d},
{0x3b, 0x80}, //60
{0x50, 0x21},
{0x51, 0x1c},
{0x52, 0xaa},
{0x53, 0x5a},
{0x54, 0x30},
{0x55, 0x10},
{0x56, 0x0c},
{0x58, 0x00},
{0x59, 0x0f},
{0x60, 0x34},
{0x61, 0x3a},
{0x62, 0x34},
{0x63, 0x39},
{0x64, 0x34},
{0x65, 0x39},
{0x72, 0x35},
{0x73, 0x38},
{0x74, 0x35},
{0x75, 0x38},
{0x80, 0x02},
{0x81, 0x2e},
{0x82, 0x0d},
{0x83, 0x10},
{0x84, 0x0d},
{0x85, 0x10},
{0x92, 0x1d},
{0x93, 0x20},
{0x94, 0x1d},
{0x95, 0x20},
{0xa0, 0x03},
{0xa1, 0x2d},
{0xa4, 0x2d},
{0xa5, 0x03},
{0xa8, 0x12},
{0xa9, 0x1b},
{0xaa, 0x22},
{0xab, 0x2b},
{0xac, 0x10},
{0xad, 0x0e},
{0xb8, 0x33},
{0xb9, 0x35},
{0xbc, 0x0c},
{0xbd, 0x0e},
{0xc0, 0x3a},
{0xc1, 0x3f},
{0xc2, 0x3a},
{0xc3, 0x3f},
{0xc4, 0x3a},
{0xc5, 0x3e},
{0xc6, 0x3a},
{0xc7, 0x3e},
{0xc8, 0x3a},
{0xc9, 0x3e},
{0xca, 0x3a},
{0xcb, 0x3e},
{0xcc, 0x3b},
{0xcd, 0x3d},
{0xce, 0x3b},
{0xcf, 0x3d},
{0xd0, 0x33},
{0xd1, 0x3f},
{0x03, 0x10},
{0x10, 0x03},
{0x11, 0x43},
{0x12, 0x30},
{0x40, 0x80},
{0x41, 0x16}, //DYOFS 02
{0x48, 0x80},
{0x50, 0x48}, //PGA brightness
{0x60, 0x01},
{0x61, 0x00},
{0x62, 0x70},
{0x63, 0x80},
{0x64, 0x48},
{0x66, 0x90},
{0x67, 0x36},
{0x80, 0x00},
{0x03, 0x11},
{0x10, 0x25},
{0x11, 0x07},
{0x20, 0x00},
{0x21, 0x60},
{0x23, 0x0a},
{0x60, 0x13},
{0x61, 0x85},
{0x62, 0x00},
{0x63, 0x00},
{0x64, 0x00},
{0x67, 0x70},
{0x68, 0x24},
{0x69, 0x04},
{0x03, 0x12},
{0x40, 0xd3},
{0x41, 0x09},
{0x50, 0x18},
{0x51, 0x24},
{0x70, 0x1f},
{0x71, 0x00},
{0x72, 0x00},
{0x73, 0x00},
{0x74, 0x12},
{0x75, 0x12},
{0x76, 0x20},
{0x77, 0x80},
{0x78, 0x88},
{0x79, 0x18},
{0x90, 0x3d},
{0x91, 0x34},
{0x99, 0x28},
{0x9c, 0x05},
{0x9d, 0x08},
{0x9e, 0x28},
{0x9f, 0x28},
{0xb0, 0x7d},
{0xb5, 0x44},
{0xb6, 0x82},
{0xb7, 0x52},
{0xb8, 0x44},
{0xb9, 0x15},
{0x03, 0x13},
{0x10, 0x01},
{0x11, 0x89},
{0x12, 0x14},
{0x13, 0x19},
{0x14, 0x08},
{0x20, 0x03},
{0x21, 0x04},
{0x23, 0x25},
{0x24, 0x21},
{0x25, 0x08},
{0x26, 0x40},
{0x27, 0x00},
{0x28, 0x08},
{0x29, 0x50},
{0x2a, 0xe0},
{0x2b, 0x10},
{0x2c, 0x28},
{0x2d, 0x40},
{0x2e, 0x00},
{0x2f, 0x00},
{0x30, 0x11},
{0x80, 0x05},
{0x81, 0x07},
{0x90, 0x04},
{0x91, 0x05},
{0x92, 0x00},
{0x93, 0x30},
{0x94, 0x30},
{0x95, 0x10},

{0x03, 0x14}, 
{0x10, 0x01},
{0x20, 0x80}, 
{0x21, 0x80}, 
{0x22, 0x87}, 
{0x23, 0x5b},
{0x24, 0x4c},
{0x27, 0x80},
{0x28, 0x70},
{0x29, 0x80},
{0x2a, 0x70},
{0x2b, 0x80},
{0x2c, 0x70},

{0x03, 0x15},
{0x10, 0x03},
{0x14, 0x52},
{0x16, 0x3a},
{0x17, 0x2f},
{0x30, 0xf1}, //RR
{0x31, 0x71}, //RG
{0x32, 0x00}, //RB
{0x33, 0x1f}, //GR
{0x34, 0xe1}, //GG
{0x35, 0x42}, //GB
{0x36, 0x01}, //BR
{0x37, 0x31}, //BG
{0x38, 0x72}, //BB
{0x40, 0x90},
{0x41, 0x82},
{0x42, 0x12},
{0x43, 0x86},
{0x44, 0x92},
{0x45, 0x18},
{0x46, 0x84},
{0x47, 0x02},
{0x48, 0x02},

{0x03, 0x16},
{0x10, 0x01},
{0x30, 0x00},
{0x31, 0x0a},
{0x32, 0x1b},
{0x33, 0x2e},
{0x34, 0x5c},
{0x35, 0x79},
{0x36, 0x95},
{0x37, 0xa4},
{0x38, 0xb1},
{0x39, 0xbd},
{0x3a, 0xc8},
{0x3b, 0xd9},
{0x3c, 0xe8},
{0x3d, 0xf5},
{0x3e, 0xff},

{0x03, 0x17},
{0xc4, 0x3c},
{0xc5, 0x32},

{0x03, 0x20},
{0x10, 0x0c},
{0x11, 0x04},
{0x20, 0x01},
{0x28, 0x27},
{0x29, 0xa1},
{0x2a, 0x90},
{0x2b, 0xf5},
{0x2c, 0x2b},
{0x30, 0xf8},
{0x3b, 0x22},
{0x3c, 0xde},
{0x39, 0x22},
{0x3a, 0xde},
{0x3b, 0x22},
{0x3c, 0xde},
{0x60, 0x70},
{0x61, 0x10},
{0x62, 0x70},
{0x63, 0x10},
{0x68, 0x30},
{0x69, 0x6a},
{0x6A, 0x27},
{0x6B, 0xbb},
{0x70, 0x3a},
{0x76, 0x22},
{0x77, 0x81},
{0x78, 0x22},
{0x79, 0x27},
{0x7a, 0x23},
{0x7c, 0x17},
{0x7d, 0x22},
{0x83, 0x01}, //EXP Normal 20.00 fps 
{0x84, 0x24}, 
{0x85, 0xf8},
{0x86, 0x00},
{0x87, 0xc8},

#if defined(CONFIG_MACH_MSM8926_VFP_KR)
{0x88, 0x02}, //EXP Max(120Hz) 7.06 fps
{0x89, 0xab},
{0x8a, 0x98},
{0xa0, 0x02}, //EXP Max(100Hz) 7.14 fps
{0xa1, 0xbf},
{0xa2, 0x20},
{0x8B, 0x3a},
{0x8C, 0x98},
{0x8D, 0x30},
{0x8E, 0xd4},
{0x91, 0x02}, //EXP Fix 7.00 fps
{0x92, 0xdc},
{0x93, 0x6c},
{0x98, 0x8C},
{0x99, 0x23},
{0x9c, 0x04},
{0x9d, 0xb0},
#else
{0x88, 0x03}, //EXP Max(120Hz) 7.06 fps 
{0x89, 0x3e}, 
{0x8a, 0x14}, 
{0xa0, 0x03}, //EXP Max(100Hz) 7.14 fps 
{0xa1, 0x34}, 
{0xa2, 0x50}, 
{0x8B, 0x3a},
{0x8C, 0x98},
{0x8D, 0x30},
{0x8E, 0xd4},
{0x91, 0x03}, //EXP Fix 7.00 fps
{0x92, 0x44}, 
{0x93, 0xb8}, 
{0x98, 0x8C},
{0x99, 0x23},
{0x9c, 0x05},
{0x9d, 0x78},
#endif
{0x9e, 0x00},
{0x9f, 0xc8},
{0xb0, 0x1d},
{0xb1, 0x14},
{0xb2, 0xf8},
{0xb3, 0x17},
{0xb4, 0x17},
{0xb5, 0x3e},
{0xb6, 0x2b},
{0xb7, 0x24},
{0xb8, 0x21},
{0xb9, 0x1f},
{0xba, 0x1e},
{0xbb, 0x1d},
{0xbc, 0x1c},
{0xbd, 0x1b},
{0xc0, 0x1a},
{0xc3, 0x48},
{0xc4, 0x48},

{0x03, 0x22},
{0x10, 0xe2},
{0x11, 0x2e},
{0x20, 0x41},
{0x21, 0x40},
{0x24, 0xfe},
{0x30, 0x80},
{0x31, 0x80},
{0x38, 0x12},
{0x39, 0x33},
{0x40, 0xf3}, //ylimit
{0x41, 0x43}, //cdiff
{0x42, 0x33}, //csum
{0x43, 0xf3}, //ylimit_tot
{0x44, 0x44}, //cdiff_tot
{0x45, 0x66}, //csum_tot
{0x46, 0x08},
{0x47, 0x63},
{0x80, 0x3d},
{0x81, 0x20},
{0x82, 0x40},
{0x83, 0x5a},
{0x84, 0x22},
{0x85, 0x57},
{0x86, 0x24},
{0x87, 0x41},
{0x88, 0x33},
{0x89, 0x3e},
{0x8a, 0x34},
{0x8b, 0x03},
{0x8d, 0x22},
{0x8e, 0x21},
{0x8f, 0x58},
{0x90, 0x56},
{0x91, 0x53},
{0x92, 0x4e},
{0x93, 0x47},
{0x94, 0x41},
{0x95, 0x3b},
{0x96, 0x33},
{0x97, 0x2f},
{0x98, 0x2b},
{0x99, 0x29},
{0x9a, 0x27},
{0x9b, 0x06},

{0x03, 0x48},
{0x11, 0x0c},
{0x16, 0x88},
{0x1a, 0x00},	
{0x1b, 0x35},	
{0x1c, 0x03},
{0x1d, 0x05},
{0x1e, 0x07},
{0x1f, 0x03},
{0x20, 0x00},
{0x21, 0xb8},
{0x28, 0x00},
{0x30, 0x05},	
{0x31, 0x00},	
{0x32, 0x07}, 
{0x35, 0x01}, 
{0x34, 0x01}, 
{0x36, 0x01}, 
{0x37, 0x03}, 
{0x38, 0x02}, 
{0x39, 0xef},		
{0x3c, 0x00},		
{0x3d, 0xfa},		
{0x3f, 0x10},		
{0x40, 0x00},		
{0x41, 0x20},		
{0x42, 0x00},		
{0x10, 0x05},

{0x03, 0x22},
{0x10, 0xfb},
{0x03, 0x20},

#if defined(CONFIG_MACH_MSM8926_VFP_KR)
{0x10, 0xec},
{0x18, 0x30},
#else
{0x10, 0x8c},
#endif
		
		//{0x03, 0x00},
		//{0x01, 0x70},
};
static struct msm_camera_i2c_reg_conf hi707_cowell_recommend_vt_settings[] = {
		
{0x03, 0x00},
{0x01, 0x71},
{0x01, 0x73},
{0x01, 0x71},
{0x03, 0x20},
{0x10, 0x1c},

#if defined(CONFIG_MACH_MSM8926_VFP_KR)
{0x18, 0x38},
#endif

{0x03, 0x22},
{0x10, 0x7b},
{0x03, 0x00},
{0x08, 0x0f},
{0x10, 0x00},
#if defined(CONFIG_MACH_MSM8926_VFP_KR)
{0x11, 0x94},
#else
{0x11, 0x90},
#endif
{0x12, 0x00},
{0x14, 0x88},
{0x0b, 0xaa},
{0x0c, 0xaa},
{0x0d, 0xaa},
{0xc0, 0x95},
{0xc1, 0x18},
{0xc2, 0x91},
{0xc3, 0x00},
{0xc4, 0x01},

{0x03, 0x00},
{0x20, 0x00},
{0x21, 0x04},
{0x22, 0x00},
{0x23, 0x04},
{0x40, 0x00},
#if defined(CONFIG_MACH_MSM8926_VFP_KR)
{0x41, 0x90},
{0x42, 0x05},
{0x43, 0x46},
#else
{0x41, 0x90},
{0x42, 0x00},
{0x43, 0x14},
#endif
             //BLC
{0x80, 0x2e},
{0x81, 0x7e},
{0x82, 0x90},
{0x83, 0x30},
{0x84, 0x2c},
{0x85, 0x4b},
{0x86, 0x01},
{0x88, 0x47},
{0x89, 0x48}, 
#if defined(CONFIG_MACH_MSM8926_VFP_KR)
{0x90, 0x0e},
{0x91, 0x0e},
#else
{0x90, 0x11},
{0x91, 0x11},
#endif
{0x92, 0xf0},
{0x93, 0xe8},
{0x98, 0x38},

{0x99, 0x40},
{0xa0, 0x40},
{0xa8, 0x42},

{0xc0, 0x95},
{0xc1, 0x18},
{0xc2, 0x91},	
{0xc3, 0x00},
{0xc4, 0x01},

{0x03, 0x02},
{0x10, 0x00},
{0x11, 0x00},
{0x13, 0x40},
{0x14, 0x04},
{0x18, 0x1c},
{0x19, 0x00},
{0x1a, 0x00},
{0x1b, 0x08},
{0x1c, 0x9c},
{0x1d, 0x03},
{0x20, 0x33},
{0x21, 0x77},
{0x22, 0xa7},
{0x23, 0x32},
{0x24, 0x33},
{0x2b, 0x40},
{0x2d, 0x32},
{0x31, 0x99},
{0x32, 0x00},
{0x33, 0x00},
{0x34, 0x3c},
{0x35, 0x0d},
{0x3b, 0x80}, //60
{0x50, 0x21},
{0x51, 0x1c},
{0x52, 0xaa},
{0x53, 0x5a},
{0x54, 0x30},
{0x55, 0x10},
{0x56, 0x0c},
{0x58, 0x00},
{0x59, 0x0f},
{0x60, 0x34},
{0x61, 0x3a},
{0x62, 0x34},
{0x63, 0x39},
{0x64, 0x34},
{0x65, 0x39},
{0x72, 0x35},
{0x73, 0x38},
{0x74, 0x35},
{0x75, 0x38},
{0x80, 0x02},
{0x81, 0x2e},
{0x82, 0x0d},
{0x83, 0x10},
{0x84, 0x0d},
{0x85, 0x10},
{0x92, 0x1d},
{0x93, 0x20},
{0x94, 0x1d},
{0x95, 0x20},
{0xa0, 0x03},
{0xa1, 0x2d},
{0xa4, 0x2d},
{0xa5, 0x03},
{0xa8, 0x12},
{0xa9, 0x1b},
{0xaa, 0x22},
{0xab, 0x2b},
{0xac, 0x10},
{0xad, 0x0e},
{0xb8, 0x33},
{0xb9, 0x35},
{0xbc, 0x0c},
{0xbd, 0x0e},
{0xc0, 0x3a},
{0xc1, 0x3f},
{0xc2, 0x3a},
{0xc3, 0x3f},
{0xc4, 0x3a},
{0xc5, 0x3e},
{0xc6, 0x3a},
{0xc7, 0x3e},
{0xc8, 0x3a},
{0xc9, 0x3e},
{0xca, 0x3a},
{0xcb, 0x3e},
{0xcc, 0x3b},
{0xcd, 0x3d},
{0xce, 0x3b},
{0xcf, 0x3d},
{0xd0, 0x33},
{0xd1, 0x3f},
{0x03, 0x10},
{0x10, 0x03},
{0x11, 0x43},
{0x12, 0x30},
{0x40, 0x80},
{0x41, 0x16}, //DYOFS 02
{0x48, 0x80},
{0x50, 0x48}, //PGA brightness
{0x60, 0x01},
{0x61, 0x00},
{0x62, 0x70}, //80},
{0x63, 0x80},
{0x64, 0x48},
{0x66, 0x90},
{0x67, 0x36},
{0x80, 0x00},
{0x03, 0x11},
{0x10, 0x25},
{0x11, 0x07},
{0x20, 0x00},
{0x21, 0x60},
{0x23, 0x0a},
{0x60, 0x13},
{0x61, 0x85},
{0x62, 0x00},
{0x63, 0x00},
{0x64, 0x00},
{0x67, 0x70},
{0x68, 0x24},
{0x69, 0x04},
{0x03, 0x12},
{0x40, 0xd3},
{0x41, 0x09},
{0x50, 0x18},
{0x51, 0x24},
{0x70, 0x1f},
{0x71, 0x00},
{0x72, 0x00},
{0x73, 0x00},
{0x74, 0x12},
{0x75, 0x12},
{0x76, 0x20},
{0x77, 0x80},
{0x78, 0x88},
{0x79, 0x18},
{0x90, 0x3d},
{0x91, 0x34},
{0x99, 0x28},
{0x9c, 0x05},
{0x9d, 0x08},
{0x9e, 0x28},
{0x9f, 0x28},
{0xb0, 0x7d},
{0xb5, 0x44},
{0xb6, 0x82},
{0xb7, 0x52},
{0xb8, 0x44},
{0xb9, 0x15},
{0x03, 0x13},
{0x10, 0x01},
{0x11, 0x89},
{0x12, 0x14},
{0x13, 0x19},
{0x14, 0x08},
{0x20, 0x03},
{0x21, 0x04},
{0x23, 0x25},
{0x24, 0x21},
{0x25, 0x08},
{0x26, 0x40},
{0x27, 0x00},
{0x28, 0x08},
{0x29, 0x50},
{0x2a, 0xe0},
{0x2b, 0x10},
{0x2c, 0x28},
{0x2d, 0x40},
{0x2e, 0x00},
{0x2f, 0x00},
{0x30, 0x11},
{0x80, 0x05},
{0x81, 0x07},
{0x90, 0x04},
{0x91, 0x05},
{0x92, 0x00},
{0x93, 0x30},
{0x94, 0x30},
{0x95, 0x10},

{0x03, 0x14}, 
{0x10, 0x01},
{0x20, 0x80}, 
{0x21, 0x80}, 

{0x22, 0x86}, //87},//85}, 
{0x23, 0x5a}, //5b},//5c},  
{0x24, 0x4b}, //4c},//49},
{0x27, 0x80}, //60},
{0x28, 0x80}, //80},
{0x29, 0x80}, //60},
{0x2a, 0x80}, //80},
{0x2b, 0x80}, //60},
{0x2c, 0x80}, //80},

{0x03, 0x15},
{0x10, 0x03},
{0x14, 0x52},
{0x16, 0x3a},
{0x17, 0x2f},
{0x30, 0xf1}, //RR
{0x31, 0x71}, //RG
{0x32, 0x00}, //RB
{0x33, 0x1f}, //GR
{0x34, 0xe1}, //GG
{0x35, 0x42}, //GB
{0x36, 0x01}, //BR
{0x37, 0x31}, //BG
{0x38, 0x72}, //BB
{0x40, 0x90},
{0x41, 0x82},
{0x42, 0x12},
{0x43, 0x86},
{0x44, 0x92},
{0x45, 0x18},
{0x46, 0x84},
{0x47, 0x02},
{0x48, 0x02},

{0x03, 0x16},
{0x10, 0x01},
{0x30, 0x00},
{0x31, 0x0a},
{0x32, 0x1b},
{0x33, 0x2e},
{0x34, 0x5c},
{0x35, 0x79},
{0x36, 0x95},
{0x37, 0xa4},
{0x38, 0xb1},
{0x39, 0xbd},
{0x3a, 0xc8},
{0x3b, 0xd9},
{0x3c, 0xe8},
{0x3d, 0xf5},
{0x3e, 0xff},

{0x03, 0x17},
{0xc4, 0x3c},
{0xc5, 0x32},

{0x03, 0x20},
{0x10, 0x0c},
{0x11, 0x04},
{0x20, 0x01},
{0x28, 0x27},
{0x29, 0xa1},
{0x2a, 0x90},
{0x2b, 0xf5},
{0x2c, 0x2b},
{0x30, 0xf8},
{0x3b, 0x22},
{0x3c, 0xde},
{0x39, 0x22},
{0x3a, 0xde},
{0x3b, 0x22},
{0x3c, 0xde},
{0x60, 0x70},
{0x61, 0x10},
{0x62, 0x70},
{0x63, 0x10},
{0x68, 0x30},
{0x69, 0x6a},
{0x6A, 0x27},
{0x6B, 0xbb},
{0x70, 0x3a}, //42},
{0x76, 0x22},
{0x77, 0x81},
{0x78, 0x22},
{0x79, 0x27},
{0x7a, 0x23},
{0x7c, 0x17},
{0x7d, 0x22},
{0x83, 0x01}, //EXP Normal 20.00 fps 
{0x84, 0x24}, 
{0x85, 0xf8},
{0x86, 0x00},
{0x87, 0xc8},

#if defined(CONFIG_MACH_MSM8926_VFP_KR)
{0x88, 0x02}, //EXP Max(120Hz) 7.06 fps
{0x89, 0xab},
{0x8a, 0x98},
{0xa0, 0x02}, //EXP Max(100Hz) 7.14 fps
{0xa1, 0xbf},
{0xa2, 0x20},
{0x8B, 0x3a},
{0x8C, 0x98},
{0x8D, 0x30},
{0x8E, 0xd4},
{0x91, 0x02}, //EXP Fix 7.00 fps
{0x92, 0xdc},
{0x93, 0x6c},
{0x98, 0x8C},
{0x99, 0x23},
{0x9c, 0x04},
{0x9d, 0xb0},
#else
{0x88, 0x03}, //EXP Max(120Hz) 7.06 fps 
{0x89, 0x3e}, 
{0x8a, 0x14}, 
{0xa0, 0x03}, //EXP Max(100Hz) 7.14 fps 
{0xa1, 0x34}, 
{0xa2, 0x50}, 
{0x8B, 0x3a},
{0x8C, 0x98},
{0x8D, 0x30},
{0x8E, 0xd4},
{0x91, 0x03}, //EXP Fix 7.00 fps
{0x92, 0x44}, 
{0x93, 0xb8}, 
{0x98, 0x8C},
{0x99, 0x23},
{0x9c, 0x05},
{0x9d, 0x78},
#endif
{0x9e, 0x00},
{0x9f, 0xc8},
{0xb0, 0x1d},
{0xb1, 0x14},
{0xb2, 0xf8},
{0xb3, 0x17},
{0xb4, 0x17},
{0xb5, 0x3e},
{0xb6, 0x2b},
{0xb7, 0x24},
{0xb8, 0x21},
{0xb9, 0x1f},
{0xba, 0x1e},
{0xbb, 0x1d},
{0xbc, 0x1c},
{0xbd, 0x1b},
{0xc0, 0x1a},
{0xc3, 0x48},
{0xc4, 0x48},

{0x03, 0x22},
{0x10, 0xe2},
{0x11, 0x2e},
{0x20, 0x41},
{0x21, 0x40},
{0x24, 0xfe},
{0x30, 0x80},
{0x31, 0x80},
{0x38, 0x12},
{0x39, 0x33},
{0x40, 0xf3}, //ylimit
{0x41, 0x43}, //cdiff
{0x42, 0x33}, //csum
{0x43, 0xf3}, //ylimit_tot
{0x44, 0x44}, //cdiff_tot
{0x45, 0x66}, //csum_tot
{0x46, 0x08},
{0x47, 0x63},
{0x80, 0x3d},
{0x81, 0x20},
{0x82, 0x40},
{0x83, 0x5a},
{0x84, 0x22},
{0x85, 0x57},
{0x86, 0x24},
{0x87, 0x41},
{0x88, 0x33},
{0x89, 0x3e},
{0x8a, 0x34},
{0x8b, 0x03},
{0x8d, 0x22},
{0x8e, 0x21},
{0x8f, 0x58},
{0x90, 0x56},
{0x91, 0x53},
{0x92, 0x4e},
{0x93, 0x47},
{0x94, 0x41},
{0x95, 0x3b},
{0x96, 0x33},
{0x97, 0x2f},
{0x98, 0x2b},
{0x99, 0x29},
{0x9a, 0x27},
{0x9b, 0x06},

{0x03, 0x48},
{0x11, 0x0c},
{0x16, 0x88},
{0x1a, 0x00},	
{0x1b, 0x35},	
{0x1c, 0x03},
{0x1d, 0x05},
{0x1e, 0x07},
{0x1f, 0x03},
{0x20, 0x00},
{0x21, 0xb8},
{0x28, 0x00},
{0x30, 0x05},	
{0x31, 0x00},	
{0x32, 0x07}, 
{0x35, 0x01}, 
{0x34, 0x01}, 
{0x36, 0x01}, 
{0x37, 0x03}, 
{0x38, 0x02}, 
{0x39, 0xef},		
{0x3c, 0x00},		
{0x3d, 0xfa},		
{0x3f, 0x10},		
{0x40, 0x00},		
{0x41, 0x20},		
{0x42, 0x00},		
{0x10, 0x05},

{0x03, 0x22},
{0x10, 0xfb},
{0x03, 0x20},
#if defined(CONFIG_MACH_MSM8926_VFP_KR)
{0x10, 0xec},
{0x18, 0x30},
#else
{0x10, 0x8c},
#endif
	
	//{0x03, 0x00},
	//{0x01, 0x70},

};
/* LGE_CHANGE_E, Fix for Dual Camera Module of HI707, 2014-02-28, dongsu.bag@lge.com */

static struct msm_camera_i2c_reg_conf hi707_reg_effect_off[] = {
	/* OFF */
	{0x03, 0x10},
	{0x11, 0x43},
	{0x12, 0x30},
	{0x44, 0x80},
	{0x45, 0x80},

};

static struct msm_camera_i2c_reg_conf hi707_reg_effect_mono[] = {
	/* MONO */
	{0x03, 0x10},
	{0x11, 0x03},
	{0x12, 0x33},
	{0x44, 0x80},
	{0x45, 0x80},

};

static struct msm_camera_i2c_reg_conf hi707_reg_effect_negative[] = {
	/* Negative: */
	{0x03, 0x10},
	{0x11, 0x03},
	{0x12, 0x38},
	{0x44, 0x80},
	{0x45, 0x80},
};

static struct msm_camera_i2c_reg_conf hi707_reg_effect_sepia[] = {
	/* SEPIA  */
	{0x03, 0x10},
	{0x11, 0x03},
	{0x12, 0x33},
	{0x44, 0x70},
	{0x45, 0x98},

};

static struct msm_camera_i2c_reg_conf hi707_reg_exposure_compensation[13][2] = {
	/* -6 */
	{
		{0x03, 0x10},
		{0x40, 0xBC},
	},
	/* -5 */
	{
		{0x03, 0x10},
		{0x40, 0xB2},
	},
	/* -4 */
	{
		{0x03, 0x10},
		{0x40, 0xA8},
	},
	/* -3 */
	{
		{0x03, 0x10},
		{0x40, 0x9E},
	},
	/* -2 */
	{
		{0x03, 0x10},
		{0x40, 0x94},
	},
	/* -1 */
	{
		{0x03, 0x10},
		{0x40, 0x8A},
	},
	/* 0 */
	{
		{0x03, 0x10},
		{0x40, 0x80},
	},
	/* 1 */
	{
		{0x03, 0x10},
		{0x40, 0x0A},
	},
	/* 2 */
	{
		{0x03, 0x10},
		{0x40, 0x14},
	},
	/* 3 */
	{
		{0x03, 0x10},
		{0x40, 0x1E},
	},
	/* 4 */
	{
		{0x03, 0x10},
		{0x40, 0x28},
	},
	/* 5 */
	{
		{0x03, 0x10},
		{0x40, 0x32},
	},
	/* 6 */
	{
		{0x03, 0x10},
		{0x40, 0x3C},
	},
};

static struct msm_camera_i2c_reg_conf hi707_reg_scene_auto[] = {
	/* SCENE_auto: 10~30fps */
	{0x03, 0x00},
	{0x09, 0x01}, //SLEEP ON	
	
	{0x03, 0x20},
	{0x10, 0x0c},
	{0x18, 0x38}, //AE Reset ON
	
	{0x03, 0x00},
	{0x11, 0x90},
	{0x40, 0x00}, //Hblank 144
	{0x41, 0x90}, 
	{0x42, 0x00}, 
	{0x43, 0x04},	
	
	{0x03, 0x20},
	{0x2a, 0xf0},
	{0x2b, 0x34}, 
	{0x83, 0x01}, //EXP Normal 20.00 fps
	{0x84, 0x24},
	{0x85, 0xf8},
	{0x88, 0x02}, //EXP Max(120Hz) 10.00 fps
	{0x89, 0x49},
	{0x8a, 0xf0},
	{0xa0, 0x02}, //EXP Max(100Hz) 10.00 fps
	{0xa1, 0x49},
	{0xa2, 0xf0},
	{0x03, 0x00}, //PAGE 0
	{0x90, 0x0c}, //BLC_TIME_TH_ON
	{0x91, 0x0c}, //BLC_TIME_TH_OFF
	{0x92, 0xa8}, //98}, //BLC_AG_TH_ON
	{0x93, 0xa0}, //90}, //BLC_AG_TH_OFF
	
	{0x03, 0x20},
	{0x10, 0xec}, //cc},
	{0x18, 0x30}, //AE Reset OFF
	
	{0x03, 0x00},
	{0x09, 0x00}, //SLEEP Off
};

static struct msm_camera_i2c_reg_conf hi707_reg_wb_auto[] = {
	/* Auto */
	{0x03, 0x22},
	{0x10, 0x7b},
	{0x11, 0x2e},
	{0x80, 0x40}, //3d},
	{0x81, 0x20},
	{0x82, 0x38}, //40},
	{0x83, 0x59}, //58}, //RMAX
	{0x84, 0x20}, //RMIN
	{0x85, 0x53}, //BMAX
	{0x86, 0x24}, //BMIN
	{0x87, 0x49}, //48}, //RMAXB
	{0x88, 0x3c}, //RMINB
	{0x89, 0x3e}, //BMAXB
	{0x8a, 0x34}, //BMINB
	{0x8d, 0x24}, //11,}, //IN/OUT slop R
	{0x8e, 0x61}, //11,}, //IN/OUT slop B
	{0x10, 0xfb},
};
/* LGE_CHANGE_S, Fixes to add product_kor info capabilities for setting Hi707 of kor medel, 2014-03-11, dongsu.bag@lge.com */
static struct msm_camera_i2c_reg_conf hi707_reg_wb_auto_kr[] = {
	/* Auto */
	{0x03, 0x22},
	{0x10, 0x7b},
	{0x11, 0x2e},
	{0x80, 0x40}, //3d},
	{0x81, 0x20},
	{0x82, 0x38}, //40},
	{0x83, 0x59}, //58}, //RMAX
	{0x84, 0x20}, //RMIN
	{0x85, 0x53}, //BMAX
	{0x86, 0x24}, //BMIN
	{0x87, 0x49}, //48}, //RMAXB
	{0x88, 0x3c}, //RMINB
	{0x89, 0x3e}, //BMAXB
	{0x8a, 0x34}, //BMINB
	{0x8d, 0x24}, //11,}, //IN/OUT slop R
	{0x8e, 0x61}, //11,}, //IN/OUT slop B
	{0x10, 0xfb},
};

static struct msm_camera_i2c_reg_conf hi707_reg_wb_incandescent_kr[] = {
	/* INCANDESCENT */
	{0x03, 0x22},
	{0x10, 0x7b},
	{0x11, 0x26},
	//INCA
	{0x80, 0x20},
	{0x81, 0x20},
	{0x82, 0x60},
	{0x83, 0x2F},
	{0x84, 0x11},
	{0x85, 0x67},
	{0x86, 0x58},
	{0x87, 0x60}, //RMAX
	{0x88, 0x20}, //RMIN
	{0x89, 0x60}, //BMAX
	{0x8a, 0x20}, //BMIN
	{0x8d, 0x00}, //IN/OUT slop R
	{0x8e, 0x00}, //IN/OUT slop B
	{0x10, 0xfb},
};

static struct msm_camera_i2c_reg_conf hi707_reg_wb_fluorescent_kr[] = {
	/* FLUORESCENT */
	{0x03, 0x22},
	{0x10, 0x7b},
	{0x11, 0x26},
	//TL84
	{0x80, 0x2d},
	{0x81, 0x20},
	{0x82, 0x50},
	{0x83, 0x37},
	{0x84, 0x23},
	{0x85, 0x55},
	{0x86, 0x4B},
	{0x87, 0x60}, //RMAX
	{0x88, 0x20}, //RMIN
	{0x89, 0x60}, //BMAX
	{0x8a, 0x20}, //BMIN
	{0x8d, 0x00}, //IN/OUT slop R
	{0x8e, 0x00}, //IN/OUT slop B
	{0x10, 0xfb},
};

static struct msm_camera_i2c_reg_conf hi707_reg_wb_sunny_kr[] = {
	/* DAYLIGHT */
	{0x03, 0x22},
	{0x10, 0x7b},
	{0x11, 0x26},
	//D50
	{0x80, 0x42},
	{0x81, 0x20},
	{0x82, 0x3d},
	{0x83, 0x49},
	{0x84, 0x3A},
	{0x85, 0x47},
	{0x86, 0x33},
	{0x87, 0x60}, //RMAX
	{0x88, 0x20}, //RMIN
	{0x89, 0x60}, //BMAX
	{0x8a, 0x20}, //BMIN
	{0x8d, 0x00}, //IN/OUT slop R
	{0x8e, 0x00}, //IN/OUT slop B
	{0x10, 0xfb},
};

static struct msm_camera_i2c_reg_conf hi707_reg_wb_cloudy_kr[] = {
	/* CLOUDY_DAYLIGHT */
	{0x03, 0x22},
	{0x10, 0x7b},
	{0x11, 0x26},
	//Cloudy
	{0x80, 0x60},
	{0x81, 0x20},
	{0x82, 0x20},
	{0x83, 0x70},
	{0x84, 0x51},
	{0x85, 0x2A},
	{0x86, 0x20},
	{0x87, 0x60}, //RMAX
	{0x88, 0x20}, //RMIN
	{0x89, 0x60}, //BMAX
	{0x8a, 0x20}, //BMIN
	{0x8d, 0x00}, //IN/OUT slop R
	{0x8e, 0x00}, //IN/OUT slop B
	{0x10, 0xfb},
};

/* LGE_CHANGE_E, Fixes to add product_kor info capabilities for setting Hi707 of kor medel, 2014-03-11, dongsu.bag@lge.com */
static struct msm_camera_i2c_reg_conf hi707_reg_wb_incandescent[] = {
	/* INCANDESCENT */
	{0x03, 0x22},
	{0x10, 0x7b},
	{0x11, 0x26},
	//INCA
	{0x80, 0x20},
	{0x81, 0x20},
	{0x82, 0x60},
	{0x83, 0x2F},
	{0x84, 0x11},
	{0x85, 0x67},
	{0x86, 0x58},
	{0x87, 0x60}, //RMAX
	{0x88, 0x20}, //RMIN
	{0x89, 0x60}, //BMAX
	{0x8a, 0x20}, //BMIN
	{0x8d, 0x00}, //IN/OUT slop R
	{0x8e, 0x00}, //IN/OUT slop B
	{0x10, 0xfb},
};

static struct msm_camera_i2c_reg_conf hi707_reg_wb_fluorescent[] = {
	/* FLUORESCENT */
	{0x03, 0x22},
	{0x10, 0x7b},
	{0x11, 0x26},
	//TL84
	{0x80, 0x2d},
	{0x81, 0x20},
	{0x82, 0x50},
	{0x83, 0x37},
	{0x84, 0x23},
	{0x85, 0x55},
	{0x86, 0x4B},
	{0x87, 0x60}, //RMAX
	{0x88, 0x20}, //RMIN
	{0x89, 0x60}, //BMAX
	{0x8a, 0x20}, //BMIN
	{0x8d, 0x00}, //IN/OUT slop R
	{0x8e, 0x00}, //IN/OUT slop B
	{0x10, 0xfb},
};

static struct msm_camera_i2c_reg_conf hi707_reg_wb_sunny[] = {
	/* DAYLIGHT */
	{0x03, 0x22},
	{0x10, 0x7b},
	{0x11, 0x26},
	//D50
	{0x80, 0x42},
	{0x81, 0x20},
	{0x82, 0x3d},
	{0x83, 0x49},
	{0x84, 0x3A},
	{0x85, 0x47},
	{0x86, 0x33},
	{0x87, 0x60}, //RMAX
	{0x88, 0x20}, //RMIN
	{0x89, 0x60}, //BMAX
	{0x8a, 0x20}, //BMIN
	{0x8d, 0x00}, //IN/OUT slop R
	{0x8e, 0x00}, //IN/OUT slop B
	{0x10, 0xfb},
};

static struct msm_camera_i2c_reg_conf hi707_reg_wb_cloudy[] = {
	/* CLOUDY_DAYLIGHT */
	{0x03, 0x22},
	{0x10, 0x7b},
	{0x11, 0x26},
	//Cloudy
	{0x80, 0x60},
	{0x81, 0x20},
	{0x82, 0x20},
	{0x83, 0x70},
	{0x84, 0x51},
	{0x85, 0x2A},
	{0x86, 0x20},
	{0x87, 0x60}, //RMAX
	{0x88, 0x20}, //RMIN
	{0x89, 0x60}, //BMAX
	{0x8a, 0x20}, //BMIN
	{0x8d, 0x00}, //IN/OUT slop R
	{0x8e, 0x00}, //IN/OUT slop B
	{0x10, 0xfb},
};

static struct msm_camera_i2c_reg_conf hi707_AE_weight[] = {
{0x60, 0x70},
{0x61, 0x00},
{0x62, 0x70},
{0x63, 0x00},
};

static struct msm_camera_i2c_reg_conf hi707_AE_window[] = {
{0x68, 0x30},
{0x69, 0x6a},
{0x6a, 0x27},
{0x6b, 0xbb},
};

static struct v4l2_subdev_info hi707_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_YUYV8_2X8,/* For YUV type sensor (YUV422) */
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

/*
static struct msm_camera_i2c_reg_conf hi707_config_change_settings[] = {
};
*/
static const struct i2c_device_id hi707_i2c_id[] = {
	{HI707_SENSOR_NAME, (kernel_ulong_t)&hi707_s_ctrl},
	{ }
};

/*LGE_CHANGE_S, fixed Fps setting of soc sensor for VT mode, 2014-01-27, dongsu.bag@lge.com*/
static struct msm_camera_i2c_reg_conf hi707_reg_7fps_fixed[] = {
	//Fixed 7fps
	
#if defined(CONFIG_MACH_MSM8926_VFP_KR)
	//No need to set 7fps since vt-mode recommended setting is 7fps and only SKT model uses 7fps setting.
#else
{0x03, 0x00},
{0x09, 0x01}, //SLEEP ON

{0x03, 0x20}, //page 20
{0x10, 0x0c}, //AE OFF
{0x18, 0x38},

{0x03, 0x00},
{0x11, 0x94},
{0x40, 0x00},
{0x41, 0x90},
{0x42, 0x00},
{0x43, 0x14},

{0x03, 0x20}, //Page 20
{0x2a, 0xf0}, 
{0x2b, 0x35}, 
{0x83, 0x01}, //EXP Normal 20.00 fps 
{0x84, 0x24}, 
{0x85, 0xf8},
{0x86, 0x00},
{0x87, 0xc8},
{0x88, 0x03}, //EXP Max(120Hz) 7.06 fps 
{0x89, 0x3e}, 
{0x8a, 0x14}, 
{0xa0, 0x03}, //EXP Max(100Hz) 7.14 fps 
{0xa1, 0x34}, 
{0xa2, 0x50}, 
{0x8B, 0x3a},
{0x8C, 0x98},
{0x8D, 0x30},
{0x8E, 0xd4},
{0x91, 0x03}, //EXP Fix 7.00 fps
{0x92, 0x44}, 
{0x93, 0xb8},  
{0x9c, 0x05},
{0x9d, 0x78},
{0x9e, 0x00},
{0x9f, 0xc8},

//BLC 
{0x03, 0x00}, //PAGE 0
{0x90, 0x11},
{0x91, 0x11},
{0x92, 0xf0},
{0x93, 0xe8},


{0x03, 0x20},
{0x10, 0x8c}, //AE ON
{0x18, 0x30},

{0x03, 0x00},
{0x09, 0x00}, //SLEEP off
#endif
};

static struct msm_camera_i2c_reg_conf hi707_reg_10fps_fixed[] = {
	//Fixed 10fps
	
	{0x03, 0x00},
	{0x09, 0x01}, //SLEEP ON
	
	{0x03, 0x20}, //page 20
	{0x10, 0x0c}, //AE OFF
	{0x18, 0x38},
	
	{0x03, 0x00},
	{0x11, 0x94},
	{0x40, 0x00}, //Hblank 144
	{0x41, 0x90}, 
	{0x42, 0x03}, 
	{0x43, 0xb6},
	
	{0x03, 0x20}, //Page 20
	{0x2a, 0xf0}, 
	{0x2b, 0x35}, 
#if defined(CONFIG_MACH_MSM8926_VFP_KR)
	{0x83, 0x00}, //EXP Normal 30.00 fps
	{0x84, 0xc3},
	{0x85, 0x50},
	{0x88, 0x02}, //EXP Max(120Hz) 10.91 fps
	{0x89, 0x19},
	{0x8a, 0x1c},
	{0xa0, 0x02}, //EXP Max(100Hz) 11.11 fps
	{0xa1, 0x0f},
	{0xa2, 0x58},
#else
	{0x83, 0x01}, //EXP Normal 20.00 fps
	{0x84, 0x24}, 
	{0x85, 0xf8},	
	{0x88, 0x02}, //EXP Max(120Hz) 10.00 fps 
	{0x89, 0x49}, 
	{0x8a, 0xf0}, 
	{0xa0, 0x02}, //EXP Max(100Hz) 10.00 fps 
	{0xa1, 0x49}, 
	{0xa2, 0xf0},  
#endif
	{0x91, 0x02}, //EXP Fix 10.00 fps
	{0x92, 0x49}, 
	{0x93, 0xf0}, 
	
	{0x03, 0x00}, //PAGE 0
	{0x90, 0x0e}, //BLC_TIME_TH_ON
	{0x91, 0x0e}, //BLC_TIME_TH_OFF 
	{0x92, 0xa8}, //BLC_AG_TH_ON
	{0x93, 0xa0}, //BLC_AG_TH_OFF
	
	{0x03, 0x20},
	{0x10, 0xec}, //AE ON
	{0x18, 0x30},  
	
	{0x03, 0x00},
	{0x09, 0x00}, //SLEEP Off
};

static struct msm_camera_i2c_reg_conf hi707_reg_15fps_fixed[] = {
	//Fixed 15fps 
	
	{0x03, 0x00}, 
	{0x09, 0x01},	
	
	{0x03, 0x20},	
	{0x10, 0x0c},	
	{0x18, 0x38},	
	
	{0x03, 0x00},
	{0x11, 0x94}, // Fixed On
	{0x40, 0x00}, //Hblank 144
	{0x41, 0x90}, 

#if defined(CONFIG_MACH_MSM8926_VFP_KR)
	{0x42, 0x01},
	{0x43, 0xc2},
#else
	{0x42, 0x00}, 
	{0x43, 0x04}, 
#endif
	
	{0x03, 0x00}, //PAGE 0
	{0x90, 0x07}, //BLC_TIME_TH_ON
	{0x91, 0x07}, //BLC_TIME_TH_OFF 
	{0x92, 0xa8}, //98}, //BLC_AG_TH_ON
	{0x93, 0xa0}, //90}, //BLC_AG_TH_OFF  
	
	{0x03, 0x20}, //Page 20
	{0x2a, 0xf0}, 
	{0x2b, 0x35},

#if defined(CONFIG_MACH_MSM8926_VFP_KR)
	{0x83, 0x00}, //EXP Normal 30.00 fps
	{0x84, 0xc3},
	{0x85, 0x50},
#else
	{0x83, 0x01}, //EXP Normal 20.00 fps 
	{0x84, 0x24}, 
	{0x85, 0xf8},	
#endif
	{0x88, 0x01}, //EXP Max(120Hz) 17.14 fps 
	{0x89, 0x55}, 
	{0x8a, 0xcc}, 
	{0xa0, 0x01}, //EXP Max(100Hz) 16.67 fps 
	{0xa1, 0x5f}, 
	{0xa2, 0x90}, 
	{0x91, 0x01}, //EXP Fix 15.00 fps
	{0x92, 0x86}, 
	{0x93, 0xa0},	 
	
	{0x03, 0x20},
	{0x10, 0xec},
	{0x18, 0x30},	        
	
	{0x03, 0x00},
	{0x09, 0x00}, //SLEEP Off
};

static struct msm_camera_i2c_reg_conf hi707_reg_20fps_fixed[] = {
//Fixed 20fps 

	{0x03, 0x00}, 
	{0x09, 0x01},	
	
	{0x03, 0x20},	
	{0x10, 0x0c},	
	{0x18, 0x38},
	
	{0x03, 0x00},
	{0x11, 0x94}, //Fixed On
	{0x40, 0x00}, //Hblank 144
	{0x41, 0x90}, 
	{0x42, 0x00}, 

#if defined(CONFIG_MACH_MSM8926_VFP_KR)
	{0x43, 0xc8},
#else
	{0x43, 0x04}, 
#endif
	
	{0x03, 0x00}, //PAGE 0
	{0x90, 0x05}, //BLC_TIME_TH_ON
	{0x91, 0x05}, //BLC_TIME_TH_OFF 
	{0x92, 0xa8}, //BLC_AG_TH_ON
	{0x93, 0xa0}, //BLC_AG_TH_OFF  
	
	{0x03, 0x20}, //Page 20
	{0x2a, 0xf0}, 
	{0x2b, 0x35},
	{0x83, 0x00}, //EXP Normal 30.00 fps 
	{0x84, 0xc3}, 
	{0x85, 0x50}, 

#if defined(CONFIG_MACH_MSM8926_VFP_KR)
	{0x88, 0x00}, //EXP Max(120Hz) 24.00 fps
	{0x89, 0xf4},
	{0x8a, 0x24},
	{0xa0, 0x00}, //EXP Max(100Hz) 25.00 fps
	{0xa1, 0xea},
	{0xa2, 0x60},
#else
	{0x88, 0x01}, //EXP Max(120Hz) 20.00 fps 
	{0x89, 0x24}, 
	{0x8a, 0xf8}, 
	{0xa0, 0x01}, //EXP Max(100Hz) 20.00 fps 
	{0xa1, 0x24}, 
	{0xa2, 0xf8}, 
#endif
	{0x91, 0x01}, //EXP Fix 20.00 fps
	{0x92, 0x24}, 
	{0x93, 0xf8},    
	
	{0x03, 0x20},
	{0x10, 0xec},
	{0x18, 0x30},	
	
	{0x03, 0x00}, 
	{0x09, 0x00},
};
/*LGE_CHANGE_E, fixed Fps setting of soc sensor for VT mode, 2014-01-27, dongsu.bag@lge.com*/
static int32_t msm_hi707_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &hi707_s_ctrl);
}

static struct i2c_driver hi707_i2c_driver = {
	.id_table = hi707_i2c_id,
	.probe  = msm_hi707_i2c_probe,
	.driver = {
		.name = HI707_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client hi707_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static const struct of_device_id hi707_dt_match[] = {
	{.compatible = "qcom,hi707", .data = &hi707_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, hi707_dt_match);

static struct platform_driver hi707_platform_driver = {
	.driver = {
		.name = "qcom,hi707",
		.owner = THIS_MODULE,
		.of_match_table = hi707_dt_match,
	},
};

static void hi707_i2c_write_table(struct msm_sensor_ctrl_t *s_ctrl,
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

static int32_t hi707_platform_probe(struct platform_device *pdev)
{
	int32_t rc;
	const struct of_device_id *match;
	match = of_match_device(hi707_dt_match, &pdev->dev);
/* LGE_CHANGE_S, WBT issue fix, 2013-11-25, hyunuk.park@lge.com */
	if(!match)
	{
		  pr_err(" %s failed ",__func__);
		  return -ENODEV;
	}
/* LGE_CHANGE_E, WBT issue fix, 2013-11-25, hyunuk.park@lge.com */
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init hi707_init_module(void)
{
	int32_t rc;
	pr_info("%s:%d\n", __func__, __LINE__);
	// LGE_CHANGE_S, jongkwon.chae, 2014.05.29, To separate power settings depending on HW revisions.
#if defined(CONFIG_MACH_MSM8926_E2_SPR_US)
		switch(lge_get_board_revno()) {
			case HW_REV_A:
				printk("%s: Sensor power is set as Rev.A\n", __func__);
				hi707_s_ctrl.power_setting_array.power_setting = hi707_power_setting;
				hi707_s_ctrl.power_setting_array.size = ARRAY_SIZE(hi707_power_setting);
				break;
			case HW_REV_B:
				printk("%s: Sensor power is set as Rev.B\n", __func__);
				hi707_s_ctrl.power_setting_array.power_setting = hi707_power_setting_rev_b;
				hi707_s_ctrl.power_setting_array.size = ARRAY_SIZE(hi707_power_setting_rev_b);
				break;
			default:
				printk("%s: Sensor power is set as Rev.10\n", __func__);
				hi707_s_ctrl.power_setting_array.power_setting = hi707_power_setting_rev_b;
				hi707_s_ctrl.power_setting_array.size = ARRAY_SIZE(hi707_power_setting_rev_b);
				break;
		}
#else
		hi707_s_ctrl.power_setting_array.power_setting = hi707_power_setting;
		hi707_s_ctrl.power_setting_array.size = ARRAY_SIZE(hi707_power_setting);
#endif
	// LGE_CHANGE_E, jongkwon.chae, 2014.05.29, To separate power settings depending on HW revisions.

	rc = platform_driver_probe(&hi707_platform_driver,
		hi707_platform_probe);
	if (!rc)
		return rc;

	return i2c_add_driver(&hi707_i2c_driver);
}

static void __exit hi707_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (hi707_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&hi707_s_ctrl);
		platform_driver_unregister(&hi707_platform_driver);
	} else
		i2c_del_driver(&hi707_i2c_driver);
	return;
}

int32_t hi707_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{

	int32_t rc = 0;
	uint16_t chipid = 0;
	int maker_gpio = 0;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
			s_ctrl->sensor_i2c_client,
			SENSOR_REG_PAGE_ADDR,
			SENSOR_REG_PAGE_0, MSM_CAMERA_I2C_BYTE_DATA);

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			s_ctrl->sensordata->slave_info->sensor_id_reg_addr,
			&chipid, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: %d %s: read id failed\n", __func__,__LINE__,
			s_ctrl->sensordata->sensor_name);
		return rc;
	}

	CDBG("%s: read id: %x expected id %x:\n", __func__, chipid,
		s_ctrl->sensordata->slave_info->sensor_id);
	if (chipid != s_ctrl->sensordata->slave_info->sensor_id) {
		pr_err("msm_sensor_match_id chip id doesnot match\n");
		return -ENODEV;
	}
/*LGE_CHANGE_S, sync with upgraded kernel version, LOS. 2014.11.18. sujeong.kwon*/
/* LGE_CHANGE_S, Fixes to add product_kor info capabilities for setting Hi707 of kor medel, 2014-03-11, dongsu.bag@lge.com */
	product_kor = s_ctrl->sensordata->sensor_info->product_kor;
	pr_err("%s: product_kor - %d", __func__, product_kor);
/* LGE_CHANGE_E, Fixes to add product_kor info capabilities for setting Hi707 of kor medel, 2014-03-11, dongsu.bag@lge.com */
/* LGE_CHANGE_S, Fix for Dual Camera Module of HI707, 2014-02-28, dongsu.bag@lge.com */
	maker_gpio = s_ctrl->sensordata->sensor_info->maker_gpio;
	pr_err("%s: maker gpio - %d", __func__, maker_gpio);
	if( maker_gpio >= 0 ){
	 	if(gpio_is_valid(maker_gpio)){
			 if(gpio_request(maker_gpio, "vt_cam_id") == 0){
				 if(gpio_direction_input(maker_gpio) == 0){
					 vt_cam_id_value = gpio_get_value(maker_gpio);
					 pr_err("vt_cam_id(gpio %d) is %d\n", maker_gpio, vt_cam_id_value);
					 gpio_free(maker_gpio);
					 }else pr_err("unable to set direction for gpio %d\n", maker_gpio);
				 }else pr_err("gpio %d request failed\n", maker_gpio);
			 }else pr_err("Invalid gpio %d\n", maker_gpio);
			}
/* LGE_CHANGE_E, Fix for Dual Camera Module of HI707, 2014-03-04, dongsu.bag@lge.com */
/*LGE_CHANGE_E, sync with upgraded kernel version, LOS. 2014.11.18. sujeong.kwon*/

	return rc;

}

static void hi707_set_exposure_compensation(struct msm_sensor_ctrl_t *s_ctrl,
	int value)
{
	int val = 0;

	switch (value) {
		case -12 :{ val = 0; break;}
		case -10 :{ val = 1; break;}
		case -8  :{ val = 2; break;}
		case -6  :{ val = 3; break;}
		case -4  :{ val = 4; break;}
		case -2  :{ val = 5; break;}
		case 0   :{ val = 6; break;}
		case 2   :{ val = 7; break;}
		case 4   :{ val = 8; break;}
		case 6   :{ val = 9; break;}
		case 8   :{ val = 10; break;}
		case 10  :{ val = 11; break;}
		case 12  :{ val = 12; break;}
		default  :{ val = 6; break;}
	}

	CDBG("%s %d", __func__, val);
	hi707_i2c_write_table(s_ctrl, &hi707_reg_exposure_compensation[val][0],
		ARRAY_SIZE(hi707_reg_exposure_compensation[val]));
}

static void hi707_set_effect(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	CDBG("%s %d", __func__, value);

	switch (value) {
	case MSM_CAMERA_EFFECT_MODE_OFF: {
		hi707_i2c_write_table(s_ctrl, &hi707_reg_effect_off[0],
			ARRAY_SIZE(hi707_reg_effect_off));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_MONO: {
		hi707_i2c_write_table(s_ctrl, &hi707_reg_effect_mono[0],
			ARRAY_SIZE(hi707_reg_effect_mono));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_NEGATIVE: {
		hi707_i2c_write_table(s_ctrl, &hi707_reg_effect_negative[0],
			ARRAY_SIZE(hi707_reg_effect_negative));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_SEPIA: {
		hi707_i2c_write_table(s_ctrl, &hi707_reg_effect_sepia[0],
			ARRAY_SIZE(hi707_reg_effect_sepia));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_SOLARIZE:
	case MSM_CAMERA_EFFECT_MODE_POSTERIZE:
	case MSM_CAMERA_EFFECT_MODE_WHITEBOARD:
	case MSM_CAMERA_EFFECT_MODE_BLACKBOARD:
	case MSM_CAMERA_EFFECT_MODE_AQUA:
	case MSM_CAMERA_EFFECT_MODE_EMBOSS:
	case MSM_CAMERA_EFFECT_MODE_SKETCH:
	case MSM_CAMERA_EFFECT_MODE_NEON:
	default:
		hi707_i2c_write_table(s_ctrl, &hi707_reg_effect_off[0],
			ARRAY_SIZE(hi707_reg_effect_off));
		break;
	}
}

static void hi707_set_scene_mode(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	CDBG("%s %d", __func__, value);

	switch (value) {
	case MSM_CAMERA_SCENE_MODE_OFF: {
		hi707_i2c_write_table(s_ctrl, &hi707_reg_scene_auto[0],
			ARRAY_SIZE(hi707_reg_scene_auto));
		break;
	}
	case MSM_CAMERA_SCENE_MODE_NIGHT: {
		break;
	}
	case MSM_CAMERA_SCENE_MODE_LANDSCAPE:
	case MSM_CAMERA_SCENE_MODE_SNOW:
	case MSM_CAMERA_SCENE_MODE_BEACH:
	case MSM_CAMERA_SCENE_MODE_SUNSET:
	case MSM_CAMERA_SCENE_MODE_PORTRAIT:
	case MSM_CAMERA_SCENE_MODE_BACKLIGHT:
	case MSM_CAMERA_SCENE_MODE_SPORTS:
	case MSM_CAMERA_SCENE_MODE_ANTISHAKE:
	case MSM_CAMERA_SCENE_MODE_FLOWERS:
	case MSM_CAMERA_SCENE_MODE_CANDLELIGHT:
	case MSM_CAMERA_SCENE_MODE_FIREWORKS:
	case MSM_CAMERA_SCENE_MODE_PARTY:
	case MSM_CAMERA_SCENE_MODE_NIGHT_PORTRAIT:
	case MSM_CAMERA_SCENE_MODE_THEATRE:
	case MSM_CAMERA_SCENE_MODE_ACTION:
	case MSM_CAMERA_SCENE_MODE_AR:
	case MSM_CAMERA_SCENE_MODE_FACE_PRIORITY:
	case MSM_CAMERA_SCENE_MODE_BARCODE:
	case MSM_CAMERA_SCENE_MODE_HDR:
	default:
		hi707_i2c_write_table(s_ctrl, &hi707_reg_scene_auto[0],
			ARRAY_SIZE(hi707_reg_scene_auto));
		break;
	}
}

static void hi707_set_white_balance_mode(struct msm_sensor_ctrl_t *s_ctrl,
	int value)
{
	CDBG("%s %d", __func__, value);

	switch (value) {
	case MSM_CAMERA_WB_MODE_AUTO: {
		hi707_i2c_write_table(s_ctrl, &hi707_reg_wb_auto[0],
			ARRAY_SIZE(hi707_reg_wb_auto));
		break;
	}
	case MSM_CAMERA_WB_MODE_INCANDESCENT: {
		hi707_i2c_write_table(s_ctrl, &hi707_reg_wb_incandescent[0],
			ARRAY_SIZE(hi707_reg_wb_incandescent));
		break;
	}
	case MSM_CAMERA_WB_MODE_FLUORESCENT: {
		hi707_i2c_write_table(s_ctrl, &hi707_reg_wb_fluorescent[0],
			ARRAY_SIZE(hi707_reg_wb_fluorescent));
		break;
	}
	case MSM_CAMERA_WB_MODE_DAYLIGHT: {
		hi707_i2c_write_table(s_ctrl, &hi707_reg_wb_sunny[0],
			ARRAY_SIZE(hi707_reg_wb_sunny));
		break;
	}
	case MSM_CAMERA_WB_MODE_CLOUDY_DAYLIGHT: {
		hi707_i2c_write_table(s_ctrl, &hi707_reg_wb_cloudy[0],
			ARRAY_SIZE(hi707_reg_wb_cloudy));
		break;
	}
	case MSM_CAMERA_WB_MODE_CUSTOM:
	case MSM_CAMERA_WB_MODE_WARM_FLUORESCENT:
	case MSM_CAMERA_WB_MODE_TWILIGHT:
	case MSM_CAMERA_WB_MODE_SHADE:
	case MSM_CAMERA_WB_MODE_OFF:
	default:
		hi707_i2c_write_table(s_ctrl, &hi707_reg_wb_auto[0],
			ARRAY_SIZE(hi707_reg_wb_auto));
		break;
	}
}

/* LGE_CHANGE_S, Fixes to add product_kor info capabilities for setting Hi707 of kor medel, 2014-03-11, dongsu.bag@lge.com */
static void hi707_set_white_balance_mode_kr(struct msm_sensor_ctrl_t *s_ctrl,
	int value)
{
	CDBG("%s %d", __func__, value);

	switch (value) {
	case MSM_CAMERA_WB_MODE_AUTO: {
		hi707_i2c_write_table(s_ctrl, &hi707_reg_wb_auto_kr[0],
			ARRAY_SIZE(hi707_reg_wb_auto_kr));
		break;
	}
	case MSM_CAMERA_WB_MODE_INCANDESCENT: {
		hi707_i2c_write_table(s_ctrl, &hi707_reg_wb_incandescent_kr[0],
			ARRAY_SIZE(hi707_reg_wb_incandescent_kr));
		break;
	}
	case MSM_CAMERA_WB_MODE_FLUORESCENT: {
		hi707_i2c_write_table(s_ctrl, &hi707_reg_wb_fluorescent_kr[0],
			ARRAY_SIZE(hi707_reg_wb_fluorescent_kr));
		break;
	}
	case MSM_CAMERA_WB_MODE_DAYLIGHT: {
		hi707_i2c_write_table(s_ctrl, &hi707_reg_wb_sunny_kr[0],
			ARRAY_SIZE(hi707_reg_wb_sunny_kr));
		break;
	}
	case MSM_CAMERA_WB_MODE_CLOUDY_DAYLIGHT: {
		hi707_i2c_write_table(s_ctrl, &hi707_reg_wb_cloudy_kr[0],
			ARRAY_SIZE(hi707_reg_wb_cloudy_kr));
		break;
	}
	case MSM_CAMERA_WB_MODE_CUSTOM:
	case MSM_CAMERA_WB_MODE_WARM_FLUORESCENT:
	case MSM_CAMERA_WB_MODE_TWILIGHT:
	case MSM_CAMERA_WB_MODE_SHADE:
	case MSM_CAMERA_WB_MODE_OFF:
	default:
		hi707_i2c_write_table(s_ctrl, &hi707_reg_wb_auto_kr[0],
			ARRAY_SIZE(hi707_reg_wb_auto_kr));
		break;
	}
}
/* LGE_CHANGE_S, Fixes to add product_kor info capabilities for setting Hi707 of kor medel, 2014-03-11, dongsu.bag@lge.com */

static int32_t hi707_set_aec_roi_mode(struct msm_sensor_ctrl_t *s_ctrl,
	int value)
{
	int32_t rc = 0;
	int16_t coordinate_x, coordinate_y;
	int16_t x_start, x_end, y_start, y_end;

	coordinate_x = ((value >> 12)&0xFFF);
	coordinate_y = value&0xFFF;

	CDBG("%s %d", __func__, value);
	CDBG("%s: coordinate_x -%x\n", __func__, coordinate_x);
	CDBG("%s: coordinate_y -%x\n", __func__, coordinate_y);
	if(coordinate_x == 0 && coordinate_y == 0)
	{
	pr_err("%s: roi zero value",__func__);
	return rc;
	}

	if(coordinate_x == -1 && coordinate_y == -1) {
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,
						SENSOR_REG_PAGE_ADDR,
						SENSOR_REG_PAGE_20,
						MSM_CAMERA_I2C_BYTE_DATA);
		/* AE weight */
	    hi707_i2c_write_table(s_ctrl,&hi707_AE_weight[0],
				ARRAY_SIZE(hi707_AE_weight));
		/* AE window */
	    hi707_i2c_write_table(s_ctrl,&hi707_AE_window[0],
				ARRAY_SIZE(hi707_AE_window));


		if (rc < 0) {
			pr_err("%s: %s: failed\n", __func__,s_ctrl->sensordata->sensor_name);
			return rc;
		}
	}
	else {

#ifdef CONFIG_HI707_ROT_180
		coordinate_x = SENSOR_PREVIEW_WIDTH - coordinate_x;
		coordinate_y = SENSOR_PREVIEW_HEIGHT -coordinate_y;
#endif

		x_start = ((coordinate_x - (AEC_ROI_DX/2) > 0)? coordinate_x - (AEC_ROI_DX/2) : 0)/4;
		x_end = ((coordinate_x + (AEC_ROI_DX/2) < SENSOR_PREVIEW_WIDTH)? coordinate_x + (AEC_ROI_DX/2) : SENSOR_PREVIEW_WIDTH)/4;

		y_start = ((coordinate_y - (AEC_ROI_DY/2) > 0)? coordinate_y - (AEC_ROI_DY/2) : 0)/2;
		y_end = ((coordinate_y + (AEC_ROI_DY/2) < SENSOR_PREVIEW_HEIGHT)? coordinate_y + (AEC_ROI_DY/2) : SENSOR_PREVIEW_HEIGHT)/2;

		CDBG("%s : (%d, %d), (%d, %d)\n", __func__, x_start, y_start, x_end, y_end);

		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, SENSOR_REG_PAGE_ADDR, SENSOR_REG_PAGE_20, MSM_CAMERA_I2C_BYTE_DATA);

		/* AE weight */
	    hi707_i2c_write_table(s_ctrl,&hi707_AE_weight[0],
				ARRAY_SIZE(hi707_AE_weight));

		/* AE window */
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x68, x_start, MSM_CAMERA_I2C_BYTE_DATA);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x69, x_end, MSM_CAMERA_I2C_BYTE_DATA);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x6a, y_start, MSM_CAMERA_I2C_BYTE_DATA);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x6b, y_end, MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0) {
			pr_err("%s: %s: failed\n", __func__,s_ctrl->sensordata->sensor_name);
			return rc;
		}
	}

	return rc;
}

static int32_t hi707_set_awb_lock_mode(struct msm_sensor_ctrl_t *s_ctrl,
	int value)
{
	int32_t rc = 0;
	int16_t temp = 0;
	CDBG("%s %d", __func__, value);

	if(PREV_SOC_AWB_LOCK != value){

		PREV_SOC_AWB_LOCK = value;

		if(PREV_SOC_AWB_LOCK == AWB_LOCK_ON){
			CDBG("%s soc_awb_lock %d\n", __func__, PREV_SOC_AWB_LOCK);

			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x03, 0x22, MSM_CAMERA_I2C_BYTE_DATA);	//page mode
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, 0x10, &temp, MSM_CAMERA_I2C_BYTE_DATA);
			temp = temp & 0x7f; //[7]bit set as '0'
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x10, temp, MSM_CAMERA_I2C_BYTE_DATA);
		}
		else if(PREV_SOC_AWB_LOCK == AWB_LOCK_OFF){
			CDBG("%s soc_awb_unlock %d\n", __func__, PREV_SOC_AWB_LOCK);

			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x03, 0x22, MSM_CAMERA_I2C_BYTE_DATA);	//page mode
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,	0x10, &temp, MSM_CAMERA_I2C_BYTE_DATA);
			temp = temp | 0x80; //[7]bit set as '1'
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x10, temp, MSM_CAMERA_I2C_BYTE_DATA);
		}
	}

	return rc;
}

static int32_t hi707_set_aec_lock_mode(struct msm_sensor_ctrl_t *s_ctrl,
	int value)
{
	int32_t rc = 0;
	int16_t temp = 0;
	CDBG("%s %d", __func__, value);

	if(PREV_SOC_AEC_LOCK != value){

		PREV_SOC_AEC_LOCK = value;

		if(PREV_SOC_AEC_LOCK == AEC_LOCK_ON){
			CDBG("%s soc_aec_lock %d\n", __func__, PREV_SOC_AEC_LOCK);

			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x03, 0x20, MSM_CAMERA_I2C_BYTE_DATA);	//page mode
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,	0x10, &temp, MSM_CAMERA_I2C_BYTE_DATA);
			temp = temp & 0x7f; //[7]bit set as '0'
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x10, temp, MSM_CAMERA_I2C_BYTE_DATA);
		}
		else if(PREV_SOC_AEC_LOCK == AEC_LOCK_OFF){
			CDBG("%s soc_aec_unlock %d\n", __func__, PREV_SOC_AEC_LOCK);

			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x03, 0x20, MSM_CAMERA_I2C_BYTE_DATA);	//page mode
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,	0x10, &temp, MSM_CAMERA_I2C_BYTE_DATA);
			temp = temp | 0x80; //[7]bit set as '1'
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x10, temp, MSM_CAMERA_I2C_BYTE_DATA);
		}
	}
	return rc;
}

//LGE_CHANGE_S,  This Function has been added only for fps of VIDEO Recording from SoC Camera Module. youngwook.song@lge.com 2013-11-04
static void hi707_set_framerate_for_soc(struct msm_sensor_ctrl_t *s_ctrl, struct msm_fps_range_setting *framerate)
{
	int32_t value = 0;
	 // 1088421888 is 7.0 in float value.
	if((framerate->min_fps == 1088421888) && (framerate->max_fps == 1088421888))
		value = 0;
	 //1092616192 is 10.0 in float value.
	else if((framerate->min_fps == 1092616192) && (framerate->max_fps == 1092616192))
		value = 1;
	//1097859072 is 15.0 in float value.
	else if((framerate->min_fps == 1097859072) && (framerate->max_fps == 1097859072))
		value = 2;
	//1101004800 is 20.0 in float value.
	else if((framerate->min_fps == 1101004800) && (framerate->max_fps == 1101004800))
		value = 3;
    else value = 4;
	
	pr_debug("%s value: %d\n", __func__, value);
/* LGE_CHANGE_S, check current fps mode to avoid setting, 2014-02-11, hyunuk.park@lge.com */
	if(mCurrentFpsMode == value) {
		pr_err("%s : no set fps since the same fps requested (mCurrentFpsMode: %d)\n", __func__, mCurrentFpsMode);
		return;
	} else {
		mCurrentFpsMode = value;
	}
/* LGE_CHANGE_E, check current fps mode to avoid setting, 2014-02-11, hyunuk.park@lge.com */
	switch (value) {
		case 0: { 
			pr_err("%s - 7fps setting", __func__);
			hi707_i2c_write_table(s_ctrl, &hi707_reg_7fps_fixed[0],
			ARRAY_SIZE(hi707_reg_7fps_fixed));
			break;
			}
		case 1: { 
			pr_err("%s - 10fps setting", __func__);
			hi707_i2c_write_table(s_ctrl, &hi707_reg_10fps_fixed[0],
			ARRAY_SIZE(hi707_reg_10fps_fixed));
			break;
			}
		case 2: {
			pr_err("%s - 15fps setting", __func__);
			hi707_i2c_write_table(s_ctrl, &hi707_reg_15fps_fixed[0],
			ARRAY_SIZE(hi707_reg_15fps_fixed));
			break;
			}
		case 3: {
			pr_err("%s - 20fps setting", __func__);
			hi707_i2c_write_table(s_ctrl, &hi707_reg_20fps_fixed[0],
			ARRAY_SIZE(hi707_reg_20fps_fixed));
			break;
			}
		default:{  //default
			pr_err("%s %d\n", __func__, value);
			hi707_i2c_write_table(s_ctrl, &hi707_reg_scene_auto[0],
			ARRAY_SIZE(hi707_reg_scene_auto));
			}
			break;
	}
}
//LGE_CHANGE_E,  This Function has been added only for fps of VIDEO Recording from SoC Camera Module. youngwook.song@lge.com 2013-11-04
int32_t hi707_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	int32_t rc = 0;
	int32_t i = 0;

	mutex_lock(s_ctrl->msm_sensor_mutex);

	CDBG("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
		s_ctrl->sensordata->sensor_name, cdata->cfgtype);

	switch (cdata->cfgtype) {

	case CFG_GET_SENSOR_INFO:
		memcpy(cdata->cfg.sensor_info.sensor_name,
			s_ctrl->sensordata->sensor_name,
			sizeof(cdata->cfg.sensor_info.sensor_name));
		cdata->cfg.sensor_info.session_id =
			s_ctrl->sensordata->sensor_info->session_id;
		for (i = 0; i < SUB_MODULE_MAX; i++)
			cdata->cfg.sensor_info.subdev_id[i] =
				s_ctrl->sensordata->sensor_info->subdev_id[i];
/*LGE_CHANGE_S, sync with upgraded kernel version. 2014.11.18. sujeong.kwon*/
		cdata->cfg.sensor_info.is_mount_angle_valid =
			s_ctrl->sensordata->sensor_info->is_mount_angle_valid;
		cdata->cfg.sensor_info.sensor_mount_angle =
			s_ctrl->sensordata->sensor_info->sensor_mount_angle;
		cdata->cfg.sensor_info.position =
			s_ctrl->sensordata->sensor_info->position;
		cdata->cfg.sensor_info.modes_supported =
			s_ctrl->sensordata->sensor_info->modes_supported;
/*LGE_CHANGE_E, sync with upgraded kernel version. 2014.11.18. sujeong.kwon*/
		CDBG("%s:%d sensor name %s\n", __func__, __LINE__,
			cdata->cfg.sensor_info.sensor_name);
		CDBG("%s:%d session id %d\n", __func__, __LINE__,
			cdata->cfg.sensor_info.session_id);
		for (i = 0; i < SUB_MODULE_MAX; i++)
			CDBG("%s:%d subdev_id[%d] %d\n", __func__, __LINE__, i,
				cdata->cfg.sensor_info.subdev_id[i]);
/*LGE_CHANGE_S, sync with upgraded kernel version. 2014.11.18. sujeong.kwon*/
		CDBG("%s:%d mount angle valid %d value %d\n", __func__,
			__LINE__, cdata->cfg.sensor_info.is_mount_angle_valid,
			cdata->cfg.sensor_info.sensor_mount_angle);
/*LGE_CHANGE_E, sync with upgraded kernel version. 2014.11.18. sujeong.kwon*/

		break;
	case CFG_SET_INIT_SETTING:
/* LGE_CHANGE_S, Fix for Dual Camera Module of HI707, 2014-02-28, dongsu.bag@lge.com */
		if( vt_cam_id_value == HI707_LGIT )
			{
			pr_err("%s: init lgit setting", __func__);
			hi707_i2c_write_table(s_ctrl,
					&hi707_lgit_recommend_settings[0],
					ARRAY_SIZE(hi707_lgit_recommend_settings));
			}else{
			pr_err("%s: init cowell setting", __func__);
		hi707_i2c_write_table(s_ctrl,
					&hi707_cowell_recommend_settings[0],
					ARRAY_SIZE(hi707_cowell_recommend_settings));
				}
/* LGE_CHANGE_E, Fix for Dual Camera Module of HI707, 2014-02-28, dongsu.bag@lge.com */

		CDBG("init setting X");
		break;
	case CFG_SET_RESOLUTION:
		break;

	case CFG_SET_STOP_STREAM:
		pr_err("%s - stop stream",__func__);
		CDBG("STOP_STREAM\n");
		hi707_i2c_write_table(s_ctrl,
			&hi707_stop_settings[0],
			ARRAY_SIZE(hi707_stop_settings));
		CDBG("STOP_STREAM X\n");
#if defined(CONFIG_MACH_MSM8926_VFP_KR)
		//fix for not setting fps after stop preview
		mCurrentFpsMode = 4;
#endif
		break;
	case CFG_SET_START_STREAM:
		pr_err("%s - start stream",__func__);
		CDBG("START_STREAM\n");
/*LGE_CHANGE_S, mipi end packet issue, 2013-10-15, kwangsik83.kim@lge.com*/
		if(s_ctrl->isFirstStream == TRUE){
			hi707_i2c_write_table(s_ctrl, &hi707_entrance_start_settings[0], ARRAY_SIZE(hi707_entrance_start_settings));
			s_ctrl->isFirstStream = FALSE;
			pr_err("[WX] %s : entrance start stream\n", __func__);
		}
		else{
			hi707_i2c_write_table(s_ctrl, &hi707_start_settings[0], ARRAY_SIZE(hi707_start_settings));
			pr_err("[WX] %s : normal start stream\n", __func__);
		}
/*LGE_CHANGE_E, mipi end packet issue, 2013-10-15, kwangsik83.kim@lge.com*/
		CDBG("START_STREAM X\n");
		break;
	case CFG_GET_SENSOR_INIT_PARAMS:
/*LGE_CHANGE_S, sync with upgraded kernel version. 2014.11.18. sujeong.kwon*/
#if 0
		cdata->cfg.sensor_init_params =
			*s_ctrl->sensordata->sensor_init_params;
#else
		cdata->cfg.sensor_init_params.modes_supported =
			s_ctrl->sensordata->sensor_info->modes_supported;
		cdata->cfg.sensor_init_params.position =
			s_ctrl->sensordata->sensor_info->position;
		cdata->cfg.sensor_init_params.sensor_mount_angle =
			s_ctrl->sensordata->sensor_info->sensor_mount_angle;
#endif
/*LGE_CHANGE_S, sync with upgraded kernel version. 2014.11.18. sujeong.kwon*/

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
/*LGE_CHANGE_S, add soc exif, 2013-10-04, kwangsik83.kim@lge.com*/
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

		size = conf_array.size;		//size for write(page_mode) and read
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
				CDBG("[WX] %s read_data : %d\n", __func__, *read_data);
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
/*LGE_CHANGE_S, modified power-up/down status for recovery, 2013-12-27, hyungtae.lee@lge.com*/
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
/*LGE_CHANGE_E, modified power-up/down status for recovery, 2013-12-27, hyungtae.lee@lge.com*/
	case CFG_SET_STOP_STREAM_SETTING: {
		struct msm_camera_i2c_reg_setting *stop_setting =
			&s_ctrl->stop_setting;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
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
		CDBG("%s: Exposure compensation Value is %d",__func__, ec_lev);
		hi707_set_exposure_compensation(s_ctrl, ec_lev);
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
		CDBG("%s: Effect mode is %d", __func__, effect_mode);
		hi707_set_effect(s_ctrl, effect_mode);
		break;
	}
	case CFG_SET_ANTIBANDING: {
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
		CDBG("%s: best shot mode is %d", __func__, bs_mode);
		hi707_set_scene_mode(s_ctrl, bs_mode);
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
		CDBG("%s: white balance is %d", __func__, wb_mode);
/* LGE_CHANGE_S, Fixes to add product_kor info capabilities for setting Hi707 of kor medel, 2014-03-11, dongsu.bag@lge.com */
		if(product_kor > 0){
		hi707_set_white_balance_mode_kr(s_ctrl, wb_mode);
	    }else{
	    hi707_set_white_balance_mode(s_ctrl, wb_mode);
	    }
/* LGE_CHANGE_E, Fixes to add product_kor info capabilities for setting Hi707 of kor medel, 2014-03-11, dongsu.bag@lge.com */
		break;
	}
	case CFG_SET_AEC_ROI:{
		int32_t coordinate;
		if (copy_from_user(&coordinate, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s: coordinate is %d", __func__, coordinate);
		rc = hi707_set_aec_roi_mode(s_ctrl, coordinate);
		break;
	}
	case CFG_SET_AWB_LOCK: {
		int32_t awb_lock;
		if (copy_from_user(&awb_lock, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s: awb_lock is %d", __func__, awb_lock);
		rc = hi707_set_awb_lock_mode(s_ctrl, awb_lock);
		break;
	}
    case CFG_SET_AEC_LOCK:{
    	int32_t aec_lock;
		if (copy_from_user(&aec_lock, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		CDBG("%s: aec_lock is %d", __func__, aec_lock);
		rc = hi707_set_aec_lock_mode(s_ctrl, aec_lock);
		break;
	}
/*LGE_CHANGE_S, fixed Fps setting of soc sensor for VT mode, 2014-01-27, dongsu.bag@lge.com*/
	case CFG_SET_INIT_SETTING_VT:{
		mCurrentFpsMode = 4; /* LGE_CHANGE, init current fps mode for vt setting , 2014-02-17, dongsu.bag@lge.com */
/* LGE_CHANGE_S, Fix for Dual Camera Module of HI707, 2014-02-28, dongsu.bag@lge.com */
		pr_err("%s - vt init", __func__);
		if( vt_cam_id_value == HI707_LGIT )
		{
		pr_err("%s - vt lgit init", __func__);
		hi707_i2c_write_table(s_ctrl,
		&hi707_lgit_recommend_vt_settings[0],
		ARRAY_SIZE(hi707_lgit_recommend_vt_settings));
		}else{
		pr_err("%s - vt cowell init", __func__);
		hi707_i2c_write_table(s_ctrl,
		&hi707_cowell_recommend_vt_settings[0],
		ARRAY_SIZE(hi707_cowell_recommend_vt_settings));
			}
/* LGE_CHANGE_E, Fix for Dual Camera Module of HI707, 2014-02-28, dongsu.bag@lge.com */
		break;
		}
	case CFG_SET_FRAMERATE_FOR_SOC:{
		struct msm_fps_range_setting *framerate;
		if (copy_from_user(&framerate, (void *)cdata->cfg.setting, sizeof(struct msm_fps_range_setting))) {
			rc = -EFAULT;
			break;
		}
		pr_err("%s - min_fps:%d, max_fps:%d",__func__,framerate->min_fps, framerate->max_fps);
		hi707_set_framerate_for_soc(s_ctrl, framerate);
		break;
	}
/*LGE_CHANGE_E, fixed Fps setting of soc sensor for VT mode, 2014-01-27, dongsu.bag@lge.com*/
	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}

static struct msm_sensor_fn_t hi707_sensor_func_tbl = {
	.sensor_config = hi707_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = hi707_sensor_match_id,
};

static struct msm_sensor_ctrl_t hi707_s_ctrl = {
	.sensor_i2c_client = &hi707_sensor_i2c_client,
	//LGE_CHANGE_S, jongkwon.chae, 2014-05-29, To separate power settings depending on HW revisions.
	//.power_setting_array.power_setting = hi707_power_setting,
	//.power_setting_array.size = ARRAY_SIZE(hi707_power_setting),
	//LGE_CHANGE_E, jongkwon.chae, 2014-05-29, To separate power settings depending on HW revisions.
	.msm_sensor_mutex = &hi707_mut,
	.sensor_v4l2_subdev_info = hi707_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(hi707_subdev_info),
	.func_tbl = &hi707_sensor_func_tbl,
};

module_init(hi707_init_module);
module_exit(hi707_exit_module);
MODULE_DESCRIPTION("Hynix VGA YUV sensor driver");
MODULE_LICENSE("GPL v2");


