/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
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

#include <linux/init.h>
#include <linux/ioport.h>
#include <mach/board.h>
#include <mach/gpio.h>
#include <mach/gpiomux.h>
#include <mach/board_lge.h>
#include <mach/socinfo.h>

/* Pin MUX Setting for e7wifi project */
#ifdef CONFIG_MACH_LGE
/* Reserved Pin Setting */

// GPIO related function <<0.Resreved Pin>>
#define MSM8x26_GPIO_END 117
static int gpio_reserved_pin_rev_A[] = {
	1, 2, 3, 5, 12, 13, 14, 15, 24, 34, 35, 36, 38, 45, 46, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 61, 63, 64, 65, 67, 73, 74, 75, 76, 77, 78, 79, 80, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 104, 107, 109, 115, 116,
	MSM8x26_GPIO_END // This is included to notify the end of reserved GPIO configuration.
};
static int gpio_reserved_pin_rev_10[] = {
	1, 2, 3, 5, 12, 13, 14, 15, 24, 34, 35, 36, 38, 45, 46, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 61, 63, 64, 65, 67, 73, 74, 75, 76, 77, 78, 79, 80, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 104, 107, 109, 115, 116,
	MSM8x26_GPIO_END // This is included to notify the end of reserved GPIO configuration.
};

static struct gpiomux_setting reserved_pin_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir  = GPIOMUX_IN,
};

static struct msm_gpiomux_config gpio_func_reserved_pin_config __initdata = {
	.gpio = 0,
	.settings = {
		[GPIOMUX_SUSPENDED] = &reserved_pin_cfg,
		[GPIOMUX_ACTIVE] = &reserved_pin_cfg,
	},
};

// GPIO related function <<1.SENSOR>>
//GPIO[004] HALLIC_INT
//GPIO[066] COMPASS_INT
//GPIO[069] ACCEL_INT
static struct gpiomux_setting sensor_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct msm_gpiomux_config gpio_func_sensor_configs[] __initdata = {
	{
		.gpio = 66,		// COMPASS_INT
		.settings = {
			[GPIOMUX_SUSPENDED] = &sensor_sus_cfg,
		},
	},
	{
		.gpio = 69,		// ACCEL_INT
		.settings = {
			[GPIOMUX_SUSPENDED] = &sensor_sus_cfg,
		},
	},
};
static struct gpiomux_setting hall_ic_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

static struct msm_gpiomux_config msm_hall_ic_configs[] __initdata = {
	{
		.gpio = 4,
		.settings = {
			[GPIOMUX_SUSPENDED] = &hall_ic_sus_cfg,
		},
	},
};

// GPIO related function <<2.I2C>>
//GPIO[006] SENSORS0_I2C_SDA
//GPIO[007] SENSORS0_I2C_SCL
//GPIO[010] COMMON_I2C_SDA
//GPIo[011] COMMON_I2C_SCL
//GPIO[018] TOUCH_I2C_SDA
//GPIO[019] TOUCH_I2C_SCL
static struct gpiomux_setting gpio_i2c_config = {
	.func = GPIOMUX_FUNC_3,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct msm_gpiomux_config gpio_func_i2c_configs[] __initdata = {
	{
		.gpio	   = 6,		/* BLSP1 QUP2 I2C_SDA */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		},
	},
	{
		.gpio	   = 7,		/* BLSP1 QUP2 I2C_SCL */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		},
	},
	{
		.gpio      = 10,         /* BLSP1 QUP3 I2C_SDA */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		},
	},
	{
		.gpio      = 11,         /* BLSP1 QUP3 I2C_SCL */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		},
	},
	{
		.gpio      = 18,	/* BLSP1 QUP5 I2C_SDA */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		},
	},
	{
		.gpio      = 19,	/* BLSP1 QUP5 I2C_SCL */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		},
	},
};

// GPIO related function <<3.UART>>
//Need to set GPIO[008] MSM_UART_TX
//Need to set GPIO[009] MSM_UART_RX
static struct gpiomux_setting gpio_uart_config = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_16MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_HIGH,
};

static struct msm_gpiomux_config gpio_func_uart_configs[] __initdata = {
	{
		.gpio      = 8,                 /* BLSP2 UART TX */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_uart_config,
		},
	},
	{
		.gpio      = 9,                 /* BLSP2 UART RX */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_uart_config,
		},
	},
};

// GPIO related function <<4.TOUCH>>
//GPIO[016] TOUCH_LDO2_EN
//GPIO[017] TOUCH_INT
//GPIO[060] TOUCH_RESET
//GPIO[112] TOUCH_LDO1_EN
//GPIO[116] TOUCH_ID
static struct gpiomux_setting atmel_chg_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting atmel_reset_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_OUT_LOW,
};

static struct gpiomux_setting atmel_reset_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_OUT_LOW,
};

static struct gpiomux_setting touch_ldo_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_OUT_LOW,
};

static struct gpiomux_setting touch_ldo_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_OUT_LOW,
};

/* Applied EVBD or Above changes */
static struct msm_gpiomux_config msm_atmel_configs[] __initdata = {
	{
		.gpio = 16, /* GPIO[016] TOUCH_LDO2_EN */
		.settings = {
			[GPIOMUX_ACTIVE] = &touch_ldo_act_cfg,
			[GPIOMUX_SUSPENDED] = &touch_ldo_sus_cfg,
		},
	},
	{
		.gpio = 17,
		.settings = {
			[GPIOMUX_ACTIVE] = &atmel_chg_cfg,
			[GPIOMUX_SUSPENDED] = &atmel_chg_cfg,
		},
	},
	{
		.gpio = 60,
		.settings = {
			[GPIOMUX_ACTIVE] = &atmel_reset_act_cfg,
			[GPIOMUX_SUSPENDED] = &atmel_reset_sus_cfg,
		},
	},
	{
		.gpio = 112, /* GPIO[016] TOUCH_LDO1_EN */
		.settings = {
			[GPIOMUX_ACTIVE] = &touch_ldo_act_cfg,
			[GPIOMUX_SUSPENDED] = &touch_ldo_sus_cfg,
		},
	},
};

// GPIO related function <<5.NFC>>
//Need to set GPIO[020] NFC_VEN
//Need to set GPIO[021] NFC_IRQ
//Need to set GPIO[022] NFC_MODE
/*  LGE_CHANGE_S, [NFC][garam.kim@lge.com], NFC Bring up*/
#ifdef CONFIG_LGE_NFC_PN547_C2
static struct gpiomux_setting nfc_pn547_ven_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};

static struct gpiomux_setting nfc_pn547_irq_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting nfc_pn547_mode_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_HIGH,
};

static struct msm_gpiomux_config msm_nfc_configs[] __initdata = {
	{
		/* VEN */
		.gpio      = 20,
		.settings = {
			[GPIOMUX_ACTIVE]    = &nfc_pn547_ven_cfg,
			[GPIOMUX_SUSPENDED] = &nfc_pn547_ven_cfg,
		},
	},
	{
		/* IRQ */
		.gpio      = 21,
		.settings = {
			[GPIOMUX_ACTIVE]    = &nfc_pn547_irq_cfg,
			[GPIOMUX_SUSPENDED] = &nfc_pn547_irq_cfg,
		},
	},
	{
		/* MODE *//* WAKE */
		.gpio      = 22,
		.settings = {
			[GPIOMUX_ACTIVE]    = &nfc_pn547_mode_cfg,
			[GPIOMUX_SUSPENDED] = &nfc_pn547_mode_cfg,
		},
	},
};
#endif
/*  LGE_CHANGE_E, [NFC][garam.kim@lge.com], NFC Bring up*/

// GPIO related function <<6.LCD>>
// GPIO[111] DSV_ENP
// GPIO[103] DSV_ENN
// GPIO[025] LCD_RESX
// GPIO[032] LCD_BL_EN
static struct gpiomux_setting lcd_settings[] = {
	{
		.func = GPIOMUX_FUNC_GPIO,		//active [0]
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_UP,
		.dir = GPIOMUX_OUT_HIGH,
	},

	{
		.func = GPIOMUX_FUNC_GPIO,		//suspend [1]
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_DOWN,
	},
};

static struct gpiomux_setting bl_settings[] = {
	{
		.func = GPIOMUX_FUNC_2,			//PWM active [0]
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},


	{
		.func = GPIOMUX_FUNC_GPIO,		//active [1]
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_UP,
		.dir = GPIOMUX_OUT_HIGH,
	},

	{
		.func = GPIOMUX_FUNC_GPIO,		//suspend [2]
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_DOWN,
		.dir = GPIOMUX_IN,
	},
};
static struct msm_gpiomux_config msm_lcd_configs[] __initdata = {
	{
		.gpio = 111,			//DSV_ENP
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcd_settings[0],
			[GPIOMUX_SUSPENDED] = &lcd_settings[1],
		},
	},
	{
		.gpio = 103,			//DSV_ENN
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcd_settings[0],
			[GPIOMUX_SUSPENDED] = &lcd_settings[1],
		},
	},
	{
		.gpio = 25,			//LCD_RESX
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcd_settings[0],
			[GPIOMUX_SUSPENDED] = &lcd_settings[1],
		},
	}
};

static struct msm_gpiomux_config msm_bl_configs[] __initdata = {
	{
		.gpio = 32,			//LCD_BL_EN
		.settings = {
			[GPIOMUX_ACTIVE]    = &bl_settings[1],
			[GPIOMUX_SUSPENDED] = &bl_settings[2],
		},
	}
};

// GPIO related function <<7.CAMERA>>
//GPIO[026] MAIN_CAM0_MCLK
//GPIO[027] VT_CAM_MCLK
//GPIO[028] VT_CAM_RESET_N
//GPIO[029] CAM0_I2C_SDA
//GPIO[030] CAM0_I2C_SCL
//+GPIO[036] MAIN_CAM0_PWDN  - using for 2.8V LDO en at Rev A
//Deleted in Rev A +GPIO[092] MAIN_CAM0_WP_N
//Need to set +GPIO[097] VT_CAM_PWDN
//Need to set +GPIO[098] MAIN_CAM0_RESET_N
// Added in Rev A GPIO[062] LDO1_EN
// Added in Rev A GPIO[110] LDO2_EN
// Added in Rev A GPIO[113] LDO3_EN
static struct gpiomux_setting cam_settings[] = {
	{
		.func = GPIOMUX_FUNC_1, /*active 1*/ /* 0 */
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_1, /*suspend*/ /* 1 */
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_DOWN,
	},

	{
		.func = GPIOMUX_FUNC_1, /*i2c suspend*/ /* 2 */
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_KEEPER,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, /*active 0*/ /* 3 */
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, /*suspend 0*/ /* 4 */
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_DOWN,
	},
};

static struct gpiomux_setting gpio_suspend_config[] = {
	{
		.func = GPIOMUX_FUNC_GPIO,  /* IN-NP */
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},
	{
		.func = GPIOMUX_FUNC_GPIO,  /* O-LOW */
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
		.dir = GPIOMUX_OUT_LOW,
	},
};

/* LGE_CHANGE_S, Change CAM0_STANDBY_N 115 to 36 for RevA, 2014.02.18 sujeong.kwon@lge.com */
static struct msm_gpiomux_config msm_sensor_configs[] __initdata = {
	{
		.gpio = 26, /* CAM_MCLK0 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &cam_settings[1],
		},
	},
	{
		.gpio = 27, /* CAM_MCLK1 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &cam_settings[1],
		},
	},
	{
		.gpio = 114, /* CAM0_RST_N */
		.settings = {
			[GPIOMUX_ACTIVE]	= &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &cam_settings[4],
		},
	},
	{
		.gpio = 28, /* CAM1_RST_N */
		.settings = {
			[GPIOMUX_ACTIVE]	= &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &cam_settings[4],
		},
	},
	{
		.gpio = 29, /* CCI_I2C_SDA0 */
		.settings = {
			[GPIOMUX_ACTIVE]	= &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[0],
		},
	},
	{
		.gpio = 30, /* CCI_I2C_SCL0 */
		.settings = {
			[GPIOMUX_ACTIVE]	= &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[0],
		},
	},
	{
		.gpio = 36, //115, /* CAM0_STANDBY_N */
		.settings = {
			[GPIOMUX_ACTIVE]	= &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &cam_settings[4],
		},
	},
	{
		.gpio = 31, /* CAM1_STANDBY_N */  /*  IO, VT_CAM_LDO_EN@REV1.0 */
		.settings = {
			[GPIOMUX_ACTIVE]	= &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &cam_settings[4],
		},
	},
	{
		.gpio = 62, /* LDO1_EN */
		.settings = {
			[GPIOMUX_ACTIVE]	= &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &cam_settings[4],
		},
	},
	{
		.gpio = 113, /* LDO3_EN */
		.settings = {
			[GPIOMUX_ACTIVE]	= &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &cam_settings[4],
		},
	},
};
/* LGE_CHANGE_S, Change CAM0_STANDBY_N 115 to 36 for RevA, 2014.02.18 sujeong.kwon@lge.com */


// GPIO related function <<8.FLASH LED>>

// GPIO related function <<9.IRRC>>
// GPIO[033] D_IRRC_TXD
static struct gpiomux_setting irrc_clk_config = {
	.func = GPIOMUX_FUNC_3,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_DOWN,
};
static struct msm_gpiomux_config gpio_func_irrc_configs[] __initdata = {
	{
		.gpio      = 33,
		.settings = {
			[GPIOMUX_ACTIVE]    = &irrc_clk_config,
			[GPIOMUX_SUSPENDED] = &irrc_clk_config,
		},
	},
};

// GPIO related function <<10.AUDIO>>
static struct msm_gpiomux_config gpio_func_audio_configs[] __initdata = {
};

// GPIO related function <<11.SD CARD>>
//GPIO[037] SD_CARD_DET - Change to SD_CARD_DET_N in Rev A
static struct gpiomux_setting sd_card_det_active_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting sd_card_det_sleep_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

static struct msm_gpiomux_config sd_card_det __initdata = {
	.gpio = 37,
	.settings = {
		[GPIOMUX_ACTIVE]    = &sd_card_det_active_config,
		[GPIOMUX_SUSPENDED] = &sd_card_det_sleep_config,
	},
};

// GPIO related function <<12.BATTERY>>


// GPIO related function <<13.BT>>
//Need to set GPIO[039] BT_SSBI
//Need to set +GPIO[047] BT_CTL
//Need to set +GPIO[048] BT_DATA
static struct msm_gpiomux_config gpio_func_bt_configs[] __initdata = {
};

// GPIO related function <<14.WIFI>>
//GPIO[040] WLAN_DATA2
//GPIO[041] WLAN_DATA1
//GPIO[042] WLAN_DATA0
//GPIO[043] WLAN_SET
//GPIO[044] WLAN_CLK
static struct gpiomux_setting wcnss_5wire_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv  = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting wcnss_5wire_active_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv  = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct msm_gpiomux_config wcnss_5wire_interface[] = {
	// ************ Need to Verify : copied from QCT Base Setting  ****************** //
	{
		.gpio = 40,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 41,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 42,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 43,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 44,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
};

// GPIO related function <<15.FM>>


// GPIO related function <<16.SIM>>


// GPIO related function <<17.SLIMBUS>>
//Need to set GPIO[070] SLIMBUS_CLK
//Need to set GPIO[071] SLIMBUS_DATA
static struct gpiomux_setting slimbus = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_KEEPER,
};

static struct msm_gpiomux_config gpio_func_slimbus_configs[] __initdata = {
	{
		.gpio   = 70,           /* slimbus clk */
		.settings = {
			[GPIOMUX_SUSPENDED] = &slimbus,
		},
	},
	{
		.gpio   = 71,           /* slimbus data */
		.settings = {
			[GPIOMUX_SUSPENDED] = &slimbus,
		},
	},
};

// GPIO related function <<18.RF>>
//Need to set +GPIO[105] GNSS_LNA_EN
static struct msm_gpiomux_config gpio_func_rf_configs[] __initdata = {
};

// GPIO related function <<19.KEY PAD>>
//GPIO[106] VOLUME_UP
//GPIO[108] VOLUME_DOWN
static struct gpiomux_setting gpio_keys_active = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting gpio_keys_suspend = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct msm_gpiomux_config msm_keypad_configs[] __initdata = {
	{
		.gpio = 106,
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_keys_active,
			[GPIOMUX_SUSPENDED] = &gpio_keys_suspend,
		},
	},
	{
		.gpio = 108,
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_keys_active,
			[GPIOMUX_SUSPENDED] = &gpio_keys_suspend,
		},
	},
};
#endif

// GPIO related function <<20.LOGIC>>
static struct msm_gpiomux_config gpio_func_logic_configs[] __initdata = {
};


void __init msm8226_init_gpiomux(void)
{
	int rc;
#ifdef CONFIG_MACH_LGE
	int gpio_index = 0;
	hw_rev_type hw_rev;
	hw_rev = lge_get_board_revno();
#endif

	// Device Tree Initailize
	rc = msm_gpiomux_init_dt();
	if (rc) {
		pr_err("%s failed %d\n", __func__, rc);
		return;
	}
#ifdef CONFIG_MACH_LGE

	//--------------------------------------------
	// MSM8X26 GPIO Confiuration via W7 Rev 0
	//--------------------------------------------
	// GPIO related function <<0.Resreved Pin>>
	// GPIO related function <<1.SENSOR>>
	// GPIO related function <<2.I2C>>
	// GPIO related function <<3.UART>>
	// GPIO related function <<4.TOUCH>>
	// GPIO related function <<5.NFC>>
	// GPIO related function <<6.LCD>>
	// GPIO related function <<7.CAMERA>>
	// GPIO related function <<8.FLASH LED>>
	// GPIO related function <<9.IRRC>>
	// GPIO related function <<10.AUDIO>>
	// GPIO related function <<11.SD CARD>>
	// GPIO related function <<12.BATTERY>>
	// GPIO related function <<13.BT>>
	// GPIO related function <<14.WIFI>>
	// GPIO related function <<15.FM>>
	// GPIO related function <<16.SIM>>
	// GPIO related function <<17.SLIMBUS>>
	// GPIO related function <<18.RF>>
	// GPIO related function <<19.KEY PAD>>
	// GPIO related function <<20.LOGIC>>

	// GPIO related function <<1.SENSOR>>
	msm_gpiomux_install(gpio_func_sensor_configs,
			ARRAY_SIZE(gpio_func_sensor_configs));

	msm_gpiomux_install(msm_hall_ic_configs,
			ARRAY_SIZE(msm_hall_ic_configs));

	// GPIO related function <<2.I2C>>
	msm_gpiomux_install(gpio_func_i2c_configs,
			ARRAY_SIZE(gpio_func_i2c_configs));

	// GPIO related function <<3.UART>>
	msm_gpiomux_install(gpio_func_uart_configs,
			ARRAY_SIZE(gpio_func_uart_configs));

	// GPIO related function <<4.TOUCH>>
	msm_gpiomux_install(msm_atmel_configs, ARRAY_SIZE(msm_atmel_configs));

	// GPIO related function <<5.NFC>>
	/*	LGE_CHANGE_S, [NFC][garam.kim@lge.com], NFC Bring up */
#ifdef CONFIG_LGE_NFC_PN547_C2
	msm_gpiomux_install(msm_nfc_configs, ARRAY_SIZE(msm_nfc_configs));
#endif
	/*	LGE_CHANGE_E, [NFC][garam.kim@lge.com], NFC Bring up */


	// GPIO related function <<6.LCD>>
	msm_gpiomux_install_nowrite(msm_lcd_configs, ARRAY_SIZE(msm_lcd_configs));
	msm_gpiomux_install_nowrite(msm_bl_configs, ARRAY_SIZE(msm_bl_configs));

	// GPIO related function <<7.CAMERA>
	msm_gpiomux_install(msm_sensor_configs, ARRAY_SIZE(msm_sensor_configs));

#if 0
/* LGE_CHANGE_S, Add gpiomux for ex-ldo used gpio, 2013-09-04, hyungtae.lee@lge.com */
	if(hw_rev == HW_REV_0) {
		msm_gpiomux_install(msm_sensor_configs, ARRAY_SIZE(msm_sensor_configs));
		printk(KERN_ERR " [Camera] below HW_REV_0 is using power source from PM\n");
	}
	else if(hw_rev == HW_REV_A) {
		msm_gpiomux_install(msm_sensor_configs_rev_a, ARRAY_SIZE(msm_sensor_configs_rev_a));
		printk(KERN_ERR " [Camera] greater than HW_REV_A is using power source from Ex-LDO used GPIO\n");
	}
	else if(hw_rev == HW_REV_B) {
		msm_gpiomux_install(msm_sensor_configs_rev_b, ARRAY_SIZE(msm_sensor_configs_rev_b));
		printk(KERN_ERR " [Camera] In greater than HW_REV_B, MAIN_CAM0_RESET_N has been changed from GPIO_98 to GPIO_114\n");
	}
	else {
		msm_gpiomux_install(msm_sensor_configs_rev_c, ARRAY_SIZE(msm_sensor_configs_rev_c));
		printk(KERN_ERR " [Camera] In greater than HW_REV_C, GPIO_110 is used for HI707\n");
	}
/* LGE_CHANGE_E, Add gpiomux for ex-ldo used gpio, 2013-09-04, hyungtae.lee@lge.com */
#endif

	// GPIO related function <<8.FLASH LED>>
	msm_gpiomux_install(gpio_func_irrc_configs,
			ARRAY_SIZE(gpio_func_irrc_configs));

	// GPIO related function <<10.AUDIO>>
	msm_gpiomux_install(gpio_func_audio_configs,
			ARRAY_SIZE(gpio_func_audio_configs));

	// GPIO related function <<11.SD CARD>>
	msm_gpiomux_install(&sd_card_det, 1);

	// GPIO related function <<12.BATTERY>>


	// GPIO related function <<13.BT>>
	msm_gpiomux_install(gpio_func_bt_configs,
			ARRAY_SIZE(gpio_func_bt_configs));

	// GPIO related function <<14.WIFI>>
	msm_gpiomux_install(wcnss_5wire_interface,
			ARRAY_SIZE(wcnss_5wire_interface));

	// GPIO related function <<15.FM>>


	// GPIO related function <<16.SIM>>


	// GPIO related function <<17.SLIMBUS>>
	msm_gpiomux_install(gpio_func_slimbus_configs,
			ARRAY_SIZE(gpio_func_slimbus_configs));

	// GPIO related function <<18.RF>>
	msm_gpiomux_install(gpio_func_rf_configs,
			ARRAY_SIZE(gpio_func_rf_configs));

	// GPIO related function <<19.KEY PAD>>
	msm_gpiomux_install(msm_keypad_configs,
			ARRAY_SIZE(msm_keypad_configs));

	// GPIO related function <<20.LOGIC>>
	msm_gpiomux_install(gpio_func_logic_configs,
			ARRAY_SIZE(gpio_func_logic_configs));


	/* Moved from the top of this function */
	// GPIO related function <<0.Resreved Pin>>
	switch ( hw_rev ){
		case HW_REV_0 :
		case HW_REV_A :
		case HW_REV_B :
			for ( gpio_index = 0 ; gpio_reserved_pin_rev_A[gpio_index] < MSM8x26_GPIO_END ; gpio_index++ ){
				gpio_func_reserved_pin_config.gpio = gpio_reserved_pin_rev_A[gpio_index];
				msm_gpiomux_install(&gpio_func_reserved_pin_config, 1);
			}
			break;
		case HW_REV_C :
		case HW_REV_D :
		case HW_REV_E :
		case HW_REV_1_0 :
		case HW_REV_1_1 :
		default :
			for ( gpio_index = 0 ; gpio_reserved_pin_rev_10[gpio_index] < MSM8x26_GPIO_END ; gpio_index++ ){
				gpio_func_reserved_pin_config.gpio = gpio_reserved_pin_rev_10[gpio_index];
				msm_gpiomux_install(&gpio_func_reserved_pin_config, 1);
				}
			break;
	}
#endif
}


