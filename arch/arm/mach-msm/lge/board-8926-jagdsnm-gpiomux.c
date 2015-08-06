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

#ifndef CONFIG_MACH_LGE
static struct gpiomux_setting usb_otg_sw_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.dir = GPIOMUX_OUT_LOW,
};

static struct msm_gpiomux_config usb_otg_sw_configs[] __initdata = {
	{
		.gpio = 67,
		.settings = {
			[GPIOMUX_SUSPENDED] = &usb_otg_sw_cfg,
		},
	},
};
#endif


#ifdef CONFIG_MACH_LGE /* JAGUAR CHINA GPIO Setting */
/* Reserved Pin Setting */

// GPIO related function <<0.Resreved Pin>>
#define MSM8x26_GPIO_END 121
static int gpio_reserved_pin_rev_0[] = {
	0, 1, 5, 60, 75, 76, 79, 80, 86, 88, 107, 115, 116, 117, 118,
	MSM8x26_GPIO_END // This is included to notify the end of reserved GPIO configuration.
	};
static int gpio_reserved_pin_rev_a[] = {
	0, 1, 2, 3, 5, 31, 34, 60, 67, 75, 76, 79, 80, 86, 88, 107, 112, 115, 116, 117, 118,
	MSM8x26_GPIO_END // This is included to notify the end of reserved GPIO configuration. add 2,3,31,34,67,112
	};
static int gpio_reserved_pin_rev_c[] = {
	0, 1, 2, 3, 5, 31, 34, 67, 75, 76, 79, 80, 86, 88, 110, 111, 112, 115, 116, 117, 118,
	MSM8x26_GPIO_END // This is included to notify the end of reserved GPIO configuration. delete 60, 107 add 110, 111
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

// GPIO related function <<1-Rev0.SENSOR>>
//Need to set GPIO[004] HALLIC_INT
//Need to set GPIO[066] COMPASS_RESET
//Need to set GPIO[067] FUEL_GAUGE_INT
//Need to set GPIO[068] CODEC_INT
//Need to set GPIO[069] ACCEL_INT
//Need to set GPIO[119] PROXIMITY_INT

static struct gpiomux_setting gpio_int_active_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting gpio_reset_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_HIGH,
};

static struct msm_gpiomux_config gpio_func_sensor_configs[] __initdata = {
	{
		.gpio = 4,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_int_active_config,
			[GPIOMUX_ACTIVE] = &gpio_int_active_config,
		},
	},
	{
		.gpio = 66,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_int_active_config,
			[GPIOMUX_ACTIVE] = &gpio_reset_act_cfg,
		},
	},
	{
		.gpio = 67,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_int_active_config,
			[GPIOMUX_ACTIVE] = &gpio_int_active_config,
		},
	},
	{
		.gpio = 68,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_int_active_config,
			[GPIOMUX_ACTIVE] = &gpio_int_active_config,
		},
	},
	{
		.gpio = 69,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_int_active_config,
			[GPIOMUX_ACTIVE] = &gpio_int_active_config,
		},
	},
	{
		.gpio = 119,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_int_active_config,
			[GPIOMUX_ACTIVE] = &gpio_int_active_config,
		},
	},
};

// GPIO related function <<1-RevA.SENSOR>>
//Need to set GPIO[004] HALLIC_INT
//Need to set GPIO[066] COMPASS_RESET
//Need to set GPIO[068] CODEC_INT
//Need to set GPIO[069] ACCEL_INT
//Need to set GPIO[119] PROXIMITY_INT


static struct msm_gpiomux_config gpio_func_sensor_configs1[] __initdata = {
	{
		.gpio = 4,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_int_active_config,
			[GPIOMUX_ACTIVE] = &gpio_int_active_config,
		},
	},
	{
		.gpio = 66,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_int_active_config,
			[GPIOMUX_ACTIVE] = &gpio_reset_act_cfg,
		},
	},
	{
		.gpio = 68,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_int_active_config,
			[GPIOMUX_ACTIVE] = &gpio_int_active_config,
		},
	},
	{
		.gpio = 69,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_int_active_config,
			[GPIOMUX_ACTIVE] = &gpio_int_active_config,
		},
	},
	{
		.gpio = 119,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_int_active_config,
			[GPIOMUX_ACTIVE] = &gpio_int_active_config,
		},
	},
};

// GPIO related function <<2-Rev0.I2C>>
//Need to set GPIO[002] GUAGE_I2C_SDA
//Need to set GPIO[003] GUAGE_I2C_SCL
//Need to set GPIO[006] SENSORS0_I2C_SDA
//Need to set GPIO[007] SENSORS0_I2C_SCL
//-GPIO[010] COMMON_I2C_SDA - moved to NFC
//-GPIO[011] COMMON_I2C_SCL - moved to NFC
//Need to set GPIO[018] TOUCH_I2C_SDA
//Need to set GPIO[019] TOUCH_I2C_SCL
static struct gpiomux_setting i2c_suspend_config = {
	.func = GPIOMUX_FUNC_3,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting i2c_active_config = {
	.func = GPIOMUX_FUNC_3,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
static struct msm_gpiomux_config gpio_func_i2c_configs[] __initdata = {
	{
		.gpio = 2,
		.settings = {
			[GPIOMUX_SUSPENDED] = &i2c_suspend_config,
			[GPIOMUX_ACTIVE] = &i2c_active_config,
		},
	},
	{
		.gpio = 3,
		.settings = {
			[GPIOMUX_SUSPENDED] = &i2c_suspend_config,
			[GPIOMUX_ACTIVE] = &i2c_active_config,
		},
	},
	{
		.gpio = 6,
		.settings = {
			[GPIOMUX_SUSPENDED] = &i2c_suspend_config,
		},
	},
	{
		.gpio = 7,
		.settings = {
			[GPIOMUX_SUSPENDED] = &i2c_suspend_config,
		},
	},
	{
		.gpio = 18,
		.settings = {
			[GPIOMUX_SUSPENDED] = &i2c_suspend_config,
			[GPIOMUX_ACTIVE] = &i2c_active_config,
		},
	},
	{
		.gpio = 19,
		.settings = {
			[GPIOMUX_SUSPENDED] = &i2c_suspend_config,
			[GPIOMUX_ACTIVE] = &i2c_active_config,
		},
	},
};

// GPIO related function <<2-RevA.I2C>>
//Need to set GPIO[006] SENSORS0_I2C_SDA
//Need to set GPIO[007] SENSORS0_I2C_SCL
//-GPIO[010] COMMON_I2C_SDA - moved to NFC
//-GPIO[011] COMMON_I2C_SCL - moved to NFC
//Need to set GPIO[018] TOUCH_I2C_SDA
//Need to set GPIO[019] TOUCH_I2C_SCL
static struct msm_gpiomux_config gpio_func_i2c_configs1[] __initdata = {
	{
		.gpio = 6,
		.settings = {
			[GPIOMUX_SUSPENDED] = &i2c_suspend_config,
		},
	},
	{
		.gpio = 7,
		.settings = {
			[GPIOMUX_SUSPENDED] = &i2c_suspend_config,
		},
	},
	{
		.gpio = 18,
		.settings = {
			[GPIOMUX_SUSPENDED] = &i2c_suspend_config,
			[GPIOMUX_ACTIVE] = &i2c_active_config,
		},
	},
	{
		.gpio = 19,
		.settings = {
			[GPIOMUX_SUSPENDED] = &i2c_suspend_config,
			[GPIOMUX_ACTIVE] = &i2c_active_config,
		},
	},
};



// GPIO related function <<3.UART>>
//Need to set GPIO[008] MSM_UART_TX
//Need to set GPIO[009] MSM_UART_RX
static struct msm_gpiomux_config gpio_func_uart_configs[] __initdata = {
};

// GPIO related function <<4.TOUCH>>
//Need to set GPIO[016] TOUCH_RESET_N
//Need to set GPIO[120] TOUCH_MAKER_ID
static struct gpiomux_setting touch_reset_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_HIGH,
};

static struct gpiomux_setting touch_reset_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting touch_int_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting touch_int_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting touch_id_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting touch_id_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

static struct msm_gpiomux_config msm_atmel_s336_configs[] __initdata = {
	{
		.gpio = 16,
		.settings = {
			[GPIOMUX_ACTIVE] = &touch_reset_act_cfg,
			[GPIOMUX_SUSPENDED] = &touch_reset_sus_cfg,
		},
	},
	{
		.gpio = 17,
		.settings = {
			[GPIOMUX_ACTIVE] = &touch_int_act_cfg,
			[GPIOMUX_SUSPENDED] = &touch_int_sus_cfg,
		},
	},
	{
		.gpio = 120,
		.settings = {
			[GPIOMUX_ACTIVE] = &touch_id_act_cfg,
			[GPIOMUX_SUSPENDED] = &touch_id_sus_cfg,
		},
	},
};

/* LGE_CHANGE_S, 2014/03/11, bonkang.koo@lge.com
 * [Touch] synaptics touch IC bring up*/

static struct gpiomux_setting synaptics_reset_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_HIGH,
};

static struct gpiomux_setting synaptics_reset_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting synaptics_int_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting synaptics_int_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting synaptics_id_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting synaptics_id_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};
static struct msm_gpiomux_config msm_synaptics_configs_rev_b[] __initdata = {
	{
	    .gpio = 16,
	    .settings = {
		[GPIOMUX_ACTIVE] = &synaptics_reset_act_cfg,
		[GPIOMUX_SUSPENDED] = &synaptics_reset_sus_cfg,
	    },
	},
	{
	    .gpio = 17,
	    .settings = {
		[GPIOMUX_ACTIVE] = &synaptics_int_act_cfg,
		[GPIOMUX_SUSPENDED] = &synaptics_int_sus_cfg,
	    },
	},
	{
	    .gpio = 120,
	    .settings = {
		[GPIOMUX_ACTIVE] = &synaptics_id_act_cfg,
		[GPIOMUX_SUSPENDED] = &synaptics_id_sus_cfg,
	    },
	},
};
/* LGE_CHANGE_E, 2014/03/11, bonkang.koo@lge.com
 * [Touch] synaptics touch IC bring up*/


// GPIO related function <<5.NFC>>
//+GPIO[010] COMMON_I2C_SDA
//+GPIO[011] COMMON_I2C_SCL
//Need to set GPIO[020] NFC_VEN
//Need to set GPIO[021] NFC_IRQ
//Need to set GPIO[022] NFC_MODE
/*  LGE_CHANGE_S, [NFC][garam.kim@lge.com], NFC Bring up*/
#ifdef CONFIG_LGE_NFC_PN547_C2
static struct gpiomux_setting nfc_pn547_sda_cfg = {
	.func = GPIOMUX_FUNC_3,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting nfc_pn547_scl_cfg = {
	.func = GPIOMUX_FUNC_3,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

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
		/* I2C SDA */
		.gpio      = 10,
		.settings = {
			[GPIOMUX_ACTIVE]    = &nfc_pn547_sda_cfg,
			[GPIOMUX_SUSPENDED] = &nfc_pn547_sda_cfg,
		},
	},
	{
		/* I2C SCL */
		.gpio      = 11,
		.settings = {
			[GPIOMUX_ACTIVE]    = &nfc_pn547_scl_cfg,
			[GPIOMUX_SUSPENDED] = &nfc_pn547_scl_cfg,
		},
	},
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

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
static struct gpiomux_setting sdc3_clk_actv_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting sdc3_cmd_data_0_3_actv_cfg = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting sdc3_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting sdc3_data_1_suspend_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct msm_gpiomux_config msm8226_sdc3_configs[] __initdata = {
	{
		/* DAT3 */
		.gpio      = 39,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdc3_cmd_data_0_3_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdc3_suspend_cfg,
		},
	},
	{
		/* DAT2 */
		.gpio      = 40,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdc3_cmd_data_0_3_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdc3_suspend_cfg,
		},
	},
	{
		/* DAT1 */
		.gpio      = 41,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdc3_cmd_data_0_3_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdc3_data_1_suspend_cfg,
		},
	},
	{
		/* DAT0 */
		.gpio      = 42,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdc3_cmd_data_0_3_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdc3_suspend_cfg,
		},
	},
	{
		/* CMD */
		.gpio      = 43,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdc3_cmd_data_0_3_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdc3_suspend_cfg,
		},
	},
	{
		/* CLK */
		.gpio      = 44,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sdc3_clk_actv_cfg,
			[GPIOMUX_SUSPENDED] = &sdc3_suspend_cfg,
		},
	},
};

static void msm_gpiomux_sdc3_install(void)
{
	msm_gpiomux_install(msm8226_sdc3_configs,
			    ARRAY_SIZE(msm8226_sdc3_configs));
}
#else
static void msm_gpiomux_sdc3_install(void) {}
#endif /* CONFIG_MMC_MSM_SDC3_SUPPORT */

/* LCD pin Setting */
// GPIO related function <<6.LCD>>
//Need to set GPIO[023] LCD_MAKER_ID
//Need to set GPIO[024] LCD_VSYNC
//Need to set GPIO[025] LCD_RESET
//Need to set GPIO[060] DSV_EN

static struct gpiomux_setting lcd_vsync_act_cfg[] = {
	{
		.func = GPIOMUX_FUNC_1, /*active 1*/ /* 0 */
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_NONE,
	},
	{
		.func = GPIOMUX_FUNC_1, /*suspend*/ /* 1 */
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
	},
};
static struct gpiomux_setting lcd_rst_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_HIGH,
};

static struct gpiomux_setting lcd_rst_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting lcd_id_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting lcd_id_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct msm_gpiomux_config msm_lcd_configs[] __initdata = {
	{
		.gpio = 25,             /* Initialize LCD_RESET */
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcd_rst_act_cfg,
			[GPIOMUX_SUSPENDED] = &lcd_rst_sus_cfg,
		},
	}
};

static struct msm_gpiomux_config msm_lcd_configs_rev_c[] __initdata = {
	{
		.gpio = 23,				/* Initialize LCD_MAKER_ID */
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcd_id_act_cfg,
			[GPIOMUX_SUSPENDED] = &lcd_id_sus_cfg,
		},
	},
	{
		.gpio = 24,				/* Initialize LCD_VSYNC */
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcd_vsync_act_cfg[0],
			[GPIOMUX_SUSPENDED] = &lcd_vsync_act_cfg[1],
		},
	},
	{
		.gpio = 25,				/* Initialize LCD_RESET */
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcd_rst_act_cfg,
			[GPIOMUX_SUSPENDED] = &lcd_rst_sus_cfg,
		},
	},
	{
		.gpio = 60,			/* Initialize DSV_EN */
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcd_rst_act_cfg,
			[GPIOMUX_SUSPENDED] = &lcd_rst_sus_cfg,
		},
	}
};

#ifdef CONFIG_TSPDRV /* for sm100 */
static struct gpiomux_setting vibrator_suspend_cfg = {
       .func = GPIOMUX_FUNC_GPIO,
       .drv = GPIOMUX_DRV_2MA,
       .pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting vibrator_active_cfg_gpio_pwm = {
       .func = GPIOMUX_FUNC_3, //2013-08-22 beekay.lee@lge.com For WX(MSM8x26). GPIO34 has alternative function 3(=GP1_CLK)
       .drv = GPIOMUX_DRV_2MA,
       .pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting vibrator_active_cfg_gpio_power = {
       .func = GPIOMUX_FUNC_GPIO,
       .drv = GPIOMUX_DRV_2MA,
       .pull = GPIOMUX_PULL_NONE,
};

static struct msm_gpiomux_config vibrator_configs[] = {
	{
		.gpio = 34,
		.settings = {
			[GPIOMUX_ACTIVE]    = &vibrator_active_cfg_gpio_pwm,
			[GPIOMUX_SUSPENDED] = &vibrator_suspend_cfg,
		},
	},
	{
		.gpio = 112,
		.settings = {
			[GPIOMUX_ACTIVE]    = &vibrator_active_cfg_gpio_power,
			[GPIOMUX_SUSPENDED] = &vibrator_suspend_cfg,
		},
    },
};
#endif

// GPIO related function <<7.CAMERA>>
//Need to set GPIO[026] MAIN_CAM0_MCLK
//Need to set GPIO[027] VT_CAM_MCLK
//Need to set GPIO[029] CAM0_I2C_SDA
//Need to set GPIO[030] CAM0_I2C_SCL
//Need to set GPIO[114] MAIN_CAM0_RESET_N
//Need to set GPIO[028] VT_CAM_RESET_N
//Need to set GPIO[062] LDO1_EN
//Need to set GPIO[113] LDO3_EN
//Need to set GPIO[036] VCM_EN
//Need to set GPIO[109] LDAF_EN
//Need to set GPIO[107] LDAF_INT

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

	{
		.func = GPIOMUX_FUNC_GPIO, /*active 2*/ /* 5 */ // for INPUT
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_UP,
		.dir = GPIOMUX_IN,
	},

	{
		.func = GPIOMUX_FUNC_GPIO, /*suspend 2*/ /* 6 */ // for INPUT
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
		.dir = GPIOMUX_IN,
	},
};

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
		.gpio = 29, /* CCI_I2C_SDA0 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[0],
		},
	},
	{
		.gpio = 30, /* CCI_I2C_SCL0 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[0],
		},
	},
#if 0
	{
		.gpio = 36, /* CAM1_STANDBY_N */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &cam_settings[4],
		},
	},
#endif
	{
		.gpio = 114, /* CAM1_RST_N */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &cam_settings[4],
		},
	},
#if 0
	{
		.gpio = 35, /* CAM2_STANDBY_N */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &cam_settings[4],
		},
	},
#endif
	{
		.gpio = 28, /* CAM2_RST_N */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &cam_settings[4],
		},
	},

	{
		.gpio = 62, /* LDO1_EN */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &cam_settings[4],
		},
	},
	{
		.gpio = 113, /* LDO3_EN */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &cam_settings[4],
		},
	},
	{
		.gpio = 36, /* VCM_EN */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &cam_settings[4],
		},
	},
	{
		.gpio = 109, /* LDAF_EN */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &cam_settings[4],
		},
	},
	{
		.gpio = 107, /* LDAF_INT */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[5],
			[GPIOMUX_SUSPENDED] = &cam_settings[6],
		},
	},
};

// GPIO related function <<8.FLASH LED>>
//VLED_FLASH_PM8926
static struct msm_gpiomux_config gpio_func_flash_led_configs[] __initdata = {
};

// GPIO related function <<10.AUDIO>>
//Need to set GPIO[035] EAR_SENSE
//Need to set GPIO[067] EAR_KEY
//Need to set GPIO[068] CODEC_INT_N
//Need to set GPIO[072] CODEC_RESET_N
//Need to set GPIO[034] LIN_MOTOR_PWM (Rev. A)


// GPIO related function <<11.SD CARD>>
//Need to set GPIO[037] SD_CARD_DET
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
//Need to set GPIO[038] BAT_ID
static struct msm_gpiomux_config gpio_func_battery_configs[] __initdata = {
};

// GPIO related function <<13.BT>>
//Need to set GPIO[039] BT_SSBI
//Need to set +GPIO[047] BT_CTL
//Need to set +GPIO[048] BT_DATA
/* LGE_CHANGE_S, [BT][teddy.ju@lge.com], 2013-05-13 */
#ifdef CONFIG_LGE_BLUETOOTH
static struct gpiomux_setting bt_gpio_uart_active_config = {
    .func = GPIOMUX_FUNC_2,
    .drv = GPIOMUX_DRV_8MA,
    .pull = GPIOMUX_PULL_NONE, /* Should be PULL NONE */
};

static struct gpiomux_setting bt_gpio_uart_suspend_config = {
    .func = GPIOMUX_FUNC_GPIO,
    .drv = GPIOMUX_DRV_2MA,
    .pull = GPIOMUX_PULL_NONE, /* PULL Configuration */
};

static struct gpiomux_setting bt_rfkill_active_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv  = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting bt_rfkill_suspend_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv  = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting bt_host_wakeup_active_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting bt_host_wakeup_suspend_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting bt_wakeup_active_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting bt_wakeup_suspend_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
#if 0
static struct gpiomux_setting bt_pcm_active_config = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting bt_pcm_suspend_config = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};
#endif
static struct msm_gpiomux_config bt_msm_blsp_configs[] __initdata = {
	{
		.gpio = 12, /* BLSP4 UART3 TX */
		.settings = {
			[GPIOMUX_ACTIVE] = &bt_gpio_uart_active_config ,
			[GPIOMUX_SUSPENDED] = &bt_gpio_uart_suspend_config ,
		},
	},
	{
		.gpio = 13, /* BLSP4 UART3 RX */
		.settings = {
			[GPIOMUX_ACTIVE] = &bt_gpio_uart_active_config ,
			[GPIOMUX_SUSPENDED] = &bt_gpio_uart_suspend_config ,
		},
	},
	{
		.gpio = 14, /* BLSP4 UART3 CTS */
		.settings = {
			[GPIOMUX_ACTIVE] = &bt_gpio_uart_active_config ,
			[GPIOMUX_SUSPENDED] = &bt_gpio_uart_suspend_config ,
		},
	},
	{
		.gpio = 15, /* BLSP4 UART3 RFR */
		.settings = {
			[GPIOMUX_ACTIVE] = &bt_gpio_uart_active_config ,
			[GPIOMUX_SUSPENDED] = &bt_gpio_uart_suspend_config ,
		},
	},
};

static struct msm_gpiomux_config bt_rfkill_configs[] = {
	{
		.gpio = 45,
		.settings = {
			[GPIOMUX_ACTIVE]    = &bt_rfkill_active_config,
			[GPIOMUX_SUSPENDED] = &bt_rfkill_suspend_config,
		},
	},
};
static struct msm_gpiomux_config bt_host_wakeup_configs[] __initdata = {
	{
		.gpio = 48,
		.settings = {
			[GPIOMUX_ACTIVE]    = &bt_host_wakeup_active_config,
			[GPIOMUX_SUSPENDED] = &bt_host_wakeup_suspend_config,
		},
	},
};

static struct msm_gpiomux_config bt_wakeup_configs[] __initdata = {
	{
		.gpio = 47,
		.settings = {
			[GPIOMUX_ACTIVE]    = &bt_wakeup_active_config,
			[GPIOMUX_SUSPENDED] = &bt_wakeup_suspend_config,
		},
	},
};

static struct gpiomux_setting auxpcm_act_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting auxpcm_sus_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct msm_gpiomux_config bt_auxpcm_configs[] __initdata = {
	{
		.gpio = 49,
		.settings = {
			[GPIOMUX_SUSPENDED] = &auxpcm_sus_cfg,
			[GPIOMUX_ACTIVE] = &auxpcm_act_cfg,
		},
	},
	{
		.gpio = 50,
		.settings = {
			[GPIOMUX_SUSPENDED] = &auxpcm_sus_cfg,
			[GPIOMUX_ACTIVE] = &auxpcm_act_cfg,
		},
	},
	{
		.gpio = 51,
		.settings = {
			[GPIOMUX_SUSPENDED] = &auxpcm_sus_cfg,
			[GPIOMUX_ACTIVE] = &auxpcm_act_cfg,
		},
	},
	{
		.gpio = 52,
		.settings = {
			[GPIOMUX_SUSPENDED] = &auxpcm_sus_cfg,
			[GPIOMUX_ACTIVE] = &auxpcm_act_cfg,
		},
	},
};

#ifdef CONFIG_SND_BRCM_FM_RADIO
static struct gpiomux_setting mi2s_act_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting mi2s_sus_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct msm_gpiomux_config msm_pri_mi2s_configs[] __initdata = {
	{
		.gpio = 63,
		.settings = {
			[GPIOMUX_SUSPENDED] = &mi2s_sus_cfg,
			[GPIOMUX_ACTIVE] = &mi2s_act_cfg,
		},
	},
	{
		.gpio = 64,
		.settings = {
			[GPIOMUX_SUSPENDED] = &mi2s_sus_cfg,
			[GPIOMUX_ACTIVE] = &mi2s_act_cfg,
		},
	},
	{
		.gpio = 65,
		.settings = {
			[GPIOMUX_SUSPENDED] = &mi2s_sus_cfg,
			[GPIOMUX_ACTIVE] = &mi2s_act_cfg,
		},
	},
};
#endif

static void bluetooth_msm_gpiomux_install(void)
{
    /* UART */
    msm_gpiomux_install(bt_msm_blsp_configs, ARRAY_SIZE(bt_msm_blsp_configs));

    /* RFKILL */
    msm_gpiomux_install(bt_rfkill_configs, ARRAY_SIZE(bt_rfkill_configs));

    /* HOST WAKE-UP */
    msm_gpiomux_install(bt_host_wakeup_configs, ARRAY_SIZE(bt_host_wakeup_configs));

    /* BT WAKE-UP */
    msm_gpiomux_install(bt_wakeup_configs, ARRAY_SIZE(bt_wakeup_configs));

    /* PCM I/F */
    msm_gpiomux_install(bt_auxpcm_configs, ARRAY_SIZE(bt_auxpcm_configs));
}
#endif /* CONFIG_LGE_BLUETOOTH */
/* LGE_CHANGE_E, BT][teddy.ju@lge.com], 2013-05-13 */

// GPIO related function <<14.WIFI>>
//BCM4334

#if defined ( CONFIG_BCMDHD ) || defined ( CONFIG_BCMDHD_MODULE )

#else
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
#endif

// GPIO related function <<15.FM>>
//Need to set GPIO[063] FM_I2S_CLK
//Need to set GPIO[064] FM_I2S_FSYNC
//Need to set GPIO[065] FM_I2S_DATA_OUT
static struct msm_gpiomux_config gpio_func_fm_configs[] __initdata = {
};

// GPIO related function <<16.WLC>>
//Need to set GPIO[000] WLC_FULL_CHARGER
static struct msm_gpiomux_config gpio_func_wlc_configs[] __initdata = {
};

// GPIO related function <<17.SIM>>
//Need to set GPIO[057] UIM1_DATA
//Need to set GPIO[058] UIM1_CLK
//Need to set GPIO[059] UIM1_RESET_N
//Need to set GPIO[061] BATT_UIM_ALARM
static struct msm_gpiomux_config gpio_func_sim_configs[] __initdata = {
};

// GPIO related function <<18.SLIMBUS>>
//Need to set GPIO[070] SLIMBUS_CLK
//Need to set GPIO[071] SLIMBUS_DATA
static struct gpiomux_setting slimbus_act_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting slimbus_sus_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct msm_gpiomux_config gpio_func_slimbus_configs[] __initdata = {
	{
		.gpio = 70,
		.settings = {
			[GPIOMUX_ACTIVE]    = &slimbus_act_cfg,
			[GPIOMUX_SUSPENDED] = &slimbus_sus_cfg,
		},
	},
	{
		.gpio = 71,
		.settings = {
			[GPIOMUX_ACTIVE]    = &slimbus_act_cfg,
			[GPIOMUX_SUSPENDED] = &slimbus_sus_cfg,
		},
	},
};

// GPIO related function <<19.RF>>
//Need to set GPIO[073] RFFE2_CLK
//Need to set GPIO[074] RFFE2_DATA
//Need to set GPIO[077] PA0_ON_2
//Need to set GPIO[078] PA0_ON_3
//Need to set GPIO[079] MOBILE_SW_DET_0
//Need to set GPIO[080] MOBILE_SW_DET_1
//Need to set GPIO[081] PA0_R_0
//Need to set GPIO[082] PA0_R_1
//Need to set GPIO[087] TX_GTR_THRESH
//Need to set GPIO[091] PA1_ON_0
//Need to set GPIO[092] PA1_ON_1
//Need to set GPIO[093] PA1_R_0
//Need to set GPIO[099] RFFE1_CLK
//Need to set GPIO[100] RFFE1_DATA
//Need to set GPIO[101] WTR0_SSBI_PRX_DRX
//Need to set GPIO[102] WTR0_SSBI_TX_GNNS
//Need to set GPIO[103] WTR1_SSBI_PRX_DRX
//Need to set GPIO[104] WTR0_SSBI_TX_GNNS
//Need to set GPIO[105] GNSS_ELNA_EN
static struct msm_gpiomux_config gpio_func_rf_configs[] __initdata = {
};

// GPIO related function <<20.KEY PAD>>
//Need to set GPIO[106] KEYPAD_SENSE_0
//Need to set GPIO[108] KEYPAD_SENSE_2
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

// GPIO related function <<21.LOGIC>>
//Need to set GPIO[112] RGB_EN
static struct msm_gpiomux_config gpio_func_logic_configs[] __initdata = {
};
#endif

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
	// MSM8X26 GPIO Confiuration via JAGUAR
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
	// GPIO related function <<16.WLC>>
	// GPIO related function <<17.SIM>>
	// GPIO related function <<18.SLIMBUS>>
	// GPIO related function <<19.RF>>
	// GPIO related function <<20.KEY PAD>>
	// GPIO related function <<21.LOGIC>>

	// GPIO related function <<0.Resreved Pin>>
	switch ( hw_rev ){
		case HW_REV_0 :
			for ( gpio_index = 0 ; gpio_reserved_pin_rev_0[gpio_index] < MSM8x26_GPIO_END ; gpio_index++ ){
				gpio_func_reserved_pin_config.gpio = gpio_reserved_pin_rev_0[gpio_index];
				msm_gpiomux_install(&gpio_func_reserved_pin_config, 1);
				}
			break;
		case HW_REV_A :
		case HW_REV_B :
			for ( gpio_index = 0 ; gpio_reserved_pin_rev_a[gpio_index] < MSM8x26_GPIO_END ; gpio_index++ ){
				gpio_func_reserved_pin_config.gpio = gpio_reserved_pin_rev_a[gpio_index];
				msm_gpiomux_install(&gpio_func_reserved_pin_config, 1);
				}
			break;
		case HW_REV_C :
		case HW_REV_D :
		case HW_REV_E :
		case HW_REV_1_0 :
		case HW_REV_1_1 :
		default :
			for ( gpio_index = 0 ; gpio_reserved_pin_rev_c[gpio_index] < MSM8x26_GPIO_END ; gpio_index++ ){
				gpio_func_reserved_pin_config.gpio = gpio_reserved_pin_rev_c[gpio_index];
				msm_gpiomux_install(&gpio_func_reserved_pin_config, 1);
				}
			break;
	}

	// GPIO related function <<1.SENSOR>>
if (lge_get_board_revno() < HW_REV_A)
	msm_gpiomux_install(gpio_func_sensor_configs, ARRAY_SIZE(gpio_func_sensor_configs));
else
	msm_gpiomux_install(gpio_func_sensor_configs1, ARRAY_SIZE(gpio_func_sensor_configs1));

	// GPIO related function <<2.I2C>>
if (lge_get_board_revno() < HW_REV_A)
	msm_gpiomux_install(gpio_func_i2c_configs, ARRAY_SIZE(gpio_func_i2c_configs));
else
	msm_gpiomux_install(gpio_func_i2c_configs1, ARRAY_SIZE(gpio_func_i2c_configs1));

	// GPIO related function <<3.UART>>
	msm_gpiomux_install(gpio_func_uart_configs, ARRAY_SIZE(gpio_func_uart_configs));

	// GPIO related function <<4.TOUCH>>

/* LGE_CHANGE_S, 2014/03/11, bonkang.koo@lge.com
 * [Touch] synaptics touch IC bring up*/
if (hw_rev <= HW_REV_A) {
	msm_gpiomux_install(msm_atmel_s336_configs, ARRAY_SIZE(msm_atmel_s336_configs));
	printk(KERN_ERR "[Touch] HW_REV_0, HW_REV_A ATMEL configs \n");
}else{
	msm_gpiomux_install(msm_synaptics_configs_rev_b, ARRAY_SIZE(msm_synaptics_configs_rev_b));
	printk(KERN_ERR "[Touch] AFTER HW_REV_B, SYNAPTICS configs \n");
}
/* LGE_CHANGE_E, 2014/03/11, bonkang.koo@lge.com
 * [Touch] synaptics touch IC bring up*/

	// GPIO related function <<5.NFC>>
/*  LGE_CHANGE_S, [NFC][garam.kim@lge.com], NFC Bring up */
#ifdef CONFIG_LGE_NFC_PN547_C2
	msm_gpiomux_install(msm_nfc_configs, ARRAY_SIZE(msm_nfc_configs));
#endif
/*  LGE_CHANGE_E, [NFC][garam.kim@lge.com], NFC Bring up */

	// GPIO related function <<6.LCD>>
if (lge_get_board_revno() < HW_REV_C)
	msm_gpiomux_install_nowrite(msm_lcd_configs, ARRAY_SIZE(msm_lcd_configs));
else
    msm_gpiomux_install_nowrite(msm_lcd_configs_rev_c, ARRAY_SIZE(msm_lcd_configs_rev_c));
	
	// GPIO related function <<7.CAMERA>>
	msm_gpiomux_install(msm_sensor_configs, ARRAY_SIZE(msm_sensor_configs));

	// GPIO related function <<8.FLASH LED>>
	msm_gpiomux_install(gpio_func_flash_led_configs, ARRAY_SIZE(gpio_func_flash_led_configs));

	// GPIO related function <<10.AUDIO>>

	// GPIO related function <<11.SD CARD>>
	msm_gpiomux_install(&sd_card_det, 1);

	// GPIO related function <<12.BATTERY>>
	msm_gpiomux_install(gpio_func_battery_configs, ARRAY_SIZE(gpio_func_battery_configs));	
	
	// GPIO related function <<13.BT>>
	/* LGE_CHANGE_S, [BT][teddy.ju@lge.com], 2013-05-13 */
	#ifdef CONFIG_LGE_BLUETOOTH
		bluetooth_msm_gpiomux_install();
	#endif /* CONFIG_LGE_BLUETOOTH */
	/* LGE_CHANGE_E, [BT][teddy.ju@lge.com], 2013-05-13 */

	// GPIO related function <<14.WIFI>>
#if defined ( CONFIG_BCMDHD ) || defined ( CONFIG_BCMDHD_MODULE )
#else
	msm_gpiomux_install(wcnss_5wire_interface,
				ARRAY_SIZE(wcnss_5wire_interface));
#endif
	
	// GPIO related function <<15.FM>>
	msm_gpiomux_install(gpio_func_fm_configs, ARRAY_SIZE(gpio_func_fm_configs));
	
	// GPIO related function <<16.WLC>>
	msm_gpiomux_install(gpio_func_wlc_configs, ARRAY_SIZE(gpio_func_wlc_configs));
	
	// GPIO related function <<17.SIM>>
	msm_gpiomux_install(gpio_func_sim_configs, ARRAY_SIZE(gpio_func_sim_configs));
	
	// GPIO related function <<18.SLIMBUS>>
	msm_gpiomux_install(gpio_func_slimbus_configs, ARRAY_SIZE(gpio_func_slimbus_configs));
	
	// GPIO related function <<19.RF>>
	msm_gpiomux_install(gpio_func_rf_configs, ARRAY_SIZE(gpio_func_rf_configs));
	
	// GPIO related function <<20.KEY PAD>>
	msm_gpiomux_install(msm_keypad_configs, ARRAY_SIZE(msm_keypad_configs));
	
	// GPIO related function <<21.LOGIC>>
	msm_gpiomux_install(gpio_func_logic_configs, ARRAY_SIZE(gpio_func_logic_configs));
#endif

#ifndef CONFIG_MACH_LGE	
#if defined(CONFIG_KS8851) || defined(CONFIG_KS8851_MODULE)
	msm_gpiomux_install(msm_eth_configs, ARRAY_SIZE(msm_eth_configs));
#endif

	msm_gpiomux_install(msm_blsp_configs, ARRAY_SIZE(msm_blsp_configs));


	if (of_board_is_cdp() || of_board_is_mtp() || of_board_is_xpm())
		msm_gpiomux_install(usb_otg_sw_configs,
					ARRAY_SIZE(usb_otg_sw_configs));
#endif

#ifdef CONFIG_TSPDRV
    if (lge_get_board_revno() < HW_REV_A)
        msm_gpiomux_install(vibrator_configs, ARRAY_SIZE(vibrator_configs));
#endif

    msm_gpiomux_sdc3_install();

#ifdef CONFIG_SND_BRCM_FM_RADIO
	msm_gpiomux_install(msm_pri_mi2s_configs, ARRAY_SIZE(msm_pri_mi2s_configs));	
#endif

	printk(KERN_ERR "Jaguar DS China(rev.%d) - GPIO MUX initialized.\n", hw_rev);
}
