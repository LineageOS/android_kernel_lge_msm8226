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

#define KS8851_IRQ_GPIO 115

#if defined(CONFIG_KS8851) || defined(CONFIG_KS8851_MODULE)
static struct gpiomux_setting gpio_eth_config = {
	.pull = GPIOMUX_PULL_UP,
	.drv = GPIOMUX_DRV_2MA,
	.func = GPIOMUX_FUNC_GPIO,
};

static struct msm_gpiomux_config msm_eth_configs[] = {
	{
		.gpio = KS8851_IRQ_GPIO,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_eth_config,
		}
	},
};
#endif

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

/* Reserved Pin Setting */
/* TODO : GPIO settting for VFP model */

#define MSM8x26_GPIO_END 121

static int gpio_reserved_pin_rev_A[] = {
	0, 1, 34, 53, 55, 56, 75, 76, 77, 78, 79, 82, 91, 92, 93, 94, 97, 98, 104, 107, 116, 117, 118,
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

#if 0
static struct gpiomux_setting gpio_spi_config = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gpio_spi_sus_config = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};
 
static struct gpiomux_setting gpio_spi_cs_config = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting gpio_spi_cs_sus_config = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting gpio_spi_cs_eth_config = {
	.func = GPIOMUX_FUNC_4,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct msm_gpiomux_config gpio_func_reserved_pin_configs[] __initdata = {
	{
		.gpio = 0,
		.settings = {
			[GPIOMUX_ACTIVE] = &reserved_pin_cfg,
			[GPIOMUX_SUSPENDED] = &reserved_pin_cfg,
		},
	},
	{
		.gpio = 1,
		.settings = {
			[GPIOMUX_ACTIVE] = &reserved_pin_cfg,
			[GPIOMUX_SUSPENDED] = &reserved_pin_cfg,
		},
	},
	{
		.gpio = 2,
		.settings = {
			[GPIOMUX_ACTIVE] = &reserved_pin_cfg,
			[GPIOMUX_SUSPENDED] = &reserved_pin_cfg,
		},
	},
	{
		.gpio = 3,
		.settings = {
			[GPIOMUX_ACTIVE] = &reserved_pin_cfg,
			[GPIOMUX_SUSPENDED] = &reserved_pin_cfg,
		},
	},
};
#endif

/* Sensor Pin Setting */
static struct gpiomux_setting sensor_ic_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting sensor_ic_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

static struct msm_gpiomux_config msm_sensor_ic_configs[] __initdata = {
	{
		.gpio = 4,			/* Initialize HALLIC_INT */
		.settings = {
			[GPIOMUX_SUSPENDED] = &sensor_ic_act_cfg,
			[GPIOMUX_ACTIVE] = &sensor_ic_sus_cfg,
		},
    },
	{
		.gpio = 66,			/* Initialize COMPASS IC */
		.settings = {
			[GPIOMUX_SUSPENDED] = &sensor_ic_act_cfg,
			[GPIOMUX_ACTIVE] = &sensor_ic_sus_cfg,
		},
    },
	{
		.gpio = 69,			/* Initialize ACCEL_INT */
		.settings = {
			[GPIOMUX_SUSPENDED] = &sensor_ic_act_cfg,
			[GPIOMUX_ACTIVE] = &sensor_ic_sus_cfg,
		},
    },
	{
		.gpio = 119,		/* Initialize PROXIMITY_INT */
		.settings = {
			[GPIOMUX_SUSPENDED] = &sensor_ic_act_cfg,
			[GPIOMUX_ACTIVE] = &sensor_ic_sus_cfg,
		},
    },
#ifdef CONFIG_BU52033NVX_CARKIT
	{
		.gpio = 5,
		.settings = {
			[GPIOMUX_ACTIVE] = &hall_ic_sus_cfg,
		},
    },
#endif
};

/* I2C Pin Setting */
static struct gpiomux_setting gpio_i2c_act_config = {
	.func = GPIOMUX_FUNC_3,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gpio_i2c_sus_config = {
	.func = GPIOMUX_FUNC_3,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};


static struct msm_gpiomux_config msm_i2c_configs[] __initdata = {
#if 0
	{
		.gpio      = 0,		/* BLSP1 QUP1 SPI_DATA_MOSI */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_spi_sus_config,
		},
	},
	{
		.gpio      = 1,		/* BLSP1 QUP1 SPI_DATA_MISO */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_spi_sus_config,
		},
	},
#endif
	{
		.gpio      = 2,		/* Fuel gauge I2C SDA */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_act_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_sus_config,
		},
	},
	{
		.gpio      = 3,		/* Fuel gauge I2C SCL */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_act_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_sus_config,
		},
	},

	{
		.gpio      = 6,	/* SENSOR0_I2C_SDA */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_act_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_sus_config,
		},
	},
	{
		.gpio      = 7,	/* SENSOR0_I2C_SCL */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_act_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_sus_config,
		},
	},
	{
		.gpio      = 10,	/* COMMON_I2C_SDA */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_act_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_sus_config,
		},
	},
	{
		.gpio      = 11,	/* COMMON_I2C_SCL */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_act_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_sus_config,
		},
	},
	{
		.gpio      = 63,		/* FM_I2S_CLK */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_act_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_sus_config,
		},
	},
	{
		.gpio      = 64,		/* FM_I2S_FSYNC */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_act_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_sus_config,
		},
	},
	{
		.gpio      = 65,		/* FM_I2S_DATA_OUT */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_act_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_sus_config,
		},
	},
};
static struct gpiomux_setting melfas_int_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};
static struct gpiomux_setting melfas_int_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};
static struct gpiomux_setting melfas_id_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};
static struct gpiomux_setting touch_gpio_i2c_sus_config = {
	.func = GPIOMUX_FUNC_3,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting touch_gpio_i2c_act_config = {
	.func = GPIOMUX_FUNC_3,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct msm_gpiomux_config msm_melfas_configs[] __initdata = {
	{		.gpio     = 17,		/* TOUCH INT */
			.settings = {
				[GPIOMUX_ACTIVE] = &melfas_int_act_cfg,
				[GPIOMUX_SUSPENDED] = &melfas_int_sus_cfg,
			},
	},
	{
			.gpio      = 18,	/* TOUCH_I2C_SDA */
			.settings = {
				[GPIOMUX_ACTIVE] = &touch_gpio_i2c_act_config,
				[GPIOMUX_SUSPENDED] = &touch_gpio_i2c_sus_config,
	},
	},
	{
			.gpio      = 19,	/* TOUCH_I2C_SCL */
			.settings = {
				[GPIOMUX_ACTIVE] = &touch_gpio_i2c_act_config,
				[GPIOMUX_SUSPENDED] = &touch_gpio_i2c_sus_config,
		},
	},
	{
			.gpio      = 120,		/* TOUCH ID */
			.settings = {
				[GPIOMUX_ACTIVE] = &melfas_id_cfg,
				[GPIOMUX_SUSPENDED] = &melfas_id_cfg,
			},
	},
};



/* UART Pin Setting */
static struct msm_gpiomux_config gpio_func_uart_configs[] __initdata = {
};

/* NFC Pin Setting */
//+GPIO[010] COMMON_I2C_SDA
//+GPIO[011] COMMON_I2C_SCL
//Need to set GPIO[020] NFC_VEN
//Need to set GPIO[021] NFC_IRQ
//Need to set GPIO[022] NFC_MODE
/*  LGE_CHANGE_S, [NFC][garam.kim@lge.com], NFC Bring up*/
#if 0//def CONFIG_LGE_NFC_PN547_C2
#if 0
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
#endif  // This pin are set at msm_i2c_configs

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
#if 0
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
#endif							// This pin are set at msm_i2c_configs
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

/* LCD pin Setting */
static struct gpiomux_setting lcd_vsync_act_cfg[] = {
	{
		.func = GPIOMUX_FUNC_1, /*active 1*/ /* 0 */
		.drv = GPIOMUX_DRV_8MA,
		.pull = GPIOMUX_PULL_NONE,
	},
	{
		.func = GPIOMUX_FUNC_1, /*suspend*/ /* 1 */
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_DOWN,
	},
};

static struct gpiomux_setting lcd_rst_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
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
	}
};


/* Camera Pin Setting */
//Need to set GPIO[026] MAIN_CAM0_MCLK
//Need to set GPIO[027] VT_CAM_MCLK
//Need to set GPIO[029] CAM0_I2C_SDA
//Need to set GPIO[030] CAM0_I2C_SCL
//Need to set GPIO[114] MAIN_CAM0_RESET_N
//Need to set GPIO[028] VT_CAM_RESET_N
//Need to set GPIO[062] LDO1_EN
//Need to set GPIO[109] LDO2_EN
//Need to set GPIO[036] VCM_EN
//Need to set GPIO[109] LDAF_EN  ---> comment out
//Need to set GPIO[107] LDAF_INT ---> comment out

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
		.gpio = 31, /* CAM2_STANDBY_N */
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
		.gpio = 109, /* LDO2_EN */
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
#if 0
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
#endif
};

/* FLASH_LED Pin Setting */
//Need to set GPIO[032] FLASH_STROBE_TRIG
static struct msm_gpiomux_config gpio_func_flash_led_configs[] __initdata = {
};

/* IRRC Pin Setting */

/* AUDIO Pin Setting */
// GPIO related function <<10.AUDIO>>
//Need to set GPIO[035] EAR_SENSE
//Need to set GPIO[067] EAR_KEY
//Need to set GPIO[068] CODEC_INT_N
//Need to set GPIO[072] CODEC_RESET_N

/* SD_CARD Pin Setting */
static struct gpiomux_setting sd_card_det_active_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

#ifdef CONFIG_MACH_LGE
static struct gpiomux_setting sd_card_det_sleep_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};
#else // not CONFIG_MACH_LGE
static struct gpiomux_setting sd_card_det_sleep_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};
#endif // CONFIG_MACH_LGE

static struct msm_gpiomux_config sd_card_det __initdata = {
	.gpio = 37,
	.settings = {
		[GPIOMUX_ACTIVE]    = &sd_card_det_active_config,
		[GPIOMUX_SUSPENDED] = &sd_card_det_sleep_config,
	},
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

/* CONFIG_MMC_MSM_SDC3_SUPPORT */

/* BATTERY & POWER Pin Setting */
static struct gpiomux_setting power_control_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting power_control_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};
static struct gpiomux_setting batt_id_act_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting batt_id_sus_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

static struct msm_gpiomux_config power_configs[] __initdata = {
	{
		.gpio = 31,			/* External-OVP control */
		.settings = {
			[GPIOMUX_ACTIVE] = &power_control_act_cfg,
			[GPIOMUX_SUSPENDED] = &power_control_sus_cfg,
		},
	},
	{
		.gpio = 38,			/* BATT_ID */
		.settings = {
			[GPIOMUX_ACTIVE] = &batt_id_act_config,
			[GPIOMUX_SUSPENDED] = &batt_id_sus_config,
		},
	},	
};

/* LGE_CHANGE_S, [TDMB][seongeun.jin@lge.com], TDMB Bring Up */
#if defined(CONFIG_LGE_BROADCAST_TDMB)
static struct gpiomux_setting gpio_blsp1_spi_active_config = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gpio_blsp1_spi_suspended_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting gpio_broadcast_ctrl_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gpio_broadcast_int_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct msm_gpiomux_config msm8926_tdmb_configs[] __initdata = {
	{
		.gpio	   = 0,		/* BLSP 1 QUP 0 (BLSP1) SPI_DATA_MOSI */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_blsp1_spi_active_config,
			[GPIOMUX_SUSPENDED] = &gpio_blsp1_spi_suspended_config,
		},
	},
	{
		.gpio	   = 1,		/* BLSP 1 QUP 0 (BLSP1) SPI_DATA_MISO */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_blsp1_spi_active_config,
			[GPIOMUX_SUSPENDED] = &gpio_blsp1_spi_suspended_config,
		},
	},
	{
		.gpio	   = 2,		/* BLSP 1 QUP 0 (BLSP1) SPI_CS_N */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_blsp1_spi_active_config,
			[GPIOMUX_SUSPENDED] = &gpio_blsp1_spi_suspended_config,
		},
	},
	{
		.gpio	   = 3,		/* BLSP 1 QUP 0 (BLSP1) SPI_CLK */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_blsp1_spi_active_config,
			[GPIOMUX_SUSPENDED] = &gpio_blsp1_spi_suspended_config,
		},
	},
	{
		.gpio	   = 63,		/* DNB_EN */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_broadcast_ctrl_config,
		},
	},
	{
		.gpio	   = 64,		/* DNB_INT */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_broadcast_int_config,
		},
	},
	{
		.gpio      = 86,		/* DMB_ANT */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_broadcast_int_config,
		},
	},
};
#endif
/* LGE_CHANGE_E, [TDMB][seongeun.jin@lge.com], TDMB Bring Up */

/* BT Pin Setting */
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

static struct msm_gpiomux_config bt_pcm_configs[] __initdata = {
	{
		.gpio	   = 49,	/* BT_PCM_CLK */
		.settings = {
			[GPIOMUX_ACTIVE]	= &bt_pcm_active_config,
			[GPIOMUX_SUSPENDED] = &bt_pcm_suspend_config,
		},
	},
	{
		.gpio	   = 50,	/* BT_PCM_SYNC */
		.settings = {
			[GPIOMUX_ACTIVE]	= &bt_pcm_active_config,
			[GPIOMUX_SUSPENDED] = &bt_pcm_suspend_config,
		},
	},
	{
		.gpio	   = 51,	/* BT_PCM_DIN */
		.settings = {
			[GPIOMUX_ACTIVE]	= &bt_pcm_active_config,
			[GPIOMUX_SUSPENDED] = &bt_pcm_suspend_config,
		},
	},
	{
		.gpio	   = 52,	/* BT_PCM_DOUT */
		.settings = {
			[GPIOMUX_ACTIVE]	= &bt_pcm_active_config,
			[GPIOMUX_SUSPENDED] = &bt_pcm_suspend_config,
		},
	}
};

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
    msm_gpiomux_install(bt_pcm_configs, ARRAY_SIZE(bt_pcm_configs));
}
#endif /* CONFIG_LGE_BLUETOOTH */
/* LGE_CHANGE_E, BT][teddy.ju@lge.com], 2013-05-13 */

/* FM Pin Setting */
// GPIO related function <<15.FM>>
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

static struct msm_gpiomux_config msm_auxpcm_configs[] __initdata = {
	{
		.gpio = 63,
		.settings = {
			[GPIOMUX_SUSPENDED] = &auxpcm_sus_cfg,
			[GPIOMUX_ACTIVE] = &auxpcm_act_cfg,
		},
	},
	{
		.gpio = 64,
		.settings = {
			[GPIOMUX_SUSPENDED] = &auxpcm_sus_cfg,
			[GPIOMUX_ACTIVE] = &auxpcm_act_cfg,
		},
	},
	{
		.gpio = 65,
		.settings = {
			[GPIOMUX_SUSPENDED] = &auxpcm_sus_cfg,
			[GPIOMUX_ACTIVE] = &auxpcm_act_cfg,
		},
	},
#if 0
	{
		.gpio = 66,
		.settings = {
			[GPIOMUX_SUSPENDED] = &auxpcm_sus_cfg,
			[GPIOMUX_ACTIVE] = &auxpcm_act_cfg,
		},
	},
#endif
};

#if 0
static struct msm_gpiomux_config gpio_func_fm_configs[] __initdata = {
};
#endif

/* SIM Pin Setting */
// GPIO related function <<17.SIM>>
//Need to set GPIO[057] UIM1_DATA
//Need to set GPIO[058] UIM1_CLK
//Need to set GPIO[059] UIM1_RESET_N
//Need to set GPIO[061] BATT_UIM_ALARM
static struct msm_gpiomux_config gpio_func_sim_configs[] __initdata = {
};

/* SLIMBUS Pin Setting */
// GPIO related function <<18.SLIMBUS>>
//Need to set GPIO[070] SLIMBUS_CLK
//Need to set GPIO[071] SLIMBUS_DATA
static struct msm_gpiomux_config gpio_func_slimbus_configs[] __initdata = {
};

/* RF Pin Setting */
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

/* KEY_PAD Pin Setting */
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
/* PP2106 Pin Setting */
static struct gpiomux_setting i2c_pp2106_active = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting i2c_pp2106_suspend = {
	.func = GPIOMUX_FUNC_2,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gpio_pp2106_active = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting gpio_pp2106_suspend = {
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
		.gpio = 107,
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
#if 0 // GPIO 109 is used for LDAF_EN
	{
		.gpio = 109,
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_keys_active,
			[GPIOMUX_SUSPENDED] = &gpio_keys_suspend,
		},
	},
#endif
};

static struct msm_gpiomux_config msm_pp2106_keypad_configs[] __initdata = {
	{
		.gpio = 115,
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_pp2106_active,
			[GPIOMUX_SUSPENDED] = &gpio_pp2106_suspend,
		},
	},
	{
		.gpio = 116,
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_pp2106_active,
			[GPIOMUX_SUSPENDED] = &gpio_pp2106_suspend,
		},
	},
	{
		.gpio = 117,
		.settings = {
			[GPIOMUX_ACTIVE]    = &i2c_pp2106_active,
			[GPIOMUX_SUSPENDED] = &i2c_pp2106_suspend,
		},
	},
	{
		.gpio = 118,
		.settings = {
			[GPIOMUX_ACTIVE]    = &i2c_pp2106_active,
			[GPIOMUX_SUSPENDED] = &i2c_pp2106_suspend,
		},
	},
};

/* LOGIC & LDO_EN & VIBRATOR Pin Setting */
static struct gpiomux_setting ldo_en_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_OUT_HIGH,
};

static struct gpiomux_setting ldo_en_cfg_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct msm_gpiomux_config ext_ldo_en_configs[] __initdata = {
	{
		.gpio = 62, /* LDO1_EN */
		.settings = {
			[GPIOMUX_ACTIVE] = &ldo_en_act_cfg ,
			[GPIOMUX_SUSPENDED] = &ldo_en_cfg_sus_cfg ,
		},
	},
	{
		.gpio = 113, /* LDO3_EN */
		.settings = {
			[GPIOMUX_ACTIVE] = &ldo_en_act_cfg ,
			[GPIOMUX_SUSPENDED] = &ldo_en_cfg_sus_cfg ,
		},
	},
};

static struct gpiomux_setting vibrator_active_cfg_gpio_pwm = {
       .func = GPIOMUX_FUNC_3, //2013-08-22 beekay.lee@lge.com For WX(MSM8x26). GPIO34 has alternative function 3(=GP1_CLK)
       .drv = GPIOMUX_DRV_8MA,
       .pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting vibrator_suspend_cfg = {
       .func = GPIOMUX_FUNC_GPIO,
       .drv = GPIOMUX_DRV_2MA,
       .pull = GPIOMUX_PULL_DOWN,
};

static struct msm_gpiomux_config vibrator_configs __initdata = {
	.gpio = 34,
	.settings = {
			[GPIOMUX_ACTIVE]    = &vibrator_active_cfg_gpio_pwm,
			[GPIOMUX_SUSPENDED] = &vibrator_suspend_cfg,
	}
};

static void logic_msm_gpiomux_install(void) {
	hw_rev_type hw_rev;
	hw_rev = lge_get_board_revno();
	
	if (hw_rev < HW_REV_B) {
		msm_gpiomux_install(ext_ldo_en_configs, ARRAY_SIZE(ext_ldo_en_configs));
		msm_gpiomux_install(&vibrator_configs, 1);
	} else {
	msm_gpiomux_install(ext_ldo_en_configs, ARRAY_SIZE(ext_ldo_en_configs));
	}
};

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

void __init msm8226_init_gpiomux(void)
{
	int rc;
	int gpio_index = 0;

	hw_rev_type hw_rev;
	hw_rev = lge_get_board_revno();

	// Device Tree Initailize
	rc = msm_gpiomux_init_dt();
	if (rc) {
		pr_err("%s failed %d\n", __func__, rc);
		return;
	}

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
	switch (hw_rev) {
	case HW_REV_0:
	case HW_REV_A:
		for (gpio_index = 0;
		     gpio_reserved_pin_rev_A[gpio_index] < MSM8x26_GPIO_END;
		     gpio_index++) {
			gpio_func_reserved_pin_config.gpio =
			    gpio_reserved_pin_rev_A[gpio_index];
			msm_gpiomux_install(&gpio_func_reserved_pin_config, 1);
		}
		break;

	case HW_REV_B :
	case HW_REV_C :
	case HW_REV_D :
	case HW_REV_E :
	case HW_REV_1_0 :
	case HW_REV_1_1 :
	default:
			break;
	}

	msm_gpiomux_install(wcnss_5wire_interface, ARRAY_SIZE(wcnss_5wire_interface));
	
	msm_gpiomux_install(&sd_card_det, 1);
	// GPIO related function <<1.SENSOR>>
	msm_gpiomux_install(msm_sensor_ic_configs, ARRAY_SIZE(msm_sensor_ic_configs));	

	// GPIO related function <<2.I2C>>
	msm_gpiomux_install(msm_i2c_configs, ARRAY_SIZE(msm_i2c_configs));

	// GPIO related function <<3.UART>>
	msm_gpiomux_install(gpio_func_uart_configs, ARRAY_SIZE(gpio_func_uart_configs));

	// GPIO related function <<4.TOUCH>>
	msm_gpiomux_install(msm_melfas_configs, ARRAY_SIZE(msm_melfas_configs));

	// GPIO related function <<5.NFC>>
/*  LGE_CHANGE_S, [NFC][garam.kim@lge.com], NFC Bring up */
#ifdef CONFIG_LGE_NFC_PN547_C2
	msm_gpiomux_install(msm_nfc_configs, ARRAY_SIZE(msm_nfc_configs));
#endif
/*  LGE_CHANGE_E, [NFC][garam.kim@lge.com], NFC Bring up */

	// GPIO related function <<6.LCD>>
		msm_gpiomux_install_nowrite(msm_lcd_configs,
						ARRAY_SIZE(msm_lcd_configs));

	// GPIO related function <<7.CAMERA>>
	msm_gpiomux_install(msm_sensor_configs, ARRAY_SIZE(msm_sensor_configs));

	// GPIO related function <<8.FLASH LED>>
	msm_gpiomux_install(gpio_func_flash_led_configs, ARRAY_SIZE(gpio_func_flash_led_configs));

	// GPIO related function <<10.AUDIO>>

	// GPIO related function <<11.SD CARD>>
	//msm_gpiomux_sdc3_install();

	// GPIO related function <<12.BATTERY>>
	msm_gpiomux_install(power_configs, ARRAY_SIZE(power_configs));
	
	// GPIO related function <<13.BT>>
#ifdef CONFIG_LGE_BLUETOOTH
    bluetooth_msm_gpiomux_install();
#endif /* CONFIG_LGE_BLUETOOTH */
	
	// GPIO related function <<15.FM>>
	msm_gpiomux_install(msm_auxpcm_configs,	ARRAY_SIZE(msm_auxpcm_configs));

	// GPIO related function <<16.WLC>>
	
	// GPIO related function <<17.SIM>>
	msm_gpiomux_install(gpio_func_sim_configs, ARRAY_SIZE(gpio_func_sim_configs));
	
	// GPIO related function <<18.SLIMBUS>>
	msm_gpiomux_install(gpio_func_slimbus_configs, ARRAY_SIZE(gpio_func_slimbus_configs));
	
	// GPIO related function <<19.RF>>
	msm_gpiomux_install(gpio_func_rf_configs, ARRAY_SIZE(gpio_func_rf_configs));
	
	// GPIO related function <<20.KEY PAD>>
	msm_gpiomux_install(msm_keypad_configs, ARRAY_SIZE(msm_keypad_configs));
	msm_gpiomux_install(msm_pp2106_keypad_configs, ARRAY_SIZE(msm_pp2106_keypad_configs));
	
	// GPIO related function <<21.LOGIC>>
	logic_msm_gpiomux_install();

/* LGE_CHANGE_S, [TDMB][hyun118.shin@lge.com], TDMB Bring Up */
#if defined(CONFIG_LGE_BROADCAST_TDMB)
	msm_gpiomux_install(msm8926_tdmb_configs, ARRAY_SIZE(msm8926_tdmb_configs));
#endif
/* LGE_CHANGE_E, [TDMB][hyun118.shin@lge.com], TDMB Bring Up */

#if defined(CONFIG_KS8851) || defined(CONFIG_KS8851_MODULE)
	msm_gpiomux_install(msm_eth_configs, ARRAY_SIZE(msm_eth_configs));
#endif

#ifndef CONFIG_MACH_LGE
	if (of_board_is_cdp() || of_board_is_mtp() || of_board_is_xpm())
		msm_gpiomux_install(usb_otg_sw_configs,
					ARRAY_SIZE(usb_otg_sw_configs));
#endif
}
