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
#include <mach/socinfo.h>
#include <mach/board_lge.h>

static struct gpiomux_setting gpio_i2c_config = {
	.func = GPIOMUX_FUNC_3,
	.drv  = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

/* GPIO I2C */
static struct gpiomux_setting gpio_common_i2c_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv  = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gpio_cam_i2c_config = {
	.func = GPIOMUX_FUNC_1,
	.drv  = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

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

static struct gpiomux_setting lcd_en_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_HIGH,
};

static struct gpiomux_setting lcd_en_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting lcd_te_act_config = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting lcd_te_sus_config = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_IN,
};

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

/* define gpio used as interrupt input */
static struct gpiomux_setting gpio_int_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting gpio_int_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

static struct msm_gpiomux_config msm_gpio_int_configs[] __initdata = {
	{
		.gpio = 84,
		.settings = {
			[GPIOMUX_ACTIVE]	= &gpio_int_act_cfg,
			[GPIOMUX_SUSPENDED]	= &gpio_int_sus_cfg,
		},
	},
};

static struct msm_gpiomux_config msm_lcd_configs[] __initdata = {
	{
		.gpio = 41,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcd_en_act_cfg,
			[GPIOMUX_SUSPENDED] = &lcd_en_sus_cfg,
		},
	},
	{
		.gpio = 12,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcd_te_act_config,
			[GPIOMUX_SUSPENDED] = &lcd_te_sus_config,
		},
	},
};

static struct msm_gpiomux_config msm_blsp_configs[] __initdata = {
	{
		.gpio      = 2,		/* BLSP1 QUP1 I2C_SDA */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		},
	},
	{
		.gpio      = 3,		/* BLSP1 QUP1 I2C_SCL */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		},
	},
	{
		.gpio      = 4,		/* GPIO I2C_SDA */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_common_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_common_i2c_config,
		},
	},
	{
		.gpio      = 5,		/* GPIO I2C_SCL */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_common_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_common_i2c_config,
		},
	},
	{
		.gpio      = 10,	/* BLSP1 QUP3 I2C_SDA */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		},
	},
	{
		.gpio      = 11,	/* BLSP1 QUP3 I2C_SCL */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		},
	},
	{
		.gpio      = 16,	/* BLSP1 QUP6 I2C_SDA */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_cam_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_cam_i2c_config,
		},
	},
	{
		.gpio      = 17,	/* BLSP1 QUP6 I2C_SCL */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_cam_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_cam_i2c_config,
		},
	},
};

#if defined (CONFIG_LGE_BROADCAST_ONESEG)
static struct gpiomux_setting lge_1seg_int_pin_suspend = {
// 1SEG SPI INTERUPT, 1SEG POWER EN, 1SEG LDO EN
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting lge_1seg_ctrl_pin_suspend = {
// 1SEG SPI INTERUPT, 1SEG POWER EN, 1SEG LDO EN
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting lge_1seg_gpio_blsp4_spi_active_config = {
	/* 80-NA437-1B_MSM8274-MSM8674-MSM8974_GPIO_CONFIGURATION_SPREADSHEET.xlsm  */
	.func = GPIOMUX_FUNC_1, //Please look @ GPIO function table for correc function
	.drv = GPIOMUX_DRV_8MA, //Drive Strength
	.pull = GPIOMUX_PULL_NONE, //Should be PULL NONE
};

static struct gpiomux_setting lge_1seg_gpio_blsp4_spi_cs_suspend_config = {
	/* 80-NA437-1B_MSM8274-MSM8674-MSM8974_GPIO_CONFIGURATION_SPREADSHEET.xlsm  */
	.func = GPIOMUX_FUNC_1, //Please look @ GPIO function table for correc function
	.drv = GPIOMUX_DRV_2MA, //Drive Strength
	.pull = GPIOMUX_PULL_UP, //Should be PULL NONE
};

static struct gpiomux_setting lge_1seg_gpio_blsp4_spi_suspend_config = {
	/* 80-NA437-1B_MSM8274-MSM8674-MSM8974_GPIO_CONFIGURATION_SPREADSHEET.xlsm  */
	.func = GPIOMUX_FUNC_1, //Please look @ GPIO function table for correc function
	.drv = GPIOMUX_DRV_2MA, //Drive Strength
	.pull = GPIOMUX_PULL_DOWN, //Should be PULL NONE
};
#endif

#if defined (CONFIG_LGE_BROADCAST_ONESEG)
static struct msm_gpiomux_config lge_1seg_blsp_configs[] __initdata = {
	{
		.gpio		= 86,				/* 1SEG_SPI_MOSI, BLSP4_3 */
		.settings = {
			[GPIOMUX_ACTIVE]	= &lge_1seg_gpio_blsp4_spi_active_config,
			[GPIOMUX_SUSPENDED] = &lge_1seg_gpio_blsp4_spi_suspend_config,
		},
	},
	{
		.gpio		= 87,				/* 1SEG_SPI_MISO, BLSP4_2 */
		.settings = {
			[GPIOMUX_ACTIVE]	= &lge_1seg_gpio_blsp4_spi_active_config,
			[GPIOMUX_SUSPENDED] = &lge_1seg_gpio_blsp4_spi_suspend_config,
		},
	},
	{
		.gpio		= 88,				/* 1SEG_SPI_CS, BLSP4_1 */
		.settings = {
			[GPIOMUX_ACTIVE]	= &lge_1seg_gpio_blsp4_spi_active_config,
			[GPIOMUX_SUSPENDED] = &lge_1seg_gpio_blsp4_spi_cs_suspend_config,
		},
	},
	{
		.gpio		= 89,				/* 1SEG_SPI_CLK, BLSP4_0 */
		.settings = {
			[GPIOMUX_ACTIVE]	= &lge_1seg_gpio_blsp4_spi_active_config,
			[GPIOMUX_SUSPENDED] = &lge_1seg_gpio_blsp4_spi_suspend_config,
		},
	},
	{
		.gpio	   = 92,					/* 1SEG_RESET_N */
		.settings = {
			[GPIOMUX_SUSPENDED] = &lge_1seg_ctrl_pin_suspend,
		},
	},
	{
		.gpio	   = 100,					/* 1SEG_LDO_EN */
		.settings = {
			[GPIOMUX_SUSPENDED] = &lge_1seg_ctrl_pin_suspend,
		},
	},
	{
		.gpio	   = 82,					// 1SEG_INT_N
		.settings = {
			[GPIOMUX_SUSPENDED] = &lge_1seg_int_pin_suspend,
		},
	},
};
#endif

static struct msm_gpiomux_config wcnss_5wire_interface[] = {
	{
		.gpio = 23,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 24,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 25,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 26,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 27,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
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

/*
 * This source has changed to match at lgps15 HDK board
 * changed by junil0814.lee@lge.com 2013-06-05
 */
static struct msm_gpiomux_config msm_sensor_configs[] __initdata = {
	{
		.gpio = 13, /* CAM_MCLK0 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &cam_settings[1],
		},
	},
	{
		.gpio = 14, /* CAM_MCLK1 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &cam_settings[1],
		},
	},
	{
		.gpio = 15, /* VT_CAM_RESET_N */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[1],
		},
	},
	{
		.gpio = 16, /* CAM_I2C_SDA */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[0],
		},
	},
	{
		.gpio = 17, /* CAM_I2C_SCL */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[0],
		},
	},
	{
		.gpio = 18, /* FLASH_LED_EN */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[1],
		},
	},
	{
		.gpio = 20, /* CAM1_STANDBY_N */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[1],
		},
	},
	{
		.gpio = 21, /* CAM1_RESET_N */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[1],
		},
	},
	{
		.gpio = 91, /* VANA_LDO_EN */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[1],
		},
	},
};

static struct msm_gpiomux_config msm_sensor_configs_rev_b[] __initdata = {
	{
		.gpio = 13, /* CAM_MCLK0 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &cam_settings[1],
		},
	},
	{
		.gpio = 14, /* CAM_MCLK1 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &cam_settings[1],
		},
	},
	{
		.gpio = 15, /* VT_CAM_RESET_N */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[1],
		},
	},
	{
		.gpio = 16, /* CAM_I2C_SDA */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[0],
		},
	},
	{
		.gpio = 17, /* CAM_I2C_SCL */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[0],
		},
	},
	{
		.gpio = 18, /* FLASH_LED_EN */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[1],
		},
	},
	{
		.gpio = 20, /* CAM1_STANDBY_N */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[1],
		},
	},
	{
		.gpio = 21, /* CAM1_RESET_N */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[1],
		},
	},
	{
		.gpio = 85, /* VANA_LDO_EN */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[1],
		},
	},
};


static struct msm_gpiomux_config msm_keypad_configs[] __initdata = {
	{
		.gpio = 72,	/* Volume UP */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_keys_active,
			[GPIOMUX_SUSPENDED] = &gpio_keys_suspend,
		},
	},
	{
		.gpio = 73,	/* Volume Down */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_keys_active,
			[GPIOMUX_SUSPENDED] = &gpio_keys_suspend,
		},
	},
	{
		.gpio = 74,	/* Home */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_keys_active,
			[GPIOMUX_SUSPENDED] = &gpio_keys_suspend,
		},
	},
	{
		.gpio = 75,	/* Q-Memo */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_keys_active,
			[GPIOMUX_SUSPENDED] = &gpio_keys_suspend,
		},
	},
};

static struct gpiomux_setting sd_card_det_active_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

#ifdef CONFIG_MACH_LGE
static struct gpiomux_setting sd_card_det_suspend_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};
#else
static struct gpiomux_setting sd_card_det_suspend_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};
#endif

static struct msm_gpiomux_config sd_card_det[] __initdata = {
	{
		.gpio = 42,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sd_card_det_active_config,
			[GPIOMUX_SUSPENDED] = &sd_card_det_suspend_config,
		},
	},
};

static struct gpiomux_setting mms100s_ts_int_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting mms100s_ts_int_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting mms100s_ts_ldo_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};

static struct gpiomux_setting mms100s_ts_ldo_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};

static struct gpiomux_setting ts_makerid = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting synap_attn_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting synap_reset_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_HIGH,
};

static struct msm_gpiomux_config mms100s_ts_configs[] __initdata = {
	{
		.gpio      = 0,		/* MMS100s INT */
		.settings = {
			[GPIOMUX_ACTIVE]    = &mms100s_ts_int_act_cfg,
			[GPIOMUX_SUSPENDED] = &mms100s_ts_int_sus_cfg,
		},
	},
	{
		.gpio      = 76,	/* GPIO MAKER ID */
		.settings = {
			[GPIOMUX_ACTIVE]    = &ts_makerid,
			[GPIOMUX_SUSPENDED] = &ts_makerid,
		},
	},
	{
		.gpio      = 75,	/* GPIO LDO */
		.settings = {
			[GPIOMUX_ACTIVE]    = &mms100s_ts_ldo_act_cfg,
			[GPIOMUX_SUSPENDED] = &mms100s_ts_ldo_sus_cfg,
		},
	},
};

static struct msm_gpiomux_config synaptics_configs[] __initdata = {
	{
		.gpio      = 0,		/* RESET_N */
		.settings = {
			[GPIOMUX_ACTIVE]    = &synap_reset_cfg,
			[GPIOMUX_SUSPENDED] = &synap_reset_cfg,
		},
	},	
	{
		.gpio      = 1,		/* ATTN_N */
		.settings = {
			[GPIOMUX_ACTIVE]    = &synap_attn_cfg,
			[GPIOMUX_SUSPENDED] = &synap_attn_cfg,
		},
	},
	{
		.gpio      = 76,	/* GPIO MAKER ID 0*/
		.settings = {
			[GPIOMUX_ACTIVE]    = &ts_makerid,
			[GPIOMUX_SUSPENDED] = &ts_makerid,
		},
	},
	{
		.gpio      = 82,	/* GPIO LDO */
		.settings = {
			[GPIOMUX_ACTIVE]    = &mms100s_ts_ldo_act_cfg,
			[GPIOMUX_SUSPENDED] = &mms100s_ts_ldo_sus_cfg,
		},
	},
};

void __init msm8610_init_gpiomux(void)
{
	int rc;
	hw_rev_type revision = lge_get_board_revno();

	rc = msm_gpiomux_init_dt();
	if (rc) {
		pr_err("%s failed %d\n", __func__, rc);
		return;
	}

	msm_gpiomux_install(msm_blsp_configs, ARRAY_SIZE(msm_blsp_configs));

       /* Touch IC Setting */
	   
      printk(KERN_INFO"[%s][TOUCH] HW_Revision = %d ", __func__, revision );
	   
      if(revision == HW_REV_A) //melfas 
        	msm_gpiomux_install(mms100s_ts_configs, ARRAY_SIZE(mms100s_ts_configs));
	else
             msm_gpiomux_install(synaptics_configs, ARRAY_SIZE(synaptics_configs));

	msm_gpiomux_install(wcnss_5wire_interface,
			ARRAY_SIZE(wcnss_5wire_interface));
	msm_gpiomux_install_nowrite(msm_lcd_configs,
				ARRAY_SIZE(msm_lcd_configs));
	msm_gpiomux_install(msm_keypad_configs,
				ARRAY_SIZE(msm_keypad_configs));

	msm_gpiomux_install(sd_card_det, ARRAY_SIZE(sd_card_det));

	/* LGE_CHANGE_S, Add gpiomux for ex-ldo used gpio, 2013-09-04, hyungtae.lee@lge.com */
	if(revision == HW_REV_0) {
		msm_gpiomux_install(msm_sensor_configs, ARRAY_SIZE(msm_sensor_configs));
		printk(KERN_ERR " [Camera] below HW_REV_0 is using power source from PM\n");
	}
	else if(revision == HW_REV_A) {
		msm_gpiomux_install(msm_sensor_configs, ARRAY_SIZE(msm_sensor_configs));
		printk(KERN_ERR " [Camera] greater than HW_REV_A is using power source from Ex-LDO used GPIO\n");
	}
	else {
		msm_gpiomux_install(msm_sensor_configs_rev_b, ARRAY_SIZE(msm_sensor_configs_rev_b));
		printk(KERN_ERR " [Camera] In greater than HW_REV_B, MAIN_CAM0_RESET_N has been changed from GPIO_98 to GPIO_114\n");
	}
/* LGE_CHANGE_E, Add gpiomux for ex-ldo used gpio, 2013-09-04, hyungtae.lee@lge.com */

	msm_gpiomux_install(msm_gpio_int_configs,
			ARRAY_SIZE(msm_gpio_int_configs));

#if defined (CONFIG_LGE_BROADCAST_ONESEG)
	if(lge_get_board_revno()>= HW_REV_A) {
		msm_gpiomux_install(lge_1seg_blsp_configs, ARRAY_SIZE(lge_1seg_blsp_configs));
		printk(KERN_ERR " [Oneseg] In greater than HW_REV_A\n");
	}
#endif  /* CONFIG_LGE_BROADCAST_ONESEG */

#ifdef CONFIG_LGE_NFC_PN547_C2
	msm_gpiomux_install(msm_nfc_configs, ARRAY_SIZE(msm_nfc_configs));
#endif
/*  LGE_CHANGE_E, [NFC][taesik.kim@lge.com], NFC Bring up */
}
