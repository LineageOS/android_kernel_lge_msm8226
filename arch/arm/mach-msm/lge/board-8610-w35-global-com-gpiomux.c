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

// GPIO related function <<0.Resreved Pin>>
#define MSM8x10_GPIO_END 102

static int gpio_reserved_pin_rev_0[] = {
	1, 4, 5, 14, 15, 18, 35, 39, 44, 46, 48, 55, 58, 61, 62, 66, 67, 
	70, 71, 74, 75, 77, 78, 83, 84, 85, 90, 91, 93, 97, 98, 99,
	MSM8x10_GPIO_END // This is included to notify the end of reserved GPIO configuration.
};

static int gpio_reserved_pin_rev_A[] = {
	1, 7, 14, 15, 18, 35, 39, 44, 46, 47, 48, 55, 57, 61, 66, 67, 70, 71, 75, 83, 84, 90, 97, 98, 99,
	MSM8x10_GPIO_END // This is included to notify the end of reserved GPIO configuration.
};

static int gpio_reserved_pin_rev_C[] = {
	7, 14, 15, 18, 35, 39, 44, 46, 47, 48, 55, 57, 61, 66, 67, 70, 71, 75, 83, 84, 90, 97, 99,
	MSM8x10_GPIO_END // This is included to notify the end of reserved GPIO configuration.
};

/// Rev_E...
static int gpio_reserved_pin_rev_E[] = {
	7, 14, 15, 18, 35, 39, 44, 46, 47, 48, 55, 57, 61, 66, 67, 70, 75, 90, 91, 93, 97, 98, 99,
	MSM8x10_GPIO_END // This is included to notify the end of reserved GPIO configuration.
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

#if !defined (CONFIG_LGE_BROADCAST_ONESEG)
static struct gpiomux_setting gpio_spi_config = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
};
#endif
static struct gpiomux_setting gpio_i2c_config = {
	.func = GPIOMUX_FUNC_3,
	.drv  = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gpio_bl_i2c_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv  = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting gpio_cam_i2c_config = {
	.func = GPIOMUX_FUNC_1,
	.drv  = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
};

static struct gpiomux_setting melfas_vdd_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_OUT_LOW,
};

static struct gpiomux_setting melfas_int_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting melfas_int_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting focaltech_int_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting focaltech_int_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting focaltech_reset_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_UP,
};

static struct gpiomux_setting focaltech_reset_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_UP,
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

static struct gpiomux_setting ts_ldo_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};

static struct gpiomux_setting ts_ldo_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
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

static struct gpiomux_setting synap_maker_id_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
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
		.gpio      = 4,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_bl_i2c_config,
		},
	},
	{
		.gpio      = 5,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_bl_i2c_config,
		},
	},
	{
		.gpio      = 6,		/* BLSP1 QUP2 I2C_SDA */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		},
	},
	{
		.gpio      = 7,		/* BLSP1 QUP2 I2C_SCL */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
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

static struct msm_gpiomux_config lge_1seg_blsp_configs_revA[] __initdata = {
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
		.gpio	   = 93,					/* 1SEG_LDO_EN */
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

static struct msm_gpiomux_config lge_1seg_blsp_configs_rev0[] __initdata = {
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
		.gpio	   = 56,					/* 1SEG_RESET_N */
		.settings = {
			[GPIOMUX_SUSPENDED] = &lge_1seg_ctrl_pin_suspend,
		},
	},
	{
		.gpio	   = 57,					/* 1SEG_LDO_EN */
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
static struct msm_gpiomux_config msm_melfas_configs[] __initdata = {
	{
		.gpio      = 0,		/* TOUCH INT */
		.settings = {
			[GPIOMUX_ACTIVE] = &melfas_int_act_cfg,
			[GPIOMUX_SUSPENDED] = &melfas_int_sus_cfg,
		},
	},
	{
		.gpio      = 62,		/* TOUCH VDD */
		.settings = {
			[GPIOMUX_ACTIVE] = &melfas_vdd_cfg,
			[GPIOMUX_SUSPENDED] = &melfas_vdd_cfg,
		},
	},
#if !defined (CONFIG_LGE_BROADCAST_ONESEG)
	{
		.gpio      = 86,		/* BLSP1 QUP4 SPI_DATA_MOSI */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_spi_config,
		},
	},
	{
		.gpio      = 87,		/* BLSP1 QUP4 SPI_DATA_MISO */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_spi_config,
		},
	},
	{
		.gpio      = 89,		/* BLSP1 QUP4 SPI_CLK */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_spi_config,
		},
	},
	{
		.gpio      = 88,		/* BLSP1 QUP4 SPI_CS */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_spi_config,
		},
	},
#endif
};

static struct msm_gpiomux_config msm_syunap_configs[] __initdata = {
	{
		.gpio = 0,    /*TOUCH RESET*/
		.settings = {
			[GPIOMUX_ACTIVE] = &synap_reset_cfg,
			[GPIOMUX_SUSPENDED] = &synap_reset_cfg,
		},
	},
	{
		.gpio = 1,	/*TOUCH INT*/
		.settings = {
			[GPIOMUX_ACTIVE] = &synap_attn_cfg,
			[GPIOMUX_SUSPENDED] = &synap_attn_cfg,
		},
	},
	{
		.gpio      = 62,		/* TOUCH VDD */
		.settings = {
			[GPIOMUX_ACTIVE] = &ts_ldo_act_cfg,
			[GPIOMUX_SUSPENDED] = &ts_ldo_sus_cfg,
		},
	},
	{
		.gpio = 76,	/*TOUCH MAKER ID*/
		.settings = {
			[GPIOMUX_ACTIVE] = &synap_maker_id_cfg,
			[GPIOMUX_SUSPENDED] = &synap_maker_id_cfg,
		},
	},
	{
		.gpio = 77,	/*TOUCH MAKER 2 ID*/
		.settings = {
			[GPIOMUX_ACTIVE] = &synap_maker_id_cfg,
			[GPIOMUX_SUSPENDED] = &synap_maker_id_cfg,
		},
	},
};


static struct msm_gpiomux_config msm_focaltech_configs[] __initdata = {
	{
		.gpio      = 0,		/* TOUCH RESET */
		.settings = {
			[GPIOMUX_ACTIVE] = &focaltech_reset_act_cfg,
			[GPIOMUX_SUSPENDED] = &focaltech_reset_sus_cfg,
		},
	},
	{
		.gpio      = 1,		/* TOUCH INT */
		.settings = {
			[GPIOMUX_ACTIVE] = &focaltech_int_act_cfg,
			[GPIOMUX_SUSPENDED] = &focaltech_int_sus_cfg,
		},
	},
#if !defined (CONFIG_LGE_BROADCAST_ONESEG)
	{
		.gpio      = 86,		/* BLSP1 QUP4 SPI_DATA_MOSI */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_spi_config,
		},
	},
	{
		.gpio      = 87,		/* BLSP1 QUP4 SPI_DATA_MISO */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_spi_config,
		},
	},
	{
		.gpio      = 89,		/* BLSP1 QUP4 SPI_CLK */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_spi_config,
		},
	},
	{
		.gpio      = 88,		/* BLSP1 QUP4 SPI_CS */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_spi_config,
		},
	},
#endif
};


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

/* LGE_CHANGE_E, Code modifying by revision for revA and revB, youngwook.song@lge.com 2013-09-21 */
static struct msm_gpiomux_config msm_sensor_configs_revA[] __initdata = {
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
		.gpio = 91, /* VDIG_LDO_EN_Port. this will start 2ex-ldo for vdig and vana at the same time., youngwook.song@lge.com, 2013.08.26 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[1],
		},
	},
};

static struct msm_gpiomux_config msm_sensor_configs_revB[] __initdata = {
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
		.gpio = 85, /* VDIG_LDO_EN_Port. this will start 2ex-ldo for vdig and vana at the same time., youngwook.song@lge.com, 2013.08.26 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[3],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[1],
		},
	},
};
/* LGE_CHANGE_X, Code modifying by revision for revA and revB, youngwook.song@lge.com 2013-09-21 */

static struct msm_gpiomux_config msm_keypad_configs[] __initdata = {
	{
		.gpio = 72,
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_keys_active,
			[GPIOMUX_SUSPENDED] = &gpio_keys_suspend,
		},
	},
	{
		.gpio = 73,
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_keys_active,
			[GPIOMUX_SUSPENDED] = &gpio_keys_suspend,
		},
	},
	{
		.gpio = 74,
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

#if defined(CONFIG_MACH_LGE) && defined(CONFIG_SWITCH_SPK_RCV)
// sangman.park@lge.com(SOUND) 2013_7_17 added speaker switch mixer control  
static struct gpiomux_setting spk_rcv_active_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_OUT_HIGH
};


static struct msm_gpiomux_config msm_spk_rcv_det[] __initdata = {
	{
		.gpio = 19,
		.settings = {
			[GPIOMUX_ACTIVE] = &spk_rcv_active_config,
		},
	}
};
#endif/*CONFIG_MACH_LGE&&CONFIG_SWITCH_SPK_RCV*/

#ifdef CONFIG_FMR_INTENNA
// real-wifi@lge.com(FM) 2013_8_23 added FM intenna GPIO control
static struct gpiomux_setting fmr_intenna_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_OUT_HIGH
};

static struct msm_gpiomux_config fmr_intenna_det[] __initdata = {
	{
		.gpio = 58,
		.settings = {
			[GPIOMUX_ACTIVE] = &fmr_intenna_config,
		},
	}
};
#endif

/// Rev_E
#ifdef CONFIG_MACH_LGE
static struct gpiomux_setting pcb_indicator_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN
};

static struct msm_gpiomux_config pcb_indicator[] __initdata = {
	{
		.gpio = 59,	/* Low Band */
		.settings = {
			[GPIOMUX_ACTIVE] = &pcb_indicator_config,
			[GPIOMUX_SUSPENDED] = &pcb_indicator_config,
		},
	},
	{
		.gpio = 83,	/* High Band */
		.settings = {
			[GPIOMUX_ACTIVE] = &pcb_indicator_config,
			[GPIOMUX_SUSPENDED] = &pcb_indicator_config,
		},
	},
	{
		.gpio = 84,	/* FMR */
		.settings = {
			[GPIOMUX_ACTIVE] = &pcb_indicator_config,
			[GPIOMUX_SUSPENDED] = &pcb_indicator_config,
		},
	}
};
#endif /* CONFIG_MACH_LGE */

#ifdef CONFIG_MACH_LGE
static struct gpiomux_setting main_cam_id_gpio_act_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN
};
static struct gpiomux_setting main_cam_id_gpio_sus_config = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN
};

static struct msm_gpiomux_config main_cam_id_gpio[] __initdata = {
	{
		.gpio = 71,
		.settings = {
			[GPIOMUX_ACTIVE] = &main_cam_id_gpio_act_config,
			[GPIOMUX_SUSPENDED] = &main_cam_id_gpio_sus_config,
		}
	}
};
#endif /* CONFIG_MACH_LGE */

void __init msm8610_init_gpiomux(void)
{
	int rc;

#ifdef CONFIG_MACH_LGE
	int gpio_index = 0;
	//2013-07-29, seungkyu.joo, add gpio85 config for revA[End]
      hw_rev_type lge_bd_rev = lge_get_board_revno();
	//2013-07-29, seungkyu.joo, add gpio85 config for revA[End]
#endif

	rc = msm_gpiomux_init_dt();
	if (rc) {
		pr_err("%s failed %d\n", __func__, rc);
		return;
	}
#ifdef CONFIG_MACH_LGE
	//--------------------------------------------
	// MSM8X10 GPIO Confiuration via W35
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

	switch ( lge_bd_rev ){
		case HW_REV_0 :
			for ( gpio_index = 0 ; gpio_reserved_pin_rev_0[gpio_index] < MSM8x10_GPIO_END ; gpio_index++ ){
				gpio_func_reserved_pin_config.gpio = gpio_reserved_pin_rev_0[gpio_index];
				msm_gpiomux_install(&gpio_func_reserved_pin_config, 1);
				}
			msm_gpiomux_install(msm_sensor_configs_revA, ARRAY_SIZE(msm_sensor_configs_revA)); // LGE_CHANGE, Code modifying by revision, youngwook.song@lge.com 2013-09-21
			break;
		case HW_REV_A :
            for ( gpio_index = 0 ; gpio_reserved_pin_rev_A[gpio_index] < MSM8x10_GPIO_END ; gpio_index++ ){
				gpio_func_reserved_pin_config.gpio = gpio_reserved_pin_rev_A[gpio_index];
				msm_gpiomux_install(&gpio_func_reserved_pin_config, 1);
				}
			msm_gpiomux_install(msm_sensor_configs_revA, ARRAY_SIZE(msm_sensor_configs_revA)); // LGE_CHANGE, Code modifying by revision, youngwook.song@lge.com 2013-09-21
			break;
		case HW_REV_B :
		case HW_REV_C :
			for ( gpio_index = 0 ; gpio_reserved_pin_rev_C[gpio_index] < MSM8x10_GPIO_END ; gpio_index++ ){
				gpio_func_reserved_pin_config.gpio = gpio_reserved_pin_rev_C[gpio_index];
				msm_gpiomux_install(&gpio_func_reserved_pin_config, 1);
				}
            msm_gpiomux_install(msm_sensor_configs_revB, ARRAY_SIZE(msm_sensor_configs_revB)); // LGE_CHANGE, Code modifying by revision, youngwook.song@lge.com 2013-09-21
            break;
		case HW_REV_D :
		case HW_REV_E :
		case HW_REV_1_0 :
		case HW_REV_1_1 :
		default :
			for ( gpio_index = 0 ; gpio_reserved_pin_rev_E[gpio_index] < MSM8x10_GPIO_END ; gpio_index++ ){
				gpio_func_reserved_pin_config.gpio = gpio_reserved_pin_rev_E[gpio_index];
				msm_gpiomux_install(&gpio_func_reserved_pin_config, 1);
				}
			msm_gpiomux_install(msm_sensor_configs_revB, ARRAY_SIZE(msm_sensor_configs_revB)); // LGE_CHANGE, Code modifying by revision, youngwook.song@lge.com 2013-09-21
			msm_gpiomux_install(pcb_indicator, ARRAY_SIZE(pcb_indicator));
			msm_gpiomux_install(main_cam_id_gpio, ARRAY_SIZE(main_cam_id_gpio));	/* MAIN_CAM_ID */
			break;
	}
#endif

	msm_gpiomux_install(msm_blsp_configs, ARRAY_SIZE(msm_blsp_configs));
	if (of_board_is_qrd()) {
		msm_gpiomux_install(msm_focaltech_configs,
			ARRAY_SIZE(msm_focaltech_configs));
	} else {
		if(lge_bd_rev < HW_REV_E) {
			printk(KERN_ERR "[TOUCH] Set GPIO for MELFAS \n");
			msm_gpiomux_install(msm_melfas_configs, ARRAY_SIZE(msm_melfas_configs));
		}else{
			printk(KERN_ERR "[TOUCH] Set GPIO for greater than Rev.E \n");
			msm_gpiomux_install(msm_syunap_configs, ARRAY_SIZE(msm_syunap_configs));
		}
	}
	msm_gpiomux_install(wcnss_5wire_interface,
			ARRAY_SIZE(wcnss_5wire_interface));
	msm_gpiomux_install_nowrite(msm_lcd_configs,
				ARRAY_SIZE(msm_lcd_configs));
	msm_gpiomux_install(msm_keypad_configs,
				ARRAY_SIZE(msm_keypad_configs));
	msm_gpiomux_install(sd_card_det, ARRAY_SIZE(sd_card_det));
	msm_gpiomux_install(msm_gpio_int_configs,
			ARRAY_SIZE(msm_gpio_int_configs));

#if defined (CONFIG_LGE_BROADCAST_ONESEG)
	if(lge_get_board_revno()>= HW_REV_B)
	msm_gpiomux_install(lge_1seg_blsp_configs, ARRAY_SIZE(lge_1seg_blsp_configs));	
	else if(lge_get_board_revno()== HW_REV_A)
	msm_gpiomux_install(lge_1seg_blsp_configs_revA, ARRAY_SIZE(lge_1seg_blsp_configs_revA));	
	else
	msm_gpiomux_install(lge_1seg_blsp_configs_rev0, ARRAY_SIZE(lge_1seg_blsp_configs_rev0));	
#endif  /* CONFIG_LGE_BROADCAST_ONESEG */
#if defined(CONFIG_MACH_LGE) && defined(CONFIG_SWITCH_SPK_RCV)
// sangman.park@lge.com(SOUND) 2013_7_17 added speaker switch mixer control  
	msm_gpiomux_install(msm_spk_rcv_det,
			ARRAY_SIZE(msm_spk_rcv_det));
#endif/* CONFIG_MACH_LGE&&CONFIG_SWITCH_SPK_RCV*/

#ifdef CONFIG_FMR_INTENNA
// real-wifi@lge.com(FM) 2013_8_23 added FM intenna GPIO control
	msm_gpiomux_install(fmr_intenna_det,
		ARRAY_SIZE(fmr_intenna_det));
#endif

}
