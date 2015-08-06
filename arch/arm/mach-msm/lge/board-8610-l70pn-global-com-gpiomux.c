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

// Resreved Pin
#define MSM8x10_GPIO_END 102

#ifdef CONFIG_LGE_NFC_PN547_C2
/* NFC_MODE(70), NFC_VEN(71), NFC_CLK_OUT(78), NFC_IRQ(99) */
static int gpio_reserved_pin_rev_a[] = {
	12, 19, 35, 39, 44, 46, 47, 48, 57, 61, 62, 66, 74, 75, 76, 79, 88, 89, 90, 91,
	92, 93, 94, 95, 96, 97, 98, 100,
	MSM8x10_GPIO_END // This is included to notify the end of reserved GPIO configuration.
};
static int gpio_reserved_pin_rev_b[] = {
	12, 19, 35, 39, 44, 46, 47, 48, 57, 61, 62, 66, 74, 79, 88, 89, 90, 91,	92, 93,
	94, 95, 96, 97, 98, 100,
	MSM8x10_GPIO_END // This is included to notify the end of reserved GPIO configuration.
};
#else /* !CONFIG_LGE_NFC_PN547_C2 */
/* NFC_MODE(70), NFC_VEN(71), NFC_CLK_OUT(78), NFC_IRQ(99) */
static int gpio_reserved_pin_rev_a[] = {
	12, 19, 35, 39, 44, 46, 47, 48, 57, 61, 62, 66, 70, 71, 74, 75, 76, 78, 79, 88,
	89, 90, 91,	92, 93, 94, 95, 96, 97, 98, 99, 100,
	MSM8x10_GPIO_END // This is included to notify the end of reserved GPIO configuration.
};
static int gpio_reserved_pin_rev_b[] = {
	12, 19, 35, 39, 44, 46, 47, 48, 57, 61, 62, 66, 70, 71, 74, 78, 79, 88, 89, 90,
	91,	92, 93, 94, 95, 96, 97, 98, 99, 100,
	MSM8x10_GPIO_END // This is included to notify the end of reserved GPIO configuration.
};
#endif /* CONFIG_LGE_NFC_PN547_C2 */

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

static struct gpiomux_setting lcd_dsv_en_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_HIGH,
};

static struct gpiomux_setting lcd_dsv_en_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting lcd_ldo_en_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_HIGH,
};

static struct gpiomux_setting lcd_ldo_en_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting lcd_dsv_fd_ctrl_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_HIGH,
};

static struct gpiomux_setting lcd_dsv_fd_ctrl_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting lcd_reset_n_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_HIGH,
};

static struct gpiomux_setting lcd_reset_n_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_HIGH,
};

static struct gpiomux_setting lcd_bl_en_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_HIGH,
};

static struct gpiomux_setting lcd_bl_en_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
};

static struct gpiomux_setting lcd_pmode_en_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_HIGH,
};

static struct gpiomux_setting lcd_pmode_en_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
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

static struct gpiomux_setting hall_int_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting hall_int_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

static struct msm_gpiomux_config hall_int_configs[] __initdata = {
	{
		.gpio = 84,
		.settings = {
			[GPIOMUX_ACTIVE]	= &hall_int_act_cfg,
			[GPIOMUX_SUSPENDED]	= &hall_int_sus_cfg,
		},
	},
};

static struct gpiomux_setting accel_int_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting accel_int_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

static struct msm_gpiomux_config accel_int_configs[] __initdata = {
	{
		.gpio = 81,
		.settings = {
			[GPIOMUX_ACTIVE]	= &accel_int_act_cfg,
			[GPIOMUX_SUSPENDED]	= &accel_int_sus_cfg,
		},
	},
};

static struct gpiomux_setting compass_reset_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};

static struct gpiomux_setting compass_reset_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};

static struct msm_gpiomux_config compass_reset_configs[] __initdata = {
	{
		.gpio = 83,
		.settings = {
			[GPIOMUX_ACTIVE]	= &compass_reset_act_cfg,
			[GPIOMUX_SUSPENDED]	= &compass_reset_sus_cfg,
		},
	},
};

static struct msm_gpiomux_config msm_lcd_configs_rev_a[] __initdata = {
	{
		.gpio = 6,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcd_dsv_en_act_cfg,
			[GPIOMUX_SUSPENDED] = &lcd_dsv_en_sus_cfg,
		},
	},
	{
		.gpio = 7,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcd_ldo_en_act_cfg,
			[GPIOMUX_SUSPENDED] = &lcd_ldo_en_sus_cfg,
		},
	},
	{
		.gpio = 20,
		.settings = {
			[GPIOMUX_ACTIVE]	= &lcd_dsv_fd_ctrl_act_cfg,
			[GPIOMUX_SUSPENDED] = &lcd_dsv_fd_ctrl_sus_cfg,
		},
	},
	{
		.gpio = 41,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcd_reset_n_act_cfg,
			[GPIOMUX_SUSPENDED] = &lcd_reset_n_sus_cfg,
		},
	},
	{
		.gpio = 60,
		.settings = {
			[GPIOMUX_ACTIVE]	= &lcd_bl_en_act_cfg,
			[GPIOMUX_SUSPENDED] = &lcd_bl_en_sus_cfg,
		},
	},
};

static struct msm_gpiomux_config msm_lcd_configs_rev_b[] __initdata = {
	{
		.gpio = 6,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcd_dsv_en_act_cfg,
			[GPIOMUX_SUSPENDED] = &lcd_dsv_en_sus_cfg,
		},
	},
	{
		.gpio = 7,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcd_ldo_en_act_cfg,
			[GPIOMUX_SUSPENDED] = &lcd_ldo_en_sus_cfg,
		},
	},
	{
		.gpio = 20,
		.settings = {
			[GPIOMUX_ACTIVE]	= &lcd_dsv_fd_ctrl_act_cfg,
			[GPIOMUX_SUSPENDED] = &lcd_dsv_fd_ctrl_sus_cfg,
		},
	},
	{
		.gpio = 41,
		.settings = {
			[GPIOMUX_ACTIVE]    = &lcd_reset_n_act_cfg,
			[GPIOMUX_SUSPENDED] = &lcd_reset_n_sus_cfg,
		},
	},
	{
		.gpio = 60,
		.settings = {
			[GPIOMUX_ACTIVE]	= &lcd_bl_en_act_cfg,
			[GPIOMUX_SUSPENDED] = &lcd_bl_en_sus_cfg,
		},
	},
	{
		.gpio = 75,
		.settings = {
			[GPIOMUX_ACTIVE]	= &lcd_pmode_en_act_cfg,
			[GPIOMUX_SUSPENDED] = &lcd_pmode_en_sus_cfg,
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

static struct gpiomux_setting touch_reset_n_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_HIGH,
};

static struct gpiomux_setting touch_int_n_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};

static struct gpiomux_setting touch_ldo_en_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};

static struct gpiomux_setting touch_ldo_en_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};

static struct gpiomux_setting touch_ldo2_en_act_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};

static struct gpiomux_setting touch_ldo2_en_sus_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};

static struct msm_gpiomux_config touch_configs_rev_a[] __initdata = {
	{
		.gpio      = 0,		/* TOUCH_RESET_N */
		.settings = {
			[GPIOMUX_ACTIVE]    = &touch_reset_n_cfg,
			[GPIOMUX_SUSPENDED] = &touch_reset_n_cfg,
		},
	},
	{
		.gpio      = 1,		/* TOUCH_INT */
		.settings = {
			[GPIOMUX_ACTIVE]    = &touch_int_n_cfg,
			[GPIOMUX_SUSPENDED] = &touch_int_n_cfg,
		},
	},
	{
		.gpio      = 82,	/* TOUCH_LDO_EN */
		.settings = {
			[GPIOMUX_ACTIVE]    = &touch_ldo_en_act_cfg,
			[GPIOMUX_SUSPENDED] = &touch_ldo_en_sus_cfg,
		},
	},
};

static struct msm_gpiomux_config touch_configs_rev_b[] __initdata = {
	{
		.gpio      = 0,		/* TOUCH_RESET_N */
		.settings = {
			[GPIOMUX_ACTIVE]    = &touch_reset_n_cfg,
			[GPIOMUX_SUSPENDED] = &touch_reset_n_cfg,
		},
	},
	{
		.gpio      = 1,		/* TOUCH_INT */
		.settings = {
			[GPIOMUX_ACTIVE]    = &touch_int_n_cfg,
			[GPIOMUX_SUSPENDED] = &touch_int_n_cfg,
		},
	},
	{
		.gpio	   = 76,	/* TOUCH_LDO2_EN */
		.settings = {
			[GPIOMUX_ACTIVE]	= &touch_ldo2_en_act_cfg,
			[GPIOMUX_SUSPENDED] = &touch_ldo2_en_sus_cfg,
		},
	},
	{
		.gpio      = 82,	/* TOUCH_LDO_EN */
		.settings = {
			[GPIOMUX_ACTIVE]    = &touch_ldo_en_act_cfg,
			[GPIOMUX_SUSPENDED] = &touch_ldo_en_sus_cfg,
		},
	},
};

/*  LGE_CHANGE_S, [Camera][yt.jeon@lge.com], front Camera Module ID*/
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
		.gpio = 86,
		.settings = {
			[GPIOMUX_ACTIVE] = &main_cam_id_gpio_act_config,
			[GPIOMUX_SUSPENDED] = &main_cam_id_gpio_sus_config,
		}
	}
};
#endif /* CONFIG_MACH_LGE */
/*  LGE_CHANGE_E, [Camera][yt.jeon@lge.com], front Camera Module ID*/

/*  LGE_CHANGE_S, [NFC][seunghoon65.lee@lge.com], NFC Bring up*/
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
		.gpio      = 71,
		.settings = {
			[GPIOMUX_ACTIVE]    = &nfc_pn547_ven_cfg,
			[GPIOMUX_SUSPENDED] = &nfc_pn547_ven_cfg,
		},
	},
	{
		/* IRQ */
		.gpio      = 99,
		.settings = {
			[GPIOMUX_ACTIVE]    = &nfc_pn547_irq_cfg,
			[GPIOMUX_SUSPENDED] = &nfc_pn547_irq_cfg,
		},
	},
	{
		/* MODE *//* WAKE */
		.gpio      = 70,
		.settings = {
			[GPIOMUX_ACTIVE]    = &nfc_pn547_mode_cfg,
			[GPIOMUX_SUSPENDED] = &nfc_pn547_mode_cfg,
		},
	},
};

#endif
/*  LGE_CHANGE_E, [NFC][seunghoon65.lee@lge.com], NFC Bring up*/

void __init msm8610_init_gpiomux(void)
{
	int rc;
	int gpio_index = 0;
	hw_rev_type revision = lge_get_board_revno();

	rc = msm_gpiomux_init_dt();
	if (rc) {
		pr_err("%s failed %d\n", __func__, rc);
		return;
	}

	switch( revision ) {
		case HW_REV_0:
		case HW_REV_A:
			for ( gpio_index = 0 ; gpio_reserved_pin_rev_a[gpio_index] < MSM8x10_GPIO_END ; gpio_index++ ) {
				gpio_func_reserved_pin_config.gpio = gpio_reserved_pin_rev_a[gpio_index];
				msm_gpiomux_install(&gpio_func_reserved_pin_config, 1);
			}

			msm_gpiomux_install(touch_configs_rev_a, ARRAY_SIZE(touch_configs_rev_a));
			msm_gpiomux_install_nowrite(msm_lcd_configs_rev_a, ARRAY_SIZE(msm_lcd_configs_rev_a));
			break;
		case HW_REV_B:
			for ( gpio_index = 0 ; gpio_reserved_pin_rev_b[gpio_index] < MSM8x10_GPIO_END ; gpio_index++ ) {
				gpio_func_reserved_pin_config.gpio = gpio_reserved_pin_rev_b[gpio_index];
				msm_gpiomux_install(&gpio_func_reserved_pin_config, 1);
			}

			msm_gpiomux_install(touch_configs_rev_b, ARRAY_SIZE(touch_configs_rev_b));
			msm_gpiomux_install_nowrite(msm_lcd_configs_rev_b, ARRAY_SIZE(msm_lcd_configs_rev_b));
			break;
		default:
			for ( gpio_index = 0 ; gpio_reserved_pin_rev_b[gpio_index] < MSM8x10_GPIO_END ; gpio_index++ ) {
				gpio_func_reserved_pin_config.gpio = gpio_reserved_pin_rev_b[gpio_index];
				msm_gpiomux_install(&gpio_func_reserved_pin_config, 1);
			}

			msm_gpiomux_install(touch_configs_rev_b, ARRAY_SIZE(touch_configs_rev_b));
			msm_gpiomux_install_nowrite(msm_lcd_configs_rev_b, ARRAY_SIZE(msm_lcd_configs_rev_b));
			break;
	}

	msm_gpiomux_install(msm_blsp_configs, ARRAY_SIZE(msm_blsp_configs));
	msm_gpiomux_install(wcnss_5wire_interface, ARRAY_SIZE(wcnss_5wire_interface));
	msm_gpiomux_install(msm_keypad_configs,	ARRAY_SIZE(msm_keypad_configs));
	msm_gpiomux_install(sd_card_det, ARRAY_SIZE(sd_card_det));
	msm_gpiomux_install(msm_sensor_configs, ARRAY_SIZE(msm_sensor_configs));
	msm_gpiomux_install(main_cam_id_gpio, ARRAY_SIZE(main_cam_id_gpio));
	msm_gpiomux_install(hall_int_configs, ARRAY_SIZE(hall_int_configs));
	msm_gpiomux_install(accel_int_configs, ARRAY_SIZE(accel_int_configs));
	msm_gpiomux_install(compass_reset_configs, ARRAY_SIZE(compass_reset_configs));
	/*  LGE_CHANGE_S, [NFC][taesik.kim@lge.com], NFC Bring up */
#ifdef CONFIG_LGE_NFC_PN547_C2
	msm_gpiomux_install(msm_nfc_configs, ARRAY_SIZE(msm_nfc_configs));
#endif
/*  LGE_CHANGE_E, [NFC][taesik.kim@lge.com], NFC Bring up */
}
