/*
 * TI LM3697 Backlight Driver
 *
 * Copyright 2014 Texas Instruments
 *
 * Author: Milo Kim <milo.kim@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_data/lm3697_bl.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/delay.h>

/* Registers */
#define LM3697_REG_OUTPUT_CFG		0x10
#define LM3697_REG_BRT_CFG			0x16
#define LM3697_REG_IMAX_A			0x17
#define LM3697_REG_IMAX_B			0x18
#define LM3697_REG_BOOST_CTRL		0x1A
#define LM3697_REG_PWM_CFG			0x1C
#define LM3697_REG_BRT_A_LSB		0x20
#define LM3697_REG_BRT_A_MSB		0x21
#define LM3697_REG_BRT_B_LSB		0x22
#define LM3697_REG_BRT_B_MSB		0x23
#define LM3697_REG_ENABLE			0x24

#define LM3697_BRT_LSB_MASK			(BIT(0) | BIT(1) | BIT(2))
#define LM3697_BRT_MSB_SHIFT		3

/* Other definitions */
#define LM3697_PWM_ID				1
#define LM3697_MAX_REGISTERS		0xB4
#define LM3697_MAX_STRINGS			3
#define LM3697_MAX_BRIGHTNESS		2047
#define LM3697_IMAX_OFFSET			6
#define LM3697_DEFAULT_NAME			"lcd-backlight"
#define LM3697_DEFAULT_PWM			"lm3697-backlight"

enum lm3697_bl_ctrl_mode {
	BL_REGISTER_BASED,
	BL_PWM_BASED,
};

#define BL_OFF	0
#define BL_ON	1

static struct backlight_device *lm3697_device;
static int backlight_status = BL_OFF;
static int cur_main_lcd_level = LM3697_MAX_BRIGHTNESS;

/*
 * struct lm3697_bl_chip
 * @dev: Parent device structure
 * @regmap: Used for I2C register access
 * @pdata: LM3697 platform data
 */
struct lm3697_bl_chip {
	struct device *dev;
	struct regmap *regmap;
	struct lm3697_platform_data *pdata;
};

/*
 * struct lm3697_bl
 * @bank_id: Control bank ID. BANK A or BANK A and B
 * @bl_dev: Backlight device structure
 * @chip: LM3697 backlight chip structure for low level access
 * @bl_pdata: LM3697 backlight platform data
 * @mode: Backlight control mode
 * @pwm: PWM device structure. Only valid in PWM control mode
 * @pwm_name: PWM device name
 */
struct lm3697_bl {
	int bank_id;
	struct backlight_device *bl_dev;
	struct lm3697_bl_chip *chip;
	struct lm3697_backlight_platform_data *bl_pdata;
	enum lm3697_bl_ctrl_mode mode;
	struct pwm_device *pwm;
	char pwm_name[20];
};

static int lm3697_bl_read_byte(struct lm3697_bl_chip *chip, u8 reg, unsigned int *data)
{
	return regmap_read(chip->regmap, reg, data);
}

static int lm3697_bl_write_byte(struct lm3697_bl_chip *chip, u8 reg, u8 data)
{
	return regmap_write(chip->regmap, reg, data);
}

static int lm3697_bl_update_bits(struct lm3697_bl_chip *chip, u8 reg, u8 mask,
				 u8 data)
{
	return regmap_update_bits(chip->regmap, reg, mask, data);
}

static int lm3697_bl_enable(struct lm3697_bl *lm3697_bl, int enable)
{
	pr_info("## %s[%d]\n", __func__, enable);
	backlight_status = enable;

	return lm3697_bl_update_bits(lm3697_bl->chip, LM3697_REG_ENABLE,
				     BIT(lm3697_bl->bank_id),
				     enable << lm3697_bl->bank_id);
}

static void lm3697_bl_pwm_ctrl(struct lm3697_bl *lm3697_bl, int br, int max_br)
{
	struct pwm_device *pwm;
	unsigned int duty, period;

	/* Request a PWM device with the consumer name */
	if (!lm3697_bl->pwm) {
		pwm = pwm_request(LM3697_PWM_ID, lm3697_bl->pwm_name);
		if (IS_ERR(pwm)) {
			dev_err(lm3697_bl->chip->dev,
				"Can not get PWM device: %s\n",
				lm3697_bl->pwm_name);
			return;
		}
		lm3697_bl->pwm = pwm;
	}

	period = lm3697_bl->bl_pdata->pwm_period;
	duty = br * period / max_br;

	pwm_config(lm3697_bl->pwm, duty, period);
	if (duty)
		pwm_enable(lm3697_bl->pwm);
	else
		pwm_disable(lm3697_bl->pwm);
}

static int lm3697_bl_set_brightness(struct lm3697_bl *lm3697_bl, int brightness)
{
	int ret;
	u8 data;
	u8 reg_lsb[] = { LM3697_REG_BRT_A_LSB, LM3697_REG_BRT_B_LSB, };
	u8 reg_msb[] = { LM3697_REG_BRT_A_MSB, LM3697_REG_BRT_B_MSB, };
	int cal_level = 0;

	/* change the brightness at the mapping table */
	if (brightness != 0) {
		if (brightness >= lm3697_bl->bl_pdata->blmap_size)
			brightness = lm3697_bl->bl_pdata->blmap_size - 1;

		if (lm3697_bl->bl_pdata->blmap)
			cal_level = lm3697_bl->bl_pdata->blmap[brightness];
		else
			cal_level = brightness;
	}

	if (lm3697_bl->mode == BL_PWM_BASED)
		lm3697_bl_pwm_ctrl(lm3697_bl, cal_level, lm3697_bl->bl_pdata->max_brightness);

	if (lm3697_bl->bl_pdata->max_brightness > 255) {
		/* Two registers(LSB and MSB) are updated for 11-bit dimming */
		data = cal_level & LM3697_BRT_LSB_MASK;
		ret = lm3697_bl_update_bits(lm3697_bl->chip,
					    reg_lsb[lm3697_bl->bank_id],
					    LM3697_BRT_LSB_MASK, data);
		if (ret)
			return ret;

		data = (cal_level >> LM3697_BRT_MSB_SHIFT) & 0xFF;
	} else {
		/* Only MSB is written for 8-bit dimming */
		data = cal_level & 0xFF;
	}

	printk("## %s[br:%d][cal_lvl:%d]\n", __func__, brightness, cal_level);

	return lm3697_bl_write_byte(lm3697_bl->chip,
				    reg_msb[lm3697_bl->bank_id], data);
}

static int lm3697_bl_set_boost_control(struct lm3697_bl *lm3697_bl)
{
	return lm3697_bl_write_byte(lm3697_bl->chip, LM3697_REG_BOOST_CTRL,
				    0x04);
}

static int lm3697_bl_set_brightness_configure(struct lm3697_bl *lm3697_bl)
{
	return lm3697_bl_write_byte(lm3697_bl->chip, LM3697_REG_BRT_CFG,
				    lm3697_bl->bl_pdata->mapping_mode);
}

static int lm3697_bl_set_ctrl_mode(struct lm3697_bl *lm3697_bl)
{
	struct lm3697_backlight_platform_data *pdata = lm3697_bl->bl_pdata;
	int bank_id = lm3697_bl->bank_id;

	if (pdata->pwm_period > 0)
		lm3697_bl->mode = BL_PWM_BASED;
	else
		lm3697_bl->mode = BL_REGISTER_BASED;

	/* Control bank assignment for PWM control */
	if (lm3697_bl->mode == BL_PWM_BASED) {
		snprintf(lm3697_bl->pwm_name, sizeof(lm3697_bl->pwm_name),
			 "%s", LM3697_DEFAULT_PWM);

		return lm3697_bl_update_bits(lm3697_bl->chip,
					     LM3697_REG_PWM_CFG, BIT(bank_id),
					     1 << bank_id);
	}

	return 0;
}

static int lm3697_bl_string_configure(struct lm3697_bl *lm3697_bl)
{
	int bank_id = lm3697_bl->bank_id;
	int is_detected = 0;
	int i, ret;

	/* Control bank assignment for backlight string configuration */
	for (i = 0; i < LM3697_MAX_STRINGS; i++) {
		if (test_bit(i, &lm3697_bl->bl_pdata->bl_string)) {
			ret = lm3697_bl_update_bits(lm3697_bl->chip,
						    LM3697_REG_OUTPUT_CFG,
						    BIT(i), bank_id << i);
			if (ret)
				return ret;

			is_detected = 1;
		}
	}

	if (!is_detected) {
		dev_err(lm3697_bl->chip->dev, "No backlight string found\n");
		return -EINVAL;
	}

	return 0;
}

static int lm3697_bl_set_current(struct lm3697_bl *lm3697_bl)
{
	u8 reg[] = { LM3697_REG_IMAX_A, LM3697_REG_IMAX_B, };

	return lm3697_bl_write_byte(lm3697_bl->chip, reg[lm3697_bl->bank_id],
				    lm3697_bl->bl_pdata->imax);
}

static int lm3697_bl_configure(struct lm3697_bl *lm3697_bl)
{
	int ret;

	ret = lm3697_bl_set_boost_control(lm3697_bl);
	if (ret)
		return ret;

	ret = lm3697_bl_set_brightness_configure(lm3697_bl);
	if (ret)
		return ret;

	ret = lm3697_bl_set_ctrl_mode(lm3697_bl);
	if (ret)
		return ret;

	ret = lm3697_bl_string_configure(lm3697_bl);
	if (ret)
		return ret;

	return lm3697_bl_set_current(lm3697_bl);
}

static void lm3697_backlight_on(struct lm3697_bl *lm3697_bl)
{
	if (backlight_status == BL_OFF) {
		pr_info("## %s with level %d\n",
			__func__, lm3697_bl->bl_dev->props.brightness);

		gpio_set_value(lm3697_bl->chip->pdata->en_gpio, 1);
		mdelay(1);
		lm3697_bl_configure(lm3697_bl);
		lm3697_bl_enable(lm3697_bl, BL_ON);

		backlight_status = BL_ON;
	}
	lm3697_bl_set_brightness(lm3697_bl, lm3697_bl->bl_dev->props.brightness);
}

static void lm3697_backlight_off(struct lm3697_bl *lm3697_bl)
{
	if (backlight_status == BL_OFF)
		return;

	lm3697_bl_set_brightness(lm3697_bl, 0);

	gpio_set_value(lm3697_bl->chip->pdata->en_gpio, 0);
	msleep(6);
	backlight_status = BL_OFF;

	pr_info("## %s\n", __func__);
}

void lm3697_lcd_backlight_set_level(int level)
{
	struct lm3697_bl *lm3697_bl;

	if (lm3697_device == NULL) {
		pr_err("## %s: lm3697 is not registered\n", __func__);
		return;
	}

	if (cur_main_lcd_level == level)
		return;

	cur_main_lcd_level = level;

	lm3697_bl = bl_get_data(lm3697_device);
	lm3697_bl->bl_dev->props.brightness = level;

	if (level == 0) {
		lm3697_backlight_off(lm3697_bl);
	} else {
		lm3697_backlight_on(lm3697_bl);
	}
}
EXPORT_SYMBOL(lm3697_lcd_backlight_set_level);

static int lm3697_bl_update_status(struct backlight_device *bl_dev)
{
	lm3697_lcd_backlight_set_level(bl_dev->props.brightness);

	return 0;
}

static int lm3697_bl_get_brightness(struct backlight_device *bl_dev)
{
	return bl_dev->props.brightness;
}

static const struct backlight_ops lm3697_bl_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = lm3697_bl_update_status,
	.get_brightness = lm3697_bl_get_brightness,
};

static enum lm3697_max_current lm3697_get_current_code(u8 imax_mA)
{
	const enum lm3697_max_current imax_table[] = {
		LM3697_IMAX_6mA,  LM3697_IMAX_7mA,  LM3697_IMAX_8mA,
		LM3697_IMAX_9mA,  LM3697_IMAX_10mA, LM3697_IMAX_11mA,
		LM3697_IMAX_12mA, LM3697_IMAX_13mA, LM3697_IMAX_14mA,
		LM3697_IMAX_15mA, LM3697_IMAX_16mA, LM3697_IMAX_17mA,
		LM3697_IMAX_18mA, LM3697_IMAX_19mA, LM3697_IMAX_20mA,
		LM3697_IMAX_21mA, LM3697_IMAX_22mA, LM3697_IMAX_23mA,
		LM3697_IMAX_24mA, LM3697_IMAX_25mA, LM3697_IMAX_26mA,
		LM3697_IMAX_27mA, LM3697_IMAX_28mA, LM3697_IMAX_29mA,
	};

	/*
	 * Convert milliampere to appropriate enum code value.
	 * Input range : 5 ~ 30mA
	 */

	if (imax_mA <= 5)
		return LM3697_IMAX_5mA;

	if (imax_mA >= 30)
		return LM3697_IMAX_30mA;

	return imax_table[imax_mA - LM3697_IMAX_OFFSET];
}

/* This helper funcion is moved from linux-v3.9 */
static inline int _of_get_child_count(const struct device_node *np)
{
	struct device_node *child;
	int num = 0;

	for_each_child_of_node(np, child)
		num++;

	return num;
}

static int lm3697_bl_parse_dt(struct device *dev, struct lm3697_bl_chip *chip)
{
	struct lm3697_platform_data *pdata;
	struct lm3697_backlight_platform_data *bl_pdata;
	struct device_node *node = dev->of_node;
	struct device_node *child;
	int num_backlights;
	int i = 0;
	unsigned int imax_mA;
	u32 *array;
	int j;

	if (!node) {
		dev_err(dev, "No device node exists\n");
		return -ENODEV;
	}

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->en_gpio = of_get_named_gpio(node, "lm3697,lcd_bl_en", 0);
	if (pdata->en_gpio < 0)
		return pdata->en_gpio;

	num_backlights = _of_get_child_count(node);
	if (num_backlights == 0) {
		dev_err(dev, "No backlight strings\n");
		return -EINVAL;
	}

	bl_pdata = devm_kzalloc(dev, sizeof(*bl_pdata) * num_backlights,
				GFP_KERNEL);
	if (!bl_pdata)
		return -ENOMEM;

	for_each_child_of_node(node, child) {
		of_property_read_string(child, "backlight-name",
					&bl_pdata[i].name);

		/* Make backlight strings */
		bl_pdata[i].bl_string = 0;
		if (of_property_read_bool(child, "lm3697,hvled1_used"))
			bl_pdata[i].bl_string |= LM3697_HVLED1;
		if (of_property_read_bool(child, "lm3697,hvled2_used"))
			bl_pdata[i].bl_string |= LM3697_HVLED2;
		if (of_property_read_bool(child, "lm3697,hvled3_used"))
			bl_pdata[i].bl_string |= LM3697_HVLED3;

		imax_mA = 0;
		of_property_read_u32(child, "lm3697,max_current_milliamp", &imax_mA);
		bl_pdata[i].imax = lm3697_get_current_code(imax_mA);

		of_property_read_u32(child, "lm3697,init_brightness",
					&bl_pdata[i].init_brightness);

		of_property_read_u32(child, "lm3697,min_brightness",
					&bl_pdata[i].min_brightness);

		of_property_read_u32(child, "lm3697,max_brightness",
					&bl_pdata[i].max_brightness);

		of_property_read_u32(child, "lm3697,mapping_mode",
					&bl_pdata[i].mapping_mode);

		of_property_read_u32(child, "lm3697,blmap_size",
					&bl_pdata[i].blmap_size);

		if (bl_pdata[i].blmap_size) {
			array = kzalloc(sizeof(u32) * bl_pdata[i].blmap_size, GFP_KERNEL);
			if (!array)
				return -ENOMEM;

			of_property_read_u32_array(child, "lm3697,blmap",
						array, bl_pdata[i].blmap_size);

			bl_pdata[i].blmap = kzalloc(sizeof(u16) * bl_pdata[i].blmap_size, GFP_KERNEL);
			if (!bl_pdata[i].blmap) {
				kfree(array);
				return -ENOMEM;
			}

			for (j=0; j < bl_pdata[i].blmap_size; j++)
				bl_pdata[i].blmap[j] = (u16)array[j];

			kfree(array);
		} else {
			pr_err("## %s: blmap_size is not defined\n", __func__);
		}

		/* PWM mode */
		of_property_read_u32(child, "lm3697,pwm_period",
					&bl_pdata[i].pwm_period);

		printk("===============================================================\n");
		printk("[LM3697] [gpio:%d][max_current:%dmA][mapping_mode:%d]\n",
	        pdata->en_gpio, imax_mA, bl_pdata[i].mapping_mode);
		printk("[LM3697] [max_brightness:%d][init_brightness:%d][pwm_period:%d]\n",
	        bl_pdata[i].max_brightness, bl_pdata[i].init_brightness, bl_pdata[i].pwm_period);
		printk("===============================================================\n");

		i++;
	}

	pdata->bl_pdata = bl_pdata;
	pdata->num_backlights = num_backlights;
	chip->pdata = pdata;

	return 0;
}

static int lm3697_bl_enable_hw(struct lm3697_bl_chip *chip)
{
	return gpio_request_one(chip->pdata->en_gpio, GPIOF_OUT_INIT_HIGH,
				"lm3697_hwen");
}

static void lm3697_bl_disable_hw(struct lm3697_bl_chip *chip)
{
	if (chip->pdata->en_gpio) {
		gpio_set_value(chip->pdata->en_gpio, 0);
		gpio_free(chip->pdata->en_gpio);
	}
}

static int lm3697_bl_add_device(struct lm3697_bl *lm3697_bl)
{
	struct backlight_device *bl_dev;
	struct backlight_properties props;
	struct lm3697_backlight_platform_data *pdata = lm3697_bl->bl_pdata;
	char name[20];

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_PLATFORM;
	props.brightness = pdata ? pdata->init_brightness : 0;
	props.max_brightness = pdata->max_brightness;

	/* Backlight device name */
	if (!pdata->name)
		snprintf(name, sizeof(name), "%s:%d", LM3697_DEFAULT_NAME,
			 lm3697_bl->bank_id);
	else
		snprintf(name, sizeof(name), "%s", pdata->name);

	bl_dev = backlight_device_register(name, lm3697_bl->chip->dev,
					   lm3697_bl, &lm3697_bl_ops, &props);
	if (IS_ERR(bl_dev))
		return PTR_ERR(bl_dev);

	lm3697_bl->bl_dev = bl_dev;

	lm3697_device = bl_dev;

	return 0;
}

static struct lm3697_bl *lm3697_bl_register(struct lm3697_bl_chip *chip)
{
	struct lm3697_backlight_platform_data *pdata = chip->pdata->bl_pdata;
	struct lm3697_bl *lm3697_bl, *each;
	int num_backlights = chip->pdata->num_backlights;
	int i, ret;

	lm3697_bl = devm_kzalloc(chip->dev, sizeof(*lm3697_bl) * num_backlights,
				 GFP_KERNEL);
	if (!lm3697_bl)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < num_backlights; i++) {
		each = lm3697_bl + i;
		each->bank_id = i;
		each->chip = chip;
		each->bl_pdata = pdata + i;

		ret = lm3697_bl_configure(lm3697_bl);
		if (ret) {
			dev_err(chip->dev, "Backlight config err: %d\n", ret);
			goto err;
		}

		ret = lm3697_bl_add_device(each);
		if (ret) {
			dev_err(chip->dev, "Backlight device err: %d\n", ret);
			goto cleanup_backlights;
		}

		backlight_update_status(each->bl_dev);
	}

	return lm3697_bl;

cleanup_backlights:
	while (--i >= 0) {
		each = lm3697_bl + i;
		backlight_device_unregister(each->bl_dev);
	}
err:
	return ERR_PTR(ret);
}

static int lm3697_bl_unregister(struct lm3697_bl *lm3697_bl)
{
	struct lm3697_bl *each;
	struct backlight_device *bl_dev;
	int num_backlights = lm3697_bl->chip->pdata->num_backlights;
	int i;

	for (i = 0; i < num_backlights; i++) {
		each = lm3697_bl + i;

		bl_dev = each->bl_dev;
		bl_dev->props.brightness = 0;
		backlight_update_status(bl_dev);
		backlight_device_unregister(bl_dev);
	}

	return 0;
}

static struct regmap_config lm3697_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = LM3697_MAX_REGISTERS,
};

static ssize_t lm3697_bl_show_level(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lm3697_bl *lm3697_bl = i2c_get_clientdata(client);
	u8 reg_lsb[] = { LM3697_REG_BRT_A_LSB, LM3697_REG_BRT_B_LSB, };
	u8 reg_msb[] = { LM3697_REG_BRT_A_MSB, LM3697_REG_BRT_B_MSB, };
	int lsb_val, msb_val;

	lm3697_bl_read_byte(lm3697_bl->chip, reg_lsb[lm3697_bl->bank_id], &lsb_val);
	lm3697_bl_read_byte(lm3697_bl->chip, reg_msb[lm3697_bl->bank_id], &msb_val);

	return snprintf(buf, PAGE_SIZE, "%d\n",
			(msb_val << LM3697_BRT_MSB_SHIFT) | lsb_val);
}

static ssize_t lm3697_bl_store_level(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lm3697_bl *lm3697_bl = i2c_get_clientdata(client);
	u8 reg_lsb[] = { LM3697_REG_BRT_A_LSB, LM3697_REG_BRT_B_LSB, };
	u8 reg_msb[] = { LM3697_REG_BRT_A_MSB, LM3697_REG_BRT_B_MSB, };
	u8 data;
	int level;

	level = simple_strtoul(buf, NULL, 10);

	if (lm3697_bl->bl_pdata->max_brightness > 255) {
		/* Two registers(LSB and MSB) are updated for 11-bit dimming */
		data = level & LM3697_BRT_LSB_MASK;
		lm3697_bl_update_bits(lm3697_bl->chip,
				reg_lsb[lm3697_bl->bank_id],
				LM3697_BRT_LSB_MASK, data);

		data = (level >> LM3697_BRT_MSB_SHIFT) & 0xFF;
	} else {
		/* Only MSB is written for 8-bit dimming */
		data = level & 0xFF;
	}

	lm3697_bl_write_byte(lm3697_bl->chip,
			reg_msb[lm3697_bl->bank_id], data);

	return count;
}

DEVICE_ATTR(bl_level, 0644, lm3697_bl_show_level, lm3697_bl_store_level);

static int dump_registers[] = { LM3697_REG_OUTPUT_CFG, \
								LM3697_REG_BRT_CFG, \
								LM3697_REG_IMAX_A, \
								LM3697_REG_BOOST_CTRL, \
								LM3697_REG_BRT_A_LSB, \
								LM3697_REG_BRT_A_MSB, \
								LM3697_REG_ENABLE };

static ssize_t lm3697_bl_show_dump_reg(struct device* dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lm3697_bl *lm3697_bl = i2c_get_clientdata(client);
	int ret = 0;
	int reg_val, i;

	ret = snprintf(buf, PAGE_SIZE, "======= LM3697 REG =======\n");
	for (i=0; i<ARRAY_SIZE(dump_registers); i++) {
		lm3697_bl_read_byte(lm3697_bl->chip, dump_registers[i], &reg_val);
		ret += snprintf(buf+ret, PAGE_SIZE, "REG[0x%2x]: 0x%x\n",
				dump_registers[i], reg_val);
	}
	ret += snprintf(buf+ret, PAGE_SIZE, "==========================\n");

	return ret;
}

DEVICE_ATTR(dump_reg, 0444, lm3697_bl_show_dump_reg, NULL);

static int lm3697_bl_probe(struct i2c_client *cl,
			   const struct i2c_device_id *id)
{
	struct device *dev = &cl->dev;
	struct lm3697_platform_data *pdata = dev_get_platdata(dev);
	struct lm3697_bl_chip *chip;
	struct lm3697_bl *lm3697_bl;
	int ret;
	int i;
	struct lm3697_backlight_platform_data *tmp;

	printk("## %s -->>\n", __func__);

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = dev;
	chip->pdata = pdata;

	chip->regmap = devm_regmap_init_i2c(cl, &lm3697_regmap_config);
	if (IS_ERR(chip->regmap))
		return PTR_ERR(chip->regmap);

	if (!chip->pdata) {
		if (IS_ENABLED(CONFIG_OF))
			ret = lm3697_bl_parse_dt(dev, chip);
		else
			return -ENODEV;

		if (ret)
			return ret;
	}

	ret = lm3697_bl_enable_hw(chip);
	if (ret) {
		goto err_enable_hw;
	}

	lm3697_bl = lm3697_bl_register(chip);
	if (IS_ERR(lm3697_bl)) {
		ret = PTR_ERR(lm3697_bl);
		goto err_register;
	}

	i2c_set_clientdata(cl, lm3697_bl);

	device_create_file(&cl->dev, &dev_attr_bl_level);
	device_create_file(&cl->dev, &dev_attr_dump_reg);

	printk("## %s --<<\n", __func__);

	return 0;

err_register:
	lm3697_bl_disable_hw(chip);
err_enable_hw:
	for (i=0; i < chip->pdata->num_backlights; i++) {
		tmp = chip->pdata->bl_pdata + i;
		kfree(tmp->blmap);
	}

	return ret;
}

static int lm3697_bl_remove(struct i2c_client *cl)
{
	struct lm3697_bl *lm3697_bl = i2c_get_clientdata(cl);

	lm3697_bl_unregister(lm3697_bl);
	lm3697_bl_disable_hw(lm3697_bl->chip);

	return 0;
}

static const struct i2c_device_id lm3697_bl_ids[] = {
	{ "lm3697", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lm3697_bl_ids);

#ifdef CONFIG_OF
static const struct of_device_id lm3697_bl_of_match[] = {
	{ .compatible = "ti,lm3697", },
	{ }
};
MODULE_DEVICE_TABLE(of, lm3697_bl_of_match);
#endif

static struct i2c_driver lm3697_bl_driver = {
	.probe = lm3697_bl_probe,
	.remove = lm3697_bl_remove,
	.driver = {
		.name = "lm3697",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(lm3697_bl_of_match),
#endif
	},
	.id_table = lm3697_bl_ids,
};
module_i2c_driver(lm3697_bl_driver);

MODULE_DESCRIPTION("TI LM3697 Backlight Driver");
MODULE_AUTHOR("Milo Kim");
MODULE_LICENSE("GPL");
