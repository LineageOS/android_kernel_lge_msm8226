/*
 * MAX14656 USB Charger Detector driver
 *
 * Copyright (C) 2014 LG Electronics, Inc
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <linux/power/max14656_charger_detector.h>

#define DEVICE_NAME		"max14656"

#ifndef BIT
#define BIT(x)	(1 << (x))
#endif

#define MAX14656_DEVICE_ID          0x00
#define MAX14656_INTERRUPT_1        0x01
#define MAX14656_INTERRUPT_2        0x02
#define MAX14656_STATUS_1           0x03
#define MAX14656_STATUS_2           0x04
#define MAX14656_INTMASK_1          0x05
#define MAX14656_INTMASK_2          0x06
#define MAX14656_CONTROL_1          0x07
#define MAX14656_CONTROL_2          0x08
#define MAX14656_CONTROL_3          0x09

#define INT_EN_REG_MASK             BIT(4)
#define CHG_TYPE_MASK              (BIT(3)|BIT(2)|BIT(1)|BIT(0))
#define DCD_TIMEOUT_MASK            BIT(7)

#define NULL_CHECK(p, err)  \
			if (!(p)) { \
				pr_err("FATAL (%s)\n", __func__); \
				return err; \
			}

#define USB_SDP_CHARGER 1
#define USB_DCP_CHARGER 2
#define USB_CDP_CHARGER 3

enum reg_address_idx {
	REG_00          = 0,
	REG_01          = 1,
	REG_02          = 2,
	REG_03          = 3,
	REG_04          = 4,
	REG_TOTAL_NUM   = 5,
};

static int max14656_read_reg(struct i2c_client *client, int reg, u8 *val)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev,
				"i2c read fail: can't read from %02x: %d\n",
				reg, ret);
		return ret;
	} else {
		*val = ret;
	}

	return 0;
}

static int max14656_write_reg(struct i2c_client *client, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		dev_err(&client->dev,
				"i2c write fail: can't write %02x to %02x: %d\n",
				val, reg, ret);
		return ret;
	}
	return 0;
}

static int max14656_read_block_reg(struct i2c_client *client, u8 reg,
		u8 length, u8 *val)
{
	int ret;

	ret = i2c_smbus_read_i2c_block_data(client, reg,
			length, val);

	if (ret < 0) {
		dev_err(&client->dev, "failed to block read reg 0x%x: %d\n",
				reg, ret);
		return ret;
	}

	return 0;
}

#if defined(CONFIG_LGE_PM_CHARGING_VZW_POWER_REQ)
int max14656_dcd_timeout = 0;
#endif

#ifdef CONFIG_LGE_PM
static int max14656_get_prop_chg_type(struct max14656_chip *chip)
{
	u8 val;
	int ret = 0;
	enum max14656_chg_type charger_type = NO_CHARGER;

	NULL_CHECK(chip, -EINVAL);
	ret = max14656_read_reg(chip->client, MAX14656_STATUS_1, &val);
	if (ret) {
		pr_err("fail to read MAX14656_STATUS_1. ret=%d\n", ret);
		charger_type = NO_CHARGER;
		return val;
	}
	val &= CHG_TYPE_MASK;
	pr_err("%s : val = %d\n", __func__, val);

	if (val == SDP_CHARGER) {
		charger_type = USB_SDP_CHARGER;
	} else if (val == CDP_CHARGER) {
		charger_type = USB_CDP_CHARGER;
	} else if (val == DCP_CHARGER) {
		charger_type = USB_DCP_CHARGER;
	} else {
		dev_err(&chip->client->dev, "unknown type charger: %d\n", val);
	}

	pr_err("%s : USB_CHG_TYPE = %d\n", __func__, charger_type);

	return charger_type;
	}
#endif

static void max14656_irq_worker(struct work_struct *work)
{
	struct max14656_chip *chip =
		container_of(work, struct max14656_chip, irq_work.work);

	u8 reg_address[REG_TOTAL_NUM];
	int ret = 0;

	ret = max14656_read_block_reg(chip->client, MAX14656_DEVICE_ID,
			REG_TOTAL_NUM, reg_address);

	pr_err("%s : REG_01:0x%02X, REG_02:0x%02X, REG_03:0x%02X, REG_04:0x%02X\n",
			__func__, reg_address[REG_01], reg_address[REG_02], reg_address[REG_03],
			reg_address[REG_04]);
#if defined(CONFIG_LGE_PM_CHARGING_VZW_POWER_REQ)
	if (reg_address[REG_01] != 0) {
		max14656_dcd_timeout = (reg_address[REG_01] & DCD_TIMEOUT_MASK) ? 1:0;
		pr_err("%s max14656_dcd_timeout = %d\n", __func__, max14656_dcd_timeout);
	}
#endif


#if 0
	u8 reg01, reg02, reg03, reg04, reg05, reg06, reg07, reg08, reg09;
	int ret =0;

	pr_err("%s : occured\n", __func__);

	ret = max14656_read_reg(chip->client, MAX14656_INTERRUPT_1, &reg01);
	if (ret)
		return;

	ret = max14656_read_reg(chip->client, MAX14656_INTERRUPT_2, &reg02);
	if (ret)
		return;

	pr_err("INTERRUPT_1_REG_01:0x%02x, INTERRUPT_2_REG_02:0x%02x\n", reg01, reg02);

	ret = max14656_read_reg(chip->client, MAX14656_STATUS_1, &reg03);
	if (ret)
		return;

	ret = max14656_read_reg(chip->client, MAX14656_STATUS_2, &reg04);
	if (ret)
		return;

	ret = max14656_read_reg(chip->client, MAX14656_INTMASK_1, &reg05);
	if (ret)
		return;

	ret = max14656_read_reg(chip->client, MAX14656_INTMASK_2, &reg06);
	if (ret)
		return;

	ret = max14656_read_reg(chip->client, MAX14656_CONTROL_1, &reg07);
	if (ret)
		return;

	ret = max14656_read_reg(chip->client, MAX14656_CONTROL_2, &reg08);
	if (ret)
		return;

	ret = max14656_read_reg(chip->client, MAX14656_CONTROL_3, &reg09);
	if (ret)
		return;

	pr_err("REG 03:0x%02x 04:0x%02x 05:0x%02x 06:0x%02x 07:0x%02x 08:0x%02x 09:0x%02x\n"
			, reg03, reg04, reg05, reg06, reg07, reg08, reg09);
#endif

#ifdef CONFIG_LGE_PM
	power_supply_changed(&chip->detect_psy);
#endif
}

static irqreturn_t max14656_irq(int irq, void *dev_id)
{
	struct max14656_chip *chip = dev_id;

	NULL_CHECK(chip, IRQ_NONE);
	schedule_delayed_work(&chip->irq_work, msecs_to_jiffies(100));

	return IRQ_HANDLED;
}

static int max14656_hw_init(struct max14656_chip *chip)
{
	int ret = 0;

	NULL_CHECK(chip, IRQ_NONE);
	ret= max14656_write_reg(chip->client, MAX14656_CONTROL_1, 0x19);
	if (ret) {
		pr_err("failed to set MAX14656_CONTROL_1 ret=%d\n", ret);
		return ret;
	}

	ret= max14656_write_reg(chip->client, MAX14656_INTMASK_1, 0xA1);
	if (ret) {
		pr_err("failed to set MAX14656_INTMASK_1 ret=%d\n", ret);
		return ret;
	}

	return ret;
}

#ifdef CONFIG_LGE_PM
static int max14656_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	struct max14656_chip *chip = container_of(psy,
			struct max14656_chip, detect_psy);

	NULL_CHECK(chip, -EINVAL);

	switch (psp) {
		case POWER_SUPPLY_PROP_USB_CHG_DETECT_DONE:
			val->intval = max14656_get_prop_chg_type(chip) ? 1 : 0;
			break;
		case POWER_SUPPLY_PROP_USB_CHG_TYPE:
			val->intval = max14656_get_prop_chg_type(chip);
			break;
		case POWER_SUPPLY_PROP_USB_DCD_TIMEOUT:
#if defined(CONFIG_LGE_PM_CHARGING_VZW_POWER_REQ)
			val->intval = max14656_dcd_timeout;
#else
			val->intval = 0;
#endif
			break;
		default:
			return -EINVAL;
	}

	return 0;
}
static enum power_supply_property max14656_battery_props[] = {
	POWER_SUPPLY_PROP_USB_CHG_DETECT_DONE,
	POWER_SUPPLY_PROP_USB_CHG_TYPE,
	POWER_SUPPLY_PROP_USB_DCD_TIMEOUT,
};
static char *pm_power_supplied_to[] = {
	"battery",
};

static struct power_supply max14656_ps = {
	.name = "max14656",
	.type = POWER_SUPPLY_TYPE_CHARGER_DETECTOR,
	.supplied_to = pm_power_supplied_to,
	.num_supplicants = ARRAY_SIZE(pm_power_supplied_to),
	.properties = max14656_battery_props,
	.num_properties = ARRAY_SIZE(max14656_battery_props),
	.get_property = max14656_get_property,
};
#endif

static int max14656_parse_dt(struct device_node *dev_node,
		struct max14656_chip *chip)
{
	int ret = 0;

	chip->int_gpio =
		of_get_named_gpio(dev_node, "max14656,int-gpio", 0);

	pr_debug("%s : int_gpio = %d.\n", __func__, chip->int_gpio);

	if (chip->int_gpio < 0) {
		pr_err("failed to get int-gpio.\n");
		ret = chip->int_gpio;
		return ret;
	}

	return ret;
}

static int __devinit max14656_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device_node *dev_node = client->dev.of_node;
	struct max14656_chip *chip;
	int ret = 0;
	int rc = 0;

	pr_err("%s:\n", __func__);

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	/*
	   chip->batt_psy = power_supply_get_by_name("battery");
	   if (!chip->batt_psy) {
	   pr_err("%s : batt_psy is not yet ready\n", __func__);
	   ret = -EPROBE_DEFER;
	   goto error;
	   }
	 */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_err("%s : i2c_check_functionality fail\n", __func__);
		return -EIO;
	}

	chip->client = client;

	/* need dts parser */
	if (dev_node) {
		ret = max14656_parse_dt(dev_node, chip);
		if (ret) {
			pr_err("failed to parse dt\n");
			goto error;
		}
	}

	ret = gpio_request_one(chip->int_gpio, GPIOF_DIR_IN,
			"max14656_int");
	if (ret) {
		pr_err("failed to request int_gpio\n");
		goto error;
	}
	chip->irq = gpio_to_irq(chip->int_gpio);
	pr_debug("int_gpio irq#=%d.\n", chip->irq);
	i2c_set_clientdata(client, NULL);

	ret = max14656_hw_init(chip);
	if (ret) {
		pr_err("max14656_hwinit failed.ret=%d\n", ret);
		goto err_hw_init;
	}

	INIT_DELAYED_WORK(&chip->irq_work, max14656_irq_worker);

	if (chip->irq) {
		ret = request_irq(chip->irq, max14656_irq,
				IRQF_TRIGGER_FALLING,
				"max14656_irq", chip);
		if (ret) {
			pr_err("request_irq %d failed\n", chip->irq);
			goto err_req_irq;
		}
		enable_irq_wake(chip->irq);
	}

	schedule_delayed_work(&chip->irq_work, msecs_to_jiffies(2000));

#ifdef CONFIG_LGE_PM
	chip->detect_psy = max14656_ps;

	rc = power_supply_register(&chip->client->dev, &chip->detect_psy);
	if (rc < 0) {
		pr_err("batt failed to register rc = %d\n", rc);
	}
#endif

	return 0;

err_req_irq:
err_hw_init:
	if (chip->int_gpio)
		gpio_free(chip->int_gpio);
error:
	kfree(chip);
	chip = NULL;
	pr_info("fail to probe\n");
	return ret;
}

static int max14656_remove(struct i2c_client *client)
{
	struct max14656_chip *chip = i2c_get_clientdata(client);

#ifdef CONFIG_LGE_PM
	power_supply_unregister(&chip->detect_psy);
#endif

	if (chip->irq)
		free_irq(chip->irq, chip);
	if (chip->int_gpio)
		gpio_free(chip->int_gpio);

	kfree(chip);
	chip = NULL;
	return 0;
}

static const struct i2c_device_id max14656_id[] = {
	{"max14656", 0},
	{}
};

static struct of_device_id max14656_match_table[] = {
	{ .compatible = "maxim,max14656", },
	{},
};

static struct i2c_driver max14656_i2c_driver = {
	.driver = {
		.name = "max14656",
		.owner = THIS_MODULE,
		.of_match_table = max14656_match_table,
	},
	.probe      = max14656_probe,
	.remove		= max14656_remove,
	.id_table   = max14656_id,
};

static int __init max14656_init(void)
{
	return i2c_add_driver(&max14656_i2c_driver);
}
module_init(max14656_init);

static void __exit max14656_exit(void)
{
	i2c_del_driver(&max14656_i2c_driver);
}
module_exit(max14656_exit);

MODULE_DESCRIPTION("MAX14656");
MODULE_LICENSE("GPL V2");
