/* lge_ts_misc.c
 *
 * Copyright (C) 2013 LGE.
 *
 * Author: WX-BSP-TS@lge.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif
#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#define MISC_DRIVER_NAME "lge_ts_misc"

#define TOUCH_INFO_MSG(fmt, args...) 	printk(KERN_ERR "[Touch Misc] " fmt, ##args)

#define PRESS_KEY				1
#define RELEASE_KEY				0

struct lge_ts_misc_info {
	struct i2c_client		*client;
	struct input_dev		*input_dev;
	char 				phys[32];
#ifdef CONFIG_FB
	struct notifier_block	fb_notifier;
#endif
	int					irq_wake;
	int					gpio_int;
	int					vdd_on;
	int					vio_l28_on;
	int					vio_l6_on;
	int					vio_lvs1_on;
	int					vdd_voltage;
	int					vio_l28_voltage;
	int					vio_l6_voltage;
	int					vio_lvs1_voltage;
	struct regulator		*vdd;
	struct regulator		*vio_l28;
	struct regulator		*vio_l6;
	struct regulator		*vio_lvs1;
};

#ifdef CONFIG_OF

enum misc_dt_type {
	DT_U8,
	DT_U16,
	DT_U32,
	DT_GPIO,
	DT_STRING,
};

struct misc_dt_map {
	const char			*name;
	void					*data;
	enum misc_dt_type	type;
	int					def_value;
};

static int lge_ts_misc_parse_dt(struct device *dev, struct lge_ts_misc_info *info)
{
	struct device_node *np = dev->of_node;
	int ret = 0;
	u32 val;
	struct misc_dt_map *itr;
	struct misc_dt_map map[] = {
		{ "irq_wake",			&info->irq_wake,			DT_U32, 0 },
		{ "gpio_int",			&info->gpio_int,			DT_U32, 0 },
		{ "vdd_on",			&info->vdd_on,			DT_U32, 0 },
		{ "vio_l28_on",		&info->vio_l28_on,		DT_U32, 0 },
		{ "vio_l6_on",			&info->vio_l6_on,		DT_U32,	0 },
		{ "vio_lvs1_on",		&info->vio_lvs1_on,		DT_U32, 0 },
		{ "vdd_voltage",		&info->vdd_voltage,		DT_U32, 0 },
		{ "vio_l28_voltage",	&info->vio_l28_voltage,	DT_U32, 0 },
		{ "vio_l6_voltage",	&info->vio_l6_voltage,	DT_U32,	0 },
		{ "vio_lvs1_voltage",	&info->vio_lvs1_voltage,	DT_U32, 0 },
		{ NULL,				NULL,					0,		0 },
	};

	for (itr = map; itr->name; ++itr) {
		val = 0;
		ret = 0;

		switch (itr->type) {
		case DT_U32:
			if ((ret = of_property_read_u32(np, itr->name, &val)) == 0) {
				*((u32 *) itr->data) = val;
				TOUCH_INFO_MSG("DT : %s = %d \n", itr->name, val);
			} else {
				*((u32 *) itr->data) = (u32) itr->def_value;
			}
			break;
		default:
			TOUCH_INFO_MSG("DT : Not support type : %d \n", itr->type);
			break;
		}
	}

	return 0;
}
#endif

static int lge_ts_misc_power(struct lge_ts_misc_info *info, bool on)
{
	int retval = 0;

	if (on == false)
		goto power_off;

	TOUCH_INFO_MSG("Power on \n");

	if(info->vio_lvs1) {
		retval = regulator_enable(info->vio_lvs1);
		if (retval < 0) {
			TOUCH_INFO_MSG("regulator_enable(vio_l6) failed retval = %d\n", retval);
		}
	}

	if(info->vio_l28) {
		retval = regulator_enable(info->vio_l28);
		if (retval < 0) {
			TOUCH_INFO_MSG("regulator_enable(vio_l28) failed retval = %d\n", retval);
		}
	}

	if(info->vdd) {
		retval = regulator_enable(info->vdd);
		if (retval < 0) {
			TOUCH_INFO_MSG("regulator_enable(vdd) failed retval = %d\n", retval);
		}
	}

	goto exit;

power_off :

	TOUCH_INFO_MSG("Power off \n");

	if(info->vdd) {
		regulator_disable(info->vdd);
	}

	if(info->vio_l28) {
		regulator_disable(info->vio_l28);
	}

	if(info->vio_lvs1) {
		regulator_disable(info->vio_lvs1);
	}

exit :
	msleep(10);

	return retval;
}

static int lge_ts_misc_regulator_configure(struct lge_ts_misc_info *info, bool on)
{
	int retval = 0;

	if (on == false)
		goto hw_shutdown;

	if (info->vdd_on && info->vdd == NULL) {
		info->vdd = regulator_get(&info->client->dev, "vdd");
		if (IS_ERR(info->vdd)) {
			TOUCH_INFO_MSG("Failed to get vdd regulator\n");
			return PTR_ERR(info->vdd);
		}
	}

	if (info->vio_l28_on && info->vio_l28 == NULL) {
		info->vio_l28 = regulator_get(&info->client->dev, "vio_l28");
		if (IS_ERR(info->vio_l28)) {
			TOUCH_INFO_MSG("Failed to get vio_l28 regulator\n");
			return PTR_ERR(info->vio_l28);
		}
	}

	if (info->vio_l6_on && info->vio_l6 == NULL) {
		info->vio_l6 = regulator_get(&info->client->dev, "vio_l6");
		if (IS_ERR(info->vio_l6)) {
			TOUCH_INFO_MSG("Failed to get vio_l6 regulator\n");
			return PTR_ERR(info->vio_l6);
		}
	}

	if (info->vio_lvs1_on && info->vio_lvs1 == NULL) {
		info->vio_lvs1 = regulator_get(&info->client->dev, "vio_lvs1");
		if (IS_ERR(info->vio_lvs1)) {
			TOUCH_INFO_MSG("Failed to get vio_l6 regulator\n");
			return PTR_ERR(info->vio_lvs1);
		}
	}

	if(info->vdd && info->vdd_voltage) {
		retval = regulator_set_voltage(info->vdd, info->vdd_voltage, info->vdd_voltage);
		if (retval)
			TOUCH_INFO_MSG("regulator_set_voltage(vdd) failed retval=%d\n", retval);
	}

	if(info->vio_l28 && info->vio_l28_voltage) {
		retval = regulator_set_voltage(info->vio_l28, info->vio_l28_voltage, info->vio_l28_voltage);
		if (retval)
			TOUCH_INFO_MSG("regulator_set_voltage(vio_l28) failed retval=%d\n", retval);
	}

	if(info->vio_l6 && info->vio_l6_voltage) {
		retval = regulator_set_voltage(info->vio_l6, info->vio_l6_voltage, info->vio_l6_voltage);
		if (retval)
			TOUCH_INFO_MSG("regulator_set_voltage(vio_l6) failed retval=%d\n", retval);
	}

	if(info->vio_lvs1 && info->vio_lvs1_voltage) {
		retval = regulator_set_voltage(info->vio_lvs1, info->vio_lvs1_voltage, info->vio_lvs1_voltage);
		if (retval)
			TOUCH_INFO_MSG("regulator_set_voltage(vio_lvs1) failed retval=%d\n", retval);
	}

	return 0;

hw_shutdown :
	if(info->vdd) {
		regulator_put(info->vdd);
	}

	if(info->vio_l28) {
		regulator_put(info->vio_l28);
	}

	if(info->vio_lvs1) {
		regulator_put(info->vio_lvs1);
	}

	return 0;
}

static int lge_ts_misc_enable(struct lge_ts_misc_info *info, bool onoff)
{
	static bool irq_enabled = true;

	if (!info->client->irq) {
		return 0;
	}

	if (onoff && !irq_enabled) {
		irq_enabled = true;
		enable_irq(info->client->irq);
		enable_irq_wake(info->client->irq);
	} else if (!onoff && irq_enabled) {
		irq_enabled = false;
		disable_irq_nosync(info->client->irq);
		disable_irq_wake(info->client->irq);
	}

	return 0;
}

static int lge_ts_misc_suspend(struct lge_ts_misc_info *info)
{
	lge_ts_misc_enable(info, true);

	return 0;
}

static int lge_ts_misc_resume(struct lge_ts_misc_info *info)
{
	lge_ts_misc_enable(info, false);

	return 0;
}

#ifdef CONFIG_FB
static int lge_ts_misc_fb_notifier_call(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *fb;
	struct lge_ts_misc_info *info = container_of(self, struct lge_ts_misc_info, fb_notifier);
	if (evdata && evdata->data && event == FB_EVENT_BLANK && info && info->client) {
		fb = evdata->data;
		switch (*fb) {
			case FB_BLANK_UNBLANK:
				lge_ts_misc_resume(info);
				break;
			case FB_BLANK_POWERDOWN:
				lge_ts_misc_suspend(info);
				break;
			default:
				break;
		}
	}
	return 0;
}
#endif

static irqreturn_t lge_ts_misc_irq(int irq, void *dev_id)
{
	struct lge_ts_misc_info *info = dev_id;

	TOUCH_INFO_MSG("Detected \n");

	lge_ts_misc_enable(info, false);
	input_report_key(info->input_dev, KEY_POWER, PRESS_KEY);
	input_sync(info->input_dev);
	//msleep(20);
	input_report_key(info->input_dev, KEY_POWER, RELEASE_KEY);
	input_sync(info->input_dev);

	return IRQ_HANDLED;
}

static int __devinit lge_ts_misc_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct lge_ts_misc_info *info = NULL;
	struct input_dev *input_dev = NULL;
	int ret = 0;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (info == NULL) {
		TOUCH_INFO_MSG("Failed to allocated memory \n");
		return -ENOMEM;
	}

	input_dev = input_allocate_device();

	if (input_dev == NULL) {
		TOUCH_INFO_MSG("Failed to allocated memory \n");
		return -ENOMEM;
	}

	info->client = client;
	info->input_dev = input_dev;
	i2c_set_clientdata(client, info);

#ifdef CONFIG_OF
	ret = lge_ts_misc_parse_dt(&client->dev, info);
	if (ret) {
		TOUCH_INFO_MSG("Failed to parse device tree \n");
		return -ENODEV;
	}
#endif

	ret = lge_ts_misc_regulator_configure(info, true);
	if (ret) {
		TOUCH_INFO_MSG("Failed to set regulator \n");
		return -ENODEV;
	}

	ret = lge_ts_misc_power(info, true);
	if (ret < 0) {
		TOUCH_INFO_MSG("Failed to regulator on \n");
		return -ENODEV;
	}

	snprintf(info->phys, sizeof(info->phys),
		"%s/input0", dev_name(&client->dev));

	input_dev->name = MISC_DRIVER_NAME;
	input_dev->phys = info->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.init_name = MISC_DRIVER_NAME;

	input_set_drvdata(input_dev, info);

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(KEY_POWER, input_dev->keybit);

	ret = input_register_device(input_dev);
	if (ret) {
		TOUCH_INFO_MSG("failed to register input dev\n");
		return -EIO;
	}

	if (!info->irq_wake) {
		info->client->irq = 0;
	}

	if (info->client->irq) {
		ret = request_threaded_irq(info->client->irq, NULL, lge_ts_misc_irq,
					IRQF_TRIGGER_HIGH | IRQF_ONESHOT, MISC_DRIVER_NAME, info);
		if (ret) {
			TOUCH_INFO_MSG("failed to register irq\n");
			return -EIO;
		}

#ifdef CONFIG_FB
		info->fb_notifier.notifier_call = lge_ts_misc_fb_notifier_call;
		ret = fb_register_client(&info->fb_notifier);
		if (ret) {
			TOUCH_INFO_MSG("failed to register fb_notifier: %d\n", ret);
		}
#endif
	}

	return 0;
}

static int __devexit lge_ts_misc_remove(struct i2c_client *client)
{
	struct lge_ts_misc_info *info = i2c_get_clientdata(client);

	lge_ts_misc_enable(info, false);

	lge_ts_misc_power(info, false);

	lge_ts_misc_regulator_configure(info, false);

	if (info->input_dev) {
		input_unregister_device(info->input_dev);
		input_free_device(info->input_dev);
	}

	if (info)
		kfree(info);

	return 0;
}

static const struct i2c_device_id lge_ts_misc_id[] = {
	{MISC_DRIVER_NAME, 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, lge_ts_misc_id);

#ifdef CONFIG_OF
static struct of_device_id lge_ts_misc_match_table[] = {
	{ .compatible = "lge,ts_misc",},
	{},
};
#endif

static struct i2c_driver lge_ts_misc_driver = {
	.probe		= lge_ts_misc_probe,
	.remove		= __devexit_p(lge_ts_misc_remove),
	.driver		= {
				.name	= MISC_DRIVER_NAME,
#ifdef CONFIG_OF
				.of_match_table = lge_ts_misc_match_table,
#endif
	},
	.id_table	= lge_ts_misc_id,
};

static int __init lge_ts_misc_init(void)
{
	return i2c_add_driver(&lge_ts_misc_driver);
}

static void __exit lge_ts_misc_exit(void)
{
	return i2c_del_driver(&lge_ts_misc_driver);
}

module_init(lge_ts_misc_init);
module_exit(lge_ts_misc_exit);

MODULE_VERSION("1.0");
MODULE_AUTHOR("WX-BSP-TS@lge.com");
MODULE_DESCRIPTION("LGE TS Misc");
MODULE_LICENSE("GPL");

