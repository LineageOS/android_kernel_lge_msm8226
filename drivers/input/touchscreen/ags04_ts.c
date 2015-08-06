/*
 * Touch Sensor driver for AD Semiconductor AGS04
 *
 * Copyright (C) 2013 LG Electronics Inc.
 * Author: LG Electronics <xxx@lge.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif
#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#define MAX_CHANNEL 4

#define REG_OUTPUT		0x00
#define REG_CH_ENABLE		0x01
#define REG_GLOBAL_CTRL1	0x02
#define REG_INTERRUPT_MODE	0x03
#define REG_SENSITIVITY1	0x04
#define REG_SENSITIVITY2	0x05
#define REG_SEN_LIMIT		0x08
#define REG_CAL_SPEED		0x0C
#define REG_RND_CAL_SPEED	0x0D
#define REG_OUT_EXPIRATION	0x0E
#define REG_PERIOD_CONTROL	0x16

struct ags04_ts_platform_data {
	int gpio_int;
	int key_code[MAX_CHANNEL];
	int num_of_on_cmds;
	int *on_cmds;
};

struct ags04_ts_info {
	struct i2c_client 		*client;
	struct input_dev 		*input_dev;

	char 				phys[32];

	bool				initialized;
	bool				enabled;
	int				output;

#ifdef CONFIG_FB
	struct notifier_block		fb_notifier;
#endif
	struct mutex 			lock;

	struct ags04_ts_platform_data 	*pdata;

	struct regulator *vdd;
	struct regulator *vcc_i2c;
};


static int ags04_ts_i2c_read(struct i2c_client *client, int reg)
{
	int rc;

	rc = i2c_smbus_read_byte_data(client, reg);
	if (rc < 0) {
		dev_err(&client->dev, "i2c error %d while reading register %d\n", rc, reg);
	}

	return rc;
}

static ssize_t ags04_test_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int a=0, b=0, c=0, d=0, e=0, f=0, g=0, h=0;

	struct i2c_client *client = to_i2c_client(dev);

	a = ags04_ts_i2c_read(client, 0x02);
	if (a < 0)
		printk("[TK] 0x02 i2c read fail a\n");

	b = ags04_ts_i2c_read(client, 0x0c);
	if (b < 0)
		printk("[TK] 0x0c i2c read fail a\n");

	c = ags04_ts_i2c_read(client, 0x0d);
	if (c < 0)
		printk("[TK] 0x0d i2c read fail a\n");

	d = ags04_ts_i2c_read(client, 0x0e);
	if (d < 0)
		printk("[TK] 0x0e i2c read fail a\n");

	e = ags04_ts_i2c_read(client, 0x16);
	if (e < 0)
		printk("[TK] 0x16 i2c read fail a\n");

	f = ags04_ts_i2c_read(client, 0x01);
	if (f < 0)
		printk("[TK] 0x01 i2c read fail a\n");

	g = ags04_ts_i2c_read(client, 0x04);
	if (g < 0)
		printk("[TK] 0x04 i2c read fail a\n");

	h = ags04_ts_i2c_read(client, 0x05);
	if (h < 0)
		printk("[TK] 0x05 i2c read fail a\n");

	return sprintf(buf, "0x02[%x] 0x0c[%x] 0x0d[%x] 0x0e[%x] 0x16[%x] 0x01[%x] 0x04[%x] 0x05[%x]\n", a, b, c, d, e, f, g, h);
}

static struct device_attribute dev_attr_device_test[] = {
	__ATTR(test, S_IRUGO | S_IWUSR | S_IXOTH, ags04_test_show, NULL),
};

static int ags04_ts_i2c_write(struct i2c_client *client, int reg, int data)
{
	int rc;

	rc = i2c_smbus_write_byte_data(client, reg, data);
	if (rc) {
		dev_err(&client->dev, "i2c error %d while writing %d to register %d\n", rc, data, reg);
	}

	return rc;
}

static int ags04_ts_power_on(struct ags04_ts_info *info, int on)
{
	static bool initialized = false;
	int rc = 0;
	int retval = 0;

	struct i2c_client *client = info->client;
	dev_info(&client->dev, "ags04_ts_power_on! initialized:%d\n", initialized);
	/* TODO : Request power source */
	if (!initialized) {
		if(info->vdd == NULL) {
				info->vdd = regulator_get(&info->client->dev, "vdd");
				if (IS_ERR(info->vdd)) {
					dev_err(&info->client->dev, "[TOUCH] %s: Failed to get vdd regulator\n", __func__);
					return PTR_ERR(info->vdd);
			}
			dev_err(&info->client->dev, "[TOUCH] regulator_get(VDD) \n");
		}
	#if 0
		if(info->vcc_i2c == NULL) {
			info->vcc_i2c = regulator_get(&info->client->dev, "vcc_i2c");
			if (IS_ERR(info->vcc_i2c)) {
				dev_err(&info->client->dev, "[TOUCH] %s: Failed to get i2c regulator\n", __func__);
				return PTR_ERR(info->vcc_i2c);
			}
			dev_err(&info->client->dev, "[TOUCH] regulator_get(I2C) \n");
		}
	#endif

		if(info->vdd) {
			retval = regulator_set_voltage(info->vdd, 1800000, 1800000);
			if (retval)
				dev_err(&info->client->dev, "[TOUCH] regulator_set_voltage(VDD) failed retval=%d\n", retval);
			else
				dev_err(&info->client->dev, "[TOUCH] regulator_set_voltage(VDD) \n");
		}
	#if 0
		if(info->vcc_i2c) {
			retval = regulator_set_voltage(info->vcc_i2c, 1800000, 1800000);
			if (retval)
				dev_err(&info->client->dev, "[TOUCH] regulator_set_voltage(I2C) failed retval=%d\n", retval);
			else
				dev_err(&info->client->dev, "[TOUCH] regulator_set_voltage(I2C) \n");
		}
	#endif
	}

	/* TODO : Turn on/off power */
	if (initialized == false)
	{
		if (on) {
			if(info->vdd) {
					retval = regulator_enable(info->vdd);
					dev_err(&info->client->dev, "[TOUCH] regulator_enable(VDD) \n");
					if (retval < 0) {
						dev_err(&info->client->dev, "[TOUCH] Regulator vdd enable failed retval = %d\n", retval);
				}
			}
	#if 0
			if(info->vcc_i2c) {
				retval = regulator_enable(info->vcc_i2c);
				dev_err(&info->client->dev, "[TOUCH] regulator_enable(I2C) \n");
				if (retval < 0) {
					dev_err(&info->client->dev, "[TOUCH] Regulator i2c enable failed retval = %d\n", retval);
				}
			}
	#endif
		} else {
	#if 0
			if(info->vdd) {
				regulator_disable(info->vdd);
				dev_err(&info->client->dev, "[TOUCH] regulator_disable(VDD) \n");
			}

			if(info->vcc_i2c) {
				regulator_disable(info->vcc_i2c);
				dev_err(&info->client->dev, "[TOUCH] regulator_disable(I2C) \n");
			}
	#endif
		}
	}
	initialized = true;

	return rc;
}

static int ags04_ts_config(struct ags04_ts_info *info)
{
	struct i2c_client *client = info->client;
	int rc = 0;
	int i;

	if (!info->pdata->num_of_on_cmds)
		return 0;
	if (!info->pdata->on_cmds)
		return 0;

	for(i = 0 ; i < info->pdata->num_of_on_cmds; i++) {
		rc = ags04_ts_i2c_write(client,
			info->pdata->on_cmds[(i * 2)],
			info->pdata->on_cmds[(i * 2) + 1]);
	}

	return rc;
}

static void ags04_ts_enable(struct ags04_ts_info *info)
{
	struct i2c_client *client = info->client;

	if (info->enabled)
		return;

	mutex_lock(&info->lock);

	//ags04_ts_power_on(info, 1);
	//msleep(100);

	info->enabled = true;
	enable_irq(client->irq);

	mutex_unlock(&info->lock);
}

static void ags04_ts_disable(struct ags04_ts_info *info)
{
	struct i2c_client *client = info->client;

	if (!info->enabled)
		return;

	mutex_lock(&info->lock);

	disable_irq(client->irq);
	info->enabled = false;

	//ags04_ts_power_on(info, 0);

	mutex_unlock(&info->lock);
}

static irqreturn_t ags04_ts_interrupt(int irq, void *dev_id)
{
	struct ags04_ts_info *info = dev_id;
	struct i2c_client *client = info->client;

	int channel;
	int changed;
	int output;
	int pressed;

	dev_info(&client->dev, "interrupt!\n");
	output = ags04_ts_i2c_read(client, REG_OUTPUT);
	if (output < 0)
		return IRQ_HANDLED;

	changed = info->output ^ output;

	for (channel = 0; channel < MAX_CHANNEL; channel++) {
		if (changed & (1 << channel)) {
			changed = changed ^ (1 << channel);
			pressed = output >> channel & 0x01;

			if (info->pdata->key_code[channel]) {
				input_report_key(info->input_dev, info->pdata->key_code[channel], pressed);
			} else {
				dev_err(&client->dev, "key code not defined at channel %d\n", channel);
			}
		}
	}
	input_sync(info->input_dev);

	info->output = output;

	return IRQ_HANDLED;
}

static int ags04_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ags04_ts_info *info = i2c_get_clientdata(client);

	mutex_lock(&info->input_dev->mutex);

	if (info->input_dev->users) {
		dev_info(&client->dev, "ags04_ts_suspend!\n");
		ags04_ts_disable(info);
	}

	mutex_unlock(&info->input_dev->mutex);
	return 0;
}

static int ags04_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ags04_ts_info *info = i2c_get_clientdata(client);

	mutex_lock(&info->input_dev->mutex);

	if (info->input_dev->users) {
		dev_info(&client->dev, "ags04_ts_resume!\n");
		ags04_ts_enable(info);
	}

	mutex_unlock(&info->input_dev->mutex);

	return 0;
}

static int ags04_ts_input_open(struct input_dev *dev)
{
	struct ags04_ts_info *info = input_get_drvdata(dev);
	struct i2c_client *client = info->client;
	dev_info(&client->dev, "ags04_ts_input_open!\n");
	//ags04_ts_enable(info);
	info = info;
	return 0;
}

static void ags04_ts_input_close(struct input_dev *dev)
{
	struct ags04_ts_info *info = input_get_drvdata(dev);
	struct i2c_client *client = info->client;
	dev_info(&client->dev, "ags04_ts_input_close!\n");
	//ags04_ts_disable(info);
	info = info;
}


#ifdef CONFIG_FB
static int ags04_ts_fb_notifier_call(struct notifier_block *self,
				   unsigned long event,
				   void *data)
{
	struct ags04_ts_info *info = container_of(self, struct ags04_ts_info, fb_notifier);
	struct fb_event *evdata = data;
	int *fb;
	if(evdata && evdata->data && event == FB_EVENT_BLANK && info && info->client) {
		fb = evdata->data;
		switch (*fb) {
			case FB_BLANK_UNBLANK:
				ags04_ts_resume(&info->client->dev);
				break;
			case FB_BLANK_POWERDOWN:
				ags04_ts_suspend(&info->client->dev);
				break;
			default:
				break;
		}
	}
	return 0;
}
#endif

#ifdef CONFIG_OF
static int ags04_ts_parse_dt(struct device *dev, struct ags04_ts_platform_data *data)
{
	struct device_node *np = dev->of_node;
	int rc;

	rc = of_property_read_u32(np, "ads,gpio-int", &data->gpio_int);
	if (rc) {
		dev_err(dev, "fail to read gpio-int (%d)\n", rc);
	}

	rc = of_property_read_u32_array(np, "ads,key_code", data->key_code, MAX_CHANNEL);
	if (rc) {
		dev_err(dev, "fail to read key_code (%d)\n", rc);
	}

	rc = of_property_read_u32(np, "ads,num-of-on-cmds", &data->num_of_on_cmds);
	if (rc) {
		dev_err(dev, "fail to read num-of-on-cmds (%d)\n", rc);
		data->num_of_on_cmds = 0;
		return 0;
	}

	data->on_cmds = kzalloc(sizeof(int) * data->num_of_on_cmds * 2, GFP_KERNEL);
	if (!data->on_cmds) {
		dev_err(dev, "fail to read alloc memory for on-cmds\n");
		return -ENOMEM;
	}

	rc = of_property_read_u32_array(np, "ads,on-cmds", data->on_cmds, data->num_of_on_cmds * 2);
	if (rc) {
		dev_err(dev, "fail to read on-cmds (%d)\n", rc);
		kfree(data->on_cmds);
		data->on_cmds = NULL;
	}

	return 0;
}
#endif

static int __devinit ags04_ts_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct ags04_ts_info *info;
	struct input_dev *input_dev;
	int rc = 0;
	int i = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -EIO;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	input_dev = input_allocate_device();

	if (!info || !input_dev) {
		dev_err(&client->dev, "Failed to allocated memory\n");
		return -ENOMEM;
	}

	info->client = client;
	info->input_dev = input_dev;
	info->output = 0;
#ifdef CONFIG_OF
	info->pdata = kzalloc(sizeof(*info->pdata), GFP_KERNEL);
	if (!info->pdata) {
		dev_err(&client->dev, "Failed to allocated memory for device tree\n");
		return -ENOMEM;
	}
	rc = ags04_ts_parse_dt(&client->dev, info->pdata);
	if (rc) {
		dev_err(&client->dev, "Failed to parse device tree\n");
		return -ENODEV;
	}
#else
	info->pdata = client->dev.platform_data;
#endif

	mutex_init(&info->lock);

	snprintf(info->phys, sizeof(info->phys),
		"%s/input0", dev_name(&client->dev));

	input_dev->name = "ags04_ts";
	input_dev->phys = info->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
	input_dev->open = ags04_ts_input_open;
	input_dev->close = ags04_ts_input_close;

	input_set_drvdata(input_dev, info);

	if (info->pdata->key_code[0]) {
		int i = 0;
		__set_bit(EV_KEY, input_dev->evbit);
		while(info->pdata->key_code[i]) {
			__set_bit(info->pdata->key_code[i], input_dev->keybit);
			i++;
		}
	}

	rc = input_register_device(input_dev);
	if (rc) {
		dev_err(&client->dev, "failed to register input dev %d\n", rc);
		return -EIO;
	}

	i2c_set_clientdata(client, info);

#ifdef CONFIG_FB
	info->fb_notifier.notifier_call = ags04_ts_fb_notifier_call;
	rc = fb_register_client(&info->fb_notifier);
	if (rc) {
		dev_err(&client->dev, "failed to register fb_notifier %d\n", rc);
	}
#endif

	for (i = 0; i < ARRAY_SIZE(dev_attr_device_test); i++) {
		rc = device_create_file(&client->dev, &dev_attr_device_test[i]);
		if (rc) {
			dev_err(&client->dev,"device_create_file fail %d\n", rc);
		}
	}
	ags04_ts_power_on(info, 1);
	msleep(200);

	rc = ags04_ts_config(info);
	if (rc) {
		dev_err(&client->dev, "failed to initialize IC %d\n", rc);
	}

	if (gpio_is_valid(info->pdata->gpio_int)) {
		gpio_request(info->pdata->gpio_int, "ags04_ts_int");
		gpio_direction_input(info->pdata->gpio_int);
	}

	rc = request_threaded_irq(client->irq, NULL, ags04_ts_interrupt,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				"ags04_ts", info);
	if (rc) {
		dev_err(&client->dev, "failed to register irq %d\n", rc);
		return -ENODEV;
	}
	disable_irq(client->irq);

	return 0;
}

static int __devexit ags04_ts_remove(struct i2c_client *client)
{
	struct ags04_ts_info *info = i2c_get_clientdata(client);

	info = info;

	return 0;
}

static const struct i2c_device_id ags04_ts_id[] = {
	{"ags04_ts", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, ags04_ts_id);

#ifdef CONFIG_OF
static struct of_device_id ags04_ts_match_table[] = {
	{ .compatible = "ads,ags04_ts",},
	{},
};
#endif

static struct i2c_driver ags04_ts_driver = {
	.probe		= ags04_ts_probe,
	.remove		= __devexit_p(ags04_ts_remove),
	.driver		= {
		.name	= "ags04_ts",
#ifdef CONFIG_OF
		.of_match_table = ags04_ts_match_table,
#endif
	},
	.id_table	= ags04_ts_id,
};

static int __init ags04_ts_init(void)
{
	return i2c_add_driver(&ags04_ts_driver);
}

static void __exit ags04_ts_exit(void)
{
	return i2c_del_driver(&ags04_ts_driver);
}

module_init(ags04_ts_init);
module_exit(ags04_ts_exit);
