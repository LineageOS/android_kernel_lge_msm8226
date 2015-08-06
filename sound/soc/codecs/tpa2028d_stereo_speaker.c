/* sound/soc/codecs/tpa2028d.c
 *
 * Copyright (C) 2009 LGE, Inc.
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

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <asm/system.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <asm/ioctls.h>
#include <mach/debug_mm.h>
#include <linux/slab.h>

#include <sound/tpa2028d.h>

#define MODULE_NAME "tpa2028d"

#undef  AMP_DEBUG_PRINT
#define AMP_DEBUG_PRINT

#define AMP_IOCTL_MAGIC 't'
#define AMP_SET_DATA	_IOW(AMP_IOCTL_MAGIC, 0, struct amp_cal *)
#define AMP_GET_DATA	_IOW(AMP_IOCTL_MAGIC, 1, struct amp_cal *)

static uint32_t msm_snd_debug = 1;
module_param_named(debug_mask, msm_snd_debug, uint, 0664);

#if defined (AMP_DEBUG_PRINT)
#define D(fmt, args...) printk(KERN_INFO "[%s:%5d]" fmt, __func__, __LINE__, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif

struct amp_config {
	struct i2c_client *client;
	struct audio_amp_platform_data *pdata;
	int state;
};

static struct amp_config *amp_data[3];


int ReadI2C(int amp_no, char reg, char *ret)
{

	unsigned int err;
	unsigned char buf = reg;

	struct i2c_msg msg[2] = {
		{ amp_data[amp_no]->client->addr, 0, 1, &buf },
		{ amp_data[amp_no]->client->addr, I2C_M_RD, 1, ret}
	};
	err = i2c_transfer(amp_data[amp_no]->client->adapter, msg, 2);
	if (err < 0)
		D("i2c read error:%x amp_no:%d\n", reg, amp_no);
	else
		D("i2c read ok:%x amp_no:%d\n", reg, amp_no);
	return 0;

}

int WriteI2C(int amp_no, char reg, char val)
{

	int err;
	unsigned char buf[2];
	struct i2c_msg msg = {amp_data[amp_no]->client->addr, 0, 2, buf };

	buf[0] = reg;
	buf[1] = val;
	err = i2c_transfer(amp_data[amp_no]->client->adapter, &msg, 1);

	if (err < 0) {
		D("i2c write error:%x:%x  amp_no:%d\n", reg, val, amp_no);
		return -EIO;
	} else {
		return 0;
	}
}

int tpa2028d_poweron(int amp_no)
{
	int fail = 0;
	char agc_compression_rate = amp_data[amp_no]->pdata->agc_compression_rate;
	char agc_output_limiter_disable = amp_data[amp_no]->pdata->agc_output_limiter_disable;
	char agc_fixed_gain = amp_data[amp_no]->pdata->agc_fixed_gain;

	agc_output_limiter_disable = (agc_output_limiter_disable<<7);

	fail |= WriteI2C(amp_no, IC_CONTROL, 0xE3); /*Tuen On*/
	fail |= WriteI2C(amp_no, AGC_ATTACK_CONTROL, 0x05); /*Tuen On*/
	fail |= WriteI2C(amp_no, AGC_RELEASE_CONTROL, 0x0B); /*Tuen On*/
	fail |= WriteI2C(amp_no, AGC_HOLD_TIME_CONTROL, 0x00); /*Tuen On*/
	fail |= WriteI2C(amp_no, AGC_FIXED_GAIN_CONTROL, agc_fixed_gain); /*Tuen On*/
	fail |= WriteI2C(amp_no, AGC1_CONTROL, 0x3A|agc_output_limiter_disable); /*Tuen On*/
	fail |= WriteI2C(amp_no, AGC2_CONTROL, 0xC0|agc_compression_rate); /*Tuen On*/
	fail |= WriteI2C(amp_no, IC_CONTROL, 0xC3); /*Tuen On*/

	return fail;
}

int tpa2028d_powerdown(void)
{
	int fail = 0;
	return fail;
}

inline void set_amp_gain(int amp_no, int amp_state)
{
	int fail = 0;

	D("set_amp_gain : amp_no[%d] amp_state[%d]\n", amp_no, amp_state);
	
	switch (amp_state) {
	case SPK_ON:
		//If  sysfs status is zero then speaker on request is ignored ( only from AAT )
		if (amp_data[amp_no]->state == 0 )
			break;

		/* if the delay time is need for chip ready,
		 * should be added to each power or enable function.
		 */
		if (amp_data[amp_no]->pdata->power)
			fail = amp_data[amp_no]->pdata->power(1);
		/*need 5 msec for prevent mute issue*/
		//msleep(5);
		mdelay(5);
		if (amp_data[amp_no]->pdata->enable)
#ifdef CONFIG_SND_SOC_TPA2028D_STEREO_E9
			fail = amp_data[amp_no]->pdata->enable(1, amp_data[amp_no]->pdata->enable_gpio, amp_no);
#else
			fail = amp_data[amp_no]->pdata->enable(1);
#endif //CONFIG_SND_SOC_TPA2028D_STEREO_E9
		/*need 10 msec for chip ready*/
		//msleep(10);
		mdelay(10);
		fail = tpa2028d_poweron(amp_no);
		//amp_data[amp_no]->state = SPK_ON;
		break;
	case SPK_OFF:
		if (amp_data[amp_no]->pdata->enable)
#ifdef CONFIG_SND_SOC_TPA2028D_STEREO_E9
			//fail = amp_data[amp_no]->pdata->enable(2, amp_data[amp_no]->pdata->enable_gpio, amp_no);
			//delete bypass
#else
			fail = amp_data[amp_no]->pdata->enable(2);
#endif //CONFIG_SND_SOC_TPA2028D_STEREO_E9
		fail = tpa2028d_powerdown();
		if (amp_data[amp_no]->pdata->enable)
#ifdef CONFIG_SND_SOC_TPA2028D_STEREO_E9
			fail = amp_data[amp_no]->pdata->enable(0, amp_data[amp_no]->pdata->enable_gpio, amp_no);
#else
			fail = amp_data[amp_no]->pdata->enable(0);
#endif //CONFIG_SND_SOC_TPA2028D_STEREO_E9
		if (amp_data[amp_no]->pdata->power)
			fail = amp_data[amp_no]->pdata->power(0);
		//amp_data[amp_no]->state = SPK_OFF;
		break;
	default:
		D("Amp_state [%d] does not support \n", amp_state);
	}
}
EXPORT_SYMBOL(set_amp_gain);

inline void irrc_amp_off(int amp_no, int amp_state)
{
	int fail = 0;

	D("irrc_amp_off : amp_no[%d] amp_state[%d]\n", amp_no, amp_state);

	if (amp_data[amp_no]->pdata->enable)
#ifdef CONFIG_SND_SOC_TPA2028D_STEREO_E9
		fail = amp_data[amp_no]->pdata->enable(0, amp_data[amp_no]->pdata->enable_gpio, amp_no);
#else
		fail = amp_data[amp_no]->pdata->enable(0);
#endif //CONFIG_SND_SOC_TPA2028D_STEREO_E9
	if (amp_data[amp_no]->pdata->power)
		fail = amp_data[amp_no]->pdata->power(0);
}
EXPORT_SYMBOL(irrc_amp_off);


static ssize_t
tpa2028d_comp_rate_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct audio_amp_platform_data *pdata = amp_data[0]->pdata;
	int val;

	D("[tpa2028d_comp_rate_store] buf(%s) pamp_no(0)\n", buf);
	if (sscanf(buf, "%d", &val) != 1)
		return -EINVAL;
	if (val > 3 || val < 0)
		return -EINVAL;

	pdata->agc_compression_rate = val;
	return count;
}


static ssize_t
tpa2028d_comp_rate_show(struct device *dev, struct device_attribute *attr,   char *buf)
{
	struct audio_amp_platform_data *pdata = amp_data[0]->pdata;
	char val = 0;

	ReadI2C(0, AGC2_CONTROL, &val);

	D("[tpa2028d_comp_rate_show] val :(%x) pamp_no(0)\n",val);

	return sprintf(buf, "AGC2_CONTROL : %x, pdata->agc_compression_rate : %d\n", val, pdata->agc_compression_rate);
}



static ssize_t
tpa2028d_out_lim_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct audio_amp_platform_data *pdata = amp_data[0]->pdata;
	int val;

	D("[tpa2028d_out_lim_store] buf(%s) pamp_no(0)\n", buf);
	if (sscanf(buf, "%d", &val) != 1)
		return -EINVAL;
	pdata->agc_output_limiter_disable = val&0x0001;
	return count;
}



static ssize_t
tpa2028d_out_lim_show(struct device *dev, struct device_attribute *attr,   char *buf)
{
	struct audio_amp_platform_data *pdata = amp_data[0]->pdata;
	char val=0;

	ReadI2C(0, AGC1_CONTROL, &val);

	D("[tpa2028d_out_lim_show] val :(%x) pamp_no(0) \n",val);

	return sprintf(buf, "AGC1_CONTROL : %x, pdata->agc_output_limiter_disable : %d\n", val, pdata->agc_output_limiter_disable);
}



static ssize_t
tpa2028d_fixed_gain_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct audio_amp_platform_data *pdata = amp_data[0]->pdata;
	int val;

	D("[tpa2028d_fixed_gain_store] buf(%s) pamp_no(0)\n", buf);
	if (sscanf(buf, "%d", &val) != 1)
		return -EINVAL;

	pdata->agc_fixed_gain = val;
	WriteI2C(0, AGC_FIXED_GAIN_CONTROL, val);
	return count;
}




static ssize_t
tpa2028d_fixed_gain_show(struct device *dev, struct device_attribute *attr,   char *buf)
{
	struct audio_amp_platform_data *pdata = amp_data[0]->pdata;
	char val=0;

	ReadI2C(0, AGC_FIXED_GAIN_CONTROL, &val);

	D("[tpa2028d_fixed_gain_show] val :(%x) pamp_no(0) \n",val);

	return sprintf(buf, "fixed_gain : %x, pdata->agc_fixed_gain : %d\n", val, pdata->agc_fixed_gain);
}


static ssize_t
tpa2028d2_comp_rate_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct audio_amp_platform_data *pdata = amp_data[1]->pdata;
	int val;

	if (sscanf(buf, "%d", &val) != 1)
		return -EINVAL;
	if (val > 3 || val < 0)
		return -EINVAL;

	pdata->agc_compression_rate = val;
	return count;
}

static ssize_t
tpa2028d2_comp_rate_show(struct device *dev, struct device_attribute *attr,   char *buf)
{
	struct audio_amp_platform_data *pdata = amp_data[1]->pdata;
	char val = 0;

	ReadI2C(1, AGC2_CONTROL, &val);

	D("[tpa2028d2_comp_rate_show] val : %x\n",val);

	return sprintf(buf, "AGC2_CONTROL : %x, pdata->agc_compression_rate : %d\n", val, pdata->agc_compression_rate);
}

static ssize_t
tpa2028d2_out_lim_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct audio_amp_platform_data *pdata = amp_data[1]->pdata;
	int val;

	if (sscanf(buf, "%d", &val) != 1)
		return -EINVAL;
	pdata->agc_output_limiter_disable = val&0x0001;
	return count;
}

static ssize_t
tpa2028d2_out_lim_show(struct device *dev, struct device_attribute *attr,   char *buf)
{
	struct audio_amp_platform_data *pdata = amp_data[1]->pdata;
	char val=0;

	ReadI2C(1, AGC1_CONTROL, &val);

	D("[tpa2028d2_out_lim_show] val : %x \n",val);

	return sprintf(buf, "AGC1_CONTROL : %x, pdata->agc_output_limiter_disable : %d\n", val, pdata->agc_output_limiter_disable);
}


static ssize_t
tpa2028d2_fixed_gain_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct audio_amp_platform_data *pdata = amp_data[1]->pdata;
	int val;

	if (sscanf(buf, "%d", &val) != 1)
		return -EINVAL;

	pdata->agc_fixed_gain = val;
	WriteI2C(1, AGC_FIXED_GAIN_CONTROL, val);
	return count;
}

static ssize_t
tpa2028d2_fixed_gain_show(struct device *dev, struct device_attribute *attr,   char *buf)
{
	struct audio_amp_platform_data *pdata = amp_data[1]->pdata;
	char val=0;

	ReadI2C(1, AGC_FIXED_GAIN_CONTROL, &val);

	D("[tpa2028d2_fixed_gain_show] val : %x \n",val);

	return sprintf(buf, "fixed_gain : %x, pdata->agc_fixed_gain : %d\n", val, pdata->agc_fixed_gain);
}

static ssize_t
tpa2028d_power_on_show(struct device *dev, struct device_attribute *attr,   char *buf)
{
	int val = amp_data[0]->state;
	return sprintf(buf, "%d\n", val);
}

static ssize_t
tpa2028d_power_on_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int val;

	if (sscanf(buf, "%d", &val) != 1)
		return -EINVAL;
	amp_data[0]->state = val;

	if ( val ==0 )
		set_amp_gain(0, SPK_OFF);
	else if (val == 1 )
		set_amp_gain(0, SPK_ON);
	else
		return -EINVAL;

	return count;
}

static ssize_t
tpa2028d2_power_on_show(struct device *dev, struct device_attribute *attr,   char *buf)
{
	int val = amp_data[1]->state;
	return sprintf(buf, "%d\n", val);
}

static ssize_t
tpa2028d2_power_on_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int val;

	if (sscanf(buf, "%d", &val) != 1)
		return -EINVAL;
	amp_data[1]->state = val;

	if ( val ==0 )
		set_amp_gain(1, SPK_OFF);
	else if (val == 1 )
		set_amp_gain(1, SPK_ON);
	else
		return -EINVAL;

	return count;
}


static struct device_attribute tpa2028d_device_attrs[] = {
	__ATTR(comp_rate,  S_IRUGO | S_IWUSR, tpa2028d_comp_rate_show, tpa2028d_comp_rate_store),
	__ATTR(out_lim, S_IRUGO | S_IWUSR, tpa2028d_out_lim_show, tpa2028d_out_lim_store),
	__ATTR(fixed_gain, S_IRUGO | S_IWUSR, tpa2028d_fixed_gain_show, tpa2028d_fixed_gain_store),
	__ATTR(power_on, 0640, tpa2028d_power_on_show, tpa2028d_power_on_store),

};

static struct device_attribute tpa2028d2_device_attrs[] = {
	__ATTR(comp_rate,  S_IRUGO | S_IWUSR, tpa2028d2_comp_rate_show, tpa2028d2_comp_rate_store),
	__ATTR(out_lim, S_IRUGO | S_IWUSR, tpa2028d2_out_lim_show, tpa2028d2_out_lim_store),
	__ATTR(fixed_gain, S_IRUGO | S_IWUSR, tpa2028d2_fixed_gain_show, tpa2028d2_fixed_gain_store),
	__ATTR(power_on, 0640, tpa2028d2_power_on_show, tpa2028d2_power_on_store),

};

#ifdef CONFIG_SND_SOC_TPA2028D_STEREO_E9
int amp_enable(int on_state, int enable_gpio, int amp_no)
{
	int err = 0;

	switch (on_state) {
		case 0:
			err = gpio_direction_output(enable_gpio, 0);
			printk(KERN_INFO "%s: AMP_%d is set to 0 err(%d)\n", __func__, amp_no, err);
			break;
		case 1:
			err = gpio_direction_output(enable_gpio, 1);
			printk(KERN_INFO "%s: AMP_%d is set to 1 err(%d)\n", __func__, amp_no, err);
			break;
		case 2:
			printk(KERN_INFO "%s: AMP_%d enable bypass(%d)\n", __func__, amp_no, on_state);
			err = 0;
			break;

		default:
			pr_err("AMP_%d enable fail\n", amp_no);
			err = 1;
			break;
	}
	return err;
}

static int tpa2028d_parse_dt(struct device *dev,
		struct audio_amp_platform_data *pdata)
{
	int ret =0;
	u32 value;
	struct device_node *np = dev->of_node;

	ret = of_property_read_u32(np, "tpa2028d,agc_compression_rate",
			&value);
	pdata->agc_compression_rate = (char)value;
	ret = of_property_read_u32(np, "tpa2028d,agc_output_limiter_disable",
			&value);
	pdata->agc_output_limiter_disable = (char)value;
	ret = of_property_read_u32(np, "tpa2028d,agc_fixed_gain",
			&value);
	pdata->agc_fixed_gain = (char)value;
	pdata->enable_gpio = of_get_named_gpio_flags(np,
			"tpa2028d,enable_gpio", 0, NULL);
	pdata->enable = amp_enable;
	return ret;
}
#endif //CONFIG_SND_SOC_TPA2028D_STEREO_E9

static int tpa2028d_amp_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct amp_config *data;
	struct audio_amp_platform_data *pdata;
	struct i2c_adapter *adapter = client->adapter;
	int err;
	int	 i;
	int	amp_no=0;

//	D("[tpa2028d_amp_probe ] id->name(%s) amp_no(%d) \n", id->name, pamp_no);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
		err = -EOPNOTSUPP;
		return err;
	}

#ifdef CONFIG_SND_SOC_TPA2028D_STEREO_E9
	if (&client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct audio_amp_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			pr_err("%s: Failed to allocate memory\n", __func__);
			return -ENOMEM;
		}
		err = tpa2028d_parse_dt(&client->dev, pdata);
		if (err != 0)
			return err;
	} else {
		pdata = client->dev.platform_data;
	}
#else
	pdata = client->dev.platform_data;
#endif //CONFIG_SND_SOC_TPA2028D_STEREO_E9
	if (pdata == NULL) {
		D("platform data is null\n");
		return -ENODEV;
	}

	data = kzalloc(sizeof(struct amp_config), GFP_KERNEL);
	if (data == NULL)
		return -ENOMEM;

	if ( !strncmp(client->name,"tpa2028d1", 9) ) {
		amp_no=0;
		for (i = 0; i < ARRAY_SIZE(tpa2028d_device_attrs); i++) {
			err = device_create_file(&client->dev, &tpa2028d_device_attrs[i]);
			if (err)
				return err;
		}
	}
	else if ( !strncmp(client->name,"tpa2028d2", 9) ) {
		amp_no=1;
		for (i = 0; i < ARRAY_SIZE(tpa2028d2_device_attrs); i++) {
			err = device_create_file(&client->dev, &tpa2028d2_device_attrs[i]);
			if (err)
				return err;
		}
	}

	D("[tpa2028d_amp_probe ] client->name(%s) amp_no(%d) \n", client->name, amp_no);
	amp_data[amp_no] = data;
	amp_data[amp_no]->pdata = pdata;
	amp_data[amp_no]->state = -1;	// Speaker initial status value   0:Speaker Off     1: Speaker On
	data->client = client;
	
	i2c_set_clientdata(client, data);

#ifdef CONFIG_SND_SOC_TPA2028D_STEREO_E9
	err = gpio_request(amp_data[amp_no]->pdata->enable_gpio, "speaker amp enable");
	if (err) {
		pr_err("%s : Error requesting GPIO %d err(%d)\n",
				__func__, amp_data[amp_no]->pdata->enable_gpio, err);
		return err;
	}
#endif //CONFIG_SND_SOC_TPA2028D_STEREO_E9
	set_amp_gain(amp_no, SPK_OFF);

	return 0;
}

static int tpa2028d_amp_remove(struct i2c_client *client)
{
	struct amp_config *data = i2c_get_clientdata(client);
	kfree(data);

	i2c_set_clientdata(client, NULL);
	return 0;
}


#ifdef CONFIG_SND_SOC_TPA2028D_STEREO_E9
static struct of_device_id tpa2028d_amp_match_table[] = {
	{ .compatible = "speaker_amp,tpa2028d1",},
	{ },
};

static struct of_device_id tpa2028d_amp2_match_table[] = {
	{ .compatible = "speaker_amp,tpa2028d2",},
	{ },
};
#endif //CONFIG_SND_SOC_TPA2028D_STEREO_E9
static struct i2c_device_id tpa2028d_amp_idtable[] = {
	{ "tpa2028d_amp", 1 },
};

static struct i2c_device_id tpa2028d_amp2_idtable[] = {
	{ "tpa2028d2_amp", 2 },
};

static struct i2c_driver tpa2028d_amp_driver = {
	.probe = tpa2028d_amp_probe,
	.remove = tpa2028d_amp_remove,
	.id_table = tpa2028d_amp_idtable,
	.driver = {
		.name = "tpa2028d",
#ifdef CONFIG_SND_SOC_TPA2028D_STEREO_E9
		.of_match_table = tpa2028d_amp_match_table,
#endif //CONFIG_SND_SOC_TPA2028D_STEREO_E9
	},
};

static struct i2c_driver tpa2028d_amp2_driver = {
	.probe = tpa2028d_amp_probe,
	.remove = tpa2028d_amp_remove,
	.id_table = tpa2028d_amp2_idtable,
	.driver = {
		.name = "tpa2028d2",
#ifdef CONFIG_SND_SOC_TPA2028D_STEREO_E9
		.of_match_table = tpa2028d_amp2_match_table,
#endif //CONFIG_SND_SOC_TPA2028D_STEREO_E9
	},
};

static int __init tpa2028d_stereo_amp_init(void)
{
	return i2c_add_driver(&tpa2028d_amp_driver);
}


static void __exit tpa2028d_stereo_amp_exit(void)
{
	return i2c_del_driver(&tpa2028d_amp_driver);
}


static int __init tpa2028d2_stereo_amp_init(void)
{
	return i2c_add_driver(&tpa2028d_amp2_driver);
}


static void __exit tpa2028d2_stereo_amp_exit(void)
{
	return i2c_del_driver(&tpa2028d_amp2_driver);
}

module_init(tpa2028d_stereo_amp_init);
module_exit(tpa2028d_stereo_amp_exit);

module_init(tpa2028d2_stereo_amp_init);
module_exit(tpa2028d2_stereo_amp_exit);


MODULE_DESCRIPTION("Support for TPA2028D Stereo Amp Control -> Updated by <meehwa.yun@lge.com> ");
MODULE_AUTHOR("Kim EunHye <ehgrace.kim@lge.com> ");
MODULE_LICENSE("GPL");
