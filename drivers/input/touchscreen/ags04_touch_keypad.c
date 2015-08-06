/*
 * drivers/input/touchscreen/ags04_touch_keypad.c - Touch keypad driver
 *
 * Copyright (C) 2013 LGE, Inc
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <mach/vreg.h>
#include <mach/board_lge.h>
#include <linux/random.h>
#include <linux/jiffies.h>
#include <linux/kfifo.h>
#include <linux/input/lge_touch_core.h>
#include <linux/input/ags04_touch_keypad.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

struct TSKEY {
	unsigned int keycode;
	unsigned char sensitivity;
};

struct ags04_device {
	struct i2c_client	*client;			/* i2c client for adapter */
	struct input_dev	*input_dev;			/* input device for android */

	struct work_struct	work_init;			/* work for init and ESD reset */
	struct work_struct	work_getkey;			/* work for get touch key data */
	struct delayed_work	work_report;			/* work for send event to framework */
	struct delayed_work	work_lock;			/* Key lock during TSP active */

	struct kfifo		key_fifo;			/* FIFO for key data */
	spinlock_t		key_fifo_lock;			/* lock for FIFO */

	struct TSKEY		key_menu;			/* Menu key */
	struct TSKEY		key_back;			/* Back key */

	bool			touchkey_lock;			/* lock for TSP */
	bool			touchkey_btn_press;		/*whether key pressed or not */
#ifdef CONFIG_FB
	struct notifier_block	fb_notifier;
#endif
	int			irq;				/* IRQ number */
	int			gpio_attn;			/* ATTN GPIO number */
	bool			initialized;			/* Whether init or not */
	int			init_retry_cnt;			/* Re-init count in error case */

	struct regulator 	*vdd;
	struct regulator 	*vcc_i2c;
	char 			poweron;
	bool			irq_enable;
};

static struct workqueue_struct *ags04_wq = NULL;
static struct ags04_device *pDev = NULL;

#define ATTN_HIGH			1
#define ATTN_LOW			0

#define KEYCODE_SIZE			sizeof(unsigned short)
#define KFIFO_SIZE			(8 * KEYCODE_SIZE)
#define WQ_DELAY(x)			msecs_to_jiffies(x)
#define ERROR_RETRY_COUNT		10

#define I2C_M_WR			0x00
#define TSKEY_REG_DUMMY			0x00
#define TSKEY_REG_OUTPUT		0x00
#define TSKEY_REG_SOFT_RESET		0x02	/* software reset */
#define TSKEY_REG_SENS_CAL_SPEED	0x0C	/* sense cal speed */
#define TSKEY_REG_RND_CAL_SPEED		0x0D	/* rnd cal speed */
#define TSKEY_REG_OUTPUT_EXP_TIME	0x0E	/* output expiration time */
#define TSKEY_REG_CHANNEL		0x01	/* channel */
#define TSKEY_REG_SENS1			0x04
#define TSKEY_REG_SENS2                 0x05
#define TSKEY_REG_SENS3                 0x06
#define TSKEY_REG_SENS4                 0x07
#define TSKEY_VAL_CH1_EN		0x01
#define TSKEY_VAL_CH2_EN		0x02
#define TSKEY_VAL_DISABLE		0x00

#define TSKEY_VAL_S2_MENU		0x02
#define TSKEY_VAL_S3_BACK		0x01
#define TSKEY_VAL_RELEASE		0x00
#define TSKEY_VAL_ALL			(TSKEY_VAL_S2_MENU | TSKEY_VAL_S3_BACK)
#define TSKEY_VAL_CANCEL		0xFF
#define TSKEY_BTN_PRESS			1
#define TSKEY_BTN_RELEASE		0
#define TSKEY_POWER_ON			1
#define TSKEY_POWER_OFF			0

struct reg_code_table {
	u8 reg;
	u8 val;
};

static struct reg_code_table initial_code_table[] = {
	{TSKEY_REG_SOFT_RESET,		0x4D}, /* Software Reset enable */
	{TSKEY_REG_SENS_CAL_SPEED,	0x33}, /* Sens Cal speed Fast set */
//	{TSKEY_REG_RND_CAL_SPEED,	0x33}, /* RND no use */
	{TSKEY_REG_OUTPUT_EXP_TIME,	0x00}, /* Output Expiration time disable */
	{TSKEY_REG_CHANNEL,		0x03}, /* CH2, CH1 enable */
	{TSKEY_REG_SENS1,		0x03}, /* CH1 Sensitivity */
	{TSKEY_REG_SENS2,		0x03}, /* CH2 Sensitivity */
	{TSKEY_REG_SOFT_RESET,		0x4C}, /* Multi output mode set */
};

static int tk_regulator_configure(struct ags04_device *ts, bool on);
static int tk_power_on(struct ags04_device *ts, bool on);

static int ags04_i2c_write(u8 reg, u8 data)
{
	struct i2c_client *client = pDev->client;
	int rc;

	rc = i2c_smbus_write_byte_data(client, reg, data);
	if (rc) {
		dev_err(&client->dev, "i2c error %d while writing %d to register %d\n", rc, data, reg);
	}

	return rc;
}

static int ags04_i2c_read(u8 reg)
{
	struct i2c_client *client = pDev->client;
	int rc;

	rc = i2c_smbus_read_byte_data(client, reg);
	if (rc < 0) {
		dev_err(&client->dev, "i2c error %d while reading register %d\n", rc, reg);
	}

	return rc;
}

static void ags04_i2c_power_onoff(bool onoff)
{
	pDev->initialized = false;
	pDev->init_retry_cnt = 0;
#if 0 //us10_temp
	pDev->tk_power(onoff, true);
#else
#if 0 // No recommend beacuse of delay (100ms ~ 7sec)
	if (onoff) {
		ags04_i2c_write(TSKEY_REG_CHANNEL, (TSKEY_VAL_CH1_EN|TSKEY_VAL_CH2_EN));
	} else {
		ags04_i2c_write(TSKEY_REG_CHANNEL, TSKEY_VAL_DISABLE);
	}
#endif
#endif
}

void ags04_keytouch_cancel(void)
{
	u16 btn_state = TSKEY_VAL_CANCEL;
	
	pDev->touchkey_lock = true;

	if (pDev->touchkey_btn_press) {
		spin_lock(&pDev->key_fifo_lock);

		cancel_delayed_work_sync(&pDev->work_lock);
		cancel_delayed_work_sync(&pDev->work_report);

		kfifo_reset(&pDev->key_fifo);
		kfifo_in(&pDev->key_fifo, &btn_state, KEYCODE_SIZE);

		spin_unlock(&pDev->key_fifo_lock);
		
		schedule_delayed_work(&pDev->work_report, WQ_DELAY(0));
	}

}

void ags04_keytouch_lock_free(void)
{
	if (pDev->touchkey_lock)
		schedule_delayed_work(&pDev->work_lock, WQ_DELAY(200));
}

static int ags04_i2c_initialize(void)
{
	int ret = 0;
	int i = 0;

	TOUCH_INFO_MSG("%s \n", __func__);
	if (pDev->init_retry_cnt >= ERROR_RETRY_COUNT)
		return -1;

	for (i = 0; i < ARRAY_SIZE(initial_code_table); i++) {
		ret = ags04_i2c_write(initial_code_table[i].reg, initial_code_table[i].val);
		if (unlikely(ret < 0)) {
			TOUCH_ERR_MSG("write initial_code_table failed (I2C NAK) \n");
			goto Exit;
		}
	}

	/*DUMMY read*/
	TOUCH_INFO_MSG("%s ags04_i2c_read - start!! \n", __func__);
	ret = ags04_i2c_read(TSKEY_REG_DUMMY);

	if (unlikely(ret < 0))
		goto Exit;

	pDev->initialized = true;
	pDev->init_retry_cnt = 0;
	
	TOUCH_INFO_MSG("%s: initialized \n", __func__);

	return 0;

Exit :
	pDev->initialized = false;
	pDev->init_retry_cnt++;

	TOUCH_INFO_MSG("%s: Failed \n", __func__);
	
	return -1;
}

static int ags04_i2c_uninit(void)
{
	TOUCH_INFO_MSG("%s \n", __func__);
	if(pDev->irq_enable == false) {
		return -1;
	}

	if (likely(pDev->irq)) {
		disable_irq_nosync(pDev->irq);
		pDev->irq_enable = false;
	}

	//ags04_i2c_power_onoff(TSKEY_POWER_OFF);

	return 0;
}

static void ags04_i2c_enqueue_keycode(u16 btn_state)
{
	unsigned long delay = 0;

	TOUCH_INFO_MSG("%s \n", __func__);

	if (pDev->touchkey_lock == false) {
		kfifo_in_locked(&pDev->key_fifo, &btn_state, KEYCODE_SIZE, &pDev->key_fifo_lock);

		if (btn_state == TSKEY_VAL_RELEASE)
			delay = WQ_DELAY(30);

		schedule_delayed_work(&pDev->work_report, delay);
	} else {
		TOUCH_INFO_MSG("KEY is Locked \n");
	}

	if (likely(pDev->irq)) {
		enable_irq(pDev->irq);
		pDev->irq_enable = true;
	}
}

static void ags04_i2c_read_keycode(void)
{
	static int retry_cnt = 0;
	int ret = 0;
	u16 btn_state = 0;

	TOUCH_INFO_MSG("%s \n", __func__);
RETRY :
	// Read IC first in order to check proper IRQ.
	btn_state = ags04_i2c_read(TSKEY_REG_OUTPUT);
	ret = btn_state;
	TOUCH_INFO_MSG(" TSKEY_VAL_CH2_EN(0x02.MENU) TSKEY_VAL_CH1_EN(0x01.BACK) \n");
	TOUCH_INFO_MSG("ags04_i2c_read_keycode btn_state:%2x \n", btn_state);

	if ((ret >= TSKEY_VAL_RELEASE) && (ret <= TSKEY_VAL_ALL)) {

		btn_state &= (TSKEY_VAL_CH2_EN|TSKEY_VAL_CH1_EN);
		switch(btn_state)
		{
			case TSKEY_VAL_CH1_EN:
				btn_state = TSKEY_VAL_S2_MENU;
				printk(KERN_DEBUG "TSKEY_MENU Detected\n");
				break;
			case TSKEY_VAL_CH2_EN:
				btn_state = TSKEY_VAL_S3_BACK;
				printk(KERN_DEBUG "TSKEY_BACK Detected\n");
				break;
			case TSKEY_VAL_RELEASE:
				btn_state = TSKEY_VAL_RELEASE;
				printk(KERN_DEBUG "TSKEY_ALL Released\n");
				break;
		}

		ags04_i2c_enqueue_keycode(btn_state);
		return;
	}

	//  I2C error means something wrong. Check again.
	if (likely(pDev->gpio_attn)) {
		if (gpio_get_value(pDev->gpio_attn) == ATTN_LOW) {
			TOUCH_INFO_MSG("ATTN is Low. Check Reset %d/%d \n", ++retry_cnt, ERROR_RETRY_COUNT);
			mdelay(10);
			if (retry_cnt >= ERROR_RETRY_COUNT) {
#if 1 //us10_temp_20130823
				TOUCH_ERR_MSG("ags04_i2c_read_keycode:I2C read error something wrong!!!!! \n");
				TOUCH_ERR_MSG("ags04 Start Re-Initialize");

				pDev->initialized = false;
				retry_cnt = 0;

				TOUCH_INFO_MSG("ags04 TK power on 1\n");
				ret = tk_regulator_configure(pDev, true);
				if (ret < 0) {
					dev_err(&pDev->client->dev, "[TOUCH] Failed to configure regulators\n");
				}
				TOUCH_INFO_MSG("ags04 TK power on 2\n");
				dev_err(&pDev->client->dev, "[TOUCH] Power Control : probe %d\n", __LINE__);
				ret = tk_power_on(pDev, true);
				if (ret < 0) {
					dev_err(&pDev->client->dev, "[TOUCH] Failed to power on\n");
				}
				TOUCH_INFO_MSG("ags04 TK power on 3 - finish\n");
				msleep(500);

				queue_work(ags04_wq, &pDev->work_init);
#endif
				return;
			} else {
				retry_cnt++;
				goto RETRY;
			}
		}
	}
	if (likely(pDev->irq)) {
		enable_irq(pDev->irq);
		pDev->irq_enable = true;
	}
}

static void ags04_i2c_work_init(struct work_struct *work)
{
	int retry_cnt = 0;
	int ret = 0;

	TOUCH_INFO_MSG("%s \n", __func__);

	if (unlikely(pDev->initialized == false)) {
		TOUCH_INFO_MSG("%s - 1\n", __func__);
INIT_FAIL_RETRY:
		ret = ags04_i2c_initialize();
		if (ret < 0) {
			if (retry_cnt++ < ERROR_RETRY_COUNT) {
				TOUCH_ERR_MSG("ags04_i2c_initialize. fail... : %d/%d \n", retry_cnt, ERROR_RETRY_COUNT);
				goto INIT_FAIL_RETRY;
			}
			else {
				TOUCH_ERR_MSG("IC failed. Power down.\n");
				ags04_i2c_uninit();
				return;
			}
		}
	}

	if (likely(pDev->irq)) {
		enable_irq(pDev->irq);
		pDev->irq_enable = true;
	}
}

static void ags04_i2c_work_getkey(struct work_struct *work)
{
	TOUCH_INFO_MSG("%s \n", __func__);
	ags04_i2c_read_keycode();
}

static void ags04_i2c_work_report(struct work_struct *work)
{
	unsigned int fifo_num = 0;
	unsigned int fifo_cnt = 0;
	unsigned int keycode = 0;
	static unsigned int old_keycode = 0;
	u16 btn_state = 0;
	int readbyte = 0;
	int i;

	TOUCH_INFO_MSG("%s \n", __func__);

	spin_lock(&pDev->key_fifo_lock);

	fifo_num = kfifo_len(&pDev->key_fifo);
	if (fifo_num == 0) {
		goto Exit;
	}

	fifo_cnt = fifo_num / KEYCODE_SIZE;

	for (i = 0; i < fifo_cnt; i++) {
		readbyte = kfifo_out(&pDev->key_fifo, &btn_state, KEYCODE_SIZE);
		if (unlikely(readbyte != KEYCODE_SIZE)) {
			TOUCH_ERR_MSG("Wrong FIFO size \n");
			goto Exit;
		}

		if (btn_state == TSKEY_VAL_CANCEL) {
			input_report_key(pDev->input_dev, old_keycode,		TSKEY_VAL_CANCEL);
			TOUCH_INFO_MSG("KEY[%2X] is canceled\n", old_keycode);
			input_sync(pDev->input_dev);
			pDev->touchkey_btn_press = false;
			goto Exit;
		}
		
		switch (btn_state) {
			case TSKEY_VAL_S2_MENU :
				keycode = pDev->key_menu.keycode;
				break;
			case TSKEY_VAL_S3_BACK :
				keycode = pDev->key_back.keycode;
				break;
		}

		switch (btn_state) {
			case TSKEY_VAL_S2_MENU :
			case TSKEY_VAL_S3_BACK :
				input_report_key(pDev->input_dev, keycode,	TSKEY_BTN_PRESS);
				input_report_key(pDev->input_dev, BTN_TOUCH,	TSKEY_BTN_PRESS);
				old_keycode = keycode;
				TOUCH_INFO_MSG("KEY[%2X] is pressed\n", keycode);
				pDev->touchkey_btn_press = true;
				break;
			case TSKEY_VAL_RELEASE :
				if (pDev->touchkey_btn_press) {
					input_report_key(pDev->input_dev, pDev->key_back.keycode,	TSKEY_BTN_RELEASE);
					input_report_key(pDev->input_dev, pDev->key_menu.keycode,	TSKEY_BTN_RELEASE);
					input_report_key(pDev->input_dev, BTN_TOUCH,						TSKEY_BTN_RELEASE);
					TOUCH_INFO_MSG("KEY[%2X] is released\n", old_keycode);
					pDev->touchkey_btn_press = false;
				}
				break;
		}

		input_sync(pDev->input_dev);
		
	}

Exit :
	spin_unlock(&pDev->key_fifo_lock);

}

static void ags04_i2c_work_lock(struct work_struct *work)
{	
	TOUCH_INFO_MSG("%s \n", __func__);
	pDev->touchkey_lock = false;
}

static irqreturn_t ags04_i2c_irq_handler(int irq, void *handle)
{
	TOUCH_INFO_MSG("%s \n", __func__);

	if (likely(pDev->irq)) {
		disable_irq_nosync(pDev->irq);
		pDev->irq_enable = false;
	}

	queue_work(ags04_wq, &pDev->work_getkey);

	return IRQ_HANDLED;
}

static int ags04_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
	TOUCH_INFO_MSG("%s \n", __func__);
	ags04_i2c_uninit();

	cancel_work_sync(&pDev->work_init);
	cancel_work_sync(&pDev->work_getkey);
	cancel_delayed_work_sync(&pDev->work_lock);
	cancel_delayed_work_sync(&pDev->work_report);

	pDev->touchkey_lock = false;
	pDev->touchkey_btn_press = false;
	kfifo_reset(&pDev->key_fifo);

	return 0;
}


static int ags04_i2c_resume(struct i2c_client *client)
{
	TOUCH_INFO_MSG("%s \n", __func__);
	//ags04_i2c_power_onoff(TSKEY_POWER_ON);

	if(pDev->irq_enable) {
		return -1;
	}
	if (likely(pDev->irq)) {
		enable_irq(pDev->irq);
		pDev->irq_enable = true;
	}

	return 0;
}

#ifdef CONFIG_FB
static int ags04_ts_fb_notifier_call(struct notifier_block *self,
				   unsigned long event,
				   void *data)
{
	struct fb_event *evdata = data;
	int *fb;

	struct ags04_device *pDev = container_of(self, struct ags04_device, fb_notifier);
	TOUCH_INFO_MSG("%s \n", __func__);

	if(evdata && evdata->data && event == FB_EVENT_BLANK && pDev && pDev->client) {
		fb = evdata->data;
		switch (*fb) {
			case FB_BLANK_UNBLANK:
				ags04_i2c_resume(pDev->client);
				break;
			case FB_BLANK_POWERDOWN:
				ags04_i2c_suspend(pDev->client, PMSG_SUSPEND);
				break;
			default:
				break;
		}
	}
	return 0;
}
#endif

static ssize_t ags04_set_sen_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "MENU[%4d] BACK[%4d]\n",
			pDev->key_menu.sensitivity, pDev->key_back.sensitivity);
}

static ssize_t ags04_test_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int a=0, b=0, c=0, d=0, e=0, f=0;

	a = ags04_i2c_read(TSKEY_REG_SOFT_RESET);
	if (a < 0)
		TOUCH_ERR_MSG("ags04 [TK] i2c read fail a\n");

	b = ags04_i2c_read(TSKEY_REG_SENS_CAL_SPEED);
	if (b < 0)
		TOUCH_ERR_MSG("ags04 [TK] i2c read fail b\n");

	c = ags04_i2c_read(TSKEY_REG_OUTPUT_EXP_TIME);
	if (c < 0)
		TOUCH_ERR_MSG("ags04 [TK] i2c read fail c\n");

	d = ags04_i2c_read(TSKEY_REG_CHANNEL);
	if (d < 0)
		TOUCH_ERR_MSG("ags04 [TK] i2c read fail d\n");

	e = ags04_i2c_read(TSKEY_REG_SENS1);
	if (e < 0)
		TOUCH_ERR_MSG("ags04 [TK] i2c read fail e\n");

	f = ags04_i2c_read(TSKEY_REG_SENS2);
	if (f < 0)
		TOUCH_ERR_MSG("ags04 [TK] i2c read fail f\n");

	return sprintf(buf, "0x02[%x] 0x0C[%x] 0x0E[%x] 0x01[%x] 0x04[%x] 0x05[%x]\n", a, b, c, d, e, f);
}

static ssize_t ags04_set_sen_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int config_value = 0;
	unsigned long after = 0;
	int ret = 0;

	ret = strict_strtoul(buf, 10, &after);
	if (ret)
		return -EINVAL;

	TOUCH_INFO_MSG("%s\n", __func__);

	/* config_value * 3.2 + 11.2 = after
	 * eg)[0x03]: (0x03 * 3.2) + 11.2 = 20.8msec */
	config_value = ( after - 112 ) / 32;

	pDev->key_menu.sensitivity = config_value;
	pDev->key_back.sensitivity = config_value;

	TOUCH_INFO_MSG("ags04 Set Menu Key Sen[%d]\n", config_value);
	ags04_i2c_write(TSKEY_REG_SENS1, config_value);

	TOUCH_INFO_MSG("ags04 Set Back Key Sen[%d]\n", config_value);
	ags04_i2c_write(TSKEY_REG_SENS2, config_value);

	return size;
}

static ssize_t ags04_reset_test(struct device *dev, struct device_attribute *attr, char *buf)
{
	int delay = 100;

	TOUCH_INFO_MSG("power will down during %d ms \n", delay);

	//pDev->tk_power(TSKEY_POWER_OFF, true);
	
	msleep(delay);

	//pDev->tk_power(TSKEY_POWER_ON, true);

	return 4;
}

static struct device_attribute dev_attr_device_test[] = {
	__ATTR(set_sen, S_IRUGO | S_IWUSR | S_IXOTH, ags04_set_sen_show, ags04_set_sen_store),
	__ATTR(test, S_IRUGO | S_IWUSR | S_IXOTH, ags04_test_show, NULL),
	__ATTR(reset, S_IRUGO | S_IWUSR | S_IXOTH, ags04_reset_test, NULL),
};

static int ags04_touch_keypad_parse_dt(struct device *dev,
				struct key_touch_platform_data *pData)
{
	struct device_node *np = dev->of_node;
	struct property *prop;
	int rc = 0;

	TOUCH_INFO_MSG("%s \n", __func__);

	/* irq gpio info */
	pData->irq = of_get_named_gpio_flags(np,
			"ags04,irq_gpio", 0, &pData->gpio_int);
	
	TOUCH_INFO_MSG("pData->irq : %d \n", pData->irq);
	TOUCH_INFO_MSG("pData->gpio_int : %d \n", pData->gpio_int);

	/* button */
	prop = of_find_property(np, "ags04,button-map", NULL);
	if (prop) {
		pData->keycodemax = prop->length / sizeof(unsigned int);
		TOUCH_INFO_MSG("keys : %d , prop->length : %d \n", pData->keycodemax, prop->length);
		rc = of_property_read_u32_array(np, "ags04,button-map", pData->keycode, pData->keycodemax);
		if (rc) {
			dev_err(dev, "failed to read key-map\n");
		}
	} else {
		dev_err(dev, "failed to find key-map in device tree\n");
	}
	TOUCH_INFO_MSG("pData->keycode[0] : %d , pData->keycode[1] : %d\n", pData->keycode[0],pData->keycode[1]);
	TOUCH_INFO_MSG("pData->keycodemax : %d \n", pData->keycodemax);
	return 0;
}

static int tk_power_on(struct ags04_device *ts, bool on)
{
	int retval = 0;

	TOUCH_INFO_MSG("%s \n", __func__);

	if (on == false)
		goto power_off;

	if (ts->poweron)
		return 0;

	if(ts->vdd) {
		retval = regulator_enable(ts->vdd);
		printk(KERN_ERR "%d [TOUCH] On : regulator_enable(VDD) = %d \n", __LINE__, retval);
		if (retval < 0) {
			dev_err(&ts->client->dev, "[TOUCH] Regulator vdd enable failed retval = %d\n", retval);
		}
	}
#if 0
	if(ts->vcc_i2c) {
		retval = regulator_enable(ts->vcc_i2c);
		printk(KERN_ERR "%d [TOUCH] On : regulator_enable(I2C) = %d \n", __LINE__, retval);
		if (retval < 0) {
			dev_err(&ts->client->dev, "[TOUCH] Regulator i2c enable failed retval = %d\n", retval);
		}
	}
#endif
	ts->poweron = 1;

	goto exit;


power_off :

	if (ts->poweron == 0)
		return 0;

	if(ts->vdd) {
		regulator_disable(ts->vdd);
		dev_err(&ts->client->dev, "[TOUCH] regulator_disable(VDD) \n");
	}
#if 0
	if(ts->vcc_i2c) {
		regulator_disable(ts->vcc_i2c);
		dev_err(&ts->client->dev, "[TOUCH] regulator_disable(I2C) \n");
	}
#endif
	ts->poweron = 0;

exit :
	printk(KERN_ERR "[TOUCH] power delay 30ms \n");
	msleep(30);

	return retval;
}

static int tk_regulator_configure(struct ags04_device *ts, bool on)
{
	int retval = 0;

	TOUCH_INFO_MSG("%s \n", __func__);

	if (on == false)
		goto hw_shutdown;

	if(ts->vdd == NULL) {
		ts->vdd = regulator_get(&ts->client->dev, "vdd");
		if (IS_ERR(ts->vdd)) {
			dev_err(&ts->client->dev, "[TOUCH] %s: Failed to get vdd regulator\n", __func__);
			return PTR_ERR(ts->vdd);
		}
		dev_err(&ts->client->dev, "[TOUCH] regulator_get(VDD) \n");
	}
#if 0
	if(ts->vcc_i2c == NULL) {
		ts->vcc_i2c = regulator_get(&ts->client->dev, "vcc_i2c");
		if (IS_ERR(ts->vcc_i2c)) {
			dev_err(&ts->client->dev, "[TOUCH] %s: Failed to get i2c regulator\n", __func__);
			return PTR_ERR(ts->vcc_i2c);
		}
		dev_err(&ts->client->dev, "[TOUCH] regulator_get(I2C) \n");
	}
#endif
	if(ts->vdd) {
		retval = regulator_set_voltage(ts->vdd, 1800000, 1800000);
		if (retval)
			dev_err(&ts->client->dev, "[TOUCH] regulator_set_voltage(VDD) failed retval=%d\n", retval);
		else
			dev_err(&ts->client->dev, "[TOUCH] regulator_set_voltage(VDD) \n");
	}
#if 0
	if(ts->vcc_i2c) {
		retval = regulator_set_voltage(ts->vcc_i2c, 1800000, 1800000);
		if (retval)
			dev_err(&ts->client->dev, "[TOUCH] regulator_set_voltage(I2C) failed retval=%d\n", retval);
		else
			dev_err(&ts->client->dev, "[TOUCH] regulator_set_voltage(I2C) \n");
	}
#endif
	return 0;

hw_shutdown :
	if(ts->vdd) {
		regulator_put(ts->vdd);
		dev_err(&ts->client->dev, "[TOUCH] regulator_put(VDD) \n");
	}

	if(ts->vcc_i2c) {
		regulator_put(ts->vcc_i2c);
		dev_err(&ts->client->dev, "[TOUCH] regulator_put(I2C) \n");
	}
	return retval;
}

static int ags04_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct key_touch_platform_data *pData = NULL;
	unsigned char keycode = KEY_UNKNOWN;
	int i = 0;
	int ret = 0;

	TOUCH_INFO_MSG("%s \n", __func__);

	if (unlikely(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))) {
		TOUCH_ERR_MSG("it is not support I2C_FUNC_I2C. \n");
		return -ENODEV;
	}

	pDev = kzalloc(sizeof(struct ags04_device), GFP_KERNEL);
	if (unlikely(pDev == NULL)) {
		TOUCH_ERR_MSG("failed to allocation \n");
		return -ENOMEM;
	}

	pDev->initialized = false;
	pDev->client = client;

	if (client->dev.of_node) {
		pData = devm_kzalloc(&client->dev,
			sizeof(*pData),
			GFP_KERNEL);
		if (!pData) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		ret = ags04_touch_keypad_parse_dt(&client->dev, pData);
		if (ret)
			return ret;
	} else {
		pData = client->dev.platform_data;
	}

#if 1 //us10_temp_0820
	TOUCH_INFO_MSG("ags04 TK power on 1\n");
	ret = tk_regulator_configure(pDev, true);
	if (ret < 0) {
		dev_err(&client->dev, "[TOUCH] Failed to configure regulators\n");
		goto Exit;
	}
	TOUCH_INFO_MSG("ags04 TK power on 2\n");
	dev_err(&pDev->client->dev, "[TOUCH] Power Control : probe %d\n", __LINE__);
	ret = tk_power_on(pDev, true);
	if (ret < 0) {
		dev_err(&client->dev, "[TOUCH] Failed to power on\n");
		goto Exit;
	}
	TOUCH_INFO_MSG("ags04 TK power on 3 - finish\n");
	msleep(500);
#endif
	pDev->key_back.keycode = KEY_BACK;
	pDev->key_menu.keycode = KEY_MENU;
	pDev->key_back.sensitivity = initial_code_table[4].val;
	pDev->key_menu.sensitivity = initial_code_table[5].val;

	pDev->gpio_attn = pData->irq;
	pDev->irq = client->irq;

	i2c_set_clientdata(pDev->client, pDev);

	pDev->input_dev = input_allocate_device();
	if (unlikely(pDev->input_dev == NULL)) {
		TOUCH_ERR_MSG("failed to input allocation \n");
		goto Exit;
	}

	pDev->input_dev->name = "touch_keypad";
	pDev->input_dev->phys = "touch_keypad/i2c";
	pDev->input_dev->id.vendor = VENDOR_LGE;
	pDev->input_dev->evbit[0] = BIT_MASK(EV_KEY);
	pDev->input_dev->keycode = pData->keycode;
	pDev->input_dev->keycodesize = sizeof(unsigned short);
	pDev->input_dev->keycodemax = pData->keycodemax;

	TOUCH_INFO_MSG("pData->keycodemax:%d \n", pData->keycodemax);
	for (i = 0; i < pData->keycodemax; i++) {
		keycode = pData->keycode[i];
		TOUCH_INFO_MSG("keycode:%d, \n", keycode);
		set_bit(keycode, pDev->input_dev->keybit);
	}


	ret = input_register_device(pDev->input_dev);
	if (unlikely(ret < 0)) {
		TOUCH_ERR_MSG("failed to register input %s \n", pDev->input_dev->name);
		goto Exit;
	}

	INIT_WORK(&pDev->work_init, ags04_i2c_work_init);
	INIT_WORK(&pDev->work_getkey, ags04_i2c_work_getkey);
	INIT_DELAYED_WORK(&pDev->work_lock, ags04_i2c_work_lock);
	INIT_DELAYED_WORK(&pDev->work_report, ags04_i2c_work_report);

	spin_lock_init(&pDev->key_fifo_lock);
	if (kfifo_alloc(&pDev->key_fifo, KFIFO_SIZE, GFP_KERNEL)) {
		TOUCH_ERR_MSG("failed to kfifo allocation \n");
		goto Exit;
	}

#ifdef CONFIG_FB
	pDev->fb_notifier.notifier_call = ags04_ts_fb_notifier_call;
	ret = fb_register_client(&pDev->fb_notifier);
	if (ret) {
		TOUCH_ERR_MSG("%s failed to register fb_notifier: %d\n", __func__, ret);
	}
#endif

	if (likely(pDev->gpio_attn)) {
		ret = gpio_request(pDev->gpio_attn, "touch_key_attn");
		if (unlikely(ret < 0)) {
			TOUCH_ERR_MSG("gpio_attn_request %d failed\n", pDev->gpio_attn);
		} else {
			gpio_direction_input(pDev->gpio_attn);
		}
	}

	for (i = 0; i < ARRAY_SIZE(dev_attr_device_test); i++) {
		ret = device_create_file(&client->dev, &dev_attr_device_test[i]);
		if (ret)
			TOUCH_ERR_MSG("%s: device_create_file fail\n", __func__);
	}

	ags04_i2c_power_onoff(TSKEY_POWER_ON);

	if(likely(pDev->irq)) {
		if (unlikely(request_irq(pDev->irq, ags04_i2c_irq_handler, IRQF_TRIGGER_FALLING|IRQF_ONESHOT, client->name, pDev)))
			TOUCH_ERR_MSG("request_irq(%d) failed \n", pDev->irq);
	}
#if 1 //us10_temp_20130826
	disable_irq_nosync(pDev->irq);
	pDev->irq_enable = false;
#endif
	queue_work(ags04_wq, &pDev->work_init);
Exit :
	return ret;
}

static int ags04_i2c_remove(struct i2c_client *client)
{
	int i = 0;

	TOUCH_INFO_MSG("%s \n", __func__);

	for (i = 0; i < ARRAY_SIZE(dev_attr_device_test); i++)
		device_remove_file(&client->dev, &dev_attr_device_test[i]);

#ifdef CONFIG_FB
	fb_unregister_client(&pDev->fb_notifier);
#endif

	if (likely(pDev->irq))
		free_irq(pDev->irq, pDev);

	if (likely(pDev->gpio_attn))
		gpio_free(pDev->gpio_attn);

	input_unregister_device(pDev->input_dev);
	input_free_device(pDev->input_dev);
	kfifo_free(&pDev->key_fifo);
	kfree(pDev);

	ags04_i2c_power_onoff(TSKEY_POWER_OFF);
	
	return 0;
}

static const struct i2c_device_id ags04_i2c_ids[] = {
		{AGS04_KEYTOUCH_NAME, 0 },
		{ },
};

static struct of_device_id ags04_match_table[] = {
	{ .compatible = "ags04,touch_keypad",},
	{ },};

static struct i2c_driver ags04_i2c_driver = {
	.probe		= ags04_i2c_probe,
	.remove		= ags04_i2c_remove,
	.id_table	= ags04_i2c_ids,
	.driver = {
		.name	= AGS04_KEYTOUCH_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = ags04_match_table,
	},
};

static int __init ags04_init(void)
{
	int ret = 0;

	TOUCH_INFO_MSG("%s \n", __func__);

	ags04_wq = create_singlethread_workqueue("ags04_wq");
	if (unlikely(!ags04_wq)) {
		TOUCH_ERR_MSG("failed to create singlethread ags04_wq \n");
		return -ENOMEM;
	}
	
	ret = i2c_add_driver(&ags04_i2c_driver);
	if (unlikely(ret < 0)) {
		TOUCH_ERR_MSG("failed to i2c_add_driver \n");
		destroy_workqueue(ags04_wq);
		return ret;
	}

	return 0;
}

static void __exit ags04_exit(void)
{
	TOUCH_INFO_MSG("%s \n", __func__);
	
	i2c_del_driver(&ags04_i2c_driver);

	if (likely(ags04_wq))
		destroy_workqueue(ags04_wq);
	
	return;
}

module_init(ags04_init);
module_exit(ags04_exit);

MODULE_DESCRIPTION("AGS04 touch keypad driver");
MODULE_LICENSE("GPL");

