/*
 * Touchscreen driver for Melfas MMS-100s series 
 *
 * Copyright (C) 2013 Melfas Inc.
 * Author: DVK team <dvk@melfas.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include "mms100s_ts.h"
#include <linux/completion.h>
#include <linux/init.h>
#include <asm/uaccess.h>
#include <asm/unaligned.h>
#include <linux/file.h>
#include <linux/syscalls.h>
#include <linux/time.h>

#if defined(CONFIG_MACH_MSM8X10_W5_MPCS_US)
#define ISIS_L2 /* block the exposure of privacy information */
#endif

#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif
#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
#include <linux/regulator/consumer.h>
#include "mms100s_isp.h"

#define SENSING_TEST
#ifdef SENSING_TEST
#include <linux/uaccess.h>
#define SENSING_TEST_PATH		"/data/ts_log.txt"
#endif

#define LGE_TOUCH_NAME		"lge_touch"

#define IC_ERROR_RETRY_CNT	5

#define POWER_FW_UP_LOCK	0x01
#define POWER_SYSFS_LOCK	0x02

#define MMS_TS_FW_VERIFIY

/*
 * ISC_XFER_LEN	- ISC unit transfer length.
 * Give number of 2 power n, where  n is between 2 and 10 
 * i.e. 4, 8, 16 ,,, 1024 
 */

#define ISC_XFER_LEN			256

#define MMS_FLASH_PAGE_SZ	1024
#define ISC_BLOCK_NUM		(MMS_FLASH_PAGE_SZ / ISC_XFER_LEN)

#define FLASH_VERBOSE_DEBUG	1
#define MAX_SECTION_NUM		3

#define MAX_FINGER_NUM		5
#define MAX_BUTTON_NUM		4
#define FINGER_EVENT_SZ		6
#define MAX_WIDTH		30
#define MAX_PRESSURE		255
#define MAX_LOG_LENGTH		128

/* Registers */
#define MMS_MODE_CONTROL	0x01
#define MMS_TX_NUM		0x0B
#define MMS_RX_NUM		0x0C
#define MMS_EVENT_PKT_SZ	0x0F
#define MMS_INPUT_EVENT		0x10
#define MMS_UNIVERSAL_CMD	0xA0
#define MMS_UNIVERSAL_RESULT	0xAF
#define MMS_CMD_ENTER_ISC	0x5F
#define MMS_FW_VERSION		0xE1
#define MMS_FW_PRODUCT		0xF6
#define MMS_CMD_RESULT_SIZE		0xAE
#define MMS_UNIVERSAL_CMD_EXIT			0x4F
#define MMS_TEST_CH_STATUS			0x41
#define MMS_GET_PIXEL_CH_STATUS		0x42
#define MMS_KEY_CH_STATUS			0x4A
#define MMS_TEST_RAW_DATA			0x43
#define MMS_GET_PIXEL_RAW_DATA		0x44
#define MMS_KEY_RAW_DATA			0x4B
#define MMS_TEST_JITTER				0x45
#define MMS_GET_PIXEL_JITTER		0x46
#define MMS_KEY_JITTER				0x4C
#define MMS_UNIVERSAL_DELTA			0x70
#define MMS_POWER_CONTROL		0xB0
#define MMS_SET_EDGE_EXPAND 0x32
#define UNIVCMD_GET_TOP_EDGE_EXPAND			0x25
#define UNIVCMD_GET_BOTTOM_EDGE_EXPAND		0x26
#define UNIVCMD_GET_LEFT_EDGE_EXPAND			0x27
#define UNIVCMD_GET_RIGHT_EDGE_EXPAND		0x28

/* Universal commands */
#define MMS_CMD_SET_LOG_MODE	0x20

/* Event types */
#define MMS_LOG_EVENT		0xD
#define MMS_NOTIFY_EVENT	0xE
#define MMS_ERROR_EVENT		0xF
#define MMS_TOUCH_KEY_EVENT	0x40

/* Firmware file name */
#define EXTERNAL_FW_NAME	"mms_ts.mfsb"
#define CORE32_FW_NAME		"melfas/mms100s_core32_v01.mfsb"

static unsigned char id_state[MAX_FINGER_NUM];
static int touched_count = 0;
static volatile int irq_flag;
static uint8_t edge_expand[4] = {0};
#ifdef SENSING_TEST
static char sensing_test = 0;
#endif

int power_block = 0;

enum {
	GET_RX_NUM	= 1,
	GET_TX_NUM,
	GET_EVENT_DATA,
};

enum {
	LOG_TYPE_U08	= 2,
	LOG_TYPE_S08,
	LOG_TYPE_U16,
	LOG_TYPE_S16,
	LOG_TYPE_U32	= 8,
	LOG_TYPE_S32,
};

enum {
	ISC_ADDR		= 0xD5,

	ISC_CMD_READ_STATUS	= 0xD9,	
	ISC_CMD_READ		= 0x4000,
	ISC_CMD_EXIT		= 0x8200,
	ISC_CMD_PAGE_ERASE	= 0xC000,
	
	ISC_PAGE_ERASE_DONE	= 0x10000,
	ISC_PAGE_ERASE_ENTER	= 0x20000,
};

struct mms_ts_info {
	struct i2c_client 		*client;
	struct input_dev 		*input_dev;
	char 				phys[32];

	int 				irq;

	struct mms_ts_platform_data 	*pdata;

	struct completion 		init_done;
#ifdef CONFIG_FB
	struct notifier_block		fb_notifier;
#endif
	struct mutex 			lock;
	bool				enabled;

	struct cdev			cdev;
	dev_t				mms_dev;
	struct class			*class;

	struct mms_log_data {
		u8			*data;
		int			cmd;
	} log;

	struct regulator *vdd;
	struct regulator *vdd_i2c;
	struct regulator *vdd_int;
	char fw_update;
	char pannel_on;
	int ic_error_cnt;
};

struct mms_bin_hdr {
	char	tag[8];
	u16	core_version;
	u16	section_num;
	u16	contains_full_binary;
	u16	reserved0;

	u32	binary_offset;
	u32	binary_length;

	u32	extention_offset;	
	u32	reserved1;
	
} __attribute__ ((packed));

struct mms_fw_img {
	u16	type;
	u16	version;

	u16	start_page;
	u16	end_page;

	u32	offset;
	u32	length;

} __attribute__ ((packed));

struct isc_packet {
	u8	cmd;
	u32	addr;
	u8	data[0];
} __attribute__ ((packed));

static void mms_report_input_data(struct mms_ts_info *info, u8 sz, u8 *buf);
#ifdef CONFIG_FB
static int mms_ts_fb_notifier_call(struct notifier_block *self, unsigned long event, void *data);
#endif
static int mms_ts_config(struct mms_ts_info *info);

static void power_lock(int value)
{
	power_block |= value;
}

static void power_unlock(int value)
{
	power_block &= ~(value);
}

static int mms_ts_power(struct mms_ts_info *ts, bool on)
{
	int retval = 0;

	if (on == false)
		goto power_off;

	TOUCH_INFO_MSG("Power on \n");

	if(ts->vdd) {
		retval = regulator_enable(ts->vdd);
		if (retval < 0) {
			TOUCH_INFO_MSG("Regulator vdd enable failed retval = %d\n", retval);
		}
	}

	if(ts->vdd_int) {
		retval = regulator_enable(ts->vdd_int);
		if (retval < 0) {
			TOUCH_INFO_MSG("Regulator int enable failed retval = %d\n", retval);
		}
	}

	if (ts->pdata->gpio_vdd_en && gpio_is_valid(ts->pdata->gpio_vdd_en)) {
		gpio_direction_output(ts->pdata->gpio_vdd_en, 1);
	}

	if(ts->vdd_i2c) {
		retval = regulator_enable(ts->vdd_i2c);
		if (retval < 0) {
			TOUCH_INFO_MSG("Regulator i2c enable failed retval = %d\n", retval);
		}
	}

	goto exit;

power_off :

	TOUCH_INFO_MSG("Power off \n");

	if(ts->vdd) {
		regulator_disable(ts->vdd);
	}

	if(ts->vdd_int) {
		regulator_disable(ts->vdd_int);
	}

	if(ts->vdd_i2c) {
		regulator_disable(ts->vdd_i2c);
	}

	if (ts->pdata->gpio_vdd_en && gpio_is_valid(ts->pdata->gpio_vdd_en)) {
		gpio_direction_output(ts->pdata->gpio_vdd_en, 0);
	}

exit :
	return retval;
}

#define TOUCH_IO_VTG		1800000

static int mms_ts_regulator_configure(struct mms_ts_info *ts, bool on)
{
	int retval = 0;

	if (on == false)
		goto hw_shutdown;

	if(ts->pdata->use_vdd && ts->vdd == NULL) {
		ts->vdd = regulator_get(&ts->client->dev, "vdd");
		if (IS_ERR(ts->vdd)) {
			TOUCH_INFO_MSG("Failed to get vdd regulator\n");
			return PTR_ERR(ts->vdd);
		}
	}

	if(ts->pdata->use_vdd_int && ts->vdd_int== NULL) {
		ts->vdd_int = regulator_get(&ts->client->dev, "vdd_int");
		if (IS_ERR(ts->vdd_int)) {
			TOUCH_INFO_MSG("Failed to get int regulator\n");
			return PTR_ERR(ts->vdd_int);
		}
	}

	if(ts->pdata->use_vdd_i2c && ts->vdd_i2c == NULL) {
		ts->vdd_i2c = regulator_get(&ts->client->dev, "vdd_i2c");
		if (IS_ERR(ts->vdd_i2c)) {
			TOUCH_INFO_MSG("Failed to get i2c regulator\n");
			return PTR_ERR(ts->vdd_i2c);
		}
	}

	if(ts->vdd) {
		retval = regulator_set_voltage(ts->vdd, ts->pdata->vdd_voltage, ts->pdata->vdd_voltage);
		if (retval)
			TOUCH_INFO_MSG("regulator_set_voltage(VDD) failed retval=%d\n", retval);
	}

	if(ts->vdd_int) {
		retval = regulator_set_voltage(ts->vdd_int, TOUCH_IO_VTG, TOUCH_IO_VTG);
		if (retval)
			TOUCH_INFO_MSG("regulator_set_voltage(INT) failed retval=%d\n", retval);
	}	

	if(ts->vdd_i2c) {
		retval = regulator_set_voltage(ts->vdd_i2c, TOUCH_IO_VTG, TOUCH_IO_VTG);
		if (retval)
			TOUCH_INFO_MSG("regulator_set_voltage(I2C) failed retval=%d\n", retval);
	}

	return 0;

hw_shutdown :
	if(ts->vdd) {
		regulator_put(ts->vdd);
	}

	if(ts->vdd_int) {
		regulator_put(ts->vdd_int);
	}

	if(ts->vdd_i2c) {
		regulator_put(ts->vdd_i2c);
	}
	
	return retval;
}

static void mms_ts_enable(struct mms_ts_info *info)
{
	if (info->enabled) {
		return;
	}

	mutex_lock(&info->lock);

	mms_ts_power(info, 1);
	msleep(50);

	info->enabled = true;
	enable_irq(info->irq);

	mutex_unlock(&info->lock);
}

static void mms_ts_disable(struct mms_ts_info *info)
{
	if (!info->enabled) {
		return;
	}

	i2c_smbus_write_byte_data(info->client, MMS_POWER_CONTROL, 1);

	mutex_lock(&info->lock);
	disable_irq(info->irq);
	msleep(50);

	mms_ts_power(info, 0);
	msleep(50);

	info->enabled = false;

	mutex_unlock(&info->lock);
}

static void mms_reboot(struct mms_ts_info *info)
{
	struct i2c_adapter *adapter = to_i2c_adapter(info->client->dev.parent);

	i2c_smbus_write_byte_data(info->client, MMS_POWER_CONTROL, 1);

	i2c_lock_adapter(adapter);
	msleep(50);

	mms_ts_power(info, 0);
	msleep(150);
	mms_ts_power(info, 1);
	msleep(50);

	i2c_unlock_adapter(adapter);
}

static void mms_clear_input_data(struct mms_ts_info *info)
{
	int i;

	for (i = 0; i < MAX_FINGER_NUM; i++) {
		id_state[i] = 0;
		input_mt_slot(info->input_dev, i);
		input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, false);
	}
	input_sync(info->input_dev);
	touched_count = 0;

	return;
}

static int mms_fs_open(struct inode *node, struct file *fp)
{
	struct mms_ts_info *info;
	struct i2c_client *client;
	struct i2c_msg msg;
	u8 buf[3] = {
		MMS_UNIVERSAL_CMD,
		MMS_CMD_SET_LOG_MODE,
		true,
	};

	info = container_of(node->i_cdev, struct mms_ts_info, cdev);
	client = info->client;

	disable_irq(info->irq);
	fp->private_data = info;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.buf = buf;
	msg.len = sizeof(buf);

	i2c_transfer(client->adapter, &msg, 1);

	info->log.data = kzalloc(MAX_LOG_LENGTH * 20 + 5, GFP_KERNEL);

	mms_clear_input_data(info);

	return 0;
}

static int mms_fs_release(struct inode *node, struct file *fp)
{
	struct mms_ts_info *info = fp->private_data;

	mms_clear_input_data(info);
	mms_reboot(info);

	kfree(info->log.data);
	enable_irq(info->irq);

	return 0;
}

static void write_file(char *filename, char *data, int time)
{
	int fd = 0;
	char time_string[64] = {0};
	struct timespec my_time;
	struct tm my_date;
	mm_segment_t old_fs = get_fs();

	my_time = __current_kernel_time();
	time_to_tm(my_time.tv_sec, sys_tz.tz_minuteswest * 60 * (-1), &my_date);
	snprintf(time_string, 64, "%02d-%02d %02d:%02d:%02d.%03lu ",
		my_date.tm_mon + 1,my_date.tm_mday,
		my_date.tm_hour, my_date.tm_min, my_date.tm_sec,
		(unsigned long) my_time.tv_nsec / 1000000);

	set_fs(KERNEL_DS);
	fd = sys_open(filename, O_WRONLY|O_CREAT|O_APPEND, 0644);
	if (fd >= 0) {
		if(time > 0)
			sys_write(fd, time_string, strlen(time_string));
		sys_write(fd, data, strlen(data));
		sys_close(fd);
	}
	set_fs(old_fs);
}

static int get_limit(struct mms_ts_info *ts, char* breakpoint, int *limit_data, int *limit_jitter)
{
	int fd = 0;
	char *fname = "/data/mms_limit.txt";
	int p = 0;
	int q = 0;
	int cipher = 1;
	int r = 0;
	const struct firmware *fwlimit = NULL;
	int ret = 0;
	char* line = NULL;

	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);

	line = kzalloc(10240, GFP_KERNEL);
	if (line == NULL) {
		TOUCH_INFO_MSG("failed to allocate line\n");
		ret = -1;
		goto exit;
	}

	fd = sys_open(fname, O_RDONLY, 0);
	if ( fd < 0 ) {
		if (ts->pdata->panel_spec_name == NULL) {
			TOUCH_INFO_MSG("panel_spec_file name is null\n");
			ret = -1;
			goto exit;
		} else if (request_firmware(&fwlimit, ts->pdata->panel_spec_name, &ts->client->dev) >= 0) {
			if (fwlimit->size > 10240) {
				ret = -1;
				goto exit;
			}
			strcpy(line, fwlimit->data);
			ret = 0;
		} else {
			TOUCH_INFO_MSG("failed to request panel_spec_file ihex");
			ret = -1;
			goto exit;
		}
	} else {
		sys_read(fd, line, 10240);
		ret = 1;
	}

	q = strstr(line, breakpoint) - line;
	if (q < 0) {
		TOUCH_INFO_MSG("failed to find breakpoint. The panel_spec_file is wrong");
		ret = -1;
		goto exit;
	}

	if (limit_data == NULL)
		goto get_jitter_limit;

	memset(limit_data, 0, (ts->pdata->rx_num * ts->pdata->tx_num + ts->pdata->key_num) * 4);

	while(1) {
		if (line[q] == ',') {
			cipher = 1;
			for (p = 1; (line[q - p] >= '0') && (line[q - p] <= '9'); p++) {
				limit_data[r] += ((line[q - p] - '0') * cipher);
				cipher *= 10;
			}
			r++;
		}
		q++;

		if(line[q] == '}') {
			if(r == ts->pdata->rx_num * ts->pdata->tx_num){
				ret = -2;
				goto exit;
			}
			ret = -3;
			goto exit;
		}

		if (r == ts->pdata->rx_num * ts->pdata->tx_num + ts->pdata->key_num) {
			break;
		}
	}

	if (line)
		kfree(line);

	if (fd >= 0)
		sys_close(fd);

	set_fs(old_fs);

	if (fwlimit)
		release_firmware(fwlimit);

	return ret;

get_jitter_limit:
	cipher = 1;
	*limit_jitter = 0;
	while(1) {
		if ((line[q] >= '0') && (line[q] <= '9')) {
			*limit_jitter = (*limit_jitter) * cipher + (line[q] - '0');
			cipher *= 10;
			if (p == 0)
				p = q - 1;
		} else if (p != 0) {
			break;
		}
		q++;
	}
	if (line[p] == '-')
		*limit_jitter *= -1;

exit :
	if (line)
		kfree(line);

	if (fd >= 0)
		sys_close(fd);

	set_fs(old_fs);

	if (fwlimit)
		release_firmware(fwlimit);

	return ret;
}


static void mms_event_handler(struct mms_ts_info *info)
{
	struct i2c_client *client = info->client;
	int sz;
	int ret;
	int row_num;
	u8 reg = MMS_INPUT_EVENT;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = &reg,
			.len = 1,
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = info->log.data,
		},

	};
	struct mms_log_pkt {
		u8	marker;
		u8	log_info;
		u8	code;
		u8	element_sz;
		u8	row_sz;
	} __attribute__ ((packed)) *pkt = (struct mms_log_pkt *)info->log.data;

	memset(pkt, 0, sizeof(*pkt));

	sz = i2c_smbus_read_byte_data(client, MMS_EVENT_PKT_SZ);
	msg[1].len = sz;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
	if (ret != ARRAY_SIZE(msg)) {
		TOUCH_INFO_MSG("failed to read %d bytes of data\n", sz);
		return;
	}

	if ((pkt->marker & 0xf) == MMS_LOG_EVENT) {
		if ((pkt->log_info & 0x7) == 0x1) {
			pkt->element_sz = 0;
			pkt->row_sz = 0;

			return;
		}

		switch (pkt->log_info >> 4) {
		case LOG_TYPE_U08:
		case LOG_TYPE_S08:
			msg[1].len = pkt->element_sz;
			break;
		case LOG_TYPE_U16:
		case LOG_TYPE_S16:
			msg[1].len = pkt->element_sz * 2;
			break;
		case LOG_TYPE_U32:
		case LOG_TYPE_S32:
			msg[1].len = pkt->element_sz * 4;
			break;
		default:
			TOUCH_INFO_MSG("invalied log type\n");
			return;
		}

		msg[1].buf = info->log.data + sizeof(struct mms_log_pkt);
		reg = MMS_UNIVERSAL_RESULT;
		row_num = pkt->row_sz ? pkt->row_sz : 1;

		while (row_num--) {
			ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));
			msg[1].buf += msg[1].len;
		};
	} else {
		mms_report_input_data(info, sz, info->log.data);
		memset(pkt, 0, sizeof(*pkt));
	}

	return;
}

static char *get_touch_button_string(u16 key_code)
{
	static char str[16] = {0};

	switch(key_code) {
		case KEY_BACK : /*158 0x9E*/
			sprintf(str, "BACK");
			break;
		case KEY_HOMEPAGE : /*172 0xAC*/
			sprintf(str, "HOME");
			break;
		case KEY_MENU : /* 139 0x8B*/
			sprintf(str, "MENU");
			break;
		case KEY_SIMSWITCH : /*249 0xF9*/
			sprintf(str, "SIM_SWITCH");
			break;
		default :
			sprintf(str, "Unknown");
			break;
	}

	return str;
}

static void mms_report_input_data(struct mms_ts_info *info, u8 sz, u8 *buf)
{
	int i;
	int id;
	int x;
	int y;
	int touch_major;
	int pressure;
	int key_code;
	int state;
	int key_index;
	int palm;
	u8 *tmp;
#ifdef SENSING_TEST
	char log_buf[256] = {0};
#endif

	if (buf[0] == MMS_NOTIFY_EVENT) {
		TOUCH_INFO_MSG("TSP mode changed (%d)\n", buf[1]);
		goto out;
	} 
	else if (buf[0] == MMS_ERROR_EVENT) {
		mms_clear_input_data(info);
		if (++info->ic_error_cnt > IC_ERROR_RETRY_CNT) {
			TOUCH_INFO_MSG("Error cannot recover \n");
			disable_irq_nosync(info->irq);
			TOUCH_INFO_MSG("disable_irq_nosync \n");
			mms_ts_power(info, 0);
			info->enabled = false;
			goto out;
		}

		TOUCH_INFO_MSG("Error or ESD detected. Retry(%d/%d)\n", info->ic_error_cnt, IC_ERROR_RETRY_CNT);
		mms_reboot(info);
		goto out;
	}

	for (i = 0; i < sz; i += FINGER_EVENT_SZ) {
		tmp = buf + i;

		if (tmp[0] & MMS_TOUCH_KEY_EVENT) {
			key_index = (tmp[0] & 0xf) - 1;
			if (key_index < 0 || key_index >= MAX_NUM_OF_KEY) {
				TOUCH_INFO_MSG("invalid key index (%d)\n", key_index);
				goto out;
			}
			key_code = info->pdata->key_code[key_index];
			if (!key_code) {
				TOUCH_INFO_MSG("key code not defined (%d)\n", key_index);
				goto out;
			}

			state = (tmp[0] & 0x80) ? 1 : 0;
			input_report_key(info->input_dev, key_code, state);
			if(state)
				TOUCH_INFO_MSG("KEY[%s:%3d] is pressed \n", get_touch_button_string((u16)key_code), key_code);
			else
				TOUCH_INFO_MSG("KEY[%s:%3d] is released \n", get_touch_button_string((u16)key_code), key_code);
		} else {
			id = (tmp[0] & 0xf) -1;
			x = tmp[2] | ((tmp[1] & 0xf) << 8);
			y = tmp[3] | (((tmp[1] >> 4 ) & 0xf) << 8);
			touch_major = tmp[4];
			pressure = tmp[5];

			input_mt_slot(info->input_dev, id);

			palm = (tmp[0]&0x10) ? 1 : 0;
			state = (tmp[0]&0x80) ? 1 : 0;

			if (palm) {
				if (state)
					TOUCH_INFO_MSG("Palm detected : %d \n", pressure);
				else
					TOUCH_INFO_MSG("Palm released : %d \n", pressure);

				mms_clear_input_data(info);
				continue;
			}

			if(id_state[id] != (u8)state)
			{
				id_state[id] = (u8)state;
				if(state == 1) {
				#ifdef ISIS_L2
					TOUCH_INFO_MSG("%d finger pressed : <%d> x[****] y[****] z[%3d] \n", ++touched_count, id, pressure);
				#else
					TOUCH_INFO_MSG("%d finger pressed : <%d> x[%3d] y[%3d] z[%3d] \n", ++touched_count, id, x, y, pressure);
				#endif
#ifdef SENSING_TEST
					if(sensing_test == 1) {
						sprintf(log_buf,"%3d %3d %3d %s\n",
							x, y, pressure,
							pressure > 0 ? "DOWN" : "UP");
						write_file(SENSING_TEST_PATH, log_buf, 1);
					}
#endif
				}
				else {
				#ifdef ISIS_L2
					TOUCH_INFO_MSG("touch_release[%s] : <%d> x[****] y[****] \n", palm?"Palm":" ", id);
				#else
					TOUCH_INFO_MSG("touch_release[%s] : <%d> x[%3d] y[%3d] \n", palm?"Palm":" ", id, x, y);
				#endif
#ifdef SENSING_TEST
					if(sensing_test == 1) {
						sprintf(log_buf,"%3d %3d %3d %s\n",
							x, y, pressure,
							pressure > 0 ? "DOWN" : "UP");
						write_file(SENSING_TEST_PATH, log_buf, 1);
					}
#endif
					touched_count--;
				}
			}
			
			if (!(tmp[0] & 0x80)) {
				input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, false);
				continue;
			}

			input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, true);
			input_report_abs(info->input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(info->input_dev, ABS_MT_POSITION_Y, y);
			input_report_abs(info->input_dev, ABS_MT_TOUCH_MAJOR, touch_major);
			input_report_abs(info->input_dev, ABS_MT_PRESSURE, pressure);
		}
	}

	input_sync(info->input_dev);

out:
	return;

}

static ssize_t mms_fs_read(struct file *fp, char *rbuf, size_t cnt, loff_t *fpos)
{
	struct mms_ts_info *info = fp->private_data;
	int ret = 0;

	switch (info->log.cmd) {
	case GET_RX_NUM:
		ret = copy_to_user(rbuf, &info->pdata->rx_num, 1);
		break;
	case GET_TX_NUM:
		ret = copy_to_user(rbuf, &info->pdata->tx_num, 1);
		break;
	case GET_EVENT_DATA:
		mms_event_handler(info);
		/* copy data without log marker */
		ret = copy_to_user(rbuf, info->log.data + 1, cnt);
		break;
	default:
		TOUCH_INFO_MSG("unknown command\n");
		ret = -EFAULT;
		break;
	}

	return ret;
}

static ssize_t mms_fs_write(struct file *fp, const char *wbuf, size_t cnt, loff_t *fpos)
{
	struct mms_ts_info *info = fp->private_data;
	struct i2c_client *client = info->client;
	u8 *buf;
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = cnt,
	};
	int ret = 0;

	mutex_lock(&info->lock);

	if (!info->enabled)
		goto tsp_disabled;
	
	msg.buf = buf = kzalloc(cnt + 1, GFP_KERNEL);

	if ((buf == NULL) || copy_from_user(buf, wbuf, cnt)) {
		TOUCH_INFO_MSG("failed to read data from user\n");
		ret = -EIO;
		goto out;
	}

	if (cnt == 1) {
		info->log.cmd = *buf;
	} else {
		if (i2c_transfer(client->adapter, &msg, 1) != 1) {
			TOUCH_INFO_MSG("failed to transfer data\n");
			ret = -EIO;
			goto out;
		}
	}

	ret = 0;

out:
	kfree(buf);
tsp_disabled:
	mutex_unlock(&info->lock);

	return ret;
}

static int mms_isc_read_status(struct mms_ts_info *info, u32 val)
{
	struct i2c_client *client = info->client;
	u8 cmd = ISC_CMD_READ_STATUS;
	u32 result = 0;
	int cnt = 100;
	int ret = 0;

	do {
		i2c_smbus_read_i2c_block_data(client, cmd, 4, (u8 *)&result);
		if (result == val)
			break;
		msleep(1);
	} while (--cnt);

	if (!cnt) {
		TOUCH_INFO_MSG("status read fail. cnt : %d, val : 0x%x != 0x%x\n",
			cnt, result, val);
	}

	return ret;
}

static int mms_isc_transfer_cmd(struct mms_ts_info *info, int cmd)
{
	struct i2c_client *client = info->client;
	struct isc_packet pkt = { ISC_ADDR, cmd };
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = sizeof(struct isc_packet),
		.buf = (u8 *)&pkt,
	};

	return (i2c_transfer(client->adapter, &msg, 1) != 1);
}

static int mms_isc_erase_page(struct mms_ts_info *info, int page)
{
	return mms_isc_transfer_cmd(info, ISC_CMD_PAGE_ERASE | page) ||
		mms_isc_read_status(info, ISC_PAGE_ERASE_DONE | ISC_PAGE_ERASE_ENTER | page);
}

static int mms_isc_enter(struct mms_ts_info *info)
{
	return i2c_smbus_write_byte_data(info->client, MMS_CMD_ENTER_ISC, true);
}

static int mms_isc_exit(struct mms_ts_info *info)
{
	return mms_isc_transfer_cmd(info, ISC_CMD_EXIT);
}

static int mms_flash_section(struct mms_ts_info *info, struct mms_fw_img *img, const u8 *data)
{
	struct i2c_client *client = info->client;
	struct isc_packet *isc_packet;
	int ret;
	int page, i;
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = ISC_XFER_LEN,
		},
	};
	int ptr = img->offset;

	isc_packet = kzalloc(sizeof(*isc_packet) + ISC_XFER_LEN, GFP_KERNEL);

	if(!isc_packet)
		return -ENOMEM;
		
	isc_packet->cmd = ISC_ADDR;

	msg[0].buf = (u8 *)isc_packet;
	msg[1].buf = kzalloc(ISC_XFER_LEN, GFP_KERNEL);

	for (page = img->start_page; page <= img->end_page; page++) {
		mms_isc_erase_page(info, page);

		for (i = 0; i < ISC_BLOCK_NUM; i++, ptr += ISC_XFER_LEN) {
			/* flash firmware */
			u32 tmp = page * 256 + i * (ISC_XFER_LEN / 4);
			put_unaligned_le32(tmp, &isc_packet->addr);
			msg[0].len = sizeof(struct isc_packet) + ISC_XFER_LEN;

			memcpy(isc_packet->data, data + ptr, ISC_XFER_LEN);
			if (i2c_transfer(client->adapter, msg, 1) != 1)
				goto i2c_err;

			/* verify firmware */
			tmp |= ISC_CMD_READ;
			put_unaligned_le32(tmp, &isc_packet->addr);
			msg[0].len = sizeof(struct isc_packet);

			if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) != ARRAY_SIZE(msg))
				goto i2c_err;
#ifdef MMS_TS_FW_VERIFIY
			if (memcmp(isc_packet->data, msg[1].buf, ISC_XFER_LEN)) {
#if FLASH_VERBOSE_DEBUG
				print_hex_dump(KERN_ERR, "mms fw wr : ",
						DUMP_PREFIX_OFFSET, 16, 1,
						isc_packet->data, ISC_XFER_LEN, false);

				print_hex_dump(KERN_ERR, "mms fw rd : ",
						DUMP_PREFIX_OFFSET, 16, 1,
						msg[1].buf, ISC_XFER_LEN, false);
#endif
				TOUCH_INFO_MSG("flash verify failed\n");
				ret = -1;
				goto out;
			}
#endif
		}
	}

	TOUCH_INFO_MSG("section [%d] update succeeded\n", img->type);

	ret = 0;
	goto out;

i2c_err:
	TOUCH_INFO_MSG("i2c failed @ %s\n", __func__);
	ret = -1;

out:
	kfree(isc_packet);
	kfree(msg[1].buf);

	return ret;
}

static int mms_ts_i2c_read(struct i2c_client *client, u8 reg, char *buf, int len)
{
	int ret = 0;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};

	ret = i2c_transfer(client->adapter, msgs, 2);

	if (ret < 0) {
		if (printk_ratelimit())
			TOUCH_INFO_MSG("transfer error: %d\n", ret);
		return -EIO;
	} else
		return 0;
}

void mms_edge_expand_write(struct mms_ts_info *ts)
{
	int i = 0;
	uint8_t write_buf[5] = {0};
	struct i2c_msg msg[] = {
		{
			.addr = ts->client->addr,
			.flags = 0,
			.buf = write_buf,
		},
		{
			.addr = ts->client->addr,
			.flags = 1,
		},
	};

	for (i = 0; i < 4; i++) {
		if (edge_expand[i] == 0 )
			return;
	}

	msg[0].len = 5;
	write_buf[0] = MMS_SET_EDGE_EXPAND;
	write_buf[1] = edge_expand[0]; //top
	write_buf[2] = edge_expand[1]; //bottom
	write_buf[3] = edge_expand[2]; //left
	write_buf[4] = edge_expand[3]; //right

	if (i2c_transfer(ts->client->adapter, &msg[0], 1) != 1) {
		TOUCH_INFO_MSG("mms_edge_expand_write i2c transfer failed\n");
	}

	TOUCH_INFO_MSG("Write Edge Expand : %d, %d, %d, %d \n",
		edge_expand[0], edge_expand[1], edge_expand[2], edge_expand[3]);
}

void mms_edge_expand_read(struct mms_ts_info *ts)
{
	int i = 0;
	uint8_t read_buf[16] = {0};
	uint8_t write_buf[4] = {0};
	uint8_t edge_tmp[4] = {0};
	struct i2c_msg msg[] = {
		{
			.addr = ts->client->addr,
			.flags = 0,
			.buf = write_buf,
		},{
			.addr = ts->client->addr,
			.flags = 1,
		},
	};

	for ( i = 0; i < 4; i++) {
		write_buf[0] = MMS_UNIVERSAL_CMD;
		write_buf[1] = UNIVCMD_GET_TOP_EDGE_EXPAND + i;
		msg[0].len = 2;

		if (i2c_transfer(ts->client->adapter, &msg[0], 1) != 1) {
			TOUCH_INFO_MSG("%s : i2c transfer failed\n", __func__);
			return;
		}

		if (mms_ts_i2c_read(ts->client, MMS_CMD_RESULT_SIZE, read_buf, 1) < 0) {
			TOUCH_INFO_MSG("%s : Fail to get MMS_CMD_RESULT_SIZE \n", __func__);
			return;
		}

		if (mms_ts_i2c_read(ts->client, MMS_UNIVERSAL_RESULT, read_buf, 1) < 0) {
			TOUCH_INFO_MSG("%s : Fail to get MMS_UNIVERSAL_RESULT \n", __func__);
			return;
		}

		if (read_buf[0] == 0)
			return;

		edge_tmp[i] = read_buf[0];
	}

	for ( i = 0; i < 4; i++)
		edge_expand[i] = edge_tmp[i];

	TOUCH_INFO_MSG("Read Edge Expand : %d, %d, %d, %d \n",
		edge_expand[0], edge_expand[1], edge_expand[2], edge_expand[3]);

	return;
}

static int get_fw_version(struct i2c_client *client, u8 *buf)
{
	u8 cmd = MMS_FW_VERSION;
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = &cmd,
			.len = 1,
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = buf,
			.len = MAX_SECTION_NUM,
		},
	};

	return (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) != ARRAY_SIZE(msg));
}

static int get_channel_num(struct i2c_client *client, u8 *buf)
{
	u8 cmd = MMS_TX_NUM;
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = &cmd,
			.len = 1,
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = buf,
			.len = 3,
		},
	};

	return (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) != ARRAY_SIZE(msg));
}

static int get_fw_product(struct i2c_client *client, u8 *buf)
{
	u8 cmd = MMS_FW_PRODUCT;
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = &cmd,
			.len = 1,
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = buf,
			.len = 8,
		},
	};

	return (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) != ARRAY_SIZE(msg));
}

static void mms_get_dev_info(struct mms_ts_info *info)
{
	u8 ver[16] = {0};

	get_fw_version(info->client, ver);
	TOUCH_INFO_MSG("Firmware Version : %d.%02d \n", (ver[2]&0x80?1:0), ver[2]&0x7F);
	TOUCH_INFO_MSG("Boot:0x%X  Core:0x%X  Config:0x%X \n", ver[0], ver[1], ver[2]);

	memset(ver, 0x0, 16);
	get_fw_product(info->client, ver);
	TOUCH_INFO_MSG("FW Product : %s \n", ver);

	memset(ver, 0x0, 16);
	get_channel_num(info->client, ver);
	info->pdata->tx_num = ver[0];
	info->pdata->rx_num = ver[1];
	info->pdata->key_num = ver[2];
	TOUCH_INFO_MSG("Num of Channel. TX:%d RX:%d KEY:%d\n", info->pdata->tx_num, info->pdata->rx_num, info->pdata->key_num);
}

static int mms_flash_fw(const struct firmware *fw, struct mms_ts_info *info)
{
	int ret;
	int i;
	struct mms_bin_hdr *fw_hdr;
	struct mms_fw_img **img;
	struct i2c_client *client = info->client;
	u8 ver[MAX_SECTION_NUM];
	u8 target[MAX_SECTION_NUM];
	int offset = sizeof(struct mms_bin_hdr);
	int retires = 3;
	bool update_flag = false;
	bool isc_flag = true;
	char name[16] = {0};
	fw_hdr = (struct mms_bin_hdr *)fw->data;

	img = kzalloc(sizeof(*img) * fw_hdr->section_num, GFP_KERNEL);

	while (retires--) {
		if (!get_fw_version(client, ver)) {
			break;
		}
		else {
			TOUCH_INFO_MSG("mms_flash_fw error \n");
			mms_reboot(info);
		}
	}

	if(info->pdata->force_fw_upgrade)
	{
		TOUCH_INFO_MSG("Force update \n");
		ver[2] = 0xFF;
	}
	else
	{
		get_fw_product(client, name);
		if(strcmp(name, info->pdata->product) != 0){
			TOUCH_INFO_MSG("Model name is not match [%s]\n", name);
			ver[2] = 0xFF;
		}
	}

	if (retires < 0) {
		TOUCH_INFO_MSG("failed to obtain ver. info\n");
		isc_flag = false;
		memset(ver, 0xff, sizeof(ver));
		
		if(info->pdata->use_isp_erase) {
			mms_isp_erase(client, info->pdata);
			mms_reboot(info);
		}
	} else {
		print_hex_dump(KERN_INFO, "[Touch] mms_ts fw ver : ", DUMP_PREFIX_NONE, 16, 1,
				ver, MAX_SECTION_NUM, false);
	}

	for (i = 0; i < fw_hdr->section_num; i++, offset += sizeof(struct mms_fw_img)) {
		img[i] = (struct mms_fw_img *)(fw->data + offset);
		target[i] = img[i]->version;

		if (ver[img[i]->type] != target[i]) {
			if(isc_flag){
				mms_isc_enter(info);
				isc_flag = false;
			}
			update_flag = true;
			TOUCH_INFO_MSG("section [%d] is need to be updated. ver : 0x%x, bin : 0x%x\n",
				img[i]->type, ver[img[i]->type], target[i]);

			if ((ret = mms_flash_section(info, img[i], fw->data + fw_hdr->binary_offset))) {
				TOUCH_INFO_MSG("mms_flash_fw end \n");
				mms_reboot(info);
				goto out;
			}
			memset(ver, 0xff, sizeof(ver));
		} else {
			TOUCH_INFO_MSG("section [%d] is up to date\n", i);
		}
	}

	if (update_flag){
		mms_isc_exit(info);
		msleep(5);
		mms_reboot(info);
		TOUCH_INFO_MSG("mms_flash_fw isc exit \n");
	}

	if (get_fw_version(client, ver)) {
		TOUCH_INFO_MSG("failed to obtain version after flash\n");
		ret = -1;
		goto out;
	} else {
		for (i = 0; i < fw_hdr->section_num; i++) {
			if (ver[img[i]->type] != target[i]) {
				TOUCH_INFO_MSG("version mismatch after flash. [%d] 0x%x != 0x%x\n",
					i, ver[img[i]->type], target[i]);

				ret = -1;
				goto out;
			}
		}
	}

	ret = 0;

out:
	kfree(img);
	if (info->pdata->force_fw_upgrade) {
		/* skip */
	} else {
		mms_ts_config(info);
	}

	return ret;
}

static void mms_fw_update_controller(const struct firmware *fw, void * context)
{
	struct mms_ts_info *info = context;
	int retires = 3;
	int ret;

	if (!fw) {
		TOUCH_INFO_MSG("failed to read firmware\n");
		mms_ts_config(info);
		enable_irq(info->irq);
		return;
	}

	power_lock(POWER_FW_UP_LOCK);

	disable_irq_nosync(info->irq);
	do {
		ret = mms_flash_fw(fw, info);
		if(ret != 0 && info->pdata->use_isp_erase) {
			mms_isp_erase(info->client,info->pdata);
			mms_reboot(info);
		}
	} while (ret && --retires);
	enable_irq(info->irq);

	power_unlock(POWER_FW_UP_LOCK);

	if (!retires) {
		if(info->pdata->use_isp_erase) {
			mms_isp_erase(info->client,info->pdata);
			mms_reboot(info);
		}
		ret = mms_flash_fw(fw, info);
		if(ret)
			TOUCH_INFO_MSG("failed to flash firmware after retires\n");
	} else {
		mms_get_dev_info(info);
	}

	if (fw) {
		TOUCH_INFO_MSG("release_firmware \n");
		release_firmware(fw);
	}

	if(info->pdata->force_fw_upgrade)
		info->pdata->force_fw_upgrade = 0;
}

static irqreturn_t mms_ts_interrupt(int irq, void *dev_id)
{
	struct mms_ts_info *info = dev_id;
	struct i2c_client *client = info->client;
	u8 buf[(MAX_FINGER_NUM + MAX_BUTTON_NUM) * FINGER_EVENT_SZ] = { 0, };
	int ret;
	int sz;
	u8 reg = MMS_INPUT_EVENT;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = &reg,
			.len = 1,
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = buf,
		},
	};

	if (!info->enabled){
		return IRQ_HANDLED;
	}

	sz = i2c_smbus_read_byte_data(client, MMS_EVENT_PKT_SZ);

	if (sz > (MAX_FINGER_NUM + MAX_BUTTON_NUM) * FINGER_EVENT_SZ) {
		TOUCH_INFO_MSG("sz buffer size exceeded. sz = %d \n", sz);
		sz = (MAX_FINGER_NUM + MAX_BUTTON_NUM) * FINGER_EVENT_SZ;
	}

        if(sz < 0)
        {
            TOUCH_INFO_MSG("size is under zero sz = %d \n", sz);
            mms_clear_input_data(info);
            mms_reboot(info);
            return IRQ_HANDLED;
        }

	msg[1].len = sz;

	ret = i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg));

	if (ret != ARRAY_SIZE(msg)) {
		TOUCH_INFO_MSG("failed to read %d bytes of touch data (%d)\n",
			sz, ret);
	} else {
		mms_report_input_data(info, sz, buf);
	}

	return IRQ_HANDLED;
}

static int mms_ts_input_open(struct input_dev *dev)
{
	struct mms_ts_info *info = input_get_drvdata(dev);

	mms_ts_enable(info);

	return 0;
}

static void mms_ts_input_close(struct input_dev *dev)
{
	struct mms_ts_info *info = input_get_drvdata(dev);

	mms_ts_disable(info);
}

static int mms_ts_config(struct mms_ts_info *info)
{
	struct i2c_client *client = info->client;
	int ret;

	ret = request_threaded_irq(client->irq, NULL, mms_ts_interrupt,
				IRQF_TRIGGER_LOW | IRQF_ONESHOT, "mms_ts", info);
	if (ret) {
		TOUCH_INFO_MSG("failed to register irq\n");
		goto out;
	}

	disable_irq(client->irq);
	info->irq = client->irq;
	barrier();

	complete_all(&info->init_done);

out:
	return ret;
}

static ssize_t bin_sysfs_read(struct file *fp, struct kobject *kobj , struct bin_attribute *attr,
                          char *buf, loff_t off,size_t count)
{
        struct device *dev = container_of(kobj, struct device, kobj);
        struct i2c_client *client = to_i2c_client(dev);
        struct mms_ts_info *info = i2c_get_clientdata(client);
        struct i2c_msg msg;
        info->client = client;

        msg.addr = client->addr;
        msg.flags = I2C_M_RD ;
        msg.buf = (u8 *)buf;
        msg.len = count;

	switch (count)
	{
		case 65535:
			mms_reboot(info);
			TOUCH_INFO_MSG("read mms_reboot\n");
			return 0;

		default :
			if(i2c_transfer(client->adapter, &msg, 1) != 1){
				TOUCH_INFO_MSG("failed to transfer data\n");
        	        	mms_reboot(info);
        	        	return 0;
        		}
			break;

	}

        return count;
}

static ssize_t bin_sysfs_write(struct file *fp, struct kobject *kobj, struct bin_attribute *attr,
                                char *buf, loff_t off, size_t count)
{
        struct device *dev = container_of(kobj, struct device, kobj);
        struct i2c_client *client = to_i2c_client(dev);
        struct mms_ts_info *info = i2c_get_clientdata(client);
        struct i2c_msg msg;

        msg.addr =client->addr;
        msg.flags = 0;
        msg.buf = (u8 *)buf;
        msg.len = count;

        if(i2c_transfer(client->adapter, &msg, 1) != 1){
                TOUCH_INFO_MSG("failed to transfer data\n");
                mms_reboot(info);
                return 0;
        }

        return count;
}

static struct bin_attribute bin_attr = {
        .attr = {
                .name = "mms_bin",
#if !defined(CONFIG_MACH_MSM8X10_W5_AIO_US)
                .mode = S_IRWXUGO,
#else
		.mode = (S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH),
#endif
        },
        .size = MMS_FLASH_PAGE_SZ,
        .read = bin_sysfs_read,
        .write = bin_sysfs_write,
};

static struct file_operations mms_fops = {
	.owner		= THIS_MODULE,
	.open		= mms_fs_open,
	.release	= mms_fs_release,
	.read		= mms_fs_read,
	.write		= mms_fs_write,
};

#ifdef CONFIG_OF
static int mms_ts_parse_dt(struct device *dev, struct mms_ts_platform_data *data)
{
	struct device_node *node = dev->of_node;
	struct property *prop;
	int rc = 0;
	int i;

	rc = of_property_read_u32(node, "melfas,auto_fw_update", &data->auto_fw_update);
	if (rc) {
		TOUCH_INFO_MSG("DT : failed to read auto_fw_update\n");
		data->auto_fw_update = 0;
	}
	
	rc = of_property_read_u32(node, "melfas,use_isp_erase", &data->use_isp_erase);
	if (rc) {
		TOUCH_INFO_MSG("DT : failed to read use_isp_erase\n");
		data->use_isp_erase = 0;
	}

	rc = of_property_read_u32(node, "melfas,use_vdd", &data->use_vdd);
	if (rc) {
		TOUCH_INFO_MSG("DT : failed to read use_vdd\n");
		data->use_vdd = 0;
	}

	if (data->use_vdd) {
		rc = of_property_read_u32(node, "melfas,vdd_voltage", &data->vdd_voltage);
		if (rc) {
			TOUCH_INFO_MSG("DT : failed to read vdd_voltage\n");
			data->vdd_voltage = 0;
		}
	}

	rc = of_property_read_u32(node, "melfas,use_vdd_i2c", &data->use_vdd_i2c);
	if (rc) {
		TOUCH_INFO_MSG("DT : failed to read use_vdd_i2c\n");
		data->use_vdd_i2c = 0;
	}

	rc = of_property_read_u32(node, "melfas,use_vdd_int", &data->use_vdd_int);
	if (rc) {
		TOUCH_INFO_MSG("DT : failed to read use_vdd_int\n");
		data->use_vdd_int = 0;
	}

	rc = of_property_read_u32(node, "melfas,max-x", &data->max_x);
	if (rc) {
		TOUCH_INFO_MSG("DT : failed to read max-x\n");
	}
	rc = of_property_read_u32(node, "melfas,max-y", &data->max_y);
	if (rc) {
		TOUCH_INFO_MSG("DT : failed to read max-y\n");
	}
	
	rc = of_property_read_u32(node, "melfas,gpio-vdd-en", &data->gpio_vdd_en);
	if (rc) {
		data->gpio_vdd_en = -1;
		TOUCH_INFO_MSG("DT : failed to read gpio-vdd-en\n");
	}

	rc = of_property_read_u32(node, "melfas,gpio-int", &data->gpio_int);
	if (rc) {
		data->gpio_int = -1;
		TOUCH_INFO_MSG("DT : failed to read gpio-int\n");
	}

	for(i = 0; i < MAX_NUM_OF_KEY; i++) {
		data->key_code[i] = 0;
	}
	prop = of_find_property(node, "melfas,key-map", NULL);
	if (prop) {
		int keys;
		
		keys = prop->length / sizeof(unsigned int);
		if (keys < MAX_NUM_OF_KEY) {
			rc = of_property_read_u32_array(node, "melfas,key-map", data->key_code, keys);
			if (rc) {
				TOUCH_INFO_MSG("DT : failed to read key-map\n");
			}
		}
	} else {
		TOUCH_INFO_MSG("DT : failed to find key-map in device tree\n");
	}
	
	rc = of_property_read_string(node, "melfas,fw-image-name",  &data->fw_name);
	if (rc && (rc != -EINVAL)) {
		TOUCH_INFO_MSG("DT : fw_name error \n");
	}

	rc = of_property_read_string(node, "melfas,panel-spec-name",  &data->panel_spec_name);
	if (rc && (rc != -EINVAL)) {
		TOUCH_INFO_MSG("DT : panel-spec-name error \n");
	}
	else
		TOUCH_INFO_MSG("DT : panel-spec-name : %s \n", data->panel_spec_name);

	rc = of_property_read_string(node, "melfas,product",  &data->product);
	if (rc && (rc != -EINVAL)) {
		TOUCH_INFO_MSG("DT : product error \n");
	}
	
	TOUCH_INFO_MSG("DT : auto_fw_update : %d \n", data->auto_fw_update);
	TOUCH_INFO_MSG("DT : use_isp_erase : %d \n", data->use_isp_erase);
	TOUCH_INFO_MSG("DT : gpio-int : %d \n", data->gpio_int);
	TOUCH_INFO_MSG("DT : max-x : %d \n", data->max_x);
	TOUCH_INFO_MSG("DT : max-y : %d \n", data->max_y);
	TOUCH_INFO_MSG("DT : use_vdd : %d \n", data->use_vdd);
	if (data->use_vdd) {
		TOUCH_INFO_MSG("DT : vdd_voltage : %d \n", data->vdd_voltage);
	}
	TOUCH_INFO_MSG("DT : use_vdd_i2c : %d \n", data->use_vdd_i2c);
	TOUCH_INFO_MSG("DT : use_vdd_int : %d \n", data->use_vdd_int);
	TOUCH_INFO_MSG("DT : gpio-vdd-en : %d \n", data->gpio_vdd_en);

	if(data->fw_name)
		TOUCH_INFO_MSG("DT : fw_name = %s\n", data->fw_name);

	if(data->product)
		TOUCH_INFO_MSG("DT : product = %s\n", data->product);

	return rc;
}
#endif

static ssize_t
mms_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mms_ts_info *ts = dev_get_drvdata(dev);
	int len = 0;

	len = snprintf(buf, PAGE_SIZE, "\nMMS-128 Device Status\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "=============================\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "irq num       is %d\n", ts->irq);
	len += snprintf(buf + len, PAGE_SIZE - len, "gpio_irq num  is %d(level=%d)\n", ts->pdata->gpio_int, gpio_get_value(ts->pdata->gpio_int));
	len += snprintf(buf + len, PAGE_SIZE - len, "gpio_scl num  is %d\n", ts->pdata->gpio_scl);
	len += snprintf(buf + len, PAGE_SIZE - len, "gpio_sda num  is %d\n", ts->pdata->gpio_sda);
	len += snprintf(buf + len, PAGE_SIZE - len, "=============================\n");
	return len;
}

static ssize_t
mms_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mms_ts_info *info = dev_get_drvdata(dev);
	int len = 0;
	u8 ver[16] = {0};

	power_lock(POWER_SYSFS_LOCK);
	mms_ts_enable(info);

	mms_get_dev_info(info);

	len += snprintf(buf + len, PAGE_SIZE - len, "\n=============================\n");

	get_fw_version(info->client, ver);
	len += snprintf(buf+len, PAGE_SIZE, "Firmware Version : %d.%02d \n", (ver[2]&0x80?1:0), ver[2]&0x7F);
	len += snprintf(buf+len, PAGE_SIZE, "Boot:0x%X  Core:0x%X  Config:0x%X \n", ver[0], ver[1], ver[2]);
	memset(ver, 0x0, 16);
	get_fw_product(info->client, ver);
	len += snprintf(buf+len, PAGE_SIZE, "FW Product : %s \n", ver);
	memset(ver, 0x0, 16);
	get_channel_num(info->client, ver);
	len += snprintf(buf+len, PAGE_SIZE, "Num of Channel. TX:%d RX:%d KEY:%d\n", ver[0], ver[1], ver[2]);
	len += snprintf(buf + len, PAGE_SIZE - len, "=============================\n");

	power_unlock(POWER_SYSFS_LOCK);

	return len;
}

static ssize_t
mms_testmode_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mms_ts_info *info = dev_get_drvdata(dev);
	int len = 0;
	u8 ver[16] = {0};
	u8 product[16] = {0};

	power_lock(POWER_SYSFS_LOCK);
	mms_ts_enable(info);

	mms_get_dev_info(info);

	get_fw_version(info->client, ver);
	get_fw_product(info->client, product);

	len += snprintf(buf+len, PAGE_SIZE, "%d.%02d(0x%X, 0x%X, 0x%X, %s)\n",
					(ver[2]&0x80?1:0), ver[2]&0x7F, ver[0], ver[1], ver[2], product);

	power_unlock(POWER_SYSFS_LOCK);

	return len;
}


static ssize_t
mms_power_control_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct mms_ts_info *info = dev_get_drvdata(dev);
	int cmd;

	if (sscanf(buf, "%d", &cmd) != 1)
		return -EINVAL;
 
	switch (cmd) {
	case 0: /*touch power off */
		mms_ts_disable(info);
		break;
	case 1: /* touch power on */
		mms_ts_enable(info);
		msleep(30);
		break;
	case 2: /* touch power reset */
		mms_reboot(info);
		break;
	default:
		TOUCH_INFO_MSG("usage: echo [0|1|2] > control\n");
		TOUCH_INFO_MSG("	- 0: power off\n");
		TOUCH_INFO_MSG("	- 1: power on\n");
		TOUCH_INFO_MSG("	- 2: power reset\n");
		break;
	}
	return count;
}

#ifdef SENSING_TEST
static ssize_t
mms136_sensing_test_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int cmd = 2;

	if(sscanf(buf, "%d", &cmd) != 1)
		return -EINVAL;

	switch (cmd) {
	case 0:
		TOUCH_INFO_MSG("Stop Writing\n");
		sensing_test = 0;
		break;
	case 1:
		TOUCH_INFO_MSG("Start Writing \n");
		sensing_test = 1;
		break;
	default:
		TOUCH_INFO_MSG("usage: echo [0(Stop Writing)|1(Start Writing)] > sensing_test\n");
		TOUCH_INFO_MSG("  - Start Writing & Stop Writing \n");
		break;
	}
	return count;
}
#endif

static ssize_t
mms_irq_control_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct mms_ts_info *ts = dev_get_drvdata(dev);
	int cmd = 0, ret = 0;

	if (sscanf(buf, "%d", &cmd) != 1)
		return -EINVAL;

	switch (cmd) {
	case 0: /* interrupt pin LOW */
		ret = gpio_direction_input(ts->pdata->gpio_int);
		if (ret < 0) {
			TOUCH_INFO_MSG("%s: gpio input direction fail\n", __FUNCTION__);
			break;
		}
		gpio_set_value(ts->pdata->gpio_int, 0);
		TOUCH_INFO_MSG("INTR GPIO pin low\n");
		break;
	case 1: /* interrupt pin high */
		ret = gpio_direction_input(ts->pdata->gpio_int);
		if (ret < 0) {
			TOUCH_INFO_MSG("%s: gpio input direction fail\n", __FUNCTION__);
			break;
		}
		gpio_set_value(ts->pdata->gpio_int, 1);
		TOUCH_INFO_MSG("INTR GPIO pin high\n");
		break;
	default:
		TOUCH_INFO_MSG("usage: echo [0|1] > control\n");
		TOUCH_INFO_MSG("  - 0: interrupt pin low\n");
		TOUCH_INFO_MSG("  - 1: interrupt pin high\n");
		break;
	}
	return count;
}

static ssize_t
mms_reg_control_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct mms_ts_info *ts = dev_get_drvdata(dev);
	int cmd = 0, ret = 0, reg_addr = 0, length = 0, i = 0;
	uint8_t reg_buf[100] = {0};

	if (sscanf(buf, "%d, 0x%x, %d", &cmd, &reg_addr, &length) != 3)
		return -EINVAL;

	mms_ts_enable(ts);

	switch (cmd) {
	case 1:
		reg_buf[0] = reg_addr;
		ret = i2c_master_send(ts->client, reg_buf, 1);
		if (ret < 0) {
			TOUCH_INFO_MSG("i2c master send fail\n");
			break;
		}
		ret = i2c_master_recv(ts->client, reg_buf, length);
		if (ret < 0) {
			TOUCH_INFO_MSG("i2c master recv fail\n");
			break;
		}
		for (i = 0; i < length; i++) {
			TOUCH_INFO_MSG("0x%x", reg_buf[i]);
		}
		TOUCH_INFO_MSG("\n 0x%x register read done\n", reg_addr);
		break;
	case 2:
		reg_buf[0] = reg_addr;
		reg_buf[1] = length;
		ret = i2c_master_send(ts->client, reg_buf, 2);
		if (ret < 0) {
			TOUCH_INFO_MSG("i2c master send fail\n");
			break;
		}
		TOUCH_INFO_MSG("\n 0x%x register write done\n", reg_addr);
		break;
	default:
		TOUCH_INFO_MSG("usage: echo [1(read)|2(write)], [reg address], [length|value] > reg_control\n");
		TOUCH_INFO_MSG("  - Register Set or Read\n");
		break;
	}
	return count;
}

static ssize_t
mms_fw_upgrade(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct mms_ts_info *info = dev_get_drvdata(dev);
	int cmd;
	char buff[128] = {0};
	u8  tmp[5]= {0xD5,0x00,0xC1,0x00,0x00};

	if (sscanf(buf, "%d", &cmd) != 1)
		return -EINVAL;

	TOUCH_INFO_MSG("force_fw_upgrade: %d \n", info->pdata->force_fw_upgrade);

	if(info->pdata->force_fw_upgrade)
		return -EINVAL;

	mms_ts_enable(info);

	snprintf(buff, sizeof(buff), "%s \n", "mms_fw_upgrade");

	info->pdata->force_fw_upgrade = 1;
	
	switch (cmd) {
	case 0:
		if(CORE32_FW_NAME != NULL) {
			TOUCH_INFO_MSG("%s \n", CORE32_FW_NAME);
			request_firmware_nowait(THIS_MODULE, true, CORE32_FW_NAME, &info->client->dev,
				GFP_KERNEL, info, mms_fw_update_controller);
		}
		break;

	case 1:
		if(info->pdata->fw_name != NULL) {
			TOUCH_INFO_MSG("%s \n", info->pdata->fw_name);
			request_firmware_nowait(THIS_MODULE, true, info->pdata->fw_name, &info->client->dev,
				GFP_KERNEL, info, mms_fw_update_controller);
		}
		break;

	case 9:
		if(EXTERNAL_FW_NAME != NULL) {
			TOUCH_INFO_MSG(" /etc/firmware/%s \n", EXTERNAL_FW_NAME);
			request_firmware_nowait(THIS_MODULE, true, EXTERNAL_FW_NAME, &info->client->dev,
				GFP_KERNEL, info, mms_fw_update_controller);
		}
		break;

	case 99:
		TOUCH_INFO_MSG(" Flash Erase \n");
		i2c_master_send(info->client, tmp, 5);
		break;
		
	default:
		TOUCH_INFO_MSG(" usage: echo [0|1|9] > fw_upgrade\n");
		TOUCH_INFO_MSG(" -0 : Default Core32 v01 \n");
		TOUCH_INFO_MSG(" -1 : From Kernel binary \n");
		TOUCH_INFO_MSG(" -9 : From external file \n");
		break;
	}

	TOUCH_INFO_MSG("%s: %s\n", __func__, buff);

	return count;
}

static ssize_t
mms_chstatus_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mms_ts_info *ts = dev_get_drvdata(dev);
	char c = '-';
	int len = 0, i = 0, j = 0, t = 0;
	uint8_t write_buf[5] = {0};
	uint8_t read_buf[80] = {0};
	uint8_t read_size = 0;
	uint16_t data = 0;
	int *error_point = NULL;
	int flag = 0;
	int *chstatus_max = NULL;
	int *chstatus_min = NULL;
	int ret = 0, count = 0;
	int alloc_flag = 0;

	error_point = kzalloc(sizeof(int) * (ts->pdata->rx_num * ts->pdata->tx_num + ts->pdata->key_num), GFP_KERNEL);
	if (error_point == NULL) {
		TOUCH_INFO_MSG("failed to allocate error_point\n");
		alloc_flag = -1;
	}

	chstatus_max = kzalloc(sizeof(int) * (ts->pdata->rx_num * ts->pdata->tx_num + ts->pdata->key_num), GFP_KERNEL);
	if (chstatus_max == NULL) {
		TOUCH_INFO_MSG("failed to allocate chstatus_max\n");
		alloc_flag = -1;
	}

	chstatus_min = kzalloc(sizeof(int) * (ts->pdata->rx_num * ts->pdata->tx_num + ts->pdata->key_num), GFP_KERNEL);
	if (chstatus_min == NULL) {
		TOUCH_INFO_MSG("failed to allocate chstatus_min\n");
		alloc_flag = -1;
	}

	power_lock(POWER_SYSFS_LOCK);

	mms_ts_enable(ts);
	ts->pdata->self_diagnostic[0] = 1;

	if (alloc_flag != -1) {
		ret = get_limit(ts, "chstatus_max", chstatus_max, NULL);
		if ((ret != -1) && (get_limit(ts, "chstatus_min", chstatus_min, NULL) == -1))
			ret = -1;
	}
	if (irq_flag == 0) {
		TOUCH_INFO_MSG("disable_irq_nosync\n");
		disable_irq_nosync(ts->client->irq);
		irq_flag = 1;
	}

	write_buf[0] = MMS_UNIVERSAL_CMD;
	write_buf[1] = MMS_TOUCH_KEY_EVENT;
	i2c_master_send(ts->client, write_buf, 2);

	do{
		while (gpio_get_value(ts->pdata->gpio_int)) {
			flag++;
			if (flag == 30) {
				flag = 0;
				break;
			}
			msleep(100);
		}

		write_buf[0] = 0x0F;
		i2c_master_send(ts->client, write_buf, 1);
		i2c_master_recv(ts->client, read_buf, 1);


		write_buf[0] = 0x10;
		i2c_master_send(ts->client, write_buf, 1);
		i2c_master_recv(ts->client, read_buf, 1);
		TOUCH_INFO_MSG("Maker is %x\n", read_buf[0]);
		count++;
	}while(read_buf[0]!=0x0C&&count!=10);

	write_buf[0] = MMS_UNIVERSAL_CMD;
	write_buf[1] = MMS_TEST_CH_STATUS;
	i2c_master_send(ts->client, write_buf, 2);

	while (gpio_get_value(ts->pdata->gpio_int)) {
		flag++;
		if (flag == 30) {
			flag = 0;
			break;
		}
		msleep(100);
	}
	flag = 0;

	write_buf[0] = MMS_CMD_RESULT_SIZE;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, &read_size, 1);

	write_buf[0] = MMS_UNIVERSAL_RESULT;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, read_buf, read_size);
	TOUCH_INFO_MSG("CHANNEL STATUS TEST =%d \n", read_buf[0]);

	len = snprintf(buf, PAGE_SIZE, "\n<< SHOW CHANNEL STATUS >>\n");
	if (ts->pannel_on == 0) {
		len += snprintf(buf + len, PAGE_SIZE - len, "************************\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "*****    LCD OFF   *****\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "************************\n");
	} else if (ts->pannel_on == 1) {
		len += snprintf(buf + len, PAGE_SIZE - len, "************************\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "*****    LCD ON    *****\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "************************\n");
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "%5c", c);
	for (j = 0; j < ts->pdata->tx_num; j++)
		len += snprintf(buf + len, PAGE_SIZE - len, "%5d", j);
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------");
	len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------");
	len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------\n");

	/* read touch screen chstatus */
	for (i = 0; i < ts->pdata->rx_num ; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len, "%2d : ", i);
			write_buf[0] = MMS_UNIVERSAL_CMD;
			write_buf[1] = MMS_GET_PIXEL_CH_STATUS;
			write_buf[2] = 0xFF;
			write_buf[3] = i;
			i2c_master_send(ts->client, write_buf, 4);

			while (gpio_get_value(ts->pdata->gpio_int)) {
				flag++;
				if (flag == 100) {
					flag = 0;
					break;
				}
				udelay(100);
			}

			write_buf[0] = MMS_CMD_RESULT_SIZE;
			i2c_master_send(ts->client, write_buf, 1);
			i2c_master_recv(ts->client, &read_size, 1);

			write_buf[0] = MMS_UNIVERSAL_RESULT;
			i2c_master_send(ts->client, write_buf, 1);
			i2c_master_recv(ts->client, read_buf, read_size);

			for (j = 0; j < ts->pdata->tx_num; j++) {
				data = read_buf[2 * j] | (read_buf[2 * j + 1] << 8);
				if ((alloc_flag != -1) && (ret != -1)) {
					if ((data > chstatus_max[i * ts->pdata->tx_num + j]) || (data < chstatus_min[i * ts->pdata->tx_num + j])) {
						error_point[i * ts->pdata->tx_num + j] = 1;
						ts->pdata->self_diagnostic[0] = 0;
					}
				}
				len += snprintf(buf + len, PAGE_SIZE - len, "%5d", data);
			}
		len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	}

	/* read touch key chstatus */
	len += snprintf(buf + len, PAGE_SIZE - len, "key: ");
	write_buf[0] = MMS_UNIVERSAL_CMD;
	write_buf[1] = MMS_KEY_CH_STATUS;
	write_buf[2] = 0xff;
	write_buf[3] = 0;
	i2c_master_send(ts->client, write_buf, 4);

	while (gpio_get_value(ts->pdata->gpio_int)) {
		flag++;
		if (flag == 100) {
			flag = 0;
			break;
		}
		udelay(100);
	}

	write_buf[0] = MMS_CMD_RESULT_SIZE;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, &read_size, 1);

	write_buf[0] = MMS_UNIVERSAL_RESULT;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, read_buf, read_size);

	for (t = 0; t < ts->pdata->key_num ; t++)
	{
		data = read_buf[2 * t] | (read_buf[2 * t + 1] << 8);
		if ((alloc_flag != -1) && (ret != -1)) {
			if ((data > chstatus_max[ts->pdata->rx_num* ts->pdata->tx_num + t]) || (data < chstatus_min[ts->pdata->rx_num* ts->pdata->tx_num + t])) {
				error_point[ts->pdata->rx_num* ts->pdata->tx_num + t] = 1;
				ts->pdata->self_diagnostic[0] = 0;
			}
		}
		len += snprintf(buf + len, PAGE_SIZE - len, "%5d", data);
	}

	len += snprintf(buf + len, PAGE_SIZE - len, "\n===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "========================\n");
	if (alloc_flag == -1) {
		len += snprintf(buf + len, PAGE_SIZE - len, "<<No spec examination>>");
	} else if (ret == -1) {
		len += snprintf(buf + len, PAGE_SIZE - len, "<<No spec file>>");
	} else if (ret == -2) {
		len += snprintf(buf + len, PAGE_SIZE - len, "<<Error spec file, please check key limit>>");
	} else if (ret == -3) {
		len += snprintf(buf + len, PAGE_SIZE - len, "<<Error spec file, please check the number of channel >>");
	} else {
		len += snprintf(buf + len, PAGE_SIZE - len, "Result = %s\n", ts->pdata->self_diagnostic[0] == 1 ? "PASS" : "FAIL");
		if (ts->pdata->self_diagnostic[0] == 0) {
			len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
			len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
			len += snprintf(buf + len, PAGE_SIZE - len, "========================\n");
			len += snprintf(buf + len, PAGE_SIZE - len, "%5c", c);
			for (j = 0; j < ts->pdata->tx_num; j++)
				len += snprintf(buf + len, PAGE_SIZE - len, "%5d", j);
			len += snprintf(buf + len, PAGE_SIZE - len, "\n");
			len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------");
			len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------");
			len += snprintf(buf + len, PAGE_SIZE - len, "------------------------\n");
			for (i = 0; i < ts->pdata->rx_num ; i++) {
				len += snprintf(buf + len, PAGE_SIZE - len, "%2d : ", i);
				for (j = 0; j <ts->pdata->tx_num; j++) {
					if (error_point[i * ts->pdata->tx_num + j] == 1) {
						len += snprintf(buf + len, PAGE_SIZE - len, "%5c", 'X');
					} else {
						len += snprintf(buf + len, PAGE_SIZE - len, "%5c", ' ');
					}
				}
				len += snprintf(buf + len, PAGE_SIZE - len, "\n");
			}
			len += snprintf(buf + len, PAGE_SIZE - len, "key:");
			for (t = 0; t <ts->pdata->key_num; t++) {
				if (error_point[ts->pdata->rx_num* ts->pdata->tx_num + t] == 1) {
					len += snprintf(buf + len, PAGE_SIZE - len, "%5c", 'X');
				} else {
					len += snprintf(buf + len, PAGE_SIZE - len, "%5c", ' ');
				}
			}
		}
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "\n===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "========================\n");

	write_buf[0] = MMS_UNIVERSAL_CMD;
	write_buf[1] = MMS_UNIVERSAL_CMD_EXIT;
	i2c_master_send(ts->client, write_buf, 2);

	write_buf[0] = MMS_CMD_RESULT_SIZE;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, &read_size, 1);

	write_buf[0] = MMS_UNIVERSAL_RESULT;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, read_buf, read_size);

	if (irq_flag == 1) {
		TOUCH_INFO_MSG("enable_irq\n");
		enable_irq(ts->client->irq);
		irq_flag = 0;
	}

	if (error_point) {
		kfree(error_point);
	}
	if (chstatus_max) {
		kfree(chstatus_max);
	}
	if (chstatus_min) {
		kfree(chstatus_min);
	}

	power_unlock(POWER_SYSFS_LOCK);

	return len;
}


static ssize_t
mms_rawdata_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mms_ts_info *ts = dev_get_drvdata(dev);
	char c = '-';
	int len = 0, i = 0, j = 0, t = 0;
	uint8_t write_buf[5] = {0};
	uint8_t read_buf[80] = {0};
	uint8_t read_size = 0;
	uint16_t data = 0;
	int* error_point = NULL;
	int flag = 0;
	int* raw_data_max = NULL;
	int* raw_data_min = NULL;
	int ret = 0, count = 0;
	int alloc_flag = 0;

	error_point = kzalloc(sizeof(int) * (ts->pdata->rx_num * ts->pdata->tx_num + ts->pdata->key_num), GFP_KERNEL);
	if (error_point == NULL) {
		TOUCH_INFO_MSG("failed to allocate error_point\n");
		alloc_flag = -1;
	}

	raw_data_max = kzalloc(sizeof(int) * (ts->pdata->rx_num * ts->pdata->tx_num + ts->pdata->key_num), GFP_KERNEL);
	if (raw_data_max == NULL) {
		TOUCH_INFO_MSG("failed to allocate raw_data_max\n");
		alloc_flag = -1;
	}

	raw_data_min = kzalloc(sizeof(int) * (ts->pdata->rx_num * ts->pdata->tx_num + ts->pdata->key_num), GFP_KERNEL);
	if (raw_data_min == NULL) {
		TOUCH_INFO_MSG("failed to allocate raw_data_min\n");
		alloc_flag = -1;
	}

	power_lock(POWER_SYSFS_LOCK);
	mms_ts_enable(ts);
	ts->pdata->self_diagnostic[1] = 1;

	if (alloc_flag != -1) {
		ret = get_limit(ts, "raw_data_max", raw_data_max, NULL);
		if ((ret != -1) && (get_limit(ts, "raw_data_min", raw_data_min, NULL) == -1))
			ret = -1;
	}

	if (irq_flag == 0) {
		TOUCH_INFO_MSG("disable_irq_nosync\n");
		disable_irq_nosync(ts->client->irq);
		irq_flag = 1;
	}

	write_buf[0] = MMS_UNIVERSAL_CMD;
	write_buf[1] = MMS_TOUCH_KEY_EVENT;
	i2c_master_send(ts->client, write_buf, 2);

	do{
		while (gpio_get_value(ts->pdata->gpio_int)) {
			flag++;
			if (flag == 30) {
				flag = 0;
				break;
			}
			msleep(100);
		}

		write_buf[0] = 0x0F;
		i2c_master_send(ts->client, write_buf, 1);
		i2c_master_recv(ts->client, read_buf, 1);

		write_buf[0] = 0x10;
		i2c_master_send(ts->client, write_buf, 1);
		i2c_master_recv(ts->client, read_buf, 1);
		TOUCH_INFO_MSG("Maker is %x\n", read_buf[0]);
		count++;
	}while(read_buf[0]!=0x0C&&count!=10);

	write_buf[0] = MMS_UNIVERSAL_CMD;
	write_buf[1] = MMS_TEST_RAW_DATA;
	i2c_master_send(ts->client, write_buf, 2);

	while (gpio_get_value(ts->pdata->gpio_int)) {
		flag++;
		if (flag == 30) {
			flag = 0;
			break;
		}
		msleep(100);
	}
	flag = 0;

	write_buf[0] = MMS_CMD_RESULT_SIZE;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, &read_size, 1);

	write_buf[0] = MMS_UNIVERSAL_RESULT;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, read_buf, read_size);
	TOUCH_INFO_MSG("RAW DATA TEST =%d \n", read_buf[0]);

	len = snprintf(buf, PAGE_SIZE, "\n<< SHOW RAW DATA >>\n");
	if (ts->pannel_on == 0) {
		len += snprintf(buf + len, PAGE_SIZE - len, "************************\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "*****    LCD OFF   *****\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "************************\n");
	} else if (ts->pannel_on == 1) {
		len += snprintf(buf + len, PAGE_SIZE - len, "************************\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "*****    LCD ON    *****\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "************************\n");
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "%5c", c);
	for (j = 0; j < ts->pdata->tx_num; j++)
		len += snprintf(buf + len, PAGE_SIZE - len, "%5d", j);
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------");
	len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------");
	len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------\n");

	/* read touch screen rawdata */
	for (i = 0; i < ts->pdata->rx_num ; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len, "%2d : ", i);
		write_buf[0] = MMS_UNIVERSAL_CMD;
		write_buf[1] = MMS_GET_PIXEL_RAW_DATA;
		write_buf[2] = 0xFF;
		write_buf[3] = i;

		i2c_master_send(ts->client, write_buf, 4);

		while (gpio_get_value(ts->pdata->gpio_int)) {
			flag++;
			if (flag == 100) {
				flag = 0;
				break;
			}
			udelay(100);
		}

		write_buf[0] = MMS_CMD_RESULT_SIZE;
		i2c_master_send(ts->client, write_buf, 1);
		i2c_master_recv(ts->client, &read_size, 1);

		write_buf[0] = MMS_UNIVERSAL_RESULT;
		i2c_master_send(ts->client, write_buf, 1);
		i2c_master_recv(ts->client, read_buf, read_size);

		for (j = 0; j < ts->pdata->tx_num; j++) {
			data = read_buf[2 * j] | (read_buf[2 * j + 1] << 8);
			if ((alloc_flag != -1) && (ret != -1)) {
				if ((data > raw_data_max[i * ts->pdata->tx_num + j]) || (data < raw_data_min[i * ts->pdata->tx_num + j])) {
					error_point[i * ts->pdata->tx_num + j] = 1;
					ts->pdata->self_diagnostic[1] = 0;
				}
			}
			len += snprintf(buf + len, PAGE_SIZE - len, "%5d", data);
		}
		len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	}

	/* read touch key rawdata */
	len += snprintf(buf + len, PAGE_SIZE - len, "key: ");

	write_buf[0] = MMS_UNIVERSAL_CMD;
	write_buf[1] = MMS_KEY_RAW_DATA;
	write_buf[2] = 0xff;
	write_buf[3] = 0;

	i2c_master_send(ts->client, write_buf, 4);

	while (gpio_get_value(ts->pdata->gpio_int)) {
		flag++;
		if (flag == 100) {
			flag = 0;
			break;
		}
		udelay(100);
	}

	write_buf[0] = MMS_CMD_RESULT_SIZE;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, &read_size, 1);

	write_buf[0] = MMS_UNIVERSAL_RESULT;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, read_buf, read_size);

	for (t = 0; t < ts->pdata->key_num ; t++)
	{
		data = read_buf[2 * t] | (read_buf[2 * t + 1] << 8);
		if ((alloc_flag != -1) && (ret != -1)) {
			if ((data > raw_data_max[ts->pdata->rx_num* ts->pdata->tx_num + t]) || (data < raw_data_min[ts->pdata->rx_num* ts->pdata->tx_num + t])) {
				error_point[ts->pdata->rx_num* ts->pdata->tx_num + t] = 1;
				ts->pdata->self_diagnostic[1] = 0;
			}
		}
		len += snprintf(buf + len, PAGE_SIZE - len, "%5d", data);
	}

	len += snprintf(buf + len, PAGE_SIZE - len, "\n===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================\n");

	if (alloc_flag == -1) {
			len += snprintf(buf + len, PAGE_SIZE - len, "<<No spec examination>>");
	} else if (ret == -1) {
			len += snprintf(buf + len, PAGE_SIZE - len, "<<No spec file>>");
	} else if (ret == -2) {
		len += snprintf(buf + len, PAGE_SIZE - len, "<<Error spec file, please check key limit>>");
	} else if (ret == -3) {
		len += snprintf(buf + len, PAGE_SIZE - len, "<<Error spec file, please check the number of channel >>");
	} else {
		len += snprintf(buf + len, PAGE_SIZE - len, "Result = %s\n", ts->pdata->self_diagnostic[1] == 1 ? "PASS" : "FAIL");
		if (ts->pdata->self_diagnostic[1] == 0) {
			len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
			len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
			len += snprintf(buf + len, PAGE_SIZE - len, "========================\n");
			len += snprintf(buf + len, PAGE_SIZE - len, "%5c", c);
			for (j = 0; j < ts->pdata->tx_num; j++)
				len += snprintf(buf + len, PAGE_SIZE - len, "%5d", j);
			len += snprintf(buf + len, PAGE_SIZE - len, "\n");
			len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------");
			len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------");
			len += snprintf(buf + len, PAGE_SIZE - len, "------------------------\n");
			for (i = 0; i < ts->pdata->rx_num ; i++) {
				len += snprintf(buf + len, PAGE_SIZE - len, "%2d : ", i);
				for (j = 0; j <ts->pdata->tx_num; j++) {
					if (error_point[i * ts->pdata->tx_num + j] == 1) {
						len += snprintf(buf + len, PAGE_SIZE - len, "%5c", 'X');
					} else {
						len += snprintf(buf + len, PAGE_SIZE - len, "%5c", ' ');
					}
				}
				len += snprintf(buf + len, PAGE_SIZE - len, "\n");
			}
			len += snprintf(buf + len, PAGE_SIZE - len, "key:");
			for (t = 0; t <ts->pdata->key_num; t++) {
				if (error_point[ts->pdata->rx_num* ts->pdata->tx_num + t] == 1) {
					len += snprintf(buf + len, PAGE_SIZE - len, "%5c", 'X');
				} else {
					len += snprintf(buf + len, PAGE_SIZE - len, "%5c", ' ');
				}
			}
		}
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "\n===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "========================\n");

	write_buf[0] = MMS_UNIVERSAL_CMD;
	write_buf[1] = MMS_UNIVERSAL_CMD_EXIT;
	i2c_master_send(ts->client, write_buf, 2);

	write_buf[0] = MMS_CMD_RESULT_SIZE;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, &read_size, 1);

	write_buf[0] = MMS_UNIVERSAL_RESULT;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, read_buf, read_size);

	if (irq_flag == 1) {
		TOUCH_INFO_MSG("enable_irq\n");
		enable_irq(ts->client->irq);
		irq_flag = 0;
	}

	if (error_point) {
		kfree(error_point);
	}
	if (raw_data_max) {
		kfree(raw_data_max);
	}
	if (raw_data_min) {
		kfree(raw_data_min);
	}

	power_unlock(POWER_SYSFS_LOCK);

	return len;
}

static ssize_t
mms_jitter_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mms_ts_info *ts = dev_get_drvdata(dev);
    char c = '-';
    int len = 0, i = 0, j = 0, t = 0;
    uint8_t write_buf[5] = {0};
    uint8_t read_buf[20] = {0};
    uint8_t read_size = 0;
    uint16_t data = 0;
	int *error_point = NULL;
	int flag = 0;
	int jitter_upper_limit = 1;
	int jitter_low_limit = -1;
	int ret = 0, count = 0;
	int alloc_flag = 0;

	error_point = kzalloc(sizeof(int) * (ts->pdata->rx_num * ts->pdata->tx_num + ts->pdata->key_num), GFP_KERNEL);
	if (error_point == NULL) {
		TOUCH_INFO_MSG("failed to allocate error_point\n");
		alloc_flag = -1;
	}

	power_lock(POWER_SYSFS_LOCK);

	mms_ts_enable(ts);
	ts->pdata->self_diagnostic[2] = 1;

	if (alloc_flag != -1) {
		ret = get_limit(ts, "jitter_upper_limit", NULL, &jitter_upper_limit);
		if ((ret != -1) && (get_limit(ts, "jitter_low_limit", NULL, &jitter_low_limit) == -1))
			ret = -1;
	}

	if (irq_flag == 0) {
		TOUCH_INFO_MSG("disable_irq_nosync\n");
		disable_irq_nosync(ts->client->irq);
		irq_flag = 1;
	}

    write_buf[0] = MMS_UNIVERSAL_CMD;
    write_buf[1] = MMS_TOUCH_KEY_EVENT;
    i2c_master_send(ts->client, write_buf, 2);

    do{
		while (gpio_get_value(ts->pdata->gpio_int)) {
			flag++;
			if (flag == 30) {
				flag = 0;
				break;
			}
			msleep(100);
		}

		write_buf[0] = 0x0F;
		i2c_master_send(ts->client, write_buf, 1);
		i2c_master_recv(ts->client, read_buf, 1);

		write_buf[0] = 0x10;
		i2c_master_send(ts->client, write_buf, 1);
		i2c_master_recv(ts->client, read_buf, 1);
		TOUCH_INFO_MSG("Maker is %x\n", read_buf[0]);
		count++;
	}while(read_buf[0]!=0x0C&&count!=10);

    write_buf[0] = MMS_UNIVERSAL_CMD;
    write_buf[1] = MMS_TEST_JITTER;
    i2c_master_send(ts->client, write_buf, 2);

	while (gpio_get_value(ts->pdata->gpio_int)) {
		flag++;
		if (flag == 30) {
			flag = 0;
			break;
		}
		msleep(100);
	}

	flag = 0;

    write_buf[0] = MMS_CMD_RESULT_SIZE;
    i2c_master_send(ts->client, write_buf, 1);
    i2c_master_recv(ts->client, &read_size, 1);

    write_buf[0] = MMS_UNIVERSAL_RESULT;
    i2c_master_send(ts->client, write_buf, 1);
    i2c_master_recv(ts->client, read_buf, read_size);


	len = snprintf(buf, PAGE_SIZE, "\n<< SHOW JITTER >>\n");
	if(ts->pannel_on == 0){
		len += snprintf(buf + len, PAGE_SIZE - len, "************************\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "*****    LCD OFF   *****\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "************************\n");
	}else if(ts->pannel_on == 1){
		len += snprintf(buf + len, PAGE_SIZE - len, "************************\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "*****    LCD ON    *****\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "************************\n");
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "========================\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "%5c", c);
    for (j = 0; j < ts->pdata->tx_num; j++)
		len += snprintf(buf + len, PAGE_SIZE - len, "%5d", j);
    len += snprintf(buf + len, PAGE_SIZE - len, "\n");
    len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------");
    len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------");
    len += snprintf(buf + len, PAGE_SIZE - len, "------------------------\n");

    for (i = 0; i < ts->pdata->rx_num ; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len, "%2d : ", i);
		for (j = 0; j < ts->pdata->tx_num; j++) {
			write_buf[0] = MMS_UNIVERSAL_CMD;
			write_buf[1] = MMS_GET_PIXEL_JITTER;
			write_buf[2] = j;
			write_buf[3] = i;
			i2c_master_send(ts->client, write_buf, 4);

			while (gpio_get_value(ts->pdata->gpio_int)) {
				flag++;
				if (flag == 100) {
					flag = 0;
					break;
				}
				udelay(100);
			}

			write_buf[0] = MMS_CMD_RESULT_SIZE;
			i2c_master_send(ts->client, write_buf, 1);
			i2c_master_recv(ts->client, &read_size, 1);

			write_buf[0] = MMS_UNIVERSAL_RESULT;
			i2c_master_send(ts->client, write_buf, 1);
			i2c_master_recv(ts->client, read_buf, read_size);

			data = read_buf[0];
			if ((alloc_flag != -1) && (ret != -1)) {
				if ((data > jitter_upper_limit) || (data < jitter_low_limit)) {
					error_point[i * ts->pdata->tx_num + j] = 1;
					ts->pdata->self_diagnostic[2] = 0;
				}
			}
			len += snprintf(buf + len, PAGE_SIZE - len, "%5d", data);
		}
		len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	}

	/* read touch key jitter */
	len += snprintf(buf + len, PAGE_SIZE - len, "key: ");
	for (t = 0; t < ts->pdata->key_num ; t++)
	{
		write_buf[0] = MMS_UNIVERSAL_CMD;
		write_buf[1] = MMS_KEY_JITTER;
		write_buf[2] = t;
		write_buf[3] = 0;

		i2c_master_send(ts->client, write_buf, 4);
		while (gpio_get_value(ts->pdata->gpio_int)) {
			flag++;
			if (flag == 100) {
				flag = 0;
				break;
			}
			udelay(100);
		}

		write_buf[0] = MMS_CMD_RESULT_SIZE;
		i2c_master_send(ts->client, write_buf, 1);
		i2c_master_recv(ts->client, &read_size, 1);

		write_buf[0] = MMS_UNIVERSAL_RESULT;
		i2c_master_send(ts->client, write_buf, 1);
		i2c_master_recv(ts->client, read_buf, read_size);

		data = read_buf[0];
		if ((alloc_flag != -1) && (ret != -1)) {
			if ((data > jitter_upper_limit) || (data < jitter_low_limit)) {
				error_point[ts->pdata->rx_num* ts->pdata->tx_num + t] = 1;
				ts->pdata->self_diagnostic[2] = 0;
			}
		}
		len += snprintf(buf + len, PAGE_SIZE - len, "%5d", data);
	}

    len += snprintf(buf + len, PAGE_SIZE - len, "\n===============================================");
    len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
    len += snprintf(buf + len, PAGE_SIZE - len, "========================\n");
	if (alloc_flag == -1) {
		len += snprintf(buf + len, PAGE_SIZE - len, "<<No spec examination>>");
	} else if (ret == -1) {
		len += snprintf(buf + len, PAGE_SIZE - len, "<<No spec file>>");
	} else {
		len += snprintf(buf + len, PAGE_SIZE - len, "Result = %s\n", ts->pdata->self_diagnostic[2] == 1 ? "PASS" : "FAIL");
		if (ts->pdata->self_diagnostic[2] == 0) {
			len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
			len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
			len += snprintf(buf + len, PAGE_SIZE - len, "========================\n");
			len += snprintf(buf + len, PAGE_SIZE - len, "%5c", c);
			for (j = 0; j < ts->pdata->tx_num; j++)
				len += snprintf(buf + len, PAGE_SIZE - len, "%5d", j);
			len += snprintf(buf + len, PAGE_SIZE - len, "\n");
			len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------");
			len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------");
			len += snprintf(buf + len, PAGE_SIZE - len, "------------------------\n");
			for (i = 0; i < ts->pdata->rx_num ; i++) {
				len += snprintf(buf + len, PAGE_SIZE - len, "%2d : ", i);
				for (j = 0; j <ts->pdata->tx_num; j++) {
					if (error_point[i * ts->pdata->tx_num + j] == 1) {
						len += snprintf(buf + len, PAGE_SIZE - len, "%5c", 'X');
					} else {
						len += snprintf(buf + len, PAGE_SIZE - len, "%5c", ' ');
					}
				}
				len += snprintf(buf + len, PAGE_SIZE - len, "\n");
			}
			len += snprintf(buf + len, PAGE_SIZE - len, "key :");
			for (i = 0; i < ts->pdata->key_num; i++) {
				if (error_point[ts->pdata->rx_num* ts->pdata->tx_num  + i] == 1) {
					len += snprintf(buf + len, PAGE_SIZE - len, "%5c", 'X');
				} else {
					len += snprintf(buf + len, PAGE_SIZE - len, "%5c", ' ');
				}
			}
		}
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "\n===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "========================\n");

    write_buf[0] = MMS_UNIVERSAL_CMD;
    write_buf[1] = MMS_UNIVERSAL_CMD_EXIT;

    i2c_master_send(ts->client, write_buf, 2);

	write_buf[0] = MMS_CMD_RESULT_SIZE;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, &read_size, 1);

	write_buf[0] = MMS_UNIVERSAL_RESULT;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, read_buf, read_size);

	while (gpio_get_value(ts->pdata->gpio_int)) {
		flag++;
		if (flag == 100) {
			flag = 0;
			break;
			}
		udelay(100);
		}

	if (irq_flag == 1) {
		TOUCH_INFO_MSG("enable_irq\n");
        enable_irq(ts->client->irq);
        irq_flag = 0;
    }

	if (error_point) {
		kfree(error_point);
	}

	power_unlock(POWER_SYSFS_LOCK);

    return len;
}

static ssize_t
mms_delta_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mms_ts_info *ts = dev_get_drvdata(dev);
	char c = '-';
	int len = 0, i = 0, j = 0;
	uint8_t write_buf[10] = {0};
	uint8_t read_buf[40] = {0};
	int8_t data = 0;
	uint8_t read_size = 0;
	int flag = 0;
	int ret = 0;

	power_lock(POWER_SYSFS_LOCK);

	mms_ts_enable(ts);

	TOUCH_INFO_MSG("disable_irq\n");
	disable_irq(ts->client->irq);

	len = snprintf(buf, PAGE_SIZE, "\n<< SHOW DELTA >>\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "%5c", c);
	for (j = 0; j < ts->pdata->tx_num; j++)
		len += snprintf(buf + len, PAGE_SIZE - len, "%5d", j);
	len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------");
	len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------");
	len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------\n");

	for (i = 0; i < ts->pdata->rx_num; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len, "%2d : ", i);
		j = 0;
		write_buf[0] = MMS_UNIVERSAL_CMD;
		write_buf[1] = MMS_UNIVERSAL_DELTA;
		write_buf[2] = 0xFF;  /*Exciting CH.*/
		write_buf[3] = i;  /*Sensing CH.*/

		ret = i2c_master_send(ts->client, write_buf, 4);

		while (gpio_get_value(ts->pdata->gpio_int)) {
			flag++;
			if (flag == 100) {
				flag = 0;
				break;
			}
			udelay(100);
		}

		write_buf[0] = MMS_CMD_RESULT_SIZE;
		i2c_master_send(ts->client, write_buf, 1);
		i2c_master_recv(ts->client, &read_size, 1);

		write_buf[0] = MMS_UNIVERSAL_RESULT;
		i2c_master_send(ts->client, write_buf, 1);

		if(read_size > 40) read_size = 40;
		i2c_master_recv(ts->client, read_buf, read_size);

		read_size >>= 1;

		for (j = 0; j < read_size ; j++) {
			data = read_buf[2 * j] | (read_buf[2 * j + 1] << 8);
			len += snprintf(buf + len, PAGE_SIZE - len, "%5d", data);
		}
		len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	}

	len += snprintf(buf + len, PAGE_SIZE - len, "key: ");

	write_buf[0] = MMS_UNIVERSAL_CMD;
	write_buf[1] = 0x71;
	write_buf[2] = 0xff;
	write_buf[3] = 0;

	i2c_master_send(ts->client, write_buf, 4);
	while (gpio_get_value(ts->pdata->gpio_int)) {
		flag++;
		if (flag == 100) {
			flag = 0;
			break;
		}
		udelay(100);
	}
	write_buf[0] = MMS_CMD_RESULT_SIZE;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, &read_size, 1);
	write_buf[0] = MMS_UNIVERSAL_RESULT;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, read_buf, read_size);
	for (i = 0; i < ts->pdata->key_num; i++) //Model Dependent
	{
		data = read_buf[2 * i] | (read_buf[2 * i + 1] << 8);
		len += snprintf(buf + len, PAGE_SIZE - len, "%5d", data);
	}

	len += snprintf(buf + len, PAGE_SIZE - len, "\n===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================\n");

	TOUCH_INFO_MSG("enable_irq\n");
	enable_irq(ts->client->irq);

	power_unlock(POWER_SYSFS_LOCK);

	return len;
}

static ssize_t
mms_edge_expand_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mms_ts_info *ts = dev_get_drvdata(dev);
	int len = 0;

	mms_ts_enable(ts);

	mms_edge_expand_read(ts);

	len += snprintf(buf+len, PAGE_SIZE, "%d, %d, %d, %d",
		edge_expand[0], edge_expand[1], edge_expand[2], edge_expand[3]);

	return len;
}

static ssize_t
mms_edge_expand_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct mms_ts_info *ts = dev_get_drvdata(dev);
	int i = 0;
	int len = 0;
	int ret = 0;
	int find_num_cnt = 0;
	char *num_pos[4] = {0};
	long value[4] = {0};
	char *ptr = (char *)buf;

	mms_ts_enable(ts);

	if (ptr[0] >= '0' && ptr[0] <= '9') {
		len = strlen(ptr);
		num_pos[find_num_cnt++] = ptr; // first value

		for ( i = 1; i < len; i++) {
			if (ptr[i] == ',' && ptr[i+1] == ' ') {
				ptr[i] = '\0';
				num_pos[find_num_cnt++] = &ptr[i+2];
			}
		}
	}

	if (find_num_cnt != 4) {
		goto ErrorExit;
	}

	for (i = 0; i < 4; i++) {
		ret = kstrtol(num_pos[i], 10, &value[i]);
		if (ret || (value[i] < 64 || value[i] > 255)) {
			goto ErrorExit;
		}
	}

	for (i = 0; i < 4; i++) {
		edge_expand[i] = (uint8_t)value[i];
	}

	mms_edge_expand_write(ts);

	return count;

ErrorExit :
	TOUCH_INFO_MSG("edge_expand_store error. %s \n", buf);

	return count;
}


static DEVICE_ATTR(status, S_IRUGO | S_IWUSR, mms_status_show, NULL);
static DEVICE_ATTR(version, S_IRUGO | S_IWUSR, mms_version_show, NULL);
static DEVICE_ATTR(testmode_ver, S_IRUGO | S_IWUSR, mms_testmode_version_show, NULL);
static DEVICE_ATTR(power_control, S_IRUGO | S_IWUSR, NULL, mms_power_control_store);
static DEVICE_ATTR(irq_control, S_IRUGO | S_IWUSR, NULL, mms_irq_control_store);
static DEVICE_ATTR(reg_control, S_IRUGO | S_IWUSR, NULL, mms_reg_control_store);
static DEVICE_ATTR(fw_upgrade, S_IRUGO | S_IWUSR, NULL, mms_fw_upgrade);
static DEVICE_ATTR(chstatus, S_IRUGO | S_IWUSR, mms_chstatus_show, NULL);
static DEVICE_ATTR(rawdata, S_IRUGO | S_IWUSR, mms_rawdata_show, NULL);
static DEVICE_ATTR(jitter, S_IRUGO | S_IWUSR, mms_jitter_show, NULL);
static DEVICE_ATTR(delta, S_IRUGO | S_IWUSR, mms_delta_show, NULL);
#if !defined(CONFIG_MACH_MSM8X10_W5_AIO_US)
static DEVICE_ATTR(edge_expand, S_IRUGO | S_IWUGO, mms_edge_expand_show, mms_edge_expand_store);
#else
static DEVICE_ATTR(edge_expand, S_IRUGO | S_IWUSR, mms_edge_expand_show, mms_edge_expand_store);
#endif
#ifdef SENSING_TEST
static DEVICE_ATTR(sensing_test, S_IRUGO | S_IWUSR, NULL, mms136_sensing_test_store);
#endif

static ssize_t
mms_self_diagnostic_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mms_ts_info *ts = dev_get_drvdata(dev);
	int len = 0;
	u8 ver[16] = {0};
	char *sd_path = "/data/ts_raw_data.txt";

	mms_version_show(dev,&dev_attr_version,buf);
	write_file(sd_path,buf, 1);
	mms_chstatus_show(dev,&dev_attr_chstatus,buf);
	write_file(sd_path,buf, 0);
	mms_rawdata_show(dev,&dev_attr_rawdata,buf);
	write_file(sd_path,buf, 0);
	mms_jitter_show(dev,&dev_attr_jitter,buf);
	write_file(sd_path,buf, 0);

	get_fw_version(ts->client, ver);
	len += snprintf(buf + len, PAGE_SIZE - len, "Firmware Version : %d.%02d \n", (ver[2]&0x80?1:0), ver[2]&0x7F);
	len += snprintf(buf + len, PAGE_SIZE - len, "Boot:0x%X Core:0x%X Config:0x%X \n", ver[0], ver[1], ver[2]);
	memset(ver, 0x0, 16);	get_fw_product(ts->client, ver);
	len += snprintf(buf + len, PAGE_SIZE - len, "FW Product : %s \n", ver);
	len += snprintf(buf + len, PAGE_SIZE - len, "=======RESULT========\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "Channel Status : %s\n", ts->pdata->self_diagnostic[0] == 1 ? "Pass" : "Fail");
	len += snprintf(buf + len, PAGE_SIZE - len, "Raw Data : %s\n", ts->pdata->self_diagnostic[1] == 1 ? "Pass" : "Fail");
	//len += snprintf(buf + len, PAGE_SIZE - len, "Jitter : %s\n", ts->pdata->self_diagnostic[2] == 1 ? "Pass" : "Fail");

	return len;
}
static DEVICE_ATTR(sd,	S_IRUGO | S_IWUSR, mms_self_diagnostic_show, NULL);


static struct attribute *lge_touch_attributes[] = {
	&dev_attr_sd.attr,
	&dev_attr_status.attr,
	&dev_attr_version.attr,
	&dev_attr_testmode_ver.attr,
	&dev_attr_power_control.attr,
	&dev_attr_irq_control.attr,
	&dev_attr_reg_control.attr,
	&dev_attr_fw_upgrade.attr,
	&dev_attr_chstatus.attr,
	&dev_attr_rawdata.attr,
	&dev_attr_jitter.attr,
	&dev_attr_delta.attr,
	&dev_attr_edge_expand.attr,
#ifdef SENSING_TEST
	&dev_attr_sensing_test.attr,
#endif
	NULL,
};

static const struct attribute_group lge_touch_attr_group = {
	.attrs = lge_touch_attributes,
};

static int __devinit mms_ts_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct mms_ts_info *info;
	struct input_dev *input_dev;
	int ret = 0;
	int i = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -EIO;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	input_dev = input_allocate_device();

	if (!info || !input_dev) {
		TOUCH_INFO_MSG("Failed to allocated memory\n");
		return -ENOMEM;
	}

	info->client = client;
	info->input_dev = input_dev;
#ifdef CONFIG_OF
	info->pdata = kzalloc(sizeof(*info->pdata), GFP_KERNEL);
	if (!info->pdata) {
		TOUCH_INFO_MSG("Failed to allocated memory\n");
		return -ENOMEM;
	}
	ret = mms_ts_parse_dt(&client->dev, info->pdata);
	if (ret) {
		TOUCH_INFO_MSG("Failed to parse device tree\n");
		return -ENODEV;
	}
#else
	info->pdata = client->dev.platform_data;
#endif
	init_completion(&info->init_done);

	if (info->pdata->gpio_vdd_en && gpio_is_valid(info->pdata->gpio_vdd_en)) {
		TOUCH_INFO_MSG("Set VDD GPIO_%d\n", info->pdata->gpio_vdd_en);
		gpio_request_one(info->pdata->gpio_vdd_en, GPIOF_OUT_INIT_HIGH, "mms_ts gpio_vdd_en");
	}

	ret = mms_ts_regulator_configure(info, 1);
	if (ret < 0) {
		TOUCH_INFO_MSG("Failed to configure regulators\n");
	}

	ret = mms_ts_power(info, 1);
	if (ret < 0) {
		TOUCH_INFO_MSG("Failed to power on\n");
		info->enabled = false;
	}
	else {
		info->enabled = true;
		msleep(50);
	}

	if (gpio_is_valid(info->pdata->gpio_int)) {
		TOUCH_INFO_MSG("Set INT GPIO_%d\n", info->pdata->gpio_int);
		gpio_request_one(info->pdata->gpio_int, GPIOF_IN, "mms_ts gpio_int");
	}

	info->pannel_on = 1;
	mutex_init(&info->lock);

	input_mt_init_slots(input_dev, MAX_FINGER_NUM);

	snprintf(info->phys, sizeof(info->phys),
		"%s/input0", dev_name(&client->dev));

	input_dev->name = DRIVER_NAME;
	input_dev->phys = info->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.init_name = LGE_TOUCH_NAME;
	input_dev->open = mms_ts_input_open;
	input_dev->close = mms_ts_input_close;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, MAX_WIDTH, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, MAX_PRESSURE, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, info->pdata->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, info->pdata->max_y, 0, 0);

	input_set_drvdata(input_dev, info);

	if (info->pdata->key_code[0]) {
		i = 0;
		__set_bit(EV_KEY, input_dev->evbit);
		while(info->pdata->key_code[i]) {
			__set_bit(info->pdata->key_code[i], input_dev->keybit);
			i++;
		}
	}

	ret = input_register_device(input_dev);
	if (ret) {
		TOUCH_INFO_MSG("failed to register input dev\n");
		return -EIO;
	}

	i2c_set_clientdata(client, info);

	if(info->pdata->auto_fw_update && info->pdata->fw_name != NULL)
	{
		TOUCH_INFO_MSG("Auto FW update. fw = %s \n", info->pdata->fw_name);
		mms_reboot(info);
		ret = request_firmware_nowait(THIS_MODULE, true, info->pdata->fw_name, &client->dev,
			GFP_KERNEL, info, mms_fw_update_controller);
		if (ret)
			TOUCH_INFO_MSG("failed to schedule firmware update\n");
	}
	else
	{
		mms_ts_config(info);
		mms_get_dev_info(info);
		enable_irq(info->irq);
	}

#ifdef CONFIG_FB
	info->fb_notifier.notifier_call = mms_ts_fb_notifier_call;
	ret = fb_register_client(&info->fb_notifier);
	if (ret) {
		TOUCH_INFO_MSG("%s: failed to register fb_notifier: %d\n", __func__, ret);
	}
#endif

	if (alloc_chrdev_region(&info->mms_dev, 0, 1, "mms_ts")) {
		TOUCH_INFO_MSG("failed to allocate device region\n");
		return -ENOMEM;
	}

	cdev_init(&info->cdev, &mms_fops);
	info->cdev.owner = THIS_MODULE;

	if (cdev_add(&info->cdev, info->mms_dev, 1)) {
		TOUCH_INFO_MSG("failed to add ch dev\n");
		return -EIO;
	}

	info->class = class_create(THIS_MODULE, "mms_ts");
	device_create(info->class, NULL, info->mms_dev, NULL, "mms_ts");

	ret = sysfs_create_bin_file(&client->dev.kobj ,&bin_attr);

	if (sysfs_create_link(NULL, &client->dev.kobj, "mms_ts")) {
		TOUCH_INFO_MSG("failed to create sysfs symlink\n");
		return -EAGAIN;
	}
/*
	if (sysfs_create_group(&client->dev.kobj, &mms_attr_group)) {
		dev_err(&client->dev, "failed to create sysfs group\n");
		return -EAGAIN;
	}*/

	ret = sysfs_create_group(&input_dev->dev.kobj, &lge_touch_attr_group);

	TOUCH_INFO_MSG("initialized\n");

	return 0;
}

static int __devexit mms_ts_remove(struct i2c_client *client)
{
	struct mms_ts_info *info = i2c_get_clientdata(client);

	if (info->irq >= 0)
		free_irq(info->irq, info);

	sysfs_remove_link(NULL, "mms_ts");
	input_unregister_device(info->input_dev);
#ifdef CONFIG_FB
	fb_unregister_client(&info->fb_notifier);
#endif
	sysfs_remove_bin_file(&client->dev.kobj, &bin_attr);
	device_destroy(info->class, info->mms_dev);
	class_destroy(info->class);
	
	kfree(info);

	return 0;
}

static int mms_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mms_ts_info *info = i2c_get_clientdata(client);

	info->pannel_on = 0;

	if(power_block)
		return 0;

	mutex_lock(&info->input_dev->mutex);

	if (info->input_dev->users) {
		mms_ts_disable(info);
		mms_clear_input_data(info);
	}

	mutex_unlock(&info->input_dev->mutex);

	return 0;
}

static int mms_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mms_ts_info *info = i2c_get_clientdata(client);

	info->pannel_on = 1;

	if(power_block)
		return 0;

	mutex_lock(&info->input_dev->mutex);

	if (info->input_dev->users) {
		mms_ts_enable(info);
	}

	info->ic_error_cnt = 0;

	mutex_unlock(&info->input_dev->mutex);

	return 0;
}

#ifdef CONFIG_FB
static int mms_ts_fb_notifier_call(struct notifier_block *self,
				   unsigned long event,
				   void *data)
{
	struct fb_event *evdata = data;
	int *fb;
	struct mms_ts_info *info = container_of(self, struct mms_ts_info, fb_notifier);
	if(evdata && evdata->data && event == FB_EVENT_BLANK && info && info->client) {
		fb = evdata->data;
		switch (*fb) {
			case FB_BLANK_UNBLANK:
				mms_ts_resume(&info->client->dev);
				break;
			case FB_BLANK_POWERDOWN:
				mms_ts_suspend(&info->client->dev);
				break;
			default:
				break;
		}
	}
	return 0;
}
#endif

static const struct i2c_device_id mms_ts_id[] = {
	{"mms_ts", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, mms_ts_id);

#ifdef CONFIG_OF
static struct of_device_id mms_ts_match_table[] = {
	{ .compatible = "melfas,mms100s",},
	{},
};
#endif

static struct i2c_driver mms_ts_driver = {
	.probe		= mms_ts_probe,
	.remove		= __devexit_p(mms_ts_remove),
	.driver		= {
				.name	= "touch_melfas",
#ifdef CONFIG_OF
				.of_match_table = mms_ts_match_table,
#endif
	},
	.id_table	= mms_ts_id,
};

static int __init mms_ts_init(void)
{
	return i2c_add_driver(&mms_ts_driver);
}

static void __exit mms_ts_exit(void)
{
	return i2c_del_driver(&mms_ts_driver);
}

module_init(mms_ts_init);
module_exit(mms_ts_exit);

MODULE_VERSION("1.5.1");
MODULE_DESCRIPTION("Touchscreen driver for MELFAS MMS-100s series");
MODULE_LICENSE("GPL");

