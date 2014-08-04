/* drivers/input/touchscreen/melfas_ts.c
 *
 * Copyright (C) 2010 Melfas, Inc.
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
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include "mms100a_ts.h"
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/firmware.h>
#include <linux/time.h>
#include <linux/file.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

// #define TOUCH_INT_SLEEP_COUNT_MEASURE
// #define TOUCH_CE_UNLIMITED_ONOFF_TEST

#define SENSING_TEST
#ifdef SENSING_TEST
#define SENSING_TEST_PATH		"/data/ts_log.txt"
#endif

#define LGE_TOUCH_NAME		"lge_touch"

#define IC_ERROR_RETRY_CNT	5

#define POWER_FW_UP_LOCK	0x01
#define POWER_SYSFS_LOCK	0x02

#define MODE_CONTROL                    	0x01
#define TS_READ_START_ADDR              	0x10

#define TS_READ_START_ADDR			0x10
#define TS_READ_VERSION_ADDR			0xE1
#define TS_HW_REVISION_ADDR             	0xF1
#define TS_CORE_VERSION_ADDR            	0xF3
#define TS_PRIVATE_CUSTOM_VERSION_ADDR  	0xF4
#define TS_PUBLIC_CUSTOM_VERSION_ADDR   	0xF5

#define UNIVERSAL_CMD				0xA0
#define UNIVERSAL_CMD_RESULT_SIZE		0xAE
#define UNIVERSAL_CMD_RESULT			0xAF
#define UNIVCMD_ENTER_TEST_MODE			0x40
#define UNIVCMD_TEST_CH_STATUS			0x41
#define UNIVCMD_GET_PIXEL_CH_STATUS		0x42
#define UNIVCMD_KEY_CH_STATUS		0x4A
#define UNIVCMD_TEST_RAW_DATA		0x43
#define UNIVCMD_GET_PIXEL_RAW_DATA		0x44
#define UNIVCMD_KEY_RAW_DATA		0x4B
#define UNIVCMD_TEST_JITTER                     0x45
#define UNIVCMD_GET_PIXEL_JITTER                0x46
#define UNIVCMD_KEY_JITTER      		0x4C
#define UNIVCMD_CMD_KEY_DELTA 0x71
#define UNIVCMD_CMD_DELTA 0x70
#define UNIVCMD_GET_TOP_EDGE_EXPAND			0x25
#define UNIVCMD_GET_BOTTOM_EDGE_EXPAND		0x26
#define UNIVCMD_GET_LEFT_EDGE_EXPAND			0x27
#define UNIVCMD_GET_RIGHT_EDGE_EXPAND		0x28

#define UNIVERSAL_CMD_EXIT			0x4F
#define TX_CH_NUM		0x0B
#define RX_CH_NUM		0x0C

#define TS_READ_REGS_LEN			100
#define TS_READ_VERSION_INFO_LEN		3

#define MELFAS_MAX_TOUCH			10  /* ts->pdata->num_of_finger */
#define MELFAS_MAX_BTN				4
#define MELFAS_PACKET_SIZE			6

#define I2C_RETRY_CNT				10

#define PRESS_KEY				1
#define RELEASE_KEY				0

#define MIP_INPUT_EVENT_PACKET_SIZE		0x0F
#define MIP_INPUT_EVENT_INFORMATION		0x10

/*#include "mms136_download.h"*/
#define MMS_FW_VERSION		0xE1
#define MMS_FW_PRODUCT		0xF6
#define MMS_CONFIRM_STATUS	0xAF
#define MMS_ENTER_ISC		0x5F
#define MMS_ENABLE_WRITE	0x55
#define MMS_STATUS_ISC_READY	0x01
#define MMS_WRITE_CMD		0xAE
#define MMS_GET_CUSTOM_ADDRESS	0xE5
#define MMS_DATA_WRITE		0xF1
#define MMS_STATUS_WRITING_DONE	0x03
#define MMS_SET_EDGE_EXPAND 0x32

/* Firmware file name */
#define EXTERNAL_FW_NAME			"mms_ts.mfsb"
#define CORE54_FW_NAME				"melfas/mms100a_core54_v01.mfsb"

#define get_time_interval(a, b) (a >= b ? a-b : 1000000 + a - b)
struct timeval t_debug[2];

static volatile int init_values[20];
static volatile int irq_flag;
static volatile int tmp_flag[10];
static volatile int point_of_release;
static volatile int time_diff;
static volatile int pre_keycode;
static volatile int touch_prestate;
static volatile int btn_prestate;
static uint8_t edge_expand[4] = {0};

#ifdef SENSING_TEST
char sensing_test = 0;
#endif

int power_block = 0;

extern int mms100_set_gpio_mux(char gpio, struct melfas_tsi_platform_data *pdata);

/***************************************************************************
 * Debug Definitions
 ***************************************************************************/
enum {
	OPERATION_MODE_NOT_SET = 0,
	OPERATION_MODE_STANBY = 1,
	OPERATION_MODE_ACTIVE = 2,
};

enum {
	MELFAS_TS_DEBUG_PROBE = 1U << 0,
	MELFAS_TS_DEBUG_KEY_EVENT = 1U << 1,
	MELFAS_TS_DEBUG_TOUCH_EVENT = 1U << 2,
	MELFAS_TS_DEBUG_TOUCH_EVENT_ONETIME = 1U << 3,
	MELFAS_TS_DEBUG_EVENT_HANDLER = 1U << 4,
	MELFAS_TS_DEBUG_IRQ_HANDLER = 1U << 5,
	MELFAS_TS_DEBUG_TIME = 1U << 6,
};

static int melfas_ts_debug_mask = 0x0;

module_param_named(
	debug_mask, melfas_ts_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP
);

#define MELFAS_TS_DPRINTK(mask, level, message, ...) \
	do { \
		if ((mask) & melfas_ts_debug_mask) \
			printk(level message, ## __VA_ARGS__); \
	} while (0)

#define MELFAS_TS_DEBUG_PRINT_TOUCH_EVENT(temp) \
	do { \
		MELFAS_TS_DPRINTK(MELFAS_TS_DEBUG_TOUCH_EVENT, KERN_INFO, \
			"[TOUCH] %s   %d : x=%d y=%d p=%d \n", \
			temp, i, g_Mtouch_info[i].posX, g_Mtouch_info[i].posY, g_Mtouch_info[i].pressure); \
		if (tmp_flag[i] == 1) { \
			MELFAS_TS_DPRINTK(MELFAS_TS_DEBUG_TOUCH_EVENT_ONETIME, KERN_INFO, \
			"[TOUCH] %s   %d : x=%d y=%d p=%d \n", \
			temp, i, g_Mtouch_info[i].posX, g_Mtouch_info[i].posY, g_Mtouch_info[i].pressure); \
			if (!strcmp (temp, "Press")) \
				tmp_flag[i] = 0;\
		} \
	} while (0)

#define MELFAS_TS_DEBUG_PRINT_TIME() \
	do { \
		if (MELFAS_TS_DEBUG_TIME & melfas_ts_debug_mask) { \
			if (t_debug[0].tv_sec == 0	&& t_debug[0].tv_usec == 0) { \
				t_debug[0].tv_sec = t_debug[1].tv_sec; \
				t_debug[0].tv_usec = t_debug[1].tv_usec; \
			} else { \
				printk("Interrupt interval: %6luus\n", get_time_interval(t_debug[1].tv_usec, t_debug[0].tv_usec)); \
				t_debug[0].tv_sec = t_debug[1].tv_sec; \
				t_debug[0].tv_usec = t_debug[1].tv_usec; \
			} \
		} \
	} while (0)

/**************************************************************************/

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
	fd = sys_open(filename, O_WRONLY|O_CREAT|O_APPEND, 0666);
	if (fd >= 0) {
		if(time > 0)
			sys_write(fd, time_string, strlen(time_string));
		sys_write(fd, data, strlen(data));
		sys_close(fd);
	}
	set_fs(old_fs);
}

enum {
	None = 0,
	TOUCH_SCREEN,
	TOUCH_KEY
};

struct muti_touch_info {
	int status;
	int strength;
	int width;
	int posX;
	int posY;
	int pressure;
	int btn_touch;
	int palm;
};

struct btn_info {
	int key_code;
	int status;
};

struct melfas_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct melfas_tsi_platform_data *pdata;
	struct mms_fw_info *info;
	int version;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	struct regulator *vdd;
	struct regulator *vcc_i2c;

	char fw_update;
	char poweron;
	char pannel_on;
	char operation_mode;
	int ic_error_cnt;

	struct mutex lock;
};

struct melfas_ts_data *gts;
extern int mms_flash_fw_file_isc(struct i2c_client *client, struct melfas_tsi_platform_data *pdata,  struct mms_fw_info *data);
extern int mms_flash_fw_file_isp(struct i2c_client *client, struct melfas_tsi_platform_data *pdata, const struct firmware *fw);
#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
#elif defined (CONFIG_HAS_EARLYSUSPEND)
static void melfas_ts_early_suspend(struct early_suspend *h);
static void melfas_ts_late_resume(struct early_suspend *h);
#endif

#if defined(CONFIG_LGE_PM)
static void melfas_ts_suspend(struct melfas_ts_data *ts);
static void melfas_ts_resume(struct melfas_ts_data *ts);
#endif

static void release_all_fingers(struct melfas_ts_data *ts);
static void release_all_keys(struct melfas_ts_data *ts);
static void melfas_ts_sw_reset(struct melfas_ts_data *ts);

static int melfas_power_on(struct melfas_ts_data *ts, bool on);

static struct muti_touch_info g_Mtouch_info[MELFAS_MAX_TOUCH];
static struct btn_info g_btn_info[MELFAS_MAX_BTN];
static int touched_count = 0;

static void power_lock(int value)
{
	power_block |= value;
}

static void power_unlock(int value)
{
	power_block &= ~(value);
}

#if defined(TOUCH_CE_UNLIMITED_ONOFF_TEST)
static struct workqueue_struct * work_ce_wq = NULL;
struct work_struct work_ce_str;
unsigned int work_ce_count = 0;
unsigned int work_int_count = 0;
bool work_ce_test_inited = false;
bool work_ce_test_run = false;

static void mms136_ce_onoff_test_func(struct work_struct *work)
{
	while(1) {
		if (work_ce_test_run == false)
			break;

		if (gpio_is_valid(60)) {
			gpio_direction_output(60, 1);
			TOUCH_INFO_MSG("CE On : %d \n", work_ce_count);
		}

		msleep(250);

		if (gpio_is_valid(60)) {
			gpio_direction_output(60, 0);
			TOUCH_INFO_MSG("CE Off : %d \n", work_ce_count);
		}

		msleep(250);

		if (work_ce_count < 0xFFFFFFFF)
			work_ce_count++;
	}

	if (gpio_is_valid(60)) {
		gpio_direction_output(60, 0);
	}

	TOUCH_INFO_MSG("Total CE On/Off Count : %d \n", work_ce_count);
	TOUCH_INFO_MSG("Total INT count = %d \n", work_int_count);
}

static ssize_t mms136_ce_test_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int len = 0;

	len += snprintf(buf+len, PAGE_SIZE, "Total CE On/Off Count : %d \n",work_ce_count);
	len += snprintf(buf+len, PAGE_SIZE, "Total INT count = %d \n",work_int_count);

	return len;
}

static ssize_t mms136_ce_test_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	int cmd = 0;

	if (sscanf(buf, "%d", &cmd) != 1)
		return -EINVAL;

	if (work_ce_test_inited == false) {
		work_ce_wq = create_singlethread_workqueue("work_ce_wq");
		if (unlikely(!work_ce_wq)) {
			TOUCH_INFO_MSG("failed to create singlethread work_ce_wq \n");
			return 0;
		}
		INIT_WORK(&work_ce_str, mms136_ce_onoff_test_func);
		work_ce_test_inited = true;
	}

	switch (cmd) {
	case 0: /*stop */
		if (work_ce_test_inited) {
			TOUCH_INFO_MSG("CE On/Off Test is stopping \n");
			work_ce_test_run = false;
			cancel_work_sync(&work_ce_str);
			disable_irq_wake(ts->client->irq);
			power_unlock(0xCE);
			melfas_power_on(ts, true);
		} else {
			TOUCH_INFO_MSG("CE On/Off Test is already stopped \n");
		}
		break;

	case 1: /* start */
		if (!work_ce_test_inited) {
			TOUCH_INFO_MSG("work_ce_test_inited = %d \n", work_ce_test_inited);
			return 0;
		}

		if (!work_ce_test_run) {
			TOUCH_INFO_MSG("CE On/Off Test is starting \n");

			if (power_block) {
				TOUCH_INFO_MSG("Power locked. Cannot run \n");
				return 0;
			}

			melfas_power_on(ts, false);
			power_lock(0xCE);

			enable_irq_wake(ts->client->irq);

			work_ce_test_run = true;
			work_ce_count = 0;
			work_int_count = 0;
			queue_work(work_ce_wq, &work_ce_str);
		} else {
			TOUCH_INFO_MSG("CE On/Off Test is under testing \n");
		}
		break;
	}
	return count;
}
#endif

#if defined(TOUCH_INT_SLEEP_COUNT_MEASURE)
static char bTouch_suspended = 0;
bool touch_int_counter_started = false;
unsigned int  touch_int_count = 0;
char touch_int_log_buf[256] = {0};
#define TOUCH_INT_COUNTER_LOG_PATH		"/data/int_cnt.txt"

static ssize_t
mms136_int_counter_expand_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int len = 0;

	len += snprintf(buf+len, PAGE_SIZE, "%d", touch_int_count);

	return len;
}

static ssize_t
mms136_int_counter_expand_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	int cmd = 0;

	if (sscanf(buf, "%d", &cmd) != 1)
		return -EINVAL;

	switch (cmd) {
	case 0:
		if (touch_int_counter_started) {
			TOUCH_INFO_MSG("Stop logging\r\n");

			disable_irq_wake(ts->client->irq);
			touch_int_counter_started = false;
		}
		break;
	case 1:
		if (!touch_int_counter_started) {
			TOUCH_INFO_MSG("Start logging\r\n");

			touch_int_counter_started = true;
			enable_irq_wake(ts->client->irq);
		}
		break;
	case 2:
		TOUCH_INFO_MSG("Reset logging count\r\n");

		touch_int_count = 0;
		break;
	}
	return count;
}

#endif

static int get_limit(struct melfas_ts_data *ts, char* breakpoint, int *limit_data, int *limit_jitter)
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

		if (r == (ts->pdata->rx_num * ts->pdata->tx_num + ts->pdata->key_num)) {
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

static int check_abs_time(void)
{
	time_diff = 0;

	if (!point_of_release)
		return 0;

	time_diff = jiffies_to_msecs(jiffies) - point_of_release;
	if (time_diff > 0)
		return time_diff;
	else
		return 0;
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

void mms_set_active_mode(struct melfas_ts_data *ts)
{
	uint8_t write_buf[2] = {0};
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

	if (!ts->pdata->enalbe_active_mode)
		return;

	if (ts->operation_mode)
		return;

	msg[0].len = 2;
	write_buf[0] = 0x33;
	write_buf[1] = OPERATION_MODE_ACTIVE;

	mdelay(50); // wait boot-up

	if (i2c_transfer(ts->client->adapter, &msg[0], 1) != 1) {
		TOUCH_INFO_MSG("%s i2c transfer failed \n", __func__);
	} else {
		TOUCH_INFO_MSG("Set Active mode \n");
		ts->operation_mode = OPERATION_MODE_ACTIVE;
	}

	mdelay(50); // wait apply
}

static void melfas_ts_event_handler(struct melfas_ts_data *ts)
{
	int ret = 0, i;
	uint8_t buf[TS_READ_REGS_LEN];
	int touchType = 0, touchState = 0, touchID = 0, pressed_type = 0, palm = 0;
	int posX = 0, posY = 0, width = 0, strength = 10;
	int keyID = 0, reportID = 0;
	uint8_t read_num = 0, pressed_count = 0;
	static int is_mix_event;
#ifdef SENSING_TEST
	char log_buf[256] = {0};
#endif

	if (ts == NULL) {
		TOUCH_INFO_MSG("%s TS is NULL\n", __func__);
		goto err_free_irq;
	}

	buf[0] = MIP_INPUT_EVENT_PACKET_SIZE;
	ret = i2c_master_send(ts->client, buf, 1);
	ret = i2c_master_recv(ts->client, &read_num, 1);
	/* touch ic reset for ESD defense  */
	if (ret < 0) {
		melfas_ts_sw_reset(ts);
		goto err_free_irq;
	}

	if (read_num == 0) {
		TOUCH_INFO_MSG("%s: read number 0 \n", __func__);
		goto err_free_irq;
	} else if (read_num > MELFAS_MAX_TOUCH*MELFAS_PACKET_SIZE) {
		TOUCH_INFO_MSG("%s: read number(%d) is out of range\n", __func__, read_num);
		goto err_free_irq;
	}

	buf[0] = MIP_INPUT_EVENT_INFORMATION;
	ret = i2c_master_send(ts->client, buf, 1);
	ret = i2c_master_recv(ts->client, &buf[0], read_num);

	/* touch ic reset for ESD defense
	     if reportID is -0x0F, meflas touch IC need sw reset */
	reportID = (buf[0] & 0x0F);
	if (reportID == 0x0F) {
		if (++ts->ic_error_cnt > IC_ERROR_RETRY_CNT) {
			TOUCH_INFO_MSG("Error cannot recover \n");
			melfas_power_on(ts, false);
			mms_set_active_mode(ts);
			release_all_fingers(ts);
			release_all_keys(ts);
			goto err_free_irq;
		}

		TOUCH_INFO_MSG("Error or ESD detected. Retry(%d/%d) \n", ts->ic_error_cnt, IC_ERROR_RETRY_CNT);
		melfas_ts_sw_reset(ts);
		goto err_free_irq;
	}

	for (i = 0; i < read_num; i = i + 6) {
		if (ret < 0) {
			TOUCH_INFO_MSG("%s: i2c failed\n", __func__);
			goto err_free_irq;
		} else {
			palm = ((buf[i] & 0x10) >> 4);						/* 1: Palm , 0: None */
			touchType  =  ((buf[i] & 0x60) >> 5);				/* Touch Screen, Touch Key */
			touchState = ((buf[i] & 0x80) == 0x80);				/* touchAction = (buf[0]>>7)&&0x01;*/
			reportID = (buf[i] & 0x0F);					/* Touch Screen -> n.th finger input
											Touch Key -> n.th touch key area. */
			posX = (uint16_t) (buf[i + 1] & 0x0F) << 8 | buf[i + 2];	/* X position (0 ~ 4096, 12 bit) */
			posY = (uint16_t) (buf[i + 1] & 0xF0) << 4 | buf[i + 3];	/* Y position (0 ~ 4096, 12 bit) */
			width = buf[i + 4];
			strength = buf[i + 5];

			if (palm) {
				if (touchState)
					TOUCH_INFO_MSG("Palm detected : %d \n", strength);
				else
					TOUCH_INFO_MSG("Palm released : %d \n", strength);

				release_all_fingers(ts);
				release_all_keys(ts);
				continue;
			}

			if (touchType == TOUCH_KEY)
				keyID = reportID;
			else if (touchType == TOUCH_SCREEN) {
				if (reportID == 0) {
					TOUCH_INFO_MSG("WARN : reportID == 0 \n");
					continue;
				}
				touchID = reportID - 1;
				pressed_type = TOUCH_SCREEN;
			}

			if (touchID > ts->pdata->num_of_finger-1)
				goto err_free_irq;

			if (touchType == TOUCH_SCREEN && touchID < MELFAS_MAX_TOUCH) {
				g_Mtouch_info[touchID].posX = posX;
				g_Mtouch_info[touchID].posY = posY;
				g_Mtouch_info[touchID].width = width;

				g_Mtouch_info[touchID].strength = strength;
				g_Mtouch_info[touchID].pressure = strength;
				g_Mtouch_info[touchID].btn_touch = touchState;
				g_Mtouch_info[touchID].palm = palm;

				if (btn_prestate && touch_prestate == 0) {
					input_report_key(ts->input_dev, pre_keycode, 0xff);
					TOUCH_INFO_MSG("Cancel : KEY[%s:%3d] \n", get_touch_button_string((u16)pre_keycode), pre_keycode);
					btn_prestate = 0;
				}
			} else if (touchType == TOUCH_KEY) {
				g_btn_info[keyID].key_code = ts->pdata->button[keyID-1];
				g_btn_info[keyID].status = touchState;

				if (keyID > ts->pdata->num_of_button || keyID == 0) {
					TOUCH_INFO_MSG("Touchkey ID error \n");
				} else if (is_mix_event == 1) {
					input_report_key(ts->input_dev, pre_keycode, 0xff);
					TOUCH_INFO_MSG("Cancel : KEY[%s:%3d] \n", get_touch_button_string((u16)pre_keycode), pre_keycode);
					is_mix_event = 0;
					btn_prestate = touchState;
				} else{
					if (touch_prestate) {
						btn_prestate = touchState;
					} else if (check_abs_time() > 0 && check_abs_time() < 100) {
						btn_prestate = touchState;
						point_of_release = 0;
					} else if (btn_prestate != touchState) {
						if (touchState == PRESS_KEY) {
							pre_keycode = ts->pdata->button[keyID-1];
							input_report_key(ts->input_dev, ts->pdata->button[keyID-1], PRESS_KEY);
							TOUCH_INFO_MSG("KEY[%s:%3d] is pressed \n", get_touch_button_string(ts->pdata->button[keyID-1]), ts->pdata->button[keyID-1]);
						} else {
							input_report_key(ts->input_dev, ts->pdata->button[keyID-1], RELEASE_KEY);
							TOUCH_INFO_MSG("KEY[%s:%3d] is released \n", get_touch_button_string(ts->pdata->button[keyID-1]), ts->pdata->button[keyID-1]);
						}
						btn_prestate = touchState;
					}
				}

				if ((read_num > 6) && (pressed_type == TOUCH_SCREEN)) {
					if (touchState && (touch_prestate == 0))
						is_mix_event = 1;
					touchType = TOUCH_SCREEN;
				}

				MELFAS_TS_DPRINTK(MELFAS_TS_DEBUG_KEY_EVENT, KERN_INFO,
					"[TOUCH] %s: keyID : %d, touchState: %d\n", __func__, keyID, touchState);
				break;
			}
		}
	}

	if (touchType == TOUCH_SCREEN) {
		for (i = 0; i < ts->pdata->num_of_finger; i++) {

#if defined(TOUCH_INT_SLEEP_COUNT_MEASURE)
			if (posX == 0 && posY == 0)
				continue;
#endif
			if (g_Mtouch_info[i].btn_touch == -1)
				continue;

			if (g_Mtouch_info[i].btn_touch == 0) {
				g_Mtouch_info[i].btn_touch = -1;
				tmp_flag[i] = 1;
				g_Mtouch_info[i].status = 0;
				TOUCH_INFO_MSG("touch_release[%s] : <%d> x[%3d] y[%3d] \n",
					g_Mtouch_info[i].palm?"Palm":" ",
					i, g_Mtouch_info[i].posX, g_Mtouch_info[i].posY);
				touched_count--;
#ifdef SENSING_TEST
				if(sensing_test == 1) {
					sprintf(log_buf, "%3d %3d %3d %s\n",
						g_Mtouch_info[i].posX,
						g_Mtouch_info[i].posY,
						g_Mtouch_info[i].strength,
						g_Mtouch_info[i].strength > 0 ? "DOWN" : "UP");
					write_file(SENSING_TEST_PATH, log_buf, 1);
				}
#endif
				continue;
			}
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, g_Mtouch_info[i].posX);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, g_Mtouch_info[i].posY);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, g_Mtouch_info[i].width);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, g_Mtouch_info[i].strength);
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
			input_mt_sync(ts->input_dev);
			if(g_Mtouch_info[i].status == 0) {
				g_Mtouch_info[i].status = 1;
				TOUCH_INFO_MSG("%d finger pressed : <%d> x[%3d] y[%3d] z[%3d] \n",
						++touched_count, i, g_Mtouch_info[i].posX, g_Mtouch_info[i].posY, g_Mtouch_info[i].strength);
#ifdef SENSING_TEST
				if(sensing_test == 1) {
					sprintf(log_buf,"%3d %3d %3d %s\n",
						g_Mtouch_info[i].posX,
						g_Mtouch_info[i].posY,
						g_Mtouch_info[i].strength,
						g_Mtouch_info[i].strength > 0 ? "DOWN" : "UP");
					write_file(SENSING_TEST_PATH, log_buf, 1);
				}
#endif
			}

			touch_prestate = 1;
			pressed_count++;
		}

		if (pressed_count == 0) {
			input_mt_sync(ts->input_dev);
			touch_prestate = 0;
			point_of_release = jiffies_to_msecs(jiffies);
		}
	}
	input_sync(ts->input_dev);

	MELFAS_TS_DEBUG_PRINT_TIME();
	return;
err_free_irq:
	return;
}

static irqreturn_t melfas_ts_irq_handler(int irq, void *handle)
{
	struct melfas_ts_data *ts = (struct melfas_ts_data *)handle;

#if defined(TOUCH_CE_UNLIMITED_ONOFF_TEST)
	if (work_ce_test_run) {
		if (work_int_count < 0xFFFFFFFF)
			work_int_count++;
		TOUCH_INFO_MSG("INT count = %d \n", work_int_count);
	}
#endif

	irq_flag = 1;

	MELFAS_TS_DPRINTK(MELFAS_TS_DEBUG_IRQ_HANDLER, KERN_INFO, "melfas_ts_irq_handler\n");

	if (MELFAS_TS_DEBUG_TIME & melfas_ts_debug_mask)
	    do_gettimeofday(&t_debug[1]);

#if defined(TOUCH_INT_SLEEP_COUNT_MEASURE)
	if (bTouch_suspended && touch_int_counter_started) {
		if (touch_int_count < 0xFFFFFFFF)
			touch_int_count++;

		memset(touch_int_log_buf, 0x0, 256);
		sprintf(touch_int_log_buf, "Cnt : %d \r\n",touch_int_count);
		TOUCH_INFO_MSG("%s", touch_int_log_buf);
		write_file(TOUCH_INT_COUNTER_LOG_PATH, touch_int_log_buf, 1);
	}else {
		melfas_ts_event_handler(ts);
	}
#else
	melfas_ts_event_handler(ts);
#endif

	irq_flag = 0;
	return IRQ_HANDLED;
}

static void melfas_get_dev_info(struct melfas_ts_data *ts)
{
	u8 temp = 0;
	uint8_t buf[TS_READ_VERSION_INFO_LEN] = {0};
	uint8_t product[16] = {0};

	temp = TX_CH_NUM;
	i2c_master_send(ts->client, &temp, 1);
	i2c_master_recv(ts->client, buf, 3);

	if(buf[0] == 0 || buf[1]  == 0) {
		temp = TX_CH_NUM;
		i2c_master_send(ts->client, &temp, 1);
		i2c_master_recv(ts->client, buf, 3);
	}

	ts->pdata->tx_num = buf[0];
	ts->pdata->rx_num = buf[1];
	ts->pdata->key_num = buf[2];

	temp = TS_READ_VERSION_ADDR;
	i2c_master_send(ts->client, &temp, 1);
	i2c_master_recv(ts->client, buf, TS_READ_VERSION_INFO_LEN);
	ts->version = buf[2];

	temp = MMS_FW_PRODUCT;
	i2c_master_send(ts->client, &temp, 1);
	i2c_master_recv(ts->client, product, 8);

	TOUCH_INFO_MSG("Firmware Version : %d.%02d \n", (buf[2]&0x80?1:0), buf[2]&0x7F);
	TOUCH_INFO_MSG("Boot:0x%X  Core:0x%X  Config:0x%X \n", buf[0], buf[1], buf[2]);
	TOUCH_INFO_MSG("FW Product : %s \n", product);
	TOUCH_INFO_MSG("Num of Channel. TX:%d RX:%d KEY:%d\n", ts->pdata->tx_num, ts->pdata->rx_num, ts->pdata->key_num);
}

static ssize_t
mms136_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	uint8_t verbuf[TS_READ_VERSION_INFO_LEN] = {0};
	uint8_t product[16] = {0};
	uint8_t chbuf[3] = {0};
	int len = 0;

	power_lock(POWER_SYSFS_LOCK);
	melfas_power_on(ts, true);
	mms_set_active_mode(ts);

	melfas_get_dev_info(ts);

	verbuf[0] = TS_READ_VERSION_ADDR;
	i2c_master_send(ts->client, &verbuf[0], 1);
	i2c_master_recv(ts->client, &verbuf[0], TS_READ_VERSION_INFO_LEN);
	ts->version = verbuf[2];

	product[0] = MMS_FW_PRODUCT;
	i2c_master_send(ts->client, &product[0], 1);
	i2c_master_recv(ts->client, &product[0], 8);

	chbuf[0] = TX_CH_NUM;
	i2c_master_send(ts->client, &chbuf[0], 1);
	i2c_master_recv(ts->client, &chbuf[0], 3);

	len += snprintf(buf + len, PAGE_SIZE - len, "\n====================================\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "Firmware Version : %d.%02d \n", (verbuf[2]&0x80?1:0), verbuf[2]&0x7F);
	len += snprintf(buf + len, PAGE_SIZE - len, "Boot:0x%X  Core:0x%X  Config:0x%X \n", verbuf[0], verbuf[1], verbuf[2]);
	len += snprintf(buf + len, PAGE_SIZE - len, "FW Product : %s \n", product);
	len += snprintf(buf + len, PAGE_SIZE - len, "Num of Channel. TX:%d RX:%d KEY:%d\n", chbuf[0], chbuf[1], chbuf[2]);
	len += snprintf(buf + len, PAGE_SIZE - len, "\n====================================\n");

	power_unlock(POWER_SYSFS_LOCK);

	return len;
}

static ssize_t
mms136_testmode_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	uint8_t verbuf[TS_READ_VERSION_INFO_LEN] = {0};
	uint8_t product[16] = {0};
	int len = 0;

	power_lock(POWER_SYSFS_LOCK);
	melfas_power_on(ts, true);
	mms_set_active_mode(ts);

	melfas_get_dev_info(ts);

	verbuf[0] = TS_READ_VERSION_ADDR;
	i2c_master_send(ts->client, &verbuf[0], 1);
	i2c_master_recv(ts->client, &verbuf[0], TS_READ_VERSION_INFO_LEN);
	ts->version = verbuf[2];

	product[0] = MMS_FW_PRODUCT;
	i2c_master_send(ts->client, &product[0], 1);
	i2c_master_recv(ts->client, &product[0], 8);

	len += snprintf(buf+len, PAGE_SIZE, "%d.%02d(0x%X, 0x%X, 0x%X, %s)\n",
					(verbuf[2]&0x80?1:0), verbuf[2]&0x7F, verbuf[0], verbuf[1], verbuf[2], product);

	power_unlock(POWER_SYSFS_LOCK);

	return len;
}

static ssize_t
mms136_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	int len = 0;

	len = snprintf(buf, PAGE_SIZE, "\nMMS-136 Device Status\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "=============================\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "irq num       is %d\n", ts->client->irq);
	len += snprintf(buf + len, PAGE_SIZE - len, "gpio_irq num  is %d(level=%d)\n", ts->pdata->i2c_int_gpio, gpio_get_value(ts->pdata->i2c_int_gpio));
	len += snprintf(buf + len, PAGE_SIZE - len, "gpio_scl num  is %d\n", 19 /*ts->pdata->gpio_scl*/);
	len += snprintf(buf + len, PAGE_SIZE - len, "gpio_sda num  is %d\n", 18 /*ts->pdata->gpio_sda*/);
	len += snprintf(buf + len, PAGE_SIZE - len, "=============================\n");
	return len;
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

void mms_edge_expand_write(struct melfas_ts_data *ts)
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
		TOUCH_INFO_MSG("edge_expand_write i2c transfer failed\n");
	}

	TOUCH_INFO_MSG("Write Edge Expand : %d, %d, %d, %d \n",
		edge_expand[0], edge_expand[1], edge_expand[2], edge_expand[3]);
}

void mms_edge_expand_read(struct melfas_ts_data *ts)
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
		write_buf[0] = UNIVERSAL_CMD;
		write_buf[1] = UNIVCMD_GET_TOP_EDGE_EXPAND + i;
		msg[0].len = 2;

		if (i2c_transfer(ts->client->adapter, &msg[0], 1) != 1) {
			TOUCH_INFO_MSG("%s : i2c transfer failed\n", __func__);
			return;
		}

		if (mms_ts_i2c_read(ts->client, UNIVERSAL_CMD_RESULT_SIZE, read_buf, 1) < 0) {
			TOUCH_INFO_MSG("%s : Fail to get UNIVERSAL_CMD_RESULT_SIZE \n", __func__);
			return;
		}

		if (mms_ts_i2c_read(ts->client, UNIVERSAL_CMD_RESULT, read_buf, 1) < 0) {
			TOUCH_INFO_MSG("%s : Fail to get UNIVERSAL_CMD_RESULT \n", __func__);
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

static int mms_ts_get_ic_info(struct i2c_client *client, struct mms_fw_info *data)
{
	char buf[16] = {0};
	int i = 0;

	if (mms_ts_i2c_read(client, MMS_FW_VERSION, buf, MAX_SECTION_NUM) < 0)
		return -EIO;

	for (i = 0; i < MAX_SECTION_NUM; i++)
		data->ts_section[i].version = buf[i];

	TOUCH_INFO_MSG("Check IC FW Version : %d.%02d \n", (data->ts_section[2].version&0x80?1:0), data->ts_section[2].version&0x7F);
	TOUCH_INFO_MSG("Check IC FW Version : Boot:0x%X Core:0x%X Config:0x%X \n", data->ts_section[0].version, data->ts_section[1].version, data->ts_section[2].version);

	if (mms_ts_i2c_read(client, MMS_GET_CUSTOM_ADDRESS, buf, 8) < 0)
		return -EIO;

	for (i = 0; i < MAX_SECTION_NUM; i++) {
		data->ts_section[i].start_addr = buf[i];
		data->ts_section[i].end_addr = buf[i+4];
	}
	
	return 0;
}

static int mms_ts_firmware_image_parse(struct mms_fw_info *data, const u8 *image_bin)
{
	int offset = 0;
	int i = 0;

	data->fw_hdr = (struct mms_bin_hdr *) image_bin;

#if 0
	TOUCH_INFO_MSG("mms_fw_hdr:\n");
	TOUCH_INFO_MSG("\ttag[%c%c%c%c%c%c%c%c]\n",
		data->fw_hdr->tag[0], data->fw_hdr->tag[1], data->fw_hdr->tag[2], data->fw_hdr->tag[3],
		data->fw_hdr->tag[4], data->fw_hdr->tag[5], data->fw_hdr->tag[6], data->fw_hdr->tag[7]);
	TOUCH_INFO_MSG("\tcore_version[0x%02x]\n", data->fw_hdr->core_version);
	TOUCH_INFO_MSG("\tsection_num[%d]\n", data->fw_hdr->section_num);
	TOUCH_INFO_MSG("\tcontains_full_binary[%d]\n", data->fw_hdr->contains_full_binary);
	TOUCH_INFO_MSG("\tbinary_offset[%d (0x%04x)]\n", data->fw_hdr->binary_offset, data->fw_hdr->binary_offset);
	TOUCH_INFO_MSG("\tbinary_length[%d]\n", data->fw_hdr->binary_length);
#endif

	offset = sizeof(struct mms_bin_hdr);

	for (i = 0; i < data->fw_hdr->section_num; i++) {
		data->fw_img[i] = (struct mms_fw_img *) (image_bin + offset);

#if 0
		TOUCH_INFO_MSG("mms_fw_hdr[%d]:\n", i);
		TOUCH_INFO_MSG("\ttype[%d]\n", data->fw_img[i]->type);
		TOUCH_INFO_MSG("\tversion[0x%02x]\n", data->fw_img[i]->version);
		TOUCH_INFO_MSG("\tstart_page[%d]\n", data->fw_img[i]->start_page);
		TOUCH_INFO_MSG("\tend_page[%d]\n", data->fw_img[i]->end_page);
		TOUCH_INFO_MSG("\toffset[%d (0x%04x)]\n", data->fw_img[i]->offset, data->fw_img[i]->offset);
		TOUCH_INFO_MSG("\tlength[%d]\n", data->fw_img[i]->length);
#endif

		offset += sizeof(struct mms_fw_img);
	}

	TOUCH_INFO_MSG("Check FW IMG Version : %d.%02d \n", (data->fw_img[2]->version&0x80?1:0), data->fw_img[2]->version&0x7F);
	TOUCH_INFO_MSG("Check FW IMG Version : Boot:0x%X Core:0x%X Config:0x%X \n", data->fw_img[0]->version, data->fw_img[1]->version, data->fw_img[2]->version);
	return 0;
}

static int mms_ts_firmware_image_up_check(struct mms_fw_info *data){

	bool fw_up_to_date = true;
	int result = 0;
	int offset = 0;
	int i = 0;

	offset = sizeof(struct mms_bin_hdr);
	for (i = 0; i < data->fw_hdr->section_num; i++) {

		if(data->fw_img[i]->version != data->ts_section[i].version){
			fw_up_to_date = false;
			if(data->fw_img[0]->version != data->ts_section[0].version){
				data->need_update[0]=true;
				data->need_update[1]=true;
				data->need_update[2]=true;
				result = 2;
				TOUCH_INFO_MSG("fw need_update = [%d %d %d]\n", data->need_update[0], data->need_update[1], data->need_update[2]);
				return result;
			}

			else if(data->fw_img[1]->version != data->ts_section[1].version){
				data->need_update[0]=false;
				data->need_update[1]=true;
				data->need_update[2]=true;
				result = 2;
				TOUCH_INFO_MSG("fw need_update = [%d %d %d]\n", data->need_update[0], data->need_update[1], data->need_update[2]);
				return result;
			}

			else if(data->fw_img[2]->version != data->ts_section[2].version){
				data->need_update[0]=false;
				data->need_update[1]=false;
				data->need_update[2]=true;
				result = 1;
				TOUCH_INFO_MSG("fw need_update = [%d %d %d]\n", data->need_update[0], data->need_update[1], data->need_update[2]);
				return result;
			}
		}		
		offset += sizeof(struct mms_fw_img);			
	}
 
	if (fw_up_to_date) {
		TOUCH_INFO_MSG("mms_ts firmware version is up to date\n");
		return 0;
	}
 
	return -EINVAL;
}

static int touch_fw_compare(struct melfas_ts_data *ts, const struct firmware *fw){
	char name[16] = {0};
	int result = 0;

	if(mms_ts_get_ic_info(ts->client, ts->info) < 0){
		TOUCH_INFO_MSG("%s : Fail to get ic info.(isc_fail) \n", __func__);
		return -EIO;
	}

	mms_ts_firmware_image_parse(ts->info, fw->data);

	if (mms_ts_i2c_read(ts->client, MMS_FW_PRODUCT, name, 8) < 0){
		TOUCH_INFO_MSG("%s : Fail to get fw product.(isc_fail) \n", __func__);
		return -EIO;
		}

	if(strlen(name) == 0){
		TOUCH_INFO_MSG("Fail to get MMS_FW_PRODUCT name. \n");
		return -EINVAL;
	}

	TOUCH_INFO_MSG("IC FW Product : %s \n", name);

	if(unlikely(ts->pdata->force_upgrade)){
		TOUCH_INFO_MSG("Force update \n");
		ts->info->need_update[0]=true;
		ts->info->need_update[1]=true;
		ts->info->need_update[2]=true;

		goto isp_upgrade;
	}

	if(strcmp(name, ts->pdata->product) != 0){
		TOUCH_INFO_MSG("Model name is not match [%s]\n", name);
		ts->info->need_update[0]=true;
		ts->info->need_update[1]=true;
		ts->info->need_update[2]=true;

		goto isp_upgrade;
	}

	result = mms_ts_firmware_image_up_check(ts->info);
	if(result == 0)
		return 0;
	else if(result == 1)
		goto isc_upgrade;
	else if(result == 2)
		goto isp_upgrade;
	else
		return -EINVAL;

isc_upgrade:
	return 1;

isp_upgrade:
	return 2;
}

static ssize_t
mms136_fw_upgrade_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	int cmd = 0;
	const struct firmware *fw = NULL;
	const char *fw_name = NULL;
	int upgrade_fuc =0;
	int retires = 3;

	if (sscanf(buf, "%d", &cmd) != 1)
		return -EINVAL;

	melfas_power_on(ts, true);

	disable_irq(ts->client->irq);

	power_lock(POWER_FW_UP_LOCK);

	ts->pdata->force_upgrade = 1;

	switch (cmd) {
	case 0:
		fw_name = CORE54_FW_NAME;
		break;

	case 1:
		fw_name = ts->pdata->fw_name;
		break;

	case 9 :
		fw_name = EXTERNAL_FW_NAME;
		break;

	default:
		TOUCH_INFO_MSG("usage: echo [0|1|9] > fw_upgrade\n");
		TOUCH_INFO_MSG(" -0 : From Core54 default fw \n");
		TOUCH_INFO_MSG(" -1 : From Kernel binary \n");
		TOUCH_INFO_MSG(" -9 : From external file \n");
		break;
	}

	if(fw_name == NULL)
		return count;

	TOUCH_INFO_MSG("%s \n", fw_name);

	if(request_firmware(&fw, fw_name, &ts->client->dev) >= 0) {
		do{
			upgrade_fuc = touch_fw_compare(ts, fw);
			if(upgrade_fuc == 1){
				if(mms_flash_fw_file_isc(ts->client, ts->pdata, ts->info) < 0){
					melfas_power_on(ts, false);
					melfas_power_on(ts, true);
					mms_flash_fw_file_isp(ts->client, ts->pdata, fw);
				}
			}else if(upgrade_fuc == 2)
				mms_flash_fw_file_isp(ts->client, ts->pdata, fw);

			if(upgrade_fuc < 0){
				melfas_power_on(ts, false);
				melfas_power_on(ts, true);
			}
		}while(upgrade_fuc < 0 && --retires);

		if (!retires){
			TOUCH_INFO_MSG("failed to flash firmware after retires, GO TO ISP\n");
			mms_flash_fw_file_isp(ts->client, ts->pdata, fw);
		}

		if (fw) {
			TOUCH_INFO_MSG("release_firmware \n");
			release_firmware(fw);
		}
	}else
		TOUCH_INFO_MSG("Fail to request firmware. \n");

	melfas_power_on(ts, false);
	melfas_power_on(ts, true);
	mms_set_active_mode(ts);

	power_unlock(POWER_FW_UP_LOCK);

	ts->pdata->force_upgrade = 0;
	enable_irq(ts->client->irq);

	msleep(50);

	melfas_get_dev_info(ts);

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
mms136_power_control_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	int cmd = 0;

	if (sscanf(buf, "%d", &cmd) != 1)
		return -EINVAL;

	switch (cmd) {
	case 0: /*touch power off */
		melfas_power_on(ts, false);
		break;
	case 1: /* touch power on */
		melfas_power_on(ts, true);
		mms_set_active_mode(ts);
		break;
	case 2:
		melfas_power_on(ts, false);
		msleep(50);
		melfas_power_on(ts, true);
		mms_set_active_mode(ts);
		break;
	default:
		TOUCH_INFO_MSG("usage: echo [0|1|2] > control\n");
		TOUCH_INFO_MSG("  - 0: power off\n");
		TOUCH_INFO_MSG("  - 1: power on\n");
		TOUCH_INFO_MSG("  - 2: power reset\n");
		break;
	}
	return count;
}

static ssize_t
mms136_fx_control_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	int cmd;
	int retval = 0;

	if (sscanf(buf, "%d", &cmd) != 1)
		return -EINVAL;

	switch (cmd) {
	case 0:
		if (irq_flag == 1) {
			TOUCH_INFO_MSG("enable_irq\n");
			enable_irq(ts->client->irq);
			irq_flag = 0;
		}
		if(ts->vdd) {
			retval = regulator_enable(ts->vdd);
			if (retval < 0) {
				TOUCH_INFO_MSG("Regulator vdd enable failed retval = %d\n", retval);
			}else{
				TOUCH_INFO_MSG("regulator_enable(VDD) \n");
			}
		}
		msleep(30);

		mms100_set_gpio_mux(2, ts->pdata);

		break;
	case 1:
		if (irq_flag == 0) {
			disable_irq_nosync(ts->client->irq);
			irq_flag = 1;
		}

		if(ts->vdd) {
			regulator_disable(ts->vdd);
			TOUCH_INFO_MSG("regulator_disable(VDD) \n");
		}

		mms100_set_gpio_mux(3, ts->pdata);

		break;
	default:
		TOUCH_INFO_MSG("usage: echo [0|1] > control\n");
		break;
	}
	return count;
}

static ssize_t
mms136_irq_control_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	int cmd = 0, ret = 0;

	if (sscanf(buf, "%d", &cmd) != 1)
		return -EINVAL;

	switch (cmd) {
	case 0: /* interrupt pin LOW */
		ret = gpio_direction_input(ts->pdata->i2c_int_gpio);
		if (ret < 0) {
			TOUCH_INFO_MSG("%s: gpio input direction fail\n", __FUNCTION__);
			break;
		}
		gpio_set_value(ts->pdata->i2c_int_gpio, 0);
		TOUCH_INFO_MSG("MMS-136 INTR GPIO pin low\n");
		break;
	case 1: /* interrupt pin high */
		ret = gpio_direction_input(ts->pdata->i2c_int_gpio);
		if (ret < 0) {
			TOUCH_INFO_MSG("%s: gpio input direction fail\n", __FUNCTION__);
			break;
		}
		gpio_set_value(ts->pdata->i2c_int_gpio, 1);
		TOUCH_INFO_MSG("MMS-136 INTR GPIO pin high\n");
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
mms136_reg_control_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	int cmd = 0, ret = 0, reg_addr = 0, length = 0, i = 0;
	uint8_t reg_buf[TS_READ_REGS_LEN] = {0};

	if (sscanf(buf, "%d, 0x%x, %d", &cmd, &reg_addr, &length) != 3)
		return -EINVAL;

	melfas_power_on(ts, true);
	mms_set_active_mode(ts);

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
mms136_chstatus_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	char c = '-';
	int len = 0, i = 0, j = 0, t = 0;
	uint8_t write_buf[5] = {0};
	uint8_t read_buf[80] = {0};
	uint8_t read_size = 0;
	uint16_t data = 0;
	int *error_point = NULL;
	int flag = 0, count = 0;
	int *chstatus_max = NULL;
	int *chstatus_min = NULL;
	int ret = 0;
	int alloc_flag = 0;

	error_point = kzalloc(sizeof(int) * (ts->pdata->rx_num * ts->pdata->tx_num  + ts->pdata->key_num), GFP_KERNEL);
	if (error_point == NULL) {
		TOUCH_INFO_MSG("failed to allocate error_point\n");
		alloc_flag = -1;
	}

	chstatus_max = kzalloc(sizeof(int) * (ts->pdata->rx_num * ts->pdata->tx_num  + ts->pdata->key_num), GFP_KERNEL);
	if (chstatus_max == NULL) {
		TOUCH_INFO_MSG("failed to allocate chstatus_max\n");
		alloc_flag = -1;
	}

	chstatus_min = kzalloc(sizeof(int) * (ts->pdata->rx_num * ts->pdata->tx_num  + ts->pdata->key_num), GFP_KERNEL);
	if (chstatus_min == NULL) {
		TOUCH_INFO_MSG("failed to allocate chstatus_min\n");
		alloc_flag = -1;
	}

	power_lock(POWER_SYSFS_LOCK);

	melfas_power_on(ts, true);
	mms_set_active_mode(ts);
	msleep(200);
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
	write_buf[0] = UNIVERSAL_CMD;
	write_buf[1] = UNIVCMD_ENTER_TEST_MODE;
	i2c_master_send(ts->client, write_buf, 2);

	do{
		while (gpio_get_value(ts->pdata->i2c_int_gpio)) {
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

	write_buf[0] = UNIVERSAL_CMD;
	write_buf[1] = UNIVCMD_TEST_CH_STATUS;
	i2c_master_send(ts->client, write_buf, 2);

	while (gpio_get_value(ts->pdata->i2c_int_gpio)) {
		flag++;
		if (flag == 30) {
			flag = 0;
			break;
		}
		msleep(100);
	}
	flag = 0;

	write_buf[0] = UNIVERSAL_CMD_RESULT_SIZE;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, &read_size, 1);

	write_buf[0] = UNIVERSAL_CMD_RESULT;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, read_buf, read_size);
	TOUCH_INFO_MSG("Chstatus TEST =%d \n", read_buf[0]);

	len = snprintf(buf, PAGE_SIZE, "\n<< SHOW CHANNEL STATUS >>\n");
	if (ts->pannel_on == 0) {
		len += snprintf(buf + len, PAGE_SIZE - len, "************************\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "*** LCD STATUS : OFF ***\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "************************\n");
	} else if (ts->pannel_on == 1) {
		len += snprintf(buf + len, PAGE_SIZE - len, "************************\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "*** LCD STATUS : ON ***\n");
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

	for (i = 0; i < ts->pdata->rx_num; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len, "%2d : ", i);
		write_buf[0] = UNIVERSAL_CMD;
		write_buf[1] = UNIVCMD_GET_PIXEL_CH_STATUS;
		write_buf[2] = 0xFF;
		write_buf[3] = i;
		i2c_master_send(ts->client, write_buf, 4);

		while (gpio_get_value(ts->pdata->i2c_int_gpio)) {
			flag++;
			if (flag == 100) {
				flag = 0;
				break;
			}
			udelay(100);
		}

		write_buf[0] = UNIVERSAL_CMD_RESULT_SIZE;
		i2c_master_send(ts->client, write_buf, 1);
		i2c_master_recv(ts->client, &read_size, 1);

		write_buf[0] = UNIVERSAL_CMD_RESULT;
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
	if (ts->pdata->key_num) {
		len += snprintf(buf + len, PAGE_SIZE - len, "key: ");
		write_buf[0] = UNIVERSAL_CMD;
		write_buf[1] = UNIVCMD_KEY_CH_STATUS;
		write_buf[2] = 0xff;
		write_buf[3] = 0;
		i2c_master_send(ts->client, write_buf, 4);

		while (gpio_get_value(ts->pdata->i2c_int_gpio)) {
			flag++;
			if (flag == 100) {
				flag = 0;
				break;
			}
			udelay(100);
		}

		write_buf[0] = UNIVERSAL_CMD_RESULT_SIZE;
		i2c_master_send(ts->client, write_buf, 1);
		i2c_master_recv(ts->client, &read_size, 1);

		write_buf[0] = UNIVERSAL_CMD_RESULT;
		i2c_master_send(ts->client, write_buf, 1);
		i2c_master_recv(ts->client, read_buf, read_size);

		for (t = 0; t < ts->pdata->key_num ; t++)
		{
			data = read_buf[2 * t] | (read_buf[2 * t + 1] << 8);
			if ((alloc_flag != -1) && (ret != -1)) {
				if ((data > chstatus_max[ts->pdata->rx_num * ts->pdata->tx_num + t]) || (data < chstatus_min[ts->pdata->rx_num * ts->pdata->tx_num + t])) {
					error_point[ts->pdata->rx_num * ts->pdata->tx_num + t] = 1;
					ts->pdata->self_diagnostic[0] = 0;
				}
			}
			len += snprintf(buf + len, PAGE_SIZE - len, "%5d", data);
		}
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
		len += snprintf(buf + len, PAGE_SIZE - len, "Result = %s\n", ts->pdata->self_diagnostic[0] == 1  ? "PASS" : "FAIL");
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
			len += snprintf(buf + len, PAGE_SIZE - len, "key :");
			for (t = 0; t < ts->pdata->key_num; t++) {
				if (error_point[ts->pdata->rx_num* ts->pdata->tx_num  + t] == 1) {
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

	write_buf[0] = UNIVERSAL_CMD;
	write_buf[1] = UNIVERSAL_CMD_EXIT;

	i2c_master_send(ts->client, write_buf, 2);

	while (gpio_get_value(ts->pdata->i2c_int_gpio)) {
		flag++;
		if (flag == 100) {
			flag = 0;
			break;
		}
		udelay(100);
	}

	write_buf[0] = UNIVERSAL_CMD_RESULT_SIZE;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, &read_size, 1);

	write_buf[0] = UNIVERSAL_CMD_RESULT;
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
mms136_rawdata_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	char c = '-';
	int len = 0, i = 0, j = 0, t = 0;
	uint8_t write_buf[5] = {0};
	uint8_t read_buf[80] = {0};
	uint8_t read_size = 0;
	uint16_t data = 0;
	int* error_point = NULL;
	int flag = 0, count = 0;
	int* raw_data_max = NULL;
	int* raw_data_min = NULL;
	int ret = 0;
	int alloc_flag = 0;

	error_point = kzalloc(sizeof(int) * (ts->pdata->rx_num * ts->pdata->tx_num  + ts->pdata->key_num), GFP_KERNEL);
	if (error_point == NULL) {
		TOUCH_INFO_MSG("failed to allocate error_point\n");
		alloc_flag = -1;
	}

	raw_data_max = kzalloc(sizeof(int) *(ts->pdata->rx_num * ts->pdata->tx_num  + ts->pdata->key_num), GFP_KERNEL);
	if (raw_data_max == NULL) {
		TOUCH_INFO_MSG("failed to allocate raw_data_max\n");
		alloc_flag = -1;
	}

	raw_data_min = kzalloc(sizeof(int) * (ts->pdata->rx_num * ts->pdata->tx_num  + ts->pdata->key_num), GFP_KERNEL);
	if (raw_data_min == NULL) {
		TOUCH_INFO_MSG("failed to allocate raw_data_min\n");
		alloc_flag = -1;
	}

	power_lock(POWER_SYSFS_LOCK);

	melfas_power_on(ts, true);
	mms_set_active_mode(ts);
	msleep(200);
	ts->pdata->self_diagnostic[1] = 1;

	if ( alloc_flag != -1 ) {
		ret = get_limit(ts, "raw_data_max", raw_data_max, NULL);
		if ((ret != -1) && (get_limit(ts, "raw_data_min", raw_data_min, NULL) == -1))
			ret = -1;
	}
	if (irq_flag == 0) {
		TOUCH_INFO_MSG("disable_irq_nosync\n");
		disable_irq_nosync(ts->client->irq);
		irq_flag = 1;
	}
	write_buf[0] = UNIVERSAL_CMD;
	write_buf[1] = UNIVCMD_ENTER_TEST_MODE;
	i2c_master_send(ts->client, write_buf, 2);

	do{
		while (gpio_get_value(ts->pdata->i2c_int_gpio)) {
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

	write_buf[0] = UNIVERSAL_CMD;
	write_buf[1] = UNIVCMD_TEST_RAW_DATA;
	i2c_master_send(ts->client, write_buf, 2);

	while (gpio_get_value(ts->pdata->i2c_int_gpio)) {
		flag++;
		if (flag == 30) {
			flag = 0;
			break;
		}
		msleep(100);
	}
	flag = 0;



	write_buf[0] = UNIVERSAL_CMD_RESULT_SIZE;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, &read_size, 1);

	write_buf[0] = UNIVERSAL_CMD_RESULT;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, read_buf, read_size);
	TOUCH_INFO_MSG("RAW DATA TEST =%d \n", read_buf[0]);

	len = snprintf(buf, PAGE_SIZE, "\n<<  SHOW RAW DATA>>\n");
	if (ts->pannel_on == 0) {
		len += snprintf(buf + len, PAGE_SIZE - len, "************************\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "*** LCD STATUS : OFF ***\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "************************\n");
	} else if (ts->pannel_on == 1) {
		len += snprintf(buf + len, PAGE_SIZE - len, "************************\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "*** LCD STATUS : ON ***\n");
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

	for (i = 0; i < ts->pdata->rx_num ; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len, "%2d : ", i);
		write_buf[0] = UNIVERSAL_CMD;
		write_buf[1] = UNIVCMD_GET_PIXEL_RAW_DATA;
		write_buf[2] = 0xFF;
		write_buf[3] = i;
		i2c_master_send(ts->client, write_buf, 4);

		while (gpio_get_value(ts->pdata->i2c_int_gpio)) {
			flag++;
			if (flag == 100) {
				flag = 0;
				break;
			}
			udelay(100);
		}

		write_buf[0] = UNIVERSAL_CMD_RESULT_SIZE;
		i2c_master_send(ts->client, write_buf, 1);
		i2c_master_recv(ts->client, &read_size, 1);

		write_buf[0] = UNIVERSAL_CMD_RESULT;
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

	if (ts->pdata->key_num) {
		len += snprintf(buf + len, PAGE_SIZE - len, "key: ");
		write_buf[0] = UNIVERSAL_CMD;
		write_buf[1] = UNIVCMD_KEY_RAW_DATA;
		write_buf[2] = 0xff;
		write_buf[3] = 0;

		i2c_master_send(ts->client, write_buf, 4);

		while (gpio_get_value(ts->pdata->i2c_int_gpio)) {
			flag++;
			if (flag == 100) {
				flag = 0;
				break;
			}
			udelay(100);
		}

		write_buf[0] = UNIVERSAL_CMD_RESULT_SIZE;
		i2c_master_send(ts->client, write_buf, 1);
		i2c_master_recv(ts->client, &read_size, 1);

		write_buf[0] = UNIVERSAL_CMD_RESULT;
		i2c_master_send(ts->client, write_buf, 1);
		i2c_master_recv(ts->client, read_buf, read_size);

		for (t = 0; t < ts->pdata->key_num ; t++)
		{
			data = read_buf[2 * t] | (read_buf[2 * t + 1] << 8);
			if ((alloc_flag != -1) && (ret != -1)) {
				if ((data > raw_data_max[ts->pdata->rx_num * ts->pdata->tx_num + t]) || (data < raw_data_min[ts->pdata->rx_num * ts->pdata->tx_num + t])) {
					error_point[ts->pdata->rx_num * ts->pdata->tx_num + t] = 1;
					ts->pdata->self_diagnostic[1] = 0;
				}
			}
			len += snprintf(buf + len, PAGE_SIZE - len, "%5d", data);
		}
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
			len += snprintf(buf + len, PAGE_SIZE - len, "key :");
			for (t = 0; t < ts->pdata->key_num; t++) {
				if (error_point[ts->pdata->rx_num* ts->pdata->tx_num  + t] == 1) {
					TOUCH_INFO_MSG("key error point : %d \n",error_point[ts->pdata->rx_num* ts->pdata->tx_num + t]);
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

	write_buf[0] = UNIVERSAL_CMD;
	write_buf[1] = UNIVERSAL_CMD_EXIT;

	i2c_master_send(ts->client, write_buf, 2);

	while (gpio_get_value(ts->pdata->i2c_int_gpio)) {
		flag++;
		if (flag == 100) {
			flag = 0;
			break;
		}
		udelay(100);
	}

	write_buf[0] = UNIVERSAL_CMD_RESULT_SIZE;

	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, read_buf, 1);

	write_buf[0] = UNIVERSAL_CMD_RESULT;

	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, read_buf, read_buf[0]);

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
mms136_jitter_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	char c = '-';
	int len, i, j ;
	uint8_t write_buf[5] = {0};
	uint8_t read_buf[20] = {0};
	uint8_t read_size = 0;
	uint16_t data = 0;
	int *error_point = NULL;
	int flag = 0, count = 0;
	int jitter_upper_limit = 1;
	int jitter_low_limit = -1;
	int ret = 0;
	int alloc_flag = 0;

	error_point = kzalloc(sizeof(int) * (ts->pdata->rx_num * ts->pdata->tx_num  + ts->pdata->key_num), GFP_KERNEL);
	if (error_point == NULL) {
		TOUCH_INFO_MSG("failed to allocate error_point\n");
		alloc_flag = -1;
	}

	power_lock(POWER_SYSFS_LOCK);

	melfas_power_on(ts, true);
	mms_set_active_mode(ts);
	msleep(200);
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

	write_buf[0] = UNIVERSAL_CMD;
	write_buf[1] = UNIVCMD_ENTER_TEST_MODE;
	i2c_master_send(ts->client, write_buf, 2);

	do{
		 while (gpio_get_value(ts->pdata->i2c_int_gpio)) {
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

	write_buf[0] = UNIVERSAL_CMD;
	write_buf[1] = UNIVCMD_TEST_JITTER;
	i2c_master_send(ts->client, write_buf, 2);

	while (gpio_get_value(ts->pdata->i2c_int_gpio)) {
		flag++;
		if (flag == 30) {
			flag = 0;
			break;
		}
		msleep(100);
	}
	flag = 0;

	write_buf[0] = UNIVERSAL_CMD_RESULT_SIZE;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, &read_size, 1);

	write_buf[0] = UNIVERSAL_CMD_RESULT;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, read_buf, read_size);
	TOUCH_INFO_MSG("JITTER TEST =%d \n", read_buf[0]);

	len = snprintf(buf, PAGE_SIZE, "\n<<  SHOW JITTER >>\n");
	if (ts->pannel_on == 0) {
		len += snprintf(buf + len, PAGE_SIZE - len, "************************\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "*** LCD STATUS : OFF ***\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "************************\n");
	} else if (ts->pannel_on == 1) {
		len += snprintf(buf + len, PAGE_SIZE - len, "************************\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "*** LCD STATUS : ON ***\n");
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
			write_buf[0] = UNIVERSAL_CMD;
			write_buf[1] = UNIVCMD_GET_PIXEL_JITTER;
			write_buf[2] = j;
			write_buf[3] = i;
			i2c_master_send(ts->client, write_buf, 4);

			while (gpio_get_value(ts->pdata->i2c_int_gpio)) {
				flag++;
				if (flag == 100) {
					flag = 0;
					break;
				}
				udelay(100);
			}

			write_buf[0] = UNIVERSAL_CMD_RESULT_SIZE;
			i2c_master_send(ts->client, write_buf, 1);
			i2c_master_recv(ts->client, &read_size, 1);

			write_buf[0] = UNIVERSAL_CMD_RESULT;
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

	if (ts->pdata->key_num) {
		len += snprintf(buf + len, PAGE_SIZE - len, "key: ");
		write_buf[0] = UNIVERSAL_CMD;
		write_buf[1] = UNIVCMD_KEY_JITTER;
		write_buf[2] = 0xff;
		write_buf[3] = 0;

		i2c_master_send(ts->client, write_buf, 4);

		while (gpio_get_value(ts->pdata->i2c_int_gpio)) {
			flag++;
			if (flag == 100) {
				flag = 0;
				break;
			}
			udelay(100);
		}

		write_buf[0] = UNIVERSAL_CMD_RESULT_SIZE;
		i2c_master_send(ts->client, write_buf, 1);
		i2c_master_recv(ts->client, &read_size, 1);

		write_buf[0] = UNIVERSAL_CMD_RESULT;
		i2c_master_send(ts->client, write_buf, 1);
		i2c_master_recv(ts->client, read_buf, read_size);

		for (i = 0; i < ts->pdata->key_num ; i++)
		{
			data = read_buf[i];
			if ((alloc_flag != -1) && (ret != -1)) {
				if ((data > jitter_upper_limit) || (data < jitter_low_limit)) {
					error_point[ ts->pdata->rx_num * ts->pdata->tx_num + i] = 1;
					ts->pdata->self_diagnostic[2] = 0;
				}
			}
			len += snprintf(buf + len, PAGE_SIZE - len, "%5d", data);
		}
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

	write_buf[0] = UNIVERSAL_CMD;
	write_buf[1] = UNIVERSAL_CMD_EXIT;
	i2c_master_send(ts->client, write_buf, 2);

	write_buf[0] = UNIVERSAL_CMD_RESULT_SIZE;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, &read_size, 1);

	write_buf[0] = UNIVERSAL_CMD_RESULT;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, read_buf, read_size);

	while (gpio_get_value(ts->pdata->i2c_int_gpio)) {
		flag++;
		if (flag == 100) {
			flag = 0;
			break;
		}
		udelay(100);
	}

	i2c_master_recv(ts->client, read_buf, read_buf[0]);
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
mms136_delta_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	char c = '-';
	int len = 0, i = 0, j = 0;
	uint8_t sz = 0;
	uint8_t read_buf[25] = {0,};
	uint8_t write_buf[4] = {0,};
	s16 data = 0;

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

	power_lock(POWER_SYSFS_LOCK);

	melfas_power_on(ts, true);
	mms_set_active_mode(ts);

	if (irq_flag == 0) {
		TOUCH_INFO_MSG("disable_irq_nosync\n");
		disable_irq_nosync(ts->client->irq);
		irq_flag = 1;
	}

	len = snprintf(buf, PAGE_SIZE, "\n<< SHOW DELTA >>\n");
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

	for (i=0; i<ts->pdata->rx_num; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len, "%2d : ", i);
		for (j = 0; j<ts->pdata->tx_num; j++) {

			write_buf[0] = UNIVERSAL_CMD;
			write_buf[1] = 0x70;
			write_buf[2] = j;
			write_buf[3] = i;
			msg[0].len = 4;

			if (i2c_transfer(ts->client->adapter, &msg[0], 1) != 1) {
				TOUCH_INFO_MSG("intensity i2c transfer failed\n");
				return -1;
			}

			sz = i2c_smbus_read_byte_data(ts->client, UNIVERSAL_CMD_RESULT_SIZE);

			write_buf[0] = UNIVERSAL_CMD_RESULT;
			msg[0].len = 1;
			msg[1].len = sz;
			msg[1].buf = read_buf;

			if (i2c_transfer(ts->client->adapter, msg, ARRAY_SIZE(msg)) != ARRAY_SIZE(msg)) {
				return -1;
			}

			sz >>= 1;

			data = read_buf[1];
			data = ((data<<8)|read_buf[0]);
			len += snprintf(buf + len, PAGE_SIZE - len, "%5d", data);
		}

		len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	}

	if (ts->pdata->key_num) {
		len += snprintf(buf + len, PAGE_SIZE - len, "key: ");
		for (j = 0; j<ts->pdata->key_num; j++) {
			write_buf[0] = UNIVERSAL_CMD;
			write_buf[1] = UNIVCMD_CMD_KEY_DELTA;
			write_buf[2] = j;
			msg[0].len = 4;

			if (i2c_transfer(ts->client->adapter, &msg[0], 1) != 1) {
				TOUCH_INFO_MSG("KEY_DELTA i2c transfer failed\n");
				return -1;
			}

			sz = i2c_smbus_read_byte_data(ts->client, UNIVERSAL_CMD_RESULT_SIZE);

			write_buf[0] = UNIVERSAL_CMD_RESULT;
			msg[0].len = 1;
			msg[1].len = sz;
			msg[1].buf = read_buf;

			if (i2c_transfer(ts->client->adapter, msg, ARRAY_SIZE(msg)) != ARRAY_SIZE(msg)) {
				return -1;
			}

			sz >>= 1;

			data = read_buf[1];
			data = ((data << 8) | read_buf[0]);

			len += snprintf(buf + len, PAGE_SIZE - len, "%5d", data);
		}
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "\n===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "========================\n");

	if (irq_flag == 1) {
		TOUCH_INFO_MSG("enable_irq\n");
		enable_irq(ts->client->irq);
		irq_flag = 0;
	}

	power_unlock(POWER_SYSFS_LOCK);

	return len;
}

static ssize_t
mms136_edge_expand_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	int len = 0;

	melfas_power_on(ts, true);
	mms_set_active_mode(ts);

	mms_edge_expand_read(ts);

	len += snprintf(buf+len, PAGE_SIZE, "%d, %d, %d, %d",
		edge_expand[0], edge_expand[1], edge_expand[2], edge_expand[3]);

	return len;
}

static ssize_t
mms136_edge_expand_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	int i = 0;
	int len = 0;
	int ret = 0;
	int find_num_cnt = 0;
	char *num_pos[4] = {0};
	long value[4] = {0};
	char *ptr = (char *)buf;

	melfas_power_on(ts, true);
	mms_set_active_mode(ts);

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

static ssize_t
mms136_active_mode_control_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	int cmd = 0;

	if (sscanf(buf, "%d", &cmd) != 1)
		return -EINVAL;

	switch (cmd) {
	case 0: /*active mode off */
		ts->pdata->enalbe_active_mode = 0;
		TOUCH_INFO_MSG("Active Mode : Off\n");
		break;
	case 1: /*active mode on */
		ts->pdata->enalbe_active_mode = 1;
		TOUCH_INFO_MSG("Active Mode : On\n");
		break;
	default:
		TOUCH_INFO_MSG("usage: echo [0|1] > active_mode\n");
		TOUCH_INFO_MSG("  - 0: Active Mode off \n");
		TOUCH_INFO_MSG("  - 1: Active Mode On \n");
		break;
	}
	return count;
}

static DEVICE_ATTR(status,	S_IRUGO | S_IWUSR, mms136_status_show, NULL);
static DEVICE_ATTR(version, S_IRUGO | S_IWUSR, mms136_version_show, NULL);
static DEVICE_ATTR(testmode_ver, S_IRUGO | S_IWUSR, mms136_testmode_version_show, NULL);
static DEVICE_ATTR(fw_upgrade, S_IRUGO | S_IWUSR, NULL, mms136_fw_upgrade_store);
static DEVICE_ATTR(power_control, S_IRUGO | S_IWUSR, NULL, mms136_power_control_store);
static DEVICE_ATTR(fx_control, S_IRUGO | S_IWUSR, NULL, mms136_fx_control_store);
static DEVICE_ATTR(irq_control, S_IRUGO | S_IWUSR, NULL, mms136_irq_control_store);
static DEVICE_ATTR(reg_control, S_IRUGO | S_IWUSR, NULL, mms136_reg_control_store);
static DEVICE_ATTR(chstatus, S_IRUGO | S_IWUSR, mms136_chstatus_show, NULL);
static DEVICE_ATTR(rawdata, S_IRUGO | S_IWUSR, mms136_rawdata_show, NULL);
static DEVICE_ATTR(jitter, S_IRUGO | S_IWUSR, mms136_jitter_show, NULL);
static DEVICE_ATTR(delta, S_IRUGO | S_IWUSR, mms136_delta_show, NULL);
static DEVICE_ATTR(edge_expand, S_IRUGO | S_IWUGO, mms136_edge_expand_show, mms136_edge_expand_store);
static DEVICE_ATTR(active_mode, S_IRUGO | S_IWUSR, NULL, mms136_active_mode_control_store);
#if defined(TOUCH_CE_UNLIMITED_ONOFF_TEST)
static DEVICE_ATTR(ce_test, S_IRUGO | S_IWUGO, mms136_ce_test_show, mms136_ce_test_store);
#endif
#if defined(TOUCH_INT_SLEEP_COUNT_MEASURE)
static DEVICE_ATTR(misc, S_IRUGO | S_IWUGO, mms136_int_counter_expand_show, mms136_int_counter_expand_store);
#endif

static ssize_t
mms136_self_diagnostic_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct melfas_ts_data *ts = dev_get_drvdata(dev);
	uint8_t verbuf[TS_READ_VERSION_INFO_LEN] = {0};
	uint8_t product[16] = {0};
	int len = 0;
	char *sd_path = "/data/touch_self_test.txt";

	mms136_version_show(dev,&dev_attr_version,buf);
	write_file(sd_path,buf,1);
	mms136_chstatus_show(dev,&dev_attr_chstatus,buf);
	write_file(sd_path,buf,0);
	mms136_rawdata_show(dev,&dev_attr_rawdata,buf);
	write_file(sd_path,buf,0);
	mms136_jitter_show(dev,&dev_attr_jitter,buf);
	write_file(sd_path,buf,0);

	verbuf[0] = TS_READ_VERSION_ADDR;
	i2c_master_send(ts->client, &verbuf[0], 1);
	i2c_master_recv(ts->client, &verbuf[0], TS_READ_VERSION_INFO_LEN);
	ts->version = verbuf[2];

	product[0] = MMS_FW_PRODUCT;
	i2c_master_send(ts->client, &product[0], 1);
	i2c_master_recv(ts->client, &product[0], 8);

	len += snprintf(buf + len, PAGE_SIZE - len, "Firmware Version : %d.%02d \n", (verbuf[2]&0x80?1:0), verbuf[2]&0x7F);
	len += snprintf(buf + len, PAGE_SIZE - len, "Boot:0x%X Core:0x%X Config:0x%X \n", verbuf[0], verbuf[1], verbuf[2]);
	len += snprintf(buf + len, PAGE_SIZE - len, "FW Product : %s \n", product);
	len += snprintf(buf + len, PAGE_SIZE - len, "=======RESULT========\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "Channel Status : %s\n", ts->pdata->self_diagnostic[0] == 1 ? "Pass" : "Fail");
	len += snprintf(buf + len, PAGE_SIZE - len, "Raw Data : %s\n", ts->pdata->self_diagnostic[1] == 1 ? "Pass" : "Fail");
	//len += snprintf(buf + len, PAGE_SIZE - len, "Jitter : %s\n", ts->pdata->self_diagnostic[2] == 1 ? "Pass" : "Fail");

	return len;
}
static DEVICE_ATTR(sd,	S_IRUGO | S_IWUSR, mms136_self_diagnostic_show, NULL);

#ifdef SENSING_TEST
static DEVICE_ATTR(sensing_test, S_IRUGO | S_IWUSR, NULL, mms136_sensing_test_store);
#endif
static struct attribute *lge_touch_attributes[] = {
	&dev_attr_sd.attr,
	&dev_attr_status.attr,
	&dev_attr_version.attr,
	&dev_attr_testmode_ver.attr,
	&dev_attr_fw_upgrade.attr,
	&dev_attr_power_control.attr,
	&dev_attr_fx_control.attr,
	&dev_attr_irq_control.attr,
	&dev_attr_reg_control.attr,
	&dev_attr_chstatus.attr,
	&dev_attr_jitter.attr,
	&dev_attr_rawdata.attr,
	&dev_attr_delta.attr,
	&dev_attr_edge_expand.attr,
	&dev_attr_active_mode.attr,
#ifdef SENSING_TEST
	&dev_attr_sensing_test.attr,
#endif
#if defined(TOUCH_INT_SLEEP_COUNT_MEASURE)
	&dev_attr_misc.attr,
#endif
#if defined(TOUCH_CE_UNLIMITED_ONOFF_TEST)
	&dev_attr_ce_test.attr,
#endif
	NULL,
};

static const struct attribute_group lge_touch_attr_group = {
	.attrs = lge_touch_attributes,
};

static int melfas_power_on(struct melfas_ts_data *ts, bool on)
{
	int retval = 0;

	mutex_lock(&ts->lock);

//W6 Rev.A temp code. Rev.B will use the SYNAPTIC. it will be removed.
#if defined(CONFIG_MACH_MSM8X10_W6)
	{
		static int init = 0;
		if (!init) {
			gpio_request(75, "touch_ldo");
			gpio_request(1, "touch_en");
			init = 1;
		}
	}
#endif

	if (on == false)
		goto power_off;

	if (ts->poweron) {
		mutex_unlock(&ts->lock);
		return 0;
	}

	ts->poweron = 1;

	TOUCH_INFO_MSG("Power on \n");

//W6 Rev.A temp code. Rev.B will use the SYNAPTIC. it will be removed.
#if defined(CONFIG_MACH_MSM8X10_W6)
	gpio_direction_output(75, 1);
	gpio_direction_output(1, 1);
#endif

	if (ts->pdata->gpio_ce && gpio_is_valid(ts->pdata->gpio_ce)) {
		gpio_direction_output(ts->pdata->gpio_ce, 1);
	}

	if(ts->vdd) {
		retval = regulator_enable(ts->vdd);
		if (retval < 0) {
			TOUCH_INFO_MSG("regulator_enable(vdd) failed retval = %d\n", retval);
		}
	}

	if(ts->vcc_i2c) {
		retval = regulator_enable(ts->vcc_i2c);
		if (retval < 0) {
			TOUCH_INFO_MSG("regulator_enable(i2c) failed retval = %d\n", retval);
		}
	}

	goto exit;

power_off :
	if (ts->poweron == 0) {
		mutex_unlock(&ts->lock);
		return 0;
	}

	ts->poweron = 0;

	TOUCH_INFO_MSG("Power off \n");

//W6 Rev.A temp code. Rev.B will use the SYNAPTIC. it will be removed.
#if defined(CONFIG_MACH_MSM8X10_W6)
	gpio_direction_output(75, 0);
	gpio_direction_output(1, 0);
#endif

	if (ts->pdata->gpio_ce && gpio_is_valid(ts->pdata->gpio_ce)) {
		gpio_direction_output(ts->pdata->gpio_ce, 0);
	}

	if(ts->vdd) {
		regulator_disable(ts->vdd);
	}

	if(ts->vcc_i2c) {
		regulator_disable(ts->vcc_i2c);
	}

	ts->operation_mode = OPERATION_MODE_NOT_SET;

exit :
	msleep(30);

	mutex_unlock(&ts->lock);

	return retval;
}

int melfas_poweron(char on)
{
	return melfas_power_on(gts, (bool)on);
}
EXPORT_SYMBOL(melfas_poweron);

static int melfas_regulator_configure(struct melfas_ts_data *ts, bool on)
{
	int retval = 0;

	if (on == false)
		goto hw_shutdown;

	if (ts->pdata->gpio_ce && gpio_is_valid(ts->pdata->gpio_ce)) {
		TOUCH_INFO_MSG("Set CE GPIO_%d\n", ts->pdata->gpio_ce);
		gpio_request_one(ts->pdata->gpio_ce, GPIOF_OUT_INIT_HIGH, "touch_ce");
	}

	if(!ts->pdata->gpio_ce && ts->vdd == NULL) {
		ts->vdd = regulator_get(&ts->client->dev, "vdd");
		if (IS_ERR(ts->vdd)) {
			TOUCH_INFO_MSG("Failed to get vdd regulator\n");
			return PTR_ERR(ts->vdd);
		}
	}

	if(ts->vcc_i2c == NULL) {
		ts->vcc_i2c = regulator_get(&ts->client->dev, "vcc_i2c");
		if (IS_ERR(ts->vcc_i2c)) {
			TOUCH_INFO_MSG("Failed to get i2c regulator\n");
			return PTR_ERR(ts->vcc_i2c);
		}
	}

	if(ts->vdd) {
		retval = regulator_set_voltage(ts->vdd, TOUCH_VDD_VTG_MIN_UV, TOUCH_VDD_VTG_MAX_UV);
		if (retval)
			TOUCH_INFO_MSG("regulator_set_voltage(VDD) failed retval=%d\n", retval);
	}

#if 0
	if(ts->vcc_i2c) {
		retval = regulator_set_voltage(ts->vcc_i2c, TOUCH_I2C_VTG_MIN_UV, TOUCH_I2C_VTG_MAX_UV);
		if (retval)
			TOUCH_INFO_MSG("regulator_set_voltage(I2C) failed retval=%d\n", retval);
	}
#endif

	return 0;

hw_shutdown :
	if(!ts->pdata->gpio_ce && ts->vdd) {
		regulator_put(ts->vdd);
	}
	
	if(ts->vcc_i2c) {
		regulator_put(ts->vcc_i2c);
	}
	return retval;
}

static int melfas_parse_dt(struct device *dev, struct melfas_tsi_platform_data *melfas_pdata)
{
	struct device_node *np = dev->of_node;
	struct property *prop;
	u32 temp_val, num_buttons;
	u32 button_map[MAX_NUM_OF_BUTTON];
	int rc, i;

	melfas_pdata->i2c_pull_up = of_property_read_bool(np, "melfas,i2c-pull-up");

	// x,y cordination
	rc = of_property_read_u32(np, "melfas,panel-x", &temp_val);
	if (rc && (rc != -EINVAL)) {
		return rc;
	} else {
		melfas_pdata->x_max = temp_val;
		TOUCH_INFO_MSG("DT : x_max = %d\n", melfas_pdata->x_max);
	}
	rc = of_property_read_u32(np, "melfas,panel-y", &temp_val);
	if (rc && (rc != -EINVAL)) {
		return rc;
	} else {
		melfas_pdata->y_max = temp_val;
		TOUCH_INFO_MSG("DT : y_max = %d\n", melfas_pdata->y_max);
	}
	
	rc = of_property_read_u32(np, "melfas,auto_fw_update", &melfas_pdata->auto_fw_update);
	if (rc) {
		TOUCH_INFO_MSG("DT : failed to read auto_fw_update\n");
		melfas_pdata->auto_fw_update = 0;
	}else{
		TOUCH_INFO_MSG("DT : auto_fw_update = %d \n", melfas_pdata->auto_fw_update);
	}

	rc = of_property_read_u32(np, "melfas,num-of-finger", &temp_val);
	if (rc && (rc != -EINVAL)) {
		return rc;
	} else {
		if(temp_val > MELFAS_MAX_TOUCH)
			temp_val = MELFAS_MAX_TOUCH;
		melfas_pdata->num_of_finger= (unsigned char) temp_val;
		TOUCH_INFO_MSG("DT : num_of_finger = %d\n", melfas_pdata->num_of_finger);
	}

	melfas_pdata->i2c_int_gpio = of_get_named_gpio_flags(np, "melfas,i2c_int_gpio", 0, &melfas_pdata->irq_flag); //irq_flag active low or high
	rc = of_property_read_u32(np, "melfas,i2c_sda_gpio", &melfas_pdata->i2c_sda_gpio);
	if (rc) {
		TOUCH_INFO_MSG("DT : failed to read i2c_sda_gpio\n");
		melfas_pdata->i2c_sda_gpio = 0;
	}else{
		TOUCH_INFO_MSG("DT : i2c_sda_gpio = %d \n", melfas_pdata->i2c_sda_gpio);
	}
	rc = of_property_read_u32(np, "melfas,i2c_scl_gpio", &melfas_pdata->i2c_scl_gpio);
	if (rc) {
		TOUCH_INFO_MSG("DT : failed to read i2c_scl_gpio\n");
		melfas_pdata->i2c_scl_gpio = 0;
	}else{
		TOUCH_INFO_MSG("DT : i2c_scl_gpio = %d \n", melfas_pdata->i2c_scl_gpio);
	}
	prop = of_find_property(np, "melfas,button-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);
		TOUCH_INFO_MSG("DT : num_buttons = %d\n", num_buttons);
		if (num_buttons <= MAX_NUM_OF_BUTTON) {
			rc = of_property_read_u32_array(np, "melfas,button-map", button_map, num_buttons);
			if (rc) {
				TOUCH_INFO_MSG("DT : Unable to read key codes\n");
				return rc;
			}
			for(i=0; i<num_buttons; i++) {
				melfas_pdata->button[i] = (unsigned short) button_map[i];
				TOUCH_INFO_MSG("DT : button[%d] = %s:%d \n", i, get_touch_button_string(melfas_pdata->button[i]), melfas_pdata->button[i]);
			}
			melfas_pdata->num_of_button = (unsigned char)num_buttons;
		}
	}
	
	rc = of_property_read_string(np, "melfas,product",  &melfas_pdata->product);
	if (rc && (rc != -EINVAL)) {
		TOUCH_INFO_MSG("DT : FW product error \n");
	}
	else
		TOUCH_INFO_MSG("DT : FW Product : %s \n", melfas_pdata->product);

	rc = of_property_read_string(np, "melfas,panel-spec-name",  &melfas_pdata->panel_spec_name);
	if (rc && (rc != -EINVAL)) {
		TOUCH_INFO_MSG("DT : panel-spec-name error \n");
	}
	else
		TOUCH_INFO_MSG("DT : panel-spec-name : %s \n", melfas_pdata->panel_spec_name);
	
	rc = of_property_read_string(np, "melfas,fw-image-name",  &melfas_pdata->fw_name);
	if (rc && (rc != -EINVAL)) {
		TOUCH_INFO_MSG("DT : FW fw-image-name error \n");
	}
	else
		TOUCH_INFO_MSG("DT : FW fw-image-name : %s \n", melfas_pdata->fw_name);

	rc = of_property_read_u32(np, "melfas,ce", &melfas_pdata->gpio_ce);
	if (rc)
		melfas_pdata->gpio_ce = 0;
	else
		TOUCH_INFO_MSG("DT : gpio_ce = %d \n", melfas_pdata->gpio_ce);

	rc = of_property_read_u32(np, "melfas,enalbe_active_mode", &melfas_pdata->enalbe_active_mode);
	if (rc)
		melfas_pdata->enalbe_active_mode = 0;
	else
		TOUCH_INFO_MSG("DT : active_mode_switch = %d \n", melfas_pdata->enalbe_active_mode);

	return 0;
	
}

static int melfas_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct melfas_ts_data *ts=NULL;
	struct melfas_tsi_platform_data *platform_data=NULL;
	const struct firmware *fw = NULL;
	int ret = 0, i;
	int upgrade_fuc;
	int retires = 3;

	irq_flag = 0;
	
	for(i=0; i<20; i++)
		init_values[i] = 1;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		TOUCH_INFO_MSG("failed to i2c functionality check\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(struct melfas_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		TOUCH_INFO_MSG("failed to allocate melfas_ts_data\n");
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	if(client->dev.of_node) {
		platform_data = devm_kzalloc(&client->dev, sizeof(*platform_data), GFP_KERNEL);
		if(!platform_data) {
			TOUCH_INFO_MSG("Fail to allocate memory\n");
			return -ENOMEM;
		}

		ret = melfas_parse_dt(&client->dev, platform_data);
		if(ret) {
			TOUCH_INFO_MSG("device tree parsing failed \n");
			return ret;
		}
	} else {
		platform_data = client->dev.platform_data;
	}
	
	ts->info = kzalloc(sizeof(struct mms_fw_info), GFP_KERNEL);

	ts->pannel_on = 1;
	ts->pdata = platform_data;
	ts->client = client;
	i2c_set_clientdata(client, ts);
	gts = ts;

	mutex_init(&ts->lock);

	ret = melfas_regulator_configure(ts, true);
	if (ret < 0) {
		goto err_reg_configure;
	}

	ret = melfas_power_on(ts, true);
	if (ret < 0) {
		goto err_power_device;
	}

	mms_set_active_mode(ts);
	melfas_get_dev_info(ts);

	ts->input_dev = input_allocate_device();
	if (!ts->input_dev) {
		TOUCH_INFO_MSG("Not enough memory, input allocate device Failed\n");
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = DRIVER_NAME;
	ts->input_dev->dev.init_name = LGE_TOUCH_NAME;
	ts->input_dev->evbit[0] = BIT_MASK(EV_ABS) | BIT_MASK(EV_KEY);

	for (i = 0; i < ts->pdata->num_of_button; i++) {
		ts->input_dev->keybit[BIT_WORD(ts->pdata->button[i])] |= BIT_MASK(ts->pdata->button[i]);
	}
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0,  ts->pdata->x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0,  ts->pdata->y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 40, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, MELFAS_MAX_TOUCH - 1, 0, 0);

	ret = input_register_device(ts->input_dev);
	if (ret) {
		TOUCH_INFO_MSG("Failed to register input device\n");
		ret = -ENOMEM;
		goto err_input_register_device_failed;
	}

	input_set_drvdata(ts->input_dev, ts);

#if defined(CONFIG_FB)
	ts->fb_notif.notifier_call = fb_notifier_callback;
	if((ret= fb_register_client(&ts->fb_notif)))
		TOUCH_INFO_MSG("Unable to register fb_notifier: %d\n", ret);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
		ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
		ts->early_suspend.suspend = melfas_ts_early_suspend;
		ts->early_suspend.resume = melfas_ts_late_resume;
		register_early_suspend(&ts->early_suspend);
#endif

	if(ts->pdata->auto_fw_update && ts->pdata->fw_name != NULL)
	{
		TOUCH_INFO_MSG("Auto FW update. fw = %s \n", ts->pdata->fw_name);
		if(request_firmware(&fw, ts->pdata->fw_name , &ts->client->dev) >= 0) {
			power_lock(POWER_FW_UP_LOCK);
			do{
				upgrade_fuc = touch_fw_compare(ts, fw);

				if(upgrade_fuc == 1){
					if(mms_flash_fw_file_isc(ts->client, ts->pdata, ts->info) < 0){
						melfas_power_on(ts, false);
						melfas_power_on(ts, true);
						mms_flash_fw_file_isp(ts->client, ts->pdata, fw);
					}
				}else if(upgrade_fuc == 2)
					mms_flash_fw_file_isp(ts->client, ts->pdata, fw);

				if(upgrade_fuc < 0){
					melfas_power_on(ts, false);
					melfas_power_on(ts, true);
				}

			}while(upgrade_fuc < 0 && --retires);

			if (!retires){
				TOUCH_INFO_MSG("failed to flash firmware after retires, GO TO ISP\n");
				mms_flash_fw_file_isp(ts->client, ts->pdata, fw);
			}

			if (fw) {
				TOUCH_INFO_MSG("release_firmware \n");
				release_firmware(fw);
			}

			power_unlock(POWER_FW_UP_LOCK);
			
			melfas_power_on(ts, false);
			melfas_power_on(ts, true);
		} else{
			TOUCH_INFO_MSG("Fail to request firmware. \n");
		}

		mms_set_active_mode(ts);
		melfas_get_dev_info(ts);
	}
	
	if (ts->client->irq) {
		TOUCH_INFO_MSG("trying to request irq: %s-%d\n", ts->client->name, ts->client->irq);
#if defined(TOUCH_INT_SLEEP_COUNT_MEASURE)
		ret = request_threaded_irq(client->irq, NULL, melfas_ts_irq_handler, IRQF_ONESHOT | IRQF_TRIGGER_FALLING, ts->client->name, ts);
#else
		ret = request_threaded_irq(client->irq, NULL, melfas_ts_irq_handler, IRQF_ONESHOT | IRQF_TRIGGER_LOW, ts->client->name, ts);
#endif
		if (ret > 0) {
			TOUCH_INFO_MSG("Can't allocate irq %d, ret %d\n", ts->client->irq, ret);
			ret = -EBUSY;
			goto err_request_irq;
		}
	}

	ret = sysfs_create_group(&ts->input_dev->dev.kobj, &lge_touch_attr_group);

	for (i = 0; i < MELFAS_MAX_TOUCH; i++) {
		g_Mtouch_info[i].btn_touch = -1;
		g_Mtouch_info[i].status = 0;
		tmp_flag[i] = 1;
	}

	TOUCH_INFO_MSG("start touch name: %s, irq: %d\n", ts->client->name, ts->client->irq);
	return 0;

err_request_irq:
	TOUCH_INFO_MSG("err_request_irq failed\n");
	free_irq(client->irq, ts);
err_input_register_device_failed:
	TOUCH_INFO_MSG("err_input_register_device failed\n");
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
	TOUCH_INFO_MSG("err_input_dev_alloc failed\n");
err_alloc_data_failed:
	TOUCH_INFO_MSG("err_alloc_data failed_\n");
err_check_functionality_failed:
	TOUCH_INFO_MSG("err_check_functionality failed_\n");
err_power_device:
	if (ts)
		melfas_regulator_configure(ts, false);
err_reg_configure:
	if (ts) {
		input_free_device(ts->input_dev);
		ts->input_dev = NULL;
	}

	return ret;
}

static int melfas_ts_remove(struct i2c_client *client)
{
	struct melfas_ts_data *ts = i2c_get_clientdata(client);
#if defined(CONFIG_FB)
	if (fb_unregister_client(&ts->fb_notif))
		TOUCH_INFO_MSG("Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts->early_suspend);
#endif
	free_irq(client->irq, ts);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static void release_all_fingers(struct melfas_ts_data *ts)
{
	int i;
	for (i = 0; i < ts->pdata->num_of_finger; i++) {
		if (g_Mtouch_info[i].btn_touch < 0)
			continue;

		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, g_Mtouch_info[i].posX);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, g_Mtouch_info[i].posY);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, g_Mtouch_info[i].width);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
		input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
		input_mt_sync(ts->input_dev);

		g_Mtouch_info[i].posX = 0;
		g_Mtouch_info[i].posY = 0;
		g_Mtouch_info[i].strength = 0;
		g_Mtouch_info[i].btn_touch = -1;
		g_Mtouch_info[i].status = 0;
		tmp_flag[i] = 1;
	}
	input_sync(ts->input_dev);
	touch_prestate = 0;
	touched_count = 0;
}

static void release_all_keys(struct melfas_ts_data *ts)
{
	int i;

	for (i = 0; i < MELFAS_MAX_BTN; i++) {
		if (g_btn_info[i].status <= 0)
			continue;
		input_report_key(ts->input_dev, g_btn_info[i].key_code, RELEASE_KEY);
	}
	btn_prestate = RELEASE_KEY;
	input_sync(ts->input_dev);
}

static void melfas_ts_sw_reset(struct melfas_ts_data *ts)
{
	TOUCH_INFO_MSG("%s\n", __func__);

	release_all_fingers(ts);
	release_all_keys(ts);
	melfas_power_on(ts, false);
	melfas_power_on(ts, true);
	mms_set_active_mode(ts);
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct melfas_ts_data *ts = container_of(self, struct melfas_ts_data, fb_notif);
	if(evdata && evdata->data && event == FB_EVENT_BLANK && ts && ts->client) {
		blank = evdata->data;
		if(*blank == FB_BLANK_UNBLANK) {
			melfas_ts_resume(ts);
		} else if (*blank == FB_BLANK_POWERDOWN) {
			melfas_ts_suspend(ts);
		}
	}
	return 0;
}

#elif defined (CONFIG_HAS_EARLYSUSPEND)
static void melfas_ts_early_suspend(struct early_suspend *h)
{
	struct melfas_ts_data *ts;
	ts = container_of(h, struct melfas_ts_data, early_suspend);
	melfas_ts_suspend_func(ts);
}

static void melfas_ts_late_resume(struct early_suspend *h)
{
	struct melfas_ts_data *ts;
	ts = container_of(h, struct melfas_ts_data, early_suspend);
	melfas_ts_resume_func(ts);
}
#endif

static void melfas_ts_suspend(struct melfas_ts_data *ts)
{
	ts->pannel_on = 0;

#if defined(TOUCH_INT_SLEEP_COUNT_MEASURE)
	if (bTouch_suspended == 0) {
		bTouch_suspended = 1;
	}
#endif

	if(power_block)
		return;

#if defined(TOUCH_INT_SLEEP_COUNT_MEASURE)
	if (!touch_int_counter_started) {
		disable_irq(ts->client->irq);
	}
#else
	if(irq_flag == 0){
		disable_irq(ts->client->irq);
		irq_flag = 1;
	}
#endif

	melfas_power_on(ts, false);

	release_all_fingers(ts);
	release_all_keys(ts);
}

static void melfas_ts_resume(struct melfas_ts_data *ts)
{
	ts->pannel_on = 1;

#if defined(TOUCH_INT_SLEEP_COUNT_MEASURE)
	if (bTouch_suspended == 1) {
		bTouch_suspended = 0;
	}
#endif

	if(power_block)
		return;

	melfas_power_on(ts, true);

	if (ts->pdata->enalbe_active_mode) {
		mms_set_active_mode(ts);
		melfas_ts_event_handler(ts);
		release_all_fingers(ts);
		release_all_keys(ts);
	}

#if defined(TOUCH_INT_SLEEP_COUNT_MEASURE)
	mdelay(100);
	melfas_ts_event_handler(ts);
	release_all_fingers(ts);
	release_all_keys(ts);
	if (!touch_int_counter_started) {
		enable_irq(ts->client->irq);
	}
#else
	if(irq_flag == 1){
		enable_irq(ts->client->irq);
		irq_flag = 0;
	}
#endif

	ts->ic_error_cnt = 0;
}

#if (!defined(CONFIG_HAS_EARLYSUSPEND) && !defined(CONFIG_FB))
static void melfas_ts_suspend_func(struct melfas_ts_data *ts)
{
	melfas_power_on(ts, false);

	/* move release timing */
	release_all_fingers(ts);
	release_all_keys(ts);
}

static void melfas_ts_resume_func(struct melfas_ts_data *ts)
{
	melfas_power_on(ts, true);
}
#endif

static const struct i2c_device_id melfas_ts_id[] = {
	{ DRIVER_NAME, 0 },
	{ }
};

#ifdef CONFIG_OF
static struct of_device_id melfas_match_table[] = {
	{ .compatible = "melfas,mms136",},
	{},
};
#else
#define melfas_match_table NULL
#endif

static struct i2c_driver melfas_ts_driver = {
	.driver		= {
		.name	= "touch_melfas",
		.owner  = THIS_MODULE,
		.of_match_table = melfas_match_table,
	},
	.id_table		= melfas_ts_id,
	.probe		= melfas_ts_probe,
	.remove		= __devexit_p (melfas_ts_remove),
#if (!defined(CONFIG_HAS_EARLYSUSPEND) && !defined(CONFIG_FB))
	.suspend		= melfas_ts_suspend,
	.resume		= melfas_ts_resume,
#endif
};

static int __devinit melfas_ts_init(void)
{
	return i2c_add_driver(&melfas_ts_driver);
}

static void __exit melfas_ts_exit(void)
{
	i2c_del_driver(&melfas_ts_driver);
}

MODULE_DESCRIPTION("Driver for Melfas MTSI Touchscreen Controller");
MODULE_AUTHOR("MinSang, Kim <kimms@melfas.com>");
MODULE_LICENSE("GPL");

module_init(melfas_ts_init);
module_exit(melfas_ts_exit);
