/*
 *  Copyright (C) 2010,Imagis Technology Co. Ltd. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/mutex.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#if 0
#include CONFIG_LGE_BOARD_HEADER_FILE
#endif
#include "ist30xx.h"
#include "ist30xx_update.h"

#if IST30XX_DEBUG
#include "ist30xx_misc.h"
#endif

#if IST30XX_TRACKING_MODE
#include "ist30xx_tracking.h"
#endif

#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 0, 0))
#include <linux/input/mt.h>
#endif

#define IRQ_THREAD_WORK_QUEUE   (0)

#define MAX_ERR_CNT             (100)

/* sukkyoon.hong@lge.com */
#define ON			1
#define OFF			0
/* sukkyoon.hong@lge.com */
#if IST30XX_USE_KEY
#if 0
int ist30xx_key_code[] = { 0, KEY_BACK, KEY_HOMEPAGE, KEY_MENU, KEY_RECENT };
#else
int ist30xx_key_code[] = { 0, KEY_BACK, KEY_HOMEPAGE, KEY_MENU};
#endif
#endif

DEFINE_MUTEX(ist30xx_mutex);
#if IST30XX_DETECT_TA
static int ist30xx_ta_status = -1;
#endif

static bool ist30xx_initialized = 0;
struct ist30xx_data *ts_data;
#if IRQ_THREAD_WORK_QUEUE
static struct work_struct work_irq_thread;
#endif
static struct delayed_work work_reset_check;
#if IST30XX_INTERNAL_BIN && IST30XX_UPDATE_BY_WORKQUEUE
static struct delayed_work work_fw_update;
#endif

static void clear_input_data(struct ist30xx_data *data);


#if IST30XX_EVENT_MODE
bool get_event_mode = true;

static struct timer_list idle_timer;
static struct timespec t_event, t_current;      // ns
#define EVENT_TIMER_INTERVAL     (HZ / 2)       // 0.5sec

# if IST30XX_NOISE_MODE
# define IST30XX_IDLE_STATUS     (0x1D4E0000)
# define IDLE_ALGORITHM_MODE     (1U << 15)
# endif // IST30XX_NOISE_MODE

#endif  // IST30XX_EVENT_MODE

#if IST30XX_DEBUG
extern TSP_INFO ist30xx_tsp_info;
extern TKEY_INFO ist30xx_tkey_info;
#endif

/* sukkyoon.hong@lge.com */
/* struct ist30xx_ts_device {
	struct i2c_client *client;
	int (*power)(unsigned char onoff);
	int num_irq;
	int scl_gpio;
	int sda_gpio;
};
struct ist30xx_ts_device ist30xx_ts_dev; */
/* sukkyoon.hong@lge.com */

/* sukkyoon.hong@lge.com */
#if 0
void Send_Touch(unsigned int x, unsigned int y)
{
	/* Press */
	input_mt_slot(ts_data->input_dev, 0);
	input_mt_report_slot_state(ts_data->input_dev, MT_TOOL_FINGER, true);
	input_report_abs(ts_data->input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(ts_data->input_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(ts_data->input_dev, ABS_MT_TOUCH_MAJOR, 1);
	input_sync(ts_data->input_dev);
	/* Release */
	input_mt_slot(ts_data->input_dev, 0);
	input_mt_report_slot_state(ts_data->input_dev, MT_TOOL_FINGER, false);
	input_sync(ts_data->input_dev);
}
EXPORT_SYMBOL(Send_Touch);
#endif
/* sukkyoon.hong@lge.com */

int ist30xx_dbg_level = IST30XX_DEBUG_LEVEL;
void tsp_printk(int level, const char *fmt, ...)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36))
	struct va_format vaf;
#endif
	va_list args;
	int r;

	if (ist30xx_dbg_level < level)
		return;

	va_start(args, fmt);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36))
	vaf.fmt = fmt;
	vaf.va = &args;

	r = printk("%s %pV", IST30XX_DEBUG_TAG, &vaf);
#else
	printk(IST30XX_DEBUG_LEVEL);
	r = vprintk(fmt, args);
#endif
	va_end(args);
}


void ist30xx_disable_irq(struct ist30xx_data *data)
{
	if (data->irq_enabled) {
		disable_irq(data->client->irq);
		data->irq_enabled = 0;
	}
}

void ist30xx_enable_irq(struct ist30xx_data *data)
{
	if (!data->irq_enabled) {
		enable_irq(data->client->irq);
		msleep(50);
		data->irq_enabled = 1;
	}
}


int ist30xx_max_error_cnt = MAX_ERR_CNT;
int ist30xx_error_cnt = 0;
static void ist30xx_request_reset(void)
{
	ist30xx_error_cnt++;
	if (ist30xx_error_cnt >= ist30xx_max_error_cnt) {
		schedule_delayed_work(&work_reset_check, 0);
		tsp_info("%s()\n", __func__);
		ist30xx_error_cnt = 0;
	}
}


void ist30xx_start(struct ist30xx_data *data)
{
#if IST30XX_DETECT_TA
	if (ist30xx_ta_status > -1) {
		ist30xx_write_cmd(data->client, CMD_SET_TA_MODE, ist30xx_ta_status);

		tsp_info("%s(), ta_mode : %d\n", __func__, ist30xx_ta_status);
	}
#endif

	ist30xx_cmd_start_scan(data->client);

#if IST30XX_EVENT_MODE
	if ((data->status.update != 1) && (data->status.calib != 1))
		ktime_get_ts(&t_event);
#endif
}


int ist30xx_get_ver_info(struct ist30xx_data *data)
{
	int ret;

	data->fw.pre_ver = data->fw.ver;
	data->fw.ver = 0;

	ret = ist30xx_read_cmd(data->client, CMD_GET_CHIP_ID, &data->chip_id);
	if (ret)
		return ret;

	ret = ist30xx_read_cmd(data->client, CMD_GET_FW_VER, &data->fw.ver);
	if (ret)
		return ret;

	ret = ist30xx_read_cmd(data->client, CMD_GET_PARAM_VER, &data->param_ver);
	if (ret)
		return ret;

	tsp_info("Chip ID : %x F/W: %x Param: %x\n",
		 data->chip_id, data->fw.ver, data->param_ver);

	if ((data->chip_id != IST30XX_CHIP_ID) &&
	    (data->chip_id != IST30XXA_CHIP_ID))
		return -EPERM;

	return 0;
}


int ist30xx_init_touch_driver(struct ist30xx_data *data)
{
	int ret = 0;

	mutex_lock(&ist30xx_mutex);
	ist30xx_disable_irq(data);

	ret = ist30xx_cmd_run_device(data->client);
	if (ret)
		goto init_touch_end;

	ret = ist30xx_get_ver_info(data);
	if (ret)
		goto init_touch_end;

init_touch_end:
	ist30xx_start(data);

	ist30xx_enable_irq(data);
	mutex_unlock(&ist30xx_mutex);

	return ret;
}


#if IST30XX_DEBUG
void ist30xx_print_info(void)
{
	TSP_INFO *tsp = &ist30xx_tsp_info;
	TKEY_INFO *tkey = &ist30xx_tkey_info;

	tsp_debug("tkey enable: %d, key num: %d\n", tkey->enable, tkey->key_num);
	tsp_debug("tkey axis_ch: %d, tx: %d\n", tkey->axis_chnum, tkey->tx_line);
	tsp_debug("tkey ch_num[0]: %d, [1]: %d, [2]: %d, [3]: %d, [4]: %d\n",
		 tkey->ch_num[0], tkey->ch_num[1], tkey->ch_num[2],
		 tkey->ch_num[3], tkey->ch_num[4]);
	tsp_debug("tscn internal x,y num: %d, %d\n", tsp->intl.x, tsp->intl.y);
	tsp_debug("tscn module x,y num: %d, %d\n", tsp->mod.x, tsp->mod.y);
	tsp_debug("tscn dir.txch_y: %d, swap: %d\n", tsp->dir.txch_y, tsp->dir.swap_xy);
	tsp_debug("tscn dir.flip_x, y: %d, %d\n", tsp->dir.flip_x, tsp->dir.flip_y);
}
#endif
#define CALIB_MSG_MASK          (0xF0000FFF)
#define CALIB_MSG_VALID         (0x80000CAB)

#if 1
int ist30xx_get_info(struct ist30xx_data *data)
{
	int ret;
	u32 calib_msg;
	int retry = 0;

	ist30xx_tsp_info.finger_num = IST30XX_MAX_MT_FINGERS;
	mutex_lock(&ist30xx_mutex);
	ist30xx_disable_irq(data);

RETRY :
	ret = ist30xx_write_cmd(data->client, CMD_RUN_DEVICE, 0);

	msleep(50);

	ret = ist30xx_get_ver_info(data);
	if(ret != 0) {
		if(retry++ < 10) {
			tsp_debug("ist30xx_get_info retry : %d \n", retry);
			ist30xx_ts_reset();
			goto RETRY;
		}
	}

	ret = ist30xx_tsp_update_info();
	ret = ist30xx_tkey_update_info();

	ist30xx_print_info();

	ret = ist30xx_read_cmd(ts_data->client, CMD_GET_CALIB_RESULT, &calib_msg);
	if (ret == 0) {
		tsp_info("calib status: 0x%08x\n", calib_msg);
		if ((calib_msg & CALIB_MSG_MASK) != CALIB_MSG_VALID ||
		    CALIB_TO_STATUS(calib_msg) > 0) {
			ist30xx_calibrate(IST30XX_FW_UPDATE_RETRY);

			ist30xx_cmd_run_device(data->client);
		}
	}

	ist30xx_start(ts_data);

#if IST30XX_EVENT_MODE
	ktime_get_ts(&t_event);
#endif

	data->status.calib = 0;

	ist30xx_enable_irq(data);

	mutex_unlock(&ist30xx_mutex);

	return ret;
}

#else
int ist30xx_get_info(struct ist30xx_data *data)
{
	int ret;
	u32 calib_msg;

	ist30xx_tsp_info.finger_num = IST30XX_MAX_MT_FINGERS;
	mutex_lock(&ist30xx_mutex);
	ist30xx_disable_irq(data);

#if !(IST30XX_INTERNAL_BIN)
	ret = ist30xx_write_cmd(data->client, CMD_RUN_DEVICE, 0);
	if (ret)
		goto get_info_end;
	msleep(10);
	ret = ist30xx_get_ver_info(data);
	if (ret) goto get_info_end;
#endif  // !(IST30XX_INTERNAL_BIN)

#if IST30XX_DEBUG
# if IST30XX_INTERNAL_BIN
	ist30xx_get_tsp_info();
	ist30xx_get_tkey_info();
# else
	ret = ist30xx_tsp_update_info();
	if (ret) goto get_info_end;

	ret = ist30xx_tkey_update_info();
	if (ret) goto get_info_end;
# endif

	ist30xx_print_info();
#endif  // IST30XX_DEBUG

	ret = ist30xx_read_cmd(ts_data->client, CMD_GET_CALIB_RESULT, &calib_msg);
	if (ret == 0) {
		tsp_info("calib status: 0x%08x\n", calib_msg);
		if ((calib_msg & CALIB_MSG_MASK) != CALIB_MSG_VALID ||
		    CALIB_TO_STATUS(calib_msg) > 0) {
			ist30xx_calibrate(IST30XX_FW_UPDATE_RETRY);

			ist30xx_cmd_run_device(data->client);
		}
	}

	ist30xx_start(ts_data);

#if IST30XX_EVENT_MODE
	ktime_get_ts(&t_event);
#endif

	data->status.calib = 0;

#if !(IST30XX_INTERNAL_BIN)
get_info_end:
#endif
	if (ret == 0)
		ist30xx_enable_irq(data);
	else
		printk("[ TSP ] ist30xx_get_info return value : %d\n", ret);
	mutex_unlock(&ist30xx_mutex);

	return ret;
}
#endif

#define PRESS_MSG_MASK          (0x01)
#define MULTI_MSG_MASK          (0x02)
#define PRESS_MSG_KEY           (0x6)

#define TOUCH_DOWN_MESSAGE      ("Touch down")
#define TOUCH_UP_MESSAGE        ("Touch up  ")
#define TOUCH_MOVE_MESSAGE      ("          ")
bool tsp_touched[IST30XX_MAX_MT_FINGERS] = { 0, };

void print_tsp_event(finger_info *finger)
{
	int idx = finger->bit_field.id - 1;
	int press = finger->bit_field.udmg & PRESS_MSG_MASK;
	if ( idx < 0 ) {
		tsp_err("finger idx err! idx value : %d\n", idx);
		return;
	}

	if (press == PRESS_MSG_MASK) {
		if (tsp_touched[idx] == 0) { // touch down
			tsp_info("%s - %d (%d, %d)\n",
				 TOUCH_DOWN_MESSAGE, finger->bit_field.id,
				 finger->bit_field.x, finger->bit_field.y);
			tsp_touched[idx] = 1;
		} else {                    // touch move
			tsp_debug("%s   %d (%d, %d)\n",
				  TOUCH_MOVE_MESSAGE, finger->bit_field.id,
				  finger->bit_field.x, finger->bit_field.y);
		}
	} else {
		if (tsp_touched[idx] == 1) { // touch up
			tsp_info("%s - %d (%d, %d)\n",
				 TOUCH_UP_MESSAGE, finger->bit_field.id,
				 finger->bit_field.x, finger->bit_field.y);
			tsp_touched[idx] = 0;
		}
	}
}


static void release_finger(finger_info *finger)
{
#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 0, 0))
	input_mt_slot(ts_data->input_dev, finger->bit_field.id - 1);
	input_mt_report_slot_state(ts_data->input_dev, MT_TOOL_FINGER, false);
#else
	input_report_abs(ts_data->input_dev, ABS_MT_POSITION_X, finger->bit_field.x);
	input_report_abs(ts_data->input_dev, ABS_MT_POSITION_Y, finger->bit_field.y);
	input_report_abs(ts_data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	input_report_abs(ts_data->input_dev, ABS_MT_WIDTH_MAJOR, 0);
	input_mt_sync(ts_data->input_dev);
#endif  // (LINUX_VERSION_CODE > KERNEL_VERSION(3, 0, 0))

	tsp_info("%s %d(%d, %d)\n", __func__,
		 finger->bit_field.id, finger->bit_field.x, finger->bit_field.y);

	finger->bit_field.udmg &= ~(PRESS_MSG_MASK);
	//print_tsp_event(finger);

	finger->bit_field.id = 0;

	input_sync(ts_data->input_dev);
}

#define CANCEL_KEY  (0xff)
#define RELEASE_KEY (0)
static void release_key(finger_info *key, u8 key_status)
{
	int id = key->bit_field.id;

	input_report_key(ts_data->input_dev, ist30xx_key_code[id], key_status);

	tsp_debug("%s key%d, event: %d, status: %d\n", __func__,
            id, key->bit_field.w, key_status);

	key->bit_field.id = 0;

	input_sync(ts_data->input_dev);
}

#ifdef CONFIG_MACH_MSM7X27A_L4II_CDMA
void uts_input_key( unsigned int code, int value)
{
	input_report_key(ts_data->input_dev, code, value);
	input_sync(ts_data->input_dev);
}
#endif

static void clear_input_data(struct ist30xx_data *data)
{
	int i;
	finger_info *fingers = (finger_info *)data->prev_fingers;
    finger_info *keys = (finger_info *)data->prev_keys;

	for (i = 0; i < data->num_fingers; i++) {
		if (fingers[i].bit_field.id == 0)
			continue;

		if (fingers[i].bit_field.udmg & PRESS_MSG_MASK)
			release_finger(&fingers[i]);
	}

    for (i = 0; i < data->num_keys; i++) {
		if (keys[i].bit_field.id == 0)
			continue;

		if (keys[i].bit_field.w == PRESS_MSG_KEY)
			release_key(&keys[i], RELEASE_KEY);
	}
}

static int check_report_data(struct ist30xx_data *data, int finger_counts, int key_counts)
{
	int i, j;
	bool valid_id;
	finger_info *fingers = (finger_info *)data->fingers;
	finger_info *prev_fingers = (finger_info *)data->prev_fingers;

	/* current finger info */
	for (i = 0; i < finger_counts; i++) {
		if ((fingers[i].bit_field.id == 0) ||
		    (fingers[i].bit_field.id > ist30xx_tsp_info.finger_num) ||
		    (fingers[i].bit_field.x > IST30XX_MAX_X) ||
		    (fingers[i].bit_field.y > IST30XX_MAX_Y)) {
			tsp_warn("Invalid touch data - %d: %d(%d, %d)\n", i,
				 fingers[i].bit_field.id,
				 fingers[i].bit_field.x,
				 fingers[i].bit_field.y);

			fingers[i].bit_field.id = 0;
			return -EPERM;
		}
	}

	/* previous finger info */
	if (data->num_fingers >= finger_counts) {
		for (i = 0; i < ist30xx_tsp_info.finger_num; i++) { // prev_fingers
			if (prev_fingers[i].bit_field.id != 0 &&
			    (prev_fingers[i].bit_field.udmg & PRESS_MSG_MASK)) {
				valid_id = false;
				for (j = 0; j < ist30xx_tsp_info.finger_num; j++) { // fingers
					if ((prev_fingers[i].bit_field.id) ==
					    (fingers[j].bit_field.id)) {
						valid_id = true;
						break;
					}
				}
				if (valid_id == false)
					release_finger(&prev_fingers[i]);
			}
		}
	}

	return 0;
}

bool finger_on_screen(void)
{
	int i;

	for (i = 0; i < IST30XX_MAX_MT_FINGERS; i++)
		if (tsp_touched[i]) return true;

	return false;
}

int key_press = 0;
int key_id = 0;
static void report_input_data(struct ist30xx_data *data, int finger_counts, int key_counts)
{
	int i, press, count;
	finger_info *fingers = (finger_info *)data->fingers;

	memset(data->prev_fingers, 0, sizeof(data->prev_fingers));

#if 1   // for LGE scenario
    if (finger_counts) {
        for (i = 0; i < 5; i++) {
            //tsp_debug("finger[%d]: %08x\n", i, data->prev_keys[i].full_field);
            if (data->prev_keys[i].bit_field.id ==0)
                continue;

            if (data->prev_keys[i].bit_field.w == PRESS_MSG_KEY) {
                tsp_warn("key cancel: %08x\n", data->prev_keys[i].full_field);
                release_key(&data->prev_keys[i], CANCEL_KEY);
            }
        }
    }
#endif
	for (i = 0, count = 0; i < finger_counts; i++) {
		press = fingers[i].bit_field.udmg & PRESS_MSG_MASK;

		//print_tsp_event(&fingers[i]);

#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 0, 0))
		input_mt_slot(data->input_dev, fingers[i].bit_field.id - 1);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER,
					   (press ? true : false));
		if (press) {
			input_report_abs(data->input_dev, ABS_MT_POSITION_X,
					 fingers[i].bit_field.x);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
					 fingers[i].bit_field.y);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR,
					 fingers[i].bit_field.w);
		}
		printk("[ TSP ] id : %2d, press : %2d , x : %3d, y :%3d,  width : %3d\n", fingers[i].bit_field.id - 1, press, fingers[i].bit_field.x, fingers[i].bit_field.y, fingers[i].bit_field.w);
#else
		input_report_abs(data->input_dev, ABS_MT_POSITION_X,
				 fingers[i].bit_field.x);
		input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
				 fingers[i].bit_field.y);
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR,
				 press);
		input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR,
				 fingers[i].bit_field.w);
		input_mt_sync(data->input_dev);
#endif          // (LINUX_VERSION_CODE > KERNEL_VERSION(3, 0, 0)
		data->prev_fingers[i] = fingers[i];
		count++;
	}

#if IST30XX_USE_KEY
	for (i = finger_counts; i < finger_counts + key_counts; i++) {
		key_id = fingers[i].bit_field.id;

		key_press = (fingers[i].bit_field.w == PRESS_MSG_KEY) ? 1 : 0;

		if (finger_on_screen()) {	// Ignore touch key
			tsp_warn("Ignore key id: %d\n", key_id);
			continue;
		}

		tsp_debug("key(%08x) id: %d, press: %d, sensitivity: %d\n",
			  fingers[i].full_field, key_id, key_press, fingers[i].bit_field.y);

		input_report_key(data->input_dev, ist30xx_key_code[key_id], key_press);

		data->prev_keys[key_id-1] = fingers[i];

		count++;
	}
#endif  // IST30XX_USE_KEY

	if (count > 0)
		input_sync(data->input_dev);

	data->num_fingers = finger_counts;
	ist30xx_error_cnt = 0;
}

/*
 * CMD : CMD_GET_COORD
 *               [31:30]  [29:26]  [25:16]  [15:10]  [9:0]
 *   Multi(1st)  UDMG     Rsvd.    NumOfKey Rsvd.    NumOfFinger
 *    Single &   UDMG     ID       X        Area     Y
 *   Multi(2nd)
 *
 *   UDMG [31] 0/1 : single/multi
 *   UDMG [30] 0/1 : unpress/press
 */
#if IRQ_THREAD_WORK_QUEUE
static void irq_thread_func(struct work_struct *work)
{
	int i, ret;
	int key_cnt, finger_cnt, read_cnt;
	struct ist30xx_data *data = ts_data;
	u32 msg[IST30XX_MAX_MT_FINGERS];
	bool unknown_idle = false;

#if IST30XX_TRACKING_MODE
	u32 ms;
#endif

	if (!data->irq_enabled)
		goto irq_end;

	memset(msg, 0, sizeof(msg));

	ret = ist30xx_get_position(data->client, msg, 1);
	if (ret)
		goto irq_err;

	tsp_verb("intr msg: 0x%08x\n", *msg);

	if (msg[0] == 0)
		goto irq_end;

#if IST30XX_EVENT_MODE
	if ((data->status.update != 1) && (data->status.calib != 1))
		ktime_get_ts(&t_event);
#endif

#if IST30XX_TRACKING_MODE
	ms = t_event.tv_sec * 1000 + t_event.tv_nsec / 1000000;
    ist30xx_put_track(ms, msg[0]);
#endif

#if IST30XX_NOISE_MODE
	if (get_event_mode) {
		if ((msg[0] & 0xFFFF0000) == IST30XX_IDLE_STATUS) {
			if (msg[0] & IDLE_ALGORITHM_MODE)
				goto irq_end;

			for (i = 0; i < IST30XX_MAX_MT_FINGERS; i++) {
				if (data->prev_fingers[i].bit_field.id == 0)
					continue;

				if (data->prev_fingers[i].bit_field.udmg & PRESS_MSG_MASK) {
					tsp_warn("prev_fingers: %08x\n",
						 data->prev_fingers[i].full_field);
					release_finger(&data->prev_fingers[i]);
					unknown_idle = true;
				}
			}

			for (i = 0; i < IST30XX_MAX_MT_FINGERS; i++) {
				if (data->prev_keys[i].bit_field.id == 0)
					continue;

				if (data->prev_keys[i].bit_field.w == PRESS_MSG_KEY) {
					tsp_warn("prev_keys: %08x\n",
						 data->prev_keys[i].full_field);
					release_key(&data->prev_keys[i], RELEASE_KEY);
					unknown_idle = true;
				}
			}

			if (unknown_idle) {
				schedule_delayed_work(&work_reset_check, 0);
				tsp_warn("Find unknown pressure\n");
			}

			goto irq_end;
		}
	}
#endif  // IST30XX_NOISE_MODE

	if ((msg[0] & CALIB_MSG_MASK) == CALIB_MSG_VALID) {
		data->status.calib_msg = msg[0];
		tsp_info("calib status: 0x%08x\n", data->status.calib_msg);
		goto irq_end;
	}

	for (i = 0; i < IST30XX_MAX_MT_FINGERS; i++)
		data->fingers[i].full_field = 0;

	key_cnt = 0;
	finger_cnt = 1;
	read_cnt = 1;
	data->fingers[0].full_field = msg[0];

	if (data->fingers[0].bit_field.udmg & MULTI_MSG_MASK) {
		key_cnt = data->fingers[0].bit_field.x;
		finger_cnt = data->fingers[0].bit_field.y;
		read_cnt = finger_cnt + key_cnt;

		if (finger_cnt > ist30xx_tsp_info.finger_num ||
		    key_cnt > ist30xx_tkey_info.key_num) {
			tsp_warn("Invalid touch count - finger: %d(%d), key: %d(%d)\n",
				 finger_cnt, ist30xx_tsp_info.finger_num,
				 key_cnt, ist30xx_tkey_info.key_num);
			goto irq_err;
		}

#if I2C_BURST_MODE
		ret = ist30xx_get_position(data->client, msg, read_cnt);
		if (ret)
			goto irq_err;

		for (i = 0; i < read_cnt; i++)
			data->fingers[i].full_field = msg[i];
#else
		for (i = 0; i < read_cnt; i++) {
			ret = ist30xx_get_position(data->client, &msg[i], 1);
			if (ret)
				goto irq_err;

			data->fingers[i].full_field = msg[i];
		}
#endif          // I2C_BURST_MODE

#if IST30XX_TRACKING_MODE
		for (i = 0; i < read_cnt; i++)
			ist30xx_put_track(ms, msg[i]);
#endif
	}

	if (check_report_data(data, finger_cnt, key_cnt))
		goto irq_end;

	if (read_cnt > 0)
		report_input_data(data, finger_cnt, key_cnt);

	goto irq_end;

irq_err:
	tsp_err("intr msg[0]: 0x%08x, ret: %d\n", msg[0], ret);
	ist30xx_request_reset();
irq_end:
	enable_irq(data->client->irq);
	return;
}
#endif // IRQ_THREAD_WORK_QUEUE

#if IRQ_THREAD_WORK_QUEUE
static irqreturn_t ist30xx_irq_thread(int irq, void *ptr)
{
	ts_data = ptr;

	printk("[ TSP ] %s\n", __func__);

	disable_irq_nosync(ts_data->client->irq);
	schedule_work(&work_irq_thread);

	return IRQ_HANDLED;
}
#else // IRQ_THREAD_WORK_QUEUE
static irqreturn_t ist30xx_irq_thread(int irq, void *ptr)
{
	int i, ret;
	int key_cnt, finger_cnt, read_cnt;
	struct ist30xx_data *data = ptr;
	u32 msg[IST30XX_MAX_MT_FINGERS];
	bool unknown_idle = false;

#if IST30XX_TRACKING_MODE
	u32 ms;
#endif

	if (!data->irq_enabled)
		return IRQ_HANDLED;

	memset(msg, 0, sizeof(msg));

	ret = ist30xx_get_position(data->client, msg, 1);
	if (ret)
		goto irq_err;

	tsp_verb("intr msg: 0x%08x\n", *msg);

	if (msg[0] == 0)
		return IRQ_HANDLED;

#if IST30XX_EVENT_MODE
	if ((data->status.update != 1) && (data->status.calib != 1))
		ktime_get_ts(&t_event);
#endif

#if IST30XX_TRACKING_MODE
	ms = t_event.tv_sec * 1000 + t_event.tv_nsec / 1000000;
    ist30xx_put_track(ms, msg[0]);
#endif

#if IST30XX_NOISE_MODE
	if (get_event_mode) {
		if ((msg[0] & 0xFFFF0000) == IST30XX_IDLE_STATUS) {
			if (msg[0] & IDLE_ALGORITHM_MODE)
				return IRQ_HANDLED;

			for (i = 0; i < IST30XX_MAX_MT_FINGERS; i++) {
				if (data->prev_fingers[i].bit_field.id == 0)
					continue;

				if (data->prev_fingers[i].bit_field.udmg & PRESS_MSG_MASK) {
					tsp_warn("prev_fingers: %08x\n",
						 data->prev_fingers[i].full_field);
					release_finger(&data->prev_fingers[i]);
					unknown_idle = true;
				}
			}

			for (i = 0; i < data->num_keys; i++) {
				if (data->prev_keys[i].bit_field.id == 0)
					continue;

				if (data->prev_keys[i].bit_field.w == PRESS_MSG_KEY) {
					tsp_warn("prev_keys: %08x\n",
						 data->prev_keys[i].full_field);
					release_key(&data->prev_keys[i], RELEASE_KEY);
					unknown_idle = true;
				}
			}

			if (unknown_idle) {
				schedule_delayed_work(&work_reset_check, 0);
				tsp_warn("Find unknown pressure\n");
			}

			return IRQ_HANDLED;
		}
	}
#endif  // IST30XX_NOISE_MODE

	if ((msg[0] & CALIB_MSG_MASK) == CALIB_MSG_VALID) {
		data->status.calib_msg = msg[0];
		tsp_info("calib status: 0x%08x\n", data->status.calib_msg);
		return IRQ_HANDLED;
	}

	for (i = 0; i < IST30XX_MAX_MT_FINGERS; i++)
		data->fingers[i].full_field = 0;

	key_cnt = 0;
	finger_cnt = 1;
	read_cnt = 1;
	data->fingers[0].full_field = msg[0];

	if (data->fingers[0].bit_field.udmg & MULTI_MSG_MASK) {
		key_cnt = data->fingers[0].bit_field.x;
		finger_cnt = data->fingers[0].bit_field.y;
		read_cnt = finger_cnt + key_cnt;

		if (finger_cnt > ist30xx_tsp_info.finger_num ||
		    key_cnt > ist30xx_tkey_info.key_num) {
			tsp_warn("Invalid touch count - finger: %d(%d), key: %d(%d)\n",
				 finger_cnt, ist30xx_tsp_info.finger_num,
				 key_cnt, ist30xx_tkey_info.key_num);
			goto irq_err;
		}

#if I2C_BURST_MODE
		ret = ist30xx_get_position(data->client, msg, read_cnt);
		if (ret)
			goto irq_err;

		for (i = 0; i < read_cnt; i++)
			data->fingers[i].full_field = msg[i];
#else
		for (i = 0; i < read_cnt; i++) {
			ret = ist30xx_get_position(data->client, &msg[i], 1);
			if (ret)
				goto irq_err;

			data->fingers[i].full_field = msg[i];
		}
#endif          // I2C_BURST_MODE

		for (i = 0; i < read_cnt; i++)
			tsp_verb("intr msg[%d]: 0x%08x\n", i, msg[i]);

#if IST30XX_TRACKING_MODE
		for (i = 0; i < read_cnt; i++)
			ist30xx_put_track(ms, msg[i]);
#endif
	}

	if (check_report_data(data, finger_cnt, key_cnt))
		return IRQ_HANDLED;

	if (read_cnt > 0)
		report_input_data(data, finger_cnt, key_cnt);

	return IRQ_HANDLED;

irq_err:
	tsp_err("intr msg[0]: 0x%08x, ret: %d\n", msg[0], ret);
	ist30xx_request_reset();
	return IRQ_HANDLED;
}
#endif // IRQ_THREAD_WORK_QUEUE

#if defined(CONFIG_FB)
static int ist30xx_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ist30xx_data *data = i2c_get_clientdata(client);

	tsp_debug("ist30xx_suspend \n");

	mutex_lock(&ist30xx_mutex);
	ist30xx_disable_irq(data);
	ist30xx_internal_suspend(data);
	clear_input_data(data);
	mutex_unlock(&ist30xx_mutex);

	return 0;
}
static int ist30xx_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ist30xx_data *data = i2c_get_clientdata(client);

	tsp_debug("ist30xx_resume \n");

	mutex_lock(&ist30xx_mutex);
	ist30xx_internal_resume(data);
	ist30xx_start(data);
	ist30xx_enable_irq(data);
	mutex_unlock(&ist30xx_mutex);

	return 0;
}

static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct ist30xx_data *ts_data = container_of(self, struct ist30xx_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
		ts_data && ts_data->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			ist30xx_resume(&(ts_data->input_dev->dev));
		else if (*blank == FB_BLANK_POWERDOWN)
			ist30xx_suspend(&(ts_data->input_dev->dev));
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#define ist30xx_suspend NULL
#define ist30xx_resume  NULL
static void ist30xx_early_suspend(struct early_suspend *h)
{
	struct ist30xx_data *data = container_of(h, struct ist30xx_data,
						 early_suspend);

	mutex_lock(&ist30xx_mutex);
	ist30xx_disable_irq(data);
	ist30xx_internal_suspend(data);
	clear_input_data(data);
	mutex_unlock(&ist30xx_mutex);
}
static void ist30xx_late_resume(struct early_suspend *h)
{
	struct ist30xx_data *data = container_of(h, struct ist30xx_data,
						 early_suspend);

	mutex_lock(&ist30xx_mutex);
	ist30xx_internal_resume(data);
	ist30xx_start(data);
	ist30xx_enable_irq(data);
	mutex_unlock(&ist30xx_mutex);
}
#endif // CONFIG_HAS_EARLYSUSPEND

/* sukkyoon.hong@lge.com */
void ist30xx_ts_reset(void)
{
	ist30xx_ts_off();
	ist30xx_ts_on();

	tsp_info("ist30xx_ts_reset\n");
}

static int ist30xx_ldo_power_on(struct ist30xx_data *ts, bool on);

int ist30xx_ts_off(void)
{
#if 1
	ist30xx_ldo_power_on(ts_data, false);
	msleep(20);
	return 0;
#else
//	struct ist30xx_ts_device *dev = NULL;
	int ret = 0;

//	dev = &ist30xx_ts_dev;

	ret = ts_data->power(OFF);

	tsp_info("ist30xx_ts_off\n");

	if(ret < 0) {
		tsp_err("ist30xx_ts_off power off failed\n");
		goto err_power_failed;
	}

	ts_data->status.power = 0;

	msleep(10);

err_power_failed:
	return ret;
#endif
}

int ist30xx_ts_on(void)
{
#if 1
	ist30xx_ldo_power_on(ts_data, true);
	msleep(50);
	return 0;
#else
//	struct ist30xx_ts_device *dev = NULL;
	int ret = 0;

//	dev = &ist30xx_ts_dev;

	ret = ts_data->power(ON);

	tsp_info("ist30xx_ts_on\n");

	if(ret < 0) {
		tsp_err("ist30xx_ts_on power on failed\n");
		goto err_power_failed;
	}

	ts_data->status.power = 1;

	msleep(30);

err_power_failed:
	return ret;
#endif
}
/* sukkyoon.hong@lge.com */

void ist30xx_set_ta_mode(bool charging)
{
#if IST30XX_DETECT_TA
	if ((ist30xx_ta_status == -1) || (charging == ist30xx_ta_status))
		return;

	ist30xx_ta_status = charging ? 1 : 0;

	tsp_info("%s(), charging = %d\n", __func__, ist30xx_ta_status);

	schedule_delayed_work(&work_reset_check, 0);
#endif
}
EXPORT_SYMBOL(ist30xx_set_ta_mode);


static void reset_work_func(struct work_struct *work)
{
	if ((ts_data == NULL) || (ts_data->client == NULL))
		return;

	tsp_info("Request reset function\n");

	if ((ts_data->status.power == 1) &&
	    (ts_data->status.update != 1) && (ts_data->status.calib != 1)) {
		mutex_lock(&ist30xx_mutex);
		ist30xx_disable_irq(ts_data);

		clear_input_data(ts_data);

		ist30xx_cmd_run_device(ts_data->client);

		ist30xx_start(ts_data);

		ist30xx_enable_irq(ts_data);
		mutex_unlock(&ist30xx_mutex);
	}
}

#if IST30XX_INTERNAL_BIN && IST30XX_UPDATE_BY_WORKQUEUE
static void fw_update_func(struct work_struct *work)
{
	if ((ts_data == NULL) || (ts_data->client == NULL))
		return;

	tsp_info("FW update function\n");

	if (ist30xx_auto_bin_update(ts_data))
		ist30xx_disable_irq(ts_data);
}
#endif // IST30XX_INTERNAL_BIN && IST30XX_UPDATE_BY_WORKQUEUE


#if IST30XX_EVENT_MODE
void timer_handler(unsigned long data)
{
	int event_ms;
	int curr_ms;

	if (get_event_mode) {
		if ((ts_data->status.power == 1) && (ts_data->status.update != 1)) {
			ktime_get_ts(&t_current);

			curr_ms = t_current.tv_sec * 1000 + t_current.tv_nsec / 1000000;
			event_ms = t_event.tv_sec * 1000 + t_event.tv_nsec / 1000000;

			tsp_verb("event_ms %d, current: %d\n", event_ms, curr_ms);

			if (ts_data->status.calib == 1) {
				if (curr_ms - event_ms >= 2000) {   // 2second
					ts_data->status.calib = 0;
					tsp_debug("calibration timeout over 3sec\n");
					schedule_delayed_work(&work_reset_check, 0);
					ktime_get_ts(&t_event);
				}
			}
#if IST30XX_NOISE_MODE
			else if (curr_ms - event_ms >= 5000) {  // 5second
				tsp_warn("idle timeout over 5sec\n");
				schedule_delayed_work(&work_reset_check, 0);
			}
#endif                  // IST30XX_NOISE_MODE
		}
	}

	mod_timer(&idle_timer, get_jiffies_64() + EVENT_TIMER_INTERVAL);
}
#endif // IST30XX_EVENT_MODE

static int  ist30xx_parse_dt(struct device *dev, struct ist30xx_tsi_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	//struct property *prop;
	u32 temp_val;//, num_buttons;
//	u32 button_map[MAX_NUM_OF_BUTTON];
	int rc;
	char propname[128];

	strcpy(propname, "imagis,i2c-pull-up");
	pdata->i2c_pull_up = of_property_read_bool(np, propname);
	tsp_debug("%s : %d \n", propname, pdata->i2c_pull_up);

	// x,y cordination
	strcpy(propname, "imagis,panel-x");
	rc = of_property_read_u32(np, propname, &temp_val);
	if (rc && (rc != -EINVAL)) {
		return rc;
	} else {
		pdata->x_max = temp_val;
	}
	tsp_debug("%s :rc=%d val=%d \n", propname, rc, pdata->x_max);
	
	strcpy(propname, "imagis,panel-y");
	rc = of_property_read_u32(np, propname, &temp_val);
	if (rc && (rc != -EINVAL)) {
		return rc;
	} else {
		pdata->y_max = temp_val;
	}
	tsp_debug("%s :rc=%d val=%d \n", propname, rc, pdata->y_max);

	strcpy(propname, "imagis,i2c_int_gpio");
	pdata->i2c_int_gpio = of_get_named_gpio_flags(np, propname, 0, &pdata->irq_flag); //irq_flag active low or high
	tsp_debug("%s : i2c_int_gpio=%d flag=%d \n", propname, pdata->i2c_int_gpio, pdata->irq_flag);

	return 0;
	
}

static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
        return (regulator_count_voltages(reg) > 0) ?
                regulator_set_optimum_mode(reg, load_uA) : 0;
}

static int ist30xx_ldo_power_on(struct ist30xx_data *ts, bool on)
{
        int retval;

        if (on == true) {
                tsp_debug("ist30xx_ldo_power_on : On \n");
        } else {
                tsp_debug("ist30xx_ldo_power_on : Off \n");
        }

        if (on == false)
                goto power_off;

        retval = reg_set_optimum_mode_check(ts->vdd, 100000);
        if (retval < 0) {
                tsp_debug("Regulator vdd set opt failed retval = %d \n", retval);
                return retval;
        }

        retval = regulator_enable(ts->vdd);
        if (retval < 0) {
                tsp_debug("Regulator vdd enable failed retval = %d \n", retval);
                goto err_reg_en_vdd;
        }

        if (ts->pdata->i2c_pull_up) {
                retval = reg_set_optimum_mode_check(ts->vcc_i2c, 100000);
                if (retval < 0) {
                        tsp_debug("Regulator vcc_i2c set opt failed retval = %d \n", retval);
                        goto err_reg_opt_i2c;
                }

                retval = regulator_enable(ts->vcc_i2c);
                if (retval < 0) {
                        tsp_debug("Regulator vcc_i2c enable failed retval = %d \n", retval);
                        goto err_reg_en_vcc_i2c;
                }
        }

        tsp_debug("ist30xx_ldo_power_on On Done \n");
        return 0;

err_reg_en_vcc_i2c:
        reg_set_optimum_mode_check(ts->vdd, 0);
err_reg_opt_i2c:
        regulator_disable(ts->vdd);
err_reg_en_vdd:
        reg_set_optimum_mode_check(ts->vdd, 0);

        tsp_debug("ist30xx_ldo_power_on err \n");

        return retval;

power_off:
        reg_set_optimum_mode_check(ts->vdd, 0);
        regulator_disable(ts->vdd);

        if (ts->pdata->i2c_pull_up) {
                reg_set_optimum_mode_check(ts->vcc_i2c, 0);
                regulator_disable(ts->vcc_i2c);
        }

        tsp_debug("ist30xx_ldo_power_on : Off Done \n");

        return 0;
}

#define IST30XX_VDD_VOLTAGE 2950000
#define IST30XX_I2C_VOLTAGE 1800000

static int ist30xx_regulator_configure(struct ist30xx_data *ts, bool on)
{
        int retval;

        if (on == true) {
                tsp_debug("ist30xx_regulator_configure : On \n");
        } else {
                tsp_debug("ist30xx_regulator_configure : Off \n");
        }

        if (on == false)
                goto hw_shutdown;

        ts->vdd = regulator_get(&ts->client->dev, "vdd");
        if (IS_ERR(ts->vdd)) {
                tsp_debug("Failed to get vdd regulator \n");
                return PTR_ERR(ts->vdd);
        }

        if (regulator_count_voltages(ts->vdd) > 0) {
		tsp_debug("regulator_set_voltage(VDD L22, %d, %d) \n", IST30XX_VDD_VOLTAGE, IST30XX_VDD_VOLTAGE);

                retval = regulator_set_voltage(ts->vdd, IST30XX_VDD_VOLTAGE, IST30XX_VDD_VOLTAGE);
                if (retval) {
                        tsp_debug("regulator set_vtg failed retval=%d \n", retval);
                        goto err_set_vtg_vdd;
                }
        }

        if (ts->pdata->i2c_pull_up) {
                tsp_debug("this device has i2c_pull_up \n");
                ts->vcc_i2c = regulator_get(&ts->client->dev, "vcc_i2c");
                if (IS_ERR(ts->vcc_i2c)) {
                        tsp_debug("Failed to get i2c regulator \n");
                        retval = PTR_ERR(ts->vcc_i2c);
                        goto err_get_vtg_i2c;
                }

                if (regulator_count_voltages(ts->vcc_i2c) > 0) {
			 tsp_debug("regulator_set_voltage(vcc_i2c, %d, %d) \n", IST30XX_I2C_VOLTAGE, IST30XX_I2C_VOLTAGE);
                        retval = regulator_set_voltage(ts->vcc_i2c, IST30XX_I2C_VOLTAGE, IST30XX_I2C_VOLTAGE);
                        if (retval) {
                                tsp_debug("reg set i2c vtg failed retval=%d \n", retval);
                                goto err_set_vtg_i2c;
                        }
                }
        }

        tsp_debug("ist30xx_regulator_configure : On Done \n");
        return 0;

err_set_vtg_i2c:
        if (ts->pdata->i2c_pull_up)
                regulator_put(ts->vcc_i2c);
err_get_vtg_i2c:
        if (regulator_count_voltages(ts->vdd) > 0)
                regulator_set_voltage(ts->vdd, 0, IST30XX_VDD_VOLTAGE);
err_set_vtg_vdd:
        regulator_put(ts->vdd);

        tsp_debug("ist30xx_regulator_configure err \n");

        return retval;

hw_shutdown:
        if (regulator_count_voltages(ts->vdd) > 0)
                regulator_set_voltage(ts->vdd, 0, IST30XX_VDD_VOLTAGE);
                regulator_put(ts->vdd);

        if (ts->pdata->i2c_pull_up) {
                if (regulator_count_voltages(ts->vcc_i2c) > 0)
                        regulator_set_voltage(ts->vcc_i2c, 0, IST30XX_I2C_VOLTAGE);
                regulator_put(ts->vcc_i2c);
        }

        tsp_debug("ist30xx_regulator_configure : Off done \n");

        return 0;
}

static int __devinit ist30xx_probe(struct i2c_client *		client,
				   const struct i2c_device_id * id)
{
	int ret;
	struct ist30xx_data *data;
	struct input_dev *input_dev;

#if 0
	/* sukkyoon.hong@lge.com */
	struct touch_platform_data *ts_pdata;
//	struct ist30xx_ts_device *dev;

	ts_pdata = client->dev.platform_data;
//	dev = &ist30xx_ts_dev;
	/* sukkyoon.hong@lge.com */
#endif

	tsp_info("\n%s(), the i2c addr=0x%x \n", __func__, client->addr);

/*	dev->power = ts_pdata->power;
	dev->num_irq = ts_pdata->irq;
	dev->sda_gpio = ts_pdata->sda;
	dev->scl_gpio  = ts_pdata->scl;*/

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		tsp_debug("failed to i2c functionality check");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}
	
	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;

	if(client->dev.of_node) {
		data->pdata = devm_kzalloc(&client->dev, sizeof(struct ist30xx_tsi_platform_data), GFP_KERNEL);
		if(!data->pdata) {
			tsp_debug("failed to allocate platform_data");
			return -ENOMEM;
		}

		ret = ist30xx_parse_dt(&client->dev, data->pdata);
		if(ret) {
			tsp_debug("device tree parsing failed");
			return ret;
		}
	} else {
		data->pdata = client->dev.platform_data;
	}

	ret = ist30xx_regulator_configure(data, true);
	if (ret < 0) {
			tsp_debug("Failed to configure regulators");
	}

	ret = ist30xx_ldo_power_on(data, true);
	if (ret < 0) {
			tsp_debug("Failed to power on");
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		ret = -ENOMEM;
		tsp_err("%s(), input_allocate_device failed (%d)\n", __func__, ret);
		goto err_alloc_dev;
	}

#if 0
	DMSG("[ TSP ] irq : %d, scl : %d, sda : %d\n", client->irq, ts_pdata->scl, ts_pdata->sda);
#endif
	data->num_fingers = IST30XX_MAX_MT_FINGERS;
	data->num_keys = IST30XX_MAX_MT_FINGERS;
	data->irq_enabled = 1;
	data->client = client;
	data->input_dev = input_dev;
#if 0
	/* sukkyoon.hong@lge.com */
	data->power = ts_pdata->power;
	/* sukkyoon.hong@lge.com */
#endif
	i2c_set_clientdata(client, data);

#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 0, 0))
	input_mt_init_slots(input_dev, IST30XX_MAX_MT_FINGERS);
#endif

	input_dev->name = "ist30xx_ts_input";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	set_bit(EV_ABS, input_dev->evbit);
#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 0, 0))
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
#endif

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, IST30XX_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, IST30XX_MAX_Y, 0, 0);
#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 0, 0))
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, IST30XX_MAX_W, 0, 0);
#else
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, IST30XX_MAX_Z, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, IST30XX_MAX_W, 0, 0);
#endif

#if IST30XX_USE_KEY
	{
		int i;
		set_bit(EV_KEY, input_dev->evbit);
		set_bit(EV_SYN, input_dev->evbit);
		for (i = 1; i < ARRAY_SIZE(ist30xx_key_code); i++)
			set_bit(ist30xx_key_code[i], input_dev->keybit);
	}
#endif

	input_set_drvdata(input_dev, data);
	ret = input_register_device(input_dev);
	if (ret) {
		input_free_device(input_dev);
		goto err_reg_dev;
	}

#if defined(CONFIG_FB)
	data->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&data->fb_notif);
	if(ret)
		tsp_debug("Unable to register fb_notifier \n");
	else
		tsp_debug("Register fb_notifier \n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = ist30xx_early_suspend;
	data->early_suspend.resume = ist30xx_late_resume;
	register_early_suspend(&data->early_suspend);
#endif

	ts_data = data;

	ret = ist30xx_init_system();
	if (ret) {
		dev_err(&client->dev, "chip initialization failed\n");
		goto err_init_drv;
	}

	ret = ist30xx_init_update_sysfs();
	if (ret)
		goto err_init_drv;

#if IST30XX_DEBUG
	ret = ist30xx_init_misc_sysfs();
	if (ret)
		goto err_init_drv;
#endif

# if IST30XX_FACTORY_TEST
	ret = ist30xx_init_factory_sysfs();
	if (ret)
		goto err_init_drv;
#endif

#if IST30XX_TRACKING_MODE
	ret = ist30xx_init_tracking_sysfs();
	if (ret)
		goto err_init_drv;
#endif

	ret = request_threaded_irq(client->irq, NULL, ist30xx_irq_thread,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "ist30xx_ts", data);
	if (ret)
		goto err_irq;

	ist30xx_disable_irq(data);

#if IST30XX_INTERNAL_BIN
# if IST30XX_UPDATE_BY_WORKQUEUE
	INIT_DELAYED_WORK(&work_fw_update, fw_update_func);
	schedule_delayed_work(&work_fw_update, IST30XX_UPDATE_DELAY);
# else
	ret = ist30xx_auto_bin_update(data);
	if (ret < 0)
		goto err_irq;
# endif
#endif  // IST30XX_INTERNAL_BIN

	ret = ist30xx_get_info(data);
	tsp_info("Get info: %s\n", (ret == 0 ? "success" : "fail"));

	INIT_DELAYED_WORK(&work_reset_check, reset_work_func);

#if IRQ_THREAD_WORK_QUEUE
	INIT_WORK(&work_irq_thread, irq_thread_func);
#endif

#if IST30XX_DETECT_TA
	ist30xx_ta_status = 0;
#endif

#if IST30XX_EVENT_MODE
	init_timer(&idle_timer);
	idle_timer.function = timer_handler;
	idle_timer.expires = jiffies_64 + (EVENT_TIMER_INTERVAL);

	mod_timer(&idle_timer, get_jiffies_64() + EVENT_TIMER_INTERVAL);

	ktime_get_ts(&t_event);
#endif

	ist30xx_initialized = 1;

	return 0;

err_irq:
	ist30xx_disable_irq(data);
	free_irq(client->irq, data);
err_init_drv:
#if IST30XX_EVENT_MODE
	get_event_mode = false;
#endif
	tsp_err("Error, ist30xx init driver\n");
//	ist30xx_power_off();
	ist30xx_ts_off();
	input_unregister_device(input_dev);
	return 0;

err_reg_dev:
err_alloc_dev:
	tsp_err("Error, ist30xx mem free\n");
	kfree(data);
err_check_functionality_failed:
	return 0;
}


static int __devexit ist30xx_remove(struct i2c_client *client)
{
	struct ist30xx_data *data = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&data->early_suspend);
#endif

	free_irq(client->irq, data);
//	ist30xx_power_off();
	ist30xx_ts_off();

	input_unregister_device(data->input_dev);
	kfree(data);

	return 0;
}


static struct i2c_device_id ist30xx_idtable[] = {
	{ IST30XX_DEV_NAME, 0 },
	{},
};


MODULE_DEVICE_TABLE(i2c, ist30xx_idtable);

#ifdef CONFIG_HAS_EARLYSUSPEND
static const struct dev_pm_ops ist30xx_pm_ops = {
	.suspend	= ist30xx_suspend,
	.resume		= ist30xx_resume,
};
#endif

#ifdef CONFIG_OF
static struct of_device_id imagis_match_table[] = {
	{ .compatible = "imagis,ist30xx",},
	{ },
};
#else
#define imagis_match_table NULL
#endif

static struct i2c_driver ist30xx_i2c_driver = {
	.id_table	= ist30xx_idtable,
	.probe		= ist30xx_probe,
	.remove		= __devexit_p(ist30xx_remove),
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= IST30XX_DEV_NAME,
		.of_match_table = imagis_match_table,
#ifdef CONFIG_HAS_EARLYSUSPEND
		.pm	= &ist30xx_pm_ops,
#endif
	},
};


static int __init ist30xx_init(void)
{
	return i2c_add_driver(&ist30xx_i2c_driver);
}


static void __exit ist30xx_exit(void)
{
	i2c_del_driver(&ist30xx_i2c_driver);
}

module_init(ist30xx_init);
module_exit(ist30xx_exit);

MODULE_DESCRIPTION("Imagis IST30XX touch driver");
MODULE_LICENSE("GPL");
