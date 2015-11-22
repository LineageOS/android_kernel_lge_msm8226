/* Lge_touch_core.c
 *
 * Copyright (C) 2011 LGE.
 *
 * Author: yehan.ahn@lge.com, hyesung.shin@lge.com
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


#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#if defined(CONFIG_FB)
#include <linux/fb.h>
#include <linux/notifier.h>
#endif
#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
#include <linux/jiffies.h>
#include <linux/sysdev.h>
#include <linux/types.h>
#include <linux/time.h>
#include <linux/version.h>
#include <asm/atomic.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/input/lge_touch_core.h>
#include <linux/wakelock.h>
#include <linux/mutex.h>

static int touch_gesture_enable;
static int ts_suspend;
static int power_block;

static struct wake_lock touch_wake_lock;
static struct mutex i2c_suspend_lock;

static void touch_double_tap_wakeup_enable(struct lge_touch_data *ts);

#include <mach/board_lge.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif

extern bool touch_irq_mask;
extern void touch_enable_irq(unsigned int irq);
extern void touch_disable_irq(unsigned int irq);

struct touch_device_driver *touch_device_func;
struct workqueue_struct *touch_wq;

struct lge_touch_attribute {
	struct attribute	attr;
	ssize_t (*show)(struct lge_touch_data *ts, char *buf);
	ssize_t (*store)(struct lge_touch_data *ts, const char *buf, size_t count);
};

#define LGE_TOUCH_ATTR(_name, _mode, _show, _store)	\
struct lge_touch_attribute lge_touch_attr_##_name = __ATTR(_name, _mode, _show, _store)

/* Debug mask value
 * usage: echo [debug_mask] > /sys/module/lge_touch_core/parameters/debug_mask
 */
u32 touch_debug_mask = 0
				| DEBUG_BASE_INFO
				| DEBUG_FW_UPGRADE;
module_param_named(debug_mask, touch_debug_mask, int, S_IRUGO|S_IWUSR|S_IWGRP);

#ifdef LGE_TOUCH_TIME_DEBUG
/* Debug mask value
 * usage: echo [debug_mask] > /sys/module/lge_touch_core/parameters/time_debug_mask
 */
u32 touch_debug_mask = DEBUG_TIME_PROFILE_NONE;
module_param_named(time_debug_mask, touch_time_debug_mask, int, S_IRUGO|S_IWUSR|S_IWGRP);
#endif

#define MAX_RETRY_COUNT			3
#define MAX_GHOST_CHECK_COUNT	3

/* ghost dtetection related stuff */
struct timeval t_debug[TIME_PROFILE_MAX];
#define get_time_interval(a, b) (a >= b ? a - b : 1000000 + a - b)
static u8 resume_flag;
static unsigned int ta_debouncing_count;
static unsigned int button_press_count;
static unsigned int ts_rebase_count;
struct timeval t_ex_debug[EX_PROFILE_MAX];
struct t_data	 ts_prev_finger_press_data;
int force_continuous_mode;
int long_press_check_count;
int long_press_check;
int finger_subtraction_check_count;
int ghost_detection;
int ghost_detection_count;
int double_tap_enabled;

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void touch_early_suspend(struct early_suspend *h);
static void touch_late_resume(struct early_suspend *h);
#endif

/* Auto Test interface for some model */
struct lge_touch_data *touch_test_dev;
EXPORT_SYMBOL(touch_test_dev);
static void release_all_ts_event(struct lge_touch_data *ts);
/* Jitter Filter
 *
 */
#define jitter_abs(x)		(x > 0 ? x : -x)
#define abs_sub(x, y)	(x > y ? x - y : y - x)

u8 synaptics_probe;
u8 device_control_reg;
u8 synaptics_rebase_retry_cnt;
int trigger_baseline;
int ts_charger_plug;
int ts_charger_type;
static void safety_reset(struct lge_touch_data *ts);
static int touch_ic_init(struct lge_touch_data *ts);
int cur_hopping_idx = 3;
extern int cns_en;
extern bool touch_irq_mask;
static struct hrtimer hr_touch_trigger_timer;
#define MS_TO_NS(x)	(x * 1E6L)

static enum hrtimer_restart touch_trigger_timer_handler(struct hrtimer *timer)
{

	if (synaptics_rebase_retry_cnt == MAX_RETRY_COUNT) {
		trigger_baseline = 0;
		synaptics_rebase_retry_cnt = 0;
		TOUCH_INFO_MSG("Do not attempt to rebase .\n");
		return HRTIMER_NORESTART;
	}

	if (touch_test_dev && touch_test_dev->pdata->role->ghost_detection_enable) {
		TOUCH_INFO_MSG("total_num : %d , palm : %d \n ", touch_test_dev->ts_data.touch_count, touch_test_dev->ts_data.palm);
		if (touch_test_dev->ts_data.total_num || touch_test_dev->ts_data.palm) {
			if (synaptics_rebase_retry_cnt < MAX_RETRY_COUNT) {
				TOUCH_INFO_MSG("retry (%d/3) \n", ++synaptics_rebase_retry_cnt);
				hrtimer_start(&hr_touch_trigger_timer, ktime_set(0, MS_TO_NS(1000)), HRTIMER_MODE_REL);
			}
		} else {
			if (trigger_baseline == 1 && atomic_read(&touch_test_dev->device_init) == 1) {
				trigger_baseline = 2;
				synaptics_rebase_retry_cnt = 0;
				atomic_inc(&touch_test_dev->next_work);
				queue_work(touch_wq, &touch_test_dev->work);
			}
		}
	}
	return HRTIMER_NORESTART;
}

void trigger_usb_state_from_otg(int type)
{
	u8 buf = 0;
	int plug_in = 0;

	if (!synaptics_probe || !device_control_reg)
		return;

	mutex_lock(&i2c_suspend_lock);

	plug_in = (type == 0) ? 0 : 1;

	if (touch_test_dev && touch_test_dev->pdata->role->ghost_detection_enable) {
		TOUCH_INFO_MSG("plug_in_type : %d \n", type);
		/* INVALID:0, SDP:1, DCP:2, CDP:3 */


	    if (plug_in == 0 || plug_in == 1) {
			if (touch_test_dev->curr_pwr_state == POWER_ON
				&& !touch_test_dev->fw_info.fw_upgrade.is_downloading) {
				if (plug_in == 0) {
					touch_i2c_read(touch_test_dev->client, device_control_reg, 1, &buf);
					buf = buf & 0xDF;
					touch_i2c_write_byte(touch_test_dev->client, device_control_reg, buf);

					cns_en = 0;
					if (cur_hopping_idx != 3)
						cur_hopping_idx = 3;
					safety_reset(touch_test_dev);
					queue_delayed_work(touch_wq, &touch_test_dev->work_init,
								msecs_to_jiffies(touch_test_dev->pdata->role->booting_delay));
				} else if (plug_in == 1) {
					touch_i2c_read(touch_test_dev->client, device_control_reg, 1, &buf);
					buf = buf | 0x20;
					touch_i2c_write_byte(touch_test_dev->client, device_control_reg, buf);
				}
			}

			ts_charger_type = type;
			TOUCH_INFO_MSG(" trigger_baseline_state_machine = %d type = %d \n", plug_in, type);
			ts_charger_plug = plug_in;

			if (trigger_baseline == 0 && plug_in == 1) {
				trigger_baseline = 1;

				hrtimer_start(&hr_touch_trigger_timer, ktime_set(0, MS_TO_NS(1000)), HRTIMER_MODE_REL);
			}
		}
	}

	mutex_unlock(&i2c_suspend_lock);

}


#define ts_caps	(ts->pdata->caps)
#define ts_role	(ts->pdata->role)
#define ts_ghost_value	(ts->pdata->caps->ghost_detection_value)

static bool is_valid_ghost_jitter(struct lge_touch_data *ts, struct t_data prev_data, struct t_data curr_data)
{
	int ret = 0;
	ret = (abs_sub(prev_data.x_position, curr_data.x_position) <= ts_ghost_value[JITTER_VALUE]
		&& abs_sub(prev_data.y_position, curr_data.y_position) <= ts_ghost_value[JITTER_VALUE]);
	return ret;
}

bool chk_time_interval(struct timeval t_aft, struct timeval t_bef, int t_val)
{
	if (t_aft.tv_sec - t_bef.tv_sec == 0) {
		if ((get_time_interval(t_aft.tv_usec, t_bef.tv_usec)) <= t_val)
			return true;
	} else if (t_aft.tv_sec - t_bef.tv_sec == 1) {
		if (t_aft.tv_usec + 1000000 - t_bef.tv_usec <= t_val)
			return true;
	}

	return false;
}

int ghost_detect_solution(struct lge_touch_data *ts)
{
	extern u8 pressure_zero;
	int first_int_detection = 0;
	int cnt = 0, id = 0;

	if (trigger_baseline == 2)
		goto out_need_to_rebase;

	if (resume_flag) {
		resume_flag = 0;
		do_gettimeofday(&t_ex_debug[EX_FIRST_INT]);

		if (chk_time_interval(t_ex_debug[EX_FIRST_INT], t_ex_debug[EX_INIT], ts_ghost_value[TIME_SINCE_BOOTING] * 1000)) {
			first_int_detection = 1;
		}
	}

	if (first_int_detection) {
		for (cnt = 0; cnt < ts_caps->max_id; cnt++) {
			if (ts->ts_data.curr_data[cnt].status == FINGER_PRESSED) {
					TOUCH_INFO_MSG("first input time is 200ms\n");
					ghost_detection = true;
			}
		}
	}

	if (pressure_zero == 1) {
		TOUCH_INFO_MSG("pressure\n");
		ghost_detection = true;
	}

	if (ts_charger_plug) {
		if ((ts_role->ta_debouncing_finger_num  <= ts->ts_data.total_num) && (ta_debouncing_count < ts_role->ta_debouncing_count)) {
			ta_debouncing_count++;
			memset(&ts->ts_data.curr_data, 0x0, sizeof(ts->ts_data.curr_data));
			goto out_need_to_debounce;
		} else if (ts->ts_data.total_num < ts_role->ta_debouncing_finger_num) {
			ta_debouncing_count = 0;
		} else
		;
	}

	if ((ts->ts_data.state != TOUCH_ABS_LOCK) && (ts->ts_data.total_num)) {

		if (ts->ts_data.prev_total_num != ts->ts_data.total_num) {
			if (ts->ts_data.prev_total_num <= ts->ts_data.total_num) {
				if (ts->gf_ctrl.stage == GHOST_STAGE_CLEAR || (ts->gf_ctrl.stage | GHOST_STAGE_1) || ts->gf_ctrl.stage == GHOST_STAGE_4)
					ts->ts_data.state = TOUCH_BUTTON_LOCK;

				for (id = 0; id < ts_caps->max_id; id++) {
					if (ts->ts_data.curr_data[id].status == FINGER_PRESSED
							&& ts->ts_data.prev_data[id].status == FINGER_RELEASED) {
						break;
					}
				}

				if (id < MAX_FINGER) {
					memcpy(&t_ex_debug[EX_PREV_PRESS], &t_ex_debug[EX_CURR_PRESS], sizeof(struct timeval));
					do_gettimeofday(&t_ex_debug[EX_CURR_PRESS]);

					if (1 <= ts->ts_data.prev_total_num && 1 <= ts->ts_data.total_num && is_valid_ghost_jitter(ts, ts_prev_finger_press_data, ts->ts_data.curr_data[id])) {
						/* if time_interval between prev fingger pressed and curr finger pressed is less than 50ms, we need to rebase touch ic. */
						if (chk_time_interval(t_ex_debug[EX_CURR_PRESS], t_ex_debug[EX_PREV_PRESS], ts_ghost_value[DURATION_BET_PRESS] * 1000)) {
							ghost_detection = true;
							ghost_detection_count++;
						} else
						;/* do not anything */
					} else if (ts->ts_data.prev_total_num == 0 && ts->ts_data.total_num == 1 && is_valid_ghost_jitter(ts, ts_prev_finger_press_data, ts->ts_data.curr_data[id])) {
					       /* if time_interval between prev fingger pressed and curr finger pressed is less than 50ms, we need to rebase touch ic. */
						if (chk_time_interval(t_ex_debug[EX_CURR_PRESS], t_ex_debug[EX_PREV_PRESS], ts_ghost_value[DURATION_BET_PRESS] * 1000)) {
							ghost_detection = true;
						} else
						;/* do not anything */
					} else if (abs_sub(ts->ts_data.prev_total_num, ts->ts_data.total_num) >= ts_ghost_value[DIFF_FINGER_NUM]) {
						 ghost_detection = true;
					} else
					;/*do not anything */

					memcpy(&ts_prev_finger_press_data, &ts->ts_data.curr_data[id], sizeof(ts_prev_finger_press_data));
				}
			} else {
				memcpy(&t_ex_debug[EX_PREV_PRESS], &t_ex_debug[EX_CURR_PRESS], sizeof(struct timeval));
				do_gettimeofday(&t_ex_debug[EX_CURR_INT]);

				/* if finger subtraction time is less than 10ms, we need to check ghost state. */
				if (chk_time_interval(t_ex_debug[EX_CURR_INT], t_ex_debug[EX_PREV_PRESS], ts_ghost_value[SUBTRACTION_TIME] * 1000))
					finger_subtraction_check_count++;
				else
					finger_subtraction_check_count = 0;

				if (finger_subtraction_check_count >= ts_ghost_value[FINGER_SUBTRACTION_CNT]) {
					finger_subtraction_check_count = 0;
					TOUCH_INFO_MSG("need_to_rebase finger_subtraction!!! \n");
					goto out_need_to_rebase;
				}
			}
		}

		if (force_continuous_mode) {
			do_gettimeofday(&t_ex_debug[EX_CURR_INT]);
			/* if 10 sec have passed since resume, then return to the original report mode. */
			if (t_ex_debug[EX_CURR_INT].tv_sec - t_ex_debug[EX_INIT].tv_sec >= ts_ghost_value[FORCE_CONT_TIME]) {
				if (touch_device_func->ic_ctrl) {
					if (touch_device_func->ic_ctrl(ts->client, IC_CTRL_REPORT_MODE, ts_role->report_mode) < 0) {
						TOUCH_ERR_MSG("IC_CTRL_BASELINE handling fail\n");
						goto out_need_to_init;
					}
				}
				force_continuous_mode = 0;
			}

			long_press_check = 0;

			for (cnt = 0; cnt < MAX_FINGER; cnt++) {
				if (ts->ts_data.curr_data[cnt].status == FINGER_PRESSED) {
					if (ts->ts_data.prev_data[cnt].status == FINGER_PRESSED && is_valid_ghost_jitter(ts, ts->ts_data.prev_data[cnt], ts->ts_data.curr_data[cnt])) {
						long_press_check = true;
					}
				}
			}

			if (long_press_check)
				long_press_check_count++;
			else
				long_press_check_count = 0;

			if (long_press_check_count > ts_ghost_value[LONG_PRESS_CNT]) {
				long_press_check_count = 0;
				TOUCH_INFO_MSG("need_to_rebase long press!!! \n");
				goto out_need_to_rebase;
			}
		}
	} else if (!ts->ts_data.total_num) {
		long_press_check_count = 0;
		finger_subtraction_check_count = 0;
	}


	if (ts->ts_data.state != TOUCH_BUTTON_LOCK) {
		if (ts->work_sync_err_cnt > 0
			&& ts->ts_data.prev_button.state == BUTTON_RELEASED) {
			/* Do nothing */
		} else if (ts_role->ghost_detection_button_enable) {

			if (button_press_count  == 0)
				do_gettimeofday(&t_ex_debug[EX_BUTTON_PRESS_START]);
			else
				do_gettimeofday(&t_ex_debug[EX_BUTTON_PRESS_END]);

			button_press_count++;

			if (button_press_count >= ts_ghost_value[BUTTON_INT_NUM]) {
				if (chk_time_interval(t_ex_debug[EX_BUTTON_PRESS_END], t_ex_debug[EX_BUTTON_PRESS_START], ts_ghost_value[BUTTON_DURATION] * 1000))	{
					TOUCH_INFO_MSG("need_to_rebase button zero\n");
					goto out_need_to_rebase;
				} else
				; /*do not anything */

				button_press_count = 0;
			} else {
				if ((t_ex_debug[EX_BUTTON_PRESS_END].tv_sec -
					t_ex_debug[EX_BUTTON_PRESS_START].tv_sec) > 1)
						button_press_count = 0;
				if (!chk_time_interval(t_ex_debug[EX_BUTTON_PRESS_END], t_ex_debug[EX_BUTTON_PRESS_START], ts_ghost_value[BUTTON_DURATION] * 1000))	{
						button_press_count = 0;
				} else
				; /*do not anything */
			}
		}
	}

	if (ghost_detection == true && ts->ts_data.total_num == 0) {
		TOUCH_INFO_MSG("need_to_rebase zero\n");
		goto out_need_to_rebase;
	} else if (ghost_detection == true && 3 <= ghost_detection_count) {
		TOUCH_INFO_MSG("need_to_rebase zero 3\n");
		goto out_need_to_rebase;
	}

	return 0;

out_need_to_debounce:
	return NEED_TO_OUT;

out_need_to_rebase:
	ghost_detection = false;
	ghost_detection_count = 0;
	memset(&ts_prev_finger_press_data, 0x0, sizeof(ts_prev_finger_press_data));
	button_press_count = 0;
	ts_rebase_count++;

	if (ts_rebase_count == 1) {
		do_gettimeofday(&t_ex_debug[EX_FIRST_GHOST_DETECT]);

		if ((t_ex_debug[EX_FIRST_GHOST_DETECT].tv_sec - t_ex_debug[EX_INIT].tv_sec) <= 3) {
			ts_rebase_count = 0;
			TOUCH_INFO_MSG("need_to_init in 3 sec\n");
			goto out_need_to_init;
		}
	} else {
		do_gettimeofday(&t_ex_debug[EX_SECOND_GHOST_DETECT]);

		if (((t_ex_debug[EX_SECOND_GHOST_DETECT].tv_sec - t_ex_debug[EX_FIRST_GHOST_DETECT].tv_sec) <= ts_ghost_value[TIME_SINCE_REBASE])) {
			ts_rebase_count = 0;
			TOUCH_INFO_MSG("need_to_init\n");
			goto out_need_to_init;
		} else {
			ts_rebase_count = 1;
			memcpy(&t_ex_debug[EX_FIRST_GHOST_DETECT], &t_ex_debug[EX_SECOND_GHOST_DETECT], sizeof(struct timeval));
		}
	}
	release_all_ts_event(ts);
	memset(&ts->ts_data, 0, sizeof(ts->ts_data));
	memset(&ts->accuracy_filter.his_data, 0, sizeof(ts->accuracy_filter.his_data));
	ts->accuracy_filter.finish_filter = 0;
	if (touch_device_func->ic_ctrl) {
		if (touch_device_func->ic_ctrl(ts->client, IC_CTRL_BASELINE, BASELINE_REBASE) < 0) {
			TOUCH_ERR_MSG("IC_CTRL_REBASE handling fail\n");
		}
	}
	TOUCH_INFO_MSG("need_to_rebase\n");

	return NEED_TO_OUT;

out_need_to_init:
	return NEED_TO_INIT;
}

static bool touch_irq_wake;
static int touch_enable_irq_wake(struct lge_touch_data *ts)
{
	int ret = 0;
	if (!touch_irq_wake) {
		touch_irq_wake = 1;
		ret = enable_irq_wake(ts->client->irq);
	} else
		TOUCH_INFO_MSG("touch_enable_irq_wake!!!\n");
	return ret;
}
static int touch_disable_irq_wake(struct lge_touch_data *ts)
{
	int ret = 0;
	if (touch_irq_wake) {
		touch_irq_wake = 0;
		ret = disable_irq_wake(ts->client->irq);
	} else
		TOUCH_INFO_MSG("touch_disable_irq_wake!!!\n");
	return ret;
}

void Send_Touch(unsigned int x, unsigned int y)
{
	if (touch_test_dev) {
		/* press */
#if !defined(MT_PROTOCOL_A)
		input_mt_slot(touch_test_dev->input_dev, 0);
		input_mt_report_slot_state(touch_test_dev->input_dev, MT_TOOL_FINGER, true);
#endif	/* !defined(MT_PROTOCOL_A) */
		input_report_abs(touch_test_dev->input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(touch_test_dev->input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(touch_test_dev->input_dev, ABS_MT_PRESSURE, 1);
		input_report_abs(touch_test_dev->input_dev, ABS_MT_WIDTH_MAJOR, 1);
		input_report_abs(touch_test_dev->input_dev, ABS_MT_WIDTH_MINOR, 1);
#if defined(MT_PROTOCOL_A)
		input_mt_sync(touch_test_dev->input_dev);
#endif	/* defined(MT_PROTOCOL_A) */
		input_sync(touch_test_dev->input_dev);

		/* release */
#if !defined(MT_PROTOCOL_A)
		input_mt_slot(touch_test_dev->input_dev, 0);
		input_mt_report_slot_state(touch_test_dev->input_dev, MT_TOOL_FINGER, false);
#else
		input_mt_sync(touch_test_dev->input_dev);
#endif	/* !defined(MT_PROTOCOL_A) */
		input_sync(touch_test_dev->input_dev);
	} else {
		TOUCH_ERR_MSG("Touch device not found\n");
	}
}
EXPORT_SYMBOL(Send_Touch);

int get_touch_ts_fw_version(char *fw_ver)
{
	if (touch_test_dev) {
		char ver[2];
		ver[0] = (touch_test_dev->fw_info.ic_fw_version[3] & 0x80) >> 7;
		ver[1] = (touch_test_dev->fw_info.ic_fw_version[3] & 0x7F);

		sprintf(fw_ver, "v%d.%02d", ver[0], ver[1]);
		return 1;
	} else {
		return 0;
	}
}
EXPORT_SYMBOL(get_touch_ts_fw_version);


/* set_touch_handle / get_touch_handle
 *
 * Developer can save their object using 'set_touch_handle'.
 * Also, they can restore that using 'get_touch_handle'.
 */
void set_touch_handle(struct i2c_client *client, void* h_touch)
{
	struct lge_touch_data *ts = i2c_get_clientdata(client);
	ts->h_touch = h_touch;
}

void *get_touch_handle(struct i2c_client *client)
{
	struct lge_touch_data *ts = i2c_get_clientdata(client);
	return ts->h_touch;
}

void power_lock_(int value)
{
	power_block |= value;
}

void power_unlock_(int value)
{
	power_block &= ~(value);
}

/* touch_i2c_read / touch_i2c_write
 *
 * Developer can use these fuctions to communicate with touch_device through I2C.
 */
int touch_i2c_read(struct i2c_client *client, u8 reg, int len, u8 *buf)
{
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};

	if (i2c_transfer(client->adapter, msgs, 2) < 0) {
		if (printk_ratelimit())
			TOUCH_ERR_MSG("transfer error\n");
		return -EIO;
	} else
		return 0;
}

int touch_i2c_write(struct i2c_client *client, u8 reg, int len, u8 * buf)
{
	unsigned char send_buf[len + 1];
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = client->flags,
			.len = len+1,
			.buf = send_buf,
		},
	};

	send_buf[0] = (unsigned char)reg;
	memcpy(&send_buf[1], buf, len);

	if (i2c_transfer(client->adapter, msgs, 1) < 0) {
		if (printk_ratelimit())
			TOUCH_ERR_MSG("transfer error\n");
		return -EIO;
	} else
		return 0;
}

int touch_i2c_write_byte(struct i2c_client *client, u8 reg, u8 data)
{
	unsigned char send_buf[2];
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = client->flags,
			.len = 2,
			.buf = send_buf,
		},
	};

	send_buf[0] = (unsigned char)reg;
	send_buf[1] = (unsigned char)data;

	if (i2c_transfer(client->adapter, msgs, 1) < 0) {
		if (printk_ratelimit())
			TOUCH_ERR_MSG("transfer error\n");
		return -EIO;
	} else
		return 0;
}

#ifdef LGE_TOUCH_TIME_DEBUG
static void time_profile_result(struct lge_touch_data *ts)
{
	if (touch_time_debug_mask & DEBUG_TIME_PROFILE_ALL) {
		if (t_debug[TIME_INT_INTERVAL].tv_sec == 0
				&& t_debug[TIME_INT_INTERVAL].tv_usec == 0) {
			t_debug[TIME_INT_INTERVAL].tv_sec = t_debug[TIME_ISR_START].tv_sec;
			t_debug[TIME_INT_INTERVAL].tv_usec = t_debug[TIME_ISR_START].tv_usec;
		} else {
			TOUCH_INFO_MSG("Interval [%6luus], Total [%6luus], IRQ -> Thread IRQ [%6luus] -> work [%6luus] -> report [%6luus]\n",
				get_time_interval(t_debug[TIME_ISR_START].tv_usec, t_debug[TIME_INT_INTERVAL].tv_usec),
				get_time_interval(t_debug[TIME_WORKQUEUE_END].tv_usec, t_debug[TIME_ISR_START].tv_usec),
				get_time_interval(t_debug[TIME_THREAD_ISR_START].tv_usec, t_debug[TIME_ISR_START].tv_usec),
				get_time_interval(t_debug[TIME_WORKQUEUE_START].tv_usec, t_debug[TIME_THREAD_ISR_START].tv_usec),
				get_time_interval(t_debug[TIME_WORKQUEUE_END].tv_usec, t_debug[TIME_WORKQUEUE_START].tv_usec));

			t_debug[TIME_INT_INTERVAL].tv_sec = t_debug[TIME_ISR_START].tv_sec;
			t_debug[TIME_INT_INTERVAL].tv_usec = t_debug[TIME_ISR_START].tv_usec;
		}
	} else {
		if (touch_time_debug_mask & DEBUG_TIME_INT_INTERVAL) {
			if (t_debug[TIME_INT_INTERVAL].tv_sec == 0
					&& t_debug[TIME_INT_INTERVAL].tv_usec == 0) {
				t_debug[TIME_INT_INTERVAL].tv_sec = t_debug[TIME_ISR_START].tv_sec;
				t_debug[TIME_INT_INTERVAL].tv_usec = t_debug[TIME_ISR_START].tv_usec;
			} else {
				TOUCH_INFO_MSG("Interrupt interval: %6luus\n",
					get_time_interval(t_debug[TIME_ISR_START].tv_usec, t_debug[TIME_INT_INTERVAL].tv_usec));

				t_debug[TIME_INT_INTERVAL].tv_sec = t_debug[TIME_ISR_START].tv_sec;
				t_debug[TIME_INT_INTERVAL].tv_usec = t_debug[TIME_ISR_START].tv_usec;
			}
		}

		if (touch_time_debug_mask & DEBUG_TIME_INT_IRQ_DELAY) {
			TOUCH_INFO_MSG("IRQ -> Thread IRQ : %6luus\n",
				get_time_interval(t_debug[TIME_THREAD_ISR_START].tv_usec, t_debug[TIME_ISR_START].tv_usec));
		}

		if (touch_time_debug_mask & DEBUG_TIME_INT_THREAD_IRQ_DELAY) {
			TOUCH_INFO_MSG("Thread IRQ -> work: %6luus\n",
				get_time_interval(t_debug[TIME_WORKQUEUE_START].tv_usec, t_debug[TIME_THREAD_ISR_START].tv_usec));
		}

		if (touch_time_debug_mask & DEBUG_TIME_DATA_HANDLE) {
			TOUCH_INFO_MSG("work -> report: %6luus\n",
				get_time_interval(t_debug[TIME_WORKQUEUE_END].tv_usec, t_debug[TIME_WORKQUEUE_START].tv_usec));
		}
	}

	if (!ts->ts_data.total_num) {
		memset(t_debug, 0x0, sizeof(t_debug));
	}

}
#endif

/* touch_asb_input_report
 *
 * finger status report
 */

static int touch_asb_input_report(struct lge_touch_data *ts, int status)
{
	u16 id = 0;
	u8 total = 0;

	if (status == FINGER_PRESSED) {
		for (id = 0; id < ts->pdata->caps->max_id; id++) {
			if (ts->pdata->role->key_type == TOUCH_SOFT_KEY
					&& (ts->ts_data.curr_data[id].y_position
						>= ts->pdata->caps->y_button_boundary))
				continue;

			if (ts->ts_data.curr_data[id].status == FINGER_PRESSED) {

				u16 temp = ts->ts_data.curr_data[id].pressure;
				u16 temp2 = (u16)(temp * 9);
				ts->ts_data.curr_data[id].pressure = (u16)(temp2 / 10);

#if !defined(MT_PROTOCOL_A)
				input_mt_slot(ts->input_dev, id);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
#endif	/* !defined(MT_PROTOCOL_A) */
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
						ts->ts_data.curr_data[id].x_position);

				/* When a user's finger cross the boundary (from key to LCD),
				    a ABS-event will change its y-position to edge of LCD, automatically.*/
				if (ts->pdata->role->key_type == TOUCH_SOFT_KEY
						&& ts->ts_data.curr_data[id].y_position < ts->pdata->caps->y_button_boundary
						&& ts->ts_data.prev_data[id].y_position > ts->pdata->caps->y_button_boundary
						&& ts->ts_data.prev_button.key_code != KEY_NULL)
					input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, ts->pdata->caps->y_button_boundary);
				else
					input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
						ts->ts_data.curr_data[id].y_position);


				if (ts->pdata->caps->is_pressure_supported)
					input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
									 ts->ts_data.curr_data[id].pressure);
				if (ts->pdata->caps->is_width_supported) {
					input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
									 ts->ts_data.curr_data[id].width_major);
					input_report_abs(ts->input_dev, ABS_MT_WIDTH_MINOR,
									 ts->ts_data.curr_data[id].width_minor);
					input_report_abs(ts->input_dev, ABS_MT_ORIENTATION,
									 ts->ts_data.curr_data[id].width_orientation);
				}
				if (ts->pdata->caps->is_id_supported)
					input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);

#if defined(MT_PROTOCOL_A)
				input_mt_sync(ts->input_dev);
#endif	/* defined(MT_PROTOCOL_A) */

				total++;

#ifdef ISIS_L2
				if (ts->ts_data.prev_data[id].status == FINGER_RELEASED)
					TOUCH_INFO_MSG("%d finger pressed : <%d> x[****] y[****] z[****]\n",
						++ts->ts_data.touch_count, id);
#else
				if (ts->ts_data.prev_data[id].status == FINGER_RELEASED) {
					if (ts->lockscreen) {
						TOUCH_INFO_MSG("%d finger pressed : <%d> x[XXXX] y[XXXX] z[%3d]\n",
							++ts->ts_data.touch_count, id,
							ts->ts_data.curr_data[id].pressure);
					} else {
						TOUCH_INFO_MSG("%d finger pressed : <%d> x[%4d] y[%4d] z[%3d]\n",
							++ts->ts_data.touch_count, id,
							ts->ts_data.curr_data[id].x_position,
							ts->ts_data.curr_data[id].y_position,
							ts->ts_data.curr_data[id].pressure);
					}
				}

				if (unlikely(touch_debug_mask & DEBUG_ABS) && !ts->lockscreen)
					TOUCH_INFO_MSG("<%d> pos[%4d,%4d] w_m[%2d] w_n[%2d] w_o[%2d] p[%3d]\n",
							ts->pdata->caps->is_id_supported ? id : 0,
							ts->ts_data.curr_data[id].x_position,
							ts->ts_data.curr_data[id].y_position,
							ts->pdata->caps->is_width_supported ?
							ts->ts_data.curr_data[id].width_major : 0,
							ts->pdata->caps->is_width_supported ?
							ts->ts_data.curr_data[id].width_minor : 0,
							ts->pdata->caps->is_width_supported ?
							ts->ts_data.curr_data[id].width_orientation : 0,
							ts->pdata->caps->is_pressure_supported ?
							ts->ts_data.curr_data[id].pressure : 0);
#endif
			} else {
#if !defined(MT_PROTOCOL_A)	/* Protocol B type only */
				/* release handling */
#ifdef ISIS_L2
				if (ts->pdata->role->key_type != TOUCH_SOFT_KEY
						&& ts->ts_data.prev_data[id].status == FINGER_PRESSED) {
					input_mt_slot(ts->input_dev, id);
					input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
					TOUCH_INFO_MSG("touch_release[ ] : <%d> x[****] y[****]\n",
						id);
					ts->ts_data.touch_count--;
#else
				if (ts->pdata->role->key_type != TOUCH_SOFT_KEY
						&& ts->ts_data.prev_data[id].status == FINGER_PRESSED) {
					input_mt_slot(ts->input_dev, id);
					input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);

					if (ts->lockscreen) {
						TOUCH_INFO_MSG("touch_release[ ] : <%d> x[XXXX] y[XXXX]\n",
							id);
					} else {
						TOUCH_INFO_MSG("touch_release[ ] : <%d> x[%4d] y[%4d]\n",
							id,
							ts->ts_data.prev_data[id].x_position,
							ts->ts_data.prev_data[id].y_position);
					}

					ts->ts_data.touch_count--;
#endif
				}
#endif
			}
		}
	} else if (status == FINGER_RELEASED) {
#if defined(MT_PROTOCOL_A)
		input_mt_sync(ts->input_dev);
#else	/* MT_PROTOCOL_B */
#ifdef ISIS_L2
		for (id = 0; id < ts->pdata->caps->max_id; id++) {
			if (ts->ts_data.prev_data[id].status == FINGER_PRESSED) {
				input_mt_slot(ts->input_dev, id);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
				TOUCH_INFO_MSG("touch_release[ ] : <%d> x[****] y[****]\n",
					id);
				ts->ts_data.touch_count--;
			}
		}
#else	/* MT_PROTOCOL_B */
		for (id = 0; id < ts->pdata->caps->max_id; id++) {
			if (ts->ts_data.prev_data[id].status == FINGER_PRESSED) {
				input_mt_slot(ts->input_dev, id);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);

				if (ts->lockscreen) {
					TOUCH_INFO_MSG("touch_release[ ] : <%d> x[XXXX] y[XXXX]\n",
						id);
				} else {
					TOUCH_INFO_MSG("touch_release[ ] : <%d> x[%4d] y[%4d]\n",
						id,
						ts->ts_data.prev_data[id].x_position,
						ts->ts_data.prev_data[id].y_position);
				}

				ts->ts_data.touch_count--;
			}
		}
#endif
#endif	/* defined(MT_PROTOCOL_A) */
	}
	return total;
}

/* release_all_ts_event
 *
 * When system enters suspend-state,
 * if user press touch-panel, release them automatically.
 */
static void release_all_ts_event(struct lge_touch_data *ts)
{
	if (ts->pdata->role->key_type == TOUCH_HARD_KEY) {
		if (ts->ts_data.prev_total_num) {
			touch_asb_input_report(ts, FINGER_RELEASED);

			if (likely(touch_debug_mask & (DEBUG_ABS | DEBUG_BASE_INFO)))
				TOUCH_INFO_MSG("touch finger position released\n");

			if (ts->ts_data.palm || ts->ts_data.prev_palm) {
				memset(ts->ts_data.prev_data, 0x0, sizeof(ts->ts_data.prev_data));
				memset(ts->ts_data.curr_data, 0x0, sizeof(ts->ts_data.curr_data));
			}
		}

		if (ts->ts_data.prev_button.state == BUTTON_PRESSED) {
			input_report_key(ts->input_dev, ts->ts_data.prev_button.key_code, BUTTON_RELEASED);

			if (likely(touch_debug_mask & (DEBUG_BUTTON | DEBUG_BASE_INFO)))
				TOUCH_INFO_MSG("Touch KEY[%d] is released\n", ts->ts_data.prev_button.key_code);

			if (ts->ts_data.palm || ts->ts_data.prev_palm) {
				memset(&ts->ts_data.prev_button, 0x0, sizeof(ts->ts_data.prev_button));
				memset(&ts->ts_data.curr_button, 0x0, sizeof(ts->ts_data.curr_button));
			}
		}
	} else if (ts->pdata->role->key_type == VIRTUAL_KEY) {
		if (ts->ts_data.prev_total_num) {
			touch_asb_input_report(ts, FINGER_RELEASED);

			if (likely(touch_debug_mask & (DEBUG_ABS | DEBUG_BASE_INFO)))
				TOUCH_INFO_MSG("touch finger position released\n");
		}
	} else if (ts->pdata->role->key_type == TOUCH_SOFT_KEY) {
		if (ts->ts_data.state == ABS_PRESS) {
			touch_asb_input_report(ts, FINGER_RELEASED);

			if (likely(touch_debug_mask & (DEBUG_ABS | DEBUG_BASE_INFO)))
				TOUCH_INFO_MSG("touch finger position released\n");
		} else if (ts->ts_data.state == BUTTON_PRESS) {
			input_report_key(ts->input_dev, ts->ts_data.prev_button.key_code, BUTTON_RELEASED);

			if (likely(touch_debug_mask & (DEBUG_BUTTON | DEBUG_BASE_INFO)))
				TOUCH_INFO_MSG("Touch KEY[%d] is released\n", ts->ts_data.prev_button.key_code);
		}
	}

	ts->ts_data.prev_total_num = 0;
	ts->ts_data.touch_count = 0;

	input_sync(ts->input_dev);
}

/* touch_power_cntl
 *
 * 1. POWER_ON
 * 2. POWER_OFF
 * 3. POWER_SLEEP
 * 4. POWER_WAKE
 */
static int touch_power_cntl(struct lge_touch_data *ts, int onoff)
{
	int ret = 0;

	if (touch_device_func->power == NULL) {
		TOUCH_INFO_MSG("There is no specific power control function\n");
		return -1;
	}

	switch (onoff) {
	case POWER_ON:
		ret = touch_device_func->power(ts->client, POWER_ON);
		if (ret < 0) {
			TOUCH_ERR_MSG("power on failed\n");
		} else {
			ts->curr_pwr_state = POWER_ON;
			TOUCH_INFO_MSG("power on \n");
		}

		break;
	case POWER_OFF:
		ret = touch_device_func->power(ts->client, POWER_OFF);
		if (ret < 0) {
			TOUCH_ERR_MSG("power off failed\n");
		} else {
			ts->curr_pwr_state = POWER_OFF;
			TOUCH_INFO_MSG("power off \n");
		}

		msleep(ts->pdata->role->reset_delay);

		atomic_set(&ts->device_init, 0);
		break;
	case POWER_SLEEP:
		ret = touch_device_func->power(ts->client, POWER_SLEEP);
		if (ret < 0) {
			TOUCH_ERR_MSG("power sleep failed\n");
		} else {
			ts->curr_pwr_state = POWER_SLEEP;
			TOUCH_INFO_MSG("power sleep \n");
		}

		break;
	case POWER_WAKE:
		ret = touch_device_func->power(ts->client, POWER_WAKE);
		if (ret < 0) {
			TOUCH_ERR_MSG("power wake failed\n");
		} else {
			ts->curr_pwr_state = POWER_WAKE;
			TOUCH_INFO_MSG("power wake \n");
		}

		break;
	default:
		break;
	}

	if (unlikely(touch_debug_mask & DEBUG_POWER))
		if (ret >= 0)
			TOUCH_INFO_MSG("%s: power_state[%d]", __FUNCTION__, ts->curr_pwr_state);

	return ret;
}

/* safety_reset
 *
 * 1. disable irq/timer.
 * 2. turn off the power.
 * 3. turn on the power.
 * 4. sleep (booting_delay)ms, usually 400ms(synaptics).
 * 5. enable irq/timer.
 *
 * After 'safety_reset', we should call 'touch_init'.
 */
static void safety_reset(struct lge_touch_data *ts)
{
	if (ts->pdata->role->operation_mode)
		touch_disable_irq(ts->client->irq);
	else
		hrtimer_cancel(&ts->timer);

	if (ts->pdata->role->ghost_detection_enable) {
		hrtimer_cancel(&hr_touch_trigger_timer);
	}
	release_all_ts_event(ts);

	if (ts->pdata->role->suspend_pwr == POWER_OFF) {
		touch_power_cntl(ts, POWER_OFF);
		touch_power_cntl(ts, POWER_ON);
	} else if (ts->pdata->role->suspend_pwr == POWER_SLEEP) {
		TOUCH_INFO_MSG("SOFT RESET in safety_reset func");
		touch_device_func->ic_ctrl(ts->client, IC_CTRL_RESET_CMD, 0);
	} else {
	}
	msleep(ts->pdata->role->booting_delay);

	if (ts->pdata->role->operation_mode)
		touch_enable_irq(ts->client->irq);
	else
		hrtimer_start(&ts->timer, ktime_set(0, ts->pdata->role->report_period), HRTIMER_MODE_REL);

	return;
}

/* touch_ic_init
 *
 * initialize the device_IC and variables.
 */
static int touch_ic_init(struct lge_touch_data *ts)
{
	int next_work = 0;
	/*unsigned char *r_mem = NULL; */
	ts->int_pin_state = 0;

	if (unlikely(ts->ic_init_err_cnt >= MAX_RETRY_COUNT)) {
		TOUCH_ERR_MSG("Init Failed: Irq-pin has some unknown problems\n");
		goto err_out_critical;
	}

	atomic_set(&ts->next_work, 0);
	atomic_set(&ts->device_init, 1);

	if (touch_device_func->init == NULL) {
		TOUCH_INFO_MSG("There is no specific IC init function\n");
		goto err_out_critical;
	}

	if (touch_device_func->init(ts->client, &ts->fw_info) < 0) {
		TOUCH_ERR_MSG("specific device initialization fail\n");
		goto err_out_retry;
	}

	if (touch_device_func->ic_ctrl) {
		if (touch_device_func->ic_ctrl(ts->client, IC_CTRL_DOUBLE_TAP_WAKEUP_MODE, 0) < 0) {
			TOUCH_ERR_MSG("IC_CTRL_DOUBLE_TAP_WAKEUP_MODE handling fail\n");
			goto err_out_retry;
		}
	}

	/* Interrupt pin check after IC init - avoid Touch lockup */
	if (ts->pdata->role->operation_mode == INTERRUPT_MODE) {
		ts->int_pin_state = gpio_get_value(ts->pdata->int_pin);
		next_work = atomic_read(&ts->next_work);

		if (unlikely(ts->int_pin_state != 1 && next_work <= 0)) {
			TOUCH_INFO_MSG("WARN: Interrupt pin is low - next_work: %d, try_count: %d]\n",
					next_work, ts->ic_init_err_cnt);
			goto err_out_retry;
		}
	}

	if (ts->pdata->role->ghost_detection_enable) {
		   /* force continuous mode after IC init  */
		if (touch_device_func->ic_ctrl) {
			TOUCH_INFO_MSG("force continuous mode !!!\n");
			if (touch_device_func->ic_ctrl(ts->client, IC_CTRL_REPORT_MODE, 0) < 0) {
				TOUCH_ERR_MSG("IC_CTRL_BASELINE handling fail\n");
				goto err_out_retry;
			}
			force_continuous_mode = 1;
		}
		trigger_baseline = 0;
		ghost_detection = 0;
		ghost_detection_count = 0;
		do_gettimeofday(&t_ex_debug[EX_INIT]);
	}

	ts->gf_ctrl.count = 0;
	ts->gf_ctrl.ghost_check_count = 0;
	ts->gf_ctrl.saved_x = -1;
	ts->gf_ctrl.saved_y = -1;

	if (ts->gf_ctrl.probe) {
		ts->gf_ctrl.stage = GHOST_STAGE_1;
		if (touch_device_func->ic_ctrl) {
			if (touch_device_func->ic_ctrl(ts->client, IC_CTRL_BASELINE, BASELINE_OPEN) < 0) {
				TOUCH_ERR_MSG("IC_CTRL_BASELINE handling fail\n");
				goto err_out_retry;
			}
		}
	} else {
		if (ts->gf_ctrl.stage & GHOST_STAGE_2) {
			ts->gf_ctrl.stage = GHOST_STAGE_1 | GHOST_STAGE_2 | GHOST_STAGE_4;
			ts->gf_ctrl.ghost_check_count = MAX_GHOST_CHECK_COUNT - 1;
			if (touch_device_func->ic_ctrl) {
				if (touch_device_func->ic_ctrl(ts->client, IC_CTRL_BASELINE, BASELINE_OPEN) < 0) {
					TOUCH_ERR_MSG("IC_CTRL_BASELINE handling fail\n");
					goto err_out_retry;
				}
			}
		} else {
			ts->gf_ctrl.stage = GHOST_STAGE_3;
			if (touch_device_func->ic_ctrl) {
				if (touch_device_func->ic_ctrl(ts->client, IC_CTRL_BASELINE, BASELINE_FIX) < 0) {
					TOUCH_ERR_MSG("IC_CTRL_BASELINE handling fail\n");
					goto err_out_retry;
				}
			}
		}
	}

	if (unlikely(touch_debug_mask & (DEBUG_BASE_INFO | DEBUG_GHOST))) {
		if (ts->fw_info.ic_fw_version[0] > 0x40) {
			TOUCH_INFO_MSG("%s %s(%s): FW ver[%s], force[%d]\n",
				ts->pdata->maker, ts->fw_info.ic_fw_identifier,
				ts->pdata->role->operation_mode ? "Interrupt mode" : "Polling mode",
				ts->fw_info.ic_fw_version, ts->fw_info.fw_upgrade.fw_force_upgrade);
		} else{
			TOUCH_INFO_MSG("%s %s(%s): FW ver[V%d.%02d (0x%02X 0x%02X 0x%02X 0x%02X)], force[%d]\n",
				ts->pdata->maker, ts->fw_info.ic_fw_identifier,
				ts->pdata->role->operation_mode ? "Interrupt mode" : "Polling mode",
				(ts->fw_info.ic_fw_version[3]&0x80 ? 1 : 0), ts->fw_info.ic_fw_version[3]&0x7F,
				ts->fw_info.ic_fw_version[0], ts->fw_info.ic_fw_version[1],
				ts->fw_info.ic_fw_version[2], ts->fw_info.ic_fw_version[3],
				ts->fw_info.fw_upgrade.fw_force_upgrade);
		}
		TOUCH_INFO_MSG("irq_pin[%d] next_work[%d] ghost_stage[0x%x]\n",
				ts->int_pin_state, next_work, ts->gf_ctrl.stage);
	}

	ts->gf_ctrl.probe = 0;

	memset(&ts->ts_data, 0, sizeof(ts->ts_data));
	memset(&ts->fw_info.fw_upgrade, 0, sizeof(ts->fw_info.fw_upgrade));
	ts->ic_init_err_cnt = 0;

	ts->jitter_filter.id_mask = 0;
	memset(ts->jitter_filter.his_data, 0, sizeof(ts->jitter_filter.his_data));
	memset(&ts->accuracy_filter.his_data, 0, sizeof(ts->accuracy_filter.his_data));

	ts->accuracy_filter.finish_filter = 0;

	return 0;

err_out_retry:
	ts->ic_init_err_cnt++;
	safety_reset(ts);
	queue_delayed_work(touch_wq, &ts->work_init, msecs_to_jiffies(10));

	return 0;

err_out_critical:
	ts->ic_init_err_cnt = 0;

	return -1;
}

/* ghost_finger_solution
 *
 * GHOST_STAGE_1
 * - melt_mode.
 * - If user press and release their finger in 1 sec, STAGE_1 will be cleared. --> STAGE_2
 * - If there is no key-guard, ghost_finger_solution is finished.
 *
 * GHOST_STAGE_2
 * - no_melt_mode
 * - if user use multi-finger, stage will be changed to STAGE_1
 *   (We assume that ghost-finger occured)
 * - if key-guard is unlocked, STAGE_2 is cleared. --> STAGE_3
 *
 * GHOST_STAGE_3
 * - when user release their finger, device re-scan the baseline.
 * - Then, GHOST_STAGE3 is cleared and ghost_finger_solution is finished.
 */
#define ghost_sub(x, y)	(x > y ? x - y : y - x)

static int ghost_finger_solution(struct lge_touch_data *ts)
{
	u8	id = 0;

	for (id = 0; id < ts->pdata->caps->max_id; id++) {
		if (ts->ts_data.curr_data[id].status == FINGER_PRESSED) {
			break;
		}
	}

	if (ts->gf_ctrl.stage & GHOST_STAGE_1) {
		if (ts->ts_data.total_num == 0 && ts->ts_data.curr_button.state == 0 && ts->ts_data.palm == 0) {
			if (ts->gf_ctrl.count < ts->gf_ctrl.min_count || ts->gf_ctrl.count >= ts->gf_ctrl.max_count) {
				if (ts->gf_ctrl.stage & GHOST_STAGE_2)
					ts->gf_ctrl.ghost_check_count = MAX_GHOST_CHECK_COUNT - 1;
				else
					ts->gf_ctrl.ghost_check_count = 0;
			} else {
				if (ghost_sub(ts->gf_ctrl.saved_x, ts->gf_ctrl.saved_last_x) > ts->gf_ctrl.max_moved ||
				   ghost_sub(ts->gf_ctrl.saved_y, ts->gf_ctrl.saved_last_y) > ts->gf_ctrl.max_moved)
					ts->gf_ctrl.ghost_check_count = MAX_GHOST_CHECK_COUNT;
				else
					ts->gf_ctrl.ghost_check_count++;

				if (unlikely(touch_debug_mask & DEBUG_GHOST))
					TOUCH_INFO_MSG("ghost_stage_1: delta[%d/%d/%d]\n",
						ghost_sub(ts->gf_ctrl.saved_x, ts->gf_ctrl.saved_last_x),
						ghost_sub(ts->gf_ctrl.saved_y, ts->gf_ctrl.saved_last_y),
						ts->gf_ctrl.max_moved);
			}

			if (unlikely(touch_debug_mask & DEBUG_GHOST || touch_debug_mask & DEBUG_BASE_INFO))
				TOUCH_INFO_MSG("ghost_stage_1: ghost_check_count+[0x%x]\n", ts->gf_ctrl.ghost_check_count);

			if (ts->gf_ctrl.ghost_check_count >= MAX_GHOST_CHECK_COUNT) {
				ts->gf_ctrl.ghost_check_count = 0;
				if (touch_device_func->ic_ctrl) {
					if (touch_device_func->ic_ctrl(ts->client, IC_CTRL_BASELINE, BASELINE_FIX) < 0)
						return -1;
				}
				ts->gf_ctrl.stage &= ~GHOST_STAGE_1;
				if (unlikely(touch_debug_mask & DEBUG_GHOST || touch_debug_mask & DEBUG_BASE_INFO))
					TOUCH_INFO_MSG("ghost_stage_1: cleared[0x%x]\n", ts->gf_ctrl.stage);
				if (!ts->gf_ctrl.stage) {
					if (unlikely(touch_debug_mask & DEBUG_GHOST))
						TOUCH_INFO_MSG("ghost_stage_finished. (NON-KEYGUARD)\n");
				}
			}
			ts->gf_ctrl.count = 0;
			ts->gf_ctrl.saved_x = -1;
			ts->gf_ctrl.saved_y = -1;
		} else if (ts->ts_data.total_num == 1 && ts->ts_data.curr_button.state == 0
				&& id == 0 && ts->ts_data.palm == 0) {
			if (ts->gf_ctrl.saved_x == -1 && ts->gf_ctrl.saved_x == -1) {
				ts->gf_ctrl.saved_x = ts->ts_data.curr_data[id].x_position;
				ts->gf_ctrl.saved_y = ts->ts_data.curr_data[id].y_position;
			}
			ts->gf_ctrl.count++;
			if (unlikely(touch_debug_mask & DEBUG_GHOST))
				TOUCH_INFO_MSG("ghost_stage_1: int_count[%d/%d]\n", ts->gf_ctrl.count, ts->gf_ctrl.max_count);
		} else {
			if (unlikely(touch_debug_mask & DEBUG_GHOST) && ts->gf_ctrl.count != ts->gf_ctrl.max_count)
				TOUCH_INFO_MSG("ghost_stage_1: Not good condition. total[%d] button[%d] id[%d] palm[%d]\n",
						ts->ts_data.total_num, ts->ts_data.curr_button.state,
						id, ts->ts_data.palm);
			ts->gf_ctrl.count = ts->gf_ctrl.max_count;
		}
	} else if (ts->gf_ctrl.stage & GHOST_STAGE_2) {
		if (ts->ts_data.total_num > 1 || (ts->ts_data.total_num == 1 && ts->ts_data.curr_button.state)) {
			ts->gf_ctrl.stage |= GHOST_STAGE_1;
			ts->gf_ctrl.ghost_check_count = MAX_GHOST_CHECK_COUNT - 1;
			ts->gf_ctrl.count = 0;
			ts->gf_ctrl.saved_x = -1;
			ts->gf_ctrl.saved_y = -1;
			if (touch_device_func->ic_ctrl) {
				if (touch_device_func->ic_ctrl(ts->client, IC_CTRL_BASELINE, BASELINE_OPEN) < 0)
					return -1;
			}
			if (unlikely(touch_debug_mask & DEBUG_GHOST))
				TOUCH_INFO_MSG("ghost_stage_2: multi_finger. return to ghost_stage_1[0x%x]\n", ts->gf_ctrl.stage);
		}
	} else if (ts->gf_ctrl.stage & GHOST_STAGE_3) {
		if (ts->ts_data.total_num == 0 && ts->ts_data.curr_button.state == 0 && ts->ts_data.palm == 0) {
			ts->gf_ctrl.ghost_check_count++;

			if (unlikely(touch_debug_mask & DEBUG_GHOST || touch_debug_mask & DEBUG_BASE_INFO))
				TOUCH_INFO_MSG("ghost_stage_3: ghost_check_count+[0x%x]\n", ts->gf_ctrl.ghost_check_count);

			if (ts->gf_ctrl.ghost_check_count >= MAX_GHOST_CHECK_COUNT) {
				ts->gf_ctrl.stage &= ~GHOST_STAGE_3;
				if (unlikely(touch_debug_mask & DEBUG_GHOST || touch_debug_mask & DEBUG_BASE_INFO))
					TOUCH_INFO_MSG("ghost_stage_3: cleared[0x%x]\n", ts->gf_ctrl.stage);
				if (!ts->gf_ctrl.stage) {
					if (unlikely(touch_debug_mask & DEBUG_GHOST))
						TOUCH_INFO_MSG("ghost_stage_finished. (NON-KEYGUARD)\n");
				}
			}
		} else if (ts->ts_data.total_num == 1 && ts->ts_data.curr_button.state == 0
				&& id == 0 && ts->ts_data.palm == 0)
		;
		else {
			ts->gf_ctrl.stage &= ~GHOST_STAGE_3;
			ts->gf_ctrl.stage |= GHOST_STAGE_1;
			ts->gf_ctrl.ghost_check_count = 0;
			ts->gf_ctrl.count = 0;
			ts->gf_ctrl.saved_x = -1;
			ts->gf_ctrl.saved_y = -1;
			if (touch_device_func->ic_ctrl) {
				if (touch_device_func->ic_ctrl(ts->client, IC_CTRL_BASELINE, BASELINE_OPEN) < 0)
					return -1;
			}

			if (unlikely(touch_debug_mask & DEBUG_GHOST)) {
				TOUCH_INFO_MSG("ghost_stage_3: Not good condition. total[%d] button[%d] id[%d] palm[%d]\n",
						ts->ts_data.total_num, ts->ts_data.curr_button.state,
						id, ts->ts_data.palm);
				TOUCH_INFO_MSG("ghost_stage_3: return to ghost_stage_1[0x%x]\n", ts->gf_ctrl.stage);
			}
		}
	} else if (ts->gf_ctrl.stage & GHOST_STAGE_4) {
		if (ts->ts_data.total_num == 0 && ts->ts_data.curr_button.state == 0 && ts->ts_data.palm == 0) {
			ts->gf_ctrl.stage = GHOST_STAGE_CLEAR;
			if (unlikely(touch_debug_mask & DEBUG_GHOST || touch_debug_mask & DEBUG_BASE_INFO))
				TOUCH_INFO_MSG("ghost_stage_4: cleared[0x%x]\n", ts->gf_ctrl.stage);
			if (unlikely(touch_debug_mask & DEBUG_GHOST))
				TOUCH_INFO_MSG("ghost_stage_finished. (KEYGUARD)\n");
		}

		if (unlikely(touch_debug_mask & DEBUG_GHOST) && ts->ts_data.palm != 0)
			TOUCH_INFO_MSG("ghost_stage_4: palm[%d]\n", ts->ts_data.palm);
	}

	ts->gf_ctrl.saved_last_x = ts->ts_data.curr_data[id].x_position;
	ts->gf_ctrl.saved_last_y = ts->ts_data.curr_data[id].y_position;

	return 0;
}


static u16 check_boundary(int x, int max)
{
	if (x < 0)
		return 0;
	else if (x > max)
		return (u16)max;
	else
		return (u16)x;
}

static int check_direction(int x)
{
	if (x > 0)
		return 1;
	else if (x < 0)
		return -1;
	else
		return 0;
}

int accuracy_filter_func(struct lge_touch_data *ts)
{
	int delta_x = 0;
	int delta_y = 0;
	u8	id = 0;

	for (id = 0; id < ts->pdata->caps->max_id; id++) {
		if (ts->ts_data.curr_data[id].status == FINGER_PRESSED) {
			break;
		}
	}

	/* finish the accuracy_filter */
	if (ts->accuracy_filter.finish_filter == 1 &&
	   (ts->accuracy_filter.his_data.count > ts->accuracy_filter.touch_max_count
	    || ts->ts_data.total_num != 1
	    || id != 0)) {
		ts->accuracy_filter.finish_filter = 0;
		ts->accuracy_filter.his_data.count = 0;
	}

	delta_x = (int)ts->accuracy_filter.his_data.x - (int)ts->ts_data.curr_data[0].x_position;
	delta_y = (int)ts->accuracy_filter.his_data.y - (int)ts->ts_data.curr_data[0].y_position;

	/* main algorithm */
	if (ts->accuracy_filter.finish_filter) {
		if (delta_x || delta_y) {
			ts->accuracy_filter.his_data.axis_x += check_direction(delta_x);
			ts->accuracy_filter.his_data.axis_y += check_direction(delta_y);
			ts->accuracy_filter.his_data.count++;
		}

		if (ts->accuracy_filter.his_data.count == 1
			|| ((abs_sub(ts->ts_data.curr_data[0].pressure, ts->accuracy_filter.his_data.pressure) > ts->accuracy_filter.ignore_pressure_gap
				|| ts->ts_data.curr_data[0].pressure > ts->accuracy_filter.max_pressure)
				&& !((ts->accuracy_filter.his_data.count > ts->accuracy_filter.time_to_max_pressure
					&& (jitter_abs(ts->accuracy_filter.his_data.axis_x) == ts->accuracy_filter.his_data.count
					|| jitter_abs(ts->accuracy_filter.his_data.axis_y) == ts->accuracy_filter.his_data.count))
					|| (jitter_abs(ts->accuracy_filter.his_data.axis_x) > ts->accuracy_filter.direction_count
					|| jitter_abs(ts->accuracy_filter.his_data.axis_y) > ts->accuracy_filter.direction_count)))) {
					ts->accuracy_filter.his_data.mod_x += delta_x;
					ts->accuracy_filter.his_data.mod_y += delta_y;
		}
	}

	/* if 'delta' > delta_max or id != 0, remove the modify-value. */
	if (id != 0 ||
		(ts->accuracy_filter.his_data.count != 1 &&
			(jitter_abs(delta_x) > ts->accuracy_filter.delta_max || jitter_abs(delta_y) > ts->accuracy_filter.delta_max))) {
		ts->accuracy_filter.his_data.mod_x = 0;
		ts->accuracy_filter.his_data.mod_y = 0;
	}

	/* start the accuracy_filter */
	if (ts->accuracy_filter.finish_filter == 0
	   && ts->accuracy_filter.his_data.count == 0
	   && ts->ts_data.total_num == 1
	   && ts->accuracy_filter.his_data.prev_total_num == 0
	   && id == 0) {
		ts->accuracy_filter.finish_filter = 1;
		memset(&ts->accuracy_filter.his_data, 0, sizeof(ts->accuracy_filter.his_data));
	}

	if (unlikely(touch_debug_mask & DEBUG_ACCURACY)) {
		TOUCH_INFO_MSG("AccuracyFilter: <%d> pos[%4d,%4d] new[%4d,%4d] his[%4d,%4d] delta[%3d,%3d] mod[%3d,%3d] p[%d,%3d,%3d] axis[%2d,%2d] count[%2d/%2d] total_num[%d,%d] finish[%d]\n",
				ts->ts_data.curr_data[0].id, ts->ts_data.curr_data[0].x_position, ts->ts_data.curr_data[0].y_position,
				check_boundary((int)ts->ts_data.curr_data[0].x_position + ts->accuracy_filter.his_data.mod_x, ts->pdata->caps->x_max),
				check_boundary((int)ts->ts_data.curr_data[0].y_position + ts->accuracy_filter.his_data.mod_y, ts->pdata->caps->y_max),
				ts->accuracy_filter.his_data.x, ts->accuracy_filter.his_data.y,
				delta_x, delta_y,
				ts->accuracy_filter.his_data.mod_x, ts->accuracy_filter.his_data.mod_y,
				abs_sub(ts->ts_data.curr_data[0].pressure, ts->accuracy_filter.his_data.pressure) > ts->accuracy_filter.ignore_pressure_gap,
				ts->ts_data.curr_data[0].pressure, ts->accuracy_filter.his_data.pressure,
				ts->accuracy_filter.his_data.axis_x, ts->accuracy_filter.his_data.axis_y,
				ts->accuracy_filter.his_data.count, ts->accuracy_filter.touch_max_count,
				ts->accuracy_filter.his_data.prev_total_num, ts->ts_data.total_num, ts->accuracy_filter.finish_filter);
	}

	ts->accuracy_filter.his_data.x = ts->ts_data.curr_data[0].x_position;
	ts->accuracy_filter.his_data.y = ts->ts_data.curr_data[0].y_position;
	ts->accuracy_filter.his_data.pressure = ts->ts_data.curr_data[0].pressure;
	ts->accuracy_filter.his_data.prev_total_num = ts->ts_data.total_num;

	if (ts->ts_data.total_num) {
		ts->ts_data.curr_data[0].x_position
			= check_boundary((int)ts->ts_data.curr_data[0].x_position + ts->accuracy_filter.his_data.mod_x, ts->pdata->caps->x_max);
		ts->ts_data.curr_data[0].y_position
			= check_boundary((int)ts->ts_data.curr_data[0].y_position + ts->accuracy_filter.his_data.mod_y, ts->pdata->caps->y_max);
	}

	return 0;
}
EXPORT_SYMBOL(accuracy_filter_func);

int jitter_filter_func(struct lge_touch_data *ts)
{
	u16 id = 0;
	int jitter_count = 0;
	u16 new_id_mask = 0;
	u16 bit_mask = 0;
	u16 bit_id = 1;
	int curr_ratio = ts->pdata->role->jitter_curr_ratio;

	for (id = 0; id < ts->pdata->caps->max_id; id++) {
		if (ts->ts_data.curr_data[id].status == FINGER_PRESSED) {
			u16 width = ts->ts_data.curr_data[id].width_major;
			/*do not set new_id_mask at total_num = 0  */
			if (ts->ts_data.total_num)
				new_id_mask |= (1 << id);

			if (ts->jitter_filter.id_mask & (1 << id)) {
				int delta_x, delta_y;
				int f_jitter = curr_ratio*width;
				int adjust_x = 0;
				int adjust_y = 0;
				int adj_ratio = 0;
				char adj_mode = 0;

				if (ts->jitter_filter.adjust_margin > 0) {
					adjust_x = (int)ts->ts_data.curr_data[id].x_position
								- (int)ts->jitter_filter.his_data[id].x;
					adjust_y = (int)ts->ts_data.curr_data[id].y_position
								- (int)ts->jitter_filter.his_data[id].y;
				}

				ts->ts_data.curr_data[id].x_position =
						(ts->ts_data.curr_data[id].x_position + ts->jitter_filter.his_data[id].x) >> 1;
				ts->ts_data.curr_data[id].y_position =
						(ts->ts_data.curr_data[id].y_position + ts->jitter_filter.his_data[id].y) >> 1;

				if (ts->jitter_filter.adjust_margin > 0) {
					adj_ratio = (((width + 1) << 6) / (curr_ratio + 1))
							+ (ts->jitter_filter.adjust_margin >> 3);
					if (jitter_abs(adjust_x) > ts->jitter_filter.adjust_margin
							|| jitter_abs(adjust_x) > adj_ratio
							|| jitter_abs(adjust_y) > ts->jitter_filter.adjust_margin
							|| jitter_abs(adjust_y) > adj_ratio) {
						adjust_x = (int)ts->ts_data.curr_data[id].x_position + (adjust_x >> 2);
						ts->ts_data.curr_data[id].x_position = check_boundary(adjust_x, ts->pdata->caps->x_max);

						adjust_y = (int)ts->ts_data.curr_data[id].y_position + (adjust_y >> 2);
						ts->ts_data.curr_data[id].y_position = check_boundary(adjust_y, ts->pdata->caps->y_max);

						adj_mode = 1;
					}
				}

				delta_x = (int)ts->ts_data.curr_data[id].x_position - (int)ts->jitter_filter.his_data[id].x;
				delta_y = (int)ts->ts_data.curr_data[id].y_position - (int)ts->jitter_filter.his_data[id].y;

				ts->jitter_filter.his_data[id].delta_x = delta_x * curr_ratio
						+ ((ts->jitter_filter.his_data[id].delta_x * (128 - curr_ratio)) >> 7);
				ts->jitter_filter.his_data[id].delta_y = delta_y * curr_ratio
						+ ((ts->jitter_filter.his_data[id].delta_y * (128 - curr_ratio)) >> 7);

				if (unlikely(touch_debug_mask & DEBUG_JITTER) && !ts->lockscreen) {
					TOUCH_INFO_MSG("JitterFilter[%s]: <%d> p[%4d,%4d] h_p[%4d,%4d] w[%d] a_r[%d] d[%d,%d] h_d[%d,%d] f_j[%d]\n",
							adj_mode ? "fast" : "norm", id, ts->ts_data.curr_data[id].x_position, ts->ts_data.curr_data[id].y_position,
							ts->jitter_filter.his_data[id].x, ts->jitter_filter.his_data[id].y,
							width, adj_ratio, delta_x, delta_y,
							ts->jitter_filter.his_data[id].delta_x, ts->jitter_filter.his_data[id].delta_y, f_jitter);
				}

				if (jitter_abs(ts->jitter_filter.his_data[id].delta_x) <= f_jitter &&
					jitter_abs(ts->jitter_filter.his_data[id].delta_y) <= f_jitter)
					jitter_count++;
			}
		}
	}

	bit_mask = ts->jitter_filter.id_mask ^ new_id_mask;

	for (id = 0, bit_id = 1; id < ts->pdata->caps->max_id; id++) {
		if ((ts->jitter_filter.id_mask & bit_id) && !(new_id_mask & bit_id)) {
			if (unlikely(touch_debug_mask & DEBUG_JITTER))
				TOUCH_INFO_MSG("JitterFilter: released - id[%d] mask[0x%x]\n", bit_id, ts->jitter_filter.id_mask);
			memset(&ts->jitter_filter.his_data[id], 0, sizeof(ts->jitter_filter.his_data[id]));
		}
		bit_id = bit_id << 1;
	}

	for (id = 0; id < ts->pdata->caps->max_id; id++) {
		if (ts->ts_data.curr_data[id].status == FINGER_PRESSED) {
			ts->jitter_filter.his_data[id].pressure = ts->ts_data.curr_data[id].pressure;
		}
	}

	if (!bit_mask && ts->ts_data.total_num && ts->ts_data.total_num == jitter_count) {
		if (unlikely(touch_debug_mask & DEBUG_JITTER))
			TOUCH_INFO_MSG("JitterFilter: ignored - jitter_count[%d] total_num[%d] bitmask[0x%x]\n",
					jitter_count, ts->ts_data.total_num, bit_mask);
		return -1;
	}

	for (id = 0; id < ts->pdata->caps->max_id; id++) {
		if (ts->ts_data.curr_data[id].status == FINGER_PRESSED) {
			ts->jitter_filter.his_data[id].x = ts->ts_data.curr_data[id].x_position;
			ts->jitter_filter.his_data[id].y = ts->ts_data.curr_data[id].y_position;
		}
	}

	ts->jitter_filter.id_mask = new_id_mask;

	return 0;
}
EXPORT_SYMBOL(jitter_filter_func);

/* touch_init_func
 *
 * In order to reduce the booting-time,
 * we used delayed_work_queue instead of msleep or mdelay.
 */
static void touch_init_func(struct work_struct *work_init)
{
	struct lge_touch_data *ts =
			container_of(to_delayed_work(work_init), struct lge_touch_data, work_init);

	if (unlikely(touch_debug_mask & DEBUG_TRACE))
		TOUCH_DEBUG_MSG("\n");

	/* Specific device initialization */
	touch_ic_init(ts);
}

static struct sys_device lge_touch_sys_device;

char *touch_wakeup_gesture[2] = { "TOUCH_GESTURE_WAKEUP=WAKEUP", NULL };
static void touch_gesture_wakeup_func(struct work_struct *work_gesture_wakeup)
{
	struct lge_touch_data *ts =
		container_of(to_delayed_work(work_gesture_wakeup), struct lge_touch_data, work_gesture_wakeup);

	if (ts->fw_info.fw_upgrade.is_downloading == UNDER_DOWNLOADING) {
		TOUCH_INFO_MSG("touch_gesture_wakeup is not executed\n");
		return;
	}

	TOUCH_INFO_MSG("called touch_gesture_wakeup_func\n");
	mutex_lock(&ts->irq_work_mutex);
	mutex_lock(&i2c_suspend_lock);

	if (touch_device_func->data(ts->client, &ts->ts_data) < 0) {
		TOUCH_ERR_MSG("touch_gesture_wakeup_func get data fail\n");
	}

	mutex_unlock(&i2c_suspend_lock);
	mutex_unlock(&ts->irq_work_mutex);

	wake_lock_timeout(&touch_wake_lock, msecs_to_jiffies(3000));
}


static void touch_lock_func(struct work_struct *work_touch_lock)
{
	struct lge_touch_data *ts =
			container_of(to_delayed_work(work_touch_lock), struct lge_touch_data, work_touch_lock);

	if (unlikely(touch_debug_mask & DEBUG_TRACE))
		TOUCH_DEBUG_MSG("\n");

	ts->ts_data.state = DO_NOT_ANYTHING;
}

/* check_log_finger_changed
 *
 * Check finger state change for Debug
 */
static void check_log_finger_changed(struct lge_touch_data *ts, u8 total_num)
{
	u16 tmp_p = 0;
	u16 tmp_r = 0;
	u16 id = 0;

	if (ts->ts_data.prev_total_num != total_num) {
		/* Finger added */
		if (ts->ts_data.prev_total_num <= total_num) {
			for (id = 0; id < ts->pdata->caps->max_id; id++) {
				if (ts->ts_data.curr_data[id].status == FINGER_PRESSED
						&& ts->ts_data.prev_data[id].status == FINGER_RELEASED) {
					break;
				}
			}
		}
	}
	if (ts->ts_data.prev_total_num == total_num && total_num == 1) {
		/* Finger changed at one finger status - IC bug check */
		for (id = 0, tmp_p = 0; id < ts->pdata->caps->max_id; id++) {
			/* find pressed */
			if (ts->ts_data.curr_data[id].status == FINGER_PRESSED
					&& ts->ts_data.prev_data[id].status == FINGER_RELEASED) {
				tmp_p = id;
			}
			/* find released */
			if (ts->ts_data.curr_data[id].status == FINGER_RELEASED
					&& ts->ts_data.prev_data[id].status == FINGER_PRESSED) {
				tmp_r = id;
			}
		}

		if (tmp_p != tmp_r
				&& (ts->ts_data.curr_data[tmp_p].status
						!= ts->ts_data.prev_data[tmp_p].status)) {
			if (ts->lockscreen) {
				TOUCH_INFO_MSG("%d finger changed : <%d -> %d> x[XXXX] y[XXXX] z[%3d]\n",
							total_num, tmp_r, tmp_p,
							ts->ts_data.curr_data[id].pressure);
			} else {
				TOUCH_INFO_MSG("%d finger changed : <%d -> %d> x[%4d] y[%4d] z[%3d]\n",
							total_num, tmp_r, tmp_p,
							ts->ts_data.curr_data[id].x_position,
							ts->ts_data.curr_data[id].y_position,
							ts->ts_data.curr_data[id].pressure);

			}
		}
	}
}

/* check_log_finger_released
 *
 * Check finger state change for Debug
 */
static void check_log_finger_released(struct lge_touch_data *ts)
{
	u16 id = 0;

	for (id = 0; id < ts->pdata->caps->max_id; id++) {
		if (ts->ts_data.prev_data[id].status == FINGER_PRESSED) {
			break;
		}
	}
}

/* touch_work_pre_proc
 *
 * Pre-process work at touch_work
 */
static int touch_work_pre_proc(struct lge_touch_data *ts)
{
	int ret = 0;
	atomic_dec(&ts->next_work);
	ts->ts_data.total_num = 0;
	ts->int_pin_state = 0;

	if (unlikely(ts->work_sync_err_cnt >= MAX_RETRY_COUNT)) {
		TOUCH_ERR_MSG("Work Sync Failed: Irq-pin has some unknown problems\n");
		return -EIO;
	}

#ifdef LGE_TOUCH_TIME_DEBUG
	do_gettimeofday(&t_debug[TIME_WORKQUEUE_START]);
#endif

	if (unlikely(touch_debug_mask & DEBUG_TRACE))
		TOUCH_DEBUG_MSG("\n");
	ret = touch_device_func->data(ts->client, &ts->ts_data);
	if (ret < 0) {
		TOUCH_ERR_MSG("get data fail\n");
		return ret;
	}

	if (likely(ts->pdata->role->operation_mode == INTERRUPT_MODE))
		ts->int_pin_state = gpio_get_value(ts->pdata->int_pin);

	/* Ghost finger solution */
	if (likely(ts->pdata->role->ghost_finger_solution_enable) && unlikely(ts->gf_ctrl.stage)) {
		if (ghost_finger_solution(ts)) {
			TOUCH_ERR_MSG("ghost_finger_solution was failed\n");
			return -EIO;
		}
	}

	/* Accuracy Solution */
	if (likely(ts->pdata->role->accuracy_filter_enable)) {
		if (accuracy_filter_func(ts) < 0)
			return -EAGAIN;;
	}

	/* Jitter Solution */
	if (likely(ts->pdata->role->jitter_filter_enable)) {
		if (jitter_filter_func(ts) < 0)
			return -EAGAIN;;
	}

	return 0;
}

/* touch_work_post_proc
 *
 * Post-process work at touch_work
 */
static void touch_work_post_proc(struct lge_touch_data *ts, int post_proc)
{
	int next_work = 0;

	if (post_proc >= WORK_POST_MAX)
		return;

	switch (post_proc) {
	case WORK_POST_OUT:
		if (likely(ts->pdata->role->operation_mode == INTERRUPT_MODE)) {
			next_work = atomic_read(&ts->next_work);

			if (unlikely(ts->int_pin_state != 1 && next_work <= 0)) {
				TOUCH_INFO_MSG("WARN: Interrupt pin is low - next_work: %d, try_count: %d]\n",
						next_work, ts->work_sync_err_cnt);
				post_proc = WORK_POST_ERR_RETRY;
				break;
			}
		}

#ifdef LGE_TOUCH_TIME_DEBUG
		do_gettimeofday(&t_debug[TIME_WORKQUEUE_END]);
		if (next_work)
			memset(t_debug, 0x0, sizeof(t_debug));
		time_profile_result(ts);
#endif

		ts->work_sync_err_cnt = 0;
		post_proc = WORK_POST_COMPLATE;
		break;

	case WORK_POST_ERR_RETRY:
		ts->work_sync_err_cnt++;
		atomic_inc(&ts->next_work);
		queue_work(touch_wq, &ts->work);
		post_proc = WORK_POST_COMPLATE;
		break;

	case WORK_POST_ERR_CIRTICAL:
		ts->work_sync_err_cnt = 0;
		safety_reset(ts);
		touch_ic_init(ts);
		post_proc = WORK_POST_COMPLATE;
		break;

	default:
		post_proc = WORK_POST_COMPLATE;
		break;
	}

	if (post_proc != WORK_POST_COMPLATE)
		touch_work_post_proc(ts, post_proc);
}

/* touch_work_func_a
 *
 * HARD_TOUCH_KEY
 */
static void touch_work_func_a(struct work_struct *work)
{
	struct lge_touch_data *ts =
			container_of(work, struct lge_touch_data, work);
	u8 report_enable = 0;
	int ret = 0;

	mutex_lock(&ts->irq_work_mutex);

	if (ts->pdata->role->ghost_detection_enable) {
		if (trigger_baseline == 2) {
			ret = ghost_detect_solution(ts);
			trigger_baseline = 0;
			touch_device_func->data(ts->client, &ts->ts_data);
			goto out;
		}
	}

	ret = touch_work_pre_proc(ts);
	if (ret == -EIO)
		goto err_out_critical;
	else if (ret == -EAGAIN)
		goto out;
	else if (ret == -IGNORE_INTERRUPT) {
		mutex_unlock(&ts->irq_work_mutex);
		return;
	}

	/* Ghost detection solution */
	if (ts->pdata->role->ghost_detection_enable) {
		ret = ghost_detect_solution(ts);
		if (ret == NEED_TO_OUT)
			goto out;
		else if (ret == NEED_TO_INIT)
			goto err_out_init;
	}

	if (ts->pdata->role->palm_detect_mode && (ts->ts_data.palm || ts->ts_data.prev_palm)) {
		release_all_ts_event(ts);
		cancel_delayed_work_sync(&ts->work_touch_lock);

		goto out;
	}

	/* Finger handle */
	if (ts->ts_data.state != TOUCH_ABS_LOCK) {
		if (!ts->ts_data.total_num) {
			touch_asb_input_report(ts, FINGER_RELEASED);
			report_enable = 1;

			queue_delayed_work(touch_wq, &ts->work_touch_lock, msecs_to_jiffies(200));

			if (likely(touch_debug_mask & (DEBUG_BASE_INFO | DEBUG_ABS))) {
				if (ts->ts_data.prev_total_num)
					check_log_finger_released(ts);
			}

			ts->ts_data.prev_total_num = 0;

			/* Reset previous finger position data */
			memset(&ts->ts_data.prev_data, 0x0, sizeof(ts->ts_data.prev_data));
		} else if (ts->ts_data.total_num <= ts->pdata->caps->max_id) {
			cancel_delayed_work_sync(&ts->work_touch_lock);

			if (ts->gf_ctrl.stage == GHOST_STAGE_CLEAR || (ts->gf_ctrl.stage | GHOST_STAGE_1) || ts->gf_ctrl.stage == GHOST_STAGE_4)
				ts->ts_data.state = TOUCH_BUTTON_LOCK;

			/* key button cancel */
			if (ts->ts_data.prev_button.state == BUTTON_PRESSED && ts->ts_data.state == TOUCH_BUTTON_LOCK) {
				input_report_key(ts->input_dev, ts->ts_data.prev_button.key_code, BUTTON_CANCLED);

				if (likely(touch_debug_mask & (DEBUG_BUTTON | DEBUG_BASE_INFO)))
					TOUCH_INFO_MSG("Touch KEY[%d] is canceled\n",
							ts->ts_data.prev_button.key_code);

				memset(&ts->ts_data.prev_button, 0x0, sizeof(ts->ts_data.prev_button));
			}

			if (likely(touch_debug_mask & (DEBUG_BASE_INFO | DEBUG_ABS)))
				check_log_finger_changed(ts, ts->ts_data.total_num);

			ts->ts_data.prev_total_num = ts->ts_data.total_num;

			touch_asb_input_report(ts, FINGER_PRESSED);
			report_enable = 1;

			memcpy(ts->ts_data.prev_data, ts->ts_data.curr_data, sizeof(ts->ts_data.curr_data));
		}

		/* Reset finger position data */
		memset(&ts->ts_data.curr_data, 0x0, sizeof(ts->ts_data.curr_data));

		if (report_enable)
			input_sync(ts->input_dev);
	}

	/* Button handle */
	if (ts->ts_data.state != TOUCH_BUTTON_LOCK) {
		/* do not check when there is no pressed button at error case
		 * 	- if you check it, sometimes touch is locked becuase button pressed via IC error.
		 */
		if (ts->work_sync_err_cnt > 0
				&& ts->ts_data.prev_button.state == BUTTON_RELEASED) {
			/* Do nothing */
		} else {
			report_enable = 0;

			if (unlikely(touch_debug_mask & DEBUG_BUTTON))
				TOUCH_INFO_MSG("Cur. button -code: %d state: %d, Prev. button -code: %d state: %d\n",
						ts->ts_data.curr_button.key_code,
						ts->ts_data.curr_button.state,
						ts->ts_data.prev_button.key_code,
						ts->ts_data.prev_button.state);

			if (ts->ts_data.curr_button.state == BUTTON_PRESSED
					&& ts->ts_data.prev_button.state == BUTTON_RELEASED) {
				/* button pressed */
				cancel_delayed_work_sync(&ts->work_touch_lock);

				input_report_key(ts->input_dev, ts->ts_data.curr_button.key_code, BUTTON_PRESSED);

				if (likely(touch_debug_mask & (DEBUG_BUTTON | DEBUG_BASE_INFO)))
					TOUCH_INFO_MSG("Touch KEY[%d] is pressed\n",
							ts->ts_data.curr_button.key_code);

				memcpy(&ts->ts_data.prev_button, &ts->ts_data.curr_button,
						sizeof(ts->ts_data.curr_button));

				report_enable = 1;
			} else if (ts->ts_data.curr_button.state == BUTTON_PRESSED
					&& ts->ts_data.prev_button.state == BUTTON_PRESSED
					&& ts->ts_data.prev_button.key_code != ts->ts_data.curr_button.key_code) {
				/* exception case - multi press button handle */
				queue_delayed_work(touch_wq, &ts->work_touch_lock, msecs_to_jiffies(200));

				/* release previous pressed button */
				input_report_key(ts->input_dev, ts->ts_data.prev_button.key_code, BUTTON_RELEASED);

				ts->ts_data.prev_button.state = BUTTON_RELEASED;

				if (likely(touch_debug_mask & (DEBUG_BUTTON | DEBUG_BASE_INFO)))
					TOUCH_INFO_MSG("Touch KEY[%d] is released\n",
							ts->ts_data.prev_button.key_code);

				report_enable = 1;
			} else if (ts->ts_data.curr_button.state == BUTTON_RELEASED /* button released */
					&& ts->ts_data.prev_button.state == BUTTON_PRESSED
					&& ts->ts_data.prev_button.key_code == ts->ts_data.curr_button.key_code) {
				/* button release */
				input_report_key(ts->input_dev, ts->ts_data.prev_button.key_code, BUTTON_RELEASED);

				if (likely(touch_debug_mask & (DEBUG_BUTTON | DEBUG_BASE_INFO)))
					TOUCH_INFO_MSG("Touch KEY[%d] is released\n",
							ts->ts_data.prev_button.key_code);

				memset(&ts->ts_data.prev_button, 0x0, sizeof(ts->ts_data.prev_button));
				memset(&ts->ts_data.curr_button, 0x0, sizeof(ts->ts_data.curr_button));

				report_enable = 1;
			}

			if (report_enable)
				input_sync(ts->input_dev);
		}
	}

out:
	touch_work_post_proc(ts, WORK_POST_OUT);
	mutex_unlock(&ts->irq_work_mutex);
	return;

err_out_critical:
	touch_work_post_proc(ts, WORK_POST_ERR_CIRTICAL);
	mutex_unlock(&ts->irq_work_mutex);
	return;

err_out_init:
	touch_work_post_proc(ts, WORK_POST_ERR_CIRTICAL);
	mutex_unlock(&ts->irq_work_mutex);

}

static bool is_in_section(struct rect rt, u16 x, u16 y)
{
	return x >= rt.left && x <= rt.right && y >= rt.top && y <= rt.bottom;
}

static u16 find_button(const struct t_data data, const struct section_info sc)
{
	int i;

	if (is_in_section(sc.panel, data.x_position, data.y_position))
		return KEY_PANEL;

	for (i = 0; i < sc.b_num; i++) {
		if (is_in_section(sc.button[i], data.x_position, data.y_position))
			return sc.b_name[i];
	}

	return KEY_BOUNDARY;
}

static bool check_cancel(u16 button, u16 x, u16 y, const struct section_info sc)
{
	int i;

	for (i = 0; i < sc.b_num; i++) {
		if (sc.b_name[i] == button)
			break;
	}

	if (i < sc.b_num) {
		if (is_in_section(sc.button_cancel[i], x, y))
			return false;
	}

	return true;
}

/* touch_work_func_b
 *
 * SOFT_TOUCH_KEY
 */
static void touch_work_func_b(struct work_struct *work)
{
	struct lge_touch_data *ts =
			container_of(work, struct lge_touch_data, work);

	u8  i;
	u8 op_mode = OP_NULL;
	u16 tmp_button = KEY_NULL;
	u16 id = 0;
	int ret = 0;

	mutex_lock(&ts->irq_work_mutex);

	ret = touch_work_pre_proc(ts);
	if (ret == -EIO)
		goto err_out_critical;
	else if (ret == -EAGAIN)
		goto out;

	if (ts->ts_data.total_num == 0)
		op_mode = OP_RELEASE;
	else if (ts->ts_data.state == TOUCH_BUTTON_LOCK || ts->ts_data.state == TOUCH_ABS_LOCK)
		op_mode = OP_LOCK;
	else if (ts->ts_data.total_num == 1)
		op_mode = OP_SINGLE;
	else
		op_mode = OP_MULTI;

	switch (op_mode) {
	case OP_RELEASE:
		if (ts->ts_data.prev_button.key_code == KEY_PANEL || ts->ts_data.prev_button.key_code == KEY_BOUNDARY
		    || ts->ts_data.prev_button.key_code == KEY_NULL)
			ts->ts_data.state = ABS_RELEASE;
		else
			ts->ts_data.state = BUTTON_RELEASE;

		ts->ts_data.curr_button.key_code = KEY_NULL;
		ts->ts_data.prev_total_num = 0;
		break;

	case OP_SINGLE:
		for (id = 0; id < ts->pdata->caps->max_id; id++) {
			if (ts->ts_data.curr_data[id].status == FINGER_PRESSED) {
				break;
			}
		}

		tmp_button = find_button(ts->ts_data.curr_data[id], ts->st_info);
		if (unlikely(touch_debug_mask & DEBUG_BUTTON))
			TOUCH_INFO_MSG("button_now [%d]\n", tmp_button);

		if (ts->ts_data.prev_button.key_code != KEY_NULL && ts->ts_data.prev_button.key_code != KEY_BOUNDARY) {
			if (ts->ts_data.prev_button.key_code == KEY_PANEL) {
				if (ts->ts_data.prev_button.key_code != tmp_button)
					ts->ts_data.state = ABS_RELEASE;
				else
					ts->ts_data.state = ABS_PRESS;
			} else {
				if (check_cancel(ts->ts_data.prev_button.key_code, ts->ts_data.curr_data[id].x_position,
						ts->ts_data.curr_data[id].y_position, ts->st_info))
					ts->ts_data.state = BUTTON_CANCEL;
				else
					ts->ts_data.state = DO_NOT_ANYTHING;
			}
		} else {
			if (tmp_button == KEY_PANEL || tmp_button == KEY_BOUNDARY)
				ts->ts_data.state = ABS_PRESS;
			else
				ts->ts_data.state = BUTTON_PRESS;
		}

		if (ts->ts_data.state == ABS_PRESS || ts->ts_data.state == BUTTON_PRESS)
			ts->ts_data.curr_button.key_code = tmp_button;
		else if (ts->ts_data.state == BUTTON_RELEASE ||
				ts->ts_data.state == BUTTON_CANCEL || ts->ts_data.state == ABS_RELEASE)
			ts->ts_data.curr_button.key_code = KEY_NULL;
		break;

	case OP_MULTI:
		if (ts->ts_data.prev_button.key_code && ts->ts_data.prev_button.key_code != KEY_PANEL
				&& ts->ts_data.prev_button.key_code != KEY_BOUNDARY)
			ts->ts_data.state = BUTTON_CANCEL;
		else
			ts->ts_data.state = ABS_PRESS;
		ts->ts_data.curr_button.key_code = KEY_PANEL;
		break;

	case OP_LOCK:
		for (id = 0; id < ts->pdata->caps->max_id; id++) {
			if (ts->ts_data.curr_data[id].status == FINGER_PRESSED) {
				if (ts->ts_data.curr_data[id].y_position < ts->pdata->caps->y_button_boundary) {
					ts->ts_data.curr_button.key_code = KEY_PANEL;
					ts->ts_data.state = ABS_PRESS;
				}
			}
		}
		break;

	default:
		break;
	}

	if (unlikely(touch_debug_mask & (DEBUG_ABS | DEBUG_BUTTON)))
		TOUCH_INFO_MSG("op_mode[%d] state[%d]\n", op_mode, ts->ts_data.state);

	switch (ts->ts_data.state) {
	case ABS_PRESS:
abs_report:
		i = 0;

		i = touch_asb_input_report(ts, FINGER_PRESSED);

		if (!i) {
			touch_asb_input_report(ts, FINGER_RELEASED);
			ts->ts_data.prev_total_num = 0;
		} else {
			if (likely(touch_debug_mask & DEBUG_ABS))
				check_log_finger_changed(ts, i);

			ts->ts_data.prev_total_num = i;
		}
		break;
	case ABS_RELEASE:
		touch_asb_input_report(ts, FINGER_RELEASED);
		break;
	case BUTTON_PRESS:
		input_report_key(ts->input_dev, ts->ts_data.curr_button.key_code, BUTTON_PRESSED);
			if (unlikely(touch_debug_mask & DEBUG_BUTTON))
			TOUCH_INFO_MSG("Touch KEY[%d] is pressed\n", ts->ts_data.curr_button.key_code);
		break;
	case BUTTON_RELEASE:
		input_report_key(ts->input_dev, ts->ts_data.prev_button.key_code, BUTTON_RELEASED);
			if (unlikely(touch_debug_mask & DEBUG_BUTTON))
			TOUCH_INFO_MSG("Touch KEY[%d] is released\n", ts->ts_data.prev_button.key_code);
		break;
	case BUTTON_CANCEL:
		input_report_key(ts->input_dev, ts->ts_data.prev_button.key_code, BUTTON_CANCLED);
		if (unlikely(touch_debug_mask & DEBUG_BUTTON))
			TOUCH_INFO_MSG("Touch KEY[%d] is canceled\n", ts->ts_data.prev_button.key_code);
		if (ts->ts_data.curr_data[0].y_position < ts->pdata->caps->y_button_boundary) {
			input_sync(ts->input_dev);
			goto abs_report;
		}
		break;
	case TOUCH_BUTTON_LOCK:
	case TOUCH_ABS_LOCK:
		goto out;
		break;
	default:
		break;
	}

	input_sync(ts->input_dev);

	if (likely(touch_debug_mask & DEBUG_BASE_INFO)) {
		if (ts->ts_data.state == ABS_RELEASE)
			check_log_finger_released(ts);
		if (ts->ts_data.state == BUTTON_RELEASE)
			TOUCH_INFO_MSG("touch_release : button[%d]\n", ts->ts_data.prev_button.key_code);
	}

	if (op_mode == OP_SINGLE && ts->ts_data.state == ABS_RELEASE)
			ts->ts_data.state = TOUCH_ABS_LOCK;

	if (ts->ts_data.state == BUTTON_CANCEL)
		ts->ts_data.state = TOUCH_BUTTON_LOCK;

	memcpy(ts->ts_data.prev_data, ts->ts_data.curr_data, sizeof(ts->ts_data.curr_data));
	memcpy(&ts->ts_data.prev_button, &ts->ts_data.curr_button, sizeof(ts->ts_data.curr_button));

out:
	touch_work_post_proc(ts, WORK_POST_OUT);
	mutex_unlock(&ts->irq_work_mutex);
	return;

err_out_critical:
	touch_work_post_proc(ts, WORK_POST_ERR_CIRTICAL);
	mutex_unlock(&ts->irq_work_mutex);
	return;
}

static void touch_work_func_c(struct work_struct *work)
{
	struct lge_touch_data *ts =
			container_of(work, struct lge_touch_data, work);
	u8 report_enable = 0;
	int ret = 0;

	mutex_lock(&ts->irq_work_mutex);

	ret = touch_work_pre_proc(ts);
	if (ret == -EIO)
		goto err_out_critical;
	else if (ret == -EAGAIN)
		goto out;
	else if (ret == -IGNORE_INTERRUPT) {
		mutex_unlock(&ts->irq_work_mutex);
		return;
	}

	if (!ts->ts_data.total_num) {
		touch_asb_input_report(ts, FINGER_RELEASED);
		report_enable = 1;

		if (likely(touch_debug_mask & (DEBUG_BASE_INFO | DEBUG_ABS))) {
			if (ts->ts_data.prev_total_num)
				check_log_finger_released(ts);
		}

		ts->ts_data.prev_total_num = 0;
		/* Reset previous finger position data */
		memset(&ts->ts_data.prev_data, 0x0, sizeof(ts->ts_data.prev_data));
	} else if (ts->ts_data.total_num <= ts->pdata->caps->max_id) {
		if (likely(touch_debug_mask & (DEBUG_BASE_INFO | DEBUG_ABS)))
			check_log_finger_changed(ts, ts->ts_data.total_num);

		ts->ts_data.prev_total_num = ts->ts_data.total_num;

		touch_asb_input_report(ts, FINGER_PRESSED);
		report_enable = 1;

		memcpy(ts->ts_data.prev_data, ts->ts_data.curr_data, sizeof(ts->ts_data.curr_data));
	}

	/* Reset finger position data */
	memset(&ts->ts_data.curr_data, 0x0, sizeof(ts->ts_data.curr_data));

	if (report_enable)
		input_sync(ts->input_dev);

out:
	touch_work_post_proc(ts, WORK_POST_OUT);
	mutex_unlock(&ts->irq_work_mutex);
	return;

err_out_critical:
	touch_work_post_proc(ts, WORK_POST_ERR_CIRTICAL);
	mutex_unlock(&ts->irq_work_mutex);
	return;
}

/* touch_fw_upgrade_func
 *
 * it used to upgrade the firmware of touch IC.
 */
static void touch_fw_upgrade_func(struct work_struct *work_fw_upgrade)
{
	struct lge_touch_data *ts =
			container_of(work_fw_upgrade, struct lge_touch_data, work_fw_upgrade);
	u8	saved_state = ts->curr_pwr_state;
	bool old_irq_mask = true;

    /*Disable IRQ before touch firmware upgrade */
    old_irq_mask = touch_irq_mask;

	if (likely(touch_debug_mask & DEBUG_FW_UPGRADE))
		TOUCH_INFO_MSG("START fw_upgrade_func, old_irq_mask = %s, touch_irq_maask = %s", old_irq_mask ? "TRUE" : "FALSE", touch_irq_mask ? "TRUE" : "FALSE");

	touch_disable_irq(ts->client->irq);

	if (unlikely(touch_debug_mask & DEBUG_TRACE))
		TOUCH_DEBUG_MSG("\n");

	if (touch_device_func->fw_upgrade == NULL) {
		TOUCH_INFO_MSG("There is no specific firmware upgrade function\n");
		goto out;
	}
	ts->fw_info.fw_upgrade.is_downloading = UNDER_DOWNLOADING;
	if (ts->curr_pwr_state == POWER_ON || ts->curr_pwr_state == POWER_WAKE) {
		if (ts->pdata->role->operation_mode)
			mutex_lock(&ts->irq_work_mutex);
		else
			hrtimer_cancel(&ts->timer);
	}
	if (ts->pdata->role->ghost_detection_enable) {
		hrtimer_cancel(&hr_touch_trigger_timer);
	}

	if (ts->curr_pwr_state == POWER_OFF) {
		touch_power_cntl(ts, POWER_ON);
		msleep(ts->pdata->role->booting_delay);
	}

	if (likely(touch_debug_mask & (DEBUG_FW_UPGRADE | DEBUG_BASE_INFO)))
		TOUCH_INFO_MSG("F/W upgrade - Start\n");

#ifdef LGE_TOUCH_TIME_DEBUG
	do_gettimeofday(&t_debug[TIME_FW_UPGRADE_START]);
#endif

	if (touch_device_func->fw_upgrade(ts->client, &ts->fw_info) < 0) {
		TOUCH_ERR_MSG("Firmware upgrade was failed\n");
		if (ts->pdata->role->operation_mode)
			mutex_unlock(&ts->irq_work_mutex);
		else
			hrtimer_start(&ts->timer, ktime_set(0, ts->pdata->role->report_period), HRTIMER_MODE_REL);

		goto err_out;
	}

#ifdef LGE_TOUCH_TIME_DEBUG
	do_gettimeofday(&t_debug[TIME_FW_UPGRADE_END]);
#endif

	touch_power_cntl(ts, POWER_OFF);

	if (saved_state != POWER_OFF) {
		touch_power_cntl(ts, POWER_ON);
		msleep(ts->pdata->role->booting_delay);

		if (ts->pdata->role->operation_mode)
			mutex_unlock(&ts->irq_work_mutex);
		else
			hrtimer_start(&ts->timer, ktime_set(0, ts->pdata->role->report_period), HRTIMER_MODE_REL);

		touch_ic_init(ts);

		if (saved_state == POWER_WAKE || saved_state == POWER_SLEEP)
			touch_power_cntl(ts, saved_state);
	}

	if (likely(touch_debug_mask & (DEBUG_FW_UPGRADE | DEBUG_BASE_INFO)))
		TOUCH_INFO_MSG("F/W upgrade - Finish\n");

#ifdef LGE_TOUCH_TIME_DEBUG
	do_gettimeofday(&t_debug[TIME_FW_UPGRADE_END]);

	if (touch_time_debug_mask & DEBUG_TIME_FW_UPGRADE
			|| touch_time_debug_mask & DEBUG_TIME_PROFILE_ALL) {
		TOUCH_INFO_MSG("FW upgrade time is under %3lu.%06lusec\n",
				get_time_interval(t_debug[TIME_FW_UPGRADE_END].tv_sec, t_debug[TIME_FW_UPGRADE_START].tv_sec),
				get_time_interval(t_debug[TIME_FW_UPGRADE_END].tv_usec, t_debug[TIME_FW_UPGRADE_START].tv_usec));
	}
#endif

	goto out;

err_out:
	mutex_lock(&ts->irq_work_mutex);
	safety_reset(ts);
	mutex_unlock(&ts->irq_work_mutex);
	touch_ic_init(ts);

out:

	memset(&ts->fw_info.fw_upgrade, 0, sizeof(ts->fw_info.fw_upgrade));
	/*Enable IRQ after touch firmware upgrade if IRQ has been enabled. */
    if (old_irq_mask) {
		touch_enable_irq(ts->client->irq);
		TOUCH_INFO_MSG("FW Upgrade finish and enable irq again\n");
    }

	return;
}

/* touch_irq_handler
 *
 * When Interrupt occurs, it will be called before touch_thread_irq_handler.
 *
 * return
 * IRQ_HANDLED: touch_thread_irq_handler will not be called.
 * IRQ_WAKE_THREAD: touch_thread_irq_handler will be called.
 */
static irqreturn_t touch_irq_handler(int irq, void *dev_id)
{
	struct lge_touch_data *ts = (struct lge_touch_data *)dev_id;

	if (unlikely(atomic_read(&ts->device_init) != 1))
	return IRQ_HANDLED;

#ifdef LGE_TOUCH_TIME_DEBUG
	do_gettimeofday(&t_debug[TIME_ISR_START]);
#endif

	atomic_inc(&ts->next_work);
	return IRQ_WAKE_THREAD;
}

/* touch_thread_irq_handler
 *
 * 1. disable irq.
 * 2. enqueue the new work.
 * 3. enalbe irq.
 */
static irqreturn_t touch_thread_irq_handler(int irq, void *dev_id)
{
	struct lge_touch_data *ts = (struct lge_touch_data *)dev_id;
#ifdef LGE_TOUCH_TIME_DEBUG
	do_gettimeofday(&t_debug[TIME_THREAD_ISR_START]);
#endif

	if (ts_suspend && touch_gesture_enable) {
		TOUCH_INFO_MSG("touch_thread_irq_handler : %d %d\n", ts_suspend, touch_gesture_enable);
		TOUCH_INFO_MSG("gesture wakeup\n");
		queue_delayed_work(touch_wq, &ts->work_gesture_wakeup, msecs_to_jiffies(0));
		return IRQ_HANDLED;
	}
	queue_work(touch_wq, &ts->work);

	return IRQ_HANDLED;
}

/* touch_timer_handler
 *
 * it will be called when timer interrupt occurs.
 */
static enum hrtimer_restart touch_timer_handler(struct hrtimer *timer)
{
	struct lge_touch_data *ts =
			container_of(timer, struct lge_touch_data, timer);

	atomic_inc(&ts->next_work);
	queue_work(touch_wq, &ts->work);
	hrtimer_start(&ts->timer,
			ktime_set(0, ts->pdata->role->report_period), HRTIMER_MODE_REL);

	return HRTIMER_NORESTART;
}

/* check_platform_data
 *
 * check-list
 * 1. Null Pointer
 * 2. lcd, touch screen size
 * 3. button support
 * 4. operation mode
 * 5. power module
 * 6. report period
 */
static int check_platform_data(struct touch_platform_data *pdata)
{
	if (unlikely(touch_debug_mask & DEBUG_TRACE))
		TOUCH_DEBUG_MSG("\n");

	if (!pdata)
		return -1;

	if (!pdata->caps || !pdata->role || !pdata->pwr)
		return -1;

	if (!pdata->caps->lcd_x || !pdata->caps->lcd_y || !pdata->caps->x_max || !pdata->caps->y_max) {
		TOUCH_ERR_MSG("lcd_x, lcd_y, x_max, y_max are should be defined\n");
		return -1;
	}

	if (pdata->caps->button_support) {
		if (!pdata->role->key_type) {
			TOUCH_ERR_MSG("button_support = 1, but key_type is not defined\n");
			return -1;
		}

		if (!pdata->caps->y_button_boundary) {
			if (pdata->role->key_type == TOUCH_HARD_KEY)
				pdata->caps->y_button_boundary = pdata->caps->y_max;
			else
				pdata->caps->y_button_boundary =
						(pdata->caps->lcd_y * pdata->caps->x_max) / pdata->caps->lcd_x;
		}

		if (pdata->caps->button_margin < 0 || pdata->caps->button_margin > 49) {
			pdata->caps->button_margin = 10;
			TOUCH_ERR_MSG("0 < button_margin < 49, button_margin is set 10 by force\n");
		}
	}

	if (!pdata->role->operation_mode) {
		if (!pdata->role->report_period) {
			TOUCH_ERR_MSG("polling mode needs report_period\n");
			return -1;
		}
	}

	if (pdata->role->suspend_pwr == POWER_OFF || pdata->role->suspend_pwr == POWER_SLEEP) {
		if (pdata->role->suspend_pwr == POWER_OFF)
			pdata->role->resume_pwr = POWER_ON;
		else
			pdata->role->resume_pwr = POWER_WAKE;
	} else {
		TOUCH_ERR_MSG("suspend_pwr = POWER_OFF or POWER_SLEEP\n");
	}

	if (pdata->pwr->use_regulator) {
		if (!pdata->pwr->vdd[0] || !pdata->pwr->vio[0]) {
			TOUCH_ERR_MSG("you should assign the name of vdd and vio\n");
			return -1;
		}
	} else {
		if (!pdata->pwr->power) {
			TOUCH_ERR_MSG("you should assign the power-control-function\n");
			return -1;
		}
	}

	if (pdata->role->report_period == 0)
		pdata->role->report_period = 12500000;

	if (pdata->caps->max_id > MAX_FINGER)
		pdata->caps->max_id = MAX_FINGER;

	return 0;
}

/* get_section
 *
 * it calculates the area of touch-key, automatically.
 */
void get_section(struct section_info *sc, struct touch_platform_data *pdata)
{
	int i;

	sc->panel.left = 0;
	sc->panel.right = pdata->caps->x_max;
	sc->panel.top = 0;
	sc->panel.bottom = pdata->caps->y_button_boundary;

	if (pdata->caps->number_of_button != 0)
		sc->b_width  = pdata->caps->x_max / pdata->caps->number_of_button;
	else
		sc->b_width  = 0; /* No button! */

	sc->b_margin = sc->b_width * pdata->caps->button_margin / 100;
	sc->b_inner_width = sc->b_width - (2*sc->b_margin);
	sc->b_height = pdata->caps->y_max - pdata->caps->y_button_boundary;
	sc->b_num = pdata->caps->number_of_button;

	for (i = 0; i < sc->b_num; i++) {
		sc->button[i].left = i * (pdata->caps->x_max / pdata->caps->number_of_button) + sc->b_margin;
		sc->button[i].right = sc->button[i].left + sc->b_inner_width;
		sc->button[i].top = pdata->caps->y_button_boundary + 1;
		sc->button[i].bottom = pdata->caps->y_max;

		sc->button_cancel[i].left = sc->button[i].left - (2*sc->b_margin) >= 0 ?
				sc->button[i].left - (2*sc->b_margin) : 0;
		sc->button_cancel[i].right = sc->button[i].right + (2*sc->b_margin) <= pdata->caps->x_max ?
				sc->button[i].right + (2*sc->b_margin) : pdata->caps->x_max;
		sc->button_cancel[i].top = sc->button[i].top;
		sc->button_cancel[i].bottom = sc->button[i].bottom;

		sc->b_name[i] = pdata->caps->button_name[i];
	}
}

/* Sysfs
 *
 * For debugging easily, we added some sysfs.
 */
static ssize_t show_platform_data(struct lge_touch_data *ts, char *buf)
{
	struct touch_platform_data *pdata = ts->pdata;
	int ret = 0;

	ret = sprintf(buf, "====== Platform data ======\n");
	ret += sprintf(buf+ret, "int_pin[%d] reset_pin[%d]\n", pdata->int_pin, pdata->reset_pin);
	ret += sprintf(buf+ret, "default panel spec[%s]\n", pdata->panel_spec);
	ret += sprintf(buf+ret, "caps:\n");
	ret += sprintf(buf+ret, "\tbutton_support        = %d\n", pdata->caps->button_support);
	ret += sprintf(buf+ret, "\ty_button_boundary     = %d\n", pdata->caps->y_button_boundary);
	ret += sprintf(buf+ret, "\tbutton_margin         = %d\n", pdata->caps->button_margin);
	ret += sprintf(buf+ret, "\tnumber_of_button      = %d\n", pdata->caps->number_of_button);
	ret += sprintf(buf+ret, "\tbutton_name           = %d, %d, %d, %d\n", pdata->caps->button_name[0],
			pdata->caps->button_name[1], pdata->caps->button_name[2], pdata->caps->button_name[3]);
	ret += sprintf(buf+ret, "\tis_width_supported    = %d\n", pdata->caps->is_width_supported);
	ret += sprintf(buf+ret, "\tis_pressure_supported = %d\n", pdata->caps->is_pressure_supported);
	ret += sprintf(buf+ret, "\tis_id_supported       = %d\n", pdata->caps->is_id_supported);
	ret += sprintf(buf+ret, "\tmax_width             = %d\n", pdata->caps->max_width);
	ret += sprintf(buf+ret, "\tmax_pressure          = %d\n", pdata->caps->max_pressure);
	ret += sprintf(buf+ret, "\tmax_id                = %d\n", pdata->caps->max_id);
	ret += sprintf(buf+ret, "\tx_max                 = %d\n", pdata->caps->x_max);
	ret += sprintf(buf+ret, "\ty_max                 = %d\n", pdata->caps->y_max);
	ret += sprintf(buf+ret, "\tlcd_x                 = %d\n", pdata->caps->lcd_x);
	ret += sprintf(buf+ret, "\tlcd_y                 = %d\n", pdata->caps->lcd_y);
	ret += sprintf(buf+ret, "role:\n");
	ret += sprintf(buf+ret, "\toperation_mode        = %d\n", pdata->role->operation_mode);
	ret += sprintf(buf+ret, "\tkey_type              = %d\n", pdata->role->key_type);
	ret += sprintf(buf+ret, "\treport_mode           = %d\n", pdata->role->report_mode);
	ret += sprintf(buf+ret, "\tdelta_pos_threshold   = %d\n", pdata->role->delta_pos_threshold);
	ret += sprintf(buf+ret, "\torientation           = %d\n", pdata->role->orientation);
	ret += sprintf(buf+ret, "\treport_period         = %d\n", pdata->role->report_period);
	ret += sprintf(buf+ret, "\tbooting_delay         = %d\n", pdata->role->booting_delay);
	ret += sprintf(buf+ret, "\treset_delay           = %d\n", pdata->role->reset_delay);
	ret += sprintf(buf+ret, "\tsuspend_pwr           = %d\n", pdata->role->suspend_pwr);
	ret += sprintf(buf+ret, "\tresume_pwr            = %d\n", pdata->role->resume_pwr);
	ret += sprintf(buf+ret, "\tirqflags              = 0x%lx\n", pdata->role->irqflags);
	ret += sprintf(buf+ret, "\tta debouncing		 = %d\n", pdata->role->ta_debouncing_count);
	ret += sprintf(buf+ret, "\tta debouncing finger num  = %d\n", pdata->role->ta_debouncing_finger_num);
	ret += sprintf(buf+ret, "\tghost_detection_enable= %d\n", pdata->role->ghost_detection_enable);
	ret += sprintf(buf+ret, "pwr:\n");
	ret += sprintf(buf+ret, "\tuse_regulator         = %d\n", pdata->pwr->use_regulator);
	ret += sprintf(buf+ret, "\tvdd                   = %s\n", pdata->pwr->vdd);
	ret += sprintf(buf+ret, "\tvdd_voltage           = %d\n", pdata->pwr->vdd_voltage);
	ret += sprintf(buf+ret, "\tvio                   = %s\n", pdata->pwr->vio);
	ret += sprintf(buf+ret, "\tvio_voltage           = %d\n", pdata->pwr->vio_voltage);
	ret += sprintf(buf+ret, "\tpower                 = %s\n", pdata->pwr->power ? "YES" : "NO");
	return ret;
}

/* show_fw_info
 *
 * show only the firmware information
 */
static ssize_t show_fw_info(struct lge_touch_data *ts, char *buf)
{
	ssize_t ret = 0;

	ret = touch_device_func->sysfs(ts->client, buf, SYSFS_SYNAPTICS_VERSION_SHOW, &ts->fw_info);
	return ret;
}

static ssize_t show_fw_version(struct lge_touch_data *ts, char *buf)
{
	ssize_t ret = 0;

	ret = touch_device_func->sysfs(ts->client, buf, SYSFS_SYNAPTICS_FW_VERSION_SHOW, &ts->fw_info);
	return ret;
}

/* store_fw_upgrade
 *
 * User can upgrade firmware, anytime, using this module.
 * Also, user can use both binary-img(SDcard) and header-file(Kernel image).
 */
static ssize_t store_fw_upgrade(struct lge_touch_data *ts, const char *buf, size_t count)
{
	int value = 0;
	int repeat = 0;
	static char path[256] = {0};

	memset(path, 0, 256);
	sscanf(buf, "%d %s", &value, path);

	TOUCH_INFO_MSG("Firmware image path: %s\n", path[0] != 0 ? path : "Internal");

	if (value) {
		for (repeat = 0; repeat < value; repeat++) {
			/* sync for n-th repeat test */
			while (ts->fw_info.fw_upgrade.is_downloading)
			;

			msleep(ts->pdata->role->booting_delay * 2);
			TOUCH_INFO_MSG("Firmware image upgrade: No.%d\n", repeat+1);

			/* for n-th repeat test - because ts->fw_info.fw_upgrade is setted 0 after FW upgrade */
			if (path[0] == 0)
				ts->fw_info.fw_upgrade.fw_path = ts->pdata->inbuilt_fw_name;
			else
				ts->fw_info.fw_upgrade.fw_path = (char *) path;

			TOUCH_INFO_MSG("Firmware image path (force update): %s\n", ts->fw_info.fw_upgrade.fw_path);

			/* set downloading flag for sync for n-th test */
			ts->fw_info.fw_upgrade.is_downloading = UNDER_DOWNLOADING;
			ts->fw_info.fw_upgrade.fw_force_upgrade = 1;

			queue_work(touch_wq, &ts->work_fw_upgrade);
		}

		/* sync for fw_upgrade test */
		while (ts->fw_info.fw_upgrade.is_downloading)
		;
	}

	return count;
}

/* show_fw_ver
 *
 * show only firmware version.
 * It will be used for AT-COMMAND
 */
static ssize_t show_fw_ver(struct lge_touch_data *ts, char *buf)
{
	int ret = 0;

	ret = touch_device_func->sysfs(ts->client, buf, SYSFS_SYNAPTICS_ATCMD_VERSION_SHOW, &ts->fw_info);
	return ret;
}

/* show_section_info
 *
 * User can check the information of touch-key-area, using this module.
 */
static ssize_t show_section_info(struct lge_touch_data *ts, char *buf)
{
	int ret = 0;
	int i;

	ret = sprintf(buf, "====== Section Info ======\n");

	ret += sprintf(buf+ret, "Panel = [%4d,%4d,%4d,%4d]\n", ts->st_info.panel.left,
			ts->st_info.panel.right, ts->st_info.panel.top, ts->st_info.panel.bottom);

	if (ts->pdata->role->key_type == TOUCH_SOFT_KEY) {
		for (i = 0; i < ts->st_info.b_num; i++) {
			ret += sprintf(buf+ret, "Button[%4d] = [%4d,%4d,%4d,%4d]\n",
					ts->st_info.b_name[i], ts->st_info.button[i].left,
					ts->st_info.button[i].right, ts->st_info.button[i].top, ts->st_info.button[i].bottom);
		}
		for (i = 0; i < ts->st_info.b_num; i++) {
			ret += sprintf(buf+ret, "Button_cancel[%4d] = [%4d,%4d,%4d,%4d]\n",
					ts->st_info.b_name[i], ts->st_info.button_cancel[i].left,
					ts->st_info.button_cancel[i].right, ts->st_info.button_cancel[i].top,
					ts->st_info.button_cancel[i].bottom);
		}
		ret += sprintf(buf+ret, "button_width        = %d\n", ts->st_info.b_width);
		ret += sprintf(buf+ret, "button_height       = %d\n", ts->st_info.b_height);
		ret += sprintf(buf+ret, "button_inner_width  = %d\n", ts->st_info.b_inner_width);
		ret += sprintf(buf+ret, "button_margin       = %d\n", ts->st_info.b_margin);
		ret += sprintf(buf+ret, "button_number       = %d\n", ts->st_info.b_num);
	}

	return ret;
}

/* store_ts_reset
 *
 * Reset the touch IC.
 */

static ssize_t store_ts_reset(struct lge_touch_data *ts, const char *buf, size_t count)
{
	unsigned char string[5];
	u8 saved_state = ts->curr_pwr_state;
	int ret = 0;

	sscanf(buf, "%s", string);

	if (ts->pdata->role->operation_mode)
		mutex_lock(&ts->irq_work_mutex);
	else
		hrtimer_cancel(&ts->timer);
		if (ts->pdata->role->ghost_detection_enable) {
			hrtimer_cancel(&hr_touch_trigger_timer);
		}

	cancel_work_sync(&ts->work);
	cancel_delayed_work_sync(&ts->work_init);
	if (ts->pdata->role->key_type == TOUCH_HARD_KEY)
		cancel_delayed_work_sync(&ts->work_touch_lock);

	release_all_ts_event(ts);

	if (saved_state == POWER_ON || saved_state == POWER_WAKE) {
		if (!strncmp(string, "soft", 4)) {
			if (touch_device_func->ic_ctrl) {
				TOUCH_INFO_MSG("SOFT RESET");
				touch_device_func->ic_ctrl(ts->client, IC_CTRL_RESET_CMD, 0);
			} else {
				TOUCH_INFO_MSG("There is no specific IC control function\n");
			}
		} else if (!strncmp(string, "pin", 3)) {
			if (gpio_is_valid(ts->pdata->reset_pin)) {
				TOUCH_INFO_MSG("PIN RESET");
				gpio_set_value(ts->pdata->reset_pin, 0);
				msleep(ts->pdata->role->reset_delay);
				gpio_set_value(ts->pdata->reset_pin, 1);
			} else
				TOUCH_INFO_MSG("There is no reset pin\n");
		} else if (!strncmp(string, "vdd", 3)) {
			TOUCH_INFO_MSG("VDD RESET");
			touch_power_cntl(ts, POWER_OFF);
			touch_power_cntl(ts, POWER_ON);
		} else {
			TOUCH_INFO_MSG("Usage: echo [soft | pin | vdd] > ts_reset\n");
			TOUCH_INFO_MSG(" - soft : reset using IC register setting\n");
			TOUCH_INFO_MSG(" - soft : reset using reset pin\n");
			TOUCH_INFO_MSG(" - hard : reset using VDD\n");
		}

		if (ret < 0) {
			TOUCH_ERR_MSG("reset fail\n");
		} else {
			atomic_set(&ts->device_init, 0);
		}

		msleep(ts->pdata->role->booting_delay);

	} else
		TOUCH_INFO_MSG("Touch is suspend state. Don't need reset\n");

	if (ts->pdata->role->operation_mode)
		mutex_unlock(&ts->irq_work_mutex);
	else
		hrtimer_start(&ts->timer,
			ktime_set(0, ts->pdata->role->report_period), HRTIMER_MODE_REL);

	if (saved_state == POWER_ON || saved_state == POWER_WAKE)
		touch_ic_init(ts);
	return count;
}

/* ic_register_ctrl
 *
 * User can see any register of touch_IC
 */
static ssize_t ic_register_ctrl(struct lge_touch_data *ts, const char *buf, size_t count)
{
	unsigned char string[6];
	int page = 0;
	int reg = 0;
	int value = 0;
	int ret = 0;
	u32 write_data;
	int i = 0;
	int offset = 0;
	unsigned char *r_mem = NULL;

	sscanf(buf, "%s %d %d %d %d", string, &page, &reg, &value, &offset);

	if (touch_device_func->ic_ctrl) {
		if (ts->curr_pwr_state == POWER_ON || ts->curr_pwr_state == POWER_WAKE) {
			if (!strncmp(string, "read", 4)) {
				write_data = ((0xFF & page) << 8) | (0xFF & reg);
				do {
					ret = touch_device_func->ic_ctrl(ts->client, IC_CTRL_READ, write_data);
					if (ret >= 0) {
						TOUCH_INFO_MSG("register[0x%04X] = 0x%02X\n", write_data, ret);
					} else {
						TOUCH_INFO_MSG("cannot read register[0x%02X%02X]\n", page, reg);
					}
					write_data++;
				} while (--value > 0);
			} else if (!strncmp(string, "write", 4)) {
				write_data = ((0xFF & page) << 16) | ((0xFF & reg) << 8) | (0xFF & value);
				ret = touch_device_func->ic_ctrl(ts->client, IC_CTRL_WRITE, write_data);
				if (ret >= 0) {
					TOUCH_INFO_MSG("register[0x%02X%02X] is set to 0x%02X\n", page, reg, value);
				} else {
					TOUCH_INFO_MSG("cannot write register[0x%02X%02X]\n", page, reg);
				}
			} else if (!strncmp(string, "rall", 4)) {
				r_mem = kzalloc(sizeof(char) * value, GFP_KERNEL);
				if (r_mem == NULL) {
					TOUCH_ERR_MSG("%d bytes allocation fail!", value);
				} else {
					if (touch_i2c_read(ts->client, reg, value, r_mem) < 0) {
						TOUCH_ERR_MSG("%d bytes read fail!", value);
					} else {
						do {
							TOUCH_INFO_MSG("register[0x%x+%d] = 0x%x\n", reg, i, *(r_mem+i));
							i++;
						} while (--value > 0);
					}
					if (r_mem != NULL)
						kfree(r_mem);
				}
			} else if (!strncmp(string, "wall", 4)) {
				r_mem = kzalloc(sizeof(char) * (offset + 1), GFP_KERNEL);

				if (r_mem == NULL) {
					TOUCH_ERR_MSG("%d bytes allocation fail!", (offset + 1));
				} else {
					if (touch_i2c_read(ts->client, reg, (offset + 1), r_mem) < 0) {
						TOUCH_ERR_MSG("%d bytes read fail!", (offset + 1));
					} else {
						*(r_mem + offset) = value;
						ret = touch_i2c_write(ts->client, reg, (offset + 1), r_mem);
						if (ret >= 0) {
							do {
								TOUCH_INFO_MSG("register[0x%x+%d] is set to 0x%x\n", reg, i, *(r_mem+i));
								i++;
							} while (--offset >= 0);
						} else {
							TOUCH_INFO_MSG("cannot write register[0x%x] ~ [0x%x+%d]\n", reg, reg, offset);
						}
						if (r_mem != NULL)
							kfree(r_mem);
					}
				}
			} else{
				TOUCH_INFO_MSG("Usage: echo [read | write] page_num reg_num value > ic_rw\n");
				TOUCH_INFO_MSG(" - page_num : register page\n");
				TOUCH_INFO_MSG(" - reg_num : register address\n");
				TOUCH_INFO_MSG(" - value [read] : number of register starting form reg_num\n");
				TOUCH_INFO_MSG(" - value [write] : set value into reg_num\n");
				TOUCH_INFO_MSG("Usage: echo [read | rall | write | wall] reg_num value offset > ic_rw\n");
				TOUCH_INFO_MSG(" - value [rall] : number of register starting form reg_num\n");
			}
		} else
			TOUCH_INFO_MSG("state=[suspend]. we cannot use I2C, now\n");
	} else
		TOUCH_INFO_MSG("There is no specific IC control function\n");

	return count;
}

/* store_keyguard_info
 *
 * This function is related with Keyguard in framework.
 * We can prevent the ghost-finger problem, using this function.
 * If you need more information, see the 'ghost_finger_solution' function.
 */
static ssize_t store_keyguard_info(struct lge_touch_data *ts, const char *buf, size_t count)
{
	int value;
	sscanf(buf, "%d", &value);

	TOUCH_INFO_MSG("%s \n", __func__);

	if (value == KEYGUARD_ENABLE)
		ts->gf_ctrl.stage = GHOST_STAGE_1 | GHOST_STAGE_2 | GHOST_STAGE_4;
	else if (value == KEYGUARD_RESERVED)
		ts->gf_ctrl.stage &= ~GHOST_STAGE_2;

	if (touch_debug_mask & DEBUG_GHOST || touch_debug_mask & DEBUG_BASE_INFO) {
		TOUCH_INFO_MSG("ghost_stage [0x%x]\n", ts->gf_ctrl.stage);
		if (value == KEYGUARD_RESERVED)
			TOUCH_INFO_MSG("ghost_stage2 : cleared[0x%x]\n", ts->gf_ctrl.stage);
	}

	return count;
}

static ssize_t store_lockscreen_info(struct lge_touch_data *ts, const char *buf, size_t count)
{
	sscanf(buf, "%d", &ts->lockscreen);

	TOUCH_INFO_MSG("%s : %d \n", __func__, ts->lockscreen);

	return count;
}


/* show_virtual_key
 *
 * /sys/devices/virtual/input/virtualkeys
 * for use the virtual_key supported by Google
 *
 * 0x01:key_code:center_x:center_y:x_width:y_width
 * 0x01 = start_point
 */
static ssize_t show_virtual_key(struct lge_touch_data *ts, char *buf)
{
	int i = 0;
	int ret = 0;

	u32 center_x = 0;
	u32 center_y = 0;

#if defined(CONFIG_MACH_MSM8926_JAGN_KR) || defined(CONFIG_MACH_MSM8226_JAG3GDS_GLOBAL_COM) || defined(CONFIG_MACH_MSM8226_JAG3GSS_GLOBAL_COM) \
	|| defined(CONFIG_MACH_MSM8926_VFP_KR) || defined(CONFIG_MACH_MSM8926_JAGNM_GLOBAL_COM) \
	|| defined(CONFIG_MACH_MSM8926_JAGDSNM_CN)/* to avoid division by zero in kernel */

	if (ts->pdata->caps->number_of_button != 0) {
		center_x = (ts->pdata->caps->x_max / (ts->pdata->caps->number_of_button * 2));
	}
#else
	center_x = (ts->pdata->caps->x_max / (ts->pdata->caps->number_of_button * 2));
#endif
	center_y = (ts->pdata->caps->y_button_boundary + (ts->st_info.b_height / 2));

	/* Register sysfs for virtualkeys*/
	if (ts->pdata->caps->button_support && ts->pdata->role->key_type == VIRTUAL_KEY) {

		for (i = 0; i < ts->pdata->caps->number_of_button; i++) {
			if (i)
				ret += sprintf(buf+ret, ":");

			ret += sprintf(buf+ret, "0x01:%d:%d:%d:%d:%d", ts->pdata->caps->button_name[i],
						center_x * (i * 2 + 1), center_y,
						ts->st_info.b_inner_width, ts->st_info.b_height);
		}
		ret += sprintf(buf+ret, "\n");
		return ret;
	} else
		return -ENOSYS;

}

static ssize_t store_touch_gesture(struct lge_touch_data *ts, const char *buf, size_t count)
{
	int value;
	sscanf(buf, "%d", &value);

	mutex_lock(&ts->irq_work_mutex);
	touch_gesture_enable = value;
	mutex_unlock(&ts->irq_work_mutex);
	return count;
}

static ssize_t store_jitter_solution(struct lge_touch_data *ts, const char *buf, size_t count)
{
	int ret = 0;

	memset(&ts->jitter_filter, 0, sizeof(ts->jitter_filter));

	ret = sscanf(buf, "%d %d",
				&ts->pdata->role->jitter_filter_enable,
				&ts->jitter_filter.adjust_margin);

	return count;
}

static ssize_t store_accuracy_solution(struct lge_touch_data *ts, const char *buf, size_t count)
{
	int ret = 0;

	memset(&ts->accuracy_filter, 0, sizeof(ts->accuracy_filter));

	ret = sscanf(buf, "%d %d %d %d %d %d %d",
				&ts->pdata->role->accuracy_filter_enable,
				&ts->accuracy_filter.ignore_pressure_gap,
				&ts->accuracy_filter.delta_max,
				&ts->accuracy_filter.touch_max_count,
				&ts->accuracy_filter.max_pressure,
				&ts->accuracy_filter.direction_count,
				&ts->accuracy_filter.time_to_max_pressure);

	return count;
}

static ssize_t show_knock_on_type(struct lge_touch_data *ts, char *buf)
{
	int ret = 0;

	ret = sprintf(buf, "%d\n", ts->pdata->knock_on_type);
	return ret;
}

static ssize_t power_control_store(struct lge_touch_data *ts, const char *buf, size_t count)
{
	int cmd = 0;

	if (sscanf(buf, "%d", &cmd) != 1)
		return -EINVAL;

	switch (cmd) {
	case 1:
		if (ts->curr_pwr_state == POWER_OFF) {
			touch_power_cntl(ts, POWER_ON);
			msleep(ts->pdata->role->booting_delay);
			touch_ic_init(ts);
			if (ts->pdata->role->operation_mode)
				mutex_unlock(&ts->irq_work_mutex);
			else
				hrtimer_start(&ts->timer, ktime_set(0, ts->pdata->role->report_period), HRTIMER_MODE_REL);
		}
		break;

	case 0:
		if (ts->curr_pwr_state == POWER_ON) {
			if (ts->pdata->role->operation_mode)
				mutex_lock(&ts->irq_work_mutex);
			else
				hrtimer_cancel(&ts->timer);

			cancel_work_sync(&ts->work);
			cancel_delayed_work_sync(&ts->work_init);
			if (ts->pdata->role->key_type == TOUCH_HARD_KEY)
				cancel_delayed_work_sync(&ts->work_touch_lock);

			release_all_ts_event(ts);
			touch_power_cntl(ts, POWER_OFF);
			atomic_set(&ts->device_init, 0);
		}
		break;

	default:
		TOUCH_INFO_MSG("usage: echo [1|0] > control\n");
		TOUCH_INFO_MSG("  - 1: power on\n");
		TOUCH_INFO_MSG("  - 0: power off\n");
		break;
	}
	return count;
}

static ssize_t store_report_mode(struct lge_touch_data *ts, const char *buf, size_t count)
{
	int ret = 0;
	int report_mode, delta_pos;

	ret = sscanf(buf, "%d %d",
				&report_mode,
				&delta_pos);

	ts->pdata->role->report_mode = report_mode;
	ts->pdata->role->delta_pos_threshold = delta_pos;

	return count;
}

static ssize_t store_debouncing_count(struct lge_touch_data *ts, const char *buf, size_t count)
{
	int ret = 0;
	int debouncing_count = 0;

	ret = sscanf(buf, "%d", &debouncing_count);

	ts->pdata->role->ta_debouncing_count = debouncing_count;
	return count;
}

static ssize_t store_debouncing_finger_num(struct lge_touch_data *ts, const char *buf, size_t count)
{
	int ret = 0;
	int debouncing_finger_num = 0;

	ret = sscanf(buf, "%d", &debouncing_finger_num);

	ts->pdata->role->ta_debouncing_finger_num = debouncing_finger_num;
	return count;
}

static ssize_t store_ghost_detection_enable(struct lge_touch_data *ts, const char *buf, size_t count)
{
	int ret = 0;
	int ghost_detection_enable = 0;

	ret = sscanf(buf, "%d", &ghost_detection_enable);

	ts->pdata->role->ghost_detection_enable = ghost_detection_enable;
	return count;
}

static ssize_t show_sd(struct lge_touch_data *ts, char *buf)
{
	int ret = 0;
	ts->sd_status = 1;
	ret = touch_device_func->sysfs(ts->client, buf, SYSFS_SELF_DIAGNOSTIC_SHOW, &ts->fw_info);
	ts->sd_status = 0;
	return ret;
}

static ssize_t show_sd_status(struct lge_touch_data *ts, char *buf)
{
	int ret = 0;
	ret = snprintf(buf, PAGE_SIZE, "%d\n", ts->sd_status);
	return ret;
}

static ssize_t show_rawdata(struct lge_touch_data *ts, char *buf)
{
	int ret = 0;

	ret = touch_device_func->sysfs(ts->client, buf, SYSFS_RAWDATA_SHOW, NULL);
	return ret;
}

static ssize_t show_delta(struct lge_touch_data *ts, char *buf)
{
	int ret = 0;

	ret = touch_device_func->sysfs(ts->client, buf, SYSFS_DELTA_SHOW, NULL);
	return ret;
}
static ssize_t show_chstatus(struct lge_touch_data *ts, char *buf)
{
	int ret = 0;

	ret = touch_device_func->sysfs(ts->client, buf, SYSFS_CHSTATUS_SHOW, &ts->fw_info);
	return ret;
}

/* send_uevent
 *
 * It will be used to send u-event to Android-framework.
 */
static struct sys_device lge_touch_sys_device;
void send_uevent(char *string[2])
{
    kobject_uevent_env(&lge_touch_sys_device.kobj, KOBJ_CHANGE, string);
    TOUCH_DEBUG_MSG("uevent[%s]\n", string[0]);
}

/* send_uevent_lpwg
 *
 * It uses wake-lock in order to prevent entering the sleep-state,
 * during recognition or verification.
 */
#define VALID_LPWG_UEVENT_SIZE 2
static char *lpwg_uevent[VALID_LPWG_UEVENT_SIZE][2] = {
														{"TOUCH_GESTURE_WAKEUP=WAKEUP", NULL},
														{"TOUCH_GESTURE_WAKEUP=PASSWORD", NULL}
													};

void send_uevent_lpwg(struct i2c_client *client, int type)
{
    struct lge_touch_data *ts = i2c_get_clientdata(client);

    if (type > 0 && type <= VALID_LPWG_UEVENT_SIZE
		&& atomic_read(&ts->state.uevent_state) == UEVENT_IDLE) {
			atomic_set(&ts->state.uevent_state, UEVENT_BUSY);
			send_uevent(lpwg_uevent[type - 1]);
    }
}

/* update_status
 *
 * Other drivers can notify their status to touch driver.
 * Do not use 'i2c_client' in other function.
 */
struct state_info *state;
struct i2c_client *client_only_for_update_status;
void update_status(int code, int value)
{
	if (code == NOTIFY_TA_CONNECTION)
		atomic_set(&state->ta_state, value ? TA_CONNECTED : TA_DISCONNECTED);
	else if (code == NOTIFY_TEMPERATURE_CHANGE)
		atomic_set(&state->temperature_state, value);
	else if (code == NOTIFY_PROXIMITY)
		atomic_set(&state->proximity_state, value ? PROXIMITY_NEAR : PROXIMITY_FAR);
	else if (code == NOTIFY_HALL_IC)
		atomic_set(&state->hallic_state, value ? HALL_COVERED : HALL_NONE);

	TOUCH_DEBUG_MSG("code[%d] value[%d]\n", code, value);
}
EXPORT_SYMBOL(update_status);

/* Sysfs - lpwg_data (Low Power Wake-up Gesture)
 *
 * read : "x1 y1\n x2 y2\n ..."
 * write
 * 1 : ENABLE/DISABLE
 * 2 : LCD SIZE
 * 3 : ACTIVE AREA
 * 4 : TAP COUNT
 */
static struct point lpwg_data[MAX_POINT_SIZE_FOR_LPWG+1];
static ssize_t show_lpwg_data(struct lge_touch_data *ts, char *buf)
{
    int i = 0, ret = 0;

    if (touch_device_func->lpwg) {
		memset(lpwg_data, 0, sizeof(struct point)*MAX_POINT_SIZE_FOR_LPWG);
		TOUCH_DEBUG_MSG("\n");
		touch_device_func->lpwg(ts->client, LPWG_READ, 0, lpwg_data);
		for (i = 0; i < MAX_POINT_SIZE_FOR_LPWG; i++) {
			if (lpwg_data[i].x == -1 && lpwg_data[i].y == -1)
				break;
			ret += sprintf(buf+ret, "%d %d\n", lpwg_data[i].x, lpwg_data[i].y);
		}
	}
    return ret;
}

static ssize_t store_lpwg_data(struct lge_touch_data *ts, const char *buf, size_t count)
{
    int reply = 0;

    sscanf(buf, "%d", &reply);
    TOUCH_INFO_MSG("LPWG RESULT = %d ", reply);

    if (touch_device_func->lpwg) {
		touch_device_func->lpwg(ts->client, LPWG_REPLY, reply, NULL);
    }

    atomic_set(&ts->state.uevent_state, UEVENT_IDLE);

    return count;
}

/* Sysfs - lpwg_notify (Low Power Wake-up Gesture)
 *
 */
#if defined(CONFIG_FB)
static int touch_fb_suspend(struct device *device);
static int touch_fb_resume(struct device *device);
#endif
static ssize_t store_lpwg_notify(struct lge_touch_data *ts, const char *buf, size_t count)
{
    int type = 0;
    int value[4] = {0};

    sscanf(buf, "%d %d %d %d %d", &type, &value[0], &value[1], &value[2], &value[3]);
    TOUCH_INFO_MSG("touch notify type = %d , value[0] = %d, value[1] = %d, valeu[2] = %d, value[3] = %d ", type, value[0], value[1], value[2], value[3]);

	if (touch_device_func->lpwg) {
		switch (type) {
		case 1:
			touch_device_func->lpwg(ts->client, LPWG_ENABLE, value[0], NULL);
			mutex_lock(&ts->irq_work_mutex);
			if (value[0])
				touch_gesture_enable = 1;
			else
				touch_gesture_enable = 0;
			atomic_set(&ts->device_init, 1);
			mutex_unlock(&ts->irq_work_mutex);
			break;
		case 2:
			touch_device_func->lpwg(ts->client, LPWG_LCD_X, value[0], NULL);
			touch_device_func->lpwg(ts->client, LPWG_LCD_Y, value[1], NULL);
			break;
		case 3:
			touch_device_func->lpwg(ts->client, LPWG_ACTIVE_AREA_X1, value[0], NULL);
			touch_device_func->lpwg(ts->client, LPWG_ACTIVE_AREA_X2, value[1], NULL);
			touch_device_func->lpwg(ts->client, LPWG_ACTIVE_AREA_Y1, value[2], NULL);
			touch_device_func->lpwg(ts->client, LPWG_ACTIVE_AREA_Y2, value[3], NULL);
			break;
		case 4:
			mutex_lock(&ts->irq_work_mutex);
			touch_device_func->lpwg(ts->client, LPWG_TAP_COUNT, value[0], NULL);
			mutex_unlock(&ts->irq_work_mutex);
			break;
		case 6:
#if defined(CONFIG_FB)
			if (value[0] == 0) {
				if (touch_gesture_enable)
					touch_fb_suspend(&ts->client->dev);
				else if (!touch_gesture_enable)
					touch_device_func->suspend(ts->client);
			} else if (value[0] == 1) {
				touch_device_func->lpwg(ts->client, LPWG_MODE_CHANGE, value[0], NULL);
				touch_fb_resume(&ts->client->dev);
			}
#endif
			break;
		case 7:
			touch_device_func->lpwg(ts->client, LPWG_STATUS_BY_PROXI, value[0], NULL);
			mutex_lock(&ts->irq_work_mutex);
			touch_gesture_enable = 1;
			atomic_set(&ts->device_init, 1);
			mutex_unlock(&ts->irq_work_mutex);
			break;
		default:
			break;
		}
	}
    return count;
}

static ssize_t store_global_access_pixel(struct lge_touch_data *ts, const char *buf, size_t count)
{
	int ret = 0;
	int pixel = 0;

	ret = sscanf(buf, "%d", &pixel);
	if (ret < 0) {
		TOUCH_ERR_MSG("Error to write pixel data.....\n");
	}

	ts->pdata->global_access_pixel = pixel;

	TOUCH_INFO_MSG("SET global_access_pixel = %d\n", ts->pdata->global_access_pixel);
	return count;
}

static ssize_t show_global_access_pixel(struct lge_touch_data *ts, char *buf)
{
	int ret = 0;

	ret = sprintf(buf, "%d\n", ts->pdata->global_access_pixel);
	TOUCH_INFO_MSG("Get global access pixel = %d\n", ts->pdata->global_access_pixel);
	return ret;
}



static LGE_TOUCH_ATTR(platform_data, S_IRUGO | S_IWUSR, show_platform_data, NULL);
static LGE_TOUCH_ATTR(firmware, S_IRUGO | S_IWUSR, show_fw_info, store_fw_upgrade);
static LGE_TOUCH_ATTR(testmode_ver, S_IRUGO | S_IWUSR, show_fw_ver, NULL);
static LGE_TOUCH_ATTR(version, S_IRUGO | S_IWUSR, show_fw_version, NULL);
static LGE_TOUCH_ATTR(section, S_IRUGO | S_IWUSR, show_section_info, NULL);
static LGE_TOUCH_ATTR(reset, S_IRUGO | S_IWUSR, NULL, store_ts_reset);
static LGE_TOUCH_ATTR(ic_rw, S_IRUGO | S_IWUSR, NULL, ic_register_ctrl);
static LGE_TOUCH_ATTR(keyguard, S_IRUGO | S_IWUSR, NULL, store_keyguard_info);
static LGE_TOUCH_ATTR(lockscreen, S_IRUGO | S_IWUSR, NULL, store_lockscreen_info);
static LGE_TOUCH_ATTR(virtualkeys, S_IRUGO | S_IWUSR, show_virtual_key, NULL);
static LGE_TOUCH_ATTR(touch_gesture, S_IRUGO | S_IWUSR, NULL, store_touch_gesture);
static LGE_TOUCH_ATTR(jitter, S_IRUGO | S_IWUSR, NULL, store_jitter_solution);
static LGE_TOUCH_ATTR(accuracy, S_IRUGO | S_IWUSR, NULL, store_accuracy_solution);
static LGE_TOUCH_ATTR(knock_on_type, S_IRUGO | S_IWUSR, show_knock_on_type, NULL);
static LGE_TOUCH_ATTR(sd, S_IRUGO | S_IWUSR, show_sd, NULL);
static LGE_TOUCH_ATTR(sd_status, S_IRUGO | S_IWUSR, show_sd_status, NULL);
static LGE_TOUCH_ATTR(rawdata, S_IRUGO | S_IWUSR, show_rawdata, NULL);
static LGE_TOUCH_ATTR(report_mode, S_IRUGO | S_IWUSR, NULL, store_report_mode);
static LGE_TOUCH_ATTR(ta_debouncing_count, S_IRUGO | S_IWUSR, NULL, store_debouncing_count);
static LGE_TOUCH_ATTR(ta_debouncing_finger_num, S_IRUGO | S_IWUSR, NULL, store_debouncing_finger_num);
static LGE_TOUCH_ATTR(ghost_detection_enable, S_IRUGO | S_IWUSR, NULL, store_ghost_detection_enable);
static LGE_TOUCH_ATTR(delta, S_IRUGO | S_IWUSR, show_delta, NULL);
static LGE_TOUCH_ATTR(chstatus, S_IRUGO | S_IWUSR, show_chstatus, NULL);
static LGE_TOUCH_ATTR(power_control, S_IRUGO | S_IWUSR, NULL, power_control_store);
static LGE_TOUCH_ATTR(global_access_pixel, S_IRUGO | S_IWUSR, show_global_access_pixel, store_global_access_pixel);
static LGE_TOUCH_ATTR(lpwg_data, S_IRUGO | S_IWUSR, show_lpwg_data, store_lpwg_data);
static LGE_TOUCH_ATTR(lpwg_notify, S_IRUGO | S_IWUSR, NULL, store_lpwg_notify);

static struct attribute *lge_touch_attribute_list[] = {
	&lge_touch_attr_platform_data.attr,
	&lge_touch_attr_firmware.attr,
	&lge_touch_attr_testmode_ver.attr,
	&lge_touch_attr_version.attr,
	&lge_touch_attr_section.attr,
	&lge_touch_attr_reset.attr,
	&lge_touch_attr_ic_rw.attr,
	&lge_touch_attr_keyguard.attr,
	&lge_touch_attr_lockscreen.attr,
	&lge_touch_attr_touch_gesture.attr,
	&lge_touch_attr_virtualkeys.attr,
	&lge_touch_attr_jitter.attr,
	&lge_touch_attr_accuracy.attr,
	&lge_touch_attr_knock_on_type.attr,
	&lge_touch_attr_power_control.attr,
	&lge_touch_attr_sd.attr,
	&lge_touch_attr_sd_status.attr,
	&lge_touch_attr_rawdata.attr,
	&lge_touch_attr_report_mode.attr,
	&lge_touch_attr_ta_debouncing_count.attr,
	&lge_touch_attr_ta_debouncing_finger_num.attr,
	&lge_touch_attr_ghost_detection_enable.attr,
	&lge_touch_attr_delta.attr,
	&lge_touch_attr_chstatus.attr,
	&lge_touch_attr_global_access_pixel.attr,
	&lge_touch_attr_lpwg_data.attr,
	&lge_touch_attr_lpwg_notify.attr,
	NULL,
};

/* lge_touch_attr_show / lge_touch_attr_store
 *
 * sysfs bindings for lge_touch
 */
static ssize_t lge_touch_attr_show(struct kobject *lge_touch_kobj, struct attribute *attr,
			     char *buf)
{
	struct lge_touch_data *ts =
			container_of(lge_touch_kobj, struct lge_touch_data, lge_touch_kobj);
	struct lge_touch_attribute *lge_touch_priv =
		container_of(attr, struct lge_touch_attribute, attr);
	ssize_t ret = 0;

	if (lge_touch_priv->show)
		ret = lge_touch_priv->show(ts, buf);

	return ret;
}

static ssize_t lge_touch_attr_store(struct kobject *lge_touch_kobj, struct attribute *attr,
			      const char *buf, size_t count)
{
	struct lge_touch_data *ts =
		container_of(lge_touch_kobj, struct lge_touch_data, lge_touch_kobj);
	struct lge_touch_attribute *lge_touch_priv =
		container_of(attr, struct lge_touch_attribute, attr);
	ssize_t ret = 0;

	if (lge_touch_priv->store)
		ret = lge_touch_priv->store(ts, buf, count);

	return ret;
}

static const struct sysfs_ops lge_touch_sysfs_ops = {
	.show	= lge_touch_attr_show,
	.store	= lge_touch_attr_store,
};

static struct kobj_type lge_touch_kobj_type = {
	.sysfs_ops		= &lge_touch_sysfs_ops,
	.default_attrs 	= lge_touch_attribute_list,
};

static struct sysdev_class lge_touch_sys_class = {
	.name = LGE_TOUCH_NAME,
};

static struct sys_device lge_touch_sys_device = {
	.id		= 0,
	.cls	= &lge_touch_sys_class,
};

int synaptics_power_on(int on)
{
	int rc = 0;

	static struct regulator *vreg_l28;
	static struct regulator *vreg_lvs1;

	struct i2c_client *client = touch_test_dev->client;

		/* 3.3V_TOUCH_VDD, VREG_L28: 2.75 ~ 3.3 */
		if (!vreg_l28) {
			vreg_l28 = regulator_get(&client->dev, "vdd");
			if (IS_ERR(vreg_l28)) {
				pr_err("%s: regulator get of pm8226_l28 failed (%ld)\n",
						__func__,
					   PTR_ERR(vreg_l28));
				rc = PTR_ERR(vreg_l28);
				vreg_l28 = NULL;
				return rc;
			}
		}
		/* 1.8V_TOUCH_IO, VREG_LVS1: 1.7 ~ 2.85 */
		if (!vreg_lvs1) {
			vreg_lvs1 = regulator_get(&client->dev, "vdd_i2c");
			if (IS_ERR(vreg_lvs1)) {
				pr_err("%s: regulator get of pm8226_lvs1 failed (%ld)\n",
						__func__,
					   PTR_ERR(vreg_lvs1));
				rc = PTR_ERR(vreg_lvs1);
				vreg_lvs1 = NULL;
				return rc;
			}
		}

		rc = regulator_set_voltage(vreg_l28, 2850000, 2850000);

		if (rc < 0) {
			printk(KERN_INFO "[Touch D] %s: cannot control regulator:%d\n",
				   __func__, rc);
			return rc;
		}

		if (on) {
			printk("[Touch D]touch enable\n");
			regulator_enable(vreg_l28);
			regulator_enable(vreg_lvs1);
		} else {
			printk("[Touch D]touch disable\n");
			regulator_disable(vreg_l28);
			regulator_disable(vreg_lvs1);
		}

		return rc;

}

static int synaptics_parse_dt(struct device *dev, struct touch_platform_data *pdata)
{
	struct device_node *node = dev->of_node;
	struct device_node *pp;
	struct touch_device_caps *caps_info;
	struct touch_operation_role *role_info;
	struct touch_power_module *pwr_info;
	struct property *prop;
	int rc = 0;
	int len = 0;
	unsigned int i;
	const char *temp_string = node->name;
	const char *reg_string = node->name;
	u32 temp_val;
	u32 temp_array[16];

	/* reset, irq gpio info */
	if (node == NULL)
		return -ENODEV;

	pdata->reset_pin = of_get_named_gpio_flags(node, "synaptics,reset-gpio", 0, NULL);
	pdata->int_pin = of_get_named_gpio_flags(node, "synaptics,irq-gpio", 0, NULL);

	rc = of_property_read_string(node, "synaptics,maker", &temp_string);
	if (rc) {
		TOUCH_DEBUG_MSG("Looking up %s property in node %s failed",
			"synaptics,maker",
			temp_string);
		return -ENODEV;
	}
	len = strlen(temp_string);
	memcpy(pdata->maker, temp_string, len);
	rc = of_property_read_u32_array(node, "synaptics,fw_version_info", temp_array, FW_VER_INFO_NUM-1);
	if (rc) {
		TOUCH_DEBUG_MSG("Looking up %s property in node %02X %02X %02X failed",
			"synaptics,fw_version",
			temp_array[0], temp_array[1], temp_array[2]);
		return -ENODEV;
	} else {
		for (i = 0; i < FW_VER_INFO_NUM-1; i++) {
			pdata->fw_version[i] = temp_array[i];
		}
		TOUCH_DEBUG_MSG("fw_ver_info : 0x%02X 0x%02X 0x%02X \n",
			pdata->fw_version[0], pdata->fw_version[1], pdata->fw_version[2]);
	}

	rc = of_property_count_strings(node, "synaptics,fw_image");
	TOUCH_INFO_MSG("firmware path, rc = %d ", rc);
	if (rc > 1) {
		TOUCH_DEBUG_MSG("Get latter inbuilt firmware path ..., due to different type of panel");
		for (i = 0; i < rc; i++) {
			of_property_read_string_index(node, "synaptics,fw_image", i, &pdata->inbuilt_fw_name_id[i]);
			TOUCH_INFO_MSG("fw_image [%d]: %s", i, pdata->inbuilt_fw_name_id[i]);
		}
	} else {
		rc = of_property_read_string_index(node, "synaptics,fw_image", 0, &pdata->inbuilt_fw_name_id[0]);
		if (rc) {
			TOUCH_DEBUG_MSG("Looking up %s property in node %s failed",
				"synaptics,fw_image", pdata->inbuilt_fw_name);
			return -ENODEV;
		} else {
			pdata->inbuilt_fw_name = pdata->inbuilt_fw_name_id[0];
			TOUCH_DEBUG_MSG("fw_image: %s", pdata->inbuilt_fw_name);
		}
	}

	rc = of_property_read_u32(node, "lge,knock_on_type",  &temp_val);
	if (rc) {
		TOUCH_DEBUG_MSG("Unable to read knock_on_type - set as 0\n");
		pdata->knock_on_type = 0;
	} else {
		pdata->knock_on_type = temp_val;
	}
	TOUCH_DEBUG_MSG("knock_on_type: %d", pdata->knock_on_type);

	/* Caps */
	pdata->num_caps = 0;
	pp = NULL;
	while ((pp = of_get_next_child(node, pp)))
		pdata->num_caps++;

	if (!pdata->num_caps)
		return 0;

	caps_info = devm_kzalloc(dev, pdata->num_caps * sizeof(struct touch_device_caps), GFP_KERNEL);
	if (!caps_info)
		return -ENOMEM;

	rc = of_property_count_strings(node, "synaptics,panel_spec");
	TOUCH_INFO_MSG("panel_spec, rc = %d ", rc);
	if (rc > 1) {
		TOUCH_DEBUG_MSG("Get latter panel spec ..., due to different type of panel");
		for (i = 0; i < rc; i++) {
			of_property_read_string_index(node, "synaptics,panel_spec", i, &pdata->panel_spec_id[i]);
			TOUCH_INFO_MSG("panel_spec [%d]: %s", i, pdata->panel_spec_id[i]);
		}
	} else {
		rc = of_property_read_string_index(node, "synaptics,panel_spec", 0, &pdata->panel_spec_id[0]);
		if (rc) {
			TOUCH_DEBUG_MSG("Looking up %s property in node %s failed",
				"synaptics,panel_spec", pdata->panel_spec);
		} else {
			pdata->panel_spec = pdata->panel_spec_id[0];
			TOUCH_DEBUG_MSG("panel_spec: %s", pdata->panel_spec);
		}
	}

	/*global_access_pixel parsing*/
	rc = of_property_read_u32(node, "synaptics,global_access_pixel", &pdata->global_access_pixel);
	if (rc) {
		TOUCH_DEBUG_MSG("Looking up %s property in node %d failed",
			"synaptics,global_access_pixel", pdata->global_access_pixel);
	} else {
		TOUCH_DEBUG_MSG("global_access_pixel: %d", pdata->global_access_pixel);
	}

	pdata->caps = caps_info;
	for_each_child_of_node(node, pp) {
		rc = of_property_read_u32(pp, "panel_type", &temp_val);
		if (rc && (rc != -EINVAL)) {
			TOUCH_DEBUG_MSG("Unable to read panel_type\n");
			pdata->panel_type = 0;
		} else
			pdata->panel_type =  temp_val;

		rc = of_property_read_u32(pp, "button_support", &temp_val);
		if (rc && (rc != -EINVAL)) {
			TOUCH_DEBUG_MSG("Unable to read button_support\n");
			return rc;
		} else
			caps_info->button_support =  temp_val;

		rc = of_property_read_u32(pp, "y_button_boundary", &temp_val);
		if (rc && (rc != -EINVAL)) {
			TOUCH_DEBUG_MSG("Unable to read y_button_boundary\n");
			return rc;
		} else
			caps_info->y_button_boundary =	temp_val;

		rc = of_property_read_u32(pp, "button_margin", &temp_val);
		if (rc && (rc != -EINVAL)) {
			TOUCH_DEBUG_MSG("Unable to read button_margin\n");
			return rc;
		} else
			caps_info->button_margin =	temp_val;

		rc = of_property_read_u32(pp, "number_of_button", &temp_val);
		if (rc && (rc != -EINVAL)) {
			TOUCH_DEBUG_MSG("Unable to read number_of_button\n");
			return rc;
		} else
			caps_info->number_of_button =  temp_val;

		prop = of_find_property(pp, "button_name", NULL);
		if (prop && (prop->length == caps_info->number_of_button)) {
				const u8 *iprop = prop->value;
				for (i = 0; i < prop->length; i++)
					caps_info->button_name[i] = iprop[i];
		}

		rc = of_property_read_u32(pp, "is_width_supported", &temp_val);
		if (rc && (rc != -EINVAL)) {
			TOUCH_DEBUG_MSG("Unable to read is_width_supported\n");
			return rc;
		} else
			caps_info->is_width_supported =  temp_val;

		rc = of_property_read_u32(pp, "is_pressure_supported", &temp_val);
		if (rc && (rc != -EINVAL)) {
			TOUCH_DEBUG_MSG("Unable to read is_pressure_supported\n");
			return rc;
		} else
			caps_info->is_pressure_supported =	temp_val;

		rc = of_property_read_u32(pp, "is_id_supported", &temp_val);
		if (rc && (rc != -EINVAL)) {
			TOUCH_DEBUG_MSG("Unable to read is_id_supported\n");
			return rc;
		} else
			caps_info->is_id_supported =  temp_val;

		rc = of_property_read_u32(pp, "max_width", &temp_val);
		if (rc && (rc != -EINVAL)) {
			TOUCH_DEBUG_MSG("Unable to read max_width\n");
			return rc;
		} else
			caps_info->max_width =	temp_val;

		rc = of_property_read_u32(pp, "max_pressure", &temp_val);
		if (rc && (rc != -EINVAL)) {
			TOUCH_DEBUG_MSG("Unable to read max_pressure\n");
			return rc;
		} else
			caps_info->max_pressure = temp_val;

		rc = of_property_read_u32(pp, "max_id", &temp_val);
		if (rc && (rc != -EINVAL)) {
			TOUCH_DEBUG_MSG("Unable to read max_id\n");
			return rc;
		} else
			caps_info->max_id = temp_val;

		rc = of_property_read_u32(pp, "x_max", &temp_val);
		if (rc && (rc != -EINVAL)) {
			TOUCH_DEBUG_MSG("Unable to read x_max\n");
			return rc;
		} else
			caps_info->x_max = temp_val;

		rc = of_property_read_u32(pp, "y_max", &temp_val);
		if (rc && (rc != -EINVAL)) {
			TOUCH_DEBUG_MSG("Unable to read y_max\n");
			return rc;
		} else
			caps_info->y_max = temp_val;

		rc = of_property_read_u32(pp, "lcd_x", &temp_val);
		if (rc && (rc != -EINVAL)) {
			TOUCH_DEBUG_MSG("Unable to read lcd_x\n");
			return rc;
		} else
			caps_info->lcd_x = temp_val;

		rc = of_property_read_u32(pp, "lcd_y", &temp_val);
		if (rc && (rc != -EINVAL)) {
			TOUCH_DEBUG_MSG("Unable to read lcd_y\n");
			return rc;
		} else
			caps_info->lcd_y = temp_val;

		/*Calculation LCD : TOUCH Ratio*/
		caps_info->lcd_touch_ratio_x = caps_info->x_max / caps_info->lcd_x;
		if (caps_info->lcd_touch_ratio_x < 1)
			caps_info->lcd_touch_ratio_x = 1;
		caps_info->lcd_touch_ratio_y = caps_info->y_max / caps_info->lcd_y;
		if (caps_info->lcd_touch_ratio_y < 1)
			caps_info->lcd_touch_ratio_y = 1;

		TOUCH_INFO_MSG("Calculated LCD_TOUCH_RATIO x = %d , y = %d \n", caps_info->lcd_touch_ratio_x, caps_info->lcd_touch_ratio_y);

		rc = of_property_read_u32(pp, "maker_id", &temp_val);
		if (rc && (rc != -EINVAL)) {
			TOUCH_DEBUG_MSG("Unable to read maker_id\n");
			caps_info->maker_id = 0;
		} else
			caps_info->maker_id =  temp_val;

		rc = of_property_read_u32(pp, "maker_id_gpio", &temp_val);
		if (rc && (rc != -EINVAL)) {
			TOUCH_DEBUG_MSG("Unable to read maker_id_gpio\n");
			caps_info->maker_id_gpio =  0;
		} else
			caps_info->maker_id_gpio =  temp_val;

		rc = of_property_read_u32(pp, "maker_id2_gpio", &temp_val);
		if (rc && (rc != -EINVAL)) {
			TOUCH_DEBUG_MSG("Unable to read maker_id2_gpio\n");
			caps_info->maker_id2_gpio =  0;
		} else{
			caps_info->maker_id2_gpio =  temp_val;
		}

		prop = of_find_property(pp, "ghost_detection_value", NULL);
		if (prop) {
			temp_val = prop->length / sizeof(temp_val);

			if (temp_val <= GHOST_VALUE_MAX + 1) {
				rc = of_property_read_u32_array(pp, "ghost_detection_value", temp_array, temp_val);
				if (rc) {
					TOUCH_INFO_MSG("DT : Unable to read ghost_detection_value\n");
				}
				for (i = 0; i < temp_val; i++)
					caps_info->ghost_detection_value[i] = temp_array[i];
			}
		}

		caps_info++;
	}

	/* Role */
		pdata->num_role = 0;
		pp = NULL;
		while ((pp = of_get_next_child(node, pp)))
			pdata->num_role++;

		if (!pdata->num_role)
			return 0;

		role_info = devm_kzalloc(dev, pdata->num_role * sizeof(struct touch_operation_role), GFP_KERNEL);
		if (!role_info)
			return -ENOMEM;

		pdata->role = role_info;
		for_each_child_of_node(node, pp) {
			rc = of_property_read_u32(pp, "palm_detect_mode", &temp_val);
			if (rc && (rc != -EINVAL)) {
				TOUCH_DEBUG_MSG("Unable to read palm_detect_mode\n");
				role_info->palm_detect_mode =  0;
			} else
				role_info->palm_detect_mode =  temp_val;

			rc = of_property_read_u32(pp, "operation_mode", &temp_val);
			if (rc && (rc != -EINVAL)) {
				TOUCH_DEBUG_MSG("Unable to read operation_mode\n");
				return rc;
			} else
				role_info->operation_mode =  temp_val;

			rc = of_property_read_u32(pp, "key_type", &temp_val);
			if (rc && (rc != -EINVAL)) {
				TOUCH_DEBUG_MSG("Unable to read key_type\n");
				return rc;
			} else
				role_info->key_type =  temp_val;

			rc = of_property_read_u32(pp, "report_mode", &temp_val);
			if (rc && (rc != -EINVAL)) {
				TOUCH_DEBUG_MSG("Unable to read report_mode\n");
				return rc;
			} else
				role_info->report_mode =  temp_val;

			rc = of_property_read_u32(pp, "delta_pos_threshold", &temp_val);
			if (rc && (rc != -EINVAL)) {
				TOUCH_DEBUG_MSG("Unable to read delta_pos_threshold\n");
				return rc;
			} else
				role_info->delta_pos_threshold =  temp_val;

			rc = of_property_read_u32(pp, "orientation", &temp_val);
			if (rc && (rc != -EINVAL)) {
				TOUCH_DEBUG_MSG("Unable to read orientation\n");
				return rc;
			} else
				role_info->orientation =  temp_val;

			rc = of_property_read_u32(pp, "report_period", &temp_val);
			if (rc && (rc != -EINVAL)) {
				TOUCH_DEBUG_MSG("Unable to read report_period\n");
				return rc;
			} else
				role_info->report_period =	temp_val;

			rc = of_property_read_u32(pp, "booting_delay", &temp_val);
			if (rc && (rc != -EINVAL)) {
				TOUCH_DEBUG_MSG("Unable to read booting_delay\n");
				return rc;
			} else
				role_info->booting_delay =	temp_val;

			rc = of_property_read_u32(pp, "reset_delay", &temp_val);
			if (rc && (rc != -EINVAL)) {
				TOUCH_DEBUG_MSG("Unable to read reset_delay\n");
				return rc;
			} else
				role_info->reset_delay =  temp_val;

			rc = of_property_read_u32(pp, "suspend_pwr", &temp_val);
			if (rc && (rc != -EINVAL)) {
				TOUCH_DEBUG_MSG("Unable to read suspend_pwr\n");
				return rc;
			} else
				role_info->suspend_pwr =  temp_val;

			rc = of_property_read_u32(pp, "resume_pwr", &temp_val);
			if (rc && (rc != -EINVAL)) {
				TOUCH_DEBUG_MSG("Unable to read resume_pwr\n");
				return rc;
			} else
				role_info->resume_pwr =  temp_val;

			rc = of_property_read_u32(pp, "jitter_filter_enable", &temp_val);
			if (rc && (rc != -EINVAL)) {
				TOUCH_DEBUG_MSG("Unable to read jitter_filter_enable\n");
				return rc;
			} else
				role_info->jitter_filter_enable =  temp_val;

			rc = of_property_read_u32(pp, "jitter_curr_ratio", &temp_val);
			if (rc && (rc != -EINVAL)) {
				TOUCH_DEBUG_MSG("Unable to read jitter_curr_ratio\n");
				return rc;
			} else
				role_info->jitter_curr_ratio =	temp_val;

			rc = of_property_read_u32(pp, "accuracy_filter_enable", &temp_val);
			if (rc && (rc != -EINVAL)) {
				TOUCH_DEBUG_MSG("Unable to read accuracy_filter_enable\n");
				return rc;
			} else
				role_info->accuracy_filter_enable =  temp_val;

			rc = of_property_read_u32(pp, "ghost_finger_solution_enable", &temp_val);
			if (rc && (rc != -EINVAL)) {
				TOUCH_DEBUG_MSG("Unable to read ghost_finger_solution_enable\n");
				return rc;
			} else
				role_info->ghost_finger_solution_enable =  temp_val;
			rc = of_property_read_u32(pp, "ghost_detection_enable", &temp_val);
			if (rc && (rc != -EINVAL)) {
				TOUCH_DEBUG_MSG("Unable to read ghost_detection_enable\n");
				return rc;
			} else
				role_info->ghost_detection_enable =  temp_val;
			of_property_read_u32(pp, "irqflags", (u32 *)&role_info->irqflags);

			rc = of_property_read_u32(pp, "ghost_detection_button_enable", &temp_val);
			if (rc && (rc != -EINVAL)) {
				TOUCH_DEBUG_MSG("Unable to read ghost_detection_button_enable\n");
				return rc;
			} else
				role_info->ghost_detection_button_enable =  temp_val;
		}

		role_info++;

	/* pwr */
		pdata->num_pwr = 0;
		pp = NULL;
		while ((pp = of_get_next_child(node, pp)))
			pdata->num_pwr++;

		if (!pdata->num_pwr)
			return 0;

		pwr_info = devm_kzalloc(dev, pdata->num_pwr * sizeof(struct touch_power_module), GFP_KERNEL);
		if (!pwr_info)
			return -ENOMEM;

		pdata->pwr = pwr_info;

		for_each_child_of_node(node, pp) {

			rc = of_property_read_u32(pp, "use_regulator", &temp_val);
			if (rc) {
				TOUCH_DEBUG_MSG("Unable to read operation_mode\n");
				return rc;
			} else
				pwr_info->use_regulator =  temp_val;

			rc = of_property_read_u32(pp, "use_vio_regulator", &temp_val);
			if (rc) {
				TOUCH_DEBUG_MSG("Unable to read operation_mode\n");
				return rc;
			} else
				pwr_info->use_vio_regulator =  temp_val;

			rc = of_property_read_string(pp, "vdd", &reg_string);
			if (rc && (rc != -EINVAL)) {
				TOUCH_DEBUG_MSG("Looking up %s property in node %s failed",
					"vdd",
					reg_string);
				return -ENODEV;
			}
			len = strlen(reg_string);
			memcpy(pwr_info->vdd, reg_string, len);

			rc = of_property_read_u32(pp, "vdd_voltage", &temp_val);
			if (rc && (rc != -EINVAL)) {
				TOUCH_DEBUG_MSG("Unable to read vdd_voltage\n");
				return rc;
			} else
				pwr_info->vdd_voltage =  temp_val;

			rc = of_property_read_string(pp, "vio", &reg_string);
			if (rc && (rc != -EINVAL)) {
				TOUCH_DEBUG_MSG("Looking up %s property in node %s failed",
					"vio",
					reg_string);
				return -ENODEV;
			}
			len = strlen(reg_string);
			memcpy(pwr_info->vio, reg_string, len);

			rc = of_property_read_u32(pp, "vio_voltage", &temp_val);
			if (rc && (rc != -EINVAL)) {
				TOUCH_DEBUG_MSG("Unable to read vio_voltage\n");
				return rc;
			} else
				pwr_info->vio_voltage =  temp_val;


			rc = of_property_read_u32(pp, "gpio_vdd_en", &temp_val);
			if (rc && (rc != -EINVAL)) {
				TOUCH_DEBUG_MSG("Unable to read gpio_vdd_en\n");
				pwr_info->gpio_vdd_en = 0;
			} else
				pwr_info->gpio_vdd_en = temp_val;

			pwr_info->power = synaptics_power_on;

			pwr_info++;
		}

	return 0;

}

#if defined(CONFIG_FB)
static int touch_fb_suspend(struct device *device)
{
	struct lge_touch_data *ts = dev_get_drvdata(device);

	if (unlikely(touch_debug_mask & DEBUG_TRACE))
		TOUCH_DEBUG_MSG("\n");

	if (!ts_suspend) {
		if (ts->fw_info.fw_upgrade.is_downloading == UNDER_DOWNLOADING) {
			TOUCH_INFO_MSG("touch_suspend is not executed\n");
			return 0;
		}

		if (power_block) {
			TOUCH_INFO_MSG("touch_suspend is not executed\n");
			return 0;
		}

		touch_disable_irq(ts->client->irq);
		TOUCH_INFO_MSG("%s : disable_irq !!\n", __func__);

		if (ts->pdata->role->ghost_detection_enable) {
			hrtimer_cancel(&hr_touch_trigger_timer);
		}

		cancel_work_sync(&ts->work);
		flush_work(&ts->work);
		cancel_delayed_work_sync(&ts->work_init);

		if (ts->pdata->role->key_type == TOUCH_HARD_KEY)
			cancel_delayed_work_sync(&ts->work_touch_lock);

		release_all_ts_event(ts);

		mutex_lock(&ts->irq_work_mutex);

		if (touch_gesture_enable) {
			touch_double_tap_wakeup_enable(ts);
		} else {
			touch_power_cntl(ts, ts->pdata->role->suspend_pwr);
		}

		ts_suspend = 1;
		atomic_set(&ts->state.uevent_state, UEVENT_IDLE);
		touch_device_func->suspend(ts->client);

		mutex_unlock(&ts->irq_work_mutex);
	}
	return 0;
}

static int touch_fb_resume(struct device *device)
{
	struct lge_touch_data *ts =  dev_get_drvdata(device);
	int ret = 0;

	if (unlikely(touch_debug_mask & DEBUG_TRACE))
		TOUCH_DEBUG_MSG("\n");
	mutex_lock(&ts->irq_work_mutex);

	if (ts_suspend) {
		if (ts->fw_info.fw_upgrade.is_downloading == UNDER_DOWNLOADING) {
			TOUCH_INFO_MSG("touch_resume is not executed\n");
			ret = 1;
			goto failed_out;
		}

		if (power_block) {
			TOUCH_INFO_MSG("touch_resume is not executed\n");
			return 0;
		}

		ts_suspend = 0;

		if (touch_disable_irq_wake(ts) != 0) {
			TOUCH_INFO_MSG("disable_irq_wake failed\n");
		}

		touch_disable_irq(ts->client->irq);
		TOUCH_INFO_MSG("%s : disable_irq !!\n", __func__);

		if (ts->curr_pwr_state != POWER_OFF) {
			touch_power_cntl(ts, ts->pdata->role->suspend_pwr);
		}
		touch_power_cntl(ts, ts->pdata->role->resume_pwr);
		msleep(100);

		touch_enable_irq(ts->client->irq);
		TOUCH_INFO_MSG("%s : enable_irq !!\n", __func__);

		if (ts->pdata->role->resume_pwr == POWER_ON)
			queue_delayed_work(touch_wq, &ts->work_init,
					msecs_to_jiffies(50));
		else
			queue_delayed_work(touch_wq, &ts->work_init, 0);
	}
failed_out:
	mutex_unlock(&ts->irq_work_mutex);
	double_tap_enabled = 0;
	return ret;
}
#endif

#if defined(CONFIG_FB)
static int touch_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct lge_touch_data *ts = container_of(self, struct lge_touch_data, fb_notifier_block);

	if (evdata && evdata->data && event == FB_EVENT_BLANK && ts && ts->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK) {
			touch_fb_resume(&ts->client->dev);
		} else if (*blank == FB_BLANK_POWERDOWN) {
			touch_fb_suspend(&ts->client->dev);
		}
	}

	return 0;
}
#endif

static void touch_double_tap_wakeup_enable(struct lge_touch_data *ts)
{

	TOUCH_INFO_MSG("called touch_double_tap_wakeup_enable\n");

	double_tap_enabled = 1;
	if (touch_enable_irq_wake(ts) != 0) {

		TOUCH_INFO_MSG("enable_irq_wake failed\n");
	}

	touch_enable_irq(ts->client->irq);
	TOUCH_INFO_MSG("%s : enable_irq !!\n", __func__);

}

#ifdef CONFIG_LGE_PM_CHARGING_CHARGERLOGO
extern int lge_boot_mode_for_touch;
#endif

static int touch_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct lge_touch_data *ts;
	int ret = 0;
	int one_sec = 0;

	wake_lock_init(&touch_wake_lock, WAKE_LOCK_SUSPEND, "touch_irq");
	mutex_init(&i2c_suspend_lock);

	if (unlikely(touch_debug_mask & DEBUG_TRACE))
		TOUCH_DEBUG_MSG("\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		TOUCH_ERR_MSG("i2c functionality check error\n");
		ret = -EPERM;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(struct lge_touch_data), GFP_KERNEL);
	if (ts == NULL) {
		TOUCH_ERR_MSG("Can not allocate memory\n");
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	if (client->dev.of_node) {

		ts->pdata = devm_kzalloc(&client->dev,
			sizeof(struct touch_platform_data), GFP_KERNEL);
		if (!ts->pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			ret = -ENOMEM;
			goto err_alloc_data_failed;
		}

		ret = synaptics_parse_dt(&client->dev, ts->pdata);
		if (ret)
			goto err_alloc_platformdata_failed;

	} else {
		ts->pdata = client->dev.platform_data;
		if (!ts->pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			ret = -EINVAL;
			goto err_alloc_data_failed;
		}
	}

	ret = check_platform_data(ts->pdata);
	if (ret < 0) {
		TOUCH_ERR_MSG("Can not read platform data\n");
		ret = -EINVAL;
		goto err_assign_platform_data;
	}

	one_sec = 1000000 / (ts->pdata->role->report_period/1000);
	ts->ic_init_err_cnt = 0;
	ts->work_sync_err_cnt = 0;

	/* Ghost-Finger solution
	 * max_count : if melt-time > 7-sec, max_count should be changed to 'one_sec * 3'
	 */
	ts->gf_ctrl.min_count = one_sec / 12;
	ts->gf_ctrl.max_count = one_sec * 3;
	ts->gf_ctrl.saved_x = -1;
	ts->gf_ctrl.saved_y = -1;
	ts->gf_ctrl.saved_last_x = 0;
	ts->gf_ctrl.saved_last_y = 0;
	ts->gf_ctrl.max_moved = ts->pdata->caps->x_max / 4;
	ts->gf_ctrl.max_pressure = 255;

	get_section(&ts->st_info, ts->pdata);

	ts->client = client;
	i2c_set_clientdata(client, ts);

	/* Specific device probe */
	if (touch_device_func->probe) {
		ret = touch_device_func->probe(ts);/*(client); */
		if (ret < 0) {
			TOUCH_ERR_MSG("specific device probe fail\n");
			goto err_assign_platform_data;
		}
	}

	synaptics_probe = 1;

	/* reset pin setting */
	if (gpio_is_valid(ts->pdata->reset_pin)) {
		ret = gpio_request(ts->pdata->reset_pin, "touch_reset");
		if (ret < 0) {
			TOUCH_ERR_MSG("FAIL: touch_reset gpio_request\n");
			goto err_assign_platform_data;
		}
		gpio_direction_output(ts->pdata->reset_pin, 1);
	}


	/* vdd_en setting */
	if (gpio_is_valid(ts->pdata->pwr->gpio_vdd_en)) {
		ret = gpio_request(ts->pdata->pwr->gpio_vdd_en, "touch_vdd_en");
		if (ret < 0) {
			TOUCH_ERR_MSG("FAIL: gpio_vdd_en gpio_request\n");
			goto err_alloc_data_failed;
		}
		gpio_direction_output(ts->pdata->pwr->gpio_vdd_en, 0);
	}

	atomic_set(&ts->device_init, 0);

	/* Power on */
	if (touch_power_cntl(ts, POWER_ON) < 0)
		goto err_power_failed;

	msleep(ts->pdata->role->booting_delay);

	/* init work_queue */
	if (ts->pdata->role->key_type == TOUCH_HARD_KEY) {
		INIT_WORK(&ts->work, touch_work_func_a);
		INIT_DELAYED_WORK(&ts->work_touch_lock, touch_lock_func);
	} else if (ts->pdata->role->key_type == TOUCH_SOFT_KEY)
		INIT_WORK(&ts->work, touch_work_func_b);
	else
		INIT_WORK(&ts->work, touch_work_func_c);

	INIT_DELAYED_WORK(&ts->work_init, touch_init_func);
	INIT_WORK(&ts->work_fw_upgrade, touch_fw_upgrade_func);


	INIT_DELAYED_WORK(&ts->work_gesture_wakeup, touch_gesture_wakeup_func);

	/* input dev setting */
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		TOUCH_ERR_MSG("Failed to allocate input device\n");
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	}

	/* Auto Test interface */
	touch_test_dev = ts;

	ts->input_dev->name = "touch_dev";
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0))
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
#endif

	if (ts->pdata->caps->button_support && ts->pdata->role->key_type != VIRTUAL_KEY) {
		set_bit(EV_KEY, ts->input_dev->evbit);
		for (ret = 0; ret < ts->pdata->caps->number_of_button; ret++) {
			set_bit(ts->pdata->caps->button_name[ret], ts->input_dev->keybit);
		}
	}

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->pdata->caps->x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0,
			ts->pdata->caps->y_button_boundary
			? ts->pdata->caps->y_button_boundary
			: ts->pdata->caps->y_max,
			0, 0);
	if (ts->pdata->caps->is_pressure_supported)
		input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, ts->pdata->caps->max_pressure, 0, 0);
	if (ts->pdata->caps->is_width_supported) {
		input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, ts->pdata->caps->max_width, 0, 0);
		input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MINOR, 0, ts->pdata->caps->max_width, 0, 0);
		input_set_abs_params(ts->input_dev, ABS_MT_ORIENTATION, 0, 1, 0, 0);
	}

#if defined(MT_PROTOCOL_A)
	if (ts->pdata->caps->is_id_supported)
		input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, ts->pdata->caps->max_id, 0, 0);
#else
	input_mt_init_slots(ts->input_dev, ts->pdata->caps->max_id);
#endif

	/* interrupt mode */
	if (ts->pdata->role->operation_mode) {
		ret = gpio_request(ts->pdata->int_pin, "touch_int");
		if (ret < 0) {
			TOUCH_ERR_MSG("FAIL: touch_int gpio_request\n");
			goto err_interrupt_failed;
		}
		gpio_direction_input(ts->pdata->int_pin);

		ret = request_threaded_irq(client->irq, touch_irq_handler,
				touch_thread_irq_handler,
				ts->pdata->role->irqflags | IRQF_ONESHOT, client->name, ts);

		if (ret < 0) {
			TOUCH_ERR_MSG("request_irq failed. use polling mode\n");
			goto err_interrupt_failed;
		}
	}
	/* polling mode */
	else{
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = touch_timer_handler;
		hrtimer_start(&ts->timer, ktime_set(0, ts->pdata->role->report_period), HRTIMER_MODE_REL);
	}

	if (ts->pdata->role->ghost_detection_enable) {
	   hrtimer_init(&hr_touch_trigger_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	   hr_touch_trigger_timer.function = touch_trigger_timer_handler;
	}
	/* ghost-finger solution */
	ts->gf_ctrl.probe = 1;

	mutex_init(&ts->irq_work_mutex);

	/* Specific device initialization */
	touch_ic_init(ts);

	/* Firmware Upgrade Check*/
	if (touch_device_func->fw_upgrade) {
		queue_work(touch_wq, &ts->work_fw_upgrade);
	}

	ret = input_register_device(ts->input_dev);
	if (ret < 0) {
		TOUCH_ERR_MSG("Unable to register %s input device\n",
				ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	/* jitter solution */
	if (ts->pdata->role->jitter_filter_enable) {
		ts->jitter_filter.adjust_margin = 50;
	}

	/* accuracy solution */
	if (ts->pdata->role->accuracy_filter_enable) {
		ts->accuracy_filter.ignore_pressure_gap = 5;
		ts->accuracy_filter.delta_max = 30;
		ts->accuracy_filter.max_pressure = 255;
		ts->accuracy_filter.time_to_max_pressure = one_sec / 20;
		ts->accuracy_filter.direction_count = one_sec / 6;
		ts->accuracy_filter.touch_max_count = one_sec / 2;
	}

#if defined(CONFIG_FB)
	ts->fb_notifier_block.notifier_call = touch_notifier_callback;
	ret = fb_register_client(&ts->fb_notifier_block);
	if (ret) {
		TOUCH_ERR_MSG("unable to register fb_notifier callback: %d\n", ret);
		goto err_lge_touch_fb_register;
	}
#endif

#if defined(CONFIG_HAS_EARLYSUSPEND)
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = touch_early_suspend;
	ts->early_suspend.resume = touch_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	/* Register sysfs for making fixed communication path to framework layer */
	ret = sysdev_class_register(&lge_touch_sys_class);
	if (ret < 0) {
		TOUCH_ERR_MSG("sysdev_class_register is failed\n");
		goto err_lge_touch_sys_class_register;
	}

	ret = sysdev_register(&lge_touch_sys_device);
	if (ret < 0) {
		TOUCH_ERR_MSG("sysdev_register is failed\n");
		goto err_lge_touch_sys_dev_register;
	}

	ret = kobject_init_and_add(&ts->lge_touch_kobj, &lge_touch_kobj_type,
			ts->input_dev->dev.kobj.parent,
			"%s", LGE_TOUCH_NAME);
	if (ret < 0) {
		TOUCH_ERR_MSG("kobject_init_and_add is failed\n");
		goto err_lge_touch_sysfs_init_and_add;
	}

	if (likely(touch_debug_mask & DEBUG_BASE_INFO))
		TOUCH_INFO_MSG("Touch driver is initialized\n");

	return 0;

err_lge_touch_sysfs_init_and_add:
	kobject_del(&ts->lge_touch_kobj);
err_lge_touch_sys_dev_register:
	sysdev_unregister(&lge_touch_sys_device);
err_lge_touch_sys_class_register:
	sysdev_class_unregister(&lge_touch_sys_class);
	mutex_destroy(&ts->irq_work_mutex);
	mutex_destroy(&i2c_suspend_lock);
#if defined(CONFIG_FB)
err_lge_touch_fb_register:
	fb_unregister_client(&ts->fb_notifier_block);
#endif
#if defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts->early_suspend);
#endif
err_interrupt_failed:
err_input_register_device_failed:
	if (ts->pdata->role->operation_mode)
		free_irq(ts->client->irq, ts);
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
	touch_power_cntl(ts, POWER_OFF);
err_alloc_platformdata_failed:
	devm_kfree(&client->dev, ts->pdata);
err_power_failed:
err_assign_platform_data:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

static int touch_remove(struct i2c_client *client)
{
	struct lge_touch_data *ts = i2c_get_clientdata(client);

	if (unlikely(touch_debug_mask & DEBUG_TRACE))
		TOUCH_DEBUG_MSG("\n");

	/* Specific device remove */
	if (touch_device_func->remove)
		touch_device_func->remove(ts->client);

	/* Power off */
	touch_power_cntl(ts, POWER_OFF);

	kobject_del(&ts->lge_touch_kobj);
	sysdev_unregister(&lge_touch_sys_device);
	sysdev_class_unregister(&lge_touch_sys_class);
#if defined(CONFIG_FB)
	fb_unregister_client(&ts->fb_notifier_block);
#endif
#if defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ts->early_suspend);
#endif
	if (ts->pdata->role->operation_mode)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);

	if (ts->pdata->role->ghost_detection_enable) {
		hrtimer_cancel(&hr_touch_trigger_timer);
	}

	input_unregister_device(ts->input_dev);

	mutex_destroy(&ts->irq_work_mutex);
	mutex_destroy(&i2c_suspend_lock);

	wake_lock_destroy(&touch_wake_lock);

	kfree(ts);

	return 0;
}

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void touch_early_suspend(struct early_suspend *h)
{
	struct lge_touch_data *ts =
			container_of(h, struct lge_touch_data, early_suspend);

	if (unlikely(touch_debug_mask & DEBUG_TRACE))
		TOUCH_DEBUG_MSG("\n");

	if (ts->fw_info.fw_upgrade.is_downloading == UNDER_DOWNLOADING) {
		TOUCH_INFO_MSG("early_suspend is not executed\n");
		return;
	}

	if (ts->pdata->role->ghost_detection_enable) {
		resume_flag = 0;
	}
	if (ts->pdata->role->operation_mode)
		touch_disable_irq(ts->client->irq);
	else
		hrtimer_cancel(&ts->timer);

	if (ts->pdata->role->ghost_detection_enable) {
		hrtimer_cancel(&hr_touch_trigger_timer);
	}

	cancel_work_sync(&ts->work);
	cancel_delayed_work_sync(&ts->work_init);
	if (ts->pdata->role->key_type == TOUCH_HARD_KEY)
		cancel_delayed_work_sync(&ts->work_touch_lock);

	release_all_ts_event(ts);
	gpio_set_value(ts->pdata->reset_pin, 0);

	touch_power_cntl(ts, ts->pdata->role->suspend_pwr);

}

static void touch_late_resume(struct early_suspend *h)
{
	struct lge_touch_data *ts =
			container_of(h, struct lge_touch_data, early_suspend);

	if (unlikely(touch_debug_mask & DEBUG_TRACE))
		TOUCH_DEBUG_MSG("\n");

	if (ts->fw_info.fw_upgrade.is_downloading == UNDER_DOWNLOADING) {
		TOUCH_INFO_MSG("late_resume is not executed\n");
		return;
	}

	touch_power_cntl(ts, ts->pdata->role->resume_pwr);

		if (ts->pdata->role->ghost_detection_enable) {
			resume_flag = 1;
			ts_rebase_count = 0;
		}
	if (ts->pdata->role->operation_mode)
		touch_enable_irq(ts->client->irq);
	else
		hrtimer_start(&ts->timer, ktime_set(0, ts->pdata->role->report_period), HRTIMER_MODE_REL);

	if (ts->pdata->role->resume_pwr == POWER_ON)
		queue_delayed_work(touch_wq, &ts->work_init,
				msecs_to_jiffies(ts->pdata->role->booting_delay));
	else
		queue_delayed_work(touch_wq, &ts->work_init, 0);
}
#endif

#if defined(CONFIG_PM)
static int touch_suspend(struct device *device)
{
	mutex_lock(&i2c_suspend_lock);
		TOUCH_DEBUG_MSG("\n");
	return 0;
}

static int touch_resume(struct device *device)
{
	mutex_unlock(&i2c_suspend_lock);
		TOUCH_DEBUG_MSG("\n");
	return 0;
}
#endif

#if defined(CONFIG_PM)
static struct dev_pm_ops touch_pm_ops = {
	.suspend 	= touch_suspend,
	.resume 	= touch_resume,
};
#endif

static struct of_device_id touch_match_table[] = {
	{ .compatible = "synaptics,s220x", },
	{ },
};
static struct i2c_device_id touch_id_table[] = {
	{ LGE_TOUCH_NAME, 0 },
};

static struct i2c_driver lge_touch_driver = {
	.probe   = touch_probe,
	.remove	 = touch_remove,
	.id_table = touch_id_table,
	.driver	 = {
		.name   = "lge_touch_synaptics",
		.owner	= THIS_MODULE,
#if defined(CONFIG_PM)
		.pm		= &touch_pm_ops,
#endif
		.of_match_table = touch_match_table,
	},
};

int touch_driver_register(struct touch_device_driver *driver)
{
	int ret = 0;

#ifdef CONFIG_LGE_PM_CHARGING_CHARGERLOGO
	if (lge_boot_mode_for_touch == 2) {/* Chargerlogo mode */
		TOUCH_INFO_MSG("Chargerlogo mode. Skip probe \n");
		ret = -EMLINK;
		goto err_touch_driver_register;
	}
#endif

	if (unlikely(touch_debug_mask & DEBUG_TRACE))
		TOUCH_DEBUG_MSG("\n");

	if (touch_device_func != NULL) {
		TOUCH_ERR_MSG("CANNOT add new touch-driver\n");
		ret = -EMLINK;
		goto err_touch_driver_register;
	}

	touch_device_func = driver;

	touch_wq = create_singlethread_workqueue("touch_wq");
	if (!touch_wq) {
		TOUCH_ERR_MSG("CANNOT create new workqueue\n");
		ret = -ENOMEM;
		goto err_work_queue;
	}

	ret = i2c_add_driver(&lge_touch_driver);
	if (ret < 0) {
		TOUCH_ERR_MSG("FAIL: i2c_add_driver\n");
		goto err_i2c_add_driver;
	}

	return 0;

err_i2c_add_driver:
	destroy_workqueue(touch_wq);
err_work_queue:
err_touch_driver_register:
	return ret;
}

void touch_driver_unregister(void)
{

#ifdef CONFIG_LGE_PM_CHARGING_CHARGERLOGO
	if (lge_boot_mode_for_touch == 2) {/* Chargerlogo mode */
		TOUCH_INFO_MSG("Chargerlogo mode. Skip probe \n");
		return;
	}
#endif


	if (unlikely(touch_debug_mask & DEBUG_TRACE))
		TOUCH_DEBUG_MSG("\n");

	i2c_del_driver(&lge_touch_driver);
	touch_device_func = NULL;

	if (touch_wq)
		destroy_workqueue(touch_wq);
}

