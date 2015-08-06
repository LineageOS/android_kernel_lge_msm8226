/* Touch_synaptics.c
 *
 * Copyright (C) 2013 LGE.
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

#include <linux/err.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/types.h>
#include <linux/file.h>
#include <linux/uaccess.h>
#include <linux/syscalls.h>

#include <linux/input/lge_touch_core.h>
#include <linux/input/touch_synaptics.h>
#include <linux/regulator/machine.h>

#include <mach/board_lge.h>
#include "./DS4/RefCode.h"
#include "./DS4/RefCode_PDTScan.h"

/* RMI4 spec from 511-000405-01 Rev.D
 * Function	Purpose									See page
 * $01		RMI Device Control						45
 * $1A		0-D capacitive button sensors			61
 * $05		Image Reporting							68
 * $07		Image Reporting							75
 * $08		BIST									82
 * $09		BIST									87
 * $11		2-D TouchPad sensors					93
 * $19		0-D capacitive button sensors			141
 * $30		GPIO/LEDs								148
 * $31		LEDs									162
 * $34		Flash Memory Management					163
 * $36		Auxiliary ADC							174
 * $54		Test Reporting							176
 */
#define RMI_DEVICE_CONTROL				0x01
#define TOUCHPAD_SENSORS				0x11
#define CAPACITIVE_BUTTON_SENSORS		0x1A
#define GPIO_LEDS						0x30
#define LEDS							0x31
#define ANALOG_CONTROL					0x54
#define TIMER							0x32
#define FLASH_MEMORY_MANAGEMENT			0x34
#define AUXILIARY_ADC					0x36

/* Register Map & Register bit mask
 * - Please check "One time" this map before using this device driver
 */
/* RMI_DEVICE_CONTROL */
#define MANUFACTURER_ID_REG				(ts->common_fc.dsc.query_base)			/* Manufacturer ID */
#define FW_REVISION_REG					(ts->common_fc.dsc.query_base+3)		/* FW revision */
#define PRODUCT_ID_REG					(ts->common_fc.dsc.query_base+11)		/* Product ID */

#define DEVICE_COMMAND_REG				(ts->common_fc.dsc.command_base)

#define DEVICE_CONTROL_REG 				(ts->common_fc.dsc.control_base)		/* Device Control */
#define DEVICE_CONTROL_NORMAL_OP		0x00	/* sleep mode : go to doze mode after 500 ms */
#define DEVICE_CONTROL_SLEEP 			0x01	/* sleep mode : go to sleep */
#define DEVICE_CONTROL_SPECIFIC			0x02	/* sleep mode : go to doze mode after 5 sec */
#define DEVICE_CONTROL_NOSLEEP			0x04
#define DEVICE_CONTROL_CONFIGURED		0x80

#define INTERRUPT_ENABLE_REG			(ts->common_fc.dsc.control_base+1)		/* Interrupt Enable 0 */

#define DEVICE_STATUS_REG				(ts->common_fc.dsc.data_base)			/* Device Status */
#define DEVICE_FAILURE_MASK				0x03
#define DEVICE_CRC_ERROR_MASK			0x04
#define DEVICE_STATUS_FLASH_PROG		0x40
#define DEVICE_STATUS_UNCONFIGURED		0x80

#define INTERRUPT_STATUS_REG			(ts->common_fc.dsc.data_base+1)		/* Interrupt Status */
#define INTERRUPT_MASK_FLASH			0x01
#define INTERRUPT_MASK_ABS0				0x04
#define INTERRUPT_MASK_BUTTON			0x10

/* TOUCHPAD_SENSORS */
#define FINGER_COMMAND_REG				(ts->finger_fc.dsc.command_base)

#define FINGER_STATE_REG				(ts->finger_fc.dsc.data_base) 			/* Finger State */
#define FINGER_DATA_REG_START                   (ts->finger_fc.dsc.data_base+2)         /* Finger Data Register */
#define FINGER_STATE_MASK				0x03
#define REG_X_POSITION					0
#define REG_Y_POSITION					1
#define REG_YX_POSITION					2
#define REG_WY_WX						3
#define REG_Z							4
#define TWO_D_EXTEND_STATUS			(ts->finger_fc.dsc.data_base+27)

#define TWO_D_REPORTING_MODE			(ts->finger_fc.dsc.control_base+0)		/* 2D Reporting Mode */
#define REPORT_BEYOND_CLIP				0x80
#define REPORT_MODE_CONTINUOUS			0x00
#define REPORT_MODE_REDUCED				0x01
#define ABS_FILTER						0x08
#define PALM_DETECT_REG 				(ts->finger_fc.dsc.control_base+1)		/* Palm Detect */
#define DELTA_X_THRESH_REG 				(ts->finger_fc.dsc.control_base+2)		/* Delta-X Thresh */
#define DELTA_Y_THRESH_REG 				(ts->finger_fc.dsc.control_base+3)		/* Delta-Y Thresh */
#define SENSOR_MAX_X_POS				(ts->finger_fc.dsc.control_base+6)		/* SensorMaxXPos */
#define SENSOR_MAX_Y_POS				(ts->finger_fc.dsc.control_base+8)		/* SensorMaxYPos */
#define FINGER_QUERY_LEN 				9

/* CAPACITIVE_BUTTON_SENSORS */
#define BUTTON_COMMAND_REG				(ts->button_fc.dsc.command_base)

#define BUTTON_DATA_REG					(ts->button_fc.dsc.data_base)			/* Button Data */
#define MAX_NUM_OF_BUTTON				4

/* ANALOG_CONTROL */
#define ANALOG_COMMAND_REG				(ts->analog_fc.dsc.command_base)
#define FORCE_UPDATE					0x04

#define ANALOG_CONTROL_REG				(ts->analog_fc.dsc.control_base)
#define FORCE_FAST_RELAXATION			0x04

#define FAST_RELAXATION_RATE			(ts->analog_fc.dsc.control_base+16)

/* FLASH_MEMORY_MANAGEMENT */
#define FLASH_CONFIG_ID_REG				(ts->flash_fc.dsc.control_base)		/* Flash Control */
#define FLASH_CONTROL_REG				(ts->flash_fc.dsc.data_base+18)
#define FLASH_STATUS_MASK				0xF0

/* Page number */
#define COMMON_PAGE						(ts->common_fc.function_page)
#define FINGER_PAGE						(ts->finger_fc.function_page)
#define BUTTON_PAGE						(ts->button_fc.function_page)
#define ANALOG_PAGE						(ts->analog_fc.function_page)
#define FLASH_PAGE						(ts->flash_fc.function_page)
#define DEFAULT_PAGE					0x00
#define LPWG_CTRL_PAGE					0x04

/* Double-tap & Multi-tap(N-Func) */
#define LPWG_CONTROL_REG				(ts->finger_fc.dsc.control_base+44)
#define MULTITAP_COUNT_REG				0x47
#define MAX_INTERTAP_TIME_REG			0x49
#define INTERTAP_DISTANCE_REG			0x4B

/* Get user-finger-data from register.
 */
#define TS_SNTS_GET_X_POSITION(_high_reg, _low_reg) \
		(((u16)((_high_reg << 4) & 0x0FF0) | (u16)(_low_reg&0x0F)))
#define TS_SNTS_GET_Y_POSITION(_high_reg, _low_reg) \
		(((u16)((_high_reg << 4) & 0x0FF0) | (u16)((_low_reg >> 4) & 0x0F)))
#define TS_SNTS_GET_WIDTH_MAJOR(_width) \
		(((((_width & 0xF0) >> 4) - (_width & 0x0F)) > 0) ? ((_width & 0xF0) >> 4) : _width & 0x0F)
#define TS_SNTS_GET_WIDTH_MINOR(_width) \
		(((((_width & 0xF0) >> 4) - (_width & 0x0F)) > 0) ? _width & 0x0F : ((_width & 0xF0) >> 4))
#define TS_SNTS_GET_ORIENTATION(_width) \
		((((_width & 0xF0) >> 4) - (_width & 0x0F)) > 0) ? 0 : 1
#define TS_SNTS_GET_PRESSURE(_pressure) \
		_pressure

/* GET_BIT_MASK & GET_INDEX_FROM_MASK
 *
 * For easily checking the user input.
 * Usually, User use only one or two fingers.
 * However, we should always check all finger-status-register
 * because we can't know the total number of fingers.
 * These Macro will prevent it.
 */
#define GET_BIT_MASK(_finger_status_reg)	\
		(((_finger_status_reg[1] & 0x01) << 4) |	\
		((_finger_status_reg[0] & 0x40) >> 3) | ((_finger_status_reg[0] & 0x10) >> 2) | \
		((_finger_status_reg[0] & 0x04) >> 1) | (_finger_status_reg[0] & 0x01))

#define GET_INDEX_FROM_MASK(_index, _bit_mask, _max_finger)	\
		for (; !((_bit_mask >> _index) & 0x01) && _index < _max_finger; _index++) \
			; \
			if (_index <= _max_finger) \
				_bit_mask &= ~(_bit_mask & (1 << (_index)));
u8 pressure_zero;
extern u8 device_control_reg;
extern int ts_charger_plug;
extern int ts_charger_type;
extern int cur_hopping_idx;
extern int double_tap_enabled;
int cns_en;
u8 hopping;

/* wrapper function for i2c communication - except defalut page
 * if you have to select page for reading or writing, then using this wrapper function */

extern unsigned int system_rev;
static char *productcode_parse(unsigned char *product);
struct i2c_client *ds4_i2c_client;
static char power_state;

bool touch_irq_mask = 1;

void touch_enable_irq(unsigned int irq)
{
	if (!touch_irq_mask) {
		touch_irq_mask = 1;
		enable_irq(irq);
		printk("[lge_touch] enable touch irq(%d)\n", touch_irq_mask);
	}
}

void touch_disable_irq(unsigned int irq)
{
	if (touch_irq_mask) {
		touch_irq_mask = 0;
		disable_irq_nosync(irq);
	}
}

void write_time_log(void)
{
	int fd = 0;
	char *fname = "/mnt/sdcard/touch_self_test.txt";
	char time_string[64] = {0};
	struct timespec my_time;
	struct tm my_date;
	mm_segment_t old_fs = get_fs();

	my_time = __current_kernel_time();
	time_to_tm(my_time.tv_sec, sys_tz.tz_minuteswest * 60 * (-1), &my_date);
	snprintf(time_string, 64, "%02d-%02d %02d:%02d:%02d.%03lu ",
		my_date.tm_mon + 1, my_date.tm_mday,
		my_date.tm_hour, my_date.tm_min, my_date.tm_sec,
		(unsigned long) my_time.tv_nsec / 1000000);

	set_fs(KERNEL_DS);
	fd = sys_open(fname, O_WRONLY|O_CREAT|O_APPEND, 0666);

	if (fd >= 0) {
		sys_write(fd, time_string, strlen(time_string));
		sys_close(fd);
	}
	set_fs(old_fs);
}

#define MS_TO_NS(x)	(x * 1E6L)
struct workqueue_struct *touch_palm_wq;

static enum hrtimer_restart touch_palm_timer_handler(struct hrtimer *palm_timer)
{
	struct synaptics_ts_data *ts = container_of(palm_timer, struct synaptics_ts_data, palm_timer);

	queue_work(touch_palm_wq, &ts->palm_work);

	return HRTIMER_NORESTART;
}

static void touch_palm_work(struct work_struct *palm_work)
{
	struct synaptics_ts_data *ts = container_of(palm_work, struct synaptics_ts_data, palm_work);
	u8 buf = 0;
	u8 palm = 0;
	static bool detected;

	if (unlikely(touch_i2c_read(ts->client, TWO_D_EXTEND_STATUS, 1, &buf) < 0)) {
		   TOUCH_ERR_MSG("TWO_D_EXTEND_STATUS read fail\n");
		   detected = false;
		   return;
	}

	ts->lge_touch_ts->ts_data.palm = palm = buf & 0x2;

	if (palm) {
		if (!detected)
			TOUCH_INFO_MSG("Palm detected \n");
		detected = true;
		hrtimer_start(&ts->palm_timer, ktime_set(0, MS_TO_NS(200)), HRTIMER_MODE_REL);
	} else {
		if (detected)
			TOUCH_INFO_MSG("Palm released \n");
		detected = false;
	}
	ts->lge_touch_ts->ts_data.prev_palm = ts->lge_touch_ts->ts_data.palm;
}


struct workqueue_struct *touch_multi_tap_wq;

static enum hrtimer_restart touch_multi_tap_timer_handler(struct hrtimer *multi_tap_timer)
{
	struct synaptics_ts_data *ts = container_of(multi_tap_timer, struct synaptics_ts_data, multi_tap_timer);

	queue_work(touch_multi_tap_wq, &ts->multi_tap_work);

	return HRTIMER_NORESTART;
}

static u8 custom_gesture_status;
static u8 multi_tap_fail_try;
static void touch_multi_tap_work(struct work_struct *multi_tap_work)
{
	struct synaptics_ts_data *ts = container_of(multi_tap_work, struct synaptics_ts_data, multi_tap_work);
	u8 r_mem = 0;
	double_tap_enabled = 1;

	if (touch_i2c_read(ts->client, TWO_D_REPORTING_MODE, 1, &r_mem) < 0) {
		TOUCH_ERR_MSG("2D_REPORT_MODE_REG read fail!\n");
	} else {
		r_mem = (r_mem & 0xf8) | 0x4;
		if (touch_i2c_write(ts->client, TWO_D_REPORTING_MODE, 1, &r_mem) < 0) {
			TOUCH_ERR_MSG("2D_REPORT_MODE_REG write fail\n");
		}
	}

	/*It should be enabled to prevent firmware from counting multi-tap before framework reply*/
	if (unlikely(touch_i2c_read(ts->client, DEVICE_CONTROL_REG, 1, &r_mem) < 0)) {
		TOUCH_ERR_MSG("DEVICE_CONTROL_REG read fail\n");
	}
	r_mem = (r_mem & 0xFC) | DEVICE_CONTROL_SLEEP;
	if (unlikely(touch_i2c_write_byte(ts->client, DEVICE_CONTROL_REG, r_mem) < 0)) {
		TOUCH_ERR_MSG("DEVICE_CONTROL_REG write fail\n");
	}
	/* if (custom_gesture_status) */
		send_uevent_lpwg(ts->client, LPWG_MULTI_TAP);
}

int synaptics_ts_page_data_read(struct i2c_client *client, u8 page, u8 reg, int size, u8 *data)
{
	if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, page) < 0)) {
		TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
		return -EIO;
	}

	if (unlikely(touch_i2c_read(client, reg, size, data) < 0)) {
		TOUCH_ERR_MSG("[%dP:%d]register read fail\n", page, reg);
		return -EIO;
	}

	if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE) < 0)) {
		TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
		return -EIO;
	}

	return 0;
}

int synaptics_ts_page_data_write(struct i2c_client *client, u8 page, u8 reg, int size, u8 *data)
{
	if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, page) < 0)) {
		TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
		return -EIO;
	}

	if (unlikely(touch_i2c_write(client, reg, size, data) < 0)) {
		TOUCH_ERR_MSG("[%dP:%d]register read fail\n", page, reg);
		return -EIO;
	}

	if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE) < 0)) {
		TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
		return -EIO;
	}

	return 0;
}

int synaptics_ts_page_data_write_byte(struct i2c_client *client, u8 page, u8 reg, u8 data)
{
	if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, page) < 0)) {
		TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
		return -EIO;
	}

	if (unlikely(touch_i2c_write_byte(client, reg, data) < 0)) {
		TOUCH_ERR_MSG("[%dP:%d]register write fail\n", page, reg);
		return -EIO;
	}

	if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE) < 0)) {
		TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
		return -EIO;
	}

	return 0;
}

static u8 lpwg_data[MAX_POINT_SIZE_FOR_LPWG*4] = {0};
int synaptics_ts_get_data(struct i2c_client *client, struct touch_data *data)
{
	struct synaptics_ts_data *ts =
			(struct synaptics_ts_data *)get_touch_handle(client);

	u16 touch_finger_bit_mask = 0;
	u8 finger_index = 0;
	u8 index = 0;
	u8 buf = 0;
	u8 cnt = 0;
	u8 buf2 = 0;
	u16 alpha = 0;
	u8 cns = 0;
	u16 im = 0;
	u16 vm = 0;
	u16 aim = 0;
	u8 num_of_finger_status_regs = 0;
	u8 lpwg_status_reg = 0;
	u8 multitap_lpwg = 0;

	data->total_num = 0;
	pressure_zero = 0;

	if (unlikely(touch_debug_mask & DEBUG_TRACE))
		TOUCH_DEBUG_MSG("\n");

	if (unlikely(touch_i2c_read(client, DEVICE_STATUS_REG,
			sizeof(ts->ts_data.interrupt_status_reg),
			&ts->ts_data.device_status_reg) < 0)) {
		TOUCH_ERR_MSG("DEVICE_STATUS_REG read fail\n");
		goto err_synaptics_getdata;
	}

	/* ESD damage check */
	if ((ts->ts_data.device_status_reg & DEVICE_FAILURE_MASK) == DEVICE_FAILURE_MASK) {
		TOUCH_ERR_MSG("ESD damage occured. Reset Touch IC\n");
		goto err_synaptics_device_damage;
	}


	/* Internal reset check */
	if (((ts->ts_data.device_status_reg & DEVICE_STATUS_UNCONFIGURED) >> 7) == 1) {
		TOUCH_ERR_MSG("Touch IC resetted internally. Reconfigure register setting\n");
		goto err_synaptics_device_damage;
	}

	if (unlikely(touch_i2c_read(client, INTERRUPT_STATUS_REG,
			sizeof(ts->ts_data.interrupt_status_reg),
			&ts->ts_data.interrupt_status_reg) < 0)) {
		TOUCH_ERR_MSG("INTERRUPT_STATUS_REG read fail\n");
		goto err_synaptics_getdata;
	}

	if (unlikely(touch_debug_mask & DEBUG_GET_DATA))
		TOUCH_INFO_MSG("Interrupt_status : 0x%x\n", ts->ts_data.interrupt_status_reg);

	/* Because of ESD damage... */
	if (unlikely(ts->ts_data.interrupt_status_reg & INTERRUPT_MASK_FLASH)) {
		TOUCH_ERR_MSG("Impossible Interrupt\n");
		goto err_synaptics_device_damage;
	}

	if (ts->ts_data.interrupt_status_reg == 0x08 || ts->ts_data.interrupt_status_reg == 0x00) {
		TOUCH_ERR_MSG("Ignore interrupt. interrupt status reg = 0x%x\n", ts->ts_data.interrupt_status_reg);
		goto ignore_interrupt;
	}

	num_of_finger_status_regs = (ts->num_of_data_points + 3) / 4;

	/* Palm check */
	if (unlikely(touch_i2c_read(client,
		ts->finger_fc.dsc.data_base + num_of_finger_status_regs + (NUM_OF_EACH_FINGER_DATA_REG * ts->num_of_data_points)/*TWO_D_EXTEND_STATUS*/,
		1, &buf) < 0)) {
		TOUCH_ERR_MSG("TWO_D_EXTEND_STATUS read fail\n");
		goto err_synaptics_getdata;
	}
	data->palm = buf & 0x2;

	if ((data->palm || data->prev_palm) && ts->pdata->role->palm_detect_mode) {
		queue_work(touch_palm_wq, &ts->palm_work);
		goto ts_noise_check;
	}

       /*check doubletap status*/
	if (unlikely(touch_i2c_read(client,
			ts->finger_fc.dsc.data_base + num_of_finger_status_regs + (NUM_OF_EACH_FINGER_DATA_REG * ts->num_of_data_points) + 1 /*LPWG_STATUS_REG */,
			1, &lpwg_status_reg) < 0)) {
		TOUCH_ERR_MSG("LPWG_DEVICE_STATUS_REG read fail\n");
		goto err_synaptics_getdata;
	}

	if (double_tap_enabled && ((lpwg_status_reg&0x01) == 0)) {
		TOUCH_ERR_MSG("Ignore interrupt. double_tap_enalbed = %d, lpwg_status_reg = %d, ADDR LPWG_STATUS = 0x%x", double_tap_enabled, lpwg_status_reg, ts->finger_fc.dsc.data_base + num_of_finger_status_regs + (NUM_OF_EACH_FINGER_DATA_REG * ts->num_of_data_points) + 1);
		goto ignore_interrupt;
	}
#if 1

	/*
	 * in case report mode is REDUCED_REPORT_MODE,
	 * despite finger pressed, interrupt_status_reg value can be 0x10.
	 * so to reserve finger pressed status, remove if condition
	 */
#else
	/* Finger */
	if (likely(ts->ts_data.interrupt_status_reg & INTERRUPT_MASK_ABS0))
#endif
	{
		if (ts->double_tap_enable)  {
			if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, LPWG_CTRL_PAGE) < 0)) {
				TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
				return -EIO;
			}

			if (touch_i2c_read(client, INTERTAP_DISTANCE_REG, 1, &multitap_lpwg) < 0) {
				TOUCH_ERR_MSG("INTERTAP_DISTANCE_REG read fail \n");
			}
			TOUCH_INFO_MSG(" %s : Max InterTap Distance read = 0x%02x\n", __FUNCTION__, multitap_lpwg);

			if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE) < 0)) {
				TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
				return -EIO;
			}
			if (ts->lpwg_mode)
				send_uevent_lpwg(client, LPWG_DOUBLE_TAP);

			goto ignore_interrupt;
		} else if (ts->multi_tap_enable) {
			if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, LPWG_CTRL_PAGE) < 0)) {
				TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
				return -EIO;
			}

			if (unlikely(touch_i2c_read(client, 0x0, 1, lpwg_data) < 0)) {
				TOUCH_ERR_MSG("MultipleTap Custom Gesture Status read fail\n");
			}
			TOUCH_INFO_MSG("custom gesture status %d \n", lpwg_data[0]);
			custom_gesture_status = lpwg_data[0];

			if (unlikely(touch_i2c_read(client, 0x1, ts->multi_tap_count*4, lpwg_data) < 0)) {
				TOUCH_ERR_MSG("MultipleTap Custom Data read fail\n");
			}

#if 1
			if (custom_gesture_status) {
				TOUCH_INFO_MSG("lpwg data \n");
			}
#else
			if (custom_gesture_status) {
				int i;
				for (i = 0; i < ts->multi_tap_count; i++) {
/*					TOUCH_INFO_MSG("lpwg data %d: 0:0x%-4x 1:0x%-4x 2:0x%-4x 3:0x%-4x\n",
							i, lpwg_data[4*i], lpwg_data[4*i+1], lpwg_data[4*i+2], lpwg_data[4*i+3]);	multi-tap coordinates */
					TOUCH_INFO_MSG("lpwg data %d \n", i);
				}
			}
#endif

			/*Multi TAP Maximum InterTap Distance*/
			if (touch_i2c_read(client, INTERTAP_DISTANCE_REG, 1, &multitap_lpwg) < 0) {
				TOUCH_ERR_MSG("%d bytes read fail \n", 1);
			}
			TOUCH_INFO_MSG(" %s : Max InterTap Distance read = 0x%02x \n", __FUNCTION__, multitap_lpwg);

			if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE) < 0)) {
				TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
				return -EIO;
			}

			{
				unsigned char r_mem = 0;
				double_tap_enabled = 0;
				if (touch_i2c_read(client, TWO_D_REPORTING_MODE, 1, &r_mem) < 0) {
					TOUCH_ERR_MSG("2D_REPORT_MODE_REG read fail!\n");
				} else {
					r_mem = r_mem & 0xf8;
					if (touch_i2c_write(client, TWO_D_REPORTING_MODE, 1, &r_mem) < 0) {
						TOUCH_ERR_MSG("2D_REPORT_MODE_REG write fail\n");
						return -EIO;
					}
				}
				hrtimer_try_to_cancel(&ts->multi_tap_timer);
				if (!hrtimer_callback_running(&ts->multi_tap_timer))
					hrtimer_start(&ts->multi_tap_timer, ktime_set(0, MS_TO_NS(200)), HRTIMER_MODE_REL);
			}
			return 0;
		}

		num_of_finger_status_regs = (ts->num_of_data_points + 3) / 4;
		if (unlikely(touch_i2c_read(client, FINGER_STATE_REG,
				num_of_finger_status_regs,
				ts->ts_data.finger.finger_status_reg) < 0)) {
			TOUCH_ERR_MSG("FINGER_STATE_REG read fail\n");
			goto err_synaptics_getdata;
		}

		touch_finger_bit_mask = GET_BIT_MASK(ts->ts_data.finger.finger_status_reg);
		if (unlikely(touch_debug_mask & DEBUG_GET_DATA)) {
			TOUCH_INFO_MSG("Finger_status : 0x%x, 0x%x, 0x%x\n", ts->ts_data.finger.finger_status_reg[0],
					ts->ts_data.finger.finger_status_reg[1], ts->ts_data.finger.finger_status_reg[2]);
			TOUCH_INFO_MSG("Touch_bit_mask: 0x%x\n", touch_finger_bit_mask);
		}

		while (touch_finger_bit_mask) {
			GET_INDEX_FROM_MASK(finger_index, touch_finger_bit_mask, MAX_NUM_OF_FINGERS)
			if (unlikely(touch_i2c_read(ts->client,
					ts->finger_fc.dsc.data_base + num_of_finger_status_regs + (NUM_OF_EACH_FINGER_DATA_REG * finger_index),
					NUM_OF_EACH_FINGER_DATA_REG,
					ts->ts_data.finger.finger_reg[finger_index]) < 0)) {
				TOUCH_ERR_MSG("FINGER_DATA_REG read fail\n");
				goto err_synaptics_getdata;
			}

			data->curr_data[finger_index].id = finger_index;
			data->curr_data[finger_index].x_position =
				TS_SNTS_GET_X_POSITION(ts->ts_data.finger.finger_reg[finger_index][REG_X_POSITION],
									   ts->ts_data.finger.finger_reg[finger_index][REG_YX_POSITION]);
			data->curr_data[finger_index].y_position =
				TS_SNTS_GET_Y_POSITION(ts->ts_data.finger.finger_reg[finger_index][REG_Y_POSITION],
									   ts->ts_data.finger.finger_reg[finger_index][REG_YX_POSITION]);
			data->curr_data[finger_index].width_major = TS_SNTS_GET_WIDTH_MAJOR(ts->ts_data.finger.finger_reg[finger_index][REG_WY_WX]);
			data->curr_data[finger_index].width_minor = TS_SNTS_GET_WIDTH_MINOR(ts->ts_data.finger.finger_reg[finger_index][REG_WY_WX]);
			data->curr_data[finger_index].width_orientation = TS_SNTS_GET_ORIENTATION(ts->ts_data.finger.finger_reg[finger_index][REG_WY_WX]);
			data->curr_data[finger_index].pressure = TS_SNTS_GET_PRESSURE(ts->ts_data.finger.finger_reg[finger_index][REG_Z]);
			data->curr_data[finger_index].status = FINGER_PRESSED;

			if (ts->pdata->role->ghost_detection_enable) {
				if (data->curr_data[finger_index].pressure == 0)
					pressure_zero = 1;
			}
			if (unlikely(touch_debug_mask & DEBUG_GET_DATA))
				TOUCH_INFO_MSG("<%d> pos(%4d,%4d) w_m[%2d] w_n[%2d] w_o[%2d] p[%2d]\n",
								finger_index, data->curr_data[finger_index].x_position, data->curr_data[finger_index].y_position,
								data->curr_data[finger_index].width_major, data->curr_data[finger_index].width_minor,
								data->curr_data[finger_index].width_orientation, data->curr_data[finger_index].pressure);
			index++;
		}
		data->total_num = index;
		if (unlikely(touch_debug_mask & DEBUG_GET_DATA))
			TOUCH_INFO_MSG("Total_num: %d\n", data->total_num);
	}

	 /* Button */
	if (unlikely(ts->button_fc.dsc.id != 0)) {
		if (likely(ts->ts_data.interrupt_status_reg & INTERRUPT_MASK_BUTTON)) {
			if (unlikely(synaptics_ts_page_data_read(client, BUTTON_PAGE, BUTTON_DATA_REG,
					sizeof(ts->ts_data.button_data_reg), &ts->ts_data.button_data_reg) < 0)) {
				TOUCH_ERR_MSG("BUTTON_DATA_REG read fail\n");
				goto err_synaptics_getdata;
			}

			if (unlikely(touch_debug_mask & DEBUG_BUTTON))
				TOUCH_DEBUG_MSG("Button register: 0x%x\n", ts->ts_data.button_data_reg);

			if (ts->ts_data.button_data_reg) {
				/* pressed - find first one */
				for (cnt = 0; cnt < ts->pdata->caps->number_of_button; cnt++) {
					if ((ts->ts_data.button_data_reg >> cnt) & 0x1) {
						ts->ts_data.button.key_code = ts->pdata->caps->button_name[cnt];
						data->curr_button.key_code = ts->ts_data.button.key_code;
						data->curr_button.state = 1;
						break;
					}
				}
			} else {
				/* release */
				data->curr_button.key_code = ts->ts_data.button.key_code;
				data->curr_button.state = 0;
			}
		}
	}

ts_noise_check:
	if ((ts_charger_plug == 1 && (data->prev_total_num != data->total_num)) ||
		(touch_debug_mask & DEBUG_NOISE)) {
		if (unlikely(synaptics_ts_page_data_read(client, ANALOG_PAGE, 0x0e, 1, &buf) < 0)) {
			TOUCH_ERR_MSG("Alpha REG read fail\n");
			goto err_synaptics_getdata;
		}
		if (unlikely(synaptics_ts_page_data_read(client, ANALOG_PAGE, 0x0f, 1, &buf2) < 0)) {
			TOUCH_ERR_MSG("Alpha REG read fail\n");
			goto err_synaptics_getdata;
		}
		alpha = (buf2<<8)|buf;

		if (unlikely(synaptics_ts_page_data_read(client, ANALOG_PAGE, 0x0D, 1, &cns) < 0)) {
			TOUCH_ERR_MSG("Current Noise State REG read fail\n");
			goto err_synaptics_getdata;
		}

		if (unlikely(synaptics_ts_page_data_read(client, ANALOG_PAGE, 0x05, 1, &buf) < 0)) {
			TOUCH_ERR_MSG("Interference Metric REG read fail\n");
			goto err_synaptics_getdata;
		}
		if (unlikely(synaptics_ts_page_data_read(client, ANALOG_PAGE, 0x06, 1, &buf2) < 0)) {
			TOUCH_ERR_MSG("Interference Metric REG read fail\n");
			goto err_synaptics_getdata;
		}
		im = (buf2<<8)|buf;

		if (unlikely(synaptics_ts_page_data_read(client, ANALOG_PAGE, 0x09, 1, &buf) < 0)) {
			TOUCH_ERR_MSG("Variance Metric REG read fail\n");
			goto err_synaptics_getdata;
		}
		if (unlikely(synaptics_ts_page_data_read(client, ANALOG_PAGE, 0x0a, 1, &buf2) < 0)) {
			TOUCH_ERR_MSG("Variance Metric REG read fail\n");
			goto err_synaptics_getdata;
		}
		vm = (buf2<<8)|buf;

		if (unlikely(synaptics_ts_page_data_read(client, ANALOG_PAGE, 0x0b, 1, &buf) < 0)) {
			TOUCH_ERR_MSG("Averaged IM REG read fail\n");
			goto err_synaptics_getdata;
		}
		if (unlikely(synaptics_ts_page_data_read(client, ANALOG_PAGE, 0x0c, 1, &buf2) < 0)) {
			TOUCH_ERR_MSG("Averaged IM REG read fail\n");
			goto err_synaptics_getdata;
		}
		aim = (buf2<<8)|buf;

		TOUCH_INFO_MSG("  A[%5d]   CNS[%d]   IM[%5d]   VM[%5d]   AIM[%5d]\n", alpha, cns, im, vm, aim);
	}

	return 0;

err_synaptics_device_damage:
err_synaptics_getdata:
	return -EIO;
ignore_interrupt:
	return -IGNORE_INTERRUPT;
}

static int read_page_description_table(struct i2c_client *client)
{
	struct synaptics_ts_data *ts =
			(struct synaptics_ts_data *)get_touch_handle(client);
	struct function_descriptor buffer;
	unsigned short u_address = 0;
	unsigned short page_num = 0;
	unsigned char query[FINGER_QUERY_LEN] = {0,};

	if (touch_debug_mask & DEBUG_TRACE)
		TOUCH_DEBUG_MSG("\n");

	memset(&buffer, 0x0, sizeof(struct function_descriptor));
	memset(&ts->common_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->finger_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->button_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->analog_fc, 0x0, sizeof(struct ts_ic_function));
	memset(&ts->flash_fc, 0x0, sizeof(struct ts_ic_function));

	for (page_num = 0; page_num < PAGE_MAX_NUM; page_num++) {
		if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, page_num) < 0)) {
			TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
			return -EIO;
		}

		for (u_address = DESCRIPTION_TABLE_START; u_address > 10; u_address -= sizeof(struct function_descriptor)) {
			if (unlikely(touch_i2c_read(client, u_address, sizeof(buffer), (unsigned char *)&buffer) < 0)) {
				TOUCH_ERR_MSG("RMI4 Function Descriptor read fail\n");
				return -EIO;
			}

			if (buffer.id == 0)
				break;

			switch (buffer.id) {
			case RMI_DEVICE_CONTROL:
				ts->common_fc.dsc = buffer;
				ts->common_fc.function_page = page_num;
				break;
			case TOUCHPAD_SENSORS:
				ts->finger_fc.dsc = buffer;
				ts->finger_fc.function_page = page_num;
				break;
			case CAPACITIVE_BUTTON_SENSORS:
				ts->button_fc.dsc = buffer;
				ts->button_fc.function_page = page_num;
				break;
			case ANALOG_CONTROL:
				ts->analog_fc.dsc = buffer;
				ts->analog_fc.function_page = page_num;
				break;
			case FLASH_MEMORY_MANAGEMENT:
				ts->flash_fc.dsc = buffer;
				ts->flash_fc.function_page = page_num;
			default:
				break;
			}
		}
	}

	if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE) < 0)) {
		TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
		return -EIO;
	}

	if (ts->common_fc.dsc.id == 0 || ts->finger_fc.dsc.id == 0
			|| ts->analog_fc.dsc.id == 0 || ts->flash_fc.dsc.id == 0) {

		if (touch_debug_mask & DEBUG_BASE_INFO)
			TOUCH_INFO_MSG("common[%dP:0x%02x] finger[%dP:0x%02x] button[%dP:0x%02x] analog[%dP:0x%02x] flash[%dP:0x%02x]\n",
					ts->common_fc.function_page, ts->common_fc.dsc.id,
					ts->finger_fc.function_page, ts->finger_fc.dsc.id,
					ts->button_fc.function_page, ts->button_fc.dsc.id,
					ts->analog_fc.function_page, ts->analog_fc.dsc.id,
					ts->flash_fc.function_page, ts->flash_fc.dsc.id);

		TOUCH_ERR_MSG("common/finger/analog/flash are not initiailized\n");
		return -EPERM;
	}

	if (unlikely(touch_i2c_read(client, ts->finger_fc.dsc.query_base, sizeof(query), query) < 0)) {
		TOUCH_ERR_MSG("Number of data point read fail\n");
		return -EIO;
	}
	if ((query[1] & 0x07) <= 4)
		ts->num_of_data_points = (query[1] & 0x07) + 1;
	else if ((query[1] & 0x07) == 5)
		ts->num_of_data_points = 10;

	if (touch_debug_mask & DEBUG_BASE_INFO)
		TOUCH_INFO_MSG("common[%dP:0x%02x] finger[%dP:0x%02x] button[%dP:0x%02x] analog[%dP:0x%02x] flash[%dP:0x%02x]\n",
				ts->common_fc.function_page, ts->common_fc.dsc.id,
				ts->finger_fc.function_page, ts->finger_fc.dsc.id,
				ts->button_fc.function_page, ts->button_fc.dsc.id,
				ts->analog_fc.function_page, ts->analog_fc.dsc.id,
				ts->flash_fc.function_page, ts->flash_fc.dsc.id);

	return 0;
}

static int synaptics_get_panel_id(struct synaptics_ts_data *ts)
{
	int panel_id = 0xFF;
	int ret = 0;
	int value = 0;
	static bool synaptics_init;

	if (synaptics_init && ts->pdata->panel_id) {
		return ts->pdata->panel_id;
	}

	if (!synaptics_init) {
		if (ts->pdata->caps->maker_id_gpio && gpio_is_valid(ts->pdata->caps->maker_id_gpio)) {
			ret = gpio_request(ts->pdata->caps->maker_id_gpio, "touch_id");
			if (ret < 0) {
				TOUCH_ERR_MSG("FAIL: touch_id gpio_request = %d\n", ret);
				goto Exit;
			} else {
				gpio_direction_input(ts->pdata->caps->maker_id_gpio);
			}
		} else {
			TOUCH_INFO_MSG("maker_id_gpio is invalid\n");
		}

		if (ts->pdata->caps->maker_id2_gpio && gpio_is_valid(ts->pdata->caps->maker_id2_gpio)) {
			ret = gpio_request(ts->pdata->caps->maker_id2_gpio, "touch_id2");
			if (ret < 0) {
				TOUCH_ERR_MSG("FAIL: touch_id2 gpio_request = %d\n", ret);
				goto Exit;
			} else {
				gpio_direction_input(ts->pdata->caps->maker_id2_gpio);
			}
		} else {
			TOUCH_INFO_MSG("maker_id2_gpio is invalid\n");
		}

		synaptics_init = true;
	}

	if (ts->pdata->caps->maker_id_gpio && gpio_is_valid(ts->pdata->caps->maker_id_gpio)) {
		value = gpio_get_value(ts->pdata->caps->maker_id_gpio);
		TOUCH_INFO_MSG("MAKER_ID : %s\n", value ? "High" : "Low");
		panel_id = (value & 0x1);
	}

	if (ts->pdata->caps->maker_id2_gpio && gpio_is_valid(ts->pdata->caps->maker_id2_gpio)) {
		value = gpio_get_value(ts->pdata->caps->maker_id2_gpio);
		TOUCH_INFO_MSG("MAKER_ID_2 : %s\n", value ? "High" : "Low");
		panel_id += ((value & 0x1) << 1);
	}

	synaptics_init = true;

	ts->pdata->panel_id = panel_id;
	TOUCH_INFO_MSG("Touch panel id : %d", ts->pdata->panel_id);

	return panel_id;

Exit:

	TOUCH_ERR_MSG("%s FAIL \n", __func__);

	return 0xFF;

}

static int synaptics_get_inbuilt_fw_path(struct synaptics_ts_data *ts, int panel_id)
{
	if (ts->pdata->inbuilt_fw_name) {
		TOUCH_DEBUG_MSG("fw_image: %s\n", ts->pdata->inbuilt_fw_name);
		return 0;
	}

	if (ts->pdata->inbuilt_fw_name_id[panel_id] == NULL)
		ts->pdata->inbuilt_fw_name = ts->pdata->inbuilt_fw_name_id[0];
	else
		ts->pdata->inbuilt_fw_name = ts->pdata->inbuilt_fw_name_id[panel_id];

	TOUCH_DEBUG_MSG("fw_image: %s\n", ts->pdata->inbuilt_fw_name);

	return 1;
}

static int synaptics_get_panel_spec(struct synaptics_ts_data *ts, int panel_id)
{
	if (ts->pdata->panel_spec) {
		TOUCH_DEBUG_MSG("panel_spec: %s\n", ts->pdata->panel_spec);
		return 0;
	}

	if (ts->pdata->panel_spec_id[panel_id] == NULL)
		ts->pdata->panel_spec = ts->pdata->panel_spec_id[0];
	else
		ts->pdata->panel_spec = ts->pdata->panel_spec_id[panel_id];

	TOUCH_DEBUG_MSG("panel_spec: %s\n", ts->pdata->panel_spec);

	return 1;
}

int get_ic_info(struct synaptics_ts_data *ts, struct touch_fw_info *fw_info)
{
#if defined(ARRAYED_TOUCH_FW_BIN)
	int cnt;
#endif

	u8 device_status = 0;
	u8 flash_control = 0;
	int panel_id = 0;

	if (unlikely(read_page_description_table(ts->client) < 0)) {
		TOUCH_ERR_MSG("read page description table fail\n");
		return -EIO;
	}

	device_control_reg = ts->common_fc.dsc.control_base;

	memset(&ts->fw_info, 0, sizeof(struct synaptics_ts_fw_info));

	if (unlikely(touch_i2c_read(ts->client, FW_REVISION_REG,
			sizeof(ts->fw_info.fw_rev), &ts->fw_info.fw_rev) < 0)) {
		TOUCH_ERR_MSG("FW_REVISION_REG read fail\n");
		return -EIO;
	}

	if (unlikely(touch_i2c_read(ts->client, MANUFACTURER_ID_REG,
			sizeof(ts->fw_info.manufacturer_id), &ts->fw_info.manufacturer_id) < 0)) {
		TOUCH_ERR_MSG("MANUFACTURER_ID_REG read fail\n");
		return -EIO;
	}

	if (unlikely(touch_i2c_read(ts->client, PRODUCT_ID_REG,
			sizeof(ts->fw_info.product_id) - 1, ts->fw_info.product_id) < 0)) {
		TOUCH_ERR_MSG("PRODUCT_ID_REG read fail\n");
		return -EIO;
	}

	if (unlikely(touch_i2c_read(ts->client, FLASH_CONFIG_ID_REG,
			sizeof(ts->fw_info.config_id) - 1, ts->fw_info.config_id) < 0)) {
		TOUCH_ERR_MSG("FLASH_CONFIG_ID_REG read fail\n");
		return -EIO;
	}

	snprintf(fw_info->ic_fw_identifier, sizeof(fw_info->ic_fw_identifier),
			"%s - %d", ts->fw_info.product_id, ts->fw_info.manufacturer_id);
	snprintf(fw_info->ic_fw_version, sizeof(fw_info->ic_fw_version),
			"%s", ts->fw_info.config_id);

	if (ts->pdata->caps->maker_id) {
		panel_id = synaptics_get_panel_id(ts);
		if (panel_id == 0xFF) {
			TOUCH_INFO_MSG("fail get panel id\n");
			ts->pdata->inbuilt_fw_name = ts->pdata->inbuilt_fw_name_id[0];
			ts->pdata->panel_spec = ts->pdata->panel_spec_id[0];
			TOUCH_DEBUG_MSG("fw_image: %s\n", ts->pdata->inbuilt_fw_name);
			TOUCH_DEBUG_MSG("panel_spec: %s\n", ts->pdata->panel_spec);
		} else {
			TOUCH_INFO_MSG("success get panel id\n");
			synaptics_get_inbuilt_fw_path(ts, panel_id);
			synaptics_get_panel_spec(ts, panel_id);
		}
	}

	if (unlikely(touch_i2c_read(ts->client, FLASH_CONTROL_REG, sizeof(flash_control), &flash_control) < 0)) {
		TOUCH_ERR_MSG("FLASH_CONTROL_REG read fail\n");
		return -EIO;
	}

	if (unlikely(touch_i2c_read(ts->client, DEVICE_STATUS_REG, sizeof(device_status), &device_status) < 0)) {
		TOUCH_ERR_MSG("DEVICE_STATUS_REG read fail\n");
		return -EIO;
	}

	/* Firmware has a problem, so we should firmware-upgrade */
	if (device_status & DEVICE_STATUS_FLASH_PROG
			|| (device_status & DEVICE_CRC_ERROR_MASK) != 0
			|| (flash_control & FLASH_STATUS_MASK) != 0) {
		TOUCH_ERR_MSG("Firmware has a unknown-problem, so it needs firmware-upgrade\n");
		TOUCH_ERR_MSG("FLASH_CONTROL[%x] DEVICE_STATUS_REG[%x]\n", (u32)flash_control, (u32)device_status);
		TOUCH_ERR_MSG("FW-upgrade Force Rework.\n");
		/* firmware version info change by force for rework */
		ts->fw_info.fw_rev = 0;
		fw_info->fw_upgrade.fw_force_upgrade = true;
		snprintf(ts->fw_info.config_id, sizeof(ts->fw_info.config_id), "ERR");
	}

	return 0;
}

static int lpwg_tap_control(struct synaptics_ts_data *ts, int on);
int synaptics_ts_init(struct i2c_client *client, struct touch_fw_info *fw_info)
{
	struct synaptics_ts_data *ts =
			(struct synaptics_ts_data *)get_touch_handle(client);

	u8 buf = 0;

	if (touch_debug_mask & DEBUG_TRACE)
		TOUCH_DEBUG_MSG("\n");

	if (!ts->is_probed && fw_info != NULL)
		if (unlikely(get_ic_info(ts, fw_info) < 0))
			return -EIO;

	if (unlikely(touch_i2c_write_byte(client, DEVICE_CONTROL_REG,
			DEVICE_CONTROL_NOSLEEP | DEVICE_CONTROL_CONFIGURED) < 0)) {
		TOUCH_ERR_MSG("DEVICE_CONTROL_REG write fail\n");
		return -EIO;
	}

	if (unlikely(touch_i2c_read(client, INTERRUPT_ENABLE_REG,
			1, &buf) < 0)) {
		TOUCH_ERR_MSG("INTERRUPT_ENABLE_REG read fail\n");
		return -EIO;
	}
	if (unlikely(touch_i2c_write_byte(client, INTERRUPT_ENABLE_REG,
			buf | INTERRUPT_MASK_ABS0 | INTERRUPT_MASK_BUTTON) < 0)) {
		TOUCH_ERR_MSG("INTERRUPT_ENABLE_REG write fail\n");
		return -EIO;
	}

	if (ts->pdata->role->report_mode == CONTINUOUS_REPORT_MODE) {
		if (unlikely(touch_i2c_write_byte(client, TWO_D_REPORTING_MODE,
				REPORT_BEYOND_CLIP | ABS_FILTER | REPORT_MODE_CONTINUOUS) < 0)) {
			TOUCH_ERR_MSG("TWO_D_REPORTING_MODE write fail\n");
			return -EIO;
		}
		TOUCH_INFO_MSG("report_mode : CONTINUOUS_REPORT_MODE \n");
	} else {	/* REDUCED_REPORT_MODE */
		if (unlikely(touch_i2c_write_byte(client, TWO_D_REPORTING_MODE,
				REPORT_BEYOND_CLIP | ABS_FILTER | REPORT_MODE_REDUCED) < 0)) {
			TOUCH_ERR_MSG("TWO_D_REPORTING_MODE write fail\n");
			return -EIO;
		}
		TOUCH_INFO_MSG("report_mode : REDUCED_REPORT_MODE \n");

		if (unlikely(touch_i2c_write_byte(client, DELTA_X_THRESH_REG,
				ts->pdata->role->delta_pos_threshold) < 0)) {
			TOUCH_ERR_MSG("DELTA_X_THRESH_REG write fail\n");
			return -EIO;
		}
		if (unlikely(touch_i2c_write_byte(client, DELTA_Y_THRESH_REG,
				ts->pdata->role->delta_pos_threshold) < 0)) {
			TOUCH_ERR_MSG("DELTA_Y_THRESH_REG write fail\n");
			return -EIO;
		}
	}

	if (unlikely(touch_i2c_read(client, INTERRUPT_STATUS_REG, 1, &buf) < 0)) {
		TOUCH_ERR_MSG("INTERRUPT_STATUS_REG read fail\n");
		return -EIO;	/* it is critical problem because interrupt will not occur. */
	}

	if (unlikely(touch_i2c_read(client, FINGER_STATE_REG,
		(ts->num_of_data_points + 3)/4, ts->ts_data.finger.finger_status_reg) < 0)) {
		TOUCH_ERR_MSG("FINGER_STATE_REG read fail\n");
		return -EIO;	/* it is critical problem because interrupt will not occur on some FW. */
	}

	ts->is_probed = 1;

	DO_SAFE(lpwg_tap_control(ts, 0), error);
	ts->double_tap_enable = 0;
	ts->multi_tap_enable = 0;
	atomic_set(&ts->is_suspend, 0);

	TOUCH_DEBUG_MSG("synaptics_ts_init lpwg_mode : %d, %d, %d\n", ts->lpwg_mode, ts->double_tap_enable, ts->multi_tap_enable);
	return 0;
error:
	return -EIO;
}

int synaptics_ts_power(struct i2c_client *client, int power_ctrl)
{
	struct synaptics_ts_data *ts =
			(struct synaptics_ts_data *)get_touch_handle(client);

	if (touch_debug_mask & DEBUG_TRACE)
		TOUCH_DEBUG_MSG("\n");

	if (power_state == power_ctrl) {
		return 0;
	}
	switch (power_ctrl) {
	case POWER_OFF:
		if (ts->pdata->role->palm_detect_mode) {
			cancel_work_sync(&ts->palm_work);
			hrtimer_cancel(&ts->palm_timer);
		}
		cancel_work_sync(&ts->multi_tap_work);
		hrtimer_cancel(&ts->multi_tap_timer);

		if (ts->pdata->pwr->use_regulator) {
			regulator_disable(ts->regulator_vio);
			regulator_disable(ts->regulator_vdd);
		} else {
			if (ts->pdata->pwr->use_vio_regulator)
				regulator_disable(ts->regulator_vio);
			if (gpio_is_valid(ts->pdata->pwr->gpio_vdd_en))
				gpio_set_value(ts->pdata->pwr->gpio_vdd_en, 0);

		}
		if (gpio_is_valid(ts->pdata->reset_pin)) {
			gpio_set_value(ts->pdata->reset_pin, 0);
		}
		break;
	case POWER_ON:
		if (ts->pdata->pwr->use_regulator) {
			regulator_enable(ts->regulator_vdd);
			regulator_enable(ts->regulator_vio);
		} else {
			if (gpio_is_valid(ts->pdata->pwr->gpio_vdd_en))
				gpio_set_value(ts->pdata->pwr->gpio_vdd_en, 1);
			if (ts->pdata->pwr->use_vio_regulator)
				regulator_enable(ts->regulator_vio);
		}
		if (gpio_is_valid(ts->pdata->reset_pin)) {
			gpio_set_value(ts->pdata->reset_pin, 1);
		}
		break;
	case POWER_SLEEP:
		if (ts->pdata->role->palm_detect_mode) {
			cancel_work_sync(&ts->palm_work);
			hrtimer_cancel(&ts->palm_timer);
		}
		cancel_work_sync(&ts->multi_tap_work);
		hrtimer_cancel(&ts->multi_tap_timer);

		if (ts->lpwg_mode == LPWG_NONE) {
			if (unlikely(touch_i2c_write_byte(client, DEVICE_CONTROL_REG,
					DEVICE_CONTROL_SLEEP | DEVICE_CONTROL_CONFIGURED) < 0)) {
				TOUCH_ERR_MSG("DEVICE_CONTROL_REG write fail\n");
				return -EIO;
			}
		} else {
		}
		break;
	case POWER_WAKE:
		if (ts->lpwg_mode == LPWG_NONE) {
			if (unlikely(touch_i2c_write_byte(client, DEVICE_CONTROL_REG,
					DEVICE_CONTROL_NORMAL_OP | DEVICE_CONTROL_CONFIGURED) < 0)) {
				TOUCH_ERR_MSG("DEVICE_CONTROL_REG write fail\n");
				return -EIO;
			}
		} else {
			/* to escape noise status during LPWG_DOUBLE_TAP,LPWG_MULTI_TAP
			 * in case of using POWER_SLEEP,POWER_WAKE */
			if (unlikely(touch_i2c_write_byte(client, DEVICE_COMMAND_REG, 1))) {
				TOUCH_ERR_MSG("IC Reset command write fail \n");
				return -EIO;
			} else {
				TOUCH_INFO_MSG("SOFT RESET \n");
			}
		}
		break;
	default:
		return -EIO;
		break;
	}
	power_state = power_ctrl;

	return 0;
}

int synaptics_ts_probe(struct lge_touch_data *lge_touch_ts)
{
	struct synaptics_ts_data *ts;
	int ret = 0;
	struct i2c_client *client =	lge_touch_ts->client;

	if (touch_debug_mask & DEBUG_TRACE)
		TOUCH_DEBUG_MSG("\n");

	ts = kzalloc(sizeof(struct synaptics_ts_data), GFP_KERNEL);
	if (!ts) {
		TOUCH_ERR_MSG("Can not allocate memory\n");
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	set_touch_handle(client, ts);

	ts->client = client;
	ds4_i2c_client = client;
	ts->pdata = lge_touch_ts->pdata;

	if (ts->pdata->pwr->use_regulator) {
		ts->regulator_vdd = regulator_get(&client->dev, ts->pdata->pwr->vdd);
		if (IS_ERR(ts->regulator_vdd)) {
			TOUCH_ERR_MSG("FAIL: regulator_get_vdd - %s\n", ts->pdata->pwr->vdd);
			ret = -EPERM;
			goto err_get_vdd_failed;
		}

		ts->regulator_vio = regulator_get(&client->dev, ts->pdata->pwr->vio);
		if (IS_ERR(ts->regulator_vio)) {
			TOUCH_ERR_MSG("FAIL: regulator_get_vio - %s\n", ts->pdata->pwr->vio);
			ret = -EPERM;
			goto err_get_vio_failed;
		}

		if (ts->pdata->pwr->vdd_voltage > 0) {
			ret = regulator_set_voltage(ts->regulator_vdd, ts->pdata->pwr->vdd_voltage, ts->pdata->pwr->vdd_voltage);
			if (ret < 0)
				TOUCH_ERR_MSG("FAIL: VDD voltage setting - (%duV)\n", ts->pdata->pwr->vdd_voltage);
		}
	}

	if (ts->pdata->pwr->use_vio_regulator) {
		ts->regulator_vio = regulator_get(&client->dev, ts->pdata->pwr->vio);
		if (IS_ERR(ts->regulator_vio)) {
			TOUCH_ERR_MSG("FAIL: regulator_get_vio - %s\n", ts->pdata->pwr->vio);
			ret = -EPERM;
			goto err_get_vio_failed;
		}

		if (ts->pdata->pwr->vio_voltage > 0) {
			ret = regulator_set_voltage(ts->regulator_vio, ts->pdata->pwr->vio_voltage, ts->pdata->pwr->vio_voltage);
			if (ret < 0)
				TOUCH_ERR_MSG("FAIL: VIO voltage setting - (%duV)\n", ts->pdata->pwr->vio_voltage);
		}
	}

	if (ts->pdata->role->palm_detect_mode) {
		TOUCH_INFO_MSG("Palm detect mode enabled \n");
		touch_palm_wq = create_singlethread_workqueue("touch_palm_wq");
		if (touch_palm_wq) {
			ts->lge_touch_ts = lge_touch_ts;
			INIT_WORK(&ts->palm_work, touch_palm_work);
			hrtimer_init(&ts->palm_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
			ts->palm_timer.function = touch_palm_timer_handler;
		}
	}

	touch_multi_tap_wq = create_singlethread_workqueue("touch_multi_tap_wq");
	if (touch_multi_tap_wq) {
		ts->lge_touch_ts = lge_touch_ts;
		INIT_WORK(&ts->multi_tap_work, touch_multi_tap_work);
		hrtimer_init(&ts->multi_tap_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->multi_tap_timer.function = touch_multi_tap_timer_handler;
	}

	atomic_set(&ts->is_suspend, 0);

	return ret;

err_get_vio_failed:
	if (ts->pdata->pwr->use_regulator || ts->pdata->pwr->use_vio_regulator)
		regulator_put(ts->regulator_vio);
err_get_vdd_failed:
	if (ts->pdata->pwr->use_regulator)
		regulator_put(ts->regulator_vdd);
err_alloc_data_failed:
	kfree(ts);
	return ret;
}

void synaptics_ts_remove(struct i2c_client *client)
{
	struct synaptics_ts_data *ts =
			(struct synaptics_ts_data *)get_touch_handle(client);

	if (touch_debug_mask & DEBUG_TRACE)
		TOUCH_DEBUG_MSG("\n");

	if (ts->pdata->pwr->use_regulator) {
		regulator_put(ts->regulator_vio);
		regulator_put(ts->regulator_vdd);
	}

	kfree(ts);
}

int compare_fw_version(struct i2c_client *client, struct touch_fw_info *fw_info)
{
	struct synaptics_ts_data *ts =
			(struct synaptics_ts_data *)get_touch_handle(client);
	int i = 0;

	for (i = 0; i < FW_VER_INFO_NUM; i++) {
		if (ts->pdata->fw_version[i] != fw_info->update_fw_version[i] && i < FW_VER_INFO_NUM - 1) {
			if (i == 0 && ts->pdata->fw_version[0] & 0x80) {
				if ((ts->pdata->fw_version[i] & 0x0F) != (fw_info->update_fw_version[i] & 0x0F)) {
					TOUCH_INFO_MSG("firmware is not matching with device. ic_fw_ver_info[%d]:0x%02X != fw_version[%d]:0x%02X\n",
						i, ts->pdata->fw_version[i] & 0x0F, i, fw_info->update_fw_version[i] & 0x0F);
					return -1;
				}
			} else {
				TOUCH_INFO_MSG("firmware is not matching with device. ic_fw_ver_info[%d]:0x%02X != fw_version[%d]:0x%02X\n",
					i, ts->pdata->fw_version[i], i, fw_info->update_fw_version[i]);
				return -1;
			}
		} else {
			if (fw_info->ic_fw_version[i] != fw_info->update_fw_version[i]) {
				TOUCH_INFO_MSG("fw version mismatch. ic_fw_version[%d]:0x%02X != fw_version[%d]:0x%02X\n",
				i, fw_info->ic_fw_version[i], i, fw_info->update_fw_version[i]);
				return 1;
			}
		}
	}
	TOUCH_INFO_MSG("fw version match\n")
	return 0;
}

int synaptics_ts_fw_upgrade(struct i2c_client *client, struct touch_fw_info *fw_info)
{
	struct synaptics_ts_data *ts =
			(struct synaptics_ts_data *)get_touch_handle(client);
	int ret = 0;
	const char *fw_path = NULL;
	const u8 *ptr = NULL;

	ts->is_probed = 0;

	if (fw_info->fw_upgrade.fw_force_upgrade && strcmp(ts->fw_info.config_id, "ERR"))
		fw_path = fw_info->fw_upgrade.fw_path;
	else
		fw_path = ts->pdata->inbuilt_fw_name;

	TOUCH_INFO_MSG("synaptics_ts_fw_upgrade, fw_path : %s \n", fw_path);
	ret = request_firmware((const struct firmware **) (&fw_info->fw_upgrade.fw), fw_path, &client->dev);
	if (ret != 0) {
		TOUCH_ERR_MSG("request_firmware() failed %d\n", ret);
		goto out;
	}


	ptr = fw_info->fw_upgrade.fw->data;
	memcpy(fw_info->update_fw_version, &ptr[0Xb100], sizeof(u8)*4);

	if (fw_info->ic_fw_version[0] > 0x40) {
		if (fw_info->update_fw_version[0] > 0x40) {
			TOUCH_INFO_MSG("IC identifier[%s] fw_version[%s(ic):%s(fw)] panel_type[%d] : force[%d]\n",
				fw_info->ic_fw_identifier, fw_info->ic_fw_version,
				fw_info->update_fw_version, ts->pdata->panel_type, fw_info->fw_upgrade.fw_force_upgrade);
		} else {
			TOUCH_INFO_MSG("IC identifier[%s] fw_version[%s(ic):V%d.%02d(fw)] panel_type[%d] : force[%d]\n",
				fw_info->ic_fw_identifier, fw_info->ic_fw_version,
				(fw_info->update_fw_version[3]&0x80 ? 1 : 0), fw_info->update_fw_version[3]&0x7F,
				ts->pdata->panel_type, fw_info->fw_upgrade.fw_force_upgrade);
			TOUCH_INFO_MSG("update_fw_info \n%s", productcode_parse(fw_info->update_fw_version));
		}
		goto firmware_upgrade;
	}

	if (fw_info->update_fw_version[0] > 0x40) {
		TOUCH_INFO_MSG("ic_fw_ver : %d.%02d \n",
			(fw_info->ic_fw_version[3]&0x80 ? 1 : 0), fw_info->ic_fw_version[3]&0x7F);
		TOUCH_INFO_MSG("ic_fw_info \n%s", productcode_parse(fw_info->ic_fw_version));
		TOUCH_INFO_MSG("update_fw_ver : %s\n", fw_info->update_fw_version);
	} else {
		TOUCH_INFO_MSG("ic_fw_ver : V%d.%02d(0x%02X 0x%02X 0x%02X 0x%02X)\n ",
			(fw_info->ic_fw_version[3]&0x80 ? 1 : 0), fw_info->ic_fw_version[3]&0x7F,
			fw_info->ic_fw_version[0], fw_info->ic_fw_version[1], fw_info->ic_fw_version[2], fw_info->ic_fw_version[3]);
		TOUCH_INFO_MSG("ic_fw_info \n%s", productcode_parse(fw_info->ic_fw_version));
		TOUCH_INFO_MSG("update_fw_ver : V%d.%02d(0x%02X 0x%02X 0x%02X 0x%02X) \n",
			(fw_info->update_fw_version[3]&0x80 ? 1 : 0), fw_info->update_fw_version[3]&0x7F,
			fw_info->update_fw_version[0], fw_info->update_fw_version[1],
			fw_info->update_fw_version[2], fw_info->update_fw_version[3]);
		TOUCH_INFO_MSG("update_fw_info \n%s", productcode_parse(fw_info->update_fw_version));
	}


	if (fw_info->fw_upgrade.fw_force_upgrade)
		goto firmware_upgrade;

	if (compare_fw_version(client, fw_info) > 0)
		goto firmware_upgrade;
	else
		goto out;

firmware_upgrade:
	ret = FirmwareUpgrade(ts, fw_info->fw_upgrade.fw);
	/* update IC info */
	if (ret >= 0) {
		get_ic_info(ts, fw_info);
		if (fw_info->update_fw_version[0] < 0x40) {
			TOUCH_INFO_MSG("ic_fw_info \n%s\n", productcode_parse(fw_info->ic_fw_version));
		}
	}
	return ret;
out:
	TOUCH_INFO_MSG("FW-upgrade is not executed\n");
	memset(&fw_info->fw_upgrade, 0, sizeof(fw_info->fw_upgrade));
	return ret;
}

err_t synaptics_ts_lpwg(struct i2c_client *client, u32 code, u32 value, struct point *data);
int synaptics_ts_ic_ctrl(struct i2c_client *client, u8 code, u32 value)
{
	struct synaptics_ts_data *ts =
			(struct synaptics_ts_data *)get_touch_handle(client);
	u8 buf = 0;

	switch (code) {
	case IC_CTRL_BASELINE:
		switch (value) {
		case BASELINE_OPEN:

			break;

			if (unlikely(synaptics_ts_page_data_write_byte(client, ANALOG_PAGE,
					ANALOG_CONTROL_REG, FORCE_FAST_RELAXATION) < 0)) {
				TOUCH_ERR_MSG("ANALOG_CONTROL_REG write fail\n");
				return -EIO;
			}

			msleep(10);

			if (unlikely(synaptics_ts_page_data_write_byte(client, ANALOG_PAGE,
					ANALOG_COMMAND_REG, FORCE_UPDATE) < 0)) {
				TOUCH_ERR_MSG("ANALOG_COMMAND_REG write fail\n");
				return -EIO;
			}

			if (unlikely(touch_debug_mask & DEBUG_GHOST))
				TOUCH_INFO_MSG("BASELINE_OPEN\n");

			break;
		case BASELINE_FIX:

			break;

			if (unlikely(synaptics_ts_page_data_write_byte(client, ANALOG_PAGE,
					ANALOG_CONTROL_REG, 0x00) < 0)) {
				TOUCH_ERR_MSG("ANALOG_CONTROL_REG write fail\n");
				return -EIO;
			}

			msleep(10);

			if (unlikely(synaptics_ts_page_data_write_byte(client, ANALOG_PAGE,
					ANALOG_COMMAND_REG, FORCE_UPDATE) < 0)) {
				TOUCH_ERR_MSG("ANALOG_COMMAND_REG write fail\n");
				return -EIO;
			}

			if (unlikely(touch_debug_mask & DEBUG_GHOST))
				TOUCH_INFO_MSG("BASELINE_FIX\n");

			break;
		case BASELINE_REBASE:
			/* rebase base line */
			if (likely(ts->finger_fc.dsc.id != 0)) {
				if (unlikely(touch_i2c_write_byte(client, FINGER_COMMAND_REG, 0x1) < 0)) {
					TOUCH_ERR_MSG("finger baseline reset command write fail\n");
					return -EIO;
				}
			}
			break;
		default:
			break;
		}
		break;

	case IC_CTRL_READ:
		if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, ((value & 0xFF00) >> 8)) < 0)) {
			TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
			return -EIO;
		}

		if (touch_i2c_read(client, (value & 0xFF), 1, &buf) < 0) {
			TOUCH_ERR_MSG("IC register read fail\n");
			return -EIO;
		}

		if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE) < 0)) {
			TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
			return -EIO;
		}
		break;

	case IC_CTRL_WRITE:
		if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, ((value & 0xFF0000) >> 16)) < 0)) {
			TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
			return -EIO;
		}

		if (touch_i2c_write_byte(client, ((value & 0xFF00) >> 8), (value & 0xFF)) < 0) {
			TOUCH_ERR_MSG("IC register write fail\n");
			return -EIO;
		}

		if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE) < 0)) {
			TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
			return -EIO;
		}
		break;

	case IC_CTRL_RESET_CMD:
		if (unlikely(touch_i2c_write_byte(client, DEVICE_COMMAND_REG, 0x1) < 0)) {
			TOUCH_ERR_MSG("IC Reset command write fail\n");
			return -EIO;
		}
		break;

	case IC_CTRL_REPORT_MODE:
		switch (value) {
		case 0:   /* continuous mode */
			if (unlikely(touch_i2c_write_byte(client, TWO_D_REPORTING_MODE,
				REPORT_BEYOND_CLIP | ABS_FILTER | REPORT_MODE_CONTINUOUS) < 0)) {
				TOUCH_ERR_MSG("TWO_D_REPORTING_MODE write fail\n");
				return -EIO;
			}
			break;
		case 1:  /* reduced mode */
			if (unlikely(touch_i2c_write_byte(client, TWO_D_REPORTING_MODE,
					REPORT_BEYOND_CLIP | ABS_FILTER | REPORT_MODE_REDUCED) < 0)) {
				TOUCH_ERR_MSG("TWO_D_REPORTING_MODE write fail\n");
				return -EIO;
			}

			break;
		default:
			break;
		}
		break;

	case IC_CTRL_DOUBLE_TAP_WAKEUP_MODE:
		switch (value) {
		unsigned char r_mem = 0;

		case 0: /* touch double-tap disable */
			TOUCH_INFO_MSG("ic_ctrl: IC_CTRL_DOUBLE_TAP_WAKEUP_MODE --- OFF\n");
			if (touch_i2c_read(client, LPWG_CONTROL_REG, 1, &r_mem) < 0) {
				TOUCH_ERR_MSG("LPWG_CONTROL_REG read fail!\n");
				return -EIO;
			} else {
				r_mem = r_mem & 0xfe;
				if (touch_i2c_write(client, LPWG_CONTROL_REG, 1, &r_mem) < 0) {
					TOUCH_ERR_MSG("LPWG_CONTROL_REG write fail\n");
					return -EIO;
				}
			}

			if (touch_i2c_read(client, TWO_D_REPORTING_MODE, 1, &r_mem) < 0) {
				TOUCH_ERR_MSG("2D_REPORT_MODE_REG read fail!\n");
			} else {
				r_mem = r_mem & 0xf8;
				if (touch_i2c_write(client, TWO_D_REPORTING_MODE, 1, &r_mem) < 0) {
					TOUCH_ERR_MSG("2D_REPORT_MODE_REG write fail\n");
					return -EIO;
				}
			}
			break;

		case 1: /* touch double-tap enable */
			TOUCH_INFO_MSG("ic_ctrl: IC_CTRL_DOUBLE_TAP_WAKEUP_MODE --- ON\n");
			if (touch_i2c_read(client, LPWG_CONTROL_REG, 1, &r_mem) < 0) {
				TOUCH_ERR_MSG("LPWG_CONTROL_REG read fail!\n");
				return -EIO;
			} else {
				r_mem = r_mem | 0x01;
				if (touch_i2c_write(client, LPWG_CONTROL_REG, 1, &r_mem) < 0) {
					TOUCH_ERR_MSG("LPWG_CONTROL_REG write fail\n");
					return -EIO;
				}
			}

			if (touch_i2c_read(client, TWO_D_REPORTING_MODE, 1, &r_mem) < 0) {
				TOUCH_ERR_MSG("TWO_D_REPORTING_MODE read fail!\n");
				return -EIO;
			} else {
				r_mem = (r_mem & 0xf8) | 0x4;
				if (touch_i2c_write(client, TWO_D_REPORTING_MODE, 1, &r_mem) < 0) {
					TOUCH_ERR_MSG("TWO_D_REPORTING_MODE write fail\n");
					return -EIO;
				}
			}
			if (touch_i2c_read(client, DEVICE_CONTROL_REG, 1, &r_mem) < 0) {
				TOUCH_ERR_MSG("DEVICE_CONTROL_REG read fail!\n");
				return -EIO;
			} else {
				r_mem = 0x80;
				if (touch_i2c_write(client, DEVICE_CONTROL_REG, 1, &r_mem) < 0) {
					TOUCH_ERR_MSG("SET_NO_SLEEP_REG write fail\n");
					return -EIO;
				}
			}

			if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, LPWG_CTRL_PAGE) < 0)) {
				TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
				return -EIO;
			}
			if (touch_i2c_read(client, INTERTAP_DISTANCE_REG, 1, &r_mem) < 0) {
				TOUCH_ERR_MSG("INTERTAP_DISTANCE_REG read fail!\n");
				return -EIO;
			} else {
				r_mem = 0x0A;
				if (touch_i2c_write(client, INTERTAP_DISTANCE_REG, 1, &r_mem) < 0) {
					TOUCH_ERR_MSG("Double TAP Maximum InterTap Distance reg(0x04P:0x4B) write fail\n");
					return -EIO;
				} else {
					TOUCH_INFO_MSG("Double TAP Maximum InterTap Distance = 0x%02x\n", r_mem);
				}
			}
			if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE) < 0)) {
				TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
				return -EIO;
			}
			DO_SAFE(synaptics_ts_page_data_read(client, LPWG_CTRL_PAGE, MULTITAP_COUNT_REG, 1, &buf), error);
/*				TOUCH_DEBUG_MSG("TAP COUNT %d\n", 2);	for double tap mode TAP COUNT */
			TOUCH_DEBUG_MSG("Double TAP COUNT\n");
			buf = (buf & 0x07) | (2 << 3);
			TOUCH_DEBUG_MSG("MultiTap LPWG Control Reg value 0x%02X\n", buf);
			DO_SAFE(synaptics_ts_page_data_write_byte(client, LPWG_CTRL_PAGE, MULTITAP_COUNT_REG, buf), error);
			DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE), error);

		break;

		case 2: /* touch multi tap enable */
			TOUCH_INFO_MSG("ic_ctrl: IC_CTRL_MULTI_TAP_WAKEUP_MODE --- ON\n");

			if (touch_i2c_read(client, LPWG_CONTROL_REG, 1, &r_mem) < 0) {
				TOUCH_ERR_MSG("LPWG_CONTROL_REG read fail!\n");
			} else {
				r_mem = r_mem | 0x01;
				if (touch_i2c_write(client, LPWG_CONTROL_REG, 1, &r_mem) < 0) {
					TOUCH_ERR_MSG("LPWG_CONTROL_REG write fail\n");
					return -EIO;
				}
			}
			if (touch_i2c_read(client, TWO_D_REPORTING_MODE, 1, &r_mem) < 0) {
				TOUCH_ERR_MSG("TWO_D_REPORTING_MODE read fail!\n");
				return -EIO;
			} else {
				r_mem = (r_mem & 0xf8) | 0x4;
				if (touch_i2c_write(client, TWO_D_REPORTING_MODE, 1, &r_mem) < 0) {
					TOUCH_ERR_MSG("TWO_D_REPORTING_MODE write fail\n");
					return -EIO;
				}
			}
			if (touch_i2c_read(client, DEVICE_CONTROL_REG, 1, &r_mem) < 0) {
				TOUCH_ERR_MSG("SET_NO_SLEEP_REG read fail!\n");
				return -EIO;
			} else {
				r_mem = 0x80;
				if (touch_i2c_write(client, DEVICE_CONTROL_REG, 1, &r_mem) < 0) {
					TOUCH_ERR_MSG("SET_NO_SLEEP_REG write fail\n");
					return -EIO;
				}
			}

			if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, LPWG_CTRL_PAGE) < 0)) {
				TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
				return -EIO;
			}
			if (touch_i2c_read(client, MAX_INTERTAP_TIME_REG, 1, &r_mem) < 0) {
				TOUCH_ERR_MSG("MAX_INTERTAP_TIME_REG read fail!\n");
				return -EIO;
			} else {
				r_mem = 0x46;
				if (touch_i2c_write(client, MAX_INTERTAP_TIME_REG, 1, &r_mem) < 0) {
					TOUCH_ERR_MSG("MultiTap Maximum InterTap Time reg(0x04P:0x49) write fail\n");
					return -EIO;
				} else {
					TOUCH_INFO_MSG("MultiTap Maximum InterTap Time = 0x%02x\n", r_mem);
				}
			}
			if (touch_i2c_read(client, INTERTAP_DISTANCE_REG, 1, &r_mem) < 0) {
				TOUCH_ERR_MSG("INTERTAP_DISTANCE_REG read fail!\n");
				return -EIO;
			} else {
				r_mem = 0xFF;
				if (touch_i2c_write(client, INTERTAP_DISTANCE_REG, 1, &r_mem) < 0) {
					TOUCH_ERR_MSG("Multi TAP Maximum InterTap Distance reg(0x04P:0x4B) write fail\n");
					return -EIO;
				} else {
					TOUCH_INFO_MSG("Multi TAP Maximum InterTap Distance = 0x%02x\n", r_mem);
				}
			}
			if (unlikely(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE) < 0)) {
				TOUCH_ERR_MSG("PAGE_SELECT_REG write fail\n");
				return -EIO;
			}

			synaptics_ts_lpwg(ts->client, LPWG_TAP_COUNT, ts->multi_tap_count, NULL);
			break;

		default:
			break;
		}
	default:
		break;
	}

	return buf;
error:
	return -EIO;
}

static char *productcode_parse(unsigned char *product)
{
	static char str[128] = {0};
	int len = 0;
	char inch[2] = {0};
	char paneltype = 0;
	char version[2] = {0};

	switch ((product[0] & 0xF0) >> 4) {
	case 0:
		len += sprintf(str+len, "ELK ");
		 break;
	case 1:
		 len += sprintf(str+len, "Suntel ");
		 break;
	case 2:
		 len += sprintf(str+len, "Tovis ");
		 break;
	case 3:
		 len += sprintf(str+len, "Innotek ");
		 break;
	default:
		 len += sprintf(str+len, "Unknown ");
		 break;
	}

	len += sprintf(str+len, "\n");

	switch (product[0] & 0x0F) {
	case 0:
		len += sprintf(str+len, "0key ");
		break;
	case 2:
		 len += sprintf(str+len, "2Key ");
		 break;
	case 3:
		 len += sprintf(str+len, "3Key ");
		 break;
	case 4:
		 len += sprintf(str+len, "4Key ");
		 break;
	default:
		 len += sprintf(str+len, "Unknown ");
		 break;
	}

	len += sprintf(str+len, "\n");

	switch ((product[1] & 0xF0) >> 4) {
	case 0:
		 len += sprintf(str+len, "Synaptics ");
		 break;
	default:
		 len += sprintf(str+len, "Unknown ");
		 break;
	}

	len += sprintf(str+len, "\n");

	inch[0] = (product[1] & 0x0F);
	inch[1] = ((product[2] & 0xF0) >> 4);
	len += sprintf(str+len, "%d.%d ", inch[0], inch[1]);

	len += sprintf(str+len, "\n");

	paneltype = (product[2] & 0x0F);
	len += sprintf(str+len, "PanelType %d ", paneltype);

	len += sprintf(str+len, "\n");

	version[0] = ((product[3] & 0x80) >> 7);
	version[1] = (product[3] & 0x7F);
	len += sprintf(str+len, "v%d.%02d\n ", version[0], version[1]);

	return str;
 }

static char *productcode_parse_short(unsigned char *product)
{
	static char str[128] = {0};
	int len = 0;
	char inch[2] = {0};
	char paneltype = 0;

	switch ((product[0] & 0xF0) >> 4) {
	case 0:
		len += sprintf(str+len, "E");
		break;
	case 1:
		len += sprintf(str+len, "S");
		break;
	case 2:
		len += sprintf(str+len, "T");
		break;
	case 3:
		len += sprintf(str+len, "I");
		break;
	default:
		len += sprintf(str+len, "Unknown ");
		break;
	}

	switch (product[0] & 0x0F) {
	case 0:
		len += sprintf(str+len, "0");
		break;
	case 2:
		len += sprintf(str+len, "2");
		break;
	case 3:
		len += sprintf(str+len, "3");
		break;
	case 4:
		len += sprintf(str+len, "4");
		break;
	default:
		len += sprintf(str+len, "Unknown ");
		break;
	}

	switch ((product[1] & 0xF0) >> 4) {
	case 0:
		len += sprintf(str+len, "S");
		break;
	default:
		len += sprintf(str+len, "Unknown ");
		break;
	}

	inch[0] = (product[1] & 0x0F);
	inch[1] = ((product[2] & 0xF0) >> 4);
	len += sprintf(str+len, "%d%d", inch[0], inch[1]);

	paneltype = (product[2] & 0x0F);
	len += sprintf(str+len, "P%d", paneltype);

	return str;
}

static ssize_t synaptics_fw_info_show(struct synaptics_ts_data *ts, char *buf, struct touch_fw_info *fw_info)
{
	int ret = 0;

	ret = sprintf(buf, "\n======== Firmware Info ========\n");
	ret += sprintf(buf+ret, "ic_fw_identifier  = %s\n", fw_info->ic_fw_identifier);
	if (ts->pdata->panel_id != 0xFF)
		ret += sprintf(buf+ret, "Panel id			= %d \n",  ts->pdata->panel_id);
	if (fw_info->ic_fw_version[0] > 0x40) {
		ret += sprintf(buf+ret, "ic_fw_version = %s\n", fw_info->ic_fw_version);
	} else {
		ret += sprintf(buf+ret, "ic_fw_version RAW = %02X %02X %02X %02X\n",
			fw_info->ic_fw_version[0], fw_info->ic_fw_version[1],
			fw_info->ic_fw_version[2], fw_info->ic_fw_version[3]);
		ret += sprintf(buf+ret, "ic_fw_version \n%s\n", productcode_parse(fw_info->ic_fw_version));
	}
	return ret;
}

static ssize_t synaptics_atcmd_fw_ver_show(struct synaptics_ts_data *ts, char *buf, struct touch_fw_info *fw_info)
{
	int ret = 0;
	if (fw_info->ic_fw_version[0] > 0x40) {
		ret += sprintf(buf+ret, "%s\n", fw_info->ic_fw_version);
	} else {
		ret = sprintf(buf, "V%d.%02d (0x%X/0x%X/0x%X/0x%X)\n",
			(fw_info->ic_fw_version[3]&0x80 ? 1 : 0), fw_info->ic_fw_version[3]&0x7F,
			fw_info->ic_fw_version[0], fw_info->ic_fw_version[1],
			fw_info->ic_fw_version[2], fw_info->ic_fw_version[3]);
	}
	return ret;
}

static ssize_t synaptics_fw_ver_show(struct synaptics_ts_data *ts, char *buf, struct touch_fw_info *fw_info)
{
	int ret = 0;
	if (fw_info->ic_fw_version[0] > 0x40) {
		ret += sprintf(buf+ret, "Firmware Version	= %s\n", fw_info->ic_fw_version);
	} else {
		ret += sprintf(buf+ret, "Firmware Version	= V%d.%02d (0x%02X, 0x%02X, 0x%02X, 0x%02X)\n",
			(fw_info->ic_fw_version[3]&0x80 ? 1 : 0), fw_info->ic_fw_version[3]&0x7F,
			fw_info->ic_fw_version[0], fw_info->ic_fw_version[1],
			fw_info->ic_fw_version[2], fw_info->ic_fw_version[3]);
		ret += sprintf(buf+ret, "FW Product		= %s \n", productcode_parse_short(fw_info->ic_fw_version));
	}
	return ret;
}

static ssize_t show_sd_(struct synaptics_ts_data *ts, char *buf, struct touch_fw_info *fw_info)
{
	int ret = 0;
	int rx_to_rx = 0;
	int tx_to_tx = 0;
	int tx_to_gnd = 0;
	int high_registance = 0;
	int full_raw_cap = 0;

	if (power_state == POWER_ON || power_state == POWER_WAKE) {
		ret += sprintf(buf+ret, "\n\n\n");
		write_log(buf);
		msleep(30);
		write_time_log();
		msleep(30);
		ret += sprintf(buf+ret, "Firmware Version : %d.%02d \n", (fw_info->ic_fw_version[3]&0x80 ? 1 : 0), fw_info->ic_fw_version[3]&0x7F);
		ret += sprintf(buf+ret, "FW ID            : %02X%02X%02X%02X\n", fw_info->ic_fw_version[0], fw_info->ic_fw_version[1], fw_info->ic_fw_version[2], fw_info->ic_fw_version[3]);
		ret += sprintf(buf+ret, "FW Product	 : %s \n", productcode_parse_short(fw_info->ic_fw_version));
		write_log(buf);
		msleep(30);

		SYNA_PDTScan();
		SYNA_ConstructRMI_F54();
		SYNA_ConstructRMI_F1A();

		touch_disable_irq(ts->client->irq);

		rx_to_rx = F54_RxToRxReport();

		if (rx_to_rx == 2) {
			ret = 0;
			ret += sprintf(buf+ret, "\nRxToRxReport read RMI fail!! \n");
			write_log(buf);
		}

		tx_to_tx = F54_TxToTxReport();
		tx_to_gnd = F54_TxToGndReport();
		high_registance = F54_HighResistance();

		if (get_limit(numberOfTx, numberOfRx, *ts->client, ts->pdata) < 0) {
			TOUCH_INFO_MSG("Can not check the limit of rawcap\n");
			full_raw_cap = F54_FullRawCap(5);
		} else {
			full_raw_cap = F54_FullRawCap(0);
		}

		ret += sprintf(buf+ret, "=======RESULT========\n");
		ret += sprintf(buf+ret, "Channel Status : %s\n", (tx_to_tx && tx_to_gnd && high_registance) ? "Pass" : "Fail");
		ret += sprintf(buf+ret, "Raw Data : %s\n", (full_raw_cap > 0) ? "Pass" : "Fail");

		synaptics_ts_init(ts->client, NULL);

		touch_enable_irq(ts->client->irq);

	} else {
		write_time_log();
		ret += sprintf(buf+ret, "state=[suspend]. we cannot use I2C, now. Test Result: Fail\n");
	}

	return ret;
}

static int pdt_scan;

/* mode - 0:For sensor, 1:For FPC, 2:CheckTSPConnection, 3:Baseline, 4:Delta image */
static ssize_t __get_f54_full_raw_cap(struct synaptics_ts_data *ts, char *buf, int mode)
{
	int ret = 0;

	if (power_state == POWER_ON || power_state == POWER_WAKE) {

		if (pdt_scan == 0) {
			pdt_scan = 1;
			SYNA_PDTScan();
			SYNA_ConstructRMI_F54();
			SYNA_ConstructRMI_F1A();
		}
		touch_disable_irq(ts->client->irq);

		ret = F54_GetFullRawCap(mode, buf);

		touch_enable_irq(ts->client->irq);
	} else {
		ret += sprintf(buf+ret, "state=[suspend]. we cannot use I2C, now. Test Result: Fail\n");
	}
	if (ret == 0)
		ret = sprintf(buf+ret, "ERROR: full_raw_cap failed.\n");

	return ret;
}

static ssize_t show_rawdata_(struct synaptics_ts_data *ts, char *buf, struct touch_fw_info *fw_info)
{
	if (get_limit(numberOfTx, numberOfRx, *ts->client, ts->pdata) < 0) {
			TOUCH_INFO_MSG("Can not check the limit of rawcap\n");
			return __get_f54_full_raw_cap(ts, buf, 5);
		} else {
			return __get_f54_full_raw_cap(ts, buf, 0);
	}
}

static ssize_t show_delta_(struct synaptics_ts_data *ts, char *buf, struct touch_fw_info *fw_info)
{
	return __get_f54_full_raw_cap(ts, buf, 4);
}
static ssize_t show_chstatus_(struct synaptics_ts_data *ts, char *buf, struct touch_fw_info *fw_info)
{
	int ret = 0;

	if (power_state == POWER_ON || power_state == POWER_WAKE) {
		touch_disable_irq(ts->client->irq);
		if (pdt_scan == 0) {
			pdt_scan = 1;
			SYNA_PDTScan();
			SYNA_ConstructRMI_F54();
			SYNA_ConstructRMI_F1A();
		}

		ret += sprintf(buf+ret, "Firmware Version : %d.%02d \n", (fw_info->ic_fw_version[3]&0x80 ? 1 : 0), fw_info->ic_fw_version[3]&0x7F);
		ret += sprintf(buf+ret, "FW ID            : %02X%02X%02X%02X\n", fw_info->ic_fw_version[0], fw_info->ic_fw_version[1], fw_info->ic_fw_version[2], fw_info->ic_fw_version[3]);
		ret += sprintf(buf+ret, "FW Product	 : %s \n", productcode_parse_short(fw_info->ic_fw_version));
		ret += sprintf(buf+ret, "=======RESULT========\n");
		ret += sprintf(buf+ret, "RxToRxReport   : ");
		ret += F54_GetRxToRxReport(buf+ret);
		ret += sprintf(buf+ret, "TxToTxReport   : ");
		ret += F54_GetTxToTxReport(buf+ret);
		ret += sprintf(buf+ret, "TxToGndReport  : ");
		ret += F54_GetTxToGndReport(buf+ret);
		ret += sprintf(buf+ret, "HighResistance : ");
		ret += F54_GetHighResistance(buf+ret);

		synaptics_ts_init(ts->client, NULL);

		touch_enable_irq(ts->client->irq);
	} else {
		ret += sprintf(buf, "ERROR: state=[suspend]. we cannot use I2C, now. Test Result: Fail\n");
	}

	if (ret == 0)
		ret += sprintf(buf+ret, "ERROR: full_raw_cap failed.\n");

	return ret;
}

int synaptics_ts_sysfs(struct i2c_client *client, char *buf, u8 code, struct touch_fw_info *fw_info)
{
	int ret = 0;
	struct synaptics_ts_data *ts =
			(struct synaptics_ts_data *)get_touch_handle(client);

	switch (code) {
	case SYSFS_SYNAPTICS_VERSION_SHOW:
		ret = synaptics_fw_info_show(ts, buf, fw_info);
		break;
	case SYSFS_SYNAPTICS_ATCMD_VERSION_SHOW:
		ret = synaptics_atcmd_fw_ver_show(ts, buf, fw_info);
		break;
	case SYSFS_CHSTATUS_SHOW:
		ret = show_chstatus_(ts, buf, fw_info);
		break;
	case SYSFS_RAWDATA_SHOW:
		ret = show_rawdata_(ts, buf, NULL);
		break;
	case SYSFS_DELTA_SHOW:
		ret = show_delta_(ts, buf, NULL);
		break;
	case SYSFS_SELF_DIAGNOSTIC_SHOW:
		ret = show_sd_(ts, buf, fw_info);
		break;
	case SYSFS_SYNAPTICS_FW_VERSION_SHOW:
		ret = synaptics_fw_ver_show(ts, buf, fw_info);
		break;

	}

	return ret;
}

static int lpwg_tap_control(struct synaptics_ts_data *ts, int on)
{
	if (on) {
		if (ts->lpwg_mode == LPWG_DOUBLE_TAP)
			synaptics_ts_ic_ctrl(ts->client, IC_CTRL_DOUBLE_TAP_WAKEUP_MODE, 1);
		else if (ts->lpwg_mode == LPWG_MULTI_TAP)
			synaptics_ts_ic_ctrl(ts->client, IC_CTRL_DOUBLE_TAP_WAKEUP_MODE, 2);
	} else {
		synaptics_ts_ic_ctrl(ts->client, IC_CTRL_DOUBLE_TAP_WAKEUP_MODE, 0);
	}
	return 0;
}

err_t synaptics_ts_suspend(struct i2c_client *client)
{
	struct synaptics_ts_data *ts = (struct synaptics_ts_data *)get_touch_handle(client);

	multi_tap_fail_try = 0;
	if (atomic_read(&ts->is_suspend))
		return NO_ERROR;

	switch (ts->lpwg_mode) {
	case LPWG_DOUBLE_TAP:
		DO_SAFE(lpwg_tap_control(ts, 1), error);
		ts->double_tap_enable = 1;
		break;
	case LPWG_MULTI_TAP:
		DO_SAFE(lpwg_tap_control(ts, 1), error);
		ts->multi_tap_enable = 1;
		break;
	default:
		break;
	}
	atomic_set(&ts->is_suspend, 1);
	TOUCH_DEBUG_MSG("synaptics_ts_suspend lpwg_mode : %d, %d, %d\n", ts->lpwg_mode, ts->double_tap_enable, ts->multi_tap_enable);

    return NO_ERROR;
error:
    return ERROR;
}

err_t synaptics_ts_resume(struct i2c_client *client)
{
    return NO_ERROR;
}

err_t synaptics_ts_lpwg(struct i2c_client *client, u32 code, u32 value, struct point *data)
{
    struct synaptics_ts_data *ts = (struct synaptics_ts_data *)get_touch_handle(client);
	u8 buf = 0;
	int i = 0;

	switch (code) {
	case LPWG_READ:
		if (ts->multi_tap_enable) {
			if (custom_gesture_status == 0) {
				data[0].x = 1;
				data[0].y = 1;
				data[1].x = -1;
				data[1].y = -1;
				break;
			}
			for (i = 0; i < ts->multi_tap_count; i++) {
				data[i].x = (lpwg_data[4*i+1]<<8 | lpwg_data[4*i]) / ts->pdata->caps->lcd_touch_ratio_x;
				data[i].y = (lpwg_data[4*i+3]<<8 | lpwg_data[4*i+2]) / ts->pdata->caps->lcd_touch_ratio_y;
				/* TOUCH_DEBUG_MSG("TAP Position x[%3d], y[%3d]\n", data[i].x, data[i].y);	multi-tap coordinates */
				/* TOUCH_DEBUG_MSG("LPWG_READ x[###], y[###]\n"); */
				/* '-1' should be assinged to the last data. */
				/* Each data should be converted to LCD-resolution. */
			}
			data[i].x = -1;
			data[i].y = -1;
		}
		break;
	case LPWG_ENABLE:
		ts->lpwg_mode = value;
		TOUCH_DEBUG_MSG("synaptics_ts_lpwg lpwg_mode : %d, %d, %d", ts->lpwg_mode, ts->double_tap_enable, ts->multi_tap_enable);
		/* The 'lpwg_mode' is changed to 'value' but it is applied in suspend-state. */
		if (atomic_read(&ts->is_suspend)) {
			if (ts->lpwg_mode) {
				TOUCH_DEBUG_MSG("touch power state : %d lpwg_mode : %d", power_state, ts->lpwg_mode);
				if (power_state == POWER_OFF) {
					synaptics_ts_power(client, POWER_ON);
					msleep(100);
					TOUCH_DEBUG_MSG("synaptics_ts_power POWER ON!!\n");
					if (unlikely(touch_i2c_write_byte(client, DEVICE_CONTROL_REG,
									DEVICE_CONTROL_NORMAL_OP | DEVICE_CONTROL_CONFIGURED) < 0)) {
						TOUCH_ERR_MSG("DEVICE_CONTROL_REG write fail\n");
					} else
						TOUCH_DEBUG_MSG("DEVICE_CONTROL_NORMAL_OP\n");
					if (unlikely(touch_i2c_read(client, INTERRUPT_ENABLE_REG,
									1, &buf) < 0)) {
						TOUCH_ERR_MSG("INTERRUPT_ENABLE_REG read fail\n");
					}
					if (unlikely(touch_i2c_write_byte(client, INTERRUPT_ENABLE_REG,
									buf | INTERRUPT_MASK_ABS0 | INTERRUPT_MASK_BUTTON) < 0)) {
						TOUCH_ERR_MSG("INTERRUPT_ENABLE_REG write fail\n");
					}
					touch_enable_irq(ts->client->irq);
					if (unlikely(touch_i2c_read(client, INTERRUPT_STATUS_REG, 1, &buf) < 0)) {
						TOUCH_ERR_MSG("INTERRUPT_STATUS_REG read fail\n");
					}

				} else if (power_state == POWER_SLEEP) {
					synaptics_ts_power(client, POWER_WAKE);
					msleep(100);
					TOUCH_DEBUG_MSG("synaptics_ts_power POWER WAKE!!\n");
					if (unlikely(touch_i2c_write_byte(client, DEVICE_CONTROL_REG,
									DEVICE_CONTROL_NORMAL_OP | DEVICE_CONTROL_CONFIGURED) < 0)) {
						TOUCH_ERR_MSG("DEVICE_CONTROL_REG write fail\n");
					} else
						TOUCH_DEBUG_MSG("DEVICE_CONTROL_NORMAL_OP\n");
					if (unlikely(touch_i2c_read(client, INTERRUPT_ENABLE_REG,
									1, &buf) < 0)) {
						TOUCH_ERR_MSG("INTERRUPT_ENABLE_REG read fail\n");
					}
					if (unlikely(touch_i2c_write_byte(client, INTERRUPT_ENABLE_REG,
									buf | INTERRUPT_MASK_ABS0 | INTERRUPT_MASK_BUTTON) < 0)) {
						TOUCH_ERR_MSG("INTERRUPT_ENABLE_REG write fail\n");
					}
					touch_enable_irq(ts->client->irq);
					if (unlikely(touch_i2c_read(client, INTERRUPT_STATUS_REG, 1, &buf) < 0)) {
						TOUCH_ERR_MSG("INTERRUPT_STATUS_REG read fail\n");
					}

				} else {
					if (unlikely(touch_i2c_write_byte(client, DEVICE_CONTROL_REG,
									DEVICE_CONTROL_NORMAL_OP | DEVICE_CONTROL_CONFIGURED) < 0)) {
						TOUCH_ERR_MSG("DEVICE_CONTROL_REG write fail\n");
					} else
						TOUCH_DEBUG_MSG("DEVICE_CONTROL_NORMAL_OP\n");
				}

			}

			switch (ts->lpwg_mode) {
			case LPWG_DOUBLE_TAP:
				DO_SAFE(lpwg_tap_control(ts, 1), error);
				ts->double_tap_enable = 1;
				break;
			case LPWG_MULTI_TAP:
				DO_SAFE(lpwg_tap_control(ts, 1), error);
				ts->multi_tap_enable = 1;
				break;
			default:
				DO_SAFE(lpwg_tap_control(ts, 0), error);
				DO_SAFE(touch_i2c_read(client, DEVICE_CONTROL_REG, 1, &buf), error);
				buf = (buf & 0xFC) | DEVICE_CONTROL_SLEEP;
				DO_SAFE(touch_i2c_write_byte(client, DEVICE_CONTROL_REG, buf), error);
				TOUCH_DEBUG_MSG("DEVICE_CONTROL_SLEEP");
				break;
			}
		} else {
			TOUCH_DEBUG_MSG("is_suspend : NULL ");
		}
		break;
	case LPWG_LCD_X:
		ts->pdata->caps->lcd_touch_ratio_x = ts->pdata->caps->x_max / value;
		TOUCH_INFO_MSG("LPWG GET X LCD INFO = %d , RATIO_X = %d", value, ts->pdata->caps->lcd_touch_ratio_x);
		break;
	case LPWG_LCD_Y:
		/* If touch-resolution is not same with LCD-resolution,
			position-data should be converted to LCD-resolution.*/
		ts->pdata->caps->lcd_touch_ratio_y = ts->pdata->caps->y_max / value;
		TOUCH_INFO_MSG("LPWG GET Y LCD INFO = %d , RATIO_Y = %d", value, ts->pdata->caps->lcd_touch_ratio_y);
		break;
	case LPWG_ACTIVE_AREA_X1:
	case LPWG_ACTIVE_AREA_X2:
	case LPWG_ACTIVE_AREA_Y1:
	case LPWG_ACTIVE_AREA_Y2:
		/* Quick Cover Area */
		break;
	case LPWG_TAP_COUNT:
		/* Tap Count Control */
		if (value) {
			DO_SAFE(synaptics_ts_page_data_read(client, LPWG_CTRL_PAGE, MULTITAP_COUNT_REG, 1, &buf), error);
			/* TOUCH_DEBUG_MSG("TAP COUNT %d \n", value);	TAP COUNT */
			TOUCH_DEBUG_MSG("Multi TAP COUNT\n");
			buf = (buf & 0x07) | (value << 3);
			TOUCH_DEBUG_MSG("MultiTap LPWG Control Reg value 0x%02X \n", buf);
			DO_SAFE(synaptics_ts_page_data_write_byte(client, LPWG_CTRL_PAGE, MULTITAP_COUNT_REG, buf), error);
			DO_SAFE(touch_i2c_write_byte(client, PAGE_SELECT_REG, DEFAULT_PAGE), error);
		}
		ts->multi_tap_count = value;
		break;
	case LPWG_REPLY:
		if (value == 0)
			multi_tap_fail_try++;
		else
			multi_tap_fail_try = 0;
		/* Do something, if you need. */
		DO_SAFE(touch_i2c_read(client, DEVICE_CONTROL_REG, 1, &buf), error);
		buf = (buf & 0xFC) | DEVICE_CONTROL_NORMAL_OP;
		DO_SAFE(touch_i2c_write_byte(client, DEVICE_CONTROL_REG, buf), error);
		break;
	case LPWG_STATUS_BY_PROXI:
		if (atomic_read(&ts->is_suspend)) {
			TOUCH_DEBUG_MSG("proxi sensor state value : %d lpwg_mode : %d", value, ts->lpwg_mode);
			if (value) {
				TOUCH_DEBUG_MSG("LPWG ENABLE by Proxi ==> near to FAR \n");
			} else {
				TOUCH_DEBUG_MSG("LPWG ENABLE by Proxi ==> far to NEAR \n");
			}
		}
		break;
	case LPWG_MODE_CHANGE:
		if (atomic_read(&ts->is_suspend)) {
			atomic_set(&ts->is_suspend, 0);
		}
		if (ts->double_tap_enable && !ts->lpwg_mode) {
				ts->lpwg_mode = LPWG_DOUBLE_TAP;
				TOUCH_DEBUG_MSG("lpwg_mode REVERT by Proxi : %d ", ts->lpwg_mode);
		} else if (ts->multi_tap_enable && !ts->lpwg_mode) {
				ts->lpwg_mode = LPWG_MULTI_TAP;
				TOUCH_DEBUG_MSG("lpwg_mode REVERT by Proxi : %d ", ts->lpwg_mode);
		} else if (!ts->double_tap_enable && !ts->multi_tap_enable && !ts->lpwg_mode) {
			ts->lpwg_mode = LPWG_NONE;
		} else {
		}
		break;
	default:
		break;
	}

	return NO_ERROR;
error:
	TOUCH_ERR_MSG("i2c fail\n");
	return ERROR;
}

struct touch_device_driver synaptics_ts_driver = {
	.probe 	= synaptics_ts_probe,
	.remove	= synaptics_ts_remove,
	.init  	= synaptics_ts_init,
	.data  	= synaptics_ts_get_data,
	.power 	= synaptics_ts_power,
	.fw_upgrade = synaptics_ts_fw_upgrade,
	.ic_ctrl	= synaptics_ts_ic_ctrl,
	.sysfs = synaptics_ts_sysfs,
	.suspend    = synaptics_ts_suspend,
	.resume     = synaptics_ts_resume,
	.lpwg       = synaptics_ts_lpwg,

};

static int __devinit touch_init(void)
{
	if (touch_debug_mask & DEBUG_TRACE)
		TOUCH_DEBUG_MSG("\n");

	return touch_driver_register(&synaptics_ts_driver);
}

static void __exit touch_exit(void)
{
	if (touch_debug_mask & DEBUG_TRACE)
		TOUCH_DEBUG_MSG("\n");
	if (touch_multi_tap_wq)
		destroy_workqueue(touch_multi_tap_wq);
	touch_driver_unregister();
}

module_init(touch_init);
module_exit(touch_exit);

MODULE_AUTHOR("yehan.ahn@lge.com, hyesung.shin@lge.com");
MODULE_DESCRIPTION("LGE Touch Driver");
MODULE_LICENSE("GPL");

