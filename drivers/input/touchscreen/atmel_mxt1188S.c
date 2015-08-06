/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Copyright (C) 2011-2012 Atmel Corporation
 * Copyright (C) 2012 Google, Inc.
 *
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; youf can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <asm/unaligned.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/input/atmel_mxt1188S.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#if 0//def CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/time.h>

#include <linux/wakelock.h>
#include <linux/mutex.h>
#include <linux/mfd/pm8xxx/cradle.h>
#include <linux/sysdev.h>
#include <mach/board_lge.h>

#define DEBUG_ABS	1

/* Configuration file */
#define MXT_CFG_MAGIC		"OBP_RAW V1"

/* Registers */
#define MXT_OBJECT_START	0x07
#define MXT_OBJECT_SIZE		6
#define MXT_INFO_CHECKSUM_SIZE	3
#define MXT_MAX_BLOCK_WRITE	256

/* MXT_GEN_MESSAGE_T5 object */
#define MXT_RPTID_NOMSG		0xff

/* MXT_GEN_COMMAND_T6 field */
#define MXT_COMMAND_RESET	0
#define MXT_COMMAND_BACKUPNV	1
#define MXT_COMMAND_CALIBRATE	2
#define MXT_COMMAND_REPORTALL	3
#define MXT_COMMAND_DIAGNOSTIC	5

/* Define for T6 status byte */
#define MXT_T6_STATUS_RESET	(1 << 7)
#define MXT_T6_STATUS_OFL	(1 << 6)
#define MXT_T6_STATUS_SIGERR	(1 << 5)
#define MXT_T6_STATUS_CAL	(1 << 4)
#define MXT_T6_STATUS_CFGERR	(1 << 3)
#define MXT_T6_STATUS_COMSERR	(1 << 2)

/* MXT_GEN_POWER_T7 field */
struct t7_config {
	u8 idle;
	u8 active;
} __packed;

#define MXT_POWER_CFG_RUN		0
#define MXT_POWER_CFG_DEEPSLEEP		1

/* MXT_TOUCH_MULTI_T9 field */
#define MXT_T9_ORIENT		9
#define MXT_T9_RANGE		18

/* MXT_TOUCH_MULTI_T9 status */
#define MXT_T9_UNGRIP		(1 << 0)
#define MXT_T9_SUPPRESS		(1 << 1)
#define MXT_T9_AMP		(1 << 2)
#define MXT_T9_VECTOR		(1 << 3)
#define MXT_T9_MOVE		(1 << 4)
#define MXT_T9_RELEASE		(1 << 5)
#define MXT_T9_PRESS		(1 << 6)
#define MXT_T9_DETECT		(1 << 7)

struct t9_range {
	u16 x;
	u16 y;
} __packed;

/* MXT_TOUCH_MULTI_T9 orient */
#define MXT_T9_ORIENT_SWITCH	(1 << 0)

/* MXT_SPT_COMMSCONFIG_T18 */
#define MXT_COMMS_CTRL		0
#define MXT_COMMS_CMD		1
#define MXT_COMMS_RETRIGEN      (1 << 6)

/* Define for MXT_GEN_COMMAND_T6 */
#define MXT_BOOT_VALUE		0xa5
#define MXT_RESET_VALUE		0x01
#define MXT_BACKUP_VALUE	0x55
#define MXT_STOP_DYNAMIC_CONFIG	0x33

/* Define for MXT_PROCI_TOUCHSUPPRESSION_T42 */
#define MXT_T42_MSG_TCHSUP	(1 << 0)

/* T47 Stylus */
#define MXT_TOUCH_MAJOR_T47_STYLUS	1

/* T61 Timer */
#define MXT_T61_TIMER_CMD_START	1
#define MXT_T61_TIMER_CMD_STOP	2

/* T100 Multiple Touch Touchscreen */
#define MXT_T100_CTRL		0
#define MXT_T100_CFG1		1
#define MXT_T100_TCHAUX		3
#define MXT_T100_XRANGE		13
#define MXT_T100_YRANGE		24

#define MXT_T100_CFG_SWITCHXY	(1 << 5)

#define MXT_T100_TCHAUX_VECT	(1 << 0)
#define MXT_T100_TCHAUX_AMPL	(1 << 1)
#define MXT_T100_TCHAUX_AREA	(1 << 2)

#define MXT_T100_DETECT		(1 << 7)
#define MXT_T100_TYPE_MASK	0x70
#define MXT_T100_TYPE_STYLUS	0x20

/* Delay times */
#define MXT_BACKUP_TIME		50	/* msec */
#define MXT_RESET_TIME		200	/* msec */
#define MXT_RESET_TIMEOUT	3000	/* msec */
#define MXT_CRC_TIMEOUT		1000	/* msec */
#define MXT_FW_RESET_TIME	1000	/* msec */
#define MXT_FW_CHG_TIMEOUT	300	/* msec */
#define MXT_WAKEUP_TIME		25	/* msec */
#define MXT_REGULATOR_DELAY	150	/* msec */
#define MXT_POWERON_DELAY	100	/* msec */
#define MXT_SELFTEST_TIME	3000	/* msec */


/* Command to unlock bootloader */
#define MXT_UNLOCK_CMD_MSB	0xaa
#define MXT_UNLOCK_CMD_LSB	0xdc

/* Bootloader mode status */
#define MXT_WAITING_BOOTLOAD_CMD	0xc0	/* valid 7 6 bit only */
#define MXT_WAITING_FRAME_DATA	0x80	/* valid 7 6 bit only */
#define MXT_FRAME_CRC_CHECK	0x02
#define MXT_FRAME_CRC_FAIL	0x03
#define MXT_FRAME_CRC_PASS	0x04
#define MXT_APP_CRC_FAIL	0x40	/* valid 7 8 bit only */
#define MXT_BOOT_STATUS_MASK	0x3f
#define MXT_BOOT_EXTENDED_ID	(1 << 5)
#define MXT_BOOT_ID_MASK	0x1f

/* Touchscreen absolute values */
#define MXT_MAX_AREA		0xff

#define MXT_PIXELS_PER_MM	20

#define CHANGE_CHARGER_CFG
#define CHANGE_PEN_CFG
#define FIRMUP_ON_PROBE
#ifdef FIRMUP_ON_PROBE
#define MXT_LATEST_CONFIG_CRC	0x331A2D
u8 latest_firmware[] = {
#ifdef USE_FW_11AA
	#include "mXT1188SC09_v1.1.AA_extid_29_.h"
#else //USE_FW_11AA
	#include "mXT1188S__APP_V2-0-AB_extid_29_.h"
#endif //USE_FW_11AA
};
#endif //FIRMUP_ON_PROBE

#ifdef CUST_A_TOUCH
#define jitter_abs(x)		(x > 0 ? x : -x)
#define jitter_sub(x, y)	(x > y ? x - y : y - x)
#define get_interval(a,b) a>=b ? a-b : 1000000+a-b

// LGE_CHANGE_S [naomi.kim@lge.com] 13.06.17, set data format
#define TOUCHEVENTFILTER	1
// LGE_CHANGE_E [naomi.kim@lge.com] 13.06.17, set data format
#endif

static bool must_calibration;
static bool is_probing;

#if 0//def CONFIG_HAS_EARLYSUSPEND
/* Early-suspend level */
#define MXT_SUSPEND_LEVEL 1

static void mxt_early_suspend(struct early_suspend *h);
static void mxt_late_resume(struct early_suspend *h);
#endif

struct mxt_info {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 matrix_xsize;
	u8 matrix_ysize;
	u8 object_num;
};

struct mxt_object {
	u8 type;
	u16 start_address;
	u8 size_minus_one;
	u8 instances_minus_one;
	u8 num_report_ids;
} __packed;

#ifdef CUST_A_TOUCH
struct t_data {
	u16	id;
	u8	status;
	u16	x_position;
	u16	y_position;
	u8	touch_major;
	// LGE_CHANGE_S [naomi.kim@lge.com] 13.06.17, set data format
	#if TOUCHEVENTFILTER
	u8	touch_minor;
	#endif //TOUCHEVENTFILTER
	// LGE_CHANGE_E [naomi.kim@lge.com] 13.06.17, set data format
	u8	pressure;
	u8	orientation;
	int tool;
	bool is_pen;
	bool skip_report;
};

struct touch_data {
	u8 total_num;
	u8 prev_total_num;
	u8 state;
	u8 palm;
	struct t_data curr_data[MXT_MAX_NUM_TOUCHES];
	struct t_data prev_data[MXT_MAX_NUM_TOUCHES];
};

struct accuracy_history_data {
	u16 x;
	u16 y;
	u16 pressure;
	int count;
	int mod_x;
	int mod_y;
	int axis_x;
	int axis_y;
	int prev_total_num;
};

struct accuracy_filter_info {
	int	ignore_pressure_gap;
	int	touch_max_count;
	int	delta_max;
	int	max_pressure;
	int	direction_count;
	int	time_to_max_pressure;
	u16	finish_filter;
	struct accuracy_history_data	his_data;
};

enum{
	NO_PROBLEM,
	NEED_TO_OUT,
	NEED_TO_INIT,
};

enum{
	TIME_EX_FIRST_INT_TIME,
	TIME_EX_PREV_PRESS_TIME,
	TIME_EX_CURR_PRESS_TIME,
	TIME_EX_FIRST_GHOST_DETECT_TIME,
	TIME_EX_SECOND_GHOST_DETECT_TIME,
	TIME_EX_CURR_INT_TIME,
	TIME_EX_PROFILE_MAX
};

static unsigned int rebase_count =0;
struct timeval time_ex_debug[TIME_EX_PROFILE_MAX];
struct t_data	 prev_finger_press_data;
int finger_subtraction_check_cnt = 0;
bool ghost_detected = 0;
int ghost_detection_cnt = 0;
//bool thermal_status = 0;
//extern int touch_thermal_mode;
#endif

/* Each client has this additional data */
struct mxt_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	char phys[64];		/* device physical location */
	struct mxt_platform_data *pdata;
	struct mxt_object *object_table;
	struct mxt_info *info;
	void *raw_info_block;
	unsigned int irq;
	unsigned int max_x;
	unsigned int max_y;
	bool in_bootloader;
	u16 mem_size;
	u8 t100_aux_ampl;
	u8 t100_aux_area;
	u8 t100_aux_vect;
	struct bin_attribute mem_access_attr;
	bool debug_enabled;
	u8 max_reportid;
	u32 config_crc;
	u32 info_crc;
	u8 bootloader_addr;
	struct t7_config t7_cfg;
	u8 *msg_buf;
	u8 t6_status;
	bool update_input;
	u8 last_message_count;
	u8 num_touchids;
	unsigned long t15_keystatus;
	bool use_retrigen_workaround;
	bool use_regulator;
	struct regulator *vdd_ana;
	struct regulator *vcc_i2c;
	struct regulator *vcc_dig;
	char *fw_name;
	char *cfg_name;
	/*Anti-touch Algorithm*/
	u8 antitouch_timer0_started;
	u8 antitouch_timer1_started;
	u8 antitouch_timer2_started;
	u8 antitouch_timer3_started;
	u8 object_num;
	u8 check_antitouch;
	u8 autocal_step;
	bool autocal_finished;
	u8 case2_1_detection_count;
	u8 case2_2_detection_count;
	u8 case3_1_detection_count;
	u8 case3_2_detection_count;
	u8 case3_3_detection_count;
	u8 case3_4_detection_count;
	u8 case3_5_detection_count;
	u8 case3_6_detection_count;
	u8 case3_7_detection_count;
	u8 case3_8_detection_count;
	u8 case3_9_detection_count;
	u8 case3_10_detection_count;
	u8 case3_11_detection_count;
	u8 case3_12_detection_count;
	bool stop_tracking_stylus;
	bool palm_pressed;
	bool check_multi_stylus;
	u16 ref_stylus_x;
	u16 ref_stylus_y;

	/* Cached parameters from object table */
	u16 T5_address;
	u8 T5_msg_size;
	u8 T6_reportid;
	u16 T6_address;
	u16 T7_address;
	u8 T9_reportid_min;
	u8 T9_reportid_max;
	u8 T15_reportid_min;
	u8 T15_reportid_max;
	u16 T18_address;
	u8 T19_reportid;
	u8 T42_reportid_min;
	u8 T42_reportid_max;
	u16 T44_address;
	u8 T48_reportid;
	u8 T57_reportid_min;
	u8 T57_reportid_max;
	u8 T61_reportid_min;
	u8 T61_reportid_max;
	u8 T100_reportid_min;
	u8 T100_reportid_max;
	u8 T24_reportid;
	u8 T25_reportid;
	/* for fw update in bootloader */
	struct completion bl_completion;

	/* for reset handling */
	struct completion reset_completion;

	/* for reset handling */
	struct completion crc_completion;

	/* Auto touch test */
	struct completion t25_completion;

	/* Enable reporting of input events */
	bool enable_reporting;

	/* Indicates whether device is in suspend */
	bool suspended;

#ifdef CUST_A_TOUCH
	struct accuracy_filter_info	accuracy_filter;
	struct touch_data ts_data;
#endif
#if 0//def CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	u8 charger_state;
#ifdef CHANGE_PEN_CFG
    bool stylus_pen_pressed;
    bool stylus_pen_cfg_adapted;
    u8 pen_pressed_ids;
#endif
	bool knock_on_enable;
	bool smart_cover_enable;
	bool self_test_result;
	bool chargerlogo;

	struct kobject lge_touch_kobj;
};

struct lge_touch_attribute {
	struct attribute	attr;
	ssize_t (*show)(struct mxt_data *data, char *buf);
	ssize_t (*store)(struct mxt_data *data, const char *buf, size_t count);
};

#define LGE_TOUCH_ATTR(_name, _mode, _show, _store)	\
struct lge_touch_attribute lge_touch_attr_##_name = __ATTR(_name, _mode, _show, _store)

#ifdef CHANGE_CHARGER_CFG
enum
{
    CHARGER_STATE_DISCONNECTED,
    CHARGER_STATE_CONNECTED,
    CHARGER_STATE_UPDATE_CAHRGER_CFG,
    CHARGER_STATE_RESTORE_CFG
};
#endif

#ifdef CHANGE_PEN_CFG
#define LGE_CFG_PEN_TO_TOUCH_CHANGE_THRESHOLD	0
#define LGE_CFG_TOUCH_TO_PEN_CHANGE_THRESHOLD	0
#define HIGH_SPEED_PEN_THRESHOLD 80
#define LOW_SPEED_PEN_THRESHOLD	2
#define PEN_SPEED_CHANGE_THRESHOLD	7
#endif

#if defined(CHANGE_CHARGER_CFG) || defined(CHANGE_PEN_CFG)
enum {
	LGE_CFG_TYPE_FINGER,
	LGE_CFG_TYPE_CHARGER,
	LGE_CFG_TYPE_UNCHARGER,
	LGE_CFG_TYPE_PEN,
	LGE_CFG_TYPE_LOW_SPEED_PEN,
	LGE_CFG_TYPE_HIGH_SPEED_PEN,
	LGE_CFG_TYPE_UNPEN,
	LGE_CFG_TYPE_SUS,
	LGE_CFG_TYPE_ACT,
	LGE_CFG_TYPE_SUS_CHARGER
};
#endif

struct mxt_data *test_data = NULL;

//#define CUST_G2_TOUCH_WAKEUP_GESTURE
#if 0//def CONFIG_TOUCHSCREEN_ATMEL_KNOCK_ON
static struct mutex i2c_suspend_lock;
static bool suspended_due_to_smart_cover = false;
static bool touch_irq_wake = 0;

static int touch_enable_irq_wake(unsigned int irq){
	int ret = 0;
	if(!touch_irq_wake){
		touch_irq_wake = 1;
		ret= enable_irq_wake(irq);
	}
	return ret;
}
static int touch_disable_irq_wake(unsigned int irq){
	int ret = 0;
	if(touch_irq_wake){
		touch_irq_wake = 0;
		ret = disable_irq_wake(irq);
	}
	return ret;
}
#endif
static bool touch_enable = 1;
static void touch_enable_irq(unsigned int irq){
	if(!touch_enable){
		touch_enable = 1;
		enable_irq(irq);
	}
}
static void touch_disable_irq(unsigned int irq){
	if(touch_enable){
		touch_enable = 0;
		disable_irq(irq);
	}
}

static void mxt_start(struct mxt_data *data);
static void mxt_stop(struct mxt_data *data);

static void mxt_reset_slots(struct mxt_data *data);
static int mxt_t6_command(struct mxt_data *data, u16 cmd_offset,
			  u8 value, bool wait);

static inline size_t mxt_obj_size(const struct mxt_object *obj)
{
	return obj->size_minus_one + 1;
}

static inline size_t mxt_obj_instances(const struct mxt_object *obj)
{
	return obj->instances_minus_one + 1;
}

static bool mxt_object_readable(unsigned int type)
{
	switch (type) {
	case MXT_GEN_COMMAND_T6:
	case MXT_GEN_POWER_T7:
	case MXT_GEN_ACQUIRE_T8:
	case MXT_GEN_DATASOURCE_T53:
	case MXT_TOUCH_MULTI_T9:
	case MXT_TOUCH_KEYARRAY_T15:
	case MXT_TOUCH_PROXIMITY_T23:
	case MXT_TOUCH_PROXKEY_T52:
	case MXT_PROCI_GRIPFACE_T20:
	case MXT_PROCG_NOISE_T22:
	case MXT_PROCI_ONETOUCH_T24:
	case MXT_PROCI_TWOTOUCH_T27:
	case MXT_PROCI_GRIP_T40:
	case MXT_PROCI_TOUCHSUPPRESSION_T42:
	case MXT_PROCI_STYLUS_T47:
	case MXT_SPT_NOISESUPPRESSION_T48:
	case MXT_SPT_COMMSCONFIG_T18:
	case MXT_SPT_GPIOPWM_T19:
	case MXT_SPT_SELFTEST_T25:
	case MXT_SPT_CTECONFIG_T28:
	case MXT_SPT_USERDATA_T38:
	case MXT_SPT_DIGITIZER_T43:
	case MXT_SPT_CTECONFIG_T46:
	case MXT_SPT_TIMER_T61:
	case MXT_PROCG_NOISESUPPRESSION_T62:
	case MXT_PROCI_LENSBENDING_T65:
	case MXT_SPT_DYNAMICCONFIGURATIONCONTROLLER_T70:
	case MXT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71:
	case MXT_SPT_CTESCANCONFIG_T77:
		return true;
	default:
		return false;
	}
}

static void mxt_dump_message(struct mxt_data *data, u8 *message)
{
	print_hex_dump(KERN_DEBUG, "MXT MSG:", DUMP_PREFIX_NONE, 16, 1,
		       message, data->T5_msg_size, false);
}

static int mxt_wait_for_completion(struct mxt_data *data,
			struct completion *comp, unsigned int timeout_ms)
{
	struct device *dev = &data->client->dev;
	unsigned long timeout = msecs_to_jiffies(timeout_ms);
	long ret;

	ret = wait_for_completion_interruptible_timeout(comp, timeout);
	if (ret < 0) {
		dev_err(dev, "Wait for completion interrupted.\n");
		return -EINTR;
	} else if (ret == 0) {
		dev_err(dev, "Wait for completion timed out.\n");
		return -ETIMEDOUT;
	}
	return 0;
}

static int mxt_bootloader_read(struct mxt_data *data,
			       u8 *val, unsigned int count)
{
	int ret;
	struct i2c_msg msg;

	msg.addr = data->bootloader_addr;
	msg.flags = data->client->flags & I2C_M_TEN;
	msg.flags |= I2C_M_RD;
	msg.len = count;
	msg.buf = val;

	ret = i2c_transfer(data->client->adapter, &msg, 1);

	if (ret == 1) {
		ret = 0;
	} else {
		ret = (ret < 0) ? ret : -EIO;
		dev_err(&data->client->dev, "i2c recv failed (%d)\n", ret);
	}

	return ret;
}

static int mxt_bootloader_write(struct mxt_data *data,
				const u8 * const val, unsigned int count)
{
	int ret;
	struct i2c_msg msg;

	msg.addr = data->bootloader_addr;
	msg.flags = data->client->flags & I2C_M_TEN;
	msg.len = count;
	msg.buf = (u8 *)val;

	ret = i2c_transfer(data->client->adapter, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		ret = (ret < 0) ? ret : -EIO;
		dev_err(&data->client->dev, "i2c send failed (%d)\n", ret);
	}

	return ret;
}

static int mxt_lookup_bootloader_address(struct mxt_data *data, u8 retry)
{
	u8 appmode = data->client->addr;
	u8 bootloader;
	u8 family_id = 0;

	if (data->info)
		family_id = data->info->family_id;

	switch (appmode) {
	case 0x4a:
	case 0x4b:
		/* 1188S/1664S use different scheme */
		if ((retry % 2) || family_id == 0xa2) {
			bootloader = appmode - 0x24;
			break;
		}
		/* Fall through for normal case */
	case 0x4c:
	case 0x4d:
	case 0x5a:
	case 0x5b:
		bootloader = appmode - 0x26;
		break;
	default:
		dev_err(&data->client->dev,
			"Appmode i2c address 0x%02x not found\n",
			appmode);
		return -EINVAL;
	}

	data->bootloader_addr = bootloader;
	return 0;
}

static int mxt_probe_bootloader(struct mxt_data *data, u8 retry)
{
	struct device *dev = &data->client->dev;
	int ret;
	u8 val;
	bool crc_failure;
	dev_info(dev, "mxt probe bootloader\n");
	ret = mxt_lookup_bootloader_address(data, retry);
	if (ret)
		return ret;

	ret = mxt_bootloader_read(data, &val, 1);
	if (ret)
		return ret;

	/* Check app crc fail mode */
	crc_failure = (val & ~MXT_BOOT_STATUS_MASK) == MXT_APP_CRC_FAIL;

	dev_err(dev, "Detected bootloader, status:%02X%s\n",
			val, crc_failure ? ", APP_CRC_FAIL" : "");

	return 0;
}

static u8 mxt_get_bootloader_version(struct mxt_data *data, u8 val)
{
	struct device *dev = &data->client->dev;
	u8 buf[3];

	if (val & MXT_BOOT_EXTENDED_ID) {
		if (mxt_bootloader_read(data, &buf[0], 3) != 0) {
			dev_err(dev, "%s: i2c failure\n", __func__);
			return -EIO;
		}

		dev_info(dev, "Bootloader ID:%d Version:%d\n", buf[1], buf[2]);

		return buf[0];
	} else {
		dev_info(dev, "Bootloader ID:%d\n", val & MXT_BOOT_ID_MASK);

		return val;
	}
}

static int mxt_check_bootloader(struct mxt_data *data, unsigned int state)
{
	struct device *dev = &data->client->dev;
	u8 val;
	int ret;

recheck:
	if (state != MXT_WAITING_BOOTLOAD_CMD) {
		/*
		 * In application update mode, the interrupt
		 * line signals state transitions. We must wait for the
		 * CHG assertion before reading the status byte.
		 * Once the status byte has been read, the line is deasserted.
		 */
		ret = mxt_wait_for_completion(data, &data->bl_completion,
					      MXT_FW_CHG_TIMEOUT);
		if (ret) {
			/*
			 * TODO: handle -EINTR better by terminating fw update
			 * process before returning to userspace by writing
			 * length 0x000 to device (iff we are in
			 * WAITING_FRAME_DATA state).
			 */
			dev_err(dev, "Update wait error %d\n", ret);
			return ret;
		}
	}

	ret = mxt_bootloader_read(data, &val, 1);
	if (ret)
		return ret;

	if (state == MXT_WAITING_BOOTLOAD_CMD)
		val = mxt_get_bootloader_version(data, val);

	switch (state) {
	case MXT_WAITING_BOOTLOAD_CMD:
	case MXT_WAITING_FRAME_DATA:
	case MXT_APP_CRC_FAIL:
		val &= ~MXT_BOOT_STATUS_MASK;
		break;
	case MXT_FRAME_CRC_PASS:
		if (val == MXT_FRAME_CRC_CHECK) {
			goto recheck;
		} else if (val == MXT_FRAME_CRC_FAIL) {
			dev_err(dev, "Bootloader CRC fail\n");
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	if (val != state) {
		dev_err(dev, "Invalid bootloader state %02X != %02X\n",
			val, state);
		return -EINVAL;
	}

	return 0;
}

static int mxt_send_bootloader_cmd(struct mxt_data *data, bool unlock)
{
	int ret;
	u8 buf[2];

	if (unlock) {
		buf[0] = MXT_UNLOCK_CMD_LSB;
		buf[1] = MXT_UNLOCK_CMD_MSB;
	} else {
		buf[0] = 0x01;
		buf[1] = 0x01;
	}

	ret = mxt_bootloader_write(data, buf, 2);
	if (ret)
		return ret;

	return 0;
}

static int __mxt_read_reg(struct i2c_client *client,
			       u16 reg, u16 len, void *val)
{
	struct i2c_msg xfer[2];
	u8 buf[2];
	int i=0;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	/* Write register */
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 2;
	xfer[0].buf = buf;

	/* Read data */
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = len;
	xfer[1].buf = val;

	do {
		if (i2c_transfer(client->adapter, xfer, ARRAY_SIZE(xfer))==2)
			return 0;
		dev_info(&client->dev, "%s: i2c retry %d\n", __func__, i+1);
		msleep(MXT_WAKEUP_TIME);
	} while (++i < 10);

	dev_err(&client->dev, "%s: i2c transfer failed\n", __func__);
	return -EIO;
}

static int __mxt_write_reg(struct i2c_client *client, u16 reg, u16 len,
			   const void *val)
{
	u8 *buf;
	size_t count;
	int i = 0;

	count = len + 2;
	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	memcpy(&buf[2], val, len);

	do {
		if (i2c_master_send(client, buf, count)==count){
			kfree(buf);
			return 0;
		}
		dev_info(&client->dev, "%s: i2c retry %d\n", __func__, i+1);
		msleep(MXT_WAKEUP_TIME);
	} while (++i < 10);

	dev_err(&client->dev, "%s: i2c transfer failed\n", __func__);
	kfree(buf);
	return -EIO;
}

static int mxt_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	return __mxt_write_reg(client, reg, 1, &val);
}

static struct mxt_object *
mxt_get_object(struct mxt_data *data, u8 type)
{
	struct mxt_object *object;
	int i;

	for (i = 0; i < data->info->object_num; i++) {
		object = data->object_table + i;
		if (object->type == type)
			return object;
	}

	dev_err(&data->client->dev, "Invalid object type T%u\n", type);
	return NULL;
}

#if 0//defined(CHANGE_CHARGER_CFG) || defined(CHANGE_PEN_CFG)
static int write_config(struct mxt_data *data, const u8** configs) {

	struct mxt_object *object;
	int i, j;
	int ret = 0;

	for (i = 0; configs[i][0] != MXT_TMAX; i++) {
		object = mxt_get_object(data, configs[i][0]);
		if (!object) {
			dev_err(&data->client->dev,
					"%s error Cannot get object_type T%d\n", __func__, configs[i][0]);
			return -EINVAL;
		}
		if ((mxt_obj_size(object) == 0) || (object->start_address == 0)) {
			dev_err(&data->client->dev,
					"%s error object_type T%d\n", __func__, object->type);
			return -ENODEV;
		}

		for (j = 1; j <= configs[i][1]; j++) {
			dev_dbg(&data->client->dev,
					"write config data T%d offset(%d) value(%d)\n",
					configs[i][0],
					configs[i][j * 2],
					configs[i][j * 2 + 1]
			);
			ret = mxt_write_reg(data->client, object->start_address+ configs[i][j * 2], configs[i][j * 2 + 1]);

			if (ret) {
				dev_err(&data->client->dev, "Failed to write configuration\n");
				return -EIO;
			}
		}
	}

	return ret;
}

static int change_config(struct mxt_data *data, int cfg_type) {

	int ret = 0;

	switch (cfg_type)
	{
#ifdef CHANGE_CHARGER_CFG
	case LGE_CFG_TYPE_FINGER:
		break;

	case LGE_CFG_TYPE_CHARGER:
		ret= write_config(data, data->pdata->charger_config);
		break;

	case LGE_CFG_TYPE_UNCHARGER:
		ret= write_config(data, data->pdata->restore_config);
		break;
#endif
#ifdef CHANGE_PEN_CFG
	case LGE_CFG_TYPE_PEN:
		ret= write_config(data, data->pdata->pen_config);
		break;

	case LGE_CFG_TYPE_LOW_SPEED_PEN:
		ret= write_config(data, data->pdata->pen_low_speed_config);
		break;

	case LGE_CFG_TYPE_HIGH_SPEED_PEN:
		ret= write_config(data, data->pdata->pen_high_speed_config);
		break;

	case LGE_CFG_TYPE_UNPEN:
		ret= write_config(data, data->pdata->unpen_config);
		break;
#endif
#ifdef CONFIG_TOUCHSCREEN_ATMEL_KNOCK_ON
	case LGE_CFG_TYPE_SUS:
		ret= write_config(data, data->pdata->sus_config);
		break;
	case LGE_CFG_TYPE_ACT:
		ret= write_config(data, data->pdata->act_config);
		break;
	case LGE_CFG_TYPE_SUS_CHARGER:
		ret= write_config(data, data->pdata->sus_charger_config);
		break;
#endif
	default :

		break;

	}

	return ret;

}

#endif

#ifdef FIRMUP_ON_PROBE
static int mxt_init_write_config(struct mxt_data *data,
		u8 type, const u8 *cfg)
{
	struct mxt_object *object;
//	u8 *temp;
	int ret;
//	int obj_num = 1;

	object = mxt_get_object(data, type);
	if (!object) {
		dev_err(&data->client->dev,
			"%s error Cannot get object_type T%d\n", __func__, type);
		return -EINVAL;
    }

	if ((mxt_obj_size(object) == 0) || (object->start_address == 0)) {
		dev_err(&data->client->dev,
			"%s error object_type T%d\n", __func__, type);
		return -ENODEV;
	}

	dev_info(&data->client->dev, "Write config T%d: start_addr=%d, size=%d * instance=%d\n",
		type, object->start_address, mxt_obj_size(object), mxt_obj_instances(object));

/*
	if (type == MXT_SPT_DYNAMICCONFIGURATIONCONTROLLER_T70)
		obj_num = 12;
	else if ( type == MXT_SPT_TIMER_T61)
		obj_num = 4;
*/
//	ret = __mxt_write_reg(data->client, object->start_address, (mxt_obj_size(object) * obj_num), cfg);
	ret = __mxt_write_reg(data->client, object->start_address,
				mxt_obj_size(object)*mxt_obj_instances(object), cfg);

	if (ret) {
		dev_err(&data->client->dev,
			"%s write error T%d address[0x%x]\n",
			__func__, type, object->start_address);
		return ret;
	}
/*
	if (mxt_obj_instances(object)) {
		temp = kzalloc(mxt_obj_size(object), GFP_KERNEL);

		if (temp == NULL)
			return -ENOMEM;

		ret |= __mxt_write_reg(data->client, object->start_address + mxt_obj_size(object),
			mxt_obj_size(object), temp);
		kfree(temp);
	}
*/
	return ret;
}

static int mxt_write_configuration(struct mxt_data *data)
{
	int i = 0;
	int ret = 0;
	u8 ** tsp_config = (u8 **)data->pdata->config_array->config_t;

	dev_info(&data->client->dev, "Write configuration data\n");

	for (i = 0; i < MXT_TMAX; i++) {
		ret = mxt_init_write_config(data, tsp_config[i][0],
							tsp_config[i] + 1);
		if (ret) {
			dev_err(&data->client->dev, "Failed to write configuration\n");
			goto out;
		}
	}

out:
	return ret;
}
#endif

#if 0
void trigger_usb_state_from_otg(int usb_type)
{
#if 0//def CHANGE_CHARGER_CFG
	if (test_data != NULL) {
		if (!test_data->suspended) {
			if (usb_type) {
				dev_info(&test_data->client->dev, "%s charger connected. Change configuration to charger mode\n", __func__);
				if (change_config(test_data, LGE_CFG_TYPE_CHARGER)) {
					dev_err(&test_data->client->dev, "%s Failed to change charger config\n", __func__);
				}
				test_data->charger_state = CHARGER_STATE_CONNECTED;
			} else {
				dev_info(&test_data->client->dev, "%s charger disconnected. Change configuration to normal mode\n", __func__);
				if (change_config(test_data, LGE_CFG_TYPE_UNCHARGER)) {
					dev_err(&test_data->client->dev, "%s Failed to change normal config\n", __func__);
				}
				test_data->charger_state = CHARGER_STATE_DISCONNECTED;
			}

			mxt_t6_command(test_data, MXT_COMMAND_CALIBRATE, 1, false);
		} else {
			//If device is in suspend mode, deal with config data in the late_resume function.
			if (usb_type) {
				dev_info(&test_data->client->dev, "%s charger connected. Need to change configuration to charger mode\n", __func__);
				test_data->charger_state = CHARGER_STATE_UPDATE_CAHRGER_CFG;
			} else {
				dev_info(&test_data->client->dev, "%s charger connected. Need to change configuration to normal mode\n", __func__);
				test_data->charger_state = CHARGER_STATE_RESTORE_CFG;
			}
		}
	}
#endif
}
#endif

#if 0//def CUST_A_TOUCH
void check_touch_bat_therm(int therm_mode){
	if (test_data != NULL) {
		dev_info(&test_data->client->dev, "[%s] thermal status = (%d)\n", __func__, therm_mode);
		mxt_t6_command(test_data, MXT_COMMAND_CALIBRATE, 1, false);
		thermal_status = therm_mode;
	}
}
#endif

static int mxt_t61_timer_set(struct mxt_data *data, u8 instance, u8 cmd) {
	struct mxt_object *object;
	u8 buf[5] = {0, };
	int ret = 0;

	if (cmd == MXT_T61_TIMER_CMD_START) {
		//Obtain config data from T38 object
		object = mxt_get_object(data, MXT_SPT_USERDATA_T38);
		if (!object)
			return -EINVAL;

		if (instance == 1) {
			ret = __mxt_read_reg(data->client, object->start_address + 17, 5, buf);
			if (ret)
				goto out;
			data->antitouch_timer1_started =1;
		} else if (instance == 2) {
			buf[0] = 0x03;
			buf[1] = 0x01;
			buf[2] = 0x00;
			buf[3] = 0xB8;
			buf[4] = 0x0B;
			data->antitouch_timer2_started =1;
		} else if (instance == 3) {
			buf[0] = 0x03;
			buf[1] = 0x01;
			buf[2] = 0x00;
			buf[3] = 0xD0;
			buf[4] = 0x07;
			data->antitouch_timer3_started =1;
		} else {
			ret = __mxt_read_reg(data->client, object->start_address + 12, 5, buf);
			if (ret)
				goto out;
			data->antitouch_timer0_started =1;
		}
	} else if (cmd == MXT_T61_TIMER_CMD_STOP) {
		buf[0] = 3;
		buf[1] = cmd;

		if (instance == 1) {
			data->antitouch_timer1_started = 0;
		} else if (instance == 2) {
			data->antitouch_timer2_started = 0;
		} else if (instance == 3) {
			data->antitouch_timer3_started = 0;
		} else {
			data->antitouch_timer0_started =0;
		}
	} else {
		dev_err(&data->client->dev, "%s Invalid command %d\n", __func__, cmd);
		return -EINVAL;
	}

	object = mxt_get_object(data, MXT_SPT_TIMER_T61);
	if (!object)
		return -EINVAL;

	if (instance == 1) {
		ret = __mxt_write_reg(data->client, object->start_address + mxt_obj_size(object), 5, buf);
		if (ret)
			goto out;
	} else if (instance ==2) {
		ret = __mxt_write_reg(data->client, object->start_address + (mxt_obj_size(object) * 2), 5, buf);
		if (ret)
			goto out;
	} else if (instance == 3) {
		ret = __mxt_write_reg(data->client, object->start_address + (mxt_obj_size(object) * 3), 5, buf);
		if (ret)
			goto out;
	} else {
		ret = __mxt_write_reg(data->client, object->start_address, 5, buf);
		if (ret)
			goto out;
	}

	dev_info(&data->client->dev, "T61 timer set instance%d %dms\n", instance, ((buf[4] << 8) | buf[3]));
out:
	return ret;
}

static int mxt_t8_autocal_set(struct mxt_data *data, u8 set_number)
{
	struct mxt_object *object;
	u8 buf[4] = {0, };
	int ret = 0;

	//Obtain config data from T38 object
	object = mxt_get_object(data, MXT_SPT_USERDATA_T38);
	if (!object)
		return -EINVAL;

	switch (set_number) {
	case 1:
		ret = __mxt_read_reg(data->client, object->start_address, 4, buf);
		if (ret)
			return ret;
		break;
	case 2:
		ret = __mxt_read_reg(data->client, object->start_address + 4, 4, buf);
		if (ret)
			return ret;
		break;
	case 3:
		ret = __mxt_read_reg(data->client, object->start_address + 8, 4, buf);
		if (ret)
			return ret;
		break;
	default:
		break;
	}

	object = mxt_get_object(data, MXT_GEN_ACQUIRE_T8);
	if (!object)
		return -EINVAL;

	ret = __mxt_write_reg(data->client, object->start_address + 6, 4, buf);
	if (ret)
		return ret;

	dev_info(&data->client->dev, "T8 autocal set%d (%d %d %d %d)\n", set_number, buf[0], buf[1], buf[2], buf[3]);
	data->autocal_step = set_number;
	return ret;
}

static void mxt_ghost_detection(struct mxt_data *data)
{
	struct mxt_object *object = NULL;

	object = mxt_get_object(data, MXT_PROCG_NOISESUPPRESSION_T62);
	if (object) {
		mxt_write_reg(data->client, object->start_address + 23, 10);
	}

	if (data->antitouch_timer0_started)
		mxt_t61_timer_set(data, 0, MXT_T61_TIMER_CMD_STOP);
	if (data->antitouch_timer1_started)
		mxt_t61_timer_set(data, 1, MXT_T61_TIMER_CMD_STOP);
	if (data->antitouch_timer2_started)
		mxt_t61_timer_set(data, 2, MXT_T61_TIMER_CMD_STOP);
	if (data->antitouch_timer3_started)
		mxt_t61_timer_set(data, 3, MXT_T61_TIMER_CMD_STOP);

	data->case2_1_detection_count = 0;
	data->case2_2_detection_count = 0;
	data->case3_1_detection_count = 0;
	data->case3_2_detection_count = 0;
	data->case3_3_detection_count = 0;
	data->case3_4_detection_count = 0;
	data->case3_5_detection_count = 0;
	data->case3_6_detection_count = 0;
	data->case3_7_detection_count = 0;
	data->case3_8_detection_count = 0;
	data->case3_9_detection_count = 0;
	data->case3_10_detection_count = 0;
	data->case3_11_detection_count = 0;
	data->case3_12_detection_count = 0;
	data->stop_tracking_stylus = false;
	data->check_multi_stylus = false;
	data->autocal_finished = false;

	mxt_t8_autocal_set(data, 1);
	mxt_t61_timer_set(data, 0, MXT_T61_TIMER_CMD_START);
}

static void mxt_proc_t6_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	struct mxt_object *object = NULL;
	u8 status = msg[1];
	u32 crc = msg[2] | (msg[3] << 8) | (msg[4] << 16);
//	int knock_charger = 0;

	if (crc != data->config_crc) {
		data->config_crc = crc;
		dev_info(dev, "T6 Config Checksum: 0x%06X\n", crc);
		complete(&data->crc_completion);
	}

	/* Detect transition out of reset */
	if ((data->t6_status & MXT_T6_STATUS_RESET) &&
	    !(status & MXT_T6_STATUS_RESET))
		complete(&data->reset_completion);

	/* Output debug if status has changed */
	if (status != data->t6_status)
		dev_dbg(dev, "T6 Status 0x%02X%s%s%s%s%s%s%s\n",
			status,
			(status == 0) ? " OK" : "",
			(status & MXT_T6_STATUS_RESET) ? " RESET" : "",
			(status & MXT_T6_STATUS_OFL) ? " OFL" : "",
			(status & MXT_T6_STATUS_SIGERR) ? " SIGERR" : "",
			(status & MXT_T6_STATUS_CAL) ? " CAL" : "",
			(status & MXT_T6_STATUS_CFGERR) ? " CFGERR" : "",
			(status & MXT_T6_STATUS_COMSERR) ? " COMSERR" : "");

	if (status & MXT_T6_STATUS_SIGERR) {
		dev_info(dev, "T6 SIGERR\n");
		object = mxt_get_object(data, MXT_PROCG_NOISESUPPRESSION_T62);
		if (object) {
			mxt_write_reg(data->client, object->start_address, 0);
			msleep(50);
			mxt_write_reg(data->client, object->start_address, 1);
		}
	}

	if (status & MXT_T6_STATUS_CAL && !data->suspended) {
		if (is_probing == false) {
			dev_info(dev, "Calibration Occured\n");
#ifdef CUST_A_TOUCH
			if (data->input_dev)
				mxt_reset_slots(data);
			memset(&data->ts_data, 0, sizeof(data->ts_data));
			memset(&data->accuracy_filter.his_data, 0, sizeof(data->accuracy_filter.his_data));
			data->accuracy_filter.finish_filter = 0;
#endif

			mxt_ghost_detection(data);
		}
	}

#if 0
	if (status & MXT_T6_STATUS_RESET && data->suspended) {
		if (is_probing == false) {
			if (data->knock_on_enable) {
				if(data->charger_state == CHARGER_STATE_CONNECTED)
					knock_charger = LGE_CFG_TYPE_SUS_CHARGER;
				else
					knock_charger = LGE_CFG_TYPE_SUS;

				mutex_lock(&i2c_suspend_lock);
				change_config(data, knock_charger);
				mutex_unlock(&i2c_suspend_lock);
			}
		}
	}
#endif
	/* Save current status */
	data->t6_status = status;
}

static void mxt_input_button(struct mxt_data *data, u8 *message)
{
	struct input_dev *input = data->input_dev;
	const struct mxt_platform_data *pdata = data->pdata;
	bool button;
	int i;

	/* do not report events if input device not yet registered */
	if (!data->enable_reporting)
		return;

	/* Active-low switch */
	for (i = 0; i < pdata->t19_num_keys; i++) {
		if (pdata->t19_keymap[i] == KEY_RESERVED)
			continue;
		button = !(message[1] & (1 << i));
		input_report_key(input, pdata->t19_keymap[i], button);
	}
}

static void mxt_input_sync(struct input_dev *input_dev)
{
	input_mt_report_pointer_emulation(input_dev, false);
	input_sync(input_dev);
}

#ifdef CUST_A_TOUCH
static u16 check_boundary(int x, int max){
	if(x < 0)
		return 0;
	else if(x > max)
		return (u16)max;
	else
		return (u16)x;
}

static int check_direction(int x){
	if(x > 0)
		return 1;
	else if(x < 0)
		return -1;
	else
		return 0;
}

int accuracy_filter_function(struct mxt_data *data)
{
	int delta_x = 0;
	int delta_y = 0;
	u8	id = 0;

	for (id = 0; id < data->pdata->numtouch; id++){
		if (data->ts_data.curr_data[id].status == FINGER_PRESSED) {
			break;
		}
	}

	// finish the accuracy_filter
	if(data->accuracy_filter.finish_filter == 1 &&
	   (data->accuracy_filter.his_data.count > data->accuracy_filter.touch_max_count
	    || data->ts_data.total_num != 1
	    || id != 0)){
		data->accuracy_filter.finish_filter = 0;
		data->accuracy_filter.his_data.count = 0;
	}

	delta_x = (int)data->accuracy_filter.his_data.x - (int)data->ts_data.curr_data[0].x_position;
	delta_y = (int)data->accuracy_filter.his_data.y - (int)data->ts_data.curr_data[0].y_position;

	// main algorithm
	if (data->accuracy_filter.finish_filter){
		if(delta_x || delta_y){
			data->accuracy_filter.his_data.axis_x += check_direction(delta_x);
			data->accuracy_filter.his_data.axis_y += check_direction(delta_y);
			data->accuracy_filter.his_data.count++;
		}

		if(data->accuracy_filter.his_data.count == 1
		    ||(  (jitter_sub(data->ts_data.curr_data[0].pressure, data->accuracy_filter.his_data.pressure) > data->accuracy_filter.ignore_pressure_gap
			|| data->ts_data.curr_data[0].pressure > data->accuracy_filter.max_pressure)
				&& !(  (data->accuracy_filter.his_data.count > data->accuracy_filter.time_to_max_pressure
				&& (jitter_abs(data->accuracy_filter.his_data.axis_x) == data->accuracy_filter.his_data.count
				|| jitter_abs(data->accuracy_filter.his_data.axis_y) == data->accuracy_filter.his_data.count))
			      ||(jitter_abs(data->accuracy_filter.his_data.axis_x) > data->accuracy_filter.direction_count
				|| jitter_abs(data->accuracy_filter.his_data.axis_y) > data->accuracy_filter.direction_count)))){
					data->accuracy_filter.his_data.mod_x += delta_x;
					data->accuracy_filter.his_data.mod_y += delta_y;
		}
	}

	// if 'delta' > delta_max or id != 0, remove the modify-value.
	if(id != 0 ||
	   (data->accuracy_filter.his_data.count != 1 &&
	    (jitter_abs(delta_x) > data->accuracy_filter.delta_max || jitter_abs(delta_y) > data->accuracy_filter.delta_max))){
		data->accuracy_filter.his_data.mod_x = 0;
		data->accuracy_filter.his_data.mod_y = 0;
	}

	// start the accuracy_filter
	if(data->accuracy_filter.finish_filter == 0
	   && data->accuracy_filter.his_data.count == 0
	   && data->ts_data.total_num == 1
	   && data->accuracy_filter.his_data.prev_total_num == 0
	   && id == 0){
		data->accuracy_filter.finish_filter = 1;
		memset(&data->accuracy_filter.his_data, 0, sizeof(data->accuracy_filter.his_data));
	}

	data->accuracy_filter.his_data.x = data->ts_data.curr_data[0].x_position;
	data->accuracy_filter.his_data.y = data->ts_data.curr_data[0].y_position;
	data->accuracy_filter.his_data.pressure = data->ts_data.curr_data[0].pressure;
	data->accuracy_filter.his_data.prev_total_num = data->ts_data.total_num;

	if(data->ts_data.total_num){
		data->ts_data.curr_data[0].x_position
			= check_boundary((int)data->ts_data.curr_data[0].x_position + data->accuracy_filter.his_data.mod_x, data->pdata->max_x);
		data->ts_data.curr_data[0].y_position
			= check_boundary((int)data->ts_data.curr_data[0].y_position + data->accuracy_filter.his_data.mod_y, data->pdata->max_y);
	}

	return 0;
}
EXPORT_SYMBOL(accuracy_filter_function);

static int ghost_detection_solution(struct mxt_data *data)
{
	int id;

	if (data->ts_data.total_num) {
		if (data->ts_data.prev_total_num != data->ts_data.total_num) {
			if (data->ts_data.prev_total_num <= data->ts_data.total_num) {
				for(id=0; id < data->pdata->numtouch; id++){
					if (data->ts_data.curr_data[id].status == FINGER_PRESSED
							&& data->ts_data.prev_data[id].status == FINGER_RELEASED) {
						break;
					}
				}

				if (id < data->pdata->numtouch) {
					memcpy(&time_ex_debug[TIME_EX_PREV_PRESS_TIME], &time_ex_debug[TIME_EX_CURR_PRESS_TIME], sizeof(struct timeval));
					do_gettimeofday(&time_ex_debug[TIME_EX_CURR_PRESS_TIME]);

					if ( 1<= data->ts_data.prev_total_num && 1<= data->ts_data.total_num
							&& jitter_sub(prev_finger_press_data.x_position,data->ts_data.curr_data[id].x_position)<=10
							&& jitter_sub(prev_finger_press_data.y_position, data->ts_data.curr_data[id].y_position)<=10)
					{
						// if time_interval between prev fingger pressed and curr finger pressed is less than 50ms, we need to rebase touch ic.
						if(((time_ex_debug[TIME_EX_CURR_PRESS_TIME].tv_sec - time_ex_debug[TIME_EX_PREV_PRESS_TIME].tv_sec)==1) &&
							(( get_interval(time_ex_debug[TIME_EX_CURR_PRESS_TIME].tv_usec+1000000, time_ex_debug[TIME_EX_PREV_PRESS_TIME].tv_usec)) <= 50*1000))
						{
							dev_info(&data->client->dev, "ghost detected 1 \n");
							ghost_detected = true;
							ghost_detection_cnt++;
						}
						else if(((time_ex_debug[TIME_EX_CURR_PRESS_TIME].tv_sec - time_ex_debug[TIME_EX_PREV_PRESS_TIME].tv_sec)==0) &&
							(( get_interval(time_ex_debug[TIME_EX_CURR_PRESS_TIME].tv_usec, time_ex_debug[TIME_EX_PREV_PRESS_TIME].tv_usec)) <= 50*1000))
						{
							dev_info(&data->client->dev, "ghost detected 2 \n");
							ghost_detected = true;
							ghost_detection_cnt++;
						}
						else	; // do not anything
					} else if (data->ts_data.prev_total_num==0 && data->ts_data.total_num==1
							&& jitter_sub(prev_finger_press_data.x_position,data->ts_data.curr_data[id].x_position)<=10
							&& jitter_sub(prev_finger_press_data.y_position,data->ts_data.curr_data[id].y_position)<=10 )
					{
					       // if distance between prev fingger pressed and curr finger pressed is less than 50ms, we need to rebase touch ic.
						if(((time_ex_debug[TIME_EX_CURR_PRESS_TIME].tv_sec - time_ex_debug[TIME_EX_PREV_PRESS_TIME].tv_sec)==1) &&
							(( get_interval(time_ex_debug[TIME_EX_CURR_PRESS_TIME].tv_usec+1000000, time_ex_debug[TIME_EX_PREV_PRESS_TIME].tv_usec)) <= 50*1000))
						{
							dev_info(&data->client->dev, "ghost detected 3 \n");
							ghost_detected = true;
						}
						else if(((time_ex_debug[TIME_EX_CURR_PRESS_TIME].tv_sec - time_ex_debug[TIME_EX_PREV_PRESS_TIME].tv_sec)==0) &&
							(( get_interval(time_ex_debug[TIME_EX_CURR_PRESS_TIME].tv_usec, time_ex_debug[TIME_EX_PREV_PRESS_TIME].tv_usec)) <= 50*1000))
						{
							dev_info(&data->client->dev, "ghost detected 4 \n");
							ghost_detected = true;
						}
						else	; // do not anything
					} else if ( 5 < jitter_sub(data->ts_data.prev_total_num,data->ts_data.total_num) )
					{
						dev_info(&data->client->dev, "ghost detected 5 \n");
						 ghost_detected = true;
					} else; //do not anything

					memcpy(&prev_finger_press_data, &data->ts_data.curr_data[id], sizeof(prev_finger_press_data));
				}
			} else {
				memcpy(&time_ex_debug[TIME_EX_PREV_PRESS_TIME], &time_ex_debug[TIME_EX_CURR_PRESS_TIME], sizeof(struct timeval));
				do_gettimeofday(&time_ex_debug[TIME_EX_CURR_INT_TIME]);

				// if finger subtraction time is less than 10ms, we need to check ghost state.
				if(((time_ex_debug[TIME_EX_CURR_INT_TIME].tv_sec - time_ex_debug[TIME_EX_PREV_PRESS_TIME].tv_sec)==1) &&
						(( get_interval(time_ex_debug[TIME_EX_CURR_INT_TIME].tv_usec+1000000, time_ex_debug[TIME_EX_PREV_PRESS_TIME].tv_usec)) < 11*1000))
					finger_subtraction_check_cnt++;
				else if(((time_ex_debug[TIME_EX_CURR_INT_TIME].tv_sec - time_ex_debug[TIME_EX_PREV_PRESS_TIME].tv_sec)==0) &&
						(( get_interval(time_ex_debug[TIME_EX_CURR_INT_TIME].tv_usec, time_ex_debug[TIME_EX_PREV_PRESS_TIME].tv_usec)) < 11*1000))
					finger_subtraction_check_cnt++;
				else
					finger_subtraction_check_cnt = 0;

				if(4<finger_subtraction_check_cnt){
					finger_subtraction_check_cnt = 0;
					dev_info(&data->client->dev, "need_to_rebase finger_subtraction!!! \n");
					goto out_need_to_rebase;
				}
			}
		}
	} else {
		finger_subtraction_check_cnt = 0;
	}

	if(ghost_detected == true && data->ts_data.total_num == 0) {
		dev_info(&data->client->dev, "need_to_rebase zero\n");

		goto out_need_to_rebase;
	}
	else if (ghost_detected == true && 3 <= ghost_detection_cnt) {
		dev_info(&data->client->dev, "need_to_rebase zero 3\n");
		goto out_need_to_rebase;
	}

	return NO_PROBLEM;

out_need_to_rebase:
	{
		ghost_detected = false;
		ghost_detection_cnt = 0;
		memset(&prev_finger_press_data, 0x0, sizeof(prev_finger_press_data));
		rebase_count++;

		dev_info(&data->client->dev, "rebase count %d\n", rebase_count);
		if(rebase_count==1) {
			do_gettimeofday(&time_ex_debug[TIME_EX_FIRST_GHOST_DETECT_TIME]);
		} else {
			do_gettimeofday(&time_ex_debug[TIME_EX_SECOND_GHOST_DETECT_TIME]);

			if(((time_ex_debug[TIME_EX_SECOND_GHOST_DETECT_TIME].tv_sec - time_ex_debug[TIME_EX_FIRST_GHOST_DETECT_TIME].tv_sec) <= 5))
			{
				rebase_count = 0;
				dev_info(&data->client->dev, "need_to_init\n");
					goto out_need_to_init;
			} else {
				rebase_count = 1;
				memcpy(&time_ex_debug[TIME_EX_FIRST_GHOST_DETECT_TIME], &time_ex_debug[TIME_EX_SECOND_GHOST_DETECT_TIME], sizeof(struct timeval));
			}
		}

		if (mxt_t6_command(data, MXT_COMMAND_CALIBRATE, 1, false)) {
			dev_info(&data->client->dev, "IC_CTRL_CAL handling fail \n");
		}
		dev_info(&data->client->dev, "need_to_rebase\n");
	}
	return NEED_TO_OUT;

out_need_to_init:
	return NEED_TO_INIT;
}
#endif

static void mxt_proc_t9_message(struct mxt_data *data, u8 *message)
{
	struct device *dev = &data->client->dev;
#ifndef CUST_A_TOUCH
	struct input_dev *input_dev = data->input_dev;
#endif
	int id;
	u8 status;
	int x;
	int y;
	int area;
	int amplitude;
	u8 vector;
	int tool;
#if 0//def CHANGE_PEN_CFG
	static int s_pre_x= 0;
	static int s_pre_y= 0;
	static int s_cur_pen_speed_mode= LGE_CFG_TYPE_PEN;
	static int s_mid_speed_count= 0;
	static int s_low_speed_count= 0;
	int cur_speed= 0;
#endif
	/* do not report events if input device not yet registered */
	if (!data->enable_reporting)
		return;

	id = message[0] - data->T9_reportid_min;
	status = message[1];
	x = (message[2] << 4) | ((message[4] >> 4) & 0xf);
	y = (message[3] << 4) | ((message[4] & 0xf));

	/* Handle 10/12 bit switching */
	if (data->max_x < 1024)
		x >>= 2;
	if (data->max_y < 1024)
		y >>= 2;

	area = message[5];

	amplitude = message[6];
	vector = message[7];

	dev_dbg(dev,
		"[%u] %c%c%c%c%c%c%c%c x: %5u y: %5u area: %3u amp: %3u vector: %02X\n",
		id,
		(status & MXT_T9_DETECT) ? 'D' : '.',
		(status & MXT_T9_PRESS) ? 'P' : '.',
		(status & MXT_T9_RELEASE) ? 'R' : '.',
		(status & MXT_T9_MOVE) ? 'M' : '.',
		(status & MXT_T9_VECTOR) ? 'V' : '.',
		(status & MXT_T9_AMP) ? 'A' : '.',
		(status & MXT_T9_SUPPRESS) ? 'S' : '.',
		(status & MXT_T9_UNGRIP) ? 'U' : '.',
		x, y, area, amplitude, vector);
#ifndef CUST_A_TOUCH
	if (id < 0 || id >= data->pdata->numtouch) {
		dev_err(dev, "limited number of finger is %d\n", data->pdata->numtouch);
		return;
	}
	input_mt_slot(input_dev, id);
#endif
	if (status & MXT_T9_DETECT) { // Press
		/* Multiple bits may be set if the host is slow to read the
		 * status messages, indicating all the events that have
		 * happened */
		if (status & MXT_T9_RELEASE) {
#ifdef CUST_A_TOUCH
			data->ts_data.curr_data[id].id = id;
			data->ts_data.curr_data[id].status = FINGER_RELEASED;
#else
			input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
			mxt_input_sync(input_dev);
#endif
		}
		/* A reported size of zero indicates that the reported touch
		 * is a stylus from a linked Stylus T47 object. */
		if (area == 0) {
			area = MXT_TOUCH_MAJOR_T47_STYLUS;
			tool = MT_TOOL_FINGER;
			//tool = MT_TOOL_PEN;
			data->ts_data.curr_data[id].is_pen = true;
#ifdef USE_FW_11AA
			message[8] = 1;
			message[9] = 1;
#endif
#ifdef CHANGE_PEN_CFG
		    data->stylus_pen_pressed = true;
#endif
		// LGE_CHANGE_S [naomi.kim@lge.com] 13.06.17, set data format
		} else if ((status & MXT_T9_SUPPRESS) /*& (status & MXT_T9_UNGRIP)*/) {
			tool = MT_TOOL_PALM;
		// LGE_CHANGE_E [naomi.kim@lge.com] 13.06.17, set data format
			data->palm_pressed = true;
		} else {
			tool = MT_TOOL_FINGER;
#ifdef CHANGE_PEN_CFG
			data->stylus_pen_pressed = false;
#endif
		}

//accuracy solution
#ifdef CUST_A_TOUCH
		data->ts_data.curr_data[id].id = id;
		data->ts_data.curr_data[id].x_position = x;
		data->ts_data.curr_data[id].y_position = y;

#ifdef USE_FW_11AA
		if (message[8] >= message[9]) {
			data->ts_data.curr_data[id].touch_major = message[8];
			data->ts_data.curr_data[id].touch_minor = message[9];
		} else {
			data->ts_data.curr_data[id].touch_major = message[9];
			data->ts_data.curr_data[id].touch_minor = message[8];
		}
#else
		data->ts_data.curr_data[id].touch_major = area;
#endif
		data->ts_data.curr_data[id].orientation = vector;
		data->ts_data.curr_data[id].tool = tool;
		if(tool == MT_TOOL_PALM)
			data->ts_data.curr_data[id].pressure = 255;
		else
			data->ts_data.curr_data[id].pressure = amplitude;

		if (status & MXT_T9_PRESS) {
			data->ts_data.curr_data[id].status = FINGER_PRESSED;

#if 0//def CHANGE_PEN_CFG
			if (data->stylus_pen_pressed) {
				//dev_info (dev, "Start writing PEN config\n");
				data->pen_pressed_ids = data->pen_pressed_ids | (1 << id);

				if (data->stylus_pen_cfg_adapted == false) {
					if (change_config(data, LGE_CFG_TYPE_PEN)) {
						dev_err(&data->client->dev, "Failed to write pen config\n");
					} else {
						dev_dbg (dev, "writing PEN config END\n");
						data->stylus_pen_cfg_adapted = true;
						s_cur_pen_speed_mode= LGE_CFG_TYPE_PEN;
					}
				}

				s_pre_x= 0;
				s_pre_y= 0;
				s_mid_speed_count= 0;
				s_low_speed_count= 0;

			}
			else if (
				data->stylus_pen_cfg_adapted == true &&
				data->pen_pressed_ids == 0) {

				if (change_config(data, LGE_CFG_TYPE_UNPEN)) {
					dev_err(&data->client->dev, "Failed to write unpen config\n");
				} else {
					dev_dbg (dev, "writing UNPEN config End\n");
					data->stylus_pen_cfg_adapted = false;
				}

			}

#endif
		} else if (status & 	MXT_T9_MOVE) {
			data->ts_data.curr_data[id].status = FINGER_MOVED;

#if 0//def CHANGE_PEN_CFG
			if (data->stylus_pen_pressed) {

				if (s_pre_x != 0 && s_pre_y !=0) {
					cur_speed= jitter_sub(x, s_pre_x) + jitter_sub(y, s_pre_y);
				}

				s_pre_x= x;
				s_pre_y= y;

				if (cur_speed <= LOW_SPEED_PEN_THRESHOLD) {
					s_low_speed_count++;
					if (PEN_SPEED_CHANGE_THRESHOLD <= s_low_speed_count
						&& s_cur_pen_speed_mode != LGE_CFG_TYPE_LOW_SPEED_PEN) {

						change_config(data, LGE_CFG_TYPE_LOW_SPEED_PEN);
						s_cur_pen_speed_mode= LGE_CFG_TYPE_LOW_SPEED_PEN;
					}

					s_mid_speed_count= 0;
				}
				else {
					s_mid_speed_count++;
					if (PEN_SPEED_CHANGE_THRESHOLD <= s_mid_speed_count
						&& s_cur_pen_speed_mode != LGE_CFG_TYPE_PEN) {

						change_config(data, LGE_CFG_TYPE_PEN);
						s_cur_pen_speed_mode= LGE_CFG_TYPE_PEN;
					}

					s_low_speed_count= 0;
				}

			}
#endif

		}
		dev_dbg(dev, "%s : curr_data[%d] x(%d), y(%d), area(%d), amplitude(%d)\n",
				__func__, id, x, y, area, amplitude);
#else		//CUST_A_TOUCH
		/* Touch active */
		input_mt_report_slot_state(input_dev, tool, 1);
		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(input_dev, ABS_MT_PRESSURE, amplitude);
		input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, area);
		input_report_abs(input_dev, ABS_MT_ORIENTATION, vector);
#endif		//CUST_A_TOUCH
	} else { // Release
		/* Touch no longer active, close out slot */
#ifdef CUST_A_TOUCH
		data->ts_data.curr_data[id].id = id;
		data->ts_data.curr_data[id].status = FINGER_RELEASED;
#else
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
		mxt_input_sync(input_dev);
#endif

#if 0//def CHANGE_PEN_CFG
		data->pen_pressed_ids = data->pen_pressed_ids & (0 << id);

		if (data->pen_pressed_ids == 0) {
			change_config(data, LGE_CFG_TYPE_UNPEN);
			data->stylus_pen_cfg_adapted = false;
		}
		else if (data->stylus_pen_pressed) {

			if (s_cur_pen_speed_mode != LGE_CFG_TYPE_PEN) {
				change_config(data, LGE_CFG_TYPE_PEN);
				s_cur_pen_speed_mode= LGE_CFG_TYPE_PEN;
			}
		}
#endif
	}

	if ((data->autocal_step == 2 && !data->antitouch_timer1_started)
			&& (status & MXT_T9_RELEASE)) {
		dev_info(dev, "[step %d]release event occured\n", data->autocal_step);
		mxt_t61_timer_set(data, data->autocal_step - 1, MXT_T61_TIMER_CMD_START);
	}
	else if ((data->autocal_step == 2 && !data->antitouch_timer1_started)
			&& data->ts_data.curr_data[id].is_pen
			&& ((status & MXT_T9_PRESS) || status & MXT_T9_MOVE)) {
		dev_info(dev, "[step %d]stylus pressed\n", data->autocal_step);
		mxt_t61_timer_set(data, data->autocal_step - 1, MXT_T61_TIMER_CMD_START);
	}
	else if (data->antitouch_timer2_started && (status & MXT_T9_RELEASE) && data->ts_data.prev_data[id].is_pen) {
		if ((jitter_sub(data->ts_data.prev_data[id].x_position, data->ref_stylus_x) < 50)
				&& (jitter_sub(data->ts_data.prev_data[id].y_position, data->ref_stylus_y) < 50)) {
			dev_info(dev, "Stylus ghost Occurred. Excute Calibration\n");
			must_calibration = true;
		} else {
			dev_info(dev, "Stop tracking stylus\n");
			data->stop_tracking_stylus = false;
		}

		data->ref_stylus_x = data->ref_stylus_y = 0;
	}

	data->update_input = true;
}

static void mxt_proc_t25_message(struct mxt_data *data, u8 *message)
{
	struct device *dev = &data->client->dev;
	u8 status = message[1];

	dev_dbg(dev, "T25 Self Test completed %u\n",status);

	if ( status == 0xFE ) {
		dev_info(dev, "[SUCCESS] All tests passed\n");
		data->self_test_result = true;
	} else {
		if (status == 0xFD) {
			dev_err(dev, "[FAIL] Invalid test code\n");
		} else if (status == 0xFC)  {
			dev_err(dev, "[FAIL] Unrelated fault\n");
		} else if (status == 0x01) {
			dev_err(dev, "[FAIL] AVdd or XVdd is not present\n");
		} else if (status == 0x12) {
			dev_err(dev, "[FAIL] Pin fault (SEQ_NUM %u, X_PIN %u, Y_PIN %u)\n", message[2], message[3], message[4]);
		} else if (status == 0x17) {
			dev_err(dev, "[FAIL] Signal limit fault (TYPE_NUM %u, TYPE_INSTANCE %u)\n", message[2], message[3]);
		} else;
		data->self_test_result = false;
	}
	complete(&data->t25_completion);
}

static void mxt_proc_t24_message(struct mxt_data *data, u8 *message)
{
	struct device *dev = &data->client->dev;
//	struct input_dev *input_dev = data->input_dev;
	u8 msg;
	int x;
	int y;

	if (data->in_bootloader)
		return;

	msg = message[1];

	x = (message[2] << 4) | ((message[4] >> 4) & 0xf);
	y = (message[3] << 4) | ((message[4] & 0xf));

	if(msg == 0x04) {
		dev_info(dev, "Double_Tap!!     %d     %d \n",x,y);
		input_report_key(data->input_dev, KEY_POWER, true);
		input_sync(data->input_dev);
		msleep(MXT_WAKEUP_TIME);
		input_report_key(data->input_dev, KEY_POWER, false);
		input_sync(data->input_dev);
	}
}

static void mxt_proc_t100_message(struct mxt_data *data, u8 *message)
{
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev = data->input_dev;
	int id;
	u8 status;
	int x;
	int y;
	int tool;

	/* do not report events if input device not yet registered */
	if (!data->enable_reporting)
		return;

	id = message[0] - data->T100_reportid_min - 2;

	/* ignore SCRSTATUS events */
	if (id < 0)
		return;

	status = message[1];
	x = (message[3] << 8) | message[2];
	y = (message[5] << 8) | message[4];

	dev_dbg(dev,
		"[%u] status:%02X x:%u y:%u area:%02X amp:%02X vec:%02X\n",
		id,
		status,
		x, y,
		(data->t100_aux_area) ? message[data->t100_aux_area] : 0,
		(data->t100_aux_ampl) ? message[data->t100_aux_ampl] : 0,
		(data->t100_aux_vect) ? message[data->t100_aux_vect] : 0);

	input_mt_slot(input_dev, id);

	if (status & MXT_T100_DETECT) {
		/* A reported size of zero indicates that the reported touch
		 * is a stylus from a linked Stylus T47 object. */
		if ((status & MXT_T100_TYPE_MASK) == MXT_T100_TYPE_STYLUS)
			tool = MT_TOOL_PEN;
		else
			tool = MT_TOOL_FINGER;

		/* Touch active */
		input_mt_report_slot_state(input_dev, tool, 1);
		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);

		if (data->t100_aux_ampl)
			input_report_abs(input_dev, ABS_MT_PRESSURE,
					 message[data->t100_aux_ampl]);

		if (data->t100_aux_area) {
			if (tool == MT_TOOL_PEN)
				input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR,
						 MXT_TOUCH_MAJOR_T47_STYLUS);
			else
				input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR,
						 message[data->t100_aux_area]);
		}

		if (data->t100_aux_vect)
			input_report_abs(input_dev, ABS_MT_ORIENTATION,
					 message[data->t100_aux_vect]);
	} else {
		/* Touch no longer active, close out slot */
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
	}

	data->update_input = true;
}


static void mxt_proc_t15_messages(struct mxt_data *data, u8 *msg)
{
	struct input_dev *input_dev = data->input_dev;
	struct device *dev = &data->client->dev;
	int key;
	bool curr_state, new_state;
	bool sync = false;
	unsigned long keystates = le32_to_cpu(msg[2]);

	/* do not report events if input device not yet registered */
	if (!data->enable_reporting)
		return;

	for (key = 0; key < data->pdata->t15_num_keys; key++) {
		curr_state = test_bit(key, &data->t15_keystatus);
		new_state = test_bit(key, &keystates);

		if (!curr_state && new_state) {
			dev_dbg(dev, "T15 key press: %u\n", key);
			__set_bit(key, &data->t15_keystatus);
			input_event(input_dev, EV_KEY,
				    data->pdata->t15_keymap[key], 1);
			sync = true;
		} else if (curr_state && !new_state) {
			dev_dbg(dev, "T15 key release: %u\n", key);
			__clear_bit(key, &data->t15_keystatus);
			input_event(input_dev, EV_KEY,
				    data->pdata->t15_keymap[key], 0);
			sync = true;
		}
	}

	if (sync)
		input_sync(input_dev);
}

static void mxt_proc_t42_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status = msg[1];

	if (status & MXT_T42_MSG_TCHSUP)
		dev_dbg(dev, "T42 suppress\n");
	else
		dev_dbg(dev, "T42 normal\n");
}

static int mxt_proc_t48_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	u8 status, state;

	status = msg[1];
	state  = msg[4];

	dev_dbg(dev, "T48 state %d status %02X %s%s%s%s%s\n",
			state,
			status,
			(status & 0x01) ? "FREQCHG " : "",
			(status & 0x02) ? "APXCHG " : "",
			(status & 0x04) ? "ALGOERR " : "",
			(status & 0x10) ? "STATCHG " : "",
			(status & 0x20) ? "NLVLCHG " : "");

	return 0;
}

static void mxt_proc_t61_messages(struct mxt_data *data, u8 *msg)
{
	struct device *dev = &data->client->dev;
	struct mxt_object *object = NULL;
	u8 report_id = msg[0];

	if (data->suspended)
		return;
#ifdef USE_FW_11AA
	if (report_id == 0x35) {
#else
	if (report_id == 0x37) {
#endif
		if (msg[1] == 0xA0) {
			dev_info(dev, "Timer0 elapsed!!\n");
			mxt_t8_autocal_set(data, 2);
			data->antitouch_timer0_started =0;
			data->check_antitouch = 1;
		}
#ifdef USE_FW_11AA
	} else if (report_id == 0x36) {
#else
	} else if (report_id == 0x38) {
#endif
		if (msg[1] == 0xA0) {
			if (data->antitouch_timer1_started) {
				dev_info(dev, "Timer1 elapsed!!\n");
				mxt_t8_autocal_set(data, 3);
				data->antitouch_timer1_started =0;
			}
		}
#ifdef USE_FW_11AA
	} else if (report_id == 0x37) {
#else
	} else if (report_id == 0x39) {
#endif
		if (msg[1] == 0xA0) {
			if (data->antitouch_timer2_started) {
				dev_info(dev, "Timer2 elapsed!!\n");
				object = mxt_get_object(data, MXT_PROCG_NOISESUPPRESSION_T62);
				if (object) {
					mxt_write_reg(data->client, object->start_address + 23, 10);
				}

				data->antitouch_timer2_started =0;
				data->stop_tracking_stylus = true;
				data->ref_stylus_x = data->ref_stylus_y = 0;
			}
		}
#ifdef USE_FW_11AA
	} else if (report_id == 0x38) {
#else
	} else if (report_id == 0x3A) {
#endif
		if (msg[1] == 0xA0) {
			if (data->antitouch_timer3_started) {
				dev_info(dev, "Timer3 elapsed. Stylus drift finished\n");
				object = mxt_get_object(data, MXT_PROCG_NOISESUPPRESSION_T62);
				if (object) {
					mxt_write_reg(data->client, object->start_address + 23, 10);
				}
				data->check_multi_stylus = false;
				data->antitouch_timer3_started = 0;
			}
		}
	}
}

//Check if touch coordinate is in boundary of Top/Bottom edge
static bool check_edge_coordinate(struct mxt_data *data) {
	int id;

	for (id = 0; id < data->pdata->numtouch; id++) {
		if (((data->ts_data.prev_data[id].y_position) > 0 && (data->ts_data.prev_data[id].y_position < 100))
				|| (data->ts_data.prev_data[id].y_position > 4000)) {
			return true;
		}
	}
	return false;
}

static int check_coin_touch_func(struct mxt_data *data, int area, int tch_area, int atch_area) {
	struct mxt_object *object;
	u8 buf[6] = {0, };
	u16 sum_size = 0;
	int ret = 0;

	//Obtain config data from T38 object
	object = mxt_get_object(data, MXT_SPT_USERDATA_T38);
	if (!object)
		goto out_without_calibration;

	ret = __mxt_read_reg(data->client, object->start_address + 22, 6, buf);
	if (ret)
		goto out_without_calibration;

	if (!check_edge_coordinate(data)) {
		sum_size = tch_area + atch_area;
		if (area < buf[0] &&//area > buf[0]
				(atch_area > buf[1] && sum_size < buf[2] && (atch_area - tch_area) > buf[3])) {
			goto out_must_calibrate;
		} else if (atch_area > buf[4] && tch_area == buf[5]) {
			goto out_must_calibrate;
		} else ;
	}

out_without_calibration:
	return 0;

out_must_calibrate:
	return 1;
}

static int check_abnormal_cal_func(struct mxt_data *data, int area, int tch_area, int atch_area) {
	struct mxt_object *object;
	u8 buf[5] = {0, };
	u16 sum_size = 0;
	int ret = 0;
	int count = 0;
	int i;

	//Obtain config data from T38 object
	object = mxt_get_object(data, MXT_SPT_USERDATA_T38);
	if (!object)
		goto out_without_calibration;

	ret = __mxt_read_reg(data->client, object->start_address + 28, 5, buf);
	if (ret)
		goto out_without_calibration;

	if (!check_edge_coordinate(data)) {
		sum_size = tch_area + atch_area;
		if ((atch_area - tch_area) > buf[0] && tch_area < buf[1]) {
			goto out_must_calibrate;
		}

		if (tch_area > buf[2] && atch_area > buf[3]) {
			if (data->ts_data.total_num == 1) {
				goto out_must_calibrate;
			}
		}
	}

	//Bus card
	for (i = 0; i < data->pdata->numtouch; i++) {
		if (data->ts_data.curr_data[i].is_pen) {
			count++;
		}
	}
	if (count > 1) {
		dev_info(&data->client->dev, "Stylus detected more than 2\n");
		goto out_must_calibrate;
	}

out_without_calibration:
	return 0;

out_must_calibrate:
	return 1;
}

static int check_last_antitouch_func(struct mxt_data *data, int area, int tch_area, int atch_area) {
	struct mxt_object *object;
	u8 buf[4] = {0, };
	int ret = 0;

	//Obtain config data from T38 object
	object = mxt_get_object(data, MXT_SPT_USERDATA_T38);
	if (!object)
		goto out_without_calibration;

	ret = __mxt_read_reg(data->client, object->start_address + 33, 4, buf);
	if (ret)
		goto out_without_calibration;

	if (area > buf[0]) {
		if ((atch_area > buf[1]) && (tch_area < buf[2]) && ((tch_area + atch_area) < buf[3]) ) {
			goto out_must_calibrate;
		} else {
			data->autocal_finished = true;
			dev_info(&data->client->dev, "All anti-touch solutions Finished\n");
		}
	} else {
		dev_dbg(&data->client->dev, "area is below %d. Keep monitoring anti-touch\n", buf[0]);
	}

out_without_calibration:
	return 0;

out_must_calibrate:
	return 1;
}

static void mxt_proc_t57_messages(struct mxt_data *data, u8 *msg) {
	struct device *dev = &data->client->dev;
	static u16 prev_area = 0;
	static u16 prev_tch = 0;
	static u16 prev_atch = 0;
	static int messege_cnt = 0;
	u16 area = (msg[2] << 8) | msg[1];
	u16 tch_area = (msg[4] << 8) | msg[3];
	u16 atch_area = (msg[6] << 8) | msg[5];

	if (data->check_antitouch) {
		if (area != prev_area || tch_area != prev_tch || atch_area != prev_atch) {
			dev_dbg(dev, "Fcnt %d, SUM %d , TCH %d , ATCH %d\n", data->ts_data.prev_total_num, area, tch_area, atch_area);
		}

		if (data->autocal_step == 2) {
			if (check_coin_touch_func(data, area, tch_area, atch_area)) {
				if (++data->case2_1_detection_count > 10) {
					dev_info(dev, "Coin touch detected. Excute calibration\n");
					goto out_must_calibrate;
				}
			}
			if (check_abnormal_cal_func(data, area, tch_area, atch_area)) {
				if (++data->case2_2_detection_count > 10) {
					dev_info(dev, "Abnormal calibration detected. Excute calibration\n");
					goto out_must_calibrate;
				}
			}
		} else if (data->autocal_step == 3) {
			if (data->autocal_finished == false) {
				if (check_last_antitouch_func(data, area, tch_area, atch_area)) {
					dev_info(dev, "Last Antitouch detected. Excute calibration\n");
					goto out_must_calibrate;
				}
			}

			if ((data->ts_data.prev_total_num == 1) && (tch_area == 0) && (atch_area > 0)) {
				if (++data->case3_1_detection_count > 10) {
					dev_info(dev, "Step 3_1! Calibration!!\n");
					goto out_must_calibrate;
				}
				dev_dbg(dev, "Case 3_1 count[%d]\n", data->case3_1_detection_count);
			} else if ((data->ts_data.prev_total_num > 2) && ((data->ts_data.prev_total_num * 4) > tch_area) && atch_area > 10) {
				if (++data->case3_2_detection_count > 20) {
					dev_info(dev, "Step 3_2! Calibration!!\n");
					goto out_must_calibrate;
				}
				dev_dbg(dev, "Case 3_2 count[%d]\n", data->case3_2_detection_count);
			}
/*
			else if ((data->ts_data.prev_total_num > 1) && (data->ts_data.prev_total_num < 6) && ((atch_area * data->ts_data.prev_total_num) > tch_area) &&
				(sum*4 > tch_area) && atch_area > 20) {
				if (++data->case3_6_detection_count > 20) {
					dev_info(dev, "Step 3_6! Calibration!!\n");
					goto out_must_calibrate;
				}
				dev_dbg(dev, "Case 3_6 count[%d]\n", data->case3_6_detection_count);
			}
*/
			else if ((data->ts_data.prev_total_num >= 5) && (atch_area <= 5) && (tch_area <= 5) && (area <= 5)) {
				if (++data->case3_7_detection_count > 20) {
					dev_info(dev, "Step 3_7! Calibration!!\n");
					goto out_must_calibrate;
				}
				dev_dbg(dev, "Case 3_7 count[%d]\n", data->case3_7_detection_count);
			} else if ((data->ts_data.prev_total_num >= 2) && (atch_area == 0) && (tch_area == 0) && (area >= 4)) {
				if (++data->case3_8_detection_count > 20) {
					dev_info(dev, "Step 3_8! Calibration!!\n");
					goto out_must_calibrate;
				}
				dev_dbg(dev, "Case 3_8 count[%d]\n", data->case3_8_detection_count);
			} else if ((data->ts_data.prev_total_num == 4) && (atch_area == 0) && (tch_area == 0) && (area <= 3)) {
				if (++data->case3_9_detection_count > 20) {
					dev_info(dev, "Step 3_9! Calibration!!\n");
					goto out_must_calibrate;
				}
				dev_dbg(dev, "Case 3_9 count[%d]\n", data->case3_9_detection_count);
			}
/*
			else if ((data->ts_data.prev_total_num >= 2)  && (atch_area == 0)
					&& (tch_area <= data->ts_data.prev_total_num) && (area <= data->ts_data.prev_total_num)) {
				if (++data->case3_11_detection_count > 20) {
					dev_info(dev, "Step 3_11! Calibration!!\n");
					goto out_must_calibrate;
				}
				dev_dbg(dev, "Case 3_11 count[%d]\n", data->case3_11_detection_count);
			}
*/
		} else;
	}

	prev_area = area;
	prev_tch = tch_area;
	prev_atch = atch_area;

	if (++messege_cnt > 3000) {
		dev_info(dev, "############# Initialize T57 Messege Count #############\n");
		messege_cnt = 0;
		data->case2_1_detection_count = 0;
		data->case2_2_detection_count = 0;
		data->case3_1_detection_count = 0;
		data->case3_2_detection_count = 0;
		data->case3_3_detection_count = 0;
		data->case3_4_detection_count = 0;
		data->case3_5_detection_count = 0;
		data->case3_6_detection_count = 0;
		data->case3_7_detection_count = 0;
		data->case3_8_detection_count = 0;
		data->case3_9_detection_count = 0;
		data->case3_10_detection_count = 0;
		data->case3_11_detection_count = 0;
		data->case3_12_detection_count = 0;
	}
	return;
out_must_calibrate:
	data->check_antitouch = 0;
	prev_area = prev_tch = prev_atch = 0;
	must_calibration = true;
}

static int mxt_proc_message(struct mxt_data *data, u8 *message)
{
	u8 report_id = message[0];
	bool dump = data->debug_enabled;

	if (report_id == MXT_RPTID_NOMSG)
		return 0;

	if (report_id == data->T6_reportid) {
		mxt_proc_t6_messages(data, message);
	} else if (report_id >= data->T9_reportid_min
	    && report_id <= data->T9_reportid_max) {
		mxt_proc_t9_message(data, message);
	} else if (report_id >= data->T100_reportid_min
	    && report_id <= data->T100_reportid_max) {
		mxt_proc_t100_message(data, message);
	} else if (report_id == data->T19_reportid) {
		mxt_input_button(data, message);
		data->update_input = true;
	} else if (report_id >= data->T42_reportid_min
		   && report_id <= data->T42_reportid_max) {
		mxt_proc_t42_messages(data, message);
	} else if (report_id == data->T48_reportid) {
		mxt_proc_t48_messages(data, message);
	} else if (report_id >= data->T15_reportid_min
		   && report_id <= data->T15_reportid_max) {
		mxt_proc_t15_messages(data, message);
	} else if (report_id >= data->T61_reportid_min
		   && report_id <= data->T61_reportid_max) {
		mxt_proc_t61_messages(data, message);
	} else if (report_id >= data->T57_reportid_min
		   && report_id <= data->T57_reportid_max) {
		mxt_proc_t57_messages(data, message);
	} else if (report_id == data->T24_reportid) {
		mxt_proc_t24_message(data, message);
	} else if (report_id == data->T25_reportid) {
		mxt_proc_t25_message(data, message);
	} else {
		dump = true;
	}

	if (dump)
		mxt_dump_message(data, message);

	return 1;
}

static int mxt_read_and_process_messages(struct mxt_data *data, u8 count)
{
	struct device *dev = &data->client->dev;
	int ret;
	int i;
	u8 num_valid = 0;

	/* Safety check for msg_buf */
	if (count > data->max_reportid)
		return -EINVAL;

	/* Process remaining messages if necessary */
	ret = __mxt_read_reg(data->client, data->T5_address,
				data->T5_msg_size * count, data->msg_buf);
	if (ret) {
		dev_err(dev, "Failed to read %u messages (%d)\n", count, ret);
		return ret;
	}

	for (i = 0;  i < count; i++) {
		ret = mxt_proc_message(data,
			data->msg_buf + data->T5_msg_size * i);

		if (ret == 1)
			num_valid++;
	}
	/* return number of messages read */
	return num_valid;
}

// LGE_CHANGE_S [naomi.kim@lge.com] 13.06.18, make width minor data
#if TOUCHEVENTFILTER
int set_minor_data(struct mxt_data *data, int area, u8 vector)
{
	struct device *dev = &data->client->dev;

	u8 tmp;
	int component1;
	int component2;
	int magnitude = 0;
	int minor = 0;

	int i;

	/* 1. get componet data */
	// component1
	tmp = ( (vector >> 4) & 0xf);
	if( tmp & 0x8 )
		component1 = (int)(((~(tmp) ) & 0xf) | 0x1 );
	else
		component1 = tmp;

	// component2
	tmp = (vector & 0xf);
	if( tmp & 0x8 )
		component2 = (int)(((~(tmp) ) & 0xf) | 0x1 );
	else
		component2 = tmp;

	/* 2. magnitude = SQRT(component1^2 + component2^2) */
	tmp = (component1 * component1) + (component2 * component2);

	/* 3. make minor date
	// when the magnitude is same, the larger area has the longer minor value.
	// that means, when the area is same, the longer magnitude has the shorter minor value.
	// 3-1. if shape is circle, minor = major.
	// when the shape is circle, vector = 0x00 / 0x01 / 0x10 / 0x11
	// then magnitude^2 = 0 /1 / 1 / 2
	*/
	if ( tmp < 3 )
	{
		minor = area;
		magnitude = 1;
	}
	else {
	/* 3-2. if shape is elipse, minor = area / magnitude.	*/
	// find SQRT(magnitude^2)
		for( i = 9 ; i > 1 ; i--) {
			if ( tmp > ((i*i) - 1) ){
				magnitude = i;
				break;
			}
		}
		minor = area / magnitude;
	}

	dev_dbg(dev,
		"%5u area: %5u minor: %5u magnitude: %5u vector: %5u component1: %5u component2: %5u \n",
			tmp, area, minor, magnitude, vector, component1, component2);

	return minor;
}
#endif	//TOUCHEVENTFILTER
// LGE_CHANGE_E [naomi.kim@lge.com] 13.06.18, make width minor data

static bool tracking_palm_coordinate(struct mxt_data *data) {
	struct device *dev = &data->client->dev;
	static u16 ref_palm_x = 0;
	static u16 ref_palm_y = 0;
	static u16 ref_palm_z = 0;
	static int press_cnt = 0;
	int distance_x = 0;
	int distance_y = 0;
	int delta_pressure = 0;
	int id;

	for (id = 0; id < data->pdata->numtouch; id++) {
		if (data->ts_data.curr_data[id].tool == MT_TOOL_PALM) {
			break;
		}
	}

	if (id >= 0) {
		distance_x = jitter_sub(data->ts_data.curr_data[id].x_position, ref_palm_x);
		distance_y = jitter_sub(data->ts_data.curr_data[id].y_position, ref_palm_y);
		delta_pressure = jitter_sub(data->ts_data.curr_data[id].pressure, ref_palm_z);
		if ((distance_x < 30) && (distance_y < 30) && (delta_pressure <= 10 )) {
			dev_dbg(dev, "cnt[%d]  DistanceX[%d] DistanceY[%d] Pressure[%d]\n", press_cnt, distance_x, distance_y, delta_pressure);
			press_cnt++;
		} else {
			ref_palm_x = data->ts_data.curr_data[id].x_position;
			ref_palm_y = data->ts_data.curr_data[id].y_position;
			ref_palm_z = data->ts_data.curr_data[id].pressure;
			press_cnt = 0;
		}
	} else {
		dev_err(dev, "Invalid ID[%d]\n", id);
	}

	if (press_cnt > 40) {
		ref_palm_x = ref_palm_y = press_cnt = 0;
		return true;
	}

	return false;
}

static int tracking_stylus_coordinate(struct mxt_data *data) {
	struct device *dev = &data->client->dev;
	static int press_cnt = 0;
	int distance_x = 0;
	int distance_y = 0;
	int id;

	for (id = 0; id < data->pdata->numtouch; id++) {
		if (data->ts_data.curr_data[id].is_pen) {
			break;
		}
	}

	if (id >= 0) {
		distance_x = jitter_sub(data->ts_data.curr_data[id].x_position, data->ref_stylus_x);
		distance_y = jitter_sub(data->ts_data.curr_data[id].y_position, data->ref_stylus_y);
		if ((distance_x < 30) && (distance_y < 15)) {
			dev_dbg(dev, "cnt[%d]  DistanceX[%d] DistanceY[%d]\n", press_cnt, distance_x, distance_y);
			press_cnt++;
		} else {
			data->ref_stylus_x = data->ts_data.curr_data[id].x_position;
			data->ref_stylus_y = data->ts_data.curr_data[id].y_position;
			press_cnt = 0;
		}

	} else {
		dev_err(dev, "Invalid ID[%d]\n", id);
	}

	if (press_cnt > 50) {
		press_cnt = 0;
		return 1; // stylus tracking count
	}

	return 0; // do nothing
}

#ifdef CUST_A_TOUCH
static char* get_tool_type(struct mxt_data *data, struct t_data touch_data) {
	if (touch_data.tool == MT_TOOL_FINGER) {
		if (touch_data.is_pen) {
			return "PEN";
		} else {
			return "FINGER";
		}
	} else if (touch_data.tool == MT_TOOL_PALM) {
		return "PALM";
	} else {
		dev_err(&data->client->dev, "Invalid tool type : %d", touch_data.tool);
	}
	return "Unknown";
}

static int mxt_soft_reset(struct mxt_data *data);
#endif

static void mxt_regulator_enable(struct mxt_data *data);
static void mxt_regulator_disable(struct mxt_data *data);

static irqreturn_t mxt_process_messages_t44(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	struct mxt_object *object = NULL;
	int ret;
	u8 count, num_left;
#ifdef CUST_A_TOUCH
	static int multi_stylus_cnt = 0;
	static int report_count = 0;
	int report_num = 0;
	int pen_count = 0;
	char *tool_type;
	int i;
//	bool long_pressed = false;
#endif

	/* Read T44 and T5 together */
	ret = __mxt_read_reg(data->client, data->T44_address,
		data->T5_msg_size + 1, data->msg_buf);
	if (ret) {
		dev_err(dev, "Failed to read T44 and T5 (%d)\n", ret);
		/* RESET IC FOR INTERRUPT */
		dev_err(dev, "Touch IC Interrupt level is Low, HW RESET\n");
		mxt_regulator_disable(data);
		msleep(300);
		mxt_regulator_enable(data);

		return IRQ_NONE;
	}

	count = data->msg_buf[0];

	if (count == 0) {
		dev_dbg(dev, "Interrupt triggered but zero messages\n");
		return IRQ_NONE;
	} else if (count > data->max_reportid) {
		dev_err(dev, "T44 count %d exceeded max report id\n", count);
		count = data->max_reportid;
	}

#ifdef CUST_A_TOUCH
		data->ts_data.total_num = 0;
		data->palm_pressed = false;
#endif

	/* Process first message */
	ret = mxt_proc_message(data, data->msg_buf + 1);
	if (ret < 0) {
		dev_warn(dev, "Unexpected invalid message\n");
		return IRQ_NONE;
	}

	num_left = count - 1;

	/* Process remaining messages if necessary */
	if (num_left) {
		ret = mxt_read_and_process_messages(data, num_left);

		if (ret < 0)
			goto end;
		else if (ret != num_left)
			dev_warn(dev, "Unexpected invalid message\n");
	}
#ifdef CUST_A_TOUCH
	for (i = 0; i < data->pdata->numtouch; i++) {
		if (data->ts_data.curr_data[i].is_pen) {
			pen_count++;
		}

		if (data->ts_data.curr_data[i].status == FINGER_INACTIVE &&
			data->ts_data.prev_data[i].status != FINGER_INACTIVE &&
			data->ts_data.prev_data[i].status != FINGER_RELEASED) {
			memcpy(&data->ts_data.curr_data[i], &data->ts_data.prev_data[i], sizeof(data->ts_data.prev_data[i]));
			data->ts_data.curr_data[i].skip_report = true;
		}else if (data->ts_data.curr_data[i].status == FINGER_INACTIVE) {
			continue;
		}
		if (data->ts_data.curr_data[i].status == FINGER_PRESSED ||
				data->ts_data.curr_data[i].status == FINGER_MOVED) {
			data->ts_data.total_num++;
		}
		report_num++;
	}

#endif
	if (must_calibration == false) {
		// Anti-touch solution
		if (data->antitouch_timer1_started && pen_count == 1 && data->stop_tracking_stylus == false && data->antitouch_timer2_started == false) {
			if(tracking_stylus_coordinate(data)) {
				dev_info(dev, "Start tracking stylus\n");
				mxt_t61_timer_set(data, 2, MXT_T61_TIMER_CMD_START);
				object = mxt_get_object(data, MXT_PROCG_NOISESUPPRESSION_T62);
				if (object) {
					mxt_write_reg(data->client, object->start_address + 23, 30);
				}
			}
		} else if (data->palm_pressed && data->ts_data.total_num == 1) {
			if (tracking_palm_coordinate(data)) {
				dev_info(dev, "Calibration because of palm tracking\n");
				must_calibration = true;
			}
		} else if ((pen_count > 1) && (pen_count == data->ts_data.total_num) && (data->autocal_step == 3) && (data->check_multi_stylus == false)) {
			if (++multi_stylus_cnt > 50) {
				dev_info(dev, "Multi-stylus detected. Make stylus drifted\n");
				object = mxt_get_object(data, MXT_PROCG_NOISESUPPRESSION_T62);
				if (object) {
					mxt_write_reg(data->client, object->start_address + 23, 35);
				}

				mxt_t61_timer_set(data, 3, MXT_T61_TIMER_CMD_START);
				data->check_multi_stylus = true;
				multi_stylus_cnt = 0;
			}
		}

		if (++report_count > 1000) {
			report_count = 0;
			multi_stylus_cnt = 0;
		}
	} else {
		mxt_t6_command(data, MXT_COMMAND_CALIBRATE, 1, false);
		must_calibration = false;
		report_count = 0;
		multi_stylus_cnt = 0;
		return IRQ_HANDLED;
	}

#ifdef CUST_A_TOUCH
	if (!data->enable_reporting || !report_num)
		goto out;

	/* Accuracy Solution */
	if (likely(data->pdata->accuracy_filter_enable)){
		if (accuracy_filter_function(data) < 0)
			goto end;
	}

	/* Ghost detection solution */
	if (data->pdata->ghost_detection_enable) {
		ret = ghost_detection_solution(data);
		if(ret == NEED_TO_OUT)
			goto out;
		else if(ret == NEED_TO_INIT)
			goto err_out_init;
	}

	for (i = 0; i < data->pdata->numtouch; i++) {
		if (data->ts_data.curr_data[i].status == FINGER_INACTIVE || data->ts_data.curr_data[i].skip_report) {
			continue;
		}

		if (data->ts_data.curr_data[i].status == FINGER_RELEASED) {
			input_mt_slot(data->input_dev, data->ts_data.curr_data[i].id);
			if (data->ts_data.prev_data[i].tool == MT_TOOL_FINGER) {
				input_mt_report_slot_state(data->input_dev,	MT_TOOL_FINGER, 0);
			} else if (data->ts_data.prev_data[i].tool == MT_TOOL_PALM) {
				input_mt_report_slot_state(data->input_dev,	MT_TOOL_PALM, 0);
			} else {
				input_mt_report_slot_state(data->input_dev,	MT_TOOL_FINGER, 0);
			}
		} else {
			input_mt_slot(data->input_dev, data->ts_data.curr_data[i].id);
			input_mt_report_slot_state(data->input_dev,
					data->ts_data.curr_data[i].tool, 1);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID,
					data->ts_data.curr_data[i].id);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X,
					data->ts_data.curr_data[i].x_position);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y,
					data->ts_data.curr_data[i].y_position);
			input_report_abs(data->input_dev, ABS_MT_PRESSURE,
					data->ts_data.curr_data[i].pressure);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR,
					data->ts_data.curr_data[i].touch_major);

			// LGE_CHANGE_S [naomi.kim@lge.com] 13.06.18, report width minor data
#ifdef USE_FW_11AA//0//TOUCHEVENTFILTER
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MINOR,
					data->ts_data.curr_data[i].touch_minor);
#else
			input_report_abs(data->input_dev, ABS_MT_ORIENTATION,
								data->ts_data.curr_data[i].orientation);

			data->ts_data.curr_data[i].touch_minor = set_minor_data(data,
													data->ts_data.curr_data[i].touch_major,
													data->ts_data.curr_data[i].orientation);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MINOR,
					data->ts_data.curr_data[i].touch_minor);
#endif	//TOUCHEVENTFILTER
			// LGE_CHANGE_E [naomi.kim@lge.com] 13.06.18, report width minor data

			// LGE_CHANGE_S [naomi.kim@lge.com] 13.06.18, add more debugging data
			#if TOUCHEVENTFILTER
			dev_dbg(dev,
				"report_data[%d] : x: %d y: %d, z: %d, M: %d, m: %d, orient: %d)\n",
					data->ts_data.curr_data[i].id,
					data->ts_data.curr_data[i].x_position,
					data->ts_data.curr_data[i].y_position,
					data->ts_data.curr_data[i].pressure,
					data->ts_data.curr_data[i].touch_major,
					data->ts_data.curr_data[i].touch_minor,		//added
					data->ts_data.curr_data[i].orientation
			);
			#else	//TOUCHEVENTFILTER
			dev_dbg(dev, "report_data[%d] : (x %d, y %d, presure %d, touch_major %d, orient %d)\n",
					i,
					data->ts_data.curr_data[i].x_position,
					data->ts_data.curr_data[i].y_position,
					data->ts_data.curr_data[i].pressure,
					data->ts_data.curr_data[i].touch_major,
					data->ts_data.curr_data[i].orientation
			);
			#endif	//TOUCHEVENTFILTER
			// LGE_CHANGE_E [naomi.kim@lge.com] 13.06.18, add more debugging data
		}
#if DEBUG_ABS
		if (data->ts_data.curr_data[i].status == FINGER_PRESSED) {
			tool_type = get_tool_type(data, data->ts_data.curr_data[i]);
			dev_info(dev, "%s Pressed <%d> : x[%4d] y[%4d], z[%3d]\n",
					tool_type,
					data->ts_data.curr_data[i].id,
					data->ts_data.curr_data[i].x_position,
					data->ts_data.curr_data[i].y_position,
					data->ts_data.curr_data[i].pressure);
		} else if (data->ts_data.curr_data[i].status == FINGER_RELEASED) {
			tool_type = get_tool_type(data, data->ts_data.prev_data[i]);
			dev_info(dev, "%s Released <%d>\n",
					tool_type,
					data->ts_data.curr_data[i].id);
		}
#endif
	}

	dev_dbg(dev, "Total_num(move+press)= %d\n",data->ts_data.total_num);
	if (data->ts_data.total_num) {
		data->ts_data.prev_total_num = data->ts_data.total_num;
		memcpy(data->ts_data.prev_data, data->ts_data.curr_data, sizeof(data->ts_data.curr_data));
	} else{
		data->ts_data.prev_total_num = 0;
		memset(data->ts_data.prev_data, 0, sizeof(data->ts_data.prev_data));
	}
	memset(data->ts_data.curr_data, 0, sizeof(data->ts_data.curr_data));
#endif//CUST_A_TOUCH

end:
	if (data->update_input) {
		mxt_input_sync(data->input_dev);
		data->update_input = false;
	}
#ifdef CUST_A_TOUCH
out:
	return IRQ_HANDLED;
err_out_init:
	mxt_soft_reset(data);
	return IRQ_HANDLED;
#endif
}

static int mxt_process_messages_until_invalid(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int count, read;
	u8 tries = 2;

	count = data->max_reportid;

	/* Read messages until we force an invalid */
	do {
		read = mxt_read_and_process_messages(data, count);
		if (read < count)
			return 0;
	} while (--tries);

	if (data->update_input) {
		mxt_input_sync(data->input_dev);
		data->update_input = false;
	}

	dev_err(dev, "CHG pin isn't cleared\n");
	return -EBUSY;
}

static irqreturn_t mxt_process_messages(struct mxt_data *data)
{
	int total_handled, num_handled;
	u8 count = data->last_message_count;

	if (count < 1 || count > data->max_reportid)
		count = 1;

	/* include final invalid message */
	total_handled = mxt_read_and_process_messages(data, count + 1);
	if (total_handled < 0)
		return IRQ_NONE;
	/* if there were invalid messages, then we are done */
	else if (total_handled <= count)
		goto update_count;

	/* read two at a time until an invalid message or else we reach
	 * reportid limit */
	do {
		num_handled = mxt_read_and_process_messages(data, 2);
		if (num_handled < 0)
			return IRQ_NONE;

		total_handled += num_handled;

		if (num_handled < 2)
			break;
	} while (total_handled < data->num_touchids);

update_count:
	data->last_message_count = total_handled;

	if (data->enable_reporting && data->update_input) {
		mxt_input_sync(data->input_dev);
		data->update_input = false;
	}

	return IRQ_HANDLED;
}

static irqreturn_t mxt_interrupt(int irq, void *dev_id)
{
	struct mxt_data *data = dev_id;

	if (data->in_bootloader) {
		/* bootloader state transition completion */
		complete(&data->bl_completion);
		return IRQ_HANDLED;
	}

	if (!data->object_table)
		return IRQ_NONE;

	if (data->T44_address){
		return mxt_process_messages_t44(data);
	} else {
		return mxt_process_messages(data);
	}
}

static int mxt_t6_command(struct mxt_data *data, u16 cmd_offset,
			  u8 value, bool wait)
{
	u16 reg;
	u8 command_register = 0;
	int timeout_counter = 0;
	int ret;

	reg = data->T6_address + cmd_offset;

	ret = mxt_write_reg(data->client, reg, value);
	if (ret)
		return ret;

	if (!wait)
		return 0;

	do {
		msleep(20);
		ret = __mxt_read_reg(data->client, reg, 1, &command_register);
		if (ret)
			return ret;
	} while ((command_register != 0) && (timeout_counter++ <= 100));

	if (timeout_counter > 100) {
		dev_err(&data->client->dev, "Command failed!\n");
		return -EIO;
	}

	return 0;
}

static int mxt_soft_reset(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int ret = 0;

	dev_info(dev, "Resetting chip\n");

	INIT_COMPLETION(data->reset_completion);

	ret = mxt_t6_command(data, MXT_COMMAND_RESET, MXT_RESET_VALUE, false);
	if (ret)
		return ret;

	ret = mxt_wait_for_completion(data, &data->reset_completion,
				      MXT_RESET_TIMEOUT);
	if (ret)
		return ret;

	return 0;
}

#ifdef FIRMUP_ON_PROBE
static int mxt_backup(struct mxt_data *data)
{
    struct device *dev = &data->client->dev;
	int ret = 0;

	dev_info(dev, "Backup configuration data\n");

	ret = mxt_t6_command(data, MXT_COMMAND_BACKUPNV, MXT_BACKUP_VALUE, false);
	if (ret)
		return ret;

	msleep(MXT_BACKUP_TIME);
	return 0;
}

static void mxt_update_crc(struct mxt_data *data, u8 cmd, u8 value)
{
	/* on failure, CRC is set to 0 and config will always be downloaded */
	data->config_crc = 0;
	INIT_COMPLETION(data->crc_completion);

	mxt_t6_command(data, cmd, value, true);
	/* Wait for crc message. On failure, CRC is set to 0 and config will
	 * always be downloaded */
#if 1
	/* Reduce waiting time. Because Touch IC doesn't send CRC message
	 * until reset Touch IC */
	msleep(MXT_RESET_TIME);
#else
	mxt_wait_for_completion(data, &data->crc_completion, MXT_CRC_TIMEOUT);
#endif
}
#endif

static void mxt_calc_crc24(u32 *crc, u8 firstbyte, u8 secondbyte)
{
	static const unsigned int crcpoly = 0x80001B;
	u32 result;
	u32 data_word;

	data_word = (secondbyte << 8) | firstbyte;
	result = ((*crc << 1) ^ data_word);

	if (result & 0x1000000)
		result ^= crcpoly;

	*crc = result;
}

static u32 mxt_calculate_crc(u8 *base, off_t start_off, off_t end_off)
{
	u32 crc = 0;
	u8 *ptr = base + start_off;
	u8 *last_val = base + end_off - 1;

	if (end_off < start_off)
		return -EINVAL;

	while (ptr < last_val) {
		mxt_calc_crc24(&crc, *ptr, *(ptr + 1));
		ptr += 2;
	}

	/* if len is odd, fill the last byte with 0 */
	if (ptr == last_val)
		mxt_calc_crc24(&crc, *ptr, 0);

	/* Mask to 24-bit */
	crc &= 0x00FFFFFF;

	return crc;
}

static int mxt_init_t7_power_cfg(struct mxt_data *data);


static int mxt_set_t7_power_cfg(struct mxt_data *data, u8 sleep)
{
	struct device *dev = &data->client->dev;
	int error;
	struct t7_config *new_config;
	struct t7_config deepsleep = { .active = 0, .idle = 0 };

	if (sleep == MXT_POWER_CFG_DEEPSLEEP)
		new_config = &deepsleep;
	else
		new_config = &data->t7_cfg;

	error = __mxt_write_reg(data->client, data->T7_address,
			sizeof(data->t7_cfg),
			new_config);
	if (error)
		return error;

	dev_dbg(dev, "Set T7 ACTV:%d IDLE:%d\n",
		new_config->active, new_config->idle);

	return 0;
}

static int mxt_init_t7_power_cfg(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int error;
	bool retry = false;

recheck:
	error = __mxt_read_reg(data->client, data->T7_address,
				sizeof(data->t7_cfg), &data->t7_cfg);
	if (error)
		return error;

	if (data->t7_cfg.active == 0 || data->t7_cfg.idle == 0) {
		if (!retry) {
			dev_info(dev, "T7 cfg zero, resetting\n");
			mxt_soft_reset(data);
			retry = true;
			goto recheck;
		} else {
		    dev_dbg(dev, "T7 cfg zero after reset, overriding\n");
		    data->t7_cfg.active = 20;
		    data->t7_cfg.idle = 100;
		    return mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);
		}
	} else {
		dev_info(dev, "Initialised power cfg: ACTV %d, IDLE %d\n",
				data->t7_cfg.active, data->t7_cfg.idle);
		return 0;
	}
}

static int mxt_acquire_irq(struct mxt_data *data)
{
	int error;

	touch_enable_irq(data->irq);

	if (data->use_retrigen_workaround) {
		error = mxt_process_messages_until_invalid(data);
		if (error)
			return error;
	}

	return 0;
}

static void mxt_free_input_device(struct mxt_data *data)
{
	if (data->input_dev) {
		input_unregister_device(data->input_dev);
		data->input_dev = NULL;
	}
}

static void mxt_free_object_table(struct mxt_data *data)
{
	kfree(data->raw_info_block);
	data->object_table = NULL;
	data->info = NULL;
	data->raw_info_block = NULL;
	kfree(data->msg_buf);
	data->msg_buf = NULL;

	mxt_free_input_device(data);

	data->enable_reporting = false;
	data->T5_address = 0;
	data->T5_msg_size = 0;
	data->T6_reportid = 0;
	data->T7_address = 0;
	data->T9_reportid_min = 0;
	data->T9_reportid_max = 0;
	data->T15_reportid_min = 0;
	data->T15_reportid_max = 0;
	data->T18_address = 0;
	data->T19_reportid = 0;
	data->T42_reportid_min = 0;
	data->T42_reportid_max = 0;
	data->T44_address = 0;
	data->T48_reportid = 0;
	data->T57_reportid_min = 0;
	data->T57_reportid_max =0;
	data->T61_reportid_min = 0;
	data->T61_reportid_max = 0;
	data->T100_reportid_min = 0;
	data->T100_reportid_max = 0;
	data->T24_reportid = 0;
	data->T25_reportid = 0;
	data->max_reportid = 0;
}

static int mxt_parse_object_table(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int i;
	u8 reportid;
	u16 end_address;

	/* Valid Report IDs start counting from 1 */
	reportid = 1;
	data->mem_size = 0;
	for (i = 0; i < data->info->object_num; i++) {
		struct mxt_object *object = data->object_table + i;
		u8 min_id, max_id;

		le16_to_cpus(&object->start_address);

		if (object->num_report_ids) {
			min_id = reportid;
			reportid += object->num_report_ids *
					mxt_obj_instances(object);
			max_id = reportid - 1;
		} else {
			min_id = 0;
			max_id = 0;
		}

		dev_dbg(&data->client->dev,
			"T%u Start:%u Size:%u Instances:%u Report IDs:%u-%u\n",
			object->type, object->start_address,
			mxt_obj_size(object), mxt_obj_instances(object),
			min_id, max_id);

		switch (object->type) {
		case MXT_GEN_MESSAGE_T5:
			if (data->info->family_id == 0x80) {
				/* On mXT224 read and discard unused CRC byte
				 * otherwise DMA reads are misaligned */
				data->T5_msg_size = mxt_obj_size(object);
			} else {
				/* CRC not enabled, so skip last byte */
				data->T5_msg_size = mxt_obj_size(object) - 1;
			}
			data->T5_address = object->start_address;
		case MXT_GEN_COMMAND_T6:
			data->T6_reportid = min_id;
			data->T6_address = object->start_address;
			break;
		case MXT_GEN_POWER_T7:
			data->T7_address = object->start_address;
			break;
		case MXT_TOUCH_MULTI_T9:
			/* Only handle messages from first T9 instance */
			data->T9_reportid_min = min_id;
			data->T9_reportid_max = min_id +
						object->num_report_ids - 1;
			data->num_touchids = object->num_report_ids;
			break;
		case MXT_TOUCH_KEYARRAY_T15:
			data->T15_reportid_min = min_id;
			data->T15_reportid_max = max_id;
			break;
		case MXT_SPT_COMMSCONFIG_T18:
			data->T18_address = object->start_address;
			break;
		case MXT_PROCI_TOUCHSUPPRESSION_T42:
			data->T42_reportid_min = min_id;
			data->T42_reportid_max = max_id;
			break;
		case MXT_SPT_MESSAGECOUNT_T44:
			data->T44_address = object->start_address;
			break;
		case MXT_SPT_GPIOPWM_T19:
			data->T19_reportid = min_id;
			break;
		case MXT_SPT_NOISESUPPRESSION_T48:
			data->T48_reportid = min_id;
			break;
		case MXT_PROCI_EXTRATOUCHSCREENDATA_T57:
			data->T57_reportid_min = min_id;
			data->T57_reportid_max = max_id;
			break;
		case MXT_SPT_TIMER_T61:
			data->T61_reportid_min = min_id;
			data->T61_reportid_max = max_id;
			break;
		case MXT_TOUCH_MULTITOUCHSCREEN_T100:
			data->T100_reportid_min = min_id;
			data->T100_reportid_max = max_id;
			/* first two report IDs reserved */
			data->num_touchids = object->num_report_ids - 2;
			break;
		case MXT_PROCI_ONETOUCH_T24:
			data->T24_reportid = min_id;
			break;
		case MXT_SPT_SELFTEST_T25:
			data->T25_reportid = min_id;
			break;
		}

		end_address = object->start_address
			+ mxt_obj_size(object) * mxt_obj_instances(object) - 1;

		if (end_address >= data->mem_size)
			data->mem_size = end_address + 1;
	}

	/* Store maximum reportid */
	data->max_reportid = reportid;

	/* If T44 exists, T5 position has to be directly after */
	if (data->T44_address && (data->T5_address != data->T44_address + 1)) {
		dev_err(&client->dev, "Invalid T44 position\n");
		return -EINVAL;
	}

	data->msg_buf = kcalloc(data->max_reportid,
				data->T5_msg_size, GFP_KERNEL);
	if (!data->msg_buf) {
		dev_err(&client->dev, "Failed to allocate message buffer\n");
		return -ENOMEM;
	}

	return 0;
}

static int mxt_read_info_block(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	size_t size;
	void *buf;
	struct mxt_info *info;
	u32 calculated_crc;
	u8 *crc_ptr;

	/* If info block already allocated, free it */
	if (data->raw_info_block != NULL)
		mxt_free_object_table(data);

	/* Read 7-byte ID information block starting at address 0 */
	size = sizeof(struct mxt_info);
	buf = kzalloc(size, GFP_KERNEL);
	if (!buf) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	error = __mxt_read_reg(client, 0, size, buf);
	if (error)
		goto err_free_mem;

	/* Resize buffer to give space for rest of info block */
	info = (struct mxt_info *)buf;
	size += (info->object_num * sizeof(struct mxt_object))
		+ MXT_INFO_CHECKSUM_SIZE;

	buf = krealloc(buf, size, GFP_KERNEL);
	if (!buf) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	/* Read rest of info block */
	error = __mxt_read_reg(client, MXT_OBJECT_START,
			       size - MXT_OBJECT_START,
			       buf + MXT_OBJECT_START);
	if (error)
		goto err_free_mem;

	/* Extract & calculate checksum */
	crc_ptr = buf + size - MXT_INFO_CHECKSUM_SIZE;
	data->info_crc = crc_ptr[0] | (crc_ptr[1] << 8) | (crc_ptr[2] << 16);

	calculated_crc = mxt_calculate_crc(buf, 0,
					   size - MXT_INFO_CHECKSUM_SIZE);

	/* CRC mismatch can be caused by data corruption due to I2C comms
	 * issue or else device is not using Object Based Protocol */
	if (data->info_crc != calculated_crc) {
		dev_err(&client->dev,
			"Info Block CRC error calculated=0x%06X read=0x%06X\n",
			data->info_crc, calculated_crc);
		return -EIO;
	}

	/* Save pointers in device data structure */
	data->raw_info_block = buf;
	data->info = (struct mxt_info *)buf;
	data->object_table = (struct mxt_object *)(buf + MXT_OBJECT_START);

	dev_info(&client->dev,
		 "Family: %02X Variant: %02X Firmware V%u.%u.%02X Objects: %u\n",
		 data->info->family_id, data->info->variant_id, data->info->version >> 4,
		 data->info->version & 0xf, data->info->build, data->info->object_num);

	/* Parse object table information */
	error = mxt_parse_object_table(data);
	if (error) {
		dev_err(&client->dev, "Error %d reading object table\n", error);
		mxt_free_object_table(data);
		return error;
	}

	return 0;

err_free_mem:
	kfree(buf);
	return error;
}

static int mxt_read_t9_resolution(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	struct t9_range range;
	unsigned char orient;
	struct mxt_object *object;

	object = mxt_get_object(data, MXT_TOUCH_MULTI_T9);
	if (!object)
		return -EINVAL;

	error = __mxt_read_reg(client,
			       object->start_address + MXT_T9_RANGE,
			       sizeof(range), &range);
	if (error)
		return error;

	le16_to_cpus(range.x);
	le16_to_cpus(range.y);

	error =  __mxt_read_reg(client,
				object->start_address + MXT_T9_ORIENT,
				1, &orient);
	if (error)
		return error;

	/* Handle default values */
	if (range.x == 0)
		range.x = 1023;

	if (range.y == 0)
		range.y = 1023;

	if (orient & MXT_T9_ORIENT_SWITCH) {
		data->max_x = range.y;
		data->max_y = range.x;
	} else {
		data->max_x = range.x;
		data->max_y = range.y;
	}

	dev_info(&client->dev,
		 "Touchscreen size X%uY%u\n", data->max_x, data->max_y);

	return 0;
}

static int mxt_check_retrigen(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	int val;

	if (data->pdata->irqflags & IRQF_TRIGGER_LOW)
		return 0;

	if (data->T18_address) {
		error = __mxt_read_reg(client,
				       data->T18_address + MXT_COMMS_CTRL,
				       1, &val);
		if (error)
			return error;

		if (val & MXT_COMMS_RETRIGEN)
			return 0;
	}

	dev_warn(&client->dev, "Enabling RETRIGEN workaround\n");
	data->use_retrigen_workaround = true;
	return 0;
}

static void mxt_regulator_enable(struct mxt_data *data)
{
	gpio_set_value(data->pdata->gpio_reset, 0);

//	gpio_set_value(data->pdata->gpio_ldo_en, 1);

	regulator_enable(data->vcc_i2c);
	regulator_enable(data->vdd_ana);

	msleep(MXT_REGULATOR_DELAY);

	gpio_set_value(data->pdata->gpio_reset, 1);
	msleep(MXT_POWERON_DELAY); /* Minimum 65ms, recommend 100ms from ATMEL */
}

static void mxt_regulator_disable(struct mxt_data *data)
{
//	gpio_set_value(data->pdata->gpio_ldo_en, 0);
	regulator_disable(data->vcc_i2c);
	regulator_disable(data->vdd_ana);
	gpio_set_value(data->pdata->gpio_reset, 0);
}

static void mxt_probe_regulators(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int error;

	/* According to maXTouch power sequencing specification, RESET line
	 * must be kept low until some time after regulators come up to
	 * voltage */
	if (!data->pdata->gpio_reset) {
		dev_warn(dev, "Must have reset GPIO to use regulator support\n");
		goto fail;
	}

	data->vcc_i2c = regulator_get(dev, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c)) {
		error = PTR_ERR(data->vcc_i2c);
		dev_err(dev, "Error %d getting i2c regulator\n", error);
		goto fail_release;
	}

	data->vdd_ana = regulator_get(dev, "vdd_ana");
	if (IS_ERR(data->vdd_ana)) {
		error = PTR_ERR(data->vdd_ana);
		dev_err(dev, "Error %d getting ana regulator\n", error);
		goto fail_release;
	}

	error = regulator_set_voltage(data->vdd_ana, 3050000, 3050000);
	if (error < 0) {
		dev_err(dev, "Error %d cannot control ana regulator\n", error);
		goto fail_release;
	}



	data->use_regulator = true;
	mxt_regulator_enable(data);

	dev_info(dev, "Initialized regulators\n");
	return;

fail_release:
	regulator_put(data->vcc_i2c);
	regulator_put(data->vdd_ana);
//	gpio_set_value(data->pdata->gpio_ldo_en, 0);
fail:
	data->vcc_i2c = NULL;
	data->vdd_ana = NULL;
	data->use_regulator = false;
}

static int mxt_read_t100_config(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	struct mxt_object *object;
	u16 range_x, range_y;
	u8 cfg, tchaux;
	u8 aux;

	object = mxt_get_object(data, MXT_TOUCH_MULTITOUCHSCREEN_T100);
	if (!object)
		return -EINVAL;

	error = __mxt_read_reg(client,
			       object->start_address + MXT_T100_XRANGE,
			       sizeof(range_x), &range_x);
	if (error)
		return error;

	le16_to_cpus(range_x);

	error = __mxt_read_reg(client,
			       object->start_address + MXT_T100_YRANGE,
			       sizeof(range_y), &range_y);
	if (error)
		return error;

	le16_to_cpus(range_y);

	error =  __mxt_read_reg(client,
				object->start_address + MXT_T100_CFG1,
				1, &cfg);
	if (error)
		return error;

	error =  __mxt_read_reg(client,
				object->start_address + MXT_T100_TCHAUX,
				1, &tchaux);
	if (error)
		return error;

	/* Handle default values */
	if (range_x == 0)
		range_x = 1023;

	/* Handle default values */
	if (range_x == 0)
		range_x = 1023;

	if (range_y == 0)
		range_y = 1023;

	if (cfg & MXT_T100_CFG_SWITCHXY) {
		data->max_x = range_y;
		data->max_y = range_x;
	} else {
		data->max_x = range_x;
		data->max_y = range_y;
	}

	/* allocate aux bytes */
	aux = 6;

	if (tchaux & MXT_T100_TCHAUX_VECT)
		data->t100_aux_vect = aux++;

	if (tchaux & MXT_T100_TCHAUX_AMPL)
		data->t100_aux_ampl = aux++;

	if (tchaux & MXT_T100_TCHAUX_AREA)
		data->t100_aux_area = aux++;

	dev_info(&client->dev,
		 "T100 Touchscreen size X%uY%u\n", data->max_x, data->max_y);

	return 0;
}

static int mxt_input_open(struct input_dev *dev);
static void mxt_input_close(struct input_dev *dev);

static int mxt_initialize_t100_input_device(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	struct input_dev *input_dev;
	int error;

	error = mxt_read_t100_config(data);
	if (error)
		dev_warn(dev, "Failed to initialize T9 resolution\n");

	input_dev = input_allocate_device();
	if (!data || !input_dev) {
		dev_err(dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	input_dev->name = "atmel_mxt_ts T100 touchscreen";

	input_dev->phys = data->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &data->client->dev;
	input_dev->open = mxt_input_open;
	input_dev->close = mxt_input_close;

	set_bit(EV_ABS, input_dev->evbit);
	input_set_capability(input_dev, EV_KEY, BTN_TOUCH);

	/* For single touch */
	input_set_abs_params(input_dev, ABS_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			     0, data->max_y, 0, 0);

	if (data->t100_aux_ampl)
		input_set_abs_params(input_dev, ABS_PRESSURE,
				     0, 255, 0, 0);

	/* For multi touch */
	error = input_mt_init_slots(input_dev, data->num_touchids);
	if (error) {
		dev_err(dev, "Error %d initialising slots\n", error);
		goto err_free_mem;
	}

	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, data->max_y, 0, 0);

	if (data->t100_aux_area)
		input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
				     0, MXT_MAX_AREA, 0, 0);

	if (data->t100_aux_ampl)
		input_set_abs_params(input_dev, ABS_MT_PRESSURE,
				     0, 255, 0, 0);

	if (data->t100_aux_vect)
		input_set_abs_params(input_dev, ABS_MT_ORIENTATION,
				     0, 255, 0, 0);

	input_set_drvdata(input_dev, data);

	error = input_register_device(input_dev);
	if (error) {
		dev_err(dev, "Error %d registering input device\n", error);
		goto err_free_mem;
	}

	data->input_dev = input_dev;

	return 0;

err_free_mem:
	input_free_device(input_dev);
	return error;
}

static int mxt_initialize_t9_input_device(struct mxt_data *data);
static int mxt_configure_objects(struct mxt_data *data);

static int mxt_initialize(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	u8 retry_count = 0;

retry_probe:
	error = mxt_read_info_block(data);
	if (error) {
		error = mxt_probe_bootloader(data, retry_count);
		if (error) {
			if (++retry_count > 6)
				/* Chip is not in appmode or bootloader mode */
				return error;

			goto retry_probe;
		} else {
#ifdef FIRMUP_ON_PROBE
			dev_err(&client->dev, "IC stay in bootloader mode\n");
			data->in_bootloader = true;
			return 0;
#else
			if (++retry_count > 10) {
				dev_err(&client->dev,
						"Could not recover device from "
						"bootloader mode\n");
				/* this is not an error state, we can reflash
				 * from here */
				data->in_bootloader = true;
				return 0;
			}

			/* Attempt to exit bootloader into app mode */
			mxt_send_bootloader_cmd(data, false);
			msleep(MXT_FW_RESET_TIME);
			goto retry_probe;
#endif
		}
	}

	return 0;
}

static int mxt_rest_init(struct mxt_data *data)
{
	int error;
	struct mxt_object *object;

	data->enable_reporting = false;
	mxt_free_input_device(data);

	error = mxt_acquire_irq(data);
	if (error)
		return error;

	error = mxt_configure_objects(data);
	if (error)
		return error;
	/* T77 Object disable after Back-up & Reset */
	object = mxt_get_object(data, MXT_SPT_CTESCANCONFIG_T77);
	if (!object) {
		return -EINVAL;
	}
	error = mxt_write_reg(data->client, object->start_address, 8);
	if (error)
		return error;

	return 0;
}

static int mxt_configure_objects(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;

	error = mxt_init_t7_power_cfg(data);
	if (error) {
		dev_err(&client->dev, "Failed to initialize power cfg\n");
		return error;
	}

#ifdef FIRMUP_ON_PROBE
	mxt_update_crc(data, MXT_COMMAND_REPORTALL, 1);

	if (data->config_crc == 0 || data->config_crc != MXT_LATEST_CONFIG_CRC) {
		//Disable T70 Dynamic configuration
		mxt_t6_command(data, MXT_COMMAND_BACKUPNV, MXT_STOP_DYNAMIC_CONFIG, false);

		error = mxt_write_configuration(data);
		if (error) {
			dev_err(&data->client->dev, "Failed to write config\n");

			error = mxt_soft_reset(data);
			if (error) {
				dev_err(&data->client->dev, "Failed to reset IC\n");
			}
		} else {
			error = mxt_backup(data);
			if (error) {
				dev_err(&data->client->dev, "Failed to backup NV data\n");
			}
		}
	}
#endif

	error = mxt_check_retrigen(data);
	if (error)
		return error;

	if (data->T9_reportid_min) {
		error = mxt_initialize_t9_input_device(data);
		if (error)
			return error;
	} else if (data->T100_reportid_min) {
		error = mxt_initialize_t100_input_device(data);
		if (error)
			return error;
	} else {
		dev_warn(&client->dev, "No touch object detected\n");
	}

	//data->enable_reporting = true;
	return 0;
}

/* Firmware Version is returned as Major.Minor.Build */
static ssize_t mxt_fw_version_show(struct mxt_data *data, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u.%u.%02X\n",
			 data->info->version >> 4, data->info->version & 0xf,
			 data->info->build);
}

/* Hardware Version is returned as FamilyID.VariantID */
static ssize_t mxt_hw_version_show(struct mxt_data *data, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%u.%u\n",
			data->info->family_id, data->info->variant_id);
}

static ssize_t mxt_show_instance(char *buf, int count,
				 struct mxt_object *object, int instance,
				 const u8 *val)
{
	int i;

	if (mxt_obj_instances(object) > 1)
		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "Instance %u\n", instance);

	for (i = 0; i < mxt_obj_size(object); i++)
		count += scnprintf(buf + count, PAGE_SIZE - count,
				"\t[%2u]: %02x (%d)\n", i, val[i], val[i]);
	count += scnprintf(buf + count, PAGE_SIZE - count, "\n");

	return count;
}

static ssize_t mxt_object_show(struct mxt_data *data, char *buf)
{
	struct mxt_object *object;
	int count = 0;
	int i, j;
	int error;
	u8 *obuf;

	/* Pre-allocate buffer large enough to hold max sized object. */
	obuf = kmalloc(256, GFP_KERNEL);
	if (!obuf)
		return -ENOMEM;

	error = 0;
	for (i = 0; i < data->info->object_num; i++) {
		object = data->object_table + i;

		if (!mxt_object_readable(object->type))
			continue;

		count += scnprintf(buf + count, PAGE_SIZE - count,
				"T%u:\n", object->type);

		for (j = 0; j < mxt_obj_instances(object); j++) {
			u16 size = mxt_obj_size(object);
			u16 addr = object->start_address + j * size;

			error = __mxt_read_reg(data->client, addr, size, obuf);
			if (error)
				goto done;

			count = mxt_show_instance(buf, count, object, j, obuf);
		}
	}

done:
	kfree(obuf);
	return error ?: count;
}

static int mxt_check_firmware_format(struct device *dev,
				     const struct firmware *fw)
{
	unsigned int pos = 0;
	char c;

	while (pos < fw->size) {
		c = *(fw->data + pos);

		if (c < '0' || (c > '9' && c < 'A') || c > 'F')
			return 0;

		pos++;
	}

	/* To convert file try
	 * xxd -r -p mXTXXX__APP_VX-X-XX.enc > maxtouch.fw */
	dev_err(dev, "Aborting: firmware file must be in binary format\n");

	return -1;
}

static int mxt_load_fw(struct device *dev, bool from_header)
{
	struct mxt_data *data = dev_get_drvdata(dev);
    const struct firmware *fw = NULL;
#ifdef FIRMUP_ON_PROBE
    struct firmware *fw_from_header = NULL;
#endif
	unsigned int frame_size;
	unsigned int pos = 0;
	unsigned int retry = 0;
	unsigned int frame = 0;
	int ret;

#ifdef FIRMUP_ON_PROBE
    if (from_header) {
		fw_from_header = kzalloc(sizeof(struct firmware), GFP_KERNEL);

		fw_from_header->data = latest_firmware;
		fw_from_header->size = sizeof(latest_firmware);
		fw = fw_from_header;
    } else {
		ret = request_firmware(&fw, data->fw_name, dev);
		if (ret) {
			dev_err(dev, "Unable to open firmware %s  ret %d\n", data->fw_name, ret);
			return ret;
		}
    }
#else
    ret = request_firmware(&fw, data->fw_name, dev);
    if (ret) {
		dev_err(dev, "Unable to open firmware %s  ret %d\n", data->fw_name, ret);
		return ret;
    }
#endif

	/* Check for incorrect enc file */
	ret = mxt_check_firmware_format(dev, fw);
	if (ret)
		goto release_firmware;

	if (data->suspended) {
		if (data->use_regulator)
			mxt_regulator_enable(data);

		touch_enable_irq(data->irq);
		data->suspended = false;
	}

	if (!data->in_bootloader) {
		/* Change to the bootloader mode */
		data->in_bootloader = true;

		ret = mxt_t6_command(data, MXT_COMMAND_RESET,
				     MXT_BOOT_VALUE, false);
		if (ret)
			goto release_firmware;

		msleep(MXT_RESET_TIME);

		/* At this stage, do not need to scan since we know
		 * family ID */
		ret = mxt_lookup_bootloader_address(data, 0);
		if (ret)
			goto release_firmware;
	}

	mxt_free_object_table(data);
	INIT_COMPLETION(data->bl_completion);

	ret = mxt_check_bootloader(data, MXT_WAITING_BOOTLOAD_CMD);
	if (ret) {
		/* Bootloader may still be unlocked from previous update
		 * attempt */
		ret = mxt_check_bootloader(data, MXT_WAITING_FRAME_DATA);
		if (ret)
			goto disable_irq;
	} else {
		dev_info(dev, "Unlocking bootloader\n");

		/* Unlock bootloader */
		ret = mxt_send_bootloader_cmd(data, true);
		if (ret)
			goto disable_irq;
	}

	while (pos < fw->size) {
		ret = mxt_check_bootloader(data, MXT_WAITING_FRAME_DATA);
		if (ret)
			goto disable_irq;

		frame_size = ((*(fw->data + pos) << 8) | *(fw->data + pos + 1));

		/* Take account of CRC bytes */
		frame_size += 2;

		/* Write one frame to device */
		ret = mxt_bootloader_write(data, fw->data + pos, frame_size);
		if (ret)
			goto disable_irq;

		ret = mxt_check_bootloader(data, MXT_FRAME_CRC_PASS);
		if (ret) {
			retry++;

			/* Back off by 20ms per retry */
			msleep(retry * 20);

			if (retry > 20) {
				dev_err(dev, "Retry count exceeded\n");
				goto disable_irq;
			}
		} else {
			retry = 0;
			pos += frame_size;
			frame++;
		}

		if (frame % 50 == 0)
			dev_info(dev, "Sent %d frames, %d/%zd bytes\n",
				 frame, pos, fw->size);
	}

	/* Wait for flash */
	ret = mxt_wait_for_completion(data, &data->bl_completion,
				      MXT_FW_RESET_TIME);
	if (ret)
		goto disable_irq;

	dev_info(dev, "Sent %d frames, %zd bytes\n", frame, pos);

#if 0 /* To avoid reset timeout */
	/* Wait for device to reset */
	ret = mxt_wait_for_completion(data, &data->bl_completion,
				      MXT_FW_RESET_TIME);
	if (ret)
		dev_err(dev, "Device didn't reset\n");
#endif

	data->in_bootloader = false;

disable_irq:
	touch_disable_irq(data->irq);
release_firmware:
#ifdef FIRMUP_ON_PROBE
if (from_header)
    kfree(fw_from_header);
else
#endif
	release_firmware(fw);
	return ret;
}

static int mxt_update_file_name(struct device *dev, char **file_name,
				const char *buf, size_t count)
{
	char *file_name_tmp;

	/* Simple sanity check */
	if (count > 64) {
		dev_warn(dev, "File name too long\n");
		return -EINVAL;
	}

	file_name_tmp = krealloc(*file_name, count + 1, GFP_KERNEL);
	if (!file_name_tmp) {
		dev_warn(dev, "no memory\n");
		return -ENOMEM;
	}

	*file_name = file_name_tmp;
	memcpy(*file_name, buf, count);

	/* Echo into the sysfs entry may append newline at the end of buf */
	if (buf[count - 1] == '\n')
		(*file_name)[count - 1] = '\0';
	else
		(*file_name)[count] = '\0';

	return 0;
}

#ifdef FIRMUP_ON_PROBE
/*
*   Update firmware from header file
*/
static ssize_t mxt_firmware_update(struct mxt_data *data)
{
    struct device *dev = &data->client->dev;
    int error;

    error = mxt_load_fw(dev, true);
    if (error) {
		dev_err(dev, "The firmware update failed(%d)\n", error);
    } else {
		dev_info(dev, "The firmware update succeeded\n");

		data->suspended = false;

		error = mxt_read_info_block(data);
    }

    return error;
}
#endif

static ssize_t mxt_update_fw_store(struct mxt_data *data,
					const char *buf, size_t count)
{
	int error;

	error = mxt_update_file_name(&data->client->dev, &data->fw_name, buf, count);
	if (error)
		return error;

	error = mxt_load_fw(&data->client->dev, false);
	if (error) {
		dev_err(&data->client->dev, "The firmware update failed(%d)\n", error);
		count = error;
	} else {
		dev_info(&data->client->dev, "The firmware update succeeded\n");

		data->suspended = false;

		error = mxt_read_info_block(data);
		if (error)
			return error;

		error = mxt_rest_init(data);
		if (error)
			return error;

		data->enable_reporting = true;
	}

	return count;
}

static ssize_t mxt_update_cfg_store(struct mxt_data *data,
		const char *buf, size_t count)
{
	int ret;

	if (data->in_bootloader) {
		dev_err(&data->client->dev, "Not in appmode\n");
		return -EINVAL;
	}

	ret = mxt_update_file_name(&data->client->dev, &data->cfg_name, buf, count);
	if (ret)
		return ret;

	data->enable_reporting = false;
	mxt_free_input_device(data);

	if (data->suspended) {
		if (data->use_regulator)
			mxt_regulator_enable(data);
		else
			mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);

		mxt_acquire_irq(data);

		data->suspended = false;
	}

	ret = mxt_configure_objects(data);
	if (ret)
		goto out;

	ret = count;
out:
	return ret;
}

static ssize_t mxt_debug_enable_show(struct mxt_data *data, char *buf)
{
	int count;
	char c;

	c = data->debug_enabled ? '1' : '0';
	count = sprintf(buf, "%c\n", c);

	return count;
}

static ssize_t mxt_debug_enable_store(struct mxt_data *data,
	const char *buf, size_t count)
{
	int i;

	if (sscanf(buf, "%u", &i) == 1 && i < 2) {
		data->debug_enabled = (i == 1);

		dev_dbg(&data->client->dev, "%s\n", i ? "debug enabled" : "debug disabled");
		return count;
	} else {
		dev_dbg(&data->client->dev, "debug_enabled write error\n");
		return -EINVAL;
	}
}

static int mxt_check_mem_access_params(struct mxt_data *data, loff_t off,
				       size_t *count)
{
	if (off >= data->mem_size)
		return -EIO;

	if (off + *count > data->mem_size)
		*count = data->mem_size - off;

	if (*count > MXT_MAX_BLOCK_WRITE)
		*count = MXT_MAX_BLOCK_WRITE;

	return 0;
}

static ssize_t mxt_mem_access_read(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret = 0;

	ret = mxt_check_mem_access_params(data, off, &count);
	if (ret < 0)
		return ret;

	if (count > 0)
		ret = __mxt_read_reg(data->client, off, count, buf);

	return ret == 0 ? count : ret;
}

static ssize_t mxt_mem_access_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off,
	size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret = 0;

	ret = mxt_check_mem_access_params(data, off, &count);
	if (ret < 0)
		return ret;

	if (count > 0)
		ret = __mxt_write_reg(data->client, off, count, buf);

	return ret == 0 ? count : 0;
}

static ssize_t mxt_change_cfg_store(struct mxt_data *data,
	const char *buf, size_t count)
{
	int ret = 0;
	int target_obj, val_obj, num_obj;
	int val_size = 1;

	struct mxt_object *object = NULL;

	ret = sscanf(buf, "%d %d %d", &target_obj, &num_obj, &val_obj);

	object = mxt_get_object(data, target_obj);
	if (!object)
		return -EINVAL;

	ret = mxt_check_mem_access_params(data, object->start_address, &val_size);
	if (ret < 0)
		return ret;
	if (count > 0)
		ret = mxt_write_reg(data->client, object->start_address + num_obj, val_obj);

	return ret == 0 ? count : 0;
}

static ssize_t mxt_object_num_show(struct mxt_data *data, char *buf)
{
	struct mxt_object *object = NULL;
	int count = 0;
	int error = 0;
	int i;
	u8 *obuf;

	/* Pre-allocate buffer large enough to hold max sized object. */
	obuf = kmalloc(256, GFP_KERNEL);
	if (!obuf)
		return -ENOMEM;

	if (!data->object_num)
		data->object_num = 9;

	object = mxt_get_object(data, data->object_num);
	if (!object)
		return -EINVAL;

	if (!mxt_object_readable(object->type)) {
		dev_err(&data->client->dev, "Cannot read T%u object\n", object->type);
		return -EINVAL;
	}

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"T%u:\n", object->type);

	for (i = 0; i < mxt_obj_instances(object); i++) {
		u16 size = mxt_obj_size(object);
		u16 addr = object->start_address + i * size;

		error = __mxt_read_reg(data->client, addr, size, obuf);
		if (error)
			goto done;

		count = mxt_show_instance(buf, count, object, i, obuf);
	}

done:
	kfree(obuf);
	return error ?: count;
}

static ssize_t mxt_object_num_store(struct mxt_data *data,
	const char *buf, size_t count)
{
	int i;

	if (sscanf(buf, "%u", &i) == 1 && i >0 && i<101) {
		if (!mxt_object_readable(i)) {
			data->object_num = 0;
			return -EINVAL;
		}
		data->object_num = i;
		return count;
	} else {
		data->object_num = 0;
		return -EINVAL;
	}
}

static ssize_t mxt_knock_on_store(struct mxt_data *data,
	const char *buf, size_t size)
{

	dev_dbg(&data->client->dev, "%s: start\n", __func__);

	if (sysfs_streq(buf, "1")) {
		data->knock_on_enable = true;
		dev_info(&data->client->dev, "Knock On Enabled\n");
		device_init_wakeup(&data->client->dev, 1);
	} else if (sysfs_streq(buf, "0")) {
		data->knock_on_enable = false;
		dev_info(&data->client->dev, "Knock On Disabled\n");
		device_init_wakeup(&data->client->dev, 0);
	} else {
		dev_err(&data->client->dev,
			"illegal command in %s, buf is: %s\n",
			buf, __func__);
	}

	return strnlen(buf, PAGE_SIZE);
}

static ssize_t mxt_smart_cover_show(struct mxt_data *data, char *buf)
{
	int count;
	char c;

	c = data->smart_cover_enable ? '1' : '0';
	count = sprintf(buf, "%c\n", c);

	return count;
}

static ssize_t mxt_smart_cover_store(struct mxt_data *data,
	const char *buf, size_t count)
{
	int i;

	if (sscanf(buf, "%u", &i) == 1 && i < 2) {
		data->smart_cover_enable = (i == 1);

		dev_info(&data->client->dev, "%s\n", i ? "smart cover enabled" : "smart cover disabled");
		return count;
	} else {
		dev_info(&data->client->dev, "smart_cover_enable write error\n");
		return -EINVAL;
	}
}

static ssize_t mxt_show_self_test(struct mxt_data *data, char *buf)
{
	struct mxt_object *object = NULL;

	int ret = 0;
	int test_cmd = 254; // TEST ALL

	object = mxt_get_object(data, MXT_SPT_SELFTEST_T25);
	if (!object)
		return -EINVAL;

	ret = mxt_write_reg(data->client, object->start_address+1, test_cmd);
	if(ret)
		return 0;

	ret = mxt_wait_for_completion(data, &data->t25_completion, MXT_SELFTEST_TIME);
	if(ret)
		ret = sprintf(buf, "ATMEL %s Self Test cannot be run\n", MXT_DEVICE_NAME);
	else
		ret = sprintf(buf, "ATMEL %s Self Test Result: %s\n",
			MXT_DEVICE_NAME,(data->self_test_result > 0) ? "Pass\n" : "Fail\n");

	return ret;
}

static ssize_t mxt_chargerlogo_state_store(struct mxt_data *data,
	const char *buf, size_t count)
{
	int i;
	int ret = 0;

	ret = sscanf(buf, "%u", &i);
	if (i== 1 && i < 2) {
		dev_info(&data->client->dev, "Boot chargerlogo mode\n");
		data->chargerlogo = true;
		mxt_stop(data);
		return ret;
	} else {
		dev_info(&data->client->dev, "Boot normal mode\n");
		data->chargerlogo = false;
		return -EINVAL;
	}
}

static LGE_TOUCH_ATTR(fw_ver, S_IRUGO, mxt_fw_version_show, NULL);
static LGE_TOUCH_ATTR(hw_version, S_IRUGO, mxt_hw_version_show, NULL);
static LGE_TOUCH_ATTR(object, S_IRUGO, mxt_object_show, NULL);
static LGE_TOUCH_ATTR(update_fw, S_IWUSR, NULL, mxt_update_fw_store);
static LGE_TOUCH_ATTR(update_cfg, S_IWUSR, NULL, mxt_update_cfg_store);
static LGE_TOUCH_ATTR(debug_enable, S_IWUSR | S_IRUSR, mxt_debug_enable_show,
		   mxt_debug_enable_store);
static LGE_TOUCH_ATTR(change_cfg, S_IWUSR, NULL, mxt_change_cfg_store);
static LGE_TOUCH_ATTR(object_num, S_IRUGO | S_IWUSR, mxt_object_num_show, mxt_object_num_store);
static LGE_TOUCH_ATTR(touch_gesture,S_IRUGO | S_IWUSR, NULL, mxt_knock_on_store);
static LGE_TOUCH_ATTR(smart_cover, S_IRUGO | S_IWUSR, mxt_smart_cover_show, mxt_smart_cover_store);
static LGE_TOUCH_ATTR(self_test,S_IRUGO | S_IWUSR, mxt_show_self_test, NULL);
static LGE_TOUCH_ATTR(chargerlogo,S_IRUGO | S_IWUSR, NULL, mxt_chargerlogo_state_store);

static struct attribute *mxt_attrs[] = {
	&lge_touch_attr_fw_ver.attr,
	&lge_touch_attr_hw_version.attr,
	&lge_touch_attr_object.attr,
	&lge_touch_attr_update_fw.attr,
	&lge_touch_attr_update_cfg.attr,
	&lge_touch_attr_debug_enable.attr,
	&lge_touch_attr_change_cfg.attr,
	&lge_touch_attr_object_num.attr,
	&lge_touch_attr_touch_gesture.attr,
	&lge_touch_attr_smart_cover.attr,
	&lge_touch_attr_self_test.attr,
	&lge_touch_attr_chargerlogo.attr,
	NULL
};

/* lge_touch_attr_show / lge_touch_attr_store
 *
 * sysfs bindings for lge_touch
 */
static ssize_t lge_touch_attr_show(struct kobject *lge_touch_kobj, struct attribute *attr,
			     char *buf)
{
	struct mxt_data *data =
			container_of(lge_touch_kobj, struct mxt_data, lge_touch_kobj);
	struct lge_touch_attribute *lge_touch_priv =
		container_of(attr, struct lge_touch_attribute, attr);
	ssize_t ret = 0;

	if (lge_touch_priv->show)
		ret = lge_touch_priv->show(data, buf);

	return ret;
}

static ssize_t lge_touch_attr_store(struct kobject *lge_touch_kobj, struct attribute *attr,
			      const char *buf, size_t count)
{
	struct mxt_data *data =
			container_of(lge_touch_kobj, struct mxt_data, lge_touch_kobj);
	struct lge_touch_attribute *lge_touch_priv =
		container_of(attr, struct lge_touch_attribute, attr);
	ssize_t ret = 0;

	if (lge_touch_priv->store)
		ret = lge_touch_priv->store(data, buf, count);

	return ret;
}

static const struct sysfs_ops lge_touch_sysfs_ops = {
	.show	= lge_touch_attr_show,
	.store	= lge_touch_attr_store,
};

static struct kobj_type lge_touch_kobj_type = {
	.sysfs_ops		= &lge_touch_sysfs_ops,
	.default_attrs 	= mxt_attrs,
};

static struct sysdev_class lge_touch_sys_class = {
	.name = LGE_TOUCH_NAME,
};

static struct sys_device lge_touch_sys_device = {
	.id		= 0,
	.cls	= &lge_touch_sys_class,
};

static void mxt_reset_slots(struct mxt_data *data)
{
	struct input_dev *input_dev = data->input_dev;
//	unsigned int num_mt_slots;
	int id;

	for (id = 0; id < data->pdata->numtouch; id++) {
		input_mt_slot(input_dev, id);
		if (data->ts_data.prev_data[id].tool == MT_TOOL_FINGER) {
			input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
		} else if (data->ts_data.prev_data[id].tool == MT_TOOL_PALM) {
			input_mt_report_slot_state(input_dev, MT_TOOL_PALM, 0);
		} else {
			input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, 0);
		}
	}

	mxt_input_sync(input_dev);
}

static void mxt_start(struct mxt_data *data)
{
	struct mxt_object *object = NULL;

	dev_info(&data->client->dev, "%s\n", __func__);

	if(data->chargerlogo) {
		dev_info(&data->client->dev, "%s chargerlogo\n", __func__);
		return;
	}

	if (!data->suspended || data->in_bootloader){
		dev_info(&data->client->dev, "%s suspended = %d, in_bootloader = %d\n", __func__, data->suspended , data->in_bootloader);
		return;
	}


#if 0//def CONFIG_TOUCHSCREEN_ATMEL_KNOCK_ON
	if(data->knock_on_enable && suspended_due_to_smart_cover == false) {
		touch_disable_irq(data->irq);
		mutex_lock(&i2c_suspend_lock);
		change_config(data, LGE_CFG_TYPE_ACT);
		mutex_unlock(&i2c_suspend_lock);
	} else if (data->use_regulator) {
#else
	if (data->use_regulator) {
#endif
		mxt_regulator_enable(data);
		object = mxt_get_object(data, MXT_SPT_CTESCANCONFIG_T77);
		if (object) {
			mxt_write_reg(data->client, object->start_address, 8);
		}
	} else {
		/* Discard any messages still in message buffer from before
		 * chip went to sleep */
		mxt_process_messages_until_invalid(data);

		mxt_set_t7_power_cfg(data, MXT_POWER_CFG_RUN);

		/* Recalibrate since chip has been in deep sleep */
		mxt_t6_command(data, MXT_COMMAND_CALIBRATE, 1, false);
	}
#if 0//def CHANGE_CHARGER_CFG
	if (data->charger_state == CHARGER_STATE_UPDATE_CAHRGER_CFG) {
		change_config(data, LGE_CFG_TYPE_CHARGER);
		data->charger_state = CHARGER_STATE_CONNECTED;
	} else if (data->charger_state == CHARGER_STATE_RESTORE_CFG) {
		change_config(data, LGE_CFG_TYPE_UNCHARGER);
		data->charger_state = CHARGER_STATE_DISCONNECTED;
	} else;

	mxt_t6_command(data, MXT_COMMAND_CALIBRATE, 1, false);
#endif
	data->enable_reporting = true;
	data->suspended = false;
	touch_enable_irq(data->irq);

	if (data->autocal_step == 0) {
		mxt_ghost_detection(data);
	}
}

static void mxt_stop(struct mxt_data *data)
{
//	int knock_charger = 0;

	if (data->suspended || data->in_bootloader)
		return;

	dev_info(&data->client->dev, "%s\n", __func__);

	data->enable_reporting = false;
	touch_disable_irq(data->irq);

#if 0//def CHANGE_PEN_CFG
//	change_config(data, LGE_CFG_TYPE_UNPEN);
	data->stylus_pen_cfg_adapted= false;
	data->pen_pressed_ids= 0;
#endif

#if 0//def CONFIG_TOUCHSCREEN_ATMEL_KNOCK_ON
	if (data->knock_on_enable && suspended_due_to_smart_cover == false) {
		if(data->charger_state == CHARGER_STATE_CONNECTED)
			knock_charger = LGE_CFG_TYPE_SUS_CHARGER;
		else
			knock_charger = LGE_CFG_TYPE_SUS;

		mutex_lock(&i2c_suspend_lock);
		change_config(data, knock_charger);
		mutex_unlock(&i2c_suspend_lock);
		if (data->antitouch_timer0_started)
			mxt_t61_timer_set(data, 0, MXT_T61_TIMER_CMD_STOP);
		if (data->antitouch_timer1_started)
			mxt_t61_timer_set(data, 1, MXT_T61_TIMER_CMD_STOP);
		if (data->antitouch_timer2_started)
			mxt_t61_timer_set(data, 2, MXT_T61_TIMER_CMD_STOP);
		if (data->antitouch_timer3_started)
			mxt_t61_timer_set(data, 3, MXT_T61_TIMER_CMD_STOP);
	} else if (data->use_regulator)
#else
	if (data->use_regulator)
#endif
		mxt_regulator_disable(data);
	else
		mxt_set_t7_power_cfg(data, MXT_POWER_CFG_DEEPSLEEP);

	mxt_reset_slots(data);
	data->suspended = true;
	if (data->autocal_step) {
		data->autocal_step = 0;
		data->check_antitouch = 0;
	}
#if 0//def CHANGE_CHARGER_CFG
	if (data->charger_state == CHARGER_STATE_CONNECTED) {
#ifdef CONFIG_TOUCHSCREEN_ATMEL_KNOCK_ON
		if(data->knock_on_enable && suspended_due_to_smart_cover == false)
			change_config(data, LGE_CFG_TYPE_CHARGER);
#endif
		data->charger_state = CHARGER_STATE_UPDATE_CAHRGER_CFG;
	}
#endif

#if 0//def CONFIG_TOUCHSCREEN_ATMEL_KNOCK_ON
	if (data->knock_on_enable && suspended_due_to_smart_cover == false) {
		touch_enable_irq(data->irq);
	}
#endif


}

static int mxt_input_open(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);
	dev_info(&data->client->dev, "%s\n", __func__);
	data->suspended = true;
	mxt_start(data);

	return 0;
}

static void mxt_input_close(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);

	mxt_stop(data);
}

static int mxt_parse_config(struct device *dev, struct device_node *np,
				struct mxt_config_info *info)
{
	struct property *prop;
	u8 *temp_cfg;
	u32 temp_length;
	int i = 0;

	const char *config_node_name[MXT_TMAX] = {
		"atmel,config_t37"
		,"atmel,config_t38"
		,"atmel,config_t71"
		,"atmel,config_t7"
		,"atmel,config_t8"
		,"atmel,config_t9"
		,"atmel,config_t15"
		,"atmel,config_t18"
		,"atmel,config_t19"
		,"atmel,config_t24"
		,"atmel,config_t25"
		,"atmel,config_t40"
		,"atmel,config_t42"
		,"atmel,config_t43"
		,"atmel,config_t46"
		,"atmel,config_t47"
		,"atmel,config_t55"
		,"atmel,config_t56"
		,"atmel,config_t57"
		,"atmel,config_t61"
		,"atmel,config_t62"
		,"atmel,config_t63"
		,"atmel,config_t65"
		,"atmel,config_t66"
		,"atmel,config_t70"
		,"atmel,config_t73"
		,"atmel,config_t77"
		,"atmel,config_t79"
		,"atmel,config_t82"
	};

	memset(&info->config_t, 0, sizeof(info->config_t));

	while(i < MXT_TMAX) {
		prop = of_find_property(np, config_node_name[i], &temp_length);
		if (!prop) {
			dev_err(dev, "Looking up %s property in node %s failed",
				config_node_name[i], np->full_name);
			return -ENODEV;
		} else if (!temp_length) {
			dev_err(dev, "Invalid length of %s configuration data\n", config_node_name[i]+13);
			return -EINVAL;
		}

		temp_cfg = devm_kzalloc(dev,
				temp_length * sizeof(u8), GFP_KERNEL);
		if (!temp_cfg) {
			dev_err(dev, "Unable to allocate memory to store %s cfg\n", config_node_name[i]+13);
			return -ENOMEM;
		}

		memcpy(temp_cfg, prop->value, temp_length);
		info->config_t[i] = temp_cfg;

		dev_dbg(dev, "%s config :%d\n", config_node_name[i]+13, info->config_t[i][0]);
		i++;
	}
	return 0;
}


static int mxt_parse_dt(struct device *dev, struct mxt_platform_data *pdata)
{
	struct mxt_config_info *info;
	struct device_node *temp;
	struct device_node *node = dev->of_node;
	int rc = 0;
	u32 temp_val;

	/* reset, irq gpio info */
	if (node == NULL)
		return -ENODEV;

	pdata->gpio_reset= of_get_named_gpio_flags(node, "atmel,reset-gpio", 0, NULL);
	pdata->gpio_int = of_get_named_gpio_flags(node, "atmel,irq-gpio", 0, NULL);
	pdata->gpio_ldo_en = of_get_named_gpio_flags(node, "atmel,ldo-gpio", 0, NULL);

	rc = of_property_read_u32(node, "atmel,numtouch", &temp_val);
	if(rc){
		dev_err(dev, "Unable to read numtouch\n");
		return rc;
	}else
		pdata->numtouch = temp_val;

	rc = of_property_read_u32(node, "atmel,max_x", &temp_val);
	if(rc){
		dev_err(dev, "Unable to read max_x\n");
		return rc;
	}else
		pdata->max_x = temp_val;

	rc = of_property_read_u32(node, "atmel,max_y", &temp_val);
	if(rc){
		dev_err(dev, "Unable to read max_y\n");
		return rc;
	}else
		pdata->max_y = temp_val;

	rc = of_property_read_u32(node, "atmel,irqflags", &temp_val);
	if(rc){
		dev_err(dev, "Unable to read irqflags\n");
		return rc;
	}else
		pdata->irqflags = temp_val;

	rc = of_property_read_u32(node, "atmel,t19_num_keys", &temp_val);
	if(rc){
		dev_err(dev, "Unable to read t19_num_keys\n");
		return rc;
	}else
		pdata->t19_num_keys = temp_val;

	/* config array size */
	pdata->config_array_size = 0;
	temp = NULL;
	while ((temp = of_get_next_child(node, temp)))
		pdata->config_array_size++;

	dev_dbg(dev, "config array size is '%d'\n",pdata->config_array_size);

	if (!pdata->config_array_size){
		dev_err(dev, "config array size is '0'\n");
		return 0;
	}

	info = devm_kzalloc(dev, pdata->config_array_size *
				sizeof(struct mxt_config_info), GFP_KERNEL);
	if (!info) {
		dev_err(dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	pdata->config_array  = info;

	for_each_child_of_node(node, temp) {
		rc = mxt_parse_config(dev, temp, info);
		if (rc) {
			dev_err(dev, "Unable to parse config data\n");
			return rc;
		}
		info++;
	}
	return 0;
}

static int mxt_handle_pdata(struct mxt_data *data)
{
	data->pdata = dev_get_platdata(&data->client->dev);

	/* Use provided platform data if present */
	if (data->pdata) {
		if (data->pdata->cfg_name)
			mxt_update_file_name(&data->client->dev,
					     &data->cfg_name,
					     data->pdata->cfg_name,
					     strlen(data->pdata->cfg_name));

		return 0;
	}

	data->pdata = kzalloc(sizeof(*data->pdata), GFP_KERNEL);
	if (!data->pdata) {
		dev_err(&data->client->dev, "Failed to allocate pdata\n");
		return -ENOMEM;
	}

	/* Set default parameters */
	data->pdata->irqflags = IRQF_TRIGGER_FALLING;

	return 0;
}

static int mxt_initialize_t9_input_device(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	const struct mxt_platform_data *pdata = data->pdata;
	struct input_dev *input_dev;
	int error;
	unsigned int num_mt_slots;
	int i;
	dev_info(dev, "mxt_initialize_t9_input_device\n");
	error = mxt_read_t9_resolution(data);
	if (error)
		dev_warn(dev, "Failed to initialize T9 resolution\n");

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	input_dev->name = "touch_dev";
	input_dev->phys = data->phys;
	input_dev->id.bustype = BUS_I2C;
//	input_dev->dev.parent = dev;
	input_dev->open = mxt_input_open;
	input_dev->close = mxt_input_close;

	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	if (pdata->t19_num_keys) {
		__set_bit(INPUT_PROP_BUTTONPAD, input_dev->propbit);

		for (i = 0; i < pdata->t19_num_keys; i++)
			if (pdata->t19_keymap[i] != KEY_RESERVED)
				input_set_capability(input_dev, EV_KEY,
						     pdata->t19_keymap[i]);

		__set_bit(BTN_TOOL_FINGER, input_dev->keybit);
		__set_bit(BTN_TOOL_DOUBLETAP, input_dev->keybit);
		__set_bit(BTN_TOOL_TRIPLETAP, input_dev->keybit);
		__set_bit(BTN_TOOL_QUADTAP, input_dev->keybit);

		input_abs_set_res(input_dev, ABS_X, MXT_PIXELS_PER_MM);
		input_abs_set_res(input_dev, ABS_Y, MXT_PIXELS_PER_MM);
		input_abs_set_res(input_dev, ABS_MT_POSITION_X,
				  MXT_PIXELS_PER_MM);
		input_abs_set_res(input_dev, ABS_MT_POSITION_Y,
				  MXT_PIXELS_PER_MM);

		input_dev->name = "Atmel maXTouch Touchpad";
	}

	/* For single touch */
	input_set_abs_params(input_dev, ABS_X, 0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, data->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 255, 0, 0);

	/* For multi touch */
	num_mt_slots = data->num_touchids;
	error = input_mt_init_slots(input_dev, num_mt_slots);
	if (error) {
		dev_err(dev, "Error %d initialising slots\n", error);
		goto err_free_mem;
	}

	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID,
			     0, data->pdata->numtouch, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR,
			     0, MXT_MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MINOR,
			     0, MXT_MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, data->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE,
			     0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_ORIENTATION,
			     0, 255, 0, 0);

	/* For T15 key array */
	if (data->T15_reportid_min) {
		data->t15_keystatus = 0;

		for (i = 0; i < data->pdata->t15_num_keys; i++)
			input_set_capability(input_dev, EV_KEY,
					     data->pdata->t15_keymap[i]);
	}

	input_set_drvdata(input_dev, data);

	error = input_register_device(input_dev);
	if (error) {
		dev_err(dev, "Error %d registering input device\n", error);
		goto err_free_mem;
	}
#if 0//def CONFIG_TOUCHSCREEN_ATMEL_KNOCK_ON
	input_set_capability(input_dev, EV_KEY, KEY_POWER);
#endif
	data->input_dev = input_dev;

	return 0;

err_free_mem:
	input_free_device(input_dev);
	return error;
}

static int __devinit mxt_probe(struct i2c_client *client,
			       const struct i2c_device_id *id)
{
	struct mxt_data *data;
	int error;
	int one_sec = 100; //from lge_touch_core
//	touch_thermal_mode = 0;

	is_probing = true;
	dev_info(&client->dev, "%s\n", __func__);

//	mutex_init(&i2c_suspend_lock);

	data = kzalloc(sizeof(struct mxt_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	snprintf(data->phys, sizeof(data->phys), "i2c-%u-%04x/input0",
		 client->adapter->nr, client->addr);
	dev_info(&client->dev, "i2c-%u-%04x/input0\n", client->adapter->nr, client->addr);
	data->client = client;
	data->irq = client->irq;
	i2c_set_clientdata(client, data);

	/*read dtsi data*/
	if (client->dev.of_node) {
		data->pdata = devm_kzalloc(&client->dev,
			sizeof(struct mxt_platform_data), GFP_KERNEL);
		if (!data->pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			error = -ENOMEM;
			goto err_free_mem;
		}

		error = mxt_parse_dt(&client->dev, data->pdata);
		if (error)
			goto err_free_mem;

	} else{
		error = mxt_handle_pdata(data);
		if (error)
			goto err_free_mem;
	}
	/*read dtsi data*/

	if (error)
		goto err_free_mem;

	init_completion(&data->bl_completion);
	init_completion(&data->reset_completion);
	init_completion(&data->crc_completion);

	/* Self Test */
	init_completion(&data->t25_completion);

	/* request reset pin */
	if(data->pdata->gpio_reset> 0){
		error = gpio_request(data->pdata->gpio_reset, "touch_reset");
		if (error < 0) {
			dev_err(&client->dev, "FAIL: touch_reset gpio_request\n");
			goto err_interrupt_failed;
		}
		gpio_direction_output(data->pdata->gpio_reset, 1);
	}

	/* request interrupt pin */
	if(data->pdata->gpio_int > 0){
		error = gpio_request(data->pdata->gpio_int, "touch_int");
		if (error < 0) {
			dev_err(&client->dev, "FAIL: touch_int gpio_request\n");
			goto err_interrupt_failed;
		}
		gpio_direction_input(data->pdata->gpio_int);
	}

	/* request ldo pin */
/*	if(data->pdata->gpio_int > 0){
		error = gpio_request(data->pdata->gpio_ldo_en, "touch_ldo_en");
		if (error < 0) {
			dev_err(&client->dev, "FAIL: touch_ldo_en gpio_request\n");
			goto err_interrupt_failed;
		}
		gpio_direction_output(data->pdata->gpio_ldo_en, 0);
	}
*/

	mxt_probe_regulators(data);

	error = mxt_initialize(data);
	if (error)
		goto err_free_pdata;

	error = request_threaded_irq(data->irq, NULL, mxt_interrupt,
				     data->pdata->irqflags | IRQF_ONESHOT,
				     client->name, data);
	if (error) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err_free_pdata;
	}

	touch_disable_irq(data->irq);

#ifdef FIRMUP_ON_PROBE
   if (data->in_bootloader ||
		data->info->version != MXT_LATEST_FW_VERSION ||
		data->info->build != MXT_LATEST_FW_BUILD) {
		mxt_acquire_irq(data);

		dev_info(&client->dev, "Execute firmware update func\n");
		error = mxt_firmware_update(data);
		if (error) {
			dev_err(&client->dev, "Failed to update firmware\n");
				return error;
		}
   }
#endif

	//Write configuration data and config objects
	error = mxt_rest_init(data);
	if (error)
		goto err_free_irq;

#ifdef CUST_A_TOUCH
	/* accuracy solution */
	if (data->pdata->accuracy_filter_enable){
		data->accuracy_filter.ignore_pressure_gap = 5;
		data->accuracy_filter.delta_max = 30;
		data->accuracy_filter.max_pressure = 255;
		data->accuracy_filter.time_to_max_pressure = one_sec / 20;
		data->accuracy_filter.direction_count = one_sec / 6;
		data->accuracy_filter.touch_max_count = one_sec / 2;
	}
#endif
#if 0//def CONFIG_HAS_EARLYSUSPEND
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + MXT_SUSPEND_LEVEL;
	data->early_suspend.suspend = mxt_early_suspend;
	data->early_suspend.resume = mxt_late_resume;
	register_early_suspend(&data->early_suspend);
#endif
	data->chargerlogo = false;

	/* disabled report touch event to prevent unnecessary event.
	* it will be enabled in open function
	*/
//	mxt_stop(data);

	/* Register sysfs for making fixed communication path to framework layer */
	error = sysdev_class_register(&lge_touch_sys_class);
	if (error < 0) {
		dev_err(&client->dev, "sysdev_class_register is failed\n");
		goto err_lge_touch_sys_class_register;
	}

	error = sysdev_register(&lge_touch_sys_device);
	if (error < 0) {
		dev_err(&client->dev, "sysdev_register is failed\n");
		goto err_lge_touch_sys_dev_register;
	}

	error = kobject_init_and_add(&data->lge_touch_kobj, &lge_touch_kobj_type,
			data->input_dev->dev.kobj.parent,
			"%s", LGE_TOUCH_NAME);
	if (error < 0) {
		dev_err(&client->dev, "kobject_init_and_add is failed\n");
		goto err_lge_touch_sysfs_init_and_add;
	}

	sysfs_bin_attr_init(&data->mem_access_attr);
	data->mem_access_attr.attr.name = "mem_access";
	data->mem_access_attr.attr.mode = S_IRUGO | S_IWUSR;//S_IRWXUGO;
	data->mem_access_attr.read = mxt_mem_access_read;
	data->mem_access_attr.write = mxt_mem_access_write;
	data->mem_access_attr.size = data->mem_size;

	if (sysfs_create_bin_file(&client->dev.kobj,
				  &data->mem_access_attr) < 0) {
		dev_err(&client->dev, "Failed to create %s\n",
			data->mem_access_attr.attr.name);
		goto err_lge_touch_sysfs_init_and_add;
	}

	is_probing = false;

	return 0;

err_lge_touch_sysfs_init_and_add:
	kobject_del(&data->lge_touch_kobj);
err_lge_touch_sys_dev_register:
	sysdev_unregister(&lge_touch_sys_device);
err_lge_touch_sys_class_register:
	sysdev_class_unregister(&lge_touch_sys_class);
	mxt_free_object_table(data);
err_free_irq:
	free_irq(data->irq, data);
err_interrupt_failed:
err_free_pdata:
err_free_mem:
	kfree(data);
	return error;
}

static int __devexit mxt_remove(struct i2c_client *client)
{
	struct mxt_data *data = i2c_get_clientdata(client);

#if 0//def CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&data->early_suspend);
#endif

	if (data->mem_access_attr.attr.name)
		sysfs_remove_bin_file(&client->dev.kobj,
				      &data->mem_access_attr);
	kobject_del(&data->lge_touch_kobj);
	sysdev_unregister(&lge_touch_sys_device);
	sysdev_class_unregister(&lge_touch_sys_class);
	if (data->pdata->gpio_int > 0)
		gpio_free(data->pdata->gpio_int);
	free_irq(data->irq, data);
	regulator_put(data->vcc_i2c);
	regulator_put(data->vdd_ana);
	mxt_free_object_table(data);
	if (!dev_get_platdata(&data->client->dev))
		kfree(data->pdata);
	kfree(data);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int mxt_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;

	if(data->chargerlogo)
		return 0;

#if 0//def CONFIG_TOUCHSCREEN_ATMEL_KNOCK_ON
	if (data->smart_cover_enable) {
		if (gpio_get_value(HALL_IC_GPIO) == 0)
			suspended_due_to_smart_cover = true;
	}
#endif

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		mxt_stop(data);

	mutex_unlock(&input_dev->mutex);
#if 0//def CONFIG_TOUCHSCREEN_ATMEL_KNOCK_ON
	if (device_may_wakeup(dev) && data->knock_on_enable && suspended_due_to_smart_cover == false) {
		touch_enable_irq_wake(data->irq);
		dev_info(&client->dev, "enable irq wake");
	}
#endif
	return 0;
}

static int mxt_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;
	dev_info(&client->dev, "mxt_resume from mxt_late_resume\n");
#if 0//def CONFIG_TOUCHSCREEN_ATMEL_KNOCK_ON
	if (device_may_wakeup(dev) && data->knock_on_enable && suspended_due_to_smart_cover == false) {
		touch_disable_irq_wake(data->irq);
		dev_info(&client->dev, "disable irq wake");
	}
#endif
	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		mxt_start(data);

	mutex_unlock(&input_dev->mutex);

#if 0//def CONFIG_TOUCHSCREEN_ATMEL_KNOCK_ON
	suspended_due_to_smart_cover = false;
#endif
	return 0;
}
#endif

#if 0//def CONFIG_HAS_EARLYSUSPEND
static void mxt_early_suspend(struct early_suspend *h)
{
	struct mxt_data *data = container_of(h, struct mxt_data, early_suspend);

	mxt_suspend(&data->client->dev);
}

static void mxt_late_resume(struct early_suspend *h)
{
	struct mxt_data *data = container_of(h, struct mxt_data, early_suspend);

	mxt_resume(&data->client->dev);
}
#endif

#if 0//def CONFIG_HAS_EARLYSUSPEND
static const struct dev_pm_ops mxt_pm_ops = {
	.suspend	= NULL,
	.resume		= NULL,
};
#else
static const struct dev_pm_ops mxt_pm_ops = {
	.suspend	= mxt_suspend,
	.resume		= mxt_resume,
};
#endif

static void mxt_shutdown(struct i2c_client *client)
{
	struct mxt_data *data = i2c_get_clientdata(client);

	touch_disable_irq(data->irq);
}

static struct of_device_id mxt_match_table[] = {
	{ .compatible = "atmel,1188S",},
	{ },
};

static const struct i2c_device_id mxt_id[] = {
	{ "qt602240_ts", 0 },
	{ "atmel_mxt_ts", 0 },
	{ "atmel_mxt_tp", 0 },
	{ "mXT224", 0 },
	{ MXT_DEVICE_NAME, 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, mxt_id);

static struct i2c_driver mxt_driver = {
	.driver = {
		.name	= MXT_DEVICE_NAME,
		.of_match_table = mxt_match_table,
		.owner	= THIS_MODULE,
		.pm	= &mxt_pm_ops,
	},
	.probe		= mxt_probe,
	.remove		= __devexit_p(mxt_remove),
	.shutdown	= mxt_shutdown,
	.id_table	= mxt_id,
};

static int __init mxt_init(void)
{
	return i2c_add_driver(&mxt_driver);
}

static void __exit mxt_exit(void)
{
	i2c_del_driver(&mxt_driver);
}

module_init(mxt_init);
module_exit(mxt_exit);

/* Module information */
MODULE_AUTHOR("LGE");
MODULE_DESCRIPTION("Atmel maXTouch Touchscreen driver");
MODULE_LICENSE("GPL");
