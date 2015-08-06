/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __LINUX_ATMEL_MXT_TS_H
#define __LINUX_ATMEL_MXT_TS_H

#ifndef CUST_A_TOUCH
#define CUST_A_TOUCH
#endif

#ifndef USE_FW_11AA
#define USE_FW_11AA
#endif

#include <linux/types.h>

#define MXT_DEVICE_NAME			"lge_touch"
#define MXT_MAX_NUM_TOUCHES		10

#ifdef USE_FW_11AA
#define MXT_LATEST_FW_VERSION       0x11
#define MXT_LATEST_FW_BUILD         0xAA
#else //USE_FW_11AA
#define MXT_LATEST_FW_VERSION       0x20
#define MXT_LATEST_FW_BUILD         0xAB
#endif //USE_FW_11AA

#define LGE_TOUCH_NAME		"lge_touch"

enum { MXT_RESERVED_T0 = 0,
	MXT_RESERVED_T1,
	MXT_DEBUG_DELTAS_T2,
	MXT_DEBUG_REFERENCES_T3,
	MXT_DEBUG_SIGNALS_T4,
	MXT_GEN_MESSAGE_T5,
	MXT_GEN_COMMAND_T6,
	MXT_GEN_POWER_T7,
	MXT_GEN_ACQUIRE_T8,
	MXT_TOUCH_MULTI_T9,
	MXT_TOUCH_SINGLETOUCHSCREEN_T10,
	MXT_TOUCH_XSLIDER_T11,
	MXT_TOUCH_YSLIDER_T12,
	MXT_TOUCH_XWHEEL_T13,
	MXT_TOUCH_YWHEEL_T14,
	MXT_TOUCH_KEYARRAY_T15,
	MXT_PROCG_SIGNALFILTER_T16,
	MXT_PROCI_LINEARIZATIONTABLE_T17,
	MXT_SPT_COMMSCONFIG_T18,
	MXT_SPT_GPIOPWM_T19,
	MXT_PROCI_GRIPFACE_T20,
	MXT_RESERVED_T21,
	MXT_PROCG_NOISE_T22,
	MXT_TOUCH_PROXIMITY_T23,
	MXT_PROCI_ONETOUCH_T24,
	MXT_SPT_SELFTEST_T25,
	MXT_DEBUG_CTERANGE_T26,
	MXT_PROCI_TWOTOUCH_T27,
	MXT_SPT_CTECONFIG_T28,
	MXT_SPT_GPI_T29,
	MXT_SPT_GATE_T30,
	MXT_TOUCH_KEYSET_T31,
	MXT_TOUCH_XSLIDERSET_T32,
	MXT_RESERVED_T33,
	MXT_GEN_MESSAGEBLOCK_T34,
	MXT_SPT_GENERICDATA_T35,
	MXT_RESERVED_T36,
	MXT_DEBUG_DIAGNOSTIC_T37,
	MXT_SPT_USERDATA_T38,
	MXT_SPARE_T39,
	MXT_PROCI_GRIP_T40,
	MXT_SPARE_T41,
	MXT_PROCI_TOUCHSUPPRESSION_T42,
	MXT_SPT_DIGITIZER_T43,
	MXT_SPT_MESSAGECOUNT_T44,
	MXT_SPARE_T45,
	MXT_SPT_CTECONFIG_T46,
	MXT_PROCI_STYLUS_T47,
	MXT_SPT_NOISESUPPRESSION_T48,
	MXT_SPARE_T49,
	MXT_SPARE_T50,
	MXT_SPARE_T51,
	MXT_TOUCH_PROXKEY_T52,
	MXT_GEN_DATASOURCE_T53,
	MXT_SPARE_T54,
	MXT_PROCI_ADAPTIVETHRESHOLD_T55,
	MXT_PROCI_SHIELDLESS_T56,
	MXT_PROCI_EXTRATOUCHSCREENDATA_T57,
	MXT_SPARE_T58,
	MXT_SPARE_T59,
	MXT_SPARE_T60,
	MXT_SPT_TIMER_T61,
	MXT_PROCG_NOISESUPPRESSION_T62,
	MXT_PROCI_ACTIVE_STYLUS_T63,
	MXT_PROCI_LENSBENDING_T65 = 65,
	MXT_GOLDENREFERENCES_T66,
	MXT_SERIALDATACOMMAND_T68 = 68,
	MXT_SPT_DYNAMICCONFIGURATIONCONTROLLER_T70 = 70,
	MXT_SPT_DYNAMICCONFIGURATIONCONTAINER_T71,
	MXT_PROCI_ZONEINDICATION_T73 = 73,
	MXT_SPT_CTESCANCONFIG_T77 = 77,
	MXT_SPT_TOUCHEVENTTRIGGER_T79 = 79,
#ifdef USE_FW_11AA
	MXT_SPT_NOISESUPEXTENSION_T82 = 82,
#endif
	MXT_TOUCH_MULTITOUCHSCREEN_T100 = 100,
	MXT_RESERVED_T255 = 255,
};

enum {
	MXT_T37 = 0,
	MXT_T38,
	MXT_T71,
	MXT_T7,
	MXT_T8,
	MXT_T9,
	MXT_T15,
	MXT_T18,
	MXT_T19,
	MXT_T24,
	MXT_T25,
	MXT_T40,
	MXT_T42,
	MXT_T43,
	MXT_T46,
	MXT_T47,
	MXT_T55,
	MXT_T56,
	MXT_T57,
	MXT_T61,
	MXT_T62,
	MXT_T63,
	MXT_T65,
	MXT_T66,
	MXT_T70,
	MXT_T73,
	MXT_T77,
	MXT_T79,
	MXT_T87,
	MXT_TMAX,
};

#ifdef CUST_A_TOUCH
enum{
	FINGER_INACTIVE,
	FINGER_RELEASED,
	FINGER_PRESSED,
	FINGER_MOVED
};
#endif

struct mxt_config_info {
	u8 *config_t[29];
};

/* The platform data for the Atmel maXTouch touchscreen driver */
struct mxt_platform_data {
	const struct mxt_config_info *config_array;
	size_t config_array_size;
	u8    numtouch;	/* Number of touches to report	*/
	int   display_res_x;
	int   display_res_y;
	int   min_x;
	int   min_y;
	int   max_x;    /* The default reported X range   */
	int   max_y;    /* The default reported Y range   */
	bool wakeup;

	unsigned long irqflags;
	u8 t19_num_keys;
	const unsigned int *t19_keymap;
	int t15_num_keys;
	const unsigned int *t15_keymap;
	unsigned long gpio_reset;
	unsigned long gpio_int;
	unsigned long gpio_ldo_en;
	const char *cfg_name;
/*
	const u8 **config;
	const u8 **charger_config;
	const u8 **restore_config;
    const u8 **pen_config;
    const u8 **pen_high_speed_config;
    const u8 **pen_low_speed_config;
	const u8 **unpen_config;
#ifdef CONFIG_TOUCHSCREEN_ATMEL_KNOCK_ON
	const u8 **sus_config;
	const u8 **sus_charger_config;
	const u8 **act_config;
#endif
*/
#ifdef CUST_A_TOUCH
	int		accuracy_filter_enable;
	int		ghost_detection_enable;
#endif
};

#endif /* __LINUX_ATMEL_MXT_TS_H */

