/*
 * include/linux/input/sx9311_platform_data.h
 *
 * SX9311 Platform Data
 * 4 cap single
 *
 * Copyright 2013 Semtech Corp.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _SX9311_PLATFORM_DATA_H_
#define _SX9311_PLATFORM_DATA_H_


#include <linux/i2c.h>

#define NUM_SET_REG_DATA  25

struct smtc_reg_data {
	unsigned char reg;
	unsigned char val;
};

struct _buttoninfo {
	/*! The Key to send to the input */
	int keycode;
	/*! Mask to look for on Touch Status */
	int mask;
	/*! Current state of button. */
	int state;
};

struct _totalbuttoninformation {
	struct _buttoninfo *buttons;
	int buttonSize;
	struct input_dev *input;
};

struct _startupcheckparameters {
	s32 dynamicthreshold_offset; // used as an offset for startup (calcualted from temperature test)
	s32 dynamicthreshold_temp_slope; // used for temperature slope
	s32 dynamicthreshold_hysteresis;
	s32 calibration_margin;
	u8 startup_touch_regavgthresh; // capacitance(of dynamicthreshold hysteresis) / 128
	u8 startup_release_regavgthresh; // same value as regproxctrl4[avgthresh]
};

struct sx9311_platform_data {
	int (*get_is_nirq_low)(void);

	int     (*init_platform_hw)(struct i2c_client *client);
	void    (*exit_platform_hw)(struct i2c_client *client);

	unsigned int irq_gpio;

	struct regulator *vdd_regulator;
	struct regulator *svdd_regulator;

	u32 vdd_supply_min;
	u32 vdd_supply_max;
	u32 vdd_load_ua;

	u32 svdd_supply_min;
	u32 svdd_supply_max;
	u32 svdd_load_ua;

	struct smtc_reg_data pi2c_reg[NUM_SET_REG_DATA];
	int i2c_reg_num;

	u32 input_pins_num; // not include ref sensor pin
	u8 input_mainsensor;  // 0x00 ~ 0x03, refer regsensorsel
	u8 input_main2sensor; // 0x00 ~ 0x03, refer regsensorsel
	u8 input_refsensor;   // 0x00 ~ 0x03, refer regsensorsel

	struct _totalbuttoninformation *pButtonInformation;

	struct _startupcheckparameters *pStartupCheckParameters;

	u32 capacitance_range;
	u32 gain_factor;
};

/*! \struct sx9311
 * Specialized struct containing input event data, platform data, and
 * last cap state read if needed.
 */
struct sx9311 {
	struct _totalbuttoninformation *pButtonInformation;
	struct _startupcheckparameters *pStartupCheckParameters;
	struct sx9311_platform_data *hw; /* specific platform data settings */
};

//static int initialize_device(struct sx86XX *this);
void StartupTouchCheckWithReferenceSensor(struct sx86XX *this,
		unsigned char mainSensor,
		unsigned char refSensor);

#endif
