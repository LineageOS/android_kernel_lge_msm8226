/*
 *  apds9130.c - Linux kernel modules for proximity sensor
 *
 *  Copyright (C) 2012 Lee Kai Koon <kai-koon.lee@avagotech.com>
 *  Copyright (C) 2012 Avago Technologies
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
#define MODULE_TAG	"<apds9130>"

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/unistd.h>
#include <linux/async.h>


#ifdef CONFIG_OF
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#endif

/*apds9130 proximity sensor calibration*/
#define APDS9130_PROXIMITY_CAL
#if defined(APDS9130_PROXIMITY_CAL)
#include <linux/syscalls.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

#define PS_DEFAULT_CROSS_TALK 150
#endif

#include "lge_log.h"
#define SENSOR_TAG	"[LGE_Proximity]"

#define LGE_PROXIMITY_NAME	"lge_proximity"

#define APDS9130_DRV_NAME	"apds9130"
#define DRIVER_VERSION		"1.0.2"

#define APDS9130_INT		IRQ_EINT20

#define APDS9130_PS_MIN_THRESHOLD			0
#define APDS9130_PS_MAX_THRESHOLD			1023
#define APDS9130_PS_DETECTION_THRESHOLD		600
#define APDS9130_PS_HYSTERESIS_THRESHOLD	500

#ifdef APDS9930_ALS_SENSOR_ENABLE
#define APDS9930_ALS_UPPER_THRESHOLD	15000
#define APDS9930_ALS_LOWER_THRESHOLD	9000
#endif

/*Device Tree System Set PPcount*/
/*#define APDS9130_PS_PULSE_NUMBER	8 //platform_data->ppcount*/

#define DEFAULT_CAL_MAX_THRESHOLD		870
#define DEFAULT_CAL_NEAR_THRESHOLD_OFFSET	150
#define DEFAULT_CAL_FAR_THRESHOLD_OFFSET	50

#define APDS9130_INTERRUPT_MODE_ENABLE

/* Change History
 *
 * 1.0.0	Funcamental Functions of APDS-9130
 *
 * 1.0.1	Remove ioctl interface, remain using sysfs
 * 1.0.2	Add LG cross-talk calibration process
 * 1.0.3	LGE Commonization (WX-BSP-TS@lge.com, 2013/8/27)
 *
 */

#define APDS9130_IOCTL_PS_ENABLE		1
#define APDS9130_IOCTL_PS_GET_ENABLE		2
#define APDS9130_IOCTL_PS_GET_PDATA		3	/* pdata*/
#define APDS9130_IOCTL_PS_GET_PSAT		4	/* ps saturation - used to detect if ps is triggered by bright light*/
#define APDS9130_IOCTL_PS_POLL_DELAY		5

#define APDS9130_DISABLE_PS			0
#define APDS9130_ENABLE_PS_WITH_INT		1
#define APDS9130_ENABLE_PS_NO_INT		2

#define APDS9130_PS_POLL_SLOW			0	/* 1 Hz (1s) */
#define APDS9130_PS_POLL_MEDIUM			1	/* 10 Hz (100ms) */
#define APDS9130_PS_POLL_FAST			2	/* 20 Hz (50ms) */

/*
 * Defines
 */

#define APDS9130_ENABLE_REG	0x00

#ifdef APDS9930_ALS_SENSOR_ENABLE
#define APDS9930_ATIME_REG	0x01
#endif

#define APDS9130_PTIME_REG	0x02
#define APDS9130_WTIME_REG	0x03

#ifdef APDS9930_ALS_SENSOR_ENABLE
#define APDS9930_AILTL_REG	0x04
#define APDS9930_AILTH_REG	0x05
#define APDS9930_AIHTL_REG	0x06
#define APDS9930_AIHTH_REG	0x07
#endif

#define APDS9130_PILTL_REG	0x08
#define APDS9130_PILTH_REG	0x09
#define APDS9130_PIHTL_REG	0x0A
#define APDS9130_PIHTH_REG	0x0B
#define APDS9130_PERS_REG	0x0C
#define APDS9130_CONFIG_REG	0x0D
#define APDS9130_PPCOUNT_REG	0x0E
#define APDS9130_CONTROL_REG	0x0F
#define APDS9130_REV_REG	0x11
#define APDS9130_ID_REG		0x12
#define APDS9130_STATUS_REG	0x13

#ifdef APDS9930_ALS_SENSOR_ENABLE
#define APDS9930_CDATA0L_REG 0x14
#define APDS9930_CDATA1L_REG 0x16
#endif

#define APDS9130_PDATAL_REG	0x18
#define APDS9130_PDATAH_REG	0x19

#define CMD_BYTE		0x80
#define CMD_WORD		0xA0
#define CMD_SPECIAL		0xE0

#define CMD_CLR_PS_INT		0xE5
#define CMD_CLR_ALS_INT		0xE6
#define CMD_CLR_PS_ALS_INT	0xE7

/* Register Value define : PERS */
#define APDS9130_PPERS_0	0x00  /* Every proximity ADC cycle */
#define APDS9130_PPERS_1	0x10  /* 1 consecutive proximity value out of range */
#define APDS9130_PPERS_2	0x20  /* 2 consecutive proximity value out of range */
#define APDS9130_PPERS_3	0x30  /* 3 consecutive proximity value out of range */
#define APDS9130_PPERS_4	0x40  /* 4 consecutive proximity value out of range */
#define APDS9130_PPERS_5	0x50  /* 5 consecutive proximity value out of range */
#define APDS9130_PPERS_6	0x60  /* 6 consecutive proximity value out of range */
#define APDS9130_PPERS_7	0x70  /* 7 consecutive proximity value out of range */
#define APDS9130_PPERS_8	0x80  /* 8 consecutive proximity value out of range */
#define APDS9130_PPERS_9	0x90  /* 9 consecutive proximity value out of range */
#define APDS9130_PPERS_10	0xA0  /* 10 consecutive proximity value out of range */
#define APDS9130_PPERS_11	0xB0  /* 11 consecutive proximity value out of range */
#define APDS9130_PPERS_12	0xC0  /* 12 consecutive proximity value out of range */
#define APDS9130_PPERS_13	0xD0  /* 13 consecutive proximity value out of range */
#define APDS9130_PPERS_14	0xE0  /* 14 consecutive proximity value out of range */
#define APDS9130_PPERS_15	0xF0  /* 15 consecutive proximity value out of range */

#ifdef APDS9930_ALS_SENSOR_ENABLE
/* Register Value define : PERS */
#define APDS9930_APERS_0	0x00  /* Every proximity ADC cycle */
#define APDS9930_APERS_1	0x01  /* 1 consecutive proximity value out of range */
#define APDS9930_APERS_2	0x02  /* 2 consecutive proximity value out of range */
#define APDS9930_APERS_3	0x03  /* 3 consecutive proximity value out of range */
#define APDS9930_APERS_4	0x04  /* 4 consecutive proximity value out of range */
#define APDS9930_APERS_5	0x05  /* 5 consecutive proximity value out of range */
#define APDS9930_APERS_6	0x06  /* 6 consecutive proximity value out of range */
#define APDS9930_APERS_7	0x07  /* 7 consecutive proximity value out of range */
#define APDS9930_APERS_8	0x08  /* 8 consecutive proximity value out of range */
#define APDS9930_APERS_9	0x09  /* 9 consecutive proximity value out of range */
#define APDS9930_APERS_10	0x0A  /* 10 consecutive proximity value out of range */
#define APDS9930_APERS_11	0x0B  /* 11 consecutive proximity value out of range */
#define APDS9930_APERS_12	0x0C  /* 12 consecutive proximity value out of range */
#define APDS9930_APERS_13	0x0D  /* 13 consecutive proximity value out of range */
#define APDS9930_APERS_14	0x0E  /* 14 consecutive proximity value out of range */
#define APDS9930_APERS_15	0x0F  /* 15 consecutive proximity value out of range */
#endif

#define APDS9130_PRX_IR_DIOD	0x20  /* Proximity uses CH1 diode */

#define APDS9130_PGAIN_1X	0x00  /* PS GAIN 1X */
#define APDS9130_PGAIN_2X	0x04  /* PS GAIN 2X */
#define APDS9130_PGAIN_4X	0x08  /* PS GAIN 4X */
#define APDS9130_PGAIN_8X	0x0C  /* PS GAIN 8X */

#ifdef APDS9930_ALS_SENSOR_ENABLE
#define APDS9930_AGAIN_1X	0x00  /* ALS AGAIN 1X */
#define APDS9930_AGAIN_8X	0x01  /* ALS AGAIN 8X */
#define APDS9930_AGAIN_16X	0x02  /* ALS AGAIN 16X */
#define APDS9930_AGAIN_120X	0x03  /* ALS AGAIN 120X */
#endif

/* I2C Suspend Check */
#define APDS9130_STATUS_RESUME		0
#define APDS9130_STATUS_SUSPEND		1
#define APDS9130_STATUS_QUEUE_WORK	2


#if 0  /*Device Tree System Set Pdrive*/
#define APDS9130_PDRVIE_100MA	0x00  /* PS 100mA LED drive */
#define APDS9130_PDRVIE_50MA	0x40  /* PS 50mA LED drive */
#define APDS9130_PDRVIE_25MA	0x80  /* PS 25mA LED drive */
#define APDS9130_PDRVIE_12_5MA	0xC0  /* PS 12.5mA LED drive */
#endif


/*
 * Structs
 */

#ifdef CONFIG_OF
enum sensor_dt_entry_status {
	DT_REQUIRED,
	DT_SUGGESTED,
	DT_OPTIONAL,
};

enum sensor_dt_entry_type {
	DT_U32,
	DT_GPIO,
	DT_BOOL,
};

struct sensor_dt_to_pdata_map {
	const char			*dt_name;
	void				*ptr_data;
	enum sensor_dt_entry_status status;
	enum sensor_dt_entry_type	type;
	int				default_val;
};

struct apds9130_platform_data {
	int irq_num;
	int (*power)(unsigned char onoff);
	unsigned int prox_int_low_threshold;
	unsigned int prox_int_high_threshold;
#ifdef APDS9930_ALS_SENSOR_ENABLE
	unsigned int als_threshold_hysteresis;
#endif
	unsigned int ppcount;

	int (*init)(struct i2c_client *client);
	void (*exit)(struct i2c_client *client);
	int (*power_on)(struct i2c_client *client, bool on);

	bool i2c_pull_up;
	bool digital_pwr_regulator;

	unsigned int irq_gpio;
	u32 irq_gpio_flags;

	struct regulator *vcc_ana;
	struct regulator *vcc_dig;
	struct regulator *vcc_i2c;

	u32 vdd_ana_supply_min;
	u32 vdd_ana_supply_max;
	u32 vdd_ana_load_ua;

	u32 vddio_dig_supply_min;
	u32 vddio_dig_supply_max;
	u32 vddio_dig_load_ua;

	u32 vddio_i2c_supply_min;
	u32 vddio_i2c_supply_max;
	u32 vddio_i2c_load_ua;
	u32 pdrive;
	u32 near_offset;
	u32 far_offset;
	u32 crosstalk_max;

#ifdef APDS9930_ALS_SENSOR_ENABLE
	u32 als_upper_threshold;
	u32 als_lower_threshold;
#endif
};
#endif

struct apds9130_data {
	struct i2c_client *client;
	struct mutex update_lock;
	struct mutex enable_lock;
#ifdef APDS9130_INTERRUPT_MODE_ENABLE
	struct delayed_work	dwork;		/* for PS interrupt */
#else
	struct delayed_work	ps_dwork;	/* for PS polling */
#endif
	struct input_dev *input_dev_ps;
	struct wake_lock ps_wlock;

#ifdef CONFIG_OF
	struct apds9130_platform_data *platform_data;
	int irq;
#endif
	unsigned int enable;
#ifdef APDS9930_ALS_SENSOR_ENABLE
	unsigned int atime;
#endif
	unsigned int ptime;
	unsigned int wtime;

#ifdef APDS9930_ALS_SENSOR_ENABLE
	unsigned int ailt;
	unsigned int aiht;
#endif

	unsigned int pilt;
	unsigned int piht;
	unsigned int pers;
	unsigned int config;
	/*	unsigned int ppcount;*/
	unsigned int control;

	/*	unsigned int pDrive;*/

	/* control flag from HAL */
	unsigned int enable_ps_sensor;

	/* PS parameters */
	unsigned int ps_threshold;
	unsigned int ps_hysteresis_threshold;	/* always lower than ps_threshold */
	unsigned int ps_detection;		/* 0 = near-to-far; 1 = far-to-near */
	unsigned int ps_data;			/* to store PS data */
	unsigned int ps_sat;			/* to store PS saturation bit */
#ifdef APDS9130_POLLING_MODE_ENABLE
	unsigned int ps_poll_delay;		/* needed for PS polling */
#endif
	unsigned int sw_mode;

#ifdef APDS9930_ALS_SENSOR_ENABLE
	unsigned int als_upper_threshold;
	unsigned int als_lower_threshold;
	unsigned int als_cdata0;
	unsigned int als_cdata1;
	unsigned int als_detection; /* 1 -> strong light detect,  0 -> normal*/
#endif

#if defined(APDS9130_PROXIMITY_CAL)
	int cross_talk;
	bool read_ps_cal_data;
	int ps_cal_result; /* [LGSI_SP4_BSP][kirankumar.vm@lge.com] Proximity Testmode changes */
#endif

	atomic_t i2c_status;
};

/*
 * Global data
 */
static struct i2c_client
		*apds9130_i2c_client; /* global i2c_client to support ioctl */
static struct workqueue_struct *apds9130_workqueue;

enum apds9130_dev_status {
	PROX_STAT_SHUTDOWN = 0,
	PROX_STAT_OPERATING,
};

enum apds9130_input_event {
	PROX_INPUT_UNKNOWN = -1,
	PROX_INPUT_NEAR = 0,
	PROX_INPUT_FAR,
};

#if defined(APDS9130_PROXIMITY_CAL)
unsigned int apds_proxi_low_threshold;
unsigned int apds_proxi_high_threshold;
static int apds9130_read_crosstalk_data_fs(void);
static void apds9130_Set_PS_Threshold_Adding_Cross_talk(
	struct i2c_client *client, int cal_data);
#endif

#ifdef APDS9930_ALS_SENSOR_ENABLE
void (*apds9930_lux_change_cb)(int);

/* only one callback is maintained. may change it to list of callbacks */
void apds9930_register_lux_change_callback (void (*callback) (int))
{
	apds9930_lux_change_cb = callback;
}
#endif

static int apds9130_init_client(struct i2c_client *client);


/*
 * Management functions
 */

static int apds9130_set_pilt(struct i2c_client *client, int threshold)
{
	struct apds9130_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client, CMD_WORD | APDS9130_PILTL_REG,
					threshold);
	mutex_unlock(&data->update_lock);

	data->pilt = threshold;

	return ret;
}

static int apds9130_set_piht(struct i2c_client *client, int threshold)
{
	struct apds9130_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client, CMD_WORD | APDS9130_PIHTL_REG,
					threshold);
	mutex_unlock(&data->update_lock);

	data->piht = threshold;

	return ret;
}

#ifdef APDS9930_ALS_SENSOR_ENABLE
static int apds9930_set_ailt(struct i2c_client *client, int threshold)
{
	struct apds9130_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client, CMD_WORD | APDS9930_AILTL_REG,
					threshold);
	mutex_unlock(&data->update_lock);

	data->ailt = threshold;
	SENSOR_LOG("Set ailt %d",data->ailt);
	return ret;
}

static int apds9930_set_aiht(struct i2c_client *client, int threshold)
{
	struct apds9130_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client, CMD_WORD | APDS9930_AIHTL_REG,
					threshold);
	mutex_unlock(&data->update_lock);

	data->aiht = threshold;
	SENSOR_LOG("Set aiht %d",data->aiht);
	return ret;
}
#endif

static int apds9130_set_command(struct i2c_client *client, int command)
{
	struct apds9130_data *data = i2c_get_clientdata(client);
	int ret;
	int clearInt;

	if (command == 0)
		clearInt = CMD_CLR_PS_INT;
	else if (command == 1)
		clearInt = CMD_CLR_ALS_INT;
	else
		clearInt = CMD_CLR_PS_ALS_INT;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte(client, clearInt);
	mutex_unlock(&data->update_lock);

	return ret;
}

static int apds9130_set_enable(struct i2c_client *client, int enable)
{
	struct apds9130_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE | APDS9130_ENABLE_REG, enable);
	mutex_unlock(&data->update_lock);

	data->enable = enable;

	return ret;
}

#if defined(APDS9130_PROXIMITY_CAL)
void apds9130_swap(int *x, int *y)
{
	int temp = *x;
	*x = *y;
	*y = temp;
}

static int apds9130_backup_crosstalk_data_fs(unsigned int val)
{
	int fd;
	int ret = 0;
	char buf[50];
	mm_segment_t old_fs = get_fs();

	memset(buf, 0, sizeof(buf));
	sprintf(buf, "%d", val);

	SENSOR_LOG("Enter");
	SENSOR_LOG("buf = %s ", buf);

	set_fs(KERNEL_DS);
	fd = sys_open("/sns/prox_calibration.dat", O_WRONLY | O_CREAT, 0664);

	if (fd >= 0) {
		sys_write(fd, buf, sizeof(buf));
		sys_fsync(fd); /*ensure calibration data write to file system*/
		sys_close(fd);
		sys_chmod("/sns/prox_calibration.dat", 0664);
		set_fs(old_fs);
	} else {
		ret++;
		sys_close(fd);
		set_fs(old_fs);
		return ret	;
	}

	return ret;
}
static int apds9130_read_crosstalk_data_fs(void)
{
	int fd;
	int ret = 0;
	int len = 0;
	char read_buf[50];
	mm_segment_t old_fs = get_fs();

	SENSOR_LOG("Enter");
	memset(read_buf, 0, sizeof(read_buf));
	set_fs(KERNEL_DS);

	fd = sys_open("/sns/prox_calibration.dat", O_RDONLY, 0);
	if (fd >= 0) {
		SENSOR_LOG("Success read Prox Cross-talk from FS ");
		len = sys_read(fd, read_buf, sizeof(read_buf));
		SENSOR_LOG("Proximity Calibration File size is = %d", len);
		if (len <= 0) {
			ret = -1;
			sys_close(fd);
			set_fs(old_fs);
			return ret;
		}
		sys_close(fd);
		set_fs(old_fs);
	} else {
		SENSOR_ERR("Fail read Prox Cross-talk FS");
		SENSOR_ERR("Return error code : %d", fd);
		ret = -1;
		sys_close(fd);
		set_fs(old_fs);
		return ret;
	}

	return (simple_strtol(read_buf, NULL, 10));

}

#ifdef APDS9930_ALS_SENSOR_ENABLE
static void apds9930_set_als_threshold(
	struct i2c_client *client, int cal_data)
{
	struct apds9130_data *data = i2c_get_clientdata(client);

	data->als_upper_threshold = data->platform_data->als_upper_threshold;
	data->als_lower_threshold = data->platform_data->als_lower_threshold;
	SENSOR_LOG("ALS H Threshold = %d, L Threshold = %d", data->als_upper_threshold, data->als_lower_threshold);
}
#endif

static void apds9130_Set_PS_Threshold_Adding_Cross_talk(
	struct i2c_client *client, int cal_data)
{
	struct apds9130_data *data = i2c_get_clientdata(client);

	if (cal_data > data->platform_data->crosstalk_max)
		cal_data = data->platform_data->crosstalk_max;
	if (cal_data < 0)
		cal_data = 0;

	data->cross_talk = cal_data;
	data->ps_threshold = data->platform_data->near_offset + cal_data;
	data->ps_hysteresis_threshold = data->ps_threshold -
					data->platform_data->far_offset;
	SENSOR_LOG("Crosstalk = %d, H Threshold = %d, L Threshold = %d", data->cross_talk,
	      data->ps_threshold, data->ps_hysteresis_threshold);
	/*apds_proxi_high_threshold = data->ps_threshold;*/
	/*apds_proxi_low_threshold = data->ps_hysteresis_threshold;*/

}

static int apds9130_Run_Cross_talk_Calibration(struct i2c_client *client)
{
	struct apds9130_data *data = i2c_get_clientdata(client);
	unsigned int sum_of_pdata = 0, temp_pdata[20];
	unsigned int ret = 0, i = 0, j = 0, ArySize = 20, cal_check_flag = 0;
	unsigned int old_enable = 0;

	SENSOR_LOG("Enter ");
RE_CALIBRATION:
	old_enable = data->enable;
	sum_of_pdata = 0;
	apds9130_set_enable(client, 0x0D);

	msleep(50);

	for (i = 0; i < 20; i++)	{
		temp_pdata[i] = i2c_smbus_read_word_data(client,
				CMD_WORD | APDS9130_PDATAL_REG);
		mdelay(6);
	}

	for (i = 0; i < ArySize - 1; i++)
		for (j = i + 1; j < ArySize; j++)
			if (temp_pdata[i] > temp_pdata[j])
				apds9130_swap(temp_pdata + i, temp_pdata + j);

	for (i = 5; i < 15; i++)
		sum_of_pdata = sum_of_pdata + temp_pdata[i];

	data->cross_talk = sum_of_pdata / 10;
	SENSOR_LOG("sum_of_pdata/10 = %d", data->cross_talk);
	if (data->cross_talk > data->platform_data->crosstalk_max) {
		if (cal_check_flag == 0) {
			cal_check_flag = 1;
			goto RE_CALIBRATION;
		} else {
			apds9130_set_enable(client, old_enable);
			return -1;
		}
	}

	data->ps_threshold = data->platform_data->near_offset + data->cross_talk;
	data->ps_hysteresis_threshold = data->ps_threshold -
					data->platform_data->far_offset;

	ret = apds9130_backup_crosstalk_data_fs(data->cross_talk);

	SENSOR_LOG("threshold : %d", data->ps_threshold);
	SENSOR_LOG("Hysteresis_threshold : %d", data->ps_hysteresis_threshold);

	apds9130_set_enable(client, old_enable);
	SENSOR_LOG("Leave");

	return data->cross_talk;
}

static ssize_t apds9130_show_run_calibration(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct apds9130_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n",
		       data->ps_cal_result); /*[LGSI_SP4_BSP][kirankumar.vm@lge.com] Proximity Testmode changes*/
}

static ssize_t apds9130_store_run_calibration(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds9130_data *data = dev_get_drvdata(dev);
	int ret = apds9130_Run_Cross_talk_Calibration(data->client);

	if (ret < 0) {
		SENSOR_ERR("Fail error :  %d", ret);
		data->ps_cal_result = 0;
	} else {
		SENSOR_LOG("Succes cross-talk :  %d", ret);
		data->ps_cal_result = 1;
	}

	return count;
}
static DEVICE_ATTR(run_calibration,
		   S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH/*|S_IWOTH*/,
		   apds9130_show_run_calibration, apds9130_store_run_calibration);

static ssize_t apds9130_show_crosstalk_data(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;

	ret = apds9130_read_crosstalk_data_fs();
	if (ret < 0)
		return sprintf(buf, "Read fail\n");

	return sprintf(buf, "%d\n", ret);
}

static ssize_t apds9130_store_crosstalk_data(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds9130_data *data = dev_get_drvdata(dev);
	int ret = 0;
	unsigned long val = simple_strtoul(buf, NULL, 10);


	SENSOR_LOG("Enter");
	ret = apds9130_backup_crosstalk_data_fs(val);
	if (ret != 0)
		return SENSOR_ERR("File open fail %d", ret);

	data->cross_talk = val;

	SENSOR_LOG("Saved cross_talk val : %d", (int)val);


	return count;
}

static DEVICE_ATTR(prox_cal_data,
		   S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH/*|S_IWOTH*/,
		   apds9130_show_crosstalk_data, apds9130_store_crosstalk_data);
#endif

#ifdef APDS9930_ALS_SENSOR_ENABLE
static int apds9930_set_atime(struct i2c_client *client, int atime)
{
	struct apds9130_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE | APDS9930_ATIME_REG, atime);
	mutex_unlock(&data->update_lock);

	data->atime = atime;

	return ret;
}
#endif

static int apds9130_set_ptime(struct i2c_client *client, int ptime)
{
	struct apds9130_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE | APDS9130_PTIME_REG, ptime);
	mutex_unlock(&data->update_lock);

	data->ptime = ptime;

	return ret;
}

static int apds9130_set_wtime(struct i2c_client *client, int wtime)
{
	struct apds9130_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE | APDS9130_WTIME_REG, wtime);
	mutex_unlock(&data->update_lock);

	data->wtime = wtime;

	return ret;
}

static int apds9130_set_pers(struct i2c_client *client, int pers)
{
	struct apds9130_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE | APDS9130_PERS_REG, pers);
	mutex_unlock(&data->update_lock);

	data->pers = pers;

	return ret;
}

static int apds9130_set_config(struct i2c_client *client, int config)
{
	struct apds9130_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE | APDS9130_CONFIG_REG, config);
	mutex_unlock(&data->update_lock);

	data->config = config;

	return ret;
}

static int apds9130_set_ppcount(struct i2c_client *client, int ppcount)
{
	struct apds9130_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE | APDS9130_PPCOUNT_REG,
					ppcount);
	mutex_unlock(&data->update_lock);

	data->platform_data->ppcount = ppcount;

	return ret;
}

static int apds9130_set_control(struct i2c_client *client, int control)
{
	struct apds9130_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE | APDS9130_CONTROL_REG,
					control);
	mutex_unlock(&data->update_lock);

	data->control = control;

	return ret;
}

static void apds9130_change_ps_threshold(struct i2c_client *client)
{
	struct apds9130_data *data = i2c_get_clientdata(client);

#ifdef APDS9930_ALS_SENSOR_ENABLE
	apds9130_set_pers(client, APDS9130_PPERS_2 | APDS9930_APERS_2);
#else
	apds9130_set_pers(client, APDS9130_PPERS_2);
#endif
	/* repeat this because of the first interrupt forced */

	if ((data->ps_sat & 0x40) == 0x40 ) {
		/* temporarily narrow interrupt condition if strong light hits.
		* pdata may linger within near range without any object, which result in continuous interrupt under normal interrupt condition.
		* Thus, if such incident happens, narrow interrupt condition (i.e set high threshold to very near range (about 2~3cm)) 
		* so that no more interrupt occurs.
		*/
		SENSOR_LOG("Triggered by strong light");
		if (data->ps_detection != 0) {
			/* Bring it back to FAR */
			input_report_abs(data->input_dev_ps, ABS_DISTANCE, PROX_INPUT_FAR);
			input_sync(data->input_dev_ps);

			SENSOR_LOG("near-to-far enforced.");
		}

		apds9130_set_pilt(client, APDS9130_PS_MIN_THRESHOLD);
		apds9130_set_piht(client, APDS9130_PS_MAX_THRESHOLD - 1);

		data->ps_detection = 0;
		SENSOR_LOG("Narrow interrupt condition.");
		/* 0 = CMD_CLR_PS_INT */
		apds9130_set_command(client, 0);
		return;
	}

	data->ps_data =	i2c_smbus_read_word_data(client,
			CMD_WORD | APDS9130_PDATAL_REG);

	if (data->ps_data >= data->piht) {
		/* far-to-near detected */
		data->ps_detection = 1;

		input_report_abs(data->input_dev_ps, ABS_DISTANCE, PROX_INPUT_NEAR);
		input_sync(data->input_dev_ps);

		apds9130_set_pilt(client, data->ps_hysteresis_threshold);
		apds9130_set_piht(client, APDS9130_PS_MAX_THRESHOLD);

		SENSOR_LOG("far-to-near detected. pdata = %d", data->ps_data);
	} else if (data->ps_data <= data->pilt) {
		/* near-to-far detected */
		data->ps_detection = 0;

		input_report_abs(data->input_dev_ps, ABS_DISTANCE, PROX_INPUT_FAR);
		input_sync(data->input_dev_ps);

		apds9130_set_pilt(client, APDS9130_PS_MIN_THRESHOLD);
		apds9130_set_piht(client, data->ps_threshold);

		SENSOR_LOG("near-to-far detected. pdata = %d", data->ps_data);
	} else {
		SENSOR_LOG("else detected. pdata = %d", data->ps_data);
	}
}

#ifdef APDS9930_ALS_SENSOR_ENABLE
static void apds9930_change_als_threshold(struct i2c_client *client)
{
	struct apds9130_data *data = i2c_get_clientdata(client);
	int v;

	apds9130_set_pers(client, APDS9130_PPERS_2 | APDS9930_APERS_2);
	/* repeat this because of the first interrupt forced */

	data->als_cdata0 =	i2c_smbus_read_word_data(client,
			CMD_WORD | APDS9930_CDATA0L_REG);

	/*
	* check PS under sunlight
	* PS was previously in far-to-near condition
	*/
	v = 1024 * (256 - data->atime);
	v = (v * 75) / 100;
	if ((data->ps_detection == 1) && (data->als_cdata0 > v)) {
		/* need to inform input event as there will be no interrupt from PS */
		/* near-to-far detected */
		data->ps_detection = 0;

		input_report_abs(data->input_dev_ps, ABS_DISTANCE, PROX_INPUT_FAR);
		input_sync(data->input_dev_ps);

		apds9130_set_pilt(client, APDS9130_PS_MIN_THRESHOLD);
		apds9130_set_piht(client, data->ps_threshold);

		SENSOR_LOG("near-to-far enforced. cdata = %d", data->als_cdata0);
	}

	SENSOR_LOG ("Change ALS Threshold : Read CData: %d", data->als_cdata0);

	if ((data->als_cdata0 > data->ailt) && (data->als_cdata0 >= data->aiht)) {
		/* strong light detected */
		data->als_detection = 1;
#ifndef CONFIG_MACH_MSM8916_C50_CRK_US
		apds9130_set_ailt(client, data->als_lower_threshold);
		apds9130_set_aiht(client, 65535);
#endif

		SENSOR_LOG("Dark -> Bright detected ");

		if(apds9930_lux_change_cb) {
			apds9930_lux_change_cb(1);
		}
	} else if ((data->als_cdata0 <= data->ailt) && (data->als_cdata0 < data->aiht)) {
		/* dark detected */
		data->als_detection = 0;

#ifndef CONFIG_MACH_MSM8916_C50_CRK_US
		apds9130_set_ailt(client, 0);
		apds9130_set_aiht(client, data->als_upper_threshold);
#endif

		SENSOR_LOG("Bright -> Dark detected ");

		if(apds9930_lux_change_cb) {
			apds9930_lux_change_cb(0);
		}
	}
}
#endif

static void apds9130_reschedule_work(struct apds9130_data *data,
				     unsigned long delay)
{
	int ret;
	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	cancel_delayed_work(&data->dwork);
	ret = queue_delayed_work(apds9130_workqueue, &data->dwork, delay);
	if (ret < 0)
		SENSOR_ERR("queue_work fail, ret = %d ", ret);
}

#ifdef APDS9130_POLLING_MODE_ENABLE
/* PS polling routine */
static void apds9130_ps_polling_work_handler(struct work_struct *work)
{
	struct apds9130_data *data = container_of(work, struct apds9130_data,
				     ps_dwork.work);
	struct i2c_client *client = data->client;
	int status;

	status = i2c_smbus_read_byte_data(client, CMD_BYTE | APDS9130_STATUS_REG);
	data->ps_data = i2c_smbus_read_word_data(client,
			CMD_WORD | APDS9130_PDATAL_REG);

	/* check PS under bright light */
	if ((data->ps_data > data->ps_threshold) && (data->ps_detection == 0)
	    && ((status & 0x40) == 0x00)) {
		/* far-to-near detected */
		data->ps_detection = 1;

		input_report_abs(data->input_dev_ps, ABS_DISTANCE, PROX_INPUT_NEAR);
		input_sync(data->input_dev_ps);

		SENSOR_LOG("far-to-near detected");
	} else if ((data->ps_data < data->ps_hysteresis_threshold)
		   && (data->ps_detection == 1) && ((status & 0x40) == 0x00)) {
		/* PS was previously in far-to-near condition */
		/* near-to-far detected */
		data->ps_detection = 0;

		input_report_abs(data->input_dev_ps, ABS_DISTANCE, PROX_INPUT_FAR);
		input_sync(data->input_dev_ps);

		SENSOR_LOG("near-to-far detected");
	} else {
		SENSOR_LOG("Triggered by background ambient noise");
	}

	if ((status & 0x40) == 0x40) {	/* need to clear psat bit if it is set */
		apds9130_set_command(client, 0);	/* 0 = CMD_CLR_PS_INT */
	}

	schedule_delayed_work(&data->ps_dwork,
			      msecs_to_jiffies(data->ps_poll_delay));	/* restart timer*/
}
#endif

/* PS interrupt routine */
static void apds9130_work_handler(struct work_struct *work)
{
	struct apds9130_data *data = container_of(work, struct apds9130_data,
				     dwork.work);
	struct i2c_client *client = data->client;
	int status;
	int enable;

	status = i2c_smbus_read_byte_data(client, CMD_BYTE | APDS9130_STATUS_REG);
	if (status < 0) {
		SENSOR_ERR("I2C Read Fail, status = %d", status);
		return;
	}
	enable = i2c_smbus_read_byte_data(client, CMD_BYTE | APDS9130_ENABLE_REG);
	if (status < 0) {
		SENSOR_ERR("I2C Read Fail, enable = %d", status);
		return;
	}

	i2c_smbus_write_byte_data(client, CMD_BYTE | APDS9130_ENABLE_REG, 1);	/* disable 9130 first */

	SENSOR_LOG("status = %x ", status);
	data->ps_sat = (status & 0x40);

	if ((status & enable & 0x30) == 0x30) {
		/* both PS and ALS are interrupted */
		SENSOR_LOG("Both PS ALS Interrupted");

#ifdef APDS9930_ALS_SENSOR_ENABLE
		apds9930_change_als_threshold(client);
#endif
		apds9130_change_ps_threshold(client);

		apds9130_set_command(client, 2);	/* 2 = CMD_CLR_PS_ALS_INT */

	} else if ((status & enable & 0x20) == 0x20) {
		/* only PS is interrupted */
		apds9130_change_ps_threshold(client);	/* far-to-near */

		apds9130_set_command(client, 0);	/* 0 = CMD_CLR_PS_INT */
	} else if ((status & enable & 0x10) == 0x10) {
		SENSOR_LOG("ALS Interrupted");
#ifdef APDS9930_ALS_SENSOR_ENABLE
		apds9930_change_als_threshold(client);
#endif
		apds9130_set_command(client, 1);	/* 1 = CMD_CLR_ALS_INT */
	} else {
		SENSOR_ERR("Unknown interrupt");
		apds9130_set_command(client, 0);	/* 0 = CMD_CLR_PS_INT */
	}

	i2c_smbus_write_byte_data(client, CMD_BYTE | APDS9130_ENABLE_REG, data->enable);
}

/* assume this is ISR */
static irqreturn_t apds9130_interrupt(int vec, void *info)
{
	struct i2c_client *client = (struct i2c_client *)info;
	struct apds9130_data *data = i2c_get_clientdata(client);
	int tmp = -1;
	tmp = atomic_read(&data->i2c_status);

	SENSOR_LOG("==> apds9130_interrupt");

	if (wake_lock_active(&data->ps_wlock))
		wake_unlock(&data->ps_wlock);
	wake_lock_timeout(&data->ps_wlock, 2 * HZ);

	if (tmp == APDS9130_STATUS_SUSPEND) {
		atomic_set(&data->i2c_status, APDS9130_STATUS_QUEUE_WORK);
		SENSOR_LOG("i2c_status = %d ", tmp);
	} else {
		if (tmp == APDS9130_STATUS_RESUME) {
			SENSOR_LOG("queue_work , i2c_status = %d ", tmp);
			apds9130_reschedule_work(data, 0);
		}
		SENSOR_LOG("i2c_status = %d  ", tmp);
	}

	return IRQ_HANDLED;
}

#ifdef APDS9130_POLLING_MODE_ENABLE
static int apds9130_set_ps_poll_delay(struct i2c_client *client,
				      unsigned int val)
{
	struct apds9130_data *data = i2c_get_clientdata(client);
	int ret;
	int wtime = 0;

	SENSOR_LOG(": %d", val);

	if ((val != APDS9130_PS_POLL_SLOW) && (val != APDS9130_PS_POLL_MEDIUM)
	    && (val != APDS9130_PS_POLL_FAST)) {
		SENSOR_ERR(":invalid value=%d", val);
		return -1;
	}

	if (val == APDS9130_PS_POLL_FAST) {
		data->ps_poll_delay = 50;	/* 50ms */
		wtime = 0xEE;	/* ~50ms */
	} else if (val == APDS9130_PS_POLL_MEDIUM) {
		data->ps_poll_delay = 100;	/* 100ms */
		wtime = 0xDC;	// ~100ms
	} else {	/* APDS9130_PS_POLL_SLOW */
		data->ps_poll_delay = 1000;	/* 1000ms */
		wtime = 0x00;	/* 696ms */
	}

	ret = apds9130_set_wtime(client, wtime);
	if (ret < 0)
		return ret;

	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	cancel_delayed_work(&data->ps_dwork);
	flush_delayed_work(&data->ps_dwork);
	queue_delayed_work(apds9130_workqueue, &data->ps_dwork,
			   msecs_to_jiffies(data->ps_poll_delay));

	return 0;
}
#endif

static int apds9130_enable_ps_sensor(struct i2c_client *client, int val)
{
	struct apds9130_data *data = i2c_get_clientdata(client);
	struct apds9130_platform_data *pdata = data->platform_data;
	int err = 0;
	int ret = 0;

	SENSOR_LOG("enable ps senosr Enter, val = ( %d)", val);

	if ((val != APDS9130_DISABLE_PS) && (val != APDS9130_ENABLE_PS_WITH_INT)
	    && (val != APDS9130_ENABLE_PS_NO_INT)) {
		SENSOR_ERR("invalid value=%d", val);
		return -1;
	}

	/* APDS9130_DISABLE_PS (0) = Disable PS */
	/* APDS9130_ENABLE_PS_WITH_INT (1) = Enable PS with interrupt enabled */
	/* APDS9130_ENABLE_PS_NO_INT (2) = Enable PS without interrupt enabled */
	mutex_lock(&data->enable_lock);
	if (val == APDS9130_ENABLE_PS_WITH_INT || val == APDS9130_ENABLE_PS_NO_INT) {

#if defined(APDS9130_PROXIMITY_CAL)
		data->cross_talk = apds9130_read_crosstalk_data_fs();

#if 1
		/* LGE_CHANGE. 2014.2.27. dongwon.you@lge.com. Fixed for CPK fail in MS323(W5 MPCS) */
		/* Fixed for CPK fail when cross_talk is 0. Don't change to default value when cross talk is 0.*/
		if (data->cross_talk <= 0) {
			SENSOR_ERR("!!! Cross talk value is 0. cross_talk:%d . Set value to 0",
			      data->cross_talk);
			data->cross_talk = 0;
		} else if (data->cross_talk > data->platform_data->crosstalk_max) {
			SENSOR_ERR("!!! ERROR!!! Cross talk value is lager than max value. cross_talk :%d . Set default to %d",
			      data->cross_talk, PS_DEFAULT_CROSS_TALK);
			data->cross_talk = PS_DEFAULT_CROSS_TALK;
		}
#else
		if (data->cross_talk <= 0
		    || data->cross_talk > data->platform_data->crosstalk_max)
			data->cross_talk = PS_DEFAULT_CROSS_TALK;
		SENSOR_LOG("Cross_talk : %d", data->cross_talk);

#endif
		apds9130_Set_PS_Threshold_Adding_Cross_talk(client, data->cross_talk);

		SENSOR_LOG("apds9130_Set_PS_Threshold_Adding_Cross_talk = %d", data->cross_talk);

#ifdef APDS9930_ALS_SENSOR_ENABLE
		apds9930_set_als_threshold(client, 0);
#endif

#endif

		if (val == APDS9130_ENABLE_PS_WITH_INT) {
			if (pdata->power_on) {
				SENSOR_LOG("power_on,	 val = %d ", val);
				pdata->power_on(client, true);
			}

			mdelay(5);
			err = apds9130_init_client(client);
			if (err < 0) {
				SENSOR_ERR("Proximity INIT FAIL err = %d ", err);
				ret = -1;
				goto unlock;
			}
			/*turn on p sensor*/
			err = apds9130_set_enable(client, 0); /* Power Off */
			if (err < 0) {
				SENSOR_ERR("first disable fail, err = %d", err);
				ret = -1;
				goto unlock;
			}

			/* init threshold for proximity */
			apds9130_set_pilt(client, data->ps_threshold);
			apds9130_set_piht(client, data->ps_hysteresis_threshold);

#ifdef APDS9930_ALS_SENSOR_ENABLE
			apds9930_set_ailt(client, 0);
			apds9930_set_aiht(client, data->als_upper_threshold);

			err = apds9130_set_enable(client, 0x3D|0x02); /* enable PS interrupt + ALS interrupt */
#else
			err = apds9130_set_enable(client, 0x2D);
#endif

			if (err < 0) {
				SENSOR_ERR("set_enable fail, err = %d", err);
				ret = -1;
				goto unlock;
			}

			/*
			if(!(wake_lock_active(&data->ps_wlock)))
			    wake_lock(&data->ps_wlock);
			*/
			data->enable_ps_sensor = val;
#ifdef APDS9130_POLLING_MODE_ENABLE
			/*
			 * If work is already scheduled then subsequent schedules will not
			 * change the scheduled time that's why we have to cancel it first.
			 */
			cancel_delayed_work(&data->ps_dwork);
			flush_delayed_work(&data->ps_dwork);
#endif
		} else { // val == APDS9130_ENABLE_PS_NO_INT
			if (pdata->power_on)
				pdata->power_on(client, true);
			mdelay(5);

			err = apds9130_init_client(client);
			if (err < 0) {
				SENSOR_ERR("Proximity INIT FAIL err = %d ", err);
				ret = -1;
				goto unlock;
			}
			err = apds9130_set_enable(client, 0x0D);	 /* no PS interrupt */
			if (err < 0) {
				SENSOR_ERR("Polling enable fail, err = %d", err);
				ret = -1;
				goto unlock;
			}

			data->enable_ps_sensor = val;

#ifdef APDS9130_POLLING_MODE_ENABLE
			/*
			 * If work is already scheduled then subsequent schedules will not
			 * change the scheduled time that's why we have to cancel it first.
			 */
			cancel_delayed_work(&data->ps_dwork);
			flush_delayed_work(&data->ps_dwork);
			schedule_delayed_work(&data->ps_dwork, msecs_to_jiffies(data->ps_poll_delay));
#endif
		}
	} else { // val == APDS9130_DISABLE_PS
		/*
		if(wake_lock_active(&data->ps_wlock))
			wake_unlock(&data->ps_wlock);
		    */
		flush_delayed_work(&data->dwork);
		cancel_delayed_work(&data->dwork);
		err = apds9130_set_enable(client, 0);
		if (err < 0) {
			SENSOR_ERR("second disable fail, err = %d", err);
			ret = -1;
			goto unlock;
		}

		mdelay(5);
		if (pdata->power_on) {
			SENSOR_LOG("power_off,  val = %d ", val);
			pdata->power_on(client, false);
		}
		data->enable_ps_sensor = 0;

		input_report_abs(data->input_dev_ps, ABS_DISTANCE, PROX_INPUT_UNKNOWN);

#ifdef APDS9130_POLLING_MODE_ENABLE
		/*
		 * If work is already scheduled then subsequent schedules will not
		 * change the scheduled time that's why we have to cancel it first.
		 */
		cancel_delayed_work(&data->ps_dwork);
		flush_delayed_work(&data->ps_dwork);
#endif
	}

unlock:
	mutex_unlock(&data->enable_lock);
	SENSOR_LOG("enable ps senosr End, val = %d", val);
	return ret;
}

/*
 * SysFS support
 */
static ssize_t apds9130_show_pdata(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct apds9130_data *data = dev_get_drvdata(dev);
	int pdata;

	mutex_lock(&data->update_lock);
	pdata = i2c_smbus_read_word_data(data->client, CMD_WORD | APDS9130_PDATAL_REG);
	mutex_unlock(&data->update_lock);

	return sprintf(buf, "%d\n", pdata);
}

static DEVICE_ATTR(pdata, S_IRUGO | S_IWUSR | S_IWGRP/*|S_IWOTH*/,
		   apds9130_show_pdata, NULL);

/*
 * SysFS support
 */
#ifdef APDS9930_ALS_SENSOR_ENABLE
static ssize_t apds9130_show_ch0data(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct apds9130_data *data = dev_get_drvdata(dev);
	int ch0data;

	mutex_lock(&data->update_lock);
	ch0data = i2c_smbus_read_word_data(data->client, CMD_WORD | APDS9930_CDATA0L_REG );
	mutex_unlock(&data->update_lock);

	return sprintf(buf, "%d\n", ch0data);
}

static DEVICE_ATTR(ch0data, S_IRUGO, apds9130_show_ch0data, NULL);

static ssize_t apds9130_show_ch1data(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct apds9130_data *data = dev_get_drvdata(dev);
	int ch1data;

	mutex_lock(&data->update_lock);
	ch1data = i2c_smbus_read_word_data(data->client, CMD_WORD | APDS9930_CDATA1L_REG );
	mutex_unlock(&data->update_lock);

	return sprintf(buf, "%d\n", ch1data);
}

static DEVICE_ATTR(ch1data, S_IRUGO, apds9130_show_ch1data, NULL);
#endif

static ssize_t apds9130_show_enable_ps_sensor(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct apds9130_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", data->enable_ps_sensor);
}

static ssize_t apds9130_store_enable_ps_sensor(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds9130_data *data = dev_get_drvdata(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int err = 0;

	SENSOR_LOG("enable ps senosr ( %ld)", val);

	if ((val != 0) && (val != 1)) {
		SENSOR_ERR("store unvalid value=%ld", val);
		return count;
	}

	err = apds9130_enable_ps_sensor(data->client, val);
	if (err < 0) {
		SENSOR_ERR("proximity sensor enable failed, err = %d", err);
		return count;
	}

	return count;
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP/*|S_IWOTH*/,
		   apds9130_show_enable_ps_sensor, apds9130_store_enable_ps_sensor);

#ifdef APDS9130_POLLING_MODE_ENABLE
static ssize_t apds9130_show_ps_poll_delay(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct apds9130_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", data->ps_poll_delay);
}

static ssize_t apds9130_store_ps_poll_delay(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds9130_data *data = dev_get_drvdata(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	if (data->enable_ps_sensor ==
	    APDS9130_ENABLE_PS_NO_INT)  /* only when ps is in polling mode*/
		apds9130_set_ps_poll_delay(data->client, val);
	else
		return 0;

	return count;
}

static DEVICE_ATTR(ps_poll_delay, S_IWUSR | S_IRUGO,
		   apds9130_show_ps_poll_delay, apds9130_store_ps_poll_delay);
#endif

static ssize_t apds9130_show_ppcount(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct apds9130_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", data->platform_data->ppcount);
}

static ssize_t apds9130_store_ppcount(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds9130_data *data = dev_get_drvdata(dev);
	int err;
	unsigned int val = simple_strtoul(buf, NULL, 10);

	err = apds9130_set_ppcount(data->client, val);
	if (err < 0) {
		SENSOR_ERR("apds9130_set_ppcount failed to set ppcount %d", val);
		return err;
	}
	return count;
}
static DEVICE_ATTR(ppcount, S_IWUSR | S_IRUGO, apds9130_show_ppcount,
		   apds9130_store_ppcount);
/*[LGSI_SP4_BSP_END][kirankumar.vm@lge.com] 31-10-2012 Added sys Fs entry for PPcount*/
#if defined(APDS9130_PROXIMITY_CAL)
static ssize_t apds9130_show_control(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct apds9130_data *data = dev_get_drvdata(dev);
	int control = 0;

	control = i2c_smbus_read_byte_data(data->client,
					   CMD_BYTE | APDS9130_CONTROL_REG);
	if (control < 0) {
		dev_err(&data->client->dev, "%s: i2c error %d in reading reg 0x%x\n", __func__, control,
			CMD_BYTE | APDS9130_CONTROL_REG);
		return control;
	}

	return sprintf(buf, "%d\n", control);
}

static ssize_t apds9130_store_control(struct device *dev,
				      struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds9130_data *data = dev_get_drvdata(dev);
	int ret = 0;
	unsigned long val;

	val = simple_strtoul(buf, NULL, 10);

	ret = apds9130_set_control(data->client, val);

	if (ret < 0)
		return ret;

	return count;
}
static DEVICE_ATTR(control,  S_IWUSR | S_IRUGO , apds9130_show_control,
		   apds9130_store_control);
#endif

/*[LGSI_SP4_BSP_BEGIN][kirankumar.vm@lge.com] Added Sys Fs access to show proximity status for Testmode*/
static ssize_t apds9130_show_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct apds9130_data *data = dev_get_drvdata(dev);
	unsigned int show = PROX_INPUT_FAR;

	if (data->enable_ps_sensor) {
		if (data->ps_detection)
			show = PROX_INPUT_NEAR;
		else
			show = PROX_INPUT_FAR;
	} else {
		show = PROX_INPUT_FAR;	/*If not enabled show it to far by default*/
	}

	return sprintf(buf, "%d\n", show);
}

static DEVICE_ATTR(value, S_IWUSR | S_IRUGO , apds9130_show_show, NULL);
/*[LGSI_SP4_BSP_END][kirankumar.vm@lge.com]*/

static ssize_t apds9130_show_pdrive(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct apds9130_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", data->platform_data->pdrive);
}

static ssize_t apds9130_store_pdrive(struct device *dev,
				     struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds9130_data *data = dev_get_drvdata(dev);
	unsigned int rdata;

	sscanf(buf, "%d", &rdata);

	if (data->client != NULL) {
		data->platform_data->pdrive = rdata;
		apds9130_set_control(data->client,
				     data->platform_data->pdrive |
				     APDS9130_PRX_IR_DIOD | APDS9130_PGAIN_4X);
	} else {
		return -1;
	}

	return count;
}

static DEVICE_ATTR(pdrive, S_IRUGO | S_IWUSR, apds9130_show_pdrive,
		   apds9130_store_pdrive);

static ssize_t apds9130_show_pilt(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct apds9130_data *data = dev_get_drvdata(dev);

	if (data->client != NULL)
		return sprintf(buf, "%d\n", data->pilt);

	return -1;
}

static ssize_t apds9130_store_pilt(struct device *dev,
				   struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds9130_data *data = dev_get_drvdata(dev);
	unsigned int rdata;

	sscanf(buf, "%d", &rdata);
#if 0
#ifdef APDS9190_TUNE
	g_pilt = rdata;
#endif
#endif
	if (data->client != NULL)
		apds9130_set_pilt(data->client, rdata);
	else
		return -1;

	return count;
}

static DEVICE_ATTR(pilt, S_IRUGO | S_IWUSR, apds9130_show_pilt,
		   apds9130_store_pilt);


static ssize_t apds9130_show_piht(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct apds9130_data *data = dev_get_drvdata(dev);

	if (data->client != NULL)
		return sprintf(buf, "%d\n", data->piht);
	else
		return -1;
}

static ssize_t apds9130_store_piht(struct device *dev,
				   struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds9130_data *data = dev_get_drvdata(dev);
	unsigned int rdata;

	sscanf(buf, "%d", &rdata);
#if 0
#ifdef APDS9190_TUNE
	g_piht = rdata;
#endif
#endif
	if (data->client != NULL)
		apds9130_set_piht(data->client, rdata);
	else
		return -1;

	return count;
}

static DEVICE_ATTR(piht, S_IRUGO | S_IWUSR, apds9130_show_piht,
		   apds9130_store_piht);

#ifdef APDS9930_ALS_SENSOR_ENABLE
/* ALS */
static ssize_t apds9930_show_ailt(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct apds9130_data *data = dev_get_drvdata(dev);

	if (data->client != NULL)
		return sprintf(buf, "%d\n", data->ailt);

	return -1;
}

static ssize_t apds9930_store_ailt(struct device *dev,
				   struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds9130_data *data = dev_get_drvdata(dev);
	unsigned int rdata;

	sscanf(buf, "%d", &rdata);
#if 0
#ifdef APDS9190_TUNE
	g_pilt = rdata;
#endif
#endif
	if (data->client != NULL)
		apds9930_set_ailt(data->client, rdata);
	else
		return -1;

	return count;
}

static DEVICE_ATTR(ailt, S_IRUGO | S_IWUSR, apds9930_show_ailt,
		   apds9930_store_ailt);

static ssize_t apds9930_show_aiht(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct apds9130_data *data = dev_get_drvdata(dev);

	if (data->client != NULL)
		return sprintf(buf, "%d\n", data->aiht);
	else
		return -1;
}

static ssize_t apds9930_store_aiht(struct device *dev,
				   struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds9130_data *data = dev_get_drvdata(dev);
	unsigned int rdata;

	sscanf(buf, "%d", &rdata);
#if 0
#ifdef APDS9190_TUNE
	g_piht = rdata;
#endif
#endif
	if (data->client != NULL)
		apds9930_set_aiht(data->client, rdata);
	else
		return -1;

	return count;
}

static DEVICE_ATTR(aiht, S_IRUGO | S_IWUSR, apds9930_show_aiht,
		   apds9930_store_aiht);
#endif

static ssize_t apds9130_show_near_offset(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct apds9130_data *data = dev_get_drvdata(dev);

	SENSOR_LOG("get near_offset = %u ", data->platform_data->near_offset);

	return sprintf(buf, "%d\n", data->platform_data->near_offset);
}

static ssize_t apds9130_store_near_offset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds9130_data *data = dev_get_drvdata(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	SENSOR_LOG("set near_offset = %ld ", val);
	data->platform_data->near_offset = val;
	return count;
}

static DEVICE_ATTR(near_offset, S_IRUGO | S_IWUSR, apds9130_show_near_offset,
		   apds9130_store_near_offset);


static ssize_t apds9130_show_far_offset(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct apds9130_data *data = dev_get_drvdata(dev);

	SENSOR_LOG("get far_offset = %u ", data->platform_data->far_offset);

	return sprintf(buf, "%d\n", data->platform_data->far_offset);
}

static ssize_t apds9130_store_far_offset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds9130_data *data = dev_get_drvdata(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	SENSOR_LOG("set far_offset = %ld ", val);
	data->platform_data->far_offset = val;
	return count;
}

static DEVICE_ATTR(far_offset, S_IRUGO | S_IWUSR, apds9130_show_far_offset,
		   apds9130_store_far_offset);

#ifdef APDS9930_ALS_SENSOR_ENABLE
static ssize_t apds9930_show_bright_threshold(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct apds9130_data *data = dev_get_drvdata(dev);

	SENSOR_LOG("get bright_threshold = %u ", data->platform_data->als_upper_threshold);

	return sprintf(buf, "%d\n", data->platform_data->als_upper_threshold);
}

static ssize_t apds9930_store_bright_threshold(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds9130_data *data = dev_get_drvdata(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	SENSOR_LOG("set bright_threshold = %ld ", val);
	data->platform_data->als_upper_threshold = val;
	return count;
}

static DEVICE_ATTR(bright_threshold, S_IRUGO | S_IWUSR, apds9930_show_bright_threshold,
		   apds9930_store_bright_threshold);


static ssize_t apds9930_show_dark_threshold(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct apds9130_data *data = dev_get_drvdata(dev);

	SENSOR_LOG("get dark_threshold = %u ", data->platform_data->als_lower_threshold);

	return sprintf(buf, "%d\n", data->platform_data->als_lower_threshold);
}

static ssize_t apds9930_store_dark_threshold(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds9130_data *data = dev_get_drvdata(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	SENSOR_LOG("set dark_threshold = %ld ", val);
	data->platform_data->als_lower_threshold = val;
	return count;
}


static DEVICE_ATTR(dark_threshold, S_IRUGO | S_IWUSR, apds9930_show_dark_threshold,
		   apds9930_store_dark_threshold);
#endif

static ssize_t apds9130_show_crosstalk_max(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct apds9130_data *data = dev_get_drvdata(dev);

	SENSOR_LOG("get crosstalk_max = %u ", data->platform_data->crosstalk_max);

	return sprintf(buf, "%d\n", data->platform_data->crosstalk_max);
}

static ssize_t apds9130_store_crosstalk_max(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds9130_data *data = dev_get_drvdata(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	SENSOR_LOG("crosstalk_max = %ld ", val);
	data->platform_data->crosstalk_max = val;
	return count;
}

static DEVICE_ATTR(crosstalk_max, S_IRUGO | S_IWUSR,
		   apds9130_show_crosstalk_max, apds9130_store_crosstalk_max);

static struct attribute *apds9130_attributes[] = {
	&dev_attr_pdata.attr,
#ifdef APDS9930_ALS_SENSOR_ENABLE
	&dev_attr_ch0data.attr,
	&dev_attr_ch1data.attr,
#endif
	&dev_attr_enable.attr,
#ifdef APDS9130_POLLING_MODE_ENABLE
	&dev_attr_ps_poll_delay.attr,
#endif
#if defined(APDS9130_PROXIMITY_CAL)
	&dev_attr_control.attr,
	&dev_attr_run_calibration.attr,
	&dev_attr_prox_cal_data.attr,
#endif
	&dev_attr_value.attr, /*[LGSI_SP4_BSP][kirankumar.vm@lge.com] Added Sys Fs access to show proximity status for Testmode*/
	&dev_attr_ppcount.attr,
	&dev_attr_pdrive.attr,/*[LGE_BSP][yunmo.yang@lge.com]add pDrive sysfs Entry*/
	&dev_attr_pilt.attr,
	&dev_attr_piht.attr,
	/*add*/
	&dev_attr_near_offset.attr,
	&dev_attr_far_offset.attr,
	&dev_attr_crosstalk_max.attr,
#ifdef APDS9930_ALS_SENSOR_ENABLE
	&dev_attr_bright_threshold.attr,
	&dev_attr_dark_threshold.attr,
	&dev_attr_ailt.attr,
	&dev_attr_aiht.attr,
#endif
	NULL
};

static const struct attribute_group apds9130_attr_group = {
	.attrs = apds9130_attributes,
};

/*
 * Initialization function
 */

static int apds9130_init_client(struct i2c_client *client)
{
	struct apds9130_data *data = i2c_get_clientdata(client);
	int err;
	int id;
	SENSOR_FUN();
	err = apds9130_set_enable(client, 0);

	if (err < 0)
		return err;

	/*data->pDrive = APDS9130_PDRVIE_100MA; */

	id = i2c_smbus_read_byte_data(client, CMD_BYTE | APDS9130_ID_REG);

	if (id == 0x39) {
		SENSOR_LOG("APDS-9130 ");
	} else {
		SENSOR_ERR("Not APDS-9130 %x", id);
		return -EIO;
	}

#ifdef APDS9930_ALS_SENSOR_ENABLE
	err = apds9930_set_atime(client, 0xED);	/* 0x6d=400ms , 0xDB=100ms, 0xED=50ms als  integration time*/
	if (err < 0) return err;
#endif

	err = apds9130_set_ptime(client, 0xFF);	/* 2.72ms Prox integration time*/
	if (err < 0) return err;

	err = apds9130_set_wtime(client, 0xED);	/* 0xED=50ms, 0xDC=100ms Wait time for POLL_MEDIUM */
	if (err < 0) return err;

	err = apds9130_set_ppcount(client, data->platform_data->ppcount);
	if (err < 0) return err;

	err = apds9130_set_config(client, 0);		/* no long wait */
	if (err < 0) return err;

#ifdef APDS9930_ALS_SENSOR_ENABLE
	err = apds9130_set_control(client,
					data->platform_data->pdrive | APDS9130_PRX_IR_DIOD |
					APDS9130_PGAIN_4X | APDS9930_AGAIN_1X);
#else
	err = apds9130_set_control(client,
					data->platform_data->pdrive | APDS9130_PRX_IR_DIOD |
					APDS9130_PGAIN_4X);
#endif
	if (err < 0) return err;

	err = apds9130_set_pilt(client, 0);		/* init threshold for proximity */
	if (err < 0) return err;

	err = apds9130_set_piht(client, APDS9130_PS_DETECTION_THRESHOLD);
	if (err < 0) return err;

	/* Force PS interrupt every PS conversion cycle to get the first interrupt */
#ifdef APDS9930_ALS_SENSOR_ENABLE
	err = apds9130_set_pers(client, APDS9130_PPERS_0 | APDS9930_APERS_0);
#else
	err = apds9130_set_pers(client, APDS9130_PPERS_0);
#endif

	if (err < 0) return err;

	/* sensor is in disabled mode but all the configurations are preset */
	/* Temp block the below code as no need to set cross talk threshold during proximity OFF state [LGSI_SP4_BSP][kirankumar.vm@lge.com]
	#if defined(APDS9130_PROXIMITY_CAL)
		err = apds9130_set_enable(client,0);
		if(err < 0){
			PINFO("%s, enable set Fail");
			return err;
		}
	#endif
	*/
	return 0;
}

#ifdef CONFIG_OF
static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0) ?
	       regulator_set_optimum_mode(reg, load_uA) : 0;
}

static int sensor_regulator_configure(struct apds9130_data *data, bool on)
{
	struct i2c_client *client = data->client;
	struct apds9130_platform_data *pdata = data->platform_data;
	int rc;

	if (on == false)
		goto hw_shutdown;

	pdata->vcc_ana = regulator_get(&client->dev, "Avago,vdd_ana");
	if (IS_ERR(pdata->vcc_ana)) {
		rc = PTR_ERR(pdata->vcc_ana);
		dev_err(&client->dev,
			"Regulator get failed vcc_ana rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(pdata->vcc_ana) > 0) {
		rc = regulator_set_voltage(pdata->vcc_ana, pdata->vdd_ana_supply_min,
					   pdata->vdd_ana_supply_max);

		if (rc) {
			dev_err(&client->dev,
				"regulator set_vtg failed rc=%d\n", rc);
			goto error_set_vtg_vcc_ana;
		}
	}
	if (pdata->digital_pwr_regulator) {
		pdata->vcc_dig = regulator_get(&client->dev, "Avago,vddio_dig");
		if (IS_ERR(pdata->vcc_dig)) {
			rc = PTR_ERR(pdata->vcc_dig);
			dev_err(&client->dev,
				"Regulator get dig failed rc=%d\n", rc);
			goto error_get_vtg_vcc_dig;
		}

		if (regulator_count_voltages(pdata->vcc_dig) > 0) {
			rc = regulator_set_voltage(pdata->vcc_dig,
						   pdata->vddio_dig_supply_min, pdata->vddio_dig_supply_max);
			if (rc) {
				dev_err(&client->dev,
					"regulator set_vtg failed rc=%d\n", rc);
				goto error_set_vtg_vcc_dig;
			}
		}
	}
	if (pdata->i2c_pull_up) {
		pdata->vcc_i2c = regulator_get(&client->dev, "Avago,vddio_i2c");
		if (IS_ERR(pdata->vcc_i2c)) {
			rc = PTR_ERR(pdata->vcc_i2c);
			dev_err(&client->dev,
				"Regulator get failed rc=%d\n", rc);
			goto error_get_vtg_i2c;
		}
		if (regulator_count_voltages(pdata->vcc_i2c) > 0) {
			rc = regulator_set_voltage(pdata->vcc_i2c,
						   pdata->vddio_i2c_supply_min, pdata->vddio_i2c_supply_max);
			if (rc) {
				dev_err(&client->dev,
					"regulator set_vtg failed rc=%d\n", rc);
				goto error_set_vtg_i2c;
			}
		}
	}

	return 0;

error_set_vtg_i2c:
	regulator_put(pdata->vcc_i2c);
error_get_vtg_i2c:
	if (pdata->digital_pwr_regulator)
		if (regulator_count_voltages(pdata->vcc_dig) > 0)
			regulator_set_voltage(pdata->vcc_dig, 0,
					      pdata->vddio_dig_supply_max);
error_set_vtg_vcc_dig:
	if (pdata->digital_pwr_regulator)
		regulator_put(pdata->vcc_dig);
error_get_vtg_vcc_dig:
	if (regulator_count_voltages(pdata->vcc_ana) > 0)
		regulator_set_voltage(pdata->vcc_ana, 0, pdata->vdd_ana_supply_max);
error_set_vtg_vcc_ana:
	regulator_put(pdata->vcc_ana);
	return rc;

hw_shutdown:
	if (regulator_count_voltages(pdata->vcc_ana) > 0)
		regulator_set_voltage(pdata->vcc_ana, 0, pdata->vdd_ana_supply_max);
	regulator_put(pdata->vcc_ana);
	if (pdata->digital_pwr_regulator) {
		if (regulator_count_voltages(pdata->vcc_dig) > 0)
			regulator_set_voltage(pdata->vcc_dig, 0,
					      pdata->vddio_dig_supply_max);

		regulator_put(pdata->vcc_dig);
	}
	if (pdata->i2c_pull_up) {
		if (regulator_count_voltages(pdata->vcc_i2c) > 0)
			regulator_set_voltage(pdata->vcc_i2c, 0,
					      pdata->vddio_i2c_supply_max);
		regulator_put(pdata->vcc_i2c);
	}
	return 0;
}


static int sensor_regulator_power_on(struct apds9130_data *data, bool on)
{
	struct i2c_client *client = data->client;
	struct apds9130_platform_data *pdata = data->platform_data;

	int rc;

	if (on == false)
		goto power_off;

	rc = reg_set_optimum_mode_check(pdata->vcc_ana, pdata->vdd_ana_load_ua);
	if (rc < 0) {
		dev_err(&client->dev,
			"Regulator vcc_ana set_opt failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(pdata->vcc_ana);
	if (rc) {
		dev_err(&client->dev,
			"Regulator vcc_ana enable failed rc=%d\n", rc);
		goto error_reg_en_vcc_ana;
	}

	if (pdata->digital_pwr_regulator) {
		rc = reg_set_optimum_mode_check(pdata->vcc_dig,
						pdata->vddio_dig_load_ua);
		if (rc < 0) {
			dev_err(&client->dev,
				"Regulator vcc_dig set_opt failed rc=%d\n",
				rc);
			goto error_reg_opt_vcc_dig;
		}

		rc = regulator_enable(pdata->vcc_dig);
		if (rc) {
			dev_err(&client->dev,
				"Regulator vcc_dig enable failed rc=%d\n", rc);
			goto error_reg_en_vcc_dig;
		}
	}

	if (pdata->i2c_pull_up) {
		rc = reg_set_optimum_mode_check(pdata->vcc_i2c, pdata->vddio_i2c_load_ua);
		if (rc < 0) {
			dev_err(&client->dev,
				"Regulator vcc_i2c set_opt failed rc=%d\n", rc);
			goto error_reg_opt_i2c;
		}

		rc = regulator_enable(pdata->vcc_i2c);
		if (rc) {
			dev_err(&client->dev,
				"Regulator vcc_i2c enable failed rc=%d\n", rc);
			goto error_reg_en_vcc_i2c;
		}
	}

	msleep(10);
	SENSOR_LOG("power on");
	return 0;

error_reg_en_vcc_i2c:
	if (pdata->i2c_pull_up)
		reg_set_optimum_mode_check(pdata->vcc_i2c, 0);
error_reg_opt_i2c:
	if (pdata->digital_pwr_regulator)
		regulator_disable(pdata->vcc_dig);
error_reg_en_vcc_dig:
	if (pdata->digital_pwr_regulator)
		reg_set_optimum_mode_check(pdata->vcc_dig, 0);
error_reg_opt_vcc_dig:
	regulator_disable(pdata->vcc_ana);
error_reg_en_vcc_ana:
	reg_set_optimum_mode_check(pdata->vcc_ana, 0);
	return rc;

power_off:
	reg_set_optimum_mode_check(pdata->vcc_ana, 0);
	regulator_disable(pdata->vcc_ana);
	if (pdata->digital_pwr_regulator) {
		reg_set_optimum_mode_check(pdata->vcc_dig, 0);
		regulator_disable(pdata->vcc_dig);
	}
	if (pdata->i2c_pull_up) {
		reg_set_optimum_mode_check(pdata->vcc_i2c, 0);
		regulator_disable(pdata->vcc_i2c);
	}
	msleep(50);
	SENSOR_LOG("power off");
	return 0;
}

static int sensor_platform_hw_power_on(struct i2c_client *client, bool on)
{
	sensor_regulator_power_on(i2c_get_clientdata(client), on);
	return 0;
}

static int sensor_platform_hw_init(struct i2c_client *client)
{
	struct apds9130_data *data = i2c_get_clientdata(client);
	int error;

	error = sensor_regulator_configure(data, true);

	if (gpio_is_valid(data->platform_data->irq_gpio)) {
		/* configure touchscreen irq gpio */
		error = gpio_request(data->platform_data->irq_gpio, "apds9130_irq_gpio");
		if (error) {
			dev_err(&client->dev, "unable to request gpio [%d]\n",
				data->platform_data->irq_gpio);
		}
		error = gpio_direction_input(data->platform_data->irq_gpio);
		if (error) {
			dev_err(&client->dev,
				"unable to set direction for gpio [%d]\n",
				data->platform_data->irq_gpio);
		}
		data->irq = client->irq = gpio_to_irq(data->platform_data->irq_gpio);
	} else {
		dev_err(&client->dev, "irq gpio not provided");
	}
	return 0;
}

static void sensor_platform_hw_exit(struct i2c_client *client)
{
	struct apds9130_data *data = i2c_get_clientdata(client);;

	sensor_regulator_configure(data, false);

	if (gpio_is_valid(data->platform_data->irq_gpio))
		gpio_free(data->platform_data->irq_gpio);

}

static int sensor_parse_dt(struct device *dev,
			   struct apds9130_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	int ret, err = 0;
	struct sensor_dt_to_pdata_map *itr;
	struct sensor_dt_to_pdata_map map[] = {
		{"Avago,i2c-pull-up",		&pdata->i2c_pull_up,		DT_REQUIRED,	DT_BOOL,	0},
		{"Avago,dig-reg-support",	&pdata->digital_pwr_regulator,	DT_REQUIRED,	DT_BOOL,	0},
		{"Avago,irq-gpio",		&pdata->irq_gpio,		DT_REQUIRED,	DT_GPIO,	0},
		{"Avago,vdd_ana_supply_min",	&pdata->vdd_ana_supply_min,	DT_SUGGESTED,	DT_U32,		0},
		{"Avago,vdd_ana_supply_max",	&pdata->vdd_ana_supply_max,	DT_SUGGESTED,	DT_U32,		0},
		{"Avago,vdd_ana_load_ua",	&pdata->vdd_ana_load_ua,	DT_SUGGESTED,	DT_U32,		0},
		{"Avago,vddio_dig_supply_min",	&pdata->vddio_dig_supply_min,	DT_SUGGESTED,	DT_U32,		0},
		{"Avago,vddio_dig_supply_max",	&pdata->vddio_dig_supply_max,	DT_SUGGESTED,	DT_U32,		0},
		{"Avago,vddio_dig_load_ua",	&pdata->vddio_dig_load_ua,	DT_SUGGESTED,	DT_U32,		0},
		{"Avago,vddio_i2c_supply_min",	&pdata->vddio_i2c_supply_min,	DT_SUGGESTED,	DT_U32,		0},
		{"Avago,vddio_i2c_supply_max",	&pdata->vddio_i2c_supply_max,	DT_SUGGESTED,	DT_U32,		0},
		{"Avago,vddio_i2c_load_ua",	&pdata->vddio_i2c_load_ua,	DT_SUGGESTED,	DT_U32,		0},
		/* add */
		{"Avago,ppcount",		&pdata->ppcount,		DT_SUGGESTED,	DT_U32,		0},
		{"Avago,pdrive",		&pdata->pdrive,			DT_SUGGESTED,	DT_U32,		0},
		{"Avago,near_offset",		&pdata->near_offset,		DT_SUGGESTED,	DT_U32,		0},
		{"Avago,far_offset",		&pdata->far_offset,		DT_SUGGESTED,	DT_U32,		0},
		{"Avago,crosstalk_max",		&pdata->crosstalk_max,		DT_SUGGESTED,	DT_U32,		0},
#ifdef APDS9930_ALS_SENSOR_ENABLE
		{"Avago,bright_threshold",	&pdata->als_upper_threshold,		DT_SUGGESTED,	DT_U32,		APDS9930_ALS_UPPER_THRESHOLD},
		{"Avago,dark_threshold",	&pdata->als_lower_threshold,		DT_SUGGESTED,	DT_U32,		APDS9930_ALS_LOWER_THRESHOLD},
#endif
		{NULL,				NULL,				0,		0,		0},
	};

	for (itr = map; itr->dt_name ; ++itr) {
		switch (itr->type) {
		case DT_GPIO:
			ret = of_get_named_gpio(np, itr->dt_name, 0);
			if (ret >= 0) {
				*((int *) itr->ptr_data) = ret;
				ret = 0;
			}
			break;
		case DT_U32:
			ret = of_property_read_u32(np, itr->dt_name,
						   (u32 *) itr->ptr_data);
			break;
		case DT_BOOL:
			*((bool *) itr->ptr_data) =
				of_property_read_bool(np, itr->dt_name);
			ret = 0;
			break;
		default:
			SENSOR_ERR("%d is an unknown DT entry type",
			      itr->type);
			ret = -EBADE;
		}

		if (ret) {
			*((int *)itr->ptr_data) = itr->default_val;

			if (itr->status < DT_OPTIONAL) {
				SENSOR_LOG("Missing '%s' DT entry",
				      itr->dt_name);

				/* cont on err to dump all missing entries */
				if (itr->status == DT_REQUIRED && !err)
					err = ret;
			}
		}
	}

	/* set functions of platform data */
	pdata->init = sensor_platform_hw_init;
	pdata->exit = sensor_platform_hw_exit;
	pdata->power_on = sensor_platform_hw_power_on;
	/*pdata->ppcount = 12;	//no need to set, dt_parse */

	return err;

}
#endif

/*
 * I2C init/probing/exit functions
 */

static struct i2c_driver apds9130_driver;
static int apds9130_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	int status;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct apds9130_data *data;
#ifdef CONFIG_OF
	struct apds9130_platform_data *platform_data;
#endif
	int err = 0;

	SENSOR_FUN();
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		return -EIO;
	}

	data = devm_kzalloc(&client->dev, sizeof(struct apds9130_data), GFP_KERNEL);
	if (!data) {
		return -ENOMEM;
	}

#ifdef CONFIG_OF
	if (client->dev.of_node) {
		platform_data = devm_kzalloc(&client->dev,
					     sizeof(struct apds9130_platform_data), GFP_KERNEL);
		if (!platform_data) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}
		data->platform_data = platform_data;
		client->dev.platform_data = platform_data;
		err = sensor_parse_dt(&client->dev, platform_data);
		if (err)
			return err;

	} else {
		platform_data = client->dev.platform_data;
	}
#endif

	data->client = client;
	apds9130_i2c_client = client;
	i2c_set_clientdata(client, data);

#ifdef CONFIG_OF
	/* h/w initialization */
	if (platform_data->init)
		err = platform_data->init(client);

	if (platform_data->power_on)
		err = platform_data->power_on(client, true);
#endif

	client->adapter->retries = 15;
	status = i2c_smbus_read_byte_data(client, CMD_BYTE | APDS9130_ID_REG);
	if (status != 0x39) {
		SENSOR_ERR("Not APDS-9130 %x", status);
		goto exit;
	}


	data->enable = 0;	/* default mode is standard */
	data->ps_threshold = APDS9130_PS_DETECTION_THRESHOLD;
	data->ps_hysteresis_threshold = APDS9130_PS_HYSTERESIS_THRESHOLD;
	data->ps_detection = 0;	/* default to no detection */
	data->enable_ps_sensor = 0;	/* default to 0 */
#ifdef APDS9130_POLLING_MODE_ENABLE
	data->ps_poll_delay = 100;	/* 100ms */
#endif
#if defined(APDS9130_PROXIMITY_CAL)
	data->cross_talk = PS_DEFAULT_CROSS_TALK;
#endif

	atomic_set(&data->i2c_status, APDS9130_STATUS_RESUME);

	mutex_init(&data->update_lock);
	mutex_init(&data->enable_lock);

	/* Initialize the APDS-9130 chip */
	err = apds9130_init_client(client);
	if (err)
		goto exit;

	wake_lock_init(&data->ps_wlock, WAKE_LOCK_SUSPEND, "proxi_wakelock");
	INIT_DELAYED_WORK(&data->dwork, apds9130_work_handler);
	/*client->irq = APDS9130_INT;*/

#ifdef APDS9130_POLLING_MODE_ENABLE
	/*  For polling mode*/
	INIT_DELAYED_WORK(&data->ps_dwork, apds9130_ps_polling_work_handler);
#endif
	if (request_irq(client->irq, apds9130_interrupt, IRQ_TYPE_EDGE_FALLING,
			APDS9130_DRV_NAME, (void *)client)) {
		SENSOR_ERR("Could not allocate APDS9130_INT %d !", client->irq);

		goto exit_irq_init_failed;
	}

	err = enable_irq_wake(client->irq);
	if (err) {
		SENSOR_ERR("Set irq to wakeup failed.");
		SENSOR_ERR("enable_irq_wake return val =%d", err);
		goto exit_free_irq;
	}

	data->input_dev_ps = input_allocate_device();
	if (!data->input_dev_ps) {
		err = -ENOMEM;
		SENSOR_ERR("Failed to allocate input device ps");
		goto exit_free_irq;
	}

	set_bit(EV_ABS, data->input_dev_ps->evbit);

	input_set_abs_params(data->input_dev_ps, ABS_DISTANCE, 0, 1, 0, 0);

	data->input_dev_ps->name = "proximity";
	data->input_dev_ps->uniq = APDS9130_DRV_NAME;
	data->input_dev_ps->dev.init_name = LGE_PROXIMITY_NAME;

	/*data->input_dev_ps->dev.parent = &data->client->dev;*/
	input_set_drvdata(data->input_dev_ps, data);
	/*dev_set_drvdata(&data->input_dev_ps->dev, data);*/

	err = input_register_device(data->input_dev_ps);
	if (err) {
		err = -ENOMEM;
		SENSOR_ERR("Unable to register input device ps: %s",
		      data->input_dev_ps->name);
		goto exit_free_dev_ps;
	}

	data->sw_mode = PROX_STAT_OPERATING;

	/* Register sysfs hooks */
	/*err = sysfs_create_group(&client->dev.kobj, &apds9130_attr_group);*/
	err = sysfs_create_group(&data->input_dev_ps->dev.kobj, &apds9130_attr_group);
	if (err)
		goto exit_unregister_dev_ps;

	apds9130_enable_ps_sensor(client, 0);
	SENSOR_LOG("probe end");
	return 0;

exit_unregister_dev_ps:
	input_unregister_device(data->input_dev_ps);
exit_free_dev_ps:
exit_free_irq:
	free_irq(client->irq, client);
exit_irq_init_failed:
	wake_lock_destroy(&data->ps_wlock);
	mutex_destroy(&data->update_lock);
	mutex_destroy(&data->enable_lock);
exit:
	return err;
}

static int apds9130_remove(struct i2c_client *client)
{
	struct apds9130_data *data = i2c_get_clientdata(client);
	struct apds9130_platform_data *pdata = data->platform_data;

	/* Power down the device */
	apds9130_set_enable(client, 0);

	sysfs_remove_group(&data->input_dev_ps->dev.kobj, &apds9130_attr_group);

	input_unregister_device(data->input_dev_ps);

	disable_irq_wake(client->irq);

	free_irq(client->irq, client);

	if (pdata->power_on)
		pdata->power_on(client, false);

	if (pdata->exit)
		pdata->exit(client);

	mutex_destroy(&data->update_lock);
	mutex_destroy(&data->enable_lock);

	return 0;
}

#ifdef CONFIG_PM

static int apds9130_suspend(struct i2c_client *client, pm_message_t mesg)
{
#if 1
	struct apds9130_data *data = i2c_get_clientdata(client);
	atomic_set(&data->i2c_status, APDS9130_STATUS_SUSPEND);

	SENSOR_FUN();
#else
	struct apds9130_data *data = i2c_get_clientdata(client);
	struct apds9130_platform_data *pdata = data->platform_data;

	SENSOR_LOG("suspend");

	if (pdata->power_on)
		pdata->power_on(client, false);
#endif
	return 0;
}

static int apds9130_resume(struct i2c_client *client)
{
#if 1
	struct apds9130_data *data = i2c_get_clientdata(client);
	SENSOR_FUN();

	if (data->enable_ps_sensor == 1) {
		if (atomic_read(&data->i2c_status) == APDS9130_STATUS_QUEUE_WORK) {
			SENSOR_LOG("Resume Trigger Queue");
			apds9130_reschedule_work(data, 0);
		}
	}
	atomic_set(&data->i2c_status, APDS9130_STATUS_RESUME);
#else
	struct apds9130_data *data = i2c_get_clientdata(client);
	struct apds9130_platform_data *pdata = data->platform_data;

	SENSOR_LOG("resume");

	if (pdata->power_on)
		pdata->power_on(client, true);
#endif
	return 0;
}

#else

#define apds9130_suspend	NULL
#define apds9130_resume		NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id apds9130_id[] = {
	{ "apds9130", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, apds9130_id);

#ifdef CONFIG_OF
static struct of_device_id apds9130_match_table[] = {
	{ .compatible = "avago,apds9130",},
	{ },
};
#else
#define apds9130_match_table NULL
#endif

static struct i2c_driver apds9130_driver = {
	.driver = {
		.name	= APDS9130_DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = apds9130_match_table,
	},
	.suspend = apds9130_suspend,
	.resume	= apds9130_resume,
	.probe	= apds9130_probe,
	.remove	= apds9130_remove,
	.id_table = apds9130_id,
};

static void async_sensor_init(void *data, async_cookie_t cookie)
{
	msleep(500);

	SENSOR_LOG("APDS9130 Proximity driver: initialize.");
	apds9130_workqueue = create_workqueue("proximity");
	i2c_add_driver(&apds9130_driver);
	return;
}

static int __init apds9130_init(void)
{
	async_schedule(async_sensor_init, NULL);
	return 0;
}

static void __exit apds9130_exit(void)
{
	SENSOR_LOG("APDS9130 Proximity driver: release.");

	if (apds9130_workqueue)
		destroy_workqueue(apds9130_workqueue);

	apds9130_workqueue = NULL;

	i2c_del_driver(&apds9130_driver);
}

MODULE_AUTHOR("Lee Kai Koon <kai-koon.lee@avagotech.com>");
MODULE_DESCRIPTION("APDS9130 proximity sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(apds9130_init);
module_exit(apds9130_exit);

