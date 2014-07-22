/******************** (C) COPYRIGHT 2013 STMicroelectronics ********************
*
* File Name          : lsm303c_mag.c
* Authors            : MSH - C&I BU - Application Team
*		     : Matteo Dameno (matteo.dameno@st.com)
*		     : Denis Ciocca (denis.ciocca@st.com)
*		     : Both authors are willing to be considered the contact
*		     : and update points for the driver.
* Version            : V.1.0.0
* Date               : 2013/Jun/17
* Description        : LSM303C magnetometer driver
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
********************************************************************************
*******************************************************************************/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>

#ifdef CONFIG_OF
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#endif

/*#include <linux/input/lsm303c.h>*/
#include "lsm303c.h"


#define	I2C_AUTO_INCREMENT	(0x80)
#define MS_TO_NS(x)		(x*1000000L)

#define	MAG_G_MAX_POS		983520	/** max positive value mag [ugauss] */
#define	MAG_G_MAX_NEG		983040	/** max negative value mag [ugauss] */

#define FUZZ			0
#define FLAT			0

/* Address registers */
#define REG_WHOAMI_ADDR		(0x0F)	/** Who am i address register */
#define REG_CNTRL1_ADDR		(0x20)	/** CNTRL1 address register */
#define REG_CNTRL2_ADDR		(0x21)	/** CNTRL2 address register */
#define REG_CNTRL3_ADDR		(0x22)	/** CNTRL3 address register */
#define REG_CNTRL4_ADDR		(0x23)	/** CNTRL4 address register */
#define REG_CNTRL5_ADDR		(0x24)	/** CNTRL5 address register */

#define REG_MAG_DATA_ADDR	(0x28)	/** Mag. data low address register */

/* Sensitivity */
#define SENSITIVITY_MAG_4G	146156	/**	ngauss/LSB	*/
#define SENSITIVITY_MAG_8G	292312	/**	ngauss/LSB	*/
#define SENSITIVITY_MAG_10G	365364	/**	ngauss/LSB	*/

/* ODR */
#define ODR_MAG_MASK		(0X1C)	/* Mask for odr change on mag */
#define LSM303C_MAG_ODR0_625	(0x00)	/* 0.625Hz output data rate */
#define LSM303C_MAG_ODR1_25	(0x04)	/* 1.25Hz output data rate */
#define LSM303C_MAG_ODR2_5	(0x08)	/* 2.5Hz output data rate */
#define LSM303C_MAG_ODR5	(0x0C)	/* 5Hz output data rate */
#define LSM303C_MAG_ODR10	(0x10)	/* 10Hz output data rate */
#define LSM303C_MAG_ODR20	(0x14)	/* 20Hz output data rate */
#define LSM303C_MAG_ODR40	(0x18)	/* 40Hz output data rate */
#define LSM303C_MAG_ODR80	(0x1C)	/* 80Hz output data rate */

/* Magnetic sensor mode */
#define MSMS_MASK		(0x03)	/* Mask magnetic sensor mode */
#define POWEROFF_MAG		(0x02)	/* Power Down */
#define CONTINUOS_CONVERSION	(0x00)	/* Continuos Conversion */

/* X and Y axis operative mode selection */
#define X_Y_PERFORMANCE_MASK		(0x60)
#define X_Y_LOW_PERFORMANCE		(0x00)
#define X_Y_MEDIUM_PERFORMANCE		(0x20)
#define X_Y_HIGH_PERFORMANCE		(0x40)
#define X_Y_ULTRA_HIGH_PERFORMANCE	(0x60)

/* Z axis operative mode selection */
#define Z_PERFORMANCE_MASK		(0x0c)
#define Z_LOW_PERFORMANCE		(0x00)
#define Z_MEDIUM_PERFORMANCE		(0x04)
#define Z_HIGH_PERFORMANCE		(0x08)
#define Z_ULTRA_HIGH_PERFORMANCE	(0x0c)

/* Default values loaded in probe function */
#define WHOIAM_VALUE		(0x3d)	/** Who Am I default value */
#define REG_DEF_CNTRL1		(0x60)	/** CNTRL1 default value */
#define REG_DEF_CNTRL2		(0x00)	/** CNTRL2 default value */
#define REG_DEF_CNTRL3		(0x03)	/** CNTRL3 default value */
#define REG_DEF_CNTRL4		(0x00)	/** CNTRL4 default value */
#define REG_DEF_CNTRL5		(0x40)	/** CNTRL5 default value */

#define REG_DEF_ALL_ZEROS	(0x00)


struct workqueue_struct *lsm303c_workqueue;


struct {
	unsigned int cutoff_us;
	u8 value;
} lsm303c_mag_odr_table[] = {
		{  12, LSM303C_MAG_ODR80  },
		{  25, LSM303C_MAG_ODR40   },
		{  50, LSM303C_MAG_ODR20   },
		{  100, LSM303C_MAG_ODR10 },
		{ 200, LSM303C_MAG_ODR5 },
		{ 400, LSM303C_MAG_ODR2_5},
		{ 800, LSM303C_MAG_ODR1_25},
		{ 1600, LSM303C_MAG_ODR0_625},
};

struct interrupt_enable {
	atomic_t enable;
	u8 address;
	u8 mask;
};

struct interrupt_value {
	int value;
	u8 address;
};


struct lsm303c_status {
	struct i2c_client *client;
	struct lsm303c_mag_platform_data *pdata_mag;

	struct mutex lock;
	struct work_struct input_work_mag;

	struct hrtimer hr_timer_mag;
	ktime_t ktime_mag;

	struct input_dev *input_dev_mag;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;

	atomic_t enabled_mag;

	int on_before_suspend;
	int use_smbus;

	u32 sensitivity_mag;

	u8 xy_mode;
	u8 z_mode;
#ifdef DEBUG
	u8 reg_addr;
#endif
};

static struct lsm303c_status *g_ptr_mag_stat;

static const struct lsm303c_mag_platform_data default_lsm303c_mag_pdata = {
	.poll_interval = 100,
	.min_interval = LSM303C_MAG_MIN_POLL_PERIOD_MS,
	.fs_range = LSM303C_MAG_FS_4G,
	.axis_map_x = 1,
	.axis_map_y = 0,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,
	.digital_pwr_regulator = 0,
	.irq_gpio = LSM303C_MAG_DEFAULT_INT1_GPIO,
};

struct reg_rw {
	u8 address;
	u8 default_value;
	u8 resume_value;
};

struct reg_r {
	u8 address;
	u8 value;
};

static struct status_registers {
	struct reg_r who_am_i;
	struct reg_rw cntrl1;
	struct reg_rw cntrl2;
	struct reg_rw cntrl3;
	struct reg_rw cntrl4;
	struct reg_rw cntrl5;
} status_registers = {
	.who_am_i.address = REG_WHOAMI_ADDR,
	.who_am_i.value = WHOIAM_VALUE,
	.cntrl1.address = REG_CNTRL1_ADDR,
	.cntrl1.default_value = REG_DEF_CNTRL1,
	.cntrl2.address = REG_CNTRL2_ADDR,
	.cntrl2.default_value = REG_DEF_CNTRL2,
	.cntrl3.address = REG_CNTRL3_ADDR,
	.cntrl3.default_value = REG_DEF_CNTRL3,
	.cntrl4.address = REG_CNTRL4_ADDR,
	.cntrl4.default_value = REG_DEF_CNTRL4,
	.cntrl5.address = REG_CNTRL5_ADDR,
	.cntrl5.default_value = REG_DEF_CNTRL5,
};

static int lsm303c_i2c_read(struct lsm303c_status *stat, u8 *buf, int len)
{
	int ret;
	u8 reg = buf[0];
	u8 cmd = reg;


	if (len > 1)
		cmd = (I2C_AUTO_INCREMENT | reg);
	if (stat->use_smbus) {
		if (len == 1) {
			ret = i2c_smbus_read_byte_data(stat->client, cmd);
			buf[0] = ret & 0xff;
#ifdef DEBUG
			dev_warn(&stat->client->dev,
				"i2c_smbus_read_byte_data: ret=0x%02x, len:%d ,"
				"command=0x%02x, buf[0]=0x%02x\n",
				ret, len, cmd , buf[0]);
#endif
		} else if (len > 1) {
			ret = i2c_smbus_read_i2c_block_data(stat->client,
								cmd, len, buf);
		} else
			ret = -1;

		if (ret < 0) {
			dev_err(&stat->client->dev,
				"read transfer error: len:%d, command=0x%02x\n",
				len, cmd);
			return 0;
		}
		return len;
	}

	ret = i2c_master_send(stat->client, &cmd, sizeof(cmd));
	if (ret != sizeof(cmd))
		return ret;

	return i2c_master_recv(stat->client, buf, len);
}

static int lsm303c_i2c_write(struct lsm303c_status *stat, u8 *buf, int len)
{
	int ret;
	u8 reg, value;

	if (len > 1)
		buf[0] = (I2C_AUTO_INCREMENT | buf[0]);

	reg = buf[0];
	value = buf[1];

	if (stat->use_smbus) {
		if (len == 1) {
			ret = i2c_smbus_write_byte_data(stat->client,
								reg, value);
#ifdef DEBUG
			dev_warn(&stat->client->dev,
				"i2c_smbus_write_byte_data: ret=%d, len:%d, "
				"command=0x%02x, value=0x%02x\n",
				ret, len, reg , value);
#endif
			return ret;
		} else if (len > 1) {
			ret = i2c_smbus_write_i2c_block_data(stat->client,
							reg, len, buf + 1);
			return ret;
		}
	}

	ret = i2c_master_send(stat->client, buf, len+1);
	return (ret == len+1) ? 0 : ret;
}

static int lsm303c_hw_init(struct lsm303c_status *stat)
{
	int err = -1;
	u8 buf[1];

	pr_info("%s: hw init start\n", LSM303C_MAG_DEV_NAME);

	buf[0] = status_registers.who_am_i.address;
	err = lsm303c_i2c_read(stat, buf, 1);

	if (err < 0) {
		dev_warn(&stat->client->dev,
		"Error reading WHO_AM_I: is device available/working?\n");
		goto err_firstread;
	} else
		stat->hw_working = 1;

	if (buf[0] != status_registers.who_am_i.value) {
		dev_err(&stat->client->dev,
		"device unknown. Expected: 0x%02x, Replies: 0x%02x\n",
				status_registers.who_am_i.value, buf[0]);
		err = -1;
		goto err_unknown_device;
	}

	status_registers.cntrl1.resume_value =
					status_registers.cntrl1.default_value;
	status_registers.cntrl2.resume_value =
					status_registers.cntrl2.default_value;
	status_registers.cntrl3.resume_value =
					status_registers.cntrl3.default_value;
	status_registers.cntrl4.resume_value =
					status_registers.cntrl4.default_value;
	status_registers.cntrl5.resume_value =
					status_registers.cntrl5.default_value;

	stat->xy_mode = X_Y_ULTRA_HIGH_PERFORMANCE;
	stat->z_mode = Z_ULTRA_HIGH_PERFORMANCE;
	stat->hw_initialized = 1;
	pr_info("%s: hw init done\n", LSM303C_MAG_DEV_NAME);

	return 0;

err_unknown_device:
err_firstread:
	stat->hw_working = 0;
	stat->hw_initialized = 0;
	return err;
}

static int lsm303c_mag_device_power_off(struct lsm303c_status *stat)
{
	int err;
	u8 buf[2];

	buf[0] = status_registers.cntrl3.address;
	buf[1] = ((MSMS_MASK & POWEROFF_MAG) |
		((~MSMS_MASK) & status_registers.cntrl3.resume_value));

	err = lsm303c_i2c_write(stat, buf, 1);
	if (err < 0)
		dev_err(&stat->client->dev,
			"magnetometer soft power off failed: %d\n", err);

	if (stat->pdata_mag->power_off)
		stat->pdata_mag->power_off();

	atomic_set(&stat->enabled_mag, 0);

	return 0;
}

static int lsm303c_mag_device_power_on(struct lsm303c_status *stat)
{
	int err = -1;
	u8 buf[6];

	if (stat->pdata_mag->power_on) {
		err = stat->pdata_mag->power_on(1);
		if (err < 0) {
			dev_err(&stat->client->dev,
				"magnetometer power_on failed: %d\n", err);
			return err;
		}
	}

	err = lsm303c_hw_init(stat);
	if (err < 0) {
		dev_err(&stat->client->dev, "hw init failed: %d\n", err);
		return err;
	}

	buf[0] = status_registers.cntrl1.address;
	buf[1] = status_registers.cntrl1.resume_value;
	err = lsm303c_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = status_registers.cntrl3.address;
	buf[1] = ((MSMS_MASK & CONTINUOS_CONVERSION) |
		((~MSMS_MASK) & status_registers.cntrl3.resume_value));


	err = lsm303c_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	atomic_set(&stat->enabled_mag, 1);

	return 0;

err_resume_state:
	atomic_set(&stat->enabled_mag, 0);
	dev_err(&stat->client->dev,
		"magnetometer hw power on error 0x%02x,0x%02x: %d\n",
							buf[0], buf[1], err);
	return err;
}

static int lsm303c_mag_update_fs_range(struct lsm303c_status *stat,
								u8 new_fs_range)
{
	int err = -1;
	u32 sensitivity;
	u8 updated_val;
	u8 buf[2];

	switch (new_fs_range) {
	case LSM303C_MAG_FS_4G:
		sensitivity = SENSITIVITY_MAG_4G;
		break;
	case LSM303C_MAG_FS_8G:
		sensitivity = SENSITIVITY_MAG_8G;
		break;
	case LSM303C_MAG_FS_10G:
		sensitivity = SENSITIVITY_MAG_10G;
		break;
	default:
		dev_err(&stat->client->dev,
			"invalid magnetometer fs range requested: %u\n",
								new_fs_range);
		return -EINVAL;
	}

	buf[0] = status_registers.cntrl2.address;
	err = lsm303c_i2c_read(stat, buf, 1);
	if (err < 0)
		goto error;

	status_registers.cntrl2.resume_value = buf[0];
	updated_val = (LSM303C_MAG_FS_MASK & new_fs_range);
	buf[1] = updated_val;
	buf[0] = status_registers.cntrl2.address;

	err = lsm303c_i2c_write(stat, buf, 1);
	if (err < 0)
		goto error;
	status_registers.cntrl2.resume_value = updated_val;
	stat->sensitivity_mag = sensitivity;

	return err;

error:
	dev_err(&stat->client->dev,
		"update magnetometer fs range failed 0x%02x,0x%02x: %d\n",
							buf[0], buf[1], err);
	return err;
}

static int lsm303c_mag_update_odr(struct lsm303c_status *stat,
						unsigned int poll_interval_ms)
{
	int err = -1;
	u8 config[2];
	int i;

	for (i = ARRAY_SIZE(lsm303c_mag_odr_table) - 1; i >= 0; i--) {
		if ((lsm303c_mag_odr_table[i].cutoff_us <= poll_interval_ms)
								|| (i == 0))
			break;
	}

	config[1] = ((ODR_MAG_MASK & lsm303c_mag_odr_table[i].value) |
		((~ODR_MAG_MASK) & status_registers.cntrl1.resume_value));

	if (atomic_read(&stat->enabled_mag)) {
		config[0] = status_registers.cntrl1.address;
		err = lsm303c_i2c_write(stat, config, 1);
		if (err < 0)
			goto error;
		status_registers.cntrl1.resume_value = config[1];
		stat->ktime_mag = ktime_set(0, MS_TO_NS(poll_interval_ms));
	}

	return err;

error:
	dev_err(&stat->client->dev,
		"update magnetometer odr failed 0x%02x,0x%02x: %d\n",
						config[0], config[1], err);

	return err;
}

static int lsm303c_mag_update_operative_mode(struct lsm303c_status *stat,
							int axis, u8 value)
{
	int err = -1;
	u8 config[2];
	u8 mask;
	u8 addr;

	if (axis == 0) {
		config[0] = REG_CNTRL1_ADDR;
		mask = X_Y_PERFORMANCE_MASK;
		addr = REG_CNTRL1_ADDR;
	} else {
		config[0] = REG_CNTRL4_ADDR;
		mask = Z_PERFORMANCE_MASK;
		addr = REG_CNTRL4_ADDR;
	}
	err = lsm303c_i2c_read(stat, config, 1);
	if (err < 0)
		goto error;
	config[1] = ((mask & value) |
		((~mask) & config[0]));

	config[0] = addr;
	err = lsm303c_i2c_write(stat, config, 1);
	if (err < 0)
		goto error;
	if (axis == 0)
		stat->xy_mode = value;
	else
		stat->z_mode = value;

	return err;

error:
	dev_err(&stat->client->dev,
		"update operative mode failed 0x%02x,0x%02x: %d\n",
						config[0], config[1], err);

	return err;
}

static int lsm303c_validate_polling(unsigned int *min_interval,
					unsigned int *poll_interval,
					unsigned int min, u8 *axis_map_x,
					u8 *axis_map_y, u8 *axis_map_z,
					struct i2c_client *client)
{
	*min_interval = max(min, *min_interval);
	*poll_interval = max(*poll_interval, *min_interval);

	if (*axis_map_x > 2 || *axis_map_y > 2 || *axis_map_z > 2) {
		dev_err(&client->dev,
			"invalid axis_map value x:%u y:%u z%u\n",
					*axis_map_x, *axis_map_y, *axis_map_z);
		return -EINVAL;
	}

	return 0;
}

static int lsm303c_validate_negate(u8 *negate_x, u8 *negate_y, u8 *negate_z,
						struct i2c_client *client)
{
	if (*negate_x > 1 || *negate_y > 1 || *negate_z > 1) {
		dev_err(&client->dev,
			"invalid negate value x:%u y:%u z:%u\n",
					*negate_x, *negate_y, *negate_z);
		return -EINVAL;
	}
	return 0;
}

static int lsm303c_mag_validate_pdata(struct lsm303c_status *stat)
{
	int res = -1;

	res = lsm303c_validate_polling(&stat->pdata_mag->min_interval,
				&stat->pdata_mag->poll_interval,
				(unsigned int)LSM303C_MAG_MIN_POLL_PERIOD_MS,
				&stat->pdata_mag->axis_map_x,
				&stat->pdata_mag->axis_map_y,
				&stat->pdata_mag->axis_map_z,
				stat->client);
	if (res < 0)
		return -EINVAL;

	res = lsm303c_validate_negate(&stat->pdata_mag->negate_x,
				&stat->pdata_mag->negate_y,
				&stat->pdata_mag->negate_z,
				stat->client);
	if (res < 0)
		return -EINVAL;

	return 0;
}

static int lsm303c_mag_enable(struct lsm303c_status *stat)
{
	int err;

	if (!atomic_cmpxchg(&stat->enabled_mag, 0, 1)) {
		err = lsm303c_mag_device_power_on(stat);
		if (err < 0) {
			atomic_set(&stat->enabled_mag, 0);
			return err;
		}
		hrtimer_start(&stat->hr_timer_mag,
					stat->ktime_mag, HRTIMER_MODE_REL);
	}

	return 0;
}

static int lsm303c_mag_disable(struct lsm303c_status *stat)
{
	if (atomic_cmpxchg(&stat->enabled_mag, 1, 0)) {
		cancel_work_sync(&stat->input_work_mag);
		hrtimer_cancel(&stat->hr_timer_mag);
		lsm303c_mag_device_power_off(stat);
	}

	return 0;
}

static void lsm303c_mag_input_cleanup(struct lsm303c_status *stat)
{
	input_unregister_device(stat->input_dev_mag);
	input_free_device(stat->input_dev_mag);
}

static ssize_t attr_get_polling_rate_mag(struct device *dev,
						struct device_attribute *attr,
		char *buf)
{
	unsigned int val;
	struct lsm303c_status *stat = dev_get_drvdata(dev);

	mutex_lock(&stat->lock);
	val = stat->pdata_mag->poll_interval;
	mutex_unlock(&stat->lock);
	return sprintf(buf, "%u\n", val);
}

static ssize_t attr_set_polling_rate_mag(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct lsm303c_status *stat = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (kstrtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	interval_ms = max_t(unsigned int, (unsigned int)interval_ms,
						stat->pdata_mag->min_interval);
	mutex_lock(&stat->lock);
	stat->pdata_mag->poll_interval = (unsigned int)interval_ms;
	lsm303c_mag_update_odr(stat, interval_ms);
	mutex_unlock(&stat->lock);
	return size;
}

static ssize_t attr_get_enable_mag(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct lsm303c_status *stat = dev_get_drvdata(dev);
	int val = (int)atomic_read(&stat->enabled_mag);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable_mag(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct lsm303c_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lsm303c_mag_enable(stat);
	else
		lsm303c_mag_disable(stat);

	return size;
}

static ssize_t attr_get_range_mag(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u8 val;
	int range = 2;
	struct lsm303c_status *stat = dev_get_drvdata(dev);

	mutex_lock(&stat->lock);
	val = stat->pdata_mag->fs_range;
	switch (val) {
	case LSM303C_MAG_FS_4G:
		range = 4;
		break;
	case LSM303C_MAG_FS_8G:
		range = 8;
		break;
	case LSM303C_MAG_FS_10G:
		range = 10;
		break;
	}
	mutex_unlock(&stat->lock);

	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range_mag(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct lsm303c_status *stat = dev_get_drvdata(dev);
	unsigned long val;
	u8 range;
	int err;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;
	switch (val) {
	case 4:
		range = LSM303C_MAG_FS_4G;
		break;
	case 8:
		range = LSM303C_MAG_FS_8G;
		break;
	case 10:
		range = LSM303C_MAG_FS_10G;
		break;
	default:
		dev_err(&stat->client->dev,
			"magnetometer invalid range request: %lu, discarded\n",
									val);
		return -EINVAL;
	}
	mutex_lock(&stat->lock);
	err = lsm303c_mag_update_fs_range(stat, range);
	if (err < 0) {
		mutex_unlock(&stat->lock);
		return err;
	}
	stat->pdata_mag->fs_range = range;
	mutex_unlock(&stat->lock);
	dev_info(&stat->client->dev,
				"magnetometer range set to: %lu g\n", val);

	return size;
}

static ssize_t attr_get_xy_mode(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u8 val;
	char mode[13];
	struct lsm303c_status *stat = dev_get_drvdata(dev);

	mutex_lock(&stat->lock);
	val = stat->xy_mode;
	switch (val) {
	case X_Y_HIGH_PERFORMANCE:
		strcpy(&(mode[0]), "high");
		break;
	case X_Y_LOW_PERFORMANCE:
		strcpy(&(mode[0]), "low");
		break;
	case X_Y_MEDIUM_PERFORMANCE:
		strcpy(&(mode[0]), "medium");
		break;
	case X_Y_ULTRA_HIGH_PERFORMANCE:
		strcpy(&(mode[0]), "ultra_high");
		break;
	}
	mutex_unlock(&stat->lock);
	return sprintf(buf, "%s\n", mode);
}

static ssize_t attr_set_xy_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct lsm303c_status *stat = dev_get_drvdata(dev);
	u8 mode;
	int err;

	err = strncmp(buf, "high", 4);
	if (err == 0) {
		mode = X_Y_HIGH_PERFORMANCE;
		goto valid;
	}
	err = strncmp(buf, "low", 3);
	if (err == 0) {
		mode = X_Y_LOW_PERFORMANCE;
		goto valid;
	}
	err = strncmp(buf, "medium", 6);
	if (err == 0) {
		mode = X_Y_MEDIUM_PERFORMANCE;
		goto valid;
	}
	err = strncmp(buf, "ultra_high", 10);
	if (err == 0) {
		mode = X_Y_ULTRA_HIGH_PERFORMANCE;
		goto valid;
	}
	goto error;

valid:
	err = lsm303c_mag_update_operative_mode(stat, 0, mode);
	if (err < 0)
		goto error;

	dev_info(&stat->client->dev,
				"magnetometer x_y op. mode set to: %s", buf);
	return size;

error:
	dev_err(&stat->client->dev,
		"magnetometer invalid value request: %s, discarded\n", buf);

	return -EINVAL;
}

static ssize_t attr_get_z_mode(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u8 val;
	char mode[13];
	struct lsm303c_status *stat = dev_get_drvdata(dev);

	mutex_lock(&stat->lock);
	val = stat->z_mode;
	switch (val) {
	case Z_HIGH_PERFORMANCE:
		strcpy(&(mode[0]), "high");
		break;
	case Z_LOW_PERFORMANCE:
		strcpy(&(mode[0]), "low");
		break;
	case Z_MEDIUM_PERFORMANCE:
		strcpy(&(mode[0]), "medium");
		break;
	case Z_ULTRA_HIGH_PERFORMANCE:
		strcpy(&(mode[0]), "ultra_high");
		break;
	}
	mutex_unlock(&stat->lock);
	return sprintf(buf, "%s\n", mode);
}

static ssize_t attr_set_z_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct lsm303c_status *stat = dev_get_drvdata(dev);
	u8 mode;
	int err;

	err = strncmp(buf, "high", 4);
	if (err == 0) {
		mode = Z_HIGH_PERFORMANCE;
		goto valid;
	}
	err = strncmp(buf, "low", 3);
	if (err == 0) {
		mode = Z_LOW_PERFORMANCE;
		goto valid;
	}
	err = strncmp(buf, "medium", 6);
	if (err == 0) {
		mode = Z_MEDIUM_PERFORMANCE;
		goto valid;
	}
	err = strncmp(buf, "ultra_high", 10);
	if (err == 0) {
		mode = Z_ULTRA_HIGH_PERFORMANCE;
		goto valid;
	}
	goto error;

valid:
	err = lsm303c_mag_update_operative_mode(stat, 1, mode);
	if (err < 0)
		goto error;

	dev_info(&stat->client->dev,
			"magnetometer z op. mode set to: %s", buf);
	return size;

error:
	dev_err(&stat->client->dev,
		"magnetometer invalid value request: %s, discarded\n", buf);

	return -EINVAL;
}

#if 0 //def DEBUG
static int write_bit_on_register(struct lsm303c_status *stat, u8 address,
					u8 *resume_value, u8 mask, int value)
{
	int err;
	u8 updated_val;
	u8 buf[2];
	u8 val = 0x00;

	buf[0] = address;
	err = lsm303c_i2c_read(stat, buf, 1);
	if (err < 0)
		return -1;

	if (resume_value != NULL)
		*resume_value = buf[0];

	if (mask == 0)
		updated_val = (u8)value;
	else {
		if (value > 0)
			val = 0xFF;

		updated_val = (mask & val) | ((~mask) & buf[0]);
	}

	buf[1] = updated_val;
	buf[0] = address;

	err = lsm303c_i2c_write(stat, buf, 1);
	if (err < 0)
		return -1;

	if (resume_value != NULL)
		*resume_value = updated_val;

	return err;
}
#endif

#ifdef DEBUG
/* PAY ATTENTION: These DEBUG functions don't manage resume_state */
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int rc;
	struct lsm303c_status *stat = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&stat->lock);
	x[0] = stat->reg_addr;
	mutex_unlock(&stat->lock);
	x[1] = val;
	rc = lsm303c_i2c_write(stat, x, 1);
	/*TODO: error need to be managed */
	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t ret;
	struct lsm303c_status *stat = dev_get_drvdata(dev);
	int rc;
	u8 data;

	mutex_lock(&stat->lock);
	data = stat->reg_addr;
	mutex_unlock(&stat->lock);
	rc = lsm303c_i2c_read(stat, &data, 1);
	/*TODO: error need to be managed */
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct lsm303c_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&stat->lock);
	stat->reg_addr = val;
	mutex_unlock(&stat->lock);
	return size;
}
#endif


#if 0
static struct device_attribute attributes[] = {
	__ATTR(pollrate_ms, 0666, attr_get_polling_rate_mag,
						attr_set_polling_rate_mag),
	__ATTR(full_scale, 0666, attr_get_range_mag, attr_set_range_mag),
	__ATTR(enable_device, 0666, attr_get_enable_mag, attr_set_enable_mag),
	__ATTR(x_y_opearative_mode, 0666, attr_get_xy_mode, attr_set_xy_mode),
	__ATTR(z_opearative_mode, 0666, attr_get_z_mode, attr_set_z_mode),
#ifdef DEBUG
	__ATTR(reg_value, 0600, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0200, NULL, attr_addr_set),
#endif
};
#else	

static DEVICE_ATTR(pollrate_ms, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH, attr_get_polling_rate_mag, attr_set_polling_rate_mag);

static DEVICE_ATTR(full_scale, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH, attr_get_range_mag, attr_set_range_mag);

static DEVICE_ATTR(enable_device, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH, attr_get_enable_mag, attr_set_enable_mag);

static DEVICE_ATTR(x_y_opearative_mode, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH, attr_get_xy_mode, attr_set_xy_mode);

static DEVICE_ATTR(z_opearative_mode, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH, attr_get_z_mode, attr_set_z_mode);

static DEVICE_ATTR(reg_value, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH, attr_reg_get, attr_reg_set);

static DEVICE_ATTR(reg_addr, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH, NULL, attr_addr_set);



static struct attribute *attributes[] = {
	&dev_attr_pollrate_ms.attr,
	&dev_attr_full_scale.attr,
	&dev_attr_enable_device.attr,
	&dev_attr_x_y_opearative_mode.attr,
	&dev_attr_z_opearative_mode.attr,
	&dev_attr_reg_value.attr,
	&dev_attr_reg_addr.attr,
	
#if 0 	
	&dev_attr_int1_threshold.attr,
	&dev_attr_click_config.attr,
	&dev_attr_click_source.attr,
	&dev_attr_click_threshold.attr,
	&dev_attr_click_timelimit.attr,
	&dev_attr_click_timelatency.attr,
	&dev_attr_click_timewindow.attr,
#endif	
	NULL
};

static struct attribute_group k303c_attribute_group = {
	.attrs = attributes
};


#endif

#if 0
static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;

error:
	for (; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static void remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
}
#endif

int lsm303c_mag_input_open(struct input_dev *input)
{
	struct lsm303c_status *stat = input_get_drvdata(input);

	return lsm303c_mag_enable(stat);
}

void lsm303c_mag_input_close(struct input_dev *dev)
{
	struct lsm303c_status *stat = input_get_drvdata(dev);

	lsm303c_mag_disable(stat);
}

static int lsm303c_mag_get_data(struct lsm303c_status *stat, int *xyz)
{
	int err = -1;
	u8 mag_data[6];
	s32 hw_d[3] = { 0 };

	mag_data[0] = (REG_MAG_DATA_ADDR);
	err = lsm303c_i2c_read(stat, mag_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = ((s32)((s16)((mag_data[1] << 8) | (mag_data[0]))));
	hw_d[1] = ((s32)((s16)((mag_data[3] << 8) | (mag_data[2]))));
	hw_d[2] = ((s32)((s16)((mag_data[5] << 8) | (mag_data[4]))));

#ifdef DEBUG
	pr_debug("%s read x=%X %X(regH regL), x=%d(dec) [ug]\n",
		LSM303C_MAG_DEV_NAME, mag_data[1], mag_data[0], hw_d[0]);
	pr_debug("%s read x=%X %X(regH regL), x=%d(dec) [ug]\n",
		LSM303C_MAG_DEV_NAME, mag_data[3], mag_data[2], hw_d[1]);
	pr_debug("%s read x=%X %X(regH regL), x=%d(dec) [ug]\n",
		LSM303C_MAG_DEV_NAME, mag_data[5], mag_data[4], hw_d[2]);
#endif
/*
	hw_d[0] = hw_d[0] * stat->sensitivity_mag;
	hw_d[1] = hw_d[1] * stat->sensitivity_mag;
	hw_d[2] = hw_d[2] * stat->sensitivity_mag;
*/
	xyz[0] = ((stat->pdata_mag->negate_x) ?
				(-hw_d[stat->pdata_mag->axis_map_x])
					: (hw_d[stat->pdata_mag->axis_map_x]));
	xyz[1] = ((stat->pdata_mag->negate_y) ?
				(-hw_d[stat->pdata_mag->axis_map_y])
					: (hw_d[stat->pdata_mag->axis_map_y]));
	xyz[2] = ((stat->pdata_mag->negate_z) ?
				(-hw_d[stat->pdata_mag->axis_map_z])
					: (hw_d[stat->pdata_mag->axis_map_z]));

	return err;
}

static void lsm303c_mag_report_values(struct lsm303c_status *stat, int *xyz)
{
	input_report_abs(stat->input_dev_mag, ABS_X, xyz[0]);
	input_report_abs(stat->input_dev_mag, ABS_Y, xyz[1]);
	input_report_abs(stat->input_dev_mag, ABS_Z, xyz[2]);
	input_sync(stat->input_dev_mag);
}

static int lsm303c_mag_input_init(struct lsm303c_status *stat)
{
	int err;

	stat->input_dev_mag = input_allocate_device();
	if (!stat->input_dev_mag) {
		err = -ENOMEM;
		dev_err(&stat->client->dev,
			"magnetometer input device allocation failed\n");
		goto err0;
	}

	stat->input_dev_mag->open = lsm303c_mag_input_open;
	stat->input_dev_mag->close = lsm303c_mag_input_close;
	stat->input_dev_mag->name = "magnetic_field";
	stat->input_dev_mag->uniq = LSM303C_MAG_DEV_NAME;
	stat->input_dev_mag->id.bustype = BUS_I2C;
	stat->input_dev_mag->dev.parent = &stat->client->dev;

	input_set_drvdata(stat->input_dev_mag, stat);

	set_bit(EV_ABS, stat->input_dev_mag->evbit);

	input_set_abs_params(stat->input_dev_mag, ABS_X,
				-MAG_G_MAX_NEG, MAG_G_MAX_POS, FUZZ, FLAT);
	input_set_abs_params(stat->input_dev_mag, ABS_Y,
				-MAG_G_MAX_NEG, MAG_G_MAX_POS, FUZZ, FLAT);
	input_set_abs_params(stat->input_dev_mag, ABS_Z,
				-MAG_G_MAX_NEG, MAG_G_MAX_POS, FUZZ, FLAT);

	err = input_register_device(stat->input_dev_mag);
	if (err) {
		dev_err(&stat->client->dev,
			"unable to register magnetometer input device %s\n",
				stat->input_dev_mag->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(stat->input_dev_mag);
err0:
	return err;
}

static void lsm303c_input_cleanup(struct lsm303c_status *stat)
{
	input_unregister_device(stat->input_dev_mag);
	input_free_device(stat->input_dev_mag);
}

static void poll_function_work_mag(struct work_struct *input_work_mag)
{
	struct lsm303c_status *stat;
	int xyz[3] = { 0 };
	int err;

	stat = container_of((struct work_struct *)input_work_mag,
			struct lsm303c_status, input_work_mag);

	mutex_lock(&stat->lock);

	if (atomic_read(&stat->enabled_mag)) {
		err = lsm303c_mag_get_data(stat, xyz);
		if (err < 0)
			dev_err(&stat->client->dev,
					"get_magnetometer_data failed\n");
		else
			lsm303c_mag_report_values(stat, xyz);
	}

	mutex_unlock(&stat->lock);
	hrtimer_start(&stat->hr_timer_mag, stat->ktime_mag, HRTIMER_MODE_REL);
}

enum hrtimer_restart poll_function_read_mag(struct hrtimer *timer)
{
	struct lsm303c_status *stat;


	stat = container_of((struct hrtimer *)timer,
				struct lsm303c_status, hr_timer_mag);

	queue_work(lsm303c_workqueue, &stat->input_work_mag);
	return HRTIMER_NORESTART;
}

#ifdef CONFIG_OF
static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0) ?
		regulator_set_optimum_mode(reg, load_uA) : 0;
}

static int sensor_regulator_configure(struct lsm303c_status *data, bool on)
{
	struct i2c_client *client = data->client;
	struct lsm303c_mag_platform_data *pdata = data->pdata_mag;
	int rc;

	if (on == false)
		goto hw_shutdown;

	pdata->vcc_ana = devm_regulator_get(&client->dev, "vdda");
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
		pdata->vcc_dig = devm_regulator_get(&client->dev, "vddio_dig");
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
		pdata->vcc_i2c = devm_regulator_get(&client->dev, "vddio");
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
	devm_regulator_put(pdata->vcc_i2c);
error_get_vtg_i2c:
	if (pdata->digital_pwr_regulator)
		if (regulator_count_voltages(pdata->vcc_dig) > 0)
			regulator_set_voltage(pdata->vcc_dig, 0,
				pdata->vddio_dig_supply_max);
error_set_vtg_vcc_dig:
	if (pdata->digital_pwr_regulator)
		devm_regulator_put(pdata->vcc_dig);
error_get_vtg_vcc_dig:
	if (regulator_count_voltages(pdata->vcc_ana) > 0)
		regulator_set_voltage(pdata->vcc_ana, 0, pdata->vdd_ana_supply_max);
error_set_vtg_vcc_ana:
	devm_regulator_put(pdata->vcc_ana);
	return rc;

hw_shutdown:
	if (regulator_count_voltages(pdata->vcc_ana) > 0)
		regulator_set_voltage(pdata->vcc_ana, 0, pdata->vdd_ana_supply_max);
	devm_regulator_put(pdata->vcc_ana);
	if (pdata->digital_pwr_regulator) {
		if (regulator_count_voltages(pdata->vcc_dig) > 0)
			regulator_set_voltage(pdata->vcc_dig, 0,
						pdata->vddio_dig_supply_max);
		devm_regulator_put(pdata->vcc_dig);
	}
	if (pdata->i2c_pull_up) {
		if (regulator_count_voltages(pdata->vcc_i2c) > 0)
			regulator_set_voltage(pdata->vcc_i2c, 0,
						pdata->vddio_i2c_supply_max);

		devm_regulator_put(pdata->vcc_i2c);
	}
	return 0;
}

static int sensor_regulator_power_on(struct lsm303c_status *data, bool on)
{
	struct i2c_client *client = data->client;
	struct lsm303c_mag_platform_data *pdata = data->pdata_mag;
	int rc;

	if (on == 0)
		goto power_off;
	dev_info(&client->dev,"Regulator Power On \n");
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

	msleep(130);

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
	return 0;
}

static int sensor_platform_hw_power_on(bool on)
{
	sensor_regulator_power_on(g_ptr_mag_stat, on);
	return 0;
}

static int sensor_platform_hw_power_off(void)
{
	sensor_regulator_power_on(g_ptr_mag_stat, 0);
	return 0;
}

static int sensor_platform_hw_init(void)
{
	struct lsm303c_status *data = g_ptr_mag_stat;
	int error;

	error = sensor_regulator_configure(data, true);
	return 0;
}

static void sensor_platform_hw_exit(void)
{
	struct lsm303c_status *data = g_ptr_mag_stat;

	sensor_regulator_configure(data, false);
}

static int sensor_parse_dt(struct device *dev, struct lsm303c_mag_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	int ret, err =0;

	struct sensor_dt_to_pdata_map *itr;
	struct sensor_dt_to_pdata_map map[] = {
	{"STMicroelectronics,i2c-pull-up",		&pdata->i2c_pull_up,			DT_REQUIRED,	DT_BOOL,	0},
	{"STMicroelectronics,dig-reg-support",		&pdata->digital_pwr_regulator,		DT_REQUIRED,	DT_BOOL,	0},
	{"STMicroelectronics,irq-gpio",			&pdata->irq_gpio,			DT_REQUIRED,	DT_GPIO,	0},
	{"STMicroelectronics,vdd_ana_supply_min",	&pdata->vdd_ana_supply_min, 		DT_SUGGESTED,	DT_U32, 	0},
	{"STMicroelectronics,vdd_ana_supply_max",	&pdata->vdd_ana_supply_max, 		DT_SUGGESTED,	DT_U32, 	0},
	{"STMicroelectronics,vdd_ana_load_ua",		&pdata->vdd_ana_load_ua,		DT_SUGGESTED,	DT_U32, 	0},
	{"STMicroelectronics,vddio_dig_supply_min",	&pdata->vddio_dig_supply_min,		DT_SUGGESTED,	DT_U32, 	0},
	{"STMicroelectronics,vddio_dig_supply_max",	&pdata->vddio_dig_supply_max,		DT_SUGGESTED,	DT_U32, 	0},
	{"STMicroelectronics,vddio_dig_load_ua", 	&pdata->vddio_dig_load_ua,		DT_SUGGESTED,	DT_U32, 	0},
	{"STMicroelectronics,vddio_i2c_supply_min",	&pdata->vddio_i2c_supply_min,		DT_SUGGESTED,	DT_U32, 	0},
	{"STMicroelectronics,vddio_i2c_supply_max",	&pdata->vddio_i2c_supply_max,		DT_SUGGESTED,	DT_U32, 	0},
	{"STMicroelectronics,vddio_i2c_load_ua", 	&pdata->vddio_i2c_load_ua,		DT_SUGGESTED,	DT_U32, 	0},
	{NULL					, 	NULL				  , 	0		, 0 	 ,	0}, 	
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
			printk(KERN_INFO "%d is an unknown DT entry type\n",
								itr->type);
			ret = -EBADE;
		}
		switch (itr->type) {
			case DT_GPIO:
			case DT_U32:	
				printk(KERN_INFO "DT entry ret:%d name:%s val:%d\n",
						ret, itr->dt_name, *((int *)itr->ptr_data));
				break;
			case DT_BOOL:
				printk(KERN_INFO "DT entry ret:%d name:%s val:%d\n",
										ret, itr->dt_name, *((bool *)itr->ptr_data));
				break;
		}

		if (ret) {
			*((int *)itr->ptr_data) = itr->default_val;

			if (itr->status < DT_OPTIONAL) {
				printk(KERN_INFO "Missing '%s' DT entry\n",
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
	pdata->power_off = sensor_platform_hw_power_off;
	return err;

}
#endif

static int lsm303c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct lsm303c_status *stat;

	u32 smbus_func = I2C_FUNC_SMBUS_BYTE_DATA |
			I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_I2C_BLOCK;

	int err = -1;
	dev_info(&client->dev, "probe start.\n");
	g_ptr_mag_stat = stat = kzalloc(sizeof(struct lsm303c_status), GFP_KERNEL);
	if (stat == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for module data: "
					"%d\n", err);
		goto exit_check_functionality_failed;
	}

	stat->use_smbus = 0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_warn(&client->dev, "client not i2c capable\n");
		if (i2c_check_functionality(client->adapter, smbus_func)) {
			stat->use_smbus = 1;
			dev_warn(&client->dev, "client using SMBUS\n");
		} else {
			err = -ENODEV;
			dev_err(&client->dev, "client nor SMBUS capable\n");
			goto exit_check_functionality_failed;
		}
	}

	if (lsm303c_workqueue == 0)
		lsm303c_workqueue = create_workqueue("lsm303c_workqueue");

	hrtimer_init(&stat->hr_timer_mag, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	stat->hr_timer_mag.function = &poll_function_read_mag;

	mutex_init(&stat->lock);
	mutex_lock(&stat->lock);

	stat->client = client;
	i2c_set_clientdata(client, stat);

#ifdef CONFIG_OF
	if(client->dev.of_node){
		stat->pdata_mag = devm_kzalloc(&client->dev,sizeof(*stat->pdata_mag), GFP_KERNEL);
		if (NULL == stat->pdata_mag) {
			err = -ENOMEM;
			dev_err(&client->dev,
				"failed to allocate memory for pdata: %d\n", err);
			goto err_mutexunlock;
		}
		memcpy(stat->pdata_mag, &default_lsm303c_mag_pdata,sizeof(*stat->pdata_mag));
		client->dev.platform_data = stat->pdata_mag;
		err = sensor_parse_dt(&client->dev, stat->pdata_mag);
		if(err){
			dev_err(&client->dev,"failed to parse Device tree: %d\n",err);
			goto err_mutexunlock;
		}		
	}
	else {
		dev_err(&client->dev,"No platform Data found\n");
		goto err_mutexunlock;
	}
#endif
	err = lsm303c_mag_validate_pdata(stat);
	if (err < 0) {
		dev_err(&client->dev,
			"failed to validate platform data for magnetometer\n");
		goto exit_kfree_pdata;
	}

	if (stat->pdata_mag->init) {
		err = stat->pdata_mag->init();
		if (err < 0) {
			dev_err(&client->dev,
				"magnetometer init failed: %d\n", err);
			goto err_pdata_mag_init;
		}
	}

	err = lsm303c_mag_device_power_on(stat);
	if (err < 0) {
		dev_err(&client->dev,
			"magnetometer power on failed: %d\n", err);
		goto err_pdata_init;
	}

	err = lsm303c_mag_update_fs_range(stat, stat->pdata_mag->fs_range);
	if (err < 0) {
		dev_err(&client->dev,
			"update_fs_range on magnetometer failed\n");
		goto  err_power_off_mag;
	}

	err = lsm303c_mag_update_odr(stat, stat->pdata_mag->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr on magnetometer failed\n");
		goto  err_power_off;
	}

	err = lsm303c_mag_input_init(stat);
	if (err < 0) {
		dev_err(&client->dev, "magnetometer input init failed\n");
		goto err_power_off;
	}

#if 0
		err = create_sysfs_interfaces(&client->dev);
#else
		err = sysfs_create_group(&stat->input_dev_mag->dev.kobj,
			&k303c_attribute_group);
#endif

	if (err < 0) {
		dev_err(&client->dev,
			"device LSM303C_MAG_DEV_NAME sysfs register failed\n");
		goto err_input_cleanup;
	}

	lsm303c_mag_device_power_off(stat);

	INIT_WORK(&stat->input_work_mag, poll_function_work_mag);

	mutex_unlock(&stat->lock);
	dev_info(&client->dev, "%s: probed\n", LSM303C_MAG_DEV_NAME);
	return 0;

err_input_cleanup:
#if 0
		remove_sysfs_interfaces(&client->dev);
#else
	sysfs_remove_group(&stat->input_dev_mag->dev.kobj,
			&k303c_attribute_group);
#endif

	lsm303c_input_cleanup(stat);
err_power_off:
err_power_off_mag:
	lsm303c_mag_device_power_off(stat);
err_pdata_init:
err_pdata_mag_init:
	if (stat->pdata_mag->exit)
		stat->pdata_mag->exit();
exit_kfree_pdata:
err_mutexunlock:
	mutex_unlock(&stat->lock);
	kfree(stat);
	if (!lsm303c_workqueue) {
		flush_workqueue(lsm303c_workqueue);
		destroy_workqueue(lsm303c_workqueue);
	}
exit_check_functionality_failed:
	pr_err("%s: Driver Init failed\n", LSM303C_MAG_DEV_NAME);
	return err;
}

static int lsm303c_remove(struct i2c_client *client)
{
	struct lsm303c_status *stat = i2c_get_clientdata(client);

	lsm303c_mag_disable(stat);
	lsm303c_mag_input_cleanup(stat);

#if 0
		remove_sysfs_interfaces(&client->dev);
#else
	sysfs_remove_group(&stat->input_dev_mag->dev.kobj,
			&k303c_attribute_group);
#endif


	if (stat->pdata_mag->exit)
		stat->pdata_mag->exit();

	if (!lsm303c_workqueue) {
		flush_workqueue(lsm303c_workqueue);
		destroy_workqueue(lsm303c_workqueue);
	}

	kfree(stat->pdata_mag);
	kfree(stat);
	return 0;
}

static const struct i2c_device_id lsm303c_id[] = {
		{ LSM303C_MAG_DEV_NAME, 0 },
		{ },
};

MODULE_DEVICE_TABLE(i2c, lsm303c_id);

#ifdef CONFIG_OF
static struct of_device_id K303C_mag_match_table[] = {
    { .compatible = "STMicroelectronics,magnetometer",},
    { },
};
#else
#define K303C_mag_match_table NULL
#endif

static struct i2c_driver lsm303c_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = LSM303C_MAG_DEV_NAME,
			.of_match_table = K303C_mag_match_table,
	},
	.probe = lsm303c_probe,
	.remove = __devexit_p(lsm303c_remove),
	.id_table = lsm303c_id,
};

static int __init lsm303c_init(void)
{
	pr_info("%s driver: init\n", LSM303C_MAG_DEV_NAME);
	return i2c_add_driver(&lsm303c_driver);
}

static void __exit lsm303c_exit(void)
{
	i2c_del_driver(&lsm303c_driver);
}

module_init(lsm303c_init);
module_exit(lsm303c_exit);

MODULE_DESCRIPTION("lsm303c magnetometer driver");
MODULE_AUTHOR("Matteo Dameno, Denis Ciocca, STMicroelectronics");
MODULE_LICENSE("GPL");
