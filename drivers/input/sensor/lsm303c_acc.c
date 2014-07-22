/******************** (C) COPYRIGHT 2013 STMicroelectronics ********************
 *
 * File Name          : lsm303c_acc.c
 * Authors            : AMS - Motion Mems Division - Application Team
 *		      : Matteo Dameno (matteo.dameno@st.com)
 *		      : Denis Ciocca (denis.ciocca@st.com)
 *		      : Both authors are willing to be considered the contact
 *		      : and update points for the driver.
 * Version            : V.1.0.0
 * Date               : 2013/Jun/17
 * Description        : LSM303C accelerometer driver
 *
 *******************************************************************************
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
 ******************************************************************************/

#include	<linux/err.h>
#include	<linux/errno.h>
#include	<linux/delay.h>
#include	<linux/fs.h>
#include	<linux/i2c.h>
#include	<linux/input.h>
#include	<linux/uaccess.h>
#include	<linux/workqueue.h>
#include	<linux/irq.h>
#include	<linux/gpio.h>
#include	<linux/interrupt.h>
#include	<linux/slab.h>
#include	<linux/kernel.h>
#include	<linux/device.h>
#include	<linux/module.h>
#include	<linux/moduleparam.h>

#ifdef CONFIG_OF
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#endif

/*#include	<linux/input/lsm303c.h>*/
#include	"lsm303c.h"

#define G_MAX			(7995148) /* (SENSITIVITY_8G * ((2^15)-1)) */
#define G_MIN			(-7995392) /* (-SENSITIVITY_8G * (2^15)) */
#define FUZZ			0
#define FLAT			0
#define I2C_RETRY_DELAY		5
#define I2C_RETRIES		5
#define I2C_AUTO_INCREMENT	(0x00)

#define MS_TO_NS(x)		(x * 1000000L)

#define SENSITIVITY_2G		 61	/**	ug/LSB	*/
#define SENSITIVITY_4G		122	/**	ug/LSB	*/
#define SENSITIVITY_8G		244	/**	ug/LSB	*/


/* Accelerometer Sensor Operating Mode */
#define LSM303C_ACC_ENABLE	(0x01)
#define LSM303C_ACC_DISABLE	(0x00)

#define AXISDATA_REG		(0x28)
#define WHOAMI_LSM303C_ACC	(0x41)	/*	Expctd content for WAI	*/
#define ALL_ZEROES		(0x00)
#define LSM303C_ACC_PM_OFF	(0x00)
#define ACC_ENABLE_ALL_AXES	(0x07)

/*	CONTROL REGISTERS	*/
#define TEMP_L			(0x0B)
#define TEMP_H			(0x0C)
#define WHO_AM_I		(0x0F)	/*	WhoAmI register		*/
#define ACT_THS			(0x1E)	/*	Activity Threshold	*/
#define ACT_DUR			(0x1F)	/*	Activity Duration	*/
/* ctrl 1: HR ODR2 ODR1 ODR0 BDU Zenable Yenable Xenable */
#define CTRL1			(0x20)	/*	control reg 1		*/
#define CTRL2			(0x21)	/*	control reg 2		*/
#define CTRL3			(0x22)	/*	control reg 3		*/
#define CTRL4			(0x23)	/*	control reg 4		*/
#define CTRL5			(0x24)	/*	control reg 5		*/
#define CTRL6			(0x25)	/*	control reg 6		*/
#define CTRL7			(0x26)	/*	control reg 7		*/

#define FIFO_CTRL		(0x2E)	/*	fifo control reg	*/

#define INT_CFG1		(0x30)	/*	interrupt 1 config	*/
#define INT_SRC1		(0x31)	/*	interrupt 1 source	*/
#define INT_THSX1		(0x32)	/*	interrupt 1 threshold x	*/
#define INT_THSY1		(0x33)	/*	interrupt 1 threshold y	*/
#define INT_THSZ1		(0x34)	/*	interrupt 1 threshold z	*/
#define INT_DUR1		(0x35)	/*	interrupt 1 duration	*/

#define INT_CFG2		(0x36)	/*	interrupt 2 config	*/
#define INT_SRC2		(0x37)	/*	interrupt 2 source	*/
#define INT_THS2		(0x38)	/*	interrupt 2 threshold	*/
#define INT_DUR2		(0x39)	/*	interrupt 2 duration	*/

#define REF_XL			(0x3A)	/*	reference_l_x		*/
#define REF_XH			(0x3B)	/*	reference_h_x		*/
#define REF_YL			(0x3C)	/*	reference_l_y		*/
#define REF_YH			(0x3D)	/*	reference_h_y		*/
#define REF_ZL			(0x3E)	/*	reference_l_z		*/
#define REF_ZH			(0x3F)	/*	reference_h_z		*/
/*	end CONTROL REGISTRES	*/



#define ACC_ODR10		(0x10)	/*   10Hz output data rate */
#define ACC_ODR50		(0x20)	/*   50Hz output data rate */
#define ACC_ODR100		(0x30)	/*  100Hz output data rate */
#define ACC_ODR200		(0x40)	/*  200Hz output data rate */
#define ACC_ODR400		(0x50)	/*  400Hz output data rate */
#define ACC_ODR800		(0x60)	/*  800Hz output data rate */
#define ACC_ODR_MASK		(0X70)

/* Registers configuration Mask and settings */
/* CTRL1 */
#define CTRL1_HR_DISABLE	(0x00)
#define CTRL1_HR_ENABLE		(0x80)
#define CTRL1_HR_MASK		(0x80)
#define CTRL1_BDU_ENABLE	(0x08)
#define CTRL1_BDU_MASK		(0x08)

/* CTRL2 */
#define CTRL2_IG1_INT1		(0x08)

/* CTRL3 */
#define CTRL3_IG1_INT1		(0x08)
#define CTRL3_DRDY_INT1

/* CTRL4 */
#define CTRL4_IF_ADD_INC_EN	(0x04)
#define CTRL4_BW_SCALE_ODR_AUT	(0x00)
#define CTRL4_BW_SCALE_ODR_SEL	(0x08)
#define CTRL4_ANTALIAS_BW_400	(0x00)
#define CTRL4_ANTALIAS_BW_200	(0x40)
#define CTRL4_ANTALIAS_BW_100	(0x80)
#define CTRL4_ANTALIAS_BW_50	(0xC0)
#define CTRL4_ANTALIAS_BW_MASK	(0xC0)

/* CTRL5 */
#define CTRL5_HLACTIVE_L	(0x02)
#define CTRL5_HLACTIVE_H	(0x00)

/* CTRL6 */
#define CTRL6_IG2_INT2		(0x10)
#define CTRL6_DRDY_INT2		(0x01)

/* CTRL7 */
#define CTRL7_LIR2		(0x08)
#define CTRL7_LIR1		(0x04)
/* */

#define NO_MASK			(0xFF)

#define INT1_DURATION_MASK	(0x7F)
#define INT1_THRESHOLD_MASK	(0x7F)



/* RESUME STATE INDICES */
#define RES_CTRL1		0
#define RES_CTRL2		1
#define RES_CTRL3		2
#define RES_CTRL4		3
#define RES_CTRL5		4
#define RES_CTRL6		5
#define RES_CTRL7		6

#define RES_INT_CFG1		7
#define RES_INT_THSX1		8
#define RES_INT_THSY1		9
#define RES_INT_THSZ1		10
#define RES_INT_DUR1		11


#define RES_INT_CFG2		12
#define RES_INT_THS2		13
#define RES_INT_DUR2		14

#define RES_TEMP_CFG_REG	15
#define RES_REFERENCE_REG	16
#define RES_FIFO_CTRL	17

#define RESUME_ENTRIES		18
/* end RESUME STATE INDICES */

#define OUTPUT_ALWAYS_ANTI_ALIASED 1

struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} lsm303c_acc_odr_table[] = {
		{    2, ACC_ODR800 },
		{    3, ACC_ODR400  },
		{    5, ACC_ODR200  },
		{   10, ACC_ODR100  },
#if (!OUTPUT_ALWAYS_ANTI_ALIASED)
		{   20, ACC_ODR50   },
		{  100, ACC_ODR10   },
#endif
};

struct lsm303c_acc_status {
	struct i2c_client *client;
	struct lsm303c_acc_platform_data *pdata;

	struct mutex lock;
	struct work_struct input_poll_work;
	struct hrtimer hr_timer_poll;
	ktime_t polling_ktime;
	struct workqueue_struct *hr_timer_poll_work_queue;

	struct input_dev *input_dev;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	atomic_t enabled;
	int on_before_suspend;
	int use_smbus;

	u8 sensitivity;

	u8 resume_state[RESUME_ENTRIES];

	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
#ifdef DEBUG
	u8 reg_addr;
#endif
};

static struct lsm303c_acc_status *g_ptr_acc_stat;

static struct lsm303c_acc_platform_data default_lsm303c_acc_pdata = {
	.fs_range = LSM303C_ACC_FS_2G,
	.axis_map_x = 1,
	.axis_map_y = 0,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,
	.poll_interval = 100,
	.min_interval = LSM303C_ACC_MIN_POLL_PERIOD_MS,
	.digital_pwr_regulator = 0,
	.irq_gpio = LSM303C_ACC_DEFAULT_INT1_GPIO,
};

/* sets default init values to be written in registers at probe stage */
static void lsm303c_acc_set_init_register_values(
						struct lsm303c_acc_status *stat)
{
	memset(stat->resume_state, 0, ARRAY_SIZE(stat->resume_state));

	stat->resume_state[RES_CTRL1] = (ALL_ZEROES |
					CTRL1_HR_DISABLE |
					CTRL1_BDU_ENABLE |
					ACC_ENABLE_ALL_AXES);

	if (stat->pdata->irq_gpio >= 0)
		stat->resume_state[RES_CTRL3] =
			(stat->resume_state[RES_CTRL3] | CTRL3_IG1_INT1);

	stat->resume_state[RES_CTRL4] = (ALL_ZEROES | CTRL4_IF_ADD_INC_EN);

	stat->resume_state[RES_CTRL5] = (ALL_ZEROES | CTRL5_HLACTIVE_H);

	stat->resume_state[RES_CTRL7] = (ALL_ZEROES | CTRL7_LIR2 | CTRL7_LIR1);

}

static int lsm303c_acc_i2c_read(struct lsm303c_acc_status *stat, u8 *buf,
									int len)
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
			return 0; /* failure */
		}
		return len; /* success */
	}

	ret = i2c_master_send(stat->client, &cmd, sizeof(cmd));
	if (ret != sizeof(cmd))
		return ret;

	return i2c_master_recv(stat->client, buf, len);
}

static int lsm303c_acc_i2c_write(struct lsm303c_acc_status *stat, u8 *buf,
									int len)
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

static int lsm303c_acc_hw_init(struct lsm303c_acc_status *stat)
{
	int err = -1;
	u8 buf[7];

	pr_info("%s: hw init start\n", LSM303C_ACC_DEV_NAME);

	buf[0] = WHO_AM_I;
	err = lsm303c_acc_i2c_read(stat, buf, 1);
	if (err < 0) {
		dev_warn(&stat->client->dev,
		"Error reading WHO_AM_I: is device available/working?\n");
		goto err_firstread;
	} else
		stat->hw_working = 1;

	if (buf[0] != WHOAMI_LSM303C_ACC) {
		dev_err(&stat->client->dev,
			"device unknown. Expected: 0x%02x,"
			" Replies: 0x%02x\n",
			WHOAMI_LSM303C_ACC, buf[0]);
		err = -1; /* choose the right coded error */
		goto err_unknown_device;
	}

	buf[0] = FIFO_CTRL;
	buf[1] = stat->resume_state[RES_FIFO_CTRL];
	err = lsm303c_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = INT_THSX1;
	buf[1] = stat->resume_state[RES_INT_THSX1];
	buf[2] = stat->resume_state[RES_INT_THSY1];
	buf[3] = stat->resume_state[RES_INT_THSZ1];
	buf[4] = stat->resume_state[RES_INT_DUR1];
	err = lsm303c_acc_i2c_write(stat, buf, 4);
	if (err < 0)
		goto err_resume_state;
	buf[0] = INT_CFG1;
	buf[1] = stat->resume_state[RES_INT_CFG1];
	err = lsm303c_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;


	buf[0] = CTRL2;
	buf[1] = stat->resume_state[RES_CTRL2];
	buf[2] = stat->resume_state[RES_CTRL3];
	buf[3] = stat->resume_state[RES_CTRL4];
	buf[4] = stat->resume_state[RES_CTRL5];
	buf[5] = stat->resume_state[RES_CTRL6];
	buf[6] = stat->resume_state[RES_CTRL7];
	err = lsm303c_acc_i2c_write(stat, buf, 6);
	if (err < 0)
		goto err_resume_state;

	buf[0] = CTRL1;
	buf[1] = stat->resume_state[RES_CTRL1];
	err = lsm303c_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto err_resume_state;

	stat->hw_initialized = 1;
	pr_info("%s: hw init done\n", LSM303C_ACC_DEV_NAME);
	return 0;

err_firstread:
	stat->hw_working = 0;
err_unknown_device:
err_resume_state:
	stat->hw_initialized = 0;
	dev_err(&stat->client->dev, "hw init error 0x%02x,0x%02x: %d\n", buf[0],
			buf[1], err);
	return err;
}

static void lsm303c_acc_device_power_off(struct lsm303c_acc_status *stat)
{
	int err;
	u8 buf[2] = { CTRL1, LSM303C_ACC_PM_OFF };

	err = lsm303c_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		dev_err(&stat->client->dev, "soft power off failed: %d\n", err);

	if (stat->pdata->power_off) {
		if (stat->pdata->irq_gpio >= 0)
			disable_irq_nosync(stat->irq1);

		stat->pdata->power_off();
		stat->hw_initialized = 0;
	}
	if (stat->hw_initialized) {
		if (stat->pdata->irq_gpio >= 0)
			disable_irq_nosync(stat->irq1);

		stat->hw_initialized = 0;
	}

}

static int lsm303c_acc_device_power_on(struct lsm303c_acc_status *stat)
{
	int err = -1;

	if (stat->pdata->power_on) {
		err = stat->pdata->power_on(1);
		if (err < 0) {
			dev_err(&stat->client->dev,
					"power_on failed: %d\n", err);
			return err;
		}
#if 0		
		if (stat->pdata->irq_gpio >= 0)
			enable_irq(stat->irq1);
#endif		
	}

	if (!stat->hw_initialized) {
		err = lsm303c_acc_hw_init(stat);
		if (stat->hw_working == 1 && err < 0) {
			lsm303c_acc_device_power_off(stat);
			return err;
		}
	}

	if (stat->hw_initialized) {
		if (stat->pdata->irq_gpio >= 0)
			enable_irq(stat->irq1);
	}
	return 0;
}


static int lsm303c_acc_update_fs_range(struct lsm303c_acc_status *stat,
							u8 new_fs_range)
{
	int err = -1;

	u8 sensitivity;
	u8 buf[2];
	u8 updated_val;
	u8 init_val;
	u8 new_val;
	u8 mask = LSM303C_ACC_FS_MASK;

	switch (new_fs_range) {
	case LSM303C_ACC_FS_2G:

		sensitivity = SENSITIVITY_2G;
		break;
	case LSM303C_ACC_FS_4G:

		sensitivity = SENSITIVITY_4G;
		break;
	case LSM303C_ACC_FS_8G:

		sensitivity = SENSITIVITY_8G;
		break;
	default:
		dev_err(&stat->client->dev, "invalid fs range requested: %u\n",
				new_fs_range);
		return -EINVAL;
	}


	/* Updates configuration register 4,
	* which contains fs range setting */
	buf[0] = CTRL4;
	err = lsm303c_acc_i2c_read(stat, buf, 1);
	if (err < 0)
		goto error;
	init_val = buf[0];
	stat->resume_state[RES_CTRL4] = init_val;
	new_val = new_fs_range;
	updated_val = ((mask & new_val) | ((~mask) & init_val));
	buf[1] = updated_val;
	buf[0] = CTRL4;
	err = lsm303c_acc_i2c_write(stat, buf, 1);
	if (err < 0)
		goto error;
	stat->resume_state[RES_CTRL4] = updated_val;
	stat->sensitivity = sensitivity;

	return err;
error:
	dev_err(&stat->client->dev,
			"update fs range failed 0x%02x,0x%02x: %d\n",
			buf[0], buf[1], err);

	return err;
}

static int lsm303c_acc_update_odr(struct lsm303c_acc_status *stat,
							int poll_interval_ms)
{
	int err;
	int i;
	u8 config[2];
	u8 updated_val;
	u8 init_val;
	u8 new_val;
	u8 mask = ACC_ODR_MASK;

	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (shortest interval) backward (longest
	 * interval), to support the poll_interval requested by the system.
	 * It must be the longest interval lower then the poll interval.*/
	for (i = ARRAY_SIZE(lsm303c_acc_odr_table) - 1; i >= 0; i--) {
		if ((lsm303c_acc_odr_table[i].cutoff_ms <= poll_interval_ms)
								|| (i == 0))
			break;
	}
	new_val = lsm303c_acc_odr_table[i].mask;

	/* Updates configuration register 1,
	* which contains odr range setting if enabled,
	* otherwise updates RES_CTRL1 for when it will */
	if (atomic_read(&stat->enabled)) {
		config[0] = CTRL1;
		err = lsm303c_acc_i2c_read(stat, config, 1);
		if (err < 0)
			goto error;
		init_val = config[0];
		stat->resume_state[RES_CTRL1] = init_val;
		updated_val = ((mask & new_val) | ((~mask) & init_val));
		config[1] = updated_val;
		config[0] = CTRL1;
		err = lsm303c_acc_i2c_write(stat, config, 1);
		if (err < 0)
			goto error;
		stat->resume_state[RES_CTRL1] = updated_val;
		return err;
	} else {
		init_val = stat->resume_state[RES_CTRL1];
		updated_val = ((mask & new_val) | ((~mask) & init_val));
		stat->resume_state[RES_CTRL1] = updated_val;
		return 0;
	}

error:
	dev_err(&stat->client->dev,
			"update odr failed 0x%02x,0x%02x: %d\n",
			config[0], config[1], err);

	return err;
}



static int lsm303c_acc_register_write(struct lsm303c_acc_status *stat,
					u8 *buf, u8 reg_address, u8 new_value)
{
	int err = -1;

		/* Sets configuration register at reg_address
		 *  NOTE: this is a straight overwrite  */
		buf[0] = reg_address;
		buf[1] = new_value;
		err = lsm303c_acc_i2c_write(stat, buf, 1);
		if (err < 0)
			return err;
	return err;
}


static int lsm303c_acc_get_data(
				struct lsm303c_acc_status *stat, int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	s32 hw_d[3] = { 0 };

	acc_data[0] = (AXISDATA_REG);
	err = lsm303c_acc_i2c_read(stat, acc_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = ((s16) ((acc_data[1] << 8) | acc_data[0]));
	hw_d[1] = ((s16) ((acc_data[3] << 8) | acc_data[2]));
	hw_d[2] = ((s16) ((acc_data[5] << 8) | acc_data[4]));

	hw_d[0] = hw_d[0] * stat->sensitivity;
	hw_d[1] = hw_d[1] * stat->sensitivity;
	hw_d[2] = hw_d[2] * stat->sensitivity;


	xyz[0] = ((stat->pdata->negate_x) ? (-hw_d[stat->pdata->axis_map_x])
		   : (hw_d[stat->pdata->axis_map_x]));
	xyz[1] = ((stat->pdata->negate_y) ? (-hw_d[stat->pdata->axis_map_y])
		   : (hw_d[stat->pdata->axis_map_y]));
	xyz[2] = ((stat->pdata->negate_z) ? (-hw_d[stat->pdata->axis_map_z])
		   : (hw_d[stat->pdata->axis_map_z]));

#ifdef DEBUG
	dev_dbg(&stat->client->dev, "%s read x=%d, y=%d, z=%d\n",
			LSM303C_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);
#endif
	return err;
}

static void lsm303c_acc_report_values(struct lsm303c_acc_status *stat,
					int *xyz)
{
	input_report_abs(stat->input_dev, ABS_X, xyz[0]);
	input_report_abs(stat->input_dev, ABS_Y, xyz[1]);
	input_report_abs(stat->input_dev, ABS_Z, xyz[2]);
	input_sync(stat->input_dev);
}

static void lsm303c_acc_report_triple(struct lsm303c_acc_status *stat)
{
	int err;
	int xyz[3];

	err = lsm303c_acc_get_data(stat, xyz);
	if (err < 0)
		dev_err(&stat->client->dev, "get_data failed\n");
	else
		lsm303c_acc_report_values(stat, xyz);
}

static irqreturn_t lsm303c_acc_isr1(int irq, void *dev)
{
	struct lsm303c_acc_status *stat = dev;

	disable_irq_nosync(irq);
	queue_work(stat->irq1_work_queue, &stat->irq1_work);
	pr_debug("%s: isr1 queued\n", LSM303C_ACC_DEV_NAME);

	return IRQ_HANDLED;
}

static void lsm303c_acc_irq1_work_func(struct work_struct *work)
{

	struct lsm303c_acc_status *stat =
	container_of(work, struct lsm303c_acc_status, irq1_work);
	/* TODO  add interrupt service procedure.
		 ie:lsm303c_acc_get_int1_source(stat); */
	/* ; */
	pr_debug("%s: IRQ1 served\n", LSM303C_ACC_DEV_NAME);
/* exit: */
	enable_irq(stat->irq1);
}

static int lsm303c_acc_enable(struct lsm303c_acc_status *stat)
{
	int err;

	if (!atomic_cmpxchg(&stat->enabled, 0, 1)) {
		err = lsm303c_acc_device_power_on(stat);
		if (err < 0) {
			atomic_set(&stat->enabled, 0);
			return err;
		}
		stat->polling_ktime = ktime_set(
				stat->pdata->poll_interval / 1000,
				MS_TO_NS(stat->pdata->poll_interval % 1000));
		hrtimer_start(&stat->hr_timer_poll,
					stat->polling_ktime, HRTIMER_MODE_REL);
	}
	return 0;
}

static int lsm303c_acc_disable(struct lsm303c_acc_status *stat)
{
	if (atomic_cmpxchg(&stat->enabled, 1, 0)) {
		cancel_work_sync(&stat->input_poll_work);
		lsm303c_acc_device_power_off(stat);
	}

	return 0;
}


static ssize_t read_single_reg(struct device *dev, char *buf, u8 reg)
{
	struct lsm303c_acc_status *stat = dev_get_drvdata(dev);
	int err;

	u8 data = reg;
	err = lsm303c_acc_i2c_read(stat, &data, 1);
	if (err < 0)
		return err;

	return sprintf(buf, "0x%02x\n", data);
}

static int write_reg(struct device *dev, const char *buf, u8 reg,
						u8 mask, int resume_index)
{
	int err = -1;
	struct lsm303c_acc_status *stat = dev_get_drvdata(dev);
	u8 x[2];
	u8 new_val;
	unsigned long val;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;

	new_val = ((u8) val & mask);
	x[0] = reg;
	x[1] = new_val;
	err = lsm303c_acc_register_write(stat, x, reg, new_val);
	if (err < 0)
		return err;
	stat->resume_state[resume_index] = new_val;
	return err;
}

static ssize_t attr_get_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct lsm303c_acc_status *stat = dev_get_drvdata(dev);
	mutex_lock(&stat->lock);
	val = stat->pdata->poll_interval;
	mutex_unlock(&stat->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	int err;
	struct lsm303c_acc_status *stat = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (kstrtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	interval_ms = max_t(unsigned int, (unsigned int)interval_ms,
						stat->pdata->min_interval);
	mutex_lock(&stat->lock);
	stat->pdata->poll_interval = interval_ms;
	err = lsm303c_acc_update_odr(stat, interval_ms);
	if (err >= 0) {
		stat->pdata->poll_interval = interval_ms;
		stat->polling_ktime = ktime_set(
				stat->pdata->poll_interval / 1000,
				MS_TO_NS(stat->pdata->poll_interval % 1000));
	}
	mutex_unlock(&stat->lock);
	return size;
}

static ssize_t attr_get_range(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	char val;
	struct lsm303c_acc_status *stat = dev_get_drvdata(dev);
	char range = 2;
	mutex_lock(&stat->lock);
	val = stat->pdata->fs_range;
	switch (val) {
	case LSM303C_ACC_FS_2G:
		range = 2;
		break;
	case LSM303C_ACC_FS_4G:
		range = 4;
		break;
	case LSM303C_ACC_FS_8G:
		range = 8;
		break;
	}
	mutex_unlock(&stat->lock);
	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct lsm303c_acc_status *stat = dev_get_drvdata(dev);
	unsigned long val;
	u8 range;
	int err;
	if (kstrtoul(buf, 10, &val))
		return -EINVAL;
	switch (val) {
	case 2:
		range = LSM303C_ACC_FS_2G;
		break;
	case 4:
		range = LSM303C_ACC_FS_4G;
		break;
	case 8:
		range = LSM303C_ACC_FS_8G;
		break;
	default:
		dev_err(&stat->client->dev,
				"invalid range request: %lu, discarded\n", val);
		return -EINVAL;
	}
	mutex_lock(&stat->lock);
	err = lsm303c_acc_update_fs_range(stat, range);
	if (err < 0) {
		mutex_unlock(&stat->lock);
		return err;
	}
	stat->pdata->fs_range = range;
	mutex_unlock(&stat->lock);
	dev_info(&stat->client->dev, "range set to: %lu g\n", val);

	return size;
}

static ssize_t attr_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lsm303c_acc_status *stat = dev_get_drvdata(dev);
	int val = atomic_read(&stat->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct lsm303c_acc_status *stat = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lsm303c_acc_enable(stat);
	else
		lsm303c_acc_disable(stat);

	return size;
}

static ssize_t attr_set_intconfig1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_CFG1, NO_MASK, RES_INT_CFG1);
}

static ssize_t attr_get_intconfig1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_CFG1);
}

static ssize_t attr_set_duration1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_DUR1, INT1_DURATION_MASK, RES_INT_DUR1);
}

static ssize_t attr_get_duration1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_DUR1);
}

static ssize_t attr_set_threshx1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_THSX1,
					INT1_THRESHOLD_MASK, RES_INT_THSX1);
}

static ssize_t attr_get_threshx1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_THSX1);
}

static ssize_t attr_set_threshy1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_THSY1,
					INT1_THRESHOLD_MASK, RES_INT_THSY1);
}

static ssize_t attr_get_threshy1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_THSY1);
}

static ssize_t attr_set_threshz1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_THSZ1,
					INT1_THRESHOLD_MASK, RES_INT_THSZ1);
}

static ssize_t attr_get_threshz1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev, buf, INT_THSZ1);
}

static ssize_t attr_get_source1(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, INT_SRC1);
}


#ifdef DEBUG
/* PAY ATTENTION: These DEBUG functions don't manage resume_state */
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int rc;
	struct lsm303c_acc_status *stat = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

	if (kstrtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&stat->lock);
	x[0] = stat->reg_addr;
	mutex_unlock(&stat->lock);
	x[1] = val;
	rc = lsm303c_acc_i2c_write(stat, x, 1);
	/*TODO: error need to be managed */
	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t ret;
	struct lsm303c_acc_status *stat = dev_get_drvdata(dev);
	int rc;
	u8 data;

	mutex_lock(&stat->lock);
	data = stat->reg_addr;
	mutex_unlock(&stat->lock);
	rc = lsm303c_acc_i2c_read(stat, &data, 1);
	/*TODO: error need to be managed */
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct lsm303c_acc_status *stat = dev_get_drvdata(dev);
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

	__ATTR(pollrate_ms, 6, attr_get_polling_rate, attr_set_polling_rate),
	__ATTR(range, 0666, attr_get_range, attr_set_range),
	__ATTR(enable_device, 0666, attr_get_enable, attr_set_enable),
	__ATTR(int1_config, 0664, attr_get_intconfig1, attr_set_intconfig1),
	__ATTR(int1_duration, 0664, attr_get_duration1, attr_set_duration1),
	__ATTR(int1_thresholdx, 0664, attr_get_threshx1, attr_set_threshx1),
	__ATTR(int1_thresholdy, 0664, attr_get_threshy1, attr_set_threshy1),
	__ATTR(int1_thresholdz, 0664, attr_get_threshz1, attr_set_threshz1),
	__ATTR(int1_source, 0444, attr_get_source1, NULL),


#ifdef DEBUG
	__ATTR(reg_value, 0600, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0200, NULL, attr_addr_set),
#endif
};
#else

static DEVICE_ATTR(pollrate_ms, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH, attr_get_polling_rate, attr_set_polling_rate);

static DEVICE_ATTR(range, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH, attr_get_range, attr_set_range);

static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH, attr_get_enable, attr_set_enable);

static DEVICE_ATTR(int1_config, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH, attr_get_intconfig1, attr_set_intconfig1);

static DEVICE_ATTR(int1_duration, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH, attr_get_duration1, attr_set_duration1);

static DEVICE_ATTR(int1_thresholdx, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH, attr_get_threshx1, attr_set_threshx1);

static DEVICE_ATTR(int1_thresholdy, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH, attr_get_threshy1, attr_set_threshy1);

static DEVICE_ATTR(int1_thresholdz, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH, attr_get_threshz1, attr_set_threshz1);

static DEVICE_ATTR(int1_source, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH, attr_get_source1, NULL);


static DEVICE_ATTR(reg_value, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH, attr_reg_get, attr_reg_set);

static DEVICE_ATTR(reg_addr, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH, NULL, attr_addr_set);


#if 0
static DEVICE_ATTR(click_timelatency, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH, attr_get_click_tlat,
						attr_set_click_tlat);
static DEVICE_ATTR(click_timewindow, S_IRUGO|S_IWUSR|S_IWGRP|S_IWOTH, attr_get_click_tw, attr_set_click_tw);
#endif

static struct attribute *attributes[] = {
	&dev_attr_pollrate_ms.attr,
	&dev_attr_range.attr,
	&dev_attr_enable.attr,
	&dev_attr_int1_config.attr,
	&dev_attr_int1_duration.attr,
	&dev_attr_int1_thresholdx.attr,
	&dev_attr_int1_thresholdy.attr,
	&dev_attr_int1_thresholdz.attr,
	&dev_attr_int1_source.attr,
	&dev_attr_reg_value.attr,
	&dev_attr_reg_addr.attr,

#if 0 	
	&dev_attr_int1_threshold.attr,
#endif

#if 0	
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


static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}
#endif

static void lsm303c_acc_input_poll_work_func(struct work_struct *work)
{
	struct lsm303c_acc_status *stat;

	stat = container_of((struct work_struct *) work,
			struct lsm303c_acc_status, input_poll_work);

	mutex_lock(&stat->lock);
	lsm303c_acc_report_triple(stat);
	mutex_unlock(&stat->lock);

	if (atomic_read(&stat->enabled))
		hrtimer_start(&stat->hr_timer_poll,
					stat->polling_ktime, HRTIMER_MODE_REL);
}

enum hrtimer_restart lsm303c_acc_hr_timer_poll_function(struct hrtimer *timer)
{
	struct lsm303c_acc_status *stat;

	stat = container_of((struct hrtimer *)timer,
				struct lsm303c_acc_status, hr_timer_poll);

	queue_work(stat->hr_timer_poll_work_queue, &stat->input_poll_work);
	return HRTIMER_NORESTART;
}

int lsm303c_acc_input_open(struct input_dev *input)
{
	struct lsm303c_acc_status *stat = input_get_drvdata(input);
#ifdef DEBUG	
	dev_info(&stat->client->dev, "%s\n", __func__);
#else
	dev_dbg(&stat->client->dev, "%s\n", __func__);
#endif
	return lsm303c_acc_enable(stat);
}

void lsm303c_acc_input_close(struct input_dev *dev)
{
	struct lsm303c_acc_status *stat = input_get_drvdata(dev);
#ifdef DEBUG	
		dev_info(&stat->client->dev, "%s\n", __func__);
#else
		dev_dbg(&stat->client->dev, "%s\n", __func__);
#endif
	lsm303c_acc_disable(stat);
}

static int lsm303c_acc_validate_pdata(struct lsm303c_acc_status *stat)
{
	/* checks for correctness of minimal polling period */
	stat->pdata->min_interval =
		max((unsigned int)LSM303C_ACC_MIN_POLL_PERIOD_MS,
						stat->pdata->min_interval);

	stat->pdata->poll_interval = max(stat->pdata->poll_interval,
			stat->pdata->min_interval);

	if (stat->pdata->axis_map_x > 2 ||
		stat->pdata->axis_map_y > 2 ||
		 stat->pdata->axis_map_z > 2) {
		dev_err(&stat->client->dev,
			"invalid axis_map value x:%u y:%u z%u\n",
						stat->pdata->axis_map_x,
						stat->pdata->axis_map_y,
						stat->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (stat->pdata->negate_x > 1 || stat->pdata->negate_y > 1
			|| stat->pdata->negate_z > 1) {
		dev_err(&stat->client->dev,
			"invalid negate value x:%u y:%u z:%u\n",
						stat->pdata->negate_x,
						stat->pdata->negate_y,
						stat->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (stat->pdata->poll_interval < stat->pdata->min_interval) {
		dev_err(&stat->client->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int lsm303c_acc_input_init(struct lsm303c_acc_status *stat)
{
	int err;

	INIT_WORK(&stat->input_poll_work, lsm303c_acc_input_poll_work_func);
	stat->input_dev = input_allocate_device();
	if (!stat->input_dev) {
		err = -ENOMEM;
		dev_err(&stat->client->dev, "input device allocation failed\n");
		goto err0;
	}

	stat->input_dev->open = lsm303c_acc_input_open;
	stat->input_dev->close = lsm303c_acc_input_close;
	//stat->input_dev->name = LSM303C_ACC_DEV_NAME;
	stat->input_dev->uniq = LSM303C_ACC_DEV_NAME;
	stat->input_dev->name = "accelerometer";
	stat->input_dev->id.bustype = BUS_I2C;
	stat->input_dev->dev.parent = &stat->client->dev;

	input_set_drvdata(stat->input_dev, stat);

	set_bit(EV_ABS, stat->input_dev->evbit);

	/*	next is used for interruptA sources data if the case */
	set_bit(ABS_MISC, stat->input_dev->absbit);
	/*	next is used for interruptB sources data if the case */
	set_bit(ABS_WHEEL, stat->input_dev->absbit);

	input_set_abs_params(stat->input_dev, ABS_X, G_MIN, G_MAX, FUZZ, FLAT);
	input_set_abs_params(stat->input_dev, ABS_Y, G_MIN, G_MAX, FUZZ, FLAT);
	input_set_abs_params(stat->input_dev, ABS_Z, G_MIN, G_MAX, FUZZ, FLAT);

	/*	next is used for interruptA sources data if the case */
	input_set_abs_params(stat->input_dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);
	/*	next is used for interruptB sources data if the case */
	input_set_abs_params(stat->input_dev, ABS_WHEEL, INT_MIN,
								INT_MAX, 0, 0);

	err = input_register_device(stat->input_dev);
	if (err) {
		dev_err(&stat->client->dev,
				"unable to register input device %s\n",
				stat->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(stat->input_dev);
err0:
	return err;
}

static void lsm303c_acc_input_cleanup(struct lsm303c_acc_status *stat)
{
	input_unregister_device(stat->input_dev);
	input_free_device(stat->input_dev);
}

#ifdef CONFIG_OF
static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0) ?
		regulator_set_optimum_mode(reg, load_uA) : 0;
}

static int sensor_regulator_configure(struct lsm303c_acc_status *data, bool on)
{
	struct i2c_client *client = data->client;
	struct lsm303c_acc_platform_data *pdata = data->pdata;
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

static int sensor_regulator_power_on(struct lsm303c_acc_status *data, bool on)
{
	struct i2c_client *client = data->client;
	struct lsm303c_acc_platform_data *pdata = data->pdata;
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
	sensor_regulator_power_on(g_ptr_acc_stat, on);
	return 0;
}

static int sensor_platform_hw_power_off(void)
{
	sensor_regulator_power_on(g_ptr_acc_stat, 0);
	return 0;
}

static int sensor_platform_hw_init(void)
{
	struct i2c_client *client = g_ptr_acc_stat->client;
	struct lsm303c_acc_status *data = g_ptr_acc_stat;
	int error;

	error = sensor_regulator_configure(data, true);

	if (gpio_is_valid(data->pdata->irq_gpio)) {
		error = gpio_request(data->pdata->irq_gpio, "lm303c_acc_irq_gpio");
		if (error) {
			dev_err(&client->dev, "unable to request gpio [%d]\n",
						data->pdata->irq_gpio);
		}
		error = gpio_direction_input(data->pdata->irq_gpio);
		if (error) {
			dev_err(&client->dev,
				"unable to set direction for gpio [%d]\n",
				data->pdata->irq_gpio);
		}
		data->irq1 = client->irq = gpio_to_irq(data->pdata->irq_gpio);
	} else {
		dev_err(&client->dev, "irq gpio not provided\n");
	}
		return 0;
}

static void sensor_platform_hw_exit(void)
{
	struct lsm303c_acc_status *data = g_ptr_acc_stat;

	sensor_regulator_configure(data, false);

	if (gpio_is_valid(data->pdata->irq_gpio))
		gpio_free(data->pdata->irq_gpio);
}

static int sensor_parse_dt(struct device *dev, struct lsm303c_acc_platform_data *pdata)
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

static int lsm303c_acc_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{

	struct lsm303c_acc_status *stat;

	u32 smbus_func = (I2C_FUNC_SMBUS_BYTE_DATA |
			I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_I2C_BLOCK);

	int err = -1;

	dev_info(&client->dev, "probe start.\n");

	g_ptr_acc_stat = stat = kzalloc(sizeof(struct lsm303c_acc_status), GFP_KERNEL);
	if (stat == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for module data: "
					"%d\n", err);
		goto exit_check_functionality_failed;
	}

	/* Support for both I2C and SMBUS adapter interfaces. */
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


	mutex_init(&stat->lock);
	mutex_lock(&stat->lock);

	stat->client = client;
	i2c_set_clientdata(client, stat);
#ifdef CONFIG_OF
	if(client->dev.of_node) {
		stat->pdata = devm_kzalloc(&client->dev,sizeof(struct lsm303c_acc_platform_data), GFP_KERNEL);
		if ( NULL == stat->pdata ) {
			err = -ENOMEM;
			dev_err(&client->dev,"failed to allocate memory for pdata: %d\n",err);
			goto err_mutexunlock;
		}
		memcpy(stat->pdata, &default_lsm303c_acc_pdata,sizeof(struct lsm303c_acc_platform_data));
		client->dev.platform_data = stat->pdata;
		err = sensor_parse_dt(&client->dev, stat->pdata);
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
	stat->hr_timer_poll_work_queue = 0;
	err = lsm303c_acc_validate_pdata(stat);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto exit_kfree_pdata;
	}


	if (stat->pdata->init) {
		err = stat->pdata->init();
		if (err < 0) {
			dev_err(&client->dev, "init failed: %d\n", err);
			goto err_pdata_init;
		}
	}

	lsm303c_acc_set_init_register_values(stat);

	err = lsm303c_acc_device_power_on(stat);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err_pdata_init;
	}

	atomic_set(&stat->enabled, 1);

	err = lsm303c_acc_update_fs_range(stat, stat->pdata->fs_range);
	if (err < 0) {
		dev_err(&client->dev, "update_fs_range failed\n");
		goto  err_power_off;
	}

	err = lsm303c_acc_update_odr(stat, stat->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto  err_power_off;
	}

	stat->hr_timer_poll_work_queue =
			create_workqueue("lsm303c_acc_hr_timer_poll_wq");
	hrtimer_init(&stat->hr_timer_poll, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	stat->hr_timer_poll.function = &lsm303c_acc_hr_timer_poll_function;

	err = lsm303c_acc_input_init(stat);
	if (err < 0) {
		dev_err(&client->dev, "input init failed\n");
		goto err_remove_hr_work_queue;
	}

#if 0
	err = create_sysfs_interfaces(&client->dev);
#else
	err = sysfs_create_group(&stat->input_dev->dev.kobj,
		&k303c_attribute_group);
#endif
	
	if (err < 0) {
		dev_err(&client->dev,
		   "device LSM303C_ACC_DEV_NAME sysfs register failed\n");
		goto err_input_cleanup;
	}


	lsm303c_acc_device_power_off(stat);

	/* As default, do not report information */
	atomic_set(&stat->enabled, 0);

	if (stat->pdata->irq_gpio >= 0) {
		INIT_WORK(&stat->irq1_work, lsm303c_acc_irq1_work_func);
		stat->irq1_work_queue =
			create_singlethread_workqueue("lsm303c_acc_wq1");
		if (!stat->irq1_work_queue) {
			err = -ENOMEM;
			dev_err(&client->dev,
					"cannot create work queue1: %d\n", err);
			goto err_remove_sysfs_int;
		}
		err = request_irq(stat->irq1, lsm303c_acc_isr1,
			IRQF_TRIGGER_RISING, "lsm303c_acc_irq1", stat);
		if (err < 0) {
			dev_err(&client->dev, "request irq1 failed: %d\n", err);
			goto err_destoyworkqueue1;
		}
		disable_irq_nosync(stat->irq1);
	}

	mutex_unlock(&stat->lock);

	dev_info(&client->dev, "%s: probed\n", LSM303C_ACC_DEV_NAME);

	return 0;

err_destoyworkqueue1:
	if (stat->pdata->irq_gpio >= 0)
		destroy_workqueue(stat->irq1_work_queue);
err_remove_sysfs_int:
#if 0
	remove_sysfs_interfaces(&client->dev);
#else
sysfs_remove_group(&stat->input_dev->dev.kobj,
		&k303c_attribute_group);
#endif
	
err_input_cleanup:
	lsm303c_acc_input_cleanup(stat);
err_remove_hr_work_queue:
	if (!stat->hr_timer_poll_work_queue) {
			flush_workqueue(stat->hr_timer_poll_work_queue);
			destroy_workqueue(stat->hr_timer_poll_work_queue);
	}
err_power_off:
	lsm303c_acc_device_power_off(stat);
err_pdata_init:
	if (stat->pdata->exit)
		stat->pdata->exit();
exit_kfree_pdata:
err_mutexunlock:
	mutex_unlock(&stat->lock);
/* err_freedata: */
	kfree(stat);
exit_check_functionality_failed:
	pr_err("%s: Driver Init failed\n", LSM303C_ACC_DEV_NAME);
	return err;
}

static int lsm303c_acc_remove(struct i2c_client *client)
{

	struct lsm303c_acc_status *stat = i2c_get_clientdata(client);

	dev_info(&stat->client->dev, "driver removing\n");

	if (stat->pdata->irq_gpio >= 0) {
		free_irq(stat->irq1, stat);
		gpio_free(stat->pdata->irq_gpio);
		destroy_workqueue(stat->irq1_work_queue);
	}

	lsm303c_acc_disable(stat);
	lsm303c_acc_input_cleanup(stat);

#if 0
	remove_sysfs_interfaces(&client->dev);
#else
sysfs_remove_group(&stat->input_dev->dev.kobj,
		&k303c_attribute_group);
#endif 
	if (!stat->hr_timer_poll_work_queue) {
			flush_workqueue(stat->hr_timer_poll_work_queue);
			destroy_workqueue(stat->hr_timer_poll_work_queue);
	}

	if (stat->pdata->exit)
		stat->pdata->exit();
	kfree(stat->pdata);
	kfree(stat);

	return 0;
}

#ifdef CONFIG_PM
static int lsm303c_acc_resume(struct i2c_client *client)
{
	struct lsm303c_acc_status *stat = i2c_get_clientdata(client);

	if (stat->on_before_suspend)
		return lsm303c_acc_enable(stat);

	return 0;
}

static int lsm303c_acc_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct lsm303c_acc_status *stat = i2c_get_clientdata(client);

	stat->on_before_suspend = atomic_read(&stat->enabled);
	return lsm303c_acc_disable(stat);
}
#else
#define lsm303c_acc_suspend	NULL
#define lsm303c_acc_resume	NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id lsm303c_acc_id[] = {
		{ LSM303C_ACC_DEV_NAME, 0 },
		{ },
};

MODULE_DEVICE_TABLE(i2c, lsm303c_acc_id);

#ifdef CONFIG_OF
static struct of_device_id K303C_acc_match_table[] = {
    { .compatible = "STMicroelectronics,accelerometer",},
    { },
};
#else
#define K303C_acc_match_table NULL
#endif

static struct i2c_driver lsm303c_acc_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = LSM303C_ACC_DEV_NAME,
			.of_match_table = K303C_acc_match_table,
	},
	.probe = lsm303c_acc_probe,
	.remove = __devexit_p(lsm303c_acc_remove),
	.suspend = lsm303c_acc_suspend,
	.resume = lsm303c_acc_resume,
	.id_table = lsm303c_acc_id,
};

static int __init lsm303c_acc_init(void)
{
	pr_info("%s accelerometer driver: init\n", LSM303C_ACC_DEV_NAME);
	return i2c_add_driver(&lsm303c_acc_driver);
}

static void __exit lsm303c_acc_exit(void)
{
	i2c_del_driver(&lsm303c_acc_driver);
}

module_init(lsm303c_acc_init);
module_exit(lsm303c_acc_exit);

MODULE_DESCRIPTION("lsm303c accelerometer driver");
MODULE_AUTHOR("Matteo Dameno, Denis Ciocca, STMicroelectronics");
MODULE_LICENSE("GPL");

