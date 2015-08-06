/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
 *
 * File Name          : k3dh_acc.c
 * Authors            : MSH - Motion Mems BU - Application Team
 *		      : Carmine Iascone (carmine.iascone@st.com)
 *		      : Matteo Dameno (matteo.dameno@st.com)
 * Version            : V.1.0.6
 * Date               : 02/03/2014
 * Description        : K3DH accelerometer sensor API
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
 * THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
 *
 ******************************************************************************
 Revision 1.0.6 02/03/2014
 Added self test function

 Revision 1.0.0 05/11/09
 First Release
 Revision 1.0.3 22/01/2010
  Linux K&R Compliant Release;
 Revision 1.0.5 16/08/2010
 modified _get_acceleration_data function
 modified _update_odr function
 manages 2 interrupts

 ******************************************************************************/

#include	<linux/err.h>
#include	<linux/errno.h>
#include	<linux/delay.h>
#include	<linux/fs.h>
#include	<linux/i2c.h>

#include	<linux/input.h>
#include	<linux/input-polldev.h>
#include	<linux/miscdevice.h>
#include	<linux/uaccess.h>

#include	<linux/workqueue.h>
#include	<linux/irq.h>
#include	<linux/gpio.h>
#include	<linux/interrupt.h>
#include	<linux/slab.h>
#include	<linux/module.h>

#include	<linux/regulator/consumer.h>
#include	<linux/of_gpio.h>

#ifdef K3DH_ACCEL_CALIBRATION
#include <linux/syscalls.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/time.h>
#endif
#include	"k3dh.h"

#define USE_DEFAULT_PINCTRL_NAME  /* Using pinctrl-names value is "default" in DTSi file */

#define K3DH_OPEN_ENABLE	1
#define K3DH_AXIS_X             0
#define K3DH_AXIS_Y             1
#define K3DH_AXIS_Z             2

#define DEBUG	0

#define	INTERRUPT_MANAGEMENT 0

#define	G_MAX		16000	/** Maximum polled-device-reported g value */

/*
#define	SHIFT_ADJ_2G		4
#define	SHIFT_ADJ_4G		3
#define	SHIFT_ADJ_8G		2
#define	SHIFT_ADJ_16G		1
*/

#define SENSITIVITY_2G		1	/**	mg/LSB	*/
#define SENSITIVITY_4G		2	/**	mg/LSB	*/
#define SENSITIVITY_8G		4	/**	mg/LSB	*/
#define SENSITIVITY_16G		12	/**	mg/LSB	*/


#define	HIGH_RESOLUTION		0x08

#define	AXISDATA_REG		0x28
#define WHOAMI_K3DH_ACC	0x33	/*	Expctd content for WAI	*/

/*	CONTROL REGISTERS	*/
#define WHO_AM_I		0x0F	/*	WhoAmI register		*/
#define	TEMP_CFG_REG		0x1F	/*	temper sens control reg	*/
/* ctrl 1: ODR3 ODR2 ODR ODR0 LPen Zenable Yenable Zenable */
#define	CTRL_REG1		0x20	/*	control reg 1		*/
#define	CTRL_REG2		0x21	/*	control reg 2		*/
#define	CTRL_REG3		0x22	/*	control reg 3		*/
#define	CTRL_REG4		0x23	/*	control reg 4		*/
#define	CTRL_REG5		0x24	/*	control reg 5		*/
#define	CTRL_REG6		0x25	/*	control reg 6		*/

#define	FIFO_CTRL_REG		0x2E	/*	FiFo control reg	*/

#define	INT_CFG1		0x30	/*	interrupt 1 config	*/
#define	INT_SRC1		0x31	/*	interrupt 1 source	*/
#define	INT_THS1		0x32	/*	interrupt 1 threshold	*/
#define	INT_DUR1		0x33	/*	interrupt 1 duration	*/

#define	INT_CFG2		0x34	/*	interrupt 2 config	*/
#define	INT_SRC2		0x35	/*	interrupt 2 source	*/
#define	INT_THS2		0x36	/*	interrupt 2 threshold	*/
#define	INT_DUR2		0x37	/*	interrupt 2 duration	*/

#define	TT_CFG			0x38	/*	tap config		*/
#define	TT_SRC			0x39	/*	tap source		*/
#define	TT_THS			0x3A	/*	tap threshold		*/
#define	TT_LIM			0x3B	/*	tap time limit		*/
#define	TT_TLAT			0x3C	/*	tap time latency	*/
#define	TT_TW			0x3D	/*	tap time window		*/
/*	end CONTROL REGISTRES	*/


#define ENABLE_HIGH_RESOLUTION	1

#define K3DH_ACC_PM_OFF		0x00
#define K3DH_ACC_ENABLE_ALL_AXES	0x07
#ifdef K3DH_ACCEL_CALIBRATION
#define K3DH_SHAKING_DETECT_THRESHOLD 200
#define CALIBRATION_DATA_AMOUNT 10
#endif

#define PMODE_MASK			0x08
#define ODR_MASK			0XF0

#define ODR1		0x10  /* 1Hz output data rate */
#define ODR10		0x20  /* 10Hz output data rate */
#define ODR25		0x30  /* 25Hz output data rate */
#define ODR50		0x40  /* 50Hz output data rate */
#define ODR100		0x50  /* 100Hz output data rate */
#define ODR200		0x60  /* 200Hz output data rate */
#define ODR400		0x70  /* 400Hz output data rate */
#define ODR1250		0x90  /* 1250Hz output data rate */



#define	IA			0x40
#define	ZH			0x20
#define	ZL			0x10
#define	YH			0x08
#define	YL			0x04
#define	XH			0x02
#define	XL			0x01
/* */
/* CTRL REG BITS*/
#define	CTRL_REG3_I1_AOI1	0x40
#define	CTRL_REG6_I2_TAPEN	0x80
#define	CTRL_REG6_HLACTIVE	0x02
/* */

#define K3DH_LP_EN_MSK      0x08
#define K3DH_LP_EN_POS      3
#define K3DH_LP_EN_REG      CTRL_REG1

#define K3DH_HR_MSK             0x08
#define K3DH_HR_POS             3
#define K3DH_HR_REG             CTRL_REG4

#define K3DH_FS_MSK             0x30
#define K3DH_FS_POS             4
#define K3DH_FS_REG             CTRL_REG4

#define K3DH_ODR_MSK          0xF0
#define K3DH_ODR_POS          4
#define K3DH_ODR_REG          CTRL_REG1

#define K3DH_MODE_LOWPOWER				0
#define K3DH_MODE_NORMAL    			1
#define K3DH_MODE_HIGH_RESOLUTION		2

/* TAP_SOURCE_REG BIT */
#define	DTAP			0x20
#define	STAP			0x10
#define	SIGNTAP			0x08
#define	ZTAP			0x04
#define	YTAP			0x02
#define	XTAZ			0x01

#ifdef ACC_INT_MODE
#define	FUZZ			32
#define	FLAT			32
#else
#define	FUZZ			0
#define	FLAT			0
#endif
#define	I2C_RETRY_DELAY		5
#define	I2C_RETRIES		5
#define	I2C_AUTO_INCREMENT	0x80

/* RESUME STATE INDICES */
#define	RES_CTRL_REG1		0
#define	RES_CTRL_REG2		1
#define	RES_CTRL_REG3		2
#define	RES_CTRL_REG4		3
#define	RES_CTRL_REG5		4
#define	RES_CTRL_REG6		5

#define	RES_INT_CFG1		6
#define	RES_INT_THS1		7
#define	RES_INT_DUR1		8
#define	RES_INT_CFG2		9
#define	RES_INT_THS2		10
#define	RES_INT_DUR2		11

#define	RES_TT_CFG		12
#define	RES_TT_THS		13
#define	RES_TT_LIM		14
#define	RES_TT_TLAT		15
#define	RES_TT_TW		16

#define	RES_TEMP_CFG_REG	17
#define	RES_REFERENCE_REG	18
#define	RES_FIFO_CTRL_REG	19

#define	RESUME_ENTRIES		20
/* end RESUME STATE INDICES */

struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} k3dh_acc_odr_table[] = {
			{ 1, ODR1250 },
			{ 3, ODR400 },
			{ 5, ODR200 },
			{ 10, ODR100 },
			{ 20, ODR50 },
			{ 40, ODR25 },
			{ 100, ODR10 },
			{ 1000, ODR1 },
};

/* K3DH acceleration data */
struct k3dh_acc {
	s16 x;
	s16 y;
	s16 z;
	int bCalLoaded;
};

struct k3dh_acc_data {
	struct i2c_client *client;
	struct k3dh_acc_platform_data *pdata;

	struct mutex lock;
	struct delayed_work input_work;

	struct input_dev *input_dev;
#ifdef K3DH_ACCEL_CALIBRATION
	struct k3dh_acc cal_data;
#endif
	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	atomic_t enabled;
	atomic_t selftest_rslt;
	int on_before_suspend;

	u8 sensitivity;

	u8 resume_state[RESUME_ENTRIES];

	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
#ifdef USE_ACC_IRQ2
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;
#endif

	struct pinctrl *k3dh_pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;

#ifdef K3DH_ACCEL_CALIBRATION
		atomic_t fast_calib_x_rslt;
		atomic_t fast_calib_y_rslt;
		atomic_t fast_calib_z_rslt;
		atomic_t fast_calib_rslt;
#endif
};

/*
 * Because misc devices can not carry a pointer from driver register to
 * open, we keep this global.  This limits the driver to a single instance.
 */
struct k3dh_acc_data *k3dh_acc_misc_data;


static int k3dh_acc_i2c_read(struct k3dh_acc_data *acc, u8 *buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg	msgs[] = {
		{
			.addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
			.len = 1,
			.buf = buf, },
		{
			.addr = acc->client->addr,
			.flags = (acc->client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = len,
			.buf = buf, },
	};

	do {
		err = i2c_transfer(acc->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2) {
		dev_err(&acc->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int k3dh_acc_i2c_write(struct k3dh_acc_data *acc, u8 *buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = { { .addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
			.len = len + 1, .buf = buf, }, };
	do {
		err = i2c_transfer(acc->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		dev_err(&acc->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int k3dh_acc_register_write(struct k3dh_acc_data *acc, u8 *buf,
		u8 reg_address, u8 new_value)
{
	int err = -1;

	if (atomic_read(&acc->enabled)) {
		/* Sets configuration register at reg_address
		 *  NOTE: this is a straight overwrite  */
		buf[0] = reg_address;
		buf[1] = new_value;
		err = k3dh_acc_i2c_write(acc, buf, 1);
		if (err < 0)
			return err;
	}
	return err;
}

static int k3dh_acc_register_read(struct k3dh_acc_data *acc, u8 *buf,
		u8 reg_address)
{

	int err = -1;
	buf[0] = (reg_address);
	err = k3dh_acc_i2c_read(acc, buf, 1);
	return err;
}

static int k3dh_acc_hw_init(struct k3dh_acc_data *acc)
{
	int err = -1;
	u8 buf[7];

	pr_info("%s: hw init start\n", K3DH_ACC_DEV_NAME);

	buf[0] = WHO_AM_I;
	err = k3dh_acc_i2c_read(acc, buf, 1);
	if (err < 0)
		goto error_firstread;
	else
		acc->hw_working = 1;
	if (buf[0] != WHOAMI_K3DH_ACC) {
		err = -1; /* choose the right coded error */
		goto error_unknown_device;
	}

	buf[0] = CTRL_REG1;
	buf[1] = acc->resume_state[RES_CTRL_REG1];
	err = k3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto error1;

	buf[0] = TEMP_CFG_REG;
	buf[1] = acc->resume_state[RES_TEMP_CFG_REG];
	err = k3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto error1;

	buf[0] = FIFO_CTRL_REG;
	buf[1] = acc->resume_state[RES_FIFO_CTRL_REG];
	err = k3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto error1;

	buf[0] = (I2C_AUTO_INCREMENT | TT_THS);
	buf[1] = acc->resume_state[RES_TT_THS];
	buf[2] = acc->resume_state[RES_TT_LIM];
	buf[3] = acc->resume_state[RES_TT_TLAT];
	buf[4] = acc->resume_state[RES_TT_TW];
	err = k3dh_acc_i2c_write(acc, buf, 4);
	if (err < 0)
		goto error1;
	buf[0] = TT_CFG;
	buf[1] = acc->resume_state[RES_TT_CFG];
	err = k3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto error1;

	buf[0] = (I2C_AUTO_INCREMENT | INT_THS1);
	buf[1] = acc->resume_state[RES_INT_THS1];
	buf[2] = acc->resume_state[RES_INT_DUR1];
	err = k3dh_acc_i2c_write(acc, buf, 2);
	if (err < 0)
		goto error1;
	buf[0] = INT_CFG1;
	buf[1] = acc->resume_state[RES_INT_CFG1];
	err = k3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto error1;

	buf[0] = (I2C_AUTO_INCREMENT | INT_THS2);
	buf[1] = acc->resume_state[RES_INT_THS2];
	buf[2] = acc->resume_state[RES_INT_DUR2];
	err = k3dh_acc_i2c_write(acc, buf, 2);
	if (err < 0)
		goto error1;
	buf[0] = INT_CFG2;
	buf[1] = acc->resume_state[RES_INT_CFG2];
	err = k3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto error1;

	buf[0] = (I2C_AUTO_INCREMENT | CTRL_REG2);
	buf[1] = acc->resume_state[RES_CTRL_REG2];
	buf[2] = acc->resume_state[RES_CTRL_REG3];
	buf[3] = acc->resume_state[RES_CTRL_REG4];
	buf[4] = acc->resume_state[RES_CTRL_REG5];
	buf[5] = acc->resume_state[RES_CTRL_REG6];
	err = k3dh_acc_i2c_write(acc, buf, 5);
	if (err < 0)
		goto error1;

	acc->hw_initialized = 1;
	pr_info("%s: hw init done\n", K3DH_ACC_DEV_NAME);
	return 0;

error_firstread:
	acc->hw_working = 0;
	dev_warn(&acc->client->dev,
		 "Error reading WHO_AM_I: is device available/working?\n");
	goto error1;
error_unknown_device:
	dev_err(&acc->client->dev,
		"device unknown. Expected: 0x%x, Replies: 0x%x\n",
		 WHOAMI_K3DH_ACC, buf[0]);
error1:
	acc->hw_initialized = 0;
	dev_err(&acc->client->dev, "hw init error 0x%x,0x%x: %d\n", buf[0],
			buf[1], err);
	return err;
}

static void k3dh_acc_device_power_off(struct k3dh_acc_data *acc)
{
	int err;
	u8 buf[2] = { CTRL_REG1, K3DH_ACC_PM_OFF };

	err = k3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		dev_err(&acc->client->dev, "soft power off failed: %d\n", err);

	if (acc->pdata->power_off) {
		disable_irq_nosync(acc->irq1);
#ifdef USE_ACC_IRQ2
		disable_irq_nosync(acc->irq2);
#endif
		acc->pdata->power_off(acc->pdata);
		acc->hw_initialized = 0;
	}
	if (acc->hw_initialized) {
		disable_irq_nosync(acc->irq1);
#ifdef USE_ACC_IRQ2
		disable_irq_nosync(acc->irq2);
#endif
		acc->hw_initialized = 0;
	}

}

static int k3dh_acc_device_power_on(struct k3dh_acc_data *acc)
{
	int err = -1;

	if (acc->pdata->power_on) {
		err = acc->pdata->power_on(acc->pdata);
		if (err < 0) {
			dev_err(&acc->client->dev,
					"power_on failed: %d\n", err);
			return err;
		}
		if (acc->pdata->gpio_int1 >= 0)
			enable_irq(acc->irq1);
#ifdef USE_ACC_IRQ2
		if (acc->pdata->gpio_int2 >= 0)
			enable_irq(acc->irq2);
#endif
	}

	if (!acc->hw_initialized) {
		err = k3dh_acc_hw_init(acc);
		if (acc->hw_working == 1 && err < 0) {
			k3dh_acc_device_power_off(acc);
			return err;
		}
	}

	pr_info("%s: end of k3dh_acc_hw_init()\n", K3DH_ACC_DEV_NAME);

	if (acc->hw_initialized) {
		/* for Warning Message Remove */
		/*
		if (acc->pdata->gpio_int1 >=0)
			enable_irq(acc->irq1);
		*/
#ifdef USE_ACC_IRQ2
		if (acc->pdata->gpio_int2 >= 0)
			enable_irq(acc->irq2);
#endif
		pr_info("%s: power on: irq enabled\n",
						K3DH_ACC_DEV_NAME);
	}
	return 0;
}

static irqreturn_t k3dh_acc_isr1(int irq, void *dev)
{
	struct k3dh_acc_data *acc = dev;

	disable_irq_nosync(irq);
	queue_work(acc->irq1_work_queue, &acc->irq1_work);

	pr_info("%s: isr1 queued\n", K3DH_ACC_DEV_NAME);
	return IRQ_HANDLED;
}
/* lgp-s19 don't used this function
     because irq2 doesn't used
*/
#ifdef USE_ACC_IRQ2
static irqreturn_t k3dh_acc_isr2(int irq, void *dev)
{
	struct k3dh_acc_data *acc = dev;

	disable_irq_nosync(irq);
	queue_work(acc->irq2_work_queue, &acc->irq2_work);
	pr_info(KERN_INFO "%s: isr2 queued\n", K3DH_ACC_DEV_NAME);

	return IRQ_HANDLED;
}
#endif


static void k3dh_acc_irq1_work_func(struct work_struct *work)
{

	struct k3dh_acc_data *acc =
	container_of(work, struct k3dh_acc_data, irq1_work);
	/* TODO  add interrupt service procedure.
		 ie:k3dh_acc_get_int1_source(acc); */
	;
	/*  */
	pr_info("%s: IRQ1 triggered\n", K3DH_ACC_DEV_NAME);
/*	k3dh_acc_input_work_func(work); */
/* exit: */
	enable_irq(acc->irq1);
}
#ifdef USE_ACC_IRQ2
static void k3dh_acc_irq2_work_func(struct work_struct *work)
{

	struct k3dh_acc_data *acc =
	container_of(work, struct k3dh_acc_data, irq2_work);
	/* TODO  add interrupt service procedure.
		 ie:k3dh_acc_get_tap_source(acc); */

	/*  */

	pr_info("%s: IRQ2 triggered\n", K3DH_ACC_DEV_NAME);
/* exit: */
	enable_irq(acc->irq2);
}
#endif
int k3dh_acc_update_g_range(struct k3dh_acc_data *acc, u8 new_g_range)
{
	int err;

	u8 sensitivity;
	u8 buf[2];
	u8 updated_val;
	u8 init_val;
	u8 new_val;
	u8 mask = K3DH_ACC_FS_MASK | HIGH_RESOLUTION;

	switch (new_g_range) {
	case K3DH_ACC_G_2G:

		sensitivity = SENSITIVITY_2G;
		break;
	case K3DH_ACC_G_4G:

		sensitivity = SENSITIVITY_4G;
		break;
	case K3DH_ACC_G_8G:

		sensitivity = SENSITIVITY_8G;
		break;
	case K3DH_ACC_G_16G:

		sensitivity = SENSITIVITY_16G;
		break;
	default:
		dev_err(&acc->client->dev, "invalid g range requested: %u\n",
				new_g_range);
		return -EINVAL;
	}

	if (atomic_read(&acc->enabled)) {
		/* Set configuration register 4, which contains g range setting
		 *  NOTE: this is a straight overwrite because this driver does
		 *  not use any of the other configuration bits in this
		 *  register.  Should this become untrue, we will have to read
		 *  out the value and only change the relevant bits --XX----
		 *  (marked by X) */
		buf[0] = CTRL_REG4;
		err = k3dh_acc_i2c_read(acc, buf, 1);
		if (err < 0)
			goto error;
		init_val = buf[0];
		acc->resume_state[RES_CTRL_REG4] = init_val;
		new_val = new_g_range | HIGH_RESOLUTION;
		updated_val = ((mask & new_val) | ((~mask) & init_val));
		buf[1] = updated_val;
		buf[0] = CTRL_REG4;
		err = k3dh_acc_i2c_write(acc, buf, 1);
		if (err < 0)
			goto error;
		acc->resume_state[RES_CTRL_REG4] = updated_val;
		acc->sensitivity = sensitivity;
	}


	return 0;
error:
	dev_err(&acc->client->dev, "update g range failed 0x%x,0x%x: %d\n",
			buf[0], buf[1], err);

	return err;
}

int k3dh_acc_update_odr(struct k3dh_acc_data *acc, int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 config[2];

	/* Convert the poll interval into an output data rate configuration
	 *  that is as low as possible.  The ordering of these checks must be
	 *  maintained due to the cascading cut off values - poll intervals are
	 *  checked from shortest to longest.  At each check, if the next lower
	 *  ODR cannot support the current poll interval, we stop searching */
	for (i = ARRAY_SIZE(k3dh_acc_odr_table) - 1; i >= 0; i--) {
		if (k3dh_acc_odr_table[i].cutoff_ms <= poll_interval_ms || i == 0)
			break;
	}
	config[1] = k3dh_acc_odr_table[i].mask;

	config[1] |= K3DH_ACC_ENABLE_ALL_AXES;

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&acc->enabled)) {
		config[0] = CTRL_REG1;
		err = k3dh_acc_i2c_write(acc, config, 1);
		if (err < 0)
			goto error;
		acc->resume_state[RES_CTRL_REG1] = config[1];
	}

	return 0;

error:
	dev_err(&acc->client->dev, "update odr failed 0x%x,0x%x: %d\n",
			config[0], config[1], err);

	return err;
}
int k3dh_acc_get_odr(struct k3dh_acc_data *acc, unsigned char *odr)
{
	int err = -1;
	unsigned char data[2] = { 0, };
	err = k3dh_acc_register_read(acc, data, K3DH_ODR_REG);
	if(err < 0)
		return err;

	data[0] = (data[0] & K3DH_ODR_MSK) >> K3DH_ODR_POS;
	*odr = data[0];
	return err;
}

static int k3dh_acc_register_update(struct k3dh_acc_data *acc, u8 *buf,
		u8 reg_address, u8 mask, u8 new_bit_values)
{
	int err = -1;
	u8 init_val;
	u8 updated_val;
	err = k3dh_acc_register_read(acc, buf, reg_address);
	if (!(err < 0)) {
		init_val = buf[1];
		updated_val = ((mask & new_bit_values) | ((~mask) & init_val));
		err = k3dh_acc_register_write(acc, buf, reg_address,
				updated_val);
	}
	return err;
}

/* */

static int k3dh_acc_get_acceleration_raw_data(struct k3dh_acc_data *acc,
		int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6] = { 0, };
	/* x,y,z hardware data */
	s16 hw_d[3] = { 0, };

	acc_data[0] = (I2C_AUTO_INCREMENT | AXISDATA_REG);
	err = k3dh_acc_i2c_read(acc, acc_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = (((s16) ((acc_data[1] << 8) | acc_data[0])) >> 4);
	hw_d[1] = (((s16) ((acc_data[3] << 8) | acc_data[2])) >> 4);
	hw_d[2] = (((s16) ((acc_data[5] << 8) | acc_data[4])) >> 4);

	hw_d[0] = hw_d[0] * acc->sensitivity;
	hw_d[1] = hw_d[1] * acc->sensitivity;
	hw_d[2] = hw_d[2] * acc->sensitivity;

	xyz[0] = ((acc->pdata->negate_x) ? (-hw_d[acc->pdata->axis_map_x])
		   : (hw_d[acc->pdata->axis_map_x]));
	xyz[1] = ((acc->pdata->negate_y) ? (-hw_d[acc->pdata->axis_map_y])
		   : (hw_d[acc->pdata->axis_map_y]));
	xyz[2] = ((acc->pdata->negate_z) ? (-hw_d[acc->pdata->axis_map_z])
		   : (hw_d[acc->pdata->axis_map_z]));

	#if DEBUG
		pr_info("%s read raw x=%d, y=%d, z=%d\n",
			K3DH_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);
	#endif
	return err;
}

static int k3dh_acc_get_acceleration_data(struct k3dh_acc_data *acc,
		int *xyz)
{

	int err = -1;

	err = k3dh_acc_get_acceleration_raw_data(acc,xyz);
	if(err < 0) {
		pr_info("k3dh_read_accel_xyz() failed ");
		return err;
	}

	xyz[0] -= acc->cal_data.x;
	xyz[1] -= acc->cal_data.y;
	xyz[2] -= acc->cal_data.z;

	#if DEBUG
		pr_info("%s read calibrated x=%d, y=%d, z=%d\n",
			K3DH_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);
	#endif

	return err;
}

#ifdef K3DH_ACCEL_CALIBRATION
static int k3dh_read_Calibration_data(struct i2c_client *client)
{
	int fd_offset_x, fd_offset_y, fd_offset_z;
	char offset_x_src[5],offset_y_src[5],offset_z_src[5];
	long offset_x,offset_y,offset_z;

	struct k3dh_acc_data *k3dh = i2c_get_clientdata(client);

	mm_segment_t old_fs = get_fs();

	set_fs(KERNEL_DS);

	memset(offset_x_src,0x00,sizeof(offset_x_src));
	memset(offset_y_src,0x00,sizeof(offset_y_src));
	memset(offset_z_src,0x00,sizeof(offset_z_src));

	fd_offset_x = sys_open("/sns/offset_x.dat",O_RDONLY, 0);
	fd_offset_y = sys_open("/sns/offset_y.dat",O_RDONLY, 0);
	fd_offset_z = sys_open("/sns/offset_z.dat",O_RDONLY, 0);

	if((fd_offset_x < 0) || (fd_offset_y < 0) || (fd_offset_z < 0))
		return -EINVAL;

	if((sys_read(fd_offset_x, offset_x_src, sizeof(offset_x_src)) < 0)
		||	(sys_read(fd_offset_y ,offset_y_src, sizeof(offset_y_src)) <0)
		||	(sys_read(fd_offset_z, offset_z_src, sizeof(offset_z_src)) <0 ))
		return -EINVAL;

	if((strict_strtol(offset_x_src, 10, &offset_x))
		||	(strict_strtol(offset_y_src, 10, &offset_y))
		||	(strict_strtol(offset_z_src, 10, &offset_z)))
		return -EINVAL;

	if(	offset_x > 1000 || offset_x < -1000
		|| offset_y > 1000 || offset_y < -1000
		|| offset_z > 1000 || offset_z < -1000 ){
		dev_err(&client->dev,"Abnormal Calibration Data");
		offset_x = 0;
		offset_y = 0;
		offset_z = 0;
	}

	k3dh->cal_data.x = offset_x;
	k3dh->cal_data.y = offset_y;
	k3dh->cal_data.z = offset_z;

	sys_close(fd_offset_x);
	sys_close(fd_offset_y);
	sys_close(fd_offset_z);
	set_fs(old_fs);

	return 0;
}

static int k3dh_store_Calibration_data(struct device *dev)
{
	unsigned char offset_x_src[5],offset_y_src[5],offset_z_src[5];
	int fd_offset_x, fd_offset_y, fd_offset_z;
	struct k3dh_acc_data *k3dh = dev_get_drvdata(dev);
	mm_segment_t old_fs = get_fs();

	set_fs(KERNEL_DS);

	memset(offset_x_src,0x00,sizeof(offset_x_src));
	memset(offset_y_src,0x00,sizeof(offset_y_src));
	memset(offset_z_src,0x00,sizeof(offset_z_src));

	fd_offset_x = sys_open("/sns/offset_x.dat",O_WRONLY|O_CREAT, 0666);
	fd_offset_y = sys_open("/sns/offset_y.dat",O_WRONLY|O_CREAT, 0666);
	fd_offset_z = sys_open("/sns/offset_z.dat",O_WRONLY|O_CREAT, 0666);

#ifdef DEBUG
	dev_info(dev, "[fd_offset_x] writing %d", fd_offset_x);
	dev_info(dev, "[fd_offset_y] writing %d", fd_offset_y );
	dev_info(dev, "[fd_offset_z] writing %d", fd_offset_z);
#endif

	if((fd_offset_x < 0) || (fd_offset_y < 0) || (fd_offset_z < 0))
		return -EINVAL;

	sprintf(offset_x_src, "%d", k3dh->cal_data.x);
	sprintf(offset_y_src, "%d", k3dh->cal_data.y);
	sprintf(offset_z_src, "%d", k3dh->cal_data.z);

#ifdef DEBUG
	dev_info(dev, "[offset_x] writing %s", offset_x_src);
	dev_info(dev, "[offset_y] writing %s", offset_y_src);
	dev_info(dev, "[offset_z] writing %s", offset_z_src);
#endif

	if((sys_write(fd_offset_x, offset_x_src, sizeof(offset_x_src)) < 0)
			||	(sys_write(fd_offset_y, offset_y_src,  sizeof(offset_y_src)) < 0)
				||	(sys_write(fd_offset_z, offset_z_src,  sizeof(offset_z_src)) < 0))
		return -EINVAL;

	atomic_set(&k3dh->fast_calib_rslt, 1);

	sys_fsync(fd_offset_x);
	sys_fsync(fd_offset_y);
	sys_fsync(fd_offset_z);

	sys_close(fd_offset_x);
	sys_close(fd_offset_y);
	sys_close(fd_offset_z);
        sys_chmod("/sns/offset_x.dat", 0664);
        sys_chmod("/sns/offset_y.dat", 0664);
        sys_chmod("/sns/offset_z.dat", 0664);
	set_fs(old_fs);

#ifdef DEBUG
	dev_info(dev, "[CLOSE FILE]!!!!! ");
#endif

	return 0;
}

#endif

static void k3dh_acc_report_values(struct k3dh_acc_data *acc, int *xyz)
{
	input_report_abs(acc->input_dev, ABS_X, xyz[0]);
	input_report_abs(acc->input_dev, ABS_Y, xyz[1]);
	input_report_abs(acc->input_dev, ABS_Z, xyz[2]);
	input_sync(acc->input_dev);
}

static int k3dh_acc_enable(struct k3dh_acc_data *acc)
{
	int err;
	pr_info("cal_data.bCalLoaded = %d",acc->cal_data.bCalLoaded);
	/* Let sensor know that calibration data needs to be loaded */
	if( !(acc->cal_data.bCalLoaded ) ) {
		err = k3dh_read_Calibration_data(acc->client);
		if(err)
			pr_info("k3dh Calibration data is not found");
		else
			acc->cal_data.bCalLoaded = 1;
	}
	if (!atomic_cmpxchg(&acc->enabled, 0, 1)) {
		err = k3dh_acc_device_power_on(acc);
		if (err < 0) {
			atomic_set(&acc->enabled, 0);
			return err;
		}
		schedule_delayed_work(&acc->input_work, msecs_to_jiffies(
				acc->pdata->poll_interval));
	}

	return 0;
}

static int k3dh_acc_disable(struct k3dh_acc_data *acc)
{
	if (atomic_cmpxchg(&acc->enabled, 1, 0)) {
		cancel_delayed_work_sync(&acc->input_work);
		k3dh_acc_device_power_off(acc);
	}

	return 0;
}

static int k3dh_acc_misc_open(struct inode *inode, struct file *file)
{
	int err;
	err = nonseekable_open(inode, file);
	if (err < 0)
		return err;

	file->private_data = k3dh_acc_misc_data;

	return 0;
}

#define SELF_TEST_2G_MAX_LSB	(360)
#define SELF_TEST_2G_MIN_LSB	(17)
#define TESTLIMIT_XY			(175)
#define TESTLIMIT_Z_USL_LSB		(1270)
#define TESTLIMIT_Z_LSL_LSB		(778)


static int k3dh_acc_get_selftest(struct k3dh_acc_data *stat, char *buf)
{
	int val, i, en_state = 0;

	u8 x[8] = {0, };
	s32 NO_ST[3] = {0, 0, 0};
	s32 NO_ST_ZOFF[3] = {0, 0, 0};
	s32 ST[3] = {0, 0, 0};
/*	s16 tmp = 0; */
	en_state = atomic_read(&stat->enabled);

	k3dh_acc_disable(stat);
	k3dh_acc_device_power_on(stat);

	x[0] = CTRL_REG1;
	x[1] = 0x47;
	k3dh_acc_i2c_write(stat, x, 1);

	x[0] = (I2C_AUTO_INCREMENT | CTRL_REG4);
	x[1] = 0x88;
	x[2] = 0x00;
	x[3] = 0x00;
	k3dh_acc_i2c_write(stat, x, 3);

	mdelay(80);

	x[0] = (I2C_AUTO_INCREMENT | AXISDATA_REG);
	k3dh_acc_i2c_read(stat, x, 6);

	for (i = 0; i < 5; i++) {
		while (1) {
			x[0] = 0x27; /*status_reg */
			val = k3dh_acc_i2c_read(stat, x, 1);
			if (val < 0) {
				pr_info("%s: I2C fail during self-test\n", __func__);
				goto ST_EXIT;
			}

			if (x[0] & 0x08)
				break;
		}

		x[0] = (I2C_AUTO_INCREMENT | AXISDATA_REG);
		k3dh_acc_i2c_read(stat, x, 6);

		/* 12 bit resolution 1LSB= 0.997mg */
		NO_ST_ZOFF[0] += (((s16) ((x[1] << 8) | x[0])) >> 4);
		NO_ST_ZOFF[1] += (((s16) ((x[3] << 8) | x[2])) >> 4);
		NO_ST_ZOFF[2] += (((s16) ((x[5] << 8) | x[4])) >> 4);

		/*10 bit resolution 1LSB=4mg */
		NO_ST[0] += (((s16) ((x[1] << 8) | x[0])) >> 6);
		NO_ST[1] += (((s16) ((x[3] << 8) | x[2])) >> 6);
		NO_ST[2] += (((s16) ((x[5] << 8) | x[4])) >> 6);


#if DEBUG
		pr_info("%s-NO_ST(%d): %d, %d, %d\n",
			 K3DH_ACC_DEV_NAME, i, NO_ST[0], NO_ST[1], NO_ST[2]);
#endif
	}

		NO_ST_ZOFF[0] /= 5;
		NO_ST_ZOFF[1] /= 5;
		NO_ST_ZOFF[2] /= 5;

		NO_ST[0] /= 5;
		NO_ST[1] /= 5;
		NO_ST[2] /= 5;

#if DEBUG
	pr_info("%s:AVE_NO_ST: %d, %d, %d\n", K3DH_ACC_DEV_NAME, NO_ST[0], NO_ST[1], NO_ST[2]);
#endif

	x[0] = CTRL_REG4;
	x[1] = 0x8A; /* ST enable */
	k3dh_acc_i2c_write(stat, x, 1);

	mdelay(80);

	x[0] = (I2C_AUTO_INCREMENT | AXISDATA_REG);
	k3dh_acc_i2c_read(stat, x, 6);

	for (i = 0; i < 5; i++) {
		while (1) {
			x[0] = 0x27;
			val = k3dh_acc_i2c_read(stat, x, 1);

			if (val < 0) {
				pr_info("%s: I2C fail during self-test\n", __func__);
				goto ST_EXIT;
			}

			if (x[0] & 0x08)
				break;
		}

		x[0] = (I2C_AUTO_INCREMENT | AXISDATA_REG);
		k3dh_acc_i2c_read(stat, x, 6);

		ST[0] += (((s16) ((x[1] << 8) | x[0])) >> 6);
		ST[1] += (((s16) ((x[3] << 8) | x[2])) >> 6);
		ST[2] += (((s16) ((x[5] << 8) | x[4])) >> 6);

#if DEBUG
		pr_info("%s-ST(%d): %d, %d, %d\n",
			K3DH_ACC_DEV_NAME, i, ST[0], ST[1], ST[2]);
#endif

	}
	ST[0] /= 5;
	ST[1] /= 5;
	ST[2] /= 5;


#if DEBUG
		pr_info("%s:AVE_ST: %d, %d, %d\n",
			K3DH_ACC_DEV_NAME, ST[0], ST[1], ST[2]);
#endif

	for (val = -1, i = 0; i < 3; i++) {
		ST[i] -= NO_ST[i];
		ST[i] = abs(ST[i]);

		if ((SELF_TEST_2G_MIN_LSB > ST[i]) || (ST[i] > SELF_TEST_2G_MAX_LSB)) {
			pr_info("%s: ST[%d]: Out of range!! (%d)\n",
				K3DH_ACC_DEV_NAME, i, ST[i]);
			val = 0;
		}
	}

	/* Check zero-g offset */
	if (val >= 0) {
		for (val = 1, i = 0; i < 3; i++) {
			if (i < 2) {
				if (abs(NO_ST_ZOFF[i]) > TESTLIMIT_XY) {
					pr_info("%s: NO_ST[%d]: Out of ZOffset!! (%d)\n",
						K3DH_ACC_DEV_NAME, i, NO_ST_ZOFF[i]);
					val = -1;
				}
			} else {
				if (NO_ST_ZOFF[i] > TESTLIMIT_Z_USL_LSB
					|| NO_ST_ZOFF[i] < TESTLIMIT_Z_LSL_LSB) {
					pr_info("%s: NO_ST[%d]: Out of ZOffset!! (%d)\n",
						K3DH_ACC_DEV_NAME, i, NO_ST[i]);
					val = -1;
				}
			}
		}
	}

#if DEBUG
	if (val) {
		pr_info("%s-%d: Self Test: OK!! (%d, %d, %d) ||| (%d, %d, %d)\n",
			K3DH_ACC_DEV_NAME, val, ST[0], ST[1], ST[2],
			 NO_ST_ZOFF[0], NO_ST_ZOFF[1], NO_ST_ZOFF[2]);
	} else {
		pr_info("%s-%d: Self Test: NG!! (%d, %d, %d) ||| (%d, %d, %d)\n",
			K3DH_ACC_DEV_NAME, val, ST[0], ST[1], ST[2],
			 NO_ST_ZOFF[0], NO_ST_ZOFF[1], NO_ST_ZOFF[2]);
	}
#endif

ST_EXIT:
	x[0] = CTRL_REG1;
	x[1] = 0x00;

	k3dh_acc_i2c_write(stat, x, 1);

	x[0] = CTRL_REG5;
	x[1] = 0x88;

	k3dh_acc_i2c_write(stat, x, 1);

	k3dh_acc_device_power_off(stat);

	if (en_state)
		k3dh_acc_enable(stat);

	return val;
}

static int k3dh_acc_get_mode(struct i2c_client *client, unsigned char *mode)
{
	int err = 0;
	unsigned char data[2] = { 0, };
	unsigned char LPen = 0;
	unsigned char HR = 0;

	struct k3dh_acc_data *k3dh = i2c_get_clientdata(client);

	err = k3dh_acc_register_read(k3dh, data, K3DH_LP_EN_REG);
	if( err < 0 )
		return err;

	LPen = (data[0]& K3DH_LP_EN_MSK)?1:0;
    err = k3dh_acc_register_read(k3dh, data, K3DH_HR_REG);
    if(err < 0)
	       return err;

    HR = (data[0] & K3DH_HR_MSK) ? 1 : 0;

    if(LPen == 1 && HR == 0) {
        *mode = K3DH_MODE_LOWPOWER;
    }
    else if(LPen == 0 && HR == 0) {
        *mode = K3DH_MODE_NORMAL;
    }
    else if(LPen == 0 && HR == 1) {
        *mode = K3DH_MODE_HIGH_RESOLUTION;
    }
    else {
        *mode = -1;
    }

	pr_info("k3dh_acc_get_mode, LPen : %d, HR : %d\n",LPen,HR);

	return err;
}

static int k3dh_acc_set_mode(struct i2c_client *client, unsigned char mode)
{
	int err = -1;
	unsigned char data[2] ={0, 0};
    unsigned char LPen = 0;
    unsigned char HR = 0;
	struct k3dh_acc_data *k3dh = i2c_get_clientdata(client);

    if((client == NULL) || (mode >= 3)) {
        return -1;
    }

    switch(mode) {
        case K3DH_MODE_LOWPOWER :
            LPen = 1;
            HR = 0;
            break;
        case K3DH_MODE_NORMAL:
            LPen = 0;
            HR =  0;
            break;
        case K3DH_MODE_HIGH_RESOLUTION :
            LPen = 0;
            HR = 1;
            break;
        default :
            break;
    }

    err = k3dh_acc_register_update(k3dh, (u8 *) data, K3DH_LP_EN_REG, K3DH_LP_EN_MSK, LPen << K3DH_LP_EN_POS);
    if(err < 0)
		return err;

	err = k3dh_acc_register_update(k3dh, (u8 *) data, K3DH_HR_REG, K3DH_HR_MSK, HR << K3DH_HR_POS);
	if(err < 0 )
		return err;

	pr_info("k3dh_set_mode, mode : %d, LPen : %d, HR : %d\n", mode, LPen, HR);
	return 0;
}

static long k3dh_acc_misc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	u8 buf[4];
	u8 mask;
	u8 reg_address;
	u8 bit_values;
	int err;
	int interval;
	u8 buf_error[50];

	struct k3dh_acc_data *acc = file->private_data;

	pr_info("%s: %s call with cmd 0x%x and arg 0x%x\n",
			K3DH_ACC_DEV_NAME, __func__, cmd, (unsigned int)arg);

	switch (cmd) {
	case K3DH_ACC_IOCTL_GET_SELFTEST:
		err = k3dh_acc_get_selftest(acc, buf_error);

		if (copy_to_user(argp, &err, sizeof(err)))
			return -EINVAL;

		break;

	case K3DH_ACC_IOCTL_GET_DELAY:
		interval = acc->pdata->poll_interval;
		if (copy_to_user(argp, &interval, sizeof(interval)))
			return -EFAULT;
		break;

	case K3DH_ACC_IOCTL_SET_DELAY:
		if (copy_from_user(&interval, argp, sizeof(interval)))
			return -EFAULT;
		if (interval < 0 || interval > 1000)
			return -EINVAL;

		acc->pdata->poll_interval = max(interval,
				acc->pdata->min_interval);
		err = k3dh_acc_update_odr(acc, acc->pdata->poll_interval);
		/* TODO: if update fails poll is still set */
		if (err < 0)
			return err;
		break;

	case K3DH_ACC_IOCTL_SET_ENABLE:
		if (copy_from_user(&interval, argp, sizeof(interval)))
			return -EFAULT;
		if (interval > 1)
			return -EINVAL;
		if (interval)
			err = k3dh_acc_enable(acc);
		else
			err = k3dh_acc_disable(acc);
		return err;
		break;

	case K3DH_ACC_IOCTL_GET_ENABLE:
		interval = atomic_read(&acc->enabled);
		if (copy_to_user(argp, &interval, sizeof(interval)))
			return -EINVAL;
		break;

	case K3DH_ACC_IOCTL_SET_G_RANGE:
		if (copy_from_user(buf, argp, 1))
			return -EFAULT;
		bit_values = buf[0];
		err = k3dh_acc_update_g_range(acc, bit_values);
		if (err < 0)
			return err;
		break;

#ifdef INTERRUPT_MANAGEMENT
	case K3DH_ACC_IOCTL_SET_CTRL_REG3:
		if (copy_from_user(buf, argp, 2))
			return -EFAULT;
		reg_address = CTRL_REG3;
		mask = buf[1];
		bit_values = buf[0];
		err = k3dh_acc_register_update(acc, (u8 *) arg, reg_address,
				mask, bit_values);
		if (err < 0)
			return err;
		acc->resume_state[RES_CTRL_REG3] = ((mask & bit_values) |
				(~mask & acc->resume_state[RES_CTRL_REG3]));
		break;

	case K3DH_ACC_IOCTL_SET_CTRL_REG6:
		if (copy_from_user(buf, argp, 2))
			return -EFAULT;
		reg_address = CTRL_REG6;
		mask = buf[1];
		bit_values = buf[0];
		err = k3dh_acc_register_update(acc, (u8 *) arg, reg_address,
				mask, bit_values);
		if (err < 0)
			return err;
		acc->resume_state[RES_CTRL_REG6] = ((mask & bit_values) |
				(~mask & acc->resume_state[RES_CTRL_REG6]));
		break;

	case K3DH_ACC_IOCTL_SET_DURATION1:
		if (copy_from_user(buf, argp, 1))
			return -EFAULT;
		reg_address = INT_DUR1;
		mask = 0x7F;
		bit_values = buf[0];
		err = k3dh_acc_register_update(acc, (u8 *) arg, reg_address,
				mask, bit_values);
		if (err < 0)
			return err;
		acc->resume_state[RES_INT_DUR1] = ((mask & bit_values) |
				(~mask & acc->resume_state[RES_INT_DUR1]));
		break;

	case K3DH_ACC_IOCTL_SET_THRESHOLD1:
		if (copy_from_user(buf, argp, 1))
			return -EFAULT;
		reg_address = INT_THS1;
		mask = 0x7F;
		bit_values = buf[0];
		err = k3dh_acc_register_update(acc, (u8 *) arg, reg_address,
				mask, bit_values);
		if (err < 0)
			return err;
		acc->resume_state[RES_INT_THS1] = ((mask & bit_values) |
				(~mask & acc->resume_state[RES_INT_THS1]));
		break;

	case K3DH_ACC_IOCTL_SET_CONFIG1:
		if (copy_from_user(buf, argp, 2))
			return -EFAULT;
		reg_address = INT_CFG1;
		mask = buf[1];
		bit_values = buf[0];
		err = k3dh_acc_register_update(acc, (u8 *) arg, reg_address,
				mask, bit_values);
		if (err < 0)
			return err;
		acc->resume_state[RES_INT_CFG1] = ((mask & bit_values) |
				(~mask & acc->resume_state[RES_INT_CFG1]));
		break;

	case K3DH_ACC_IOCTL_GET_SOURCE1:
		err = k3dh_acc_register_read(acc, buf, INT_SRC1);
		if (err < 0)
			return err;
#if DEBUG
		pr_info("INT1_SRC content: %d , 0x%x\n",
				buf[0], buf[0]);
#endif
		if (copy_to_user(argp, buf, 1))
			return -EINVAL;
		break;

	case K3DH_ACC_IOCTL_SET_DURATION2:
		if (copy_from_user(buf, argp, 1))
			return -EFAULT;
		reg_address = INT_DUR2;
		mask = 0x7F;
		bit_values = buf[0];
		err = k3dh_acc_register_update(acc, (u8 *) arg, reg_address,
				mask, bit_values);
		if (err < 0)
			return err;
		acc->resume_state[RES_INT_DUR2] = ((mask & bit_values) |
				(~mask & acc->resume_state[RES_INT_DUR2]));
		break;

	case K3DH_ACC_IOCTL_SET_THRESHOLD2:
		if (copy_from_user(buf, argp, 1))
			return -EFAULT;
		reg_address = INT_THS2;
		mask = 0x7F;
		bit_values = buf[0];
		err = k3dh_acc_register_update(acc, (u8 *) arg, reg_address,
				mask, bit_values);
		if (err < 0)
			return err;
		acc->resume_state[RES_INT_THS2] = ((mask & bit_values) |
				(~mask & acc->resume_state[RES_INT_THS2]));
		break;

	case K3DH_ACC_IOCTL_SET_CONFIG2:
		if (copy_from_user(buf, argp, 2))
			return -EFAULT;
		reg_address = INT_CFG2;
		mask = buf[1];
		bit_values = buf[0];
		err = k3dh_acc_register_update(acc, (u8 *) arg, reg_address,
				mask, bit_values);
		if (err < 0)
			return err;
		acc->resume_state[RES_INT_CFG2] = ((mask & bit_values) |
				(~mask & acc->resume_state[RES_INT_CFG2]));
		break;

	case K3DH_ACC_IOCTL_GET_SOURCE2:
		err = k3dh_acc_register_read(acc, buf, INT_SRC2);
		if (err < 0)
			return err;
#if DEBUG
		pr_info("INT2_SRC content: %d , 0x%x\n",
				buf[0], buf[0]);
#endif
		if (copy_to_user(argp, buf, 1))
			return -EINVAL;
		break;

	case K3DH_ACC_IOCTL_GET_TAP_SOURCE:
		err = k3dh_acc_register_read(acc, buf, TT_SRC);
		if (err < 0)
			return err;
#if DEBUG
		pr_info("TT_SRC content: %d , 0x%x\n",
				buf[0], buf[0]);
#endif
		if (copy_to_user(argp, buf, 1)) {
			dev_err(&acc->client->dev, "%s: %s error in copy_to_user\n",
					K3DH_ACC_DEV_NAME, __func__);
			return -EINVAL;
		}
		break;

	case K3DH_ACC_IOCTL_SET_TAP_CFG:
		if (copy_from_user(buf, argp, 2))
			return -EFAULT;
		reg_address = TT_CFG;
		mask = buf[1];
		bit_values = buf[0];
		err = k3dh_acc_register_update(acc, (u8 *) arg, reg_address,
				mask, bit_values);
		if (err < 0)
			return err;
		acc->resume_state[RES_TT_CFG] = ((mask & bit_values) |
				(~mask & acc->resume_state[RES_TT_CFG]));
		break;

	case K3DH_ACC_IOCTL_SET_TAP_TLIM:
		if (copy_from_user(buf, argp, 2))
			return -EFAULT;
		reg_address = TT_LIM;
		mask = buf[1];
		bit_values = buf[0];
		err = k3dh_acc_register_update(acc, (u8 *) arg, reg_address,
				mask, bit_values);
		if (err < 0)
			return err;
		acc->resume_state[RES_TT_LIM] = ((mask & bit_values) |
				(~mask & acc->resume_state[RES_TT_LIM]));
		break;

	case K3DH_ACC_IOCTL_SET_TAP_THS:
		if (copy_from_user(buf, argp, 2))
			return -EFAULT;
		reg_address = TT_THS;
		mask = buf[1];
		bit_values = buf[0];
		err = k3dh_acc_register_update(acc, (u8 *) arg, reg_address,
				mask, bit_values);
		if (err < 0)
			return err;
		acc->resume_state[RES_TT_THS] = ((mask & bit_values) |
				(~mask & acc->resume_state[RES_TT_THS]));
		break;

	case K3DH_ACC_IOCTL_SET_TAP_TLAT:
		if (copy_from_user(buf, argp, 2))
			return -EFAULT;
		reg_address = TT_TLAT;
		mask = buf[1];
		bit_values = buf[0];
		err = k3dh_acc_register_update(acc, (u8 *) arg, reg_address,
				mask, bit_values);
		if (err < 0)
			return err;
		acc->resume_state[RES_TT_TLAT] = ((mask & bit_values) |
				(~mask & acc->resume_state[RES_TT_TLAT]));
		break;

	case K3DH_ACC_IOCTL_SET_TAP_TW:
		if (copy_from_user(buf, argp, 2))
			return -EFAULT;
		reg_address = TT_TW;
		mask = buf[1];
		bit_values = buf[0];
		err = k3dh_acc_register_update(acc, (u8 *) arg, reg_address,
				mask, bit_values);
		if (err < 0)
			return err;
		acc->resume_state[RES_TT_TW] = ((mask & bit_values) |
				(~mask & acc->resume_state[RES_TT_TW]));
		break;

#endif /* INTERRUPT_MANAGEMENT */

	default:
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations k3dh_acc_misc_fops = {
		.owner = THIS_MODULE,
		.open = k3dh_acc_misc_open,
		.unlocked_ioctl = k3dh_acc_misc_ioctl,
};

static struct miscdevice k3dh_acc_misc_device = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = K3DH_ACC_DEV_NAME,
		.fops = &k3dh_acc_misc_fops,
};

static void k3dh_acc_input_work_func(struct work_struct *work)
{
	struct k3dh_acc_data *acc;

	int xyz[3] = { 0 };
	int err;

	acc = container_of((struct delayed_work *)work,
			struct k3dh_acc_data,	input_work);

	mutex_lock(&acc->lock);
	err = k3dh_acc_get_acceleration_data(acc, xyz);
	if (err < 0)
		dev_err(&acc->client->dev, "get_acceleration_data failed\n");
	else
		k3dh_acc_report_values(acc, xyz);

	schedule_delayed_work(&acc->input_work, msecs_to_jiffies(
			acc->pdata->poll_interval));
	mutex_unlock(&acc->lock);
}

#ifdef K3DH_OPEN_ENABLE
int k3dh_acc_input_open(struct input_dev *input)
{
	struct k3dh_acc_data *acc = input_get_drvdata(input);

	return k3dh_acc_enable(acc);
}

void k3dh_acc_input_close(struct input_dev *dev)
{
	struct k3dh_acc_data *acc = input_get_drvdata(dev);

	k3dh_acc_disable(acc);
}
#endif

static int k3dh_acc_validate_pdata(struct k3dh_acc_data *acc)
{
	acc->pdata->poll_interval = max(acc->pdata->poll_interval,
			acc->pdata->min_interval);

	if (acc->pdata->axis_map_x > 2 || acc->pdata->axis_map_y > 2
			|| acc->pdata->axis_map_z > 2) {
		dev_err(&acc->client->dev, "invalid axis_map value x:%u y:%u z%u\n",
			 acc->pdata->axis_map_x, acc->pdata->axis_map_y,
			 acc->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (acc->pdata->negate_x > 1 || acc->pdata->negate_y > 1
			|| acc->pdata->negate_z > 1) {
		dev_err(&acc->client->dev, "invalid negate value x:%u y:%u z:%u\n",
			 acc->pdata->negate_x, acc->pdata->negate_y,
			 acc->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (acc->pdata->poll_interval < acc->pdata->min_interval) {
		dev_err(&acc->client->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int k3dh_acc_input_init(struct k3dh_acc_data *acc)
{
	int err;
	INIT_DELAYED_WORK(&acc->input_work, k3dh_acc_input_work_func);
	acc->input_dev = input_allocate_device();
	if (!acc->input_dev) {
		err = -ENOMEM;
		dev_err(&acc->client->dev, "input device allocate failed\n");
		goto err0;
	}

#ifdef K3DH_ACC_OPEN_ENABLE
	acc->input_dev->open = k3dh_acc_input_open;
	acc->input_dev->close = k3dh_acc_input_close;
#endif

	input_set_drvdata(acc->input_dev, acc);

	set_bit(EV_ABS, acc->input_dev->evbit);
	/*	next is used for interruptA sources data if the case */
	set_bit(ABS_MISC, acc->input_dev->absbit);
	/*	next is used for interruptB sources data if the case */
	set_bit(ABS_WHEEL, acc->input_dev->absbit);

	input_set_abs_params(acc->input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(acc->input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(acc->input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);
	/*	next is used for interruptA sources data if the case */
	input_set_abs_params(acc->input_dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);
	/*	next is used for interruptB sources data if the case */
	input_set_abs_params(acc->input_dev, ABS_WHEEL, INT_MIN, INT_MAX, 0, 0);

	acc->input_dev->name = "accelerometer";
	acc->input_dev->uniq = "k3dh";

	err = input_register_device(acc->input_dev);
	if (err) {
		dev_err(&acc->client->dev,
				"unable to register input polled device %s %s\n",
				acc->input_dev->name,
				acc->input_dev->uniq);
		goto err1;
	}

	return 0;

err1:
	input_free_device(acc->input_dev);
err0:
	return err;
}
/* power on/off function*/
int k3dh_power(struct k3dh_acc_platform_data *pdata, int on)
{
	int ret;

	if (on) {
		ret = regulator_enable(pdata->vdd_supply);
		ret += regulator_enable(pdata->vcc_i2c_supply);
		pr_info("k3dh power on [status : %d ]\n", ret);
	} else {
		ret = regulator_disable(pdata->vdd_supply);
		ret += regulator_disable(pdata->vcc_i2c_supply);
		pr_info("k3dh power off [status : %d ]\n", ret);
	}
	return ret;
}

int k3dh_power_on(struct k3dh_acc_platform_data *pdata)
{
	int ret;
	ret =  k3dh_power(pdata, 1);
	return ret;
}

int k3dh_power_off(struct k3dh_acc_platform_data *pdata)
{
	int ret;
	ret =  k3dh_power(pdata, 0);
	return ret;
}

/* DTS parse function */
#ifdef CONFIG_OF
static int k3dh_parse_dt(struct device *dev, struct k3dh_acc_platform_data *pdata)
{
	int rc;
	int ret, err = 0;
	struct device_node *np = dev->of_node;

	struct sensor_dt_to_pdata_map *itr;
	struct sensor_dt_to_pdata_map map[] = {
	{"st,gpio-int",		&pdata->gpio_int1,		DT_REQUIRED,		DT_GPIO, 	0},
#ifdef USE_ACC_IRQ2
	{"st_gpio_int2",	&pdata->gpio_int2,		DT_REQUIRED,		DT_GPIO, 	0},
#endif
	{"g_range",			&pdata->g_range,		DT_SUGGESTED,		DT_U32,		0},
	{"axis_map_x",		&pdata->axis_map_x,		DT_SUGGESTED,		DT_U32,		0},
	{"axis_map_y",		&pdata->axis_map_y,		DT_SUGGESTED,		DT_U32,		0},
	{"axis_map_z",		&pdata->axis_map_z,		DT_SUGGESTED,		DT_U32,		0},
	{"negate_x",		&pdata->negate_x,		DT_SUGGESTED,		DT_U32,		0},
	{"negate_y",		&pdata->negate_y,		DT_SUGGESTED,		DT_U32,		0},
	{"negate_z",		&pdata->negate_z,		DT_SUGGESTED,		DT_U32,		0},
	{"poll_interval",	&pdata->poll_interval,	DT_SUGGESTED,		DT_U32,		0},
	{"min_interval",	&pdata->min_interval,	DT_SUGGESTED,		DT_U32,		0},
	{NULL,				NULL,				 	0, 					0, 	 		0},
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
			dev_err(dev,"%d is an unknown DT entry type\n",itr->type);
			ret = -EBADE;
		}

		dev_info(dev,"DT entry ret:%d name:%s val:%d\n",
					ret,itr->dt_name, *((int *)itr->ptr_data));

		if (ret) {
			*((int *)itr->ptr_data) = itr->default_val;

			if (itr->status < DT_OPTIONAL) {
				dev_err(dev,"Missing '%s' DT entry\n",itr->dt_name);
				/* cont on err to dump all missing entries */
				if (itr->status == DT_REQUIRED && !err)
					err = ret;
			}
		}
	}

	pdata->vdd_supply = regulator_get(dev, "stm,sensor_vdd");
	if (IS_ERR(pdata->vdd_supply)) {
		rc = PTR_ERR(pdata->vdd_supply);
		dev_err(dev, "Regulator get failed sensor_vdd-supply rc=%d\n", rc);
		return rc;
	}
	pdata->vcc_i2c_supply = regulator_get(dev, "stm,sensor_vcc_i2c");
	if (IS_ERR(pdata->vcc_i2c_supply)) {
		rc = PTR_ERR(pdata->vcc_i2c_supply);
		dev_err(dev, "Regulator get failed vcc_i2c_supply rc=%d\n", rc);
		return rc;
	}
	/* debug print disable */
	/*
	   dev_info(dev, "parse_dt data [gpio_int = %d]\n", pdata->gpio_int);
	 */

	return 0;
}
#else
static int k3dh_parse_dt(struct device *dev, struct k3dh_acc_platform_data *pdata)
{
	return -ENODEV;
}
#endif

#ifndef USE_DEFAULT_PINCTRL_NAME
static int k3dh_pinctrl_init(struct k3dh_acc_data *data)
{
	int retval;

	pr_info("%s start\n", __func__);

	/* Get pinctrl if target uses pinctrl */
	data->k3dh_pinctrl = devm_pinctrl_get(&(data->client->dev));
	if (IS_ERR_OR_NULL(data->k3dh_pinctrl)) {
		dev_dbg(&data->client->dev,
			"Target does not use pinctrl\n");
		retval = PTR_ERR(data->k3dh_pinctrl);
		data->k3dh_pinctrl = NULL;
		return retval;
	}

	data->gpio_state_active
		= pinctrl_lookup_state(data->k3dh_pinctrl, "k2dh_active");
	if (IS_ERR_OR_NULL(data->gpio_state_active)) {
		dev_dbg(&data->client->dev,
			"Can not get ts default pinstate\n");
		retval = PTR_ERR(data->gpio_state_active);
		data->k3dh_pinctrl = NULL;
		return retval;
	}

	data->gpio_state_suspend
		= pinctrl_lookup_state(data->k3dh_pinctrl, "k2dh_suspend");
	if (IS_ERR_OR_NULL(data->gpio_state_suspend)) {
		dev_dbg(&data->client->dev,
			"Can not get ts sleep pinstate\n");
		retval = PTR_ERR(data->gpio_state_suspend);
		data->k3dh_pinctrl = NULL;
		return retval;
	}

	return 0;
}

static int k3dh_pinctrl_select(struct k3dh_acc_data *data, bool on)
{
	struct pinctrl_state *pins_state;
	int ret;

	dev_err("k3dh_pinctrl_select call\n");
	pins_state = on ? data->gpio_state_active : data->gpio_state_suspend;
	if (!IS_ERR_OR_NULL(pins_state)) {
		ret = pinctrl_select_state(data->k3dh_pinctrl, pins_state);
		if (ret) {
			dev_err(&data->client->dev,
				"can not set %s pins\n",
				on ? "k3dh_active" : "k3dh_suspend");
			return ret;
		}
	} else
		dev_err(&data->client->dev,
			"not a valid '%s' pinstate\n",
				on ? "k3dh_active" : "k3dh_suspend");

	return 0;
}
#endif

static int k3dh_gpio_config(struct k3dh_acc_data *data)
{
	int ret;
	/* configure irq gpio */
	ret = gpio_request(data->pdata->gpio_int1, "k3dh_irq_gpio");
	if (ret) {
		dev_err(&data->client->dev,
				"unable to request gpio [%d]\n",
				data->pdata->gpio_int1);
	}
	ret = gpio_direction_input(data->pdata->gpio_int1);
	if (ret) {
		dev_err(&data->client->dev,
			"unable to set direction for gpio [%d]\n",
			 data->pdata->gpio_int1);
	}
	return ret;
}

static int k3dh_do_calibration(struct device *dev, char *axis)
{
	struct k3dh_acc_data *k3dh = dev_get_drvdata(dev);
	unsigned int timeout_shaking = 0;
	int acc_cal_pre[3] = { 0, };
	int acc_cal[3] = { 0, };
	int sum[3] = { 0, };

	k3dh_acc_get_acceleration_raw_data(k3dh, acc_cal_pre);
	do{
		mdelay(20);
		k3dh_acc_get_acceleration_raw_data(k3dh, acc_cal);
		dev_info(dev, "===============moved %s =============== timeout = %d", axis, timeout_shaking);
		dev_info(dev, "(%d, %d, %d) (%d, %d, %d)", acc_cal_pre[K3DH_AXIS_X], acc_cal_pre[K3DH_AXIS_Y], acc_cal_pre[K3DH_AXIS_Z], acc_cal[K3DH_AXIS_X], acc_cal[K3DH_AXIS_Y], acc_cal[K3DH_AXIS_Z]);

		if((abs(acc_cal[K3DH_AXIS_X] - acc_cal_pre[K3DH_AXIS_X]) > K3DH_SHAKING_DETECT_THRESHOLD)
			|| (abs((acc_cal[K3DH_AXIS_Y] - acc_cal_pre[K3DH_AXIS_Y])) > K3DH_SHAKING_DETECT_THRESHOLD)
			|| (abs((acc_cal[K3DH_AXIS_Z] - acc_cal_pre[K3DH_AXIS_Z])) > K3DH_SHAKING_DETECT_THRESHOLD)){
				atomic_set(&k3dh->fast_calib_rslt, 0);
				dev_info(dev, "===============shaking %s ===============", axis);
				return -EINVAL;
		}
		else{
			sum[K3DH_AXIS_X] += acc_cal[K3DH_AXIS_X];
			sum[K3DH_AXIS_Y] += acc_cal[K3DH_AXIS_Y];
			sum[K3DH_AXIS_Z] += acc_cal[K3DH_AXIS_Z];

			acc_cal_pre[K3DH_AXIS_X] = acc_cal[K3DH_AXIS_X];
			acc_cal_pre[K3DH_AXIS_Y] = acc_cal[K3DH_AXIS_Y];
			acc_cal_pre[K3DH_AXIS_Z] = acc_cal[K3DH_AXIS_Z];
		}
			timeout_shaking++;
	}while(timeout_shaking < CALIBRATION_DATA_AMOUNT);
	dev_info(dev, "===============complete shaking %s check===============", axis);

	// check zero-g offset
	if((abs(sum[K3DH_AXIS_X]/CALIBRATION_DATA_AMOUNT) >TESTLIMIT_XY) ||
		(abs(sum[K3DH_AXIS_Y]/CALIBRATION_DATA_AMOUNT) >TESTLIMIT_XY) ||
		 ((abs(sum[K3DH_AXIS_Z]/CALIBRATION_DATA_AMOUNT) > TESTLIMIT_Z_USL_LSB) || (abs(sum[K3DH_AXIS_Z]/CALIBRATION_DATA_AMOUNT) < TESTLIMIT_Z_LSL_LSB))) {
			dev_err(dev, "Calibration zero-g offset check failed (%d, %d, %d)\n",
					sum[K3DH_AXIS_X]/CALIBRATION_DATA_AMOUNT, sum[K3DH_AXIS_Y]/CALIBRATION_DATA_AMOUNT, sum[K3DH_AXIS_Z]/CALIBRATION_DATA_AMOUNT);
			atomic_set(&k3dh->fast_calib_rslt, 0);
			return -EINVAL;
	}

	k3dh->cal_data.x = sum[0] / CALIBRATION_DATA_AMOUNT;  //K3DH(12bit) 0+-154
	k3dh->cal_data.y = sum[1] / CALIBRATION_DATA_AMOUNT;  //K3DH(12bit) 0+-154

	if(sum[2] >= 0) {
		k3dh->cal_data.z = (sum[2] / CALIBRATION_DATA_AMOUNT) - 1024; //K3DH(12bit) 1024 +-226
	}
	else{
		k3dh->cal_data.z = (sum[2] / CALIBRATION_DATA_AMOUNT) + 1024; //K3DH(12bit) 1024 +-226
	}

	dev_info(dev, "=============== %s fast calibration finished===============", axis);

	return 0;
}

static ssize_t k3dh_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct k3dh_acc_data *k3dh = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", atomic_read(&k3dh->enabled));
}


static ssize_t k3dh_enable_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long onoff;
	int error;
    struct k3dh_acc_data *k3dh = dev_get_drvdata(dev);
	error = strict_strtoul(buf, 10, &onoff);
	if (error)
		return error;

	if (onoff == 0)
		k3dh_acc_disable(k3dh);
	else if (onoff == 1)
		k3dh_acc_enable(k3dh);

	return size;
}

/* See CTRL_REG1(20h) register ODR<3:0> data rate configuration table at datasheet*/
static ssize_t k3dh_delay_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char delay = 0;
	struct k3dh_acc_data *k3dh = dev_get_drvdata(dev);

	if( k3dh_acc_get_odr(k3dh, &delay) < 0)
		return sprintf(buf, "Read error\n");
	else
		return sprintf(buf,"%d\n",delay);
}


static ssize_t k3dh_delay_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long data = 0;
	int error;
	struct k3dh_acc_data *k3dh = dev_get_drvdata(dev);

	error = strict_strtoul(buf, 10, &data);
	if(error)
		return error;
	if(k3dh_acc_update_odr(k3dh,data) < 0)
		pr_info("invalid content '%s', length = %d\n",buf,size);

	return size;
}

static ssize_t k3dh_sensor_cal_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int xyz[3] = { 0, };
	int err = 0;
	struct k3dh_acc_data *k3dh = dev_get_drvdata(dev);

	err = k3dh_acc_get_acceleration_data(k3dh,xyz);
	if (err < 0)
		return sprintf(buf, "Read Calibrated Data failed");

	return sprintf(buf, "Read Calibrated Data x=%d y=%d z=%d \n", xyz[K3DH_AXIS_X], xyz[K3DH_AXIS_Y], xyz[K3DH_AXIS_Z]);
}

static ssize_t k3dh_sensor_raw_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int xyz[3] = { 0, };
	int err = 0;
	struct k3dh_acc_data *k3dh = dev_get_drvdata(dev);

	err = k3dh_acc_get_acceleration_raw_data(k3dh,xyz);
	if (err < 0)
		return sprintf(buf, "Read Calibrated Data failed");

	return sprintf(buf, "Read Raw Data x=%d y=%d z=%d \n", xyz[K3DH_AXIS_X], xyz[K3DH_AXIS_Y], xyz[K3DH_AXIS_Z]);
}

static ssize_t k3dh_opmode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;
	struct k3dh_acc_data *k3dh = dev_get_drvdata(dev);

	if(k3dh_acc_get_mode(k3dh->client, &data) < 0)
		return sprintf(buf,"Read error\n");
	else
		{
			if(data == 0)
				return sprintf(buf, "0:LP MODE\n");
			else if(data == 1)
				return sprintf(buf, "1:Normal MODE\n");
			else if(data == 2)
				return sprintf(buf, "2:HR MODE\n");
			else
				return sprintf(buf, "Error\n");
		}
}

static ssize_t k3dh_opmode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long data = 0;
	int error = -1;
	struct k3dh_acc_data *k3dh = dev_get_drvdata(dev);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;
	if( k3dh_acc_set_mode(k3dh->client, (unsigned char) data) < 0)
		pr_info("invalid content '%s', length = %d\n",buf,size);

	return size;
}

static ssize_t k3dh_selftest_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct k3dh_acc_data *k3dh = dev_get_drvdata(dev);

	return sprintf(buf,"%s\n",atomic_read(&k3dh->selftest_rslt) ? "Pass" : "Fail");
}

static ssize_t k3dh_selftest_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned long data = 0;
	int error = -1;
	u8 buf_error[50];
	struct k3dh_acc_data *k3dh = dev_get_drvdata(dev);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if(data == 1) {
		if(k3dh_acc_get_selftest(k3dh,buf_error))
			atomic_set(&k3dh->selftest_rslt, 1);
		else
			atomic_set(&k3dh->selftest_rslt, 0);
	}
	else {
			pr_info("Selftest is failed\n");
			return -EINVAL;
	}

	return size;
}

static ssize_t k3dh_fast_calibration_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct k3dh_acc_data *k3dh = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", atomic_read(&k3dh->fast_calib_rslt));
}

static ssize_t k3dh_fast_calibration_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct k3dh_acc_data *k3dh = dev_get_drvdata(dev);
	unsigned int timeout_shaking = 0;
	unsigned long data;
	int acc_cal_pre[3] = { 0, };
	int acc_cal[3] = { 0, };
	int sum[3] = { 0, };
	int error = 0;

	atomic_set(&k3dh->fast_calib_rslt, 0);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	k3dh_acc_get_acceleration_raw_data(k3dh, acc_cal_pre);
	do{
		mdelay(20);
		k3dh_acc_get_acceleration_raw_data(k3dh, acc_cal);
		dev_info(dev, "===============moved x,y,z=============== timeout = %d",timeout_shaking);
		dev_info(dev, "(%d, %d, %d) (%d, %d, %d)", acc_cal_pre[K3DH_AXIS_X], acc_cal_pre[K3DH_AXIS_Y], acc_cal_pre[K3DH_AXIS_Z], acc_cal[K3DH_AXIS_X], acc_cal[K3DH_AXIS_Y], acc_cal[K3DH_AXIS_Z]);

		if((abs(acc_cal[K3DH_AXIS_X] - acc_cal_pre[K3DH_AXIS_X]) > K3DH_SHAKING_DETECT_THRESHOLD)
			|| (abs((acc_cal[K3DH_AXIS_Y] - acc_cal_pre[K3DH_AXIS_Y])) > K3DH_SHAKING_DETECT_THRESHOLD)
			|| (abs((acc_cal[K3DH_AXIS_Z] - acc_cal_pre[K3DH_AXIS_Z])) > K3DH_SHAKING_DETECT_THRESHOLD)){
				atomic_set(&k3dh->fast_calib_rslt, 0);
				dev_info(dev, "===============shaking x,y,z===============");
				return -EINVAL;
		}
		else{
			sum[K3DH_AXIS_X] += acc_cal[K3DH_AXIS_X];
			sum[K3DH_AXIS_Y] += acc_cal[K3DH_AXIS_Y];
			sum[K3DH_AXIS_Z] += acc_cal[K3DH_AXIS_Z];

			acc_cal_pre[K3DH_AXIS_X] = acc_cal[K3DH_AXIS_X];
			acc_cal_pre[K3DH_AXIS_Y] = acc_cal[K3DH_AXIS_Y];
			acc_cal_pre[K3DH_AXIS_Z] = acc_cal[K3DH_AXIS_Z];
		}
			timeout_shaking++;
	}while(timeout_shaking < CALIBRATION_DATA_AMOUNT);
	dev_info(dev, "===============complete shaking x,y,z check===============");

	// check zero-g offset
	if((abs(sum[K3DH_AXIS_X]/CALIBRATION_DATA_AMOUNT) >TESTLIMIT_XY) ||
		(abs(sum[K3DH_AXIS_Y]/CALIBRATION_DATA_AMOUNT) >TESTLIMIT_XY) ||
		 ((abs(sum[K3DH_AXIS_Z]/CALIBRATION_DATA_AMOUNT) > TESTLIMIT_Z_USL_LSB) || (abs(sum[K3DH_AXIS_Z]/CALIBRATION_DATA_AMOUNT) < TESTLIMIT_Z_LSL_LSB))) {
			dev_err(dev, "Calibration zero-g offset check failed (%d, %d, %d)\n",
					sum[K3DH_AXIS_X]/CALIBRATION_DATA_AMOUNT, sum[K3DH_AXIS_Y]/CALIBRATION_DATA_AMOUNT, sum[K3DH_AXIS_Z]/CALIBRATION_DATA_AMOUNT);
			atomic_set(&k3dh->fast_calib_rslt, 0);
			return -EINVAL;
	}

	k3dh->cal_data.x = sum[0] / CALIBRATION_DATA_AMOUNT;  //K3DH(12bit) 0+-154
	k3dh->cal_data.y = sum[1] / CALIBRATION_DATA_AMOUNT;  //K3DH(12bit) 0+-154

	if(sum[2] >= 0) {
		k3dh->cal_data.z = (sum[2] / CALIBRATION_DATA_AMOUNT) - 1024; //K3DH(12bit) 1024 +-226
	}
	else{
		k3dh->cal_data.z = (sum[2] / CALIBRATION_DATA_AMOUNT) + 1024; //K3DH(12bit) 1024 +-226
	}

	dev_info(dev, "===============x,y,z fast calibration finished===============");


	error = k3dh_store_Calibration_data(dev);
	if(error) {
		dev_err(dev,"k3dh_fast_calibration_store failed");
		return error;
	}
	atomic_set(&k3dh->fast_calib_rslt, 1);

	return size;

}

static ssize_t k3dh_eeprom_writing_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct k3dh_acc_data *k3dh = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", atomic_read(&k3dh->fast_calib_x_rslt));
}

static ssize_t k3dh_eeprom_writing_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int error = 0;
	error = k3dh_store_Calibration_data(dev);
	if(error)
		return error;

	return count;
}

static ssize_t k3dh_fast_calibration_x_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct k3dh_acc_data *k3dh = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", atomic_read(&k3dh->fast_calib_x_rslt));
}

static ssize_t k3dh_fast_calibration_x_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	int error = 0;
	char axis = 'x';
	struct k3dh_acc_data *k3dh = dev_get_drvdata(dev);

	atomic_set(&k3dh->fast_calib_rslt, 0);

	error = k3dh_do_calibration(dev,&axis);
	if(error){
		dev_err(dev,"[k3dh_fast_calibration_x_store] do_calibration failed");
		return error;
	}
	error = k3dh_store_Calibration_data(dev);
	if(error){
		dev_err(dev,"[k3dh_fast_calibration_x_store] store_Calibration_data failed");
		return error;
	}
	atomic_set(&k3dh->fast_calib_x_rslt, 1);
	return size;
}

static ssize_t k3dh_fast_calibration_y_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct k3dh_acc_data *k3dh = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", atomic_read(&k3dh->fast_calib_y_rslt));
}

static ssize_t k3dh_fast_calibration_y_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	int error = 0;
	char axis = 'y';
	struct k3dh_acc_data *k3dh = dev_get_drvdata(dev);

	atomic_set(&k3dh->fast_calib_y_rslt, 0);

	error = k3dh_do_calibration(dev,&axis);
	if(error){
		dev_err(dev,"[k3dh_fast_calibration_y_store] do_calibration failed");
		return error;
	}
	error = k3dh_store_Calibration_data(dev);
	if(error){
		dev_err(dev,"[k3dh_fast_calibration_y_store] store_Calibration_data failed");
		return error;
	}
	atomic_set(&k3dh->fast_calib_y_rslt, 1);
	return size;
}

static ssize_t k3dh_fast_calibration_z_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct k3dh_acc_data *k3dh = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", atomic_read(&k3dh->fast_calib_z_rslt));
}

static ssize_t k3dh_fast_calibration_z_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	int error = 0;
	char axis = 'z';
	struct k3dh_acc_data *k3dh = dev_get_drvdata(dev);

	atomic_set(&k3dh->fast_calib_z_rslt, 0);

	error = k3dh_do_calibration(dev,&axis);
	if(error){
		dev_err(dev,"[k3dh_fast_calibration_z_store] do_calibration failed");
		return error;
	}
	error = k3dh_store_Calibration_data(dev);
	if(error){
		dev_err(dev,"[k3dh_fast_calibration_z_store] store_Calibration_data failed");
		return error;
	}
	atomic_set(&k3dh->fast_calib_z_rslt, 1);
	return size;
}

static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR |S_IWGRP, k3dh_enable_show, k3dh_enable_store);
static DEVICE_ATTR(poll_delay, S_IRUGO|S_IWUSR |S_IWGRP, k3dh_delay_show, k3dh_delay_store);
static DEVICE_ATTR(value, S_IRUGO|S_IWUSR|S_IWGRP, k3dh_sensor_cal_data_show, NULL);
static DEVICE_ATTR(raw, S_IRUGO|S_IWUSR|S_IWGRP, k3dh_sensor_raw_data_show, NULL);
static DEVICE_ATTR(opmode, S_IRUGO|S_IWUSR |S_IWGRP, k3dh_opmode_show, k3dh_opmode_store);
static DEVICE_ATTR(selftest, S_IRUGO|S_IWUSR |S_IWGRP, k3dh_selftest_show, k3dh_selftest_store);
static DEVICE_ATTR(run_fast_calibration, S_IRUGO|S_IWUSR|S_IWGRP,	k3dh_fast_calibration_show, k3dh_fast_calibration_store);
static DEVICE_ATTR(run_calibration, S_IRUGO|S_IWUSR|S_IWGRP,	k3dh_eeprom_writing_show, k3dh_eeprom_writing_store);
static DEVICE_ATTR(fast_calibration_x, S_IRUGO|S_IWUSR|S_IWGRP, k3dh_fast_calibration_x_show, k3dh_fast_calibration_x_store);
static DEVICE_ATTR(fast_calibration_y, S_IRUGO|S_IWUSR|S_IWGRP, k3dh_fast_calibration_y_show, k3dh_fast_calibration_y_store);
static DEVICE_ATTR(fast_calibration_z, S_IRUGO|S_IWUSR|S_IWGRP, k3dh_fast_calibration_z_show, k3dh_fast_calibration_z_store);

static struct attribute *acc_sysfs_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_poll_delay.attr,
	&dev_attr_value.attr,
	&dev_attr_raw.attr,
	&dev_attr_opmode.attr,
	&dev_attr_selftest.attr,
#ifdef K3DH_ACCEL_CALIBRATION
	&dev_attr_run_fast_calibration.attr,
	&dev_attr_run_calibration.attr,
	&dev_attr_fast_calibration_x.attr,
	&dev_attr_fast_calibration_y.attr,
	&dev_attr_fast_calibration_z.attr,
	//&dev_attr_diag.attr,
	//&dev_attr_cnt.attr,
	//&dev_attr_cal_result.attr, */
#endif
	NULL
};

static struct attribute_group acc_attribute_group = {
	.attrs = acc_sysfs_attrs,
};

static void k3dh_acc_input_cleanup(struct k3dh_acc_data *acc)
{
	input_unregister_device(acc->input_dev);
	input_free_device(acc->input_dev);
}

static int k3dh_acc_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{

	struct k3dh_acc_data *acc;
	struct k3dh_acc_platform_data *pdata;

	int err = -1;
	int tempvalue;

	pr_info("%s: probe start.\n", K3DH_ACC_DEV_NAME);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	if (!i2c_check_functionality(client->adapter,
					I2C_FUNC_SMBUS_BYTE |
					I2C_FUNC_SMBUS_BYTE_DATA |
					I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(&client->dev, "client not smb-i2c capable:2\n");
		err = -EIO;
		goto exit_check_functionality_failed;
	}


	if (!i2c_check_functionality(client->adapter,
						I2C_FUNC_SMBUS_I2C_BLOCK)){
		dev_err(&client->dev, "client not smb-i2c capable:3\n");
		err = -EIO;
		goto exit_check_functionality_failed;
	}

	/*
	 * OK. From now, we presume we have a valid client. We now create the
	 * client structure, even though we cannot fill it completely yet.
	 */

	acc = devm_kzalloc(&client->dev, sizeof(struct k3dh_acc_data), GFP_KERNEL);
	if (acc == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
			"failed to allocate memory for module data: %d\n", err);
		goto exit_alloc_data_failed;
	}

	pdata = devm_kzalloc(&client->dev, sizeof(struct k3dh_acc_platform_data), GFP_KERNEL);

	if (pdata == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
			"failed to allocate memory for platform data:%d\n", err);
		goto exit_alloc_data_failed;
	}
	mutex_init(&acc->lock);
	mutex_lock(&acc->lock);

	acc->client = client;
	acc->pdata = pdata;
	i2c_set_clientdata(client, acc);

	if (client->dev.of_node) {
		/* device tree type */
		err = k3dh_parse_dt(&client->dev, pdata);
		if (err)
			return err;
	} else {
		/* platform data type */
		if (client->dev.platform_data == NULL)
			pr_info("using platform_data : pdata = NULL\n");
		else
			pr_info("using platform_data\n");
	}
	/* Binding power in,off functions */
	pdata->power_on = k3dh_power_on;
	pdata->power_off = k3dh_power_off;

#ifndef USE_DEFAULT_PINCTRL_NAME
	err = k3dh_pinctrl_init(acc);
	if (err)
		pr_info("k3dh_pinctrl_init ERROR  = %d\n", err);

	if (!err && acc->k3dh_pinctrl) {
		err = k3dh_pinctrl_select(acc, true);
		if (err < 0) {
			pr_info("k3dh_pinctrl_select ERROR  = %d\n", err);
			goto exit_alloc_data_failed;
		}
	}
#endif

	err = k3dh_gpio_config(acc);
	if (err)
		pr_info("k3dh_gpio_config ERROR  = %d\n", err);

	INIT_WORK(&acc->irq1_work, k3dh_acc_irq1_work_func);
	acc->irq1_work_queue = create_singlethread_workqueue("k3dh_acc_wq1");
	if (!acc->irq1_work_queue) {
		err = -ENOMEM;
		dev_err(&client->dev, "cannot create work queue1: %d\n", err);
		goto err_mutexunlock;
	}
#ifdef USE_ACC_IRQ2
	INIT_WORK(&acc->irq2_work, k3dh_acc_irq2_work_func);
	acc->irq2_work_queue = create_singlethread_workqueue("k3dh_acc_wq2");
	if (!acc->irq2_work_queue) {
		err = -ENOMEM;
		dev_err(&client->dev, "cannot create work queue2: %d\n", err);
		goto err_destoyworkqueue1;
	}
#endif
	if (i2c_smbus_read_byte(client) < 0) {
		dev_err(&acc->client->dev, "i2c_smbus_read_byte error!!\n");
		goto err_destoyworkqueue2;
	} else {
		pr_info("%s Device detected!\n", K3DH_ACC_DEV_NAME);
	}

	/* read chip id */

	tempvalue = i2c_smbus_read_word_data(client, WHO_AM_I);
	if ((tempvalue & 0x00FF) == WHOAMI_K3DH_ACC) {
		pr_info("%s I2C driver registered!\n",
							K3DH_ACC_DEV_NAME);
	} else {
		acc->client = NULL;
		pr_info("I2C driver not registered! Device unknown\n");
		goto err_destoyworkqueue2;
	}

	err = k3dh_acc_validate_pdata(acc);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto exit_alloc_data_failed;
	}

	i2c_set_clientdata(client, acc);

	if (acc->pdata->init != NULL) {
		err = acc->pdata->init();
		if (err < 0) {
			dev_err(&client->dev, "init failed: %d\n", err);
			goto err2;
		}
	}

	memset(acc->resume_state, 0, ARRAY_SIZE(acc->resume_state));

	if (acc->pdata->gpio_int1 >= 0) {

		acc->irq1 = gpio_to_irq(acc->pdata->gpio_int1);
		pr_info("%s: %s has set irq1 to irq: %d mapped on gpio:%d\n",
			K3DH_ACC_DEV_NAME, __func__, acc->irq1, acc->pdata->gpio_int1);
	}
/* lgp-s19 not used interrupt2 */
#ifdef USE_ACC_IRQ2
	if (acc->pdata->gpio_int2 >= 0) {

		acc->irq2 = gpio_to_irq(acc->pdata->gpio_int2);
		pr_info("%s: %s has set irq2 to irq: %d mapped on gpio:%d\n",
			K3DH_ACC_DEV_NAME, __func__, acc->irq2, acc->pdata->gpio_int2);

	}
#endif

	acc->resume_state[RES_CTRL_REG1] = K3DH_ACC_ENABLE_ALL_AXES;
	acc->resume_state[RES_CTRL_REG2] = 0x00;
	acc->resume_state[RES_CTRL_REG3] = 0x00;
	acc->resume_state[RES_CTRL_REG4] = 0x00;
	acc->resume_state[RES_CTRL_REG5] = 0x00;
	acc->resume_state[RES_CTRL_REG6] = 0x00;

	acc->resume_state[RES_TEMP_CFG_REG] = 0x00;
	acc->resume_state[RES_FIFO_CTRL_REG] = 0x00;
	acc->resume_state[RES_INT_CFG1] = 0x00;
	acc->resume_state[RES_INT_THS1] = 0x00;
	acc->resume_state[RES_INT_DUR1] = 0x00;
	acc->resume_state[RES_INT_CFG2] = 0x00;
	acc->resume_state[RES_INT_THS2] = 0x00;
	acc->resume_state[RES_INT_DUR2] = 0x00;

	acc->resume_state[RES_TT_CFG] = 0x00;
	acc->resume_state[RES_TT_THS] = 0x00;
	acc->resume_state[RES_TT_LIM] = 0x00;
	acc->resume_state[RES_TT_TLAT] = 0x00;
	acc->resume_state[RES_TT_TW] = 0x00;
#ifdef K3DH_ACCEL_CALIBRATION
	acc->cal_data.bCalLoaded = 0;
#endif
	err = k3dh_acc_device_power_on(acc);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err2;
	}

	atomic_set(&acc->enabled, 1);

	err = k3dh_acc_update_g_range(acc, acc->pdata->g_range);
	if (err < 0) {
		dev_err(&client->dev, "update_g_range failed\n");
		goto  err_power_off;
	}


	err = k3dh_acc_update_odr(acc, acc->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto  err_power_off;
	}

	err = k3dh_acc_input_init(acc);
	if (err < 0) {
		dev_err(&client->dev, "input init failed\n");
		goto err_power_off;
	}

	/* create sysfs */
	err = sysfs_create_group(&acc->input_dev->dev.kobj, &acc_attribute_group);
	if (err) {
		dev_err(&client->dev, "Creating attribute group failed\n");
		goto err_input_cleanup;
	}

	k3dh_acc_misc_data = acc;

	err = misc_register(&k3dh_acc_misc_device);
	if (err < 0) {
		dev_err(&client->dev,
				"misc K3DH_ACC_DEV_NAME register failed\n");
		goto err_input_cleanup;
	}

/* for test */
	k3dh_acc_device_power_off(acc);

	/* As default, do not report information */
	atomic_set(&acc->enabled, 0);

	if (acc->pdata->gpio_int1 >= 0) {

		err = request_irq(acc->irq1, k3dh_acc_isr1, IRQF_TRIGGER_RISING,
				"k3dh_acc_irq1", acc);
		if (err < 0) {
			dev_err(&client->dev, "request irq1 failed: %d\n", err);
			goto err_misc_dereg;
		}
		disable_irq_nosync(acc->irq1);
	}
/* lgp-s19 not used interrupt2 */
#ifdef USE_ACC_IRQ2
	if (acc->pdata->gpio_int2 >= 0) {
		err = request_irq(acc->irq2, k3dh_acc_isr2, IRQF_TRIGGER_RISING,
				"k3dh_acc_irq2", acc);
		if (err < 0) {
			dev_err(&client->dev, "request irq2 failed: %d\n", err);
			goto err_free_irq1;
		}
		disable_irq_nosync(acc->irq2);

	}
#endif
	mutex_unlock(&acc->lock);


	dev_info(&client->dev, "%s: probed\n", K3DH_ACC_DEV_NAME);

	return 0;
/* lgp-s19 not used interrupt2 and then dont't need
     err_free_irq1 label */
#ifdef USE_ACC_IRQ2
err_free_irq1:
	free_irq(acc->irq1, acc);
#endif
err_misc_dereg:
	misc_deregister(&k3dh_acc_misc_device);
err_input_cleanup:
	k3dh_acc_input_cleanup(acc);
err_power_off:
	k3dh_acc_device_power_off(acc);
err2:
	if (acc->pdata->exit)
		acc->pdata->exit();
err_destoyworkqueue2:
#ifdef USE_ACC_IRQ2
	destroy_workqueue(acc->irq2_work_queue);
err_destoyworkqueue1:
#endif
	destroy_workqueue(acc->irq1_work_queue);
err_mutexunlock:
	mutex_unlock(&acc->lock);
exit_alloc_data_failed:
exit_check_functionality_failed:
	//dev_err(&acc->client->dev, "%s: Driver Init failed\n", K3DH_ACC_DEV_NAME);
	return err;
}

static int k3dh_acc_remove(struct i2c_client *client)
{
	/* TODO: revisit ordering here once _probe order is finalized */
	struct k3dh_acc_data *acc = i2c_get_clientdata(client);

	free_irq(acc->irq1, acc);
#ifdef USE_ACC_IRQ2
	free_irq(acc->irq2, acc);
#endif
	gpio_free(acc->pdata->gpio_int1);
#ifdef USE_ACC_IRQ2
	gpio_free(acc->pdata->gpio_int2);
#endif
	destroy_workqueue(acc->irq1_work_queue);
#ifdef USE_ACC_IRQ2
	destroy_workqueue(acc->irq2_work_queue);
#endif
	misc_deregister(&k3dh_acc_misc_device);
	k3dh_acc_input_cleanup(acc);
	k3dh_acc_device_power_off(acc);
	if (acc->pdata->exit)
		acc->pdata->exit();

	return 0;
}

static int k3dh_acc_resume(struct i2c_client *client)
{
	struct k3dh_acc_data *acc = i2c_get_clientdata(client);
	pr_info("%s accelerometer driver: resume\n", K3DH_ACC_DEV_NAME);


	if (acc->on_before_suspend)
		return k3dh_acc_enable(acc);
	return 0;
}

static int k3dh_acc_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct k3dh_acc_data *acc = i2c_get_clientdata(client);
	pr_info("%s accelerometer driver: suspend\n", K3DH_ACC_DEV_NAME);

	acc->on_before_suspend = atomic_read(&acc->enabled);
	return k3dh_acc_disable(acc);
}

static const struct i2c_device_id k3dh_acc_id[]	= {
	{ K3DH_ACC_DEV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, k3dh_acc_id);

#ifdef CONFIG_OF
static struct of_device_id k2dh_match_table[] = {
	{ .compatible = "st,k2dh",},
	{ },
};
#else
#define k2dh_match_table NULL
#endif

static struct i2c_driver k3dh_acc_driver = {
	.driver = {
		.name = K3DH_ACC_DEV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = k2dh_match_table,
	},
	.probe = k3dh_acc_probe,
	.remove = k3dh_acc_remove,
	.resume = k3dh_acc_resume,
	.suspend = k3dh_acc_suspend,
	.id_table = k3dh_acc_id,
};

static int __init k3dh_acc_init(void)
{
	pr_info("%s accelerometer driver: init\n", K3DH_ACC_DEV_NAME);
	return i2c_add_driver(&k3dh_acc_driver);
}

static void __exit k3dh_acc_exit(void)
{
	#if DEBUG
	pr_info("%s accelerometer driver exit\n", K3DH_ACC_DEV_NAME);
	#endif
	i2c_del_driver(&k3dh_acc_driver);
	return;
}

module_init(k3dh_acc_init);
module_exit(k3dh_acc_exit);

MODULE_DESCRIPTION("k3dh accelerometer misc driver");
MODULE_AUTHOR("STMicroelectronics");
MODULE_LICENSE("GPL");
