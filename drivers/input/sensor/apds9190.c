/*
 *  apds9190.c - Linux kernel modules for proximity sensor
 *
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
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/* Change History
 *
 * 1.0.1	Functions apds9190_show_rev(), apds9190_show_id() and apds9190_show_status()
 *			have missing CMD_BYTE in the i2c_smbus_read_byte_data(). APDS-9190 needs
 *			CMD_BYTE for i2c write/read byte transaction.
 *
 * 1.0.2	Include PS switching threshold level when interrupt occurred
 *
 * 1.0.3	Implemented ISR and delay_work, correct PS threshold storing
 *
 * 1.0.4	Added Input Report Event
 *
 * 1.0.5	2012/09/27, eee3114.lee@lge.com(Sanghun.lee)
 * 		- calibration implemenation
 *
 * 1.0.6	2012/12/05, eee3114.lee@lge.com(Sanghun.lee)
 * 		- bug fix : calibratoin after set enable
 *
 * 1.0.7	2013/07/11, WX-BSP-TS@lge.com(Sanghun.lee)
 * 		- device tree implementation
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <mach/gpio.h>
#include <asm/uaccess.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <mach/vreg.h>
#include <linux/wakelock.h>
#include <mach/msm_i2ckbd.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/syscalls.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

#ifdef CONFIG_OF
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#endif

#define SENSOR_NAME		"proximity"
#define SENSOR_DEVICE		"apds9190"
#define APDS9190_DRV_NAME	"proximity_apds9190"
#define DRIVER_VERSION		"1.0.8"


#define INFO_PRINT	1
#define DEBUG_PRINT	1
#define ERROR_PRINT	1

#if INFO_PRINT > 0
#define INFO_MSG(fmt, args...) 		pr_info("[I %s] " fmt, SENSOR_DEVICE,\
						##args)
#else
#define INFO_MSG(fmt, args...)		do {} while(0)
#endif


#if DEBUG_PRINT > 0
#define DEBUG_MSG(fmt, args...) 	pr_info("[D %s %d] " fmt,\
						__func__, __LINE__, ##args)
#else
#define DEBUG_MSG(fmt, args...)		do {} while(0)
#endif

#if ERROR_PRINT > 0
#define ERR_MSG(fmt, args...) 		pr_err("[E %s %d] " fmt,\
						__func__, __LINE__, ##args)
#else
#define ERR_MSG(fmt, args...)     	do {} while(0)
#endif


#define APDS9190_PROXIMITY_CAL
#define PS_DEFAULT_CROSS_TALK 		100

#if defined(APDS9190_PROXIMITY_CAL)
#define CAL_MSG(fmt, args...) 		DEBUG_MSG(fmt, ##args)
#else
#define CAL_MSG(fmt, args...)     	do {} while(0)
#endif

/*
 * Defines
 */

#define APDS9190_ENABLE_REG		0x00
#define APDS9190_ATIME_REG		0x01 //ALS Non Use, Reserved
#define APDS9190_PTIME_REG		0x02
#define APDS9190_WTIME_REG		0x03
#define APDS9190_AILTL_REG		0x04 //ALS Non Use, Reserved
#define APDS9190_AILTH_REG		0x05 //ALS Non Use, Reserved
#define APDS9190_AIHTL_REG		0x06 //ALS Non Use, Reserved
#define APDS9190_AIHTH_REG		0x07 //ALS Non Use, Reserved
#define APDS9190_PILTL_REG		0x08
#define APDS9190_PILTH_REG		0x09
#define APDS9190_PIHTL_REG		0x0A
#define APDS9190_PIHTH_REG		0x0B
#define APDS9190_PERS_REG		0x0C
#define APDS9190_CONFIG_REG		0x0D

/*
 * Defines
 */

#define APDS9190_ENABLE_REG		0x00
#define APDS9190_ATIME_REG		0x01 //ALS Non Use, Reserved
#define APDS9190_PTIME_REG		0x02
#define APDS9190_WTIME_REG		0x03
#define APDS9190_AILTL_REG		0x04 //ALS Non Use, Reserved
#define APDS9190_AILTH_REG		0x05 //ALS Non Use, Reserved
#define APDS9190_AIHTL_REG		0x06 //ALS Non Use, Reserved
#define APDS9190_AIHTH_REG		0x07 //ALS Non Use, Reserved
#define APDS9190_PILTL_REG		0x08
#define APDS9190_PILTH_REG		0x09
#define APDS9190_PIHTL_REG		0x0A
#define APDS9190_PIHTH_REG		0x0B
#define APDS9190_PERS_REG		0x0C
#define APDS9190_CONFIG_REG		0x0D
#define APDS9190_PPCOUNT_REG		0x0E
#define APDS9190_CONTROL_REG		0x0F
#define APDS9190_REV_REG		0x11
#define APDS9190_ID_REG			0x12
#define APDS9190_STATUS_REG		0x13
#define APDS9190_CDATAL_REG		0x14 //ALS Non Use, Reserved
#define APDS9190_CDATAH_REG		0x15 //ALS Non Use, Reserved
#define APDS9190_IRDATAL_REG		0x16 //ALS Non Use, Reserved
#define APDS9190_IRDATAH_REG		0x17 //ALS Non Use, Reserved
#define APDS9190_PDATAL_REG		0x18
#define APDS9190_PDATAH_REG		0x19

#define CMD_BYTE			0x80
#define CMD_WORD			0xA0
#define CMD_SPECIAL			0xE0

#define CMD_CLR_PS_INT			0xE5
#define CMD_CLR_ALS_INT			0xE6
#define CMD_CLR_PS_ALS_INT		0xE7

#define APDS9190_ENABLE_PIEN 		0x20
#define APDS9190_ENABLE_AIEN 		0x10
#define APDS9190_ENABLE_WEN 		0x08
#define APDS9190_ENABLE_PEN 		0x04
//Proximity false Interrupt by strong ambient light
#define APDS9190_ENABLE_AEN 		0x02
#define APDS9190_ENABLE_PON 		0x01

//Proximity false Interrupt by strong ambient light
#define ATIME 	 			0xDB 	// 100.64ms . minimum ALS integration time //ALS Non Use
#define WTIME 				0xFF 	// 2.72 ms .
#define PTIME 	 			0xFF

#define PDIODE 			 	0x20 	// IR Diode
#define PGAIN 			 	0x00 	//1x Prox gain
#define AGAIN 				0x00 	//1x ALS gain, Reserved ALS Non Use

#define DEFAULT_APDS_PROXIMITY_HIGH_THRESHHOLD	600
#define DEFAULT_APDS_PROXIMITY_LOW_THRESHHOLD	550

#define DEFAULT_PPCOUNT 		8 		//prox pulse count
#define DEFAULT_PDRIVE 	 		0 		//100mA of LED Power

#define DEFAULT_CAL_MAX_THRESHOLD		770
#define DEFAULT_CAL_NEAR_THRESHOLD_OFFSET	250
#define DEFAULT_CAL_FAR_THRESHOLD_OFFSET	80

#define APDS9190_STATUS_PINT_AINT	0x30
#define APDS9190_STATUS_PINT		0x20
#define APDS9190_STATUS_AINT		0x10 // ALS Interrupt STATUS ALS Non Use

#define APDS9190_DELAYED_WORK

#define APDS9190_STATUS_RESUME		0
#define APDS9190_STATUS_SUSPEND		1
#define APDS9190_STATUS_QUEUE_WORK	2



/*
 * Structs
 */

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
	const char                  *dt_name;
	void                        *ptr_data;
	enum sensor_dt_entry_status status;
	enum sensor_dt_entry_type   type;
	int                          default_val;
};

struct apds9190_platform_data {
	int irq_num;
	unsigned int prox_int_low_threshold;
	unsigned int prox_int_high_threshold;
	unsigned int als_threshold_hsyteresis;
	unsigned int ppcount;

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
};


struct apds9190_data {
	struct i2c_client *client;
	struct mutex update_lock;
	struct input_dev *input_dev_ps;
#ifdef 	APDS9190_DELAYED_WORK
	struct delayed_work	dwork;
#else
	struct work_struct dwork;
#endif
	struct apds9190_platform_data *platform_data;

	unsigned int enable;
	unsigned int atime;
	unsigned int ptime;
	unsigned int wtime;
	unsigned int ailt;
	unsigned int aiht;
	unsigned int pilt;
	unsigned int piht;
	unsigned int pers;
	unsigned int config;
	unsigned int control;

/* control flag from HAL */
//	unsigned int enable_ps_sensor;

	unsigned int GA;
	unsigned int DF;
	unsigned int LPC;
	/* PS parameters */
	unsigned int ps_threshold;
	unsigned int ps_hysteresis_threshold; /* always lower than ps_threshold */
	unsigned int ps_detection;		/* 0 = near-to-far; 1 = far-to-near */
	int irq;

	unsigned int sw_mode;
	spinlock_t lock;

	int cross_talk;
	bool read_ps_cal_data;
	bool ps_cal_result;
	struct wake_lock wakelock;
	atomic_t status;
};

static int apds9190_read_crosstalk_data_fs(void);
static void apds9190_Set_PS_Threshold_Adding_Cross_talk(struct i2c_client *client, int cal_data);
/*
 * Global data
 */
enum apds9190_dev_status {
	PROX_STAT_SHUTDOWN = 0,
	PROX_STAT_OPERATING,
};

enum apds9190_input_event {
	PROX_INPUT_NEAR = 0,
	PROX_INPUT_FAR,
};

static struct workqueue_struct *proximity_wq = NULL;

/*
 * Management functions
 */
static int apds9190_init_client(struct i2c_client *client);
static int apds9190_disable(struct i2c_client *i2c_dev, pm_message_t state);
static int apds9190_enable(struct i2c_client *i2c_dev);


static int apds9190_set_command(struct i2c_client *client, int command)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret, i;
	int clearInt;

	if (command == 0) {
		clearInt = CMD_CLR_PS_INT;
		CAL_MSG("clear [CMD_CLR_PS_INT]\n");
	}
	else if (command == 1) {
		clearInt = CMD_CLR_ALS_INT;
	} else {
		clearInt = CMD_CLR_PS_ALS_INT;
		CAL_MSG("clear [CMD_CLR_PS_ALS_INT]\n");
	}

	mutex_lock(&data->update_lock);

	for (i = 0; i < 10; i++) {
		if((ret = i2c_smbus_write_byte(client, clearInt)) == 0)
			break;

		ERR_MSG("I2C Fail\n");
	}

	mutex_unlock(&data->update_lock);

	return ret;
}

static int apds9190_set_enable(struct i2c_client *client, int enable)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret = 0;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9190_ENABLE_REG, enable);
	mutex_unlock(&data->update_lock);
	CAL_MSG("apds9190_set_enable = [%x]\n", enable);
	data->enable = enable;

	return ret;
}

static int apds9190_set_atime(struct i2c_client *client, int atime)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret = 0;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9190_ATIME_REG, atime);
	mutex_unlock(&data->update_lock);

	data->atime = atime;

	return ret;
}

static int apds9190_set_ptime(struct i2c_client *client, int ptime)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret = 0;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9190_PTIME_REG, ptime);
	mutex_unlock(&data->update_lock);

	data->ptime = ptime;

	return ret;
}

static int apds9190_set_wtime(struct i2c_client *client, int wtime)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret = 0;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9190_WTIME_REG, wtime);
	mutex_unlock(&data->update_lock);

	data->wtime = wtime;

	return ret;
}
static int apds9190_set_ailt(struct i2c_client* client, int threshold)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret = 0;
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client, CMD_WORD|APDS9190_AILTL_REG, threshold);
	mutex_unlock(&data->update_lock);
	data->ailt = threshold;

		return ret;
}
static int apds9190_set_aiht(struct i2c_client* client, int threshold)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret = 0;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client, CMD_WORD|APDS9190_AIHTL_REG, threshold);
	mutex_unlock(&data->update_lock);
	data->aiht = threshold;

	return ret;
}

static int apds9190_set_pilt(struct i2c_client *client, int threshold)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret = 0;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client, CMD_WORD|APDS9190_PILTL_REG, threshold);
	mutex_unlock(&data->update_lock);

	data->pilt = threshold;

	return ret;
}

static int apds9190_set_piht(struct i2c_client *client, int threshold)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret = 0;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_word_data(client, CMD_WORD|APDS9190_PIHTL_REG, threshold);
	mutex_unlock(&data->update_lock);

	data->piht = threshold;

	return ret;
}

static int apds9190_set_pers(struct i2c_client *client, int pers)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret = 0;
	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9190_PERS_REG, pers);
	mutex_unlock(&data->update_lock);

	if (ret < 0) {
		INFO_MSG("pers Register Write Fail\n");
		return -1;
	}

	data->pers = pers;

	return ret;
}

static int apds9190_set_config(struct i2c_client *client, int config)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret = 0;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9190_CONFIG_REG, config);
	mutex_unlock(&data->update_lock);

	data->config = config;

	return ret;
}

static int apds9190_set_ppcount(struct i2c_client *client, int ppcount)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9190_PPCOUNT_REG, ppcount);
	mutex_unlock(&data->update_lock);

	data->platform_data->ppcount = ppcount;

	return ret;
}

static int apds9190_set_control(struct i2c_client *client, int control)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret;

	mutex_lock(&data->update_lock);
	ret = i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9190_CONTROL_REG, control);
	mutex_unlock(&data->update_lock);

	data->control = control;
	return ret;
}

void apds9190_swap(int *x, int *y)
{
     int temp = *x;
     *x = *y;
     *y = temp;
}

 static int apds9190_backup_crosstalk_data_fs(unsigned int val)
{
	int fd;
	int ret = 0;
	char buf[50];
	mm_segment_t old_fs = get_fs();

	memset(buf, 0, sizeof(buf));
	sprintf(buf, "%d", val);

	CAL_MSG("buf[%s]\n", buf);

	set_fs(KERNEL_DS);
	fd = sys_open("/sns/prox_calibration.dat",O_WRONLY|O_CREAT, 0664);
	if(fd >=0)
	{
		sys_write(fd, buf, sizeof(buf));
                sys_fsync(fd);
		sys_close(fd);
                sys_chmod("/sns/prox_calibration.dat", 0664);
		set_fs(old_fs);
	}
	else
	{
		ret++;
		sys_close(fd);
		set_fs(old_fs);
		return ret	;
	}

	return ret;
}

static int apds9190_read_crosstalk_data_fs(void)
{
	int fd;
	char read_buf[50];
	mm_segment_t old_fs = get_fs();

	memset(read_buf, 0, sizeof(read_buf));
	set_fs(KERNEL_DS);
	fd = sys_open("/sns/prox_calibration.dat",O_RDONLY, 0);

	if (fd < 0)
	{
		CAL_MSG("Prox Cross-talk get default value FS, fd[%d]\n", fd);
		sys_close(fd);
		set_fs(old_fs);
		return -1;
	}

	CAL_MSG("Success read Prox Cross-talk from FS\n");
	sys_read(fd, read_buf, sizeof(read_buf));
	sys_close(fd);
	set_fs(old_fs);

	return (simple_strtol(read_buf, NULL, 10));

}

static int apds9190_Run_Cross_talk_Calibration(struct i2c_client *client)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	unsigned int sum_of_pdata = 0, temp_pdata[20];
	unsigned int ret = 0, i = 0, j = 0, ArySize = 20, cal_check_flag = 0;
	unsigned int old_enable = data->enable;

	CAL_MSG("Enter\n");

RE_CALIBRATION:
	sum_of_pdata = 0;
	apds9190_set_enable(client, 0x0D);

	msleep(50);

	for (i = 0; i < 20; i++) {
		mutex_lock(&data->update_lock);
		temp_pdata[i] = i2c_smbus_read_word_data(client, CMD_WORD|APDS9190_PDATAL_REG);
		mutex_unlock(&data->update_lock);

		mdelay(6);
	}

	for (i = 0; i < ArySize - 1; i++)
		for (j = i + 1; j < ArySize; j++)
			if (temp_pdata[i] > temp_pdata[j])
				apds9190_swap(temp_pdata+i, temp_pdata+j);

	for (i = 5; i < 15; i++)
		sum_of_pdata = sum_of_pdata + temp_pdata[i];

	data->cross_talk = sum_of_pdata / 10;
	if (data->cross_talk > data->platform_data->crosstalk_max) {
		if (cal_check_flag == 0) {
			cal_check_flag = 1;
			goto RE_CALIBRATION;
		} else {

			apds9190_set_enable(client,0x00); /* calibration bug fix */
			apds9190_set_enable(client,old_enable);
			return -1;
		}
	}

	data->ps_threshold = data->platform_data->near_offset + data->cross_talk;
	data->ps_hysteresis_threshold = data->ps_threshold - data->platform_data->far_offset;

	ret = apds9190_backup_crosstalk_data_fs(data->cross_talk);

	CAL_MSG("threshold[%d], Hysteresis_threshold[%d]\n", data->ps_threshold, data->ps_hysteresis_threshold);

	apds9190_set_enable(client,0x00); /* calibration bug fix */
	apds9190_set_enable(client,old_enable);

	CAL_MSG("Leave\n");

	return data->cross_talk;
}

static void apds9190_Set_PS_Threshold_Adding_Cross_talk(struct i2c_client *client, int cal_data)
{
	struct apds9190_data *data = i2c_get_clientdata(client);

	if (cal_data > data->platform_data->crosstalk_max)
		cal_data = data->platform_data->crosstalk_max;
	if (cal_data<0)
		cal_data = 0;

	data->cross_talk = cal_data;
	data->ps_threshold = data->platform_data->near_offset + cal_data;
	data->ps_hysteresis_threshold = data->ps_threshold - data->platform_data->far_offset;

}
 static ssize_t apds9190_show_run_calibration(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct apds9190_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%x\n", data->ps_cal_result);
}

static ssize_t apds9190_store_run_calibration(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds9190_data *data = dev_get_drvdata(dev);
	int ret = apds9190_Run_Cross_talk_Calibration(data->client);
	if(ret < 0) {
		CAL_MSG("get default value : %d\n", ret);
		data->ps_cal_result = 0;
	} else {
		CAL_MSG("Succes cross-talk : %d\n", ret);
		data->ps_cal_result = 1;
	}

	return count;
}
 static ssize_t apds9190_show_crosstalk_data(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;

	ret = apds9190_read_crosstalk_data_fs();
	if(ret < 0)
		return sprintf(buf, "Read fail\n");

	return sprintf(buf, "%d\n", ret);
}

static ssize_t apds9190_store_crosstalk_data(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds9190_data *data = dev_get_drvdata(dev);

	int ret = 0;
	unsigned long val = simple_strtoul(buf, NULL, 10);

	CAL_MSG("\n");

	ret = apds9190_backup_crosstalk_data_fs(val);
	if(ret != 0)
		return printk("File open get default value %d\n", ret);

	data->cross_talk = val;
	CAL_MSG("Saved cross_talk val : %d\n", (int)val);

	return count;
}

void apds9190_change_ps_threshold(struct i2c_client *client)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int irdata = 0, ps_data = 0;

	apds9190_set_pers(client, 0x33);	// 29-Feb-2012 KK
													// repeat this because of the force first interrupt
	ps_data = i2c_smbus_read_word_data(client, CMD_WORD|APDS9190_PDATAL_REG);
	irdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS9190_IRDATAL_REG);

	CAL_MSG(">>>>>>>>>> apds9190_change_ps_threshold ps_data  = %d \n",ps_data);
	CAL_MSG(">>>>>>>>>> apds9190_change_ps_threshold data->piht = %d \n",data->piht);
	CAL_MSG(">>>>>>>>>> apds9190_change_ps_threshold data->pilt = %d \n",data->pilt);

	if((ps_data > data->pilt) && (ps_data >= data->piht) && (irdata != (100*(1024*(256-data->atime)))/100)) {
		/* far-to-NEAR */
		data->ps_detection = PROX_INPUT_NEAR;
		CAL_MSG("\t===> far-to-Near\n");
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, PROX_INPUT_NEAR);/* FAR-to-NEAR detection */
		input_sync(data->input_dev_ps);
		CAL_MSG(">>>>>>>>>> apds9190_change_ps_threshold 1 \n");
		CAL_MSG(">>>>>>>>>> apds9190_change_ps_threshold 1 ps_data= %d\n",ps_data);
		CAL_MSG(">>>>>>>>>> apds9190_change_ps_threshold 1 ps_threshold= %d\n",data->ps_threshold);
		apds9190_set_pilt(client, data->ps_hysteresis_threshold);
		apds9190_set_piht(client, 1023);
	} else if((ps_data < data->piht) && (ps_data <= data->pilt)) {
		/* near-to-FAR */
		data->ps_detection = PROX_INPUT_FAR;

		input_report_abs(data->input_dev_ps, ABS_DISTANCE, PROX_INPUT_FAR);/* NEAR-to-FAR detection */
		input_sync(data->input_dev_ps);

		apds9190_set_pilt(client, 0);
		apds9190_set_piht(client, data->ps_threshold);
		CAL_MSG("\t===> near-to-FAR\n");
	} else if ( (irdata == (100*(1024*(256-data->atime)))/100) && (data->ps_detection == PROX_INPUT_NEAR) ) {
		/* under strong ambient light condition*/
		/* near-to-FAR */
		data->ps_detection = PROX_INPUT_FAR;
		CAL_MSG(">>>>>>>>>> ps_threshold 3 \n");
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, PROX_INPUT_FAR);/* NEAR-to-FAR detection */
		input_sync(data->input_dev_ps);

		/*Keep the threshold NEAR condition to prevent the frequent Interrup under strong ambient light*/

		apds9190_set_pilt(client, data->ps_hysteresis_threshold);
		apds9190_set_piht(client, 1023);
		CAL_MSG("* Set PS Threshold NEAR condition to prevent the frequent Interrupt under strong ambient light\n");
		CAL_MSG("\t===> near-to-FAR\n\n");
	} else if ( (data->pilt == 1023) && (data->piht == 0) ) {
		/* this is the first near-to-far forced interrupt */
		data->ps_detection = PROX_INPUT_FAR;
		CAL_MSG(">>>>>>>>>> ps_threshold 4 \n");
		input_report_abs(data->input_dev_ps, ABS_DISTANCE, PROX_INPUT_FAR);/* NEAR-to-FAR detection */
		input_sync(data->input_dev_ps);

		apds9190_set_pilt(client, 0);
		apds9190_set_piht(client, data->ps_threshold);
		CAL_MSG("\tnear-to-FAR detected\n\n");
	}
}


/*
 * SysFS support
 */

static ssize_t
apds9190_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct apds9190_data *data = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", data->sw_mode);
}

static ssize_t apds9190_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds9190_data *pdev = dev_get_drvdata(dev);
	pm_message_t dummy_state;
	int mode;

	dummy_state.event = 0;

	sscanf(buf, "%d", &mode);

	if ((mode != PROX_STAT_SHUTDOWN) && (mode != PROX_STAT_OPERATING)) {
		CAL_MSG("Usage: echo [0 | 1] > enable");
		CAL_MSG(" 0: disable\n");
		CAL_MSG(" 1: enable\n");
		return count;
	}

	if (mode) {
		pdev->cross_talk = apds9190_read_crosstalk_data_fs();

		if(pdev->cross_talk <= 0 || pdev->cross_talk > pdev->platform_data->crosstalk_max)
			pdev->cross_talk = PS_DEFAULT_CROSS_TALK;
		CAL_MSG("%s Cross_talk : %d\n", __FUNCTION__, pdev->cross_talk);
		apds9190_Set_PS_Threshold_Adding_Cross_talk(pdev->client, pdev->cross_talk);

		CAL_MSG("%s apds9190_Set_PS_Threshold_Adding_Cross_talk\n", __FUNCTION__);
		CAL_MSG("%s apds9190_Set_PS_Threshold_Adding_Cross_talk = %d\n", __FUNCTION__,pdev->cross_talk);
	}


	if (mode == pdev->sw_mode) {
		CAL_MSG("mode is already %d\n", pdev->sw_mode);
		return count;
	} else {
		if (mode) {
			apds9190_enable(pdev->client);
			CAL_MSG("Power On Enable\n");
		} else {
			apds9190_disable(pdev->client, dummy_state);
			CAL_MSG("Power Off Disable\n");
		}
	}

	return count;
}

static ssize_t apds9190_show_ptime(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct apds9190_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n",data->ptime);
}

static ssize_t apds9190_store_ptime(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{

	struct apds9190_data *data = dev_get_drvdata(dev);
	unsigned long rdata = simple_strtoul(buf, NULL, 10);

	apds9190_set_ptime(data->client,rdata);

	return count;
}

static ssize_t apds9190_show_wtime(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct apds9190_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n",data->wtime);
}

static ssize_t apds9190_store_wtime(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds9190_data *data = dev_get_drvdata(dev);
	unsigned long rdata = simple_strtoul(buf, NULL, 10);

	apds9190_set_wtime(data->client,rdata);

	return count;
}

static ssize_t apds9190_show_ppcount(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct apds9190_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n",data->platform_data->ppcount);
}

static ssize_t apds9190_store_ppcount(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds9190_data *data = dev_get_drvdata(dev);
	int rdata;

	sscanf(buf, "%d", &rdata);
	apds9190_set_ppcount(data->client,rdata);

	return count;
}


static ssize_t apds9190_show_pdrive(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct apds9190_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n",data->platform_data->pdrive);
}


static ssize_t apds9190_store_pdrive(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds9190_data *data = dev_get_drvdata(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret = 0;

	ret = apds9190_set_control(data->client, val | PDIODE | PGAIN | AGAIN);
	if (ret < 0)
		return ret;

	data->platform_data->pdrive = val;
	return count;
}


static ssize_t apds9190_show_near_offset(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct apds9190_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n",data->platform_data->near_offset);
}


static ssize_t apds9190_store_near_offset(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds9190_data *data = dev_get_drvdata(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	data->platform_data->near_offset = val;
	return count;
}


static ssize_t apds9190_show_far_offset(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct apds9190_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n",data->platform_data->far_offset);
}


static ssize_t apds9190_store_far_offset(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds9190_data *data = dev_get_drvdata(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	data->platform_data->far_offset = val;
	return count;
}


static ssize_t apds9190_show_crosstalk_max(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct apds9190_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n",data->platform_data->crosstalk_max);
}


static ssize_t apds9190_store_crosstalk_max(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds9190_data *data = dev_get_drvdata(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	data->platform_data->crosstalk_max = val;
	return count;
}


static ssize_t apds9190_show_pers(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct apds9190_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n",data->pers);
}

static ssize_t apds9190_store_pers(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds9190_data *data = dev_get_drvdata(dev);
	unsigned long rdata = simple_strtoul(buf, NULL, 10);

	apds9190_set_pers(data->client,rdata);

	return count;
}

static ssize_t apds9190_show_control(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct apds9190_data *data = dev_get_drvdata(dev);

	int control = 0;

	control = i2c_smbus_read_byte_data(data->client, CMD_BYTE|APDS9190_CONTROL_REG);

	if (control < 0) {
		ERR_MSG("i2c error %d in reading reg 0x%x\n", control, CMD_BYTE|APDS9190_CONTROL_REG);
		return control;
	}

	return sprintf(buf, "reg(%x),buf(%x)\n", control,data->control);
}

static ssize_t apds9190_store_control(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds9190_data *data = dev_get_drvdata(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret = 0;

	ret = apds9190_set_control(data->client, val);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t apds9190_show_ps_threshold(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct apds9190_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", data->ps_threshold);
}

static ssize_t apds9190_store_ps_threshold(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t ps_th)
{
	struct apds9190_data *data = dev_get_drvdata(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	data->ps_threshold = val;

	return ps_th;
}


static ssize_t apds9190_show_ps_hysteresis_threshold(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct apds9190_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", data->ps_hysteresis_threshold);
}

static ssize_t apds9190_store_ps_hysteresis_threshold(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t ps_hy_th)
{
	struct apds9190_data *data = dev_get_drvdata(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	data->ps_hysteresis_threshold = val;

	return ps_hy_th;
}

static ssize_t apds9190_show_pdata(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct apds9190_data *data = dev_get_drvdata(dev);
	int pdata = i2c_smbus_read_word_data(data->client, CMD_WORD|APDS9190_PDATAL_REG);
	return sprintf(buf, "%d\n",pdata);
}



static ssize_t apds9190_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct apds9190_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n",data->ps_detection);
}

static DEVICE_ATTR(value, S_IRUGO | S_IWUSR, apds9190_status_show, NULL);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP, apds9190_enable_show, apds9190_enable_store);
static DEVICE_ATTR(ptime, S_IRUGO | S_IWUSR, apds9190_show_ptime, apds9190_store_ptime);
static DEVICE_ATTR(wtime, S_IRUGO | S_IWUSR, apds9190_show_wtime, apds9190_store_wtime);
static DEVICE_ATTR(ppcount, S_IRUGO | S_IWUSR, apds9190_show_ppcount, apds9190_store_ppcount);
static DEVICE_ATTR(pers, S_IRUGO | S_IWUSR, apds9190_show_pers, apds9190_store_pers);
static DEVICE_ATTR(pdata, S_IRUGO,  apds9190_show_pdata, NULL);
static DEVICE_ATTR(control,  S_IWUSR | S_IRUGO | S_IRGRP | S_IROTH , apds9190_show_control, apds9190_store_control);
static DEVICE_ATTR(ps_threshold,  S_IWUSR | S_IRUGO|S_IWGRP |S_IRGRP |S_IROTH/*|S_IWOTH*/,
		   apds9190_show_ps_threshold, apds9190_store_ps_threshold);
static DEVICE_ATTR(ps_hysteresis,  S_IWUSR | S_IRUGO|S_IWGRP |S_IRGRP |S_IROTH/*|S_IWOTH*/,
		   apds9190_show_ps_hysteresis_threshold, apds9190_store_ps_hysteresis_threshold);
static DEVICE_ATTR(run_calibration,  S_IWUSR | S_IRUGO|S_IWGRP |S_IRGRP |S_IROTH/*|S_IWOTH*/,
		   apds9190_show_run_calibration, apds9190_store_run_calibration);
static DEVICE_ATTR(prox_cal_data,  S_IWUSR | S_IRUGO|S_IWGRP |S_IRGRP |S_IROTH/*|S_IWOTH*/,
		   apds9190_show_crosstalk_data, apds9190_store_crosstalk_data);
static DEVICE_ATTR(pdrive, S_IRUGO | S_IWUSR, apds9190_show_pdrive, apds9190_store_pdrive);
static DEVICE_ATTR(near_offset, S_IRUGO | S_IWUSR, apds9190_show_near_offset, apds9190_store_near_offset);
static DEVICE_ATTR(far_offset, S_IRUGO | S_IWUSR, apds9190_show_far_offset, apds9190_store_far_offset);
static DEVICE_ATTR(crosstalk_max, S_IRUGO | S_IWUSR, apds9190_show_crosstalk_max, apds9190_store_crosstalk_max);



static struct attribute *apds9190_attributes[] = {
	&dev_attr_value.attr,
	&dev_attr_enable.attr,
	&dev_attr_ptime.attr,
	&dev_attr_wtime.attr,
	&dev_attr_ppcount.attr,
	&dev_attr_pers.attr,
	&dev_attr_pdata.attr,
	&dev_attr_control.attr,
	&dev_attr_ps_threshold.attr,
	&dev_attr_ps_hysteresis.attr,
	&dev_attr_run_calibration.attr,
	&dev_attr_prox_cal_data.attr,
	&dev_attr_pdrive.attr,
	&dev_attr_near_offset.attr,
	&dev_attr_far_offset.attr,
	&dev_attr_crosstalk_max.attr,
	NULL
};

static const struct attribute_group apds9190_attr_group = {
		.attrs = apds9190_attributes,
};

/*
 * Initialization function
 */
static int apds9190_init_client(struct i2c_client *client)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int err = 0;
	int id;

	err = apds9190_set_enable(client, 0);

	if (err < 0)
		return err;

	id = i2c_smbus_read_byte_data(client, CMD_BYTE|APDS9190_ID_REG);
	if (id == 0x29) {
		DEBUG_MSG("APDS-9190\n");
	} else {
		DEBUG_MSG("NOT APDS-9190\n");
		return -EIO;
	}

	err =  apds9190_set_atime(client, ATIME);
	if (err < 0) {
		DEBUG_MSG("atime set Fail\n");
		goto EXIT;
	}

	err = apds9190_set_ptime(client, PTIME);
	if (err < 0) {
		DEBUG_MSG("ptime set Fail\n");
		goto EXIT;
	}
	err = apds9190_set_wtime(client, WTIME);
	if (err < 0) {
		DEBUG_MSG("wtime set Faile\n");
		goto EXIT;
	}

	err = apds9190_set_ppcount(client, data->platform_data->ppcount);
	if (err < 0) {
		DEBUG_MSG("ppcount set Fail\n");
		goto EXIT;
	}
	err = apds9190_set_config(client, 0x00); // Wait long timer <- no needs so set 0
	if (err < 0) {
		DEBUG_MSG("config set Fail\n");
		goto EXIT;
	}
	err = apds9190_set_control(client, data->platform_data->pdrive | PDIODE | PGAIN | AGAIN);
	if (err < 0) {
		DEBUG_MSG("control set Fail\n");
		goto EXIT;
	}

	err = apds9190_set_pilt(client, 1023); // init threshold for proximity
	if (err < 0) {
		DEBUG_MSG("pilt set Fail\n");
		goto EXIT;
	}

	err = apds9190_set_piht(client, 0);
	if (err < 0) {
		DEBUG_MSG("piht set Fail\n");
		goto EXIT;
	}

	data->ps_detection = PROX_INPUT_FAR;			// we are forcing Near-to-Far interrupt, so this is defaulted to 1
	err = apds9190_set_ailt(client, 0);
	if (err < 0) {
		DEBUG_MSG("ailt set Fail\n");
		goto EXIT;
	}
	err = apds9190_set_aiht(client, (99*(1024*(256-data->atime)))/100);
	if (err < 0) {
		DEBUG_MSG("aiht set Fail\n");
		goto EXIT;
	}
	apds9190_set_pers(client, 0x03);	// 3 consecutive Interrupt persistence
						// 29-Feb-2012 KK, 30-Aug-2012 SH
						// Force PS interrupt every PS conversion cycle
						// instead of comparing threshold value
	if (err < 0) {
		DEBUG_MSG("pers set Fail\n");
		goto EXIT;
	}
	return 0;
EXIT:
	return err;
}

/* PS interrupt routine */
static void apds9190_work_handler(struct work_struct *work)
{
#ifdef APDS9190_DELAYED_WORK
struct apds9190_data *data =
	container_of(work, struct apds9190_data, dwork.work);
#else
	struct apds9190_data *data =
		container_of(work, struct apds9190_data, dwork);
#endif
	struct i2c_client *client = data->client;
	int status=0,cdata=0;
	int irdata=0;
#if defined(APDS900_SENSOR_DEBUG)
	int pdata=0;
	int ailt=0,aiht=0;
	int pilt=0,piht=0;
#endif
	int org_enable = data->enable;


	if(wake_lock_active(&data->wakelock))
		wake_unlock(&data->wakelock);

	wake_lock_timeout(&data->wakelock, 2 * HZ);

	status = i2c_smbus_read_byte_data(client, CMD_BYTE|APDS9190_STATUS_REG);
	i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9190_ENABLE_REG, 1);	/* disable 9190's ADC first */
	DEBUG_MSG("status = %x\n", status);

	if ((status & APDS9190_STATUS_PINT_AINT) == 0x30)
	{
		/* both ALS and PS are interrupted */
		disable_irq(data->irq);

		/* Change ALS threshold under the strong ambient light */
		cdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS9190_CDATAL_REG);

		//		printk(KERN_INFO "%s, [APDS9190_STATUS_PINT_AINT1] status : %d,   cdata : %d, isNear : %d\n",__func__, status, cdata, data->isNear);

		if (cdata == 100*(1024*(256-data->atime))/100) {
			//				pdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS9190_PDATAL_REG);
			//				printk(KERN_INFO "%s, [APDS9190_STATUS_PINT_AINT] cdata : %d, pdata : %d\n", __func__, cdata, pdata);

			//				printk(KERN_INFO "%s, [APDS9190_STATUS_PINT_AINT] status change Near to Far while Near status but couldn't recognize Far\n", __func__);
			//				printk(KERN_INFO "%s, [APDS9190_STATUS_PINT_AINT] Force status to change Far\n",__func__);

			apds9190_set_ailt(client, (99*(1024*(256-data->atime)))/100);
			apds9190_set_aiht(client, (100*(1024*(256-data->atime)))/100);
			DEBUG_MSG("* Set ALS Theshold under the strong sunlight\n");
		} else {
			apds9190_set_ailt(client, 0);
			apds9190_set_aiht(client, (99*(1024*(256-data->atime)))/100);
			DEBUG_MSG("* Set ALS Theshold for normal mode\n");
		}

		/* check if this is triggered by strong ambient light  */
		irdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS9190_IRDATAL_REG);
		if (irdata != (100*(1024*(256-data->atime)))/100) {
			apds9190_change_ps_threshold(client);
		} else {
			if (data->ps_detection == PROX_INPUT_NEAR) {
				apds9190_change_ps_threshold(client);
				DEBUG_MSG("* Triggered by background ambient noise\n\t==> near-to-FAR\n");
			} else {
				/*Keep the threshold NEAR condition to prevent the frequent Interrupt under strong ambient light*/
				apds9190_set_pilt(client, data->ps_hysteresis_threshold);
				apds9190_set_piht(client, 1023);
				DEBUG_MSG("* Set PS Threshold NEAR condition to prevent the frequent Interrupt under strong ambient light\n");
				DEBUG_MSG("* Triggered by background ambient noise\n");
				DEBUG_MSG("\n ==> maintain FAR \n");
			}
		}
		apds9190_set_command(client, 2);	/* 2 = CMD_CLR_PS_ALS_INT */
		enable_irq(data->irq);
	} else if((status & APDS9190_STATUS_PINT)  == 0x20) {
		/* only PS is interrupted */
		disable_irq(data->irq);

		/* Change ALS threshold under the strong ambient light */
		cdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS9190_CDATAL_REG);
		//			DEBUG_MSG(KERN_INFO "%s, [APDS9190_STATUS_PINT2] status : %d,   cdata : %d, isNear : %d\n",__func__, status, cdata, data->isNear);
		if (cdata == 100*(1024*(256-data->atime))/100) {
			apds9190_set_ailt(client, (99*(1024*(256-data->atime)))/100);
			apds9190_set_aiht(client, (100*(1024*(256-data->atime)))/100);
			DEBUG_MSG("* Set ALS Theshold under the strong sunlight\n");
		} else {
			apds9190_set_ailt(client, 0);
			apds9190_set_aiht(client, (99*(1024*(256-data->atime)))/100);
			DEBUG_MSG("* Set ALS Theshold for normal mode\n");
		}


		/* check if this is triggered by strong ambient light */
		irdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS9190_IRDATAL_REG);
		if (irdata != (100*(1024*(256-data->atime)))/100) {
			apds9190_change_ps_threshold(client);
		} else {
			if (data->ps_detection == PROX_INPUT_NEAR) {
				apds9190_change_ps_threshold(client);
				DEBUG_MSG("* Triggered by background ambient noise\n");
				DEBUG_MSG("\n ==> near-to-FAR\n");
			} else {
				/*Keep the threshold NEAR condition to prevent the frequent Interrupt under strong ambient light*/
				apds9190_set_pilt(client, data->ps_hysteresis_threshold);
				apds9190_set_piht(client, 1023);
				DEBUG_MSG("* Set PS Threshold NEAR condition to prevent the frequent Interrupt under strong ambient light\n");
				DEBUG_MSG("* Triggered by background ambient noise\n");
				DEBUG_MSG("\n ==> maintain FAR \n");
			}
		}
		apds9190_set_command(client, 0);	/* 0 = CMD_CLR_PS_INT */
		enable_irq(data->irq);
	} else if((status & APDS9190_STATUS_AINT) == 0x10) {
		/* only ALS is interrupted */

		/* Change ALS Threshold under the strong ambient light */
		cdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS9190_CDATAL_REG);
		if (cdata == 100*(1024*(256-data->atime))/100) {
			apds9190_set_ailt(client, (99*(1024*(256-data->atime)))/100);
			apds9190_set_aiht(client, (100*(1024*(256-data->atime)))/100);
			DEBUG_MSG("* Set ALS Theshold under the strong sunlight\n");
		} else {
			apds9190_set_ailt(client, 0);
			apds9190_set_aiht(client, (99*(1024*(256-data->atime)))/100);
			DEBUG_MSG("* Set ALS Theshold for normal mode\n");
		}

		/* check if this is triggered by the strong ambient light */
		irdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS9190_IRDATAL_REG);
		if (irdata != (100*(1024*(256-data->atime)))/100) {
			apds9190_change_ps_threshold(client);
		} else {
			if (data->ps_detection == PROX_INPUT_NEAR) {
				apds9190_change_ps_threshold(client);
				DEBUG_MSG("* Triggered by background ambient noise\n\t==> near-to-FAR\n");
			} else {
				/*Keep the threshold NEAR condition to prevent the frequent Interrupt under strong ambient light*/
				apds9190_set_pilt(client, data->ps_hysteresis_threshold);
				apds9190_set_piht(client, 1023);
				DEBUG_MSG("* Set PS Threshold NEAR condition to prevent the frequent Interrupt under strong ambient light\n");
				DEBUG_MSG("* Triggered by background ambient noise\n\t==> maintain FAR \n");
			}
		}
		apds9190_set_command(client, 1);	/* 1 = CMD_CLR_ALS_INT */
	}

#if defined(APDS900_SENSOR_DEBUG)
	pdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS9190_PDATAL_REG);
	cdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS9190_CDATAL_REG);
	irdata = i2c_smbus_read_word_data(client, CMD_WORD|APDS9190_IRDATAL_REG);
	ailt = i2c_smbus_read_word_data(client, CMD_WORD|APDS9190_AILTL_REG);
	aiht = i2c_smbus_read_word_data(client, CMD_WORD|APDS9190_AIHTL_REG);
	pilt = i2c_smbus_read_word_data(client, CMD_WORD|APDS9190_PILTL_REG);
	piht = i2c_smbus_read_word_data(client, CMD_WORD|APDS9190_PIHTL_REG);

	DEBUG_MSG("cdata = %d, irdata = %d, pdata = %d\n", cdata,irdata,pdata);
	DEBUG_MSG("ailt = %d, aiht = %d\n", ailt,aiht);
	DEBUG_MSG("pilt = %d, piht = %d\n", pilt,piht);

	DEBUG_MSG("--------------------\n");
#endif

	i2c_smbus_write_byte_data(client, CMD_BYTE|APDS9190_ENABLE_REG, org_enable);
}


static int apds_9190_add_workqueue(struct apds9190_data *data)
{
	int ret;

	DEBUG_MSG("enter\n");
	spin_lock(&data->lock);
#ifdef APDS9190_DELAYED_WORK
	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	__cancel_delayed_work(&data->dwork);
	ret = queue_delayed_work(proximity_wq, &data->dwork, msecs_to_jiffies(0));
#else
	ret = queue_work(proximity_wq, &data->dwork);
#endif
	spin_unlock(&data->lock);
	return ret;
}


static irqreturn_t apds_9190_irq_handler(int irq, void *dev_id)
{
	struct apds9190_data *data = dev_id;

	DEBUG_MSG("enter\n");

	if(atomic_read(&data->status) == APDS9190_STATUS_SUSPEND) {
		atomic_set(&data->status, APDS9190_STATUS_QUEUE_WORK);
	} else {
		if(apds_9190_add_workqueue(data) < 0){
			DEBUG_MSG("queue_work Error\n");
		}
	}
	return IRQ_HANDLED;
}

/*
 * I2C init/probing/exit functions
 */

static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0) ?
	       regulator_set_optimum_mode(reg, load_uA) : 0;
}

static int sensor_regulator_configure(struct apds9190_data *data, bool on)
{
	struct i2c_client *client = data->client;
	struct apds9190_platform_data *pdata = data->platform_data;
	int rc;

	if (on == false)
		goto hw_shutdown;

	pdata->vcc_ana = regulator_get(&client->dev, "Avago,vdd_ana");
	if (IS_ERR(pdata->vcc_ana)) {
		rc = PTR_ERR(pdata->vcc_ana);
		ERR_MSG("Regulator get failed vcc_ana rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(pdata->vcc_ana) > 0) {
		rc = regulator_set_voltage(pdata->vcc_ana, pdata->vdd_ana_supply_min,
					   pdata->vdd_ana_supply_max);
		if (rc) {
			ERR_MSG("regulator set_vtg failed rc=%d\n", rc);
			goto error_set_vtg_vcc_ana;
		}
	}
	if (pdata->digital_pwr_regulator) {
		pdata->vcc_dig = regulator_get(&client->dev, "Avago,vddio_dig");
		if (IS_ERR(pdata->vcc_dig)) {
			rc = PTR_ERR(pdata->vcc_dig);
			ERR_MSG("Regulator get dig failed rc=%d\n", rc);
			goto error_get_vtg_vcc_dig;
		}

		if (regulator_count_voltages(pdata->vcc_dig) > 0) {
			rc = regulator_set_voltage(pdata->vcc_dig,
						   pdata->vddio_dig_supply_min, pdata->vddio_dig_supply_max);
			if (rc) {
				ERR_MSG("regulator set_vtg failed rc=%d\n", rc);
				goto error_set_vtg_vcc_dig;
			}
		}
	}
	if (pdata->i2c_pull_up) {
		pdata->vcc_i2c = regulator_get(&client->dev, "Avago,vddio_i2c");
		if (IS_ERR(pdata->vcc_i2c)) {
			rc = PTR_ERR(pdata->vcc_i2c);
			ERR_MSG("Regulator get failed rc=%d\n", rc);
			goto error_get_vtg_i2c;
		}
		if (regulator_count_voltages(pdata->vcc_i2c) > 0) {
			rc = regulator_set_voltage(pdata->vcc_i2c,
						   pdata->vddio_i2c_supply_min, pdata->vddio_i2c_supply_max);
			if (rc) {
				ERR_MSG("regulator set_vtg failed rc=%d\n", rc);
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


static int sensor_regulator_power_on(struct apds9190_data *data, bool on)
{
	struct apds9190_platform_data *pdata = data->platform_data;
	int rc;

	if (on == false)
		goto power_off;

	rc = reg_set_optimum_mode_check(pdata->vcc_ana, pdata->vdd_ana_load_ua);
	if (rc < 0) {
		ERR_MSG("Regulator vcc_ana set_opt failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(pdata->vcc_ana);
	if (rc) {
		ERR_MSG("Regulator vcc_ana enable failed rc=%d\n", rc);
		goto error_reg_en_vcc_ana;
	}

	if (pdata->digital_pwr_regulator) {
		rc = reg_set_optimum_mode_check(pdata->vcc_dig,
						pdata->vddio_dig_load_ua);

		if (rc < 0) {
			ERR_MSG("Regulator vcc_dig set_opt failed rc=%d\n",
				rc);
			goto error_reg_opt_vcc_dig;
		}

		rc = regulator_enable(pdata->vcc_dig);
		if (rc) {
			ERR_MSG("Regulator vcc_dig enable failed rc=%d\n", rc);
			goto error_reg_en_vcc_dig;
		}
	}

	if (pdata->i2c_pull_up) {
		rc = reg_set_optimum_mode_check(pdata->vcc_i2c, pdata->vddio_i2c_load_ua);
		if (rc < 0) {
			ERR_MSG("Regulator vcc_i2c set_opt failed rc=%d\n", rc);
			goto error_reg_opt_i2c;
		}

		rc = regulator_enable(pdata->vcc_i2c);
		if (rc) {
			ERR_MSG("Regulator vcc_i2c enable failed rc=%d\n", rc);
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

static int sensor_platform_hw_power_on(struct apds9190_data *data, bool on)
{
	sensor_regulator_power_on(data, on);
	return 0;
}
static int sensor_platform_hw_init(struct apds9190_data *data)
{
	struct i2c_client *client = data->client;
	int error;

	error = sensor_regulator_configure(data, true);

	if (gpio_is_valid(data->platform_data->irq_gpio)) {
		/* configure touchscreen irq gpio */
		error = gpio_request(data->platform_data->irq_gpio, "apds9130_irq_gpio");
		if (error) {
			ERR_MSG("unable to request gpio [%d]\n",
				data->platform_data->irq_gpio);
		}
		error = gpio_direction_input(data->platform_data->irq_gpio);
		if (error) {
			ERR_MSG("unable to set direction for gpio [%d]\n",
				data->platform_data->irq_gpio);
		}
		data->irq = client->irq = gpio_to_irq(data->platform_data->irq_gpio);
	} else {
		ERR_MSG("irq gpio not provided\n");
	}
	return 0;
}

static void sensor_platform_hw_exit(struct apds9190_data *data)
{
	sensor_regulator_configure(data, false);

	if (gpio_is_valid(data->platform_data->irq_gpio))
		gpio_free(data->platform_data->irq_gpio);
}

static int sensor_parse_dt(struct device *dev, struct apds9190_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	int ret, err =0;
	struct sensor_dt_to_pdata_map *itr;
	struct sensor_dt_to_pdata_map map[] = {
	{"Avago,i2c-pull-up",		&pdata->i2c_pull_up,		DT_REQUIRED,	DT_BOOL,	0},
	{"Avago,dig-reg-support",	&pdata->digital_pwr_regulator,	DT_REQUIRED,	DT_BOOL,	0},
	{"Avago,irq-gpio",		&pdata->irq_gpio,		DT_REQUIRED,	DT_GPIO,	0},
	{"Avago,vdd_ana_supply_min",	&pdata->vdd_ana_supply_min, 	DT_SUGGESTED,	DT_U32, 	0},
	{"Avago,vdd_ana_supply_max",	&pdata->vdd_ana_supply_max, 	DT_SUGGESTED,	DT_U32, 	0},
	{"Avago,vdd_ana_load_ua",	&pdata->vdd_ana_load_ua,	DT_SUGGESTED,	DT_U32, 	0},
	{"Avago,vddio_dig_supply_min",	&pdata->vddio_dig_supply_min,	DT_SUGGESTED,	DT_U32, 	0},
	{"Avago,vddio_dig_supply_max",	&pdata->vddio_dig_supply_max,	DT_SUGGESTED,	DT_U32, 	0},
	{"Avago,vddio_dig_load_ua", 	&pdata->vddio_dig_load_ua,	DT_SUGGESTED,	DT_U32, 	0},
	{"Avago,vddio_i2c_supply_min",	&pdata->vddio_i2c_supply_min,	DT_SUGGESTED,	DT_U32, 	0},
	{"Avago,vddio_i2c_supply_max",	&pdata->vddio_i2c_supply_max,	DT_SUGGESTED,	DT_U32, 	0},
	{"Avago,vddio_i2c_load_ua", 	&pdata->vddio_i2c_load_ua,	DT_SUGGESTED,	DT_U32, 	0},
	{"Avago,ppcount", 		&pdata->ppcount,		DT_SUGGESTED,	DT_U32, 	0},
	{"Avago,pdrive", 		&pdata->pdrive,			DT_SUGGESTED,	DT_U32, 	0},
	{"Avago,near_offset", 		&pdata->near_offset,		DT_SUGGESTED,	DT_U32, 	0},
	{"Avago,far_offset", 		&pdata->far_offset,		DT_SUGGESTED,	DT_U32, 	0},
	{"Avago,crosstalk_max", 	&pdata->crosstalk_max,		DT_SUGGESTED,	DT_U32, 	0},
	{NULL			, NULL				  , 0			, 0 	 ,	0},
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

		printk(KERN_INFO "DT entry ret:%d name:%s val:%d\n",
				ret, itr->dt_name, *((int *)itr->ptr_data));

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

	return err;

}

static int __devinit apds9190_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct apds9190_data *data;

	struct apds9190_platform_data *pdata;
	pm_message_t dummy_state;
	int err = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		err = -EIO;
		goto exit;
	}

	data = devm_kzalloc(&client->dev, sizeof(struct apds9190_data), GFP_KERNEL);

	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct apds9190_platform_data), GFP_KERNEL);
		if (!pdata) {
			ERR_MSG("Failed to allocate memory\n");
			return -ENOMEM;
		}
		data->platform_data = pdata;
		client->dev.platform_data = pdata;
		err = sensor_parse_dt(&client->dev, pdata);
		if(err)
			return err;

	} else {
		dev_err(&client->dev, "no of_node\n");
		return -EINVAL;
	}

	data->client = client;
	i2c_set_clientdata(data->client, data);

	/* h/w initialization */
	sensor_platform_hw_init(data);
	sensor_platform_hw_power_on(data, true);

	data->cross_talk = PS_DEFAULT_CROSS_TALK;
	data->sw_mode = PROX_STAT_OPERATING;
	atomic_set(&data->status, APDS9190_STATUS_RESUME);

	mdelay(50);

	spin_lock_init(&data->lock);
	mutex_init(&data->update_lock);

	/* Initialize the APDS9190 chip */
	client->adapter->retries=15;
	err = apds9190_init_client(client);
	if (err < 0) {
		ERR_MSG("Proximity apds9190_init_client Fail in Probe\n");
		goto exit_kfree;
	}

	INFO_MSG("enable = %s\n", data->enable ? "1" : "0");

	data->input_dev_ps = input_allocate_device();

	if (!data->input_dev_ps) {
		ERR_MSG("not enough memory for input device\n");
		goto exit_kfree;
	}

	data->input_dev_ps->name = SENSOR_NAME;
	data->input_dev_ps->uniq = SENSOR_DEVICE;
	set_bit(EV_SYN, data->input_dev_ps->evbit);
	set_bit(EV_ABS, data->input_dev_ps->evbit);
	input_set_abs_params(data->input_dev_ps, ABS_DISTANCE, 0, 1, 0, 0);
	input_set_drvdata(data->input_dev_ps, data);
	err = input_register_device(data->input_dev_ps);

	if (err) {
		DEBUG_MSG("Unable to register input device: %s\n",
				data->input_dev_ps->name);
		goto exit_input_register_device_failed;
	}

	wake_lock_init(&data->wakelock, WAKE_LOCK_SUSPEND, "apds9190");

#ifdef 	APDS9190_DELAYED_WORK
	INIT_DELAYED_WORK(&data->dwork, apds9190_work_handler);
#else
	INIT_WORK(&data->dwork, apds9190_work_handler);
#endif

	data->irq = gpio_to_irq(pdata->irq_gpio);
	if (request_irq(data->irq,apds_9190_irq_handler, IRQF_TRIGGER_FALLING, "proximity_irq", data) < 0){
		err = -EIO;
		ERR_MSG("Proximity irq Fail in Probe\n");
		goto exit_request_irq_failed;
	}
	err = irq_set_irq_wake(data->irq, 1);
	if (err)
		irq_set_irq_wake(data->irq, 0);

	/* Register sysfs hooks */
	err = sysfs_create_group(&data->input_dev_ps->dev.kobj, &apds9190_attr_group);
	if (err)
		goto exit_sysfs_create_failed;

	dummy_state.event = 0;
	apds9190_disable(data->client, dummy_state);

	return 0;

exit_sysfs_create_failed:
	apds9190_set_enable(client, 0);
	free_irq(data->irq, NULL);
exit_request_irq_failed:
	sensor_platform_hw_exit(data);
	wake_lock_destroy(&data->wakelock);
	input_unregister_device(data->input_dev_ps);
exit_input_register_device_failed:
exit_kfree:
	dev_info(&client->dev, "probe error\n");
exit:
	return err;
}

static int __devexit apds9190_remove(struct i2c_client *client)
{
	struct apds9190_data *data = i2c_get_clientdata(client);

	DEBUG_MSG("\n");

	sysfs_remove_group(&data->input_dev_ps->dev.kobj, &apds9190_attr_group);
	input_unregister_device(data->input_dev_ps);
	input_free_device(data->input_dev_ps);

	irq_set_irq_wake(data->irq, 0);
	apds9190_set_enable(client, 0);
	free_irq(data->irq, NULL);
	sensor_platform_hw_exit(data);
	wake_lock_destroy(&data->wakelock);

	return 0;
}

static int apds9190_disable(struct i2c_client *client, pm_message_t mesg)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int err;

	DEBUG_MSG("data->sw_mode[%d]\n", data->sw_mode);

	if (!data->sw_mode)
		return 0;

	apds9190_set_enable(client, 0);
	apds9190_set_command(client, 2);

#ifdef APDS9190_DELAYED_WORK
	__cancel_delayed_work(&data->dwork);
#else
	cancel_work_sync(&data->dwork);
	flush_work(&data->dwork);
#endif

	flush_workqueue(proximity_wq);

	data->sw_mode = PROX_STAT_SHUTDOWN;
	disable_irq(data->irq);
	err = sensor_platform_hw_power_on(data, false);
	if (err < 0) {
		DEBUG_MSG("Proximity Power Off Fail in susped\n");
		return err;
	}

	irq_set_irq_wake(data->irq, 0);

	return 0;
}


static int apds9190_enable(struct i2c_client *client)
{
	struct apds9190_data *data = i2c_get_clientdata(client);
	int ret;

	DEBUG_MSG("\n");

	ret = sensor_platform_hw_power_on(data, true);
	if (ret < 0) {
		DEBUG_MSG("Proximity Power On Fail in Resume\n");
		return ret;
	}

	mdelay(50);

	ret = apds9190_init_client(client);
	ret = apds9190_set_enable(client,0x3F);

	if (ret < 0) {
		DEBUG_MSG("Proximity apds_9190_initialize Fail in Resume\n");
		return ret;
	}

	data->sw_mode = PROX_STAT_OPERATING;

	enable_irq(data->irq);
	ret = irq_set_irq_wake(data->irq, 1);
	if (ret)
		irq_set_irq_wake(data->irq, 0);

	apds9190_set_command(client, 2);

	return 0;
}


#ifdef CONFIG_PM
static int apds9190_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct apds9190_data *data = i2c_get_clientdata(client);

	atomic_set(&data->status, APDS9190_STATUS_SUSPEND);
	return 0;
}

static int apds9190_resume(struct i2c_client *client)
{
	struct apds9190_data *data = i2c_get_clientdata(client);

	if (data->sw_mode == PROX_STAT_OPERATING) {
		if (atomic_read(&data->status) == APDS9190_STATUS_QUEUE_WORK) {
			apds_9190_add_workqueue(data);
		}
	}
	atomic_set(&data->status, APDS9190_STATUS_RESUME);
	return 0;
}
#endif

static const struct i2c_device_id apds9190_id[] = {
	{ "apds9190", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, apds9190_id);


#ifdef CONFIG_OF
static struct of_device_id apds9190_match_table[] = {
    { .compatible = "Avago,apds9190",},
    { },
};
#endif

static struct i2c_driver apds9190_driver = {
	.driver = {
		.name	= APDS9190_DRV_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = apds9190_match_table,
#endif
	},
	.probe	= apds9190_probe,
#ifdef CONFIG_PM
	.suspend = apds9190_suspend,
	.resume	= apds9190_resume,
#endif
	.remove	= __devexit_p(apds9190_remove),
	.id_table = apds9190_id,
};

static int __init apds9190_init(void)
{
	int err;
	if(proximity_wq == NULL) {
		proximity_wq = create_singlethread_workqueue("proximity_wq");
			if(NULL == proximity_wq)
				return -ENOMEM;
		err = i2c_add_driver(&apds9190_driver);
		if(err < 0){
			ERR_MSG("Failed to i2c_add_driver \n");
			return err;
		}
	}
	return 0;
}

static void __exit apds9190_exit(void)
{
	i2c_del_driver(&apds9190_driver);
	if(proximity_wq != NULL) {
		if(proximity_wq)
			destroy_workqueue(proximity_wq);
	}
}

MODULE_AUTHOR("Lee Kai Koon <kai-koon.lee@avagotech.com>");
MODULE_DESCRIPTION("APDS9190 proximity sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(apds9190_init);
module_exit(apds9190_exit);


