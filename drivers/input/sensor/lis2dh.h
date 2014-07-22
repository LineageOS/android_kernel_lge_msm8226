
/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
*
* File Name	: lis2dh_acc.h
* Authors	: MH - C&I BU - Application Team
*		: Matteo Dameno (matteo.dameno@st.com)
* Version	: V.1.0.12
* Date		: 2012/Feb/29
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
*******************************************************************************/
/*******************************************************************************
Version History.

 Revision 1.0.10: 2011/Aug/16

 Revision 1.0.11: 2012/Jan/09
  moved under input/misc
 Revision 1.0.12: 2012/Feb/29
  moved use_smbus inside status struct; modified:-update_fs_range;-set_range
  input format; allows gpio_intX to be passed as parameter at insmod time;
  renamed field g_range to fs_range in lis2dh_acc_platform_data;
  replaced defines SA0L and SA0H with LIS2DH_SAD0x
*******************************************************************************/

#ifndef	__LIS2DH_H__
#define	__LIS2DH_H__

#define	LIS2DH_ACC_DEV_NAME		"accelerometer"


#define	LIS2DH_ACC_MIN_POLL_PERIOD_MS	1


#ifdef __KERNEL__

#define LIS2DH_SAD0L				(0x00)
#define LIS2DH_SAD0H				(0x01)
#define LIS2DH_ACC_I2C_SADROOT		(0x0C)

/* I2C address if acc SA0 pin to GND */
#define LIS2DH_ACC_I2C_SAD_L		((LIS2DH_ACC_I2C_SADROOT<<1)| \
						LIS2DH_SAD0L)

/* I2C address if acc SA0 pin to Vdd */
#define LIS2DH_ACC_I2C_SAD_H		((LIS2DH_ACC_I2C_SADROOT<<1)| \
						LIS2DH_SAD0H)

#define LIS2DH_ACC_DEFAULT_INT1_GPIO		(-EINVAL)
#define LIS2DH_ACC_DEFAULT_INT2_GPIO		(-EINVAL)

/* Accelerometer Sensor Full Scale */
#define	LIS2DH_ACC_FS_MASK		(0x30)
#define LIS2DH_ACC_G_2G			(0x00)
#define LIS2DH_ACC_G_4G			(0x10)
#define LIS2DH_ACC_G_8G			(0x20)
#define LIS2DH_ACC_G_16G		(0x30)

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



struct lis2dh_acc_platform_data {
	unsigned int poll_interval;
	unsigned int min_interval;

	u8 fs_range;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(bool);

	int (*power_off)(void);

	/* set gpio_int[1,2] either to the choosen gpio pin number or to -EINVAL
	 * if leaved unconnected
	 */
	int gpio_int1;
	int gpio_int2;
	bool i2c_pull_up;
	bool digital_pwr_regulator;

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
#ifdef CONFIG_BMA_USE_PLATFORM_DATA
	u32 place;
#endif
};

#endif	/* __KERNEL__ */

#endif	/* __LIS2DH_H__ */



