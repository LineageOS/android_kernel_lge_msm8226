
/******************** (C) COPYRIGHT 2013 STMicroelectronics ********************
*
* File Name	: lsm303c.h
* Authors	: AMS - Motion Mems Division - Application Team
*		: Matteo Dameno (matteo.dameno@st.com)
*		: Denis Ciocca (denis.ciocca@st.com)
* Version	: V.1.0.0
* Date		: 2013/Jun/17
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
/******************************************************************************/

#ifndef	__LSM303C_H__
#define	__LSM303C_H__


#define LSM303C_ACC_DEV_NAME			"lsm303c_acc"
#define	LSM303C_MAG_DEV_NAME			"lsm303c_mag"

#define LSM303C_ACC_MIN_POLL_PERIOD_MS		2

#define DEBUG

#ifdef __KERNEL__

/* to set gpios numb connected to interrupt pins,
 * the unused ones have to be set to -EINVAL
 */
#define LSM303C_ACC_DEFAULT_INT1_GPIO		(81)

#define LSM303C_MAG_DEFAULT_INT1_GPIO		(83)

/* Accelerometer Sensor Full Scale */
#define LSM303C_ACC_FS_MASK			(0x30)
#define LSM303C_ACC_FS_2G			(0x00)
#define LSM303C_ACC_FS_4G			(0x20)
#define LSM303C_ACC_FS_8G			(0x30)

/* Magnetometer Sensor Full Scale */
#define LSM303C_MAG_FS_MASK			(0x60)
#define LSM303C_MAG_FS_4G			(0x00)	/* Full scale 4 G */
#define LSM303C_MAG_FS_8G			(0x20)	/* Full scale 8 G */
#define LSM303C_MAG_FS_10G			(0x40)	/* Full scale 10 G */

#define LSM303C_MAG_MIN_POLL_PERIOD_MS		5


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
	const char                  *dt_name;
	void                        *ptr_data;
	enum sensor_dt_entry_status status;
	enum sensor_dt_entry_type   type;
	int                          default_val;
};
#endif


struct lsm303c_acc_platform_data {
	unsigned int poll_interval;
	unsigned int min_interval;
	u8 fs_range;
	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;
	u8 negate_x;
	u8 negate_y;
	u8 negate_z;
#ifdef CONFIG_OF
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
#endif	
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(bool);
	int (*power_off)(void);
};

struct lsm303c_mag_platform_data {
	unsigned int poll_interval;
	unsigned int min_interval;
	u8 fs_range;
	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;
	u8 negate_x;
	u8 negate_y;
	u8 negate_z;
#ifdef CONFIG_OF
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
#endif
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(bool);
	int (*power_off)(void);
};

#endif	/* __KERNEL__ */

#endif	/* __LSM303C_H__ */



