
/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
*
* File Name          : k3dh.h
* Authors            : MH - C&I BU - Application Team
*		     : Carmine Iascone (carmine.iascone@st.com)
*		     : Matteo Dameno (matteo.dameno@st.com)
* Version			 : V.1.0.6
* Date				 : 02/03/2014
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
* THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
*
*******************************************************************************/
/*******************************************************************************
Version History.
Revision 1.0.6  02/03/2014
 Added self test function

Revision 1-0-0 05/11/2009
First Release
Revision 1-0-1 26/01/2010
Linux K&R Compliant Release
Revision 1-0-5 16/08/2010
Interrupt Management

*******************************************************************************/

#ifndef	__K3DH_H__
#define	__K3DH_H__

#include	<linux/ioctl.h>	/* For IOCTL macros */
#include	<linux/input.h>

#define SAD0L			0x00
#define SAD0H			0x01
#define K3DH_ACC_I2C_SADROOT	0x0C
#define K3DH_ACC_I2C_SAD_L	((K3DH_ACC_I2C_SADROOT<<1)|SAD0L)
#define K3DH_ACC_I2C_SAD_H	((K3DH_ACC_I2C_SADROOT<<1)|SAD0H)
#define	K3DH_ACC_DEV_NAME	"K3DH"


#define	K3DH_ACC_IOCTL_BASE 77
/** The following define the IOCTL command values via the ioctl macros */
#define	K3DH_ACC_IOCTL_SET_DELAY		_IOW(K3DH_ACC_IOCTL_BASE, 0, int)
#define	K3DH_ACC_IOCTL_GET_DELAY		_IOR(K3DH_ACC_IOCTL_BASE, 1, int)
#define	K3DH_ACC_IOCTL_SET_ENABLE		_IOW(K3DH_ACC_IOCTL_BASE, 2, int)
#define	K3DH_ACC_IOCTL_GET_ENABLE		_IOR(K3DH_ACC_IOCTL_BASE, 3, int)
#define	K3DH_ACC_IOCTL_SET_FULLSCALE		_IOW(K3DH_ACC_IOCTL_BASE, 4, int)
#define	K3DH_ACC_IOCTL_SET_G_RANGE		K3DH_ACC_IOCTL_SET_FULLSCALE

#define	K3DH_ACC_IOCTL_SET_CTRL_REG3		_IOW(K3DH_ACC_IOCTL_BASE, 6, int)
#define	K3DH_ACC_IOCTL_SET_CTRL_REG6		_IOW(K3DH_ACC_IOCTL_BASE, 7, int)
#define	K3DH_ACC_IOCTL_SET_DURATION1		_IOW(K3DH_ACC_IOCTL_BASE, 8, int)
#define	K3DH_ACC_IOCTL_SET_THRESHOLD1		_IOW(K3DH_ACC_IOCTL_BASE, 9, int)
#define	K3DH_ACC_IOCTL_SET_CONFIG1		_IOW(K3DH_ACC_IOCTL_BASE, 10, int)

#define	K3DH_ACC_IOCTL_SET_DURATION2		_IOW(K3DH_ACC_IOCTL_BASE, 11, int)
#define	K3DH_ACC_IOCTL_SET_THRESHOLD2		_IOW(K3DH_ACC_IOCTL_BASE, 12, int)
#define	K3DH_ACC_IOCTL_SET_CONFIG2		_IOW(K3DH_ACC_IOCTL_BASE, 13, int)

#define	K3DH_ACC_IOCTL_GET_SOURCE1		_IOW(K3DH_ACC_IOCTL_BASE, 14, int)
#define	K3DH_ACC_IOCTL_GET_SOURCE2		_IOW(K3DH_ACC_IOCTL_BASE, 15, int)

#define	K3DH_ACC_IOCTL_GET_TAP_SOURCE		_IOW(K3DH_ACC_IOCTL_BASE, 16, int)

#define	K3DH_ACC_IOCTL_SET_TAP_TW		_IOW(K3DH_ACC_IOCTL_BASE, 17, int)
#define	K3DH_ACC_IOCTL_SET_TAP_CFG		_IOW(K3DH_ACC_IOCTL_BASE, 18, int)
#define	K3DH_ACC_IOCTL_SET_TAP_TLIM		_IOW(K3DH_ACC_IOCTL_BASE, 19, int)
#define	K3DH_ACC_IOCTL_SET_TAP_THS		_IOW(K3DH_ACC_IOCTL_BASE, 20, int)
#define	K3DH_ACC_IOCTL_SET_TAP_TLAT		_IOW(K3DH_ACC_IOCTL_BASE, 21, int)

#define K3DH_ACC_IOCTL_GET_SELFTEST		_IOW(K3DH_ACC_IOCTL_BASE, 22, int)


/************************************************/
/*			Accelerometer defines section				   */
/************************************************/

/* Accelerometer Sensor Full Scale */
#define	K3DH_ACC_FS_MASK		0x30
#define K3DH_ACC_G_2G		0x00
#define K3DH_ACC_G_4G		0x10
#define K3DH_ACC_G_8G		0x20
#define K3DH_ACC_G_16G		0x30


/* Accelerometer Sensor Operating Mode */
#define K3DH_ACC_ENABLE		0x01
#define K3DH_ACC_DISABLE		0x00






#ifdef	__KERNEL__

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

struct k3dh_acc_platform_data {
	int poll_interval;
	int min_interval;

	u8 g_range;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(struct k3dh_acc_platform_data *);
	int (*power_off)(struct k3dh_acc_platform_data *);

	unsigned int gpio_int1;
#ifdef USE_ACC_IRQ2
	unsigned int gpio_int2;
#endif
	struct regulator *vdd_supply;
	struct regulator *vcc_i2c_supply;

};
#endif	/* __KERNEL__ */

#endif	/* __K3DH_H__ */



