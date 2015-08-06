/* lge_touch_melfas.h
 *
 * Copyright (C) 2013 LGE.
 *
 * Author: WX-BSP-TS@lge.com
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/types.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/gpio.h>
#include <linux/firmware.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <asm/unaligned.h>
#include <mach/gpiomux.h>

#include "lge_ts_core.h"

#ifndef LGE_TS_MELFAS_H
#define LGE_TS_MELFAS_H

#define FW_BLOCK_SIZE		256
#define FW_MAX_SIZE		(64 * 1024)

enum melfas_ic_type {
	MMS100S = 0,
	MMS128S = MMS100S,
	MMS134S = MMS100S,
	MMS100A = 1,
	MMS136 = MMS100A,
	MMS144 = MMS100A,
	MMS152 = MMS100A,
	MIT200 = 2,
};

enum {
	FW_UP_TO_DATE = 0,
	FW_UPDATE_BY_ISC,
	FW_UPDATE_BY_ISP,
};

enum  {
	GPIOMODE_ISP_START = 0,
	GPIOMODE_ISP_END,
	GPIOMODE_FX_START,
	GPIOMODE_FX_END,
};

enum {
	GPIO_SDA = 0,
	GPIO_SCL,
	GPIO_INT
};

#define GPIO_TOUCH_INT (info->pdata->int_pin)
#define GPIO_TOUCH_SCL (info->pdata->scl_pin)
#define GPIO_TOUCH_SDA (info->pdata->sda_pin)

#define FINGER_EVENT_SZ		6
#define MIT_FINGER_EVENT_SZ		8
#define MIT_LPWG_EVENT_SZ		5
#define MAX_WIDTH		30
#define MAX_PRESSURE		255
#define MAX_LOG_LENGTH		128

#define SECTION_NUM		3
#define PAGE_HEADER		3
#define PAGE_DATA		1024
#define PAGE_CRC		2
#define PACKET_SIZE		(PAGE_HEADER + PAGE_DATA + PAGE_CRC)

#define MAX_COL	15
#define MAX_ROW	25

#define OPENSHORT_MAX	550
#define OPENSHORT_MIN	10

/* MIT 200 Registers */

#define MIT_REGH_CMD				0x10

#define MIT_REGL_MODE_CONTROL			0x01
#define MIT_REGL_ROW_NUM			0x0B
#define MIT_REGL_COL_NUM			0x0C
#define MIT_REGL_RAW_TRACK			0x0E
#define MIT_REGL_EVENT_PKT_SZ			0x0F
#define MIT_REGL_INPUT_EVENT			0x10
#define MIT_REGL_UCMD				0xA0
#define MIT_REGL_UCMD_RESULT_LENGTH		0xAE
#define MIT_REGL_UCMD_RESULT			0xAF
#define MIT_FW_VERSION				0xC2
#define MIT_FW_PRODUCT		0xE0

/*MMS Registers */

#define MMS_MODE_CONTROL	0x01
#define MMS_XY_RESOLUTION_HIGH	0x02
#define MMS_TX_NUM		0x0B
#define MMS_RX_NUM		0x0C
#define MMS_KEY_NUM		0x0D

#define MMS_MAX_TX_NUM		26
#define MMS_MAX_RX_NUM		18
#define MMS_MAX_KEY_NUM	4

#define MMS_EVENT_PKT_SZ			0x0F
#define MMS_INPUT_EVENT				0x10
#define MMS_SET_EDGE_EXPAND		0x32

#define MMS_UNIVCMD_GET_TOP_EDGE_EXPAND			0x25
#define MMS_UNIVCMD_GET_BOTTOM_EDGE_EXPAND		0x26
#define MMS_UNIVCMD_GET_LEFT_EDGE_EXPAND			0x27
#define MMS_UNIVCMD_GET_RIGHT_EDGE_EXPAND		0x28

#define MMS_UNIVERSAL_CMD			0xA0
#define MMS_UNIVERSAL_RESULT_SIZE	0xAE
#define MMS_UNIVERSAL_RESULT		0xAF

#define MMS_ENTER_TEST_MODE			0x40
#define MMS_TEST_CHSTATUS			0x41
#define MMS_GET_PIXEL_CHSTATUS		0x42
#define MMS_TEST_RAWDATA			0x43
#define MMS_GET_PIXEL_RAWDATA		0x44
#define MMS_TEST_JITTER				0x45
#define MMS_GET_PIXEL_JITTER		0x46
#define MMS_KEY_CHSTATUS			0x4A
#define MMS_KEY_RAWDATA				0x4B
#define MMS_KEY_JITTER				0x4C
#define MMS_DELTA_SCREEN			0x70
#define MMS_DELTA_KEY				0x71

#define MMS_UNIVERSAL_CMD_EXIT		0x4F

#define MMS_FW_VERSION		0xE1
#define MMS_FW_PRODUCT		0xF6
#define MMS_POWER_CONTROL	0xB0

#define MMS_GET_CUSTOM_ADDRESS	0xE5
#define MMS_ENTER_ISC		0x5F
#define MMS_WRITE_CMD	   0xAE
#define MMS_ENABLE_WRITE    0x55
#define MMS_DATA_WRITE	   0xF1
#define MMS_CONFIRM_STATUS  0xAF
#define MMS_STATUS_ISC_READY    0x01
#define MMS_STATUS_WRITING_DONE 0x03

/* Universal commands */
#define MIT_UNIV_ENTER_TESTMODE			0x40
#define MIT_UNIV_TESTA_START			0x41
#define MIT_UNIV_GET_RAWDATA			0x44
#define MIT_UNIV_TESTB_START			0x48
#define MIT_UNIV_GET_OPENSHORT_TEST		0x50
#define MIT_UNIV_EXIT_TESTMODE			0x6F

#define MMS_CMD_SET_LOG_MODE	0x20

/* Event types */
#define MMS_LOG_EVENT		0xD
#define MMS_NOTIFY_EVENT	0xE
#define MMS_ERROR_EVENT		0xF
#define MMS_TOUCH_KEY_EVENT	0x40
#define MMS_LPWG_EVENT	0xE

enum {
	LOG_TYPE_U08	= 2,
	LOG_TYPE_S08,
	LOG_TYPE_U16,
	LOG_TYPE_S16,
	LOG_TYPE_U32	= 8,
	LOG_TYPE_S32,
};

enum {
	NONE = 0,
	RAW_DATA,
	OPENSHORT,
};

struct mms_dev {
	u16 x_resolution;
	u16 y_resolution;
	u8 contact_on_event_thres;
	u8 moving_event_thres;
	u8 active_report_rate;
	u8 operation_mode;
	u8 tx_ch_num;
	u8 rx_ch_num;
	u8 row_num;
	u8 col_num;
	u8 key_num;
};

struct mms_section {
	u8 version;
	u8 compatible_version;
	u8 start_addr;
	u8 end_addr;
	int offset;
	u32 crc;
};

const static char *section_name[SECTION_NUM] = {
	"boot", "core", "config"
};

struct mms_module {
	u8 product_code[16];
	u8 version[2];
};

struct mms_log {
	u8 *data;
	int cmd;
};

struct mms_bin_hdr {
	char	tag[8];
	u16	core_version;
	u16	section_num;
	u16	contains_full_binary;
	u16	reserved0;

	u32	binary_offset;
	u32	binary_length;

	u32	extention_offset;
	u32	reserved1;
} __attribute__ ((packed));

struct mms_fw_img {
	u16	type;
	u16	version;

	u16	start_page;
	u16	end_page;

	u32	offset;
	u32	length;

} __attribute__ ((packed));

struct mms_data {
	bool probed;

	struct i2c_client *client;
	struct touch_platform_data *pdata;
	struct regulator *vdd_regulator[TOUCH_PWR_NUM];

	struct mms_dev dev;
	bool need_update[SECTION_NUM];
	struct mms_section ts_section[SECTION_NUM];
	struct mms_bin_hdr *fw_hdr;
	struct mms_fw_img *fw_img[SECTION_NUM];
	struct mms_module module;
	char buf[PACKET_SIZE];
	struct mms_log log;
	u8 *get_data;
	int *raw_data_max;
	int *raw_data_min;
	u8 test_mode;
};

struct mms_log_pkt {
	u8	marker;
	u8	log_info;
	u8	code;
	u8	element_sz;
	u8	row_sz;
} __attribute__ ((packed));

#define mms_i2c_write_block(client, buf, len) i2c_master_send(client, buf, len)

int mms_i2c_read(struct i2c_client *client, u8 reg, char *buf, int len);
int mms_100s_isc(struct mms_data *ts, struct touch_fw_info *info);
int mms_100a_fw_upgrade(struct mms_data *ts, struct touch_fw_info *info);
int mit_isc_fwupdate(struct mms_data *ts, struct touch_fw_info *info);
int mms_set_gpio_mode(struct touch_platform_data *pdata, int mode);
int mms_power_ctrl(struct i2c_client *client, int power_ctrl);
int mms_power_reset(struct mms_data *ts);
ssize_t mit_delta_show(struct i2c_client *client, char *buf);
ssize_t mit_rawdata_show(struct i2c_client *client, char *buf);
ssize_t mit_openshort_show(struct i2c_client *client, char *buf);
int mit_isc_page_read(struct mms_data *ts, u8 *rdata, int addr);
int mit_isc_exit(struct mms_data *ts);
#endif /*LGE_TS_MELFAS_H */

