/* lge_ts_melfas.c
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

/* History :
 *
 */
#include <linux/uaccess.h>
#include <linux/time.h>
#include <linux/file.h>
#include <linux/syscalls.h>

#include "lge_ts_melfas.h"

#define ts_pdata		((ts)->pdata)
#define ts_caps		(ts_pdata->caps)
#define ts_role		(ts_pdata->role)
#define ts_pwr		(ts_pdata->pwr)

#ifdef SENSING_TEST
#define SENSING_TEST_PATH		"/data/ts_log.txt"
static char sensing_test = 0;
#endif
static uint8_t edge_expand[4] = {0};

static int mms_get_packet(struct i2c_client *client);
static int mms_power(struct i2c_client* client, int power_ctrl);

int mms_i2c_read(struct i2c_client *client, u8 reg, char *buf, int len)
{
	int ret = 0;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};

	ret = i2c_transfer(client->adapter, msgs, 2);

	if (ret < 0) {
		if (printk_ratelimit())
			TOUCH_ERR_MSG("transfer error: %d\n", ret);
		return -EIO;
	} else
		return 0;
}

#if 0
static int mms_i2c_write(struct i2c_client *client, u8 reg, int len, u8 *buf)
{
	unsigned char send_buf[len + 1];
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = client->flags,
			.len = len+1,
			.buf = send_buf,
		},
	};

	send_buf[0] = (unsigned char)reg;
	memcpy(&send_buf[1], buf, len);

	if (i2c_transfer(client->adapter, msgs, 1) < 0) {
		if (printk_ratelimit())
			TOUCH_ERR_MSG("transfer error\n");
		return -EIO;
	} else
		return 0;
}
#endif

static struct gpiomux_setting gpio_isp_config[] = {
	{ // SDA, SCL
		.func = GPIOMUX_FUNC_GPIO,
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_DOWN,
		.dir = GPIOMUX_OUT_LOW,
	},
	{ // INT
		.func = GPIOMUX_FUNC_GPIO,
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_UP,
		.dir = GPIOMUX_IN,
	}
};

static struct gpiomux_setting gpio_i2c_config[] = {
	{ // SDA, SCL
		.func = GPIOMUX_FUNC_3,
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
		.dir = GPIOMUX_IN,
	},
	{ // INT
		.func = GPIOMUX_FUNC_GPIO,
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_UP,
		.dir = GPIOMUX_IN,
	}
};

static struct gpiomux_setting gpio_fx_config[] = {
	{ // SDA, SCL
		.func = GPIOMUX_FUNC_GPIO,
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
		.dir = GPIOMUX_IN,
	},
	{ // INT
		.func = GPIOMUX_FUNC_GPIO,
		.drv = GPIOMUX_DRV_2MA,
		.pull = GPIOMUX_PULL_NONE,
		.dir = GPIOMUX_IN,
	}
};

static struct msm_gpiomux_config gpio_configs[] = {
	{
		.gpio = 0,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_config[0],
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config[0],
		}
	},
	{
		.gpio = 0,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_config[0],
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config[0],
		}
	},
	{
		.gpio = 0,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_config[1],
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config[1],
		}
	},
};

int mms_set_gpio_mode(struct touch_platform_data *pdata, int mode)
{
	int ret = 0;

	gpio_configs[GPIO_SDA].gpio = pdata->sda_pin;
	gpio_configs[GPIO_SCL].gpio = pdata->scl_pin;
	gpio_configs[GPIO_INT].gpio = pdata->int_pin;

	TOUCH_INFO_MSG("PIN Set : SDA:%d SCL:%d INT:%d \n", pdata->sda_pin, pdata->scl_pin, pdata->int_pin);

	if (mode == GPIOMODE_ISP_START) {
		TOUCH_INFO_MSG("%s (ISP Start) \n", __func__);
		gpio_configs[GPIO_SDA].settings[GPIOMUX_ACTIVE] = &gpio_isp_config[0];
		gpio_configs[GPIO_SDA].settings[GPIOMUX_SUSPENDED] = &gpio_isp_config[0];
		gpio_configs[GPIO_SCL].settings[GPIOMUX_ACTIVE] = &gpio_isp_config[0];
		gpio_configs[GPIO_SCL].settings[GPIOMUX_SUSPENDED] = &gpio_isp_config[0];
		gpio_configs[GPIO_INT].settings[GPIOMUX_ACTIVE] = &gpio_isp_config[1];
		gpio_configs[GPIO_INT].settings[GPIOMUX_SUSPENDED] = &gpio_isp_config[1];

		msm_gpiomux_install(gpio_configs, ARRAY_SIZE(gpio_configs));
		msleep(30);

		ret = gpio_request(pdata->sda_pin, "GPIO_TOUCH_SDA");
		if (ret) {
			gpio_free(pdata->sda_pin);
			ret = gpio_request(pdata->sda_pin, "GPIO_TOUCH_SDA");
			if (ret)
				TOUCH_INFO_MSG("gpio_request(%d) failed \n", pdata->sda_pin);
		}
		ret = gpio_request(pdata->scl_pin, "GPIO_TOUCH_SCL");
		if (ret) {
			gpio_free(pdata->scl_pin);
			ret = gpio_request(pdata->scl_pin, "GPIO_TOUCH_SCL");
			if (ret)
				TOUCH_INFO_MSG("gpio_request(%d) failed \n", pdata->scl_pin);
		}
		ret = gpio_request(pdata->int_pin, "GPIO_TOUCH_INT");
		if (ret) {
			gpio_free(pdata->int_pin);
			ret = gpio_request(pdata->int_pin, "GPIO_TOUCH_INT");
			if (ret)
				TOUCH_INFO_MSG("gpio_request(%d) failed \n", pdata->int_pin);
		}
	} else if (mode == GPIOMODE_ISP_END) {
		TOUCH_INFO_MSG("%s (ISP End) \n", __func__);
		gpio_configs[GPIO_SDA].settings[GPIOMUX_ACTIVE] = &gpio_i2c_config[0];
		gpio_configs[GPIO_SDA].settings[GPIOMUX_SUSPENDED] = &gpio_i2c_config[0];
		gpio_configs[GPIO_SCL].settings[GPIOMUX_ACTIVE] = &gpio_i2c_config[0];
		gpio_configs[GPIO_SCL].settings[GPIOMUX_SUSPENDED] = &gpio_i2c_config[0];
		gpio_configs[GPIO_INT].settings[GPIOMUX_ACTIVE] = &gpio_i2c_config[1];
		gpio_configs[GPIO_INT].settings[GPIOMUX_SUSPENDED] = &gpio_i2c_config[1];

		gpio_free(pdata->sda_pin);
		gpio_free(pdata->scl_pin);
		gpio_free(pdata->int_pin);
		msleep(30);

		msm_gpiomux_install(gpio_configs , ARRAY_SIZE(gpio_configs));
	} else if (mode == GPIOMODE_FX_START) {
		TOUCH_INFO_MSG("%s (FX Start) \n", __func__);
		gpio_configs[GPIO_SDA].settings[GPIOMUX_ACTIVE] = &gpio_fx_config[0];
		gpio_configs[GPIO_SDA].settings[GPIOMUX_SUSPENDED] = &gpio_fx_config[0];
		gpio_configs[GPIO_SCL].settings[GPIOMUX_ACTIVE] = &gpio_fx_config[0];
		gpio_configs[GPIO_SCL].settings[GPIOMUX_SUSPENDED] = &gpio_fx_config[0];
		gpio_configs[GPIO_INT].settings[GPIOMUX_ACTIVE] = &gpio_fx_config[1];
		gpio_configs[GPIO_INT].settings[GPIOMUX_SUSPENDED] = &gpio_fx_config[1];

		msm_gpiomux_install(gpio_configs , ARRAY_SIZE(gpio_configs));
	} else if (mode == GPIOMODE_FX_END) {
		TOUCH_INFO_MSG("%s (FX End) \n", __func__);
		gpio_configs[GPIO_SDA].settings[GPIOMUX_ACTIVE] = &gpio_i2c_config[0];
		gpio_configs[GPIO_SDA].settings[GPIOMUX_SUSPENDED] = &gpio_i2c_config[0];
		gpio_configs[GPIO_SCL].settings[GPIOMUX_ACTIVE] = &gpio_i2c_config[0];
		gpio_configs[GPIO_SCL].settings[GPIOMUX_SUSPENDED] = &gpio_i2c_config[0];
		gpio_configs[GPIO_INT].settings[GPIOMUX_ACTIVE] = &gpio_i2c_config[1];
		gpio_configs[GPIO_INT].settings[GPIOMUX_SUSPENDED] = &gpio_i2c_config[1];

		msm_gpiomux_install(gpio_configs , ARRAY_SIZE(gpio_configs));
	}

	return ret;
}
EXPORT_SYMBOL(mms_set_gpio_mode);

void mms_edge_expand_write(struct mms_data *ts)
{
	int i = 0;
	uint8_t write_buf[5] = {0};
	struct i2c_msg msg[] = {
		{
			.addr = ts->client->addr,
			.flags = 0,
			.buf = write_buf,
		},
		{
			.addr = ts->client->addr,
			.flags = 1,
		},
	};

	for (i = 0; i < 4; i++) {
		if (edge_expand[i] == 0 )
			return;
	}

	msg[0].len = 5;
	write_buf[0] = MMS_SET_EDGE_EXPAND;
	write_buf[1] = edge_expand[0]; //top
	write_buf[2] = edge_expand[1]; //bottom
	write_buf[3] = edge_expand[2]; //left
	write_buf[4] = edge_expand[3]; //right

	if (i2c_transfer(ts->client->adapter, &msg[0], 1) != 1) {
		TOUCH_INFO_MSG("mms_edge_expand_write i2c transfer failed\n");
	}

	TOUCH_INFO_MSG("Write Edge Expand : %d, %d, %d, %d \n",
		edge_expand[0], edge_expand[1], edge_expand[2], edge_expand[3]);
}

void mms_edge_expand_read(struct mms_data *ts)
{
	int i = 0;
	uint8_t read_buf[16] = {0};
	uint8_t write_buf[4] = {0};
	uint8_t edge_tmp[4] = {0};
	struct i2c_msg msg[] = {
		{
			.addr = ts->client->addr,
			.flags = 0,
			.buf = write_buf,
		},{
			.addr = ts->client->addr,
			.flags = 1,
		},
	};

	for ( i = 0; i < 4; i++) {
		write_buf[0] = MMS_UNIVERSAL_CMD;
		write_buf[1] = MMS_UNIVCMD_GET_TOP_EDGE_EXPAND + i;
		msg[0].len = 2;

		if (i2c_transfer(ts->client->adapter, &msg[0], 1) != 1) {
			TOUCH_INFO_MSG("%s : i2c transfer failed\n", __func__);
			return;
		}

		if (mms_i2c_read(ts->client, MMS_UNIVERSAL_RESULT_SIZE, read_buf, 1) < 0) {
			TOUCH_INFO_MSG("%s : Fail to get MMS_UNIVERSAL_RESULT_SIZE \n", __func__);
			return;
		}

		if (mms_i2c_read(ts->client, MMS_UNIVERSAL_RESULT, read_buf, 1) < 0) {
			TOUCH_INFO_MSG("%s : Fail to get MMS_UNIVERSAL_RESULT \n", __func__);
			return;
		}

		if (read_buf[0] == 0)
			return;

		edge_tmp[i] = read_buf[0];
	}

	for ( i = 0; i < 4; i++)
		edge_expand[i] = edge_tmp[i];

	TOUCH_INFO_MSG("Read Edge Expand : %d, %d, %d, %d \n",
		edge_expand[0], edge_expand[1], edge_expand[2], edge_expand[3]);

	return;
}

static int mms_get_panel_id(struct mms_data *ts)
{
	int panel_id = 0xFF;
	int ret = 0;
	int value = 0;
	static bool init = false;

	if (init && ts_pdata->panel_id) {
		return ts_pdata->panel_id;
	}

	if (!init) {
		if (ts_pdata->id_pin && gpio_is_valid(ts_pdata->id_pin)) {
			ret = gpio_request(ts_pdata->id_pin, "touch_id");
			if (ret < 0) {
				TOUCH_ERR_MSG("FAIL: touch_id gpio_request = %d\n", ret);
				goto Exit;
			} else {
				gpio_direction_input(ts_pdata->id_pin);
			}
		}

		if (ts_pdata->id2_pin && gpio_is_valid(ts_pdata->id2_pin)) {
			ret = gpio_request(ts_pdata->id2_pin, "touch_id2");
			if (ret < 0) {
				TOUCH_ERR_MSG("FAIL: touch_id2 gpio_request = %d\n", ret);
				goto Exit;
			} else {
				gpio_direction_input(ts_pdata->id2_pin);
			}
		}

		init = true;
	}

	if (ts_pdata->id_pin && gpio_is_valid(ts_pdata->id_pin)) {
		value = gpio_get_value(ts_pdata->id_pin);
		panel_id = (value & 0x1);
	}

	if (ts_pdata->id2_pin && gpio_is_valid(ts_pdata->id2_pin)) {
		value = gpio_get_value(ts_pdata->id2_pin);
		panel_id += ((value & 0x1) << 1);
	}

	ts_pdata->panel_id = panel_id;

	return panel_id;

Exit :

	TOUCH_ERR_MSG("%s FAIL \n", __func__);

	return 0xFF;
}

static int mms_get_ic_info(struct mms_data *ts, struct touch_fw_info *fw_info)
{
	struct i2c_client *client = ts->client;
	char buf[16] = {0};
	int i = 0;
	int panel_id = 0;

	TOUCH_TRACE_FUNC();

	if (mms_i2c_read(client, MMS_XY_RESOLUTION_HIGH, buf, 8) < 0)
		return -EIO;

	ts->dev.x_resolution = (buf[0] & 0x0f) << 8 | buf[1];
	ts->dev.y_resolution = (buf[0] & 0xf0) << 4 | buf[2];
	ts->dev.contact_on_event_thres = buf[3];
	ts->dev.moving_event_thres = buf[4];
	ts->dev.active_report_rate = buf[5];
	ts->dev.operation_mode = buf[7];

	if (mms_i2c_read(client, MMS_TX_NUM, buf, 3) < 0)
		return -EIO;

	ts->dev.tx_ch_num = buf[0];
	ts->dev.rx_ch_num = buf[1];
	ts->dev.key_num = buf[2];

	if (mms_i2c_read(client, MMS_FW_VERSION, buf, SECTION_NUM) < 0)
		return -EIO;

	for (i = 0; i < SECTION_NUM; i++)
		ts->ts_section[i].version = buf[i];

	ts->ts_section[1].compatible_version = ts->ts_section[0].version;
	ts->ts_section[2].compatible_version = ts->ts_section[1].version;

	if (mms_i2c_read(client, MMS_GET_CUSTOM_ADDRESS, buf, 8) < 0)
		return -EIO;

	for (i = 0; i < SECTION_NUM; i++) {
		ts->ts_section[i].start_addr = buf[i];
		ts->ts_section[i].end_addr = buf[i+4];
	}

	if (mms_i2c_read(client, MMS_FW_PRODUCT, (u8 *) &ts->module, 8) < 0)
		return -EIO;

	panel_id = mms_get_panel_id(ts);
	if (unlikely((touch_debug_mask_ & DEBUG_BASE_INFO))) {
#if 1
		TOUCH_INFO_MSG("Firmware Version : %d.%02d \n", (ts->ts_section[2].version&0x80?1:0), ts->ts_section[2].version&0x7F);
		TOUCH_INFO_MSG("Boot:0x%X Core:0x%X Config:0x%X \n", ts->ts_section[0].version, ts->ts_section[1].version, ts->ts_section[2].version);
		TOUCH_INFO_MSG("FW Product : %s \n", ts->module.product_code);
		if (panel_id != 0xFF)
			TOUCH_INFO_MSG("Panel ID : %d [%c] \n", panel_id, ts_pdata->panel_type[panel_id]);
		TOUCH_INFO_MSG("Num of Channel. TX:%d RX:%d KEY:%d\n", ts->dev.tx_ch_num, ts->dev.rx_ch_num, ts->dev.key_num);
#else
		TOUCH_INFO_MSG("mms dev:\n");
		TOUCH_INFO_MSG("\tx_resolution=%d, y_resolution=%d\n",
				ts->dev.x_resolution, ts->dev.y_resolution);
		TOUCH_INFO_MSG("\ttx_ch_num=%d, rx_ch_num=%d, key_num=%d\n",
				ts->dev.tx_ch_num, ts->dev.rx_ch_num, ts->dev.key_num);
		TOUCH_INFO_MSG("\tcontact_on_event_thres=%d, moving_event_thres=%d\n",
				ts->dev.contact_on_event_thres, ts->dev.moving_event_thres);
		TOUCH_INFO_MSG("\tactive_report_rate=%d, operation_mode=%d\n",
				ts->dev.active_report_rate, ts->dev.operation_mode);
		for (i = 0; i < SECTION_NUM; i++) {
			TOUCH_INFO_MSG("mms section[%d:%s]: ver=0x%02x(compatible:0x%02x), addr(0x%02d-0x%02d)\n",
					i, section_name[i], ts->ts_section[i].version, ts->ts_section[i].compatible_version,
					ts->ts_section[i].start_addr, ts->ts_section[i].end_addr);
		}
		TOUCH_INFO_MSG("mms product: %s\n", ts->module.product_code);
#endif
	}

	return 0;
}

static void write_file(char *filename, char *data, int time)
{
	int fd = 0;
	char time_string[64] = {0};
	struct timespec my_time;
	struct tm my_date;
	mm_segment_t old_fs = get_fs();

	my_time = __current_kernel_time();
	time_to_tm(my_time.tv_sec, sys_tz.tz_minuteswest * 60 * (-1), &my_date);
	snprintf(time_string, 64, "%02d-%02d %02d:%02d:%02d.%03lu ",
		my_date.tm_mon + 1,my_date.tm_mday,
		my_date.tm_hour, my_date.tm_min, my_date.tm_sec,
		(unsigned long) my_time.tv_nsec / 1000000);

	set_fs(KERNEL_DS);
	fd = sys_open(filename, O_WRONLY|O_CREAT|O_APPEND, 0666);
	if (fd >= 0) {
		if(time > 0)
			sys_write(fd, time_string, strlen(time_string));
		sys_write(fd, data, strlen(data));
		sys_close(fd);
	}
	set_fs(old_fs);
}

static int mms_probe(struct i2c_client *client, struct touch_platform_data *pdata)
{
	struct mms_data *ts = NULL;
	int ret = 0;
	int i = 0;
	char gpio_request_name[16] = {0};

	TOUCH_TRACE_FUNC();

	ts = devm_kzalloc(&client->dev, sizeof(struct mms_data), GFP_KERNEL);

	if (ts == NULL) {
		TOUCH_ERR_MSG("Can not allocate memory\n");
		return -ENOMEM;
	}

	ts->client = client;
	ts->pdata = pdata;
	ts->log.data = ts->buf;
	set_touch_handle_(client, ts);

	for (i = 0; i < TOUCH_PWR_NUM; ++i) {
		if (ts_pwr[i].type == 1 && gpio_is_valid(ts_pwr[i].value)) {
			snprintf(gpio_request_name, 16, "touch_vdd_%d", i);
			if (!strncmp(ts_pwr[i].name, "low", strlen("low")))
				ret = gpio_request_one(ts_pwr[i].value, GPIOF_OUT_INIT_LOW, gpio_request_name);
			else
				ret = gpio_request_one(ts_pwr[i].value, GPIOF_OUT_INIT_HIGH, gpio_request_name);

			if (ret) {
				ts_pwr[i].value = -1;
				goto err_regulator_get;
			}
		} else if (ts_pwr[i].type == 2) {
			ts->vdd_regulator[i] = regulator_get(&client->dev, ts_pwr[i].name);
			if (IS_ERR(ts->vdd_regulator[i])) {
				ret = PTR_ERR(ts->vdd_regulator[i]);
				TOUCH_ERR_MSG("Can NOT get regulator : %s, ret = %d\n",
						ts_pwr[i].name, ret);
				goto err_regulator_get;
			}

			if (regulator_count_voltages(ts->vdd_regulator[i]) > 0) {
				if ((ret = regulator_set_voltage(ts->vdd_regulator[i], ts_pwr[i].value, ts_pwr[i].value)) < 0) {
					TOUCH_ERR_MSG("Error set regulator(%s) voltage %d\n",
							ts_pwr[i].name, ts_pwr[i].value);
					goto err_regulator_get;
				}
			}
		}
	}

	return 0;

err_regulator_get:
	do {
		if (ts_pwr[i].type == 1) {
			if (gpio_is_valid(ts_pwr[i].value))
				gpio_free(ts_pwr[i].value);
		} else if(ts_pwr[i].type == 2) {
			if (ts->vdd_regulator != NULL && !IS_ERR(ts->vdd_regulator[i]))
				regulator_put(ts->vdd_regulator[i]);
		}
	} while(--i >= 0);
	return ret;
}

static void mms_remove(struct i2c_client* client)
{
	struct mms_data *ts = get_touch_handle_(client);
	int i = TOUCH_PWR_NUM-1;

	TOUCH_TRACE_FUNC();

	do {
		if (ts_pwr[i].type == 1) {
			if (!strncmp(ts_pwr[i].name, "low", strlen("low")))
				gpio_direction_output(ts_pwr[i].value, 1);
			else
				gpio_direction_output(ts_pwr[i].value, 0);

			if (gpio_is_valid(ts_pwr[i].value))
				gpio_free(ts_pwr[i].value);
		} else if (ts_pwr[i].type == 2) {
			if (ts->vdd_regulator[i] != NULL && !IS_ERR(ts->vdd_regulator[i])) {
				regulator_put(ts->vdd_regulator[i]);
			}
		}
	} while(--i >= 0);
}

static int mms_init(struct i2c_client* client, struct touch_fw_info* fw_info)
{
	struct mms_data *ts = get_touch_handle_(client);

	TOUCH_TRACE_FUNC();

	if (unlikely(mms_get_ic_info(ts, fw_info) < 0))
		return -EIO;

	ts->probed = true;

	/* TODO : charger related code */

	/* TODO : wakeup */

#if 0
	/* enable interrupt */
	mms_get_packet(client);
#endif

	return 0;
}


static int mms_touch_event(struct i2c_client *client, struct touch_data *data, u8 *buf, int sz)
{
	struct mms_data *ts = get_touch_handle_(client);
	u8 *tmp = buf;
	int i = 0;

	u8 touch_count = 0;
	u8 index = 0;
	u8 state = 0;
	u8 palm = 0;

	u8 id = 0;
	u16 x = 0;
	u16 y = 0;
	u8 touch_major = 0;
	u8 pressure = 0;
#ifdef SENSING_TEST
	char log_buf[256] = {0};
#endif

	TOUCH_TRACE_FUNC();

	data->total_num = touch_count;
	for (i = 0; i < sz; i += FINGER_EVENT_SZ) {
		tmp = buf + i;
		index = (tmp[0] & 0xf) - 1;
		state = (tmp[0] & 0x80) ? 1 : 0;

		if (tmp[0] & MMS_TOUCH_KEY_EVENT) {
			if (index < 0 || index >= ts_caps->number_of_button) {
				TOUCH_ERR_MSG("invalid key index (%d)\n", index);
				return -EIO;
			}
			data->curr_button.key_code = ts_caps->button_name[index];
			data->curr_button.state = state;

			if (unlikely(touch_debug_mask_ & DEBUG_GET_DATA))
				TOUCH_INFO_MSG("key_code=[0x%02x-%d], state=[%d]\n",
						data->curr_button.key_code, data->curr_button.key_code, data->curr_button.state);
		} else {
			if (index < 0 || index >= ts_caps->max_id) {
				TOUCH_ERR_MSG("invalid touch index (%d)\n", index);
				return -EIO;
			}

			id = index;
			palm = (tmp[0] & 0x10) ? 1 : 0;
			x = tmp[2] | ((tmp[1] & 0x0f) << 8);
			y = tmp[3] | ((tmp[1] & 0xf0) << 4);
			touch_major = tmp[4];
			pressure = tmp[5];

			if (palm) {
				if (state) {
					TOUCH_INFO_MSG("Palm detected : %d \n", pressure);
					data->palm = true;
				}
				else {
					TOUCH_INFO_MSG("Palm released : %d \n", pressure);
					data->palm = false;
				}
				return 0;
			}

			if (state) {
				data->curr_data[id].id = id;
				data->curr_data[id].x_position = x;
				data->curr_data[id].y_position = y;
				data->curr_data[id].width_major = touch_major;
				data->curr_data[id].width_minor = 0;
				data->curr_data[id].width_orientation = 0;
				data->curr_data[id].pressure = pressure;
				data->curr_data[id].status = FINGER_PRESSED;
				touch_count++;
			} else {
				data->curr_data[id].status = FINGER_RELEASED;
			}

			if (unlikely(touch_debug_mask_ & DEBUG_GET_DATA)) {
				TOUCH_INFO_MSG("<%d> pos(%4d,%4d) w_m[%2d] w_n[%2d] w_o[%2d] p[%2d]\n, s[%d]",
						id, data->curr_data[id].x_position, data->curr_data[id].y_position,
						data->curr_data[id].width_major, data->curr_data[id].width_minor,
						data->curr_data[id].width_orientation, data->curr_data[id].pressure, state);
			}

			if (data->curr_data[id].status == FINGER_PRESSED
				&& data->prev_data[id].status <= FINGER_RELEASED
				&& !data->curr_data[id].point_log_state) {
					++data->touch_count_num;
					if (likely(touch_debug_mask_ & DEBUG_ABS_POINT)) {
						TOUCH_INFO_MSG("%d finger pressed : <%d> x[%3d] y[%3d] z[%3d]\n",
						data->touch_count_num, id,
						data->curr_data[id].x_position,
						data->curr_data[id].y_position,
						data->curr_data[id].pressure);
#ifdef SENSING_TEST
						if(sensing_test == 1) {
							sprintf(log_buf,"%3d %3d %3d %s\n",
								data->curr_data[id].x_position,
								data->curr_data[id].y_position,
								data->curr_data[id].pressure,
								data->curr_data[id].status > 0 ? "DOWN" : "UP");
							write_file(SENSING_TEST_PATH, log_buf, 1);
						}
#endif
					}
					data->curr_data[id].point_log_state = 1;
			}

			if (data->curr_data[id].status == FINGER_RELEASED
				&& data->prev_data[id].point_log_state) {
					data->touch_count_num--;

					if (likely(touch_debug_mask_ & DEBUG_ABS_POINT)) {
						TOUCH_INFO_MSG("touch_release[%s] : <%d> x[%3d] y[%3d]\n",
						data->palm?"Palm":" ", id,
						data->prev_data[id].x_position,
						data->prev_data[id].y_position);
#ifdef SENSING_TEST
						if(sensing_test == 1) {
							sprintf(log_buf,"%3d %3d     UP\n",
								data->curr_data[id].x_position,
								data->curr_data[id].y_position);
							write_file(SENSING_TEST_PATH, log_buf, 1);
						}
#endif
					}
					data->curr_data[id].point_log_state = 0;
			}
		}
	}

	data->total_num = touch_count;

	if (unlikely(touch_debug_mask_ & DEBUG_GET_DATA))
		TOUCH_INFO_MSG("Total_num: %d\n", data->total_num);

	return 0;
}

static int mms_log_event(struct i2c_client *client, struct mms_data *ts)
{
	struct mms_log_pkt *pkt = (struct mms_log_pkt *) ts->buf;
	char *tmp = NULL;
	int len = 0;
	u8 row_num = 0;

	TOUCH_TRACE_FUNC();

	if ((pkt->log_info & 0x7) == 0x1) {
		pkt->element_sz = 0;
		pkt->row_sz = 0;
		return -EIO;
	}

	switch (pkt->log_info >> 4) {
	case LOG_TYPE_U08:
	case LOG_TYPE_S08:
		len = pkt->element_sz;
		break;
	case LOG_TYPE_U16:
	case LOG_TYPE_S16:
		len = pkt->element_sz * 2;
		break;
	case LOG_TYPE_U32:
	case LOG_TYPE_S32:
		len = pkt->element_sz * 4;
		break;
	default:
		dev_err(&client->dev, "invalied log type\n");
		return -EIO;
	}

	tmp = ts->buf + sizeof(struct mms_log_pkt);
	row_num = pkt->row_sz ? pkt->row_sz : 1;

	while (row_num--) {
		mms_i2c_read(client, MMS_UNIVERSAL_RESULT, tmp, len);
		tmp += len;
	}

	return 0;
}

static int mms_get_packet(struct i2c_client *client)
{
	struct mms_data *ts = get_touch_handle_(client);
	u8 sz = 0;

	TOUCH_TRACE_FUNC();

	if (mms_i2c_read(client, MMS_EVENT_PKT_SZ, &sz, 1) < 0)
		return -EIO;

	if (sz == 0) {
		TOUCH_ERR_MSG("mms_get_packet sz=0 \n");
		return 0;
	}

	memset(ts->buf, 0, sizeof(struct mms_log_pkt));

	if (mms_i2c_read(client, MMS_INPUT_EVENT, ts->buf, sz) < 0)
		return -EIO;

	return (int) sz;
}


static int mms_get_data(struct i2c_client *client, struct touch_data *data)
{
	struct mms_data *ts = get_touch_handle_(client);
	int sz = 0;
	u8 event_type;

	TOUCH_TRACE_FUNC();

	sz = mms_get_packet(client);
	if (sz == 0)
		return 0;
	if ((sz) < 0)
		return -EIO;

	event_type = ts->buf[0] & 0xf;

	if (event_type >= 0x1 && event_type <= 0xa) {
		if (mms_touch_event(client, data, ts->buf, sz) < 0)
			goto err_event_type;
	} else if (event_type == MMS_NOTIFY_EVENT) {
		TOUCH_INFO_MSG("TSP mode changed %d\n", ts->buf[0]);
		return 0;
	} else if (event_type == MMS_ERROR_EVENT) {
		goto mms_error_event;
	} else if (event_type == MMS_LOG_EVENT) {
		if (mms_log_event(client, ts) < 0)
			goto err_event_type;
	} else {
		TOUCH_ERR_MSG("Unkown, event type %d\n", event_type);
		goto err_event_type;
	}

	return 0;

err_event_type:
	return -EIO;
mms_error_event:
	return -ENXIO;
}

static int mms_sleep(struct i2c_client *client)
{
	return 0;
}

static int mms_wake(struct i2c_client *client)
{
	return 0;
}

static int mms_power(struct i2c_client* client, int power_ctrl)
{
	struct mms_data* ts = get_touch_handle_(client);
	static char power_state = 0;
	int i = 0;

	TOUCH_TRACE_FUNC();

	TOUCH_POWER_MSG("power_ctrl = %d\n", power_ctrl);

	if (power_state == power_ctrl) {
		return 0;
	}

	switch (power_ctrl) {
	case POWER_OFF:
		if(ts->pdata->ic_type == MMS100S){
			i2c_smbus_write_byte_data(client, MMS_POWER_CONTROL, 1);

			msleep(50);
		}

		i = TOUCH_PWR_NUM-1;
		do {
			if (ts_pwr[i].type == 1) {
				if (!strncmp(ts_pwr[i].name, "low", strlen("low"))) {
					gpio_direction_output(ts_pwr[i].value, 1);
					TOUCH_POWER_MSG("power[%d]: gpio[%d] set 1", i, ts_pwr[i].value);
				} else {
					gpio_direction_output(ts_pwr[i].value, 0);
					TOUCH_POWER_MSG("power[%d]: gpio[%d] set 0", i, ts_pwr[i].value);
				}
			} else if (ts_pwr[i].type == 2) {
				if (ts->vdd_regulator[i] != NULL && !IS_ERR(ts->vdd_regulator[i])) {
					regulator_disable(ts->vdd_regulator[i]);
					/*
					TOUCH_POWER_MSG("power[%d]: regulator[%s] disabled", i, 
							rdev_get_name(ts->vdd_regulator[i]->rdev));
					*/
				}
			}
		} while(--i >= 0);
		break;

	case POWER_ON:
		i = 0;
		do {
			if (ts_pwr[i].type == 1) {
				if (!strncmp(ts_pwr[i].name, "low", strlen("low"))) {
					gpio_direction_output(ts_pwr[i].value, 0);
					TOUCH_POWER_MSG("power[%d]: gpio[%d] set 0", i, ts_pwr[i].value);
				} else {
					gpio_direction_output(ts_pwr[i].value, 1);
					TOUCH_POWER_MSG("power[%d]: gpio[%d] set 1", i, ts_pwr[i].value);
				}
			} else if (ts_pwr[i].type == 2) {
				if (ts->vdd_regulator[i] != NULL && !IS_ERR(ts->vdd_regulator[i])) {
					regulator_enable(ts->vdd_regulator[i]);
					/*
					TOUCH_POWER_MSG("power[%d]: regulator[%s] enabled", i, 
							rdev_get_name(ts->vdd_regulator[i]->rdev));
					*/
				}
			}
		} while(++i < TOUCH_PWR_NUM);
		break;

	case POWER_SLEEP:
		if (mms_sleep(client))
			return -EIO;
		break;

	case POWER_WAKE:
		if (mms_wake(client))
			return -EIO;
		break;

	default:
		return -EIO;
		break;
	}

	power_state = power_ctrl;

	return 0;
}

int mms_power_ctrl(struct i2c_client* client, int power_ctrl)
{
	int result =0;
	TOUCH_INFO_MSG("%s : %d \n", __func__, power_ctrl);

	result = mms_power(client, power_ctrl);

	return result;
}
EXPORT_SYMBOL(mms_power_ctrl);

static int mms_firmware_img_parse_show(const char *image_bin, char *show_buf, int ret)
{
	struct mms_bin_hdr *fw_hdr = NULL;
	struct mms_fw_img *img = NULL;
	int offset = 0;
	int i = 0;

	fw_hdr = (struct mms_bin_hdr *) image_bin;
	offset = sizeof(struct mms_bin_hdr);

	ret += sprintf(show_buf + ret, "mms_fw_hdr:\n");
	ret += sprintf(show_buf + ret, "\ttag[%c%c%c%c%c%c%c%c]\n",
			fw_hdr->tag[0], fw_hdr->tag[1], fw_hdr->tag[2], fw_hdr->tag[3],
			fw_hdr->tag[4], fw_hdr->tag[5], fw_hdr->tag[6], fw_hdr->tag[7]);
	ret += sprintf(show_buf + ret, "\tcore_version[0x%02x]\n", fw_hdr->core_version);
	ret += sprintf(show_buf + ret, "\tsection_num[%d]\n", fw_hdr->section_num);
	ret += sprintf(show_buf + ret, "\tcontains_full_binary[%d]\n", fw_hdr->contains_full_binary);
	ret += sprintf(show_buf + ret, "\tbinary_offset[%d (0x%04x)]\n", fw_hdr->binary_offset, fw_hdr->binary_offset);
	ret += sprintf(show_buf + ret, "\tbinary_length[%d]\n", fw_hdr->binary_length);

	for (i = 0; i < fw_hdr->section_num; i++) {
		img = (struct mms_fw_img *) (image_bin + offset);
		ret += sprintf(show_buf + ret, "mms_fw_hdr[%d:%s]:\n", i, section_name[img->type]);
		ret += sprintf(show_buf + ret, "\ttype[%d],version[0x%02X],page[%02d-%02d],offset[%d],length[%d]\n", 
				img->type, img->version, img->start_page, img->end_page, img->offset, img->length);
		offset += sizeof(struct mms_fw_img);
	}

	return ret;
}

static int mms_fw_upgrade(struct i2c_client* client, struct touch_fw_info *info)
{
	struct mms_data *ts = get_touch_handle_(client);
	int ret = 0;

	TOUCH_TRACE_FUNC();

	if (info->fw) {
		if(ts->pdata->ic_type == MMS100A)
			ret = mms_100a_fw_upgrade(ts, info);
		else
			ret = mms_100s_isc(ts, info);
	}

	info->force_upgrade = 0;
	return ret;
}

static int mms_ic_ctrl(struct i2c_client *client, u32 code, u32 value)
{
	struct mms_data* ts = (struct mms_data *) get_touch_handle_(client);
	struct ic_ctrl_param *param = (struct ic_ctrl_param *) value;
	int ret = 0;
	char *buf = NULL;

	TOUCH_TRACE_FUNC();

	switch (code) {
	case IC_CTRL_FIRMWARE_IMG_SHOW:
		ret = mms_firmware_img_parse_show((const char *) param->v1, (char *) param->v2, param->v3);
		break;

	case IC_CTRL_INFO_SHOW:
		mms_get_ic_info(ts, NULL);
		buf = (char *) param->v1;
		if(buf) {
			ret += sprintf(buf+ret, "Firmware Version : %d.%02d \n", (ts->ts_section[2].version&0x80?1:0), ts->ts_section[2].version&0x7F);
			ret += sprintf(buf+ret, "Boot:0x%X  Core:0x%X  Config:0x%X \n", ts->ts_section[0].version, ts->ts_section[1].version, ts->ts_section[2].version);
			ret += sprintf(buf+ret, "FW Product : %s \n", ts->module.product_code);
			if (ts_pdata->panel_id != 0xFF)
				ret += sprintf(buf+ret, "Panel ID : %d [%c] \n", ts_pdata->panel_id, ts_pdata->panel_type[ts_pdata->panel_id]);
			ret += sprintf(buf+ret, "Num of Channel. TX:%d RX:%d KEY:%d\n", ts->dev.tx_ch_num, ts->dev.rx_ch_num, ts->dev.key_num);
		}
		break;

	case IC_CTRL_TESTMODE_VERSION_SHOW:
		mms_get_ic_info(ts, NULL);
		buf = (char *) param->v1;
		if(buf) {
			ret += sprintf(buf+ret, "%d.%02d(0x%X, 0x%X, 0x%X, %s)\n",
					(ts->ts_section[2].version&0x80?1:0), ts->ts_section[2].version&0x7F, ts->ts_section[0].version, ts->ts_section[1].version, ts->ts_section[2].version, ts->module.product_code);
		}
		break;

	case IC_CTRL_SAVE_IC_INFO:
		mms_get_ic_info(ts, NULL);
		break;
	}
	return ret;
}

static int mms_reg_control_store(struct i2c_client *client, const char *buf)
{
	struct mms_data *ts = (struct mms_data *) get_touch_handle_(client);
	int cmd = 0, ret = 0, reg_addr = 0, length = 0, i = 0;
	uint8_t reg_buf[100] = {0};

	mms_get_ic_info(ts, NULL);

	if (sscanf(buf, "%d, 0x%x, %d", &cmd, &reg_addr, &length) != 3)
		return -EINVAL;
	switch (cmd) {
		case 1:
			reg_buf[0] = reg_addr;
			ret = i2c_master_send(ts->client, reg_buf, 1);
			if (ret < 0) {
				TOUCH_INFO_MSG("i2c master send fail\n");
				break;
			}
			ret = i2c_master_recv(ts->client, reg_buf, length);
			if (ret < 0) {
				TOUCH_INFO_MSG("i2c master recv fail\n");
				break;
			}
			for (i = 0; i < length; i++) {
				TOUCH_INFO_MSG("0x%x", reg_buf[i]);
			}
			TOUCH_INFO_MSG("\n 0x%x register read done\n", reg_addr);
			break;
		case 2:
			reg_buf[0] = reg_addr;
			reg_buf[1] = length;
			ret = i2c_master_send(ts->client, reg_buf, 2);
			if (ret < 0) {
				TOUCH_INFO_MSG("i2c master send fail\n");
				break;
			}
			TOUCH_INFO_MSG("\n 0x%x register write done\n", reg_addr);
			break;
		default:
			TOUCH_INFO_MSG("usage: echo [1(read)|2(write)], [reg address], [length|value] > reg_control\n");
			TOUCH_INFO_MSG("  - Register Set or Read\n");
			break;
	}
	return 0;
}

static int mms_fx_control_store(struct i2c_client *client, const char *buf)
{
	struct mms_data *ts = (struct mms_data *) get_touch_handle_(client);
	static int fx_state = 0;
	int	ret = 0;
	int cmd = 0;
	if (sscanf(buf, "%d", &cmd) != 1)
		return -EINVAL;

	switch (cmd) {
	case 0:
		if (fx_state == 0) {
			TOUCH_INFO_MSG("already no fx mode\n");
			break;
		}
		if (ts->pdata->panel_on == 0) {
			TOUCH_INFO_MSG("try again after power on\n");
			break;
		}
		fx_state = 0;

		enable_irq(ts->client->irq);
		TOUCH_INFO_MSG("enable irq\n");

		ts_pwr[1].type = 0;
		ret = mms_power(client, POWER_ON);
		ts_pwr[1].type = 2;
		if (ret < 0) {
			TOUCH_INFO_MSG("Regulator vdd enable failed retval = %d\n", ret);
		}else{
			TOUCH_INFO_MSG("regulator_enable(VDD) \n");
		}

		msleep(30);

		mms_set_gpio_mode(ts->pdata, GPIOMODE_FX_END);
		break;
	case 1:
		if (fx_state == 1) {
			TOUCH_INFO_MSG("already fx mode\n");
			break;
		}
		if (ts->pdata->panel_on == 0) {
			TOUCH_INFO_MSG("try again after power on\n");
			break;
		}
		fx_state = 1;

		disable_irq_nosync(ts->client->irq);
		TOUCH_INFO_MSG("disable irq\n");

		ts_pwr[1].type = 0;
		ret = mms_power(client, POWER_OFF);
		ts_pwr[1].type = 2;
		TOUCH_INFO_MSG("regulator_disable(VDD) \n");
		mms_set_gpio_mode(ts->pdata, GPIOMODE_FX_START);
		break;
	default:
		TOUCH_INFO_MSG("usage: echo [0|1] > control\n");
		break;
	}
	return 0;
}


static int get_limit(struct mms_data *ts, char* breakpoint, int *limit_data, int *limit_jitter)
{
	int fd = 0;
	char *fname = "/data/mms_limit.txt";
	int p = 0;
	int q = 0;
	int cipher = 1;
	int r = 0;
	const struct firmware *fwlimit = NULL;
	int ret = 0;
	char* line = NULL;

	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);

	line = kzalloc(10240, GFP_KERNEL);
	if (line == NULL) {
		TOUCH_INFO_MSG("failed to allocate line\n");
		ret = -1;
		goto exit;
	}

	fd = sys_open(fname, O_RDONLY, 0);
	if ( fd < 0 ) {
		if (ts->pdata->panel_spec == NULL) {
			TOUCH_INFO_MSG("panel_spec_file name is null\n");
			ret = -1;
			goto exit;
		} else if (request_firmware(&fwlimit, ts->pdata->panel_spec, &ts->client->dev) >= 0) {
			if (fwlimit->size > 10240) {
				ret = -1;
				goto exit;
			}
			strcpy(line, fwlimit->data);
			ret = 0;
		} else {
			TOUCH_INFO_MSG("failed to request limit ihex");
			ret = -1;
			goto exit;
		}
	} else {
		sys_read(fd, line, 10240);
		ret = 1;
	}

	q = strstr(line, breakpoint) - line;
	if (q < 0) {
		TOUCH_INFO_MSG("failed to find breakpoint. The limit file is wrong");
		ret = -1;
		goto exit;
	}

	if (limit_data == NULL)
		goto get_jitter_limit;

	memset(limit_data, 0, (ts->dev.rx_ch_num * ts->dev.tx_ch_num + ts->dev.key_num) * 4);
	while(1) {
		if (line[q] == ',') {
			cipher = 1;
			for (p = 1; (line[q - p] >= '0') && (line[q - p] <= '9'); p++) {
				limit_data[r] += ((line[q-p] - '0') * cipher);
				cipher *= 10;
			}
			r++;
		}
		q++;

		if(line[q] == '}') {
			if(r == ts->dev.rx_ch_num * ts->dev.tx_ch_num){
				ret = -2;
				goto exit;
			}
			ret = -3;
			goto exit;
		}
		if (r == ts->dev.rx_ch_num * ts->dev.tx_ch_num + ts->dev.key_num) {
			break;
		}
	}

	if(line)
		kfree(line);

	if (fd >= 0)
		sys_close(fd);

	set_fs(old_fs);

	if (fwlimit)
		release_firmware(fwlimit);

	return ret;

get_jitter_limit:
	cipher = 1;
	*limit_jitter = 0;
	while(1) {
		if ((line[q] >= '0') && (line[q] <= '9')) {
			*limit_jitter = (*limit_jitter) * cipher + (line[q] - '0');
			cipher *= 10;
			if (p == 0)
				p = q - 1;
		} else if (p != 0) {
			break;
		}
		q++;
	}
	if (line[p] == '-')
		*limit_jitter *= -1;

exit :
	if(line)
		kfree(line);

	if (fd >= 0)
		sys_close(fd);

	set_fs(old_fs);

	if (fwlimit)
		release_firmware(fwlimit);

	return ret;
}

#ifdef SENSING_TEST
static int mms_sensing_test_store(struct i2c_client *client, const char *buf)
{
	int cmd = 2;

	if(sscanf(buf, "%d", &cmd) != 1)
		return -EINVAL;

	switch (cmd) {
	case 0:
		TOUCH_INFO_MSG("Stop Writing\n");
		sensing_test = 0;
		break;
	case 1:
		TOUCH_INFO_MSG("Start Writing \n");
		sensing_test = 1;
		break;
	default:
		TOUCH_INFO_MSG("usage: echo [0|1] > sensing_test\n");
		TOUCH_INFO_MSG("  - 0: Stop Writing\n");
		TOUCH_INFO_MSG("  - 1: Start Writing\n");
		break;
	}
	return 0;
}
#endif

static ssize_t mms_chstatus_show(struct i2c_client *client, char *buf)
{
	struct mms_data *ts = (struct mms_data *) get_touch_handle_(client);
	char c = '-';
	int len=0, i=0, j=0, t=0;
	uint8_t write_buf[5] = {0};
	uint8_t read_buf[80] = {0};
	uint8_t read_size = 0;
	uint16_t chstatus = 0;
	int flag = 0;
	int count = 0;
	int *error_point = NULL;
	int *chstatus_max = NULL;
	int *chstatus_min = NULL;
	int ret = 0;
	int alloc_flag = 0;

	mms_get_ic_info(ts, NULL);
	ts->pdata->selfdiagnostic_state[0] = 1;

	if ((ts->dev.tx_ch_num > MMS_MAX_TX_NUM) || (ts->dev.rx_ch_num > MMS_MAX_RX_NUM) || (ts->dev.key_num > MMS_MAX_KEY_NUM)) {
		ts->pdata->selfdiagnostic_state[0] = 0;
		TOUCH_INFO_MSG("error. exceed max num of ch.\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "error. exceed max num of ch.\n");
		return len;
	}

	error_point = kzalloc(sizeof(int) * (ts->dev.rx_ch_num * ts->dev.tx_ch_num + ts->dev.key_num), GFP_KERNEL);
	if (error_point == NULL) {
		TOUCH_INFO_MSG("failed to allocate error_point\n");
		alloc_flag = -1;
	}

	chstatus_max = kzalloc(sizeof(int) * (ts->dev.rx_ch_num * ts->dev.tx_ch_num + ts->dev.key_num), GFP_KERNEL);
	if (chstatus_max == NULL) {
		TOUCH_INFO_MSG("failed to allocate chstatus_max\n");
		alloc_flag = -1;
	}

	chstatus_min = kzalloc(sizeof(int) * (ts->dev.rx_ch_num * ts->dev.tx_ch_num + ts->dev.key_num), GFP_KERNEL);
	if (chstatus_min == NULL) {
		TOUCH_INFO_MSG("failed to allocate chstatus_min\n");
		alloc_flag = -1;
	}

	if (alloc_flag != -1) {
		ret = get_limit(ts, "chstatus_max", chstatus_max, NULL);
		if ((ret != -1) && (get_limit(ts, "chstatus_min", chstatus_min, NULL) == -1))
			ret = -1;
	}

	TOUCH_INFO_MSG("disable_irq\n");
	disable_irq(ts->client->irq);

	write_buf[0] = MMS_UNIVERSAL_CMD;
	write_buf[1] = MMS_ENTER_TEST_MODE;
	i2c_master_send(ts->client, write_buf, 2);

	do{
		while (gpio_get_value(ts->pdata->int_pin)) {
			flag++;
			if (flag == 30) {
				flag = 0;
				break;
			}
			msleep(100);
		}
		flag = 0;

		write_buf[0] = 0x0F;
		i2c_master_send(ts->client, write_buf, 1);
		i2c_master_recv(ts->client, read_buf, 1);


		write_buf[0] = 0x10;
		i2c_master_send(ts->client, write_buf, 1);
		i2c_master_recv(ts->client, read_buf, 1);
		TOUCH_INFO_MSG("Maker is %x\n", read_buf[0]);
		count++;
	}while(read_buf[0]!=0x0C&&count!=10);

	write_buf[0] = MMS_UNIVERSAL_CMD;
	write_buf[1] = MMS_TEST_CHSTATUS;
	i2c_master_send(ts->client, write_buf, 2);


	while (gpio_get_value(ts->pdata->int_pin)) {
		flag++;
		if (flag == 30) {
			flag = 0;
			break;
		}
		msleep(100);
	}
	flag = 0;

	write_buf[0] = MMS_UNIVERSAL_RESULT_SIZE;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, &read_size, 1);
	TOUCH_INFO_MSG("read size = %d\n", read_size);
	write_buf[0] = MMS_UNIVERSAL_RESULT;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, read_buf, read_size);
	TOUCH_INFO_MSG("Chstatus TEST =%d \n", read_buf[0]);

	len = snprintf(buf, PAGE_SIZE, "\n<< SHOW CHANNEL STATUS >>\n");
	if(ts->pdata->panel_on == POWER_OFF){
		len += snprintf(buf + len, PAGE_SIZE - len, "************************\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "*** LCD STATUS : OFF ***\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "************************\n");
	}else if(ts->pdata->panel_on == POWER_ON){
		len += snprintf(buf + len, PAGE_SIZE - len, "************************\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "*** LCD STATUS : O N ***\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "************************\n");
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "========================\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "%5c", c);
	for (j = 0; j < ts->dev.tx_ch_num; j++)
		len += snprintf(buf + len, PAGE_SIZE - len, "%5d", j);
	len += snprintf(buf + len, PAGE_SIZE - len, "\n-----------------------------------------------");
	len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------");
	len += snprintf(buf + len, PAGE_SIZE - len, "------------------------\n");

	for (i = 0; i < ts->dev.rx_ch_num ; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len, "%2d : ", i);
		for (j = 0; j < ts->dev.tx_ch_num; j++) {
			write_buf[0] = MMS_UNIVERSAL_CMD;
			write_buf[1] = MMS_GET_PIXEL_CHSTATUS;
			write_buf[2] = j;
			write_buf[3] = i;
			i2c_master_send(ts->client, write_buf, 4);
			while (gpio_get_value(ts->pdata->int_pin)) {
				flag++;
				if (flag == 100) {
					flag = 0;
					break;
				}
				udelay(100);
			}
			flag = 0;
			write_buf[0] = MMS_UNIVERSAL_RESULT_SIZE;
			i2c_master_send(ts->client, write_buf, 1);
			i2c_master_recv(ts->client, &read_size, 1);
			write_buf[0] = MMS_UNIVERSAL_RESULT;
			i2c_master_send(ts->client, write_buf, 1);
			i2c_master_recv(ts->client, read_buf, read_size);

			chstatus = read_buf[1];
			chstatus = ((chstatus << 8) | read_buf[0]);
			if ((alloc_flag != -1) && (ret != -1)) {
					if ((chstatus > chstatus_max[i * ts->dev.tx_ch_num + j]) || (chstatus < chstatus_min[i * ts->dev.tx_ch_num + j])) {
						error_point[i * ts->dev.tx_ch_num + j] = 1;
						ts->pdata->selfdiagnostic_state[0] = 0;
					}
			}
			len += snprintf(buf + len, PAGE_SIZE - len, "%5d", chstatus);
		}
		len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	}
	/* read touch key chstatus */
	len += snprintf(buf + len, PAGE_SIZE - len, "key: ");

	write_buf[0] = MMS_UNIVERSAL_CMD;
	write_buf[1] = MMS_KEY_CHSTATUS;
	write_buf[2] = 0xff; //KEY CH.
	write_buf[3] = 0; //Dummy Info
	i2c_master_send(ts->client, write_buf, 4);
	while (gpio_get_value(ts->pdata->int_pin)) {
		flag++;

		if (flag == 100) {
			flag = 0;
			break;
		}
		udelay(100);
	}
	flag = 0;
	write_buf[0] = MMS_UNIVERSAL_RESULT_SIZE;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, &read_size, 1);
	write_buf[0] = MMS_UNIVERSAL_RESULT;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, read_buf, read_size);
	for (t = 0; t < ts->dev.key_num ; t++) //Model Dependent
	{
		chstatus = read_buf[2 * t] | (read_buf[2 * t + 1] << 8);
		if((alloc_flag != -1) && (ret != -1)){
			if((chstatus > chstatus_max[ts->dev.rx_ch_num* ts->dev.tx_ch_num + t]) || (chstatus < chstatus_min[ts->dev.rx_ch_num* ts->dev.tx_ch_num + t])){
				error_point[ts->dev.rx_ch_num* ts->dev.tx_ch_num + t] = 1;
				ts->pdata->selfdiagnostic_state[0] = 0;
			}
		}
		len += snprintf(buf + len, PAGE_SIZE - len, "%5d", chstatus);
	}

	len += snprintf(buf + len, PAGE_SIZE - len, "\n===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "========================\n");
	write_buf[0] = MMS_UNIVERSAL_CMD;
	write_buf[1] = MMS_UNIVERSAL_CMD_EXIT;
	i2c_master_send(ts->client, write_buf, 2);

	while (gpio_get_value(ts->pdata->int_pin)) {
		flag++;
		if (flag == 30) {
			flag = 0;
			break;
		}
		msleep(100);
	}
	flag = 0;

	write_buf[0] = MMS_UNIVERSAL_RESULT_SIZE;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, &read_size, 1);

	write_buf[0] = MMS_UNIVERSAL_RESULT;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, read_buf, read_size);

	if (alloc_flag == -1) {
		len += snprintf(buf + len, PAGE_SIZE - len, "<<No spec examination>>");
	} else if (ret == -1) {
		len += snprintf(buf + len, PAGE_SIZE - len, "<<No spec file>>");
	} else if (ret == -2) {
		len += snprintf(buf + len, PAGE_SIZE - len, "<<Error spec file, please check key limit>>");
	} else if (ret == -3) {
		len += snprintf(buf + len, PAGE_SIZE - len, "<<Error spec file, please check the number of channel >>");
	} else {
		len += snprintf(buf + len, PAGE_SIZE - len, "Result = %s\n", ts->pdata->selfdiagnostic_state[0] == 1 ? "PASS" : "FAIL");
		if (ts->pdata->selfdiagnostic_state[0] == 0) {
			len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
			len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
			len += snprintf(buf + len, PAGE_SIZE - len, "========================\n");
			len += snprintf(buf + len, PAGE_SIZE - len, "%5c", c);
			for (j = 0; j < ts->dev.tx_ch_num; j++)
				len += snprintf(buf + len, PAGE_SIZE - len, "%5d", j);
			len += snprintf(buf + len, PAGE_SIZE - len, "\n");
			len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------");
			len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------");
			len += snprintf(buf + len, PAGE_SIZE - len, "------------------------\n");
			for (i = 0; i < ts->dev.rx_ch_num ; i++) {
				len += snprintf(buf + len, PAGE_SIZE - len, "%2d : ", i);
				for (j = 0; j < ts->dev.tx_ch_num; j++) {
					if (error_point[i * ts->dev.tx_ch_num + j] == 1) {
						len += snprintf(buf + len, PAGE_SIZE - len, "%5c", 'X');
					} else {
						len += snprintf(buf + len, PAGE_SIZE - len, "%5c", ' ');
					}
				}
				len += snprintf(buf + len, PAGE_SIZE - len, "\n");
			}
			len += snprintf(buf + len, PAGE_SIZE - len, "key:");
			for (t = 0; t < ts->dev.tx_ch_num; t++) {
				if (error_point[ts->dev.rx_ch_num * ts->dev.tx_ch_num + t] == 1) {
					len += snprintf(buf + len, PAGE_SIZE - len, "%5c", 'X');
				} else {
					len += snprintf(buf + len, PAGE_SIZE - len, "%5c", ' ');
				}
			}
		}
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "\n===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "========================\n");

	TOUCH_INFO_MSG("enable_irq\n");
	enable_irq(ts->client->irq);

	if (error_point) {
		kfree(error_point);
	}
	if (chstatus_max) {
		kfree(chstatus_max);
	}
	if (chstatus_min) {
		kfree(chstatus_min);
	}

	return len;
}

static ssize_t mms_rawdata_show(struct i2c_client *client, char *buf)
{
	struct mms_data *ts = (struct mms_data *) get_touch_handle_(client);
	char c = '-';
	int len=0, i=0, j=0, t=0;
	uint8_t write_buf[5] = {0};
	uint8_t read_buf[80] = {0};
	uint8_t read_size = 0;
	uint16_t rawdata = 0;
	int flag = 0;
	int count = 0;
	int* error_point = NULL;
	int* raw_data_max = NULL;
	int* raw_data_min = NULL;
	int ret = 0;
	int alloc_flag = 0;

	mms_get_ic_info(ts, NULL);
	ts->pdata->selfdiagnostic_state[1] = 1;

	if ((ts->dev.tx_ch_num > MMS_MAX_TX_NUM) || (ts->dev.rx_ch_num > MMS_MAX_RX_NUM) || (ts->dev.key_num > MMS_MAX_KEY_NUM)) {
		ts->pdata->selfdiagnostic_state[1] = 0;
		TOUCH_INFO_MSG("error. exceed max num of ch.\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "error. exceed max num of ch.\n");
		return len;
	}

	error_point = kzalloc(sizeof(int) * (ts->dev.rx_ch_num * ts->dev.tx_ch_num + ts->dev.key_num), GFP_KERNEL);
	if (error_point == NULL) {
		TOUCH_INFO_MSG("failed to allocate error_point\n");
		alloc_flag = -1;
	}

	raw_data_max = kzalloc(sizeof(int) * (ts->dev.rx_ch_num * ts->dev.tx_ch_num + ts->dev.key_num), GFP_KERNEL);
	if (raw_data_max == NULL) {
		TOUCH_INFO_MSG("failed to allocate raw_data_max\n");
		alloc_flag = -1;
	}

	raw_data_min = kzalloc(sizeof(int) * (ts->dev.rx_ch_num * ts->dev.tx_ch_num + ts->dev.key_num), GFP_KERNEL);
	if (raw_data_min == NULL) {
		TOUCH_INFO_MSG("failed to allocate raw_data_min\n");
		alloc_flag = -1;
	}
	if (alloc_flag != -1) {
		ret = get_limit(ts, "raw_data_max", raw_data_max, NULL);
		if ((ret != -1) && (get_limit(ts, "raw_data_min", raw_data_min, NULL) == -1))
			ret = -1;
	}

	TOUCH_INFO_MSG("disable_irq\n");
	disable_irq(ts->client->irq);

	write_buf[0] = MMS_UNIVERSAL_CMD;
	write_buf[1] = MMS_ENTER_TEST_MODE;
	i2c_master_send(ts->client, write_buf, 2);

	do{
		while (gpio_get_value(ts->pdata->int_pin)) {
			flag++;
			if (flag == 30) {
				flag = 0;
				break;
			}
			msleep(100);
		}
		flag = 0;

		write_buf[0] = 0x0F;
		i2c_master_send(ts->client, write_buf, 1);
		i2c_master_recv(ts->client, read_buf, 1);

		write_buf[0] = 0x10;
		i2c_master_send(ts->client, write_buf, 1);
		i2c_master_recv(ts->client, read_buf, 1);
		TOUCH_INFO_MSG("Maker is %x\n", read_buf[0]);
		count++;
	}while(read_buf[0]!=0x0C&&count!=10);

	write_buf[0] = MMS_UNIVERSAL_CMD;
	write_buf[1] = MMS_TEST_RAWDATA;
	i2c_master_send(ts->client, write_buf, 2);

	while (gpio_get_value(ts->pdata->int_pin)) {
		flag++;
		if (flag == 30) {
			flag = 0;
			break;
		}
		msleep(100);
	}
	flag = 0;

	write_buf[0] = MMS_UNIVERSAL_RESULT_SIZE;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, &read_size, 1);
	TOUCH_INFO_MSG("read size = %d\n", read_size);
	write_buf[0] = MMS_UNIVERSAL_RESULT;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, read_buf, read_size);
	TOUCH_INFO_MSG("RAWDATA TEST =%d \n", read_buf[0]);

	len = snprintf(buf, PAGE_SIZE, "\n<< SHOW RAWDATA TEST >>\n");
	if(ts->pdata->panel_on == POWER_OFF){
		len += snprintf(buf + len, PAGE_SIZE - len, "************************\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "*** LCD STATUS : OFF ***\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "************************\n");
	}else if(ts->pdata->panel_on == POWER_ON){
		len += snprintf(buf + len, PAGE_SIZE - len, "************************\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "*** LCD STATUS : O N ***\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "************************\n");
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "========================\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "%5c", c);
	for (j = 0; j < ts->dev.tx_ch_num; j++)
		len += snprintf(buf + len, PAGE_SIZE - len, "%5d", j);
	len += snprintf(buf + len, PAGE_SIZE - len, "\n-----------------------------------------------");
	len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------");
	len += snprintf(buf + len, PAGE_SIZE - len, "------------------------\n");

	for (i = 0; i < ts->dev.rx_ch_num ; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len, "%2d : ", i);
		for (j = 0; j < ts->dev.tx_ch_num; j++) {
			write_buf[0] = MMS_UNIVERSAL_CMD;
			write_buf[1] = MMS_GET_PIXEL_RAWDATA;
			write_buf[2] = j;
			write_buf[3] = i;
			i2c_master_send(ts->client, write_buf, 4);
			while (gpio_get_value(ts->pdata->int_pin)) {
				flag++;
				if (flag == 100) {
					flag = 0;
					break;
				}
				udelay(100);
			}
			flag = 0;
			write_buf[0] = MMS_UNIVERSAL_RESULT_SIZE;
			i2c_master_send(ts->client, write_buf, 1);
			i2c_master_recv(ts->client, &read_size, 1);
			write_buf[0] = MMS_UNIVERSAL_RESULT;
			i2c_master_send(ts->client, write_buf, 1);
			i2c_master_recv(ts->client, read_buf, read_size);

			rawdata = read_buf[1];
			rawdata = ((rawdata << 8) | read_buf[0]);
			if ((alloc_flag != -1) && (ret != -1)) {
				if ((rawdata > raw_data_max[i * ts->dev.tx_ch_num + j]) || (rawdata < raw_data_min[i * ts->dev.tx_ch_num + j])) {
					error_point[i * ts->dev.tx_ch_num + j] = 1;
					ts->pdata->selfdiagnostic_state[1] = 0;
				}
			}
			len += snprintf(buf + len, PAGE_SIZE - len, "%5d", rawdata);
		}
		len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	}
	/* read touch key rawdata */
	len += snprintf(buf + len, PAGE_SIZE - len, "key: ");

	write_buf[0] = MMS_UNIVERSAL_CMD;
	write_buf[1] = MMS_KEY_RAWDATA;
	write_buf[2] = 0xff; //KEY CH.
	write_buf[3] = 0; //Dummy Info
	i2c_master_send(ts->client, write_buf, 4);
	while (gpio_get_value(ts->pdata->int_pin)) {
		flag++;

		if (flag == 100) {
			flag = 0;
			break;
		}
		udelay(100);
	}
	flag = 0;
	write_buf[0] = MMS_UNIVERSAL_RESULT_SIZE;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, &read_size, 1);
	write_buf[0] = MMS_UNIVERSAL_RESULT;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, read_buf, read_size);
	for (t = 0; t < ts->dev.key_num ; t++) //Model Dependent
	{
		rawdata = read_buf[2 * t] | (read_buf[2 * t + 1] << 8);
		if ((rawdata > raw_data_max[ts->dev.rx_ch_num* ts->dev.tx_ch_num + t]) || (rawdata < raw_data_min[ts->dev.rx_ch_num* ts->dev.tx_ch_num + t])) {
			error_point[ts->dev.rx_ch_num* ts->dev.tx_ch_num + t] = 1;
			ts->pdata->selfdiagnostic_state[1] = 0;
		}
		len += snprintf(buf + len, PAGE_SIZE - len, "%5d", rawdata);
	}

	len += snprintf(buf + len, PAGE_SIZE - len, "\n===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "========================\n");
	write_buf[0] = MMS_UNIVERSAL_CMD;
	write_buf[1] = MMS_UNIVERSAL_CMD_EXIT;
	i2c_master_send(ts->client, write_buf, 2);

	while (gpio_get_value(ts->pdata->int_pin)) {
		flag++;
		if (flag == 30) {
			flag = 0;
			break;
		}
		msleep(100);
	}
	flag = 0;

	write_buf[0] = MMS_UNIVERSAL_RESULT_SIZE;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, &read_size, 1);

	write_buf[0] = MMS_UNIVERSAL_RESULT;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, read_buf, read_size);

	if (alloc_flag == -1) {
			len += snprintf(buf + len, PAGE_SIZE - len, "<<No spec examination>>");
	} else if (ret == -1) {
			len += snprintf(buf + len, PAGE_SIZE - len, "<<No spec file>>");
	} else if (ret == -2) {
		len += snprintf(buf + len, PAGE_SIZE - len, "<<Error spec file, please check key limit>>");
	} else if (ret == -3) {
		len += snprintf(buf + len, PAGE_SIZE - len, "<<Error spec file, please check the number of channel >>");
	} else {
		len += snprintf(buf + len, PAGE_SIZE - len, "Result = %s\n", ts->pdata->selfdiagnostic_state[1] == 1 ? "PASS" : "FAIL");
		if (ts->pdata->selfdiagnostic_state[1] == 0) {
			len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
			len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
			len += snprintf(buf + len, PAGE_SIZE - len, "========================\n");
			len += snprintf(buf + len, PAGE_SIZE - len, "%5c", c);
			for (j = 0; j < ts->dev.tx_ch_num; j++)
				len += snprintf(buf + len, PAGE_SIZE - len, "%5d", j);
			len += snprintf(buf + len, PAGE_SIZE - len, "\n");
			len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------");
			len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------");
			len += snprintf(buf + len, PAGE_SIZE - len, "------------------------\n");
			for (i = 0; i < ts->dev.rx_ch_num; i++) {
				len += snprintf(buf + len, PAGE_SIZE - len, "%2d : ", i);
				for (j = 0; j < ts->dev.tx_ch_num; j++) {
					if (error_point[i * ts->dev.tx_ch_num + j] == 1) {
						len += snprintf(buf + len, PAGE_SIZE - len, "%5c", 'X');
					} else {
						len += snprintf(buf + len, PAGE_SIZE - len, "%5c", ' ');
					}
				}
				len += snprintf(buf + len, PAGE_SIZE - len, "\n");
			}
			len += snprintf(buf + len, PAGE_SIZE - len, "key:");
			for (t = 0; t < ts->dev.tx_ch_num; t++) {
				if (error_point[ts->dev.rx_ch_num * ts->dev.tx_ch_num + t] == 1) {
					len += snprintf(buf + len, PAGE_SIZE - len, "%5c", 'X');
				} else {
					len += snprintf(buf + len, PAGE_SIZE - len, "%5c", ' ');
				}
			}
		}
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "\n===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "========================\n");

	TOUCH_INFO_MSG("enable_irq\n");
	enable_irq(ts->client->irq);

	if (error_point) {
		kfree(error_point);
	}
	if (raw_data_max) {
		kfree(raw_data_max);
	}
	if (raw_data_min) {
		kfree(raw_data_min);
	}

	return len;
}

static ssize_t mms_delta_show(struct i2c_client *client, char *buf)
{
	struct mms_data *ts = (struct mms_data *) get_touch_handle_(client);
	char c = '-';
	int len = 0, i = 0, j = 0;
	u8 sz = 0;
	u8 read_buf[40] = {0,};
	u8 write_buf[4] = {0,};
	s16 delta = 0;

	struct i2c_msg msg[] = {
		{
			.addr = ts->client->addr,
			.flags = 0,
			.buf = write_buf,
		},
		{
			.addr = ts->client->addr,
			.flags = 1,
		},
	};

	mms_get_ic_info(ts, NULL);

	if ((ts->dev.tx_ch_num > MMS_MAX_TX_NUM) || (ts->dev.rx_ch_num > MMS_MAX_RX_NUM) || (ts->dev.key_num > MMS_MAX_KEY_NUM)) {
		TOUCH_INFO_MSG("error. exceed max num of ch.\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "error. exceed max num of ch.\n");
		return len;
	}

	TOUCH_INFO_MSG("disable_irq\n");
	disable_irq(ts->client->irq);

	len += snprintf(buf + len, PAGE_SIZE - len, "=============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "========================\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "%5c", c);
	for (j = 0; j < ts->dev.tx_ch_num; j++)
		len += snprintf(buf + len, PAGE_SIZE - len, "%5d", j);
	len += snprintf(buf + len, PAGE_SIZE - len, "\n---------------------------------------------");
	len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------");
	len += snprintf(buf + len, PAGE_SIZE - len, "------------------------\n");

	for(i=0; i<ts->dev.rx_ch_num; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len, "%5d", i);
		for(j = 0; j<ts->dev.tx_ch_num; j++){
			write_buf[0] = MMS_UNIVERSAL_CMD;
			write_buf[1] = MMS_DELTA_SCREEN;
			write_buf[2] = j;
			write_buf[3] = i;
			msg[0].len = 4;
			if(i2c_transfer(ts->client->adapter, &msg[0], 1)!=1){
				TOUCH_INFO_MSG("intensity i2c transfer failed\n");
				return -1;
			}
			sz = i2c_smbus_read_byte_data(ts->client, MMS_UNIVERSAL_RESULT_SIZE);
			write_buf[0] = MMS_UNIVERSAL_RESULT;
			msg[0].len = 1;
			msg[1].len = sz;
			msg[1].buf = read_buf;
			if(i2c_transfer(ts->client->adapter, msg, ARRAY_SIZE(msg))!=ARRAY_SIZE(msg)){
				return -1;
			}
			sz>>=1;
			delta = read_buf[1];
			delta = ((delta<<8)|read_buf[0]);
			len += snprintf(buf + len, PAGE_SIZE - len, "%5d", delta);
		}
		len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	}

	len += snprintf(buf + len, PAGE_SIZE - len, "key: ");
	for(j = 0; j<ts->dev.key_num; j++){
		write_buf[0] = MMS_UNIVERSAL_CMD;
		write_buf[1] = MMS_DELTA_KEY;
		write_buf[2] = j;
		msg[0].len = 4;
		if(i2c_transfer(ts->client->adapter, &msg[0], 1)!=1){
			TOUCH_INFO_MSG("intensity i2c transfer failed\n");
			return -1;
		}
		sz = i2c_smbus_read_byte_data(ts->client, MMS_UNIVERSAL_RESULT_SIZE);
		write_buf[0] = MMS_UNIVERSAL_RESULT;
		msg[0].len = 1;
		msg[1].len = sz;
		msg[1].buf = read_buf;
		if(i2c_transfer(ts->client->adapter, msg, ARRAY_SIZE(msg))!=ARRAY_SIZE(msg)){
			return -1;
		}
		sz>>=1;
		delta = read_buf[1];
		delta = ((delta<<8)|read_buf[0]);
		len += snprintf(buf + len, PAGE_SIZE - len, "%5d", delta);
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "\n=============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "========================\n");

	TOUCH_INFO_MSG("enable_irq\n");
	enable_irq(ts->client->irq);

	return len;
}

static ssize_t mms_jitter_show(struct i2c_client *client, char *buf)
{
	struct mms_data *ts = (struct mms_data *) get_touch_handle_(client);
	char c = '-';
	int len=0, i=0, j=0, t=0;
	uint8_t write_buf[5] = {0};
	uint8_t read_buf[40] = {0};
	uint8_t read_size = 0;
	uint16_t jitter = 0;
	int flag = 0;
	int count = 0;
	int *error_point = NULL;
	int jitter_upper_limit = 1;
	int jitter_low_limit = -1;
	int ret = 0;
	int alloc_flag = 0;

	mms_get_ic_info(ts, NULL);
	ts->pdata->selfdiagnostic_state[2] = 1;

	if ((ts->dev.tx_ch_num > MMS_MAX_TX_NUM) || (ts->dev.rx_ch_num > MMS_MAX_RX_NUM) || (ts->dev.key_num > MMS_MAX_KEY_NUM)) {
		ts->pdata->selfdiagnostic_state[2] = 0;
		TOUCH_INFO_MSG("error. exceed max num of ch.\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "error. exceed max num of ch.\n");
		return len;
	}

	error_point = kzalloc(sizeof(int) * (ts->dev.rx_ch_num * ts->dev.tx_ch_num + ts->dev.key_num), GFP_KERNEL);
	if (error_point == NULL) {
		TOUCH_INFO_MSG("failed to allocate error_point\n");
		alloc_flag = -1;
	}

	if (alloc_flag != -1) {
		ret = get_limit(ts, "jitter_upper_limit", NULL, &jitter_upper_limit);
		if ((ret != -1) && (get_limit(ts, "jitter_low_limit", NULL, &jitter_low_limit) == -1))
			ret = -1;
	}

	TOUCH_INFO_MSG("disable_irq\n");
	disable_irq(ts->client->irq);

	write_buf[0] = MMS_UNIVERSAL_CMD;
	write_buf[1] = MMS_ENTER_TEST_MODE;
	i2c_master_send(ts->client, write_buf, 2);

	do{
		while (gpio_get_value(ts->pdata->int_pin)) {
			flag++;
			if (flag == 30) {
				flag = 0;
				break;
			}
			msleep(100);
		}
		flag = 0;

		write_buf[0] = 0x0F;
		i2c_master_send(ts->client, write_buf, 1);
		i2c_master_recv(ts->client, read_buf, 1);

		write_buf[0] = 0x10;
		i2c_master_send(ts->client, write_buf, 1);
		i2c_master_recv(ts->client, read_buf, 1);
		TOUCH_INFO_MSG("Maker is %x\n", read_buf[0]);
		count++;
	}while(read_buf[0]!=0x0C&&count!=10);

	write_buf[0] = MMS_UNIVERSAL_CMD;
	write_buf[1] = MMS_TEST_JITTER;
	i2c_master_send(ts->client, write_buf, 2);

	while (gpio_get_value(ts->pdata->int_pin)) {
		flag++;
		if (flag == 30) {
			flag = 0;
			break;
		}
		msleep(100);
	}
	flag = 0;

	write_buf[0] = MMS_UNIVERSAL_RESULT_SIZE;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, &read_size, 1);
	TOUCH_INFO_MSG("read size = %d\n", read_size);
	write_buf[0] = MMS_UNIVERSAL_RESULT;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, read_buf, read_size);
	TOUCH_INFO_MSG("JITTER TEST =%d \n", read_buf[0]);

	len = snprintf(buf, PAGE_SIZE, "\n<< SHOW JITTER TEST >>\n");
	if(ts->pdata->panel_on == POWER_OFF){
		len += snprintf(buf + len, PAGE_SIZE - len, "************************\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "*** LCD STATUS : OFF ***\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "************************\n");
	}else if(ts->pdata->panel_on == POWER_ON){
		len += snprintf(buf + len, PAGE_SIZE - len, "************************\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "*** LCD STATUS : O N ***\n");
		len += snprintf(buf + len, PAGE_SIZE - len, "************************\n");
	}

	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "========================\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "%5c", c);
	for (j = 0; j < ts->dev.tx_ch_num; j++)
		len += snprintf(buf + len, PAGE_SIZE - len, "%5d", j);
	len += snprintf(buf + len, PAGE_SIZE - len, "\n-----------------------------------------------");
	len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------");
	len += snprintf(buf + len, PAGE_SIZE - len, "------------------------\n");

	for (i = 0; i < ts->dev.rx_ch_num ; i++) {
		len += snprintf(buf + len, PAGE_SIZE - len, "%2d : ", i);
		for (j = 0; j < ts->dev.tx_ch_num; j++) {
			write_buf[0] = MMS_UNIVERSAL_CMD;
			write_buf[1] = MMS_GET_PIXEL_JITTER;
			write_buf[2] = j;
			write_buf[3] = i;
			i2c_master_send(ts->client, write_buf, 4);
			while (gpio_get_value(ts->pdata->int_pin)) {
				flag++;
				if (flag == 100) {
					flag = 0;
					break;
				}
				udelay(100);
			}
			flag = 0;
			write_buf[0] = MMS_UNIVERSAL_RESULT_SIZE;
			i2c_master_send(ts->client, write_buf, 1);
			i2c_master_recv(ts->client, &read_size, 1);
			write_buf[0] = MMS_UNIVERSAL_RESULT;
			i2c_master_send(ts->client, write_buf, 1);
			i2c_master_recv(ts->client, read_buf, read_size);

			jitter = read_buf[0];
			if ((alloc_flag != -1) && (ret != -1)) {
				if ((jitter > jitter_upper_limit) || (jitter < jitter_low_limit)) {
					error_point[i * ts->dev.tx_ch_num + j] = 1;
					ts->pdata->selfdiagnostic_state[2] = 0;
				}
			}
			len += snprintf(buf + len, PAGE_SIZE - len, "%5d", jitter);
		}
		len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	}
	/* read touch key jitter */
	len += snprintf(buf + len, PAGE_SIZE - len, "key: ");

	write_buf[0] = MMS_UNIVERSAL_CMD;
	write_buf[1] = MMS_KEY_JITTER;
	write_buf[2] = 0xff; //KEY CH.
	write_buf[3] = 0; //Dummy Info
	i2c_master_send(ts->client, write_buf, 4);
	while (gpio_get_value(ts->pdata->int_pin)) {
		flag++;

		if (flag == 100) {
			flag = 0;
			break;
		}
		udelay(100);
	}
	flag = 0;
	write_buf[0] = MMS_UNIVERSAL_RESULT_SIZE;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, &read_size, 1);
	write_buf[0] = MMS_UNIVERSAL_RESULT;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, read_buf, read_size);
	for (t = 0; t < ts->dev.key_num ; t++) //Model Dependent
	{
		//jitter = read_buf[2 * t] | (read_buf[2 * t + 1] << 8);
		jitter = read_buf[0];
		if ((jitter > jitter_upper_limit) || (jitter < jitter_low_limit)) {
			error_point[ts->dev.rx_ch_num* ts->dev.tx_ch_num + t] = 1;
			ts->pdata->selfdiagnostic_state[2] = 0;
		}
		len += snprintf(buf + len, PAGE_SIZE - len, "%5d", jitter);
	}

	len += snprintf(buf + len, PAGE_SIZE - len, "\n===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "========================\n");
	write_buf[0] = MMS_UNIVERSAL_CMD;
	write_buf[1] = MMS_UNIVERSAL_CMD_EXIT;
	i2c_master_send(ts->client, write_buf, 2);

	while (gpio_get_value(ts->pdata->int_pin)) {
		flag++;
		if (flag == 30) {
			flag = 0;
			break;
		}
		msleep(100);
	}
	flag = 0;

	write_buf[0] = MMS_UNIVERSAL_RESULT_SIZE;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, &read_size, 1);

	write_buf[0] = MMS_UNIVERSAL_RESULT;
	i2c_master_send(ts->client, write_buf, 1);
	i2c_master_recv(ts->client, read_buf, read_size);

	if (alloc_flag == -1) {
		len += snprintf(buf + len, PAGE_SIZE - len, "<<No spec examination>>");
	} else if (ret == -1) {
		len += snprintf(buf + len, PAGE_SIZE - len, "<<No spec file>>");
	} else {
		len += snprintf(buf + len, PAGE_SIZE - len, "Result = %s\n", ts->pdata->selfdiagnostic_state[2] == 1 ? "PASS" : "FAIL");

		if (ts->pdata->selfdiagnostic_state[2] == 0) {
			len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
			len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
			len += snprintf(buf + len, PAGE_SIZE - len, "========================\n");
			len += snprintf(buf + len, PAGE_SIZE - len, "%5c", c);
			for (j = 0; j < ts->dev.tx_ch_num; j++)
				len += snprintf(buf + len, PAGE_SIZE - len, "%5d", j);
			len += snprintf(buf + len, PAGE_SIZE - len, "\n");
			len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------");
			len += snprintf(buf + len, PAGE_SIZE - len, "-----------------------------------------------");
			len += snprintf(buf + len, PAGE_SIZE - len, "------------------------\n");
			for (i = 0; i < ts->dev.rx_ch_num ; i++) {
				len += snprintf(buf + len, PAGE_SIZE - len, "%2d : ", i);
				for (j = 0; j < ts->dev.tx_ch_num; j++) {
					if (error_point[i * ts->dev.tx_ch_num + j] == 1) {
						len += snprintf(buf + len, PAGE_SIZE - len, "%5c", 'X');
					} else {
						len += snprintf(buf + len, PAGE_SIZE - len, "%5c", ' ');
					}
				}
				len += snprintf(buf + len, PAGE_SIZE - len, "\n");
			}
			len += snprintf(buf + len, PAGE_SIZE - len, "key:");
			for (t = 0; t < ts->dev.tx_ch_num; t++) {
				if (error_point[ts->dev.rx_ch_num * ts->dev.tx_ch_num + t] == 1) {
					len += snprintf(buf + len, PAGE_SIZE - len, "%5c", 'X');
				} else {
					len += snprintf(buf + len, PAGE_SIZE - len, "%5c", ' ');
				}
			}
		}
	}
	len += snprintf(buf + len, PAGE_SIZE - len, "\n===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "===============================================");
	len += snprintf(buf + len, PAGE_SIZE - len, "========================\n");

	TOUCH_INFO_MSG("enable_irq\n");
	enable_irq(ts->client->irq);

	if (error_point) {
		kfree(error_point);
	}

	return len;
}

static ssize_t mms_self_diagnostic_show(struct i2c_client *client, char *buf)
{
	struct mms_data *ts = (struct mms_data *) get_touch_handle_(client);
	int len = 0;
	char *sd_path = "/data/touch_self_test.txt";
	struct ic_ctrl_param param;

	param.v1 = (u32) buf;


	write_file(sd_path, buf, 1);
	msleep(30);
	mms_chstatus_show(client, buf);
	write_file(sd_path, buf, 0);
	msleep(30);
	mms_rawdata_show(client, buf);
	write_file(sd_path, buf, 0);
	msleep(30);
	mms_jitter_show(client, buf);
	write_file(sd_path, buf, 0);

	len += snprintf(buf + len, PAGE_SIZE - len, "Firmware Version : %d.%02d \n", (ts->ts_section[2].version&0x80?1:0), ts->ts_section[2].version&0x7F);
	len += snprintf(buf + len, PAGE_SIZE - len, "Boot:0x%X Core:0x%X Config:0x%X \n", ts->ts_section[0].version, ts->ts_section[1].version, ts->ts_section[2].version);
	len += snprintf(buf + len, PAGE_SIZE - len, "FW Product : %s \n", ts->module.product_code);
	if (ts_pdata->panel_id != 0xFF)
		len += snprintf(buf + len, PAGE_SIZE - len, "Panel ID : %d [%c] \n", ts_pdata->panel_id, ts_pdata->panel_type[ts_pdata->panel_id]);
	len += snprintf(buf + len, PAGE_SIZE - len, "=======RESULT========\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "Channel Status : %s\n", ts->pdata->selfdiagnostic_state[0] == 1 ? "PASS" : "FAIL");
	len += snprintf(buf + len, PAGE_SIZE - len, "Raw Data : %s\n", ts->pdata->selfdiagnostic_state[1] == 1 ? "PASS" : "FAIL");
	//len += snprintf(buf + len, PAGE_SIZE - len, "Jitter Test: %s\n", ts->pdata->selfdiagnostic_state[2] == 1 ? "PASS" : "FAIL");

	return len;
}

static ssize_t mms_edge_expand_show(struct i2c_client *client, char *buf)
{
	struct mms_data *ts = (struct mms_data *) get_touch_handle_(client);
	int len = 0;

	mms_edge_expand_read(ts);

	len += snprintf(buf + len, PAGE_SIZE - len, "%d, %d, %d, %d",
		edge_expand[0], edge_expand[1], edge_expand[2], edge_expand[3]);

	return len;
}

static ssize_t mms_edge_expand_store(struct i2c_client *client, const char *buf )
{
	struct mms_data *ts = (struct mms_data *) get_touch_handle_(client);
	int i = 0;
	int len = 0;
	int ret = 0;
	int find_num_cnt = 0;
	char *num_pos[4] = {0};
	long value[4] = {0};
	char *ptr = (char *)buf;

	if (ptr[0] >= '0' && ptr[0] <= '9') {
		len = strlen(ptr);
		num_pos[find_num_cnt++] = ptr; // first value

		for ( i = 1; i < len; i++) {
			if (ptr[i] == ',' && ptr[i+1] == ' ') {
				ptr[i] = '\0';
				num_pos[find_num_cnt++] = &ptr[i+2];
			}
		}
	}

	if (find_num_cnt != 4) {
		goto ErrorExit;
	}

	for (i = 0; i < 4; i++) {
		ret = kstrtol(num_pos[i], 10, &value[i]);
		if (ret || (value[i] < 64 || value[i] > 255)) {
			goto ErrorExit;
		}
	}

	for (i = 0; i < 4; i++) {
		edge_expand[i] = (uint8_t)value[i];
	}

	mms_edge_expand_write(ts);

	return 0;

ErrorExit :
	TOUCH_INFO_MSG("edge_expand_store error. %s \n", buf);

	return 0;
}

static int mms_sysfs(struct i2c_client *client, char *buf1, const char *buf2, u32 code)
{
	int ret = 0;
	struct ic_ctrl_param param;

	TOUCH_TRACE_FUNC();

	power_lock(POWER_SYSFS_LOCK);
	mms_power(client, POWER_ON);
	msleep(30);

	switch (code) {
	case SYSFS_VERSION_SHOW :
		param.v1 = (u32) buf1;
		ret = mms_ic_ctrl(client, IC_CTRL_INFO_SHOW, (u32) &param);
		break;
	case SYSFS_TESTMODE_VERSION_SHOW :
		param.v1 = (u32) buf1;
		ret = mms_ic_ctrl(client, IC_CTRL_TESTMODE_VERSION_SHOW, (u32) &param);
		break;
	case SYSFS_REG_CONTROL_STORE:
		ret = mms_reg_control_store(client, buf2);
		break;
	case SYSFS_FX_CONTROL_STORE:
		ret = mms_fx_control_store(client, buf2);
		break;
#ifdef SENSING_TEST
	case SYSFS_SENSING_TEST_STORE:
		ret = mms_sensing_test_store(client, buf2);
		break;
#endif
	case SYSFS_CHSTATUS_SHOW:
		ret = mms_chstatus_show(client, buf1);
		break;
	case SYSFS_RAWDATA_SHOW:
		ret = mms_rawdata_show(client, buf1);
		break;
	case SYSFS_JITTER_SHOW:
		ret = mms_jitter_show(client, buf1);
		break;
	case SYSFS_DELTA_SHOW:
		ret = mms_delta_show(client, buf1);
		break;
	case SYSFS_SELF_DIAGNOSTIC_SHOW:
		ret = mms_self_diagnostic_show(client, buf1);
		break;
	case SYSFS_EDGE_EXPAND_SHOW :
		ret = mms_edge_expand_show(client, buf1);
		break;
	case SYSFS_EDGE_EXPAND_STORE :
		ret = mms_edge_expand_store(client, buf2);
		break;
	}

	power_unlock(POWER_SYSFS_LOCK);

	return ret;
}

struct touch_device_driver mms_driver = {
	.probe = mms_probe,
	.remove = mms_remove,
	.init = mms_init,
	.data = mms_get_data,
	.power = mms_power,
	.fw_upgrade = mms_fw_upgrade,
	.ic_ctrl = mms_ic_ctrl,
	.sysfs = mms_sysfs,
};

static int __devinit touch_init(void)
{
	TOUCH_TRACE_FUNC();
	return touch_driver_register_(&mms_driver);
}

static void __exit touch_exit(void)
{
	TOUCH_TRACE_FUNC();
	touch_driver_unregister_();
}

module_init(touch_init);
module_exit(touch_exit);

MODULE_AUTHOR("WX-BSP-TS@lge.com");
MODULE_DESCRIPTION("LGE Touch Driver");
MODULE_LICENSE("GPL");

