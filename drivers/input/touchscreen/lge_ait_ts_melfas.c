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

#include "lge_ait_ts_melfas.h"

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
	u8 cmd[2] = {MIT_REGH_CMD,reg};
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = &reg,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};

	if(reg == MIT_FW_VERSION) {
		cmd[0] = 0x00;
	}
	msgs[0].buf = cmd;

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
static int mms_get_ic_info(struct mms_data *ts, struct touch_fw_info *fw_info)
{
	struct i2c_client *client = ts->client;

	TOUCH_TRACE_FUNC();

	if (mms_i2c_read(client, MMS_TX_NUM, &ts->dev.row_num, 1) < 0) {
		TOUCH_INFO_MSG("MMS_TX_NUM read failed");
		return -EIO;
	}

	if (mms_i2c_read(client, MMS_RX_NUM, &ts->dev.col_num, 1) < 0) {
		TOUCH_INFO_MSG("MMS_RX_NUM read failed");
		return -EIO;
	}

	if (mms_i2c_read(client, MIT_FW_VERSION,(u8 *) &ts->module.version, 2) < 0) {
		TOUCH_INFO_MSG("MIT_FW_VERSION read failed");
		return -EIO;
	}

	if (mms_i2c_read(client, MIT_FW_PRODUCT,(u8 *) &ts->module.product_code, 16) < 0){
		TOUCH_INFO_MSG("MIT_FW_PRODUCT read failed");
		return -EIO;
	}

	TOUCH_INFO_MSG("======================\n");
	TOUCH_INFO_MSG("F/W Version : %X.%02X \n", ts->module.version[0], ts->module.version[1]);
	TOUCH_INFO_MSG("F/W Product : %s \n", ts->module.product_code);
	TOUCH_INFO_MSG("F/W Row : %d Col : %d \n", ts->dev.row_num, ts->dev.col_num);
	TOUCH_INFO_MSG("======================\n");

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
	snprintf(time_string, 64, "\n%02d-%02d %02d:%02d:%02d.%03lu \n\n\n",
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
	ts->pdata->tap_count = 4; //default tap count set
	ts->pdata->lpwg_prox = 1; //default proxi sensor information
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
				TOUCH_ERR_MSG("Can NOT get regulator : %s, ret = %d\n", ts_pwr[i].name, ret);
				goto err_regulator_get;
			}

			if (regulator_count_voltages(ts->vdd_regulator[i]) > 0) {
				ret = regulator_set_voltage(ts->vdd_regulator[i], ts_pwr[i].value, ts_pwr[i].value);
				if (ret) {
					TOUCH_ERR_MSG("Error(ret=%d) set regulator(%s) voltage %d\n", ret, ts_pwr[i].name, ts_pwr[i].value);
					goto err_regulator_get;
				}
			}
		}
	}

	for(i = 0; i < MAX_ROW; i++) {
		ts->mit_data[i] = kzalloc(sizeof(uint16_t) * MAX_COL, GFP_KERNEL);
		if (ts->mit_data[i] == NULL) {
			TOUCH_ERR_MSG("mit_data kzalloc error\n");
			return -ENOMEM;
		}
		ts->intensity_data[i] = kzalloc(sizeof(uint16_t) * MAX_COL, GFP_KERNEL);
		if (ts->intensity_data[i] == NULL) {
			TOUCH_ERR_MSG("intensity_data kzalloc error\n");
			return -ENOMEM;
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

	for(i = 0; i < MAX_ROW; i++) {
		if (ts->mit_data[i] != NULL) {
			kfree(ts->mit_data[i]);
		}
		if (ts->intensity_data[i] != NULL) {
			kfree(ts->intensity_data[i]);
		}
	}

}

static int mms_init(struct i2c_client* client, struct touch_fw_info* fw_info)
{
	struct mms_data *ts = get_touch_handle_(client);

	TOUCH_TRACE_FUNC();

	ts->probed = true;
	return 0;
}

#define LPWG_XY_MARGIN	30
#define LPWG_STR_MAX		256
struct lpwg_xy
{
	int x;
	int y;
};

struct lpwg_xy pInput[10];

void mms_lpwg_touch_test(struct mms_data *ts, u16 x, u16 y)
{
	int i = 0;
	int len = 0;
	int tap_count = ts->pdata->lpwg_test_count;
	int passwords[10] = {0};
	static int current_count = 0;
	struct lpwg_xy most_left = {0};
	struct lpwg_xy most_top = {0};
	struct lpwg_xy most_right = {0};
	struct lpwg_xy most_bottom = {0};
	int x_line_only = 0;
	int y_line_only = 0;
	char buf[LPWG_STR_MAX] = {0};

	if (tap_count == 0) {
		if (current_count == 0)
			return;

		current_count = 0;
		memset(pInput, 0x0, sizeof(struct lpwg_xy)*10);
		return;
	}

	if (current_count < tap_count) {
		pInput[current_count].x = x;
		pInput[current_count].y = y;
		current_count++;
	}

	if (current_count < tap_count)
		return;

	most_left.x = 0xFFFF;
	most_top.y = 0xFFFF;

	for (i = 0; i < tap_count; i++) {
		if (pInput[i].x < most_left.x) {
			most_left.x = pInput[i].x;
			most_left.y = pInput[i].y;
		}

		if (pInput[i].x > most_right.x) {
			most_right.x = pInput[i].x;
			most_right.y = pInput[i].y;
		}

		if (pInput[i].y < most_top.y) {
			most_top.x = pInput[i].x;
			most_top.y = pInput[i].y;
		}

		if (pInput[i].y > most_bottom.y) {
			most_bottom.x = pInput[i].x;
			most_bottom.y = pInput[i].y;
		}
	}

	if (((most_left.x - LPWG_XY_MARGIN) <= most_right.x) && ((most_left.x + LPWG_XY_MARGIN) >= most_right.x)) {
		x_line_only = 1;
	}

	if (((most_top.y - LPWG_XY_MARGIN) <= most_bottom.y) && ((most_top.y + LPWG_XY_MARGIN) >= most_bottom.y)) {
		y_line_only = 1;
	}

	if (x_line_only && !y_line_only) { /* "1 and 3" or "2 and 4" */

		for (i = 0; i < tap_count; i++) {
			if (((most_left.x - LPWG_XY_MARGIN) <= pInput[i].x) && ((most_left.x + LPWG_XY_MARGIN) >= pInput[i].x) &&
				((most_top.y - LPWG_XY_MARGIN) <= pInput[i].y) && ((most_top.y + LPWG_XY_MARGIN) >= pInput[i].y)) {
				passwords[i] = 1;
			} else
				passwords[i] = 3;
		}

	} else if (!x_line_only && y_line_only) { /* "1 and 2" or "3 and 4" */

		for (i = 0; i < tap_count; i++) {
			if (((most_left.x - LPWG_XY_MARGIN) <= pInput[i].x) && ((most_left.x + LPWG_XY_MARGIN) >= pInput[i].x) &&
				((most_top.y - LPWG_XY_MARGIN) <= pInput[i].y) && ((most_top.y + LPWG_XY_MARGIN) >= pInput[i].y)) {
				passwords[i] = 1;
			} else
				passwords[i] = 2;
		}

	} else {
		for (i = 0; i < tap_count; i++) {
			if (((most_left.x - LPWG_XY_MARGIN) <= pInput[i].x) && ((most_left.x + LPWG_XY_MARGIN) >= pInput[i].x)) {
				if (((most_top.y - LPWG_XY_MARGIN) <= pInput[i].y) && ((most_top.y + LPWG_XY_MARGIN) >= pInput[i].y)) {
					passwords[i] = 1;
				} else
					passwords[i] = 3;
			} else {
				if (((most_top.y - LPWG_XY_MARGIN) <= pInput[i].y) && ((most_top.y + LPWG_XY_MARGIN) >= pInput[i].y)) {
					passwords[i] = 2;
				} else {
					passwords[i] = 4;
				}
			}
		}
	}

	len += snprintf(buf + len, LPWG_STR_MAX - len, "LPWG : ");

	for (i = 0; i < tap_count; i++) {
		len += snprintf(buf + len, LPWG_STR_MAX - len, "%d ", passwords[i]);
	}

	TOUCH_INFO_MSG("%s \n", buf);

	for (i = 0; i < tap_count; i++) {
		TOUCH_INFO_MSG("LPWG : %d  x[%3d] y[%3d] \n", passwords[i], pInput[i].x, pInput[i].y);
	}

	current_count = 0;
	memset(pInput, 0x0, sizeof(struct lpwg_xy)*10);
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
	int finger_event_sz = 0;
#ifdef SENSING_TEST
	char log_buf[256] = {0};
#endif

	TOUCH_TRACE_FUNC();

	finger_event_sz = MIT_FINGER_EVENT_SZ;

	data->total_num = touch_count;

	for (i = 0; i < sz; i += finger_event_sz) {
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
					data->curr_data[id].touch_conut = 0;
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

			else if (data->curr_data[id].status == FINGER_RELEASED
				&& data->prev_data[id].point_log_state) {
					data->touch_count_num--;

					if (likely(touch_debug_mask_ & DEBUG_ABS_POINT)) {
						TOUCH_INFO_MSG("touch_release[%s] : <%d> x[%3d] y[%3d] M:%d\n",
						data->palm?"Palm":" ", id,
						data->prev_data[id].x_position,
						data->prev_data[id].y_position,
						data->curr_data[id].touch_conut);

						mms_lpwg_touch_test(ts, data->prev_data[id].x_position, data->prev_data[id].y_position);
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

			else {
					data->curr_data[id].touch_conut++;
			}
		}
	}

	data->total_num = touch_count;

	if (unlikely(touch_debug_mask_ & DEBUG_GET_DATA))
		TOUCH_INFO_MSG("Total_num: %d\n", data->total_num);

	return 0;
}

static int mms_lpwg_event(struct i2c_client *client, struct touch_data *data, u8 *buf, int sz)
{
	struct mms_data *ts = get_touch_handle_(client);
	int i = 0;
	int id = 0;
	int x = 0;
	int y = 0;
	u8 *tmp = NULL;

	ts->pdata->send_lpwg = 0;
	ts->pdata->lpwg_size = 0;

	if (buf[1] == 0) {
		TOUCH_INFO_MSG("LPWG Password Tap detected \n");
		for (i = 2; i < sz; i += MIT_LPWG_EVENT_SZ) {
			tmp = buf + i;
			id = i;
			x = tmp[1] | ((tmp[0] & 0xf) << 8);
			y = tmp[2] | (((tmp[0] >> 4 ) & 0xf) << 8);
			TOUCH_INFO_MSG("LPWG %d TAP x[%3d] y[%3d] \n", (i+1)/MIT_LPWG_EVENT_SZ, x, y);
			ts->pdata->lpwg_x[((i + 1) / MIT_LPWG_EVENT_SZ) - 1] = x;
			ts->pdata->lpwg_y[((i + 1) / MIT_LPWG_EVENT_SZ) - 1] = y;
			ts->pdata->lpwg_size++;
			ts->pdata->send_lpwg = LPWG_MULTI_TAP;
		}
	} else if (buf[1] == 1) {
		TOUCH_INFO_MSG("LPWG Double Tap detected \n");
		ts->pdata->send_lpwg = LPWG_DOUBLE_TAP;
	} else {
		TOUCH_INFO_MSG("Unknown Packet Error : %02X %02X %02X %02X %02X \n", buf[0], buf[1], buf[2], buf[3], buf[4]);
	}

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

	memset(ts->buf, 0, FINGER_EVENT_SZ * ts->pdata->caps->max_id);

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
	} else if (event_type == MMS_LPWG_EVENT) {
		if (mms_lpwg_event(client, data, ts->buf, sz) < 0)
			goto err_event_type;
	} else if (event_type == MMS_ERROR_EVENT) {
		data->state = ts->buf[1];
		goto mms_error_event;
	} else if (event_type == MMS_LOG_EVENT) {
		if (mms_log_event(client, ts) < 0)
			goto err_event_type;
	} else {
		TOUCH_ERR_MSG("Unkown, event type %d\n", event_type);
		if(event_type == 0)
			return 0;
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
	int ret = 0;

	TOUCH_DEBUG_MSG("%s : = %d\n", __func__, power_ctrl);
	if (power_state == power_ctrl) {
		TOUCH_DEBUG_MSG("%s : power_state == power_ctrl\n", __func__);
		return 0;
	}

	switch (power_ctrl) {
	case POWER_OFF:
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
                        mdelay(2);
		} while(--i >= 0);
		gpio_direction_output(ts_pdata->reset_pin, 0);
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
					ret = regulator_enable(ts->vdd_regulator[i]);
					if (ret) {
						TOUCH_POWER_MSG("power: regulator[%d] enable failed ret =%d\n", i, ret );
					}
					/*
					TOUCH_POWER_MSG("power[%d]: regulator[%s] enabled", i,
						rdev_get_name(ts->vdd_regulator[i]->rdev));
					*/
				}
			}
                        mdelay(2);
		} while(++i < TOUCH_PWR_NUM);
		gpio_direction_output(ts_pdata->reset_pin, 1);
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
	TOUCH_INFO_MSG("%s : %d \n", __func__, power_ctrl);

	return mms_power(client, power_ctrl);
}
EXPORT_SYMBOL(mms_power_ctrl);

int mms_power_reset(struct mms_data *ts)
{
	TOUCH_INFO_MSG("Power Reset \n");

	mms_power(ts->client, POWER_OFF);
	msleep(ts->pdata->role->reset_delay);
	mms_power(ts->client, POWER_ON);
	msleep(ts->pdata->role->reset_delay);

	return 0;
}

static int mms_firmware_img_parse_show(const char *image_bin, char *show_buf, int ret)
{
	struct mms_bin_hdr *fw_hdr = NULL;
	return 0;

	fw_hdr = (struct mms_bin_hdr *) image_bin;

	ret += sprintf(show_buf + ret, "mms_fw_hdr:\n");
	ret += sprintf(show_buf + ret, "\ttag[%c%c%c%c%c%c%c%c]\n",
			fw_hdr->tag[0], fw_hdr->tag[1], fw_hdr->tag[2], fw_hdr->tag[3],
			fw_hdr->tag[4], fw_hdr->tag[5], fw_hdr->tag[6], fw_hdr->tag[7]);
	ret += sprintf(show_buf + ret, "\tcore_version[0x%02x]\n", fw_hdr->core_version);
	ret += sprintf(show_buf + ret, "\tsection_num[%d]\n", fw_hdr->section_num);
	ret += sprintf(show_buf + ret, "\tcontains_full_binary[%d]\n", fw_hdr->contains_full_binary);
	ret += sprintf(show_buf + ret, "\tbinary_offset[%d (0x%04x)]\n", fw_hdr->binary_offset, fw_hdr->binary_offset);
	ret += sprintf(show_buf + ret, "\tbinary_length[%d]\n", fw_hdr->binary_length);

	return ret;
}

static int mms_fw_upgrade(struct i2c_client* client, struct touch_fw_info *info)
{
	struct mms_data *ts = get_touch_handle_(client);
	int ret = 0;

	TOUCH_TRACE_FUNC();

	touch_disable(ts->client->irq);

	if (info->fw)
		ret = mit_isc_fwupdate(ts, info);

	touch_enable(ts->client->irq);

	return ret;
}

static int mms_lpwg_start(struct mms_data* ts, u8 mode)
{
	char write_buf[255] = {0};
	int ret = 0;
	if (mode) {
		write_buf[0] = MIT_REGH_CMD;
		write_buf[1] = MIT_LPWG_START;
		write_buf[2] = 0x01;
		if (i2c_master_send(ts->client,write_buf,3) != 3) {
			TOUCH_INFO_MSG("MIT_LPWG_START write error \n");
		} else {
			TOUCH_INFO_MSG("MIT_LPWG_START \n");
		}
	}
	return ret;
}

static int mms_set_lpwg(struct mms_data* ts, u8 mode)
{
	char write_buf[255] = {0};
	int ret = 0;

	if (mode) {  // LPWG enable mode
		write_buf[0] = MIT_REGH_CMD;
		write_buf[1] = MIT_LPWG_MODE;
		write_buf[2] = mode & (LPWG_DOUBLE_TAP | LPWG_MULTI_TAP);
		if (i2c_master_send(ts->client,write_buf,3) != 3) {
			TOUCH_INFO_MSG("MIT_LPWG_MODE write error \n");
		} else {
			TOUCH_INFO_MSG("MIT_LPWG_MODE = %X \n", write_buf[2]);
		}

		if (mode & LPWG_MULTI_TAP) {
			write_buf[0] = MIT_REGH_CMD;
			write_buf[1] = MIT_LPWG_TOTAL_TAP_COUNT;
			write_buf[2] = ts->pdata->tap_count;
			if (i2c_master_send(ts->client,write_buf,3) != 3) {
				TOUCH_INFO_MSG("MIT_LPWG_TOTAL_TAP_COUNT write error \n");
			} else {
				TOUCH_INFO_MSG("MIT_LPWG_TOTAL_TAP_COUNT = %d \n", write_buf[2]);
			}

			write_buf[0] = MIT_REGH_CMD;
			write_buf[1] = MIT_LPWG_TAP_DISTANCE;
			write_buf[2] = 10;
			if (i2c_master_send(ts->client,write_buf,3) != 3) {
				TOUCH_INFO_MSG("MIT_LPWG_TAP_DISTANCE write error \n");
			} else {
				TOUCH_INFO_MSG("MIT_LPWG_TAP_DISTANCE = %d \n", write_buf[2]);
				TOUCH_INFO_MSG("MIT_LPWG_TAP_DISTANCE2 = 0xff \n");
			}
		} else if (mode & LPWG_DOUBLE_TAP) {
			write_buf[0] = MIT_REGH_CMD;
			write_buf[1] = MIT_LPWG_TAP_DISTANCE;
			write_buf[2] = 10;
			if (i2c_master_send(ts->client,write_buf,3) != 3) {
				TOUCH_INFO_MSG("MIT_LPWG_TAP_DISTANCE write error \n");
			} else {
				TOUCH_INFO_MSG("MIT_LPWG_TAP_DISTANCE = %d \n", write_buf[2]);
			}
		}

		write_buf[0] = MIT_REGH_CMD;
		write_buf[1] = MIT_LPWG_TOUCH_SLOP;
		write_buf[2] = 10;
		if(i2c_master_send(ts->client,write_buf,3)!=3) {
			TOUCH_INFO_MSG("MIT_LPWG_TOUCH_SLOP write error \n");
		} else {
			TOUCH_INFO_MSG("MIT_LPWG_TOUCH_SLOP = %d \n", write_buf[2]);
		}

		write_buf[0] = MIT_REGH_CMD;
		write_buf[1] = MIT_LPWG_TAP_TIME_GAP;
		write_buf[2] = (700 >> 8);
		write_buf[3] = (700 & 0xFF);
		if(i2c_master_send(ts->client,write_buf,3)!=3) {
			TOUCH_INFO_MSG("MIT_LPWG_TAP_TIME_GAP write error \n");
		} else {
			TOUCH_INFO_MSG("MIT_LPWG_TAP_TIME_GAP = %d, %d \n", write_buf[2], write_buf[3]);
		}

		write_buf[0] = MIT_REGH_CMD;
		write_buf[1] = MIT_LPWG_ACTIVE_AREA;
		write_buf[2] = ts->pdata->lpwg_area_x1 >> 8; write_buf[3] = ts->pdata->lpwg_area_x1 & 0xFF;
		write_buf[4] = ts->pdata->lpwg_area_y1 >> 8; write_buf[5] = ts->pdata->lpwg_area_y1 & 0xFF;
		write_buf[6] = ts->pdata->lpwg_area_x2 >> 8; write_buf[7] = ts->pdata->lpwg_area_x2 & 0xFF;
		write_buf[8] = ts->pdata->lpwg_area_y2 >> 8; write_buf[9] = ts->pdata->lpwg_area_y2 & 0xFF;
		
		if(i2c_master_send(ts->client,write_buf,10)!=10) {
			TOUCH_INFO_MSG("MIT_LPWG_ACTIVE_AREA write error \n");
		} else {
			TOUCH_DEBUG_MSG("MIT_LPWG_ACTIVE_AREA = %d %d  | %d %d | %d %d | %d %d \n", write_buf[2], write_buf[3], write_buf[4], write_buf[5], write_buf[6], write_buf[7], write_buf[8], write_buf[9]);
		}

	} else { // LPWG disable mode

		write_buf[0] = MIT_REGH_CMD;
		write_buf[1] = MIT_LPWG_MODE;
		write_buf[2] = 0;
		if (i2c_master_send(ts->client,write_buf,3) != 3) {
			TOUCH_INFO_MSG("MIT_LPWG_MODE write error \n");
		} else {
			TOUCH_INFO_MSG("MIT_LPWG_MODE = %X \n", write_buf[2]);
		}

		mms_lpwg_start(ts, 1);

	} // LPWG disable mode end

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
		if (param) {
			buf = (char *) param->v1;
			if (buf) {
				ret += sprintf(buf + ret, "======================\n");
				ret += sprintf(buf + ret, "F/W Version : %X.%02X \n", ts->module.version[0], ts->module.version[1]);
				ret += sprintf(buf + ret, "F/W Product : %s \n", ts->module.product_code);
				ret += sprintf(buf + ret, "F/W Row : %d Col : %d \n", ts->dev.row_num, ts->dev.col_num);
				ret += sprintf(buf + ret, "======================\n");
			}
		}
		break;

	case IC_CTRL_TESTMODE_VERSION_SHOW:
		if (ts->module.product_code[0])
			TOUCH_INFO_MSG("F/W : %X.%02X (%s)\n", ts->module.version[0], ts->module.version[1], ts->module.product_code);
		if (param) {
			buf = (char *) param->v1;
			if (buf) {
				ret += sprintf(buf + ret, "%X.%02X(%s)\n", ts->module.version[0], ts->module.version[1], ts->module.product_code);
			}
		}
		break;

	case IC_CTRL_SAVE_IC_INFO:
		mms_get_ic_info(ts, NULL);
		break;

	case IC_CTRL_LPWG:
		mms_set_lpwg(ts, (u8)param->v1);
		if (param) {
			// LPWG enable mode
			mms_lpwg_start(ts, (u8)param->v1);
		} else {
			// LPWG disable mode
			mms_lpwg_start(ts, 1);
		}
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
#if 0
static int get_limit(struct mms_data *ts, char* breakpoint, int *limit_data, int *limit_spec)
{
	int fd = 0;
	char *fname = "/data/mit_limit.txt";
	int p = 0;
	int q = 0;
	char *qq = NULL;
	int cipher = 1;
	int r = 0;
	const struct firmware *fwlimit = NULL;
	int ret = 0;
	char* line = NULL;

	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);

	line = kzalloc(1024, GFP_KERNEL);
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
			if (fwlimit->size > 1024) {
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
		sys_read(fd, line, 1024);
		ret = 1;
	}

	qq = strstr(line, breakpoint);
	if (qq == NULL) {
		TOUCH_INFO_MSG("failed to find breakpoint. The limit file is wrong");
		ret = -1;
		goto exit;
	} else {
		q = (int)(qq - line);
	}

	if (limit_data == NULL)
		goto get_openshort_limit;

	memset(limit_data, 0, MAX_ROW * sizeof(int));
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
			if(r == ts->dev.row_num){
				ret = -2;
				goto exit;
			}
			ret = -3;
			goto exit;
		}
		if (r == ts->dev.row_num) {
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

get_openshort_limit:
	cipher = 10;
	*limit_spec = 0;
	while(1) {
		if ((line[q] >= '0') && (line[q] <= '9')) {
			*limit_spec = (*limit_spec) * cipher + (line[q] - '0');
			if (p == 0)
				p = q - 1;
		} else if (p != 0) {
			break;
		}
		q++;
	}
	if (line[p] == '-')
		*limit_spec *= -1;

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
#endif

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
/*
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

	if (ts->dev.key_num) {
		// read touch key chstatus
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
*/
static ssize_t mms_rawdata_show(struct i2c_client *client, char *buf)
{
	struct mms_data *ts = (struct mms_data *) get_touch_handle_(client);
	int ret = 0;
	ts->pdata->selfdiagnostic_state[0] = 1;	// rawdata

	TOUCH_TRACE_FUNC();

	ret = mit_get_test_result(client, buf, RAW_DATA_SHOW);
	if (ret < 0) {
		memset(buf, 0, PAGE_SIZE);
		ret = snprintf(buf, PAGE_SIZE, "failed to get raw data\n");
	}
	return ret;
}

static ssize_t mms_rawdata_store(struct i2c_client *client, const char *buf)
{
	int ret = 0;
	char temp_buf[255];
	TOUCH_TRACE_FUNC();
	strcpy(temp_buf,buf);

	ret = mit_get_test_result(client, temp_buf, RAW_DATA_STORE);

	return ret;
}

static ssize_t mit_chstatus_show(struct i2c_client *client, char *buf)
{
	int ret = 0;
	int len = 0;
	struct mms_data *ts = (struct mms_data *) get_touch_handle_(client);
	ts->pdata->selfdiagnostic_state[1] = 1;	// openshort
	ts->pdata->selfdiagnostic_state[2] = 1;	// slope

	TOUCH_TRACE_FUNC();
	TOUCH_INFO_MSG("mit_chstatus_show\n");

	ret = mit_get_test_result(client, buf, OPENSHORT);
	memset(buf, 0, PAGE_SIZE);
	if (ret < 0) {
		TOUCH_INFO_MSG("Failed to get OPEN SHORT Test result. \n");
		ret = snprintf(buf, PAGE_SIZE, "failed to OPEN SHORT data\n");
		goto error;
	}
	ret = mit_get_test_result(client, buf, SLOPE);
	memset(buf, 0, PAGE_SIZE);
	if (ret < 0) {
		TOUCH_INFO_MSG("Failed to get SLOPE Test result. \n");
		ret = snprintf(buf, PAGE_SIZE, "failed to SLOPE data\n");
		goto error;
	}

	len = snprintf(buf, PAGE_SIZE - len, "Firmware Version : %X.%02X \n", ts->module.version[0], ts->module.version[1]);
	len += snprintf(buf + len, PAGE_SIZE - len, "FW Product : %s \n", ts->module.product_code);
	len += snprintf(buf + len, PAGE_SIZE - len, "=======RESULT========\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "OPEN /  SHORT Test : %s\n", ts->pdata->selfdiagnostic_state[1]==1 ? "PASS" : "FAIL");
	len += snprintf(buf + len, PAGE_SIZE - len, "SLOPE Test : %s\n", ts->pdata->selfdiagnostic_state[2] == 1 ? "PASS" : "FAIL");

	return len;

error:
	return ret;
}

static int melfas_delta_show(struct i2c_client* client, char *buf)
{
	int ret = 0;

	TOUCH_TRACE_FUNC();

	ret = mit_delta_show(client, buf);

	return ret;
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

	if (ts->dev.key_num) {
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
	int ret = 0;
	int row = 0;
	int col = 0;
	int max = 0;
	int min = 0;
	char *sd_path = "/sdcard/touch_self_test.txt";
	ts->pdata->selfdiagnostic_state[0] = 1;	// rawdata
	ts->pdata->selfdiagnostic_state[1] = 1;	// openshort
	ts->pdata->selfdiagnostic_state[2] = 1;	// slope

	write_file(sd_path, buf, 1);
	msleep(30);

	ret = mit_get_test_result(client, buf, OPENSHORT);
	if (ret < 0) {
		TOUCH_ERR_MSG("failed to get open short data\n");
		memset(buf, 0, PAGE_SIZE);
		len += snprintf(buf, PAGE_SIZE, "failed to get open short data\n\n");
		ts->pdata->selfdiagnostic_state[1] = 0;
	}
	write_file(sd_path, buf, 0);
	msleep(30);

	memset(buf, 0, PAGE_SIZE);
	ret = mit_get_test_result(client, buf, SLOPE);
	if (ret < 0) {
		TOUCH_ERR_MSG("failed to get slope data\n");
		memset(buf, 0, PAGE_SIZE);
		len = snprintf(buf, PAGE_SIZE, "failed to get slope data\n\n");
		ts->pdata->selfdiagnostic_state[2] = 0;
	}
	write_file(sd_path, buf, 0);
	msleep(30);

	memset(buf, 0, PAGE_SIZE);
	ret = mit_get_test_result(client, buf, RAW_DATA_SHOW);
	if (ret < 0) {
		TOUCH_ERR_MSG("failed to get raw data\n");
		memset(buf, 0, PAGE_SIZE);
		len = snprintf(buf, PAGE_SIZE, "failed to get raw data\n\n");
		ts->pdata->selfdiagnostic_state[0] = 0;
	}
	min = ts->mit_data[0][0];
	max = ts->mit_data[0][0];
	ret += sprintf(buf+ret,"RAW DATA SPEC (UPPER : %d  LOWER : %d)\n",ts->pdata->limit->raw_data_max ,ts->pdata->limit->raw_data_min);
	TOUCH_INFO_MSG("RAW DATA SPEC (UPPER : %d  LOWER : %d)\n",ts->pdata->limit->raw_data_max ,ts->pdata->limit->raw_data_min);

		for(row = 0 ; row < MAX_ROW; row++) {
			if (ts->pdata->selfdiagnostic_state[0] == 0) {
				ret += sprintf(buf+ret,"[%2d]  ",row);
				printk("[%2d]  ",row);
			}
			for(col = 0 ; col < MAX_COL ; col++) {
				min = (min > ts->mit_data[row][col]) ? ts->mit_data[row][col] : min;
				max = (max < ts->mit_data[row][col]) ? ts->mit_data[row][col] : max;
				if (ts->pdata->selfdiagnostic_state[0] == 0) {
					if(ts->mit_data[row][col] <= ts->pdata->limit->raw_data_max && ts->mit_data[row][col] >= ts->pdata->limit->raw_data_min ){
							ret += sprintf(buf+ret," ,");
							printk(" ,");
						}else{
							ret += sprintf(buf+ret,"X,");
							printk("X,");
						}
				}
			}
			if (ts->pdata->selfdiagnostic_state[0] == 0) {
				printk("\n");
				ret += sprintf(buf+ret,"\n");
			}
		}
	if (ts->pdata->selfdiagnostic_state[0] == 0) {
		ret += sprintf(buf+ret,"RawData : FAIL\n\n");
		TOUCH_INFO_MSG("RawData : FAIL\n\n");
	}else {
		ret += sprintf(buf+ret,"RawData : PASS\n\n");
		TOUCH_INFO_MSG("RawData : PASS\n\n");
	}
	write_file(sd_path, buf, 0);
	msleep(30);

	TOUCH_INFO_MSG("Firmware Version : %X.%02X \n", ts->module.version[0], ts->module.version[1]);
	TOUCH_INFO_MSG("FW Product : %s \n", ts->module.product_code);
	TOUCH_INFO_MSG("MAX : %d,  MIN : %d\nMAX - MIN = %d\n", max, min, max - min);
	TOUCH_INFO_MSG("=======RESULT========\n");
	TOUCH_INFO_MSG("Channel Status : %s\n", (ts->pdata->selfdiagnostic_state[1] * ts->pdata->selfdiagnostic_state[2]) == 1 ? "PASS" : "FAIL");
	TOUCH_INFO_MSG("Raw Data : %s\n", ts->pdata->selfdiagnostic_state[0] == 1 ? "PASS" : "FAIL");

	memset(buf, 0, PAGE_SIZE);
	len = snprintf(buf, PAGE_SIZE , "Firmware Version : %X.%02X \n", ts->module.version[0], ts->module.version[1]);
	len += snprintf(buf + len, PAGE_SIZE - len, "FW Product : %s \n", ts->module.product_code);
	len += snprintf(buf + len, PAGE_SIZE - len, "MAX = %d,  MIN = %d\nMAX - MIN = %d\n", max, min, max - min);
	len += snprintf(buf + len, PAGE_SIZE - len, "=======RESULT========\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "Channel Status : %s\n", (ts->pdata->selfdiagnostic_state[1] * ts->pdata->selfdiagnostic_state[2]) == 1 ? "PASS" : "FAIL");
	len += snprintf(buf + len, PAGE_SIZE - len, "Raw Data : %s\n", ts->pdata->selfdiagnostic_state[0] == 1 ? "PASS" : "FAIL");
	write_file(sd_path, buf, 0);
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

static int mms_sensing_block_control(struct i2c_client *client, u8 type, u8 onoff)
{
	struct mms_data* ts = get_touch_handle_(client);
	u8 wbuf[6] = {32, 0, 81, 1, 0, 0};
	int i = 0;

	switch(type) {
		case 0 :
			break;
		case 81 :
		case 82 :
		case 83 :
			wbuf[2] = (u8)type;
			break;
		default :
			TOUCH_INFO_MSG("not support %d \n", type);
			return 0;
	}

	if (onoff)
		wbuf[4] = 1;
	else
		wbuf[4] = 0;

	if (type == 0) {
		for (i = 81; i <= 83; i++) {
			wbuf[2] = i;
			i2c_master_send(ts->client, wbuf, 6);
			TOUCH_INFO_MSG("Sensing Block (%d) : %s \n", wbuf[2], onoff ? "On" : "Off");
		}
	} else {
		i2c_master_send(ts->client, wbuf, 6);
		TOUCH_INFO_MSG("Sensing Block (%d) : %s \n", wbuf[2], onoff ? "On" : "Off");
	}

	wbuf[0] = 0x1F;
	wbuf[1] = 0xFF;
	wbuf[2] = 1;
	i2c_master_send(ts->client, wbuf, 3);


	if (onoff) {
		touch_disable(ts->client->irq);
		mms_power_reset(ts);
		touch_enable(ts->client->irq);
	}

	return 0;
}

static ssize_t mms_fw_dump_show(struct i2c_client *client, char *buf)
{
	struct mms_data *ts = (struct mms_data *) get_touch_handle_(client);
	int len = 0;
	u8 *pDump = NULL;
	int readsize = 0;
	int addr = 0;
	int retrycnt = 0;
	int fd = 0;
	char *dump_path = "/sdcard/touch_dump.fw";
	mm_segment_t old_fs = get_fs();

	TOUCH_INFO_MSG("F/W Dumping... \n");

	touch_disable(ts->client->irq);

	pDump = kzalloc(FW_MAX_SIZE, GFP_KERNEL);

RETRY :
	readsize = 0;
	retrycnt++;
	mms_power_reset(ts);
	msleep(50);

	for(addr = 0; addr < FW_MAX_SIZE; addr += FW_BLOCK_SIZE ) {
		if ( mit_isc_page_read(ts, &pDump[addr], addr) ) {
			TOUCH_INFO_MSG("F/W Read failed \n");
			if (retrycnt > 10) {
				len += snprintf(buf + len, PAGE_SIZE - len, "dump failed \n");
				goto EXIT;
			}
			else
				goto RETRY;
		}

		readsize += FW_BLOCK_SIZE;
		if (readsize % (FW_BLOCK_SIZE * 20) == 0) {
			TOUCH_INFO_MSG("\t Dump %5d / %5d bytes\n", readsize, FW_MAX_SIZE);
		}
	}

	TOUCH_INFO_MSG("\t Dump %5d / %5d bytes\n", readsize, FW_MAX_SIZE);

	set_fs(KERNEL_DS);
	fd = sys_open(dump_path, O_WRONLY|O_CREAT, 0666);
	if (fd >= 0) {
		sys_write(fd, pDump, FW_MAX_SIZE);
		sys_close(fd);
		len += snprintf(buf + len, PAGE_SIZE - len, "%s saved \n", dump_path);
	} else {
		len += snprintf(buf + len, PAGE_SIZE - len, "%s open failed \n", dump_path);
	}

	set_fs(old_fs);

EXIT :
	kfree(pDump);

	mit_isc_exit(ts);

	mms_power_reset(ts);

	touch_enable(ts->client->irq);

	return len;
}

static ssize_t mms_lpwg_show(struct i2c_client *client, char *buf)
{
	struct mms_data *ts = (struct mms_data *) get_touch_handle_(client);
	int len = 0;

	len += snprintf(buf + len, PAGE_SIZE - len, "LPWG Mode : %X (0:None, 1:Knock-On, 10, Knock-Code) \n", ts->pdata->lpwg_mode);

	return len;
}

static ssize_t mms_lpwg_store(struct i2c_client *client, char* buf1, const char *buf2 )
{
	struct mms_data *ts = (struct mms_data *) get_touch_handle_(client);
	int mode = LPWG_NONE;
	int tap_count = 0;

	/* set tap_count */
	if (buf1 != NULL && !strcmp(buf1,"tap_count")) {
		sscanf(buf2, "%d" ,&tap_count);
		ts->pdata->tap_count = tap_count;
		TOUCH_INFO_MSG("Set Touch Tap Count  = %d \n", ts->pdata->tap_count);
		return 0;
	}

	/* Proximity return temp */
        if(ts->pdata->lpwg_prox == 0)
            return 0;

	/* set lpwg mode */
	if (buf2 == NULL) {
		TOUCH_INFO_MSG(" mode is NULL, Can't not set LPWG\n");
		return 0;
	}
	sscanf(buf2, "%X", &mode);
	ts->pdata->lpwg_mode = (u8)mode;

/*
*set active area
*/
	if(buf1 != NULL && !strcmp(buf1,"area")) {
		ts->pdata->lpwg_area_x1 = ts->pdata->lpwg_gap;
		ts->pdata->lpwg_area_x2 = ts->pdata->caps->lcd_x - ts->pdata->lpwg_gap;
		ts->pdata->lpwg_area_y1 = 0;
		ts->pdata->lpwg_area_y2 = ts->pdata->caps->lcd_y;			
		TOUCH_DEBUG_MSG("X1:%d, X2:%d, Y1:%d, Y2:%d\n",ts->pdata->lpwg_area_x1, ts->pdata->lpwg_area_x2, ts->pdata->lpwg_area_y1, ts->pdata->lpwg_area_y2);
		return 0;
	}
#if 0
	/* Proximity Sensor on/off */
	if (ts->pdata->panel_on == 0 && ts->pdata->lpwg_panel_on == 0) {
		TOUCH_INFO_MSG("SUSPEND AND SET\n");
		if (ts->pdata->lpwg_mode == 0) {
			touch_disable_wake(ts->client->irq);
			touch_disable(ts->client->irq);
			mms_set_lpwg(ts,0);
			TOUCH_INFO_MSG("SUSPEND AND SET  off\n");
		} else  {
			touch_enable(ts->client->irq);
			touch_enable_wake(ts->client->irq);
			mms_ic_ctrl(client, IC_CTRL_LPWG, (u32)&(ts->pdata->lpwg_mode));
			TOUCH_INFO_MSG("SUSPEND AND SET power on\n");
		}
	}
#endif
	TOUCH_INFO_MSG("%s %X \n", __func__, ts->pdata->lpwg_mode);

	return 0;
}

static ssize_t mms_lpwg_wakeup_show(struct i2c_client *client, char *buf)
{
	struct lge_touch_data *ts = i2c_get_clientdata(client);

	touch_send_wakeup((void *)ts);

	return 0;
}

static ssize_t mms_lpwg_test_store(struct i2c_client *client, const char *buf )
{
	struct mms_data *ts = (struct mms_data *) get_touch_handle_(client);
	int lpwg_test_count = 0;

	sscanf(buf, "%d", &lpwg_test_count);

	if (lpwg_test_count > 10)
		lpwg_test_count = 10;

	ts->pdata->lpwg_test_count = (u8)lpwg_test_count;

	TOUCH_INFO_MSG("%s %d \n", __func__, ts->pdata->lpwg_test_count);

	return 0;
}

static int mms_sysfs(struct i2c_client *client, char *buf1, const char *buf2, u32 code)
{
	int ret = 0;
	struct ic_ctrl_param param;
	struct mms_data *ts = (struct mms_data *) get_touch_handle_(client);
	int type = 0;
	int onoff = 0;

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
		touch_disable(ts->client->irq);
		ret = mit_chstatus_show(client, buf1);
		touch_enable(ts->client->irq);
		break;
	case SYSFS_RAWDATA_SHOW:
		touch_disable(ts->client->irq);
		ret = mms_rawdata_show(client, buf1);
		touch_enable(ts->client->irq);
		break;
	case SYSFS_RAWDATA_STORE:
		touch_disable(ts->client->irq);
		ret = mms_rawdata_store(client, buf2);
		touch_enable(ts->client->irq);
		break;
	case SYSFS_JITTER_SHOW:
		ret = mms_jitter_show(client, buf1);
		break;
	case SYSFS_DELTA_SHOW:
		ret = melfas_delta_show(client, buf1);
		break;
	case SYSFS_SELF_DIAGNOSTIC_SHOW:
		touch_disable(ts->client->irq);
		ret = mms_self_diagnostic_show(client, buf1);
		touch_enable(ts->client->irq);
		break;
	case SYSFS_EDGE_EXPAND_SHOW :
		ret = mms_edge_expand_show(client, buf1);
		break;
	case SYSFS_EDGE_EXPAND_STORE :
		ret = mms_edge_expand_store(client, buf2);
		break;
	case SYSFS_SENSING_ALL_BLOCK_CONTROL :
		sscanf(buf1, "%d", &onoff);
		type = 0;
		ret = mms_sensing_block_control(client, (u8)type, (u8)onoff);
		break;
	case SYSFS_SENSING_BLOCK_CONTROL :
		sscanf(buf1, "%d", &onoff);
		sscanf(buf2, "%d", &type);
		ret = mms_sensing_block_control(client, (u8)type, (u8)onoff);
		break;
	case SYSFS_FW_DUMP :
		ret = mms_fw_dump_show(client, buf1);
		break;
	case SYSFS_LPWG_SHOW :
		ret = mms_lpwg_show(client, buf1);
		break;
	case SYSFS_LPWG_STORE :
		ret = mms_lpwg_store(client, buf1 ,buf2);
		break;
	case SYSFS_LPWG_WAKEUP_SHOW :
		ret = mms_lpwg_wakeup_show(client, buf1);
		break;
	case SYSFS_LPWG_TEST_STORE :
		ret = mms_lpwg_test_store(client, buf2);
		break;
	}

	power_unlock(POWER_SYSFS_LOCK);

	return ret;
}

enum window_status mms_check_crack(struct i2c_client *client)
{
	int ret = NO_CRACK;
	int result = 0;
	char buf[4] = {0,};
	struct mms_data *ts = (struct mms_data *) get_touch_handle_(client);

	TOUCH_TRACE_FUNC();

	touch_disable(ts->client->irq);
	result = mit_get_test_result(client, buf, CRACK_CHECK);
	touch_enable(ts->client->irq);

	if (result < 0 || ts->count_short > CRACK_SPEC) {
		ret = CRACK;
	}
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
	.inspection_crack = mms_check_crack,
};

static int __init touch_init(void)
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

