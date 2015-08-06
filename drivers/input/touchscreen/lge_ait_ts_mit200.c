/* lge_ts_mms100s.c
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
	

#define ISC_MASS_ERASE				{0xFB, 0x4A, 0x00, 0x15, 0x00, 0x00}
#define ISC_PAGE_WRITE				{0xFB, 0x4A, 0x00, 0x5F, 0x00, 0x00}
#define ISC_FLASH_READ				{0xFB, 0x4A, 0x00, 0xC2, 0x00, 0x00}
#define ISC_STATUS_READ				{0xFB, 0x4A, 0x00, 0xC8, 0x00, 0x00}
#define ISC_EXIT						{0xFB, 0x4A, 0x00, 0x66, 0x00, 0x00}

#define MAX_ITERATOR	30000

struct isc_packet {
	u8 	cmd;
	u32	addr;
	u8	data[0];
} __attribute__ ((packed));

static int mit_isc_check_status(struct mms_data *ts)
{
	struct i2c_client *client = ts->client;
	int count = 50;
	u8 cmd[6] = ISC_STATUS_READ;
	u8 buf = 0;
	struct i2c_msg msg[]={
		{
			.addr = client->addr,
			.flags = 0,
			.buf = cmd,
			.len = 6,
		},{
			.addr=client->addr,
			.flags = I2C_M_RD,
			.buf = &buf,
			.len = 1,
		}
	};

	while(count--) {
		if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) != ARRAY_SIZE(msg)) {
			TOUCH_INFO_MSG("failed to read isc status \n");
			return -1;
		}

		if ( buf == 0xAD ) {
			return 0;
		}

		msleep(1);
	}
	TOUCH_INFO_MSG("failed to read isc status \n");
	return -1;
}

static int mit_isc_verify_erased(struct mms_data *ts)
{
	struct i2c_client *client = ts->client;
	u8 cmd[6] = ISC_FLASH_READ;
	u8 buf[4] = {0};

	struct i2c_msg msg[]={
		{
			.addr = client->addr,
			.flags = 0,
			.buf = cmd,
			.len = 6,
		},{
			.addr=client->addr,
			.flags = I2C_M_RD,
			.buf = buf,
			.len = 4,
		}
	};

	if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) != ARRAY_SIZE(msg)) {
		TOUCH_INFO_MSG("failed to verifty isc \n");
		return -1;
	}


	if (buf[0]!=0xFF || buf[1]!=0xFF || buf[2]!=0xFF || buf[3]!=0xFF) {
		TOUCH_INFO_MSG("failed to erase IC \n");
		return -1;
	}

	return 0;
}

static int mit_isc_mass_erase(struct mms_data *ts)
{
	char tmp[6] = ISC_MASS_ERASE;

	TOUCH_TRACE_FUNC();

	memset(ts->module.product_code, 0, sizeof(ts->module.product_code));
	memset(ts->module.version, 0, sizeof(ts->module.version));

	if (mms_i2c_write_block(ts->client, tmp, 6) < 0) {
		TOUCH_ERR_MSG("failed to send message for erase\n");
		return -EIO;
		}
	if (mit_isc_check_status(ts) < 0) {
		TOUCH_ERR_MSG("failed to erase check status\n");
		return -EIO;
	}
	if (mit_isc_verify_erased(ts) < 0) {
		TOUCH_ERR_MSG("failed to erase verify\n");
		return -EIO;
	}

	TOUCH_INFO_MSG("IC F/W Erased\n");

	return 0;
}

static int mit_fw_version_check(struct mms_data *ts, struct touch_fw_info *info)
{
	char ver[2] = {0};
	u8 target[2] = {0};

	TOUCH_TRACE_FUNC();

	if (info->fw->size != 64*1024) {
		TOUCH_INFO_MSG("F/W file is not for MIT-200\n");
		return 1;
	}

	if (memcmp("T2H0", &info->fw->data[0xFFF0], 4)) {
		TOUCH_INFO_MSG("F/W file is not for MIT-200\n");
		return 1;
	}

	/*if (strcmp(ts->pdata->fw_product, ts->module.product_code) != 0) {
		TOUCH_INFO_MSG("F/W Product is not matched [%s] \n", ts->module.product_code);
		if (ts->module.product_code[0] == 0)
			return 2;
	} else */{
		if (mms_i2c_read(ts->client, MIT_FW_VERSION, ver, 2) < 0) {
			TOUCH_INFO_MSG("F/W Version read fail \n");
			return 0;
		}

		TOUCH_INFO_MSG("IC Version   : %X.%02X\n",ver[0],ver[1]);

		target[0] = info->fw->data[0xFFFA];
		target[1] = info->fw->data[0xFFFB];

		TOUCH_INFO_MSG("File Version : %X.%02X\n", target[0], target[1]);

		if (ver[0] == target[0] && ver[1] == target[1]) {
			TOUCH_INFO_MSG("F/W is already updated \n");
			return 1;
		} else {
			return 2;
		}
	}

	return 1;
}

int mit_isc_exit(struct mms_data *ts)
{
	struct i2c_client *client = ts->client;
	u8 cmd[6] = ISC_EXIT;

	TOUCH_TRACE_FUNC();

	if (i2c_master_send(client, cmd, 6) != 6) {
		TOUCH_INFO_MSG("failed to isc exit\n");
		return -1;
	}

	return 0;
}

static int mit_isc_page_write(struct mms_data *ts, const u8 *wdata, int addr)
{
	struct i2c_client *client = ts->client;
	u8 cmd[FW_BLOCK_SIZE + 6] = ISC_PAGE_WRITE;

	cmd[4] = (addr & 0xFF00) >> 8;
	cmd[5] = (addr & 0x00FF) >> 0;

	memcpy(&cmd[6], wdata, FW_BLOCK_SIZE);

	if (i2c_master_send(client, cmd, FW_BLOCK_SIZE+6 )!= FW_BLOCK_SIZE+6) {
		TOUCH_INFO_MSG("failed to f/w write\n");
		return -1;
	}

	if (mit_isc_check_status(ts) < 0) {
		TOUCH_ERR_MSG("failed to check writing status\n");
		return -1;
		}

	return 0;
}

int mit_isc_page_read(struct mms_data *ts, u8 *rdata, int addr)
{
	struct i2c_client *client = ts->client;
	u8 cmd[6] = ISC_FLASH_READ;
	struct i2c_msg msg[]={
		{
			.addr = client->addr,
			.flags = 0,
			.buf = cmd,
			.len = 6,
		},{
			.addr=client->addr,
			.flags = I2C_M_RD,
			.buf = rdata,
			.len = FW_BLOCK_SIZE,
		}
	};

	cmd[4] = (addr&0xFF00)>>8;
	cmd[5] = (addr&0x00FF)>>0;

	return (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) != ARRAY_SIZE(msg));
}

static int mit_flash_section(struct mms_data *ts, struct touch_fw_info *info, bool compare)
{
	int addr = 0;
	u8 cmpdata[FW_BLOCK_SIZE] = {0};
	int wrttensize = 0;

	TOUCH_TRACE_FUNC();

	TOUCH_INFO_MSG("F/W Writing... \n");

	for(addr = ((int)info->fw->size) - FW_BLOCK_SIZE; addr >= 0; addr -= FW_BLOCK_SIZE ) {
		if ( mit_isc_page_write(ts, &info->fw->data[addr], addr) ) {
			return -1;
		}

		if (compare) {
			if ( mit_isc_page_read(ts, cmpdata, addr) ) {
				TOUCH_INFO_MSG("F/W Read for verify failed \n");
				return -1;
			}

			if (memcmp(&info->fw->data[addr], cmpdata, FW_BLOCK_SIZE)) {
				TOUCH_INFO_MSG("Addr[0x%06x] Verify failed \n", addr);

				print_hex_dump(KERN_ERR, "[Touch] W : ",
					DUMP_PREFIX_OFFSET, 16, 1, &info->fw->data[addr], FW_BLOCK_SIZE, false);

				print_hex_dump(KERN_ERR, "[Touch] R : ",
					DUMP_PREFIX_OFFSET, 16, 1, cmpdata, FW_BLOCK_SIZE, false);

				return -1;
			}
		}

		wrttensize += FW_BLOCK_SIZE;
		if (wrttensize % (FW_BLOCK_SIZE * 50) == 0) {
			TOUCH_INFO_MSG("\t Updated %5d / %5d bytes\n", wrttensize, (int)info->fw->size);
		}
	}

	TOUCH_INFO_MSG("\t Updated %5d / %5d bytes\n", wrttensize, (int)info->fw->size);

	return 0;
}

int mit_isc_fwupdate(struct mms_data *ts, struct touch_fw_info *info)
{
	int retires = MAX_RETRY_COUNT;
	int ret = 0;
	int i = 0;

	TOUCH_TRACE_FUNC();

	while (retires--) {
		ret = mit_fw_version_check(ts, info);
		if (ret) {
			break;
		} else {
			mms_power_reset(ts);
		}
	}
	mms_power_reset(ts);
	retires = MAX_RETRY_COUNT;

	if (ret == 0) {
		TOUCH_INFO_MSG("IC Error detected. F/W force update \n");
		goto START;
	} else if (ret == 2) {
		goto START;
	}

	if (info->force_upgrade) {
		TOUCH_INFO_MSG("F/W force update \n");
		goto START;
	}

	return 0;

START :
	for (i = 0; i < MAX_RETRY_COUNT; i++) {
		ret = mit_isc_mass_erase(ts);

		mms_power_reset(ts);

		if (ret == 0) {
			if (info->eraseonly) {
				return 0;
			}

			ret = mit_flash_section(ts, info, false);
			if (ret == 0) {
				break;
			} else {
				TOUCH_INFO_MSG("F/W Writing Failed (%d/%d) \n", i + 1, MAX_RETRY_COUNT);
			}
		} else {
			TOUCH_INFO_MSG("F/W erase failed (%d/%d) \n", i+1, MAX_RETRY_COUNT);
		}
	}

	mit_isc_exit(ts);

	return ret;
}

static int get_intensity(struct mms_data *ts)
{
	struct i2c_client *client = ts->client;
	int col = 0;
	int row = 0;
	u8 write_buf[8] = {0};
	u8 read_buf[60] = {0};
	u8 nLength = 0;
	s16 temp_data[MAX_COL][MAX_ROW]={{0}};

	TOUCH_TRACE_FUNC();

	for(col = 0 ; col < ts->dev.col_num ; col++) {
		write_buf[0] = MIT_REGH_CMD;
		write_buf[1] = MIT_REGL_UCMD;
		write_buf[2] = 0x70;
		write_buf[3] = 0xFF;
		write_buf[4] = col;

		if (i2c_master_send(client,write_buf, 5) != 5) {
			TOUCH_INFO_MSG("intensity i2c send failed\n");
			return -1;
		}

		write_buf[0] = MIT_REGH_CMD;
		write_buf[1] = MIT_REGL_UCMD_RESULT_LENGTH;

		if (i2c_master_send(client,write_buf, 2) != 2) {
			TOUCH_INFO_MSG("send : i2c failed\n");
			return -1;
		}

		if (i2c_master_recv(client,read_buf, 1) != 1) {
			TOUCH_INFO_MSG("recv : i2c failed\n");
			return -1;
		}

		nLength = read_buf[0];
		write_buf[0] = MIT_REGH_CMD;
		write_buf[1] = MIT_REGL_UCMD_RESULT;

		if (i2c_master_send(client, write_buf,2) != 2) {
			TOUCH_INFO_MSG("send : i2c failed\n");
			return -1;
		}

		if (i2c_master_recv(client,read_buf, nLength) != nLength) {
			TOUCH_INFO_MSG("recv : i2c failed\n");
			return -1;
		}

		nLength >>= 1;
		for(row = 0 ; row <nLength ; row++) {
			temp_data[col][row] = (s16)(read_buf[2*row] | (read_buf[2*row+1] << 8));
		}

	}

	for(row = 0; row < MAX_ROW; row++) {
		for(col = 0; col < MAX_COL; col++) {
			ts->intensity_data[row][col] = temp_data[col][row];
		}
	}


	return 0;
}

static int  print_intensity(struct mms_data *ts, char *buf) {
	int col = 0;
	int row = 0;
	int ret = 0;

	ret += sprintf(buf + ret, "Start-Intensity\n\n");

	for(row = 0 ; row < MAX_ROW ; row++) {
		printk("[Touch] [%2d]  ", row);
		ret += sprintf(buf + ret,"[%2d]  ", row);
		for(col = 0 ; col < MAX_COL ; col++) {
			ret += sprintf(buf + ret,"%4d ", ts->intensity_data[row][col]);
			printk("%4d ", ts->intensity_data[row][col]);
		}
		printk("\n");
		ret += sprintf(buf + ret,"\n");
	}

	return ret;
}

ssize_t mit_delta_show(struct i2c_client *client, char *buf)
{
	struct mms_data *ts = (struct mms_data *) get_touch_handle_(client);
	int i = 0;
	int ret = 0;

	TOUCH_TRACE_FUNC();

	for( i = 0 ; i < MAX_ROW ; i++) {
		memset(ts->intensity_data[i], 0, sizeof(uint16_t) * MAX_COL);
	}

	touch_disable(ts->client->irq);
	if (get_intensity(ts) == -1) {
		TOUCH_INFO_MSG("intensity printf failed");
		goto error;
	}
	ret = print_intensity(ts, buf);
	if ( ret < 0) {
		TOUCH_ERR_MSG("fail to print intensity data\n");
		goto error;
	}

	touch_enable(ts->client->irq);

	return ret;

error :
	touch_enable(ts->client->irq);
	return -1;

}


static int mit_enter_test(struct mms_data *ts) {
	struct i2c_client *client = ts->client;
	u8 cmd[3] ={ MIT_REGH_CMD, MIT_REGL_UCMD, MIT_UNIV_ENTER_TESTMODE};
	u8 marker;
	int ret = 0;
	int count = 100;
	int iterator = 0;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = cmd,
			.len = sizeof(cmd),
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = &marker,
			.len = 1,
		},
	};

	if (i2c_transfer(client->adapter, &msg[0], 1)!=1) {
		dev_err(&client->dev,"failed enter Test-Mode\n");
	}
	do{
		do{
			udelay(100);
			iterator ++;
		}while(gpio_get_value(ts->pdata->int_pin) && (iterator < MAX_ITERATOR));
		if (iterator >= MAX_ITERATOR) {
			ret = -1;
			goto TESTOUT;
		}
		msg[0].len = 2;
		cmd[1] = MIT_REGL_EVENT_PKT_SZ;
		if (i2c_transfer(client->adapter, msg ,ARRAY_SIZE(msg))!=ARRAY_SIZE(msg)) {
			dev_err(&client->dev,"ENTER Test-Mode MIT_REGL_EVENT_PKT_SZ ERROR!\n");
			ret = -1;
			goto TESTOUT;
		}
		cmd[1] = MIT_REGL_INPUT_EVENT;

		if (i2c_transfer(client->adapter, msg,ARRAY_SIZE(msg))!=ARRAY_SIZE(msg)) {
			dev_err(&client->dev,"ENTER Test-Mode MIT_REGL_INPUT_EVETN ERROR!\n");
			ret = -1;
			goto TESTOUT;
		}
		if (count) {
			count--;
		}else{
			ret = -1;
			goto TESTOUT;
		}
	}while(marker !=0x0C);
	return ret;

TESTOUT:
	dev_err(&client->dev,"ENTER TEST MODE maker is not 0x0C\n");
	return ret;
}


static int mit_exit_test(struct mms_data *ts) {
	struct i2c_client *client = ts->client;
	u8 cmd[3] ={ MIT_REGH_CMD, MIT_REGL_UCMD, MIT_UNIV_EXIT_TESTMODE};
	u8 marker;
	int ret = 0;
	int count = 100;
	int iterator = 0;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = cmd,
			.len = 3,
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = &marker,
			.len = 1,
		},
	};

	if (i2c_master_send(client, cmd, 3)!=3) {
		dev_err(&client->dev,"failed exit Test-Mode\n");
	}
	do{
		do{
			udelay(100);
			iterator ++;
		} while(gpio_get_value(ts->pdata->int_pin) && (iterator < MAX_ITERATOR));
		if (iterator >= MAX_ITERATOR) {
			ret = -1;
			goto TESTEXITOUT;
		}
		msg[0].len 	= 2;
		cmd[1] = MIT_REGL_EVENT_PKT_SZ;
		if (i2c_transfer(client->adapter, msg,ARRAY_SIZE(msg))!=ARRAY_SIZE(msg)) {
			dev_err(&client->dev,"EXIT Test-Mode MIT_REGL_EVENT_PKT_SZ ERROR!\n");
			ret = -1;
			goto TESTEXITOUT;
		}
		cmd[1] = MIT_REGL_INPUT_EVENT;

		if (i2c_transfer(client->adapter, msg,ARRAY_SIZE(msg))!=ARRAY_SIZE(msg)) {
			dev_err(&client->dev,"EXIT Test-Mode MIT_REGL_INPUT_EVETN ERROR!\n");
			ret = -1;
			goto TESTEXITOUT;
		}
		if (count) {
			count--;
		}else{
			ret = -1;
			goto TESTEXITOUT;
		}
	}while(marker !=0x0C);

	return ret;

TESTEXITOUT:
	dev_err(&client->dev,"exit TEST MODE maker is not 0x0C\n");
	return ret;
}


static int mit_select_test(struct mms_data *ts) {

	struct i2c_client *client = ts->client;
	u8 cmd[3] ={ MIT_REGH_CMD, MIT_REGL_UCMD, };
	u8 marker;
	int ret = 0;
	int count = 100;
	int iterator = 0;
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = cmd,
			.len = sizeof(cmd),
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = &marker,
			.len = 1,
		},
	};

	switch (ts->test_mode) {
	case RAW_DATA_SHOW:
	case RAW_DATA_STORE:
	case SLOPE:
		cmd[2] = MIT_UNIV_TESTA_START;
		break;
	case OPENSHORT :
	case CRACK_CHECK :
		cmd[2] = MIT_UNIV_TESTB_START;
		break;
	}

	if (i2c_transfer(client->adapter,&msg[0], 1)!=1) {
		dev_err(&client->dev,"failed enter Test-Mode\n");
	}
	do{
		do{
			udelay(100);
			iterator ++;
		}while(gpio_get_value(ts->pdata->int_pin) && (iterator < MAX_ITERATOR));
		if (iterator >= MAX_ITERATOR) {
			ret = -1;
			goto TESTOUT;
		}
		msg[0].len =2;
		cmd[1] = MIT_REGL_EVENT_PKT_SZ;
		if (i2c_transfer(client->adapter, msg,ARRAY_SIZE(msg))!=ARRAY_SIZE(msg)) {
			dev_err(&client->dev,"SELECT Test-Mode MIT_REGL_EVENT_PKT_SZ ERROR!\n");
			ret = -1;
			goto TESTOUT;
		}
		cmd[1] = MIT_REGL_INPUT_EVENT;
		if (i2c_transfer(client->adapter, msg,ARRAY_SIZE(msg))!=ARRAY_SIZE(msg)) {
			dev_err(&client->dev,"SELECT Test-Mode MIT_REGL_INPUT_EVETN ERROR!\n");
			ret = -1;
			goto TESTOUT;
		}
		if (count) {
			count--;
		}else{
			ret = -1;
			goto TESTOUT;
		}
	}while(marker !=0x0C);
	return ret;
TESTOUT:
	dev_err(&client->dev,"ENTER TESTB MODE maker is not 0x0C\n");
	return ret;
}

static int get_rawdata(struct mms_data *ts)
{
	struct i2c_client *client = ts->client;
	int col = 0;
	int row = 0;
	u8 write_buf[8] = {0,};
	u8 read_buf[25 * 8] = {0,};
	u8 nLength = 0;
	u16 nReference = 0;
	uint16_t temp_data[MAX_COL][MAX_ROW]={{0}};
	TOUCH_TRACE_FUNC();


	for(col = 0 ; col < ts->dev.col_num ; col++) {
		write_buf[0] = MIT_REGH_CMD;
		write_buf[1] = MIT_REGL_UCMD;
		write_buf[2] = MIT_UNIV_GET_RAWDATA;
		write_buf[3] = 0xFF;
		write_buf[4] = col;

		if (i2c_master_send(client,write_buf,5) != 5) {
			dev_err(&client->dev, "rawdata i2c send failed\n");
			return -1;
		}

		write_buf[0] = MIT_REGH_CMD;
		write_buf[1] = MIT_REGL_UCMD_RESULT_LENGTH;

		if (i2c_master_send(client,write_buf,2) != 2) {
			dev_err(&client->dev,"send : i2c failed\n");
			return -1;
		}

		if (i2c_master_recv(client,read_buf,1) != 1) {
			dev_err(&client->dev,"recv1: i2c failed\n");
			return -1;
		}

		nLength = read_buf[0];
		write_buf[0] = MIT_REGH_CMD;
		write_buf[1] = MIT_REGL_UCMD_RESULT;

		if (i2c_master_send(client,write_buf,2) != 2) {
			dev_err(&client->dev,"send : i2c failed\n");
			return -1;
		}

		if (i2c_master_recv(client,read_buf,nLength) != nLength) {
			dev_err(&client->dev,"recv2: i2c failed\n");
			return -1;
		}

		nLength >>=1;
		for(row = 0 ; row <nLength ; row++) {
			nReference = (u16)(read_buf[2*row] | (read_buf[2*row+1] << 8));
			temp_data[col][row] = nReference;
		}
	}
	for(row = 0; row < MAX_ROW; row++) {
		for(col = 0; col < MAX_COL; col++) {
			ts->mit_data[row][col]	= temp_data[col][row];
			if (ts->mit_data[row][col] < ts->pdata->limit->raw_data_min|| ts->mit_data[row][col] > ts->pdata->limit->raw_data_max)
				ts->pdata->selfdiagnostic_state[0] = 0;
		}
	}
	return 0;
}

static int get_openshort(struct mms_data *ts)
{
	struct i2c_client *client = ts->client;
	int col = 0;
	int row = 0;
	u8 write_buf[8];
	u8 read_buf[MAX_ROW * 8];
	u8 nLength = 0;
	u16 nReference = 0;
	uint16_t temp_data[MAX_COL][MAX_ROW]={{0}};

	TOUCH_TRACE_FUNC();

	for(col = 0 ; col < ts->dev.col_num ; col++) {
		write_buf[0] = MIT_REGH_CMD;
		write_buf[1] = MIT_REGL_UCMD;
		write_buf[2] = MIT_UNIV_GET_OPENSHORT_TEST;
		write_buf[3] = 0xFF;
		write_buf[4] = col;

		if (i2c_master_send(client,write_buf,5)!=5) {
			dev_err(&client->dev, "openshort i2c send failed\n");
			return -1;
		}

		write_buf[0] = MIT_REGH_CMD;
		write_buf[1] = MIT_REGL_UCMD_RESULT_LENGTH;

		if (i2c_master_send(client,write_buf,2)!=2) {
			dev_err(&client->dev,"send : i2c failed\n");
			return -1;
		}

		if (i2c_master_recv(client,read_buf,1)!=1) {
			dev_err(&client->dev,"recv1: i2c failed\n");
			return -1;
		}

		nLength = read_buf[0];
		write_buf[0] = MIT_REGH_CMD;
		write_buf[1] = MIT_REGL_UCMD_RESULT;

		if (i2c_master_send(client,write_buf,2)!=2) {
			dev_err(&client->dev,"send : i2c failed\n");
			return -1;
		}

		if (i2c_master_recv(client,read_buf,nLength)!=nLength) {
			dev_err(&client->dev,"recv2: i2c failed\n");
			return -1;
		}

		nLength >>=1;
		for(row = 0 ; row <nLength ; row++) {
			nReference = (u16)(read_buf[2*row] | (read_buf[2*row+1] << 8));
			temp_data[col][row] = nReference;
		}
	}

	for(row = 0; row < MAX_ROW ; row++) {
		for(col = 0; col < MAX_COL; col++) {
			ts->mit_data[row][col] = temp_data[col][row];
			if (ts->mit_data[row][col] < ts->pdata->limit->open_short_min || ts->mit_data[row][col] > ts->pdata->limit->open_short_max) {
				ts->pdata->selfdiagnostic_state[1] = 0;
				ts->count_short++;
			}
		}
	}

	return 0;
}

static int  print_rawdata(struct mms_data *ts, char *buf,int type) {
	int col = 0;
	int row = 0;
	int ret = 0;
	int min = 0;
	int max = 0;

	min = ts->mit_data[0][0];
	max = ts->mit_data[0][0];

	for(row = 0 ; row < MAX_ROW ; row++) {
		if (type == RAW_DATA_SHOW) {
			ret += sprintf(buf + ret,"[%2d]  ",row);
			printk("[Touch] [%2d]  ",row);
		}
		for(col = 0 ; col < MAX_COL ; col++) {
			min = (min > ts->mit_data[row][col]) ? ts->mit_data[row][col] : min;
			max = (max < ts->mit_data[row][col]) ? ts->mit_data[row][col] : max;
			ret += sprintf(buf + ret,"%5d ", ts->mit_data[row][col]);
			printk("%5d ", ts->mit_data[row][col]);
			if (type == RAW_DATA_STORE) {
				ret += sprintf(buf + ret,",");
			}
		}
		if (type == RAW_DATA_SHOW) {
			ret += sprintf(buf + ret,"\n");
			printk("\n");
		}
	}
	if (type == RAW_DATA_SHOW) {
			ret += sprintf(buf + ret,"MAX = %d,  MIN = %d  (MAX - MIN = %d)\n\n",max , min, max - min);
			TOUCH_INFO_MSG("MAX : %d,  MIN : %d  (MAX - MIN = %d)\n\n",max , min, max - min);
	}

	return ret;
}

static int  print_openshort_data(struct mms_data *ts, char *buf) {
	int col = 0;
	int row = 0;
	int ret = 0;

	for(row = 0 ; row < MAX_ROW ; row++) {
		printk("[Touch] [%2d]  ", row);
		ret += sprintf(buf + ret,"[%2d]  ", row);
		for(col = 0 ; col < MAX_COL ; col++) {
			printk("%5d ", ts->mit_data[row][col]);
			ret += sprintf(buf + ret,"%5d ", ts->mit_data[row][col]);
		}
		printk("\n");
		ret += sprintf(buf + ret,"\n");
	}
	printk("\n");
	ret += sprintf(buf + ret,"\n");

	TOUCH_INFO_MSG("OPEN / SHORT TEST SPEC(UPPER : %d  LOWER : %d)\n", ts->pdata->limit->open_short_max, ts->pdata->limit->open_short_min);
	ret += sprintf(buf + ret,"OPEN / SHORT TEST SPEC(UPPER : %d  LOWER : %d)\n", ts->pdata->limit->open_short_max, ts->pdata->limit->open_short_min);

	if (ts->pdata->selfdiagnostic_state[1] == 0 || ret == 0) {
		for(row = 0 ; row < MAX_ROW ; row++) {
			printk("[Touch] [%2d]  ",row);
			ret += sprintf(buf + ret,"[%2d]  ",row);
			for(col = 0 ; col < MAX_COL ; col++) {
				if (ts->mit_data[row][col] <= ts->pdata->limit->open_short_max && ts->mit_data[row][col] >= ts->pdata->limit->open_short_min ) {
					printk(" ,");
					ret += sprintf(buf + ret," ,");
				}else{
					printk("X,");
					ret += sprintf(buf + ret,"X,");
					ts->pdata->selfdiagnostic_state[1] = 0;
				}
			}
			printk("\n");
			ret += sprintf(buf + ret,"\n");
		}
		TOUCH_INFO_MSG("OpenShort Test : Fail\n\n");
		ret += sprintf(buf + ret,"OpenShort Test : FAIL\n\n");
	} else {
		TOUCH_INFO_MSG("OpenShort Test : Pass\n\n");
		ret += sprintf(buf + ret,"OpenShort Test : PASS\n\n");
	}
	return ret;
}

static int  check_slope_data(struct mms_data *ts, char *buf) {
	int row = 0;
	int col = 0;
	int ret = 0;
	uint16_t get_data[MAX_ROW][MAX_COL]={{0}};

	for(col = 0 ; col < MAX_COL ; col++) {
		for(row = 1 ; row < MAX_ROW - 1 ; row++) {
			get_data[row][col] = (2 * ts->mit_data[row][col] * 100)/(ts->mit_data[row - 1][col] + ts->mit_data[row + 1][col]);
			if (get_data[row][col] > 999) {
				get_data[row][col] = 999;
			}
			if (get_data[row][col] < ts->pdata->limit->slope_min || get_data[row][col] > ts->pdata->limit->slope_max)
				ts->pdata->selfdiagnostic_state[2] = 0;
		}
	}
	for(row = 1 ; row < MAX_ROW - 1 ; row++) {
		ret += sprintf(buf + ret,"[%2d]  ",row);
		printk("[Touch] [%2d]  ",row);

		for(col = 0 ; col < MAX_COL ; col++) {
			ret += sprintf(buf + ret,"%3d ", get_data[row][col]);
			printk("%3d ", get_data[row][col]);
		}
		printk("\n");
		ret += sprintf(buf + ret,"\n");
	}
	printk("\n");
	ret += sprintf(buf + ret,"\n");

	ret += sprintf(buf + ret,"Slope Spec(UPPER : %d  LOWER : %d)\n", ts->pdata->limit->slope_max, ts->pdata->limit->slope_min);
	TOUCH_INFO_MSG("Slope Spec(UPPER : %d  LOWER : %d)\n", ts->pdata->limit->slope_max, ts->pdata->limit->slope_min);

	if (ts->pdata->selfdiagnostic_state[2] == 0) {
		for(row = 1 ; row < MAX_ROW-1 ; row++) {
			ret += sprintf(buf + ret,"[%2d]  ",row);
			printk("[Touch] [%2d]  ",row);
			for(col = 0 ; col < MAX_COL ; col++) {
				if (get_data[row][col] >= ts->pdata->limit->slope_min && get_data[row][col] <= ts->pdata->limit->slope_max) {
					ret += sprintf(buf + ret," ,");
					printk(" ,");
				} else {
					ret += sprintf(buf + ret,"X,");
					printk("X,");
				}
			}
			printk("\n");
			ret += sprintf(buf + ret,"\n");
		}

		TOUCH_INFO_MSG("Slope : FAIL\n\n");
		ret += sprintf(buf + ret,"Slope : FAIL\n\n");
	} else {
		TOUCH_INFO_MSG("Slope : PASS\n\n");
		ret += sprintf(buf + ret,"Slope : PASS\n\n");
	}

	return ret;
}

ssize_t mit_get_test_result(struct i2c_client *client, char *buf, int type)
{
	struct mms_data *ts = (struct mms_data *) get_touch_handle_(client);
	char temp_buf[255] = {0,};
	int i = 0;
	int ret = 0;
	int fd = 0;
	char data_path[64] = {0,};
	char *read_buf;
	mm_segment_t old_fs = get_fs();

	for( i = 0 ; i < MAX_ROW ; i++) {
		memset(ts->mit_data[i], 0, sizeof(uint16_t) * MAX_COL);
	}

	read_buf=kzalloc(sizeof(u8)*4096,GFP_KERNEL);
	if (read_buf == NULL) {
		TOUCH_ERR_MSG("read_buf mem_error");
		goto mem_error;
	}

	ts->test_mode = type;

	if (mit_enter_test(ts) == -1) {
			TOUCH_ERR_MSG("test enter failed");
			goto error;
		}
		if (mit_select_test(ts)==-1) {
			TOUCH_ERR_MSG("test select failed");
			goto error;
		}
	if (ts->test_mode == RAW_DATA_SHOW || ts->test_mode == RAW_DATA_STORE || ts->test_mode == SLOPE) {
			if (get_rawdata(ts) == -1) {
				TOUCH_ERR_MSG("getting raw data failed");
				goto error;
			}
	} else {
		if (get_openshort(ts) == -1) {
			TOUCH_ERR_MSG("getting open_short data failed");
			goto error;
		}
	}
	if (mit_exit_test(ts)==-1) {
		TOUCH_ERR_MSG("test exit failed");
		goto error;
	}
	switch(type) {
		case RAW_DATA_SHOW:
			ret = print_rawdata(ts, buf, type);
			if (ret < 1) {
				TOUCH_ERR_MSG("fail to print raw data\n");
				ts->pdata->selfdiagnostic_state[0] = 0;
				goto error;
			}
			break;
		case RAW_DATA_STORE:
			snprintf(temp_buf,strlen(buf),"%s",buf);
			sprintf(data_path,"/sdcard/%s.csv",temp_buf);

			ret = print_rawdata(ts, read_buf, type);
			if (ret < 1) {
				TOUCH_ERR_MSG("fail to print raw data\n");
				ts->pdata->selfdiagnostic_state[0] = 0;
				goto error;
			}
			set_fs(KERNEL_DS);
			fd = sys_open(data_path, O_WRONLY|O_CREAT, 0666);
			if (fd >= 0) {
				sys_write(fd, read_buf, 4096);
				sys_close(fd);
				TOUCH_INFO_MSG("%s saved \n", data_path);
			} else {
				TOUCH_INFO_MSG("%s open failed \n", data_path);
			}
			set_fs(old_fs);
			break;
		case OPENSHORT :
			ret = print_openshort_data(ts,buf);
			if ( ret < 1) {
				TOUCH_ERR_MSG("fail to print open short data\n");
				ts->pdata->selfdiagnostic_state[1] = 0;
				goto error;
			}
			break;
		case SLOPE :
			ret = check_slope_data(ts, buf);
			if ( ret < 1) {
				TOUCH_ERR_MSG("fail to print open short data\n");
				ts->pdata->selfdiagnostic_state[2] = 0;
				goto error;
			}
			break;
		case CRACK_CHECK :
			break;
		default :
			TOUCH_INFO_MSG("type = default[%d]\n",type);
			break;
		}

	if (read_buf != NULL)
		kfree(read_buf);
	return ret;

error :
	if (read_buf != NULL)
		kfree(read_buf);

	return -1;

mem_error :
	if (read_buf != NULL)
		kfree(read_buf);
	return -1;
}
