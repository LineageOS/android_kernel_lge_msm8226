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

#include "lge_ts_melfas.h"

#define ISC_MASS_ERASE				{0xFB, 0x4A, 0x00, 0x15, 0x00, 0x00}
#define ISC_PAGE_WRITE				{0xFB, 0x4A, 0x00, 0x5F, 0x00, 0x00}
#define ISC_FLASH_READ				{0xFB, 0x4A, 0x00, 0xC2, 0x00, 0x00}
#define ISC_STATUS_READ				{0xFB, 0x4A, 0x00, 0xC8, 0x00, 0x00}
#define ISC_EXIT						{0xFB, 0x4A, 0x00, 0x66, 0x00, 0x00}

struct isc_packet {
	u8 	cmd;
	u32	addr;
	u8	data[0];
} __attribute__ ((packed));

static int mit_isc_mass_erase(struct mms_data *ts)
{
	char tmp[6] = ISC_MASS_ERASE;

	TOUCH_TRACE_FUNC();

	if (mms_i2c_write_block(ts->client, tmp, 6) < 0)
		return -EIO;

	memset(ts->module.product_code, 0, sizeof(ts->module.product_code));
	memset(ts->module.version, 0, sizeof(ts->module.version));

	TOUCH_INFO_MSG("F/W Erased\n");

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

	if (strcmp(ts->pdata->fw_product, ts->module.product_code) != 0) {
		TOUCH_INFO_MSG("F/W Product is not matched [%s] \n", ts->module.product_code);
	} else {
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
			TOUCH_INFO_MSG("failed to status read\n");
			return -1;
		}

		if ( buf == 0xAD ) {
			return 0;
		}

		msleep(1);
	}
	TOUCH_INFO_MSG("failed to status read\n");
	return -1;
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

	if ( mit_isc_check_status(ts) )
		return -1;

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
		mms_power_reset(ts);

		mit_isc_mass_erase(ts);

		if (info->eraseonly) {
			return 0;
		}

		ret = mit_flash_section(ts, info, true);
		if (ret == 0) {
			break;
		} else {
			TOUCH_INFO_MSG("F/W Writing Failed (%d/%d) \n", i + 1, MAX_RETRY_COUNT);
		}
	}

	mit_isc_exit(ts);

	return ret;
}

static int get_intensity(struct mms_data *ts)
{
	struct i2c_client *client = ts->client;
	int col = 0, row = 0;
	char tmp_data[255] = {0};
	u8 write_buf[8] = {0};
	u8 read_buf[60] = {0};
	u8 nLength = 0;
	s16 nIntensity = 0;

	TOUCH_TRACE_FUNC();

	memset(ts->get_data, 0, sizeof(ts->get_data));
	sprintf(tmp_data, "Start-Intensity\n\n");
	strcat(ts->get_data, tmp_data);
	memset(tmp_data, 0, 255);
	disable_irq(ts->client->irq);

	for(col = 0 ; col < ts->dev.col_num ; col++) {
		printk("[%2d]  ", col);

		sprintf(tmp_data,"[%2d]  ", col);
		strcat(ts->get_data,tmp_data);

		write_buf[0] = MIT_REGH_CMD;
		write_buf[1] = MIT_REGL_UCMD;
		write_buf[2] = 0x70;
		write_buf[3] = 0xFF;
		write_buf[4] = col;

		if (i2c_master_send(client,write_buf, 5) != 5) {
			TOUCH_INFO_MSG("intensity i2c send failed\n");
			enable_irq(ts->client->irq);
			return -1;
		}

		write_buf[0] = MIT_REGH_CMD;
		write_buf[1] = MIT_REGL_UCMD_RESULT_LENGTH;

		if (i2c_master_send(client,write_buf, 2) != 2) {
			TOUCH_INFO_MSG("send : i2c failed\n");
			enable_irq(ts->client->irq);
			return -1;
		}

		if (i2c_master_recv(client,read_buf, 1) != 1) {
			TOUCH_INFO_MSG("recv : i2c failed\n");
			enable_irq(ts->client->irq);
			return -1;
		}

		nLength = read_buf[0];
		write_buf[0] = MIT_REGH_CMD;
		write_buf[1] = MIT_REGL_UCMD_RESULT;

		if (i2c_master_send(client, write_buf,2) != 2) {
			TOUCH_INFO_MSG("send : i2c failed\n");
			enable_irq(ts->client->irq);
			return -1;
		}

		if (i2c_master_recv(client,read_buf, nLength) != nLength) {
			TOUCH_INFO_MSG("recv : i2c failed\n");
			enable_irq(ts->client->irq);
			return -1;
		}

		nLength >>= 1;
		for(row = 0 ; row <nLength ; row++) {
			nIntensity = (s16)(read_buf[2*row] | (read_buf[2*row+1] << 8));
			sprintf(tmp_data,"%4d ", nIntensity);
			strcat(ts->get_data, tmp_data);
			printk("%4d ", nIntensity);
		}
		printk("\n");

		sprintf(tmp_data,"\n");
		strcat(ts->get_data, tmp_data);
	}
	enable_irq(ts->client->irq);

	return 0;
}

ssize_t mit_delta_show(struct i2c_client *client, char *buf)
{
	struct mms_data *ts = (struct mms_data *) get_touch_handle_(client);
	int ret = 0;

	TOUCH_TRACE_FUNC();

	ts->get_data=kzalloc(sizeof(u8)*PAGE_SIZE, GFP_KERNEL);
	memset(ts->get_data, 0, sizeof(ts->get_data));

	if (get_intensity(ts)==-1) {
		TOUCH_INFO_MSG("intensity printf failed");
		ret =-1;
		return ret;
	}

	ret = snprintf(buf,PAGE_SIZE,"%s\n", ts->get_data);
	kfree(ts->get_data);
	return ret;
}


static int mit_enter_test(struct mms_data *ts){
	struct i2c_client *client = ts->client;
	u8 cmd[3] ={ MIT_REGH_CMD, MIT_REGL_UCMD, MIT_UNIV_ENTER_TESTMODE};
	u8 marker;
	int ret = 0;
	int count = 100;
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

	if(i2c_transfer(client->adapter, &msg[0], 1)!=1){
		dev_err(&client->dev,"failed enter Test-Mode\n");
	}
	do{
		do{
			udelay(100);
		}while(gpio_get_value(ts->pdata->int_pin));
		msg[0].len = 2;
		cmd[1] = MIT_REGL_EVENT_PKT_SZ;
		if(i2c_transfer(client->adapter, msg ,ARRAY_SIZE(msg))!=ARRAY_SIZE(msg)){
			dev_err(&client->dev,"ENTER Test-Mode MIT_REGL_EVENT_PKT_SZ ERROR!\n");
			ret = -1;
			goto TESTOUT;
		}
		cmd[1] = MIT_REGL_INPUT_EVENT;

		if(i2c_transfer(client->adapter, msg,ARRAY_SIZE(msg))!=ARRAY_SIZE(msg)){
			dev_err(&client->dev,"ENTER Test-Mode MIT_REGL_INPUT_EVETN ERROR!\n");
			ret = -1;
			goto TESTOUT;
		}
		if(count){
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


static int mit_exit_test(struct mms_data *ts){
	struct i2c_client *client = ts->client;
	u8 cmd[3] ={ MIT_REGH_CMD, MIT_REGL_UCMD, MIT_UNIV_EXIT_TESTMODE};
	u8 marker;
	int ret = 0;
	int count = 100;
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

	if(i2c_master_send(client, cmd, 3)!=3){
		dev_err(&client->dev,"failed exit Test-Mode\n");
	}
	do{
		do{
			udelay(100);
		}while(gpio_get_value(ts->pdata->int_pin));
		msg[0].len 	= 2;
		cmd[1] = MIT_REGL_EVENT_PKT_SZ;
		if(i2c_transfer(client->adapter, msg,ARRAY_SIZE(msg))!=ARRAY_SIZE(msg)){
			dev_err(&client->dev,"exit Test-Mode MIT_REGL_EVENT_PKT_SZ ERROR!\n");
			ret = -1;
			goto TESTEXITOUT;
		}
		cmd[1] = MIT_REGL_INPUT_EVENT;

		if(i2c_transfer(client->adapter, msg,ARRAY_SIZE(msg))!=ARRAY_SIZE(msg)){
			dev_err(&client->dev,"exit Test-Mode MIT_REGL_INPUT_EVETN ERROR!\n");
			ret = -1;
			goto TESTEXITOUT;
		}
		if(count){
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


static int mit_select_test(struct mms_data *ts){

	struct i2c_client *client = ts->client;
	u8 cmd[3] ={ MIT_REGH_CMD, MIT_REGL_UCMD, };
	u8 marker;
	int ret = 0;
	int count = 100;
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
	case RAW_DATA :
		cmd[2] = MIT_UNIV_TESTA_START;
		break;
	case OPENSHORT :
		cmd[2] = MIT_UNIV_TESTB_START;
		break;
	}

	if(i2c_transfer(client->adapter,&msg[0], 1)!=1){
		dev_err(&client->dev,"failed enter TestB-Mode\n");
	}
	do{
		do{
			udelay(100);
		}while(gpio_get_value(ts->pdata->int_pin));
		msg[0].len =2;
		cmd[1] = MIT_REGL_EVENT_PKT_SZ;
		if(i2c_transfer(client->adapter, msg,ARRAY_SIZE(msg))!=ARRAY_SIZE(msg)){
			dev_err(&client->dev,"ENTER TestB-Mode MIT_REGL_EVENT_PKT_SZ ERROR!\n");
			ret = -1;
			goto TESTOUT;
		}
		cmd[1] = MIT_REGL_INPUT_EVENT;
		if(i2c_transfer(client->adapter, msg,ARRAY_SIZE(msg))!=ARRAY_SIZE(msg)){
			dev_err(&client->dev,"ENTER TestB-Mode MIT_REGL_INPUT_EVETN ERROR!\n");
			ret = -1;
			goto TESTOUT;
		}
		if(count){
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
	int ret = 0;
	int i = 0;
	char tmp_data[255] = {0,};
	u8 write_buf[8] = {0,};
	u8 read_buf[25 * 8] = {0,};
	u8 nLength = 0;
	u16 nReference = 0;
	uint16_t *raw_data[MAX_COL];


	for(i=0;i<MAX_COL;i++){
		raw_data[i] = kzalloc(sizeof(uint16_t)*MAX_ROW,GFP_KERNEL);
		if(raw_data[i]==NULL)
			TOUCH_ERR_MSG("raw_data kzalloc error");
	}

	TOUCH_TRACE_FUNC();

	memset(ts->get_data, 0, sizeof(ts->get_data));
	ret = snprintf(tmp_data,PAGE_SIZE,"Start-rawdata\n\n");
	strcat(ts->get_data,tmp_data);
	memset(tmp_data,0,255);
	for(col = 0 ; col < ts->dev.col_num ; col++)
	{
		printk("[%2d]  ",col);
		ret += snprintf(tmp_data,PAGE_SIZE,"[%2d]  ",col);
		strcat(ts->get_data,tmp_data);

		write_buf[0] = MIT_REGH_CMD;
		write_buf[1] = MIT_REGL_UCMD;
		write_buf[2] = MIT_UNIV_GET_RAWDATA;
		write_buf[3] = 0xFF;
		write_buf[4] = col;

		if(i2c_master_send(client,write_buf,5)!=5)
		{
			dev_err(&client->dev, "rawdata i2c send failed\n");
			return -1;
		}

		write_buf[0] = MIT_REGH_CMD;
		write_buf[1] = MIT_REGL_UCMD_RESULT_LENGTH;

		if(i2c_master_send(client,write_buf,2)!=2)
		{
			dev_err(&client->dev,"send : i2c failed\n");
			return -1;
		}

		if(i2c_master_recv(client,read_buf,1)!=1)
		{
			dev_err(&client->dev,"recv1: i2c failed\n");
			return -1;
		}

		nLength = read_buf[0];
		write_buf[0] = MIT_REGH_CMD;
		write_buf[1] = MIT_REGL_UCMD_RESULT;

		if(i2c_master_send(client,write_buf,2)!=2)
		{
			dev_err(&client->dev,"send : i2c failed\n");
			return -1;
		}

		if(i2c_master_recv(client,read_buf,nLength)!=nLength)
		{
			dev_err(&client->dev,"recv2: i2c failed\n");
			return -1;
		}

		nLength >>=1;
		for(row = 0 ; row <nLength ; row++)
		{
			nReference = (u16)(read_buf[2*row] | (read_buf[2*row+1] << 8));
			raw_data[col][row] = nReference;
			printk("%5d,", nReference);
			ret += snprintf(tmp_data,PAGE_SIZE,"%5d,", nReference);
			strcat(ts->get_data,tmp_data);
		}
		printk("\n");
		ret += snprintf(tmp_data,PAGE_SIZE,"\n");
		strcat(ts->get_data,tmp_data);
	}

	ret += snprintf(tmp_data,PAGE_SIZE,"\n");
	strcat(ts->get_data,tmp_data);
	for(col = 0 ; col < ts->dev.col_num ; col++){
		printk("[%2d]  ",col);
		ret += snprintf(tmp_data,PAGE_SIZE,"[%2d]  ",col);
		strcat(ts->get_data,tmp_data);
		for(row = 0 ; row <nLength ; row++){
			if(raw_data[col][row] <= ts->raw_data_max[row] && raw_data[col][row] >= ts->raw_data_min[row] ){
					printk(" ,");
					ret += snprintf(tmp_data,PAGE_SIZE," ,");
					strcat(ts->get_data,tmp_data);
				}else{
					printk("X,");
					ret += snprintf(tmp_data,PAGE_SIZE,"X,");
					strcat(ts->get_data,tmp_data);
				}

		}
		printk("\n");
		ret += snprintf(tmp_data,PAGE_SIZE,"\n");
		strcat(ts->get_data,tmp_data);
	}

	for(i=0;i<MAX_COL;i++)
		kfree(raw_data[i]);

	return 0;
}

static int get_openshort(struct mms_data *ts)
{
	struct i2c_client *client = ts->client;
	int col = 0;
	int row = 0;
	int i = 0;
	char tmp_data[255] = {0,};
	u8 write_buf[8];
	u8 read_buf[MAX_ROW * 8];
	u8 nLength = 0;
	u16 nReference = 0;
	uint16_t *openshort_data[MAX_COL];

	TOUCH_TRACE_FUNC();
	
	for(i=0;i<MAX_COL;i++) {
		openshort_data[i] = kzalloc(sizeof(uint16_t)*MAX_ROW,GFP_KERNEL);
		if(openshort_data[i]==NULL)
			TOUCH_ERR_MSG("openshort kzalloc error");
	}

	memset(ts->get_data,0,sizeof(ts->get_data));
	sprintf(tmp_data,"Start-openshort-TEST\n\n");
	strcat(ts->get_data,tmp_data);
	memset(tmp_data,0,255);
	for(col = 0 ; col < ts->dev.col_num ; col++)
	{
		printk("[%2d]  ",col);
		sprintf(tmp_data,"[%2d]  ",col);
		strcat(ts->get_data,tmp_data);

		write_buf[0] = MIT_REGH_CMD;
		write_buf[1] = MIT_REGL_UCMD;
		write_buf[2] = MIT_UNIV_GET_OPENSHORT_TEST;
		write_buf[3] = 0xFF;
		write_buf[4] = col;

		if(i2c_master_send(client,write_buf,5)!=5)
		{
			dev_err(&client->dev, "openshort i2c send failed\n");
			return -1;
		}

		write_buf[0] = MIT_REGH_CMD;
		write_buf[1] = MIT_REGL_UCMD_RESULT_LENGTH;

		if(i2c_master_send(client,write_buf,2)!=2)
		{
			dev_err(&client->dev,"send : i2c failed\n");
			return -1;
		}

		if(i2c_master_recv(client,read_buf,1)!=1)
		{
			dev_err(&client->dev,"recv1: i2c failed\n");
			return -1;
		}

		nLength = read_buf[0];
		write_buf[0] = MIT_REGH_CMD;
		write_buf[1] = MIT_REGL_UCMD_RESULT;

		if(i2c_master_send(client,write_buf,2)!=2)
		{
			dev_err(&client->dev,"send : i2c failed\n");
			return -1;
		}

		if(i2c_master_recv(client,read_buf,nLength)!=nLength)
		{
			dev_err(&client->dev,"recv2: i2c failed\n");
			return -1;
		}

		nLength >>=1;
		for(row = 0 ; row <nLength ; row++)
		{
			nReference = (u16)(read_buf[2*row] | (read_buf[2*row+1] << 8));
			printk("%5d,", nReference);
			openshort_data[col][row] = nReference;
			sprintf(tmp_data,"%5d,", nReference);
			strcat(ts->get_data,tmp_data);
		}
		printk("\n");
		sprintf(tmp_data,"\n");
		strcat(ts->get_data,tmp_data);
	}
	printk("OPEN / SHORT TEST (MIN : %d  MAX : %d) \n",OPENSHORT_MIN ,OPENSHORT_MAX);
	sprintf(tmp_data,"OPEN / SHORT TEST (MIN : %d  MAX : %d) \n",OPENSHORT_MIN ,OPENSHORT_MAX);
	strcat(ts->get_data,tmp_data);

	for(col = 0 ; col < ts->dev.col_num ; col++){
		printk("[%2d]  ",col);
		sprintf(tmp_data,"[%2d]  ",col);
		strcat(ts->get_data,tmp_data);
		for(row = 0 ; row <nLength ; row++){
			if(openshort_data[col][row] <= OPENSHORT_MAX && openshort_data[col][row] >= OPENSHORT_MIN ){
				printk(" ,");
				sprintf(tmp_data," ,");
				strcat(ts->get_data,tmp_data);
			}else{
				printk("X,");
				sprintf(tmp_data,"X,");
				strcat(ts->get_data,tmp_data);
			}
		}
		printk("\n");
		sprintf(tmp_data,"\n");
		strcat(ts->get_data,tmp_data);
	}

	for(i=0;i<MAX_COL;i++)
		kfree(openshort_data[i]);
	
	return 0;
}

ssize_t mit_rawdata_show(struct i2c_client *client,char *buf)
{
	struct mms_data *ts = (struct mms_data *) get_touch_handle_(client);
	int ret = 0;

	ts->get_data=kzalloc(sizeof(u8)*4096,GFP_KERNEL);
	if(ts->get_data == NULL)
		goto mem_error;

	touch_disable(ts);
	if(mit_enter_test(ts)==-1){
		ret = snprintf(buf,PAGE_SIZE,"raw enter failed\n");
		TOUCH_ERR_MSG("raw enter failed");
		goto error;
	}
	if(mit_select_test(ts)==-1){
		ret = snprintf(buf,PAGE_SIZE,"raw enter failed\n");
		TOUCH_ERR_MSG("raw data select failed");
		goto error;
	}
	ret = get_rawdata(ts);
	if(ret == -1){
		ret = snprintf(buf,PAGE_SIZE,"raw printf failed\n");
		TOUCH_ERR_MSG("raw printf failed");
		goto error;
	}

	if(mit_exit_test(ts)==-1){
		TOUCH_ERR_MSG("raw printf failed");
		goto error;
	}

	ret = snprintf(buf,PAGE_SIZE,"%s\n",ts->get_data);
	touch_enable(ts);
	kfree(ts->get_data);
	return ret;

	error :
		touch_enable(ts);
		kfree(ts->get_data);
		return -1;
	mem_error :
		return -1;
}


ssize_t mit_openshort_show(struct i2c_client *client, char *buf)
{
	struct mms_data *ts = (struct mms_data *) get_touch_handle_(client);
	int ret = 0;
	ts->get_data=kzalloc(sizeof(u8)*4096,GFP_KERNEL);
	if(ts->get_data == NULL)
		goto mem_error;

	touch_disable(ts);
	if(mit_enter_test(ts)==-1){
		ret = snprintf(buf,PAGE_SIZE,"openshort enter failed\n");
		TOUCH_ERR_MSG("openshort enter failed");
		goto error;

	}
	if(mit_select_test(ts)==-1){
		ret = snprintf(buf,PAGE_SIZE,"openshort enter failed\n");
		TOUCH_ERR_MSG("openshort select failed");
		goto error;

	}
	if(get_openshort(ts)==-1){
		ret = snprintf(buf,PAGE_SIZE,"openshort printf failed\n");
		TOUCH_ERR_MSG("openshort printf failed");
		goto error;
	}

	if(mit_exit_test(ts)==-1){
		TOUCH_ERR_MSG("raw printf failed");
		goto error;
	}
	ret = snprintf(buf,PAGE_SIZE,"%s\n",ts->get_data);
	touch_disable(ts);
	kfree(ts->get_data);
	return ret;

error :
	touch_enable(ts);
	kfree(ts->get_data);
	return -1;
mem_error :
	return -1;

}

