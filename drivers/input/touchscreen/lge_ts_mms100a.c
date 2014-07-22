/* lge_touch_mms100a.c
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

/* each address addresses 4-byte words */
#define ISP_MAX_FW_SIZE		(0x1F00 * 4)
#define ISP_IC_INFO_ADDR	0x1F00

#define ISP_CAL_INFO_ADDR	7936
#define ISP_CAL_DATA_SIZE	256

enum {
	ISP_MODE_FLASH_ERASE	= 0x59F3,
	ISP_MODE_FLASH_WRITE	= 0x62CD,
	ISP_MODE_FLASH_READ	= 0x6AC9,
};

struct mms_info {
	struct i2c_client		*client;
	struct touch_platform_data	*pdata;
};

static const char crc0[31] = {
	0x1D, 0x2C, 0x05, 0x34, 0x95, 0xA4, 0x8D, 0xBC, 0x59, 0x68, 0x41, 0x70,
	0xD1, 0xE0, 0xC9, 0xF8, 0x3F, 0x0E, 0x27, 0x16, 0xB7, 0x86, 0xAF, 0x9E,
	0x7B, 0x4A, 0x63, 0x52, 0xF3, 0xC2, 0xEB
};

static const char crc1[31] = {
	0x1E, 0x9C, 0xDF, 0x5D, 0x76, 0xF4, 0xB7, 0x35, 0x2A, 0xA8, 0xEB, 0x69,
	0x42, 0xC0, 0x83, 0x01, 0x04, 0x86, 0xC5, 0x47, 0x6C, 0xEE, 0xAD, 0x2F,
	0x30, 0xB2, 0xF1, 0x73, 0x58, 0xDA, 0x99
};

#define is_section_valid(addr) ((addr) > 0 && (addr) <= 30)

#define __calc_crc(seed, crc) \
	do { \
		int _bit_index; \
		u16 send, in, xor_bit_1, xor_bit_2, xor_bit_3; \
		for (_bit_index = 7; _bit_index >= 0; _bit_index--) { \
			in = (seed >> _bit_index) & 0x01; \
			xor_bit_1 = (crc & 0x0001) ^ in; \
			xor_bit_2 = xor_bit_1 ^ (crc >> 11 & 0x01); \
			xor_bit_3 = xor_bit_1 ^ (crc >> 4 & 0x01); \
			send = (xor_bit_1 << 4) | (crc >> 12 & 0x0F); \
			send = (send << 7) | (xor_bit_2 << 6) | (crc >> 5 & 0x3F); \
			send = (send << 4) | (xor_bit_3 << 3) | (crc >> 1 & 0x0007); \
			crc = send; \
		} \
	} while (0)

/* firmware update routine using i2c */
static int mms_100a_firmware_image_parse(struct mms_data *ts, const u8 *image_bin)
{
	int offset = 0;
	int i = 0;

	TOUCH_TRACE_FUNC();

	ts->fw_hdr = (struct mms_bin_hdr *) image_bin;

	if (touch_debug_mask_ & DEBUG_FW_UPGRADE) {
		TOUCH_INFO_MSG("mms_fw_hdr:\n");
		TOUCH_INFO_MSG("\ttag[%c%c%c%c%c%c%c%c]\n",
				ts->fw_hdr->tag[0], ts->fw_hdr->tag[1], ts->fw_hdr->tag[2], ts->fw_hdr->tag[3],
				ts->fw_hdr->tag[4], ts->fw_hdr->tag[5], ts->fw_hdr->tag[6], ts->fw_hdr->tag[7]);
		TOUCH_INFO_MSG("\tcore_version[0x%02x]\n", ts->fw_hdr->core_version);
		TOUCH_INFO_MSG("\tsection_num[%d]\n", ts->fw_hdr->section_num);
		TOUCH_INFO_MSG("\tcontains_full_binary[%d]\n", ts->fw_hdr->contains_full_binary);
		TOUCH_INFO_MSG("\tbinary_offset[%d (0x%04x)]\n", ts->fw_hdr->binary_offset, ts->fw_hdr->binary_offset);
		TOUCH_INFO_MSG("\tbinary_length[%d]\n", ts->fw_hdr->binary_length);
	}

	offset = sizeof(struct mms_bin_hdr);
	for (i = 0; i < ts->fw_hdr->section_num; i++) {
		ts->fw_img[i] = (struct mms_fw_img *) (image_bin + offset);

		offset += sizeof(struct mms_fw_img);

		if (touch_debug_mask_ & DEBUG_FW_UPGRADE) {
			TOUCH_INFO_MSG("mms_fw_hdr[%d:%s]:\n", i, section_name[ts->fw_img[i]->type]);
			TOUCH_INFO_MSG("\ttype[%d]\n", ts->fw_img[i]->type);
			TOUCH_INFO_MSG("\tversion[0x%02x]\n", ts->fw_img[i]->version);
			TOUCH_INFO_MSG("\tstart_page[%d]\n", ts->fw_img[i]->start_page);
			TOUCH_INFO_MSG("\tend_page[%d]\n", ts->fw_img[i]->end_page);
			TOUCH_INFO_MSG("\toffset[%d (0x%04x)]\n", ts->fw_img[i]->offset, ts->fw_img[i]->offset);
			TOUCH_INFO_MSG("\tlength[%d]\n", ts->fw_img[i]->length);
		}
	}

	if (touch_debug_mask_ & DEBUG_FW_UPGRADE)
		TOUCH_INFO_MSG("ts_section version = [0x%02x 0x%02x 0x%02x]\n", ts->ts_section[0].version, ts->ts_section[1].version, ts->ts_section[2].version);

	return 0;
}

static int mms_100a_fw_version_check(struct mms_data *ts){

	bool fw_up_to_date = true;
	int offset = 0;
	int i = 0;

	TOUCH_TRACE_FUNC();

	offset = sizeof(struct mms_bin_hdr);
	for (i = 0; i < ts->fw_hdr->section_num; i++) {

		if(ts->fw_img[i]->version != ts->ts_section[i].version){
			fw_up_to_date = false;
			ts->need_update[i]=true;

			if(ts->fw_img[0]->version != ts->ts_section[0].version){
				ts->need_update[0]=true;
				ts->need_update[1]=true;
				ts->need_update[2]=true;
				return FW_UPDATE_BY_ISP;
			}

			else if(ts->fw_img[1]->version != ts->ts_section[1].version){
				ts->need_update[0]=false;
				ts->need_update[1]=true;
				ts->need_update[2]=true;
				return FW_UPDATE_BY_ISP;
			}

			else if(ts->fw_img[2]->version != ts->ts_section[2].version){
				ts->need_update[0]=false;
				ts->need_update[1]=false;
				ts->need_update[2]=true;
				return FW_UPDATE_BY_ISC;
			}

		}
		offset += sizeof(struct mms_fw_img);
	}

	if (touch_debug_mask_ & DEBUG_FW_UPGRADE)
		TOUCH_INFO_MSG("fw need_update = [%d %d %d]\n", ts->need_update[0], ts->need_update[1], ts->need_update[2]);

	if (fw_up_to_date) {
		if (touch_debug_mask_ & DEBUG_FW_UPGRADE)
			TOUCH_INFO_MSG("mms_ts firmware version is up to date\n");

		return FW_UP_TO_DATE;
	}

	return -EINVAL;
}

static int mms_100a_isc_enter(struct mms_data *ts)
{
	char tmp[2] = { MMS_ENTER_ISC, 0x01 };

	TOUCH_TRACE_FUNC();

	if(mms_i2c_write_block(ts->client, tmp, 2) < 0)
		return -EIO;

	TOUCH_INFO_MSG("isc entered...\n");

	return 0;
}

static int mms_100a_isc_write_enable(struct mms_data *ts)
{
	char tmp[10] = { MMS_WRITE_CMD, MMS_ENABLE_WRITE, };

	TOUCH_TRACE_FUNC();

	if (mms_i2c_write_block(ts->client, tmp, 10) < 0)
		return -EIO;

	if (mms_i2c_read(ts->client, MMS_CONFIRM_STATUS, tmp, 1) < 0)
		return -EIO;

	if (tmp[0] != MMS_STATUS_ISC_READY) {
		TOUCH_ERR_MSG("STATUS_WRITING_DONE check failed\n");
		return -EIO;
	}

	TOUCH_INFO_MSG("isc write enabled...\n");

	return 0;
}

static int mms_100a_isc_clear_page(struct mms_data *ts, u8 page)
{
	char tmp[1] = {0};

	TOUCH_TRACE_FUNC();

	ts->buf[0] = MMS_WRITE_CMD;
	ts->buf[1] = MMS_DATA_WRITE;
	ts->buf[2] = page;
	memset(&ts->buf[PAGE_HEADER], 0xFF, PAGE_DATA);
	ts->buf[PAGE_HEADER + PAGE_DATA] = crc0[page];
	ts->buf[PAGE_HEADER + PAGE_DATA+1] = crc1[page];

	if (mms_i2c_write_block(ts->client, ts->buf, PACKET_SIZE) < 0)
		return -EIO;

	if (mms_i2c_read(ts->client, MMS_CONFIRM_STATUS, tmp, 1) < 0)
		return -EIO;

	if (tmp[0] != MMS_STATUS_WRITING_DONE) {
		TOUCH_ERR_MSG("STATUS_WRTING_DONE check failed\n");
		return -EIO;
	}

	return 0;
}

static int mms_100a_isc_clear_validate_makers(struct mms_data *ts)
{
	int i = 0, j = 0;
	bool matched = false;

	TOUCH_TRACE_FUNC();

	TOUCH_INFO_MSG("fw need_update = [%d %d %d]\n", ts->need_update[0], ts->need_update[1], ts->need_update[2]);

	for (i = 1; i < SECTION_NUM; i++)
		if (ts->need_update[i])
			if (is_section_valid(ts->ts_section[i].end_addr)) {
				if (mms_100a_isc_clear_page(ts, ts->ts_section[i].end_addr) < 0) {
					TOUCH_ERR_MSG("clear page[%d] failed\n", ts->ts_section[i].end_addr);
					return -EIO;
				}
				
				if (touch_debug_mask_ & DEBUG_FW_UPGRADE)
					TOUCH_INFO_MSG("ts:page[%d] cleared...\n", ts->ts_section[i].end_addr);
			}

	for (i = 1; i < SECTION_NUM; i++) {
		if (ts->need_update[i]) {
			matched = false;

			for (j = 1; j < SECTION_NUM; j++)
				if (ts->fw_img[i]->end_page == ts->ts_section[j].end_addr) {
					matched = true;
					break;
				}

			if (matched == false && is_section_valid(ts->fw_img[i]->end_page)) {
				if (mms_100a_isc_clear_page(ts, ts->fw_img[i]->end_page) < 0) {
					TOUCH_ERR_MSG("clear page[%d] failed\n", ts->fw_img[i]->end_page);
					return -EIO;
				}

				if (touch_debug_mask_ & DEBUG_FW_UPGRADE)
					TOUCH_INFO_MSG("bin:page[%d] cleared...\n", ts->fw_img[i]->end_page);
			}
		}
	}

	return 0;
}

static u16 mms_100a_isc_cacl_crc(char *crc, int seed, char *buf)
{
	u16 _crc = 0xffff;
	int i = 0;

	TOUCH_TRACE_FUNC();

	__calc_crc(seed, _crc);

	for (i = 0; i < 1024; i++) {
		__calc_crc(buf[i], _crc);
	}

	crc[0] = (char)((_crc >> 8) & 0xff);
	crc[1] = (char)(_crc  & 0xff);
	return _crc;
}

static int mms_100a_isc_update_section_data(struct mms_data *ts)
{
	char crc[2] = {0};
	char tmp[1] = {0};
	char *ptr_fw = NULL;
	struct mms_bin_hdr *fw_hdr = NULL;
	struct mms_fw_img *fw_img = NULL;
	int i = 0, j = 0, page = 0;

	TOUCH_TRACE_FUNC();

	fw_hdr = ts->fw_hdr;

	if (touch_debug_mask_ & DEBUG_FW_UPGRADE)
		TOUCH_INFO_MSG("fw need_update = [%d %d %d]\n", ts->need_update[0], ts->need_update[1], ts->need_update[2]);

	for (i = 0; i < SECTION_NUM; i++)
		if (ts->need_update[i]) {
			fw_img = ts->fw_img[i];
			ptr_fw = (char *) fw_hdr + fw_hdr->binary_offset + fw_img->offset;

			for (page = fw_img->start_page; page <= fw_img->end_page; page++) {
				ts->buf[0] = MMS_WRITE_CMD;
				ts->buf[1] = MMS_DATA_WRITE;
				ts->buf[2] = (char) page;

				for (j = 0; j < PAGE_DATA; j += 4) {
					ts->buf[PAGE_HEADER + j + 0] = ptr_fw[j + 3];
					ts->buf[PAGE_HEADER + j + 1] = ptr_fw[j + 2];
					ts->buf[PAGE_HEADER + j + 2] = ptr_fw[j + 1];
					ts->buf[PAGE_HEADER + j + 3] = ptr_fw[j + 0];
				}
				mms_100a_isc_cacl_crc(crc, page, &ts->buf[PAGE_HEADER]);

				ts->buf[PAGE_HEADER + PAGE_DATA + 0] = crc[0];
				ts->buf[PAGE_HEADER + PAGE_DATA + 1] = crc[1];

				if (mms_i2c_write_block(ts->client, ts->buf, PACKET_SIZE) < 0) {
					TOUCH_ERR_MSG("page[%d] write failed\n", page);
					return -EIO;
				}

				if (mms_i2c_read(ts->client, MMS_CONFIRM_STATUS, tmp, 1) < 0) {
					TOUCH_ERR_MSG("page[%d] status read failed\n", page);
					return -EIO;
				}

				if (tmp[0] != MMS_STATUS_WRITING_DONE)
					TOUCH_ERR_MSG("page[%d] check failed\n", page);

				if (touch_debug_mask_ & DEBUG_FW_UPGRADE)
					TOUCH_INFO_MSG("page[%d] is written.. crc=0x%02x0x%02x\n", page, crc[0], crc[1]);
				ptr_fw += PAGE_DATA;
			}

			ts->need_update[i] = false;
		}

	return 0;
}

int mms_100a_isc(struct mms_data *ts)
{
	TOUCH_TRACE_FUNC();

	if (mms_100a_isc_enter(ts) < 0)
		goto err_isc;

	if (mms_100a_isc_write_enable(ts) < 0)
		goto err_isc;

	if (mms_100a_isc_clear_validate_makers(ts) < 0)
		goto err_isc;

	if (mms_100a_isc_update_section_data(ts) < 0)
		goto err_isc;

	return 0;

err_isc:
	return -EIO;
}

static void mms_reboot(struct mms_info *info, bool bootloader)
{
	mms_power_ctrl(info->client, POWER_OFF);
	gpio_direction_output(GPIO_TOUCH_SDA, bootloader ? 0 : 1);
	gpio_direction_output(GPIO_TOUCH_SCL, bootloader ? 0 : 1);
	gpio_direction_output(GPIO_TOUCH_INT, 0);
	msleep(30);
	mms_power_ctrl(info->client, POWER_ON);

	if (bootloader) {
		gpio_set_value(GPIO_TOUCH_SCL, 0);
		gpio_set_value(GPIO_TOUCH_SDA, 1);
	} else {
		gpio_set_value(GPIO_TOUCH_INT, 1);
		gpio_direction_input(GPIO_TOUCH_INT);
		gpio_direction_input(GPIO_TOUCH_SCL);
		gpio_direction_input(GPIO_TOUCH_SDA);
	}
	msleep(40);
}

static void isp_toggle_clk(struct mms_info *info, int start_lvl, int end_lvl, int hold_us)
{
	gpio_set_value(GPIO_TOUCH_SCL, start_lvl);
	udelay(hold_us);
	gpio_set_value(GPIO_TOUCH_SCL, end_lvl);
	udelay(hold_us);
}

/* 1 <= cnt <= 32 bits to write */
static void isp_send_bits(struct mms_info *info, u32 data, int cnt)
{
	gpio_direction_output(GPIO_TOUCH_INT, 0);
	gpio_direction_output(GPIO_TOUCH_SCL, 0);
	gpio_direction_output(GPIO_TOUCH_SDA, 0);

	/* clock out the bits, msb first */
	while (cnt--) {
		gpio_set_value(GPIO_TOUCH_SDA, (data >> cnt) & 1);
		udelay(3);
		isp_toggle_clk(info, 1, 0, 3);
	}
}

/* 1 <= cnt <= 32 bits to read */
static u32 isp_recv_bits(struct mms_info *info, int cnt)
{
	u32 data = 0;

	gpio_direction_output(GPIO_TOUCH_INT, 0);
	gpio_direction_output(GPIO_TOUCH_SCL, 0);
	gpio_set_value(GPIO_TOUCH_SDA, 0);
	gpio_direction_input(GPIO_TOUCH_SDA);

	/* clock in the bits, msb first */
	while (cnt--) {
		isp_toggle_clk(info, 0, 1, 1);
		data = (data << 1) | (!!gpio_get_value(GPIO_TOUCH_SDA));
	}

	gpio_direction_output(GPIO_TOUCH_SDA, 0);
	return data;
}

static void isp_enter_mode(struct mms_info *info, u32 mode)
{
	int cnt = 0;
	unsigned long flags = 0;

	local_irq_save(flags);
	gpio_direction_output(GPIO_TOUCH_INT, 0);
	gpio_direction_output(GPIO_TOUCH_SCL, 0);
	gpio_direction_output(GPIO_TOUCH_SDA, 1);

	mode &= 0xffff;
	for (cnt = 15; cnt >= 0; cnt--) {
		gpio_set_value(GPIO_TOUCH_INT, (mode >> cnt) & 1);
		udelay(3);
		isp_toggle_clk(info, 1, 0, 3);
	}

	gpio_set_value(GPIO_TOUCH_INT, 0);
	local_irq_restore(flags);
}

static void isp_exit_mode(struct mms_info *info)
{
	int i = 0;
	unsigned long flags = 0;

	local_irq_save(flags);
	gpio_direction_output(GPIO_TOUCH_INT, 0);
	udelay(3);

	for (i = 0; i < 10; i++)
		isp_toggle_clk(info, 1, 0, 3);
	local_irq_restore(flags);
}

static void flash_set_address(struct mms_info *info, u16 addr)
{
	/* Only 13 bits of addr are valid.
	 * The addr is in bits 13:1 of cmd */
	isp_send_bits(info, (u32)(addr & 0x1fff) << 1, 18);
}

static void flash_erase(struct mms_info *info)
{
	isp_enter_mode(info, ISP_MODE_FLASH_ERASE);

	gpio_direction_output(GPIO_TOUCH_INT, 0);
	gpio_direction_output(GPIO_TOUCH_SCL, 0);
	gpio_direction_output(GPIO_TOUCH_SDA, 1);

	/* 4 clock cycles with different timings for the erase to
	 * get processed, clk is already 0 from above */
	udelay(7);
	isp_toggle_clk(info, 1, 0, 3);
	udelay(7);
	isp_toggle_clk(info, 1, 0, 3);
	usleep_range(25000, 35000);
	isp_toggle_clk(info, 1, 0, 3);
	usleep_range(150, 200);
	isp_toggle_clk(info, 1, 0, 3);

	gpio_set_value(GPIO_TOUCH_SDA, 0);

	isp_exit_mode(info);
}

static u32 flash_readl(struct mms_info *info, u16 addr)
{
	int i = 0;
	u32 val = 0;
	unsigned long flags = 0;

	local_irq_save(flags);
	isp_enter_mode(info, ISP_MODE_FLASH_READ);
	flash_set_address(info, addr);

	gpio_direction_output(GPIO_TOUCH_SCL, 0);
	gpio_direction_output(GPIO_TOUCH_SDA, 0);
	udelay(40);

	/* data load cycle */
	for (i = 0; i < 6; i++)
		isp_toggle_clk(info, 1, 0, 10);

	val = isp_recv_bits(info, 32);
	isp_exit_mode(info);
	local_irq_restore(flags);

	return val;
}

static void flash_writel(struct mms_info *info, u16 addr, u32 val)
{
	unsigned long flags = 0;

	local_irq_save(flags);
	isp_enter_mode(info, ISP_MODE_FLASH_WRITE);
	flash_set_address(info, addr);
	isp_send_bits(info, val, 32);

	gpio_direction_output(GPIO_TOUCH_SDA, 1);
	/* 6 clock cycles with different timings for the data to get written
	 * into flash */
	isp_toggle_clk(info, 0, 1, 3);
	isp_toggle_clk(info, 0, 1, 3);
	isp_toggle_clk(info, 0, 1, 6);
	isp_toggle_clk(info, 0, 1, 12);
	isp_toggle_clk(info, 0, 1, 3);
	isp_toggle_clk(info, 0, 1, 3);

	isp_toggle_clk(info, 1, 0, 1);

	gpio_direction_output(GPIO_TOUCH_SDA, 0);
	isp_exit_mode(info);
	local_irq_restore(flags);
	usleep_range(300, 400);
}

static int fw_write_image(struct mms_info *info, const u8 *data, size_t len)
{
	u16 addr = 0;
	u32 val = 0;

	for (addr = 0; addr < (len / 4); addr++, data += 4) {
		val = get_unaligned_le32(data);

		flash_writel(info, addr, val);
	}

	return 0;
}

static int fw_download(struct mms_info *info, const u8 *data, size_t len)
{
	struct i2c_client *client = info->client;
	u32 val = 0;
	int ret = 0;
	u32 *buf = kzalloc(ISP_CAL_DATA_SIZE * 4, GFP_KERNEL);
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

	if (len % 4) {
		TOUCH_INFO_MSG("fw image size (%d) must be a multiple of 4 bytes\n", len);
		ret = -EINVAL;
		goto out;
	} else if (len > ISP_MAX_FW_SIZE) {
		TOUCH_INFO_MSG("fw image is too big, %d > %d\n", len, ISP_MAX_FW_SIZE);
		ret = -EINVAL;
		goto out;
	}

	TOUCH_INFO_MSG("fw download start\n");

	i2c_lock_adapter(adapter);
	mms_set_gpio_mode(info->pdata, GPIOMODE_ISP_START);

#if 0
	//gpio_direction_output(info->pdata->gpio_vdd_en, 0);
	melfas_poweron(0);
	gpio_direction_output(GPIO_TOUCH_SDA, 0);
	gpio_direction_output(GPIO_TOUCH_SCL, 0);
	gpio_direction_output(GPIO_TOUCH_INT, 0);
#endif

	mms_reboot(info, true);

#if 0
	TOUCH_INFO_MSG("calibration data backup\n");
	for (i = 0; i < ISP_CAL_DATA_SIZE; i++) {
		buf[i] = flash_readl(info, ISP_CAL_INFO_ADDR);
		//dev_info(&client->dev, "cal data : 0x%02x\n", buf[i]);
	}
#endif

	val = flash_readl(info, ISP_IC_INFO_ADDR);
	TOUCH_INFO_MSG("IC info: 0x%02x (%x)\n", val & 0xff, val);

	TOUCH_INFO_MSG("fw erase...\n");
	flash_erase(info);

	TOUCH_INFO_MSG("fw write...\n");

	/* XXX: what does this do?! */
	flash_writel(info, ISP_IC_INFO_ADDR, 0xffffff00 | (val & 0xff));
	usleep_range(1000, 1500);

	ret = fw_write_image(info, data, len);
	if (ret)
		goto out;

	TOUCH_INFO_MSG("fw download done...\n");

#if 0
	TOUCH_INFO_MSG("restoring data\n");
	for (i = 0; i < ISP_CAL_DATA_SIZE; i++) {
		flash_writel(info, addr, buf[i]);
	}

	mms_reboot(info, false);
#endif
	ret = 0;

out:
	if (ret != 0)
		TOUCH_INFO_MSG("fw download failed...\n");

	mms_reboot(info, false);
	kfree(buf);

	mms_set_gpio_mode(info->pdata, GPIOMODE_ISP_END);
	i2c_unlock_adapter(adapter);

	return ret;
}

int mms_100a_isp(struct mms_data *ts, const struct firmware *fw)
{
	int ret = 0;
	struct mms_info *info = kzalloc(sizeof(struct mms_info), GFP_KERNEL);
	const u8 *fw_data = fw->data + sizeof(struct mms_bin_hdr) + sizeof(struct mms_fw_img)*3;
	size_t fw_size = fw->size - sizeof(struct mms_bin_hdr) - sizeof(struct mms_fw_img)*3; 

	info->client = ts->client;
	info->pdata = ts->pdata;

	TOUCH_INFO_MSG("[%s]\n", __func__);
	TOUCH_INFO_MSG("ISP Update firmware (fw_size = %d) \n", fw_size);
	if(fw_size == 31744)
		ret = fw_download(info, fw_data, fw_size);
	else
		TOUCH_INFO_MSG("ISP Update firmware size error  \n");

	kfree(info);

	return ret;
}

int mms_100a_fw_upgrade(struct mms_data *ts, struct touch_fw_info *info)
{
	int upgrade_fuc = 0;
	int isc_up_result = 0;
	int retires = 3;

	TOUCH_TRACE_FUNC();
	TOUCH_INFO_MSG("[%s]\n", __func__);

	mms_100a_firmware_image_parse(ts, info->fw->data);

	do{
		isc_up_result = mms_100a_isc_enter(ts);
	}while(isc_up_result < 0 && --retires);

	if (!retires){
		TOUCH_INFO_MSG("failed to isc enter after retires, F/W upgrade by ISP\n");
		return mms_100a_isp(ts, info->fw);
	}

	if (unlikely(info->force_upgrade)) {
		ts->need_update[0] = true;
		ts->need_update[1] = true;
		ts->need_update[2] = true;

		return mms_100a_isp(ts, info->fw);
	}

	if(strcmp(ts->module.product_code, ts->pdata->fw_product) != 0 || strlen(ts->module.product_code) == 0){
		TOUCH_INFO_MSG("Model name is not match [%s]\n", ts->module.product_code);
		ts->need_update[0] = true;
		ts->need_update[1] = true;
		ts->need_update[2] = true;

		return mms_100a_isp(ts, info->fw);
	}

	upgrade_fuc = mms_100a_fw_version_check(ts);

	if (upgrade_fuc == FW_UP_TO_DATE)
		return 0;
	else if(upgrade_fuc == FW_UPDATE_BY_ISC){
		isc_up_result = 0;
		retires = 3;
		do{
			isc_up_result = mms_100a_isc(ts);
			mms_power_ctrl(ts->client, POWER_OFF);
			mms_power_ctrl(ts->client, POWER_ON);
		}while(isc_up_result < 0 && --retires);

		if (!retires){
			TOUCH_INFO_MSG("failed to flash firmware after retires, GO TO ISP\n");
			return mms_100a_isp(ts, info->fw);
		}
	}
	else if(upgrade_fuc == FW_UPDATE_BY_ISP)
		return mms_100a_isp(ts, info->fw);
	else
		goto err;

	return 0;

err:
	return -EIO;
}

