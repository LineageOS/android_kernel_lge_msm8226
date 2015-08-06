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

enum {
	ISC_ADDR		= 0xD5,

	ISC_CMD_READ_STATUS	= 0xD9,
	ISC_CMD_READ		= 0x4000,
	ISC_CMD_EXIT		= 0x8200,
	ISC_CMD_PAGE_ERASE	= 0xC000,

	ISC_PAGE_ERASE_DONE	= 0x10000,
	ISC_PAGE_ERASE_ENTER	= 0x20000,
};

struct isc_packet {
	u8 	cmd;
	u32	addr;
	u8	data[0];
} __attribute__ ((packed));

#define ISC_XFER_LEN		256
#define MMS_FLASH_PAGE_SZ	1024
#define ISC_BLOCK_NUM		(MMS_FLASH_PAGE_SZ / ISC_XFER_LEN)

static int mms_100s_firmware_image_parse(struct mms_data *ts, const u8 *image_bin)
{
	int offset = 0;
	int i = 0;
	struct mms_fw_img *img = NULL;

	ts->fw_hdr = (struct mms_bin_hdr *) image_bin;
	ts->fw_img[0] = NULL;
	ts->fw_img[1] = NULL;

	if (touch_debug_mask_ & DEBUG_FW_UPGRADE) {
		TOUCH_INFO_MSG("mms_fw_hdr:\n");
		TOUCH_INFO_MSG("\ttag [%c%c%c%c%c%c%c%c]\n",
				ts->fw_hdr->tag[0], ts->fw_hdr->tag[1], ts->fw_hdr->tag[2], ts->fw_hdr->tag[3],
				ts->fw_hdr->tag[4], ts->fw_hdr->tag[5], ts->fw_hdr->tag[6], ts->fw_hdr->tag[7]);
		TOUCH_INFO_MSG("\tcore_version [0x%02x]\n", ts->fw_hdr->core_version);
		TOUCH_INFO_MSG("\tsection_num [%d]\n", ts->fw_hdr->section_num);
		TOUCH_INFO_MSG("\tcontains_full_binary [%d]\n", ts->fw_hdr->contains_full_binary);
		TOUCH_INFO_MSG("\tbinary_offset [%d (0x%04x)]\n", ts->fw_hdr->binary_offset, ts->fw_hdr->binary_offset);
		TOUCH_INFO_MSG("\tbinary_length [%d]\n", ts->fw_hdr->binary_length);
	}

	if (ts->fw_hdr->section_num != 2)
		return -EINVAL;

	offset = sizeof(struct mms_bin_hdr);
	for (i = 0; i < ts->fw_hdr->section_num; i++) {
		img = (struct mms_fw_img *) (image_bin + offset);

		if (touch_debug_mask_ & DEBUG_FW_UPGRADE) {
			TOUCH_INFO_MSG("\ttype[%d:%s],version[0x%02X],page[%02d-%02d],offset[%d],length[%d]\n",
				img->type, section_name[img->type], img->version, img->start_page, img->end_page, img->offset, img->length);
		}

		ts->fw_img[i] = img;
		offset += sizeof(struct mms_fw_img);
	}

	return 0;
}

static int mms_100s_isc_enter(struct mms_data *ts)
{
	char tmp[2] = { MMS_ENTER_ISC, 0x01 };

	TOUCH_TRACE_FUNC();

	if (mms_i2c_write_block(ts->client, tmp, 2) < 0)
		return -EIO;

	TOUCH_FW_MSG("isc entered...\n");

	return 0;
}

static int mms_100s_fw_version_check(struct mms_data *ts)
{
	int i = 0;
	int type = 0;

	struct mms_fw_img *img = NULL;

	if (strcmp(ts->pdata->fw_product, ts->module.product_code) != 0) {
		TOUCH_FW_MSG("product is not matched [%s] \n", ts->module.product_code);
		ts->need_update[0] = true;
		ts->need_update[1] = true;
	} else {
		for (i = 0; i < ts->fw_hdr->section_num; i++) {
			img = ts->fw_img[i];
			type = img->type;
			if (img->version != ts->ts_section[type].version) {
				ts->need_update[i] = true;
			}
		}
	}

	TOUCH_FW_MSG("fw need_update = [%d %d]\n", ts->need_update[0], ts->need_update[1]);

	return 0;
}

static int mms_100s_isc_exit(struct mms_data *ts)
{
	struct isc_packet pkt = { ISC_ADDR, ISC_CMD_EXIT, };

	TOUCH_TRACE_FUNC();

	if (mms_i2c_write_block(ts->client, (char *) &pkt, sizeof(struct isc_packet)) < 0) {
		TOUCH_FW_MSG("failed\n");
		return -EIO;
	}

	TOUCH_FW_MSG("fw need_update = [%d %d]\n", ts->need_update[0], ts->need_update[1]);
	msleep(5);
	return 0;
}

static int mms_100s_isc_erase_page(struct mms_data *ts, u8 page)
{
	u32 status = 0;
	int retry = 0;
	int max_ms = 100;
	int sleep_ms = 10;
	struct isc_packet pkt = { ISC_ADDR, ISC_CMD_PAGE_ERASE | page, };

	TOUCH_TRACE_FUNC();

	if (mms_i2c_write_block(ts->client, (char *) &pkt, sizeof(struct isc_packet)) < 0)
		return -EIO;

	do {
		msleep(sleep_ms);

		mms_i2c_read(ts->client, ISC_CMD_READ_STATUS, (char *) &status, sizeof(status));
		if (status == (ISC_PAGE_ERASE_DONE | ISC_PAGE_ERASE_ENTER | page)) {
			TOUCH_FW_MSG("page[%d] erased, retry %d\n", page, retry);
			goto isc_erase_success;
		}
		retry++;
	} while (retry*sleep_ms < max_ms);

	TOUCH_ERR_MSG("page[%d] erase failed\n", page);
	return -1;

isc_erase_success:
	return 0;
}

static int mms_100s_flash_section(struct mms_data *ts)
{
	char *ptr_fw = NULL;
	struct mms_bin_hdr *fw_hdr = ts->fw_hdr;
	struct mms_fw_img *fw_img = NULL;
	struct isc_packet *pkt = (struct isc_packet *) ts->buf;
	int i = 0;
	int j = 0;
	int page = 0;
	int tmp = 0;

	TOUCH_FW_MSG("fw need_update = [%d %d]\n", ts->need_update[0], ts->need_update[1]);

	for (i = 0; i < ts->fw_hdr->section_num; i++) {
		if (ts->need_update[i]) {
			fw_img = ts->fw_img[i];
			ptr_fw = (char *) fw_hdr + fw_hdr->binary_offset + fw_img->offset;

			for (page = fw_img->start_page; page <= fw_img->end_page; page++) {
				if (mms_100s_isc_erase_page(ts, page) < 0)
					return -EIO;

				for (j = 0; j < ISC_BLOCK_NUM; j++, ptr_fw += ISC_XFER_LEN) {
					tmp = (page << 8) + j * (ISC_XFER_LEN >> 2);
					put_unaligned_le32(tmp, &pkt->addr);
					memcpy(pkt->data, ptr_fw, ISC_XFER_LEN);

					if (mms_i2c_write_block(ts->client, ts->buf, sizeof(struct isc_packet) + ISC_XFER_LEN) < 0) {
						TOUCH_FW_MSG("page[%d], offset[%d] write failed\n", page, j * (ISC_XFER_LEN));
						return -EIO;
					}
				}

				TOUCH_FW_MSG("page[%d], written\n", page);
			}

			ts->need_update[i] = false;
		}
	}

	return 0;
}

int mms_100s_isc(struct mms_data *ts, struct touch_fw_info *info)
{
	TOUCH_TRACE_FUNC();

	mms_100s_firmware_image_parse(ts, info->fw->data);

	if (unlikely(!info->force_upgrade)) {
		if (mms_100s_fw_version_check(ts) < 0)
			goto err_isc;
	} else {
		ts->need_update[0] = true;
		ts->need_update[1] = true;
	}

	if (mms_100s_isc_enter(ts) < 0)
		goto err_isc;

	if (mms_100s_flash_section(ts) < 0)
		goto err_isc;

	if (mms_100s_isc_exit(ts) < 0)
		goto err_isc;

	return 0;

err_isc:
	return -EIO;
}

