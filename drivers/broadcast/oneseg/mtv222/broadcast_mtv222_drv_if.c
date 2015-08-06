/*
 * broadcast_mtv222_drv_if.c
 *
 * Driver for ISDB-T.
 *
 * Copyright (C) (2013, LGE)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>

#include "broadcast_dmb_typedef.h"
#include "broadcast_dmb_drv_ifdef.h"

#include "broadcast_mtv222.h"

#ifdef _MTV_KERNEL_FILE_DUMP_ENABLE
	struct file *mtv222_ts_filp = NULL;
#endif

static inline int get_sig_antenna_level(U32 cnr)
{
	UINT antenna_level;

	antenna_level = rtvISDBT_GetAntennaLevel(cnr);
	switch (antenna_level) {
	case 0:
		return 0;

	case 1:
	case 2:
		return 1;

	case 3:
	case 4:
		return 2;
		break;

	default: /* 5 ~ 6 */
		return 3;
	}
}

static inline int get_sig_sys_info(void)
{
	RTV_ISDBT_TMCC_INFO bb_tmcc;
	int sysinfo = 0;

	rtvISDBT_GetTMCC(&bb_tmcc);

	sysinfo |= (bb_tmcc.eTvMode << 6);
	sysinfo |= (bb_tmcc.eGuard << 4);

	return sysinfo;
}

static inline int get_sig_tmcc_info(void)
{
	RTV_ISDBT_TMCC_INFO bb_tmcc;
	int tmccinfo = 0;

	rtvISDBT_GetTMCC(&bb_tmcc);

	tmccinfo = (0 << 6); /* TV only for 1seg */

	tmccinfo = (0xF << 2);

	if (!bb_tmcc.fEWS)
		tmccinfo |= (0 << 1);
	else
		tmccinfo |= (1 << 1);

	if (bb_tmcc.eSeg == RTV_ISDBT_SEG_1)
		tmccinfo |= (1 << 0);
	else
		tmccinfo |= (0 << 0);

	return tmccinfo;
}

static inline int get_sig_layer_info(void)
{
	RTV_ISDBT_TMCC_INFO bb_tmcc;
	int layerinfo = 0;

	rtvISDBT_GetTMCC(&bb_tmcc);

	layerinfo |= (bb_tmcc.eModulation << 13);
	layerinfo |= (bb_tmcc.eCodeRate << 10);

	switch (bb_tmcc.eInterlv) {
	case RTV_ISDBT_INTERLV_0:
		layerinfo |= (0 << 4);
		break;
	case RTV_ISDBT_INTERLV_1:
		layerinfo |= (1 << 4);
		break;
	case RTV_ISDBT_INTERLV_2:
		layerinfo |= (2 << 4);
		break;
	case RTV_ISDBT_INTERLV_4:
		layerinfo |= (4 << 4);
		break;
	case RTV_ISDBT_INTERLV_8:
		layerinfo |= (8 << 4);
		break;
	case RTV_ISDBT_INTERLV_16:
		layerinfo |= (16 << 4);
		break;
	case RTV_ISDBT_INTERLV_32:
		layerinfo |= (32 << 4);
		break;
	default:
		layerinfo |= (63 << 4);
		break;
	}

	if (bb_tmcc.eSeg == RTV_ISDBT_SEG_1)
		layerinfo |= (1 << 0);
	else
		layerinfo |= (3 << 0);

	return layerinfo;
}

static inline int get_sig_receive_status(void)
{
	RTV_ISDBT_TMCC_INFO bb_tmcc;
	unsigned int lock_mask;
	int receive_status = 0;

	rtvISDBT_GetTMCC(&bb_tmcc);
	if (!bb_tmcc.fEWS)
		receive_status |= (0 << 7);
	else
		receive_status |= (1 << 7);

	lock_mask = rtvISDBT_GetLockStatus();
	if (lock_mask == RTV_ISDBT_CHANNEL_LOCK_OK)
		receive_status |= (0 << 0);
	else if (lock_mask & RTV_ISDBT_TMCC_LOCK_MASK)
		receive_status |= (0 << 0);
	else if (lock_mask & RTV_ISDBT_OFDM_LOCK_MASK)
		receive_status |= (1 << 0);
	else
		receive_status |= (3 << 0);

	return receive_status;
}

static inline int get_sig_scan_status(void)
{
	unsigned int lock_mask;
	int scan_status = 0;

	lock_mask = rtvISDBT_GetLockStatus();
	if (lock_mask == RTV_ISDBT_CHANNEL_LOCK_OK)
		scan_status = 2;
	else if (lock_mask & RTV_ISDBT_OFDM_LOCK_MASK)
		scan_status = 1;
	else
		scan_status = 0;

	return scan_status;
}

static inline unsigned int get_sync_status(void)
{
	unsigned int lock_mask, sync_status;

	lock_mask = rtvISDBT_GetLockStatus();
	if (lock_mask == RTV_ISDBT_CHANNEL_LOCK_OK)
		sync_status = 3;
	else if (lock_mask & RTV_ISDBT_TMCC_LOCK_MASK)
		sync_status = 3;
	else if (lock_mask & RTV_ISDBT_OFDM_LOCK_MASK)
		sync_status = 2;
	else
		sync_status = 0;

	return sync_status;
}

static inline int get_sig_rssi(void)
{
	int rssi;
	S32 bb_rssi;

	bb_rssi = rtvISDBT_GetRSSI();

	rssi = bb_rssi / RTV_ISDBT_RSSI_DIVIDER;

	return rssi;
}

static inline int get_sig_cnr(U32 *bb_cnr)
{
	int cnr;

	*bb_cnr = rtvISDBT_GetCNR();

	cnr = *bb_cnr / RTV_ISDBT_CNR_DIVIDER;

	return cnr;
}

static inline int get_sig_per(void)
{
	int per;
	U32 bb_per;

	bb_per = rtvISDBT_GetPER();

	per = (bb_per * 100000) / 700;

	return per;
}

static inline unsigned int get_lock_status(void)
{
	unsigned int lock_mask;

	lock_mask = rtvISDBT_GetLockStatus();

	return ((lock_mask == RTV_ISDBT_CHANNEL_LOCK_OK) ? 1 : 0);
}

static void get_onseg_sig_info(oneseg_sig_info *oneseg_info)
{
	U32 bb_cnr;

	oneseg_info->lock = get_lock_status();

	oneseg_info->cn = get_sig_cnr(&bb_cnr);
	if (oneseg_info->cn >= 28)
		oneseg_info->cn = 28;

	oneseg_info->ber = rtvISDBT_GetBER();
	oneseg_info->per = get_sig_per();
	oneseg_info->agc = rtvISDBT_GetAGC();
	oneseg_info->rssi = get_sig_rssi();
	oneseg_info->ErrTSP = mtv222_cb->err_tsp_cnt;
	oneseg_info->TotalTSP = mtv222_cb->total_tsp_cnt;
	oneseg_info->antenna_level = get_sig_antenna_level(bb_cnr);

	oneseg_info->Num = 0;
	oneseg_info->Exp = 0;
	oneseg_info->mode = 0;

	MTVMSG("lock(%d), bb_cnr(%u), cn(%d), ber(%d), per(%d), rssi(%d), ErrTSP(%d), TotalTSP(%d), antenna_level(%d)\n",
			oneseg_info->lock, bb_cnr, oneseg_info->cn, oneseg_info->ber, oneseg_info->per,
			oneseg_info->rssi, oneseg_info->ErrTSP, oneseg_info->TotalTSP, oneseg_info->antenna_level);

	SHOW_DEBUG_TSPB_STAT;
}

int	broadcast_drv_if_power_on(void)
{
	int rc = OK;

	mutex_lock(&mtv222_cb->ioctl_lock);

	if (mtv222_device_power_state() == false) {
		if (mtv222_device_power_on() == 0){
			wake_lock(&mtv222_cb->wake_lock);
			MTVMSG("Success\n");
		}else {
			rc = ERROR;
			MTVERR("Power On error(%d)\n", rc);
		}
	}
	else{
		MTVMSG("Already power on\n");
	}

	mutex_unlock(&mtv222_cb->ioctl_lock);

	return rc;
}

int	broadcast_drv_if_power_off(void)
{
	mutex_lock(&mtv222_cb->ioctl_lock);

	if (mtv222_device_power_state() == true) {
		mtv222_cb->tsout_enabled = false;

		mtv222_device_power_off();

		wake_unlock(&mtv222_cb->wake_lock);
	}

	mutex_unlock(&mtv222_cb->ioctl_lock);

	return OK;
}

int	broadcast_drv_if_open(void)
{
	int rc = OK;

	mutex_lock(&mtv222_cb->ioctl_lock);

	if (mtv222_device_power_state() == false) {
		MTVERR("Device was power off\n");
		rc = ERROR;
		goto release_mutex;
	}

	/* Check if the driver was already opened ? */
	if (atomic_cmpxchg(&mtv222_cb->open_flag, 0, 1)) {
		MTVERR("Already opened\n");
		rc = ERROR;
		goto release_mutex;
	}

	RTV_GUARD_INIT;
	mtv222_cb->tsout_enabled = false;

	rc = rtvISDBT_Initialize(RTV_COUNTRY_BAND_BRAZIL,
							ISDBT_1SEG_SPI_INTR_SIZE);
	if (rc != RTV_SUCCESS) {
		rc = ERROR;
		goto release_mutex;
	}

	mtv222_enable_irq(); /* After DMB init */

	mutex_unlock(&mtv222_cb->ioctl_lock);

	return OK;

release_mutex:
	atomic_set(&mtv222_cb->open_flag, 0);

	mutex_unlock(&mtv222_cb->ioctl_lock);

	return rc;
}

int	broadcast_drv_if_close(void)
{
	mutex_lock(&mtv222_cb->ioctl_lock);

	mtv222_cb->tsout_enabled = false; /* to fast stop read-thread */

	mtv222_disable_irq();

	RTV_GUARD_DEINIT;

	atomic_set(&mtv222_cb->open_flag, 0); /* Reset */

	mutex_unlock(&mtv222_cb->ioctl_lock);

	return OK;
}

int	broadcast_drv_if_set_channel(struct broadcast_dmb_set_ch_info *udata)
{
	int demod_ret;
	int rc = OK;

	mutex_lock(&mtv222_cb->ioctl_lock);

	if (mtv222_device_power_state() == false) {
		MTVERR("Device was power off\n");
		rc = ERROR;
		goto release_mutex;
	}

	if (mtv222_cb->tsout_enabled == true) {
		MTVERR("WARNNING! Stream was not stoppped!\n");

		mtv222_cb->tsout_enabled = false;
		mtv222_tsb_disable();

		rtvISDBT_DisableStreamOut();

#ifndef FEATURE_DMB_USE_WORKQUEUE
		atomic_set(&mtv222_cb->isr_cnt, 0); /* Reset */
#endif

#ifdef _MTV_KERNEL_FILE_DUMP_ENABLE
		mtv_ts_dump_kfile_close();
#endif
	}

	if (udata->mode == 6)//1) /* Scan mode */
		demod_ret = rtvISDBT_ScanFrequency(udata->channel);
	else {
		demod_ret = rtvISDBT_ScanFrequency(udata->channel);
		if (demod_ret == RTV_CHANNEL_NOT_DETECTED)
			demod_ret = RTV_SUCCESS;
	}

	if (demod_ret == RTV_SUCCESS) {
		mtv222_init_tsp_statistics();

		RESET_DEBUG_TSPB_STAT;

		mtv222_tsb_enable();

		mtv222_cb->tsout_enabled = true;

#ifdef _MTV_KERNEL_FILE_DUMP_ENABLE
		if (udata->mode != 6) { /* Play mode */
			if (mtv_ts_dump_kfile_open(udata->channel) != 0) {
				rc = ERROR;
				goto release_mutex;
			}
		}
#endif

		rtvISDBT_EnableStreamOut();
	}
	else {
		MTVMSG("Result(%d)\n", demod_ret);
		rc = ERROR;
		goto release_mutex;
	}

release_mutex:
	mutex_unlock(&mtv222_cb->ioctl_lock);

	return rc;
}

int	broadcast_drv_if_resync(void)
{
	printk(KERN_DEBUG"[mtv222]broadcast_drv_if_resync\n");

	return OK;
}

int	broadcast_drv_if_detect_sync(struct broadcast_dmb_sync_info *udata)
{
	mutex_lock(&mtv222_cb->ioctl_lock);

	udata->sync_status = get_sync_status();

	mutex_unlock(&mtv222_cb->ioctl_lock);

	return OK;
}

int	broadcast_drv_if_get_sig_info(struct broadcast_dmb_sig_info *pInfo)
{
	//U32 bb_cnr;
	oneseg_sig_info *oneseg_info;
	//mmb_sig_info *mmb_info;
	int rc = OK;

	if (pInfo == NULL) {
		MTVERR("Null pointer info\n");
		return ERROR;
	}

	oneseg_info = &pInfo->info.oneseg_info;
	//mmb_info = &pInfo->sig_info.info.mmb_info;

	mutex_lock(&mtv222_cb->ioctl_lock);

	if (mtv222_device_power_state() == false) {
		MTVERR("Device was power off\n");
		rc = ERROR;
		goto release_mutex;
	}

#if 1
	get_onseg_sig_info(oneseg_info);

#else
	switch(pInfo->cmd_info.cmd) {
	case 1: /* all */
#if 0
		mmb_info->layerinfo = get_sig_layer_info();
		mmb_info->receive_status = get_sig_receive_status();
		mmb_info->scan_status = get_sig_scan_status();
		mmb_info->sysinfo = get_sig_sys_info();
#endif
		get_onseg_sig_info(oneseg_info);
		break;

	case 2: /* BER */
		oneseg_info->ber = rtvISDBT_GetBER();
		break;

	case 3: /* PER */
		oneseg_info->per = get_sig_per();
		break;

	case 4: /* CN */
		oneseg_info->cn = get_sig_cnr(&bb_cnr);
		break;

	case 5: /* CN per Layer */
		break;

	case 6: /* Layer Info */
		//mmb_info->layerinfo = get_sig_layer_info();
		break;

	case 7: /* Receive status */
		//mmb_info->receive_status = get_sig_receive_status();
		break;

	case 8: /* rssi */
		oneseg_info->rssi = get_sig_rssi();
		break;

	case 9: /* scan status */
		//mmb_info->scan_status = get_sig_scan_status();
		break;

	case 10: /* sys info */
		//mmb_info->sysinfo = get_sig_sys_info();
		break;

	case 11: /* tmcc info */
		//mmb_info->tmccinfo = get_sig_tmcc_info();
		break;

	case 12: /* oneseg_sig_info */
		get_onseg_sig_info(oneseg_info);
		break;

	default:
		MTVERR("Invalid the command of getting signal. cmd(%u)\n", pInfo->cmd_info.cmd);
		rc = ERROR;
		break;
	}
#endif

release_mutex:
	mutex_unlock(&mtv222_cb->ioctl_lock);

	return rc;
}

int	broadcast_drv_if_get_ch_info(struct broadcast_dmb_ch_info *ch_info)
{
	return ERROR;
}

/* ioctl */
int	broadcast_drv_if_get_dmb_data(struct broadcast_dmb_data_info *pdmb_data)
{
	int copy_size;
	int rc = OK;

	if (pdmb_data == NULL) {
		MTVERR("Null pointer pdmb_data\n");
		return ERROR;
	}

	if (pdmb_data->data_buf == 0) {
		MTVERR("Null pointer buffer\n");
		return ERROR;
	}

	if (pdmb_data->data_buf_size < 188) {
		MTVERR("Invalid size(%u)\n", pdmb_data->data_buf_size);
		return ERROR;
	}

	/* Check if the device is already readed ? */
	if (atomic_cmpxchg(&mtv222_cb->read_flag, 0, 1)) {
		MTVERR("Busy...\n");
		return ERROR;
	}

	if (mtv222_cb->tsout_enabled == true) {
		copy_size = mtv222_tsb_read(pdmb_data->data_buf, pdmb_data->data_buf_size);
		if (copy_size > 0) {
			pdmb_data->copied_size = copy_size;
			pdmb_data->packet_cnt = copy_size / 188;

#if 0//def _MTV_KERNEL_FILE_DUMP_ENABLE
			mtv_ts_dump_kfile_write(pdmb_data->data_buf, copy_size);
#endif


		} else
			rc = ERROR;
	} else {
		rc = OK; /* Channel was not started. */
		pdmb_data->copied_size = 0;
		pdmb_data->packet_cnt = 0;
	}

	atomic_set(&mtv222_cb->read_flag, 0);

	return rc;
}

int	broadcast_drv_if_reset_ch(void)
{
	int rc = OK;

	mutex_lock(&mtv222_cb->ioctl_lock);

	if (mtv222_device_power_state() == false) {
		MTVERR("Device was power off\n");
		rc = ERROR;
		goto release_mutex;
	}

	mtv222_cb->tsout_enabled = false;
	mtv222_tsb_disable();

	rtvISDBT_DisableStreamOut();

#ifndef FEATURE_DMB_USE_WORKQUEUE
	atomic_set(&mtv222_cb->isr_cnt, 0); /* Reset */
#endif

#ifdef _MTV_KERNEL_FILE_DUMP_ENABLE
	mtv_ts_dump_kfile_close();
#endif

release_mutex:
	mutex_unlock(&mtv222_cb->ioctl_lock);

	return rc;
}

int	broadcast_drv_if_user_stop(int mode)
{
	printk(KERN_DEBUG"[mtv222]broadcast_drv_if_user_stop\n");

	return OK;
}

int	broadcast_drv_if_select_antenna(unsigned int sel)
{

	printk(KERN_DEBUG"[mtv222]broadcast_drv_if_select_antenna\n");

	return OK;
}

/* file read */
int	broadcast_drv_if_read_control(char *buf, unsigned int size)
{
	int ret;

	printk(KERN_DEBUG"[mtv222]broadcast_drv_if_read_control\n");


	if (buf == 0) {
		MTVERR("Null pointer buffer\n");
		return -EINVAL;
	}

	if (size < 188) {
		MTVERR("Invalid size(%u)\n", size);
		return -EINVAL;
	}

	/* Check if the device is already readed ? */
	if (atomic_cmpxchg(&mtv222_cb->read_flag, 0, 1)) {
		MTVERR("Busy...\n");
		return -EBUSY;
	}

	if (mtv222_cb->tsout_enabled == true) {
		ret = mtv222_tsb_read(buf, size);
		if (ret < 0)
			ret = -EFAULT;

#if 0//def _MTV_KERNEL_FILE_DUMP_ENABLE
		if (ret > 0)
			mtv_ts_dump_kfile_write(buf, ret);
#endif
	} else {
		/* Channel was not started. */
		ret = 0;//-EINVAL;
	}

	atomic_set(&mtv222_cb->read_flag, 0);

	return ret;
}

int	broadcast_drv_if_get_mode (unsigned short *mode)
{
	printk(KERN_DEBUG"[mtv222]broadcast_drv_if_get_mode\n");

	return ERROR;
}



