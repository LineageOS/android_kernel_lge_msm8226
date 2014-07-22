/*
 *  linux/include/linux/mmc/discard.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Card driver specific definitions.
 */

#ifndef _LINUX_MEM_LOG_H
#define _LINUX_MEM_LOG_H

#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/delay.h>

#include <linux/vmalloc.h>
#include <linux/string.h>

#include <linux/mmc/mmc.h>
#include <linux/mmc/core.h>
#include <linux/blkdev.h>
#include <linux/sched.h>
#include <linux/kthread.h>

#ifndef CONFIG_FMBT_MEM_LOG_BUF_SHIFT
#define CONFIG_FMBT_MEM_LOG_BUF_SHIFT	0
#endif

#define __MEM_LOG_BUF_LEN	(1 << CONFIG_FMBT_MEM_LOG_BUF_SHIFT)

#define MEM_LOG_IDLE		0x00	/* 000 */
#define MEM_LOG_MMC			0x01	/* 001 */
#define MEM_LOG_SCSI		0x02	/* 010 */
#define MEM_LOG_BLOCK		0x03	/* 011 */
#define MEM_LOG_APP			0x04	/* 100 */

#define MEM_LOG_READ		0x01	/* 001 */
#define MEM_LOG_WRITE		0x02	/* 010 */
#define MEM_LOG_OPCODE		0x03	/* 011 */
#if defined(CONFIG_FMBT_TRACE_EMMC)
#define MEM_LOG_PACKED		0x04    /* 100 */
#endif
#define MEM_LOG_APP_END		0x00		/* 1 */
#define MEM_LOG_APP_START	0x01		/* 0 */

#define mem_target_getopt(c, opt)	(((c)->flag & 0x07) == opt)
#define mem_cmd_getopt(c, opt)	(((c)->flag >> 3 & 0x07) == opt)
//#define mem_app_getopt(c, opt) (((c)->flag >> 6) & opt)
#define mem_app_getopt(c, opt) (((c)->flag >> 6 & 0x01) == opt)

/*
#define mem_target_getopt(c, opt)	(((c)->flag & opt) == opt)
#define mem_cmd_getopt(c, opt)	(((c)->flag >> 3 & opt) == opt)
//#define mem_app_getopt(c, opt) (((c)->flag >> 6) & opt)
#define mem_app_getopt(c, opt) (((c)->flag >> 6 & opt) == opt)
*/
#define mem_target_setopt(c, opt)	((c)->flag |= opt)
#define mem_cmd_setopt(c, opt)	((c)->flag |= opt << 3)
#define mem_app_setopt(c, opt) ((c)->flag |= opt << 6)

#define mem_target_clearopt(c, opt)	((c)->flag &= ~opt)
#define mem_cmd_clearopt(c, opt)	((c)->flag &= ~opt << 3)
#define mem_app_clearopt(c, opt) ((c)->flag &= ~opt << 6)

enum MEM_APP_NAME {
	MEM_APP_CAMERA		= 0,
	MEM_APP_WEB_S_ON,
	MEM_APP_WEB_S_OFF,
	MEM_APP_WEB_L_OFF,
	MEM_APP_CONTACTS,
	MEM_APP_PACKAGE,
	MEM_APP_YOUTUBE,
	MEM_APP_MANUAL,
};

#pragma pack(1)
/* total 27byte */
typedef struct _MEM_LOG_PARCER_T{
	char flag;
	char app_num;
	unsigned int sector;			/* if opcode_add then sector is opcode number */
	unsigned int sector_len;
	unsigned long long cur_time;
	unsigned long long latency_time;
	//unsigned char queue_count;
	unsigned char opcode;			/* if block_add then opcode is queue_count */
	unsigned int packed_cmd_hdr;
	unsigned char hdr;
}mem_log_parcer_t;

typedef struct _MEM_LOG_T{
	mem_log_parcer_t* log_buf;
	unsigned long max_index;
	unsigned long cur_index;
	int start;
	int enable;
	spinlock_t lock;
	struct task_struct	*kthread;
	struct semaphore	thread_sem;
} mem_log_t;

typedef struct _PACKED_CMD_T{
	unsigned int packed_cmd_hdr[128];
	unsigned char num_packed;
	unsigned int packed_blocks;
} packed_cmd_t;
#pragma pack()


int init_memLog(void);
int memlog_emmc_add(struct mmc_request *mrq, unsigned long long curTime, unsigned long long latency);
#if defined(CONFIG_FMBT_TRACE_EMMC)
int memlog_packed_add(u32 packed_cmd_hdr, u8 hdr);
#endif
int memlog_opcode_add(struct mmc_request *mrq, unsigned long long curTime, unsigned long long latency);
int memlog_app_add(int start, int name);
int memlog_set_enable(int enable);
int memlog_print(void);
int memlog_release(void);

//#define CONFIG_MMC_MEM_LOG_DEBUG

/* debug utility functions */
#ifdef CONFIG_MMC_MEM_LOG_DEBUG
#define _mdbg_msg(fmt, args...) printk(KERN_INFO "[MEMLOG] %s(%d): " fmt, __func__, __LINE__, ##args)
#else
#define _mdbg_msg(fmt, args...)
#endif /* CONFIG_MMC_MEM_LOG_BUFF_DEBUG */

#define _err_msg(fmt, args...) printk(KERN_ERR "%s(%d): " fmt, __func__, __LINE__, ##args)

#endif /*_LINUX_MEM_LOG_H */

