/**===================================================================
 * Copyright(c) 2009 LG Electronics Inc. All Rights Reserved
 *
 * File Name : broadcast_mtv319.h
 * Description : EDIT HISTORY FOR MODULE
 * This section contains comments describing changes made to the module.
 * Notice that changes are listed in reverse chronological order.
 *
 * when			model		who			what
 * 10.27.2009		android		inb612		Create for Android platform
====================================================================**/

#ifndef _BROADCAST_MTV222_H_
#define _BROADCAST_MTV222_H_

#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/fcntl.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/wait.h>
#include <linux/ioctl.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/vmalloc.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/atomic.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/jiffies.h>
#include <linux/wakelock.h>
#include <linux/kthread.h>
#include <linux/clk.h>
#include <linux/of_gpio.h>

#include "raontv.h"

/*############################################################################
# Debug TSP buffer
############################################################################*/
//#define DEBUG_TSP_BUF

#define TS_PACKET_SIZE	188

#define ISDBT_1SEG_SPI_INTR_PKT_CNT		16

/* Define the number of TSP chnuks to be buffering. */
#define ISDBT_NUM_TSP_CHNUKS	30


#define ISDBT_1SEG_SPI_INTR_SIZE\
	(ISDBT_1SEG_SPI_INTR_PKT_CNT * TS_PACKET_SIZE)


#define MTVERR(fmt, args...) \
	printk(KERN_ERR "MTV222: %s(): " fmt, __func__, ## args)

#define MTVMSG(fmt, args...) \
	printk(KERN_INFO "MTV222: %s(): " fmt, __func__, ## args)

/* SPI Data read using workqueue */
//#define FEATURE_DMB_USE_WORKQUEUE


/*############################################################################
# File dump Configuration
	* TS dump filename: /data/local/isdbt_ts_FREQ.ts
############################################################################*/
//#define _MTV_KERNEL_FILE_DUMP_ENABLE

#ifdef _MTV_KERNEL_FILE_DUMP_ENABLE
	extern struct file *mtv222_ts_filp;
#endif

static inline int mtv_ts_dump_kfile_write(char *buf, size_t len)
{
#ifdef _MTV_KERNEL_FILE_DUMP_ENABLE
	mm_segment_t oldfs;
	struct file *filp;
	int ret = 0;

	if (mtv222_ts_filp != NULL) {
		filp = mtv222_ts_filp;
		oldfs = get_fs();
		set_fs(KERNEL_DS);
		ret = filp->f_op->write(filp, buf, len, &filp->f_pos);
		set_fs(oldfs);
		if (!ret)
			MTVERR("File write error (%d)\n", ret);
	}

	return ret;
#else
	return 0;
#endif
}

static inline void mtv_ts_dump_kfile_close(void)
{
#ifdef _MTV_KERNEL_FILE_DUMP_ENABLE
	if (mtv222_ts_filp != NULL) {
		filp_close(mtv222_ts_filp, NULL);
		mtv222_ts_filp = NULL;
	}
#endif
}

static inline int mtv_ts_dump_kfile_open(unsigned int channel)
{
#ifdef _MTV_KERNEL_FILE_DUMP_ENABLE
	char fname[32];
	struct file *filp = NULL;

	if (mtv222_ts_filp == NULL) {
		sprintf(fname, "/data/local/isdbt_ts_%u.ts", channel);
		filp = filp_open(fname, O_RDWR|O_CREAT|O_TRUNC, S_IRUSR);

		if (IS_ERR(filp)) {
			filp = NULL;
			MTVERR("File open error: %s!\n", fname);
			return PTR_ERR(filp);
		}

		mtv222_ts_filp = filp;

		MTVMSG("Kernel dump file opened(%s)\n", fname);
	} else {
		MTVERR("Already TS file opened! Should closed!\n");
		return -1;
	}
#endif

	return 0;
}


/* MTV222 drvier Control Block */
struct MTV222_CB {
	struct spi_device *spi_ptr;

	struct wake_lock wake_lock;

	volatile bool power_state;
	volatile bool tsout_enabled;

	struct mutex ioctl_lock;

	atomic_t open_flag; /* to open only once */

	/* to prevent reading of ts from the multiple applicatoin threads. */
	atomic_t read_flag;

	int total_tsp_cnt;
	int err_tsp_cnt;

#ifdef FEATURE_DMB_USE_WORKQUEUE
	struct work_struct			spi_work;
	struct workqueue_struct*	spi_wq;

#else
	struct task_struct *isr_thread_cb;
	wait_queue_head_t isr_wq;
	atomic_t isr_cnt;
#endif

#ifdef FEATURE_DMB_USE_XO
	struct clk					*clk;
#endif

#ifdef DEBUG_TSP_BUF
	unsigned int max_alloc_seg_cnt;
	unsigned int max_enqueued_seg_cnt;

	unsigned int max_enqueued_tsp_cnt;
	unsigned long alloc_tspb_err_cnt;
#endif
};


extern struct MTV222_CB *mtv222_cb;

#ifdef DEBUG_TSP_BUF
static INLINE void reset_debug_tspb_stat(void)
{
	mtv222_cb->max_alloc_seg_cnt = 0;
	mtv222_cb->max_enqueued_seg_cnt = 0;
	mtv222_cb->max_enqueued_tsp_cnt = 0;
	mtv222_cb->alloc_tspb_err_cnt = 0;
}
#define RESET_DEBUG_TSPB_STAT	reset_debug_tspb_stat()

#define SHOW_DEBUG_TSPB_STAT	\
do {	\
	MTVMSG("max_alloc_seg(%u), max_enqueued_seg(%u) max_enqueued_tsp(%u), alloc_err(%ld)\n",\
		mtv222_cb->max_alloc_seg_cnt,\
		mtv222_cb->max_enqueued_seg_cnt,\
		mtv222_cb->max_enqueued_tsp_cnt,\
		mtv222_cb->alloc_tspb_err_cnt);\
} while (0)

#else
#define RESET_DEBUG_TSPB_STAT		do {} while (0)
#define SHOW_DEBUG_TSPB_STAT		do {} while (0)
#endif /* #ifdef DEBUG_TSP_BUF*/

void mtv222_init_tsp_statistics(void);

bool mtv222_device_power_state(void);
int mtv222_device_power_on(void);
int mtv222_device_power_off(void);

int mtv222_tsb_read(void *buf, unsigned int size);

int mtv222_tsb_disable(void);
int mtv222_tsb_enable(void);

int mtv222_tsb_enqueue(unsigned char *ts_chunk);
unsigned char *mtv222_tsb_get(void);

void mtv222_disable_irq(void);
void mtv222_enable_irq(void);

#endif

