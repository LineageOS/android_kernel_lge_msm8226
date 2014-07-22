/*
 *  linux/drivers/mmc/mem_log.c
 *
 *  Copyright 2011-2012 joys
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

//************************************************************************************

#include <linux/mmc/mem_log.h>
#include <linux/delay.h>
#include <linux/export.h>


mem_log_t tmemLog;

#define MEMLOG_COPY_TO_USER 1
#include <linux/miscdevice.h>

#if MEMLOG_COPY_TO_USER 
/*
 * Debug Level
 *	2 : Print all debug messages
 *	1 : Print only dbg_memlog() messages
 *	0 : No debug messages
 */
#define MEMLOG_DEBUG_LEVEL	0
#define DRV_NAME "memlog"

#if (MEMLOG_DEBUG_LEVEL == 2)
#define dbg_memlog(format, arg...)	\
	printk(KERN_ALERT DRV_NAME ": Debug: " format, ## arg)
#define enter()			\
	printk(KERN_ALERT DRV_NAME ": Enter: %s\n", __func__)
#define leave()			\
	printk(KERN_ALERT DRV_NAME ": Leave: %s\n", __func__)
#elif (MEMLOG_DEBUG_LEVEL == 1)
#define dbg_memlog(format, arg...)	\
	printk(KERN_ALERT DRV_NAME ": Debug: " format, ## arg)
#define enter()
#define leave()
#else
#define dbg_memlog(format, arg...)
#define enter()
#define leave()
#endif

#define IOCTL_START_MEMLOG	_IO('u', 0x1)
#define IOCTL_STOP_MEMLOG	_IO('u', 0x0)
#define IOCTL_MEMLOG_BUFFER_RELEASE	_IO('u', 0x2)
#define IOCTL_MEMLOG_CURRENT_INDEX_COUNT	_IOR('u', 0x3, unsigned long)

int memlog_open(struct inode *inode, struct file *filp)
{
	enter();

	leave();
	return 0;
}

ssize_t memlog_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	unsigned long buf_len = 0;
	int result=0;

	enter();
	
	buf_len = __MEM_LOG_BUF_LEN / sizeof(mem_log_parcer_t);		// total size = 16MB, available sampling size = 621378
	
	dbg_memlog("total sampling buffer count = %d, read_count = %d, mem_log_parcer_t = %d bytes\n", buf_len, count, sizeof(mem_log_parcer_t));

	result = copy_to_user(buf, (const void *)tmemLog.log_buf, count); 	// (sizeof(mem_log_parcer_t)*buf_len));

	if(result<0)
	{
		dbg_memlog("error copy_to_user memory..!!\n");
		return result;
	}
	dbg_memlog("success copy_to_user memory..!! result = %d \n", result);

	leave();
	return count;
}

ssize_t memlog_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
	enter();

	leave();
	return count;
}

static DEFINE_MUTEX(memlog_mutex);
long memlog_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int size;
	int ctu_result;

	enter();

	size = _IOC_SIZE(cmd);

	mutex_lock(&memlog_mutex);
	switch(cmd){
		case IOCTL_START_MEMLOG:
			memlog_set_enable(1);		
			memlog_app_add(0, 7);
			dbg_memlog("Start Log Capture..!! \n");
			break;

		case IOCTL_STOP_MEMLOG:
			memlog_app_add(0, 7);
			memlog_set_enable(0);		
			dbg_memlog("Stop Log Capture..!! \n");
			break;
		
		case IOCTL_MEMLOG_BUFFER_RELEASE:
			memlog_release();
			dbg_memlog("MEM Log Buffer Release..!! \n");
			break;

		case IOCTL_MEMLOG_CURRENT_INDEX_COUNT:
			//	unsigned long cur_index = tmemLog.cur_index;
			//put_user(tmemLog.cur_index,(void *) arg); 
			dbg_memlog("MEM Log current index count = %d , cmd size = %d \n", tmemLog.cur_index, size);
			ctu_result = copy_to_user((void *) arg, (const void *) &tmemLog.cur_index, (unsigned long) size);
			break;

		default:
			break;
	}
	mutex_unlock(&memlog_mutex);

	leave();
	return 0;
}

int memlog_close(struct inode *inode, struct file *filp)
{
	enter();
	leave();
	return 0;
}


struct file_operations memlog_fops =
{
    .owner = THIS_MODULE,
//	.ioctl = memlog_ioctl,
	.unlocked_ioctl = memlog_ioctl,
	.read = memlog_read,
	.write = memlog_write,
	.open = memlog_open,
	.release = memlog_close,
};

static struct miscdevice memlog_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "memlog",
	.fops = &memlog_fops,
};	
#endif // MEMLOG_COPY_TO_USER

//mem_log_t tmemLog;

int init_memLog(void)
{		
	unsigned long buf_len = 0;
	
	memset(&tmemLog, 0, sizeof(mem_log_t));

	buf_len = __MEM_LOG_BUF_LEN / sizeof(mem_log_parcer_t);

	if (tmemLog.log_buf == NULL)
		tmemLog.log_buf = (mem_log_parcer_t*)vmalloc(sizeof(mem_log_parcer_t) * buf_len);
	
	if (tmemLog.log_buf == NULL) {
		_err_msg("Memory alloc fail!\n");
		return -1;
	}
	
	tmemLog.max_index = buf_len;
	tmemLog.cur_index = 0;
	spin_lock_init(&tmemLog.lock);

	_err_msg("Init Success! // Available Max Line : %lu\n", tmemLog.max_index);
	_err_msg("Init Success! // mem log buf len : %d // sizeof(mem_log_parcer_t) : %d\n", __MEM_LOG_BUF_LEN, sizeof(mem_log_parcer_t));

#if MEMLOG_COPY_TO_USER 
	{
		int result;
		result = misc_register(&memlog_miscdev);
		//result = register_chrdev(250, "memlog", &memlog_fops);
		if(result<0)
		{
			dbg_memlog("error for register char device driver\n");
			return result;
		}
		dbg_memlog("success to register char fileops \n");
	}
#endif

#if 0 	// booting test
	memlog_set_enable(1);		
	memlog_app_add(0, 7);
#endif	

	return 0;
}

int memlog_insert(mem_log_parcer_t* log_parcer)
{
	_mdbg_msg("cur : %lu // max : %lu\n", tmemLog.cur_index, tmemLog.max_index);
	
	if (tmemLog.cur_index >= tmemLog.max_index) {
		_err_msg("Max log buff is full!\n");
		return -1;
	}

//	memset(&tmemLog.log_buf[*cur_index], 0, sizeof(mem_log_parcer_t));
	memcpy(&tmemLog.log_buf[tmemLog.cur_index], log_parcer, sizeof(mem_log_parcer_t));

	_mdbg_msg("flag : %d \n", tmemLog.log_buf[tmemLog.cur_index].flag);

	tmemLog.cur_index++;
	
	return 0;
}

int memlog_parcer_print(mem_log_parcer_t* log_parcer)
{
	char log_buf[100] = {0,};
	char cmd[10] = {0,};

	if (log_parcer == NULL)
		return 0;
	
	if (mem_target_getopt(log_parcer, MEM_LOG_APP)) 
	{
		if (mem_app_getopt(log_parcer, MEM_LOG_APP_START))  
		{
			sprintf(log_buf, "Start logging");			
		}
		else {
			sprintf(log_buf, "End logging");
		}
	}

	if (mem_target_getopt(log_parcer, MEM_LOG_MMC)) {
		if (mem_cmd_getopt(log_parcer, MEM_LOG_READ))
			sprintf(cmd, "READ");
		else if (mem_cmd_getopt(log_parcer, MEM_LOG_WRITE))
			sprintf(cmd, "WRITE");
		else if (mem_cmd_getopt(log_parcer, MEM_LOG_OPCODE))
			sprintf(cmd, "COMMAND");
		else if (mem_cmd_getopt(log_parcer, MEM_LOG_PACKED))
			sprintf(cmd, "PACKED");
		
		if(!strcmp(cmd, "COMMAND"))
		{
			sprintf(log_buf, "%15llu elapsed CMD%4u(%s) arg %u ", log_parcer->latency_time, log_parcer->opcode,
		       			cmd, log_parcer->sector);

		}
		else if(!strcmp(cmd, "PACKED"))
		{
			sprintf(log_buf, "packed arg : %u", log_parcer->packed_cmd_hdr);
		}
		else
		{
			sprintf(log_buf, "%15llu elapsed CMD%4u(%s) block %u (%u sectors) ", log_parcer->latency_time, log_parcer->opcode,
		       			cmd, log_parcer->sector, log_parcer->sector_len);
		}
	}
	printk(KERN_INFO "%s\n", log_buf);
	return 0;
}

int memlog_release(void)
{
	memset(tmemLog.log_buf, 0, sizeof(mem_log_parcer_t) * tmemLog.max_index);
	/*
	if (tmemLog.log_buf != NULL)
		vfree(tmemLog.log_buf);
	*/
	
	tmemLog.cur_index = 0;
	tmemLog.start = MEM_LOG_APP_END;
	return 0;
}

int memlog_destroy(void)
{
	if (tmemLog.log_buf != NULL)
		vfree(tmemLog.log_buf);

	tmemLog.max_index = 0;
	tmemLog.cur_index = 0;
	return 0;
}

void memlog_exit_print_thread(void)
{
	kthread_stop(tmemLog.kthread);
}

static inline void sleep(unsigned sec)
{
	current->state = TASK_INTERRUPTIBLE;
	schedule_timeout(sec * HZ);
}

static int memlog_print_thread(void* arg)
{
	unsigned long index = 0;
	unsigned long cur_index = tmemLog.cur_index;
	mem_log_parcer_t* buf = tmemLog.log_buf;
	
	if (cur_index < 1)
		return 0;
	
	do {
		set_current_state(TASK_INTERRUPTIBLE);

		if (kthread_should_stop()) {
			break;
		}		
		memlog_parcer_print(&buf[index]);

		index++;
		schedule_timeout(0);
		//ndelay(1);
		
	} while (index < cur_index);
	set_current_state(TASK_RUNNING);	
	//memlog_parcer_print(&buf[cur_index - 1]);

	memlog_release();
	return 0;
}

int memlog_print(void)
{
	int err = 0;
	tmemLog.kthread = kthread_run(memlog_print_thread, NULL, "memlog_thread");
	if (IS_ERR(tmemLog.kthread)) {
		err = PTR_ERR(tmemLog.kthread);
		_err_msg("Fail memlog_thread Start! err = %d\n", err);
		return -1;
	}
	return 0;
}

int memlog_emmc_add(struct mmc_request *mrq, unsigned long long curTime, unsigned long long latency)
{
	mem_log_parcer_t log_parcer;
	unsigned long flags;

	spin_lock_irqsave(&tmemLog.lock, flags);
	
	if (tmemLog.enable == 0) {
		spin_unlock_irqrestore(&tmemLog.lock, flags);
		return 0;
	}
	memset(&log_parcer, 0, sizeof(mem_log_parcer_t));
	
	mem_target_setopt(&log_parcer, MEM_LOG_MMC);

	if(mrq->data)
	{
		if (mrq->data->flags == MMC_DATA_WRITE)
			mem_cmd_setopt(&log_parcer, MEM_LOG_WRITE);
		else if (mrq->data->flags == MMC_DATA_READ)
			mem_cmd_setopt(&log_parcer, MEM_LOG_READ);
		log_parcer.sector = (unsigned int)mrq->cmd->arg;
		log_parcer.sector_len = mrq->data->blocks;
	}
	else
	{
		mem_cmd_setopt(&log_parcer, MEM_LOG_OPCODE);
		log_parcer.sector = (unsigned int)mrq->cmd->arg;
		log_parcer.sector_len = 0;
	}

	log_parcer.cur_time = curTime;
	log_parcer.latency_time = latency;
	log_parcer.opcode = (unsigned int)mrq->cmd->opcode;

	memlog_insert(&log_parcer);

	spin_unlock_irqrestore(&tmemLog.lock, flags);
	
	return 0;
}

int memlog_packed_add(u32 packed_cmd_hdr, u8 hdr)
{
	mem_log_parcer_t log_parcer;
	unsigned long flags;

	spin_lock_irqsave(&tmemLog.lock, flags);

	if(tmemLog.enable == 0){
		spin_unlock_irqrestore(&tmemLog.lock, flags);
		return 0;
	}
	memset(&log_parcer, 0, sizeof(mem_log_parcer_t));
	
	mem_target_setopt(&log_parcer, MEM_LOG_MMC);
	mem_cmd_setopt(&log_parcer, MEM_LOG_PACKED);
	
	log_parcer.packed_cmd_hdr = packed_cmd_hdr;
	log_parcer.hdr = hdr;
	memlog_insert(&log_parcer);

	spin_unlock_irqrestore(&tmemLog.lock, flags);

	return 0;
}


int memlog_opcode_add(struct mmc_request *mrq, unsigned long long curTime, unsigned long long latency)
{
	mem_log_parcer_t log_parcer;
	unsigned long flags;

	spin_lock_irqsave(&tmemLog.lock, flags);
	
	if (tmemLog.enable == 0) {
		spin_unlock_irqrestore(&tmemLog.lock, flags);
		return 0;
	}
	memset(&log_parcer, 0, sizeof(mem_log_parcer_t));
	
	mem_target_setopt(&log_parcer, MEM_LOG_MMC);

	mem_cmd_setopt(&log_parcer, MEM_LOG_OPCODE);

	log_parcer.sector = (unsigned int)mrq->cmd->arg;
	log_parcer.sector_len = 0;
	log_parcer.cur_time = curTime;
	log_parcer.latency_time = latency;
	log_parcer.opcode = (unsigned int)mrq->cmd->opcode;

	memlog_insert(&log_parcer);

	spin_unlock_irqrestore(&tmemLog.lock, flags);
	
	return 0;
}

int memlog_app_add(int start, int name)
{
	mem_log_parcer_t log_parcer;
	unsigned long flags;

	spin_lock_irqsave(&tmemLog.lock, flags);
	
	if (tmemLog.enable == 0) {
		spin_unlock_irqrestore(&tmemLog.lock, flags);
		return 0;
	}

	memset(&log_parcer, 0, sizeof(mem_log_parcer_t));
	mem_target_setopt(&log_parcer, MEM_LOG_APP);

	if (name == MEM_APP_MANUAL) {
		if (tmemLog.start == 0) {
			mem_app_setopt(&log_parcer, MEM_LOG_APP_START);
		}
		else {
			mem_app_setopt(&log_parcer, MEM_LOG_APP_END);
		}	
			
	}
	else {
		if (start == MEM_LOG_APP_START) {
			mem_app_setopt(&log_parcer, MEM_LOG_APP_START);
		}
		else {
			mem_app_setopt(&log_parcer, MEM_LOG_APP_END);
		}
	}
	
	log_parcer.cur_time = sched_clock();
	log_parcer.app_num = name;
	memlog_insert(&log_parcer);

	tmemLog.start = MEM_LOG_APP_START;
	if (start == MEM_LOG_APP_START)
		_err_msg("flag :%d // start :%d // name : %d // time: %llu\n", log_parcer.flag, start, name, log_parcer.cur_time);

	spin_unlock_irqrestore(&tmemLog.lock, flags);
	
	return 0;
}

int memlog_set_enable(int enable)
{
	unsigned long flags;
	
	spin_lock_irqsave(&tmemLog.lock, flags);
	tmemLog.enable = enable;
	spin_unlock_irqrestore(&tmemLog.lock, flags);

	if (enable == 1)
		_err_msg("Enable Memory Log! // Available Line : %lu\n", tmemLog.max_index - tmemLog.cur_index);
	else
		_err_msg("Disable Memory Log!\n");
	
	return 0;
}
