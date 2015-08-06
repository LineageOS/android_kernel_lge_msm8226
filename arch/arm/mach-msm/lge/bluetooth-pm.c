

#include <linux/err.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/regulator/consumer.h>
#include <linux/rfkill.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/wakelock.h>
#include <linux/pm_qos.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/termios.h>
#include <linux/bitops.h>
#include <linux/param.h>


#include <mach/bluetooth_pm.h>


struct bluetooth_pm_data {
	int bt_reset;
	int host_wake;
	int ext_wake;	
	unsigned host_wake_irq;
	spinlock_t slock;
	
	struct rfkill *rfkill;
	struct wake_lock wake_lock;

#ifdef UART_CONTROL_MSM
	struct uart_port *uport;
#endif/*UART_CONTROL_MSM*/

#ifdef QOS_REQUEST_MSM
	struct pm_qos_request dma_qos;
	int dma_qos_request;
#endif/*QOS_REQUEST_MSM*/

#ifdef QOS_REQUEST_TEGRA
	int resume_min_frequency;
	struct pm_qos_request resume_cpu_freq_req;
#endif/*QOS_REQUEST_TEGRA*/
};

#define PROC_DIR	"bluetooth/sleep"


#define ASSERT 		0
#define DEASSERT 	1


/* work function */
static void bluetooth_pm_sleep_work(struct work_struct *work);

/* work queue */
DECLARE_DELAYED_WORK(sleep_workqueue, bluetooth_pm_sleep_work);

/* Macros for handling sleep work */
#define bluetooth_pm_rx_busy()     schedule_delayed_work(&sleep_workqueue, 0)
#define bluetooth_pm_tx_busy()     schedule_delayed_work(&sleep_workqueue, 0)
#define bluetooth_pm_rx_idle()     schedule_delayed_work(&sleep_workqueue, 0)
#define bluetooth_pm_tx_idle()     schedule_delayed_work(&sleep_workqueue, 0)


/* 5 second timeout */
#define TX_TIMER_INTERVAL	5


/* state variable names and bit positions */
#define BT_PROTO	0x01
#define BT_TXDATA	0x02
#define BT_ASLEEP	0x04


/* module usage */
static atomic_t open_count = ATOMIC_INIT(1);


static struct bluetooth_pm_data *bsi;


/** Global state flags */
static unsigned long flags;


/** Tasklet to respond to change in hostwake line */
static struct tasklet_struct hostwake_task;


/** Transmission timer */
static struct timer_list tx_timer;


/** Lock for state transitions */
static spinlock_t rw_lock;


struct proc_dir_entry *bluetooth_dir, *sleep_dir;


#ifdef UART_CONTROL_MSM
/*
 * Local functions
 */
static void hsuart_power(int on)
{	
	if (on) {
		printk("%s, (1)", __func__);
		msm_hs_request_clock_on(bsi->uport);
		msm_hs_set_mctrl(bsi->uport, TIOCM_RTS);
	} else {
		printk("%s, (0)", __func__);
		msm_hs_set_mctrl(bsi->uport, 0);
		msm_hs_request_clock_off(bsi->uport);
	}
}
#endif/*UART_CONTROL_MSM*/


/**
 * @return 1 if the Host can go to sleep, 0 otherwise.
 */
static inline int bluetooth_pm_can_sleep(void)
{
	/* check if MSM_WAKE_BT_GPIO and BT_WAKE_MSM_GPIO are both deasserted */
#ifdef UART_CONTROL_MSM
	return gpio_get_value(bsi->ext_wake) && gpio_get_value(bsi->host_wake) && (bsi->uport != NULL);
#else/*UART_CONTROL_MSM*/
	return gpio_get_value(bsi->ext_wake) && gpio_get_value(bsi->host_wake);
#endif/*UART_CONTROL_MSM*/
}


void bluetooth_pm_sleep_wakeup(void)
{
	if (test_bit(BT_ASLEEP, &flags)) {
		printk("%s, waking up...\n", __func__);

		wake_lock(&bsi->wake_lock);		

		clear_bit(BT_ASLEEP, &flags);	

#ifdef QOS_REQUEST_MSM
		if(bsi->dma_qos_request == REQUESTED) {
			pm_qos_update_request(&bsi->dma_qos, 19); 
		}
#endif/*QOS_REQUEST_MSM*/

#ifdef QOS_REQUEST_TEGRA
		if (bsi->resume_min_frequency)
			pm_qos_update_request(&bsi->resume_cpu_freq_req,
						bsi->resume_min_frequency);
#endif/*QOS_REQUEST_TEGRA*/
		
#ifdef UART_CONTROL_MSM
		/*Activating UART */
		hsuart_power(1);
#endif/*UART_CONTROL_MSM*/
	}
	else
	{
		int wake, host_wake;
		wake = gpio_get_value(bsi->ext_wake);
		host_wake = gpio_get_value(bsi->host_wake);

		printk("%s, %d, %d\n", __func__, wake, host_wake);
	
		if(wake == DEASSERT && host_wake == ASSERT)
		{
			printk("%s, Start Timer : check hostwake status when timer expired\n", __func__);
			mod_timer(&tx_timer, jiffies + (TX_TIMER_INTERVAL * HZ));
		}
		
#ifdef UART_CONTROL_MSM	
		printk("%s, Workaround for uart runtime suspend : Check UART Satus", __func__);
		
		if(bsi->uport != NULL && msm_hs_get_bt_uport_clock_state(bsi->uport) == CLOCK_REQUEST_AVAILABLE)
		{			
			printk("%s, Enter abnormal status, HAVE to Call hsuart_power(1)!", __func__);		
			hsuart_power(1);						
		}		
#endif /*UART_CONTROL_MSM*/		
	}
}


static void bluetooth_pm_sleep_work(struct work_struct *work)
{
	printk("+++ %s\n", __func__);

	if (bluetooth_pm_can_sleep()) {
		printk("%s, bluetooth_pm_can_sleep is true\n", __func__);
		
		/* already asleep, this is an error case */
		if (test_bit(BT_ASLEEP, &flags)) {
			printk("%s, already asleep\n", __func__);
			return;
		}
		
		printk("%s, going to sleep...\n", __func__);
		set_bit(BT_ASLEEP, &flags);		

#ifdef UART_CONTROL_MSM		
		/*Deactivating UART */
		hsuart_power(0);
#endif /*UART_CONTROL_MSM*/

#ifdef QOS_REQUEST_MSM
		if(bsi->dma_qos_request == REQUESTED) {
			pm_qos_update_request(&bsi->dma_qos, 0x7FFFFFF);
		}
#endif /* QOS_REQUEST_MSM */

#ifdef QOS_REQUEST_TEGRA
		pm_qos_update_request(&bsi->resume_cpu_freq_req,
					PM_QOS_DEFAULT_VALUE);
#endif/*QOS_REQUEST_TEGRA*/

		wake_lock_timeout(&bsi->wake_lock, HZ / 2);
	} else {
		printk("%s, bluetooth_pm_can_sleep is false. call BT Wake Up\n", __func__);
		bluetooth_pm_sleep_wakeup();
	}

	printk("--- %s\n", __func__);
}


static void bluetooth_pm_hostwake_task(unsigned long data)
{
	printk("+++ %s\n", __func__);

	spin_lock(&rw_lock);

	if(gpio_get_value(bsi->host_wake) == ASSERT)
	{
		printk("%s, hostwake GPIO ASSERT(Low)\n", __func__);

		bluetooth_pm_rx_busy();

		mod_timer(&tx_timer, jiffies + (TX_TIMER_INTERVAL * HZ));		
	}
	else
	{
		printk("%s, hostwake GPIO High\n", __func__);
	}

	spin_unlock(&rw_lock);

	printk("+++ %s\n", __func__);	
}


static void bluetooth_pm_tx_timer_expire(unsigned long data)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&rw_lock, irq_flags);

	printk("%s, Tx timer expired\n", __func__);

	/* already asleep, this is an error case */
	if (test_bit(BT_ASLEEP, &flags)) {
		printk("%s, already asleep\n", __func__);
		spin_unlock_irqrestore(&rw_lock, irq_flags);
		return;
	}

	bluetooth_pm_tx_idle();	

	spin_unlock_irqrestore(&rw_lock, irq_flags);
}


static irqreturn_t bluetooth_pm_hostwake_isr(int irq, void *dev_id)
{
	/* schedule a tasklet to handle the change in the host wake line */
	int wake, host_wake;
	wake = gpio_get_value(bsi->ext_wake);
	host_wake = gpio_get_value(bsi->host_wake);
	printk("%s, bluesleep_hostwake_isr %d, %d\n", __func__, wake, host_wake);

	irq_set_irq_type(irq, host_wake ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);

	if(host_wake == ASSERT)
	{
		printk("%s, bluesleep_hostwake_isr : Registration Tasklet\n", __func__);
		tasklet_schedule(&hostwake_task);
	}
	
	return IRQ_HANDLED;
}


static int bluetooth_pm_sleep_start(void)
{
	int ret;
	unsigned long irq_flags;
	
	printk("%s\n", __func__);

	spin_lock_irqsave(&rw_lock, irq_flags);

	if (test_bit(BT_PROTO, &flags)) {
		spin_unlock_irqrestore(&rw_lock, irq_flags);
		return 0;
	}

	spin_unlock_irqrestore(&rw_lock, irq_flags);

	if (!atomic_dec_and_test(&open_count)) {
		atomic_inc(&open_count);
		return -EBUSY;
	}

#ifdef QOS_REQUEST_MSM
	if(bsi->dma_qos_request == NOT_REQUESTED) {
		bsi->dma_qos_request = REQUESTED;
		pm_qos_add_request(&bsi->dma_qos, PM_QOS_CPU_DMA_LATENCY, 19);
	}
#endif /* QOS_REQUEST_MSM */	

#ifdef QOS_REQUEST_TEGRA				
		if (bsi->resume_min_frequency)
			pm_qos_add_request(&bsi->resume_cpu_freq_req,
						PM_QOS_CPU_FREQ_MIN,
						PM_QOS_DEFAULT_VALUE);	
#endif/*QOS_REQUEST_TEGRA*/		
	
#ifdef UART_CONTROL_MSM
	/*Activate UART*/
	hsuart_power(1);
#endif/*UART_CONTROL_MSM*/
	
	ret = request_irq(bsi->host_wake_irq, bluetooth_pm_hostwake_isr,
				IRQF_DISABLED | IRQF_TRIGGER_LOW,
				"bluetooth hostwake", NULL);
				
	if (ret  < 0) {
		printk("%s, Couldn't acquire BT_HOST_WAKE IRQ\n", __func__);
		goto fail;
	}

	ret = enable_irq_wake(bsi->host_wake_irq);
	if (ret < 0) {
		printk("%s, Couldn't enable BT_HOST_WAKE as wakeup interrupt\n", __func__);
		free_irq(bsi->host_wake_irq, NULL);
		goto fail;
	}

	set_bit(BT_PROTO, &flags);

	wake_lock(&bsi->wake_lock);

	return 0;
fail:
	atomic_inc(&open_count);

	return ret;
}


/**
 * Stops the Sleep-Mode Protocol on the Host.
 */
static void bluetooth_pm_sleep_stop(void)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&rw_lock, irq_flags);

	if (!test_bit(BT_PROTO, &flags)) {
		spin_unlock_irqrestore(&rw_lock, irq_flags);
		return;
	}

	printk("%s\n", __func__);	

	del_timer(&tx_timer);

	clear_bit(BT_PROTO, &flags);

	if (test_bit(BT_ASLEEP, &flags)) {
		clear_bit(BT_ASLEEP, &flags);
	} 
#ifdef UART_CONTROL_MSM		
	else{
		/*Deactivate UART*/
		//hsuart_power(0); //block uart power off
	}
#endif/*UART_CONTROL_MSM*/
	
	atomic_inc(&open_count);

	spin_unlock_irqrestore(&rw_lock, irq_flags);

#ifdef QOS_REQUEST_MSM
	if(bsi->dma_qos_request == REQUESTED) {
		pm_qos_remove_request(&bsi->dma_qos);
		bsi->dma_qos_request = NOT_REQUESTED;
	}
#endif /* QOS_REQUEST_MSM */	

#ifdef QOS_REQUEST_TEGRA
			if (bsi->resume_min_frequency)
				pm_qos_remove_request(&bsi->resume_cpu_freq_req);
#endif/*QOS_REQUEST_TEGRA*/		
	
	if (disable_irq_wake(bsi->host_wake_irq))
		printk("%s, Couldn't disable hostwake IRQ wakeup mode\n", __func__);
	free_irq(bsi->host_wake_irq, NULL);
	
	wake_lock_timeout(&bsi->wake_lock, HZ / 2);
}

/**
 * Read the <code>BT_WAKE</code> GPIO pin value via the proc interface.
 * When this function returns, <code>page</code> will contain a 1 if the
 * pin is high, 0 otherwise.
 * @param page Buffer for writing data.
 * @param start Not used.
 * @param offset Not used.
 * @param count Not used.
 * @param eof Whether or not there is more data to be read.
 * @param data Not used.
 * @return The number of bytes written.
 */
static int bluetooth_pm_read_proc_btwake(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	*eof = 1;
	return sprintf(page, "btwake:%u\n", gpio_get_value(bsi->ext_wake));
}


/**
 * Write the <code>BT_WAKE</code> GPIO pin value via the proc interface.
 * @param file Not used.
 * @param buffer The buffer to read from.
 * @param count The number of bytes to be written.
 * @param data Not used.
 * @return On success, the number of bytes written. On error, -1, and
 * <code>errno</code> is set appropriately.
 */
static int bluetooth_pm_write_proc_btwake(struct file *file, const char *buffer,
					unsigned long count, void *data)
{
	char *buf;

	if (count < 1)
		return -EINVAL;

	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (copy_from_user(buf, buffer, count)) {
		kfree(buf);
		return -EFAULT;
	}

	if (buf[0] == '0') {
		printk("%s, BT WAKE Set to Wake\n", __func__);
	
		gpio_set_value(bsi->ext_wake, 0);

		bluetooth_pm_sleep_wakeup();	
	} else if (buf[0] == '1') {
		printk("%s, BT WAKE Set to Sleep", __func__);
		
		gpio_set_value(bsi->ext_wake, 1);
	
		bluetooth_pm_tx_idle();
	} else {
		kfree(buf);
		return -EINVAL;
	}

	kfree(buf);
	
	return count;
}


/**
 * Read the <code>BT_HOST_WAKE</code> GPIO pin value via the proc interface.
 * When this function returns, <code>page</code> will contain a 1 if the pin
 * is high, 0 otherwise.
 * @param page Buffer for writing data.
 * @param start Not used.
 * @param offset Not used.
 * @param count Not used.
 * @param eof Whether or not there is more data to be read.
 * @param data Not used.
 * @return The number of bytes written.
 */
static int bluetooth_pm_read_proc_hostwake(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	*eof = 1;
	return sprintf(page, "hostwake: %u \n", gpio_get_value(bsi->host_wake));
}


/**
 * Read the low-power status of the Host via the proc interface.
 * When this function returns, <code>page</code> contains a 1 if the Host
 * is asleep, 0 otherwise.
 * @param page Buffer for writing data.
 * @param start Not used.
 * @param offset Not used.
 * @param count Not used.
 * @param eof Whether or not there is more data to be read.
 * @param data Not used.
 * @return The number of bytes written.
 */
static int bluetooth_pm_read_proc_asleep(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	unsigned int asleep;

	asleep = test_bit(BT_ASLEEP, &flags) ? 1 : 0;
	*eof = 1;
	return sprintf(page, "asleep: %u\n", asleep);
}

/**
 * Read the low-power protocol being used by the Host via the proc interface.
 * When this function returns, <code>page</code> will contain a 1 if the Host
 * is using the Sleep Mode Protocol, 0 otherwise.
 * @param page Buffer for writing data.
 * @param start Not used.
 * @param offset Not used.
 * @param count Not used.
 * @param eof Whether or not there is more data to be read.
 * @param data Not used.
 * @return The number of bytes written.
 */
static int bluetooth_pm_read_proc_proto(char *page, char **start, off_t offset,
					int count, int *eof, void *data)
{
	unsigned int proto;

	proto = test_bit(BT_PROTO, &flags) ? 1 : 0;
	*eof = 1;
	return sprintf(page, "proto: %u\n", proto);
}

/**
 * Modify the low-power protocol used by the Host via the proc interface.
 * @param file Not used.
 * @param buffer The buffer to read from.
 * @param count The number of bytes to be written.
 * @param data Not used.
 * @return On success, the number of bytes written. On error, -1, and
 * <code>errno</code> is set appropriately.
 */
static int bluetooth_pm_write_proc_proto(struct file *file, const char *buffer,
					unsigned long count, void *data)
{
	char proto;

	if (count < 1)
		return -EINVAL;

	if (copy_from_user(&proto, buffer, 1))
		return -EFAULT;

	if (proto == '1')
                bluetooth_pm_sleep_start();
	else
		bluetooth_pm_sleep_stop();

	/* claim that we wrote everything */
	return count;
}


#if defined(CONFIG_BCM4335BT)
/* +++BRCM 4335 AXI Patch */
#define BTLOCK_NAME     "btlock"
#define BTLOCK_MINOR    224
/* BT lock waiting timeout, in second */
#define BTLOCK_TIMEOUT	2


struct btlock {
	int lock;
	int cookie;
};
static struct semaphore btlock;
static int count = 1;
static int owner_cookie = -1;

int bcm_bt_lock(int cookie)
{
	int ret;
	char cookie_msg[5] = {0};

	ret = down_timeout(&btlock, msecs_to_jiffies(BTLOCK_TIMEOUT*1000));
	if (ret == 0) {
		memcpy(cookie_msg, &cookie, sizeof(cookie));
		owner_cookie = cookie;
		count--;
		printk("%s, btlock acquired cookie: %s\n", __func__, cookie_msg);	
	}

	return ret;
}
EXPORT_SYMBOL(bcm_bt_lock);

void bcm_bt_unlock(int cookie)
{
	char owner_msg[5] = {0};
	char cookie_msg[5] = {0};

	memcpy(cookie_msg, &cookie, sizeof(cookie));
	if (owner_cookie == cookie) {
		owner_cookie = -1;
		if (count++ > 1)
			printk("%s, error, release a lock that was not acquired**\n", __func__);
		up(&btlock);
		printk("%s, btlock released, cookie: %s\n", __func__, cookie_msg);
	} else {
		memcpy(owner_msg, &owner_cookie, sizeof(owner_cookie));
		printk("%s, ignore lock release,  cookie mismatch: %s owner %s\n", __func__, cookie_msg,
				owner_cookie == 0 ? "NULL" : owner_msg);
	}
}
EXPORT_SYMBOL(bcm_bt_unlock);

static int btlock_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int btlock_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t btlock_write(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	struct btlock lock_para;
	ssize_t ret = 0;

	if (count < sizeof(struct btlock))
		return -EINVAL;

	if (copy_from_user(&lock_para, buffer, sizeof(struct btlock)))
		return -EFAULT;

	if (lock_para.lock == 0)
		bcm_bt_unlock(lock_para.cookie);
	else if (lock_para.lock == 1)
		ret = bcm_bt_lock(lock_para.cookie);
	else if (lock_para.lock == 2)
		ret = bcm_bt_lock(lock_para.cookie);

	return ret;
}

static long btlock_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return 0;
}

static const struct file_operations btlock_fops = {
	.owner   = THIS_MODULE,
	.open    = btlock_open,
	.release = btlock_release,
	.write   = btlock_write,
	.unlocked_ioctl = btlock_unlocked_ioctl,
};

static struct miscdevice btlock_misc = {
	.name  = BTLOCK_NAME,
	.minor = BTLOCK_MINOR,
	.fops  = &btlock_fops,
};

static int bcm_btlock_init(void)
{
	int ret;

	printk("%s\n", __func__);

	ret = misc_register(&btlock_misc);
	if (ret != 0) {
		printk("%s, Error: failed to register Misc driver,  ret = %d\n", __func__, ret);
		return ret;
	}
	sema_init(&btlock, 1);

	return ret;
}

static void bcm_btlock_exit(void)
{
	printk("%s\n", __func__);

	misc_deregister(&btlock_misc);
}
/* ---BRCM */
#endif /* defined(CONFIG_BCM4335BT) */


static int bluetooth_pm_rfkill_set_power(void *data, bool blocked)
{
	unsigned long irq_flags;
#if defined(CONFIG_BCM4335BT)
	/* +++BRCM 4335 AXI Patch */
	int lock_cookie_bt = 'B' | 'T'<<8 | '3'<<16 | '5'<<24;	/* cookie is "BT35" */
	/* ---BRCM */
#endif /* defined(CONFIG_BCM4335BT) */

	printk("%s: blocked (%d)\n", __func__, blocked);
	
	if (gpio_get_value(bsi->bt_reset) == !blocked)
	{
		printk("%s: Receive same status(%d)\n", __func__, blocked);
		return 0;
	}

	// Check Proto...
	spin_lock_irqsave(&rw_lock, irq_flags);
	
	if (test_bit(BT_PROTO, &flags)) {
		spin_unlock_irqrestore(&rw_lock, irq_flags);

		printk("%s: proto is enabled. kill proto first..\n", __func__);
		bluetooth_pm_sleep_stop();

		mdelay(100);	
	}
	else
	{
		spin_unlock_irqrestore(&rw_lock, irq_flags);
	}	

	if (blocked) {//	BT Off...	
		if (bsi->bt_reset)
			gpio_set_value(bsi->bt_reset, 0);
	} else {// BT On...
#if defined(CONFIG_BCM4335BT)
		/* +++BRCM 4335 AXI Patch */
		if (bcm_bt_lock(lock_cookie_bt) != 0)
			printk("** BT rfkill: timeout in acquiring bt lock**\n");
		/* ---BRCM */
#endif /* defined(CONFIG_BCM4335BT) */
	
		if (bsi->bt_reset)
			gpio_set_value(bsi->bt_reset, 0);

		mdelay(30);	
		
		if (bsi->bt_reset)
			gpio_set_value(bsi->bt_reset, 1);
	}

	return 0;
}


static const struct rfkill_ops bluetooth_pm_rfkill_ops = {
	.set_block = bluetooth_pm_rfkill_set_power,
};


static int bluetooth_pm_suspend(struct platform_device *pdev,
						pm_message_t state)
{
	printk("%s:\n", __func__);

	return 0;
}

static int bluetooth_pm_resume(struct platform_device *pdev)
{
	printk("%s:\n", __func__);

	return 0;
}


static int bluetooth_pm_create_bt_proc_interface(void)
{
	int ret;
	struct proc_dir_entry *ent;

	bluetooth_dir = proc_mkdir("bluetooth", NULL);
	if (bluetooth_dir == NULL) {
		printk("%s, Unable to create /proc/bluetooth directory\n", __func__);
		return -ENOMEM;
	}

	sleep_dir = proc_mkdir("sleep", bluetooth_dir);
	if (sleep_dir == NULL) {
		printk("%s, Unable to create /proc/%s directory\n", __func__, PROC_DIR);
		return -ENOMEM;
	}

	/* Creating read/write "btwake" entry */
	ent = create_proc_entry("btwake", 0, sleep_dir);
	if (ent == NULL) {
		printk("%s, Unable to create /proc/%s/btwake entry\n", __func__, PROC_DIR);
		ret = -ENOMEM;
		goto fail;
	}
	ent->read_proc = bluetooth_pm_read_proc_btwake;
	ent->write_proc = bluetooth_pm_write_proc_btwake;
	
#ifdef BTA_NOT_USE_ROOT_PERM
    ent->uid = AID_BLUETOOTH;
    ent->gid = AID_NET_BT_STACK;
#endif  //BTA_NOT_USE_ROOT_PERM

	/* read only proc entries */
	if (create_proc_read_entry("hostwake", 0, sleep_dir,
				bluetooth_pm_read_proc_hostwake, NULL) == NULL) {
		printk("%s, Unable to create /proc/%s/hostwake entry\n", __func__, PROC_DIR);
		ret = -ENOMEM;
		goto fail;
	}

	/* read/write proc entries */
	ent = create_proc_entry("proto", 0, sleep_dir);
	if (ent == NULL) {
		printk("%s, Unable to create /proc/%s/proto entry\n", __func__, PROC_DIR);
		ret = -ENOMEM;
		goto fail;
	}
	ent->read_proc = bluetooth_pm_read_proc_proto;
	ent->write_proc = bluetooth_pm_write_proc_proto;
	
#ifdef BTA_NOT_USE_ROOT_PERM
    ent->uid = AID_BLUETOOTH;
    ent->gid = AID_NET_BT_STACK;
#endif  //BTA_NOT_USE_ROOT_PERM

	/* read only proc entries */
	if (create_proc_read_entry("asleep", 0,
			sleep_dir, bluetooth_pm_read_proc_asleep, NULL) == NULL) {
		printk("%s, Unable to create /proc/%s/asleep entry\n", __func__, PROC_DIR);
		ret = -ENOMEM;
		goto fail;
	}

	return 0;

fail:
	remove_proc_entry("asleep", sleep_dir);
	remove_proc_entry("proto", sleep_dir);
	remove_proc_entry("hostwake", sleep_dir);
	remove_proc_entry("btwake", sleep_dir);
	remove_proc_entry("sleep", bluetooth_dir);
	remove_proc_entry("bluetooth", 0);
	return ret;
}



static int bluetooth_pm_probe(struct platform_device *pdev)
{
	int ret;
#if 0	
	struct resource *res;
#endif
	struct rfkill *rfkill;

#if defined(CONFIG_BCM4335BT)
	/* +++BRCM 4335 AXI Patch */
	bcm_btlock_init();
	/* ---BRCM */
#endif /* defined(CONFIG_BCM4335BT) */


	bsi = kzalloc(sizeof(struct bluetooth_pm_data), GFP_KERNEL);
	if (!bsi)
		return -ENOMEM;	


	// BT_RESET_N
	bsi->bt_reset = BT_RESET_GPIO;
	ret = gpio_request(bsi->bt_reset, "bt_reset");
	if (ret) {
		printk("%s: Failed to get bt_reset gpio\n", __func__);
		goto free_res;
	}
	gpio_direction_output(bsi->bt_reset, 0);

	// RFKILL INIT...
	if (bsi->bt_reset) {
		rfkill = rfkill_alloc(pdev->name, &pdev->dev,
				RFKILL_TYPE_BLUETOOTH, &bluetooth_pm_rfkill_ops,
				bsi);

		if (unlikely(!rfkill))
			goto free_res;
		
		rfkill_set_states(rfkill, 1, 0);

		ret = rfkill_register(rfkill);

		if (unlikely(ret)) {
			rfkill_destroy(rfkill);
			kfree(rfkill);
			goto free_res;
		}
		bsi->rfkill = rfkill;
	}	


	// BT_HOST_WAKE
	bsi->host_wake = BT_HOST_WAKE_GPIO;
	ret = gpio_request(bsi->host_wake, "bt_host_wake");
	if (ret) {
		printk("%s: Failed to get bt_host_wake gpio\n", __func__);
		goto free_res;
	}
	gpio_direction_input(bsi->host_wake);	

	// BT_EXT_WAKE
	bsi->ext_wake = BT_EXT_WAKE_GPIO;
	ret = gpio_request(bsi->ext_wake, "bt_wake");
	if (ret) {
		printk("%s: Failed to get bt_wake gpio\n", __func__);
		goto free_res;
	}
	gpio_direction_output(bsi->ext_wake, 0);		

	// BT_HOST_WAKE_IRQ
	bsi->host_wake_irq = gpio_to_irq(BT_HOST_WAKE_GPIO);

#ifdef UART_CONTROL_MSM
	bsi->uport= msm_hs_get_bt_uport(BT_PORT_NUM);
#endif/*UART_CONTROL_MSM*/

#ifdef QOS_REQUEST_MSM
		bsi->dma_qos_request = NOT_REQUESTED;
#endif /* QOS_REQUEST_MSM */

#ifdef QOS_REQUEST_TEGRA
		bsi->resume_min_frequency = 204000;
#endif/*QOS_REQUEST_TEGRA*/

	wake_lock_init(&bsi->wake_lock, WAKE_LOCK_SUSPEND, "bluetooth_pm");

	flags = 0; /* clear all status bits */

	/* Initialize spinlock. */
	spin_lock_init(&rw_lock);

	/* Initialize timer */
	init_timer(&tx_timer);
	tx_timer.function = bluetooth_pm_tx_timer_expire;
	tx_timer.data = 0;

	/* initialize host wake tasklet */
	tasklet_init(&hostwake_task, bluetooth_pm_hostwake_task, 0);

	bluetooth_pm_create_bt_proc_interface();

	return 0;

free_res:	
	if(bsi->ext_wake)
		gpio_free(bsi->ext_wake);
	if(bsi->host_wake)
		gpio_free(bsi->host_wake);
	if(bsi->bt_reset)
		gpio_free(bsi->bt_reset);
	if (bsi->rfkill) {
		rfkill_unregister(bsi->rfkill);
		rfkill_destroy(bsi->rfkill);
		kfree(bsi->rfkill);
	}
	kfree(bsi);
	
	return ret;
}


static void bluetooth_pm_remove_bt_proc_interface(void)
{
	remove_proc_entry("asleep", sleep_dir);
	remove_proc_entry("proto", sleep_dir);
	remove_proc_entry("hostwake", sleep_dir);
	remove_proc_entry("btwake", sleep_dir);
	remove_proc_entry("sleep", bluetooth_dir);
	remove_proc_entry("bluetooth", 0);
}


static int bluetooth_pm_remove(struct platform_device *pdev)
{
	/* assert bt wake */
	gpio_set_value(bsi->ext_wake, 0);
	if (test_bit(BT_PROTO, &flags)) {
		if (disable_irq_wake(bsi->host_wake_irq))
			printk("%s, Couldn't disable hostwake IRQ wakeup mode \n", __func__);
		free_irq(bsi->host_wake_irq, NULL);
		del_timer(&tx_timer);
	}

	bluetooth_pm_remove_bt_proc_interface();

	if (bsi->ext_wake)
		gpio_free(bsi->ext_wake);		
	if (bsi->host_wake)
		gpio_free(bsi->host_wake);		
	if (bsi->rfkill) {
		rfkill_unregister(bsi->rfkill);
		rfkill_destroy(bsi->rfkill);
		kfree(bsi->rfkill);
	}
	if (bsi->bt_reset)
		gpio_free(bsi->bt_reset);

	wake_lock_destroy(&bsi->wake_lock);
	
	kfree(bsi);
	
#if defined(CONFIG_BCM4335BT)
	/* +++BRCM 4335 AXI Patch */
	bcm_btlock_exit();
	/* ---BRCM */
#endif /* defined(CONFIG_BCM4335BT) */

	return 0;
}


static struct platform_device bluetooth_pm_device = {
	.name = "bluetooth_pm",
	.id             = -1,
};


static struct platform_driver bluetooth_pm_driver = {
	.probe = bluetooth_pm_probe,
	.remove = bluetooth_pm_remove,
	.suspend = bluetooth_pm_suspend,
	.resume = bluetooth_pm_resume,
	.driver = {
		   .name = "bluetooth_pm",
		   .owner = THIS_MODULE,
	},
};


static int __init bluetooth_pm_init(void)
{
	int ret;
	
	printk("+++bluetooth_pm_init\n");
	ret = platform_driver_register(&bluetooth_pm_driver);
	if (ret)
		printk("Fail to register bluetooth_pm platform driver\n");
	printk("---bluetooth_pm_init done\n");

	return platform_device_register(&bluetooth_pm_device);
}


static void __exit bluetooth_pm_exit(void)
{
	platform_driver_unregister(&bluetooth_pm_driver);
}

//device_initcall(bluetooth_pm_init);
late_initcall_sync(bluetooth_pm_init);
module_exit(bluetooth_pm_exit);

MODULE_DESCRIPTION("bluetooth PM");
MODULE_AUTHOR("UNKNOWN");
#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif
