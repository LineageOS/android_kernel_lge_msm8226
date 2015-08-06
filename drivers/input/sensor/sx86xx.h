
#ifndef _SX86XX_H_
#define _SX86XX_H_

/* User is able to use the older method for receiving
 * interrupts, or use the newer more preferred method
 * of threaded interrupts.
 */
#define USE_THREADED_IRQ

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/wakelock.h>

#define MAX_NUM_STATUS_BITS (8)

#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#include <linux/earlysuspend.h>
#include <linux/suspend.h>
#endif

struct sx86XX 
{
    void * bus; /* either i2c_client or spi_client */

    struct device *pdev; /* common device struction for linux */

    void *pDevice; /* device specific struct pointer */

    /* Function Pointers */
    int (*init)(struct sx86XX *this); /* (re)initialize device */

    /* since we are trying to avoid knowing registers, create a pointer to a
     * common read register which would be to read what the interrupt source
     * is from 
     */
    int (*refreshStatus)(struct sx86XX *this); /* read register status */

    int (*get_nirq_low)(void); /* get whether nirq is low (platform data) */

    /* array of functions to call for corresponding status bit */
    void (*statusFunc[MAX_NUM_STATUS_BITS])(struct sx86XX *this); 

#if defined(USE_THREADED_IRQ)
    struct mutex mutex;
#else  
    spinlock_t      lock; /* Spin Lock used for nirq worker function */
#endif 

    int irq; /* irq number used */

    /* whether irq should be ignored.. cases if enable/disable irq is not used
     * or does not work properly
     */
    char irq_disabled;

    u8 useIrqTimer; /* older models need irq timer for pen up cases */

    int irqTimeout; /* msecs only set if useIrqTimer is true */

    /* struct workqueue_struct	*ts_workq;  */  /* if want to use non default */
    struct delayed_work dworker; /* work struct for worker function */
 
#ifdef CONFIG_HAS_WAKELOCK
    struct early_suspend early_suspend;  /* early suspend data  */
#endif

    /* Create a flag to track if we are in touch from a startup detection or not */
    bool inStartupTouch;

    struct wake_lock capsensor_wake_lock;

#ifdef USE_COMMON_SYSFS
	struct kobject	lge_cap_kobj;
#endif
};

void sx86XX_process_interrupt(struct sx86XX *this,u8 nirqlow);
void sx86XX_suspend(struct sx86XX *this);
void sx86XX_resume(struct sx86XX *this);
int sx86XX_init(struct sx86XX *this);
int sx86XX_remove(struct sx86XX *this);

#endif // _SX86XX_H_

