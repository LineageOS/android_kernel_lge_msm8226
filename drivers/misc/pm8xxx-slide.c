/*
 * Copyright LG Electronics (c) 2011
 * All rights reserved.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mfd/pm8xxx/core.h>
#include <linux/mfd/pm8xxx/slide.h>
#include <linux/gpio.h>
#include <linux/switch.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/wakelock.h>
#include <mach/board_lge.h>

struct pm8xxx_slide {
	struct switch_dev sdev;
	struct switch_dev hani_sdev;
	struct delayed_work slide_work;
	struct device *dev;
	struct wake_lock wake_lock;
	const struct pm8xxx_slide_platform_data *pdata;
	spinlock_t lock;
	int state;
	int hall_state;
};

#ifdef CONFIG_MACH_MSM8926_AKA_CN
#include <mach/msm_smem.h>

typedef struct {
    unsigned int cable_type;        /* PIF detection */
    unsigned int qem;               /* QEM */
    unsigned int build_info;
    unsigned int reserved;
    uint8_t port_type;         /* TA/USB */
} smem_vendor1_type;

int pm8xxx_get_qem(void)
{
	int smem_size = 0;
	smem_vendor1_type* smem_vendor1_type_ptr = (smem_vendor1_type *)
		(smem_get_entry(SMEM_ID_VENDOR1, &smem_size));

	if (smem_vendor1_type_ptr != NULL) {
		return smem_vendor1_type_ptr->qem;
	} else {
		return 0;
	}
}
#endif

#define SLIDE_DETECT_DELAY 150

static struct workqueue_struct *slide_wq;
static struct pm8xxx_slide *slide;

bool top_irq_mask = 1;
bool bottom_irq_mask = 1;

static void slide_top_enable_irq(unsigned int irq)
{
	if(!top_irq_mask){
		top_irq_mask = 1;
		enable_irq_wake(irq);
		enable_irq(irq);
		printk("%s : [Slide] slide_top_enable_irq\n", __func__);
	}
}

static void slide_top_disable_irq(unsigned int irq)
{
	if(top_irq_mask){
		top_irq_mask = 0;
		disable_irq_wake(irq);
		disable_irq(irq);
		printk("%s : [Slide] slide_disable_irq\n", __func__);
	}
}

static void slide_bottom_enable_irq(unsigned int irq)
{
	if(!bottom_irq_mask){
		bottom_irq_mask = 1;
		enable_irq_wake(irq);
		enable_irq(irq);
		printk("%s : [Slide] slide_bottom_enable_irq\n", __func__);
	}
}

static void slide_bottom_disable_irq(unsigned int irq)
{
	if(bottom_irq_mask){
		bottom_irq_mask = 0;
		disable_irq_wake(irq);
		disable_irq(irq);
		printk("%s : [Slide] slide_bottom_disable_irq\n", __func__);
	}
}

bool slide_boot_mode(void)
{
	enum lge_boot_mode_type boot_mode;
	boot_mode = lge_get_boot_mode();
	if (boot_mode == LGE_BOOT_MODE_QEM_56K || boot_mode == LGE_BOOT_MODE_QEM_130K)
	{
		printk("[Slide] boot_mode == 56K || 130K\n");
		return 1;
	}
	printk("[Slide] boot_mode ==%d\n", boot_mode);
	return 0;
}
EXPORT_SYMBOL(slide_boot_mode);

static void boot_slide_factory_func(void)
{
	int slide_state = 0;
	unsigned long flags;
	int tmp_top = 0;
	int tmp_bottom = 0;

	spin_lock_irqsave(&slide->lock, flags);

	if (slide->pdata->hallic_top_detect_pin)
		tmp_top = !gpio_get_value(slide->pdata->hallic_top_detect_pin);
	if (slide->pdata->hallic_bottom_detect_pin)
		tmp_bottom = !gpio_get_value(slide->pdata->hallic_bottom_detect_pin);

	if (tmp_top == 1 && tmp_bottom == 1)
		slide_state = SMARTCOVER_SLIDE_CLOSED;
	else if (tmp_top == 0 && tmp_bottom == 1)
		slide_state = SMARTCOVER_SLIDE_HALF;
	else if (tmp_top == 0 && tmp_bottom == 0)
		slide_state = SMARTCOVER_SLIDE_OPENED;
	else
		slide_state = SMARTCOVER_SLIDE_TOP;

	if (slide->state != slide_state) {
		slide->state = slide_state;
		spin_unlock_irqrestore(&slide->lock, flags);
		wake_lock_timeout(&slide->wake_lock, msecs_to_jiffies(3000));
		switch_set_state(&slide->sdev, slide->state);
		switch_set_state(&slide->hani_sdev, slide->state);
		printk("%s : [Slide] Slide value is %d\n", __func__ , slide_state);
	}
	else {
		spin_unlock_irqrestore(&slide->lock, flags);
		printk("%s : [Slide] Slide value is %d (no change)\n", __func__ , slide_state);
	}
}

void pm8xxx_slide_enable(void)
{
	slide_top_enable_irq(slide->pdata->hallic_top_irq);
	slide_bottom_enable_irq(slide->pdata->hallic_bottom_irq);
	printk("%s : [Slide] Authentication Success, Enable IRQ\n", __func__ );

}
EXPORT_SYMBOL(pm8xxx_slide_enable);

void pm8xxx_slide_boot_func(void)

{
	int slide_state = 0;
	unsigned long flags;
	int tmp_top = 0;
	int tmp_bottom = 0;

	slide_top_enable_irq(slide->pdata->hallic_top_irq);
	slide_bottom_enable_irq(slide->pdata->hallic_bottom_irq);
	printk("%s : [Slide] Authentication Success, Enable IRQ\n", __func__ );

	spin_lock_irqsave(&slide->lock, flags);

	if (slide->pdata->hallic_top_detect_pin)
		tmp_top = !gpio_get_value(slide->pdata->hallic_top_detect_pin);
	if (slide->pdata->hallic_bottom_detect_pin)
		tmp_bottom = !gpio_get_value(slide->pdata->hallic_bottom_detect_pin);

	if (tmp_top == 1 && tmp_bottom == 1)
		slide_state = SMARTCOVER_SLIDE_CLOSED;
	else if (tmp_top == 0 && tmp_bottom == 1)
		slide_state = SMARTCOVER_SLIDE_HALF;
	else
		slide_state = SMARTCOVER_SLIDE_OPENED;

	if (slide->state != slide_state) {
		slide->state = slide_state;
		slide->hall_state = slide_state;
		spin_unlock_irqrestore(&slide->lock, flags);
		wake_lock_timeout(&slide->wake_lock, msecs_to_jiffies(3000));
		switch_set_state(&slide->sdev, slide->state);
		printk("%s : [Slide] Slide value is %d\n", __func__ , slide_state);
	}
	else {
		spin_unlock_irqrestore(&slide->lock, flags);
		printk("%s : [Slide] Slide value is %d (no change)\n", __func__ , slide_state);
	}

}
EXPORT_SYMBOL(pm8xxx_slide_boot_func);

void pm8xxx_slide_disable(void)
{
#ifdef CONFIG_MACH_MSM8926_AKA_CN
    if(!pm8xxx_get_qem()) {
#endif

	slide_top_disable_irq(slide->pdata->hallic_top_irq);
	slide_bottom_disable_irq(slide->pdata->hallic_bottom_irq);
	switch_set_state(&slide->sdev, 0);
	printk("%s : [Slide] Disable IRQ\n", __func__ );
#ifdef CONFIG_MACH_MSM8926_AKA_CN
    }
#endif
}
EXPORT_SYMBOL(pm8xxx_slide_disable);

static void pm8xxx_slide_work_func(struct work_struct *work)
{
	int slide_state = 0;
	int hall_state = 0;
	unsigned long flags;
	int tmp_top = 0;
	int tmp_bottom = 0;

	spin_lock_irqsave(&slide->lock, flags);

	if (slide->pdata->hallic_top_detect_pin)
		tmp_top = !gpio_get_value(slide->pdata->hallic_top_detect_pin);
	if (slide->pdata->hallic_bottom_detect_pin)
		tmp_bottom = !gpio_get_value(slide->pdata->hallic_bottom_detect_pin);

	if (tmp_top == 1 && tmp_bottom == 1)
		slide_state = SMARTCOVER_SLIDE_CLOSED;
	else if (tmp_top == 0 && tmp_bottom == 1)
		slide_state = SMARTCOVER_SLIDE_HALF;
	else if (tmp_top == 0 && tmp_bottom == 0)
		slide_state = SMARTCOVER_SLIDE_OPENED;
	else
		slide_state = SMARTCOVER_SLIDE_TOP;

	hall_state = slide_state;

	if (slide->hall_state != hall_state) {
		slide->state = slide_state;
		slide->hall_state = hall_state;
		spin_unlock_irqrestore(&slide->lock, flags);
		wake_lock_timeout(&slide->wake_lock, msecs_to_jiffies(3000));

		if (SMARTCOVER_SLIDE_TOP == slide->state)
		 slide->state = SMARTCOVER_SLIDE_OPENED;

		switch_set_state(&slide->sdev, slide->state);
		switch_set_state(&slide->hani_sdev, slide->hall_state);
		printk("%s : [Slide] Slide value is %d\n", __func__ , slide->state);
	}
	else {
		spin_unlock_irqrestore(&slide->lock, flags);
		printk("%s : [Slide] Slide value is %d (no change)\n", __func__ , slide_state);
	}
}


static irqreturn_t pm8xxx_top_irq_handler(int irq, void *handle)
{
	struct pm8xxx_slide *slide_handle = handle;
	int v = 0;
	printk("[Slide] top irq!!!!\n");

	v = 1 + 1*(!gpio_get_value(slide->pdata->hallic_top_detect_pin));
	wake_lock_timeout(&slide->wake_lock, msecs_to_jiffies(3000));
	queue_delayed_work(slide_wq, &slide_handle->slide_work, msecs_to_jiffies(SLIDE_DETECT_DELAY*v+5));
	return IRQ_HANDLED;
}

static irqreturn_t pm8xxx_bottom_irq_handler(int irq, void *handle)
{
	struct pm8xxx_slide *slide_handle = handle;
	int v = 0;
	printk("[Slide] bottom irq!!!!\n");

	v = 1 + 1*(!gpio_get_value(slide->pdata->hallic_bottom_detect_pin));
	wake_lock_timeout(&slide->wake_lock, msecs_to_jiffies(3000));
	queue_delayed_work(slide_wq, &slide_handle->slide_work, msecs_to_jiffies(SLIDE_DETECT_DELAY*v+5));
	return IRQ_HANDLED;
}


static ssize_t
slide_slide_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int len;
	struct pm8xxx_slide *slide = dev_get_drvdata(dev);
	len = snprintf(buf, PAGE_SIZE, "sensing(slide state) : %d\n", slide->state);

	return len;
}


static ssize_t
slide_slide_store(struct device *dev, struct device_attribute *attr,
               const char *buf, size_t count)
{
       struct pm8xxx_slide *slide = dev_get_drvdata(dev);

       sscanf(buf, "%d\n", &slide->state);
       switch_set_state(&slide->sdev, slide->state);
       return count;
}


static struct device_attribute slide_slide_attr = __ATTR(slide, S_IRUGO | S_IWUSR, slide_slide_show, slide_slide_store);


static ssize_t slide_print_name(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(sdev)) {
	case 0:
		return sprintf(buf, "OPEN\n");
	case 1:
		return sprintf(buf, "CLOSE\n");
	case 2:
		return sprintf(buf, "HALF\n");
	}
	return -EINVAL;
}

static void bu52061nvx_parse_dt(struct device *dev,
		struct pm8xxx_slide_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	if ((pdata->hallic_top_detect_pin = of_get_named_gpio_flags(np, "hallic-top-irq-gpio", 0, NULL)) > 0)
		pdata->hallic_top_irq = gpio_to_irq(pdata->hallic_top_detect_pin);

	printk("[Slide] hallic_top_gpio: %d\n", pdata->hallic_top_detect_pin);

	if ((pdata->hallic_bottom_detect_pin = of_get_named_gpio_flags(np, "hallic-bottom-irq-gpio", 0, NULL)) > 0)
		pdata->hallic_bottom_irq = gpio_to_irq(pdata->hallic_bottom_detect_pin);

	printk("[Slide] hallic_bottom_gpio: %d\n", pdata->hallic_bottom_detect_pin);

	pdata->irq_flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;
}

static int __devinit pm8xxx_slide_probe(struct platform_device *pdev)
{
	int ret;
	unsigned int hall_top_gpio_irq = 0, hall_bottom_gpio_irq =0;

	struct pm8xxx_slide_platform_data *pdata;

	if (pdev->dev.of_node) {
		pdata = devm_kzalloc(&pdev->dev,
				sizeof(struct pm8xxx_slide_platform_data),
				GFP_KERNEL);
		if (pdata == NULL) {
			pr_err("%s: [Slide] no pdata\n", __func__);
			return -ENOMEM;
		}
		pdev->dev.platform_data = pdata;

		bu52061nvx_parse_dt(&pdev->dev, pdata);

	} else {
		pdata = pdev->dev.platform_data;
	}
	if (!pdata) {
		pr_err("%s: [Slide] no pdata\n", __func__);
		return -ENOMEM;
	}

	slide = kzalloc(sizeof(*slide), GFP_KERNEL);
	if (!slide)
		return -ENOMEM;

	slide->pdata	= pdata;
	slide->sdev.name = "smartcover";
	slide->hani_sdev.name = "hallstate";
	slide->sdev.print_name = slide_print_name;
	slide->state = 0;
	slide->hall_state = 0;

	spin_lock_init(&slide->lock);

	ret = switch_dev_register(&slide->sdev);
	ret = switch_dev_register(&slide->hani_sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	wake_lock_init(&slide->wake_lock, WAKE_LOCK_SUSPEND, "hall_ic_wakeups");

	INIT_DELAYED_WORK(&slide->slide_work, pm8xxx_slide_work_func);

	printk("%s : [Slide] init slide\n", __func__);

	/* initialize irq of gpio_hall */
	if (slide->pdata->hallic_top_detect_pin > 0) {
		hall_top_gpio_irq = gpio_to_irq(slide->pdata->hallic_top_detect_pin);
		printk("%s : [Slide] hall_top_gpio_irq = [%d]\n", __func__, hall_top_gpio_irq);
		if (hall_top_gpio_irq < 0) {
			printk("Failed : [Slide] GPIO TO IRQ \n");
			ret = hall_top_gpio_irq;
			goto err_request_irq;
		}

		ret = request_irq(hall_top_gpio_irq, pm8xxx_top_irq_handler, pdata->irq_flags, HALL_IC_DEV_NAME, slide);
		if (ret > 0) {
			printk(KERN_ERR "%s: [Slide] Can't allocate irq %d, ret %d\n", __func__, hall_top_gpio_irq, ret);
			goto err_request_irq;
		}

		if (enable_irq_wake(hall_top_gpio_irq) == 0)
			printk("%s :[Slide] enable_irq_wake Enable(1)\n",__func__);
		else
			printk("%s :[Slide] enable_irq_wake failed(1)\n",__func__);
	}

	if (slide->pdata->hallic_bottom_detect_pin > 0) {
		hall_bottom_gpio_irq = gpio_to_irq(slide->pdata->hallic_bottom_detect_pin);
		printk("%s : [Slide] hall_bottom_gpio_irq = [%d]\n", __func__, hall_bottom_gpio_irq);
		if (hall_bottom_gpio_irq < 0) {
			printk("Failed : [Slide] GPIO TO IRQ \n");
			ret = hall_bottom_gpio_irq;
			goto err_request_irq;
		}

		ret = request_irq(hall_bottom_gpio_irq, pm8xxx_bottom_irq_handler, pdata->irq_flags, HALL_IC_DEV_NAME, slide);
		if (ret > 0) {
			printk(KERN_ERR "%s: [Slide] Can't allocate irq %d, ret %d\n", __func__, hall_bottom_gpio_irq, ret);
			goto err_request_irq;
		}

		if (enable_irq_wake(hall_bottom_gpio_irq) == 0)
			printk("%s :[Slide] enable_irq_wake Enable(2)\n",__func__);
		else
			printk("%s :[Slide] enable_irq_wake failed(2)\n",__func__);
	}

	printk("%s : [Slide] pdata->irq_flags = [%d]\n", __func__,(int)pdata->irq_flags);

	if (slide_boot_mode()) {
		boot_slide_factory_func();
	}else
		pm8xxx_slide_disable();

	ret = device_create_file(&pdev->dev, &slide_slide_attr);
	if (ret)
		goto err_request_irq;

	platform_set_drvdata(pdev, slide);
	return 0;

err_request_irq:
	if (hall_top_gpio_irq)
		free_irq(hall_top_gpio_irq, 0);
	if (hall_bottom_gpio_irq)
		free_irq(hall_bottom_gpio_irq, 0);

err_switch_dev_register:
	switch_dev_unregister(&slide->sdev);
	switch_dev_unregister(&slide->hani_sdev);
	kfree(slide);
	return ret;
}

static int __devexit pm8xxx_slide_remove(struct platform_device *pdev)
{
	struct pm8xxx_slide *slide = platform_get_drvdata(pdev);
	cancel_delayed_work_sync(&slide->slide_work);
	switch_dev_unregister(&slide->sdev);
	switch_dev_unregister(&slide->hani_sdev);
	platform_set_drvdata(pdev, NULL);
	kfree(slide);

	return 0;
}

static int pm8xxx_slide_suspend(struct device *dev)
{
	return 0;
}

static int pm8xxx_slide_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops pm8xxx_slide_pm_ops = {
	.suspend = pm8xxx_slide_suspend,
	.resume = pm8xxx_slide_resume,
};

#ifdef CONFIG_OF
static struct of_device_id bu52061nvx_match_table[] = {
	{ .compatible = "rohm,hall-bu52061nvx", },
	{ },
};
#endif

static struct platform_driver pm8xxx_slide_driver = {
	.probe		= pm8xxx_slide_probe,
	.remove		= __devexit_p(pm8xxx_slide_remove),
	.driver		= {
        	.name    = HALL_IC_DEV_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = bu52061nvx_match_table,
#endif
#ifdef CONFIG_PM
		.pm	= &pm8xxx_slide_pm_ops,
#endif
	},
};

static int __init pm8xxx_slide_init(void)
{
	slide_wq = create_singlethread_workqueue("slide_wq");
       printk(KERN_ERR "Slide init \n");
	if (!slide_wq)
		return -ENOMEM;
	return platform_driver_register(&pm8xxx_slide_driver);
}
module_init(pm8xxx_slide_init);

static void __exit pm8xxx_slide_exit(void)
{
	if (slide_wq)
		destroy_workqueue(slide_wq);
	platform_driver_unregister(&pm8xxx_slide_driver);
}
module_exit(pm8xxx_slide_exit);

MODULE_ALIAS("platform:" HALL_IC_DEV_NAME);
MODULE_AUTHOR("LG Electronics Inc.");
MODULE_DESCRIPTION("pm8xxx slide driver");
MODULE_LICENSE("GPL");
