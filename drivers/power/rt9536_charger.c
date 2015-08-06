/*
 * Copyright LG Electronics (c) 2013
 * All rights reserved.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/switch.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/wakelock.h>
#include <mach/board_lge.h>
#include <linux/power/rt9536_charger.h>

//static int pre_set_flag;

static struct rt9536_chip *the_chip;
//struct rt9536_platform_data *pdata;

extern int qpnp_is_cblpwr_on(void);
extern int32_t qpnp_is_pon_ready(void);



static struct workqueue_struct *local_charge_wq;
static struct workqueue_struct *local_charge_done_wq;


static int wlc_is_connected(void)
{
	return qpnp_is_cblpwr_on();
}
static int wlc_irq_state = 2;
static void wlc_enable_irq(struct rt9536_chip *rt9536_chg, int on)
{
	if(on == wlc_irq_state)
	{
		printk("[RT9536] on(%d) and wlc_irq_state(%d) is same!!!\n",on,wlc_irq_state);
		return;
	}
	if(on)
	{
		enable_irq(rt9536_chg->pdata->wlc_irq);
		printk("[RT9536] enable IRQ !!!\n");
		wlc_irq_state=1;
	}
	else
	{
		disable_irq(rt9536_chg->pdata->wlc_irq);
		printk("[RT9536] disable IRQ !!!\n");
		wlc_irq_state=0;
	}
	return;
}
static int rt9536_get_status(struct rt9536_chip *chip)
{
	int ret=POWER_SUPPLY_STATUS_UNKNOWN;
//	struct rt9536_chip *chip = container_of(psy, struct rt9536_chip, charger);
	if (wlc_is_connected())
	{
		if(chip->chg_done)
		{
			ret = POWER_SUPPLY_STATUS_FULL;
		}
		else
		{
			ret = POWER_SUPPLY_STATUS_CHARGING;
		}
	}
	else
	{
		ret = POWER_SUPPLY_STATUS_DISCHARGING;
	}
	return ret;
}


static void rt9536_make_pulse(struct rt9536_chip *chip, int count)
{
		int i = 0;
		for(i=0;i<count;i++)
		{
			gpio_set_value(chip->pdata->wlc_en_set_pin, 1);
			//usleep_range(200, 500);
			udelay(200);
			gpio_set_value(chip->pdata->wlc_en_set_pin, 0);
			//usleep_range(200, 500);
			udelay(200);
		}
		return;
}


static int rt9536_set_charger_enable(struct rt9536_chip *chip, int on)
{
//	printk("[RT9536] %s: setting value is %d\n",__func__,on);

	if (!gpio_is_valid(chip->pdata->wlc_en_set_pin)) {
		pr_err("wlc_en_set_pin gpio_request failed for %d\n", chip->pdata->wlc_en_set_pin);
		return -1;
	}	

	if(on){
		gpio_set_value(chip->pdata->wlc_en_set_pin, 0);
		udelay(100);
#if 0		
		gpio_set_value(chip->pdata->wlc_en_set_pin, 1);
		usleep_range(200, 500);
		gpio_set_value(chip->pdata->wlc_en_set_pin, 0);
#endif
		rt9536_make_pulse(chip,1);
		mdelay(3);
		printk("[RT9536] %s: WLC_EN(gpio #%d) makes just one pulse to set ISET\n", __func__, the_chip->pdata->wlc_en_set_pin);

	}else{
		printk("[RT9536] %s: WLC_EN(gpio #%d) set to HIGH for disable cherger\n", __func__, the_chip->pdata->wlc_en_set_pin);
		gpio_set_value(chip->pdata->wlc_en_set_pin, 1);
		mdelay(4);

	}

	
	return 0;
}


static int rt9536_set_cc_mode_4_35V(struct rt9536_chip *chip)
{
//	pr_err("[RT9536] %s: enter!!!\n",__func__);
	if (!gpio_is_valid(chip->pdata->wlc_en_set_pin)) {
		pr_err("wlc_en_set_pin gpio_request failed for %d\n", chip->pdata->wlc_en_set_pin);
		return -1;
	}
	gpio_set_value(chip->pdata->wlc_en_set_pin, 0);
	mdelay(3);
	gpio_set_value(chip->pdata->wlc_en_set_pin, 1);
	udelay(800);
	gpio_set_value(chip->pdata->wlc_en_set_pin, 0);

	mdelay(4);
	return 0;
}

static int wlc_check_irq_pin(struct rt9536_chip *chip)
{
	int ret=0;
	
	ret = gpio_get_value(chip->pdata->wlc_irq_pin);
//	printk("[RT9536] %s: WLC_IRQ_PIN is %d",__func__,ret);
	return ret;
}
static void wlc_set_enable_and_4_35V(struct rt9536_chip *rt9536_chg)
{
	unsigned long flags;
	wake_lock(&rt9536_chg->wl);
//	rt9536_set_charger_enable(1);  // temp
	spin_lock_irqsave(&rt9536_chg->enable_lock,flags);
	rt9536_chg->chg_online=1;
	rt9536_chg->chg_done=0;
	if(rt9536_set_charger_enable(rt9536_chg,0))
	{
		pr_err("%s : charger disable fail!!!\n", __func__);
	}	
	mdelay(1);
	if(rt9536_set_charger_enable(rt9536_chg,1))
	{
		pr_err("%s : charger enable fail!!!\n", __func__);
	}
//	msleep(3);
	spin_unlock_irqrestore(&rt9536_chg->enable_lock,flags);
	if(rt9536_set_cc_mode_4_35V(rt9536_chg))
	{
		pr_err("%s : CC mode setting fail!!!\n", __func__);
	}
	
	power_supply_changed(&rt9536_chg->charger);
	//enable_irq(rt9536_chg->pdata->wlc_irq);
	wlc_enable_irq(rt9536_chg,1);
	msleep(100);
	if(wlc_check_irq_pin(rt9536_chg)==1)
	{
		schedule_work(&rt9536_chg->wireless_eoc_work);
	}

}

static void wlc_set_disable(struct rt9536_chip *rt9536_chg)
{
	//	if(rt9536_chg->chg_done==0)
//		disable_irq(rt9536_chg->pdata->wlc_irq);
		wlc_enable_irq(rt9536_chg,0);
	//	rt9536_chg->chg_done=0;
	//	rt9536_set_charger_enable(1);  // temp
		rt9536_chg->chg_online=0;
		if(rt9536_set_charger_enable(rt9536_chg,0))
		{
			pr_err("%s : charger enable fail!!!\n", __func__);
		}
		power_supply_changed(&rt9536_chg->charger);
		if(wake_lock_active(&rt9536_chg->wl))
		{
			wake_unlock(&rt9536_chg->wl);
			wake_lock_timeout(&rt9536_chg->wl,1000);
		}
	
}

static void wlc_set_online_work(struct work_struct *work)
{
	
	struct rt9536_chip *rt9536_chg = container_of(work,
		struct rt9536_chip,wireless_set_online_work);
	printk("[RT9536] %s \n",__func__);
	if(rt9536_chg->pdata->wlc_enable==0)
	{
		pr_err("[RT9536] %s WLC is disable. \n",__func__);
		return;
	}
	wlc_set_enable_and_4_35V(rt9536_chg);
	return;
}

static void wlc_set_offline_work(struct work_struct *work)
{
	struct rt9536_chip *rt9536_chg = container_of(work,
		struct rt9536_chip,wireless_set_offline_work);
	printk("[RT9536] %s \n",__func__);
	if(rt9536_chg->pdata->wlc_enable==0)
	{
		pr_err("[RT9536] %s WLC is disable. \n",__func__);
		return;
	}	
	wlc_set_disable(rt9536_chg);
	return;
}

static void wlc_eoc_work(struct work_struct *work)
{
	struct rt9536_chip *rt9536_chg = container_of(work, struct rt9536_chip,
				 wireless_eoc_work);

	pr_err("[RT9536] %s \n",__func__);
	rt9536_chg->chg_done=1;
	if(rt9536_set_charger_enable(rt9536_chg,0))
	{
		pr_err("%s : charger enable fail!!!\n", __func__);
	}
	power_supply_changed(&rt9536_chg->charger);
	if(wake_lock_active(&rt9536_chg->wl))
		wake_unlock(&rt9536_chg->wl);
	

#if 0
	int chg_state=0;
	int wlc_connect_state=0;
	struct rt9536_chip *rt9536_chg = container_of(work, struct rt9536_chip,
			 wireless_eoc_work.work);
	pr_err("[RT9536] %s \n",__func__);	

	chg_state = wlc_check_irq_pin(rt9536_chg);
	wlc_connect_state = wlc_is_connected();
	if(chg_state)
	{
		if(wlc_connect_state==1)
		{
			power_supply_changed(&rt9536_chg->charger);
			rt9536_chg->chg_done=1;
		}
		else
		{
			schedule_delayed_work(&rt9536_chg->wireless_set_offline_work,
				round_jiffies_relative(msecs_to_jiffies(500)));
		}
	}
	else
	{
		schedule_delayed_work(&rt9536_chg->wireless_set_online_work,
			round_jiffies_relative(msecs_to_jiffies(500)));
	}

//	msleep(3500);
//	gpio_set_value(rt9536_chg->pdata->wlc_chg_full_pin,0);
//	rt9536_chg->chg_done=1;
//	rt9536_set_charger_enable(0);  // temp
#endif
	return;
}

static void wlc_interrupt_worker(struct work_struct *work)
{
	int chg_state=0;
	int wlc_connect_state=0;
	struct rt9536_chip *rt9536_chg = container_of(work, struct rt9536_chip,
			 wireless_interrupt_work);
	printk("[RT9536] %s \n",__func__);
//	disable_irq(rt9536_chg->pdata->wlc_irq);
	msleep(100);
	chg_state = wlc_check_irq_pin(rt9536_chg);
	printk("[RT9536] %s chg_state is %d \n",__func__,chg_state);
//	msleep(100);
	wlc_connect_state = wlc_is_connected();
	printk("[RT9536] %s wlc_connect_state is %d \n",__func__,wlc_connect_state);
	if(chg_state)
	{
		if(wlc_connect_state==1)
		{
			schedule_work(&rt9536_chg->wireless_eoc_work);
			return;
		}
		else
		{
		//	schedule_delayed_work(&the_chip->wireless_set_offline_work,
		//		round_jiffies_relative(msecs_to_jiffies(500)));
		}	
	}
	else
	{
		//schedule_delayed_work(&the_chip->wireless_set_online_work,
		//	round_jiffies_relative(msecs_to_jiffies(500)));
	}
//	enable_irq(rt9536_chg->pdata->wlc_irq);
	return;
}

static irqreturn_t wlc_irq_handler(int irq, void *handle)
{
//	struct pm8xxx_cradle *cradle_handle = handle;
	int chg_state;
	struct rt9536_chip *chip = handle;
	printk("WLC irq!!!!\n");
	chg_state = wlc_check_irq_pin(chip);
	if(chg_state)
	{
		printk("[RT9536] %s: WLC_IRQ_PIN is HIGH",__func__);
	}
	else
	{
		printk("[RT9536] %s: WLC_IRQ_PIN is LOW",__func__);
	}
	schedule_work(&chip->wireless_interrupt_work);
//	queue_delayed_work(local_charge_wq, &chip->charge_work, msecs_to_jiffies(200));
	return IRQ_HANDLED;
}

static int set_wlc_chg_enable(struct rt9536_chip *chip, int enable)
{
	int ret = 0;
	if(wlc_is_connected())
	{
		if(enable)
		{
			wlc_set_enable_and_4_35V(chip);
			
		}
		else
		{
			wlc_set_disable(chip);
		}
	}
	chip->pdata->wlc_enable= enable;
	return ret;
}

static int rt9536_charger_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_WLC_ENABLE:
		return 1;
	default:
		break;
	}

	return 0;
}

static int rt9536_charger_set_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	int ret = 0;
	struct rt9536_chip *chip = container_of(psy, struct rt9536_chip, charger);

	switch (psp) {
	case POWER_SUPPLY_PROP_WLC_ENABLE:
		if(chip->pdata->wlc_enable!=val->intval)
			ret = set_wlc_chg_enable(chip, val->intval);
		break;
	default:
		return -EINVAL;
	}
	if (ret < 0)
		return -EINVAL;
	
    pr_err("[LGE]  wlc_enable[%d]\n", chip->pdata->wlc_enable);
	power_supply_changed(&chip->charger);
	
	return 0;
}

static int rt9536_charger_get_property(struct power_supply *psy,
                                        enum power_supply_property psp,
                                        union power_supply_propval *val)
{
	
	struct rt9536_chip *chip = container_of(psy, struct rt9536_chip, charger);
	int ret = 0;
//	int chg_status = 0;

	//printk("@@@@ Inside bq24160_charger_get_property,chip->chg_online:%d,psp:%d @@@@\n",chip->chg_online,psp);
	
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_PRESENT:	
		val->intval = chip->chg_online;
		//printk("[RT9536] %s : power_supply online = %d\n", __func__,chip->chg_online);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = rt9536_get_status(chip);
//		pr_err("[RT9536] %s : POWER_SUPPLY_PROP_STATUS is %d\n",__func__,val->intval);

		break;	
	case POWER_SUPPLY_PROP_WLC_ENABLE:
		val->intval = chip->pdata->wlc_enable;
		break;
    default:
		ret = -ENODEV;
		break;
	}
	//printk("@@@@ bq24160_charger_get_property,val->intval:%d,ret:%d @@@@\n",val->intval,ret);
	return ret;
}


static enum power_supply_property rt9536_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_WLC_ENABLE,
};

static char *pm_power_supplied_to[] = {
	"battery",
};

static struct power_supply rt9536_charger_ps = {
	/*.name = "wireless-charger",*/
	.name = "wireless",
	.type = POWER_SUPPLY_TYPE_WIRELESS/*POWER_SUPPLY_TYPE_MAINS*/,
	.supplied_to = pm_power_supplied_to,
	.num_supplicants = ARRAY_SIZE(pm_power_supplied_to),
	.properties = rt9536_charger_props,
	.num_properties = ARRAY_SIZE(rt9536_charger_props),
	.get_property = rt9536_charger_get_property,
	.set_property = rt9536_charger_set_property,
	.property_is_writeable = rt9536_charger_property_is_writeable,
};

static void rt9536_parse_dt(struct device *dev,
		struct rt9536_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	int rc = 0;
	if ((pdata->wlc_irq_pin = of_get_named_gpio_flags(np, "rt9536,wlc_irq_n", 0, NULL)) > 0)
	{
		pdata->wlc_irq= gpio_to_irq(pdata->wlc_irq_pin);
	}
	

	printk("[RT9536] wlc_irq_pin: %d\n", pdata->wlc_irq_pin);

	pdata->irq_flags = IRQF_TRIGGER_RISING;
//	pdata->irq_flags = IRQF_TRIGGER_FALLING;

	pdata->wlc_en_set_pin = of_get_named_gpio(np,"rt9536,chg_en_set",0);
	printk("[RT9536] wlc_en_set_pin: %d\n", pdata->wlc_en_set_pin);
#if !defined(CONFIG_MACH_MSM8926_JAGNM_ATT) && !defined(CONFIG_MACH_MSM8926_JAGNM_RGS) && !defined(CONFIG_MACH_MSM8926_JAGNM_TLS) \
	&& !defined(CONFIG_MACH_MSM8926_JAGNM_VTR) && !defined(CONFIG_MACH_MSM8926_JAGNM_BELL) && !defined(CONFIG_MACH_MSM8926_JAGC_SPR)
	pdata->wlc_chg_full_pin = of_get_named_gpio(np,"rt9536,wlc_chg_full",0);
	printk("[RT9536] wlc_chg_full_pin: %d\n", pdata->wlc_chg_full_pin);
#endif
	rc = of_property_read_u32(np, "rt9536,wlc_enable",&pdata->wlc_enable);
	printk("[RT9536] wlc_enable: %d\n", pdata->wlc_enable);
	
}


static __devinit int rt9536_probe(struct platform_device *pdev)
{

	struct rt9536_chip *chip;
	int ret = 0;

	printk("@@@@ Inside rt9536_probe @@@@\n");


	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

//	INIT_DELAYED_WORK(&chip->charge_work, rt9536_charge);
//	INIT_DELAYED_WORK(&chip->charge_done_work, rt9536_charge_done);

	
	//chip->pdata = client->dev.platform_data;
	wake_lock_init(&chip->wl, WAKE_LOCK_SUSPEND, "rt9536");
	spin_lock_init(&chip->enable_lock);
	ret=qpnp_is_pon_ready();

	if(ret)
		goto out;

	if (pdev->dev.of_node) {
		chip->pdata = devm_kzalloc(&pdev->dev,
				sizeof(struct rt9536_platform_data),
				GFP_KERNEL);
		if (chip->pdata == NULL) {
			pr_err("%s: no pdata\n", __func__);
			return -ENOMEM;
		}
		pdev->dev.platform_data = chip->pdata;
		rt9536_parse_dt(&pdev->dev, chip->pdata);
	


		if (!gpio_is_valid(chip->pdata->wlc_en_set_pin))
		{
			pr_err("Fail to get named gpio for wlc_en_set_pin.\n");
			goto err_qpio_request;
		}
		else
		{
			ret = gpio_request(chip->pdata->wlc_en_set_pin, "wlc_en_set_pin");
			
			if(ret) {
				pr_err("request wlc_en_set_pin gpio failed, ret=%d\n", ret);
				//gpio_free(chip->pdata->wlc_en_set_pin);
				goto err_qpio_request;
			}
			gpio_tlmm_config(GPIO_CFG(chip->pdata->wlc_en_set_pin, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA), GPIO_CFG_ENABLE);
		}
			/* initialize irq of gpio_hall */
		if (chip->pdata->wlc_irq_pin > 0) {
			ret = gpio_request(chip->pdata->wlc_irq_pin,
						"wlc_irq");
			if(ret)
			{
				pr_err("%s :gpio request fail for wlc_irq and ret is %d\n",__func__,ret);
				gpio_free(chip->pdata->wlc_irq_pin);
				goto err_request_irq;
			}
			ret = gpio_direction_input(chip->pdata->wlc_irq_pin);
			gpio_tlmm_config(GPIO_CFG(chip->pdata->wlc_irq_pin, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_4MA), GPIO_CFG_ENABLE);
				chip->chg_online=gpio_get_value(chip->pdata->wlc_irq_pin);
				printk("[RT9536] %s: chg_online is %d\n",__func__,chip->chg_online);
			
			if(ret)
			{
				pr_err("%s :gpio_direction_input fail for wlc_irq and ret is %d\n",__func__,ret);
				gpio_free(chip->pdata->wlc_irq_pin);
				goto err_request_irq;
			}

			ret = request_irq(chip->pdata->wlc_irq, wlc_irq_handler, chip->pdata->irq_flags, "rt9536", chip);
			if (ret) {
				pr_err("%s :request_irq and ret is %d\n",__func__,ret);
				gpio_free(chip->pdata->wlc_irq_pin);
				goto err_request_irq;
			}

#if 0
			if (enable_irq_wake(chip->pdata->wlc_irq) == 0)
			{
				pr_err("%s :enable_irq_wake Enable(1)\n",__func__);
			}
			else
			{
				pr_err("%s :enable_irq_wake failed(1)\n",__func__);
				if (chip->pdata->wlc_irq)
					free_irq(chip->pdata->wlc_irq, 0);
				goto out;
			}
#endif			
		}
	
	}

//	i2c_set_clientdata(client, chip);

	chip->charger = rt9536_charger_ps;
#if 1
	ret = power_supply_register(&pdev->dev, &chip->charger);
	if (ret) {
		dev_err(&pdev->dev, "failed: power supply register\n");
	//	i2c_set_clientdata(client, NULL);
		goto out;
	}
#endif
	chip->chg_done=0;
	chip->chg_online=0;

	INIT_WORK(&chip->wireless_interrupt_work, wlc_interrupt_worker);
	INIT_WORK(&chip->wireless_set_online_work,wlc_set_online_work);
	INIT_WORK(&chip->wireless_set_offline_work,wlc_set_offline_work);
	INIT_WORK(&chip->wireless_eoc_work,wlc_eoc_work);


#ifndef BIGLAKE_TEST
		printk("[RT9536] wlc_is_connected is %d\n",wlc_is_connected());
#endif
	

	printk("%s : chip->pdata->irq_flags = [%d]\n", __func__,(int)chip->pdata->irq_flags);



	the_chip = chip;
#if 0 
	disable_irq(chip->pdata->wlc_irq);
#endif
//	disable_irq(chip->pdata->wlc_irq);
	wlc_enable_irq(chip,0);
	if(rt9536_set_charger_enable(chip,0))
	{
		pr_err("%s : charger disable fail!!!\n", __func__);
	}	
	msleep(10);
	if(rt9536_set_charger_enable(chip,1))
	{
		pr_err("%s : charger enable fail!!!\n", __func__);
	}
	//msleep(3);
	if(rt9536_set_cc_mode_4_35V(chip))
	{
		pr_err("%s : CC mode setting fail!!!\n", __func__);
	}
//	schedule_work(&chip->wireless_interrupt_work);
	msleep(100);
	//enable_irq(chip->pdata->wlc_irq);
	wlc_enable_irq(chip,1);
	if(wlc_is_connected())
	{
	//		disable_irq(chip->pdata->wlc_irq);
		if(wlc_check_irq_pin(chip)==0)
		{
			// enable_irq(chip->pdata->wlc_irq);
			wake_lock(&chip->wl);
			chip->chg_online=1;
			power_supply_changed(&chip->charger);
			printk("[RT9536] WLC is connected\n");
		}
		else
		{
			schedule_work(&chip->wireless_eoc_work);
		}
	}
	else
	{
		schedule_work(&chip->wireless_set_offline_work);
		printk("[RT9536] WLC is disconnected\n");
	}

//	disable_irq(chip->pdata->wlc_irq);

	
#if 0
	__set_charger(chip, 1);	//just for test


#ifdef BQ24160_CABLE_PWRON_INT
	ret = request_threaded_irq(client->irq, NULL, bq24160_valid_handler,
		IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, client->name, chip);
#else
	ret = request_threaded_irq(client->irq, NULL, bq24160_charger_handler,
		IRQF_ONESHOT | IRQF_TRIGGER_LOW, client->name, chip);
#endif

	if (unlikely(ret < 0))
	{
		dev_err(&client->dev, "%s failed to request IRQ %d\n",__func__, ret);
		goto out;
	}
	irq_set_irq_wake(client->irq, 1);
#endif


	printk("[RT9536] rt9536_probe done\n");
	return 0;


err_request_irq:
	if (chip->pdata->wlc_irq)
		free_irq(chip->pdata->wlc_irq, 0);

err_qpio_request:
	 gpio_free(chip->pdata->wlc_en_set_pin);

out:

	wake_lock_destroy(&chip->wl);
	kfree(chip);
	the_chip=NULL;
	return ret;
	
}






static int __devexit rt9536_remove(struct platform_device *pdev)
{
	struct rt9536_chip *chip = platform_get_drvdata(pdev);
	power_supply_unregister(&rt9536_charger_ps);
	
	wake_lock_destroy(&chip->wl);

	kfree(chip);


	return 0;
}

static int rt9536_suspend(struct device *dev)
{
	return 0;
}

static int rt9536_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops rt9536_pm_ops = {
	.suspend = rt9536_suspend,
	.resume = rt9536_resume,
};

#ifdef CONFIG_OF
static struct of_device_id rt9536_match_table[] = {
	{ .compatible = "richtek,rt9536", },
	{ },
};
#endif

static struct platform_driver rt9536_driver = {
	.probe		= rt9536_probe,
	.remove		= __devexit_p(rt9536_remove),
	.driver		= {
        .name    = "rt9536",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = rt9536_match_table,
#endif
#ifdef CONFIG_PM
		.pm	= &rt9536_pm_ops,
#endif
	},
};

static int __init rt9536_init(void)
{
	local_charge_wq = create_workqueue("rt9536_charge_work");
	if(!local_charge_wq)
		return -ENOMEM;
	
	local_charge_done_wq = create_workqueue("rt9536_charge_done_work");
	if(!local_charge_done_wq)
		return -ENOMEM;

	return platform_driver_register(&rt9536_driver);
}
module_init(rt9536_init);

static void __exit rt9536_exit(void)
{
	if(local_charge_wq)
		destroy_workqueue(local_charge_wq);
	local_charge_wq = NULL;

	if(local_charge_done_wq)
		destroy_workqueue(local_charge_done_wq);
	local_charge_done_wq = NULL;

	platform_driver_unregister(&rt9536_driver);
}
module_exit(rt9536_exit);

MODULE_AUTHOR("Daeho Choi <daeho.choi@lge.com>");
MODULE_DESCRIPTION("RT9536 charger");
MODULE_LICENSE("GPL");
