/* Copywrite(c) 2011-2012, LGE. All rights reserved.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */
#ifdef CONFIG_LGE_ENABLE_MMC_STRENGTH_CONTROL
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/module.h>

#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <asm/mach/mmc.h>

#include "../../../../drivers/mmc/core/core.h"
#include "../../../../drivers/mmc/core/mmc_ops.h"
#include "../../../../drivers/mmc/host/sdhci-pltfm.h"

/* This structure keeps information per regulator */
struct sdhci_msm_reg_data {
	/* voltage regulator handle */
	struct regulator *reg;
	/* regulator name */
	const char *name;
	/* voltage level to be set */
	u32 low_vol_level;
	u32 high_vol_level;
	/* Load values for low power and high power mode */
	u32 lpm_uA;
	u32 hpm_uA;

	/* is this regulator enabled? */
	bool is_enabled;
	/* is this regulator needs to be always on? */
	bool is_always_on;
	/* is low power mode setting required for this regulator? */
	bool lpm_sup;
};

/*
 * This structure keeps information for all the
 * regulators required for a SDCC slot.
 */
struct sdhci_msm_slot_reg_data {
	/* keeps VDD/VCC regulator info */
	struct sdhci_msm_reg_data *vdd_data;
	 /* keeps VDD IO regulator info */
	struct sdhci_msm_reg_data *vdd_io_data;
};

struct sdhci_msm_gpio {
	u32 no;
	const char *name;
	bool is_enabled;
};

struct sdhci_msm_gpio_data {
	struct sdhci_msm_gpio *gpio;
	u8 size;
};

struct sdhci_msm_pad_pull {
	enum msm_tlmm_pull_tgt no;
	u32 val;
};

struct sdhci_msm_pad_pull_data {
	struct sdhci_msm_pad_pull *on;
	struct sdhci_msm_pad_pull *off;
	u8 size;
};

struct sdhci_msm_pad_drv {
	enum msm_tlmm_hdrive_tgt no;
	u32 val;
};

struct sdhci_msm_pad_drv_data {
	struct sdhci_msm_pad_drv *on;
	struct sdhci_msm_pad_drv *off;
	u8 size;
};

struct sdhci_msm_pad_data {
	struct sdhci_msm_pad_pull_data *pull;
	struct sdhci_msm_pad_drv_data *drv;
};


struct sdhci_msm_pin_data {
	/*
	 * = 1 if controller pins are using gpios
	 * = 0 if controller has dedicated MSM pads
	 */
	u8 is_gpio;
	bool cfg_sts;
	struct sdhci_msm_gpio_data *gpio_data;
	struct sdhci_msm_pad_data *pad_data;
};

struct sdhci_msm_bus_voting_data {
	struct msm_bus_scale_pdata *bus_pdata;
	unsigned int *bw_vecs;
	unsigned int bw_vecs_size;
};

struct sdhci_msm_pltfm_data {
	/* Supported UHS-I Modes */
	u32 caps;

	/* More capabilities */
	u32 caps2;

	unsigned long mmc_bus_width;
	struct sdhci_msm_slot_reg_data *vreg_data;
	bool nonremovable;
	struct sdhci_msm_pin_data *pin_data;
	u32 cpu_dma_latency_us;
	int status_gpio; /* card detection GPIO that is configured as IRQ */
	struct sdhci_msm_bus_voting_data *voting_data;
	u32 *sup_clk_table;
	unsigned char sup_clk_cnt;
};

struct sdhci_msm_bus_vote {
	uint32_t client_handle;
	uint32_t curr_vote;
	int min_bw_vote;
	int max_bw_vote;
	bool is_max_bw_needed;
	struct delayed_work vote_work;
	struct device_attribute max_bus_bw;
};

struct sdhci_msm_host {
	struct platform_device	*pdev;
	void __iomem *core_mem;    /* MSM SDCC mapped address */
	int	pwr_irq;	/* power irq */
	struct clk	 *clk;     /* main SD/MMC bus clock */
	struct clk	 *pclk;    /* SDHC peripheral bus clock */
	struct clk	 *bus_clk; /* SDHC bus voter clock */
	struct clk	 *ff_clk; /* CDC calibration fixed feedback clock */
	struct clk	 *sleep_clk; /* CDC calibration sleep clock */
	atomic_t clks_on; /* Set if clocks are enabled */
	struct sdhci_msm_pltfm_data *pdata;
	struct mmc_host  *mmc;
	struct sdhci_pltfm_data sdhci_msm_pdata;
	u32 curr_pwr_state;
	u32 curr_io_level;
	struct completion pwr_irq_completion;
	struct sdhci_msm_bus_vote msm_bus_vote;
	struct device_attribute	polling;
	u32 clk_rate; /* Keeps track of current clock rate that is set */
	bool tuning_done;
	bool calibration_done;
	u8 saved_tuning_phase;
};

enum vdd_io_level {
	/* set vdd_io_data->low_vol_level */
	VDD_IO_LOW,
	/* set vdd_io_data->high_vol_level */
	VDD_IO_HIGH,
	/*
	 * set whatever there in voltage_level (third argument) of
	 * sdhci_msm_set_vdd_io_vol() function.
	 */
	VDD_IO_SET_LEVEL,
};

extern struct sdhci_msm_host *mmc_control_mmchost;
extern unsigned int clock_max;
extern char clock_flag;
extern unsigned int clock_show ;
extern char voltage_flag;
	
char received_set_value[15];
char real_value[3];

typedef struct {
	unsigned char clk;
	unsigned char cmd;
	unsigned char data ;
		
}mmc_control_data;
static unsigned int clock_setting_value=0;
static unsigned int voltage_setting_value=0;



static int mmc_strength_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct sdhci_msm_host *mmc = mmc_control_mmchost;
	int i;
	char str[2];
	
	if(mmc ==NULL)
		return -22;
	
	memset(received_set_value,0x00,sizeof(received_set_value));
	memset(str,0x00,sizeof(str));

	
	for (i = 0; i <mmc->pdata->pin_data->pad_data->drv->size; i++) {
		
		switch(mmc->pdata->pin_data->pad_data->drv->on[i].val)
		{
			case GPIO_CFG_2MA:
				str[0] = '0';
				str[1] = '2';
				break;

			case GPIO_CFG_4MA:
				str[0] = '0';
				str[1] = '4';
				break;

			case GPIO_CFG_6MA:
				str[0] = '0';
				str[1] = '6';
				break;

			case GPIO_CFG_8MA:
				str[0] = '0';
				str[1] = '8';
				break;

			case GPIO_CFG_10MA:
				str[0] = '1';
				str[1] = '0';
				break;

			case GPIO_CFG_12MA:
				str[0] = '1';
				str[1] = '2';
				break;

			case GPIO_CFG_14MA:
				str[0] = '1';
				str[1] = '4';
				break;

			case GPIO_CFG_16MA:
				str[0] = '1';
				str[1] = '6';
				break;
			}
		if(mmc->pdata->pin_data->pad_data->drv->on[i].no == TLMM_HDRV_SDC2_CLK)
		{	
			received_set_value[0] = str[0];
			received_set_value[1] = str[1];
		}

		else if(mmc->pdata->pin_data->pad_data->drv->on[i].no==TLMM_HDRV_SDC2_CMD)
		{	
			received_set_value[3] = str[0];
			received_set_value[4] = str[1];
		}

		else if(mmc->pdata->pin_data->pad_data->drv->on[i].no == TLMM_HDRV_SDC2_DATA)
		{	
			received_set_value[6] = str[0];
			received_set_value[7] = str[1];
		}
	}

	received_set_value[2] = ';';
	received_set_value[5] = ';';
	received_set_value[8] = '\0';
	printk("%s : test test .\n", __func__);
	return sprintf(buf, "%s\n", received_set_value);
}

static int mmc_strength_store(struct device *dev, struct device_attribute *attr, const char * buf, size_t count)
{
	
	struct sdhci_msm_host *mmc = mmc_control_mmchost;
	int i;
	
	static mmc_control_data mmc_set_data;
	
	printk("%s : test mmc_strength_store count=%d .\n", __func__,count);
	
	if(count !=6)
	{
		printk("%s : input data failed !!!\n", __func__);
	   	return -22;
	}
	
	if(mmc ==NULL)
		return -22;
		
		
	memcpy(received_set_value, buf, count);
	
	mmc_set_data.clk=(received_set_value[0]-0x30);
	mmc_set_data.cmd=(received_set_value[2]-0x30);
	mmc_set_data.data=(received_set_value[4]-0x30);
	
	
	
	for (i = 0; i <mmc->pdata->pin_data->pad_data->drv->size; i++) {

				if(mmc->pdata->pin_data->pad_data->drv->on[i].no == TLMM_HDRV_SDC2_CLK)
				{	
					mmc->pdata->pin_data->pad_data->drv->on[i].val =(mmc_set_data.clk - 1);
					msm_tlmm_set_hdrive(mmc->pdata->pin_data->pad_data->drv->on[i].no,mmc->pdata->pin_data->pad_data->drv->on[i].val );
				}

				else if(mmc->pdata->pin_data->pad_data->drv->on[i].no==TLMM_HDRV_SDC2_CMD)
				{	
					mmc->pdata->pin_data->pad_data->drv->on[i].val =(mmc_set_data.cmd - 1);
					msm_tlmm_set_hdrive(mmc->pdata->pin_data->pad_data->drv->on[i].no,mmc->pdata->pin_data->pad_data->drv->on[i].val );
				}

				else if(mmc->pdata->pin_data->pad_data->drv->on[i].no == TLMM_HDRV_SDC2_DATA)
				{	
					mmc->pdata->pin_data->pad_data->drv->on[i].val =(mmc_set_data.data - 1);
					msm_tlmm_set_hdrive(mmc->pdata->pin_data->pad_data->drv->on[i].no,mmc->pdata->pin_data->pad_data->drv->on[i].val );
				}
			}
		
	printk("%s : clk=%d\n", __func__,  mmc_set_data.clk);
	printk("%s : cmd=%d\n", __func__,  mmc_set_data.cmd);
	printk("%s : data=%d\n", __func__,  mmc_set_data.data);
	
	return count;
}


DEVICE_ATTR(mmc_strength, 0777, mmc_strength_show, mmc_strength_store);





////////////////////////////////clock/////////////////////////////////////////////////////////////////
static int mmc_clock_setting_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	clock_setting_value = clock_show/(1000*1000);
	
	printk("%s : test test .\n", __func__);
	return sprintf(buf, "%d", clock_setting_value);
	
}
static int mmc_clock_setting_store(struct device *dev, struct device_attribute *attr, const char * buf, size_t count)
{
	struct sdhci_msm_host *sdhci_mmc = mmc_control_mmchost;
	struct mmc_host *mmc = sdhci_mmc->mmc;

	
	if(mmc ==NULL)
		return -22;


		
	printk("%s : test mmc_clock_setting_store count=%d .\n", __func__,count);
	memcpy(received_set_value, buf, count);
		
	if(count ==2)
	{
	clock_setting_value = ((received_set_value[0]-0x30) *10); 
	clock_setting_value += (received_set_value[1]-0x30) ; 
	
	}
	else if (count == 3)
	{
	
	clock_setting_value = ((received_set_value[0]-0x30) *100); 
	clock_setting_value += ((received_set_value[1]-0x30) *10); 
	clock_setting_value += (received_set_value[2]-0x30) ; 	
	}
	else
	{
	printk("%s : input data failed !!!\n", __func__);	
	return -22;
	}
	
	clock_setting_value = (clock_setting_value * 1000 * 1000);
	

	mmc_host_clk_hold(mmc);
	mmc_set_clock(mmc, clock_setting_value);
	mmc_host_clk_release(mmc);

	
	clock_max = clock_setting_value;
	clock_flag = 1;
	clock_show = clock_setting_value;

	printk("%s : result of clock value =%d\n", __func__,clock_setting_value);	
	return count;
}

DEVICE_ATTR(mmc_clock_setting, 0777, mmc_clock_setting_show, mmc_clock_setting_store);



////////////////////////////////voltage /////////////////////////////////////////////////////////////////

static int mmc_voltage_setting_store(struct device *dev, struct device_attribute *attr, const char * buf, size_t count)
{
	struct sdhci_msm_host *sdhci_mmc = mmc_control_mmchost;
	struct mmc_host *mmc = sdhci_mmc->mmc;
	struct mmc_ios	*ios = &mmc->ios;
	unsigned int previous_voltage_value;

	
	if(mmc ==NULL)
		return -22;
		

		
		printk("%s : test mmc_voltage_setting_store count=%d .\n", __func__,count);
		memcpy(received_set_value, buf, count);
		
		previous_voltage_value = voltage_setting_value;
	
		if (count == 3)
		{
		voltage_setting_value = ((received_set_value[0]-0x30) *100); 
		voltage_setting_value +=( (received_set_value[1]-0x30)*10); 
		voltage_setting_value += (received_set_value[2]-0x30);	
		}
		else
		{
		printk("%s : input data failed !!!\n", __func__);
		
		return -22;
		}
		 
		if(voltage_setting_value%5 || voltage_setting_value < 185 || voltage_setting_value > 295)
		{
			voltage_setting_value = previous_voltage_value ;
			return -22;
		}

		switch(ios->signal_voltage)
		{
			case MMC_SIGNAL_VOLTAGE_330:
				sdhci_mmc->pdata->vreg_data->vdd_io_data->high_vol_level = voltage_setting_value * 10000;
				break;

			case MMC_SIGNAL_VOLTAGE_180:
				sdhci_mmc->pdata->vreg_data->vdd_io_data->low_vol_level = voltage_setting_value * 10000;
				break;
				
			default:
				return -22;
		}
		
		//msmsdcc_set_vdd_io_vol(host,VDD_IO_SET_LEVEL,(voltage_setting_value * 10000));

		

		printk("%s : result of clock value =%d\n", __func__,voltage_setting_value);	
		return count;
}


static int mmc_voltage_setting_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	
	struct sdhci_msm_host *sdhci_mmc = mmc_control_mmchost;
	struct mmc_host *mmc = sdhci_mmc->mmc;
	struct mmc_ios	*ios = &mmc->ios;
	
	if(mmc ==NULL)
		return -22;


	if(voltage_flag ==0)
	{
		switch(ios->signal_voltage)
		{
			case MMC_SIGNAL_VOLTAGE_330:
				voltage_setting_value = sdhci_mmc->pdata->vreg_data->vdd_io_data->high_vol_level/10000 ;
				break;

			case MMC_SIGNAL_VOLTAGE_180:
				voltage_setting_value = sdhci_mmc->pdata->vreg_data->vdd_io_data->low_vol_level /10000;
				break;
				
			default:
				return -22;
		}
		voltage_flag =1;
	}
		
	
//	voltage_setting_value =msmsdcc_get_vdd_io_vol(host)/10000;
	printk("%s : test test .\n", __func__);
	printk("voltage : %d",voltage_setting_value);
	return sprintf(buf, "%d", voltage_setting_value);
}
DEVICE_ATTR(mmc_voltage_setting, 0777, mmc_voltage_setting_show,mmc_voltage_setting_store);


static int lge_mmc_strength_probe(struct platform_device *pdev)
{
	int err;

	err = device_create_file(&pdev->dev, &dev_attr_mmc_strength);
	if (err < 0)
		printk("%s : Cannot create the sysfs\n", __func__);

	err = device_create_file(&pdev->dev, &dev_attr_mmc_clock_setting);
	if (err < 0)
		printk("%s : Cannot create the sysfs\n", __func__);

	err = device_create_file(&pdev->dev, &dev_attr_mmc_voltage_setting);
	if (err < 0)
		printk("%s : Cannot create the sysfs\n", __func__);
	return 0;
}

static int lge_mmc_strength_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver lge_mmc_strength_driver = {
	.probe = lge_mmc_strength_probe,
	.remove = lge_mmc_strength_remove,
	.suspend = NULL,
	.resume = NULL,
	.driver = {
		.name = "lge_mmc_strength_driver",
		.owner = THIS_MODULE,
	},
};

static int __init lge_mmc_strength_init(void)
{
	platform_driver_register(&lge_mmc_strength_driver);
	return 0;
}

static void __exit lge_mmc_strength_exit(void)
{
	platform_driver_unregister(&lge_mmc_strength_driver);
}

module_init(lge_mmc_strength_init);

MODULE_DESCRIPTION("LGE KERNEL DRIVER");
MODULE_AUTHOR("p1 <p1_filesystem@lge.com>");
MODULE_LICENSE("GPL");
#endif // CONFIG_LGE_ENABLE_MMC_STRENGTH_CONTROL