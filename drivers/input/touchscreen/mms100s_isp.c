#include <linux/delay.h>
#include <linux/gpio.h>
#include "mms100s_ts.h"
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_i2c.h>
#include <linux/errno.h>
#include <asm/unaligned.h>
#include <mach/gpiomux.h>
#include <linux/regulator/consumer.h>

struct mms_info {
	struct i2c_client		*client;
	struct mms_ts_platform_data	*pdata;
};

#if defined(CONFIG_ARCH_MSM8610)
#define GPIO_TOUCH_INT 0
#define GPIO_TSP_SCL 3
#define GPIO_TSP_SDA 2
#define GPIO_TSP_VDD 62
#else
#define GPIO_TOUCH_INT 16
#define GPIO_TSP_SCL 19
#define GPIO_TSP_SDA 18
#endif

static struct gpiomux_setting gpio_i2c_config[2] = {
	{
		.pull = GPIOMUX_PULL_DOWN,
		.drv = GPIOMUX_DRV_8MA,
		.func = GPIOMUX_FUNC_GPIO,
		.dir = GPIOMUX_OUT_LOW,
	},
	{
		.pull = GPIOMUX_PULL_NONE,
		.drv = GPIOMUX_DRV_2MA,
		.func = GPIOMUX_FUNC_3,
	},
};

static struct gpiomux_setting gpio_tocuhint_config = {
	.pull = GPIOMUX_PULL_UP,
	.drv = GPIOMUX_DRV_8MA,
	.func = GPIOMUX_FUNC_GPIO,
	.dir = GPIOMUX_IN,
};

static struct msm_gpiomux_config melfas_gpio_configs[] = {
	{
		.gpio = GPIO_TSP_SDA,
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_i2c_config[0],
			[GPIOMUX_SUSPENDED]    = &gpio_i2c_config[0],
		}
	},
	{
		.gpio = GPIO_TSP_SCL,
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_i2c_config[0],
			[GPIOMUX_SUSPENDED]    = &gpio_i2c_config[0],
		}
	},
	{
		.gpio = GPIO_TOUCH_INT,
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_tocuhint_config,
			[GPIOMUX_SUSPENDED]    = &gpio_tocuhint_config,
		}
	},
};

static struct msm_gpiomux_config melfas_i2c_configs[] = {
	{
		.gpio = GPIO_TSP_SDA,
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_i2c_config[1],
			[GPIOMUX_SUSPENDED]    = &gpio_i2c_config[1],
		}
	},
	{
		.gpio = GPIO_TSP_SCL,
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_i2c_config[1],
			[GPIOMUX_SUSPENDED]    = &gpio_i2c_config[1],
		}
	},
};

int mms100_set_gpio_mode(char gpio)
{
	int ret = 0;
	
	if(gpio == 1)
	{
		printk("<MELFAS> mms100_set_gpio_mode(GPIO) \n");
		msm_gpiomux_install(melfas_gpio_configs, ARRAY_SIZE(melfas_gpio_configs));

		ret = gpio_request(GPIO_TSP_SDA, "Melfas_I2C_SDA");
		if(ret < 0) {
			gpio_free(GPIO_TSP_SDA);
			ret = gpio_request(GPIO_TSP_SDA, "Melfas_I2C_SDA");
		}
		printk(KERN_ERR "[TOUCH] GPIO_TSP_SDA = %d \n", ret);
		
		ret = gpio_request(GPIO_TSP_SCL, "Melfas_I2C_SCL");
		if(ret < 0) {
			gpio_free(GPIO_TSP_SCL);
			ret = gpio_request(GPIO_TSP_SCL, "Melfas_I2C_SCL");
		}
		printk(KERN_ERR "[TOUCH] GPIO_TSP_SCL = %d \n", ret);
		
		ret = gpio_request(GPIO_TOUCH_INT, "Melfas_I2C_INT");
		if(ret < 0) {
			gpio_free(GPIO_TOUCH_INT);
			ret = gpio_request(GPIO_TOUCH_INT, "Melfas_I2C_INT");
		}
		printk(KERN_ERR "[TOUCH] GPIO_TOUCH_INT = %d \n", ret);
	}
	else
	{
		printk("<MELFAS> mms100_set_gpio_mode(I2C) \n");
		msm_gpiomux_install(melfas_i2c_configs , ARRAY_SIZE(melfas_i2c_configs));

		gpio_free(GPIO_TSP_SDA);
		gpio_free(GPIO_TSP_SCL);
		gpio_free(GPIO_TOUCH_INT);
	}

	return ret;
}

static u16 mainErase[2] = {16, 0xD9F3};

#if defined(CONFIG_ARCH_MSM8610)
static void mms_isp_reboot(struct mms_info *info)
{
	printk(KERN_ERR "[TOUCH] mms_isp_reboot \n");
	
	gpio_direction_output(GPIO_TSP_SCL, 0);
	gpio_direction_output(GPIO_TSP_SDA, 0);
	gpio_direction_output(GPIO_TOUCH_INT, 0);


	gpio_direction_output(GPIO_TSP_VDD, 0);
	gpio_set_value(GPIO_TOUCH_INT, 0);
	msleep(150);
	gpio_direction_output(GPIO_TSP_VDD, 1);
	msleep(50);
	
	gpio_set_value(GPIO_TSP_SCL, 0);
	gpio_set_value(GPIO_TSP_SDA, 1);

	msleep(40);
}
#else
static void mms_isp_reboot(struct mms_info *info)
{
	struct regulator *vdd;

	printk(KERN_ERR "[TOUCH] mms_isp_reboot \n");
	
	gpio_direction_output(GPIO_TSP_SCL, 0);
	gpio_direction_output(GPIO_TSP_SDA, 0);
	gpio_direction_output(GPIO_TOUCH_INT, 0);

	vdd = regulator_get(&info->client->dev, "vdd");
	regulator_disable(vdd);
	gpio_set_value(GPIO_TOUCH_INT, 0);
	msleep(150);
	regulator_enable(vdd);
	msleep(50);
	
	gpio_set_value(GPIO_TSP_SCL, 0);
	gpio_set_value(GPIO_TSP_SDA, 1);

	msleep(40);
}
#endif

static void isp_erase_enter(struct mms_info *info)
{
	int i;
	u16 bitnum = mainErase[0];
	u16 code = mainErase[1];

	printk(KERN_ERR "[TOUCH] isp_erase_enter \n");

	for(i=0;i<bitnum;i++)
	{	
		gpio_set_value(GPIO_TSP_SCL,1);
		gpio_set_value(GPIO_TOUCH_INT,0);

		if(code & 0x8000) 
			gpio_set_value(GPIO_TOUCH_INT,1);
		else
			gpio_set_value(GPIO_TOUCH_INT,0);

		code <<=1;
		gpio_set_value(GPIO_TSP_SCL,0);
	}

}


static void isp_erase_code(struct mms_info *info)
{
	printk(KERN_ERR "[TOUCH] isp_erase_code \n");

	gpio_set_value(GPIO_TSP_SDA,1);

	udelay(6);
	gpio_set_value(GPIO_TSP_SCL, 1);
	gpio_set_value(GPIO_TSP_SCL, 0);
	udelay(6);
	gpio_set_value(GPIO_TSP_SCL, 1);
	gpio_set_value(GPIO_TSP_SCL, 0);
	udelay(6);
	gpio_set_value(GPIO_TSP_SCL, 1);
	gpio_set_value(GPIO_TSP_SCL, 0);
	udelay(21);
	gpio_set_value(GPIO_TSP_SCL, 1);
	gpio_set_value(GPIO_TSP_SCL, 0);
	udelay(110);
	gpio_set_value(GPIO_TSP_SCL, 1);
	gpio_set_value(GPIO_TSP_SCL, 0);
	udelay(1);
	gpio_set_value(GPIO_TSP_SCL, 1);
	gpio_set_value(GPIO_TSP_SCL, 0);


}

static void isp_erase_exit(struct mms_info *info)
{
	printk(KERN_ERR "[TOUCH] isp_erase_exit \n");

	gpio_set_value(GPIO_TOUCH_INT,0);
	
	gpio_set_value(GPIO_TSP_SCL,1);
	gpio_set_value(GPIO_TSP_SCL,0);

	gpio_set_value(GPIO_TSP_SCL,1);
	gpio_set_value(GPIO_TSP_SCL,0);

	gpio_set_value(GPIO_TSP_SCL,1);
	gpio_set_value(GPIO_TSP_SCL,0);

	gpio_set_value(GPIO_TSP_SCL,1);
	gpio_set_value(GPIO_TSP_SCL,0);

	gpio_set_value(GPIO_TSP_SCL,1);
	gpio_set_value(GPIO_TSP_SCL,0);

	gpio_set_value(GPIO_TSP_SCL,1);
	gpio_set_value(GPIO_TSP_SCL,0);

	gpio_set_value(GPIO_TSP_SCL,1);
	gpio_set_value(GPIO_TSP_SCL,0);

	gpio_set_value(GPIO_TSP_SCL,1);
	gpio_set_value(GPIO_TSP_SCL,0);

	gpio_set_value(GPIO_TSP_SCL,1);
	gpio_set_value(GPIO_TSP_SCL,0);

	gpio_set_value(GPIO_TSP_SCL,1);
	gpio_set_value(GPIO_TSP_SCL,0);
	
}

int mms_isp_erase(struct i2c_client *client, struct mms_ts_platform_data *pdata)
{
	struct mms_info *info = kzalloc(sizeof(struct mms_info), GFP_KERNEL);

	info->client =client;
	info->pdata =pdata;

	printk(KERN_ERR "[TOUCH] mms_isp_erase start\n");
	
	i2c_lock_adapter(client->adapter);

	mms100_set_gpio_mode(1);
	
	mms_isp_reboot(info);
	isp_erase_enter(info);
	udelay(200);
	isp_erase_code(info);
	isp_erase_exit(info);

	mms100_set_gpio_mode(0);
	
	i2c_unlock_adapter(client->adapter);

	printk(KERN_ERR "[TOUCH] mms_isp_erase end\n");

	kfree(info);
	return 0;
}


