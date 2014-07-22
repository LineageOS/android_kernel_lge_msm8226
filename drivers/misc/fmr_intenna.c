/*
 * FM radio Intenna driver
 *
 * Copyright (C) 2009-2012 LGE, Inc.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#if defined(CONFIG_FMR_INTENNA)

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include "../staging/android/timed_output.h"
#include <linux/types.h>
#include <linux/err.h>
#include <mach/msm_iomap.h>
#include <linux/io.h>
#include <mach/gpiomux.h>
#include <mach/board_lge.h>
#include <linux/i2c.h>
#include <mach/msm_xo.h>
#include <linux/slab.h>

#include <linux/ioctl.h>
#include <asm/ioctls.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/of_gpio.h>
#include <linux/clk.h>

#define FMR_INTENNA_IOCTL_MAGIC 'a'
#define FMR_INTENNA_START        _IOW(FMR_INTENNA_IOCTL_MAGIC, 0, int)
#define FMR_INTENNA_STOP         _IOW(FMR_INTENNA_IOCTL_MAGIC, 1, int)

#define GPIO_FMR_INTENNA_PWM               58

struct fmr_intenna_data {
	struct platform_device dev;
	int fmr_intenna_gpio;
};

#ifdef CONFIG_OF
static void fmr_intenna_parse_dt(struct device *dev, struct fmr_intenna_data *data)
{
	struct device_node *np = dev->of_node;
	data->fmr_intenna_gpio = of_get_named_gpio_flags(np, "lge,fmr-intenna-gpio", 0, NULL);
	pr_info("fmr_intenna_gpio = %d\n", data->fmr_intenna_gpio);
}

static struct of_device_id fmr_intenna_match_table[] = {
	{ .compatible = "lge,fmr_intenna",},
	{ },
};
#endif

static int fmr_intenna_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int fmr_intenna_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t fmr_intenna_write(struct file *file, const char __user *buf,
	size_t count, loff_t *pos)
{
	return 0;
}

int fmr_intenna_fsync(struct file *file, loff_t a, loff_t b, int datasync)
{
	return 0;
}

static long fmr_intenna_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int rc = 0;

	switch (cmd) {
	case FMR_INTENNA_START:
		pr_info("FMR Intenna start");
		gpio_direction_output(GPIO_FMR_INTENNA_PWM,1);
		break;

	case FMR_INTENNA_STOP:
		pr_info("FMR Intenna stop");
		gpio_direction_output(GPIO_FMR_INTENNA_PWM,0);
		break;
	default:
		pr_info("CMD ERROR: cmd:%d\n", cmd);
		rc = -EINVAL;
	}

	return rc;
}

static const struct file_operations fmr_intenna_fops = {
	.owner		= THIS_MODULE,
	.open		= fmr_intenna_open,
	.release	= fmr_intenna_release,
	.write		= fmr_intenna_write,
	.unlocked_ioctl	= fmr_intenna_ioctl,
	.fsync = fmr_intenna_fsync,
};

struct miscdevice fmr_intenna_misc = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "msm_fmr_intenna_dec",
	.fops	= &fmr_intenna_fops,
};

struct fmr_intenna_data fmr_intenna = {
	.dev.name = "fmr_intenna",
};

#ifdef CONFIG_DEBUG_FS
static struct dentry *debugfs_wcd9xxx_dent;
static struct dentry *debugfs_poke;

static int codec_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static int get_parameters(char *buf, long int *param1, int num_of_par)
{
	char *token;
	int base, cnt = 0;
	token = strsep(&buf, " ");

	while (token != NULL) {
		if ((strlen(token) > 2) && ((token[1] == 'x') || (token[1] == 'X')))
			base = 16;
		else
			base = 10;

		if (strict_strtoul(token, base, &param1[cnt]) != 0){
			pr_err("strict_strtoul error!!\n");
			return -EINVAL;
		}
		cnt ++;
		token = strsep(&buf, " ");
	}
	return 0;
}

static ssize_t codec_debug_write(struct file *filp,
	const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	char *access_str = filp->private_data;
	char lbuf[32];
	int rc;
	long int param[5] = {0,};

	if (cnt > sizeof(lbuf) - 1)
		return -EINVAL;

	rc = copy_from_user(lbuf, ubuf, cnt);
	if (rc)
		return -EFAULT;

	lbuf[cnt] = '\0';

	//INFO_MSG("access_str:%s lbuf:%s cnt:%d\n", access_str, lbuf, cnt);

	if (!strncmp(access_str, "poke", 6)) {
		rc = get_parameters(lbuf, param, 1);
		if (rc) {
			pr_err("error!!! get_parameters rc = %d\n", rc);
			return rc;
		}

		//if you want to turn on with pwm clock(33Khz), duty(50%), #echo 1 33 50 > /sys/kernel/debug/sw_irrc/poke
		//if you want to turn on with gpio level high.             #echo 1 0 0 > /sys/kernel/debug/sw_irrc/poke
		//if you want to turn off,                                 #echo 0 33 50 > /sys/kernel/debug/sw_irrc/poke
		switch (param[0]) {
		case 1:
			pr_info("FMR Intenna start");
			gpio_direction_output(GPIO_FMR_INTENNA_PWM,1);
			break;
		case 0:
			pr_info("FMR Intenna stop");
			gpio_direction_output(GPIO_FMR_INTENNA_PWM,0);
			break;
		default:
			rc = -EINVAL;
		}

	}

	if (rc == 0)
		rc = cnt;
	else
		pr_err("rc = %d\n", rc);

	return rc;
}

static const struct file_operations codec_debug_ops = {
	.open = codec_debug_open,
	.write = codec_debug_write,
};
#endif


static int fmr_intenna_probe(struct platform_device *pdev)
{
	int nRet;
	struct fmr_intenna_data *fmr;

	pr_info("probe\n");

	platform_set_drvdata(pdev, &fmr_intenna);
	fmr = (struct fmr_intenna_data *)platform_get_drvdata(pdev);

#ifdef CONFIG_OF
	if (pdev->dev.of_node) {
		fmr_intenna_parse_dt(&pdev->dev, fmr);
	}
#endif

	nRet = gpio_request(fmr->fmr_intenna_gpio, "fmr_intenna");
	gpio_direction_output(fmr->fmr_intenna_gpio, 0);

	if (nRet) {
		pr_err("FMR Intenna GPIO set failed.\n");
		return -ENODEV;
	}

	nRet = misc_register(&fmr_intenna_misc);
	if (nRet) {
		pr_err("misc_register failed.\n");
		return -ENODEV;
	}

	pdev->dev.init_name = fmr->dev.name;
	pr_info("dev->init_name : %s, dev->kobj : %s\n", pdev->dev.init_name, pdev->dev.kobj.name);

#ifdef CONFIG_DEBUG_FS
	debugfs_wcd9xxx_dent = debugfs_create_dir("fmr_intenna", 0);
	if (!IS_ERR(debugfs_wcd9xxx_dent)) {
		debugfs_poke = debugfs_create_file("poke", S_IFREG | S_IRUGO, debugfs_wcd9xxx_dent, (void *) "poke", &codec_debug_ops);
	}
#endif
	return 0;
}

static int fmr_intenna_remove(struct platform_device *pdev)
{
	misc_deregister(&fmr_intenna_misc);

#ifdef CONFIG_DEBUG_FS
	debugfs_remove(debugfs_poke);
	debugfs_remove(debugfs_wcd9xxx_dent);
#endif
	return 0;
}

static void fmr_intenna_shutdown(struct platform_device *pdev)
{

}

static struct platform_driver fmr_intenna_driver = {
	.probe = fmr_intenna_probe,
	.remove = fmr_intenna_remove,
	.shutdown = fmr_intenna_shutdown,
	.suspend = NULL,
	.resume = NULL,
	.driver = {
		.name = "fmr_intenna",
#ifdef CONFIG_OF
		.of_match_table = fmr_intenna_match_table,
#endif
	},
};

static int __init fmr_intenna_init(void)
{
	pr_info("init\n");

	return platform_driver_register(&fmr_intenna_driver);
}

static void __exit fmr_intenna_exit(void)
{
	pr_info("exit\n");
	platform_driver_unregister(&fmr_intenna_driver);
}

late_initcall_sync(fmr_intenna_init); /* to let init lately */
module_exit(fmr_intenna_exit);

MODULE_AUTHOR("LG Electronics Inc.");
MODULE_DESCRIPTION("FMR Intenna Driver");
MODULE_LICENSE("GPL");

#endif
