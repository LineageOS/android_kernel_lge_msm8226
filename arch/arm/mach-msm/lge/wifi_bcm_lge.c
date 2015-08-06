#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_device.h>
#endif /* CONFIG_OF */
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>

#include <linux/of_gpio.h>

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/if.h>
#include <linux/random.h>
#include <asm/io.h>
#ifdef CONFIG_WIFI_CONTROL_FUNC
#include <linux/skbuff.h>
#include <linux/wlan_plat.h>
#endif
#ifdef CONFIG_BCMDHD_PCIE
#include <linux/pci.h>
#endif

#include <linux/pm_qos.h>
#include <mach/board_lge.h> // add for hw revision check by hayun.kim

#undef SUPPORT_DTS // for support DeviceTree

#define WLAN_POWER    46
#if defined(CONFIG_MACH_MSM8926_X10_VZW)
#define WLAN_HOSTWAKE 56
#elif defined(CONFIG_MACH_MSM8926_JAGDSNM_CMCC_CN) || defined(CONFIG_MACH_MSM8926_JAGDSNM_CUCC_CN) || defined(CONFIG_MACH_MSM8926_JAGDSNM_CTC_CN)
#define WLAN_HOSTWAKE 35
#elif defined(CONFIG_MACH_MSM8926_JAGNM_ATT) || defined(CONFIG_MACH_MSM8926_JAGN_KR) || defined(CONFIG_MACH_MSM8926_JAGNM_GLOBAL_COM) || defined(CONFIG_MACH_MSM8926_JAGNM_KDDI_JP) || defined(CONFIG_MACH_MSM8926_JAGC_SPR) || defined (CONFIG_MACH_MSM8926_VFP_KR) || defined (CONFIG_MACH_MSM8926_JAGNM_RGS) || defined (CONFIG_MACH_MSM8926_JAGNM_TLS) || defined (CONFIG_MACH_MSM8926_JAGNM_VTR) || defined(CONFIG_MACH_MSM8926_JAGNM_BELL) || defined(CONFIG_MACH_MSM8926_JAGC_SPR)
#define WLAN_HOSTWAKE 54
#elif defined(CONFIG_MACH_MSM8226_JAG3GDS_GLOBAL_COM) || defined(CONFIG_MACH_MSM8226_JAG3GSS_GLOBAL_COM)
#define WLAN_HOSTWAKE 35
#elif defined(CONFIG_MACH_MSM8926_AKA_KR) || defined(CONFIG_MACH_MSM8926_AKA_CN)
#define WLAN_HOSTWAKE 33
#else
#define WLAN_HOSTWAKE 67
#endif

static int gpio_wlan_power = WLAN_POWER;
static int gpio_wlan_hostwake = WLAN_HOSTWAKE;

#ifdef SUPPORT_DTS
static struct pinctrl *wifi_reg_on_pinctrl = NULL;
#else
/* for wifi power supply */
static unsigned wifi_config_power_on[] = {
	    GPIO_CFG(WLAN_POWER, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA) };

static unsigned wlan_wakes_msm[] = {
	    GPIO_CFG(WLAN_HOSTWAKE, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA) };
#endif

#define LGE_BCM_WIFI_DMA_QOS_CONTROL

#if defined(CONFIG_BCM4335BT) 
extern int bcm_bt_lock(int cookie);
extern void bcm_bt_unlock(int cookie);
static int lock_cookie_wifi = 'W' | 'i'<<8 | 'F'<<16 | 'i'<<24; /* cookie is "WiFi" */
#endif // defined(CONFIG_BCM4335BT) 

/*
	Memory allocation is done at dhd_attach
	so static allocation is only necessary in module type driver
*/
#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM

#define PREALLOC_WLAN_NUMBER_OF_SECTIONS	12
#define PREALLOC_WLAN_NUMBER_OF_BUFFERS		160
#define PREALLOC_WLAN_SECTION_HEADER		24

/*
This definition is from driver's dhd.h

enum dhd_prealloc_index {
	DHD_PREALLOC_PROT = 0, 
	DHD_PREALLOC_RXBUF, 
	DHD_PREALLOC_DATABUF, 
	DHD_PREALLOC_OSL_BUF, 
#if defined(STATIC_WL_PRIV_STRUCT) 
	DHD_PREALLOC_WIPHY_ESCAN0 = 5, 
#if defined(CUSTOMER_HW4) && defined(DUAL_ESCAN_RESULT_BUFFER) 
	DHD_PREALLOC_WIPHY_ESCAN1, 
#endif 
#endif
	DHD_PREALLOC_DHD_INFO = 7 
	DHD_PREALLOC_DHD_WLFC_INFO = 8,
	DHD_PREALLOC_IF_FLOW_LKUP = 9,
	DHD_PREALLOC_FLOWRING = 10	
};
*/

#ifdef CONFIG_BCMDHD_SDIO
#define DHD_SKB_HDRSIZE			336
#define DHD_SKB_1PAGE_BUFSIZE		((PAGE_SIZE*1)-DHD_SKB_HDRSIZE)
#define DHD_SKB_2PAGE_BUFSIZE		((PAGE_SIZE*2)-DHD_SKB_HDRSIZE)
#define DHD_SKB_4PAGE_BUFSIZE		((PAGE_SIZE*4)-DHD_SKB_HDRSIZE)
#define WLAN_SKB_BUF_NUM		17

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];
#endif

#define WLAN_SECTION_SKBUFF_IDX		4

#define WLAN_SECTION_SIZE_0		(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#ifdef CONFIG_BCMDHD_SDIO
#define WLAN_SECTION_SIZE_1		(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_2		(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 512)
#else
#define WLAN_SECTION_SIZE_1		0
#define WLAN_SECTION_SIZE_2		0
#endif
#define WLAN_SECTION_SIZE_3		(PREALLOC_WLAN_NUMBER_OF_BUFFERS*1024)
#define WLAN_SECTION_SIZE_4		0 /* Index 4 is static socket buffer */
#define WLAN_SECTION_SIZE_5		(65536)
#define WLAN_SECTION_SIZE_6		(65536)
#define WLAN_SECTION_SIZE_7		(16 * 1024)
#define WLAN_SECTION_SIZE_8		(25 * 1024) // 23032
#ifdef CONFIG_BCMDHD_SDIO
#define WLAN_SECTION_SIZE_9		0
#define WLAN_SECTION_SIZE_10		0
#else
#define WLAN_SECTION_SIZE_9		(18 * 1024) // 16338
#define WLAN_SECTION_SIZE_10		(32 * 1024)
#endif
#ifdef CONFIG_BCMDHD_SDIO
#define WLAN_SECTION_SIZE_11		(73760)	/* sizeof(WLFC_HANGER_SIZE(3072)) */
#else
#define WLAN_SECTION_SIZE_11		0
#endif

struct wlan_mem_prealloc {
	void *mem_ptr;
	unsigned long size;
};

static struct wlan_mem_prealloc wlan_mem_array[PREALLOC_WLAN_NUMBER_OF_SECTIONS] = {
	{ NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_1) },
	{ NULL, (WLAN_SECTION_SIZE_2) },
	{ NULL, (WLAN_SECTION_SIZE_3) },
	{ NULL, (WLAN_SECTION_SIZE_4) },       
	{ NULL, (WLAN_SECTION_SIZE_5) },
	{ NULL, (WLAN_SECTION_SIZE_6) },
	{ NULL, (WLAN_SECTION_SIZE_7) },
	{ NULL, (WLAN_SECTION_SIZE_8) },
	{ NULL, (WLAN_SECTION_SIZE_9) },
	{ NULL, (WLAN_SECTION_SIZE_10) },
	{ NULL, (WLAN_SECTION_SIZE_11) }
};

static void *bcm_wlan_get_mem(int section, unsigned long size)
{
#ifdef CONFIG_BCMDHD_SDIO
	if (section == WLAN_SECTION_SKBUFF_IDX)
		return wlan_static_skb;
#endif

	if ((section < 0) || (section > PREALLOC_WLAN_NUMBER_OF_SECTIONS))
		return NULL;

	if (wlan_mem_array[section].size < size)
		return NULL;

	return wlan_mem_array[section].mem_ptr;
}

static int brcm_init_wlan_mem(void)
{
	int i;
	int j;

#ifdef CONFIG_BCMDHD_SDIO
	for (i = 0; i < 8; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_1PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	for (; i < 16; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_2PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_4PAGE_BUFSIZE);
	if (!wlan_static_skb[i])
		goto err_skb_alloc;
#endif

	for (i = 0 ; i < PREALLOC_WLAN_NUMBER_OF_SECTIONS; i++) {
		if ((i != WLAN_SECTION_SKBUFF_IDX) && (wlan_mem_array[i].size)) {
			wlan_mem_array[i].mem_ptr =
					kmalloc(wlan_mem_array[i].size, GFP_KERNEL);
			if (!wlan_mem_array[i].mem_ptr)
				goto err_mem_alloc;
		}
	}

	printk("%s: WIFI MEM Allocated\n", __FUNCTION__);
	return 0;

err_mem_alloc:
	pr_err("Failed to mem_alloc for WLAN\n");
	for (j = 0 ; j < i ; j++) {
		if ((j != WLAN_SECTION_SKBUFF_IDX) && wlan_mem_array[j].size) {
			kfree(wlan_mem_array[j].mem_ptr);
		}
	}

#ifdef CONFIG_BCMDHD_SDIO
err_skb_alloc:
	i = WLAN_SKB_BUF_NUM;
	pr_err("Failed to skb_alloc for WLAN\n");
	for (j = 0 ; j < i ; j++)
		dev_kfree_skb(wlan_static_skb[j]);
#endif
	return -ENOMEM;
}
#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */


#ifdef LGE_BCM_WIFI_DMA_QOS_CONTROL
static int wifi_dma_state; // 0 : INATIVE, 1:INIT, 2:IDLE, 3:ACTIVE
static struct pm_qos_request wifi_dma_qos;
static struct delayed_work req_dma_work;
static uint32_t packet_transfer_cnt = 0;

static void bcm_wifi_req_dma_work(struct work_struct * work)
{
	switch ( wifi_dma_state ) {
		case 2: //IDLE State
			if ( packet_transfer_cnt < 100 ) {
				// IDLE -> INIT
				wifi_dma_state = 1;
				//printk(KERN_ERR "%s: schedule work : %d : (IDLE -> INIT)\n", __func__, packet_transfer_cnt);
			}
			else {
				// IDLE -> ACTIVE
				wifi_dma_state = 3;
				pm_qos_update_request(&wifi_dma_qos, 7);
				schedule_delayed_work(&req_dma_work, msecs_to_jiffies(50));
				//printk(KERN_ERR "%s: schedule work : %d : (IDLE -> ACTIVE)\n", __func__, packet_transfer_cnt);
			}
			break;

		case 3: //ACTIVE State
			if ( packet_transfer_cnt < 10 ) {
				// ACTIVE -> IDLE
				wifi_dma_state = 2;
				pm_qos_update_request(&wifi_dma_qos, PM_QOS_DEFAULT_VALUE);
				schedule_delayed_work(&req_dma_work, msecs_to_jiffies(1000));
				//printk(KERN_ERR "%s: schedule work : %d : (ACTIVE -> IDLE)\n", __func__, packet_transfer_cnt);
			}
			else {
				// Keep ACTIVE
				schedule_delayed_work(&req_dma_work, msecs_to_jiffies(50));
				//printk(KERN_ERR "%s: schedule work : %d :  (ACTIVE -> ACTIVE)\n", __func__, packet_transfer_cnt);
			}
			break;

		default:
			break;
		
	}

	packet_transfer_cnt = 0;
}

void bcm_wifi_req_dma_qos(int vote)
{
	if (vote) {
		packet_transfer_cnt++;
	}

	// INIT -> IDLE
	if ( wifi_dma_state == 1 && vote ) {
		wifi_dma_state = 2; // IDLE
		schedule_delayed_work(&req_dma_work, msecs_to_jiffies(1000));
		//printk(KERN_ERR "%s: schedule work (INIT -> IDLE)\n", __func__);
	}
}
#endif

int bcm_wifi_set_power(int enable)
{
	int ret = 0;
		
#if defined(CONFIG_BCM4335BT) 
	printk("%s: trying to acquire BT lock\n", __func__);
	if (bcm_bt_lock(lock_cookie_wifi) != 0)
		printk("%s:** WiFi: timeout in acquiring bt lock**\n", __func__);
	else 
		printk("%s: btlock acquired\n", __func__);
#endif // defined(CONFIG_BCM4335BT) 


	if (enable) {
		ret = gpio_direction_output(gpio_wlan_power, 1);
		if (ret) {
			printk(KERN_ERR "%s: WL_REG_ON  failed to pull up (%d)\n",
					__func__, ret);
			ret = -EIO;
			goto out;
		}

		// WLAN chip to reset
		mdelay(150);
		printk(KERN_ERR "%s: wifi power successed to pull up\n",__func__);
	} else {
		ret = gpio_direction_output(gpio_wlan_power, 0); 
		if (ret) {
			printk(KERN_ERR "%s:  WL_REG_ON  failed to pull down (%d)\n",
					__func__, ret);
			ret = -EIO;
			goto out;
		}
		// WLAN chip down 
		printk(KERN_ERR "%s: wifi power successed to pull down\n",__func__);
	}

#if defined(CONFIG_BCM4335BT) 
	bcm_bt_unlock(lock_cookie_wifi);
#endif // defined(CONFIG_BCM4335BT) 

	return ret;

out : 
#if defined(CONFIG_BCM4335BT) 
	/* For a exceptional case, release btlock */
	printk("%s: exceptional bt_unlock\n", __func__);
	bcm_bt_unlock(lock_cookie_wifi);
#endif // defined(CONFIG_BCM4335BT) 

	return ret;
}

static int bcm_wifi_reset(int on)
{
	return 0;
}

#ifdef CONFIG_BCMDHD_SDIO
static unsigned int g_wifi_detect;
static void *sdc_dev;
void (*sdc_status_cb)(int card_present, void *dev);

int wcf_status_register(void (*cb)(int card_present, void *dev), void *dev)
{
	pr_info("%s\n", __func__);

	if (sdc_status_cb)
		return -EINVAL;

	sdc_status_cb = cb;
	sdc_dev = dev;

	return 0;
}

unsigned int wcf_status(struct device *dev)
{
	pr_info("%s: wifi_detect = %d\n", __func__, g_wifi_detect);
	return g_wifi_detect;
}
#endif

static int bcm_wifi_carddetect(int val)
{
	int ret = 0;

#ifdef CONFIG_BCMDHD_SDIO

	g_wifi_detect = val;

	if (sdc_status_cb)
		sdc_status_cb(val, sdc_dev);
	else
		pr_warn("%s: There is no callback for notify\n", __func__);
	return ret;

#elif defined(CONFIG_BCMDHD_PCIE)

#define PCIE_VENDOR_ID_RCP	0x17cb
#define PCIE_DEVICE_ID_RCP	0x0300
#define PCIE_RCP_NAME		"0001:00:00.0"
#define PCIE_POWERUP_RETRY	10

	int found = 0;
	int count = 0;
	struct pci_dev *pcidev = NULL;

	if (val == 1) {
		do {
			pcidev = pci_get_device(PCIE_VENDOR_ID_RCP, PCIE_DEVICE_ID_RCP, pcidev);
			if (pcidev && (!strcmp(pci_name(pcidev), (const char *)PCIE_RCP_NAME))) {
				printk("P:%s:PCI device found[%X:%X]!!!\n", __func__, pcidev->vendor, pcidev->device);
				found = 1;
			} else {
				count++;
				printk("P:%s:retry count[%d]\n", __func__, count);
				msleep(100);
			}
		} while(!found && (count < PCIE_POWERUP_RETRY));

		if (!found) {
			ret = -1;
		}
	}

#endif
	return ret;
}

static int bcm_wifi_get_mac_addr(unsigned char *buf)
{
	uint rand_mac;
	static unsigned char mymac[6] = {0,};
	const unsigned char nullmac[6] = {0,};
	pr_debug("%s: %p\n", __func__, buf);

	if( buf == NULL ) return -EAGAIN;

	if( memcmp( mymac, nullmac, 6 ) != 0 )
	{
		/* Mac displayed from UI are never updated..
		   So, mac obtained on initial time is used */
		memcpy( buf, mymac, 6 );
		return 0;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0))
	prandom_seed((uint)jiffies);
	rand_mac = prandom_u32();
#else
	srandom32((uint)jiffies);
	rand_mac = random32();
#endif
	buf[0] = 0x00;
	buf[1] = 0x90;
	buf[2] = 0x4c;
	buf[3] = (unsigned char)rand_mac;
	buf[4] = (unsigned char)(rand_mac >> 8);
	buf[5] = (unsigned char)(rand_mac >> 16);

	memcpy(mymac, buf, 6);

	printk(KERN_INFO "[%s] Exiting. MyMac :  %x : %x : %x : %x : %x : %x\n", __func__ , buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

	return 0;
}

#define COUNTRY_BUF_SZ	4
struct cntry_locales_custom {
	char iso_abbrev[COUNTRY_BUF_SZ];
	char custom_locale[COUNTRY_BUF_SZ];
	int custom_locale_rev;
};

/* Customized Locale table */
const struct cntry_locales_custom bcm_wifi_translate_custom_table[] = {
/* Table should be filled out based on custom platform regulatory requirement BCM4334 series */
           {"",       "GB",     0},
           {"AD",    "GB",     0},
           {"AE",    "BR",     0},
           {"AF",    "GB",     0},
           {"AG",    "BR",     0},
           {"AI",     "US",     100},
           {"AL",    "GB",     0},
           {"AM",   "KW",      1},
           {"AN",   "BR",     0},
           {"AO",   "GB",      0},
           {"AR",    "AR",     1},
           {"AS",    "US",     100},
           {"AT",    "GB",     0},
           {"AU",    "AR",    1},
           {"AW",   "BR",     0},
           {"AZ",    "GB",     0},
           {"BA",    "GB",     0},
           {"BB",    "CN",     0},
           {"BD",    "QA",    0},
           {"BE",    "GB",     0},
           {"BF",    "GB",    0},
           {"BG",    "GB",     0},
           {"BH",    "CN",     0},
           {"BI",     "GB",      0},
           {"BJ",     "GB",      0},
           {"BM",   "US",     100},
           {"BN",    "CN",     0},
           {"BO",    "NG",      0},
           {"BR",    "BR",     0},
           {"BS",    "US",     100},
           {"BT",    "GB",      0},
           {"BW",   "GB",     0},
           {"BY",    "GB",     0},
           {"BZ",    "BR",      0},
           {"CA",    "US",     100},
           {"CD",    "GB",      0},
           {"CF",    "GB",      0},
           {"CG",    "GB",      0},
           {"CH",    "GB",     0},
           {"CI",     "GB",      0},
           {"CK",    "BR",     0},
           {"CL",    "CN",     0},
           {"CM",   "GB",      0},
           {"CN",    "CN",    0},
           {"CO",    "BR",     0},
           {"CR",    "BR",     0},
           {"CU",    "BR",     0},
           {"CV",    "GB",     0},
           {"CX",    "AR",    	1},
           {"CY",    "GB",     0},
           {"CZ",    "GB",     0},
           {"DE",    "GB",     0},
           {"DJ",    "IL",      10},
           {"DK",    "GB",     0},
           {"DM",   "BR",     0},
           {"DO",   "BR",     0},
           {"DZ",    "GB",    0},
           {"EC",    "BR",     0},
           {"EE",    "GB",     0},
           {"EG",    "CN",     0},
           {"ER",    "IL",      10},
           {"ES",    "GB",     0},
           {"ET",    "GB",     0},
           {"FI",     "GB",     0},
           {"FJ",     "AR",      1},
           {"FK",     "GB",      0},
           {"FM",   "US",     100},
           {"FO",    "GB",     0},
           {"FR",    "GB",     0},
           {"GA",    "GB",      0},
           {"GB",    "GB",     0},
           {"GD",    "BR",     0},
           {"GE",    "GB",     0},
           {"GF",    "GB",     0},
           {"GH",    "GB",     0},
           {"GI",     "GB",     0},
           {"GL",   "GB",      0},
           {"GM",   "GB",      0},
           {"GN",   "GB",      10},
           {"GP",    "GB",     0},
           {"GQ",   "GB",      0},
           {"GR",    "GB",     0},
           {"GT",    "BR",     0},
           {"GU",    "US",     100},
           {"GW",   "GB",      0},
           {"GY",    "QA",    0},
           {"HK",    "BR",     0},
           {"HN",   "QA",    0},
           {"HR",    "GB",     0},
           {"HT",    "QA",     0},
           {"HU",    "GB",     0},
           {"ID",     "ID",    1},
           {"IE",     "GB",     0},
           {"IL",     "KW",      1},
           {"IM",    "GB",     0},
           {"IN",    "CN",     0},
           {"IQ",    "GB",      0},
           {"IR",     "IL",      10},
           {"IS",     "GB",     0},
           {"IT",     "GB",     0},
           {"JE",     "GB",     0},
           {"JM",    "QA",     0},
           {"JO",    "JO",     0},
#if defined (CONFIG_MACH_MSM8926_JAGNM_KDDI_JP)
           {"JP",     "JP",      36},
#else
           {"JP",     "JP",      5},
#endif
           {"KE",    "SA",     0},
           {"KG",    "GB",      0},
           {"KH",    "BR",     0},
           {"KI",     "AR",    1},
           {"KM",   "GB",      0},
           {"KP",    "IL",      10},
           {"KR",    "KR",     24},
           {"KW",   "KW",    1},
           {"KY",    "US",     100},
           {"KZ",    "GB",     0},
           {"LA",    "BR",     0},
           {"LB",    "BR",     0},
           {"LC",    "BR",     0},
           {"LI",     "GB",     0},
           {"LK",    "BR",     0},
           {"LR",    "BR",     0},
           {"LS",     "GB",     0},
           {"LT",     "GB",     0},
           {"LU",    "GB",     0},
           {"LV",     "GB",     0},
           {"LY",     "GB",      0},
           {"MA",   "KW",    1},
           {"MC",   "GB",     0},
           {"MD",   "GB",     0},
           {"ME",   "GB",     0},
           {"MF",   "BR",     0},
           {"MG",   "GB",      0},
           {"MG",   "GB",      0},
           {"MK",   "GB",     0},
           {"ML",    "GB",      0},
           {"MM",  "GB",      0},
           {"MN",   "BR",      0},
           {"MO",   "BR",    0},
           {"MP",   "US",     100},
           {"MQ",   "GB",     0},
           {"MR",   "GB",     0},
           {"MS",   "GB",     0},
           {"MT",   "GB",     0},
           {"MU",   "GB",     0},
           {"MD",   "GB",     0},
           {"ME",   "GB",     0},
           {"MF",   "BR",     0},
           {"MG",   "GB",      0},
           {"MH",   "BR",     0},
           {"MK",   "GB",     0},
           {"ML",    "GB",      0},
           {"MM",  "GB",      0},
           {"MN",   "BR",      0},
           {"MO",   "BR",    0},
           {"MP",   "US",     100},
           {"MQ",   "GB",     0},
           {"MR",   "GB",     0},
           {"MS",   "GB",     0},
           {"MT",   "GB",     0},
           {"MU",   "GB",     0},
           {"MV",   "CN",     0},
           {"MW",  "BR",    0},
           {"MX",   "AR",     1},
           {"MY",   "CN",     0},
           {"MZ",   "BR",     0},
           {"NA",   "BR",     0},
           {"NC",    "GB",      0},
           {"NE",    "GB",     0},
           {"NF",    "BR",     0},
           {"NG",   "NG",    0},
           {"NI",    "BR",     0},
           {"NL",    "GB",     0},
           {"NO",   "GB",     0},
           {"NP",    "NP",     0},
           {"NR",    "GB",      0},
           {"NU",   "BR",     0},
           {"NZ",    "BR",     0},
           {"OM",   "GB",     0},
           {"PA",    "CN",     0},
           {"PE",    "BR",     0},
           {"PF",    "GB",     0},
           {"PG",    "AR",     1},
           {"PH",    "BR",     0},
           {"PK",    "QA",    0},
           {"PL",     "GB",     0},
           {"PM",   "GB",     0},
           {"PN",    "GB",     0},
           {"PR",    "US",     100},
           {"PS",    "BR",     0},
           {"PT",    "GB",     0},
           {"PW",   "BR",     0},
           {"PY",    "BR",     0},
           {"QA",   "QA",    0},
           {"RE",    "GB",     0},
           {"RKS",   "GB",     0},
           {"RO",    "GB",     0},
           {"RS",    "GB",     0},
           {"RU",    "AR",     1},
           {"RW",   "GB",    0},
           {"SA",    "SA",     0},
           {"SB",    "IL",      10},
           {"SC",    "BR",      10},
           {"SD",    "GB",     0},
           {"SE",    "GB",     0},
           {"SG",    "BR",     0},
           {"SI",     "GB",     0},
           {"SK",    "GB",     0},
           {"SKN",  "BR",   0},
           {"SL",     "GB",      0},
           {"SM",   "GB",     0},
           {"SN",    "KW",     1},
           {"SO",    "IL",      10},
           {"SR",    "GB",      0},
           {"SS",    "GB",     0},
           {"ST",    "GB",      0},
           {"SV",    "BR",     0},
           {"SY",    "BR",     0},
           {"SZ",    "GB",      0},
           {"TC",    "GB",     0},
           {"TD",    "GB",      0},
           {"TF",    "GB",     0},
           {"TG",    "GB",      0},
           {"TH",    "BR",     0},
           {"TJ",     "GB",      0},
           {"TL",     "BR",     0},
           {"TM",   "GB",      0},
           {"TN",    "KW",    1},
           {"TO",    "IL",      10},
           {"TR",    "GB",     0},
           {"TT",    "BR",     0},
           {"TV",    "IL",      10},
           {"TW",   "TW",    2},
           {"TZ",    "GB",    0},
           {"UA",    "CN",     0},
           {"UG",    "SA",     0},
           {"UM",    "US",     100},
           {"US",    "US",     100},
           {"UY",    "CN",     0},
           {"UZ",    "KW",      1},
           {"VA",    "GB",     0},
           {"VC",    "BR",     0},
           {"VE",    "CN",     0},
           {"VG",    "BR",     0},
           {"VI",     "US",     100},
           {"VN",    "BR",     0},
           {"VU",    "BR",      0},
           {"WS",   "SA",     0},
           {"YE",    "BR",      0},
           {"YT",    "GB",     0},
           {"ZA",    "GB",     0},
           {"ZM",   "BR",     0},
           {"ZW",   "GB",     0},
};

static void *bcm_wifi_get_country_code(char *ccode)
{
	int size, i;
	static struct cntry_locales_custom country_code;

	size = ARRAY_SIZE(bcm_wifi_translate_custom_table);

	if ((size == 0) || (ccode == NULL))
		return NULL;

	for (i = 0; i < size; i++) {
		if (strcmp(ccode, bcm_wifi_translate_custom_table[i].iso_abbrev) == 0) {
			return (void *)&bcm_wifi_translate_custom_table[i];
		}
	}   

	memset(&country_code, 0, sizeof(struct cntry_locales_custom));
	strlcpy(country_code.custom_locale, ccode, COUNTRY_BUF_SZ);

	return (void *)&country_code;
}

static struct wifi_platform_data bcm_wifi_control = {
#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
	.mem_prealloc	= bcm_wlan_get_mem,
#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */
	.set_power		= bcm_wifi_set_power,
	.set_reset      = bcm_wifi_reset,
	.set_carddetect = bcm_wifi_carddetect,
	.get_mac_addr   = bcm_wifi_get_mac_addr, 
	.get_country_code = bcm_wifi_get_country_code,
};

static struct resource wifi_resource[] = {
	[0] = {
		.name = "bcmdhd_wlan_irq",
		.start = 0,  //assigned later
		.end   = 0,  //assigned later
		//.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE, // for HW_OOB
		.flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE | IORESOURCE_IRQ_LOWEDGE | IORESOURCE_IRQ_SHAREABLE, // for SW_OOB
	},
};

static struct platform_device bcm_wifi_device = {
	.name           = "bcmdhd_wlan",
	.id             = 1,
	.num_resources  = ARRAY_SIZE(wifi_resource),
	.resource       = wifi_resource,
	.dev            = {
		.platform_data = &bcm_wifi_control,
	},
};

int bcm_wifi_init_mem(struct platform_device *platdev)
{
#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
	brcm_init_wlan_mem();
#endif
	printk(KERN_INFO "bcm_wifi_init_mem successfully\n");

	return 0;
}

int bcm_wifi_init_gpio(struct platform_device *platdev)
{
	int ret = 0;
	
#ifdef SUPPORT_DTS
	struct device_node *np = platdev->dev.of_node;

	wifi_reg_on_pinctrl = devm_pinctrl_get(&platdev->dev);
	if (IS_ERR_OR_NULL(wifi_reg_on_pinctrl)) {
		printk("%s: target does not use pinctrl for wifi reg on\n", __func__);
	}

	gpio_wlan_power = of_get_named_gpio_flags(np, "wlan-en-gpio", 0, NULL);
	printk(KERN_INFO "%s: gpio_wlan_power : %d\n", __FUNCTION__, gpio_wlan_power);
	if (!gpio_is_valid(gpio_wlan_power)) {
		printk("P:%s:gpio %d for reset is not valid.\n", __func__, gpio_wlan_power);
	}

	gpio_wlan_hostwake = of_get_named_gpio_flags(np, "wlan-hostwake-gpio", 0, NULL);
	printk(KERN_INFO "%s: gpio_wlan_hostwake : %d\n", __FUNCTION__, gpio_wlan_hostwake);
	if (!gpio_is_valid(gpio_wlan_hostwake)) {
		printk("P:%s:gpio %d for reset is not valid.\n", __func__, gpio_wlan_hostwake);
	}
#else
	{
		int hw_rev = HW_REV_A;	
		hw_rev = lge_get_board_revno();

#if defined (CONFIG_MACH_MSM8926_JAGC_SPR)
			if ( HW_REV_A < hw_rev ) {
					gpio_wlan_hostwake		= 54;
				}
				else {
					gpio_wlan_hostwake		= 56;
				}
#endif
		
#if defined (CONFIG_MACH_MSM8926_JAGNM_ATT) || defined (CONFIG_MACH_MSM8926_JAGN_KR) || defined(CONFIG_MACH_MSM8926_JAGNM_GLOBAL_COM) || defined(CONFIG_MACH_MSM8926_JAGNM_KDDI_JP) || defined (CONFIG_MACH_MSM8926_VFP_KR)|| defined (CONFIG_MACH_MSM8926_JAGNM_RGS) || defined (CONFIG_MACH_MSM8926_JAGNM_TLS) || defined (CONFIG_MACH_MSM8926_JAGNM_VTR) || defined(CONFIG_MACH_MSM8926_JAGNM_BELL) || defined(CONFIG_MACH_MSM8926_JAGC_SPR)
				if ( HW_REV_A <= hw_rev ) {
					gpio_wlan_hostwake	= 54;
				}
				else {
					gpio_wlan_hostwake	= 56;
				}
#endif
		
		
#if defined (CONFIG_MACH_MSM8226_JAG3GDS_GLOBAL_COM) || defined(CONFIG_MACH_MSM8226_JAG3GSS_GLOBAL_COM)
			if ( HW_REV_A < hw_rev ) {
					gpio_wlan_hostwake		= 35;
				}
				else {
					gpio_wlan_hostwake		= 34;
				}
#endif
		
#if defined (CONFIG_MACH_MSM8926_B2L_ATT) || defined(CONFIG_MACH_MSM8926_X10_VZW) || defined(CONFIG_MACH_MSM8926_B2LN_KR)
				if ( HW_REV_B <= hw_rev ) {
						gpio_wlan_hostwake		= 54;
				}
#endif

		wifi_config_power_on[0] = GPIO_CFG(gpio_wlan_power, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA);
		wlan_wakes_msm[0] = GPIO_CFG(gpio_wlan_hostwake, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);

		//WLAN_POWER
		ret = gpio_tlmm_config(wifi_config_power_on[0], GPIO_CFG_ENABLE);
		if (ret) {
			printk(KERN_ERR "%s: Failed to configure WiFi Reset GPIO:[%d]\n", __func__, ret);
		}
		//HOST_WAKEUP
		ret = gpio_tlmm_config(wlan_wakes_msm[0], GPIO_CFG_ENABLE);	
		if (ret) {
			printk(KERN_ERR "%s: Failed to configure Hostwakeup:[%d]\n",__func__, ret);
		}
	}
#endif

	/* WLAN_POWER */
	if ((ret = gpio_request_one(gpio_wlan_power, GPIOF_OUT_INIT_LOW, "wifi_reg_on")) < 0)
		printk("%s: Failed to request gpio %d for bcmdhd_wifi_reg_on:[%d]\n", __func__, gpio_wlan_power, ret);

	msleep(10);

#if defined(CONFIG_BCMDHD_SDIO) && defined(SUPPORT_DTS)
	gpio_free(gpio_wlan_power); // for cd-gpios
#endif

	ret = gpio_request_one(gpio_wlan_hostwake, GPIOF_IN, "wifi_hostwakeup");
	if (ret) {
		printk("Failed to request gpio %d for wlan_wakes_msm:[%d]\n", gpio_wlan_hostwake, ret);
	}

	if (gpio_is_valid(gpio_wlan_hostwake)) {
		wifi_resource[0].start = wifi_resource[0].end = gpio_to_irq(gpio_wlan_hostwake);
	}

	printk(KERN_INFO "bcm_wifi_init_gpio successfully\n");

	return 0;
}

#ifdef SUPPORT_DTS
static int bcm_wifi_probe(struct platform_device *pdev)
{
	bcm_wifi_init_mem(pdev);
	bcm_wifi_init_gpio(pdev);

	return 0;
}

static int bcm_wifi_remove(struct platform_device *pdev)
{

	return 0;
}

static struct of_device_id bcm_wifi_match_table[] = {
	{ .compatible = "lge,bcmdhd_wlan" },
	{ },
};

static struct platform_driver bcm_wifi_driver = {
	.probe = bcm_wifi_probe,
	.remove = bcm_wifi_remove,
	.driver = {
		.name = "wifi_bcm_lge",
		.owner = THIS_MODULE,
		.of_match_table = bcm_wifi_match_table,
	},
};
#endif

static int __init init_bcm_wifi(void)
{
#ifdef CONFIG_WIFI_CONTROL_FUNC

#ifdef LGE_BCM_WIFI_DMA_QOS_CONTROL
	INIT_DELAYED_WORK(&req_dma_work, bcm_wifi_req_dma_work);
	pm_qos_add_request(&wifi_dma_qos, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
	wifi_dma_state = 1; //INIT
	printk("%s: wifi_dma_qos is added\n", __func__);
#endif

	platform_device_register(&bcm_wifi_device);
#ifdef SUPPORT_DTS
	platform_driver_register(&bcm_wifi_driver);
#else
	bcm_wifi_init_mem(NULL);
	bcm_wifi_init_gpio(NULL);
#endif
#endif

	return 0;
}

subsys_initcall(init_bcm_wifi);

