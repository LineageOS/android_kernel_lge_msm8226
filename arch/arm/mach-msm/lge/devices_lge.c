#include <linux/kernel.h>
#include <linux/string.h>

#include <mach/board_lge.h>

#include <linux/platform_device.h>
#include <linux/persistent_ram.h>
#include <asm/setup.h>
#include <asm/system_info.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#ifdef CONFIG_LGE_HANDLE_PANIC
#include <mach/lge_handle_panic.h>
#endif

#ifdef CONFIG_LGE_PM
#include <linux/qpnp/qpnp-adc.h>
#include <mach/board_lge.h>
#endif

#define PROP_VAL_MAX_SIZE 50

#ifdef CONFIG_ANDROID_PERSISTENT_RAM
#include <mach/msm_memtypes.h>
#endif

#ifdef CONFIG_LGE_QSDL_SUPPORT
#include <mach/lge_qsdl.h>
#endif
/* in drivers/staging/android */
#include "ram_console.h"

#ifdef CONFIG_USB_G_LGE_ANDROID
#include <linux/platform_data/lge_android_usb.h>
#endif
#include <linux/power_supply.h>

static int cn_arr_len = 3;

#ifdef CONFIG_LGE_PM_USB_ID
struct chg_cable_info_table {
	int threshhold;
	acc_cable_type type;
	unsigned ta_ma;
	unsigned usb_ma;
};

#define ADC_NO_INIT_CABLE   0
#define C_NO_INIT_TA_MA     0
#define C_NO_INIT_USB_MA    0
#define ADC_CABLE_NONE      1900000
#define C_NONE_TA_MA        700
#define C_NONE_USB_MA       500

#define MAX_CABLE_NUM		12

static bool cable_type_defined;
static struct chg_cable_info_table pm_acc_cable_type_data[MAX_CABLE_NUM];
#endif

struct cn_prop {
	char *name;
	enum cn_prop_type type;
	uint32_t cell_u32;
	uint64_t cell_u64;
	char str[PROP_VAL_MAX_SIZE];
	uint8_t is_valid;
};

static struct cn_prop cn_array[] = {
	{
		.name = "lge,log_buffer_phy_addr",
		.type = CELL_U32,
	},
	{
		.name = "lge,sbl_delta_time",
		.type = CELL_U32,
	},
	{
		.name = "lge,lk_delta_time",
		.type = CELL_U32,
	},
};

int __init lge_init_dt_scan_chosen(unsigned long node, const char *uname,
								int depth, void *data)
{
	unsigned long len;
	int i;
	enum cn_prop_type type;
	char *p;
	uint32_t *u32;
	void *temp;

    if (depth != 1 || (strcmp(uname, "chosen") != 0
					   && strcmp(uname, "chosen@0") != 0))
		return 0;
	for (i = 0; i < cn_arr_len; i++) {
		type = cn_array[i].type;
		temp = of_get_flat_dt_prop(node, cn_array[i].name, &len);
		if (temp == NULL)
			continue;
		if (type == CELL_U32) {
			u32 = of_get_flat_dt_prop(node, cn_array[i].name, &len);
			if(u32)
			    cn_array[i].cell_u32 = of_read_ulong(u32, 1);
		} else if (type == CELL_U64) {
			u32 = of_get_flat_dt_prop(node, cn_array[i].name, &len);
			if(u32)
			    cn_array[i].cell_u64 = of_read_number(u32, 2);
		} else {
			p = of_get_flat_dt_prop(node, cn_array[i].name, &len);
			if(p)
			    strlcpy(cn_array[i].str, p, len);
		}
		cn_array[i].is_valid = 1;
	}

	return 0;
}

void get_dt_cn_prop_u32(const char *name, uint32_t *u32)
{
	int i;
	for (i = 0; i < cn_arr_len; i++) {
		if (cn_array[i].is_valid &&
			!strcmp(name, cn_array[i].name)) {
			*u32 = cn_array[i].cell_u32;
			return;
		}
	}
	printk(KERN_ERR "The %s node have not property value\n", name);
}

void get_dt_cn_prop_u64(const char *name, uint64_t *u64)
{
	int i;
	for (i = 0; i < cn_arr_len; i++) {
		if (cn_array[i].is_valid &&
			!strcmp(name, cn_array[i].name)) {
			*u64 = cn_array[i].cell_u64;
			return;
		}
	}
	printk(KERN_ERR "The %s node have not property value\n", name);
}

void get_dt_cn_prop_str(const char *name, char *value)
{
	int i;
	for (i = 0; i < cn_arr_len; i++) {
		if (cn_array[i].is_valid &&
			!strcmp(name, cn_array[i].name)) {
			strlcpy(value, cn_array[i].str, strlen(cn_array[i].str));
			return;
		}
	}
	printk(KERN_ERR "The %s node have not property value\n", name);
}

#ifdef CONFIG_ANDROID_RAM_CONSOLE
static struct ram_console_platform_data ram_console_pdata = {
	.bootinfo = "UTS_VERSION\n",
};

static struct platform_device ram_console_device = {
	.name = "ram_console",
	.id = -1,
	.dev = {
		.platform_data = &ram_console_pdata,
	}
};
#endif /*CONFIG_ANDROID_RAM_CONSOLE*/

#ifdef CONFIG_LGE_DIAG_ENABLE_SYSFS
static struct platform_device lg_diag_cmd_device = {
	.name = "lg_diag_cmd",
	.id = -1,
	.dev    = {
		.platform_data = 0, //&lg_diag_cmd_pdata
	},
};

void __init lge_add_diag_devices(void)
{
	platform_device_register(&lg_diag_cmd_device);
}
#endif

#ifdef CONFIG_PERSISTENT_TRACER
static struct platform_device persistent_trace_device = {
	.name = "persistent_trace",
	.id = -1,
};
#endif

#ifdef CONFIG_ANDROID_PERSISTENT_RAM
static struct persistent_ram_descriptor lge_pram_descs[] = {
#ifdef CONFIG_ANDROID_RAM_CONSOLE
	{
		.name = "ram_console",
		.size = LGE_RAM_CONSOLE_SIZE,
	},
#endif

#ifdef CONFIG_PERSISTENT_TRACER
	{
		.name = "persistent_trace",
		.size = LGE_RAM_CONSOLE_SIZE,
	},
#endif
};

static struct persistent_ram lge_persist_ram = {
	.size = LGE_PERSISTENT_RAM_SIZE,
	.num_descs = ARRAY_SIZE(lge_pram_descs),
	.descs = lge_pram_descs,
};

void __init lge_add_persist_ram_devices(void)
{
	int ret;
	struct memtype_reserve *mt = &reserve_info->memtype_reserve_table[MEMTYPE_EBI1];

	/* ram->start = 0x7D600000; */
	/* change to variable value to ram->start value */
	lge_persist_ram.start = mt->start - LGE_PERSISTENT_RAM_SIZE;
	pr_info("PERSIST RAM CONSOLE START ADDR : 0x%x\n", lge_persist_ram.start);

	ret = persistent_ram_early_init(&lge_persist_ram);
	if (ret) {
		pr_err("%s: failed to initialize persistent ram\n", __func__);
		return;
	}
}
#endif /*CONFIG_ANDROID_PERSISTENT_RAM*/

void __init lge_reserve(void)
{
#if defined(CONFIG_ANDROID_PERSISTENT_RAM)
	lge_add_persist_ram_devices();
#endif
}

void __init lge_add_persistent_device(void)
{
#ifdef CONFIG_ANDROID_RAM_CONSOLE
	platform_device_register(&ram_console_device);
#ifdef CONFIG_LGE_HANDLE_PANIC
	/* write ram console addr to imem */
	lge_set_ram_console_addr(lge_persist_ram.start,
			LGE_RAM_CONSOLE_SIZE);
#endif
#endif
#ifdef CONFIG_PERSISTENT_TRACER
	platform_device_register(&persistent_trace_device);
#endif

}

#ifdef CONFIG_LGE_QFPROM_INTERFACE
static struct platform_device qfprom_device = {
		.name = "lge-qfprom",
		.id = -1,
};

void __init lge_add_qfprom_devices(void)
{
		platform_device_register(&qfprom_device);
}
#endif

#ifdef CONFIG_LGE_ENABLE_MMC_STRENGTH_CONTROL
static struct platform_device lge_mmc_strength_device = {
	.name = "lge_mmc_strength_driver",
	.id = -1,
};

void __init lge_add_mmc_strength_devices(void)
{
	platform_device_register(&lge_mmc_strength_device);
}
#endif

#ifdef CONFIG_LGE_DIAG_USB_ACCESS_LOCK
static struct platform_device lg_diag_cmd_device = {
	.name = "lg_diag_cmd",
	.id = -1,
	.dev    = {
		.platform_data = 0, /* &lg_diag_cmd_pdata */
	},
};

static int __init lge_diag_devices_init(void)
{
	return platform_device_register(&lg_diag_cmd_device);
}
arch_initcall(lge_diag_devices_init);
#endif

#ifdef CONFIG_LGE_PM_USB_ID
int lge_pm_get_cable_data_from_dt(void *of_node)
{
	int i;
	u32 cable_value[3];
	struct device_node *lge_usb_cable_node;
	struct device_node *lge_usb_cable_each_rev_node;
	bool usb_id_node_exist = 0;

	const char *propname[MAX_CABLE_NUM] = {
		"lge,no-init-cable",
		"lge,cable-56k",
		"lge,cable-100k",
		"lge,cable-130k",
		"lge,cable-180k",
		"lge,cable-200k",
		"lge,cable-220k",
		"lge,cable-270k",
		"lge,cable-330k",
		"lge,cable-620k",
		"lge,cable-910k",
		"lge,cable-none"
	};

	if (cable_type_defined) {
		pr_info("Cable type is already defined\n");
		return 0;
	}

	lge_usb_cable_node = of_find_compatible_node(NULL, NULL, "lge,usb-id");
	if (!lge_usb_cable_node) {
		pr_err("< Error > USB ID Node \"lge,usb-id\" is not exist.\n");
		goto usb_id_error;
	}


	for_each_child_of_node(lge_usb_cable_node,lge_usb_cable_each_rev_node){
		if(of_device_is_available(lge_usb_cable_each_rev_node)
			&& of_device_is_available_revision(lge_usb_cable_each_rev_node)){
			usb_id_node_exist = 1;
			break;
		}
	}

	if (!usb_id_node_exist) {
		pr_err("< Error > USB ID the compatible setting of HW rev: %d is not exist.\n",lge_get_board_revno());
		goto usb_id_error;
	}

	for (i = 0 ; i < MAX_CABLE_NUM ; i++) {
			of_property_read_u32_array(lge_usb_cable_each_rev_node, propname[i],
					cable_value, 3);
			pm_acc_cable_type_data[i].threshhold = cable_value[0];
			pm_acc_cable_type_data[i].type = i;
			pm_acc_cable_type_data[i].ta_ma = cable_value[1];
			pm_acc_cable_type_data[i].usb_ma = cable_value[2];
			pr_info(">>> Rev : %d / %s ADC high threshold : %d mV\n",lge_get_board_revno(), propname[i], cable_value[0]);
	}
	cable_type_defined = 1;
	return 0;
	usb_id_error :
	return 1;
}

int lge_pm_get_cable_info(struct chg_cable_info *cable_info, int32_t usb_id_val)
{
	char *type_str[] = {
		"NOT INIT", "56K", "100K",
		"130K", "180K", "200K", "220K", "270K", "330K", "620K",
		"910K", "OPEN"
	};

	struct chg_cable_info *info = cable_info;
	struct chg_cable_info_table *table;
	int table_size = ARRAY_SIZE(pm_acc_cable_type_data);
	int i;

	if (!info) {
		pr_err("%s: invalid info parameters\n",__func__);
		return -1;
	}

	if (!cable_type_defined) {
		pr_info("%s: LGE USB Cable Type is not defined yet\n",__func__);
		return -1;
	}

	info->cable_type = NO_INIT_CABLE;
	info->ta_ma = C_NO_INIT_TA_MA;
	info->usb_ma = C_NO_INIT_USB_MA;

	/* assume: adc value must be existed in ascending order */
	for (i = 0; i < table_size; i++) {
		table = &pm_acc_cable_type_data[i];

		if (usb_id_val <= table->threshhold) {
			info->cable_type = table->type;
			info->ta_ma = table->ta_ma;
			info->usb_ma = table->usb_ma;
			break;
		}
	}

	pr_info("\n\n[PM]Cable detected: %d(%s)(%d, %d)\n\n",
			usb_id_val, type_str[info->cable_type],
			info->ta_ma, info->usb_ma);
	return 0;
}

/* Belows are for using in interrupt context */
static struct chg_cable_info lge_cable_info;

acc_cable_type lge_pm_get_cable_type(void)
{
	return lge_cable_info.cable_type;
}

unsigned lge_pm_get_ta_current(void)
{
	return lge_cable_info.ta_ma;
}

unsigned lge_pm_get_usb_current(void)
{
	return lge_cable_info.usb_ma;
}

/* This must be invoked in process context */
void lge_pm_read_cable_info(int32_t usb_id_val)
{
	lge_cable_info.cable_type = NO_INIT_CABLE;
	lge_cable_info.ta_ma = C_NO_INIT_TA_MA;
	lge_cable_info.usb_ma = C_NO_INIT_USB_MA;

	lge_pm_get_cable_info(&lge_cable_info, usb_id_val);
}

void lge_pm_set_usb_cable_to_minimum(void){

	lge_cable_info.cable_type = NO_INIT_CABLE;
#if 1 // Set 0mA when cable detection failed. We should judge what is better.
	lge_cable_info.ta_ma = C_NO_INIT_TA_MA;
	lge_cable_info.usb_ma = C_NO_INIT_USB_MA;
#else
	lge_cable_info.ta_ma = C_NONE_USB_MA;
	lge_cable_info.usb_ma = C_NONE_USB_MA;
#endif
	pr_info("\n\n[PM]Cable set the minimum current: (%d, %d)\n\n",
			lge_cable_info.ta_ma, lge_cable_info.usb_ma);
}


#endif


#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
struct power_supply *ac_psy;
struct power_supply *usb_psy;

#if defined(CONFIG_MACH_MSM8926_X5_VZW) || defined(CONFIG_MACH_MSM8926_X3C_TRF_US) || \
	defined(CONFIG_MACH_MSM8926_X3N_OPEN_EU) || defined(CONFIG_MACH_MSM8926_X3N_GLOBAL_COM) || defined(CONFIG_MACH_MSM8926_F70N_GLOBAL_COM) || defined(CONFIG_MACH_MSM8926_F70N_GLOBAL_AME) || defined(CONFIG_MACH_MSM8926_X3N_GLOBAL_SCA) || \
	defined(CONFIG_MACH_MSM8926_X3_TRF_US) || defined(CONFIG_MACH_MSM8926_X3N_KR) || defined(CONFIG_MACH_MSM8926_F70N_KR) || defined(CONFIG_MACH_MSM8926_F70_GLOBAL_COM) || defined(CONFIG_MACH_MSM8926_E2_SPR_US)
int lge_battery_info = BATT_ID_DS2704_N;// BATT_ID_UNKNOWN;
#else
int lge_battery_info = BATT_ID_UNKNOWN;
#endif

bool is_lge_battery_valid(void)
{
	union power_supply_propval ac_val = {0,}, usb_val = {0,};
#ifdef CONFIG_LGE_PM_BATTERY_4_2VOLT
	return true;
#else
	if (!ac_psy)
		ac_psy = power_supply_get_by_name("ac");
	if (!usb_psy)
		usb_psy = power_supply_get_by_name("usb");
	if (!ac_psy || !usb_psy)
		return false;
	ac_psy->get_property(ac_psy, POWER_SUPPLY_PROP_PRESENT, &ac_val);
	usb_psy->get_property(usb_psy, POWER_SUPPLY_PROP_PRESENT, &usb_val);
	if((lge_pm_get_cable_type()== CABLE_56K ||
		lge_pm_get_cable_type()== CABLE_130K ||
		lge_pm_get_cable_type()== CABLE_910K) &&
		(ac_val.intval || usb_val.intval))
		return true;

	if (lge_battery_info == BATT_ID_DS2704_N ||
		lge_battery_info == BATT_ID_DS2704_L ||
		lge_battery_info == BATT_ID_DS2704_C ||
		lge_battery_info == BATT_ID_ISL6296_N ||
		lge_battery_info == BATT_ID_ISL6296_L ||
#ifdef CONFIG_LGE_PM_BATTERY_ID_RANIX_SILICON_WORKS
		lge_battery_info == BATT_ID_RA4301_VC0 ||
		lge_battery_info == BATT_ID_RA4301_VC1 ||
		lge_battery_info == BATT_ID_RA4301_VC2 ||
		lge_battery_info == BATT_ID_SW3800_VC0 ||
		lge_battery_info == BATT_ID_SW3800_VC1 ||
		lge_battery_info == BATT_ID_SW3800_VC2 ||
#endif
		lge_battery_info == BATT_ID_ISL6296_C)
		return true;

#if defined(CONFIG_MACH_MSM8926_G2M_OPEN) || defined(CONFIG_MACH_MSM8926_G2M_VDF) || defined(CONFIG_MACH_MSM8926_G2M_GLOBAL) || defined(CONFIG_MACH_MSM8926_G2M_KR) || \
	defined(CONFIG_MACH_MSM8926_G2MH_SBM)
	if( lge_get_board_revno() < HW_REV_B ) return true;
#endif

	return false;


#endif //CONFIG_LGE_PM_BATTERY_4_2VOLT
}
//EXPORT_SYMBOL(is_lge_battery_valid);

int read_lge_battery_id(void)
{
		return lge_battery_info;
}
//EXPORT_SYMBOL(read_lge_battery_id);

static int __init battery_information_setup(char *batt_info)
{
        if(!strcmp(batt_info, "DS2704_N"))
                lge_battery_info = BATT_ID_DS2704_N;
        else if(!strcmp(batt_info, "DS2704_L"))
                lge_battery_info = BATT_ID_DS2704_L;
        else if(!strcmp(batt_info, "DS2704_C"))
                lge_battery_info = BATT_ID_DS2704_C;
        else if(!strcmp(batt_info, "ISL6296_N"))
                lge_battery_info = BATT_ID_ISL6296_N;
        else if(!strcmp(batt_info, "ISL6296_L"))
                lge_battery_info = BATT_ID_ISL6296_L;
        else if(!strcmp(batt_info, "ISL6296_C"))
                lge_battery_info = BATT_ID_ISL6296_C;
#ifdef CONFIG_LGE_PM_BATTERY_ID_RANIX_SILICON_WORKS
        else if(!strcmp(batt_info, "RA4301_VC0"))
                lge_battery_info = BATT_ID_RA4301_VC0;
        else if(!strcmp(batt_info, "RA4301_VC1"))
                lge_battery_info = BATT_ID_RA4301_VC1;
        else if(!strcmp(batt_info, "RA4301_VC2"))
                lge_battery_info = BATT_ID_RA4301_VC2;
        else if(!strcmp(batt_info, "SW3800_VC0"))
                lge_battery_info = BATT_ID_SW3800_VC0;
        else if(!strcmp(batt_info, "SW3800_VC1"))
                lge_battery_info = BATT_ID_SW3800_VC1;
        else if(!strcmp(batt_info, "SW3800_VC2"))
                lge_battery_info = BATT_ID_SW3800_VC2;
#endif
        else
                lge_battery_info = BATT_ID_UNKNOWN;

        printk(KERN_INFO "Battery : %s %d\n", batt_info, lge_battery_info);

        return 1;
}
__setup("lge.battid=", battery_information_setup);
#endif
#if defined(CONFIG_LGE_KSWITCH)
static int kswitch_status;
#endif


/* setting whether uart console is enalbed or disabled */
static unsigned int uart_console_mode = 1;  /* Alway Off */

unsigned int lge_get_uart_mode(void)
{

#ifdef CONFIG_LGE_KSWITCH
if ((kswitch_status & LGE_KSWITCH_UART_DISABLE) >> 3)
	 uart_console_mode = 0;
#endif

	return uart_console_mode;
}

void lge_set_uart_mode(unsigned int um)
{
	uart_console_mode = um;
}

static int __init lge_uart_mode(char *uart_mode)
{
	if (!strncmp("enable", uart_mode, 6)) {
		printk(KERN_INFO"UART CONSOLE : enable\n");
		lge_set_uart_mode((UART_MODE_ALWAYS_ON_BMSK | UART_MODE_EN_BMSK)
				& ~UART_MODE_ALWAYS_OFF_BMSK);
	} else if (!strncmp("detected", uart_mode, 8)) {
		printk(KERN_INFO"UART CONSOLE : detected\n");
		lge_set_uart_mode(UART_MODE_EN_BMSK & ~UART_MODE_ALWAYS_OFF_BMSK);
	} else {
		printk(KERN_INFO"UART CONSOLE : disable\n");
	}

	return 1;
}
__setup("uart_console=", lge_uart_mode);

#ifdef CONFIG_LGE_PM_CHARGING_CHARGERLOGO
int lge_boot_mode_for_touch = (int)LGE_BOOT_MODE_NORMAL;
#endif

static enum lge_boot_mode_type lge_boot_mode = LGE_BOOT_MODE_NORMAL;
int __init lge_boot_mode_init(char *s)
{
	if (!strcmp(s, "charger"))
		lge_boot_mode = LGE_BOOT_MODE_CHARGER;
#ifdef CONFIG_LGE_PM_CHARGING_CHARGERLOGO
	else if(!strcmp(s, "chargerlogo"))
		lge_boot_mode = LGE_BOOT_MODE_CHARGERLOGO;
#endif
	else if (!strcmp(s, "qem_56k"))
		lge_boot_mode = LGE_BOOT_MODE_QEM_56K;
	else if (!strcmp(s, "qem_130k"))
		lge_boot_mode = LGE_BOOT_MODE_QEM_130K;
	else if (!strcmp(s, "qem_910k"))
		lge_boot_mode = LGE_BOOT_MODE_QEM_910K;
	else if (!strcmp(s, "pif_56k"))
		lge_boot_mode = LGE_BOOT_MODE_PIF_56K;
	else if (!strcmp(s, "pif_130k"))
		lge_boot_mode = LGE_BOOT_MODE_PIF_130K;
	else if (!strcmp(s, "pif_910k"))
		lge_boot_mode = LGE_BOOT_MODE_PIF_910K;
	else
		lge_boot_mode = LGE_BOOT_MODE_NORMAL;

#ifdef CONFIG_LGE_PM_CHARGING_CHARGERLOGO
        lge_boot_mode_for_touch = (int)lge_boot_mode;
#endif

	printk("ANDROID BOOT MODE : %d %s\n", lge_boot_mode, s);
	return 1;
}

__setup("androidboot.mode=", lge_boot_mode_init);

enum lge_boot_mode_type lge_get_boot_mode(void)
{
    return lge_boot_mode;
}

int lge_get_factory_boot(void)
{
    int res;

    /*   if boot mode is factory,
     *   cable must be factory cable.
     */
    switch (lge_boot_mode) {
        case LGE_BOOT_MODE_QEM_56K:
        case LGE_BOOT_MODE_QEM_130K:
        case LGE_BOOT_MODE_QEM_910K:
        case LGE_BOOT_MODE_PIF_56K:
        case LGE_BOOT_MODE_PIF_130K:
        case LGE_BOOT_MODE_PIF_910K:
            res = 1;
            break;
        default:
            res = 0;
            break;
    }
    return res;
}

static enum lge_boot_cable_type lge_boot_cable = LGE_BOOT_NO_INIT_CABLE;
int __init lge_boot_cable_type_init(char *s)
{
	lge_boot_cable = 0;

	for (;; s++) {
		switch (*s) {
			case '0' ... '9':
				lge_boot_cable = 10*lge_boot_cable+(*s-'0');
				break;
			default:
				return 1;
		}
	}
}
__setup("bootcable.type=", lge_boot_cable_type_init);

enum lge_boot_cable_type lge_get_boot_cable_type(void)
{
	return lge_boot_cable;
}

static enum lge_laf_mode_type lge_laf_mode = LGE_LAF_MODE_NORMAL;

int __init lge_laf_mode_init(char *s)
{
    if (strcmp(s, ""))
        lge_laf_mode = LGE_LAF_MODE_LAF;

    return 1;
}
__setup("androidboot.laf=", lge_laf_mode_init);

enum lge_laf_mode_type lge_get_laf_mode(void)
{
    return lge_laf_mode;
}


/* for board revision */
static hw_rev_type lge_bd_rev = HW_REV_A;

/* CAUTION: These strings are come from LK. */
#if defined(CONFIG_MACH_MSM8926_X3N_OPEN_EU) || defined(CONFIG_MACH_MSM8926_X3N_GLOBAL_COM) || defined(CONFIG_MACH_MSM8926_F70N_GLOBAL_COM) || defined(CONFIG_MACH_MSM8926_F70N_GLOBAL_AME) || defined(CONFIG_MACH_MSM8926_F70_GLOBAL_COM) || defined(CONFIG_MACH_MSM8926_X3N_GLOBAL_SCA) || \
	defined(CONFIG_MACH_MSM8926_X3_TRF_US) || defined(CONFIG_MACH_MSM8926_X3N_KR) || defined(CONFIG_MACH_MSM8926_F70N_KR)
char *rev_str[] = {"rev_0", "rev_a", "rev_a2", "rev_b", "rev_b2",
	"rev_c", "rev_10", "rev_11", "revserved"};
#else
char *rev_str[] = {"rev_0", "rev_a", "rev_b", "rev_c", "rev_d",
	"rev_e", "rev_10", "rev_11", "revserved"};
#endif
static int __init board_revno_setup(char *rev_info)
{
	int i;

	for (i = 0; i < HW_REV_MAX; i++) {
		if (!strncmp(rev_info, rev_str[i], 6)) {
			lge_bd_rev = (hw_rev_type) i;
			/* it is defined externally in <asm/system_info.h> */
			system_rev = lge_bd_rev;
			break;
		}
	}

	printk(KERN_INFO "BOARD : LGE %s \n", rev_str[lge_bd_rev]);
	return 1;
}
__setup("lge.rev=", board_revno_setup);

hw_rev_type lge_get_board_revno(void)
{
    return lge_bd_rev;
}

#if defined(CONFIG_LCD_KCAL)
int g_kcal_r = 255;
int g_kcal_g = 255;
int g_kcal_b = 255;

extern int kcal_set_values(int kcal_r, int kcal_g, int kcal_b);
static int __init display_kcal_setup(char *kcal)
{
	char vaild_k = 0;
	int kcal_r = 255;
	int kcal_g = 255;
	int kcal_b = 255;

	sscanf(kcal, "%d|%d|%d|%c", &kcal_r, &kcal_g, &kcal_b, &vaild_k );
	pr_info("kcal is %d|%d|%d|%c\n", kcal_r, kcal_g, kcal_b, vaild_k);
	printk("display_kcal_setup ==> kcal_r=[%d], kcal_g=[%d], kcal_b=[%d], vaild_k=[%c]\n", kcal_r, kcal_g, kcal_b, vaild_k);

	if (vaild_k != 'K') {
		pr_info("kcal not calibrated yet : %d\n", vaild_k);
		kcal_r = kcal_g = kcal_b = 255;
		pr_info("set to default : %d\n", kcal_r);
	}

	kcal_set_values(kcal_r, kcal_g, kcal_b);
	return 1;
}
__setup("lge.kcal=", display_kcal_setup);
#endif

#ifdef CONFIG_USB_G_LGE_ANDROID
static int get_factory_cable(void)
{
    /* It is factory cable */
    switch(lge_pm_get_cable_type()) {
        case CABLE_56K:
            return LGEUSB_FACTORY_56K;

        case CABLE_130K:
            return LGEUSB_FACTORY_130K;

        case CABLE_910K:
            return LGEUSB_FACTORY_910K;

        default:
            break;
    }

    /* if boot mode is factory, cable must be factory cable. */
    switch (lge_get_boot_mode()) {
        case LGE_BOOT_MODE_QEM_56K:
        case LGE_BOOT_MODE_PIF_56K:
            return LGEUSB_FACTORY_56K;

        case LGE_BOOT_MODE_QEM_130K:
        case LGE_BOOT_MODE_PIF_130K:
            return LGEUSB_FACTORY_130K;

        case LGE_BOOT_MODE_QEM_910K:
        case LGE_BOOT_MODE_PIF_910K:
            return LGEUSB_FACTORY_910K;

        default:
            break;
    }

    /* It is normal cable */
    return LGEUSB_NORMAL;
}

struct lge_android_usb_platform_data lge_android_usb_pdata = {
    .vendor_id = 0x1004,
    .factory_pid = 0x6000,
    .iSerialNumber = 0,
    .product_name = "LGE Android Phone",
    .manufacturer_name = "LG Electronics Inc.",
    .factory_composition = "acm,diag",
    .get_factory_cable = get_factory_cable,
};

struct platform_device lge_android_usb_device = {
    .name = "lge_android_usb",
    .id = -1,
    .dev = {
        .platform_data = &lge_android_usb_pdata,
    },
};

void __init lge_android_usb_init(void)
{
    platform_device_register(&lge_android_usb_device);
}
#endif /* CONFIG_USB_G_LGE_ANDROID */

#ifdef CONFIG_LGE_CRASH_FOOTPRINT
static unsigned long int lge_bootreason = 0;
int __init lge_crash_footprint_init(char *s)
{
    if (s != NULL) {
        s += 2;
        lge_bootreason = simple_strtoul(s, NULL, 16);
        //pr_info("crash.footprint: (%s)%lx\n", s, lge_bootreason);
    }

    return 1;
}
__setup("lge.bootreason=", lge_crash_footprint_init);

unsigned long int lge_get_crash_footprint(void)
{
    return lge_bootreason;
}
#endif

#if defined(CONFIG_MACH_MSM8X10_W5) || defined(CONFIG_MACH_MSM8X10_W65)
// 0 is primary
// 1 is secondary
// 4 is unknown
int lge_lcd_id = 4;
static int __init lcd_maker_id_setup(char *lcd_maker_id)
{
        if (!strcmp(lcd_maker_id, "lcd_primary"))
        {
        	lge_lcd_id = 0;
        }
        else if(!strcmp(lcd_maker_id, "lcd_secondary"))
        {
        	lge_lcd_id = 1;
        }
        else
        {
            lge_lcd_id = 4;
        }

        printk(KERN_INFO "lcd_maker_id : %s %d\n", lcd_maker_id, lge_lcd_id);

        return 1;
}
__setup("lcd_maker_id=", lcd_maker_id_setup);
#endif
#ifdef CONFIG_LGE_QSDL_SUPPORT
static struct lge_qsdl_platform_data lge_qsdl_pdata = {
	.oneshot_read = 0,
	.using_uevent = 0
};

static struct platform_device lge_qsdl_device = {
	.name = LGE_QSDL_DEV_NAME,
	.id = -1,
	.dev = {
		.platform_data = &lge_qsdl_pdata,
	}
};

void __init lge_add_qsdl_device(void)
{
	platform_device_register(&lge_qsdl_device);
}
#endif /* CONFIG_LGE_QSDL_SUPPORT */

#if defined(CONFIG_LGE_KSWITCH)
 static int atoi(const char *name)
{
	int val = 0;
	for (;; name++) {
		switch (*name) {
		case '0' ... '9':
			val = 10*val+(*name-'0');
			break;
		default:
		 return val;
		}
	}
}

static int __init kswitch_setup(char *value)
{
	kswitch_status = atoi(value);
	if (kswitch_status < 0)
		kswitch_status = 0;

	printk(KERN_INFO "[KSwitch] %d \n", kswitch_status);
		 return 1;
}
__setup("kswitch=", kswitch_setup);
int lge_get_kswitch_status(void)
{
	return kswitch_status;
}
#endif
#ifdef CONFIG_LGE_LCD_OFF_DIMMING
static int lge_boot_reason = -1; /*  undefined for error checking */
static int __init lge_check_bootreason(char *reason)
{
	int ret = 0;

	/*  handle corner case of kstrtoint */
	if (!strcmp(reason, "0xffffffff")) {
		lge_boot_reason = 0xffffffff;
		return 1;
	}

	ret = kstrtoint(reason, 16, &lge_boot_reason);
	if (!ret)
		printk(KERN_INFO "LGE REBOOT REASON: %x\n", lge_boot_reason);
	else
		printk(KERN_INFO "LGE REBOOT REASON: Couldn't get bootreason - %d\n",
				ret);

	return 1;
}
__setup("lge.bootreason=", lge_check_bootreason);

int lge_get_bootreason(void)
{
	return lge_boot_reason;
}
#endif
