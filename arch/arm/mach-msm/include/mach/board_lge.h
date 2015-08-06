#ifndef __ASM_ARCH_MSM_BOARD_LGE_H
#define __ASM_ARCH_MSM_BOARD_LGE_H

#if defined(CONFIG_MACH_MSM8926_X3N_OPEN_EU) || defined(CONFIG_MACH_MSM8926_X3N_GLOBAL_SCA) || defined(CONFIG_MACH_MSM8926_X3N_GLOBAL_COM) || defined(CONFIG_MACH_MSM8926_F70N_GLOBAL_COM) || defined(CONFIG_MACH_MSM8926_F70N_GLOBAL_AME) || \
	defined(CONFIG_MACH_MSM8926_F70_GLOBAL_COM) || defined(CONFIG_MACH_MSM8926_X3_TRF_US) || defined(CONFIG_MACH_MSM8926_X3N_KR) || defined(CONFIG_MACH_MSM8926_F70N_KR)
typedef enum {
	HW_REV_0 = 0,
	HW_REV_A,
	HW_REV_A2,
	HW_REV_B,
	HW_REV_B2,
	HW_REV_C,
	HW_REV_1_0,
	HW_REV_1_1,
	HW_REV_MAX
} hw_rev_type;
#else
typedef enum {
	HW_REV_0 = 0,
	HW_REV_A,
	HW_REV_B,
	HW_REV_C,
	HW_REV_D,
	HW_REV_E,
	HW_REV_1_0,
	HW_REV_1_1,
	HW_REV_MAX
} hw_rev_type;
#endif

extern char *rev_str[];
hw_rev_type lge_get_board_revno(void);

#ifdef CONFIG_LGE_PM_USB_ID

#define MAX_LGE_CABLE_RETRY_COUNT 10

typedef enum {
	NO_INIT_CABLE = 0,
	CABLE_56K,
	CABLE_100K,
	CABLE_130K,
	CABLE_180K,
	CABLE_200K,
	CABLE_220K,
	CABLE_270K,
	CABLE_330K,
	CABLE_620K,
	CABLE_910K,
	CABLE_NONE
} acc_cable_type;

struct chg_cable_info {
	acc_cable_type cable_type;
	unsigned ta_ma;
	unsigned usb_ma;
};

int lge_pm_get_cable_data_from_dt(void *of_node);
void lge_pm_read_cable_info(int32_t);
void lge_pm_set_usb_cable_to_minimum(void);
int lge_pm_get_cable_info(struct chg_cable_info *, int32_t);

acc_cable_type lge_pm_get_cable_type(void);
unsigned lge_pm_get_ta_current(void);
unsigned lge_pm_get_usb_current(void);

#if defined(CONFIG_PRE_SELF_DIAGNOSIS)
int lge_pre_self_diagnosis(char *drv_bus_code, int func_code, char *dev_code, char *drv_code, int errno);
#endif

enum lge_boot_cable_type lge_get_boot_cable_type(void);
#endif

#ifdef CONFIG_LGE_DIAG_ENABLE_SYSFS
void __init lge_add_diag_devices(void);
#endif

#ifdef CONFIG_LGE_PM_FACTORY_PSEUDO_BATTERY
struct pseudo_batt_info_type {
	int mode;
	int id;
	int therm;
	int temp;
	int volt;
	int capacity;
	int charging;
};

#endif

#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
enum {
	BATT_ID_UNKNOWN = 0,
	BATT_ID_DS2704_N,
	BATT_ID_DS2704_L,
	BATT_ID_DS2704_C,
	BATT_ID_ISL6296_N,
	BATT_ID_ISL6296_L,
	BATT_ID_ISL6296_C,
#ifdef CONFIG_LGE_PM_BATTERY_ID_RANIX_SILICON_WORKS
	BATT_ID_RA4301_VC0,
	BATT_ID_RA4301_VC1,
	BATT_ID_RA4301_VC2,
	BATT_ID_SW3800_VC0,
	BATT_ID_SW3800_VC1,
	BATT_ID_SW3800_VC2,
#endif
};
bool is_lge_battery_valid(void);
int read_lge_battery_id(void);
extern int lge_battery_info;
#endif

enum lge_boot_mode_type {
    LGE_BOOT_MODE_NORMAL = 0,
    LGE_BOOT_MODE_CHARGER,
#ifdef CONFIG_LGE_PM_CHARGING_CHARGERLOGO
    LGE_BOOT_MODE_CHARGERLOGO,
#endif
    LGE_BOOT_MODE_QEM_56K,
    LGE_BOOT_MODE_QEM_130K,
    LGE_BOOT_MODE_QEM_910K,
    LGE_BOOT_MODE_PIF_56K,
    LGE_BOOT_MODE_PIF_130K,
    LGE_BOOT_MODE_PIF_910K,
};

enum lge_boot_mode_type lge_get_boot_mode(void);
int lge_get_factory_boot(void);

/* from cable_type */
enum lge_boot_cable_type {
	LGE_BOOT_LT_CABLE_56K = 6,
	LGE_BOOT_LT_CABLE_130K,
	LGE_BOOT_USB_CABLE_400MA,
	LGE_BOOT_USB_CABLE_DTC_500MA,
	LGE_BOOT_ABNORMAL_USB_CABLE_400MA,
	LGE_BOOT_LT_CABLE_910K,
	LGE_BOOT_NO_INIT_CABLE,
};

#if defined(CONFIG_PRE_SELF_DIAGNOSIS)
int lge_pre_self_diagnosis(char *drv_bus_code, int func_code, char *dev_code, char *drv_code, int errno);
#endif

#if defined(CONFIG_LCD_KCAL)
struct kcal_data {
	int red;
	int green;
	int blue;
};

struct kcal_platform_data {
	int (*set_values) (int r, int g, int b);
	int (*get_values) (int *r, int *g, int *b);
	int (*refresh_display) (void);
};
#endif

#if defined(CONFIG_PRE_SELF_DIAGNOSIS)
struct pre_selfd_platform_data {
	int (*set_values) (int r, int g, int b);
	int (*get_values) (int *r, int *g, int *b);
};
#endif

enum lge_laf_mode_type {
    LGE_LAF_MODE_NORMAL = 0,
    LGE_LAF_MODE_LAF,
};

enum lge_laf_mode_type lge_get_laf_mode(void);

enum cn_prop_type {
	CELL_U32 = 0,
	CELL_U64,
	STRING,
};

int __init lge_init_dt_scan_chosen(unsigned long node, const char *uname,
								  int depth, void *data);

void get_dt_cn_prop_str(const char *name, char *value);
void get_dt_cn_prop_u64(const char *name, uint64_t *u64);
void get_dt_cn_prop_u32(const char *name, uint32_t *u32);

#define UART_MODE_ALWAYS_OFF_BMSK   BIT(0)
#define UART_MODE_ALWAYS_ON_BMSK    BIT(1)
#define UART_MODE_INIT_BMSK         BIT(2)
#define UART_MODE_EN_BMSK           BIT(3)

extern unsigned int lge_get_uart_mode(void);
extern void lge_set_uart_mode(unsigned int um);

#if defined(CONFIG_LGE_CRASH_FOOTPRINT)
extern unsigned long int lge_get_crash_footprint(void);
#endif//
#ifdef CONFIG_ANDROID_PERSISTENT_RAM
#define LGE_PERSISTENT_RAM_SIZE (SZ_256K)
#endif

#if defined(CONFIG_ANDROID_RAM_CONSOLE)
#define LGE_RAM_CONSOLE_SIZE (128 * SZ_1K * 2)
#endif

void __init lge_reserve(void);
void __init lge_add_persistent_device(void);


#if defined(CONFIG_ANDROID_PERSISTENT_RAM)
void __init lge_add_persist_ram_devices(void);
#endif

#if defined(CONFIG_LCD_KCAL)
void __init lge_add_lcd_kcal_devices(void);
#endif

#ifdef CONFIG_USB_G_LGE_ANDROID
void __init lge_android_usb_init(void);
#endif

#ifdef CONFIG_LGE_QFPROM_INTERFACE
void __init lge_add_qfprom_devices(void);
#endif

#ifdef CONFIG_LGE_ENABLE_MMC_STRENGTH_CONTROL
void __init lge_add_mmc_strength_devices(void);

#endif

#ifdef CONFIG_LGE_QSDL_SUPPORT
void __init lge_add_qsdl_device(void);
#endif

#if defined(CONFIG_LGE_KSWITCH)
#define LGE_KSWITCH_UART_DISABLE     0x1 << 3
int lge_get_kswitch_status(void);
#endif

#ifdef CONFIG_LGE_LCD_OFF_DIMMING
int lge_get_bootreason(void);
#endif

#endif

