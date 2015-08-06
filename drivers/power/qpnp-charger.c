/* Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/module.h>

#if defined (CONFIG_LGE_PM_CHARGING_EXTERNAL_OVP)
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#endif

#include <linux/slab.h>
#include <linux/err.h>
#include <linux/spmi.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/radix-tree.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/power_supply.h>
#include <linux/bitops.h>
#include <linux/ratelimit.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/of_batterydata.h>
#include <linux/qpnp-revid.h>
#include <linux/android_alarm.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/qpnp/pin.h>

#include <linux/usb/otg.h>
#include <linux/usb/msm_hsusb.h>

#ifdef CONFIG_LGE_USING_CHG_LED
#include <linux/qpnp/pwm.h>
#include <mach/board_lge.h>
#endif

/* LGE_CHANGE_S: Cable Detect & Current Set */
#ifdef CONFIG_LGE_PM
#include "../../arch/arm/mach-msm/smd_private.h"
#include <mach/board_lge.h>
#endif
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
#if defined(CONFIG_MACH_MSM8926_X5_SPR) || defined(CONFIG_MACH_MSM8X10_W5C_SPR_US) || \
    defined(CONFIG_MACH_MSM8926_JAGC_SPR)
#include <mach/lge_charging_scenario_sprint.h>
#else
#include <mach/lge_charging_scenario.h>
#endif
#define MONITOR_BATTEMP_POLLING_PERIOD          (60*HZ)
#endif
#ifdef CONFIG_MAX17048_FUELGAUGE
#include <linux/max17048_battery.h>
#endif
/* LGE_CHANGE_E */

/* LGE_CHANGE_S */
#ifdef CONFIG_LGE_PM_CHARGING_EXTERNAL_OVP
#include <linux/of_gpio.h>
#endif
/* LGE_CHANGE_E */

#ifdef CONFIG_LGE_PM_PWR_KEY_FOR_CHG_LOGO
#include <linux/input.h>
#include <mach/board_lge.h>
#endif


/* Interrupt offsets */
#define INT_RT_STS(base)			(base + 0x10)
#define INT_SET_TYPE(base)			(base + 0x11)
#define INT_POLARITY_HIGH(base)			(base + 0x12)
#define INT_POLARITY_LOW(base)			(base + 0x13)
#define INT_LATCHED_CLR(base)			(base + 0x14)
#define INT_EN_SET(base)			(base + 0x15)
#define INT_EN_CLR(base)			(base + 0x16)
#define INT_LATCHED_STS(base)			(base + 0x18)
#define INT_PENDING_STS(base)			(base + 0x19)
#define INT_MID_SEL(base)			(base + 0x1A)
#define INT_PRIORITY(base)			(base + 0x1B)

/* Peripheral register offsets */
#define CHGR_CHG_OPTION				0x08
#define CHGR_ATC_STATUS				0x0A
#define CHGR_VBAT_STATUS			0x0B
#define CHGR_IBAT_BMS				0x0C
#define CHGR_IBAT_STS				0x0D
#define CHGR_VDD_MAX				0x40
#define CHGR_VDD_SAFE				0x41
#define CHGR_VDD_MAX_STEP			0x42
#ifdef CONFIG_LGE_PM_NEED_TO_MONITORING_QCT_PATCH	/* Removing 100mV Drop in VDDMAX */
#define CHGR_VDDMAX_GSM_ADJ			0x43
#endif
#define CHGR_IBAT_MAX				0x44
#define CHGR_IBAT_SAFE				0x45
#define CHGR_VIN_MIN				0x47
#define CHGR_VIN_MIN_STEP			0x48
#define CHGR_CHG_CTRL				0x49
#define CHGR_CHG_FAILED				0x4A
#define CHGR_ATC_CTRL				0x4B
#define CHGR_ATC_FAILED				0x4C
#define CHGR_VBAT_TRKL				0x50
#define CHGR_VBAT_WEAK				0x52
#define CHGR_IBAT_ATC_A				0x54
#define CHGR_IBAT_ATC_B				0x55
#define CHGR_IBAT_TERM_CHGR			0x5B
#define CHGR_IBAT_TERM_BMS			0x5C
#define CHGR_VBAT_DET				0x5D
#define CHGR_TTRKL_MAX_EN			0x5E
#define CHGR_TTRKL_MAX				0x5F
#define CHGR_TCHG_MAX_EN			0x60
#define CHGR_TCHG_MAX				0x61
#define CHGR_CHG_WDOG_TIME			0x62
#define CHGR_CHG_WDOG_DLY			0x63
#define CHGR_CHG_WDOG_PET			0x64
#define CHGR_CHG_WDOG_EN			0x65
#define CHGR_IR_DROP_COMPEN			0x67
#define CHGR_I_MAX_REG			0x44
#define CHGR_USB_USB_SUSP			0x47
#define CHGR_USB_USB_OTG_CTL			0x48
#define CHGR_USB_ENUM_T_STOP			0x4E
#define CHGR_USB_TRIM				0xF1
#define CHGR_CHG_TEMP_THRESH			0x66
#define CHGR_BAT_IF_PRES_STATUS			0x08
#define CHGR_STATUS				0x09
#define CHGR_BAT_IF_VCP				0x42
#define CHGR_BAT_IF_BATFET_CTRL1		0x90
#define CHGR_BAT_IF_BATFET_CTRL4		0x93
#define CHGR_BAT_IF_SPARE			0xDF
#define CHGR_MISC_BOOT_DONE			0x42
#define CHGR_BUCK_PSTG_CTRL			0x73
#define CHGR_BUCK_COMPARATOR_OVRIDE_1		0xEB
#define CHGR_BUCK_COMPARATOR_OVRIDE_2		0xEC
#define CHGR_BUCK_COMPARATOR_OVRIDE_3		0xED
#define CHG_OVR0				0xED
#define CHG_TRICKLE_CLAMP			0xE3
#define CHGR_BUCK_BCK_VBAT_REG_MODE		0x74
#define MISC_REVISION2				0x01
#define USB_OVP_CTL				0x42
#define USB_CHG_GONE_REV_BST			0xED
#define BUCK_VCHG_OV				0x77
#define BUCK_TEST_SMBC_MODES			0xE6
#define BUCK_CTRL_TRIM1				0xF1
#define BUCK_CTRL_TRIM3				0xF3
#define SEC_ACCESS				0xD0
#define BAT_IF_VREF_BAT_THM_CTRL		0x4A
#define BAT_IF_BPD_CTRL				0x48
#define BOOST_VSET				0x41
#define BOOST_ENABLE_CONTROL			0x46
#define COMP_OVR1				0xEA
#define BAT_IF_BTC_CTRL				0x49
#define USB_OCP_THR				0x52
#define USB_OCP_CLR				0x53
#define BAT_IF_TEMP_STATUS			0x09
#define BOOST_ILIM				0x78
#define USB_SPARE				0xDF
#define DC_COMP_OVR1				0xE9
#define CHGR_COMP_OVR1				0xEE
#define USB_CHGPTH_CTL				0x40
#define REG_OFFSET_PERP_SUBTYPE			0x05

/* SMBB peripheral subtype values */
#define SMBB_CHGR_SUBTYPE			0x01
#define SMBB_BUCK_SUBTYPE			0x02
#define SMBB_BAT_IF_SUBTYPE			0x03
#define SMBB_USB_CHGPTH_SUBTYPE			0x04
#define SMBB_DC_CHGPTH_SUBTYPE			0x05
#define SMBB_BOOST_SUBTYPE			0x06
#define SMBB_MISC_SUBTYPE			0x07

/* SMBB peripheral subtype values */
#define SMBBP_CHGR_SUBTYPE			0x31
#define SMBBP_BUCK_SUBTYPE			0x32
#define SMBBP_BAT_IF_SUBTYPE			0x33
#define SMBBP_USB_CHGPTH_SUBTYPE		0x34
#define SMBBP_BOOST_SUBTYPE			0x36
#define SMBBP_MISC_SUBTYPE			0x37

/* SMBCL peripheral subtype values */
#define SMBCL_CHGR_SUBTYPE			0x41
#define SMBCL_BUCK_SUBTYPE			0x42
#define SMBCL_BAT_IF_SUBTYPE			0x43
#define SMBCL_USB_CHGPTH_SUBTYPE		0x44
#define SMBCL_MISC_SUBTYPE			0x47

#define QPNP_CHARGER_DEV_NAME	"qcom,qpnp-charger"

/* Status bits and masks */
#define CHGR_BOOT_DONE			BIT(7)
#define CHGR_CHG_EN			BIT(7)
#define CHGR_ON_BAT_FORCE_BIT		BIT(0)
#define USB_VALID_DEB_20MS		0x03
#define BUCK_VBAT_REG_NODE_SEL_BIT	BIT(0)
#define VREF_BATT_THERM_FORCE_ON	0xC0
#ifdef CONFIG_ARCH_MSM8610	/* Need to verify */
#define BAT_IF_BPD_CTRL_SEL		0x02
#else
#define BAT_IF_BPD_CTRL_SEL		0x03
#endif
#define VREF_BAT_THM_ENABLED_FSM	0x80
#define REV_BST_DETECTED		BIT(0)
#define BAT_THM_EN			BIT(1)
#define BAT_ID_EN			BIT(0)
#define BOOST_PWR_EN			BIT(7)
#define OCP_CLR_BIT			BIT(7)
#define OCP_THR_MASK			0x03
#define OCP_THR_900_MA			0x02
#define OCP_THR_500_MA			0x01
#define OCP_THR_200_MA			0x00
#define DC_HIGHER_PRIORITY		BIT(7)

/* Interrupt definitions */
/* smbb_chg_interrupts */
#define CHG_DONE_IRQ			BIT(7)
#define CHG_FAILED_IRQ			BIT(6)
#define FAST_CHG_ON_IRQ			BIT(5)
#define TRKL_CHG_ON_IRQ			BIT(4)
#define STATE_CHANGE_ON_IR		BIT(3)
#define CHGWDDOG_IRQ			BIT(2)
#define VBAT_DET_HI_IRQ			BIT(1)
#define VBAT_DET_LOW_IRQ		BIT(0)

/* smbb_buck_interrupts */
#define VDD_LOOP_IRQ			BIT(6)
#define IBAT_LOOP_IRQ			BIT(5)
#define ICHG_LOOP_IRQ			BIT(4)
#define VCHG_LOOP_IRQ			BIT(3)
#define OVERTEMP_IRQ			BIT(2)
#define VREF_OV_IRQ			BIT(1)
#define VBAT_OV_IRQ			BIT(0)

/* smbb_bat_if_interrupts */
#define PSI_IRQ				BIT(4)
#define VCP_ON_IRQ			BIT(3)
#define BAT_FET_ON_IRQ			BIT(2)
#define BAT_TEMP_OK_IRQ			BIT(1)
#define BATT_PRES_IRQ			BIT(0)

/* smbb_usb_interrupts */
#define CHG_GONE_IRQ			BIT(2)
#define USBIN_VALID_IRQ			BIT(1)
#define COARSE_DET_USB_IRQ		BIT(0)

/* smbb_dc_interrupts */
#define DCIN_VALID_IRQ			BIT(1)
#define COARSE_DET_DC_IRQ		BIT(0)

/* smbb_boost_interrupts */
#define LIMIT_ERROR_IRQ			BIT(1)
#define BOOST_PWR_OK_IRQ		BIT(0)

/* smbb_misc_interrupts */
#define TFTWDOG_IRQ			BIT(0)

/* SMBB types */
#define SMBB				BIT(1)
#define SMBBP				BIT(2)
#define SMBCL				BIT(3)

/* Workaround flags */
#define CHG_FLAGS_VCP_WA		BIT(0)
#define BOOST_FLASH_WA			BIT(1)
#define POWER_STAGE_WA			BIT(2)

#ifdef CONFIG_LGE_PM_VZW_FAST_CHG
enum vzw_chg_state {
	VZW_NO_CHARGER 				= 0,
	VZW_NORMAL_CHARGING			= 1,
	VZW_NOT_CHARGING			= 2,
	VZW_UNDER_CURRENT_CHARGING	= 3,
	VZW_USB_DRIVER_UNINSTALLED  = 4,
#ifdef CONFIG_LGE_PM_VZW_LLK
	VZW_LLK_NOT_CHARGING        = 5,
#endif
};

static enum vzw_chg_state chg_state = VZW_NO_CHARGER;

#define NOT_PRESENT 			0
#define CHARGER_PRESENT 		1
#define UNKNOWN_PRESENT 		2
#define SLOW_PRESENT 		3
#define USB_PRESENT             4

static int vzw_chg_present = NOT_PRESENT;

#define IS_OPEN_TA 0
#define IS_USB_DRIVER_UNINSTALLED 1
#define IS_USB_DRIVER_INSTALLED   2
#define IS_USB_CHARGING_ENABLE    3

static int usb_chg_state = IS_USB_DRIVER_INSTALLED;
#ifdef CONFIG_LGE_PM_VZW_LLK
static int temp_state = 0;
bool store_demo_enabled = false;
bool llk_monitor_soc_flag = false;
bool llk_stop_chg_flag = false;
#endif

extern int lge_usb_config_finish;
extern void send_drv_state_uevent(int usb_drv_state);
#endif

static struct qpnp_chg_chip *dummy_chip;

struct qpnp_chg_irq {
	int		irq;
	unsigned long		disabled;
	unsigned long		wake_enable;
	bool			is_wake;
};

struct qpnp_chg_regulator {
	struct regulator_desc			rdesc;
	struct regulator_dev			*rdev;
};
#if defined (CONFIG_MACH_MSM8926_B2LN_KR) || defined (CONFIG_MACH_MSM8926_JAGN_KR) || defined (CONFIG_MACH_MSM8926_AKA_CN) || defined(CONFIG_MACH_MSM8926_AKA_KR)
int count_count = 0 ;
#define VBUS_USB_THRESHOLD  2500
#endif
/* Charging information Debugging Log */
#if defined (CONFIG_MACH_MSM8926_X3N_KR) || defined(CONFIG_MACH_MSM8926_F70N_KR) || \
    defined (CONFIG_MACH_MSM8926_JAGN_KR) || defined (CONFIG_MACH_MSM8926_B2LN_KR) || \
    defined (CONFIG_MACH_MSM8926_VFP_KR) || defined (CONFIG_MACH_MSM8226_E8WIFI) || \
    defined (CONFIG_MACH_MSM8926_E8LTE) || defined (CONFIG_MACH_MSM8226_E7WIFI) || \
    defined (CONFIG_MACH_MSM8226_E9WIFI) || defined (CONFIG_MACH_MSM8226_E9WIFIN) || \
    defined (CONFIG_MACH_MSM8926_AKA_CN) || defined (CONFIG_MACH_MSM8926_E9LTE) || \
    defined (CONFIG_MACH_MSM8926_T8LTE) || defined(CONFIG_MACH_MSM8926_AKA_KR)
#define CIDL
#endif

#ifdef CONFIG_LGE_USING_CHG_LED
/**
 *  pwm_config_data - pwm configuration data
 *  @pwm_device - pwm device
 *  @pwm_channel - pwm channel to be configured for led
 *  @pwm_period_us - period for pwm, in us
 *  @mode - mode the led operates in
 *  @old_duty_pcts - storage for duty pcts that may need to be reused
 *  @default_mode - default mode of LED as set in device tree
 *  @use_blink - use blink sysfs entry
 *  @blinking - device is currently blinking w/LPG mode
 */
struct pwm_config_data {
	struct pwm_device	*pwm_dev;
	int			pwm_channel;
	u32			pwm_period_us;
	struct pwm_duty_cycles	*duty_cycles;
	int	*old_duty_pcts;
	u8	mode;
	u8	default_mode;
	bool use_blink;
	bool blinking;
};
#endif
/**
 * struct qpnp_chg_chip - device information
 * @dev:			device pointer to access the parent
 * @spmi:			spmi pointer to access spmi information
 * @chgr_base:			charger peripheral base address
 * @buck_base:			buck  peripheral base address
 * @bat_if_base:		battery interface  peripheral base address
 * @usb_chgpth_base:		USB charge path peripheral base address
 * @dc_chgpth_base:		DC charge path peripheral base address
 * @boost_base:			boost peripheral base address
 * @misc_base:			misc peripheral base address
 * @freq_base:			freq peripheral base address
 * @bat_is_cool:		indicates that battery is cool
 * @bat_is_warm:		indicates that battery is warm
 * @chg_done:			indicates that charging is completed
 * @usb_present:		present status of usb
 * @dc_present:			present status of dc
 * @batt_present:		present status of battery
 * @use_default_batt_values:	flag to report default battery properties
 * @btc_disabled		Flag to disable btc (disables hot and cold irqs)
 * @max_voltage_mv:		the max volts the batt should be charged up to
 * @min_voltage_mv:		min battery voltage before turning the FET on
 * @batt_weak_voltage_mv:	Weak battery voltage threshold
 * @vbatdet_max_err_mv		resume voltage hysterisis
 * @max_bat_chg_current:	maximum battery charge current in mA
 * @warm_bat_chg_ma:	warm battery maximum charge current in mA
 * @cool_bat_chg_ma:	cool battery maximum charge current in mA
 * @warm_bat_mv:		warm temperature battery target voltage
 * @cool_bat_mv:		cool temperature battery target voltage
 * @resume_delta_mv:		voltage delta at which battery resumes charging
 * @term_current:		the charging based term current
 * @safe_current:		battery safety current setting
 * @maxinput_usb_ma:		Maximum Input current USB
 * @maxinput_dc_ma:		Maximum Input current DC
 * @hot_batt_p			Hot battery threshold setting
 * @cold_batt_p			Cold battery threshold setting
 * @warm_bat_decidegc		Warm battery temperature in degree Celsius
 * @cool_bat_decidegc		Cool battery temperature in degree Celsius
 * @revision:			PMIC revision
 * @type:			SMBB type
 * @tchg_mins			maximum allowed software initiated charge time
 * @thermal_levels		amount of thermal mitigation levels
 * @thermal_mitigation		thermal mitigation level values
 * @therm_lvl_sel		thermal mitigation level selection
 * @dc_psy			power supply to export information to userspace
 * @usb_psy			power supply to export information to userspace
 * @bms_psy			power supply to export information to userspace
 * @batt_psy:			power supply to export information to userspace
 * @flags:			flags to activate specific workarounds
 *				throughout the driver
 *
 */
struct qpnp_chg_chip {
	struct device			*dev;
	struct spmi_device		*spmi;
	u16				chgr_base;
	u16				buck_base;
	u16				bat_if_base;
	u16				usb_chgpth_base;
	u16				dc_chgpth_base;
	u16				boost_base;
	u16				misc_base;
	u16				freq_base;
	struct qpnp_chg_irq		usbin_valid;
	struct qpnp_chg_irq		usb_ocp;
	struct qpnp_chg_irq		dcin_valid;
	struct qpnp_chg_irq		chg_gone;
	struct qpnp_chg_irq		chg_fastchg;
	struct qpnp_chg_irq		chg_trklchg;
	struct qpnp_chg_irq		chg_failed;
	struct qpnp_chg_irq		chg_vbatdet_lo;
	struct qpnp_chg_irq		batt_pres;
	struct qpnp_chg_irq		batt_temp_ok;
	struct qpnp_chg_irq		coarse_det_usb;
	bool				bat_is_cool;
	bool				bat_is_warm;
	bool				chg_done;
	bool				charger_monitor_checked;
	bool				usb_present;
	u8				usbin_health;
	bool				usb_coarse_det;
	bool				dc_present;
	bool				batt_present;
	bool				charging_disabled;
	bool				ovp_monitor_enable;
	bool				usb_valid_check_ovp;
	bool				btc_disabled;
	bool				use_default_batt_values;
	bool				duty_cycle_100p;
	bool				ibat_calibration_enabled;
	bool				aicl_settled;
	bool				use_external_rsense;
	bool				fastchg_on;
	bool				parallel_ovp_mode;
	unsigned int			bpd_detection;
	unsigned int			max_bat_chg_current;
	unsigned int			warm_bat_chg_ma;
	unsigned int			cool_bat_chg_ma;
	unsigned int			safe_voltage_mv;
	unsigned int			max_voltage_mv;
	unsigned int			min_voltage_mv;
	unsigned int			batt_weak_voltage_mv;
	unsigned int			vbatdet_max_err_mv;
	int				prev_usb_max_ma;
	int				set_vddmax_mv;
	int				delta_vddmax_mv;
	u8				trim_center;
	unsigned int			warm_bat_mv;
	unsigned int			cool_bat_mv;
	unsigned int			resume_delta_mv;
	int				insertion_ocv_uv;
	int				term_current;
	int				soc_resume_limit;
	bool				resuming_charging;
	unsigned int			maxinput_usb_ma;
	unsigned int			maxinput_dc_ma;
	unsigned int			hot_batt_p;
	unsigned int			cold_batt_p;
	int				warm_bat_decidegc;
	int				cool_bat_decidegc;
	int				fake_battery_soc;
	unsigned int			safe_current;
	unsigned int			revision;
	unsigned int			type;
	unsigned int			tchg_mins;
#ifndef CONFIG_LGE_PM
	unsigned int			thermal_levels;
	unsigned int			therm_lvl_sel;
	unsigned int			*thermal_mitigation;
#endif
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
	struct delayed_work 	battemp_work;
	struct wake_lock		lcs_wake_lock;
	enum   lge_btm_states	btm_state;
	int 					pseudo_ui_chg;
	int						not_chg;
	bool					is_charger_changed_from_irq;
	int						current_batt_temp;
#ifdef CONFIG_LGE_PM_THERMAL
	int						chg_current_te;
#endif
#endif
#ifdef CIDL
	struct delayed_work		charging_inform_work;
#endif
#if defined (CONFIG_MACH_MSM8926_B2LN_KR) || defined (CONFIG_MACH_MSM8926_JAGN_KR) || defined (CONFIG_MACH_MSM8926_AKA_CN)|| defined(CONFIG_MACH_MSM8926_AKA_KR)
	struct delayed_work     usbin_valid_work;
#endif
#if defined (CONFIG_MACH_MSM8926_JAGC_SPR) || defined (CONFIG_MACH_MSM8926_JAGNM_ATT) || defined (CONFIG_LGE_PM_CHARGING_DEBUG_LOG) \
	|| defined(CONFIG_MACH_MSM8926_JAGNM_RGS) || defined(CONFIG_MACH_MSM8926_JAGNM_TLS) || defined(CONFIG_MACH_MSM8926_JAGNM_VTR) || defined(CONFIG_MACH_MSM8926_JAGNM_BELL) || defined(CONFIG_MACH_MSM8926_JAGC_SPR)
	struct delayed_work		charging_check_work;
#endif

	struct power_supply		dc_psy;
	struct power_supply		*usb_psy;
	struct power_supply		*bms_psy;
	struct power_supply		batt_psy;
	uint32_t			flags;
	struct qpnp_adc_tm_btm_param	adc_param;
	struct work_struct		adc_measure_work;
	struct work_struct		adc_disable_work;
	struct delayed_work		arb_stop_work;
	struct delayed_work		eoc_work;
	struct delayed_work		usbin_health_check;
#ifdef CONFIG_LGE_PM_PWR_KEY_FOR_CHG_LOGO
    struct delayed_work     pwr_key_monitor_for_chg_logo;
#endif
#ifndef CONFIG_LGE_PM_4_25V_CHARGING_START
	struct work_struct		soc_check_work;
#endif
	struct delayed_work		aicl_check_work;

#ifdef CONFIG_LGE_PM_VZW_FAST_CHG
	struct delayed_work		input_current_check_work;
	struct work_struct		cancel_input_current_check_work;
	bool					is_vzw_slow_charging;
#endif

	struct work_struct		insertion_ocv_work;
	struct work_struct		ocp_clear_work;
	struct qpnp_chg_regulator	flash_wa_vreg;
	struct qpnp_chg_regulator	otg_vreg;
	struct qpnp_chg_regulator	boost_vreg;
#ifdef CONFIG_LGE_PM
	unsigned int            ac_online;
	unsigned int            current_max;
	bool                    chg_fail_irq_happen;
	bool					chg_timer;
#endif
	struct qpnp_chg_regulator	batfet_vreg;
	bool				batfet_ext_en;
	struct work_struct		batfet_lcl_work;
	struct qpnp_vadc_chip		*vadc_dev;
	struct qpnp_iadc_chip		*iadc_dev;
	struct qpnp_adc_tm_chip		*adc_tm_dev;
	struct mutex			jeita_configure_lock;
	spinlock_t			usbin_health_monitor_lock;
	struct mutex			batfet_vreg_lock;
	struct alarm			reduce_power_stage_alarm;
	struct work_struct		reduce_power_stage_work;
	bool				power_stage_workaround_running;
	bool				power_stage_workaround_enable;
	bool				is_flash_wa_reg_enabled;
	bool				ext_ovp_ic_gpio_enabled;
	unsigned int			ext_ovp_isns_gpio;
	unsigned int			usb_trim_default;
	u8				chg_temp_thresh_default;
/* LGE_CHANGE_S */
#ifdef CONFIG_LGE_PM_CHARGING_EXTERNAL_OVP
	int ext_ovp_gpio;
	int ext_ovp_slow_chg_trim;
#endif
/* LGE_CHANGE_E */
#ifdef CONFIG_LGE_PM_4_25V_CHARGING_START
	bool    from_temp_monitor_vbat_det_high;
	struct work_struct	resume_check_work;
	struct work_struct	batt_temp_cancel_work;
#endif

#ifdef CONFIG_LGE_PM_WORKAROUND_USB_VALID_BY_REVERSE_BOOST
	struct delayed_work property_work;
#endif
#ifdef CONFIG_LGE_PM
	struct wake_lock	uevent_wake_lock;
#endif
#ifdef CONFIG_LGE_PM_WORKAROUND_WEAK_CHARGER_REMOVAL_DETECTION
	struct wake_lock	weak_chg_wake_lock;
#endif
#ifdef CONFIG_LGE_USING_CHG_LED
	struct led_classdev    cdev;
	struct pwm_config_data *pwm_cfg;
#endif
#ifdef CONFIG_MAX17048_FUELGAUGE
	struct power_supply *maxim17048;
#endif
#ifdef CONFIG_LGE_WIRELESS_CHARGER_RT9536
	struct power_supply *wireless;
	struct work_struct	wlc_enable_work;
	struct work_struct	wlc_disable_work;
#endif
#ifdef CONFIG_LGE_PM_VZW_LLK
	struct delayed_work		vzw_llk_stop_chg_work;
#endif
};

static void
qpnp_chg_set_appropriate_battery_current(struct qpnp_chg_chip *chip);

#if defined(CONFIG_MACH_MSM8926_JAGNM_ATT) || defined(CONFIG_MACH_MSM8926_JAGNM_RGS) || defined(CONFIG_MACH_MSM8926_JAGNM_TLS) \
	|| defined(CONFIG_MACH_MSM8926_JAGNM_VTR) || defined(CONFIG_MACH_MSM8926_JAGNM_BELL) || defined(CONFIG_MACH_MSM8926_JAGC_SPR)
extern int max17048_get_voltage(void);
extern int max17048_get_capacity(void);
#endif

static struct of_device_id qpnp_charger_match_table[] = {
	{ .compatible = QPNP_CHARGER_DEV_NAME, },
	{}
};

#ifdef CONFIG_LGE_PM_FACTORY_PSEUDO_BATTERY
#if defined (CONFIG_MACH_MSM8226_E9WIFI) || defined (CONFIG_MACH_MSM8226_E9WIFIN) || \
    defined (CONFIG_MACH_MSM8226_E8WIFI) || defined (CONFIG_MACH_MSM8926_E8LTE) || defined (CONFIG_MACH_MSM8926_E9LTE)
#define PSEUDO_BATT_MAX 1500
#elif defined (CONFIG_MACH_MSM8226_E7WIFI) || defined (CONFIG_MACH_MSM8926_T8LTE)
#define PSEUDO_BATT_MAX 1100
#else
#define PSEUDO_BATT_MAX 700
#endif

static struct qpnp_chg_chip *qpnp_chg;
struct pseudo_batt_info_type pseudo_batt_info = {
	.mode = 0,
};
#endif

#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
int testmode_stop_eoc = 0;
#endif

#ifdef CONFIG_LGE_PM
#define LT_CABLE_56K                6
#define LT_CABLE_130K               7
#define LT_CABLE_910K		    11
#endif

#ifdef CONFIG_LGE_PM_CHARGING_EXTERNAL_OVP
bool lge_check_fast_chg_irq(void);
void lge_set_chg_path_to_external(void);
void lge_set_chg_path_to_internal(void);
#endif
#ifdef CONFIG_LGE_PM
static int qpnp_chg_tchg_disable(struct qpnp_chg_chip *chip);
static int qpnp_chg_tchg_max_set(struct qpnp_chg_chip *chip, int minutes);
static int qpnp_chg_ibatmax_set(struct qpnp_chg_chip *chip, int chg_current);

#endif
#ifdef CONFIG_LGE_PM_WORKAROUND_WEAK_CHARGER_REMOVAL_DETECTION
static int
get_prop_battery_voltage_now(struct qpnp_chg_chip *chip);
#endif
/* LGE_CHANGE_S: Cable Detect & Current Set */
/* BEGIN : janghyun.baek@lge.com 2013-01-25 For factory cable detection */
#ifdef CONFIG_LGE_PM_USB_ID
static unsigned int cable_type;
static bool is_factory_cable(void)
{
	unsigned int cable_info;
	cable_info = lge_pm_get_cable_type();

	if ((cable_info == CABLE_56K ||
		cable_info == CABLE_130K ||
		cable_info == CABLE_910K) ||
		(cable_type == LT_CABLE_56K ||
		cable_type == LT_CABLE_130K ||
		cable_type == LT_CABLE_910K))
		return 1;
	else
		return 0;
}

extern void lge_pm_set_usb_id_handle(struct qpnp_vadc_chip *);
#endif
/* END : janghyun.baek@lge.com 2013-01-25 */
/* LGE_CHANGE_E */

#if defined (CONFIG_MACH_MSM8226_E7WIFI) || defined (CONFIG_MACH_MSM8226_E8WIFI) || \
    defined (CONFIG_MACH_MSM8926_E8LTE) || defined (CONFIG_MACH_MSM8926_T8LTE)
#define FACTORY_IUSB_MAX_FOR_EMBEDDED_BATTERY 500
#endif
#if defined (CONFIG_MACH_MSM8226_E9WIFI) || defined (CONFIG_MACH_MSM8226_E9WIFIN) || defined (CONFIG_MACH_MSM8926_E9LTE)
#define FACTORY_IUSB_MAX_FOR_EMBEDDED_BATTERY   600
#endif
#define FACTORY_IBAT_MAX_FOR_EMBEDDED_BATTERY 100

#if defined (CONFIG_MACH_MSM8226_E7WIFI) || defined (CONFIG_MACH_MSM8226_E8WIFI) || \
    defined (CONFIG_MACH_MSM8926_E8LTE) || defined (CONFIG_MACH_MSM8226_E9WIFI) || \
    defined (CONFIG_MACH_MSM8226_E9WIFIN) || defined (CONFIG_MACH_MSM8926_E9LTE) || \
    defined (CONFIG_MACH_MSM8926_T8LTE)
static bool is_56k_910k_factory_cable(void)
{
	unsigned int cable_info;
	cable_info = lge_pm_get_cable_type();

	if ((cable_info == CABLE_56K ||
		cable_info == CABLE_910K) ||
		(cable_type == LT_CABLE_56K ||
		cable_type == LT_CABLE_910K))
		return 1;
	else
		return 0;
}
#endif

/* LGE_CHANGE_S */
#ifdef CONFIG_LGE_PM_CHARGING_EXTERNAL_OVP
enum {
	EXT_OVP_CTRL_LOW = 0 ,	/* Not using external ovp path */
	EXT_OVP_CTRL_HIGH = 1,	/* Using external ovp path */
};

static void lge_pm_set_ext_ovp(int ext_ovp_gpio, bool on)
{
#if defined (CONFIG_MACH_MSM8926_JAGNM_ATT) || defined (CONFIG_MACH_MSM8926_JAGC_SPR) || defined (CONFIG_MACH_MSM8926_JAGNM_GLOBAL_COM) \
	|| defined (CONFIG_MACH_MSM8926_JAGN_KR) || defined(CONFIG_MACH_MSM8926_JAGNM_RGS) \
	|| defined(CONFIG_MACH_MSM8926_JAGNM_TLS) || defined(CONFIG_MACH_MSM8926_JAGNM_VTR) || defined(CONFIG_MACH_MSM8926_JAGNM_BELL) || defined(CONFIG_MACH_MSM8926_AKA_CN) || defined(CONFIG_MACH_MSM8926_JAGC_SPR) || defined(CONFIG_MACH_MSM8926_AKA_KR)
		gpio_direction_output(31, on);
#elif defined (CONFIG_MACH_MSM8926_E2_MPCS_US)
		gpio_direction_output(33, on);
#endif
	if (gpio_is_valid(ext_ovp_gpio))
		gpio_set_value(ext_ovp_gpio, on);
}
#endif
/* LGE_CHANGE_E */

#ifdef CONFIG_LGE_PM_4_25V_CHARGING_START
static void
lge_pm_batt_temp_cancel_work(struct work_struct *work)
{
	struct qpnp_chg_chip *chip = container_of(work,
				struct qpnp_chg_chip, batt_temp_cancel_work);

	cancel_delayed_work_sync(&chip->battemp_work);

	chip->pseudo_ui_chg = 0;
	if (wake_lock_active(&chip->lcs_wake_lock))
		wake_unlock(&chip->lcs_wake_lock);
}
#endif
#ifdef CONFIG_LGE_PM
int32_t qpnp_charger_is_ready(void)
{
	struct qpnp_chg_chip *chg = qpnp_chg;

	if (!chg)
		return -EPROBE_DEFER;
	return 0;
}
EXPORT_SYMBOL(qpnp_charger_is_ready);
#endif

enum bpd_type {
	BPD_TYPE_BAT_ID,
	BPD_TYPE_BAT_THM,
	BPD_TYPE_BAT_THM_BAT_ID,
};

static const char * const bpd_label[] = {
	[BPD_TYPE_BAT_ID] = "bpd_id",
	[BPD_TYPE_BAT_THM] = "bpd_thm",
	[BPD_TYPE_BAT_THM_BAT_ID] = "bpd_thm_id",
};

enum btc_type {
	HOT_THD_25_PCT = 25,
	HOT_THD_35_PCT = 35,
	COLD_THD_70_PCT = 70,
	COLD_THD_80_PCT = 80,
};

static u8 btc_value[] = {
	[HOT_THD_25_PCT] = 0x0,
	[HOT_THD_35_PCT] = BIT(0),
	[COLD_THD_70_PCT] = 0x0,
	[COLD_THD_80_PCT] = BIT(1),
};

enum usbin_health {
	USBIN_UNKNOW,
	USBIN_OK,
	USBIN_OVP,
};

static int ext_ovp_isns_present;
module_param(ext_ovp_isns_present, int, 0444);
static int ext_ovp_isns_r;
module_param(ext_ovp_isns_r, int, 0444);

static bool ext_ovp_isns_online;
static long ext_ovp_isns_ua;
#define MAX_CURRENT_LENGTH_9A	10
#define ISNS_CURRENT_RATIO	2500
static int ext_ovp_isns_read(char *buffer, const struct kernel_param *kp)
{
	int rc;
	struct qpnp_vadc_result results;
	struct power_supply *batt_psy = power_supply_get_by_name("battery");
	struct qpnp_chg_chip *chip = container_of(batt_psy,
				struct qpnp_chg_chip, batt_psy);

	if (!ext_ovp_isns_present)
		return 0;

	rc = qpnp_vadc_read(chip->vadc_dev, P_MUX7_1_1, &results);
	if (rc) {
		pr_err("Unable to read vbat rc=%d\n", rc);
		return 0;
	}

	pr_debug("voltage %lld uV, current: %d\n mA", results.physical,
			((int) results.physical /
			 (ext_ovp_isns_r / ISNS_CURRENT_RATIO)));

	return snprintf(buffer, MAX_CURRENT_LENGTH_9A, "%d\n",
			((int)results.physical /
			 (ext_ovp_isns_r / ISNS_CURRENT_RATIO)));
}

static int ext_ovp_isns_enable(const char *val, const struct kernel_param *kp)
{
	int rc;
	struct power_supply *batt_psy = power_supply_get_by_name("battery");
	struct qpnp_chg_chip *chip = container_of(batt_psy,
				struct qpnp_chg_chip, batt_psy);

	rc = param_set_bool(val, kp);
	if (rc) {
		pr_err("Unable to set gpio en: %d\n", rc);
		return rc;
	}

	if (*(bool *)kp->arg) {
		gpio_direction_output(
						chip->ext_ovp_isns_gpio, 1);
		chip->ext_ovp_ic_gpio_enabled = 1;
		pr_debug("enabled GPIO\n");
	} else {
		gpio_direction_output(
						chip->ext_ovp_isns_gpio, 0);
		chip->ext_ovp_ic_gpio_enabled = 0;
		pr_debug("disabled GPIO\n");
	}

	return rc;
}

static struct kernel_param_ops ext_ovp_isns_ops = {
	.get = ext_ovp_isns_read,
};
module_param_cb(ext_ovp_isns_ua, &ext_ovp_isns_ops, &ext_ovp_isns_ua, 0644);

static struct kernel_param_ops ext_ovp_en_ops = {
	.set = ext_ovp_isns_enable,
	.get = param_get_bool,
};
module_param_cb(ext_ovp_isns_online, &ext_ovp_en_ops,
		&ext_ovp_isns_online, 0664);

static inline int
get_bpd(const char *name)
{
	int i = 0;
	for (i = 0; i < ARRAY_SIZE(bpd_label); i++) {
		if (strcmp(bpd_label[i], name) == 0)
			return i;
	}
	return -EINVAL;
}

static bool
is_within_range(int value, int left, int right)
{
	if (left >= right && left >= value && value >= right)
		return 1;
	if (left <= right && left <= value && value <= right)
		return 1;
	return 0;
}

static int
qpnp_chg_read(struct qpnp_chg_chip *chip, u8 *val,
			u16 base, int count)
{
	int rc = 0;
	struct spmi_device *spmi = chip->spmi;

	if (base == 0) {
		pr_err("base cannot be zero base=0x%02x sid=0x%02x rc=%d\n",
			base, spmi->sid, rc);
		return -EINVAL;
	}

	rc = spmi_ext_register_readl(spmi->ctrl, spmi->sid, base, val, count);
	if (rc) {
		pr_err("SPMI read failed base=0x%02x sid=0x%02x rc=%d\n", base,
				spmi->sid, rc);
		return rc;
	}
	return 0;
}

static int
qpnp_chg_write(struct qpnp_chg_chip *chip, u8 *val,
			u16 base, int count)
{
	int rc = 0;
	struct spmi_device *spmi = chip->spmi;

	if (base == 0) {
		pr_err("base cannot be zero base=0x%02x sid=0x%02x rc=%d\n",
			base, spmi->sid, rc);
		return -EINVAL;
	}

	rc = spmi_ext_register_writel(spmi->ctrl, spmi->sid, base, val, count);
	if (rc) {
		pr_err("write failed base=0x%02x sid=0x%02x rc=%d\n",
			base, spmi->sid, rc);
		return rc;
	}

	return 0;
}

static int
qpnp_chg_masked_write(struct qpnp_chg_chip *chip, u16 base,
						u8 mask, u8 val, int count)
{
	int rc;
	u8 reg;

	rc = qpnp_chg_read(chip, &reg, base, count);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n", base, rc);
		return rc;
	}
	pr_debug("addr = 0x%x read 0x%x\n", base, reg);

	reg &= ~mask;
	reg |= val & mask;

	pr_debug("Writing 0x%x\n", reg);

	rc = qpnp_chg_write(chip, &reg, base, count);
	if (rc) {
		pr_err("spmi write failed: addr=%03X, rc=%d\n", base, rc);
		return rc;
	}

	return 0;
}

static void
qpnp_chg_enable_irq(struct qpnp_chg_irq *irq)
{
	if (__test_and_clear_bit(0, &irq->disabled)) {
		pr_debug("number = %d\n", irq->irq);
		enable_irq(irq->irq);
	}
	if ((irq->is_wake) && (!__test_and_set_bit(0, &irq->wake_enable))) {
		pr_debug("enable wake, number = %d\n", irq->irq);
		enable_irq_wake(irq->irq);
	}
}

static void
qpnp_chg_disable_irq(struct qpnp_chg_irq *irq)
{
	if (!__test_and_set_bit(0, &irq->disabled)) {
		pr_debug("number = %d\n", irq->irq);
		disable_irq_nosync(irq->irq);
	}
	if ((irq->is_wake) && (__test_and_clear_bit(0, &irq->wake_enable))) {
		pr_debug("disable wake, number = %d\n", irq->irq);
		disable_irq_wake(irq->irq);
	}
}

static void
qpnp_chg_irq_wake_enable(struct qpnp_chg_irq *irq)
{
	if (!__test_and_set_bit(0, &irq->wake_enable)) {
		pr_debug("number = %d\n", irq->irq);
		enable_irq_wake(irq->irq);
	}
	irq->is_wake = true;
}

static void
qpnp_chg_irq_wake_disable(struct qpnp_chg_irq *irq)
{
	if (__test_and_clear_bit(0, &irq->wake_enable)) {
		pr_debug("number = %d\n", irq->irq);
		disable_irq_wake(irq->irq);
	}
	irq->is_wake = false;
}

#define USB_OTG_EN_BIT	BIT(0)
static int
qpnp_chg_is_otg_en_set(struct qpnp_chg_chip *chip)
{
	u8 usb_otg_en;
	int rc;

	rc = qpnp_chg_read(chip, &usb_otg_en,
				 chip->usb_chgpth_base + CHGR_USB_USB_OTG_CTL,
				 1);

	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n",
				chip->usb_chgpth_base + CHGR_STATUS, rc);
		return rc;
	}
	pr_debug("usb otg en 0x%x\n", usb_otg_en);

	return (usb_otg_en & USB_OTG_EN_BIT) ? 1 : 0;
}

static int
qpnp_chg_is_boost_en_set(struct qpnp_chg_chip *chip)
{
	u8 boost_en_ctl;
	int rc;

	rc = qpnp_chg_read(chip, &boost_en_ctl,
		chip->boost_base + BOOST_ENABLE_CONTROL, 1);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n",
				chip->boost_base + BOOST_ENABLE_CONTROL, rc);
		return rc;
	}

	pr_debug("boost en 0x%x\n", boost_en_ctl);

	return (boost_en_ctl & BOOST_PWR_EN) ? 1 : 0;
}

/* LGE not useed BTM feature  */
#ifndef CONFIG_LGE_PM
static int
qpnp_chg_is_batt_temp_ok(struct qpnp_chg_chip *chip)
{
	u8 batt_rt_sts;
	int rc;

	rc = qpnp_chg_read(chip, &batt_rt_sts,
				 INT_RT_STS(chip->bat_if_base), 1);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n",
				INT_RT_STS(chip->bat_if_base), rc);
		return rc;
	}

	return (batt_rt_sts & BAT_TEMP_OK_IRQ) ? 1 : 0;
}
#endif
static int
qpnp_chg_is_batt_present(struct qpnp_chg_chip *chip)
{
	u8 batt_pres_rt_sts;
	int rc;

	rc = qpnp_chg_read(chip, &batt_pres_rt_sts,
				 INT_RT_STS(chip->bat_if_base), 1);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n",
				INT_RT_STS(chip->bat_if_base), rc);
		return rc;
	}

	return (batt_pres_rt_sts & BATT_PRES_IRQ) ? 1 : 0;
}

static int
qpnp_chg_is_batfet_closed(struct qpnp_chg_chip *chip)
{
	u8 batfet_closed_rt_sts;
	int rc;

	rc = qpnp_chg_read(chip, &batfet_closed_rt_sts,
				 INT_RT_STS(chip->bat_if_base), 1);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n",
				INT_RT_STS(chip->bat_if_base), rc);
		return rc;
	}

	return (batfet_closed_rt_sts & BAT_FET_ON_IRQ) ? 1 : 0;
}

static int
qpnp_chg_is_usb_chg_plugged_in(struct qpnp_chg_chip *chip)
{
	u8 usb_chgpth_rt_sts;
	int rc;

	rc = qpnp_chg_read(chip, &usb_chgpth_rt_sts,
				 INT_RT_STS(chip->usb_chgpth_base), 1);

	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n",
				INT_RT_STS(chip->usb_chgpth_base), rc);
		return rc;
	}
	pr_debug("chgr usb sts 0x%x\n", usb_chgpth_rt_sts);

	return (usb_chgpth_rt_sts & USBIN_VALID_IRQ) ? 1 : 0;
}

static bool
qpnp_is_dc_higher_prio(struct qpnp_chg_chip *chip)
{
	int rc;
	u8 usb_ctl;

	if (!chip->type == SMBB)
		return false;

	rc = qpnp_chg_read(chip, &usb_ctl,
			chip->usb_chgpth_base + USB_CHGPTH_CTL, 1);
	if (rc) {
		pr_err("failed to read usb ctl rc=%d\n", rc);
		return 0;
	}

	return !!(usb_ctl & DC_HIGHER_PRIORITY);
}
#ifdef CONFIG_LGE_PM_VZW_LLK
bool external_qpnp_chg_is_usb_chg_plugged_in(void)
{
	return qpnp_chg_is_usb_chg_plugged_in(qpnp_chg);
}
#endif


static bool
qpnp_chg_is_ibat_loop_active(struct qpnp_chg_chip *chip)
{
	int rc;
	u8 buck_sts;

	rc = qpnp_chg_read(chip, &buck_sts,
			INT_RT_STS(chip->buck_base), 1);
	if (rc) {
		pr_err("failed to read buck RT status rc=%d\n", rc);
		return 0;
	}

	return !!(buck_sts & IBAT_LOOP_IRQ);
}

#define USB_VALID_MASK		0xC0
#define USB_VALID_IN_MASK	BIT(7)
#define USB_COARSE_DET		0x10
#define USB_VALID_OVP_VALUE	0x40
static int
qpnp_chg_check_usb_coarse_det(struct qpnp_chg_chip *chip)
{
	u8 usbin_chg_rt_sts;
	int rc;
	rc = qpnp_chg_read(chip, &usbin_chg_rt_sts,
		chip->usb_chgpth_base + CHGR_STATUS , 1);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n",
			chip->usb_chgpth_base + CHGR_STATUS, rc);
		return rc;
	}
	return (usbin_chg_rt_sts & USB_COARSE_DET) ? 1 : 0;
}

static int
qpnp_chg_check_usbin_health(struct qpnp_chg_chip *chip)
{
	u8 usbin_chg_rt_sts, usb_chgpth_rt_sts;
	u8 usbin_health = 0;
	int rc;

	rc = qpnp_chg_read(chip, &usbin_chg_rt_sts,
		chip->usb_chgpth_base + CHGR_STATUS , 1);

	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n",
		chip->usb_chgpth_base + CHGR_STATUS, rc);
		return rc;
	}

	rc = qpnp_chg_read(chip, &usb_chgpth_rt_sts,
		INT_RT_STS(chip->usb_chgpth_base) , 1);

	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n",
		INT_RT_STS(chip->usb_chgpth_base), rc);
		return rc;
	}

	pr_debug("chgr usb sts 0x%x, chgpth rt sts 0x%x\n",
				usbin_chg_rt_sts, usb_chgpth_rt_sts);
	if ((usbin_chg_rt_sts & USB_COARSE_DET) == USB_COARSE_DET) {
		if ((usbin_chg_rt_sts & USB_VALID_MASK)
			 == USB_VALID_OVP_VALUE) {
			usbin_health = USBIN_OVP;
			pr_err("Over voltage charger inserted\n");
		} else if ((usb_chgpth_rt_sts & USBIN_VALID_IRQ) != 0) {
			usbin_health = USBIN_OK;
			pr_debug("Valid charger inserted\n");
		}
	} else {
		usbin_health = USBIN_UNKNOW;
		pr_debug("Charger plug out\n");
	}

	return usbin_health;
}

static int
qpnp_chg_is_dc_chg_plugged_in(struct qpnp_chg_chip *chip)
{
	u8 dcin_valid_rt_sts;
	int rc;

	if (!chip->dc_chgpth_base)
		return 0;

	rc = qpnp_chg_read(chip, &dcin_valid_rt_sts,
				 INT_RT_STS(chip->dc_chgpth_base), 1);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n",
				INT_RT_STS(chip->dc_chgpth_base), rc);
		return rc;
	}

	return (dcin_valid_rt_sts & DCIN_VALID_IRQ) ? 1 : 0;
}

static int
qpnp_chg_is_ichg_loop_active(struct qpnp_chg_chip *chip)
{
	u8 buck_sts;
	int rc;

	rc = qpnp_chg_read(chip, &buck_sts, INT_RT_STS(chip->buck_base), 1);

	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n",
				INT_RT_STS(chip->buck_base), rc);
		return rc;
	}
	pr_debug("buck usb sts 0x%x\n", buck_sts);

	return (buck_sts & ICHG_LOOP_IRQ) ? 1 : 0;
}

#define QPNP_CHG_I_MAX_MIN_100		100
#define QPNP_CHG_I_MAX_MIN_150		150
#define QPNP_CHG_I_MAX_MIN_MA		200
#define QPNP_CHG_I_MAX_MAX_MA		2500
#define QPNP_CHG_I_MAXSTEP_MA		100
static int
qpnp_chg_idcmax_set(struct qpnp_chg_chip *chip, int mA)
{
	int rc = 0;
	u8 dc = 0;

	if (mA < QPNP_CHG_I_MAX_MIN_100
			|| mA > QPNP_CHG_I_MAX_MAX_MA) {
		pr_err("bad mA=%d asked to set\n", mA);
		return -EINVAL;
	}

	if (mA == QPNP_CHG_I_MAX_MIN_100) {
		dc = 0x00;
		pr_debug("current=%d setting %02x\n", mA, dc);
		return qpnp_chg_write(chip, &dc,
			chip->dc_chgpth_base + CHGR_I_MAX_REG, 1);
	} else if (mA == QPNP_CHG_I_MAX_MIN_150) {
		dc = 0x01;
		pr_debug("current=%d setting %02x\n", mA, dc);
		return qpnp_chg_write(chip, &dc,
			chip->dc_chgpth_base + CHGR_I_MAX_REG, 1);
	}

	dc = mA / QPNP_CHG_I_MAXSTEP_MA;

	pr_debug("current=%d setting 0x%x\n", mA, dc);
	rc = qpnp_chg_write(chip, &dc,
		chip->dc_chgpth_base + CHGR_I_MAX_REG, 1);

	return rc;
}

static int
qpnp_chg_iusb_trim_get(struct qpnp_chg_chip *chip)
{
	int rc = 0;
	u8 trim_reg;

	rc = qpnp_chg_read(chip, &trim_reg,
			chip->usb_chgpth_base + CHGR_USB_TRIM, 1);
	if (rc) {
		pr_err("failed to read USB_TRIM rc=%d\n", rc);
		return 0;
	}

	return trim_reg;
}

static int
qpnp_chg_usb_iusbmax_get(struct qpnp_chg_chip *chip)
{
	int rc, iusbmax_ma;
	u8 iusbmax;

	rc = qpnp_chg_read(chip, &iusbmax,
		chip->usb_chgpth_base + CHGR_I_MAX_REG, 1);
	if (rc) {
		pr_err("failed to read IUSB_MAX rc=%d\n", rc);
		return 0;
	}

	if (iusbmax == 0)
		iusbmax_ma = QPNP_CHG_I_MAX_MIN_100;
	else if (iusbmax == 0x01)
		iusbmax_ma = QPNP_CHG_I_MAX_MIN_150;
	else
		iusbmax_ma = iusbmax * QPNP_CHG_I_MAXSTEP_MA;

	pr_debug("iusbmax = 0x%02x, ma = %d\n", iusbmax, iusbmax_ma);

	return iusbmax_ma;
}

static int
qpnp_chg_iusb_trim_set(struct qpnp_chg_chip *chip, int trim)
{
	int rc = 0;
	rc = qpnp_chg_masked_write(chip,
		chip->usb_chgpth_base + SEC_ACCESS,
		0xFF,
		0xA5, 1);
	if (rc) {
		pr_err("failed to write SEC_ACCESS rc=%d\n", rc);
		return rc;
	}

	rc = qpnp_chg_masked_write(chip,
		chip->usb_chgpth_base + CHGR_USB_TRIM,
		0xFF,
		trim, 1);
	if (rc) {
		pr_err("failed to write USB TRIM rc=%d\n", rc);
		return rc;
	}

	return rc;
}

#define IOVP_USB_WALL_TRSH_MA   150
static int
qpnp_chg_iusbmax_set(struct qpnp_chg_chip *chip, int mA)
{
	int rc = 0;
	u8 usb_reg = 0, temp = 8;

	pr_err("[LGE] qpnp_chg_iusbmax_set, current = %d\n", mA);

	if (mA < 0 || mA > QPNP_CHG_I_MAX_MAX_MA) {
		pr_err("bad mA=%d asked to set\n", mA);
		return -EINVAL;
	}

	if (mA <= QPNP_CHG_I_MAX_MIN_100) {
		usb_reg = 0x00;
		pr_debug("current=%d setting %02x\n", mA, usb_reg);
		return qpnp_chg_write(chip, &usb_reg,
		chip->usb_chgpth_base + CHGR_I_MAX_REG, 1);
	} else if (mA == QPNP_CHG_I_MAX_MIN_150) {
		usb_reg = 0x01;
		pr_debug("current=%d setting %02x\n", mA, usb_reg);
		return qpnp_chg_write(chip, &usb_reg,
		chip->usb_chgpth_base + CHGR_I_MAX_REG, 1);
	}

	/* Impose input current limit */
	if (chip->maxinput_usb_ma)
		mA = (chip->maxinput_usb_ma) <= mA ? chip->maxinput_usb_ma : mA;

	usb_reg = mA / QPNP_CHG_I_MAXSTEP_MA;

	if (chip->flags & CHG_FLAGS_VCP_WA) {
		temp = 0xA5;
		rc =  qpnp_chg_write(chip, &temp,
			chip->buck_base + SEC_ACCESS, 1);
		rc =  qpnp_chg_masked_write(chip,
			chip->buck_base + CHGR_BUCK_COMPARATOR_OVRIDE_3,
			0x0C, 0x0C, 1);
	}

	pr_info("current=%d setting 0x%x\n", mA, usb_reg);
	rc = qpnp_chg_write(chip, &usb_reg,
		chip->usb_chgpth_base + CHGR_I_MAX_REG, 1);

	if (chip->flags & CHG_FLAGS_VCP_WA) {
		temp = 0xA5;
		udelay(200);
		rc =  qpnp_chg_write(chip, &temp,
			chip->buck_base + SEC_ACCESS, 1);
		rc =  qpnp_chg_masked_write(chip,
			chip->buck_base + CHGR_BUCK_COMPARATOR_OVRIDE_3,
			0x0C, 0x00, 1);
	}
#if defined(CONFIG_MACH_MSM8926_JAGC_SPR) || defined(CONFIG_MACH_MSM8926_JAGNM_ATT) || \
	defined(CONFIG_MACH_MSM8926_JAGNM_BELL) || defined(CONFIG_MACH_MSM8926_JAGC_SPR) || \
	defined(CONFIG_MACH_MSM8926_JAGNM_RGS)
	if (!qpnp_chg_is_usb_chg_plugged_in(chip))
		mA = 0;

	if (!qpnp_chg_is_dc_chg_plugged_in(chip))
		mA = 0;
#endif
	return rc;
}

#define QPNP_CHG_VINMIN_MIN_MV		4000
#define QPNP_CHG_VINMIN_HIGH_MIN_MV	5600
#define QPNP_CHG_VINMIN_HIGH_MIN_VAL	0x2B
#define QPNP_CHG_VINMIN_MAX_MV		9600
#define QPNP_CHG_VINMIN_STEP_MV		50
#define QPNP_CHG_VINMIN_STEP_HIGH_MV	200
#define QPNP_CHG_VINMIN_MASK		0x3F
#define QPNP_CHG_VINMIN_MIN_VAL	0x0C
static int
qpnp_chg_vinmin_set(struct qpnp_chg_chip *chip, int voltage)
{
	u8 temp;

	if ((voltage < QPNP_CHG_VINMIN_MIN_MV)
			|| (voltage > QPNP_CHG_VINMIN_MAX_MV)) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}
	if (voltage >= QPNP_CHG_VINMIN_HIGH_MIN_MV) {
		temp = QPNP_CHG_VINMIN_HIGH_MIN_VAL;
		temp += (voltage - QPNP_CHG_VINMIN_HIGH_MIN_MV)
			/ QPNP_CHG_VINMIN_STEP_HIGH_MV;
	} else {
		temp = QPNP_CHG_VINMIN_MIN_VAL;
		temp += (voltage - QPNP_CHG_VINMIN_MIN_MV)
			/ QPNP_CHG_VINMIN_STEP_MV;
	}

	pr_debug("voltage=%d setting %02x\n", voltage, temp);
	return qpnp_chg_masked_write(chip,
			chip->chgr_base + CHGR_VIN_MIN,
			QPNP_CHG_VINMIN_MASK, temp, 1);
}

static int
qpnp_chg_vinmin_get(struct qpnp_chg_chip *chip)
{
	int rc, vin_min_mv;
	u8 vin_min;

	rc = qpnp_chg_read(chip, &vin_min, chip->chgr_base + CHGR_VIN_MIN, 1);
	if (rc) {
		pr_err("failed to read VIN_MIN rc=%d\n", rc);
		return 0;
	}

	if (vin_min == 0)
		vin_min_mv = QPNP_CHG_I_MAX_MIN_100;
	else if (vin_min >= QPNP_CHG_VINMIN_HIGH_MIN_VAL)
		vin_min_mv = QPNP_CHG_VINMIN_HIGH_MIN_MV +
			(vin_min - QPNP_CHG_VINMIN_HIGH_MIN_VAL)
				* QPNP_CHG_VINMIN_STEP_HIGH_MV;
	else
		vin_min_mv = QPNP_CHG_VINMIN_MIN_MV +
			(vin_min - QPNP_CHG_VINMIN_MIN_VAL)
				* QPNP_CHG_VINMIN_STEP_MV;
	pr_debug("vin_min= 0x%02x, ma = %d\n", vin_min, vin_min_mv);

	return vin_min_mv;
}

#define QPNP_CHG_VBATWEAK_MIN_MV	2100
#define QPNP_CHG_VBATWEAK_MAX_MV	3600
#define QPNP_CHG_VBATWEAK_STEP_MV	100
static int
qpnp_chg_vbatweak_set(struct qpnp_chg_chip *chip, int vbatweak_mv)
{
	u8 temp;

	if (vbatweak_mv < QPNP_CHG_VBATWEAK_MIN_MV
			|| vbatweak_mv > QPNP_CHG_VBATWEAK_MAX_MV)
		return -EINVAL;

	temp = (vbatweak_mv - QPNP_CHG_VBATWEAK_MIN_MV)
			/ QPNP_CHG_VBATWEAK_STEP_MV;

	pr_debug("voltage=%d setting %02x\n", vbatweak_mv, temp);
	return qpnp_chg_write(chip, &temp,
		chip->chgr_base + CHGR_VBAT_WEAK, 1);
}

#define ILIMIT_OVR_0	0x02
static int
override_dcin_ilimit(struct qpnp_chg_chip *chip, bool override)
{
	int rc;

	pr_debug("override %d\n", override);
	rc = qpnp_chg_masked_write(chip,
			chip->dc_chgpth_base + SEC_ACCESS,
			0xA5,
			0xA5, 1);
	rc |= qpnp_chg_masked_write(chip,
			chip->dc_chgpth_base + DC_COMP_OVR1,
			0xFF,
			override ? ILIMIT_OVR_0 : 0, 1);
	if (rc) {
		pr_err("Failed to override dc ilimit rc = %d\n", rc);
		return rc;
	}

	return rc;
}

#define DUAL_PATH_EN	BIT(7)
static int
switch_parallel_ovp_mode(struct qpnp_chg_chip *chip, bool enable)
{
	int rc = 0;

	if (!chip->usb_chgpth_base || !chip->dc_chgpth_base)
		return rc;

	pr_debug("enable %d\n", enable);
	rc = override_dcin_ilimit(chip, 1);
	udelay(10);

	/* enable/disable dual path mode */
	rc = qpnp_chg_masked_write(chip,
			chip->usb_chgpth_base + SEC_ACCESS,
			0xA5,
			0xA5, 1);
	rc |= qpnp_chg_masked_write(chip,
			chip->usb_chgpth_base + USB_SPARE,
			0xFF,
			enable ? DUAL_PATH_EN : 0, 1);
	if (rc) {
		pr_err("Failed to turn on usb ovp rc = %d\n", rc);
		return rc;
	}

	if (enable)
		rc = override_dcin_ilimit(chip, 0);
	return rc;
}

#define USB_SUSPEND_BIT	BIT(0)
static int
qpnp_chg_usb_suspend_enable(struct qpnp_chg_chip *chip, int enable)
{
	/* Turn off DC OVP FET when going into USB suspend */
	if (chip->parallel_ovp_mode && enable)
		switch_parallel_ovp_mode(chip, 0);

	return qpnp_chg_masked_write(chip,
			chip->usb_chgpth_base + CHGR_USB_USB_SUSP,
			USB_SUSPEND_BIT,
			enable ? USB_SUSPEND_BIT : 0, 1);
}

static int
qpnp_chg_charge_en(struct qpnp_chg_chip *chip, int enable)
{
	if (chip->insertion_ocv_uv == 0 && enable) {
		pr_debug("Battery not present, skipping\n");
		return 0;
	}
	pr_err("charging %s\n", enable ? "enabled" : "disabled");
	return qpnp_chg_masked_write(chip, chip->chgr_base + CHGR_CHG_CTRL,
			CHGR_CHG_EN,
			enable ? CHGR_CHG_EN : 0, 1);
}

#ifdef CONFIG_LGE_PM
int32_t external_qpnp_enable_charging(bool enable)
{
	int ret;

	pr_debug("external_qpnp_enable_charging = %d.\n", enable);

	ret = qpnp_chg_charge_en(qpnp_chg, enable);

	if (ret) {
		pr_err("Failed to set qpnp_chg_charge_en rc=%d\n", ret);
		return ret;
		}

	return 0;
}
#endif

static int
qpnp_chg_force_run_on_batt(struct qpnp_chg_chip *chip, int disable)
{
	/* Don't run on battery for batteryless hardware */
	if (chip->use_default_batt_values)
		return 0;
	/* Don't force on battery if battery is not present */
	if (!qpnp_chg_is_batt_present(chip))
		return 0;

	/* This bit forces the charger to run off of the battery rather
	 * than a connected charger */
	return qpnp_chg_masked_write(chip, chip->chgr_base + CHGR_CHG_CTRL,
			CHGR_ON_BAT_FORCE_BIT,
			disable ? CHGR_ON_BAT_FORCE_BIT : 0, 1);
}

#define BUCK_DUTY_MASK_100P	0x30
static int
qpnp_buck_set_100_duty_cycle_enable(struct qpnp_chg_chip *chip, int enable)
{
	int rc;

	pr_debug("enable: %d\n", enable);

	rc = qpnp_chg_masked_write(chip,
		chip->buck_base + SEC_ACCESS, 0xA5, 0xA5, 1);
	if (rc) {
		pr_debug("failed to write sec access rc=%d\n", rc);
		return rc;
	}

	rc = qpnp_chg_masked_write(chip,
		chip->buck_base + BUCK_TEST_SMBC_MODES,
			BUCK_DUTY_MASK_100P, enable ? 0x00 : 0x10, 1);
	if (rc) {
		pr_debug("failed enable 100p duty cycle rc=%d\n", rc);
		return rc;
	}

	return rc;
}

#define COMPATATOR_OVERRIDE_0	0x80
static int
qpnp_chg_toggle_chg_done_logic(struct qpnp_chg_chip *chip, int enable)
{
	int rc;

	pr_debug("toggle: %d\n", enable);

	rc = qpnp_chg_masked_write(chip,
		chip->buck_base + SEC_ACCESS, 0xA5, 0xA5, 1);
	if (rc) {
		pr_debug("failed to write sec access rc=%d\n", rc);
		return rc;
	}

	rc = qpnp_chg_masked_write(chip,
		chip->buck_base + CHGR_BUCK_COMPARATOR_OVRIDE_1,
			0xC0, enable ? 0x00 : COMPATATOR_OVERRIDE_0, 1);
	if (rc) {
		pr_debug("failed to toggle chg done override rc=%d\n", rc);
		return rc;
	}

	return rc;
}

#define QPNP_CHG_VBATDET_MIN_MV	3240
#define QPNP_CHG_VBATDET_MAX_MV	5780
#define QPNP_CHG_VBATDET_STEP_MV	20
static int
qpnp_chg_vbatdet_set(struct qpnp_chg_chip *chip, int vbatdet_mv)
{
	u8 temp;

	if (vbatdet_mv < QPNP_CHG_VBATDET_MIN_MV
			|| vbatdet_mv > QPNP_CHG_VBATDET_MAX_MV) {
		pr_err("bad mV=%d asked to set\n", vbatdet_mv);
		return -EINVAL;
	}
	temp = (vbatdet_mv - QPNP_CHG_VBATDET_MIN_MV)
			/ QPNP_CHG_VBATDET_STEP_MV;
#ifdef CONFIG_LGE_PM_4_25V_CHARGING_START
	pr_err("qpnp_chg_vbatdet_set voltage=%d setting %02x\n", vbatdet_mv, temp);
#else
	pr_debug("voltage=%d setting %02x\n", vbatdet_mv, temp);
#endif
	return qpnp_chg_write(chip, &temp,
		chip->chgr_base + CHGR_VBAT_DET, 1);
}

static void
qpnp_chg_set_appropriate_vbatdet(struct qpnp_chg_chip *chip)
{
	if (chip->bat_is_cool)
		qpnp_chg_vbatdet_set(chip, chip->cool_bat_mv
			- chip->resume_delta_mv);
	else if (chip->bat_is_warm)
		qpnp_chg_vbatdet_set(chip, chip->warm_bat_mv
			- chip->resume_delta_mv);
	else if (chip->resuming_charging)
		qpnp_chg_vbatdet_set(chip, chip->max_voltage_mv
			+ chip->resume_delta_mv);
	else
		qpnp_chg_vbatdet_set(chip, chip->max_voltage_mv
			- chip->resume_delta_mv);
}

#ifdef CONFIG_LGE_PM_WORKAROUND_WEAK_CHARGER_REMOVAL_DETECTION
static int
qpnp_chg_ovpfet_on(struct qpnp_chg_chip *chip, bool on)
{
	int rc = 0;

	rc = qpnp_chg_masked_write(chip,
		chip->usb_chgpth_base + SEC_ACCESS,
		0xFF,
		0xA5, 1);

	if (rc)
		pr_err("[WCRD] failed to write SEC_ACCESS rc=%d\n", rc);

	if (on) {
		pr_info("[WCRD] Set 0x1350 as 0x00 ------------- \n");
		rc = qpnp_chg_masked_write(chip,
			chip->usb_chgpth_base + 0x50,
			0xFF,
			0x00, 1);
		if (rc)
			pr_err("[WCRD] failed to write hidden_reg rc=%d\n", rc);
	} else {
		pr_info("[WCRD] Set 0x1350 as 0x02 ------------- \n");
		rc = qpnp_chg_masked_write(chip,
			chip->usb_chgpth_base + 0x50,
			0xFF,
			0x02, 1);
		if (rc)
			pr_err("[WCRD] failed to write hidden_reg rc=%d\n", rc);
	}

	return rc;
}

static int
qpnp_chg_ovpfet_get(struct qpnp_chg_chip *chip)
{
	int rc = 0;
	u8 hidden_reg;

	rc = qpnp_chg_read(chip, &hidden_reg,
			chip->usb_chgpth_base + 0x50, 1);
	if (rc) {
		pr_err("[WCRD] failed to read hidden_reg rc=%d\n", rc);
		return rc;
	}

	return hidden_reg;
}
#endif

static void
qpnp_arb_stop_work(struct work_struct *work)
{
#if defined(CONFIG_MACH_MSM8926_JAGC_SPR) || defined(CONFIG_MACH_MSM8926_JAGNM_ATT) || \
	defined(CONFIG_MACH_MSM8926_JAGNM_BELL) || defined(CONFIG_MACH_MSM8926_JAGC_SPR) || \
	defined(CONFIG_MACH_MSM8926_JAGNM_RGS)
	bool check_usb_ta = false;
#endif
	struct delayed_work *dwork = to_delayed_work(work);
	struct qpnp_chg_chip *chip = container_of(dwork,
				struct qpnp_chg_chip, arb_stop_work);

#ifdef CONFIG_LGE_PM_WORKAROUND_WEAK_CHARGER_REMOVAL_DETECTION
	int rc;
	u8 usb_sts;
	int vbat_mv;
	int ovp_sts;
	bool polling_exit = 1;

	rc = qpnp_chg_read(chip, &usb_sts,
			INT_RT_STS(chip->usb_chgpth_base), 1);
	if (rc)
		pr_err("[WCRD] failed to read usb_chgpth_sts rc=%d\n", rc);


	//see if USBIN is still valid and chg_gone is still true
	if(qpnp_chg_is_usb_chg_plugged_in(chip)&& (usb_sts & CHG_GONE_IRQ))
	{
		if (!wake_lock_active(&chip->weak_chg_wake_lock)){
			pr_info("[WCRD] Acquire wakelock\n");
			wake_lock(&chip->weak_chg_wake_lock);
		}

		ovp_sts = qpnp_chg_ovpfet_get(chip);
		if (ovp_sts != 0x02)
			/* test if it's the weak charger by opening the ovp_fet */
			qpnp_chg_ovpfet_on(chip, false);

		if(qpnp_chg_is_usb_chg_plugged_in(chip))
		{
			// weak charger connected because USBIN is still valid after OVP_FET is open
			pr_info("[WCRD] weak charger detected, please replace it\n");

			/* periodical check until the weak charger is removed */
			schedule_delayed_work(&chip->arb_stop_work,
					msecs_to_jiffies(3000));
			polling_exit = 0;

			vbat_mv = get_prop_battery_voltage_now(chip) / 1000;
			/* periodical battery voltage check for recharging */
			if (vbat_mv > (chip->max_voltage_mv - chip->resume_delta_mv))
				return;
			pr_info("[WCRD] vbat_mv = %d -> close FET \n", vbat_mv);
		}
		msleep(500);
	}

	ovp_sts = qpnp_chg_ovpfet_get(chip);
	if(ovp_sts != 0x00)
	{
		qpnp_chg_ovpfet_on(chip,true);
		pr_info("[WCRD] close ovp_fet\n");
		msleep(500);
	}

	if (polling_exit){
		if (wake_lock_active(&chip->weak_chg_wake_lock)){
			pr_info("[WCRD] Release wakelock\n");
			wake_unlock(&chip->weak_chg_wake_lock);
		}
	}
#endif	/* CONFIG_LGE_PM_WORKAROUND_WEAK_CHARGER_REMOVAL_DETECTION */

#if defined(CONFIG_MACH_MSM8926_JAGC_SPR) || defined(CONFIG_MACH_MSM8926_JAGNM_ATT) || \
	defined(CONFIG_MACH_MSM8926_JAGNM_BELL) || defined(CONFIG_MACH_MSM8926_JAGC_SPR) || \
	defined(CONFIG_MACH_MSM8926_JAGNM_RGS)
	if (qpnp_chg_is_usb_chg_plugged_in(chip) || qpnp_chg_is_dc_chg_plugged_in(chip))
		check_usb_ta = true;
	else
		check_usb_ta = false;

	if (check_usb_ta && !chip->chg_done)
#else
	if (!chip->chg_done)
#endif
		qpnp_chg_charge_en(chip, !chip->charging_disabled);

	qpnp_chg_force_run_on_batt(chip, chip->charging_disabled);
}

static void
qpnp_bat_if_adc_measure_work(struct work_struct *work)
{
	struct qpnp_chg_chip *chip = container_of(work,
				struct qpnp_chg_chip, adc_measure_work);

	if (qpnp_adc_tm_channel_measure(chip->adc_tm_dev, &chip->adc_param))
		pr_err("request ADC error\n");
}

static void
qpnp_bat_if_adc_disable_work(struct work_struct *work)
{
	struct qpnp_chg_chip *chip = container_of(work,
				struct qpnp_chg_chip, adc_disable_work);

	qpnp_adc_tm_disable_chan_meas(chip->adc_tm_dev, &chip->adc_param);
}

#define EOC_CHECK_PERIOD_MS	10000
static irqreturn_t
qpnp_chg_vbatdet_lo_irq_handler(int irq, void *_chip)
{
	struct qpnp_chg_chip *chip = _chip;
	u8 chg_sts = 0;
	int rc;

	pr_err("vbatdet-lo triggered\n");

	rc = qpnp_chg_read(chip, &chg_sts, INT_RT_STS(chip->chgr_base), 1);
	if (rc)
		pr_err("failed to read chg_sts rc=%d\n", rc);

	pr_err("chg_done chg_sts: 0x%x triggered\n", chg_sts);
	if (!chip->charging_disabled && (chg_sts & FAST_CHG_ON_IRQ)) {
#if defined (CONFIG_MACH_MSM8226_E8WIFI) ||defined (CONFIG_MACH_MSM8926_E8LTE) || \
    defined (CONFIG_MACH_MSM8926_E9LTE) || defined (CONFIG_MACH_MSM8926_E7LTE_VZW_US) || \
    defined (CONFIG_MACH_MSM8926_T8LTE)
	if(lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO){
		if(chip->ac_online && !chip->chg_done){
			pr_err("DEBUG : Resume on EOC state. Turns on EXT OVP FET\n");
			lge_set_chg_path_to_external();
		}
	}
#endif
		schedule_delayed_work(&chip->eoc_work,
			msecs_to_jiffies(EOC_CHECK_PERIOD_MS));
		pm_stay_awake(chip->dev);
	}
	qpnp_chg_disable_irq(&chip->chg_vbatdet_lo);

	pr_debug("psy changed usb_psy\n");
	power_supply_changed(chip->usb_psy);
	if (chip->dc_chgpth_base) {
		pr_debug("psy changed dc_psy\n");
		power_supply_changed(&chip->dc_psy);
	}
	if (chip->bat_if_base) {
		pr_debug("psy changed batt_psy\n");
		power_supply_changed(&chip->batt_psy);
	}
	return IRQ_HANDLED;
}

#ifdef CONFIG_LGE_PM_WORKAROUND_USB_VALID_BY_REVERSE_BOOST
static void
qpnp_property_work(struct work_struct *work)
{
	int usb_present = false;

	struct delayed_work *dwork = to_delayed_work(work);
	struct qpnp_chg_chip *chip = container_of(dwork, struct qpnp_chg_chip, property_work);
	usb_present = qpnp_chg_is_usb_chg_plugged_in(chip);

	if (chip->usb_present ^ usb_present) {
		pr_err("USB UVD Detection fail restored. %d -> %d\n", chip->usb_present, usb_present);
		chip->usb_present = usb_present;
		power_supply_set_present(chip->usb_psy, chip->usb_present);
	}
}
#endif

#ifdef CONFIG_LGE_PM	/* Using Interanl & External OVP control */
void lge_power_supply_set_type(int supply_type);
int lge_power_supply_get_type(void);
#endif

#define ARB_STOP_WORK_MS	1000
static irqreturn_t
qpnp_chg_usb_chg_gone_irq_handler(int irq, void *_chip)
{
	struct qpnp_chg_chip *chip = _chip;
	u8 usb_sts;
	int rc;

	rc = qpnp_chg_read(chip, &usb_sts,
			INT_RT_STS(chip->usb_chgpth_base), 1);
	if (rc)
		pr_err("failed to read usb_chgpth_sts rc=%d\n", rc);

#ifdef CONFIG_LGE_PM_PWR_KEY_FOR_CHG_LOGO
    pr_err("============ QPNP CHG GONE ==============\n");
    if (lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO) {
		pm_stay_awake(chip->dev);
    }
#endif

	pr_debug("chg_gone triggered\n");
/* LGE_CHANGE_S */
#ifdef CONFIG_LGE_PM_CHARGING_EXTERNAL_OVP
    lge_set_chg_path_to_internal();
#endif

#ifdef CONFIG_LGE_PM
    chip->chg_fail_irq_happen = false;
#endif

#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
	chip->is_charger_changed_from_irq = true;
#endif

#ifdef CONFIG_LGE_PM_VZW_FAST_CHG
	chip->is_vzw_slow_charging = false;
#endif
/* LGE_CHANGE_E */
	if ((qpnp_chg_is_usb_chg_plugged_in(chip)
			|| qpnp_chg_is_dc_chg_plugged_in(chip))
			&& (usb_sts & CHG_GONE_IRQ)) {
		if (ext_ovp_isns_present) {
			pr_debug("EXT OVP IC ISNS disabled due to ARB WA\n");
			gpio_direction_output(chip->ext_ovp_isns_gpio, 0);
		}

		qpnp_chg_charge_en(chip, 0);

		qpnp_chg_force_run_on_batt(chip, 1);
#ifdef CONFIG_LGE_PM_4_25V_CHARGING_START
		schedule_work(&chip->resume_check_work);
#endif
		schedule_delayed_work(&chip->arb_stop_work,
			msecs_to_jiffies(ARB_STOP_WORK_MS));
	}
#ifdef CONFIG_LGE_WIRELESS_CHARGER_RT9536
	else {
		schedule_work(&chip->wlc_enable_work);
	}
#endif

#ifdef CONFIG_LGE_PM_WORKAROUND_USB_VALID_BY_REVERSE_BOOST
	schedule_delayed_work(&chip->property_work,
		msecs_to_jiffies(100));
#endif

	return IRQ_HANDLED;
}

static irqreturn_t
qpnp_chg_usb_usb_ocp_irq_handler(int irq, void *_chip)
{
	struct qpnp_chg_chip *chip = _chip;

	pr_err("usb-ocp triggered\n");

	schedule_work(&chip->ocp_clear_work);

	return IRQ_HANDLED;
}

#define BOOST_ILIMIT_MIN	0x07
#define BOOST_ILIMIT_DEF	0x02
#define BOOST_ILIMT_MASK	0xFF
static void
qpnp_chg_ocp_clear_work(struct work_struct *work)
{
	int rc;
	u8 usb_sts;
	struct qpnp_chg_chip *chip = container_of(work,
		struct qpnp_chg_chip, ocp_clear_work);

	if (chip->type == SMBBP) {
		rc = qpnp_chg_masked_write(chip,
				chip->boost_base + BOOST_ILIM,
				BOOST_ILIMT_MASK,
				BOOST_ILIMIT_MIN, 1);
		if (rc) {
			pr_err("Failed to turn configure ilim rc = %d\n", rc);
			return;
		}
	}

	rc = qpnp_chg_masked_write(chip,
			chip->usb_chgpth_base + USB_OCP_CLR,
			OCP_CLR_BIT,
			OCP_CLR_BIT, 1);
	if (rc)
		pr_err("Failed to clear OCP bit rc = %d\n", rc);

	/* force usb ovp fet off */
	rc = qpnp_chg_masked_write(chip,
			chip->usb_chgpth_base + CHGR_USB_USB_OTG_CTL,
			USB_OTG_EN_BIT,
			USB_OTG_EN_BIT, 1);
	if (rc)
		pr_err("Failed to turn off usb ovp rc = %d\n", rc);

	if (chip->type == SMBBP) {
		/* Wait for OCP circuitry to be powered up */
		msleep(100);
		rc = qpnp_chg_read(chip, &usb_sts,
				INT_RT_STS(chip->usb_chgpth_base), 1);
		if (rc) {
			pr_err("failed to read interrupt sts %d\n", rc);
			return;
		}

		if (usb_sts & COARSE_DET_USB_IRQ) {
			rc = qpnp_chg_masked_write(chip,
				chip->boost_base + BOOST_ILIM,
				BOOST_ILIMT_MASK,
				BOOST_ILIMIT_DEF, 1);
			if (rc) {
				pr_err("Failed to set ilim rc = %d\n", rc);
				return;
			}
		} else {
			pr_warn_ratelimited("USB short to GND detected!\n");
		}
	}
}

#define QPNP_CHG_VDDMAX_MIN		3400
#define QPNP_CHG_V_MIN_MV		3240
#define QPNP_CHG_V_MAX_MV		4500
#define QPNP_CHG_V_STEP_MV		10
#define QPNP_CHG_BUCK_TRIM1_STEP	10
#define QPNP_CHG_BUCK_VDD_TRIM_MASK	0xF0
static int
qpnp_chg_vddmax_and_trim_set(struct qpnp_chg_chip *chip,
		int voltage, int trim_mv)
{
	int rc, trim_set;
	u8 vddmax = 0, trim = 0;

#ifdef CONFIG_LGE_PM_NEED_TO_MONITORING_QCT_PATCH	/* Removing 100mV Drop in VDDMAX */
	u8 temp = 0;
#endif

	if (voltage < QPNP_CHG_VDDMAX_MIN
			|| voltage > QPNP_CHG_V_MAX_MV) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}

	vddmax = (voltage - QPNP_CHG_V_MIN_MV) / QPNP_CHG_V_STEP_MV;
	rc = qpnp_chg_write(chip, &vddmax, chip->chgr_base + CHGR_VDD_MAX, 1);
	if (rc) {
		pr_err("Failed to write vddmax: %d\n", rc);
		return rc;
	}
#ifdef CONFIG_LGE_PM_NEED_TO_MONITORING_QCT_PATCH	/* Removing 100mV Drop in VDDMA */
	temp = 0x4A;
	qpnp_chg_write(chip, &temp, chip->chgr_base + CHGR_VDDMAX_GSM_ADJ, 1);
#endif
	rc = qpnp_chg_masked_write(chip,
		chip->buck_base + SEC_ACCESS,
		0xFF,
		0xA5, 1);
	if (rc) {
		pr_err("failed to write SEC_ACCESS rc=%d\n", rc);
		return rc;
	}
	trim_set = clamp((int)chip->trim_center
			+ (trim_mv / QPNP_CHG_BUCK_TRIM1_STEP),
			0, 0xF);
	trim = (u8)trim_set << 4;
	rc = qpnp_chg_masked_write(chip,
		chip->buck_base + BUCK_CTRL_TRIM1,
		QPNP_CHG_BUCK_VDD_TRIM_MASK,
		trim, 1);
	if (rc) {
		pr_err("Failed to write buck trim1: %d\n", rc);
		return rc;
	}
	pr_debug("voltage=%d+%d setting vddmax: %02x, trim: %02x\n",
			voltage, trim_mv, vddmax, trim);
	return 0;
}

static int
qpnp_chg_vddmax_get(struct qpnp_chg_chip *chip)
{
	int rc;
	u8 vddmax = 0;

	rc = qpnp_chg_read(chip, &vddmax, chip->chgr_base + CHGR_VDD_MAX, 1);
	if (rc) {
		pr_err("Failed to write vddmax: %d\n", rc);
		return rc;
	}

	return QPNP_CHG_V_MIN_MV + (int)vddmax * QPNP_CHG_V_STEP_MV;
}

/* JEITA compliance logic */
static void
qpnp_chg_set_appropriate_vddmax(struct qpnp_chg_chip *chip)
{
	if (chip->bat_is_cool)
		qpnp_chg_vddmax_and_trim_set(chip, chip->cool_bat_mv,
				chip->delta_vddmax_mv);
	else if (chip->bat_is_warm)
		qpnp_chg_vddmax_and_trim_set(chip, chip->warm_bat_mv,
				chip->delta_vddmax_mv);
	else
		qpnp_chg_vddmax_and_trim_set(chip, chip->max_voltage_mv,
				chip->delta_vddmax_mv);
}

static void
qpnp_usbin_health_check_work(struct work_struct *work)
{
	int usbin_health = 0;
	u8 psy_health_sts = 0;
	struct delayed_work *dwork = to_delayed_work(work);
	struct qpnp_chg_chip *chip = container_of(dwork,
				struct qpnp_chg_chip, usbin_health_check);

	usbin_health = qpnp_chg_check_usbin_health(chip);
	spin_lock(&chip->usbin_health_monitor_lock);
	if (chip->usbin_health != usbin_health) {
		pr_debug("health_check_work: pr_usbin_health = %d, usbin_health = %d",
			chip->usbin_health, usbin_health);
		chip->usbin_health = usbin_health;
		if (usbin_health == USBIN_OVP)
			psy_health_sts = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else if (usbin_health == USBIN_OK)
			psy_health_sts = POWER_SUPPLY_HEALTH_GOOD;
		power_supply_set_health_state(chip->usb_psy, psy_health_sts);
		power_supply_changed(chip->usb_psy);
	}
	/* enable OVP monitor in usb valid after coarse-det complete */
	chip->usb_valid_check_ovp = true;
	spin_unlock(&chip->usbin_health_monitor_lock);
	return;
}

#define USB_VALID_DEBOUNCE_TIME_MASK		0x3
#define USB_DEB_BYPASS		0x0
#define USB_DEB_5MS			0x1
#define USB_DEB_10MS		0x2
#define USB_DEB_20MS		0x3
static irqreturn_t
qpnp_chg_coarse_det_usb_irq_handler(int irq, void *_chip)
{
	struct qpnp_chg_chip *chip = _chip;
	int host_mode, rc = 0;
	int debounce[] = {
		[USB_DEB_BYPASS] = 0,
		[USB_DEB_5MS] = 5,
		[USB_DEB_10MS] = 10,
		[USB_DEB_20MS] = 20 };
	u8 ovp_ctl;
	bool usb_coarse_det;

	host_mode = qpnp_chg_is_otg_en_set(chip);
	usb_coarse_det = qpnp_chg_check_usb_coarse_det(chip);
	pr_err("usb coarse-det triggered: %d host_mode: %d\n",
			usb_coarse_det, host_mode);

	if (host_mode)
		return IRQ_HANDLED;
	/* ignore to monitor OVP in usbin valid irq handler
	 if the coarse-det fired first, do the OVP state monitor
	 in the usbin_health_check work, and after the work,
	 enable monitor OVP in usbin valid irq handler */
	chip->usb_valid_check_ovp = false;
	if (chip->usb_coarse_det ^ usb_coarse_det) {
		chip->usb_coarse_det = usb_coarse_det;
		if (usb_coarse_det) {
			/* usb coarse-det rising edge, check the usbin_valid
			debounce time setting, and start a delay work to
			check the OVP status*/
			rc = qpnp_chg_read(chip, &ovp_ctl,
					chip->usb_chgpth_base + USB_OVP_CTL, 1);

			if (rc) {
				pr_err("spmi read failed: addr=%03X, rc=%d\n",
					chip->usb_chgpth_base + USB_OVP_CTL,
					rc);
				return rc;
			}
			ovp_ctl = ovp_ctl & USB_VALID_DEBOUNCE_TIME_MASK;
			schedule_delayed_work(&chip->usbin_health_check,
					msecs_to_jiffies(debounce[ovp_ctl]));
		} else {
			/* usb coarse-det rising edge, set the usb psy health
			status to unknown */
			pr_debug("usb coarse det clear, set usb health to unknown\n");
			chip->usbin_health = USBIN_UNKNOW;
			power_supply_set_health_state(chip->usb_psy,
				POWER_SUPPLY_HEALTH_UNKNOWN);
			power_supply_changed(chip->usb_psy);
		}

	}
	return IRQ_HANDLED;
}

#define BATFET_LPM_MASK		0xC0
#define BATFET_LPM		0x40
#define BATFET_NO_LPM		0x00
static int
qpnp_chg_regulator_batfet_set(struct qpnp_chg_chip *chip, bool enable)
{
	int rc = 0;

	if (chip->charging_disabled || !chip->bat_if_base)
		return rc;

#ifdef CONFIG_LGE_PM_DISABLE_ULPM_MODE
	enable = 1;
#endif

	if (chip->type == SMBB)
		rc = qpnp_chg_masked_write(chip,
			chip->bat_if_base + CHGR_BAT_IF_SPARE,
			BATFET_LPM_MASK,
			enable ? BATFET_NO_LPM : BATFET_LPM, 1);
	else
		rc = qpnp_chg_masked_write(chip,
			chip->bat_if_base + CHGR_BAT_IF_BATFET_CTRL4,
			BATFET_LPM_MASK,
			enable ? BATFET_NO_LPM : BATFET_LPM, 1);

	return rc;
}

#define USB_WALL_THRESHOLD_MA	500
#define ENUM_T_STOP_BIT		BIT(0)
#define USB_5V_UV	5000000
#define USB_9V_UV	9000000
#ifdef CONFIG_TOUCHSCREEN_ATMEL_S540
void trigger_early_baseline_state_machine(int plug_in);
#endif
#if defined (CONFIG_MACH_MSM8926_B2LN_KR) || defined (CONFIG_MACH_MSM8926_JAGN_KR) || defined (CONFIG_MACH_MSM8926_AKA_CN) || defined(CONFIG_MACH_MSM8926_AKA_KR)
static irqreturn_t
qpnp_chg_usb_usbin_valid_irq_handler(int irq, void *_chip)
{
	struct qpnp_chg_chip *chip = _chip;
	int usb_present, host_mode;
#ifdef CONFIG_LGE_PM
	if ((chip->uevent_wake_lock.ws.name) != NULL)
		wake_lock_timeout(&chip->uevent_wake_lock, HZ*2);
#endif

	usb_present = qpnp_chg_is_usb_chg_plugged_in(chip);
	host_mode = qpnp_chg_is_otg_en_set(chip);
	printk("usbin-valid triggered: %d host_mode: %d\n",
		usb_present, host_mode);

#if !defined(CONFIG_LGE_SUPPORT_TYPE_A_USB)
	/* In host mode notifications cmoe from USB supply */
	if (host_mode)
		return IRQ_HANDLED;
#endif

	schedule_delayed_work(&chip->usbin_valid_work,
							msecs_to_jiffies(0));
	return IRQ_HANDLED;
}

static int
qpnp_chg_get_vbus_level_uv(struct qpnp_chg_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	rc = qpnp_vadc_read(chip->vadc_dev, USBIN, &results);
	if (rc) {
		pr_err("Unable to read vbat rc=%d\n", rc);
		return 0;
	}
	return results.physical;
}

static void
qpnp_usbin_valid_work(struct work_struct *work)
{
    struct delayed_work *dwork = to_delayed_work(work);
    struct qpnp_chg_chip *chip = container_of(dwork,
	struct qpnp_chg_chip, usbin_valid_work);

    int usb_present,  usbin_health;
	int vbus_usb_level_uv = 0 ;
	int vbus_usb_level_mv = 0 ;
	u8 psy_health_sts;
	usb_present = qpnp_chg_is_usb_chg_plugged_in(chip);

	vbus_usb_level_uv = qpnp_chg_get_vbus_level_uv(chip);
	if (0 == vbus_usb_level_uv) {
		printk("Unable to read vbus_usb_level_uv=%d\n", vbus_usb_level_uv);
	}

	vbus_usb_level_mv = (vbus_usb_level_uv / 1000);
	printk("usbin_valid_work triggered: pre_usb_present: %d cur_usb_present: %d count_count: %d vbus_usb_level_mv: %d\n",
			chip->usb_present, usb_present, count_count, vbus_usb_level_mv);

	if ((usb_present == 1) && (vbus_usb_level_mv < VBUS_USB_THRESHOLD)) {
		usb_present = 0 ;
		printk("usbin_valid_work retry: pre_usb_present: %d cur_usb_present: %d count_count: %d vbus_usb_level_mv: %d\n",
			chip->usb_present, usb_present, count_count, vbus_usb_level_mv);
	}

	if (chip->usb_present ^ usb_present) {
		count_count = 0 ;
#ifdef CONFIG_TOUCHSCREEN_ATMEL_S540
		trigger_early_baseline_state_machine(usb_present);
#endif
#ifdef CONFIG_LGE_WIRELESS_CHARGER_RT9536
		schedule_work(&chip->wlc_disable_work);
#endif
		chip->usb_present = usb_present;
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
		chip->is_charger_changed_from_irq = true;
		schedule_delayed_work(&chip->battemp_work, HZ*1);
#endif
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
		chip->is_charger_changed_from_irq = true;

		if (wake_lock_active(&chip->lcs_wake_lock))
			wake_unlock(&chip->lcs_wake_lock);
#endif
		if (!usb_present) {
#ifdef CONFIG_LGE_PM_VZW_FAST_CHG
			if (usb_chg_state == IS_OPEN_TA || chg_state == VZW_NOT_CHARGING || chg_state == VZW_USB_DRIVER_UNINSTALLED) {
				lge_usb_config_finish = 0;
				usb_chg_state = IS_USB_DRIVER_UNINSTALLED;
				chg_state = VZW_NO_CHARGER;
				vzw_chg_present = NOT_PRESENT;
#ifdef CONFIG_LGE_PM_VZW_LLK
				temp_state = 0;
				llk_monitor_soc_flag = false;
				llk_stop_chg_flag = false;
#endif
				power_supply_changed(&chip->batt_psy);
			}
			/* input_current_check_work should be cancelled if charger is removed before checking input current by AICL */
			schedule_work(&chip->cancel_input_current_check_work);
#endif
			/* when a valid charger inserted, and increase the
			 *  charger voltage to OVP threshold, then
			 *  usb_in_valid falling edge interrupt triggers.
			 *  So we handle the OVP monitor here, and ignore
			 *  other health state changes */
			qpnp_chg_disable_irq(&chip->chg_vbatdet_lo);
			if (chip->ovp_monitor_enable &&
				       (chip->usb_valid_check_ovp)) {
				usbin_health =
					qpnp_chg_check_usbin_health(chip);
				if ((chip->usbin_health != usbin_health)
					&& (usbin_health == USBIN_OVP)) {
					chip->usbin_health = usbin_health;
					psy_health_sts =
					POWER_SUPPLY_HEALTH_OVERVOLTAGE;
					power_supply_set_health_state(
						chip->usb_psy,
						psy_health_sts);
					power_supply_changed(chip->usb_psy);
				}
			}
			if (!qpnp_chg_is_dc_chg_plugged_in(chip)) {
				chip->delta_vddmax_mv = 0;
				qpnp_chg_set_appropriate_vddmax(chip);
				chip->chg_done = false;
			}
			qpnp_chg_usb_suspend_enable(chip, 0);
			qpnp_chg_iusbmax_set(chip, QPNP_CHG_I_MAX_MIN_100);
			chip->prev_usb_max_ma = -EINVAL;
			chip->aicl_settled = false;
		} else {
			/* when OVP clamped usbin, and then decrease
			 * the charger voltage to lower than the OVP
			 * threshold, a usbin_valid rising edge
			 * interrupt triggered. So we change the usb
			 * psy health state back to good */
			if (chip->ovp_monitor_enable &&
				       (chip->usb_valid_check_ovp)) {
				usbin_health =
					qpnp_chg_check_usbin_health(chip);
				if ((chip->usbin_health != usbin_health)
					&& (usbin_health == USBIN_OK)) {
					chip->usbin_health = usbin_health;
					psy_health_sts =
						POWER_SUPPLY_HEALTH_GOOD;
					power_supply_set_health_state(
						chip->usb_psy,
						psy_health_sts);
					power_supply_changed(chip->usb_psy);
				}
			}

			if (!qpnp_chg_is_dc_chg_plugged_in(chip)) {
				chip->delta_vddmax_mv = 0;
				qpnp_chg_set_appropriate_vddmax(chip);
			}
			schedule_delayed_work(&chip->eoc_work,
				msecs_to_jiffies(EOC_CHECK_PERIOD_MS));
#ifndef CONFIG_LGE_PM_4_25V_CHARGING_START
			schedule_work(&chip->soc_check_work);
#endif

#ifdef CONFIG_LGE_PM_4_25V_CHARGING_START
			schedule_work(&chip->resume_check_work);

			/* pr_err("entered qpnp_chg_usb_usbin_valid_irq_handler\n"); */
#endif

#ifdef CONFIG_LGE_PM_VZW_FAST_CHG
			if (lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO)
				schedule_delayed_work(&chip->input_current_check_work, HZ*30);
			else
				schedule_delayed_work(&chip->input_current_check_work, HZ*10);
			chip->is_vzw_slow_charging = false;
#endif
		}
		power_supply_set_present(chip->usb_psy, chip->usb_present);
		schedule_work(&chip->batfet_lcl_work);
	} else if ((chip->usb_present == 1) && (usb_present == 1) && (count_count <= 2)) {
			count_count += 1 ;
			schedule_delayed_work(&chip->usbin_valid_work,
									msecs_to_jiffies(50));
	} else if ((chip->usb_present == 1) && (usb_present == 1) && (count_count == 3)) {
			count_count += 1 ;
			schedule_delayed_work(&chip->usbin_valid_work,
									msecs_to_jiffies(300));
	} else if (count_count == 4) {
			count_count = 0 ;
			printk("count_count is 4 = %d\n", count_count);
	}
}
#else
static irqreturn_t
qpnp_chg_usb_usbin_valid_irq_handler(int irq, void *_chip)
{
	struct qpnp_chg_chip *chip = _chip;
	int usb_present, host_mode, usbin_health;
	u8 psy_health_sts;

#ifdef CONFIG_LGE_PM
	if ((chip->uevent_wake_lock.ws.name) != NULL)
		wake_lock_timeout(&chip->uevent_wake_lock, HZ*2);
#endif

	usb_present = qpnp_chg_is_usb_chg_plugged_in(chip);
	host_mode = qpnp_chg_is_otg_en_set(chip);
	pr_info("[DEBUG] usbin-valid triggered: %d host_mode: %d\n",
		usb_present, host_mode);
	/* In host mode notifications cmoe from USB supply */
	if (host_mode)
		return IRQ_HANDLED;

#ifdef CONFIG_LGE_PM
	/* Sometimes when USB cable is removed, usb_present value is still true
	 * because CHGR_STATUS register does not changed after remove USB cable
	 * And then usb_present check one more time when usb_present does not change */
	if (!(chip->usb_present ^ usb_present)) {
		usb_present = qpnp_chg_is_usb_chg_plugged_in(chip);
		pr_err("need to check usbin-valid triggered: %d \n", usb_present);
	}
#endif

	if (chip->usb_present ^ usb_present) {
#ifdef CONFIG_TOUCHSCREEN_ATMEL_S540
		trigger_early_baseline_state_machine(usb_present);
#endif
#ifdef CONFIG_LGE_WIRELESS_CHARGER_RT9536
		schedule_work(&chip->wlc_disable_work);
#endif
		chip->aicl_settled = false;
		chip->usb_present = usb_present;
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
		chip->is_charger_changed_from_irq = true;
		schedule_delayed_work(&chip->battemp_work, HZ*1);
#endif
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
		chip->is_charger_changed_from_irq = true;

		if (wake_lock_active(&chip->lcs_wake_lock))
			wake_unlock(&chip->lcs_wake_lock);
#endif
		if (!usb_present) {
#ifdef CONFIG_LGE_PM_VZW_FAST_CHG
			if (usb_chg_state == IS_OPEN_TA || chg_state == VZW_NOT_CHARGING || chg_state == VZW_USB_DRIVER_UNINSTALLED) {
				lge_usb_config_finish = 0;
				usb_chg_state = IS_USB_DRIVER_UNINSTALLED;
				chg_state = VZW_NO_CHARGER;
				vzw_chg_present = NOT_PRESENT;
#ifdef CONFIG_LGE_PM_VZW_LLK
				temp_state = 0;
				llk_monitor_soc_flag = false;
				llk_stop_chg_flag = false;
#endif
				power_supply_changed(&chip->batt_psy);
			}
			/* input_current_check_work should be cancelled if charger is removed before checking input current by AICL */
			schedule_work(&chip->cancel_input_current_check_work);
#endif
			/* when a valid charger inserted, and increase the
			 *  charger voltage to OVP threshold, then
			 *  usb_in_valid falling edge interrupt triggers.
			 *  So we handle the OVP monitor here, and ignore
			 *  other health state changes */
			if (chip->ovp_monitor_enable &&
				       (chip->usb_valid_check_ovp)) {
				usbin_health =
					qpnp_chg_check_usbin_health(chip);
				if ((chip->usbin_health != usbin_health)
					&& (usbin_health == USBIN_OVP)) {
					chip->usbin_health = usbin_health;
					psy_health_sts =
					POWER_SUPPLY_HEALTH_OVERVOLTAGE;
					power_supply_set_health_state(
						chip->usb_psy,
						psy_health_sts);
					power_supply_changed(chip->usb_psy);
				}
			}
			if (!qpnp_chg_is_dc_chg_plugged_in(chip))
				chip->chg_done = false;

			if (!qpnp_is_dc_higher_prio(chip))
				qpnp_chg_idcmax_set(chip, chip->maxinput_dc_ma);

			qpnp_chg_usb_suspend_enable(chip, 0);
			qpnp_chg_iusbmax_set(chip, QPNP_CHG_I_MAX_MIN_100);
			qpnp_chg_iusb_trim_set(chip, chip->usb_trim_default);
			chip->prev_usb_max_ma = -EINVAL;
		} else {
			/* when OVP clamped usbin, and then decrease
			 * the charger voltage to lower than the OVP
			 * threshold, a usbin_valid rising edge
			 * interrupt triggered. So we change the usb
			 * psy health state back to good */
			if (chip->ovp_monitor_enable &&
				       (chip->usb_valid_check_ovp)) {
				usbin_health =
					qpnp_chg_check_usbin_health(chip);
				if ((chip->usbin_health != usbin_health)
					&& (usbin_health == USBIN_OK)) {
					chip->usbin_health = usbin_health;
					psy_health_sts =
						POWER_SUPPLY_HEALTH_GOOD;
					power_supply_set_health_state(
						chip->usb_psy,
						psy_health_sts);
					power_supply_changed(chip->usb_psy);
				}
			}

			schedule_delayed_work(&chip->eoc_work,
				msecs_to_jiffies(EOC_CHECK_PERIOD_MS));
#ifndef CONFIG_LGE_PM_4_25V_CHARGING_START
			schedule_work(&chip->soc_check_work);
#endif

#ifdef CONFIG_LGE_PM_4_25V_CHARGING_START
			schedule_work(&chip->resume_check_work);

			/* pr_err("entered qpnp_chg_usb_usbin_valid_irq_handler\n"); */
#endif

#ifdef CONFIG_LGE_PM_VZW_FAST_CHG
			if (lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO)
				schedule_delayed_work(&chip->input_current_check_work, HZ*30);
			else
				schedule_delayed_work(&chip->input_current_check_work, HZ*10);
			chip->is_vzw_slow_charging = false;
#endif
		}

		power_supply_set_present(chip->usb_psy, chip->usb_present);
		schedule_work(&chip->batfet_lcl_work);
	}

	return IRQ_HANDLED;
}
#endif

#define BUCK_VIN_LOOP_CMP_OVRD_MASK	0x30
static int
qpnp_chg_bypass_vchg_loop_debouncer(struct qpnp_chg_chip *chip, bool bypass)
{
	int rc;
	u8 value = bypass ? 0x10 : 0;

	pr_debug("bypass vchg_loop debouncer: %d\n", bypass);

	rc = qpnp_chg_masked_write(chip, chip->buck_base + SEC_ACCESS,
					0xFF, 0xA5, 1);
	if (rc) {
		pr_err("failed to write SEC_ACCESS register, rc = %d\n", rc);
		return rc;
	}

	rc = qpnp_chg_masked_write(chip,
			chip->buck_base + CHGR_BUCK_COMPARATOR_OVRIDE_2,
			BUCK_VIN_LOOP_CMP_OVRD_MASK, value, 1);
	if (rc)
		pr_err("failed to write BUCK_COMP_OVRIDE_2, rc = %d\n", rc);

	return rc;
}

static int
qpnp_chg_vchg_loop_debouncer_setting_get(struct qpnp_chg_chip *chip)
{
	int rc;
	u8 value;

	rc = qpnp_chg_read(chip, &value,
			chip->buck_base + CHGR_BUCK_COMPARATOR_OVRIDE_2, 1);
	if (rc) {
		pr_err("failed to read BUCK_CMP_OVERIDE_2, rc = %d\n", rc);
		return 0;
	}

	return value & BUCK_VIN_LOOP_CMP_OVRD_MASK;
}

#define TEST_EN_SMBC_LOOP		0xE5
#define IBAT_REGULATION_DISABLE		BIT(2)

/* LGE not useed BTM feature  */
#ifndef CONFIG_LGE_PM
static irqreturn_t
qpnp_chg_bat_if_batt_temp_irq_handler(int irq, void *_chip)
{
	struct qpnp_chg_chip *chip = _chip;
	int batt_temp_good, batt_present, rc;

	batt_temp_good = qpnp_chg_is_batt_temp_ok(chip);
	pr_err("batt-temp triggered: %d\n", batt_temp_good);

	batt_present = qpnp_chg_is_batt_present(chip);
	if (batt_present) {
		rc = qpnp_chg_masked_write(chip,
			chip->buck_base + SEC_ACCESS,
			0xFF,
			0xA5, 1);
		if (rc) {
			pr_err("failed to write SEC_ACCESS rc=%d\n", rc);
			return rc;
		}

		rc = qpnp_chg_masked_write(chip,
			chip->buck_base + TEST_EN_SMBC_LOOP,
			IBAT_REGULATION_DISABLE,
			batt_temp_good ? 0 : IBAT_REGULATION_DISABLE, 1);
		if (rc) {
			pr_err("failed to write COMP_OVR1 rc=%d\n", rc);
			return rc;
		}
	}

	pr_debug("psy changed batt_psy\n");
	power_supply_changed(&chip->batt_psy);
	return IRQ_HANDLED;
}
#endif
static irqreturn_t
qpnp_chg_bat_if_batt_pres_irq_handler(int irq, void *_chip)
{
	struct qpnp_chg_chip *chip = _chip;
#ifndef CONFIG_LGE_PM
	int batt_present, batt_temp_good, rc;
#else
	int batt_present;
#endif

	batt_present = qpnp_chg_is_batt_present(chip);
	pr_err("batt-pres triggered: %d\n", batt_present);

	if (chip->batt_present ^ batt_present) {
		if (batt_present) {
#ifndef CONFIG_LGE_PM
			batt_temp_good = qpnp_chg_is_batt_temp_ok(chip);
			rc = qpnp_chg_masked_write(chip,
				chip->buck_base + SEC_ACCESS,
				0xFF,
				0xA5, 1);
			if (rc) {
				pr_err("failed to write SEC_ACCESS: %d\n", rc);
				return rc;
			}

			rc = qpnp_chg_masked_write(chip,
				chip->buck_base + TEST_EN_SMBC_LOOP,
				IBAT_REGULATION_DISABLE,
				batt_temp_good
				? 0 : IBAT_REGULATION_DISABLE, 1);
			if (rc) {
				pr_err("failed to write COMP_OVR1 rc=%d\n", rc);
				return rc;
			}
#endif
			schedule_work(&chip->insertion_ocv_work);
		} else {
#ifndef CONFIG_LGE_PM
			rc = qpnp_chg_masked_write(chip,
				chip->buck_base + SEC_ACCESS,
				0xFF,
				0xA5, 1);
			if (rc) {
				pr_err("failed to write SEC_ACCESS: %d\n", rc);
				return rc;
			}

			rc = qpnp_chg_masked_write(chip,
				chip->buck_base + TEST_EN_SMBC_LOOP,
				IBAT_REGULATION_DISABLE,
				0, 1);
			if (rc) {
				pr_err("failed to write COMP_OVR1 rc=%d\n", rc);
				return rc;
			}
#endif
			chip->insertion_ocv_uv = 0;
			qpnp_chg_charge_en(chip, 0);
		}
		chip->batt_present = batt_present;
		pr_debug("psy changed batt_psy\n");
		power_supply_changed(&chip->batt_psy);
		pr_debug("psy changed usb_psy\n");
		power_supply_changed(chip->usb_psy);

		if ((chip->cool_bat_decidegc || chip->warm_bat_decidegc)
						&& batt_present) {
			pr_debug("enabling vadc notifications\n");
			schedule_work(&chip->adc_measure_work);
		} else if ((chip->cool_bat_decidegc || chip->warm_bat_decidegc)
				&& !batt_present) {
			schedule_work(&chip->adc_disable_work);
			pr_debug("disabling vadc notifications\n");
		}
	}

	return IRQ_HANDLED;
}

static irqreturn_t
qpnp_chg_dc_dcin_valid_irq_handler(int irq, void *_chip)
{
	struct qpnp_chg_chip *chip = _chip;
	int dc_present;

#ifdef CONFIG_LGE_PM
	if ((chip->uevent_wake_lock.ws.name) != NULL)
		wake_lock_timeout(&chip->uevent_wake_lock, HZ*2);
#endif

	dc_present = qpnp_chg_is_dc_chg_plugged_in(chip);
	pr_err("dcin-valid triggered: %d\n", dc_present);

	if (chip->dc_present ^ dc_present) {
		chip->dc_present = dc_present;
		if (qpnp_chg_is_otg_en_set(chip))
			qpnp_chg_force_run_on_batt(chip, !dc_present ? 1 : 0);
		if (!dc_present && (!qpnp_chg_is_usb_chg_plugged_in(chip) ||
					qpnp_chg_is_otg_en_set(chip))) {
			chip->chg_done = false;
		} else {
			schedule_delayed_work(&chip->eoc_work,
				msecs_to_jiffies(EOC_CHECK_PERIOD_MS));
#ifndef CONFIG_LGE_PM_4_25V_CHARGING_START
			schedule_work(&chip->soc_check_work);
#endif
		}

		if (qpnp_is_dc_higher_prio(chip)) {
			pr_debug("dc has higher priority\n");
			if (dc_present) {
				qpnp_chg_iusbmax_set(chip,
						QPNP_CHG_I_MAX_MIN_100);
				power_supply_set_voltage_limit(chip->usb_psy,
						USB_5V_UV);
			} else {
				chip->aicl_settled = false;
				qpnp_chg_iusbmax_set(chip,
						USB_WALL_THRESHOLD_MA);
				power_supply_set_voltage_limit(chip->usb_psy,
						USB_9V_UV);
			}
		}

		pr_debug("psy changed dc_psy\n");
		power_supply_changed(&chip->dc_psy);
		pr_debug("psy changed batt_psy\n");
		power_supply_changed(&chip->batt_psy);
		schedule_work(&chip->batfet_lcl_work);
	}

	return IRQ_HANDLED;
}

#define CHGR_CHG_FAILED_BIT	BIT(7)
static irqreturn_t
qpnp_chg_chgr_chg_failed_irq_handler(int irq, void *_chip)
{
	struct qpnp_chg_chip *chip = _chip;
	int rc;

	pr_err("chg_failed triggered\n");
#ifdef CONFIG_LGE_PM_4_25V_CHARGING_START
	if (chip->from_temp_monitor_vbat_det_high == true) {
		qpnp_chg_vbatdet_set(chip, chip->max_voltage_mv - chip->resume_delta_mv);
		chip->from_temp_monitor_vbat_det_high = false;
	}
#endif

#if defined(CONFIG_MACH_MSM8926_B2LN_KR) || defined(CONFIG_MACH_MSM8926_JAGN_KR) || defined (CONFIG_MACH_MSM8926_AKA_CN) || defined(CONFIG_MACH_MSM8926_AKA_KR)
		pr_info("=========== Safety Timer Expired~! BUT DON'T [CHG STOP] ==============\n");
#else
#ifdef CONFIG_LGE_PM
	/* For the CHG Stop */
	if (!pseudo_batt_info.mode) {
		pr_info("=========== [CHG STOP] ==============\n");

		chip->chg_fail_irq_happen = true;
	    qpnp_chg_charge_en(chip, 0);
	}
#endif
#endif

	rc = qpnp_chg_masked_write(chip,
		chip->chgr_base + CHGR_CHG_FAILED,
		CHGR_CHG_FAILED_BIT,
		CHGR_CHG_FAILED_BIT, 1);
	if (rc)
		pr_err("Failed to write chg_fail clear bit!\n");

	if (chip->bat_if_base) {
		pr_debug("psy changed batt_psy\n");
		power_supply_changed(&chip->batt_psy);
	}
	pr_debug("psy changed usb_psy\n");
	power_supply_changed(chip->usb_psy);
	if (chip->dc_chgpth_base) {
		pr_debug("psy changed dc_psy\n");
		power_supply_changed(&chip->dc_psy);
	}
	return IRQ_HANDLED;
}

static irqreturn_t
qpnp_chg_chgr_chg_trklchg_irq_handler(int irq, void *_chip)
{
	struct qpnp_chg_chip *chip = _chip;

	pr_err("TRKL IRQ triggered\n");
#ifdef CONFIG_LGE_PM_4_25V_CHARGING_START
	if (chip->from_temp_monitor_vbat_det_high == true) {
		qpnp_chg_vbatdet_set(chip, chip->max_voltage_mv - chip->resume_delta_mv);
		chip->from_temp_monitor_vbat_det_high = false;
	}
#endif
	chip->chg_done = false;
	if (chip->bat_if_base) {
		pr_debug("psy changed batt_psy\n");
		power_supply_changed(&chip->batt_psy);
	}

	return IRQ_HANDLED;
}

static int qpnp_chg_is_fastchg_on(struct qpnp_chg_chip *chip)
{
	u8 chgr_sts;
	int rc;

	qpnp_chg_irq_wake_disable(&chip->chg_fastchg);

	rc = qpnp_chg_read(chip, &chgr_sts, INT_RT_STS(chip->chgr_base), 1);
	if (rc) {
		pr_err("failed to read interrupt status %d\n", rc);
		return rc;
	}
	pr_debug("chgr_sts 0x%x\n", chgr_sts);
	return (chgr_sts & FAST_CHG_ON_IRQ) ? 1 : 0;
}

#define VBATDET_BYPASS	0x01
static int
bypass_vbatdet_comp(struct qpnp_chg_chip *chip, bool bypass)
{
	int rc;

	pr_debug("bypass %d\n", bypass);
		rc = qpnp_chg_masked_write(chip,
			chip->chgr_base + SEC_ACCESS,
			0xA5,
			0xA5, 1);
	rc |= qpnp_chg_masked_write(chip,
			chip->chgr_base + CHGR_COMP_OVR1,
			0xFF,
			bypass ? VBATDET_BYPASS : 0, 1);
	if (rc) {
		pr_err("Failed to bypass vbatdet comp rc = %d\n", rc);
		return rc;
	}

	return rc;
}

static irqreturn_t
qpnp_chg_chgr_chg_fastchg_irq_handler(int irq, void *_chip)
{
	struct qpnp_chg_chip *chip = _chip;
	bool fastchg_on = false;

	fastchg_on = qpnp_chg_is_fastchg_on(chip);

	pr_info("FAST_CHG IRQ triggered, fastchg_on: %d\n", fastchg_on);

	if (chip->fastchg_on ^ fastchg_on) {
		chip->fastchg_on = fastchg_on;
#ifdef CONFIG_LGE_PM_4_25V_CHARGING_START
		if (fastchg_on) {
#endif
		if (chip->bat_if_base) {
			pr_debug("psy changed batt_psy\n");
			power_supply_changed(&chip->batt_psy);
		}

		pr_debug("psy changed usb_psy\n");
		power_supply_changed(chip->usb_psy);

		if (chip->dc_chgpth_base) {
			pr_debug("psy changed dc_psy\n");
			power_supply_changed(&chip->dc_psy);
		}
#ifdef CONFIG_LGE_PM_4_25V_CHARGING_START
		}
#endif

		if (fastchg_on) {
			chip->chg_done = false;
#ifndef CONFIG_LGE_PM_4_25V_CHARGING_START
			bypass_vbatdet_comp(chip, 1);
#endif
			if (chip->bat_is_warm || chip->bat_is_cool) {
				qpnp_chg_set_appropriate_vddmax(chip);
				qpnp_chg_set_appropriate_battery_current(chip);
			}

			if (chip->resuming_charging) {
				chip->resuming_charging = false;
				qpnp_chg_set_appropriate_vbatdet(chip);
			}

			if (!chip->charging_disabled) {
				schedule_delayed_work(&chip->eoc_work,
					msecs_to_jiffies(EOC_CHECK_PERIOD_MS));
				pm_stay_awake(chip->dev);
			}
			if (chip->parallel_ovp_mode)
				switch_parallel_ovp_mode(chip, 1);

			if (ext_ovp_isns_present &&
					chip->ext_ovp_ic_gpio_enabled) {
				pr_debug("EXT OVP IC ISNS enabled\n");
				gpio_direction_output(
						chip->ext_ovp_isns_gpio, 1);
			}
		} else {
			if (chip->parallel_ovp_mode)
				switch_parallel_ovp_mode(chip, 0);
			if (!chip->bat_is_warm && !chip->bat_is_cool)
				bypass_vbatdet_comp(chip, 0);
		}
	}

#ifdef CONFIG_LGE_PM_4_25V_CHARGING_START
	if (chip->from_temp_monitor_vbat_det_high == true) {
		qpnp_chg_vbatdet_set(chip, chip->max_voltage_mv - chip->resume_delta_mv);
		chip->from_temp_monitor_vbat_det_high = false;
	}
#endif
#if defined (CONFIG_MACH_MSM8926_B2LN_KR) || defined (CONFIG_MACH_MSM8926_JAGN_KR)
#else
	qpnp_chg_enable_irq(&chip->chg_vbatdet_lo);
#endif
#ifdef CONFIG_LGE_PM_CHARGING_EXTERNAL_OVP
	if (!is_factory_cable()) {
		if (fastchg_on) {
			pr_info("=== [FAST CHG IRQ] DCP = %d, SDP = %d ===\n",\
				chip->ac_online, qpnp_chg_is_usb_chg_plugged_in(chip));

			if (chip->ac_online && qpnp_chg_is_usb_chg_plugged_in(chip)) {	/* DCP Cable */
				pr_info("[FAST CHG IRQ] DCP CABLE - Ext. OVP is HIGH\n");
				lge_set_chg_path_to_external();
			} else if (!chip->ac_online && qpnp_chg_is_usb_chg_plugged_in(chip)) {	/* SDP Cable */
				pr_info("[FAST CHG IRQ] SDP CABLE - Ext. OVP is LOW\n");
				lge_set_chg_path_to_internal();
			}
		}
	}
#endif
	return IRQ_HANDLED;
}

#ifndef CONFIG_LGE_PM
static int
qpnp_dc_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		return 1;
	default:
		break;
	}

	return 0;
}
#endif

#ifdef CONFIG_LGE_PM
static int
qpnp_power_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_SAFTETY_CHARGER_TIMER:
		return 1;
	default:
		break;
	}

	return 0;

}
#endif

static int
qpnp_batt_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
#ifndef CONFIG_LGE_PM
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
#endif
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_TRIM:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED:
	case POWER_SUPPLY_PROP_VCHG_LOOP_DBC_BYPASS:
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
#ifndef CONFIG_LGE_PM
	case POWER_SUPPLY_PROP_COOL_TEMP:
	case POWER_SUPPLY_PROP_WARM_TEMP:
#endif
	case POWER_SUPPLY_PROP_CAPACITY:
		return 1;
	default:
		break;
	}

	return 0;
}

#ifndef CONFIG_LGE_PM
static int
qpnp_chg_buck_control(struct qpnp_chg_chip *chip, int enable)
{
	int rc;

	if (chip->charging_disabled && enable) {
		pr_debug("Charging disabled\n");
		return 0;
	}

	rc = qpnp_chg_charge_en(chip, enable);
	if (rc) {
		pr_err("Failed to control charging %d\n", rc);
		return rc;
	}

	rc = qpnp_chg_force_run_on_batt(chip, !enable);
	if (rc)
		pr_err("Failed to control charging %d\n", rc);

	return rc;
}
#endif

static int
switch_usb_to_charge_mode(struct qpnp_chg_chip *chip)
{
	int rc;

	pr_debug("switch to charge mode\n");
	if (!qpnp_chg_is_otg_en_set(chip))
		return 0;

	if (chip->type == SMBBP) {
		rc = qpnp_chg_masked_write(chip,
			chip->boost_base + BOOST_ILIM,
			BOOST_ILIMT_MASK,
			BOOST_ILIMIT_DEF, 1);
		if (rc) {
			pr_err("Failed to set ilim rc = %d\n", rc);
			return rc;
		}
	}

	/* enable usb ovp fet */
	rc = qpnp_chg_masked_write(chip,
			chip->usb_chgpth_base + CHGR_USB_USB_OTG_CTL,
			USB_OTG_EN_BIT,
			0, 1);
	if (rc) {
		pr_err("Failed to turn on usb ovp rc = %d\n", rc);
		return rc;
	}

	rc = qpnp_chg_force_run_on_batt(chip, chip->charging_disabled);
	if (rc) {
		pr_err("Failed re-enable charging rc = %d\n", rc);
		return rc;
	}

	return 0;
}

static int
switch_usb_to_host_mode(struct qpnp_chg_chip *chip)
{
	int rc;
	u8 usb_sts;

	pr_debug("switch to host mode\n");
	if (qpnp_chg_is_otg_en_set(chip))
		return 0;

	if (chip->parallel_ovp_mode)
		switch_parallel_ovp_mode(chip, 0);

	if (chip->type == SMBBP) {
		rc = qpnp_chg_masked_write(chip,
				chip->boost_base + BOOST_ILIM,
				BOOST_ILIMT_MASK,
				BOOST_ILIMIT_MIN, 1);
		if (rc) {
			pr_err("Failed to turn configure ilim rc = %d\n", rc);
			return rc;
		}
	}

	if (!qpnp_chg_is_dc_chg_plugged_in(chip)) {
		rc = qpnp_chg_force_run_on_batt(chip, 1);
		if (rc) {
			pr_err("Failed to disable charging rc = %d\n", rc);
			return rc;
		}
	}

	/* force usb ovp fet off */
	rc = qpnp_chg_masked_write(chip,
			chip->usb_chgpth_base + CHGR_USB_USB_OTG_CTL,
			USB_OTG_EN_BIT,
			USB_OTG_EN_BIT, 1);
	if (rc) {
		pr_err("Failed to turn off usb ovp rc = %d\n", rc);
		return rc;
	}

	if (chip->type == SMBBP) {
		/* Wait for OCP circuitry to be powered up */
		msleep(100);
		rc = qpnp_chg_read(chip, &usb_sts,
				INT_RT_STS(chip->usb_chgpth_base), 1);
		if (rc) {
			pr_err("failed to read interrupt sts %d\n", rc);
			return rc;
		}

		if (usb_sts & COARSE_DET_USB_IRQ) {
			rc = qpnp_chg_masked_write(chip,
				chip->boost_base + BOOST_ILIM,
				BOOST_ILIMT_MASK,
				BOOST_ILIMIT_DEF, 1);
			if (rc) {
				pr_err("Failed to set ilim rc = %d\n", rc);
				return rc;
			}
		} else {
			pr_warn_ratelimited("USB short to GND detected!\n");
		}
	}

	return 0;
}

static enum power_supply_property pm_power_props_mains[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
#ifdef CONFIG_LGE_PM
	POWER_SUPPLY_PROP_SAFTETY_CHARGER_TIMER,
#endif
};

static enum power_supply_property msm_batt_power_props[] = {
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_STATUS,
#ifdef CONFIG_LGE_PM
	POWER_SUPPLY_PROP_STATUS_ORIGINAL,
#endif
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_TRIM,
	POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED,
	POWER_SUPPLY_PROP_VCHG_LOOP_DBC_BYPASS,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_TEMP,
#ifndef CONFIG_LGE_PM
	POWER_SUPPLY_PROP_COOL_TEMP,
	POWER_SUPPLY_PROP_WARM_TEMP,
	POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL,
#endif
#ifdef CONFIG_LGE_PM_FACTORY_PSEUDO_BATTERY
	POWER_SUPPLY_PROP_PSEUDO_BATT,
#endif
#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
	POWER_SUPPLY_PROP_VALID_BATT,
#endif
#ifdef CONFIG_LGE_PM_VZW_FAST_CHG
	POWER_SUPPLY_PROP_VZW_CHG_STATE,
#endif
#ifdef CONFIG_LGE_PM_VZW_LLK
	POWER_SUPPLY_PROP_STORE_DEMO_ENABLED,
#endif
#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
	POWER_SUPPLY_PROP_HW_REV,
	POWER_SUPPLY_PROP_USB_ID,
#endif
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
};

static char *pm_power_supplied_to[] = {
	"battery",
};

static char *pm_batt_supplied_to[] = {
	"bms",
};


#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
int chg_status_set(int value)
{
	struct qpnp_chg_chip *chip = qpnp_chg;


	if (value == 0) {
		/* stop charging */
		pr_info("[kernel] stop charging start\n");
		qpnp_chg_charge_en(chip, 0);
		testmode_stop_eoc = 1;

	} else if (value == 1) {
		/* start charging */
		pr_info("[kernel] start charging start\n");
		qpnp_chg_charge_en(chip, 1);
	}

	power_supply_changed(&chip->batt_psy);

	return 0;
}

EXPORT_SYMBOL(chg_status_set);
#endif
static int charger_monitor;
module_param(charger_monitor, int, 0644);

static int ext_ovp_present;
module_param(ext_ovp_present, int, 0444);

#define OVP_USB_WALL_TRSH_MA   200
static int
qpnp_power_get_property_mains(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct qpnp_chg_chip *chip = container_of(psy, struct qpnp_chg_chip,
								dc_psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 0;
		if (chip->charging_disabled)
			return 0;
#ifdef CONFIG_LGE_PM
#ifdef CONFIG_LGE_PM_VZW_FAST_CHG
		if (chg_state == VZW_NOT_CHARGING) {
			val->intval = 1;
		} else
#endif
		val->intval = chip->ac_online;
#else	/* Qualcomm Original */
		val->intval = qpnp_chg_is_dc_chg_plugged_in(chip);
#endif
		break;
#ifdef CONFIG_LGE_PM
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = chip->current_max;
		break;

	case POWER_SUPPLY_PROP_SAFTETY_CHARGER_TIMER:
		val->intval = chip->chg_timer;
		break;
#else
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = chip->maxinput_dc_ma * 1000;
		break;
#endif
	default:
		return -EINVAL;
	}
	return 0;
}
#ifdef CONFIG_LGE_PM
static int
qpnp_power_set_property_mains(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	struct qpnp_chg_chip *chip = container_of(psy, struct qpnp_chg_chip,
								dc_psy);
	int rc;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		chip->ac_online = val->intval;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		chip->current_max = val->intval;
		break;
	case POWER_SUPPLY_PROP_SAFTETY_CHARGER_TIMER:
		if (chip->chg_timer != val->intval) {
			if (val->intval == 1) {
				rc = qpnp_chg_tchg_max_set(chip, chip->tchg_mins);
				if (rc) {
					pr_debug("failed setting tchg_mins rc=%d\n", rc);
					return rc;
				}
			} else {
				rc = qpnp_chg_tchg_disable(chip);
				if (rc) {
					pr_debug("failed disable chg timer rc=%d\n", rc);
					return rc;
				}
			}
			chip->chg_timer = val->intval;
		}
		break;
	default:
		return -EINVAL;
	}

    pr_debug("[LGE]  %s cur_max[%d], online[%d]\n",
				__func__, chip->current_max, chip->ac_online);
	power_supply_changed(&chip->dc_psy);
	return 0;
}
#endif

static void
qpnp_aicl_check_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct qpnp_chg_chip *chip = container_of(dwork,
				struct qpnp_chg_chip, aicl_check_work);
	union power_supply_propval ret = {0,};
#ifdef CONFIG_LGE_PM
	static int aicl_check_count = 0;
	static bool aicl_check_wake_lock = false;
#endif

#ifdef CONFIG_LGE_PM
	pr_info("DEBUG : Charger_monitor = %d\n", charger_monitor);
	if (aicl_check_wake_lock == true) {
		pm_relax(chip->dev);
		pr_info("[LGE_AICL] aicl_pm_relax , aicl_check_wake_lock = %d\n", aicl_check_wake_lock);
		aicl_check_wake_lock = false;
	}

	if (!charger_monitor) {
		if (aicl_check_count < 6) /* 60 seconds at 10 second intervals */
			goto aicl_check_retry;
	}
#endif

	if (!charger_monitor && qpnp_chg_is_usb_chg_plugged_in(chip)) {
		chip->usb_psy->get_property(chip->usb_psy,
			  POWER_SUPPLY_PROP_CURRENT_MAX, &ret);
		if ((ret.intval / 1000) > USB_WALL_THRESHOLD_MA) {
			pr_info("[LGE_AICL]no charger_monitor present set iusbmax %d\n",
					ret.intval / 1000);
			qpnp_chg_iusbmax_set(chip, ret.intval / 1000);
		}
	} else {
#ifdef CONFIG_LGE_PM
		aicl_check_count = 0;
		pr_info("[LGE_AICL] aicl_check_count set to ZERO = %d\n", aicl_check_count);
#endif
		pr_info("[LGE_AICL]charger_monitor is present\n");
	}
	chip->charger_monitor_checked = true;

#ifdef CONFIG_LGE_PM
	return;
#endif

#ifdef CONFIG_LGE_PM
aicl_check_retry:
	aicl_check_count++;
	pr_info("[LGE_AICL] aicl_check_count = %d\n", aicl_check_count);
	pm_stay_awake(chip->dev);
	aicl_check_wake_lock = true;
	schedule_delayed_work(&chip->aicl_check_work, msecs_to_jiffies(EOC_CHECK_PERIOD_MS));
#endif
}

static int
get_prop_battery_voltage_now(struct qpnp_chg_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

#if 0
	int batt_volt = 0;
	int use_fuelgauge = 0;

	union power_supply_propval ret = {0,};

	if (chip->bms_psy) {
		use_fuelgauge = chip->bms_psy->use_external_fuelgauge;
	} else {
		use_fuelgauge = false;
	}

	if (chip->maxim17048 == NULL)
		chip->maxim17048 = power_supply_get_by_name("max17048");

	if ((use_fuelgauge) && (chip->maxim17048 != NULL)) {
		chip->maxim17048->get_property(chip->maxim17048,
				POWER_SUPPLY_PROP_VOLTAGE_NOW, &ret);

		batt_volt = ret.intval;
	} else {
		if (chip->bms_psy) {
#endif
#ifdef CONFIG_LGE_PM
    if (chip->revision < 0 && chip->type == SMBB) {
#else
	if (chip->revision == 0 && chip->type == SMBB) {
#endif
		pr_err("vbat reading not supported for 1.0 rc=%d\n", rc);
		return 0;
	} else {
		rc = qpnp_vadc_read(chip->vadc_dev, VBAT_SNS, &results);
		if (rc) {
			pr_err("unable to read vbat rc in voltage_now = %d\n", rc);
			return 0;
		}
#if 0
		batt_volt = (int)results.physical;
    }
	}
	}
	return batt_volt;
}
#endif
		return results.physical;
	}
}

#define BATT_PRES_BIT BIT(7)
static int
get_prop_batt_present(struct qpnp_chg_chip *chip)
{
	u8 batt_present;
	int rc;

	rc = qpnp_chg_read(chip, &batt_present,
				chip->bat_if_base + CHGR_BAT_IF_PRES_STATUS, 1);
	if (rc) {
		pr_err("Couldn't read battery status read failed rc=%d\n", rc);
		return 0;
	};
	return (batt_present & BATT_PRES_BIT) ? 1 : 0;
}

int batt_present_touch = -1;

int get_batt_present_touch(void){
    u8 batt_present;
    int rc;

    if(dummy_chip != NULL){

	rc = qpnp_chg_read(dummy_chip, &batt_present,
				dummy_chip->bat_if_base + CHGR_BAT_IF_PRES_STATUS, 1);
	if (rc) {
		pr_err("Couldn't read battery status read failed rc=%d\n", rc);
		return -1;
	};

	return (batt_present & BATT_PRES_BIT) ? 1 : 0;
    }

    pr_err("SKIP Batt Present dummy chip is NULL");
    return -1;
}
EXPORT_SYMBOL(get_batt_present_touch);

#if defined (CONFIG_MACH_MSM8226_E7WIFI) || defined (CONFIG_MACH_MSM8226_E8WIFI) || \
    defined (CONFIG_MACH_MSM8926_E8LTE) || defined (CONFIG_MACH_MSM8226_E9WIFI) || \
    defined (CONFIG_MACH_MSM8226_E9WIFIN) || defined (CONFIG_MACH_MSM8926_T8LTE)
int
qpnp_get_batt_present(void)
{
	struct qpnp_chg_chip *chip = qpnp_chg;
	u8 batt_present;
	int rc;

	rc = qpnp_chg_read(chip, &batt_present,
				chip->bat_if_base + CHGR_BAT_IF_PRES_STATUS, 1);
	if (rc) {
		pr_err("Couldn't read battery status read failed rc=%d\n", rc);
		return 0;
	};
	return (batt_present & BATT_PRES_BIT) ? 1 : 0;
}
EXPORT_SYMBOL(qpnp_get_batt_present);
#endif

#define BATT_TEMP_HOT	BIT(6)
#define BATT_TEMP_OK	BIT(7)

#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
#define BATT_TEMP_OVERHEAT 550
#define BATT_TEMP_COLD (-100)
#endif
static int
get_prop_batt_health(struct qpnp_chg_chip *chip)
{
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
	if (chip->usb_present) {
		if (chip->btm_state == BTM_HEALTH_OVERHEAT)
			return POWER_SUPPLY_HEALTH_OVERHEAT;
		if (chip->btm_state == BTM_HEALTH_COLD)
			return POWER_SUPPLY_HEALTH_COLD;
		else
			return POWER_SUPPLY_HEALTH_GOOD;
	} else {
		if (chip->current_batt_temp > BATT_TEMP_OVERHEAT)
			return POWER_SUPPLY_HEALTH_OVERHEAT;
		if (chip->current_batt_temp < BATT_TEMP_COLD)
			return POWER_SUPPLY_HEALTH_COLD;
		else
			return POWER_SUPPLY_HEALTH_GOOD;
	}
#else
	u8 batt_health;
	int rc;

	rc = qpnp_chg_read(chip, &batt_health,
				chip->bat_if_base + CHGR_STATUS, 1);
	if (rc) {
		pr_err("Couldn't read battery health read failed rc=%d\n", rc);
		return POWER_SUPPLY_HEALTH_UNKNOWN;
	};

	if (BATT_TEMP_OK & batt_health)
		return POWER_SUPPLY_HEALTH_GOOD;
	if (BATT_TEMP_HOT & batt_health)
		return POWER_SUPPLY_HEALTH_OVERHEAT;
	else
		return POWER_SUPPLY_HEALTH_COLD;
#endif
}

static int
get_prop_charge_type(struct qpnp_chg_chip *chip)
{
	int rc;
	u8 chgr_sts;
#ifdef CONFIG_LGE_WIRELESS_CHARGER_RT9536
	union power_supply_propval ret = {0,};
	int wlc_online = 0;
	if (chip->wireless == NULL)
		chip->wireless = power_supply_get_by_name("wireless");
	if (chip->wireless != NULL) {
		/* if wireless charger has been registered, use the present property */
		chip->wireless->get_property(chip->wireless,
					POWER_SUPPLY_PROP_ONLINE, &ret);
		wlc_online = ret.intval;
	}
	if (!get_prop_batt_present(chip))
		return POWER_SUPPLY_CHARGE_TYPE_NONE;

	rc = qpnp_chg_read(chip, &chgr_sts,
				INT_RT_STS(chip->chgr_base), 1);
	if (rc) {
		pr_err("failed to read interrupt sts %d\n", rc);
		if (wlc_online)
			return POWER_SUPPLY_CHARGE_TYPE_WIRELESS;
		else
			return POWER_SUPPLY_CHARGE_TYPE_NONE;
	}

	if (chgr_sts & TRKL_CHG_ON_IRQ)
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	if (chgr_sts & FAST_CHG_ON_IRQ)
		return POWER_SUPPLY_CHARGE_TYPE_FAST;

	if (wlc_online)
		return POWER_SUPPLY_CHARGE_TYPE_WIRELESS;

	return POWER_SUPPLY_CHARGE_TYPE_NONE;
#else

	if (!get_prop_batt_present(chip))
		return POWER_SUPPLY_CHARGE_TYPE_NONE;

	rc = qpnp_chg_read(chip, &chgr_sts,
				INT_RT_STS(chip->chgr_base), 1);
	if (rc) {
		pr_err("failed to read interrupt sts %d\n", rc);
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	}

	if (chgr_sts & TRKL_CHG_ON_IRQ)
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	if (chgr_sts & FAST_CHG_ON_IRQ)
		return POWER_SUPPLY_CHARGE_TYPE_FAST;

	return POWER_SUPPLY_CHARGE_TYPE_NONE;
#endif
}

#define DEFAULT_CAPACITY	50
static int
get_batt_capacity(struct qpnp_chg_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (chip->fake_battery_soc >= 0)
		return chip->fake_battery_soc;
	if (chip->use_default_batt_values || !get_prop_batt_present(chip))
		return DEFAULT_CAPACITY;
	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CAPACITY, &ret);
		return ret.intval;
	}
	return DEFAULT_CAPACITY;
}

static int
get_prop_batt_status(struct qpnp_chg_chip *chip)
{
	int rc;
	u8 chgr_sts, bat_if_sts;
	int chg_type = get_prop_charge_type(chip);
#ifdef CONFIG_LGE_WIRELESS_CHARGER_RT9536
	union power_supply_propval ret = {0,};
	int wlc_status = POWER_SUPPLY_STATUS_UNKNOWN;
	/* int wlc_online = 0;
	printk("[WLC] %s: chg_type is %d\n",__func__,chg_type); */
#endif
#ifdef CONFIG_LGE_WIRELESS_CHARGER_RT9536
	if (chip->wireless == NULL)
		chip->wireless = power_supply_get_by_name("wireless");
	if (chip->wireless != NULL) {
		/* if wireless charger has been registered, use the present property */
		chip->wireless->get_property(chip->wireless,
					POWER_SUPPLY_PROP_STATUS, &ret);
		wlc_status = ret.intval;
	}
	if (wlc_status == POWER_SUPPLY_STATUS_FULL) {
		/* printk("[WLC] %s: WLC Battery Full!!!\n", __func__); */
		return POWER_SUPPLY_STATUS_FULL;
	}
#endif
	rc = qpnp_chg_read(chip, &chgr_sts, INT_RT_STS(chip->chgr_base), 1);
	if (rc) {
		pr_err("failed to read interrupt sts %d\n", rc);
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	}

	rc = qpnp_chg_read(chip, &bat_if_sts, INT_RT_STS(chip->bat_if_base), 1);
	if (rc) {
		pr_err("failed to read bat_if sts %d\n", rc);
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
	}
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
	if (qpnp_chg_is_usb_chg_plugged_in(chip)) {
		if (chip->pseudo_ui_chg)
			return POWER_SUPPLY_STATUS_CHARGING;
		else if (chip->not_chg == CHG_BATT_STPCHG_STATE)
			return POWER_SUPPLY_STATUS_NOT_CHARGING;
	}
#endif
#ifdef CONFIG_LGE_WIRELESS_CHARGER_RT9536
	if (chg_type == POWER_SUPPLY_CHARGE_TYPE_WIRELESS) {
		/* printk("[WLC] %s: WLC Battery Charging!!!\n", __func__); */
		return POWER_SUPPLY_STATUS_CHARGING;
	}
#endif
	if (chg_type == POWER_SUPPLY_CHARGE_TYPE_UNKNOWN ||
		chg_type == POWER_SUPPLY_CHARGE_TYPE_NONE) {
#ifdef CONFIG_LGE_PM_VZW_FAST_CHG
		if ((chip->usb_present) || (chg_state == VZW_NOT_CHARGING)
				|| (chg_state == VZW_USB_DRIVER_UNINSTALLED)) {
#else
		if (chip->usb_present) {

#endif
			return POWER_SUPPLY_STATUS_NOT_CHARGING;
		} else {
			return POWER_SUPPLY_STATUS_DISCHARGING;
		}

	}

	if ((chgr_sts & TRKL_CHG_ON_IRQ) && !(bat_if_sts & BAT_FET_ON_IRQ))
		return POWER_SUPPLY_STATUS_CHARGING;
	if (chgr_sts & FAST_CHG_ON_IRQ && bat_if_sts & BAT_FET_ON_IRQ)
		return POWER_SUPPLY_STATUS_CHARGING;

	/*
	 * Report full if state of charge is 100 or chg_done is true
	 * when a charger is connected and boost is disabled
	 */
	if ((qpnp_chg_is_usb_chg_plugged_in(chip) ||
		qpnp_chg_is_dc_chg_plugged_in(chip)) &&
		(chip->chg_done || get_batt_capacity(chip) == 100)
		&& qpnp_chg_is_boost_en_set(chip) == 0) {
		return POWER_SUPPLY_STATUS_FULL;
	}

	return POWER_SUPPLY_STATUS_DISCHARGING;
}
#ifdef CONFIG_LGE_PM
static int
get_prop_batt_status_lge(struct qpnp_chg_chip *chip)
{
	/* report full if state of charge is 100, even if phone is charging */
	if ((qpnp_chg_is_usb_chg_plugged_in(chip) ||
		qpnp_chg_is_dc_chg_plugged_in(chip))
			&& get_batt_capacity(chip) >= 100) {
		return POWER_SUPPLY_STATUS_FULL;
	} else {
		return get_prop_batt_status(chip);
	}
}
#endif

static int
get_prop_current_now(struct qpnp_chg_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
			  POWER_SUPPLY_PROP_CURRENT_NOW, &ret);
		return ret.intval;
	} else {
		pr_debug("No BMS supply registered return 0\n");
	}

	return 0;
}

#if defined(CONFIG_LGE_PM) && !defined(CONFIG_MAX17048_FUELGAUGE)
static int
get_prop_full_design(struct qpnp_chg_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
			  POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN, &ret);
		return ret.intval;
	} else {
		pr_debug("No BMS supply registered return 0\n");
	}

	return 0;
}

static int
get_prop_charge_full(struct qpnp_chg_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
			  POWER_SUPPLY_PROP_CHARGE_FULL, &ret);
		return ret.intval;
	} else {
		pr_debug("No BMS supply registered return 0\n");
	}

	return 0;
}

static int
get_prop_capacity(struct qpnp_chg_chip *chip)
{
	union power_supply_propval ret = {0,};
	int soc;
#ifndef CONFIG_LGE_PM_4_25V_CHARGING_START
	int battery_status, bms_status, charger_in;
#endif

	if (chip->fake_battery_soc >= 0)
		return chip->fake_battery_soc;

	if (chip->use_default_batt_values || !get_prop_batt_present(chip))
		return DEFAULT_CAPACITY;

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CAPACITY, &ret);
		soc = ret.intval;
#ifndef CONFIG_LGE_PM_4_25V_CHARGING_START
		battery_status = get_prop_batt_status(chip);
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_STATUS, &ret);
		bms_status = ret.intval;
		charger_in = qpnp_chg_is_usb_chg_plugged_in(chip) ||
			qpnp_chg_is_dc_chg_plugged_in(chip);

		if (battery_status != POWER_SUPPLY_STATUS_CHARGING
				&& bms_status != POWER_SUPPLY_STATUS_CHARGING
				&& charger_in
				&& !chip->bat_is_cool
				&& !chip->bat_is_warm
				&& !chip->resuming_charging
				&& !chip->charging_disabled
				&& chip->soc_resume_limit
				&& soc <= chip->soc_resume_limit) {
			pr_debug("resuming charging at %d%% soc\n", soc);
			chip->resuming_charging = true;
			qpnp_chg_irq_wake_enable(&chip->chg_fastchg);
			qpnp_chg_set_appropriate_vbatdet(chip);
			qpnp_chg_charge_en(chip, !chip->charging_disabled);
		}
#endif
		if (soc == 0) {
			if (!qpnp_chg_is_usb_chg_plugged_in(chip)
				&& !qpnp_chg_is_usb_chg_plugged_in(chip))
				pr_warn_ratelimited("Battery 0, CHG absent\n");
		}
		return soc;
	} else {
		pr_debug("No BMS supply registered return 50\n");
	}

	/* return default capacity to avoid userspace
	 * from shutting down unecessarily */
	return DEFAULT_CAPACITY;
}
#else
static int
get_prop_full_design(struct qpnp_chg_chip *chip)
{
#ifdef CONFIG_MAX17048_FUELGAUGE
	int result = 0;
	int use_fuelgauge = 0;

	union power_supply_propval ret = {0,};

	if (chip->bms_psy) {
		use_fuelgauge = chip->bms_psy->use_external_fuelgauge;
		/* pr_err("[LGE] use_external_fuelgauge is %d !!!\n", use_fuelgauge); */
	} else {
		use_fuelgauge = 0;
		pr_err("[LGE] There is no BMS driver!!!\n");
	}

	if (chip->maxim17048 == NULL)
		chip->maxim17048 = power_supply_get_by_name("max17048");
	if ((use_fuelgauge) && (chip->maxim17048 != NULL)) {
		/* if battery has been registered, use the present property */
		chip->maxim17048->get_property(chip->maxim17048,
					POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN, &ret);
		result = ret.intval;
	} else {
		if (chip->bms_psy) {
			chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN, &ret);
			return ret.intval;
		} else {
			pr_debug("No BMS supply registered return 0\n");
		}

		return 0;
	}
	return result;

	/* return max17048_get_fulldesign(); */
#else
	pr_err("CONFIG_MAX17048_FUELGAUGE is not defined.\n");
	return 2500;
#endif
}

static int
get_prop_charge_full(struct qpnp_chg_chip *chip)
{
#ifdef CONFIG_MAX17048_FUELGAUGE
	int result = 0;
	int use_fuelgauge = 0;

	union power_supply_propval ret = {0,};

	if (chip->bms_psy) {
		use_fuelgauge = chip->bms_psy->use_external_fuelgauge;
		/* pr_err("[LGE] use_external_fuelgauge is %d !!!\n",use_fuelgauge); */
	} else {
		use_fuelgauge = 0;
		pr_err("[LGE] There is no BMS driver!!!\n");
	}

	if (chip->maxim17048 == NULL)
			chip->maxim17048 = power_supply_get_by_name("max17048");
	if ((use_fuelgauge) && (chip->maxim17048 != NULL)) {
		/* if battery has been registered, use the present property */
		chip->maxim17048->get_property(chip->maxim17048,
					POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN, &ret);
		result = ret.intval;
	} else {
		if (chip->bms_psy) {
			chip->bms_psy->get_property(chip->bms_psy,
				  POWER_SUPPLY_PROP_CHARGE_FULL, &ret);
			return ret.intval;
		} else {
			pr_debug("No BMS supply registered return 0\n");
		}

		return 0;
	}
	return result;

	/* return max17048_get_fulldesign(); */
#else
	pr_err("CONFIG_MAX17048_FUELGAUGE is not defined.\n");
	return 2500;
#endif
}

#define DEFAULT_CAPACITY	50
static int
get_prop_capacity(struct qpnp_chg_chip *chip)
{
#ifdef CONFIG_MAX17048_FUELGAUGE
	int capacity = 0;
	int soc;
	int use_fuelgauge = 0;
#ifndef CONFIG_LGE_PM_4_25V_CHARGING_START
	int battery_status, bms_status, charger_in;
#endif

	union power_supply_propval ret = {0,};
	if (chip->bms_psy) {
		use_fuelgauge = chip->bms_psy->use_external_fuelgauge;
	} else {
		use_fuelgauge = 1;
		pr_err("[LGE] There is no BMS driver!!!\n");
	}


	if (chip->maxim17048 == NULL)
			chip->maxim17048 = power_supply_get_by_name("max17048");
	if ((use_fuelgauge) && (chip->maxim17048 != NULL)) {
		/* if battery has been registered, use the present property */
		chip->maxim17048->get_property(chip->maxim17048,
					POWER_SUPPLY_PROP_CAPACITY, &ret);
		capacity = ret.intval;
	} else {
		if (chip->fake_battery_soc >= 0)
			return chip->fake_battery_soc;

		if (chip->use_default_batt_values || !get_prop_batt_present(chip))
			return DEFAULT_CAPACITY;

		if (chip->bms_psy) {
			chip->bms_psy->get_property(chip->bms_psy,
					POWER_SUPPLY_PROP_CAPACITY, &ret);
			soc = ret.intval;
#ifndef CONFIG_LGE_PM_4_25V_CHARGING_START
			battery_status = get_prop_batt_status(chip);
			chip->bms_psy->get_property(chip->bms_psy,
					POWER_SUPPLY_PROP_STATUS, &ret);
			bms_status = ret.intval;
			charger_in = qpnp_chg_is_usb_chg_plugged_in(chip) ||
				qpnp_chg_is_dc_chg_plugged_in(chip);

			if (battery_status != POWER_SUPPLY_STATUS_CHARGING
					&& bms_status != POWER_SUPPLY_STATUS_CHARGING
					&& charger_in
					&& !chip->bat_is_cool
					&& !chip->bat_is_warm
					&& !chip->resuming_charging
					&& !chip->charging_disabled
					&& chip->soc_resume_limit
					&& soc <= chip->soc_resume_limit) {
				pr_debug("resuming charging at %d%% soc\n", soc);
				chip->resuming_charging = true;
				qpnp_chg_set_appropriate_vbatdet(chip);
				qpnp_chg_charge_en(chip, !chip->charging_disabled);
			}
#endif
			if (soc == 0) {
				if (!qpnp_chg_is_usb_chg_plugged_in(chip)
					&& !qpnp_chg_is_usb_chg_plugged_in(chip))
					pr_warn_ratelimited("Battery 0, CHG absent\n");
			}
			return soc;
		} else {
			pr_debug("No BMS supply registered return 50\n");
		}

		/* return default capacity to avoid userspace
		 * from shutting down unecessarily */
		return DEFAULT_CAPACITY;
	}
	/* printk("[LGE] battery capacity is %d...\n",capacity); */
	return capacity;
#else
	pr_err("NOT INIT max17048, Cannot get capacity.\n");
	return 51;
#endif
}

#endif
#define DEFAULT_TEMP		250
#define MAX_TOLERABLE_BATT_TEMP_DDC	680
static int
get_prop_batt_temp(struct qpnp_chg_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

#ifdef CONFIG_LGE_PM_FACTORY_PSEUDO_BATTERY
	if (pseudo_batt_info.mode) {
		pr_debug("battery fake mode : %d \n", pseudo_batt_info.mode);
		return pseudo_batt_info.temp * 10;
		}
#endif
	if (chip->use_default_batt_values || !get_prop_batt_present(chip))
		return DEFAULT_TEMP;

	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX1_BATT_THERM, &results);
	if (rc) {
		pr_debug("Unable to read batt temperature rc=%d\n", rc);
		return 0;
	}
	pr_debug("get_bat_temp %d, %lld\n",
		results.adc_code, results.physical);

	return (int)results.physical;
}

static int get_prop_cycle_count(struct qpnp_chg_chip *chip)
{
	union power_supply_propval ret = {0,};

	if (chip->bms_psy)
		chip->bms_psy->get_property(chip->bms_psy,
			  POWER_SUPPLY_PROP_CYCLE_COUNT, &ret);
	return ret.intval;
}

#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
static int
get_prop_batt_id_valid(void)
{
	return (int)is_lge_battery_valid();
}
#endif
#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
static int
get_prop_hw_rev(void)
{
	return lge_get_board_revno();
}

static int
get_prop_usb_id(void)
{
	return lge_pm_get_cable_type();
}
#endif
static int get_prop_vchg_loop(struct qpnp_chg_chip *chip)
{
	u8 buck_sts;
	int rc;

	rc = qpnp_chg_read(chip, &buck_sts, INT_RT_STS(chip->buck_base), 1);

	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n",
				INT_RT_STS(chip->buck_base), rc);
		return rc;
	}
	pr_debug("buck usb sts 0x%x\n", buck_sts);

	return (buck_sts & VCHG_LOOP_IRQ) ? 1 : 0;
}

static int get_prop_online(struct qpnp_chg_chip *chip)
{
	return qpnp_chg_is_batfet_closed(chip);
}

#ifdef CONFIG_LGE_PM_4_25V_CHARGING_START
static void
qpnp_chg_resume_check_work(struct work_struct *work)
{
	struct qpnp_chg_chip *chip = container_of(work,
				struct qpnp_chg_chip, resume_check_work);

    int battery_charging_status, charger_inserted;
	battery_charging_status = get_prop_batt_status(chip);

	charger_inserted = qpnp_chg_is_usb_chg_plugged_in(chip);
	pr_err("qpnp_chg_resume_check_work, chagerin=%d, batt_status=%d\n", charger_inserted, battery_charging_status);
	if ((battery_charging_status != POWER_SUPPLY_STATUS_CHARGING)
		&& charger_inserted) {
		/* pr_err("start charing in monitor temp, vbatt=%d\n", req.batt_volt); */
		qpnp_chg_vbatdet_set(chip, chip->max_voltage_mv + chip->resume_delta_mv);
#if defined(CONFIG_MACH_MSM8926_JAGNM_ATT) || defined(CONFIG_MACH_MSM8926_JAGC_SPR) || \
	defined(CONFIG_MACH_MSM8926_JAGNM_BELL) || defined(CONFIG_MACH_MSM8926_JAGC_SPR) || \
	defined(CONFIG_MACH_MSM8926_JAGNM_RGS)
		if (charger_inserted)
			qpnp_chg_charge_en(chip, !chip->charging_disabled);
		else
			qpnp_chg_charge_en(chip, 0);
#else
		qpnp_chg_charge_en(chip, !chip->charging_disabled);
#endif
		/* chip->from_usbin_valid_irq = false; */
		chip->from_temp_monitor_vbat_det_high = true;
	}
}
#endif


#ifdef CONFIG_LGE_PM_VZW_LLK
int32_t vzw_llk_enable_charging(bool enable)
{
	int ret = 0;
	struct qpnp_chg_chip *chip = qpnp_chg;

	pr_err("enable=%d.\n", enable);

	if (enable) {
		schedule_delayed_work(&chip->battemp_work, HZ*5);
		schedule_delayed_work(&chip->eoc_work,
				msecs_to_jiffies(EOC_CHECK_PERIOD_MS));
		cancel_delayed_work_sync(&chip->vzw_llk_stop_chg_work);
		ret = qpnp_chg_charge_en(chip, enable);
	} else {
		schedule_delayed_work(&chip->vzw_llk_stop_chg_work,
								msecs_to_jiffies(1000*4));
	}

	if (ret) {
		pr_err("Failed to set CHG_ENABLE_BIT rc=%d\n", ret);
		return ret;
	}

	if (!enable && !(chg_state == VZW_LLK_NOT_CHARGING)) {
		temp_state = chg_state;
		chg_state = VZW_LLK_NOT_CHARGING;
		pr_info("CHG_STATE : %d\n", chg_state);
	} else if ((chg_state == VZW_LLK_NOT_CHARGING) && enable) {
		chg_state = temp_state;
		pr_info("CHG_STATE : %d\n", chg_state);
	}

	power_supply_changed(&chip->batt_psy);

	return 0;
}

int store_demo_enabled_set(bool enabled)
{
	struct qpnp_chg_chip *chip = qpnp_chg;

	pr_info("store demo enabled = %d\n", enabled);
	store_demo_enabled = enabled;

	if (store_demo_enabled == 1) {
		if (get_prop_capacity(chip) >  34) {
			vzw_llk_enable_charging(0);
			printk(KERN_INFO "%s : VZW LLK Charging Stop!!\n", __func__);
		} else {
			vzw_llk_enable_charging(1);
			printk(KERN_INFO "%s : VZW LLK Charging Enable!!\n", __func__);
		}
	} else {
		vzw_llk_enable_charging(1);
	}

	power_supply_changed(&chip->batt_psy);

	return 0;
}
EXPORT_SYMBOL(store_demo_enabled_set);
#endif

#ifdef CONFIG_LGE_PM_VZW_FAST_CHG
extern int lge_usb_config_finish;
extern void send_drv_state_uevent(int usb_drv_state);

void set_vzw_usb_charging_state(int state)
{
	struct qpnp_chg_chip *chip = qpnp_chg;
	usb_chg_state = state;

	switch (state) {
	case IS_OPEN_TA:
		cancel_delayed_work_sync(&chip->battemp_work);
		cancel_delayed_work_sync(&chip->eoc_work);
		pm_relax(chip->dev);
		qpnp_chg_charge_en(chip, 0);
		chg_state = VZW_NOT_CHARGING;
		pr_info("%s : OPEN TA is connected!!\n", __func__);
		break;
	case IS_USB_DRIVER_UNINSTALLED:
		qpnp_chg_charge_en(chip, 0);
		send_drv_state_uevent(0);
		chg_state = VZW_USB_DRIVER_UNINSTALLED;
		pr_info("[USB_DRV] USB DRIVER UNINSTALLED !!\n");
		break;
	case IS_USB_DRIVER_INSTALLED:
	    send_drv_state_uevent(1);
		chg_state = VZW_NORMAL_CHARGING;
	    pr_info("[USB_DRV] USB DRIVER INSTALLED !!\n");
	    break;
	}

	power_supply_changed(&chip->batt_psy);
}
EXPORT_SYMBOL(set_vzw_usb_charging_state);

int get_vzw_usb_charging_state(void)
{
    return usb_chg_state;
}
EXPORT_SYMBOL(get_vzw_usb_charging_state);

static void vzw_fast_chg_change_usb_charging_state(struct qpnp_chg_chip *chip)
{
	struct usb_phy *otg_xceiv;
	struct msm_otg *motg;

	otg_xceiv = usb_get_transceiver();
	if (!otg_xceiv) {
		pr_err("Failed to get usb transceiver.\n");
		return;
	}

	motg = container_of(otg_xceiv, struct msm_otg, phy);
	if (!motg) {
		pr_err("Failed to get otg driver data.\n");
		return;
	}

	if (lge_usb_config_finish == 0) {
		qpnp_chg_charge_en(chip, 0);
		pr_info("USB cable is connected, but USB is not configured!\n");
	} else if (usb_chg_state == IS_USB_DRIVER_INSTALLED) {
		qpnp_chg_charge_en(chip, !chip->charging_disabled);
		pr_info("USB is configured and USB Driver is installed!\n");
		usb_chg_state = IS_USB_CHARGING_ENABLE;
	}
}

static void vzw_fast_chg_set_charging(struct qpnp_chg_chip *chip)
{
	if (vzw_chg_present == NOT_PRESENT) {
		if (chip->is_vzw_slow_charging) {
			vzw_chg_present = SLOW_PRESENT;
			pr_info("slow TA, decrease charging current.\n");
		} else {
			vzw_chg_present = UNKNOWN_PRESENT;
		}
	} else if (vzw_chg_present == SLOW_PRESENT) {
		chg_state = VZW_UNDER_CURRENT_CHARGING;
		pr_info("chg_state 1 = %d\n", chg_state);
	} else {
		chg_state = VZW_NORMAL_CHARGING;
		pr_info("chg_state 2 = %d\n", chg_state);
	}
	power_supply_changed(&chip->batt_psy);
}

static void vzw_fast_chg_input_current_check_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct qpnp_chg_chip *chip = container_of(dwork,
				struct qpnp_chg_chip, input_current_check_work);
	int iusb_max, threshold = 500;
	int ovp_thershold = 200;
	int trim, trim_threshold = 64;

#ifdef CONFIG_LGE_PM_CHARGING_EXTERNAL_OVP
	if (chip->ext_ovp_slow_chg_trim)
		trim_threshold = chip->ext_ovp_slow_chg_trim;
#endif

	iusb_max = qpnp_chg_usb_iusbmax_get(chip);
	trim = qpnp_chg_iusb_trim_get(chip);

	if (!ext_ovp_present) {
		if (iusb_max <= threshold && (lge_power_supply_get_type() == POWER_SUPPLY_TYPE_USB_DCP)){
			pr_info("Slow charger is detected , input_current_max = %d trim = %d\n", iusb_max, trim);
			chip->is_vzw_slow_charging = true;
			vzw_chg_present = SLOW_PRESENT;
		} else {
			pr_info("Normal charger is detected , input_current_max = %d trim = %d\n", iusb_max, trim);
			chip->is_vzw_slow_charging = false;
		}
	} else {
		if (iusb_max <= ovp_thershold && trim <= trim_threshold
				&& (lge_power_supply_get_type() == POWER_SUPPLY_TYPE_USB_DCP)){
			pr_info("Slow charger is detected , input_current_max = %d trim = %d", iusb_max, trim);
			chip->is_vzw_slow_charging = true;
			vzw_chg_present = SLOW_PRESENT;
		} else {
			pr_info("Normal charger is detected , input_current_max = %d trim = %d\n", iusb_max, trim);
			chip->is_vzw_slow_charging = false;
		}
	}

	if (lge_power_supply_get_type() == POWER_SUPPLY_TYPE_USB_DCP)
		vzw_fast_chg_set_charging(chip);
}

static void vzw_fast_chg_cancel_input_current_check_work(struct work_struct *work)
{
	struct qpnp_chg_chip *chip = container_of(work,
				struct qpnp_chg_chip, cancel_input_current_check_work);

	cancel_delayed_work_sync(&chip->input_current_check_work);
	chip->is_vzw_slow_charging = false;
}
#endif

#ifdef CONFIG_LGE_WIRELESS_CHARGER_RT9536
static void wireless_charging_enable_work(struct work_struct *work)
{
	struct qpnp_chg_chip *chip = container_of(work,
				struct qpnp_chg_chip, wlc_enable_work);

	union power_supply_propval ret = {0,};
	if (chip->wireless == NULL)
		chip->wireless = power_supply_get_by_name("wireless");
	if (chip->wireless != NULL) {
		/* if wireless charger has been registered, use the present property */
		ret.intval = 1;
		chip->wireless->set_property(chip->wireless,
					POWER_SUPPLY_PROP_WLC_ENABLE, &ret);
	}
}

static void wireless_charging_disable_work(struct work_struct *work)
{
	struct qpnp_chg_chip *chip = container_of(work,
				struct qpnp_chg_chip, wlc_disable_work);

	union power_supply_propval ret = {0,};
	if (chip->wireless == NULL)
		chip->wireless = power_supply_get_by_name("wireless");
	if (chip->wireless != NULL) {
		/* if wireless charger has been registered, use the present property */
		ret.intval = 0;
		chip->wireless->set_property(chip->wireless,
					POWER_SUPPLY_PROP_WLC_ENABLE, &ret);
	}
}
#endif

#ifdef CONFIG_LGE_PM_VZW_LLK
static void
vzw_llk_stop_chg(struct work_struct *work)
{
	struct qpnp_chg_chip *chip = container_of(work,
				struct qpnp_chg_chip, vzw_llk_stop_chg_work.work);
	int ret;
	cancel_delayed_work_sync(&chip->battemp_work);
	cancel_delayed_work_sync(&chip->eoc_work);
	schedule_work(&chip->cancel_input_current_check_work);
	pm_relax(qpnp_chg->dev);
	ret = qpnp_chg_charge_en(chip, 0);

	if (ret) {
		pr_err("Failed to set CHG_ENABLE_BIT rc=%d\n", ret);
		return;
	}
}
#endif

static void
qpnp_batt_external_power_changed(struct power_supply *psy)
{
	struct qpnp_chg_chip *chip = container_of(psy, struct qpnp_chg_chip,
								batt_psy);
	union power_supply_propval ret = {0,};

	if (!chip->bms_psy)
		chip->bms_psy = power_supply_get_by_name("bms");

	chip->usb_psy->get_property(chip->usb_psy,
			  POWER_SUPPLY_PROP_ONLINE, &ret);


/* BEGIN : janghyun.baek@lge.com 2013-01-25 Draw max current when factory cable inserted */
#ifdef CONFIG_LGE_PM
	if (is_factory_cable()) {
#if defined (CONFIG_MACH_MSM8226_E7WIFI) || defined (CONFIG_MACH_MSM8226_E8WIFI) || \
    defined (CONFIG_MACH_MSM8926_E8LTE) || defined (CONFIG_MACH_MSM8226_E9WIFI) || \
    defined (CONFIG_MACH_MSM8226_E9WIFIN) || defined (CONFIG_MACH_MSM8926_E9LTE) || \
    defined (CONFIG_MACH_MSM8926_T8LTE)
		if (get_prop_batt_present(chip) && is_56k_910k_factory_cable()) {
			 pr_info("Factory cable is connected (56K, 910K)\n");
			 qpnp_chg_iusbmax_set(chip, FACTORY_IUSB_MAX_FOR_EMBEDDED_BATTERY);
			 qpnp_chg_ibatmax_set(chip, FACTORY_IBAT_MAX_FOR_EMBEDDED_BATTERY);
		} else {
			 pr_info("Factory cable is connected (130K) IUSB MAX SETTING\n");
			 qpnp_chg_iusbmax_set(chip, QPNP_CHG_I_MAX_MAX_MA);
		}
#else
		qpnp_chg_iusbmax_set(chip, QPNP_CHG_I_MAX_MAX_MA);
#endif
		qpnp_chg_usb_suspend_enable(chip, 0);
		power_supply_changed(&chip->batt_psy);
		return;
	}
#endif
/* END : janghyun.baek@lge.com 2013-01-25 */
#ifdef CONFIG_LGE_PM_FACTORY_PSEUDO_BATTERY
	if ((pseudo_batt_info.mode == 1) && (!is_factory_cable())) {
		qpnp_chg_iusbmax_set(chip, PSEUDO_BATT_MAX);
		qpnp_chg_usb_suspend_enable(chip, 0);
		goto skip_set_iusb_max;
	}
#endif

    #ifdef CONFIG_LGE_PM
    pr_info("[LGE] usb_online: %d ac_online: %d\n", ret.intval, chip->ac_online);
	pr_info("[LGE] power_supply_type = %d\n", lge_power_supply_get_type());
    #endif

	/* Only honour requests while USB is present */
	if (qpnp_chg_is_usb_chg_plugged_in(chip)) {
#ifdef CONFIG_LGE_PM
		if (lge_power_supply_get_type() == POWER_SUPPLY_TYPE_USB_DCP) {	/* DCP */
			chip->dc_psy.get_property(&chip->dc_psy,
		      POWER_SUPPLY_PROP_CURRENT_MAX, &ret);
#ifdef CONFIG_LGE_PM_VZW_FAST_CHG
			vzw_fast_chg_set_charging(chip);
		} else if (ret.intval) {
#else
		} else {	/* others */
#endif
			chip->usb_psy->get_property(chip->usb_psy,
			POWER_SUPPLY_PROP_CURRENT_MAX, &ret);
#ifdef CONFIG_LGE_PM_VZW_FAST_CHG
			vzw_chg_present = USB_PRESENT;
			if (usb_chg_state != IS_USB_CHARGING_ENABLE) {
				qpnp_chg_charge_en(chip, 0);
				vzw_fast_chg_change_usb_charging_state(chip);
				if (usb_chg_state == IS_USB_CHARGING_ENABLE) {
					schedule_work(&chip->resume_check_work);
				}
			}
		} else {	/* others */
			chip->usb_psy->get_property(chip->usb_psy,
			POWER_SUPPLY_PROP_CURRENT_MAX, &ret);
#endif
		}
#else
		chip->usb_psy->get_property(chip->usb_psy,
			POWER_SUPPLY_PROP_CURRENT_MAX, &ret);
#endif
#ifdef CONFIG_LGE_PM_VZW_LLK
		if (store_demo_enabled == 1) {
			if ((get_prop_capacity(chip) > 34) || (llk_stop_chg_flag == true)) {
				vzw_llk_enable_charging(0);
				pr_info("VZW LLK Charging Stop!!, CHG_STATE : %d, llk_stop_chg_flag : %d\n", chg_state, llk_stop_chg_flag);
			}
		}
#endif

		if (chip->prev_usb_max_ma == ret.intval)
			goto skip_set_iusb_max;

		chip->prev_usb_max_ma = ret.intval;

		if (ret.intval <= 2 && !chip->use_default_batt_values &&
						get_prop_batt_present(chip)) {
#ifdef CONFIG_LGE_PM /* work-around code for prevent current setting zero when aconline & usbonline both zero on cable detect state */
			if (ret.intval != 0) {
				qpnp_chg_usb_suspend_enable(chip, 1);
				qpnp_chg_iusbmax_set(chip, QPNP_CHG_I_MAX_MIN_100);
				pr_err("[LGE] current zero set usb_online: %d ac_online: %d\n", ret.intval, chip->ac_online);
			}
#else
			if (ret.intval ==  2)
				qpnp_chg_usb_suspend_enable(chip, 1);
			qpnp_chg_iusbmax_set(chip, QPNP_CHG_I_MAX_MIN_100);
#endif
		} else {
			qpnp_chg_usb_suspend_enable(chip, 0);
			if (qpnp_is_dc_higher_prio(chip)
				&& qpnp_chg_is_dc_chg_plugged_in(chip)) {
					pr_debug("dc has higher priority\n");
					qpnp_chg_iusbmax_set(chip,
							QPNP_CHG_I_MAX_MIN_100);
			} else if (((ret.intval / 1000) > USB_WALL_THRESHOLD_MA)
					&& (charger_monitor ||
					!chip->charger_monitor_checked)) {
					if (!qpnp_is_dc_higher_prio(chip))
						qpnp_chg_idcmax_set(chip,
							QPNP_CHG_I_MAX_MIN_100);
					if (unlikely(ext_ovp_present)) {
						qpnp_chg_iusbmax_set(chip,
							OVP_USB_WALL_TRSH_MA);
					} else if (unlikely(
							ext_ovp_isns_present)) {
						qpnp_chg_iusb_trim_set(chip, 0);
						qpnp_chg_iusbmax_set(chip,
							IOVP_USB_WALL_TRSH_MA);
					} else {
						qpnp_chg_iusbmax_set(chip,
							USB_WALL_THRESHOLD_MA);
					}
			} else {
				qpnp_chg_iusbmax_set(chip, ret.intval / 1000);
			}

			if ((chip->flags & POWER_STAGE_WA)
			&& ((ret.intval / 1000) > USB_WALL_THRESHOLD_MA)
			&& !chip->power_stage_workaround_running
			&& chip->power_stage_workaround_enable) {
				chip->power_stage_workaround_running = true;
				pr_debug("usb wall chg inserted starting power stage workaround charger_monitor = %d\n",
						charger_monitor);
				schedule_work(&chip->reduce_power_stage_work);
			}
		}
	} else {
		qpnp_chg_iusbmax_set(chip, QPNP_CHG_I_MAX_MIN_100);
		qpnp_chg_usb_suspend_enable(chip, 1);
#ifdef CONFIG_LGE_PM_VZW_FAST_CHG
		lge_usb_config_finish = 0;
		usb_chg_state = IS_USB_DRIVER_UNINSTALLED;
		chg_state = VZW_NO_CHARGER;
		vzw_chg_present = NOT_PRESENT;
#ifdef CONFIG_LGE_PM_VZW_LLK
		temp_state = 0;
		llk_monitor_soc_flag = false;
		llk_stop_chg_flag = false;
#endif
#endif
	}

skip_set_iusb_max:
	pr_debug("end of power supply changed\n");
	pr_debug("psy changed batt_psy\n");
	power_supply_changed(&chip->batt_psy);
}

static int
qpnp_batt_power_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct qpnp_chg_chip *chip = container_of(psy, struct qpnp_chg_chip,
								batt_psy);

	switch (psp) {
#ifdef CONFIG_LGE_PM
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = get_prop_batt_status_lge(chip);	/*for Userspace*/
		break;
	case POWER_SUPPLY_PROP_STATUS_ORIGINAL:
		val->intval = get_prop_batt_status(chip);	/*for Kernel(BMS)*/
		break;
#else
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = get_prop_batt_status(chip);
		break;
#endif
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = get_prop_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = get_prop_batt_health(chip);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = get_prop_batt_present(chip);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = chip->max_voltage_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = chip->min_voltage_mv * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		/* val->intval = get_prop_battery_voltage_now(chip) * 1000; */
		val->intval = get_prop_battery_voltage_now(chip);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		val->intval = chip->insertion_ocv_uv;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = get_prop_batt_temp(chip);
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
		chip->current_batt_temp = val->intval;
#endif
		break;
#ifndef CONFIG_LGE_PM
	case POWER_SUPPLY_PROP_COOL_TEMP:
		val->intval = chip->cool_bat_decidegc;
		break;
	case POWER_SUPPLY_PROP_WARM_TEMP:
		val->intval = chip->warm_bat_decidegc;
		break;
#endif
	case POWER_SUPPLY_PROP_CAPACITY:
#ifdef CONFIG_LGE_PM_FACTORY_PSEUDO_BATTERY
		if (pseudo_batt_info.mode) {
			val->intval = pseudo_batt_info.capacity;
			break;
		}
#endif
		val->intval = get_prop_capacity(chip);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = get_prop_current_now(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = get_prop_full_design(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = get_prop_charge_full(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = !(chip->charging_disabled);
		break;
#ifndef CONFIG_LGE_PM
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		val->intval = chip->therm_lvl_sel;
		break;
#endif
#ifdef CONFIG_LGE_PM_FACTORY_PSEUDO_BATTERY
	case POWER_SUPPLY_PROP_PSEUDO_BATT:
		val->intval = pseudo_batt_info.mode;
		break;
#endif
#ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
	case POWER_SUPPLY_PROP_VALID_BATT:
		val->intval = get_prop_batt_id_valid();
		break;
#endif
#ifdef CONFIG_LGE_PM_VZW_FAST_CHG
	case POWER_SUPPLY_PROP_VZW_CHG_STATE:
		val->intval = chg_state;
		break;
#endif
#ifdef CONFIG_LGE_PM_VZW_LLK
	case POWER_SUPPLY_PROP_STORE_DEMO_ENABLED:
		val->intval = store_demo_enabled;
		break;
#endif
#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
	case POWER_SUPPLY_PROP_HW_REV:
		val->intval = get_prop_hw_rev();
		break;
	case POWER_SUPPLY_PROP_USB_ID:
		val->intval = get_prop_usb_id();
		break;
#endif
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		val->intval = get_prop_cycle_count(chip);
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_REGULATION:
		val->intval = get_prop_vchg_loop(chip);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		val->intval = qpnp_chg_usb_iusbmax_get(chip) * 1000;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_TRIM:
		val->intval = qpnp_chg_iusb_trim_get(chip);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED:
		val->intval = chip->aicl_settled;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		val->intval = qpnp_chg_vinmin_get(chip) * 1000;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = get_prop_online(chip);
		break;
	case POWER_SUPPLY_PROP_VCHG_LOOP_DBC_BYPASS:
		val->intval = qpnp_chg_vchg_loop_debouncer_setting_get(chip);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

#define BTC_CONFIG_ENABLED	BIT(7)
#define BTC_COLD		BIT(1)
#define BTC_HOT			BIT(0)
static int
qpnp_chg_bat_if_configure_btc(struct qpnp_chg_chip *chip)
{
	u8 btc_cfg = 0, mask = 0;

	/* Do nothing if battery peripheral not present */
	if (!chip->bat_if_base)
		return 0;

	if ((chip->hot_batt_p == HOT_THD_25_PCT)
			|| (chip->hot_batt_p == HOT_THD_35_PCT)) {
		btc_cfg |= btc_value[chip->hot_batt_p];
		mask |= BTC_HOT;
	}

	if ((chip->cold_batt_p == COLD_THD_70_PCT) ||
			(chip->cold_batt_p == COLD_THD_80_PCT)) {
		btc_cfg |= btc_value[chip->cold_batt_p];
		mask |= BTC_COLD;
	}

#ifdef CONFIG_LGE_PM_FACTORY_PSEUDO_BATTERY

	if (pseudo_batt_info.mode) {	/* pseudo == 1 , btc is disable. */
		mask |= BTC_CONFIG_ENABLED;
	} else if (chip->btc_disabled) {
		mask |= BTC_CONFIG_ENABLED;
	} else {						/* pseudo ==0, btc is enable. */
		mask |= BTC_CONFIG_ENABLED;
		btc_cfg |= BTC_CONFIG_ENABLED;
	}

#else

	if (chip->btc_disabled)
		mask |= BTC_CONFIG_ENABLED;

#endif

	return qpnp_chg_masked_write(chip,
			chip->bat_if_base + BAT_IF_BTC_CTRL,
			mask, btc_cfg, 1);
}

#ifdef CONFIG_LGE_PM_FACTORY_PSEUDO_BATTERY
int pseudo_batt_set(struct pseudo_batt_info_type *info)
{
	int rc = 0;
	struct qpnp_chg_chip *chip = qpnp_chg;
	pr_err("pseudo_batt_set\n");
	pseudo_batt_info.mode = info->mode;
	pseudo_batt_info.id = info->id;
	pseudo_batt_info.therm = info->therm;
	pseudo_batt_info.temp = info->temp;
	pseudo_batt_info.volt = info->volt;
	pseudo_batt_info.capacity = info->capacity;
	pseudo_batt_info.charging = info->charging;

	if ((pseudo_batt_info.mode == 1) && (!is_factory_cable())) {
		pr_info("Pseudo battery mode, set IUSB to PSEUDO_BATT_MAX.\n");
		qpnp_chg_iusbmax_set(chip, PSEUDO_BATT_MAX);
	}

	rc = qpnp_chg_bat_if_configure_btc(chip);
	if (rc) {
		pr_err("failed to configure btc %d\n", rc);
	}

	power_supply_changed(&chip->batt_psy);

	return 0;
}
EXPORT_SYMBOL(pseudo_batt_set);
#endif

#define QPNP_CHG_IBATSAFE_MIN_MA		100
#define QPNP_CHG_IBATSAFE_MAX_MA		3250
#define QPNP_CHG_I_STEP_MA		50
#define QPNP_CHG_I_MIN_MA		100
#define QPNP_CHG_I_MASK			0x3F
static int
qpnp_chg_ibatsafe_set(struct qpnp_chg_chip *chip, int safe_current)
{
	u8 temp;

	if (safe_current < QPNP_CHG_IBATSAFE_MIN_MA
			|| safe_current > QPNP_CHG_IBATSAFE_MAX_MA) {
		pr_err("bad mA=%d asked to set\n", safe_current);
		return -EINVAL;
	}

	temp = safe_current / QPNP_CHG_I_STEP_MA;
	return qpnp_chg_masked_write(chip,
			chip->chgr_base + CHGR_IBAT_SAFE,
			QPNP_CHG_I_MASK, temp, 1);
}

#define QPNP_CHG_ITERM_MIN_MA		100
#define QPNP_CHG_ITERM_MAX_MA		250
#define QPNP_CHG_ITERM_STEP_MA		50
#define QPNP_CHG_ITERM_MASK			0x03
static int
qpnp_chg_ibatterm_set(struct qpnp_chg_chip *chip, int term_current)
{
	u8 temp;

	if (term_current < QPNP_CHG_ITERM_MIN_MA
			|| term_current > QPNP_CHG_ITERM_MAX_MA) {
		pr_err("bad mA=%d asked to set\n", term_current);
		return -EINVAL;
	}

	temp = (term_current - QPNP_CHG_ITERM_MIN_MA)
				/ QPNP_CHG_ITERM_STEP_MA;
	return qpnp_chg_masked_write(chip,
			chip->chgr_base + CHGR_IBAT_TERM_CHGR,
			QPNP_CHG_ITERM_MASK, temp, 1);
}

#define QPNP_CHG_IBATMAX_MIN	50
#define QPNP_CHG_IBATMAX_MAX	3250
static int
qpnp_chg_ibatmax_set(struct qpnp_chg_chip *chip, int chg_current)
{
	u8 temp;

	if (chg_current < QPNP_CHG_IBATMAX_MIN
			|| chg_current > QPNP_CHG_IBATMAX_MAX) {
		pr_err("bad mA=%d asked to set\n", chg_current);
		return -EINVAL;
	}
	temp = chg_current / QPNP_CHG_I_STEP_MA;
	return qpnp_chg_masked_write(chip, chip->chgr_base + CHGR_IBAT_MAX,
			QPNP_CHG_I_MASK, temp, 1);
}

static int
qpnp_chg_ibatmax_get(struct qpnp_chg_chip *chip, int *chg_current)
{
	int rc;
	u8 temp;

	*chg_current = 0;
	rc = qpnp_chg_read(chip, &temp, chip->chgr_base + CHGR_IBAT_MAX, 1);
	if (rc) {
		pr_err("failed read ibat_max rc=%d\n", rc);
		return rc;
	}

	*chg_current = ((temp & QPNP_CHG_I_MASK) * QPNP_CHG_I_STEP_MA);

	return 0;
}

#define QPNP_CHG_TCHG_MASK	0x7F
#define QPNP_CHG_TCHG_EN_MASK	0x80
#define QPNP_CHG_TCHG_MIN	4
#define QPNP_CHG_TCHG_MAX	512
#define QPNP_CHG_TCHG_STEP	4
static int qpnp_chg_tchg_max_set(struct qpnp_chg_chip *chip, int minutes)
{
	u8 temp;
	int rc;

	if (minutes < QPNP_CHG_TCHG_MIN || minutes > QPNP_CHG_TCHG_MAX) {
		pr_err("bad max minutes =%d asked to set\n", minutes);
		return -EINVAL;
	}

	rc = qpnp_chg_masked_write(chip, chip->chgr_base + CHGR_TCHG_MAX_EN,
			QPNP_CHG_TCHG_EN_MASK, 0, 1);
	if (rc) {
		pr_err("failed write tchg_max_en rc=%d\n", rc);
		return rc;
	}

	temp = minutes / QPNP_CHG_TCHG_STEP - 1;

	rc = qpnp_chg_masked_write(chip, chip->chgr_base + CHGR_TCHG_MAX,
			QPNP_CHG_TCHG_MASK, temp, 1);
	if (rc) {
		pr_err("failed write tchg_max_en rc=%d\n", rc);
		return rc;
	}

	rc = qpnp_chg_masked_write(chip, chip->chgr_base + CHGR_TCHG_MAX_EN,
			QPNP_CHG_TCHG_EN_MASK, QPNP_CHG_TCHG_EN_MASK, 1);
	if (rc) {
		pr_err("failed write tchg_max_en rc=%d\n", rc);
		return rc;
	}

	return 0;
}
#ifdef CONFIG_LGE_PM
static int qpnp_chg_tchg_disable(struct qpnp_chg_chip *chip)
{
	int rc;


	rc = qpnp_chg_masked_write(chip, chip->chgr_base + CHGR_TCHG_MAX_EN,
			QPNP_CHG_TCHG_EN_MASK, 0, 1);
	if (rc) {
		pr_err("failed write tchg_max_en rc=%d\n", rc);
		return rc;
	}

	return 0;
}

#endif

static void
qpnp_chg_set_appropriate_battery_current(struct qpnp_chg_chip *chip)
{
	unsigned int chg_current = chip->max_bat_chg_current;

	if (chip->bat_is_cool)
		chg_current = min(chg_current, chip->cool_bat_chg_ma);

	if (chip->bat_is_warm)
		chg_current = min(chg_current, chip->warm_bat_chg_ma);

#ifndef CONFIG_LGE_PM

	if (chip->therm_lvl_sel != 0 && chip->thermal_mitigation)
		chg_current = min(chg_current,
			chip->thermal_mitigation[chip->therm_lvl_sel]);
#endif

	pr_debug("setting %d mA\n", chg_current);
	qpnp_chg_ibatmax_set(chip, chg_current);
}

static int
qpnp_chg_vddsafe_set(struct qpnp_chg_chip *chip, int voltage)
{
	u8 temp;

	if (voltage < QPNP_CHG_V_MIN_MV
			|| voltage > QPNP_CHG_V_MAX_MV) {
		pr_err("bad mV=%d asked to set\n", voltage);
		return -EINVAL;
	}
	temp = (voltage - QPNP_CHG_V_MIN_MV) / QPNP_CHG_V_STEP_MV;
	pr_debug("voltage=%d setting %02x\n", voltage, temp);
	return qpnp_chg_write(chip, &temp,
		chip->chgr_base + CHGR_VDD_SAFE, 1);
}

#define IBAT_TRIM_TGT_MA		500
#define IBAT_TRIM_OFFSET_MASK		0x7F
#define IBAT_TRIM_GOOD_BIT		BIT(7)
#define IBAT_TRIM_LOW_LIM		20
#define IBAT_TRIM_HIGH_LIM		114
#define IBAT_TRIM_MEAN			64

static void
qpnp_chg_trim_ibat(struct qpnp_chg_chip *chip, u8 ibat_trim)
{
	int ibat_now_ma, ibat_diff_ma, rc;
	struct qpnp_iadc_result i_result;
	enum qpnp_iadc_channels iadc_channel;

	iadc_channel = chip->use_external_rsense ?
				EXTERNAL_RSENSE : INTERNAL_RSENSE;
	rc = qpnp_iadc_read(chip->iadc_dev, iadc_channel, &i_result);
	if (rc) {
		pr_err("Unable to read bat rc=%d\n", rc);
		return;
	}

	ibat_now_ma = i_result.result_ua / 1000;

	if (qpnp_chg_is_ibat_loop_active(chip)) {
		ibat_diff_ma = ibat_now_ma - IBAT_TRIM_TGT_MA;

		if (abs(ibat_diff_ma) > 50) {
			ibat_trim += (ibat_diff_ma / 20);
			ibat_trim &= IBAT_TRIM_OFFSET_MASK;
			/* reject new ibat_trim if it is outside limits */
			if (!is_within_range(ibat_trim, IBAT_TRIM_LOW_LIM,
						IBAT_TRIM_HIGH_LIM))
				return;
		}
		ibat_trim |= IBAT_TRIM_GOOD_BIT;
		rc = qpnp_chg_write(chip, &ibat_trim,
				chip->buck_base + BUCK_CTRL_TRIM3, 1);
		if (rc)
			pr_err("failed to set IBAT_TRIM rc=%d\n", rc);

		pr_debug("ibat_now=%dmA, itgt=%dmA, ibat_diff=%dmA, ibat_trim=%x\n",
					ibat_now_ma, IBAT_TRIM_TGT_MA,
					ibat_diff_ma, ibat_trim);
	} else {
		pr_debug("ibat loop not active - cannot calibrate ibat\n");
	}
}

static int
qpnp_chg_input_current_settled(struct qpnp_chg_chip *chip)
{
	int rc, ibat_max_ma;
	u8 reg, chgr_sts, ibat_trim, i;
	bool usb_present = qpnp_chg_is_usb_chg_plugged_in(chip);

	if (!usb_present) {
		pr_debug("Ignoring AICL settled, since USB is removed\n");
		return 0;
	}
	chip->aicl_settled = true;

	/*
	 * Perform the ibat calibration.
	 * This is for devices which have a IBAT_TRIM error
	 * which can show IBAT_MAX out of spec.
	 */
	if (!chip->ibat_calibration_enabled)
		return 0;

	if (chip->type != SMBB)
		return 0;

	rc = qpnp_chg_read(chip, &reg,
			chip->buck_base + BUCK_CTRL_TRIM3, 1);
	if (rc) {
		pr_err("failed to read BUCK_CTRL_TRIM3 rc=%d\n", rc);
		return rc;
	}
	if (reg & IBAT_TRIM_GOOD_BIT) {
		pr_debug("IBAT_TRIM_GOOD bit already set. Quitting!\n");
		return 0;
	}
	ibat_trim = reg & IBAT_TRIM_OFFSET_MASK;

	if (!is_within_range(ibat_trim, IBAT_TRIM_LOW_LIM,
					IBAT_TRIM_HIGH_LIM)) {
		pr_debug("Improper ibat_trim value=%x setting to value=%x\n",
						ibat_trim, IBAT_TRIM_MEAN);
		ibat_trim = IBAT_TRIM_MEAN;
		rc = qpnp_chg_masked_write(chip,
				chip->buck_base + BUCK_CTRL_TRIM3,
				IBAT_TRIM_OFFSET_MASK, ibat_trim, 1);
		if (rc) {
			pr_err("failed to set ibat_trim to %x rc=%d\n",
						IBAT_TRIM_MEAN, rc);
			return rc;
		}
	}

	rc = qpnp_chg_read(chip, &chgr_sts,
				INT_RT_STS(chip->chgr_base), 1);
	if (rc) {
		pr_err("failed to read interrupt sts rc=%d\n", rc);
		return rc;
	}
	if (!(chgr_sts & FAST_CHG_ON_IRQ)) {
		pr_debug("Not in fastchg\n");
		return rc;
	}

	/* save the ibat_max to restore it later */
	rc = qpnp_chg_ibatmax_get(chip, &ibat_max_ma);
	if (rc) {
		pr_debug("failed to save ibatmax rc=%d\n", rc);
		return rc;
	}

	rc = qpnp_chg_ibatmax_set(chip, IBAT_TRIM_TGT_MA);
	if (rc) {
		pr_err("failed to set ibatmax rc=%d\n", rc);
		return rc;
	}

	for (i = 0; i < 3; i++) {
		/*
		 * ibat settling delay - to make sure the BMS controller
		 * has sufficient time to sample ibat for the configured
		 * ibat_max
		 */
		msleep(20);
		if (qpnp_chg_is_ibat_loop_active(chip))
			qpnp_chg_trim_ibat(chip, ibat_trim);
		else
			pr_debug("ibat loop not active\n");

		/* read the adjusted ibat_trim for further adjustments */
		rc = qpnp_chg_read(chip, &ibat_trim,
			chip->buck_base + BUCK_CTRL_TRIM3, 1);
		if (rc) {
			pr_err("failed to read BUCK_CTRL_TRIM3 rc=%d\n", rc);
			break;
		}
	}

	/* restore IBATMAX */
	rc = qpnp_chg_ibatmax_set(chip, ibat_max_ma);
	if (rc)
		pr_err("failed to restore ibatmax rc=%d\n", rc);

	return rc;
}


#define BOOST_MIN_UV	4200000
#define BOOST_MAX_UV	5500000
#define BOOST_STEP_UV	50000
#define BOOST_MIN	16
#define N_BOOST_V	((BOOST_MAX_UV - BOOST_MIN_UV) / BOOST_STEP_UV + 1)
static int
qpnp_boost_vset(struct qpnp_chg_chip *chip, int voltage)
{
	u8 reg = 0;

	if (voltage < BOOST_MIN_UV || voltage > BOOST_MAX_UV) {
		pr_err("invalid voltage requested %d uV\n", voltage);
		return -EINVAL;
	}

	reg = DIV_ROUND_UP(voltage - BOOST_MIN_UV, BOOST_STEP_UV) + BOOST_MIN;

	pr_debug("voltage=%d setting %02x\n", voltage, reg);
	return qpnp_chg_write(chip, &reg, chip->boost_base + BOOST_VSET, 1);
}

static int
qpnp_boost_vget_uv(struct qpnp_chg_chip *chip)
{
	int rc;
	u8 boost_reg;

	rc = qpnp_chg_read(chip, &boost_reg,
		 chip->boost_base + BOOST_VSET, 1);
	if (rc) {
		pr_err("failed to read BOOST_VSET rc=%d\n", rc);
		return rc;
	}

	if (boost_reg < BOOST_MIN) {
		pr_err("Invalid reading from 0x%x\n", boost_reg);
		return -EINVAL;
	}

	return BOOST_MIN_UV + ((boost_reg - BOOST_MIN) * BOOST_STEP_UV);
}

#ifndef CONFIG_LGE_PM
static void
qpnp_batt_system_temp_level_set(struct qpnp_chg_chip *chip, int lvl_sel)
{
	if (lvl_sel >= 0 && lvl_sel < chip->thermal_levels) {
		chip->therm_lvl_sel = lvl_sel;
		if (lvl_sel == (chip->thermal_levels - 1)) {
			/* disable charging if highest value selected */
			qpnp_chg_buck_control(chip, 0);
		} else {
			qpnp_chg_buck_control(chip, 1);
			qpnp_chg_set_appropriate_battery_current(chip);
		}
	} else {
		pr_err("Unsupported level selected %d\n", lvl_sel);
	}
}
#endif

/*
 * Increase the SMBB/SMBBP charger overtemp threshold to 150C while firing
 * the flash (and/or torch for PM8x26) when the bharger is used as the
 * power source.
 */
static int
qpnp_chg_temp_threshold_set(struct qpnp_chg_chip *chip, u8 value)
{
	int rc;

	rc = qpnp_chg_masked_write(chip, chip->chgr_base +
			CHGR_CHG_TEMP_THRESH ,
			0xFF, value, 1);
	if (rc)
		pr_err("set CHG_TEMP_THRESH_Flash failed, value = %d, rc = %d\n",
				value, rc);

	return rc;
}

#define CHG_TEMP_THRESH_FOR_FLASH		0xFD
#define CHG_TEMP_THRESH_DEFAULT			0x94
static int
qpnp_chg_regulator_flash_wa_enable(struct regulator_dev *rdev)
{
	struct qpnp_chg_chip *chip = rdev_get_drvdata(rdev);
	int rc = 0;

	if (chip->flags & BOOST_FLASH_WA) {
		rc = qpnp_chg_temp_threshold_set(chip,
				CHG_TEMP_THRESH_FOR_FLASH);
		if (rc) {
			pr_err("set chg temp threshold failed rc = %d\n", rc);
			return rc;
		}
	}
	chip->is_flash_wa_reg_enabled = true;

	return rc;
}

static int
qpnp_chg_regulator_flash_wa_disable(struct regulator_dev *rdev)
{
	struct qpnp_chg_chip *chip = rdev_get_drvdata(rdev);
	int rc = 0;

	if (chip->flags & BOOST_FLASH_WA) {
		rc = qpnp_chg_temp_threshold_set(chip,
				chip->chg_temp_thresh_default);
		if (rc) {
			pr_err("set chg temp threshold failed rc = %d\n", rc);
			return rc;
		}

	}
	chip->is_flash_wa_reg_enabled = false;

	return rc;
}

static int
qpnp_chg_regulator_flash_wa_is_enabled(struct regulator_dev *rdev)
{
	struct qpnp_chg_chip *chip = rdev_get_drvdata(rdev);

	return chip->is_flash_wa_reg_enabled;
}

/* OTG regulator operations */
static int
qpnp_chg_regulator_otg_enable(struct regulator_dev *rdev)
{
	struct qpnp_chg_chip *chip = rdev_get_drvdata(rdev);

	return switch_usb_to_host_mode(chip);
}

static int
qpnp_chg_regulator_otg_disable(struct regulator_dev *rdev)
{
	struct qpnp_chg_chip *chip = rdev_get_drvdata(rdev);

	return switch_usb_to_charge_mode(chip);
}

static int
qpnp_chg_regulator_otg_is_enabled(struct regulator_dev *rdev)
{
	struct qpnp_chg_chip *chip = rdev_get_drvdata(rdev);

	return qpnp_chg_is_otg_en_set(chip);
}

static int
qpnp_chg_regulator_boost_enable(struct regulator_dev *rdev)
{
	struct qpnp_chg_chip *chip = rdev_get_drvdata(rdev);
	int usb_present = qpnp_chg_is_usb_chg_plugged_in(chip);
	int rc;

	if (usb_present && (chip->flags & BOOST_FLASH_WA)) {

		if (ext_ovp_isns_present && chip->ext_ovp_ic_gpio_enabled) {
			pr_debug("EXT OVP IC ISNS disabled\n");
			gpio_direction_output(chip->ext_ovp_isns_gpio, 0);
		}

		qpnp_chg_usb_suspend_enable(chip, 1);

		rc = qpnp_chg_masked_write(chip,
			chip->usb_chgpth_base + SEC_ACCESS,
			0xFF,
			0xA5, 1);
		if (rc) {
			pr_err("failed to write SEC_ACCESS rc=%d\n", rc);
			return rc;
		}

		rc = qpnp_chg_masked_write(chip,
			chip->usb_chgpth_base + COMP_OVR1,
			0xFF,
			0x2F, 1);
		if (rc) {
			pr_err("failed to write COMP_OVR1 rc=%d\n", rc);
			return rc;
		}
	}

	rc = qpnp_chg_masked_write(chip,
		chip->boost_base + BOOST_ENABLE_CONTROL,
		BOOST_PWR_EN,
		BOOST_PWR_EN, 1);
	if (rc) {
		pr_err("failed to enable boost rc = %d\n", rc);
		return rc;
	}
	/*
	 * update battery status when charger is connected and state is full
	 */
	if (usb_present && (chip->chg_done
			|| (get_batt_capacity(chip) == 100)
			|| (get_prop_batt_status(chip) ==
			POWER_SUPPLY_STATUS_FULL)))
		power_supply_changed(&chip->batt_psy);

	return rc;
}

/* Boost regulator operations */
#define ABOVE_VBAT_WEAK		BIT(1)
static int
qpnp_chg_regulator_boost_disable(struct regulator_dev *rdev)
{
	struct qpnp_chg_chip *chip = rdev_get_drvdata(rdev);
	int rc;
	u8 vbat_sts;

	rc = qpnp_chg_masked_write(chip,
		chip->boost_base + BOOST_ENABLE_CONTROL,
		BOOST_PWR_EN,
		0, 1);
	if (rc) {
		pr_err("failed to disable boost rc=%d\n", rc);
		return rc;
	}

	rc = qpnp_chg_read(chip, &vbat_sts,
			chip->chgr_base + CHGR_VBAT_STATUS, 1);
	if (rc) {
		pr_err("failed to read bat sts rc=%d\n", rc);
		return rc;
	}

	if (!(vbat_sts & ABOVE_VBAT_WEAK) && (chip->flags & BOOST_FLASH_WA)) {
		rc = qpnp_chg_masked_write(chip,
			chip->chgr_base + SEC_ACCESS,
			0xFF,
			0xA5, 1);
		if (rc) {
			pr_err("failed to write SEC_ACCESS rc=%d\n", rc);
			return rc;
		}

		rc = qpnp_chg_masked_write(chip,
			chip->chgr_base + COMP_OVR1,
			0xFF,
			0x20, 1);
		if (rc) {
			pr_err("failed to write COMP_OVR1 rc=%d\n", rc);
			return rc;
		}

		usleep(2000);

		rc = qpnp_chg_masked_write(chip,
			chip->chgr_base + SEC_ACCESS,
			0xFF,
			0xA5, 1);
		if (rc) {
			pr_err("failed to write SEC_ACCESS rc=%d\n", rc);
			return rc;
		}

		rc = qpnp_chg_masked_write(chip,
			chip->chgr_base + COMP_OVR1,
			0xFF,
			0x00, 1);
		if (rc) {
			pr_err("failed to write COMP_OVR1 rc=%d\n", rc);
			return rc;
		}
	}

	if (qpnp_chg_is_usb_chg_plugged_in(chip)
			&& (chip->flags & BOOST_FLASH_WA)) {
		rc = qpnp_chg_masked_write(chip,
			chip->usb_chgpth_base + SEC_ACCESS,
			0xFF,
			0xA5, 1);
		if (rc) {
			pr_err("failed to write SEC_ACCESS rc=%d\n", rc);
			return rc;
		}

		rc = qpnp_chg_masked_write(chip,
			chip->usb_chgpth_base + COMP_OVR1,
			0xFF,
			0x00, 1);
		if (rc) {
			pr_err("failed to write COMP_OVR1 rc=%d\n", rc);
			return rc;
		}

		usleep(1000);

		qpnp_chg_usb_suspend_enable(chip, 0);
	}

	/*
	 * When a charger is connected,if state of charge is not full
	 * resumeing charging else update battery status
	 */
	if (qpnp_chg_is_usb_chg_plugged_in(chip)) {
		if (get_batt_capacity(chip) < 100 || !chip->chg_done) {
			chip->chg_done = false;
			chip->resuming_charging = true;
			qpnp_chg_set_appropriate_vbatdet(chip);
		} else if (chip->chg_done) {
			power_supply_changed(&chip->batt_psy);
		}
	}

	if (ext_ovp_isns_present && chip->ext_ovp_ic_gpio_enabled) {
		pr_debug("EXT OVP IC ISNS enable\n");
		gpio_direction_output(chip->ext_ovp_isns_gpio, 1);
	}

	return rc;
}

static int
qpnp_chg_regulator_boost_is_enabled(struct regulator_dev *rdev)
{
	struct qpnp_chg_chip *chip = rdev_get_drvdata(rdev);

	return qpnp_chg_is_boost_en_set(chip);
}

static int
qpnp_chg_regulator_boost_set_voltage(struct regulator_dev *rdev,
		int min_uV, int max_uV, unsigned *selector)
{
	int uV = min_uV;
	int rc;
	struct qpnp_chg_chip *chip = rdev_get_drvdata(rdev);

	if (uV < BOOST_MIN_UV && max_uV >= BOOST_MIN_UV)
		uV = BOOST_MIN_UV;


	if (uV < BOOST_MIN_UV || uV > BOOST_MAX_UV) {
		pr_err("request %d uV is out of bounds\n", uV);
		return -EINVAL;
	}

	*selector = DIV_ROUND_UP(uV - BOOST_MIN_UV, BOOST_STEP_UV);
	if ((*selector * BOOST_STEP_UV + BOOST_MIN_UV) > max_uV) {
		pr_err("no available setpoint [%d, %d] uV\n", min_uV, max_uV);
		return -EINVAL;
	}

	rc = qpnp_boost_vset(chip, uV);

	return rc;
}

static int
qpnp_chg_regulator_boost_get_voltage(struct regulator_dev *rdev)
{
	struct qpnp_chg_chip *chip = rdev_get_drvdata(rdev);

	return qpnp_boost_vget_uv(chip);
}

static int
qpnp_chg_regulator_boost_list_voltage(struct regulator_dev *rdev,
			unsigned selector)
{
	if (selector >= N_BOOST_V)
		return 0;

	return BOOST_MIN_UV + (selector * BOOST_STEP_UV);
}

static struct regulator_ops qpnp_chg_flash_wa_reg_ops = {
	.enable			= qpnp_chg_regulator_flash_wa_enable,
	.disable		= qpnp_chg_regulator_flash_wa_disable,
	.is_enabled		= qpnp_chg_regulator_flash_wa_is_enabled,
};

static struct regulator_ops qpnp_chg_otg_reg_ops = {
	.enable			= qpnp_chg_regulator_otg_enable,
	.disable		= qpnp_chg_regulator_otg_disable,
	.is_enabled		= qpnp_chg_regulator_otg_is_enabled,
};

static struct regulator_ops qpnp_chg_boost_reg_ops = {
	.enable			= qpnp_chg_regulator_boost_enable,
	.disable		= qpnp_chg_regulator_boost_disable,
	.is_enabled		= qpnp_chg_regulator_boost_is_enabled,
	.set_voltage		= qpnp_chg_regulator_boost_set_voltage,
	.get_voltage		= qpnp_chg_regulator_boost_get_voltage,
	.list_voltage		= qpnp_chg_regulator_boost_list_voltage,
};

static int
qpnp_chg_bat_if_batfet_reg_enabled(struct qpnp_chg_chip *chip)
{
	int rc = 0;
	u8 reg = 0;

	if (!chip->bat_if_base)
		return rc;

	if (chip->type == SMBB)
		rc = qpnp_chg_read(chip, &reg,
				chip->bat_if_base + CHGR_BAT_IF_SPARE, 1);
	else
		rc = qpnp_chg_read(chip, &reg,
			chip->bat_if_base + CHGR_BAT_IF_BATFET_CTRL4, 1);

	if (rc) {
		pr_err("failed to read batt_if rc=%d\n", rc);
		return rc;
	}

	if ((reg & BATFET_LPM_MASK) == BATFET_NO_LPM)
		return 1;

	return 0;
}

static int
qpnp_chg_regulator_batfet_enable(struct regulator_dev *rdev)
{
	struct qpnp_chg_chip *chip = rdev_get_drvdata(rdev);
	int rc = 0;

	mutex_lock(&chip->batfet_vreg_lock);
	/* Only enable if not already enabled */
	if (!qpnp_chg_bat_if_batfet_reg_enabled(chip)) {
		rc = qpnp_chg_regulator_batfet_set(chip, 1);
		if (rc)
			pr_err("failed to write to batt_if rc=%d\n", rc);
	}

	chip->batfet_ext_en = true;
	mutex_unlock(&chip->batfet_vreg_lock);

	return rc;
}

static int
qpnp_chg_regulator_batfet_disable(struct regulator_dev *rdev)
{
	struct qpnp_chg_chip *chip = rdev_get_drvdata(rdev);
	int rc = 0;

	mutex_lock(&chip->batfet_vreg_lock);
	/* Don't allow disable if charger connected */
	if (!qpnp_chg_is_usb_chg_plugged_in(chip) &&
			!qpnp_chg_is_dc_chg_plugged_in(chip)) {
		rc = qpnp_chg_regulator_batfet_set(chip, 0);
		if (rc)
			pr_err("failed to write to batt_if rc=%d\n", rc);
	}

	chip->batfet_ext_en = false;
	mutex_unlock(&chip->batfet_vreg_lock);

	return rc;
}

static int
qpnp_chg_regulator_batfet_is_enabled(struct regulator_dev *rdev)
{
	struct qpnp_chg_chip *chip = rdev_get_drvdata(rdev);

	return chip->batfet_ext_en;
}

static struct regulator_ops qpnp_chg_batfet_vreg_ops = {
	.enable			= qpnp_chg_regulator_batfet_enable,
	.disable		= qpnp_chg_regulator_batfet_disable,
	.is_enabled		= qpnp_chg_regulator_batfet_is_enabled,
};

#define MIN_DELTA_MV_TO_INCREASE_VDD_MAX	8
#define MAX_DELTA_VDD_MAX_MV			80
#define VDD_MAX_CENTER_OFFSET			4
static void
qpnp_chg_adjust_vddmax(struct qpnp_chg_chip *chip, int vbat_mv)
{
	int delta_mv, closest_delta_mv, sign;

	delta_mv = chip->max_voltage_mv - VDD_MAX_CENTER_OFFSET - vbat_mv;
	if (delta_mv > 0 && delta_mv < MIN_DELTA_MV_TO_INCREASE_VDD_MAX) {
		pr_debug("vbat is not low enough to increase vdd\n");
		return;
	}

	sign = delta_mv > 0 ? 1 : -1;
	closest_delta_mv = ((delta_mv + sign * QPNP_CHG_BUCK_TRIM1_STEP / 2)
			/ QPNP_CHG_BUCK_TRIM1_STEP) * QPNP_CHG_BUCK_TRIM1_STEP;
	pr_debug("max_voltage = %d, vbat_mv = %d, delta_mv = %d, closest = %d\n",
			chip->max_voltage_mv, vbat_mv,
			delta_mv, closest_delta_mv);
	chip->delta_vddmax_mv = clamp(chip->delta_vddmax_mv + closest_delta_mv,
			-MAX_DELTA_VDD_MAX_MV, MAX_DELTA_VDD_MAX_MV);
	pr_debug("using delta_vddmax_mv = %d\n", chip->delta_vddmax_mv);
	qpnp_chg_set_appropriate_vddmax(chip);
}

#ifdef CONFIG_LGE_PM_PWR_KEY_FOR_CHG_LOGO
bool is_eoc_work_stop = false;
extern bool is_enter_first;
#endif


#define CONSECUTIVE_COUNT	3
#define VBATDET_MAX_ERR_MV	50
#if defined (CONFIG_MACH_MSM8926_JAGC_SPR) || defined (COFIG_MACH_MSM8926_JAGNM_ATT) || defined(CONFIG_MACH_MSM8926_JAGNM_BELL) || defined(CONFIG_MACH_MSM8926_JAGC_SPR)
#define VBATDET_MAX_MARGIN 30
#endif

static void
qpnp_eoc_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct qpnp_chg_chip *chip = container_of(dwork,
				struct qpnp_chg_chip, eoc_work);
	static int count;
	static int vbat_low_count;
	int ibat_ma, vbat_mv, rc = 0;
	u8 batt_sts = 0, buck_sts = 0, chg_sts = 0;
	bool vbat_lower_than_vbatdet;
#ifdef CONFIG_LGE_PM_FIX_CEC_FAIL
	union power_supply_propval prop_val = {0,};
#endif

#ifdef CONFIG_LGE_PM_PWR_KEY_FOR_CHG_LOGO
    is_eoc_work_stop = false;
#endif


#ifdef CONFIG_LGE_PM_FACTORY_TESTMODE
	if (testmode_stop_eoc == 1) {
		testmode_stop_eoc = 0;
		goto stop_eoc;
	}
#endif

#ifdef CONFIG_LGE_PM
    if (chip->chg_fail_irq_happen)
		goto stop_eoc;
#endif

	pm_stay_awake(chip->dev);
#if defined(CONFIG_MACH_MSM8926_JAGNM_ATT) || defined(CONFIG_MACH_MSM8926_JAGC_SPR) || \
	defined(CONFIG_MACH_MSM8926_JAGNM_BELL) || defined(CONFIG_MACH_MSM8926_JAGC_SPR) || \
	defined(CONFIG_MACH_MSM8926_JAGNM_RGS)
	if (qpnp_chg_is_usb_chg_plugged_in(chip) || qpnp_chg_is_dc_chg_plugged_in(chip))
		qpnp_chg_charge_en(chip, !chip->charging_disabled);
	else
		qpnp_chg_charge_en(chip, 0);
#else
	qpnp_chg_charge_en(chip, !chip->charging_disabled);
#endif

	rc = qpnp_chg_read(chip, &batt_sts, INT_RT_STS(chip->bat_if_base), 1);
	if (rc) {
		pr_err("failed to read batt_if rc=%d\n", rc);
		return;
	}

	rc = qpnp_chg_read(chip, &buck_sts, INT_RT_STS(chip->buck_base), 1);
	if (rc) {
		pr_err("failed to read buck rc=%d\n", rc);
		return;
	}

	rc = qpnp_chg_read(chip, &chg_sts, INT_RT_STS(chip->chgr_base), 1);
	if (rc) {
		pr_err("failed to read chg_sts rc=%d\n", rc);
		return;
	}

	pr_debug("chgr: 0x%x, bat_if: 0x%x, buck: 0x%x\n",
		chg_sts, batt_sts, buck_sts);

	if (!qpnp_chg_is_usb_chg_plugged_in(chip) &&
			!qpnp_chg_is_dc_chg_plugged_in(chip)) {
		pr_debug("no chg connected, stopping\n");
		goto stop_eoc;
	}

	if ((batt_sts & BAT_FET_ON_IRQ) && (chg_sts & FAST_CHG_ON_IRQ
					|| chg_sts & TRKL_CHG_ON_IRQ)) {
		ibat_ma = get_prop_current_now(chip) / 1000;
		vbat_mv = get_prop_battery_voltage_now(chip) / 1000;

		pr_debug("ibat_ma = %d vbat_mv = %d term_current_ma = %d\n",
				ibat_ma, vbat_mv, chip->term_current);

		vbat_lower_than_vbatdet = !(chg_sts & VBAT_DET_LOW_IRQ);
#if defined (CONFIG_MACH_MSM8226_E7WIFI) || defined (CONFIG_MACH_MSM8226_E9WIFI) || \
    defined (CONFIG_MACH_MSM8226_E9WIFIN) || defined (CONFIG_MACH_MSM8926_E9LTE)
		if (vbat_lower_than_vbatdet && vbat_mv < 4330) {
#elif defined (CONFIG_MACH_MSM8226_E8WIFI) || defined(CONFIG_MACH_MSM8926_E8LTE) || \
      defined (CONFIG_MACH_MSM8926_T8LTE)
		if (vbat_lower_than_vbatdet && vbat_mv < 4120) {
#else
		if (vbat_lower_than_vbatdet && vbat_mv <
				(chip->max_voltage_mv - chip->resume_delta_mv
				 - chip->vbatdet_max_err_mv)) {
#endif
			vbat_low_count++;
			pr_debug("woke up too early vbat_mv = %d, max_mv = %d, resume_mv = %d tolerance_mv = %d low_count = %d\n",
					vbat_mv, chip->max_voltage_mv,
					chip->resume_delta_mv,
					chip->vbatdet_max_err_mv,
					vbat_low_count);
			if (vbat_low_count >= CONSECUTIVE_COUNT) {
				pr_debug("woke up too early stopping\n");
#if defined (CONFIG_MACH_MSM8926_B2LN_KR) || defined (CONFIG_MACH_MSM8926_JAGN_KR)
#else
				qpnp_chg_enable_irq(&chip->chg_vbatdet_lo);
#endif
#ifdef CONFIG_LGE_PM
/*
 * This code is given by Qualcomm.
 * During CC charging , the device does not go to power collapse by this code.
 */
				vbat_low_count = 0;
				count = 0;
				goto check_again_later;
#else
				goto stop_eoc;
#endif
			} else {
				goto check_again_later;
			}
		} else {
			vbat_low_count = 0;
		}

		if (buck_sts & VDD_LOOP_IRQ)
			qpnp_chg_adjust_vddmax(chip, vbat_mv);

		if (!(buck_sts & VDD_LOOP_IRQ)) {
			pr_debug("Not in CV\n");
			count = 0;
#if defined (CONFIG_MACH_MSM8226_E9WIFI) || defined (CONFIG_MACH_MSM8226_E9WIFIN) || defined (CONFIG_MACH_MSM8926_E9LTE)
		/*E9 wifi : Change the termination current for battery specification*/
		/*200mA -> 400mA */
		} else if ((ibat_ma * -1) > chip->term_current + 200) {
#else
		} else if ((ibat_ma * -1) > chip->term_current) {
#endif
			pr_debug("Not at EOC, battery current too high\n");
			count = 0;
		} else if (ibat_ma > 0) {
			pr_debug("Charging but system demand increased\n");
#ifdef CONFIG_LGE_PM_4_25V_CHARGING_START
			/*
			 * with 4.35V battery(full charging) ->insert charger(TA)->EOC work is working
			 * but, the ibatt current is + values. after this, the phone doesn't start resume charging
			 * when the vbatt goes to resume voltage.
			 */
				if (chip->from_temp_monitor_vbat_det_high == true) {
					/* pr_err("Charging but system demand increased\n"); */
					qpnp_chg_vbatdet_set(chip, chip->max_voltage_mv - chip->resume_delta_mv);
					chip->from_temp_monitor_vbat_det_high = false;
				}
#endif
			count = 0;
		} else {
			if (count == CONSECUTIVE_COUNT) {
				if (!chip->bat_is_cool && !chip->bat_is_warm) {
					pr_info("End of Charging\n");
					chip->chg_done = true;
#if defined (CONFIG_MACH_MSM8226_E8WIFI) || defined (CONFIG_MACH_MSM8926_E8LTE) || \
    defined (CONFIG_MACH_MSM8926_E9LTE) || defined (CONFIG_MACH_MSM8926_E7LTE_VZW_US) || \
    defined (CONFIG_MACH_MSM8926_T8LTE)
					if((lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO) && chip->ac_online){
						pr_err("DEBUG : EOC state. Turns off EXT OVP FET\n");
						lge_set_chg_path_to_internal();
					}
#endif
#ifdef CONFIG_LGE_PM_FIX_CEC_FAIL
					prop_val.intval = 0;
					if (!chip->bms_psy)
						chip->bms_psy = power_supply_get_by_name("bms");
					if (chip->bms_psy) {
							chip->bms_psy->set_property(chip->bms_psy,
										POWER_SUPPLY_PROP_RELEASE_CV_LOCK, &prop_val);
					}
#endif
				} else {
					pr_info("stop charging: battery is %s, vddmax = %d reached\n",
						chip->bat_is_cool
							? "cool" : "warm",
						qpnp_chg_vddmax_get(chip));
				}
				qpnp_chg_charge_en(chip, 0);
				/* sleep for a second before enabling */
				msleep(2000);
				qpnp_chg_charge_en(chip,
						!chip->charging_disabled);
				pr_debug("psy changed batt_psy\n");
				power_supply_changed(&chip->batt_psy);
				qpnp_chg_enable_irq(&chip->chg_vbatdet_lo);
				goto stop_eoc;
			} else {
				count += 1;
				pr_debug("EOC count = %d\n", count);
			}
		}
	} else {
		pr_debug("not charging\n");
		goto stop_eoc;
	}

check_again_later:
	schedule_delayed_work(&chip->eoc_work,
		msecs_to_jiffies(EOC_CHECK_PERIOD_MS));
	return;

stop_eoc:
	vbat_low_count = 0;
	count = 0;

#ifdef CONFIG_LGE_PM_PWR_KEY_FOR_CHG_LOGO
	is_eoc_work_stop = true;
	if(lge_get_boot_mode() != LGE_BOOT_MODE_CHARGERLOGO)
	{
		pr_info("===== [qpnp_eoc_work] PM RELAX !!! ======\n");
		pm_relax(chip->dev);
	}
#else
    pm_relax(chip->dev);
#endif
}

static void
qpnp_chg_insertion_ocv_work(struct work_struct *work)
{
	struct qpnp_chg_chip *chip = container_of(work,
				struct qpnp_chg_chip, insertion_ocv_work);
	u8 bat_if_sts = 0, charge_en = 0;
	int rc;

	chip->insertion_ocv_uv = get_prop_battery_voltage_now(chip);

	rc = qpnp_chg_read(chip, &bat_if_sts, INT_RT_STS(chip->bat_if_base), 1);
	if (rc)
		pr_err("failed to read bat_if sts %d\n", rc);

	rc = qpnp_chg_read(chip, &charge_en,
			chip->chgr_base + CHGR_CHG_CTRL, 1);
	if (rc)
		pr_err("failed to read bat_if sts %d\n", rc);

	pr_debug("batfet sts = %02x, charge_en = %02x ocv = %d\n",
			bat_if_sts, charge_en, chip->insertion_ocv_uv);
	qpnp_chg_charge_en(chip, !chip->charging_disabled);
	pr_debug("psy changed batt_psy\n");
	power_supply_changed(&chip->batt_psy);
}

#ifndef CONFIG_LGE_PM_4_25V_CHARGING_START
static void
qpnp_chg_soc_check_work(struct work_struct *work)
{
	struct qpnp_chg_chip *chip = container_of(work,
				struct qpnp_chg_chip, soc_check_work);

	get_prop_capacity(chip);
}
#endif

#define HYSTERISIS_DECIDEGC 20
static void
qpnp_chg_adc_notification(enum qpnp_tm_state state, void *ctx)
{
	struct qpnp_chg_chip *chip = ctx;
	bool bat_warm = 0, bat_cool = 0;
	int temp;

	if (state >= ADC_TM_STATE_NUM) {
		pr_err("invalid notification %d\n", state);
		return;
	}

	temp = get_prop_batt_temp(chip);

	pr_debug("temp = %d state = %s\n", temp,
			state == ADC_TM_WARM_STATE ? "warm" : "cool");

#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
	/* LGE_CHANGE_S
	return was temporary added, it cause of low temperature
	discharging/recharging sceanario conflict over LGE's.
	issue is after dischaging low batt temperature,
	it couldn't recharging before approximate 17 degree.
	please someone to fix this code.*/
	return;
#endif
	if (state == ADC_TM_WARM_STATE) {
		if (temp >= chip->warm_bat_decidegc) {
			/* Normal to warm */
			bat_warm = true;
			bat_cool = false;
			chip->adc_param.low_temp =
				chip->warm_bat_decidegc - HYSTERISIS_DECIDEGC;
			chip->adc_param.state_request =
				ADC_TM_COOL_THR_ENABLE;
		} else if (temp >=
				chip->cool_bat_decidegc + HYSTERISIS_DECIDEGC){
			/* Cool to normal */
			bat_warm = false;
			bat_cool = false;

			chip->adc_param.low_temp = chip->cool_bat_decidegc;
			chip->adc_param.high_temp = chip->warm_bat_decidegc;
			chip->adc_param.state_request =
					ADC_TM_HIGH_LOW_THR_ENABLE;
		}
	} else {
		if (temp <= chip->cool_bat_decidegc) {
			/* Normal to cool */
			bat_warm = false;
			bat_cool = true;
			chip->adc_param.high_temp =
				chip->cool_bat_decidegc + HYSTERISIS_DECIDEGC;
			chip->adc_param.state_request =
				ADC_TM_WARM_THR_ENABLE;
		} else if (temp <=
				chip->warm_bat_decidegc - HYSTERISIS_DECIDEGC){
			/* Warm to normal */
			bat_warm = false;
			bat_cool = false;

			chip->adc_param.low_temp = chip->cool_bat_decidegc;
			chip->adc_param.high_temp = chip->warm_bat_decidegc;
			chip->adc_param.state_request =
					ADC_TM_HIGH_LOW_THR_ENABLE;
		}
	}

	if (chip->bat_is_cool ^ bat_cool || chip->bat_is_warm ^ bat_warm) {
		chip->bat_is_cool = bat_cool;
		chip->bat_is_warm = bat_warm;

		/**
		 * set appropriate voltages and currents.
		 *
		 * Note that when the battery is hot or cold, the charger
		 * driver will not resume with SoC. Only vbatdet is used to
		 * determine resume of charging.
		 */
		if (bat_cool || bat_warm) {
			chip->resuming_charging = false;
			qpnp_chg_set_appropriate_vbatdet(chip);

			/* To avoid ARB, only vbatdet is configured in
			 * warm/cold zones. Once vbat < vbatdet the
			 * appropriate vddmax/ibatmax adjustments will
			 * be made in the fast charge interrupt. */
#ifndef CONFIG_LGE_PM_4_25V_CHARGING_START
			bypass_vbatdet_comp(chip, 1);
#endif
			qpnp_chg_charge_en(chip, !chip->charging_disabled);
			qpnp_chg_charge_en(chip, chip->charging_disabled);
			qpnp_chg_charge_en(chip, !chip->charging_disabled);
		} else {
			bypass_vbatdet_comp(chip, 0);
			/* restore normal parameters */
			qpnp_chg_set_appropriate_vbatdet(chip);
			qpnp_chg_set_appropriate_vddmax(chip);
#ifndef CONFIG_LGE_PM
			qpnp_chg_set_appropriate_battery_current(chip);
#endif
		}
	}

	pr_debug("warm %d, cool %d, low = %d deciDegC, high = %d deciDegC\n",
			chip->bat_is_warm, chip->bat_is_cool,
			chip->adc_param.low_temp, chip->adc_param.high_temp);

	if (qpnp_adc_tm_channel_measure(chip->adc_tm_dev, &chip->adc_param))
		pr_err("request ADC error\n");
}

#ifndef CONFIG_LGE_PM
#define MIN_COOL_TEMP	-300
#define MAX_WARM_TEMP	1000

static int
qpnp_chg_configure_jeita(struct qpnp_chg_chip *chip,
		enum power_supply_property psp, int temp_degc)
{
	int rc = 0;

	if ((temp_degc < MIN_COOL_TEMP) || (temp_degc > MAX_WARM_TEMP)) {
		pr_err("Bad temperature request %d\n", temp_degc);
		return -EINVAL;
	}

	mutex_lock(&chip->jeita_configure_lock);
	switch (psp) {
	case POWER_SUPPLY_PROP_COOL_TEMP:
		if (temp_degc >=
			(chip->warm_bat_decidegc - HYSTERISIS_DECIDEGC)) {
			pr_err("Can't set cool %d higher than warm %d - hysterisis %d\n",
					temp_degc, chip->warm_bat_decidegc,
					HYSTERISIS_DECIDEGC);
			rc = -EINVAL;
			goto mutex_unlock;
		}
		if (chip->bat_is_cool)
			chip->adc_param.high_temp =
				temp_degc + HYSTERISIS_DECIDEGC;
		else if (!chip->bat_is_warm)
			chip->adc_param.low_temp = temp_degc;

		chip->cool_bat_decidegc = temp_degc;
		break;
	case POWER_SUPPLY_PROP_WARM_TEMP:
		if (temp_degc <=
			(chip->cool_bat_decidegc + HYSTERISIS_DECIDEGC)) {
			pr_err("Can't set warm %d higher than cool %d + hysterisis %d\n",
					temp_degc, chip->warm_bat_decidegc,
					HYSTERISIS_DECIDEGC);
			rc = -EINVAL;
			goto mutex_unlock;
		}
		if (chip->bat_is_warm)
			chip->adc_param.low_temp =
				temp_degc - HYSTERISIS_DECIDEGC;
		else if (!chip->bat_is_cool)
			chip->adc_param.high_temp = temp_degc;

		chip->warm_bat_decidegc = temp_degc;
		break;
	default:
		rc = -EINVAL;
		goto mutex_unlock;
	}

	schedule_work(&chip->adc_measure_work);

mutex_unlock:
	mutex_unlock(&chip->jeita_configure_lock);
	return rc;
}
#endif

#define POWER_STAGE_REDUCE_CHECK_PERIOD_SECONDS		20
#define POWER_STAGE_REDUCE_MAX_VBAT_UV			3900000
#define POWER_STAGE_REDUCE_MIN_VCHG_UV			4800000
#define POWER_STAGE_SEL_MASK				0x0F
#define POWER_STAGE_REDUCED				0x01
#define POWER_STAGE_DEFAULT				0x0F
static bool
qpnp_chg_is_power_stage_reduced(struct qpnp_chg_chip *chip)
{
	int rc;
	u8 reg;

	rc = qpnp_chg_read(chip, &reg,
				 chip->buck_base + CHGR_BUCK_PSTG_CTRL,
				 1);
	if (rc) {
		pr_err("Error %d reading power stage register\n", rc);
		return false;
	}

	if ((reg & POWER_STAGE_SEL_MASK) == POWER_STAGE_DEFAULT)
		return false;

	return true;
}

static int
qpnp_chg_power_stage_set(struct qpnp_chg_chip *chip, bool reduce)
{
	int rc;
	u8 reg = 0xA5;

	rc = qpnp_chg_write(chip, &reg,
				 chip->buck_base + SEC_ACCESS,
				 1);
	if (rc) {
		pr_err("Error %d writing 0xA5 to buck's 0x%x reg\n",
				rc, SEC_ACCESS);
		return rc;
	}

	reg = POWER_STAGE_DEFAULT;
	if (reduce)
		reg = POWER_STAGE_REDUCED;
	rc = qpnp_chg_write(chip, &reg,
				 chip->buck_base + CHGR_BUCK_PSTG_CTRL,
				 1);

	if (rc)
		pr_err("Error %d writing 0x%x power stage register\n", rc, reg);
	return rc;
}
#ifdef CIDL
#if defined (CONFIG_MACH_MSM8226_E7WIFI) || defined (CONFIG_MACH_MSM8226_E9WIFI) || \
    defined (CONFIG_MACH_MSM8226_E9WIFIN) || defined (CONFIG_MACH_MSM8226_E8WIFI) || \
    defined (CONFIG_MACH_MSM8926_E9LTE)
static int
qpnp_chg_get_pa_therm(struct qpnp_chg_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	rc = qpnp_vadc_read(chip->vadc_dev, P_MUX5_1_1, &results);
	if (rc) {
		pr_err("Unable to read pa_therm rc=%d\n", rc);
		return 0;
	}
	return results.physical;
}
#elif defined (CONFIG_MACH_MSM8926_E8LTE) || defined (CONFIG_MACH_MSM8926_T8LTE)
static int
qpnp_chg_get_pa_therm0(struct qpnp_chg_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	rc = qpnp_vadc_read(chip->vadc_dev, P_MUX5_1_1, &results);
	if (rc) {
		pr_err("Unable to read pa_therm rc=%d\n", rc);
		return 0;
	}
	return results.physical;
}

static int
qpnp_chg_get_pa_therm1(struct qpnp_chg_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	rc = qpnp_vadc_read(chip->vadc_dev, P_MUX8_1_1, &results);
	if (rc) {
		pr_err("Unable to read pa_therm rc=%d\n", rc);
		return 0;
	}
	return results.physical;
}
#endif
#endif
static int
qpnp_chg_get_vusbin_uv(struct qpnp_chg_chip *chip)
{
	int rc = 0;
	struct qpnp_vadc_result results;

	rc = qpnp_vadc_read(chip->vadc_dev, USBIN, &results);
	if (rc) {
		pr_err("Unable to read vbat rc=%d\n", rc);
		return 0;
	}
	return results.physical;
}

static
int get_vusb_averaged(struct qpnp_chg_chip *chip, int sample_count)
{
	int vusb_uv = 0;
	int i;

	/* avoid  overflows */
	if (sample_count > 256)
		sample_count = 256;

	for (i = 0; i < sample_count; i++)
		vusb_uv += qpnp_chg_get_vusbin_uv(chip);

	vusb_uv = vusb_uv / sample_count;
	return vusb_uv;
}

static
int get_vbat_averaged(struct qpnp_chg_chip *chip, int sample_count)
{
	int vbat_uv = 0;
	int i;

	/* avoid  overflows */
	if (sample_count > 256)
		sample_count = 256;

	for (i = 0; i < sample_count; i++)
		vbat_uv += get_prop_battery_voltage_now(chip);

	vbat_uv = vbat_uv / sample_count;
	return vbat_uv;
}

static void
qpnp_chg_reduce_power_stage(struct qpnp_chg_chip *chip)
{
	struct timespec ts;
	bool power_stage_reduced_in_hw = qpnp_chg_is_power_stage_reduced(chip);
	bool reduce_power_stage = false;
	int vbat_uv = get_vbat_averaged(chip, 16);
	int vusb_uv = get_vusb_averaged(chip, 16);
	bool fast_chg =
		(get_prop_charge_type(chip) == POWER_SUPPLY_CHARGE_TYPE_FAST);
	static int count_restore_power_stage;
	static int count_reduce_power_stage;
	bool vchg_loop = get_prop_vchg_loop(chip);
	bool ichg_loop = qpnp_chg_is_ichg_loop_active(chip);
	bool usb_present = qpnp_chg_is_usb_chg_plugged_in(chip);
	bool usb_ma_above_wall =
		(qpnp_chg_usb_iusbmax_get(chip) > USB_WALL_THRESHOLD_MA);

	if (fast_chg
		&& usb_present
		&& usb_ma_above_wall
		&& vbat_uv < POWER_STAGE_REDUCE_MAX_VBAT_UV
		&& vusb_uv > POWER_STAGE_REDUCE_MIN_VCHG_UV)
		reduce_power_stage = true;

	if ((usb_present && usb_ma_above_wall)
		&& (vchg_loop || ichg_loop))
		reduce_power_stage = true;

	if (power_stage_reduced_in_hw && !reduce_power_stage) {
		count_restore_power_stage++;
		count_reduce_power_stage = 0;
	} else if (!power_stage_reduced_in_hw && reduce_power_stage) {
		count_reduce_power_stage++;
		count_restore_power_stage = 0;
	} else if (power_stage_reduced_in_hw == reduce_power_stage) {
		count_restore_power_stage = 0;
		count_reduce_power_stage = 0;
	}

	pr_debug("power_stage_hw = %d reduce_power_stage = %d usb_present = %d usb_ma_above_wall = %d vbat_uv(16) = %d vusb_uv(16) = %d fast_chg = %d , ichg = %d, vchg = %d, restore,reduce = %d, %d\n",
			power_stage_reduced_in_hw, reduce_power_stage,
			usb_present, usb_ma_above_wall,
			vbat_uv, vusb_uv, fast_chg,
			ichg_loop, vchg_loop,
			count_restore_power_stage, count_reduce_power_stage);

	if (!power_stage_reduced_in_hw && reduce_power_stage) {
		if (count_reduce_power_stage >= 2) {
			qpnp_chg_power_stage_set(chip, true);
			power_stage_reduced_in_hw = true;
		}
	}

	if (power_stage_reduced_in_hw && !reduce_power_stage) {
		if (count_restore_power_stage >= 6
				|| (!usb_present || !usb_ma_above_wall)) {
			qpnp_chg_power_stage_set(chip, false);
			power_stage_reduced_in_hw = false;
		}
	}

	if (usb_present && usb_ma_above_wall) {
		getnstimeofday(&ts);
		ts.tv_sec += POWER_STAGE_REDUCE_CHECK_PERIOD_SECONDS;
		alarm_start_range(&chip->reduce_power_stage_alarm,
					timespec_to_ktime(ts),
					timespec_to_ktime(ts));
	} else {
		pr_debug("stopping power stage workaround\n");
		chip->power_stage_workaround_running = false;
	}
}

static void
qpnp_chg_batfet_lcl_work(struct work_struct *work)
{
	struct qpnp_chg_chip *chip = container_of(work,
				struct qpnp_chg_chip, batfet_lcl_work);

	mutex_lock(&chip->batfet_vreg_lock);
	if (qpnp_chg_is_usb_chg_plugged_in(chip) ||
			qpnp_chg_is_dc_chg_plugged_in(chip)) {
		qpnp_chg_regulator_batfet_set(chip, 1);
		pr_debug("disabled ULPM\n");
	} else if (!chip->batfet_ext_en && !qpnp_chg_is_usb_chg_plugged_in(chip)
			&& !qpnp_chg_is_dc_chg_plugged_in(chip)) {
		qpnp_chg_regulator_batfet_set(chip, 0);
		pr_debug("enabled ULPM\n");
	}
	mutex_unlock(&chip->batfet_vreg_lock);
}

static void
qpnp_chg_reduce_power_stage_work(struct work_struct *work)
{
	struct qpnp_chg_chip *chip = container_of(work,
				struct qpnp_chg_chip, reduce_power_stage_work);

	qpnp_chg_reduce_power_stage(chip);
}

static void
qpnp_chg_reduce_power_stage_callback(struct alarm *alarm)
{
	struct qpnp_chg_chip *chip = container_of(alarm, struct qpnp_chg_chip,
						reduce_power_stage_alarm);

	schedule_work(&chip->reduce_power_stage_work);
}

#ifndef CONFIG_LGE_PM
static int
qpnp_dc_power_set_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	struct qpnp_chg_chip *chip = container_of(psy, struct qpnp_chg_chip,
								dc_psy);
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (!val->intval)
			break;

		rc = qpnp_chg_idcmax_set(chip, val->intval / 1000);
		if (rc) {
			pr_err("Error setting idcmax property %d\n", rc);
			return rc;
		}
		chip->maxinput_dc_ma = (val->intval / 1000);

		break;
	default:
		return -EINVAL;
	}

	pr_debug("psy changed dc_psy\n");
	power_supply_changed(&chip->dc_psy);
	return rc;
}
#endif	/* #ifndef CONFIG_LGE_PM */

static int
qpnp_batt_power_set_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	struct qpnp_chg_chip *chip = container_of(psy, struct qpnp_chg_chip,
								batt_psy);
	int rc = 0;

	switch (psp) {
#ifndef CONFIG_LGE_PM
	case POWER_SUPPLY_PROP_COOL_TEMP:
		rc = qpnp_chg_configure_jeita(chip, psp, val->intval);
		break;
	case POWER_SUPPLY_PROP_WARM_TEMP:
		rc = qpnp_chg_configure_jeita(chip, psp, val->intval);
		break;
#endif
	case POWER_SUPPLY_PROP_CAPACITY:
		chip->fake_battery_soc = val->intval;
		power_supply_changed(&chip->batt_psy);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		chip->charging_disabled = !(val->intval);
		if (chip->charging_disabled) {
			/* disable charging */
			qpnp_chg_charge_en(chip, !chip->charging_disabled);
			qpnp_chg_force_run_on_batt(chip,
						chip->charging_disabled);
		} else {
			/* enable charging */
			qpnp_chg_force_run_on_batt(chip,
					chip->charging_disabled);
			qpnp_chg_charge_en(chip, !chip->charging_disabled);
		}
		break;
#ifndef CONFIG_LGE_PM
	case POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL:
		qpnp_batt_system_temp_level_set(chip, val->intval);
		break;
#endif
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		if (qpnp_chg_is_usb_chg_plugged_in(chip) &&
			!(qpnp_is_dc_higher_prio(chip)
			&& qpnp_chg_is_dc_chg_plugged_in(chip)))
			qpnp_chg_iusbmax_set(chip, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_TRIM:
		qpnp_chg_iusb_trim_set(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_SETTLED:
		if (val->intval)
			qpnp_chg_input_current_settled(chip);
		else
			chip->aicl_settled = false;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		qpnp_chg_vinmin_set(chip, val->intval / 1000);
		break;
	case POWER_SUPPLY_PROP_VCHG_LOOP_DBC_BYPASS:
		rc = qpnp_chg_bypass_vchg_loop_debouncer(chip, !!val->intval);
		break;
	default:
		return -EINVAL;
	}

	pr_debug("psy changed batt_psy\n");
	power_supply_changed(&chip->batt_psy);
	return rc;
}

static int
qpnp_chg_setup_flags(struct qpnp_chg_chip *chip)
{
#ifdef CONFIG_LGE_PM
    if (chip->revision >= 0 && chip->type == SMBB)
#else
	if (chip->revision > 0 && chip->type == SMBB)
#endif
		chip->flags |= CHG_FLAGS_VCP_WA;
	if (chip->type == SMBB)
		chip->flags |= BOOST_FLASH_WA;
	if (chip->type == SMBBP) {
		struct device_node *revid_dev_node;
		struct pmic_revid_data *revid_data;

		chip->flags |=  BOOST_FLASH_WA;

		revid_dev_node = of_parse_phandle(chip->spmi->dev.of_node,
						"qcom,pmic-revid", 0);
		if (!revid_dev_node) {
			pr_err("Missing qcom,pmic-revid property\n");
			return -EINVAL;
		}
		revid_data = get_revid_data(revid_dev_node);
		if (IS_ERR(revid_data)) {
			pr_err("Couldnt get revid data rc = %ld\n",
						PTR_ERR(revid_data));
			return PTR_ERR(revid_data);
		}

		if (revid_data->rev4 < PM8226_V2P1_REV4
			|| ((revid_data->rev4 == PM8226_V2P1_REV4)
				&& (revid_data->rev3 <= PM8226_V2P1_REV3))) {
			chip->flags |= POWER_STAGE_WA;
		}
	}
	return 0;
}

static int
qpnp_chg_request_irqs(struct qpnp_chg_chip *chip)
{
	int rc = 0;
	struct resource *resource;
	struct spmi_resource *spmi_resource;
	u8 subtype;
	struct spmi_device *spmi = chip->spmi;

	spmi_for_each_container_dev(spmi_resource, chip->spmi) {
		if (!spmi_resource) {
				pr_err("qpnp_chg: spmi resource absent\n");
			return rc;
		}

		resource = spmi_get_resource(spmi, spmi_resource,
						IORESOURCE_MEM, 0);
		if (!(resource && resource->start)) {
			pr_err("node %s IO resource absent!\n",
				spmi->dev.of_node->full_name);
			return rc;
		}

		rc = qpnp_chg_read(chip, &subtype,
				resource->start + REG_OFFSET_PERP_SUBTYPE, 1);
		if (rc) {
			pr_err("Peripheral subtype read failed rc=%d\n", rc);
			return rc;
		}

		switch (subtype) {
		case SMBB_CHGR_SUBTYPE:
		case SMBBP_CHGR_SUBTYPE:
		case SMBCL_CHGR_SUBTYPE:
			chip->chg_fastchg.irq = spmi_get_irq_byname(spmi,
						spmi_resource, "fast-chg-on");
			if (chip->chg_fastchg.irq < 0) {
				pr_err("Unable to get fast-chg-on irq\n");
				return rc;
			}

			chip->chg_trklchg.irq = spmi_get_irq_byname(spmi,
						spmi_resource, "trkl-chg-on");
			if (chip->chg_trklchg.irq < 0) {
				pr_err("Unable to get trkl-chg-on irq\n");
				return rc;
			}

			chip->chg_failed.irq = spmi_get_irq_byname(spmi,
						spmi_resource, "chg-failed");
			if (chip->chg_failed.irq < 0) {
				pr_err("Unable to get chg_failed irq\n");
				return rc;
			}

			chip->chg_vbatdet_lo.irq = spmi_get_irq_byname(spmi,
						spmi_resource, "vbat-det-lo");
			if (chip->chg_vbatdet_lo.irq < 0) {
				pr_err("Unable to get fast-chg-on irq\n");
				return rc;
			}

			rc |= devm_request_irq(chip->dev, chip->chg_failed.irq,
				qpnp_chg_chgr_chg_failed_irq_handler,
				IRQF_TRIGGER_RISING, "chg-failed", chip);
			if (rc < 0) {
				pr_err("Can't request %d chg-failed: %d\n",
						chip->chg_failed.irq, rc);
				return rc;
			}

			rc |= devm_request_irq(chip->dev, chip->chg_fastchg.irq,
					qpnp_chg_chgr_chg_fastchg_irq_handler,
					IRQF_TRIGGER_RISING |
					IRQF_TRIGGER_FALLING,
					"fast-chg-on", chip);
			if (rc < 0) {
				pr_err("Can't request %d fast-chg-on: %d\n",
						chip->chg_fastchg.irq, rc);
				return rc;
			}

			rc |= devm_request_irq(chip->dev, chip->chg_trklchg.irq,
				qpnp_chg_chgr_chg_trklchg_irq_handler,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				"trkl-chg-on", chip);
			if (rc < 0) {
				pr_err("Can't request %d trkl-chg-on: %d\n",
						chip->chg_trklchg.irq, rc);
				return rc;
			}

			rc |= devm_request_irq(chip->dev,
				chip->chg_vbatdet_lo.irq,
				qpnp_chg_vbatdet_lo_irq_handler,
				IRQF_TRIGGER_RISING,
				"vbat-det-lo", chip);
			if (rc < 0) {
				pr_err("Can't request %d vbat-det-lo: %d\n",
						chip->chg_vbatdet_lo.irq, rc);
				return rc;
			}

			qpnp_chg_irq_wake_enable(&chip->chg_trklchg);
			qpnp_chg_irq_wake_enable(&chip->chg_failed);
			qpnp_chg_irq_wake_enable(&chip->chg_vbatdet_lo);
			qpnp_chg_disable_irq(&chip->chg_vbatdet_lo);

			break;
		case SMBB_BAT_IF_SUBTYPE:
		case SMBBP_BAT_IF_SUBTYPE:
		case SMBCL_BAT_IF_SUBTYPE:
			chip->batt_pres.irq = spmi_get_irq_byname(spmi,
						spmi_resource, "batt-pres");
			if (chip->batt_pres.irq < 0) {
				pr_err("Unable to get batt-pres irq\n");
				return rc;
			}
			rc = devm_request_irq(chip->dev, chip->batt_pres.irq,
				qpnp_chg_bat_if_batt_pres_irq_handler,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
				| IRQF_SHARED | IRQF_ONESHOT,
				"batt-pres", chip);
			if (rc < 0) {
				pr_err("Can't request %d batt-pres irq: %d\n",
						chip->batt_pres.irq, rc);
				return rc;
			}

			qpnp_chg_irq_wake_enable(&chip->batt_pres);

/* LGE not useed BTM feature  */
#ifndef CONFIG_LGE_PM
			chip->batt_temp_ok.irq = spmi_get_irq_byname(spmi,
						spmi_resource, "bat-temp-ok");
			if (chip->batt_temp_ok.irq < 0) {
				pr_err("Unable to get bat-temp-ok irq\n");
				return rc;
			}
			rc = devm_request_irq(chip->dev, chip->batt_temp_ok.irq,
				qpnp_chg_bat_if_batt_temp_irq_handler,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				"bat-temp-ok", chip);
			if (rc < 0) {
				pr_err("Can't request %d bat-temp-ok irq: %d\n",
						chip->batt_temp_ok.irq, rc);
				return rc;
			}
			qpnp_chg_bat_if_batt_temp_irq_handler(0, chip);

			qpnp_chg_irq_wake_enable(&chip->batt_temp_ok);
#endif
			break;
		case SMBB_BUCK_SUBTYPE:
		case SMBBP_BUCK_SUBTYPE:
		case SMBCL_BUCK_SUBTYPE:
			break;

		case SMBB_USB_CHGPTH_SUBTYPE:
		case SMBBP_USB_CHGPTH_SUBTYPE:
		case SMBCL_USB_CHGPTH_SUBTYPE:
			if (chip->ovp_monitor_enable) {
				chip->coarse_det_usb.irq =
					spmi_get_irq_byname(spmi,
					spmi_resource, "coarse-det-usb");
				if (chip->coarse_det_usb.irq < 0) {
					pr_err("Can't get coarse-det irq\n");
					return rc;
				}
				rc = devm_request_irq(chip->dev,
					chip->coarse_det_usb.irq,
					qpnp_chg_coarse_det_usb_irq_handler,
					IRQF_TRIGGER_RISING |
					IRQF_TRIGGER_FALLING,
					"coarse-det-usb", chip);
				if (rc < 0) {
					pr_err("Can't req %d coarse-det: %d\n",
						chip->coarse_det_usb.irq, rc);
					return rc;
				}
			}

			chip->usbin_valid.irq = spmi_get_irq_byname(spmi,
						spmi_resource, "usbin-valid");
			if (chip->usbin_valid.irq < 0) {
				pr_err("Unable to get usbin irq\n");
				return rc;
			}
			rc = devm_request_irq(chip->dev, chip->usbin_valid.irq,
				qpnp_chg_usb_usbin_valid_irq_handler,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
					"usbin-valid", chip);
			if (rc < 0) {
				pr_err("Can't request %d usbin-valid: %d\n",
						chip->usbin_valid.irq, rc);
				return rc;
			}

			chip->chg_gone.irq = spmi_get_irq_byname(spmi,
						spmi_resource, "chg-gone");
			if (chip->chg_gone.irq < 0) {
				pr_err("Unable to get chg-gone irq\n");
				return rc;
			}
			rc = devm_request_irq(chip->dev, chip->chg_gone.irq,
				qpnp_chg_usb_chg_gone_irq_handler,
				IRQF_TRIGGER_RISING,
					"chg-gone", chip);
			if (rc < 0) {
				pr_err("Can't request %d chg-gone: %d\n",
						chip->chg_gone.irq, rc);
				return rc;
			}

			if ((subtype == SMBBP_USB_CHGPTH_SUBTYPE) ||
				(subtype == SMBCL_USB_CHGPTH_SUBTYPE)) {
				chip->usb_ocp.irq = spmi_get_irq_byname(spmi,
						spmi_resource, "usb-ocp");
				if (chip->usb_ocp.irq < 0) {
					pr_err("Unable to get usbin irq\n");
					return rc;
				}
				rc = devm_request_irq(chip->dev,
					chip->usb_ocp.irq,
					qpnp_chg_usb_usb_ocp_irq_handler,
					IRQF_TRIGGER_RISING, "usb-ocp", chip);
				if (rc < 0) {
					pr_err("Can't request %d usb-ocp: %d\n",
							chip->usb_ocp.irq, rc);
					return rc;
				}

				qpnp_chg_irq_wake_enable(&chip->usb_ocp);
			}

			qpnp_chg_irq_wake_enable(&chip->usbin_valid);
			qpnp_chg_irq_wake_enable(&chip->chg_gone);
			break;
		case SMBB_DC_CHGPTH_SUBTYPE:
			chip->dcin_valid.irq = spmi_get_irq_byname(spmi,
					spmi_resource, "dcin-valid");
			if (chip->dcin_valid.irq < 0) {
				pr_err("Unable to get dcin irq\n");
				return -rc;
			}
			rc = devm_request_irq(chip->dev, chip->dcin_valid.irq,
				qpnp_chg_dc_dcin_valid_irq_handler,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				"dcin-valid", chip);
			if (rc < 0) {
				pr_err("Can't request %d dcin-valid: %d\n",
						chip->dcin_valid.irq, rc);
				return rc;
			}

			qpnp_chg_irq_wake_enable(&chip->dcin_valid);
			break;
		}
	}

	return rc;
}
#ifdef CONFIG_LGE_USING_CHG_LED
static void qpnp_chg_led_set(struct led_classdev *led_cdev,
								enum led_brightness value)
{
	struct qpnp_chg_chip *chip = container_of(led_cdev, struct qpnp_chg_chip, cdev);
	int rc;
	int duty_us;

	chip->cdev.brightness = value;
	if (chip->cdev.brightness > 255)
		chip->cdev.brightness = 255;

	if (chip->cdev.brightness < 0)
		chip->cdev.brightness = 0;

	duty_us = ((chip->pwm_cfg->pwm_period_us * chip->cdev.brightness)/255);

	pwm_config(chip->pwm_cfg->pwm_dev, duty_us, chip->pwm_cfg->pwm_period_us);
	pwm_enable(chip->pwm_cfg->pwm_dev);

	if (value) {
		rc = qpnp_chg_masked_write(chip,
						chip->chgr_base + 0x4D,
						0x3, 0x3, 1);
	} else {
		rc = qpnp_chg_masked_write(chip,
						chip->chgr_base + 0x4D,
						0x3, 0x0, 1);
	}
}

static enum led_brightness qpnp_chg_led_get(struct led_classdev *led_cdev)
{
	struct qpnp_chg_chip *chip = container_of(led_cdev, struct qpnp_chg_chip, cdev);

	return chip->cdev.brightness;
}

#define LED_BUFF_SIZE 50
static ssize_t red_blink_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct qpnp_chg_chip *chip;
	int type = 0;
	int offMS = 0;
	int onMS = 0;
	int brightness = 0;

	chip = container_of(led_cdev, struct qpnp_chg_chip, cdev);

	brightness = led_cdev->brightness;

	return snprintf(buf, LED_BUFF_SIZE, "%d, %d, %d, %d\n", type, offMS, onMS, brightness);
}

static ssize_t red_blink_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct qpnp_chg_chip *chip;
	int brightness;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	int type = 0;
	int offMS = 0;	int onMS = 0;

	if (sscanf(buf, "%d,%d,%d,%d", &type, &offMS, &onMS, &brightness) != 4)
		printk("bad arguments ");

	chip = container_of(led_cdev, struct qpnp_chg_chip, cdev);
	chip->cdev.brightness = brightness;

	printk(KERN_INFO "%s(), LINE:%d type:%d, offMS:%d, onMS:%d, brightness:%d\n",
			__func__, __LINE__, type, offMS, onMS, brightness);

	qpnp_chg_led_set(led_cdev, brightness);
	return count;
}
static DEVICE_ATTR(red_blink, 0664, red_blink_show, red_blink_store);
static struct attribute *blink_attrs[] = {
	&dev_attr_red_blink.attr,
	NULL
};
static const struct attribute_group blink_attr_group = {
	.attrs = blink_attrs,
};
#endif

#ifdef CONFIG_LGE_PM_THERMAL
static int qpnp_thermal_mitigation;
static int
qpnp_set_thermal_chg_current(const char *val, struct kernel_param *kp)
{
	int ret;

	ret = param_set_int(val, kp);
	if (ret) {
		pr_err("error setting value %d\n", ret);
		return ret;
	}

	if (!qpnp_chg) {
		pr_err("called before init\n");
		return ret;
	}
	if (qpnp_thermal_mitigation <= 0)
		qpnp_chg->chg_current_te = qpnp_chg->max_bat_chg_current;
	else
		qpnp_chg->chg_current_te = qpnp_thermal_mitigation;

	pr_err("thermal-engine set charging current to %d(mA)\n", qpnp_chg->chg_current_te);

	return 0;
}
module_param_call(qpnp_thermal_mitigation, qpnp_set_thermal_chg_current,
	param_get_uint, &qpnp_thermal_mitigation, 0644);
#endif

static int
qpnp_chg_load_battery_data(struct qpnp_chg_chip *chip)
{
	struct bms_battery_data batt_data;
	struct device_node *node;
	struct qpnp_vadc_result result;
	int rc;

	node = of_find_node_by_name(chip->spmi->dev.of_node,
			"qcom,battery-data");
	if (node) {
		memset(&batt_data, 0, sizeof(struct bms_battery_data));
		rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX2_BAT_ID, &result);
		if (rc) {
			pr_err("error reading batt id channel = %d, rc = %d\n",
						LR_MUX2_BAT_ID, rc);
			return rc;
		}

		batt_data.max_voltage_uv = -1;
		batt_data.iterm_ua = -1;
		rc = of_batterydata_read_data(node,
				&batt_data, result.physical);
		if (rc) {
			pr_err("failed to read battery data: %d\n", rc);
			return rc;
		}

		if (batt_data.max_voltage_uv >= 0) {
			chip->max_voltage_mv = batt_data.max_voltage_uv / 1000;
			chip->safe_voltage_mv = chip->max_voltage_mv
				+ MAX_DELTA_VDD_MAX_MV;
		}
		if (batt_data.iterm_ua >= 0)
			chip->term_current = batt_data.iterm_ua / 1000;
	}

	return 0;
}

#define WDOG_EN_BIT	BIT(7)
static int
qpnp_chg_hwinit(struct qpnp_chg_chip *chip, u8 subtype,
				struct spmi_resource *spmi_resource)
{
	int rc = 0;
	u8 reg = 0;
	struct regulator_init_data *init_data;
	struct regulator_desc *rdesc;

	switch (subtype) {
	case SMBB_CHGR_SUBTYPE:
	case SMBBP_CHGR_SUBTYPE:
	case SMBCL_CHGR_SUBTYPE:
		qpnp_chg_vbatweak_set(chip, chip->batt_weak_voltage_mv);

		rc = qpnp_chg_vinmin_set(chip, chip->min_voltage_mv);
		if (rc) {
			pr_debug("failed setting  min_voltage rc=%d\n", rc);
			return rc;
		}
		rc = qpnp_chg_vddsafe_set(chip, chip->safe_voltage_mv);
		if (rc) {
			pr_debug("failed setting safe_voltage rc=%d\n", rc);
			return rc;
		}
		rc = qpnp_chg_vbatdet_set(chip,
				chip->max_voltage_mv - chip->resume_delta_mv);
		if (rc) {
			pr_debug("failed setting resume_voltage rc=%d\n", rc);
			return rc;
		}
		rc = qpnp_chg_ibatmax_set(chip, chip->max_bat_chg_current);
		if (rc) {
			pr_debug("failed setting ibatmax rc=%d\n", rc);
			return rc;
		}
		if (chip->term_current) {
			rc = qpnp_chg_ibatterm_set(chip, chip->term_current);
			if (rc) {
				pr_debug("failed setting ibatterm rc=%d\n", rc);
				return rc;
			}
		}
		rc = qpnp_chg_ibatsafe_set(chip, chip->safe_current);
		if (rc) {
			pr_debug("failed setting ibat_Safe rc=%d\n", rc);
			return rc;
		}
		rc = qpnp_chg_tchg_max_set(chip, chip->tchg_mins);
		if (rc) {
			pr_debug("failed setting tchg_mins rc=%d\n", rc);
			return rc;
		}
#ifdef CONFIG_LGE_PM
		chip->chg_timer = 1;
#endif
		/* HACK: Disable wdog */
		rc = qpnp_chg_masked_write(chip, chip->chgr_base + 0x62,
			0xFF, 0xA0, 1);

		/* HACK: use analog EOC */
		rc = qpnp_chg_masked_write(chip, chip->chgr_base +
			CHGR_IBAT_TERM_CHGR,
			0xFF, 0x08, 1);

		/* HACK: trkl stuck workaround */

		rc = qpnp_chg_masked_write(chip,
			chip->chgr_base + SEC_ACCESS,
			0xFF,
			0xA5, 1);

		rc = qpnp_chg_masked_write(chip, chip->chgr_base +
			CHG_OVR0,
			0xFF, 0x00, 1);

		rc = qpnp_chg_masked_write(chip,
			chip->chgr_base + SEC_ACCESS,
			0xFF,
			0xA5, 1);

		rc = qpnp_chg_masked_write(chip, chip->chgr_base +
			CHG_TRICKLE_CLAMP,
			0xFF, 0x00, 1);

		rc = qpnp_chg_read(chip, &chip->chg_temp_thresh_default,
				chip->chgr_base + CHGR_CHG_TEMP_THRESH, 1);
		if (rc) {
			pr_debug("read CHG_TEMP_THRESH failed, rc = %d\n", rc);
			chip->chg_temp_thresh_default =
				CHG_TEMP_THRESH_DEFAULT;
		}

		init_data = of_get_regulator_init_data(chip->dev,
						       spmi_resource->of_node);
		if (!init_data) {
			pr_err("unable to get regulator init data for flash_wa\n");
			return -ENOMEM;
		}

		if (init_data->constraints.name) {
			rdesc			= &(chip->flash_wa_vreg.rdesc);
			rdesc->owner		= THIS_MODULE;
			rdesc->type		= REGULATOR_VOLTAGE;
			rdesc->ops		= &qpnp_chg_flash_wa_reg_ops;
			rdesc->name		= init_data->constraints.name;

			init_data->constraints.valid_ops_mask
				|= REGULATOR_CHANGE_STATUS;

			chip->flash_wa_vreg.rdev =
				regulator_register(rdesc, chip->dev, init_data,
						chip, spmi_resource->of_node);
			if (IS_ERR(chip->flash_wa_vreg.rdev)) {
				rc = PTR_ERR(chip->flash_wa_vreg.rdev);
				chip->flash_wa_vreg.rdev = NULL;
				pr_err("Flash wa failed, rc=%d\n", rc);
				return rc;
			}
		}
		break;
	case SMBB_BUCK_SUBTYPE:
	case SMBBP_BUCK_SUBTYPE:
	case SMBCL_BUCK_SUBTYPE:
		rc = qpnp_chg_toggle_chg_done_logic(chip, 0);
		if (rc)
			return rc;

		rc = qpnp_chg_masked_write(chip,
			chip->buck_base + CHGR_BUCK_BCK_VBAT_REG_MODE,
			BUCK_VBAT_REG_NODE_SEL_BIT,
			BUCK_VBAT_REG_NODE_SEL_BIT, 1);
		if (rc) {
			pr_debug("failed to enable IR drop comp rc=%d\n", rc);
			return rc;
		}

		rc = qpnp_chg_read(chip, &chip->trim_center,
				chip->buck_base + BUCK_CTRL_TRIM1, 1);
		if (rc) {
			pr_debug("failed to read trim center rc=%d\n", rc);
			return rc;
		}
		chip->trim_center >>= 4;
		pr_debug("trim center = %02x\n", chip->trim_center);
		break;
	case SMBB_BAT_IF_SUBTYPE:
	case SMBBP_BAT_IF_SUBTYPE:
	case SMBCL_BAT_IF_SUBTYPE:
		/* Select battery presence detection */
		switch (chip->bpd_detection) {
		case BPD_TYPE_BAT_THM:
			reg = BAT_THM_EN;
			break;
		case BPD_TYPE_BAT_ID:
			reg = BAT_ID_EN;
			break;
		case BPD_TYPE_BAT_THM_BAT_ID:
			reg = BAT_THM_EN | BAT_ID_EN;
			break;
		default:
			reg = BAT_THM_EN;
			break;
		}

		rc = qpnp_chg_masked_write(chip,
			chip->bat_if_base + BAT_IF_BPD_CTRL,
			BAT_IF_BPD_CTRL_SEL,
			reg, 1);
		if (rc) {
			pr_debug("failed to chose BPD rc=%d\n", rc);
			return rc;
		}
		/* Force on VREF_BAT_THM */
		rc = qpnp_chg_masked_write(chip,
			chip->bat_if_base + BAT_IF_VREF_BAT_THM_CTRL,
			VREF_BATT_THERM_FORCE_ON,
			VREF_BATT_THERM_FORCE_ON, 1);
		if (rc) {
			pr_debug("failed to force on VREF_BAT_THM rc=%d\n", rc);
			return rc;
		}

		init_data = of_get_regulator_init_data(chip->dev,
					       spmi_resource->of_node);

		if (init_data->constraints.name) {
			rdesc			= &(chip->batfet_vreg.rdesc);
			rdesc->owner		= THIS_MODULE;
			rdesc->type		= REGULATOR_VOLTAGE;
			rdesc->ops		= &qpnp_chg_batfet_vreg_ops;
			rdesc->name		= init_data->constraints.name;

			init_data->constraints.valid_ops_mask
				|= REGULATOR_CHANGE_STATUS;

			chip->batfet_vreg.rdev = regulator_register(rdesc,
					chip->dev, init_data, chip,
					spmi_resource->of_node);
			if (IS_ERR(chip->batfet_vreg.rdev)) {
				rc = PTR_ERR(chip->batfet_vreg.rdev);
				chip->batfet_vreg.rdev = NULL;
				if (rc != -EPROBE_DEFER)
					pr_err("batfet reg failed, rc=%d\n",
							rc);
				return rc;
			}
		}
		break;
	case SMBB_USB_CHGPTH_SUBTYPE:
	case SMBBP_USB_CHGPTH_SUBTYPE:
	case SMBCL_USB_CHGPTH_SUBTYPE:
		if (qpnp_chg_is_usb_chg_plugged_in(chip)) {
			rc = qpnp_chg_masked_write(chip,
				chip->usb_chgpth_base + CHGR_USB_ENUM_T_STOP,
				ENUM_T_STOP_BIT,
				ENUM_T_STOP_BIT, 1);
			if (rc) {
				pr_err("failed to write enum stop rc=%d\n", rc);
				return -ENXIO;
			}
		}

		init_data = of_get_regulator_init_data(chip->dev,
						       spmi_resource->of_node);
		if (!init_data) {
			pr_err("unable to allocate memory\n");
			return -ENOMEM;
		}

		if (init_data->constraints.name) {
			if (of_get_property(chip->dev->of_node,
						"otg-parent-supply", NULL))
				init_data->supply_regulator = "otg-parent";

			rdesc			= &(chip->otg_vreg.rdesc);
			rdesc->owner		= THIS_MODULE;
			rdesc->type		= REGULATOR_VOLTAGE;
			rdesc->ops		= &qpnp_chg_otg_reg_ops;
			rdesc->name		= init_data->constraints.name;

			init_data->constraints.valid_ops_mask
				|= REGULATOR_CHANGE_STATUS;

			chip->otg_vreg.rdev = regulator_register(rdesc,
					chip->dev, init_data, chip,
					spmi_resource->of_node);
			if (IS_ERR(chip->otg_vreg.rdev)) {
				rc = PTR_ERR(chip->otg_vreg.rdev);
				chip->otg_vreg.rdev = NULL;
				if (rc != -EPROBE_DEFER)
					pr_err("OTG reg failed, rc=%d\n", rc);
				return rc;
			}
		}

		rc = qpnp_chg_masked_write(chip,
			chip->usb_chgpth_base + USB_OVP_CTL,
			USB_VALID_DEB_20MS,
			USB_VALID_DEB_20MS, 1);

		rc = qpnp_chg_masked_write(chip,
			chip->usb_chgpth_base + CHGR_USB_ENUM_T_STOP,
			ENUM_T_STOP_BIT,
			ENUM_T_STOP_BIT, 1);

		rc = qpnp_chg_masked_write(chip,
			chip->usb_chgpth_base + SEC_ACCESS,
			0xFF,
			0xA5, 1);

		rc = qpnp_chg_masked_write(chip,
			chip->usb_chgpth_base + USB_CHG_GONE_REV_BST,
			0xFF,
			0x80, 1);

		if ((subtype == SMBBP_USB_CHGPTH_SUBTYPE) ||
			(subtype == SMBCL_USB_CHGPTH_SUBTYPE)) {
			rc = qpnp_chg_masked_write(chip,
				chip->usb_chgpth_base + USB_OCP_THR,
				OCP_THR_MASK,
				OCP_THR_900_MA, 1);
			if (rc)
				pr_err("Failed to configure OCP rc = %d\n", rc);
		}

		break;
	case SMBB_DC_CHGPTH_SUBTYPE:
		break;
	case SMBB_BOOST_SUBTYPE:
	case SMBBP_BOOST_SUBTYPE:
		init_data = of_get_regulator_init_data(chip->dev,
					       spmi_resource->of_node);
		if (!init_data) {
			pr_err("unable to allocate memory\n");
			return -ENOMEM;
		}

		if (init_data->constraints.name) {
			if (of_get_property(chip->dev->of_node,
						"boost-parent-supply", NULL))
				init_data->supply_regulator = "boost-parent";

			rdesc			= &(chip->boost_vreg.rdesc);
			rdesc->owner		= THIS_MODULE;
			rdesc->type		= REGULATOR_VOLTAGE;
			rdesc->ops		= &qpnp_chg_boost_reg_ops;
			rdesc->name		= init_data->constraints.name;

			init_data->constraints.valid_ops_mask
				|= REGULATOR_CHANGE_STATUS
					| REGULATOR_CHANGE_VOLTAGE;

			chip->boost_vreg.rdev = regulator_register(rdesc,
					chip->dev, init_data, chip,
					spmi_resource->of_node);
			if (IS_ERR(chip->boost_vreg.rdev)) {
				rc = PTR_ERR(chip->boost_vreg.rdev);
				chip->boost_vreg.rdev = NULL;
				if (rc != -EPROBE_DEFER)
					pr_err("boost reg failed, rc=%d\n", rc);
				return rc;
			}
		}
		break;
	case SMBB_MISC_SUBTYPE:
	case SMBBP_MISC_SUBTYPE:
	case SMBCL_MISC_SUBTYPE:
		if (subtype == SMBB_MISC_SUBTYPE)
			chip->type = SMBB;
		else if (subtype == SMBBP_MISC_SUBTYPE)
			chip->type = SMBBP;
		else if (subtype == SMBCL_MISC_SUBTYPE)
			chip->type = SMBCL;

		pr_debug("Setting BOOT_DONE\n");
		rc = qpnp_chg_masked_write(chip,
			chip->misc_base + CHGR_MISC_BOOT_DONE,
			CHGR_BOOT_DONE, CHGR_BOOT_DONE, 1);
		rc = qpnp_chg_read(chip, &reg,
				 chip->misc_base + MISC_REVISION2, 1);
		if (rc) {
			pr_err("failed to read revision register rc=%d\n", rc);
			return rc;
		}

		chip->revision = reg;
		break;
	default:
		pr_err("Invalid peripheral subtype\n");
	}
	return rc;
}

/* LGE_CHANGE_S: Cable Detect & Current Set */
#ifdef CONFIG_LGE_PM
static unsigned int cable_smem_size;
unsigned int lge_chg_cable_type(void)
{
	return cable_type;
}
EXPORT_SYMBOL(lge_chg_cable_type);
#endif
/* LGE_CHANGE_E */

#define OF_PROP_READ(chip, prop, qpnp_dt_property, retval, optional)	\
do {									\
	if (retval)							\
		break;							\
									\
	retval = of_property_read_u32(chip->spmi->dev.of_node,		\
					"qcom," qpnp_dt_property,	\
					&chip->prop);			\
									\
	if ((retval == -EINVAL) && optional)				\
		retval = 0;						\
	else if (retval)						\
		pr_err("Error reading " #qpnp_dt_property		\
				" property rc = %d\n", rc);		\
} while (0)

static int
qpnp_charger_read_dt_props(struct qpnp_chg_chip *chip)
{
	int rc = 0;
	const char *bpd;

	OF_PROP_READ(chip, max_voltage_mv, "vddmax-mv", rc, 0);
	OF_PROP_READ(chip, min_voltage_mv, "vinmin-mv", rc, 0);
	OF_PROP_READ(chip, safe_voltage_mv, "vddsafe-mv", rc, 0);
	OF_PROP_READ(chip, resume_delta_mv, "vbatdet-delta-mv", rc, 0);
	OF_PROP_READ(chip, safe_current, "ibatsafe-ma", rc, 0);
	OF_PROP_READ(chip, max_bat_chg_current, "ibatmax-ma", rc, 0);
	if (rc)
		pr_err("failed to read required dt parameters %d\n", rc);

	OF_PROP_READ(chip, term_current, "ibatterm-ma", rc, 1);
	OF_PROP_READ(chip, maxinput_dc_ma, "maxinput-dc-ma", rc, 1);
	OF_PROP_READ(chip, maxinput_usb_ma, "maxinput-usb-ma", rc, 1);
	OF_PROP_READ(chip, warm_bat_decidegc, "warm-bat-decidegc", rc, 1);
	OF_PROP_READ(chip, cool_bat_decidegc, "cool-bat-decidegc", rc, 1);
	OF_PROP_READ(chip, tchg_mins, "tchg-mins", rc, 1);
	OF_PROP_READ(chip, hot_batt_p, "batt-hot-percentage", rc, 1);
	OF_PROP_READ(chip, cold_batt_p, "batt-cold-percentage", rc, 1);
	OF_PROP_READ(chip, soc_resume_limit, "resume-soc", rc, 1);
	OF_PROP_READ(chip, batt_weak_voltage_mv, "vbatweak-mv", rc, 1);
	OF_PROP_READ(chip, vbatdet_max_err_mv, "vbatdet-maxerr-mv", rc, 1);

	if (rc)
		return rc;

	rc = of_property_read_string(chip->spmi->dev.of_node,
		"qcom,bpd-detection", &bpd);
	if (rc) {
		/* Select BAT_THM as default BPD scheme */
		chip->bpd_detection = BPD_TYPE_BAT_THM;
		rc = 0;
	} else {
		chip->bpd_detection = get_bpd(bpd);
		if (chip->bpd_detection < 0) {
			pr_err("failed to determine bpd schema %d\n", rc);
			return rc;
		}
	}

/* LGE_CHANGE_S */
#ifdef CONFIG_LGE_PM_CHARGING_EXTERNAL_OVP
	chip->ext_ovp_gpio = of_get_named_gpio_flags(chip->spmi->dev.of_node, "lge,ext_ovp_gpio" , 0, NULL);
	if(of_property_read_u32(chip->spmi->dev.of_node, "lge,ext_ovp_slow_chg_trim",
				&chip->ext_ovp_slow_chg_trim))
		pr_err("lge,ext_ovp_slow_chg_trim is not defined\n");
#endif
/* LGE_CHANGE_E */
	if (!chip->vbatdet_max_err_mv)
		chip->vbatdet_max_err_mv = VBATDET_MAX_ERR_MV;

	/* Look up JEITA compliance parameters if cool and warm temp provided */
	if (chip->cool_bat_decidegc || chip->warm_bat_decidegc) {
		chip->adc_tm_dev = qpnp_get_adc_tm(chip->dev, "chg");
		if (IS_ERR(chip->adc_tm_dev)) {
			rc = PTR_ERR(chip->adc_tm_dev);
			if (rc != -EPROBE_DEFER)
				pr_err("adc-tm not ready, defer probe\n");
			return rc;
		}

		OF_PROP_READ(chip, warm_bat_chg_ma, "ibatmax-warm-ma", rc, 1);
		OF_PROP_READ(chip, cool_bat_chg_ma, "ibatmax-cool-ma", rc, 1);
		OF_PROP_READ(chip, warm_bat_mv, "warm-bat-mv", rc, 1);
		OF_PROP_READ(chip, cool_bat_mv, "cool-bat-mv", rc, 1);
		if (rc)
			return rc;
	}

	/* Get the use-external-rsense property */
	chip->use_external_rsense = of_property_read_bool(
			chip->spmi->dev.of_node,
			"qcom,use-external-rsense");

	/* Get the btc-disabled property */
	chip->btc_disabled = of_property_read_bool(chip->spmi->dev.of_node,
					"qcom,btc-disabled");

	ext_ovp_present = of_property_read_bool(chip->spmi->dev.of_node,
					"qcom,ext-ovp-present");

	/* Check if external IOVP part is configured */
	chip->ext_ovp_isns_gpio = of_get_named_gpio(chip->spmi->dev.of_node,
					"qcom,ext-ovp-isns-enable-gpio", 0);
	if (gpio_is_valid(chip->ext_ovp_isns_gpio)) {
		ext_ovp_isns_present = true;
		rc = of_property_read_u32(chip->spmi->dev.of_node,
				"qcom,ext-ovp-isns-r-ohm", &ext_ovp_isns_r);
		if (rc)
			return rc;
	}

	/* Get the charging-disabled property */
	chip->charging_disabled = of_property_read_bool(chip->spmi->dev.of_node,
					"qcom,charging-disabled");

	chip->ovp_monitor_enable = of_property_read_bool(chip->spmi->dev.of_node,
					"qcom,ovp-monitor-en");

	/* Get the duty-cycle-100p property */
	chip->duty_cycle_100p = of_property_read_bool(
					chip->spmi->dev.of_node,
					"qcom,duty-cycle-100p");

	/* Get the fake-batt-values property */
	chip->use_default_batt_values =
			of_property_read_bool(chip->spmi->dev.of_node,
					"qcom,use-default-batt-values");

	/* Disable charging when faking battery values */
	if (chip->use_default_batt_values)
		chip->charging_disabled = true;

	chip->power_stage_workaround_enable =
			of_property_read_bool(chip->spmi->dev.of_node,
					"qcom,power-stage-reduced");
	chip->ibat_calibration_enabled =
			of_property_read_bool(chip->spmi->dev.of_node,
					"qcom,ibat-calibration-enabled");
	chip->parallel_ovp_mode =
			of_property_read_bool(chip->spmi->dev.of_node,
					"qcom,parallel-ovp-mode");

#ifndef CONFIG_LGE_PM
	of_get_property(chip->spmi->dev.of_node, "qcom,thermal-mitigation",
		&(chip->thermal_levels));
#endif

#ifdef CONFIG_LGE_PM_THERMAL
	chip->chg_current_te = chip->max_bat_chg_current;
#endif

#ifndef CONFIG_LGE_PM
	if (chip->thermal_levels > sizeof(int)) {
		chip->thermal_mitigation = devm_kzalloc(chip->dev,
			chip->thermal_levels,
			GFP_KERNEL);

		if (chip->thermal_mitigation == NULL) {
			pr_err("thermal mitigation kzalloc() failed.\n");
			return -ENOMEM;
		}

		chip->thermal_levels /= sizeof(int);
		rc = of_property_read_u32_array(chip->spmi->dev.of_node,
				"qcom,thermal-mitigation",
				chip->thermal_mitigation, chip->thermal_levels);
		if (rc) {
			pr_err("qcom,thermal-mitigation missing in dt\n");
			return rc;
		}
	}
#endif

	return rc;
}
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
static void qpnp_monitor_batt_temp(struct work_struct *work)
{
	struct qpnp_chg_chip *chip =
		container_of(work, struct qpnp_chg_chip, battemp_work.work);
	struct charging_info req;
	struct charging_rsp res;
	union power_supply_propval ret = {0,};

	if (lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO && !is_enter_first) {
		pr_info("===== [qpnp_monitor_batt_temp] PM STAY AWAKE !!! ======\n");
		pm_stay_awake(chip->dev);
	}
	chip->batt_psy.get_property(&(chip->batt_psy),
			POWER_SUPPLY_PROP_TEMP, &ret);
	req.batt_temp = ret.intval;
#if defined (CONFIG_MACH_MSM8926_JAGC_SPR) || defined (CONFIG_MACH_MSM8926_JAGNM_ATT) || defined(CONFIG_MACH_MSM8926_JAGNM_RGS) \
	|| defined(CONFIG_MACH_MSM8926_JAGNM_TLS) || defined(CONFIG_MACH_MSM8926_JAGNM_VTR) || defined(CONFIG_MACH_MSM8926_JAGNM_BELL) || defined(CONFIG_MACH_MSM8926_JAGC_SPR)
	get_prop_batt_health(chip);
#endif
	chip->batt_psy.get_property(&(chip->batt_psy),
			  POWER_SUPPLY_PROP_VOLTAGE_NOW, &ret);
	req.batt_volt = ret.intval;

	req.is_charger = qpnp_chg_is_usb_chg_plugged_in(chip);

#ifdef CONFIG_LGE_PM_THERMAL
	req.chg_current_ma = chip->max_bat_chg_current;
	req.chg_current_te = chip->chg_current_te;
#endif
	/* Added this code to update state when charger is reconnected during wait delay. */
	if (chip->is_charger_changed_from_irq == true) {
		req.is_charger_changed = true;
		chip->is_charger_changed_from_irq = false;
	} else
		req.is_charger_changed = false;

	lge_monitor_batt_temp(req, &res);

	if (((res.change_lvl != STS_CHE_NONE) && req.is_charger) ||
		(res.force_update == true)) {
		if (res.change_lvl == STS_CHE_NORMAL_TO_DECCUR ||
			(res.force_update == true && res.state == CHG_BATT_DECCUR_STATE &&
#if defined(CONFIG_LGE_PM_VZW_CHARGING_TEMP_SCENARIO) || defined(CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO_V1_8)
			res.dc_current != DC_CURRENT_DEF && res.change_lvl != STS_CHE_STPCHG_TO_DECCUR)) {
#else
			res.dc_current != DC_CURRENT_DEF)) {
#endif
			qpnp_chg_ibatmax_set(chip, res.dc_current);
		} else if (res.change_lvl == STS_CHE_NORMAL_TO_STPCHG ||
			(res.force_update == true &&
			res.state == CHG_BATT_STPCHG_STATE)) {
			wake_lock(&chip->lcs_wake_lock);
#ifdef CONFIG_LGE_PM_VZW_FAST_CHG
			schedule_work(&chip->cancel_input_current_check_work);
#endif
			cancel_delayed_work_sync(&chip->eoc_work);
			pm_relax(chip->dev);	/* Bug fix. When stopping charging at high temperature, device does not enter suspend state after disconnecting charger. */
			qpnp_chg_charge_en(chip, !res.disable_chg);
		} else if (res.change_lvl == STS_CHE_DECCUR_TO_NORAML) {
#ifdef CONFIG_LGE_PM_THERMAL
			qpnp_chg_ibatmax_set(chip, res.dc_current);
#else
			qpnp_chg_ibatmax_set(chip, chip->max_bat_chg_current);
#endif
		} else if (res.change_lvl == STS_CHE_DECCUR_TO_STPCHG) {
			wake_lock(&chip->lcs_wake_lock);
			qpnp_chg_ibatmax_set(chip, chip->max_bat_chg_current);
			cancel_delayed_work_sync(&chip->eoc_work);
			pm_relax(chip->dev);	/* Bug fix. When stopping charging at high temperature, device does not enter suspend state after disconnecting charger. */
			qpnp_chg_charge_en(chip, !res.disable_chg);
		} else if (res.change_lvl == STS_CHE_STPCHG_TO_NORMAL) {
#ifdef CONFIG_LGE_PM_THERMAL
			qpnp_chg_ibatmax_set(chip, res.dc_current);
#endif
			schedule_delayed_work(&chip->eoc_work,
				msecs_to_jiffies(EOC_CHECK_PERIOD_MS));
			qpnp_chg_charge_en(chip, !res.disable_chg);
			wake_unlock(&chip->lcs_wake_lock);
		}
#if defined(CONFIG_LGE_PM_VZW_CHARGING_TEMP_SCENARIO) || defined(CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO_V1_8)
		else if (res.change_lvl == STS_CHE_STPCHG_TO_DECCUR) {
#ifdef CONFIG_LGE_PM_THERMAL
			qpnp_chg_ibatmax_set(chip, res.dc_current);
#endif
			schedule_delayed_work(&chip->eoc_work,
				msecs_to_jiffies(EOC_CHECK_PERIOD_MS));
			qpnp_chg_charge_en(chip, !res.disable_chg);
			wake_unlock(&chip->lcs_wake_lock);
		}
#endif
#ifdef CONFIG_LGE_PM_THERMAL
		else if (res.force_update == true && res.state == CHG_BATT_NORMAL_STATE &&
					res.dc_current != DC_CURRENT_DEF) {
			qpnp_chg_ibatmax_set(chip, res.dc_current);
			schedule_delayed_work(&chip->eoc_work,
				msecs_to_jiffies(EOC_CHECK_PERIOD_MS));
		}
#endif

		chip->pseudo_ui_chg = res.pseudo_chg_ui;
		chip->not_chg = res.state;
		chip->btm_state = res.btm_state;
	}

	power_supply_changed(&chip->batt_psy);

	if (req.is_charger) {
		if (res.state == CHG_BATT_NORMAL_STATE) {
			schedule_delayed_work(&chip->battemp_work,
				MONITOR_BATTEMP_POLLING_PERIOD);
		} else if (res.state == CHG_BATT_DECCUR_STATE || res.state == CHG_BATT_WARNIG_STATE) {
			schedule_delayed_work(&chip->battemp_work,
				MONITOR_BATTEMP_POLLING_PERIOD/3);
		} else if (res.state == CHG_BATT_STPCHG_STATE) {
			schedule_delayed_work(&chip->battemp_work,
				MONITOR_BATTEMP_POLLING_PERIOD/6);
		}
	} else {
		/* Bug fix. When resuming device at high temperature, device does not enter suspend state. */
		if (wake_lock_active(&chip->lcs_wake_lock))
			wake_unlock(&chip->lcs_wake_lock);
	}
}
#endif

#ifdef CONFIG_LGE_PM_PWR_KEY_FOR_CHG_LOGO
u32 lge_get_bl_level(void);

static bool key_filter_end = false;
static bool key_filter_start = false;
static int prev_key_val = 1;
static int prev_key_code = 0;

#define QPNP_PWR_KEY_MONITOR_PERIOD_MS 500
#define KEY_UP_EVENT    0

void
qpnp_goto_suspend_for_chg_logo(void)
{
    if(lge_get_boot_mode() == LGE_BOOT_MODE_CHARGERLOGO && !is_enter_first)
    {
        pr_info("===== [qpnp_goto_suspend_for_chg_logo] PM RELAX !!! ======\n");

		key_filter_start = false;
		key_filter_end = true;

		if (qpnp_chg != NULL) {
			pm_relax(qpnp_chg->dev);
		}
    }
}
EXPORT_SYMBOL(qpnp_goto_suspend_for_chg_logo);


static void
qpnp_pwr_key_filter_delay_for_chg_logo(struct work_struct *work)
{
	pr_info("=== Key filter timer expired (SoC status is %d ======\n", !is_enter_first);

    key_filter_start = false;
	key_filter_end = true;
}

void
qpnp_pwr_key_action_set_for_chg_logo(struct input_dev *dev, unsigned int code, int value)
{
	if (qpnp_chg == NULL) {
		return;
	} else {
		pm_stay_awake(qpnp_chg->dev);
	}

	if (value == 0 && code != prev_key_code) {
		/* We'll receive only one key in onetime. */
		pr_info("=== Other key up event is detected. ignore it~\n");
		return;
	}

	/*
	 * ignore the key down event if key down event is happened within 300ms after key down event
	 * 1. KEY DOWN: value is 1
	 * 2. KEY UP: value is 0
	 */
	if (prev_key_val == KEY_UP_EVENT && key_filter_start && !key_filter_end) {
		pr_info("=== Very fast key input. return it~!!! ======\n");
		return;
	}

	if (prev_key_val == KEY_UP_EVENT && value == KEY_UP_EVENT && !key_filter_start) {
		pr_info("=== Not matched key pair. return it~!!! ======\n");
		return;
	}

	prev_key_val = value;

	if (key_filter_start && !key_filter_end) {
		key_filter_start = false;
		key_filter_end = true;

		cancel_delayed_work_sync(&qpnp_chg->pwr_key_monitor_for_chg_logo);
		return;
	}

	pr_info("=== Key posting code is %d value is %d ======\n", code, value);
	prev_key_code = code;
	input_report_key(dev, code, value);
	input_sync(dev);

	if (value == KEY_UP_EVENT) {
		key_filter_start = true;
		key_filter_end = false;
		schedule_delayed_work(&qpnp_chg->pwr_key_monitor_for_chg_logo, msecs_to_jiffies(QPNP_PWR_KEY_MONITOR_PERIOD_MS));
	} else {
		key_filter_start = false;

		power_supply_changed(&qpnp_chg->batt_psy);
	}
}
EXPORT_SYMBOL(qpnp_pwr_key_action_set_for_chg_logo);
#endif

#if defined (CONFIG_MACH_MSM8926_JAGC_SPR) || defined (CONFIG_MACH_MSM8926_JAGNM_ATT) || defined (CONFIG_LGE_PM_CHARGING_DEBUG_LOG) \
	|| defined(CONFIG_MACH_MSM8926_JAGNM_RGS) || defined(CONFIG_MACH_MSM8926_JAGNM_TLS) || defined(CONFIG_MACH_MSM8926_JAGNM_VTR) || defined(CONFIG_MACH_MSM8926_JAGNM_BELL) || defined(CONFIG_MACH_MSM8926_JAGC_SPR)
#define SBMM_BAT_FET				0x0B
#define CHECK_STATUS_PERIOD        10000

static void devices_state_monitor_work (struct work_struct *work) {
	int rc = 0;		int ret = 0;
	int usb_in = 0; int usb_in_volt = 0;	int usb_in_volt_avg = 0;	int usb_in_volt_min_mv = 0;
	int batt_volt = 0;	int batt_soc = 0;		int iusb_max = 0;		int ibat_max = 0; 	int ibat_cur = 0;
	u8 chg_ctrl = 0;	u8 bat_fet_sts = 0;		u8 bat_if_sts = 0;		int pseudo_batt_check = 0;
	int batt_temp = 0;	u8 chg_sts = 0;		int cable_info = 0;
	char *charging_state = "Discharging";	int counter = 5;	char *check_chg_done = "Discharging";
	static int checked_usb_in;
#if  defined (CONFIG_MAX17048_FUELGAUGE)
	int fuel_soc = 0; int fuel_volt = 0;
#endif

	struct delayed_work		*dwork = to_delayed_work(work);
	struct qpnp_chg_chip		*chip = container_of(dwork, struct qpnp_chg_chip, charging_check_work);
	struct qpnp_vadc_result temp_results;
#if  defined (CONFIG_MAX17048_FUELGAUGE)
	struct qpnp_vadc_result v_results;
	union power_supply_propval val = {0,};
#endif
	usb_in = qpnp_chg_is_usb_chg_plugged_in(chip) || qpnp_chg_is_dc_chg_plugged_in(chip);
	usb_in_volt = qpnp_chg_get_vusbin_uv(chip) / 1000;

	if (checked_usb_in != usb_in && usb_in == 1)
		counter = 5;
	else
		counter = 1;

	usb_in_volt_avg =  get_vusb_averaged(chip, counter) / 1000;	/* Check usbin voltage */
	if (usb_in) {
		if (chip->chg_done)
			check_chg_done = "Charging_complete";
		else
			check_chg_done = "Not_complete";
	} else {
		check_chg_done = "Discharging";
	}
	checked_usb_in = usb_in;
#if  defined (CONFIG_MAX17048_FUELGAUGE)
	fuel_soc = max17048_get_capacity();
	fuel_volt = max17048_get_voltage();

	qpnp_vadc_read(chip->vadc_dev, VBAT_SNS, &v_results);
	batt_volt = (int)v_results.physical;

	if (chip->bms_psy) {
		chip->bms_psy->get_property(chip->bms_psy,
				POWER_SUPPLY_PROP_CAPACITY, &val);
		batt_soc = val.intval;
	}
#else
	batt_volt = get_prop_battery_voltage_now(chip)/1000;
	batt_soc = get_prop_capacity(chip);
#endif
	pseudo_batt_check = pseudo_batt_info.mode;
	iusb_max = qpnp_chg_usb_iusbmax_get(chip);
	ibat_cur = get_prop_current_now(chip) / 1000;
	usb_in_volt_min_mv = qpnp_chg_vinmin_get(chip);
	rc = qpnp_vadc_read(chip->vadc_dev, LR_MUX1_BATT_THERM,
			&temp_results);

	batt_temp = (int)temp_results.physical;

	cable_info = lge_pm_get_cable_type();

	rc = qpnp_chg_ibatmax_get(chip, &ibat_max);
	rc = qpnp_chg_read(chip, &chg_ctrl, chip->chgr_base + CHGR_CHG_CTRL, 1);
#ifdef CONFIG_LGE_PM
	bat_fet_sts = qpnp_chg_is_batfet_closed(chip);
#else
	rc = qpnp_chg_read(chip, &bat_fet_sts, chip->bat_if_base + SBMM_BAT_FET, 1);
#endif
	rc = qpnp_chg_read(chip, &chg_sts, INT_RT_STS(chip->chgr_base), 1);
	if (chg_sts == 0x20)
		charging_state = "CC_Charging";
	else if (chg_sts == 0x21)
		charging_state = "CV_Charging";
	else if (chg_sts == 0x0)
		if (usb_in == 0x1)
			charging_state = "Charging";
		else
			charging_state = "Discharging";
	else
		charging_state = "Discharging";

	printk("[STATUS] USB_IN : %d, USB_IN_VOL : %d, USB_IN_AVG. : %d, USB_IN_MIN. : %d, Cable_info : %d(%s), IUSB_MAX : %d\n", usb_in, usb_in_volt, usb_in_volt_avg, usb_in_volt_min_mv, cable_info, chip->ac_online ? "AC" : usb_in ? "USB" : "None",  iusb_max);
	printk("[STATUS] BATT_VOL : %d , BATT_SOC : %d , IBAT_MAX : %d , IBAT_NOW : %d BATT_TEMP : %d, pseudo_battery : %d\n", batt_volt, batt_soc, ibat_max, ibat_cur, batt_temp, pseudo_batt_check);
#if defined(CONFIG_MAX17048_FUELGAUGE)
	printk("[STATUS] FUEL_VOL : %d , FUEL_SOC : %d \n", fuel_volt, fuel_soc);
#endif

#ifdef CONFIG_LGE_PM
	printk("[STATUS] CHG_STS : %s, Charging_complete : %s, BAT_FET_STS : %s, BAT_IF_STS : 0x%x, CHG_TIMEOUT : %d\n", charging_state, check_chg_done, bat_fet_sts > 0 ? "Closed" : "Open", bat_if_sts, chip->chg_fail_irq_happen);
	/* printk("[STATUS] CHG_CTRL : 0x%02x, BAT_FET_STS : %s, CHG_STS : %s, BAT_IF_STS : 0x%x, CHG_TIMEOUT : %d, Charging_complete : %s\n", chg_ctrl, bat_fet_sts > 0 ? "Closed" : "Open", charging_state, bat_if_sts, chip->chg_fail_irq_happen, check_chg_done); */
#else
	printk("[STATUS] CHG_CTRL : 0x%x, BAT_FET_STS : 0x%x, CHG_STS : 0x%x, BAT_IF_STS : 0x%x\n", chg_ctrl, bat_fet_sts, chg_sts, bat_if_sts);
#endif

	ret = schedule_delayed_work(&chip->charging_check_work,
		round_jiffies_relative(msecs_to_jiffies(CHECK_STATUS_PERIOD)));
}

static void charging_check_probe(struct qpnp_chg_chip *chip)
{
	printk(KERN_INFO "[STATUS] Devices status\n");
	INIT_DELAYED_WORK(&chip->charging_check_work, devices_state_monitor_work);
	schedule_delayed_work(&chip->charging_check_work,
		round_jiffies_relative(msecs_to_jiffies(CHECK_STATUS_PERIOD)));
}
#endif

#ifdef CIDL
#define SBMM_BAT_FET				0x0B
#define CHARGING_INFORM_NORMAL_TIME		20000

static void charging_information(struct work_struct *work)
{
	int rc = 0;			int ret = 0;
	int usb_in = 0;		int qpnp_volt = 0;	int qpnp_soc = 0;	int iusb_max = 0;	int ibat_max = 0;
	int ibat = 0;		u8 chg_ctrl = 0;	u8 bat_fet_sts = 0;	u8 bat_if_sts = 0;
	int bat_temp = 0;	u8 chg_sts = 0;		int cable_info = 0;	int vin_min_mv = 0;
#if defined (CONFIG_MACH_MSM8226_E7WIFI) || defined (CONFIG_MACH_MSM8226_E8WIFI) || \
    defined (CONFIG_MACH_MSM8226_E9WIFI) || defined (CONFIG_MACH_MSM8226_E9WIFIN) || \
    defined (CONFIG_MACH_MSM8926_E9LTE)
	int pa_therm = 0; int vinput = 0;
#elif defined (CONFIG_MACH_MSM8926_E8LTE) || defined (CONFIG_MACH_MSM8926_T8LTE)
	int pa_therm0 = 0; int pa_therm1 = 0; int vinput = 0;
#endif
	struct delayed_work		*dwork = to_delayed_work(work);
	struct qpnp_chg_chip		*chip = container_of(dwork, struct qpnp_chg_chip, charging_inform_work);

	usb_in = qpnp_chg_is_usb_chg_plugged_in(chip);
	qpnp_volt = get_prop_battery_voltage_now(chip)/1000;
	qpnp_soc = get_prop_capacity(chip);
	iusb_max = qpnp_chg_usb_iusbmax_get(chip);
	rc = qpnp_chg_ibatmax_get(chip, &ibat_max);
	ibat = get_prop_current_now(chip)/1000;
	rc = qpnp_chg_read(chip, &chg_ctrl, chip->chgr_base + CHGR_CHG_CTRL, 1);
	rc = qpnp_chg_read(chip, &bat_fet_sts, chip->bat_if_base + SBMM_BAT_FET, 1);
	rc = qpnp_chg_read(chip, &chg_sts, INT_RT_STS(chip->chgr_base), 1);
	rc = qpnp_chg_read(chip, &bat_if_sts, INT_RT_STS(chip->bat_if_base), 1);
	bat_temp = get_prop_batt_temp(chip)/10;
	cable_info = lge_pm_get_cable_type();
	vin_min_mv = qpnp_chg_vinmin_get(chip);

#if defined (CONFIG_MACH_MSM8226_E7WIFI) || defined (CONFIG_MACH_MSM8226_E8WIFI) || \
	defined (CONFIG_MACH_MSM8226_E9WIFI) || defined (CONFIG_MACH_MSM8226_E9WIFIN) || defined (CONFIG_MACH_MSM8926_E9LTE)
	vinput = qpnp_chg_get_vusbin_uv(chip);
	pa_therm = qpnp_chg_get_pa_therm(chip);
/*
	pr_info("[%s] 0. USB plugged      = %d\n",__func,usb_in);
	pr_info("[%s] 1. Input Voltage     = %d\n",__func,vinput);
	pr_info("[%s] 2. Battery current   = %d\n",__func, ibat);
	pr_info("[%s] 3. Battery Voltage  = %d\n",__func,qpnp_volt);
	pr_info("[%s] 4. Current setting    = %d\n",__func,iusb_max);
	pr_info("[%s] 5. BATT_THERM      = %d\n",__func,bat_temp);
	pr_info("[%s] 6. PA_THERM           = %d\n",__func,pa_therm);
*/
	/*[Parsing] Can copy to Excel sheet*/
	pr_info("[C] USB_IRQ, Vinput, Ibat, SOC, Vbat, Iset, Batt_temp, PA_therm\n");
	pr_info("[I] %d, %d, %d, %d, %d, %d, %d, %d\n", \
		usb_in, vinput/1000, ibat, qpnp_soc, qpnp_volt, iusb_max, bat_temp, pa_therm);
#elif defined (CONFIG_MACH_MSM8926_E8LTE) || defined (CONFIG_MACH_MSM8926_T8LTE)
	vinput = qpnp_chg_get_vusbin_uv(chip);
	pa_therm0 = qpnp_chg_get_pa_therm0(chip);
	pa_therm1 = qpnp_chg_get_pa_therm1(chip);

	pr_info("[C] USB_IRQ, Vinput, Ibat, SOC, Vbat, Iset, Batt_temp, PA_therm0, PA_therm1\n");
	pr_info("[I] %d, %d, %d, %d, %d, %d, %d, %d, %d\n", \
		usb_in, vinput/1000, ibat, qpnp_soc, \
		qpnp_volt, iusb_max, bat_temp, pa_therm0, pa_therm1);
#else
	pr_info("[C], USB_IRQ, Qpnp_Volt, Qpnp_SOC, VINMIN_MV, IUSB_MAX, IBAT_MAX, IBAT_NOW, CHG_CTRL, BAT_FET_STS, CHG_STS, BAT_IF_STS, BAT_TEMP, Cable_info\n");
	pr_info("[I], %d , %d , %d , %d , %d , %d , %d , 0x%x , 0x%x , 0x%x , 0x%x , %d , %d(11)\n",\
		usb_in, qpnp_volt, qpnp_soc, vin_min_mv, iusb_max,\
		ibat_max, ibat, chg_ctrl, bat_fet_sts, chg_sts, bat_if_sts, bat_temp, cable_info);
#endif
#if defined (CONFIG_MACH_MSM8226_E7WIFI) || defined (CONFIG_MACH_MSM8226_E8WIFI) || \
    defined (CONFIG_MACH_MSM8926_E8LTE) || defined (CONFIG_MACH_MSM8226_E9WIFI) || \
    defined (CONFIG_MACH_MSM8226_E9WIFIN) || defined (CONFIG_MACH_MSM8926_E9LTE) || \
    defined (CONFIG_MACH_MSM8926_T8LTE)
	ret = schedule_delayed_work(&chip->charging_inform_work,
		round_jiffies_relative(msecs_to_jiffies(15000)));
#else
	ret = schedule_delayed_work(&chip->charging_inform_work,
		round_jiffies_relative(msecs_to_jiffies(CHARGING_INFORM_NORMAL_TIME)));
#endif
}

static void charging_information_probe(struct qpnp_chg_chip *chip)
{
	printk(KERN_INFO "[CHARGING_INFORMATION] Probe\n");
	INIT_DELAYED_WORK(&chip->charging_inform_work, charging_information);
	schedule_delayed_work(&chip->charging_inform_work,
		round_jiffies_relative(msecs_to_jiffies(CHARGING_INFORM_NORMAL_TIME)));
}
#endif

static int __devinit
qpnp_charger_probe(struct spmi_device *spmi)
{
	u8 subtype;
	struct qpnp_chg_chip	*chip;
	struct resource *resource;
	struct spmi_resource *spmi_resource;
	int rc = 0;
#ifdef CONFIG_LGE_PM_WORKAROUND_PORT_OPEN_FAIL_IN_FACTORY_TEST
    enum lge_boot_mode_type boot_mode = 0;
#endif

/* LGE_CHANGE_S: Cable Detect & Current Set */
#ifdef CONFIG_LGE_PM
	unsigned int *p_cable_type = (unsigned int *)
		(smem_get_entry(SMEM_ID_VENDOR1, &cable_smem_size));

	if (p_cable_type)
		cable_type = *p_cable_type;
	else
		cable_type = 0;

	pr_info("[LGE] cable_type is = %d\n", cable_type);
#endif
/* LGE_CHANGE_E */

	chip = devm_kzalloc(&spmi->dev,
			sizeof(struct qpnp_chg_chip), GFP_KERNEL);
	if (chip == NULL) {
		pr_err("kzalloc() failed.\n");
		return -ENOMEM;
	}

	chip->prev_usb_max_ma = -EINVAL;
	chip->fake_battery_soc = -EINVAL;
	chip->dev = &(spmi->dev);
	chip->spmi = spmi;

	chip->usb_psy = power_supply_get_by_name("usb");
	if (!chip->usb_psy) {
		pr_err("usb supply not found deferring probe\n");
		rc = -EPROBE_DEFER;
		goto fail_chg_enable;
	}
#ifdef CONFIG_LGE_PM_USB_ID
	rc = lge_pm_get_cable_data_from_dt(spmi->dev.of_node);
	if (rc)
		goto fail_chg_enable;
#endif

	mutex_init(&chip->jeita_configure_lock);
	spin_lock_init(&chip->usbin_health_monitor_lock);
	alarm_init(&chip->reduce_power_stage_alarm, ANDROID_ALARM_RTC_WAKEUP,
			qpnp_chg_reduce_power_stage_callback);
	INIT_WORK(&chip->reduce_power_stage_work,
			qpnp_chg_reduce_power_stage_work);
	mutex_init(&chip->batfet_vreg_lock);
	INIT_WORK(&chip->ocp_clear_work,
			qpnp_chg_ocp_clear_work);
	INIT_WORK(&chip->batfet_lcl_work,
			qpnp_chg_batfet_lcl_work);
	INIT_WORK(&chip->insertion_ocv_work,
			qpnp_chg_insertion_ocv_work);
#ifdef CONFIG_LGE_PM_4_25V_CHARGING_START
	INIT_WORK(&chip->batt_temp_cancel_work,
		    lge_pm_batt_temp_cancel_work);
#endif
	/* Get all device tree properties */
	rc = qpnp_charger_read_dt_props(chip);
	if (rc)
		return rc;

	if (ext_ovp_isns_present)
		chip->ext_ovp_ic_gpio_enabled = 0;

	/*
	 * Check if bat_if is set in DT and make sure VADC is present
	 * Also try loading the battery data profile if bat_if exists
	 */
	spmi_for_each_container_dev(spmi_resource, spmi) {
		if (!spmi_resource) {
			pr_err("qpnp_chg: spmi resource absent\n");
			rc = -ENXIO;
			goto fail_chg_enable;
		}

		resource = spmi_get_resource(spmi, spmi_resource,
						IORESOURCE_MEM, 0);
		if (!(resource && resource->start)) {
			pr_err("node %s IO resource absent!\n",
				spmi->dev.of_node->full_name);
			rc = -ENXIO;
			goto fail_chg_enable;
		}

		rc = qpnp_chg_read(chip, &subtype,
				resource->start + REG_OFFSET_PERP_SUBTYPE, 1);
		if (rc) {
			pr_err("Peripheral subtype read failed rc=%d\n", rc);
			goto fail_chg_enable;
		}

		if (subtype == SMBB_BAT_IF_SUBTYPE ||
			subtype == SMBBP_BAT_IF_SUBTYPE ||
			subtype == SMBCL_BAT_IF_SUBTYPE) {
			chip->vadc_dev = qpnp_get_vadc(chip->dev, "chg");
			if (IS_ERR(chip->vadc_dev)) {
				rc = PTR_ERR(chip->vadc_dev);
				if (rc != -EPROBE_DEFER)
					pr_err("vadc property missing\n");
				goto fail_chg_enable;
			}
#ifdef CONFIG_LGE_PM_USB_ID
		lge_pm_set_usb_id_handle(chip->vadc_dev);
#endif

			if (subtype == SMBB_BAT_IF_SUBTYPE) {
				chip->iadc_dev = qpnp_get_iadc(chip->dev,
						"chg");
				if (IS_ERR(chip->iadc_dev)) {
					rc = PTR_ERR(chip->iadc_dev);
					if (rc != -EPROBE_DEFER)
						pr_err("iadc property missing\n");
					goto fail_chg_enable;
				}
			}

			rc = qpnp_chg_load_battery_data(chip);
			if (rc)
				goto fail_chg_enable;
		}
	}

	spmi_for_each_container_dev(spmi_resource, spmi) {
		if (!spmi_resource) {
			pr_err("qpnp_chg: spmi resource absent\n");
			rc = -ENXIO;
			goto fail_chg_enable;
		}

		resource = spmi_get_resource(spmi, spmi_resource,
						IORESOURCE_MEM, 0);
		if (!(resource && resource->start)) {
			pr_err("node %s IO resource absent!\n",
				spmi->dev.of_node->full_name);
			rc = -ENXIO;
			goto fail_chg_enable;
		}

		rc = qpnp_chg_read(chip, &subtype,
				resource->start + REG_OFFSET_PERP_SUBTYPE, 1);
		if (rc) {
			pr_err("Peripheral subtype read failed rc=%d\n", rc);
			goto fail_chg_enable;
		}

		switch (subtype) {
		case SMBB_CHGR_SUBTYPE:
		case SMBBP_CHGR_SUBTYPE:
		case SMBCL_CHGR_SUBTYPE:
			chip->chgr_base = resource->start;
#ifdef CONFIG_LGE_USING_CHG_LED
			{
				hw_rev_type hw_rev;
				hw_rev = lge_get_board_revno();

				if (hw_rev == HW_REV_A) {
					chip->cdev.name = "red";
					chip->cdev.max_brightness = 0;
					chip->cdev.default_trigger = "none";
					chip->cdev.brightness_set    = qpnp_chg_led_set;
					chip->cdev.brightness_get    = qpnp_chg_led_get;
					rc = led_classdev_register(&spmi->dev, &chip->cdev);
					chip->pwm_cfg = devm_kzalloc(chip->dev,
								sizeof(struct pwm_config_data),	GFP_KERNEL);
					chip->pwm_cfg->pwm_channel = 2;
					chip->pwm_cfg->pwm_dev = pwm_request(chip->pwm_cfg->pwm_channel, "chg_led");
					chip->pwm_cfg->pwm_period_us = 20000;
					rc = sysfs_create_group(&chip->cdev.dev->kobj, &blink_attr_group);
				}
			}
#endif

			rc = qpnp_chg_hwinit(chip, subtype, spmi_resource);
			if (rc) {
				pr_err("Failed to init subtype 0x%x rc=%d\n",
						subtype, rc);
				goto fail_chg_enable;
			}
			break;
		case SMBB_BUCK_SUBTYPE:
		case SMBBP_BUCK_SUBTYPE:
		case SMBCL_BUCK_SUBTYPE:
			chip->buck_base = resource->start;
			rc = qpnp_chg_hwinit(chip, subtype, spmi_resource);
			if (rc) {
				pr_err("Failed to init subtype 0x%x rc=%d\n",
						subtype, rc);
				goto fail_chg_enable;
			}

			rc = qpnp_chg_masked_write(chip,
				chip->buck_base + SEC_ACCESS,
				0xFF,
				0xA5, 1);

			rc = qpnp_chg_masked_write(chip,
				chip->buck_base + BUCK_VCHG_OV,
				0xff,
				0x00, 1);

			if (chip->duty_cycle_100p) {
				rc = qpnp_buck_set_100_duty_cycle_enable(chip,
						1);
				if (rc) {
					pr_err("failed to set duty cycle %d\n",
						rc);
					goto fail_chg_enable;
				}
			}

			break;
		case SMBB_BAT_IF_SUBTYPE:
		case SMBBP_BAT_IF_SUBTYPE:
		case SMBCL_BAT_IF_SUBTYPE:
			chip->bat_if_base = resource->start;
			rc = qpnp_chg_hwinit(chip, subtype, spmi_resource);
			if (rc) {
				pr_err("Failed to init subtype 0x%x rc=%d\n",
						subtype, rc);
				goto fail_chg_enable;
			}
			break;
		case SMBB_USB_CHGPTH_SUBTYPE:
		case SMBBP_USB_CHGPTH_SUBTYPE:
		case SMBCL_USB_CHGPTH_SUBTYPE:
			chip->usb_chgpth_base = resource->start;
			rc = qpnp_chg_hwinit(chip, subtype, spmi_resource);
			if (rc) {
				if (rc != -EPROBE_DEFER)
					pr_err("Failed to init subtype 0x%x rc=%d\n",
						subtype, rc);
				goto fail_chg_enable;
			}
#if defined (CONFIG_MACH_MSM8926_JAGC_SPR) || defined (CONFIG_MACH_MSM8926_JAGNM_ATT) || defined (CONFIG_MACH_MSM8926_JAGNM_GLOBAL_COM) \
	|| defined(CONFIG_MACH_MSM8926_JAGNM_RGS) || defined(CONFIG_MACH_MSM8926_JAGNM_TLS) || defined(CONFIG_MACH_MSM8926_JAGNM_VTR) || defined(CONFIG_MACH_MSM8926_JAGNM_BELL) || defined(CONFIG_MACH_MSM8926_AKA_CN) || defined(CONFIG_MACH_MSM8926_JAGC_SPR) || defined (CONFIG_MACH_MSM8926_AKA_KR)
			gpio_request(31, "ext_ovp");
#elif defined (CONFIG_MACH_MSM8926_E2_MPCS_US)
			gpio_request(33, "ext_ovp");
#endif
			break;
		case SMBB_DC_CHGPTH_SUBTYPE:
			chip->dc_chgpth_base = resource->start;
			rc = qpnp_chg_hwinit(chip, subtype, spmi_resource);
			if (rc) {
				pr_err("Failed to init subtype 0x%x rc=%d\n",
						subtype, rc);
				goto fail_chg_enable;
			}
			break;
		case SMBB_BOOST_SUBTYPE:
		case SMBBP_BOOST_SUBTYPE:
			chip->boost_base = resource->start;
			rc = qpnp_chg_hwinit(chip, subtype, spmi_resource);
			if (rc) {
				if (rc != -EPROBE_DEFER)
					pr_err("Failed to init subtype 0x%x rc=%d\n",
						subtype, rc);
				goto fail_chg_enable;
			}
			break;
		case SMBB_MISC_SUBTYPE:
		case SMBBP_MISC_SUBTYPE:
		case SMBCL_MISC_SUBTYPE:
			chip->misc_base = resource->start;
			rc = qpnp_chg_hwinit(chip, subtype, spmi_resource);
			if (rc) {
				pr_err("Failed to init subtype=0x%x rc=%d\n",
						subtype, rc);
				goto fail_chg_enable;
			}
			break;
		default:
			pr_err("Invalid peripheral subtype=0x%x\n", subtype);
			rc = -EINVAL;
			goto fail_chg_enable;
		}
	}
	dev_set_drvdata(&spmi->dev, chip);
	device_init_wakeup(&spmi->dev, 1);

	chip->insertion_ocv_uv = -EINVAL;
	chip->batt_present = qpnp_chg_is_batt_present(chip);

/* BEGIN : janghyun.baek@lge.com 2013-01-25 Draw max current when factory cable inserted */
#ifdef CONFIG_LGE_PM
	if (is_factory_cable()) {
#if defined (CONFIG_MACH_MSM8226_E7WIFI) || defined (CONFIG_MACH_MSM8226_E8WIFI) || \
    defined (CONFIG_MACH_MSM8926_E8LTE) || defined (CONFIG_MACH_MSM8226_E9WIFI) || \
    defined (CONFIG_MACH_MSM8226_E9WIFIN) || defined (CONFIG_MACH_MSM8926_E9LTE) || \
    defined (CONFIG_MACH_MSM8926_T8LTE)
		if (get_prop_batt_present(chip) && is_56k_910k_factory_cable()) {
			 pr_info("Factory cable is connected (56K, 910K)\n");
			 qpnp_chg_iusbmax_set(chip, FACTORY_IUSB_MAX_FOR_EMBEDDED_BATTERY);
			 qpnp_chg_ibatmax_set(chip, FACTORY_IBAT_MAX_FOR_EMBEDDED_BATTERY);
		} else {
			 pr_info("Factory cable is connected (130K) IUSB MAX SETTING\n");
			 qpnp_chg_iusbmax_set(chip, QPNP_CHG_I_MAX_MAX_MA);
		}
#else
		pr_info("Factory cable is detected, set IUSB to MAX.\n");
		qpnp_chg_iusbmax_set(chip, QPNP_CHG_I_MAX_MAX_MA);
#endif
	}
#endif
/* END : janghyun.baek@lge.com 2013-01-25 */

	if (chip->bat_if_base) {
		chip->batt_psy.name = "battery";
		chip->batt_psy.type = POWER_SUPPLY_TYPE_BATTERY;
		chip->batt_psy.properties = msm_batt_power_props;
		chip->batt_psy.num_properties =
			ARRAY_SIZE(msm_batt_power_props);
		chip->batt_psy.get_property = qpnp_batt_power_get_property;
		chip->batt_psy.set_property = qpnp_batt_power_set_property;
		chip->batt_psy.property_is_writeable =
				qpnp_batt_property_is_writeable;
		chip->batt_psy.external_power_changed =
				qpnp_batt_external_power_changed;
		chip->batt_psy.supplied_to = pm_batt_supplied_to;
		chip->batt_psy.num_supplicants =
				ARRAY_SIZE(pm_batt_supplied_to);

		rc = power_supply_register(chip->dev, &chip->batt_psy);
		if (rc < 0) {
			pr_err("batt failed to register rc = %d\n", rc);
			goto fail_chg_enable;
		}
		INIT_WORK(&chip->adc_measure_work,
			qpnp_bat_if_adc_measure_work);
		INIT_WORK(&chip->adc_disable_work,
			qpnp_bat_if_adc_disable_work);
	}
#if defined (CONFIG_MACH_MSM8926_B2LN_KR) || defined (CONFIG_MACH_MSM8926_JAGN_KR) || defined (CONFIG_MACH_MSM8926_AKA_CN) || defined (CONFIG_MACH_MSM8926_AKA_KR)
	INIT_DELAYED_WORK(&chip->usbin_valid_work, qpnp_usbin_valid_work);
#endif
	INIT_DELAYED_WORK(&chip->eoc_work, qpnp_eoc_work);
	INIT_DELAYED_WORK(&chip->arb_stop_work, qpnp_arb_stop_work);
	INIT_DELAYED_WORK(&chip->usbin_health_check,
			qpnp_usbin_health_check_work);
#ifdef CONFIG_LGE_PM_WORKAROUND_USB_VALID_BY_REVERSE_BOOST
	INIT_DELAYED_WORK(&chip->property_work, qpnp_property_work);
#endif
#ifdef CONFIG_LGE_PM_PWR_KEY_FOR_CHG_LOGO
    INIT_DELAYED_WORK(&chip->pwr_key_monitor_for_chg_logo, qpnp_pwr_key_filter_delay_for_chg_logo);
#endif

#ifndef CONFIG_LGE_PM_4_25V_CHARGING_START
	INIT_WORK(&chip->soc_check_work, qpnp_chg_soc_check_work);
#endif
	INIT_DELAYED_WORK(&chip->aicl_check_work, qpnp_aicl_check_work);
#ifdef CONFIG_LGE_PM_4_25V_CHARGING_START
	INIT_WORK(&chip->resume_check_work, qpnp_chg_resume_check_work);
#endif
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
	chip->is_charger_changed_from_irq = false;
	wake_lock_init(&chip->lcs_wake_lock,
		WAKE_LOCK_SUSPEND, "LGE charging scenario");
	INIT_DELAYED_WORK(&chip->battemp_work, qpnp_monitor_batt_temp);
#endif

#ifdef CONFIG_LGE_PM
	wake_lock_init(&chip->uevent_wake_lock, WAKE_LOCK_SUSPEND, "qpnp_chg_uevent");
#endif

#ifdef CONFIG_LGE_PM_WORKAROUND_WEAK_CHARGER_REMOVAL_DETECTION
	wake_lock_init(&chip->weak_chg_wake_lock, WAKE_LOCK_SUSPEND, "qpnp_weak_chg_workaround");
#endif

#ifdef CONFIG_LGE_PM_VZW_FAST_CHG
	INIT_DELAYED_WORK(&chip->input_current_check_work, vzw_fast_chg_input_current_check_work);
	INIT_WORK(&chip->cancel_input_current_check_work, vzw_fast_chg_cancel_input_current_check_work);
#endif
#ifdef CONFIG_LGE_WIRELESS_CHARGER_RT9536
	INIT_WORK(&chip->wlc_enable_work, wireless_charging_enable_work);
	INIT_WORK(&chip->wlc_disable_work, wireless_charging_disable_work);
#endif
#ifdef CONFIG_LGE_PM_VZW_LLK
	INIT_DELAYED_WORK(&chip->vzw_llk_stop_chg_work, vzw_llk_stop_chg);
#endif
    /* LGE_CHANGE_S: */
    /* LGE do not use DC Charger, DC -> AC(TA) change */
#ifdef CONFIG_LGE_PM
		chip->dc_psy.name = "ac";
		chip->dc_psy.type = POWER_SUPPLY_TYPE_MAINS;
		chip->dc_psy.supplied_to = pm_power_supplied_to;
		chip->dc_psy.num_supplicants = ARRAY_SIZE(pm_power_supplied_to);
		chip->dc_psy.properties = pm_power_props_mains;
		chip->dc_psy.num_properties = ARRAY_SIZE(pm_power_props_mains);
		chip->dc_psy.set_property = qpnp_power_set_property_mains;
		chip->dc_psy.get_property = qpnp_power_get_property_mains;
		chip->dc_psy.property_is_writeable =
				qpnp_power_property_is_writeable;
		rc = power_supply_register(chip->dev, &chip->dc_psy);
		if (rc < 0) {
			pr_err("power_supply_register dc failed rc=%d\n", rc);
			goto unregister_batt;
		}
#else	/* Qualcomm Original */
	if (chip->dc_chgpth_base) {
		chip->dc_psy.name = "qpnp-dc";
		chip->dc_psy.type = POWER_SUPPLY_TYPE_MAINS;
		chip->dc_psy.supplied_to = pm_power_supplied_to;
		chip->dc_psy.num_supplicants = ARRAY_SIZE(pm_power_supplied_to);
		chip->dc_psy.properties = pm_power_props_mains;
		chip->dc_psy.num_properties = ARRAY_SIZE(pm_power_props_mains);
		chip->dc_psy.get_property = qpnp_power_get_property_mains;
		chip->dc_psy.set_property = qpnp_dc_power_set_property;
		chip->dc_psy.property_is_writeable =
				qpnp_dc_property_is_writeable;

		rc = power_supply_register(chip->dev, &chip->dc_psy);
		if (rc < 0) {
			pr_err("power_supply_register dc failed rc=%d\n", rc);
			goto unregister_batt;
		}
	}
#endif
	/* LGE_CHANGE_E */

	/* Turn on appropriate workaround flags */
	rc = qpnp_chg_setup_flags(chip);
	if (rc < 0) {
		pr_err("failed to setup flags rc=%d\n", rc);
		goto unregister_dc_psy;
	}

	if (chip->maxinput_dc_ma && chip->dc_chgpth_base) {
		rc = qpnp_chg_idcmax_set(chip, chip->maxinput_dc_ma);
		if (rc) {
			pr_err("Error setting idcmax property %d\n", rc);
			goto unregister_dc_psy;
		}
	}

	if ((chip->cool_bat_decidegc || chip->warm_bat_decidegc)
							&& chip->bat_if_base) {
		chip->adc_param.low_temp = chip->cool_bat_decidegc;
		chip->adc_param.high_temp = chip->warm_bat_decidegc;
		chip->adc_param.timer_interval = ADC_MEAS2_INTERVAL_1S;
		chip->adc_param.state_request = ADC_TM_HIGH_LOW_THR_ENABLE;
		chip->adc_param.btm_ctx = chip;
		chip->adc_param.threshold_notification =
						qpnp_chg_adc_notification;
		chip->adc_param.channel = LR_MUX1_BATT_THERM;

		if (get_prop_batt_present(chip)) {
			rc = qpnp_adc_tm_channel_measure(chip->adc_tm_dev,
							&chip->adc_param);
			if (rc) {
				pr_err("request ADC error %d\n", rc);
				goto unregister_dc_psy;
			}
		}
	}
	rc = qpnp_chg_bat_if_configure_btc(chip);
	if (rc) {
		pr_err("failed to configure btc %d\n", rc);
		goto unregister_dc_psy;
	}

	chip->usb_trim_default = qpnp_chg_iusb_trim_get(chip);
	qpnp_chg_charge_en(chip, !chip->charging_disabled);
	qpnp_chg_force_run_on_batt(chip, chip->charging_disabled);
	qpnp_chg_set_appropriate_vddmax(chip);

	if (chip->parallel_ovp_mode) {
		rc = override_dcin_ilimit(chip, 1);
		if (rc) {
			pr_err("Override DCIN LLIMIT %d\n", rc);
			goto unregister_dc_psy;
		}
	}

#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
	schedule_delayed_work(&chip->battemp_work,
							MONITOR_BATTEMP_POLLING_PERIOD / 12);
#endif

#ifdef CONFIG_LGE_PM
	chip->chg_fail_irq_happen = false;
	qpnp_chg = chip;
#endif
	rc = qpnp_chg_request_irqs(chip);
	if (rc) {
		pr_err("failed to request interrupts %d\n", rc);
		goto unregister_dc_psy;
	}

	qpnp_chg_usb_chg_gone_irq_handler(chip->chg_gone.irq, chip);
	qpnp_chg_usb_usbin_valid_irq_handler(chip->usbin_valid.irq, chip);
	qpnp_chg_dc_dcin_valid_irq_handler(chip->dcin_valid.irq, chip);
	power_supply_set_present(chip->usb_psy,
			qpnp_chg_is_usb_chg_plugged_in(chip));

/* LGE_CHANGE_S : It is fixed for detecting AC and USB problem after connecting TA cable and boot-up. */
#ifndef CONFIG_LGE_PM
	/* LGE does not use this code because it is not matched with LGE scenario.
	 * Qualcomm confirmed that it is possible to remove this code through SR 01217561. */
	/* Set USB psy online to avoid userspace from shutting down if battery
	 * capacity is at zero and no chargers online. */
	if (qpnp_chg_is_usb_chg_plugged_in(chip))
		power_supply_set_online(chip->usb_psy, 1);
#endif
/* LGE_CHANGE_E */

#ifdef CONFIG_LGE_PM_WORKAROUND_PORT_OPEN_FAIL_IN_FACTORY_TEST
		boot_mode = lge_get_boot_mode();
		printk("[qpnp-charger OVD] start qpnp_chg_masked_write boot_mode = %d\n", boot_mode);
		if ((boot_mode == LGE_BOOT_MODE_QEM_56K) ||
			(boot_mode == LGE_BOOT_MODE_QEM_130K) ||
			(boot_mode == LGE_BOOT_MODE_QEM_910K)) {
			rc = qpnp_chg_masked_write(chip,
					chip->usb_chgpth_base + USB_OVP_CTL,
					0x30, 0x30, 1);
			printk("[qpnp-charger OVD] OVD = 7V, address = 0x1342, value = 0x30, rc = %d!!\n", rc);
		} else {
			rc = qpnp_chg_masked_write(chip,
					chip->usb_chgpth_base + USB_OVP_CTL,
#ifdef CONFIG_LGE_PM_4_25V_CHARGING_START
					0x30, 0x30, 1);
			printk("[qpnp-charger OVD] OVD = 7V, address = 0x1342, value = 0x30, rc = %d!!\n", rc);
#else
					0x30, 0x20, 1);
			printk("[qpnp-charger OVD] OVD = 6.5V, address = 0x1342, value = 0x20, rc = %d!!\n", rc);
#endif
		}
#endif

#if defined (CONFIG_MACH_MSM8926_JAGC_SPR) || defined (CONFIG_MACH_MSM8926_JAGNM_ATT) || defined (CONFIG_LGE_PM_CHARGING_DEBUG_LOG) \
	|| defined(CONFIG_MACH_MSM8926_JAGNM_RGS) || defined(CONFIG_MACH_MSM8926_JAGNM_TLS) || defined(CONFIG_MACH_MSM8926_JAGNM_VTR) || defined(CONFIG_MACH_MSM8926_JAGNM_BELL) || defined(CONFIG_MACH_MSM8926_JAGC_SPR)
		charging_check_probe(chip);
#endif

#ifdef CIDL
		charging_information_probe(chip);
#endif
#if defined (CONFIG_MACH_MSM8226_E7WIFI) || defined (CONFIG_MACH_MSM8226_E8WIFI) || \
    defined (CONFIG_MACH_MSM8926_E8LTE)|| defined (CONFIG_MACH_MSM8226_E9WIFI) || \
    defined (CONFIG_MACH_MSM8226_E9WIFIN) || defined (CONFIG_MACH_MSM8926_E9LTE) || \
    defined (CONFIG_MACH_MSM8926_T8LTE)
	if(is_56k_910k_factory_cable()){
		pr_info("DEBUG : 56K, 910K factory cable is inserted, not check input current\n");
	} else {
		schedule_delayed_work(&chip->aicl_check_work,
			msecs_to_jiffies(EOC_CHECK_PERIOD_MS));
	}
#else
	schedule_delayed_work(&chip->aicl_check_work,
		msecs_to_jiffies(EOC_CHECK_PERIOD_MS));
#endif
#ifdef CONFIG_LGE_PM_DISABLE_ULPM_MODE
	qpnp_chg_regulator_batfet_set(chip, 1);
#endif
	pr_info("success chg_dis = %d, bpd = %d, usb = %d, dc = %d b_health = %d batt_present = %d\n",
			chip->charging_disabled,
			chip->bpd_detection,
			qpnp_chg_is_usb_chg_plugged_in(chip),
			qpnp_chg_is_dc_chg_plugged_in(chip),
			get_prop_batt_present(chip),
			get_prop_batt_health(chip));
        dummy_chip = chip;
	batt_present_touch = get_batt_present_touch();
	return 0;

unregister_dc_psy:
	if (chip->dc_chgpth_base)
		power_supply_unregister(&chip->dc_psy);
unregister_batt:
	if (chip->bat_if_base)
		power_supply_unregister(&chip->batt_psy);
fail_chg_enable:
	regulator_unregister(chip->otg_vreg.rdev);
	regulator_unregister(chip->boost_vreg.rdev);
	return rc;
}

static int __devexit
qpnp_charger_remove(struct spmi_device *spmi)
{
	struct qpnp_chg_chip *chip = dev_get_drvdata(&spmi->dev);
	if ((chip->cool_bat_decidegc || chip->warm_bat_decidegc)
						&& chip->batt_present) {
		qpnp_adc_tm_disable_chan_meas(chip->adc_tm_dev,
							&chip->adc_param);
	}

	cancel_delayed_work_sync(&chip->aicl_check_work);
	power_supply_unregister(&chip->dc_psy);
#ifndef CONFIG_LGE_PM_4_25V_CHARGING_START
	cancel_work_sync(&chip->soc_check_work);
#endif

#ifdef CONFIG_LGE_PM_4_25V_CHARGING_START
	cancel_work_sync(&chip->resume_check_work);
#endif
	cancel_delayed_work_sync(&chip->usbin_health_check);
	cancel_delayed_work_sync(&chip->arb_stop_work);
	cancel_delayed_work_sync(&chip->eoc_work);
	cancel_work_sync(&chip->adc_disable_work);
	cancel_work_sync(&chip->adc_measure_work);
	power_supply_unregister(&chip->batt_psy);
	cancel_work_sync(&chip->batfet_lcl_work);
	cancel_work_sync(&chip->insertion_ocv_work);
	cancel_work_sync(&chip->reduce_power_stage_work);
	alarm_cancel(&chip->reduce_power_stage_alarm);
#if defined (CONFIG_MACH_MSM8926_B2LN_KR) || defined (CONFIG_MACH_MSM8926_JAGN_KR) || defined (CONFIG_MACH_MSM8926_AKA_CN)
	cancel_delayed_work_sync(&chip->usbin_valid_work);
#endif
	mutex_destroy(&chip->batfet_vreg_lock);
	mutex_destroy(&chip->jeita_configure_lock);

	regulator_unregister(chip->otg_vreg.rdev);
	regulator_unregister(chip->boost_vreg.rdev);
#ifdef CIDL
	cancel_delayed_work_sync(&chip->charging_inform_work);
#endif
#ifdef CONFIG_LGE_WIRELESS_CHARGER_RT9536
	cancel_work_sync(&chip->wlc_enable_work);
	cancel_work_sync(&chip->wlc_disable_work);
#endif

	return 0;
}

static int qpnp_chg_resume(struct device *dev)
{
	struct qpnp_chg_chip *chip = dev_get_drvdata(dev);
	int rc = 0;
	pr_info("DEBUG_____chg_resume\n");
	if (chip->bat_if_base) {
		rc = qpnp_chg_masked_write(chip,
			chip->bat_if_base + BAT_IF_VREF_BAT_THM_CTRL,
			VREF_BATT_THERM_FORCE_ON,
			VREF_BATT_THERM_FORCE_ON, 1);
		if (rc)
			pr_debug("failed to force on VREF_BAT_THM rc=%d\n", rc);
	}
#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
	schedule_delayed_work(&chip->battemp_work, HZ*10);
#endif
#ifdef CIDL
	schedule_delayed_work(&chip->charging_inform_work, HZ*20);
#endif

#if defined (CONFIG_MACH_MSM8926_JAGC_SPR) || defined (CONFIG_MACH_MSM8926_JAGNM_ATT) || defined (CONFIG_LGE_PM_CHARGING_DEBUG_LOG) \
	|| defined(CONFIG_MACH_MSM8926_JAGNM_RGS) || defined(CONFIG_MACH_MSM8926_JAGNM_TLS) || defined(CONFIG_MACH_MSM8926_JAGNM_VTR) || defined(CONFIG_MACH_MSM8926_JAGNM_BELL) || defined(CONFIG_MACH_MSM8926_JAGC_SPR)
	schedule_delayed_work(&chip->charging_check_work, HZ*20);
#endif

	return rc;
}

static int qpnp_chg_suspend(struct device *dev)
{
	struct qpnp_chg_chip *chip = dev_get_drvdata(dev);
	int rc = 0;

	if (chip->bat_if_base) {
		rc = qpnp_chg_masked_write(chip,
			chip->bat_if_base + BAT_IF_VREF_BAT_THM_CTRL,
			VREF_BATT_THERM_FORCE_ON,
			VREF_BAT_THM_ENABLED_FSM, 1);
		if (rc)
			pr_debug("failed to set FSM VREF_BAT_THM rc=%d\n", rc);
	}

#if defined (CONFIG_MACH_MSM8226_E7WIFI) || defined (CONFIG_MACH_MSM8226_E8WIFI) || \
    defined (CONFIG_MACH_MSM8926_E8LTE)|| defined (CONFIG_MACH_MSM8226_E9WIFI) || \
    defined (CONFIG_MACH_MSM8226_E9WIFIN) || defined (CONFIG_MACH_MSM8926_E9LTE) || \
    defined (CONFIG_MACH_MSM8926_T8LTE)
	if(chip->ac_online && chip->chg_done){
		pr_info("DEBUG: EOC state, ChargerSuspend\n");
	}
#endif

#ifdef CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO
	cancel_delayed_work_sync(&chip->battemp_work);
#endif
#ifdef CIDL
	cancel_delayed_work_sync(&chip->charging_inform_work);
#endif
#if defined (CONFIG_MACH_MSM8926_B2LN_KR) || defined (CONFIG_MACH_MSM8926_JAGN_KR) || defined (CONFIG_MACH_MSM8926_AKA_CN)
	cancel_delayed_work_sync(&chip->usbin_valid_work);
#endif
#if defined (CONFIG_MACH_MSM8926_JAGC_SPR) || defined (CONFIG_MACH_MSM8926_JAGNM_ATT) || defined (CONFIG_LGE_PM_CHARGING_DEBUG_LOG) \
	|| defined(CONFIG_MACH_MSM8926_JAGNM_RGS) || defined(CONFIG_MACH_MSM8926_JAGNM_TLS) || defined(CONFIG_MACH_MSM8926_JAGNM_VTR) || defined(CONFIG_MACH_MSM8926_JAGNM_BELL) || defined(CONFIG_MACH_MSM8926_JAGC_SPR)
	cancel_delayed_work_sync(&chip->charging_check_work);
#endif

	return rc;
}

#ifdef CONFIG_LGE_PM_CHARGING_EXTERNAL_OVP
bool lge_check_fast_chg_irq(void)
{
    int rc;
    u8 chgr_sts;

	rc = qpnp_chg_read(qpnp_chg, &chgr_sts, INT_RT_STS(qpnp_chg->chgr_base), 1);
	if (rc) {
		pr_err("failed to read interrupt sts %d\n", rc);
		return false;
	}

	if (chgr_sts & FAST_CHG_ON_IRQ)
		return true;

	return false;
}
EXPORT_SYMBOL(lge_check_fast_chg_irq);

void lge_set_chg_path_to_external(void)
{
	lge_pm_set_ext_ovp(qpnp_chg->ext_ovp_gpio, EXT_OVP_CTRL_HIGH);
}
EXPORT_SYMBOL(lge_set_chg_path_to_external);

void lge_set_chg_path_to_internal(void)
{
	lge_pm_set_ext_ovp(qpnp_chg->ext_ovp_gpio, EXT_OVP_CTRL_LOW);
}
EXPORT_SYMBOL(lge_set_chg_path_to_internal);
#endif

static const struct dev_pm_ops qpnp_chg_pm_ops = {
	.resume		= qpnp_chg_resume,
	.suspend	= qpnp_chg_suspend,
};

static struct spmi_driver qpnp_charger_driver = {
	.probe		= qpnp_charger_probe,
	.remove		= __devexit_p(qpnp_charger_remove),
	.driver		= {
		.name		= QPNP_CHARGER_DEV_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= qpnp_charger_match_table,
		.pm		= &qpnp_chg_pm_ops,
	},
};

/**
 * qpnp_chg_init() - register spmi driver for qpnp-chg
 */
int __init
qpnp_chg_init(void)
{
	return spmi_driver_register(&qpnp_charger_driver);
}
module_init(qpnp_chg_init);

static void __exit
qpnp_chg_exit(void)
{
	spmi_driver_unregister(&qpnp_charger_driver);
}
module_exit(qpnp_chg_exit);


MODULE_DESCRIPTION("QPNP charger driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" QPNP_CHARGER_DEV_NAME);
