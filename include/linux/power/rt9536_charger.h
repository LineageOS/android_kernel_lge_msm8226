/*
 * Copyright LG Electronics (c) 2013
 * All rights reserved.
 */


#ifndef RT9536_CHARGER_H
#define RT9536_CHARGER_H

#include <linux/types.h>
#include <linux/power_supply.h>


struct rt9536_platform_data {
	unsigned int wlc_irq_pin;
	unsigned int wlc_irq;
	unsigned long irq_flags;
	unsigned int wlc_en_set_pin;
	unsigned int wlc_chg_full_pin;
	unsigned int wlc_enable;
};

struct rt9536_chip {
	struct power_supply charger;
	struct power_supply *batt_psy;
	struct rt9536_platform_data *pdata;
	int irq;
	int chg_online;
	int chg_done;
#if 0 //def CONFIG_LGE_WIRELESS_CHARGER_RT9536
	struct msm_hardware_charger	adapter_hw_chg;
#endif

	struct wake_lock wl;
	spinlock_t enable_lock;
	struct work_struct wireless_interrupt_work;
	struct work_struct wireless_set_online_work;
	struct work_struct wireless_set_offline_work;
	struct work_struct wireless_eoc_work;
	struct work_struct wireless_wakelock_work;

	u8  tmr_rst;		/* watchdog timer reset */
	u8  supply_sel;		/* supply selection */

	u8	reset;			/* reset all reg to default values */
	u8	iusblimit;		/* usb current limit */
	u8 	enstat;			/* enable STAT */
	u8	te;				/* enable charger termination */
	u8 	ce;				/* charger enable : 0 disable : 1 */
	u8	hz_mode;		/* high impedance mode */

	u8 	vbatt_reg;		/* battery regulation voltage */
	u8	inlimit_in;		/* input limit for IN input */
	u8	dpdm_en;		/* D+/D- detention */

	u8	chgcrnt;		/* charge current */
	u8	termcrnt;		/* termination current sense */

	u8	minsys_stat;	/* minimum system voltage mode */
	u8	dpm_stat;		/* Vin-DPM mode */
	u8	vindpm_usb;		/* usb input Vin-dpm voltage */
	u8	vindpm_in;		/* IN input Vin-dpm voltage */

	u8	tmr2x_en;		/* timer slowed by 2x */
	u8	safety_tmr;		/* safety timer */
	u8	ts_en;			/* ts function enable */
	u8	ts_fault;		/* ts fault mode */
	
	int 	valid_n_gpio;
};

#endif /* RT9536_CHARGER_H */
