obj-$(CONFIG_MACH_LGE) += devices_lge.o

obj-$(CONFIG_MACH_MSM8226_W7DS_OPEN_CIS) += board-8226-w7ds-open-cis.o board-8226-w7ds-open-cis-gpiomux.o

obj-$(CONFIG_LGE_BOOTLOADER_LOG) += lge_bootloader_log.o
obj-$(CONFIG_LGE_HANDLE_PANIC) += lge_handle_panic.o
obj-$(CONFIG_LGE_BOOTLOADER_TIME_CHECKER) += lge_bootloader_time_checker.o
ifdef CONFIG_MACH_MSM8926_X5_SPR
obj-$(CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO) += lge_charging_scenario_sprint.o
else
obj-$(CONFIG_LGE_PM_CHARGING_TEMP_SCENARIO) += lge_charging_scenario.o
endif
obj-$(CONFIG_MACH_LGE) += lge_kcal_ctrl.o
#[WiFi]_S[sangheon.shim@lge.com] BRCM WLAN
obj-$(CONFIG_WIFI_CONTROL_FUNC) += wifi_bcm_lge.o
obj-$(CONFIG_LGE_QFPROM_INTERFACE) += lge_qfprom_access.o
obj-$(CONFIG_LGE_ENABLE_MMC_STRENGTH_CONTROL) += lge_mmc_strength_control.o

ifneq ($(CONFIG_MACH_MSM8926_B1L_VZW),)
EXTRA_CFLAGS += -DCONFIG_MACH_MSM8926_B1L_VZW
endif
#[Power] Battery Data 
ifdef CONFIG_LGE_PM_BATTERY_PROFILE_DATA
	ifdef CONFIG_LGE_PM_BATTERY_ID_CHECKER
#This is for default setting / remove later
		obj-$(CONFIG_MACH_LGE) += bms-batterydata-LGE_BL_54SH_2540mAh_LG_Chem_Final.o 
		obj-$(CONFIG_LGE_PM_BATTERY_CAPACITY_1540mAh) += bms-batterydata-LGC_BL44JR_1540.o
		obj-$(CONFIG_LGE_PM_BATTERY_CAPACITY_1700mAh) += bms-batterydata-LGC_BL44JH_1700.o bms-batterydata-LGC_BL44JH_1700_Dallas.o bms-batterydata-Tocad_BL44JH_1700_Intersil.o
		obj-$(CONFIG_LGE_PM_BATTERY_CAPACITY_2100mAh) += bms-batterydata-LGE_LGC_2040mAh_Final.o bms-batterydata-LGE_Tocad_2040mAh_Final.o
		obj-$(CONFIG_LGE_PM_BATTERY_CAPACITY_2440mAh) += bms-batterydata-LGE_BL_59UH_2440mAh_LG_Chem_Final.o bms-batterydata-LGE_BL_59UH_2440mAh_TOCAD_Final.o
		obj-$(CONFIG_LGE_PM_BATTERY_CAPACITY_2540mAh) += bms-batterydata-LGE_BL_54SH_2540mAh_LG_Chem_Final.o bms-batterydata-LGE_BL_54SH_2540mAh_Sanyo_Final.o
		obj-$(CONFIG_LGE_PM_BATTERY_CAPACITY_3000mAh) += bms-batterydata-LGE_BL_64SH_3000mAh_LG_Chem_Final.o bms-batterydata-LGE_BL_64SH_3000mAh_Technohill_Final.o
	else
		obj-$(CONFIG_MACH_LGE) += bms-batterydata-LGE_BL_54SH_2540mAh_LG_Chem_Final.o
	endif
endif

CFLAGS_devices_lge.o += -Idrivers/staging/android
