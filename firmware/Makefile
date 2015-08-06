#
# kbuild file for firmware/
#

# Create $(fwabs) from $(CONFIG_EXTRA_FIRMWARE_DIR) -- if it doesn't have a
# leading /, it's relative to $(srctree).
fwdir := $(subst ",,$(CONFIG_EXTRA_FIRMWARE_DIR))
fwabs := $(addprefix $(srctree)/,$(filter-out /%,$(fwdir)))$(filter /%,$(fwdir))

fw-external-y := $(subst ",,$(CONFIG_EXTRA_FIRMWARE))

# There are three cases to care about:
# 1. Building kernel with CONFIG_FIRMWARE_IN_KERNEL=y -- $(fw-shipped-y) should
#    include the firmware files to include, according to .config
# 2. 'make modules_install', which will install firmware for modules, and 
#    _also_ for the in-kernel drivers when CONFIG_FIRMWARE_IN_KERNEL=n
# 3. 'make firmware_install', which installs all firmware, unconditionally.

# For the former two cases we want $(fw-shipped-y) and $(fw-shipped-m) to be
# accurate. In the latter case it doesn't matter -- it'll use $(fw-shipped-all).
# But be aware that the config file might not be included at all.

ifdef CONFIG_ACENIC_OMIT_TIGON_I
acenic-objs := acenic/tg2.bin
fw-shipped- += acenic/tg1.bin
else
acenic-objs := acenic/tg1.bin acenic/tg2.bin
endif
# LGE_CHANGE
# add synaptics touch firmware to be updated
# this firmware will be used on synaptics_fw_update.c and this name matching with defined on touch dtsi file
# 2013-04-29 kenneth.kang@lge.com
ifdef CONFIG_MACH_MSM8X26_LGPS14
fw-shipped-$(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4) += synaptics/PR1237618-DS4.3.5.0.16.img
endif

touch_limit_txt := $(wildcard $(srctree)/firmware/melfas/$(TARGET_PRODUCT)/*.txt)
$(shell objcopy -I binary -O ihex $(touch_limit_txt) $(touch_limit_txt).ihex)

ifdef CONFIG_MACH_MSM8926_VFP_KR
touch_fw_mfsb := $(wildcard $(srctree)/firmware/melfas/$(TARGET_PRODUCT)/*.mfsb)
$(shell objcopy -I binary -O ihex $(touch_fw_mfsb) $(touch_fw_mfsb).ihex)
endif

touch_fw_list := $(subst .ihex, , $(notdir $(wildcard $(srctree)/firmware/melfas/$(TARGET_PRODUCT)/*.ihex)))
touch_fw_files := $(foreach var, $(touch_fw_list), melfas/$(TARGET_PRODUCT)/$(var))

# LGE_CHANGE - START
# KR only for same DTS usage,  2013-11-28 kyle.jeon@lge.com
ifdef TOUCH_FIRMWARE_KOR_DIR
touch_limit_txt := $(wildcard $(srctree)/firmware/melfas/$(TOUCH_FIRMWARE_KOR_DIR)/*.txt)
$(shell objcopy -I binary -O ihex $(touch_limit_txt) $(touch_limit_txt).ihex)
touch_fw_list := $(subst .ihex, , $(notdir $(wildcard $(srctree)/firmware/melfas/$(TOUCH_FIRMWARE_KOR_DIR)/*.ihex)))
touch_fw_files := $(foreach var, $(touch_fw_list), melfas/$(TOUCH_FIRMWARE_KOR_DIR)/$(var))
endif
# LGE_CHANGE - END

# LGE_CHANGE - START
# Jaguar SPR/ATT classify Atmel Firmware files for Kernel Memory.
# 2014-03-13 donghyun.yoon@lge.com
# LGE_CHANGE - END

ifdef CONFIG_MACH_MSM8926_JAGNM_ATT
atmel_fw_file := $(wildcard $(srctree)/firmware/atmel/$(TARGET_PRODUCT)/*.fw)
$(foreach var, $(atmel_fw_file),  $(shell objcopy -I binary -O ihex $(var) $(var).ihex))
atmel_ihex_list := $(subst .ihex, , $(notdir $(wildcard $(srctree)/firmware/atmel/$(TARGET_PRODUCT)/*.ihex)))
atmel_ihex_files := $(foreach var, $(atmel_ihex_list), atmel/$(TARGET_PRODUCT)/$(var))
else 
ifdef CONFIG_MACH_MSM8926_JAGNM_BELL
atmel_fw_file := $(wildcard $(srctree)/firmware/atmel/$(TARGET_PRODUCT)/*.fw)
$(foreach var, $(atmel_fw_file),  $(shell objcopy -I binary -O ihex $(var) $(var).ihex))
atmel_ihex_list := $(subst .ihex, , $(notdir $(wildcard $(srctree)/firmware/atmel/$(TARGET_PRODUCT)/*.ihex)))
atmel_ihex_files := $(foreach var, $(atmel_ihex_list), atmel/$(TARGET_PRODUCT)/$(var))
else 
ifdef CONFIG_MACH_MSM8926_JAGC_SPR
atmel_fw_file := $(wildcard $(srctree)/firmware/atmel/$(TARGET_PRODUCT)/*.fw)
$(foreach var, $(atmel_fw_file),  $(shell objcopy -I binary -O ihex $(var) $(var).ihex))
atmel_ihex_list := $(subst .ihex, , $(notdir $(wildcard $(srctree)/firmware/atmel/$(TARGET_PRODUCT)/*.ihex)))
atmel_ihex_files := $(foreach var, $(atmel_ihex_list), atmel/$(TARGET_PRODUCT)/$(var))
else 
ifdef CONFIG_MACH_MSM8926_JAGC_SPR
atmel_fw_file := $(wildcard $(srctree)/firmware/atmel/$(TARGET_PRODUCT)/*.fw)
$(foreach var, $(atmel_fw_file),  $(shell objcopy -I binary -O ihex $(var) $(var).ihex))
atmel_ihex_list := $(subst .ihex, , $(notdir $(wildcard $(srctree)/firmware/atmel/$(TARGET_PRODUCT)/*.ihex)))
atmel_ihex_files := $(foreach var, $(atmel_ihex_list), atmel/$(TARGET_PRODUCT)/$(var))
else 
ifdef CONFIG_MACH_MSM8926_JAGNM_RGS
atmel_fw_file := $(wildcard $(srctree)/firmware/atmel/$(TARGET_PRODUCT)/*.fw)
$(foreach var, $(atmel_fw_file),  $(shell objcopy -I binary -O ihex $(var) $(var).ihex))
atmel_ihex_list := $(subst .ihex, , $(notdir $(wildcard $(srctree)/firmware/atmel/$(TARGET_PRODUCT)/*.ihex)))
atmel_ihex_files := $(foreach var, $(atmel_ihex_list), atmel/$(TARGET_PRODUCT)/$(var))
else 
ifdef CONFIG_MACH_MSM8926_JAGNM_TLS
atmel_fw_file := $(wildcard $(srctree)/firmware/atmel/$(TARGET_PRODUCT)/*.fw)
$(foreach var, $(atmel_fw_file),  $(shell objcopy -I binary -O ihex $(var) $(var).ihex))
atmel_ihex_list := $(subst .ihex, , $(notdir $(wildcard $(srctree)/firmware/atmel/$(TARGET_PRODUCT)/*.ihex)))
atmel_ihex_files := $(foreach var, $(atmel_ihex_list), atmel/$(TARGET_PRODUCT)/$(var))
else 
ifdef CONFIG_MACH_MSM8926_JAGNM_VTR
atmel_fw_file := $(wildcard $(srctree)/firmware/atmel/$(TARGET_PRODUCT)/*.fw)
$(foreach var, $(atmel_fw_file),  $(shell objcopy -I binary -O ihex $(var) $(var).ihex))
atmel_ihex_list := $(subst .ihex, , $(notdir $(wildcard $(srctree)/firmware/atmel/$(TARGET_PRODUCT)/*.ihex)))
atmel_ihex_files := $(foreach var, $(atmel_ihex_list), atmel/$(TARGET_PRODUCT)/$(var))
else
ifdef CONFIG_MACH_MSM8926_T8LTE
atmel_fw_file := $(wildcard $(srctree)/firmware/atmel/t8lte/*.fw)
$(foreach var, $(atmel_fw_file),  $(shell objcopy -I binary -O ihex $(var) $(var).ihex))
atmel_ihex_list := $(subst .ihex, , $(notdir $(wildcard $(srctree)/firmware/atmel/t8lte/*.ihex)))
atmel_ihex_files := $(foreach var, $(atmel_ihex_list), atmel/t8lte/$(var))
else 
atmel_fw_file := $(wildcard $(srctree)/firmware/atmel/*.fw)
$(foreach var, $(atmel_fw_file),  $(shell objcopy -I binary -O ihex $(var) $(var).ihex))
atmel_ihex_list := $(subst .ihex, , $(notdir $(wildcard $(srctree)/firmware/atmel/*.ihex)))
atmel_ihex_files := $(foreach var, $(atmel_ihex_list), atmel/$(var))
endif
endif
endif
endif
endif
endif
endif
endif
fw-shipped-$(CONFIG_TOUCHSCREEN_ATMEL_S336) += $(atmel_ihex_files)
fw-shipped-$(CONFIG_TOUCHSCREEN_ATMEL_T641) += $(atmel_ihex_files)
fw-shipped-$(CONFIG_TOUCHSCREEN_ATMEL_T1664) += $(atmel_ihex_files)
fw-shipped-$(CONFIG_TOUCHSCREEN_ATMEL_T1066) += $(atmel_ihex_files)

ifdef CONFIG_TOUCHSCREEN_LGE_AIT_MELFAS
ifdef CONFIG_MACH_MSM8X10_L70P
ait_fw_file := $(wildcard $(srctree)/firmware/melfas/mit200/l70p/*.fw)
$(foreach var, $(ait_fw_file),  $(shell objcopy -I binary -O ihex $(var) $(var).ihex))

ait_fw_list := $(subst .ihex, , $(notdir $(wildcard $(srctree)/firmware/melfas/mit200/l70p/*.ihex)))
ait_fw_files := $(foreach var, $(ait_fw_list), melfas/mit200/l70p/$(var))
fw-shipped-$(CONFIG_TOUCHSCREEN_LGE_AIT_MELFAS) += $(ait_fw_files)
else
ait_limit_txt := $(wildcard $(srctree)/firmware/melfas/mit200/*.txt)
$(shell objcopy -I binary -O ihex $(ait_limit_txt) $(ait_limit_txt).ihex)
ait_fw_file := $(wildcard $(srctree)/firmware/melfas/mit200/*.fw)
$(foreach var, $(ait_fw_file),  $(shell objcopy -I binary -O ihex $(var) $(var).ihex))

ait_fw_list := $(subst .ihex, , $(notdir $(wildcard $(srctree)/firmware/melfas/mit200/*.ihex)))
ait_fw_files := $(foreach var, $(ait_fw_list), melfas/mit200/$(var))
fw-shipped-$(CONFIG_TOUCHSCREEN_LGE_AIT_MELFAS) += $(ait_fw_files)
endif
else
fw-shipped-$(CONFIG_TOUCHSCREEN_MELFAS_MMS136) += melfas/mms100a_core54_v01.mfsb
fw-shipped-$(CONFIG_TOUCHSCREEN_MELFAS_MMS136) += $(touch_fw_files)

fw-shipped-$(CONFIG_TOUCHSCREEN_MELFAS_MMS100S) += melfas/mms100s_core32_v01.mfsb
fw-shipped-$(CONFIG_TOUCHSCREEN_MELFAS_MMS100S) += $(touch_fw_files)
ifdef CONFIG_MACH_MSM8926_VFP_KR
fw-shipped-$(CONFIG_TOUCHSCREEN_LGE_MELFAS) += melfas/$(TARGET_PRODUCT)/vfp_lgu_kr_rev_a_v001.mfsb
else
fw-shipped-$(CONFIG_TOUCHSCREEN_LGE_MELFAS) += melfas/mms100a_core54_v01.mfsb
fw-shipped-$(CONFIG_TOUCHSCREEN_LGE_MELFAS) += melfas/mms100s_core32_v01.mfsb
endif
fw-shipped-$(CONFIG_TOUCHSCREEN_LGE_MELFAS) += $(touch_fw_files)
endif

fw-shipped-$(CONFIG_TOUCHSCREEN_SYNAPTICS_DSX) += $(synaptics_fw_files)

ifdef CONFIG_TOUCHSCREEN_APS_MIT200_E2
mit200_fw_file := $(wildcard $(srctree)/firmware/melfas/mit200/e2/*)
mit200_firmware := $(subst $(srctree)/firmware/,,$(mit200_fw_file))
fw-shipped-$(CONFIG_TOUCHSCREEN_APS_MIT200_E2) += $(mit200_firmware)
endif

synaptics_img_file := $(wildcard $(srctree)/firmware/synaptics/$(TARGET_PRODUCT)/*.img)
$(foreach var, $(synaptics_img_file),  $(shell objcopy -I binary -O ihex $(var) $(var).ihex))
synaptics_fw_list := $(subst .ihex, , $(notdir $(wildcard $(srctree)/firmware/synaptics/$(TARGET_PRODUCT)/*.ihex)))
synaptics_fw_files := $(foreach var, $(synaptics_fw_list), synaptics/$(TARGET_PRODUCT)/$(var))

touch_synap_limit_txt := $(wildcard $(srctree)/firmware/synaptics/$(TARGET_PRODUCT)/*.txt)
$(foreach var, $(touch_synap_limit_txt),  $(shell objcopy -I binary -O ihex $(var) $(var).ihex))

# LGE_CHANGE - START
# KR only for same DTS usage,  2013-11-28 kyle.jeon@lge.com
ifdef TOUCH_FIRMWARE_KOR_DIR
touch_synap_limit_txt := $(wildcard $(srctree)/firmware/synaptics/$(TOUCH_FIRMWARE_KOR_DIR)/*.txt)
$(shell objcopy -I binary -O ihex $(touch_synap_limit_txt) $(touch_synap_limit_txt).ihex)
synaptics_img_file := $(wildcard $(srctree)/firmware/synaptics/$(TOUCH_FIRMWARE_KOR_DIR)/*.img)
$(foreach var, $(synaptics_img_file),  $(shell objcopy -I binary -O ihex $(var) $(var).ihex))
synaptics_fw_list := $(subst .ihex, , $(notdir $(wildcard $(srctree)/firmware/synaptics/$(TOUCH_FIRMWARE_KOR_DIR)/*.ihex)))
synaptics_fw_files := $(foreach var, $(synaptics_fw_list), synaptics/$(TOUCH_FIRMWARE_KOR_DIR)/$(var))
endif
ifdef TOUCH_FIRMWARE_CN_DIR
touch_synap_limit_txt := $(wildcard $(srctree)/firmware/synaptics/$(TOUCH_FIRMWARE_CN_DIR)/*.txt)
$(shell objcopy -I binary -O ihex $(touch_synap_limit_txt) $(touch_synap_limit_txt).ihex)
synaptics_img_file := $(wildcard $(srctree)/firmware/synaptics/$(TOUCH_FIRMWARE_CN_DIR)/*.img)
$(foreach var, $(synaptics_img_file),  $(shell objcopy -I binary -O ihex $(var) $(var).ihex))
synaptics_fw_list := $(subst .ihex, , $(notdir $(wildcard $(srctree)/firmware/synaptics/$(TOUCH_FIRMWARE_CN_DIR)/*.ihex)))
synaptics_fw_files := $(foreach var, $(synaptics_fw_list), synaptics/$(TOUCH_FIRMWARE_CN_DIR)/$(var))
endif

# LGE_CHANGE - END

fw-shipped-$(CONFIG_LGE_TOUCH_SYNAPTICS) += $(synaptics_fw_files)

# ATMF04
ifdef CONFIG_MACH_MSM8926_T8LTE
fw-shipped-$(CONFIG_SENSOR_ATMF04) += atmf04/atmf04_T8_0_42.fw
endif

fw-shipped-$(CONFIG_3C359) += 3com/3C359.bin
fw-shipped-$(CONFIG_ACENIC) += $(acenic-objs)
fw-shipped-$(CONFIG_ADAPTEC_STARFIRE) += adaptec/starfire_rx.bin \
					 adaptec/starfire_tx.bin
fw-shipped-$(CONFIG_ATARI_DSP56K) += dsp56k/bootstrap.bin
fw-shipped-$(CONFIG_ATM_AMBASSADOR) += atmsar11.fw
fw-shipped-$(CONFIG_BNX2X) += bnx2x/bnx2x-e1-6.2.9.0.fw \
			      bnx2x/bnx2x-e1h-6.2.9.0.fw \
			      bnx2x/bnx2x-e2-6.2.9.0.fw
fw-shipped-$(CONFIG_BNX2) += bnx2/bnx2-mips-09-6.2.1a.fw \
			     bnx2/bnx2-rv2p-09-6.0.17.fw \
			     bnx2/bnx2-rv2p-09ax-6.0.17.fw \
			     bnx2/bnx2-mips-06-6.2.1.fw \
			     bnx2/bnx2-rv2p-06-6.0.15.fw
fw-shipped-$(CONFIG_CASSINI) += sun/cassini.bin
fw-shipped-$(CONFIG_COMPUTONE) += intelliport2.bin
fw-shipped-$(CONFIG_CHELSIO_T3) += cxgb3/t3b_psram-1.1.0.bin \
				   cxgb3/t3c_psram-1.1.0.bin \
				   cxgb3/t3fw-7.10.0.bin \
				   cxgb3/ael2005_opt_edc.bin \
				   cxgb3/ael2005_twx_edc.bin \
				   cxgb3/ael2020_twx_edc.bin
fw-shipped-$(CONFIG_DRM_MGA) += matrox/g200_warp.fw matrox/g400_warp.fw
fw-shipped-$(CONFIG_DRM_R128) += r128/r128_cce.bin
fw-shipped-$(CONFIG_DRM_RADEON) += radeon/R100_cp.bin radeon/R200_cp.bin \
				   radeon/R300_cp.bin radeon/R420_cp.bin \
				   radeon/RS690_cp.bin radeon/RS600_cp.bin \
				   radeon/R520_cp.bin \
				   radeon/R600_pfp.bin radeon/R600_me.bin \
				   radeon/RV610_pfp.bin radeon/RV610_me.bin \
				   radeon/RV630_pfp.bin radeon/RV630_me.bin \
				   radeon/RV620_pfp.bin radeon/RV620_me.bin \
				   radeon/RV635_pfp.bin radeon/RV635_me.bin \
				   radeon/RV670_pfp.bin radeon/RV670_me.bin \
				   radeon/RS780_pfp.bin radeon/RS780_me.bin \
				   radeon/RV770_pfp.bin radeon/RV770_me.bin \
				   radeon/RV730_pfp.bin radeon/RV730_me.bin \
				   radeon/RV710_pfp.bin radeon/RV710_me.bin
fw-shipped-$(CONFIG_DVB_AV7110) += av7110/bootcode.bin
fw-shipped-$(CONFIG_DVB_TTUSB_BUDGET) += ttusb-budget/dspbootcode.bin
fw-shipped-$(CONFIG_E100) += e100/d101m_ucode.bin e100/d101s_ucode.bin \
			     e100/d102e_ucode.bin
fw-shipped-$(CONFIG_MYRI_SBUS) += myricom/lanai.bin
fw-shipped-$(CONFIG_PCMCIA_PCNET) += cis/LA-PCM.cis cis/PCMLM28.cis \
				     cis/DP83903.cis cis/NE2K.cis \
				     cis/tamarack.cis cis/PE-200.cis \
				     cis/PE520.cis
fw-shipped-$(CONFIG_PCMCIA_3C589) += cis/3CXEM556.cis
fw-shipped-$(CONFIG_PCMCIA_3C574) += cis/3CCFEM556.cis
fw-shipped-$(CONFIG_SERIAL_8250_CS) += cis/MT5634ZLX.cis cis/RS-COM-2P.cis \
				       cis/COMpad2.cis cis/COMpad4.cis \
				       cis/SW_555_SER.cis cis/SW_7xx_SER.cis \
				       cis/SW_8xx_SER.cis
fw-shipped-$(CONFIG_PCMCIA_SMC91C92) += ositech/Xilinx7OD.bin
fw-shipped-$(CONFIG_SCSI_ADVANSYS) += advansys/mcode.bin advansys/38C1600.bin \
				      advansys/3550.bin advansys/38C0800.bin
fw-shipped-$(CONFIG_SCSI_QLOGIC_1280) += qlogic/1040.bin qlogic/1280.bin \
					 qlogic/12160.bin
fw-shipped-$(CONFIG_SCSI_QLOGICPTI) += qlogic/isp1000.bin
fw-shipped-$(CONFIG_INFINIBAND_QIB) += qlogic/sd7220.fw
fw-shipped-$(CONFIG_SMCTR) += tr_smctr.bin
fw-shipped-$(CONFIG_SND_KORG1212) += korg/k1212.dsp
fw-shipped-$(CONFIG_SND_MAESTRO3) += ess/maestro3_assp_kernel.fw \
				     ess/maestro3_assp_minisrc.fw
fw-shipped-$(CONFIG_SND_SB16_CSP) += sb16/mulaw_main.csp sb16/alaw_main.csp \
				     sb16/ima_adpcm_init.csp \
				     sb16/ima_adpcm_playback.csp \
				     sb16/ima_adpcm_capture.csp
fw-shipped-$(CONFIG_SND_YMFPCI) += yamaha/ds1_ctrl.fw yamaha/ds1_dsp.fw \
				   yamaha/ds1e_ctrl.fw
fw-shipped-$(CONFIG_SND_WAVEFRONT) += yamaha/yss225_registers.bin
fw-shipped-$(CONFIG_TEHUTI) += tehuti/bdx.bin
fw-shipped-$(CONFIG_TIGON3) += tigon/tg3.bin tigon/tg3_tso.bin \
			       tigon/tg3_tso5.bin
fw-shipped-$(CONFIG_TYPHOON) += 3com/typhoon.bin
fw-shipped-$(CONFIG_USB_DABUSB) += dabusb/firmware.fw dabusb/bitstream.bin
fw-shipped-$(CONFIG_USB_EMI26) += emi26/loader.fw emi26/firmware.fw \
				  emi26/bitstream.fw
fw-shipped-$(CONFIG_USB_EMI62) += emi62/loader.fw emi62/bitstream.fw \
				  emi62/spdif.fw emi62/midi.fw
fw-shipped-$(CONFIG_USB_KAWETH) += kaweth/new_code.bin kaweth/trigger_code.bin \
				   kaweth/new_code_fix.bin \
				   kaweth/trigger_code_fix.bin
ifdef CONFIG_FIRMWARE_IN_KERNEL
fw-shipped-$(CONFIG_USB_SERIAL_KEYSPAN_MPR) += keyspan/mpr.fw
fw-shipped-$(CONFIG_USB_SERIAL_KEYSPAN_USA18X) += keyspan/usa18x.fw
fw-shipped-$(CONFIG_USB_SERIAL_KEYSPAN_USA19) += keyspan/usa19.fw
fw-shipped-$(CONFIG_USB_SERIAL_KEYSPAN_USA19QI) += keyspan/usa19qi.fw
fw-shipped-$(CONFIG_USB_SERIAL_KEYSPAN_USA19QW) += keyspan/usa19qw.fw
fw-shipped-$(CONFIG_USB_SERIAL_KEYSPAN_USA19W) += keyspan/usa19w.fw
fw-shipped-$(CONFIG_USB_SERIAL_KEYSPAN_USA28) += keyspan/usa28.fw
fw-shipped-$(CONFIG_USB_SERIAL_KEYSPAN_USA28XA) += keyspan/usa28xa.fw
fw-shipped-$(CONFIG_USB_SERIAL_KEYSPAN_USA28XB) += keyspan/usa28xb.fw
fw-shipped-$(CONFIG_USB_SERIAL_KEYSPAN_USA28X) += keyspan/usa28x.fw
fw-shipped-$(CONFIG_USB_SERIAL_KEYSPAN_USA49W) += keyspan/usa49w.fw
fw-shipped-$(CONFIG_USB_SERIAL_KEYSPAN_USA49WLC) += keyspan/usa49wlc.fw
else
fw-shipped- += keyspan/mpr.fw keyspan/usa18x.fw keyspan/usa19.fw	\
	keyspan/usa19qi.fw keyspan/usa19qw.fw keyspan/usa19w.fw		\
	keyspan/usa28.fw keyspan/usa28xa.fw keyspan/usa28xb.fw		\
	keyspan/usa28x.fw keyspan/usa49w.fw keyspan/usa49wlc.fw
endif
fw-shipped-$(CONFIG_USB_SERIAL_TI) += ti_3410.fw ti_5052.fw \
				      mts_cdma.fw mts_gsm.fw mts_edge.fw
fw-shipped-$(CONFIG_USB_SERIAL_EDGEPORT) += edgeport/boot.fw edgeport/boot2.fw \
					    edgeport/down.fw edgeport/down2.fw
fw-shipped-$(CONFIG_USB_SERIAL_EDGEPORT_TI) += edgeport/down3.bin
fw-shipped-$(CONFIG_USB_SERIAL_WHITEHEAT) += whiteheat_loader.fw whiteheat.fw \
					   # whiteheat_loader_debug.fw
fw-shipped-$(CONFIG_USB_SERIAL_KEYSPAN_PDA) += keyspan_pda/keyspan_pda.fw
fw-shipped-$(CONFIG_USB_SERIAL_XIRCOM) += keyspan_pda/xircom_pgs.fw
fw-shipped-$(CONFIG_USB_VICAM) += vicam/firmware.fw
fw-shipped-$(CONFIG_VIDEO_CPIA2) += cpia2/stv0672_vp4.bin
fw-shipped-$(CONFIG_YAM) += yam/1200.bin yam/9600.bin

fw-shipped-all := $(fw-shipped-y) $(fw-shipped-m) $(fw-shipped-)

# Directories which we _might_ need to create, so we have a rule for them.
firmware-dirs := $(sort $(addprefix $(objtree)/$(obj)/,$(dir $(fw-external-y) $(fw-shipped-all))))

quiet_cmd_mkdir = MKDIR   $(patsubst $(objtree)/%,%,$@)
      cmd_mkdir = mkdir -p $@

quiet_cmd_ihex  = IHEX    $@
      cmd_ihex  = $(OBJCOPY) -Iihex -Obinary $< $@

quiet_cmd_copy  = COPY    $@
      cmd_copy  = cp $(srctree)/$@ $@

quiet_cmd_ihex2fw  = IHEX2FW $@
      cmd_ihex2fw  = $(objtree)/$(obj)/ihex2fw $< $@

quiet_cmd_h16tofw  = H16TOFW $@
      cmd_h16tofw  = $(objtree)/$(obj)/ihex2fw -w $< $@

quiet_cmd_fwbin = MK_FW   $@
      cmd_fwbin = FWNAME="$(patsubst firmware/%.gen.S,%,$@)";		     \
		  FWSTR="$(subst /,_,$(subst .,_,$(subst -,_,$(patsubst	     \
				firmware/%.gen.S,%,$@))))";		     \
		  ASM_WORD=$(if $(CONFIG_64BIT),.quad,.long);		     \
		  ASM_ALIGN=$(if $(CONFIG_64BIT),3,2);			     \
		  PROGBITS=$(if $(CONFIG_ARM),%,@)progbits;		     \
		  echo "/* Generated by firmware/Makefile */"		> $@;\
		  echo "    .section .rodata"				>>$@;\
		  echo "    .p2align $${ASM_ALIGN}"			>>$@;\
		  echo "_fw_$${FWSTR}_bin:"				>>$@;\
		  echo "    .incbin \"$(2)\""				>>$@;\
		  echo "_fw_end:"					>>$@;\
		  echo "   .section .rodata.str,\"aMS\",$${PROGBITS},1"	>>$@;\
		  echo "    .p2align $${ASM_ALIGN}"			>>$@;\
		  echo "_fw_$${FWSTR}_name:"				>>$@;\
		  echo "    .string \"$$FWNAME\""			>>$@;\
		  echo "    .section .builtin_fw,\"a\",$${PROGBITS}"	>>$@;\
		  echo "    .p2align $${ASM_ALIGN}"			>>$@;\
		  echo "    $${ASM_WORD} _fw_$${FWSTR}_name"		>>$@;\
		  echo "    $${ASM_WORD} _fw_$${FWSTR}_bin"		>>$@;\
		  echo "    $${ASM_WORD} _fw_end - _fw_$${FWSTR}_bin"	>>$@;

# One of these files will change, or come into existence, whenever
# the configuration changes between 32-bit and 64-bit. The .S files
# need to change when that happens.
wordsize_deps := $(wildcard include/config/64bit.h include/config/32bit.h \
		include/config/ppc32.h include/config/ppc64.h \
		include/config/superh32.h include/config/superh64.h \
		include/config/x86_32.h include/config/x86_64.h)

# Workaround for make < 3.81, where .SECONDEXPANSION doesn't work.
# It'll end up depending on these targets, so make them a PHONY rule which
# depends on _all_ the directories in $(firmware-dirs), and it'll work out OK.
PHONY += $(objtree)/$$(%) $(objtree)/$(obj)/$$(%)
$(objtree)/$$(%) $(objtree)/$(obj)/$$(%): $(firmware-dirs)
	@true

# For the $$(dir %) trick, where we need % to be expanded first.
.SECONDEXPANSION:

$(patsubst %,$(obj)/%.gen.S, $(fw-shipped-y)): %: $(wordsize_deps) \
		| $(objtree)/$$(dir %)
	$(call cmd,fwbin,$(patsubst %.gen.S,%,$@))
$(patsubst %,$(obj)/%.gen.S, $(fw-external-y)): %: $(wordsize_deps) \
		include/config/extra/firmware/dir.h | $(objtree)/$$(dir %)
	$(call cmd,fwbin,$(fwabs)/$(patsubst $(obj)/%.gen.S,%,$@))

# The .o files depend on the binaries directly; the .S files don't.
$(patsubst %,$(obj)/%.gen.o, $(fw-shipped-y)): %.gen.o: %
$(patsubst %,$(obj)/%.gen.o, $(fw-external-y)): $(obj)/%.gen.o: $(fwdir)/%

# .ihex is used just as a simple way to hold binary files in a source tree
# where binaries are frowned upon. They are directly converted with objcopy.
$(obj)/%: $(obj)/%.ihex | $(objtree)/$(obj)/$$(dir %)
	$(call cmd,ihex)

$(obj)/%.fw: $(objtree)/$(obj)/$$(dir %.fw)
	$(call cmd,copy)

# Don't depend on ihex2fw if we're installing and it already exists.
# Putting it after | in the dependencies doesn't seem sufficient when
# we're installing after a cross-compile, because ihex2fw has dependencies
# on stuff like /usr/lib/gcc/ppc64-redhat-linux/4.3.0/include/stddef.h and 
# thus wants to be rebuilt. Which it can't be, if the prebuilt kernel tree
# is exported read-only for someone to run 'make install'.
ifeq ($(INSTALL):$(wildcard $(obj)/ihex2fw),install:$(obj)/ihex2fw)
ihex2fw_dep :=
else
ihex2fw_dep := $(obj)/ihex2fw
endif

# .HEX is also Intel HEX, but where the offset and length in each record
# is actually meaningful, because the firmware has to be loaded in a certain
# order rather than as a single binary blob. Thus, we convert them into our
# more compact binary representation of ihex records (<linux/ihex.h>)
$(obj)/%.fw: $(obj)/%.HEX $(ihex2fw_dep) | $(objtree)/$(obj)/$$(dir %)
	$(call cmd,ihex2fw)

# .H16 is our own modified form of Intel HEX, with 16-bit length for records.
$(obj)/%.fw: $(obj)/%.H16 $(ihex2fw_dep) | $(objtree)/$(obj)/$$(dir %)
	$(call cmd,h16tofw)

$(firmware-dirs):
	$(call cmd,mkdir)

obj-y				 += $(patsubst %,%.gen.o, $(fw-external-y))
obj-$(CONFIG_FIRMWARE_IN_KERNEL) += $(patsubst %,%.gen.o, $(fw-shipped-y))

# Remove .S files and binaries created from ihex
# (during 'make clean' .config isn't included so they're all in $(fw-shipped-))
targets := $(fw-shipped-) $(patsubst $(obj)/%,%, \
				$(shell find $(obj) -name \*.gen.S 2>/dev/null))

# Without this, built-in.o won't be created when it's empty, and the
# final vmlinux link will fail.
obj-n := dummy

hostprogs-y := ihex2fw
