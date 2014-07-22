#
# Makefile for the kernel mmc core.
#

obj-$(CONFIG_MMC)		+= mmc_core.o
mmc_core-y			:= core.o bus.o host.o \
				   mmc.o mmc_ops.o sd.o sd_ops.o \
				   sdio.o sdio_ops.o sdio_bus.o \
				   sdio_cis.o sdio_io.o sdio_irq.o \
				   quirks.o cd-gpio.o

# LGE_CHANGE_S 
# Comment : FMBT porting
# 2013-11-22, p1-fs@lge.com
mmc_core-$(CONFIG_FMBT_TRACE_EMMC) += mem_log.o
# LGE_CHANGE_E

mmc_core-$(CONFIG_DEBUG_FS)	+= debugfs.o
