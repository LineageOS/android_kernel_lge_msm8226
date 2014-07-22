#
# Makefile for nfc devices
#

obj-$(CONFIG_LGE_NFC)	:= pn544_lge.o
#obj-$(CONFIG_LGE_NFC)	:= pn544.o
obj-$(CONFIG_LGE_NFC)	+= pn544_lge_hwadapter.o

ccflags-$(CONFIG_NFC_DEBUG) := -DDEBUG
