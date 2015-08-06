
obj-m += exfatfs.o

exfatfs-y := exfat_super.o exfat.o exfat_api.o exfat_blkdev.o exfat_cache.o \
				exfat_data.o exfat_global.o exfat_nls.o exfat_oal.o exfat_upcase.o

L_KOUT ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

all:
	make -C $(L_KOUT) $(L_TARGET_ARCH) $(L_TARGET_CC) M=$(PWD) modules

clean:
	make -C $(L_KOUT) $(L_TARGET_ARCH) M=$(PWD) clean

