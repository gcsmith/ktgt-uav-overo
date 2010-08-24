################################################################################
# Makefile for building the PWM kernel module
# Garrett Smith 2010
################################################################################

ifneq ($(KERNELRELEASE),)

# use kernel build system
obj-m := pwm_module.o

else

PWD   = $(shell pwd)
OETMP = ${OVEROTOP}/tmp

TOOLDIR       ?= ${OETMP}/sysroots/`uname -m`-linux/usr/armv7a/bin
STAGEDIR      ?= ${OETMP}/sysroots/armv7a-angstrom-linux-gnueabi/usr
KERNEL_PATH   ?= $(OETMP)/sysroots/overo-angstrom-linux-gnueabi/kernel
CROSS_COMPILE ?= ${TOOLDIR}/arm-angstrom-linux-gnueabi-
ARCH          ?= arm

export ARCH CROSS_COMPILE

KBUILD_FLAGS = -C $(KERNEL_PATH) M=$(PWD)

default: modules

modules:
	$(MAKE) $(KBUILD_FLAGS) modules

clean:
	rm -rf *.o *~ *.ko *.mod.c .*.cmd .tmp* modules.order Module.symvers

endif

