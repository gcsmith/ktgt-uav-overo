#############################################################################
# Garrett Smith 2010
##############################################################################

LIBNAME = pwm
STATIC_LIBNAME = lib$(LIBNAME).a

PWD   = $(shell pwd)
OETMP = ${OVEROTOP}/tmp

TOOLDIR       ?= ${OETMP}/sysroots/`uname -m`-linux/usr/armv7a/bin
STAGEDIR      ?= ${OETMP}/sysroots/armv7a-angstrom-linux-gnueabi/usr
KERNEL_PATH   ?= $(OETMP)/sysroots/overo-angstrom-linux-gnueabi/kernel
CROSS_COMPILE ?= ${TOOLDIR}/arm-angstrom-linux-gnueabi-
ARCH          ?= arm

CC = $(CROSS_COMPILE)gcc
AR = $(CROSS_COMPILE)ar

CFLAGS += -std=gnu99 \
          -Wall \
          -Wimplicit \
          -Wpointer-arith \
          -Wswitch \
          -Wredundant-decls \
          -Wreturn-type \
          -Wshadow \
          -Wunused \
          -Wcast-qual \
          -Wnested-externs \
          -Wmissing-prototypes \
          -Wstrict-prototypes \
          -Wmissing-declarations \
          -I ../pwm-module

DEP_OUTPUT_OPTION = -MMD -MF $(@:.o=.d)

OBJ_FILES = pwm_lib.o
STATIC_OBJ_FILES = $(addprefix static/,$(OBJ_FILES))

all : $(STATIC_LIBNAME)

$(STATIC_OBJ_FILES) : static

static:
	mkdir $@

.PHONY: %.d
%.d: ;

static/%.o : %.c %.d
	@echo "Compiling $< ..."
	$(COMPILE.c) $(DEP_OUTPUT_OPTION) $(OUTPUT_OPTION) $<

$(STATIC_LIBNAME) : $(STATIC_OBJ_FILES)
	@echo "Creating static library $@ ..."
	rm -f $@
	$(AR) $(ARFLAGS) $@ $^

clean:
	rm -rf static *.a *.so*

DEP_FILES = $(strip $(STATIC_OBJ_FILES:.o=.d))

ifneq ($(DEP_FILES),)
ifeq ($(strip $(filter clean%, $(MAKECMDGOALS))),)
-include $(DEP_FILES)
endif
endif


