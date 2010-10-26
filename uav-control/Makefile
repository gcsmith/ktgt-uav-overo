################################################################################
# Makefile for building the UAV control software
# Garrett Smith 2010
################################################################################

APPNAME = uav_control
OETMP = ${OVEROTOP}/tmp

TOOLDIR       ?= ${OETMP}/sysroots/`uname -m`-linux/usr/armv7a/bin
STAGEDIR      ?= ${OETMP}/sysroots/armv7a-angstrom-linux-gnueabi/usr
CROSS_COMPILE ?= ${TOOLDIR}/arm-angstrom-linux-gnueabi-
CC            = @${CROSS_COMPILE}gcc
PROFILE       ?= "release"

DEP_OUTPUT_OPTION = -MMD -MF $(@:.o=.d)

CFLAGS += -Wall -Werror -D_GNU_SOURCE \
          -I../pwm-module \
          -I../pwm-lib \
          -I../gpio-event-module \
          -I../gpio-lib

LFLAGS += $(LDFLAGS) \
          -L../pwm-lib \
          -L../gpio-lib \
          -lpthread -ljpeg -lpwm -lgpio -lm -lrt -lv4l2

ifeq ("$(PROFILE)", "gprof")
	CFLAGS += -O0 -ggdb -pg
	LFLAGS += -O0 -ggdb -pg
else
ifeq ("$(PROFILE)", "debug")
	CFLAGS += -O0 -ggdb
	LFLAGS += -O0 -ggdb
else
	CFLAGS += -O2
	LFLAGS += -O2
endif
endif

OBJ_FILES = colordetect.o \
            flight_control.o \
            gpio_event.o \
            pid.o \
            razor_imu.o \
            readwritejpeg.o \
			tracking.o \
            uav_control.o \
            utility.o \
            v4l2uvc.o \
            video_uvc.o

DEP_FILES = $(strip $(OBJ_FILES:.o=.d))

$(APPNAME) : $(OBJ_FILES)
	@echo "[LD] $@"
	$(CC) -o $@ $^ $(LFLAGS)

.PHONY: %.d
%.d: ;

%.o : %.c %.d
	@echo "[CC] $<"
	$(CC) $(CFLAGS) $(DEP_OUTPUT_OPTION) -c -o $@ $<

clean:
	@rm -rf $(APPNAME) *.o *.d 

ifneq ($(DEP_FILES),)
ifeq ($(strip $(filter clean%, $(MAKECMDGOALS))),)
-include $(DEP_FILES)
endif
endif

