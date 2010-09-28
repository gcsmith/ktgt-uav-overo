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

DEP_OUTPUT_OPTION = -MMD -MF $(@:.o=.d)

CFLAGS += -Wall -Werror -O2 -D_GNU_SOURCE \
          -I../pwm-module \
          -I../pwm-lib \
          -I../gpio-event-module \
          -I../gpio-lib

LFLAGS += $(LDFLAGS) \
          -L../pwm-lib \
          -L../gpio-lib \
          -lpthread -ljpeg -lpwm -lgpio

OBJ_FILES = flight_control.o \
            gpio_event.o \
            razor_imu.o \
            server.o \
            uav_control.o \
            video_uvc.o \
            mjpg-streamer/v4l2uvc.o

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
	@rm -rf $(APPNAME) *.o *.d mjpg-streamer/*.o mjpg-streamer/*.d

ifneq ($(DEP_FILES),)
ifeq ($(strip $(filter clean%, $(MAKECMDGOALS))),)
-include $(DEP_FILES)
endif
endif

