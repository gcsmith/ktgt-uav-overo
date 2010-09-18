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
CFLAGS += -Wall -Werror -O2

OBJ_FILES = uav_control.o \
			mjpg-streamer/jpeg_utils.o \
			mjpg-streamer/v4l2uvc.o \
			mjpg-streamer/input_uvc.o \
			mjpg-streamer/output_udp.o \
			mjpg-streamer/mjpg_streamer.o
DEP_FILES = $(strip $(OBJ_FILES:.o=.d))

$(APPNAME) : $(OBJ_FILES)
	@echo "[LD] $@"
	$(CC) $(LDFLAGS) -lpthread -ljpeg -o $@ $^

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



