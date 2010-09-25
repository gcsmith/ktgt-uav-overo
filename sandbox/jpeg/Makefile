################################################################################
# Garrett Smith 2010
################################################################################

OETMP = ${OVEROTOP}/tmp

TOOLDIR       ?= ${OETMP}/sysroots/`uname -m`-linux/usr/armv7a/bin
STAGEDIR      ?= ${OETMP}/sysroots/armv7a-angstrom-linux-gnueabi/usr
CROSS_COMPILE ?= ${TOOLDIR}/arm-angstrom-linux-gnueabi-
CC            = @${CROSS_COMPILE}gcc

DEP_OUTPUT_OPTION = -MMD -MF $(@:.o=.d)
OUTPUT_OPTION = -c -o $@
CFLAGS += -Wall -Werror -O2
LDFLAGS += -ljpeg -lm

MP = ../uav-control/mjpg-streamer

APP_NAMES = jpeg_test
JPEG_OBJS = colordetect.o readwritejpeg.o

OBJ_FILES = ${JPEG_OBJS} ${PWM_OBJS} ${RSSI_OBJS}
DEP_FILES = $(strip $(OBJ_FILES:.o=.d))

all: ${APP_NAMES}

jpeg_test: ${JPEG_OBJS}
	@echo "[LD] $@"
	$(CC) $(LDFLAGS) -lpthread -o $@ $^

.PHONY: %.d
%.d: ;

%.o : %.c %.d
	@echo "[CC] $<"
	$(CC) $(CFLAGS) $(DEP_OUTPUT_OPTION) $(OUTPUT_OPTION) $<

clean:
	@rm -rf $(APP_NAMES) *.o *.d

ifneq ($(DEP_FILES),)
ifeq ($(strip $(filter clean%, $(MAKECMDGOALS))),)
-include $(DEP_FILES)
endif
endif

