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
CFLAGS += -Wall -Werror -O2 \
          -I../pwm-module \
          -I../pwm-lib \
          -I../gpio-event-module \
          -I../gpio-lib

MP = ../uav-control/mjpg-streamer

APP_NAMES = evnt_test gles_test gpio_test madc_test mjpg_test pwm_test rssi_test
EVNT_OBJS = evnt_test.o
GLES_OBJS = gles_test.o
GPIO_OBJS = gpio_test.o
MADC_OBJS = madc_test.o
MJPG_OBJS = $(MP)/jpeg_utils.o $(MP)/v4l2uvc.o $(MP)/input_uvc.o $(MP)/output_udp.o $(MP)/mjpg_streamer.o mjpg_test.o
PWM_OBJS  = pwm_test.o
RSSI_OBJS = rssi_test.o

OBJ_FILES = ${EVNT_OBJS} ${PWM_OBJS} ${RSSI_OBJS}
DEP_FILES = $(strip $(OBJ_FILES:.o=.d))

all: ${APP_NAMES}

evnt_test: ${EVNT_OBJS}
	@echo "[LD] $@"
	$(CC) $(LDFLAGS) -lpthread -o $@ $^

gles_test: ${GLES_OBJS}
	@echo "[LD] $@"
	$(CC) $(LDFLAGS) -lpthread -lGLES_CM -lEGL -o $@ $^

gpio_test: ${GPIO_OBJS}
	@echo "[LD] $@"
	$(CC) $(LDFLAGS) -lpthread -o $@ $^ -L../gpio-lib -lgpio

madc_test: ${MADC_OBJS}
	@echo "[LD] $@"
	$(CC) $(LDFLAGS) -lpthread -o $@ $^

mjpg_test: ${MJPG_OBJS}
	@echo "[LD] $@"
	$(CC) $(LDFLAGS) -lpthread -ljpeg -o $@ $^

pwm_test: ${PWM_OBJS}
	@echo "[LD] $@"
	$(CC) $(LDFLAGS) -lpthread -o $@ $^

rssi_test: ${RSSI_OBJS}
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

