################################################################################
# Garrett Smith 2010
################################################################################

OETMP = ${OVEROTOP}/tmp

TOOLDIR       ?= ${OETMP}/sysroots/`uname -m`-linux/usr/armv7a/bin
STAGEDIR      ?= ${OETMP}/sysroots/armv7a-angstrom-linux-gnueabi/usr
CROSS_COMPILE ?= ${TOOLDIR}/arm-angstrom-linux-gnueabi-
CC            = @${CROSS_COMPILE}gcc
PROFILE       ?= "release"

DEP_OUTPUT_OPTION = -MMD -MF $(@:.o=.d)
OUTPUT_OPTION = -c -o $@
CFLAGS += -Wall -Werror \
          -I../pwm-module \
          -I../pwm-lib \
          -I../gpio-event-module \
          -I../gpio-lib \
          -I../uav-control

UC = ../uav-control

ifeq ("$(PROFILE)", "gprof")
    CFLAGS += -O0 -ggdb -pg
    LDFLAGS += -O0 -ggdb -pg
else
ifeq ("$(PROFILE)", "debug")
    CFLAGS += -O0 -ggdb
    LDFLAGS += -O0 -ggdb
else
    CFLAGS += -O2
    LDFLAGS += -O2
endif
endif

APP_NAMES = evnt_test \
            gles_test \
            gpio_test \
            fixed_test \
            jpeg_test \
            madc_test \
            mjpg_test \
            pwm_test \
            rssi_test

EVNT_OBJS = evnt_test.o
GLES_OBJS = gles_test.o \
            $(UC)/colordetect.o \
            $(UC)/readwritejpeg.o \
            $(UC)/video_uvc.o \
            $(UC)/v4l2uvc.o
GPIO_OBJS = gpio_test.o
FIXED_OBJS = fixed_test.o \
            $(UC)/colordetect.o \
            $(UC)/readwritejpeg.o
JPEG_OBJS = jpeg_test.o \
            $(UC)/colordetect.o \
            $(UC)/readwritejpeg.o
MADC_OBJS = madc_test.o
MJPG_OBJS = mjpg_test.o \
            $(UC)/video_uvc.o \
            $(UC)/v4l2uvc.o
PWM_OBJS  = pwm_test.o
RSSI_OBJS = rssi_test.o

OBJ_FILES = ${EVNT_OBJS} \
            ${GLES_OBJS} \
            ${GPIO_OBJS} \
            ${FIXED_OBJS} \
            ${JPEG_OBJS} \
            ${MADC_OBJS} \
            ${MJPG_OBJS} \
            ${PWM_OBJS} \
            ${RSSI_OBJS}
DEP_FILES = $(strip $(OBJ_FILES:.o=.d))

all: ${APP_NAMES}

evnt_test: ${EVNT_OBJS}
	@echo "[LD] $@"
	$(CC) $(LDFLAGS) -lpthread -o $@ $^

gles_test: ${GLES_OBJS}
	@echo "[LD] $@"
	$(CC) $(LDFLAGS) -lpthread -ljpeg -lGLES_CM -lEGL -lv4l2 -o $@ $^

gpio_test: ${GPIO_OBJS}
	@echo "[LD] $@"
	$(CC) $(LDFLAGS) -lpthread -o $@ $^ -L../gpio-lib -lgpio
	
fixed_test: ${FIXED_OBJS}
	@echo "[LD] $@"
	$(CC) $(LDFLAGS) -lpthread -ljpeg -lm -lrt -o $@ $^ 

jpeg_test: ${JPEG_OBJS}
	@echo "[LD] $@"
	$(CC) $(LDFLAGS) -lpthread -ljpeg -lm -lrt -o $@ $^ 

madc_test: ${MADC_OBJS}
	@echo "[LD] $@"
	$(CC) $(LDFLAGS) -lpthread -o $@ $^

mjpg_test: ${MJPG_OBJS}
	@echo "[LD] $@"
	$(CC) $(LDFLAGS) -lpthread -ljpeg -lv4l2 -o $@ $^

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
	@rm -rf $(APP_NAMES) *.o *.d ../uav-control/*.o ../uav-control/*.d

ifneq ($(DEP_FILES),)
ifeq ($(strip $(filter clean%, $(MAKECMDGOALS))),)
-include $(DEP_FILES)
endif
endif

