################################################################################
# Garrett Smith 2010
################################################################################

OETMP = ${OVEROTOP}/tmp

TOOLDIR       ?= ${OETMP}/sysroots/`uname -m`-linux/usr/armv7a/bin
STAGEDIR      ?= ${OETMP}/sysroots/armv7a-angstrom-linux-gnueabi/usr
CROSS_COMPILE ?= ${TOOLDIR}/arm-angstrom-linux-gnueabi-
CC            = @${CROSS_COMPILE}gcc

DEP_OUTPUT_OPTION = -MMD -MF $(@:.o=.d)
OUTPUT_OPTION = -c
CFLAGS += -Wall -Werror -O2

APP_NAMES = gpio_test pwm_test rssi_test madc_test gles_test mjpg_test
GPIO_OBJS = gpio_test.o
PWM_OBJS  = pwm_test.o
RSSI_OBJS = rssi_test.o
MADC_OBJS = madc_test.o
GLES_OBJS = gles_test.o
MJPG_OBJS = jpeg_utils.o v4l2uvc.o input_uvc.o output_udp.o mjpg_streamer.o mjpg_test.o
UDP_SERV  = udp_server.o

OBJ_FILES = ${GPIO_OBJS} ${PWM_OBJS} ${RSSI_OBJS}
DEP_FILES = $(strip $(OBJ_FILES:.o=.d))

all: ${APP_NAMES}

gpio_test: ${GPIO_OBJS}
	@echo "[LD] $@"
	$(CC) $(LDFLAGS) -lpthread -o $@ $^

pwm_test: ${PWM_OBJS}
	@echo "[LD] $@"
	$(CC) $(LDFLAGS) -lpthread -o $@ $^

rssi_test: ${RSSI_OBJS}
	@echo "[LD] $@"
	$(CC) $(LDFLAGS) -lpthread -o $@ $^

madc_test: ${MADC_OBJS}
	@echo "[LD] $@"
	$(CC) $(LDFLAGS) -lpthread -o $@ $^

gles_test: ${GLES_OBJS}
	@echo "[LD] $@"
	$(CC) $(LDFLAGS) -lpthread -lGLES_CM -lEGL -o $@ $^

mjpg_test: ${MJPG_OBJS}
	@echo "[LD] $@"
	$(CC) $(LDFLAGS) -lpthread -ljpeg -o $@ $^

udp_serv: $(UDP_SERV)
	@echo "[LD] $@"
	$(CC) $(LDFLAGS) -o $@ $^

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

