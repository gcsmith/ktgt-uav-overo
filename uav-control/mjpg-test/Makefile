################################################################################
# Makefile for building the UAV control software
# Garrett Smith 2010
################################################################################

APPNAME = mjpg_test 
OETMP = ${OVEROTOP}/tmp

TOOLDIR       ?= ${OETMP}/sysroots/`uname -m`-linux/usr/armv7a/bin
STAGEDIR      ?= ${OETMP}/sysroots/armv7a-angstrom-linux-gnueabi/usr
CROSS_COMPILE ?= ${TOOLDIR}/arm-angstrom-linux-gnueabi-
CC            = @${CROSS_COMPILE}gcc

DEP_OUTPUT_OPTION = -MMD -MF $(@:.o=.d)
OUTPUT_OPTION = -c
CFLAGS += -Wall -Werror -O2

OBJ_FILES = ../jpeg_utils.o ../v4l2uvc.o ../input_uvc.o ../output_udp.o ../mjpg_streamer.o mjpg_test.o
DEP_FILES = $(strip $(OBJ_FILES:.o=.d))

$(APPNAME) : $(OBJ_FILES)
	@echo "[LD] $@"
	$(CC) $(LDFLAGS) -lpthread -ljpeg -o $@ $^

.PHONY: %.d
%.d: ;

%.o : %.c %.d
	@echo "[CC] $<"
	$(CC) $(CFLAGS) $(DEP_OUTPUT_OPTION) $(OUTPUT_OPTION) $<

clean:
	@rm -rf $(APPNAME) *.o *.d

ifneq ($(DEP_FILES),)
ifeq ($(strip $(filter clean%, $(MAKECMDGOALS))),)
-include $(DEP_FILES)
endif
endif



