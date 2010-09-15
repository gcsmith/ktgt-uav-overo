// -----------------------------------------------------------------------------
// Test for measuring from onboard analog-to-digital input (MADC).
// Garrett Smith 2010
// -----------------------------------------------------------------------------

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

typedef uint8_t u8;
typedef uint16_t u16;

#include "twl4030-madc.h"

struct adc_channel {
    int number;
    char name[16];
    float input_range;
};

struct adc_channel channels[] = {
    {
        .number = 2,
        .name = "ADCIN2",
        .input_range = 2.5,
    },
    {
        .number = 3,
        .name = "ADCIN3",
        .input_range = 2.5,
    },
    {
        .number = 4,
        .name = "ADCIN4",
        .input_range = 2.5,
    },
    {
        .number = 5,
        .name = "ADCIN5",
        .input_range = 2.5,
    },
    {
        .number = 6,
        .name = "ADCIN6",
        .input_range = 2.5,
    },
    {
        .number = 7,
        .name = "ADCIN7",
        .input_range = 2.5,
    },
    {
        .number = 8,
        .name = "VBUS_USB_OTG",
        .input_range = 7.0,
    },
    {
        .number = 12,
        .name = "VBATT/3.3V_RAIL",
        .input_range = 6.0,
    },
}; 

int main (int argc, char *argv[])
{
    int fd, rc;
    struct twl4030_madc_user_parms parms;

    if (0 > (fd = open("/dev/twl4030-madc", O_RDWR | O_NONBLOCK))) {
        perror("failed to open /dev/twl4030-madc\n");
        return EXIT_FAILURE;
    }

    for (;;) {
        memset(&parms, 0, sizeof(parms));
        parms.channel = channels[2].number;

        rc = ioctl(fd, TWL4030_MADC_IOCX_ADC_RAW_READ, &parms);
        printf("read %x from %s\n", parms.result, channels[2].name);
    }

    close(fd);
    return EXIT_SUCCESS;
}

