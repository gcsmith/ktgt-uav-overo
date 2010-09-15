// -----------------------------------------------------------------------------
// Test for outputting to PWM channel using ioctl interface.
// Garrett Smith 2010
// -----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "pwm_interface.h"

int main(int argc, char *argv[])
{
    int i, j;
    unsigned int lower, upper, range;
    int fd = open("/dev/pwm8", 0);
    if (fd < 0) {
        fprintf(stderr, "failed to open pwm device\n");
        return 1;
    }
    printf("successfully opened pwm device\n");

    // test with low precision 'duty' command
    for (j = 0; j < 3; j++) {
        for (i = 2; i < 13; i++) {
            ioctl(fd, PWM_IOCT_DUTY, i);
            usleep(100000);
        }
        for (i = 13; i >= 2; i--) {
            ioctl(fd, PWM_IOCT_DUTY, i);
            usleep(100000);
        }
    }

    //  test with high precision 'compare' command
    lower = ioctl(fd, PWM_IOCQ_MINRANGE);
    upper = ioctl(fd, PWM_IOCQ_MAXRANGE);
    range = upper - lower;

    printf("compare range min: %u\n", lower);
    printf("compare range max: %u\n", upper);
    printf("precision capable: %u\n", range);

    range /= 7; // gives us a range of 0-14% duty cycle

    for (j = 0; j < 3; j++) {
        for (i = lower; i < (lower + range); i += 100) {
            ioctl(fd, PWM_IOCT_COMPARE, i);
            usleep(5000);
        }
        for (i = lower + range; i >= lower; i -= 100) {
            ioctl(fd, PWM_IOCT_COMPARE, i);
            usleep(5000);
        }
    }

    close(fd);
    return 0;
}

