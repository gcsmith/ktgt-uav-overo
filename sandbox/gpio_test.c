// -----------------------------------------------------------------------------
// Garrett Smith 2010
// -----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <ctype.h>
#include <unistd.h>
#include "user-gpio.h"

int main(int argc, char *argv[])
{
    static int gpio = 170;
    int i;

    if (0 > gpio_init()) {
        perror("gpio_init failed");
        return 1;
    }

    if (0 > gpio_request(gpio, "gpio_test output")) {
        perror("gpio_request failed");
        return 1;
    }

    if (0 > gpio_direction_output(gpio, 0)) {
        perror("gpio_direction_output failed");
        return 1;
    }

    for (i = 0; i < 10; i++)
    {
        printf("setting value to %d\n", i % 2);
        gpio_set_value(gpio, i % 2);
        sleep(1);
    }

    gpio_term();
    return 0;
}

