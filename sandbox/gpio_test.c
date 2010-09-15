#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <ctype.h>
#include <unistd.h>
#include "gpio-event-drv.h"

int main(int argc, char *argv[])
{
    struct timeval tv;
    GPIO_EventMonitor_t monitor;
    GPIO_Event_t event;
    fd_set rdset;
    int fd = 0, rc = 0, last_usec = 0;

    /* open the gpio-event device node */
    fd = open("/dev/gpio-event", 0);
    if (fd < 0) {
        fprintf(stderr, "failed to open /dev/gpio-event\n");
        return 1;
    }
    printf("successfully opened /dev/gpio-event for reading\n");

    /* set read mode to binary (default is ascii) */
    ioctl(fd, GPIO_EVENT_IOCTL_SET_READ_MODE, GPIO_EventReadModeBinary);

    /* initialize monitor for gpio146, detect both rising/falling edges */
    monitor.gpio  = 146;
    monitor.onOff = 1;
    monitor.edgeType = GPIO_EventBothEdges;
    monitor.debounceMilliSec = 0;

    if (ioctl(fd, GPIO_EVENT_IOCTL_MONITOR_GPIO, &monitor)) {
        fprintf(stderr, "failed to set gpio monitor\n");
        return 1;
    }
    printf("monitoring rising and falling edge for gpio%d\n", monitor.gpio);

    /* sit in an infinite loop querying for gpio event status */
    for (;;) {
        /* wait for IO to become ready using select */
        for (;;) {
            FD_ZERO(&rdset);
            FD_SET(fd, &rdset);

            tv.tv_sec = 1;
            tv.tv_usec = 0;

            rc = select(fd + 1, &rdset, NULL, NULL, &tv);
            if (rc < 0) {
                fprintf(stderr, "select call failed\n");
            }
            else if (rc > 0) {
                /* data is available */
                break;
            }
            else {
                printf(".");
                fflush(stdout);
            }
        }

        /* read the GPIO event structure as a binary object */
        if (sizeof(event) != read(fd, &event, sizeof(event))) {
            fprintf(stderr, "read failed: unexpected number of bytes\n");
            continue;
        }

        switch (event.edgeType)
        {
        case GPIO_EventRisingEdge:
            last_usec = event.time.tv_usec;
            break;
        case GPIO_EventFallingEdge:
            if (last_usec != 0) {
                int delta = event.time.tv_usec - last_usec;
                printf("gpio%d delta %d\n", event.gpio, delta);
            }
            last_usec = 0;
            break;
        default:
            fprintf(stderr, "unexpected case statement\n");
            break;
        }
    }

    close(fd);
    return 0;
}

