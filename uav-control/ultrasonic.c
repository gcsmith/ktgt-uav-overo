// -----------------------------------------------------------------------------
// Implementation of ultrasonic (PWM) communication.
// Garrett Smith 2010
// -----------------------------------------------------------------------------

#include <sys/ioctl.h>
#include <syslog.h>
#include <fcntl.h>
#include <ctype.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "ultrasonic.h"
#include "gpio-event-drv.h"

// -----------------------------------------------------------------------------
static void *gpio_rd_thread(void *thread_args)
{
    ultrasonic_data_t *data = (ultrasonic_data_t *)thread_args;
    struct timeval tv;
    GPIO_Event_t event;
    fd_set rdset;
    int rc = 0, last_sec = 0, last_usec = 0;

    while (data->running) {
        // wait for IO to become ready using select
        for (;;) {
            FD_ZERO(&rdset);
            FD_SET(data->fd, &rdset);

            tv.tv_sec = 1;
            tv.tv_usec = 0;

            rc = select(data->fd + 1, &rdset, NULL, NULL, &tv);
            if (rc < 0) {
                fprintf(stderr, "select call failed\n");
            }
            else if (rc > 0) {
                // data is available
                break;
            }
            else {
                printf(".");
                fflush(stdout);
            }
        }

        // read the GPIO event structure as a binary object
        if (sizeof(event) != read(data->fd, &event, sizeof(event))) {
            fprintf(stderr, "read failed: unexpected number of bytes\n");
            continue;
        }

        switch (event.edgeType)
        {
        case GPIO_EventRisingEdge:
            // start timing on the rising edge of the pwm
            last_sec = event.time.tv_sec;
            last_usec = event.time.tv_usec;
            break;
        case GPIO_EventFallingEdge:
            // stop timing on the falling edge of the pwm
            if (last_usec != 0) {
                int delta = event.time.tv_usec - last_usec;
                if (last_sec < event.time.tv_sec)
                {
                    // account for microsecond overflow
                    delta += 1000000;
                }
                pthread_mutex_lock(&data->lock);
                // taken from maxbotix from spec: 147 us == 1 inch
                data->height = delta / 147;
                // printf("gpio%d: delta: %d\n", event.gpio, delta);
                pthread_mutex_unlock(&data->lock);
            }
            last_usec = 0;
            break;
        default:
            fprintf(stderr, "unexpected case statement\n");
            break;
        }
    }

    pthread_exit(NULL);
}

// -----------------------------------------------------------------------------
int ultrasonic_init(int gpio, ultrasonic_data_t *data)
{
    GPIO_EventMonitor_t monitor;
    void *arg;
    int rc;

    memset(data, 0, sizeof(ultrasonic_data_t));
    data->running = 1;
    data->gpio = gpio;

    // open the gpio-event device node
    data->fd = open("/dev/gpio-event", 0);
    if (data->fd < 0) {
        fprintf(stderr, "failed to open /dev/gpio-event\n");
        return 0;
    }
    printf("successfully opened /dev/gpio-event for reading\n");

    // set read mode to binary (default is ascii)
    ioctl(data->fd, GPIO_EVENT_IOCTL_SET_READ_MODE, GPIO_EventReadModeBinary);

    // initialize monitor for gpio146, detect both rising/falling edges
    monitor.gpio  = gpio;
    monitor.onOff = 1;
    monitor.edgeType = GPIO_EventBothEdges;
    monitor.debounceMilliSec = 0;

    if (ioctl(data->fd, GPIO_EVENT_IOCTL_MONITOR_GPIO, &monitor)) {
        fprintf(stderr, "failed to set gpio monitor\n");
        return 0;
    }
    printf("monitoring rising and falling edge for gpio%d\n", monitor.gpio);

    if (0 != (rc = pthread_mutex_init(&data->lock, NULL))) {
        syslog(LOG_ERR, "error creating serial mutex (%d)", rc);
        return 0;
    }

    arg = (void *)data;
    if (0 != (rc = pthread_create(&data->thread, NULL, gpio_rd_thread, arg))) {
        syslog(LOG_ERR, "error creating serial thread (%d)", rc);
        return 0;
    }

    return 1;
}

// -----------------------------------------------------------------------------
void ultrasonic_shutdown(ultrasonic_data_t *data)
{
    GPIO_EventMonitor_t monitor;

    data->running = 0;
    pthread_cancel(data->thread);
    pthread_mutex_destroy(&data->lock);

    monitor.gpio = data->gpio;
    monitor.onOff = 0;
    ioctl(data->fd, GPIO_EVENT_IOCTL_MONITOR_GPIO, &monitor);

    close(data->fd);
}

