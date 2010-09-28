// -----------------------------------------------------------------------------
// User space interface to gpio event driver.
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
#include "gpio_event.h"
#include "gpio-event-drv.h"

typedef struct gpio_globals
{
    int running;
    int fd;
    pthread_t thread;
    gpio_event_t *gpio[GPIO_COUNT];
} gpio_globals_t;

static gpio_globals_t globals;

// -----------------------------------------------------------------------------
static void *gpio_thread(void *pargs)
{
    gpio_globals_t *data = (gpio_globals_t *)pargs;
    gpio_event_t *pevent = NULL;
    struct timeval tv;
    GPIO_Event_t event;
    fd_set rdset;
    int rc = 0;

    while (globals.running) {
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

        pevent = globals.gpio[event.gpio];
        if (!pevent || !pevent->enabled) {
            continue;
        }

        switch (event.edgeType)
        {
        case GPIO_EventRisingEdge:
            // start timing on the rising edge of the pwm
            pevent->last_sec = event.time.tv_sec;
            pevent->last_usec = event.time.tv_usec;
            break;
        case GPIO_EventFallingEdge:
            // stop timing on the falling edge of the pwm
            if (pevent->last_usec != 0) {
                int delta = event.time.tv_usec - pevent->last_usec;
                if (pevent->last_sec < event.time.tv_sec) {
                    // account for microsecond overflow
                    delta += 1000000;
                }

                // update data for this event
                pthread_mutex_lock(&pevent->lock);

                pevent->pulsewidth = delta;
                pevent->sample++;

                pthread_cond_broadcast(&pevent->cond);
                pthread_mutex_unlock(&pevent->lock);
            }
            pevent->last_usec = 0;
            break;
        default:
            fprintf(stderr, "unexpected case statement\n");
            break;
        }
    }

    pthread_exit(NULL);
}

// -----------------------------------------------------------------------------
int gpio_event_init()
{
    void *arg;
    int rc;

    // clear out the gpio event list
    memset(globals.gpio, 0, sizeof(gpio_event_t *) * GPIO_COUNT);

    // open the gpio-event device node
    globals.fd = open("/dev/gpio-event", 0);
    if (globals.fd < 0) {
        fprintf(stderr, "failed to open /dev/gpio-event\n");
        return 0;
    }
    fprintf(stderr, "successfully opened /dev/gpio-event for reading\n");

    // set read mode to binary (default is ascii)
    ioctl(globals.fd, GPIO_EVENT_IOCTL_SET_READ_MODE, GPIO_EventReadModeBinary);

    // kick off the event monitoring thread for all gpio pins
    globals.running = 1;
    arg = (void *)&globals;
    if (0 != (rc = pthread_create(&globals.thread, NULL, gpio_thread, arg))) {
        fprintf(stderr, "error creating serial thread (%d)", rc);
        return 0;
    }

    return 1;
}

// -----------------------------------------------------------------------------
int gpio_event_attach(gpio_event_t *event, int gpio)
{
    GPIO_EventMonitor_t monitor;
    int rc;

    // initialize monitor for this gpio, detect both rising/falling edges
    monitor.gpio  = gpio;
    monitor.onOff = 1;
    monitor.edgeType = GPIO_EventBothEdges;
    monitor.debounceMilliSec = 0;

    event->gpio = gpio;
    event->enabled = 1;
    event->sample = 0;
    event->last_sec = 0;
    event->last_usec = 0;
    globals.gpio[gpio] = event;

    if (ioctl(globals.fd, GPIO_EVENT_IOCTL_MONITOR_GPIO, &monitor)) {
        fprintf(stderr, "failed to set gpio monitor\n");
        return 0;
    }
    printf("monitoring rising and falling edge for gpio%d\n", monitor.gpio);

    if (0 != (rc = pthread_mutex_init(&event->lock, NULL))) {
        syslog(LOG_ERR, "error creating gpio event mutex (%d)", rc);
        return 0;
    }

    if (0 != (rc = pthread_cond_init(&event->cond, NULL))) {
        syslog(LOG_ERR, "error creating gpio event condition (%d)", rc);
        return 0;
    }

    return 1;
}

// -----------------------------------------------------------------------------
void gpio_event_detach(gpio_event_t *event)
{
    GPIO_EventMonitor_t monitor;
    monitor.gpio = event->gpio;
    monitor.onOff = 0;

    event->enabled = 0;
    pthread_mutex_destroy(&event->lock);
    pthread_cond_destroy(&event->cond);

    ioctl(globals.fd, GPIO_EVENT_IOCTL_MONITOR_GPIO, &monitor);
}

// -----------------------------------------------------------------------------
void gpio_event_shutdown()
{
    globals.running = 0;
    pthread_cancel(globals.thread);

    if (globals.fd >= 0) {
        fprintf(stderr, "closing /dev/gpio-event...\n");
        close(globals.fd);
    }
}

