// -----------------------------------------------------------------------------
// File:    gpio_event.c
// Authors: Garrett Smith
// Created: 09-18-2010
//
// Definitions for communication between uav_control and gpio input devices.
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
    int running;                    // keep track of subsystem enabled state
    int fd;                         // file descriptor for /dev/gpio-event
    pthread_t thread;               // thread to monitor gpio-event activity 
    gpio_event_t *gpio[GPIO_COUNT]; // descriptors for each gpio pin
} gpio_globals_t;

static gpio_globals_t globals = { 0 };

// -----------------------------------------------------------------------------
static void *gpio_thread(void *pargs)
{
    gpio_globals_t *data = (gpio_globals_t *)pargs;
    gpio_event_t *pevent = NULL;
    struct timeval tv;
    GPIO_Event_t event;
    fd_set rdset;
    int rc, i, j, sorted_samples[GPIO_SAMPLES];

    while (globals.running) {
        // wait for IO to become ready using select
        for (;;) {
            FD_ZERO(&rdset);
            FD_SET(data->fd, &rdset);

            tv.tv_sec = 1;
            tv.tv_usec = 0;

            rc = select(data->fd + 1, &rdset, NULL, NULL, &tv);
            if (rc < 0) {
                syslog(LOG_ERR, "select call failed\n");
            }
            else if (rc > 0) {
                // data is available
                break;
            }
            else {
                // we timed out - print out an indicator and move on
                fprintf(stderr, ".");
            }
        }

        // read the GPIO event structure as a binary object
        if (sizeof(event) != read(data->fd, &event, sizeof(event))) {
            syslog(LOG_ERR, "read failed: unexpected number of bytes\n");
            continue;
        }

        pevent = globals.gpio[event.gpio];
        if (!pevent || !pevent->enabled) {
            continue;
        }

        switch (event.edgeType) {
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

                // insert the new sample at the current location, update ptr
                pevent->samples[pevent->samp_idx] = delta;
                if (++pevent->samp_idx >= GPIO_SAMPLES)
                    pevent->samp_idx = 0;

#if 0
                for (i = 0; i < GPIO_SAMPLES; i++)
                    mean += pevent->samples[i];
                mean >>= GPIO_SHIFT;
                sq = (mean - delta) * (mean - delta);

                if (sq > 10000000) {
                    //fprintf(stderr, "skipping - %8d %8d %8d\n", delta, mean, sq);
                }
                else
                    pevent->pulsewidth = delta;
#else
                sorted_samples[0] = pevent->samples[0];
                for (i = 1; i < GPIO_SAMPLES; i++) {
                    int value = pevent->samples[i];
                    j = i - 1;
                    while (j >= 0 && sorted_samples[j] > value) {
                        sorted_samples[j + 1] = sorted_samples[j];
                        --j;
                    }
                    sorted_samples[j + 1] = value;
                }
                pevent->pulsewidth = sorted_samples[GPIO_SAMPLES >> 1];
#endif

                pevent->num_samples++;
                pthread_cond_broadcast(&pevent->cond);
                pthread_mutex_unlock(&pevent->lock);
            }
            pevent->last_usec = 0;
            break;
        default:
            syslog(LOG_ERR, "unexpected case statement\n");
            break;
        }
    }

    pthread_exit(NULL);
}

// -----------------------------------------------------------------------------
int gpio_event_init(void)
{
    void *arg;
    int rc;

    // don't allow multiple calls to gpio_event_init
    if (globals.running) {
        syslog(LOG_INFO, "attempting to call gpio_event_init multiple times\n");
        return 0;
    }

    // clear out the gpio event list
    memset(globals.gpio, 0, sizeof(gpio_event_t *) * GPIO_COUNT);

    // open the gpio-event device node
    globals.fd = open("/dev/gpio-event", 0);
    if (globals.fd < 0) {
        syslog(LOG_ERR, "failed to open /dev/gpio-event\n");
        return 0;
    }
    syslog(LOG_INFO, "successfully opened /dev/gpio-event for reading\n");

    // set read mode to binary (default is ascii)
    ioctl(globals.fd, GPIO_EVENT_IOCTL_SET_READ_MODE, GPIO_EventReadModeBinary);

    // kick off the event monitoring thread for all gpio pins
    globals.running = 1;
    arg = (void *)&globals;
    if (0 != (rc = pthread_create(&globals.thread, NULL, gpio_thread, arg))) {
        syslog(LOG_ERR, "error creating serial thread (%d)", rc);
        return 0;
    }

    return 1;
}

// -----------------------------------------------------------------------------
void gpio_event_shutdown(void)
{
    // don't allow shutdown if we didn't call gpio_event_init prior
    if (!globals.running) {
        syslog(LOG_INFO, "attempting to shutdown gpio_event prior to init\n");
        return;
    }

    globals.running = 0;
    pthread_cancel(globals.thread);

    if (globals.fd >= 0) {
        syslog(LOG_INFO, "closing /dev/gpio-event...\n");
        close(globals.fd);
    }
}

// -----------------------------------------------------------------------------
int gpio_event_attach(gpio_event_t *event, int gpio)
{
    GPIO_EventMonitor_t monitor;
    int rc;

    // zero out the entire structure... just in case
    memset(event, 0, sizeof(gpio_event_t));

    // initialize monitor for this gpio, detect both rising/falling edges
    monitor.gpio = gpio;
    monitor.onOff = 1;
    monitor.edgeType = GPIO_EventBothEdges;
    monitor.debounceMilliSec = 0;

    event->gpio = gpio;
    event->enabled = 1;
    event->num_samples = 0;
    event->last_sec = 0;
    event->last_usec = 0;
    globals.gpio[gpio] = event;

    if (ioctl(globals.fd, GPIO_EVENT_IOCTL_MONITOR_GPIO, &monitor)) {
        syslog(LOG_ERR, "failed to set gpio monitor\n");
        return 0;
    }
    syslog(LOG_INFO, "monitoring activity for gpio%d\n", monitor.gpio);

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
int gpio_event_read(gpio_event_t *event, access_mode_t mode)
{
    int pulse = 0;
    pthread_mutex_lock(&event->lock);

    switch (mode) {
    case ACCESS_ASYNC:
        // access in an asynchronous (non-blocking) fashion
        pulse = event->pulsewidth;
        break;
    case ACCESS_SYNC:
        // access in a synchronous (blocking) fashion
        pthread_cond_wait(&event->cond, &event->lock);
        pulse = event->pulsewidth;
        break;
    default:
        // just return zero for invalid accesses
        pulse = 0;
        syslog(LOG_ERR, "gpio_event_read: invalid access mode");
        break;
    }

    pthread_mutex_unlock(&event->lock);
    return pulse;
}

// -----------------------------------------------------------------------------
int gpio_event_set_filter(gpio_event_t *event, int samples)
{
    // TODO: implement me
    pthread_mutex_lock(&event->lock);
    event->avg_len = samples;
    pthread_mutex_unlock(&event->lock);
    return 1;
}

// -----------------------------------------------------------------------------
int gpio_event_get_filter(gpio_event_t *event)
{
    int rval;
    pthread_mutex_lock(&event->lock);
    rval = event->avg_len;
    pthread_mutex_unlock(&event->lock);
    return rval;
}

