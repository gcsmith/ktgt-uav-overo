// -----------------------------------------------------------------------------
// File:    gpio_event.h
// Authors: Garrett Smith
// Created: 09-18-2010
//
// Definitions for communication between uav_control and gpio input devices.
// -----------------------------------------------------------------------------

#ifndef _UAV_GPIO_EVENT__H_
#define _UAV_GPIO_EVENT__H_

#include <pthread.h>
#include "utility.h"

#define GPIO_COUNT 256
#define GPIO_SHIFT 2
#define GPIO_SAMPLES (1 << GPIO_SHIFT)

typedef struct gpio_event
{
    pthread_mutex_t lock;   // mutex lock for data access
    pthread_cond_t cond;    // condition variable for update notification
    int gpio;               // gpio pin number for this event
    int enabled;            // is this pin being actively monitored?
    int pulsewidth;         // pulse width of PWM signal on GPIO
    int num_samples;        // current sample index
    int last_sec;           // last second counter for this event
    int last_usec;          // last microsecond counter for this event
    int avg_len;            // number of samples to average
    int samples[GPIO_SAMPLES];
    int samp_idx;
} gpio_event_t;

// initialize the gpio event subsystem
int gpio_event_init();

// shutdown the gpio event subsystem
void gpio_event_shutdown();

// attach an event to the specified gpio device index
int gpio_event_attach(gpio_event_t *event, int gpio);

// detach and destroy the specified gpio event structure
void gpio_event_detach(gpio_event_t *event);

// perform an asynchronous (but thread safe) read
int gpio_event_read(gpio_event_t *event, access_mode_t mode);

// set the number of samples for the averaging filter
int gpio_event_set_filter(gpio_event_t *event, int samples);

// get the number of samples for the averaging filter
int gpio_event_get_filter(gpio_event_t *event);

#endif // _UAV_GPIO_EVENT__H_

