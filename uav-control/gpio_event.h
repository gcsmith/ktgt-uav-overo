// -----------------------------------------------------------------------------
// Definitions for communication between gumstix and GPIO PWM signals.
// Garrett Smith 2010
// -----------------------------------------------------------------------------

#ifndef _UAV_GPIO_EVENT__H_
#define _UAV_GPIO_EVENT__H_

#include <pthread.h>

#define GPIO_COUNT 256

typedef struct gpio_event
{
    pthread_mutex_t lock;   // mutex lock for data access
    pthread_cond_t cond;    // condition variable for update notification
    int gpio;               // gpio pin number for this event
    int enabled;            // is this pin being actively monitored?
    int pulsewidth;         // pulse width of PWM signal on GPIO
    int sample;             // current sample index
    int last_sec;
    int last_usec;
} gpio_event_t;

int gpio_event_init();
int gpio_event_attach(gpio_event_t *event, int gpio);
int gpio_event_sync_read(gpio_event_t *event);
int gpio_event_read(gpio_event_t *event);
void gpio_event_detach(gpio_event_t *event);
void gpio_event_shutdown();

#endif // _UAV_GPIO_EVENT__H_

