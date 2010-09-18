// -----------------------------------------------------------------------------
// Definitions for communication between gumstix and ultrasonic range finder.
// Garrett Smith 2010
// -----------------------------------------------------------------------------

#ifndef _UAV_ULTRASONIC__H_
#define _UAV_ULTRASONIC__H_

#include <pthread.h>

typedef struct ultrasonic_data
{
    int fd;
    pthread_t hthrd;
    pthread_mutex_t lock;
    unsigned int height;
    unsigned long sample;
} ultrasonic_data_t;

int ultrasonic_init(int gpio, ultrasonic_data_t *data);
void ultrasonic_shutdown(ultrasonic_data_t *data);

#endif // _UAV_ULTRASONIC__H_

