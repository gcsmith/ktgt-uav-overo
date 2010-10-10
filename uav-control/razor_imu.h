// -----------------------------------------------------------------------------
// Definitions for communication between gumstix and 9DOF IMU.
// Garrett Smith 2010
// -----------------------------------------------------------------------------

#ifndef _UAV_RAZOR_IMU__H_
#define _UAV_RAZOR_IMU__H_

#include <termios.h>
#include <pthread.h>

#define IMU_DATA_YAW   0
#define IMU_DATA_PITCH 1
#define IMU_DATA_ROLL  2

typedef struct imu_data
{
    int running;
    int fd;                 // file descriptor for serial device
    pthread_t thread;       // serial thread object
    pthread_mutex_t lock;   // critical section for serial data
    float angles[3];        // orientation angles from IMU
    unsigned long sample;
} imu_data_t;

int imu_init(const char *device, int baud, imu_data_t *data);
void imu_shutdown(imu_data_t *data);

#endif // _UAV_RAZOR_IMU__H_

