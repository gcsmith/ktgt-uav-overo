// -----------------------------------------------------------------------------
// File:    razor_imu.h
// Authors: Garrett Smith, Kevin Macksamie
// Created: 09-18-2010
// 
// Definitions for communication between gumstix and 9DOF IMU.
// -----------------------------------------------------------------------------

#ifndef _UAV_RAZOR_IMU__H_
#define _UAV_RAZOR_IMU__H_

#include <termios.h>
#include <pthread.h>
#include "utility.h"

#define IMU_DATA_ROLL  0
#define IMU_DATA_PITCH 1
#define IMU_DATA_YAW   2

typedef struct imu_data
{
    int running;
    int fd;                 // file descriptor for serial device
    pthread_t thread;       // serial thread object
    pthread_mutex_t lock;   // critical section for serial data
    pthread_cond_t cond;    // condition variable for update notification
    real_t angles[3];       // orientation angles from IMU
    real_t moving_sum[3];
    real_t *samples;
    real_t inv_avg_len;
    unsigned int avg_len;
    unsigned int avg_idx;
    unsigned long sample;
} imu_data_t;

int imu_init(const char *device, int baud, imu_data_t *data);
int imu_read_angles(imu_data_t *imu, float *angles, access_mode_t mode);
int imu_set_avg_filter(imu_data_t *data, unsigned int avg_len);
int imu_get_avg_filter(imu_data_t *data);
void imu_shutdown(imu_data_t *data);

#endif // _UAV_RAZOR_IMU__H_

