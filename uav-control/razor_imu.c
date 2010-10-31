// -----------------------------------------------------------------------------
// File:    razor_imu.c
// Authors: Garrett Smith, Kevin Macksamie
// Created: 09-18-2010
// 
// Definitions for communication between gumstix and 9DOF IMU.
// -----------------------------------------------------------------------------

#include <fcntl.h>
#include <syslog.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "razor_imu.h"

// -----------------------------------------------------------------------------
static inline void calc_moving_avg(imu_data_t *data, float *new_samples)
{
    // subtract out the value to remove from the moving sum
    real_t *samples = &data->samples[data->avg_idx * 3];
    data->moving_sum[0] -= samples[0];
    data->moving_sum[1] -= samples[1];
    data->moving_sum[2] -= samples[2];

    // add in the new samples to the moving sum
    data->moving_sum[0] += (samples[0] = new_samples[0]);
    data->moving_sum[1] += (samples[1] = new_samples[1]);
    data->moving_sum[2] += (samples[2] = new_samples[2]);

    // wrap around the sample index into our ring buffer if necessary
    if (++data->avg_idx >= data->avg_len)
        data->avg_idx = 0;

    // calculate the average of each channel
    data->angles[0] = data->moving_sum[0] * data->inv_avg_len;
    data->angles[1] = data->moving_sum[1] * data->inv_avg_len;
    data->angles[2] = data->moving_sum[2] * data->inv_avg_len;
}

// -----------------------------------------------------------------------------
static inline void calc_low_pass_filter(imu_data_t *data, float *new_samples)
{
    // calculate output angles using running-sum low pass filter
    data->angles[0] = .1f * new_samples[0] + .9f * data->moving_sum[0];
    data->angles[1] = .1f * new_samples[1] + .9f * data->moving_sum[1];
    data->angles[2] = .1f * new_samples[2] + .9f * data->moving_sum[2];

    // store the current unfiltered samples for the next iteration
    data->moving_sum[0] = new_samples[0];
    data->moving_sum[1] = new_samples[1];
    data->moving_sum[2] = new_samples[2];
}

// -----------------------------------------------------------------------------
static void *imu_rd_thread(void *thread_args)
{
    imu_data_t *data = (imu_data_t *)thread_args;
    char buff[128], num_buff[16], *ptr;
    int nb, i, data_idx = 0, num_idx = 0;
    real_t temp_data[3];

    // XXX: rewrite me -- I can barely make sense of what I wrote here
    while (data->running) {
        ptr = buff;
        while (0 < (nb = read(data->fd, ptr, buff + sizeof(buff) - ptr))) {
            // process characters as they arrive
            for (i = 0; i < nb; i++) {
                switch (ptr[i]) {
                case '\n':
                    num_buff[num_idx++] = '\0';
                    temp_data[data_idx++] = (real_t)strtof(num_buff, NULL);
                    if (data_idx == 3) {
                        pthread_mutex_lock(&data->lock);

                        if (data->avg_len <= 0) {
                            // store the immediate sample values
                            data->angles[0] = temp_data[0];
                            data->angles[1] = temp_data[1];
                            data->angles[2] = temp_data[2];
                        }
                        else {
                            // calculate a moving average of the samples
                            // calc_moving_avg(data, temp_data);
                            calc_low_pass_filter(data, temp_data);
                        }

                        //fprintf(stderr, "imu ( %f , %f , %f )\n",
                        //        data->angles[0],
                        //        data->angles[1],
                        //        data->angles[2]);

                        data->sample++;
                        pthread_cond_broadcast(&data->cond);
                        pthread_mutex_unlock(&data->lock);
                        data_idx = 0;
                    }
                    num_idx = 0;
                    break;
                case '!': case 'A': case 'N': case 'G': case ':':
                    data_idx = 0;
                    num_idx = 0;
                    break;
                case ',':
                    num_buff[num_idx++] = '\0';
                    temp_data[data_idx++] = strtof(num_buff, NULL);
                    num_idx = 0;
                    break;
                default:
                    // number or prefix, keep going...
                    num_buff[num_idx++] = ptr[i];
                    break;
                }
            }
            ptr += nb;
        }
    }

    pthread_exit(NULL);
}

// -----------------------------------------------------------------------------
int imu_init(const char *device, int baud, imu_data_t *data)
{
    int rc;
    void *arg;
    struct termios term_opt;

    memset((char *)data, 0, sizeof(imu_data_t));
    data->running = 1;

    // attempt to open the specified serial device for binary RW
    if (0 > (data->fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY))) {
        syslog(LOG_ERR, "unable to open IMU UART");
        return 0;
    }

    // clear flags and get current attributes
    fcntl(data->fd, F_SETFL, 0);
    tcgetattr(data->fd, &term_opt);

    // set the baud rate, local line ownership, and enable receiver
    cfsetispeed(&term_opt, baud);
    cfsetospeed(&term_opt, baud);
    term_opt.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(data->fd, TCSANOW, &term_opt);

    if (0 != (rc = pthread_mutex_init(&data->lock, NULL))) {
        syslog(LOG_ERR, "error creating serial mutex (%d)", rc);
        return 0;
    }

    arg = (void *)data;
    if (0 != (rc = pthread_create(&data->thread, NULL, imu_rd_thread, arg))) {
        syslog(LOG_ERR, "error creating serial thread (%d)", rc);
        return 0;
    }

    if (0 != (rc = pthread_cond_init(&data->cond, NULL))) {
        syslog(LOG_ERR, "error creating imu condition (%d)", rc);
        return 0;
    }

    return 1;
}

// -----------------------------------------------------------------------------
int imu_read_angles(imu_data_t *imu, float *angles, access_mode_t mode)
{
    pthread_mutex_lock(&imu->lock);

    switch (mode) {
    case ACCESS_ASYNC:
        // access in an asynchronous (non-blocking) fashion
        break;
    case ACCESS_SYNC:
        // access in a synchronous (blocking) fashion
        pthread_cond_wait(&imu->cond, &imu->lock);
        break;
    default:
        // zero out the angles if invalid access mode encountered
        pthread_mutex_unlock(&imu->lock);
        memset(angles, 0, sizeof(float) * 3);
        syslog(LOG_ERR, "imu_read_angles: invalid access mode\n");
        return 0;
    }

    angles[IMU_YAW]   = imu->angles[IMU_YAW];
    angles[IMU_PITCH] = imu->angles[IMU_PITCH];
    angles[IMU_ROLL]  = imu->angles[IMU_ROLL];

    pthread_mutex_unlock(&imu->lock);
    return 1;
}

// -----------------------------------------------------------------------------
int imu_set_avg_filter(imu_data_t *data, unsigned int avg_len)
{
    pthread_mutex_lock(&data->lock);
    
    // clear the previous average buffer, if there was one
    if (data->samples) {
        free(data->samples);
        data->samples = NULL;
    }

    // reset to initial values, as we're starting over
    data->avg_len = avg_len;
    data->avg_idx = 0;
    data->moving_sum[0] = 0.0f;
    data->moving_sum[1] = 0.0f;
    data->moving_sum[2] = 0.0f;

    if (!avg_len) {
        pthread_mutex_unlock(&data->lock);
        syslog(LOG_INFO, "avg_len is zero, disabling imu filter\n");
        return 1;
    }

    if (avg_len > 64) {
        data->avg_len = 0;
        pthread_mutex_unlock(&data->lock);
        syslog(LOG_ERR, "imu averaging filter may not exceed 64 samples\n");
        return 0;
    }

    // set the new parameters and allocate the running average buffer
    data->samples = (real_t *)calloc(avg_len * 3, sizeof(real_t));
    data->inv_avg_len = 1.0f / (real_t)avg_len;

    pthread_mutex_unlock(&data->lock);
    return 1;
}

// -----------------------------------------------------------------------------
int imu_get_avg_filter(imu_data_t *data)
{
    int rval;
    pthread_mutex_lock(&data->lock);
    rval = data->avg_len;
    pthread_mutex_unlock(&data->lock);
    return rval;
}

// -----------------------------------------------------------------------------
void imu_shutdown(imu_data_t *data)
{
    pthread_cancel(data->thread);
    pthread_mutex_destroy(&data->lock);
    pthread_cond_destroy(&data->cond);
    close(data->fd);
}

