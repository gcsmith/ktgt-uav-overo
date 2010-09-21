// -----------------------------------------------------------------------------
// Implementation of razor imu (serial) communication.
// Garrett Smith 2010
// -----------------------------------------------------------------------------

#include <fcntl.h>
#include <syslog.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "razor_imu.h"

// -----------------------------------------------------------------------------
static void *imu_rd_thread(void *thread_args)
{
    imu_data_t *data = (imu_data_t *)thread_args;
    char buff[128], num_buff[16], *ptr;
    int nb, i, data_idx = 0, num_idx = 0;
    float temp_data[3];

    // XXX: rewrite me -- I can barely make sense of what I wrote here
    while (data->running) {
        ptr = buff;
        while (0 < (nb = read(data->fd, ptr, buff + sizeof(buff) - ptr))) {
            // process characters as they arrive
            for (i = 0; i < nb; i++) {
                switch (ptr[i]) {
                case '\n':
                    num_buff[num_idx++] = '\0';
                    temp_data[data_idx++] = strtof(num_buff, NULL);
                    if (data_idx == 3) {
                        pthread_mutex_lock(&data->lock);
                        data->angles[0] = temp_data[0];
                        data->angles[1] = temp_data[1];
                        data->angles[2] = temp_data[2];
                        data->sample++;
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

    return 1;
}

// -----------------------------------------------------------------------------
void imu_shutdown(imu_data_t *data)
{
    pthread_cancel(data->thread);
    pthread_mutex_destroy(&data->lock);
    close(data->fd);
}

