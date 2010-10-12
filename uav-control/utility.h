// -----------------------------------------------------------------------------
// Garrett Smith 2010
// -----------------------------------------------------------------------------

#ifndef _UAV_UTILITY_IMU__H_
#define _UAV_UTILITY_IMU__H_

#include <stdint.h>

#define MIN(a, b)   (((a) < (b)) ? (a) : (b))
#define MAX(a, b)   (((a) > (b)) ? (a) : (b))

#define MIN3(a,b,c) ((a)<(b) ? ((a)<(c) ? (a) : (c)) : ((b)<(c) ? (b) : (c)))
#define MAX3(a,b,c) ((a)>(b) ? ((a)>(c) ? (a) : (c)) : ((b)>(c) ? (b) : (c)))

#define ELEMENTS(x) (sizeof(x) / sizeof((x)[0]))

typedef float real_t;

typedef struct client_info
{
    int fd;                 // file handle from socket accept()
    pthread_mutex_t lock;   // lock to synchronize packet sends
} client_info_t;

void daemonize(void);

int adc_open_channels(void);

void adc_close_channels(void);

int read_vbatt(void);

uint32_t read_wlan_rssi(int sock);

int recv_packet(client_info_t *client, uint32_t *cmd_buffer);

size_t send_packet(client_info_t *client, uint32_t *cmd_buffer, size_t length);

size_t send_simple_packet(client_info_t *client, uint32_t command);

void close_client(client_info_t *client);

#endif // _UAV_UTILITY_IMU__H_

