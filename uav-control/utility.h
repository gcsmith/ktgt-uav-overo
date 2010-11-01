// -----------------------------------------------------------------------------
// File:    utility.h
// Authors: Garrett Smith, Kevin Macksamie, Timothy Miller
// Created: 10-02-2010
//
// General purpose helper functions and macros used throughout uav_control.
// -----------------------------------------------------------------------------

#ifndef _UAV_UTILITY_IMU__H_
#define _UAV_UTILITY_IMU__H_

#include <stdint.h>
#include <time.h>
#include <math.h>
#include <pthread.h>

#ifndef PI
#define PI ((real_t)3.141592653589793)
#endif

#define BATTERY_MIN         60.0f
#define BATTERY_CHECK_TICKS 15
#define BATTERY_PRINT_TICKS 5

// return the smallest/largest of the two specified values
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

// return the smallest/largest of the three specified values
#define MIN3(a,b,c) ((a)<(b) ? ((a)<(c) ? (a) : (c)) : ((b)<(c) ? (b) : (c)))
#define MAX3(a,b,c) ((a)>(b) ? ((a)>(c) ? (a) : (c)) : ((b)>(c) ? (b) : (c)))

// clamp the value x within the bounds [l, u]
#define CLAMP(x, l, h) (((x) > (h)) ? (h) : (((x) < (l)) ? (l) : (x)))

// return the number of elements of a statically declared array
#define ELEMENTS(x) (sizeof(x) / sizeof((x)[0]))

// convert between degrees and radians
#define DEG_TO_RAD(x)  ((x) * (PI / 180.0f))
#define RAD_TO_DEG(x)  ((x) * (180.0f / PI))

// allow us to toggle the precision of all internal floating point algorithms
typedef float real_t;
typedef struct timespec timespec_t;
typedef uint8_t u8;
typedef uint16_t u16;

typedef struct client_info
{
    int fd;                 // file handle from socket accept()
    pthread_mutex_t lock;   // lock to synchronize packet sends
} client_info_t;

typedef struct cpu_info
{
    int user;
    int nice;
    int system;    
    int idle;
    int hint;
    int sint;
    int percentage;
} cpu_info_t;

typedef enum access_mode {
    ACCESS_ASYNC,   // asynchronous access (non-blocking)
    ACCESS_SYNC,    // synchronous access (blocking)
} access_mode_t;

// turn the calling process into a daemon process
void daemonize(void);

// compute the delta between two timespecs
int timespec_diff(const timespec_t *t0, const timespec_t *t1, timespec_t *td);

// compute the delta between two timespecs, return as a scalar (microseconds)
long timespec_delta(struct timespec *t0, struct timespec *t1);

// initialize analog-to-digital channels for reading
int adc_open_channels(void);

// close analog-to-digital channels
void adc_close_channels(void);

// read the current battery life from the adc
int read_vbatt(void);

// read the current wireless link signal strength
uint32_t read_wlan_rssi(int sock);

// receive an arbitrary length packet from the specified client
int recv_packet(client_info_t *client, uint32_t *cmd_buffer);

// send an arbitrary length packet to the specified client
size_t send_packet(client_info_t *client, uint32_t *cmd_buffer, size_t length);

// send a simple fixed length command packet to the specified client
size_t send_simple_packet(client_info_t *client, uint32_t command);

// close the socket connection to the specified client
void close_client(client_info_t *client);

// Get cpu utilization
int get_cpu_utilization(void);

#endif // _UAV_UTILITY_IMU__H_

