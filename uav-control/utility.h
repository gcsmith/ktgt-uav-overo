// -----------------------------------------------------------------------------
// Garrett Smith 2010
// -----------------------------------------------------------------------------

#ifndef _UAV_UTILITY_IMU__H_
#define _UAV_UTILITY_IMU__H_

#include <stdint.h>

typedef float real_t;

#define MIN(a, b)   (((a) < (b)) ? (a) : (b))
#define MAX(a, b)   (((a) > (b)) ? (a) : (b))
#define LEN(x)      (sizeof(x) / sizeof((x)[0]))

uint32_t read_wlan_rssi(int sock);
void daemonize(void);

#endif // _UAV_UTILITY_IMU__H_

