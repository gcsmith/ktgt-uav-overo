// -----------------------------------------------------------------------------
// Implementation of network communication code (server).
// Garrett Smith 2010
// -----------------------------------------------------------------------------

#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/wireless.h>
#include "server.h"

// -----------------------------------------------------------------------------
uint32_t read_wlan_rssi(int sock)
{
    int rssi;
    struct iw_statistics stats;
    struct iwreq req = {
        .ifr_name = "wlan0",
        .u.data = {
            .length = sizeof(struct iw_statistics),
            .pointer = &stats,
            .flags = 1
        }
    };

    if (0 > ioctl(sock, SIOCGIWSTATS, &req)) {
        // perror("error invoking SIOCGIWSTATS ioctl");
        return 0;
    }

    rssi = stats.qual.level;
    if (!(stats.qual.updated & IW_QUAL_DBM)) {
        // convert to dBm
        rssi += 0x100;
    }

#if 0
    fprintf(stderr, "read wlan0 rssi = %d\n", rssi);
#endif
    return rssi;
}

