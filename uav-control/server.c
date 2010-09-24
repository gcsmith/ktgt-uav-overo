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
    static int rssi = 0;
    int new_rssi;
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

    new_rssi = stats.qual.level;
    if (!(stats.qual.updated & IW_QUAL_DBM)) {
        // convert to dBm
        new_rssi += 0x100;
    }

    if (0 != new_rssi) {
        rssi = new_rssi;
    }

    // fprintf(stderr, "read wlan0 rssi = %d\n", new_rssi);
    return rssi;
}

