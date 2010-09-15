// -----------------------------------------------------------------------------
// Test for measuring RSSI signal strength of wireless interface.
// Garrett Smith 2010
// -----------------------------------------------------------------------------

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/wireless.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char *argv[])
{
    int sock;

    struct iw_statistics stats;
    struct iwreq req = {
        .ifr_name = "wlan0",
        .u.data = {
            .length = sizeof(struct iw_statistics),
            .pointer = &stats,
            .flags = 1
        }
    };

    if (0 > (sock = socket(AF_INET, SOCK_DGRAM, 0))) {
        perror("failed to create datagram socket");
        return EXIT_FAILURE;
    }

    for (;;) {
        if (0 > ioctl(sock, SIOCGIWSTATS, &req)) {
            perror("error invoking SIOCGIWSTATS ioctl");
            return EXIT_FAILURE;
        }

        printf("Signal level%s is %d%s.\n",
                (stats.qual.updated & IW_QUAL_DBM ? " (in dBm)" :""),
                stats.qual.level,
                (stats.qual.updated & IW_QUAL_LEVEL_UPDATED ? " (updated)" :""));

        sleep(1);
    }

    close(sock);
    return 0;
}

