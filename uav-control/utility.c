// -----------------------------------------------------------------------------
// Misc utility routines.
// Garrett Smith 2010
// -----------------------------------------------------------------------------

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/wireless.h>
#include <syslog.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <pthread.h>
#include <string.h>

#include "utility.h"
#include "uav_protocol.h"

typedef uint8_t u8;
typedef uint16_t u16;

#include "twl4030-madc.h"

struct twl4030_madc_user_parms parms;
int adc_fd;

#define ADC_INPUT_RANGE 2.5f
#define ADC_VBATT_PORT  4

#define VBATT_MIN       1.0f // needs to be verified as the minimum
#define VBATT_MAX       2.0f
#define VBATT_RANGE     (VBATT_MAX - VBATT_MIN)

// -----------------------------------------------------------------------------
// Daemonize the process by forking from init and chdir-ing to /.
void daemonize(void)
{
    pid_t pid, sid;

    pid = fork();
    if (pid < 0) {
        // failed to fork
        syslog(LOG_ERR, "failed to fork process");
        exit(EXIT_FAILURE);
    }
    else if (pid > 0) {
        // parent process - terminate
        exit(EXIT_SUCCESS);
    }

    umask(0);

    sid = setsid();
    if (sid < 0) {
        syslog(LOG_ERR, "failed to execute setsid()");
        exit(EXIT_FAILURE);
    }

    if (chdir("/") < 0) {
        syslog(LOG_ERR, "failed to chdir() to /");
        exit(EXIT_FAILURE);
    }

    close(STDIN_FILENO);
    close(STDOUT_FILENO);
    close(STDERR_FILENO);
}

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
        syslog(LOG_ERR, "error invoking SIOCGIWSTATS ioctl");
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

    return rssi;
}

// -----------------------------------------------------------------------------
int adc_open_channels(void)
{
    adc_fd = open("/dev/twl4030-madc", O_RDWR | O_NONBLOCK);
    return adc_fd;
}

// -----------------------------------------------------------------------------
void adc_close_channels(void)
{
    close(adc_fd);
}

// -----------------------------------------------------------------------------
int read_vbatt(void)
{
    float voltage;

    memset(&parms, 0, sizeof(parms));
    parms.channel = ADC_VBATT_PORT;

    ioctl(adc_fd, TWL4030_MADC_IOCX_ADC_RAW_READ, &parms);
    voltage = ((unsigned int)parms.result) / 1024.f * ADC_INPUT_RANGE;

    if (voltage > VBATT_MAX)
        voltage = VBATT_MAX;
    if (voltage < VBATT_MIN)
        voltage = VBATT_MIN;

    voltage = (voltage - VBATT_MIN) / VBATT_RANGE;
    voltage = (int)(voltage * 100);
    
    return (int)voltage;
}

// -----------------------------------------------------------------------------
// Receive a single packet. Blocks until the entire message has been consumed.
int recv_packet(client_info_t *client, uint32_t *cmd_buffer)
{
    // read in the packet header to determine length
    int rc, bytes_left = PKT_BASE_LENGTH;
    char *ptr = (char *)cmd_buffer;
    while (bytes_left > 0) {
        if (1 > (rc = recv(client->fd, (void *)ptr, bytes_left, 0))) {
            // assume client disconnected for now
            return 0;
        }
        bytes_left -= rc;
        ptr += rc; }

    // read in the rest of the packet
    bytes_left = cmd_buffer[PKT_LENGTH] - PKT_BASE_LENGTH;
    if (bytes_left > PKT_BUFF_LEN) {
        // disconnect the client if the packet size exceeds the buffer size
        // there's no reason we should ever exceed the maximum limit
        syslog(LOG_ERR, "encountered invalid packet size (%d)\n", bytes_left);
        return 0;
    }

    while (bytes_left > 0) {
        if (1 > (rc = recv(client->fd, (void *)ptr, bytes_left, 0))) {
            // assume client disconnected for now
            return 0;
        }
        bytes_left -= rc;
        ptr += rc;
    }

    return 1;
}

// -----------------------------------------------------------------------------
// Send a single packet of the specified length (in units of bytes).
size_t send_packet(client_info_t *client, uint32_t *cmd_buffer, size_t length)
{
    size_t bytes_written = 0;
    pthread_mutex_lock(&client->lock);
    bytes_written = send(client->fd, (void *)cmd_buffer, length, 0);
    pthread_mutex_unlock(&client->lock);
    return bytes_written;
}

// -----------------------------------------------------------------------------
// Send a simple packet without any arguments.
size_t send_simple_packet(client_info_t *client, uint32_t command)
{
    size_t bytes_written = 0;
    uint32_t cmd_buffer[] = { command, PKT_BASE_LENGTH };
    pthread_mutex_lock(&client->lock);
    bytes_written = send(client->fd, (void *)cmd_buffer, sizeof(cmd_buffer), 0);
    pthread_mutex_unlock(&client->lock);
    return bytes_written;
}

// -----------------------------------------------------------------------------
void close_client(client_info_t *client)
{
    pthread_mutex_lock(&client->lock);
    if (client->fd < 0) {
        // we should never get here...
        syslog(LOG_ERR, "calling close_client on closed client\n");
    }
    else {
        shutdown(client->fd, SHUT_RDWR);
        close(client->fd);
        client->fd = -1;
    }
    pthread_mutex_unlock(&client->lock);
}

