// -----------------------------------------------------------------------------
// File:    utility.c
// Authors: Garrett Smith, Kevin Macksamie, Timothy Miller
// Created: 10-05-2010
//
// General purpose helper functions and macros used throughout uav_control.
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
#include <stdio.h>

#include "utility.h"
#include "uav_protocol.h"
#include "twl4030-madc.h"

struct twl4030_madc_user_parms parms;
static int adc_fd = 0;

#define ADC_INPUT_RANGE 2.5f
#define ADC_VBATT_PORT  4

#define VBATT_MIN       1.0f // needs to be verified as the minimum
#define VBATT_MAX       2.0f
#define VBATT_RANGE     (VBATT_MAX - VBATT_MIN)

#define VBATT_HISTORY_SHIFT  4
#define VBATT_HISTORY_LENGTH (1 << VBATT_HISTORY_SHIFT)

#define CPU_HISTORY_SHIFT  4
#define CPU_HISTORY_LENGTH (1 << CPU_HISTORY_SHIFT)

// -----------------------------------------------------------------------------
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
int timespec_diff(const timespec_t *t0, const timespec_t *t1, timespec_t *td)
{
    td->tv_sec  = t1->tv_sec  - t0->tv_sec;
    td->tv_nsec = t1->tv_nsec - t0->tv_nsec;

    if (td->tv_sec >= 0 && td->tv_nsec > 0) {
        return 1;
    }
    else if (td->tv_sec > 0 && td->tv_nsec < 0) {
        td->tv_nsec += 1000000000L;
        td->tv_sec--;
        return 1;
    }
    else if (td->tv_sec < 0 && td->tv_nsec > 0) {
        td->tv_nsec -= 1000000000L;
        td->tv_sec++;
        td->tv_sec = -td->tv_sec;
        td->tv_nsec = -td->tv_nsec;
        return -1;
    }
    else if (td->tv_sec <= 0 && td->tv_nsec < 0) {
        td->tv_sec = -td->tv_sec;
        td->tv_nsec = -td->tv_nsec;
        return -1;
    }
    else /* (td->tv_sec == 0 && td->tv_nsec == 0) */ {
        return 0;
    }
}

// -----------------------------------------------------------------------------
long timespec_delta(struct timespec *t0, struct timespec *t1)
{
    return (t1->tv_nsec / 1000) - (t0->tv_nsec / 1000) +
           (t1->tv_sec - t0->tv_sec) * 1000000;
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
    adc_fd = 0;
}

// -----------------------------------------------------------------------------
int read_vbatt(void)
{
    static int vbatt_history[VBATT_HISTORY_LENGTH];
    static int vbatt_index = 0;
    static int vbatt_moving_avg = 0;
    static int vbatt_firsttime = 1;

    int i;
    float voltage = VBATT_MIN;

    // always report 100% battery if feature not enabled
    if (!adc_fd)
        return 100;
        
    if (vbatt_firsttime) {
        for (i = 0; i < VBATT_HISTORY_LENGTH; i++) {
            vbatt_history[i] = 100;
        }
        vbatt_moving_avg = VBATT_HISTORY_LENGTH * 100; 
        vbatt_firsttime = 0;   
    }
    
    vbatt_moving_avg -= vbatt_history[vbatt_index];
    
    memset(&parms, 0, sizeof(parms));
    parms.channel = ADC_VBATT_PORT;

    if (ioctl(adc_fd, TWL4030_MADC_IOCX_ADC_RAW_READ, &parms) != -1)
        voltage = ((unsigned int)parms.result) / 1024.f * ADC_INPUT_RANGE;

    voltage = CLAMP(voltage, VBATT_MIN, VBATT_MAX);
    voltage = (int)(100 * (voltage - VBATT_MIN) / VBATT_RANGE);
    
    vbatt_history[vbatt_index] = voltage; 
    vbatt_moving_avg += vbatt_history[vbatt_index];
     
    if (++vbatt_index >= VBATT_HISTORY_LENGTH)
       vbatt_index = 0;
        
    return (int)vbatt_moving_avg >> VBATT_HISTORY_SHIFT;
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

//-----------------------------------------------------------------------------
int get_cpu_utilization(void)
{
    static cpu_info_t cpu_history[CPU_HISTORY_LENGTH];
    static int cpu_index = 0;
    static int cpu_moving_avg = 0;

    FILE *fp;
    char c[10];
    int user, system, nice, idle, iow, hint, sint;
    real_t percentage;

    if (!(fp = fopen("/proc/stat","r"))) {
        return -1;
    }

    if (!fscanf(fp, "%s\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n", c,
                &user, &nice, &system, &idle, &iow, &hint, &sint)) {
        // failed to read
        fclose(fp);
        return -1;
    }
    fclose(fp);

    // calculate moving average of cpu load
    real_t userdif = user   - cpu_history[cpu_index].user;
    real_t sysdif  = system - cpu_history[cpu_index].system;
    real_t nicedif = nice   - cpu_history[cpu_index].nice;
    real_t idledif = idle   - cpu_history[cpu_index].idle;
    real_t hintdif = hint   - cpu_history[cpu_index].hint;
    real_t sintdif = sint   - cpu_history[cpu_index].sint;
    
    percentage = ((userdif + sysdif + hintdif + sintdif) / 
            (userdif + sysdif + nicedif + idledif + hintdif + sintdif)) * 100;

    cpu_moving_avg -= cpu_history[cpu_index].percentage;

    cpu_history[cpu_index].user      = user;
    cpu_history[cpu_index].system    = system;
    cpu_history[cpu_index].idle      = idle;
    cpu_history[cpu_index].nice      = nice;    
    cpu_history[cpu_index].hint      = hint; 
    cpu_history[cpu_index].sint      = sint;
    cpu_history[cpu_index].percentage= percentage;  

    cpu_moving_avg += cpu_history[cpu_index].percentage;
    
    if (++cpu_index >= CPU_HISTORY_LENGTH)
        cpu_index = 0;
    
    return (int)cpu_moving_avg >> CPU_HISTORY_SHIFT;
}

