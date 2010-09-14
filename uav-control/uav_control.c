// -----------------------------------------------------------------------------
// UAV control software for Gumstix Overo
// Garrett Smith 2010
// -----------------------------------------------------------------------------

#define _GNU_SOURCE
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <pthread.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <syslog.h>
#include <unistd.h>
#include <getopt.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#define MAX_LEN 64

#define IDENT_MAGIC     0x09291988  // identification number
#define IDENT_VERSION   0x00000001  // software version

#define SERVER_REQ_IDENT        0   // request client to identify itself
#define SERVER_ACK_IGNORED      1   // client request ignored (invalid state)
#define SERVER_ACK_TAKEOFF      2   // acknowledge request to take off
#define SERVER_ACK_LANDING      3   // acknowledge request to land
#define SERVER_ACK_TELEMETRY    4   // acknowledge request for telemetry (+data)
#define SERVER_MJPG_FRAME       5   // transmit a single frame of video

#define CLIENT_ACK_IDENT        0   // identify self to server
#define CLIENT_REQ_TAKEOFF      1   // command the helicopter to take off
#define CLIENT_REQ_LANDING      2   // command the helicopter to land
#define CLIENT_REQ_TELEMETRY    3   // state, orientation, altitude, battery

typedef struct serial_data
{
    int hdev;               // file descriptor for serial device
    pthread_t hthrd;        // serial thread object
    pthread_mutex_t lock;   // critical section for serial data
    float angles[3];        // orientation angles from IMU
    unsigned long sample;
} serial_data_t;


// -----------------------------------------------------------------------------
// Perform any necessary signal handling. Does nothing useful at the moment.
void signal_handler(int sig)
{
    switch (sig) {
    default:
        syslog(LOG_WARNING, "unhandled signal (%s)", strsignal(sig));
        break;
    }
}

// -----------------------------------------------------------------------------
// Daemonize the process by forking from init and chdir-ing to /.
void daemonize()
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
void *serial_consumer(void *thread_args)
{
    serial_data_t *data = (serial_data_t *)thread_args;
    char buff[128], num_buff[16], *ptr;
    int nb, i, data_idx = 0, num_idx = 0;
    float temp_data[3];

    // XXX: rewrite me -- I can barely make sense of what I wrote here
    for (;;) {
        ptr = buff;
        while (0 < (nb = read(data->hdev, ptr, buff + sizeof(buff) - ptr))) {
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
void initialize_serial(const char *device, int baud, serial_data_t *data)
{
    int rc;
    struct termios term_opt;

    memset((char *)data, 0, sizeof(serial_data_t));

    // attempt to open the specified serial device for binary RW
    if (0 > (data->hdev = open(device, O_RDWR | O_NOCTTY | O_NDELAY))) {
        syslog(LOG_ERR, "unable to open IMU UART");
        exit(EXIT_FAILURE);
    }

    // clear flags and get current attributes
    fcntl(data->hdev, F_SETFL, 0);
    tcgetattr(data->hdev, &term_opt);

    // set the baud rate, local line ownership, and enable receiver
    cfsetispeed(&term_opt, baud);
    cfsetospeed(&term_opt, baud);
    term_opt.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(data->hdev, TCSANOW, &term_opt);

    if (0 != (rc = pthread_mutex_init(&data->lock, NULL))) {
        syslog(LOG_ERR, "error creating serial mutex (%d)", rc);
        exit(EXIT_FAILURE);
    }

    void *arg = (void *)data;
    if (0 != (rc = pthread_create(&data->hthrd, NULL, serial_consumer, arg))) {
        syslog(LOG_ERR, "error creating serial thread (%d)", rc);
        exit(EXIT_FAILURE);
    }
}

// -----------------------------------------------------------------------------
void run_server(serial_data_t *data, const char *port)
{
    struct sockaddr_storage addr;
    struct sockaddr_in *sa;
    struct addrinfo info, *r;
    socklen_t addr_sz = sizeof(addr);
    int hsock, hclient, rc;
    int cmd_buffer[32];
    char ip4[INET_ADDRSTRLEN];

    memset(&info, 0, sizeof(info));
    info.ai_family   = AF_UNSPEC;
    info.ai_socktype = SOCK_STREAM;
    info.ai_flags    = AI_PASSIVE;

    // initialize socket for network communication
    if (0 != (rc = getaddrinfo(NULL, port, &info, &r))) {
        syslog(LOG_ERR, "server failed to get address info (%d)", rc);
        exit(EXIT_FAILURE);
    }

    if (0 > (hsock = socket(r->ai_family, r->ai_socktype, r->ai_protocol))) {
        syslog(LOG_ERR, "server failed to create socket (%d)", hsock);
        exit(EXIT_FAILURE);
    }

    if (0 > (rc = bind(hsock, r->ai_addr, r->ai_addrlen))) {
        syslog(LOG_ERR, "server failed to bind socket (%d)", rc);
        exit(EXIT_FAILURE);
    }

    if (0 > (rc = listen(hsock, 1))) {
        syslog(LOG_ERR, "failed to listen on incoming connections (%d)", rc);
        exit(EXIT_FAILURE);
    }

    for (;;) {
        // block until we receive an incoming connection from a client
        syslog(LOG_INFO, "waiting for incoming connection");
        if (0 > (hclient = accept(hsock, (struct sockaddr *)&addr, &addr_sz))) {
            syslog(LOG_ERR, "failed to accept incoming client (%d)", hclient);
            exit(EXIT_FAILURE);
        }
        sa = (struct sockaddr_in *)&addr;
        inet_ntop(AF_INET, &(sa->sin_addr), ip4, INET_ADDRSTRLEN);
        syslog(LOG_INFO, "established connection to client (%s)", ip4);

        // send request for client identification
        cmd_buffer[0] = SERVER_REQ_IDENT;
        send(hclient, (void *)cmd_buffer, sizeof(int) * 1, 0);

        if (1 > recv(hclient, (void *)cmd_buffer, 32, 0)) {
            syslog(LOG_INFO, "read failed -- client disconnected?");
            goto client_disconnect;
        }

        if ((cmd_buffer[0] != CLIENT_ACK_IDENT) ||
            (cmd_buffer[1] != IDENT_MAGIC) ||
            (cmd_buffer[2] != IDENT_VERSION)) {
            syslog(LOG_INFO, "unexpected client response (c:%x, m:%x, v:%x)",
                   cmd_buffer[0], cmd_buffer[1], cmd_buffer[2]);
            goto client_disconnect;
        }
        syslog(LOG_INFO, "client provided valid identification");

        // enter main communication loop
        for (;;) {
            if (1 > recv(hclient, (void *)cmd_buffer, 32, 0)) {
                syslog(LOG_INFO, "read failed -- client disconnected?");
                goto client_disconnect;
            }

            switch (cmd_buffer[0]) {
            case CLIENT_REQ_TAKEOFF:
                syslog(LOG_INFO, "user requested takeoff -- taking off...\n");
                cmd_buffer[0] = SERVER_ACK_TAKEOFF;
                send(hclient, (void *)cmd_buffer, sizeof(int) * 1, 0);
                break;
            case CLIENT_REQ_LANDING:
                syslog(LOG_INFO, "user requested landing -- landing...\n");
                cmd_buffer[0] = SERVER_ACK_LANDING;
                send(hclient, (void *)cmd_buffer, sizeof(int) * 1, 0);
                break;
            case CLIENT_REQ_TELEMETRY:
                // syslog(LOG_INFO, "user requested telemetry -- sending...\n");
                cmd_buffer[0] = SERVER_ACK_TELEMETRY;
                pthread_mutex_lock(&data->lock);
                cmd_buffer[1] = *(int *)&data->angles[0];
                cmd_buffer[2] = *(int *)&data->angles[1];
                cmd_buffer[3] = *(int *)&data->angles[2];
                pthread_mutex_unlock(&data->lock);
                cmd_buffer[4] = 0;  // altitude
                cmd_buffer[5] = 0;  // battery
                send(hclient, (void *)cmd_buffer, sizeof(int) * 6, 0);
                break;
            default:
                syslog(LOG_ERR, "invalid client command (%d)", cmd_buffer[0]);
                cmd_buffer[0] = SERVER_ACK_IGNORED;
                send(hclient, (void *)cmd_buffer, sizeof(int) * 1, 0);
                goto client_disconnect;
            }
        }

        // perform cleanup -- disconnect and wait for next connection
client_disconnect:
        syslog(LOG_INFO, "disconnected from client (%d)", hclient);
        shutdown(hclient, SHUT_RDWR);
        close(hclient);
    }
}

// -----------------------------------------------------------------------------
// Display program usage message.
void print_usage()
{
    printf("usage: uav_control [options]\n\n"
           "Program options:\n"
           "  -D [ --daemonize ]    : run as a background process\n"
           "  -d [ --device ] arg   : specify serial device for IMU\n"
           "  -p [ --port ] arg     : specify port for network socket\n"
           "  -v [ --verbose ]      : enable verbose logging\n"
           "  -h [ --help ]         : display this usage message\n");
}

// -----------------------------------------------------------------------------
// Program entry point -- process command line arguments and initialize daemon.
int main(int argc, char *argv[])
{
    int index, opt, log_opt, baud = B57600;
    int flag_verbose = 0, flag_daemonize = 0, flag_device = 0, flag_port = 0;
    char device[MAX_LEN], port[MAX_LEN];
    serial_data_t serial;

    static struct option long_options[] = {
        { "daemonize", no_argument,       NULL, 'D' },
        { "device",    required_argument, NULL, 'd' },
        { "verbose",   no_argument,       NULL, 'v' },
        { "help",      no_argument,       NULL, 'h' },
        { 0, 0, 0, 0 }
    };

    static const char *str = "Dd:p:vh?";

    while (-1 != (opt = getopt_long(argc, argv, str, long_options, &index))) {
        switch (opt) {
        case 'D':
            flag_daemonize = 1;
            break;
        case 'd':
            strncpy(device, optarg, MAX_LEN);
            flag_device = 1;
            break;
        case 'p':
            strncpy(port, optarg, MAX_LEN);
            flag_port = 1;
            break;
        case 'v':
            flag_verbose = 1;
            break;
        case 'h': // fall through
        case '?':
            print_usage();
            exit(EXIT_SUCCESS);
        case 0:
            break;
        default:
            assert(!"unhandled case in option handling -- this is an error");
            break;
        }
    }

    if (flag_daemonize) {
        // run as a background process
        daemonize();
    }

    // attach to the system log server
    log_opt = flag_verbose ? (LOG_PID | LOG_PERROR) : LOG_PID;
    openlog("uav", log_opt, LOG_DAEMON);
    syslog(LOG_INFO, "uav-control initialized");

    if (!flag_device) {
        // set default device to UART1 if unspecified
        strncpy(device, "/dev/ttyS0", MAX_LEN);
    }
    syslog(LOG_INFO, "opening and configuring device '%s'\n", device);
    initialize_serial(device, baud, &serial);

    if (!flag_port) {
        // set default port number if unspecified
        strncpy(port, "8090", MAX_LEN);
    }
    syslog(LOG_INFO, "opening network socket on port %s\n", port);

    // server entry point
    run_server(&serial, port);

    // perform cleanup
    pthread_exit(NULL);
    pthread_mutex_destroy(&serial.lock);
    close(serial.hdev);
    syslog(LOG_INFO, "process terminating");
    closelog();
    return EXIT_SUCCESS;
}

