// -----------------------------------------------------------------------------
// UAV control software for Gumstix Overo
// Garrett Smith 2010
// -----------------------------------------------------------------------------

#include <sys/types.h>
#include <sys/socket.h>
#include <pthread.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <syslog.h>
#include <unistd.h>
#include <getopt.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "razor_imu.h"
#include "server.h"
#include "ultrasonic.h"
#include "uav_protocol.h"
#include "video_uvc.h"

#define MAX_LEN 64

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
void run_server(imu_data_t *imu, ultrasonic_data_t *us, const char *port)
{
    struct sockaddr_storage addr;
    struct sockaddr_in *sa;
    struct addrinfo info, *r;
    socklen_t addr_sz = sizeof(addr);
    int hsock, hclient, rc;
    uint32_t cmd_buffer[32];
    uint32_t *big_buffer = NULL;
    unsigned long buff_sz = 0;
    uint32_t vcm_type = VCM_TYPE_RADIO, vcm_axes = VCM_AXIS_ALL;
    char ip4[INET_ADDRSTRLEN];
    video_data vid_data;

    union {
        float f;
        uint32_t i;
    } temp;

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
        cmd_buffer[PKT_COMMAND] = SERVER_REQ_IDENT;
        cmd_buffer[PKT_LENGTH]  = PKT_BASE_LENGTH;
        send(hclient, (void *)cmd_buffer, PKT_BASE_LENGTH, 0);

        if (1 > recv(hclient, (void *)cmd_buffer, 32, 0)) {
            syslog(LOG_INFO, "read failed -- client disconnected?");
            goto client_disconnect;
        }

        if ((cmd_buffer[PKT_COMMAND] != CLIENT_ACK_IDENT) ||
            (cmd_buffer[PKT_LENGTH] != PKT_RCI_LENGTH) ||
            (cmd_buffer[PKT_RCI_MAGIC] != IDENT_MAGIC) ||
            (cmd_buffer[PKT_RCI_VERSION] != IDENT_VERSION)) {
            syslog(LOG_INFO, "unexpected client response (c:%x s:%x m:%x v:%x)",
                   cmd_buffer[0], cmd_buffer[1], cmd_buffer[2], cmd_buffer[3]);
            goto client_disconnect;
        }
        syslog(LOG_INFO, "client provided valid identification");

        // enter main communication loop
        for (;;) {
            if (1 > recv(hclient, (void *)cmd_buffer, 32, 0)) {
                syslog(LOG_INFO, "read failed -- client disconnected?");
                goto client_disconnect;
            }
            // fprintf(stderr, "packet received\n");

            switch (cmd_buffer[PKT_COMMAND]) {
            case CLIENT_REQ_TAKEOFF:
                syslog(LOG_INFO, "user requested takeoff -- taking off...\n");
                cmd_buffer[PKT_COMMAND] = SERVER_ACK_TAKEOFF;
                cmd_buffer[PKT_LENGTH]  = PKT_BASE_LENGTH;
                send(hclient, (void *)cmd_buffer, PKT_BASE_LENGTH, 0);
                break;
            case CLIENT_REQ_LANDING:
                syslog(LOG_INFO, "user requested landing -- landing...\n");
                cmd_buffer[PKT_COMMAND] = SERVER_ACK_LANDING;
                cmd_buffer[PKT_LENGTH]  = PKT_BASE_LENGTH;
                send(hclient, (void *)cmd_buffer, PKT_BASE_LENGTH, 0);
                break;
            case CLIENT_REQ_TELEMETRY:
                // syslog(LOG_INFO, "user requested telemetry -- sending...\n");
                cmd_buffer[PKT_COMMAND] = SERVER_ACK_TELEMETRY;
                cmd_buffer[PKT_LENGTH]  = PKT_VTI_LENGTH;

                pthread_mutex_lock(&imu->lock);
                temp.f = imu->angles[0]; cmd_buffer[PKT_VTI_YAW]   = temp.i;
                temp.f = imu->angles[1]; cmd_buffer[PKT_VTI_PITCH] = temp.i;
                temp.f = imu->angles[2]; cmd_buffer[PKT_VTI_ROLL]  = temp.i;
                pthread_mutex_unlock(&imu->lock);

                cmd_buffer[PKT_VTI_RSSI]  = read_wlan_rssi(hclient);

                pthread_mutex_unlock(&us->lock);
                cmd_buffer[PKT_VTI_ALT] = us->height;
                pthread_mutex_unlock(&us->lock);

                cmd_buffer[PKT_VTI_BATT]  = 100;  // battery
                send(hclient, (void *)cmd_buffer, PKT_VTI_LENGTH, 0);
                break;
            case CLIENT_REQ_MJPG_FRAME:
                fprintf(stderr, "client requested frame... waiting\n");

                // safely lock and copy the jpeg image to our output buffer
                video_lock(&vid_data);
                if (buff_sz < (vid_data.length + PKT_MJPG_LENGTH))
                {
                    free(big_buffer);
                    buff_sz = vid_data.length + PKT_MJPG_LENGTH;
                    big_buffer = (uint32_t *)malloc(buff_sz);
                }

                memcpy(&big_buffer[PKT_MJPG_IMG], vid_data.data, vid_data.length);
                video_unlock();

                // now send out the entire jpeg frame
                big_buffer[PKT_COMMAND] = SERVER_ACK_MJPG_FRAME;
                big_buffer[PKT_LENGTH]  = vid_data.length + PKT_MJPG_LENGTH;
                big_buffer[PKT_MJPG_WIDTH] = vid_data.width;
                big_buffer[PKT_MJPG_HEIGHT] = vid_data.height;
                big_buffer[PKT_MJPG_FPS] = vid_data.fps;
                
                send(hclient, (void *)big_buffer, vid_data.length + PKT_MJPG_LENGTH, 0);
                fprintf(stderr, "send frame size %lu, pkt size %lu\n",
                        vid_data.length, vid_data.length + PKT_BASE_LENGTH);
                break;
            case CLIENT_REQ_SET_CTL_MODE:
                // interpret client's request for input mode change
                switch (cmd_buffer[PKT_VCM_TYPE])
                {
                case VCM_TYPE_RADIO:
                    fprintf(stderr, "switching to radio control\n");
                    vcm_type = VCM_TYPE_RADIO;
                    vcm_axes = VCM_AXIS_ALL; // all axes radio controlled
                    break;
                case VCM_TYPE_AUTO:
                    fprintf(stderr, "switching to autonomous control\n");
                    vcm_type = VCM_TYPE_AUTO;
                    vcm_axes = VCM_AXIS_ALL; // all axes autonomously controlled
                    break;
                case VCM_TYPE_MIXED:
                    fprintf(stderr, "switching to remote control mode\n");
                    vcm_type = VCM_TYPE_MIXED;
                    vcm_axes = cmd_buffer[PKT_VCM_AXES] & VCM_AXIS_ALL;
                    break;
                default:
                    fprintf(stderr, "bad control mode requested. ignoring\n");
                    cmd_buffer[PKT_COMMAND] = SERVER_ACK_IGNORED;
                    cmd_buffer[PKT_LENGTH]  = PKT_BASE_LENGTH;
                    send(hclient, (void *)cmd_buffer, PKT_BASE_LENGTH, 0);
                    continue;
                }

                cmd_buffer[PKT_COMMAND]  = SERVER_ACK_SET_CTL_MODE;
                cmd_buffer[PKT_LENGTH]   = PKT_VCM_LENGTH;
                cmd_buffer[PKT_VCM_TYPE] = vcm_type;
                cmd_buffer[PKT_VCM_AXES] = vcm_axes;
                send(hclient, (void *)cmd_buffer, PKT_VCM_LENGTH, 0);
                break;
            default:
                syslog(LOG_ERR, "invalid client command (%d)",
                       cmd_buffer[PKT_COMMAND]);
                cmd_buffer[PKT_COMMAND] = SERVER_ACK_IGNORED;
                cmd_buffer[PKT_LENGTH]  = PKT_BASE_LENGTH;
                send(hclient, (void *)cmd_buffer, PKT_BASE_LENGTH, 0);
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
           "  -D [ --daemonize ]      : run as a background process\n"
           "  -s [ --stty_dev ] arg   : specify serial device for IMU\n"
           "  -v [ --v4l_dev ] arg    : specify video device for webcam\n"
           "  -p [ --port ] arg       : specify port for network socket\n"
           "  -x [ --width ] arg      : specify resolution width for webcam\n"
           "  -y [ --height ] arg     : specify resolution height for webcam\n"
           "  -f [ --framerate ] arg  : specify capture framerate for webcam\n"
           "  -N [ --null-video ] arg : do not capture video from webcam\n"
           "  -v [ --verbose ]        : enable verbose logging\n"
           "  -h [ --help ]           : display this usage message\n");
}

// -----------------------------------------------------------------------------
// Program entry point -- process command line arguments and initialize daemon.
int main(int argc, char *argv[])
{
    int index, opt, log_opt, baud = B57600, ret = EXIT_SUCCESS;
    int flag_verbose = 0, flag_daemonize = 0, flag_nullvideo = 0;
    int flag_v4l = 0, flag_stty = 0;
    int arg_port = 8090, arg_width = 320, arg_height = 240, arg_fps = 15;
    int arg_ultrasonic = 176, arg_override = 65;
    char stty_dev[MAX_LEN], v4l_dev[MAX_LEN], port_str[MAX_LEN];
    imu_data_t imu;
    ultrasonic_data_t ultrasonic;

    static struct option long_options[] = {
        { "daemonize",  no_argument,       NULL, 'D' },
        { "stty_dev",   required_argument, NULL, 's' },
        { "v4l_dev",    required_argument, NULL, 'v' },
        { "port",       required_argument, NULL, 'p' },
        { "ultrasonic", required_argument, NULL, 'u' },
        { "override",   required_argument, NULL, 'o' },
        { "width",      required_argument, NULL, 'x' },
        { "height",     required_argument, NULL, 'y' },
        { "framerate",  required_argument, NULL, 'f' },
        { "null-video", no_argument,       NULL, 'N' },
        { "verbose",    no_argument,       NULL, 'V' },
        { "help",       no_argument,       NULL, 'h' },
        { 0, 0, 0, 0 }
    };

    static const char *str = "Ds:v:p:u:o:x:y:f:NVh?";

    while (-1 != (opt = getopt_long(argc, argv, str, long_options, &index))) {
        switch (opt) {
        case 'D':
            flag_daemonize = 1;
            break;
        case 's':
            strncpy(stty_dev, optarg, MAX_LEN);
            flag_stty = 1;
            break;
        case 'v':
            strncpy(v4l_dev, optarg, MAX_LEN);
            flag_v4l = 1;
            break;
        case 'p':
            arg_port = atoi(optarg);
            break;
        case 'u':
            arg_ultrasonic = atoi(optarg);
            break;
        case 'o':
            arg_override = atoi(optarg);
            break;
        case 'x':
            arg_width = atoi(optarg);
            break;
        case 'y':
            arg_height = atoi(optarg);
            break;
        case 'f':
            arg_fps = atoi(optarg);
            break;
        case 'V':
            flag_verbose = 1;
            break;
        case 'N':
            flag_nullvideo = 1;
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

    if (!flag_stty) {
        // set default device to UART1 if unspecified
        strncpy(stty_dev, "/dev/ttyS0", MAX_LEN);
    }
    syslog(LOG_INFO, "opening and configuring stty device '%s'\n", stty_dev);

    if (!flag_v4l) {
        // set default device to video0 if unspecified
        strncpy(v4l_dev, "/dev/video0", MAX_LEN);
    }
    syslog(LOG_INFO, "opening and configuring v4l device '%s'\n", v4l_dev);

    snprintf(port_str, MAX_LEN, "%d", arg_port);
    syslog(LOG_INFO, "opening network socket on port %s\n", port_str);

    // attempt to initialize imu communication
    if (!imu_init(stty_dev, baud, &imu)) {
        syslog(LOG_ERR, "failed to initialize IMU");
        ret = EXIT_FAILURE;
        goto cleanup;
    }

    // attempt to initialize ultrasonic communication
    if (!ultrasonic_init(arg_ultrasonic, &ultrasonic)) {
        syslog(LOG_ERR, "failed to initialize ultrasonic");
        ret = EXIT_FAILURE;
        goto cleanup;
    }

    if (!flag_nullvideo) {
        video_init(v4l_dev, arg_width, arg_height, arg_fps);
    }

    // server entry point
    run_server(&imu, &ultrasonic, port_str);

    // perform cleanup
cleanup:
    pthread_exit(NULL);
    ultrasonic_shutdown(&ultrasonic);
    imu_shutdown(&imu);
    close(ultrasonic.fd);
    syslog(LOG_INFO, "process terminating");
    closelog();
    return EXIT_SUCCESS;
}

