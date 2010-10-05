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
#include <getopt.h>
#include <signal.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "flight_control.h"
#include "pwm_interface.h"
#include "razor_imu.h"
#include "gpio_event.h"
#include "uav_protocol.h"
#include "video_uvc.h"
#include "user-gpio.h"
#include "utility.h"

#define DEV_LEN         64
#define PKT_BUFF_LEN    2048

imu_data_t g_imu;
gpio_event_t g_gpio_alt; // ultrasonic PWM
gpio_event_t g_gpio_aux; // auxiliary PWM

static ctl_sigs_t client_sigs = { 0 };
static char autonomous = 1;
static int g_muxsel = -1;

// -----------------------------------------------------------------------------
// Perform final shutdown and cleanup.
void uav_shutdown(int rc)
{
    fc_close_controls();

    syslog(LOG_INFO, "shutting down gpio user space subsystem...\n");
    if (g_muxsel > 0) {
        gpio_free(g_muxsel);
    }
    gpio_term();

    syslog(LOG_INFO, "shutting down gpio event subsystem...\n");
    gpio_event_detach(&g_gpio_aux);
    gpio_event_detach(&g_gpio_alt);
    gpio_event_shutdown();

    syslog(LOG_INFO, "shutting down imu subsystem...\n");
    imu_shutdown(&g_imu);

    syslog(LOG_INFO, "shutting down video subsystem...\n");
    video_shutdown();

    syslog(LOG_INFO, "shutting down uav control...\n");
    // pthread_exit(NULL);

    syslog(LOG_INFO, "process terminating");
    closelog();

    exit(rc);
}

// -----------------------------------------------------------------------------
// Perform any necessary signal handling. Does nothing useful at the moment.
void signal_handler(int sig)
{
    switch (sig) {
    case SIGINT:
        uav_shutdown(EXIT_SUCCESS);
        break;
    default:
        syslog(LOG_WARNING, "unhandled signal (%s)", strsignal(sig));
        break;
    }
}

// -----------------------------------------------------------------------------
// Receive a single packet. Blocks until the entire message has been consumed.
int recv_packet(int hclient, uint32_t *cmd_buffer)
{
    // read in the packet header to determine length
    int rc, bytes_left = PKT_BASE_LENGTH;
    char *ptr = (char *)cmd_buffer;
    while (bytes_left > 0) {
        if (1 > (rc = recv(hclient, (void *)ptr, bytes_left, 0))) {
            // assume client disconnected for now
            return 0;
        }
        bytes_left -= rc;
        ptr += rc;
    }

    // read in the rest of the packet
    bytes_left = cmd_buffer[PKT_LENGTH] - PKT_BASE_LENGTH;
    while (bytes_left > 0) {
        if (1 > (rc = recv(hclient, (void *)ptr, bytes_left, 0))) {
            // assume client disconnected for now
            return 0;
        }
        bytes_left -= rc;
        ptr += rc;
    }

    return 1;
}

// -----------------------------------------------------------------------------
// Server entry-point. Sits in a loop accepting and processing incoming client
// connections until terminated.
void run_server(imu_data_t *imu, const char *port)
{
    struct sockaddr_storage addr;
    struct sockaddr_in *sa;
    struct addrinfo info, *r;
    socklen_t addr_sz = sizeof(addr);
    int hsock, hclient, rc;
    uint32_t cmd_buffer[PKT_BUFF_LEN];
    uint32_t *jpg_buf = NULL;
    unsigned long buff_sz = 0;
    uint32_t vcm_type = VCM_TYPE_RADIO, vcm_axes = VCM_AXIS_ALL;
    char ip4[INET_ADDRSTRLEN];
    video_data_t vid_data;

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

        if (1 > recv(hclient, (void *)cmd_buffer, PKT_BUFF_LEN, 0)) {
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
            // read until we've consumed an entire packet
            if (!recv_packet(hclient, cmd_buffer)) {
                syslog(LOG_INFO, "read failed -- client disconnected?");
                goto client_disconnect;
            }

            switch (cmd_buffer[PKT_COMMAND]) {
            case CLIENT_REQ_TAKEOFF:
                syslog(LOG_INFO, "user requested takeoff -- taking off...\n");
                fc_takeoff();
                cmd_buffer[PKT_COMMAND] = SERVER_ACK_TAKEOFF;
                cmd_buffer[PKT_LENGTH]  = PKT_BASE_LENGTH;
                send(hclient, (void *)cmd_buffer, PKT_BASE_LENGTH, 0);
                break;
            case CLIENT_REQ_LANDING:
                syslog(LOG_INFO, "user requested landing -- landing...\n");
                fc_land();
                cmd_buffer[PKT_COMMAND] = SERVER_ACK_LANDING;
                cmd_buffer[PKT_LENGTH]  = PKT_BASE_LENGTH;
                send(hclient, (void *)cmd_buffer, PKT_BASE_LENGTH, 0);
                break;
            case CLIENT_REQ_TELEMETRY:
                // syslog(LOG_INFO, "user requested telemetry - sending...\n");
                cmd_buffer[PKT_COMMAND] = SERVER_ACK_TELEMETRY;
                cmd_buffer[PKT_LENGTH]  = PKT_VTI_LENGTH;

                pthread_mutex_lock(&imu->lock);
                temp.f = imu->angles[0]; cmd_buffer[PKT_VTI_YAW]   = temp.i;
                temp.f = imu->angles[1]; cmd_buffer[PKT_VTI_PITCH] = temp.i;
                temp.f = imu->angles[2]; cmd_buffer[PKT_VTI_ROLL]  = temp.i;
                pthread_mutex_unlock(&imu->lock);

                cmd_buffer[PKT_VTI_RSSI] = read_wlan_rssi(hclient);
                cmd_buffer[PKT_VTI_BATT] = 100;

                pthread_mutex_lock(&g_gpio_alt.lock);
                // taken from maxbotix from spec: 147 us == 1 inch
                cmd_buffer[PKT_VTI_ALT] = g_gpio_alt.pulsewidth / 147;
                pthread_mutex_unlock(&g_gpio_alt.lock);

                pthread_mutex_lock(&g_gpio_aux.lock);
                cmd_buffer[PKT_VTI_AUX]  = g_gpio_aux.pulsewidth;
                pthread_mutex_unlock(&g_gpio_aux.lock);

                send(hclient, (void *)cmd_buffer, PKT_VTI_LENGTH, 0);
                break;
            case CLIENT_REQ_MJPG_FRAME:
                // syslog(LOG_INFO, "user requested mjpg frame - sending...\n");
                if (!video_lock(&vid_data, 0)) {
                    // video disabled, non-functioning, or frame not ready
                    continue;
                }

                // copy the jpeg to our buffer now that we're safely locked
                if (buff_sz < (vid_data.length + PKT_MJPG_LENGTH))
                {
                    free(jpg_buf);
                    buff_sz = vid_data.length + PKT_MJPG_LENGTH;
                    jpg_buf = (uint32_t *)malloc(buff_sz);
                }

                memcpy(&jpg_buf[PKT_MJPG_IMG], vid_data.data, vid_data.length);
                video_unlock();

                // now send out the entire jpeg frame
                jpg_buf[PKT_COMMAND] = SERVER_ACK_MJPG_FRAME;
                jpg_buf[PKT_LENGTH]  = vid_data.length + PKT_MJPG_LENGTH;
                jpg_buf[PKT_MJPG_WIDTH] = vid_data.width;
                jpg_buf[PKT_MJPG_HEIGHT] = vid_data.height;
                jpg_buf[PKT_MJPG_FPS] = vid_data.fps;
                
                send(hclient, (void *)jpg_buf, vid_data.length + PKT_MJPG_LENGTH, 0);
                syslog(LOG_DEBUG, "send frame size %lu, pkt size %lu\n",
                       vid_data.length, vid_data.length + PKT_BASE_LENGTH);
                break;
            case CLIENT_REQ_SET_CTL_MODE:
                // interpret client's request for input mode change
                switch (cmd_buffer[PKT_VCM_TYPE])
                {
                case VCM_TYPE_RADIO:
                    syslog(LOG_DEBUG, "switching to radio control\n");
                    vcm_type = VCM_TYPE_RADIO;
                    vcm_axes = VCM_AXIS_ALL; // all axes radio controlled
                    gpio_set_value(g_muxsel, 1);
                    break;
                case VCM_TYPE_AUTO:
                    syslog(LOG_DEBUG, "switching to autonomous control\n");
                    vcm_type = VCM_TYPE_AUTO;
                    vcm_axes = VCM_AXIS_ALL; // all axes autonomously controlled
                    autonomous = 1;
                    gpio_set_value(g_muxsel, 0);
                    break;
                case VCM_TYPE_MIXED:
                    syslog(LOG_DEBUG, "switching to remote control mode\n");
                    vcm_type = VCM_TYPE_MIXED;
                    vcm_axes = cmd_buffer[PKT_VCM_AXES];
                    autonomous = 0;
                    gpio_set_value(g_muxsel, 0);
                    break;
                case VCM_TYPE_KILL:
                    syslog(LOG_DEBUG, "switching to killswitch enabled mode\n");
                    vcm_type = VCM_TYPE_KILL;
                    vcm_axes = VCM_AXIS_ALL; // all axes disabled
                    gpio_set_value(g_muxsel, 1);
                    break;
                default:
                    syslog(LOG_DEBUG, "bad control mode requested. ignoring\n");
                    cmd_buffer[PKT_COMMAND] = SERVER_ACK_IGNORED;
                    cmd_buffer[PKT_LENGTH]  = PKT_BASE_LENGTH;
                    send(hclient, (void *)cmd_buffer, PKT_BASE_LENGTH, 0);
                    continue;
                }

                fc_update_axes(vcm_axes);

                cmd_buffer[PKT_COMMAND]  = SERVER_ACK_SET_CTL_MODE;
                cmd_buffer[PKT_LENGTH]   = PKT_VCM_LENGTH;
                cmd_buffer[PKT_VCM_TYPE] = vcm_type;
                cmd_buffer[PKT_VCM_AXES] = vcm_axes;
                send(hclient, (void *)cmd_buffer, PKT_VCM_LENGTH, 0);
                break;
            case CLIENT_REQ_FLIGHT_CTL:
                // collect signals
                temp.i = cmd_buffer[PKT_MCM_AXIS_ALT];
                client_sigs.alt = temp.f;

                temp.i = cmd_buffer[PKT_MCM_AXIS_PITCH];
                client_sigs.pitch = temp.f;

                temp.i = cmd_buffer[PKT_MCM_AXIS_ROLL];
                client_sigs.roll = temp.f;

                temp.i = cmd_buffer[PKT_MCM_AXIS_YAW];
                client_sigs.yaw = temp.f;

                syslog(LOG_DEBUG, "Received controls: %f, %f, %f, %f\n",
                        client_sigs.alt, client_sigs.pitch,
                        client_sigs.roll, client_sigs.yaw);
                
                // send signals off to PWMs
                flight_control(&client_sigs, vcm_axes);
                break;
            case CLIENT_REQ_THRO_EVT:
                // client is telling flight control to increment the throttle signal
                temp.i = cmd_buffer[PKT_THRO_EVT_VALUE];
                client_sigs.alt = temp.f;
                fprintf(stderr, "received throttle event %f\n", client_sigs.alt);
                flight_control(&client_sigs, VCM_AXIS_ALT);
                break;
            default:
                // dump a reasonable number of entries for debugging purposes
                syslog(LOG_ERR, "invalid client command (C:%d L:%d 0:%d 1:%d)",
                       cmd_buffer[PKT_COMMAND],
                       cmd_buffer[PKT_LENGTH],
                       cmd_buffer[PKT_BASE + 0],
                       cmd_buffer[PKT_BASE + 1]);

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
           "  -s [ --stty_dev ] arg   : specify serial device for IMU\n"
           "  -v [ --v4l_dev ] arg    : specify video device for webcam\n"
           "  -p [ --port ] arg       : specify port for network socket\n"
           "  -m [ --mux ] arg        : specify gpio for mux select line\n"
           "  -u [ --ultrasonic ] arg : specify gpio for ultrasonic pwm\n"
           "  -o [ --override ] arg   : specify gpio for override pwm\n"
           "  -x [ --width ] arg      : specify resolution width for webcam\n"
           "  -y [ --height ] arg     : specify resolution height for webcam\n"
           "  -f [ --framerate ] arg  : specify capture framerate for webcam\n"
           "  -D [ --daemonize ]      : run as a background process\n"
           "  -V [ --verbose ]        : enable verbose logging\n"
           "  -h [ --help ]           : display this usage message\n"
           "  --no-video              : do not capture video from webcam\n"
           "  --no-gpio               : do not perform any gpio processing\n");
}

// -----------------------------------------------------------------------------
// Program entry point -- process command line arguments and initialize daemon.
int main(int argc, char *argv[])
{
    int index, opt, log_opt, baud = B57600;
    int flag_verbose = 0, flag_daemonize = 0, flag_novideo = 0, flag_nogpio = 0;
    int arg_port = 8090, arg_width = 320, arg_height = 240, arg_fps = 15;
    int arg_mux = 170, arg_ultrasonic = 171, arg_override = 172;
    char port_str[DEV_LEN];
    char stty_dev[DEV_LEN] = "/dev/ttyS0";
    char v4l_dev[DEV_LEN] = "/dev/video0";

    struct option long_options[] = {
        { "stty_dev",   required_argument, NULL, 's' },
        { "v4l_dev",    required_argument, NULL, 'v' },
        { "port",       required_argument, NULL, 'p' },
        { "mux",        required_argument, NULL, 'm' },
        { "ultrasonic", required_argument, NULL, 'u' },
        { "override",   required_argument, NULL, 'o' },
        { "width",      required_argument, NULL, 'x' },
        { "height",     required_argument, NULL, 'y' },
        { "framerate",  required_argument, NULL, 'f' },
        { "daemonize",  no_argument,       NULL, 'D' },
        { "verbose",    no_argument,       NULL, 'V' },
        { "help",       no_argument,       NULL, 'h' },
        { "no-video",   no_argument,       &flag_novideo, 1 },
        { "no-gpio",    no_argument,       &flag_nogpio,  1 },
        { 0, 0, 0, 0 }
    };

    static const char *str = "s:v:p:m:u:o:x:y:f:DVh?";

    while (-1 != (opt = getopt_long(argc, argv, str, long_options, &index))) {
        switch (opt) {
        case 's':
            strncpy(stty_dev, optarg, DEV_LEN);
            break;
        case 'v':
            strncpy(v4l_dev, optarg, DEV_LEN);
            break;
        case 'p':
            arg_port = atoi(optarg);
            break;
        case 'm':
            arg_mux = atoi(optarg);
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
        case 'D':
            flag_daemonize = 1;
            break;
        case 'V':
            flag_verbose = 1;
            break;
        case 'h': // fall through
        case '?':
            print_usage();
            exit(EXIT_SUCCESS);
        case 0:
            break;
        default:
            syslog(LOG_ERR, "unexpected argument '%c'\n", opt);
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

    snprintf(port_str, DEV_LEN, "%d", arg_port);
    syslog(LOG_INFO, "opening network socket on port %s\n", port_str);

    // install signal handler for clean shutdown
    if (SIG_ERR == signal(SIGINT, uav_shutdown)) {
        syslog(LOG_INFO, "failed to install sigint handler\n");
        uav_shutdown(EXIT_FAILURE);
    }

    // attempt to initialize imu communication
    syslog(LOG_INFO, "opening and configuring stty device '%s'\n", stty_dev);
    if (!imu_init(stty_dev, baud, &g_imu)) {
        syslog(LOG_ERR, "failed to initialize IMU");
        uav_shutdown(EXIT_FAILURE);
    }

    // initialize the gpio subsystem(s) unless specified not to (no-gpio)
    if (!flag_nogpio)
    {
        // initialize gpio event monitors
        if (!gpio_event_init()) {
            syslog(LOG_ERR, "failed to initialize gpio event subsystem");
            uav_shutdown(EXIT_FAILURE);
        }

        if (!gpio_event_attach(&g_gpio_alt, arg_ultrasonic)) {
            syslog(LOG_ERR, "failed to monitor ultrasonic gpio pin");
            uav_shutdown(EXIT_FAILURE);
        }

        if (!gpio_event_attach(&g_gpio_aux, arg_override)) {
            syslog(LOG_ERR, "failed to monitor auxiliary override gpio pin");
            uav_shutdown(EXIT_FAILURE);
        }

        // attempt to initialize gpio pin for multiplexer select
        if (0 > gpio_init()) {
            syslog(LOG_ERR, "failed to initialize gpio user space library");
            uav_shutdown(EXIT_FAILURE);
        }

        g_muxsel = arg_mux;
        if (0 > gpio_request(arg_mux, "uav_control mux select line")) {
            syslog(LOG_ERR, "failed to request mux select gpio %d", arg_mux);
            uav_shutdown(EXIT_FAILURE);
        }

        if (0 > gpio_direction_output(arg_mux, 1)) {
            syslog(LOG_ERR, "failed to set gpio %d direction to output", arg_mux);
            uav_shutdown(EXIT_FAILURE);
        }
    }

    // initialize video subsystem unless specified not to (no-video)
    if (!flag_novideo) {
        syslog(LOG_INFO, "opening and configuring v4l device '%s'\n", v4l_dev);
        video_init(v4l_dev, arg_width, arg_height, arg_fps);
    }

    // open PWM ports for mixed controlling
    fc_open_controls(&g_gpio_alt);

    // server entry point
    run_server(&g_imu, port_str);
    
    uav_shutdown(EXIT_SUCCESS);
    return EXIT_SUCCESS;
}

