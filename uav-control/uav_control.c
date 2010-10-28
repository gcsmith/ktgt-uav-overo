// -----------------------------------------------------------------------------
// File:    uav_control.c
// Authors: Garrett Smith, Kevin Macksamie, Tyler Thierolf, Timothy Miller
// Created: 08-23-2010
//
// UAV control software for Gumstix Overo & Blade CX-2 RC helicoptor.
// -----------------------------------------------------------------------------

#include <sys/resource.h>
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
#include "gpio_event.h"
#include "pwm_interface.h"
#include "razor_imu.h"
#include "tracking.h"
#include "uav_protocol.h"
#include "user-gpio.h"
#include "video_uvc.h"
#include "utility.h"

#define DEV_LEN 64
#define MUX_SEL_UPROC 0
#define MUX_SEL_RADIO 1
#define FC_TRGT_SIG_ALL  0
#define FC_TRGT_SIG_THRO 1

static imu_data_t g_imu;
static gpio_event_t g_gpio_alt; // ultrasonic PWM
static gpio_event_t g_gpio_aux; // auxiliary PWM
static client_info_t g_client;

static pthread_t g_aux_thread;
static ctl_sigs_t client_sigs = { 0 };
static int g_muxsel = -1;

// -----------------------------------------------------------------------------
static void *aux_trigger_thread(void *arg)
{
    uint32_t cmd_buffer[16];
    int pulse = 0, axes = 0, type = 0, last_type = 0;

    for (;;) {
        // go to sleep until we're pinged by the gpio_event subsystem
        pulse = gpio_event_read(&g_gpio_aux, ACCESS_SYNC);
        fc_get_vcm(&axes, &type);

        if (VCM_TYPE_KILL == type) {
            syslog(LOG_INFO, "AUX: killed, exiting aux thread\n");
            break;
        }

        // determine if we need to perform a state transition
        if (VCM_TYPE_LOCKOUT != type && pulse > 1475) {
            syslog(LOG_INFO, "AUX: switch from autonomous to manual\n");
            fc_set_vcm(axes, VCM_TYPE_LOCKOUT);
            gpio_set_value(g_muxsel, MUX_SEL_RADIO);
            last_type = type;

            cmd_buffer[PKT_COMMAND]  = SERVER_UPDATE_CTL_MODE;
            cmd_buffer[PKT_LENGTH]   = PKT_VCM_LENGTH;
            cmd_buffer[PKT_VCM_TYPE] = VCM_TYPE_LOCKOUT;
            cmd_buffer[PKT_VCM_AXES] = VCM_AXIS_ALL;
            send_packet(&g_client, cmd_buffer, PKT_VCM_LENGTH);
        }
        else if (VCM_TYPE_LOCKOUT == type && pulse <= 1475) {
            syslog(LOG_INFO, "AUX: switch from manual to autonomous\n");
            fc_set_vcm(axes, last_type);
            gpio_set_value(g_muxsel, MUX_SEL_UPROC);

            cmd_buffer[PKT_COMMAND]  = SERVER_UPDATE_CTL_MODE;
            cmd_buffer[PKT_LENGTH]   = PKT_VCM_LENGTH;
            cmd_buffer[PKT_VCM_TYPE] = VCM_TYPE_AUTO;
            cmd_buffer[PKT_VCM_AXES] = VCM_AXIS_ALL;
            send_packet(&g_client, cmd_buffer, PKT_VCM_LENGTH);
        }
    }

    pthread_exit(NULL);
}

// -----------------------------------------------------------------------------
void send_enum_ctrl(const struct v4l2_queryctrl *qc)
{
    uint32_t cmd_buffer[32];

    cmd_buffer[PKT_COMMAND] = SERVER_UPDATE_CAM_DCI;
    cmd_buffer[PKT_LENGTH]  = PKT_CAM_DCI_LENGTH;

    // convert the V4L type to something more generic
    switch (qc->type) {
    case V4L2_CTRL_TYPE_INTEGER:
    case V4L2_CTRL_TYPE_INTEGER64:
        cmd_buffer[PKT_CAM_DCI_TYPE] = CAM_DCI_TYPE_INT;
        break;
    case V4L2_CTRL_TYPE_BOOLEAN:
        cmd_buffer[PKT_CAM_DCI_TYPE] = CAM_DCI_TYPE_BOOL;
        break;
    case V4L2_CTRL_TYPE_MENU:
        cmd_buffer[PKT_CAM_DCI_TYPE] = CAM_DCI_TYPE_MENU;
        break;
    default:
        // simply ignore types that we don't care about
        return;
    }

    // fill in the rest of the parameters
    cmd_buffer[PKT_CAM_DCI_ID]      = qc->id;
    cmd_buffer[PKT_CAM_DCI_MIN]     = qc->minimum;
    cmd_buffer[PKT_CAM_DCI_MAX]     = qc->maximum;
    cmd_buffer[PKT_CAM_DCI_STEP]    = qc->step;
    cmd_buffer[PKT_CAM_DCI_DEFAULT] = qc->default_value;
    cmd_buffer[PKT_CAM_DCI_CURRENT] = video_get_devctrl(qc->id);
    memcpy(&cmd_buffer[PKT_CAM_DCI_NAME], qc->name, 32);

    syslog(LOG_INFO, "sending camera device control info");
    send_packet(&g_client, cmd_buffer, PKT_CAM_DCI_LENGTH);
}

// -----------------------------------------------------------------------------
void send_enum_menu(const struct v4l2_querymenu *qm)
{
    uint32_t cmd_buffer[32];

    cmd_buffer[PKT_COMMAND] = SERVER_UPDATE_CAM_DCM;
    cmd_buffer[PKT_LENGTH]  = PKT_CAM_DCM_LENGTH;

    cmd_buffer[PKT_CAM_DCM_ID]    = qm->id;
    cmd_buffer[PKT_CAM_DCM_INDEX] = qm->index;
    memcpy(&cmd_buffer[PKT_CAM_DCM_NAME], qm->name, 32);

    syslog(LOG_INFO, "sending camera device menu info");
    send_packet(&g_client, cmd_buffer, PKT_CAM_DCM_LENGTH);
}

// -----------------------------------------------------------------------------
// Perform final shutdown and cleanup.
void uav_shutdown(int rc)
{
    adc_close_channels();
    fc_shutdown();

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

    syslog(LOG_INFO, "shutting color tracking subsystem...\n");
    tracking_shutdown();

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
// Server entry-point. Sits in a loop accepting and processing incoming client
// connections until terminated.
void run_server(imu_data_t *imu, const char *port)
{
    struct sockaddr_storage addr;
    struct sockaddr_in *sa;
    struct addrinfo info, *r;
    socklen_t addr_sz = sizeof(addr);
    int hsock, rc, samples, track_fps;
    uint32_t cmd_buffer[PKT_BUFF_LEN];
    uint32_t *jpg_buf = NULL;
    unsigned long buff_sz = 0;
    int vcm_type = VCM_TYPE_RADIO, vcm_axes = VCM_AXIS_ALL;
    char ip4[INET_ADDRSTRLEN];
    video_data_t vid_data;
    track_color_t tc;
    float curr_alt, pid_val, pid_params[PID_PARAM_COUNT] = { 0 };
    float angles[3] = { 0 };

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

    if (0 != (rc = pthread_mutex_init(&g_client.lock, NULL))) {
        syslog(LOG_ERR, "error creating client mutex lock (%d)", rc);
        exit(EXIT_FAILURE);
    }

    for (;;) {
        // block until we receive an incoming connection from a client
        syslog(LOG_INFO, "waiting for incoming connection");
        if (0 > (g_client.fd = accept(hsock, (struct sockaddr *)&addr, &addr_sz))) {
            syslog(LOG_ERR, "failed to accept incoming client (%d)", g_client.fd);
            exit(EXIT_FAILURE);
        }
        sa = (struct sockaddr_in *)&addr;
        inet_ntop(AF_INET, &(sa->sin_addr), ip4, INET_ADDRSTRLEN);
        syslog(LOG_INFO, "established connection to client (%s)", ip4);

        // send request for client identification
        send_simple_packet(&g_client, SERVER_REQ_IDENT);

        if (!recv_packet(&g_client, cmd_buffer)) {
            syslog(LOG_INFO, "recv unsuccessful -- disconnecting client");
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

        // ping the client with our initial state
        fc_get_vcm(&vcm_axes, &vcm_type);
        cmd_buffer[PKT_COMMAND]  = SERVER_UPDATE_CTL_MODE;
        cmd_buffer[PKT_LENGTH]   = PKT_VCM_LENGTH;
        cmd_buffer[PKT_VCM_TYPE] = vcm_type;
        cmd_buffer[PKT_VCM_AXES] = vcm_axes;
        send_packet(&g_client, cmd_buffer, PKT_VCM_LENGTH);

        // enter main communication loop
        for (;;) {
            // read until we've consumed an entire packet
            if (!recv_packet(&g_client, cmd_buffer)) {
                syslog(LOG_INFO, "recv unsuccessful -- disconnecting client");
                goto client_disconnect;
            }

            switch (cmd_buffer[PKT_COMMAND]) {
            case CLIENT_REQ_TAKEOFF:
                syslog(LOG_INFO, "user requested takeoff -- taking off...\n");
                fc_request_takeoff();
                send_simple_packet(&g_client, SERVER_ACK_TAKEOFF);
                break;
            case CLIENT_REQ_LANDING:
                syslog(LOG_INFO, "user requested landing -- landing...\n");
                fc_request_landing();
                send_simple_packet(&g_client, SERVER_ACK_LANDING);
                break;
            case CLIENT_REQ_TELEMETRY:
                // syslog(LOG_INFO, "user requested telemetry - sending...\n");
                cmd_buffer[PKT_COMMAND] = SERVER_ACK_TELEMETRY;
                cmd_buffer[PKT_LENGTH]  = PKT_VTI_LENGTH;

                imu_read_angles(imu, angles, ACCESS_ASYNC);
                memcpy(&cmd_buffer[PKT_VTI_YAW],   &angles[0], 4);
                memcpy(&cmd_buffer[PKT_VTI_PITCH], &angles[1], 4);
                memcpy(&cmd_buffer[PKT_VTI_ROLL],  &angles[2], 4);
                
                cmd_buffer[PKT_VTI_RSSI] = read_wlan_rssi(g_client.fd);
                cmd_buffer[PKT_VTI_BATT] = read_vbatt();

                // taken from maxbotix from spec: 147 us == 1 inch
                curr_alt = gpio_event_read(&g_gpio_alt, ACCESS_ASYNC) / 147.0f;
                memcpy(&cmd_buffer[PKT_VTI_ALT], &curr_alt, 4);

                cmd_buffer[PKT_VTI_AUX] = gpio_event_read(&g_gpio_aux, ACCESS_ASYNC);
                cmd_buffer[PKT_VTI_CPU] = get_cpu_utilization();
                send_packet(&g_client, cmd_buffer, PKT_VTI_LENGTH);
                break;
            case CLIENT_REQ_MJPG_FRAME:
                // syslog(LOG_INFO, "user requested mjpg frame - sending...\n");
                if (!video_lock(&vid_data, ACCESS_ASYNC)) {
                    // video disabled, non-functioning, or frame not ready
                    continue;
                }

                // copy the jpeg to our buffer now that we're safely locked
                if (buff_sz < (vid_data.length + PKT_MJPG_LENGTH)) {
                    free(jpg_buf);
                    buff_sz = vid_data.length + PKT_MJPG_LENGTH;
                    jpg_buf = (uint32_t *)malloc(buff_sz);
                }

                memcpy(&jpg_buf[PKT_MJPG_IMG], vid_data.data, vid_data.length);
                video_unlock();

                // now send out the entire jpeg frame
                jpg_buf[PKT_COMMAND] = SERVER_ACK_MJPG_FRAME;
                jpg_buf[PKT_LENGTH]  = vid_data.length + PKT_MJPG_LENGTH;

                jpg_buf[PKT_MJPG_WIDTH]  = vid_data.mode.width;
                jpg_buf[PKT_MJPG_HEIGHT] = vid_data.mode.height;
                jpg_buf[PKT_MJPG_FPS]    = vid_data.mode.fps;
                
                send_packet(&g_client, jpg_buf, vid_data.length + PKT_MJPG_LENGTH);
                //syslog(LOG_DEBUG, "send frame size %lu, pkt size %lu\n",
                //       vid_data.length, vid_data.length + PKT_BASE_LENGTH);
                break;
            case CLIENT_REQ_SET_CTL_MODE:
                // interpret client's request for input mode change
                fc_get_vcm(&vcm_axes, &vcm_type);
                if (vcm_type == VCM_TYPE_KILL || vcm_type == VCM_TYPE_LOCKOUT)
                {
                    // if killswitch engaged or locked out, don't allow change
                    send_simple_packet(&g_client, SERVER_ACK_IGNORED);
                    continue;
                }

                switch (cmd_buffer[PKT_VCM_TYPE]) {
                case VCM_TYPE_RADIO:
                    syslog(LOG_DEBUG, "switching to radio control\n");
                    vcm_type = VCM_TYPE_RADIO;
                    vcm_axes = VCM_AXIS_ALL;
                    gpio_set_value(g_muxsel, MUX_SEL_RADIO);
                    break;
                case VCM_TYPE_AUTO:
                    syslog(LOG_DEBUG, "switching to autonomous control\n");
                    vcm_type = VCM_TYPE_AUTO;
                    vcm_axes = VCM_AXIS_ALL;
                    gpio_set_value(g_muxsel, MUX_SEL_UPROC);
                    break;
                case VCM_TYPE_MIXED:
                    syslog(LOG_DEBUG, "switching to remote control mode\n");
                    vcm_type = VCM_TYPE_MIXED;
                    vcm_axes = cmd_buffer[PKT_VCM_AXES];
                    gpio_set_value(g_muxsel, MUX_SEL_UPROC);
                    break;
                case VCM_TYPE_KILL:
                    syslog(LOG_DEBUG, "switching to killswitch enabled mode\n");
                    vcm_type = VCM_TYPE_KILL;
                    vcm_axes = VCM_AXIS_ALL;
                    gpio_set_value(g_muxsel, MUX_SEL_UPROC);
                    break;
                case VCM_TYPE_LOCKOUT:
                    // fall through - this isn't user specified
                default:
                    syslog(LOG_DEBUG, "bad control mode requested. ignoring\n");
                    send_simple_packet(&g_client, SERVER_ACK_IGNORED);
                    continue;
                }

                fc_set_vcm(vcm_axes, vcm_type);

                cmd_buffer[PKT_COMMAND]  = SERVER_UPDATE_CTL_MODE;
                cmd_buffer[PKT_LENGTH]   = PKT_VCM_LENGTH;
                cmd_buffer[PKT_VCM_TYPE] = vcm_type;
                cmd_buffer[PKT_VCM_AXES] = vcm_axes;
                send_packet(&g_client, cmd_buffer, PKT_VCM_LENGTH);
                break;
            case CLIENT_REQ_FLIGHT_CTL:
                // collect signals
                memcpy(&client_sigs.alt,   &cmd_buffer[PKT_MCM_AXIS_ALT],   4);
                memcpy(&client_sigs.pitch, &cmd_buffer[PKT_MCM_AXIS_PITCH], 4);
                memcpy(&client_sigs.roll,  &cmd_buffer[PKT_MCM_AXIS_ROLL],  4);
                memcpy(&client_sigs.yaw,   &cmd_buffer[PKT_MCM_AXIS_YAW],   4);

                //syslog(LOG_DEBUG, "Received controls: %f, %f, %f, %f\n",
                //        client_sigs.alt, client_sigs.pitch,
                //        client_sigs.roll, client_sigs.yaw);
                
                fc_set_ctl(&client_sigs);
                break;
            case CLIENT_REQ_CAM_TC:
                // determine whether color tracking is enabled or disabled
                tracking_enable(cmd_buffer[PKT_CAM_TC_ENABLE]);

                // update our color tracking parameters
                tc.r = cmd_buffer[PKT_CAM_TC_CH0];
                tc.g = cmd_buffer[PKT_CAM_TC_CH1];
                tc.b = cmd_buffer[PKT_CAM_TC_CH2];
                tc.ht = cmd_buffer[PKT_CAM_TC_TH0];
                tc.st = cmd_buffer[PKT_CAM_TC_TH1];
                tc.lt = cmd_buffer[PKT_CAM_TC_TH2];
                tc.filter = cmd_buffer[PKT_CAM_TC_FILTER];
                track_fps = cmd_buffer[PKT_CAM_TC_FPS];
                tracking_set_color(&tc);
                tracking_set_fps(track_fps);
                break;
             case CLIENT_REQ_CAM_COLORS:
                // determine whether color tracking is enabled or disabled
                // tracking_enable(cmd_buffer[PKT_CAM_TC_ENABLE]);
                tc = tracking_get_color();
                cmd_buffer[PKT_COMMAND]  = SERVER_UPDATE_COLOR;
                cmd_buffer[PKT_LENGTH]   = PKT_CAM_TC_LENGTH;
                // update our color tracking parameters
                cmd_buffer[PKT_CAM_TC_CH0] = tc.r;
                cmd_buffer[PKT_CAM_TC_CH1] = tc.g;
                cmd_buffer[PKT_CAM_TC_CH2] = tc.b;
                cmd_buffer[PKT_CAM_TC_TH0] = tc.ht;
                cmd_buffer[PKT_CAM_TC_TH1] = tc.st;
                cmd_buffer[PKT_CAM_TC_FILTER] = tc.filter;
                cmd_buffer[PKT_CAM_TC_FPS] = tracking_get_fps();
                send_packet(&g_client, cmd_buffer, PKT_CAM_TC_LENGTH);
                break;
            case CLIENT_REQ_CAM_DCI:
                // actual packet sending handled in callback routines
                if (!video_enum_devctrl(send_enum_ctrl, send_enum_menu)) {
                    syslog(LOG_ERR, "failed to enumerate device controls\n");
                    send_simple_packet(&g_client, SERVER_ACK_IGNORED);
                }
                break;
            case CLIENT_REQ_CAM_DCC:
                // attempt to set the device control, ignore if unsupported
                if (!video_set_devctrl(cmd_buffer[PKT_CAM_DCC_ID],
                                       cmd_buffer[PKT_CAM_DCC_VALUE])) {
                    syslog(LOG_ERR, "failed to set device control\n");
                    send_simple_packet(&g_client, SERVER_ACK_IGNORED);
                }
                break;
            case CLIENT_REQ_STS:
                // update trim for the specified axis
                fc_set_trims(cmd_buffer[PKT_STS_AXES],
                             cmd_buffer[PKT_STS_VALUE]);
                break;
            case CLIENT_REQ_GTS:
                // respond to client's request for axis trim settings
                cmd_buffer[PKT_COMMAND]   = SERVER_ACK_GTS;
                cmd_buffer[PKT_LENGTH]    = PKT_GTS_LENGTH;
                cmd_buffer[PKT_GTS_YAW]   = fc_get_trim(VCM_AXIS_YAW);
                cmd_buffer[PKT_GTS_PITCH] = fc_get_trim(VCM_AXIS_PITCH);
                cmd_buffer[PKT_GTS_ROLL]  = fc_get_trim(VCM_AXIS_ROLL);
                cmd_buffer[PKT_GTS_ALT]   = fc_get_trim(VCM_AXIS_ALT);
                send_packet(&g_client, cmd_buffer, PKT_GTS_LENGTH);
                break;
            case CLIENT_REQ_SFS:
                // respond to client's request to adjust filter settings
                samples = cmd_buffer[PKT_SFS_SAMPLES];
                switch (cmd_buffer[PKT_SFS_SIGNAL]) {
                case SFS_IMU:
                    // set averaging filter samples for imu angles
                    syslog(LOG_INFO, "set %d imu filter samples", samples);
                    imu_set_avg_filter(&g_imu, samples);
                    break;
                case SFS_ALT:
                    // set averaging filter samples for altitude pwm
                    syslog(LOG_INFO, "set %d alt filter samples", samples);
                    gpio_event_set_filter(&g_gpio_alt, samples);
                    break;
                case SFS_AUX:
                    // set averaging filter samples for auxiliary pwm
                    syslog(LOG_INFO, "set %d aux filter samples", samples);
                    gpio_event_set_filter(&g_gpio_aux, samples);
                    break;
                case SFS_BATT:
                    // set averaging filter for battery adc
                    syslog(LOG_INFO, "set %d batt filter samples", samples);
                    send_simple_packet(&g_client, SERVER_ACK_IGNORED);
                    break;
                }
                break;
            case CLIENT_REQ_GFS:
                // respond to client's request for current filter settings
                cmd_buffer[PKT_COMMAND]  = SERVER_ACK_GFS;
                cmd_buffer[PKT_LENGTH]   = PKT_GFS_LENGTH;
                cmd_buffer[PKT_GFS_IMU]  = imu_get_avg_filter(&g_imu);
                cmd_buffer[PKT_GFS_ALT]  = gpio_event_get_filter(&g_gpio_alt);
                cmd_buffer[PKT_GFS_AUX]  = gpio_event_get_filter(&g_gpio_aux);
                cmd_buffer[PKT_GFS_BATT] = 0;
                send_packet(&g_client, cmd_buffer, PKT_GFS_LENGTH);
                break;
            case CLIENT_REQ_SPIDS:
                // update specific PID parameter in flight control
                memcpy(&pid_val, &cmd_buffer[PKT_SPIDS_VALUE], 4);
                fc_set_pid_param(cmd_buffer[PKT_SPIDS_AXIS],
                                 cmd_buffer[PKT_SPIDS_PARAM], pid_val);
                break;
            case CLIENT_REQ_GPIDS:
                // respond to client's request for current PID parameters
                fc_get_pid_params(cmd_buffer[PKT_GPIDS_AXIS], pid_params);
                cmd_buffer[PKT_COMMAND]  = SERVER_ACK_GPIDS;
                cmd_buffer[PKT_LENGTH]   = PKT_GPIDS_LENGTH;

                // pack PID Kp/Ki/Kd into buffer
                memcpy(&cmd_buffer[PKT_GPIDS_KP], &pid_params[PID_PARAM_KP], 4);
                memcpy(&cmd_buffer[PKT_GPIDS_KI], &pid_params[PID_PARAM_KI], 4);
                memcpy(&cmd_buffer[PKT_GPIDS_KD], &pid_params[PID_PARAM_KD], 4);
                memcpy(&cmd_buffer[PKT_GPIDS_SP], &pid_params[PID_PARAM_SP], 4);
                send_packet(&g_client, cmd_buffer, PKT_GPIDS_LENGTH);

                fprintf(stderr, "sent: p = %f, i = %f, d = %f, set = %f for axis: %d\n", 
                        pid_params[PID_PARAM_KP], pid_params[PID_PARAM_KI],
                        pid_params[PID_PARAM_KD], pid_params[PID_PARAM_SP],
                        cmd_buffer[PKT_GPIDS_AXIS]);
                break;
            default:
                // dump a reasonable number of entries for debugging purposes
                syslog(LOG_ERR, "invalid client command (C:%d L:%d 0:%d 1:%d)",
                       cmd_buffer[PKT_COMMAND],
                       cmd_buffer[PKT_LENGTH],
                       cmd_buffer[PKT_BASE + 0],
                       cmd_buffer[PKT_BASE + 1]);
                send_simple_packet(&g_client, SERVER_ACK_IGNORED);
                goto client_disconnect;
            }
        }

        // perform cleanup -- disconnect and wait for next connection
client_disconnect:
        syslog(LOG_INFO, "disconnected from client (%d)", g_client.fd);
        close_client(&g_client);
    }
}

// -----------------------------------------------------------------------------
// Display program usage message.
void print_usage()
{
    printf("usage: uav_control [options]\n\n"
           "Program options:\n"
           "  -c, --capture=PATH        capture and store control inputs\n"
           "  -r, --replay=PATH         set autonomous replay from path\n"
           "  -s, --stty_dev=DEV        specify serial device for IMU\n"
           "  -v, --v4l_dev=DEV         specify video device for webcam\n"
           "  -p, --port=NUM            specify port for network socket\n"
           "  -m, --mux=NUM             specify gpio for mux select line\n"
           "  -u, --ultrasonic=NUM      specify gpio for ultrasonic pwm\n"
           "  -o, --override=NUM        specify gpio for override pwm\n"
           "  -x, --width=NUM           specify resolution width for webcam\n"
           "  -y, --height=NUM          specify resolution height for webcam\n"
           "  -f, --fps=NUM             specify capture framerate for webcam\n"
           "  -F, --track-fps=NUM       specify color tracking framerate\n"
           "      --(axis)-(value)=NUM  specify value to set for an axis\n"
           "                            (axis)   = yaw, pitch, roll, alt\n"
           "                            (values) = trim, kp, ki, kd, sp\n"
           /*
           "      --yaw-trim=NUM        specify yaw axis TRIM value\n"
           "      --yaw-kp=NUM          specify yaw axis KP value\n"
           "      --yaw-ki=NUM          specify yaw axis KI value\n"
           "      --yaw-kd=NUM          specify yaw axis KD value\n"
           "      --yaw-sp=NUM          specify yaw axis setpoint value\n"
           "      --pitch-trim=NUM      specify pitch axis TRIM value\n"
           "      --pitch-kp=NUM        specify pitch axis KP value\n"
           "      --pitch-ki=NUM        specify pitch axis KI value\n"
           "      --pitch-kd=NUM        specify pitch axis KD value\n"
           "      --pitch-sp=NUM        specify pitch axis setpoint value\n"
           "      --roll-trim=NUM       specify roll axis TRIM value\n"
           "      --roll-kp=NUM         specify roll axis KP value\n"
           "      --roll-ki=NUM         specify roll axis KI value\n"
           "      --roll-kd=NUM         specify roll axis KD value\n"
           "      --roll-sp=NUM         specify roll axis setpoint value\n"
           "      --alt-trim=NUM        specify alt axis TRIM value\n"
           "      --alt-kp=NUM          specify alt axis KP value\n"
           "      --alt-ki=NUM          specify alt axis KI value\n"
           "      --alt-kd=NUM          specify alt axis KD value\n"  
           "      --alt-sp=NUM          specify alt axis setpoint value\n"
           */              
           "  -D, --daemonize           run as a background process\n"
           "  -V, --verbose             enable verbose logging\n"
           "  -h, --help                display this usage message\n"
           "      --no-adc              do not capture data from adc\n"
           "      --no-fc               do not enable autonomous flight\n"
           "      --no-gpio             do not perform any gpio processing\n"
           "      --no-track            do not perform color tracking\n"
           "      --no-video            do not capture video from webcam\n");
}

// -----------------------------------------------------------------------------
// Program entry point -- process command line arguments and initialize daemon.
int main(int argc, char *argv[])
{
    int index, opt, log_opt, baud = B57600;
    int flag_verbose = 0, flag_daemonize = 0;
    int flag_no_adc = 0, flag_no_video = 0, flag_no_gpio = 0, flag_no_track = 0;
    int flag_no_fc = 0;
    int arg_port = 8090, arg_width = 320, arg_height = 240;
    int arg_fps = 15, arg_track_fps = 5;
    int arg_mux = 170, arg_ultrasonic = 171, arg_override = 172;
    int    arg_yaw_trim = 0, arg_pitch_trim = 0, arg_roll_trim = 0, arg_alt_trim = 0;
    double arg_yaw_kp   = 0, arg_pitch_kp   = 0, arg_roll_kp   = 0, arg_alt_kp   = 0;
    double arg_yaw_ki   = 0, arg_pitch_ki   = 0, arg_roll_ki   = 0, arg_alt_ki   = 0;
    double arg_yaw_kd   = 0, arg_pitch_kd   = 0, arg_roll_kd   = 0, arg_alt_kd   = 0;
    double arg_yaw_sp   = PID_YAW_DEF_SP;
    double arg_pitch_sp = PID_PITCH_DEF_SP;
    double arg_roll_sp  = PID_ROLL_DEF_SP;
    double arg_alt_sp   = PID_ALT_DEF_SP;
    char port_str[DEV_LEN];
    char stty_dev[DEV_LEN] = "/dev/ttyS0";
    char v4l_dev[DEV_LEN] = "/dev/video0";
    char *capture_path = NULL, *replay_path = NULL;

    #define OPTION_START    2000
    #define YAW_TRIM        OPTION_START + 0
    #define YAW_KP          OPTION_START + 1
    #define YAW_KI          OPTION_START + 2
    #define YAW_KD          OPTION_START + 3
    #define YAW_SP          OPTION_START + 4
    #define PITCH_TRIM      OPTION_START + 5
    #define PITCH_KP        OPTION_START + 6
    #define PITCH_KI        OPTION_START + 7
    #define PITCH_KD        OPTION_START + 8
    #define PITCH_SP        OPTION_START + 9
    #define ROLL_TRIM       OPTION_START + 10
    #define ROLL_KP         OPTION_START + 11
    #define ROLL_KI         OPTION_START + 12
    #define ROLL_KD         OPTION_START + 13
    #define ROLL_SP         OPTION_START + 14
    #define ALT_TRIM        OPTION_START + 15
    #define ALT_KP          OPTION_START + 16
    #define ALT_KI          OPTION_START + 17
    #define ALT_KD          OPTION_START + 18
    #define ALT_SP          OPTION_START + 19
    
    struct option long_options[] = {
        { "capture",    required_argument, NULL, 'c' },
        { "replay",     required_argument, NULL, 'r' },
        { "stty_dev",   required_argument, NULL, 's' },
        { "v4l_dev",    required_argument, NULL, 'v' },
        { "port",       required_argument, NULL, 'p' },
        { "mux",        required_argument, NULL, 'm' },
        { "ultrasonic", required_argument, NULL, 'u' },
        { "override",   required_argument, NULL, 'o' },
        { "width",      required_argument, NULL, 'x' },
        { "height",     required_argument, NULL, 'y' },
        { "fps",        required_argument, NULL, 'f' },
        { "track-fps",  required_argument, NULL, 'F' },

        { "yaw-trim",   required_argument, NULL, YAW_TRIM },
        { "yaw-kp",     required_argument, NULL, YAW_KP },
        { "yaw-ki",     required_argument, NULL, YAW_KI },
        { "yaw-kd",     required_argument, NULL, YAW_KD },
        { "yaw-sp",     required_argument, NULL, YAW_SP },
        { "pitch-trim", required_argument, NULL, PITCH_TRIM },
        { "pitch-kp",   required_argument, NULL, PITCH_KP },
        { "pitch-ki",   required_argument, NULL, PITCH_KI },
        { "pitch-kd",   required_argument, NULL, PITCH_KD },
        { "pitch-sp",   required_argument, NULL, PITCH_SP },
        { "roll-trim",  required_argument, NULL, ROLL_TRIM },
        { "roll-kp",    required_argument, NULL, ROLL_KP },
        { "roll-ki",    required_argument, NULL, ROLL_KI },
        { "roll-kd",    required_argument, NULL, ROLL_KD },
        { "roll-sp",    required_argument, NULL, ROLL_SP },
        { "alt-trim",   required_argument, NULL, ALT_TRIM },
        { "alt-kp",     required_argument, NULL, ALT_KP },
        { "alt-ki",     required_argument, NULL, ALT_KI },
        { "alt-kd",     required_argument, NULL, ALT_KD },
        { "alt-sp",     required_argument, NULL, ALT_SP },

        { "daemonize",  no_argument,       NULL, 'D' },
        { "verbose",    no_argument,       NULL, 'V' },
        { "help",       no_argument,       NULL, 'h' },
        { "no-adc",     no_argument,       &flag_no_adc,   1 },
        { "no-fc",      no_argument,       &flag_no_fc,    1 },
        { "no-gpio",    no_argument,       &flag_no_gpio,  1 },
        { "no-track",   no_argument,       &flag_no_track, 1 },
        { "no-video",   no_argument,       &flag_no_video, 1 },
        { 0, 0, 0, 0 }
    };

    static const char *str = "c:r:s:v:p:m:u:o:x:y:f:F:Y:P:R:A:DVh?";

    while (-1 != (opt = getopt_long(argc, argv, str, long_options, &index))) {
        switch (opt) {
        case YAW_TRIM:
            arg_yaw_trim = atof(optarg);
            break;
        case YAW_KP:
            arg_yaw_kp = atof(optarg);
            break;
        case YAW_KI:
            arg_yaw_ki = atof(optarg);
            break;
        case YAW_KD:
            arg_yaw_kd = atof(optarg);
            break;
        case YAW_SP:
            arg_yaw_sp = atof(optarg);
            break;

        case PITCH_TRIM:
            arg_pitch_trim = atof(optarg);
            break;
        case PITCH_KP:
            arg_pitch_kp = atof(optarg);
            break;
        case PITCH_KI:
            arg_pitch_ki = atof(optarg);
            break;
        case PITCH_KD:
            arg_pitch_kd = atof(optarg);
            break;
        case PITCH_SP:
            arg_pitch_sp = atof(optarg);
            break;

        case ROLL_TRIM:
            arg_roll_trim = atof(optarg);
            break;
        case ROLL_KP:
            arg_roll_kp = atof(optarg);
            break;
        case ROLL_KI:
            arg_roll_ki = atof(optarg);
            break;
        case ROLL_KD:
            arg_roll_kd = atof(optarg);
            break;
        case ROLL_SP:
            arg_roll_sp = atof(optarg);
            break;

        case ALT_TRIM:
            arg_alt_trim = atof(optarg);
            break;  
        case ALT_KP:
            arg_alt_kp = atof(optarg);
            break;
        case ALT_KI:
            arg_alt_ki = atof(optarg);
            break;
        case ALT_KD:
            arg_alt_kd = atof(optarg);
            break;
        case ALT_SP:
            arg_alt_sp = atof(optarg);
            break;

        case 'c':
            capture_path = strdup(optarg);
            break;
        case 'r':
            replay_path = strdup(optarg);
            break;
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
        case 'F':
            arg_track_fps = atoi(optarg);
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

    // attempt to initialize the flight control subsystem
    if (!flag_no_fc) {
        syslog(LOG_INFO, "opening flight control\n");
        if (!fc_init(&g_gpio_alt, &g_imu)) {
            syslog(LOG_ERR, "failed to open flight control\n");
            uav_shutdown(EXIT_FAILURE);
        }

        // initialize flight control trims (default zero)
        fc_set_trims(VCM_AXIS_YAW,   arg_yaw_trim);
        fc_set_trims(VCM_AXIS_PITCH, arg_pitch_trim);
        fc_set_trims(VCM_AXIS_ROLL,  arg_roll_trim);
        fc_set_trims(VCM_AXIS_ALT,   arg_alt_trim);

        fc_set_pid_param(VCM_AXIS_YAW,   PID_PARAM_KP, arg_yaw_kp); 
        fc_set_pid_param(VCM_AXIS_YAW,   PID_PARAM_KI, arg_yaw_ki); 
        fc_set_pid_param(VCM_AXIS_YAW,   PID_PARAM_KD, arg_yaw_kd); 
        fc_set_pid_param(VCM_AXIS_YAW,   PID_PARAM_SP, arg_yaw_sp);

        fc_set_pid_param(VCM_AXIS_PITCH, PID_PARAM_KP, arg_pitch_kp); 
        fc_set_pid_param(VCM_AXIS_PITCH, PID_PARAM_KI, arg_pitch_ki); 
        fc_set_pid_param(VCM_AXIS_PITCH, PID_PARAM_KD, arg_pitch_kd); 
        fc_set_pid_param(VCM_AXIS_PITCH, PID_PARAM_SP, arg_pitch_sp);

        fc_set_pid_param(VCM_AXIS_ROLL,  PID_PARAM_KP, arg_roll_kp); 
        fc_set_pid_param(VCM_AXIS_ROLL,  PID_PARAM_KI, arg_roll_ki); 
        fc_set_pid_param(VCM_AXIS_ROLL,  PID_PARAM_KD, arg_roll_kd); 
        fc_set_pid_param(VCM_AXIS_ROLL,  PID_PARAM_SP, arg_roll_sp);
        
        fc_set_pid_param(VCM_AXIS_ALT,   PID_PARAM_KP, arg_alt_kp); 
        fc_set_pid_param(VCM_AXIS_ALT,   PID_PARAM_KI, arg_alt_ki); 
        fc_set_pid_param(VCM_AXIS_ALT,   PID_PARAM_KD, arg_alt_kd); 
        fc_set_pid_param(VCM_AXIS_ALT,   PID_PARAM_SP, arg_alt_sp);

        // check if we're capturing or replaying a trace
        if (capture_path && replay_path) {
            // does not make sense to specify both at once
            syslog(LOG_ERR, "cannot specify both --capture and --replay\n");
            uav_shutdown(EXIT_FAILURE);
        }
        else if (capture_path) {
            // tell flight_control to capture input for mixed mode control
            syslog(LOG_INFO, "enabling capture mode for flight control\n");
            fc_set_capture(capture_path);
        }
        else if (replay_path) {
            // tell flight_control to replay stored input signals
            syslog(LOG_INFO, "enabling replay mode for flight control\n");
            fc_set_replay(replay_path);
        }
    } 
    else {
        if (capture_path || replay_path) {
            // warn the user but don't bother failing
            syslog(LOG_ERR, "cannot capture or replay --no-fc (ignoring)\n");
        }
    }

    // initialize the gpio subsystem(s) unless specified not to (no-gpio)
    if (!flag_no_gpio) {
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

        // attempt to initialize gpio pin for multiplexer select (output)
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
            syslog(LOG_ERR, "failed to set gpio %d direction to out", arg_mux);
            uav_shutdown(EXIT_FAILURE);
        }

        // initialize the auxiliary pwm thread
        if (0 != pthread_create(&g_aux_thread, NULL, aux_trigger_thread, 0)) {
            syslog(LOG_ERR, "failed to create aux trigger thread");
            uav_shutdown(EXIT_FAILURE);
        }

        // start with mux select on computer control (we start in autonomous)
        gpio_set_value(g_muxsel, MUX_SEL_UPROC);
    }

    // initialize video subsystem unless specified not to (no-video)
    if (!flag_no_video) {
        video_mode_t mode = { arg_width, arg_height, arg_fps };
        syslog(LOG_INFO, "opening and configuring v4l device '%s'\n", v4l_dev);
        if (!video_init(v4l_dev, &mode)) {
            syslog(LOG_ERR, "failed to initialize video subsystem\n");
            uav_shutdown(EXIT_FAILURE);
        }
    }

    // initialize color tracking subsystem
    if (!flag_no_track) {
        if (!flag_no_video) {
            // make a note that tracking isn't possible with video
            syslog(LOG_INFO, "initializing color tracking subsystem\n");
            if (!tracking_init(&g_client)) {
                syslog(LOG_ERR, "failed to initialize tracking subsystem\n");
                uav_shutdown(EXIT_FAILURE);
            }

            // set the initial tracking framerate (not tied to webcam framerate)
            tracking_set_fps(arg_track_fps);
        }
        else {
            syslog(LOG_INFO, "color tracking not possible without video");
        }
    }

    // initialize adc channels for battery monitoring
    if (!flag_no_adc) {
        syslog(LOG_INFO, "opening adc channels for battery monitoring\n");
        if (0 > adc_open_channels()) {
            syslog(LOG_ERR, "failed to open adc channels\n");
            uav_shutdown(EXIT_FAILURE);
        }
    }
    
    // server entry point
    run_server(&g_imu, port_str);
    
    uav_shutdown(EXIT_SUCCESS);
    return EXIT_SUCCESS;
}

