// -----------------------------------------------------------------------------
// File:    cmdline.c
// Authors: Garrett Smith, Kevin Macksamie
// Created: 10-28-2010
// 
// Routines for command line processing.
// -----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>
#include <syslog.h>
#include "cmdline.h"

// -----------------------------------------------------------------------------
// Display program usage message.
static void print_usage(void)
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
           "  -D, --daemonize           run as a background process\n"
           "  -V, --verbose             enable verbose logging\n"
           "  -h, --help                display this usage message\n"
           "      --[AXIS]-[PARAM]=NUM  specify value to set for an axis\n"
           "                              AXIS = yaw | pitch | roll | alt\n"
           "                              PARAM = trim | kp | ki | kd | sp\n"
           "      --no-adc              do not capture data from adc\n"
           "      --no-fc               do not enable autonomous flight\n"
           "      --no-gpio             do not perform any gpio processing\n"
           "      --no-track            do not perform color tracking\n"
           "      --no-video            do not capture video from webcam\n");
}

// -----------------------------------------------------------------------------
int cmdline_parse(int argc, char *argv[], cmdline_opts_t *opts)
{
    int index, opt;
    static const char *str = "c:r:s:v:p:m:u:o:x:y:f:F:Y:P:R:A:DVh?";
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
        { "yaw-trim",   required_argument, NULL, OPT_YAW_TRIM },
        { "yaw-kp",     required_argument, NULL, OPT_YAW_KP },
        { "yaw-ki",     required_argument, NULL, OPT_YAW_KI },
        { "yaw-kd",     required_argument, NULL, OPT_YAW_KD },
        { "yaw-sp",     required_argument, NULL, OPT_YAW_SP },
        { "pitch-trim", required_argument, NULL, OPT_PITCH_TRIM },
        { "pitch-kp",   required_argument, NULL, OPT_PITCH_KP },
        { "pitch-ki",   required_argument, NULL, OPT_PITCH_KI },
        { "pitch-kd",   required_argument, NULL, OPT_PITCH_KD },
        { "pitch-sp",   required_argument, NULL, OPT_PITCH_SP },
        { "roll-trim",  required_argument, NULL, OPT_ROLL_TRIM },
        { "roll-kp",    required_argument, NULL, OPT_ROLL_KP },
        { "roll-ki",    required_argument, NULL, OPT_ROLL_KI },
        { "roll-kd",    required_argument, NULL, OPT_ROLL_KD },
        { "roll-sp",    required_argument, NULL, OPT_ROLL_SP },
        { "alt-trim",   required_argument, NULL, OPT_ALT_TRIM },
        { "alt-kp",     required_argument, NULL, OPT_ALT_KP },
        { "alt-ki",     required_argument, NULL, OPT_ALT_KI },
        { "alt-kd",     required_argument, NULL, OPT_ALT_KD },
        { "alt-sp",     required_argument, NULL, OPT_ALT_SP },
        { "daemonize",  no_argument,       NULL, 'D' },
        { "verbose",    no_argument,       NULL, 'V' },
        { "help",       no_argument,       NULL, 'h' },
        { "no-adc",     no_argument,       &opts->no_adc,   1 },
        { "no-fc",      no_argument,       &opts->no_fc,    1 },
        { "no-gpio",    no_argument,       &opts->no_gpio,  1 },
        { "no-track",   no_argument,       &opts->no_track, 1 },
        { "no-video",   no_argument,       &opts->no_video, 1 },
        { 0, 0, 0, 0 }
    };

    // set default parameter values
    memset(opts, 0, sizeof(cmdline_opts_t));
    strcpy(opts->stty_dev, "/dev/ttyS0");
    strcpy(opts->v4l_dev, "/dev/video0");

    opts->port       = DEFAULT_PORT;
    opts->vid_width  = DEFAULT_WIDTH;
    opts->vid_height = DEFAULT_HEIGHT;
    opts->vid_fps    = DEFAULT_FPS;
    opts->track_fps  = DEFAULT_TRACK;
    opts->mux        = DEFAULT_GPIO_MUX;
    opts->uss        = DEFAULT_GPIO_USS;
    opts->ovr        = DEFAULT_GPIO_OVR;

    while (-1 != (opt = getopt_long(argc, argv, str, long_options, &index))) {
        switch (opt) {
        case OPT_YAW_TRIM:
        case OPT_YAW_KP:
        case OPT_YAW_KI:
        case OPT_YAW_KD:
        case OPT_YAW_SP:
            opts->yaw[opt - OPT_YAW_TRIM] = atof(optarg);
            break;
        case OPT_PITCH_TRIM:
        case OPT_PITCH_KP:
        case OPT_PITCH_KI:
        case OPT_PITCH_KD:
        case OPT_PITCH_SP:
            opts->pitch[opt - OPT_PITCH_TRIM] = atof(optarg);
            break;
        case OPT_ROLL_TRIM:
        case OPT_ROLL_KP:
        case OPT_ROLL_KI:
        case OPT_ROLL_KD:
        case OPT_ROLL_SP:
            opts->roll[opt - OPT_ROLL_TRIM] = atof(optarg);
            break;
        case OPT_ALT_TRIM:
        case OPT_ALT_KP:
        case OPT_ALT_KI:
        case OPT_ALT_KD:
        case OPT_ALT_SP:
            opts->alt[opt - OPT_ALT_TRIM] = atof(optarg);
            break;
        case 'c':
            opts->capture_path = strdup(optarg);
            break;
        case 'r':
            opts->replay_path = strdup(optarg);
            break;
        case 's':
            strncpy(opts->stty_dev, optarg, DEV_LEN);
            break;
        case 'v':
            strncpy(opts->v4l_dev, optarg, DEV_LEN);
            break;
        case 'p':
            opts->port = atoi(optarg);
            break;
        case 'm':
            opts->mux = atoi(optarg);
            break;
        case 'u':
            opts->uss = atoi(optarg);
            break;
        case 'o':
            opts->ovr = atoi(optarg);
            break;
        case 'x':
            opts->vid_width = atoi(optarg);
            break;
        case 'y':
            opts->vid_height = atoi(optarg);
            break;
        case 'f':
            opts->vid_fps = atoi(optarg);
            break;
        case 'F':
            opts->track_fps = atoi(optarg);
            break;
        case 'D':
            opts->daemonize = 1;
            break;
        case 'V':
            opts->verbose = 1;
            break;
        case 'h': // fall through
        case '?':
            print_usage();
            return 0;
        case 0:
            break;
        default:
            syslog(LOG_ERR, "unexpected argument '%c'\n", opt);
            print_usage();
            assert(!"unhandled case in option handling -- this is an error");
            return 0;
        }
    }

    return 1;
}

