// -----------------------------------------------------------------------------
// File:    cmdline.h
// Authors: Garrett Smith, Kevin Macksamie
// Created: 10-28-2010
// 
// Routines for command line processing.
// -----------------------------------------------------------------------------

#ifndef _UAV_CMDLINE__H_
#define _UAV_CMDLINE__H_

// default values for unspecified parameters

#define DEFAULT_PORT        8090
#define DEFAULT_WIDTH       320
#define DEFAULT_HEIGHT      240
#define DEFAULT_FPS         15
#define DEFAULT_TRACK       5

#define DEFAULT_GPIO_MUX    170
#define DEFAULT_GPIO_USS    171
#define DEFAULT_GPIO_OVR    172

#define DEFAULT_YAW_TRIM    710.0f
#define DEFAULT_YAW_KP      0.00300f
#define DEFAULT_YAW_KI      0.00010f
#define DEFAULT_YAW_KD      0.04000f
#define DEFAULT_YAW_SP      0.0f

#define DEFAULT_PITCH_TRIM  -2676.0f
#define DEFAULT_PITCH_KP    0.00500f
#define DEFAULT_PITCH_KI    0.00010f
#define DEFAULT_PITCH_KD    0.06000f
#define DEFAULT_PITCH_SP    -2.0f

#define DEFAULT_ROLL_TRIM   1033.0f
#define DEFAULT_ROLL_KP     0.00500f
#define DEFAULT_ROLL_KI     0.00010f
#define DEFAULT_ROLL_KD     0.06000f
#define DEFAULT_ROLL_SP     -0.5f

#define DEFAULT_ALT_TRIM    0.0f
#define DEFAULT_ALT_KP      0.00300f
#define DEFAULT_ALT_KI      0.00010f
#define DEFAULT_ALT_KD      0.04000f
#define DEFAULT_ALT_SP      42.0f

// identifiers for long options without short equivalents

#define OPT_FIRST       2000
#define OPT_YAW_TRIM    (OPT_FIRST + 0)
#define OPT_YAW_KP      (OPT_FIRST + 1)
#define OPT_YAW_KI      (OPT_FIRST + 2)
#define OPT_YAW_KD      (OPT_FIRST + 3)
#define OPT_YAW_SP      (OPT_FIRST + 4)
#define OPT_PITCH_TRIM  (OPT_FIRST + 5)
#define OPT_PITCH_KP    (OPT_FIRST + 6)
#define OPT_PITCH_KI    (OPT_FIRST + 7)
#define OPT_PITCH_KD    (OPT_FIRST + 8)
#define OPT_PITCH_SP    (OPT_FIRST + 9)
#define OPT_ROLL_TRIM   (OPT_FIRST + 10)
#define OPT_ROLL_KP     (OPT_FIRST + 11)
#define OPT_ROLL_KI     (OPT_FIRST + 12)
#define OPT_ROLL_KD     (OPT_FIRST + 13)
#define OPT_ROLL_SP     (OPT_FIRST + 14)
#define OPT_ALT_TRIM    (OPT_FIRST + 15)
#define OPT_ALT_KP      (OPT_FIRST + 16)
#define OPT_ALT_KI      (OPT_FIRST + 17)
#define OPT_ALT_KD      (OPT_FIRST + 18)
#define OPT_ALT_SP      (OPT_FIRST + 19)

#define DEV_LEN 64

typedef struct cmdline_opts {
    int verbose;
    int daemonize;
    int no_adc;
    int no_video;
    int no_gpio;
    int no_track;
    int no_fc;
    int port;
    int vid_width;
    int vid_height;
    int vid_fps;
    int track_fps;
    int mux;
    int uss;
    int ovr;
    float yaw[5];
    float pitch[5];
    float roll[5];
    float alt[5];
    char port_str[DEV_LEN];
    char stty_dev[DEV_LEN];
    char v4l_dev[DEV_LEN];
    char *capture_path;
    char *replay_path;
} cmdline_opts_t;

int cmdline_parse(int argc, char *argv[], cmdline_opts_t *opts);

#endif // _UAV_CMDLINE__H_

