// -----------------------------------------------------------------------------
// File:    video_uvc.h
// Authors: Garrett Smith, Kevin Macksamie, Tyler Thierolf, Timothy Miller
// Created: 09-18-2010
//
// Definitions for communication between gumstix and uvc webcam.
// -----------------------------------------------------------------------------

#ifndef _UAV_VIDEO_UVC__H_
#define _UAV_VIDEO_UVC__H_

#include "v4l2uvc.h"

typedef struct video_mode
{
    unsigned int width;     // frame width in pixels
    unsigned int height;    // frame height in pixels
    unsigned int fps;       // frames per second
} video_mode_t;

typedef struct video_data
{
    uint8_t     *data;      // global buffer
    size_t       length;    // global buffer size
    video_mode_t mode;      // current video mode
} video_data_t;

typedef enum lock_type {
    LOCK_ASYNC,
    LOCK_SYNC
} lock_type_t;

// initialize the video subsystem, given the specified resolution and framerate
int video_init(const char *dev, video_mode_t *mode);

// shutdown the video capture subsystem
void video_shutdown(void);

// lock the current frame, and return pointer to its buffer and dimensions
int video_lock(video_data_t *data, lock_type_t lock_flag);

// release our lock on the current frame's resources
void video_unlock();

void video_condition_wait();

// override the current video mode (resolution, framerate, etc)
int video_set_mode(video_mode_t *mode);

// get an enumerated list of supported v4l/uvc device controls
int video_enum_devctrl(enum_ctrl_fn c_fn, enum_menu_fn m_fn);

// set the specified device control value
int video_set_devctrl(int id, int value);

// get the specified device control value
int video_get_devctrl(int id);

int video_get_fps();

#endif // _UAV_VIDEO_UVC__H_

