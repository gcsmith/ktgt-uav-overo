// -----------------------------------------------------------------------------
// File:    video_uvc.h
// Authors: Garrett Smith, Kevin Macksamie, Tyler Thierolf, Timothy Miller
// Created: 09-18-2010
//
// Definitions for communication between gumstix and uvc webcam.
// -----------------------------------------------------------------------------

#ifndef _UAV_VIDEO_UVC__H_
#define _UAV_VIDEO_UVC__H_

typedef struct video_data
{
    const char *data;       // global buffer
    unsigned long length;   // global buffer size
    int width, height;      // frame resolution
    int fps;                // frame rate
} video_data_t;

// initialize the video subsystem, given the specified resolution and framerate
int video_init(const char *dev, int width, int height, int fps);

// shutdown the video capture subsystem
void video_shutdown(void);

// configure the camera's exposure. if automatic is set, abs_value is ignored
int video_cfg_exposure(int automatic, int abs_value);

// configure the camera's focus. if automatic is set, abs_value is ignored
int video_cfg_focus(int automatic, int abs_value);

// configure the camera's white balance. if automatic is not set, set balance
int video_cfg_whitebalance(int automatic);

// lock the current frame, and return pointer to its buffer and dimensions
int video_lock(video_data_t *vdata, int type);

// unlock the previously locked frame
void video_unlock();

#endif // _UAV_VIDEO_UVC__H_

