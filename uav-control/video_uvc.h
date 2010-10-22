// -----------------------------------------------------------------------------
// File:    video_uvc.h
// Authors: Garrett Smith, Kevin Macksamie, Tyler Thierolf, Timothy Miller
// Created: 09-18-2010
//
// Definitions for communication between gumstix and uvc webcam.
// -----------------------------------------------------------------------------

#ifndef _UAV_VIDEO_UVC__H_
#define _UAV_VIDEO_UVC__H_

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

// initialize the video subsystem, given the specified resolution and framerate
int video_init(const char *dev, video_mode_t *mode);

// shutdown the video capture subsystem
void video_shutdown(void);

// lock the current frame, and return pointer to its buffer and dimensions
int video_lock(video_data_t *data, int type);

// release our lock on the current frame's resources
void video_unlock();

// override the current video mode (resolution, framerate, etc)
int video_set_mode(video_mode_t *mode);

// configure the camera's exposure. if automatic is set, abs_value is ignored
int video_set_exposure(int automatic, int abs_value);

// configure the camera's focus. if automatic is set, abs_value is ignored
int video_set_focus(int automatic, int abs_value);

// configure the camera's white balance. if automatic is not set, set balance
int video_set_whitebalance(int automatic);

#endif // _UAV_VIDEO_UVC__H_

