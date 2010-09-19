// -----------------------------------------------------------------------------
// Definitions for communication between gumstix and uvc webcam.
// Garrett Smith 2010
// -----------------------------------------------------------------------------

#ifndef _UAV_VIDEO_UVC__H_
#define _UAV_VIDEO_UVC__H_

#include "mjpg-streamer/v4l2uvc.h"
#include "mjpg-streamer/huffman.h"
#include "mjpg-streamer/jpeg_utils.h"

int video_init(const char *dev, int width, int height, int fps);
void video_lock(const char **data, unsigned long *length);
void video_unlock();

#endif // _UAV_VIDEO_UVC__H_

