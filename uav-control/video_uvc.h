// -----------------------------------------------------------------------------
// Definitions for communication between gumstix and uvc webcam.
// Garrett Smith 2010
// -----------------------------------------------------------------------------

#ifndef _UAV_VIDEO_UVC__H_
#define _UAV_VIDEO_UVC__H_

#include "mjpg-streamer/v4l2uvc.h"
#include "mjpg-streamer/huffman.h"
#include "mjpg-streamer/jpeg_utils.h"

typedef struct _video_data video_data;
struct _video_data
{
    const char *data;      /* global buffer */
    unsigned long length;  /* global buffer size */
    int width, height;     /* frame resolution */
    int fps;               /* frame rate */
};

int video_init(const char *dev, int width, int height, int fps);
void video_shutdown(void);
void video_lock(video_data *vdata);
void video_unlock();

#endif // _UAV_VIDEO_UVC__H_

