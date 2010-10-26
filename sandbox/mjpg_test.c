#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include <syslog.h>
#include <stdint.h>
#include "video_uvc.h"

int g_running = 1;

void sig_handler(int id)
{
    // Stop mjpg-streamer
    // mjpg_streamer_stop(0);
    video_shutdown();
    g_running = 0;
}

int main (int argc, char **argv)
{
    const int width = 320;
    const int height = 240;
    const int fps = 15;
    const char *v4l_dev = "/dev/video0";
    uint32_t *jpg_buf = NULL;
    unsigned long buff_sz = 0;
    video_data_t vid_data;
    video_mode_t vid_mode = { width, height, fps };

    // register CTRL+C to exit
    signal(SIGINT, sig_handler);

    if (!video_init(v4l_dev, &vid_mode)) {
        fprintf(stderr, "failed to initialize video subsystem\n");
        return EXIT_FAILURE;
    }

    while (g_running) {
        if (!video_lock(&vid_data, 0)) {
            // video disabled, non-functioning, or frame not ready
            continue;
        }

        // copy the jpeg to our buffer now that we're safely locked
        if (buff_sz < vid_data.length) {
            free(jpg_buf);
            buff_sz = vid_data.length;
            jpg_buf = (uint32_t *)malloc(buff_sz);
        }

        memcpy(jpg_buf, vid_data.data, vid_data.length);
        video_unlock();

        fprintf(stderr, "read frame size %zu\n", vid_data.length);
    }

    fprintf(stderr, "freeing resources and shutting down...\n");
    free(jpg_buf);

    return 0;
}
