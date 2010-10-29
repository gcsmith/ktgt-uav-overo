// -----------------------------------------------------------------------------
// File:    tracking.c
// Authors: Tyler Thierolf, Timothy Miller, Garrett Smith
// Created: 10-26-2010
//
// Routines for tracking colors.
// -----------------------------------------------------------------------------

#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <syslog.h>
#include <time.h>
#include "readwritejpeg.h"
#include "uav_protocol.h"
#include "video_uvc.h"
#include "tracking.h"

typedef struct tracking_args
{
    client_info_t *client;      // connect client handle
    track_color_t  color;       // current color to track
    track_coords_t box;         // bounding box of tracked color 
    unsigned int   running;     // subsystem is currently running
    unsigned int   tracking;
    pthread_t      thread;      // handle to color detect thread
    unsigned int   trackingRate;
    pthread_mutex_t lock;
} tracking_args_t;

static tracking_args_t g_globals;

// -----------------------------------------------------------------------------
static void *tracking_thread(void *arg)
{
    tracking_args_t *data = (tracking_args_t *)arg;
    track_coords_t *box = &data->box;
    video_data_t vid_data;
    unsigned long buff_sz = 0;
    uint8_t *jpg_buf = NULL, *rgb_buff = NULL;
    uint32_t cmd_buffer[16];
    int delta, frames_computed = 0, tracking_fps = 0, streaming_fps = 0;
    struct timespec t0, t1, tc;
    
    clock_gettime(CLOCK_REALTIME, &t0);
    while (data->running) {
        // lock the colordetect parameters and determine our tracking fps
        pthread_mutex_lock(&data->lock);
        streaming_fps = video_get_fps();
        tracking_fps = MIN(data->trackingRate, streaming_fps);
        pthread_mutex_unlock(&data->lock);
        
        // if tracking is disable - sleep and check for a change every second
        if (tracking_fps <= 0) {
            sleep(1);
            continue;
        }
        
        // make synchronous frame read -- sleep for a second and retry on fail
        if (!video_lock(&vid_data, ACCESS_SYNC)) {
            syslog(LOG_ERR, "colordetect failed to lock frame\n");
            sleep(1);
            continue;
        }

        clock_gettime(CLOCK_REALTIME, &tc);
        delta = timespec_delta(&t0, &tc);
        if (((frames_computed * 1000000) / delta) >= tracking_fps) {
            video_unlock();
            continue;
        }

        // copy the jpeg to our buffer now that we're safely locked
        if (buff_sz < vid_data.length) {
            free(jpg_buf);
            buff_sz = vid_data.length;
            jpg_buf = (uint8_t *)malloc(buff_sz);
        }

        memcpy(jpg_buf, vid_data.data, vid_data.length);     
        video_unlock();

        if (0 != jpeg_rd_mem(jpg_buf, buff_sz, &rgb_buff,
                             &box->width, &box->height)) {
            colordetect_hsl_fp32(rgb_buff, &data->color, box);
            frames_computed++;
        }

        if (box->detected) {
            // we've detected an object, send the updated bounding box
            cmd_buffer[PKT_COMMAND]   = SERVER_UPDATE_TRACKING;
            cmd_buffer[PKT_LENGTH]    = PKT_CTS_LENGTH;
            cmd_buffer[PKT_CTS_STATE] = CTS_STATE_DETECTED;
            cmd_buffer[PKT_CTS_X1]    = (uint32_t)box->x1;
            cmd_buffer[PKT_CTS_Y1]    = (uint32_t)box->y1;
            cmd_buffer[PKT_CTS_X2]    = (uint32_t)box->x2;
            cmd_buffer[PKT_CTS_Y2]    = (uint32_t)box->y2;
            cmd_buffer[PKT_CTS_XC]    = (uint32_t)box->xc;
            cmd_buffer[PKT_CTS_YC]    = (uint32_t)box->yc;

            send_packet(data->client, cmd_buffer, PKT_CTS_LENGTH);
            data->tracking = 1;

            fprintf(stderr, "Bounding box: (%d,%d) (%d,%d)\n",
                    box->x1, box->y1, box->x2, box->y2);
        }
        else if (data->tracking) {
            // this means we were tracking, but lost our target. tell the client
            memset(cmd_buffer, 0, PKT_CTS_LENGTH);
            cmd_buffer[PKT_COMMAND]   = SERVER_UPDATE_TRACKING;
            cmd_buffer[PKT_LENGTH]    = PKT_CTS_LENGTH;
            cmd_buffer[PKT_CTS_STATE] = CTS_STATE_SEARCHING;
            send_packet(data->client, cmd_buffer, PKT_CTS_LENGTH);
            data->tracking = 0;

            fprintf(stderr, "lost target...\n");
        }

        if (frames_computed >= streaming_fps) {
            // every time we hit the streaming fps, dump out the tracking rate
            clock_gettime(CLOCK_REALTIME, &t1);
            real_t delta = timespec_delta(&t0, &t1) / (real_t)1000000;
            fprintf(stderr, "Tracking FPS = %f\n", streaming_fps / delta);
            frames_computed = 0;
            t0 = t1;
        }
    }

    pthread_exit(NULL);
}

// -----------------------------------------------------------------------------
int tracking_init(client_info_t *client)
{   
    int rc;
    if (g_globals.running) {
        syslog(LOG_INFO, "attempting multiple colordetect_init calls\n");
        return 0;
    }

    // zero out all globals
    memset(&g_globals, 0, sizeof(g_globals));

    g_globals.running = 1;
    g_globals.tracking = 0;
    g_globals.client = client;

    // set initial color value to track
    g_globals.color.r = 159;
    g_globals.color.g = 39;
    g_globals.color.b = 100;

    // set initial tracking threshold values
    g_globals.color.ht = 10;
    g_globals.color.st = 20;
    g_globals.color.lt = 30;

    g_globals.color.filter = 10;


    if (0 != (rc = pthread_mutex_init(&g_globals.lock, NULL))) {
        syslog(LOG_ERR, "error creating colordetect mutex (%d)", rc);
        return 0;
    }

    // create and kick off the color tracking thread
    pthread_create(&g_globals.thread, 0, tracking_thread, &g_globals);
    pthread_detach(g_globals.thread);

    return 1;
}

// -----------------------------------------------------------------------------
void tracking_shutdown(void)
{   
    if (!g_globals.running) {
        syslog(LOG_INFO, "calling colordetect_shutdown prior to init\n");
        return;
    }

    g_globals.running = 0;
    pthread_cancel(g_globals.thread);
    pthread_mutex_destroy(&g_globals.lock);
}

// -----------------------------------------------------------------------------
void tracking_enable(int enabled)
{
    if (enabled)
        syslog(LOG_INFO, "TODO: requested color tracking enable\n");
    else
        syslog(LOG_INFO, "TODO: requested color tracking disable\n");
}

// -----------------------------------------------------------------------------
void tracking_set_color(track_color_t *color)
{
    g_globals.color = *color;
}

// -----------------------------------------------------------------------------
track_color_t tracking_get_color(void)
{
    return g_globals.color;
}

//------------------------------------------------------------------------------
void tracking_set_fps(unsigned int fps)
{
    pthread_mutex_lock(&g_globals.lock);
    g_globals.trackingRate = fps;
    pthread_mutex_unlock(&g_globals.lock);
}

//------------------------------------------------------------------------------
unsigned int tracking_get_fps()
{
    unsigned int rval;
    pthread_mutex_lock(&g_globals.lock);
    rval = g_globals.trackingRate;
    pthread_mutex_unlock(&g_globals.lock);
    return rval;
}

