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
    client_info_t  *client;     // connect client handle
    track_color_t   color;      // current color to track
    track_coords_t  coords;     // bounding box of tracked color 
    unsigned int    running;    // subsystem is currently running
    unsigned int    tracking;
    pthread_t       thread;     // handle to color detect thread
    unsigned int    trackingRate;
    pthread_mutex_t lock;
    pthread_mutex_t coord_lock;
    pthread_cond_t  coord_cond;
} tracking_args_t;

static tracking_args_t globals;

// -----------------------------------------------------------------------------
static void *tracking_thread(void *arg)
{
    tracking_args_t *data = (tracking_args_t *)arg;
    track_coords_t coords;
    video_data_t vid_data;
    unsigned long buff_sz = 0;
    uint8_t *jpg_buf = NULL, *rgb_buff = NULL;
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
                             &coords.width, &coords.height)) {
            colordetect_hsl_fp32(rgb_buff, &data->color, &coords);
            frames_computed++;
        }

        pthread_mutex_lock(&data->coord_lock);

        // copy over the updated coordinates to the global struct
        globals.coords = coords;

        // inform any listeners that new data is available
        pthread_cond_broadcast(&globals.coord_cond);
        pthread_mutex_unlock(&data->coord_lock);

        if (frames_computed >= streaming_fps) {
            // every time we hit the streaming fps, dump out the tracking rate
            clock_gettime(CLOCK_REALTIME, &t1);
            real_t delta = timespec_delta(&t0, &t1) / (real_t)1000000;
            syslog(LOG_INFO, "tracking fps: %f\n", streaming_fps / delta);
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
    if (globals.running) {
        syslog(LOG_INFO, "attempting multiple colordetect_init calls\n");
        return 0;
    }

    // zero out all globals
    memset(&globals, 0, sizeof(globals));

    globals.running = 1;
    globals.tracking = 0;
    globals.client = client;

    // set initial color value to track
    globals.color.r = 159;
    globals.color.g = 39;
    globals.color.b = 100;

    // set initial tracking threshold values
    globals.color.ht = 10;
    globals.color.st = 20;
    globals.color.lt = 30;
    globals.color.filter = 10;

    if (0 != (rc = pthread_mutex_init(&globals.lock, NULL))) {
        syslog(LOG_ERR, "error creating colordetect param mutex (%d)", rc);
        return 0;
    }

    if (0 != (rc = pthread_mutex_init(&globals.coord_lock, NULL))) {
        syslog(LOG_ERR, "error creating colordetect coord mutex (%d)", rc);
        return 0;
    }

    if (0 != (rc = pthread_cond_init(&globals.coord_cond, NULL))) {
        syslog(LOG_ERR, "error creating tracking event condition (%d)", rc);
        return 0;
    }

    // create and kick off the color tracking thread
    pthread_create(&globals.thread, 0, tracking_thread, &globals);
    pthread_detach(globals.thread);

    return 1;
}

// -----------------------------------------------------------------------------
void tracking_shutdown(void)
{   
    if (!globals.running) {
        syslog(LOG_INFO, "calling colordetect_shutdown prior to init\n");
        return;
    }

    globals.running = 0;
    pthread_cancel(globals.thread);
    pthread_mutex_destroy(&globals.lock);
    pthread_mutex_destroy(&globals.coord_lock);
    pthread_cond_destroy(&globals.coord_cond);
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
    globals.color = *color;
}

// -----------------------------------------------------------------------------
track_color_t tracking_get_color(void)
{
    return globals.color;
}

//------------------------------------------------------------------------------
void tracking_set_fps(unsigned int fps)
{
    pthread_mutex_lock(&globals.lock);
    globals.trackingRate = fps;
    pthread_mutex_unlock(&globals.lock);
}

//------------------------------------------------------------------------------
unsigned int tracking_get_fps()
{
    unsigned int rval;
    pthread_mutex_lock(&globals.lock);
    rval = globals.trackingRate;
    pthread_mutex_unlock(&globals.lock);
    return rval;
}

//------------------------------------------------------------------------------
int tracking_read_state(track_coords_t *coords, access_mode_t mode)
{
    pthread_mutex_lock(&globals.coord_lock);

    switch (mode) {
    case ACCESS_ASYNC:
        // access in an asynchronous (non-blocking) fashion
        break;
    case ACCESS_SYNC:
        // access in a synchronous (blocking) fashion
        pthread_cond_wait(&globals.coord_cond, &globals.coord_lock);
        break;
    default:
        pthread_mutex_unlock(&globals.coord_lock);
        memset(coords, 0, sizeof(track_coords_t));
        syslog(LOG_ERR, "tracking_read_state: invalid access mode\n");
        return 0;
    }

    *coords = globals.coords;
    pthread_mutex_unlock(&globals.coord_lock);
    return 1;
}

