// -----------------------------------------------------------------------------
// File:    video_uvc.c
// Authors: Garrett Smith, Kevin Macksamie, Tyler Thierolf, Timothy Miller
// Created: 09-18-2010
//
// Definitions for communication between gumstix and uvc webcam.
// -----------------------------------------------------------------------------

#include <pthread.h>
#include <syslog.h>
#include <stdlib.h>
#include "v4l2uvc.h"
#include "video_uvc.h"

typedef struct video_globals
{
    int enabled;                // indicate initialization status of video
    int size;                   // size of current frame in bytes
    int width, height, fps;     // resolution and framerate
    uint8_t *buf;               // pointer to current frame buffer
    pthread_mutex_t db;         // lock frame buffer
    pthread_cond_t  db_update;  // signal frame updates
    struct vdIn *vd;            // v4l2uvc internal data
    pthread_t camthrd;          // video capture thread
    int is_fresh;               // keep track of fresh frames for async access
    int sync_access;            // keep track of fresh frames for sync access
} video_globals_t;

video_globals_t global = { 0 };

// -----------------------------------------------------------------------------
static void *video_capture_thread(void *arg)
{
    // set cleanup handler to cleanup allocated ressources
    video_globals_t *pglobal = (video_globals_t *)arg;
    // pthread_cleanup_push(cam_cleanup, NULL);

    while (pglobal->enabled) {

        // attempt to grab the next frame from the webcam (block until ready)
        if (v4l2GrabFrame(global.vd) < 0) {
            syslog(LOG_ERR, "error grabbing frame from uvc device\n");
            // sleep for a second and then try again
            sleep(1);
            continue;
        }

#if 0
        // Workaround for broken, corrupted frames:
        // Under low light conditions corrupted frames may get captured.
        // The good thing is such frames are quite small compared to the regular
        // pictures.  For example a VGA (640x480) webcam picture is normally
        // >= 8kByte large, corrupted frames are smaller.
        if (global.vd->buf.bytesused < minimum_size) {
            DBG("dropping too small frame, assuming it as broken\n");
            continue;
        }
#endif

        // copy JPG picture to global buffer
        pthread_mutex_lock(&pglobal->db);

        pglobal->size = v4l2CopyBuffer(pglobal->buf,
                global.vd->tmpbuffer, global.vd->buf.bytesused);

#if 0
        // Motion detection can be done just by comparing the picture size,
        // but it is not very accurate!!
        if ((prev_size-global->size)*(prev_size-global->size) > 4*1024*1024) {
            DBG("motion detected (delta: %d kB)\n", (prev_size - global->size) / 1024);
        }
        prev_size = global->size;
#endif

        global.is_fresh = 1;
        global.sync_access = 1;

        // signal fresh_frame
        pthread_cond_broadcast(&pglobal->db_update);
        pthread_mutex_unlock(&pglobal->db);

        // only use usleep if the fps is below 5, otherwise the overhead is too long
        if (global.vd->fps < 5) {
            usleep(1000 * 1000 / global.vd->fps);
        }
    }

    // pthread_cleanup_pop(1);
    return NULL;
}

// -----------------------------------------------------------------------------
void print_enum_ctrl(const struct v4l2_queryctrl *qc)
{
    int enum_menu = 0;
    const char *type = "unknown";

    switch (qc->type) {
    case V4L2_CTRL_TYPE_INTEGER:
        type = "int";
        break;
    case V4L2_CTRL_TYPE_BOOLEAN:
        type = "bool";
        break;
    case V4L2_CTRL_TYPE_MENU:
        type = "menu";
        enum_menu = 1;
        break;
    case V4L2_CTRL_TYPE_BUTTON:
        type = "button";
        break;
    case V4L2_CTRL_TYPE_INTEGER64:
        type = "int64";
        break;
    case V4L2_CTRL_TYPE_CTRL_CLASS:
        type = "class";
        break;
    default:
        // silently ignore any other control types
        return;
    }

    syslog(LOG_INFO, " + %s [type:%s min:%d max:%d step:%d default:%d flags:%d]",
           qc->name, type, qc->minimum, qc->maximum, qc->step,
           qc->default_value, qc->flags);
}

// -----------------------------------------------------------------------------
void print_enum_menu(const struct v4l2_querymenu *qm)
{
    syslog(LOG_INFO, "   + %s", qm->name);
}

// -----------------------------------------------------------------------------
int video_init(const char *dev, video_mode_t *mode)
{
    int format = V4L2_PIX_FMT_MJPEG;

    if (global.enabled) {
        syslog(LOG_INFO, "attempting to call video_init multiple times\n");
        return 0;
    }

    global.buf = NULL;
    global.size = 0;

    global.width  = mode->width;
    global.height = mode->height;
    global.fps    = mode->fps;

    if (0 != pthread_mutex_init(&global.db, NULL)) {
        syslog(LOG_ERR, "could not initialize mutex variable\n");
        closelog();
        return 0;
    }

    if (0 != pthread_cond_init(&global.db_update, NULL)) {
        syslog(LOG_ERR, "could not initialize condition variable\n");
        closelog();
        return 0;
    }

    // allocate webcam datastructure
    global.vd = calloc(1, sizeof(struct vdIn));
    if (NULL == global.vd) {
        syslog(LOG_ERR, "not enough memory for global.vd\n");
        return 0;
    }

    // open video device and prepare data structure
    if (v4l2DeviceOpen(global.vd, (char *)dev, mode->width, mode->height,
                mode->fps, format, 1) < 0) {
        syslog(LOG_ERR, "init_videoIn failed\n");
        return 0;
    }

    // display the parsed values
    syslog(LOG_INFO, "  V4L2 Device  : %s\n", dev);
    syslog(LOG_INFO, "  Device Name  : %s\n", global.vd->cap.card);
    syslog(LOG_INFO, "  Resolution   : %i x %i\n", mode->width, mode->height);
    syslog(LOG_INFO, "  Framerate    : %i\n", mode->fps);
    syslog(LOG_INFO, "  Video Format : %s\n", "MJPEG");

    // enumerate the supported device controls and print to log / stdout
    v4l2EnumControls(global.vd, print_enum_ctrl, print_enum_menu);

    global.buf = malloc(global.vd->framesizeIn);
    if (NULL == global.buf) {
        syslog(LOG_ERR, "could not allocate memory in input_run()\n");
        return 0;
    }

    pthread_create(&global.camthrd, 0, video_capture_thread, &global);
    pthread_detach(global.camthrd);

    global.enabled = 1;
    return 1;
}

// -----------------------------------------------------------------------------
void video_shutdown(void)
{
    if (!global.enabled) {
        syslog(LOG_INFO, "attempting to call video_shutdown prior to init\n");
        return;
    }

    global.enabled = 0;
    pthread_cancel(global.camthrd);

    pthread_cond_destroy(&global.db_update);
    pthread_mutex_destroy(&global.db);

    v4l2DeviceClose(global.vd);

    if (NULL != global.vd->tmpbuffer) {
        free(global.vd->tmpbuffer);
        global.vd->tmpbuffer = NULL;
    }

    if (NULL != global.vd) {
        free(global.vd);
        global.vd = NULL;
    }

    if (NULL != global.buf) {
        free(global.buf);
        global.buf = NULL;
    }
}

// -----------------------------------------------------------------------------
int video_lock(video_data_t *data, access_mode_t mode)
{
    pthread_mutex_lock(&global.db);

    if (!global.enabled) {
        pthread_mutex_unlock(&global.db);
        syslog(LOG_ERR, "attempting to call video_async_lock prior to init\n");
        return 0;
    }
    
    switch (mode)
    {
    case ACCESS_ASYNC:
        // asynchronous lock -- if not fresh, free mutex and exit immediately
        if (!global.is_fresh) {
            pthread_mutex_unlock(&global.db);
            return 0;
        }
        global.is_fresh = 0;
        break;
    case ACCESS_SYNC:
        // synchronous lock -- sleep until signalled by next fresh frame
        if (!global.sync_access) {
            pthread_cond_wait(&global.db_update, &global.db);
        }
        global.sync_access = 0;
        break;
    default:
        syslog(LOG_ERR, "specified invalid lock type\n");
        pthread_mutex_unlock(&global.db);
        return 0;
    }

    data->length = (size_t)global.size;
    data->data = (uint8_t *)global.buf;

    data->mode.width  = global.width;
    data->mode.height = global.height;
    data->mode.fps    = global.fps;
    return 1;
}

// -----------------------------------------------------------------------------
void video_unlock()
{
    // allow others to access the global buffer again
    pthread_mutex_unlock(&global.db);
}

// -----------------------------------------------------------------------------
int video_set_mode(video_mode_t *mode)
{
    syslog(LOG_INFO, "TODO: implement video_set_mode\n");
    return 1;
}

// -----------------------------------------------------------------------------
int video_enum_devctrl(enum_ctrl_fn c_fn, enum_menu_fn m_fn)
{
    if (!global.enabled) {
        syslog(LOG_ERR, "attempting to enumerate controls prior to init\n");
        return 0;
    }

    return (0 <= v4l2EnumControls(global.vd, c_fn, m_fn));
}

// -----------------------------------------------------------------------------
int video_set_devctrl(int id, int value)
{
    return (0 <= v4l2SetControl(global.vd, id, value));
}

// -----------------------------------------------------------------------------
int video_get_devctrl(int id)
{
    return v4l2GetControl(global.vd, id);
}

// -----------------------------------------------------------------------------
int video_get_fps()
{
    return global.fps;
}

