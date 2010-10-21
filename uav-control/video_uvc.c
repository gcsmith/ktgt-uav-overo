#include <pthread.h>
#include <syslog.h>
#include <stdlib.h>
#include "v4l2uvc.h"
#include "video_uvc.h"

uvc_globals_t global = { 0 };
static struct vdIn *g_videoin;
static pthread_t g_camthrd;
static int g_vid_enabled = 0;
static int g_is_fresh = 0;
static int g_unprocessed = 0;

// -----------------------------------------------------------------------------
void *cam_input_thread(void *arg)
{
    // set cleanup handler to cleanup allocated ressources
    uvc_globals_t *pglobal = (uvc_globals_t *)arg;
    // pthread_cleanup_push(cam_cleanup, NULL);

    while (!pglobal->stop) {

        // grab a frame
        if (uvcGrab(g_videoin) < 0) {
            syslog(LOG_ERR, "Error grabbing frames\n");
            sleep(1);
            continue;
        }

        // Workaround for broken, corrupted frames:
        // Under low light conditions corrupted frames may get captured.
        // The good thing is such frames are quite small compared to the regular
        // pictures.  For example a VGA (640x480) webcam picture is normally
        // >= 8kByte large, corrupted frames are smaller.
#if 0
        if (g_videoin->buf.bytesused < minimum_size) {
            DBG("dropping too small frame, assuming it as broken\n");
            continue;
        }
#endif

        // copy JPG picture to global buffer
        pthread_mutex_lock(&pglobal->db);

        // If capturing in YUV mode convert to JPEG now.
        // This compression requires many CPU cycles, so try to avoid YUV
        // format. Getting JPEGs straight from the webcam, is one of the major
        // advantages of Linux-UVC compatible devices.
#if 0
        if (g_videoin->formatIn == V4L2_PIX_FMT_YUYV) {
            DBG("compressing frame\n");
            pglobal->size = compress_yuyv_to_jpeg(g_videoin, pglobal->buf,
                                                  g_videoin->framesizeIn,
                                                  gquality);
        }
        else {
#endif
            pglobal->size = memcpy_picture(pglobal->buf, g_videoin->tmpbuffer,
                                           g_videoin->buf.bytesused);
#if 0
        }
#endif

#if 0
        // motion detection can be done just by comparing the picture size,
        // but it is not very accurate!!
        if ((prev_size-global->size)*(prev_size-global->size) > 4*1024*1024) {
            DBG("motion detected (delta: %d kB)\n", (prev_size - global->size) / 1024);
        }
        prev_size = global->size;
#endif
        g_is_fresh = 1;
        g_unprocessed = 1;

        // signal fresh_frame
        // pthread_cond_broadcast(&pglobal->db_update);
        pthread_mutex_unlock( &pglobal->db );

        // only use usleep if the fps is below 5, otherwise the overhead is too long
        if ( g_videoin->fps < 5 ) {
            usleep(1000*1000/g_videoin->fps);
        }
    }

    // pthread_cleanup_pop(1);
    return NULL;
}

// -----------------------------------------------------------------------------
int video_lock(video_data_t *vdata, int type)
{
    if (!g_vid_enabled) {
        syslog(LOG_ERR, "attempting to call video_lock prior to init\n");
        return 0;
    }
    
    pthread_mutex_lock(&global.db);
    
    // type 0 = Heliview Fetch (Use g_is_fresh)
    // type 1 = Video Processing (use g_unprocessed)
    if ((!g_is_fresh && type == 0) || (!g_unprocessed && type == 1)) {
        pthread_mutex_unlock(&global.db);
        return 0;
    }
    
    if (type == 0) {
        g_is_fresh = 0;
    }
    else if (type == 1) {
        g_unprocessed = 0;
    } 

    // pthread_cond_wait(&global.db_update, &global.db);

    vdata->length = (unsigned long)global.size;
    vdata->data = (const char *)global.buf;

    vdata->width = global.width;
    vdata->height = global.height;
    vdata->fps = global.fps;

    return 1;
}

// -----------------------------------------------------------------------------
void video_unlock()
{
    // allow others to access the global buffer again
    pthread_mutex_unlock(&global.db);
}

// -----------------------------------------------------------------------------
int video_init(const char *dev, int width, int height, int fps)
{
    int format = V4L2_PIX_FMT_MJPEG;
    static uvc_globals_t *pglobal;

    if (g_vid_enabled) {
        syslog(LOG_INFO, "attempting to call video_init multiple times\n");
        return 0;
    }

    global.stop = 0;
    global.buf  = NULL;
    global.size = 0;

    global.width  = width;
    global.height = height;
    global.fps    = fps;

    if (pthread_mutex_init(&global.db, NULL) != 0) {
        syslog(LOG_ERR, "could not initialize mutex variable\n");
        closelog();
        return 0;
    }

    if (pthread_cond_init(&global.db_update, NULL) != 0) {
        syslog(LOG_ERR, "could not initialize condition variable\n");
        closelog();
        return 0;
    }

    // keep a pointer to the global variables
    pglobal = &global;

    // allocate webcam datastructure
    g_videoin = malloc(sizeof(struct vdIn));
    if (g_videoin == NULL) {
        syslog(LOG_ERR, "not enough memory for g_videoin\n");
        return 0;
    }
    memset(g_videoin, 0, sizeof(struct vdIn));

    // display the parsed values
    syslog(LOG_INFO, "  V4L2 device        : %s\n", dev);
    syslog(LOG_INFO, "  Desired Resolution : %i x %i\n", width, height);
    syslog(LOG_INFO, "  Frames Per Second  : %i\n", fps);
    syslog(LOG_INFO, "  Vdeo Stream Format : %s\n", "MJPEG");
    //(format==V4L2_PIX_FMT_YUYV)?"YUV":"MJPEG");
#if 0
    if ( format == V4L2_PIX_FMT_YUYV )
        syslog(LOG_ERR, "JPEG Quality......: %d\n", gquality);
#endif
    // open video device and prepare data structure
    if (init_videoIn(g_videoin, (char *)dev, width, height, fps, format, 1, pglobal) < 0) {
        syslog(LOG_ERR, "init_VideoIn failed\n");
        return 0;
    }

    pglobal->buf = malloc(g_videoin->framesizeIn);
    if (pglobal->buf == NULL) {
        syslog(LOG_ERR, "could not allocate memory in input_run()\n");
        return 0;
    }

    pthread_create(&g_camthrd, 0, cam_input_thread, pglobal);
    pthread_detach(g_camthrd);

    g_vid_enabled = 1;
    return 1;
}

// -----------------------------------------------------------------------------
void video_shutdown(void)
{
    if (!g_vid_enabled) {
        syslog(LOG_INFO, "attempting to call video_shutdown prior to init\n");
        return;
    }

    g_vid_enabled = 0;
    global.stop = 1;
    pthread_cancel(g_camthrd);

    pthread_cond_destroy(&global.db_update);
    pthread_mutex_destroy(&global.db);

    close_v4l2(g_videoin);

    if (NULL != g_videoin->tmpbuffer)
        free(g_videoin->tmpbuffer);
    if (NULL != g_videoin)
        free(g_videoin);
    if (NULL != global.buf)
        free(global.buf);
}

// -----------------------------------------------------------------------------
int video_cfg_exposure(int automatic, int abs_value)
{
    if (automatic) {
        // set exposure mode to automatic and let the camera take over
        syslog(LOG_INFO, "setting video exposure to automatic mode\n");
        if (0 > v4l2SetControl(g_videoin, V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_AUTO)) {
            syslog(LOG_ERR, "failed to set video exposure mode to auto\n");
            return 0;
        }
    }
    else {
        // set the exposure mode to manual adjustment
        syslog(LOG_INFO, "setting video exposure to manual mode\n");
        if (0 > v4l2SetControl(g_videoin, V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL)) {
            syslog(LOG_ERR, "failed to set video exposure mode to manual\n");
            return 0;
        }

        // set the absolute exposure value (units are undefined)
        syslog(LOG_INFO, "setting video absolute exposure to %d\n", abs_value);
        if (0 > v4l2SetControl(g_videoin, V4L2_CID_EXPOSURE_ABSOLUTE, abs_value)) {
            syslog(LOG_ERR, "failed to set video absolute exposure to %d\n", abs_value);
            return 0;
        }
    }

    return 1;
}

// -----------------------------------------------------------------------------
int video_cfg_focus(int automatic, int abs_value)
{
    if (automatic) {
        // set focus mode to automatic and let the camera take over
        syslog(LOG_INFO, "setting video focus to automatic mode\n");
        if (0 > v4l2SetControl(g_videoin, V4L2_CID_FOCUS_AUTO, 1)) {
            syslog(LOG_ERR, "failed to set video focus mode to auto\n");
            return 0;
        }
    }
    else {
        // set the focus mode to manual adjustment
        syslog(LOG_INFO, "setting video focus to manual mode\n");
        if (0 > v4l2SetControl(g_videoin, V4L2_CID_FOCUS_AUTO, 0)) {
            syslog(LOG_ERR, "failed to set video focus mode to manaul\n");
            return 0;
        }

        // set the absolute focus value (units are undefined)
        syslog(LOG_INFO, "setting video absolute focus to %d\n", abs_value);
        if (0 > v4l2SetControl(g_videoin, V4L2_CID_FOCUS_ABSOLUTE, abs_value)) {
            syslog(LOG_ERR, "failed to set video absolute focus to %d\n", abs_value);
            return 0;
        }
    }

    return 1;
}

// -----------------------------------------------------------------------------
int video_cfg_whitebalance(int automatic)
{
    if (automatic) {
        // set white balance mode to automatic and let the camera take over
        syslog(LOG_INFO, "setting video white balance to automatic mode\n");
        if (0 > v4l2SetControl(g_videoin, V4L2_CID_AUTO_WHITE_BALANCE, 1)) {
            syslog(LOG_ERR, "failed to set video white balance mode to auto\n");
            return 0;
        }
    }
    else {
        syslog(LOG_INFO, "setting video white balance to manual mode\n");
        if (0 > v4l2SetControl(g_videoin, V4L2_CID_AUTO_WHITE_BALANCE, 0)) {
            syslog(LOG_ERR, "failed to set video white balance mode to auto\n");
            return 0;
        }

        syslog(LOG_INFO, "setting video white balance\n");
        if (0 > v4l2SetControl(g_videoin, V4L2_CID_DO_WHITE_BALANCE, 0)) {
            syslog(LOG_ERR, "failed to set video white balance\n");
            return 0;
        }
    }

    return 1;
}

