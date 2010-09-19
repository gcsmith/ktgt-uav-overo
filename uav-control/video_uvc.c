#include <pthread.h>
#include <syslog.h>
#include <stdlib.h>
#include "video_uvc.h"

globals global;
static struct vdIn *videoIn;
static pthread_t cam;

void *cam_input_thread( void *arg ) {
  /* set cleanup handler to cleanup allocated ressources */
  globals *pglobal = (globals *)arg;
  // pthread_cleanup_push(cam_cleanup, NULL);

  while( !pglobal->stop ) {

    /* grab a frame */
    if( uvcGrab(videoIn) < 0 ) {
      IPRINT("Error grabbing frames\n");
      exit(EXIT_FAILURE);
    }

    DBG("received frame of size: %d\n", videoIn->buf.bytesused);

    /*
     * Workaround for broken, corrupted frames:
     * Under low light conditions corrupted frames may get captured.
     * The good thing is such frames are quite small compared to the regular pictures.
     * For example a VGA (640x480) webcam picture is normally >= 8kByte large,
     * corrupted frames are smaller.
     */
#if 0
    if ( videoIn->buf.bytesused < minimum_size ) {
      DBG("dropping too small frame, assuming it as broken\n");
      continue;
    }
#endif

    /* copy JPG picture to global buffer */
    pthread_mutex_lock( &pglobal->db );

    /*
     * If capturing in YUV mode convert to JPEG now.
     * This compression requires many CPU cycles, so try to avoid YUV format.
     * Getting JPEGs straight from the webcam, is one of the major advantages of
     * Linux-UVC compatible devices.
     */
#if 0
    if (videoIn->formatIn == V4L2_PIX_FMT_YUYV) {
      DBG("compressing frame\n");
      pglobal->size = compress_yuyv_to_jpeg(videoIn, pglobal->buf, videoIn->framesizeIn, gquality);
    }
    else {
#endif
      DBG("copying frame\n");
      pglobal->size = memcpy_picture(pglobal->buf, videoIn->tmpbuffer, videoIn->buf.bytesused);
#if 0
    }
#endif

#if 0
    /* motion detection can be done just by comparing the picture size, but it is not very accurate!! */
    if ( (prev_size - global->size)*(prev_size - global->size) > 4*1024*1024 ) {
        DBG("motion detected (delta: %d kB)\n", (prev_size - global->size) / 1024);
    }
    prev_size = global->size;
#endif

    /* signal fresh_frame */
    pthread_cond_broadcast(&pglobal->db_update);
    pthread_mutex_unlock( &pglobal->db );

    DBG("waiting for next frame\n");

    /* only use usleep if the fps is below 5, otherwise the overhead is too long */
    if ( videoIn->fps < 5 ) {
      usleep(1000*1000/videoIn->fps);
    }
  }

  DBG("leaving input thread, calling cleanup function now\n");
  // pthread_cleanup_pop(1);

  return NULL;
}

int video_init(const char *dev, int width, int height, int fps)
{
  int format = V4L2_PIX_FMT_MJPEG;
  static globals *pglobal;

  ////////
        global.stop = 0;
        global.buf  = NULL;
        global.size = 0;

        global.in_param.res_width = width;
        global.in_param.res_height = height;
        global.in_param.fps = fps;
        global.in_param.global = &global;
        strcpy(global.in_param.dev, dev);

        if( pthread_mutex_init(&global.db, NULL) != 0 ) {
            LOG("could not initialize mutex variable\n");
            closelog();
            return -1;//exit(EXIT_FAILURE);
        }
        if( pthread_cond_init(&global.db_update, NULL) != 0 ) {
            LOG("could not initialize condition variable\n");
            closelog();
            return -1;//exit(EXIT_FAILURE);
        }

  ////////


  /* keep a pointer to the global variables */
  pglobal = &global;

  /* allocate webcam datastructure */
  videoIn = malloc(sizeof(struct vdIn));
  if ( videoIn == NULL ) {
    IPRINT("not enough memory for videoIn\n");
    return -1; //exit(EXIT_FAILURE);
  }
  memset(videoIn, 0, sizeof(struct vdIn));

  /* display the parsed values */
  IPRINT("Using V4L2 device.: %s\n", dev);
  IPRINT("Desired Resolution: %i x %i\n", width, height);
  IPRINT("Frames Per Second.: %i\n", fps);
  IPRINT("Format............: %s\n", "MJPEG");
    //(format==V4L2_PIX_FMT_YUYV)?"YUV":"MJPEG");
#if 0
  if ( format == V4L2_PIX_FMT_YUYV )
    IPRINT("JPEG Quality......: %d\n", gquality);
#endif
  /* open video device and prepare data structure */
  if (init_videoIn(videoIn, (char *)dev, width, height, fps, format, 1, pglobal) < 0) {
    IPRINT("init_VideoIn failed\n");
    //closelog();
    return -1; //exit(EXIT_FAILURE);
  }

  pglobal->buf = malloc(videoIn->framesizeIn);
  if (pglobal->buf == NULL) {
    fprintf(stderr, "could not allocate memory in input_run()\n");
    return -1; //exit(EXIT_FAILURE);
  }

  pthread_create(&cam, 0, cam_input_thread, pglobal);
  pthread_detach(cam);

  return 1;
}

void video_lock(const char **data, unsigned long *length)
{
    // pthread_cond_wait(&global.db_update, &global.db);
    pthread_mutex_lock(&global.db);

    *length = (unsigned long)global.size;
    *data = (const char *)global.buf;
}

void video_unlock()
{
    // allow others to access the global buffer again
    pthread_mutex_unlock(&global.db);
}

