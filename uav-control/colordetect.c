#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <syslog.h>
#include <pthread.h>
#include "jpeglib.h"
#include "readwritejpeg.h"
#include "colordetect.h"
#include "utility.h"
#include "video_uvc.h"

// -----------------------------------------------------------------------------
#ifndef STANDALONE_DEMO
static pthread_t g_color_thread;
static int g_vision_init = 0;
static int running = 0;

void *color_detect_thread(void *arg)
{
    printf("IMAGE PROC\n");
    track_color_t color = {
        .r = 151,
        .g = 242,
        .b = 192,
        .ht = 22, //30,
        .st = 71, //100,
        .lt = 255, //360,
        .filter = 5    
    };
    track_coords_t box = {};

    video_data_t vid_data;
    uint8_t *jpg_buf = NULL;
    unsigned long buff_sz = 0;
    uint8_t *rgb_buff = NULL;

    while (running) {
        //pthread_cond_wait(pthread_cond_t *cond, pthread_mutex_t *mutex);
        if (!video_lock(&vid_data, 1)) {
            // video disabled, non-functioning, or frame not ready
             //printf("FAILURE TO LOCK\n"); fflush(stdout);
            continue;
        }
        // copy the jpeg to our buffer now that we're safely locked
        if (buff_sz < vid_data.length) {
            free(jpg_buf);
            buff_sz = (vid_data.length);
            jpg_buf = (uint8_t *)malloc(buff_sz);
        }

        memcpy(jpg_buf, vid_data.data, vid_data.length);     
        fprintf(stderr, "img %d x %d (sz %d)\n",
                vid_data.width, vid_data.height, (int)vid_data.length);
        video_unlock();

        if(jpeg_rd_mem(jpg_buf, buff_sz, &rgb_buff, &box.width, &box.height)
            != 0){
            color_detect_rgb(rgb_buff, &color, &box);
        }
        //if(rgb_buff != NULL){
        //    free(rgb_buff);
        //}
        

        //runColorDetectionMemory(jpg_buf, &buff_sz, &color, &box);
        if (box.detected) {
            printf("Bounding box: (%d,%d) (%d,%d)\n",
                   box.x1, box.y1, box.x2, box.y2);
        }
        else {
            printf("Target object not found!\n");
        }
        fflush(stdout);
    }

    pthread_exit(NULL);
}

// -----------------------------------------------------------------------------
void colordetect_init(void)
{   
    running =  1;
    if (g_vision_init) {
        syslog(LOG_INFO, "attempting multiple colordetect_init calls\n");
        return;
    }

    g_vision_init = 1;
    pthread_create(&g_color_thread, 0, color_detect_thread, NULL);
    pthread_detach(g_color_thread);
}

// -----------------------------------------------------------------------------
void colordetect_shutdown(void)
{   
    running = 0;
    if (!g_vision_init) {
        syslog(LOG_INFO, "calling colordetect_shutdown prior to init\n");
        return;
    }

    g_vision_init = 0;
    pthread_cancel(g_color_thread);
}

#endif

// -----------------------------------------------------------------------------
void color_detect_rgb(const uint8_t *rgb_in,
        track_color_t *color, track_coords_t *box)
{
    findColorRGB(rgb_in, color, box);
}

// -----------------------------------------------------------------------------
void color_detect_rgb_dist (const uint8_t *rgb_in, real_t threshold,
        track_color_t *color, track_coords_t *box)
{
    findColorRGB_dist(rgb_in, threshold, color, box);
}

// -----------------------------------------------------------------------------
void color_detect_hsl (uint8_t *rgb_in, 
        track_color_t *color, track_coords_t *box) 
{
    // convert detect color from RGB to HSL
    RGB2HSL(&color->r, &color->g, &color->b);                     
                        
    // convert the image to HSL
    COLORimageRGBtoHSL(rgb_in, box->width, box->height);

    // find HSL values in image 
    findColorHSL(rgb_in, color, box);
}

// -----------------------------------------------------------------------------
void runColorDetectionFile(const char *infile, const char *outfile,
        track_color_t *color, track_coords_t *box)
{
    uint8_t *rgb = NULL;

    // decode image from file, run color detection algorithm, write it back out
    jpeg_rd_file(infile, &rgb, &box->width, &box->height);    
    color_detect_hsl(rgb, color, box);
#if 0
    jpeg_wr_file(outfile, color->quality, rgb, box->width, box->height);
#endif
}

// -----------------------------------------------------------------------------
void runColorDetectionMemory(const uint8_t *stream_in, unsigned long *length,
        track_color_t *color, track_coords_t *box)
{
    uint8_t *rgb = NULL;

    // decode image from mem, run color detection algorithm, write it back out
    jpeg_rd_mem(stream_in, *length, &rgb, &box->width, &box->height);    
    color_detect_hsl(rgb, color, box);
#if 0
    jpeg_wr_mem(&stream_in, length, color->quality, rgb, &box->width, &box->height);
#endif
}

// -----------------------------------------------------------------------------
void findColorHSL (const uint8_t *hsl_in, 
        track_color_t *color, track_coords_t *box) {
    int x = 0, y = 0, consec = 0, noise_filter = color->filter;
    int img_pitch = box->width * 3, scan_start = 0, pix_start = 0;

    // initialize box to obviously invalid state so we know if we didn't detect
    box->x1 = box->width;
    box->y1 = box->height;
    box->x2 = 0;
    box->y2 = 0;

    // iterate over each scanline in the source image
    for (y = 0; y < box->height; ++y) {

        // reset consecutive pixel count and calculate scanline start offset
        consec = 0;
        scan_start = y * img_pitch;

        // iterate over each pixel in the source scanline
        for (x = 0; x < box->width; ++x) {

            // if within threshold, update the bounding box
            pix_start = scan_start + x * 3;
            if (abs((hsl_in[pix_start + 0]) - color->r) < color->ht &&
                abs((hsl_in[pix_start + 1]) - color->g) < color->st &&
                abs((hsl_in[pix_start + 2]) - color->b) < color->lt) {
                //color->r is the h value

                // only update bounding box of consecutive pixels >= "filter"
                if (++consec >= noise_filter) {
                    if (x < box->x1) box->x1 = x;
                    if (x > box->x2) box->x2 = x;
                    if (y < box->y1) box->y1 = y;
                    if (y > box->y2) box->y2 = y; 
                }
            }
            else {
                // reset number of consecutive pixels if we didn't match
                consec = 0;
            }
        }
    }

    if ((0 == box->x2 && box->x1 == box->width) && 
        (0 == box->y2 && box->y1 == box->height)) {
        // color was not detected
        box->detected = 0;
    }
    else {
        // color was detected
        box->detected = 1;
    }
}

// -----------------------------------------------------------------------------
void COLORimageRGBtoHSL (uint8_t *rgb_in, int width, int height)
{
    int i = 0, j = 0;
    for (i = 0; i < height; i++) {
        for (j = 0; j < width; j++) {
            // convert single rgb pixel to hsl color space
            RGB2HSL(&(rgb_in[(i * width * 3) + (j * 3) + 0]),
                    &(rgb_in[(i * width * 3) + (j * 3) + 1]),
                    &(rgb_in[(i * width * 3) + (j * 3) + 2]));
        }
    }
}

// -----------------------------------------------------------------------------
void RGB2HSL(uint8_t * r_h, uint8_t * g_s, uint8_t * b_l)
{
    real_t r = *r_h / 255.0f;
    real_t g = *g_s / 255.0f;
    real_t b = *b_l / 255.0f;
    real_t max;
    real_t min;

    real_t vm;
    real_t h = 0.0f, s = 0.0f, l = 0.0f; // set to black by default

    max = MAX(MAX(r, g), b);
    min = MIN(MIN(r, g), b);
    l = (max + min) / 2.0f;

    if (max == min) {
        return;
    }

    vm = max - min;
    s = l > 0.5f ? vm / (2.0f - max - min) : vm / (max + min);

    if (max == r) {
        h = (g - b) / vm + (g < b ? 6.0f : 0.0f);
    }
    else if (max == g) {
        h = (b - r) / vm + 2.0f;
    }
    else {
        h = (r - g) / vm + 4.0f;
    }

    h /= 6.0f;

    (*r_h) = h * 255.0f;
    (*g_s) = s * 255.0f;
    (*b_l) = l * 255.0f;
}

// -----------------------------------------------------------------------------
void findColorRGB(const uint8_t *rgb_in,
        track_color_t *color, track_coords_t *box)
{
    int x = 0, y = 0, consec = 0, noise_filter = 5;
    int img_width = box->width, img_height = box->height;
    int img_pitch = img_width * 3, scan_start = 0, pix_start = 0;
    int r_track = color->r, g_track = color->g, b_track = color->b;
    int r_thresh = color->ht, g_thresh = color->st, b_thresh = color->lt;

    // initialize box to obviously invalid state so we know if we didn't detect
    int x1 = img_width, y1 = img_height, x2 = 0, y2 = 0;

    // iterate over each scanline in the source image
    for (y = 0; y < img_height; ++y) {

        // reset consecutive pixel count and calculate scanline start offset
        consec = 0;
        scan_start = y * img_pitch;

        // iterate over each pixel in the source scanline
        for (x = 0; x < img_width; ++x) {

            pix_start = scan_start + x * 3;
            // if within threshold, update the bounding box
            if (abs(rgb_in[pix_start + 0] - r_track) < r_thresh &&
                abs(rgb_in[pix_start + 1] - g_track) < g_thresh &&
                abs(rgb_in[pix_start + 2] - b_track) < b_thresh) {

                // only update bounding box of consecutive pixels >= "filter"
                if (++consec >= noise_filter) {
                    if (x < x1) x1 = x;
                    if (x > x2) x2 = x;
                    if (y < y1) y1 = y;
                    if (y > y2) y2 = y; 
                }
            }
            else {
                // reset number of consecutive pixels if we didn't match
                consec = 0;
            }
        }
    }

    box->x1 = x1;
    box->y1 = y1;
    box->x2 = x2;
    box->y2 = y2;

    if ((0 == box->x2 && box->x1 == box->width) && 
        (0 == box->y2 && box->y1 == box->height)) {
        // color was not detected
        box->detected = 0;
    }
    else {
        // color was detected
        box->detected = 1;
    }
}

// -----------------------------------------------------------------------------
void findColorRGB_dist(const uint8_t *rgb_in, int threshold,
        track_color_t *color, track_coords_t *box)
{
    int x = 0, y = 0, consec = 0, noise_filter = 5;
    int img_width = box->width, img_height = box->height;
    int img_pitch = img_width * 3, scan_start = 0, pix_start = 0;
    int r_track = color->r, g_track = color->g, b_track = color->b;
    int r_diff, g_diff, b_diff;
    int dist;

    // square the input threshold value
    threshold *= threshold;

    // initialize box to obviously invalid state so we know if we didn't detect
    int x1 = img_width, y1 = img_height, x2 = 0, y2 = 0;

    // iterate over each scanline in the source image
    for (y = 0; y < img_height; ++y) {

        // reset consecutive pixel count and calculate scanline start offset
        consec = 0;
        scan_start = y * img_pitch;

        // iterate over each pixel in the source scanline
        for (x = 0; x < img_width; ++x) {

            pix_start = scan_start + x * 3;
            r_diff = rgb_in[pix_start + 0] - r_track;
            g_diff = rgb_in[pix_start + 1] - g_track;
            b_diff = rgb_in[pix_start + 2] - b_track;
            dist = r_diff * r_diff + g_diff * g_diff + b_diff * b_diff;

            // if within threshold, update the bounding box
            if (dist < threshold) {

                // only update bounding box of consecutive pixels >= "filter"
                if (++consec >= noise_filter) {
                    if (x < x1) x1 = x;
                    if (x > x2) x2 = x;
                    if (y < y1) y1 = y;
                    if (y > y2) y2 = y; 
                }
            }
            else {
                // reset number of consecutive pixels if we didn't match
                consec = 0;
            }
        }
    }

    box->x1 = x1;
    box->y1 = y1;
    box->x2 = x2;
    box->y2 = y2;

    if ((0 == box->x2 && box->x1 == box->width) && 
        (0 == box->y2 && box->y1 == box->height)) {
        // color was not detected
        box->detected = 0;
    }
    else {
        // color was detected
        box->detected = 1;
    }
}

