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
void *color_detect_thread(void *arg)
{
    printf("IMAGE PROC\n");
    track_color_t color = {
        .r = 151,
        .g = 242,
        .b = 192,
        .h = 30,
        .s = 100,
        .l = 360,
        .filter = 5    
    };
    track_coords_t box = {};

    video_data_t vid_data;
    uint8_t *jpg_buf = NULL;
    unsigned long buff_sz = 0;

    for (;;) {
        //pthread_cond_wait(pthread_cond_t *cond, pthread_mutex_t *mutex);
        if (!video_lock(&vid_data, 1)) {
            // video disabled, non-functioning, or frame not ready
            continue;
        }

        // copy the jpeg to our buffer now that we're safely locked
        if (buff_sz < vid_data.length) {
            free(jpg_buf);
            buff_sz = (vid_data.length);
            jpg_buf = (uint8_t *)malloc(buff_sz);
        }

        memcpy(jpg_buf, vid_data.data, vid_data.length);
        video_unlock();

        runColorDetectionMemory(jpg_buf, &buff_sz, &color, &box);
        if (box.detected) {
            printf("HSL Bounding box: (%d,%d) (%d,%d)\n",
                   box.x1, box.y1, box.x2, box.y2);
        }
        else {
            printf("Target object not found!\n");
        }
    }

    pthread_exit(NULL);
}
#endif

// -----------------------------------------------------------------------------
void runColorDetectionFile(const char *infile, const char *outfile,
        track_color_t *color, track_coords_t *box)
{
    uint8_t *rgb;

    // decode image from file, run color detection algorithm, write it back out
    jpeg_rd_file(infile, &rgb, &box->width, &box->height);    
    runColorDetection(rgb, color, box);
#if 0
    jpeg_wr_file(outfile, color->quality, rgb, box->width, box->height);
#endif
}

// -----------------------------------------------------------------------------
void runColorDetectionMemory(const uint8_t *stream_in, unsigned long *length,
        track_color_t *color, track_coords_t *box)
{
    uint8_t *rgb;   

    // decode image from mem, run color detection algorithm, write it back out
    jpeg_rd_mem(stream_in, *length, &rgb, &box->width, &box->height);    
    runColorDetection(rgb, color, box);
#if 0
    jpeg_wr_mem(&stream_in, length, color->quality, rgb, &box->width, &box->height);
#endif
}

// -----------------------------------------------------------------------------
void runColorDetection(const uint8_t *rgb_in,
        track_color_t *color, track_coords_t *box)
{

    // color to look for in HSL (will be calculated from RGB)
    short h = 0, s = 0, l = 0;                   
    short *hsl_buff = malloc(sizeof(short) * box->height * box->width * 3);

    // convert detect color from RGB to HSL
    RGB2HSL(color->r, color->g, color->b, &h, &s, &l);                     
                        
    // convert the image to HSL
    COLORimageRGBtoHSL(rgb_in, hsl_buff, box->width, box->height);

    // find HSL values in image 
    findColorHSL(hsl_buff, h, s, l, color, box);
}

// -----------------------------------------------------------------------------
void findColorHSL(const short *hsl_in, short h, short s, short l,
        track_color_t *color, track_coords_t *box)
{
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
            if (abs((hsl_in[pix_start + 0]) - h) < color->h &&
                abs((hsl_in[pix_start + 1]) - s) < color->s &&
                abs((hsl_in[pix_start + 2]) - l) < color->l) {

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
void COLORimageRGBtoHSL(const uint8_t *rgb_in, short *hsl_out,
        int width, int height)
{
    int i = 0, j = 0;
    for (i = 0; i < height; i++) {
        for (j = 0; j < width; j++) {
            // convert single rgb pixel to hsl color space
            RGB2HSL(rgb_in[(i * width * 3) + (j * 3) + 0],
                    rgb_in[(i * width * 3) + (j * 3) + 1],
                    rgb_in[(i * width * 3) + (j * 3) + 2],
                    &hsl_out[(i * width * 3) + (j * 3) + 0],
                    &hsl_out[(i * width * 3) + (j * 3) + 1],
                    &hsl_out[(i * width * 3) + (j * 3) + 2]);
        }
    }
}

// -----------------------------------------------------------------------------
void RGB2HSL(uint8_t r_in, uint8_t g_in, uint8_t b_in,
        short *h_out, short *s_out, short *l_out)
{
    real_t r = r_in / 255.0f;
    real_t g = g_in / 255.0f;
    real_t b = b_in / 255.0f;
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

    (*h_out) = h * 360.0f;
    (*s_out) = s * 360.0f;
    (*l_out) = l * 360.0f;
}

// -----------------------------------------------------------------------------
void findColorRGB(const uint8_t *rgb_in, int width, int height,
        uint8_t r, uint8_t g, uint8_t b, int threshold)
{
    int x = 0, y = 0, consec = 0, noise_filter = 5;
    int x1 = width, x2 = 0, y1 = height, y2 = 0;
    int img_pitch = width * 3, scan_start = 0, pix_start = 0;
    int r_diff, g_diff, b_diff;
    real_t dist;

    // iterate over each scanline in the source image
    for (y = 0; y < height; ++y) {

        // reset consecutive pixel count and calculate scanline start offset
        consec = 0;
        scan_start = y * img_pitch;

        // iterate over each pixel in the source scanline
        for (x = 0; x < width; ++x) {

            pix_start = scan_start + x * 3;
            r_diff = rgb_in[pix_start + 0];
            g_diff = rgb_in[pix_start + 1];
            b_diff = rgb_in[pix_start + 2];
            dist = sqrt(r_diff * r_diff + g_diff * g_diff + b_diff * b_diff);

            // if within threshold, update the bounding box
            if (dist < (real_t)threshold) {

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
    printf("(%d,%d) (%d,%d)\n",x1,y1,x2,y2);
}

