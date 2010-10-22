#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <pthread.h>

#include "uav_protocol.h"
#include "readwritejpeg.h"
#include "colordetect.h"
#include "utility.h"
#include "video_uvc.h"
#include "fp32.h"

// -----------------------------------------------------------------------------
#ifndef STANDALONE_DEMO

typedef struct color_detect_args
{
    client_info_t *client;      // connect client handle
    track_color_t  color;       // current color to track
    track_coords_t box;         // bounding box of tracked color 
    unsigned int   running;     // subsystem is currently running
    unsigned int   tracking;
    pthread_t      thread;      // handle to color detect thread
} color_detect_args_t;

static color_detect_args_t g_globals;

void *color_detect_thread(void *arg)
{
    color_detect_args_t *data = (color_detect_args_t *)arg;
    track_color_t *color = &data->color;
    track_coords_t *box = &data->box;
    video_data_t vid_data;
    unsigned long buff_sz = 0;
    uint8_t *jpg_buf = NULL, *rgb_buff = NULL;
    uint32_t cmd_buffer[16];

    while (data->running) {
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
        video_unlock();

        if (0 != jpeg_rd_mem(jpg_buf, buff_sz, &rgb_buff, &box->width, &box->height)) {
            color_detect_hsl_fp32(rgb_buff, color, box);
        }

        if (box->detected) {

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

            printf("Bounding box: (%d,%d) (%d,%d)\n",
                   box->x1, box->y1, box->x2, box->y2);
        }
        else if (data->tracking) {

            memset(cmd_buffer, 0, PKT_CTS_LENGTH);
            cmd_buffer[PKT_COMMAND]   = SERVER_UPDATE_TRACKING;
            cmd_buffer[PKT_LENGTH]    = PKT_CTS_LENGTH;
            cmd_buffer[PKT_CTS_STATE] = CTS_STATE_SEARCHING;
            send_packet(data->client, cmd_buffer, PKT_CTS_LENGTH);
            data->tracking = 0;

            printf("lost target...\n");
        }
        fflush(stdout);
    }

    pthread_exit(NULL);
}

// -----------------------------------------------------------------------------
int colordetect_init(client_info_t *client)
{   
    if (g_globals.running) {
        syslog(LOG_INFO, "attempting multiple colordetect_init calls\n");
        return 0;
    }

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

    // create and kick off the color tracking thread
    pthread_create(&g_globals.thread, 0, color_detect_thread, &g_globals);
    pthread_detach(g_globals.thread);

    return 1;
}

// -----------------------------------------------------------------------------
void colordetect_shutdown(void)
{   
    if (!g_globals.running) {
        syslog(LOG_INFO, "calling colordetect_shutdown prior to init\n");
        return;
    }

    g_globals.running = 0;
    pthread_cancel(g_globals.thread);
}

// -----------------------------------------------------------------------------
void colordetect_set_track_color(track_color_t *color)
{
    g_globals.color = *color;
}

// -----------------------------------------------------------------------------
void colordetect_enable(int enabled)
{
    if (enabled)
        syslog(LOG_INFO, "TODO: requested color tracking enable\n");
    else
        syslog(LOG_INFO, "TODO: requested color tracking disable\n");
}

#endif

// -----------------------------------------------------------------------------
void color_detect_rgb(const uint8_t *rgb_in,
        const track_color_t *color, track_coords_t *box)
{
    findColorRGB(rgb_in, color, box);
}

// -----------------------------------------------------------------------------
void color_detect_rgb_dist (const uint8_t *rgb_in, real_t threshold,
        const track_color_t *color, track_coords_t *box)
{
    findColorRGB_dist(rgb_in, threshold, color, box);
}

// -----------------------------------------------------------------------------
void color_detect_hsl(uint8_t *rgb_in, 
        const track_color_t *color, track_coords_t *box) 
{
    track_color_t track_color = *color;
    // convert detect color from RGB to HSL
    rgb_to_hsl(&track_color.r, &track_color.g, &track_color.b);                     
                
    // convert the image to HSL
    COLORimageRGBtoHSL(rgb_in, box->width, box->height);

    // find HSL values in image 
    findColorHSL(rgb_in, &track_color, box);
}

// -----------------------------------------------------------------------------
void color_detect_hsl_fp32(uint8_t *rgb_in, 
        const track_color_t *color, track_coords_t *box) 
{
    track_color_t track_color = *color;
    // convert detect color from RGB to HSL
    rgb_to_hsl_fp32(&track_color.r, &track_color.g, &track_color.b);                     
                        
    // convert the image to HSL
    COLORimageRGBtoHSLfixed(rgb_in, box->width, box->height);

    // find HSL values in image 
    findColorHSL(rgb_in, &track_color, box);
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
void COLORimageRGBtoHSL(uint8_t *rgb_in, int width, int height)
{
    int i = 0, j = 0;
    for (i = 0; i < height; i++) {
        for (j = 0; j < width; j++) {
            // convert single rgb pixel to hsl color space
            rgb_to_hsl(&(rgb_in[(i * width * 3) + (j * 3) + 0]),
                       &(rgb_in[(i * width * 3) + (j * 3) + 1]),
                       &(rgb_in[(i * width * 3) + (j * 3) + 2]));
        }
    }
}

// -----------------------------------------------------------------------------
void COLORimageRGBtoHSLfixed(uint8_t *rgb_in, int width, int height)
{
    int i = 0, j = 0;
    for (i = 0; i < height; i++) {
        for (j = 0; j < width; j++) {
            // convert single rgb pixel to hsl color space
            rgb_to_hsl_fp32(&(rgb_in[(i * width * 3) + (j * 3) + 0]),
                            &(rgb_in[(i * width * 3) + (j * 3) + 1]),
                            &(rgb_in[(i * width * 3) + (j * 3) + 2]));
        }
    }
}

// -----------------------------------------------------------------------------
void rgb_to_hsl(uint8_t *r_h, uint8_t *g_s, uint8_t *b_l)
{
    real_t r = *r_h * 0.00392156863f;
    real_t g = *g_s * 0.00392156863f;
    real_t b = *b_l * 0.00392156863f;
    real_t min = MIN3(r, g, b);
    real_t max = MAX3(r, g, b);
    real_t vm, h, s, l = (min + max) * 0.5f;

    if (min == max) {
        *r_h = 0;
        *g_s = 0;
        *b_l = l * 255.0f;
    }
    else {
        vm = max - min;
        s = l > 0.5f ? vm / (2.0f - max - min) : vm / (max + min);

        if (max == r)
            h = (g - b) / vm + (g < b ? 6.0f : 0.0f);
        else if (max == g)
            h = (b - r) / vm + 2.0f;
        else
            h = (r - g) / vm + 4.0f;

        (*r_h) = h *  42.5f;
        (*g_s) = s * 255.0f;
        (*b_l) = l * 255.0f;
    }
}

// -----------------------------------------------------------------------------
void rgb_to_hsl_fp32(uint8_t *r_h, uint8_t *g_s, uint8_t *b_l)
{
    const int32_t fp_half = FP32_DIV(FP32_FROM_INT(1), FP32_FROM_INT(2));
    const int32_t fp_2 = FP32_FROM_INT(2);
    const int32_t fp_4 = FP32_FROM_INT(4);
    const int32_t fp_6 = FP32_FROM_INT(6);
    const int32_t fp_255 = FP32_FROM_INT(255);
    int32_t r = FP32_DIV(FP32_FROM_INT(*r_h), fp_255);
    int32_t g = FP32_DIV(FP32_FROM_INT(*g_s), fp_255);
    int32_t b = FP32_DIV(FP32_FROM_INT(*b_l), fp_255);
    int32_t min = MIN3(r, g, b);
    int32_t max = MAX3(r, g, b);
    int32_t vm, h, s, l = FP32_MUL((min + max), fp_half);

    if (min == max) {
        *r_h = 0;
        *g_s = 0;
        *b_l = (uint8_t)FP32_TO_INT(FP32_MUL(l, fp_255));
    }
    else {
        vm = max - min;
        s = l > fp_half ? FP32_DIV(vm, (fp_2 - max - min))
                        : FP32_DIV(vm, (max + min));

        if (max == r)
            h = FP32_DIV((g - b), vm) + (g < b ? fp_6 : 0);
        else if (max == g)
            h = FP32_DIV((b - r), vm) + fp_2;
        else
            h = FP32_DIV((r - g), vm) + fp_4;

        (*r_h) = (uint8_t)FP32_TO_INT(FP32_DIV(FP32_MUL(h, fp_255), fp_6));
        (*g_s) = (uint8_t)FP32_TO_INT(FP32_MUL(s, fp_255));
        (*b_l) = (uint8_t)FP32_TO_INT(FP32_MUL(l, fp_255));
    }
}

// -----------------------------------------------------------------------------
void findColorHSL(const uint8_t *hsl_in, 
        track_color_t *color, track_coords_t *box) {
    int x = 0, y = 0, h = 0, s = 0, l = 0, consec = 0, noise_filter = color->filter;
    int img_width = box->width, img_height = box->height;
    int img_pitch = box->width * 3, scan_start = 0, pix_start = 0;

    // initialize box to obviously invalid state so we know if we didn't detect
    int xs = img_width, x1 = img_width, y1 = img_height, x2 = 0, y2 = 0;

    // iterate over each scanline in the source image
    for (y = 0; y < img_height; ++y) {

        // reset consecutive pixel count and calculate scanline start offset
        consec = 0;
        scan_start = y * img_pitch;

        // iterate over each pixel in the source scanline
        for (x = 0; x < img_width; ++x) {

            // if within threshold, update the bounding box
            pix_start = scan_start + x * 3;
            h = hsl_in[pix_start + 0];
            s = hsl_in[pix_start + 1];
            l = hsl_in[pix_start + 2];   
            
            if (abs(h - (int)color->r) < (int)color->ht &&
                (s > ((int)color->g - (int)color->st)) &&
		        (s < ((int)color->g + (int)color->st)) &&
                (l > 25) && 
                (l < 225)) {

                // only update bounding box of consecutive pixels >= "filter"
                if (consec >= noise_filter) {
                    if (xs < x1) x1 = xs;
                    if (x > x2) x2 = x;
                    if (y < y1) y1 = y;
                    if (y > y2) y2 = y;
                }
                else if (0 == consec) {
                    xs = x;
                }
                ++consec;
            }
            else {
                // reset number of consecutive pixels if we didn't match
                consec = 0;
            }
        }
    }

    // initialize box to obviously invalid state so we know if we didn't detect
    box->x1 = x1;
    box->y1 = y1;
    box->x2 = x2;
    box->y2 = y2;

    if ((0 == x2) && (x1 == img_width) && (0 == y2) && (y1 == img_height)) {
        // color was not detected
        box->detected = 0;
    }
    else {
        // color was detected
        box->detected = 1;
    }
}

// -----------------------------------------------------------------------------
void findColorRGB(const uint8_t *rgb_in,
        const track_color_t *color, track_coords_t *box)
{
    int x = 0, y = 0, consec = 0, noise_filter = color->filter;
    int img_width = box->width, img_height = box->height;
    int img_pitch = img_width * 3, scan_start = 0, pix_start = 0;
    int r_track = color->r, g_track = color->g, b_track = color->b;
    int r_thresh = color->ht, g_thresh = color->st, b_thresh = color->lt;

    // initialize box to obviously invalid state so we know if we didn't detect
    int xs = img_width, x1 = img_width, y1 = img_height, x2 = 0, y2 = 0;

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
                if (consec >= noise_filter) {
                    if (xs < x1) x1 = xs;
                    if (x > x2) x2 = x;
                    if (y < y1) y1 = y;
                    if (y > y2) y2 = y; 
                }
                else if (0 == consec) {
                    xs = x;
                }
                ++consec;
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

    if ((0 == x2) && (x1 == img_width) && (0 == y2) && (y1 == img_height)) {
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
        const track_color_t *color, track_coords_t *box)
{
    int x = 0, y = 0, consec = 0, noise_filter = color->filter;
    int img_width = box->width, img_height = box->height;
    int img_pitch = img_width * 3, scan_start = 0, pix_start = 0;
    int r_track = color->r, g_track = color->g, b_track = color->b;
    int r_diff, g_diff, b_diff;
    int dist;

    // initialize box to obviously invalid state so we know if we didn't detect
    int xs = img_width, x1 = img_width, y1 = img_height, x2 = 0, y2 = 0;

    // square the input threshold value
    threshold *= threshold;

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
                if (consec >= noise_filter) {
                    if (x < x1) x1 = x;
                    if (x > x2) x2 = x;
                    if (y < y1) y1 = y;
                    if (y > y2) y2 = y; 
                }
                else if (0 == consec) {
                    xs = x;
                }
                ++consec;
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

    if ((0 == x2) && (x1 == img_width) && (0 == y2) && (y1 == img_height)) {
        // color was not detected
        box->detected = 0;
    }
    else {
        // color was detected
        box->detected = 1;
    }
}

