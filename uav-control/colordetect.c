// -----------------------------------------------------------------------------
// File:    colordetect.c
// Authors: Tyler Thierolf, Timothy Miller, Garrett Smith
// Created: 10-01-2010
//
// Routines for detecting solid colors within RGB or HSL images, as well as
// algorithms to convert between color spaces.
// -----------------------------------------------------------------------------

#include <stdlib.h>
#include "readwritejpeg.h"
#include "colordetect.h"
#include "utility.h"
#include "fp32.h"

// -----------------------------------------------------------------------------
void colordetect_rgb(const uint8_t *rgb_in,
        const track_color_t *color, track_coords_t *box)
{
    find_color_rgb(rgb_in, color, box);
}

// -----------------------------------------------------------------------------
void colordetect_rgb_dist (const uint8_t *rgb_in, real_t threshold,
        const track_color_t *color, track_coords_t *box)
{
    find_color_rgb_dist(rgb_in, threshold, color, box);
}

// -----------------------------------------------------------------------------
void colordetect_hsl(uint8_t *rgb_in, 
        const track_color_t *color, track_coords_t *box) 
{
    track_color_t track_color = *color;
    // convert detect color from RGB to HSL
    rgb_to_hsl(&track_color.r, &track_color.g, &track_color.b);                     
                
    // convert the image to HSL
    color_image_rgb_to_hsl(rgb_in, box->width, box->height);

    // find HSL values in image 
    find_color_hsl(rgb_in, &track_color, box);
}

// -----------------------------------------------------------------------------
void colordetect_hsl_fp32(uint8_t *rgb_in, 
        const track_color_t *color, track_coords_t *box) 
{
    track_color_t track_color = *color;
    // convert detect color from RGB to HSL
    rgb_to_hsl_fp32(&track_color.r, &track_color.g, &track_color.b);                     
                        
    // convert the image to HSL
    color_image_rgb_to_hsl_fixed(rgb_in, box->width, box->height);

    // find HSL values in image 
    find_color_hsl(rgb_in, &track_color, box);
}

// -----------------------------------------------------------------------------
void run_color_detection_file(const char *infile, const char *outfile,
        track_color_t *color, track_coords_t *box)
{
    uint8_t *rgb = NULL;

    // decode image from file, run color detection algorithm, write it back out
    jpeg_rd_file(infile, &rgb, &box->width, &box->height);    
    colordetect_hsl(rgb, color, box);
#if 0
    jpeg_wr_file(outfile, color->quality, rgb, box->width, box->height);
#endif
}

// -----------------------------------------------------------------------------
void run_color_detection_memory(const uint8_t *stream_in, unsigned long *length,
        track_color_t *color, track_coords_t *box)
{
    uint8_t *rgb = NULL;

    // decode image from mem, run color detection algorithm, write it back out
    jpeg_rd_mem(stream_in, *length, &rgb, &box->width, &box->height);    
    colordetect_hsl(rgb, color, box);
#if 0
    jpeg_wr_mem(&stream_in, length, color->quality, rgb, &box->width, &box->height);
#endif
}

// -----------------------------------------------------------------------------
void color_image_rgb_to_hsl(uint8_t *rgb_in, int width, int height)
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
void color_image_rgb_to_hsl_fixed(uint8_t *rgb_in, int width, int height)
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
void find_color_hsl(const uint8_t *hsl_in, 
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
void find_color_rgb(const uint8_t *rgb_in,
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
void find_color_rgb_dist(const uint8_t *rgb_in, int threshold,
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

