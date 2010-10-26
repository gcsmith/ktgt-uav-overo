// -----------------------------------------------------------------------------
// File:    colordetect.h
// Authors: Tyler Thierolf, Timothy Miller, Garrett Smith
// Created: 10-01-2010
//
// Routines for detecting solid colors within RGB or HSL images, as well as
// algorithms to convert between color spaces.
// -----------------------------------------------------------------------------

#ifndef _UAV_COLORDETECT__H_
#define _UAV_COLORDETECT__H_

#include "utility.h"
#include <time.h>

typedef struct track_color {
    uint8_t r, g, b;    // red/green/blue color channels
    uint8_t ht, st, lt; // hue/saturation/lightness thresholds
    int filter;         // simple filter (consecutive pixels)
} track_color_t;

typedef struct {
    int width;      // width of image in pixels
    int height;     // height of image in pixels
    int x1, y1;     // top-left coordinate of bounding box
    int x2, y2;     // bottom-right coordinate of bounding box
    int xc, yc;     // centroid coordinate of tracked color blob
    int detected;   // 0 if color not detected, 1 if detected
} track_coords_t;

int color_detect_init(client_info_t *client);
void color_detect_shutdown(void);
void color_detect_set_track_color(track_color_t *color);
track_color_t color_detect_get_track_color();
void color_detect_enable(int enabled);

// determine color bounding box using simple red/green/blue thresholding
void color_detect_rgb(const uint8_t *rgb_in,
        const track_color_t *color, track_coords_t *box);

// determine color bounding box using more complex red/green/blue thresholding
void color_detect_rgb_dist(const uint8_t *rgb_in, real_t threshold,
        const track_color_t *color, track_coords_t *box);

// determine color bounding box using hue/saturation/lightness thresholding
void color_detect_hsl(uint8_t *rgb_in, 
        const track_color_t *color, track_coords_t *box);
        
void color_detect_hsl_fp32(uint8_t *rgb_in, 
        const track_color_t *color, track_coords_t *box);

void run_color_detection_file(const char *infile, const char *outfile,
        track_color_t *color, track_coords_t *box);

void run_color_detection_memory(const uint8_t *stream_in, unsigned long *length,
        track_color_t *color, track_coords_t *box);

void find_color_rgb(const uint8_t *rgb_in,
        const track_color_t *color, track_coords_t *box);

void find_color_rgb_dist(const uint8_t *rgb_in, int threshold,
        const track_color_t *color, track_coords_t *box);

void find_color_hsl(const uint8_t *hsl_in, 
        track_color_t *color, track_coords_t *box);

void color_image_rgb_to_hsl( uint8_t *rgb_in, int width, int height);
void color_image_rgb_to_hsl_fixed( uint8_t *rgb_in, int width, int height);

void rgb_to_hsl(uint8_t * r_h, uint8_t * g_s, uint8_t * b_l);
void rgb_to_hsl_fp32(uint8_t * r_h, uint8_t * g_s, uint8_t * b_l);

void color_detect_set_tracking_rate(unsigned int fps);
int color_detect_get_tracking_rate();

#endif // _UAV_COLORDETECT__H_

