#ifndef _UAV_COLORDETECT__H_
#define _UAV_COLORDETECT__H_

#include "utility.h"

typedef struct track_color {
    uint8_t r, g, b;        // red/green/blue channels (why not hsl?)
    uint16_t ht, st, lt;    // hue/saturation/lightness thresholds
    int filter;             // simple filter (consecutive pixels)
} track_color_t;

typedef struct {
    int width;      // width of image in pixels
    int height;     // height of image in pixels
    int x1, y1;     // top-left coordinate of bounding box
    int x2, y2;     // bottom-right coordinate of bounding box
    int xc, yc;     // centroid coordinate of tracked color blob
    int detected;   // 0 if color not detected, 1 if detected
} track_coords_t;

void colordetect_init(void);
void colordetect_shutdown(void);

// determine color bounding box using simple red/green/blue thresholding
void color_detect_rgb(const uint8_t *rgb_in,
        track_color_t *color, track_coords_t *box);

// determine color bounding box using more complex red/green/blue thresholding
void color_detect_rgb_sqrt(const uint8_t *rgb_in, real_t threshold,
        track_color_t *color, track_coords_t *box);

// determine color bounding box using hue/saturation/lightness thresholding
void color_detect_hsl(const uint8_t *rgb_in, uint16_t **hsl_buff,
        track_color_t *color, track_coords_t *box);

void runColorDetectionFile(const char *infile, const char *outfile,
        track_color_t *color, track_coords_t *box);

void runColorDetectionMemory(const uint8_t *stream_in, unsigned long *length,
        track_color_t *color, track_coords_t *box);

void findColorRGB(const uint8_t *rgb_in,
        track_color_t *color, track_coords_t *box);

void findColorRGB_sqrt(const uint8_t *rgb_in, real_t threshold,
        track_color_t *color, track_coords_t *box);

void findColorHSL(const uint16_t *hsl_in, uint16_t h, uint16_t s, uint16_t l,
        track_color_t *color, track_coords_t *box);

void COLORimageRGBtoHSL(const uint8_t *rgb_in, uint16_t *hsl_out,
        int width, int height);

void RGB2HSL(uint8_t r_in, uint8_t g_in, uint8_t b_in,
        uint16_t *h_out, uint16_t *s_out, uint16_t *l_out);

#endif // _UAV_COLORDETECT__H_

