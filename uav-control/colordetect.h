#ifndef _UAV_COLORDETECT__H_
#define _UAV_COLORDETECT__H_

#include "utility.h"

typedef struct track_color {
    uint8_t r, g, b;    // red/green/blue channels (why is this here?)
    uint16_t h, s, l;   // hue/saturation/lightness channels
    int filter;         // simple filter (consecutive pixels)
} track_color_t;

typedef struct {
    int width;          // width of image in pixels
    int height;         // height of image in pixels
    int x1, y1;         // top-left coordinate of bounding box
    int x2, y2;         // bottom-right coordinate of bounding box
    int xc, yc;         // centroid coordinate of tracked color blob
    int detected;       // 0 if color not detected, 1 if detected
} track_coords_t;

void runColorDetectionFile(const char *infile, const char *outfile,
        track_color_t *color, track_coords_t *box);

void runColorDetectionMemory(const uint8_t *stream_in, unsigned long *length,
        track_color_t *color, track_coords_t *box);

void runColorDetection(const uint8_t *rgb_in, uint16_t **hsl_buff,
        track_color_t *color, track_coords_t *box);

void findColorRGB(const uint8_t *rgb_in, int width, int height,
        uint8_t R, uint8_t G, uint8_t B, int thresh);

void findColorHSL(const uint16_t *hsl_in, uint16_t h, uint16_t s, uint16_t l,
        track_color_t *color, track_coords_t *box);

void COLORimageRGBtoHSL(const uint8_t *rgb_in, uint16_t *hsl_out,
        int width, int height);

void RGB2HSL(uint8_t r_in, uint8_t g_in, uint8_t b_in,
        uint16_t *h_out, uint16_t *s_out, uint16_t *l_out);

#endif // _UAV_COLORDETECT__H_

