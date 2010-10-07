#ifndef _UAV_COLORDETECT__H_
#define _UAV_COLORDETECT__H_

#include "utility.h"

// WRITE_IMAGE  0

typedef struct track_color {
    uint8_t R;
    uint8_t G; 
    uint8_t B; 
    
    uint16_t Ht;
    uint16_t St; 
    uint16_t Lt;
    
    char quality;
} track_color_t;

typedef struct {
    int width;
    int height;
    
    int x1;
    int y1;
    
    int x2;
    int y2;
    
    int xc;
    int yc;
    char detected;
} track_coords_t;

void runColorDetectionFile(const char *infile, const char *outfile,
                           track_color_t *color, track_coords_t *box);

void runColorDetectionMemory(const uint8_t *buffer, unsigned long *length,
                             track_color_t *color, track_coords_t *box);
                        
void runColorDetection(const uint8_t *buffer,
                       track_color_t * color, track_coords_t * box);

void findColorRGB(const uint8_t *buffer, int width, int height,
                  uint8_t R, uint8_t G, uint8_t B, int thresh);
    
void findColorHSL(const short *buffer, short H, short S, short L,
                  track_color_t *color, track_coords_t *box);
                
void COLORimageRGBtoHSL(const uint8_t *rgb_in, short *hsl_out,
                        int width, int height);

void RGB2HSL (uint8_t R_in, uint8_t G_in, uint8_t B_in,
              short * h_out, short * s_out, short * l_out);

#endif // _UAV_COLORDETECT__H_

