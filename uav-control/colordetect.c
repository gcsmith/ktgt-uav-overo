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

JSAMPLE * image_buffer;
int noiseFilter = 5;

// -----------------------------------------------------------------------------
#ifndef STANDALONE_DEMO
void *color_detect_thread(void *arg) {
    printf("IMAGE PROC\n");
    track_color_t color = {
        .R = 151,
        .G = 242,
        .B = 192,
        .Ht = 30,
        .St = 100,
        .Lt = 360,
        .quality = 25    
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

        if (!(box.x1 == box.width &&
              box.y1 == box.height &&
              box.x2 == 0 && box.y2 == 0)) {
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
void runColorDetectionFile(const char * infilename, const char * outfilename,
    track_color_t * color, track_coords_t * box){
                        
    //Define RGBimage
    uint8_t * RGBimage;   
                        
    jpeg_rd_file(infilename,&RGBimage,&box->width,&box->height);    

    runColorDetection(RGBimage,color, box);

#ifdef WRITE_IMAGE
    // Write the Image back
    jpeg_wr_file(outfilename, color->quality, RGBimage, box->width, box->height);
#endif                        
}

// -----------------------------------------------------------------------------
void runColorDetectionMemory(const uint8_t *buffer, unsigned long *length,
    track_color_t * color, track_coords_t * box){
                        
    //Define RGBimage
    uint8_t * RGBimage;   
printf("pre read jpeg\n");                        
    jpeg_rd_mem(buffer,(*length),&RGBimage,&box->width,&box->height);    
printf("post read jpeg\n");
    runColorDetection(RGBimage,color, box);

#ifdef WRITE_IMAGE    
    // Write the Image back
    jpeg_wr_mem (&buffer, length,color->quality,
        RGBimage,&box->width,&box->height);                   
#endif                        
}

// -----------------------------------------------------------------------------
void runColorDetection(const uint8_t *rgb_in,
                       track_color_t *color, track_coords_t *box) {
    
    // color to look for in HSL (will be calculated from RGB)
    short H = 0;
    short S = 0;
    short L = 0;                   

    // convert detect color from RGB to HSL
    RGB2HSL(color->R, color->G, color->B, &H, &S, &L);                     
                        
    // convert the image to HSL
    short *hsl_buff = malloc(sizeof(short) * box->height * box->width * 3);
    COLORimageRGBtoHSL(rgb_in, hsl_buff, box->width, box->height);

    // find HSL values in image 
    findColorHSL(hsl_buff, H, S, L, color, box);
}

// -----------------------------------------------------------------------------
void findColorHSL(const short *buffer, short H, short S, short L,
                  track_color_t *color, track_coords_t *box) {
                 
    int i = 0;
    int j = 0;
    int consPix = 0;
    box->x1 = box->width;
    box->y1 = box->height;
    box->x2 = 0;
    box->y2 = 0;

    for( i = 0; i < box->height; i++){
        consPix = 0;
        for ( j = 0; j < box->width; j++){
            if( abs((buffer[(i * box->width * 3) + (j*3) + 0]) - H) < color->Ht
             && abs((buffer[(i * box->width * 3) + (j*3) + 1]) - S) < color->St
             && abs((buffer[(i * box->width * 3) + (j*3) + 2]) - L) < color->Lt){
                consPix++;
                if(consPix >= noiseFilter){
                    if(j < box->x1)
                        box->x1 = j;
                    if(j > box->x2)
                        box->x2 = j;
                    if(i < box->y1)
                        box->y1 = i;
                    if(i > box->y2)
                        box->y2 = i; 
                }
            }else{
                consPix = 0;
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
                        int width, int height) {
    int i = 0, j = 0;
    for (i = 0; i < height; i++) {
        for (j = 0; j < width; j++) {
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
void RGB2HSL (uint8_t R_in, uint8_t G_in, uint8_t B_in,
              short * h_out, short * s_out, short * l_out)
{
    real_t r = R_in/255.0;
    real_t g = G_in/255.0;
    real_t b = B_in/255.0;
    real_t max;
    real_t min;

    real_t vm;
    real_t h,s,l;

    h = 0; // default to black
    s = 0;
    l = 0;
    max = MAX(r,g);
    max = MAX(max,b);
    min = MIN(r,g);
    min = MIN(min,b);
    l = (max + min) / 2.0;

    if(max == min){
      return;
    }

    vm = max - min;

    s = l > 0.5 ? vm / (2 - max - min) : vm / (max + min);

    if( max == r ){
        h = (g - b) / vm + (g < b ? 6.0 : 0);
    } else if(max == g) {
        h = (b - r) / vm + 2.0;
    } else {
        h = (r - g) / vm + 4.0;
    }

    h /= 6.0;

    (*h_out) = h * 360;
    (*s_out) = s * 360;
    (*l_out) = l * 360;
}


//Still needed?

// -----------------------------------------------------------------------------
void findColorRGB(const uint8_t *buffer, int width, int height,
                  uint8_t R, uint8_t G, uint8_t B, int threshold){
    int i = 0;
    int j = 0;
    int consPix = 0;
    int x1 = width, x2 = 0, y1 = height, y2 = 0;
    for( i = 0; i < height; i++){
       consPix = 0;
       for ( j = 0; j < width; j++){
       if(sqrt( (real_t)(pow((real_t)( (uint8_t) buffer[(i * width * 3) + (j*3) + 0]) - R , 2 ) ) +
              (real_t)(pow((real_t)( (uint8_t) buffer[(i * width * 3) + (j*3) + 1]) - G , 2 ) ) +
              (real_t)(pow((real_t)( (uint8_t) buffer[(i * width * 3) + (j*3) + 2]) - B , 2 ) ) 
      ) < (real_t)threshold ){
                
		consPix++;		
		if(consPix >= noiseFilter){
			if(j < x1)
			  x1 = j;
		        if(j > x2)
		          x2 = j;
		        if(i < y1)
		          y1 = i;
		        if(i > y2)
		          y2 = i; 
		}
            }else{
               
		consPix = 0;
            }
       }
    }
    printf("(%d,%d) (%d,%d)\n",x1,y1,x2,y2);
}

