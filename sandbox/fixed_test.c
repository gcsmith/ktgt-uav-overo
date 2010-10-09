#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <jpeglib.h>
#include <getopt.h>
#include <syslog.h>
#include <errno.h>
#include <assert.h>
#include <string.h>
#include <../uav-control/colordetect.h>
//#define FIXED_Y 8


int main(){
        
    uint8_t r_f = 255;
    uint8_t g_f = 0;
    uint8_t b_f = 0;
    
    uint8_t r_d = 255;
    uint8_t g_d = 0;
    uint8_t b_d = 0;
    
    
    double average_r = 0;
    double average_g = 0;
    double average_b = 0;
    /*
    double max_r = 0;
    double max_b = 0;
    double max_g = 0;
    
    double min_r = 255;
    double min_b = 255;
    double min_g = 255;
    */
    
    int i = 0;
    int j = 0;
    int k = 0;
    
    for(i = 0;i < 256; i++){
        for(j = 0; j < 256; j++){
            for(k = 0; k < 256; k++){
                r_f = r_d = i;
                g_f = g_d = j;
                b_f = b_d = k;
                
                RGB2HSLfixed(&r_f,&g_f,&b_f);
                RGB2HSL(&r_d,&g_d, &b_d);
                
                average_r += abs((double)r_f - (double)r_d);
                average_g += abs((double)g_f - (double)g_d);
                average_b += abs((double)b_f - (double)b_d);
                
                
            
            }
        }
    }
        
    average_r /= (255 * 255 * 255);    
    average_g /= (255 * 255 * 255); 
    average_b /= (255 * 255 * 255); 
        
    
    
    
    printf("r: %f,g: %f,b: %f\n",average_r,average_r,average_b);
    
    

    return 0;
}

