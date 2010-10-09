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
    uint8_t r = 255;
    uint8_t g = 0;
    uint8_t b = 0;
    
    
    
    
    RGB2HSLfixed(&r,&g,&b);
    
    printf("H: %d,S: %d,L: %d\n",r,g,b);
    
    

    return 0;
}

