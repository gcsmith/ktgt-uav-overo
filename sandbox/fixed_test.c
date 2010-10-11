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
#include <../uav-control/fixed.h>
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
    
    int i = 0;
    int j = 0;
    int k = 0;
    
    float x, y;
    int xf = 0;
    int yf = 0;
    
    int result_fix = 0;
    double fix_dif = 0;
    int fix_integer = 0;
    double fix_decimals_double = 0;
    double fix_result_as_double = 0;
    double double_result = 0;
    
    double average = 0;
    double errcount = 0;
    for(i=1; i<256; i++){
        for(j=1; j<256; j++) {
            x = 1.0/i;
            y = 1.0/j;
            xf = FIX_DIV((INT_2_FIX((int)1)),(INT_2_FIX((int)i)));
            yf = FIX_DIV((INT_2_FIX((int)1)),(INT_2_FIX((int)j)));
            
            result_fix = FIX_DIV(xf,yf);
            fix_integer = FIX_2_INT(result_fix);
            //printf("fixint: %d\n", fix_integer);
            fix_decimals_double = (FIX_DECIMALS(result_fix));
            //printf("fixdec: %d",FIX_DECIMALS(result_fix));
            fix_decimals_double /= 255.0;
            //printf("fixdoub: %f\n",fix_decimals_double);
            fix_result_as_double = (double)fix_decimals_double + (double)fix_integer;
            
            double_result = (x/y);
            fix_dif = fix_result_as_double - double_result;
            
            average += fix_dif;
            if( abs(fix_dif) > 0 ){
                errcount++;
                printf("DIV x: %f  y: %f  fixI: %d fixD: %f fixR: %f  double: %f  dif: %f\n",
                    x,y, fix_integer,fix_decimals_double,fix_result_as_double,double_result,fix_dif);
            }
            /*
            if( abs(FIX_MULT(xf,yf) - x/y) > 0 ){
               printf("MUL x:%f  y:%f   dif:%d\n", x,y,abs(FIX_MULT(xf,yf) - x/y));
            }  
            */      
        }
    }
    printf("Average: %f ERR Avg: %f \n",average/(255.0*255.0),average/errcount);

double err_r = 0;
double err_g = 0;
double err_b = 0;   

double max_r = 0; 
double max_g = 0;
double max_b = 0;

double min_r = 255; 
double min_g = 255;
double min_b = 255;
//RGB TO HSL Average ERROR
    for(i = 0;i < 256; i++){
        for(j = 0; j < 256; j++){
            for(k = 0; k < 256; k++){
                r_f = r_d = i;
                g_f = g_d = j;
                b_f = b_d = k;
                
                RGB2HSLfixed(&r_f,&g_f,&b_f);
                RGB2HSL(&r_d,&g_d, &b_d);
                
                
                err_r = abs((double)r_f - (double)r_d);
                err_g = abs((double)g_f - (double)g_d);
                err_b = abs((double)b_f - (double)b_d);
                
                average_r += err_r;
                average_g += err_g;
                average_b += err_b;
                
                max_r = (err_r > max_r) ? err_r : max_r;
                max_g = (err_g > max_g) ? err_g : max_g;
                max_b = (err_b > max_b) ? err_b : max_b;
                
                min_r = (err_r < min_r) ? err_r : min_r;
                min_g = (err_g < min_g) ? err_g : min_g;
                min_b = (err_b < min_b) ? err_b : min_b;
                
            
            }
        }
    }
        
    average_r /= (255 * 255 * 255);    
    average_g /= (255 * 255 * 255); 
    average_b /= (255 * 255 * 255); 
        
    
    
    printf("max: r: %f g:%f b:%f\n", max_r, max_b, max_g);
    printf("min: r: %f g:%f b:%f\n", min_r, min_b, min_g);
    printf("r_h: %f, g_s: %f, b_l: %f\n",average_r,average_g,average_b);
    
    printf("divtest: %d\n",FIX_2_INT(FIX_DIV(INT_2_FIX(100),INT_2_FIX(1))));

    return 0;
}

