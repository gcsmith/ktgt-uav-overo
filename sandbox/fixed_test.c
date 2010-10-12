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

#include "../uav-control/colordetect.h"
#include "../uav-control/fp32.h"

//#define FIXED_Y 8

int main(int argc, char *argv[])
{

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

    const double EPSILON = 0.1;
    double average = 0, err_average = 0;
    int errcount = 0;

    for(i = 1; i < 256; i++) {
        for (j = 1; j < 256; j++) {
            x = 1.0 / i;
            y = 1.0 / j;
            xf = FP32_DIV((FP32_FROM_INT((int)1)), (FP32_FROM_INT((int)i)));
            yf = FP32_DIV((FP32_FROM_INT((int)1)), (FP32_FROM_INT((int)j)));

            result_fix = FP32_DIV(xf,yf);
            fix_integer = FP32_TO_INT(result_fix);
            fix_decimals_double = FP32_TO_FRAC(result_fix);
            fix_result_as_double = FP32_TO_FLOAT(result_fix);

            double_result = (x/y);
            fix_dif = fabs(fix_result_as_double - double_result);

            average += fix_dif;
            if( fix_dif > EPSILON ){
                err_average += fix_dif;
                errcount++;
                printf("DIV x: %f  y: %f  fixI: %d fixD: %f fixR: %f  double: %f  dif: %f\n",
                        x,y, fix_integer,fix_decimals_double,fix_result_as_double,double_result,fix_dif);
            }
#if 0
            if (fabs(FP32_MUL(xf,yf) - x/y) > 0) {
                printf("MUL x:%f  y:%f   dif:%d\n", x,y,fabs(FP32_MUL(xf,yf) - x/y));
            }  
#endif
        }
    }
    printf("#Err: %d Average: %f ERR Avg: %f \n", errcount, average/(255.0*255.0),err_average/errcount);

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
    for (i = 0;i < 256; i++) {
        for (j = 0; j < 256; j++) {
            for (k = 0; k < 256; k++) {
                r_f = r_d = i;
                g_f = g_d = j;
                b_f = b_d = k;

                rgb_to_hsl_fp32(&r_f, &g_f, &b_f);
                rgb_to_hsl(&r_d, &g_d, &b_d);

                err_r = fabs((double)r_f - (double)r_d);
                err_g = fabs((double)g_f - (double)g_d);
                err_b = fabs((double)b_f - (double)b_d);

                average_r += err_r;
                average_g += err_g;
                average_b += err_b;

                if (err_g > max_g) {
                    printf("maxERRg: %f R:%3d G:%3d B:%3d    FP_H: %3d, FP_S: %3d FP_L: %3d    D_H: %3d, D_S: %3d D_L: %3d\n",
                            err_g, i, j, k, r_f, g_f, b_f, r_d, g_d, b_d);
                }

                if (err_r> max_r) {
                    printf("maxERRr: %f R:%3d G:%3d B:%3d    FP_H: %3d, FP_S: %3d FP_L: %3d    D_H: %3d, D_S: %3d D_L: %3d\n",
                            err_r, i, j, k, r_f, g_f, b_f, r_d, g_d, b_d);
                }

                max_r = (err_r > max_r) ? err_r : max_r;
                max_g = (err_g > max_g) ? err_g : max_g; 
                max_b = (err_b > max_b) ? err_b : max_b;

                min_r = (err_r < min_r) ? err_r : min_r;
                min_g = (err_g < min_g) ? err_g : min_g;
                min_b = (err_b < min_b) ? err_b : min_b;

                if (err_b > 1.0) {
                    printf("ERRB: %f R:%d G:%d B:%d , FP_H: %d, FP_S: %d FP_L: %d D_H: %d, D_S: %d D_L: %d\n",
                            err_b, i, j, k, r_f, g_f, b_f, r_d, g_d, b_d);
                }
            }
        }
    }

    average_r /= (255 * 255 * 255);    
    average_g /= (255 * 255 * 255); 
    average_b /= (255 * 255 * 255); 

    printf("max: r: %f g:%f b:%f\n", max_r, max_g, max_b);
    printf("min: r: %f g:%f b:%f\n", min_r, min_g, min_b);
    printf("r_h: %f, g_s: %f, b_l: %f\n",average_r,average_g,average_b);

    //printf("divtest: %d\n",FP32_TO_INT(FP32_DIV(FP32_FROM_INT(100),FP32_FROM_INT(1))));

    return 0;
}

