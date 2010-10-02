#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "jpeglib.h"
#include "readwritejpeg.h"
#include "colordetect.h"
#include <time.h>

int main(int argc, char *argv[]) {
    //int image_height;
    //int image_width;
    //Color to Look for
    //Green Frisbee RGB= 151 242 192
    unsigned char R = 151;
    unsigned char G = 242;
    unsigned char B = 192;

    //HSL Threshold values
    //Green Frisbee on carpet 30 100 360
    short Ht = 30;
    short St = 100;
    short Lt = 360;

    //Define RGBimage
    //unsigned char * RGBimage;
    //Define file name
    const char *filename = "./Data/frisbee.jpg";

    if (argc!= 1 && argc != 2 && argc != 5 && argc != 8) {
        printf("Usage:\n %s filename R G B\n or \n %s filename"
               "R G B Ht St Lt\n *t -> threshold value\n", argv[0], argv[0]);
        return 0;
    }

    if (argc >= 2) {
        //Define File name (first argument 
        filename = argv[1];
    }

    if (argc >= 5) {
        // Define RGB
        R = (unsigned char)atoi(argv[2]);
        G = (unsigned char)atoi(argv[3]);
        B = (unsigned char)atoi(argv[4]);
    }

    if (argc == 8) {    
        // Define a new thereshold
        Ht = (short)atoi(argv[5]);
        St = (short)atoi(argv[6]);
        Lt = (short)atoi(argv[7]);

    }
    
    colorToFind color = {
        .R = R,
        .G = G,
        .B = B,
        .Ht = Ht,
        .St = St,
        .Lt = Lt,
        .quality = 25    
    };
    boxCoordinates box = {};
    
    struct timespec spec1, spec2;
    const int num_iter = 10;
    long long delta;

    for(;;){
        int i;
        clock_gettime(CLOCK_REALTIME, &spec1);
        for( i = 0; i < num_iter; i++){
            runColorDetectionFile (filename, "testimage.jpg", &color, &box);
            if(!(box.x1 == box.width && box.y1 == box.height &&
                box.x2 == 0 && box.y2 == 0 ) ){
                printf( "HSL Bounding box: (%d,%d) (%d,%d)\n",box.x1,box.y1,box.x2,box.y2);
            } else {
                printf( "Target object not found!\n" );
            }
        }
        clock_gettime(CLOCK_REALTIME, &spec2);

        delta = spec2.tv_nsec - spec1.tv_nsec;
        if (spec2.tv_sec > spec1.tv_sec) {
            /* add x second */
            delta += 1000000000 * (spec2.tv_sec - spec1.tv_sec);
        }

        printf("ticks = %lld\n", delta);
        double fps = num_iter / ((double)delta / 1000000000.0);
        printf("%f FPS\n", fps);
    }

#if 0
    read_JPEG_file(filename,&RGBimage,&image_width,&image_height);
    runColorDetection(RGBimage,image_width,image_height, R, B, G, Ht, St, Lt);
    // Write the Image back
    write_JPEG_file ("testimage.jpg", 100,RGBimage,image_width,image_height);
#endif

    printf("End Test\n");
    return 0;
}

