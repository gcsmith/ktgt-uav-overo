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
    
    time_t t1,t2;
    for(;;){
        int i;
        t1 = time(NULL);
        for( i = 0; i < 10; i++){
            runColorDetectionFile (filename, "testimage.jpg", &color, &box);
        }
        t2 = time(NULL);
	float FPS = 10.0/difftime( t2, t1 );
        printf("%f FPS\n", FPS );
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

