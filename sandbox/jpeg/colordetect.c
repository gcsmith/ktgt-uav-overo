#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "jpeglib.h"
#include "readwritejpeg.h"
JSAMPLE * image_buffer;
   int image_height;
   int image_width;

void findColor(unsigned char* RGBimage, unsigned char* BWimage,int width, int height,unsigned char R, unsigned char G, unsigned char B, int threshold);
int main(){
   unsigned char * RGBimage;
   unsigned char * BWimage;
   printf("Starting Test\n");
   read_JPEG_file("small.jpg",&RGBimage,&image_width,&image_height);
   printf("\n\n RGBImage\n");
   int i = 0;
   int j = 0;
   int k = 0;
   printf("%3d %3d\n",(unsigned char) RGBimage[0],(unsigned char)RGBimage[1]);
   for(i = 0; i < image_height; i++){
   	for(j=0; j < image_width; j++){
		for(k = 0; k < 3; k++){
			//printf("%3d:",(i * image_height*3) + (j*3) + k);
			printf("%3d ",(unsigned char) RGBimage[(i * image_height * 3) + (j*3) + k]);
		}
	}
	printf("\n");
   }

   printf("\nDone getting Image - Start Threshold\n");
   BWimage = calloc( image_width * image_height, sizeof(unsigned char)  ); 
   
   findColor(RGBimage,BWimage,image_width,image_height, 255, 0, 0, 300);

   printf("\nPrinting Detected color\n");

   for(i = 0; i < image_height; i++){
   	for(j=0; j < image_width; j++){
	    printf("%3d ",(unsigned char) BWimage[(i * image_height ) + j]);
	}
	printf("\n");
   }
   


   printf("End Test\n");
}


void findColor(unsigned char* RGBimage, unsigned char* BWimage,int width, int height,unsigned char R, unsigned char G, unsigned char B, int threshold){
    int i = 0;
    int j = 0;
    for( i = 0; i < height; i++){
      
       for ( j = 0; j < width; j++){
	 if(sqrt( (double)(pow((double)( (unsigned char) RGBimage[(i * height * 3) + (j*3) + 0]) - R , 2 ) ) +
                     (double)(pow((double)( (unsigned char) RGBimage[(i * height * 3) + (j*3) + 1]) - G , 2 ) ) +
                     (double)(pow((double)( (unsigned char) RGBimage[(i * height * 3) + (j*3) + 2]) - B , 2 ) ) 
		  ) < (double)threshold ){
                BWimage[ (i * height) + j ] = 1;
            }else{
                BWimage[ (i * height) + j ] = 0;
            }
       }
    }
}
