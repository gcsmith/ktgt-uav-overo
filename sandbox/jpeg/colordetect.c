#include <stdio.h>
#include "jpeglib.h"
#include "readwritejpeg.h"
JSAMPLE * image_buffer;
   int image_height;
   int image_width;

int main(){
   unsigned char * RGBimage;
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
			printf("%3d:",(i * image_height*3) + (j*3) + k);
			printf("%3d ",(unsigned char) RGBimage[(i * image_height * 3) + (j*3) + k]);
		}
	}
	printf("\n");
}

   printf("End Test\n");
}
