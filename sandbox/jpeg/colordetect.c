#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "jpeglib.h"
#include "readwritejpeg.h"
JSAMPLE * image_buffer;
int image_height;
int image_width;

void findColorRGB(unsigned char* RGBimage, unsigned char* BWimage,int width, int height,unsigned char R, unsigned char G, unsigned char B, int threshold);


void COLORimageRGBtoHSL(unsigned char** COLORimage, int width, int height);
void RGB2HSL (unsigned char R_in, unsigned char G_in, unsigned char B_in,
              unsigned char * h_out, unsigned char * s_out, unsigned char * l_out);

double max_double(double x, double y);
double min_double(double x, double y);


void printCOLORimage(int height, int width, unsigned char * COLORimage);
void printBWimage(int height, int width, unsigned char * BWimage);

int main(){
   unsigned char * RGBimage;
   unsigned char * BWimage;

   int i = 0;
   int j = 0;
   int k = 0;

   printf("Starting Test\n");

   read_JPEG_file("small.jpg",&RGBimage,&image_width,&image_height);

   printf("\n\n RGBImage\n");

   printCOLORimage(image_height,image_width,RGBimage);

   printf("\nDone getting Image - Start Threshold\n");
   BWimage = calloc( image_width * image_height, sizeof(unsigned char)  ); 
   
   findColorRGB(RGBimage,BWimage,image_width,image_height, 255, 0, 0, 300);

   printf("\nPrinting Detected color\n");

   printBWimage(image_height,image_width, BWimage);

   printf("\nRGB to HSL\n");

  COLORimageRGBtoHSL(&RGBimage, image_width, image_height);

   printCOLORimage(image_height,image_width,RGBimage);
   printf("End Test\n");
}


void findColorRGB(unsigned char* RGBimage, unsigned char* BWimage,int width, int height,unsigned char R, unsigned char G, unsigned char B, int threshold){
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

void COLORimageRGBtoHSL(unsigned char** COLORimage, int width, int height){
    int i = 0;
    int j = 0;
    for( i = 0; i < height; i++){
      
       for ( j = 0; j < width; j++){
        RGB2HSL( (*COLORimage)[(i * height * 3) + (j * 3) + 0],(*COLORimage)[(i * height * 3) + (j * 3) + 1],(*COLORimage)[(i * height * 3) + (j * 3) + 2],
	         (*COLORimage)+(i * height * 3) + (j * 3) + 0,(*COLORimage)+(i * height * 3) + (j * 3) + 1,(*COLORimage)+(i * height * 3) + (j * 3) + 2);



       }
    }
}

void RGB2HSL (unsigned char R_in, unsigned char G_in, unsigned char B_in,
              unsigned char * h_out, unsigned char * s_out, unsigned char * l_out)
{
double r = R_in/255.0;
double g = G_in/255.0;
double b = B_in/255.0;
double v;
double m;
double vm;
double r2, g2, b2;
double h,s,l;

h = 0; // default to black
s = 0;
l = 0;
v = max_double(r,g);
v = max_double(v,b);
m = min_double(r,g);
m = min_double(m,b);
l = (m + v) / 2.0;
if (l <= 0.0)
{
    return;
}
vm = v - m;
s = vm;
if (s > 0.0)
{
    s /= (l <= 0.5) ? (v + m ) : (2.0 - v - m) ;
}
else
{
    return;
}
r2 = (v - r) / vm;
g2 = (v - g) / vm;
b2 = (v - b) / vm;
if (r == v)
{
    h = (g == m ? 5.0 + b2 : 1.0 - g2);
}
else if (g == v)
{
    h = (b == m ? 1.0 + r2 : 3.0 - b2);
}
else
{
    h = (r == m ? 3.0 + g2 : 5.0 - r2);
}
    h /= 6.0;
(*h_out) = h * 255;
(*s_out) = s * 255;
(*l_out) = l * 255;
}

double max_double(double x, double y){
 return x > y? x : y;
 }
 double min_double(double x, double y){
 return x< y? x : y;
 }

 void printBWimage(int height, int width, unsigned char * BWimage){
   int i = 0;
   int j = 0;
   for(i = 0; i < height; i++){
   	for(j=0; j < width; j++){
	    printf("%3d ",(unsigned char) BWimage[(i * height ) + j]);
	}
	printf("\n");
   }
 } 


 void printCOLORimage(int height, int width, unsigned char * COLORimage){
   int i = 0;
   int j = 0;
   int k = 0;
   for(i = 0; i < height; i++){
   	for(j=0; j < width; j++){
		for(k = 0; k < 3; k++){
			//printf("%3d:",(i * height*3) + (j*3) + k);
			printf("%3d ",(unsigned char) COLORimage[(i * height * 3) + (j*3) + k]);
		}
	}
	printf("\n");
   }
}

