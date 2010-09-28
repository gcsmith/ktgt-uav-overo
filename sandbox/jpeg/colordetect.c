#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "jpeglib.h"
#include "readwritejpeg.h"
#include "colordetect.h"
JSAMPLE * image_buffer;
int image_height;
int image_width;
int noiseFilter = 5;

int main(){
   unsigned char * RGBimage;

   read_JPEG_file("frisbee.jpg",&RGBimage,&image_width,&image_height);

   findColorRGB(RGBimage,image_width,image_height, 255, 0, 0, 16);

   // Convert the image to HSL
   short * HSLimage = malloc(sizeof(short)*image_height*image_width*3);	  
   
   COLORimageRGBtoHSL(RGBimage, HSLimage, image_width, image_height);



   // Find HSL Values in image
   findColorHSL(HSLimage,image_width,image_height, 146, 279, 278, 30, 100, 360,RGBimage);

   // Write the Image back
   write_JPEG_file ("testimage.jpg", 100,RGBimage,image_width,image_height);

   printf("End Test\n");
   return 0;
}


void findColorRGB(unsigned char* RGBimage,int width, int height,unsigned char R, unsigned char G, unsigned char B, int threshold){
    int i = 0;
    int j = 0;
    int consPix = 0;
    int x1 = width, x2 = 0, y1 = height, y2 = 0;
    for( i = 0; i < height; i++){
       consPix = 0;
       for ( j = 0; j < width; j++){
	 if(sqrt( (double)(pow((double)( (unsigned char) RGBimage[(i * width * 3) + (j*3) + 0]) - R , 2 ) ) +
                     (double)(pow((double)( (unsigned char) RGBimage[(i * width * 3) + (j*3) + 1]) - G , 2 ) ) +
                     (double)(pow((double)( (unsigned char) RGBimage[(i * width * 3) + (j*3) + 2]) - B , 2 ) ) 
		  ) < (double)threshold ){
                
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

void findColorHSL(short* HSLimage, int width, int height,short H, short S, short L, int Hthreshold, int Sthreshold, int Lthreshold,unsigned char * RGBimage){
    int i = 0;
    int j = 0;
    int consPix = 0;
    int x1 = width, x2 = 0, y1 = height, y2 = 0;
    int box = 0;
    double h,s,l;
    //hsv_to_hsl((145.0/360.0),(36.0/360.0),(95.0/360.0),&h,&s,&l);
    //printf("%d %d %d\n", (int)(h*360), (int)(s*360), (int)(l*360));
    for( i = 0; i < height; i++){
        consPix = 0;

        for ( j = 0; j < width; j++){

            if( abs((HSLimage[(i * width * 3) + (j*3) + 0]) - H) < Hthreshold 
            && abs((HSLimage[(i * width * 3) + (j*3) + 1]) - S) < Sthreshold
            && abs((HSLimage[(i * width * 3) + (j*3) + 2]) - L) < Lthreshold){

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
    printf("HSL Bounding box: (%d,%d) (%d,%d)\n",x1,y1,x2,y2);
}


void COLORimageRGBtoHSL(unsigned char* COLORimage, short* HSLimage, int width, int height){
    int i = 0, j = 0;
    for( i = 0; i < height; i++){ 
       for ( j = 0; j < width; j++){
        RGB2HSL((COLORimage)[(i * width * 3) + (j * 3) + 0],
                (COLORimage)[(i * width * 3) + (j * 3) + 1],
                (COLORimage)[(i * width * 3) + (j * 3) + 2],
                (HSLimage)+(i * width * 3) + (j * 3) + 0,
                (HSLimage)+(i * width * 3) + (j * 3) + 1,
                (HSLimage)+(i * width * 3) + (j * 3) + 2);
       }
    }
}

void RGB2HSL (unsigned char R_in, unsigned char G_in, unsigned char B_in,
              short * h_out, short * s_out, short * l_out)
{
    double r = R_in/255.0;
    double g = G_in/255.0;
    double b = B_in/255.0;
    double max;
    double min;

    double vm;
    double h,s,l;

    h = 0; // default to black
    s = 0;
    l = 0;
    max = max_double(r,g);
    max = max_double(max,b);
    min = min_double(r,g);
    min = min_double(min,b);
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



double max_double(double x, double y){
    return x > y? x : y;
}
double min_double(double x, double y){
    return x< y? x : y;
}




//Still needed?
 void hsv_to_hsl(double h, double s, double v, double* hh, double* ss, double* ll){ 
     printf("%f %f %f\n", h, s, v);
     *hh = h;
     *ll = (2 - s) * v;
     *ss = s * v;
     if (*ll <= 1) *ss /= *ll;
     else *ss /= (2 - *ll);
     *ll /= 2;

     printf("%f %f %f\n", *hh, *ss, *ll);
 }

//NOT NEEDED

 void printCOLORimageHSLshort(int height, int width, short * COLORimage){
   int i = 0;
   int j = 0;
   int k = 0;
   for(i = 0; i < height; i++){
   	for(j=0; j < width; j++){
		for(k = 0; k < 3; k++){
			//printf("%3d:",(i * width*3) + (j*3) + k);
			printf("%3d ",(short) COLORimage[(i * width * 3) + (j*3) + k]);
		}
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
			//printf("%3d:",(i * width*3) + (j*3) + k);
			printf("%3d ",(unsigned char) COLORimage[(i * width * 3) + (j*3) + k]);
		}
	}
	printf("\n");
   }
 }

void printBWimage(int height, int width, unsigned char * BWimage){
    int i = 0;
    int j = 0;
    for(i = 0; i < height; i++){
        for(j=0; j < width; j++){
            printf("%3d ",(unsigned char) BWimage[(i * width ) + j]);
        }
        printf("\n");
    }
} 

 void findBoundingCoordinates(int height, int width, unsigned char * BWimage){
   int i = 0;
   int j = 0;
   int x1=width, y1=height, x2=0, y2=0;
   for(i = 0; i < height; i++){
     for(j = 0; j < width; j++){
       if(BWimage[(i * width) + j] == 1){
         if(j < x1){
	   x1 = j;
         }else if(j > x2){
           x2 = j;
         }
         if(i < y1){
	   y1 = i;
	 }else if(i > y2){
	   y2 = i;
	 }
       }
     }
   }
   printf("(%d,%d) (%d,%d)\n", x1, y1, x2, y2);	
 }



