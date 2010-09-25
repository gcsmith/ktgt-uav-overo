#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "jpeglib.h"
#include "readwritejpeg.h"
JSAMPLE * image_buffer;
int image_height;
int image_width;
int noiseFilter = 5;

void findColorRGB(unsigned char* RGBimage, int width, int height,unsigned char R, unsigned char G, unsigned char B, int threshold);
void findColorHSL(short* HSLimage, int width, int height,short H, short S, short L, int Hthreshold, int Sthreshold, int Lthreshold);
void hsv_to_hsl(double h, double s, double v, double* hh, double* ss, double* ll);


void COLORimageRGBtoHSL(unsigned char* COLORimage, short* HSLimage, int width, int height);
void RGB2HSL (unsigned char R_in, unsigned char G_in, unsigned char B_in,
              short * h_out, short * s_out, short * l_out);

double max_double(double x, double y);
double min_double(double x, double y);


void printCOLORimage(int height, int width, unsigned char * COLORimage);
void printBWimage(int height, int width, unsigned char * BWimage);
void findBoundingCoordinates(int height, int width, unsigned char * BWimage);

int main(){
   unsigned char * RGBimage;
   //unsigned char * BWimage;

#if 0
   int i = 0;
   int j = 0;
   int k = 0;
#endif

   printf("Starting Test box.jpeg\n");

   read_JPEG_file("frisbee2.jpg",&RGBimage,&image_width,&image_height);

   printf("\n\n RGBImage\n");

   //printCOLORimage(image_height,image_width,RGBimage);

   printf("\nDone getting Image - Start Threshold\n");
   //BWimage = calloc( image_width * image_height, sizeof(unsigned char)  ); 
   
   findColorRGB(RGBimage,image_width,image_height, 156, 243, 192, 16);

   printf("\nPrinting Detected color\n");

   //printBWimage(image_height,image_width, BWimage);

   printf("\nRGB to HSL\n");
   short * HSLimage = malloc(sizeof(short)*image_height*image_width*3);	
   COLORimageRGBtoHSL(RGBimage, HSLimage, image_width, image_height);

   findColorHSL(HSLimage,image_width,image_height, 145, 18, 90, 4, 4, 20);

  // printCOLORimage(image_height,image_width,RGBimage);
   printf("End Test\n");
   //findBoundingCoordinates(image_height, image_width, BWimage);
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
	 if(sqrt( (double)(pow((double)( (unsigned char) RGBimage[(i * height * 3) + (j*3) + 0]) - R , 2 ) ) +
                     (double)(pow((double)( (unsigned char) RGBimage[(i * height * 3) + (j*3) + 1]) - G , 2 ) ) +
                     (double)(pow((double)( (unsigned char) RGBimage[(i * height * 3) + (j*3) + 2]) - B , 2 ) ) 
		  ) < (double)threshold ){
                //BWimage[ (i * height) + j ] = 1;
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
                //BWimage[ (i * height) + j ] = 0;
		consPix = 0;
            }
       }
    }
    printf("(%d,%d) (%d,%d)\n",x1,y1,x2,y2);
}

void findColorHSL(short* HSLimage, int width, int height,short H, short S, short L, int Hthreshold, int Sthreshold, int Lthreshold){
    int i = 0;
    int j = 0;
    int consPix = 0;
    int x1 = width, x2 = 0, y1 = height, y2 = 0;
    double h,s,l;
    hsv_to_hsl((145.0/360.0),(36.0/360.0),(95.0/360.0),&h,&s,&l);
    printf("%d %d %d\n", (int)(h*360), (int)(s*360), (int)(l*360));
    for( i = 0; i < height; i++){
       consPix = 0;
       for ( j = 0; j < width; j++){
	 if( abs((HSLimage[(i * height * 3) + (j*3) + 0]) - H) < Hthreshold 
	  && abs((HSLimage[(i * height * 3) + (j*3) + 1]) - S) < Sthreshold
	  && abs((HSLimage[(i * height * 3) + (j*3) + 2]) - L) < Lthreshold){
                //BWimage[ (i * height) + j ] = 1;
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
                //BWimage[ (i * height) + j ] = 0;
		consPix = 0;
            }
       }
    }
    printf("(%d,%d) (%d,%d)\n",x1,y1,x2,y2);
}


void COLORimageRGBtoHSL(unsigned char* COLORimage, short* HSLimage, int width, int height){
    int i = 0, j = 0;
    for( i = 0; i < height; i++){ 
       for ( j = 0; j < width; j++){
        RGB2HSL( (COLORimage)[(i * height * 3) + (j * 3) + 0],(COLORimage)[(i * height * 3) + (j * 3) + 1],(COLORimage)[(i * height * 3) + (j * 3) + 2],
	         (HSLimage)+(i * height * 3) + (j * 3) + 0,(HSLimage)+(i * height * 3) + (j * 3) + 1,(HSLimage)+(i * height * 3) + (j * 3) + 2);
       }
    }
}

void RGB2HSL (unsigned char R_in, unsigned char G_in, unsigned char B_in,
              short * h_out, short * s_out, short * l_out)
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

 void findBoundingCoordinates(int height, int width, unsigned char * BWimage){
   int i = 0;
   int j = 0;
   int x1=width, y1=height, x2=0, y2=0;
   for(i = 0; i < height; i++){
     for(j = 0; j < width; j++){
       if(BWimage[(i * height) + j] == 1){
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


