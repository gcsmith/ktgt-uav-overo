#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "jpeglib.h"
#include "readwritejpeg.h"
#include "colordetect.h"
#include <syslog.h>
#include "utility.h"
JSAMPLE * image_buffer;

#define REAL double

int noiseFilter = 5;

void runColorDetectionFile (const char * infilename, const char * outfilename,
    int quality,
    unsigned char R,unsigned char G, unsigned char B, 
    short Ht, short St, short Lt){
                        
    int width = 0;
    int height = 0;
    //Define RGBimage
    unsigned char * RGBimage;   
                        
    read_JPEG_file(infilename,&RGBimage,&width,&height);    

    runColorDetection(RGBimage,width,height, R, B, G, Ht, St, Lt);

#ifdef WRITE_IMAGE
    // Write the Image back
    write_JPEG_file(outfilename,quality,RGBimage,width,height);                   
#endif                        
}

void runColorDetectionMemory(unsigned char * inbuffer, unsigned long * insize,
    int quality,
    unsigned char R,unsigned char G, unsigned char B, 
    short Ht, short St, short Lt){
                        
    int width = 0;
    int height = 0;
    //Define RGBimage
    unsigned char * RGBimage;   
                        
    read_JPEG_stream(inbuffer,(*insize),&RGBimage,&width,&height);    

    runColorDetection(RGBimage,width,height, R, B, G, Ht, St, Lt);

#ifdef WRITE_IMAGE    
    // Write the Image back
    write_JPEG_stream (&inbuffer, insize,quality,RGBimage,width,height);                   
#endif                        
}

void runColorDetection(unsigned char * RGBimage,int image_width, int image_height,
    unsigned char R,unsigned char G, unsigned char B, 
    short Ht, short St, short Lt){
    
     //Color to look for in HSL (will be calculated from RGB)
    short H = 0;
    short S = 0;
    short L = 0;                   

   //Convert detect color from RGB to HSL
    RGB2HSL (R, G, B,&H, &S, &L);                     
                        
    // Convert the image to HSL
    short * HSLimage = malloc(sizeof(short) * image_height * image_width * 3);	  

    COLORimageRGBtoHSL(RGBimage, HSLimage, image_width, image_height);
    // Find HSL Values in image
    findColorHSL(HSLimage, image_width, image_height, H, S, L, Ht, St, Lt,RGBimage);
}

void findColorHSL(short* HSLimage, int width, int height,
                short H, short S, short L,
                int Hthreshold, int Sthreshold, int Lthreshold,
                unsigned char * RGBimage){
    int i = 0;
    int j = 0;
    int consPix = 0;
    int x1 = width, x2 = 0, y1 = height, y2 = 0;


#ifdef DRAW_BOUNDING_BOX
    int thickness = 4;  
    
    unsigned char Rbox = 0;
    unsigned char Gbox = 255;
    unsigned char Bbox = 0;
    
    unsigned char Rnoise = 0;
    unsigned char Gnoise = 0;
    unsigned char Bnoise = 255;
    
    unsigned char Rvalid = 255;
    unsigned char Gvalid = 0;
    unsigned char Bvalid = 0;
#endif

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
#ifdef DRAW_BOUNDING_BOX                      
                    
                        RGBimage[(i * width * 3) + (j*3) + 0] = Rnoise;
                        RGBimage[(i * width * 3) + (j*3) + 1] = Gnoise;
                        RGBimage[(i * width * 3) + (j*3) + 2] = Bnoise;                    
                      
#endif                                                         
                }
#ifdef DRAW_BOUNDING_BOX                
               
                    RGBimage[(i * width * 3) + (j*3) + 0] = Rvalid;
                    RGBimage[(i * width * 3) + (j*3) + 1] = Gvalid;
                    RGBimage[(i * width * 3) + (j*3) + 2] = Bvalid;                    
                 
#endif                
            }else{
                consPix = 0;
            }
        }
    }
#ifdef DRAW_BOUNDING_BOX    
        drawBoundingbox(RGBimage,width,height, x1, y1, x2, y2, thickness, Rbox, Gbox, Bbox);
#endif
syslog(LOG_INFO, "HSL Bounding box: (%d,%d) (%d,%d)\n",x1,y1,x2,y2);
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
    REAL r = R_in/255.0;
    REAL g = G_in/255.0;
    REAL b = B_in/255.0;
    REAL max;
    REAL min;

    REAL vm;
    REAL h,s,l;

    h = 0; // default to black
    s = 0;
    l = 0;
    max = MAX(r,g);
    max = MAX(max,b);
    min = MIN(r,g);
    min = MIN(min,b);
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











//Still needed?

 void findColorRGB(unsigned char* RGBimage,int width, int height,unsigned char R, unsigned char G, unsigned char B, int threshold){
    int i = 0;
    int j = 0;
    int consPix = 0;
    int x1 = width, x2 = 0, y1 = height, y2 = 0;
    for( i = 0; i < height; i++){
       consPix = 0;
       for ( j = 0; j < width; j++){
	 if(sqrt( (REAL)(pow((REAL)( (unsigned char) RGBimage[(i * width * 3) + (j*3) + 0]) - R , 2 ) ) +
                (REAL)(pow((REAL)( (unsigned char) RGBimage[(i * width * 3) + (j*3) + 1]) - G , 2 ) ) +
                (REAL)(pow((REAL)( (unsigned char) RGBimage[(i * width * 3) + (j*3) + 2]) - B , 2 ) ) 
		  ) < (REAL)threshold ){
                
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



#ifdef DRAW_BOUNDING_BOX 
void drawBoundingbox(unsigned char * RGBimage, int width, int height, int x1, int y1, int x2, int y2, int thickness ,
 unsigned char R, unsigned char G, unsigned char B){
 int i = 0;
 int j = 0;
      //Top Line
        if(y1 > thickness){
            i = y1 - thickness;
        }else{
            i = 0;
        }
        for(;i<=y1;i++){
            for(j = x1; j <= x2; j++){
                RGBimage[(i * width * 3) + (j*3) + 0] = R;
                RGBimage[(i * width * 3) + (j*3) + 1] = G;
                RGBimage[(i * width * 3) + (j*3) + 2] = B;             
            }
        }
        
        //Bottom Line
        if(height - y2 > thickness){
            i = y2 + thickness;
        }else{
            i = y2 + (height - y2);
        }
        for(;i>=y2;i--){
            for(j = x1; j < x2; j++){
                RGBimage[(i * width * 3) + (j*3) + 0] = R;
                RGBimage[(i * width * 3) + (j*3) + 1] = G;
                RGBimage[(i * width * 3) + (j*3) + 2] = B;             
            }
        }
        
        //Right Line
        if(width - x2 > thickness){
            j = x2 + thickness;
        }else{
            j = width;
        }
        for(;j>=x2;j--){
            for(i=y1 ;i < y2; i++){
                RGBimage[(i * width * 3) + (j*3) + 0] = R;
                RGBimage[(i * width * 3) + (j*3) + 1] = G;
                RGBimage[(i * width * 3) + (j*3) + 2] = B;             
            }
        }
        
        //Left Line
        if(x1 > thickness){
            j = x1 - thickness;
        }else{
            j = 0;
        }
        for(;j<=x1;j++){
            for(i=y1 ;i < y2; i++){
                RGBimage[(i * width * 3) + (j*3) + 0] = R;
                RGBimage[(i * width * 3) + (j*3) + 1] = G;
                RGBimage[(i * width * 3) + (j*3) + 2] = B;             
            }
        }
}
#endif

