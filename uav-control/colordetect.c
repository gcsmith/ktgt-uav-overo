#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "jpeglib.h"
#include "readwritejpeg.h"
#include "colordetect.h"
JSAMPLE * image_buffer;

int noiseFilter = 5;

/*
int main(int argc, char *argv[]){
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
    char* filename = "frisbee.jpg";
    
   

    if( argc!= 1 && argc != 2 && argc != 5 && argc != 8 ){
        printf( "Usage:\n %s filename R G B\n or \n %s filename \
                R G B Ht St Lt\n *t -> threshold value\n",argv[0],argv[0]);
        return 0;
    }
    //Define File name (first argument 
    if(argc >= 2){
        filename = argv[1];
    }
    // Define RGB
    if( argc >= 5){
        R = (unsigned char)atoi(argv[2]);
        G = (unsigned char)atoi(argv[3]);
        B = (unsigned char)atoi(argv[4]);
    }
    if( argc == 8){    
    // Define a new thereshold
        Ht = (short)atoi(argv[5]);
        St = (short)atoi(argv[6]);
        Lt = (short)atoi(argv[7]);
    
    }
    
    
    
    runColorDetectionFile(filename,"testimage.jpg",25,R,B,G,Ht,St,Lt);
   
    read_JPEG_file(filename,&RGBimage,&image_width,&image_height);

    

    runColorDetection(RGBimage,image_width,image_height, R, B, G, Ht, St, Lt);

    
    // Write the Image back
    write_JPEG_file ("testimage.jpg", 100,RGBimage,image_width,image_height);
    

    printf("End Test\n");
    return 0;
}
*/

void runColorDetectionFile(char * infilename,char * outfilename, int quality,
                        unsigned char R,
                        unsigned char G, 
                        unsigned char B, 
                        short Ht, 
                        short St, 
                        short Lt){
                        
    int width = 0;
    int height = 0;
    //Define RGBimage
    unsigned char * RGBimage;
    
                        
    read_JPEG_file(infilename,&RGBimage,&width,&height);

    

    runColorDetection(RGBimage,width,height, R, B, G, Ht, St, Lt);

    
    // Write the Image back
    //write_JPEG_file(outfilename,quality,RGBimage,width,height);                   
                        
}






void runColorDetectionMemory(unsigned char * inbuffer, unsigned long * insize, int quality,
                        unsigned char R,
                        unsigned char G, 
                        unsigned char B, 
                        short Ht, 
                        short St, 
                        short Lt){
                        
    int width = 0;
    int height = 0;
    //Define RGBimage
    unsigned char * RGBimage;
    
                        
    read_JPEG_stream(inbuffer,(*insize),&RGBimage,&width,&height);

    

    runColorDetection(RGBimage,width,height, R, B, G, Ht, St, Lt);

    
    // Write the Image back
    write_JPEG_stream (&inbuffer, insize,quality,RGBimage,width,height);                   
                        
}




void runColorDetection(unsigned char * RGBimage,int image_width, int image_height,
                        unsigned char R,
                        unsigned char G, 
                        unsigned char B, 
                        short Ht, 
                        short St, 
                        short Lt){
     //Color to look for in HSL (will be calculated from RGB)
    short H = 0;
    short S = 0;
    short L = 0;                   
   
    int RecolorPixels = 1;
    int DrawBox = 1;
   //Convert detect color from RGB to HSL
    RGB2HSL (R, G, B,&H, &S, &L);                     
                        
    // Convert the image to HSL
    short * HSLimage = malloc(sizeof(short)*image_height*image_width*3);	  

    COLORimageRGBtoHSL(RGBimage, HSLimage, image_width, image_height);
    // Find HSL Values in image
    findColorHSL(HSLimage,image_width,image_height, H, S, L, Ht, St, Lt,RGBimage, RecolorPixels, DrawBox);                    
                        
}



void findColorHSL(short* HSLimage, int width, int height,
                short H, short S, short L,
                int Hthreshold, int Sthreshold, int Lthreshold,
                unsigned char * RGBimage, int recolorPixels, int drawBox){
    int i = 0;
    int j = 0;
    int consPix = 0;
    int x1 = width, x2 = 0, y1 = height, y2 = 0;
    //int box = 0;
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
                      
                    if(recolorPixels == 1){
                        RGBimage[(i * width * 3) + (j*3) + 0] = Rnoise;
                        RGBimage[(i * width * 3) + (j*3) + 1] = Gnoise;
                        RGBimage[(i * width * 3) + (j*3) + 2] = Bnoise;                    
                    }                                        
                }else if(recolorPixels == 1){
                    RGBimage[(i * width * 3) + (j*3) + 0] = Rvalid;
                    RGBimage[(i * width * 3) + (j*3) + 1] = Gvalid;
                    RGBimage[(i * width * 3) + (j*3) + 2] = Bvalid;                    
                }   
            }else{
                consPix = 0;
            }
        }
    }
    if(drawBox == 1){
        drawBoundingbox(RGBimage,width,height, x1, y1, x2, y2, thickness, Rbox, Gbox, Bbox);
  
    }
    printf("HSL Bounding box: (%d,%d) (%d,%d)\n",x1,y1,x2,y2);
}
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



