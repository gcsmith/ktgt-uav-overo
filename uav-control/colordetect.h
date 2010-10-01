
typedef struct {
    unsigned char R;
    unsigned char G; 
    unsigned char B; 
    
    short Ht;
    short St; 
    short Lt;
    
    char quality;
} colorToFind;

typedef struct {
    unsigned char Rvalid;
    unsigned char Gvalid;
    unsigned char Bvalid;
    
    unsigned char Rnoise;
    unsigned char Gnoise;
    unsigned char Bnoise;
    
    unsigned char Rbox;
    unsigned char Gbox;
    unsigned char Bbox;
} boxColors;

typedef struct {
    int width;
    int height;
    
    int x1;
    int y1;
    
    int x2;
    int y2;
    
    int xc;
    int yc;
    char detected;
} boxCoordinates;

void runColorDetectionFile(char * infilename,char * outfilename, int quality,
                        unsigned char R,
                        unsigned char G, 
                        unsigned char B, 
                        short Ht, 
                        short St, 
                        short Lt);

void runColorDetectionMemory(unsigned char * inbuffer, unsigned long * insize, int quality,
                        unsigned char R,
                        unsigned char G, 
                        unsigned char B, 
                        short Ht, 
                        short St, 
                        short Lt);
                        
void runColorDetection(unsigned char * RGBimage,int image_width, int image_height,
                        unsigned char R,
                        unsigned char G, 
                        unsigned char B, 
                        short Ht, 
                        short St, 
                        short Lt);


void findColorRGB(unsigned char* RGBimage, int width, int height,unsigned char R, unsigned char G, unsigned char B, int threshold);
void findColorHSL(short* HSLimage, int width, int height,
                short H, short S, short L,
                int Hthreshold, int Sthreshold, int Lthreshold,
                unsigned char * RGBimage, int recolorPixels, int drawBox);
                
void drawBoundingbox(unsigned char * RGBimage, int width, int height, int x1, int y1, int x2, int y2, int thickness ,
 unsigned char R, unsigned char G, unsigned char B);                
                



void COLORimageRGBtoHSL(unsigned char* COLORimage, short* HSLimage, int width, int height);

void RGB2HSL (unsigned char R_in, unsigned char G_in, unsigned char B_in,
              short * h_out, short * s_out, short * l_out);

double max_double(double x, double y);
double min_double(double x, double y);





void hsv_to_hsl(double h, double s, double v, double* hh, double* ss, double* ll);
void printCOLORimageHSLshort(int height, int width, short * COLORimage);
void printCOLORimage(int height, int width, unsigned char * COLORimage);
void printBWimage(int height, int width, unsigned char * BWimage);
void findBoundingCoordinates(int height, int width, unsigned char * BWimage);

