// DRAW_BOUNDING_BOX  0
// WRITE_IMAGE  0
typedef struct {
    unsigned char R;
    unsigned char G; 
    unsigned char B; 
    
    unsigned short Ht;
    unsigned short St; 
    unsigned short Lt;
    
    char quality;
} colorToFind;


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

void runColorDetectionFile(const char * infilename, const char * outfilename,
    colorToFind * color, boxCoordinates * box);

void runColorDetectionMemory(unsigned char * inbuffer, unsigned long * insize,
    colorToFind * color, boxCoordinates * box);
                        
void runColorDetection(unsigned char * RGBimage,
    colorToFind * color, boxCoordinates * box);


void findColorRGB(unsigned char* RGBimage, int width, int height,
    unsigned char R, unsigned char G, unsigned char B, int threshold);
    
int findColorHSL(short* HSLimage,
                short H, short S, short L,
                colorToFind * color,
                boxCoordinates * box,
                unsigned char * RGBimage);
                
#ifdef DRAW_BOUNDING_BOX              
void drawBoundingbox(unsigned char * RGBimage,
    boxCoordinates * box, int thickness,
    unsigned char R, unsigned char G, unsigned char B);                
#endif                

void COLORimageRGBtoHSL(unsigned char* COLORimage, short* HSLimage,
    int width, int height);

void RGB2HSL (unsigned char R_in, unsigned char G_in, unsigned char B_in,
              short * h_out, short * s_out, short * l_out);

