
void findColorRGB(unsigned char* RGBimage, int width, int height,unsigned char R, unsigned char G, unsigned char B, int threshold);
void findColorHSL(short* HSLimage, int width, int height,
                short H, short S, short L,
                int Hthreshold, int Sthreshold, int Lthreshold,
                unsigned char * RGBimage, int recolorPixels, int drawBox);
void hsv_to_hsl(double h, double s, double v, double* hh, double* ss, double* ll);


void COLORimageRGBtoHSL(unsigned char* COLORimage, short* HSLimage, int width, int height);

void RGB2HSL (unsigned char R_in, unsigned char G_in, unsigned char B_in,
              short * h_out, short * s_out, short * l_out);


double max_double(double x, double y);
double min_double(double x, double y);


void printCOLORimageHSLshort(int height, int width, short * COLORimage);
void printCOLORimage(int height, int width, unsigned char * COLORimage);
void printBWimage(int height, int width, unsigned char * BWimage);
void findBoundingCoordinates(int height, int width, unsigned char * BWimage);

