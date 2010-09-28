
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <jpeglib.h>


void write_JPEG_file (char * filename, int quality,unsigned char *RGBimage,int width,int height);
//void write_JPEG_file (char * filename, int quality);
void my_error_exit (j_common_ptr cinfo);
int read_JPEG_file (char * filename,unsigned char ** RGBimage,int * width, int * height);

