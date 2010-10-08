#ifndef _UAV_READWRITEJPEG__H_
#define _UAV_READWRITEJPEG__H_

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <jpeglib.h>
#include "utility.h"

void jpeg_wr_file(const char *filename, int quality,
        const uint8_t *buffer, int width, int height);
    
int jpeg_rd_file(const char *filename, uint8_t **buffer,
        int *width, int *height);

void jpeg_wr_mem(uint8_t **stream_out, unsigned long *length,
        int quality, const uint8_t *rgb_in, int width, int height);
    
int jpeg_rd_mem(const uint8_t *stream, unsigned long length,
        uint8_t **rgb_out, int *width, int *height);
    
void my_error_exit(j_common_ptr cinfo);

#endif // _UAV_READWRITEJPEG__H_

