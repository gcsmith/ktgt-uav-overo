// -----------------------------------------------------------------------------
// File:    readwritejpeg.h
// Authors: Tyler Thierolf, Timothy Miller, Garrett Smith
// Created: 10-01-2010
//
// Routines for reading / writing jpeg images from files or memory streams.
// -----------------------------------------------------------------------------

#ifndef _UAV_READWRITEJPEG__H_
#define _UAV_READWRITEJPEG__H_

#include <stdint.h>

// compress the given buffer and write out into the specified file
int jpeg_wr_file(const char *filename, int quality,
        const uint8_t *buffer, int width, int height);
    
// decompress the specified file and store into a buffer
int jpeg_rd_file(const char *filename, uint8_t **buffer,
        int *width, int *height);

// compress the given buffer and write out into the specified memory stream
int jpeg_wr_mem(uint8_t **stream_out, unsigned long *length,
        int quality, const uint8_t *rgb_in, int width, int height);

// decompress the specified memory stream and store into a buffer
int jpeg_rd_mem(const uint8_t *stream, unsigned long length,
        uint8_t **rgb_out, int *width, int *height);
    
#endif // _UAV_READWRITEJPEG__H_

