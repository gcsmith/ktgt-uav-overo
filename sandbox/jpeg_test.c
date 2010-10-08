#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <jpeglib.h>

#include "readwritejpeg.h"
#include "colordetect.h"

// -----------------------------------------------------------------------------
// Return the difference from (t1 - t0) in microseconds.
unsigned long compute_delta(struct timespec *t0, struct timespec *t1)
{
    unsigned long delta = (t1->tv_nsec / 1000) - (t0->tv_nsec / 1000);
    if (t1->tv_sec > t0->tv_sec) {
        delta += 1000000 * (t1->tv_sec - t0->tv_sec);
    }
    return delta;
}

// -----------------------------------------------------------------------------
// Benchmark jpeg decompression from memory stream.
float test_decompress_loop(int iterations, const uint8_t *stream_in,
        unsigned long length)
{
    struct timespec t0, t1;
    track_coords_t box = { 0 };
    unsigned long j;
    uint8_t *rgb_buff = NULL;

    clock_gettime(CLOCK_REALTIME, &t0);
    for (j = 0; j < iterations; j++) {
        // simply decompress the jpeg stream each iteration
        jpeg_rd_mem(stream_in, length, &rgb_buff, &box.width, &box.height);
        printf("."); fflush(stdout);
    }
    clock_gettime(CLOCK_REALTIME, &t1);
    return iterations / ((double)compute_delta(&t0, &t1) / 1000000.0);
}

// -----------------------------------------------------------------------------
// Benchmark jpeg decompression and hsl color tracking from memory stream.
float test_hsl_stream_loop(int iterations, const uint8_t *stream_in,
        unsigned long length, track_color_t *color)
{
    struct timespec t0, t1;
    track_coords_t box = { 0 };
    unsigned long j;
    uint8_t *rgb_buff = NULL;
    uint16_t *hsl_buff = NULL;

    clock_gettime(CLOCK_REALTIME, &t0);
    for (j = 0; j < iterations; j++) {
        jpeg_rd_mem(stream_in, length, &rgb_buff, &box.width, &box.height);
        color_detect_hsl(rgb_buff, &hsl_buff, color, &box);
        printf(box.detected ? "." : "?"); fflush(stdout);
    }
    clock_gettime(CLOCK_REALTIME, &t1);
    return iterations / ((double)compute_delta(&t0, &t1) / 1000000.0);
}

// -----------------------------------------------------------------------------
// Benchmark jpeg decompression and rgb color tracking from memory stream.
float test_rgb1_stream_loop(int iterations, const uint8_t *stream_in,
        unsigned long length, track_color_t *color)
{
    struct timespec t0, t1;
    track_coords_t box = { 0 };
    unsigned long j;
    uint8_t *rgb_buff = NULL;

    color->ht = 10;
    color->st = 10;
    color->lt = 10;

    clock_gettime(CLOCK_REALTIME, &t0);
    for (j = 0; j < iterations; j++) {
        jpeg_rd_mem(stream_in, length, &rgb_buff, &box.width, &box.height);
        color_detect_rgb(rgb_buff, color, &box);
        printf(box.detected ? "." : "?"); fflush(stdout);
    }
    clock_gettime(CLOCK_REALTIME, &t1);
    return iterations / ((double)compute_delta(&t0, &t1) / 1000000.0);
}

// -----------------------------------------------------------------------------
// Benchmark jpeg decompression and rgb color tracking from memory stream.
float test_rgb2_stream_loop(int iterations, const uint8_t *stream_in,
        unsigned long length, track_color_t *color)
{
    struct timespec t0, t1;
    track_coords_t box = { 0 };
    unsigned long j;
    uint8_t *rgb_buff = NULL;

    clock_gettime(CLOCK_REALTIME, &t0);
    for (j = 0; j < iterations; j++) {
        jpeg_rd_mem(stream_in, length, &rgb_buff, &box.width, &box.height);
        color_detect_rgb_sqrt(rgb_buff, (real_t)15, color, &box);
        printf(box.detected ? "." : "?"); fflush(stdout);
    }
    clock_gettime(CLOCK_REALTIME, &t1);
    return iterations / ((double)compute_delta(&t0, &t1) / 1000000.0);
}

// -----------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    unsigned long img_size, bytes_read, i;
    uint8_t *stream_in;
    FILE *jpeg_fd;

    // default values for rgb/hsl color tracking
    track_color_t color = {
        151, 242, 192,  // rgb target color
        21, 71, 255,    // hsl thresholds
        5               // filter consecutive pixels
    };

    // default jpeg image file path to load
    const char *filename = "./Data/frisbee.jpg";

    if (argc!= 1 && argc != 2 && argc != 5 && argc != 8) {
        printf("Usage: %s filename R G B\n or \n %s filename"
               "R G B Ht St Lt\n *t -> threshold value\n", argv[0], argv[0]);
        return 0;
    }

    if (argc >= 2) {
        // open user-specified jpeg image
        filename = argv[1];
    }

    if (argc >= 5) {
        // define target rgb color channels
        color.r = (uint8_t)atoi(argv[2]);
        color.g = (uint8_t)atoi(argv[3]);
        color.b = (uint8_t)atoi(argv[4]);
    }

    if (argc == 8) {    
        // define hsl thresholds
        color.ht = (short)atoi(argv[5]);
        color.st = (short)atoi(argv[6]);
        color.lt = (short)atoi(argv[7]);
    }
    
    // open the jpeg image, determine its length in bytes
    jpeg_fd = fopen(filename,"r");
    fseek(jpeg_fd, 0, SEEK_END);
    img_size = ftell(jpeg_fd);
    fseek(jpeg_fd, 0, SEEK_SET);

    // load the compressed image from disk into memory stream
    stream_in = (uint8_t *)malloc(img_size);
    bytes_read = fread(stream_in, 1, img_size, jpeg_fd);
    fclose(jpeg_fd);

    if (bytes_read != img_size) {
        fprintf(stderr, "expected %lu bytes, read %lu\n", img_size, bytes_read);
        return EXIT_FAILURE;
    }
    printf("successfully read %lu bytes from \"%s\"\n", img_size, filename);

    for (i = 0; i < 80; ++i) printf("-"); printf("\n");
    printf("Decompress loop  : ");
    printf(" %f FPS\n", test_decompress_loop(40, stream_in, img_size));
    printf("HSL stream loop  : ");
    printf(" %f FPS\n", test_hsl_stream_loop(40, stream_in, img_size, &color));
    printf("RGB1 stream loop : ");
    printf(" %f FPS\n", test_rgb1_stream_loop(40, stream_in, img_size, &color));
    printf("RGB2 stream loop : ");
    printf(" %f FPS\n", test_rgb2_stream_loop(40, stream_in, img_size, &color));
    for (i = 0; i < 80; ++i) printf("-"); printf("\n");

    return 0;
}

