#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <jpeglib.h>
#include <getopt.h>
#include <syslog.h>
#include <errno.h>
#include <assert.h>
#include <string.h>

#include "readwritejpeg.h"
#include "colordetect.h"
#include "fp32.h"

#define MAX_PATH 256
#define PUT_BANNER(c) for (i = 0; i < c; ++i) printf("-"); printf("\n");

// -----------------------------------------------------------------------------
// Return the difference from (t1 - t0) in microseconds.
unsigned long compute_delta(struct timespec *t0, struct timespec *t1)
{
    return (t1->tv_nsec / 1000) - (t0->tv_nsec / 1000) +
           (t1->tv_sec - t0->tv_sec) * 1000000;
}

// -----------------------------------------------------------------------------
// Benchmark jpeg decompression from memory stream.
float test_decompress(int iterations, const uint8_t *stream_in,
        unsigned long length)
{
    struct timespec t0, t1;
    track_coords_t box = { 0 };
    unsigned long j;
    uint8_t *rgb_buff = NULL;
    uint8_t img_in[length];

    clock_gettime(CLOCK_REALTIME, &t0);
    for (j = 0; j < iterations; j++) {
        // simply decompress the jpeg stream each iteration
        memcpy(img_in, stream_in, length);
        jpeg_rd_mem(img_in, length, &rgb_buff, &box.width, &box.height);
        printf("."); fflush(stdout);
    }
    clock_gettime(CLOCK_REALTIME, &t1);
    return iterations / ((double)compute_delta(&t0, &t1) / 1000000.0);
}

// -----------------------------------------------------------------------------
// Benchmark jpeg decompression and hsl color tracking from memory stream.
float test_hsl_stream(int iterations, const uint8_t *stream_in,
        unsigned long length, track_color_t *color)
{
    struct timespec t0, t1;
    track_coords_t box = { 0 };
    unsigned long j;
    uint8_t *rgb_buff = NULL;
    uint8_t img_in[length];
    track_color_t temp;

    clock_gettime(CLOCK_REALTIME, &t0);
    for (j = 0; j < iterations; j++) {
        memcpy(&temp, color, sizeof(track_color_t));
        memcpy(img_in, stream_in, length);
        jpeg_rd_mem(img_in, length, &rgb_buff, &box.width, &box.height);
        colordetect_hsl(rgb_buff, &temp, &box);
        printf(box.detected ? "." : "?"); fflush(stdout);
    }
    clock_gettime(CLOCK_REALTIME, &t1);
    return iterations / ((double)compute_delta(&t0, &t1) / 1000000.0);
}

// -----------------------------------------------------------------------------
// Benchmark jpeg decompression and hsl color tracking from memory stream.
float test_hsl_fp32_stream(int iterations, const uint8_t *stream_in,
        unsigned long length, track_color_t *color)
{
    struct timespec t0, t1;
    track_coords_t box = { 0 };
    unsigned long j;
    uint8_t *rgb_buff = NULL;
    uint8_t img_in[length];
    track_color_t temp;

    clock_gettime(CLOCK_REALTIME, &t0);
    for (j = 0; j < iterations; j++) {
        memcpy(&temp, color, sizeof(track_color_t));
        memcpy(img_in, stream_in, length);
        jpeg_rd_mem(img_in, length, &rgb_buff, &box.width, &box.height);
        colordetect_hsl_fp32(rgb_buff, &temp, &box);
        printf(box.detected ? "." : "?"); fflush(stdout);
    }
    clock_gettime(CLOCK_REALTIME, &t1);
    return iterations / ((double)compute_delta(&t0, &t1) / 1000000.0);
}

// -----------------------------------------------------------------------------
// Benchmark jpeg decompression and rgb color tracking from memory stream.
float test_rgb1_stream(int iterations, const uint8_t *stream_in,
        unsigned long length, track_color_t *color)
{
    struct timespec t0, t1;
    track_coords_t box = { 0 };
    unsigned long j;
    uint8_t *rgb_buff = NULL;
    uint8_t img_in[length];

    color->ht = 10;
    color->st = 10;
    color->lt = 10;

    clock_gettime(CLOCK_REALTIME, &t0);
    for (j = 0; j < iterations; j++) {
        memcpy(img_in, stream_in, length);
        jpeg_rd_mem(img_in, length, &rgb_buff, &box.width, &box.height);
        colordetect_rgb(rgb_buff, color, &box);
        printf(box.detected ? "." : "?"); fflush(stdout);
    }
    clock_gettime(CLOCK_REALTIME, &t1);
    return iterations / ((double)compute_delta(&t0, &t1) / 1000000.0);
}

// -----------------------------------------------------------------------------
// Benchmark jpeg decompression and rgb color tracking from memory stream.
float test_rgb2_stream(int iterations, const uint8_t *stream_in,
        unsigned long length, track_color_t *color)
{
    struct timespec t0, t1;
    track_coords_t box = { 0 };
    unsigned long j;
    uint8_t *rgb_buff = NULL;
    uint8_t img_in[length];

    clock_gettime(CLOCK_REALTIME, &t0);
    for (j = 0; j < iterations; j++) {
        memcpy(img_in, stream_in, length);
        jpeg_rd_mem(img_in, length, &rgb_buff, &box.width, &box.height);
        colordetect_rgb_dist(rgb_buff, (real_t)15, color, &box);
        printf(box.detected ? "." : "?"); fflush(stdout);
    }
    clock_gettime(CLOCK_REALTIME, &t1);
    return iterations / ((double)compute_delta(&t0, &t1) / 1000000.0);
}

// -----------------------------------------------------------------------------
// Information about command line arguments
void print_usage()
{
    printf("usage: jpeg_test [options]\n\n"
           "Program options:\n"
           "  -f [ --file ] arg       : specify a path to an image file\n"
           "  -r [ --rgb ] arg        : specify an rgb color value\n"
           "  -t [ --thresh ] arg     : specify a set of HSL thresholds\n"
           "  -h [ --help ]           : display this usage message\n"
           "  --no-decompress         : do not run the decompression benchmark\n"
           "  --no-hsl                : do not run the hsl benchmark\n"
           "  --no-rgb1               : do not run the rgb1 benchmark\n"
           "  --no-rgb2               : do not run the rgb2 benchmark\n");
}

// -----------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    int flag_decompress = 0, flag_hsl = 0, flag_rgb1 = 0, flag_rgb2 = 0;
    int opt, index;
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
    char filename[MAX_PATH] = "./Data/frisbee.jpg";

    struct option long_options[] = {
        { "file",           required_argument, NULL,            'f' },
        { "rgb",            required_argument, NULL,            'r' },
        { "thresh",         required_argument, NULL,            't' },
        { "no-decompress",  no_argument,       &flag_decompress, 1 },
        { "no-hsl",         no_argument,       &flag_hsl,        1 },
        { "no-rgb1",        no_argument,       &flag_rgb1,       1 },
        { "no-rgb2",        no_argument,       &flag_rgb2,       1 },
        { 0, 0, 0, 0 }
    };

    static const char *str = "f:r:t:h?";

    while (-1 != (opt = getopt_long(argc, argv, str, long_options, &index))) {
        switch (opt) {
        case 'f':
            strncpy(filename, optarg, MAX_PATH);
            break;
        case 'r':
            sscanf(optarg, "%c,%c,%c", &color.r, &color.g, &color.b);
            break;
        case 't':
            sscanf(optarg, "%c,%c,%c", &color.ht, &color.st, &color.lt);
            break;
        case 'h': // fall through
        case '?':
            print_usage();
            exit(EXIT_SUCCESS);
        case 0:
            break;
        default:
            syslog(LOG_ERR, "unexpected argument '%c'\n", opt);
            assert(!"unhandled case in option handling -- this is an error");
            break;
        }
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

    PUT_BANNER(80);

    if (!flag_decompress) {
        printf("Decompress loop      : ");
        printf(" %f FPS\n", test_decompress(40, stream_in, img_size));
    }

    if (!flag_hsl) {
        printf("HSL stream loop      : ");
        printf(" %f FPS\n", test_hsl_stream(40, stream_in, img_size, &color));
    }

    if (!flag_hsl) {
        printf("HSL fp32 stream loop : ");
        printf(" %f FPS\n", test_hsl_fp32_stream(40, stream_in, img_size, &color));
    }

    if (!flag_rgb1) {
        printf("RGB1 stream loop     : ");
        printf(" %f FPS\n", test_rgb1_stream(40, stream_in, img_size, &color));
    }

    if (!flag_rgb2) {
        printf("RGB2 stream loop     : ");
        printf(" %f FPS\n", test_rgb2_stream(40, stream_in, img_size, &color));
    }

    PUT_BANNER(80);
    return 0;
}

