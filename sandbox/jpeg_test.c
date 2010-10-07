#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <jpeglib.h>

#include "readwritejpeg.h"
#include "colordetect.h"

unsigned long compute_delta(struct timespec *t0, struct timespec *t1) {
    unsigned long delta = (t1->tv_nsec / 1000) - (t0->tv_nsec / 1000);
    if (t1->tv_sec > t0->tv_sec) {
        // add x second
        delta += 1000000 * (t1->tv_sec - t0->tv_sec);
    }
    return delta;
}

float test_decompress_loop(int iterations, const uint8_t *stream_in,
                           unsigned long length) {
    struct timespec t0, t1;
    track_coords_t box = { 0 };
    unsigned long j;
    uint8_t *rgb_out;

    clock_gettime(CLOCK_REALTIME, &t0);
    for (j = 0; j < iterations; j++) {
        // simply decompress the jpeg stream each iteration
        jpeg_rd_mem(stream_in, length, &rgb_out, &box.width, &box.height);
        printf("."); fflush(stdout);
    }
    clock_gettime(CLOCK_REALTIME, &t1);
    return iterations / ((double)compute_delta(&t0, &t1) / 1000000.0);
}

float test_hsl_stream_loop(int iterations, const uint8_t *stream_in,
                           unsigned long length, track_color_t *color) {
    struct timespec t0, t1;
    track_coords_t box = { 0 };
    unsigned long j;
    uint8_t *rgb_out;

    clock_gettime(CLOCK_REALTIME, &t0);
    for (j = 0; j < iterations; j++) {
        jpeg_rd_mem(stream_in, length, &rgb_out, &box.width, &box.height);
        runColorDetection(rgb_out, color, &box);
        printf(box.detected ? "." : "?"); fflush(stdout);
    }
    clock_gettime(CLOCK_REALTIME, &t1);
    return iterations / ((double)compute_delta(&t0, &t1) / 1000000.0);
}

int main(int argc, char *argv[]) {
    track_color_t color = { 0 };
    unsigned long img_size, bytes_read;
    uint8_t *stream_in;
    FILE *jpeg_fd;

    // default values for rgb/hsl color tracking
    uint8_t track_rgb[] = { 151, 242, 192 };
    uint8_t track_hsl[] = { 21, 71, 255 };

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
        // define RGB color channels
        track_rgb[0] = (uint8_t)atoi(argv[2]);
        track_rgb[1] = (uint8_t)atoi(argv[3]);
        track_rgb[2] = (uint8_t)atoi(argv[4]);
    }

    if (argc == 8) {    
        // define HSL channels
        track_hsl[0] = (short)atoi(argv[5]);
        track_hsl[1] = (short)atoi(argv[6]);
        track_hsl[2] = (short)atoi(argv[7]);
    }
    
    // populate color track structure with specified thresholds
    color.R = track_rgb[0];
    color.G = track_rgb[1];
    color.B = track_rgb[2];
    color.Ht = track_hsl[0];
    color.St = track_hsl[1];
    color.Lt = track_hsl[2];
    color.quality = 25;

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

    // jpeg_rd_file(filename, &rgb_out, &box.width, &box.height);
    
    printf("---------------------------------------------------------------\n");
    printf("Decompress loop : ");
    printf(" %f FPS\n", test_decompress_loop(30, stream_in, img_size));
    printf("HSL stream loop : ");
    printf(" %f FPS\n", test_hsl_stream_loop(30, stream_in, img_size, &color));
    printf("---------------------------------------------------------------\n");

    return 0;
}

