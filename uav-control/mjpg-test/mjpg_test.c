#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <linux/videodev.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <signal.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <getopt.h>
#include <pthread.h>
#include <dlfcn.h>
#include <fcntl.h>
#include <syslog.h>

#include "../mjpg_streamer.h"

void sig_handler(int id)
{
    // Stop mjpg-streamer
    mjpg_streamer_stop(0);

    exit(0);
}

int main (int argc, char **argv)
{
    char *input = "input_uvc.so --resolution 640x480 --fps 15 --device /dev/video0";
    char *output = "output_udp.soi --port 2010";

    // register CTRL+C to exit
    signal(SIGINT, sig_handler);

    // Start mjpg-streamer
    mjpg_streamer_start(input, output);

    // Wait for signals
    pause();

    return 0;
}
