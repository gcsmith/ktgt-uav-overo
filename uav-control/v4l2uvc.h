/*******************************************************************************
# Linuc-UVC streaming input-plugin for MJPG-streamer                           #
#                                                                              #
# This package work with the Logitech UVC based webcams with the mjpeg feature #
#                                                                              #
# Copyright (C) 2005 2006 Laurent Pinchart &&  Michel Xhaard                   #
#                    2007 Lucas van Staden                                     #
#                    2007 Tom Stöveken                                         #
#                                                                              #
# This program is free software; you can redistribute it and/or modify         #
# it under the terms of the GNU General Public License as published by         #
# the Free Software Foundation; either version 2 of the License, or            #
# (at your option) any later version.                                          #
#                                                                              #
# This program is distributed in the hope that it will be useful,              #
# but WITHOUT ANY WARRANTY; without even the implied warranty of               #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                #
# GNU General Public License for more details.                                 #
#                                                                              #
# You should have received a copy of the GNU General Public License            #
# along with this program; if not, write to the Free Software                  #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA    #
#                                                                              #
# Adapted from the original mjpg-streamer sources for use in uav_control.      #
*******************************************************************************/

#ifndef _UAV_V4L2UVC__H_
#define _UAV_V4L2UVC__H_

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <linux/videodev.h>
#include <libv4l2.h>
#define NB_BUFFER 4
#define IOCTL_VIDEO(fd, req, value) v4l2_ioctl(fd, req, value)
#define OPEN_VIDEO(fd, flags) v4l2_open(fd, flags)
#define CLOSE_VIDEO(fd) v4l2_close(fd)

typedef enum _streaming_state streaming_state;
enum _streaming_state {
    STREAMING_OFF = 0,
    STREAMING_ON = 1,
    STREAMING_PAUSED = 2,
};

typedef struct uvc_globals {
    int enabled;
    int size;
    int width, height, fps;
    unsigned char *buf;
    pthread_mutex_t db;
    pthread_cond_t  db_update;
} uvc_globals_t;

struct vdIn {
    int fd;
    char *videodevice;
    char *status;
    char *pictName;
    struct v4l2_capability cap;
    struct v4l2_format fmt;
    struct v4l2_buffer buf;
    struct v4l2_requestbuffers rb;
    void *mem[NB_BUFFER];
    unsigned char *tmpbuffer;
    unsigned char *framebuffer;
    streaming_state streamingState;
    int grabmethod;
    int width;
    int height;
    int fps;
    int formatIn;
    int formatOut;
    int framesizeIn;
    int signalquit;
    int toggleAvi;
    int getPict;
    int rawFrameCapture;
    /* raw frame capture */
    unsigned int fileCounter;
    /* raw frame stream capture */
    unsigned int rfsFramesWritten;
    unsigned int rfsBytesWritten;
    /* raw stream capture */
    FILE *captureFile;
    unsigned int framesWritten;
    unsigned int bytesWritten;
    int framecount;
    int recordstart;
    int recordtime;
};

int v4l2DeviceOpen(struct vdIn *vd, char *device, int width, int height,
        int fps, int format, int grabmethod, uvc_globals_t *pglobal);
int v4l2DeviceClose(struct vdIn *vd);
int v4l2CopyBuffer(unsigned char *out, unsigned char *buf, int size);
int v4l2GrabFrame(struct vdIn *vd);
int v4l2EnumControls(struct vdIn *vd);
int v4l2QueryControl(struct vdIn *vd, int control, struct v4l2_queryctrl *qc);
int v4l2GetControl(struct vdIn *vd, int control);
int v4l2SetControl(struct vdIn *vd, int control, int value);
int v4l2ResetControl(struct vdIn *vd, int control);
int v4l2SetResolution(struct vdIn *vd, int width, int height);

#endif // _UAV_V4L2UVC__H_

