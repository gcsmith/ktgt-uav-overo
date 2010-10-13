/*******************************************************************************
# Linux-UVC streaming input-plugin for MJPG-streamer                           #
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
*******************************************************************************/

#include <stdlib.h>
#include "v4l2uvc.h"
#include "huffman.h"
#if 0
#include "dynctrl.h"
#endif

static int debug = 0;
static int init_v4l2(struct vdIn *vd);

int init_videoIn(struct vdIn *vd, char *device, int width,
                 int height, int fps, int format, int grabmethod,
                 uvc_globals_t *pglobal)
{
    if (vd == NULL || device == NULL)
        return -1;
    if (width == 0 || height == 0)
        return -1;
    if (grabmethod < 0 || grabmethod > 1)
        grabmethod = 1;		//mmap by default;
    vd->videodevice = NULL;
    vd->status = NULL;
    vd->pictName = NULL;
    vd->videodevice = (char *) calloc (1, 16 * sizeof (char));
    vd->status = (char *) calloc (1, 100 * sizeof (char));
    vd->pictName = (char *) calloc (1, 80 * sizeof (char));
    snprintf (vd->videodevice, 12, "%s", device);
    vd->toggleAvi = 0;
    vd->getPict = 0;
    vd->signalquit = 1;
    vd->width = width;
    vd->height = height;
    vd->fps = fps;
    vd->formatIn = format;
    vd->grabmethod = grabmethod;
    if (init_v4l2 (vd) < 0) {
        fprintf (stderr, " Init v4L2 failed !! exit fatal \n");
        goto error;;
    }
#if 0
    struct v4l2_queryctrl ctrl;
#endif

#if 0
    pglobal->in.parametercount = 0;
#endif

#if 0
    /* Try the extended control API first */
    #ifdef V4L2_CTRL_FLAG_NEXT_CTRL
    ctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL;
    if(0 == ioctl(vd->fd, VIDIOC_QUERYCTRL, &ctrl)) {
        do {
            control_readed(vd, &ctrl, pglobal);
            ctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
        } while(0 == ioctl (vd->fd, VIDIOC_QUERYCTRL, &ctrl));
    } else
    #endif
    {
        /* Fall back on the standard API */
        /* Check all the standard controls */
        int i;
        for(i = V4L2_CID_BASE; i<V4L2_CID_LASTP1; i++) {
            ctrl.id = i;
            if(ioctl(vd->fd, VIDIOC_QUERYCTRL, &ctrl) == 0) {
                control_readed(vd, &ctrl, pglobal);
            }
        }

        /* Check any custom controls */
        for(i=V4L2_CID_PRIVATE_BASE; ; i++) {
            ctrl.id = i;
            if(ioctl(vd->fd, VIDIOC_QUERYCTRL, &ctrl) == 0) {
                control_readed(vd, &ctrl, pglobal);
            } else {
                break;
            }
        }
    }
#endif

  /* alloc a temp buffer to reconstruct the pict */
  vd->framesizeIn = (vd->width * vd->height << 1);
  switch (vd->formatIn) {
  case V4L2_PIX_FMT_MJPEG:
    vd->tmpbuffer = (unsigned char *) calloc(1, (size_t) vd->framesizeIn);
    if (!vd->tmpbuffer)
      goto error;
    vd->framebuffer =
        (unsigned char *) calloc(1, (size_t) vd->width * (vd->height + 8) * 2);
    break;
  case V4L2_PIX_FMT_YUYV:
    vd->framebuffer =
        (unsigned char *) calloc(1, (size_t) vd->framesizeIn);
    break;
  default:
    fprintf(stderr, " should never arrive exit fatal !!\n");
    goto error;
    break;

  }

  if (!vd->framebuffer)
    goto error;
  return 0;
error:
#if 0
  free(pglobal->in.in_parameters);
#endif
  free(vd->videodevice);
  free(vd->status);
  free(vd->pictName);
  close(vd->fd);
  return -1;
}

static int init_v4l2(struct vdIn *vd)
{
  int i;
  int ret = 0;

  if ((vd->fd = open(vd->videodevice, O_RDWR)) == -1) {
    perror("ERROR opening V4L interface");
    return -1;
  }

  memset(&vd->cap, 0, sizeof(struct v4l2_capability));
  ret = ioctl(vd->fd, VIDIOC_QUERYCAP, &vd->cap);
  if (ret < 0) {
    fprintf(stderr, "Error opening device %s: unable to query device.\n", vd->videodevice);
    goto fatal;
  }

  if ((vd->cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == 0) {
    fprintf(stderr, "Error opening device %s: video capture not supported.\n",
           vd->videodevice);
    goto fatal;;
  }

  if (vd->grabmethod) {
    if (!(vd->cap.capabilities & V4L2_CAP_STREAMING)) {
      fprintf(stderr, "%s does not support streaming i/o\n", vd->videodevice);
      goto fatal;
    }
  } else {
    if (!(vd->cap.capabilities & V4L2_CAP_READWRITE)) {
      fprintf(stderr, "%s does not support read i/o\n", vd->videodevice);
      goto fatal;
    }
  }

  /*
   * set format in
   */
  memset(&vd->fmt, 0, sizeof(struct v4l2_format));
  vd->fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  vd->fmt.fmt.pix.width = vd->width;
  vd->fmt.fmt.pix.height = vd->height;
  vd->fmt.fmt.pix.pixelformat = vd->formatIn;
  vd->fmt.fmt.pix.field = V4L2_FIELD_ANY;
  ret = ioctl(vd->fd, VIDIOC_S_FMT, &vd->fmt);
  if (ret < 0) {
    fprintf(stderr, "Unable to set format: %d res: %dx%d", vd->formatIn, vd->width, vd->height);
    goto fatal;
  }

  if ((vd->fmt.fmt.pix.width != vd->width) ||
      (vd->fmt.fmt.pix.height != vd->height)) {
    fprintf(stderr, "i: The format asked unavailable, so the  width %d height %d \n", vd->fmt.fmt.pix.width, vd->fmt.fmt.pix.height);
    vd->width = vd->fmt.fmt.pix.width;
    vd->height = vd->fmt.fmt.pix.height;
    /*
     * look the format is not part of the deal ???
     */
    if (vd->formatIn != vd->fmt.fmt.pix.pixelformat) {
        if (vd->formatIn == V4L2_PIX_FMT_MJPEG) {
            fprintf(stderr, "The inpout device does not supports MJPEG mode\nYou may also try the YUV mode (-yuv option), but it requireses a lot of CPU power\n");
            goto fatal;
        } else if (vd->formatIn == V4L2_PIX_FMT_YUYV) {
            fprintf(stderr, "The input device does not supports YUV mode\n");
            goto fatal;
        }
    } else {
        vd->formatIn = vd->fmt.fmt.pix.pixelformat;
    }
  }

  /*
   * set framerate
   */
  struct v4l2_streamparm *setfps;
  setfps = (struct v4l2_streamparm *) calloc(1, sizeof(struct v4l2_streamparm));
  memset(setfps, 0, sizeof(struct v4l2_streamparm));
  setfps->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  setfps->parm.capture.timeperframe.numerator = 1;
  setfps->parm.capture.timeperframe.denominator = vd->fps;
  ret = ioctl(vd->fd, VIDIOC_S_PARM, setfps);

  /*
   * request buffers
   */
  memset(&vd->rb, 0, sizeof(struct v4l2_requestbuffers));
  vd->rb.count = NB_BUFFER;
  vd->rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  vd->rb.memory = V4L2_MEMORY_MMAP;

  ret = ioctl(vd->fd, VIDIOC_REQBUFS, &vd->rb);
  if (ret < 0) {
    perror("Unable to allocate buffers");
    goto fatal;
  }

  /*
   * map the buffers
   */
  for (i = 0; i < NB_BUFFER; i++) {
    memset(&vd->buf, 0, sizeof(struct v4l2_buffer));
    vd->buf.index = i;
    vd->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    vd->buf.memory = V4L2_MEMORY_MMAP;
    ret = ioctl(vd->fd, VIDIOC_QUERYBUF, &vd->buf);
    if (ret < 0) {
      perror("Unable to query buffer");
      goto fatal;
    }

    if (debug)
      fprintf(stderr, "length: %u offset: %u\n", vd->buf.length, vd->buf.m.offset);

    vd->mem[i] = mmap(0 /* start anywhere */ ,
                      vd->buf.length, PROT_READ, MAP_SHARED, vd->fd,
                      vd->buf.m.offset);
    if (vd->mem[i] == MAP_FAILED) {
      perror("Unable to map buffer");
      goto fatal;
    }
    if (debug)
      fprintf(stderr, "Buffer mapped at address %p.\n", vd->mem[i]);
  }

  /*
   * Queue the buffers.
   */
  for (i = 0; i < NB_BUFFER; ++i) {
    memset(&vd->buf, 0, sizeof(struct v4l2_buffer));
    vd->buf.index = i;
    vd->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    vd->buf.memory = V4L2_MEMORY_MMAP;
    ret = ioctl(vd->fd, VIDIOC_QBUF, &vd->buf);
    if (ret < 0) {
      perror("Unable to queue buffer");
      goto fatal;;
    }
  }
  return 0;
fatal:
  return -1;

}

static int video_enable(struct vdIn *vd)
{
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  int ret;

  ret = ioctl(vd->fd, VIDIOC_STREAMON, &type);
  if (ret < 0) {
    perror("Unable to start capture");
    return ret;
  }
  vd->isstreaming = 1;
  return 0;
}

static int video_disable(struct vdIn *vd)
{
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  int ret;

  ret = ioctl(vd->fd, VIDIOC_STREAMOFF, &type);
  if (ret < 0) {
    perror("Unable to stop capture");
    return ret;
  }
  vd->isstreaming = 0;
  return 0;
}

/******************************************************************************
Description.:
Input Value.:
Return Value:
******************************************************************************/
int is_huffman(unsigned char *buf)
{
  unsigned char *ptbuf;
  int i = 0;
  ptbuf = buf;
  while (((ptbuf[0] << 8) | ptbuf[1]) != 0xffda) {
    if (i++ > 2048)
      return 0;
    if (((ptbuf[0] << 8) | ptbuf[1]) == 0xffc4)
      return 1;
    ptbuf++;
  }
  return 0;
}

/******************************************************************************
Description.:
Input Value.:
Return Value:
******************************************************************************/
int memcpy_picture(unsigned char *out, unsigned char *buf, int size)
{
  unsigned char *ptdeb, *ptlimit, *ptcur = buf;
  int sizein, pos=0;

  if (!is_huffman(buf)) {
    ptdeb = ptcur = buf;
    ptlimit = buf + size;
    while ((((ptcur[0] << 8) | ptcur[1]) != 0xffc0) && (ptcur < ptlimit))
      ptcur++;
    if (ptcur >= ptlimit)
        return pos;
    sizein = ptcur - ptdeb;

    memcpy(out+pos, buf, sizein); pos += sizein;
    memcpy(out+pos, dht_data, sizeof(dht_data)); pos += sizeof(dht_data);
    memcpy(out+pos, ptcur, size - sizein); pos += size-sizein;
  } else {
    memcpy(out+pos, ptcur, size); pos += size;
  }
  return pos;
}

int uvcGrab(struct vdIn *vd)
{
#define HEADERFRAME1 0xaf
  int ret;

  if (!vd->isstreaming)
    if (video_enable(vd))
      goto err;

  memset(&vd->buf, 0, sizeof(struct v4l2_buffer));
  vd->buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  vd->buf.memory = V4L2_MEMORY_MMAP;

  ret = ioctl(vd->fd, VIDIOC_DQBUF, &vd->buf);
  if (ret < 0) {
    perror("Unable to dequeue buffer");
    goto err;
  }

  switch (vd->formatIn) {
    case V4L2_PIX_FMT_MJPEG:
      if (vd->buf.bytesused <= HEADERFRAME1) {    /* Prevent crash
                                                  * on empty image */
        fprintf(stderr, "Ignoring empty buffer ...\n");
        return 0;
      }

      /* memcpy(vd->tmpbuffer, vd->mem[vd->buf.index], vd->buf.bytesused);

      memcpy (vd->tmpbuffer, vd->mem[vd->buf.index], HEADERFRAME1);
      memcpy (vd->tmpbuffer + HEADERFRAME1, dht_data, sizeof(dht_data));
      memcpy (vd->tmpbuffer + HEADERFRAME1 + sizeof(dht_data), vd->mem[vd->buf.index] + HEADERFRAME1, (vd->buf.bytesused - HEADERFRAME1));
      */

      memcpy(vd->tmpbuffer, vd->mem[vd->buf.index], vd->buf.bytesused);

      if (debug)
        fprintf(stderr, "bytes in used %d \n", vd->buf.bytesused);
      break;

    case V4L2_PIX_FMT_YUYV:
      if (vd->buf.bytesused > vd->framesizeIn)
        memcpy (vd->framebuffer, vd->mem[vd->buf.index], (size_t) vd->framesizeIn);
      else
        memcpy (vd->framebuffer, vd->mem[vd->buf.index], (size_t) vd->buf.bytesused);
      break;

    default:
      goto err;
    break;
  }

  ret = ioctl(vd->fd, VIDIOC_QBUF, &vd->buf);
  if (ret < 0) {
    perror("Unable to requeue buffer");
    goto err;
  }

  return 0;

err:
  vd->signalquit = 0;
  return -1;
}

int close_v4l2(struct vdIn *vd)
{
  if (vd->isstreaming)
    video_disable(vd);
  if (vd->tmpbuffer)
    free(vd->tmpbuffer);
  vd->tmpbuffer = NULL;
  free(vd->framebuffer);
  vd->framebuffer = NULL;
  free(vd->videodevice);
  free(vd->status);
  free(vd->pictName);
  vd->videodevice = NULL;
  vd->status = NULL;
  vd->pictName = NULL;

  return 0;
}

#if 0
/* return >= 0 ok otherwhise -1 */
static int isv4l2Control(struct vdIn *vd, int control, struct v4l2_queryctrl *queryctrl) {
  int err =0;

  queryctrl->id = control;
  if ((err= ioctl(vd->fd, VIDIOC_QUERYCTRL, queryctrl)) < 0) {
    //fprintf(stderr, "ioctl querycontrol error %d \n",errno);
    return -1;
  }

  if (queryctrl->flags & V4L2_CTRL_FLAG_DISABLED) {
    //fprintf(stderr, "control %s disabled \n", (char *) queryctrl->name);
    return -1;
  }

  if (queryctrl->type & V4L2_CTRL_TYPE_BOOLEAN) {
    return 1;
  }

  if (queryctrl->type & V4L2_CTRL_TYPE_INTEGER) {
    return 0;
  }

  fprintf(stderr, "contol %s unsupported  \n", (char *) queryctrl->name);
  return -1;
}
#endif

#if 0
int v4l2GetControl(struct vdIn *vd, int control) {
  struct v4l2_queryctrl queryctrl;
  struct v4l2_control control_s;
  int err;

  if ((err = isv4l2Control(vd, control, &queryctrl)) < 0) {
    return -1;
  }

  control_s.id = control;
  if ((err = ioctl(vd->fd, VIDIOC_G_CTRL, &control_s)) < 0) {
    return -1;
  }

  return control_s.value;
}
#endif

#if 0
int v4l2SetControl(struct vdIn *vd, int control, int value) {
  struct v4l2_control control_s;
  struct v4l2_queryctrl queryctrl;
  int min, max, step, val_def;
  int err;

  if (isv4l2Control(vd, control, &queryctrl) < 0)
    return -1;

  min = queryctrl.minimum;
  max = queryctrl.maximum;
  step = queryctrl.step;
  val_def = queryctrl.default_value;

  if ((value >= min) && (value <= max)) {
    control_s.id = control;
    control_s.value = value;
    if ((err = ioctl(vd->fd, VIDIOC_S_CTRL, &control_s)) < 0) {
      return -1;
    }
  }

  return 0;
}
#endif

#if 0
int v4l2UpControl(struct vdIn *vd, int control) {
  struct v4l2_control control_s;
  struct v4l2_queryctrl queryctrl;
  int min, max, current, step, val_def;
  int err;

  if (isv4l2Control(vd, control, &queryctrl) < 0)
    return -1;

  min = queryctrl.minimum;
  max = queryctrl.maximum;
  step = queryctrl.step;
  val_def = queryctrl.default_value;
  if ( (current = v4l2GetControl(vd, control)) == -1 )
    return -1;

  current += step;

  //fprintf(stderr, "max %d, min %d, step %d, default %d ,current %d \n",max,min,step,val_def,current);
  if (current <= max) {
    control_s.id = control;
    control_s.value = current;
    if ((err = ioctl(vd->fd, VIDIOC_S_CTRL, &control_s)) < 0) {
      return -1;
    }
    //fprintf(stderr, "Control name:%s set to value:%d\n", queryctrl.name, control_s.value);
  } else {
    //fprintf(stderr, "Control name:%s already has max value:%d \n", queryctrl.name, max);
    return -1;
  }

  return 0;
}
#endif

#if 0
int v4l2DownControl(struct vdIn *vd, int control) {
  struct v4l2_control control_s;
  struct v4l2_queryctrl queryctrl;
  int min, max, current, step, val_def;
  int err;

  if (isv4l2Control(vd, control, &queryctrl) < 0)
    return -1;

  min = queryctrl.minimum;
  max = queryctrl.maximum;
  step = queryctrl.step;
  val_def = queryctrl.default_value;
  if ( (current = v4l2GetControl(vd, control)) == -1 )
    return -1;

  current -= step;
  //fprintf(stderr, "max %d, min %d, step %d, default %d ,current %d \n",max,min,step,val_def,current);

  if (current >= min) {
    control_s.id = control;
    control_s.value = current;
    if ((err = ioctl(vd->fd, VIDIOC_S_CTRL, &control_s)) < 0) {
      return -1;
    }
    //fprintf(stderr, "Control name:%s set to value:%d\n", queryctrl.name, control_s.value);
  }
  else {
    return -1;
    //fprintf(stderr, "Control name:%s already has min value:%d \n", queryctrl.name, min);
  }

  return 0;
}
#endif

#if 0
int v4l2ToggleControl(struct vdIn *vd, int control) {
  struct v4l2_control control_s;
  struct v4l2_queryctrl queryctrl;
  int current;
  int err;

  if (isv4l2Control(vd, control, &queryctrl) != 1)
    return -1;

  if ( (current = v4l2GetControl(vd, control)) == -1 )
    return -1;

  control_s.id = control;
  control_s.value = !current;

  if ((err = ioctl(vd->fd, VIDIOC_S_CTRL, &control_s)) < 0) {
    return -1;
  }

  return 0;
}
#endif

#if 0
int v4l2ResetControl(struct vdIn *vd, int control) {
  struct v4l2_control control_s;
  struct v4l2_queryctrl queryctrl;
  int val_def;
  int err;

  if (isv4l2Control(vd, control, &queryctrl) < 0)
    return -1;

  val_def = queryctrl.default_value;
  control_s.id = control;
  control_s.value = val_def;

  if ((err = ioctl(vd->fd, VIDIOC_S_CTRL, &control_s)) < 0) {
    return -1;
  }

  return 0;
};
#endif

#if 0
void control_readed(struct vdIn *vd,struct v4l2_queryctrl *ctrl, globals *pglobal)
{
    struct v4l2_control c;
    c.id = ctrl->id;
    if(ioctl(vd->fd, VIDIOC_G_CTRL, &c) != -1) {
        pglobal->in.in_parameters =
            (struct input_control*)realloc(
                                           pglobal->in.in_parameters,
                                           (pglobal->in.parametercount + 1) * sizeof(struct input_control));
            memcpy(&pglobal->in.in_parameters[pglobal->in.parametercount].ctrl, ctrl, sizeof(struct v4l2_queryctrl));
            pglobal->in.in_parameters[pglobal->in.parametercount].value = c.value;
            if (ctrl->type == V4L2_CTRL_TYPE_MENU) {
                pglobal->in.in_parameters[pglobal->in.parametercount].menuitems =
                    (struct v4l2_querymenu*)malloc((ctrl->maximum+1) * sizeof(struct v4l2_querymenu));
                int i;
                for(i=ctrl->minimum; i<=ctrl->maximum; i++) {
                    struct v4l2_querymenu qm;
                    qm.id = ctrl->id;
                    qm.index = i;
                    if(ioctl(vd->fd, VIDIOC_QUERYMENU, &qm) == 0) {
                        memcpy(&pglobal->in.in_parameters[pglobal->in.parametercount].menuitems[i], &qm, sizeof(struct v4l2_querymenu));
                        DBG("Menu item found for %d: %s %d %d\n", qm.index, qm.name, ctrl->minimum, ctrl->maximum);
                    } else {
                        DBG("Unable to get menu item for %s, index=%d\n", ctrl->name, qm.index);
                    }
                }
            } else {
                pglobal->in.in_parameters[pglobal->in.parametercount].menuitems = NULL;
            }
            DBG("v4l2 parameter found: %s\ value %d %d th \n", ctrl->name, c.value, pglobal->in.parametercount);
            pglobal->in.parametercount++;
    } else {
        DBG("Unable to get the value of %s", ctrl->name);
    }
};
#endif
