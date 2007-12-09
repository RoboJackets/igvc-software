/*******************************************************************************
#               uvcview: Sdl video Usb Video Class grabber           .         #
#This package work with the Logitech UVC based webcams with the mjpeg feature. #
#All the decoding is in user space with the embedded jpeg decoder              #
#.                                                                             #
#               Copyright (C) 2005 2006 Laurent Pinchart &&  Michel Xhaard     #
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

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <linux/videodev2.h>

#define NB_BUFFER 2
#define DHT_SIZE 432

#define V4L2_CID_BACKLIGHT_COMPENSATION       (V4L2_CID_PRIVATE_BASE+0)
#define V4L2_CID_POWER_LINE_FREQUENCY       (V4L2_CID_PRIVATE_BASE+1)
#define V4L2_CID_SHARPNESS              (V4L2_CID_PRIVATE_BASE+2)
#define V4L2_CID_HUE_AUTO              (V4L2_CID_PRIVATE_BASE+3)
#define V4L2_CID_FOCUS_AUTO              (V4L2_CID_PRIVATE_BASE+4)
#define V4L2_CID_FOCUS_ABSOLUTE              (V4L2_CID_PRIVATE_BASE+5)
#define V4L2_CID_FOCUS_RELATIVE              (V4L2_CID_PRIVATE_BASE+6)

#define V4L2_CID_PANTILT_RELATIVE       (V4L2_CID_PRIVATE_BASE+7)
#define V4L2_CID_PANTILT_RESET              (V4L2_CID_PRIVATE_BASE+8)

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
    int isstreaming;
    int grabmethod;
    int width;
    int height;
    int formatIn;
    int formatOut;
    int framesizeIn;
    int signalquit;
    int toggleAvi;
    int getPict;

};

int
init_videoIn(struct vdIn *vd, char *device, int width, int height,
            int format, int grabmethod);
int uvcGrab(struct vdIn *vd);
int close_v4l2(struct vdIn *vd);

int v4l2GetControl(struct vdIn *vd, int control);
int v4l2SetControl(struct vdIn *vd, int control, int value);
int v4l2UpControl(struct vdIn *vd, int control);
int v4l2DownControl(struct vdIn *vd, int control);
int v4l2ToggleControl(struct vdIn *vd, int control);
int v4l2ResetControl(struct vdIn *vd, int control);
int v4l2ResetPanTilt(struct vdIn *vd,int pantilt);
int v4L2UpDownPan(struct vdIn *vd, short inc);
int v4L2UpDownTilt(struct vdIn *vd,short inc);
