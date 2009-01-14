/*
 *  V4L1 video capture example
 */
/**
 * This file is part of the OpenVIDIA project at http://openvidia.sf.net
 * Copyright (C) 2004, James Fung
 *
 * OpenVIDIA is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * OpenVIDIA is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with OpenVIDIA; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 * based on V4L2.cc which is in its turn based on  Video4linux2 capture.c example code from
 * http://v4l2spec.bytesex.org/spec/capture-example.html
 *
 * This is for capture devices which only support video4linux 1
 * In particular the philips webcams are supported
 * Interface calls are identical to dc1394
 * extra functions are getCaptureWidth() en getCaptureHeight()
 *	If  V4L1 is compiled with #define HAVE_PWC_DRIVER
 * then you get 352x288 resolution and a yuv420->rgb32 conversion is done in V4l.
 * Otherwise you get 640x480 and it expects the capturesource to have native RGB32 support
 * If you need other capture sizes or conversions, just change the code.
 * The conversion routines should probably be moved to conversions.cc
 **/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <getopt.h>		/* getopt_long() */

#include <fcntl.h>		/* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <iostream>
#include <fstream>
//#define HAVE_PWC_DRIVER
#include <openvidia/openvidia_lnx.h>

using namespace std;
void
V4L1::TellRWMHeCanUseImage (int numBufs, const char *dma_bufs_[])
{
    unsigned char *dma_buf_ = (unsigned char *) dma_bufs_[0];
    lock ();
    memcpy ((unsigned char *) ptr (), dma_buf_, width () * height () * 3);
    unlock ();
    post ();
}

void
V4L1::tellThreadDoneWithBuffer ()
{
    if (bufferUsed != 0)
    {
    }
    bufferUsed = 0;
}

V4L1::~V4L1 ()
{
}

#ifdef HAVE_PWC_DRIVER
/*
   functions ripped from xawtv source (v3.56): libng/color_yuv2rgb.c
   and slightly modified
*/

/* name in xawtv source: ng_color_yuv2rgb_init() */
void
yuv2rgb_init (void)
{
    int i;

    /* init Lookup tables */
    for (i = 0; i < 256; i++)
    {
        ng_yuv_gray[i] = i * LUN_MUL >> 8;
        ng_yuv_red[i] = (RED_ADD + i * RED_MUL) >> 8;
        ug_yuv_blue[i] = (BLUE_ADD + i * BLUE_MUL) >> 8;
        ng_yuv_g1[i] = (GREEN1_ADD + i * GREEN1_MUL) >> 8;
        ng_yuv_g2[i] = (GREEN2_ADD + i * GREEN2_MUL) >> 8;
    }
    for (i = 0; i < CLIP; i++)
        ng_clip[i] = 0;
    for (; i < CLIP + 256; i++)
        ng_clip[i] = i - CLIP;
    for (; i < 2 * CLIP + 256; i++)
        ng_clip[i] = 255;
}

/* name in xawtv source: yuv420p_to_rgb24() */
static void
yuv2rgb (char *out_addr, char *in_addr, int rowstride, int width, int height)
{
    unsigned char *y, *u, *v;
    unsigned char *us, *vs;
    unsigned char *dp, *d;
    int i, j, gray, padbytesy, padbytesuv;
    padbytesy = 0;
    padbytesuv = 0;
    dp = (unsigned char *) out_addr;
    y = (unsigned char *) in_addr;
    u = y + width * height;
    v = u + width * height / 4;

    for (i = 0; i < height; i++)
    {
        d = dp;
        us = u;
        vs = v;
        for (j = 0; j < width; j += 2)
        {
            gray = GRAY (*y);
            *(d++) = RED (gray, *v);
            *(d++) = GREEN (gray, *v, *u);
            *(d++) = BLUE (gray, *u);
            *(d++) = 255;
            y++;
            gray = GRAY (*y);
            *(d++) = RED (gray, *v);
            *(d++) = GREEN (gray, *v, *u);
            *(d++) = BLUE (gray, *u);
            *(d++) = 255;
            y++;
            u++;
            v++;
        }
        y += padbytesy;
        u += padbytesuv;
        v += padbytesuv;
        if (0 == (i % 2))
        {
            u = us;
            v = vs;
        }
        dp += rowstride;
    }
}
static void
yuv2rgb24 (char *out_addr, char *in_addr, int rowstride, int width,
           int height)
{
    unsigned char *y, *u, *v;
    unsigned char *us, *vs;
    unsigned char *dp, *d;
    int i, j, gray;

    dp = (unsigned char *) out_addr;
    y = (unsigned char *) in_addr;
    u = y + width * height;
    v = u + width * height / 4;

    for (i = 0; i < height; i++)
    {
        d = dp;
        us = u;
        vs = v;
        for (j = 0; j < width; j += 2)
        {
            gray = GRAY (*y);
            *(d++) = RED (gray, *v);
            *(d++) = GREEN (gray, *v, *u);
            *(d++) = BLUE (gray, *u);
            y++;
            gray = GRAY (*y);
            *(d++) = RED (gray, *v);
            *(d++) = GREEN (gray, *v, *u);
            *(d++) = BLUE (gray, *u);
            y++;
            u++;
            v++;
        }
        if (0 == (i % 2))
        {
            u = us;
            v = vs;
        }
        dp += rowstride;
    }
}
#endif
void
V4L1::set_controls ()
{

    struct video_picture picture;
    //struct v4l2_control control;

    memset (&picture, 0, sizeof (picture));
    //queryctrl.id = V4L1_CID_BRIGHTNESS;

    if (-1 == ioctl (fd, VIDIOCGPICT, &picture))
    {
        if (errno != EINVAL)
        {
            perror ("VIDIOCGPICT");
            exit ();
        }
        else
        {
            printf ("VIDIOCGPICT is not supported\n");
        }
    }
    else
    {
        picture.brightness = 65535 / 2;	//default value ?
        //picture.brightness -= 3000;
        cerr << "[V4L1] Brightness : " << picture.brightness << endl;
        picture.colour = 65535 / 2;
        //picture.colour += 24000;
        cerr << "[V4L1] Saturation : " << picture.colour << endl;
        picture.contrast = 65535 / 2;
        //picture.contrast += 20000;
        cerr << "[V4L1] Contrast : " << picture.contrast << endl;
        picture.hue = 65535 / 2;
        //picture.hue = 65535 ; // increase hue helps in low light environment
        //picture.hue -= 4000;
        cerr << "[V4L1] Hue : " << picture.hue << endl;
        //picture.palette=format;
        cerr << "[V4L1] Palette : " << picture.palette << endl;
        cerr << "[V4L1]    --> ";
        switch (picture.palette)
        {
        case VIDEO_PALETTE_RGB24:
            cerr << "RGB24 Format" << endl;
            break;
        case VIDEO_PALETTE_YUV420:
            cerr << "YUV420 Format" << endl;
            break;
        default:
            cerr << "See /usr/include/linux/videodev.h for palette type " <<
                 picture.palette << endl;
            break;
        }
        format = picture.palette;
        //picture.depth=24;//TODO check if this is neccesary
        cerr << "[V4L1] Depth : " << picture.depth << endl;
        if (-1 == ioctl (fd, VIDIOCSPICT, &picture))
        {
            perror ("VIDIOCSPICT");
            exit ();
        }

    }

}


#define CLEAR(x) memset (&(x), 0, sizeof (x))
static void
errno_exit (const char *s)
{
    fprintf (stderr, "%s error %d, %s\n", s, errno, strerror (errno));

    // (EXIT_FAILURE);
}

static int
xioctl (int fd, int request, void *arg)
{
    int r;

    do
        r = ioctl (fd, request, arg);
    while (-1 == r && EINTR == errno);

    return r;
}


static void
write_image (const void *p)
{
    FILE *FD = fopen ("output.ppm", "w");
    fprintf (FD, "P6\n320 240\n255\n");
    fwrite (p, 352 * 288, 3, FD);
    fclose (FD);
    //(0);
}

static void
process_image (const void *p)
{
    fputc ('.', stdout);
    write_image (p);
    fflush (stdout);
}

char *
V4L1::read_frame (void)
{
    //struct v4l2_buffer buf;
    struct video_mmap videoMMap;
    unsigned int next_buffernum;

    switch (io)
    {
    case IO_METHOD_READ:
        if (-1 == read (fd, buffers[0].start, buffers[0].length))
        {
            switch (errno)
            {
            case EAGAIN:
                return 0;

            case EIO:
                /* Could ignore EIO, see spec. */

                /* fall through */

            default:
                errno_exit ("read");
            }
        }
        cerr << "Read complete";
        //process_image (buffers[0].start);
        return (char *) buffers[0].start;

        break;

    case IO_METHOD_MMAP:
        int capnum;
        CLEAR (videoMMap);
        next_buffernum = ((int) current_buffernum - 1 < 0) ? n_buffers - 1 : current_buffernum - 1;
        videoMMap.width = CaptureWidth;
        videoMMap.height = CaptureHeight;
        videoMMap.format = format;
        videoMMap.frame = next_buffernum;
        assert (next_buffernum < n_buffers && next_buffernum >= 0);
        /*printf(" videoMMap.width %i\n", videoMMap.width);
           printf(" videoMMap.height %i\n", videoMMap.height);
           printf(" videoMMap.format %i\n", videoMMap.format);
           printf("  videoMMap.frame %i\n",  videoMMap.frame);
           printf("  n_buffers %i\n", n_buffers); */

        //initiate capture of next frame
        if (ioctl (fd, VIDIOCMCAPTURE, &videoMMap) < 0)
        {
            // return (errno);
            switch (errno)
            {
            case EAGAIN:
                return 0;

            case EIO:
                /* Could ignore EIO, see spec. */

                /* fall through */

            default:
                printf ("2\n");
                errno_exit ("VIDIOCMCAPTURE");
            }
        }			// end if
        assert (current_buffernum < n_buffers);
        //Now wait for current frame to be finished
        if (ioctl (fd, VIDIOCSYNC, &current_buffernum) < 0)
        {
            switch (errno)
            {
            case EAGAIN:
                return 0;

            case EIO:
                /* Could ignore EIO, see spec. */

                /* fall through */

            default:
                errno_exit ("VIDIOCSYNC");
            }
        }			// end if
        capnum = current_buffernum;
        current_buffernum++;
        if (current_buffernum >= n_buffers)
            current_buffernum = 0;
        //return (char *)buffers[0].start;
        return (char *) buffers[capnum].start;
        break;
    case IO_METHOD_USERPTR:
        //Not implemented for V4L1
        break;
    }

    return NULL;
}

/*
void V4L1::
mainloop                        (void)
*/
void
V4L1::run ()
{
    char *newimgbuf;
    while (1)
    {
        for (;;)
        {
            if (newimgbuf = read_frame ())
                break;

            /* EAGAIN - continue select loop. */
        }
        bufferUsed = 1;
#ifdef HAVE_PWC_DRIVER
        yuv2rgb (newimgbuf_rgb,
                 newimgbuf, CaptureWidth * 4, CaptureWidth, CaptureHeight);
        newimgbuf = newimgbuf_rgb;
#endif

        bufs[0] = newimgbuf;
        TellRWMHeCanUseImage (1, (const char **) bufs);

        //TellRWMHeCanUseImage((const char *)newimgbuf);


    }
}

void
V4L1::stop_capturing (void)
{
    //enum v4l2_buf_type type;

    switch (io)
    {
    case IO_METHOD_READ:
        /* Nothing to do. */
        break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
        /*type = V4L1_BUF_TYPE_VIDEO_CAPTURE;

           if (-1 == xioctl (fd, VIDIOC_STREAMOFF, &type))
           errno_exit ("VIDIOC_STREAMOFF"); */

        break;
    }
    delete[]newimgbuf_rgb;
}

void
V4L1::start_capturing (void)
{

    switch (io)
    {
    case IO_METHOD_READ:
        /* Nothing to do. */
        break;

    case IO_METHOD_MMAP:
        struct video_mmap videoMMap;
        CLEAR (videoMMap);
        videoMMap.width = CaptureWidth;
        videoMMap.height = CaptureHeight;
        videoMMap.format = format;

        /* Queue all available buffers for capture */
        for (current_buffernum = 0; current_buffernum < v4l1_mbuf.frames - 1; current_buffernum++)	//v4l1_mbuf.frames-1 because last frame capture will be initiated in loop
        {
            videoMMap.frame = current_buffernum;
            if (ioctl (fd, VIDIOCMCAPTURE, &videoMMap) < 0)
            {
                printf ("1\n");
                errno_exit ("VIDIOCMCAPTURE");
                //goto bail;
            }
            else
            {
                printf ("1.1 frame %i went ok\n", current_buffernum);
            }
        }
        current_buffernum = 0;
        break;

    case IO_METHOD_USERPTR:
        //not implemented for v4l1
        break;
    }
#ifdef HAVE_PWC_DRIVER
    newimgbuf_rgb = (char *) calloc (CaptureWidth * CaptureHeight * 4, sizeof (char));	//TODO should be released somehow
#endif
}

void
V4L1::uninit_device (void)
{

    switch (io)
    {
    case IO_METHOD_READ:
        free (buffers[0].start);
        break;

    case IO_METHOD_MMAP:
        if (-1 == munmap (buffers[0].start, buffers[0].length))
            errno_exit ("munmap");
        break;

    case IO_METHOD_USERPTR:
        //for (i = 0; i < n_buffers; ++i)
        //      free (buffers[i].start);
        break;
    }

    free (buffers);
}

void
V4L1::init_read (unsigned int buffer_size)
{
    buffers = (struct buffer *) calloc (1, sizeof (*buffers));

    if (!buffers)
    {
        fprintf (stderr, "Out of memory\n");
        // (EXIT_FAILURE);
    }

    buffers[0].length = buffer_size;
    buffers[0].start = malloc (buffer_size);

    if (!buffers[0].start)
    {
        fprintf (stderr, "Out of memory\n");
        // (EXIT_FAILURE);
    }
}

void
V4L1::init_mmap (void)
{
    buffers = (struct buffer *) calloc (8, sizeof (*buffers));	//one big buffer containing all the frames


    if (!buffers)
    {
        fprintf (stderr, "Out of memory\n");
        // (EXIT_FAILURE);
    }
    /* Query the actual buffers available */
    CLEAR (v4l1_mbuf);
    if (ioctl (fd, VIDIOCGMBUF, &v4l1_mbuf) < 0)
    {
        if (EINVAL == errno)
        {
            // (EXIT_FAILURE);
            fprintf (stderr, "%s does not support " "memory mapping\n",
                     dev_name);
        }
        else
        {
            errno_exit ("VIDIOCGMBUF");
        }
        //goto bail;
    }

    cerr << "mmap info" << endl;
    cerr << "mmap buffer: size=" << v4l1_mbuf.size << "frames=" << v4l1_mbuf.
         frames << endl;
    for (int i = 0; i < v4l1_mbuf.frames; i++)
    {
        printf ("frame %d: offset %d\n", i, v4l1_mbuf.offsets[i]);
    }

    n_buffers = v4l1_mbuf.frames;
    if (n_buffers < 2)
    {
        fprintf (stderr, "Insufficient buffer memory on %s\n", dev_name);
        // (EXIT_FAILURE);
    }

    buffers[0].length = v4l1_mbuf.size;
    buffers[0].start = mmap (NULL /* start anywhere */ ,
                             v4l1_mbuf.size,
                             PROT_READ | PROT_WRITE /* required */ ,
                             MAP_SHARED /* recommended */ ,
                             fd, 0);

    if (MAP_FAILED == buffers[0].start)
        errno_exit ("mmap");
    for (int i = 0; i < v4l1_mbuf.frames; i++)
    {
#if defined (__x86_64__)
        buffers[i].start = (unsigned char *)
                           ((unsigned long) buffers[0].start +
                            (unsigned long) v4l1_mbuf.offsets[i]);
#else
        buffers[i].start = (unsigned char *)
                           ((unsigned int) buffers[0].start +
                            (unsigned int) v4l1_mbuf.offsets[i]);
#endif
    }

}


void
V4L1::init_userp (unsigned int buffer_size)
{
    //not implemented for v4l1
}

void
V4L1::init_device (void)
{
    struct video_capability cap;
    //struct v4l2_cropcap cropcap;
    //struct v4l2_crop crop;
    //struct v4l2_format fmt;
    //unsigned int min;

    if (-1 == xioctl (fd, VIDIOCGCAP, &cap))
    {
        if (EINVAL == errno)
        {
            fprintf (stderr, "%s is no V4L1 device\n", dev_name);
            // (EXIT_FAILURE);
        }
        else
        {
            errno_exit ("VIDIOCGCAP");
        }
    }

    if (!(cap.type & VID_TYPE_CAPTURE))
    {
        fprintf (stderr, "%s is no video capture device\n", dev_name);
        // (EXIT_FAILURE);
    }

    switch (io)
    {
    case IO_METHOD_READ:
        /*
        		if (!(cap.capabilities & V4L1_CAP_READWRITE)) {
        			fprintf (stderr, "%s does not support read i/o\n",
        				 dev_name);
        			// (EXIT_FAILURE);
        		}
        */

        break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
        /*if (!(cap.capabilities & V4L1_CAP_STREAMING)) {
           fprintf (stderr, "%s does not support streaming i/o\n",
           dev_name);
           // (EXIT_FAILURE);
           } */

        break;
    }

    /* Select video input, video standard and tune here. */

    /*
       cropcap.type = V4L1_BUF_TYPE_VIDEO_CAPTURE;

       if (-1 == xioctl (fd, VIDIOC_CROPCAP, &cropcap)) {
       // Errors ignored.
       }

       crop.type = V4L1_BUF_TYPE_VIDEO_CAPTURE;
       crop.c = cropcap.defrect; // reset to default

       if (-1 == xioctl (fd, VIDIOC_S_CROP, &crop)) {
       switch (errno) {
       case EINVAL:
       // Cropping not supported.
       break;
       default:
       // Errors ignored.
       break;
       }
       }
     */

    //fmt.type                = V4L1_BUF_TYPE_VIDEO_CAPTURE;

    //fmt.fmt.pix.width       = 640;
    //fmt.fmt.pix.height      = 480;
    ////fmt.fmt.pix.pixelformat = V4L1_PIX_FMT_YUYV;
    ////fmt.fmt.pix.width       = 320;
    ////fmt.fmt.pix.height      = 240;
    //fmt.fmt.pix.pixelformat = V4L1_PIX_FMT_RGB32;
    //fmt.fmt.pix.field       = V4L1_FIELD_INTERLACED;

#ifdef HAVE_PWC_DRIVER
    CaptureWidth = 352;		//maximum width my philips webcam can do
    CaptureHeight = 288;		//maximum height my philips webcam can do
    format = VIDEO_PALETTE_YUV420P;
    yuv2rgb_init ();
    fprintf (stderr, "[V4L1] Requesting %ix%i", CaptureWidth, CaptureHeight);
    fprintf (stderr, " YUV420P\n");
#else
    CaptureWidth = 640;
    CaptureHeight = 480;
    format = VIDEO_PALETTE_RGB32;	//VIDEO_PALETTE_RGB24;
    fprintf (stderr, "[V4L1] Requesting %ix%i", CaptureWidth, CaptureHeight);
    fprintf (stderr, " RGB32 Interlaced\n");
#endif

    //cerr<<"[V4L1] Requesting %ix%i RGB32 Interlaced"<<endl;
    //if (-1 == xioctl (fd, VIDIOC_S_FMT, &fmt)) errno_exit ("VIDIOC_S_FMT");
    /* Note VIDIOC_S_FMT may change width and height. */
    struct video_window vwin;
    CLEAR (vwin);
    if (-1 == xioctl (fd, VIDIOCGWIN, &vwin))
        errno_exit ("VIDIOCGWIN");
    fprintf (stderr, "original video_window.width %i\n", vwin.width);
    fprintf (stderr, "original video_window.height %i\n", vwin.height);
    //  vwin.width=CaptureWidth;
    //  vwin.height=CaptureHeight;
#ifdef HAVE_PWC_DRIVER
#define PWC_FPS_SHIFT		16
#define PWC_FPS_MASK		0x00FF0000
    fprintf (stderr, "original  framerate %i\n",
             (vwin.flags & PWC_FPS_MASK) >> PWC_FPS_SHIFT);
    //vwin.flags &= ~PWC_FPS_FRMASK;
    //vwin.flags |= (15 << PWC_FPS_SHIFT);//try to set framerate to 30
    for (int fps = 30; fps >= 0; fps /= 2)
    {
        vwin.flags &= ~PWC_FPS_MASK;
        vwin.flags |= (fps << PWC_FPS_SHIFT);	//retry without framerate request;
        if (xioctl (fd, VIDIOCSWIN, &vwin) != -1)
            break;
    }
#endif
    if (-1 == xioctl (fd, VIDIOCSWIN, &vwin))
        errno_exit ("VIDIOCSWIN");

    if (-1 == xioctl (fd, VIDIOCGWIN, &vwin))
        errno_exit ("VIDIOCGWIN");
    fprintf (stderr, "new video_window.width %i\n", vwin.width);
    fprintf (stderr, "new video_window.height %i\n", vwin.height);
#ifdef HAVE_PWC_DRIVER
    fprintf (stderr, "new framerate %i\n",
             (vwin.flags & PWC_FPS_MASK) >> PWC_FPS_SHIFT);
#endif
    CaptureWidth = vwin.width;	//maybe changed by driver
    CaptureHeight = vwin.height;	//maybe changed by driver
    resize (vwin.width, vwin.height, 4);
    /* Buggy driver paranoia. */
    /*min = fmt.fmt.pix.width * 2;
       if (fmt.fmt.pix.bytesperline < min)
       fmt.fmt.pix.bytesperline = min;
       min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
       if (fmt.fmt.pix.sizeimage < min)
       fmt.fmt.pix.sizeimage = min; */

    switch (io)
    {
    case IO_METHOD_READ:
        init_read (CaptureWidth * CaptureHeight);
        break;

    case IO_METHOD_MMAP:
        init_mmap ();
        break;

    case IO_METHOD_USERPTR:
        init_userp (CaptureWidth * CaptureHeight);
        break;
    }

    /* v4l1 choose input. This is commented out so you can set a different input with the v4lctl program before starting this program.
       struct video_channel v4lchannel;
       CLEAR(v4lchannel);
       v4lchannel.channel = 0;
       if (ioctl(fd, VIDIOCGCHAN, &v4lchannel) < 0) {
       perror ("VIDIOCGCHAN");
       } // end if
       v4lchannel.norm = VIDEO_MODE_NTSC;//VIDEO_MODE_PAL,VIDEO_MODE_SECAM
       if (ioctl(fd, VIDIOCSCHAN, &v4lchanne) < 0) {
       perror ("VIDIOCSCHAN");
       } // end if
     */


}

void
V4L1::close_device (void)
{
    /*
            if (-1 == close (fd))
    	        errno_exit ("close");

            fd = -1;
    */
//can't figure out how to use close in c++ with the threading
}

void
V4L1::open_device (void)
{
    struct stat st;

    if (-1 == stat (dev_name, &st))
    {
        fprintf (stderr, "Cannot identify '%s': %d, %s\n",
                 dev_name, errno, strerror (errno));
        // (EXIT_FAILURE);
    }

    if (!S_ISCHR (st.st_mode))
    {
        fprintf (stderr, "%s is no device\n", dev_name);
        // (EXIT_FAILURE);
    }

    fd = open (dev_name, O_RDWR /* required */  | O_NONBLOCK, 0);

    if (-1 == fd)
    {
        fprintf (stderr, "Cannot open '%s': %d, %s\n",
                 dev_name, errno, strerror (errno));
        // (EXIT_FAILURE);
    }
}

int
V4L1::getCaptureWidth ()
{
    return CaptureWidth;
}

int
V4L1::getCaptureHeight ()
{
    return CaptureHeight;
}

int
V4L1::getCaptureDepth ()
{
    return 32;
}
static void
usage (FILE * fp, int argc, char **argv)
{
    fprintf (fp,
             "Usage: %s [options]\n\n"
             "Options:\n"
             "-d | --device name   Video device name [/dev/video]\n"
             "-h | --help          Print this message\n"
             "-m | --mmap          Use memory mapped buffers\n"
             "-r | --read          Use read() calls\n"
             "-u | --userp         Use application allocated buffers\n"
             "", argv[0]);
}

static const char short_options[] = "d:hmru";

static const struct option long_options[] = {
    {"device", required_argument, NULL, 'd'},
    {"help", no_argument, NULL, 'h'},
    {"mmap", no_argument, NULL, 'm'},
    {"read", no_argument, NULL, 'r'},
    {"userp", no_argument, NULL, 'u'},
    {0, 0, 0, 0}
};

/*
int
main                            (int                    argc,
                                 char **                argv)
*/
V4L1::V4L1 (int W, int H): LBuffer (W, H, 5)
{
    dev_name = NULL;
    io = IO_METHOD_MMAP;
    fd = -1;
    buffers = NULL;
    n_buffers = 0;

    dev_name = "/dev/video0";	// having the 0 seems to be the
    //new 2.6 kernel
    // default instead of /dev/video
    /*
            for (;;) {
                    int index;
                    int c;
                    c = getopt_long (argc, argv,
                                     short_options, long_options,
                                     &index);
                    if (-1 == c)
                            break;
                    switch (c) {
                    case 0:
                            break;
                    case 'd':
                            dev_name = optarg;
                            break;
                    case 'h':
                            usage (stdout, argc, argv);
                            // (EXIT_SUCCESS);
                    case 'm':
                            io = IO_METHOD_MMAP;
    			break;
                    case 'r':
                            io = IO_METHOD_READ;
    			break;
                    case 'u':
                            io = IO_METHOD_USERPTR;
    			break;
                    default:
                            usage (stderr, argc, argv);
                            // (EXIT_FAILURE);
                    }
            }
    */
    open_device ();
    init_device ();
    set_controls ();
    start_capturing ();
    /*
            mainloop ();
            stop_capturing ();
            uninit_device ();
            close_device ();
    */
    // (EXIT_SUCCESS);
}
