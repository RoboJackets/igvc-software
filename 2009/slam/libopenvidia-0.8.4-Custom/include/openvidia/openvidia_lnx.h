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
 **/
#ifndef _ONV_LBUFFER_H
#define _ONV_LBUFFER_H

#include <cc++/thread.h>
#include <cc++/config.h>
#include <cc++/exception.h>
#include <pthread.h>
#include <semaphore.h>
#include <time.h>
#include <signal.h>
#include <unistd.h>

using namespace ost;
/**\brief LBuffer is basically a malloc'd buffer with an associated mutex locking access
 *
 * LBuffer is basically a malloc'd buffer with an associated
 *   mutex lock that can be checked before/locked during
 *   reads/writes.  It has an "width/height"
 */
class LBuffer : public Mutex, public Thread, public Semaphore {
private:
    int Width, Height, ElementSz;
    void *Ptr;
public:

    ///Initializes a lockable buffer, of size W*H*sz (just like a malloc)
    LBuffer( int W,  ///< Width
             int H,  ///< Height
             int sz  )  ///< Size per element
    {
        Ptr = malloc(sz*W*H);
        Width = (size_t)W;
        Height = (size_t)H;
        ElementSz = (size_t)sz ;
    };

    ///Deletes the buffer object, and frees its associated memory.
    ~LBuffer() {
        free(Ptr);
    };
    /// Lock the buffer.  Any subsequent attempts to lock it while it is
    /// already locked will block.
    void lock() {
        ENTER_CRITICAL;
    }

    /// Unlock the buffer.
    void unlock() {
        LEAVE_CRITICAL;
    }

    /// Return the "width" of the buffer
    size_t width() {
        return (size_t)Width;
    }

    /// Return the "height" of the buffer
    size_t height() {
        return (size_t)Height;
    }

    /// Return the number of bytes of each element
    size_t elmentsz() {
        return (size_t)ElementSz;
    }

    /// Return the size of the buffer (W*H*elementsz)
    size_t size() {
        return (size_t)(ElementSz*Width*Height);
    }

    /// Return a pointer to the associated memory
    void *ptr() {
        return Ptr;
    }

    /// Resize the buffer.
    int resize( int W,  ///< Width
                int H,  ///< Height
                int sz  )  ///< Size per element
    {
        Ptr = realloc( Ptr, sz*W*H );
        Width = (size_t)W;
        Height = (size_t)H;
        ElementSz = (size_t)sz ;
        return (sz*W*H);
    }

};

#endif
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
 **/

#ifndef _DC1394_H
#define _DC1394_H


#include <cc++/thread.h>
//#include <thread.h>
#include <cc++/config.h>
#include <cc++/exception.h>
#include <pthread.h>
#include <semaphore.h>
#include <time.h>
#include <signal.h>
#include <unistd.h>
#include <libraw1394/raw1394.h>
#include <libdc1394/dc1394_control.h>

using namespace std;
using namespace ost;


/** \brief Dc1394 is a class which encapsulates the libdc1394 camera
    handling routines.

Dc1394 is a class which encapsulates the libdc1394 camera handling routines
    It is a "Thread" class (commonc++), which means that after it is
    constructed, it's start() method must be called which starts
    the thread running.  While dc1394 is running, it is continually
    grabbing images from a camera, and placing them in its lockable
    buffer.  When an image is recieved from the
    camera, Dc1394 is locked, updated, and unlocked.  If another
    thread wants to use this image, it should lock, use( ptr() ), then unlock.

    See the LBuffer class for more information.

  Typical usage:
 <pre>
<b>
  //construct a 1394 capture object, using 320x240 resolution.
  // options are 640x480, 320x240, 160x120
  Dc1394 CamSource(320,240);
</b>

  // make a texture
  glGenTextures(1, &tex);              // texture
  glTexEnvi( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE,  GL_REPLACE );
  glBindTexture(GL_TEXTURE_RECTANGLE_NV, tex);
  glTexImage2D(GL_TEXTURE_RECTANGLE_NV, 0, GL_RGBA, width,height, 0,
                GL_RGB, GL_UNSIGNED_BYTE,NULL );
<b>
  //start the camera capture
  CamSource.start();
</b>

   // rendering loop
   while(1) {
<b>
        //wait for a new frame to be captured.
        // this can be removed if you dont mind re-using a previous frame.
        CamSource.wait();
        //lock the image data so it is not updated while we are capturing.
        CamSource.lock();
</b>
        glBindTexture(GL_TEXTURE_RECTANGLE_NV, tex);
        glTexSubImage2D(GL_TEXTURE_RECTANGLE_NV, 0, 0,0, width,height,
                         GL_RGB, GL_UNSIGNED_BYTE,<b>CamSource.ptr()</b>);
<b>
        //free the image data so it can be updated again.
        CamSource.unlock();
</b>

        //use the image...
   }
 </pre>
*/
//class Dc1394 : public Thread, public Mutex, public Semaphore
//class Dc1394 : public Thread, public LBuffer, public Semaphore
class Dc1394 : public LBuffer
{
private:

    void TellRWMHeCanUseImage(int numBufs, const char *dma_bufs_[] );
    const static int DefaultCaptureWidth=320;
    const static int DefaultCaptureHeight=240;
    int CaptureWidth,CaptureHeight;

    dc1394_cameracapture cameras[8];
    raw1394handle_t handles[8]; // selected camera
    nodeid_t *camera_nodes; // all nodes on bus

    int bufferUsed;
    int numCamsToUse;
    int numCamsUsed;
    char *bufs[8] ; //array of adresses of the buffers where the images end up
    bool noDMA;

    bool doOHCI();
//  void lock() { ENTER_CRITICAL; }
//  void unlock() { LEAVE_CRITICAL; }
    void tellThreadDoneWithBuffer();

    void releaseBarrier();

    /// Start Thread.  Camera will begin capturing, writing results to
    /// memory at ptr().  To access image, lock() this object, use the ptr()
    /// data, then unlock() when finished with the buffer.
    void run();
public:
    ///The lockable LBuffer which holds the camera image data.  When accessing it,
    /// remember to: lock(), use(), unlock()
    //LBuffer rgb_buffer;

    ///Initializes the camera capture.
    Dc1394( int W=320, ///< desired capture width, either 320 or 640
            int H=240  ///<desired capture height, either 240 or 480
          );
    ~Dc1394();
};

#endif

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

 * based on Video4linux2 capture.c example code.
 **/

#ifndef _ONV_V4L1_H
#define _ONV_V4L1_H
#include <cc++/thread.h>
//#include <thread.h>
#include <cc++/config.h>
#include <cc++/exception.h>
#include <pthread.h>
#include <semaphore.h>
#include <time.h>
#include <signal.h>
#include <unistd.h>
#include <asm/types.h>          /* for videodev2.h */
#include <linux/videodev.h>
#include <libdc1394/dc1394_control.h>


typedef enum {
    IO_METHOD_READ,
    IO_METHOD_MMAP,
    IO_METHOD_USERPTR,
} io_method;

struct buffer {
    void *                  start;
    size_t                  length;
};
#ifdef HAVE_PWC_DRIVER
#  include "yuv2rgb.h"
#endif
using namespace std;
using namespace ost;

//class V4L1 : public Thread, public Mutex, public VidCap
//class V4L1 : public Thread, public LBuffer, public Semaphore
class V4L1 : public LBuffer
{
private:
    void TellRWMHeCanUseImage(int numBufs, const char *dma_bufs_[] );
    int bufferUsed;
    char *dev_name;
    io_method io;
    int fd;
    struct buffer *buffers;
    struct video_mbuf v4l1_mbuf;
    unsigned int n_buffers;
    unsigned int current_buffernum;
    unsigned	int format;
    char *newimgbuf_rgb;
    char *read_frame(void);
    void mainloop(void);
    void start_capturing(void);
    void stop_capturing(void);
    void uninit_device(void);
    void init_mmap(void);
    void init_read(unsigned int);
    void init_userp(unsigned int);
    void init_device(void);
    void close_device(void);
    void open_device(void);
    void set_controls();


//protected:
    const static int DefaultCaptureWidth=320;
    const static int DefaultCaptureHeight=240;
    int CaptureWidth,CaptureHeight;
    void run();
    void tellThreadDoneWithBuffer();

public:
    V4L1(int W, int H);
    ~V4L1();
    // Start Thread
    //void lock() { ENTER_CRITICAL; }
    //void unlock() { LEAVE_CRITICAL; }
    int getCaptureWidth();
    int getCaptureHeight();
    int getCaptureDepth();
    char *bufs[8];

};

#endif
