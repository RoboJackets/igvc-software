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
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <openvidia/openvidia_lnx.h>
#include "conversions.h"

/*maximum number of cards/controllers to be recognized */
#define MAX_PORTS 4

using namespace std;
using namespace ost;
//void TellRWMHeCanUseImage(int numbufs, const char **bufsarray) ;


/* Tell function executed in context of Dc1394 thread */
void Dc1394::TellRWMHeCanUseImage(int numBufs, const char *dma_bufs_[] ) {
    unsigned char *dma_buf_ = (unsigned char *)dma_bufs_[0];
    //write to new frame
    lock();
    if ( CaptureWidth == 640 ) {
        uyyvyy2rgb(dma_buf_, (unsigned char *)ptr() ,CaptureWidth*CaptureHeight);
    } else if ( CaptureWidth == 320 ) {
        uyvy2rgb(dma_buf_, (unsigned char *)ptr() ,CaptureWidth*CaptureHeight);
    } else if ( CaptureWidth == 160 ) {
        uyv2rgb(dma_buf_, (unsigned char *)ptr() ,CaptureWidth*CaptureHeight);
    }
    unlock();
    releaseBarrier();

}

void Dc1394::releaseBarrier()
{
    //while( Semaphore::getValue() <= 0 ) post();
    post();
}



Dc1394 :: ~Dc1394() {

    for ( int i=0 ; i<1 ; i++ ) {
        if (handles[i]!=0) {
            dc1394_release_camera(handles[i],&cameras[i]);
            raw1394_destroy_handle(handles[i]);
        }
        cout << "Destroyed dc1394" << endl;
    }
//this->terminate();
// this->exit();
}

Dc1394 :: Dc1394(int W, int H ):
        LBuffer(W,H,5) {
    bufferUsed=0;
    noDMA = false;
    CaptureWidth = W;
    CaptureHeight = H;
    cout <<"Capturing at "<<CaptureWidth<<"x"<<CaptureHeight<<endl;
    numCamsToUse = 2;
    if ( !doOHCI() ) {
        cerr<<"*"<<endl<<"*"<<endl<<"*"<<endl;
        cerr<<"[Dc1394] Failed to initialize camera capture."<<endl;
        cerr<<"*"<<endl<<"*"<<endl<<"*"<<endl;
    }
    setCancel(cancelDeferred);
}


/*
 * Populate handle, camera_nodes
 */
bool Dc1394 :: doOHCI() {
    int numNodes;
    int numCameras;
    int numPorts = -1 ;

    raw1394handle_t raw_handle;
    struct raw1394_portinfo ports[MAX_PORTS];


    dc1394_camerainfo info;

    /*-----------------------------------------------------------------------
     *   get the number of ports (cards)
     *-----------------------------------------------------------------------*/
    raw_handle = raw1394_new_handle();
    if (raw_handle==NULL) {
        perror("Unable to aquire a raw1394 handle\n");
        perror("did you load the drivers?\n");
        //std::exit(-1);
        return false;
    }

    numPorts = raw1394_get_port_info(raw_handle, ports, numPorts);
    raw1394_destroy_handle(raw_handle);
    fprintf(stderr, "[Dc1394] number of ports (cards) = %d\n", numPorts);


    /*-----------------------------------------------------------------------
      *  get the camera nodes and describe them as we find them
      *-----------------------------------------------------------------------*/
    raw_handle = raw1394_new_handle();
    raw1394_set_port( raw_handle, 0 );

    camera_nodes = dc1394_get_camera_nodes(raw_handle,&numCameras,1);
    raw1394_destroy_handle(raw_handle);

    if (numCameras<1) {
        fprintf( stderr, "no cameras found :(\n");
        raw1394_destroy_handle(handles[0]);
        return false;
    }
    else {
        fprintf(stderr, "[Dc1394] %d cameras found on port.\n", numCameras);
    }



    /*-----------------------------------------------------------------------
     *  Open ohci and asign handle to it
     *-----------------------------------------------------------------------*/

    /* argument to create handle is the port (card) to be used.  defaults
     * to 0 (first) card found for now */

    for ( int i =0 ; i<numCamsToUse && i<numCameras  ; i++ ) {
        cerr<<" i = "<<i<<endl;
        handles[i] = dc1394_create_handle(0);
        if (handles[i]==NULL)
        {
            fprintf( stderr, "Unable to aquire a raw1394 or video1394 handle\n\n"
                     "Please check \n"
                     "  - if the kernel modules `ieee1394',`raw1394' and `ohci1394' are loaded \n"
                     "  - if you have read/write access to /dev/raw1394\n\n");
            cerr<<"killing thread."<<endl;
            //int *killer=0;
            //int a=*killer;
            return false;
        }


        /*-----------------------------------------------------------------------
         *  get the camera nodes and describe them as we find them
         *-----------------------------------------------------------------------*/
        /*
          numNodes = raw1394_get_nodecount(handles[0]);
          camera_nodes = dc1394_get_camera_nodes(handles[0],&numCameras,1);
        */

        printf("working with the first camera on the bus\n");
        dc1394_get_camera_info(handles[i],  camera_nodes[i], &info);
        //if ( !strncmp(info.vendor, "PYRO", 4)  ) {
        //  cerr<<"ads pyro webcam detected, no DMA support so disabling DMA "<<endl;
        //  noDMA = true;
        //}
        //dc1394_print_camera_info( &info );

        /*-----------------------------------------------------------------------
         *  to prevent the iso-transfer bug from raw1394 system, check if
         *  camera is highest node. For details see
         *  http://linux1394.sourceforge.net/faq.html#DCbusmgmt
         *  and
         *  http://sourceforge.net/tracker/index.php?func=detail&aid=435107&group_id=8157&atid=108157
         *-----------------------------------------------------------------------*/
        if ( camera_nodes[i] == numNodes-1)
        {
            fprintf( stderr, "\n"
                     "Sorry, your camera is the highest numbered node\n"
                     "of the bus, and has therefore become the root node.\n"
                     "The root node is responsible for maintaining \n"
                     "the timing of isochronous transactions on the IEEE \n"
                     "1394 bus.  However, if the root node is not cycle master \n"
                     "capable (it doesn't have to be), then isochronous \n"
                     "transactions will not work.  The host controller card is \n"
                     "cycle master capable, however, most cameras are not.\n"
                     "\n"
                     "The quick solution is to add the parameter \n"
                     "attempt_root=1 when loading the OHCI driver as a \n"
                     "module.  So please do (as root):\n"
                     "\n"
                     "   rmmod ohci1394\n"
                     "   insmod ohci1394 attempt_root=1\n"
                     "\n"
                     "for more information see the FAQ at \n"
                     "http://linux1394.sourceforge.net/faq.html#DCbusmgmt\n"
                     "\n");
        }

        /*-----------------------------------------------------------------------
         *  setup capture
         *-----------------------------------------------------------------------*/
        int mode=-1;
        switch (CaptureWidth) {
        case 160:
            mode=MODE_160x120_YUV444;
            cout<<"160x120_YUV444  capture selected"<<endl;
            break;
        case 320:
            mode=MODE_320x240_YUV422;
            cout<<"320x240_YUV422 capture selected"<<endl;
            break;
        case 640:
            mode=MODE_640x480_YUV411;
            //mode=MODE_640x480_RGB;
            cout<<"640x480_YUV411 capture selected"<<endl;
            break;
        default :
            cerr<<"unknown capture height"<<endl;
            _Exit(-1);
        }

        if ( noDMA ) {
            printf("Setting up Raw camera capture.\n");
            if (dc1394_setup_capture(handles[i],
                                     camera_nodes[i], // camera id
                                     //0, // iso channel
                                     i+1, // iso channel
                                     FORMAT_VGA_NONCOMPRESSED, //format
                                     //MODE_320x240_YUV422, //mode
                                     mode, //mode
                                     SPEED_400, //max speed
                                     FRAMERATE_30, //framerate
                                     //FRAMERATE_15, //framerate
                                     &(cameras[i]))!=DC1394_SUCCESS)
            {
                fprintf( stderr,"unable to setup camera for single image cap-\n"
                         "check line %d of %s to make sure\n"
                         "that the video mode,framerate and format are\n"
                         "supported by your camera\n",
                         __LINE__,__FILE__);
                dc1394_release_camera(handles[i],&cameras[i]);
                raw1394_destroy_handle(handles[i]);
                //exit(1);
            }
        }
        else {
            if (dc1394_dma_setup_capture(handles[i],
                                         camera_nodes[i], // camera id
                                         //	       0, // iso channel
                                         i+1,
                                         FORMAT_VGA_NONCOMPRESSED, //format
                                         mode,
                                         //		       			       MODE_320x240_YUV422,
                                         // MODE_640x480_YUV411,
                                         //MODE_640x480_MONO
                                         //MODE_640x480_RGB,
                                         SPEED_400, //max speed
                                         FRAMERATE_30, //framerate
                                         3, //num dma buffers
                                         //0,//dma extra buffers
                                         1, //dropframes
                                         "/dev/video1394/0", //videodevice
                                         &(cameras[i]) )!=DC1394_SUCCESS)
            {
                fprintf( stderr,"unable to setup camera for dma capture\n"
                         "check line %d of %s to make sure\n"
                         "that the video mode,framerate and format are\n"
                         "supported by your camera\n",
                         __LINE__,__FILE__);
                dc1394_release_camera(handles[i],&cameras[i]);
                raw1394_destroy_handle(handles[i]);
            }
            cerr<<"CLOEXEC is "<<fcntl(cameras[i].dma_fd, F_GETFD)<<endl;
            fcntl(cameras[i].dma_fd, F_SETFD, FD_CLOEXEC);
            cerr<<"CLOEXEC is "<<fcntl(cameras[i].dma_fd, F_GETFD)<<endl;
            cerr <<"Dc1394 using rgb24"<<endl;
        }//endif noDMA

        /*-----------------------------------------------------------------------
         *  have the camera start sending us data
         *-----------------------------------------------------------------------*/
        if (dc1394_start_iso_transmission(handles[i],cameras[i].node)
                !=DC1394_SUCCESS)
        {
            fprintf( stderr, "unable to start camera iso transmission\n");
            dc1394_release_camera(handles[i],&cameras[i]);
            raw1394_destroy_handle(handles[i]);
            //int *killer=0;
            //int a=*killer;
            return false;
        }
        numCamsUsed = i+1;
    }
    dc1394_free_camera_nodes(camera_nodes);
    return true;
}


// Thread runs this method
void Dc1394 :: run() {
    int retval;
    while (1) {
        if ( noDMA ) {
            if ( numCamsToUse > 1 ) {
                cerr<<"[ Dc1394] Unable to multicap for raw1394 cams"<<endl;
            }  else {
                retval = dc1394_single_capture(handles[0],&cameras[0]);
            }
        }
        else {
            if ( numCamsToUse > 1 ) {
                retval = dc1394_dma_multi_capture(cameras, numCamsUsed );
            } else {
                retval = dc1394_dma_single_capture(&cameras[0]) ;
            }
        }
        if ( retval != DC1394_SUCCESS ) {
            cerr<< "single frame capture failed"<<endl;
            cerr<<"killing thread."<<endl;
            int *killer = NULL;
            int a=*killer;
        }
        //bufferUsed=1; xxx
        // notify thread he can grab buffer (shouldn't touch it.

        for ( int i = 0 ; i <numCamsUsed ; i++ ) {
            bufs[i] = (char *)cameras[i].capture_buffer;
        }
        //TellRWMHeCanUseImage((const char *)cameras[0].capture_buffer);
        TellRWMHeCanUseImage( numCamsUsed, (const char **)bufs );
        // later: Dc1394->return(pointer p).
        // dc1394_dma_done_with_buffer(&camera);
        for ( int i=0 ; i<numCamsUsed ; i++ ) {
            dc1394_dma_done_with_buffer(&cameras[i]);
        }
    }
}

void Dc1394 :: tellThreadDoneWithBuffer() {
}
