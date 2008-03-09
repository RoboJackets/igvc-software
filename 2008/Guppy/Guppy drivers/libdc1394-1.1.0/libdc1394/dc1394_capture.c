/*
 * 1394-Based Digital Camera Capture Code for the Control Library
 *
 * Written by Chris Urmson <curmson@ri.cmu.edu>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <stdlib.h>
#include <string.h>
#include <netinet/in.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>

#include "config.h"
#include "dc1394_control.h"
#include "kernel-video1394.h"
#include "dc1394_internal.h"

#define NUM_ISO_CHANNELS 64
#define MAX_NUM_PORTS 8

/*Variables used for simultaneous capture of video from muliple cameras*/
int * _dc1394_buffer[NUM_ISO_CHANNELS];
int _dc1394_frame_captured[NUM_ISO_CHANNELS];
int _dc1394_offset[NUM_ISO_CHANNELS];
int _dc1394_quadlets_per_frame[NUM_ISO_CHANNELS];
int _dc1394_quadlets_per_packet[NUM_ISO_CHANNELS];
int _dc1394_all_captured;

/* variables to handle multiple cameras using a single fd. */
int *_dc1394_dma_fd = NULL;
int *_dc1394_num_using_fd = NULL;

/**********************/
/* Internal functions */
/**********************/

/*****************************************/
/* Functions defined in dc1394_control.c */
/*****************************************/
extern int
_dc1394_get_wh_from_format(int format, int mode, int *w, int *h);

extern int 
_dc1394_get_quadlets_per_packet(int format, int mode, int frame_rate); 

extern int 
_dc1394_quadlets_from_format(int format, int mode);

/**************************************************************
 _dc1394_video_iso_handler

 This is the routine that is plugged into the raw1394 callback
 hook to allow us to capture mutliple iso video streams.  This
 is used in the non DMA capture routines.
***************************************************************/
int 
_dc1394_video_iso_handler(raw1394handle_t handle,  
                          int channel, size_t length, quadlet_t *data) 
{

    /* the first packet of a frame has a 1 in the lsb of the header */
#ifdef LIBRAW1394_OLD
    if ( (data[0] & 0x1) && (_dc1394_frame_captured[channel] != 1) )
#else
    if ( (ntohl(data[0]) & 0x00000001UL) && (_dc1394_frame_captured[channel] != 1) )
#endif

    {
        _dc1394_offset[channel]= 0;
        _dc1394_frame_captured[channel]= 2;

        /* the plus 1 is to shift past the first header quadlet*/
        memcpy((char*)_dc1394_buffer[channel], (char*)(data+1),
               4*_dc1394_quadlets_per_packet[channel]);
        _dc1394_offset[channel]+=_dc1394_quadlets_per_packet[channel];
        
    }
    else if (_dc1394_frame_captured[channel] == 2)
    {
      int copy_n_quadlets = _dc1394_quadlets_per_packet[channel];
      if( _dc1394_offset[channel] + copy_n_quadlets
          > _dc1394_quadlets_per_frame[channel])
      {
        /* this is the last packet. Maybe, we just need a part of its data */
        copy_n_quadlets= _dc1394_quadlets_per_frame[channel]
            - _dc1394_offset[channel];
      }
      
        memcpy((char*)(_dc1394_buffer[channel]+_dc1394_offset[channel]),
               (char*)(data+1), 4*copy_n_quadlets);

        _dc1394_offset[channel]+= copy_n_quadlets;

        if (_dc1394_offset[channel] == _dc1394_quadlets_per_frame[channel])
        {
            _dc1394_frame_captured[channel]= 1;
            _dc1394_all_captured--;
            _dc1394_offset[channel]= 0;

        }
    
    }

    return 1;
}

/************************************************************
 _dc1394_basic_setup

 Sets up camera features that are capture type independent

 Returns DC1394_SUCCESS on success, DC1394_FAILURE otherwise
*************************************************************/
int 
_dc1394_basic_setup(raw1394handle_t handle, nodeid_t node,
                    int channel, int format, int mode,
                    int speed, int frame_rate, 
                    dc1394_cameracapture * camera)
{
    dc1394bool_t is_iso_on= DC1394_FALSE;

/*    if (dc1394_init_camera(handle,node) != DC1394_SUCCESS) 
    {
        printf("(%s) Unable to initialize camera!\n", __FILE__);
        return DC1394_FAILURE;
    }
*/
    /* Addition by Alexis Weiland: Certain cameras start sending iso
       data when they are reset, so we need to stop them so we can set
       up the camera properly.  Setting camera parameters "on the fly"
       while they are sending data doesn't seem to work.  I don't think
       this will cause any problems for other cameras. */
    /* Addition by Dan Dennedy: Restart iso transmission later if it is on */
    if (dc1394_get_iso_status(handle, node, &is_iso_on) != DC1394_SUCCESS)
        return DC1394_FAILURE;

    if (is_iso_on)
    {

        if (dc1394_stop_iso_transmission(handle, node) != DC1394_SUCCESS)
        {
            printf("(%s) Unable to stop iso transmission!\n", __FILE__);
            return DC1394_FAILURE;
        }

    }

    if (dc1394_set_iso_channel_and_speed(handle,node,channel,speed)
        != DC1394_SUCCESS) 
    {
        printf("(%s) Unable to set channel %d and speed %d!\n", __FILE__,
               channel,speed);
        return DC1394_FAILURE;
    }

    if (dc1394_set_video_format(handle,node,format) != DC1394_SUCCESS) 
    {
        printf("(%s) Unable to set video format %d!\n", __FILE__, format);
        return DC1394_FAILURE;
    }

    if (dc1394_set_video_mode(handle, node,mode) != DC1394_SUCCESS) 
    {
        printf("(%s) Unable to set video mode %d!\n", __FILE__, mode);
        return DC1394_FAILURE;
    }

    if (dc1394_set_video_framerate(handle,node,frame_rate) != DC1394_SUCCESS) 
    {
        printf("(%s) Unable to set framerate %d!\n", __FILE__, frame_rate);
        return DC1394_FAILURE;
    }

    if (is_iso_on)
    {

        if (dc1394_start_iso_transmission(handle,node) != DC1394_SUCCESS)
        {
            printf("(%s) Unable to restart iso transmission!\n", __FILE__);
            return DC1394_FAILURE;
        }

    }

    camera->node= node;
    camera->frame_rate= frame_rate;
    camera->channel= channel;
    camera->quadlets_per_packet= _dc1394_get_quadlets_per_packet(format, mode,
                                                                 frame_rate);

    if (camera->quadlets_per_packet < 0)
    {
        return DC1394_FAILURE;
    }
  
    camera->quadlets_per_frame= _dc1394_quadlets_from_format(format, mode);

    if (camera->quadlets_per_frame < 0) 
    {
        return DC1394_FAILURE;
    }

    if (_dc1394_get_wh_from_format(format,mode,&camera->frame_width,
                                   &camera->frame_height) != DC1394_SUCCESS) 
    {
        return DC1394_FAILURE;
    }

    return DC1394_SUCCESS;
}


/*****************************************************
 dc1394_dma_basic_setup

 This sets up the dma for the given camera

******************************************************/
int
_dc1394_dma_basic_setup(int channel,
                        int num_dma_buffers,
                        dc1394_cameracapture *camera)
{

    struct video1394_mmap vmmap;
    struct video1394_wait vwait;
    int i;

    if (camera->dma_device_file == NULL) {
		camera->dma_device_file = malloc(32);
		sprintf((char*)camera->dma_device_file, "/dev/video1394/%d", camera->port );
    }

    /* using_fd counter array NULL if not used yet -- initialize */
    if( NULL == _dc1394_num_using_fd ) {
        _dc1394_num_using_fd = calloc( MAX_NUM_PORTS, sizeof(int) );
        _dc1394_dma_fd = calloc( MAX_NUM_PORTS, sizeof(int) );
    }
	
    if (_dc1394_num_using_fd[camera->port] == 0)
    {

        if ( (camera->dma_fd = open(camera->dma_device_file,O_RDONLY)) < 0 )
        {
            printf("(%s) unable to open video1394 device %s\n", 
		__FILE__,
		camera->dma_device_file);
	    perror( __FILE__ );
            return DC1394_FAILURE;
        }
        _dc1394_dma_fd[camera->port] = camera->dma_fd;

    }
    else
	camera->dma_fd = _dc1394_dma_fd[camera->port];


    _dc1394_num_using_fd[camera->port]++;
    vmmap.sync_tag= 1;
    vmmap.nb_buffers= num_dma_buffers;
    vmmap.flags= VIDEO1394_SYNC_FRAMES;
    vmmap.buf_size= camera->quadlets_per_frame * 4; //number of bytes needed
    vmmap.channel= channel;

    /* tell the video1394 system that we want to listen to the given channel */
    if (ioctl(camera->dma_fd, VIDEO1394_IOC_LISTEN_CHANNEL, &vmmap) < 0)
    {
        printf("(%s) VIDEO1394_IOC_LISTEN_CHANNEL ioctl failed!\n", __FILE__);
        return DC1394_FAILURE;
    }
    //fprintf(stderr,"listening channel set\n");

    camera->dma_frame_size= vmmap.buf_size;
    camera->num_dma_buffers= vmmap.nb_buffers;
    camera->dma_last_buffer= -1;
    vwait.channel= channel;

    /* QUEUE the buffers */
    for (i= 0; i < vmmap.nb_buffers; i++)
    {
        vwait.buffer= i;

        if (ioctl(camera->dma_fd,VIDEO1394_IOC_LISTEN_QUEUE_BUFFER,&vwait) < 0)
        {
            printf("(%s) VIDEO1394_IOC_LISTEN_QUEUE_BUFFER ioctl failed!\n",
                   __FILE__);
            ioctl(camera->dma_fd, VIDEO1394_IOC_UNLISTEN_CHANNEL,
                  &(vwait.channel));
            return DC1394_FAILURE;
        }

    }

    camera->dma_ring_buffer= mmap(0, vmmap.nb_buffers * vmmap.buf_size,
                           PROT_READ,MAP_SHARED, camera->dma_fd, 0);


    /* make sure the ring buffer was allocated */
    if (camera->dma_ring_buffer == (unsigned char*)(-1)) {
        printf("(%s) mmap failed!\n", __FILE__);
        ioctl(camera->dma_fd, VIDEO1394_IOC_UNLISTEN_CHANNEL, &vmmap.channel);
        return DC1394_FAILURE;
    }

    camera->dma_buffer_size= vmmap.buf_size * vmmap.nb_buffers;
    camera->num_dma_buffers_behind = 0;
    return DC1394_SUCCESS;
}



/********************************
 libraw Capture Functions

 These functions use libraw
 to grab frames from the cameras,
 the dma routines are faster, and 
 should be used instead.
*********************************/

/*************************************************************
 dc1394_setup_capture

 Sets up both the camera and the cameracapture structure
 to be used other places.

 Returns DC1394_SUCCESS on success, DC1394_FAILURE otherwise
**************************************************************/
int 
dc1394_setup_capture(raw1394handle_t handle, nodeid_t node, 
                     int channel, int format, int mode, 
                     int speed, int frame_rate, 
                     dc1394_cameracapture * camera) 
{
  if( format == FORMAT_SCALABLE_IMAGE_SIZE)
  {
    return dc1394_setup_format7_capture( handle, node, channel, mode,
                                         speed,
                                         QUERY_FROM_CAMERA, /*bytes_per_paket*/
                                         QUERY_FROM_CAMERA, /*left*/
                                         QUERY_FROM_CAMERA, /*top*/
                                         QUERY_FROM_CAMERA, /*width*/
                                         QUERY_FROM_CAMERA, /*height*/
                                         camera);  
  }
  else
  {
    if (_dc1394_basic_setup(handle,node, channel, format, mode, 
                            speed,frame_rate, camera) != DC1394_SUCCESS)
    {
      return DC1394_FAILURE;
    }
    camera->capture_buffer= (int*)malloc(camera->quadlets_per_frame*4);
   
    if (camera->capture_buffer == NULL)
    {
      printf("(%s) unable to allocate memory for capture buffer\n",
             __FILE__);
      return DC1394_FAILURE;
    }
  }
  
  return DC1394_SUCCESS;
}

/****************************************************
 dc1394_release_camera

 Frees buffer space contained in the cameracapture 
 structure
*****************************************************/
int 
dc1394_release_camera(raw1394handle_t handle,dc1394_cameracapture *camera)
{
    dc1394_unset_one_shot(handle, camera->node);

    if (camera->capture_buffer != NULL) 
    {
        free(camera->capture_buffer);
    }

    return DC1394_SUCCESS;
}

/*****************************************************
 dc1394_single_capture

 Captures a frame of video from the camera specified
******************************************************/
int 
dc1394_single_capture(raw1394handle_t handle,
                      dc1394_cameracapture *camera)
{
    return dc1394_multi_capture(handle, camera, 1);
}

/***************************************************************************
 dc1394_multi_capture

 This routine captures a frame from each camera specified in the cams array.
 Cameras must be set up first using dc1394_setup_camera.

 Returns DC1394_FAILURE if it fails, DC1394_SUCCESS if it succeeds
****************************************************************************/
int 
dc1394_multi_capture(raw1394handle_t handle, dc1394_cameracapture *cams,
                     int num) 
{
    int i, j;
    _dc1394_all_captured= num;

    /*
      this first routine does the setup-
      sets up the global variables needed in the handler,
      sets the iso handler,
      tells the 1394 subsystem to listen for iso packets
    */
    for (i= 0; i < num; i++) 
    {
        _dc1394_buffer[cams[i].channel]= cams[i].capture_buffer;

        if (raw1394_set_iso_handler(handle,cams[i].channel,
                                    _dc1394_video_iso_handler) < 0) 
        {
            /* error handling- for some reason something didn't work, 
               so we have to reset everything....*/
            printf("(%s:%d) error!\n",__FILE__, __LINE__);

            for (j= i - 1; j > -1; j--) 
            {
                raw1394_stop_iso_rcv(handle, cams[j].channel);
                raw1394_set_iso_handler(handle, cams[j].channel, NULL);
            }

            return DC1394_FAILURE;
        }

        _dc1394_frame_captured[cams[i].channel]= 0;
        _dc1394_quadlets_per_frame[cams[i].channel]=
            cams[i].quadlets_per_frame;
        _dc1394_quadlets_per_packet[cams[i].channel]=
            cams[i].quadlets_per_packet;

        if (raw1394_start_iso_rcv(handle,cams[i].channel) < 0) 
        {
            /* error handling- for some reason something didn't work, 
               so we have to reset everything....*/
            printf("(%s:%d) error!\n", __FILE__, __LINE__);

            for (j= 0; j < num; j++) 
            {
                raw1394_stop_iso_rcv(handle, cams[j].channel);
                raw1394_set_iso_handler(handle, cams[j].channel, NULL);
            }

            return DC1394_FAILURE;
        }

    }

    /* now we iterate till the data is here*/
    while (_dc1394_all_captured != 0) 
    {
        raw1394_loop_iterate(handle);
    }
  
    /* now stop the subsystem from listening*/
    for (i= 0; i < num; i++) 
    {
        raw1394_stop_iso_rcv(handle, cams[i].channel);
        raw1394_set_iso_handler(handle,cams[i].channel, NULL);
    }

    return DC1394_SUCCESS;
}

/**********************************
 DMA Capture Functions 

 These routines will be much faster
 than the above capture routines.
***********************************/

/*****************************************************
 dc1394_dma_setup_capture

 This sets up the given camera to capture images using
 the dma engine.  Should be much faster than the above
 routines
******************************************************/
int
dc1394_dma_setup_capture(raw1394handle_t handle, nodeid_t node,
                         int channel, int format, int mode,
                         int speed, int frame_rate,
                         int num_dma_buffers, 
			 int drop_frames,
			 const char *dma_device_file,
			 dc1394_cameracapture *camera)
{
 
    dc1394_camerahandle *camera_handle;
    camera_handle = (dc1394_camerahandle*) raw1394_get_userdata( handle );
 
    if( format == FORMAT_SCALABLE_IMAGE_SIZE)
    {
      return dc1394_dma_setup_format7_capture( handle, node, channel, mode, speed,
					       QUERY_FROM_CAMERA, /*bytes_per_paket*/
					       QUERY_FROM_CAMERA, /*left*/
					       QUERY_FROM_CAMERA, /*top*/
					       QUERY_FROM_CAMERA, /*width*/
					       QUERY_FROM_CAMERA, /*height*/
					       num_dma_buffers,
					       drop_frames,
					       dma_device_file,
					       camera);  
    }
    else {
      camera->port = camera_handle->port;
      camera->dma_device_file = dma_device_file;
      camera->drop_frames = drop_frames;

      if (_dc1394_basic_setup(handle,node, channel, format, mode,
			      speed,frame_rate, camera) != DC1394_SUCCESS)
	{
	  return DC1394_FAILURE;
	}

      return _dc1394_dma_basic_setup (channel, num_dma_buffers, camera);
    }
}

/*****************************************************
 dc1394_dma_release_camera

 This releases memory that was mapped by
 dc1394_dma_setup_camera
*****************************************************/
int 
dc1394_dma_release_camera(raw1394handle_t handle,
                          dc1394_cameracapture *camera) 
{
    //dc1394_stop_iso_transmission(handle,camera->node);
    //ioctl(_dc1394_dma_fd,VIDEO1394_IOC_UNLISTEN_CHANNEL, &(camera->channel));

    if (camera->dma_ring_buffer)
    {
        munmap((void*)camera->dma_ring_buffer,camera->dma_buffer_size);

	_dc1394_num_using_fd[camera->port]--;
	
	if (_dc1394_num_using_fd[camera->port] == 0)
	  {
	    
	    while (close(camera->dma_fd) != 0)
	      {
		printf("(%s) waiting for dma_fd to close\n", __FILE__);
		sleep(1);
	      }
	    
	  }
    }

    return DC1394_SUCCESS;
}

/*****************************************************
 dc1394_dma_unlisten

 This tells video1394 to halt iso reception.
*****************************************************/
int 
dc1394_dma_unlisten(raw1394handle_t handle, dc1394_cameracapture *camera)
{

    if (ioctl(camera->dma_fd, VIDEO1394_IOC_UNLISTEN_CHANNEL, &(camera->channel))
        < 0)
    {
        return DC1394_FAILURE;
    }
    else
    {
        return DC1394_SUCCESS;
    }

}

/****************************************************
 _dc1394_dma_multi_capture_private

 This capture a frame from each of the cameras passed
 in cams.  After you are finished with the frame, you
 must return the buffer to the pool by calling
 dc1394_dma_done_with_buffer.

 This function is private.

*****************************************************/
int
_dc1394_dma_multi_capture_private(dc1394_cameracapture *cams, int num, dc1394videopolicy_t policy) 
{
    struct video1394_wait vwait;
    int i;
    int cb;
    int j;
    int result=-1;
    int last_buffer_orig;
    int extra_buf;

    for (i= 0; i < num; i++)
    {
        last_buffer_orig = cams[i].dma_last_buffer;
        cb = (cams[i].dma_last_buffer + 1) % cams[i].num_dma_buffers;
        cams[i].dma_last_buffer = cb;

			vwait.channel = cams[i].channel;
			vwait.buffer = cb;
			switch (policy) {
			case VIDEO1394_POLL:
			  result=ioctl(cams[i].dma_fd, VIDEO1394_IOC_LISTEN_POLL_BUFFER, &vwait);
			  break;
			case VIDEO1394_WAIT:
			default:
			  result=ioctl(cams[i].dma_fd, VIDEO1394_IOC_LISTEN_WAIT_BUFFER, &vwait);
			  break;
			}
			if ( result != 0) 
			{       
			        cams[i].dma_last_buffer = last_buffer_orig;
                                if ((policy==VIDEO1394_POLL) && (errno == EINTR))
				  {                       
				    // when no frames is present, say so.
				    return DC1394_NO_FRAME;
				  }
				else
				  {
				    printf("(%s) VIDEO1394_IOC_LISTEN_WAIT/POLL_BUFFER ioctl failed!\n",
					   __FILE__);
				    cams[i].dma_last_buffer++; //Ringbuffer-index or counter?
				    return DC1394_FAILURE;
				  }
			}

			extra_buf = vwait.buffer;

			if (cams[i].drop_frames)
			{
				if (extra_buf > 0)
 				{
				    for (j = 0; j < extra_buf; j++)
				    {
					vwait.buffer = (cb + j) % cams[i].num_dma_buffers;
					if (ioctl(cams[i].dma_fd, VIDEO1394_IOC_LISTEN_QUEUE_BUFFER, &vwait) < 0) 
					{
						printf("(%s) VIDEO1394_IOC_LISTEN_QUEUE_BUFFER failed in "
							   "multi capture!\n", __FILE__);
						return DC1394_FAILURE;
					}
				    }
				    cams[i].dma_last_buffer = (cb + extra_buf) % cams[i].num_dma_buffers;

				    /* Get the corresponding filltime: */
				    vwait.buffer = cams[i].dma_last_buffer;
				    if(ioctl(cams[i].dma_fd, VIDEO1394_IOC_LISTEN_POLL_BUFFER, &vwait) < 0)
				    {
					printf("(%s) VIDEO1394_IOC_LISTEN_POLL_BUFFER "
					       "failed in multi capture!\n",
					       __FILE__);
					return DC1394_FAILURE;
				    }
				}
			}

			/* point to the next buffer in the dma ringbuffer */
			cams[i].capture_buffer = (int*)(cams[i].dma_ring_buffer +
							cams[i].dma_last_buffer *
							cams[i].dma_frame_size);

			cams[i].filltime = vwait.filltime;
			cams[i].num_dma_buffers_behind = extra_buf;
    }

    return DC1394_SUCCESS;
}

/****************************************************
 dc1394_dma_single_capture

 This captures a frame from the given camera. Two
 policies are available: wait for a frame or return
 if no frame is available (POLL)
*****************************************************/
int 
dc1394_dma_single_capture(dc1394_cameracapture *camera) 
{
    return _dc1394_dma_multi_capture_private(camera,1, VIDEO1394_WAIT);
}

int
dc1394_dma_single_capture_poll(dc1394_cameracapture *camera)
{
    return _dc1394_dma_multi_capture_private(camera,1, VIDEO1394_POLL);
}

/****************************************************
 dc1394_dma_multi_capture

 This captures a frame from the given camera. Two
 policies are available: wait for a frame or return
 if no frame is available (POLL)
*****************************************************/
int 
dc1394_dma_multi_capture(dc1394_cameracapture *camera, int num) 
{
    return _dc1394_dma_multi_capture_private(camera, num, VIDEO1394_WAIT);
}

int
dc1394_dma_multi_capture_poll(dc1394_cameracapture *camera, int num)
{
    return _dc1394_dma_multi_capture_private(camera, num, VIDEO1394_POLL);
}

/****************************************************
 dc1394_dma_done_with_buffer

 This allows the driver to use the buffer previously
 handed to the user by dc1394_dma_*_capture
*****************************************************/
int 
dc1394_dma_done_with_buffer(dc1394_cameracapture *camera) 
{
    
    struct video1394_wait vwait;
    
    if (camera->dma_last_buffer == -1)
        return DC1394_SUCCESS;

    vwait.channel= camera->channel;
    vwait.buffer= camera->dma_last_buffer;

    if (ioctl(camera->dma_fd, VIDEO1394_IOC_LISTEN_QUEUE_BUFFER, &vwait) < 0) 
    {
        printf("(%s) VIDEO1394_IOC_LISTEN_QUEUE_BUFFER failed in "
               "done with buffer!\n", __FILE__);
        return DC1394_FAILURE;
    }

    return DC1394_SUCCESS;
}
