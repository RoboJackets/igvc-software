/*
 * 1394-Based Digital Camera Format_7 functions for the Control Library
 *
 * Written by Damien Douxchamps <douxchamps@ieee.org>
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
#include <unistd.h>
#include <netinet/in.h>
#include <errno.h>
#include <stdlib.h>
 
#include "dc1394_control.h"
#include "dc1394_internal.h"
#include "config.h"

#define REG_CAMERA_V_CSR_INQ_BASE                        0x2E0U

#define REG_CAMERA_FORMAT7_MAX_IMAGE_SIZE_INQ            0x000U
#define REG_CAMERA_FORMAT7_UNIT_SIZE_INQ                 0x004U
#define REG_CAMERA_FORMAT7_IMAGE_POSITION                0x008U
#define REG_CAMERA_FORMAT7_IMAGE_SIZE                    0x00CU
#define REG_CAMERA_FORMAT7_COLOR_CODING_ID               0x010U
#define REG_CAMERA_FORMAT7_COLOR_CODING_INQ              0x014U
#define REG_CAMERA_FORMAT7_PIXEL_NUMBER_INQ              0x034U
#define REG_CAMERA_FORMAT7_TOTAL_BYTES_HI_INQ            0x038U
#define REG_CAMERA_FORMAT7_TOTAL_BYTES_LO_INQ            0x03CU
#define REG_CAMERA_FORMAT7_PACKET_PARA_INQ               0x040U
#define REG_CAMERA_FORMAT7_BYTE_PER_PACKET               0x044U
#define REG_CAMERA_FORMAT7_PACKET_PER_FRAME_INQ          0x048U
#define REG_CAMERA_FORMAT7_UNIT_POSITION_INQ             0x04CU
#define REG_CAMERA_FORMAT7_FRAME_INTERVAL_INQ            0x050U
#define REG_CAMERA_FORMAT7_DATA_DEPTH_INQ                0x054U
#define REG_CAMERA_FORMAT7_COLOR_FILTER_ID               0x058U
#define REG_CAMERA_FORMAT7_VALUE_SETTING                 0x07CU

/**********************/ 
/* Internal functions */
/**********************/

static int
QueryFormat7CSROffset(raw1394handle_t handle, nodeid_t node, int mode,
		      quadlet_t *value)
{
    int retval;

    if ( (mode > MODE_FORMAT7_MAX) || (mode < MODE_FORMAT7_MIN) )
    {
        return DC1394_FAILURE;
    }

    mode-= MODE_FORMAT7_MIN;
    retval= GetCameraControlRegister(handle, node,
                                     REG_CAMERA_V_CSR_INQ_BASE +
                                     (mode * 0x04U), value);
    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

static int
GetCameraFormat7Register(raw1394handle_t handle, nodeid_t node,
                         unsigned int mode, octlet_t offset, quadlet_t *value)
{
    int retval, retry= MAX_RETRIES;
    quadlet_t csr;
    
    dc1394_camerahandle *camera;
    camera = (dc1394_camerahandle*) raw1394_get_userdata( handle );

    if ( (mode > MODE_FORMAT7_MAX) || (mode < MODE_FORMAT7_MIN) )
    {
        return DC1394_FAILURE;
    }

    if (camera->format7_csr[mode-MODE_FORMAT7_MIN]==0) {
      if (QueryFormat7CSROffset(handle, node, mode, &csr) != DC1394_SUCCESS) 
	return DC1394_FAILURE;
      else 
	camera->format7_csr[mode-MODE_FORMAT7_MIN]=csr;
    }
    else {
      csr=camera->format7_csr[mode-MODE_FORMAT7_MIN];
    }

    csr*=0x04UL;

    /* retry a few times if necessary (addition by PDJ) */
    while(retry--)
    {
        retval= raw1394_read(handle, 0xffc0 | node,
                             CONFIG_ROM_BASE + csr + offset, 4, value);

#ifdef LIBRAW1394_OLD
        if (retval >= 0)
        {
            int ack= retval >> 16;
            int rcode= retval & 0xffff;

#ifdef SHOW_ERRORS
            printf("Format 7 reg read ack of %x rcode of %x\n", ack, rcode);
#endif

            if ( ((ack == ACK_PENDING) || (ack == ACK_LOCAL)) &&
                 (rcode == RESP_COMPLETE) )
            { 
                /* conditionally byte swap the value */
                *value= ntohl(*value); 
                return DC1394_SUCCESS;
            }

        }
#else
        if (!retval)
        {
            /* conditionally byte swap the value */
            *value= ntohl(*value);
            return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
        }
        else if (errno != EAGAIN)
        {
            return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
        }
#endif /* LIBRAW1394_VERSION <= 0.8.2 */

        usleep(SLOW_DOWN);
    }
    
    *value = ntohl(*value);
    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

static int
SetCameraFormat7Register(raw1394handle_t handle, nodeid_t node,
                         unsigned int mode, octlet_t offset, quadlet_t value)
{
    int retval, retry= MAX_RETRIES;
    quadlet_t csr;
    
    dc1394_camerahandle *camera;
    camera = (dc1394_camerahandle*) raw1394_get_userdata( handle );

    if ( (mode > MODE_FORMAT7_MAX) || (mode < MODE_FORMAT7_MIN) )
    {
        return DC1394_FAILURE;
    }

    if (camera->format7_csr[mode-MODE_FORMAT7_MIN]==0) {
      if (QueryFormat7CSROffset(handle, node, mode, &csr) != DC1394_SUCCESS) 
	return DC1394_FAILURE;
      else 
	camera->format7_csr[mode-MODE_FORMAT7_MIN]=csr;
    }
    else {
      csr=camera->format7_csr[mode-MODE_FORMAT7_MIN];
    }

    csr*=0x04UL;

    /* conditionally byte swap the value (addition by PDJ) */
    value= htonl(value);
 
    /* retry a few times if necessary (addition by PDJ) */
    while(retry--)
    {
        retval= raw1394_write(handle, 0xffc0 | node,
                              CONFIG_ROM_BASE + offset + csr, 4, &value);

#ifdef LIBRAW1394_OLD
        if (retval >= 0)
        {
            int ack= retval >> 16;
            int rcode= retval & 0xffff;

#ifdef SHOW_ERRORS
            printf("Format 7 reg write ack of %x rcode of %x\n", ack, rcode);
#endif

            if ( ((ack == ACK_PENDING) || (ack == ACK_LOCAL) ||
                  (ack == ACK_COMPLETE)) &&
                 ((rcode == RESP_COMPLETE) || (rcode == RESP_SONY_HACK)) ) 
            {
                return DC1394_SUCCESS;
            }
            
            
        }
#else
        if (!retval || (errno != EAGAIN))
        {
            return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
        }
#endif /* LIBRAW1394_VERSION <= 0.8.2 */

 	usleep(SLOW_DOWN);
    }
    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
    
}

/*==========================================================================
 * This function implements the handshaking available (and sometimes required)
 * on some cameras that comply with the IIDC specs v1.30. Thanks to Yasutoshi
 * Onishi for his feedback and info.
 *==========================================================================*/

int
_dc1394_v130_handshake(raw1394handle_t handle, nodeid_t node, int mode)
{
  int setting_1, err_flag1, err_flag2, v130handshake;
  int exit_loop;

  dc1394_camerahandle *camera;
  camera = (dc1394_camerahandle*) raw1394_get_userdata( handle );

  if (camera->sw_version >= IIDC_VERSION_1_30) {
    // We don't use > because 114 is for ptgrey cameras which are not 1.30 but 1.20
    if (dc1394_query_format7_value_setting(handle, node, mode, &v130handshake,
					   &setting_1, &err_flag1, &err_flag2)
	!= DC1394_SUCCESS) {
      printf("(%s) Unable to read value setting register.\n", __FILE__);
      return DC1394_FAILURE;
    }
  }
  else {
    v130handshake=0;
  }   
  
  if (v130handshake==1) {
    // we should use advanced IIDC v1.30 handshaking.
    //fprintf(stderr,"using handshaking\n");
    // set value setting to 1
    if (dc1394_set_format7_value_setting(handle, node, mode) != DC1394_SUCCESS) {
      printf("(%s) Unable to set value setting register.\n", __FILE__);
      return DC1394_FAILURE;
    }
    // wait for value setting to clear:
    exit_loop=0;
    while (!exit_loop) { // WARNING: there is no timeout in this loop yet.
      if (dc1394_query_format7_value_setting(handle, node, mode, &v130handshake,
					     &setting_1, &err_flag1, &err_flag2)
	  != DC1394_SUCCESS) {
	printf("(%s) Unable to read value setting register.\n", __FILE__);
	return DC1394_FAILURE;
      }
      exit_loop=(setting_1==0);
      usleep(0); 
    }
    if (err_flag1>0) {
      printf("(%s) Invalid image position, size, color coding, ISO speed or bpp\n", __FILE__);
      return DC1394_FAILURE;
    }
    
    // bytes per packet... registers are ready for reading.
  }
  return DC1394_SUCCESS;
}

int
_dc1394_v130_errflag2(raw1394handle_t handle, nodeid_t node, int mode)
{
  int setting_1, err_flag1, err_flag2, v130handshake;

  dc1394_camerahandle *camera;
  camera = (dc1394_camerahandle*) raw1394_get_userdata( handle );

  //fprintf(stderr,"Checking error flags\n");

  if (camera->sw_version >= IIDC_VERSION_1_30) { // if version is 1.30.
    // We don't use > because 0x114 is for ptgrey cameras which are not 1.30 but 1.20
    if (dc1394_query_format7_value_setting(handle, node, mode, &v130handshake,
					   &setting_1, &err_flag1, &err_flag2)
	!= DC1394_SUCCESS) {
      printf("(%s) Unable to read value setting register.\n", __FILE__);
      return DC1394_FAILURE;
    }
  }
  else {
    v130handshake=0;
  }
      
  if (v130handshake==1) {
    if (err_flag2==0)
      return DC1394_SUCCESS;
    else {
      printf("(%s) Error flag 2 is set: proposed bytes per packet is not a valid value.\n", __FILE__);
      return DC1394_FAILURE;
    }
  }

  return DC1394_SUCCESS;
}

/*=================================================================================*
 * The following function returns the bytes per pixel for a defined color coding.  *
 *=================================================================================*/

float
_Format7BytePerPixel(int color_coding)
{
  switch (color_coding)
    {
    case COLOR_FORMAT7_MONO8: return 1.0;
      break;
    case COLOR_FORMAT7_YUV411: return 1.5;
      break;
    case COLOR_FORMAT7_YUV422: return 2.0;
      break;
    case COLOR_FORMAT7_YUV444: return 3.0;
      break;
    case COLOR_FORMAT7_RGB8: return 3.0;
      break;
    case COLOR_FORMAT7_MONO16: return 2.0;
      break;
    case COLOR_FORMAT7_RGB16: return 6.0;
      break;
    case COLOR_FORMAT7_MONO16S: return 2.0;
      break;
    case COLOR_FORMAT7_RGB16S: return 6.0;
      break;
    case COLOR_FORMAT7_RAW8: return 1.0;
      break;
    case COLOR_FORMAT7_RAW16: return 2.0;
      break;
    default: return (-1);
    }
}

 
/*====================================================================== 
 *   see documentation of dc1394_setup_format7_capture() in
 *   dc1394_control.h
 *======================================================================*/
int
_dc1394_basic_format7_setup(raw1394handle_t handle, nodeid_t node,
                            int channel, int mode, int speed,
                            int bytes_per_packet,
                            int left, int top,
                            int width, int height, 
                            dc1394_cameracapture *camera)
{
  dc1394bool_t is_iso_on= DC1394_FALSE;
  unsigned int unit_bytes, max_bytes;
  unsigned packet_bytes=0;
  unsigned int recom_bpp;
  int packets_per_frame;
  int color_coding;
  unsigned int camera_left = 0;
  unsigned int camera_top = 0;
  unsigned int camera_width = 0;
  unsigned int camera_height = 0;
  unsigned int max_width = 0;
  unsigned int max_height = 0;

  dc1394_camerahandle *camerahandle;
  camerahandle = (dc1394_camerahandle*) raw1394_get_userdata( handle );

  if (dc1394_get_iso_status(handle, node, &is_iso_on) != DC1394_SUCCESS) {
    return DC1394_FAILURE;
  }

  if (is_iso_on) {
    if (dc1394_stop_iso_transmission(handle, node) != DC1394_SUCCESS) {
      printf("(%s) Unable to stop iso transmission!\n", __FILE__);
      return DC1394_FAILURE;
    }
  }

  if (dc1394_set_iso_channel_and_speed(handle,node,channel,speed)!=DC1394_SUCCESS) {
    printf("(%s) Unable to set channel %d and speed %d!\n",__FILE__,channel,speed);
    return DC1394_FAILURE;
  }
  
  if (dc1394_set_video_format(handle,node,FORMAT_SCALABLE_IMAGE_SIZE)!=DC1394_SUCCESS) {
    printf("(%s) Unable to set video format %d!\n",__FILE__, FORMAT_SCALABLE_IMAGE_SIZE);
    return DC1394_FAILURE;
  }
  
  if (dc1394_set_video_mode(handle, node,mode) != DC1394_SUCCESS) {
    printf("(%s) Unable to set video mode %d!\n", __FILE__, mode);
    return DC1394_FAILURE;
  }

  // get BPP before setting sizes,...
  if (bytes_per_packet==QUERY_FROM_CAMERA) {
    if (dc1394_query_format7_byte_per_packet(handle, node, mode, &bytes_per_packet) != DC1394_SUCCESS){
      printf("(%s) Unable to get F7 bpp %d!\n", __FILE__, mode);
      return DC1394_FAILURE;
    }
  }

  /*-----------------------------------------------------------------------
   *  set image position. If QUERY_FROM_CAMERA was given instead of a
   *  position, use the actual value from camera
   *-----------------------------------------------------------------------*/

  /* The image position should be checked regardless of the left and top values
     as we also use it for the size setting */

  if (dc1394_query_format7_image_position(handle, node, mode, &camera_left, &camera_top) != DC1394_SUCCESS) {
    printf("(%s) Unable to query image position\n", __FILE__);
    return DC1394_FAILURE;
  }
    
  if( left == QUERY_FROM_CAMERA) left = camera_left;
  if( top == QUERY_FROM_CAMERA)  top = camera_top;
  
  if (dc1394_set_format7_image_position(handle, node, mode, left, top) != DC1394_SUCCESS) {
    printf("(%s) Unable to set format 7 image position to "
           "left=%d and top=%d!\n",  __FILE__, left, top);
    return DC1394_FAILURE;
  }

  /*-----------------------------------------------------------------------
   *  If QUERY_FROM_CAMERA was given instead of an image size
   *  use the actual value from camera.
   *-----------------------------------------------------------------------*/
  if( width == QUERY_FROM_CAMERA || height == QUERY_FROM_CAMERA) {
    if (dc1394_query_format7_image_size(handle, node, mode, &camera_width, &camera_height) != DC1394_SUCCESS) {
      printf("(%s) Unable to query image size\n", __FILE__);
      return DC1394_FAILURE;
    }
    
    /* Idea from Ralf Ebeling: we should check if the image sizes are > 0.
       If == 0, we use the maximum size available */
    if (width == QUERY_FROM_CAMERA) {
      if (camera_width>0)
	width = camera_width;
      else
	width = USE_MAX_AVAIL;
    }
    if (height == QUERY_FROM_CAMERA) {
      if (camera_height>0)
	height = camera_height;
      else
	height = USE_MAX_AVAIL;
    }
  }

  /*-----------------------------------------------------------------------
   *  If USE_MAX_AVAIL was given instead of an image size
   *  use the max image size for the given image position
   *-----------------------------------------------------------------------*/
  if( width == USE_MAX_AVAIL || height == USE_MAX_AVAIL) {
    if (dc1394_query_format7_max_image_size(handle, node, mode, &max_width, &max_height) != DC1394_SUCCESS) {
      printf("(%s) Unable to query max image size\n", __FILE__);
      return DC1394_FAILURE;
    }
    if( width == USE_MAX_AVAIL)  width  = max_width - left;
    if( height == USE_MAX_AVAIL) height = max_height - top;
    
  }

  if (dc1394_set_format7_image_size(handle, node, mode, width, height) != DC1394_SUCCESS) {
    printf("(%s) Unable to set format 7 image size to width=%d and height=%d!\n", __FILE__, width, height);
    return DC1394_FAILURE;
  }

  /*-----------------------------------------------------------------------
   *  Bytes-per-packet definition
   *-----------------------------------------------------------------------*/
  if (dc1394_query_format7_recommended_byte_per_packet(handle, node, mode, &recom_bpp) != DC1394_SUCCESS) {
    printf("Recommended byte-per-packet inq error\n");
    return DC1394_FAILURE;
  }
  
  if (dc1394_query_format7_packet_para(handle, node, mode, &unit_bytes, &max_bytes) != DC1394_SUCCESS) { /* PACKET_PARA_INQ */
    printf("Packet para inq error\n");
    return DC1394_FAILURE;
  }

  //fprintf(stderr,"recommended bpp: %d\n",recom_bpp);
  
  switch (bytes_per_packet) {
  case USE_RECOMMENDED:
    if (recom_bpp>0) {
      bytes_per_packet=recom_bpp;
    }
    else { // recom. bpp asked, but register is 0. IGNORED
      printf("(%s) Recommended bytes per packet asked, but register is zero. Falling back to MAX BPP for mode %d \n", __FILE__, mode);
      bytes_per_packet=max_bytes;
    }
    break;
  case USE_MAX_AVAIL:
    bytes_per_packet = max_bytes;
    break;
  //this case was handled by a previous call. Show error if we get in there. 
  case QUERY_FROM_CAMERA:
    /*if (dc1394_query_format7_byte_per_packet(handle, node, mode, &bytes_per_packet) != DC1394_SUCCESS) {
      printf("(%s) Bytes_per_packet query failure\n", __FILE__);
      return DC1394_FAILURE;
    }*/
    // if we wanted QUERY_FROM_CAMERA, the QUERY_FROM_CAMERA value has been overwritten by
    // the current value at the beginning of the program. It is thus not possible to reach this code fragment.
    printf("(%s:%d) Bytes_per_packet error: we should not reach this code region\n", __FILE__,__LINE__);
    break;
  default:
    // we have to take several tricks into account here:
    // 1) BPP could be zero, in which case it becomes MAX_BPP
    // 2) UNIT_BYTES could also be zero, in which case we force it to MAX_BPP.
    //    This actually further forces BPP to be set to MAX_BPP too.

    if (unit_bytes==0) {
      unit_bytes=max_bytes;
    }
    if (bytes_per_packet > max_bytes) {
      bytes_per_packet = max_bytes;
    }
    else {
      if (bytes_per_packet < unit_bytes) {
	bytes_per_packet = unit_bytes;
      }
    }
    bytes_per_packet-=bytes_per_packet % unit_bytes;
    break;
  }
      
  if (dc1394_set_format7_byte_per_packet(handle, node, mode, bytes_per_packet) != DC1394_SUCCESS) {
    printf("(%s) Unable to set format 7 bytes per packet %d \n", __FILE__, mode);
    return DC1394_FAILURE;
  }
  
  if (dc1394_query_format7_byte_per_packet(handle, node, mode, &packet_bytes) == DC1394_SUCCESS) {
    camera->quadlets_per_packet= packet_bytes /4;
    if (camera->quadlets_per_packet<=0) {
      printf("(%s) No format 7 bytes per packet %d \n", __FILE__, mode);
      return DC1394_FAILURE;
    }
    //printf("Camera has now %d bytes per packet\n", packet_bytes);
  }
  else {
    printf("(%s) Unable to get format 7 bytes per packet %d \n", __FILE__, mode);
    return DC1394_FAILURE;
  }
  
  camera->node = node;
  /*
   * TODO: frame_rate not used for format 7, may be calculated
   */
  camera->frame_rate = 0; 
  camera->channel=channel;
  
  /*-----------------------------------------------------------------------
   *  ensure that quadlet aligned buffers are big enough, still expect
   *  problems when width*height  != quadlets_per_frame*4
   *-----------------------------------------------------------------------*/
  if (camerahandle->sw_version >= IIDC_VERSION_1_30) { // if version is 1.30
    if (dc1394_query_format7_packet_per_frame(handle, node, mode, &packets_per_frame)!=DC1394_SUCCESS) {
      printf("(%s) Unable to get format 7 packets per frame %d \n", __FILE__, mode);
      return DC1394_FAILURE;
    }
    camera->quadlets_per_frame=(packets_per_frame*packet_bytes)/4;
  }
  else {
    // For other specs revisions, we use a trick to determine the total bytes.
    // We don't use the total_bytes register in 1.20 as it has been interpreted in
    // different ways by manufacturers. Thanks to Martin Gramatke for pointing this trick out.
    if (dc1394_query_format7_color_coding_id(handle, node, mode, &color_coding)!=DC1394_SUCCESS) {
      printf("(%s) Unable to get format 7 color coding for mode %d \n", __FILE__, mode);
      return DC1394_FAILURE;
    }
    else {
      //fprintf(stderr,"color coding: %d\n",color_coding);
      packets_per_frame = ((int)(width * height * _Format7BytePerPixel(color_coding)) +
			   bytes_per_packet -1) / bytes_per_packet;
      camera->quadlets_per_frame=(packets_per_frame*bytes_per_packet)/4;
    }
    /*
      if (dc1394_query_format7_total_bytes(handle, node, mode, &camera->quadlets_per_frame)!= DC1394_SUCCESS) {
      printf("(%s) Unable to get format 7 total bytes per frame %d \n", __FILE__, mode);
      return DC1394_FAILURE;
      }
      camera->quadlets_per_frame/=4;
    */
    //fprintf(stderr,"quadlets per frame: %d\n",camera->quadlets_per_frame);
  }

  if (camera->quadlets_per_frame<=0) {
    return DC1394_FAILURE;
  }
  camera->frame_width = width; /* irrespective of pixel depth */
  camera->frame_height= height;
  
  if (is_iso_on){
    if (dc1394_start_iso_transmission(handle,node) != DC1394_SUCCESS) {
      printf("(%s) Unable to start iso transmission!\n", __FILE__);
      return DC1394_FAILURE;
    }
  }

  return DC1394_SUCCESS;
}


/**********************/
/* External functions */
/**********************/


/*=========================================================================
 *  DESCRIPTION OF FUNCTION:  dc1394_setup_format7_capture
 *  ==> see headerfile
 *=======================================================================*/
int
dc1394_setup_format7_capture(raw1394handle_t handle, nodeid_t node,
                             int channel, int mode, int speed,
                             int bytes_per_packet,
                             unsigned int left, unsigned int top,
                             unsigned int width, unsigned int height, 
                             dc1394_cameracapture * camera)

{
  /* printf( "trying to setup format7 with \n"
          "bpp    = %d\n"
          "pos_x  = %d\n"
          "pos_y  = %d\n"
          "size_x = %d\n"
          "size_y = %d\n",
          bytes_per_packet, left, top, width, height);*/
  
  if (_dc1394_basic_format7_setup(handle,node, channel, mode,
                                  speed, bytes_per_packet,
                                  left, top, width, height, camera) ==
      DC1394_FAILURE)
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
  
  return DC1394_SUCCESS;
}



/*=========================================================================
 *  DESCRIPTION OF FUNCTION:  dc1394_dma_setup_format7_capture
 *  ==> see headerfile
 *=======================================================================*/
int
dc1394_dma_setup_format7_capture(raw1394handle_t handle, nodeid_t node,
                                 int channel, int mode, int speed,
                                 int bytes_per_packet,
                                 unsigned int left, unsigned int top,
                                 unsigned int width, unsigned int height,
                                 int num_dma_buffers,
				 int drop_frames,
				 const char *dma_device_file,
                                 dc1394_cameracapture *camera)
{
    dc1394_camerahandle *camera_handle;
    camera_handle = (dc1394_camerahandle*) raw1394_get_userdata( handle );
 
    if (_dc1394_basic_format7_setup(handle,node, channel, mode,
				    speed, bytes_per_packet,
				    left, top, width, height, camera) != DC1394_SUCCESS)
    {
      return DC1394_FAILURE;
    }

    camera->port = camera_handle->port;
    camera->dma_device_file = dma_device_file;
    camera->drop_frames = drop_frames;

    return _dc1394_dma_basic_setup(channel,num_dma_buffers, camera);

}



int
dc1394_query_format7_max_image_size(raw1394handle_t handle, nodeid_t node,
 				    unsigned int mode,
                                    unsigned int *horizontal_size,
 				    unsigned int *vertical_size)
{
    int retval;
    quadlet_t value;
    
    if ( (mode > MODE_FORMAT7_MAX) || (mode < MODE_FORMAT7_MIN) )
    {
        return DC1394_FAILURE;
    }

    retval= GetCameraFormat7Register(handle, node, mode,
				     REG_CAMERA_FORMAT7_MAX_IMAGE_SIZE_INQ,
				     &value);
    *horizontal_size  = (unsigned int) ( value & 0xFFFF0000UL ) >> 16;
    *vertical_size= (unsigned int) ( value & 0x0000FFFFUL );
    
    return retval;
}
 
int
dc1394_query_format7_unit_size(raw1394handle_t handle, nodeid_t node,
 			       unsigned int mode,
                               unsigned int *horizontal_unit,
 			       unsigned int *vertical_unit)
{
    int retval;
    quadlet_t value;
   
    if ( (mode > MODE_FORMAT7_MAX) || (mode < MODE_FORMAT7_MIN) )
    {
        return DC1394_FAILURE;
    }

    retval=GetCameraFormat7Register(handle, node, mode,
				    REG_CAMERA_FORMAT7_UNIT_SIZE_INQ,
				    &value);
    *horizontal_unit  = (unsigned int) ( value & 0xFFFF0000UL ) >> 16;
    *vertical_unit= (unsigned int) ( value & 0x0000FFFFUL );

    return retval;
}
 
int
dc1394_query_format7_image_position(raw1394handle_t handle, nodeid_t node,
 				    unsigned int mode,
                                    unsigned int *left_position,
 				    unsigned int *top_position)
{
    int retval;
    quadlet_t value;
   
    if ( (mode > MODE_FORMAT7_MAX) || (mode < MODE_FORMAT7_MIN) )
    {
        return DC1394_FAILURE;
    }

    retval=GetCameraFormat7Register(handle, node, mode,
				    REG_CAMERA_FORMAT7_IMAGE_POSITION,
				    &value);
    *left_position = (unsigned int) ( value & 0xFFFF0000UL ) >> 16;
    *top_position= (unsigned int) ( value & 0x0000FFFFUL );       

    return retval;
}
 
 
int
dc1394_query_format7_image_size(raw1394handle_t handle, nodeid_t node,
 				unsigned int mode, unsigned int *width,
 				unsigned int *height)
{
    int retval;
    quadlet_t value;

    if ( (mode > MODE_FORMAT7_MAX) || (mode < MODE_FORMAT7_MIN) )
    {
        return DC1394_FAILURE;
    }

    retval=GetCameraFormat7Register(handle, node, mode,
				    REG_CAMERA_FORMAT7_IMAGE_SIZE, &value);
    *width= (unsigned int) ( value & 0xFFFF0000UL ) >> 16;
    *height = (unsigned int) ( value & 0x0000FFFFUL );       

    return retval;
}
 
int
dc1394_query_format7_color_coding_id(raw1394handle_t handle, nodeid_t node,
 				     unsigned int mode, unsigned int *color_id)
{
    int retval;
    quadlet_t value;
   
    if ( (mode > MODE_FORMAT7_MAX) || (mode < MODE_FORMAT7_MIN) )
    {
        return DC1394_FAILURE;
    }

    retval= GetCameraFormat7Register(handle, node, mode,
				     REG_CAMERA_FORMAT7_COLOR_CODING_ID,
				     &value);
    value=value>>24;
    if (retval==DC1394_SUCCESS) *color_id= (unsigned int)value+COLOR_FORMAT7_MIN;

    return retval;
}
 
int
dc1394_query_format7_color_coding(raw1394handle_t handle, nodeid_t node,
 				  unsigned int mode, quadlet_t *value)
{
    int retval;

    if ( (mode > MODE_FORMAT7_MAX) || (mode < MODE_FORMAT7_MIN) )
    {
        return DC1394_FAILURE;
    }

    retval= GetCameraFormat7Register(handle, node, mode,
				     REG_CAMERA_FORMAT7_COLOR_CODING_INQ,
				     value);

    return retval;
}
 
int
dc1394_query_format7_pixel_number(raw1394handle_t handle, nodeid_t node,
 				  unsigned int mode, unsigned int *pixnum)
{
    int retval;
    quadlet_t value;
   
    if ( (mode > MODE_FORMAT7_MAX) || (mode < MODE_FORMAT7_MIN) )
    {
        return DC1394_FAILURE;
    }
    retval= GetCameraFormat7Register(handle, node, mode,
				     REG_CAMERA_FORMAT7_PIXEL_NUMBER_INQ,
				     &value);
    *pixnum= (unsigned int) value;

    return retval;
}
 
int
dc1394_query_format7_total_bytes(raw1394handle_t handle, nodeid_t node,
 				 unsigned int mode, unsigned long long int *total_bytes)
{
    int retval;
    unsigned long long int value_hi, value_lo;
    quadlet_t value;
   
    if ( (mode > MODE_FORMAT7_MAX) || (mode < MODE_FORMAT7_MIN) )
    {
        return DC1394_FAILURE;
    }

    retval= GetCameraFormat7Register(handle, node, mode,
				     REG_CAMERA_FORMAT7_TOTAL_BYTES_HI_INQ,
				     &value);
    value_hi=value;
    if (retval != DC1394_SUCCESS)
      return DC1394_FAILURE;
    
    retval= GetCameraFormat7Register(handle, node, mode,
				     REG_CAMERA_FORMAT7_TOTAL_BYTES_LO_INQ,
				     &value);
    value_lo=value;
    
    *total_bytes= (value_lo | ( value_hi << 32) ); 

    return retval;
}
 
int
dc1394_query_format7_packet_para(raw1394handle_t handle, nodeid_t node,
 				 unsigned int mode, unsigned int *min_bytes,
				 unsigned int *max_bytes)
{
    int retval;
    quadlet_t value;

    if ( (mode > MODE_FORMAT7_MAX) || (mode < MODE_FORMAT7_MIN) )
    {
        return DC1394_FAILURE;
    }

    retval= GetCameraFormat7Register(handle, node, mode,
				     REG_CAMERA_FORMAT7_PACKET_PARA_INQ,
				     &value);
    *min_bytes= (unsigned int) ( value & 0xFFFF0000UL ) >> 16;
    *max_bytes= (unsigned int) ( value & 0x0000FFFFUL );       

    return retval;
}
 
int
dc1394_query_format7_byte_per_packet(raw1394handle_t handle, nodeid_t node,
 				     unsigned int mode,
                                     unsigned int *packet_bytes)
{
    int retval;
    quadlet_t value;
   
    if ( (mode > MODE_FORMAT7_MAX) || (mode < MODE_FORMAT7_MIN) )
    {
        return DC1394_FAILURE;
    }

    retval= GetCameraFormat7Register(handle, node, mode,
				     REG_CAMERA_FORMAT7_BYTE_PER_PACKET,
				     &value);
    *packet_bytes= (unsigned int) ( value & 0xFFFF0000UL ) >> 16;

    if (packet_bytes==0) {
      printf("(%s): BYTES_PER_PACKET is zero. This should not happen.\n", __FILE__);
      return DC1394_FAILURE;
    }

    return retval;
}
 
int
dc1394_set_format7_image_position(raw1394handle_t handle, nodeid_t node,
 				  unsigned int mode, unsigned int left,
 				  unsigned int top)
{
    if (SetCameraFormat7Register(handle, node, mode, REG_CAMERA_FORMAT7_IMAGE_POSITION,
                                         (quadlet_t)((left << 16) | top)) != DC1394_SUCCESS)
      {
	printf("(%s) Format7 image position setting failure \n", __FILE__);
	return DC1394_FAILURE;
      }
    else
      // IIDC v1.30 handshaking:
      return _dc1394_v130_handshake(handle, node, mode);
}
 
int
dc1394_set_format7_image_size(raw1394handle_t handle, nodeid_t node,
                              unsigned int mode, unsigned int width,
                              unsigned int height)
{
    if (SetCameraFormat7Register(handle, node, mode, REG_CAMERA_FORMAT7_IMAGE_SIZE,
				 (quadlet_t)((width << 16) | height)) != DC1394_SUCCESS)
      {
	printf("(%s) Format7 image size setting failure \n", __FILE__);
	return DC1394_FAILURE;
      }
    else
      // IIDC v1.30 handshaking:
      return _dc1394_v130_handshake(handle, node, mode);
}
 
int
dc1394_set_format7_color_coding_id(raw1394handle_t handle, nodeid_t node,
 				   unsigned int mode, unsigned int color_id)
{
    if ( (color_id < COLOR_FORMAT7_MIN) || (color_id > COLOR_FORMAT7_MAX) )
    {
        return DC1394_FAILURE;
    }

    color_id-= COLOR_FORMAT7_MIN;
    color_id=color_id<<24;
    if (SetCameraFormat7Register(handle, node, mode,REG_CAMERA_FORMAT7_COLOR_CODING_ID,
				 (quadlet_t)color_id) != DC1394_SUCCESS)
      {
	printf("(%s) Format7 color coding ID setting failure \n", __FILE__);
	return DC1394_FAILURE;
      }
    else
      // IIDC v1.30 handshaking:
      return _dc1394_v130_handshake(handle, node, mode);
}
 
int
dc1394_set_format7_byte_per_packet(raw1394handle_t handle, nodeid_t node,
 				   unsigned int mode,
                                   unsigned int packet_bytes)
{
    if (SetCameraFormat7Register(handle, node, mode, REG_CAMERA_FORMAT7_BYTE_PER_PACKET,
				 (quadlet_t)(packet_bytes) << 16 ) != DC1394_SUCCESS)
      {
	printf("(%s) Format7 bytes-per-packet setting failure \n", __FILE__);
	return DC1394_FAILURE;
      }
    else
      // IIDC v1.30 error checking:
      return _dc1394_v130_errflag2(handle, node, mode);
}

int
dc1394_query_format7_value_setting(raw1394handle_t handle, nodeid_t node,
				   unsigned int mode,
				   unsigned int *present,
				   unsigned int *setting1,
				   unsigned int *err_flag1,
				   unsigned int *err_flag2)
{
    int retval;
    quadlet_t value;
   
    dc1394_camerahandle *camera;
    camera = (dc1394_camerahandle*) raw1394_get_userdata( handle );
   
    if (camera->sw_version>=IIDC_VERSION_1_30) {

      if ( (mode > MODE_FORMAT7_MAX) || (mode < MODE_FORMAT7_MIN) )
	{
	  return DC1394_FAILURE;
	}

      retval= GetCameraFormat7Register(handle, node, mode,
				       REG_CAMERA_FORMAT7_VALUE_SETTING,
				       &value);
      *present= (unsigned int) ( value & 0x80000000UL ) >> 31;
      *setting1= (unsigned int) ( value & 0x40000000UL ) >> 30;
      *err_flag1= (unsigned int) ( value & 0x00800000UL ) >> 23;
      *err_flag2= (unsigned int) ( value & 0x00400000UL ) >> 22;
    }
    else {
      *present=0;
      return DC1394_SUCCESS;
    }

    return retval;
}

int
dc1394_set_format7_value_setting(raw1394handle_t handle, nodeid_t node,
				 unsigned int mode)
{
    return SetCameraFormat7Register(handle, node, mode,
                                     REG_CAMERA_FORMAT7_VALUE_SETTING,
                                     (quadlet_t)0x40000000UL);
}
 
int
dc1394_query_format7_recommended_byte_per_packet(raw1394handle_t handle, nodeid_t node,
						  unsigned int mode,
						  unsigned int *bpp)
{
    int retval;
    quadlet_t value;
   
    if ( (mode > MODE_FORMAT7_MAX) || (mode < MODE_FORMAT7_MIN) )
    {
        return DC1394_FAILURE;
    }

    retval= GetCameraFormat7Register(handle, node, mode,
				     REG_CAMERA_FORMAT7_BYTE_PER_PACKET,
				     &value);
    *bpp= (unsigned int) ( value & 0x0000FFFFUL );

    return retval;
}

int
dc1394_query_format7_packet_per_frame(raw1394handle_t handle, nodeid_t node,
				      unsigned int mode,
				      unsigned int *ppf)
{
    int retval;
    quadlet_t value;
    unsigned int packet_bytes;
    unsigned long long int total_bytes;

    dc1394_camerahandle *camera;
    camera = (dc1394_camerahandle*) raw1394_get_userdata( handle );
   
    if (camera->sw_version>=IIDC_VERSION_1_30) {

      if ( (mode > MODE_FORMAT7_MAX) || (mode < MODE_FORMAT7_MIN) )
	{
	  return DC1394_FAILURE;
	}

      retval= GetCameraFormat7Register(handle, node, mode,
				       REG_CAMERA_FORMAT7_PACKET_PER_FRAME_INQ,
				       &value);
      *ppf= (unsigned int) (value);

      return retval;
    }
    else {
      // return an estimate, NOT TAKING ANY PADDING INTO ACCOUNT
      if (dc1394_query_format7_byte_per_packet(handle, node, mode, &packet_bytes)!=DC1394_SUCCESS) {
	return DC1394_FAILURE;
      }
      if (packet_bytes==0) {
	return DC1394_FAILURE;
      }
      if (dc1394_query_format7_total_bytes(handle, node, mode, &total_bytes)!=DC1394_SUCCESS) {
	return DC1394_FAILURE;
      }
      if (total_bytes%packet_bytes!=0)
	*ppf=total_bytes/packet_bytes+1;
      else
	*ppf=total_bytes/packet_bytes;

      return DC1394_SUCCESS;
    }

}

int
dc1394_query_format7_unit_position(raw1394handle_t handle, nodeid_t node,
				   unsigned int mode,
				   unsigned int *horizontal_pos,
				   unsigned int *vertical_pos)
{
    int retval;
    quadlet_t value;
   
    dc1394_camerahandle *camera;
    camera = (dc1394_camerahandle*) raw1394_get_userdata( handle );
   
    if ( (mode > MODE_FORMAT7_MAX) || (mode < MODE_FORMAT7_MIN) )
      {
	return DC1394_FAILURE;
      }

    if (camera->sw_version>=IIDC_VERSION_1_30) {
      retval= GetCameraFormat7Register(handle, node, mode,
				       REG_CAMERA_FORMAT7_UNIT_POSITION_INQ,
				       &value);
    }
    else {
      // if version is not 1.30, use the UNIT_SIZE_INQ register
      retval= GetCameraFormat7Register(handle, node, mode,
				       REG_CAMERA_FORMAT7_UNIT_SIZE_INQ,
				       &value);
    }
    
    *horizontal_pos = (unsigned int) (( value & 0xFFFF0000UL )>>16);
    *vertical_pos   = (unsigned int) ( value & 0x0000FFFFUL );

    return retval;
}

int
dc1394_query_format7_frame_interval(raw1394handle_t handle, nodeid_t node,
				    unsigned int mode,
				    float *interval)
{   
  quadlet_t value;

  if ( (mode > MODE_FORMAT7_MAX) || (mode < MODE_FORMAT7_MIN) )
    return DC1394_FAILURE;
  
  if (GetCameraFormat7Register(handle, node, mode, REG_CAMERA_FORMAT7_FRAME_INTERVAL_INQ,
			       &value)==DC1394_FAILURE)
    return DC1394_FAILURE;
  
  *interval=value;
  
  return DC1394_SUCCESS;
}   
    
int
dc1394_query_format7_data_depth(raw1394handle_t handle, nodeid_t node,
				unsigned int mode,
				unsigned int *data_depth)
{   
  quadlet_t value;

  if ( (mode > MODE_FORMAT7_MAX) || (mode < MODE_FORMAT7_MIN) )
    return DC1394_FAILURE;
  
  if (GetCameraFormat7Register(handle, node, mode, REG_CAMERA_FORMAT7_DATA_DEPTH_INQ,
			       &value)==DC1394_FAILURE)
    return DC1394_FAILURE;
  
  *data_depth=value >> 24;
  return DC1394_SUCCESS;
}   
    
int
dc1394_query_format7_color_filter_id(raw1394handle_t handle, nodeid_t node,
				     unsigned int mode,
				     unsigned int *color_id)
{   
  quadlet_t value;
  
  if ( (mode > MODE_FORMAT7_MAX) || (mode < MODE_FORMAT7_MIN) )
    return DC1394_FAILURE;
  
  if (GetCameraFormat7Register(handle, node, mode, REG_CAMERA_FORMAT7_COLOR_FILTER_ID,
			       &value)==DC1394_FAILURE)
    return DC1394_FAILURE;
  
  *color_id= (value >> 24)+COLOR_FORMAT7_MIN;
  return DC1394_SUCCESS;
}   
     
int
dc1394_set_format7_color_filter_id(raw1394handle_t handle, nodeid_t node,
				   unsigned int mode,
				   unsigned int color_id)
{   

  if ( (mode > MODE_FORMAT7_MAX) || (mode < MODE_FORMAT7_MIN) )
    return DC1394_FAILURE;
  
  if (SetCameraFormat7Register(handle, node, mode, REG_CAMERA_FORMAT7_COLOR_FILTER_ID,
			       color_id - COLOR_FORMAT7_MIN)==DC1394_FAILURE)
    return DC1394_FAILURE;
  
  return DC1394_SUCCESS;
}   
