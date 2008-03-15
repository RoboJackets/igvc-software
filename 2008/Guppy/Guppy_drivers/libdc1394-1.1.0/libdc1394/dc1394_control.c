/*
 * 1394-Based Digital Camera Control Library
 * Copyright (C) 2000 SMART Technologies Inc.
 *
 * Written by Gord Peters <GordPeters@smarttech.com>
 * Additions by Chris Urmson <curmson@ri.cmu.edu>
 * Additions by Damien Douxchamps <douxchamps@ieee.org>
 *
 * Acknowledgments:
 * Per Dalgas Jakobsen <pdj@maridan.dk>
 *   - Added retries to ROM and CSR reads
 *   - Nicer endianness handling
 *
 * Robert Ficklin <rficklin@westengineering.com>
 *   - bug fixes
 * 
 * Julie Andreotti <JulieAndreotti@smarttech.com>
 *   - bug fixes
 * 
 * Ann Dang <AnnDang@smarttech.com>
 *   - bug fixes
 *
 * Dan Dennedy <dan@dennedy.org>
 *  - bug fixes
 *
 * Yves Jaeger <yves.jaeger@fraenz-jaeger.de>
 *  - software trigger functions
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
#include "dc1394_internal.h"
#include "kernel-video1394.h"


/********************/
/* Base ROM offsets */ 
/********************/

#define ROM_BUS_INFO_BLOCK             0x400U
#define ROM_ROOT_DIRECTORY             0x414U
#define CSR_CONFIG_ROM_END             0x800U

/**********************************/
/* Configuration Register Offsets */
/**********************************/

/* See the 1394-Based Digital Camera Spec. for definitions of these */
#define REG_CAMERA_INITIALIZE          0x000U
#define REG_CAMERA_V_FORMAT_INQ        0x100U
#define REG_CAMERA_V_MODE_INQ_BASE     0x180U
#define REG_CAMERA_V_RATE_INQ_BASE     0x200U
#define REG_CAMERA_V_REV_INQ_BASE      0x2C0U
#define REG_CAMERA_BASIC_FUNC_INQ      0x400U
#define REG_CAMERA_FEATURE_HI_INQ      0x404U
#define REG_CAMERA_FEATURE_LO_INQ      0x408U
#define REG_CAMERA_ADV_FEATURE_INQ     0x480U
#define REG_CAMERA_FEATURE_HI_BASE_INQ 0x500U
#define REG_CAMERA_FEATURE_LO_BASE_INQ 0x580U
#define REG_CAMERA_FRAME_RATE          0x600U
#define REG_CAMERA_VIDEO_MODE          0x604U
#define REG_CAMERA_VIDEO_FORMAT        0x608U
#define REG_CAMERA_ISO_DATA            0x60CU
#define REG_CAMERA_POWER               0x610U
#define REG_CAMERA_ISO_EN              0x614U
#define REG_CAMERA_MEMORY_SAVE         0x618U
#define REG_CAMERA_ONE_SHOT            0x61CU
#define REG_CAMERA_MEM_SAVE_CH         0x620U
#define REG_CAMERA_CUR_MEM_CH          0x624U
#define REG_CAMERA_SOFT_TRIGGER        0x62CU
#define REG_CAMERA_DATA_DEPTH          0x630U

#define REG_CAMERA_FEATURE_HI_BASE     0x800U
#define REG_CAMERA_FEATURE_LO_BASE     0x880U

#define REG_CAMERA_BRIGHTNESS          0x800U
#define REG_CAMERA_EXPOSURE            0x804U
#define REG_CAMERA_SHARPNESS           0x808U
#define REG_CAMERA_WHITE_BALANCE       0x80CU
#define REG_CAMERA_HUE                 0x810U
#define REG_CAMERA_SATURATION          0x814U
#define REG_CAMERA_GAMMA               0x818U
#define REG_CAMERA_SHUTTER             0x81CU
#define REG_CAMERA_GAIN                0x820U
#define REG_CAMERA_IRIS                0x824U
#define REG_CAMERA_FOCUS               0x828U
#define REG_CAMERA_TEMPERATURE         0x82CU
#define REG_CAMERA_TRIGGER_MODE        0x830U
#define REG_CAMERA_TRIGGER_DELAY       0x834U
#define REG_CAMERA_WHITE_SHADING       0x838U
#define REG_CAMERA_FRAME_RATE_FEATURE  0x83CU
#define REG_CAMERA_ZOOM                0x880U
#define REG_CAMERA_PAN                 0x884U
#define REG_CAMERA_TILT                0x888U
#define REG_CAMERA_OPTICAL_FILTER      0x88CU
#define REG_CAMERA_CAPTURE_SIZE        0x8C0U
#define REG_CAMERA_CAPTURE_QUALITY     0x8C4U 


/***********************/
/*  Macro definitions  */
/***********************/

#define FEATURE_TO_VALUE_OFFSET(feature, offset)                      \
                                                                      \
    if ( (feature > FEATURE_MAX) || (feature < FEATURE_MIN) )         \
    {                                                                 \
        return DC1394_FAILURE;                                        \
    }                                                                 \
    else if (feature < FEATURE_ZOOM)                                  \
    {                                                                 \
        offset= REG_CAMERA_FEATURE_HI_BASE;                           \
        feature-= FEATURE_MIN;                                        \
    }                                                                 \
    else                                                              \
    {                                                                 \
        offset= REG_CAMERA_FEATURE_LO_BASE;                           \
                                                                      \
        if (feature >= FEATURE_CAPTURE_SIZE)                          \
        {                                                             \
            feature+= 12;                                             \
        }                                                             \
        feature-= FEATURE_ZOOM;                                       \
                                                                      \
    }                                                                 \
                                                                      \
    offset+= feature * 0x04U;


#define FEATURE_TO_INQUIRY_OFFSET(feature, offset)                    \
                                                                      \
    if ( (feature > FEATURE_MAX) || (feature < FEATURE_MIN) )         \
    {                                                                 \
        return DC1394_FAILURE;                                        \
    }                                                                 \
    else if (feature < FEATURE_ZOOM)                                  \
    {                                                                 \
        offset= REG_CAMERA_FEATURE_HI_BASE_INQ;                       \
        feature-= FEATURE_MIN;                                        \
    }                                                                 \
    else                                                              \
    {                                                                 \
        offset= REG_CAMERA_FEATURE_LO_BASE_INQ;                       \
                                                                      \
        if (feature >= FEATURE_CAPTURE_SIZE)                          \
        {                                                             \
            feature+= 12;                                             \
        }                                                             \
        feature-= FEATURE_ZOOM;                                       \
                                                                      \
    }                                                                 \
                                                                      \
    offset+= feature * 0x04U;

/**************************/
/*  Constant definitions  */
/**************************/

const char *dc1394_feature_desc[NUM_FEATURES] =
{
    "Brightness",
    "Exposure",
    "Sharpness",
    "White Balance",
    "Hue",
    "Saturation",
    "Gamma",
    "Shutter",
    "Gain",
    "Iris",
    "Focus",
    "Temperature",
    "Trigger",
    "Zoom",
    "Pan",
    "Tilt",
    "Optical Filter",
    "Capture Size",
    "Capture Quality"
};

/*
  These arrays define how many image quadlets there
  are in a packet given a mode and a frame rate
  This is defined in the 1394 digital camera spec 
*/
const int quadlets_per_packet_format_0[56] = 
{
  -1,  -1,  15,  30,  60,  120,  240,  480,
  10,  20,  40,  80, 160,  320,  640, 1280,
  30,  60, 120, 240, 480,  960, 1920, 3840,
  40,  80, 160, 320, 640, 1280, 2560, 5120,
  60, 120, 240, 480, 960, 1920, 3840, 7680,
  20,  40,  80, 160, 320,  640, 1280, 2560,
  40,  80, 160, 320, 640, 1280, 2560, 5120
};
 
const int quadlets_per_packet_format_1[64] = 
{
   -1, 125, 250,  500, 1000, 2000, 4000, 8000,
   -1,  -1, 375,  750, 1500, 3000, 6000,   -1,
   -1,  -1, 125,  250,  500, 1000, 2000, 4000,
   96, 192, 384,  768, 1536, 3072, 6144,   -1,
  144, 288, 576, 1152, 2304, 4608,   -1,   -1,
   48,  96, 192,  384,  768, 1536, 3073, 6144,
   -1, 125, 250,  500, 1000, 2000, 4000, 8000,
   96, 192, 384,  768, 1536, 3072, 6144,   -1
};
 
const int quadlets_per_packet_format_2[64] = 
{
  160, 320,  640, 1280, 2560, 5120,   -1,  -1,
  240, 480,  960, 1920, 3840, 7680,   -1,  -1,
   80, 160,  320,  640, 1280, 2560, 5120,  -1,
  250, 500, 1000, 2000, 4000, 8000,   -1,  -1,
  375, 750, 1500, 3000, 6000,   -1,   -1,  -1,
  125, 250,  500, 1000, 2000, 4000, 8000,  -1,
  160, 320,  640, 1280, 2560, 5120,   -1,  -1,
  250, 500, 1000, 2000, 4000, 8000,   -1,  -1
};
   

/**********************/
/* Internal functions */
/**********************/

int
_dc1394_get_wh_from_format(int format, int mode, int *w, int *h) 
{

    switch(format) 
    {
    case FORMAT_VGA_NONCOMPRESSED:
        switch(mode) 
        {
        case MODE_160x120_YUV444:
            *w = 160;*h=120;
            return DC1394_SUCCESS;
        case MODE_320x240_YUV422:
            *w = 320;*h=240;
            return DC1394_SUCCESS;
        case MODE_640x480_YUV411:
        case MODE_640x480_YUV422:
        case MODE_640x480_RGB:
        case MODE_640x480_MONO:
        case MODE_640x480_MONO16:
            *w =640;*h=480;
            return DC1394_SUCCESS;
        default:
            return DC1394_FAILURE;
        }
    case FORMAT_SVGA_NONCOMPRESSED_1:
        switch(mode) 
        {
        case MODE_800x600_YUV422:
        case MODE_800x600_RGB:
        case MODE_800x600_MONO:
        case MODE_800x600_MONO16:
            *w=800;*h=600;
            return DC1394_SUCCESS;
        case MODE_1024x768_YUV422:
        case MODE_1024x768_RGB:
        case MODE_1024x768_MONO:
        case MODE_1024x768_MONO16:
            *w=1024;*h=768;
            return DC1394_SUCCESS;
        default:
            return DC1394_FAILURE;
        }
    case FORMAT_SVGA_NONCOMPRESSED_2:
        switch(mode) 
        {
        case MODE_1280x960_YUV422:
        case MODE_1280x960_RGB:
        case MODE_1280x960_MONO:
        case MODE_1280x960_MONO16:
            *w=1280;*h=960;
            return DC1394_SUCCESS;
        case MODE_1600x1200_YUV422:
        case MODE_1600x1200_RGB:
        case MODE_1600x1200_MONO:
        case MODE_1600x1200_MONO16:
            *w=1600;*h=1200;
            return DC1394_SUCCESS;
        default:
            return DC1394_FAILURE;
        }
    default:
        return DC1394_FAILURE;
    }

}
	
/********************************************************
 _dc1394_get_quadlets_per_packet

 This routine reports the number of useful image quadlets 
 per packet
*********************************************************/
int 
_dc1394_get_quadlets_per_packet(int format, int mode, int frame_rate) 
{
    int mode_index, frame_rate_index= frame_rate - FRAMERATE_MIN;

    switch(format) 
    {
    case FORMAT_VGA_NONCOMPRESSED:
        mode_index= mode - MODE_FORMAT0_MIN;

        if ( ((mode >= MODE_FORMAT0_MIN) && (mode <= MODE_FORMAT0_MAX)) && 
             ((frame_rate >= FRAMERATE_MIN) && (frame_rate <= FRAMERATE_MAX)) )
        {
            return quadlets_per_packet_format_0[NUM_FRAMERATES*mode_index+frame_rate_index];
        }
        else
        {
            printf("(%s) Invalid framerate (%d) or mode (%d)!\n", __FILE__,
                   frame_rate, format);
        }

        break;
    case FORMAT_SVGA_NONCOMPRESSED_1:
        mode_index= mode - MODE_FORMAT1_MIN;

        if ( ((mode >= MODE_FORMAT1_MIN) && (mode <= MODE_FORMAT1_MAX)) && 
             ((frame_rate >= FRAMERATE_MIN) && (frame_rate <= FRAMERATE_MAX)) )
        {
            return quadlets_per_packet_format_1[NUM_FRAMERATES*mode_index+frame_rate_index];
        }
        else
        {
            printf("(%s) Invalid framerate (%d) or mode (%d)!\n", __FILE__,
                   frame_rate, format);
        }

        break;
    case FORMAT_SVGA_NONCOMPRESSED_2:
        mode_index= mode - MODE_FORMAT2_MIN;

        if ( ((mode >= MODE_FORMAT2_MIN) && (mode <= MODE_FORMAT2_MAX)) && 
             ((frame_rate >= FRAMERATE_MIN) && (frame_rate <= FRAMERATE_MAX)) )
        {
            return quadlets_per_packet_format_2[NUM_FRAMERATES*mode_index+frame_rate_index];
        }
        else
        {
            printf("(%s) Invalid framerate (%d) or mode (%d)!\n", __FILE__,
                   frame_rate, format);
        }

        break;
    default:
        printf("(%s) Quadlets per packet unkown for format %d!\n",
               __FILE__, format);
        break;
    }

    return -1;
}

/**********************************************************
 _dc1394_quadlets_from_format

 This routine reports the number of quadlets that make up a 
 frame given the format and mode
***********************************************************/
int
_dc1394_quadlets_from_format(int format, int mode) 
{

    switch (format) 
    {
    case FORMAT_VGA_NONCOMPRESSED:

        switch(mode) 
        {
        case MODE_160x120_YUV444:
            return 14400;   //160x120*3/4
        case MODE_320x240_YUV422:
            return 38400;   //320x240/2
        case MODE_640x480_YUV411:
            return 115200;  //640x480x1.5/4
        case MODE_640x480_YUV422:
            return 153600;  //640x480/2
        case MODE_640x480_RGB:
            return 230400;  //640x480x3/4
        case MODE_640x480_MONO:
            return 76800;   //640x480/4
        case MODE_640x480_MONO16:
            return 153600;  //640x480/2
        default:
            printf("(%s) Improper mode specified: %d\n", __FILE__, mode);
            break;
        }

        break;
    case FORMAT_SVGA_NONCOMPRESSED_1: 

        switch(mode) 
        {
        case MODE_800x600_YUV422:
            return 240000;  //800x600/2
        case MODE_800x600_RGB:
            return 360000;  //800x600x3/4
        case MODE_800x600_MONO:
            return 120000;  //800x600/4
        case MODE_1024x768_YUV422:
            return 393216;  //1024x768/2
        case MODE_1024x768_RGB:
            return 589824;  //1024x768x3/4
        case MODE_1024x768_MONO:
            return 196608;  //1024x768/4
        case MODE_800x600_MONO16:
            return 240000;  //800x600/2
        case MODE_1024x768_MONO16:
            return 393216;  //1024x768/2
        default:
            printf("(%s) Improper mode specified: %d\n", __FILE__, mode);
            break;
        }

        break;
    case FORMAT_SVGA_NONCOMPRESSED_2:

        switch (mode) 
        {
        case MODE_1280x960_YUV422:
            return 614400;  //1280x960/2
        case MODE_1280x960_RGB:
            return 921600;  //1280x960x3/4
        case MODE_1280x960_MONO:
            return 307200;  //1280x960/4
        case MODE_1600x1200_YUV422:
            return 960000;  //1600x1200/2
        case MODE_1600x1200_RGB:
            return 1440000; //1600x1200x3/4
        case MODE_1600x1200_MONO:
            return 480000;  //1600x1200/4
        case MODE_1280x960_MONO16:
            return 614400;  //1280x960/2
        case MODE_1600x1200_MONO16:
            return 960000;  //1600x1200/2
        default:
            printf("(%s) Improper mode specified: %d\n", __FILE__, mode);
            break;
        }

        break;
    case FORMAT_STILL_IMAGE:
        printf("(%s) Don't know how many quadlets per frame for "
               "FORMAT_STILL_IMAGE mode:%d\n", __FILE__, mode);
        break;
    case FORMAT_SCALABLE_IMAGE_SIZE:
        printf("(%s) Don't know how many quadlets per frame for "
               "FORMAT_SCALABLE_IMAGE mode:%d\n", __FILE__, mode);
        break;
    default:
        printf("(%s) Improper format specified: %d\n", __FILE__, format);
        break;
    }

    return -1;
}

static int
GetCameraROMValue(raw1394handle_t handle, nodeid_t node,
                  octlet_t offset, quadlet_t *value) {
    int retval, retry= MAX_RETRIES;

    /* retry a few times if necessary (addition by PDJ) */
    while(retry--) 
    {
        retval= raw1394_read(handle, 0xffc0 | node, CONFIG_ROM_BASE + offset,
                             4, value);

#ifdef LIBRAW1394_OLD
        if (retval >= 0)
        {
            int ack= retval >> 16;
            int rcode= retval & 0xffff;

#ifdef SHOW_ERRORS
            printf("ROM read ack of %x rcode of %x\n", ack, rcode);
#endif

            if ( ((ack == ACK_PENDING) || (ack == ACK_LOCAL)) &&
                 (rcode == RESP_COMPLETE) )
            { 
                /* conditionally byte swap the value */
                *value= ntohl(*value); 
                return 0;
            }

        }
#else
        if (!retval)
        {
            /* conditionally byte swap the value */
            *value= ntohl(*value);
            return retval;
        }
        else if (errno != EAGAIN)
        {
            return retval;
        }
#endif /* LIBRAW1394_VERSION <= 0.8.2 */

        usleep(SLOW_DOWN);
    }
    
    *value= ntohl(*value);
    return retval;
}

int
GetCameraControlRegister(raw1394handle_t handle, nodeid_t node,
                         octlet_t offset, quadlet_t *value)
{
    int retval, retry= MAX_RETRIES;
    dc1394_camerahandle *camera;
    camera = (dc1394_camerahandle*) raw1394_get_userdata( handle );

    /* get the ccr_base address if not yet retrieved */
    if (camera != NULL && camera->ccr_base == 0)
    {
        dc1394_camerainfo info;
        if ( dc1394_get_camera_info(handle, node, &info) != DC1394_SUCCESS )
            return -1;
    }
    else if (camera == NULL)
        return -1;

    /* retry a few times if necessary (addition by PDJ) */
    while(retry--) 
    {
        retval= raw1394_read(handle, 0xffc0 | node, camera->ccr_base + offset,
                             4, value);

#ifdef LIBRAW1394_OLD
        if (retval >= 0)
        {
            int ack= retval >> 16;
            int rcode= retval & 0xffff;

#ifdef SHOW_ERRORS
            printf("CCR read ack of %x rcode of %x\n", ack, rcode);
#endif

            if ( ((ack == ACK_PENDING) || (ack == ACK_LOCAL)) &&
                 (rcode == RESP_COMPLETE) )
            { 
                /* conditionally byte swap the value */
                *value= ntohl(*value); 
                return 0;
            }

        }
#else
        if (!retval)
        {
            /* conditionally byte swap the value (addition by PDJ) */
            *value= ntohl(*value);  
            return retval; 
        }
        else if (errno != EAGAIN)
        {
            return retval;
        }
#endif /* LIBRAW1394_VERSION <= 0.8.2 */

        usleep(SLOW_DOWN);
    }
    
    *value= ntohl(*value);
    return retval;
}

int
SetCameraControlRegister(raw1394handle_t handle, nodeid_t node,
                         octlet_t offset, quadlet_t value)
{
    int retval, retry= MAX_RETRIES;
    dc1394_camerahandle *camera;
    camera = (dc1394_camerahandle*) raw1394_get_userdata( handle );

    /* get the ccr_base address if not yet retrieved */
    if (camera != NULL && camera->ccr_base == 0)
    {
        dc1394_camerainfo info;
        if ( dc1394_get_camera_info(handle, node, &info) != DC1394_SUCCESS )
            return -1;
    }
    else if (camera == NULL)
        return -1;

    /* conditionally byte swap the value (addition by PDJ) */
    value= htonl(value);

    /* retry a few times if necessary */
    while(retry--)
    {
        retval= raw1394_write(handle, 0xffc0 | node, camera->ccr_base + offset, 4,
                              &value);

#ifdef LIBRAW1394_OLD
        if (retval >= 0)
        {
            int ack= retval >> 16;
            int rcode= retval & 0xffff;

#ifdef SHOW_ERRORS
            printf("CCR write ack of %x rcode of %x\n", ack, rcode);
#endif

            if ( ((ack == ACK_PENDING) || (ack == ACK_LOCAL) ||
                  (ack == ACK_COMPLETE)) &&
                 ((rcode == RESP_COMPLETE) || (rcode == RESP_SONY_HACK)) ) 
            {
                return 0;
            }
            
            
        }
#else
        if (!retval || (errno != EAGAIN))
        {
            return retval;
        }
#endif /* LIBRAW1394_VERSION <= 0.8.2 */

        usleep(SLOW_DOWN);
    }

    return retval;
}

static int
IsFeatureBitSet(quadlet_t value, unsigned int feature)
{

    if (feature >= FEATURE_ZOOM)
    {
      if (feature >= FEATURE_CAPTURE_SIZE) {
	feature+= 12;
      }
        feature-= FEATURE_ZOOM;
    }
    else
    {
        feature-= FEATURE_MIN;
    }

    value&=(0x80000000UL >> feature);

    if (value>0)
      return DC1394_TRUE;
    else
      return DC1394_FALSE;
}

static int 
SetFeatureValue(raw1394handle_t handle, nodeid_t node,
                unsigned int feature, unsigned int value)
{
    quadlet_t curval;
    octlet_t offset;
    int retval;

    FEATURE_TO_VALUE_OFFSET(feature, offset);

    if (GetCameraControlRegister(handle, node, offset, &curval) < 0)
    {
        return DC1394_FAILURE;
    }

    retval= SetCameraControlRegister(handle, node, offset,
                                    (curval & 0xFFFFF000UL) |
                                     (value & 0xFFFUL));
    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

static int
GetFeatureValue(raw1394handle_t handle, nodeid_t node,
                unsigned int feature, unsigned int *value)
{
    quadlet_t quadval;
    octlet_t offset;
    int retval;

    FEATURE_TO_VALUE_OFFSET(feature, offset);

    if (!(retval= GetCameraControlRegister(handle, node, offset, &quadval)))
    {
        *value= (unsigned int)(quadval & 0xFFFUL);
    }

    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

static int
GetConfigROMTaggedRegister(raw1394handle_t handle, nodeid_t node,
			   unsigned int tag, octlet_t *offset, quadlet_t *value)
{
  unsigned int block_length;
  int i;

  //fprintf(stderr,"tag\n");
  // get the block length
  if (GetCameraROMValue(handle,node,*offset,value)<0) {
    //fprintf(stderr,"rom probe failed for tag 0x%x\n",tag);
    return DC1394_FAILURE;
  }
  
  //fprintf(stderr,"tag\n");
  block_length=*value>>16;

  if (*offset+block_length*4>CSR_CONFIG_ROM_END) {
    block_length=(CSR_CONFIG_ROM_END-*offset)/4;
  }

  // find the tag and return the result
  for (i=0;i<block_length;i++) {
    *offset+=4;
    if (GetCameraROMValue(handle,node,*offset,value)<0) {
      //fprintf(stderr,"rom probe failed for tag 0x%x\n",tag);
      return DC1394_FAILURE;
    }
    else {
      if ((*value>>24)==tag) {
	//fprintf(stderr,"got tag 0x%x\n",tag);
	return DC1394_SUCCESS;
      }
    }
  }

  //fprintf(stderr,"failed for tag 0x%x\n",tag);
  return DC1394_FAILURE;

}


/**********************/
/* External functions */
/**********************/

nodeid_t* 
dc1394_get_camera_nodes(raw1394handle_t handle, int *numCameras,
                        int showCameras) 
{
    nodeid_t * nodes; 
    dc1394bool_t isCamera;
    int numNodes;
    int i;  
    dc1394_camerainfo caminfo;

    numNodes= raw1394_get_nodecount(handle);
    *numCameras= 0;

    /* we know that the computer isn't a camera so there are only
       numNodes-1 possible camera nodes */
    nodes=(nodeid_t*)calloc(numNodes - 1, sizeof(nodeid_t));

    for (i= 0; i < (numNodes - 1); i++) 
    {
        nodes[i]= DC1394_NO_CAMERA;
    }

    for (i= 0; i < numNodes; i++) 
    {
      dc1394_is_camera(handle, i, &isCamera);

        if (isCamera) 
	  {
            nodes[*numCameras]= i;
            (*numCameras)++;

            if (showCameras)
            {

                if (dc1394_get_camera_info(handle, i, &caminfo)
                    == DC1394_SUCCESS) 
                {
                    dc1394_print_camera_info(&caminfo);
                }
                else 
                {
                    printf("Couldn't get camera info (%d)!\n", i);
                }

            }

        }
        /*
	  else 
        {
            printf("node %d is not a camera\n", i);
        }
	*/
    }

    return nodes; 
}

/**********************************************************************
 dc1394_get_camera_nodes

 This returns the available cameras on the bus.

 It returns the node id's in the same index as the id specified
 the ids array contains a list of the low quadlet of the unique camera 
 ids.
 Returns -1 in numCameras and NULL from the call if there is a problem, 
 otherwise the number of cameras and the nodeid_t array from the call
***********************************************************************/
nodeid_t* 
dc1394_get_sorted_camera_nodes(raw1394handle_t handle,int numIds, 
                               int *ids, int *numCameras,
                               int showCameras) 
{
    int numNodes, i,j, uid, foundId, extraId;
    dc1394bool_t isCamera;
    nodeid_t *nodes;
    dc1394_camerainfo caminfo;

    *numCameras= 0;
    numNodes= raw1394_get_nodecount(handle);

    /* we know that the computer isn't a camera so there are only
       numNodes-1 possible camera nodes */
    nodes= (nodeid_t*)calloc(numNodes - 1, sizeof(nodeid_t));

    for (i= 0; i < (numNodes - 1); i++) 
    {
      nodes[i]= DC1394_NO_CAMERA;
   }

    extraId= numIds;

    for (i= 0; i < numNodes; i++) 
    {
        dc1394_is_camera(handle, i, &isCamera);

        if (isCamera)
        {
            (*numCameras)++;
            dc1394_get_camera_info(handle, i, &caminfo);

            if (showCameras) dc1394_print_camera_info(&caminfo);

            uid= caminfo.euid_64 & 0xffffffff;
            foundId= 0;

            for (j= 0; j < numIds; j++) 
            {

                if (ids[j] == uid) 
                {
                    nodes[j]= i;
                    foundId= 1;
                    break;
                }

            }

            /* if it isn't then we need to put it in one of the extra
               spaces- but check first to make sure we aren't overflowing
               our bounds */
            if (foundId == 0) 
            {

                if (extraId >= (numNodes-1)) 
                {
                    *numCameras= -1;
                    free(nodes);
                    return NULL;
                }

                nodes[extraId++]= i;
            }

        }

    }

    return nodes;
}



/*****************************************************
 dc1394_create_handle

 This creates a raw1394_handle
 If a handle can't be created, it returns NULL
*****************************************************/
raw1394handle_t 
dc1394_create_handle(int port) 
{
    raw1394handle_t handle;
    int i;
    dc1394_camerahandle *camera = malloc(sizeof(dc1394_camerahandle));
    memset(camera, 0, sizeof(dc1394_camerahandle));

#ifdef LIBRAW1394_OLD
    if (!(handle= raw1394_get_handle()))
#else
    if (!(handle= raw1394_new_handle()))
#endif
    {
        printf("(%s) Couldn't get raw1394 handle!\n",__FILE__);
        return NULL;
    }

    if (raw1394_set_port(handle, port) < 0) 
    {

        if (handle != NULL)
            raw1394_destroy_handle(handle);

        printf("(%s) Couldn't raw1394_set_port!\n",__FILE__);
        return NULL;
    }

    camera->port = port;
    camera->ccr_base=0;
    camera->sw_version=0;

    for (i=0;i<NUM_MODE_FORMAT7;i++) {
      camera->format7_csr[i]=0;
    }
    raw1394_set_userdata( handle, (void*) camera );

    return handle;
}

int
dc1394_destroy_handle( raw1394handle_t handle )
{
    dc1394_camerahandle *camera;

    camera = (dc1394_camerahandle*) raw1394_get_userdata( handle );
    if(camera) 
        free(camera);
    if( handle != NULL )
        raw1394_destroy_handle(handle);

    return DC1394_SUCCESS;
}

int
dc1394_is_camera(raw1394handle_t handle, nodeid_t node, dc1394bool_t *value)
{
    octlet_t offset;
    octlet_t ud_offset = 0;
    quadlet_t quadval = 0;
    dc1394bool_t ptgrey;

    /* Note on Point Grey  (PG) cameras:
       Although not advertised, PG cameras are 'sometimes' compatible with
       IIDC specs. The following modifications have been tested with a stereo
       head, the BumbleBee. More cameras should be compatible, please consider
       contributing to the lib if your PG camera is not recognized.

       PG cams have a Unit_Spec_ID of 0xB09D, instead of the 0xA02D of classic
       IIDC cameras. Also, their software revision differs. I could only
       get a 1.14 version from my BumbleBee, other versions might exist.

       Damien
     */

    /* get the unit_directory offset */
    offset= ROM_ROOT_DIRECTORY;
    if (GetConfigROMTaggedRegister(handle, node, 0xD1, &offset, &quadval)!=DC1394_SUCCESS) {
      *value= DC1394_FALSE;
      return DC1394_FAILURE;
    }
    else {
      ud_offset=(quadval & 0xFFFFFFUL)*4+offset;
    }
  
    /* get the unit_spec_ID (should be 0x00A02D for 1394 digital camera) */
    offset=ud_offset;
    if (GetConfigROMTaggedRegister(handle, node, 0x12, &offset, &quadval)!=DC1394_SUCCESS) {
      *value= DC1394_FALSE;
      return DC1394_FAILURE;
    }
    else {
      quadval&=0xFFFFFFUL;
    }

    ptgrey=(quadval == 0x00B09DUL);
  
    if ( ! ( (quadval == 0x00A02DUL) || ptgrey) )
    {
        *value= DC1394_FALSE;
        return DC1394_SUCCESS;
    }

    quadval = 0;
    /* get the unit_sw_version (should be 0x000100 - 0x000102 for 1394 digital camera) */
    /* DRD> without this check, AV/C cameras show up as well */
    offset = ud_offset;
    if (GetConfigROMTaggedRegister(handle, node, 0x13, &offset, &quadval)!=DC1394_SUCCESS) {
      *value= DC1394_FALSE;
      return DC1394_FAILURE;
    }
    else {
      quadval&=0xFFFFFFUL;
    }
    //fprintf(stderr,"0x%x\n",quadval);

    if ((quadval == 0x000100UL) || 
        (quadval == 0x000101UL) ||
        (quadval == 0x000102UL) ||
        ((quadval == 0x000114UL) && ptgrey) ||
        ((quadval == 0x800002UL) && ptgrey))
    {
        *value= DC1394_TRUE;
    }
    else
    {
        *value= DC1394_FALSE;
    }

    return DC1394_SUCCESS;
}

int
dc1394_get_sw_version(raw1394handle_t handle, nodeid_t node, int *version)
{
  dc1394_camerahandle *camera;
  octlet_t offset;
  octlet_t ud_offset = 0;
  octlet_t udd_offset = 0;
  quadlet_t quadval = 0;

  camera = (dc1394_camerahandle*) raw1394_get_userdata( handle );  
  if (camera != NULL && camera->sw_version > 0) {
    *version = camera->sw_version;
  }
  else {
    
    /* get the unit_directory offset */
    offset= ROM_ROOT_DIRECTORY;
    if (GetConfigROMTaggedRegister(handle, node, 0xD1, &offset, &quadval)!=DC1394_SUCCESS) {
      *version= -1;
      return DC1394_FAILURE;
    }
    else {
      ud_offset=(quadval & 0xFFFFFFUL)*4+offset;
    }
      
    /* get the unit_sw_version  */
    offset = ud_offset;
    if (GetConfigROMTaggedRegister(handle, node, 0x13, &offset, &quadval)!=DC1394_SUCCESS) {
      *version = -1;
      return DC1394_FAILURE;
    }
    else {
      switch (quadval&0xFFFFFFUL) {
      case 0x100:
	*version=IIDC_VERSION_1_04;
	break;
      case 0x101:
	*version=IIDC_VERSION_1_20;
	break;
      case 0x102:
	*version=IIDC_VERSION_1_30;
	break;
      case 0x114:
      case 0x800002:
	/* get the unit_spec_ID (should be 0x00A02D for 1394 digital camera) */
	if (GetConfigROMTaggedRegister(handle, node, 0x12, &ud_offset, &quadval)!=DC1394_SUCCESS) {
	  *version = -1;
	  return DC1394_FAILURE;
	}
	else {
	  quadval&=0xFFFFFFUL;
	}
	if (quadval == 0x00B09DUL)
	  *version=IIDC_VERSION_PTGREY;
	else
	  *version = -1;
	break;
      }
    }
  }
  //fprintf(stderr,"sw version is %d\n",*version);
  /*IIDC 1.31 check*/
  if (*version==IIDC_VERSION_1_30) {
    
    /* get the unit_dependent_directory offset */
    offset= ud_offset;
    if (GetConfigROMTaggedRegister(handle, node, 0xD4, &offset, &quadval)!=DC1394_SUCCESS) {
      *version = -1;
      return DC1394_FAILURE;
    }
    else {
      udd_offset=(quadval & 0xFFFFFFUL)*4+offset;
    }
    
    if (GetConfigROMTaggedRegister(handle, node, 0x38, &udd_offset, &quadval)!=DC1394_SUCCESS) {
      // if it fails here we return success with the most accurate version estimation: 1.30.
      // this is because the GetConfigROMTaggedRegister will return a failure both if there is a comm
      // problem but also if the tag is not found. In the latter case it simply means that the
      // camera version is 1.30
      *version=IIDC_VERSION_1_30;
      return DC1394_SUCCESS;
    }
    else {
      //fprintf(stderr,"1.3x register is 0x%x\n",quadval);
      switch (quadval&0xFFFFFFUL) {
      case 0x10:
	*version=IIDC_VERSION_1_31;
	break;
      case 0x20:
	*version=IIDC_VERSION_1_32;
	break;
      case 0x30:
	*version=IIDC_VERSION_1_33;
	break;
      case 0x40:
	*version=IIDC_VERSION_1_34;
	break;
      case 0x50:
	*version=IIDC_VERSION_1_35;
	break;
      case 0x60:
	*version=IIDC_VERSION_1_36;
	break;
      case 0x70:
	*version=IIDC_VERSION_1_37;
	break;
      case 0x80:
	*version=IIDC_VERSION_1_38;
	break;
      case 0x90:
	*version=IIDC_VERSION_1_39;
	break;
      default:
	*version=IIDC_VERSION_1_30;
	break;
      }
    }
  }
  camera->sw_version=*version;
  
  return DC1394_SUCCESS;
}

void
dc1394_print_camera_info(dc1394_camerainfo *info) 
{
    quadlet_t value[2];

    value[0]= info->euid_64 & 0xffffffff;
    value[1]= (info->euid_64 >>32) & 0xffffffff;
    printf("CAMERA INFO\n===============\n");
    printf("Node: %x\n", info->id);
    printf("CCR_Offset: %Lux\n", info->ccr_offset);
    //L added by tim evers 
    printf("UID: 0x%08x%08x\n", value[1], value[0]);
    printf("Vendor: %s\tModel: %s\n\n", info->vendor, info->model);
    fflush(stdout);
}

int
dc1394_get_camera_info(raw1394handle_t handle, nodeid_t node,
                       dc1394_camerainfo *info)
{
    dc1394bool_t iscamera;
    int retval, len;
    octlet_t offset;
    quadlet_t value[2], quadval;
    unsigned int count;
    octlet_t ud_offset, udd_offset;
    dc1394_camerahandle *camera;
    camera = (dc1394_camerahandle*) raw1394_get_userdata( handle );

    if ( (retval= dc1394_is_camera(handle, node, &iscamera)) !=
         DC1394_SUCCESS )
    {
#ifdef SHOW_ERRORS
        printf("Error - this is not a camera (get_camera_info)\n");
#endif
        return DC1394_FAILURE;
    }
    else if (iscamera != DC1394_TRUE)
    {
        return DC1394_FAILURE;
    }

    info->handle= handle;
    info->id= node;

    /* now get the EUID-64 */
    if (GetCameraROMValue(handle, node, ROM_BUS_INFO_BLOCK+0x0C, &value[0]) < 0)
    {
        return DC1394_FAILURE;
    }

    if (GetCameraROMValue(handle, node, ROM_BUS_INFO_BLOCK+0x10, &value[1]) < 0)
    {
        return DC1394_FAILURE;
    }

    info->euid_64= ((u_int64_t)value[0] << 32) | (u_int64_t)value[1];

    /* get the unit_directory offset */
    offset= ROM_ROOT_DIRECTORY;
    if (GetConfigROMTaggedRegister(handle, node, 0xD1, &offset, &quadval)!=DC1394_SUCCESS) {
      return DC1394_FAILURE;
    }
    else {
      ud_offset=(quadval & 0xFFFFFFUL)*4+offset;
    }

    /* get the unit_dependent_directory offset */
    offset= ud_offset;
    if (GetConfigROMTaggedRegister(handle, node, 0xD4, &offset, &quadval)!=DC1394_SUCCESS) {
      return DC1394_FAILURE;
    }
    else {
      udd_offset=(quadval & 0xFFFFFFUL)*4+offset;
    }

    /* now get the command_regs_base */
    offset= udd_offset;
    if (GetConfigROMTaggedRegister(handle, node, 0x40, &offset, &quadval)!=DC1394_SUCCESS) {
      return DC1394_FAILURE;
    }
    else {
      info->ccr_offset= (octlet_t)(quadval & 0xFFFFFFUL)*4;
        if (camera != NULL)
          camera->ccr_base = CONFIG_ROM_BASE + info->ccr_offset;
    }

    /* get the vendor_name_leaf offset (optional) */
    offset= udd_offset;
    info->vendor[0] = '\0';
    if (GetConfigROMTaggedRegister(handle, node, 0x81, &offset, &quadval)==DC1394_SUCCESS) {
      offset=(quadval & 0xFFFFFFUL)*4+offset;

      /* read in the length of the vendor name */
      if (GetCameraROMValue(handle, node, offset, &value[0]) < 0)
      {
        return DC1394_FAILURE;
      }

      len= (int)(value[0] >> 16)*4-8; /* Tim Evers corrected length value */ 

      if (len > MAX_CHARS)
      {
          len= MAX_CHARS;
      }
      offset+= 12;
      count= 0;

      /* grab the vendor name */
      while (len > 0)
      {
        if (GetCameraROMValue(handle, node, offset+count, &value[0]) < 0)
        {
            return DC1394_FAILURE;
        }
        info->vendor[count++]= (value[0] >> 24);
        info->vendor[count++]= (value[0] >> 16) & 0xFFUL;
        info->vendor[count++]= (value[0] >> 8) & 0xFFUL;
        info->vendor[count++]= value[0] & 0xFFUL;
        len-= 4;
      }
      info->vendor[count]= '\0';
    }

    /* get the model_name_leaf offset (optional) */
    offset= udd_offset;
    info->model[0] = '\0';
    if (GetConfigROMTaggedRegister(handle, node, 0x82, &offset, &quadval)==DC1394_SUCCESS) {
      offset=(quadval & 0xFFFFFFUL)*4+offset;

      /* read in the length of the model name */
      if (GetCameraROMValue(handle, node, offset, &value[0]) < 0)
      {
        return DC1394_FAILURE;
      }

      len= (int)(value[0] >> 16)*4-8; /* Tim Evers corrected length value */ 

      if (len > MAX_CHARS)
      {
          len= MAX_CHARS;
      }
      offset+= 12;
      count= 0;

      /* grab the model name */
      while (len > 0)
      {
        if (GetCameraROMValue(handle, node, offset+count, &value[0]) < 0)
        {
            return DC1394_FAILURE;
        }
        info->model[count++]= (value[0] >> 24);
        info->model[count++]= (value[0] >> 16) & 0xFFUL;
        info->model[count++]= (value[0] >> 8) & 0xFFUL;
        info->model[count++]= value[0] & 0xFFUL;
        len-= 4;
      }
      info->model[count]= '\0';
    }

    return DC1394_SUCCESS;
}

/*****************************************************
 dc1394_get_camera_misc_info

 Collects other camera info registers
*****************************************************/
int
dc1394_get_camera_misc_info(raw1394handle_t handle, nodeid_t node,
                            dc1394_miscinfo *info)
{
    quadlet_t value;

    if (dc1394_get_iso_channel_and_speed(handle, node,
                                         &info->iso_channel, 
                                         &info->iso_speed)!= DC1394_SUCCESS)
        return DC1394_FAILURE;

    if (dc1394_get_video_format(handle, node, &info->format) != DC1394_SUCCESS)
        return DC1394_FAILURE;

    if (dc1394_get_video_mode(handle, node, &info->mode) != DC1394_SUCCESS)
        return DC1394_FAILURE;

    if (dc1394_get_video_framerate(handle, node, &info->framerate)
        != DC1394_SUCCESS)
        return DC1394_FAILURE;

    if (dc1394_get_iso_status(handle, node, &info->is_iso_on)
        != DC1394_SUCCESS)
        return DC1394_FAILURE;

    if (dc1394_query_basic_functionality(handle,node,&value) != DC1394_SUCCESS)
        return DC1394_FAILURE;
    else {
      info->mem_channel_number = (value & 0x0000000F);
      info->bmode_capable      = (value & 0x00800000) != 0;
      info->one_shot_capable   = (value & 0x00000800) != 0;
      info->multi_shot_capable = (value & 0x00001000) != 0;
    }

    if (info->mem_channel_number>0) {
      if (dc1394_get_memory_load_ch(handle, node, &info->load_channel)
	  != DC1394_SUCCESS)
        return DC1394_FAILURE;
      
      if (dc1394_get_memory_save_ch(handle, node, &info->save_channel)
	  != DC1394_SUCCESS)
        return DC1394_FAILURE;
    }
    else {
      info->load_channel=0;
      info->save_channel=0;
    }

    return DC1394_SUCCESS;
} 

/*****************************************************
 dc1394_get_camera_feature_set

 Collects the available features for the camera
 described by node and stores them in features.
*****************************************************/
int
dc1394_get_camera_feature_set(raw1394handle_t handle, nodeid_t node,
                              dc1394_feature_set *features) 
{
    int i, j;

    for (i= FEATURE_MIN, j= 0; i <= FEATURE_MAX; i++, j++) 
    {
        features->feature[j].feature_id= i;
        dc1394_get_camera_feature(handle, node, &features->feature[j]);
    }

    return DC1394_SUCCESS;
}

/*****************************************************
 dc1394_get_camera_feature

 Stores the bounds and options associated with the
 feature described by feature->feature_id
*****************************************************/
int
dc1394_get_camera_feature(raw1394handle_t handle, nodeid_t node,
                          dc1394_feature_info *feature) 
{
    octlet_t offset;
    quadlet_t value;
    unsigned int orig_fid, updated_fid;

    orig_fid= feature->feature_id;
    updated_fid= feature->feature_id;
    

    // check presence
    if (dc1394_is_feature_present(handle, node, feature->feature_id, &(feature->available))!=DC1394_SUCCESS) {
      return DC1394_FAILURE;
    }

    if (feature->available == DC1394_FALSE)
    {
        return DC1394_SUCCESS;
    }

    // get capabilities
    if (dc1394_query_feature_characteristics(handle, node, feature->feature_id, &value)!=DC1394_SUCCESS) {
      return DC1394_FAILURE;
    }

    switch (feature->feature_id) {
    case FEATURE_TRIGGER:
      feature->one_push= DC1394_FALSE;
      feature->polarity_capable=
	(value & 0x02000000UL) ? DC1394_TRUE : DC1394_FALSE;
      feature->trigger_mode_capable_mask= ((value >> 12) & 0x0f);
      feature->auto_capable= DC1394_FALSE;
      feature->manual_capable= DC1394_FALSE;
      break;
    default:
      feature->polarity_capable= 0;
      feature->trigger_mode= 0;
      feature->one_push= (value & 0x10000000UL) ? DC1394_TRUE : DC1394_FALSE;
      feature->auto_capable=
	(value & 0x02000000UL) ? DC1394_TRUE : DC1394_FALSE;
      feature->manual_capable=
	(value & 0x01000000UL) ? DC1394_TRUE : DC1394_FALSE;
      
      feature->min= (value & 0xFFF000UL) >> 12;
      feature->max= (value & 0xFFFUL);
      break;
    }

    feature->absolute_capable=
        (value & 0x40000000UL) ? DC1394_TRUE : DC1394_FALSE;
    feature->readout_capable=
        (value & 0x08000000UL) ? DC1394_TRUE : DC1394_FALSE;
    feature->on_off_capable=
        (value & 0x04000000UL) ? DC1394_TRUE : DC1394_FALSE;

    // get current values
    updated_fid= orig_fid;
    FEATURE_TO_VALUE_OFFSET(updated_fid, offset);

    if (GetCameraControlRegister(handle, node, offset, &value) < 0) {
      return DC1394_FAILURE;
    }

    switch (feature->feature_id) {
    case FEATURE_TRIGGER:
        feature->one_push_active= DC1394_FALSE;
        feature->trigger_polarity=
            (value & 0x01000000UL) ? DC1394_TRUE : DC1394_FALSE;
        feature->trigger_mode= (int)((value >> 14) & 0xF);
        feature->auto_active= DC1394_FALSE;
	break;
    case FEATURE_TRIGGER_DELAY:
        feature->one_push_active= DC1394_FALSE;
	feature->auto_active=DC1394_FALSE;
    default:
        feature->one_push_active=
            (value & 0x04000000UL) ? DC1394_TRUE : DC1394_FALSE;
        feature->auto_active=
            (value & 0x01000000UL) ? DC1394_TRUE : DC1394_FALSE;
        feature->trigger_polarity= DC1394_FALSE;
	break;
    }

    feature->is_on= (value & 0x02000000UL) ? DC1394_TRUE : DC1394_FALSE;

    switch (orig_fid) {
    case FEATURE_WHITE_BALANCE:
      feature->RV_value= value & 0xFFFUL;
      feature->BU_value= (value & 0xFFF000UL) >> 12;
      break;
    case FEATURE_WHITE_SHADING:
      feature->R_value=value & 0xFFUL;
      feature->G_value=(value & 0xFF00UL)>>8;
      feature->B_value=(value & 0xFF0000UL)>>16;
      break;
    case FEATURE_TEMPERATURE:
      feature->value= value & 0xFFFUL;
      feature->target_value= value & 0xFFF000UL;
      break;
    default:
      feature->value= value & 0xFFFUL;
      break;
    }

    if (feature->absolute_capable>0) {
      dc1394_query_absolute_feature_min_max(handle, node, orig_fid,
					    &feature->abs_min, &feature->abs_max);
      dc1394_query_absolute_feature_value(handle, node, orig_fid,
					  &feature->abs_value);
      dc1394_query_absolute_control(handle, node, orig_fid,
				    &feature->abs_control);
    }

    return DC1394_SUCCESS;
}

/*****************************************************
 dc1394_print_feature

 Displays the bounds and options of the given feature
*****************************************************/
void
dc1394_print_feature(dc1394_feature_info *f) 
{
    //char * tags[2] = {"NO","YES"};
    //char * autoMode[2] = {"Manual","Auto"};
    int fid= f->feature_id;

    if ( (fid < FEATURE_MIN) || (fid > FEATURE_MAX) )
    {
        printf("Invalid feature code\n");
        return;
    }

    printf("%s:\n\t", dc1394_feature_desc[fid - FEATURE_MIN]);

    if (!f->available) 
    {
        printf("NOT AVAILABLE\n");
        return;
    }

    if (f->one_push) 
        printf("OP  ");
    if (f->readout_capable)
        printf("RC  ");
    if (f->on_off_capable)
        printf("O/OC  ");
    if (f->auto_capable)
        printf("AC  ");
    if (f->manual_capable)
        printf("MC  ");
    if (f->absolute_capable)
        printf("ABS  ");
    printf("\n");

    if (f->on_off_capable) 
    {
        if (f->is_on) 
            printf("\tFeature: ON  ");
        else
            printf("\tFeature: OFF  ");
    }
    else 
    {
        printf("\t");
    }

    if (f->one_push) 
    {
        if (f->one_push_active)
            printf("One push: ACTIVE  ");
        else
            printf("One push: INACTIVE  ");
    }

    if (f->auto_active) 
        printf("AUTO  ");
    else
        printf("MANUAL ");

    if (fid != FEATURE_TRIGGER) 
    {
        printf("min: %d max %d\n", f->min, f->max);
    }
    switch(fid) {
    case FEATURE_TRIGGER:
      printf("\n\tAvailableTriggerModes: ");
      
      if (f->trigger_mode_capable_mask & 0x08)
	printf("0 ");
      if (f->trigger_mode_capable_mask & 0x04)
	printf("1 ");
      if (f->trigger_mode_capable_mask & 0x02)
	printf("2 ");
      if (f->trigger_mode_capable_mask & 0x02)
	printf("3 ");
      if (!(f->trigger_mode_capable_mask & 0x0f))
	printf("No modes available");
      
      printf("\n\tPolarity Change Capable: ");
      
      if (f->polarity_capable) 
	printf("True");
      else 
	printf("False");
      
      printf("\n\tCurrent Polarity: ");
      
      if (f->trigger_polarity) 
	printf("POS");
      else 
	printf("NEG");
      
      printf("\n\tcurrent mode: %d\n", f->trigger_mode);
      break;
    case FEATURE_WHITE_BALANCE: 
      printf("\tB/U value: %d R/V value: %d\n", f->BU_value, f->RV_value);
      break;
    case FEATURE_TEMPERATURE:
      printf("\tTarget temp: %d Current Temp: %d\n", f->target_value, f->value);
      break;
    case FEATURE_WHITE_SHADING: 
      printf("\tR value: %d G value: %d B value: %d\n", f->R_value,
	     f->G_value, f->B_value);
      break;
    default:
      printf("\tcurrent value is: %d\n",f->value);
      break;
    }
    if (f->absolute_capable)
      printf("\tabsolute settings:\n\t value: %f\n\t min: %f\n\t max: %f\n",
         f->abs_value,f->abs_min,f->abs_max);
}

/*****************************************************
 dc1394_print_feature_set

 Displays the entire feature set stored in features
*****************************************************/
void
dc1394_print_feature_set(dc1394_feature_set *features) 
{
    int i, j;

    printf("FEATURE SETTINGS\n==================================\n");
    printf("OP- one push capable\n");
    printf("RC- readout capable\n");
    printf("O/OC- on/off capable\n");
    printf("AC- auto capable\n");
    printf("MC- manual capable\n");
    printf("ABS- absolute capable\n");
    printf("==================================\n");

    for (i= FEATURE_MIN, j= 0; i <= FEATURE_MAX; i++, j++) 
    {
        dc1394_print_feature(&features->feature[j]);
    }

    printf("==================================\n");
}

int
dc1394_init_camera(raw1394handle_t handle, nodeid_t node)
{
    int retval= SetCameraControlRegister(handle, node, REG_CAMERA_INITIALIZE,
                                         ON_VALUE);
    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_query_supported_formats(raw1394handle_t handle, nodeid_t node,
                               quadlet_t *value)
{
    int retval= GetCameraControlRegister(handle, node, REG_CAMERA_V_FORMAT_INQ,
                                         value);
    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_query_supported_modes(raw1394handle_t handle, nodeid_t node,
                             unsigned int format, quadlet_t *value)
{
    int retval;

    if ( (format > FORMAT_MAX) || (format < FORMAT_MIN) )
    {
        return DC1394_FAILURE;
    }

    format-= FORMAT_MIN;
    retval= GetCameraControlRegister(handle, node,
                                     REG_CAMERA_V_MODE_INQ_BASE +
                                     (format * 0x04U), value);
    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_query_supported_framerates(raw1394handle_t handle, nodeid_t node,
                                  unsigned int format, unsigned int mode,
                                  quadlet_t *value)
{
    int retval;
    int max_mode_for_format, min_mode_for_format;

    switch(format)
    {
    case FORMAT_VGA_NONCOMPRESSED:
        min_mode_for_format= MODE_FORMAT0_MIN;
        max_mode_for_format= MODE_FORMAT0_MAX;
        break;
    case FORMAT_SVGA_NONCOMPRESSED_1:
        min_mode_for_format= MODE_FORMAT1_MIN;
        max_mode_for_format= MODE_FORMAT1_MAX;
        break;
    case FORMAT_SVGA_NONCOMPRESSED_2:
        min_mode_for_format= MODE_FORMAT2_MIN;
        max_mode_for_format= MODE_FORMAT2_MAX;
        break;
    default:
        printf("Invalid format query\n");
        return DC1394_FAILURE;
    }

    if ( (format > FORMAT_MAX) || (format < FORMAT_MIN) ||
         (mode > max_mode_for_format) || (mode < min_mode_for_format) )
    {
        return DC1394_FAILURE;
    }

    format-= FORMAT_MIN;
    mode-= min_mode_for_format;
    retval= GetCameraControlRegister(handle, node,
                                     REG_CAMERA_V_RATE_INQ_BASE +
                                     (format * 0x20U) + (mode * 0x04U), value);
    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_query_revision(raw1394handle_t handle, nodeid_t node, int mode,
                      quadlet_t *value)
{
    int retval;

    if ( (mode > MODE_FORMAT6_MAX) || (mode < MODE_FORMAT6_MIN) )
    {
        return DC1394_FAILURE;
    }

    mode-= MODE_FORMAT6_MIN;
    retval= GetCameraControlRegister(handle, node,
                                     REG_CAMERA_V_REV_INQ_BASE +
                                     (mode * 0x04U), value);
    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_query_basic_functionality(raw1394handle_t handle, nodeid_t node,
                                 quadlet_t *value)
{
    int retval= GetCameraControlRegister(handle, node,
                                         REG_CAMERA_BASIC_FUNC_INQ, value);
    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_query_advanced_feature_offset(raw1394handle_t handle, nodeid_t node,
                                     quadlet_t *value)
{
    int retval= GetCameraControlRegister(handle, node,
                                         REG_CAMERA_ADV_FEATURE_INQ, value);
    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_query_feature_characteristics(raw1394handle_t handle, nodeid_t node,
                                     unsigned int feature, quadlet_t *value)
{
    octlet_t offset;
    int retval;

    FEATURE_TO_INQUIRY_OFFSET(feature, offset);
    retval= GetCameraControlRegister(handle, node, offset, value);
    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_get_video_framerate(raw1394handle_t handle, nodeid_t node,
                           unsigned int *framerate)
{
    quadlet_t value;
    int retval= GetCameraControlRegister(handle, node, REG_CAMERA_FRAME_RATE,
                                         &value);

    if (!retval) {
        *framerate= (unsigned int)((value >> 29) & 0x7UL) + FRAMERATE_MIN;
    }

    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_set_video_framerate(raw1394handle_t handle, nodeid_t node,
                           unsigned int framerate)
{
    int retval;

    if ( (framerate < FRAMERATE_MIN) || (framerate > FRAMERATE_MAX) ) {
        return DC1394_FAILURE;
    }

    retval= SetCameraControlRegister(handle, node, REG_CAMERA_FRAME_RATE,
                     (quadlet_t)(((framerate - FRAMERATE_MIN) & 0x7UL) << 29));
    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_get_video_mode(raw1394handle_t handle, nodeid_t node,
                      unsigned int *mode)
{
    quadlet_t value;
    int retval;
    unsigned int format;

    if (dc1394_get_video_format(handle, node, &format) != DC1394_SUCCESS) {
        return DC1394_FAILURE;
    }

    retval= GetCameraControlRegister(handle, node, REG_CAMERA_VIDEO_MODE,
                                     &value);

    if (!retval) {

        switch(format) {
        case FORMAT_VGA_NONCOMPRESSED:
            *mode= (unsigned int)((value >> 29) & 0x7UL) + MODE_FORMAT0_MIN;
            break;
        case FORMAT_SVGA_NONCOMPRESSED_1:
            *mode= (unsigned int)((value >> 29) & 0x7UL) + MODE_FORMAT1_MIN;
            break;
        case FORMAT_SVGA_NONCOMPRESSED_2:
            *mode= (unsigned int)((value >> 29) & 0x7UL) + MODE_FORMAT2_MIN;
            break;
        case FORMAT_STILL_IMAGE:
            *mode= (unsigned int)((value >> 29) & 0x7UL) + MODE_FORMAT6_MIN;
            break;
        case FORMAT_SCALABLE_IMAGE_SIZE:
            *mode= (unsigned int)((value >> 29) & 0x7UL) + MODE_FORMAT7_MIN;
            break;
        default:
            return DC1394_FAILURE;
            break;
        }

    }

    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_set_video_mode(raw1394handle_t handle, nodeid_t node, unsigned int mode)
{
    int retval;
    unsigned int format, min, max;

    if (dc1394_get_video_format(handle, node, &format) != DC1394_SUCCESS) {
        return DC1394_FAILURE;
    }

    switch(format) {
        case FORMAT_VGA_NONCOMPRESSED:
            min= MODE_FORMAT0_MIN;
            max= MODE_FORMAT0_MAX;
            break;
        case FORMAT_SVGA_NONCOMPRESSED_1:
            min= MODE_FORMAT1_MIN;
            max= MODE_FORMAT1_MAX;
            break;
        case FORMAT_SVGA_NONCOMPRESSED_2:
            min= MODE_FORMAT2_MIN;
            max= MODE_FORMAT2_MAX;
            break;
        case FORMAT_STILL_IMAGE:
            min= MODE_FORMAT6_MIN;
            max= MODE_FORMAT6_MAX;
            break;
        case FORMAT_SCALABLE_IMAGE_SIZE:
            min= MODE_FORMAT7_MIN;
            max= MODE_FORMAT7_MAX;
            break;
        default:
            return DC1394_FAILURE;
            break;
    }

    if ( (mode < min) || (mode > max) ) {
        return DC1394_FAILURE;
    }

    retval= SetCameraControlRegister(handle, node, REG_CAMERA_VIDEO_MODE,
                                    (quadlet_t)(((mode - min) & 0x7UL) << 29));

    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_get_video_format(raw1394handle_t handle, nodeid_t node,
                        unsigned int *format)
{
    quadlet_t value;
    int retval= GetCameraControlRegister(handle, node, REG_CAMERA_VIDEO_FORMAT,
                                         &value);

    if (!retval) {
        *format= (unsigned int)((value >> 29) & 0x7UL) + FORMAT_MIN;
    }

    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_set_video_format(raw1394handle_t handle, nodeid_t node,
                        unsigned int format)
{
    int retval;

    if ( (format < FORMAT_MIN) || (format > FORMAT_MAX) ) {
        return DC1394_FAILURE;
    }

    retval= SetCameraControlRegister(handle, node, REG_CAMERA_VIDEO_FORMAT,
                           (quadlet_t)(((format - FORMAT_MIN) & 0x7UL) << 29));
    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}


int
dc1394_get_operation_mode(raw1394handle_t handle, nodeid_t node, unsigned int *mode)
{
  quadlet_t value_inq, value;
  int retval;

  retval= dc1394_query_basic_functionality(handle, node, &value_inq);
  retval= GetCameraControlRegister(handle, node, REG_CAMERA_ISO_DATA, &value);
  
  if (value_inq & 0x00800000) {
    *mode=((value & 0x00008000) >0);
  }
  else {
    *mode=OPERATION_MODE_LEGACY;
  }
  
  return retval;
}


int
dc1394_set_operation_mode(raw1394handle_t handle, nodeid_t node, unsigned int mode)
{
  quadlet_t value_inq, value;
  int retval;
  
  retval= dc1394_query_basic_functionality(handle, node, &value_inq);
  retval= GetCameraControlRegister(handle, node, REG_CAMERA_ISO_DATA, &value);

  if (mode==OPERATION_MODE_LEGACY) {
    retval= SetCameraControlRegister(handle, node, REG_CAMERA_ISO_DATA,
				     (quadlet_t) (value & 0xFFFF7FFF));
  }
  else { // 1394b
    if (value_inq & 0x00800000) { // if 1394b available
      retval= SetCameraControlRegister(handle, node, REG_CAMERA_ISO_DATA,
				       (quadlet_t) (value | 0x00008000));
    }
    else { // 1394b asked, but it is not available
      return DC1394_FAILURE;
    }
  }

  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
  
}


int
dc1394_get_iso_channel_and_speed(raw1394handle_t handle, nodeid_t node,
                                 unsigned int *channel, unsigned int *speed)
{
  quadlet_t value_inq, value;
  int retval;
  
  retval= dc1394_query_basic_functionality(handle, node, &value_inq);
  retval= GetCameraControlRegister(handle, node, REG_CAMERA_ISO_DATA, &value);
  
  if (value_inq & 0x00800000) { // check if 1394b is available
    if (value & 0x00008000) { //check if we are now using 1394b
      *channel= (unsigned int)((value >> 8) & 0x3FUL);
      *speed= (unsigned int)(value& 0x7UL);
    }
    else { // fallback to legacy
      *channel= (unsigned int)((value >> 28) & 0xFUL);
      *speed= (unsigned int)((value >> 24) & 0x3UL);
    }
  }
  else { // legacy
    *channel= (unsigned int)((value >> 28) & 0xFUL);
    *speed= (unsigned int)((value >> 24) & 0x3UL);
  }
  
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_set_iso_channel_and_speed(raw1394handle_t handle, nodeid_t node,
                                 unsigned int channel, unsigned int speed)
{
  quadlet_t value_inq, value;
  int retval;

  retval= dc1394_query_basic_functionality(handle, node, &value_inq);
  retval= GetCameraControlRegister(handle, node, REG_CAMERA_ISO_DATA, &value);
  if ((value_inq & 0x00800000)&&(value & 0x00008000)) {
    // check if 1394b is available and if we are now using 1394b
    retval= SetCameraControlRegister(handle, node, REG_CAMERA_ISO_DATA,
				     (quadlet_t) ( ((channel & 0x3FUL) << 8) |
						   (speed & 0x7UL) |
						   (0x1 << 15) ));
  }
  else { // fallback to legacy
    if (speed>SPEED_400) {
      fprintf(stderr,"(%s) line %d: an ISO speed >400Mbps was requested while the camera is in LEGACY mode\n",__FILE__,__LINE__);
      fprintf(stderr,"              Please set the operation mode to OPERATION_MODE_1394B before asking for\n");
      fprintf(stderr,"              1394b ISO speeds\n");
      return DC1394_FAILURE;
    }
    retval= SetCameraControlRegister(handle, node, REG_CAMERA_ISO_DATA,
				     (quadlet_t) (((channel & 0xFUL) << 28) |
						  ((speed & 0x3UL) << 24) ));
  }
  
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_camera_on(raw1394handle_t handle, nodeid_t node)
{
    int retval= SetCameraControlRegister(handle, node, REG_CAMERA_POWER,
                                         ON_VALUE);
    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_camera_off(raw1394handle_t handle, nodeid_t node)
{
    int retval= SetCameraControlRegister(handle, node, REG_CAMERA_POWER,
                                         OFF_VALUE);
    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_start_iso_transmission(raw1394handle_t handle, nodeid_t node)
{
    int retval= SetCameraControlRegister(handle, node, REG_CAMERA_ISO_EN,
                                         ON_VALUE);
    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_stop_iso_transmission(raw1394handle_t handle, nodeid_t node)
{
    int retval= SetCameraControlRegister(handle, node, REG_CAMERA_ISO_EN,
                                         OFF_VALUE);
    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_get_iso_status(raw1394handle_t handle, nodeid_t node,
                      dc1394bool_t *is_on)
{
    int retval;
    quadlet_t value;
    retval= GetCameraControlRegister(handle, node, REG_CAMERA_ISO_EN,&value);
    *is_on= (value & ON_VALUE)>>31;
    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_set_one_shot(raw1394handle_t handle, nodeid_t node)
{
    int retval= SetCameraControlRegister(handle, node, REG_CAMERA_ONE_SHOT,
                                         ON_VALUE);
    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_unset_one_shot(raw1394handle_t handle, nodeid_t node)
{
    int retval= SetCameraControlRegister(handle, node, REG_CAMERA_ONE_SHOT,
                                         OFF_VALUE);
    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_get_one_shot(raw1394handle_t handle, nodeid_t node, dc1394bool_t *is_on)
{
 quadlet_t value;
 int retval = GetCameraControlRegister(handle, node,
                                          REG_CAMERA_ONE_SHOT, &value);
 *is_on = value & ON_VALUE;
 return (retval ? DC1394_FAILURE : DC1394_SUCCESS);     
}

int
dc1394_get_multi_shot(raw1394handle_t handle, nodeid_t node, dc1394bool_t *is_on,
		      unsigned int *numFrames)
{
 quadlet_t value;
 int retval = GetCameraControlRegister(handle, node,
                                          REG_CAMERA_ONE_SHOT, &value);
 *is_on = (value & (ON_VALUE>>1)) >> 30;
 *numFrames= value & 0xFFFFUL;

 return (retval ? DC1394_FAILURE : DC1394_SUCCESS);     
}

int
dc1394_set_multi_shot(raw1394handle_t handle, nodeid_t node,
                      unsigned int numFrames)
{
    int retval= SetCameraControlRegister(handle, node, REG_CAMERA_ONE_SHOT,
                                      (0x40000000UL | (numFrames & 0xFFFFUL)));
    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_unset_multi_shot(raw1394handle_t handle, nodeid_t node)
{
    return dc1394_unset_one_shot(handle, node);
}

int
dc1394_get_brightness(raw1394handle_t handle, nodeid_t node,
                      unsigned int *brightness)
{
    return GetFeatureValue(handle, node, FEATURE_BRIGHTNESS, brightness);
}

int
dc1394_set_brightness(raw1394handle_t handle, nodeid_t node,
                      unsigned int brightness)
{
    return SetFeatureValue(handle, node, FEATURE_BRIGHTNESS, brightness);
}

int
dc1394_get_exposure(raw1394handle_t handle, nodeid_t node,
                    unsigned int *exposure)
{
    return GetFeatureValue(handle, node, FEATURE_EXPOSURE, exposure);
}

int
dc1394_set_exposure(raw1394handle_t handle, nodeid_t node,
                    unsigned int exposure)
{
    return SetFeatureValue(handle, node, FEATURE_EXPOSURE, exposure);
}

int
dc1394_get_sharpness(raw1394handle_t handle, nodeid_t node,
                     unsigned int *sharpness)
{
    return GetFeatureValue(handle, node, FEATURE_SHARPNESS, sharpness);
}

int
dc1394_set_sharpness(raw1394handle_t handle, nodeid_t node,
                     unsigned int sharpness)
{
    return SetFeatureValue(handle, node, FEATURE_SHARPNESS, sharpness);
}

int
dc1394_get_white_balance(raw1394handle_t handle, nodeid_t node,
                         unsigned int *u_b_value, unsigned int *v_r_value)
{
    quadlet_t value;
    int retval= GetCameraControlRegister(handle, node,
                                         REG_CAMERA_WHITE_BALANCE, &value);

    *u_b_value= (unsigned int)((value & 0xFFF000UL) >> 12);
    *v_r_value= (unsigned int)(value & 0xFFFUL);
    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_set_white_balance(raw1394handle_t handle, nodeid_t node,
                         unsigned int u_b_value, unsigned int v_r_value)
{
    int retval;
    quadlet_t curval;

    if (GetCameraControlRegister(handle, node, REG_CAMERA_WHITE_BALANCE,
                                 &curval) < 0)
    {
        return DC1394_FAILURE;
    }

    curval= (curval & 0xFF000000UL) | ( ((u_b_value & 0xFFFUL) << 12) |
                                        (v_r_value & 0xFFFUL) );
    retval= SetCameraControlRegister(handle, node, REG_CAMERA_WHITE_BALANCE,
                                     curval);
    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_get_hue(raw1394handle_t handle, nodeid_t node,
               unsigned int *hue)
{
    return GetFeatureValue(handle, node, FEATURE_HUE, hue);
}

int
dc1394_set_hue(raw1394handle_t handle, nodeid_t node,
               unsigned int hue)
{
    return SetFeatureValue(handle, node, FEATURE_HUE, hue);
}

int
dc1394_get_saturation(raw1394handle_t handle, nodeid_t node,
                      unsigned int *saturation)
{
    return GetFeatureValue(handle, node, FEATURE_SATURATION, saturation);
}

int
dc1394_set_saturation(raw1394handle_t handle, nodeid_t node,
                      unsigned int saturation)
{
    return SetFeatureValue(handle, node, FEATURE_SATURATION, saturation);
}

int
dc1394_get_gamma(raw1394handle_t handle, nodeid_t node,
                 unsigned int *gamma)
{
    return GetFeatureValue(handle, node, FEATURE_GAMMA, gamma);
}

int
dc1394_set_gamma(raw1394handle_t handle, nodeid_t node,
                 unsigned int gamma)
{
    return SetFeatureValue(handle, node, FEATURE_GAMMA, gamma);
}

int
dc1394_get_shutter(raw1394handle_t handle, nodeid_t node,
                   unsigned int *shutter)
{
    return GetFeatureValue(handle, node, FEATURE_SHUTTER, shutter);
}

int
dc1394_set_shutter(raw1394handle_t handle, nodeid_t node,
                   unsigned int shutter)
{
    return SetFeatureValue(handle, node, FEATURE_SHUTTER, shutter);
}

int
dc1394_get_gain(raw1394handle_t handle, nodeid_t node,
                unsigned int *gain)
{
    return GetFeatureValue(handle, node, FEATURE_GAIN, gain);
}

int
dc1394_set_gain(raw1394handle_t handle, nodeid_t node,
                unsigned int gain)
{
  return SetFeatureValue(handle, node, FEATURE_GAIN, gain);
}

int
dc1394_get_iris(raw1394handle_t handle, nodeid_t node,
                unsigned int *iris)
{
  return GetFeatureValue(handle, node, FEATURE_IRIS, iris);
}

int
dc1394_set_iris(raw1394handle_t handle, nodeid_t node,
                unsigned int iris)
{
  return SetFeatureValue(handle, node, FEATURE_IRIS, iris);
}

int
dc1394_get_focus(raw1394handle_t handle, nodeid_t node,
                 unsigned int *focus)
{
  return GetFeatureValue(handle, node, FEATURE_FOCUS, focus);
}

int
dc1394_set_focus(raw1394handle_t handle, nodeid_t node,
                 unsigned int focus)
{
  return SetFeatureValue(handle, node, FEATURE_FOCUS, focus);
}

int
dc1394_get_temperature(raw1394handle_t handle, nodeid_t node,
                       unsigned int *target_temperature,
                       unsigned int *temperature)
{
  quadlet_t value;
  int retval= GetCameraControlRegister(handle, node,
				       REG_CAMERA_TEMPERATURE, &value);
  *target_temperature= (unsigned int)((value >> 12) & 0xFFF);
  *temperature= (unsigned int)(value & 0xFFFUL);
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_set_temperature(raw1394handle_t handle, nodeid_t node,
                       unsigned int target_temperature)
{
  int retval;
  quadlet_t curval;
  
  if (GetCameraControlRegister(handle, node, REG_CAMERA_TEMPERATURE,
			       &curval) < 0) {
    return DC1394_FAILURE;
  }
  
  curval= (curval & 0xFF000FFFUL) | ((target_temperature & 0xFFFUL) << 12);
  retval= SetCameraControlRegister(handle, node, REG_CAMERA_TEMPERATURE,
				   curval);
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_get_white_shading(raw1394handle_t handle, nodeid_t node,
                         unsigned int *r_value, unsigned int *g_value, unsigned int *b_value)
{
  quadlet_t value;
  int retval= GetCameraControlRegister(handle, node,
				       REG_CAMERA_WHITE_SHADING, &value);
  
  *r_value= (unsigned int)((value & 0xFF0000UL) >> 16);
  *g_value= (unsigned int)((value & 0xFF00UL) >> 8);
  *b_value= (unsigned int)(value & 0xFFUL);
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_set_white_shading(raw1394handle_t handle, nodeid_t node,
			 unsigned int r_value, unsigned int g_value, unsigned int b_value)

{
  int retval;
  quadlet_t curval;
  
  if (GetCameraControlRegister(handle, node, REG_CAMERA_WHITE_SHADING,
			       &curval) < 0)
    return DC1394_FAILURE;
  
  curval= (curval & 0xFF000000UL) | ( ((r_value & 0xFFUL) << 16) |
				      ((g_value & 0xFFUL) << 8) |
				      (b_value & 0xFFUL) );
  retval= SetCameraControlRegister(handle, node, REG_CAMERA_WHITE_SHADING,
				   curval);
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_get_trigger_delay(raw1394handle_t handle, nodeid_t node,
			 unsigned int *trigger_delay)
{
  return GetFeatureValue(handle, node, FEATURE_TRIGGER_DELAY, trigger_delay);
}

int
dc1394_set_trigger_delay(raw1394handle_t handle, nodeid_t node,
			 unsigned int trigger_delay)
{
  return SetFeatureValue(handle, node, FEATURE_TRIGGER_DELAY, trigger_delay);
}

int
dc1394_get_frame_rate(raw1394handle_t handle, nodeid_t node,
		      unsigned int *frame_rate)
{
  return GetFeatureValue(handle, node, FEATURE_FRAME_RATE, frame_rate);
}

int
dc1394_set_frame_rate(raw1394handle_t handle, nodeid_t node,
		      unsigned int frame_rate)
{
  return SetFeatureValue(handle, node, FEATURE_FRAME_RATE, frame_rate);
}

int
dc1394_get_trigger_mode(raw1394handle_t handle, nodeid_t node,
                        unsigned int *mode)
{
  quadlet_t value;
  int retval= GetCameraControlRegister(handle, node,
				       REG_CAMERA_TRIGGER_MODE, &value);
  
  *mode= (unsigned int)( ((value >> 16) & 0xFUL) ) + TRIGGER_MODE_MIN;
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_set_trigger_mode(raw1394handle_t handle, nodeid_t node,
                        unsigned int mode)
{
    int retval;
    quadlet_t curval;

    if ( (mode < TRIGGER_MODE_MIN) || (mode > TRIGGER_MODE_MAX) )
    {
        return DC1394_FAILURE;
    }

    if (GetCameraControlRegister(handle, node, REG_CAMERA_TRIGGER_MODE,
                                 &curval) < 0)
    {
        return DC1394_FAILURE;
    }

    mode-= TRIGGER_MODE_MIN;
    curval= (curval & 0xFFF0FFFFUL) | ((mode & 0xFUL) << 16);
    retval= SetCameraControlRegister(handle, node, REG_CAMERA_TRIGGER_MODE,
                                     curval);
    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_get_zoom(raw1394handle_t handle, nodeid_t node,
                unsigned int *zoom)
{
    return GetFeatureValue(handle, node, FEATURE_ZOOM, zoom);
}

int
dc1394_set_zoom(raw1394handle_t handle, nodeid_t node,
                unsigned int zoom)
{
    return SetFeatureValue(handle, node, FEATURE_ZOOM, zoom);
}

int
dc1394_get_pan(raw1394handle_t handle, nodeid_t node,
               unsigned int *pan)
{
    return GetFeatureValue(handle, node, FEATURE_PAN, pan);
}

int
dc1394_set_pan(raw1394handle_t handle, nodeid_t node,
               unsigned int pan)
{
    return SetFeatureValue(handle, node, FEATURE_PAN, pan);
}

int
dc1394_get_tilt(raw1394handle_t handle, nodeid_t node,
                unsigned int *tilt)
{
    return GetFeatureValue(handle, node, FEATURE_TILT, tilt);
}

int
dc1394_set_tilt(raw1394handle_t handle, nodeid_t node,
                unsigned int tilt)
{
    return SetFeatureValue(handle, node, FEATURE_TILT, tilt);
}

int
dc1394_get_optical_filter(raw1394handle_t handle, nodeid_t node,
                          unsigned int *optical_filter)
{
    return GetFeatureValue(handle, node, FEATURE_OPTICAL_FILTER,
                                    optical_filter);
}

int
dc1394_set_optical_filter(raw1394handle_t handle, nodeid_t node,
                          unsigned int optical_filter)
{
    return SetFeatureValue(handle, node, FEATURE_OPTICAL_FILTER,
                           optical_filter);
}

int
dc1394_get_capture_size(raw1394handle_t handle, nodeid_t node,
                        unsigned int *capture_size)
{
    return GetFeatureValue(handle, node, FEATURE_CAPTURE_SIZE,
                                    capture_size);
}

int
dc1394_set_capture_size(raw1394handle_t handle, nodeid_t node,
                        unsigned int capture_size)
{
    return SetFeatureValue(handle, node, FEATURE_CAPTURE_SIZE,
                           capture_size);
}

int
dc1394_get_capture_quality(raw1394handle_t handle, nodeid_t node,
                           unsigned int *capture_quality)
{
    return GetFeatureValue(handle, node, FEATURE_CAPTURE_QUALITY,
                                    capture_quality);
}

int
dc1394_set_capture_quality(raw1394handle_t handle, nodeid_t node,
                           unsigned int capture_quality)
{
    return SetFeatureValue(handle, node, FEATURE_CAPTURE_QUALITY,
                           capture_quality);
}

int
dc1394_get_feature_value(raw1394handle_t handle, nodeid_t node,
                         unsigned int feature, unsigned int *value)
{
    return GetFeatureValue(handle, node, feature, value);
}

int
dc1394_set_feature_value(raw1394handle_t handle, nodeid_t node,
                         unsigned int feature, unsigned int value)
{
    return SetFeatureValue(handle, node, feature, value);
}

int
dc1394_is_feature_present(raw1394handle_t handle, nodeid_t node,
                          unsigned int feature, dc1394bool_t *value)
{
    octlet_t offset;
    quadlet_t quadval;

    // check feature presence in 0x404 and 0x408
    if ( (feature > FEATURE_MAX) || (feature < FEATURE_MIN) )
    {
      return DC1394_FAILURE;
    }
    else if (feature < FEATURE_ZOOM)
    {
        offset= REG_CAMERA_FEATURE_HI_INQ;
    }
    else
    {
        offset= REG_CAMERA_FEATURE_LO_INQ;
    }
    if (GetCameraControlRegister(handle, node, offset, &quadval) < 0)
    {
      return DC1394_FAILURE;
    }

    if (IsFeatureBitSet(quadval, feature)!=DC1394_TRUE) {
      *value=DC1394_FALSE;
      return DC1394_SUCCESS;
    }

    FEATURE_TO_INQUIRY_OFFSET(feature, offset);
    
    if (GetCameraControlRegister(handle, node, offset, &quadval) < 0)
      {
	*value=DC1394_FALSE;
	return DC1394_FAILURE;
      }
    
    if (quadval & 0x80000000UL)
      {
	*value= DC1394_TRUE;
      }
    else
      {
	*value= DC1394_FALSE;
      }
    
    return DC1394_SUCCESS;
}

int
dc1394_has_one_push_auto(raw1394handle_t handle, nodeid_t node,
                         unsigned int feature, dc1394bool_t *value)
{
    octlet_t offset;
    quadlet_t quadval;

    FEATURE_TO_INQUIRY_OFFSET(feature, offset);

    if (GetCameraControlRegister(handle, node, offset, &quadval) < 0)
    {
        return DC1394_FAILURE;
    }

    if (quadval & 0x10000000UL)
    {
        *value= DC1394_TRUE;
    }
    else
    {
        *value= DC1394_FALSE;
    }

    return DC1394_SUCCESS;
}

int
dc1394_is_one_push_in_operation(raw1394handle_t handle, nodeid_t node,
                                unsigned int feature, dc1394bool_t *value)
{
    octlet_t offset;
    quadlet_t quadval;

    if (feature == FEATURE_TRIGGER)
    {
        return DC1394_FAILURE;
    }

    FEATURE_TO_VALUE_OFFSET(feature, offset);

    if (GetCameraControlRegister(handle, node, offset, &quadval) < 0)
    {
        return DC1394_FAILURE;
    }

    if (quadval & 0x04000000UL)
    {
        *value= DC1394_TRUE;
    }
    else
    {
        *value= DC1394_FALSE;
    }

    return DC1394_SUCCESS;
}

int
dc1394_start_one_push_operation(raw1394handle_t handle, nodeid_t node,
                                unsigned int feature)
{
    octlet_t offset;
    quadlet_t curval;

    if (feature == FEATURE_TRIGGER)
    {
        return DC1394_FAILURE;
    }

    FEATURE_TO_VALUE_OFFSET(feature, offset);

    if (GetCameraControlRegister(handle, node, offset, &curval) < 0)
    {
        return DC1394_FAILURE;
    }

    if (!(curval & 0x04000000UL))
    {
        int retval;

        curval|= 0x04000000UL;
        retval= SetCameraControlRegister(handle, node, offset, curval);
        return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
    }

    return DC1394_SUCCESS;
}

int
dc1394_can_read_out(raw1394handle_t handle, nodeid_t node,
                    unsigned int feature, dc1394bool_t *value)
{
    octlet_t offset;
    quadlet_t quadval;

    FEATURE_TO_INQUIRY_OFFSET(feature, offset);

    if (GetCameraControlRegister(handle, node, offset, &quadval) < 0)
    {
        return DC1394_FAILURE;
    }

    if (quadval & 0x08000000UL)
    {
        *value= DC1394_TRUE;
    }
    else
    {
        *value= DC1394_FALSE;
    }

    return DC1394_SUCCESS;
}

int
dc1394_can_turn_on_off(raw1394handle_t handle, nodeid_t node,
                       unsigned int feature, dc1394bool_t *value)
{
    octlet_t offset;
    quadlet_t quadval;

    FEATURE_TO_INQUIRY_OFFSET(feature, offset);

    if (GetCameraControlRegister(handle, node, offset, &quadval) < 0)
    {
        return DC1394_FAILURE;
    }

    if (quadval & 0x04000000UL)
    {
        *value= DC1394_TRUE;
    }
    else
    {
        *value= DC1394_FALSE;
    }

    return DC1394_SUCCESS;
}

int
dc1394_is_feature_on(raw1394handle_t handle, nodeid_t node,
                     unsigned int feature, dc1394bool_t *value)
{
    octlet_t offset;
    quadlet_t quadval;

    FEATURE_TO_VALUE_OFFSET(feature, offset);

    if (GetCameraControlRegister(handle, node, offset, &quadval) < 0)
    {
        return DC1394_FAILURE;
    }

    if (quadval & 0x02000000UL)
    {
        *value= DC1394_TRUE;
    }
    else
    {
        *value= DC1394_FALSE;
    }

    return DC1394_SUCCESS;
}

int
dc1394_feature_on_off(raw1394handle_t handle, nodeid_t node,
                      unsigned int feature, unsigned int value)
{
    octlet_t offset;
    quadlet_t curval;

    FEATURE_TO_VALUE_OFFSET(feature, offset);

    if (GetCameraControlRegister(handle, node, offset, &curval) < 0)
    {
        return DC1394_FAILURE;
    }

    if (value && !(curval & 0x02000000UL))
    {
        int retval;

        curval|= 0x02000000UL;
        retval= SetCameraControlRegister(handle, node, offset, curval);
        return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
    }
    else if (!value && (curval & 0x02000000UL))
    {
        int retval;

        curval&= 0xFDFFFFFFUL;
        retval= SetCameraControlRegister(handle, node, offset, curval);
        return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
    }

    return DC1394_SUCCESS;
}

int
dc1394_has_auto_mode(raw1394handle_t handle, nodeid_t node,
                     unsigned int feature, dc1394bool_t *value)
{
    octlet_t offset;
    quadlet_t quadval;

    if (feature == FEATURE_TRIGGER)
    {
        return DC1394_FAILURE;
    }

    FEATURE_TO_INQUIRY_OFFSET(feature, offset);

    if (GetCameraControlRegister(handle, node, offset, &quadval) < 0)
    {
        return DC1394_FAILURE;
    }

    if (quadval & 0x02000000UL)
    {
        *value= DC1394_TRUE;
    }
    else
    {
        *value= DC1394_FALSE;
    }

    return DC1394_SUCCESS;
}

int
dc1394_has_manual_mode(raw1394handle_t handle, nodeid_t node,
                       unsigned int feature, dc1394bool_t *value)
{
    octlet_t offset;
    quadlet_t quadval;

    if (feature == FEATURE_TRIGGER)
    {
        return DC1394_FAILURE;
    }

    FEATURE_TO_INQUIRY_OFFSET(feature, offset);

    if (GetCameraControlRegister(handle, node, offset, &quadval) < 0)
    {
        return DC1394_FAILURE;
    }

    if (quadval & 0x01000000UL)
    {
        *value= DC1394_TRUE;
    }
    else
    {
        *value= DC1394_FALSE;
    }

    return DC1394_SUCCESS;
}

int
dc1394_is_feature_auto(raw1394handle_t handle, nodeid_t node,
                       unsigned int feature, dc1394bool_t *value)
{
    octlet_t offset;
    quadlet_t quadval;

    if (feature == FEATURE_TRIGGER)
    {
        return DC1394_FAILURE;
    }

    FEATURE_TO_VALUE_OFFSET(feature, offset);

    if (GetCameraControlRegister(handle, node, offset, &quadval) < 0)
    {
        return DC1394_FAILURE;
    }

    if (quadval & 0x01000000UL)
    {
        *value= DC1394_TRUE;
    }
    else
    {
        *value= DC1394_FALSE;
    }

    return DC1394_SUCCESS;
}

int
dc1394_auto_on_off(raw1394handle_t handle, nodeid_t node,
                   unsigned int feature, unsigned int value)
{
    octlet_t offset;
    quadlet_t curval;

    if (feature == FEATURE_TRIGGER)
    {
        return DC1394_FAILURE;
    }

    FEATURE_TO_VALUE_OFFSET(feature, offset);

    if (GetCameraControlRegister(handle, node, offset, &curval) < 0)
    {
        return DC1394_FAILURE;
    }

    if (value && !(curval & 0x01000000UL))
    {
        int retval;

        curval|= 0x01000000UL;
        retval= SetCameraControlRegister(handle, node, offset, curval);
        return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
    }
    else if (!value && (curval & 0x01000000UL))
    {
        int retval;

        curval&= 0xFEFFFFFFUL;
        retval= SetCameraControlRegister(handle, node, offset, curval);
        return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
    }

    return DC1394_SUCCESS;
}

int
dc1394_get_min_value(raw1394handle_t handle, nodeid_t node,
                     unsigned int feature, unsigned int *value)
{
    octlet_t offset;
    quadlet_t quadval;

    if (feature == FEATURE_TRIGGER)
    {
        return DC1394_FAILURE;
    }

    FEATURE_TO_INQUIRY_OFFSET(feature, offset);

    if (GetCameraControlRegister(handle, node, offset, &quadval) < 0)
    {
        return DC1394_FAILURE;
    }

    *value= (unsigned int)((quadval & 0xFFF000UL) >> 12);
    return DC1394_SUCCESS;
}

int
dc1394_get_max_value(raw1394handle_t handle, nodeid_t node,
                     unsigned int feature, unsigned int *value)
{
    octlet_t offset;
    quadlet_t quadval;

    if (feature == FEATURE_TRIGGER)
    {
        return DC1394_FAILURE;
    }

    FEATURE_TO_INQUIRY_OFFSET(feature, offset);

    if (GetCameraControlRegister(handle, node, offset, &quadval) < 0)
    {
        return DC1394_FAILURE;
    }

    *value= (unsigned int)(quadval & 0xFFFUL);
    return DC1394_SUCCESS;
}


/*
 * Memory load/save functions
 */
int
dc1394_get_memory_save_ch(raw1394handle_t handle, nodeid_t node,
                          unsigned int *channel)
{
    quadlet_t value;
    int retval= GetCameraControlRegister(handle, node, REG_CAMERA_MEM_SAVE_CH,
                                         &value);
    *channel= (unsigned int)((value >> 28) & 0xFUL);
    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}


int 
dc1394_get_memory_load_ch(raw1394handle_t handle, nodeid_t node,
                          unsigned int *channel)
{
    quadlet_t value;
    int retval= GetCameraControlRegister(handle, node, REG_CAMERA_CUR_MEM_CH,
                                         &value);
    *channel= (unsigned int)((value >> 28) & 0xFUL);
    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}


int 
dc1394_is_memory_save_in_operation(raw1394handle_t handle, nodeid_t node,
                                   dc1394bool_t *value)
{
    quadlet_t quadlet;
    int retval= GetCameraControlRegister(handle, node, REG_CAMERA_MEMORY_SAVE,
                                         &quadlet);
    *value = (quadlet & ON_VALUE) >> 31;
    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int 
dc1394_set_memory_save_ch(raw1394handle_t handle, nodeid_t node,
                          unsigned int channel)
{
    int retval= SetCameraControlRegister(handle, node, REG_CAMERA_MEM_SAVE_CH,
                                         (quadlet_t)((channel & 0xFUL) << 28));
    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_memory_save(raw1394handle_t handle, nodeid_t node)
{
    int retval= SetCameraControlRegister(handle, node, REG_CAMERA_MEMORY_SAVE,
                                         ON_VALUE);
    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_memory_load(raw1394handle_t handle, nodeid_t node, unsigned int channel)
{
    int retval= SetCameraControlRegister(handle, node, REG_CAMERA_CUR_MEM_CH,
                                         (quadlet_t)((channel & 0xFUL) << 28));
    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

/*
 * Trigger functions
 */

int
dc1394_set_trigger_polarity(raw1394handle_t handle, nodeid_t node,
                            dc1394bool_t polarity)
{
    int retval;
    quadlet_t curval;

    if (GetCameraControlRegister(handle, node, REG_CAMERA_TRIGGER_MODE,
                                 &curval) < 0)
    {
        return DC1394_FAILURE;
    }

    curval= (curval & 0xFEFFFFFFUL) | ((polarity & 0x1UL) << 24);
    retval= SetCameraControlRegister(handle, node, REG_CAMERA_TRIGGER_MODE,
                                     curval);
    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_get_trigger_polarity(raw1394handle_t handle, nodeid_t node,
                            dc1394bool_t *polarity)
{
    quadlet_t value;
    int retval= GetCameraControlRegister(handle, node,
                                         REG_CAMERA_TRIGGER_MODE, &value);

    *polarity= (unsigned int)( ((value >> 24) & 0x1UL) );
    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_trigger_has_polarity(raw1394handle_t handle, nodeid_t node,
                            dc1394bool_t *polarity)
{
    octlet_t offset;
    quadlet_t quadval;

    offset= REG_CAMERA_FEATURE_HI_BASE_INQ;

    if (GetCameraControlRegister(handle, node,
                                 offset +
                                 ((FEATURE_TRIGGER - FEATURE_MIN) * 0x04U),
                                 &quadval) < 0)
    {
        return DC1394_FAILURE;
    }

    if (quadval & 0x02000000UL)
    {
        *polarity= DC1394_TRUE;
    }
    else
    {
        *polarity= DC1394_FALSE;
    }

    return DC1394_SUCCESS;
}

int
dc1394_set_trigger_on_off(raw1394handle_t handle, nodeid_t node,
                          dc1394bool_t on_off)
{
  return dc1394_feature_on_off(handle, node, FEATURE_TRIGGER, on_off);
}

int
dc1394_get_trigger_on_off(raw1394handle_t handle, nodeid_t node,
                          dc1394bool_t *on_off)
{
  return dc1394_is_feature_on(handle, node, FEATURE_TRIGGER, on_off);
}

int
dc1394_set_soft_trigger(raw1394handle_t handle, nodeid_t node)
{
  int retval= SetCameraControlRegister(handle, node, REG_CAMERA_SOFT_TRIGGER, ON_VALUE);
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_unset_soft_trigger(raw1394handle_t handle, nodeid_t node)
{
  int retval= SetCameraControlRegister(handle, node, REG_CAMERA_SOFT_TRIGGER, OFF_VALUE);
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_get_soft_trigger(raw1394handle_t handle, nodeid_t node, dc1394bool_t *is_on)
{
  quadlet_t value;
  int retval;
  
  //printf("before gst\n");
  retval = GetCameraControlRegister(handle, node, REG_CAMERA_SOFT_TRIGGER, &value);
  //printf("after gst %x, %x\n", retval, value);
  *is_on = value & ON_VALUE;
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

int
dc1394_query_absolute_control(raw1394handle_t handle, nodeid_t node,
                              unsigned int feature, dc1394bool_t *value)
{
    octlet_t offset;
    quadlet_t quadval;

    FEATURE_TO_VALUE_OFFSET(feature, offset);

    if (GetCameraControlRegister(handle, node, offset, &quadval) < 0)
    {
        return DC1394_FAILURE;
    }

    if (quadval & 0x40000000UL)
    {
        *value= DC1394_TRUE;
    }
    else
    {
        *value= DC1394_FALSE;
    }

    return DC1394_SUCCESS;
}

int
dc1394_absolute_setting_on_off(raw1394handle_t handle, nodeid_t node,
                               unsigned int feature, unsigned int value)
{
    octlet_t offset;
    quadlet_t curval;
    int retval;

    FEATURE_TO_VALUE_OFFSET(feature, offset);

    if (GetCameraControlRegister(handle, node, offset, &curval) < 0)
    {
        return DC1394_FAILURE;
    }

    if (value && !(curval & 0x40000000UL))
      {
        curval|= 0x40000000UL;
        retval= SetCameraControlRegister(handle, node, offset, curval);
        return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
      }
    else if (!value && (curval & 0x40000000UL))
      {
        curval&= 0xBFFFFFFFUL;
        retval= SetCameraControlRegister(handle, node, offset, curval);
        return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
      }

    return DC1394_SUCCESS;
}


int
dc1394_has_absolute_control(raw1394handle_t handle, nodeid_t node,
                            unsigned int feature, dc1394bool_t *value)
{
    octlet_t offset;
    quadlet_t quadval;

    FEATURE_TO_INQUIRY_OFFSET(feature, offset);

    if (GetCameraControlRegister(handle, node, offset, &quadval) < 0)
    {
        return DC1394_FAILURE;
    }

    if (quadval & 0x40000000UL)
    {
        *value= DC1394_TRUE;
    }
    else
    {
        *value= DC1394_FALSE;
    }

    return DC1394_SUCCESS;
}


/* This function returns the bandwidth used by the camera in bandwidth units.
   The 1394 bus has 4915 bandwidth units available per cycle. Each unit corresponds
   to the time it takes to send one quadlet at ISO speed S1600. The bandwidth usage
   at S400 is thus four times the number of quadlets per packet. Thanks to Krisitian
   Hogsberg for clarifying this.
*/

int
dc1394_get_bandwidth_usage(raw1394handle_t handle, nodeid_t node, unsigned int *bandwidth)
{
  
  unsigned int format, iso, mode, qpp, channel, speed, framerate=0;

  // get camera ISO status:
  if (dc1394_get_iso_status(handle, node, &iso)!= DC1394_SUCCESS)
    return DC1394_FAILURE;
  
  if (iso==DC1394_TRUE) {

    // get format and mode
    if (dc1394_get_video_format(handle, node, &format) != DC1394_SUCCESS)
      return DC1394_FAILURE;
    
    if (dc1394_get_video_mode(handle, node, &mode) != DC1394_SUCCESS)
      return DC1394_FAILURE;
    
    if (format==FORMAT_SCALABLE_IMAGE_SIZE) {
      // use the bytes per packet value:
      if (dc1394_query_format7_byte_per_packet(handle, node, mode, &qpp) != DC1394_SUCCESS)
	return DC1394_FAILURE;
      qpp=qpp/4;
    }
    else {
      // get the framerate:
      if (dc1394_get_video_framerate(handle, node, &framerate) != DC1394_SUCCESS)
	return DC1394_FAILURE;
      qpp=_dc1394_get_quadlets_per_packet(format, mode, framerate); 
    }
    // add the ISO header and footer:
    qpp+=3;

    // get camera ISO speed:
    if (dc1394_get_iso_channel_and_speed(handle, node, &channel, &speed)!= DC1394_SUCCESS)
      return DC1394_FAILURE;
    
    // mutiply by 4 anyway because the best speed is SPEED_400 only
    if (speed>=SPEED_1600)
      *bandwidth = qpp << (SPEED_1600-speed);
    else
      *bandwidth = qpp >> (speed-SPEED_1600);
  }
  else {
    *bandwidth=0;
  }
  return DC1394_SUCCESS;
    
}

int
dc1394_get_camera_port(raw1394handle_t handle)
{
  dc1394_camerahandle *camerahandle;
  camerahandle=(dc1394_camerahandle*) raw1394_get_userdata(handle);
  return(camerahandle->port);

}
