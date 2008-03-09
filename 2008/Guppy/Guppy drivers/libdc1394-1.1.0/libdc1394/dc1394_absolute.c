/*
 * 1394-Based Digital Camera Absolute Setting functions
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

#define REG_CAMERA_FEATURE_ABS_HI_BASE 0x700U
#define REG_CAMERA_FEATURE_ABS_LO_BASE 0x780U

#define REG_CAMERA_ABS_MIN             0x000U
#define REG_CAMERA_ABS_MAX             0x004U
#define REG_CAMERA_ABS_VALUE           0x008U

#define FEATURE_TO_ABS_VALUE_OFFSET(feature, offset)                  \
                                                                      \
    if ( (feature > FEATURE_MAX) || (feature < FEATURE_MIN) )         \
    {                                                                 \
	return DC1394_FAILURE;                                        \
    }                                                                 \
    else if (feature < FEATURE_ZOOM)                                  \
    {                                                                 \
	offset= REG_CAMERA_FEATURE_ABS_HI_BASE;                       \
        feature-= FEATURE_MIN;                                        \
    }                                                                 \
    else                                                              \
    {                                                                 \
	offset= REG_CAMERA_FEATURE_ABS_LO_BASE;                       \
	feature-= FEATURE_ZOOM;                                       \
                                                                      \
	if (feature >= FEATURE_CAPTURE_SIZE)                          \
	{                                                             \
	    feature+= 12;                                             \
	}                                                             \
                                                                      \
    }                                                                 \
                                                                      \
    offset+= feature * 0x04U;

/**********************/ 
/* Internal functions */
/**********************/

static int
QueryAbsoluteCSROffset(raw1394handle_t handle, nodeid_t node, int feature,
		       quadlet_t *value)
{
    int retval;
    int offset;

    FEATURE_TO_ABS_VALUE_OFFSET(feature, offset)

    retval= GetCameraControlRegister(handle, node, offset, value);
    return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

static int
GetCameraAbsoluteRegister(raw1394handle_t handle, nodeid_t node,
			  int feature, octlet_t offset, quadlet_t *value)
{
    int retval, retry= MAX_RETRIES;
    quadlet_t csr;
    
    if (QueryAbsoluteCSROffset(handle, node, feature, &csr) != DC1394_SUCCESS)
    {
        return DC1394_FAILURE;
    }

    csr*= 0x04UL;

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
            printf("Absolute reg read ack of %x rcode of %x\n", ack, rcode);
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
SetCameraAbsoluteRegister(raw1394handle_t handle, nodeid_t node,
			  int feature, octlet_t offset, quadlet_t* value)
{
    int retval, retry= MAX_RETRIES;
    quadlet_t csr;
    
    if (QueryAbsoluteCSROffset(handle, node, feature, &csr)!=DC1394_SUCCESS)
    {
        return DC1394_FAILURE;
    }
    csr*= 0x04UL;
  
    /* conditionally byte swap the value (addition by PDJ) */
    *value= htonl(*value);
 
    /* retry a few times if necessary (addition by PDJ) */
    while(retry--)
    {
        retval= raw1394_write(handle, 0xffc0 | node,
                              CONFIG_ROM_BASE + offset + csr, 4, value);

#ifdef LIBRAW1394_OLD
        if (retval >= 0)
        {
            int ack= retval >> 16;
            int rcode= retval & 0xffff;

#ifdef SHOW_ERRORS
            printf("Absolute reg write ack of %x rcode of %x\n", ack, rcode);
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

/**************************
 *   EXTERNAL FUNCTIONS   *
 **************************/

int
dc1394_query_absolute_feature_min_max(raw1394handle_t handle, nodeid_t node,
				      unsigned int feature,
				      float *min, float *max)
{
    int retval;
    
    if ( (feature > FEATURE_MAX) || (feature < FEATURE_MIN) )
    {
        return DC1394_FAILURE;
    }
    else     
    {
        retval= GetCameraAbsoluteRegister(handle, node, feature,
					  REG_CAMERA_ABS_MAX,
					  (quadlet_t*)max);
        retval= GetCameraAbsoluteRegister(handle, node, feature,
					  REG_CAMERA_ABS_MIN,
					  (quadlet_t*)min);
    }

    return retval;
}


int
dc1394_query_absolute_feature_value(raw1394handle_t handle, nodeid_t node,
				    int feature, float *value)
{
    int retval;
    
    if ( (feature > FEATURE_MAX) || (feature < FEATURE_MIN) )
    {
        return DC1394_FAILURE;
    }
    else     
    {
        retval= GetCameraAbsoluteRegister(handle, node, feature,
					  REG_CAMERA_ABS_VALUE,
					  (quadlet_t*)value);
    }

    return retval;
}
 

int
dc1394_set_absolute_feature_value(raw1394handle_t handle, nodeid_t node,
				  int feature, float value)
{
  if (SetCameraAbsoluteRegister(handle, node, feature, REG_CAMERA_ABS_VALUE,
				(quadlet_t*)(&value)) != DC1394_SUCCESS)
    {
      printf("(%s) Absolute value setting failure \n", __FILE__);
      return DC1394_FAILURE;
    }
  return DC1394_SUCCESS;
}
 
