/*
 * 1394-Based Digital Camera Control Library
 * Copyright (C) 2005 Inria Sophia-Antipolis
 *
 * Written by Pierre MOOS <pierre.moos@gmail.com> 
 * Version : 16/02/2005 
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

#include "dc1394_vendor_avt.h"

/********************************************************/
/* Configuration Register Offsets for Advances features */
/********************************************************/
/* See the Marlin Technical Manuel for definitions of these */

#define REG_CAMERA_VERSION_INFO1  	0x010U  
#define REG_CAMERA_VERSION_INFO3  	0x018U  
#define REG_CAMERA_ADV_INQ_1  		0x040U  
#define REG_CAMERA_ADV_INQ_2  		0x044U  
#define REG_CAMERA_ADV_INQ_3  		0x048U  
#define REG_CAMERA_ADV_INQ_4  		0x04CU  
#define REG_CAMERA_MAX_RESOLUTION  	0x200U  
#define REG_CAMERA_TIMEBASE  		0x208U  
#define REG_CAMERA_EXTD_SHUTTER  	0x20CU  
#define REG_CAMERA_TEST_IMAGE  		0x210U  
#define REG_CAMERA_SEQUENCE_CTRL  	0x220U  /* except MF131x */
#define REG_CAMERA_SEQUENCE_PARAM  	0x224U  /* except MF131x */
#define REG_CAMERA_LUT_CTRL  		0x240U  
#define REG_CAMERA_LUT_MEM_CTRL  	0x244U  
#define REG_CAMERA_LUT_INFO  		0x248U  
#define REG_CAMERA_SHDG_CTRL  		0x250U  
#define REG_CAMERA_SHDG_MEM_CTRL  	0x254U  
#define REG_CAMERA_SHDG_INFO  		0x258U  
#define REG_CAMERA_DEFERRED_TRANS  	0x260U  
#define REG_CAMERA_FRAMEINFO  		0x270U  
#define REG_CAMERA_FRAMECOUNTER  	0x274U  
#define REG_CAMERA_HDR_CONTROL 		0x280U  /* MF131x only */
#define REG_CAMERA_KNEEPOINT_1  	0x284U  /* MF131x only */
#define REG_CAMERA_KNEEPOINT_2  	0x288U  /* MF131x only */
#define REG_CAMERA_KNEEPOINT_3  	0x28CU  /* MF131x only */
#define REG_CAMERA_DSNU_CONTROL  	0x290U  /* MF131B only; Firmware 2.02 */
#define REG_CAMERA_BLEMISH_CONTROL  	0x294U  /* MF131x only; Firmware 2.02 */
#define REG_CAMERA_IO_INP_CTRL1  	0x300U  
#define REG_CAMERA_IO_INP_CTRL2  	0x304U  
#define REG_CAMERA_IO_INP_CTRL3  	0x308U  /* Dolphin series only */
#define REG_CAMERA_IO_OUTP_CTRL1  	0x320U  
#define REG_CAMERA_IO_OUTP_CTRL2	0x324U  
#define REG_CAMERA_IO_OUTP_CTRL3  	0x328U  /* Dolphin series only */
#define REG_CAMERA_IO_INTENA_DELAY  	0x340U  
#define REG_CAMERA_AUTOSHUTTER_CTRL  	0x360U  /* Marlin series only */
#define REG_CAMERA_AUTOSHUTTER_LO  	0x364U  /* Marlin series only */
#define REG_CAMERA_AUTOSHUTTER_HI 	0x368U  /* Marlin series only */
#define REG_CAMERA_AUTOGAIN_CTRL  	0x370U  /* Marlin series only */
#define REG_CAMERA_AUTOFNC_AOI  	0x390U  /* Marlin series only */
#define REG_CAMERA_AF_AREA_POSITION  	0x394U  /* Marlin series only */
#define REG_CAMERA_AF_AREA_SIZE  	0x398U  /* Marlin series only */
#define REG_CAMERA_COLOR_CORR  		0x3A0U  /* Marlin C-type cameras only */
#define REG_CAMERA_TRIGGER_DELAY  	0x400U  
#define REG_CAMERA_MIRROR_IMAGE  	0x410U  /* Marlin series only */
#define REG_CAMERA_SOFT_RESET  		0x510U  
#define REG_CAMERA_GPDATA_INFO  	0xFFCU  
#define REG_CAMERA_GPDATA_BUFFER	0x1000U

/************************************************************************/
/* Get Version  	(Read Only)					*/
/************************************************************************/
int
dc1394_avt_get_version(raw1394handle_t handle, nodeid_t node, 
		       unsigned int *Version, unsigned int *Camera_ID, unsigned int *FPGA_Version  )
{
  quadlet_t value;
  int retval;
  
  /* Retrieve uC Version*/
  retval = GetCameraAdvControlRegister(handle, node,REG_CAMERA_VERSION_INFO1, &value);
  
  if(!retval) {
    /* uC Version : Bits 16..31 */
    *Version =(unsigned int)(value & 0xFFFFUL );
    
    /*  Retrieve Camera ID and FPGA_Version */
    retval = GetCameraAdvControlRegister(handle, node,REG_CAMERA_VERSION_INFO3, &value);
    
    /* Camera_ID : bit 0-15 */
    *Camera_ID =(unsigned int)(value >>16 );      
    
    /* FPGA_Version : bit 16-31 */
    *FPGA_Version=(unsigned int)(value & 0xFFFFUL );      
  }
  
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
  
}

/************************************************************************/
/* Get Advanced feature inquiry						*/
/************************************************************************/
int
dc1394_avt_get_advanced_feature_inquiry(raw1394handle_t handle, nodeid_t node, 
					dc1394_avt_adv_feature_info_t *adv_feature  )
{
  quadlet_t value;
  int retval;
  
  /* Retrieve first group of features presence */
  retval = GetCameraAdvControlRegister(handle, node,REG_CAMERA_ADV_INQ_1, &value);
  
  if(!retval) {
    
    adv_feature->MaxResolution=	(value & 0x80000000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->TimeBase= 	(value & 0x40000000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->ExtdShutter= 	(value & 0x20000000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->TestImage= 	(value & 0x10000000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->FrameInfo= 	(value & 0x08000000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->Sequences= 	(value & 0x04000000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->VersionInfo= 	(value & 0x02000000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->Lookup_Tables= (value & 0x00800000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->Shading= 	(value & 0x00400000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->DeferredTrans= (value & 0x00200000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->HDR_Mode= 	(value & 0x00100000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->DSNU= 		(value & 0x00080000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->TriggerDelay= 	(value & 0x00040000UL) ? DC1394_TRUE : DC1394_FALSE;    
    adv_feature->GP_Buffer= 	(value & 0x00000001UL) ? DC1394_TRUE : DC1394_FALSE;        
       
    /* Remember this request have been done */ 
    adv_feature->features_requested = DC1394_TRUE;               
       
  }       

  /* Retrieve second group of features presence */    
  retval = GetCameraAdvControlRegister(handle, node,REG_CAMERA_ADV_INQ_2, &value);
  
  if(!retval) {
    
    adv_feature->Input_1 = 	(value & 0x80000000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->Input_2 = 	(value & 0x40000000UL) ? DC1394_TRUE : DC1394_FALSE;
    adv_feature->Output_1= 	(value & 0x00800000UL) ? DC1394_TRUE : DC1394_FALSE;  
    adv_feature->Output_2= 	(value & 0x00400000UL) ? DC1394_TRUE : DC1394_FALSE;        
    adv_feature->IntEnaDelay= 	(value & 0x00008000UL) ? DC1394_TRUE : DC1394_FALSE;     
       
  }
    

  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
  
}

/************************************************************************/
/* Print Advanced features 						*/
/************************************************************************/
int
dc1394_avt_print_advanced_feature(dc1394_avt_adv_feature_info_t *adv_feature )
{

  puts ("ADVANCED FEATURES SUPPORTED:");
  if(adv_feature->MaxResolution == DC1394_TRUE) puts (" MaxResolution ");
  if(adv_feature->TimeBase == DC1394_TRUE) 	puts (" TimeBase ");
  if(adv_feature->ExtdShutter == DC1394_TRUE) 	puts (" ExtdShutter ");
  if(adv_feature->TestImage == DC1394_TRUE) 	puts (" TestImage ");
  if(adv_feature->FrameInfo == DC1394_TRUE) 	puts (" FrameInfo ");
  if(adv_feature->Sequences == DC1394_TRUE) 	puts (" Sequences ");
  if(adv_feature->VersionInfo == DC1394_TRUE) 	puts (" VersionInfo ");
  if(adv_feature->Lookup_Tables == DC1394_TRUE)	puts (" Lookup_Tables ");
  if(adv_feature->Shading == DC1394_TRUE) 	puts (" Shading ");
  if(adv_feature->DeferredTrans == DC1394_TRUE) puts (" DeferredTrans ");
  if(adv_feature->HDR_Mode == DC1394_TRUE) 	puts (" HDR_Mode ");
  if(adv_feature->DSNU == DC1394_TRUE) 		puts (" DSNU ");
  if(adv_feature->TriggerDelay == DC1394_TRUE) 	puts (" TriggerDelay ");
  if(adv_feature->GP_Buffer == DC1394_TRUE) 	puts (" GP_Buffer ");
  if(adv_feature->Input_1 == DC1394_TRUE)	puts (" Input_1 ");
  if(adv_feature->Input_2 == DC1394_TRUE) 	puts (" Input_2 ");
  if(adv_feature->Output_1 == DC1394_TRUE) 	puts (" Output_1 ");
  if(adv_feature->Output_2 == DC1394_TRUE) 	puts (" Output_2 ");
  if(adv_feature->IntEnaDelay == DC1394_TRUE) 	puts (" IntEnaDelay ");
  
  return 0;
  
}


/************************************************************************/
/* Get shading mode							*/
/************************************************************************/
int
dc1394_avt_get_shading(raw1394handle_t handle, nodeid_t node, 
		       dc1394bool_t *on_off, unsigned int *frame_nb  )
{
  quadlet_t value;
  int retval;
  
  /* Retrieve shading properties */
  retval = GetCameraAdvControlRegister(handle, node,REG_CAMERA_SHDG_CTRL, &value);
  
  if(!retval) {
    /* Shading ON / OFF : Bit 6 */
    *on_off = (unsigned int)((value & 0x2000000UL) >> 25); 
    
    /* Number of images for auto computing of the shading reference: Bits 24..31 */
    *frame_nb =(unsigned int)((value & 0xFFUL));
  }       
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
  
}


/************************************************************************/
/* Set shading mode							*/
/************************************************************************/
int
dc1394_avt_set_shading(raw1394handle_t handle, nodeid_t node,
		       dc1394bool_t on_off,dc1394bool_t compute, unsigned int frame_nb)
{
  quadlet_t curval;
  int retval;
  
  /* Retrieve current shading properties */    
  if(GetCameraAdvControlRegister(handle, node,REG_CAMERA_SHDG_CTRL, &curval))
    return DC1394_FAILURE;
  
  /* Shading ON / OFF : Bit 6 */
  curval = (curval & 0xFDFFFFFFUL) | ((on_off ) << 25); 
  
  /* Compute : Bit 5 */
  curval = (curval & 0xFBFFFFFFUL) | ((compute ) << 26); 
  
  /* Number of images : Bits 24..31 */
  curval = (curval & 0xFFFFFF00UL) | ((frame_nb & 0xFFUL ));   
  
  /* Set new parameters */    
  retval = SetCameraAdvControlRegister(handle, node,REG_CAMERA_SHDG_CTRL, curval);
  
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}


/************************************************************************/
/* Get shading  mem ctrl						*/
/************************************************************************/
int
dc1394_avt_get_shading_mem_ctrl(raw1394handle_t handle, nodeid_t node, dc1394bool_t *en_write, 
				dc1394bool_t *en_read, unsigned int *addroffset)
{
  quadlet_t value;
  int retval;
  
  /* Retrieve current memory shading properties */    
  retval = GetCameraAdvControlRegister(handle, node,REG_CAMERA_SHDG_MEM_CTRL, &value);
  
  if(!retval) {
    /* Enable write access : Bit 5 */
    *en_write = (unsigned int)((value & 0x4000000UL) >> 26); 
    
    /* Enable read access : Bit 6 */
    *en_read = (unsigned int)((value & 0x2000000UL) >> 25); 
    
    /* addroffset in byte : Bits 8..31 */
    *addroffset =(unsigned int)((value & 0xFFFFFFUL));
  }       
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
  
}


/************************************************************************/
/* Set shading mem ctrl							*/
/************************************************************************/
int
dc1394_avt_set_shading_mem_ctrl(raw1394handle_t handle, nodeid_t node,
				dc1394bool_t en_write, dc1394bool_t en_read, unsigned int addroffset)
{
  quadlet_t curval;
  int retval;
  
  /* Retrieve current shading properties */        
  if(GetCameraAdvControlRegister(handle, node,REG_CAMERA_SHDG_MEM_CTRL, &curval))
    return DC1394_FAILURE;
  
  /* read access enable : Bit 6 */
  curval = (curval & 0xFDFFFFFFUL) | ((en_read ) << 25); 
  
  /* write access enable : Bit 5 */
  curval = (curval & 0xFBFFFFFFUL) | ((en_write ) << 26); 
  
  /* Number of images : Bits 8..31 */
  curval = (curval & 0xFF000000UL) | ((addroffset & 0xFFFFFFUL ));   
  
  /* Set new parameters */ 
  retval = SetCameraAdvControlRegister(handle, node,REG_CAMERA_LUT_MEM_CTRL, curval);
  
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

/************************************************************************/
/* Get shading  info							*/
/************************************************************************/
int
dc1394_avt_get_shading_info(raw1394handle_t handle, nodeid_t node, unsigned int *MaxImageSize)
{
  quadlet_t value;
  
  /* Retrieve shading info */    
  int retval = GetCameraAdvControlRegister(handle, node,REG_CAMERA_SHDG_INFO, &value);
  
  if(!retval) {
    /* Max Shading Image size(byte) : Bits 8..31 */
    *MaxImageSize =(unsigned int)((value & 0xFFFFFFUL));
  }       
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
  
}


/************************************************************************/
/* Get Multiple slope parameters	(HDR)				*/
/************************************************************************/
int
dc1394_avt_get_multiple_slope(raw1394handle_t handle, nodeid_t node, 
			      dc1394bool_t *on_off, unsigned int *points_nb,unsigned int *kneepoint1, 
			      unsigned int *kneepoint2, unsigned int *kneepoint3)
{
  quadlet_t value;
  int retval;    
  
  /* Retrieve current hdr parameters */    
  retval = GetCameraAdvControlRegister(handle, node,REG_CAMERA_HDR_CONTROL, &value);
  
  if(!retval) {
    /* Multiple slope ON / OFF : Bit 6 */
    *on_off = (unsigned int)((value & 0x2000000UL) >> 25); 
    
    /* Number of actives points : Bits 28..31 */
    *points_nb =(unsigned int)((value & 0xFUL));
    
    /* kneepoints */
    if(GetCameraAdvControlRegister(handle, node,REG_CAMERA_KNEEPOINT_1, kneepoint1)) 
      return DC1394_FAILURE; 
    if(GetCameraAdvControlRegister(handle, node,REG_CAMERA_KNEEPOINT_2, kneepoint2))
      return DC1394_FAILURE;
    if(GetCameraAdvControlRegister(handle, node,REG_CAMERA_KNEEPOINT_3, kneepoint3))
      return DC1394_FAILURE;
    
  }
  
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
        
}


/************************************************************************/
/* Set Multiple slope parameters					*/
/************************************************************************/
int
dc1394_avt_set_multiple_slope(raw1394handle_t handle, nodeid_t node, 
			      dc1394bool_t on_off, unsigned int points_nb, unsigned int kneepoint1, 
			      unsigned int kneepoint2, unsigned int kneepoint3)
{
  quadlet_t curval;
  int retval;
  
  /* Retrieve current hdr parameters */        
  if(GetCameraAdvControlRegister(handle, node,REG_CAMERA_HDR_CONTROL, &curval))
    return DC1394_FAILURE;
  
  /* Shading ON / OFF : Bit 6 */
  curval = (curval & 0xFDFFFFFFUL) | ((on_off ) << 25); 
  
  /* Number of points : Bits 28..31 */
  curval = (curval & 0xFFFFFFF0UL) | ((points_nb & 0xFUL ));   
  
  if(SetCameraAdvControlRegister(handle, node,REG_CAMERA_HDR_CONTROL, curval))
    return DC1394_FAILURE;
  
  /* kneepoints */
  if(SetCameraAdvControlRegister(handle, node,REG_CAMERA_KNEEPOINT_1, kneepoint1))
    return DC1394_FAILURE;
  if(SetCameraAdvControlRegister(handle, node,REG_CAMERA_KNEEPOINT_2, kneepoint2))
    return DC1394_FAILURE;
  
  /* Set new hdr parameters */    
  retval=SetCameraAdvControlRegister(handle, node,REG_CAMERA_KNEEPOINT_3, kneepoint3);
  
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}






/************************************************************************/
/* Get Shutter Timebase 						*/
/************************************************************************/
int
dc1394_avt_get_timebase(raw1394handle_t handle, nodeid_t node, 
			unsigned int *timebase_id  )
{
  quadlet_t value;
  int retval;
        
  /* Retrieve current timebase */    
  retval = GetCameraAdvControlRegister(handle, node,REG_CAMERA_TIMEBASE, &value);
  
  if(!retval) {
    /* Time base ID : Bits 29..31 */
    *timebase_id =(unsigned int)((value & 0xFUL));
  }       
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
  
}


/************************************************************************/
/* Set Shutter Timebase (acquisition must be stopped)			*/
/************************************************************************/
int
dc1394_avt_set_timebase(raw1394handle_t handle, nodeid_t node,
			unsigned int timebase_id)
{
  quadlet_t curval;
  int retval;
  
  /* Retrieve current timebase */        
  if(GetCameraAdvControlRegister(handle, node,REG_CAMERA_TIMEBASE, &curval))
    return DC1394_FAILURE;
  
  curval = (curval & 0xFFFFFFF0UL) | ((timebase_id & 0xFUL ));   
  
  /* Set new timebase */     
  retval = SetCameraAdvControlRegister(handle, node,REG_CAMERA_TIMEBASE, curval);
  
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}


/************************************************************************/
/* Get Extented Shutter  						*/
/************************************************************************/
int
dc1394_avt_get_extented_shutter(raw1394handle_t handle, nodeid_t node, 
				unsigned int *timebase_id  )
{
  quadlet_t value;
  int retval;    
    
  /* Retrieve current extented shutter value */    
  retval = GetCameraAdvControlRegister(handle, node,REG_CAMERA_EXTD_SHUTTER, &value);
  
  if(!retval) {
    /* Exposure Time in us: Bits 6..31 */
    *timebase_id =(unsigned int)((value & 0xFFFFFFFUL));
  }       
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
  
}


/************************************************************************/
/* Set Extented shutter							*/
/************************************************************************/
int
dc1394_avt_set_extented_shutter(raw1394handle_t handle, nodeid_t node,
				unsigned int timebase_id)
{
  quadlet_t curval;
  int retval;
  
  /* Retrieve current extented shutter value */        
  if(GetCameraAdvControlRegister(handle, node,REG_CAMERA_EXTD_SHUTTER, &curval))
    return DC1394_FAILURE;
  
  /* Time base ID : Bits 6..31 */
  curval = (curval & 0xF0000000UL) | ((timebase_id & 0x0FFFFFFFUL ));   
  
  /* Set new extented shutter value */    
  retval = SetCameraAdvControlRegister(handle, node,REG_CAMERA_EXTD_SHUTTER, curval);
  
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}


/************************************************************************/
/* Get MaxResolution  	(Read Only)					*/
/************************************************************************/
int
dc1394_avt_get_MaxResolution(raw1394handle_t handle, nodeid_t node, 
			     unsigned int *MaxHeight, unsigned int *MaxWidth  )
{
  quadlet_t value;
  int retval;
  
  /* Retrieve the maximum resolution */    
  retval = GetCameraAdvControlRegister(handle, node,REG_CAMERA_MAX_RESOLUTION, &value);
  
  if(!retval) {
    /* MaxHeight : Bits 0..15 */
    *MaxHeight =(unsigned int)(value >> 16);
    /* MaxWidth : Bits 16..31 */
    *MaxWidth =(unsigned int)(value & 0xFFFFUL );      
  }       
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
  
}


/************************************************************************/
/* Get Auto Shutter  							*/
/************************************************************************/
int
dc1394_avt_get_auto_shutter(raw1394handle_t handle, nodeid_t node, 
			    unsigned int *MinValue, unsigned int *MaxValue  )
{
  quadlet_t value;
  
  /* Retrieve current min auto shutter value */    
  int retval = GetCameraAdvControlRegister(handle, node,REG_CAMERA_AUTOSHUTTER_LO, &value);
  
  if(!retval) {
    *MinValue =(unsigned int)value;
    
    /* Retrieve current max auto shutter value */    
    retval = GetCameraAdvControlRegister(handle, node,REG_CAMERA_AUTOSHUTTER_HI, &value); 
    
    if(!retval)
      *MaxValue =(unsigned int)value;
  }
  
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
        
}


/************************************************************************/
/* Set Auto shutter							*/
/************************************************************************/
int
dc1394_avt_set_auto_shutter(raw1394handle_t handle, nodeid_t node,
			    unsigned int MinValue, unsigned int MaxValue  )
{
  int retval;
  /* Set min auto shutter value */    
  retval = SetCameraAdvControlRegister(handle, node,REG_CAMERA_AUTOSHUTTER_LO, MinValue);
  
  /* Set max auto shutter value */    
  if(!retval)
    retval = SetCameraAdvControlRegister(handle, node,REG_CAMERA_AUTOSHUTTER_HI, MaxValue); 
  
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}


/************************************************************************/
/* Get Auto Gain  							*/
/************************************************************************/
int
dc1394_avt_get_auto_gain(raw1394handle_t handle, nodeid_t node, 
			 unsigned int *MinValue, unsigned int *MaxValue  )
{
  quadlet_t value;
  int retval;
    
  /* Retrieve auto gain values */    
  retval = GetCameraAdvControlRegister(handle, node,REG_CAMERA_AUTOGAIN_CTRL, &value);
  
  if(!retval) {
    /* Min : bits 20..31 */
    *MinValue =(unsigned int)(value & 0xFFFUL);
    /* Max : bits 4..15 */
    *MaxValue =(unsigned int)((value >> 16) & 0xFFFUL);
  }
  
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
        
}


/************************************************************************/
/* Set Auto gain							*/
/************************************************************************/
int
dc1394_avt_set_auto_gain(raw1394handle_t handle, nodeid_t node,
			 unsigned int MinValue, unsigned int MaxValue  )
{
  int retval;
  quadlet_t value;    
  
  /* Max : bits 4..15, Min : bits 20..31  */
  value = ( MaxValue <<16 ) | ( MinValue );
  
  /* Set new parameters */    
  retval= SetCameraAdvControlRegister(handle, node,REG_CAMERA_AUTOGAIN_CTRL,value );
  
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}


/************************************************************************/
/* Get Trigger delay							*/
/************************************************************************/
int
dc1394_avt_get_trigger_delay(raw1394handle_t handle, nodeid_t node, 
			     dc1394bool_t *on_off, unsigned int *DelayTime  )
{
  quadlet_t value;
  int retval;
  
  /* Retrieve trigger delay */    
  retval = GetCameraAdvControlRegister(handle, node,REG_CAMERA_TRIGGER_DELAY, &value);
  
  if(!retval) {
    /* trigger_delay ON / OFF : Bit 6 */
    *on_off = (unsigned int)((value & 0x2000000UL) >> 25); 

    /* Delai time in us : Bits 11..31 */
    *DelayTime =(unsigned int)((value & 0xFFFFFUL));
  }       
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
  
}

/************************************************************************/
/* Set Trigger delay							*/
/************************************************************************/
int
dc1394_avt_set_trigger_delay(raw1394handle_t handle, nodeid_t node,
			     dc1394bool_t on_off, unsigned int DelayTime)
{
  quadlet_t curval;
  int retval;
  
  /* Retrieve trigger delay */        
  if(GetCameraAdvControlRegister(handle, node,REG_CAMERA_TRIGGER_DELAY, &curval))
    return DC1394_FAILURE;
  
  /* trigger_delay ON / OFF : Bit 6 */
  curval = (curval & 0xFDFFFFFFUL) | ((on_off ) << 25); 
  
  /* Delay time in us : Bits 11..31 */
  curval = (curval & 0xFFF00000UL) | DelayTime;   
  
  /* Set new parameters */     
  retval = SetCameraAdvControlRegister(handle, node,REG_CAMERA_TRIGGER_DELAY, curval);
  
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}


/************************************************************************/
/* Get Mirror 								*/
/************************************************************************/
int
dc1394_avt_get_mirror(raw1394handle_t handle, nodeid_t node, 
		      dc1394bool_t *on_off)
{
  quadlet_t value;
  int retval;    
  /* Retrieve Mirror mode */
  retval = GetCameraAdvControlRegister(handle, node,REG_CAMERA_MIRROR_IMAGE, &value);
  
  if(!retval) {
    /* mirror ON / OFF : Bit 6 */
    *on_off = (unsigned int)((value & 0x2000000UL) >> 25); 
  }       
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
  
}

/************************************************************************/
/* Set Mirror								*/
/************************************************************************/
int
dc1394_avt_set_mirror(raw1394handle_t handle, nodeid_t node,
		      dc1394bool_t on_off)
{
  quadlet_t curval;
  int retval;
  
  /* ON / OFF : Bit 6 */
  curval = on_off << 25; 
  
  /* Set mirror mode */     
  retval = SetCameraAdvControlRegister(handle, node,REG_CAMERA_MIRROR_IMAGE, curval);
  
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

/************************************************************************/
/* Get DSNU 								*/
/************************************************************************/
int
dc1394_avt_get_dsnu(raw1394handle_t handle, nodeid_t node, 
		    dc1394bool_t *on_off,unsigned int *frame_nb)
{
  quadlet_t value;
  /* Retrieve dsnu parameters */        
  int retval = GetCameraAdvControlRegister(handle, node,REG_CAMERA_DSNU_CONTROL, &value);
  
  if(!retval) {
    
    /* ON / OFF : Bit 6 */
    *on_off = (unsigned int)((value & 0x2000000UL) >> 25); 
    
    /* Number of images : Bits 24..31 */
    *frame_nb =(unsigned int)((value & 0xFFUL));
    
  }       
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
  
}

/************************************************************************/
/* Set DSNU								*/
/************************************************************************/
int
dc1394_avt_set_dsnu(raw1394handle_t handle, nodeid_t node,
		    dc1394bool_t on_off, dc1394bool_t compute, unsigned int frame_nb)
{
  quadlet_t curval;
  int retval;
  /* Retrieve current dsnu parameters */            
  retval = GetCameraAdvControlRegister(handle, node,REG_CAMERA_DSNU_CONTROL, &curval);
  
  /* Compute : Bit 5 */
  curval = (curval & 0xFBFFFFFFUL) | ((compute ) << 26); 
  
  /* ON / OFF : Bit 6 */
  curval = (curval & 0xFDFFFFFFUL) | ((on_off ) << 25); 
  
  /* Number of images : Bits 24..31 */
  curval = (curval & 0xFFFFFF00UL) | ((frame_nb & 0xFFUL ));   
  
  /* Set new dsnu parameters */        
  retval = SetCameraAdvControlRegister(handle, node,REG_CAMERA_DSNU_CONTROL, curval);
  
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

/************************************************************************/
/* Get BLEMISH 								*/
/************************************************************************/
int
dc1394_avt_get_blemish(raw1394handle_t handle, nodeid_t node, 
		       dc1394bool_t *on_off, unsigned int *frame_nb)
{
  quadlet_t value;
  int retval;    
  
  retval = GetCameraAdvControlRegister(handle, node,REG_CAMERA_BLEMISH_CONTROL, &value);
  
  if(!retval) {
    
    /* ON / OFF : Bit 6 */
    *on_off = (unsigned int)((value & 0x2000000UL) >> 25); 
    
    /* Number of images : Bits 24..31 */
    *frame_nb =(unsigned int)((value & 0xFFUL));
    
  }       
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
  
}

/************************************************************************/
/* Set BLEMISH								*/
/************************************************************************/
int
dc1394_avt_set_blemish(raw1394handle_t handle, nodeid_t node,
		       dc1394bool_t on_off, dc1394bool_t compute, unsigned int frame_nb)
{
  quadlet_t curval;
  
  /* Retrieve current blemish parameters */        
  int retval = GetCameraAdvControlRegister(handle, node,REG_CAMERA_BLEMISH_CONTROL, &curval);
  
  /* Compute : Bit 5 */
  curval = (curval & 0xFBFFFFFFUL) | ((compute ) << 26); 
  
  /* ON / OFF : Bit 6 */
  curval = (curval & 0xFDFFFFFFUL) | ((on_off ) << 25); 
  
  /* Number of images : Bits 24..31 */
  curval = (curval & 0xFFFFFF00UL) | ((frame_nb & 0xFFUL ));   
  
  /* Set new blemish parameters */         
  retval = SetCameraAdvControlRegister(handle, node,REG_CAMERA_BLEMISH_CONTROL, curval);
  
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}



/************************************************************************/
/* Get IO	REG_CAMERA_IO_INP_CTRLx	or REG_CAMERA_IO_OUTP_CTRLx	*/
/************************************************************************/
int
dc1394_avt_get_io(raw1394handle_t handle, nodeid_t node, unsigned int IO,
		  dc1394bool_t *polarity, unsigned int *mode,dc1394bool_t *pinstate)
{
  quadlet_t value;
  int retval;    
  
  /* Retrieve IO parameters */        
  retval = GetCameraAdvControlRegister(handle, node,IO, &value);
 
  if(!retval) {
    /* polarity : Bit 7 */
    *polarity = (unsigned int)((value & 0x1000000UL) >> 24); 
    
    /* pinstate : Bit 31 */
    *pinstate = (unsigned int)((value & 0x1UL)); 
    
    /* mode : Bits 11..15 */
    *mode =(unsigned int)((value >> 16 ) & 0x1FUL);
    
  }       
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
  
}

/************************************************************************/
/* Set IO	REG_CAMERA_IO_INP_CTRLx	or REG_CAMERA_IO_OUTP_CTRLx	*/
/************************************************************************/
int
dc1394_avt_set_io(raw1394handle_t handle, nodeid_t node,unsigned int IO,
		  dc1394bool_t polarity, unsigned int mode)
{
  quadlet_t curval;
  int retval;    
  
  /* Retrieve current IO parameters */            
  retval = GetCameraAdvControlRegister(handle, node,IO, &curval);
  
  /* polarity : Bit 7 */
  curval = (curval & 0xFEFFFFFFUL) | ((polarity ) << 24); 
  
  /* mode : Bits 11..15 */
  curval = (curval & 0xFFE0FFFFUL) | ((mode << 16) & 0x1F0000UL );   
  
  /* Set  new IO parameters */            
  retval = SetCameraAdvControlRegister(handle, node,IO, curval);
  
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

/************************************************************************/
/* BusReset IEEE1394							*/
/************************************************************************/
int
dc1394_avt_reset(raw1394handle_t handle, nodeid_t node)
{
  quadlet_t value;
  /* ON / OFF : Bit 6 */
  value= (1<<25) + 200; /*2sec*/
  /* Reset */                    
  int retval = SetCameraAdvControlRegister(handle,node,REG_CAMERA_SOFT_RESET,value); 
  
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}


/************************************************************************/
/* Get Lookup Tables (LUT)						*/
/************************************************************************/
int
dc1394_avt_get_lut(raw1394handle_t handle, nodeid_t node, 
		   dc1394bool_t *on_off, unsigned int *lutnb  )
{
  quadlet_t value;
  int retval;    
  
  /* Retrieve current luts parameters */            
  retval = GetCameraAdvControlRegister(handle, node,REG_CAMERA_LUT_CTRL, &value);
  
  if(!retval) {
    /* Shading ON / OFF : Bit 6 */
    *on_off = (unsigned int)((value & 0x2000000UL) >> 25); 
    
    /* Number of lut : Bits 26..31 */
    *lutnb =(unsigned int)((value & 0x3FUL));
  }       
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
  
}


/************************************************************************/
/* Set Lookup Tables (LUT)						*/
/************************************************************************/
int
dc1394_avt_set_lut(raw1394handle_t handle, nodeid_t node,
		   dc1394bool_t on_off, unsigned int lutnb)
{
  quadlet_t curval;
  int retval;
  
  /* Retrieve current luts parameters */                
  if(GetCameraAdvControlRegister(handle, node,REG_CAMERA_LUT_CTRL, &curval))
    return DC1394_FAILURE;
  
  /* Shading ON / OFF : Bit 6 */
  curval = (curval & 0xFDFFFFFFUL) | ((on_off ) << 25); 
  
  /* Number of lut : Bits 26..31 */
  curval = (curval & 0xFFFFFFB0UL) | ((lutnb & 0x3FUL ));   
  
  /* Set new luts parameters */            
  retval = SetCameraAdvControlRegister(handle, node,REG_CAMERA_LUT_CTRL, curval);
  
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}




/************************************************************************/
/* Get LUT ctrl								*/
/************************************************************************/
int
dc1394_avt_get_lut_mem_ctrl(raw1394handle_t handle, nodeid_t node, dc1394bool_t *en_write, 
			    unsigned int * AccessLutNo,unsigned int *addroffset)
{
  quadlet_t value;
  int retval;    
  
  /* Retrieve current memory luts parameters */                
  retval = GetCameraAdvControlRegister(handle, node,REG_CAMERA_LUT_MEM_CTRL, &value);
  
  if(!retval) {
    /* Enable write access : Bit 5 */
    *en_write = (unsigned int)((value & 0x4000000UL) >> 26); 
    
    /* AccessLutNo : Bits 8..15 */
    *AccessLutNo=(unsigned int)((value >> 16) & 0xFFUL);
    
    /* addroffset in byte : Bits 16..31 */
    *addroffset =(unsigned int)((value & 0xFFFFUL));
  }       
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
  
}


/************************************************************************/
/* Set LUT ctrl							*/
/************************************************************************/
int
dc1394_avt_set_lut_mem_ctrl(raw1394handle_t handle, nodeid_t node,
			    dc1394bool_t en_write, unsigned int AccessLutNo, unsigned int addroffset)
{
  quadlet_t curval;
  int retval;
  
  /* Retrieve current memory luts parameters */                    
  if(GetCameraAdvControlRegister(handle, node,REG_CAMERA_LUT_MEM_CTRL, &curval))
    return DC1394_FAILURE;
  
  /* write access enable : Bit 5 */
  curval = (curval & 0xFBFFFFFFUL) | ((en_write ) << 26); 
  
  /* AccessLutNo : Bits 8..15 */
  curval = (curval & 0xFF00FFFFUL) | ((AccessLutNo << 16) & 0xFF0000UL );   
  
  /* Number of images : Bits 16..31 */
  curval = (curval & 0xFFFF0000UL) | ((addroffset & 0xFFFFUL ));   
  
  /* Set new parameters */     
  retval = SetCameraAdvControlRegister(handle, node,REG_CAMERA_LUT_MEM_CTRL, curval);
  
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}


/************************************************************************/
/* Get LUT  info							*/
/************************************************************************/
int
dc1394_avt_get_lut_info(raw1394handle_t handle, nodeid_t node, unsigned int *NumOfLuts, 
			unsigned int *MaxLutSize)
{
  quadlet_t value;
  /* Retrieve luts info */                
  int retval = GetCameraAdvControlRegister(handle, node,REG_CAMERA_LUT_INFO, &value);

  if(!retval) {

    /* NumOfLuts : Bits 8..15 */
    *NumOfLuts=(unsigned int)((value >> 16) & 0xFFUL);
    
    /* MaxLutSize : Bits 16..31 */
    *MaxLutSize =(unsigned int)((value & 0xFFFFUL));
  }       
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
  
}



/************************************************************************/
/* Get Automatic white balance	with Area Of Interest AOI		*/
/************************************************************************/
int
dc1394_avt_get_aoi(raw1394handle_t handle, nodeid_t node, 
		   dc1394bool_t *on_off, int *left, int *top, int *width, int *height)
{
  quadlet_t value;
  int retval;
  
  /* Retrieve current mode*/                
  retval = GetCameraAdvControlRegister(handle, node,REG_CAMERA_AUTOFNC_AOI, &value);
  
  if(!retval) {
    /*  ON / OFF : Bit 6 */
    *on_off = (unsigned int)((value & 0x2000000UL) >> 25); 
    
    /* Retrieve current size of area*/                      
    retval = GetCameraAdvControlRegister(handle, node,REG_CAMERA_AF_AREA_SIZE, &value);
    
    if(!retval) {
      /* width : Bits 0..15 */
      *width =(unsigned int)(value >> 16);
      /* height : Bits 16..31 */
      *height =(unsigned int)(value & 0xFFFFUL );
      
      /* Retrieve current position of area*/                      	
      retval = GetCameraAdvControlRegister(handle, node,REG_CAMERA_AF_AREA_POSITION, &value);
      if(!retval) {
	/* left : Bits 0..15 */
	*left =(unsigned int)(value >> 16);
	/* top : Bits 16..31 */
	*top =(unsigned int)(value & 0xFFFFUL );
      }
      
    }        
    
  }       
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
  
}

/************************************************************************/
/* Set Automatic white balance with Area Of Interest AOI		*/
/************************************************************************/
int
dc1394_avt_set_aoi(raw1394handle_t handle, nodeid_t node,
		   dc1394bool_t on_off,int left, int top, int width, int height)
{
  quadlet_t curval;
  int retval;
  
  /* ON / OFF : Bit 6 */
  curval = on_off << 25; 
  
  /* Set feature on off */
  retval = SetCameraAdvControlRegister(handle, node,REG_CAMERA_AUTOFNC_AOI, curval);
  
  /* Set size */
  if(!retval)
    retval = SetCameraAdvControlRegister(handle, node,REG_CAMERA_AF_AREA_SIZE, (width << 16) | height); 
  
  /* Set position */  
  if(!retval)
    retval = SetCameraAdvControlRegister(handle, node,REG_CAMERA_AF_AREA_POSITION,(left << 16) | top ); 
  
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

/************************************************************************/
/* Get test_images							*/
/************************************************************************/
int
dc1394_avt_get_test_images(raw1394handle_t handle, nodeid_t node, unsigned int *image_no)
{
  quadlet_t value;
  int retval;    
  
  /* Retrieve test image number */
  retval = GetCameraAdvControlRegister(handle, node,REG_CAMERA_TEST_IMAGE, &value);
  
  if(!retval) {
    /* Numero Image : Bits 28..31 */
    *image_no =(unsigned int)((value & 0xFUL));    
  }
  
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
  
}


/************************************************************************/
/* Set test_images							*/
/************************************************************************/
int
dc1394_avt_set_test_images(raw1394handle_t handle, nodeid_t node, unsigned int image_no)
{
  quadlet_t curval;

  /* Retrieve current test image */
  int retval = GetCameraAdvControlRegister(handle, node,REG_CAMERA_TEST_IMAGE, &curval);    
  
  /* Numero Image : Bits 28..31 */
  curval = (curval & 0xFFFFFFF0UL) | ((image_no & 0xFUL ));   
  
  /* Set new test image */
  retval=SetCameraAdvControlRegister(handle, node,REG_CAMERA_TEST_IMAGE,curval);
  
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}


/************************************************************************/
/* Get frame info							*/
/************************************************************************/
int
dc1394_avt_get_frame_info(raw1394handle_t handle, nodeid_t node, unsigned int *framecounter)
{
  quadlet_t value;
  
  /* Retrieve frame info */
  int retval = GetCameraAdvControlRegister(handle, node,REG_CAMERA_FRAMECOUNTER, &value);
  
  if(!retval) {
    /* framecounter : Bits 0..31 */
    *framecounter =(unsigned int)((value));    
  }
  
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
  
}


/************************************************************************/
/* Reset frame info							*/
/************************************************************************/
int
dc1394_avt_reset_frame_info(raw1394handle_t handle, nodeid_t node)
{
  int retval;
  
  /* Reset counter */
  retval=SetCameraAdvControlRegister(handle, node,REG_CAMERA_FRAMEINFO,1 << 30);
  
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}

/************************************************************************/
/* Get Deferred image transport						*/
/************************************************************************/
int
dc1394_avt_get_deferred_trans(raw1394handle_t handle, nodeid_t node, 
			      dc1394bool_t *HoldImage, dc1394bool_t * FastCapture, unsigned int *FifoSize, 
			      unsigned int *NumOfImages )
{
  quadlet_t value;
  int retval;
  
  /* Retrieve Deferred image transport mode */
  retval = GetCameraAdvControlRegister(handle, node,REG_CAMERA_DEFERRED_TRANS, &value);
  
  if(!retval) {
    /* enable/disable deferred transport mode : Bit 6 */
    *HoldImage = (unsigned int)((value & 0x2000000UL) >> 25); 
    
    /* enable/disable fast capture mode (format 7 only) : Bit 7 */
    *FastCapture = (unsigned int)((value & 0x1000000UL) >> 24); 
    
    /* Size of fifo in number of image : Bits 16..23 */
    *FifoSize =(unsigned int)((value >> 8 & 0xFFUL));
    
    /* Number of images in buffer: Bits 24..31 */
    *NumOfImages =(unsigned int)((value & 0xFFUL));
  }       
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
        
}


/************************************************************************/
/* Set Deferred image transport						*/
/************************************************************************/
int
dc1394_avt_set_deferred_trans(raw1394handle_t handle, nodeid_t node,
			      dc1394bool_t HoldImage,dc1394bool_t  FastCapture, unsigned int FifoSize, 
			      unsigned int NumOfImages, dc1394bool_t SendImage )
{
  quadlet_t curval;
  int retval;
  
  /* Retrieve current image transport mode */
  if(GetCameraAdvControlRegister(handle, node,REG_CAMERA_DEFERRED_TRANS, &curval))
    return DC1394_FAILURE;
  
  /* Send NumOfImages now : Bit 5 */
  curval = (curval & 0xFBFFFFFFUL) | ((SendImage ) << 26); 
  
  /* enable/disable deferred transport mode : Bit 6 */
  curval = (curval & 0xFDFFFFFFUL) | ((HoldImage ) << 25); 
  
  /* enable/disable fast capture mode (format 7 only) : Bit 7 */       
  curval = (curval & 0xFEFFFFFFUL) | ((FastCapture ) << 24);  
  
  /* Size of fifo in number of image : Bits 16..23 */
  curval = (curval & 0xFFFF00FFUL) | (((FifoSize << 8) & 0xFF00UL ));   
  
  /* Number of images : Bits 24..31 */
  curval = (curval & 0xFFFFFF00UL) | ((NumOfImages & 0xFFUL ));   
  
  /* Set new parameters */    
  retval = SetCameraAdvControlRegister(handle, node,REG_CAMERA_DEFERRED_TRANS, curval);
  
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}



/************************************************************************/
/* Get GPData info							*/
/************************************************************************/
int
dc1394_avt_get_gpdata_info(raw1394handle_t handle, nodeid_t node, unsigned int *BufferSize)
{
  quadlet_t value;
  /* Retrieve info on the general purpose buffer */
  int retval = GetCameraAdvControlRegister(handle, node,REG_CAMERA_GPDATA_INFO, &value);
  
  if(!retval) {
    /* BufferSize : Bits 16..31 */
    *BufferSize =(unsigned int)((value & 0xFFFFUL));    
  }
  
  return (retval ? DC1394_FAILURE : DC1394_SUCCESS);
}


/************************************************************************/
/* Get pdata_buffer : experimental, does not work			*/
/************************************************************************/
int
dc1394_avt_get_pdata_buffer(raw1394handle_t handle, nodeid_t node, unsigned int *buff)
{
  return DC1394_FAILURE ;       
}


/************************************************************************/
/* Set pdata_buffer	experimental, does not work			*/
/************************************************************************/
int
dc1394_avt_set_pdata_buffer(raw1394handle_t handle, nodeid_t node, unsigned long buff)
{
  return DC1394_FAILURE;
}


