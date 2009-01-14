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
#ifndef _DC1394_VENDOR_AVT_H
#define _DC1394_VENDOR_AVT_H

#include <libraw1394/raw1394.h>
#include "dc1394_control.h"
#include "dc1394_internal.h"
#include "config.h"

typedef struct __dc1394_avt_adv_feature_info_struct 
{
  unsigned int feature_id;
  dc1394bool_t features_requested;
  dc1394bool_t MaxResolution;
  dc1394bool_t TimeBase;
  dc1394bool_t ExtdShutter;
  dc1394bool_t TestImage;
  dc1394bool_t FrameInfo;
  dc1394bool_t Sequences;
  dc1394bool_t VersionInfo;
  dc1394bool_t Lookup_Tables;
  dc1394bool_t Shading;
  dc1394bool_t DeferredTrans;
  dc1394bool_t HDR_Mode;
  dc1394bool_t DSNU;
  dc1394bool_t TriggerDelay;    
  dc1394bool_t GP_Buffer;
  dc1394bool_t Input_1;    
  dc1394bool_t Input_2;        
  dc1394bool_t Output_1;    
  dc1394bool_t Output_2;        
  dc1394bool_t IntEnaDelay;        
  
} dc1394_avt_adv_feature_info_t;


/************************************************************************/
/* Get Version  	(Read Only)					*/
/*----------------------------------------------------------------------*/
/* Retrieve the firmware version, FPGA version and the camera ID	*/	 
/************************************************************************/
int
dc1394_avt_get_version(raw1394handle_t handle, nodeid_t node, 
		       unsigned int *Version, unsigned int *Camera_ID,
		       unsigned int *FPGA_Version);



/************************************************************************/
/* Get Advanced feature inquiry						*/
/*----------------------------------------------------------------------*/
/* Retrieve the supported features					*/	 
/************************************************************************/
int
dc1394_avt_get_advanced_feature_inquiry(raw1394handle_t handle, nodeid_t node,
					dc1394_avt_adv_feature_info_t *adv_feature);


/************************************************************************/
/* Print Advanced feature 						*/
/*----------------------------------------------------------------------*/
/* Print the supported features requested 				*/	 
/************************************************************************/
int
dc1394_avt_print_advanced_feature(dc1394_avt_adv_feature_info_t *adv_feature);


/************************************************************************/
/* Get the shading mode							*/
/*----------------------------------------------------------------------*/
/* Retrieve if shading is on and the number of frames used to compute 	*/	 
/* The shading reference frame						*/
/************************************************************************/
int dc1394_avt_get_shading(raw1394handle_t handle, nodeid_t node, 
			   dc1394bool_t *on_off, unsigned int *frame_nb);


/************************************************************************/
/* Set the shading mode							*/
/*----------------------------------------------------------------------*/
/* Set the shading to on/off and the number of frames used to compute 	*/	 
/* The shading reference frame						*/
/************************************************************************/
int dc1394_avt_set_shading(raw1394handle_t handle, nodeid_t node,
			   dc1394bool_t on_off, dc1394bool_t compute,
			   unsigned int frame_nb);
		

		
/************************************************************************/
/* Get shading  mem ctrl						*/
/*----------------------------------------------------------------------*/
/* Retrieve write and read access mode of the shading reference frame	*/
/************************************************************************/
int
dc1394_avt_get_shading_mem_ctrl(raw1394handle_t handle, nodeid_t node, 
				dc1394bool_t *en_write, dc1394bool_t *en_read, 
				unsigned int *addroffset);


/************************************************************************/
/* Set shading mem ctrl							*/
/*----------------------------------------------------------------------*/
/* Set write and read access mode of the shading reference frame	*/
/************************************************************************/
int
dc1394_avt_set_shading_mem_ctrl(raw1394handle_t handle, nodeid_t node,
				dc1394bool_t en_write, dc1394bool_t en_read, 
				unsigned int addroffset);


/************************************************************************/
/* Get shading  info							*/
/*----------------------------------------------------------------------*/
/* Retrieve the max size of a shading image				*/
/************************************************************************/
int
dc1394_avt_get_shading_info(raw1394handle_t handle, nodeid_t node, 
			    unsigned int *MaxImageSize);



/************************************************************************/
/* Get Multiple slope parameters					*/
/*----------------------------------------------------------------------*/
/* Retrieve if on/off, the nb of kneepoints used and the 		*/
/* kneepoints values							*/
/************************************************************************/
int
dc1394_avt_get_multiple_slope(raw1394handle_t handle, nodeid_t node, 
			      dc1394bool_t *on_off, unsigned int *points_nb, 
			      unsigned int *kneepoint1, unsigned int *kneepoint2, 
			      unsigned int *kneepoint3);


/************************************************************************/
/* Set Multiple slope parameters					*/
/*----------------------------------------------------------------------*/
/* Set on/off, the nb of kneepoints to use and the 			*/
/* kneepoints values							*/
/************************************************************************/
int
dc1394_avt_set_multiple_slope(raw1394handle_t handle, nodeid_t node, 
			      dc1394bool_t on_off, unsigned int points_nb, 
			      unsigned int kneepoint1, unsigned int kneepoint2, 
			      unsigned int kneepoint3);



/************************************************************************/
/* Get Shutter Timebase 						*/
/*----------------------------------------------------------------------*/
/* Get the timebase value with an Id. See Manual for correspondance	*/
/************************************************************************/
int
dc1394_avt_get_timebase(raw1394handle_t handle, nodeid_t node, 
			unsigned int *timebase_id  );


/************************************************************************/
/* Set Shutter Timebase 						*/
/*----------------------------------------------------------------------*/
/* Set the timebase value with an Id. See Manual for correspondance	*/
/************************************************************************/
int
dc1394_avt_set_timebase(raw1394handle_t handle, nodeid_t node,
			unsigned int timebase_id);



/************************************************************************/
/* Get Extented Shutter  						*/
/*----------------------------------------------------------------------*/
/* Get the extented shutter value in us					*/
/************************************************************************/
int
dc1394_avt_get_extented_shutter(raw1394handle_t handle, nodeid_t node, 
				unsigned int *timebase_id);


/************************************************************************/
/* Set Extented shutter							*/
/*----------------------------------------------------------------------*/
/* Set the extented shutter value in us					*/
/************************************************************************/
int
dc1394_avt_set_extented_shutter(raw1394handle_t handle, nodeid_t node, 
				unsigned int timebase_id);



/************************************************************************/
/* Get MaxResolution  	(Read Only)					*/
/*----------------------------------------------------------------------*/
/* Get the Max reachable resolution 					*/
/************************************************************************/
int
dc1394_avt_get_MaxResolution(raw1394handle_t handle, nodeid_t node, 
			     unsigned int *MaxHeight, unsigned int *MaxWidth);



/************************************************************************/
/* Get Auto Shutter  							*/
/*----------------------------------------------------------------------*/
/* Get min and max shutter values for autoshutter			*/
/************************************************************************/
int
dc1394_avt_get_auto_shutter(raw1394handle_t handle, nodeid_t node, 
			    unsigned int *MinValue, unsigned int *MaxValue);


/************************************************************************/
/* Set Auto shutter							*/
/*----------------------------------------------------------------------*/
/* Set min and max shutter values for autoshutter			*/
/************************************************************************/
int
dc1394_avt_set_auto_shutter(raw1394handle_t handle, nodeid_t node,
			    unsigned int MinValue, unsigned int MaxValue);



/************************************************************************/
/* Get Auto gain							*/
/*----------------------------------------------------------------------*/
/* Get min and max gain values for autogain				*/
/************************************************************************/
int
dc1394_avt_get_auto_gain(raw1394handle_t handle, nodeid_t node, 
			 unsigned int *MinValue, unsigned int *MaxValue);


/************************************************************************/
/* Set Auto gain							*/
/*----------------------------------------------------------------------*/
/* Set min and max gain values for autogain				*/
/************************************************************************/
int
dc1394_avt_set_auto_gain(raw1394handle_t handle, nodeid_t node,
			 unsigned int MinValue, unsigned int MaxValue);



/************************************************************************/
/* Get Trigger delay							*/
/*----------------------------------------------------------------------*/
/* Get if trigger delay on and the trigger delay			*/
/************************************************************************/
int
dc1394_avt_get_trigger_delay(raw1394handle_t handle, nodeid_t node, 
			     dc1394bool_t *on_off, unsigned int *DelayTime);
		

/************************************************************************/
/* Set Trigger delay							*/
/*----------------------------------------------------------------------*/
/* Set trigger delay on/off  and the trigger delay value		*/
/************************************************************************/
int
dc1394_avt_set_trigger_delay(raw1394handle_t handle, nodeid_t node,
			     dc1394bool_t on_off, unsigned int DelayTime);



/************************************************************************/		
/* Get Mirror 								*/
/*----------------------------------------------------------------------*/
/* Get mirror mode							*/
/************************************************************************/
int
dc1394_avt_get_mirror(raw1394handle_t handle, nodeid_t node, 
		      dc1394bool_t *on_off);


/************************************************************************/
/* Set Mirror								*/
/*----------------------------------------------------------------------*/
/* Set mirror mode							*/
/************************************************************************/
int
dc1394_avt_set_mirror(raw1394handle_t handle, nodeid_t node,
		      dc1394bool_t on_off);



/************************************************************************/
/* Get DSNU 								*/
/*----------------------------------------------------------------------*/
/* Get DSNU mode and num of frames used for computing  dsnu correction  */
/************************************************************************/
int
dc1394_avt_get_dsnu(raw1394handle_t handle, nodeid_t node, 
		    dc1394bool_t *on_off, unsigned int *frame_nb);


/************************************************************************/
/* Set DSNU								*/
/*----------------------------------------------------------------------*/
/* Set DSNU mode, number of frames used for computing 			*/
/*  and launch the the computation of the dsnu frame			*/
/************************************************************************/
int
dc1394_avt_set_dsnu(raw1394handle_t handle, nodeid_t node,
		    dc1394bool_t on_off, dc1394bool_t compute,
		    unsigned int frame_nb);

		

/************************************************************************/
/* Get BLEMISH 								*/
/*----------------------------------------------------------------------*/
/* Get Blemish mode and num of frames used for computing the correction */
/************************************************************************/
int
dc1394_avt_get_blemish(raw1394handle_t handle, nodeid_t node, 
		       dc1394bool_t *on_off, unsigned int *frame_nb);
		
		
/************************************************************************/
/* Set BLEMISH								*/
/*----------------------------------------------------------------------*/
/* Set Blemish mode, num of frames used for computing 			*/
/*  and launch the the computation of the blemish correction		*/
/************************************************************************/
int
dc1394_avt_set_blemish(raw1394handle_t handle, nodeid_t node,
		       dc1394bool_t on_off, dc1394bool_t compute,
		       unsigned int frame_nb);



/************************************************************************/
/* Get IO	REG_CAMERA_IO_INP_CTRLx	or REG_CAMERA_IO_OUTP_CTRLx	*/
/*----------------------------------------------------------------------*/
/*  Get the polarity, the mode, the state of the IO			*/
/************************************************************************/
int
dc1394_avt_get_io(raw1394handle_t handle, nodeid_t node, unsigned int IO,
		  dc1394bool_t *polarity, unsigned int *mode, dc1394bool_t *pinstate);


/************************************************************************/
/* Set IO	REG_CAMERA_IO_INP_CTRLx	or REG_CAMERA_IO_OUTP_CTRLx	*/
/*----------------------------------------------------------------------*/
/*  Set the polarity and the mode of the IO				*/
/************************************************************************/
int
dc1394_avt_set_io(raw1394handle_t handle, nodeid_t node,unsigned int IO,
		  dc1394bool_t polarity, unsigned int mode);
		


/************************************************************************/
/* BusReset IEEE1394							*/
/*----------------------------------------------------------------------*/
/* Reset the bus and the fpga						*/
/************************************************************************/
int
dc1394_avt_reset(raw1394handle_t handle, nodeid_t node);



/************************************************************************/
/* Get Lookup Tables (LUT)						*/
/*----------------------------------------------------------------------*/
/* Get on/off and the num of the current lut loaded			*/
/************************************************************************/
int
dc1394_avt_get_lut(raw1394handle_t handle, nodeid_t node, 
		   dc1394bool_t *on_off, unsigned int *lutnb  );
		

/************************************************************************/
/* Set Lookup Tables (LUT)						*/
/*----------------------------------------------------------------------*/
/* Set on/off and the num of the current lut to load			*/
/************************************************************************/
int
dc1394_avt_set_lut(raw1394handle_t handle, nodeid_t node,
		   dc1394bool_t on_off, unsigned int lutnb);


/************************************************************************/
/* Get LUT ctrl								*/
/*----------------------------------------------------------------------*/
/* Get access mode of a lut						*/
/************************************************************************/
int
dc1394_avt_get_lut_mem_ctrl(raw1394handle_t handle, nodeid_t node, 
			    dc1394bool_t *en_write, unsigned int * AccessLutNo,
			    unsigned int *addroffset);


/************************************************************************/
/* Set LUT ctrl								*/
/*----------------------------------------------------------------------*/
/* Set access mode of a lut						*/
/************************************************************************/

int
dc1394_avt_set_lut_mem_ctrl(raw1394handle_t handle, nodeid_t node,
			    dc1394bool_t en_write, unsigned int AccessLutNo, 
			    unsigned int addroffset);
	
		
/************************************************************************/
/* Get LUT  info							*/
/*----------------------------------------------------------------------*/
/* Get num of luts present and the max size				*/
/************************************************************************/
int
dc1394_avt_get_lut_info(raw1394handle_t handle, nodeid_t node, 
			unsigned int *NumOfLuts, unsigned int *MaxLutSize);


/************************************************************************/
/* Get Automatic white balance with Area Of Interest AOI		*/
/*----------------------------------------------------------------------*/
/* Get on/off and area							*/
/************************************************************************/

int
dc1394_avt_get_aoi(raw1394handle_t handle, nodeid_t node, 
		   dc1394bool_t *on_off, int *left, int *top, 
		   int *width, int *height);

/************************************************************************/
/* Set Automatic white balance with Area Of Interest AOI		*/
/*----------------------------------------------------------------------*/
/* Set on/off and area							*/
/************************************************************************/
int
dc1394_avt_set_aoi(raw1394handle_t handle, nodeid_t node,
		   dc1394bool_t on_off,int left, int top, int width, int height);



/************************************************************************/
/* Get test_images							*/
/*----------------------------------------------------------------------*/
/* Get current test image						*/
/************************************************************************/
int
dc1394_avt_get_test_images(raw1394handle_t handle, nodeid_t node, 
			   unsigned int *image_no);


/************************************************************************/
/* Set test_images							*/
/*----------------------------------------------------------------------*/
/* Set num of test image						*/
/************************************************************************/
int
dc1394_avt_set_test_images(raw1394handle_t handle, nodeid_t node, 
			   unsigned int image_no);



/************************************************************************/
/* Get frame info							*/
/*----------------------------------------------------------------------*/
/* Get the number of captured frames					*/
/************************************************************************/
int
dc1394_avt_get_frame_info(raw1394handle_t handle, nodeid_t node, 
			  unsigned int *framecounter);


/************************************************************************/
/* Reset frame info 							*/
/*----------------------------------------------------------------------*/
/* Reset frame counter							*/
/************************************************************************/
int
dc1394_avt_reset_frame_info(raw1394handle_t handle, nodeid_t node);
		


/************************************************************************/
/* Get GPData info							*/
/*----------------------------------------------------------------------*/
/* Get the size of the buffer						*/
/************************************************************************/
int
dc1394_avt_get_gpdata_info(raw1394handle_t handle, nodeid_t node, 
			   unsigned int *BufferSize);		



/************************************************************************/
/* Get Deferred image transport						*/
/*----------------------------------------------------------------------*/
/* Get the fifo control mode						*/
/************************************************************************/
int
dc1394_avt_get_deferred_trans(raw1394handle_t handle, nodeid_t node, 
			      dc1394bool_t *HoldImage,dc1394bool_t * FastCapture, 
			      unsigned int *FifoSize, unsigned int *NumOfImages );
		
		
/************************************************************************/
/* Set Deferred image transport						*/
/*----------------------------------------------------------------------*/
/* Set the fifo control mode						*/
/************************************************************************/
int
dc1394_avt_set_deferred_trans(raw1394handle_t handle, nodeid_t node,
			      dc1394bool_t HoldImage,dc1394bool_t  FastCapture, 
			      unsigned int FifoSize, unsigned int NumOfImages, 
			      dc1394bool_t SendImage );



/************************************************************************/
/* Get pdata_buffer : DOESNT WORK					*/
/*----------------------------------------------------------------------*/
/* will Get the  buffer	...						*/
/************************************************************************/
int
dc1394_avt_get_pdata_buffer(raw1394handle_t handle, nodeid_t node, 
			    unsigned int *buff);


/************************************************************************/
/* Set pdata_buffer : DOESNT WORK					*/
/*----------------------------------------------------------------------*/
/* will Set the  buffer	...						*/
/************************************************************************/
int
dc1394_avt_set_pdata_buffer(raw1394handle_t handle, nodeid_t node, 
			    unsigned long buff);


#endif
