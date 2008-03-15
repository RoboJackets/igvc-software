/*
 * IIDC1394-Based Digital Camera Control Library Extension for AVT Cameras
 *
 * Written by Menuka (and extended by Georg)
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

#ifndef _AVT1394_H
#define _AVT1394_H


#include "libdc1394/dc1394_control.h"
#include <errno.h>



/*Defines for timebase value */
#define AVT_TIMEBASE_1              1
#define AVT_TIMEBASE_2              2
#define AVT_TIMEBASE_5              5
#define AVT_TIMEBASE_10            10
#define AVT_TIMEBASE_20            20
#define AVT_TIMEBASE_50            50
#define AVT_TIMEBASE_100          100
#define AVT_TIMEBASE_200          200
#define AVT_TIMEBASE_500          500
#define AVT_TIMEBASE_1000        1000

/*Defines for camera type */
/* Dolphin */
#define AVT_CAMERA_F145B             1
#define AVT_CAMERA_F145C             2
#define AVT_CAMERA_F201B             3
#define AVT_CAMERA_F201C             4
#define AVT_CAMERA_F145B1            5
#define AVT_CAMERA_F145C1            6
#define AVT_CAMERA_F201B1            7
#define AVT_CAMERA_F201C1            8
/* Marlin */
#define AVT_CAMERA_MF033B            9
#define AVT_CAMERA_MF033C           10
#define AVT_CAMERA_MF046B           11
#define AVT_CAMERA_MF046C           12
#define AVT_CAMERA_MF080B           13
#define AVT_CAMERA_MF080C           14
#define AVT_CAMERA_MF145B2          15
#define AVT_CAMERA_MF145C2          16
#define AVT_CAMERA_MF131B           17
#define AVT_CAMERA_MF131C           18
#define AVT_CAMERA_MF145B215        19
#define AVT_CAMERA_MF145C215        20
/*  Marlin2 */
#define AVT_CAMERA_M2F033B          21
#define AVT_CAMERA_M2F033C	    22
#define AVT_CAMERA_M2F046B          23
#define AVT_CAMERA_M2F046C	    24
#define AVT_CAMERA_M2F080B          25
#define AVT_CAMERA_M2F080C	    26
#define AVT_CAMERA_M2F145B2         27
#define AVT_CAMERA_M2F145C2         28
#define AVT_CAMERA_M2F145B215       31
#define AVT_CAMERA_M2F145C215       32
#define AVT_CAMERA_M2F080B30        43
#define AVT_CAMERA_M2F080C30        44
/*  Oscar */
#define AVT_CAMERA_OF320C           38
#define AVT_CAMERA_OF510C           40
#define AVT_CAMERA_OF810C           42
/*  MIN/MAX */
#define AVT_CAMERA_MIN              AVT_CAMERA_F145B
#define AVT_CAMERA_MAX              AVT_CAMERA_M2F080C30

/* define of Camera advanced features for ADV_INQ1 set*/
#define AVT_MAXRES_FEATURE           31
#define AVT_TIMEBASE_FEATURE         30
#define AVT_EXTDSHUTTER_FEATURE      29
#define AVT_TESTIMAGE_FEATURE        28
#define AVT_FRAMEINFO_FEATURE        27
#define AVT_SEQUENCES_FEATURE        26
#define AVT_VERSIONINFO_FEATURE      25
#define AVT_LUT_FEATURE              23
#define AVT_SHADING_FEATURE          22
#define AVT_DEFERREDTRANS_FEATURE    21
#define AVT_HDR_FEATURE              20
#define AVT_FPN_FEATURE              19
#define AVT_TRIGGER_FEATURE          18
#define AVT_MISC_FEATURE             17
#define AVT_SOFT_RESET_FEATURE       16
#define AVT_HIGH_SNR_FEATURE         15
#define AVT_COLOR_CORR_FEATURE       14
#define AVT_GPBUFFER_FEATURE          0



/* defines of Camera advanced features for ADV_INQ2 set*/
#define AVT_INP1_FEATURE             31
#define AVT_INP2_FEATURE             30
#define AVT_INP3_FEATURE             29
#define AVT_OUTP1_FEATURE            23
#define AVT_OUTP2_FEATURE            22
#define AVT_OUTP3_FEATURE            21
#define AVT_INTENADELAY_FEATURE      15     
#define AVT_INCDECODER_FEATURE       14    

/*define for internal use*/
#define AVT_AVAILABLE                0x80000000UL

/*define for avt hardware baud */
#define AVT_HW_BAUD_300               0
#define AVT_HW_BAUD_600               1
#define AVT_HW_BAUD_1200              2
#define AVT_HW_BAUD_2400              3
#define AVT_HW_BAUD_4800              4
#define AVT_HW_BAUD_9600              5
#define AVT_HW_BAUD_19200             6
#define AVT_HW_BAUD_38400             7
#define AVT_HW_BAUD_57600             8
#define AVT_HW_BAUD_115200            9
#define AVT_HW_BAUD_230400            10

/*defines for avt hardware charlen*/
#define AVT_HW_7_CHAR                  7
#define AVT_HW_8_CHAR                  8

/*defines for avt parity*/
#define AVT_HW_NO_PARITY               0
#define AVT_HW_ODD_PARITY              1  
#define AVT_HW_EVEN_PARITY             2


/*defines for avt stop bits*/
#define AVT_HW_ONE_STOPBIT             0
#define AVT_HW_ONEHALF_STOPBIT         1
#define AVT_HW_TWO_STOPBIT             2

/* flags for DSNU / BLEMISH control */
#define AVT_DSNU DC1394_TRUE
#define AVT_BLEMISH DC1394_FALSE


/* enum of Camera inquiry  set*/
enum 
  {
    AVT_ADV_INQ_1,
    AVT_ADV_INQ_2
  };

/* enum of Timebase id  set*/
enum 
  {
    AVT_TIMEID0 = 0,
    AVT_TIMEID1,
    AVT_TIMEID2,
    AVT_TIMEID3,
    AVT_TIMEID4,
    AVT_TIMEID5,
    AVT_TIMEID6,
    AVT_TIMEID7,
    AVT_TIMEID8,
    AVT_TIMEID9

  };
/* enums for Input/Output Pin control */
enum
  {
    AVT_IO_INP_CTRL1,
    AVT_IO_INP_CTRL2,
    AVT_IO_INP_CTRL3,
    AVT_IO_OUT_CTRL1,
    AVT_IO_OUT_CTRL2,
    AVT_IO_OUT_CTRL3
  };

/*enums for Baud Rate */
enum
  {
    AVT_BAUD_300   = 300,
    AVT_BAUD_600   = 600,
    AVT_BAUD_1200  = 1200,
    AVT_BAUD_2400  = 2400,
    AVT_BAUD_4800  = 4800,
    AVT_BAUD_9600  = 9600,
    AVT_BAUD_19200 = 19200,
    AVT_BAUD_38400 = 38400,
    AVT_BAUD_57600 = 57600,
    AVT_BAUD_115200 = 115200,
    AVT_BAUD_230400 = 230400
  };

/*enums for Character length*/
enum
  {
    AVT_7_CHAR = 7,
    AVT_8_CHAR =8
  };

/*enums for Parity*/
enum
  {
    AVT_NO_PARITY,
    AVT_ODD_PARITY,
    AVT_EVEN_PARITY    
  };

/*enums for StopBits*/
enum
  {
    AVT_ONE_STOPBIT,
    AVT_ONEHALF_STOPBIT,
    AVT_TWO_STOPBIT
  };

/* structure for firmware version*/
typedef struct
{
  unsigned int nCameraId;
  unsigned int nMajor;
  unsigned int nMinor;
  dc1394bool_t bMarlin;			     
}avt_firmware_t;

/* structure for dsnu blemish*/
typedef struct
{
  dc1394bool_t bType;
  dc1394bool_t bShowImg;
  dc1394bool_t bCompute;
  dc1394bool_t bBusy;
  dc1394bool_t bLoadData;
  dc1394bool_t bZero;
  unsigned int nNumImg;
}avt_dsnu_blemish_t;

/*structure for serial mode*/
typedef struct
{
  unsigned int nBaudRate;
  unsigned int nCharLen;
  unsigned int nParity;
  unsigned int nStopBit;
  unsigned int nBufferSize;  
}avt_serial_mode_t;


/*structure for serial control information*/
typedef struct
{
  dc1394bool_t bRcvEnable;
  dc1394bool_t bTxEnable;
  dc1394bool_t bTxBufferReady;
  dc1394bool_t bRcvBufferReady;
  dc1394bool_t bRcvOverrunError;
  dc1394bool_t bFramingError;
  dc1394bool_t bParityError;
}avt_serial_control_t;


/* structure for serial buffer characters*/

typedef struct
{
  int nChar0;
  int nChar1;
  int nChar2;
  int nChar3;
}avt_serial_data_t;



#ifdef __cplusplus
extern "C" {
#endif


/********************************************************
 AvtDCGetCameraControlRegister
 AvtGetCameraControlRegister
 AvtDCSetCameraControlRegister
 AvtSetCameraControlRegister

 Routines for direct access to the
 camera-control-registers
*********************************************************/
  int
  AvtDCGetCameraControlRegister(raw1394handle_t handle, nodeid_t node,
				octlet_t offset, quadlet_t *value);

  int
  AvtGetCameraControlRegister(raw1394handle_t handle, nodeid_t node,
			      octlet_t offset, quadlet_t *value,
			      dc1394bool_t bRS232);

  int
  AvtDCSetCameraControlRegister(raw1394handle_t handle, nodeid_t node,
				octlet_t offset, quadlet_t value);

  int
  AvtSetCameraControlRegister(raw1394handle_t handle, nodeid_t node,
			      octlet_t offset, quadlet_t value,
			      dc1394bool_t bRS232);

/********************************************************
 avt1394_get_uc_version

 This routine reports the version of microprocessor
*********************************************************/
  int
  avt1394_get_uc_version(raw1394handle_t handle, nodeid_t node,
			 unsigned int *pMajor,unsigned int *pMinor);

/********************************************************
 avt1394_get_firmware_version

 This routine reports the firmware version of FPGA and camera
 type.Value of pMarlin indicates whether camera is of Marlin
 series or dolphin series
*********************************************************/
  int
  avt1394_get_firmware_version(raw1394handle_t handle, nodeid_t node,
			       avt_firmware_t *pData);

/********************************************************
 avt1394_is_adv_inq1_feature

 This routine reports whether advanced feature inquiry1
 is present
*********************************************************/
  int 
  avt1394_is_adv_inq1_feature(raw1394handle_t handle, nodeid_t node,
			      unsigned int nFeature,dc1394bool_t *pPresent);

/********************************************************
 avt1394_is_adv_inq2_feature

 This routine reports whether the advanced inqury2 
 feature is present  or not
*********************************************************/
  int 
  avt1394_is_adv_inq2_feature(raw1394handle_t handle, nodeid_t node,
			      unsigned int nFeature, dc1394bool_t *pPresent);

/********************************************************
 avt1394_get_max_resolution

 This routine retrieves the max resolution. Applicable
 only for format 7 mode 0
**********************************************************/
  int 
  avt1394_get_max_resolution(raw1394handle_t handle, nodeid_t node,
			     unsigned int *pWidth, unsigned int * pHeight);

/********************************************************
 avt1394_get_timebase

 This routine reports the current timebase which is in usec
*********************************************************/
  int 
  avt1394_get_timebase(raw1394handle_t handle, nodeid_t node,
		       unsigned int *pValue);

/********************************************************
 avt1394_set_timebase

 This routine sets the value of  timebase which is in usec
*********************************************************/
  int 
  avt1394_set_timebase(raw1394handle_t handle, nodeid_t node,
		       unsigned int Value);

/********************************************************
 avt1394_get_exposure_time

 This routine reports the currently used exposure time
 in usec.
 It therefore returns the value of the EXTD_SHUTTER register,
 if this available. EXTD_SHUTTER reflects the current
 exposure time in usec (without the camera-specific offset),
 because a change of the standard SHUTTER register will also
 change the value of this register. If the TIMEBASE register
 has been changed, the exposure time will not be changed
 until the next change of the SHUTTER register (or a change
 of the EXTD_SHUTTER register).
 If no EXTD_SHUTTER register is availble the returned value
 is calculated by either:
  SHUTTER * TIMEBASE + OFFSET
 or:
  SHUTTER + OFFSET (in case that no TIMEBASE is available)
 This may not be the real shutter speed currently in use
 (see above for details), therefore the 'Valid' parameter is
 set to 'false'.
 Unfortunately the camera-specific OFFSET is only known
 for (some) MARLIN-cameras (for more details see
  'avt1394_get_offset()')
*********************************************************/
  int 
  avt1394_get_exposure_time(raw1394handle_t handle, nodeid_t node,
			    unsigned int *pValue, dc1394bool_t *pValid);

/********************************************************
 avt1394_set_exposure_time

 This routine sets the exposure time in usec.
 If possible it will use a combination of SHUTTER and
 TIMEBASE (if available), EXTD_SHUTTER otherwise.
 It will also take into account the camera specific
 offset!
*********************************************************/
  int 
  avt1394_set_exposure_time(raw1394handle_t handle, nodeid_t node,
			    unsigned int nValue);

/********************************************************
 avt1394_get_exposure_offset

 This routine reports the 'exposure offset' from the
 timebase-register. !No firmware-version-check is performed!
*********************************************************/
  int 
  avt1394_get_exposure_offset(raw1394handle_t handle, nodeid_t node,
			      unsigned int *pValue);

/********************************************************
 avt1394_get_extended_shutter

 This routine reports the raw value from the IIDC extended
 shutter register (without the camera-specific offset)
*********************************************************/
  int 
  avt1394_get_extended_shutter(raw1394handle_t handle, nodeid_t node,
			       unsigned int *pValue);

/********************************************************
 avt1394_set_extended_shutter

 This routine sets the extended shutter register value
*********************************************************/
  int 
  avt1394_set_extended_shutter(raw1394handle_t handle, nodeid_t node,
			       unsigned int Value);

/********************************************************
 avt1394_get_shutter_offset

 This routine returns the offset for some known cameras.
 Unfortunately there is no register (or the like) available
 for getting the correct value directly from the camera.
 Only the offsets for the MARLIN cameras are know:
  F033    24us
  F046    24us
  F080    43us
  F131    <1us
  F145    43us
*********************************************************/
  void
  avt1394_get_shutter_offset(raw1394handle_t handle, nodeid_t node,
			     unsigned int *pOffset);

/********************************************************
 avt1394_get_test_image

 This routine reports which test images are present and
 active (max. 7 images).
 E.g., if the value of 'pPresent' is 0x1010111, this would
 mean that the test images 1,3,5,6 and 7 are present and
 the remaining ones are not available.
 pActive indicates the number of the active test image,
 i.e., if pActive is 0 no test image is currently active
*********************************************************/
  int 
  avt1394_get_test_image(raw1394handle_t handle, nodeid_t node,
			 unsigned int *pPresent, unsigned int *pActive);

/********************************************************
 avt1394_set_test_image

 This routine activates test image if present
*********************************************************/
  int 
  avt1394_set_test_image(raw1394handle_t handle, nodeid_t node,
			 unsigned int nActive);

/********************************************************
 avt1394_get_seq_info

 This routine retrieves the sequence information i.e. 
 Maximum Length
**********************************************************/
  int 
  avt1394_get_seq_info(raw1394handle_t handle, nodeid_t node,
		       unsigned int *pMaxLength,
		       dc1394bool_t *pApply, dc1394bool_t *on);

/********************************************************
 avt1394_enable_sequence

 This routine enables or disables  sequence
**********************************************************/
  int 
  avt1394_enable_sequence(raw1394handle_t handle, nodeid_t node,
			  dc1394bool_t bEnable);

/********************************************************
 avt1394_set_seq_param

 This routine sets the sequence parameters
**********************************************************/
  int 
  avt1394_set_seq_param(raw1394handle_t handle, nodeid_t node,
			dc1394bool_t bAutoRewind,
			unsigned int nSeqLenght,
			dc1394bool_t bIncImageNo,
			unsigned int nImageNo);

/********************************************************
 avt1394_get_seq_param

 This routine returns the current sequence parameters
**********************************************************/
  int 
  avt1394_get_seq_param(raw1394handle_t handle, nodeid_t node,
			dc1394bool_t *pAutoRewind,
			unsigned int *pSeqLenght,
			dc1394bool_t *pIncImageNo,
			unsigned int *pImageNo);

/********************************************************
 avt1394_apply_seq_param

 This routine applies the current sequence parameters
**********************************************************/
  int 
  avt1394_apply_seq_param(raw1394handle_t handle, nodeid_t node,
			  dc1394bool_t autoInc);

/********************************************************
 avt1394_get_gpdata_size

 This routine retrieves data buffer size
**********************************************************/
  int 
  avt1394_get_gpdata_size(raw1394handle_t handle, nodeid_t node,
			  unsigned int *pSize);

/********************************************************
 avt1394_get_lut_info

 This routine retrieves lookup table information
**********************************************************/
  int 
  avt1394_get_lut_info(raw1394handle_t handle, nodeid_t node,
		       unsigned int *pMaxLutNo,
		       unsigned int *pMaxLutSize,
		       dc1394bool_t *on);

/********************************************************
 avt1394_select_lut

 This routine selects the lookup table by setting lookup
 table number (after testing that the number is 'in-range')
**********************************************************/
  int 
  avt1394_select_lut(raw1394handle_t handle, nodeid_t node,
		     unsigned int nLutNo);

/********************************************************
 avt1394_get_lut_selection

 This routine returns the number of the selected LUT
**********************************************************/
  int 
  avt1394_get_lut_selection(raw1394handle_t handle, nodeid_t node,
			    unsigned int *nLutNo);

/********************************************************
 avt1394_enable_lut

 This routine enables / disables the selected lookup table
**********************************************************/
  int 
  avt1394_enable_lut(raw1394handle_t handle, nodeid_t node,
		     dc1394bool_t enable);

/********************************************************
 avt1394_load_lut

 This routine loads a shading image into the camera
**********************************************************/
  int 
  avt1394_load_lut(raw1394handle_t handle, nodeid_t node,
		   unsigned int nLutno,unsigned char *pData,
		   unsigned int nSize);

/********************************************************
 avt1394_get_shading_info

 This routine retrieves shading information
**********************************************************/
  int 
  avt1394_get_shading_info(raw1394handle_t handle, nodeid_t node,
			   dc1394bool_t *pShowImg, dc1394bool_t *pBuildImg,
			   dc1394bool_t *pBusy, unsigned int *pNumImg,
			   unsigned int *pMaxImg, dc1394bool_t *on);

/********************************************************
 avt1394_get_max_shading__img_size

 This routine retrieves the max. shading image size
**********************************************************/
  int 
  avt1394_get_max_shading_img_size(raw1394handle_t handle, nodeid_t node,
				   unsigned int *pMaxImg);

/********************************************************
 avt1394_show_shading_img

 This routine sets the parameters for shading correction
**********************************************************/
  int 
  avt1394_show_shading_img(raw1394handle_t handle, nodeid_t node,
			   dc1394bool_t bShowImg);

/********************************************************
 avt1394_load_shading

 This routine loads a shading image into the camera
**********************************************************/
  int 
  avt1394_load_shading(raw1394handle_t handle, nodeid_t node,
		       unsigned char *pData,unsigned int nSize);

/********************************************************
 avt1394_read_shading

 This routine reads a shading image from the camera
**********************************************************/
  int 
  avt1394_read_shading(raw1394handle_t handle, nodeid_t node,
		       unsigned char *pData,unsigned int nSize);

/********************************************************
 avt1394_enable_shading

 Enables / Disables the shading correction
**********************************************************/
  int 
  avt1394_enable_shading(raw1394handle_t handle, nodeid_t node,
			 dc1394bool_t bEnable);

/********************************************************
 avt1394_build_shading_image

 This routine starts the building of a new shading image
**********************************************************/
  int 
  avt1394_build_shading_image(raw1394handle_t handle, nodeid_t node,
			      unsigned int grabCount);

/********************************************************
 avt1394_set_shading_grab_count

 This routine sets the GrabCount parameter of the
 Shading Correction feature
**********************************************************/
  int 
  avt1394_set_shading_grab_count(raw1394handle_t handle, nodeid_t node,
				 unsigned int grabCount);

/********************************************************
 avt1394_get_deferr_trans

 This routine retrieves deferred transport information
**********************************************************/
  int 
  avt1394_get_deferr_trans(raw1394handle_t handle, nodeid_t node,
			   dc1394bool_t *pSendImg, dc1394bool_t *pHoldImg,
			   dc1394bool_t *pFastCapture, unsigned int *pFifoSize,
			   unsigned int *pNumImg);

/********************************************************
 avt1394_set_deferr_trans

 This routine sets deferred transport ie either enabling 
 or disabling it,enabling or disabling HoldImg mode,
 enabling or disabling Fastcapture mode
**********************************************************/
  int 
  avt1394_set_deferr_trans(raw1394handle_t handle, nodeid_t node,
			   dc1394bool_t bSendImg, dc1394bool_t bHoldImg,
			   dc1394bool_t bFastCapture, unsigned int nNumImg);

/********************************************************
 avt1394_get_frame_info

 This routine retrieves frame counter  ie number of captured
 frames since last reset
**********************************************************/
  int 
  avt1394_get_frame_info(raw1394handle_t handle, nodeid_t node,
			 unsigned int *pFrameCtr);

/********************************************************
 avt1394_reset_frame_counter

 This routine resets the frame counter
**********************************************************/
  int 
  avt1394_reset_frame_counter(raw1394handle_t handle, nodeid_t node);

/********************************************************
 avt1394_get_hdr_info

 This routine retrieves max. number knee points,
 kneepoint values and the current state (on/off)
**********************************************************/
  int 
  avt1394_get_hdr_info(raw1394handle_t handle, nodeid_t node,
		       unsigned int *pMaxKnee,
		       unsigned int *pKnee1,
		       unsigned int *pKnee2,
		       unsigned int *pKnee3,
		       dc1394bool_t *on);


/********************************************************
 avt1394_get_active_knees

 This routine returns the number of activated knees.
 In the first version of the library, this was a parameter
 of 'avt1394_get_hdr_info()', but it was removed from the
 second release, because of ambiguous information from the
 available manuals. Because the first assumption was correct
 this function is now added. It was done as an extra function
 mainly for compatibility reasons.
 So, if you want to read out the parameters of the cameras
 kneepoint values, you have to call both 'avt1394_get_hdr_info()'
 for getting the values and 'avt1394_get_active_knees()' for
 getting the information about which values are currently active.
**********************************************************/
  int 
  avt1394_get_active_knees(raw1394handle_t handle, nodeid_t node,
			   unsigned int *pActiveKnees);


/********************************************************
 avt1394_set_knee_values

 This routine sets the number of active knee points
 and their values without influencing the state of the
 HDR-mode (this has to be done with 'avt1394_enable_hdr_mode()')
**********************************************************/
  int 
  avt1394_set_knee_values(raw1394handle_t handle, nodeid_t node,
			  unsigned int nActiveKnee, unsigned int dKnee1,
			  unsigned int dKnee2, unsigned int dKnee3);

/********************************************************
 avt1394_set_knee_values

 This routine sets the number of active knee points
 and their values relative to the current shutter (%)
 without influencing the state of the HDR-mode (this
  has to be done with 'avt1394_enable_hdr_mode()')
**********************************************************/
  int 
  avt1394_set_knee_values_percent(raw1394handle_t handle, nodeid_t node,
				  unsigned int nActiveKnee, unsigned int dPerKnee1,
				  unsigned int dPerKnee2, unsigned int dPerKnee3);

/********************************************************
 avt1394_enable_hdr_mode

 This routine enables / disables the hdr mode
 depending on the 'enable'-parameter
**********************************************************/
  int 
  avt1394_enable_hdr_mode(raw1394handle_t handle, nodeid_t node, dc1394bool_t enable);


/********************************************************
 avt1394_get_dsnu_blemish

 This routine retrieves dsnu or blemish control feature
 values
**********************************************************/
  int 
  avt1394_get_dsnu_blemish(raw1394handle_t handle, nodeid_t node,
			   avt_dsnu_blemish_t *pData, dc1394bool_t *on);

/********************************************************
 avt1394_set_dsnu_blemish

 This routine sets the DSNU- and Blemish-values respectively,
 without influencing the state of the feature
 (-> 'avt1394_enable_dsnu_blemish()').
 If the feature is busy it return DC1394_FAILURE.
**********************************************************/
  int 
  avt1394_set_dsnu_blemish(raw1394handle_t handle, nodeid_t node,
			   avt_dsnu_blemish_t oData);

/********************************************************
 avt1394_enable_dsnu_blemish

 This routine enables / disables dsnu or blemish control feature
**********************************************************/
  int 
  avt1394_enable_dsnu_blemish(raw1394handle_t handle, nodeid_t node,
			      dc1394bool_t type, dc1394bool_t enable);


/********************************************************
 avt1394_get_io_ctrl

 This routine retrieves input output control
**********************************************************/
  int 
  avt1394_get_io_ctrl(raw1394handle_t handle, nodeid_t node,
		      unsigned int nIoFeature, dc1394bool_t *pPolarity,
		      unsigned int *pMode,dc1394bool_t *pState);

/********************************************************
 avt1394_set_io_ctrl

 This routine sets input output control
**********************************************************/
  int 
  avt1394_set_io_ctrl(raw1394handle_t handle, nodeid_t node,
		      unsigned int nIoFeature, dc1394bool_t bPolarity,
		      unsigned int nMode,dc1394bool_t nState);

/********************************************************
 avt1394_get_int_delay

 This routine retrieves integration delay time in usec
**********************************************************/
  int 
  avt1394_get_int_delay(raw1394handle_t handle, nodeid_t node,
			unsigned int *pDelay, dc1394bool_t *on);

/********************************************************
 avt1394_set_int_delay

 This routine specifies the integration delay time in usec
**********************************************************/
  int 
  avt1394_set_int_delay(raw1394handle_t handle, nodeid_t node,
			   unsigned int nDelay);

/********************************************************
 avt1394_enable_int_delay

 This routine enables / disables integration delay
**********************************************************/
  int 
  avt1394_enable_int_delay(raw1394handle_t handle, nodeid_t node,
			   dc1394bool_t enable);

/********************************************************
 avt1394_get_auto_shutter_limits

 This routine retrieves  the auto shutter limits (i.e. both
 minimum and maximum value)
**********************************************************/
  int 
  avt1394_get_auto_shutter_limits(raw1394handle_t handle, nodeid_t node,
				  unsigned int *pMinVal,unsigned int *pMaxVal);

/********************************************************
 avt1394_set_auto_shutter_limits

 This routine specifies the auto shutter limits (i.e. both
 minimum and maximum value)
**********************************************************/
  int 
  avt1394_set_auto_shutter_limits(raw1394handle_t handle, nodeid_t node,
				  unsigned int nMinVal,unsigned int nMaxVal);

/********************************************************
 avt1394_get_auto_gain_limits

 This routine returns the auto gain limits (i.e. both
 minimum and maximum value)
**********************************************************/
  int 
  avt1394_get_auto_gain_limits(raw1394handle_t handle, nodeid_t node,
			       unsigned int *pMinVal,unsigned int *pMaxVal);

/********************************************************
 avt1394_set_auto_gain_limits

 This routine specifies the auto gain limits (i.e. both
 minimum and maximum value)
**********************************************************/
  int 
  avt1394_set_auto_gain_limits(raw1394handle_t handle, nodeid_t node,
			       unsigned int nMinVal,unsigned int nMaxVal);

/********************************************************
 avt1394_get_auto_aoi_dimensions

 This routine retrieves the dimensions of the work area
 (AOI) for some autofunctions
**********************************************************/
  int 
  avt1394_get_auto_aoi_dimensions(raw1394handle_t handle, nodeid_t node,
				  unsigned int *nLeft, unsigned int *nTop,
				  unsigned int *nWidth, unsigned int *nHeight,
				  dc1394bool_t *showWorkArea, dc1394bool_t *on);

/********************************************************
 avt1394_set_auto_aoi_dimensions

 This routine sets the dimensions of the work area
 (AOI) for some autofunctions
**********************************************************/
  int 
  avt1394_set_auto_aoi_dimensions(raw1394handle_t handle, nodeid_t node,
				  unsigned int nLeft, unsigned int nTop,
				  unsigned int nWidth, unsigned int nHeight);

/********************************************************
 avt1394_enable_auto_aoi

 This routine enables / disables the 'auto aoi' feature
**********************************************************/
  int
  avt1394_enable_auto_aoi(raw1394handle_t handle, nodeid_t node, dc1394bool_t enable);

/********************************************************
 avt1394_enable_auto_aoi_area

 This routine enables / disables the work area of the
 'auto aoi' feature
**********************************************************/
  int 
  avt1394_enable_auto_aoi_area(raw1394handle_t handle, nodeid_t node,
			       dc1394bool_t enable);

/********************************************************
 avt1394_get_color_corr_state

 This routine returns the current state of the
 color correction feature (ON / OFF) 
 Mainly applicable for Marlin C Cameras only
**********************************************************/
  int
  avt1394_get_color_corr_state(raw1394handle_t handle, nodeid_t node,
			       dc1394bool_t *on);

/********************************************************
 avt1394_enable_color_corr

 This routine enables or disables color correction feature. 
 Mainly applicable for Marlin C Cameras only
**********************************************************/
  int
  avt1394_enable_color_corr(raw1394handle_t handle, nodeid_t node,
			     dc1394bool_t bEnable);

/********************************************************
 avt1394_get_adv_trigger_delay

 This routine retrieves trigger delay time in usec
**********************************************************/
  int 
  avt1394_get_adv_trigger_delay(raw1394handle_t handle, nodeid_t node,
				unsigned int *pDelay, dc1394bool_t *on);

/********************************************************
 avt1394_set_adv_trigger_delay

 This routine sets trigger delay time specified in usec
**********************************************************/
  int 
  avt1394_set_adv_trigger_delay(raw1394handle_t handle, nodeid_t node,
				unsigned int pDelay);

/********************************************************
 avt1394_enable_sdv_trigger_delay

 This routine enables / disables trigger delay
**********************************************************/
  int 
  avt1394_enable_adv_trigger_delay(raw1394handle_t handle, nodeid_t node,
				   dc1394bool_t enable);


/********************************************************
 avt1394_get_mirror_image_state

 This routine returns the current state of the mirror image
 feature
**********************************************************/
  int 
  avt1394_get_mirror_image_state(raw1394handle_t handle, nodeid_t node,
				 dc1394bool_t *on);

/********************************************************
 avt1394_enable_mirror_image

 This routine enables or disables mirror image
**********************************************************/
  int 
  avt1394_enable_mirror_image(raw1394handle_t handle, nodeid_t node,
			      dc1394bool_t bEnable);

/********************************************************
 avt1394_delayed_soft_reset

 This routine resets the camera via the SOFT_RESET register,
 while using 'nDelay' to delay the reset in 10ms steps
 (see manual for more information)
**********************************************************/
  int 
  avt1394_delayed_soft_reset(raw1394handle_t handle, nodeid_t node,
		     unsigned int nDelay);

/********************************************************
 avt1394_soft_reset

 This routine resets the camera via the SOFT_RESET register
 (see manual for more information)
**********************************************************/
  int 
  avt1394_soft_reset(raw1394handle_t handle, nodeid_t node);

/********************************************************
 avt1394_get_soft_reset_delay

 This routine reports the current soft-reset-delay value
**********************************************************/
  int 
  avt1394_get_soft_reset_delay(raw1394handle_t handle, nodeid_t node,
			       unsigned int *pDelay);

/********************************************************
 avt1394_set_soft_reset_delay

 This routine changes the current soft-reset-delay value
**********************************************************/
  int
  avt1394_set_soft_reset_delay(raw1394handle_t handle, nodeid_t node,
			       unsigned int nDelay);

/********************************************************
 avt1394_high_snr_enable

 Enables / Disables the 'High SNR' mode
**********************************************************/
  int 
  avt1394_high_snr_enable(raw1394handle_t handle, nodeid_t node,
			 dc1394bool_t bEnable);

/********************************************************
 avt1394_get_high_snr_info

 This routine reports the 'GrabCount' value of the
 HIGH_SNR register, and the status of the 'ON_OFF' flag
**********************************************************/
  int 
  avt1394_get_high_snr_info(raw1394handle_t handle,
			    nodeid_t node,
			    unsigned int *pGrabCount,
			    dc1394bool_t *pEnabled);

/********************************************************
 avt1394_set_high_snr_grab_count

 This routine sets the 'GrabCount' value of the
 HIGH_SNR register
**********************************************************/
  int 
  avt1394_set_high_snr_grab_count(raw1394handle_t handle,
				  nodeid_t node,
				  unsigned int nGrabCount);

/********************************************************
 avt1394_enable_io_decoder

 This routine enables or disables  io decoder
**********************************************************/
  int 
  avt1394_enable_io_decoder(raw1394handle_t handle, nodeid_t node,
			    dc1394bool_t bEnable);

/********************************************************
 avt1394_set_io_decoder

 This routine sets io decoder value
**********************************************************/
  int 
  avt1394_set_io_decoder(raw1394handle_t handle, nodeid_t node,
			 unsigned int nCmp);

/********************************************************
 avt1394_get_io_decoder

 This routine retrieves the counter value of io decoder
**********************************************************/
  int 
  avt1394_get_io_decoder(raw1394handle_t handle, nodeid_t node,
			 unsigned int *pCounter, unsigned int *pCmp,
			 dc1394bool_t *on);

/********************************************************
 avt1394_reset_io_decoder_counter

 Clears the IO decoders position counter ('auto-reset')
**********************************************************/
  int 
  avt1394_reset_io_decoder_counter(raw1394handle_t handle, nodeid_t node);

/********************************************************
 avt1394_get_recv_serialbuffersize

 This routine retrieves the RBUF_ST and RBUF_CNT
**********************************************************/
  int 
  avt1394_get_recv_serialbuffersize(raw1394handle_t handle, nodeid_t node,
				    unsigned int *pRbufSt,
				    unsigned int *pRbufCnt);

/********************************************************
 avt1394_set_recv_serialbuffersize

 This routine sets serial buffer size
**********************************************************/
  int 
  avt1394_set_recv_serialbuffersize(raw1394handle_t handle, nodeid_t node,
				    unsigned int nSize);

/********************************************************
 avt1394_get_tx_serialbuffersize

 This routine retrieves the TBUF_ST and TBUF_CNT
**********************************************************/
  int 
  avt1394_get_tx_serialbuffersize(raw1394handle_t handle, nodeid_t node,
				  unsigned int *pTbufSt,unsigned int *pTbufCnt);

/********************************************************
 avt1394_set_tx_serialbuffersize

 This routine sets serial buffer size
**********************************************************/
  int 
  avt1394_set_tx_serialbuffersize(raw1394handle_t handle, nodeid_t node,
				  unsigned int nSize);

/********************************************************
 avt1394_get_serial_data

 This routine retrieves characters from SIO_DATA_REGISTER.
**********************************************************/
  int 
  avt1394_get_serial_data(raw1394handle_t handle, nodeid_t node,
			 avt_serial_data_t *pData);

/********************************************************
 avt1394_send_serial_data

 This routine sends serial data
**********************************************************/
  int 
  avt1394_send_serial_data(raw1394handle_t handle, nodeid_t node,
			   avt_serial_data_t oData);

/********************************************************
 avt1394_get_serial_mode

 This routine retrieves serial mode
**********************************************************/
  int 
  avt1394_get_serial_mode(raw1394handle_t handle, nodeid_t node,
			  avt_serial_mode_t *pData);

/********************************************************
 avt1394_set_serial_mode

 This routine sets serial mode information
**********************************************************/
  int 
  avt1394_set_serial_mode(raw1394handle_t handle, nodeid_t node,
			  avt_serial_mode_t oData);

/********************************************************
 avt1394_get_serial_control

 This routine retrieves serial control information
**********************************************************/
  int 
  avt1394_get_serial_control(raw1394handle_t handle, nodeid_t node,
			     avt_serial_control_t *pData);

/********************************************************
 avt1394_set_serial_control

 This routine sets serial control information
**********************************************************/
  int 
  avt1394_set_serial_control(raw1394handle_t handle, nodeid_t node,
			     avt_serial_control_t oData);

#ifdef __cplusplus
}
#endif

#endif /* _AVT1394_H */
