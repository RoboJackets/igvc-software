/*
 * IIDC1394-Based Digital Camera Control Library Extension for AVT Cameras
 *
 * Written by Menuka (and extended by Georg)
 */


#include "avt1394.h"
#include <math.h>
#include <unistd.h>
#include <netinet/in.h>


/**********************************/
/* Configuration Register Offsets */
/**********************************/

#define REG_CAMERA_VERSION_INFO               0x1000010U
#define REG_CAMERA_VERSION_INFO3              0x1000018U
#define REG_CAMERA_ADVANCED_INQ1              0x1000040U
#define REG_CAMERA_ADVANCED_INQ2              0x1000044U
#define REG_CAMERA_MAX_RESOLUTION             0x1000200U
#define REG_CAMERA_TIMEBASE                   0x1000208U
#define REG_CAMERA_EXTD_SHUTTER               0x100020CU
#define REG_CAMERA_TEST_IMAGE                 0x1000210U
#define REG_CAMERA_SEQUENCE_CTRL              0x1000220U
#define REG_CAMERA_SEQUENCE_PARAM             0x1000224U
#define REG_CAMERA_LUT_CTRL                   0x1000240U
#define REG_CAMERA_LUT_MEM_CTRL               0x1000244U
#define REG_CAMERA_LUT_INFO                   0x1000248U
#define REG_CAMERA_SHDG_CTRL                  0x1000250U
#define REG_CAMERA_SHDG_MEM_CTRL              0x1000254U
#define REG_CAMERA_SHDG_INFO                  0x1000258U
#define REG_CAMERA_DEFERRED_TRANS             0x1000260U
#define REG_CAMERA_FRAMEINFO                  0x1000270U
#define REG_CAMERA_FRAMECOUNTER               0x1000274U
#define REG_CAMERA_HDR_CONTROL                0x1000280U
#define REG_CAMERA_KNEEPOINT_1                0x1000284U
#define REG_CAMERA_KNEEPOINT_2                0x1000288U
#define REG_CAMERA_KNEEPOINT_3                0x100028CU
#define REG_CAMERA_DSNU_CONTROL               0x1000290U
#define REG_CAMERA_BLEMISH_CONTROL            0x1000294U
#define REG_CAMERA_IO_INP_CTRL1               0x1000300U
#define REG_CAMERA_IO_INP_CTRL2               0x1000304U
#define REG_CAMERA_IO_INP_CTRL3               0x1000308U
#define REG_CAMERA_IO_OUTP_CTRL1              0x1000320U
#define REG_CAMERA_IO_OUTP_CTRL2              0x1000324U
#define REG_CAMERA_IO_OUTP_CTRL3              0x1000328U
#define REG_CAMERA_INTENA_DELAY               0x1000340U
#define REG_CAMERA_IO_DECODER_CTRL            0x1000350U
#define REG_CAMERA_IO_DECODER_VAL             0x1000354U
#define REG_CAMERA_AUTOSHUTTER_CTRL           0x1000360U
#define REG_CAMERA_AUTOSHUTTER_LO             0x1000364U
#define REG_CAMERA_AUTOSHUTTER_HI             0x1000368U
#define REG_CAMERA_AUTOGAIN_CTRL              0x1000370U
#define REG_CAMERA_AUTOFNC_AOI                0x1000390U
#define REG_CAMERA_AF_AREA_POSITION           0x1000394U
#define REG_CAMERA_AF_AREA_SIZE               0x1000398U
#define REG_CAMERA_COLOR_CORR                 0x10003A0U
#define REG_CAMERA_TRIGGER_DELAY              0x1000400U
#define REG_CAMERA_MIRROR_IMAGE               0x1000410U
#define REG_CAMERA_SOFT_RESET                 0x1000510U
#define REG_CAMERA_HIGH_SNR                   0x1000520U
#define REG_CAMERA_GPDATA_INFO                0x1000FFCU
#define REG_CAMERA_GPDATA_BUFFER              0x1001000U
#define REG_OPT_FUNCTION_INQ                  0x40CU
#define REG_CAMERA_SIO_CTRL_INQ               0x488U
#define AVT_SERIAL_MODE                       0x000U
#define AVT_SERIAL_CTRL                       0x0004U
#define AVT_RCV_BUF_STATUS_CTRL               0x008U
#define AVT_TX_BUF_STATUS_CTRL                0x00CU
#define AVT_SERIAL_DATA                       0x100U

#define AVT_ON_OFF_MASK  0x02000000UL
#define AVT_SOFT_RESET_MASK 0x02000000UL



/* stuff taken from 'dc1394_internal.h' */


/* Definitions which application developers shouldn't care about */
#define CONFIG_ROM_BASE             0xFFFFF0000000ULL

/* Maximum number of write/read retries */
#define MAX_RETRIES                 20

/* A hard compiled factor that makes sure async read and writes don't happen
   too fast */
#define SLOW_DOWN                   20


typedef struct __dc1394_camerahandle {
  int       port;
  octlet_t  ccr_base;
  quadlet_t sw_version;
  octlet_t  format7_csr[NUM_MODE_FORMAT7];

} dc1394_camerahandle;



/*********************************************************************/
/* Internal functions ************************************************/
/*********************************************************************/

//static
int
AvtDCGetCameraControlRegister(raw1394handle_t handle, nodeid_t node,
			      octlet_t offset, quadlet_t *value) {
  int retval, retry= MAX_RETRIES;
  dc1394_camerahandle *camera = (dc1394_camerahandle*) raw1394_get_userdata(handle);


  /* get the ccr_base address if not yet retrieved */
  if (camera != NULL && camera->ccr_base == 0) {
    dc1394_camerainfo info;
    if (dc1394_get_camera_info(handle, node, &info) != DC1394_SUCCESS)
      return DC1394_FAILURE;
  }
  else {
    if (camera == NULL)
      return DC1394_FAILURE;
  }

  /* retry a few times if necessary (addition by PDJ) */
  while(retry--) { 
    /* try to read data from camera */
    if((retval = raw1394_read(handle, 0xffc0 | node,
			      camera->ccr_base + offset,
			      4, value)) != 0) {
      if (errno != EAGAIN)
	return DC1394_FAILURE;
    }
    else
      break;
    
    usleep(SLOW_DOWN);
  }
  
  /* if 'raw1394_read()' failed (even after all retries) - return 'failure' */
  if(retval != 0)
    return DC1394_FAILURE;

  /* conditionally byte swap the value (addition by PDJ) */
  *value= ntohl(*value);  

  return DC1394_SUCCESS;
}


//static
int
AvtGetCameraControlRegister(raw1394handle_t handle, nodeid_t node,
			    octlet_t offset, quadlet_t *value,
			    dc1394bool_t bRS232) {
  int retval, retry= MAX_RETRIES;
  dc1394_camerahandle *camera;
  quadlet_t advquadval;


  camera = (dc1394_camerahandle*) raw1394_get_userdata(handle);

  /* get the ccr_base address if not yet retrieved */
  if(camera != NULL && camera->ccr_base == 0) {
    dc1394_camerainfo info;
    if(dc1394_get_camera_info(handle, node, &info) == DC1394_FAILURE)
      return DC1394_FAILURE;
  }
  else {
    if(camera == NULL)
      return DC1394_FAILURE;
  }
    
  /* Query Advanced Feature offset */
  if(dc1394_query_advanced_feature_offset(handle, node, &advquadval) == DC1394_FAILURE) {
#if defined(AVT_DEBUG)
    printf("\nFailure in getting advanced feature offset");
#endif
    return DC1394_FAILURE;
  }

  /* retry a few times if necessary (addition by PDJ) */
  while(retry--) {
    if(bRS232 == DC1394_TRUE) {
      quadlet_t quadval;

      /* the SIO-register needs a special offset */
      if(AvtDCGetCameraControlRegister(handle, node,
				       REG_CAMERA_SIO_CTRL_INQ,
				       &quadval) != DC1394_SUCCESS)
	return DC1394_FAILURE;

      retval = raw1394_read(handle, 0xffc0 | node,
			    CONFIG_ROM_BASE + (quadval * 4) + offset,
			    4, value);
#if defined(AVT_DEBUG)
      printf("\nSIO-Base is %x",quadval);
      printf("\noffset is %x",(unsigned int)offset);
#endif
    }
    else {
      retval= raw1394_read(handle, 0xffc0 | node, 
			   CONFIG_ROM_BASE + offset,
			   4, value);
    }

    if(retval != 0) {
      if(errno != EAGAIN)
       return DC1394_FAILURE;
    }
    else
      break;

    usleep(SLOW_DOWN);
  }

  /* if 'raw1394_read()' failed (even after all retries) - return 'failure' */
  if(retval != 0)
    return DC1394_FAILURE;

  /* conditionally byte swap the value (addition by PDJ) */
  *value= ntohl(*value);

#if defined(AVT_DEBUG) 
  printf("\nSucess PDJ");
#endif

  return DC1394_SUCCESS;
}


int
AvtDCSetCameraControlRegister(raw1394handle_t handle, nodeid_t node,
			    octlet_t offset, quadlet_t value) {
  int retval, retry= MAX_RETRIES;
  dc1394_camerahandle *camera;
  camera = (dc1394_camerahandle*) raw1394_get_userdata( handle );

  /* get the ccr_base address if not yet retrieved */
  if (camera != NULL && camera->ccr_base == 0) {
    dc1394_camerainfo info;
    if (dc1394_get_camera_info(handle, node, &info) != DC1394_SUCCESS)
      return DC1394_FAILURE;
  }
  else {
    if (camera == NULL)
      return DC1394_FAILURE;
  }

  /* conditionally byte swap the value (addition by PDJ) */
  value= htonl(value);

  /* retry a few times if necessary */
  while(retry--) {
    retval = raw1394_write(handle, 0xffc0 | node, camera->ccr_base + offset,
			   4,&value);

    if(retval != 0) {
      if(errno != EAGAIN)
	return DC1394_FAILURE;
    }
    else
      break;

    usleep(SLOW_DOWN);
  }

  if(retval != 0)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


//static
int
AvtSetCameraControlRegister(raw1394handle_t handle, nodeid_t node,
			    octlet_t offset, quadlet_t value,
			    dc1394bool_t bRS232) {
  int retval, retry= MAX_RETRIES;
  dc1394_camerahandle *camera;
  camera = (dc1394_camerahandle*) raw1394_get_userdata( handle );

  /* get the ccr_base address if not yet retrieved */
  if (camera != NULL && camera->ccr_base == 0) {
    dc1394_camerainfo info;
    if (dc1394_get_camera_info(handle, node, &info) != DC1394_SUCCESS)
      return DC1394_FAILURE;
  }
  else {
    if (camera == NULL)
      return DC1394_FAILURE;
  }

  /* conditionally byte swap the value (addition by PDJ) */
  value= htonl(value);

  /* retry a few times if necessary */
  while(retry--) {
    if(bRS232 == DC1394_TRUE) {
      quadlet_t quadval;
      if(AvtDCGetCameraControlRegister(handle, node,
				       REG_CAMERA_SIO_CTRL_INQ,
				       &quadval) != DC1394_SUCCESS)
	return DC1394_FAILURE;

      retval = raw1394_write(handle, 0xffc0 | node,
			     CONFIG_ROM_BASE + (quadval* 4) + offset,
			     4,&value);
    }
    else {
      retval = raw1394_write(handle, 0xffc0 | node, CONFIG_ROM_BASE + offset,
			    4,&value);
    }

    if(retval != 0) {
      if(errno != EAGAIN)
	return DC1394_FAILURE;
    }
    else
      break;

    usleep(SLOW_DOWN);
  }

  if(retval != 0)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


static int
AvtGPSetCameraControlRegister(raw1394handle_t handle, nodeid_t node,
			      octlet_t offset, unsigned int nSize, quadlet_t *pValue) {
  int retval, retry = MAX_RETRIES, nCount;
  dc1394_camerahandle *camera = (dc1394_camerahandle*) raw1394_get_userdata(handle);


  /* get the ccr_base address if not yet retrieved */
  if((camera != NULL) && (camera->ccr_base == 0)) {
    dc1394_camerainfo info;
    if(dc1394_get_camera_info(handle, node, &info) != DC1394_SUCCESS)
      return DC1394_FAILURE;
  }
  else {
    if(camera == NULL)
      return DC1394_FAILURE;
  }


  /* conditionally byte swap the value (addition by PDJ) */
  for(nCount = 0; nCount < nSize; nCount++) {
    pValue[nCount]= htonl(pValue[nCount]);
  }



  /* retry a few times if necessary */
  while(retry--) {
    if((retval = raw1394_write(handle, 0xffc0 | node,
			       CONFIG_ROM_BASE + offset,
			       nSize * 4,
			       pValue)) != 0) {
      if(errno != EAGAIN)
	return DC1394_FAILURE;
    }
    else
      break;
    
    usleep(SLOW_DOWN);
  }

  if(retval != 0)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


static int
AvtGPGetCameraControlRegister(raw1394handle_t handle, nodeid_t node,
			      octlet_t offset, unsigned int nSize, quadlet_t *pValue) {
  int retval, retry = MAX_RETRIES, nCount;
  dc1394_camerahandle *camera = (dc1394_camerahandle*) raw1394_get_userdata( handle );


  /* get the ccr_base address if not yet retrieved */
  if((camera != NULL) && (camera->ccr_base == 0)) {
    dc1394_camerainfo info;
    if(dc1394_get_camera_info(handle, node, &info) == DC1394_FAILURE)
      return DC1394_FAILURE;
  }
  else {
    if(camera == NULL)
      return DC1394_FAILURE;
  }

  /* retry a few times if necessary */
  while(retry--) {
    if((retval = raw1394_read(handle, 0xffc0 | node,
			      CONFIG_ROM_BASE + offset,
			      nSize * 4,
			      pValue)) != 0) {
      if(errno != EAGAIN)
	return DC1394_FAILURE;
    }
    else
      break;
    
    usleep(SLOW_DOWN);
  }

  /* if 'raw1394_read()' failed (even after all retries) - return 'failure' */
  if(retval != 0)
    return DC1394_FAILURE;

  /* byte swap the values */
  for(nCount = 0; nCount < nSize; nCount++) {
    pValue[nCount]= ntohl(pValue[nCount]);
  }


  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_is_feature_present

 This routine reports whether a feature is present or not
*********************************************************/
int 
avt1394_is_feature_present(raw1394handle_t handle, nodeid_t node,
			   unsigned int feature, unsigned int enquiry_no,
			   dc1394bool_t *pPresent) {
 
  int retval = DC1394_FAILURE;
  quadlet_t quadval;


  if(enquiry_no == AVT_ADV_INQ_1) {
    retval = AvtGetCameraControlRegister(handle,node,
					 REG_CAMERA_ADVANCED_INQ1,
					 &quadval, DC1394_FALSE);
#if defined(AVT_DEBUG)
    printf("quadval %x", quadval);
#endif

  }
  else {
    retval = AvtGetCameraControlRegister(handle,node,
					 REG_CAMERA_ADVANCED_INQ2,
					 &quadval, DC1394_FALSE);
  }

  if(retval != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nSuccess in feature present");
  printf("\nFeature quadval %x", quadval);
#endif

  int tmp = pow(2, feature);
  if((tmp & quadval) > 0)
    *pPresent = DC1394_TRUE;
  else
    *pPresent = DC1394_FALSE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_get_camera_id

 This routine reports the camera id
*********************************************************/
int 
avt1394_get_camera_id(unsigned int id, unsigned int *pCameraId,
		      dc1394bool_t *pMarlin) {
  
  if((id < AVT_CAMERA_MIN) || (id > AVT_CAMERA_MAX)) {
    *pCameraId = id; /* unknown, but dont care - in the future!*/
    *pMarlin = DC1394_FALSE;
    return DC1394_SUCCESS;
  }

  *pCameraId = id;

  if((id >= AVT_CAMERA_MIN) && (id <= AVT_CAMERA_F201C1))
    *pMarlin = DC1394_FALSE;
  else
    *pMarlin = DC1394_TRUE;

  return DC1394_SUCCESS;
    
}


/********************************************************
 avt1394_get_timeval

 This routine reports the timebase in usec for id
*********************************************************/
int 
avt1394_get_timeval(unsigned int id, unsigned int *pValue) {
  switch(id) {
  case AVT_TIMEID0:
    *pValue = AVT_TIMEBASE_1;
    break;
  case AVT_TIMEID1:
    *pValue = AVT_TIMEBASE_2;
    break;
  case AVT_TIMEID2:
    *pValue = AVT_TIMEBASE_5;
    break;
  case AVT_TIMEID3:
    *pValue = AVT_TIMEBASE_10;
    break;
  case AVT_TIMEID4:
    *pValue = AVT_TIMEBASE_20;
    break;
  case AVT_TIMEID5:
    *pValue = AVT_TIMEBASE_50;
    break;
  case AVT_TIMEID6:
    *pValue = AVT_TIMEBASE_100;
    break;
  case AVT_TIMEID7:
    *pValue = AVT_TIMEBASE_200;
    break;
  case AVT_TIMEID8:
    *pValue = AVT_TIMEBASE_500;
    break;
  case AVT_TIMEID9:
    *pValue = AVT_TIMEBASE_1000;
    break;
  default:
    return DC1394_FAILURE;
  }
  return DC1394_SUCCESS;    
}


/********************************************************
 avt1394_get_timeid

 This routine reports the timeid for timevalue
*********************************************************/
int 
avt1394_get_timeid(unsigned int *pId, unsigned int pValue) {
  switch(pValue) {
  case AVT_TIMEBASE_1:
    *pId = AVT_TIMEID0;
    break;

  case AVT_TIMEBASE_2:
    *pId = AVT_TIMEID1;
    break;

  case AVT_TIMEBASE_5:
    *pId = AVT_TIMEID2;
    break;

  case AVT_TIMEBASE_10:
    *pId = AVT_TIMEID3;
    break;

  case AVT_TIMEBASE_20:
    *pId = AVT_TIMEID4;
    break;

  case AVT_TIMEBASE_50:
    *pId = AVT_TIMEID5;
    break;

  case AVT_TIMEBASE_100:
    *pId = AVT_TIMEID6;
    break;

  case AVT_TIMEBASE_200:
    *pId = AVT_TIMEID7;
    break;

  case AVT_TIMEBASE_500:
    *pId = AVT_TIMEID8;
    break;

  case AVT_TIMEBASE_1000:
    *pId = AVT_TIMEID9;
    break;

  default:
    return DC1394_FAILURE;
    break;
  }

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_get_io_register

 This routine gets the register for IO control
**********************************************************/
int
avt1394_get_io_register(unsigned int nIoFeature,
			quadlet_t *pRegister,
			dc1394bool_t *pOutput) {
  switch(nIoFeature) {
  case AVT_INP1_FEATURE:
    *pRegister = REG_CAMERA_IO_INP_CTRL1;
    *pOutput = DC1394_FALSE;
    break;

  case AVT_INP2_FEATURE:
    *pRegister = REG_CAMERA_IO_INP_CTRL2;
    *pOutput = DC1394_FALSE;
    break;

  case AVT_INP3_FEATURE:
    *pRegister = REG_CAMERA_IO_INP_CTRL3;
    *pOutput = DC1394_FALSE;
    break;	    

  case AVT_OUTP1_FEATURE:
    *pRegister = REG_CAMERA_IO_OUTP_CTRL1;
    *pOutput = DC1394_TRUE;
    break;

  case AVT_OUTP2_FEATURE:
    *pRegister = REG_CAMERA_IO_OUTP_CTRL2;
    *pOutput = DC1394_TRUE;
    break;

  case AVT_OUTP3_FEATURE:
    *pRegister = REG_CAMERA_IO_OUTP_CTRL3;
    *pOutput = DC1394_TRUE;
    break;

  default:
    return DC1394_FAILURE;
    break;
  }

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_check_availability

 This routine checks on off bit
*********************************************************/
int 
avt1394_check_availability(quadlet_t quadval) {
    
  quadval = quadval & AVT_AVAILABLE; 
  if(!quadval )
    return DC1394_FAILURE;    
  return DC1394_SUCCESS;    
}



/********************************************************
 avt1394_get_baudrate

 This routine reports the baud rate
*********************************************************/
int 
avt1394_get_baudrate(unsigned int nBaud, unsigned int *pBaud) {
  switch(nBaud) {
  case 0:
    *pBaud = AVT_BAUD_300;
    break;

  case 1:
    *pBaud = AVT_BAUD_600;
    break;

  case 2:
    *pBaud = AVT_BAUD_1200;
    break;

  case 3:
    *pBaud = AVT_BAUD_2400;
    break;

  case 4:
    *pBaud = AVT_BAUD_4800;
    break;

  case 5:
    *pBaud = AVT_BAUD_9600;
    break;

  case 6:
    *pBaud = AVT_BAUD_19200;
    break;

  case 7:
    *pBaud = AVT_BAUD_38400;
    break;

  case 8:
    *pBaud = AVT_BAUD_57600;
    break;

  case 9:
    *pBaud = AVT_BAUD_115200;
    break;

  case 10:
    *pBaud = AVT_BAUD_230400;
    break;

  default:
    return DC1394_FAILURE; 
    break;
  }

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_get_parity

 This routine reports the parity setting
*********************************************************/
int 
avt1394_get_parity(unsigned int nParity, unsigned int *pParity) {
  switch(nParity) {
  case 0:
    *pParity = AVT_NO_PARITY;
    break;

  case 1:
    *pParity = AVT_ODD_PARITY;
    break;

  case 2:
    *pParity = AVT_EVEN_PARITY;
    break;

  default:
    return DC1394_FAILURE; 
    break;
  }

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_get_stopbit

 This routine reports the stop bit
*********************************************************/
int 
avt1394_get_stopbit(unsigned int nStop, unsigned int *pStop) {
  switch(nStop) {
  case 0:
    *pStop = AVT_ONE_STOPBIT;
    break;

  case 1:
    *pStop = AVT_ONEHALF_STOPBIT;
    break;

  case 2:
    *pStop = AVT_TWO_STOPBIT;
    break;

  default:
    return DC1394_FAILURE; 
    break;
  }

  return DC1394_SUCCESS;

}


/********************************************************
 avt1394_get_charlen

 This routine reports the character length
*********************************************************/
int 
avt1394_get_charlen(unsigned int nCharLen, unsigned int *pLen) {
  switch(nCharLen) {
  case 7:
    *pLen = AVT_7_CHAR;
    break;

  case 8:
    *pLen = AVT_8_CHAR;
    break;

  default:
    return DC1394_FAILURE;
    break;
  }

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_set_baud_rate

 This routine set the baud rate in register
*********************************************************/
int 
avt1394_set_baud_rate(unsigned int nBaud, unsigned int *pBaud) {
  switch(nBaud) {
  case AVT_BAUD_300:
    *pBaud = AVT_HW_BAUD_300;
    break;

  case AVT_BAUD_600:
    *pBaud = AVT_HW_BAUD_600;
    break;

  case AVT_BAUD_1200:
    *pBaud = AVT_HW_BAUD_1200;
    break;

  case AVT_BAUD_2400:
    *pBaud = AVT_HW_BAUD_2400;
    break;

  case AVT_BAUD_4800:
    *pBaud = AVT_HW_BAUD_4800;
    break;

  case AVT_BAUD_9600:
    *pBaud = AVT_HW_BAUD_9600;
    break;

  case AVT_BAUD_19200:
    *pBaud = AVT_HW_BAUD_19200;
    break;

  case AVT_BAUD_38400:
    *pBaud = AVT_HW_BAUD_38400;
    break;

  case AVT_BAUD_57600:
    *pBaud = AVT_HW_BAUD_57600;
    break;

  case AVT_BAUD_115200:
    *pBaud = AVT_HW_BAUD_115200;
    break;

  case AVT_BAUD_230400:
    *pBaud = AVT_HW_BAUD_230400;
    break;

  default:
    return DC1394_FAILURE; 
    break;
  }

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_set_char_len

 This routine sets the character length
*********************************************************/
int 
avt1394_set_char_len(unsigned int nCharLen, unsigned int *pLen) {
  switch(nCharLen) {
  case  AVT_7_CHAR:
    *pLen = AVT_HW_7_CHAR;
    break;

  case AVT_8_CHAR:
    *pLen = AVT_HW_8_CHAR;
    break;

  default:
    return DC1394_FAILURE;
    break;
  }

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_set_parity

 This routine sets the parity setting
*********************************************************/
int 
avt1394_set_parity(unsigned int nParity, unsigned int *pParity) {
  switch(nParity) {
  case AVT_NO_PARITY:
    *pParity = AVT_HW_NO_PARITY;
    break;

  case AVT_ODD_PARITY:
    *pParity = AVT_HW_ODD_PARITY;
    break;

  case AVT_EVEN_PARITY:
    *pParity = AVT_HW_EVEN_PARITY;
    break;

  default:
    return DC1394_FAILURE;
    break;
  }

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_set_stopbit

 This routine sets the stop bit
*********************************************************/
int 
avt1394_set_stopbit(unsigned int nStop, unsigned int *pStop) {
  switch(nStop) {
  case AVT_ONE_STOPBIT:
    *pStop = AVT_HW_ONE_STOPBIT;
    break;

  case AVT_ONEHALF_STOPBIT:
    *pStop = AVT_HW_ONEHALF_STOPBIT;
    break;

  case AVT_TWO_STOPBIT:
    *pStop = AVT_HW_TWO_STOPBIT;
    break;

  default:
    return DC1394_FAILURE;
    break;
  }

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_write_gpdata

 This routine writes into GPData buffer
*********************************************************/
int 
avt1394_write_gpdata(raw1394handle_t handle, nodeid_t node,
		     unsigned char *pData, unsigned int nSize) {
  unsigned int nGPDataBufferSize, nGPDataBufferQuadSize;
  unsigned int nQuadWriteSize;
  unsigned int newBufferSize;
  unsigned int i;
  unsigned int index = 0;
  unsigned int nextIndex;
  quadlet_t *pQuadval=NULL;
  dc1394bool_t finish = DC1394_FALSE;


  /* determine 'nGPDataBufferSize' (as 'read-block-size') */
  if(avt1394_get_gpdata_size(handle, node, &nGPDataBufferSize) == DC1394_FAILURE)
    return DC1394_FAILURE;



  /* calculate the number of 'quadlets' fitting into GPDataBuffer */
  if((nGPDataBufferSize % 4) == 0)
    nGPDataBufferQuadSize = nGPDataBufferSize / 4;
  else
    nGPDataBufferQuadSize = (nGPDataBufferSize + (4 - (nGPDataBufferSize % 4))) / 4;


  /* allocate memory for the 'write-buffer' */
  if((pQuadval = (quadlet_t*)malloc(nGPDataBufferQuadSize * sizeof(quadlet_t))) == NULL)
    return DC1394_FAILURE;


  do {
    /* clear 'write-buffer' */
    for(i = 0; i < nGPDataBufferQuadSize; i++) {
      pQuadval[i] = 0;
    }


    /* calculate the index after writing the next block */
    nextIndex = index + (nGPDataBufferQuadSize * 4);
    /* if the next index lies behind the allocated memory -> align */
    if(nSize < nextIndex) {
      newBufferSize = (nGPDataBufferQuadSize * 4) - (nextIndex - nSize);
      /* if the reduced write-buffer size (write-buffer-size - 'overhang') is dividable by 4 */
      if((newBufferSize % 4) == 0)
	/* take it ... */
	nQuadWriteSize = newBufferSize / 4;
      else
	/* else -> 'add 1' */
	nQuadWriteSize = (newBufferSize + (4 - (newBufferSize % 4))) / 4;

      finish = DC1394_TRUE; /* ...because it's the last block */
    }
    else
      nQuadWriteSize = nGPDataBufferQuadSize;


    if(nextIndex == nSize) {
      finish = DC1394_TRUE;
    }


    /* copy block-contents to 'pData' */
    for(i = 0; i < nQuadWriteSize; i++) {
      pQuadval[i] = pData[index + (i*4)] |
	(pData[index + (i*4) + 1] << 8)|
	(pData[index + (i*4) + 2] << 16) |
	(pData[index + (i*4) + 3] << 24);
    }


    /* write block */
    if(AvtGPSetCameraControlRegister(handle, node,
				     REG_CAMERA_GPDATA_BUFFER,
				     nQuadWriteSize,
				     &(pQuadval[0])) != DC1394_SUCCESS) {
      if(pQuadval != NULL)
	free(pQuadval);
      return DC1394_FAILURE;
    }


    index += (nQuadWriteSize * 4);

    /* loop until all bytes are read */
  }while(!finish);


  return DC1394_SUCCESS;
} /* avt1394_write_gpdata() */


/********************************************************
 avt1394_read_gpdata

 This routine reads the GPData buffer
*********************************************************/
int 
avt1394_read_gpdata(raw1394handle_t handle, nodeid_t node,
		    unsigned char *pData, unsigned int nSize) {
  unsigned int nGPDataBufferSize, nGPDataBufferQuadSize;
  unsigned int nQuadReadSize;
  unsigned int newBufferSize;
  unsigned int i;
  unsigned int index = 0;
  unsigned int nextIndex;
  quadlet_t *pQuadval;
  dc1394bool_t finish = DC1394_FALSE;


  /* determine 'nGPDataBufferSize' (as 'read-block-size') */
  if(avt1394_get_gpdata_size(handle, node, &nGPDataBufferSize) == DC1394_FAILURE)
    return DC1394_FAILURE;


  /* calculate the number of 'quadlets' fitting into GPDataBuffer */
  if((nGPDataBufferSize % 4) == 0)
    nGPDataBufferQuadSize = nGPDataBufferSize / 4;
  else
    nGPDataBufferQuadSize = (nGPDataBufferSize + (4 - (nGPDataBufferSize % 4))) / 4;


  /* allocate memory for the 'read-buffer' */
  if((pQuadval = (quadlet_t*)malloc(nGPDataBufferQuadSize * sizeof(quadlet_t))) == NULL)
    return DC1394_FAILURE;


  do {
    /* clear 'read-buffer' */
    for(i = 0; i < nGPDataBufferQuadSize; i++) {
      pQuadval[i] = 0;
    }


    /* calculate the index after reading the next block */
    nextIndex = index + (nGPDataBufferQuadSize * 4);
    /* if the next index lies behind the allocated memory -> align */
    if(nSize < nextIndex) {
      newBufferSize = (nGPDataBufferQuadSize * 4) - (nextIndex - nSize);
      /* if the reduced read-buffer size (read-buffer-size - 'overhang') is dividable by 4 */
      if((newBufferSize % 4) == 0)
	/* take it ... */
	nQuadReadSize = newBufferSize / 4;
      else
	/* else -> 'add 1' */
	nQuadReadSize = (newBufferSize + (4 - (newBufferSize % 4))) / 4;

      finish = DC1394_TRUE; /* ...because it's the last block */
    }
    else
      nQuadReadSize = nGPDataBufferQuadSize;


    if(nextIndex == nSize) {
      finish = DC1394_TRUE;
    }


    /* read block */
    if(AvtGPGetCameraControlRegister(handle, node,
				     REG_CAMERA_GPDATA_BUFFER,
				     nQuadReadSize,
				     &(pQuadval[0])) != DC1394_SUCCESS) {
      if(pQuadval != NULL)
	free(pQuadval);
      return DC1394_FAILURE;
    }
    

    /* copy block-contents to 'pData' */
    for(i = 0; i < nQuadReadSize; i++) {
      pData[index + (i*4) + 3] = (unsigned char)((pQuadval[i] >> 24) & 0x000000FFUL);
      pData[index + (i*4) + 2] = (unsigned char)((pQuadval[i] >> 16) & 0x000000FFUL);
      pData[index + (i*4) + 1] = (unsigned char)((pQuadval[i] >> 8) & 0x000000FFUL);
      pData[index + (i*4)] = (unsigned char)(pQuadval[i] & 0x000000FFUL);
    }

    index += (nQuadReadSize * 4);
    
    /* loop until all bytes are read */
  }while(!finish);
  
  
  return DC1394_SUCCESS;
}



/*********************************************************************/
/* External functions ************************************************/
/*********************************************************************/



/********************************************************
 avt1394_get_uc_version

 This routine reports the version of microprocessor
*********************************************************/
int 
avt1394_get_uc_version(raw1394handle_t handle, nodeid_t node,
		       unsigned int *pMajor,unsigned int *pMinor) {
  int lastdigit;
  int thirddigit;
  int seconddigit;
  int firstdigit;
  quadlet_t quadval, temp;
  

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_VERSION_INFO ,
				 &quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nUC version is %x",quadval);
#endif

  temp = quadval  & 0x0000FFFFUL;
  lastdigit = temp & 0x0000000FUL;
  thirddigit= ((temp & 0x000000F0UL) >> 4) * 10;
  *pMinor = lastdigit + thirddigit;
  seconddigit = ((temp & 0x00000F00UL) >> 8) ;
  firstdigit = ((temp & 0x0000F000UL) >> 12) * 10;
  *pMajor = firstdigit + seconddigit;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_get_firmware_version

 This routine reports the firmware version of FPGA and camera
 type
*********************************************************/
int 
avt1394_get_firmware_version(raw1394handle_t handle, nodeid_t node,
			     avt_firmware_t *pData) {
  unsigned int lastdigit;
  unsigned int thirddigit;
  unsigned int seconddigit;
  unsigned int firstdigit;
  unsigned int tempval;
  quadlet_t quadval, temp;
 

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_VERSION_INFO3,
				 &quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\n Quadval = %x",quadval);
#endif

  temp = quadval & 0x0000FFFFUL;
  lastdigit = temp & 0x0000000FUL;
  thirddigit= ((temp & 0x000000F0UL) >> 4) * 10;
  pData->nMinor = lastdigit + thirddigit;
  seconddigit = ((temp & 0x00000F00UL) >> 8) ;
  firstdigit = ((temp & 0x0000F000UL) >> 12) * 10;
  pData->nMajor = firstdigit + seconddigit;
  tempval = (quadval & 0xFFFF0000UL) >> 16;
  
  if(avt1394_get_camera_id(tempval, &(pData->nCameraId),
			   &(pData->bMarlin)) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;

}


/********************************************************
 avt1394_is_adv_inq1_feature

 This routine reports whether advanced feature inquiry1
 is present
*********************************************************/
int 
avt1394_is_adv_inq1_feature(raw1394handle_t handle, nodeid_t node,
			    unsigned int nFeature,dc1394bool_t *pPresent) {

  return avt1394_is_feature_present(handle,node,nFeature,
				    AVT_ADV_INQ_1,pPresent);
}


/********************************************************
 avt1394_is_adv_inq2_feature

 This routine reports whether the advanced inqury2 
 feature is present  or not
*********************************************************/
int 
avt1394_is_adv_inq2_feature(raw1394handle_t handle, nodeid_t node,
			    unsigned int nFeature, dc1394bool_t *pPresent) {
    
  return avt1394_is_feature_present(handle,node,nFeature,
				    AVT_ADV_INQ_2,pPresent);
}


/********************************************************
 avt1394_get_max_resolution

 This routine retrieves the max resolution. Applicable
 only for format 7 mode 0
******************************************************/
int 
avt1394_get_max_resolution(raw1394handle_t handle, nodeid_t node,
			   unsigned int *pWidth, unsigned int * pHeight) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq1_feature(handle,node,AVT_MAXRES_FEATURE,
				       &present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;
	
  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_MAX_RESOLUTION,
				 &quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("Quadval %x",quadval);
#endif

  *pHeight = quadval & 0x0000FFFFUL;
  *pWidth = (quadval & 0xFFFF0000UL) >> 16;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_get_timebase

 This routine reports the current timebase which is in usec
*********************************************************/
int 
avt1394_get_timebase(raw1394handle_t handle, nodeid_t node,
		     unsigned int *pValue) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq1_feature(handle,node,
				       AVT_TIMEBASE_FEATURE,&present);   
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_TIMEBASE,&quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval = %x",quadval);
#endif

  /* check current availability of timebase*/
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  quadval &= 0x0000000FUL;

  /* convert Timebase-ID into usec-value */
  if(avt1394_get_timeval(quadval, pValue) == DC1394_FAILURE)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_set_timebase

 This routine sets the value of  timebase which is in usec
*********************************************************/
int 
avt1394_set_timebase(raw1394handle_t handle, nodeid_t node,
		     unsigned int nValue) {
  dc1394bool_t retval, present;
  quadlet_t quadval, qtimeid;
  unsigned int Timeid;


  retval = avt1394_is_adv_inq1_feature(handle,node,
				       AVT_TIMEBASE_FEATURE,&present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;

  /* convert usec-value into Timebase-ID */
  retval = avt1394_get_timeid(&Timeid, nValue);

  if(retval == DC1394_FAILURE)
    return DC1394_FAILURE;

  qtimeid = Timeid & 0x0000000FUL;

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_TIMEBASE,&quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  /* check current availability of timebase*/
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  quadval = quadval & 0xFFFFFFF0UL;

#if defined(AVT_DEBUG)
  printf("\ntimebase value is : %x",quadval);
  printf("\nTime id is %x",qtimeid);
#endif

  quadval = quadval | qtimeid;

#if defined(AVT_DEBUG)
  printf("\nValue before setting is %x",quadval);
#endif

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_TIMEBASE,quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


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
  'avt1394_get_shutter_offset()')
*********************************************************/
int 
avt1394_get_exposure_time(raw1394handle_t handle, nodeid_t node,
			  unsigned int *pValue, dc1394bool_t *pValid) {
  dc1394bool_t retval, present;
  quadlet_t quadval;
  unsigned int nTimeBase, nOffset, nStdShutter;

   
  retval = avt1394_is_adv_inq1_feature(handle, node,
				       AVT_EXTDSHUTTER_FEATURE, &present);
  if(retval == DC1394_FAILURE || present == DC1394_FALSE)
    return DC1394_FAILURE;

  if(!present) {
    *pValid = DC1394_FALSE;

    /* read the value of the standard-shutter-register */
    if(dc1394_get_shutter(handle, node, &nStdShutter) == DC1394_FAILURE)
      return DC1394_FAILURE;

    /* read the offset of the camera */
    avt1394_get_shutter_offset(handle, node, &nOffset);

    /* read the value of the timebase register */
    if(avt1394_get_timebase(handle, node, &nTimeBase) == DC1394_FAILURE)
      /* No TIMEBASE register available - only the content of the SHUTTER register will be used */
      *pValue = nStdShutter + nOffset;
    else
      *pValue = (nStdShutter * nTimeBase) + nOffset;

    /* If the exposure time is less then 10 usec then 
       return 'Failure' as min. exposure time is 10usec */
    if(*pValue < 10)
      return DC1394_FAILURE;
  }
  else {
    *pValid = DC1394_TRUE;

    if(AvtGetCameraControlRegister(handle,node,
				   REG_CAMERA_EXTD_SHUTTER,
				   &quadval,DC1394_FALSE) != DC1394_SUCCESS)
      return DC1394_FAILURE;

#if defined(AVT_DEBUG)
    printf("\nQuadval %x",quadval);
#endif

    /* check current availability of extended shutter */
    if(avt1394_check_availability(quadval) == DC1394_FAILURE)
      return DC1394_FAILURE;

    *pValue = quadval & 0x03FFFFFFUL;

    /* If the exposure time is less then 10 usec then 
       return failure as min exposure time is 10usec */
    if(*pValue < 10)
      return DC1394_FAILURE; 
  }

  return DC1394_SUCCESS;
}


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
			  unsigned int nValue) {
  dc1394bool_t retval, present;
  quadlet_t qexptime, qtimebase;
  unsigned int nOffset;
  unsigned int tb[10] = {1, 2, 5, 10, 20, 50, 100, 200, 500, 1000};
  unsigned int ntb = 10;
  unsigned int i, bestI;
  unsigned int currRest;


  retval = avt1394_is_adv_inq1_feature(handle,node,
				       AVT_EXTDSHUTTER_FEATURE,&present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;

  /* read the shutter-offset of the camera */
  avt1394_get_shutter_offset(handle, node, &nOffset);

  /* subtract the offset from the exposure-time value */
  nValue -= nOffset;

  /* If value is less than 10 usec then return failure */
  if(nValue < 10)
    return DC1394_FAILURE;

  /* 'mask out' bits 31 - 26 */
  qexptime = nValue & 0x03FFFFFFUL;

  /* if the exposure time is less than 4.096 ms */
  if(qexptime < 4096000) {
    /* if exposure time to set is greater than 0 */
    if(qexptime != 0) {
      /* calculate the best shutter / timebase pair */
      currRest = qexptime;
      for(i = 0; i < ntb; i++) {
	if(((qexptime / tb[i]) < 4096) &&
	   ((qexptime % tb[i]) < currRest)) {

	  bestI = i;
	  currRest = qexptime % tb[i];
	}
      }

      if(currRest == 0) {
	qtimebase = tb[bestI];
	qexptime /= tb[bestI];

	/* set the SHUTTER and TIMEBASE registers accordingly */
	if(dc1394_set_shutter(handle, node, qexptime) == DC1394_FAILURE)
	  return DC1394_FAILURE;

	if(avt1394_set_timebase(handle, node, qtimebase) == DC1394_FAILURE)
	  return DC1394_FAILURE;
      }
      else {
	/* exact shutter speed only possible with the extended shutter register! */
	if(avt1394_set_extended_shutter(handle, node, qexptime) == DC1394_FAILURE)
	  return DC1394_FAILURE;
      }
    }
    else {
      /* shutter speed == 0 => only set of SHUTTER is necessary */
      if(dc1394_set_shutter(handle, node, qexptime) == DC1394_FAILURE)
	return DC1394_FAILURE;
    }
  }
  else {
    /* value only possible with EXTD_SHUTTER register */
    retval = avt1394_set_extended_shutter(handle, node, qexptime);
    if(retval == DC1394_FAILURE)
      return DC1394_FAILURE;
  }

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_get_exposure_offset

 This routine reports the 'exposure offset' from the
 timebase-register. !No firmware-version-check is performed!
*********************************************************/
int 
avt1394_get_exposure_offset(raw1394handle_t handle, nodeid_t node,
			    unsigned int *pValue) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq1_feature(handle,node,
				       AVT_TIMEBASE_FEATURE,&present);   
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_TIMEBASE, &quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval = %x",quadval);
#endif

  /* check current availability of timebase*/
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  *pValue = (quadval >> 12) & 0x00000FFFUL;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_get_extended_shutter

 This routine reports the raw value from the IIDC extended
 shutter register (without the camera-specific offset)
*********************************************************/
int 
avt1394_get_extended_shutter(raw1394handle_t handle, nodeid_t node,
			     unsigned int *pValue) {
  dc1394bool_t retval, present;
  quadlet_t quadval;
   

  retval = avt1394_is_adv_inq1_feature(handle,node,
				       AVT_EXTDSHUTTER_FEATURE,&present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_EXTD_SHUTTER,
				 &quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval %x",quadval);
#endif

  /* check current availability of timebase*/
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;
	
  *pValue = quadval & 0x03FFFFFFUL;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_set_extended_shutter

 This routine sets the extended shutter register value
*********************************************************/
int 
avt1394_set_extended_shutter(raw1394handle_t handle, nodeid_t node,
			     unsigned int Value) {
  dc1394bool_t retval, present;
  quadlet_t quadval, qexptime;


  retval = avt1394_is_adv_inq1_feature(handle,node,
				       AVT_EXTDSHUTTER_FEATURE,&present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;

  /* If value is less than 10 usec then return failure*/
  if(Value < 10)
    return DC1394_FAILURE;

  qexptime = Value & 0x03FFFFFFUL;

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_EXTD_SHUTTER,
				 &quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nJust retrieved quadval :%x",quadval);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  quadval = quadval & 0xFC000000UL;
  quadval = qexptime | quadval;

#if defined(AVT_DEBUG)
  printf("\nNew Quadval is %x",quadval);
#endif

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_EXTD_SHUTTER,quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


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
			   unsigned int *pOffset) {
  avt_firmware_t oData;


  /* first try to get the offset from camera... */
  if(avt1394_get_exposure_offset(handle, node, pOffset))
    if(*pOffset != 0)
      return;

  /* use 'static'-version, if the offset cannot be read from camera */
  if(avt1394_get_firmware_version(handle,node,&oData) != DC1394_SUCCESS) {
    *pOffset = 24;
    return;
  }

  switch(oData.nCameraId) {
  case AVT_CAMERA_MF080B:
  case AVT_CAMERA_MF080C:
  case AVT_CAMERA_MF145B2:
  case AVT_CAMERA_MF145C2:
    *pOffset = 43;
    break;    

  case AVT_CAMERA_MF131B:
  case AVT_CAMERA_MF131C:
    *pOffset = 1;
    break;	

  case AVT_CAMERA_MF033B:
  case AVT_CAMERA_MF033C:
  case AVT_CAMERA_MF046B:
  case AVT_CAMERA_MF046C:
  default:
    *pOffset = 24;
    break;
  }	
}


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
		       unsigned int *pPresent, unsigned int *pActive) {
  dc1394bool_t retval, present;
  quadlet_t quadval, temp;
  unsigned int i;


  retval = avt1394_is_adv_inq1_feature(handle,node,
				       AVT_TESTIMAGE_FEATURE,
				       &present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;
   
  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_TEST_IMAGE,&quadval,
				 DC1394_FAILURE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nJust retrieved quadval :%x",quadval);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  temp = (quadval >> 17) & 0x0000007FUL;
  *pPresent = 0;
  for(i = 0; i < 7; i++) {
    if(((temp << i) & 0x40) != 0) {
      *pPresent |= (1 << i);
    }
  }

  *pActive = quadval & 0x0000000FUL;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_set_test_image

 This routine activates test image if present
*********************************************************/
int 
avt1394_set_test_image(raw1394handle_t handle, nodeid_t node,
		       unsigned int nActive) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq1_feature(handle,node,
				       AVT_TESTIMAGE_FEATURE,
				       &present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;
   
  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_TEST_IMAGE,&quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nJust retrieved quadval :%x",quadval);
#endif

  /* check current availability of test image*/
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  if(nActive > 7)
    return DC1394_FAILURE;

  if(nActive > 0) {
    /* check availability of the test image */
    if(((quadval >> (24 - nActive)) & 0x00000001UL) == 0)
      return DC1394_FAILURE;
  }
  else
    /* 'nActive' < 0 is not allowed !*/
    if(nActive < 0)
      return DC1394_FAILURE;
    
  quadval &= 0xFFFFFFF0UL;
  quadval |= nActive & 0x0000000FUL;
    
#if defined(AVT_DEBUG)
  printf("\nValue setting is %x",quadval);
#endif
    
  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_TEST_IMAGE,
				 quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_get_seq_info

 This routine retrieves the sequence information ie 
 Maximum Length
**********************************************************/
int 
avt1394_get_seq_info(raw1394handle_t handle, nodeid_t node,
		     unsigned int *pMaxLength,
		     dc1394bool_t *pApply, dc1394bool_t *on) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq1_feature(handle,node,AVT_SEQUENCES_FEATURE,
				       &present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;
   
  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_SEQUENCE_CTRL,
				 &quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)	
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability*/
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  *on = ((quadval & AVT_ON_OFF_MASK) != 0) ? DC1394_TRUE : DC1394_FALSE;

  *pMaxLength = (quadval >> 8) & 0x000000FFUL;
    
  if(AvtGetCameraControlRegister(handle, node,
				 REG_CAMERA_SEQUENCE_PARAM,
				 &quadval, DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  *pApply = (quadval & 0x04000000UL ? DC1394_TRUE : DC1394_FALSE);

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_enable_sequence

 This routine enables or disables  sequence
**********************************************************/
int 
avt1394_enable_sequence(raw1394handle_t handle, nodeid_t node,
			dc1394bool_t bEnable) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq1_feature(handle,node,AVT_SEQUENCES_FEATURE,
				       &present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;
   
  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_SEQUENCE_CTRL,
				 &quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  quadval = bEnable ? (quadval | AVT_ON_OFF_MASK) : (quadval & ~AVT_ON_OFF_MASK);

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_SEQUENCE_CTRL,quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_set_seq_param

 This routine sets the sequence parameters
**********************************************************/
int 
avt1394_set_seq_param(raw1394handle_t handle, nodeid_t node,
		      dc1394bool_t bAutoRewind,
		      unsigned int nSeqLength,
		      dc1394bool_t bIncImageNo,
		      unsigned int nImageNo) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq1_feature(handle,node,AVT_SEQUENCES_FEATURE,
				       &present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;
   
  /* 1st apply changes to the SEQUENCE_PARAM register */
  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_SEQUENCE_PARAM,
				 &quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  quadval &= 0xFFFFFF00UL;
  quadval |= (nImageNo & 0x000000FFUL);

  quadval &= 0xFDFFFFFFUL;
  quadval |= ((bIncImageNo << 25) & 0x02000000UL);

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  if(AvtSetCameraControlRegister(handle, node,
				 REG_CAMERA_SEQUENCE_PARAM, quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  /* then apply changes to the SEQUENCE_CTRL register */
  if(AvtGetCameraControlRegister(handle, node,
				 REG_CAMERA_SEQUENCE_CTRL,
				 &quadval, DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  quadval &= 0xFBFFFFFFUL;
  quadval |= ((bAutoRewind << 26) & 0x04000000UL);

  quadval &= 0xFFFFFF00UL;
  quadval |= (nSeqLength & 0x000000FFUL);

  if(AvtSetCameraControlRegister(handle, node,
				 REG_CAMERA_SEQUENCE_CTRL, quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_get_seq_param

 This routine returns the current sequence parameters
**********************************************************/
int 
avt1394_get_seq_param(raw1394handle_t handle, nodeid_t node,
		      dc1394bool_t *pAutoRewind,
		      unsigned int *pSeqLength,
		      dc1394bool_t *pIncImageNo,
		      unsigned int *pImageNo) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq1_feature(handle,node,AVT_SEQUENCES_FEATURE,
				       &present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;
   
  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_SEQUENCE_PARAM,
				 &quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif
  
  *pIncImageNo = (quadval >> 25) & 0x00000001UL;
  *pImageNo = quadval & 0x000000FFUL;

  if(AvtGetCameraControlRegister(handle, node,
				 REG_CAMERA_SEQUENCE_CTRL,
				 &quadval, DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  *pAutoRewind = (quadval >> 26) & 0x00000001UL;
  *pSeqLength = quadval & 0x000000FFUL;
  
  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_apply_seq_param

 This routine applies the current sequence parameters
**********************************************************/
int 
avt1394_apply_seq_param(raw1394handle_t handle, nodeid_t node, dc1394bool_t autoInc) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq1_feature(handle,node,AVT_SEQUENCES_FEATURE,
				       &present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;
   
  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_SEQUENCE_PARAM,
				 &quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  quadval |= 0x04000000UL; /* set 'ApplyParameters' */

  /* if 'autoInc' is set */
  if(autoInc == DC1394_TRUE) {
    /* set 'IncImageNo' */
    quadval |= 0x02000000UL;
  }

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_SEQUENCE_PARAM,quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_get_gpdata_size

 This routine retrieves data buffer size
**********************************************************/
int 
avt1394_get_gpdata_size(raw1394handle_t handle, nodeid_t node,
			unsigned int *pSize) {
  dc1394bool_t retval, present;
  quadlet_t quadval;
    

  retval = avt1394_is_adv_inq1_feature(handle,node,
				       AVT_GPBUFFER_FEATURE,&present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_GPDATA_INFO,&quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  *pSize = quadval & 0x0000FFFFUL;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_get_lut_info

 This routine retrieves lookup table information
**********************************************************/
int 
avt1394_get_lut_info(raw1394handle_t handle, nodeid_t node,
		     unsigned int *pMaxLutNo, unsigned int *pMaxLutSize,
		     dc1394bool_t *on) {
  dc1394bool_t retval, present;
  quadlet_t quadval1, quadval2;


  retval = avt1394_is_adv_inq1_feature(handle,node,AVT_LUT_FEATURE,&present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle, node,
				 REG_CAMERA_LUT_INFO, &quadval1,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval1);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval1) == DC1394_FAILURE)
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle, node,
				 REG_CAMERA_LUT_CTRL, &quadval2,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval1);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval2) == DC1394_FAILURE)
    return DC1394_FAILURE;

  *on = ((quadval2 & AVT_ON_OFF_MASK) != 0) ? DC1394_TRUE : DC1394_FALSE;

  *pMaxLutSize = quadval1 & 0x0000FFFFUL;
  *pMaxLutNo = (quadval1 >> 16) & 0x000000FFUL;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_select_lut

 This routine selects the lookup table by setting lookup
 table number (after testing that the number is 'in-range')
**********************************************************/
int
avt1394_select_lut(raw1394handle_t handle, nodeid_t node,
		   unsigned int nLutno) {
  unsigned int numLUT, nLutSize;
  quadlet_t quadval;
  dc1394bool_t on;
  

  /* the inquiry of the presence of the LUT support of the camera
     is done within 'avt1394_get_lut_info()' */
  if(avt1394_get_lut_info(handle, node, &numLUT, &nLutSize, &on) == DC1394_FAILURE)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\n Number of LUT %d",numLUT);
#endif

  /* only 'numLUT' LUTs are available */
  if(nLutno > numLUT)
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_LUT_CTRL, &quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

    /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  quadval = quadval & 0xFFFFFFC0UL;
  quadval = quadval | (nLutno & 0x0000003FUL);

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_LUT_CTRL,quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_get_lut_selection

 This routine returns the number of the selected LUT
**********************************************************/
int 
avt1394_get_lut_selection(raw1394handle_t handle, nodeid_t node,
			  unsigned int *nLutNo) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq1_feature(handle,node,AVT_LUT_FEATURE,&present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle, node,
				 REG_CAMERA_LUT_CTRL, &quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;
    
  *nLutNo = quadval & 0x0000003FUL;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_enable_lut

 This routine enables / disables the selected lookup table
**********************************************************/
int 
avt1394_enable_lut(raw1394handle_t handle, nodeid_t node,
		   dc1394bool_t enable) {
  dc1394bool_t retval, present;
  quadlet_t quadval;
    

  retval = avt1394_is_adv_inq1_feature(handle,node,AVT_LUT_FEATURE,&present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;


  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_LUT_CTRL,&quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  if(enable)
    quadval |= AVT_ON_OFF_MASK;
  else
    quadval &= ~AVT_ON_OFF_MASK;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_LUT_CTRL,quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_load_lut

 This routine  loads lookup table into the camera
**********************************************************/
int 
avt1394_load_lut(raw1394handle_t handle, nodeid_t node,
                 unsigned int nLutno,unsigned char *pData,unsigned int nSize) {
  unsigned int numLUT, nMaxSize;
  quadlet_t quadval;
  dc1394bool_t on;
  
  
  /* the inquiry of the presence of the LUT support of the camera
     is done within 'avt1394_get_lut_info()' */
  if(avt1394_get_lut_info(handle, node, &numLUT, &nMaxSize, &on) == DC1394_FAILURE)
    return DC1394_FAILURE;
   
  if(nLutno > (numLUT-1))
    return DC1394_FAILURE;       
    
    
  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_LUT_MEM_CTRL,
				 &quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  /* set 'AccessLutNo' and 'AddrOffset' = 0 */
  quadval = (quadval & 0xFF000000UL) | ((nLutno << 16) & 0x00FF0000UL);
  /* set 'EnableMemWR' */
  quadval = quadval | 0x04000000;

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_LUT_MEM_CTRL,quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  /* copy data via 'GPDataBuffer' */
  if(avt1394_write_gpdata(handle,node,pData,nSize) == DC1394_FAILURE)
    return DC1394_FAILURE;
 	    
  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_LUT_MEM_CTRL,
				 &quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  /* reset 'EnableMemWR' */
  quadval = quadval & 0xFBFFFFFFUL;

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_LUT_MEM_CTRL,quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;
  
  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_get_shading_info

 This routine retrieves shading information
**********************************************************/
int 
avt1394_get_shading_info(raw1394handle_t handle, nodeid_t node,
			 dc1394bool_t *pShowImg, dc1394bool_t *pBuildImg,
			 dc1394bool_t *pBusy, unsigned int *pNumImg,
			 unsigned int *pMaxImgSize, dc1394bool_t *on) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq1_feature(handle,node,AVT_SHADING_FEATURE,
				       &present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;

  retval = avt1394_get_max_shading_img_size(handle, node, pMaxImgSize);
  if(retval == DC1394_FAILURE)
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_SHDG_CTRL,&quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  *on = ((quadval & AVT_ON_OFF_MASK) != 0) ? DC1394_TRUE : DC1394_FALSE;

  /* show image flag */
  *pShowImg  = (quadval & 0x08000000) >> 27;
  /* build image flag (auto-reset!) */
  *pBuildImg  = (quadval & 0x04000000) >> 26;
  /* busy-flag - indicates that a new image calculation is still in progress */
  *pBusy     = (quadval & 0x01000000) >> 24;
  /* number of images to be grabbed for automatic generation of correction data */
  *pNumImg = quadval & 0x000000FFUL;             
  
  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_get_max_shading_img_size

 This routine retrieves the max. shading image size
**********************************************************/
int 
avt1394_get_max_shading_img_size(raw1394handle_t handle, nodeid_t node,
				 unsigned int *pMaxImgSize) {
  dc1394bool_t retval, present;
  quadlet_t quadval;
    

  retval = avt1394_is_adv_inq1_feature(handle,node,AVT_SHADING_FEATURE,
				       &present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle, node,
				 REG_CAMERA_SHDG_INFO, &quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  *pMaxImgSize = quadval & 0x00FFFFFF;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_show_shading_img

 This routine enables or disables  shading correction
**********************************************************/
int 
avt1394_show_shading_img(raw1394handle_t handle, nodeid_t node,
			 dc1394bool_t bShowImg) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq1_feature(handle,node,AVT_SHADING_FEATURE,
				       &present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;
   
  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_SHDG_CTRL,&quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  if(bShowImg)
    quadval = quadval | 0x08000000UL;
  else
    quadval = quadval & ~(0x08000000UL);

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_SHDG_CTRL,quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_load_shading

 This routine loads a shading image into the camera
**********************************************************/
int 
avt1394_load_shading(raw1394handle_t handle, nodeid_t node,
		     unsigned char *pData, unsigned int nSize) {
  dc1394bool_t retval, present;
  unsigned int nMaxImg;
  quadlet_t quadval;

    
  /* (1) query limits by reading SHDG_INFO and GPDATA_INFO */
  retval = avt1394_is_adv_inq1_feature(handle,node,
				       AVT_SHADING_FEATURE,
				       &present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;

  if(avt1394_get_max_shading_img_size(handle, node, &nMaxImg) == DC1394_FAILURE)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\n nMaxImage is %d",nMaxImg);
  printf("\n nSize is %d", nSize);
#endif
  
  if(nSize > nMaxImg)
    return DC1394_FAILURE;

  /* (2) set 'EnableMemRD'
     set 'AddrOffset' = 0 */
  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_SHDG_MEM_CTRL,
				 &quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x", quadval);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  /* set Address Offset to zero */
  quadval = quadval & 0xFF000000UL;
  /* EnableMemWR to True */
  quadval = quadval | 0x04000000UL;

#if defined(AVT_DEBUG)
  printf("\nOffset Quadval is %x",quadval);
#endif

  /* write values back to SHDG_MEM_CTRL  */
  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_SHDG_MEM_CTRL,
				 quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  /* (3) write n data bytes to GPDATA_BUFFER
     < (4) add the number of bytes written to 'AddrOffset' >
     make sure 'EnableMemWR' and 'AccessLutNo' aren't changed
     (5) repeat from step (3) until all data is read */
  if(avt1394_write_gpdata(handle, node, pData, nSize) == DC1394_FAILURE)
    return DC1394_FAILURE;
    
  /* (6) reset 'EnableMemRD' */
  if(AvtGetCameraControlRegister(handle, node,
				 REG_CAMERA_SHDG_MEM_CTRL,
				 &quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval = %x",quadval);
#endif

  /* reset 'EnableMemRD' */
  quadval = quadval & 0xFBFFFFFFUL;

#if defined(AVT_DEBUG)
  printf("\nQuadval = %x",quadval);
#endif

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_SHDG_MEM_CTRL,
				 quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_read_shading

 This routine reads a shading image from the camera
**********************************************************/
int 
avt1394_read_shading(raw1394handle_t handle, nodeid_t node,
		     unsigned char *pData,unsigned int nSize) {
  dc1394bool_t retval, present;
  unsigned int nMaxImgSize;
  quadlet_t quadval;


  /* (1) query limits by reading SHDG_INFO and GPDATA_INFO */
  retval = avt1394_is_adv_inq1_feature(handle,node,
				       AVT_SHADING_FEATURE,
				       &present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;

  if(avt1394_get_max_shading_img_size(handle, node, &nMaxImgSize) == DC1394_FAILURE)
    return DC1394_FAILURE;
  
#if defined(AVT_DEBUG)
  printf("\n nMaxImage is %d",nMaxImg);
  printf("\n nSize is %d", nSize);
#endif
  
  if(nSize > nMaxImgSize)
    return DC1394_FAILURE;

  /* (2) set 'EnableMemRD'
     set 'AddrOffset' = 0 */
  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_SHDG_MEM_CTRL,
				 &quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x", quadval);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  /* set 'AddrOffset' = 0 */
  quadval = quadval & 0xFF000000UL;
  /* set EnableMemRD */
  quadval = quadval | 0x02000000UL;

#if defined(AVT_DEBUG)
  printf("\nOffset Quadval is %x",quadval);
#endif

  /* write values back to SHDG_MEM_CTRL */
  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_SHDG_MEM_CTRL,
				 quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  /* (3) read n data bytes from GPDATA_BUFFER
     < (4) add the number of bytes read to 'AddrOffset' >
     make sure 'EnableMemRD' and 'AccessLutNo' aren't changed
     (5) repeat from step (3) until all data is read */
  if(avt1394_read_gpdata(handle, node, pData, nSize) == DC1394_FAILURE)
    return DC1394_FAILURE;
 	    
  /* (6) reset 'EnableMemRD' */
  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_SHDG_MEM_CTRL,
				 &quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval = %x",quadval);
#endif

  /* reset 'EnableMemRD' */
  quadval = quadval & 0xFDFFFFFFUL;

#if defined(AVT_DEBUG)
  printf("\nQuadval = %x",quadval);
#endif

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_SHDG_MEM_CTRL,
				 quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_enable_shading

 Enables / Disables the shading correction
**********************************************************/
int 
avt1394_enable_shading(raw1394handle_t handle, nodeid_t node,
		       dc1394bool_t bEnable) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq1_feature(handle,node,AVT_SHADING_FEATURE,
				       &present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;
   
  if(AvtGetCameraControlRegister(handle, node,
				 REG_CAMERA_SHDG_CTRL, &quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;
	
  quadval = bEnable ? (quadval | AVT_ON_OFF_MASK) : (quadval & ~AVT_ON_OFF_MASK);

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_SHDG_CTRL,quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_build_shading_image

 This routine starts the building of a new shading image
**********************************************************/
int 
avt1394_build_shading_image(raw1394handle_t handle, nodeid_t node,
			    unsigned int grabCount) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq1_feature(handle,node,AVT_SHADING_FEATURE,
				       &present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE)) {
    fprintf(stderr, "\nShadingCorrection is not present\n");
    return DC1394_FAILURE;
  }
   
  if(AvtGetCameraControlRegister(handle, node,
				 REG_CAMERA_SHDG_CTRL, &quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\n'REG_CAMERA_SHDG_CTRL'-Quadval is %x\n",quadval);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  /* set 'GrabCount' */
  quadval = (quadval & 0xFFFFFF00UL) | (grabCount & 0x000000FFUL);

  /* set 'BuildImage' flag */
  quadval = quadval | 0x04000000UL;

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_SHDG_CTRL,quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_set_shading_grab_count

 This routine sets the GrabCount parameter of the
 Shading Correction feature
**********************************************************/
int 
avt1394_set_shading_grab_count(raw1394handle_t handle, nodeid_t node,
			       unsigned int grabCount) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq1_feature(handle,node,AVT_SHADING_FEATURE,
				       &present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE)) {
    fprintf(stderr, "\nShadingCorrection is not present\n");
    return DC1394_FAILURE;
  }
   
  if(AvtGetCameraControlRegister(handle, node,
				 REG_CAMERA_SHDG_CTRL, &quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\n'REG_CAMERA_SHDG_CTRL'-Quadval is %x\n",quadval);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  /* set 'GrabCount' */
  quadval = (quadval & 0xFFFFFF00UL) | (grabCount & 0x000000FFUL);

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_SHDG_CTRL,quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_get_deferr_trans

 This routine retrieves deferred transport information
**********************************************************/
int 
avt1394_get_deferr_trans(raw1394handle_t handle, nodeid_t node,
			 dc1394bool_t *pSendImg, dc1394bool_t *pHoldImg,
			 dc1394bool_t *pFastCapture, unsigned int *pFifoSize,
			 unsigned int *pNumImg) {
  dc1394bool_t retval, present;
  quadlet_t quadval;
   

  retval = avt1394_is_adv_inq1_feature(handle,node, 
				       AVT_DEFERREDTRANS_FEATURE,&present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;
    
  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_DEFERRED_TRANS,
				 &quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  *pSendImg = (quadval >> 26) & 0x00000001UL;
  *pHoldImg = (quadval >> 25) & 0x00000001UL;
  *pFastCapture = (quadval >> 24) & 0x00000001UL;
  *pFifoSize = (quadval >> 8) & 0x000000FFUL;
  *pNumImg = quadval & 0x000000FFUL;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_set_deferr_trans

 This routine sets deferred transport ie either enabling 
 or disabling it,enabling or disabling HoldImg mode,
 enabling or disabling Fastcapture mode
**********************************************************/
int 
avt1394_set_deferr_trans(raw1394handle_t handle, nodeid_t node,
			 dc1394bool_t bSendImg, dc1394bool_t bHoldImg,
			 dc1394bool_t bFastCapture, unsigned int nNumImg) {
  dc1394bool_t retval, present;
  quadlet_t quadval;
  

  retval = avt1394_is_adv_inq1_feature(handle,node,
				       AVT_DEFERREDTRANS_FEATURE, &present);

  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_DEFERRED_TRANS,
				 &quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

    /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  quadval = (quadval & 0xF8FFFF00UL) |
    (((bSendImg << 26) | (bHoldImg << 25) | (bFastCapture << 24) | nNumImg) &
     0x070000FFUL);

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_DEFERRED_TRANS,quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_get_frame_info

 This routine retrieves frame counter  ie number of captured
 frames since last reset
**********************************************************/
int 
avt1394_get_frame_info(raw1394handle_t handle, nodeid_t node,
		       unsigned int *pFrameCtr) {
  dc1394bool_t retval, present;
  quadlet_t quadval;
    

  retval = avt1394_is_adv_inq1_feature(handle,node,
				       AVT_FRAMEINFO_FEATURE,&present);

  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_FRAMEINFO,&quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_FRAMECOUNTER,&quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  *pFrameCtr = quadval;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_reset_frame_counter

 This routine resets the frame counter
**********************************************************/
int 
avt1394_reset_frame_counter(raw1394handle_t handle, nodeid_t node) {
  dc1394bool_t retval, present;
  quadlet_t quadval;
    
  retval = avt1394_is_adv_inq1_feature(handle,node,
				       AVT_FRAMEINFO_FEATURE,&present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;
  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_FRAMEINFO,&quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

    /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;
	 
  quadval = quadval | 0x40000000UL;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_FRAMEINFO,quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


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
		     dc1394bool_t *on) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq1_feature(handle,node,
				       AVT_HDR_FEATURE,&present);

  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_HDR_CONTROL,&quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nHDR_CONTROL Quadval is %x",quadval);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  *pMaxKnee = (quadval >> 8) & 0x0000000FUL;
  if(pMaxKnee == 0)
    return DC1394_FAILURE;

  *on = ((quadval & AVT_ON_OFF_MASK) != 0) ? DC1394_TRUE : DC1394_FALSE;

  /* initialize all knee-values with '0' */
  *pKnee1 = *pKnee2 = *pKnee3 = 0;

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_KNEEPOINT_1,&quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nKNEEPOINT1 Quadval: %x",quadval);
#endif

  *pKnee1 = quadval  & 0x0000FFFFUL;

  /* read knee2 only if available */
  if(*pMaxKnee > 1) {
    if(AvtGetCameraControlRegister(handle,node,
				   REG_CAMERA_KNEEPOINT_2,&quadval,
				   DC1394_FALSE) != DC1394_SUCCESS)
      return DC1394_FAILURE;

#if defined(AVT_DEBUG)
    printf("\nKNEEPOINT2 Quadval: %x",quadval);
#endif

    *pKnee2 = quadval  & 0x0000FFFFUL;
  }

  /* read knee3 only if available */
  if(*pMaxKnee > 2) {
    if(AvtGetCameraControlRegister(handle,node,
				   REG_CAMERA_KNEEPOINT_3,&quadval,
				   DC1394_FALSE) != DC1394_SUCCESS)
      return DC1394_FAILURE;

#if defined(AVT_DEBUG)
    printf("\nKNEEPOINT3 Quadval: %x",quadval);
#endif

    *pKnee3 = quadval  & 0x0000FFFFUL;
  }
   
  return DC1394_SUCCESS;
}


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
			 unsigned int *pActiveKnees) {
  dc1394bool_t retval, present;
  quadlet_t quadval;
   

  retval = avt1394_is_adv_inq1_feature(handle,node,
				       AVT_HDR_FEATURE,&present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_HDR_CONTROL,&quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nHDR_CONTROL Quadval is %x",quadval);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  *pActiveKnees = (quadval) & 0x0000000FUL;
   
  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_set_knee_values

 This routine sets the number of active knee points
 and their values without influencing the state of the
 HDR-mode (this has to be done with 'avt1394_enable_hdr_mode()')
**********************************************************/
int 
avt1394_set_knee_values(raw1394handle_t handle, nodeid_t node,
			unsigned int nActiveKnee, unsigned int dKnee1,
			unsigned int  dKnee2, unsigned int dKnee3) {
  dc1394bool_t retval, present;
  quadlet_t quadval;
  unsigned int numKnees;


  retval = avt1394_is_adv_inq1_feature(handle,node,AVT_HDR_FEATURE,&present);

  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE)) {

#if defined(AVT_DEBUG)
    printf("\nfeature not available\n");
#endif

    return DC1394_FAILURE;
  }

/*
  if((nActiveKnee > 3) || (dKnee1 < dKnee2) || (dKnee2 < dKnee3)) {
#if defined(AVT_DEBUG)
    printf("\nwrong parameters (knee1 >= knee2 >= knee3!)\n");
#endif
    return DC1394_FAILURE;
  }
*/

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_HDR_CONTROL,
				 &quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  numKnees = (quadval & 0x00000F00UL) >> 8;

  if(nActiveKnee > numKnees) {

#if defined(AVT_DEBUG)
      printf("\nnumber of requested kneepoints > number of available kneepoints\n");
#endif

      return DC1394_FAILURE;
  }

  quadval = (quadval & 0xFFFFFFF0UL) | (nActiveKnee & 0x0000000FUL);

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_HDR_CONTROL,quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  quadval = dKnee1 & 0x0000FFFFUL;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_KNEEPOINT_1,quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  quadval = dKnee2 & 0x0000FFFFUL;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_KNEEPOINT_2,quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  quadval = dKnee3 & 0x0000FFFFUL;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_KNEEPOINT_3,quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_set_knee_values_percent

 This routine sets the number of active knee points
 and their values relative to the current shutter (%)
 without influencing the state of the HDR-mode (this
  has to be done with 'avt1394_enable_hdr_mode()')
**********************************************************/
int 
avt1394_set_knee_values_percent(raw1394handle_t handle, nodeid_t node,
				unsigned int nActiveKnee, unsigned int dPerKnee1,
				unsigned int  dPerKnee2, unsigned int dPerKnee3) {
  dc1394bool_t retval, present;
  quadlet_t quadval;
  unsigned int nKneePoint1, nKneePoint2, nKneePoint3, nShutter;
  unsigned int numKnees;
  dc1394bool_t pValid;


  retval = avt1394_is_adv_inq1_feature(handle,node,AVT_HDR_FEATURE,&present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;

  if((nActiveKnee > 3) || (dPerKnee1 < dPerKnee2) || (dPerKnee2 < dPerKnee3))
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_HDR_CONTROL,
				 &quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;	 

  numKnees = (quadval & 0x00000F00UL) >> 8;

  if(nActiveKnee > numKnees)
    return DC1394_FAILURE;

  quadval |= nActiveKnee & 0x0000000FUL;

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_HDR_CONTROL,quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  if(avt1394_get_exposure_time(handle, node, &nShutter, &pValid) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  nKneePoint1 = (dPerKnee1 * nShutter)/ 100;
  nKneePoint2 = (dPerKnee2  * nShutter) /100;
  nKneePoint3 = (dPerKnee3  * nShutter) /100;

  quadval = nKneePoint1 & 0x0000FFFF;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
  printf("\n nKneePoint3 is %d",nKneePoint3);
#endif

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_KNEEPOINT_1,quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  quadval = nKneePoint2;
  quadval = quadval & 0x0000FFFF;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_KNEEPOINT_2,quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  quadval = nKneePoint3;
  quadval = quadval & 0x0000FFFF;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_KNEEPOINT_3,quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_enable_hdr_mode

 This routine enables / disables the hdr mode
 depending on the 'enable'-parameter
**********************************************************/
int
avt1394_enable_hdr_mode(raw1394handle_t handle, nodeid_t node, dc1394bool_t bEnable) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


#if defined(AVT_DEBUG)
  printf("\n avt1394_enable_hdr_mode() \n");
#endif

  retval = avt1394_is_adv_inq1_feature(handle,node,AVT_HDR_FEATURE,&present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE)) {
#if defined(AVT_DEBUG)
    printf("\n 1st availability-inquiry failed!\n");
#endif
    return DC1394_FAILURE;
  }
  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_HDR_CONTROL,&quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval (before) is %x\n",quadval);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  /* set ON_OFF bit accordingly */
  quadval = bEnable ? (quadval | AVT_ON_OFF_MASK) : (quadval & ~AVT_ON_OFF_MASK);

#if defined(AVT_DEBUG)
  printf("\nQuadval (after) is %x",quadval);
#endif

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_HDR_CONTROL,quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_HDR_CONTROL,&quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  printf("\nQuadval (set) is %x\n", quadval);
#endif

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_get_dsnu_blemish

 This routine retrieves dsnu or blemish control feature
 values
**********************************************************/
int 
avt1394_get_dsnu_blemish(raw1394handle_t handle, nodeid_t node,
			 avt_dsnu_blemish_t *pData, dc1394bool_t *on) {
  dc1394bool_t retval, present;
  quadlet_t quadval,qOffset;
   

  /* Only 'DSNU'-availability can be tested with the 'FPN correction'-bit
     in the 'ADV_INQ_1'-Register. The availability of 'Blemish' can only be
     tested with the 'Presence_Inq'-bit in the 'BLEMISH_CONTROL'-register! */
  if(pData->bType == AVT_DSNU) {
    retval = avt1394_is_adv_inq1_feature(handle,node,AVT_FPN_FEATURE,&present);
    if((retval == DC1394_FAILURE) || (present == DC1394_FALSE)) {

#if defined(AVT_DEBUG)
      if(retval == DC1394_FAILURE)
	printf("\n'avt1394_is_adv_inq1_feature()' returned 'Failure'\n");
      else
	if(present == DC1394_FALSE)
	  printf("\n'AVT_FPN_FEATURE' not available\n");
#endif

      return DC1394_FAILURE;
    }

#if defined(AVT_DEBUG)
    printf("\nretrieving DSNU values");
#endif

    qOffset = REG_CAMERA_DSNU_CONTROL;
  }
  else {

#if defined(AVT_DEBUG)
    printf("\nretrieving Blemisch values");
#endif

    qOffset = REG_CAMERA_BLEMISH_CONTROL;
  }

  if(AvtGetCameraControlRegister(handle,node,
				 qOffset,
				 &quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

/*
  pData->bShowImg  = (quadval & 0x08000000UL) >> 27;
  pData->bCompute  = (quadval & 0x04000000UL) >> 26;
  pData->bBusy     = (quadval & 0x01000000UL) >> 24;
  pData->bLoadData = (quadval & 0x00400000UL) >> 22;
  pData->bZero     = (quadval & 0x00200000UL) >> 21;
  pData->nNumImg = quadval & 0x000000FFUL;
*/

  pData->bShowImg  = ((quadval & 0x08000000UL) != 0) ? DC1394_TRUE : DC1394_FALSE;
  pData->bCompute  = ((quadval & 0x04000000UL) != 0) ? DC1394_TRUE : DC1394_FALSE;
  pData->bBusy     = ((quadval & 0x01000000UL) != 0) ? DC1394_TRUE : DC1394_FALSE;
  pData->bLoadData = ((quadval & 0x00400000UL) != 0) ? DC1394_TRUE : DC1394_FALSE;
  pData->bZero     = ((quadval & 0x00200000UL) != 0) ? DC1394_TRUE : DC1394_FALSE;
  pData->nNumImg = quadval & 0x000000FFUL;

  *on = ((quadval & AVT_ON_OFF_MASK) != 0) ? DC1394_TRUE : DC1394_FALSE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_set_dsnu_blemish

 This routine sets the DSNU- and Blemish-values respectively,
 without influencing the state of the feature
 (-> 'avt1394_enable_dsnu_blemish()').
 If the feature is busy it return DC1394_FAILURE.
**********************************************************/
int 
avt1394_set_dsnu_blemish(raw1394handle_t handle, nodeid_t node,
			 avt_dsnu_blemish_t oData) {
  dc1394bool_t retval, present;
  quadlet_t quadval,qOffset;
   

  if(oData.bType == AVT_DSNU) {
    retval = avt1394_is_adv_inq1_feature(handle,node,AVT_FPN_FEATURE,&present);

    if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
      return DC1394_FAILURE;

    qOffset = REG_CAMERA_DSNU_CONTROL;
  }
  else {
    qOffset = REG_CAMERA_BLEMISH_CONTROL;
  }

  if(AvtGetCameraControlRegister(handle,node,
				 qOffset,
				 &quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;
	
  if((quadval & 0x01000000UL) != 0) {
    /* exit if feature is busy */
    return DC1394_FAILURE;
  }

/*
  if(oData.bShowImg)
    quadval = quadval | (oData.bShowImg << 27);
  else
    quadval = quadval & ~(oData.bShowImg << 27);

  if(oData.bCompute)
    quadval = quadval | (oData.bCompute << 26);
  else
    quadval = quadval & ~(oData.bCompute << 26);

  if(oData.bLoadData)
    quadval = quadval | (oData.bLoadData << 22);
  else
    quadval = quadval & ~(oData.bLoadData << 22);

  if(oData.bZero)
    quadval = quadval | (oData.bZero << 21);
  else
    quadval = quadval & ~(oData.bZero << 21);
*/
  quadval = (oData.bShowImg == DC1394_TRUE) ? (quadval | 0x08000000UL) : (quadval & 0xF7FFFFFFUL);
  quadval = (oData.bCompute == DC1394_TRUE) ? (quadval | 0x04000000UL) : (quadval & 0xFBFFFFFFUL);
  quadval = (oData.bLoadData == DC1394_TRUE) ? (quadval | 0x00400000UL) : (quadval & 0xFFBFFFFFUL);
  quadval = (oData.bZero == DC1394_TRUE) ? (quadval | 0x00200000UL) : (quadval & 0xFFDFFFFFUL);

#if defined(AVT_DEBUG)
  printf("\nQuadval %x",quadval);
#endif

  if(oData.nNumImg != 0) {
    /* set new number of images (max. 256) */
    quadval = (quadval & 0xFFFFFF00UL) | (oData.nNumImg & 0x000000FFUL);
  }

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  if(AvtSetCameraControlRegister(handle,node,
				 qOffset,quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_enable_dsnu_blemish

 This routine enables / disables dsnu or blemish control feature
**********************************************************/
int 
avt1394_enable_dsnu_blemish(raw1394handle_t handle, nodeid_t node,
			    dc1394bool_t type, dc1394bool_t bEnable) {
  dc1394bool_t retval, present;
  quadlet_t quadval, qOffset;
   

  if(type == AVT_DSNU) {
    retval = avt1394_is_adv_inq1_feature(handle, node, AVT_FPN_FEATURE, &present);
    if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
      return DC1394_FAILURE;

    qOffset = REG_CAMERA_DSNU_CONTROL;
  }
  else {
    qOffset = REG_CAMERA_BLEMISH_CONTROL;
  }

  if(AvtGetCameraControlRegister(handle,node,
				 qOffset,
				 &quadval, DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability */
  if((quadval & 0x80000000UL) == 0)
    return DC1394_FAILURE;

  quadval = (bEnable == DC1394_TRUE) ? (quadval | AVT_ON_OFF_MASK) : (quadval & ~AVT_ON_OFF_MASK);

  if(AvtSetCameraControlRegister(handle,node,
				 qOffset,quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_get_io_ctrl

 This routine retrieves input output control
**********************************************************/
int 
avt1394_get_io_ctrl(raw1394handle_t handle, nodeid_t node,
		    unsigned int nIoFeature, dc1394bool_t *pPolarity,
		    unsigned int *pMode, dc1394bool_t *pState) {
  dc1394bool_t retval, present;
  quadlet_t quadval, nIORegister;
  dc1394bool_t bOutput;
    

  /* determine the respective register-address (and if it's an output or input) */
  if(avt1394_get_io_register(nIoFeature, &nIORegister, &bOutput) == DC1394_FAILURE)
    return DC1394_FAILURE;

  /* determine if the feature is available */
  retval = avt1394_is_adv_inq2_feature(handle, node,
				       nIoFeature, &present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\n nIORegister =%x", nIORegister);
#endif

  /* read the current register-contents */
  if(AvtGetCameraControlRegister(handle, node,
				 nIORegister,
				 &quadval, DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x", quadval);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;	

  /* fill 'return-parameters' */
  *pMode = ((quadval >> 16) & 0x0000000FUL);
  *pPolarity = ((((quadval >> 24) & 0x00000001UL) == 0) ? DC1394_FALSE : DC1394_TRUE);
  *pState = (((quadval & 0x00000001UL) == 0) ? DC1394_FALSE : DC1394_TRUE);
/*
  if(*pPolarity == DC1394_FALSE) {
    *pState = (((quadval & 0x00000001UL) == 0) ? DC1394_TRUE : DC1394_FALSE);
  }
  else {
    *pState = (((quadval & 0x00000001UL) == 0) ? DC1394_FALSE : DC1394_TRUE);
  }
*/

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_set_io_ctrl

 This routine sets input output control
**********************************************************/
int 
avt1394_set_io_ctrl(raw1394handle_t handle, nodeid_t node,
		    unsigned int nIoFeature, dc1394bool_t bPolarity,
		    unsigned int nMode,dc1394bool_t nState) {
  dc1394bool_t retval, present;
  quadlet_t quadval, nIORegister;
  dc1394bool_t bOutput;
  

  /* determine IO-Register corresponding to current request / action */
  if(avt1394_get_io_register(nIoFeature, &nIORegister, &bOutput) == DC1394_FAILURE)
    return DC1394_FAILURE;

  /* test if the feature is implemented in camera */
  retval = avt1394_is_adv_inq2_feature(handle, node,
				       nIoFeature, &present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;

  /* read current register contents */
  if(AvtGetCameraControlRegister(handle,node,
				 nIORegister,
				 &quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  quadval &= 0xFEE0FFFFUL; /* set bits to change to '0' */
  quadval |= ((nMode << 16) & 0x001F0000UL); /* change 'mode' bits according to parameter */
  quadval |= (bPolarity == DC1394_TRUE) ? 0x01000000UL : 0x00000000UL; /* and 'polarity' bit too */

#if defined(AVT_DEBUG)
  printf("\nbOutput %d",bOutput);
#endif

  if(bOutput == DC1394_TRUE) {
/*
    quadval = (quadval & 0xFFFFFFFEUL) |
      ((nState == DC1394_TRUE) ? 
       ((bPolarity == DC1394_TRUE) ? 0x00000001UL : 0x00000000UL) :
       ((bPolarity == DC1394_TRUE) ? 0x00000000UL : 0x00000001UL));
*/

    /* change output-state according to parameter */
    quadval &= 0xFFFFFFFEUL;
    quadval |= ((nState == DC1394_TRUE) ? 0x00000001UL : 0x00000000UL);
/*
    if(bPolarity == DC1394_FALSE) {
      quadval |= ((nState == DC1394_TRUE) ? 0x00000000UL : 0x00000001UL);
    }
    else {
      quadval |= ((nState == DC1394_TRUE) ? 0x00000001UL : 0x00000000UL);
    }
*/
  }

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  if(AvtSetCameraControlRegister(handle, node,
				 nIORegister, quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_get_int_delay

 This routine retrieves integration delay time in usec
**********************************************************/
int 
avt1394_get_int_delay(raw1394handle_t handle, nodeid_t node,
		      unsigned int *pDelay, dc1394bool_t *on) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq2_feature(handle,node,
				       AVT_INTENADELAY_FEATURE,&present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;
	
  if(AvtGetCameraControlRegister(handle, node,
				 REG_CAMERA_INTENA_DELAY,
				 &quadval, DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability of Integration delay*/
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;
    
  *on = ((quadval & AVT_ON_OFF_MASK) != 0) ? DC1394_TRUE : DC1394_FALSE;
  
  *pDelay = quadval & 0x000FFFFF;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_set_int_delay

 This routine specifies the integration delay time in usec
**********************************************************/
int 
avt1394_set_int_delay(raw1394handle_t handle, nodeid_t node,
			 unsigned int nDelay) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq2_feature(handle,node,
				       AVT_INTENADELAY_FEATURE,&present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;
	
  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_INTENA_DELAY,
				 &quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  quadval = (quadval & 0xFFF00000UL) | (nDelay & 0x000FFFFFUL);

#if defined(AVT_DEBUG)
  printf("\nValue before setting is %x",quadval);
#endif

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_INTENA_DELAY,quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_enable_int_delay

 This routine enables / disables integration delay
**********************************************************/
int 
avt1394_enable_int_delay(raw1394handle_t handle, nodeid_t node, dc1394bool_t enable) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq2_feature(handle,node,
				       AVT_INTENADELAY_FEATURE,&present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;


  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_INTENA_DELAY,
				 &quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  if(enable)
    quadval = quadval | AVT_ON_OFF_MASK;
  else
    quadval = quadval & ~AVT_ON_OFF_MASK;

#if defined(AVT_DEBUG)
  printf("\nQuadval before setting is %x",quadval);
#endif

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_INTENA_DELAY,quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_get_auto_shutter_limits

 This routine retrieves  the auto shutter limits (i.e. both
 minimum and maximum value)
**********************************************************/
int 
avt1394_get_auto_shutter_limits(raw1394handle_t handle, nodeid_t node,
				unsigned int *pMinVal,unsigned int *pMaxVal) {
  quadlet_t quadval;
   

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_AUTOSHUTTER_CTRL,
				 &quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability*/
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;
	   
  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_AUTOSHUTTER_LO,
				 &quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nMinQuadval is %x",quadval);
#endif

  *pMinVal = quadval;

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_AUTOSHUTTER_HI,
				 &quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nMaxQuadval is %x",quadval);
#endif

  *pMaxVal = quadval;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_set_auto_shutter_limits

 This routine specifies the auto shutter limits (i.e. both
 minimum and maximum value)
**********************************************************/
int 
avt1394_set_auto_shutter_limits(raw1394handle_t handle, nodeid_t node,
				unsigned int nMinVal,unsigned int nMaxVal) {
  quadlet_t quadval;


  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_AUTOSHUTTER_CTRL,
				 &quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;


#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_AUTOSHUTTER_LO,
				 nMinVal,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_AUTOSHUTTER_HI,
				 nMaxVal,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_get_auto_gain_limits

 This routine returns the auto gain limits (i.e. both
 minimum and maximum value)
**********************************************************/
int 
avt1394_get_auto_gain_limits(raw1394handle_t handle, nodeid_t node,
			     unsigned int *pMinVal,unsigned int *pMaxVal) {
  quadlet_t quadval;
   

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_AUTOGAIN_CTRL,
				 &quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nValue retrieved :%x",quadval);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;
	
  *pMaxVal = (quadval & 0x0FFF0000) >> 16;
  *pMinVal = quadval & 0x00000FFF;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_set_auto_gain_limits

 This routine specifies the auto gain limits (i.e. both
 minimum and maximum value)
**********************************************************/
int 
avt1394_set_auto_gain_limits(raw1394handle_t handle, nodeid_t node,
			     unsigned int nMinVal,unsigned int nMaxVal) {
  quadlet_t quadval;
  unsigned int nGainMin,nGainMax;


  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_AUTOGAIN_CTRL,
				 &quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nValue retrieved :%x",quadval);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  if(dc1394_get_min_value(handle,node,FEATURE_GAIN,&nGainMin) == DC1394_FAILURE)
    return DC1394_FAILURE;

  if(dc1394_get_max_value(handle,node,FEATURE_GAIN,&nGainMax) == DC1394_FAILURE)
    return DC1394_FAILURE;
	
	
#if defined(AVT_DEBUG)
  printf("\n Max Gain is %d , Min Gain is %d",nGainMax,nGainMin);
#endif

  quadval &= 0xF000F000UL;
  quadval |= ((nMaxVal << 16) & 0x0FFF0000UL);
  quadval |= (nMinVal & 0x00000FFFUL);

#if defined(AVT_DEBUG)
  printf("\nValue before Setting %x",quadval);
#endif

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_AUTOGAIN_CTRL,quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;
    
  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_get_auto_aoi_dimensions

 This routine retrieves the dimensions of the work area
 (AOI) for some autofunctions
**********************************************************/
int 
avt1394_get_auto_aoi_dimensions(raw1394handle_t handle, nodeid_t node,
				unsigned int *pLeft, unsigned int *pTop,
				unsigned int *pWidth, unsigned int *pHeight,
				dc1394bool_t *showWorkArea, dc1394bool_t *on) {
  quadlet_t quadval;
    	

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_AUTOFNC_AOI,&quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  *showWorkArea = (quadval & 0x08000000UL) ? DC1394_TRUE : DC1394_FALSE;
  *on = ((quadval & AVT_ON_OFF_MASK) != 0) ? DC1394_TRUE : DC1394_FALSE;

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_AF_AREA_POSITION,
				 &quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  *pTop = quadval & 0x0000FFFF;
  *pLeft = (quadval >> 16) & 0x0000FFFF;

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_AF_AREA_SIZE,
				 &quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  *pHeight = quadval & 0x0000FFFF;
  *pWidth  = (quadval >> 16) & 0x0000FFFF;	

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_set_auto_aoi_dimensions

 This routine sets the dimensions of the work area
 (AOI) for some autofunctions
**********************************************************/
int 
avt1394_set_auto_aoi_dimensions(raw1394handle_t handle, nodeid_t node,
				unsigned int nLeft, unsigned int nTop,
				unsigned int nWidth, unsigned int nHeight) {
  quadlet_t quadval;


  if(((nWidth % 4) != 0) ||((nHeight % 4) !=0))
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_AUTOFNC_AOI,&quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;
	
  quadval = (nLeft << 16) & 0xFFFF0000UL;
  quadval |= (nTop & 0x0000FFFFUL);

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_AF_AREA_POSITION,
				 quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  quadval = (nWidth << 16) & 0xFFFF0000UL;
  quadval |= (nHeight & 0x0000FFFFUL);

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_AF_AREA_SIZE,
				 quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_enable_auto_aoi

 This routine enables / disables the 'auto aoi' feature
**********************************************************/
int
avt1394_enable_auto_aoi(raw1394handle_t handle, nodeid_t node, dc1394bool_t enable) {
    
  quadlet_t quadval;
    	

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_AUTOFNC_AOI,&quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  /*Disable ON_OFF bit and work area*/
  if(enable)
    quadval = quadval | AVT_ON_OFF_MASK;
  else
    quadval = quadval & ~AVT_ON_OFF_MASK;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_AUTOFNC_AOI,quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_enable_auto_aoi_area

 This routine enables / disables the work area of the
 'auto aoi' feature
**********************************************************/
int 
avt1394_enable_auto_aoi_area(raw1394handle_t handle, nodeid_t node,
			     dc1394bool_t enable) {
  quadlet_t quadval;


  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_AUTOFNC_AOI,&quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;
	
  /* Enable work area */
  if(enable)
    quadval = quadval | 0x08000000;
  else
    quadval = quadval & 0xf7ffffff;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_AUTOFNC_AOI,quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_get_color_corr_state

 This routine returns the current state of the
 color correction feature (ON / OFF) 
 Mainly applicable for Marlin C Cameras only
**********************************************************/
int 
avt1394_get_color_corr_state(raw1394handle_t handle, nodeid_t node,
			     dc1394bool_t *on) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq1_feature(handle,node,
				       AVT_COLOR_CORR_FEATURE, &present);   
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_COLOR_CORR,&quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability of color correction*/
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;
	
  *on = ((quadval & AVT_ON_OFF_MASK) != 0) ? DC1394_TRUE : DC1394_FALSE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_enable_color_corr

 This routine enables or disables color correction feature. 
 Mainly applicable for Marlin C Cameras only
**********************************************************/
int 
avt1394_enable_color_corr(raw1394handle_t handle, nodeid_t node,
			   dc1394bool_t bEnable) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq1_feature(handle,node,
				       AVT_COLOR_CORR_FEATURE, &present);   
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_COLOR_CORR,&quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability of color correction*/
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  quadval = bEnable ? (quadval | AVT_ON_OFF_MASK) : (quadval & ~AVT_ON_OFF_MASK);

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_COLOR_CORR,quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_get_adv_trigger_delay

 This routine retrieves trigger delay time in usec
**********************************************************/
int 
avt1394_get_adv_trigger_delay(raw1394handle_t handle, nodeid_t node,
			      unsigned int *pDelay, dc1394bool_t *on) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq1_feature(handle,node,
				       AVT_TRIGGER_FEATURE,&present);

  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;
	
  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_TRIGGER_DELAY,
				 &quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability of the advanced trigger delay */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  /* to check ON_OFF bit */
  *on = ((quadval & AVT_ON_OFF_MASK) != 0) ? DC1394_TRUE : DC1394_FALSE;

  *pDelay = quadval & 0x001FFFFF;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_set_adv_trigger_delay

 This routine sets trigger delay specified in usec
**********************************************************/
int 
avt1394_set_adv_trigger_delay(raw1394handle_t handle, nodeid_t node,
			      unsigned int nDelay) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq1_feature(handle,node,
				       AVT_TRIGGER_FEATURE,&present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;
	
  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_TRIGGER_DELAY, &quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability of trigger delay*/
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;
	
  quadval = (quadval & 0xFFE00000UL) | (nDelay & 0x001FFFFFUL);

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_TRIGGER_DELAY,quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_enable_adv_trigger_delay

 This routine enables / disables trigger delay
**********************************************************/
int 
avt1394_enable_adv_trigger_delay(raw1394handle_t handle, nodeid_t node,
				 dc1394bool_t enable) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq1_feature(handle,node,
				       AVT_TRIGGER_FEATURE,&present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_TRIGGER_DELAY,
				 &quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability of trigger delay*/
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  /* set ON_OFF bit */
  if(enable)
    quadval = quadval | AVT_ON_OFF_MASK;
  else
    quadval = quadval & ~AVT_ON_OFF_MASK;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_TRIGGER_DELAY,quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_get_mirror_image_state

 This routine returns the current state of the mirror image
 feature
**********************************************************/
int 
avt1394_get_mirror_image_state(raw1394handle_t handle, nodeid_t node,
			       dc1394bool_t *on) {
  quadlet_t quadval;


  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_TIMEBASE,&quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_MIRROR_IMAGE,&quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability of mirror image*/
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  *on = ((quadval & AVT_ON_OFF_MASK) != 0) ? DC1394_TRUE : DC1394_FALSE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_enable_mirror_image

 This routine enables or disables mirror image
**********************************************************/
int 
avt1394_enable_mirror_image(raw1394handle_t handle, nodeid_t node,
			    dc1394bool_t bEnable) {
  quadlet_t quadval;


  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_MIRROR_IMAGE,&quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability of mirror image*/
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  quadval = bEnable ? (quadval | AVT_ON_OFF_MASK) : (quadval & ~AVT_ON_OFF_MASK);

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_MIRROR_IMAGE,quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_delayed_soft_reset

 This routine resets the camera via the SOFT_RESET register,
 while using 'delay' to delay the reset in 10ms steps
 (see manual for more information)
**********************************************************/
int 
avt1394_delayed_soft_reset(raw1394handle_t handle, nodeid_t node,
		   unsigned int nDelay) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq1_feature(handle,node,
				       AVT_SOFT_RESET_FEATURE,&present);   
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_SOFT_RESET, &quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability of mirror image*/
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  /* set the delay time */
  quadval = (quadval & 0x00000000UL) | nDelay;

  /* set the reset-flag */
  quadval = quadval | AVT_SOFT_RESET_MASK;

  /* execute 'reset' */
  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_SOFT_RESET, quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_soft_reset

 This routine resets the camera via the SOFT_RESET register
 (see manual for more information)
**********************************************************/
int 
avt1394_soft_reset(raw1394handle_t handle, nodeid_t node) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq1_feature(handle,node,
				       AVT_SOFT_RESET_FEATURE,&present);   
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_SOFT_RESET, &quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability of mirror image*/
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  /* set the reset-flag */
  quadval = quadval | AVT_SOFT_RESET_MASK;

  /* execute 'reset' */
  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_SOFT_RESET, quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_get_soft_reset_delay

 This routine reports the current soft-reset-delay value
**********************************************************/
int 
avt1394_get_soft_reset_delay(raw1394handle_t handle, nodeid_t node,
			     unsigned int *pDelay) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq1_feature(handle,node,
				       AVT_SOFT_RESET_FEATURE,&present);   
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_SOFT_RESET, &quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability of mirror image*/
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  /* extract the delay time */
  *pDelay = quadval & 0x00000FFFUL;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_set_soft_reset_delay

 This routine reports the current soft-reset-delay value
**********************************************************/
int 
avt1394_set_soft_reset_delay(raw1394handle_t handle, nodeid_t node,
			     unsigned int nDelay) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq1_feature(handle,node,
				       AVT_SOFT_RESET_FEATURE,&present);   
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_SOFT_RESET, &quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability of mirror image*/
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  /* change the delay value */
  quadval = (quadval & 0xFFFFF000UL) | (nDelay & 0x00000FFFUL);

  /* write the new value to camera */
  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_SOFT_RESET, quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_high_snr_enable

 Enables / Disables the 'High SNR' mode
**********************************************************/
int
avt1394_high_snr_enable(raw1394handle_t handle, nodeid_t node,
			dc1394bool_t bEnable) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq1_feature(handle,node,
				       AVT_HIGH_SNR_FEATURE,&present);   
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_HIGH_SNR, &quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability of mirror image*/
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  quadval = bEnable ? (quadval | AVT_ON_OFF_MASK) : (quadval & ~AVT_ON_OFF_MASK);

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_HIGH_SNR, quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_get_high_snr_info

 This routine reports the 'GrabCount' value of the
 HIGH_SNR register, and the status of the 'ON_OFF' flag
**********************************************************/
int
avt1394_get_high_snr_info(raw1394handle_t handle,
			  nodeid_t node,
			  unsigned int *pGrabCount,
			  dc1394bool_t *pEnabled) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq1_feature(handle,node,
				       AVT_HIGH_SNR_FEATURE,&present);   
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_HIGH_SNR, &quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability of mirror image*/
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  /* extract 'ON_OFF' flag */
  *pGrabCount = quadval & 0x000001FFUL;

  /* extract 'GrabCount' value */
  *pEnabled = (quadval & AVT_ON_OFF_MASK) == 0 ? DC1394_FALSE : DC1394_TRUE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_set_high_snr_grab_count

 This routine sets the 'GrabCount' value of the
 HIGH_SNR register
**********************************************************/
int
avt1394_set_high_snr_grab_count(raw1394handle_t handle,
				nodeid_t node,
				unsigned int nGrabCount) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq1_feature(handle,node,
				       AVT_HIGH_SNR_FEATURE,&present);   
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_HIGH_SNR, &quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability of mirror image*/
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  /* set the GrabCount value */
  quadval = (quadval & 0xFFFFFE00UL) | nGrabCount;

  /* write back to register */
  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_HIGH_SNR, quadval,
				 DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_enable_io_decoder

 This routine enables or disables  io decoder
**********************************************************/
int 
avt1394_enable_io_decoder(raw1394handle_t handle, nodeid_t node,
			  dc1394bool_t bEnable) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq2_feature(handle,node,AVT_INCDECODER_FEATURE,
				       &present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;
   
  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_IO_DECODER_CTRL,
				 &quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;	

  quadval = bEnable ? (quadval | AVT_ON_OFF_MASK) : (quadval & ~AVT_ON_OFF_MASK);

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_IO_DECODER_CTRL,
				 quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_set_io_decoder

 This routine sets io decoder value
**********************************************************/
int 
avt1394_set_io_decoder(raw1394handle_t handle, nodeid_t node,
		       unsigned int nCmp) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq2_feature(handle,node,AVT_INCDECODER_FEATURE,
				       &present);

  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;
   
  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_IO_DECODER_VAL,
				 &quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  quadval= (quadval & 0xF000FFFFUL) | ((nCmp << 16) & 0x0FFF0000UL);

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_IO_DECODER_VAL,
				 quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_get_io_decoder

 This routine retrieves the counter value of io decoder
**********************************************************/
int 
avt1394_get_io_decoder(raw1394handle_t handle, nodeid_t node,
		       unsigned int *pCounter, unsigned int *pCmp,
		       dc1394bool_t *on) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq2_feature(handle,node,AVT_INCDECODER_FEATURE,
				       &present);

  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_IO_DECODER_VAL,
				 &quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  *pCounter = quadval & 0x00000FFFUL;
  *pCmp = (quadval >> 16) & 0x00000FFFUL;

  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_IO_DECODER_CTRL,
				 &quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  *on = ((quadval & AVT_ON_OFF_MASK) != 0) ? DC1394_TRUE : DC1394_FALSE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_reset_io_decoder_counter

 Clears the IO decoders position counter ('auto-reset')
**********************************************************/
int 
avt1394_reset_io_decoder_counter(raw1394handle_t handle, nodeid_t node) {
  dc1394bool_t retval, present;
  quadlet_t quadval;


  retval = avt1394_is_adv_inq2_feature(handle,node,AVT_INCDECODER_FEATURE,
				       &present);
  if((retval == DC1394_FAILURE) || (present == DC1394_FALSE))
    return DC1394_FAILURE;
   
  if(AvtGetCameraControlRegister(handle,node,
				 REG_CAMERA_IO_DECODER_CTRL,
				 &quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  /* check current availability */
  if(avt1394_check_availability(quadval) == DC1394_FAILURE)
    return DC1394_FAILURE;

  quadval |= 0x01000000UL;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x",quadval);
#endif

  if(AvtSetCameraControlRegister(handle,node,
				 REG_CAMERA_IO_DECODER_CTRL,
				 quadval,DC1394_FALSE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_get_recv_serialbuffersize

 This routine retrieves the RBUF_ST and RBUF_CNT
**********************************************************/
int 
avt1394_get_recv_serialbuffersize(raw1394handle_t handle, nodeid_t node,
				  unsigned int *pRbufSt, unsigned int *pRbufCnt) {
  quadlet_t quadval;


  /* query 'Opt_Function_Inq' for availability of 'SIO' */
  if(AvtDCGetCameraControlRegister(handle, node,
				   REG_OPT_FUNCTION_INQ,
				   &quadval) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle, node,
				 AVT_RCV_BUF_STATUS_CTRL,
				 &quadval, DC1394_TRUE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x", quadval);
#endif

  *pRbufSt = (quadval & 0xFF000000UL) >> 24;
  *pRbufCnt = (quadval & 0x00FF0000UL) >> 16;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_set_recv_serialbuffersize

 This routine sets serial buffer size
**********************************************************/
int 
avt1394_set_recv_serialbuffersize(raw1394handle_t handle, nodeid_t node,
				  unsigned int nSize) {
  quadlet_t quadval;
       

  /* query 'Opt_Function_Inq' for availability of 'SIO' */
  if(AvtDCGetCameraControlRegister(handle, node,
				   REG_OPT_FUNCTION_INQ,
				   &quadval) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle, node,
				 AVT_RCV_BUF_STATUS_CTRL,
				 &quadval, DC1394_TRUE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x", quadval);
#endif

  quadval = (quadval & 0xFF00FFFFUL) | ((nSize << 16) & 0x00FF0000UL);

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x", quadval);
#endif

  if(AvtSetCameraControlRegister(handle, node,
				 AVT_RCV_BUF_STATUS_CTRL,
				 quadval, DC1394_TRUE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_get_tx_serialbuffersize

 This routine retrieves the TBUF_ST and TBUF_CNT
**********************************************************/
int 
avt1394_get_tx_serialbuffersize(raw1394handle_t handle, nodeid_t node,
				unsigned int *pTbufSt, unsigned int *pTbufCnt) {
  quadlet_t quadval;


  /* query 'Opt_Function_Inq' for availability of 'SIO' */
  if(AvtDCGetCameraControlRegister(handle, node,
				   REG_OPT_FUNCTION_INQ,
				   &quadval) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle, node,
				 AVT_TX_BUF_STATUS_CTRL,
				 &quadval, DC1394_TRUE) != DC1394_SUCCESS)
    return DC1394_FAILURE;
    
#if defined(AVT_DEBUG)
  printf("\nQuadval is %x", quadval);
#endif

  *pTbufSt  = (quadval & 0xFF000000UL) >> 24;
  *pTbufCnt = (quadval & 0x00FF0000UL) >> 16;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_set_tx_serialbuffersize

 This routine sets serial buffer size
**********************************************************/
int 
avt1394_set_tx_serialbuffersize(raw1394handle_t handle, nodeid_t node,
				unsigned int nSize) {
  quadlet_t quadval;
       

  /* query 'Opt_Function_Inq' for availability of 'SIO' */
  if(AvtDCGetCameraControlRegister(handle, node,
				   REG_OPT_FUNCTION_INQ,
				   &quadval) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle, node,
				 AVT_TX_BUF_STATUS_CTRL,
				 &quadval, DC1394_TRUE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x", quadval);
#endif

  quadval = (quadval & 0xFF00FFFFUL) | ((nSize << 16) & 0x00FF0000UL);

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x", quadval);
#endif

  if(AvtSetCameraControlRegister(handle, node,
				 AVT_TX_BUF_STATUS_CTRL,
				 quadval, DC1394_TRUE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_get_serial_data

 This routine retrieves characters from SIO_DATA_REGISTER.
**********************************************************/
int 
avt1394_get_serial_data(raw1394handle_t handle, nodeid_t node,
		       avt_serial_data_t *pData) {
  quadlet_t quadval;
    

  /* query 'Opt_Function_Inq' for availability of 'SIO' */
  if(AvtDCGetCameraControlRegister(handle, node,
				   REG_OPT_FUNCTION_INQ,
				   &quadval) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle, node,
				 AVT_SERIAL_DATA,
				 &quadval, DC1394_TRUE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x", quadval);
#endif

  pData->nChar0 = (quadval & 0xFF000000UL) >> 24;
  pData->nChar1 = (quadval & 0x00FF0000UL) >> 16;
  pData->nChar2 = (quadval & 0x0000FF00UL) >> 8;
  pData->nChar3 =  quadval & 0x000000FFUL;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_send_serial_data

 This routine sends serial data
**********************************************************/
int 
avt1394_send_serial_data(raw1394handle_t handle, nodeid_t node,
			 avt_serial_data_t oData) {
  quadlet_t quadval;
       

  /* query 'Opt_Function_Inq' for availability of 'SIO' */
  if(AvtDCGetCameraControlRegister(handle, node,
				   REG_OPT_FUNCTION_INQ,
				   &quadval) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  quadval = ((oData.nChar0 << 24) & 0xFF000000UL) |
    ((oData.nChar1 << 16) & 0x00FF0000UL) | 
    ((oData.nChar2 << 8) & 0x0000FF00UL) |
    ((oData.nChar3) & 0x000000FFUL);

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x", quadval);
#endif

  if(AvtSetCameraControlRegister(handle, node,
				AVT_SERIAL_DATA,
				quadval, DC1394_TRUE) != DC1394_SUCCESS)
    return DC1394_FAILURE;
    
  return DC1394_SUCCESS;
}   


/********************************************************
 avt1394_get_serial_mode

 This routine retrieves serial mode
**********************************************************/
int 
avt1394_get_serial_mode(raw1394handle_t handle, nodeid_t node,
			avt_serial_mode_t *pData) {
  quadlet_t quadval;
   

  /* query 'Opt_Function_Inq' for availability of 'SIO' */
  if(AvtDCGetCameraControlRegister(handle, node,
				   REG_OPT_FUNCTION_INQ,
				   &quadval) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle, node,
				 AVT_SERIAL_MODE,
				 &quadval, DC1394_TRUE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x", quadval);
#endif

  if(avt1394_get_baudrate((quadval & 0xFF000000UL) >> 24,
			  &(pData->nBaudRate)) == DC1394_FAILURE)
    return DC1394_FAILURE;

  if(avt1394_get_charlen((quadval & 0x00FF0000UL) >> 16,
				    &(pData->nCharLen)) == DC1394_FAILURE)
    return DC1394_FAILURE;

  if(avt1394_get_parity((quadval & 0x0000c000UL) >> 14,
			&(pData->nParity)) == DC1394_FAILURE)
    return DC1394_FAILURE;

  if(avt1394_get_stopbit((quadval & 0x00003000UL) >> 12,
			 &(pData->nStopBit)) == DC1394_FAILURE)
    return DC1394_FAILURE;

  pData->nBufferSize = quadval & 0x000000FFUL;	

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_set_serial_mode

 This routine sets serial mode information
**********************************************************/
int 
avt1394_set_serial_mode(raw1394handle_t handle, nodeid_t node,
			avt_serial_mode_t oData) {
  quadlet_t quadval;
  unsigned int nBaud, nCharLen, nParity, nStopBit;
   

  /* query 'Opt_Function_Inq' for availability of 'SIO' */
  if(AvtDCGetCameraControlRegister(handle, node,
				   REG_OPT_FUNCTION_INQ,
				   &quadval) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle, node,
				 AVT_SERIAL_MODE,
				 &quadval, DC1394_TRUE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nInternal Quadval is %x", quadval);
  printf("\nBaud rate is %d", oData.nBaudRate);
#endif

  if(avt1394_set_baud_rate(oData.nBaudRate, &nBaud) == DC1394_FAILURE)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nBaudRate set");
#endif

  if(avt1394_set_char_len(oData.nCharLen, &nCharLen) == DC1394_FAILURE)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nCharLen set");
#endif

  if(avt1394_set_parity(oData.nParity, &nParity) == DC1394_FAILURE)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nParity set");
#endif

  if(avt1394_set_stopbit(oData.nStopBit, &nStopBit) == DC1394_FAILURE)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nStopBits set");
#endif

  quadval = (quadval & 0x00000FFFUL)
    | ((nBaud & 0x000000FFUL) << 24)
    | ((nCharLen & 0x000000FFUL) << 16) 
    | ((nParity & 0x00000003UL) << 14) 
    | ((nStopBit & 0x00000003UL) << 12);

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x", quadval);
#endif

  if(AvtSetCameraControlRegister(handle, node,
				 AVT_SERIAL_MODE,
				 quadval, DC1394_TRUE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}


/********************************************************
 avt1394_get_serial_control

 This routine retrieves serial control information
**********************************************************/
int 
avt1394_get_serial_control(raw1394handle_t handle, nodeid_t node,
			   avt_serial_control_t *pData) {
  quadlet_t quadval;

   
  /* query 'Opt_Function_Inq' for availability of 'SIO' */
  if(AvtDCGetCameraControlRegister(handle, node,
				   REG_OPT_FUNCTION_INQ,
				   &quadval) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle, node,
				 AVT_SERIAL_CTRL,
				 &quadval, DC1394_TRUE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x", quadval);
#endif

  pData->bRcvEnable       = (quadval & 0x80000000UL) >> 31;
  pData->bTxEnable        = (quadval & 0x40000000UL) >> 30;
  pData->bTxBufferReady   = (quadval & 0x00800000UL) >> 23;
  pData->bRcvBufferReady  = (quadval & 0x00200000UL) >> 21;
  pData->bRcvOverrunError = (quadval & 0x00080000UL) >> 19;
  pData->bFramingError    = (quadval & 0x00040000UL) >> 18;
  pData->bParityError     = (quadval & 0x00020000UL) >> 17;

  return DC1394_SUCCESS;
}

 
/********************************************************
 avt1394_set_serial_control

 This routine sets serial control information
**********************************************************/
int 
avt1394_set_serial_control(raw1394handle_t handle, nodeid_t node,
			   avt_serial_control_t oData) {
  quadlet_t quadval;

   
  /* query 'Opt_Function_Inq' for availability of 'SIO' */
  if(AvtDCGetCameraControlRegister(handle, node,
				   REG_OPT_FUNCTION_INQ,
				   &quadval) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  if(AvtGetCameraControlRegister(handle, node,
				 AVT_SERIAL_CTRL,
				 &quadval, DC1394_TRUE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x", quadval);
#endif

  quadval = quadval & 0x3FF1FFFFUL;
  quadval =  quadval | (oData.bRcvEnable << 31)
    | (oData.bTxEnable << 30) 
    | (oData.bRcvOverrunError << 19) 
    | (oData.bFramingError << 18)
    | (oData.bParityError <<17);

#if defined(AVT_DEBUG)
  printf("\nQuadval is %x", quadval);
#endif

  if(AvtSetCameraControlRegister(handle, node,
				 AVT_SERIAL_CTRL,
				 quadval, DC1394_TRUE) != DC1394_SUCCESS)
    return DC1394_FAILURE;

  return DC1394_SUCCESS;
}
 
   
