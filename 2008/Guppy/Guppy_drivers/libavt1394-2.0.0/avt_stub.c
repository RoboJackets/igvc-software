/*
 * IIDC1394-Based Digital Camera Control Library Extension for AVT Cameras
 * 
 * Test-Application
 *
 * Written by Menuka
 */


#include "avt1394.h"


raw1394handle_t g_handle;
nodeid_t *g_pNode;


void Initialize() {
  int numcamera;
  dc1394bool_t bPresent;
  int retval,port;
  nodeid_t *pNodes;
  quadlet_t quadval;
  dc1394_miscinfo info;

  g_handle = dc1394_create_handle(0);
  if(g_handle == NULL) {
    printf("\nError ....handle is NULL");
    exit(1);
  }
  else
    printf("\nHandle created successfully");

  port = dc1394_get_camera_port(g_handle);
  printf("\nPort %d",port);
  pNodes = dc1394_get_camera_nodes(g_handle,&numcamera,1);
  printf("\nNumber of camera are %d",numcamera);
  retval = dc1394_is_camera(g_handle,pNodes[0],&bPresent);
  if(retval == DC1394_FAILURE) {
    printf("\nNode is not camera ");
    exit(1);
  }
  else
    printf("Node is camera");
  if(bPresent == 1)
    g_pNode = &(pNodes[0]);
  else
    exit(1);
  printf("Node is %d",*g_pNode);
  printf("\nNodes info %d",pNodes[0]);

  /*Getting ISo status*/
  retval = dc1394_get_iso_status(g_handle,*g_pNode,&bPresent);
  if(retval == DC1394_FAILURE)
    printf("\nFailure in getting ISO status");
  printf("\nValue of ISO status is %d",bPresent);  

  retval =dc1394_query_basic_functionality(g_handle,*g_pNode,&quadval);
  if(retval == DC1394_FAILURE)
    printf("\nFailure in getting normal feature offset");
  printf("\nBasic functionality %x",quadval);
  info.mem_channel_number= (quadval & 0xF);
  printf("\nChannel number %d", info.mem_channel_number);  
  /*Query Advanced Feature offset*/
  retval = dc1394_query_advanced_feature_offset(g_handle,*g_pNode,&quadval);
  if(retval == DC1394_FAILURE)
    printf("\nFailure in getting advanced feature offset");
  printf("\nAdvanced Feature offset is  %x",quadval);
   
   
}

void microprocessor_inquiry() {
  unsigned int nMajor,nMinor;
  int retval;
       
  retval = avt1394_get_uc_version(g_handle,*g_pNode,&nMajor,&nMinor);
  if(retval == DC1394_FAILURE) {
    printf("\nFailure in Microprocessor inquiry");
    exit(1);
  }
  printf("\n Major Version number is %d",nMajor);
  printf("\n Minor Version number is %d",nMinor);
}

void timebase_inq() {
  int value,retval;
   
  printf("\nInvoking getting timebase");
  retval = avt1394_get_timebase(g_handle,*g_pNode,&value);
  if(retval == DC1394_FAILURE) {
    printf("\nFailure in timebase inquiry");
    exit(1);
  }
  else
    printf("\n Timebase is %d us",value);
}

void set_time() {
  int retval,nTimeBase;

  printf("\nEnter the timebase:");
  scanf("%d",&nTimeBase);
  retval = avt1394_set_timebase(g_handle,*g_pNode,nTimeBase);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in setting time");
  else
    printf("\nFailure in Setting Time");
}

void max_res() {
  int retval;
  unsigned int nWidth,nHeight;

  retval = avt1394_get_max_resolution(g_handle,*g_pNode,&nWidth,&nHeight);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in retrieving Maximum Resolution");
  else {
    printf("\nFailure in retrieving Max resolution");
    exit(1);
  }
  printf("\nWidth is %d",nWidth);
  printf("\nHeight is %d",nHeight);
}

void ShowCameraType(unsigned int nCameraId) {

  switch(nCameraId) {
  case AVT_CAMERA_F145B: 
    printf("\nCamera Type is F145B");
    break;
  case AVT_CAMERA_F145C: 
    printf("\nCamera Type is F145C");
    break;
  case AVT_CAMERA_F201B: 
    printf("\nCamera Type is F201B");
    break;
  case AVT_CAMERA_F201C: 
    printf("\nCamera Type is F201C");
    break;
  case AVT_CAMERA_F145B1: 
    printf("\nCamera Type is F145B-1");
    break;
  case AVT_CAMERA_F145C1: 
    printf("\nCamera Type is F145C-1");
    break;
  case AVT_CAMERA_F201B1: 
    printf("\nCamera Type is F201B-1");
    break;
  case AVT_CAMERA_F201C1: 
    printf("\nCamera Type is F201C-1");
    break;
  case AVT_CAMERA_MF033B: 
    printf("\nCamera Type is MF033B");
    break;
  case AVT_CAMERA_MF033C: 
    printf("\nCamera Type is MF033C");
    break;
  case AVT_CAMERA_MF046B: 
    printf("\nCamera Type is MF046B");
    break;
  case AVT_CAMERA_MF046C: 
    printf("\nCamera Type is MF046C");
    break;
  case AVT_CAMERA_MF080B: 
    printf("\nCamera Type is MF080B");
    break;     
  case AVT_CAMERA_MF080C: 
    printf("\nCamera Type is MF080C");
    break;
  case AVT_CAMERA_MF145B2: 
    printf("\nCamera Type is MF145B2");
    break;
  case AVT_CAMERA_MF145C2: 
    printf("\nCamera Type is MF145C2");
    break;
  case AVT_CAMERA_MF131B: 
    printf("\nCamera Type is MF131B");
    break;
  case AVT_CAMERA_MF131C: 
    printf("\nCamera Type is MF131C");
    break;
  case AVT_CAMERA_MF145B215: 
    printf("\nCamera Type is MF145B2-15fps");
    break;
  case AVT_CAMERA_MF145C215: 
    printf("\nCamera Type is MF145C2-15fps");
    break;
  case AVT_CAMERA_M2F033B:
    printf("\nCamera Type is M2F033B");
    break;
  case AVT_CAMERA_M2F033C:
    printf("\nCamera Type is M2F033C");
    break;
  case AVT_CAMERA_M2F046B:
    printf("\nCamera Type is M2F046B");
    break;
  case AVT_CAMERA_M2F046C:
    printf("\nCamera Type is M2F046C");
    break;
  case AVT_CAMERA_M2F080B:
    printf("\nCamera Type is M2F080B");
    break;
  case AVT_CAMERA_M2F080C:
    printf("\nCamera Type is M2F080C");
    break;
  case AVT_CAMERA_M2F145B2:
    printf("\nCamera Type is M2F145B2");
    break;
  case AVT_CAMERA_M2F145C2:
    printf("\nCamera Type is M2F145C2");
    break;
  case AVT_CAMERA_M2F145B215:
    printf("\nCamera Type is M2F145B215");
    break;
  case AVT_CAMERA_M2F145C215:
    printf("\nCamera Type is M2F145C215");
    break;
  case AVT_CAMERA_M2F080B30:
    printf("\nCamera Type is M2F080B30");
    break;
  case AVT_CAMERA_M2F080C30:
    printf("\nCamera Type is M2F080C30");
    break;
  case AVT_CAMERA_OF320C:
    printf("\nCamera Type is OF320C");
    break;
  case AVT_CAMERA_OF510C:
    printf("\nCamera Type is OF510C");
    break;
  case AVT_CAMERA_OF810C:
    printf("\nCamera Type is OF810C");
    break;
  default:
    printf("\nUnknown Camera Type");
    break;
  }
}

void firmware_inquiry() {
  int retval;
  avt_firmware_t oData;

  retval = avt1394_get_firmware_version(g_handle,*g_pNode,&oData);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in retrieving Firmware Version");
  else {
    printf("\nFailure in retrieving Firmware Version");
    exit(1);
  }
  printf("\nnMinor Version is %d",oData.nMinor);
  printf("\nnMajor Version is %d",oData.nMajor);
  printf("\nCamera Id is %d",oData.nCameraId);
  ShowCameraType(oData.nCameraId);
  if(oData.bMarlin == DC1394_TRUE)
    printf("\nCamera is marlin");
  else
    printf("\nCamera is dolphin");
}

void avt_get_exp_time() {
  int retval;
  unsigned int nExpTime;
  dc1394bool_t bValid;


  retval = avt1394_get_exposure_time(g_handle, *g_pNode, &nExpTime, &bValid);

  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in retrieving Exposure Time");
  else {
    printf("\nFailure in retrieving Exposure Time");
    exit(1);
  }
  
  if(bValid == DC1394_TRUE) {
    printf("\nExposure time is %d usec",nExpTime);
  }
  else {
    printf("\nExposure time 'should be' %d usec\n", nExpTime);
    printf("!Warning! - This value is only true, if the timebase has not changed\n");
    printf("            since the last change of the shutter-time\n");
  }
}

void avt_set_exp_time() {
  int retval;
  unsigned int nExpTime;

  printf("\n Enter the exposure time in usec:");
  scanf("%d",&nExpTime);
  retval = avt1394_set_exposure_time(g_handle,*g_pNode,nExpTime);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in setting Exposure Time");
  else {
    printf("\nFailure in setting Exposure Time");
    exit(1);
  }
    
}

void avt_get_int_delay() {
  int retval;
  unsigned int nIntDelay;
  dc1394bool_t on;


  retval = avt1394_get_int_delay(g_handle,*g_pNode,&nIntDelay, &on);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in retrieving Integration delay");
  else {
    printf("\nFailure in retrieving Integration Delay");
    exit(1);
  }
  printf("\nIntegration Delay time is %d usec",nIntDelay);  
  printf("\nFeature is %s\n", (on ? "enabled" : "disabled"));
}

void avt_en_int_delay() {
  int retval;
  
  
  retval = avt1394_enable_int_delay(g_handle,*g_pNode, DC1394_TRUE);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in setting Integration Delay");
  else {
    printf("\nFailure in setting Integration Delay");
    exit(1);
  }
  
}

void avt_dis_int_delay() {
  int retval;
  
  
  retval = avt1394_enable_int_delay(g_handle, *g_pNode, DC1394_FALSE);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in Disabling Integration Delay");
  else {
    printf("\nFailure in Disabling Integration Delay");
    exit(1);
  }
}

void avt_set_shutter() {
  int retval;
  unsigned int nMin,nMax;
  
  
  printf("\n Enter the minimum value of auto shutter:");
  scanf("%d",&nMin);
  printf("\n Enter the maximum value of auto shutter:");
  scanf("%d",&nMax);
  retval = avt1394_set_auto_shutter_limits(g_handle,*g_pNode,nMin,nMax);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in setting Auto shutter limits\n");
  else {
    printf("\nFailure in setting Auto shutter limits\n");
    exit(1);
  }    
}

void avt_get_shutter() {
  int retval;
  unsigned int nMin,nMax;

  
  retval = avt1394_get_auto_shutter_limits(g_handle,*g_pNode,&nMin,&nMax);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in retrieving  Auto shutter limits\n");
  else {
    printf("\nFailure in retrieving Auto shutter limits\n");
    exit(1);
  }    
  printf("\n Min Value of Auto Shutter is %d\n",nMin);
  printf("\n Max Value of Auto Shutter is %d\n",nMax);
}

void avt_set_gain() {
  int retval;
  unsigned int nMin,nMax;

  printf("\n Enter the minimum value of auto gain:");
  scanf("%d",&nMin);
  printf("\n Enter the maximum value of auto gain:");
  scanf("%d",&nMax);
  retval = avt1394_set_auto_gain_limits(g_handle,*g_pNode,nMin,nMax);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in setting Auto Gain Limits\n");
  else {
    printf("\nFailure in setting Auto Gain Limits\n");
    exit(1);
  }    
}

void avt_get_gain() {
  int retval;
  unsigned int nMin,nMax;

  retval = avt1394_get_auto_gain_limits(g_handle,*g_pNode,&nMin,&nMax);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in retrieving  Auto Gain Limits\n");
  else {
    printf("\nFailure in retrieving Auto Gain Limits\n");
    exit(1);
  }    
  printf("\n Min Value of Auto Gain is %d\n",nMin);
  printf("\n Max Value of Auto Gain is %d\n",nMax);
}

void avt_en_color() {
  int retval;
    
  retval = avt1394_enable_color_corr(g_handle,*g_pNode,DC1394_TRUE);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in Enabling color correction");
  else {
    printf("\nFailure in Enabling color correction");
    exit(1);
  }    
}

void avt_dis_color() {
  int retval;
    
  retval = avt1394_enable_color_corr(g_handle,*g_pNode,DC1394_FALSE);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in disabling color correction");
  else {
    printf("\nFailure in Disabling color correction");
    exit(1);
  }    
}

void avt_en_mirror() {
  int retval;
    
  retval = avt1394_enable_mirror_image(g_handle,*g_pNode,DC1394_TRUE);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in Enabling mirror image");
  else {
    printf("\nFailure in Enabling mirror image");
    exit(1);
  }    
}

void avt_dis_mirror() {
  int retval;
    
  retval = avt1394_enable_mirror_image(g_handle,*g_pNode,DC1394_FALSE);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in disabling mirror image");
  else {
    printf("\nFailure in Disabling mirror image");
    exit(1);
  }    
}

void avt_en_adv_trigger() {
  int retval;

  retval = avt1394_enable_adv_trigger_delay(g_handle, *g_pNode, DC1394_TRUE);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in enabling adv. trigger delay\n");
  else {
    printf("\nFailure in enabling adv. trigger delay\n");
    exit(1);
  }    
}

void avt_dis_adv_trigger() {
  int retval;
    
  retval = avt1394_enable_adv_trigger_delay(g_handle,*g_pNode, DC1394_FALSE);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in disabling adv. trigger delay\n");
  else {
    printf("\nFailure in Disabling adv. trigger delay\n");
    exit(1);
  }    
}

void avt_get_adv_trigger() {
  int retval,nTriggerDelay;
  dc1394bool_t on;

    
  retval = avt1394_get_adv_trigger_delay(g_handle, *g_pNode, &nTriggerDelay, &on);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in retrieving adv. trigger delay\n");
  else {
    printf("\nFailure in retrieving adv. trigger delay\n");
    exit(1);
  } 
  printf("\nAdv. Trigger Delay is %d usec\n",nTriggerDelay);
  printf("\nAdv. Trigger is %s\n", (on ? "enabled" : "disabled"));

}

void avt_get_frame() {
  int retval,nFrame;
    
  retval = avt1394_get_frame_info(g_handle,*g_pNode,&nFrame);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in retrieving frame counter");
  else {
    printf("\nFailure in retrieving frame counter");
    exit(1);
  } 
  printf("\n Frame Counter is %d ",nFrame);
}

void avt_reset_frame() {
  int retval;
    
  retval = avt1394_reset_frame_counter(g_handle,*g_pNode);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in resetting frame counter");
  else {
    printf("\nFailure in resetting frame counter");
    exit(1);
  }     
}

void avt_get_test() {
  unsigned int retval,nActive,nPresent;
    
  retval = avt1394_get_test_image(g_handle,*g_pNode,&nPresent,&nActive);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in getting test image");
  else {
    printf("\nFailure in getting test image");
    exit(1);
  } 
  printf("\n Active test images %d",nActive);
  printf("\nPresent value %d",nPresent);
  if(nPresent & 0x40)
    printf("\n Test image 1 is present");
}

void avt_set_test() {
  unsigned int retval,nActive;
  printf("\nEnter the image number u want to activate:");
  scanf("%d",&nActive);
    
  retval = avt1394_set_test_image(g_handle,*g_pNode,nActive);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in setting test image");
  else {
    printf("\nFailure in setting test image");
    exit(1);
  }    
}

void avt_get_hdr() {
  unsigned int retval,nMaxKnee,nKnee1,nKnee2,nKnee3;
  dc1394bool_t on;
    
  retval = avt1394_get_hdr_info(g_handle,*g_pNode,
				&nMaxKnee,
				&nKnee1,&nKnee2,&nKnee3, &on);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in getting hdr info");
  else {
    printf("\nFailure in getting hdr info\n");
    exit(1);
  }
  printf("\nMaximum Knee points: %d",nMaxKnee);
  switch(nMaxKnee) {
  case 1:
    printf("\n Knee point 1: %d",nKnee1);
    break;
  case 2:
    printf("\n Knee point 1: %d",nKnee1);
    printf("\n Knee point 2: %d",nKnee2);
    break;
  case 3:
    printf("\n Knee point 1: %d",nKnee1);
    printf("\n Knee point 2: %d",nKnee2);
    printf("\n Knee point 3: %d",nKnee3);
    break;
  default:
    printf("\n only 3 kneepoints are supported");
    break;
  }

  printf("\n HDR is switched %s\n", (on ? "on" : "off"));
}

void avt_get_io() {
  unsigned int retval,nIOFeature,nMode;
  dc1394bool_t bPolarity,bState;

  printf("\nEnter the feature value to be retrieved:");
  scanf("%d",&nIOFeature);
  retval = avt1394_get_io_ctrl(g_handle,*g_pNode,nIOFeature,
			       &bPolarity,&nMode,&bState);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in retrieving IO info");
  else {
    printf("\nFailure in getting IO info");
    exit(1);
  } 
  printf("\n Polarity is   %d",bPolarity);
  printf("\nMode is %d",nMode);
  printf("\n State is  %d",bState);
    
}

void avt_set_io() {
    
  unsigned int retval,nIOFeature,nMode, bPolarity, bState;
  //    dc1394bool_t bPolarity,bState;
    
  printf("\nEnter the feature value to be retrieved:");
  scanf("%d", &nIOFeature);
  printf("\nEnter the polarity:");
  scanf("%d", &bPolarity);
  printf("\nEnter the mode:");
  scanf("%d", &nMode);
  printf("\nEnter the state:");
  scanf("%d", &bState);
  retval = avt1394_set_io_ctrl(g_handle, *g_pNode,
			       nIOFeature, (dc1394bool_t)bPolarity,
			       nMode, (dc1394bool_t)bState);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in setting IO info");
  else {
    printf("\nFailure in setting IO info");
    exit(1);
  }  
    
}

void avt_en_hdr() {
  unsigned int retval;
    
  retval = avt1394_enable_hdr_mode(g_handle,*g_pNode, DC1394_TRUE);
  if(retval == DC1394_SUCCESS)
    printf("\nEnable hdr successful\n");
  else {
    printf("\nFailure in enabling  hdr info\n");
    exit(1);
  } 
}

void avt_dis_hdr() {
  unsigned int retval;
    
  retval = avt1394_enable_hdr_mode(g_handle,*g_pNode, DC1394_FALSE);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in disabling hdr mode\n");
  else {
    printf("\nFailure in disabling hdr mode\n");
    exit(1);
  }     
}

void avt_set_deferr() {
  unsigned int retval, nImg;
  unsigned int bSend, bHold, bFast;
    
  printf("\nEnter the send image:");
  scanf("%d",&bSend);
  printf("\nEnter the hold image:");
  scanf("%d",&bHold);
  printf("\nEnter the fast image:");
  scanf("%d",&bFast);
  printf("\nEnter the number of images:");
  scanf("%d",&nImg);
  retval = avt1394_set_deferr_trans(g_handle,*g_pNode,
				    (dc1394bool_t)bSend,
				    (dc1394bool_t)bHold,
				    (dc1394bool_t)bFast, nImg);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in setting deferr transport");
  else {
    printf("\nFailure in setting deferr transport");
    exit(1);
  }  
    
}

void avt_get_deferr() {
  unsigned int retval,nFifo,nImg;
  dc1394bool_t bSend,bHold,bFast;

   
  retval = avt1394_get_deferr_trans(g_handle,*g_pNode,&bSend,&bHold,
				    &bFast,&nFifo,&nImg);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in retrieving deferred transport");
  else {
    printf("\nFailure in getting deferred transport");
    exit(1);
  } 
  printf("\n Send Image is    %d",bSend);
  printf("\nHold Image is %d",bHold);
  printf("\nFast capture is  %d",bFast);
  printf("\nFifo size is %d",nFifo);
  printf("\nNum of images are %d",nImg);
}

void avt_get_gpdata() {
  unsigned int retval,nSize;
    
  retval = avt1394_get_gpdata_size(g_handle,*g_pNode,&nSize);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in retrieving gpdata");
  else {
    printf("\nFailure in getting gpdata");
    exit(1);
  } 
  printf("\n Buffer size  is %d",nSize);    
}

void avt_get_auto_aoi_dim() {
  unsigned int retval,nLeft,nTop,nWidth,nHeight;
  dc1394bool_t showArea, on;

    
  retval = avt1394_get_auto_aoi_dimensions(g_handle, *g_pNode,
					   &nLeft, &nTop,
					   &nWidth, &nHeight,
					   &showArea, &on);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in retrieving image area\n");
  else {
    printf("\nFailure in getting image area\n");
    exit(1);
  } 

  printf("\nLeft  is %d",nLeft);
  printf("\nTop is %d",nTop);
  printf("\nWidth is %d",nWidth);
  printf("\nHeight is %d",nHeight);
  printf("\nshowWorkArea is %s", (showArea ? "enabled" : "disabled"));
  printf("\nAutofunction-AOI is %s\n", (on ? "enabled" : "disabled"));
}

void avt_en_auto_aoi() {
  unsigned int retval;
    
  retval = avt1394_enable_auto_aoi(g_handle,*g_pNode, DC1394_TRUE);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in enabling auto aoi\n");
  else {
    printf("\nFailure in enabling auto aoi\n");
    exit(1);
  } 
  
}

void avt_dis_auto_aoi() {
  unsigned int retval;
    
  retval = avt1394_enable_auto_aoi(g_handle,*g_pNode, DC1394_FALSE);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in disabling auto aoi\n");
  else {
    printf("\nFailure in disabling auto aoi\n");
    exit(1);
  } 
   
}

void avt_load_lut() {
  unsigned int retval;
  unsigned int nLut,nSize;
  unsigned char pData[2049];
  int i, nCount = 0;
        
  printf("\nEnter the number of LUT:");

  scanf("%d", &nLut);

  while(nCount < 1024) {
    for(i = 0; i < 256; i++) {
      pData[nCount] = i;
      nCount++;
      if(nCount > 1023)
	break;	    
      /*printf("\n Ncount%d",nCount);*/
    }
  }
   
  printf("\nEnter the size:");

  scanf("%d",&nSize);

  retval = avt1394_load_lut(g_handle, *g_pNode, nLut, pData, nSize);

  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in loading lut ");
  else {
    printf("\nFailure in loading lut");
    exit(1);
  } 
}

void avt_en_lut() {
  unsigned int retval;
     
  retval = avt1394_enable_lut(g_handle, *g_pNode, DC1394_TRUE);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in enabling lut ");
  else {
    printf("\nFailure in enabling lut");
    exit(1);
  } 
}

void avt_dis_lut() {
  unsigned int retval;

  retval = avt1394_enable_lut(g_handle, *g_pNode, DC1394_FALSE);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in disabling lut ");
  else {
    printf("\nFailure in disabling lut");
    exit(1);
  } 
}

void avt_get_dsnu() {
  unsigned int retval;
  avt_dsnu_blemish_t oData;
  dc1394bool_t on;
     
     
  printf("\nEnter the type (0-Blemish ,1-DSNU):");
  scanf("%d",(unsigned int *)&oData.bType);      
    
  retval = avt1394_get_dsnu_blemish(g_handle,*g_pNode,&oData, &on);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in retrieving dsnu/blemish\n");
  else {
    printf("\nFailure in retrieving dsnu/blemish\n");
    exit(1);
  } 
  printf("\n bShow is %d",oData.bShowImg);
  printf("\n bCompute is %d",oData.bCompute);
  printf("\n bBusy is %d",oData.bBusy);
  printf("\n bLoad is %d",oData.bLoadData);
  printf("\n bZero is %d",oData.bZero);
  printf("\n Image number is %d\n",oData.nNumImg);
  printf("\n The Feature is switched %s\n", (on ? "on" : "off"));
}

void avt_en_dsnu() {
  unsigned int retval;
  unsigned int type;

          
  printf("\nEnter the value (0-Blemish ,1-DSNU):");
  scanf("%d",&type);      
     
  retval = avt1394_enable_dsnu_blemish(g_handle, *g_pNode,
				       (dc1394bool_t)type,
				       DC1394_TRUE);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in enabling dsnu/blemish ");
  else {
    printf("\nFailure in enabling dsnu/blemish");
    exit(1);
  } 
   
  
}

void avt_dis_dsnu() {
  unsigned int retval;
  unsigned int type;

     
  printf("\nEnter the value (0-Blemish ,1-DSNU):");
  scanf("%d",&type);      
    
  retval = avt1394_enable_dsnu_blemish(g_handle, *g_pNode,
				       (dc1394bool_t)type,
				       DC1394_FALSE);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in disabling dsnu/blemish ");
  else {
    printf("\nFailure in disabling dsnu/blemish");
    exit(1);
  } 
    
}

void avt_get_shading() {
  unsigned int retval;
  dc1394bool_t bShow,bBuild,bBusy;
  unsigned int nImg,nMaxImg;
  dc1394bool_t on;
             
    
  retval = avt1394_get_shading_info(g_handle, *g_pNode,
				    &bShow, &bBuild, &bBusy, &nImg,
				    &nMaxImg, &on);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in getting shading info ");
  else {
    printf("\nFailure in getting shading info");
    exit(1);
  } 
  printf("\n bShow is %d",bShow);
  printf("\n bBuild is %d",bBuild);
  printf("\n bBusy is %d",bBusy);
  printf("\n nMaxImage is %d",nMaxImg);
  printf("\n Image number is %d",nImg);
  printf("\n Feature is %s\n", on ? "enabled" : "disabled");
}

void avt_en_shading() {
  unsigned int retval;
            
  retval = avt1394_enable_shading(g_handle, *g_pNode, DC1394_TRUE);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in enabling shading ");
  else {
    printf("\nFailure in enabling shading");
    exit(1);
  }    
}

void avt_enable_seq() {
  unsigned int retval;
  unsigned int nEnable, nAutoRewind, nIncImageNo;
  unsigned int nSeqLength, nImageNo;
  dc1394bool_t tmpAutoRewind;
  unsigned int tmpSeqLength;

            
  printf("\nEnter the enabling bit (0/1):");
  scanf("%d",&nEnable);
  printf("\nEnter the value of Auto Rewind (0/1):");
  scanf("%d",&nAutoRewind);
  printf("\nEnter the value of seq length:");
  scanf("%d",&nSeqLength);
    
  retval = avt1394_enable_sequence(g_handle,*g_pNode,	
				   (dc1394bool_t)nEnable);

  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in enabling sequence");
  else {
    printf("\nFailure in enabling sequence");
    exit(1);
  }    

  retval = avt1394_get_seq_param(g_handle, *g_pNode,
				 &tmpAutoRewind,
				 &tmpSeqLength,
				 (dc1394bool_t *)&nIncImageNo,
				 &nImageNo);

  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in enabling sequence");
  else {
    printf("\nFailure in enabling sequence");
    exit(1);
  }    

  retval = avt1394_set_seq_param(g_handle, *g_pNode,
				 (dc1394bool_t)nAutoRewind,
				 nSeqLength,
				 (dc1394bool_t)nIncImageNo,
				 nImageNo);

  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in enabling sequence");
  else {
    printf("\nFailure in enabling sequence");
    exit(1);
  }    
}

void avt_load_shading() {
  unsigned int retval,nCount=0,i;
  unsigned int nSize;
  unsigned char pData[1310800];
          
  while(nCount<1310800) {
    for(i=255;i>0;i--) {
      pData[nCount] = i;
      nCount++;
      if(nCount > 1310799)
	break;	    
    }
  }
  printf("\nEnter the size:");
  scanf("%d",&nSize);
  retval = avt1394_load_shading(g_handle,*g_pNode,pData,nSize);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in loading shading ");
  else {
    printf("\nFailure in loading shading");
    exit(1);
  } 
}

void avt_get_seq_info() {
  unsigned int retval;
  unsigned int nMaxLength;
  dc1394bool_t on;
  dc1394bool_t bApply;
     
    
  retval = avt1394_get_seq_info(g_handle,*g_pNode,	
				&nMaxLength, &bApply, &on);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in getting sequence info");
  else {
    printf("\nFailure in getting sequence info");
    exit(1);
  }    
  printf("\nMaximum length is %d",nMaxLength);
  printf("\nFeature is %s\n", (on ? "enabled" : "disabled"));

}

void avt_set_seq_param() {
  unsigned int retval;
  unsigned int nApply, nIncImage;
  unsigned int nImage;
  unsigned int nSeqLength;
  dc1394bool_t bAutoRewind;
  unsigned int tmpImageNo;
  dc1394bool_t tmpIncImage;

            
  printf("\nEnter the Apply Parameter bit (0/1):");
  scanf("%d",&nApply);
  printf("\nEnter the value of Inc Image (0/1):");
  scanf("%d",&nIncImage);
  printf("\nEnter the value of Image Numbe:");
  scanf("%d",&nImage);
  
  retval = avt1394_get_seq_param(g_handle, *g_pNode,
				 &bAutoRewind,
				 &nSeqLength,
				 &tmpIncImage,
				 &tmpImageNo);
				 
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in setting sequence param");
  else {
    printf("\nFailure in setting sequence param");
    exit(1);
  }    

  retval = avt1394_set_seq_param(g_handle, *g_pNode,
				 bAutoRewind,
				 nSeqLength,
				 (dc1394bool_t)nIncImage,
				 nImage);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in setting sequence param");
  else {
    printf("\nFailure in setting sequence param");
    exit(1);
  }

  if(nApply == 1) {
    retval = avt1394_apply_seq_param(g_handle, *g_pNode, DC1394_FALSE);
    
    if(retval == DC1394_SUCCESS)
      printf("\nSuccess in setting sequence param");
    else {
      printf("\nFailure in setting sequence param");
      exit(1);
    }
  }
}

void avt_get_decoder() {
  unsigned int retval;
  unsigned int nCounter;
  unsigned int nCmp;
  dc1394bool_t on;
     
    
  retval = avt1394_get_io_decoder(g_handle,*g_pNode,	
				  &nCounter, &nCmp, &on);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in getting counter info");
  else {
    printf("\nFailure in getting counter info");
    exit(1);
  }    
  printf("\ncounter info is %d",nCounter);
  printf("\ncompare-value is %d", nCmp);
  printf("\nfeature is %s\n", (on ? "enabled" : "disabled"));
}

void avt_set_decoder() {
  unsigned int retval;
  unsigned int nCmp;
            
     
  printf("\nEnter the value of compare:");
  scanf("%d",&nCmp);
    
  retval = avt1394_set_io_decoder(g_handle,*g_pNode,	
				  nCmp);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in setting io decoder");
  else {
    printf("\nFailure in setting io decoder");
    exit(1);
  }    
}

void avt_enable_decoder() {
  unsigned int retval;
  unsigned int  bEnable, bClear;
                
  printf("\nEnter the enabling bit (0/1):");
  scanf("%d",&bEnable);
  printf("\nEnter the value of clear counter (0/1):");
  scanf("%d",&bClear);
  
  
  retval = avt1394_enable_io_decoder(g_handle,*g_pNode,	
				     (dc1394bool_t)bEnable);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in enabling io decoder");
  else {
    printf("\nFailure in enabling io decoder");
    exit(1);
  }    
}

void avt_set_multishot() {
  unsigned int retval,nFrames;
  
  printf("\nEnter the number of frames");
  scanf("%d",&nFrames);
  retval = dc1394_set_multi_shot(g_handle,*g_pNode,nFrames);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in enabling multishot");
  else {
    printf("\nFailure in enabling multishot");
    exit(1);
  }    
}

void avt_start_iso() {
  unsigned int retval;
   
  retval = dc1394_start_iso_transmission(g_handle,*g_pNode);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in starting iso transmission");
  else {
    printf("\nFailure in starting iso transmission");
    exit(1);
  }    

}

void avt_stop_iso() {
  unsigned int retval;
   
  retval = dc1394_stop_iso_transmission(g_handle,*g_pNode);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in stopping iso transmission");
  else {
    printf("\nFailure in stopping iso transmission");
    exit(1);
  }    

}

void avt_get_rcv_size() {
  unsigned int retval;
  unsigned int nRBufSt,nRBufCnt;
            
      
  retval = avt1394_get_recv_serialbuffersize(g_handle,*g_pNode,	
					     &nRBufSt,&nRBufCnt);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in getting serial size info");
  else {
    printf("\nFailure in getting serial size info");
    exit(1);
  }    
  printf("\nValid size of current receive buffer  %d",nRBufSt);
  printf("\nSize of buffer control %d",nRBufCnt);
}

void avt_set_serial_recvbuffer() {
  unsigned int retval,nSize;
  printf("\nEnter the data size of recv buffer for RS232 feature");
  scanf("%d",&nSize);
  retval = avt1394_set_recv_serialbuffersize(g_handle,*g_pNode,nSize);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in setting serial receive buffer");
  else {
    printf("\nFailure in setting serial receive buffer");
    exit(1);
  }    
}

void avt_get_transmit_size() {
  unsigned int retval;
  unsigned int nTBufSt,nTBufCnt;
            
  printf("\nInfo retrieving");
  retval = avt1394_get_tx_serialbuffersize(g_handle,*g_pNode,	
					   &nTBufSt,&nTBufCnt);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in getting transmit serial size info");
  else {
    printf("\nFailure in getting transmit serial size info");
    exit(1);
  }    
  printf("\nValid size of current Transmit buffer  %d",nTBufSt);
  printf("\nSize of buffer control %d",nTBufCnt);
}

void avt_set_serial_txbuffer() {
  unsigned int retval,nSize;
  printf("\nEnter the data size of transmit buffer for RS232 feature");
  scanf("%d",&nSize);
  retval = avt1394_set_tx_serialbuffersize(g_handle,*g_pNode,nSize);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in setting serial transmit buffer");
  else {
    printf("\nFailure in setting serial transmit buffer");
    exit(1);
  }    
}

void avt_get_serial_data() {
    
  unsigned int retval;
  avt_serial_data_t oData;
           
  retval = avt1394_get_serial_data(g_handle,*g_pNode,	
				  &oData);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in getting serial data");
  else {
    printf("\nFailure in getting serial data");
    exit(1);
  }  
    
  printf("\n Character is :%d %c",oData.nChar0, oData.nChar0);
  printf("\n Character is :%d %c",oData.nChar1, oData.nChar1);
  printf("\n Character is :%d %c",oData.nChar2, oData.nChar2);
  printf("\n Character is :%d %c",oData.nChar3, oData.nChar3);
}

void avt_send_serial_data() {
    
  unsigned int retval;
  avt_serial_data_t oData;

  printf("\nEnter the character: ");
  scanf("%d",&oData.nChar0);
  printf("\nEnter the character:");
  scanf("%d",&oData.nChar1);
  printf("\nEnter the character:");
  scanf("%d",&oData.nChar2);
  printf("\nEnter the character:");
  scanf("%d",&oData.nChar3);

/*
  oData.nChar0 = 'a';
  oData.nChar1 = 's';
  oData.nChar2 = 'd';
  oData.nChar3 = 'f';
*/

  retval = avt1394_send_serial_data(g_handle,*g_pNode,	
				    oData);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in sending serial data");
  else {
    printf("\nFailure in sending serial data");
    exit(1);
  }        
}

void avt_show_baud_rate(unsigned int nBaudRate) {
  switch(nBaudRate) {
  case AVT_BAUD_300:
    printf("\nBaud rate is 300 bps");
    break;
  case AVT_BAUD_600:
    printf("\nBaud rate is 600 bps");
    break;
  case AVT_BAUD_1200:
    printf("\nBaud rate is 1200 bps");
    break;
  case AVT_BAUD_2400:
    printf("\nBaud rate is 2400 bps");
    break;
  case AVT_BAUD_4800:
    printf("\nBaud rate is 4800 bps");
    break;
  case AVT_BAUD_9600:
    printf("\nBaud rate is 9600 bps");
    break;
  case AVT_BAUD_19200:
    printf("\nBaud rate is 19200 bps");
    break;
  case AVT_BAUD_38400:
    printf("\nBaud rate is 38400 bps");
    break;
  case AVT_BAUD_57600:
    printf("\nBaud rate is 57600 bps");
    break;
  case AVT_BAUD_115200:
    printf("\nBaud rate is 115200 bps");
    break;
  case AVT_BAUD_230400:
    printf("\nBaud rate is 230400 bps");
    break;    
  default:
    printf("\nError...Invalid Baud rate");
    break;
  }
}

void avt_show_char_len(unsigned int nCharLen) {
  switch(nCharLen) {
  case AVT_7_CHAR:
    printf("\nCharacter length is 7 bits");
    break;	    
  case AVT_8_CHAR:
    printf("\nCharacter length is 8 bits");
    break;
  default:
    printf("\nError...Invalid character length");
    break;
  }
}

void avt_show_parity(unsigned int nParity) {
  switch(nParity) {
  case AVT_NO_PARITY:
    printf("\nZero Parity");
    break;	    
  case AVT_ODD_PARITY:
    printf("\nOdd Parity");
    break;
  case AVT_EVEN_PARITY:
    printf("\nEven Parity");
    break;
  default:
    printf("\nError...Invalid character length");
    break;
  }
}

void avt_show_stopbit(unsigned int nStop) {
  switch(nStop) {
  case AVT_ONE_STOPBIT:
    printf("\nONE STOP BIT");
    break;	    
  case AVT_ONEHALF_STOPBIT:
    printf("\nOne half stop bit");
    break;
  case AVT_TWO_STOPBIT:
    printf("\nTwo Stopbit");
    break;
  default:
    printf("\nError...Invalid character length");
    break;
  }
}

void avt_get_serial_mode() {
    
  unsigned int retval;
  avt_serial_mode_t oData;
            
      
  retval = avt1394_get_serial_mode(g_handle,*g_pNode,	
				   &oData);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in getting serial mode info");
  else {
    printf("\nFailure in getting serial mode info");
    exit(1);
  }    
  avt_show_baud_rate(oData.nBaudRate);
  avt_show_char_len(oData.nCharLen);
  avt_show_parity(oData.nParity);
  avt_show_stopbit(oData.nStopBit);
  printf("\nBuffer size is %d",oData.nBufferSize);

}

void avt_set_serial_mode() {
  unsigned int retval;
  avt_serial_mode_t oData;
            
  printf("\nEnter the value of Baud rate:");
  scanf("%d",&oData.nBaudRate);
  printf("\nEnter the value of Character Length:");
  scanf("%d",&oData.nCharLen);
  printf("\nEnter the value of Parity");
  scanf("%d",&oData.nParity);
  printf("\nEnter the value of stopbit:");
  scanf("%d",&oData.nStopBit);
  retval = avt1394_set_serial_mode(g_handle,*g_pNode,	
				   oData);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in setting serial modeinfo");
  else {
    printf("\nFailure in setting serial mode info");
    exit(1);
  }   
}

void avt_get_serial_ctrl() {
    
  unsigned int retval;
  avt_serial_control_t oData;
            
      
  retval = avt1394_get_serial_control(g_handle,*g_pNode,	
				      &oData);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in getting serial control info");
  else {
    printf("\nFailure in getting serial control info");
    exit(1);
  }    
  if(oData.bRcvEnable == DC1394_TRUE)
    printf("\nReceive Enable is True");
  else
    printf("\nReceive Enable is False");
  if(oData.bTxEnable == DC1394_TRUE)
    printf("\nTransmit Enable is True");
  else
    printf("\nTransmit Enable is False");
  if(oData.bTxBufferReady == DC1394_TRUE)
    printf("\nTransmit data buffer is ready");
  else
    printf("\nTransmit data buffer is not ready");
  if(oData.bRcvBufferReady == DC1394_TRUE)
    printf("\nReceive Buffer is ready");
  else
    printf("\nReceive buffer is not ready");
  if(oData.bRcvOverrunError == DC1394_TRUE)
    printf("\nOverrun error is there");
  else
    printf("\nOverrun error is not there");
  if(oData.bFramingError == DC1394_TRUE)
    printf("\nFraming Error is True");
  else
    printf("\nFraming Error  is False");

  if(oData.bParityError == DC1394_TRUE)
    printf("\nParity Erroris there ");
  else
    printf("\nParity Error is not there");
}

void avt_set_serial_ctrl() {
    
  unsigned int retval;
  avt_serial_control_t oData;
            
  printf("\nEnter the value of Receive Enable(0/1):");
  scanf("%d",(unsigned int *)&oData.bRcvEnable);
  printf("\nEnter the value of Transmit Enable(0/1):");
  scanf("%d",(unsigned int *)&oData.bTxEnable);
  printf("\nEnter the value of Overrun data buffer clearing(0/1):");
  scanf("%d",(unsigned int *)&oData.bRcvOverrunError);
  printf("\nEnter the value of framing error(0/1):");
  scanf("%d",(unsigned int *)&oData.bFramingError);
  printf("\nEnter the value of Parity Error(0/1):");
  scanf("%d",(unsigned int *)&oData.bParityError);
  retval = avt1394_set_serial_control(g_handle,*g_pNode,	
				      oData);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in setting serial control info");
  else {
    printf("\nFailure in setting serial control info");
    exit(1);
  }   
      
}


void avt_build_shading() {
  quadlet_t grabCount;
  unsigned int retval;


  printf("\nEnter the number of images:");
  scanf("%x",&grabCount);

  retval = avt1394_build_shading_image(g_handle, *g_pNode, grabCount);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in building shading image");
  else {
    printf("\nFailure in building shading image");
    exit(1);
  }   
}


void avt_lut_info() {
  unsigned int numLut,nMaxLutSize;
  unsigned int retval;
  dc1394bool_t on;

   
  retval = avt1394_get_lut_info(g_handle, *g_pNode,
				&numLut, &nMaxLutSize, &on);
  if(retval == DC1394_SUCCESS) {
    printf("\nnumber of lut are %d",numLut);
    printf("\nMax lut size is :%d",nMaxLutSize);
    printf("\nFeature is %s\n", on ? "enabled" : "disabled");
  }
  else {
    printf("\nFailure in retrieving LUT info");
    exit(1);
  }    
}

void avt_gamma_on() {
     
  unsigned int retval;
   
  retval = dc1394_feature_on_off(g_handle,*g_pNode,	
				 FEATURE_GAMMA,1);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in setting gamma on");
  else {
    printf("\nFailure in setting gamma on");
    exit(1);
  }    
    
}

void avt_gamma_off() {
  unsigned int retval;
   
  retval = dc1394_feature_on_off(g_handle,*g_pNode,	
				 FEATURE_GAMMA,0);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in setting gamma off");
  else {
    printf("\nFailure in setting gamma off");
    exit(1);
  }    
}

void avt_set_knee() {
  unsigned int retval,nActive;
  unsigned int  dKnee1, dKnee2, dKnee3;
   
  printf("\nEnter the number of  kneepoints:");
  scanf("%d",&nActive);
  switch(nActive) {
  case 1:
    printf("\nEnter the value for knee1:");
    scanf("%d",&dKnee1);
    break;
  case 2:
    printf("\nEnter the value for knee1:");
    scanf("%d",&dKnee1);
    printf("\nEnter the value for knee2:");
    scanf("%d",&dKnee2);
    break;
  case 3:
    printf("Only 3 kneepoints are supported\n");
    nActive = 3;
    printf("\nEnter the value for knee1:");
    scanf("%d",&dKnee1);
    printf("\nEnter the value for knee2:");
    scanf("%d",&dKnee2);
    printf("\nEnter the value for knee3:");
    scanf("%d",&dKnee3);
    break;
  }
  retval = avt1394_set_knee_values(g_handle,*g_pNode,
				   nActive,
				   dKnee1,dKnee2,
				   dKnee3);

  if(retval == DC1394_SUCCESS)
    printf("\nhdr values set successfully\n");
  else {
    printf("\nFailure in setting hdr values\n");
    exit(1);
  } 
}

void avt_set_knee_percent() {
  unsigned int retval,nActive;
  unsigned int dKnee1, dKnee2, dKnee3;
   
  printf("\nEnter the number of  kneepoints:");
  scanf("%d",&nActive);
  switch(nActive) {
  case 1:
    printf("\nEnter the value (%%) for knee1:");
    scanf("%d",&dKnee1);
    break;
  case 2:
    printf("\nEnter the value (%%) for knee1:");
    scanf("%d",&dKnee1);
    printf("\nEnter the value (%%) for knee2:");
    scanf("%d",&dKnee2);
    break;
  case 3:
    printf("\nEnter the value (%%) for knee1:");
    scanf("%d",&dKnee1);
    printf("\nEnter the value (%%) for knee2:");
    scanf("%d",&dKnee2);
    printf("\nEnter the value (%%) for knee3:");
    scanf("%d",&dKnee3);
    break;
  default:
    printf("Only 3 kneepoints are supported\n");
    nActive = 3;
    printf("\nEnter the value (%%) for knee1:");
    scanf("%d",&dKnee1);
    printf("\nEnter the value (%%) for knee2:");
    scanf("%d",&dKnee2);
    printf("\nEnter the value (%%) for knee3:");
    scanf("%d",&dKnee3);
    break;
  }

  retval = avt1394_set_knee_values_percent(g_handle,*g_pNode,
					   nActive,
					   dKnee1,dKnee2,
					   dKnee3);

  if(retval == DC1394_SUCCESS)
    printf("\nhdr values set successfully\n");
  else {
    printf("\nFailure in setting hdr values\n");
    exit(1);
  } 
}

void avt_set_dsnu_blemish() {
  unsigned int retval;
  avt_dsnu_blemish_t oData;
     
     
  printf("\nEnter the type (0-Blemish ,1-DSNU):");
  scanf("%d",(unsigned int *)&oData.bType);
  printf("\nShow correction Image (0-no, 1-yes)?");
  scanf("%d",(unsigned int *)&oData.bShowImg);
  printf("\nCompute new correction image (0-no ,1-yes)?");
  scanf("%d",(unsigned int *)&oData.bCompute);
  printf("\nLoad factury correction data (0-no ,1-yes)?");
  scanf("%d",(unsigned int *)&oData.bLoadData);
  printf("\n'Zero' correction data (0-no ,1-yes)?");
  scanf("%d",(unsigned int *)&oData.bZero);
  printf("\nEnter the number of images (1 < number < 255; 0 - don't change number):");
  scanf("%d",&oData.nNumImg);
    
  retval = avt1394_set_dsnu_blemish(g_handle,*g_pNode,oData);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in setting dsnu/blemish\n");
  else {
    printf("\nFailure in setting dsnu/blemish\n");
    exit(1);
  } 

}

void avt_set_auto_aoi_dim() {
  unsigned int retval, nLeft, nTop, nWidth, nHeight;

  printf("\nleft: ");
  scanf("%d", &nLeft);
  printf("\ntop: ");
  scanf("%d", &nTop);
  printf("\nwidth: ");
  scanf("%d", &nWidth);
  printf("\nheight: ");
  scanf("%d", &nHeight);

  retval = avt1394_set_auto_aoi_dimensions(g_handle, *g_pNode,
					   nLeft, nTop, nWidth, nHeight);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in setting new auto-aoi dimensions\n");
  else {
    printf("\nFailed to set new auto-aoi dimensions\n");
    exit(1);
  }

}

void avt_enable_aoi_work_area() {
  unsigned int retval;

  retval = avt1394_enable_auto_aoi_area(g_handle, *g_pNode, DC1394_TRUE);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in enabling Auto AOI work area\n");
  else {
    printf("\nFailed to enable Auto AOI work area\n");
    exit(1);
  }
}


void avt_disable_aoi_work_area() {
  unsigned int retval;


  retval = avt1394_enable_auto_aoi_area(g_handle, *g_pNode, DC1394_FALSE);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in disabling Auto AOI work area\n");
  else {
    printf("\nFailed to disable Auto AOI work area\n");
    exit(1);
  }
}


void avt_set_adv_trigger() {
  int retval;
  unsigned int nTrigger;


  printf("\nEnter the new value for the adv. Trigger Delay:");
  scanf("%d",&nTrigger);

  retval = avt1394_set_adv_trigger_delay(g_handle, *g_pNode,nTrigger);
  if(retval == DC1394_SUCCESS)
    printf("\nSuccess in setting new adv. trigger delay\n");
  else {
    printf("\nFailure in setting new adv. trigger delay\n");
    exit(1);
  }    
}


void avt_set_int_delay() {
  unsigned int nIntDelay;


  printf("\nEnter the new value for the Integration-Delay:");
  scanf("%d",&nIntDelay);

  if(avt1394_set_int_delay(g_handle, *g_pNode, nIntDelay) == DC1394_SUCCESS)
    printf("\nSuccess in setting new int. delay\n");
  else {
    printf("\nFailure in setting new int. delay\n");
    exit(1);
  }    
}


void avt_select_lut() {
  unsigned int numLut;


  printf("\nEnter the Number of the LUT:");
  scanf("%d", &numLut);

  if(avt1394_select_lut(g_handle, *g_pNode, numLut) == DC1394_SUCCESS)
    printf("\nSuccess in LUT selection\n");
  else {
    printf("\nFailure in LUT selection\n");
    exit(1);
  }
}


void avt_get_lut_selection() {
  unsigned int numLut;


  if(avt1394_get_lut_selection(g_handle, *g_pNode, &numLut) == DC1394_SUCCESS)
    printf("\nCurrent LUT selection: %d\n", numLut);
  else {
    printf("\nFailure in LUT selection\n");
    exit(1);
  }
}


int main(int argc,char *argv[]) {
  int nChoice;

  Initialize();
  printf("\n**************************************");
  printf("\n\tMainMenu ");
  printf("\n**************************************\n\n");
  printf("\n 0. Exit without any changes");
  printf("\n 1. Microprocessor Version Inquiry");
  printf("\n 2. Firmware Version Inquiry");
  printf("\n 3. Timebase Inquiry");
  printf("\n 4. Setting Time base");
  printf("\n 5. Retrieval of Maximum Resolution");
  printf("\n 6. Exposure Time retrieval");
  printf("\n 7. Setting of exposure time");
  printf("\n 8. Integration Delay Retrieval");
  printf("\n 9. Enabling  of Integration Delay");
  printf("\n 10.Disabling of Integration Delay");
  printf("\n 11.Setting Limits of Auto Shutter");
  printf("\n 12.Getting Limits of Auto Shutter");
  printf("\n 13.Setting Limits of Auto Gain");
  printf("\n 14.Getting Limits of Auto Gain");
  printf("\n 15.Enable color correction");
  printf("\n 16.Disable color correction");
  printf("\n 17.Enable mirror image");
  printf("\n 18.Disable mirror image");
  printf("\n 19. Enable Advanced Trigger Delay");
  printf("\n 20. Disable Advanced Trigger Delay");
  printf("\n 21. Get Advanced Trigger delay");
  printf("\n 22. Get Frame Counter");
  printf("\n 23. Reset Frame counter");
  printf("\n 24. Retrieval of Test Image");
  printf("\n 25. Setting of Test Image");
  printf("\n 26. Retrieval of hdr info");
  printf("\n 27. Enable hdr mode");
  printf("\n 28. Disable hdr mode");
  printf("\n 29. Setting IO Control");
  printf("\n 30. Retrieving IO Control info");
  printf("\n 31. Set Deferred Image transport");
  printf("\n 32. Get Deferred Image transport");
  printf("\n 33. Retrieval of GPData buffer");
  printf("\n 34. Get Auto AOI Dimensions");
  printf("\n 35. Enable Auto AOI");
  printf("\n 36. Disable Auto AOI");
  printf("\n 37. Loading of Lookup table");
  printf("\n 38. Enabling of Lookup table");
  printf("\n 39. Disabling of lookup table");
  printf("\n 40. Retrieval of DSNU blemish");
  printf("\n 41. Enabling of DSNU blemish");
  printf("\n 42. Disabling of DSNU blemish");
  printf("\n 43. Retrieval of shading info");
  printf("\n 44. Enable or disable Shading");
  printf("\n 45. Loading of Shading Image");
  printf("\n 46. Enable or disabling of Sequence control");
  printf("\n 47. Setting of seq Param");
  printf("\n 48. Get Seq info");
  printf("\n 49. Get IO Decoder");
  printf("\n 50. set io decoder");
  printf("\n 51. enable or disable IO Decoder");
  printf("\n 52. Set multi shot");
  printf("\n 53. Start iso transmission");
  printf("\n 54. Stop iso transmission");
  printf("\n 55. Receive Buffer Status Control");
  printf("\n 56. Setting Receiving Buffer size");
  printf("\n 57. Retrieving Serial Data");
  printf("\n 58. Sending of Serial Data");
  printf("\n 59. Retrieval of serial control info");
  printf("\n 60. Setting of serial control info");
  printf("\n 61. Transmit Buffer Status Control Information");
  printf("\n 62. Setting Transmit Buffer size");
  printf("\n 63. Getting  Serial mode information");
  printf("\n 64. Setting serial mode information");
  printf("\n 65. Building shading image");
  printf("\n 66. Lookup table info");
  printf("\n 67. Gamma Correction on");
  printf("\n 68  Gamma Correction off");
  printf("\n 69. Set knee values");
  printf("\n 70. Set knee values (%%)");
  printf("\n 71. Set DSNU / Blemish values");
  printf("\n 72. Set Auto AOI Dimensions");
  printf("\n 73. Enable Auto AOI Work Area");
  printf("\n 74. Disable Auto AOI Work Area");
  printf("\n 75. Set Advanced Trigger Delay");
  printf("\n 76. Setting  of Integration Delay");
  printf("\n 77. Selection of Lookup Table");
  printf("\n 78. Current LookupTable selection");
  printf("\n\nEnter your choice : ");

  scanf("%d",&nChoice);

  switch(nChoice) {
  case 0:
    break;
  case 1 :
    microprocessor_inquiry();
    break;       
  case 2:
    firmware_inquiry();
    break;
  case 3 :
    timebase_inq();
    break;
  case 4: 
    set_time();
    break;
  case 5:
    max_res();
    break;
  case 6:
    avt_get_exp_time();
    break;
  case 7:
    avt_set_exp_time();
    break;
  case 8:
    avt_get_int_delay();
    break;
  case 9:
    avt_en_int_delay();
    break;
  case 10:
    avt_dis_int_delay();
    break;
  case 11:
    avt_set_shutter();
    break;
  case 12:
    avt_get_shutter();
    break;
  case 13:
    avt_set_gain();
    break;
  case 14:
    avt_get_gain();
    break;
  case 15:
    avt_en_color();
    break;
  case 16:
    avt_dis_color();
    break;
  case 17:
    avt_en_mirror();
    break;
  case 18:
    avt_dis_mirror();
    break;
  case 19:
    avt_en_adv_trigger();
    break;
  case 20:
    avt_dis_adv_trigger();
    break;
  case 21:
    avt_get_adv_trigger();
    break;
  case 22:
    avt_get_frame();
    break;
  case 23:
    avt_reset_frame();
    break;
  case 24:
    avt_get_test();
    break;
  case 25:
    avt_set_test();
    break;
  case 26:
    avt_get_hdr();
    break;
  case 27:
    avt_en_hdr();
    break;
  case 28:
    avt_dis_hdr();
    break;
  case 29:
    avt_set_io();
    break;
  case 30:
    avt_get_io();
    break;
  case 31:
    avt_set_deferr();
    break;
  case 32:
    avt_get_deferr();
    break;
  case 33:
    avt_get_gpdata();
    break;  
  case 34:
    avt_get_auto_aoi_dim();
    break;
  case 35:
    avt_en_auto_aoi();
    break;
  case 36:
    avt_dis_auto_aoi();
    break;  
  case 37:
    avt_load_lut();
    break;
  case 38:
    avt_en_lut();
    break;
  case 39:
    avt_dis_lut();
    break;
  case 40:
    avt_get_dsnu();
    break;
  case 41:
    avt_en_dsnu();
    break;
  case 42:
    avt_dis_dsnu();
    break;
  case 43:
    avt_get_shading();
    break;    
  case 44:
    avt_en_shading();
    break;
  case 45:
    avt_load_shading();
    break;  
  case 46:
    avt_enable_seq();
    break;	
  case 47:
    avt_set_seq_param();
    break;
  case 48:
    avt_get_seq_info();
    break;
  case 49:
    avt_get_decoder();
    break;  
  case 50:
    avt_set_decoder();
    break;	
  case 51:
    avt_enable_decoder();
    break;
  case 52:
    avt_set_multishot();
    break;
  case 53:
    avt_start_iso();
    break;
  case 54:
    avt_stop_iso();
    break;
  case 55:
    avt_get_rcv_size();
    break;
  case 56:
    avt_set_serial_recvbuffer();
    break;
  case 57:
    avt_get_serial_data();
    break;
  case 58:
    avt_send_serial_data();
    break;
  case 59:
    avt_get_serial_ctrl();
    break;
  case 60:
    avt_set_serial_ctrl();
    break;
  case 61:
    avt_get_transmit_size();
    break;
  case 62:
    avt_set_serial_txbuffer();
    break;
  case 63:
    avt_get_serial_mode();
    break;
  case 64:
    avt_set_serial_mode();
    break;
  case 65:
    avt_build_shading();
    break;
  case 66:
    avt_lut_info();
    break;
  case 67:
    avt_gamma_on();
    break;
  case 68:
    avt_gamma_off();
    break;
  case 69:
    avt_set_knee();
    break;
  case 70:
    avt_set_knee_percent();
    break;
  case 71:
    avt_set_dsnu_blemish();
    break;
  case 72:
    avt_set_auto_aoi_dim();
    break;
  case 73:
    avt_enable_aoi_work_area();
    break;
  case 74:
    avt_disable_aoi_work_area();
    break;
  case 75:
    avt_set_adv_trigger();
    break;
  case 76:
    avt_set_int_delay();
    break;
  case 77:
    avt_select_lut();
    break;
  case 78:
    avt_get_lut_selection();
    break;
  default: 
    printf("\n Invalid Choice... Please enter valid choice");
    exit(1);
  }
  printf("\n");

  exit(1);
}
