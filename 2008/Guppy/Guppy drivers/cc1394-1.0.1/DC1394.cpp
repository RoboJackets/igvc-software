/*
  DC1394.cpp
  
  Implementation of the DC1394-class
  
  1.9.04 G.Glock
*/

#include "DC1394.h"


DC1394::DC1394(MessageLogger *ml) {
    logger = ml;
    
    
    for(int i = 0; i < MAX_CAMERAS; i++) {
        handles[i] = NULL;
        
        isoStarted[i] = false;
        dmaSetUp[i] = false;
        
        format[i] = 0;
        mode[i] = 0;
        framerate[i] = 0;
        colorCodingID[i] = 0;
        width[i] = 0;
        height[i] = 0;
        x[i] = 0;
        y[i] = 0;
        
        f7FPS[i] = 0;
        
        shadingValid[i] = false;
    }
    
    numPorts = MAX_PORTS;
    camCount = 0;
}


DC1394::~DC1394() {
    // cleanup 1394-port
}


// static
char *DC1394::featureString(unsigned int featureIndex) {
    // Array of 'Feature-Strings'
    char *featureStrings[NUM_FEATURES] =
    {
        "Brightness", // 416
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
        "Trigger Delay",
        "White Shading",
        "Framerate",
        "Zoom",
        "Pan",
        "Tilt",
        "Optical Filter",
        "Capture Size",
        "Capture Quality"
    };
    
    return featureStrings[featureIndex];
}


int DC1394::initRaw1394() {
    struct raw1394_portinfo ports[MAX_PORTS];
    raw1394handle_t rawHandle;
    
    // initialize and open DC1394 connection
    if((rawHandle = raw1394_new_handle()) == NULL) {
        logger->log("'DC1394::initRaw1394': Error! Unable to acquire a 'raw1394' handle!");
        return 0;
    }
    
    // get available ports
    if((numPorts = raw1394_get_port_info(rawHandle, ports, MAX_PORTS)) <= 0) {
        logger->log("'DC1394::initRaw1394': No ports available!");
        raw1394_destroy_handle(rawHandle);
        return 0;
    }
    
    logger->log("'DC1394::initRaw1394': %d ports found.", numPorts);
    
    for(unsigned int i = 0; i < numPorts; i++) {
        logger->log("'DC1394::initRaw1394': Port-No %d (%s) has %d nodes",
                    i, ports[i].name, ports[i].nodes);
    }
    
    if(raw1394_reset_bus_new(rawHandle, RAW1394_LONG_RESET) < 0)
        logger->log("DC1394::initRaw1394': Error while trying to reset the bus(es)");
    else
        logger->log("'DC1394::initRaw1394': Bus(es) successfully reset.");
    
    // cleanup unused handles
    raw1394_destroy_handle(rawHandle);
    
    return numPorts;
}


int DC1394::dc1394Init(QString devName, bool subDirs, unsigned int devNo) {
    raw1394handle_t rawHandle;
    dc1394_miscinfo miscInfo;
    unsigned int portBase;
    int cCount;
    unsigned int colorCoding;
    
    
    nodeid_t *cameraNodes = NULL;
    
    if(!subDirs) {
        // if '/dev/video1394' is the only avilable device - scan all ports
        portBase = 0;
    }
    else {
        // else - scan only the port given as parameter
        portBase = devNo;
        numPorts = devNo + 1;
    }
    
    
    // get dc1394-handles for each port
    for(unsigned int p = portBase; p < numPorts; p++) {
        logger->log("'DC1394::dc1394Init': Setting up Port %d (%s)", p, devName.ascii());
        
        // get the camera nodes and describe them as we find them
        if((rawHandle = raw1394_new_handle()) == NULL) {
            logger->log("'DC1394::dc1394Init': Error while trying to get a new raw1394-handle!");
            return 0;
        }
        if((raw1394_set_port(rawHandle, p)) == -1) {
            logger->log("'DC1394::dc1394Init': Error while trying to set raw1394-port (%d)!", p);
            raw1394_destroy_handle(rawHandle);
            return 0;
        }
        // get camera nodes but don't show information about the camera,
        // because a crash may occur
        if((cameraNodes = dc1394_get_camera_nodes(rawHandle, &cCount, 0)) == NULL) {
            logger->log("'DC1394::dc1394Init': Error while trying to read camera-nodes!");
            raw1394_destroy_handle(rawHandle);
            return 0;
        }
        logger->log("'DC1394::dc1394Init': %d camera node(s) found on port %d (%s)",
                    cCount, p, devName.ascii());
        
        // raw1394 handle no longer needed
        raw1394_destroy_handle(rawHandle);
        
        // if connected cameras are available
        for(int i = 0; i < cCount; i++) {
            logger->log("'DC1394::dc1394Init': Setting DC1394 parameters for camera %d on port %d (%s).",
                        camCount, p, devName.ascii());
            // create a dc1394-handle for the current port
            if((handles[camCount] = dc1394_create_handle(p)) == NULL) {
                logger->log("'DC1394::dc1394Init': Unable to create a raw1394 handle for port %d", p);
                dc1394_free_camera_nodes(cameraNodes);
                return 0;
            }
            
            camPort[camCount] = p;
            camNode[camCount] = i;
            
            // copy the camera nodes respective to the ieee1394 ports into the
            // 'cameras' structure array
            cameras[camCount].node = cameraNodes[i];
            
            if(dc1394_get_camera_info(handles[camCount],
                                      cameras[camCount].node,
                                      &camInfos[camCount]) != DC1394_SUCCESS) {
                logger->log("'DC1394::dc1394Init': Error while trying to read the camerainfo of camera %d!",
                            camCount);
                dc1394_free_camera_nodes(cameraNodes);
                return 0;
            }
            
            if(dc1394_get_camera_feature_set(handles[camCount],
                                             cameras[camCount].node,
                                             &featureSets[camCount]) != DC1394_SUCCESS) {
                logger->log("'DC1394::dc1394Init': Error while trying to read the featureset of camera %d!",
                            camCount);
                dc1394_free_camera_nodes(cameraNodes);
                return 0;
            }
            
            // supported Format 7 Color-Codings
            for(unsigned int f7Mode = MODE_FORMAT7_MIN; f7Mode <= MODE_FORMAT7_MAX;
                f7Mode++) {
                if(dc1394_query_format7_color_coding(handles[camCount],
                                                     cameras[camCount].node,
                                                     f7Mode,
                                                     &colorCoding) != DC1394_SUCCESS) {
                    logger->log("'DC1394::dc1394Init': Error while trying to read Color-Codings for mode %d!",
                                f7Mode);
                    dc1394_free_camera_nodes(cameraNodes);
                    return 0;
                }
                colorCodings[f7Mode - MODE_FORMAT7_MIN][camCount] = colorCoding;
                logger->log("'DC1394::dc1394Init': Camera %d - F7 Mode %d - 'ColorCoding': 0x%x",
                            camCount, f7Mode,
                            colorCodings[f7Mode - MODE_FORMAT7_MIN][camCount]);
            }
            
            // get the iso-cannel and -speed for the current camera-port
            // i.e. get mainly the speed of the current camera-port, beacuse the
            // iso-channel will be set individually with the next dc1394-call
            if(dc1394_get_iso_channel_and_speed(handles[camCount],
                                                cameras[camCount].node,
                                                &isoChannel[camCount],
                                                &isoSpeed[camCount])
                != DC1394_SUCCESS) {
                
                logger->log("'DC1394::dc1394Init': Error while trying to read ISO-channel and -speed!");
                dc1394_free_camera_nodes(cameraNodes);
                return 0;
            }
            
            logger->log("'DC1394::dc1394Init': ISO-channel for camera %d: %d\nISO-speed for camera %d: %d",
                        camCount, isoChannel[camCount], camCount, isoSpeed[camCount]);
            logger->log("'DC1394::dc1394Init': Now trying to set ISO-channel %d for camera %d\nand ISO-speed %d for camera %d",
                        i + 1, camCount, isoSpeed[camCount], camCount);
            
            // Set the iso-channel for the current camera-port
            if(dc1394_set_iso_channel_and_speed(handles[camCount],
                                                cameras[camCount].node,
                                                i + 1,
                                                //      camCount +1,
                                                //      isoChannel[camCount],
                                                isoSpeed[camCount])
                != DC1394_SUCCESS) {
                
                logger->log("'DC1394::dc1394Init': Error while trying to set iso-channel and -speed!");
                dc1394_free_camera_nodes(cameraNodes);
                return 0;
            }
            
            // read ISO channel and speed again
            if(dc1394_get_iso_channel_and_speed(handles[camCount],
                                                cameras[camCount].node,
                                                &isoChannel[camCount],
                                                &isoSpeed[camCount])
                != DC1394_SUCCESS) {
                
                logger->log("'DC1394::dc1394Init': Error while trying to read iso-channel and -speed!");
                dc1394_free_camera_nodes(cameraNodes);
                return 0;
            }
            
            logger->log("'DC1394::dc1394Init': ISO-channel for camera %d: %d\nISO-speed for camera %d: %d",
                        camCount, isoChannel[camCount], camCount, isoSpeed[camCount]);
            
            // read the current format, mode and framerate
            if(dc1394_get_camera_misc_info(handles[camCount],
                                           cameras[camCount].node,
                                           &miscInfo)
                != DC1394_SUCCESS) {
                
                logger->log("'DC1394::dc1394Init': Error while trying to read 'misc-camera-infos' for camera %d",
                            camCount);
                return 0;
            }
            
            logger->log("'DC1394::dc1394Init': Camera-Parameters: format = %d, mode = %d, framerate = %d",
                        miscInfo.format, miscInfo.mode, miscInfo.framerate);
            logger->log("'DC1394::dc1394Init': ISO-Parameters: channel = %d (%s), speed = %d\n",
                        miscInfo.iso_channel,
                        (miscInfo.is_iso_on == DC1394_TRUE) ? "on" : "off",
                        miscInfo.iso_speed);
            
            format[camCount] = miscInfo.format;
            mode[camCount] = miscInfo.mode;
            framerate[camCount] = miscInfo.framerate;
            
            deviceName[camCount] = devName.ascii();
            
            camCount++;
        }
        
        dc1394_free_camera_nodes(cameraNodes);
    }
    
    return cCount;
} // dc1394Init()


bool DC1394::dc1394Cleanup() {
    bool success = true;
    
    // destroy all handles
    for(unsigned int i = 0; i < camCount; i++) {
        if(handles[i] != NULL) {
            if(dc1394_destroy_handle(handles[i]) != DC1394_SUCCESS) {
                logger->log("'DC1394::dc1394Cleanup': Error while destroying dc1394 handle!");
                success = false;
            }
        }
    }
    
    return success;
}


bool DC1394::createHandle(unsigned int id) {
    if(handles[id] != NULL)
        return true;
    
    if((handles[id] = dc1394_create_handle(camPort[id])) == NULL)
        return false;
    
    return true;
}


bool DC1394::destroyHandle(unsigned int id) {
    if(handles[id] == NULL)
        return true;
    
    if(dc1394_destroy_handle(handles[id]) != DC1394_SUCCESS)
        return false;
    
    handles[id] = NULL;
    
    return true;
}


raw1394handle_t DC1394::getHandle(unsigned int id) {
    return handles[id];
}


bool DC1394::getCameraID(unsigned int id, unsigned int *camID) {
    avt_firmware_t fw;
    
    if(avt1394_get_firmware_version(handles[id],
                                    cameras[id].node,
                                    &fw) != DC1394_SUCCESS)
        return false;
    
    *camID = fw.nCameraId;
    return true;
}


unsigned int DC1394::getPortNo(unsigned int id) {
    return camPort[id];
}


unsigned int DC1394::getNodeNo(unsigned int id) {
    return camNode[id];
}


quadlet_t DC1394::getSupportedFormats(unsigned int id) {
    quadlet_t formats;
    
    if(dc1394_query_supported_formats(handles[id], cameras[id].node, &formats) != DC1394_SUCCESS) {
        logger->log("'DC1394::getSupportedFormats': Unable to query supported formats!");
        return 0;
    }
    
    return formats;
}


quadlet_t DC1394::getSupportedModes(unsigned int id, unsigned int format) {
    quadlet_t modes;
    
    if(dc1394_query_supported_modes(handles[id], cameras[id].node, format, &modes) != DC1394_SUCCESS) {
        logger->log("'DC1394::getSupportedModes': Unable to query supported modes!");
        return 0;
    }
    
    return modes;
}


quadlet_t DC1394::getSupportedFramerates(unsigned int id, unsigned int format, unsigned int mode) {
    quadlet_t framerates;
    
    if(dc1394_query_supported_framerates(handles[id], cameras[id].node, format, mode, &framerates) != DC1394_SUCCESS) {
        logger->log("'DC1394::getSupportedFramerates': Unable to query supported framerates!");
        return 0;
    }
    
    return framerates;
}


bool DC1394::setParameters(unsigned int id, unsigned int f, unsigned int m, unsigned int fr,
                           unsigned int cc) {
    // DMA capture-setup for the respective camera
    if(dc1394_set_video_format(handles[id],
                               cameras[id].node,
                               f) != DC1394_SUCCESS) {
        logger->log("'DC1394::setParameters': Setting video format failed!");
        return false;
    }
    
    format[id] = f;
    
    if(dc1394_set_video_mode(handles[id],
                             cameras[id].node,
                             m) != DC1394_SUCCESS) {
        logger->log("'DC1394::setParameters': Setting video mode failed!");
        return false;
    }
    
    mode[id] = m;
    
    // framerate is only available at modes 0...2
    if(m <= MODE_FORMAT2_MAX) {
        if(dc1394_set_video_framerate(handles[id],
                                      cameras[id].node,
                                      fr) != DC1394_SUCCESS) {
            logger->log("'DC1394::setParameters': Setting framerate (%d) failed!", fr);
            return false;
        }
        
        framerate[id] = fr;
    }
    else {
        // Format 7
        if(m >= MODE_FORMAT7_MIN)  {
            if(dc1394_set_format7_color_coding_id(handles[id],
                                                  cameras[id].node,
                                                  mode[id],
                                                  cc) != DC1394_SUCCESS) {
                logger->log("'DC1394::setParameters': Setting F7 color coding failed!");
                return false;
            }
            
            colorCodingID[id] = cc;
        }
        else {
            // Format 6
            logger->log("'DC1394::setParameters': Format 6 not yet supported!");
            return false;
        }
    }
    
    setWidthAndHeight(id);
    
    return true;
}


unsigned int DC1394::getFormat(unsigned int id) {
    return format[id];
}


unsigned int DC1394::getMode(unsigned int id) {
    return mode[id];
}


unsigned int DC1394::getFramerate(unsigned int id) {
    return framerate[id];
}


unsigned int DC1394::getWidth(unsigned int id) {
    return width[id];
}


unsigned int DC1394::getHeight(unsigned int id) {
    return height[id];
}


unsigned int DC1394::getX(unsigned int id) {
    return x[id];
}


unsigned int DC1394::getY(unsigned int id) {
    return y[id];
}


unsigned int DC1394::getF7FPS(unsigned int id) {
    return f7FPS[id];
}


QString DC1394::getFramerateString(unsigned int id) {
    switch(framerate[id]) {
    case FRAMERATE_1_875:
        return QString::number(1.875);
    case FRAMERATE_3_75:
        return QString::number(3.75);
    case FRAMERATE_7_5:
        return QString::number(7.5);
    case FRAMERATE_15:
        return QString::number(15);
    case FRAMERATE_30:
        return QString::number(30);
    case FRAMERATE_60:
        return QString::number(60);
    default:
        return QString("<unknown>");
    }
}


QString DC1394::getColorCodingIDString(unsigned int id) {
    switch(colorCodingID[id]) {
    case COLOR_FORMAT7_MONO8:
        return QString(F7_CC0);
    case COLOR_FORMAT7_YUV411:
        return QString(F7_CC1);
    case COLOR_FORMAT7_YUV422:
        return QString(F7_CC2);
    case COLOR_FORMAT7_YUV444:
        return QString(F7_CC3);
    case COLOR_FORMAT7_RGB8:
        return QString(F7_CC4);
    case COLOR_FORMAT7_MONO16:
        return QString(F7_CC5);
    case COLOR_FORMAT7_RGB16:
        return QString(F7_CC6);
    default:
        return QString("<unknown>");
    }
}


QString DC1394::getModeString(unsigned int id) {
    switch(mode[id]) {
    case MODE_160x120_YUV444:
        return QString("YUV 4:4:4");
        
    case MODE_320x240_YUV422:
    case MODE_640x480_YUV422:
    case MODE_800x600_YUV422:
    case MODE_1024x768_YUV422:
    case MODE_1280x960_YUV422:
    case MODE_1600x1200_YUV422:
        return QString("YUV 4:2:2");
        
    case MODE_640x480_YUV411:
        return QString("YUV 4:1:1");
        
    case MODE_640x480_RGB:
    case MODE_800x600_RGB:
    case MODE_1024x768_RGB:
    case MODE_1280x960_RGB:
    case MODE_1600x1200_RGB:
        return QString("RGB");
        
    case MODE_640x480_MONO:
    case MODE_800x600_MONO:
    case MODE_1024x768_MONO:
    case MODE_1280x960_MONO:
    case MODE_1600x1200_MONO:
        return QString("Mono 8bit");
        
    case MODE_640x480_MONO16:
    case MODE_800x600_MONO16:
    case MODE_1024x768_MONO16:
    case MODE_1280x960_MONO16:
    case MODE_1600x1200_MONO16:
        return QString("Mono 16bit");
        
    case MODE_FORMAT7_0:
    case MODE_FORMAT7_1:
    case MODE_FORMAT7_2:
    case MODE_FORMAT7_3:
    case MODE_FORMAT7_4:
    case MODE_FORMAT7_5:
    case MODE_FORMAT7_6:
    case MODE_FORMAT7_7:
        return getColorCodingIDString(id);
        
    default:
        return QString("");
    }
}


unsigned int DC1394::getIsoChannel(unsigned int id) {
    return isoChannel[id];
}


unsigned int DC1394::getIsoSpeed(unsigned int id) {
    return isoSpeed[id];
}


bool DC1394::setIsoSpeed(unsigned int id, unsigned int speed) {
    unsigned int channel, currentSpeed;
    
    
    if(dc1394_get_iso_channel_and_speed(handles[id],
                                        cameras[id].node,
                                        &channel,
                                        &currentSpeed) != DC1394_SUCCESS)
        return false;
    
    if(dc1394_set_iso_channel_and_speed(handles[id],
                                        cameras[id].node,
                                        channel,
                                        speed) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::dmaSetup(unsigned int id) {
    logger->log("'DC1394::dmaSetup': Setting up DMA for camera %d (port:%s, ISO-Channel: %d)",
                id, deviceName[id].ascii(), isoChannel[id]);
    
    // DMA capture-setup for the respective camera
    if(dc1394_dma_setup_capture(handles[id],
                                cameras[id].node,
                                isoChannel[id], // iso-channel (just set before)
                                format[id],
                                mode[id],
                                isoSpeed[id],
                                framerate[id],
                                NUM_BUFFERS, // number of DMA buffers (?)
                                //    0, // extra buffering (?) - deleted since 'libdc1394_control.so.11.0.0'
                                DROP_FRAMES, // drop frames (?)
                                deviceName[id].ascii(),
                                &cameras[id]) != DC1394_SUCCESS) {
        return false;
    }
    
    dmaSetUp[id] = true;
    return true;
}


bool DC1394::dmaF7Setup(unsigned int id) {
    unsigned int w, h, xPos, yPos, colorID;
    unsigned int pixelNo, minBytes, maxBytes, bytesPerPacket;
    unsigned long long int totalBytes;
    unsigned int presence, setting1, errFlag1, errFlag2;
    unsigned int hUnitSize, vUnitSize, recBPP, ppf;
    unsigned int hUnitPos, vUnitPos;
    unsigned int ccID, byteDepth;
    unsigned long long a, b;
    
    
    logger->log("'DC1394::dmaF7Setup': Setting up F7-DMA for camera %d (port:%s, ISO-Channel: %d)",
                id, deviceName[id].ascii(), isoChannel[id]);
    logger->log("\tUsing Mode %d", mode[id]);
    logger->log("\tColor-Coding-ID: %d", colorID);
    
    if(dc1394_query_format7_color_coding_id(handles[id],
                                            cameras[id].node,
                                            mode[id],
                                            &colorID) != DC1394_SUCCESS) {
        logger->log("'DC1394::dmaF7Setup': Querying Color-Coding-ID failed!");
        return false;
    }
    logger->log("\tSize: %d x %d; Position: %d, %d", width[id], height[id], x[id], y[id]);
    
    if(dc1394_query_format7_image_size(handles[id],
                                       cameras[id].node,
                                       mode[id],
                                       &w, &h) != DC1394_SUCCESS) {
        logger->log("'DC1394::dmaF7Setup': Querying F7 Image Size failed!");
        return false;
    }
    if(dc1394_query_format7_image_position(handles[id],
                                           cameras[id].node,
                                           mode[id],
                                           &xPos, &yPos) != DC1394_SUCCESS) {
        logger->log("'DC1394::dmaF7Setup': Querying F7 Image Position failed!");
        return false;
    }
    logger->log("\tCamera-Size: %d x %d; Camera-Posision: %d, %d", w, h, xPos, yPos);
    
    if(w == 0 || h == 0) {
        logger->log("'DC1394::dmaF7Setup': width and/or height are '0' - trying to correct...");
        if(!setF7ImageSize(id, width[id], height[id])) {
            logger->log("'DC1394::dmaF7Setup': Error while trying to correct size!");
            return false;
        }
        else {
            if(!setF7ImagePos(id, x[id], y[id])) {
                logger->log("'DC1394::dmaF7Setup': Error while trying to correct position!");
                return false;
            }
        }
        logger->log("\t...successfully changed size and position - re-reading...");
        
        if(dc1394_query_format7_image_size(handles[id],
                                           cameras[id].node,
                                           mode[id],
                                           &w, &h) != DC1394_SUCCESS) {
            logger->log("'DC1394::dmaF7Setup': Querying F7 Image Size failed!");
            return false;
        }
        if(dc1394_query_format7_image_position(handles[id],
                                               cameras[id].node,
                                               mode[id],
                                               &xPos, &yPos) != DC1394_SUCCESS) {
            logger->log("'DC1394::dmaF7Setup': Querying F7 Image Position failed!");
            return false;
        }
        logger->log("\tCamera-Size: %d x %d; Camera-Posision: %d, %d", w, h, xPos, yPos);
        
    }
    
    if(dc1394_query_format7_unit_size(handles[id],
                                      cameras[id].node,
                                      mode[id],
                                      &hUnitSize, &vUnitSize) != DC1394_SUCCESS) {
        logger->log("'DC1394::dmaF7Setup': Querying F7 'Unit-Size' failed!");
        return false;
    }
    if(dc1394_query_format7_unit_position(handles[id],
                                          cameras[id].node,
                                          mode[id],
                                          &hUnitPos, &vUnitPos) != DC1394_SUCCESS) {
        logger->log("'DC1394::dmaF7Setup': Querying F7 'Unit Position' failed!");
        return false;
    }
    logger->log("\t'Unit-Size': %d x %d; 'Unit-Position': %d, %d",
                hUnitSize, vUnitSize, hUnitPos, vUnitPos);
    
    if(dc1394_query_format7_pixel_number(handles[id],
                                         cameras[id].node,
                                         mode[id],
                                         &pixelNo) != DC1394_SUCCESS) {
        logger->log("'DC1394::dmaF7Setup': Querying F7 'Pixels-No.' failed!");
        return false;
    }
    if(dc1394_query_format7_total_bytes(handles[id],
                                        cameras[id].node,
                                        mode[id],
                                        &totalBytes) != DC1394_SUCCESS) {
        logger->log("'DC1394::dmaF7Setup': Querying F7 'total bytes' failed!");
        return false;
    }
    logger->log("\t'Pixel-No.': %d (0x%x); 'Total Bytes': %d (0x%x)", pixelNo, pixelNo,
                (unsigned int)totalBytes, (unsigned int)totalBytes);
    
    if(dc1394_query_format7_packet_para(handles[id],
                                        cameras[id].node,
                                        mode[id],
                                        &minBytes, &maxBytes) != DC1394_SUCCESS) {
        logger->log("'DC1394::dmaF7Setup': Querying F7 'packet parameters' failed!");
        return false;
    }
    logger->log("\t'Packet Parameters': %d (min.), %d (max.)", minBytes, maxBytes);
    
    if(dc1394_query_format7_byte_per_packet(handles[id],
                                            cameras[id].node,
                                            mode[id],
                                            &bytesPerPacket) != DC1394_SUCCESS) {
        logger->log("'DC1394::dmaF7Setup': Querying F7 'bytes per packet' failed!");
        return false;
    }
    logger->log("\t'Bytes per Packet': %d", bytesPerPacket);
    
    // DMA capture-setup for the respective camera
    if(dc1394_dma_setup_format7_capture(handles[id],
                                        cameras[id].node,
                                        isoChannel[id],
                                        mode[id],
                                        isoSpeed[id],
                                        (bytesPerPacket == 0) ? USE_MAX_AVAIL : QUERY_FROM_CAMERA,
                                        (w == 0) || (h == 0) ? (unsigned int) QUERY_FROM_CAMERA : xPos,
                                        (w == 0) || (h == 0) ? (unsigned int) QUERY_FROM_CAMERA : yPos,
                                        (w == 0) || (h == 0) ? (unsigned int) QUERY_FROM_CAMERA : w,
                                        (w == 0) || (h == 0) ? (unsigned int) QUERY_FROM_CAMERA : h,
                                        NUM_BUFFERS, // number of DMA buffers
                                        DROP_FRAMES, // drop frames (?)
                                        deviceName[id].ascii(),
                                        &(cameras[id])) != DC1394_SUCCESS) {
        return false;
    }
    
    dmaSetUp[id] = true;
    
    logger->log("\tCamera-Informations after initiating F7-DMA:");
    
    if(dc1394_query_format7_pixel_number(handles[id],
                                         cameras[id].node,
                                         mode[id],
                                         &pixelNo) != DC1394_SUCCESS) {
        logger->log("'DC1394::dmaF7Setup': Querying F7 'Pixels-No.' failed!");
        return false;
    }
    logger->log("\t'Pixel-No.': %d", pixelNo);
    
    if(dc1394_query_format7_total_bytes(handles[id],
                                        cameras[id].node,
                                        mode[id],
                                        &totalBytes) != DC1394_SUCCESS) {
        logger->log("'DC1394::dmaF7Setup': Querying F7 'total bytes' failed!");
        return false;
    }
    logger->log("\t'Total Bytes': %d", (unsigned int)totalBytes);
    
    if(dc1394_query_format7_packet_para(handles[id],
                                        cameras[id].node,
                                        mode[id],
                                        &minBytes, &maxBytes) != DC1394_SUCCESS) {
        logger->log("'DC1394::dmaF7Setup': Querying F7 'packet parameters' failed!");
        return false;
    }
    logger->log("\t'Packet Parameters': %d (min.), %d (max.)", minBytes, maxBytes);
    
    if(dc1394_query_format7_byte_per_packet(handles[id],
                                            cameras[id].node,
                                            mode[id],
                                            &bytesPerPacket) != DC1394_SUCCESS) {
        logger->log("'DC1394::dmaF7Setup': Querying F7 'bytes per packet' failed!");
        return false;
    }
    logger->log("\t'Bytes per Packet': %d", bytesPerPacket);
    
    if(dc1394_query_format7_value_setting(handles[id],
                                          cameras[id].node,
                                          mode[id],
                                          &presence, &setting1,
                                          &errFlag1, &errFlag2) != DC1394_SUCCESS) {
        logger->log("'DC1394::dmaF7Setup': Querying F7 'value setting' failed!");
        return false;
    }
    logger->log("\t'value setting': 0x%x (presence), 0x%x (setting1), 0x%x (errFlag1), 0x%x (errFlag2)",
                presence, setting1, errFlag1, errFlag2);
    
    if(dc1394_query_format7_unit_size(handles[id],
                                      cameras[id].node,
                                      mode[id],
                                      &hUnitSize, &vUnitSize) != DC1394_SUCCESS) {
        logger->log("'DC1394::dmaF7Setup': Querying F7 'Unit-Size' failed!");
        return false;
    }
    logger->log("\t'Unit-Size': %d (hor.) / %d (vert.)", hUnitSize, vUnitSize);
    
    if(dc1394_query_format7_recommended_byte_per_packet(handles[id],
                                                        cameras[id].node,
                                                        mode[id],
                                                        &recBPP) != DC1394_SUCCESS) {
        logger->log("'DC1394::dmaF7Setup': Querying F7 'recommended bytes-per-packet' failed!");
        return false;
    }
    logger->log("\t'Recommended Bytes per Packet': %d", recBPP);
    
    if(dc1394_query_format7_packet_per_frame(handles[id],
                                             cameras[id].node,
                                             mode[id],
                                             &ppf) != DC1394_SUCCESS) {
        logger->log("'DC1394::dmaF7Setup': Querying F7 'packet per frame' failed!");
        return false;
    }
    logger->log("\t'Packet per Frame': %d", ppf);
    
    if(dc1394_query_format7_unit_position(handles[id],
                                          cameras[id].node,
                                          mode[id],
                                          &hUnitPos, &vUnitPos) != DC1394_SUCCESS) {
        logger->log("'DC1394::dmaF7Setup': Querying F7 'Unit Position' failed!");
        return false;
    }
    logger->log("\t'Unit-Position': %d, %d", hUnitPos, vUnitPos);
    
    // framerate-determination
    if(dc1394_query_format7_image_size(handles[id],
                                       cameras[id].node,
                                       mode[id],
                                       &w, &h) != DC1394_SUCCESS) {
        logger->log("'DC1394::dmaF7Setup': Querying F7 Image Size failed!");
        return false;
    }
    if(dc1394_query_format7_byte_per_packet(handles[id],
                                            cameras[id].node,
                                            mode[id],
                                            &bytesPerPacket) != DC1394_SUCCESS) {
        logger->log("'DC1394::dmaF7Setup': Querying F7 'bytes per packet' failed!");
        return false;
    }
    ccID = getF7ColorCodingID(id, mode[id]);
    
    switch(ccID) {
    case COLOR_FORMAT7_MONO8:
        byteDepth = 10;
        break;
    case COLOR_FORMAT7_MONO16:
    case COLOR_FORMAT7_YUV422:
        byteDepth = 20;
        break;
    case COLOR_FORMAT7_YUV411:
        byteDepth = 15;
        break;
    default:
        logger->log("'DC1394::dmaF7Setup': Warning! Current ColorCoding (%d) is not supported...",
                    ccID);
        byteDepth = 10;
        break;
    }
    a = ((unsigned long long)bytesPerPacket * (unsigned long long)10000000);
    b = ((unsigned long long)w * (unsigned long long)h * (unsigned long long)byteDepth *
         (unsigned long long)125);
    f7FPS[id] = (int)(a / b);
    logger->log("\tF7-FPS = %d fps", f7FPS[id]);
    
    return true;
}


bool DC1394::dmaRelease(unsigned int id) {
    bool success = true;
    
    if(dc1394_dma_unlisten(handles[id],
                           &cameras[id]) != DC1394_SUCCESS) {
        logger->log("'DC1394::dmaRelease': 'dc1394_dma_unlisten()' failed");
        success = false;
    }
    
    if(dc1394_dma_release_camera(handles[id],
                                 &cameras[id]) !=
       DC1394_SUCCESS) {
        logger->log("'DC1394::dmaRelease': 'dc1394_dma_release_camera()' failed!");
        success = false;
    }
    
    dmaSetUp[id] = false;
    
    return success;
}


bool DC1394::startIsoTransmission(unsigned int id) {
    dc1394bool_t on;
    unsigned int waitCount = 0;
    
    
    logger->log("DC1394::startIsoTransmission': Starting Iso Transmission for Camera %d (%s)",
                id,
                deviceName[id].ascii());
    
    if(dc1394_get_iso_status(handles[id],
                             cameras[id].node, &on) !=
       DC1394_SUCCESS) {
        logger->log("'DC1394::startIsoTransmission': Error while trying to read ISO status!");
        return false;
    }
    
    if(on == DC1394_TRUE) {
        logger->log("'DC1394::startIsoTransmission': ISO-Transmission already working!");
        isoStarted[id] = true;
        return true;
    }
    
    do{
        if(on == DC1394_FALSE) {
            
            // start isochronous transmission for the respective camera
            if(dc1394_start_iso_transmission(handles[id],
                                             cameras[id].node) !=
               DC1394_SUCCESS) {
                logger->log("'DC1394::startIsoTransmission': Error while trying to start ISO transmission!");
                return false;
            }
            
            sleep(1);
            waitCount++;
            
            if(dc1394_get_iso_status(handles[id],
                                     cameras[id].node, &on) !=
               DC1394_SUCCESS) {
                logger->log("'DC1394::startIsoTransmission': Error while trying to read ISO status");
                return false;
            }
        }
    }while((on == DC1394_FALSE) && (waitCount < 3));
    
    logger->log("%s", (on == DC1394_TRUE) ?
                "'DC1394::startIsoTransmission': ISO Transmission enabled" :
                "'DC1394::startIsoTransmission': ISO Transmission enabling failed!");
    
    isoStarted[id] = (on == DC1394_TRUE);
    return (on == DC1394_TRUE);
}


bool DC1394::stopIsoTransmission(unsigned int id) {
    dc1394bool_t on;
    unsigned int waitCount = 0;
    
    
    // read iso-transmission-status
    if(dc1394_get_iso_status(handles[id],
                             cameras[id].node, &on) !=
       DC1394_SUCCESS) {
        logger->log("'DC1394::stopIsoTransmission': Error while reading iso-transmission-status!");
        return false;
    }
    
    if(on == DC1394_FALSE) {
        logger->log("'DC1394::stopIsoTransmission': ISO Transmission already stopped (flag == %s)",
                    isoStarted[id] ? "true !!!" : "false ???");
        isoStarted[id] = false;
        return true;
    }
    
    // wait for iso transmission to be stopped
    do{
        if(on == DC1394_TRUE) {
            // stop isochronous transmission for the respective camera
            if(dc1394_stop_iso_transmission(handles[id],
                                            cameras[id].node) !=
               DC1394_SUCCESS) {
                logger->log("'DC1394::stopIsoTransmission': Error while trying to stop iso transmission");
                return false;
            }
            
            // wait for one second
            sleep(1);
            waitCount++;
            
            // get the current iso-transmission-status
            if(dc1394_get_iso_status(handles[id],
                                     cameras[id].node, &on) !=
               DC1394_SUCCESS) {
                logger->log("'DC1394::stopIsoTransmission': Error while reading iso-transmission-status!");
                return false;
            }
        }
    } while((on == DC1394_TRUE) && (waitCount < 3));
    
    isoStarted[id] = (on == DC1394_TRUE);
    
    return (on != DC1394_TRUE);
}


bool DC1394::getIsoStatus(unsigned int id, bool *on) {
    dc1394bool_t state;
    
    if(dc1394_get_iso_status(handles[id],
                             cameras[id].node, &state) !=
       DC1394_SUCCESS)
        return false;
    
    *on = state == DC1394_TRUE;
    
    return true;
}


int DC1394::getNumberOfCameras() {
    return camCount;
}


char *DC1394::getCameraModel(unsigned int id) {
    return camInfos[id].model;
}


char *DC1394::getCameraVendor(unsigned int id) {
    return camInfos[id].vendor;
}


octlet_t DC1394::getCCROffset(unsigned int id) {
    return camInfos[id].ccr_offset;
}


u_int64_t DC1394::getEUID(unsigned int id) {
    return camInfos[id].euid_64;
}


bool DC1394::getSWVersion(unsigned int id, int *swVersion) {
    if(dc1394_get_sw_version(handles[id], cameras[id].node,
                             swVersion) != DC1394_SUCCESS) {
        return false;
    }
    
    return true;
}


bool DC1394::getRevision(unsigned int id, quadlet_t * revision) {
    if(dc1394_query_revision(handles[id], cameras[id].node, MODE_EXIF,
                             revision) != DC1394_SUCCESS) {
        return false;
    }
    
    return true;
}


bool DC1394::getuCVersion(unsigned int id, unsigned int *major, unsigned int *minor) {
    if(avt1394_get_uc_version(handles[id],
                              cameras[id].node,
                              major, minor) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::getFirmwareVersion(unsigned int id, unsigned int *camID,
                                unsigned int *major,
                                unsigned int *minor,
                                bool *marlin) {
    avt_firmware_t firmware;
    
    if(avt1394_get_firmware_version(handles[id],
                                    cameras[id].node,
                                    &firmware) != DC1394_SUCCESS)
        return false;
    
    
    *camID = firmware.nCameraId;
    *major = firmware.nMajor;
    *minor = firmware.nMinor;
    *marlin = (firmware.bMarlin == DC1394_TRUE);
    
    return true;
}


bool DC1394::dmaSingleCapture(unsigned int id) {
    int singleCaptureCall = 0;
    
    if((camCount >= id) && dmaSetUp[id]
       && isoStarted[id]
       && ((singleCaptureCall = dc1394_dma_single_capture(&cameras[id])) != 0)) {
        if(cameras[id].frame_width != (int)width[id]) {
            logger->log("'DC1394::dmaSingleCapture': Width of grabbed image differs from expected value (%d - %d)",
                        cameras[id].frame_width,
                        width[id]);
            return false;
        }
        if(cameras[id].frame_height != (int)height[id]) {
            logger->log("'DC1394::dmaSingleCapture': Height of grabbed image differs from expected value (%d - %d)",
                        cameras[id].frame_width,
                        width[id]);
            return false;
        }
        return true;
    }
    
    return false;
}


bool DC1394::dmaReleaseBuffer(unsigned int id) {
    if(cameras[id].capture_buffer != NULL) {
        if(dc1394_dma_done_with_buffer(&cameras[id]) !=
           DC1394_SUCCESS) {
            logger->log("'DC1394::dmaReleaseBuffer': 'dma_done_with_buffer()' failed!");
            return false;
        }
        
        return true;
    }
    else {
        logger->log("'DC1394::dmaReleaseBuffer': DMA-Buffer == NULL!");
        return false;
    }
}


int *DC1394::getCaptureBuffer(unsigned int id) {
    return cameras[id].capture_buffer;
}


bool DC1394::getCurrentCaptureMode(unsigned int id, unsigned int *mode,
                                   unsigned int *frameCount) {
    dc1394bool_t iso, oneShot, multiShot;
    
    
    if(dc1394_get_iso_status(handles[id],
                             cameras[id].node,
                             &iso) != DC1394_SUCCESS) {
        
        logger->log("'DC1394::getCurrentCaptureMode': Reading ISO-State failed!");
        return false;
    }
    
    if(iso == DC1394_TRUE) {
        *mode = CAPTURE_MODE_FREERUN;
        *frameCount = 0;
        return true;
    }
    
    if(dc1394_get_one_shot(handles[id],
                           cameras[id].node,
                           &oneShot) != DC1394_SUCCESS) {
        logger->log("'DC1394::getCurrentCaptureMode': Reading OneShot-State failed!");
        return false;
    }
    
    if(oneShot == DC1394_TRUE) {
        *mode = CAPTURE_MODE_ONESHOT;
        *frameCount = 0;
        return true;
    }
    
    if(dc1394_get_multi_shot(handles[id],
                             cameras[id].node,
                             &multiShot, frameCount) != DC1394_SUCCESS) {
        logger->log("'DC1394::getCurrentCaptureMode': Reading MultiShot-State failed!");
        return false;
    }
    
    if(multiShot == DC1394_TRUE) {
        *mode = CAPTURE_MODE_MULTISHOT;
        return true;
    }
    
    *mode = CAPTURE_MODE_UNSUPPORTED;
    *frameCount = 0;
    return false;
}


dc1394_cameracapture DC1394::getCameracapture(unsigned int id) {
    return cameras[id];
}


dc1394_feature_info DC1394::featureInfo(unsigned int id, unsigned int f) {
    return ((featureSets[id]).feature)[f];
}


dc1394_feature_set DC1394::featureSet(unsigned int id) {
    // read current featureSet from camera
    if(dc1394_get_camera_feature_set(handles[id],
                                     cameras[id].node,
                                     &featureSets[id]) != DC1394_SUCCESS) {
        
        logger->log("'DC1394::featureSet': Error while trying to read the featureset of camera %d",
                    camCount);
        for(int i = 0; i < NUM_FEATURES; i++) {
            ((featureSets[id]).feature)[i].available = (dc1394bool_t)false;
        }
    }
    
    
    return featureSets[id];
}


bool DC1394::featureReadOutCapable(unsigned int id, int featureIndex) {
    dc1394bool_t capability;
    
    if(dc1394_can_read_out(handles[id],
                           cameras[id].node,
                           featureIndex + FEATURE_MIN,
                           &capability) != DC1394_SUCCESS)
        return false;
    
    if(capability == DC1394_TRUE)
        return true;
    return false;
}


bool DC1394::dc1394InitCamera(unsigned int id) {
    if(dc1394_init_camera(handles[id], cameras[id].node) != DC1394_SUCCESS)
        return false;
    
    return true;
}


// direct access

bool DC1394::readData(unsigned int id, unsigned long address, unsigned int *data, bool sio,
                      bool f7, unsigned int f7Mode) {
    octlet_t addr = (octlet_t)address;
    quadlet_t d;
    unsigned int f7Offset = 0;
    
    
    if(f7) {
        readData(id, 0x2e0 + (f7Mode * 4), &f7Offset);
        f7Offset *= 4;
    }
    
    if(!sio && !f7 && address < 0x1000) {
        if(AvtDCGetCameraControlRegister(handles[id],
                                       cameras[id].node, addr, &d) != DC1394_SUCCESS)
            return false;
    }
    else {
        if(AvtGetCameraControlRegister(handles[id],
                                       cameras[id].node, addr + f7Offset, &d,
                                       sio ? DC1394_TRUE : DC1394_FALSE) != DC1394_SUCCESS)
            return false;
    }
    
    *data = (unsigned int)d;
    
    return true;
}


bool DC1394::writeData(unsigned int id, unsigned long address, unsigned int data, bool sio,
                       bool f7, unsigned int f7Mode) {
    octlet_t addr = (octlet_t)address;
    quadlet_t d = (quadlet_t)data;
    unsigned int f7Offset = 0;
    
    
    if(f7) {
        readData(id, 0x2e0 + (f7Mode * 4), &f7Offset);
        f7Offset *= 4;
    }
    
    if(!sio && !f7 && address < 0x1000) {
        if(AvtDCSetCameraControlRegister(handles[id],
                                         cameras[id].node, addr, d) != DC1394_SUCCESS)
            return false;
    }
    else {
        if(AvtSetCameraControlRegister(handles[id],
                                         cameras[id].node, addr + f7Offset, d,
                                         sio ? DC1394_TRUE : DC1394_FALSE) != DC1394_SUCCESS)
            return false;
    }
    
    return true;
}

            

// Std. Features
bool DC1394::setFeatureValue(unsigned int id, int featureIndex, int newValue) {
    if(((featureSets[id]).feature[featureIndex]).abs_control == DC1394_TRUE) {
        if(dc1394_set_absolute_feature_value(handles[id],
                                             cameras[id].node,
                                             featureIndex + FEATURE_MIN,
                                             (float)newValue) != DC1394_SUCCESS) {
            logger->log("'DC1394::setFeatureValue': Unable to set absolute value of feature %s to %f",
                        featureString(featureIndex), (float)newValue);
            return false;
        }
    }
    else {
        if(dc1394_set_feature_value(handles[id],
                                    cameras[id].node,
                                    featureIndex + FEATURE_MIN,
                                    newValue) != DC1394_SUCCESS) {
            logger->log("'DC1394::setFeatureValue': Unable to set value of feature %s to %d",
                        featureString(featureIndex), newValue);
            return false;
        }
    }
    
    return true;
}


bool DC1394::setWhiteBalanceValue(unsigned int id, int newUValue, int newVValue) {
    if(dc1394_set_white_balance(handles[id],
                                cameras[id].node,
                                newUValue,
                                newVValue) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::setFeatureAutoMode(unsigned int id, int featureIndex, bool newValue) {
    if(dc1394_auto_on_off(handles[id],
                          cameras[id].node,
                          featureIndex + FEATURE_MIN,
                          newValue) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::setFeatureOn(unsigned int id, int featureIndex, bool newValue) {
    if(dc1394_feature_on_off(handles[id],
                             cameras[id].node,
                             featureIndex + FEATURE_MIN,
                             newValue ? 1 : 0) != DC1394_SUCCESS)
        return false;
    
    if(featureIndex == FEATURE_TRIGGER - FEATURE_MIN) {
        if(!printISOState(id))
            return false;
    }
    
    return true;
}


bool DC1394::featureOnePush(unsigned int id, int featureIndex) {
    if(dc1394_start_one_push_operation(handles[id],
                                       cameras[id].node,
                                       featureIndex + FEATURE_MIN) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::setTriggerMode(unsigned int id, int mode) {
    quadlet_t trigger;
    
    
    if(mode < 4) {
        // only modes 0...3 are supported up to 'dc1394_control.so.11.0'
        if(dc1394_set_trigger_mode(handles[id],
                                   cameras[id].node,
                                   mode + TRIGGER_MODE_MIN) != DC1394_SUCCESS) {
            logger->log("'DC1394::setTriggerMode': Unable to set trigger mode!");
            return false;
        }
    }
    else {
        if(mode != 15) {
            logger->log("'DC1394::setTriggerMode': Only vendor-specific 'Mode 15' supported!");
            return false;
        }
        
        if(GetCameraControlRegister(handles[id],
                                    cameras[id].node,
                                    TRIGGER_MODE_OFFSET,
                                    &trigger) == -1) {
            logger->log("'DC1394::setTriggerMode': Error while trying to read TRIGGER_MODE Register contents!");
            return false;
        }
        
        trigger &= RAW_TRIGGER_MODE_MASK;
        trigger |= RAW_TRIGGER_MODE_15;
        
        if(SetCameraControlRegister(handles[id],
                                    cameras[id].node,
                                    TRIGGER_MODE_OFFSET,
                                    trigger) == -1) {
            logger->log("'DC1394::setTriggerMode': Error while trying to change Trigger-Mode!");
            return false;
        }
    }
    
    return true;
}


bool DC1394::setTriggerPolarity(unsigned int id, dc1394bool_t polarity) {
    if(dc1394_set_trigger_polarity(handles[id],
                                   cameras[id].node,
                                   polarity) != DC1394_SUCCESS)
        return false;
    
    return true;
}


int DC1394::getTriggerMode(unsigned int id) {
    unsigned int mode;
    
    if(dc1394_get_trigger_mode(handles[id],
                               cameras[id].node,
                               &mode) != DC1394_SUCCESS)
        return -1;
    
    return mode - TRIGGER_MODE_0;
}


bool DC1394::getTriggerModeCapableMask(unsigned int id, unsigned int *mask) {
    int retval;
    quadlet_t value = 0xffffffff;
    
    if((retval = GetCameraControlRegister(handles[id],
                                          cameras[id].node,
                                          TRIGGER_INQ_OFFSET,
                                          &value)) == -1)
        return false;
    
    *mask = (unsigned int)value;
    
    return true;
}


bool DC1394::getTriggerState(unsigned int id, bool *state) {
    dc1394bool_t on;
    
    if(dc1394_is_feature_on(handles[id],
                            cameras[id].node,
                            FEATURE_TRIGGER, &on) != DC1394_SUCCESS)
        return false;
    
    *state = (on == DC1394_TRUE);
    
    return true;
}


bool DC1394::getTriggerSources(unsigned int id, unsigned int *sources) {
    quadlet_t mode;
    quadlet_t src;
    
    if(GetCameraControlRegister(handles[id],
                                cameras[id].node,
                                TRIGGER_INQ_OFFSET,
                                &src) == -1)
        return false;
    
    if(GetCameraControlRegister(handles[id],
                                cameras[id].node,
                                TRIGGER_MODE_OFFSET,
                                &mode) == -1)
        return false;
    
    *sources = (src & 0x00ff0000UL) >> 16;
    
    return true;
}


bool DC1394::getCurrentTriggerSource(unsigned int id, unsigned int *source) {
    quadlet_t currentSource;
    
    if(GetCameraControlRegister(handles[id],
                                cameras[id].node,
                                TRIGGER_MODE_OFFSET,
                                &currentSource) == -1)
        return false;
    
    *source = (currentSource & 0x00e00000UL) >> 21;
    
    return true;
}


bool DC1394::setTriggerSource(unsigned int id, unsigned int source) {
    quadlet_t currentSource;
    
    if(GetCameraControlRegister(handles[id],
                                cameras[id].node,
                                TRIGGER_MODE_OFFSET,
                                &currentSource) == -1)
        return false;
    
    currentSource = (currentSource & 0xff1fffff) | (source << 21);
    
    if(SetCameraControlRegister(handles[id],
                                cameras[id].node,
                                TRIGGER_MODE_OFFSET,
                                currentSource) == -1)
        return false;
    
    return true;
}


bool DC1394::swTriggerAvailable(unsigned int id) {
    int retval;
    quadlet_t value = 0xffffffff;
    
    if((retval = GetCameraControlRegister(handles[id ],
                                          cameras[id].node,
                                          SW_TRIGGER_OFFSET,
                                          &value)) == -1) {
        logger->log("'DC1394::swTriggerAvailable': 'GetCameraControlRegister('SW-Trigger')' returned: %d (value = %d)",
                    retval, value);
        return false;
    }
    
    return true;
}


bool DC1394::getSWTriggerState(unsigned int id, bool *state) {
    int retval;
    quadlet_t value = 0xffffffff;
    
    if((retval = GetCameraControlRegister(handles[id ],
                                          cameras[id].node,
                                          SW_TRIGGER_OFFSET,
                                          &value)) == -1) {
        logger->log("'DC1394::getSWTriggerState': 'GetCameraControlRegister()' returned: 0x%x (value = 0x%x)", retval, value);
        return false;
    }
    
    *state = (value == 0x00000000) ? SW_TRIGGER_READY : SW_TRIGGER_BUSY;
    
    return true;
}


bool DC1394::startSWTriggerExecute(unsigned int id) {
    if(SetCameraControlRegister(handles[id],
                                cameras[id].node,
                                SW_TRIGGER_OFFSET,
                                SW_TRIGGER_SET) == -1)
        return false;
    
    return true;
}


bool DC1394::stopSWTriggerExecute(unsigned int id) {
    if(SetCameraControlRegister(handles[id],
                                cameras[id].node,
                                SW_TRIGGER_OFFSET,
                                SW_TRIGGER_RESET) == -1)
        return false;
    
    return true;
}


bool DC1394::triggerDelayAvailable(unsigned int id)  {
    quadlet_t value;
    
    if(GetCameraControlRegister(handles[id],
                                cameras[id].node,
                                TRIGGER_DLY_OFFSET,
                                &value) == -1)
        return false;
    
    return true;
}


bool DC1394::getTriggerDelayInfo(unsigned int id, unsigned int *triggerInfo) {
    if(!triggerDelayAvailable(id))
        return false;
    
    if(GetCameraControlRegister(handles[id],
                                cameras[id].node,
                                TRIGGER_DLY_INQ_OFFSET,
                                triggerInfo) == -1)
        return false;
    
    return true;
}


bool DC1394::triggerDelayEnabled(unsigned int id, bool *on) {
    quadlet_t value;
    
    if(GetCameraControlRegister(handles[id],
                                cameras[id].node,
                                TRIGGER_DLY_OFFSET,
                                &value) == -1)
        return false;
    
    *on = ((value & TRIGGER_DLY_ON_MASK) != 0);
    
    return true;
}


bool DC1394::triggerDelayEnable(unsigned int id, bool on) {
    quadlet_t value;
    
    if(GetCameraControlRegister(handles[id],
                                cameras[id].node,
                                TRIGGER_DLY_OFFSET,
                                &value) == -1)
        return false;
    
    value = (value & ~TRIGGER_DLY_ON_MASK) |
            (on ? TRIGGER_DLY_ON_MASK : 0x00000000UL);
    
    if(SetCameraControlRegister(handles[id],
                                cameras[id].node,
                                TRIGGER_DLY_OFFSET,
                                value) == -1)
        return false;
    
    return true;
}


bool DC1394::getTriggerDelay(unsigned int id, unsigned int *delay) {
    quadlet_t value;
    
    if(GetCameraControlRegister(handles[id],
                                cameras[id].node,
                                TRIGGER_DLY_OFFSET,
                                &value) == -1)
        return false;
    
    if((value & TRIGGER_DLY_PRESENT_MASK) == 0)
        return false;
    
    *delay = value & TRIGGER_DLY_VALUE_MASK;
    
    return true;
}


bool DC1394::setTriggerDelay(unsigned int id, unsigned int delay) {
    quadlet_t value;
    
    if(GetCameraControlRegister(handles[id],
                                cameras[id].node,
                                TRIGGER_DLY_OFFSET,
                                &value) == -1)
        return false;
    
    if((value & TRIGGER_DLY_PRESENT_MASK) == 0)
        return false;
    
    value = (value & ~TRIGGER_DLY_VALUE_MASK) |
            (delay & TRIGGER_DLY_VALUE_MASK);
    
    if(SetCameraControlRegister(handles[id],
                                cameras[id].node,
                                TRIGGER_DLY_OFFSET,
                                value) == -1)
        return false;
    
    return true;
}


bool DC1394::enableAbsoluteControl(unsigned int id, unsigned int feature, bool enable) {
    if(dc1394_absolute_setting_on_off(handles[id],
                                      cameras[id].node,
                                      feature + FEATURE_MIN,
                                      (enable ? DC1394_TRUE : DC1394_FALSE)) !=
       DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::enableTriggerDelayAbsControl(unsigned int id, bool enable) {
    quadlet_t triggerDelay;
    unsigned int triggerDelayInfo;
    
    // query if trigger-delay abs-control available
    if(!getTriggerDelayInfo(id, &triggerDelayInfo))
        return false;
    
    if((triggerDelayInfo & TRIGGER_DLY_ABSCTL_MASK) == 0)
        return false;
    
    // get current TriggerDelay contents
    if(GetCameraControlRegister(handles[id],
                                cameras[id].node,
                                TRIGGER_DLY_OFFSET,
                                &triggerDelay) == -1)
        return false;
    
    // enable / disable TriggerDelay Abs.Control
    triggerDelay = (triggerDelay & ~TRIGGER_DLY_ABS_MASK) |
                   (enable ? TRIGGER_DLY_ABS_MASK : 0x00000000UL);
    
    // write back (modified) TriggerDelay contents
    if(SetCameraControlRegister(handles[id],
                                cameras[id].node,
                                TRIGGER_DLY_OFFSET,
                                triggerDelay) == -1)
        return false;
    
    return true;
}


bool DC1394::printISOState(unsigned int id) {
    quadlet_t value;
    
    // ISO_EN/Continuous_Shot
    if(GetCameraControlRegister(handles[id],
                                cameras[id].node,
                                0x00000614,
                                &value) == -1) {
        logger->log("'DC1394::printISOState': Reading 'ISO_EN/Continuous_Shot' failed!");
        return false;
    }
    
    logger->log("\t'ISO_EN/Continuous_Shot' = 0x%x", value);
    
    // One_Shot/Multi_Shot
    if(GetCameraControlRegister(handles[id],
                                cameras[id].node,
                                0x0000061c,
                                &value) == -1) {
        logger->log("'DC1394::printISOState': Reading 'One_Shot/Multi_Shot' failed!");
        return false;
    }
    
    logger->log("\t'One_Shot/Multi_Shot' = 0x%x", value);
    
    // Software_Trigger
    if(GetCameraControlRegister(handles[id],
                                cameras[id].node,
                                0x0000062c,
                                &value) == -1) {
        logger->log("'DC1394::printISOState': Reading 'Software_Trigger' failed!");
        return false;
    }
    
    logger->log("\t'Software_Trigger' = 0x%x\n", value);
    
    return true;
}



// Format 7
bool DC1394::f7Available(unsigned int id) {
    unsigned int colorCoding;
    
    if(dc1394_query_format7_color_coding(handles[id],
                                         cameras[id].node,
                                         mode[id],
                                         &colorCoding) != DC1394_SUCCESS)
        return false;
    
    return true;
}


unsigned int DC1394::getF7ColorCodingID(unsigned int id, unsigned int md) {
    if(colorCodingID[id] == 0) {
        if(dc1394_query_format7_color_coding_id(handles[id],
                                                cameras[id].node,
                                                md,
                                                &colorCodingID[id]) != DC1394_SUCCESS)
            return 0;
    }
    
    return colorCodingID[id];
}


bool DC1394::getF7ColorCoding(unsigned int id, unsigned int mode,unsigned int *colorCoding) {
    unsigned int ccID;
    
    *colorCoding = colorCodings[mode-MODE_FORMAT7_MIN][id];
    
    // read current color-coding-ID
    if(dc1394_query_format7_color_coding_id(handles[id],
                                            cameras[id].node,
                                            mode, &ccID) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::getF7MaxSize(unsigned int id, unsigned int md,
                          unsigned int *maxX, unsigned int *maxY) {
    if(dc1394_query_format7_max_image_size(handles[id],
                                           cameras[id].node,
                                           (md > 0) ? md : mode[id],
                                           maxX, maxY) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::getF7ImageSize(unsigned int id, unsigned int *w, unsigned int *h) {
    if(width[id] != 0) {
        *w = width[id];
        *h = height[id];
    }
    else {
        if(dc1394_query_format7_image_size(handles[id],
                                           cameras[id].node,
                                           mode[id],
                                           w, h) != DC1394_SUCCESS)
            return false;
    }
    
    return true;
}


bool DC1394::setF7ImageSize(unsigned int id, unsigned int w, unsigned int h) {
    if(dc1394_set_format7_image_size(handles[id],
                                     cameras[id].node,
                                     mode[id],
                                     w, h) != DC1394_SUCCESS)
        return false;
    
    width[id] = w;
    height[id] = h;
    
    return true;
}


bool DC1394::getF7ImagePos(unsigned int id, unsigned int *x, unsigned int *y) {
    if(dc1394_query_format7_image_position(handles[id],
                                           cameras[id].node,
                                           mode[id],
                                           x, y) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::setF7ImagePos(unsigned int id, unsigned int newX, unsigned int newY) {
    if(dc1394_set_format7_image_position(handles[id],
                                         cameras[id].node,
                                         mode[id],
                                         newX, newY) != DC1394_SUCCESS)
        return false;
    
    x[id] = newX;
    y[id] = newY;
    
    return true;
}


bool DC1394::getF7Units(unsigned int id, unsigned int *hUnitSize, unsigned int *vUnitSize,
                        unsigned int *hPosUnit, unsigned int *vPosUnit) {
    if(dc1394_query_format7_unit_size(handles[id],
                                      cameras[id].node,
                                      mode[id],
                                      hUnitSize, vUnitSize) != DC1394_SUCCESS) {
        logger->log("'DC1394::getF7Units': Trying to read F7 Unit-Size for Mode %d failed", mode[id]);
        return false;
    }
    
    if(dc1394_query_format7_unit_position(handles[id],
                                          cameras[id].node,
                                          mode[id],
                                          hPosUnit, vPosUnit) != DC1394_SUCCESS) {
        logger->log("'DC1394::getF7Units': Trying to read F7 Position Units for Mode %d failed", mode[id]);
        
        return false;
    }
    
    return true;
}


bool DC1394::getF7PixelNumber(unsigned int id, unsigned int *pixelNumber) {
    if(dc1394_query_format7_pixel_number(handles[id],
                                         cameras[id].node,
                                         mode[id],
                                         pixelNumber) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::getF7TotalBytes(unsigned int id, unsigned long long int *totalBytes) {
    if(dc1394_query_format7_total_bytes(handles[id],
                                        cameras[id].node,
                                        mode[id],
                                        totalBytes) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::getF7PacketPerFrame(unsigned int id, unsigned int *ppf) {
    if(dc1394_query_format7_packet_per_frame(handles[id],
                                             cameras[id].node,
                                             mode[id],
                                             ppf) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::getF7BytePerPacket(unsigned int id, unsigned int *bpp) {
    if(dc1394_query_format7_byte_per_packet(handles[id],
                                            cameras[id].node,
                                            mode[id],
                                            bpp) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::setF7BytePerPacket(unsigned int id, unsigned int bpp) {
    if(dc1394_set_format7_byte_per_packet(handles[id],
                                          cameras[id].node,
                                          mode[id],
                                          bpp) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::getF7RecBytePerPacket(unsigned int id, unsigned int *bpp) {
    if(dc1394_query_format7_recommended_byte_per_packet(handles[id],
                                                        cameras[id].node,
                                                        mode[id],
                                                        bpp) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::getF7PacketParameters(unsigned int id, unsigned int *minBPP,
                                   unsigned int *maxBPP) {
    if(dc1394_query_format7_packet_para(handles[id],
                                        cameras[id].node,
                                        mode[id],
                                        minBPP, maxBPP) != DC1394_SUCCESS)
        return false;
    
    return true;
}



// adv. Features (AVT-specific)
bool DC1394::extShutterAvailable(unsigned int id) {
    unsigned int value;
    
    if(avt1394_get_extended_shutter(handles[id],
                                    cameras[id].node,
                                    &value) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::getExtShutter(unsigned int id, unsigned int *time) {
    if(avt1394_get_extended_shutter(handles[id],
                                    cameras[id].node,
                                    time) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::setExtShutter(unsigned int id, unsigned int time) {
    if(avt1394_set_extended_shutter(handles[id],
                                    cameras[id].node,
                                    time) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::getExposureTime(unsigned int id, unsigned int *time, bool *valid) {
    dc1394bool_t camValid;
    
    if(avt1394_get_exposure_time(handles[id],
                                 cameras[id].node,
                                 time, &camValid) != DC1394_SUCCESS)
        return false;
    
    *valid = (camValid == DC1394_TRUE);
    return true;
}


bool DC1394::getTimebase(unsigned int id, unsigned int *timebase) {
    if(avt1394_get_timebase(handles[id],
                            cameras[id].node,
                            timebase) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::setTimebase(unsigned int id, unsigned int timebase) {
    if(avt1394_set_timebase(handles[id],
                            cameras[id].node,
                            timebase) != DC1394_SUCCESS)
        return false;
    
    return true;
}


void DC1394::getShutterOffset(unsigned int id, unsigned int *offset) {
    avt1394_get_shutter_offset(handles[id],
                               cameras[id].node,
                               offset);
}


bool DC1394::shadingCorrectionAvailable(unsigned int id) {
    dc1394bool_t showImg, buildImg, busy, on;
    unsigned int numImg, maxImg;
    
    if(avt1394_get_shading_info(handles[id],
                                cameras[id].node,
                                &showImg, &buildImg, &busy,
                                &numImg, &maxImg, &on) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::shadingCorrectionBusy(unsigned int id, bool *pBusy) {
    dc1394bool_t showImg, buildImg, busy, on;
    unsigned int numImg, maxImg;
    
    if(avt1394_get_shading_info(handles[id],
                                cameras[id].node,
                                &showImg, &buildImg, &busy,
                                &numImg, &maxImg, &on) != DC1394_SUCCESS)
        return false;
    
    *pBusy = (busy == DC1394_TRUE);
    return true;
}


bool DC1394::getShadingCorrectionInfos(unsigned int id, bool *pShowImage,
                                       unsigned int *pGrabCount,
                                       unsigned int *pMaxSize, bool *pOn) {
    dc1394bool_t showImg, buildImg, busy, on;
    unsigned int numImg, maxImg;
    
    if(avt1394_get_shading_info(handles[id],
                                cameras[id].node,
                                &showImg, &buildImg, &busy,
                                &numImg, &maxImg, &on) != DC1394_SUCCESS)
        return false;
    
    *pShowImage = (showImg == DC1394_TRUE);
    *pGrabCount = numImg;
    *pMaxSize = maxImg;
    *pOn = (on == DC1394_TRUE);
    return true;
}


bool DC1394::shadingCorrectionValid(unsigned int id) {
    return shadingValid[id];
}


bool DC1394::shadingCorrectionBuildImage(unsigned int id, unsigned int grabCount) {
    if(avt1394_build_shading_image(handles[id],
                                   cameras[id].node,
                                   grabCount) != DC1394_SUCCESS)
        return false;
    
    shadingValid[id] = true;
    return true;
}


bool DC1394::shadingCorrectionLoadImage(unsigned int id, unsigned char *img,
                                        unsigned int size) {
    if(avt1394_load_shading(handles[id],
                            cameras[id].node,
                            img, size) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::shadingCorrectionReadImage(unsigned int id, unsigned char *img,
                                        unsigned int size) {
    if(avt1394_read_shading(handles[id],
                            cameras[id].node,
                            img, size) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::shadingCorrectionShowImage(unsigned int id, bool show) {
    if(avt1394_show_shading_img(handles[id],
                                cameras[id].node,
                                (show ? DC1394_TRUE : DC1394_FALSE)) != 
       DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::shadingCorrectionEnable(unsigned int id, bool enable) {
    if(avt1394_enable_shading(handles[id],
                              cameras[id].node,
                              enable ? DC1394_TRUE : DC1394_FALSE) !=
       DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::shadingCorrectionSetGrabCount(unsigned int id, unsigned int grabCount) {
    if(avt1394_set_shading_grab_count(handles[id],
                                      cameras[id].node,
                                      grabCount) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::lutAvailable(unsigned int id) {
    unsigned int lut, maxSize;
    dc1394bool_t on;
    
    if(avt1394_get_lut_info(handles[id],
                            cameras[id].node,
                            &lut, &maxSize, &on) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::getLUTInfos(unsigned int id, unsigned int *maxLutNo,
                         unsigned int *maxSize, bool *on){
    dc1394bool_t featureOn;
    
    if(avt1394_get_lut_info(handles[id],
                            cameras[id].node,
                            maxLutNo, maxSize, &featureOn) != DC1394_SUCCESS)
        return false;
    
    *on = (featureOn == DC1394_TRUE);
    
    return true;
}


bool DC1394::getLUTSelection(unsigned int id, unsigned int *lutNo) {
    if(avt1394_get_lut_selection(handles[id],
                                 cameras[id].node,
                                 lutNo) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::loadLUT(unsigned int id, unsigned char *lut, unsigned int size,
                     unsigned int lutNo) {
    if(avt1394_load_lut(handles[id],
                        cameras[id].node,
                        lutNo,  lut, size) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::selectLUT(unsigned int id, unsigned int lutNo) {
    if(avt1394_select_lut(handles[id],
                          cameras[id].node,
                          lutNo) != DC1394_SUCCESS)
        return false;
    
    return true;
}



bool DC1394::lutEnable(unsigned int id, bool enable) {
    if(avt1394_enable_lut(handles[id],
                          cameras[id].node,
                          (enable ? DC1394_TRUE : DC1394_FALSE)) !=
       DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::hdrAvailable(unsigned int id) {
    unsigned int maxKnee, knee1, knee2, knee3;
    dc1394bool_t on;
    
    if(avt1394_get_hdr_info(handles[id],
                            cameras[id].node,
                            &maxKnee, &knee1, &knee2, &knee3, &on) !=
       DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::getHDRInfos(unsigned int id, unsigned int *maxKnees,
                         unsigned int *knee1, unsigned int *knee2,
                         unsigned int *knee3, unsigned int *activeKnees, bool *on) {
    dc1394bool_t camOn;
    
    if(avt1394_get_hdr_info(handles[id],
                            cameras[id].node,
                            maxKnees, knee1, knee2, knee3, &camOn) !=
       DC1394_SUCCESS) {
        logger->log("'DC1394::getHDRInfos': Reading Knee-Values failed!");
        return false;
    }
    
    if(avt1394_get_active_knees(handles[id],
                                cameras[id].node,
                                activeKnees) != DC1394_SUCCESS) {
        logger->log("'DC1394::getHDRInfos': Reading number of active Knees failed!");
        return false;
    }
    
    *on = (camOn == DC1394_TRUE);
    return true;
}


bool DC1394::hdrSetValues(unsigned int id, unsigned int numKnees,
                          unsigned int knee1, unsigned int knee2, unsigned int knee3) {
    if(avt1394_set_knee_values(handles[id],
                               cameras[id].node,
                               numKnees, knee1, knee2, knee3) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::hdrEnable(unsigned int id, bool enable)  {
    if(avt1394_enable_hdr_mode(handles[id],
                               cameras[id].node,
                               (enable ? DC1394_TRUE : DC1394_FALSE)) !=
       DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::mirrorImageAvailable(unsigned int id) {
    dc1394bool_t on;
    
    if(avt1394_get_mirror_image_state(handles[id],
                                      cameras[id].node,
                                      &on) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::getMirrorImageState(unsigned int id, bool *state) {
    dc1394bool_t on;
    
    if(avt1394_get_mirror_image_state(handles[id],
                                      cameras[id].node,
                                      &on) != DC1394_SUCCESS)
        return false;
    
    *state = (on == DC1394_TRUE);
    return true;
}


bool DC1394::mirrorImageEnable(unsigned int id, bool enable) {
    if(avt1394_enable_mirror_image(handles[id],
                                   cameras[id].node,
                                   (enable ? DC1394_TRUE : DC1394_FALSE)) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::testImageAvailable(unsigned int id) {
    unsigned int present, active;
    
    if(avt1394_get_test_image(handles[id],
                              cameras[id].node,
                              &present,
                              &active) != DC1394_SUCCESS)
        return false;
    
    if(present == 0)
        return false;
    
    return true;
}


bool DC1394::getAvailableTestImages(unsigned int id, unsigned int *testImages) {
    unsigned int active;
    
    if(avt1394_get_test_image(handles[id],
                              cameras[id].node,
                              testImages,
                              &active) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::getTestImage(unsigned int id, unsigned int *imageNo){
    unsigned int present;
    
    if(avt1394_get_test_image(handles[id],
                              cameras[id].node,
                              &present,
                              imageNo) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::testImageSelect(unsigned int id, unsigned int imageNo) {
    if(avt1394_set_test_image(handles[id],
                              cameras[id].node,
                              imageNo) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::testImageEnable(unsigned int id, bool enable, unsigned int testImageNo) {
    if(avt1394_set_test_image(handles[id],
                              cameras[id].node,
                              (enable ? testImageNo : 0)) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::colorCorrAvailable(unsigned int id) {
    dc1394bool_t on;
    
    if(avt1394_get_color_corr_state(handles[id],
                                    cameras[id].node,
                                    &on) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::getColorCorrState(unsigned int id, bool *state) {
    dc1394bool_t on;
    
    if(avt1394_get_color_corr_state(handles[id],
                                    cameras[id].node,
                                    &on) != DC1394_SUCCESS)
        return false;
    
    *state = (on == DC1394_TRUE);
    return true;
}


bool DC1394::colorCorrEnable(unsigned int id, bool enable) {
    if(avt1394_enable_color_corr(handles[id],
                                 cameras[id].node,
                                 (enable ? DC1394_TRUE : DC1394_FALSE)) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::ioAvailable(unsigned int id, unsigned int ioNo) {
    dc1394bool_t polarity, state;
    unsigned int mode;
    unsigned int inFeature;
    unsigned int outFeature;
    
    
    switch(ioNo) {
    case 1:
        inFeature = AVT_INP1_FEATURE;
        outFeature = AVT_OUTP1_FEATURE;
        break;
    case 2:
        inFeature = AVT_INP2_FEATURE;
        outFeature = AVT_OUTP2_FEATURE;
        break;
    case 3:
        inFeature = AVT_INP3_FEATURE;
        outFeature = AVT_OUTP3_FEATURE;
        break;
    default:
        inFeature = AVT_INP1_FEATURE;
        outFeature = AVT_OUTP1_FEATURE;
        break;
    }
    
    // input available ?
    if(avt1394_get_io_ctrl(handles[id],
                           cameras[id].node,
                           inFeature,
                           &polarity, &mode, &state) != DC1394_SUCCESS) {
        logger->log("DC1394::ioAvailable': Reading Input-Feature-Infos failed!");
        return false;
    }
    
    // output available ?
    if(avt1394_get_io_ctrl(handles[id],
                           cameras[id].node,
                           outFeature,
                           &polarity, &mode, &state) != DC1394_SUCCESS) {
        logger->log("DC1394::ioAvailable': Reading Output-Feature-Infos failed!");
        return false;
    }
    
    // both - input and output - available
    return true;
}


bool DC1394::getIOInCtrl(unsigned int id, unsigned int ioNo, bool *polarity,
                         unsigned int *mode, bool *state) {
    dc1394bool_t camPol, camState;
    unsigned int ioFeature;
    
    
    switch(ioNo) {
    case 1:
        ioFeature = AVT_INP1_FEATURE;
        break;
    case 2:
        ioFeature = AVT_INP2_FEATURE;
        break;
    case 3:
        ioFeature = AVT_INP3_FEATURE;
        break;
    default:
        ioFeature = AVT_INP1_FEATURE;
        break;
    }
    
    if(avt1394_get_io_ctrl(handles[id],
                           cameras[id].node,
                           ioFeature,
                           &camPol, mode, &camState) != DC1394_SUCCESS)
        return false;
    
    *polarity = (camPol == DC1394_TRUE);
    *state = (camState == DC1394_TRUE);
    
    return true;
}


bool DC1394::getIOOutCtrl(unsigned int id, unsigned int ioNo, bool *polarity,
                          unsigned int *mode, bool *state) {
    dc1394bool_t camPol, camState;
    unsigned int ioFeature;
    
    
    switch(ioNo) {
    case 1:
        ioFeature = AVT_OUTP1_FEATURE;
        break;
    case 2:
        ioFeature = AVT_OUTP2_FEATURE;
        break;
    case 3:
        ioFeature = AVT_OUTP3_FEATURE;
        break;
    default:
        ioFeature = AVT_OUTP1_FEATURE;
        break;
    }
    
    if(avt1394_get_io_ctrl(handles[id],
                           cameras[id].node,
                           ioFeature,
                           &camPol, mode, &camState) != DC1394_SUCCESS)
        return false;
    
    *polarity = (camPol == DC1394_TRUE);
    *state = (camState == DC1394_TRUE);
    
    return true;
}


bool DC1394::setIOInCtrl(unsigned int id, unsigned int ioNo, bool polarity,
                         unsigned int mode) {
    unsigned int ioFeature;
    
    
    switch(ioNo) {
    case 1:
        ioFeature = AVT_INP1_FEATURE;
        break;
    case 2:
        ioFeature = AVT_INP2_FEATURE;
        break;
    case 3:
        ioFeature = AVT_INP3_FEATURE;
        break;
    default:
        ioFeature = AVT_INP1_FEATURE;
        break;
    }
    
    if(avt1394_set_io_ctrl(handles[id],
                           cameras[id].node,
                           ioFeature,
                           (polarity ? DC1394_TRUE : DC1394_FALSE),
                           mode, DC1394_FALSE) !=
       DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::setIOOutCtrl(unsigned int id, unsigned int ioNo, bool polarity,
                          unsigned int mode, bool state) {
    unsigned int ioFeature;
    
    
    switch(ioNo) {
    case 1:
        ioFeature = AVT_OUTP1_FEATURE;
        break;
    case 2:
        ioFeature = AVT_OUTP2_FEATURE;
        break;
    case 3:
        ioFeature = AVT_OUTP3_FEATURE;
        break;
    default:
        ioFeature = AVT_OUTP1_FEATURE;
        break;
    }
    
    if(avt1394_set_io_ctrl(handles[id],
                           cameras[id].node,
                           ioFeature,
                           (polarity ? DC1394_TRUE : DC1394_FALSE),
                           mode,
                           (state ? DC1394_TRUE : DC1394_FALSE)) !=
       DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::serialAvailable(unsigned int id) {
    avt_serial_mode_t mode;
    
    if(avt1394_get_serial_mode(handles[id],
                               cameras[id].node,
                               &mode) != DC1394_SUCCESS) {
        logger->log("#DC1394::serialAvailable': Error while trying to read SIO mode-register!");
        return false;
    }
    
/*
    if(mode.nBufferSize == 0)
        return false;
*/
    
    return true;
}


bool DC1394::getSerialParameters(unsigned int id, unsigned int *baud,
                                 unsigned int *len, unsigned int *par, unsigned int *stop,
                                 unsigned int *bufSize) {
    avt_serial_mode_t mode;
    
    if(avt1394_get_serial_mode(handles[id],
                               cameras[id].node,
                               &mode) != DC1394_SUCCESS)
        return false;
    
    *baud = mode.nBaudRate;
    
    switch(mode.nCharLen) {
    case AVT_7_CHAR:
        *len = 7;
        break;
    case AVT_8_CHAR:
        *len = 8;
        break;
    default:
        return false;
    }
    
    switch(mode.nParity) {
    case AVT_NO_PARITY:
        *par = SERIAL_NO_PARITY;
        break;
    case AVT_ODD_PARITY:
        *par = SERIAL_ODD_PARITY;
        break;
    case AVT_EVEN_PARITY:
        *par = SERIAL_EVEN_PARITY;
        break;
    default:
        return false;
    }
    
    switch(mode.nStopBit) {
    case AVT_ONE_STOPBIT:
        *stop = SERIAL_1_STOPBIT;
        break;
    case AVT_ONEHALF_STOPBIT:
        *stop = SERIAL_3_HALF_STOPBIT;
        break;
    case AVT_TWO_STOPBIT:
        *stop = SERIAL_2_STOPBIT;
        break;
    default:
        return false;
    }
    
    *bufSize = mode.nBufferSize;
    
    return true;
}


bool DC1394::setSerialParameters(unsigned int id, unsigned int baud, unsigned int len,
                                 unsigned int par, unsigned int stop) {
    avt_serial_mode_t mode;
    bool rxEnabled, txEnabled, rxReady, txReady, ovErr, frameErr, parErr;
    
    
    switch(baud) {
    case 300:
        mode.nBaudRate = AVT_BAUD_300;
        break;
    case 600:
        mode.nBaudRate = AVT_BAUD_600;
        break;
    case 1200:
        mode.nBaudRate = AVT_BAUD_1200;
        break;
    case 2400:
        mode.nBaudRate = AVT_BAUD_2400;
        break;
    case 4800:
        mode.nBaudRate = AVT_BAUD_4800;
        break;
    case 9600:
        mode.nBaudRate = AVT_BAUD_9600;
        break;
    case 19200:
        mode.nBaudRate = AVT_BAUD_19200;
        break;
    case 38400:
        mode.nBaudRate = AVT_BAUD_38400;
        break;
    case 57600:
        mode.nBaudRate = AVT_BAUD_57600;
        break;
    case 115200:
        mode.nBaudRate = AVT_BAUD_115200;
        break;
    case 230400:
        mode.nBaudRate = AVT_BAUD_230400;
        break;
    default:
        return false;
    }
    
    switch(len) {
    case 7:
        mode.nCharLen = AVT_7_CHAR;
        break;
    case 8:
        mode.nCharLen = AVT_8_CHAR;
        break;
    default:
        return false;
    }
    
    switch(par) {
    case SERIAL_NO_PARITY:
        mode.nParity = AVT_NO_PARITY;
        break;
    case SERIAL_ODD_PARITY:
        mode.nParity = AVT_ODD_PARITY;
        break;
    case SERIAL_EVEN_PARITY:
        mode.nParity = AVT_EVEN_PARITY;
        break;
    default:
        return false;
    }
    
    switch(stop) {
    case SERIAL_1_STOPBIT:
        mode.nStopBit = AVT_ONE_STOPBIT;
        break;
    case SERIAL_3_HALF_STOPBIT:
        mode.nStopBit = AVT_ONEHALF_STOPBIT;
        break;
    case SERIAL_2_STOPBIT:
        mode.nStopBit = AVT_TWO_STOPBIT;
        break;
    default:
        return false;
    }
    
    // mode.nBufferSize - !READ-ONLY!
    
    // before changing 'SERIAL_MODE_REG' - 'SERIAL_CONTROL_REG' == 0
    // read current 'SERIAL_CONTROL_REG' status
    if(!getSerialControl(id, &rxEnabled, &txEnabled,
                         &rxReady, &txReady,
                         &ovErr, &frameErr, &parErr))
        return false;
    
    // clear 'SERIAL_CONTROL_REG'
    if(!setSerialControl(id, false, false, true))
        return false;
    
    if(avt1394_set_serial_mode(handles[id],
                               cameras[id].node,
                               mode) != DC1394_SUCCESS)
        return false;
    
    // restore 'SERIAL_CONTROL_REG'
    if(!setSerialControl(id, rxEnabled, txEnabled, true))
        return false;
    
    return true;
}


bool DC1394::getSerialControl(unsigned int id, bool *rxEnable, bool *txEnable,
                              bool *rxBufferReady, bool *txBufferReady,
                              bool *overrunError, bool *framingError, bool *parityError) {
    avt_serial_control_t controlData;
    
    
    if(avt1394_get_serial_control(handles[id],
                                  cameras[id].node,
                                  &controlData) != DC1394_SUCCESS)
        return false;
    
    *rxEnable = (controlData.bRcvEnable == DC1394_TRUE);
    *txEnable = (controlData.bTxEnable == DC1394_TRUE);
    *rxBufferReady = (controlData.bRcvBufferReady == DC1394_TRUE);
    *txBufferReady = (controlData.bTxBufferReady == DC1394_TRUE);
    *overrunError = (controlData.bRcvOverrunError == DC1394_TRUE);
    *framingError = (controlData.bFramingError == DC1394_TRUE);
    *parityError = (controlData.bParityError == DC1394_TRUE);
    
    return true;
}


bool DC1394::setSerialControl(unsigned int id, bool rxEnable, bool txEnable, bool clearErrorFlags) {
    avt_serial_control_t controlData;
    
    
    controlData.bRcvEnable = (rxEnable ? DC1394_TRUE : DC1394_FALSE);
    controlData.bTxEnable = (txEnable ? DC1394_TRUE : DC1394_FALSE);
    controlData.bRcvOverrunError = (clearErrorFlags ? DC1394_FALSE : DC1394_TRUE);
    controlData.bFramingError = (clearErrorFlags ? DC1394_FALSE : DC1394_TRUE);
    controlData.bParityError = (clearErrorFlags ? DC1394_FALSE : DC1394_TRUE);
    
    if(avt1394_set_serial_control(handles[id],
                                  cameras[id].node,
                                  controlData) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::getSerialBufferSize(unsigned int id, unsigned int *size) {
    avt_serial_mode_t mode;
    
    if(avt1394_get_serial_mode(handles[id],
                               cameras[id].node,
                               &mode) != DC1394_SUCCESS)
        return false;
    
    *size = mode.nBufferSize;
    
    return true;
}


bool DC1394::getNoOfRxBytes(unsigned int id, unsigned int *number) {
    unsigned int cnt;
    
    if(avt1394_get_recv_serialbuffersize(handles[id],
                                         cameras[id].node,
                                         number, &cnt) != DC1394_SUCCESS)
        return false;
    
    return true;
}


int DC1394::readSerialData(unsigned int id, char *byte, unsigned int number) {
    unsigned bufStatus, bufCnt;
    avt_serial_data_t serialData;
    
    
    // only a maximum of 4 bytes will be read
    if(number > 4)
        number = 4;
    
    // read valid data size of current receive bufffer 'RBUF_ST' and the buffer size 'RBUF_CNT'
    if(avt1394_get_recv_serialbuffersize(handles[id],
                                         cameras[id].node,
                                         &bufStatus, &bufCnt) !=
       DC1394_SUCCESS)
        return -1;
    
    if(bufStatus < number) {
        number  = bufStatus;
    }
    
    // write input data length to 'RBUF_CNT'
    if(avt1394_set_recv_serialbuffersize(handles[id],
                                         cameras[id].node,
                                         number) != DC1394_SUCCESS)
        return -1;
    
    // read received character from 'SIO_DATA_REGISTER'
    if(avt1394_get_serial_data(handles[id],
                               cameras[id].node,
                               &serialData) != DC1394_SUCCESS)
        return -1;
    
    byte[0] = (char)serialData.nChar0;
    byte[1] = (char)serialData.nChar1;
    byte[2] = (char)serialData.nChar2;
    byte[3] = (char)serialData.nChar3;
    
    return number;
}


bool DC1394::sendSerialData(unsigned int id, char *buffer, unsigned int reqSize) {
    unsigned int i, bufStatus, bufSize;
    avt_serial_data_t serialData;
    
    
    for(i = 0; i < reqSize; i++) {
        // read valid data size of current receive bufffer 'TBUF_ST' and the buffer size 'TBUF_CNT'
        if(avt1394_get_tx_serialbuffersize(handles[id],
                                           cameras[id].node,
                                           &bufStatus, &bufSize) !=
           DC1394_SUCCESS)
            return false;
        
        if(bufStatus == 0)
            return false;
        
        // prepare output buffer
        serialData.nChar0 = buffer[i];
        serialData.nChar1 = 0;
        serialData.nChar2 = 0;
        serialData.nChar3 = 0;
        
        // write data to be transmitted into 'SIO_DATA_REGISTER'
        if(avt1394_send_serial_data(handles[id],
                                    cameras[id].node,
                                    serialData) != DC1394_SUCCESS)
            return false;
        
        // write number of bytes to be transmitted for starting the transmission
        if(avt1394_set_tx_serialbuffersize(handles[id],
                                           cameras[id].node,
//                                           4) != DC1394_SUCCESS)
                                           1) != DC1394_SUCCESS)
            return false;
        
    }
    
    return true;
}


bool DC1394::advTriggerDelayAvailable(unsigned int id)  {
    unsigned int delay;
    dc1394bool_t state;
    
    if(avt1394_get_adv_trigger_delay(handles[id],
                                     cameras[id].node,
                                     &delay, &state) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::advTriggerDelayEnabled(unsigned int id, bool *on) {
    unsigned int delay;
    dc1394bool_t state;
    
    if(avt1394_get_adv_trigger_delay(handles[id],
                                     cameras[id].node,
                                     &delay, &state) != DC1394_SUCCESS)
        return false;
    
    *on = state;
    
    return true;
}


bool DC1394::advTriggerDelayEnable(unsigned int id, bool on) {
    if(avt1394_enable_adv_trigger_delay(handles[id],
                                        cameras[id].node,
                                        (on ? DC1394_TRUE : DC1394_FALSE)) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::getAdvTriggerDelay(unsigned int id, unsigned int *delay) {
    dc1394bool_t state;
    
    if(avt1394_get_adv_trigger_delay(handles[id],
                                     cameras[id].node,
                                     delay, &state) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::setAdvTriggerDelay(unsigned int id, unsigned int delay) {
    if(avt1394_set_adv_trigger_delay(handles[id],
                                     cameras[id].node,
                                     delay) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::dsnuAvailable(unsigned int id) {
    avt_dsnu_blemish_t data;
    dc1394bool_t on;
    
    data.bType = AVT_DSNU;
    
    if(avt1394_get_dsnu_blemish(handles[id],
                                cameras[id].node,
                                &data, &on) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::blemishAvailable(unsigned int id) {
    avt_dsnu_blemish_t data;
    dc1394bool_t on;
    
    data.bType = AVT_BLEMISH;
    
    if(avt1394_get_dsnu_blemish(handles[id],
                                cameras[id].node,
                                &data, &on) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::dsnuEnabled(unsigned int id, bool *on) {
    avt_dsnu_blemish_t data;
    dc1394bool_t state;
    
    data.bType = AVT_DSNU;
    
    if(avt1394_get_dsnu_blemish(handles[id],
                                cameras[id].node,
                                &data, &state) != DC1394_SUCCESS)
        return false;
    
    *on = (state == DC1394_TRUE);
    
    return true;
}


bool DC1394::dsnuEnable(unsigned int id, bool on) {
    if(avt1394_enable_dsnu_blemish(handles[id],
                                   cameras[id].node,
                                   AVT_DSNU,
                                   on ? DC1394_TRUE : DC1394_FALSE) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::blemishEnabled(unsigned int id, bool *on) {
    avt_dsnu_blemish_t data;
    dc1394bool_t state;
    
    data.bType = AVT_BLEMISH;
    
    if(avt1394_get_dsnu_blemish(handles[id],
                                cameras[id].node,
                                &data, &state) != DC1394_SUCCESS)
        return false;
    
    *on = (state == DC1394_TRUE);
    
    return true;
}


bool DC1394::blemishEnable(unsigned int id, bool on) {
    if(avt1394_enable_dsnu_blemish(handles[id],
                                   cameras[id].node,
                                   AVT_BLEMISH,
                                   on ? DC1394_TRUE : DC1394_FALSE) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::dsnuBusy(unsigned int id, bool *busy) {
    avt_dsnu_blemish_t data;
    dc1394bool_t on;
    
    data.bType = AVT_DSNU;
    
    if(avt1394_get_dsnu_blemish(handles[id],
                                cameras[id].node,
                                &data, &on) != DC1394_SUCCESS)
        return false;
    
    *busy = (data.bBusy == DC1394_TRUE);
    
    return true;
}


bool DC1394::blemishBusy(unsigned int id, bool *busy) {
    avt_dsnu_blemish_t data;
    dc1394bool_t on;
    
    data.bType = AVT_BLEMISH;
    
    if(avt1394_get_dsnu_blemish(handles[id],
                                cameras[id].node,
                                &data, &on) != DC1394_SUCCESS)
        return false;
    
    *busy = (data.bBusy == DC1394_TRUE);
    
    return true;
}


bool DC1394::getDSNUImgCnt(unsigned int id, unsigned int *cnt) {
    avt_dsnu_blemish_t data;
    dc1394bool_t on;
    
    data.bType = AVT_DSNU;
    
    if(avt1394_get_dsnu_blemish(handles[id],
                                cameras[id].node,
                                &data, &on) != DC1394_SUCCESS)
        return false;
    
    *cnt = data.nNumImg;
    
    return true;
}


bool DC1394::getBlemishImgCnt(unsigned int id, unsigned int *cnt) {
    avt_dsnu_blemish_t data;
    dc1394bool_t on;
    
    data.bType = AVT_BLEMISH;
    
    if(avt1394_get_dsnu_blemish(handles[id],
                                cameras[id].node,
                                &data, &on) != DC1394_SUCCESS)
        return false;
    
    *cnt = data.nNumImg;
    
    return true;
}


bool DC1394::setDSNUImgCnt(unsigned int id, unsigned int cnt) {
    avt_dsnu_blemish_t data;
    dc1394bool_t on;
    
    data.bType = AVT_DSNU;
    
    if(avt1394_get_dsnu_blemish(handles[id],
                                cameras[id].node,
                                &data, &on) != DC1394_SUCCESS)
        return false;
    
    data.nNumImg = cnt;
    
    if(avt1394_set_dsnu_blemish(handles[id],
                                cameras[id].node,
                                data) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::setBlemishImgCnt(unsigned int id, unsigned int cnt) {
    avt_dsnu_blemish_t data;
    dc1394bool_t on;
    
    data.bType = AVT_BLEMISH;
    
    if(avt1394_get_dsnu_blemish(handles[id],
                                cameras[id].node,
                                &data, &on) != DC1394_SUCCESS)
        return false;
    
    data.nNumImg = cnt;
    
    if(avt1394_set_dsnu_blemish(handles[id],
                                cameras[id].node,
                                data) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::dsnuShowImg(unsigned int id, bool show) {
    avt_dsnu_blemish_t data;
    dc1394bool_t on;
    
    data.bType = AVT_DSNU;
    
    if(avt1394_get_dsnu_blemish(handles[id],
                                cameras[id].node,
                                &data, &on) != DC1394_SUCCESS)
        return false;
    
    data.bShowImg = (show ? DC1394_TRUE : DC1394_FALSE);
    
    if(avt1394_set_dsnu_blemish(handles[id],
                                cameras[id].node,
                                data) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::blemishShowImg(unsigned int id, bool show) {
    avt_dsnu_blemish_t data;
    dc1394bool_t on;
    
    data.bType = AVT_BLEMISH;
    
    if(avt1394_get_dsnu_blemish(handles[id],
                                cameras[id].node,
                                &data, &on) != DC1394_SUCCESS)
        return false;
    
    data.bShowImg = (show ? DC1394_TRUE : DC1394_FALSE);
    
    if(avt1394_set_dsnu_blemish(handles[id],
                                cameras[id].node,
                                data) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::dsnuImgOn(unsigned int id, bool *on) {
    avt_dsnu_blemish_t data;
    dc1394bool_t state;
    
    data.bType = AVT_DSNU;
    
    if(avt1394_get_dsnu_blemish(handles[id],
                                cameras[id].node,
                                &data, &state) != DC1394_SUCCESS)
        return false;
    
    *on  = (data.bShowImg == DC1394_TRUE);
    
    return true;
}


bool DC1394::blemishImgOn(unsigned int id, bool *on) {
    avt_dsnu_blemish_t data;
    dc1394bool_t state;
    
    data.bType = AVT_BLEMISH;
    
    if(avt1394_get_dsnu_blemish(handles[id],
                                cameras[id].node,
                                &data, &state) != DC1394_SUCCESS)
        return false;
    
    *on  = (data.bShowImg == DC1394_TRUE);
    
    return true;
}


bool DC1394::dsnuComputeImg(unsigned int id) {
    avt_dsnu_blemish_t data;
    dc1394bool_t on;
    
    data.bType = AVT_DSNU;
    
    if(avt1394_get_dsnu_blemish(handles[id],
                                cameras[id].node,
                                &data, &on) != DC1394_SUCCESS)
        return false;
    
    data.bCompute = DC1394_TRUE;
    
    if(avt1394_set_dsnu_blemish(handles[id],
                                cameras[id].node,
                                data) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::blemishComputeImg(unsigned int id) {
    avt_dsnu_blemish_t data;
    dc1394bool_t on;
    
    data.bType = AVT_BLEMISH;
    
    if(avt1394_get_dsnu_blemish(handles[id],
                                cameras[id].node,
                                &data, &on) != DC1394_SUCCESS)
        return false;
    
    data.bCompute = DC1394_TRUE;
    
    if(avt1394_set_dsnu_blemish(handles[id],
                                cameras[id].node,
                                data) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::dsnuZeroImg(unsigned int id) {
    avt_dsnu_blemish_t data;
    dc1394bool_t on;
    
    data.bType = AVT_DSNU;
    
    if(avt1394_get_dsnu_blemish(handles[id],
                                cameras[id].node,
                                &data, &on) != DC1394_SUCCESS)
        return false;
    
    data.bZero = DC1394_TRUE;
    
    if(avt1394_set_dsnu_blemish(handles[id],
                                cameras[id].node,
                                data) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::blemishZeroImg(unsigned int id) {
    avt_dsnu_blemish_t data;
    dc1394bool_t on;
    
    data.bType = AVT_BLEMISH;
    
    if(avt1394_get_dsnu_blemish(handles[id],
                                cameras[id].node,
                                &data, &on) != DC1394_SUCCESS)
        return false;
    
    data.bZero = DC1394_TRUE;
    
    if(avt1394_set_dsnu_blemish(handles[id],
                                cameras[id].node,
                                data) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::dsnuLoadFactoryData(unsigned int id) {
    avt_dsnu_blemish_t data;
    dc1394bool_t on;
    
    data.bType = AVT_DSNU;
    
    if(avt1394_get_dsnu_blemish(handles[id],
                                cameras[id].node,
                                &data, &on) != DC1394_SUCCESS)
        return false;
    
    data.bLoadData = DC1394_TRUE;
    
    if(avt1394_set_dsnu_blemish(handles[id],
                                cameras[id].node,
                                data) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::blemishLoadFactoryData(unsigned int id) {
    avt_dsnu_blemish_t data;
    dc1394bool_t on;
    
    data.bType = AVT_BLEMISH;
    
    if(avt1394_get_dsnu_blemish(handles[id],
                                cameras[id].node,
                                &data, &on) != DC1394_SUCCESS)
        return false;
    
    data.bLoadData = DC1394_TRUE;
    
    if(avt1394_set_dsnu_blemish(handles[id],
                                cameras[id].node,
                                data) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::oneShotEnabled(unsigned int id, bool *on) {
    dc1394bool_t state;
    
    if(dc1394_get_one_shot(handles[id],
                           cameras[id].node,
                           &state) != DC1394_SUCCESS)
        return false;
    
    *on = state == DC1394_TRUE;
    
    return true;
}


bool DC1394::multiShotEnabled(unsigned int id, bool *on, unsigned int *frames) {
    dc1394bool_t state;
    
    if(dc1394_get_multi_shot(handles[id],
                             cameras[id].node,
                             &state, frames) != DC1394_SUCCESS)
        return false;
    
    *on = state == DC1394_TRUE;
    
    return true;
}


bool DC1394::freerunEnabled(unsigned int id, bool *on) {
    dc1394bool_t enabled;
    
    
    if(dc1394_get_iso_status(handles[id],
                             cameras[id].node,
                             &enabled) != DC1394_SUCCESS)
        return false;
    
    *on = (enabled == DC1394_TRUE);
    
    return true;
}


bool DC1394::enableOneShot(unsigned int id, bool on) {
    if(on) {
        if(dc1394_set_one_shot(handles[id],
                               cameras[id].node) != DC1394_SUCCESS)
            return false;
    }
    else {
        if(dc1394_unset_one_shot(handles[id],
                                 cameras[id].node) != DC1394_SUCCESS)
            return false;
    }
    
    return true;
}


bool DC1394::enableMultiShot(unsigned int id, bool on, unsigned int frameNumber) {
    if(on) {
        if(dc1394_set_multi_shot(handles[id],
                                 cameras[id].node,
                                 frameNumber) != DC1394_SUCCESS)
            return false;
    }
    else {
        if(dc1394_unset_multi_shot(handles[id],
                                   cameras[id].node) != DC1394_SUCCESS)
            return false;
    }
    
    return true;
}


bool DC1394::getMultiShotFrameCount(unsigned int id, unsigned int *frameCount) {
    dc1394bool_t state;
    
    if(dc1394_get_multi_shot(handles[id],
                             cameras[id].node,
                             &state, frameCount) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::enableFreerun(unsigned int id, bool on) {
    if(on) {
        if(dc1394_start_iso_transmission(handles[id],
                                         cameras[id].node) != DC1394_SUCCESS)
            return false;
    }
    else {
        if(dc1394_stop_iso_transmission(handles[id],
                                        cameras[id].node) != DC1394_SUCCESS)
            return false;
    }
    
    return true;
}


bool DC1394::autoShutterCtlAvailable(unsigned int id) {
    unsigned int minValue, maxValue;
    
    
    if(avt1394_get_auto_shutter_limits(handles[id],
                                       cameras[id].node,
                                       &minValue, &maxValue) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::getAutoShutterLimits(unsigned int id, unsigned int *minValue,
                                  unsigned int *maxValue) {
    if(avt1394_get_auto_shutter_limits(handles[id],
                                       cameras[id].node,
                                       minValue, maxValue) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::setAutoShutterLimits(unsigned int id, unsigned int minValue,
                                  unsigned int maxValue) {
    if(avt1394_set_auto_shutter_limits(handles[id],
                                       cameras[id].node,
                                       minValue, maxValue) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::autoGainCtlAvailable(unsigned int id) {
    unsigned int minValue, maxValue;
    
    
    if(avt1394_get_auto_gain_limits(handles[id],
                                    cameras[id].node,
                                    &minValue, &maxValue) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::getAutoGainLimits(unsigned int id, unsigned int *minValue,
                               unsigned int *maxValue) {
    if(avt1394_get_auto_gain_limits(handles[id],
                                    cameras[id].node,
                                    minValue, maxValue) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::setAutoGainLimits(unsigned int id, unsigned int minValue,
                               unsigned int maxValue) {
    if(avt1394_set_auto_gain_limits(handles[id],
                                    cameras[id].node,
                                    minValue, maxValue) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::autoAOIAvailable(unsigned int id) {
    unsigned int x, y, w, h;
    dc1394bool_t show, on;
    
    
    if(avt1394_get_auto_aoi_dimensions(handles[id],
                                       cameras[id].node,
                                       &x, &y, &w, &h, &show, &on) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::getAutoAOISize(unsigned int id, unsigned int *width, unsigned int *height) {
    unsigned int x, y;
    dc1394bool_t show, on;
    
    
    if(avt1394_get_auto_aoi_dimensions(handles[id],
                                       cameras[id].node,
                                       &x, &y, width, height, &show, &on) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::setAutoAOISize(unsigned int id, unsigned int width, unsigned int height) {
    unsigned int x, y, w, h;
    dc1394bool_t show, on;
    
    
    if(avt1394_get_auto_aoi_dimensions(handles[id],
                                       cameras[id].node,
                                       &x, &y, &w, &h, &show, &on) != DC1394_SUCCESS)
        return false;
    
    if(avt1394_set_auto_aoi_dimensions(handles[id],
                                       cameras[id].node,
                                       x, y, width, height) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::getAutoAOIPos(unsigned int id, unsigned int *x, unsigned int *y) {
    unsigned int w, h;
    dc1394bool_t show, on;
    
    
    if(avt1394_get_auto_aoi_dimensions(handles[id],
                                       cameras[id].node,
                                       x, y, &w, &h, &show, &on) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::setAutoAOIPos(unsigned int id, unsigned int newX, unsigned int newY) {
    unsigned int x, y, w, h;
    dc1394bool_t show, on;
    
    
    if(avt1394_get_auto_aoi_dimensions(handles[id],
                                       cameras[id].node,
                                       &x, &y, &w, &h, &show, &on) != DC1394_SUCCESS)
        return false;
    
    if(avt1394_set_auto_aoi_dimensions(handles[id],
                                       cameras[id].node,
                                       newX, newY, w, h) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::getAutoAOIStatus(unsigned int id, bool *autoAOIOn, bool *showAOIEnabled) {
    unsigned int x, y, width, height;
    dc1394bool_t on, areaShown;
    
    
    if(avt1394_get_auto_aoi_dimensions(handles[id],
                                       cameras[id].node,
                                       &x, &y, &width, &height, &areaShown, &on) != DC1394_SUCCESS)
        return false;
    
    *autoAOIOn = (on == DC1394_TRUE);
    *showAOIEnabled = (areaShown == DC1394_TRUE);
    
    return true;
}


bool DC1394::enableAutoAOI(unsigned int id, bool on) {
    if(avt1394_enable_auto_aoi(handles[id],
                               cameras[id].node,
                               on ? DC1394_TRUE : DC1394_FALSE) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::showAutoAOIArea(unsigned int id, bool show) {
    if(avt1394_enable_auto_aoi_area(handles[id],
                                    cameras[id].node,
                                    show ? DC1394_TRUE : DC1394_FALSE) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::deferredImgAvailable(unsigned int id) {
    dc1394bool_t sendImg, holdImg, fastCapture;
    unsigned int fifoSize, numImg;
    
    if(avt1394_get_deferr_trans(handles[id],
                                cameras[id].node,
                                &sendImg, &holdImg,
                                &fastCapture, &fifoSize,
                                &numImg) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::getDeferredImgStatus(unsigned int id, bool *status) {
    dc1394bool_t sendImg, holdImg, fastCapture;
    unsigned int fifoSize, numImg;
    
    if(avt1394_get_deferr_trans(handles[id],
                                cameras[id].node,
                                &sendImg, &holdImg,
                                &fastCapture, &fifoSize,
                                &numImg) != DC1394_SUCCESS)
        return false;
    
    *status = (holdImg == DC1394_TRUE);
    
    return true;
}


bool DC1394::enableDeferredImg(unsigned int id, bool enable) {
    dc1394bool_t sendImg, holdImg, fastCapture;
    unsigned int fifoSize, numImg;
    
    if(avt1394_get_deferr_trans(handles[id],
                                cameras[id].node,
                                &sendImg, &holdImg,
                                &fastCapture, &fifoSize,
                                &numImg) != DC1394_SUCCESS)
        return false;
    
    if(avt1394_set_deferr_trans(handles[id],
                                cameras[id].node,
                                sendImg,
                                (enable ? DC1394_TRUE : DC1394_FALSE),
                                fastCapture,
                                numImg) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::deferredImgSend(unsigned int id, unsigned int frameCount) {
    dc1394bool_t sendImg, holdImg, fastCapture;
    unsigned int fifoSize, numImg;
    
    if(avt1394_get_deferr_trans(handles[id],
                                cameras[id].node,
                                &sendImg, &holdImg,
                                &fastCapture, &fifoSize,
                                &numImg) != DC1394_SUCCESS)
        return false;
    
    if(avt1394_set_deferr_trans(handles[id],
                                cameras[id].node,
                                DC1394_TRUE,
                                holdImg,
                                fastCapture, frameCount) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::deferredImgSendActive(unsigned int id, bool *sendActive) {
    dc1394bool_t sendImg, holdImg, fastCapture;
    unsigned int fifoSize, numImg;
    
    if(avt1394_get_deferr_trans(handles[id],
                                cameras[id].node,
                                &sendImg, &holdImg,
                                &fastCapture, &fifoSize,
                                &numImg) != DC1394_SUCCESS)
        return false;
    
    *sendActive = (sendImg == DC1394_TRUE);
    
    return true;
}

bool DC1394::deferredImgFastCaptureEnable(unsigned int id, bool enable) {
    dc1394bool_t sendImg, holdImg, fastCapture;
    unsigned int fifoSize, numImg;
    
    if(avt1394_get_deferr_trans(handles[id],
                                cameras[id].node,
                                &sendImg, &holdImg,
                                &fastCapture, &fifoSize,
                                &numImg) != DC1394_SUCCESS)
        return false;
    
    if(avt1394_set_deferr_trans(handles[id],
                                cameras[id].node,
                                sendImg, holdImg,
                                (enable ? DC1394_TRUE : DC1394_FALSE),
                                numImg) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::deferredImgFastCaptureStatus(unsigned int id, bool *on) {
    dc1394bool_t sendImg, holdImg, fastCapture;
    unsigned int fifoSize, numImg;
    
    if(avt1394_get_deferr_trans(handles[id],
                                cameras[id].node,
                                &sendImg, &holdImg,
                                &fastCapture, &fifoSize,
                                &numImg) != DC1394_SUCCESS)
        return false;
    
    *on = (fastCapture == DC1394_TRUE);
    
    return true;
}


bool DC1394::getDeferredImgFifoSize(unsigned int id, unsigned int *size) {
    dc1394bool_t sendImg, holdImg, fastCapture;
    unsigned int numImg;
    
    if(avt1394_get_deferr_trans(handles[id],
                                cameras[id].node,
                                &sendImg, &holdImg,
                                &fastCapture, size,
                                &numImg) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::setDeferredImgFrameCount(unsigned int id, unsigned int number) {
    dc1394bool_t sendImg, holdImg, fastCapture;
    unsigned int fifoSize, numImg;
    
    if(avt1394_get_deferr_trans(handles[id],
                                cameras[id].node,
                                &sendImg, &holdImg,
                                &fastCapture, &fifoSize,
                                &numImg) != DC1394_SUCCESS)
        return false;
    
    if(avt1394_set_deferr_trans(handles[id],
                                cameras[id].node,
                                sendImg, holdImg,
                                fastCapture, number) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::getDeferredImgFrameCount(unsigned int id, unsigned int *number) {
    dc1394bool_t sendImg, holdImg, fastCapture;
    unsigned int fifoSize;
    
    if(avt1394_get_deferr_trans(handles[id],
                                cameras[id].node,
                                &sendImg, &holdImg,
                                &fastCapture, &fifoSize,
                                number) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::frameInfoAvailable(unsigned int id) {
    unsigned int counter;
    
    
    if(avt1394_get_frame_info(handles[id],
                              cameras[id].node,
                              &counter) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::frameInfoReset(unsigned int id) {
    if(avt1394_reset_frame_counter(handles[id],
                                   cameras[id].node) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::getFrameInfoCount(unsigned int id, unsigned int *count) {
    if(avt1394_get_frame_info(handles[id],
                              cameras[id].node,
                              count) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::delIntAvailable(unsigned int id) {
    unsigned int delay;
    dc1394bool_t on;
    
    
    if(avt1394_get_int_delay(handles[id],
                             cameras[id].node,
                             &delay, &on) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::delIntStatus(unsigned int id, bool *status) {
    unsigned int delay;
    dc1394bool_t on;
    
    
    if(avt1394_get_int_delay(handles[id],
                             cameras[id].node,
                             &delay, &on) != DC1394_SUCCESS)
        return false;
    
    *status = (on == DC1394_TRUE);
    
    return true;
}


bool DC1394::delIntEnable(unsigned int id, bool enable) {
    if(avt1394_enable_int_delay(handles[id],
                                cameras[id].node,
                                (enable ? DC1394_TRUE : DC1394_FALSE)) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::getDelIntValue(unsigned int id, unsigned int *value) {
    dc1394bool_t on;
    
    
    if(avt1394_get_int_delay(handles[id],
                             cameras[id].node,
                             value, &on) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::setDelIntValue(unsigned int id, unsigned int value) {
    if(avt1394_set_int_delay(handles[id],
                             cameras[id].node,
                             value) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::incDecAvailable(unsigned int id) {
    dc1394bool_t on;
    unsigned int counter, cmp;
    
    
    if(avt1394_get_io_decoder(handles[id],
                              cameras[id].node,
                              &counter, &cmp, &on) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::incDecStatus(unsigned int id, bool *status) {
    dc1394bool_t on;
    unsigned int counter, cmp;
    
    
    if(avt1394_get_io_decoder(handles[id],
                              cameras[id].node,
                              &counter, &cmp, &on) != DC1394_SUCCESS)
        return false;
    
    *status = (on == DC1394_SUCCESS);
    
    return true;
}


bool DC1394::incDecEnable(unsigned int id, bool enable) {
    if(avt1394_enable_io_decoder(handles[id],
                                 cameras[id].node,
                                 (enable ? DC1394_TRUE : DC1394_FALSE)) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::getIncDecCompare(unsigned int id, unsigned int *value) {
    dc1394bool_t on;
    unsigned int count;
    
    
    if(avt1394_get_io_decoder(handles[id],
                              cameras[id].node,
                              &count, value, &on) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::setIncDecCompare(unsigned int id, unsigned int value) {
    if(avt1394_set_io_decoder(handles[id],
                              cameras[id].node,
                              value) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::getIncDecCounter(unsigned int id, unsigned int *value) {
    dc1394bool_t on;
    unsigned int cmp;
    
    
    if(avt1394_get_io_decoder(handles[id],
                              cameras[id].node,
                              value, &cmp, &on) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::incDecReset(unsigned int id) {
    if(avt1394_reset_io_decoder_counter(handles[id],
                                        cameras[id].node) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::softResetAvailable(unsigned int id) {
    unsigned int tmpValue;
    
    
    if(avt1394_get_soft_reset_delay(handles[id],
                                    cameras[id].node, &tmpValue) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::getSoftResetDelay(unsigned int id, unsigned int *delay) {
    if(avt1394_get_soft_reset_delay(handles[id],
                                    cameras[id].node, delay) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::setSoftResetDelay(unsigned int id, unsigned int delay) {
    if(avt1394_set_soft_reset_delay(handles[id],
                                    cameras[id].node, delay) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::softReset(unsigned int id) {
    if(avt1394_soft_reset(handles[id], cameras[id].node) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::delayedSoftReset(unsigned int id, unsigned int delay) {
    if(avt1394_delayed_soft_reset(handles[id], cameras[id].node, delay) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::highSNRAvailable(unsigned int id) {
    unsigned int grabCount;
    dc1394bool_t enable;
    
    
    if(avt1394_get_high_snr_info(handles[id],
                                 cameras[id].node,
                                 &grabCount,
                                 &enable) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::highSNRStatus(unsigned int id, unsigned int *grabCount, bool *enabled) {
    dc1394bool_t enable;
    
    
    if(avt1394_get_high_snr_info(handles[id],
                                 cameras[id].node,
                                 grabCount,
                                 &enable) != DC1394_SUCCESS)
        return false;
    
    *enabled = (enable == DC1394_TRUE);
    
    return true;
}


bool DC1394::highSNREnable(unsigned int id, bool enable) {
    if(avt1394_high_snr_enable(handles[id],
                               cameras[id].node,
                               enable ? DC1394_TRUE : DC1394_FALSE) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::setHighSNRGrabCount(unsigned int id, unsigned int grabCount) {
    if(avt1394_set_high_snr_grab_count(handles[id],
                                       cameras[id].node,
                                       grabCount) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::seqCtlAvailable(unsigned int id) {
    unsigned int maxLength;
    dc1394bool_t apply, on;
    
    
    if(avt1394_get_seq_info(handles[id],
                            cameras[id].node,
                            &maxLength,
                            &apply,
                            &on) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::seqCtlStatus(unsigned int id, bool *status, bool *autoRewind, bool *autoInc) {
    unsigned int maxLength, seqLength, imageNo;
    dc1394bool_t apply, on, aR, aI;
    
    
    if(avt1394_get_seq_info(handles[id],
                            cameras[id].node,
                            &maxLength,
                            &apply,
                            &on) != DC1394_SUCCESS)
        return false;
    
    if(avt1394_get_seq_param(handles[id],
                             cameras[id].node,
                             &aR,
                             &seqLength,
                             &aI,
                             &imageNo) != DC1394_SUCCESS)
        return false;
    
    *status = (on == DC1394_TRUE);
    *autoRewind = (aR == DC1394_TRUE);
    *autoInc = (aI == DC1394_TRUE);
    
    return true;
}


bool DC1394::seqCtlEnable(unsigned int id, bool enable) {
    if(avt1394_enable_sequence(handles[id],
                               cameras[id].node,
                               enable ? DC1394_TRUE : DC1394_FALSE) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::seqCtlEnableAutoRewind(unsigned int id, bool enable) {
    unsigned int seqLength, imageNo;
    dc1394bool_t aR, aI;
    
    
    if(avt1394_get_seq_param(handles[id],
                             cameras[id].node,
                             &aR,
                             &seqLength,
                             &aI,
                             &imageNo) != DC1394_SUCCESS)
        return false;
    
    if(avt1394_set_seq_param(handles[id],
                             cameras[id].node,
                             enable ? DC1394_TRUE : DC1394_FALSE,
                             seqLength,
                             aI,
                             imageNo) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::seqCtlEnableAutoInc(unsigned int id, bool enable) {
    unsigned int seqLength, imageNo;
    dc1394bool_t aR, aI;
    
    
    if(avt1394_get_seq_param(handles[id],
                             cameras[id].node,
                             &aR,
                             &seqLength,
                             &aI,
                             &imageNo) != DC1394_SUCCESS)
        return false;
    
    if(avt1394_set_seq_param(handles[id],
                             cameras[id].node,
                             aR,
                             seqLength,
                             enable ? DC1394_TRUE : DC1394_FALSE,
                             imageNo) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::getSeqCtlInfo(unsigned int id, unsigned int *seqLimit, unsigned int *seqLength,
                           unsigned int *imgNo) {
    dc1394bool_t apply, on, aR, aI;
    
    
    if(avt1394_get_seq_info(handles[id],
                            cameras[id].node,
                            seqLimit,
                            &apply,
                            &on) != DC1394_SUCCESS)
        return false;
    
    if(avt1394_get_seq_param(handles[id],
                             cameras[id].node,
                             &aR,
                             seqLength,
                             &aI,
                             imgNo) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::setSeqCtlImageNo(unsigned int id, unsigned int imgNo) {
    unsigned int imageNo, seqLength;
    dc1394bool_t aR, aI;
    
    
    if(avt1394_get_seq_param(handles[id],
                             cameras[id].node,
                             &aR,
                             &seqLength,
                             &aI,
                             &imageNo) != DC1394_SUCCESS)
        return false;
    
    if(avt1394_set_seq_param(handles[id],
                             cameras[id].node,
                             aR,
                             seqLength,
                             aI,
                             imgNo) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::setSeqCtlLength(unsigned int id, unsigned int length) {
    unsigned int imageNo, seqLength;
    dc1394bool_t aR, aI;
    
    
    if(avt1394_get_seq_param(handles[id],
                             cameras[id].node,
                             &aR,
                             &seqLength,
                             &aI,
                             &imageNo) != DC1394_SUCCESS)
        return false;
    
    if(avt1394_set_seq_param(handles[id],
                             cameras[id].node,
                             aR,
                             length,
                             aI,
                             imageNo) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::seqCtlApply(unsigned int id, bool autoInc) {
    if(avt1394_apply_seq_param(handles[id],
                               cameras[id].node,
                               autoInc ? DC1394_TRUE : DC1394_FALSE) != DC1394_SUCCESS)
        return false;
    
    return true;
}


bool DC1394::seqCtlBusy(unsigned int id, bool *busy) {
    unsigned int maxLength;
    dc1394bool_t apply, on;
    
    
    if(avt1394_get_seq_info(handles[id],
                            cameras[id].node,
                            &maxLength,
                            &apply,
                            &on) != DC1394_SUCCESS)
        return false;
    
    *busy = (apply == DC1394_TRUE);
    
    return true;
}



// protected functions

void DC1394::setWidthAndHeight(unsigned int id) {
    x[id] = 0;
    y[id] = 0;
    
    switch(mode[id]) {
        // Format 0
    case MODE_160x120_YUV444:
        width[id] = 160;
        height[id] = 120;
        break;
    case MODE_320x240_YUV422:
        width[id] = 320;
        height[id] = 240;
        break;
    case MODE_640x480_YUV411:
    case MODE_640x480_YUV422:
    case MODE_640x480_RGB:
    case MODE_640x480_MONO:
    case MODE_640x480_MONO16:
        width[id] = 640;
        height[id] = 480;
        break;
        // Format 1
    case MODE_800x600_YUV422:
    case MODE_800x600_RGB:
    case MODE_800x600_MONO:
    case MODE_800x600_MONO16:
        width[id] = 800;
        height[id] = 600;
        break;
    case MODE_1024x768_YUV422:
    case MODE_1024x768_RGB:
    case MODE_1024x768_MONO:
    case MODE_1024x768_MONO16:
        width[id] = 1024;
        height[id] = 768;
        break;
        // Format 2
    case MODE_1280x960_YUV422:
    case MODE_1280x960_RGB:
    case MODE_1280x960_MONO:
    case MODE_1280x960_MONO16:
        width[id] = 1280;
        height[id] = 960;
        break;
    case MODE_1600x1200_YUV422:
    case MODE_1600x1200_RGB:
    case MODE_1600x1200_MONO:
    case MODE_1600x1200_MONO16:
        width[id] = 1600;
        height[id] = 1200;
        break;
        // Format 7
    case MODE_FORMAT7_0:
    case MODE_FORMAT7_1:
    case MODE_FORMAT7_2:
    case MODE_FORMAT7_3:        
    case MODE_FORMAT7_4:
    case MODE_FORMAT7_5:
    case MODE_FORMAT7_6:
    case MODE_FORMAT7_7:
        if(dc1394_query_format7_image_size(handles[id],
                                           cameras[id].node,
                                           mode[id],
                                           &width[id], &height[id]) != DC1394_SUCCESS) {
            logger->log("'DC1394::setWidthAndHeight': Query F7 Image Size failed!");
            width[id] = 0;
            height[id] = 0;
        }
        else {
            if((width[id] == 0) || (height[id] == 0)) {
                logger->log("'DC1394::setWidthAndHeight': Either width or height or both are '0' - trying to query max. size.");
                if(dc1394_query_format7_max_image_size(handles[id],
                                                       cameras[id].node,
                                                       mode[id],
                                                       &width[id], &height[id]) != DC1394_SUCCESS) {
                    logger->log("'DC1394::setWidthAndHeight': Querying max. size for mode %d failed!",
                                mode[id]);
                }
            }
            else {
                if(dc1394_query_format7_image_position(handles[id],
                                                       cameras[id].node,
                                                       mode[id],
                                                       &x[id], &y[id]) != DC1394_SUCCESS) {
                    logger->log("'DC1394::setWidthAndHeight': Querying current image position failed!");
                }
            }
        }
        break;
    default:
        // Format 6
        logger->log("'DC1394::setWidthAndHeight': Unsupported format - setting both width and height to '0'!");
        width[id] = 0;
        height[id] = 0;
        break;
    }
}

