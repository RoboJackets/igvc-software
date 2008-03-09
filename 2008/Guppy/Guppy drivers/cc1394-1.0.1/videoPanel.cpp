/*
  videoPanel.cpp
  
  implementation of the VideoPanel class
  
  25.8.04 G.Glock
*/

#include "CamWindow.h"
#include "videoPanel.h"

#include <qapplication.h>
#include <qpainter.h>
#include <qevent.h>
#include <qrect.h>


VideoPanel::VideoPanel(DC1394 *dc, unsigned int i, unsigned int hOff, unsigned int vOff,
                       QWidget *p, const char * name, WFlags f) : QWidget(p, name, f) {
    parent = (CamWindow *)p;
    dc1394 = dc;
    id = i;
    hOffset = hOff;
    vOffset = vOff;
    
    
    format = dc1394->getFormat(id);
    res = dc1394->getMode(id);
    deviceWidth = dc1394->getWidth(id);
    deviceHeight = dc1394->getHeight(id);
    if(format <= FORMAT_SVGA_NONCOMPRESSED_2) {
        fps = dc1394->getFramerate(id);
        colorCodingID = 0;
    }
    else {
        fps = 0;
        colorCodingID = dc1394->getF7ColorCodingID(id, res);
    }
    
    videoImage = new QImage(deviceWidth, deviceHeight, 32);
    videoImage->setAlphaBuffer(false);
    
    threadExit = false;
    threadInterrupt = false;
    threadRunning = false;
    geometryUpdate = false;
    f7GeometryUpdate = false;
    captureModeChanged = false;
    f7BPPChanged = false;
    updateUI = false;
    
    captureMode = CAPTURE_MODE_FREERUN;
    
    setGeometry(0, 0, deviceWidth, deviceHeight);
}


VideoPanel::~VideoPanel() {
    delete videoImage;
}


void VideoPanel::stop() {
    parent->log("'videoPanel::stop': stopping display-thread");
    threadExit = true;
}


void VideoPanel::terminateDMA() {
    dc1394->stopIsoTransmission(id);
    
    dc1394->dmaRelease(id);
}


bool VideoPanel::stopCapture() {
    unsigned int waitCount = 0;
    
    if(!threadRunning)
        return true;
    
    parent->log("'videoPanel::stopCapture': interrupting video capture");
    threadInterrupt = true;
    
    while(threadRunning && (waitCount < 5)) {
        sleep(1);
        waitCount ++;
    }
    
    if(threadRunning)
        threadInterrupt = false;
    
    parent->log("'videoPanel::stopCapture': video capture %s",
                (threadRunning ? "still running" : "interrupted"));
    
    return(!threadRunning);
}


bool VideoPanel::startCapture() {
    unsigned int waitCount = 0;
    
    if(threadRunning)
        return true;
    
    parent->log("'videoPanel::startCapture': starting video capture");
    threadInterrupt = false;
    
    while(!threadRunning && (waitCount < 5)) {
        sleep(1);
        waitCount ++;
    }
    
    if(!threadRunning)
        threadInterrupt = true;
    
    parent->log("'videoPanel::startCapture': video capture %s",
                (threadRunning ? "running" : "still interrupted"));
    
    return(threadRunning);
}


void VideoPanel::updateSize(unsigned int f, unsigned int m, unsigned int fr, unsigned int cc) {
    qApp->lock();
    
    newFormat = f;
    newRes = m;
    newFps = fr;
    newColorCoding = cc;
    
    geometryUpdate = true;
    
    parent->log("'videoPanel::updateSize': New Geometry Parameters - Format: %d, Mode: %d, Resolution: %dx%d",
                newFormat, newRes, newDeviceWidth, newDeviceHeight);
    
    parent->log("'videoPanel::updateSize': DC1394::getWidth(): %d , DC1394::getHeight(): %d",
                dc1394->getWidth(id), dc1394->getHeight(id));
    
    qApp->unlock();
}


void VideoPanel::updateF7Size(unsigned int width, unsigned int height,
                            unsigned int x, unsigned int y) {
    qApp->lock();
    
    parent->log("'videoPanel::updateF7Size': Updating F7 Size and Position to 'X = %d, Y = %d / %d x %d'",
                x, y, width, height);
    
    newF7Width = width;
    newF7Height = height;
    newF7X = x;
    newF7Y = y;
    
    f7GeometryUpdate = true;
    
    qApp->unlock();
}


void VideoPanel::updateF7BPP() {
    f7BPPChanged = true;
}


void VideoPanel::getFrameCountUpdateInfos() {
}


void VideoPanel::multiShot(unsigned int frameCount) {
    captureMode = CAPTURE_MODE_MULTISHOT;
    captureFrameCount = frameCount;
    captureModeChanged = true;
}


void VideoPanel::oneShot() {
    captureMode = CAPTURE_MODE_ONESHOT;
    captureModeChanged = true;
}


void VideoPanel::freerun() {
    captureMode = CAPTURE_MODE_FREERUN;
    captureModeChanged = true;
}


void VideoPanel::run() {
    QRect rect;
    QPaintEvent *pe;
    VPanelCloseEvent *closeEvent = new VPanelCloseEvent();
    UpdateF7Event *updateEvent;
    
    bool success = true;
    bool repaint = true; // repaint whole region during first loop
    
    parent->log("'videoPanel::run': Video-Stream-Thread started");
    
    threadRunning = true;
    threadInterrupt = false;
    
    // Main Video-Thread-Loop
    while(!threadExit) {
        // Video-Stream stopped (e.g. for changing the TIMEBASE register)
        if(threadInterrupt) {
            if(threadRunning) {
                parent->log("'videoPanel::run': Video-Thread interrupted - stopping transmission");
                dc1394->stopIsoTransmission(id);
                dc1394->dmaRelease(id);
                
                threadRunning = false;
            }
            
            sleep(1);
            continue;
        }
        else {
            if(!threadRunning) {
                parent->log("'videoPanel::run': Thread restarted");
                if(format <= FORMAT_SVGA_NONCOMPRESSED_2)
                    dc1394->dmaSetup(id);
                if(format == FORMAT_SCALABLE_IMAGE_SIZE)
                    dc1394->dmaF7Setup(id);
                dc1394->startIsoTransmission(id);
                
                threadRunning = true;
            }
        }
        
        // Geometry has changed - recreate the videoImage
        if(geometryUpdate || f7GeometryUpdate || f7BPPChanged) {
            if(!updateGeometry(&repaint)) {
                success = false;
                parent->log("'videoPanel::run': 'updateGeometry()' failed!");
                break;
            }
        }
        
        // capture frame
        if(!dc1394->dmaSingleCapture(id)) {
            parent->log("'videoPanel::run': 'dma_single_capture()' failed!");
            dc1394->dmaReleaseBuffer(id);
            continue;
        }
        
        if(updateUI) {
            updateEvent = new UpdateF7Event();
            QApplication::postEvent(parent, updateEvent);
            
            ((CamWindow *)parent)->updateStatusBar();
            
            geometryUpdate = false;
            f7GeometryUpdate = false;
            f7BPPChanged = false;
            updateUI = false;
        }
        
        switch(format) {
        case FORMAT_VGA_NONCOMPRESSED: // F0
        case FORMAT_SVGA_NONCOMPRESSED_1: // F1
        case FORMAT_SVGA_NONCOMPRESSED_2: // F2
            convertFormat0_2();
            
            rect = QRect(0, 0, deviceWidth, deviceHeight);
            
            if(!threadExit) {
                pe = new QPaintEvent(rect, repaint);
                QApplication::postEvent((QWidget *)this, pe);
                
                // repaint whole area only once (after init or geometry-change)
                if(repaint)
                    repaint = false;
            }
            
            break;
            
        case FORMAT_STILL_IMAGE: // F6
            // not supported by AVT cameras - therefore not yet implemented
            break;
            
        case FORMAT_SCALABLE_IMAGE_SIZE: // F7
            convertFormat7();
            
            rect = QRect(0, 0, deviceWidth, deviceHeight);
            
            if(!threadExit) {
                pe = new QPaintEvent(rect, repaint);
                QApplication::postEvent((QWidget *)this, pe);
                
                // repaint whole area only once (after init or geometry-change)
                if(repaint)
                    repaint = false;
            }
            
            break;
            
        default: // no further formats known
            break;
        }
        
        if(!dc1394->dmaReleaseBuffer(id)) {
            parent->log("'videoPanel::run': 'dmaReleaseBuffer()' failed at the end of the 'Thread-Loop'!");
            continue;
        }
    } // while(!threadExit)
    
    threadRunning = false;
    
    parent->log("'videoPanel::run': Thread terminated %s",
                (success ? "successfully" : "abnormally"));
    
    if(success) {
        // stop ISO transmission
        if(!dc1394->stopIsoTransmission(id))
            parent->log("'videoPanel::run': Stopping ISO transmission failed!");
        
        // release DMA
        if(!dc1394->dmaRelease(id))
            parent->log("'videoPanel': 'dmaRelease()' failed!");
    }
    else {
        QApplication::postEvent(parent, closeEvent);
    }
} // run()


bool VideoPanel::updateGeometry(bool *repaint) {
    qApp->lock();
    
    parent->log("'videoPanel::updateGeometry': Geometry-Update requested...");
    
    if(!dc1394->stopIsoTransmission(id)) {
        parent->log("'videoPanel::updateGeometry': Error while stopping ISO-Transmission for 'geometry-update'!");
        return false;
    }
    if(!dc1394->dmaRelease(id)) {
        parent->log("'videoPanel::updateGeometry': Error with 'dmaRelease()' for 'geometry-update'!");
        return false;
    }
    
    delete videoImage;
    videoImage = NULL;
    
    if(geometryUpdate) {
        if(!dc1394->setParameters(id, newFormat, newRes, newFps, newColorCoding)) {
            parent->log("'videoPanel::updateGeometry': Changing geometry failed!");
            return false;
        }
    }
    
    if(f7GeometryUpdate) {
        if(!dc1394->setF7ImageSize(id, newF7Width, newF7Height)) {
            parent->log("'videoPanel::updateGeometry': Writing new F7 Image Size to camera failed!");
            return false;
        }
        
        if(!dc1394->setF7ImagePos(id, newF7X, newF7Y)) {
            parent->log("'videoPanel::updateGeometry': Writing new F7 Image Pos to camera failed!");
            return false;
        }
    }
    
    // use the new geometry data
    format = dc1394->getFormat(id);
    res = dc1394->getMode(id);
    deviceWidth = dc1394->getWidth(id);
    deviceHeight = dc1394->getHeight(id);

    if(format <= FORMAT_SVGA_NONCOMPRESSED_2) {
        fps = dc1394->getFramerate(id);
        colorCodingID = 0;
    }
    else {
        fps = 0;
        colorCodingID = dc1394->getF7ColorCodingID(id, res);
    }
    
    // recreate the videoImage
    videoImage = new QImage(deviceWidth, deviceHeight, 32);
    videoImage->setAlphaBuffer(false);
    
    setGeometry(0, 0, deviceWidth, deviceHeight);
    parent->setGeometry(0, 0, deviceWidth + hOffset, deviceHeight + vOffset);
    
    if(format == FORMAT_SCALABLE_IMAGE_SIZE) {
        if(!dc1394->dmaF7Setup(id)) {
            parent->log("'videoPanel::updateGeometry': 'dmaF7Setup()' failed after 'geometry-update'!");
            return false;
        }
    }
    else {
        if(!dc1394->dmaSetup(id)) {
            parent->log("'videoPanel::updateGeometry': 'dmaSetup()' failed after 'geometry-update'!");
            return false;
        }
    }
    
    if(!dc1394->startIsoTransmission(id)) {
        parent->log("'videoPanel::updateGeometry': 'startIsoTransmission()' failed after 'geometry-update'!");
        dc1394->dmaRelease(id);
        return false;
    }
    
    *repaint = true; // repaint whole region during first loop of changed geometry
    
    qApp->unlock();
    
    updateUI = true;
    
    return true;
}


bool VideoPanel::changeCaptureMode() {
    bool currentISOState, currentOneShotState, currentMultiShotState;
    unsigned int msFrameCount;
    
    // get current capture-mode-states
    if(!dc1394->getIsoStatus(id, &currentISOState)) {
        parent->log("'videoPanel::changeCaptureMode': Reading current ISO state failed!");
        return false;
    }
    if(!dc1394->oneShotEnabled(id, &currentOneShotState)) {
        parent->log("'videoPanel::changeCaptureMode': Reading current OneShot state failed!");
        return false;
    }
    if(!dc1394->multiShotEnabled(id, &currentMultiShotState, &msFrameCount)) {
        parent->log("'videoPanel::changeCaptureMode': Reading current MultiShot sate failed!");
        return false;
    }
    
    // switch to requested capture-mode
    switch(captureMode) {
    case CAPTURE_MODE_FREERUN:
        // if ISO transmission is stopped
        if(!currentISOState) {
            // start iso-transmission
            if(!dc1394->startIsoTransmission(id)) {
                parent->log("'videoPanel::changeCaptureMode': Starting ISO transmission failed!");
                return false;
            }
        }
        break;
        
    case CAPTURE_MODE_ONESHOT:
        // if ISO transmission is running
        if(currentISOState) {
            // stop ISO transmission
            if(!dc1394->stopIsoTransmission(id)) {
                parent->log("'videoPanel::changeCaptureMode': Stopping ISO transmission failed!");
                return false;
            }
        }
        // if multi-shot is enabled
        if(currentMultiShotState) {
            // disable multi-shot
            if(!dc1394->enableMultiShot(id, false, 0)) {
                parent->log("'videoPanel::changeCaptureMode': Disabling MultiShot failed!");
                return false;
            }
        }
        // if one-shot is not enabled
        if(!currentOneShotState) {
            if(!dc1394->enableOneShot(id, true)) {
                parent->log("'videoPanel::changeCaptureMode': Enabling OneShot failed!");
                return false;
            }
        }
        // enable one-shot
        break;
        
    case CAPTURE_MODE_MULTISHOT:
        if(currentISOState) {
            if(!dc1394->stopIsoTransmission(id)) {
                parent->log("'videoPanel::changeCaptureMode': Stopping ISO transmission failed!");
                return false;
            }
        }
        
        if(currentOneShotState) {
            if(!dc1394->enableOneShot(id, false)) {
                parent->log("'videoPanel::changeCaptureMode': Disabling OneShot failed!");
                return false;
            }
        }
        
        if(!currentMultiShotState) {
            if(!dc1394->enableMultiShot(id, true, captureFrameCount)) {
                parent->log("'videoPanel::changeCaptureMode': Enabling MultiShot failed!");
                return false;
            }
        }
        break;
        
    default:
        return false;
    }   
    
    captureModeChanged = false;
    return true;
}


void VideoPanel::convertFormat0_2() {
    switch(res) {
    case MODE_640x480_YUV411:
        convert411ToQImage(videoImage, (const void*)dc1394->getCaptureBuffer(id));
        break;
        
    case MODE_320x240_YUV422:
    case MODE_640x480_YUV422:
    case MODE_800x600_YUV422:
    case MODE_1024x768_YUV422:
    case MODE_1280x960_YUV422:
    case MODE_1600x1200_YUV422:
        convert422ToQImage(videoImage, (const void*)dc1394->getCaptureBuffer(id));
        break;
        
    case MODE_640x480_RGB:
    case MODE_800x600_RGB:
    case MODE_1024x768_RGB:
    case MODE_1280x960_RGB:
    case MODE_1600x1200_RGB:
        // should be something like this (but cannot be tested, because of lacking camera)
        // memcpy((void *)(videoImage)->scanLine(0), (const void*)dc1394.capture_buffer, deviceWidth * deviceHeight);
        break;
        
    case MODE_640x480_MONO:
    case MODE_800x600_MONO:
    case MODE_1024x768_MONO:
    case MODE_1280x960_MONO:
    case MODE_1600x1200_MONO:
        convert400ToQImage(videoImage, (const void*)dc1394->getCaptureBuffer(id), 8);
        break;
        
    case MODE_640x480_MONO16:
    case MODE_800x600_MONO16:
    case MODE_1024x768_MONO16:
    case MODE_1280x960_MONO16:
    case MODE_1600x1200_MONO16:
        convert400ToQImage(videoImage, (const void*)dc1394->getCaptureBuffer(id), 16);
        break;
        
    default:
        break;
    }
    
}


void VideoPanel::convertFormat7() {
    switch(colorCodingID) {
    case COLOR_FORMAT7_YUV411:
        convert411ToQImage(videoImage, (const void*)dc1394->getCaptureBuffer(id));
        break;
        
    case COLOR_FORMAT7_YUV422:
        convert422ToQImage(videoImage, (const void*)dc1394->getCaptureBuffer(id));
        break;
        
    case COLOR_FORMAT7_YUV444:
        // this resolution is not yet supported
        break;
        
    case COLOR_FORMAT7_RGB8:
        // should be something like this (but cannot be tested, because of lacking camera)
        // memcpy((void *)(videoImage)->scanLine(0), (const void*)dc1394.capture_buffer, deviceWidth * deviceHeight);
        break;
        
    case COLOR_FORMAT7_RGB16:
        // should be something like this (but cannot be tested, because of lacking camera)
        // memcpy((void *)(videoImage)->scanLine(0), (const void*)dc1394.capture_buffer, deviceWidth * deviceHeight);
        break;
        
    case COLOR_FORMAT7_MONO8:
        convert400ToQImage(videoImage, (const void*)dc1394->getCaptureBuffer(id), 8);
        break;
        
    case COLOR_FORMAT7_MONO16:
        convert400ToQImage(videoImage, (const void*)dc1394->getCaptureBuffer(id), 16);
        break;
        
    default:
        break;
    }
    
}


void VideoPanel::paintEvent(QPaintEvent *) {
    QPainter p(this);
    
    p.drawImage(0, 0, *videoImage);
    
}


// YUV 4:2:2 / 16 bpp
void VideoPanel::convert422ToQImage(QImage *image, const void *capture_buffer) {
    for(int lineCount = 0; lineCount < image->height(); lineCount++) {
        convert422LinetoRGB(&((unsigned char*)capture_buffer)[lineCount * image->width() * 2],
                            (RGBAlphaPixel *)image->scanLine(lineCount), image->width());
    }
    
}


// YUV 4:1:1 / 12 bpp
void VideoPanel::convert411ToQImage(QImage *image, const void *capture_buffer) {
    for(int lineCount = 0; lineCount < image->height(); lineCount++) {
        convert411LinetoRGB(&((unsigned char*)capture_buffer)[lineCount * image->width() * 3/2],
                            (RGBAlphaPixel *)image->scanLine(lineCount), image->width());
    }
    
}


// YUV 4:0:0 / 8/16 bpp
void VideoPanel::convert400ToQImage(QImage *image, const void *capture_buffer,
                                    unsigned int bpp) {
    for(int lineCount = 0; lineCount < image->height(); lineCount++) {
        convert400LinetoRGB(&((unsigned char*)capture_buffer)[lineCount * image->width() *
                                                              (bpp / 8)],
                            (RGBAlphaPixel *)image->scanLine(lineCount), image->width(), bpp);
    }
    
}


// Line converters to RGB (8 Bit Version)

void VideoPanel::convert422LinetoRGB(unsigned char *ycbcrBuffer,
                                     RGBAlphaPixel *rgbaBuffer ,
                                     unsigned int numPixels) {
    YCbCrAlphaPixel ycbcrPixel;
    unsigned short Cb1, Y1, Cr1, Cb2, Y2, Cr2;
    unsigned int count;
    
    
    // convert a 4:2:2 line (CbYCrYCbYCrY....) to RGBAlphaPixels
    // 2 RGBAlphaPixels at a time.
    
    Cb1 = *ycbcrBuffer++;
    Y1 = *ycbcrBuffer++;
    Cr1 = *ycbcrBuffer++;
    
    for(count = 0; count < (numPixels - 2); count += 2) {
        ycbcrPixel.cb = (unsigned char)Cb1;
        ycbcrPixel.y = (unsigned char)Y1;
        ycbcrPixel.cr = (unsigned char)Cr1;
        
        SDConvertYCbCrtoRGB(&ycbcrPixel, &rgbaBuffer[count]);
        
        // Read lone midde Y;
        ycbcrPixel.y = *ycbcrBuffer++;
        
        // Read Next full bandwidth sample
        Cb2 = *ycbcrBuffer++;
        Y2 = *ycbcrBuffer++;
        Cr2 = *ycbcrBuffer++;
        
        // Interpolate and write Inpterpolated RGBAlphaPixel
        ycbcrPixel.cb = (unsigned char)((Cb1 + Cb2) / 2);
        ycbcrPixel.cr = (unsigned char)((Cr1 + Cr2) / 2);
        
        SDConvertYCbCrtoRGB(&ycbcrPixel, &rgbaBuffer[count + 1]);
        
        // Setup for next loop
        Cb1 = Cb2;
        Cr1 = Cr2;
        Y1 = Y2;
    }
    
    // conversion of the last two samples can only use one Cb/Cr value (there is no 'next' one!)
    ycbcrPixel.cb = (unsigned char)Cb1;
    ycbcrPixel.y = (unsigned char)Y1;
    ycbcrPixel.cr = (unsigned char)Cr1;
    
    SDConvertYCbCrtoRGB(&ycbcrPixel, &rgbaBuffer[count]);
    
    // Read lone midde Y;
    ycbcrPixel.y = *ycbcrBuffer++;
    
    // Interpolate and write Inpterpolated RGBAlphaPixel
    ycbcrPixel.cb = (unsigned char)(Cb1 / 2);
    ycbcrPixel.cr = (unsigned char)(Cr1 / 2);
    
    SDConvertYCbCrtoRGB(&ycbcrPixel, &rgbaBuffer[count + 1]);
    
}


void VideoPanel::convert411LinetoRGB(unsigned char *ycbcrBuffer,
                                     RGBAlphaPixel *rgbaBuffer ,
                                     unsigned int numPixels) {
    YCbCrAlphaPixel ycbcrPixel;
    unsigned short Cb1, Y1, Y2, Cr1, Y3, Y4, Cb2, Y5, Cr2;
    unsigned int count;
    
    
    // convert a 4:1:1 line (CbYYCrYYCbYYCrYY....) to RGBAlphaPixels
    // 4 RGBAlphaPixels at a time.
    
    Cb1 = *ycbcrBuffer++;
    Y1 = *ycbcrBuffer++;
    Y2 = *ycbcrBuffer++;
    Cr1 = *ycbcrBuffer++;
    
    for(count = 0; count < (numPixels - 4); count += 4) {
        ycbcrPixel.cb = (unsigned char)Cb1;
        ycbcrPixel.y = (unsigned char)Y1;
        ycbcrPixel.cr = (unsigned char)Cr1;
        
        // convert 1st Sample
        SDConvertYCbCrtoRGB(&ycbcrPixel, &rgbaBuffer[count]);
        
        // use 2nd Y of 1st Sample
        ycbcrPixel.y = (unsigned char)Y2;
        
        // convert 2nd Sample
        SDConvertYCbCrtoRGB(&ycbcrPixel, &rgbaBuffer[count +1]);
        
        // use first lone midde Y
        ycbcrPixel.y = *ycbcrBuffer++;
        
        // read second lone middle Y
        Y3 = *ycbcrBuffer++;
        
        // Read Next full bandwidth sample
        Cb2 = *ycbcrBuffer++;
        Y4 = *ycbcrBuffer++;
        Y5 = *ycbcrBuffer++;
        Cr2 = *ycbcrBuffer++;
        
        // Interpolate and write Inpterpolated RGBAlphaPixel
        ycbcrPixel.cb = (unsigned char)((Cb1 + Cb2) / 2);
        ycbcrPixel.cr = (unsigned char)((Cr1 + Cr2) / 2);
        
        // convert 3rd Sample
        SDConvertYCbCrtoRGB(&ycbcrPixel, &rgbaBuffer[count + 2]);
        
        // use 2nd lone middle Y
        ycbcrPixel.y = (unsigned char)Y3;
        
        // convert 4th Sample
        SDConvertYCbCrtoRGB(&ycbcrPixel, &rgbaBuffer[count + 3]);
        
        // Setup for next loop
        Cb1 = Cb2;
        Cr1 = Cr2;
        Y1 = Y4;
        Y2 = Y5;
    }
    
    // The last samples are not alowed to read the 'next sample values' ahead!
    ycbcrPixel.cb = (unsigned char)Cb1;
    ycbcrPixel.y = (unsigned char)Y1;
    ycbcrPixel.cr = (unsigned char)Cr1;
    
    // convert 1st Sample
    SDConvertYCbCrtoRGB(&ycbcrPixel, &rgbaBuffer[count]);
    
    // use 2nd Y of 1st Sample
    ycbcrPixel.y = (unsigned char)Y2;
    
    // convert 2nd Sample
    SDConvertYCbCrtoRGB(&ycbcrPixel, &rgbaBuffer[count +1]);
    
    // use first lone midde Y
    ycbcrPixel.y = *ycbcrBuffer++;
    
    // read second lone middle Y
    Y3 = *ycbcrBuffer++;
    
    // Interpolate and write Inpterpolated RGBAlphaPixel
    ycbcrPixel.cb = (unsigned char)(Cb1 / 2);
    ycbcrPixel.cr = (unsigned char)(Cr1 / 2);
    
    // convert 3rd Sample
    SDConvertYCbCrtoRGB(&ycbcrPixel, &rgbaBuffer[count + 2]);
    
    // use 2nd lone middle Y
    ycbcrPixel.y = (unsigned char)Y3;
    
    // convert 4th Sample
    SDConvertYCbCrtoRGB(&ycbcrPixel, &rgbaBuffer[count + 3]);
}


void VideoPanel::convert400LinetoRGB(unsigned char *ycbcrBuffer,
                                     RGBAlphaPixel *rgbaBuffer ,
                                     unsigned int numPixels,
                                     unsigned int bpp) {
    
    // convert a 4:0:0 line (YYY....) to RGBAlphaPixels
    // 1 RGBAlphaPixel at a time.
    for(unsigned int count = 0; count < numPixels; count++) {
        (&rgbaBuffer[count])->Red = *ycbcrBuffer;
        (&rgbaBuffer[count])->Green = *ycbcrBuffer;
        (&rgbaBuffer[count])->Blue = *ycbcrBuffer;
        ycbcrBuffer += (bpp / 8);
    }
}

