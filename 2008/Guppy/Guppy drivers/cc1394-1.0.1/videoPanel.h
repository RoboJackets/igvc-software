/*
  videoPanel.h
  
  class declaration for a video panel
  
  25.8.04 G.Glock
*/

#ifndef VIDEOPANEL_H
#define VIDEOPANEL_H

#include "constants.h"
#include "DC1394.h"
#include "UpdateF7Event.h"
#include "VPanelCloseEvent.h"

#include <libdc1394/dc1394_control.h>

#include <qwidget.h>
#include <qthread.h>
#include <qevent.h>
#include <qimage.h>



#define ClipRGB_8BIT(X) ((X) > MAX_RGB_8BIT ? (MAX_RGB_8BIT) : ((X) < MIN_RGB_8BIT ? (MIN_RGB_8BIT) : (X)))



extern "C" {
    typedef struct {
        unsigned char Blue;
        unsigned char Green;
        unsigned char Red;
        unsigned char Alpha;
    } RGBAlphaPixel;
    
    
    typedef struct {
        unsigned char Red;
        unsigned char Green;
        unsigned char Blue;
    } RGBPixel;
    
    
    typedef struct {
        unsigned char cb;
        unsigned char y;
        unsigned char cr;
    } YCbCrPixel;
    
    
    
    typedef struct {
        unsigned char Alpha;
        unsigned char cr;
        unsigned char y;
        unsigned char cb;
    } YCbCrAlphaPixel;
    
}


class CamWindow;


class VideoPanel : public QWidget, public QThread {
    Q_OBJECT
public:
    VideoPanel(DC1394 *c, unsigned int id, unsigned int hOffset, unsigned int vOffset,
               QWidget * parent = 0, const char * name = 0, WFlags f = 0);
    ~VideoPanel();
    
    void stop();
    
    void terminateDMA();
    
    bool stopCapture();
    bool startCapture();
    
    void updateSize(unsigned int format, unsigned int mode, unsigned int framerate,
                    unsigned int colorCoding);
    void updateF7Size(unsigned int width, unsigned int height, unsigned int x, unsigned int y);
    void updateF7BPP();
    
    void getFrameCountUpdateInfos();
    
    void multiShot(unsigned int frameCount);
    void oneShot();
    void freerun();
    
protected:
    CamWindow *parent;
    bool threadExit, threadInterrupt, threadRunning;
    bool geometryUpdate, f7GeometryUpdate, captureModeChanged;
    bool f7BPPChanged;
    bool updateUI;
    unsigned int captureMode, captureFrameCount;
    DC1394 *dc1394;
    unsigned int id;
    unsigned int hOffset, vOffset;
    QImage *videoImage;
    unsigned int deviceWidth, deviceHeight, res, fps, format, colorCodingID;
    unsigned int newDeviceWidth, newDeviceHeight, newRes, newFps, newFormat;
    unsigned int newF7Width, newF7Height, newF7X, newF7Y;
    unsigned int newColorCoding;
    
    virtual void run();
    
    bool updateGeometry(bool *repaint);
    bool changeCaptureMode();
    
    void convertFormat0_2();
    void convertFormat7();
    
    virtual void paintEvent(QPaintEvent *);
    
    void convert422ToQImage(QImage *image, const void *capture_buffer);
    void convert411ToQImage(QImage *image, const void *capture_buffer);
    void convert400ToQImage(QImage *image, const void *capture_buffer, unsigned int bpp);
    
    void convert422LinetoRGB(unsigned char *ycbcrBuffer, RGBAlphaPixel *rgbaBuffer ,
                             unsigned int numPixels);
    void convert411LinetoRGB(unsigned char *ycbcrBuffer, RGBAlphaPixel *rgbaBuffer ,
                             unsigned int numPixels);
    void convert400LinetoRGB(unsigned char *ycbcrBuffer, RGBAlphaPixel *rgbaBuffer ,
                             unsigned int numPixels, unsigned int bpp);
    
    inline void SDConvertYCbCrtoRGB(YCbCrAlphaPixel *pSource,
                                    RGBAlphaPixel *pTarget) {
        
        register int Red,Green,Blue;
        register long ConvertedY;
        
        ConvertedY = 0x12A15*((int)pSource->y - CCIR601_8BIT_BLACK);
        
        Red = FixedRound(ConvertedY +
                         0x19895*((int)(pSource->cr-CCIR601_8BIT_CHROMAOFFSET)));
        
        pTarget->Red = (unsigned char)ClipRGB_8BIT(Red);
        
        Blue = FixedRound(ConvertedY +
                          0x20469*((int)(pSource->cb-CCIR601_8BIT_CHROMAOFFSET) ));
        
        pTarget->Blue = (unsigned char)ClipRGB_8BIT(Blue);
        
        Green = FixedRound(ConvertedY - 
                           0x644A*((int)(pSource->cb-CCIR601_8BIT_CHROMAOFFSET) ) -
                           0xD01F*((int)(pSource->cr-CCIR601_8BIT_CHROMAOFFSET) ));
        
        pTarget->Green = (unsigned char)ClipRGB_8BIT(Green);
        
        pTarget->Alpha = pSource->Alpha;
    };
    
    inline short FixedRound(long inFix) { 
        register short retValue;
        
        if(inFix < 0 ) {
            retValue = -((-inFix + 0x8000) >> 16);
        }
        else {
            retValue = (inFix + 0x8000) >> 16;
        }
        return retValue;
    };
    
};


#endif // VIDEOPANEL_H
