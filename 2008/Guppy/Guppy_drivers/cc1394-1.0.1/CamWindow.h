/*
  CamWindow.h
  
  CamWindow-class for displaying a camera-image (MDI-Window)
  
  31.8.04 G.Glock
*/

#ifndef CAMWINDOW_H
#define CAMWINDOW_H

#include <qapplication.h>
#include <qwidget.h>
#include <qevent.h>
#include <qscrollview.h>
#include <qpopupmenu.h>
#include <qcursor.h>
#include <qstatusbar.h>
#include <qlayout.h>
#include <qfileinfo.h>

#include <stdio.h>
#include <stdarg.h>

#include <libdc1394/dc1394_control.h>

#include "adjustmentsform.h"
#include "seqctldemoform.h"
#include "caminfoform.h"
#include "constants.h"
#include "CamWindowCloseEvent.h"
#include "videoPanel.h"
#include "DC1394.h"
#include "VPanelCloseEvent.h"
#include "F7NotifyData.h"
#include "F7UpdateNotifyEvent.h"
#include "CustomEvents.h"
#include "messagelogger.h"
#include "SoftResetDelay.h"
#include "drmform.h"


#define POPUPMENU_ADJ 0
#define POPUPMENU_INFO 1
#define POPUPMENU_CLOSE 2
#define POPUPMENU_INIT 3
#define POPUPMENU_SEQCTL 4
#define POPUPMENU_DRM 5

#define NO_DIALOG 0
#define ADJUSTMENTS_DIALOG 1
#define SEQ_CTL_DIALOG 2


// external declarations ('pre-definition'-declarations)
class AdjustmentsForm;
class SeqCtlDemoForm;


class CamWindow : public QWidget {
    Q_OBJECT
public:
    CamWindow(int id, DC1394 *dc1394, unsigned int hOffset, unsigned int vOffset,
              MessageLogger *ml, const char *caption, QWidget *parent = 0,
              QWidget *mainWindow = 0, const char *name = 0, WFlags flags = 0);
    void enableAbsolute(bool abs);
    void updateSize(unsigned int format, unsigned int mode, unsigned int framerate,
                    unsigned int colorCoding);
    void updateF7Size(unsigned int width, unsigned int height, unsigned int x, unsigned int y);
    void updateStatusBar();
    unsigned int getID();
    void execAdjustments();
    void execCamInfos();
    void execDRM();
    void execCameraReset();
    void execSeqCtlDemo();
    
    void customEvent(QCustomEvent *e);
    
    bool stopCapture();
    bool startCapture();
    
    void getFrameCountUpdateInfos();
    
    void multiShot(unsigned int frameCount);
    void oneShot();
    void freerun();
    
    void updateAdjustments();
    
    void log(const char *msg, ...);
    
    void enableIO3(bool enable);
    
protected:
    void mousePressEvent(QMouseEvent *e);
    void closeEvent(QCloseEvent *e);
    void displayCameraInfos();
    void disableUIControl();
    void enableUIControl();
    
    dc1394_camerainfo camInfo;
    quadlet_t swVersion;
    quadlet_t revision;
    
    QWidget *mainWindow;
    int id;
    DC1394 *dc1394;
    MessageLogger *logger;
    VideoPanel *vPanel;
    QBoxLayout *layout;
    QScrollView *scrollView;
    QStatusBar *statusBar;
    AdjustmentsForm *adjustments;
    SeqCtlDemoForm *seqCtlDemo;
    bool seqCtlAvailable;
    unsigned int openDialog;
    
    unsigned int captureMode;
    unsigned int captureFrameCount;
    
    unsigned int hOffset, vOffset;
    
    bool vPanelError;
    
    DRMForm drm;
    
};


#endif // CAMWINDOW_H
