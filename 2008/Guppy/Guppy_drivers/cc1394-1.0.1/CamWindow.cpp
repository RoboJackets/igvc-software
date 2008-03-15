/*
  CamWindow.cpp
  
  Implementation of the 'CamWindow' class
  
  31.8.04 G.Glock
*/


#include "adjustmentsform.h"
#include "CamWindow.h"
#include "mainform.h"


CamWindow::CamWindow(int i, DC1394 *dc, unsigned int hOff, unsigned int vOff,
                     MessageLogger *ml, const char *caption, QWidget *p, QWidget* m,
                     const char *n, WFlags f) : QWidget(p, n, f) {
    id = i;
    dc1394 = dc;
    hOffset = hOff;
    vOffset = vOff;
    logger = ml;
    
    
    // determine the current capture state of the camera
    if(!dc1394->getCurrentCaptureMode(id, &captureMode, &captureFrameCount)) {
        logger->log("Error while trying to determine the current CaptureMode!\n");
        return;
    }
    
    // save Main-Window-reference for event-posting purposes
    mainWindow = m;
    
    setCaption(caption);
    
    // create a statusbar for this camera-window
    statusBar = new QStatusBar(this);
    statusBar->setSizeGripEnabled(true);
    
    layout = new QBoxLayout(this, QBoxLayout::TopToBottom);
    
    // determine the current capture state of the camera
    if(!dc1394->getCurrentCaptureMode(id, &captureMode, &captureFrameCount)) {
        logger->log("Error while trying to determine the current CaptureMode!\n");
        return;
    }
    
    // create a videoPanel for displaying the video-stream
    vPanel = new VideoPanel(dc1394, id, hOffset, vOffset, this);
    // starting the videoPanel-Thread
    vPanelError = false;
    vPanel->start();
    
    // create a scrollView
    scrollView = new QScrollView(this);
    // adding 'video-output' as the child of this scroll-view
    scrollView->addChild(vPanel);
    
    updateStatusBar();
    
    // insert scrollView and statusBar into the layout
    layout->addWidget(scrollView);
    layout->addWidget(statusBar);
    
    // initial geometry is set to the initial video-format
    setGeometry(0, 0,
                dc1394->getWidth(id) + hOffset,
                dc1394->getHeight(id) + vOffset);
    
    //    setMaximumSize(dc1394->getWidth(id) + 4,
    //  (dc1394->getHeight(id)) +
    //  statusBar->height() - 5);
    
    
    // create the adjustmentform-object
    adjustments = new AdjustmentsForm();
    // change the caption accordingly
    adjustments->setCaption(QString("Picture Control for Camera ") + this->caption());
    adjustments->setFixedSize(adjustments->frameSize());
    
    // and initialize it
    adjustments->initAdjustments(this, dc1394, id, false, logger);
    
    // sequence control demo dialog
    if(dc1394->seqCtlAvailable(id)) {
        seqCtlDemo = new SeqCtlDemoForm();
        seqCtlDemo->setCaption(QString("Seqence Control Demo for Camera ") + this->caption());
        seqCtlDemo->setFixedSize(seqCtlDemo->frameSize());
        
        seqCtlDemo->initSeqCtl(this, dc1394, id, false, logger);
        
        seqCtlAvailable = true;
    }
    else {
        seqCtlAvailable = false;
    }
    
    openDialog = NO_DIALOG;
} // CamWindow::CamWindow()


void CamWindow::enableAbsolute(bool abs) {
    adjustments->initAdjustments(this, dc1394, id, abs, logger);
}


void CamWindow::updateSize(unsigned int format, unsigned int mode, unsigned int framerate,
                           unsigned int colorCoding) {
    vPanel->updateSize(format, mode, framerate, colorCoding);
    updateStatusBar();
}


void CamWindow::updateF7Size(unsigned int width, unsigned int height,
                             unsigned int x, unsigned int y) {
    vPanel->updateF7Size(width, height, x, y);
}


void CamWindow::updateStatusBar() {
    unsigned int format;
    QString statusMessage;
    
    format = dc1394->getFormat(id);
    
    switch(format) {
    case FORMAT_STILL_IMAGE:
        statusMessage = QString("");
        break;
        
    case FORMAT_SCALABLE_IMAGE_SIZE:
        statusMessage = QString::number(dc1394->getWidth(id)) +
                        "x" + QString::number(dc1394->getHeight(id)) + " (" +
                        QString::number(dc1394->getX(id)) + ", " +
                        QString::number(dc1394->getY(id)) + ") " +
                        dc1394->getModeString(id) + " " +
                        QString::number(dc1394->getF7FPS(id)) + "fps";
        break;
        
    default:
        statusMessage = QString::number(dc1394->getWidth(id)) +
                        "x" + QString::number(dc1394->getHeight(id)) + " " +
                        dc1394->getModeString(id) +
                        " (" +  dc1394->getFramerateString(id) + " fps)";
        break;
    }
    
    statusBar->message(statusMessage);
}


unsigned int CamWindow::getID() {
    return id;
}


void CamWindow::execAdjustments() {
    disableUIControl();
    // 'adjustments' dialog has to be 'show()'n (opened 'non-modal')
    adjustments->show();
    
    openDialog = ADJUSTMENTS_DIALOG;
}


void CamWindow::execCamInfos() {
    // show 'Camera-Infos' Dialog
    displayCameraInfos();
}


void CamWindow::execDRM() {
    // 'exec()'ute 'Direct Register Manipulation' Dialog
    drm.setFixedSize(drm.frameSize());
    drm.setParameters(id, dc1394, logger);
    drm.exec();
}


void CamWindow::execCameraReset() {
    SoftResetDelay softResetDelayDialog;
    unsigned int delay;
    
    
    if(!dc1394->softResetAvailable(id)) {
        // 'SoftReset' not available - try to use 'Standard-Initialization'
        
        // show Warning Dialog
        if(QMessageBox::warning(this,
                                "Init Camera",
                                "This will set the camera to the initial factory settings\nAre you sure?",
                                QMessageBox::Yes,
                                QMessageBox::No | QMessageBox::Default | QMessageBox::Escape,
                                QMessageBox::NoButton) == QMessageBox::Yes) {
            
            QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
            
            // stop iso-transmission
            if(!stopCapture()) {
                QApplication::restoreOverrideCursor();
                logger->log("Error while trying to stop capturing!\n");
                return;
            }
            
            // try to initialize camera via the standard 'INITIALITZE' register
            if(!dc1394->dc1394InitCamera(id)) {
                logger->log("Error while trying to initialize the camera!");
            }
            
            // wait for 1 second
            sleep(1);
            
            // restart iso-transmission
            if(!startCapture()) {
                logger->log("Error while trying to 'restart' capturing after 'Camera-Reset'!");
            }
            
            QApplication::restoreOverrideCursor();
        }
    }
    else {
        // 'SoftReset' available
        
        // get current value...
        if(!dc1394->getSoftResetDelay(id, &delay)) {
            logger->log("Error while trying to read current 'SoftResetDelay' value!");
            return;
        }
        
        // ...and set it in the dialog
        softResetDelayDialog.setValue(delay);
        
        // warning and query for 'SoftReset'-specific delay
        if(softResetDelayDialog.exec() == QDialog::Accepted) {
            QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
            
            // get the new delay-value from the dialog
            delay = softResetDelayDialog.getValue() / 10;
            
            // and propagate it to the camera
            if(!dc1394->setSoftResetDelay(id, delay)) {
                QApplication::restoreOverrideCursor();
                logger->log("Error while trying to set new SoftResetDelay value!");
                return;
            }
            
            // stop iso-transmission
            if(!stopCapture()) {
                QApplication::restoreOverrideCursor();
                logger->log("Error while trying to stop capturing!");
                return;
            }
            
            // execute 'SoftReset'
            if(!dc1394->softReset(id)) {
                logger->log("Error while trying to 'SoftReset' the camera");
            }
            
            // wait for 'SoftReset' (1s  at least)
            sleep((delay < 100) ? 1 : ((delay / 1000) +1));
            
            // restart iso-transmission
            if(!startCapture()) {
                logger->log("Error while trying to 'restart' capturing after 'Camera-Reset'!");
            }
            
            QApplication::restoreOverrideCursor();
        }
    }
}


void CamWindow::execSeqCtlDemo() {
    logger->log("starting 'Sequence Control Dialog' for camera %d", id);
    disableUIControl();
    
    // open SeqCtlDemo dialog
    seqCtlDemo->show();
    
    openDialog = SEQ_CTL_DIALOG;
}


void CamWindow::customEvent(QCustomEvent *e) {
    unsigned int width, height, x, y;
    F7UpdateNotifyEvent *notifyEvent = (F7UpdateNotifyEvent *)e;
    
    switch(e->type()) {
    case VPANELCLOSE:
        logger->log("Video-Stream from camera seems to be stopped because of an error - therefore closing the Camera window!");
        vPanelError = true;
        close();
        break;
        
        // videoPanel -> CamWindow -> 'Adjustments' Dialog
    case UPDATEF7:
        // 'vPanel' ready with 'F7-changes'
        switch(openDialog) {
        case ADJUSTMENTS_DIALOG:
            adjustments->enableUpdateF7();
            break;
        case SEQ_CTL_DIALOG:
            seqCtlDemo->enableUpdateF7();
            break;
        default:
            break;
        }
        break;
        
    case ADJUSTMENTSCLOSE:
        // enable 'normal control' (MainWindow) after closing the 'Adjustments' dialog
        enableUIControl();
        openDialog = NO_DIALOG;
        break;
        
        // 'Adjustments' Dialog -> CamWindow -> videoPanel
    case F7UPDATENOTIFY:
        logger->log("'F7-Update-Notification' received...");
        width = (notifyEvent->geometry()).width;
        height = (notifyEvent->geometry()).height;
        x = (notifyEvent->geometry()).x;
        y = (notifyEvent->geometry()).y;
        if((width == 0) || (height == 0)) {
            QMessageBox::warning(this, "Format 7 Size",
                                 "Wrong window size!\nEither 'width', 'height' or both are '0'!\nSource of Error: " + (notifyEvent->geometry()).sender,
                                 QMessageBox::Ok | QMessageBox::Default | QMessageBox::Escape,
                                 QMessageBox::NoButton,
                                 QMessageBox::NoButton);
            
            logger->log("Warning! Wrong window size (either width, height, or both are zero!");
            logger->log("\tSize: %d x %d / Position: %d, %d", width, height, x, y);
            logger->log("Sender: %s", ((notifyEvent->geometry()).sender).ascii());
            
            break;
        }
        logger->log("Received new 'F7-Geometry': %dx%d / %d, %d", width, height, x, y);
        vPanel->updateF7Size(width, height, x, y);
        break;
        
        // 'Adjustments' Dialog -> CamWindow -> videoPanel
    case F7BPPCHANGED:
        logger->log("'F7-BPP-Changed-Notification' received...");
        vPanel->updateF7BPP();
        break;
        
    default:
        logger->log("Info - Unknown Event received (%d)!", e->type());
    }
}


bool CamWindow::stopCapture() {
    return vPanel->stopCapture();
}


bool CamWindow::startCapture() {
    return vPanel->startCapture();
}


void CamWindow::getFrameCountUpdateInfos() {
    vPanel->getFrameCountUpdateInfos();
}


void CamWindow::multiShot(unsigned int frameCount) {
    vPanel->multiShot(frameCount);
}


void CamWindow::oneShot() {
    vPanel->oneShot();
}


void CamWindow::freerun() {
    vPanel->freerun();
}


void CamWindow::updateAdjustments() {
    switch(openDialog) {
    case ADJUSTMENTS_DIALOG:
        adjustments->enableUpdateF7();
        break;
    case SEQ_CTL_DIALOG:
        seqCtlDemo->enableUpdateF7();
        break;
    default:
        break;
    }
}


// 'log()' 'transfers' logging-messages from the 'camera-windows' to the 'logger'.
// This function is necessary, because an 'elipsis'-parameter cannot be tranfered!
// Therefore the message will be 'parsed' to a fix 'QString'.
void CamWindow::log(const char *msg, ...) {
    QString message = QString("");
    va_list ap;
    const char *p, *sval;
    int ival;
    
    
    va_start(ap, msg);
    for(p = msg; *p; p++) {
        if(*p != '%') {
            message += *p;
            continue;
        }
        
        switch(*++p) {
        case 'd':
            ival = va_arg(ap, int);
            message += QString::number(ival);
            break;
        case 'x':
            ival = va_arg(ap, int);
            message += QString::number(ival, 16);
            break;
        case 's':
            for(sval = va_arg(ap, char *); *sval; sval++)
                message += *sval;
            break;
        default:
            message += *p;
            break;
        }
    }
    
    va_end(ap);
    
    logger->log(message);
}


void CamWindow::enableIO3(bool enable) {
    adjustments->enableIO3(enable);
}



// protected

void CamWindow::closeEvent(QCloseEvent *e) {
    bool triggerState;
    bool stopSuccess = false;
    bool on;
    
    
    // close open dialogs
    if(adjustments->isShown()) {
        e->ignore();
        return;
//        adjustments->close();
//        enableUIControl();
    }
    if(seqCtlDemo->isShown()) {
        e->ignore();
        return;
//        seqCtlDemo->close();
//        enableUIControl();
    }
    
    // if Trigger is enabled - warning!
    if(!dc1394->getTriggerState(id, &triggerState)) {
        logger->log("Error! Reading current trigger state failed - ignoring the 'closeEvent'!\n");
        e->ignore();
        return;
    }
    
    if(triggerState) {
        switch(QMessageBox::warning(this, "Trigger active", "Trigger still active!",
                                    "Deactivate", "Exit anyway", "Cancel", 0, 1)) {
        case 0:
            if(!dc1394->setFeatureOn(id, FEATURE_TRIGGER - FEATURE_MIN, false)) {
                logger->log("Error! Trigger can't be stopped - therefore ignoring the 'closeEvent'!");
                e->ignore();
                return;
            }
            break;
        case 1:
            break;
        default:
            e->ignore();
            return;
        }
    }
    
    if(!dc1394->freerunEnabled(id, &on)) {
        logger->log("Error while trying to read current ISO-state!");
        e->ignore();
        return;
    }
    
    if(!on) {
        if(!dc1394->enableFreerun(id, true)) {
            logger->log("Error while trying to enable ISO-Transmission for 'window-closing'!");
            e->ignore();
            return;
        }
    }
        
        
    CamWindowCloseEvent *event = new CamWindowCloseEvent();
    event->setData(&id);
    
    if(!vPanelError) {
        vPanel->stop();
        
        logger->log("waiting on vPanel-Thread to exit...");
        QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
        stopSuccess = vPanel->wait(5000);
        QApplication::restoreOverrideCursor();
        
        if(!stopSuccess) {
            logger->log("Timeout while waiting on Thread to exit...\nnow trying to 'terminate' the Thread");
            vPanel->terminate();
            QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
            stopSuccess = vPanel->wait(5000);
            QApplication::restoreOverrideCursor();
            
            if(!stopSuccess)
                logger->log("\t...this also fails...");
            
            vPanel->terminateDMA();
        }
    }
    else
        logger->log("vPanel Thread ended abnormally - 'ISO-tranmission' is stopped");
    
    delete vPanel;
    
    QApplication::postEvent(mainWindow, event);
    
    e->accept();
}


void CamWindow::mousePressEvent(QMouseEvent *e) {
    QPopupMenu *menu = new QPopupMenu(this);
    int id;
    
    menu->insertItem("Picture Control...", POPUPMENU_ADJ);
    // insert 'sequence control' menu entry only if this feature is available
    if(seqCtlAvailable)
        menu->insertItem("Sequence Control Demo...", POPUPMENU_SEQCTL);
    menu->insertItem("Factory Settings...", POPUPMENU_INIT);
    menu->insertSeparator();
    menu->insertItem("Camera-Infos...", POPUPMENU_INFO);
    menu->insertSeparator();
    menu->insertItem("Direct Register Manipulation...", POPUPMENU_DRM);
    menu->insertSeparator();
    menu->insertItem("Close", POPUPMENU_CLOSE);
    
    // right mouse button leads to a context-menu (related to the displayed camera)
    if(e->button() == RightButton) {
        id = menu->exec(QCursor::pos());
        switch(id) {
        case POPUPMENU_ADJ:
            // open the adjustments-dialog (which will control the camera-adjustments itself)
            execAdjustments();
            break;
        case POPUPMENU_SEQCTL:
            execSeqCtlDemo();
            break;
        case POPUPMENU_INIT:
            execCameraReset();
            break;
        case POPUPMENU_INFO:
            // read camera-specific information
            // and display them in a little dialog
            displayCameraInfos();
            break;
        case POPUPMENU_DRM:
            // display 'Direct Register Manipulation' Dialog
            execDRM();
            break;
        case POPUPMENU_CLOSE:
            close();
            break;
        default:
            logger->log("Info - %d selected", id);
        }
    }
}

void CamWindow::displayCameraInfos() {
    CamInfoForm *infos = new CamInfoForm();
    int swVersion = 0;
    quadlet_t revision = 0;
    unsigned int uCMajor = 0, uCMinor = 0;
    unsigned int fwCamID = 0, fwMajor = 0, fwMinor = 0;
    bool marlin = false;
    QFileInfo fInfo;
    
    dc1394->getSWVersion(id, &swVersion);
    dc1394->getRevision(id, &revision);
    dc1394->getuCVersion(id, &uCMajor, &uCMinor);
    dc1394->getFirmwareVersion(id, &fwCamID, &fwMajor, &fwMinor, &marlin);
    
    infos->setParameters(dc1394->getCameraVendor(id),
                         dc1394->getCameraModel(id),
                         dc1394->getEUID(id),
                         dc1394->getCCROffset(id),
                         swVersion, revision, uCMajor, uCMinor, fwCamID,
                         fwMajor, fwMinor,
                         dc1394->getPortNo(id),
                         dc1394->getNodeNo(id));
    
    infos->setFixedSize(infos->frameSize());
    
    infos->exec();
}


void CamWindow::disableUIControl() {
    ((mainForm *)mainWindow)->disableUIControl();
    
    this->setEnabled(false);
}


void CamWindow::enableUIControl() {
    ((mainForm *)mainWindow)->enableUIControl();
    
    this->setEnabled(true);
}


