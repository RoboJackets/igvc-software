/****************************************************************************
** ui.h extension file, included from the uic-generated form implementation.
**
** If you want to add, delete, or rename functions or slots, use
** Qt Designer to update this file, preserving your code.
**
** You should not define a constructor or destructor in this file.
** Instead, write your code in functions called init() and destroy().
** These will automatically be called by the form's constructor and
** destructor.
*****************************************************************************/


void mainForm::fileOpen() {
    
}


void mainForm::fileSave() {
    
}


void mainForm::fileSaveAs() {
    
}


void mainForm::filePrint() {
    
}


void mainForm::fileExit() {
    close();
}


void mainForm::helpIndex() {
    
}


void mainForm::helpContents() {
    
}



// opens a 'CamWindow' for camera 'id'
bool mainForm::openCameraWindow(int id) {
    unsigned int hOffset, vOffset;
    
    
    // camera-menu entries have always positive ids
    if(id >= 0 && id < MAX_CAMERAS) {
        logger.log("camera %d about to open\n", id);
        
        if(!cameraOpen[id]) {
            logger.log("trying to open camera with Format %d (%d), Mode %d, Framerate %d",
                   selectedFormatItemID,
                   selectedFormatItemID + FORMAT_MIN,
                   selectedResItemID,
                   selectedFpsItemID);

            // get the current 'global' camera-parameters (framerate, size, mode...) from the menu(s)
            if(!dc1394->setParameters(id, selectedFormatItemID +
                                      FORMAT_MIN,
                                      selectedResItemID, selectedFpsItemID,
                                      selectedColorCodingItemID)) {
                logger.log("Error! Setting DC1394-Port-Parameters failed!\n");
                return false;
            }
            
            // initialize DMA (with these parameters)
            if(selectedFormatItemID <= 2) {
                logger.log("\n\tStarting 'normal' DMA...");
                if(!dc1394->dmaSetup(id)) {
                    QMessageBox::warning(this, "cc1394",
                                         "Error while trying to setup a camera-connection!\nPlease check if the 'video1394'-module is loaded,\nthe permissions are sufficient,\nor another application using it is (still) running!",
                                         QMessageBox::Ok | QMessageBox::Default,
                                         QMessageBox::NoButton,
                                         QMessageBox::NoButton);
                    return false;
                }
            }
            
            if(selectedFormatItemID == 7) {
                logger.log("\n\tStarting 'F7' DMA...");
                if(!dc1394->dmaF7Setup(id)) {
                    QMessageBox::warning(this, "cc1394",
                                         "Error while trying to setup a camera-connection!\nPlease check if the 'video1394'-module is loaded,\nthe permissions are sufficient,\nor another application using it is (still) running!",
                                         QMessageBox::Ok | QMessageBox::Default,
                                         QMessageBox::NoButton,
                                         QMessageBox::NoButton);
                    return false;
                }
            }
            
            // start ISO transmission
            if(!dc1394->startIsoTransmission(id)) {
                dc1394->dmaRelease(id);
                logger.log("Error! 'startIsoTransmission()' failed!\n");
                return false;
            }
            
/*
            logger.log("\n\tMainWindow-FrameGeometry: (%d, %x) %dx%d \n\tMainWindow-Geometry: (%d, %d) %dx%d\n",
                   frameGeometry().x(), frameGeometry().y(),
                   frameGeometry().width(), frameGeometry().height(),
                   geometry().x(), geometry().y(),
                   geometry().width(), geometry().height());
            
            logger.log("\tStatusBar-FrameGeometry: (%d, %d) %dx%d",
                   statusBar()->frameGeometry().x(), statusBar()->frameGeometry().y(),
                   statusBar()->frameGeometry().width(), statusBar()->frameGeometry().height());
            
            logger.log("\tStatusBar-Geometry: (%d, %d) %dx%d\n",
                   statusBar()->geometry().x(), statusBar()->geometry().y(),
                   statusBar()->geometry().width(), statusBar()->geometry().height());
            
            logger.log("\tWorkspace-FrameGeometry: (%d, %d) %dx%d",
                   workspace->frameGeometry().x(), workspace->frameGeometry().y(),
                   workspace->frameGeometry().width(),
                   workspace->frameGeometry().height());
            
            logger.log("\tWorkspace-Geometry: (%d, %d) %dx%d\n",
                   workspace->geometry().x(), workspace->geometry().y(),
                   workspace->geometry().width(), workspace->geometry().height());
            
            logger.log("\tMainMenuBar-FrameGeometry: (%d, %d) %dx%d",
                   mainMenuBar->frameGeometry().x(), mainMenuBar->frameGeometry().y(),
                   mainMenuBar->frameGeometry().width(),
                   mainMenuBar->frameGeometry().height());
            
            logger.log("\tMainMenuBar-Geometry: (%d, %d) %dx%d\n",
                   mainMenuBar->geometry().x(), mainMenuBar->geometry().y(),
                   mainMenuBar->geometry().width(), mainMenuBar->geometry().height());
*/
            
            hOffset = frameGeometry().width() - geometry().width();
            vOffset = frameGeometry().height() - geometry().height();
            
            // create a new camera-window
            cw[id] = new CamWindow(id, dc1394, hOffset, vOffset, &logger, Cameras->text(id),
                                   workspace, this);
            // ...and open it
            cw[id]->show();
            
/*
            logger.log("\n\tCameraWindow-Geometry: (%d, %d) %dx%d",
                   cw[id]->geometry().x(), cw[id]->geometry().y(),
                   cw[id]->geometry().width(), cw[id]->geometry().height());
            
            logger.log("\tCameraWindow-FrameGeometry: (%d, %d) %dx%d\n",
                   cw[id]->frameGeometry().x(), cw[id]->frameGeometry().y(),
                   cw[id]->frameGeometry().width(), cw[id]->frameGeometry().height());
*/
            
            cameraOpen[id] = true;
            
            // increment open-cameras-counter
            openCameras++;
            
            // at least one camera is opened - so 'close all' does makes sense
            camerasClose_All_CamerasAction->setEnabled(true);
            camerasISO_SpeedAction->setEnabled(true);
            adjustmentsPicture_ControlAction->setEnabled(true);
            adjustmentsI_O3Action->setEnabled(true);
            adjustmentsDirect_Register_ManipulationAction->setEnabled(true);
            adjustmentsAbsolute_ControlAction->setEnabled(true);
             // enable the 'sequence control' menu entry depending on the implementation of this feature
            // by the active camera
//            adjustmentsSequence_ControlAction->setEnabled(true);
            adjustmentsSequence_ControlAction->setEnabled(dc1394->seqCtlAvailable(id));
            helpCamera_InfosAction->setEnabled(true);
            adjustmentsFactory_SettingsAction->setEnabled(true);
        }
        else {
            // camera already open...
            QMessageBox::information(this, "cc1394",
                                     QString("Camera ") + QString::number(id) + QString(" already open"),
                                     QMessageBox::Ok | QMessageBox::Default | QMessageBox::Escape,
                                     QMessageBox::NoButton,
                                     QMessageBox::NoButton);
 
            logger.log("camera %d already open\n", id);
            
            // try to raise the window of the selected camera
            if(cw[id]->isMaximized()) {
                cw[id]->showMaximized();
            }
            else {
                cw[id]->showNormal();
            }
        }
    }
    else {
        logger.log("Error! Menu-ID '%d' out of range\n", id);
        return false;
    }
    
    return true;
}


void mainForm::initFormatMenu(int f) {
    unsigned int formatMask = 0x80000000;
    bool selected = false; // for selecting the first found entry
    
    Format->clear();
    
    Format->insertItem(FORMAT0, F0);
    Format->setItemEnabled(F0, false);
    Format->insertItem(FORMAT1, F1);
    Format->setItemEnabled(F1, false);
    Format->insertItem(FORMAT2, F2);
    Format->setItemEnabled(F2, false);
    Format->insertItem(FORMAT7, F7);
    Format->setItemEnabled(F7, false);
    
    
    for(unsigned int i = 0; i < NUM_FORMATS; i++) {
        if((supportedFormats[selectedCameraItemID] & formatMask) != 0) {
            switch(i) {
            case (FORMAT_VGA_NONCOMPRESSED - FORMAT_MIN):
                //  Format->insertItem(FORMAT0, F0);
                Format->setItemEnabled(F0, true);
                break;
            case (FORMAT_SVGA_NONCOMPRESSED_1 - FORMAT_MIN):
                //  Format->insertItem(FORMAT1, F1);
                Format->setItemEnabled(F1, true);
                break;
            case (FORMAT_SVGA_NONCOMPRESSED_2 - FORMAT_MIN):
                //  Format->insertItem(FORMAT2, F2);
                Format->setItemEnabled(F2, true);
                break;
            case (FORMAT_STILL_IMAGE - FORMAT_MIN):
                break;
            case (FORMAT_SCALABLE_IMAGE_SIZE - FORMAT_MIN):
                //  Format->insertItem(FORMAT7, F7);
                Format->setItemEnabled(F7, true);
                break;
            default:
                break;
            }
            
            if((f == -1) && (!selected)) {
                Format->setItemChecked(i, true);
                selectedFormatItemID = i;
                selected = true;
            }
        }
        formatMask >>= 1;
    }
    
    if(f != -1) {
        Format->setItemChecked(f - FORMAT_MIN, true);
        selectedFormatItemID = f - FORMAT_MIN;
        selected = true;
    }
    
    Format->setEnabled(true);
    
}


void mainForm::initFramerateMenu(int frate) {
    unsigned int framerateMask = 0x80000000;
    bool selected = false;
    unsigned int minMode[8];
    
    minMode[0] = MODE_FORMAT0_MIN;
    minMode[1] = MODE_FORMAT1_MIN;
    minMode[2] = MODE_FORMAT2_MIN;
    minMode[3] = 0;
    minMode[4] = 0;
    minMode[5] = 0;
    minMode[6] = MODE_FORMAT6_MIN;
    minMode[7] = MODE_FORMAT7_MIN;
    
    Framerate->clear();
    Color_Coding->clear();
    
    // init framerate-menu, except for format 7 - it will be implemented later
    Framerate->insertItem(FR_1_875, FRAMERATE_1_875);
    Framerate->insertItem(FR_3_75, FRAMERATE_3_75);
    Framerate->insertItem(FR_7_5, FRAMERATE_7_5);
    Framerate->insertItem(FR_15, FRAMERATE_15);
    Framerate->insertItem(FR_30, FRAMERATE_30);
    Framerate->insertItem(FR_60, FRAMERATE_60);
    
    if(frate != -1) {
        Framerate->setItemChecked(frate, true);
        selectedFpsItemID = frate;
        selected = true;
    }
    
    for(unsigned int fr = 0; fr < NUM_FRAMERATES; fr++) {
        if(!(supportedFramerates[selectedCameraItemID][selectedFormatItemID]
             [selectedResItemID - minMode[selectedFormatItemID]] & framerateMask)) {
            Framerate->setItemEnabled(fr + FRAMERATE_MIN, false);
        }
        else {
            //     if((supportedFramerates[selectedCameraItemID][selectedFormatItemID]
            //   [selectedResItemID - minMode[selectedFormatItemID]] &
            //   framerateMask) != 0) {
/*
                switch(fr) {
                case (FRAMERATE_1_875 - FRAMERATE_MIN):
                    Framerate->insertItem(FR_1_875, FRAMERATE_1_875);
                    break;
                case (FRAMERATE_3_75 - FRAMERATE_MIN):
                    Framerate->insertItem(FR_3_75, FRAMERATE_3_75);
                    break;
                case (FRAMERATE_7_5 - FRAMERATE_MIN):
                    Framerate->insertItem(FR_7_5, FRAMERATE_7_5);
                    break;
                case (FRAMERATE_15 - FRAMERATE_MIN):
                    Framerate->insertItem(FR_15, FRAMERATE_15);
                    break;
                case (FRAMERATE_30 - FRAMERATE_MIN):
                    Framerate->insertItem(FR_30, FRAMERATE_30);
                    break;
                case (FRAMERATE_60 - FRAMERATE_MIN):
                    Framerate->insertItem(FR_60, FRAMERATE_60);
                    break;
                default:
                    break;
                }
*/  
            if((frate == -1) && (!selected)) {
                Framerate->setItemChecked(fr + FRAMERATE_MIN, true);
                selectedFpsItemID = fr + FRAMERATE_MIN;
                selected = true;
            }
        }
        framerateMask >>= 1;
    }
/*
        if(frate != -1) {
            Framerate->setItemChecked(frate, true);
            selectedFpsItemID = frate;
            selected = true;
        }
*/
    Framerate->setEnabled(true);
}


void mainForm::initColorCodingMenu(unsigned int camID) {
    unsigned int colorCoding, colorID;
    bool selected = false;
    
    
    Framerate->clear();
    Color_Coding->clear();
    
    Color_Coding->insertItem(F7_CC0, COLOR_FORMAT7_MONO8);
    Color_Coding->insertItem(F7_CC1, COLOR_FORMAT7_YUV411);
    Color_Coding->insertItem(F7_CC2, COLOR_FORMAT7_YUV422);
    Color_Coding->insertItem(F7_CC3, COLOR_FORMAT7_YUV444);
    Color_Coding->insertItem(F7_CC4, COLOR_FORMAT7_RGB8);
    Color_Coding->insertItem(F7_CC5, COLOR_FORMAT7_MONO16);
    Color_Coding->insertItem(F7_CC6, COLOR_FORMAT7_RGB16);
    
    if(!dc1394->getF7ColorCoding(camID, selectedResItemID, &colorCoding)) {
        logger.log("Error! Querying color-coding from camera %d (Mode: %d) failed!",
               camID, selectedResItemID);
        return;
    }
    
    logger.log("Available Color Coding: 0x%x", colorCoding);
    
    if((colorID = dc1394->getF7ColorCodingID(camID, selectedResItemID)) == 0) {
        logger.log("\n\tError while trying to read current Color-Coding-ID!\n");
    }
    else {
        logger.log("Current Color-Coding-ID: %d", colorID);
    }
    
    Color_Coding->setItemChecked(colorID, true);
    selectedColorCodingItemID = colorID;
    selected = true;
    
    for(unsigned int cc = 0; cc < NUM_COLOR_FORMAT7; cc++) {
        if(((colorCoding << cc) & 0x80000000) == 0) {
            Color_Coding->setItemEnabled(cc + COLOR_FORMAT7_MIN, false);
        }
    }
    
    Color_Coding->setEnabled(true);
}


void mainForm::initResolutionMenu(unsigned int id, int m) {
    switch(selectedFormatItemID) {
    case F0:
        initF0ResMenu(m);
        break;
    case F1:
        initF1ResMenu(m);
        break;
    case F2:
        initF2ResMenu(m);
        break;
/*
    case F3:
        initF3ResMenu(m);
        break;
    case F4:
        initF4ResMenu(m);
        break;
    case F5:
        initF5ResMenu(m);
        break;
*/
    case F6:
        // initF6ResMenu(m);
        break;
    case F7:
        initF7ResMenu(id, m);
        break;
    default:
        break;
    }
    
    Resolution->setEnabled(true);
    
}


void mainForm::initF0ResMenu(int mode) {
    unsigned int modeMask = 0x80000000;
    bool selected = false;
    
    // clear menu for rebuild
    Resolution->clear();
    
    // insert all possible modes
    Resolution->insertItem(F0_M0, MODE_160x120_YUV444);
    Resolution->insertItem(F0_M1, MODE_320x240_YUV422);
    Resolution->insertItem(F0_M2, MODE_640x480_YUV411);
    Resolution->insertItem(F0_M3, MODE_640x480_YUV422);
    Resolution->insertItem(F0_M4, MODE_640x480_RGB);
    Resolution->insertItem(F0_M5, MODE_640x480_MONO);
    Resolution->insertItem(F0_M6, MODE_640x480_MONO16);
    
    // if a mode is given as parameter - select it
    if(mode != -1) {
        Resolution->setItemChecked(mode, true);
        selectedResItemID = mode;
        selected = true;
    }
    
    // enable available modes
    for(unsigned int m = 0; m < NUM_FORMAT0_MODES; m++) {
        if(!(supportedModes[selectedCameraItemID][selectedFormatItemID] & modeMask)) {
            Resolution->setItemEnabled(m + MODE_FORMAT0_MIN, false);
        }
        else {
            // if((supportedModes[selectedCameraItemID][selectedFormatItemID] & modeMask) != 0) {
/*
            switch(m) {
            case (MODE_160x120_YUV444 - MODE_FORMAT0_MIN):
                Resolution->insertItem(F0_M0, MODE_160x120_YUV444);
                break;
            case (MODE_320x240_YUV422 - MODE_FORMAT0_MIN):
                Resolution->insertItem(F0_M1, MODE_320x240_YUV422);
                break;
            case (MODE_640x480_YUV411 - MODE_FORMAT0_MIN):
                Resolution->insertItem(F0_M2, MODE_640x480_YUV411);
                break;
            case (MODE_640x480_YUV422 - MODE_FORMAT0_MIN):
                Resolution->insertItem(F0_M3, MODE_640x480_YUV422);
                break;
            case (MODE_640x480_RGB - MODE_FORMAT0_MIN):
                Resolution->insertItem(F0_M4, MODE_640x480_RGB);
                break;
            case (MODE_640x480_MONO - MODE_FORMAT0_MIN):
                Resolution->insertItem(F0_M5, MODE_640x480_MONO);
                break;
            case (MODE_640x480_MONO16 - MODE_FORMAT0_MIN):
                Resolution->insertItem(F0_M6, MODE_640x480_MONO16);
                break;
            default:
                break;
            }
*/
            // if no mode was given as a parameter
            if((mode == -1) && (!selected)) {
                // select the first one
                Resolution->setItemChecked(m + MODE_FORMAT0_MIN, true);
                selectedResItemID = m + MODE_FORMAT0_MIN;
                selected = true;
            }
        }
        modeMask >>= 1;
    }
/*
    if(mode != -1) {
        Resolution->setItemChecked(mode, true);
        selectedResItemID = mode;
        selected = true;
    }
*/
}


void mainForm::initF1ResMenu(int mode) {
    unsigned int modeMask = 0x80000000;
    bool selected = false;
    
    // clear menu for rebuild
    Resolution->clear();
    
    // insert all possible modes
    Resolution->insertItem(F1_M0, MODE_800x600_YUV422);
    Resolution->insertItem(F1_M1, MODE_800x600_RGB);
    Resolution->insertItem(F1_M2, MODE_800x600_MONO);
    Resolution->insertItem(F1_M3, MODE_1024x768_YUV422);
    Resolution->insertItem(F1_M4, MODE_1024x768_RGB);
    Resolution->insertItem(F1_M5, MODE_1024x768_MONO);
    Resolution->insertItem(F1_M6, MODE_800x600_MONO16);
    Resolution->insertItem(F1_M7, MODE_1024x768_MONO16);
    
    // if a mode is given as a parameter
    if(mode != -1) {
        // select it
        Resolution->setItemChecked(mode, true);
        selectedResItemID = mode;
        selected = true;
    }
    
    // enable available modes
    for(unsigned int m = 0; m < NUM_FORMAT1_MODES; m++) {
        if(!(supportedModes[selectedCameraItemID][selectedFormatItemID] & modeMask)) {
            Resolution->setItemEnabled(m + MODE_FORMAT1_MIN, false);
        }
        else {
            // if((supportedModes[selectedCameraItemID][selectedFormatItemID] & modeMask) != 0) {
/*
            switch(m) {
            case (MODE_800x600_YUV422 - MODE_FORMAT1_MIN):
                Resolution->insertItem(F1_M0, MODE_800x600_YUV422);
                break;
            case (MODE_800x600_RGB - MODE_FORMAT1_MIN):
                Resolution->insertItem(F1_M1, MODE_800x600_RGB);
                break;
            case (MODE_800x600_MONO - MODE_FORMAT1_MIN):
                Resolution->insertItem(F1_M2, MODE_800x600_MONO);
                break;
            case (MODE_1024x768_YUV422 - MODE_FORMAT1_MIN):
                Resolution->insertItem(F1_M3, MODE_1024x768_YUV422);
                break;
            case (MODE_1024x768_RGB - MODE_FORMAT1_MIN):
                Resolution->insertItem(F1_M4, MODE_1024x768_RGB);
                break;
            case (MODE_1024x768_MONO - MODE_FORMAT1_MIN):
                Resolution->insertItem(F1_M5, MODE_1024x768_MONO);
                break;
            case (MODE_800x600_MONO16 - MODE_FORMAT1_MIN):
                Resolution->insertItem(F1_M6, MODE_800x600_MONO16);
                break;
            case (MODE_1024x768_MONO16 - MODE_FORMAT1_MIN):
                Resolution->insertItem(F1_M7, MODE_1024x768_MONO16);
                break;
            default:
                break;
            }
*/
            // if no mode is given as a parameter
            if((mode == -1) && (!selected)) {
                // select the first available entry
                Resolution->setItemChecked(m + MODE_FORMAT1_MIN, true);
                selectedResItemID = m + MODE_FORMAT1_MIN;
                selected = true;
            }
        }
        modeMask >>= 1;
    }
    
/*
    if(mode != -1) {
        Resolution->setItemChecked(mode, true);
        selectedResItemID = mode;
        selected = true;
    }
*/
}


void mainForm::initF2ResMenu(int mode) {
    unsigned int modeMask = 0x80000000;
    bool selected = false;
    
    // clear menu for rebuild
    Resolution->clear();
    
    // insert all possible modes
    Resolution->insertItem(F2_M0, MODE_1280x960_YUV422);
    Resolution->insertItem(F2_M1, MODE_1280x960_RGB);
    Resolution->insertItem(F2_M2, MODE_1280x960_MONO);
    Resolution->insertItem(F2_M3, MODE_1600x1200_YUV422);
    Resolution->insertItem(F2_M4, MODE_1600x1200_RGB);
    Resolution->insertItem(F2_M5, MODE_1600x1200_MONO);
    Resolution->insertItem(F2_M6, MODE_1280x960_MONO16);
    Resolution->insertItem(F2_M7, MODE_1600x1200_MONO16);
    
    // if a mode was given as a parameter
    if(mode != -1) {
        // select it
        Resolution->setItemChecked(mode, true);
        selectedResItemID = mode;
        selected = true;
    }
    
    // enable available modes
    for(unsigned int m = 0; m < NUM_FORMAT2_MODES; m++) {
        if(!(supportedModes[selectedCameraItemID][selectedFormatItemID] & modeMask)) {
            Resolution->setItemEnabled(m + MODE_FORMAT2_MIN, false);
        }
        else {
            // if((supportedModes[selectedCameraItemID][selectedFormatItemID] & modeMask) != 0) {
/*
            switch(m) {
            case (MODE_1280x960_YUV422 - MODE_FORMAT2_MIN):
                Resolution->insertItem(F2_M0, MODE_1280x960_YUV422);
                break;
            case (MODE_1280x960_RGB - MODE_FORMAT2_MIN):
                Resolution->insertItem(F2_M1, MODE_1280x960_RGB);
                break;
            case (MODE_1280x960_MONO - MODE_FORMAT2_MIN):
                Resolution->insertItem(F2_M2, MODE_1280x960_MONO);
                break;
            case (MODE_1600x1200_YUV422 - MODE_FORMAT2_MIN):
                Resolution->insertItem(F2_M3, MODE_1600x1200_YUV422);
                break;
            case (MODE_1600x1200_RGB - MODE_FORMAT2_MIN):
                Resolution->insertItem(F2_M4, MODE_1600x1200_RGB);
                break;
            case (MODE_1600x1200_MONO - MODE_FORMAT2_MIN):
                Resolution->insertItem(F2_M5, MODE_1600x1200_MONO);
                break;
            case (MODE_1280x960_MONO16 - MODE_FORMAT2_MIN):
                Resolution->insertItem(F2_M6, MODE_1280x960_MONO16);
                break;
            case (MODE_1600x1200_MONO16 - MODE_FORMAT2_MIN):
                Resolution->insertItem(F2_M7, MODE_1600x1200_MONO16);
                break;
            default:
                break;
            }
*/
            // if no mode was given as a parameter
            if((mode == -1) && (!selected)) {
                // select the first available one
                Resolution->setItemChecked(m +  MODE_FORMAT2_MIN, true);
                selectedResItemID = m + MODE_FORMAT2_MIN;
                selected = true;
            }
        }
        modeMask >>= 1;
    }
    
/*
    if(mode != -1) {
        Resolution->setItemChecked(mode, true);
        selectedResItemID = mode;
        selected = true;
    }
*/
}


void mainForm::initF7ResMenu(unsigned int id, int mode) {
    QString menuText;
    unsigned int maxX, maxY;
    unsigned int modeMask = 0x80000000;
    bool selected = false;
    
    // clear menu for rebuild
    Resolution->clear();
    
    // insert all possible modes
    Resolution->insertItem(F7_M0, MODE_FORMAT7_0);
    Resolution->insertItem(F7_M1, MODE_FORMAT7_1);
    Resolution->insertItem(F7_M2, MODE_FORMAT7_2);
    Resolution->insertItem(F7_M3, MODE_FORMAT7_3);
    Resolution->insertItem(F7_M4, MODE_FORMAT7_4);
    
    // if a mode was given as a parameter
    if(mode != -1) {
        // select it
        Resolution->setItemChecked(mode, true);
        selectedResItemID = mode;
        selected = true;
    }
    
    // enable available modes
    for(unsigned int m = 0; m < NUM_MODE_FORMAT7; m++) {
        if(!(supportedModes[id][selectedFormatItemID] & 
             (modeMask >> m))) {
            Resolution->setItemEnabled(m +  MODE_FORMAT7_MIN, false);
        
            continue;
        }
        
        if(!dc1394->getF7MaxSize(id, m + MODE_FORMAT7_MIN, &maxX, &maxY)) {
            logger.log("Error! Reading max image size for mode %d failed", m);
        }
        else {
            menuText = Resolution->text(m + MODE_FORMAT7_MIN);
            Resolution->changeItem((m + MODE_FORMAT7_MIN),
                                   QString(menuText + " - up to " + QString::number(maxX) +
                                           "x" + QString::number(maxY)));
        }
        
        // if no mode was given as a parameter
        if((mode == -1) && !selected) {
            // select the first available entry
            Resolution->setItemChecked(m + MODE_FORMAT7_MIN, true);
            selectedResItemID = m + MODE_FORMAT7_MIN;
            selected = true;
        }
        
    }
}


// 'Constructor'
void mainForm::init() {
    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
    
    // Cameras-Menu
    connect(Cameras, SIGNAL(activated(int)), this, SLOT(selectCamera(int)));
    Cameras->setCheckable(true);
    selectedCameraItemID = -1;
    
    closeForce = false;
    
    loggingEnabled = false;
    
    // Workspace
    workspace = new QWorkspace(this);
    workspace->setScrollBarsEnabled(true);
    setCentralWidget(workspace);
    
    connect(workspace, SIGNAL(windowActivated(QWidget *)),
            this, SLOT(windowActivated(QWidget *)));
    
    dc1394 = new DC1394(&logger);
    
    openCameras = 0;
    
    if(!initCameras()) {
        QApplication::restoreOverrideCursor();
        
        QMessageBox::warning(this, "cc1394",
                             "No Cameras found!\nPlease check cables and connections!",
                             QMessageBox::Ok | QMessageBox::Default,
                             QMessageBox::NoButton,
                             QMessageBox::NoButton);
        return;
    }
    
    QApplication::restoreOverrideCursor();
}


void mainForm::disableUIControl() {
    // disable menu-bar
    mainMenuBar->setEnabled(false);
    // disable all windows
    workspace->setEnabled(false);
}


void mainForm::enableUIControl() {
    // enable menu-bar
    mainMenuBar->setEnabled(true);
    // enable all windows
    workspace->setEnabled(true);
}


// Initialization of the 'Cameras' menu (mainly entry of found cameras)
bool mainForm::initCameras() {
    int portCount;
    
    
    camCount = 0;
    
    // all cameras are initially closed
    for(int i = 0; i < MAX_CAMERAS; i++) {
        cameraOpen[i] = false;
    }
    
    if((portCount = dc1394->initRaw1394()) <= 0) {
        QApplication::restoreOverrideCursor();
        
        QMessageBox::warning(this, "cc1394",
                             "No 'raw1394'-ports found!\nPlease check if 'raw1394'-module is loaded\n and the permissions are sufficient!",
                             QMessageBox::Ok | QMessageBox::Default,
                             QMessageBox::NoButton,
                             QMessageBox::NoButton);
        exit(0);
    }
    
    // IEEE1394 port initialization
    if(QFile::exists("/dev/video1394/0")) {
        // scan all busses for cameras
        for(int i = 0; i < portCount; i++) {
            if(QFile::exists(QString("/dev/video1394/" + QString::number(i)))) {
                camCount += dc1394->dc1394Init(
                        QString("/dev/video1394/" + QString::number(i)), true, i);
            }
            else { // '/dev/video1394/<i>' does not exist
                logger.log("Error! No '/dev'ice-node for video1394-port %d (%s) found!\n", i,
                       (QString("/dev/video1394/" + QString::number(i))).ascii());
            }
        }
    }
    else { // '/dev/video1394/0' does not exist
        if(QFile::exists("/dev/video1394")) {
//            QMessageBox::information(this, "cc1394",
//                                     "This system-configuration supports\nonly one 'IEEE1394' bus!",
//                                     QMessageBox::Ok | QMessageBox::Default,
//                                     QMessageBox::NoButton,
//                                     QMessageBox::NoButton);
            
            logger.log("This system-configuration supports only 1 IEEE1394 bus (OCHI-Port)");
            logger.log("because only '/dev/video1394' node is a available.");
            logger.log("(For more information see 'http://www.linux1394.org')");
            
            camCount += dc1394->dc1394Init("/dev/video1394");
        }
        else { // neither '/dev/video1394/0' nor '/dev/video1394' exist
            switch(QMessageBox::warning(this, "cc1394",
                                        "Unable to locate any standard 'video1394'-device!",
                                        "OK", "Cancel", "Locate...", 0, 0)) {
            case 2:
                for(int i = 0; i < portCount; i++) {
                    // file selector for video1394-port <i>
                    QString devName = QFileDialog::getOpenFileName("/dev", "video1394 (*)",
                                                                   this, "open device dialog",
                                                                   QString("Video1394-Device %d") +
                                                                   QString::number(i) +
                                                                   QString(" - Major 171 / Minor ") +
                                                                   QString::number(16 + i));
                    // if 'OK '- try to initialize port
                    if(devName == "") {
                        logger.log("Error! No device for port %d (minor '%d') selected", i, i + 16);
                        continue;
                    }
                    
                    camCount += dc1394->dc1394Init(devName, true, i);
                }
                break;
                
            default:
                exit(0);
                break;
            }
        }
    }
    
    // enter found cameras into the camera-menu
    if(camCount > 0) {
        logger.log("found %d cameras in total - now trying to get the parameters", camCount);
        for(unsigned int i = 0; i < camCount; i++) {
            logger.log("Camera %d Init...", i);
            // enter the models of the found cameras into the camera-menu
            Cameras->insertItem(dc1394->getCameraModel(i), i);
            // get camera-parameters
            getCameraParameters(i);
        }
        
        // enable the 'open all cameras' menu-entry
        camerasOpen_All_CamerasAction->setEnabled(true);
    }
    else {
        // if no cameras were found 'open all' does not makes any sense
        camerasOpen_All_CamerasAction->setEnabled(false);
        camerasTileAction->setEnabled(false);
        camerasCascadeAction->setEnabled(false);
        
        return false;
    }
    
    // Camera-Menu
    // 'close all' menu entry initially disabled
    camerasClose_All_CamerasAction->setEnabled(false);
    camerasISO_SpeedAction->setEnabled(false);
    adjustmentsPicture_ControlAction->setEnabled(false);
    adjustmentsI_O3Action->setEnabled(false);
    adjustmentsDirect_Register_ManipulationAction->setEnabled(false);
    adjustmentsSequence_ControlAction->setEnabled(false);
    adjustmentsAbsolute_ControlAction->setEnabled(false);
    helpCamera_InfosAction->setEnabled(false);
    adjustmentsFactory_SettingsAction->setEnabled(false);
    // initialize the 'openCameras' counter
    openCameras = 0;
    
    
    // Format-Menu
    Format->clear();
    Format->setCheckable(true);
    connect(Format, SIGNAL(activated(int)), this, SLOT(formatActivated(int)));
    Format->setEnabled(false);
    selectedFormatItemID = -1;
    
    // Resolution-Menu (default - Resolution-Menu for Foramt 0)
    Resolution->clear();
    Resolution->setCheckable(true);
    connect(Resolution, SIGNAL(activated(int)), this, SLOT(resActivated(int)));
    Resolution->setEnabled(false);
    selectedResItemID = -1;
    
    // Framerate-Menu
    Framerate->clear();
    Framerate->setCheckable(true);
    connect(Framerate, SIGNAL(activated(int)), this, SLOT(fpsActivated(int)));
    Framerate->setEnabled(false);
    selectedFpsItemID = -1;
    
    // ColorCoding-Menu
    Color_Coding->clear();
    Color_Coding->setCheckable(true);
    connect(Color_Coding, SIGNAL(activated(int)), this, SLOT(cCActivated(int)));
    Color_Coding->setEnabled(false);
    selectedColorCodingItemID = -1;
    
    logger.log("cameras initialized!");
    
    if(camCount == 1) {
        logger.log("Info - only one camera available - it will be opened at the start...");
        selectCamera(0);
    }
    
    return true;
} // initCameras()


void mainForm::quit() {
    QApplication::exit();
}


void mainForm::customEvent(QCustomEvent *e) {
    if(e->type() == CAMWINDOWCLOSE) {
        logger.log("CamWindowCloseEvent received for camera %d", *((int *)e->data()));
        
        // release raw1394 port
        if(!dc1394->destroyHandle(*((int *)e->data())))
            logger.log("Error while trying to release 'raw1394 handle'!\n");
        
        cameraOpen[*((int *)e->data())] = false;
        
        delete cw[*((int *)e->data())];
        
        // disable the 'close all' menu entry if no cameras already open
        if((--openCameras) == 0) {
            camerasClose_All_CamerasAction->setEnabled(false);
            camerasOpen_All_CamerasAction->setEnabled(true);
            camerasTileAction->setEnabled(false);
            camerasCascadeAction->setEnabled(false);
            camerasISO_SpeedAction->setEnabled(false);
            adjustmentsPicture_ControlAction->setEnabled(false);
            adjustmentsI_O3Action->setEnabled(false);
            adjustmentsDirect_Register_ManipulationAction->setEnabled(false);
            adjustmentsSequence_ControlAction->setEnabled(false);
            adjustmentsAbsolute_ControlAction->setEnabled(false);
            helpCamera_InfosAction->setEnabled(false);
            adjustmentsFactory_SettingsAction->setEnabled(false);
            Cameras->setItemChecked(selectedCameraItemID, false);
            selectedCameraItemID = -1;
            Format->setEnabled(false);
            selectedFormatItemID = -1;
            Resolution->setEnabled(false);
            selectedResItemID = -1;
            Framerate->setEnabled(false);
            selectedFpsItemID = -1;
            
            if(closeForce) {
                logger.log("last camera closed - rescanning bus");
                rescanBus();
                closeForce = false;
            }
        }
        else {
            if(openCameras > 1) {
                camerasTileAction->setEnabled(true);
                camerasCascadeAction->setEnabled(true);
            }
            else {
                camerasTileAction->setEnabled(false);
                camerasCascadeAction->setEnabled(false);
            }
        }
    }
    else
        logger.log("Error! Unknown 'custom event' received (%d)!\n", e->type());
}


void mainForm::closeEvent(QCloseEvent *e) {
    bool saveBeforeExit = false;
    
    
    logger.log("'CloseEvent'...");
    
    if(loggingEnabled && !logger.saved()) {
        switch(QMessageBox::warning(this, "cc1394",
                                    "Some Messages have not been saved.\n\nSave now?",
                                    QMessageBox::Yes,
                                    QMessageBox::No | QMessageBox::Default,
                                    QMessageBox::Cancel | QMessageBox::Escape)) {
        case QMessageBox::Yes:
            saveBeforeExit = true;
            break;
        case QMessageBox::No:
            break;
        case QMessageBox::Cancel:
            e->ignore();
            return;
            break;
        default:
            break;
        }
    }
    
    logger.close();
    
    // close all open cameras
    if(openCameras > 0) {
        logger.log("Now trying to close %d open camera-window(s)\n", openCameras);
        
        for(unsigned int i = 0; i < camCount; i++) {
            // but try to close only the open windows
            if(cameraOpen[i]) {
                // close the window
                cw[i]->close();
                if(cw[i]->isShown()) {
                    e->ignore();
                    return;
                }
                cameraOpen[i] = false;
            }
        }
    }
    
    if(saveBeforeExit) {
        logger.saveMessages(true);
    }
    
    // cleanup all dc1394-stuff
    if(!dc1394->dc1394Cleanup()) {
        logger.log("Error! 'dc1394Cleanup()' failed!\n");
    }
    
    delete workspace;
    
    delete dc1394;
    
    logger.log("... Last Message Before Application-Exit ...");
    
    e->accept();
}



void mainForm::closeAllCameras() {
    for(unsigned int i = 0; i < MAX_CAMERAS; i++) {
        // but try to close only the open windows
        if(cameraOpen[i]) {
            // just try close the window (an event will inform about the closure)
            cw[i]->close();
        }
    }
    
    camerasClose_All_CamerasAction->setEnabled(false);
    camerasOpen_All_CamerasAction->setEnabled(true);
    camerasTileAction->setEnabled(false);
    camerasCascadeAction->setEnabled(false);
    camerasISO_SpeedAction->setEnabled(false);
    adjustmentsPicture_ControlAction->setEnabled(false);
    adjustmentsI_O3Action->setEnabled(false);
    adjustmentsDirect_Register_ManipulationAction->setEnabled(false);
    adjustmentsSequence_ControlAction->setEnabled(false);
    adjustmentsAbsolute_ControlAction->setEnabled(false);
    helpCamera_InfosAction->setEnabled(false);
    adjustmentsFactory_SettingsAction->setEnabled(false);
    
}


void mainForm::openAllCameras() {
    // If enabled (in future!) - a message-box will be opened to choose a mode common to all
    // available (connected) cameras.
    // If no common mode is available a message-box will appear to inform the user about this issue.
    // The implementation below is from a former test, but no longer valid!!!
    // Therefore the Menu-Item is disabled directly in 'designer'!!!
    
    logger.log("%d cameras about to open...", camCount);
    
    for(unsigned int i = 0; i < camCount; i++) {
        // just 'try' to open the respective camera
        selectCamera(i);
    }
}


void mainForm::tileCameraWindows() {
    workspace->tile();
}


void mainForm::cascadeCameraWindows() {
    workspace->cascade();
}


void mainForm::fpsActivated(int id) {
    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
    QApplication::flush();
    
    Framerate->setItemChecked(selectedFpsItemID, false);
    Framerate->setItemChecked(id, true);
    selectedFpsItemID = id;
    
    if(cameraOpen[selectedCameraItemID]) {
        cw[selectedCameraItemID]->updateSize(selectedFormatItemID + FORMAT_MIN,
                                             selectedResItemID, selectedFpsItemID,
                                             selectedColorCodingItemID);
    }
    
    QApplication::restoreOverrideCursor();
}


void mainForm::cCActivated(int id) {
    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
    QApplication::flush();
    
    Color_Coding->setItemChecked(selectedColorCodingItemID, false);
    Color_Coding->setItemChecked(id, true);
    selectedColorCodingItemID = id;
    
    if(cameraOpen[selectedCameraItemID]) {
        cw[selectedCameraItemID]->updateSize(selectedFormatItemID + FORMAT_MIN,
                                             selectedResItemID, selectedFpsItemID,
                                             selectedColorCodingItemID);
    }
    
    QApplication::restoreOverrideCursor();
}


void mainForm::formatActivated( int id ) {
    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
    QApplication::flush();
    
    switch(id) {
    case 6:
        // message-box to inform the user about the issue, that F6+F7 are not yet supported
        QMessageBox::information(this, "Format unsupported",
                                 "This format is not yet supported by 'cc1394'!",
                                 QMessageBox::Ok | QMessageBox::Default,
                                 QMessageBox::NoButton,
                                 QMessageBox::NoButton);
        break;
        
    case 7:
        Format->setItemChecked(selectedFormatItemID, false);
        Format->setItemChecked(id, true);
        selectedFormatItemID = id;
        
        
        initResolutionMenu(selectedCameraItemID, -1);
/*
        if(selectedResItemID < 7)
            initFramerateMenu(-1);
        else
*/
            initColorCodingMenu(selectedCameraItemID);
        
        // activate F7
        if(cameraOpen[selectedCameraItemID]) {
            cw[selectedCameraItemID]->updateSize(selectedFormatItemID +
                                                 FORMAT_MIN, selectedResItemID, selectedFpsItemID,
                                                 selectedColorCodingItemID);
        }
        
//        initResolutionMenu(-1);
//        initFramerateMenu(-1);
        
        break;
        
    default:
        Format->setItemChecked(selectedFormatItemID, false);
        Format->setItemChecked(id, true);
        selectedFormatItemID = id;
        
        
        initResolutionMenu(selectedCameraItemID, -1);
        
//        if(selectedResItemID < 7)
            initFramerateMenu(-1);
//        else
//            initColorCodingMenu(selectedCameraItemID);
        
        if(cameraOpen[selectedCameraItemID]) {
            cw[selectedCameraItemID]->updateSize(selectedFormatItemID +
                                                 FORMAT_MIN, selectedResItemID, selectedFpsItemID,
                                                 selectedColorCodingItemID);
        }
        break;
    }
    
    QApplication::restoreOverrideCursor();
    
}


void mainForm::resActivated( int id ) {
    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
    QApplication::flush();
    
    Resolution->setItemChecked(selectedResItemID, false);
    Resolution->setItemChecked(id, true);
    selectedResItemID = id;
    
    if(selectedFormatItemID < 7)
        initFramerateMenu(-1);
    else
        initColorCodingMenu(selectedCameraItemID);
    
    if(cameraOpen[selectedCameraItemID]) {
        cw[selectedCameraItemID]->updateSize(selectedFormatItemID + FORMAT_MIN,
                                             selectedResItemID, selectedFpsItemID,
                                             selectedColorCodingItemID);
    }
    
    QApplication::restoreOverrideCursor();
}


void mainForm::aboutSlot() {
    QMessageBox::information(this, "About...",
                             (QString("'cc1394'\nCamera Control Application\nfor IIDC1394 cameras\nVersion ") +
                             QString(VERSION) +
                             QString("\n(Uses 'libavt1394' - Version ") + QString(LIBAVT_VERSION) +
                             QString(")\n\n(C) 2005, AVT")).ascii(),
                             QMessageBox::Ok | QMessageBox::Default,
                             QMessageBox::NoButton,
                             QMessageBox::NoButton);
}


void mainForm::selectCamera(int id) {
    int format = -1;
    int mode = -1;
    int framerate = -1;
    
    
    if((id >= 0) && (id < MAX_CAMERAS)) {
        // uncheck the currently checked camera-menu-item
        // if(selectedCameraItemID >= 0) {
        //     Cameras->setItemChecked(selectedCameraItemID, false);
        // }
        
        // check the selected camera-menu-item
        // Cameras->setItemChecked(id, true);
        // save the selected camera-menu-item-id
        // selectedCameraItemID = id;
        if(!cameraOpen[id]) {
            if(!dc1394->createHandle(id)) {
                logger.log("Error while trying to create a 'raw1394 handle'!\n");
                return;
            }
        }
        
        logger.log("trying to get the current parameters...");
        format = dc1394->getFormat(id);
        logger.log("format = %d", format);
        mode = dc1394->getMode(id);
        logger.log("mode = %d", mode);
        
        // initialize the Format- Mode- and Framerate-Menus accordingly
        initFormatMenu(format);
        initResolutionMenu(id, mode);
        
        if(format <= FORMAT_SVGA_NONCOMPRESSED_2) {
            framerate = dc1394->getFramerate(id);
            logger.log("framerate = %d\n", framerate);
            initFramerateMenu(framerate);
        }
        else {
            if(format == FORMAT_SCALABLE_IMAGE_SIZE) {
                logger.log("F7 - determining the available color codings for camera %d...", id);
                initColorCodingMenu(id);
            }
            else {
                logger.log("Warning! F6 not yet supported!");
            }
        }
        
/*
        // if the camera is opened - set the format, framerate, and mode-menues accordingly
        if(cameraOpen[id]) {
            logger.log("...and raising the window, 'cause it's open");
            if(cw[id]->isMaximized()) {
                cw[id]->showMaximized();
            }
            else {
                cw[id]->showNormal();
            }
        }
*/
        if(openCameraWindow(id)) {
            if(selectedCameraItemID >= 0) {
                Cameras->setItemChecked(selectedCameraItemID, false);
            }
            
            Cameras->setItemChecked(id, true);
            selectedCameraItemID = id;
            
            if(openCameras > 0) {
                camerasOpen_All_CamerasAction->setEnabled(openCameras < camCount);
                camerasClose_All_CamerasAction->setEnabled(true);
                camerasISO_SpeedAction->setEnabled(true);
                adjustmentsAbsolute_ControlAction->setEnabled(true);
                if(openCameras > 1) {
                    camerasTileAction->setEnabled(true);
                    camerasCascadeAction->setEnabled(true);
                }
                else {
                    camerasTileAction->setEnabled(false);
                    camerasCascadeAction->setEnabled(false);
                }
            }
            else {
                camerasOpen_All_CamerasAction->setEnabled(true);
                camerasClose_All_CamerasAction->setEnabled(false);
                camerasTileAction->setEnabled(false);
                camerasCascadeAction->setEnabled(false);
                camerasISO_SpeedAction->setEnabled(false);
                adjustmentsAbsolute_ControlAction->setEnabled(false);
            }
        }
        else {
            if(openCameras > 0) {
                camerasOpen_All_CamerasAction->setEnabled(false);
                camerasClose_All_CamerasAction->setEnabled(true);
                camerasISO_SpeedAction->setEnabled(true);
                adjustmentsAbsolute_ControlAction->setEnabled(true);
                if(openCameras > 1) {
                    camerasTileAction->setEnabled(true);
                    camerasCascadeAction->setEnabled(true);
                }
                else {
                    camerasTileAction->setEnabled(false);
                    camerasCascadeAction->setEnabled(false);
                }
            }
            else {
                camerasOpen_All_CamerasAction->setEnabled(false);
                camerasClose_All_CamerasAction->setEnabled(false);
                camerasTileAction->setEnabled(false);
                camerasCascadeAction->setEnabled(false);
                camerasISO_SpeedAction->setEnabled(false);
                adjustmentsAbsolute_ControlAction->setEnabled(false);
            }
        }
    }
}



void mainForm::getCameraParameters( unsigned int id ) {
    unsigned int formatsMask = 0x80000000;
    unsigned int modesMask = 0x80000000;
    unsigned int framerateMask = 0x80000000;
    unsigned int minMode[NUM_FORMATS];
    unsigned int numModes[NUM_FORMATS];
    
    minMode[0] = MODE_FORMAT0_MIN;
    minMode[1] = MODE_FORMAT1_MIN;
    minMode[2] = MODE_FORMAT2_MIN;
    minMode[3] = 0;
    minMode[4] = 0;
    minMode[5] = 0;
    minMode[6] = MODE_FORMAT6_MIN;
    minMode[7] = MODE_FORMAT7_MIN;
    
    numModes[0] = NUM_FORMAT0_MODES;
    numModes[1] = NUM_FORMAT1_MODES;
    numModes[2] = NUM_FORMAT2_MODES;
    numModes[3] = 0;
    numModes[4] = 0;
    numModes[5] = 0;
    numModes[6] = NUM_FORMAT6_MODES;
    numModes[7] = NUM_MODE_FORMAT7;
    
    
    // read supported formats
    supportedFormats[id] = dc1394->getSupportedFormats(id);
    
    // for each supported format - read supported modes
    for(unsigned int f = 0; f < NUM_FORMATS; f++) {
        if(supportedFormats[id] & formatsMask) {
            supportedModes[id][f] = dc1394->getSupportedModes(id, FORMAT_MIN +f);
            // for each supported mode - read supported framerates
            modesMask = 0x80000000;
            for(unsigned int m = 0; m < numModes[f]; m++) {
                if((supportedModes[id][f] & modesMask) && (f != 7)) {
                    supportedFramerates[id][f][m] = dc1394->getSupportedFramerates(id, FORMAT_MIN + f, minMode[f] + m);
                    
                    framerateMask = 0x80000000;
                    for(unsigned int fr = 0; fr < 8; fr++) {
                        framerateMask >>= 1;
                    }
                }
                else {
                    supportedFramerates[id][f][m] = 0x00000000;
                }
                modesMask >>= 1;
            }
        }
        else {
            supportedModes[id][f] = 0x00000000;
        }
        formatsMask >>= 1;
    }
}


void mainForm::windowActivated(QWidget *w) {
    CamWindow *window = (CamWindow *)w;
    int format = -1;
    int mode = -1;
    int framerate = -1;
    unsigned int id;
    
    
    if(window != NULL) {
        logger.log("window %s activated", (window->caption()).ascii());
        
        Cameras->setItemChecked(selectedCameraItemID, false);
        id = window->getID();
        selectedCameraItemID = id;
        Cameras->setItemChecked(selectedCameraItemID, true);
        
        // get the current parameters for the selected window
        format = dc1394->getFormat(id);
        mode = dc1394->getMode(id);
        framerate = dc1394->getFramerate(id);
        
        // initialize the Format- Mode- and Framerate-Menus accordingly
        initFormatMenu(format);
        initResolutionMenu(selectedCameraItemID, mode);
        if((format - FORMAT_MIN) < 7)
            initFramerateMenu(framerate);
        else
            initColorCodingMenu(selectedCameraItemID);
    }
    else {
        logger.log("Error! Window activated with a NULL pointer!");
    }
}


void mainForm::pictureControl() {
    // if there is currently a camera-window selected
    if(selectedCameraItemID >= 0)
        // exec the corresponding 'Adjustments-Form'
        cw[selectedCameraItemID]->execAdjustments();
    
}



void mainForm::adjustmentsAbsolute_ControlAction_toggled( bool newValue ) {
    if(newValue) {
        adjustmentsAbsolute_ControlAction->toggle();
        return;
    }
    
    QMessageBox::information(this, "cc1394",
                             "Absolute Control not yet available!",
                             QMessageBox::Ok | QMessageBox::Default | QMessageBox::Escape,
                             QMessageBox::NoButton,
                             QMessageBox::NoButton);
    
//    logger.log("Absolute Control switched %s", newValue ? "on" : "off");
    
//    cw[selectedCameraItemID]->enableAbsolute(newValue);
}


void mainForm::cameraInfos() {
    if(selectedCameraItemID >= 0)
        cw[selectedCameraItemID]->execCamInfos();
}


void mainForm::cameraReset() {
    if(selectedCameraItemID >= 0)
        cw[selectedCameraItemID]->execCameraReset();
}


void mainForm::rescanCameras() {
    // release all IEEE1394-bus-informations
    logger.log("'rescan cameras' - closing all cameras...");
    
    if(openCameras != 0) {
        closeForce = true;
        closeAllCameras();
    }
    else
        rescanBus();
}


void mainForm::rescanBus() {
    // remove camera-entries from menu
    for(unsigned int i = 0; i < camCount; i++) {
        logger.log("'rescan cameras' - removing item %d (%s) from 'Cameras' menu...",
               i, (Cameras->text(i)).ascii());
        Cameras->removeItem(i);
    }
    
    // reset IEEE1394-bus
    if(!dc1394->dc1394Cleanup()) {
        logger.log("Error! 'dc1394Cleanup()' failed!\n");
    }
    
    delete dc1394;
    
    selectedCameraItemID = -1;
    
    // rescan IEEE1394-bus
    logger.log("'rescan cameras' - creating new 'DC1394' object...");
    dc1394 = new DC1394(&logger);
    
    openCameras = 0;
    
    logger.log("'rescan cameras' - initializing cameras...");
    if(!initCameras()) {
        QMessageBox::warning(this, "cc1394 - Rescan Cameras",
                             "No Cameras found!\nPlease check cables and connections!",
                             QMessageBox::Ok | QMessageBox::Default,
                             QMessageBox::NoButton,
                             QMessageBox::NoButton);
    }
    
    logger.log("'rescan cameras' - cameras initialized...");
}



void mainForm::seqCtlDemo() {
    // if there is currently a camera-window selected
    if(selectedCameraItemID >= 0)
        // open SequenceControlDemo dialog
        cw[selectedCameraItemID]->execSeqCtlDemo();
    
}


void mainForm::fileMessageLogger() {
    logger.show();
}


void mainForm::changeISOSpeed() {
    unsigned int isoSpeed = dc1394->getIsoSpeed(selectedCameraItemID);
    
    QMessageBox::information(this, "cc1394",
                             "Change of ISO-Speed not yet supported!\nCurrent Value: " +
                             QString::number(pow(2,isoSpeed) * 100) +
                             "Mbps",
                             QMessageBox::Ok | QMessageBox::Default | QMessageBox::Escape,
                             QMessageBox::NoButton,
                             QMessageBox::NoButton);
}


void mainForm::enableMessageLogging(bool enable) {
    loggingEnabled = enable;
    // close Logging Window if opened
    logger.close();
    fileMessage_LoggerAction->setEnabled(enable);
    
}


void mainForm::directRegisterManipulation() {
    cw[selectedCameraItemID]->execDRM();
}


void mainForm::io3(bool io3) {
    // if 'io3' == true - replace the 'RS232-controls' in the 'Picture-Control' dialog with
    // the 'I/O3'-controls
    
    for(unsigned int i = 0; i < camCount; i++)
        cw[i]->enableIO3(io3);
}
