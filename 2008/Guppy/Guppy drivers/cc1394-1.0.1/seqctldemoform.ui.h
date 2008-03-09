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


void SeqCtlDemoForm::init() {
    // dialog-box not yet initialized ('initSeqCtl()' has to called explicitly!)
    initialized = false;
    
    // image controls
    gainGroupBox->setEnabled(false);
    
    // timing controls
    exposureTimeGroupBox->setEnabled(false);
    
    f7AOIGroupBox->setEnabled(false);
    
    seqCtlStopPushButton->setEnabled(false);
    
    updateF7UI = true;
    f7CorrectValue = false;
    
    videoRunning = true;
    initSeq = false;
    seqComplete = false;
    autoInc = false;
    imgCount = 0;
    sequenceLength = 1;
} // init()


void SeqCtlDemoForm::initSeqCtl(QWidget *cw, DC1394 *dc, int i, bool absControl,
                                MessageLogger *ml) {
    if(initialized) {
        updateUI();
        return;
    }
    
    dc1394 = dc;
    id = i;
    absolute = absControl;
    parent = cw;
    logger = ml;
    
    
    // update the UI (after reading the current state of the camera feature set)
    f7Changed = false;
    
    updateUI();
    
    // AdjustmentsForm initialized
    initialized = true;
    
    
    applyPushButton->setEnabled(false);
    discardPushButton->setEnabled(false);
    
    seqCtlGroupBox->setEnabled(true);
} // initSeqCtl()


void SeqCtlDemoForm::show() {
    // update UI for getting the current state of the camera
    updateUI();
    
    // initially fill up internal sequence-array with the current values
    // (because of consistency reasons this will be done at every time the dialog is displayed)
    for(int i = 0; i < MAX_SEQ_LENGTH; i++) {
        // initial 'gainLevel'
        if(gainGroupBox->isEnabled()) {
            sequence[i].gainAvailable = true;
            sequence[i].gainLevel = gainSpinBox->value();
        }
        else
            sequence[i].gainAvailable = false;
        
        // initial 'shutterValue'
        if(shutterGroupBox->isEnabled()) {
            sequence[i].shutterAvailable = true;
            sequence[i].shutterValue = shutterSpeedSpinBox->value();
        }
        else
            sequence[i].shutterAvailable = false;
        
        // initial 'f7Availability'
        sequence[i].f7Available = f7AOIGroupBox->isEnabled();
        // if F7 is available - initialize the corresponding variables
        if(sequence[i].f7Available) {
            // 'f7Width'
            sequence[i].f7Width = f7AOIWidthSpinBox->value();
            // 'f7Height'
            sequence[i].f7Height = f7AOIHeightSpinBox->value();
            // 'f7X'
            sequence[i].f7X = f7AOIXSpinBox->value();
            // 'f7Y'
            sequence[i].f7Y = f7AOIYSpinBox->value();
        }
    }
    
    QDialog::show();
}


void SeqCtlDemoForm::closeEvent(QCloseEvent *e) {
    CamWindow *camWindow;
    AdjustmentsCloseEvent *closeEvent = new AdjustmentsCloseEvent();
    
    
    // verify parent-class
    if(QString(parent->className()) != "CamWindow") {
        logger->log("\nParent of 'AdjustmentsForm' is '%s', but 'CamWindow' is expected!",
               parent->className());
        QApplication::restoreOverrideCursor();
        return;
    }
    else {
        camWindow  = (CamWindow *)parent;
    }
    
    if(!videoRunning) {
        switch(QMessageBox::information(this, "Sequence Control",
                                        "Video stream is halted!\n\nRestart?",
                                        QMessageBox::Yes | QMessageBox::Default,
                                        QMessageBox::No | QMessageBox::Escape,
                                        QMessageBox::Cancel)) {
        case QMessageBox::Yes:
            dc1394->seqCtlEnable(id, false);
            QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
            camWindow->startCapture();
            QApplication::restoreOverrideCursor();
            videoRunning = true;
            break;
            
        case QMessageBox::Cancel:
            return;
            break;
            
        default:
            break;
        }
    }
    
    if(seqCtlStopPushButton->isEnabled()) {
        switch(QMessageBox::information(this, "Sequence Control",
                                        "Sequence Control is still running.\n\nStop now?",
                                        QMessageBox::Yes | QMessageBox::Default,
                                        QMessageBox::No | QMessageBox::Escape,
                                        QMessageBox::Cancel)) {
        case QMessageBox::Yes:
            // disable seq.-ctl.
            if(!dc1394->seqCtlEnable(id, false)) {
                logger->log("Error while trying to disable 'Sequence Control'");
            }
            break;
            
        case QMessageBox::Cancel:
            return;
            break;
            
        default:
            break;
            
        }
    }
    
    QApplication::postEvent(parent, closeEvent);
    
    e->accept();
}


void SeqCtlDemoForm::updateUI() {
    if(seqCtlStopPushButton->isEnabled())
        return;
    
    if(applyPushButton->isEnabled()) {
        switch(QMessageBox::warning(this, "Warning",
                                    "Some changes are not yet 'applied'!\nThey have to be 'applied' or 'discarded'\nbefore continuing with updating the UI!\n\n'Apply' before continuing?",
                                    QMessageBox::Yes | QMessageBox::Default,
                                    QMessageBox::No,
                                    QMessageBox::Cancel | QMessageBox::Escape)) {
        case QMessageBox::Cancel:
            return;
            break;
        case QMessageBox::Yes:
            // apply changes
            featuresApply();
            break;
        case QMessageBox::No:
            // discard changes
            featuresDiscard();
            break;
        default:
            logger->log("This message should never appear!!!");
            return;
            break;
        }
    }
    
    // re-read camera-information
    featureSet = dc1394->featureSet(id);
    
    // update gain UI-controls
    if(dc1394->featureReadOutCapable(id, FEATURE_GAIN - FEATURE_MIN) &&
       (featureSet.feature[FEATURE_GAIN - FEATURE_MIN]).available) {
        gainGroupBox->setEnabled(true);
        updateGain(featureSet.feature[FEATURE_GAIN - FEATURE_MIN]);
    }
    else {
        logger->log("'Gain' not available'");
        gainGroupBox->setEnabled(false);
    }
    
    // update shutter UI-controls
    if(dc1394->featureReadOutCapable(id, FEATURE_SHUTTER - FEATURE_MIN) &&
       (featureSet.feature[FEATURE_SHUTTER - FEATURE_MIN]).available) {
        exposureTimeGroupBox->setEnabled(true);
        updateShutter(featureSet.feature[FEATURE_SHUTTER - FEATURE_MIN]);
    }
    else {
        logger->log("'Shutter Speed' not available");
        exposureTimeGroupBox->setEnabled(false);
    }
    
    // update Format 7 UI-controls
    logger->log("\n\tNow updating F7...\n");
    if(updateF7UI)
        updateF7();
    
    // update sequence control UI-controls
    updateSeqCtl();
}


void SeqCtlDemoForm::updateGain(dc1394_feature_info featureInfo) {
    bool initSave = initialized;
    unsigned int minValue = featureInfo.min;
    unsigned int maxValue = featureInfo.max;
    unsigned int valueRange = maxValue - minValue;
    unsigned int steps = 1;
    unsigned int value = featureInfo.value;
    QFocusData *focusdata = focusData();
    bool autoCapable = (featureInfo.auto_capable == DC1394_TRUE);
    bool autoActive = (featureInfo.auto_active == DC1394_TRUE);
    bool onePush = (featureInfo.one_push == DC1394_TRUE);
    bool onOffCapable = (featureInfo.on_off_capable == DC1394_TRUE);
    bool isOn = (featureInfo.is_on == DC1394_TRUE);
    bool readoutCapable = (featureInfo.readout_capable == DC1394_TRUE);
    bool manualCapable = (featureInfo.manual_capable == DC1394_TRUE);
    
    
    // update UI from camera
    
    // On-check-box and OnePush-push-button
    if(onOffCapable) {
        gainOnCheckBox->setEnabled(autoCapable ? !autoActive : true);
        gainOnCheckBox->setChecked(isOn);
        
        gainOnePushPushButton->setEnabled(autoCapable ?
                                          (!autoActive && onePush && isOn) : (onePush && isOn));
        gainAutoModeCheckBox->setEnabled(autoCapable && isOn);
    }
    else {
        gainOnCheckBox->setEnabled(false);
        gainOnCheckBox->setChecked(false);
        
        gainOnePushPushButton->setEnabled(autoCapable ?
                                          (!autoActive && onePush) : onePush);
        gainAutoModeCheckBox->setEnabled(autoCapable);
    }
    
    // slider and spin-box
    if(readoutCapable && manualCapable && !autoActive) {
        gainSlider->setEnabled(onOffCapable ? (isOn && (valueRange > 0)) : (valueRange > 0));
        if(gainSpinBox->isEnabled() && gainSpinBox->hasFocus()) {
            focusdata->next()->setFocus();
        }
        gainSpinBox->setEnabled(onOffCapable ? (isOn && (valueRange > 0)) : (valueRange > 0));
    }
    else {
        gainSlider->setEnabled(false);
        if(gainSpinBox->isEnabled() && gainSpinBox->hasFocus()) {
            focusdata->next()->setFocus();
        }
        gainSpinBox->setEnabled(false);
        
        return;
    }
    
    if(valueRange > 0) {
        initialized = false;
        
        gainSlider->setRange(minValue, maxValue);
        gainSlider->setSteps(steps, steps);
        gainSpinBox->setRange(minValue, maxValue);
        gainSpinBox->setSteps(steps, steps);
        gainSpinBox->setSuffix("");
        
        gainSlider->setValue(value);
        gainSpinBox->setValue(value);
        
        initialized = initSave;
    }
}


void SeqCtlDemoForm::updateShutter(dc1394_feature_info featureInfo) {
    bool initSave = initialized;
    unsigned int minValue = featureInfo.min;
    unsigned int maxValue = featureInfo.max;
    unsigned int valueRange = maxValue - minValue;
    unsigned int steps = 1;
    unsigned int value = featureInfo.value;
    unsigned int timebase, offset;
    QFocusData *focusdata = focusData();
    QString suffix = "";
    bool autoCapable = (featureInfo.auto_capable == DC1394_TRUE);
    bool autoActive = (featureInfo.auto_active == DC1394_TRUE);
    bool onePush = (featureInfo.one_push == DC1394_TRUE);
    bool onOffCapable = (featureInfo.on_off_capable == DC1394_TRUE);
    bool isOn = (featureInfo.is_on == DC1394_TRUE);
    bool readoutCapable = (featureInfo.readout_capable == DC1394_TRUE);
    bool manualCapable = (featureInfo.manual_capable == DC1394_TRUE);
    
    
    // update UI from camera
    
    // On-check-box and OnePush-push-button
    if(featureInfo.on_off_capable) {
        shutterSpeedOnCheckBox->setEnabled(true);
        shutterSpeedOnCheckBox->setChecked(isOn);
        
        shutterSpeedOnePushPushButton->setEnabled(onePush && isOn);
        shutterSpeedAutoModeCheckBox->setEnabled(autoCapable && isOn);
    }
    else {
        shutterSpeedOnCheckBox->setEnabled(false);
        shutterSpeedOnCheckBox->setChecked(false);
        
        shutterSpeedOnePushPushButton->setEnabled(onePush);
        shutterSpeedAutoModeCheckBox->setEnabled(autoCapable);
    }
    
    
    // slider and spin-box
    if(readoutCapable && manualCapable && !autoActive) {
        shutterSpeedSlider->setEnabled(onOffCapable ? (isOn && (valueRange > 0)) : 
                                       (valueRange > 0));
        if(shutterSpeedSpinBox->isEnabled() && shutterSpeedSpinBox->hasFocus()) {
            focusdata->next()->setFocus();
        }
        shutterSpeedSpinBox->setEnabled(onOffCapable ? (isOn && (valueRange > 0)) :
                                        (valueRange > 0));
    }
    else {
        shutterSpeedSlider->setEnabled(false);
        if(shutterSpeedSpinBox->isEnabled() && shutterSpeedSpinBox->hasFocus()) {
            focusdata->next()->setFocus();
        }
        shutterSpeedSpinBox->setEnabled(false);
        
        offsetLineEdit->setText("");
        timebaseLineEdit->setText("");
        shutterFormulaLineEdit->setText("");
        expTimeLineEdit->setText("");
    }
    
    if(valueRange > 0) {
        initialized = false;
        
        shutterSpeedSlider->setRange(minValue, maxValue);
        shutterSpeedSlider->setSteps(steps, steps);
        shutterSpeedSpinBox->setRange(minValue, maxValue);
        shutterSpeedSpinBox->setSteps(steps, steps);
        shutterSpeedSpinBox->setSuffix("");
        
        shutterSpeedSlider->setValue(value);
        shutterSpeedSpinBox->setValue(value);
        
        dc1394->getShutterOffset(id, &offset);
        offsetLineEdit->setText(QString::number(offset) + "us");
        
        if(!dc1394->getTimebase(id, &timebase)) {
            // timebase (currently) not available
            logger->log("Unable to read current Timebase value while updating Shutter-UI-Controls!");
            shutterFormulaLineEdit->setText("Std. Shutter Value + 'Offset'");
            expTimeLineEdit->setText(
                    QString::number(shutterSpeedSpinBox->value() + offset) + "us");
            timebaseLineEdit->setText("");
        }
        else {
            shutterFormulaLineEdit->setText("Std. Shutter Value * Timebase + 'Offset'");
            expTimeLineEdit->setText(
                    QString::number((shutterSpeedSpinBox->value() * timebase) +
                                    offset) + "us");
            timebaseLineEdit->setText(QString::number(timebase) + "us");
        }
        
        initialized = initSave;
    }
    else {
        offsetLineEdit->setText("");
        timebaseLineEdit->setText("");
        shutterFormulaLineEdit->setText("");
        expTimeLineEdit->setText("");
    }
}


// F7
void SeqCtlDemoForm::updateF7() {
    if(!dc1394->f7Available(id)) {
        logger->log("F7 currently not available - disabling the UI-Controls");
        disableF7();
        return;
    }
    else
        f7AOIWarningTextLabel->setEnabled(false);
    
    updateF7AOI();
}


// special function needed by 'CamWindow' to re-enable UI-controls after 'F7-Update'
void SeqCtlDemoForm::enableUpdateF7() {
    updateF7UI = true;
    updateUI();
    this->setEnabled(true);
}


void SeqCtlDemoForm::disableF7() {
    logger->log("F7 is not available");
    f7AOIGroupBox->setEnabled(false);
    f7AOIWarningTextLabel->setEnabled(true);
}


void SeqCtlDemoForm::updateF7AOI() {
    unsigned int maxX, maxY, w, h, x, y;
    unsigned int maxXSize, maxYSize;
    unsigned int hUnit, vUnit, hPosUnit, vPosUnit;
    bool initSave;
    F7NotifyData notifyData;
    F7UpdateNotifyEvent *notifyEvent;
    
    
    if(f7Changed) {
        // UI-controls have been changed - applying changes to camera
        logger->log("\n\tF7 UI controls have changed - therefore updating camera...\n");
        
        notifyData.width = f7AOIWidthSpinBox->value();
        notifyData.height = f7AOIHeightSpinBox->value();
        notifyData.x = f7AOIXSpinBox->value();
        notifyData.y = f7AOIYSpinBox->value();
        notifyData.sender = QString("SeqCtlDemoForm");
        
        notifyEvent = new F7UpdateNotifyEvent(notifyData);
        
        QApplication::postEvent(parent, notifyEvent);
        
        QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
        sleep(3);
        QApplication::restoreOverrideCursor();
    }
    else {
        // refreshing UI-controls from camera-settings
        logger->log("\n\tUpdating F7 UI controls from camera...\n");
        
        f7AOIGroupBox->setEnabled(true);
        
        if(!dc1394->getF7MaxSize(id, 0, &maxX, &maxY)) {
            logger->log("reading maximum image size failed!");
            return;
        }
        
        if(!dc1394->getF7ImageSize(id, &w, &h)) {
            logger->log("reading current image size failed!");
            return;
        }
        
        if(!dc1394->getF7ImagePos(id, &x, &y)) {
            logger->log("reading current image position failed!");
            return;
        }
        
        if(!dc1394->getF7Units(id, &hUnit, &vUnit, &hPosUnit, &vPosUnit)) {
            logger->log("reading current unit values failed!");
            return;
        }
        
        logger->log("'updateF7AOI()': x=%d (max-x=%d), y=%d (max-y=%d), width=%d, height=%d",
               x, maxX, y, maxY, w, h);
        logger->log("H-Unit: %d, V-Unit: %d, H-Pos-Unit: %d, V-Pos-Unit: %d",
               hUnit, vUnit, hPosUnit, vPosUnit);
        
        initSave = initialized;
        initialized = false;
        
        f7AOIWidthSlider->setMinValue(0);
        f7AOIWidthSlider->setMaxValue(maxX);
        f7AOIWidthSlider->setSteps(hUnit, hUnit);
        f7AOIWidthSpinBox->setMinValue(0);
        f7AOIWidthSpinBox->setMaxValue(maxX);
        f7AOIWidthSpinBox->setSteps(hUnit, hUnit);
        
        f7AOIHeightSlider->setMinValue(0);
        f7AOIHeightSlider->setMaxValue(maxY);
        f7AOIHeightSlider->setSteps(vUnit, vUnit);
        f7AOIHeightSpinBox->setMinValue(0);
        f7AOIHeightSpinBox->setMaxValue(maxY);
        f7AOIHeightSpinBox->setSteps(vUnit, vUnit);
        
        f7AOIXSlider->setMinValue(0);
        f7AOIXSlider->setMaxValue(maxX - w);
        f7AOIXSlider->setSteps((hPosUnit == 0 ? hUnit : hPosUnit),
                               (hPosUnit == 0 ? hUnit : hPosUnit));
        f7AOIXSpinBox->setMinValue(0);
        f7AOIXSpinBox->setMaxValue(maxX - w);
        f7AOIXSpinBox->setSteps((hPosUnit == 0 ? hUnit : hPosUnit),
                                (hPosUnit == 0 ? hUnit : hPosUnit));
        
        f7AOIYSlider->setMinValue(0);
        f7AOIYSlider->setMaxValue(maxY - h);
        f7AOIYSlider->setSteps((vPosUnit == 0 ? vUnit : vPosUnit),
                               (vPosUnit == 0 ? vUnit : vPosUnit));
        f7AOIYSpinBox->setMinValue(0);
        f7AOIYSpinBox->setMaxValue(maxY - h);
        f7AOIYSpinBox->setSteps((vPosUnit == 0 ? vUnit : vPosUnit),
                                (vPosUnit == 0 ? vUnit : vPosUnit));
        
        // current values of the sliders and spin-boxes
        f7CorrectValue = (unsigned int)f7AOIWidthSlider->value() != w;
        f7AOIWidthSlider->setValue(w);
        f7CorrectValue = (unsigned int)f7AOIWidthSpinBox->value() != w;
        f7AOIWidthSpinBox->setValue(w);
        f7CorrectValue = (unsigned int)f7AOIHeightSlider->value() != h;
        f7AOIHeightSlider->setValue(h);
        f7CorrectValue = (unsigned int)f7AOIHeightSpinBox->value() != h;
        f7AOIHeightSpinBox->setValue(h);
        
        f7CorrectValue = (unsigned int)f7AOIXSlider->value() != x;
        f7AOIXSlider->setValue(x);
        f7CorrectValue = (unsigned int)f7AOIXSpinBox->value() != x;
        f7AOIXSpinBox->setValue(x);
        f7CorrectValue = (unsigned int)f7AOIYSlider->value() != y;
        f7AOIYSlider->setValue(y);
        f7CorrectValue = (unsigned int)f7AOIYSpinBox->value() != y;
        f7AOIYSpinBox->setValue(y);
        
        // graphical representation of the current region
        maxXSize = f7AOIImageFrame->width();
        maxYSize = f7AOIImageFrame->height();
        
        f7AOISectorFrame->resize((maxXSize - 6) * w / maxX, (maxYSize - 6) * h / maxY);
//        f7AOISectorFrame->move(3, 3);
        f7AOISectorFrame->move((x * (maxXSize - 3) / maxX) + 3,
                               (y * (maxYSize - 3) / maxY) + 3);
        
        initialized = initSave;
    }
}


// sequence control UI-controls
void SeqCtlDemoForm::updateSeqCtl() {
    unsigned int seqLimit, seqLength, imgNo;
    bool status, autoRewind, aInc;
    bool initSave = initialized;
    
    
    // update SeqCtl UI-controls from camera
    if(!dc1394->seqCtlStatus(id, &status, &autoRewind, &aInc)) {
        logger->log("Errror while trying to read current 'sequence control' status!");
        seqCtlGroupBox->setEnabled(false);
        return;
    }
    
    if(!dc1394->getSeqCtlInfo(id, &seqLimit, &seqLength, &imgNo)) {
        logger->log("Error while trying to read 'sequence control' limit and length!");
        seqCtlGroupBox->setEnabled(false);
        return;
    }
    
    if(seqLength == 0) {
        logger->log("Current 'sequence-length' is '0' - correcting to '1'...");
        if(!dc1394->setSeqCtlLength(id, 1)) {
            logger->log("Error while trying to correct 'Sequence-Length' ('0' -> '1')!");
            return;
        }
        // and 're-read' SequenceControl status
        updateSeqCtl();
        return;
    }
    
    logger->log("Current 'sequence control' state: %s", status ? "enabled" : "disabled");
    logger->log("Current 'sequence length': %d (Limit: %d)", seqLength, seqLimit);
    logger->log("Current 'image number': %d", imgNo);
    
    initialized = false;
    
    seqCtlEnableCheckBox->setEnabled(seqCtlLengthSpinBox->value() > 0);
    
    // 'Stop' button availability depends on the 'ON_OFF' flag in the camera
    // because the main funcionality of this button is to 'disable' the sequence
    seqCtlStopPushButton->setEnabled(status);
    
    // keep current 'status' (independent of the 'ON_OFF' flag in the camera
    status = seqCtlEnableCheckBox->isChecked();
    
    seqCtlAutoRewindCheckBox->setEnabled(status);
    
    seqCtlAutoIncCheckBox->setEnabled(status);
    
    seqCtlLengthSpinBox->setEnabled(status);
    seqCtlLengthSpinBox->setMaxValue(seqLimit);
    
    seqCtlImgNoSpinBox->setEnabled(status);
    seqCtlImgNoSpinBox->setMaxValue(seqCtlLengthSpinBox->value());
    
    seqCtlApplyPushButton->setEnabled(status);
    seqCtlRunPushButton->setEnabled(status);
    
    initialized = initSave;
}


void SeqCtlDemoForm::exitAccept() {
    CamWindow *camWindow;
    AdjustmentsCloseEvent *closeEvent = new AdjustmentsCloseEvent();
    
    
    // apply modifications
    if(f7Changed)
        featuresApply();
    
    // verify parent-class
    if(QString(parent->className()) != "CamWindow") {
        logger->log("\nParent of 'AdjustmentsForm' is '%s', but 'CamWindow' is expected!",
               parent->className());
        QApplication::restoreOverrideCursor();
        return;
    }
    else {
        camWindow  = (CamWindow *)parent;
    }
    
    if(!videoRunning) {
        switch(QMessageBox::information(this, "Sequence Control",
                                        "Video stream is halted!\n\nRestart Video?",
                                        QMessageBox::Yes | QMessageBox::Default,
                                        QMessageBox::No | QMessageBox::Escape,
                                        QMessageBox::Cancel)) {
        case QMessageBox::Yes:
            QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
            camWindow->startCapture();
            QApplication::restoreOverrideCursor();
            videoRunning = true;
            break;
            
        case QMessageBox::Cancel:
            return;
            break;
            
        default:
            break;
        }
    }
    
    if(seqCtlStopPushButton->isEnabled()) {
        switch(QMessageBox::information(this, "Sequence Control",
                                        "Sequence Control is still active.\n\nDeactivate ?",
                                        QMessageBox::Yes | QMessageBox::Default,
                                        QMessageBox::No | QMessageBox::Escape,
                                        QMessageBox::Cancel)) {
        case QMessageBox::Yes:
            // disable seq.-ctl.
            seqCtlStop();
            sleep(1);
            break;
            
        case QMessageBox::Cancel:
            return;
            break;
            
        default:
            break;
                
        }
    }
    
    QApplication::postEvent(parent, closeEvent);
    
    accept();
}


void SeqCtlDemoForm::exitReject() {
    CamWindow *camWindow;
    AdjustmentsCloseEvent *closeEvent = new AdjustmentsCloseEvent();
    
    if(f7Changed) {
        // ask for rejecting them
        if(QMessageBox::question(this, "'Unapplied' changes'",
                                 "Some changes are not yet 'applied'\n\nExit anyway?",
                                 QMessageBox::Yes | QMessageBox::Default,
                                 QMessageBox::No | QMessageBox::Escape,
                                 QMessageBox::NoButton)
            == QMessageBox::No) {
            
            // just exit this function, if the changes should not be rejected
            return;
        }
    }
    
    // discard changes for next-time-opening
    featuresDiscard();
    
    // verify parent-class
    if(QString(parent->className()) != "CamWindow") {
        logger->log("\nParent of 'AdjustmentsForm' is '%s', but 'CamWindow' is expected!",
               parent->className());
        QApplication::restoreOverrideCursor();
        return;
    }
    else {
        camWindow  = (CamWindow *)parent;
    }
    
    if(!videoRunning) {
        switch(QMessageBox::information(this, "Sequence Control",
                                        "Video stream is halted!\n\nRestart Video?",
                                        QMessageBox::Yes | QMessageBox::Default,
                                        QMessageBox::No | QMessageBox::Escape,
                                        QMessageBox::Cancel)) {
        case QMessageBox::Yes:
            QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
            camWindow->startCapture();
            QApplication::restoreOverrideCursor();
            videoRunning = true;
            break;
            
        case QMessageBox::Cancel:
            return;
            break;
            
        default:
            break;
        }
    }
    
    if(seqCtlStopPushButton->isEnabled()) {
        switch(QMessageBox::information(this, "Sequence Control",
                                        "Sequence Control is still active.\n\nDeactivate ?",
                                        QMessageBox::Yes | QMessageBox::Default,
                                        QMessageBox::No | QMessageBox::Escape,
                                        QMessageBox::Cancel)) {
        case QMessageBox::Yes:
            // disable seq.-ctl.
            seqCtlStop();
            sleep(1);
            break;
            
        case QMessageBox::Cancel:
            return;
            break;
            
        default:
            break;
                
        }
    }
    
    QApplication::postEvent(parent, closeEvent);
    
    reject();
}


void SeqCtlDemoForm::featuresApply() {
    bool stdFeatureChanged = false;
    
    
    // re-read 'featureSet'
    featureSet = dc1394->featureSet(id);
    
    if(f7Changed) {
        this->setEnabled(false);
        updateF7UI = false;
        updateF7();
        f7Changed = false;
    }
    
    // disable the 'apply' and 'discard' buttons
    applyPushButton->setEnabled(false);
    discardPushButton->setEnabled(false);
    
    // update all UI-controls
    updateUI();
    
    // enable UI-controls
    if((!updateF7UI) || (stdFeatureChanged)) {
        this->setEnabled(true);
    }
}


void SeqCtlDemoForm::featuresDiscard() {
    // re-read 'featureSet'
    featureSet = dc1394->featureSet(id);
    
    if(f7Changed) {
        updateF7UI = false;
        f7Changed = false;
        updateF7();
        logger->log("Discarded F7 changes");
    }
    
    
    applyPushButton->setEnabled(false);
    discardPushButton->setEnabled(false);
}


// UI-control callbacks

// Gain
void SeqCtlDemoForm::gainSliderChanged(int newValue) {
    bool initSave = initialized;
    
    
    if(initialized) {
        initialized = false;
        gainSpinBox->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, FEATURE_GAIN - FEATURE_MIN, newValue))
            logger->log("Error while trying to change GAIN to %d", gainSpinBox->value());
    }
}


void SeqCtlDemoForm::gainSpinBoxChanged(int newValue) {
    bool initSave = initialized;
    
    
    if(initialized) {
        initialized = false;
        gainSlider->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, FEATURE_GAIN - FEATURE_MIN, newValue))
            logger->log("Error while trying to change GAIN to %d", gainSpinBox->value());
    }
}


void SeqCtlDemoForm::gainOnChecked(bool newValue) {
    bool initSave = initialized;
    unsigned int featureIndex = FEATURE_GAIN - FEATURE_MIN;
    
    if(initialized) {
        if(!dc1394->setFeatureOn(id, featureIndex, newValue)) {
            logger->log("Error while trying to %s GAIN!", newValue ? "enable" : "disable");
            initialized = false;
            gainOnCheckBox->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void SeqCtlDemoForm::gainAutoChecked(bool enable) {
    bool initSave = initialized;
    QFocusData *focusdata = focusData();
    dc1394_feature_info featureInfo = dc1394->featureInfo(id, FEATURE_GAIN - FEATURE_MIN);
    bool onePush = (featureInfo.one_push == DC1394_TRUE);
    bool onOffCapable = (featureInfo.on_off_capable == DC1394_TRUE);
    bool isOn = (featureInfo.is_on == DC1394_TRUE);
    bool readoutCapable = (featureInfo.readout_capable == DC1394_TRUE);
    bool manualCapable = (featureInfo.manual_capable == DC1394_TRUE);
    
    
    if(!initialized)
        return;
    
    if(!dc1394->setFeatureAutoMode(id, FEATURE_GAIN - FEATURE_MIN,
                                enable)) {
        logger->log("Error while trying to %s GAIN-AutoMode",
               enable ? "enable" : "disable");
        initialized = false;
        gainAutoModeCheckBox->setChecked(!enable);
        initialized = initSave;
        return;
    }
    
    // enable / disable other controls
    if(!enable) {
        if(readoutCapable && manualCapable) {
            gainSlider->setEnabled(onOffCapable ? isOn : true);
            if(gainSpinBox->isEnabled() && gainSpinBox->hasFocus()) {
                focusdata->next()->setFocus();
            }
            gainSpinBox->setEnabled(onOffCapable ? isOn : true);
        }
        if(onePush)
            gainOnePushPushButton->setEnabled(true);
    }
    else {
        gainSlider->setEnabled(false);
        if(gainSpinBox->isEnabled() && gainSpinBox->hasFocus()) {
            focusdata->next()->setFocus();
        }
        gainSpinBox->setEnabled(false);
        gainOnePushPushButton->setEnabled(false);
    }
        
    updateUI();
}


void SeqCtlDemoForm::gainOnePushActivated() {
    if(!dc1394->featureOnePush(id, FEATURE_GAIN - FEATURE_MIN)) {
        logger->log("Error while trying to set-up gain automatically");
        return;
    }
    
    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
    
    // wait for the camera to be ready after the 'OnePush' procedure
    do {
        // read current camera-features-status
        featureSet = dc1394->featureSet(id);
    } while((featureSet.feature[FEATURE_GAIN - FEATURE_MIN]).one_push_active);
    
    QApplication::restoreOverrideCursor();
}


// Shutter
void SeqCtlDemoForm::shutterSpeedSliderChanged(int newValue) {
    bool initSave = initialized;
    QString timebase = timebaseLineEdit->text();
    QString offset = offsetLineEdit->text();
    
    
    if(initialized) {
        initialized = false;
        shutterSpeedSpinBox->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, FEATURE_SHUTTER - FEATURE_MIN, newValue))
            logger->log("Trying to change Shutter failed!");
        
        shutterFormulaLineEdit->setText("Std. Shutter Value * Timebase + 'Offset'");
        timebase.truncate(timebase.length() - 2);
        offset.truncate(offset.length() - 2);
        expTimeLineEdit->setText(QString::number((newValue *
                                                  timebase.toUInt()) +
                                                 offset.toUInt()) + "us");
    }
}


void SeqCtlDemoForm::shutterSpeedSpinBoxChanged(int newValue) {
    bool initSave = initialized;
    QString timebase = timebaseLineEdit->text();
    QString offset = offsetLineEdit->text();
    
    
    if(initialized) {
        initialized = false;
        shutterSpeedSlider->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, FEATURE_SHUTTER - FEATURE_MIN, newValue))
            logger->log("Trying to change Shutter failed!");
        
        shutterFormulaLineEdit->setText("Std. Shutter Value * Timebase + 'Offset'");
        timebase.truncate(timebase.length() - 2);
        offset.truncate(offset.length() - 2);
        expTimeLineEdit->setText(QString::number((newValue *
                                                  timebase.toUInt()) +
                                                 offset.toUInt()) + "us");
    }
}


void SeqCtlDemoForm::shutterSpeedOnChecked(bool newValue) {
    bool initSave = initialized;
    unsigned int featureIndex = FEATURE_SHUTTER - FEATURE_MIN;
    
    if(initialized) {
        if(!dc1394->setFeatureOn(id, featureIndex, newValue)) {
            logger->log("Error while trying to %s SHUTTER!", newValue ? "enable" : "disable");
            initialized = false;
            gainOnCheckBox->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void SeqCtlDemoForm::shutterSpeedAutoChecked(bool enable) {
    bool initSave = initialized;
    QFocusData *focusdata = focusData();
    dc1394_feature_info featureInfo = dc1394->featureInfo(id, FEATURE_SHUTTER - FEATURE_MIN);
    bool onePush = (featureInfo.one_push == DC1394_TRUE);
    bool onOffCapable = (featureInfo.on_off_capable == DC1394_TRUE);
    bool isOn = (featureInfo.is_on == DC1394_TRUE);
    bool readoutCapable = (featureInfo.readout_capable == DC1394_TRUE);
    bool manualCapable = (featureInfo.manual_capable == DC1394_TRUE);
    
    
    if(!initialized)
        return;
    
    if(!dc1394->setFeatureAutoMode(id, FEATURE_SHUTTER - FEATURE_MIN,
                                enable)) {
        logger->log("Error while trying to %s SHUTTER-AutoMode",
               enable ? "enable" : "disable");
        initialized = false;
        shutterSpeedAutoModeCheckBox->setChecked(!enable);
        initialized = initSave;
        return;
    }
    
    // enable / disable other controls
    if(!enable) {
        if(readoutCapable && manualCapable) {
            shutterSpeedSlider->setEnabled(onOffCapable ? isOn : true);
            if(shutterSpeedSpinBox->isEnabled() && shutterSpeedSpinBox->hasFocus()) {
                focusdata->next()->setFocus();
            }
            shutterSpeedSpinBox->setEnabled(onOffCapable ? isOn : true);
        }
        if(onePush)
            shutterSpeedOnePushPushButton->setEnabled(true);
    }
    else {
        shutterSpeedSlider->setEnabled(false);
        if(shutterSpeedSpinBox->isEnabled() && shutterSpeedSpinBox->hasFocus()) {
            focusdata->next()->setFocus();
        }
        shutterSpeedSpinBox->setEnabled(false);
        shutterSpeedOnePushPushButton->setEnabled(false);
    }
        
    updateUI();
}


void SeqCtlDemoForm::shutterSpeedOnePushActivated() {
    if(!dc1394->featureOnePush(id, FEATURE_SHUTTER - FEATURE_MIN)) {
        logger->log("Error while trying to set-up shutter automatically");
        return;
    }
    
    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
    
    // wait for the camera to be ready after the 'OnePush' procedure
    do {
        // read current camera-features-status
        featureSet = dc1394->featureSet(id);
    } while((featureSet.feature[FEATURE_SHUTTER - FEATURE_MIN]).one_push_active);
    
    QApplication::restoreOverrideCursor();
}


// F7
void SeqCtlDemoForm::f7AOIWidthSliderChanged(int newValue) {
    unsigned int maxXSize = f7AOIImageFrame->width();
    unsigned int maxYSize = f7AOIImageFrame->height();
    unsigned int maxX = f7AOIWidthSlider->maxValue();
    unsigned int h = f7AOIHeightSlider->value();
    unsigned int maxY = f7AOIHeightSlider->maxValue();
    unsigned int hUnit, vUnit, hPosUnit, vPosUnit;
    unsigned int diff;
    bool initSave = initialized;
    
    
    if(f7CorrectValue) {
        f7CorrectValue = false;
        return;
    }
    
    if(!dc1394->getF7Units(id, &hUnit, &vUnit, &hPosUnit, &vPosUnit)) {
        logger->log("unable to read Unit-Values from camera");
    }
    else {
        logger->log("Units: %d x %d (%d, %d)", hUnit, vUnit,
               hPosUnit, vPosUnit);
        
        if((diff = newValue % hUnit) != 0) {
            logger->log("new value (%d) does not match current 'units' (remainder: %d) - fixing to '%d'",
                   newValue, newValue % hUnit, (newValue / hUnit) * hUnit);
            newValue -= diff;
            
            f7CorrectValue = true;
            f7AOIWidthSlider->setValue(newValue);
        }
    }
    
    f7AOISectorFrame->resize((maxXSize - 6) * newValue / maxX, (maxYSize - 6) * h / maxY);
    f7AOIXSlider->setMaxValue(maxX - newValue);

    if(initialized) {
        // value manually changed -> propagate to SpinBox
        initialized = false;
        f7AOIWidthSpinBox->setValue(newValue);
        initialized = initSave;
        
        // enable the apply- and discard-button
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        // set feature-changed-flag
        f7Changed = true;
    }
}


void SeqCtlDemoForm::f7AOIWidthSpinBoxChanged(int newValue) {
    unsigned int maxXSize = f7AOIImageFrame->width();
    unsigned int maxYSize = f7AOIImageFrame->height();
    unsigned int maxX = f7AOIWidthSpinBox->maxValue();
    unsigned int h = f7AOIHeightSpinBox->value();
    unsigned int maxY = f7AOIHeightSpinBox->maxValue();
    unsigned int hUnit, vUnit, hPosUnit, vPosUnit;
    unsigned int diff;
    bool initSave = initialized;
    
    
    if(f7CorrectValue) {
        f7CorrectValue = false;
        return;
    }
    
    if(!dc1394->getF7Units(id, &hUnit, &vUnit, &hPosUnit, &vPosUnit)) {
        logger->log("unable to read Unit-Values from camera");
    }
    else {
        logger->log("Units: %d x %d (%d, %d)", hUnit, vUnit,
               hPosUnit, vPosUnit);
        
        if((diff = newValue % hUnit) != 0) {
            logger->log("new value (%d) does not match current 'units' - fixing to '%d'",
                   newValue, (newValue / hUnit) * hUnit);
            newValue -= diff;
            
            f7CorrectValue = true;
            f7AOIWidthSpinBox->setValue(newValue);
        }
    }
    
    f7AOISectorFrame->resize((maxXSize - 6) * newValue / maxX, (maxYSize - 6) * h / maxY);
    f7AOIXSpinBox->setMaxValue(maxX - newValue);

    if(initialized) {
        initialized = false;
        f7AOIWidthSlider->setValue(newValue);
        initialized = initSave;
        
        // enable the apply- and discard-button
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        // set feature-changed-flag
        f7Changed = true;
    }
}


void SeqCtlDemoForm::f7AOIHeightSliderChanged(int newValue) {
    unsigned int maxXSize = f7AOIImageFrame->width();
    unsigned int maxYSize = f7AOIImageFrame->height();
    unsigned int w = f7AOIWidthSlider->value();
    unsigned int maxX = f7AOIWidthSlider->maxValue();
    unsigned int maxY = f7AOIHeightSlider->maxValue();
    unsigned int hUnit, vUnit, hPosUnit, vPosUnit;
    unsigned int diff;
    bool initSave = initialized;
    
    
    if(f7CorrectValue) {
        f7CorrectValue = false;
        return;
    }
    
    if(!dc1394->getF7Units(id, &hUnit, &vUnit, &hPosUnit, &vPosUnit)) {
        logger->log("unable to read Unit-Values from camera");
    }
    else {
        if((diff = newValue % vUnit) != 0) {
            logger->log("new value (%d) does not match current 'units' - fixing to '%d'",
                   newValue, (newValue / vUnit) * vUnit);
            newValue -= diff;
            
            f7CorrectValue = true;
            f7AOIHeightSlider->setValue(newValue);
        }
    }
    
    f7AOISectorFrame->resize((maxXSize - 6) * w / maxX, (maxYSize - 6) * newValue / maxY);
    f7AOIYSlider->setMaxValue(maxY - newValue);

    if(initialized) {
        initialized = false;
        f7AOIHeightSpinBox->setValue(newValue);
        initialized = initSave;
        
        // enable the apply- and discard-button
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        // set feature-changed-flag
        f7Changed = true;
    }
}


void SeqCtlDemoForm::f7AOIHeightSpinBoxChanged(int newValue) {
    unsigned int maxXSize = f7AOIImageFrame->width();
    unsigned int maxYSize = f7AOIImageFrame->height();
    unsigned int w = f7AOIWidthSpinBox->value();
    unsigned int maxX = f7AOIWidthSpinBox->maxValue();
    unsigned int maxY = f7AOIHeightSpinBox->maxValue();
    unsigned int hUnit, vUnit, hPosUnit, vPosUnit;
    unsigned int diff;
    bool initSave = initialized;
    
    
    if(f7CorrectValue) {
        logger->log("F7CorrectSliderValue == true");
        f7CorrectValue = false;
        return;
    }
    
    if(!dc1394->getF7Units(id, &hUnit, &vUnit, &hPosUnit, &vPosUnit)) {
        logger->log("unable to read Unit-Values from camera");
    }
    else {
        if((diff = newValue % vUnit) != 0) {
            logger->log("new value (%d) does not match current 'units' - fixing to '%d'",
                   newValue, (newValue / vUnit) * vUnit);
            newValue -= diff;
            
            f7CorrectValue = true;
            f7AOIHeightSpinBox->setValue(newValue);
        }
    }
    
    f7AOISectorFrame->resize((maxXSize - 6) * w / maxX, (maxYSize - 6) * newValue / maxY);
    f7AOIYSpinBox->setMaxValue(maxY - newValue);

    if(initialized) {
        initialized = false;
        f7AOIHeightSlider->setValue(newValue);
        initialized = initSave;
        
        // enable the apply- and discard-button
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        // set feature-changed-flag
        f7Changed = true;
    }
}


void SeqCtlDemoForm::f7AOIXSliderChanged(int newValue) {
    unsigned int maxXSize = f7AOIImageFrame->width();
    unsigned int maxYSize = f7AOIImageFrame->height();
    unsigned int maxX = f7AOIWidthSlider->maxValue();
    unsigned int y = f7AOIYSlider->value();
    unsigned int maxY = f7AOIHeightSlider->maxValue();
    unsigned int hUnit, vUnit, hPosUnit, vPosUnit;
    unsigned int diff;
    bool initSave = initialized;
    
    
    if(f7CorrectValue) {
        logger->log("F7CorrectSliderValue == true");
        f7CorrectValue = false;
        return;
    }
    
    if(!dc1394->getF7Units(id, &hUnit, &vUnit, &hPosUnit, &vPosUnit)) {
        logger->log("unable to read Unit-Values from camera");
    }
    else {
        if((diff = newValue % (hPosUnit == 0 ? hUnit : hPosUnit)) != 0) {
            logger->log("new value (%d) does not match current 'units' - fixing to '%d'",
                   newValue, (newValue / (hPosUnit == 0 ? hUnit : hPosUnit)) * 
                   (hPosUnit == 0 ? hUnit : hPosUnit));
            newValue -= diff;
            
            f7CorrectValue = true;
            f7AOIXSlider->setValue(newValue);
        }
    }
    
    f7AOISectorFrame->move((newValue * (maxXSize - 3) / maxX) + 3,
                           (y * (maxYSize - 3) / maxY) + 3);

    if(initialized) {
        initialized = false;
        f7AOIXSpinBox->setValue(newValue);
        initialized = initSave;
        
        // enable the apply- and discard-button
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        // set feature-changed-flag
        f7Changed = true;
    }
}


void SeqCtlDemoForm::f7AOIXSpinBoxChanged(int newValue) {
    unsigned int maxXSize = f7AOIImageFrame->width();
    unsigned int maxYSize = f7AOIImageFrame->height();
    unsigned int maxX = f7AOIWidthSpinBox->maxValue();
    unsigned int y = f7AOIYSpinBox->value();
    unsigned int maxY = f7AOIHeightSpinBox->maxValue();
    unsigned int hUnit, vUnit, hPosUnit, vPosUnit;
    unsigned int diff;
    bool initSave = initialized;
    
    
    if(f7CorrectValue) {
        logger->log("F7CorrectSliderValue == true");
        f7CorrectValue = false;
        return;
    }
    
    if(!dc1394->getF7Units(id, &hUnit, &vUnit, &hPosUnit, &vPosUnit)) {
        logger->log("unable to read Unit-Values from camera");
    }
    else {
        if((diff = newValue % (hPosUnit == 0 ? hUnit : hPosUnit)) != 0) {
            logger->log("new value (%d) does not match current 'units' - fixing to '%d'",
                   newValue, (newValue / (hPosUnit == 0 ? hUnit : hPosUnit)) * 
                   (hPosUnit == 0 ? hUnit : hPosUnit));
            newValue -= diff;
            
            f7CorrectValue = true;
            f7AOIXSpinBox->setValue(newValue);
        }
    }
    
    f7AOISectorFrame->move((newValue * (maxXSize - 3) / maxX) + 3,
                           (y * (maxYSize - 3) / maxY) + 3);

    if(initialized) {
        initialized = false;
        f7AOIXSlider->setValue(newValue);
        initialized = initSave;
        
        // enable the apply- and discard-button
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        // set feature-changed-flag
        f7Changed = true;
    }
}


void SeqCtlDemoForm::f7AOIYSliderChanged(int newValue) {
    unsigned int maxXSize = f7AOIImageFrame->width();
    unsigned int maxYSize = f7AOIImageFrame->height();
    unsigned int x = f7AOIXSlider->value();
    unsigned int maxX = f7AOIWidthSlider->maxValue();
    unsigned int maxY = f7AOIHeightSlider->maxValue();
    unsigned int hUnit, vUnit, hPosUnit, vPosUnit;
    unsigned int diff;
    bool initSave = initialized;
    
    
    if(f7CorrectValue) {
        logger->log("F7CorrectSliderValue == true");
        f7CorrectValue = false;
        return;
    }
    
    if(!dc1394->getF7Units(id, &hUnit, &vUnit, &hPosUnit, &vPosUnit)) {
        logger->log("unable to read Unit-Values from camera");
    }
    else {
        if((diff = newValue % (vPosUnit == 0 ? vUnit : vPosUnit)) != 0) {
            logger->log("new value (%d) does not match current 'units' - fixing to '%d'",
                   newValue, (newValue / (vPosUnit == 0 ? vUnit : vPosUnit)) * 
                   (vPosUnit == 0 ? hUnit : hPosUnit));
            newValue -= diff;
            
            f7CorrectValue = true;
            f7AOIYSlider->setValue(newValue);
        }
    }
    
    f7AOISectorFrame->move((x * (maxXSize - 3) / maxX) + 3,
                           (newValue * (maxYSize - 3) / maxY) + 3);

    if(initialized) {
        initialized = false;
        f7AOIYSpinBox->setValue(newValue);
        initialized = initSave;
        
        // enable the apply- and discard-button
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        // set feature-changed-flag
        f7Changed = true;
    }
}


void SeqCtlDemoForm::f7YSpinBoxChanged(int newValue) {
    unsigned int maxXSize = f7AOIImageFrame->width();
    unsigned int maxYSize = f7AOIImageFrame->height();
    unsigned int x = f7AOIXSpinBox->value();
    unsigned int maxX = f7AOIWidthSpinBox->maxValue();
    unsigned int maxY = f7AOIHeightSpinBox->maxValue();
    unsigned int hUnit, vUnit, hPosUnit, vPosUnit;
    unsigned int diff;
    bool initSave = initialized;
    
    
    if(f7CorrectValue) {
        logger->log("F7CorrectSliderValue == true");
        f7CorrectValue = false;
        return;
    }
    
    if(!dc1394->getF7Units(id, &hUnit, &vUnit, &hPosUnit, &vPosUnit)) {
        logger->log("unable to read Unit-Values from camera");
    }
    else {
        if((diff = newValue % (vPosUnit == 0 ? vUnit : vPosUnit)) != 0) {
            logger->log("new value (%d) does not match current 'units' - fixing to '%d'",
                   newValue, (newValue / (vPosUnit == 0 ? vUnit : vPosUnit)) * 
                   (vPosUnit == 0 ? hUnit : hPosUnit));
            newValue -= diff;
            
            f7CorrectValue = true;
            f7AOIYSpinBox->setValue(newValue);
        }
    }
    
    f7AOISectorFrame->move((x * (maxXSize - 3) / maxX) + 3,
                           (newValue * (maxYSize - 3) / maxY) + 3);

    if(initialized) {
        initialized = false;
        f7AOIYSlider->setValue(newValue);
        initialized = initSave;
        
        // enable the apply- and discard-button
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        // set feature-changed-flag
        f7Changed = true;
    }
}


void SeqCtlDemoForm::f7AOIMaximize() {
    bool initSave = initialized;
    
    
    initialized = false;
    
     f7AOIWidthSlider->setValue(f7AOIWidthSlider->maxValue());
     f7AOIWidthSpinBox->setValue(f7AOIWidthSpinBox->maxValue());
    
     f7AOIHeightSlider->setValue(f7AOIHeightSlider->maxValue());
     f7AOIHeightSpinBox->setValue(f7AOIHeightSpinBox->maxValue());
    
    initialized = initSave;
    
    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
    sleep(3);
    QApplication::restoreOverrideCursor();
    
    f7Changed = true;
    featuresApply();
}


void SeqCtlDemoForm::seqCtlEnableChecked(bool newState) {
    bool initSave = initialized;
    
    
    if(!initialized)
        return;
    
    // if the 'SeqCtl' should be disabled while the sequence is still active
    // the sequence will be deactivated
    if(!newState && seqCtlStopPushButton->isEnabled()) {
        if(QMessageBox::warning(this, "Sequence Control",
                                "Sequence is still running!\n\nStop running sequence?",
                                QMessageBox::Yes | QMessageBox::Default,
                                QMessageBox::No | QMessageBox::Escape,
                                QMessageBox::NoButton) == QMessageBox::No) {
            initialized = false;
            seqCtlEnableCheckBox->setChecked(true);
            initialized = initSave;
            return;
        }
        
        seqCtlStop();
        
        return;
    }
    
    // if feature is 're-activated' (i.e. F7 is disabled at the time of switching on)
    // a new F7-size can be selected (afterwards the whole new sequence will
    // be fixed at that size again)
    if(newState && !f7AOIWidthSlider->isEnabled()) {
        f7AOIWidthSlider->setEnabled(true);
        f7AOIWidthSpinBox->setEnabled(true);
        f7AOIHeightSlider->setEnabled(true);
        f7AOIHeightSpinBox->setEnabled(true);
    }
    
    updateSeqCtl();
}


void SeqCtlDemoForm::seqCtlLengthChanged(int newValue) {
    seqCtlImgNoSpinBox->setMaxValue(newValue);
}


void SeqCtlDemoForm::seqCtlImgNoChanged(int newValue) {
    // change UI-Controls and camera to the values stored in image-buffer-enty 'newValue'
    
    // write values to camera...
    if(sequence[newValue - 1].gainAvailable) {
        if(!dc1394->setFeatureValue(id, FEATURE_GAIN - FEATURE_MIN,
                                    sequence[newValue - 1].gainLevel)) {
            logger->log("Error while trying to set GAIN-Level " +
                        QString::number(sequence[newValue - 1].gainLevel));
        }
    }
    if(sequence[newValue - 1].shutterAvailable) {
        if(!dc1394->setFeatureValue(id, FEATURE_SHUTTER - FEATURE_MIN,
                                    sequence[newValue - 1].shutterValue)) {
            logger->log("Error while trying to set SHUTTER-Speed " +
                        QString::number(sequence[newValue - 1].shutterValue));
        }
    }
    
    if(sequence[newValue - 1].f7Available) {
        f7AOIWidthSlider->setValue(sequence[newValue -1].f7Width);
        f7AOIWidthSpinBox->setValue(sequence[newValue -1].f7Width);
        
        f7AOIHeightSlider->setValue(sequence[newValue -1].f7Height);
        f7AOIHeightSpinBox->setValue(sequence[newValue -1].f7Height);
        
        f7AOIXSlider->setValue(sequence[newValue -1].f7X);
        f7AOIXSpinBox->setValue(sequence[newValue -1].f7X);
        
        f7AOIYSlider->setValue(sequence[newValue -1].f7Y);
        f7AOIYSpinBox->setValue(sequence[newValue -1].f7Y);
        
        // 'updateUI' (incl. F7)
        f7Changed = true;
        featuresApply();
    }
    else {
        // if F7 is not available - then at least update the other UI controls
        updateUI();
    }
}


void SeqCtlDemoForm::seqCtlApply() {
    unsigned int index = seqCtlImgNoSpinBox->value() - 1;
    
    
    // save current SeqCtl-UI-controls-status in respective buffer-entry (No. 'ImgNo')
    if(sequence[index].gainAvailable)
        sequence[index].gainLevel = gainSpinBox->value();
    
    if(sequence[index].shutterAvailable)
        sequence[index].shutterValue = shutterSpeedSpinBox->value();
    
    if(sequence[index].f7Available) {
        if(f7AOIWidthSlider->isEnabled()) {
            if(QMessageBox::warning(this, "F7-Size",
                                    "F7-Size can only be changed BEFORE the first 'Apply'!\n\nContinue ?",
                                    QMessageBox::Yes | QMessageBox::Default,
                                    QMessageBox::No | QMessageBox::Escape,
                                    QMessageBox::NoButton) == QMessageBox::No) {
                return;
            }
            
            // disable (fix) F7-Size
            f7AOIWidthSlider->setEnabled(false);
            f7AOIWidthSpinBox->setEnabled(false);
            f7AOIHeightSlider->setEnabled(false);
            f7AOIHeightSpinBox->setEnabled(false);
            
            // set new F7-Size in all sequence-entries
            for(int i = 0; i < MAX_SEQ_LENGTH; i++) {
                sequence[i].f7Width = f7AOIWidthSpinBox->value();
                sequence[i].f7Height = f7AOIHeightSpinBox->value();
                sequence[i].f7X = f7AOIXSpinBox->value();
                sequence[i].f7Y = f7AOIYSpinBox->value();
            }
        }
        sequence[index].f7Width = f7AOIWidthSpinBox->value();
        sequence[index].f7Height = f7AOIHeightSpinBox->value();
        sequence[index].f7X = f7AOIXSpinBox->value();
        sequence[index].f7Y = f7AOIYSpinBox->value();
    }
    
    if(seqCtlAutoIncCheckBox->isChecked())
        seqCtlImgNoSpinBox->setValue(index + 2);
}


void SeqCtlDemoForm::seqCtlPlay() {
    CamWindow *camWindow;
    bool busy;
    
    
    // check 'parent'-pointer
    if(QString(parent->className()) != "CamWindow") {
        logger->log("\nParent of 'AdjustmentsForm' is '%s', but 'CamWindow' is expected!",
                    parent->className());
        QApplication::restoreOverrideCursor();
        return;
    }
    else {
        camWindow  = (CamWindow *)parent;
    }
    
    // output - just for testing purposes
    for(int i = 0; i < seqCtlLengthSpinBox->value(); i++) {
        logger->log("Entry " + QString::number(i) + " :");
        if(sequence[i].gainAvailable) {
            logger->log("\tGain: " + QString::number(sequence[i].gainLevel));
        }
        if(sequence[i].shutterAvailable) {
            logger->log("\tSutter: " + QString::number(sequence[i].shutterValue));
        }
        if(sequence[i].f7Available) {
            logger->log("F7-Pos.: " + QString::number(sequence[i].f7X) + " , " +
                        QString::number(sequence[i].f7Y));
            logger->log("F7-Size.: " + QString::number(sequence[i].f7Width) + " x " +
                        QString::number(sequence[i].f7Height));
        }
    }
    
    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
    
    // stop capture if necessary before enabling 'sequence control'
    logger->log("Stop capturing for 'Sequence-Control'...");
    if(videoRunning) {
        if(!camWindow->stopCapture()) {
            QMessageBox::warning(this, "Running Sequence",
                                 "Error while trying to run sequence!\nSee 'Message Logger' output for details.",
                                 QMessageBox::Ok | QMessageBox::Default | QMessageBox::Escape,
                                 QMessageBox::NoButton,
                                 QMessageBox::NoButton);
            logger->log("Error while trying to stop capturing!");
            QApplication::restoreOverrideCursor();
            return;
        }
        videoRunning = false;
    }
    
    // !!!! 'SeqLength' has to be set before enabling the sequence (ON_OFF = 1)
    // !!!! because this value must be '!= 0' - otherwise the ON_OFF-flag will be
    // !!!! (automatically) reset to '0' !!!!
    
    // set sequence-length ( -1- set  'SeqLength' to the desired length of the sequnce)
    if(!dc1394->setSeqCtlLength(id, seqCtlLengthSpinBox->value())) {
        QMessageBox::warning(this, "Running Sequence",
                             "Error while trying to run sequence!\nSee 'Message Logger' output for details.",
                             QMessageBox::Ok | QMessageBox::Default | QMessageBox::Escape,
                             QMessageBox::NoButton,
                             QMessageBox::NoButton);
        logger->log("Error while trying to set 'Sequence' length!");
        // restart capturing
        camWindow->startCapture();
        videoRunning = true;
        QApplication::restoreOverrideCursor();
        return;
    }
    
    // Enable Sequence ( -1 (cont.) - set 'On' to true)
    if(!dc1394->seqCtlEnable(id, true)) {
        QMessageBox::warning(this, "Running Sequence",
                             "Error while trying to run sequence!\nSee 'Message Logger' output for details.",
                             QMessageBox::Ok | QMessageBox::Default | QMessageBox::Escape,
                             QMessageBox::NoButton,
                             QMessageBox::NoButton);
        logger->log("Error while trying to Enable 'Sequence Control'!");
        // restart capturing
        camWindow->startCapture();
        videoRunning = true;
        QApplication::restoreOverrideCursor();
        return;
    }
    
    // enable 'AutoIncrement' for convenience (only apply is necessary to set the respective parameters)
    if(!dc1394->seqCtlEnableAutoInc(id, seqCtlAutoIncCheckBox->isChecked())) {
        QMessageBox::warning(this, "Running Sequence",
                             "Error while trying to run sequence!\nSee 'Message Logger' output for details.",
                             QMessageBox::Ok | QMessageBox::Default | QMessageBox::Escape,
                             QMessageBox::NoButton,
                             QMessageBox::NoButton);
        logger->log("Error while trying to enable 'AutoInc' while setting up the sequence!");
        // restart capturing
        camWindow->startCapture();
        videoRunning = true;
        QApplication::restoreOverrideCursor();
        return;
    }
    
    // intialize 'Sequence-Counter' with '0' ( -2- set 'ImageNo' to '0')
    if(!dc1394->setSeqCtlImageNo(id, 0)) {
        QMessageBox::warning(this, "Running Sequence",
                             "Error while trying to run sequence!\nSee 'Message Logger' output for details.",
                             QMessageBox::Ok | QMessageBox::Default | QMessageBox::Escape,
                             QMessageBox::NoButton,
                             QMessageBox::NoButton);
        logger->log("Error while trying to initialize 'ImageNo' (with 0)!");
        // restart capturing
        camWindow->startCapture();
        videoRunning = true;
        QApplication::restoreOverrideCursor();
        return;
    }
    
    // 'apply' sequence parameters
    for(int i = 0; i < seqCtlLengthSpinBox->value(); i++) {
        // ( -3- set the DCAM register as desired)
        // set 'gain' values
        if(sequence[i].gainAvailable) {
            if(!dc1394->setFeatureValue(id, FEATURE_GAIN - FEATURE_MIN,
                                        sequence[i].gainLevel)) {
                logger->log("Error while trying to change 'Gain'!");
            }
        }
        // set 'shutter' values
        if(sequence[i].shutterAvailable) {
            if(!dc1394->setFeatureValue(id, FEATURE_SHUTTER - FEATURE_MIN,
                                        sequence[i].shutterValue)) {
                logger->log("Error while trying to change 'Shutter-Speed'!");
            }
        }
        // set 'F7' values
        if(sequence[i].f7Available) {
            if(!dc1394->setF7ImagePos(id, sequence[i].f7X, sequence[i].f7Y)) {
                logger->log("Error while trying to change F7 position");
            }
            if(!dc1394->setF7ImageSize(id, sequence[i].f7Width, sequence[i].f7Height)) {
                logger->log("Error while trying to change F7 size");
            }
        }
        
        // apply seq.-parameters ( -4- set 'ApplyParameters' to true)
        if(!dc1394->seqCtlApply(id, true)) {
            QMessageBox::warning(this, "Running Sequence",
                                 "Error while trying to run sequence!\nSee 'Message Logger' output for details.",
                                 QMessageBox::Ok | QMessageBox::Default | QMessageBox::Escape,
                                 QMessageBox::NoButton,
                                 QMessageBox::NoButton);
            logger->log("Error while trying to 'Apply' current parameters to sequence (Set No. " +
                        QString::number(i) + ")");
            // restart capturing
            camWindow->startCapture();
            videoRunning = true;
            QApplication::restoreOverrideCursor();
            return;
        }
        
        // wait while sequence-control is 'busy'
        while(1) {
            if(!dc1394->seqCtlBusy(id, &busy)) {
                logger->log("Error while trying to get current status (busy) of the sequence control!");
                sleep(1);
                continue;
            }
            
            if(!busy)
                break;
            
            sleep(1);
        }
        // -5- increment 'ImageNo' -- this might be omitted, if 'IncImageNo' is set to true (see above)
    } // repeat steps 3 to 5 until all parameters for all images of the sequence have been set
    
    // enable 'AutoRewind' depending on the corresponding CheckBox
    if(!dc1394->seqCtlEnableAutoRewind(id, seqCtlAutoRewindCheckBox->isChecked())) {
        QMessageBox::warning(this, "Running Sequence",
                             "Error while trying to run sequence!\nSee 'Message Logger' output for details.",
                             QMessageBox::Ok | QMessageBox::Default | QMessageBox::Escape,
                             QMessageBox::NoButton,
                             QMessageBox::NoButton);
        logger->log("Error while trying to enable 'AutoRewind'!");
        // restart capturing
        camWindow->startCapture();
        videoRunning = true;
        QApplication::restoreOverrideCursor();
        return;
    }
    
    
    // restart capturing for starting the sequence
    camWindow->startCapture();
    videoRunning = true;
    
    // disable all but 'Stop'
    seqCtlTabWidget->setEnabled(false);
    seqCtlLengthSpinBox->setEnabled(false);
    seqCtlImgNoSpinBox->setEnabled(false);
    seqCtlApplyPushButton->setEnabled(false);
    seqCtlAutoRewindCheckBox->setEnabled(false);
    
    seqCtlRunPushButton->setEnabled(false);
    seqCtlStopPushButton->setEnabled(true);
    
    QApplication::restoreOverrideCursor();
}


void SeqCtlDemoForm::seqCtlStop() {
    unsigned int waitCnt = 5;
    bool busy = true;
    
    
    // disable SequenceControl
    if(!dc1394->seqCtlEnable(id, false)) {
        logger->log("Error while trying to disable SequenceControl!");
    }
    
    // enable all but 'Stop'
    seqCtlTabWidget->setEnabled(true);
    seqCtlLengthSpinBox->setEnabled(true);
    seqCtlImgNoSpinBox->setEnabled(true);
    seqCtlApplyPushButton->setEnabled(true);
    seqCtlAutoRewindCheckBox->setEnabled(true);
    
    seqCtlRunPushButton->setEnabled(true);
    seqCtlStopPushButton->setEnabled(false);
    
    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
    do {
        if(!dc1394->seqCtlBusy(id, &busy)) {
            logger->log("Error while waiting on Sequence Control availability...");
        }
        
        if(!busy)
            break;
        
        sleep(1);
    } while(busy && (--waitCnt > 0));
    
    QApplication::restoreOverrideCursor();
    
    if(seqCtlImgNoSpinBox->value() == 1) {
        seqCtlImgNoChanged(1);
    }
    seqCtlImgNoSpinBox->setValue(1);
}
