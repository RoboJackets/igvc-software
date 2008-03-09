/****************************************************************************
** ui.h extension file, included from the uic-generated form implementation.
**
** If you wish to add, delete or rename functions or slots use
** Qt Designer which will update this file, preserving your code. Create an
** init() function in place of a constructor, and a destroy() function in
** place of a destructor.
*****************************************************************************/

void AdjustmentsForm::init() {
    // dialog-box not yet initialized ('initAdjustments()' has to be called explicitly!)
    initialized = false;
    
    rejecting = false;
    
    for(int i = 0; i < NUM_FEATURES; i++) {
        featureGroup[i] = NULL;
        featureSlider[i] = NULL;
        featureSpinBox[i] = NULL;
        featureOnCheckBox[i] = NULL;
        featureAutoModeCheckBox[i] = NULL;
        featureOnePushPushButton[i] = NULL;
    }
    
    // image controls
    brightnessGroupBox->setEnabled(false);
    gainGroupBox->setEnabled(false);
    gammaGroupBox->setEnabled(false);
    sharpnessGroupBox->setEnabled(false);
    
    featureGroup[FEATURE_BRIGHTNESS - FEATURE_MIN] = brightnessGroupBox;
    featureSlider[FEATURE_BRIGHTNESS - FEATURE_MIN] = brightnessSlider;
    featureSpinBox[FEATURE_BRIGHTNESS - FEATURE_MIN] = brightnessSpinBox;
    featureOnCheckBox[FEATURE_BRIGHTNESS - FEATURE_MIN] = brightnessOnCheckBox;
    featureAutoModeCheckBox[FEATURE_BRIGHTNESS - FEATURE_MIN] =
            brightnessAutoModeCheckBox;
    featureOnePushPushButton[FEATURE_BRIGHTNESS - FEATURE_MIN] =
            brightnessOnePushPushButton;
    
    featureGroup[FEATURE_GAIN - FEATURE_MIN] = gainGroupBox;
    featureSlider[FEATURE_GAIN - FEATURE_MIN] = gainSlider;
    featureSpinBox[FEATURE_GAIN - FEATURE_MIN] = gainSpinBox;
    featureOnCheckBox[FEATURE_GAIN - FEATURE_MIN] = gainOnCheckBox;
    featureAutoModeCheckBox[FEATURE_GAIN - FEATURE_MIN] = gainAutoModeCheckBox;
    featureOnePushPushButton[FEATURE_GAIN - FEATURE_MIN] = gainOnePushPushButton;
    
    featureGroup[FEATURE_GAMMA - FEATURE_MIN] = gammaGroupBox;
    featureSlider[FEATURE_GAMMA - FEATURE_MIN] = gammaSlider;
    featureSpinBox[FEATURE_GAMMA - FEATURE_MIN] = gammaSpinBox;
    featureOnCheckBox[FEATURE_GAMMA - FEATURE_MIN] = gammaOnCheckBox;
    featureAutoModeCheckBox[FEATURE_GAMMA - FEATURE_MIN] =
            gammaAutoModeCheckBox;
    featureOnePushPushButton[FEATURE_GAMMA - FEATURE_MIN] =
            gammaOnePushPushButton;
    
    gammaOnCanceled = false;
    
    featureGroup[FEATURE_SHARPNESS - FEATURE_MIN] = sharpnessGroupBox;
    featureSlider[FEATURE_SHARPNESS - FEATURE_MIN] = sharpnessSlider;
    featureSpinBox[FEATURE_SHARPNESS - FEATURE_MIN] = sharpnessSpinBox;
    featureOnCheckBox[FEATURE_SHARPNESS - FEATURE_MIN] = sharpnessOnCheckBox;
    featureAutoModeCheckBox[FEATURE_SHARPNESS - FEATURE_MIN] =
            sharpnessAutoModeCheckBox;
    featureOnePushPushButton[FEATURE_SHARPNESS - FEATURE_MIN] =
            sharpnessOnePushPushButton;
    
    // color controls
    hueGroupBox->setEnabled(false);
    saturationGroupBox->setEnabled(false);
    whiteBalGroupBox->setEnabled(false);
    
    featureGroup[FEATURE_HUE - FEATURE_MIN] = hueGroupBox;
    featureSlider[FEATURE_HUE - FEATURE_MIN] = hueSlider;
    featureSpinBox[FEATURE_HUE - FEATURE_MIN] = hueSpinBox;
    featureOnCheckBox[FEATURE_HUE - FEATURE_MIN] = hueOnCheckBox;
    featureAutoModeCheckBox[FEATURE_HUE - FEATURE_MIN] = hueAutoModeCheckBox;
    featureOnePushPushButton[FEATURE_HUE - FEATURE_MIN] = hueOnePushPushButton;
    
    featureGroup[FEATURE_SATURATION - FEATURE_MIN] = saturationGroupBox;
    featureSlider[FEATURE_SATURATION - FEATURE_MIN] = saturationSlider;
    featureSpinBox[FEATURE_SATURATION - FEATURE_MIN] = saturationSpinBox;
    featureOnCheckBox[FEATURE_SATURATION - FEATURE_MIN] = saturationOnCheckBox;
    featureAutoModeCheckBox[FEATURE_SATURATION - FEATURE_MIN] =
            saturationAutoModeCheckBox;
    featureOnePushPushButton[FEATURE_SATURATION - FEATURE_MIN] =
            saturationOnePushPushButton;
    
    featureGroup[FEATURE_WHITE_BALANCE - FEATURE_MIN] = whiteBalGroupBox;
    featureSlider[FEATURE_WHITE_BALANCE - FEATURE_MIN] = NULL;
    featureSpinBox[FEATURE_WHITE_BALANCE - FEATURE_MIN] = NULL;
    featureOnCheckBox[FEATURE_WHITE_BALANCE - FEATURE_MIN] = whiteBalOnCheckBox;
    featureAutoModeCheckBox[FEATURE_WHITE_BALANCE - FEATURE_MIN] =
            whiteBalAutoModeCheckBox;
    featureOnePushPushButton[FEATURE_WHITE_BALANCE - FEATURE_MIN] =
            whiteBalOnePushPushButton;
    
    // timing controls
    autoExposureGroupBox->setEnabled(false);
    exposureTimeGroupBox->setEnabled(false);
    
    featureGroup[FEATURE_EXPOSURE - FEATURE_MIN] = autoExposureGroupBox;
    featureSlider[FEATURE_EXPOSURE - FEATURE_MIN] = autoExposureSlider;
    featureSpinBox[FEATURE_EXPOSURE - FEATURE_MIN] = autoExposureSpinBox;
    featureOnCheckBox[FEATURE_EXPOSURE - FEATURE_MIN] = autoExposureOnCheckBox;
    featureAutoModeCheckBox[FEATURE_EXPOSURE - FEATURE_MIN] =
            autoExposureAutoModeCheckBox;
    featureOnePushPushButton[FEATURE_EXPOSURE - FEATURE_MIN] =
            autoExposureOnePushPushButton;
    
    featureGroup[FEATURE_SHUTTER - FEATURE_MIN] = stdShutterGroupBox;
    featureSlider[FEATURE_SHUTTER - FEATURE_MIN] = shutterSpeedSlider;
    featureSpinBox[FEATURE_SHUTTER - FEATURE_MIN] = shutterSpeedSpinBox;
    featureOnCheckBox[FEATURE_SHUTTER - FEATURE_MIN] = shutterSpeedOnCheckBox;
    featureAutoModeCheckBox[FEATURE_SHUTTER - FEATURE_MIN] =
            shutterSpeedAutoModeCheckBox;
    featureOnePushPushButton[FEATURE_SHUTTER - FEATURE_MIN] =
            shutterSpeedOnePushPushButton;
    
    
    // trigger controls
    triggerGroupBox->setEnabled(false);
    
    featureGroup[FEATURE_TRIGGER - FEATURE_MIN] = triggerGroupBox;
    featureOnCheckBox[FEATURE_TRIGGER - FEATURE_MIN] = triggerOnCheckBox;
    
    // trigger-delay
    triggerDelayGroupBox->setEnabled(false);
    
    featureGroup[FEATURE_TRIGGER_DELAY - FEATURE_MIN] = triggerDelayGroupBox;
    featureSlider[FEATURE_TRIGGER_DELAY - FEATURE_MIN] = triggerDelaySlider;
    featureSpinBox[FEATURE_TRIGGER_DELAY - FEATURE_MIN] = triggerDelaySpinBox;
    featureOnCheckBox[FEATURE_TRIGGER_DELAY - FEATURE_MIN] = triggerDelayOnCheckBox;
    featureAutoModeCheckBox[FEATURE_TRIGGER_DELAY - FEATURE_MIN] =
            triggerDelayAutoModeCheckBox;
    featureOnePushPushButton[FEATURE_TRIGGER_DELAY - FEATURE_MIN] =
            triggerDelayOnePushPushButton;
    
    
    // initially all feature-values are displayed in percent
    for(int i = 0; i < NUM_FEATURES; i++) {
        featureValPercent[i] = true;
    }
    
    f7AOIGroupBox->setEnabled(false);
    f7FrameInfoGroupBox->setEnabled(false);
    
    
    // advanced features - in pogress...
    shadingCorrectionGroupBox->setEnabled(false);
    lutGroupBox->setEnabled(false);
    hdrGroupBox->setEnabled(false);
    
    // DSNU-Blemish
    dsnuGrabCountSpinBox->setMinValue(0);
    dsnuGrabCountSpinBox->setMaxValue(255);
    
    // AutoAOI
    autoAOIGroupBox->setEnabled(false);
    
    focusdata = focusData();
    
    updateF7UI = true;
    f7CorrectWidthSliderValue = false;
    f7CorrectWidthSpinBoxValue = false;
    f7CorrectHeightSliderValue = false;
    f7CorrectHeightSpinBoxValue = false;
    f7CorrectXSliderValue = false;
    f7CorrectXSpinBoxValue = false;
    f7CorrectYSliderValue = false;
    f7CorrectYSpinBoxValue = false;
    
    autoAOICorrectWidthSliderValue = false;
    autoAOICorrectWidthSpinBoxValue = false;
    autoAOICorrectHeightSliderValue = false;
    autoAOICorrectHeightSpinBoxValue = false;
    autoAOICorrectXSliderValue = false;
    autoAOICorrectXSpinBoxValue = false;
    autoAOICorrectYSliderValue = false;
    autoAOICorrectYSpinBoxValue = false;
    
    autoAOIChanged = false;
    autoAOISwitched = false;
    
    shadingImgLoaded = false;
    lutLoaded = false;
} // init()


void AdjustmentsForm::show() {
    updateUI();
    
    rejecting = false;
    
    // Widgets on Stack (I/O 3 - SIO):
    // 0 - GP-I/O3 (IO3_CONTROL)
    // 1 -  Serial Interface (SIO_CONTROL)
    io3WidgetStack->raiseWidget(gpIO3Enabled ? IO3_CONTROL : SIO_CONTROL);
    
    QDialog::show();
}


void AdjustmentsForm::initAdjustments(QWidget *cw, DC1394 *dc, int i, bool absControl, MessageLogger *ml) {
    bool bTmp1, bTmp2;
    unsigned int uIntTmp;
    unsigned int currentLUT;
    unsigned int maxLutNo;
    
    
    if(initialized && (absControl == absolute)) {
        updateUI();
        return;
    }
    
    dc1394 = dc;
    id = i;
    absolute = absControl;
    parent = cw;
    logger = ml;
    
    // set MaxGrabCount from constants depending on the camera-type
    if(!dc1394->getCameraID(id, &cameraID)) {
        logger->log("AdjustmentsForm::initAdjustments: Error while trying to read CameraID - using some defaults!");
        maxGrabCount = 1;
    }
    
    switch(cameraID) {
    case F145b:
    case F145c:
        maxGrabCount = F145_FB_SIZE;
        break;
    case F201b:
    case F201c:
        maxGrabCount = F201_FB_SIZE;
        break;
    case F145b_1:
    case F145c_1:
        maxGrabCount = F145_1_FB_SIZE;
        break;
    case F201b_1:
    case F201c_1:
        maxGrabCount = F201_1_FB_SIZE;
        break;
    case MF033B:
    case MF033C:
        maxGrabCount = MF033_FB_SIZE;
        break;
    case MF046B:
    case MF046C:
        maxGrabCount = MF046_FB_SIZE;
        break;
    case MF080B:
    case MF080C:
        maxGrabCount = MF080_FB_SIZE;
        break;
    case MF145B2:
    case MF145C2:
        maxGrabCount = MF145_FB_SIZE;
        break;
    case MF131B:
    case MF131C:
        maxGrabCount = MF131_FB_SIZE;
        break;
    case MF145B2_15fps:
    case MF145C2_15fps:
        maxGrabCount = MF145_FB_SIZE;
        break;
    default:
        maxGrabCount = 1;
        break;
    }
    
    // setting the shadingCorrectionGrabCountSpinBox-Max.Value to 'maxGrabCount'
    shadingCorrectionGrabCountSpinBox->setMaxValue(maxGrabCount);
    
    // setting shadingCorrectionGrabCountSpinBox to the value currently stored in the camera
    if(!dc1394->getShadingCorrectionInfos(id, &bTmp1, &grabCount, &uIntTmp, &bTmp2)) {
        logger->log("AdjustmentsForm::initAdjustments: Error while trying to read the current ShadingCorrection GrabCount value!");
        logger->log("\t-> Using '0' insted...");
        grabCount = 0;
    }
    shadingCorrectionGrabCountSpinBox->setValue(grabCount);
    
    // the BuildImg-PushButton should only be enabled if a valid image is inside of the camera
    shadingCorrectionBuildImgPushButton->setEnabled(grabCount > 0);
    
    // GrabCount SpinBox should always be available
    shadingCorrectionGrabCountSpinBox->setEnabled(true);
    
    
    // the LUT-Load PushButton should always be enabled
    lutLoadPushButton->setEnabled(true);
    // and so the LUT-Number SpinBox
    if(dc1394->getLUTInfos(id, &maxLutNo, &maxLUTSize, &bTmp1)) {
        lutNumberSpinBox->setMinValue(1);
        lutNumberSpinBox->setMaxValue(maxLutNo);
        lutMaxSizeLineEdit->setText(QString::number(maxLUTSize));
        
        if(dc1394->getLUTSelection(id, &currentLUT)) {
            lutNumberSpinBox->setEnabled(true);
            lutNumberSpinBox->setValue(currentLUT + 1);
        }
    }
    else {
        logger->log("AdjustmentsForm::initAdjustments: Error while trying to read the current LUT selection!");
        if(lutNumberSpinBox->isEnabled() && lutNumberSpinBox->hasFocus()) {
            focusdata->next()->setFocus();
        }
        lutNumberSpinBox->setEnabled(false);
    }
    
    lutOnCanceled = false;
    
    // HDR
    hdrGroupBox->setEnabled((dc1394->hdrAvailable(id)) && initHDR());
    
    // init absolute control if possible and enabled
//    initAbsoluteControl();
    
    // init IO-combo-boxes
    initIO1();
    initIO2();
    initIO3();
    
    // init CaptureMode
    initCaptureMode();
    
    // update the UI (after reading the current state of the camera feature set)
    extShutterChanged = false;
    autoShutterChanged = false;
    autoGainChanged = false;
    autoAOIChanged = false;
    updateUI();
    
    // AdjustmentsForm initialized
    initialized = true;
    
    for(int i = 0; i < NUM_FEATURES; i++)
        featureChanged[i] = false;
    
    f7Changed = false;
    f7BPPChanged = false;
    extShutterChanged = false;
    initExtShutterSlider = 0;
    initExtShutterSpinBox = 0;
    tbaseChanged = false;
    shadingCorrectionChanged = false;
    lutChanged = false;
    hdrChanged = false;
    autoShutterChanged = false;
    autoGainChanged = false;
    autoAOIChanged = false;
    
    gpIO3Enabled = false;
    
    applyPushButton->setEnabled(false);
    discardPushButton->setEnabled(false);
    
} // initAdjustments()


void AdjustmentsForm::enableIO3(bool enable) {
    gpIO3Enabled = (dc1394->ioAvailable(id, 3)) ? enable : false;
}


void AdjustmentsForm::featuresApply() {
    bool stdFeatureChanged = false;

    
    // Standard Features
    for(int i = 0; i < NUM_FEATURES; i++) {
        if(featureChanged[i]) {
            // write all 'unapplied' changes to camera
            applyUIChanges(i);
            
            // reset flag
            featureChanged[i] = false;
            stdFeatureChanged = true;
        }
    }
    
    
    // Format 7
    if(f7BPPChanged) {
        this->setEnabled(false);
        updateF7BPP();
        f7BPPChanged = false;
    }
    
    if(f7Changed) {
        this->setEnabled(false);
        updateF7UI = false;
        updateF7();
        f7Changed = false;
    }
    
    // Advanced features
    
    // Ext. Shutter
    if(extShutterChanged) {
        updateExtShutter();
        extShutterChanged = false;
        stdFeatureChanged = true;
    }
    
    // Timebase
    if(tbaseChanged) {
        updateTimebase();
        tbaseChanged = false;
        stdFeatureChanged = true;
    }
    
    // Adv. Trigger Delay
    if(advTriggerDelayChanged) {
        updateAdvTriggerDelay();
        advTriggerDelayChanged = false;
        stdFeatureChanged = true;
    }
    
    // Shading Correction
    if(shadingCorrectionChanged) {
        updateShadingCorrection();
        shadingCorrectionChanged = false;
        stdFeatureChanged = true;
    }
    
    // LUT
    if(lutChanged) {
        updateLUT();
        lutChanged = false;
        stdFeatureChanged = true;
    }
    
    // HDR
    if(hdrChanged) {
        updateHDR();
        hdrChanged = false;
        stdFeatureChanged = true;
    }
    
    // AutoShutter
    if(autoShutterChanged) {
        updateAutoShutter();
        autoShutterChanged = false;
        stdFeatureChanged = true;
    }
    
    // AutoGain
    if(autoGainChanged) {
        updateAutoGain();
        autoGainChanged = false;
        stdFeatureChanged = true;
    }
    
    // AutoAOI
    if(autoAOIChanged) {
        updateAutoAOI();
        autoAOIChanged = false;
        stdFeatureChanged = true;
    }
    
    // DeferredImageTransport
    if(deferredImgChanged) {
        updateDeferredImg();
        deferredImgChanged = false;
        stdFeatureChanged = true;
    }
    
    // FrameInfo
    updateFrameInfo();
    
    // Delayed Integration
    if(delIntChanged) {
        updateDelInt();
        delIntChanged = false;
        stdFeatureChanged = true;
    }
    
    // Incremental Decoder
    if(incDecChanged) {
        updateIncDec();
        incDecChanged = false;
        stdFeatureChanged = true;
    }
    
    // disable the buttons and update the flags accordingly
    applyPushButton->setEnabled(false);
    discardPushButton->setEnabled(false);
    
    updateUI();
    
    // enable UI-controls
    if((!updateF7UI) || (stdFeatureChanged)) {
        this->setEnabled(true);
    }
}


void AdjustmentsForm::featuresDiscard() {
    // undo all 'unapplied' changes
    
    // Standard Features
    for(int i = 0; i < NUM_FEATURES; i++) {
        if(featureChanged[i]) {
            // discard all recently made changes
            discardUIChanges(i);
            
            // reset flag
            featureChanged[i] = false;
        }
    }
    
    // Format 7
    if(f7Changed) {
        updateF7UI = false;
        discardF7();
        f7Changed = false;
    }
    
    // Advanced features
    
    // Timebase
    if(tbaseChanged) {
        discardTimebase();
        tbaseChanged = false;
    }
    
    // Std. Trigger Delay
    if(triggerDelayChanged) {
        discardTriggerDelay();
        triggerDelayChanged = false;
    }
    
    // Adv. Trigger Delay
    if(advTriggerDelayChanged) {
        discardAdvTriggerDelay();
        advTriggerDelayChanged = false;
    }
    
    // Shading Correction
    if(shadingCorrectionChanged) {
        discardShadingCorrection();
        shadingCorrectionChanged = false;
    }
    
    // LUT
    if(lutChanged) {
        discardLUT();
        lutChanged = false;
    }
    
    // HDR
    if(hdrChanged) {
        discardHDR();
        hdrChanged = false;
    }
    
    // AutoShutter
    if(autoShutterChanged) {
        discardAutoShutter();
        autoShutterChanged = false;
    }
    
    // AutoGain
    if(autoGainChanged) {
        discardAutoGain();
        autoGainChanged = false;
    }
    
    // AutoAOI
    if(autoAOIChanged) {
        discardAutoAOI();
        autoAOIChanged = false;
    }
    
    // DeferredImageTranport
    if(deferredImgChanged) {
        discardDeferredImg();
        deferredImgChanged = false;
    }
    
    // Delayed Integration
    if(delIntChanged) {
        discardDelInt();
        delIntChanged = false;
    }
    
    // Incremental Encoder
    if(incDecChanged) {
        discardIncDec();
        incDecChanged = false;
    }
    
    // disable the buttons and update the flags accordingly
    applyPushButton->setEnabled(false);
    discardPushButton->setEnabled(false);
}


void AdjustmentsForm::applyUIChanges(int featureIndex) {
    int newValue, newUValue, newVValue;
    int minValue = (featureSet.feature[featureIndex]).min;
    int maxValue = (featureSet.feature[featureIndex]).max;
    int range = maxValue - minValue;
    
    logger->log("AdjustmentsForm::applyUIChanges: Applying UI Changes to camera...");
    
    // get the changed UI values - and write 'em to the camera
    
    // Trigger is something different...
    if(featureIndex == FEATURE_TRIGGER - FEATURE_MIN) {
        updateTrigger();
        return;
    }
    
    // Slider / Spin-Box
    if(featureIndex == FEATURE_WHITE_BALANCE - FEATURE_MIN) {
        // white balance value handling (if one component is enabled - the other is too)
        if(whiteBalUSlider->isEnabled()) {
            newUValue = minValue + (whiteBalUSlider->value() + (range / 2));
            newVValue = minValue + (whiteBalVSlider->value() + (range / 2));
            
            if(!dc1394->setWhiteBalanceValue(id, newUValue, newVValue)) {
                logger->log("AjdustmentsForm::applyUIChanges: White-Balance adjust failed!");
            }
        }
    }
    else {
        if((featureSlider[featureIndex] != NULL) && featureSlider[featureIndex]->isEnabled()) {
            if((featureValPercent[featureIndex]) &&
               (featureSet.feature[featureIndex].abs_control != DC1394_TRUE))
                newValue = minValue + ((featureSlider[featureIndex]->value() * range) / 100);
            else
                newValue = featureSlider[featureIndex]->value();
            
            if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
                logger->log("AdjustmentsForm::applyUIChanges: Feature(%d)-Value adjust (%d) failed!",
                            featureIndex, newValue);
            }
        }
    }
    
    // Auto-Mode Check-Box
    if((featureAutoModeCheckBox[featureIndex] != NULL) &&
       featureAutoModeCheckBox[featureIndex]->isEnabled()) {
        
        if(!dc1394->setFeatureAutoMode(id, featureIndex,
                                       featureAutoModeCheckBox[featureIndex]->isChecked())) {
            logger->log("AdjustmentsForm::applyUIChanges: %s Auto-Mode failed!",
                   featureAutoModeCheckBox[featureIndex]->isChecked() ? "Enabling" :
                   "Disabling");
        }
    }
    
    // On Check-Box
    if((featureOnCheckBox[featureIndex] != NULL) &&
       featureOnCheckBox[featureIndex]->isEnabled()) {
        
        if(!dc1394->setFeatureOn(id, featureIndex,
                                 featureOnCheckBox[featureIndex]->isChecked())) {
            logger->log("AdjustmentsForm::applyUIChanges: Switching feature %d %s failed!",
                        featureIndex,
                        featureOnCheckBox[featureIndex]->isChecked() ? "on" : "off");
        }
    }
}


void AdjustmentsForm::updateTrigger() {
    bool initSave;
    dc1394bool_t triggerPolarity;
    unsigned int triggerMode;
    
    
    // On Check-Box
    if(triggerOnCheckBox->isEnabled()) {
        if(!triggerOnCheckBox->isChecked() && swTriggerStopPushButton->isEnabled()) {
            QMessageBox::warning(this, "SW-Trigger",
                                 "Software-Trigger still active!\nPlease 'Stop' first!",
                                 QMessageBox::Ok | QMessageBox::Default,
                                 QMessageBox::NoButton,
                                 QMessageBox::NoButton);
            
            logger->log("AdjustmentsForm::updateTrigger: Trigger still active - therefore ignoring disabling!");
            featuresDiscard();
            return;
        }
        
        if(!dc1394->setFeatureOn(id, FEATURE_TRIGGER - FEATURE_MIN,
                                 triggerOnCheckBox->isChecked())) {
            logger->log("AdjustmentsForm::updateTrigger: Switching trigger %s failed!",
                        triggerOnCheckBox->isChecked() ? "on" : "off");
            initSave = initialized;
            initialized = false;
            triggerOnCheckBox->setChecked(false);
            initialized = initSave;
        }
        else {
            triggerMode = (triggerModeComboBox->currentText()).toUInt();
            
            // if camera is in trigger mode 15 (AVT-specific 'bulk-mode'), the 'videoPanel'
            // must be informed for updating the UI
            if((triggerMode == 15) && triggerOnCheckBox->isChecked()) {
                ((CamWindow *)parent)->getFrameCountUpdateInfos();
            }
        }
        
        // because SW-Trigger is not yet implemented, this is deferred (and remains disabled)
        // until it's available
        if(dc1394->swTriggerAvailable(id))
            swTriggerPushButton->setEnabled(false);
//            swTriggerPushButton->setEnabled(triggerOnCheckBox->isChecked());
        else
            swTriggerPushButton->setEnabled(false);
    }
    
    // Polarity
    if(triggerPolarityComboBox->isEnabled()) {
        // entry 0 - low-active / entry 1 - hight-active
        triggerPolarity = ((triggerPolarityComboBox->currentItem()) == 1) ? DC1394_TRUE :
                          DC1394_FALSE;
        
        if(!dc1394->setTriggerPolarity(id, triggerPolarity)) {
            logger->log("AdjustmentsForm::updateTrigger: Setting Trigger-Polarity to %s failed!", 
                        triggerPolarity ? "low act." : "hight act.");
        }
    }
    
    // Mode-ComboBox
    if(triggerModeComboBox->isEnabled()) {
        if(!dc1394->setTriggerMode(id, (triggerModeComboBox->currentText()).toInt())) {
            logger->log("AdjustmentsForm::updateTrigger: Setting Trigger-Mode to %s failed!",
                        (triggerModeComboBox->currentText()).ascii());
        }
    }
    
    // Trigger-Source
    if(triggerSourceComboBox->isEnabled()) {
        if(!dc1394->setTriggerSource(id, (triggerSourceComboBox->currentText()).toInt())) {
            logger->log("AdjustmentsForm::updateTrigger: Enabling Trigger-Source %s failed!",
                        (triggerSourceComboBox->currentText()).ascii());
        }
    }
}


void AdjustmentsForm::discardUIChanges(int featureIndex) {
    dc1394_feature_info featureInfo = featureSet.feature[featureIndex];
    int range = featureInfo.max - featureInfo.min;
    int value, exp = 0;
    bool initSave = initialized;
    
    
    // restore the UI-controls to the values still valid in the camera
    switch(featureIndex) {
    case FEATURE_WHITE_BALANCE - FEATURE_MIN:
        if(range > 0) {
            whiteBalUSlider->setValue(featureInfo.BU_value - (range / 2));
            whiteBalUSpinBox->setValue(featureInfo.BU_value - (range / 2));
            whiteBalVSlider->setValue(featureInfo.RV_value - (range / 2));
            whiteBalVSpinBox->setValue(featureInfo.RV_value - (range / 2));
        }
        else {
            whiteBalUSlider->setEnabled(false);
            if(whiteBalUSpinBox->isEnabled() && whiteBalUSpinBox->hasFocus()) {
                focusdata->next()->setFocus();
            }
            whiteBalUSpinBox->setEnabled(false);
            whiteBalVSlider->setEnabled(false);
            if(whiteBalVSpinBox->isEnabled() && whiteBalVSpinBox->hasFocus()) {
                focusdata->next()->setFocus();
            }
            whiteBalVSpinBox->setEnabled(false);
        }
        break;
        
    case FEATURE_TRIGGER - FEATURE_MIN:
        // Std. Trigger Feature has no Slider or SpinBox
        break;
        
    default:
        if(featureSlider[featureIndex] != NULL) {
            if(range > 0) {
                if(featureInfo.abs_control == DC1394_TRUE) {
                    // use absolute value
//                    logger->log("Using abs value for feature %d", featureIndex + FEATURE_MIN);
                    value = absToInt(featureInfo.abs_value, &exp);
//                    logger->log("value ~= %d * 10^-%d", value, exp);
                    featureSlider[featureIndex]->setValue(value);
                    featureSpinBox[featureIndex]->setValue(value);
                }
                else {
                    // use the 'normal' value
                    if(featureValPercent[featureIndex]) {
                        featureSlider[featureIndex]->setValue(featureInfo.value * 100 / range);
                        featureSpinBox[featureIndex]->setValue(featureInfo.value * 100 / range);
                    }
                    else {
                        featureSlider[featureIndex]->setValue(featureInfo.value);
                        featureSpinBox[featureIndex]->setValue(featureInfo.value);
                    }
                }
            }
            else {
                featureSlider[featureIndex]->setEnabled(false);
                if(featureSpinBox[featureIndex]->isEnabled() &&
                   featureSpinBox[featureIndex]->hasFocus()) {
                    focusdata->next()->setFocus();
                }
                featureSpinBox[featureIndex]->setEnabled(false);
            }
        }
        break;
    }
    
    //    Auto-Mode CheckBox Discard
    if((featureAutoModeCheckBox[featureIndex] != NULL) &&
       featureAutoModeCheckBox[featureIndex]->isEnabled()) {
        featureAutoModeCheckBox[featureIndex]->setChecked(featureInfo.auto_active);
    }
    
    //    On CheckBox Discard
    initialized = false;
    if((featureOnCheckBox[featureIndex] != NULL) &&
       featureOnCheckBox[featureIndex]->isEnabled()) {
        featureOnCheckBox[featureIndex]->setChecked(featureInfo.is_on);
    }
    initialized = initSave;
    
    // Trigger-Discard
    if(featureIndex == FEATURE_TRIGGER - FEATURE_MIN) {
        if(dc1394->swTriggerAvailable(id))
            swTriggerPushButton->setEnabled(false);
//            swTriggerPushButton->setEnabled(featureInfo.is_on);
        else
            swTriggerPushButton->setEnabled(false);
        
        if(triggerPolarityComboBox->isEnabled()) {
            triggerPolarityComboBox->setCurrentItem((featureInfo.trigger_polarity ==
                                                     DC1394_TRUE) ? 1 : 0);
        }
        
        if(triggerModeComboBox->isEnabled()) {
            int mode = dc1394->getTriggerMode(id);
            if(mode < 0)
                logger->log("AdjustmentForm::discardUIChanges: Unable to read current trigger-mode!");
            else
                triggerModeComboBox->setCurrentText(QString::number(mode));
        }
        
        if(triggerSourceComboBox->isEnabled()) {
            unsigned int source;
            if(!dc1394->getCurrentTriggerSource(id, &source)) {
                logger->log("AdjustmentForm::discardUIChanges: Reading current trigger-source failed!");
            }
            else {
                triggerSourceComboBox->setCurrentText(QString::number(source));
            }
        }
    }
}


void AdjustmentsForm::exitReject() {
    AdjustmentsCloseEvent *closeEvent = new AdjustmentsCloseEvent();
    bool unappliedChanges = false;
    
    if(!rejecting) {
        // find out if there are some 'unapplied' changes
        if(applyPushButton->isEnabled() || discardPushButton->isEnabled())
            unappliedChanges = true;
        
        // if so
        if(unappliedChanges) {
            // ask for rejecting them
            if(QMessageBox::question(this, "'Unapplied' changes'",
                                     "Some changes are not yet 'applied'\nExit anyway?",
                                     QMessageBox::Yes | QMessageBox::Default,
                                     QMessageBox::No | QMessageBox::Escape,
                                     QMessageBox::NoButton)
                == QMessageBox::No) {
                
                // just exit this function, if the changes should not be rejected
                return;
            }
        }
        
        // if trigger is avtive
        if(triggerOnCheckBox->isEnabled() && triggerOnCheckBox->isChecked()) {
            if(QMessageBox::warning(this, "Picture Control",
                                    "Trigger is still active - Deactivate before closing the Dialog?",
                                    QMessageBox::Yes | QMessageBox::Default,
                                    QMessageBox::No | QMessageBox::Escape,
                                    QMessageBox::NoButton) == QMessageBox::Yes) {
                // disable Trigger
                if(!dc1394->setFeatureOn(id, FEATURE_TRIGGER - FEATURE_MIN, false)) {
                    logger->log("Disabling Trigger failed!");
                }
            }
        }
                                 
        
        // discard changes for next-time-opening
        featuresDiscard();
        
        QApplication::postEvent(parent, closeEvent);
        
        rejecting = true;
        
        // Cancel Button clicked - reject all changes and exit
        reject();
    }
}


void AdjustmentsForm::exitAccept() {
    AdjustmentsCloseEvent *closeEvent = new AdjustmentsCloseEvent();
    
    // find out if there are some 'unapplied' changes
    for(int i = 0; i < NUM_FEATURES; i++) {
        if(featureChanged[i]) {
            // apply the changed settings
            applyUIChanges(i);
        }
    }
    if(shadingCorrectionChanged || lutChanged || hdrChanged || f7Changed || f7BPPChanged ||
       autoAOIChanged)
        featuresApply();
    
    QApplication::postEvent(parent, closeEvent);
    
    accept();
}


void AdjustmentsForm::closeEvent(QCloseEvent *e) {
    if(!rejecting)
        exitReject();
    e->accept();
}


void AdjustmentsForm::disableFeatureUIControls(int featureIndex) {
    if(featureGroup[featureIndex] == NULL)
        return;
    
    featureGroup[featureIndex]->setEnabled(false);
    if((featureIndex == FEATURE_SHUTTER - FEATURE_MIN) &&
       !dc1394->extShutterAvailable(id))
        exposureTimeGroupBox->setEnabled(false);
    
    if(featureIndex == FEATURE_WHITE_BALANCE - FEATURE_MIN) {
        whiteBalUSlider->setMinValue(-100);
        whiteBalUSlider->setMaxValue(100);
        whiteBalUSlider->setValue(0);
        
        whiteBalUSpinBox->setMinValue(-100);
        whiteBalUSpinBox->setMaxValue(100);
        whiteBalUSpinBox->setValue(0);
        
        whiteBalVSlider->setMinValue(-100);
        whiteBalVSlider->setMaxValue(100);
        whiteBalVSlider->setValue(0);
        
        whiteBalVSpinBox->setMinValue(-100);
        whiteBalVSpinBox->setMaxValue(100);
        whiteBalVSpinBox->setValue(0);
    } // white-balance
    else {
        switch(featureIndex) {
        case (FEATURE_HUE - FEATURE_MIN):
            featureSlider[featureIndex]->setMinValue(-40);
            featureSlider[featureIndex]->setMaxValue(40);
            featureSlider[featureIndex]->setValue(0);
            
            featureSpinBox[featureIndex]->setMinValue(-40);
            featureSpinBox[featureIndex]->setMaxValue(40);
            featureSpinBox[featureIndex]->setValue(0);
            break;
            
        case (FEATURE_SATURATION - FEATURE_MIN):
            featureSlider[featureIndex]->setMinValue(0);
            featureSlider[featureIndex]->setMaxValue(200);
            featureSlider[featureIndex]->setValue(100);
            
            featureSpinBox[featureIndex]->setMinValue(0);
            featureSpinBox[featureIndex]->setMaxValue(200);
            featureSpinBox[featureIndex]->setValue(100);
            break;
            
        default:
            featureSlider[featureIndex]->setMinValue(0);
            featureSlider[featureIndex]->setMaxValue(0);
            featureSlider[featureIndex]->setValue(0);
            
            featureSpinBox[featureIndex]->setMinValue(0);
            featureSpinBox[featureIndex]->setMaxValue(0);
            featureSpinBox[featureIndex]->setValue(0);
            break;
        }
    } // 'normal' control
    
    featureAutoModeCheckBox[featureIndex]->setChecked(false);
    featureOnCheckBox[featureIndex]->setChecked(false);
    if(featureIndex == FEATURE_TRIGGER - FEATURE_MIN) {
        swTriggerPushButton->setEnabled(false);
    }
    
}


void AdjustmentsForm::setFeatureUIState(int featureIndex, dc1394_feature_info featureInfo) {
    if(featureGroup[featureIndex] == NULL) {
        logger->log("AdjustmentsForm::setFeatureUIState: Error! Feature %d has no 'GroupBox'!",
                    featureIndex);
        return;
    }
    
    // enable the 'control-group'
    if(featureIndex == FEATURE_SHUTTER - FEATURE_MIN) {
        exposureTimeGroupBox->setEnabled(true);
        featureGroup[featureIndex]->setEnabled(!extShutterRangeCheckBox->isChecked());
    }
    else
        featureGroup[featureIndex]->setEnabled(true);
    
    // 'virtually' enable all features
    autoModeEnabled = true;
    valuesEnabled = true;
    onePushEnabled = true;
    triggerEnabled = true;
    
    // set the On check-box accordingly (and 'virtually' enable / disable the other features)
    setFeatureOn(featureIndex, featureInfo);
    
    switch(featureIndex) {
    case FEATURE_TRIGGER - FEATURE_MIN:
        // set the trigger controls accordingly (if available)
        setFeatureTrigger(featureInfo);
        break;
        
    default:
        // update the AutoMode control (and 'virtually' enable / disable the other features)
        setFeatureAutoMode(featureIndex, featureInfo);
        
        // update the value controls
        setFeatureValues(featureIndex, featureInfo);
        
        // enable / disable the OnePush button
        setFeatureOnePush(featureIndex, featureInfo);
        break;
    }
    
}


void AdjustmentsForm::setFeatureOn(int featureIndex, dc1394_feature_info featureInfo) {
    if(featureInfo.on_off_capable) {
        // if available - enable the 'control'
        featureOnCheckBox[featureIndex]->setEnabled(true);
        
        // if the state of the control has not changed
        if(featureOnCheckBox[featureIndex]->isChecked() == featureInfo.is_on) {
            // the control may be about to be initialized
            initOn[featureIndex] = false;
        }
        else {
            // set the 'on-off'-check-box according to the state of the camera
            featureOnCheckBox[featureIndex]->setChecked(featureInfo.is_on);
            if(featureIndex == FEATURE_TRIGGER - FEATURE_MIN) {
                if(dc1394->swTriggerAvailable(id))
                    swTriggerPushButton->setEnabled(false);
//                    swTriggerPushButton->setEnabled(featureInfo.is_on);
                else
                    swTriggerPushButton->setEnabled(false);
            }
        }
        
        // if the state is 'off' - disable the other controls
        if(!featureInfo.is_on) {
            // 'virtually' disable all other controls
            autoModeEnabled = false;
            valuesEnabled = false;
            onePushEnabled = false;
            triggerEnabled = false;
        }
    }
    else { // On/Off not available
        logger->log("AdjustmentsForm::setFeatureOn: Info - Feature %d is not 'on_off_capable'",
                    featureIndex);
        // if an OnCheckBox is availabe
        if(featureOnCheckBox[featureIndex] != NULL) {
            // disable
            featureOnCheckBox[featureIndex]->setEnabled(false);
            // and 'uncheck' 'em
            featureOnCheckBox[featureIndex]->setChecked(false);
            if(featureIndex == FEATURE_TRIGGER - FEATURE_MIN) {
                swTriggerPushButton->setEnabled(false);
            }
        }
    }
}


void AdjustmentsForm::setFeatureAutoMode(int featureIndex, dc1394_feature_info featureInfo) {
    // AutoMode
    if(featureInfo.auto_capable) {
        // if available - enable the 'control' (if not switched off)
        featureAutoModeCheckBox[featureIndex]->setEnabled(autoModeEnabled);
        
        // if the state of the control has not changed
        if(featureAutoModeCheckBox[featureIndex]->isChecked() == featureInfo.auto_active) {
            // the control may be about to be initialized
            initAutoMode[featureIndex] = false;
        }
        else {
            // set the 'auto-mode'-check-box according to the state of the camera
            featureAutoModeCheckBox[featureIndex]->setChecked(featureInfo.auto_active);
        }
        
        // if AutoMode is enabled 'virtually' disable the other controls
        if(featureInfo.auto_active) {
            valuesEnabled = false;
            onePushEnabled = false;
            triggerEnabled = false;
        }
    }
    else { // AutoMode not available
        logger->log("AdjustmentsForm:setFeatureAutoMode: Info - Feature %d has no AutoMode!",
                    featureIndex);
        // if an AutoMode check-box is available
        if(featureAutoModeCheckBox[featureIndex] != NULL) {
            // disable
            featureAutoModeCheckBox[featureIndex]->setEnabled(false);
            // and 'uncheck' 'em
            featureAutoModeCheckBox[featureIndex]->setChecked(false);
        }
    }
}


void AdjustmentsForm::setFeatureValues(int featureIndex, dc1394_feature_info featureInfo) {
    int featureRange;
    int value, steps;
    int normValue, normRangeMin, normRangeMax, normSteps = 1;
    int rangeMin = 0, rangeMax = 100;
    int exp = 0, minExp = 0, maxExp = 0;
    QString suffix;
    
    
    // calculate the range
    featureRange = featureInfo.max - featureInfo.min;
    
    // enable / disable and set the value-controls according to the calculated range
    if(featureInfo.manual_capable && (featureRange > 0)) {
        // calculate the 'steps' for the value-controls
        if(featureRange > 100) {
            steps = 1;
        }
        else {
            steps = 100 / featureRange;
        }
        
        
        // white balance-value-controls are different from the other ones (split into U and V!)
        if(featureIndex == FEATURE_WHITE_BALANCE - FEATURE_MIN) {
            normRangeMax = (featureInfo.max - featureInfo.min) / 2;
            normRangeMin = -normRangeMax;
            
            whiteBalUSlider->setEnabled(valuesEnabled);
            if(!valuesEnabled && whiteBalUSpinBox->isEnabled() &&
               whiteBalUSpinBox->hasFocus()) {
                focusdata->next()->setFocus();
            }
            whiteBalUSpinBox->setEnabled(valuesEnabled);
            
            whiteBalVSlider->setEnabled(valuesEnabled);
            if(!valuesEnabled && whiteBalVSpinBox->isEnabled() &&
               whiteBalVSpinBox->hasFocus()) {
                focusdata->next()->setFocus();
            }
            whiteBalVSpinBox->setEnabled(valuesEnabled);
            
            // U
            //     value = featureInfo.BU_value * 100 / featureRange;
            normValue = featureInfo.BU_value - (featureRange / 2);
            
            whiteBalUSlider->setRange(normRangeMin, normRangeMax);
            whiteBalUSlider->setSteps(normSteps, normSteps);
            
            whiteBalUSpinBox->setRange(normRangeMin, normRangeMax);
            whiteBalUSpinBox->setSteps(normSteps, normSteps);
            
            if(normValue == whiteBalUSlider->value()) {
                initWhiteBalUSlider = 0;
                initWhiteBalUSpinBox = 0;
            }
            
            whiteBalUSlider->setValue(normValue);
            whiteBalUSpinBox->setValue(normValue);
            
            // V
            //     value = featureInfo.RV_value * 100 / featureRange;
            normValue = featureInfo.RV_value - (featureRange / 2);
            
            whiteBalVSlider->setRange(normRangeMin, normRangeMax);
            whiteBalVSlider->setSteps(normSteps, normSteps);
            
            whiteBalVSpinBox->setRange(normRangeMin, normRangeMax);
            whiteBalVSpinBox->setSteps(normSteps, normSteps);
            
            if(normValue == whiteBalVSlider->value()) {
                initWhiteBalVSlider = 0;
                initWhiteBalVSpinBox = 0;
            }
            
            whiteBalVSlider->setValue(normValue);
            whiteBalVSpinBox->setValue(normValue);
        } // white-balance
        else { // 'normal' control
            if(featureSlider[featureIndex] != NULL) {
                featureSlider[featureIndex]->setEnabled(valuesEnabled);
                if(!valuesEnabled &&
                   featureSpinBox[featureIndex]->isEnabled() &&
                   featureSpinBox[featureIndex]->hasFocus()) {
                    focusdata->next()->setFocus();
                }
                featureSpinBox[featureIndex]->setEnabled(valuesEnabled);
                
                if(featureInfo.abs_control == DC1394_TRUE) {
                    logger->log("AdjustmentsForm::setFeatureValues: Using absolute value for feature %d",
                           featureIndex + FEATURE_MIN);
                    // translate float into a usefull integer value and set the unit accordingly
                    value = absToInt(featureInfo.abs_value, &exp);
                    logger->log("\tvalue ~= %d * 10^-%d", value, exp);
                    rangeMin = absToInt(featureInfo.abs_min, &minExp);
                    logger->log("\tMin-Value ~=%d * 10^-%d", rangeMin, minExp);
                    rangeMax = absToInt(featureInfo.abs_max, &maxExp);
                    logger->log("\tMax-Value ~=%d * 10^-%d", rangeMax, maxExp);
                    logger->log("\tValue = %d, Min = %d, Max = %d",
                           value * ((int)pow(10, minExp) /  (int)pow(10, exp)),
                           rangeMin,
                           rangeMax * ((int)pow(10, minExp) / (int)pow(10, maxExp)));
                    
                    value = value * ((int)pow(10, minExp) /  (int)pow(10, exp));
                    rangeMax = rangeMax * ((int)pow(10, minExp) / (int)pow(10, maxExp));
                    
                    steps = 1;
                    
                }
                else {
                    // case-selection - values determined according to their functionality
                    value = calculateValue(featureIndex, featureInfo,
                                           &rangeMin, &rangeMax, &steps);
                }
                
                suffix = determineSuffix(featureIndex, featureInfo);
                
                logger->log("AdjustmentsForm::setFeatureValue: Feature %s: %d, Min = %d, Max = %d, Steps = %d %s",
                       DC1394::featureString(featureIndex),
                       value, rangeMin, rangeMax, steps,
                       (featureValPercent[featureIndex] ? "(converted to '%')" : ""));
                
                featureSlider[featureIndex]->setRange(rangeMin, rangeMax);
                featureSlider[featureIndex]->setSteps(steps, steps);
                
                featureSpinBox[featureIndex]->setRange(rangeMin, rangeMax);
                featureSpinBox[featureIndex]->setSteps(steps, steps);
                
                if(value == featureSlider[featureIndex]->value()) {
                    initSlider[featureIndex] = 0;
                    initSpinBox[featureIndex] = 0;
                }
                
                featureSlider[featureIndex]->setValue(value);
                featureSpinBox[featureIndex]->setValue(value);
                featureSpinBox[featureIndex]->setSuffix(suffix);
            }
        } // 'normal' control
    } // range > 0
    else { // range <= 0
        // slider and spin-box will be disabled if the range is > 0!
        logger->log("AdjustmentsForm::setFeatureValue: Feature: %d, Min: %d, Max: %d",
                    featureIndex, featureInfo.min, featureInfo.max);
        if(featureIndex == FEATURE_WHITE_BALANCE - FEATURE_MIN) {
            whiteBalUSlider->setEnabled(false);
            if(whiteBalUSpinBox->isEnabled() && whiteBalUSpinBox->hasFocus()) {
                focusdata->next()->setFocus();
            }
            whiteBalUSpinBox->setEnabled(false);
            
            whiteBalVSlider->setEnabled(false);
            if(whiteBalVSpinBox->isEnabled() && whiteBalVSpinBox->hasFocus()) {
                focusdata->next()->setFocus();
            }
            whiteBalVSpinBox->setEnabled(false);
        }
        else {
            if(featureSlider[featureIndex] != NULL) {
                featureSlider[featureIndex]->setEnabled(false);
                if(featureSpinBox[featureIndex]->isEnabled() &&
                   featureSpinBox[featureIndex]->hasFocus()) {
                    focusdata->next()->setFocus();
                }
                featureSpinBox[featureIndex]->setEnabled(false);
            }
        }
    } // range <= 0
}


int AdjustmentsForm::calculateValue(int featureIndex, dc1394_feature_info featureInfo,
                                    int *rangeMin, int *rangeMax, int *steps) {
    int value = featureInfo.value;
    
    *rangeMin = featureInfo.min;
    *rangeMax = featureInfo.max;
    
    switch(featureIndex + FEATURE_MIN) {
    case FEATURE_BRIGHTNESS:
    case FEATURE_GAIN:
    case FEATURE_GAMMA:
    case FEATURE_SHARPNESS:
    case FEATURE_IRIS:
    case FEATURE_FOCUS:
    case FEATURE_OPTICAL_FILTER:
    case FEATURE_PAN:
    case FEATURE_TILT:
    case FEATURE_SHUTTER:
    case FEATURE_TRIGGER:
    case FEATURE_TEMPERATURE:
    case FEATURE_CAPTURE_SIZE:
    case FEATURE_CAPTURE_QUALITY:
        *steps = 1;
        featureValPercent[featureIndex] = false;
        break;
    case FEATURE_HUE:
        *steps = 1;
        if((featureInfo.max - featureInfo.min) != 0) {
            *rangeMax = (featureInfo.max - featureInfo.min) / 2;
            *rangeMin = -(*rangeMax);
            value = featureInfo.value - (*rangeMax);
        }
        else {
            *rangeMax = 100;
            *rangeMin = -(*rangeMax);
            value = 0;
        }     
        featureValPercent[featureIndex] = true;
        break;
    case FEATURE_SATURATION:
        *steps = 1;
        *rangeMax = 200;
        *rangeMin = 0;
        if((*rangeMax) != 0)
            value = (featureInfo.value * 200) / (*rangeMax);
        else
            value = 100;
        featureValPercent[featureIndex] = true;
        break;
    default:
        if((featureInfo.max - featureInfo.min) != 0)
            value = (featureInfo.value * 100) / (featureInfo.max - featureInfo.min);
        else
            value = 0;
        *rangeMin = 0;
        *rangeMax = 100;
        featureValPercent[featureIndex] = true;
        break;
    }
    
    return value;
}


QString AdjustmentsForm::determineSuffix(int featureIndex, dc1394_feature_info featureInfo) {
    QString suffix = "";
    
    switch(featureIndex + FEATURE_MIN) {
    case FEATURE_GAIN:
        suffix = (featureInfo.abs_control == DC1394_TRUE) ? "dB" : "";
        break;
    case FEATURE_HUE:
        suffix = (featureInfo.abs_control == DC1394_TRUE) ? " deg." : " deg.";
        break;
    case FEATURE_SATURATION:
        suffix = (featureInfo.abs_control == DC1394_TRUE) ? "%" : "%";
        break;
    case FEATURE_IRIS:
        suffix = (featureInfo.abs_control == DC1394_TRUE) ? " F" : " F";
        break;
    case FEATURE_FOCUS:
        suffix = (featureInfo.abs_control == DC1394_TRUE) ? "m" : "m";
        break;
    case FEATURE_PAN:
        suffix = (featureInfo.abs_control == DC1394_TRUE) ? " deg." : " deg.";
        break;
    case FEATURE_TILT:
        suffix = (featureInfo.abs_control == DC1394_TRUE) ? " deg." : " deg.";
        break;
    case FEATURE_SHUTTER:
        suffix = (featureInfo.abs_control == DC1394_TRUE) ? "us" : "";
        break;
    case FEATURE_TRIGGER:
        suffix = (featureInfo.abs_control == DC1394_TRUE) ? " times" : "";
        break;
    case FEATURE_TEMPERATURE:
        suffix = (featureInfo.abs_control == DC1394_TRUE) ? "C" : "C";
        break;
    case FEATURE_BRIGHTNESS:
        suffix = (featureInfo.abs_control == DC1394_TRUE) ? "%" : "";
        break;
    case FEATURE_ZOOM:
        suffix = (featureInfo.abs_control == DC1394_TRUE) ? " power" : " power";
        break;
    case FEATURE_EXPOSURE:
        suffix = (featureInfo.abs_control == DC1394_TRUE) ? " EV" : "%";
        break;
    default:
        break;
    }
    
    return suffix;
}


int AdjustmentsForm::absToInt(float absValue, int *exp) {
    float fVal;
    int iVal = (int)absValue;
    
    *exp = 0;
    
    if(absValue == 0.000000f)
        return 0;
    
    for(int i = 0; i <= 6; i += 3) {
        fVal = absValue * pow(10, i);
        iVal = (int)fVal;
        if(iVal != 0) {
            *exp = i;
            break;
        }
    }
    
    return iVal;
}


void AdjustmentsForm::setFeatureOnePush(int featureIndex, dc1394_feature_info featureInfo) {
    // OnePush is just a PushButton
    // - hence it only has to be enabled / disabled according to its availability
    if(featureInfo.one_push) {
        featureOnePushPushButton[featureIndex]->setEnabled(onePushEnabled);
    }
    else {
        if(featureOnePushPushButton[featureIndex] != NULL) {
            featureOnePushPushButton[featureIndex]->setEnabled(false);
        }
    }
}    


void AdjustmentsForm::setFeatureTrigger(dc1394_feature_info featureInfo) {
    QString trigModeString;
    int mode;
    unsigned int triggerModeCapableMask;
    unsigned int triggerSource;
    
    
    // output the trigger-values
    logger->log("AdjustmentsForm::setFeatureTrigger: Mode-Capable-Mask = 0x%x, Mode: %d, Polarity: %s",
           featureInfo.trigger_mode_capable_mask,
           featureInfo.trigger_mode,
           featureInfo.trigger_polarity ? "H" : "L");
    
    if(!dc1394->getTriggerModeCapableMask(id, &triggerModeCapableMask))
        logger->log("AdjustmentsForm::setFeatureTrigger: Error while trying to read 'Trigger-Mode-Capable Mask'!");
    else
        logger->log("\tTrigger-Mode-Capable Mask = 0x%x", triggerModeCapableMask);
    
    triggerModeComboBox->clear();
    
    for(int i = 0; i < 4; i++) {
        // if mode
        if(((featureInfo.trigger_mode_capable_mask << i) & 0x08) != 0) {
            logger->log("\tTrigger-Mode %d supported", i);
            triggerModeComboBox->insertItem(QString::number(i));
        }
        else
            logger->log("\tTrigger-Mode %d not supported", i);
    }
    
    if((triggerModeCapableMask & 0x01) != 0) {
        logger->log("\tTrigger-Mode 15 also available");
        triggerModeComboBox->insertItem(QString::number(15));
    }
    else
        logger->log("\tTrigger-Mode 15 not supported");
    
    // if the capable_mask is != 0 -> enable the control
    if(featureInfo.trigger_mode_capable_mask != 0) {
        triggerModeComboBox->setEnabled(true);
        mode = dc1394->getTriggerMode(id);
        if(mode < 0)
            logger->log("AdjustmentsForm::setFeatureTrigger: Unable to read current trigger-mode!");
        else {
            logger->log("\tCurrently selected Trigger-Mode: %d", mode);
            triggerModeComboBox->setCurrentText(QString::number(mode));
        }
    }
    else {
        triggerModeComboBox->setEnabled(false);
    }
    
    // trigger-sources
    if(!dc1394->getTriggerSources(id, &triggerSource)) {
        logger->log("AdjustmentsForm::setFeatureTrigger: Reading Trigger-Sources failed - disabling Trigger!");
        triggerGroupBox->setEnabled(false);
        return;
    }
    
    triggerSourceComboBox->clear();
    
    logger->log("\tTrigger-Sources: 0x%x", triggerSource);
    
    for(int i = 0; i < 4; i++) {
        if(((triggerSource << i) & 0x80) != 0) {
            logger->log("\tTrigger-Source %d available", i);
            triggerSourceComboBox->insertItem(QString::number(i));
            triggerSourceComboBox->setEnabled(true);
        }
        else {
            logger->log("\tTrigger-Source %d not available!", i);
        }
    }
    
    if(triggerSourceComboBox->count() == 0) {
        logger->log("\tOfficially no TriggerModes available - forcing Mode 0!");
        triggerSourceComboBox->insertItem(QString::number(0));
        triggerSourceComboBox->setEnabled(true);
    }
    
    if((triggerSource & 0x01) != 0) {
        logger->log("\tSW-Trigger supported");
        triggerSourceComboBox->insertItem(QString::number(7));
    }
    else {
        logger->log("\tSW-Trigger not supported!");
    }
    
    if(!dc1394->getCurrentTriggerSource(id, &triggerSource)) {
        logger->log("AdjustmentsForm::setFeatureTrigger: Reading current trigger source failed!");
    }
    else {
        logger->log("\tCurrently selected Trigger-Source: %d", triggerSource);
        if(triggerSource == 7)
            triggerSourceComboBox->setCurrentText(QString::number(7));
        else
            triggerSourceComboBox->setCurrentText(QString::number(triggerSource));
    }
    
    // trigger-polarity
    if(featureInfo.polarity_capable) {
        triggerPolarityComboBox->setEnabled(true);
        triggerPolarityComboBox->setCurrentItem((featureInfo.trigger_polarity ==
                                                 DC1394_TRUE) ? 1 : 0);
    }
    else {
        triggerPolarityComboBox->setEnabled(false);
    }
    
    if(dc1394->swTriggerAvailable(id))
        swTriggerPushButton->setEnabled(false);
//        swTriggerPushButton->setEnabled(triggerEnabled);
    else
        swTriggerPushButton->setEnabled(false);
}



void AdjustmentsForm::featureSliderChanged(int featureIndex, int newValue) {
    if(initSlider[featureIndex] > 0) {
        featureSpinBox[featureIndex]->setValue(newValue);
        
        initSlider[featureIndex]--;
        
        return;
    }
    
    featureSpinBox[featureIndex]->setValue(newValue);
    
    if(initialized) {
        // enable the apply- and discard-button
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        // set feature-changed-flag
        featureChanged[featureIndex] = true;
    }
}



void AdjustmentsForm::featureSpinBoxChanged(int featureIndex, int newValue) {
    if(initSpinBox[featureIndex] > 0) {
        featureSlider[featureIndex]->setValue(newValue);
        
        initSpinBox[featureIndex]--;
        
        return;
    }
    
    featureSlider[featureIndex]->setValue(newValue);
    
    if(initialized) {
        // enable the apply- and discard-button
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        // set feature-changed-flag
        featureChanged[featureIndex] = true;
    }
}


void AdjustmentsForm::featureAutoModeChanged(int featureIndex, int newValue) {
    featureAutoModeCheckBox[featureIndex]->setChecked(newValue);
    
    if(initAutoMode[featureIndex]) {
        initAutoMode[featureIndex] = false;
        return;
    }
    
    if(initialized) {
        // enable the apply- and discard-button
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        // set feature-changed-flag
        featureChanged[featureIndex] = true;
    }
}


void AdjustmentsForm::featureOnChanged(int featureIndex, int) {
    //    featureOnCheckBox[featureIndex]->setChecked(newValue);
    
    if(initOn[featureIndex]) {
        initOn[featureIndex] = false;
        return;
    }
    
    if(initialized) {
        // enable the apply- and discard-button
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        // set feature-changed-flag
        featureChanged[featureIndex] = true;
    }
}


void AdjustmentsForm::featureOnePushActivated(int featureIndex) {
    // disable (at least) the button, as long as the procedure is working
    featureGroup[featureIndex]->setEnabled(false);
    
    // OnePush will execute directly (without the need of 'activating' this feature!)
    // !! there is also no 'discard' available
    if(!dc1394->featureOnePush(id, featureIndex)) {
        logger->log("AdjustmentsForm::featureOnePushActivated: Starting OnePush-Execution failed!");
    }
    
    // change the cursor-shape
    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
    
    // wait for the camera to be ready after the 'OnePush' procedure
    do {
        // read current camera-features-status
        featureSet = dc1394->featureSet(id);
    } while((featureSet.feature[featureIndex]).one_push_active);
    
    // restore the cursor-shape
    QApplication::restoreOverrideCursor();
    
    // and enable the button again
    featureGroup[featureIndex]->setEnabled(true);
    
    // update the UI for displaying the automatically detected values
    updateUI();
}


// special white-balance handling
void AdjustmentsForm::whiteBalUSliderChanged(int newValue) {
    bool initSave = initialized;
    
    
    if(initialized) {
        initialized = false;
        whiteBalUSpinBox->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setWhiteBalanceValue(id,
                                         newValue + whiteBalUSlider->maxValue(),
                                         whiteBalVSpinBox->value() + whiteBalVSpinBox->maxValue()))
            logger->log("AdjustmentsForm::whiteBalUSliderChanged: Trying to change WHITE BALANCE 'U' failed!");
    }
    
    return;
    
    if(initWhiteBalUSlider > 0) {
        whiteBalUSpinBox->setValue(newValue);
        initWhiteBalUSlider--;
        return;
    }
    
    whiteBalUSpinBox->setValue(newValue);
    
    if(initialized) {
        // enable the apply- and discard-button
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        // set feature-changed-flag
        featureChanged[FEATURE_WHITE_BALANCE - FEATURE_MIN] = true;
    }
}


void AdjustmentsForm::whiteBalUSpinBoxChanged(int newValue) {
    bool initSave = initialized;
    
    
    if(initialized) {
        initialized = false;
        whiteBalUSlider->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setWhiteBalanceValue(id,
                                         newValue + whiteBalUSpinBox->maxValue(),
                                         whiteBalVSpinBox->value() + whiteBalVSpinBox->maxValue()))
            logger->log("AdjustmentsForm::whiteBalUSpinBoxChanged: Trying to change WHITE BALANCE 'U' failed!");
    }
    
    return;
    
    if(initWhiteBalUSpinBox > 0) {
        whiteBalUSlider->setValue(newValue);
        initWhiteBalUSpinBox--;
        return;
    }
    
    whiteBalUSlider->setValue(newValue);
    
    if(initialized) {
        // enable the apply- and discard-button
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        // set feature-changed-flag
        featureChanged[FEATURE_WHITE_BALANCE - FEATURE_MIN] = true;
    }
}


void AdjustmentsForm::whiteBalVSliderChanged(int newValue) {
    bool initSave = initialized;
    
    
    if(initialized) {
        initialized = false;
        whiteBalVSpinBox->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setWhiteBalanceValue(id,
                                         whiteBalUSpinBox->value() + whiteBalUSpinBox->maxValue(),
                                         newValue + whiteBalVSlider->maxValue()))
            logger->log("AdjustmentsForm::whiteBalVSliderChanged: Trying to change WHITE BALANCE 'V' failed!");
    }
    
    return;
    
    if(initWhiteBalVSlider > 0) {
        whiteBalVSpinBox->setValue(newValue);
        initWhiteBalVSlider--;
        return;
    }
    
    whiteBalVSpinBox->setValue(newValue);
    
    if(initialized) {
        // enable the apply- and discard-button
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        // set feature-changed-flag
        featureChanged[FEATURE_WHITE_BALANCE - FEATURE_MIN] = true;
    }
}


void AdjustmentsForm::whiteBalVSpinBoxChanged(int newValue) {
    bool initSave = initialized;
    
    
    if(initialized) {
        initialized = false;
        whiteBalVSlider->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setWhiteBalanceValue(id,
                                         whiteBalUSpinBox->value() + whiteBalUSpinBox->maxValue(),
                                         newValue + whiteBalVSpinBox->maxValue()))
            logger->log("AdjustmentsForm::whiteBalVSpinBoxChanged: Trying to change WHITE BALANCE 'V' failed!");
    }
    
    return;
    
    if(initWhiteBalVSpinBox > 0) {
        whiteBalVSlider->setValue(newValue);
        initWhiteBalVSpinBox--;
        return;
    }
    
    whiteBalVSlider->setValue(newValue);
    
    if(initialized) {
        // enable the apply- and discard-button
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        // set feature-changed-flag
        featureChanged[FEATURE_WHITE_BALANCE - FEATURE_MIN] = true;
    }
}


// special trigger handling
void AdjustmentsForm::triggerPolarityChanged(int newValue) {
    dc1394bool_t triggerPolarity = ((triggerPolarityComboBox->currentItem()) == 1) ?
                                   DC1394_TRUE : DC1394_FALSE;
        
    if(!dc1394->setTriggerPolarity(id, triggerPolarity)) {
        logger->log("AdjustmentsForm::triggerPolarityChnaged: Changing Trigger-Polarity to %s failed!", 
                    triggerPolarity ? "low act." : "hight act.");
    }
    
    logger->log("AdjustmentsForm::triggerPolarityChanged: ... to %s",
                (newValue == 1) ? "'high-active'" : "'low-active'");
}


void AdjustmentsForm::triggerModeChanged( const QString& newModeString ) {
    if(!dc1394->setTriggerMode(id, (triggerModeComboBox->currentText()).toInt())) {
        logger->log("AdjustmentsForm::triggerModeChanged: Changing Trigger-Mode to %s failed",
                    (triggerModeComboBox->currentText()).ascii());
    }
    
    logger->log("AdjustmentsForm::triggerModeChanged: Selected Mode: %s",
                newModeString.ascii());
}


void AdjustmentsForm::triggerSourceChanged(const QString& newSourceString) {
    if(!dc1394->setTriggerSource(id, (triggerSourceComboBox->currentText()).toInt())) {
        logger->log("AdjustmentsForm::triggerSourceChanged: Enabling Trigger-Source %s failed!",
                    (triggerSourceComboBox->currentText()).ascii());
    }
    
    logger->log("AdjustmentsForm::triggerSourceChanged: Source %s selected",
                newSourceString.ascii());
}


// ??????????????? 'manually' implemented SW-Trigger ?????????
void AdjustmentsForm::swTrigger() {
    CamWindow *camWindow;
    unsigned int count = 3;
    unsigned int currentMode;
    bool state;
    
    
    if(QString(parent->className()) != "CamWindow") {
        logger->log("AdjustmentsForm::swTrigger: Parent of 'AdjustmentsForm' is '%s', but 'CamWindow' is expected!",
               parent->className());
        return;
    }
    else {
        camWindow  = (CamWindow *)parent;
    }
    
    if(featureChanged[FEATURE_TRIGGER - FEATURE_MIN]) {
        switch(QMessageBox::warning(this,
                                    "Trigger changed", "Trigger-Changes have not been applied!",
                                    "&Apply", "&Discard", "&Cancel", 0, 1)) {
        case 0:
            logger->log("AdjustmentsForm::swTrigger: Applying UI changes before trigger-execution");
            featuresApply();
            logger->log("\tChanges applied");
            break;
        case 1:
            logger->log("AdjustmentsForm::swTrigger: Discarding UI changes before trigger-execution");
            featuresDiscard();
            logger->log("\tChanges discarded");
            break;
        default:
            logger->log("AdjustmentsForm::swTrigger: Canceling trigger-execution without any changes");
            return;
        }
    }
    
    logger->log("AdjustmentsForm::swTrigger: SW-Trigger executed - current 'ISO-State':");
    dc1394->printISOState(id);
    
    // check if SW-Trigger is ready
    if(!dc1394->getSWTriggerState(id, &state)) {
        logger->log("AdjustmentsForm::swTrigger: Error while trying to read SW-Trigger state!");
        return;
    }
    
    if(state == SW_TRIGGER_BUSY) {
        QMessageBox::warning(this, "SW-Trigger", "SW-Trigger busy!",
                             QMessageBox::Ok | QMessageBox::Default,
                             QMessageBox::NoButton,
                             QMessageBox::NoButton);
        
        return;
    }
    
    logger->log("AdjustmentsForm::swTrigger: Ready for SW-Trigger - now executing SW-Trigger");
    
    if(!dc1394->startSWTriggerExecute(id)) {
        QMessageBox::warning(this, "SW-Trigger", "SW-Trigger failed!",
                             QMessageBox::Ok | QMessageBox::Default,
                             QMessageBox::NoButton,
                             QMessageBox::NoButton);
        
        return;
    }
    
    logger->log("AdjustmentsForm::swTrigger: SW-Trigger executed");
    
    currentMode = (triggerModeComboBox->currentText()).toUInt();
    if((currentMode == 1) || (currentMode == 15)) {
        swTriggerStopPushButton->setEnabled(true);
    }
    else {
        logger->log("\tSW-Trigger will end automatically - waiting for end");
        
        do{
            if(!dc1394->getSWTriggerState(id, &state)) {
                logger->log("AdjustmentsForm::swTrigger: Error while trying to read current Trigger-State!");
                break;
            }
            count--;
            if(state == SW_TRIGGER_BUSY)
                sleep(1);
            logger->log("\tTrigger %s",
                   state == SW_TRIGGER_READY ? "ready" : "active - still waiting");
        } while((state == SW_TRIGGER_BUSY) && (count > 0));
    }
    
    if(count == 0) {
        logger->log("AdjustmentsForm::swTrigger: Trigger was not reset automatically - now trying to reset 'manually'!");
        
        count = 3;
        
        do{
            if(!dc1394->stopSWTriggerExecute(id)) {
                logger->log("AdjustmentsForm::swTrigger: Error while trying to stop Trigger-Execution");
                break;
            }
            
            if(!dc1394->getSWTriggerState(id, &state)) {
                logger->log("AdjustmentsForm::swTrigger: Error while trying to read current Trigger-State!");
                break;
            }
            
            count--;
            if(state == SW_TRIGGER_BUSY)
                sleep(1);
            logger->log("\tTrigger %s",
                   state == SW_TRIGGER_READY ? "ready" : "already active - trying again to stop");
        } while((state == SW_TRIGGER_BUSY) && (count > 0));
        
        if(count == 0) {
            logger->log("AdjustmentsForm::swTrigger: Manual-SW-Trigger-Reset failed!");
        }
        else {
            logger->log("AdjustmentsForm::swTrigger: Manual-SW-Trigger-Reset succeeded");
        }
    }
}


void AdjustmentsForm::swTriggerStop() {
    CamWindow *camWindow;
    unsigned int currentMode;
    unsigned int count = 3;
    bool state;
    
    
    if(QString(parent->className()) != "CamWindow") {
        logger->log("AdjustmentsForm::swTriggerStop: Parent of 'AdjustmentsForm' is '%s', but 'CamWindow' is expected!",
               parent->className());
        logger->log("AdjustmentsForm::swTriggerStop: Therefore a change of the 'Timebase' is not possible!");
        QApplication::restoreOverrideCursor();
        return;
    }
    else {
        camWindow  = (CamWindow *)parent;
    }
    
    currentMode = (triggerModeComboBox->currentText()).toUInt();
    
    if(!dc1394->getSWTriggerState(id, &state)) {
        logger->log("AdjustmentsForm::swTriggerStop: Error while trying to read current SW-Trigger-State!");
        return;
    }
    
    logger->log("AdjustmentsForm::swTriggerStop: Current SW-Trigger State: %s",
                state == SW_TRIGGER_READY ? "ready" : "busy");
    
    if(state == SW_TRIGGER_READY)
        return;
    
    if(!dc1394->stopSWTriggerExecute(id)) {
        QMessageBox::warning(this, "SW-Trigger", "Stopping SW-Trigger failed!",
                             QMessageBox::Ok | QMessageBox::Default,
                             QMessageBox::NoButton,
                             QMessageBox::NoButton);
        
        return;
    }
    
    do{
        if(!dc1394->getSWTriggerState(id, &state))
            logger->log("AdjustmentsForm::swTriggerStop: Error while trying to read SW-Trigger State!");
        
        count--;
        if(state == SW_TRIGGER_BUSY)
            sleep(1);
        logger->log("\tTrigger %s", state == SW_TRIGGER_READY ? "ready" : "already active - still waiting");
    } while((state == SW_TRIGGER_BUSY) && (count > 0));
    
    if(count == 0) {
        logger->log("AdjustmentsForm::swTriggerStop: Trigger-Stop failed");
    }
    
    swTriggerStopPushButton->setEnabled(false);
}


// brightness control slots
void AdjustmentsForm::brightnessSliderChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_BRIGHTNESS - FEATURE_MIN;
    
    
    if(initialized) {
        initialized = false;
        featureSpinBox[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::brightnessSliderChanged: Trying to change BRIGHTNESS failed!");
            updateUI();
        }
    }    
}


void AdjustmentsForm::brightnessSpinBoxChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_BRIGHTNESS - FEATURE_MIN;
    
    if(initialized) {
        initialized = false;
        featureSlider[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::brightnessSpinBoxChanged: Trying to change BRIGHTNESS failed!");
            updateUI();
        }
    }    
}


void AdjustmentsForm::brightnessAutoModeChecked( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_BRIGHTNESS - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureAutoMode(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::brightnessAutoModeChecked: Trying to enable Auto-Mode failed!");
            initialized = false;
            featureAutoModeCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::brightnessOnChecked( bool newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_BRIGHTNESS - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureOn(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::brightnessOnChecked: Error while trying to %s the feature", newValue ? "enable" : "disable");
            initialized = false;
            featureOnCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::brightnessOnePushActivated() {
    int featureIndex = FEATURE_BRIGHTNESS - FEATURE_MIN;
    
    featureOnePushActivated(featureIndex);
}


// gain control slots
void AdjustmentsForm::gainSliderChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_GAIN - FEATURE_MIN;
    
    
    if(initialized) {
        initialized = false;
        featureSpinBox[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::gainSliderChanged: Trying to change GAIN failed!");
            updateUI();
        }
    }
}


void AdjustmentsForm::gainSpinBoxChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_GAIN - FEATURE_MIN;
    
    
    if(initialized) {
        initialized = false;
        featureSlider[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::gainSpinBoxChanged: Trying to change GAIN failed!");
            updateUI();
        }
    }    
}


void AdjustmentsForm::gainAutoModeChecked( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_GAIN - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureAutoMode(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::gainAutoMode: Trying to enable Auto-Mode failed!");
            initialized = false;
            featureAutoModeCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::gainOnChecked( bool newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_GAIN - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureOn(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::gainOnChecked: Error while trying to %s the feature",
                        newValue ? "enable" : "disable");
            initialized = false;
            featureOnCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::gainOnePushActivated() {
    int featureIndex = FEATURE_GAIN - FEATURE_MIN;
    
    featureOnePushActivated(featureIndex);
}


// gamma control slots
void AdjustmentsForm::gammaSliderChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_GAMMA - FEATURE_MIN;
    
    
    if(initialized) {
        initialized = false;
        featureSpinBox[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::gammaSliderChanged: Trying to change GAMMA failed!");	updateUI();
        }
    }
}


void AdjustmentsForm::gammaSpinBoxChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_GAMMA - FEATURE_MIN;
    
    
    if(initialized) {
        initialized = false;
        featureSlider[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::gammaSpinBoxChanged: Trying to change GAMMA failed!");
            updateUI();
        }
    }
}


void AdjustmentsForm::gammaAutoModeChecked( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_GAMMA - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureAutoMode(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::gammaAutoMode: Trying to enable Auto-Mode failed!");
            initialized = false;
            featureAutoModeCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::gammaOnChecked( bool newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_GAMMA - FEATURE_MIN;
    
    
    if(gammaOnCanceled) {
        gammaOnCanceled = false;
        return;
    }
    
    if(initialized) {
        if(newValue) {
            // enable Gamma...
            if(lutOnCheckBox->isChecked()) {
                // while LUT is activated can lead to strange behaviour
                if(QMessageBox::warning(this, "Warning",
                                        "Because the LUT is also activated, 'Gamma-Correction' might\noverwrite the LUT-contents, depending on the camera-type!\n\nContinue?",
                                        QMessageBox::Yes | QMessageBox::Default,
                                        QMessageBox::No | QMessageBox::Escape,
                                        QMessageBox::NoButton) == QMessageBox::No) {
                    gammaOnCanceled = true;
                    gammaOnCheckBox->setChecked(false);
                    return;
                }
            }
        }
        else {
            // disable Gamma...
            if(lutOnCheckBox->isChecked()) {
                // while LUT is still activated can lead to strange behaviour
                if(QMessageBox::warning(this, "Warning",
                                        "Depending on the camera-type, deactivating Gamma-Correction, while LUT is activated,\nwill neither activate a former LUT nor will the Gamma-LUT stay active.\nIn this case continuing will activate a 'linear' LUT.\n\nContinue?",
                                        QMessageBox::Yes | QMessageBox::Default,
                                        QMessageBox::No | QMessageBox::Escape,
                                        QMessageBox::NoButton) == QMessageBox::No) {
                    gammaOnCanceled = true;
                    gammaOnCheckBox->setChecked(true);
                    return;
                }
            }
        }
    }
    
    
    if(initialized) {
        if(!dc1394->setFeatureOn(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::gammaOnChecked: Error while trying to %s the feature",
                        newValue ? "enable" : "disable");
            initialized = false;
            featureOnCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::gammaOnePushActivated() {
    int featureIndex = FEATURE_GAMMA - FEATURE_MIN;
    
    featureOnePushActivated(featureIndex);
}


// sharpness control slots
void AdjustmentsForm::sharpnessSliderChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_SHARPNESS - FEATURE_MIN;
    
    
    if(initialized) {
        initialized = false;
        featureSpinBox[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::sharpnessSliderChanged: Trying to change SHARPNESS failed!");
            updateUI();
        }
    }
}


void AdjustmentsForm::sharpnessSpinBoxChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_SHARPNESS - FEATURE_MIN;
    
    
    if(initialized) {
        initialized = false;
        featureSlider[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::sharpnessSpinBoxChanged: Trying to change SHARPNESS failed!");
            updateUI();
        }
    }
}


void AdjustmentsForm::sharpnessAutoModeChecked( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_SHARPNESS - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureAutoMode(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::sharpnessAutoMode: Trying to enable Auto-Mode failed!");
            initialized = false;
            featureAutoModeCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::sharpnessOnChecked( bool newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_SHARPNESS - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureOn(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::sharpnessOnChecked: Error while trying to %s the feature", newValue ? "enable" : "disable");
            initialized = false;
            featureOnCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::sharpnessOnePushActivated() {
    int featureIndex = FEATURE_SHARPNESS - FEATURE_MIN;
    
    featureOnePushActivated(featureIndex);
}


// optical filter control slots
void AdjustmentsForm::optFilterSliderChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_OPTICAL_FILTER - FEATURE_MIN;
    
    
    if(initialized) {
        initialized = false;
        featureSpinBox[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::optFilterSliderChanged: Trying to change value failed!");
            updateUI();
        }
    }
}


void AdjustmentsForm::optFilterSpinBoxChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_OPTICAL_FILTER - FEATURE_MIN;
    
    
    if(initialized) {
        initialized = false;
        featureSlider[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::optFilterSpinBoxChanged: Trying to change value failed!");
            updateUI();
        }
    }
}


void AdjustmentsForm::optFilterAutoModeChecked( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_OPTICAL_FILTER - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureAutoMode(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::optFilterAutoMode: Trying to enable Auto-Mode failed!");
            initialized = false;
            featureAutoModeCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::optFilterOnChecked( bool newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_OPTICAL_FILTER - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureOn(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::optFilterOnChecked: Error while trying to %s the feature", newValue ? "enable" : "disable");
            initialized = false;
            featureOnCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::optFilterOnePushActivated() {
    int featureIndex = FEATURE_OPTICAL_FILTER - FEATURE_MIN;
    
    featureOnePushActivated(featureIndex);
}


// hue control slots
void AdjustmentsForm::hueSliderChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_HUE - FEATURE_MIN;
    
    
    if(initialized) {
        initialized = false;
        featureSpinBox[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::hueSliderChanged: Trying to change value failed!");
            updateUI();
        }
    }
}


void AdjustmentsForm::hueSpinBoxChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_HUE - FEATURE_MIN;
    
        
    if(initialized) {
        initialized = false;
        featureSlider[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::hueSpinBoxChanged: Trying to change value failed!");
            updateUI();
        }
    }
}


void AdjustmentsForm::hueAutoModeChecked( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_HUE - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureAutoMode(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::hueAutoMode: Trying to enable Auto-Mode failed!");
            initialized = false;
            featureAutoModeCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::hueOnChecked( bool newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_HUE - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureOn(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::hueOnChecked: Error while trying to %s the feature", newValue ? "enable" : "disable");
            initialized = false;
            featureOnCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::hueOnePushActivated() {
    int featureIndex = FEATURE_HUE - FEATURE_MIN;
    
    featureOnePushActivated(featureIndex);
}


// saturation control slots
void AdjustmentsForm::saturationSliderChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_SATURATION - FEATURE_MIN;
    
    
    if(initialized) {
        initialized = false;
        featureSpinBox[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::saturationSliderChanged: Trying to change value failed!");
            updateUI();
        }
    }
}


void AdjustmentsForm::saturationSpinBoxChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_SATURATION - FEATURE_MIN;
    
        
    if(initialized) {
        initialized = false;
        featureSlider[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::saturationSpinBoxChanged: Trying to change value failed!");
            updateUI();
        }
    }
}


void AdjustmentsForm::saturationAutoModeChecked( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_SATURATION - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureAutoMode(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::saturationAutoMode: Trying to enable Auto-Mode failed!");
            initialized = false;
            featureAutoModeCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::saturationOnChecked( bool newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_SATURATION - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureOn(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::saturationOnChecked: Error while trying to %s the feature", newValue ? "enable" : "disable");
            initialized = false;
            featureOnCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::saturationOnePushActivated() {
    int featureIndex = FEATURE_SATURATION - FEATURE_MIN;
    
    featureOnePushActivated(featureIndex);
}


// white-balance control slots
void AdjustmentsForm::whiteBalAutoModeChecked( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_WHITE_BALANCE - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureAutoMode(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::whiteBalAutoMode: Trying to enable Auto-Mode failed!");
            initialized = false;
            featureAutoModeCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::whiteBalOnChecked( bool newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_WHITE_BALANCE - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureOn(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::whiteBalOnChecked: Error while trying to %s the feature", newValue ? "enable" : "disable");
            initialized = false;
            featureOnCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::whiteBalOnePushActivated() {
    int featureIndex = FEATURE_WHITE_BALANCE - FEATURE_MIN;
    
    featureOnePushActivated(featureIndex);
}


// iris control slots
void AdjustmentsForm::irisSliderChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_IRIS - FEATURE_MIN;
    
    
    if(initialized) {
        initialized = false;
        featureSpinBox[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::irisSliderChanged: Trying to change value failed!");
            updateUI();
        }
    }
}


void AdjustmentsForm::irisSpinBoxChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_IRIS - FEATURE_MIN;
    
        
    if(initialized) {
        initialized = false;
        featureSlider[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::irisSpinBoxChanged: Trying to change value failed!");
            updateUI();
        }
    }
}


void AdjustmentsForm::irisAutoModeChecked( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_IRIS - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureAutoMode(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::irisAutoModeChecked: Trying to enable Auto-Mode failed!");
            initialized = false;
            featureAutoModeCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::irisOnChecked( bool newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_IRIS - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureOn(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::irisOnChecked: Error while trying to %s the feature", newValue ? "enable" : "disable");
            initialized = false;
            featureOnCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::irisOnePushActivated() {
    int featureIndex = FEATURE_IRIS - FEATURE_MIN;
    
    featureOnePushActivated(featureIndex);
}


// focus control slots
void AdjustmentsForm::focusSliderChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_FOCUS - FEATURE_MIN;
    
    
    if(initialized) {
        initialized = false;
        featureSpinBox[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::focusSliderChanged: Trying to change value failed!");
            updateUI();
        }
    }
}


void AdjustmentsForm::focusSpinBoxChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_FOCUS - FEATURE_MIN;
    
        
    if(initialized) {
        initialized = false;
        featureSlider[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::focusSpinBoxChanged: Trying to change value failed!");
            updateUI();
        }
    }
}


void AdjustmentsForm::focusAutoModeChecked( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_FOCUS - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureAutoMode(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::focusAutoModeChecked: Trying to enable Auto-Mode failed!");
            initialized = false;
            featureAutoModeCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::focusOnChecked( bool newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_FOCUS - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureOn(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::focusOnChecked: Error while trying to %s the feature", newValue ? "enable" : "disable");
            initialized = false;
            featureOnCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::focusOnePushActivated() {
    int featureIndex = FEATURE_FOCUS - FEATURE_MIN;
    
    featureOnePushActivated(featureIndex);
}


// zoom control slots
void AdjustmentsForm::zoomSliderChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_ZOOM - FEATURE_MIN;
    
    
    if(initialized) {
        initialized = false;
        featureSpinBox[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::zoomSliderChanged: Trying to change value failed!");
            updateUI();
        }
    }
}


void AdjustmentsForm::zoomSpinBoxChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_ZOOM - FEATURE_MIN;
    
        
    if(initialized) {
        initialized = false;
        featureSlider[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::zoomSpinBoxChanged: Trying to change value failed!");
            updateUI();
        }
    }
}


void AdjustmentsForm::zoomAutoModeChecked( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_ZOOM - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureAutoMode(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::zoomAutoModeChecked: Trying to enable Auto-Mode failed!");
            initialized = false;
            featureAutoModeCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::zoomOnChecked( bool newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_ZOOM - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureOn(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::zoomOnChecked: Error while trying to %s the feature", newValue ? "enable" : "disable");
            initialized = false;
            featureOnCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::zoomOnePushActivated() {
    int featureIndex = FEATURE_ZOOM - FEATURE_MIN;
    
    featureOnePushActivated(featureIndex);
}


// pan control slots
void AdjustmentsForm::panSliderChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_PAN - FEATURE_MIN;
    
    
    if(initialized) {
        initialized = false;
        featureSpinBox[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::panSliderChanged: Trying to change value failed!");
            updateUI();
        }
    }
}


void AdjustmentsForm::panSpinBoxChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_PAN - FEATURE_MIN;
    
        
    if(initialized) {
        initialized = false;
        featureSlider[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::panSpinBoxChanged: Trying to change value failed!");
            updateUI();
        }
    }
}


void AdjustmentsForm::panAutoModeChecked( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_PAN - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureAutoMode(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::panAutoModeChecked: Trying to enable Auto-Mode failed!");
            initialized = false;
            featureAutoModeCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::panOnChecked( bool newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_PAN - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureOn(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::panOnChecked: Error while trying to %s the feature", newValue ? "enable" : "disable");
            initialized = false;
            featureOnCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::panOnePushActivated() {
    int featureIndex = FEATURE_PAN - FEATURE_MIN;
    
    featureOnePushActivated(featureIndex);
}


// tilt control slots
void AdjustmentsForm::tiltSliderChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_TILT - FEATURE_MIN;
    
    
    if(initialized) {
        initialized = false;
        featureSpinBox[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::tiltSliderChanged: Trying to change value failed!");
            updateUI();
        }
    }
}


void AdjustmentsForm::tiltSpinBoxChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_TILT - FEATURE_MIN;
    
        
    if(initialized) {
        initialized = false;
        featureSlider[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::tiltSpinBoxChanged: Trying to change value failed!");
            updateUI();
        }
    }
}


void AdjustmentsForm::tiltAutoModeChecked( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_TILT - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureAutoMode(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::tiltAutoModeChecked: Trying to enable Auto-Mode failed!");
            initialized = false;
            featureAutoModeCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::tiltOnChecked( bool newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_TILT - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureOn(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::tiltOnChecked: Error while trying to %s the feature", newValue ? "enable" : "disable");
            initialized = false;
            featureOnCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::tiltOnePushActivated() {
    int featureIndex = FEATURE_TILT - FEATURE_MIN;
    
    featureOnePushActivated(featureIndex);
}


// exposure control slots
void AdjustmentsForm::autoExposureSliderChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_EXPOSURE - FEATURE_MIN;
    
    
    if(initialized) {
        initialized = false;
        featureSpinBox[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::autoExposureSliderChanged: Trying to change AUTO-EXPOSURE failed!");
            updateUI();
        }
    }
}


void AdjustmentsForm::autoExposureSpinBoxChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_EXPOSURE - FEATURE_MIN;
    
    
    if(initialized) {
        initialized = false;
        featureSlider[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::autoExposureSpinBoxChanged: Trying to change AUTO-EXPOSURE failed!");
            updateUI();
        }
    }
}


void AdjustmentsForm::autoExposureAutoModeChecked( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_EXPOSURE - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureAutoMode(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::autoExposureAutoMode: Trying to enable Auto-Mode failed!");
            initialized = false;
            featureAutoModeCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::autoExposureOnChecked( bool newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_EXPOSURE - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureOn(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::autoExposureOnChecked: Error while trying to %s the feature", newValue ? "enable" : "disable");
            initialized = false;
            featureOnCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::autoExposureOnePushActivated() {
    int featureIndex = FEATURE_EXPOSURE - FEATURE_MIN;
    
    featureOnePushActivated(featureIndex);
}


// shutter control slots
void AdjustmentsForm::shutterSpeedSliderChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_SHUTTER - FEATURE_MIN;
    QString timebase = timebaseComboBox->currentText();
    QString offset = offsetLineEdit->text();
    
    
    if(initialized) {
        initialized = false;
        featureSpinBox[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::ShutterSliderChanged: Trying to change SHUTTER failed!");
            updateUI();
            return;
        }
        
        shutterFormulaLineEdit->setText("Std. Shutter Value * Timebase + 'Offset'");
        timebase.truncate(timebase.length() - 2);
        offset.truncate(offset.length() - 2);
        expTimeLineEdit->setText(QString::number((newValue *
                                                  timebase.toUInt()) +
                                                 offset.toUInt()) + "us");
    }
}


void AdjustmentsForm::shutterSpeedSpinBoxChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_SHUTTER - FEATURE_MIN;
    QString timebase = timebaseComboBox->currentText();
    QString offset = offsetLineEdit->text();
    
    
    if(initialized) {
        initialized = false;
        featureSlider[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::ShutterSpinBoxChanged: Trying to change SHUTTER failed!");
            updateUI();
            return;
        }
        
        shutterFormulaLineEdit->setText("Std. Shutter Value * Timebase + 'Offset'");
        expTimeLineEdit->setText(QString::number((newValue *
                                                  timebase.toUInt()) +
                                                 offset.toUInt()) + "us");
    }
}


void AdjustmentsForm::shutterSpeedAutoModeChecked( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_SHUTTER - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureAutoMode(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::ShutterAutoMode: Trying to enable Auto-Mode failed!");
            initialized = false;
            featureAutoModeCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::shutterSpeedOnChecked( bool newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_SHUTTER - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureOn(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::ShutterOnChecked: Error while trying to %s the feature",
                        newValue ? "enable" : "disable");
            initialized = false;
            featureOnCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::shutterSpeedOnePushActivated() {
    int featureIndex = FEATURE_SHUTTER - FEATURE_MIN;
    
    featureOnePushActivated(featureIndex);
}


// trigger control slots
void AdjustmentsForm::triggerOnChecked(bool newValue) {
    bool initSave = initialized;
    int featureIndex = FEATURE_TRIGGER - FEATURE_MIN;
    
    /*
    // if SW-Trigger is started (i.e. 'Stop'-Button enabled)
    if(swTriggerStopPushButton->isEnabled()) {
        // first stop procedure
        swTriggerStop();
        sleep(1);
    }
*/
    
    
    if(initialized) {
        if(!dc1394->setFeatureOn(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::TriggerOnChecked: Error while trying to %s the feature",
                        newValue ? "enable" : "disable");
            initialized = false;
            featureOnCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
}


// trigger-delay control slots --- because these Features can not be handled via
// V1.0.0 of 'libdc1394_control', they are implemented via '<Get/Set>CameraControlRegister()'
// (see below at about the place of the implementation of the 'Adv. Trigger-Delay' - it's similar!)
void AdjustmentsForm::triggerDelaySliderChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_TRIGGER_DELAY - FEATURE_MIN;
    
    
    if(initialized) {
        initialized = false;
        featureSpinBox[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::triggerDelaySliderChanged: Trying to change value failed!");
            updateUI();
        }
    }
}


void AdjustmentsForm::triggerDelaySpinBoxChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_TRIGGER_DELAY - FEATURE_MIN;
    
    
    if(initialized) {
        initialized = false;
        featureSpinBox[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::triggerDelaySpinBoxChanged: Trying to change value failed!");
            updateUI();
        }
    }
}


void AdjustmentsForm::triggerDelayAutoModeChecked( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_TRIGGER_DELAY - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureAutoMode(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::triggerDelayAutoMode: Trying to enable Auto-Mode failed!");
            initialized = false;
            featureAutoModeCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::triggerDelayOnChecked( bool newValue ) {
    bool initSave = initialized;
    
    
    if(initialized) {
        if(newValue && !triggerOnCheckBox->isChecked()) {
            switch(QMessageBox::warning(this, "Trigger", "Enabling Std. Trigger-Delay has no further effect,\n because the Trigger is not enabled!", "OK", "Cancel", "Enable Trigger", 0, 1)) {
            case 1:
                // Cancel
                initialized = false;
                triggerDelayOnCheckBox->setChecked(false);
                initialized = initSave;
                return;
            case 2:
                // Enable Trigger
                triggerOnCheckBox->setChecked(true);
                break;
            default:
                // Ok
                break;
            }
        }
        
        if(!dc1394->triggerDelayEnable(id, newValue)) {
            logger->log("AdjustmentsForm::triggerDelayOnChecked: Failed while trying to %s 'Trigger-Delay'",
                        newValue ? "enable" : "disable");
            initialized = false;
            triggerDelayOnCheckBox->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::triggerDelayOnePushActivated() {
    int featureIndex = FEATURE_TRIGGER_DELAY - FEATURE_MIN;
    
    featureOnePushActivated(featureIndex);
}


// temperature control slots
void AdjustmentsForm::temperatureSliderChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_TEMPERATURE - FEATURE_MIN;
    
    
    if(initialized) {
        initialized = false;
        featureSpinBox[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::temperatureSliderChanged: Trying to change value failed!");
            updateUI();
        }
    }
}


void AdjustmentsForm::temperatureSpinBoxChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_TEMPERATURE - FEATURE_MIN;
    
    
    if(initialized) {
        initialized = false;
        featureSpinBox[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::temperatureSpinBoxChanged: Trying to change value failed!");
            updateUI();
        }
    }
}


void AdjustmentsForm::temperatureAutoModeChecked( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_TEMPERATURE - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureAutoMode(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::TemperatureAutoMode: Trying to enable Auto-Mode failed!");
            initialized = false;
            featureAutoModeCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::temperatureOnChecked( bool newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_TEMPERATURE - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureOn(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::temperatureOnChecked: Error while trying to %s the feature", newValue ? "enable" : "disable");
            initialized = false;
            featureOnCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::temperatureOnePushActivated() {
    int featureIndex = FEATURE_TEMPERATURE - FEATURE_MIN;
    
    featureOnePushActivated(featureIndex);
}


// capture size control slots
void AdjustmentsForm::captImageSizeSliderChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_CAPTURE_SIZE - FEATURE_MIN;
    
    
    if(initialized) {
        initialized = false;
        featureSpinBox[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::captSizeSliderChanged: Trying to change value failed!");
            updateUI();
        }
    }
}


void AdjustmentsForm::captImageSizeSpinBoxChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_CAPTURE_SIZE - FEATURE_MIN;
    
    
    if(initialized) {
        initialized = false;
        featureSpinBox[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::captSizeSpinBoxChanged: Trying to change value failed!");
            updateUI();
        }
    }
}


void AdjustmentsForm::captImageSizeAutoModeChecked( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_CAPTURE_SIZE - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureAutoMode(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::captImageSitzeAutoMode: Trying to enable Auto-Mode failed!");
            initialized = false;
            featureAutoModeCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::captImageSizeOnChecked( bool newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_CAPTURE_SIZE - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureOn(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::captImageSizeOnChecked: Error while trying to %s the feature", newValue ? "enable" : "disable");
            initialized = false;
            featureOnCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::captImageSizeOnePushActivated() {
    int featureIndex = FEATURE_CAPTURE_SIZE - FEATURE_MIN;
    
    featureOnePushActivated(featureIndex);
}


// capture image quality control slots
void AdjustmentsForm::captImageQualitySliderChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_CAPTURE_QUALITY - FEATURE_MIN;
    
    
    if(initialized) {
        initialized = false;
        featureSpinBox[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::captImageQualitySliderChanged: Trying to change value failed!");
            updateUI();
        }
    }
}


void AdjustmentsForm::captImageQualitySpinBoxChanged( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_CAPTURE_QUALITY - FEATURE_MIN;
    
    
    if(initialized) {
        initialized = false;
        featureSpinBox[featureIndex]->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setFeatureValue(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::captImageQualitySpinBoxChanged: Trying to change value failed!");
            updateUI();
        }
    }
}


void AdjustmentsForm::captImageQualityAutoModeChecked( int newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_CAPTURE_QUALITY - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureAutoMode(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::captImageQualityAutoMode: Trying to enable Auto-Mode failed!");
            initialized = false;
            featureAutoModeCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::captImageQualityOnChecked( bool newValue ) {
    bool initSave = initialized;
    int featureIndex = FEATURE_CAPTURE_QUALITY - FEATURE_MIN;
    
    
    if(initialized) {
        if(!dc1394->setFeatureOn(id, featureIndex, newValue)) {
            logger->log("AdjustmentsForm::captImageQualityOnChecked: Error while trying to %s the feature", newValue ? "enable" : "disable");
            initialized = false;
            featureOnCheckBox[featureIndex]->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::captImageQualityOnePushActivated() {
    int featureIndex = FEATURE_CAPTURE_QUALITY - FEATURE_MIN;
    
    featureOnePushActivated(featureIndex);
}


// ??????????????
void AdjustmentsForm::initAbsoluteControl() {
    featureSet = dc1394->featureSet(id);
    
    for(int featureIndex = 0; featureIndex < NUM_FEATURES; featureIndex++) {
        // if absolute control is possible
        if(featureSet.feature[featureIndex].absolute_capable == DC1394_TRUE) {
            // enable it if selected
            dc1394->enableAbsoluteControl(id, featureIndex, absolute);
        }
        
        logger->log("Using %s values for feature %s (Range: %s ... %s, Value: %s)",
               (absolute ? "absolute" : "'normal'"),
               DC1394::featureString(featureIndex),
               (absolute ? (QString::number(featureSet.feature[featureIndex].abs_min)).ascii() :
                (QString::number(featureSet.feature[featureIndex].min)).ascii()),
               (absolute ? (QString::number(featureSet.feature[featureIndex].abs_max)).ascii() :
                (QString::number(featureSet.feature[featureIndex].max)).ascii()),
               (absolute ? (QString::number(featureSet.feature[featureIndex].abs_value)).ascii() :
                (QString::number(featureSet.feature[featureIndex].value)).ascii()));
    }
    
    // Trigger-Delay absolute-control needs special attention
    if(!dc1394->enableTriggerDelayAbsControl(id, absolute)) {
        logger->log("%s Std. TriggerDelay Abs.Control failed", absolute ? "Enabling" : "Disabling");
        triggerDelayAbsAvailable = false;
    }
    else {
        logger->log("Std. TriggerDelay Abs.Control %s", absolute ? "enabled" : "disabled");
        triggerDelayAbsAvailable = true;
    }
}
// ??????????????????????????


void AdjustmentsForm::initIO1() {
    in1ModeComboBox->clear();
    
    in1ModeComboBox->insertItem(IO_IN_OFF_STRING); // 0x00
    in1ModeComboBox->insertItem(IO_IN_TRIGGER_STRING); // 0x02
    in1ModeComboBox->insertItem(IO_IN_INC_DEC_STRING); // 0x03
    in1ModeComboBox->insertItem(IO_IN_SPI_STRING); // 0x05
    
    
    out1ModeComboBox->clear();
    
    out1ModeComboBox->insertItem(IO_OUT_OFF_STRING); // 0x00
    out1ModeComboBox->insertItem(IO_OUT_PIN_STATE_STRING); // 0x01
    out1ModeComboBox->insertItem(IO_OUT_INT_ENA_STRING); // 0x02
    out1ModeComboBox->insertItem(IO_OUT_INC_DEC_CMP_STRING); // 0x03
    out1ModeComboBox->insertItem(IO_OUT_SPI_INT_STRING); // 0x04
    out1ModeComboBox->insertItem(IO_OUT_SPI_EXT_STRING); // 0x05
    out1ModeComboBox->insertItem(IO_OUT_FRAME_VALID_STRING); // 0x06
    out1ModeComboBox->insertItem(IO_OUT_BUSY_STRING); // 0x07
    out1ModeComboBox->insertItem(IO_OUT_CORR_IN_STRING); // 0x08
}


void AdjustmentsForm::initIO2() {
    in2ModeComboBox->clear();
    
    in2ModeComboBox->insertItem(IO_IN_OFF_STRING);
    in2ModeComboBox->insertItem(IO_IN_TRIGGER_STRING);
    in2ModeComboBox->insertItem(IO_IN_INC_DEC_STRING);
    in2ModeComboBox->insertItem(IO_IN_SPI_STRING);
    
    
    out2ModeComboBox->clear();
    
    out2ModeComboBox->insertItem(IO_OUT_OFF_STRING);
    out2ModeComboBox->insertItem(IO_OUT_PIN_STATE_STRING);
    out2ModeComboBox->insertItem(IO_OUT_INT_ENA_STRING);
    out2ModeComboBox->insertItem(IO_OUT_INC_DEC_CMP_STRING);
    out2ModeComboBox->insertItem(IO_OUT_SPI_INT_STRING);
    out2ModeComboBox->insertItem(IO_OUT_SPI_EXT_STRING);
    out2ModeComboBox->insertItem(IO_OUT_FRAME_VALID_STRING);
    out2ModeComboBox->insertItem(IO_OUT_BUSY_STRING);
    out2ModeComboBox->insertItem(IO_OUT_CORR_IN_STRING);
}


void AdjustmentsForm::initIO3() {
    in3ModeComboBox->clear();
    
    in3ModeComboBox->insertItem(IO_IN_OFF_STRING);
    in3ModeComboBox->insertItem(IO_IN_TRIGGER_STRING);
    in3ModeComboBox->insertItem(IO_IN_INC_DEC_STRING);
    in3ModeComboBox->insertItem(IO_IN_SPI_STRING);
    
    
    out3ModeComboBox->clear();
    
    out3ModeComboBox->insertItem(IO_OUT_OFF_STRING);
    out3ModeComboBox->insertItem(IO_OUT_PIN_STATE_STRING);
    out3ModeComboBox->insertItem(IO_OUT_INT_ENA_STRING);
    out3ModeComboBox->insertItem(IO_OUT_INC_DEC_CMP_STRING);
    out3ModeComboBox->insertItem(IO_OUT_SPI_INT_STRING);
    out3ModeComboBox->insertItem(IO_OUT_SPI_EXT_STRING);
    out3ModeComboBox->insertItem(IO_OUT_FRAME_VALID_STRING);
    out3ModeComboBox->insertItem(IO_OUT_BUSY_STRING);
    out3ModeComboBox->insertItem(IO_OUT_CORR_IN_STRING);
}


void AdjustmentsForm::initCaptureMode() {
    unsigned int frameCount; //, currentCaptureMode;
    
    
    captureModeComboBox->setEnabled(true);
    captureModeComboBox->clear();
    captureModeComboBox->insertItem(CAPTURE_MODE_FREERUN_STRING);
    captureModeComboBox->insertItem(CAPTURE_MODE_MULTISHOT_STRING);
    captureModeComboBox->insertItem(CAPTURE_MODE_ONESHOT_STRING);
    
    captureModeStartPushButton->setEnabled(false);
    
    captureModeFramesSpinBox->setEnabled(false);
    captureModeFramesSpinBox->setMinValue(0);
    captureModeFramesSpinBox->setMaxValue(65535);
    
    if(!dc1394->getCurrentCaptureMode(id, &currentCaptureMode, &frameCount)) {
        logger->log("AdjustmentsForm::initCaptureMode: Error while trying to read Capture-States!");
        return;
    }
    
    logger->log("AdjustmentsForm::initCaptureMode: Current Capture-Mode: %s%s%s",
           currentCaptureMode == CAPTURE_MODE_FREERUN ? "Freerun" : "",
           currentCaptureMode == CAPTURE_MODE_ONESHOT ? "One Shot" : "",
           currentCaptureMode == CAPTURE_MODE_MULTISHOT ? "Multi Shot" : "");
    
    captureModeFramesSpinBox->setValue(frameCount);
    
    switch(currentCaptureMode) {
    case CAPTURE_MODE_FREERUN:
        captureModeComboBox->setCurrentText(CAPTURE_MODE_FREERUN_STRING);
        break;
        
    case CAPTURE_MODE_ONESHOT:
        captureModeComboBox->setCurrentText(CAPTURE_MODE_ONESHOT_STRING);
        captureModeStartPushButton->setEnabled(true);
        break;
        
    case CAPTURE_MODE_MULTISHOT:
        captureModeComboBox->setCurrentText(CAPTURE_MODE_MULTISHOT_STRING);
        if(frameCount > 0) {
            captureModeStartPushButton->setEnabled(true);
        }
        break;
        
    default:
        captureModeComboBox->setCurrentText(CAPTURE_MODE_FREERUN_STRING);
        captureModeComboBox->setEnabled(false);
        captureModeStartPushButton->setEnabled(false);
        captureModeFramesSpinBox->setValue(0);
        captureModeFramesSpinBox->setEnabled(false);
        break;
    }
}


void AdjustmentsForm::enableUpdateF7() {
    updateF7UI = true;
    updateUI();
    this->setEnabled(true);
}


void AdjustmentsForm::updateUI() {
    logger->log("AdjustmentsForm::updateUI: updating the UI...");
    
    if(applyPushButton->isEnabled() || discardPushButton->isEnabled()) {
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
            // ???
            logger->log("AdjustmentsForm::updateUI: This message should never appear!!!");
            return;
            break;
        }
    }
    
    // re-read camera-information
    featureSet = dc1394->featureSet(id);
    
    // set the flags accordingly
    
    // white-balance-flags are simple variables
    initWhiteBalUSlider = 1;
    initWhiteBalUSpinBox = 1;
    initWhiteBalVSlider = 1;
    initWhiteBalVSpinBox = 1;
    
    initHDRKnee1Slider = 0;
    initHDRKnee1SpinBox = 0;
    initHDRKnee2Slider = 0;
    initHDRKnee2SpinBox = 0;
    initHDRKnee3Slider = 0;
    initHDRKnee3SpinBox = 0;
    
    initExtShutterSlider = 0;
    initExtShutterSpinBox = 0;
    
    initTriggerDelaySlider = 0;
    initTriggerDelaySpinBox = 0;
    
    initAdvTriggerDelaySlider = 0;
    initAdvTriggerDelaySpinBox = 0;
    
    
    // the flags for the remaining features are stored inside 'feature-indexed' arrays
    for(int featureIndex = 0; featureIndex < NUM_FEATURES; featureIndex++) {
        // set all init-flags, because the UI is updated not changed
        featureChanged[featureIndex] = false;
        initSlider[featureIndex] = 1;
        initSpinBox[featureIndex] = 1;
        initAutoMode[featureIndex] = true;
        initOn[featureIndex] = true;
        
        // at last - update the UI (with the values, just set, taken directly from the camera)
        if(dc1394->featureReadOutCapable(id, featureIndex) &&
           (featureSet.feature[featureIndex]).available) {
            
            setFeatureUIState(featureIndex, featureSet.feature[featureIndex]);
        }
        else {
            disableFeatureUIControls(featureIndex);
        }
    }
    
    // Format 7
    logger->log("\tNow updating F7...");
    if(updateF7UI)
        updateF7();
    
    // adv. Features
    triggerDelayChanged = false;
    advTriggerDelayChanged = false;
    shadingCorrectionChanged = false;
    lutChanged = false;
    hdrChanged = false;
    
    
    // Exp. Time
    logger->log("\tUpdating 'Exposure Time'...");
    updateExposureTime();
    
    logger->log("\tNow updating Advanced Features...");
    updateAdvFeatures();
}


void AdjustmentsForm::updateExposureTime() {
    unsigned int timebase, offset;
    
    
    if(exposureTimeGroupBox->isEnabled()) {
        //    if(featureGroup[FEATURE_SHUTTER - FEATURE_MIN]->isEnabled()) {
        if(!dc1394->getTimebase(id, &timebase)) {
            timebaseComboBox->setEnabled(false);
            timebaseComboBox->setCurrentItem(0);
            offsetLineEdit->setText("");
            shutterFormulaLineEdit->setText("only Std. Shutter Value available");
            expTimeLineEdit->setText(QString::number(shutterSpeedSpinBox->value()) +
                                     "us");
        }
        else {
            timebaseComboBox->setEnabled(true);
            
            logger->log("AdjustmentsForm::updateExposureTime: Current Timebase-ID is: %d",
                        timebase);
            
            timebaseComboBox->setCurrentText(QString::number(timebase) + "us");
            
            dc1394->getShutterOffset(id, &offset);
            offsetLineEdit->setText(QString::number(offset) + "us");
            
            // ext. Shutter
            if(dc1394->extShutterAvailable(id)) {
                extShutterRangeCheckBox->setEnabled(true);
                
                if(extShutterRangeCheckBox->isChecked()) {
                    extShutterSlider->setEnabled(true);
                    extShutterSpinBox->setEnabled(true);
                    shutterFormulaLineEdit->setText("Ext. Shutter Register Value + Offset");
                    updateExtShutter();
                    expTimeLineEdit->setText(
                            QString::number(extShutterSpinBox->value() + offset) + "us");
                }
                else {
                    extShutterSlider->setEnabled(false);
                    if(extShutterSpinBox->isEnabled() &&
                       extShutterSpinBox->hasFocus()) {
                        focusdata->next()->setFocus();
                    }
                    extShutterSpinBox->setEnabled(false);
                    shutterFormulaLineEdit->setText("Std. Shutter Value * Timebase + 'Offset'");
                    updateExtShutter();
                    expTimeLineEdit->setText(
                            QString::number((shutterSpeedSpinBox->value() * timebase) +
                                            offset) + "us");
                }
            }
            else {
                extShutterRangeCheckBox->setEnabled(false);
                extShutterSlider->setEnabled(false);
                if(extShutterSpinBox->isEnabled() && extShutterSpinBox->hasFocus()) {
                    focusdata->next()->setFocus();
                }
                extShutterSpinBox->setEnabled(false);
                shutterFormulaLineEdit->setText(
                        "Std. Shutter Value * Timebase + 'Offset'");
                expTimeLineEdit->setText(
                        QString::number((shutterSpeedSpinBox->value() * timebase) +
                                        offset) + "us");
            }
            
        }
    }
}


void AdjustmentsForm::updateAdvFeatures() {
    bool dsnu, blemish;
    
    if(dc1394->triggerDelayAvailable(id)) {
        triggerDelayGroupBox->setEnabled(true);
//        triggerDelayGroupBox->setEnabled(false);
        updateTriggerDelay();
    }
    else {
        logger->log("AdjustmentsForm::updateAdvFeatures: Std. Trigger Delay not available!");
        triggerDelayGroupBox->setEnabled(false);
    }
    
    // Adv. TriggerDelay
    if(dc1394->advTriggerDelayAvailable(id)) {
        advTriggerDelayGroupBox->setEnabled(true);
        updateAdvTriggerDelay();
    }
    else {
        logger->log("AdjustmentsForm::updateAdvFeatures: Adv. Trigger Delay not available!");
        advTriggerDelayGroupBox->setEnabled(false);
    }
    
    // ShadingCorrection
    if(dc1394->shadingCorrectionAvailable(id)) {
        shadingCorrectionGroupBox->setEnabled(true);
        updateShadingCorrection();
    }
    else {
        logger->log("AdjustmentsForm::updateAdvFeatures: Shading Corrections not available!");
        shadingCorrectionGroupBox->setEnabled(false);
    }
    
    // LUT
    if(dc1394->lutAvailable(id)) {
        lutGroupBox->setEnabled(true);
        updateLUT();
    }
    else {
        logger->log("AdjustmentsForm::updateAdvFeatures: LUT not available!");
        lutGroupBox->setEnabled(false);
    }
    
    // HDR
    if(dc1394->hdrAvailable(id)) {
        hdrGroupBox->setEnabled(true);
        updateHDR();
    }
    else {
        logger->log("AdjustmentsForm::updateAdvFeatures: HDR not available!");
        hdrGroupBox->setEnabled(false);
    }
    
    // MirrorImage
    if(dc1394->mirrorImageAvailable(id)) {
        mirrorImageGroupBox->setEnabled(true);
        updateMirrorImage();
    }
    else {
        logger->log("AdjustmentsForm::updateAdvFeatures: Mirror Image not available!");
        mirrorImageGroupBox->setEnabled(false);
    }
    
    // Test Image
    if(dc1394->testImageAvailable(id)) {
        testImageGroupBox->setEnabled(true);
        updateTestImage();
    }
    else {
        logger->log("AdjustmentsForm::updateAdvFeatures: Test Images not available!");
        testImageGroupBox->setEnabled(false);
    }
    
    
    // Color Correction
    if(dc1394->colorCorrAvailable(id)) {
        colorCorrGroupBox->setEnabled(true);
        updateColorCorr();
    }
    else {
        logger->log("AdjustmentsForm::updateAdvFeatures: Color Correction not available!");
        colorCorrGroupBox->setEnabled(false);
    }
    
    // I/O 1
    if(dc1394->ioAvailable(id, 1)) {
        gpIO1GroupBox->setEnabled(true);
        updateIO(1, true);
    }
    else {
        logger->log("AdjustmentsForm::updateAdvFeatures: IO1 not available!");
        gpIO1GroupBox->setEnabled(false);
    }
    
    // I/O 2
    if(dc1394->ioAvailable(id, 2)) {
        gpIO2GroupBox->setEnabled(true);
        updateIO(2, true);
    }
    else {
        logger->log("AdjustmentsForm::updateAdvFeatures: IO2 not available!");
        gpIO2GroupBox->setEnabled(false);
    }
    
    // I/O 3
    if(dc1394->ioAvailable(id, 3)) {
        gpIO3GroupBox->setEnabled(true);
        updateIO(3, true);
    }
    else {
        logger->log("AdjustmentsForm::updateAdvFeatures: IO3 not available!");
        gpIO3GroupBox->setEnabled(false);
    }
    
    // serial Interface
    if(dc1394->serialAvailable(id)) {
        serialIOGroupBox->setEnabled(true);
        updateSerialIO();
    }
    else {
        logger->log("AdjustmentsForm::updateAdvFeatures: Serial I/O not available!");
        serialIOGroupBox->setEnabled(false);
    }
    
    // DSNU/Blemish
    if(dsnu = dc1394->dsnuAvailable(id)) {
        logger->log("AdjustmentsForm::updateAdvFeatures: DSNU available");
        dsnuOnCheckBox->setEnabled(true);
//        dsnuGrabCountSpinBox->setEnabled(true);
    }
    else {
        dsnuOnCheckBox->setChecked(false);
        dsnuOnCheckBox->setEnabled(false);
//        dsnuShowImageCheckBox->setChecked(false);
//        dsnuShowImageCheckBox->setEnabled(false);
//        dsnuGrabCountSpinBox->setEnabled(false);
    }
    
    if(blemish = dc1394->blemishAvailable(id)) {
        logger->log("AdjustmentsForm::updateAdvFeatures: Blemish available");
        blemishOnCheckBox->setEnabled(true);
//        blemishGrabCountSpinBox->setEnabled(true);
    }
    else {
        blemishOnCheckBox->setChecked(false);
        blemishOnCheckBox->setEnabled(false);
//        blemishShowImageCheckBox->setChecked(false);
//        blemishShowImageCheckBox->setEnabled(false);
//        blemishGrabCountSpinBox->setEnabled(false);
    }
    
    if(dsnu || blemish) {
//        if(!initialized)
//            dsnuUsedImageLineEdit->setText("");
        
        dsnuGroupBox->setEnabled(true);
        dsnuShowImageCheckBox->setEnabled(true);
        dsnuGrabCountSpinBox->setEnabled(true);
        
        updateDSNU();
    }
    else {
        logger->log("AdjustmentsForm::updateAdvFeatures: DSNU and Blemish are not available!");
        dsnuUsedImageLineEdit->setText("");
        dsnuGroupBox->setEnabled(false);
        dsnuShowImageCheckBox->setEnabled(false);
        dsnuGrabCountSpinBox->setEnabled(false);
    }
    
    // AutoShutter
    if(dc1394->autoShutterCtlAvailable(id)) {
        autoShutterGroupBox->setEnabled(true);
        updateAutoShutter();
    }
    else {
        logger->log("AdjustmentsForm::updateAdvFeatures: AutoShutter control not available");
        autoShutterGroupBox->setEnabled(false);
    }
    
    //AutoGain
    if(dc1394->autoGainCtlAvailable(id)) {
        autoGainGroupBox->setEnabled(true);
        updateAutoGain();
    }
    else {
        logger->log("AdjustmentsForm::updateAdvFeatures: AutoGain control not available");
        autoGainGroupBox->setEnabled(false);
    }
    
    // AutoAOI
    if(dc1394->autoAOIAvailable(id)) {
        autoAOIGroupBox->setEnabled(true);
        updateAutoAOI();
    }
    else {
        logger->log("AdjustmentsForm::updateAdvFeatures: AutoAOI not available");
        autoAOIGroupBox->setEnabled(false);
    }
    
    // Deferred Image Transport
    if(dc1394->deferredImgAvailable(id)) {
        deferredImgGroupBox->setEnabled(true);
        updateDeferredImg();
    }
    else {
        logger->log("AdjustmentsForm::updateAdvFeatures: Deferred Image Transport not available");
        deferredImgGroupBox->setEnabled(false);
    }
    
    // FrameInfo
    if(dc1394->frameInfoAvailable(id)) {
        frameInfoGroupBox->setEnabled(true);
        updateFrameInfo();
    }
    else {
        logger->log("AdjustmentsForm::updateAdvFeatures: FrameInfo not available");
        frameInfoGroupBox->setEnabled(false);
    }
    
    
    // Delayed Integration
    if(dc1394->delIntAvailable(id)) {
        delIntGroupBox->setEnabled(true);
        updateDelInt();
    }
    else {
        logger->log("AdjustmentsForm::updateAdvFeatures: Delayed Integration not availabe");
        delIntGroupBox->setEnabled(false);
    }
    
    // Incremental Decoder
    if(dc1394->incDecAvailable(id)) {
        incDecGroupBox->setEnabled(true);
        updateIncDec();
    }
    else {
        logger->log("AdjustmentsForm::updateAdvFeatures: Incremental Decoder not available");
        incDecGroupBox->setEnabled(false);
    }
    
} // updateAdvFeatures()


void AdjustmentsForm::updateF7() {
    if(!dc1394->f7Available(id)) {
        logger->log("AdjustmentsForm::updateF7: F7 currently not available - disabling the UI-Controls");
        disableF7();
        return;
    }
    
    updateF7AOI();
    updateF7FrameInfoGroupBox();
}


void AdjustmentsForm::disableF7() {
    logger->log("AdjustmentsForm::disableF7: F7 is not available");
    f7AOIGroupBox->setEnabled(false);
    f7FrameInfoGroupBox->setEnabled(false);
    f7ImagePixelsLineEdit->setText("");
    f7PaddingLineEdit->setText("");
    f7TotalBytesLineEdit->setText("");
}


void AdjustmentsForm::updateF7AOI() {
    unsigned int maxX, maxY, w, h, x, y;
    unsigned int maxXSize, maxYSize;
    unsigned int hUnit, vUnit, hPosUnit, vPosUnit;
    bool initSave;
    F7NotifyData notifyData;
    F7UpdateNotifyEvent *notifyEvent;
    
    
    if(f7Changed) {
        // UI-controls have been changed - applying changes to camera
        logger->log("AdjustmentsForm::updateF7AOI: F7 UI controls have changed - therefore updating camera...");
        
        notifyData.width = f7AOIWidthSpinBox->value();
        notifyData.height = f7AOIHeightSpinBox->value();
        notifyData.x = f7AOIXSpinBox->value();
        notifyData.y = f7AOIYSpinBox->value();
        notifyData.sender = QString("AdjustmentsForm");
        
        notifyEvent = new F7UpdateNotifyEvent(notifyData);
        
        QApplication::postEvent(parent, notifyEvent);
        
        QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
        sleep(3);
        QApplication::restoreOverrideCursor();
    }
    else {
        // refreshing UI-controls from camera-settings
        logger->log("AdjustmentsForm::updateF7AOI: Updating F7 UI controls from camera...");
        
        f7AOIGroupBox->setEnabled(true);
        
        if(!dc1394->getF7MaxSize(id, 0, &maxX, &maxY)) {
            logger->log("AdjustmentsForm::updateF7AOI: Error! Reading maximum image size failed!");
            return;
        }
        
        if(!dc1394->getF7ImageSize(id, &w, &h)) {
            logger->log("AdjustmentsForm::updateF7AOI: Error! Reading current image size failed!");
            return;
        }
        
        if(!dc1394->getF7ImagePos(id, &x, &y)) {
            logger->log("AdjustmentsForm::updateF7AOI: Error! Reading current image position failed!");
            return;
        }
        
        if(!dc1394->getF7Units(id, &hUnit, &vUnit, &hPosUnit, &vPosUnit)) {
            logger->log("AdjustmentsForm::updateF7AOI: Error! Reading current unit values failed!");
            return;
        }
        
        logger->log("AdjustmentsForm::updateF7AOI: x=%d (max-x=%d), y=%d (max-y=%d), width=%d, height=%d",
               x, maxX, y, maxY, w, h);
        logger->log("\tH-Unit: %d, V-Unit: %d, H-Pos-Unit: %d, V-Pos-Unit: %d",
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
        f7CorrectWidthSliderValue = (unsigned int)f7AOIWidthSlider->value() != w;
        f7AOIWidthSlider->setValue(w);
        f7CorrectWidthSpinBoxValue = (unsigned int)f7AOIWidthSpinBox->value() != w;
        f7AOIWidthSpinBox->setValue(w);
        f7CorrectHeightSliderValue = (unsigned int)f7AOIHeightSlider->value() != h;
        f7AOIHeightSlider->setValue(h);
        f7CorrectHeightSpinBoxValue = (unsigned int)f7AOIHeightSpinBox->value() != h;
        f7AOIHeightSpinBox->setValue(h);
        
        f7CorrectXSliderValue = (unsigned int)f7AOIXSlider->value() != x;
        f7AOIXSlider->setValue(x);
        f7CorrectXSpinBoxValue = (unsigned int)f7AOIXSpinBox->value() != x;
        f7AOIXSpinBox->setValue(x);
        f7CorrectYSliderValue = (unsigned int)f7AOIYSlider->value() != y;
        f7AOIYSlider->setValue(y);
        f7CorrectYSpinBoxValue = (unsigned int)f7AOIYSpinBox->value() != y;
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


void AdjustmentsForm::updateF7FrameInfoGroupBox() {
    unsigned int pixelNo;
    unsigned long long int totalBytes;
    unsigned int ppf, minBPP, maxBPP, bpp, recBPP, fps;
    bool initSave = initialized;
    
    
    initialized = false;
    
    f7FrameInfoGroupBox->setEnabled(true);
    
    if(!dc1394->getF7PixelNumber(id, &pixelNo)) {
        logger->log("AdjustmentsForm::updateF7FrameInfoGroupBox: Reading current number of pixels failed!");
        f7ImagePixelsLineEdit->setText("");
    }
    else {
        f7ImagePixelsLineEdit->setText(QString::number(pixelNo));
    }

    if(!dc1394->getF7TotalBytes(id, &totalBytes)) {
        logger->log("AdjustmentsForm::updateF7FrameInfoGroupBox: Reading 'total bytes' failed!");
        f7TotalBytesLineEdit->setText("");
        totalBytes = 0;
    }
    else {
        f7TotalBytesLineEdit->setText(QString::number(totalBytes));
    }
    
    if(!dc1394->getF7PacketPerFrame(id, &ppf)) {
        logger->log("AdjustmentsForm::updateF7FrameInfoGroupBox: Reading current 'packet-per-frame' value failed!");
        f7PPFLineEdit->setText("");
        ppf = 0;
    }
    else {
        f7PPFLineEdit->setText(QString::number(ppf));
    }
    
    if(!dc1394->getF7PacketParameters(id, &minBPP, &maxBPP)) {
        logger->log("AdjustmentsForm::updateF7FrameInfoGroupBox: Reading 'packet-parameters' failed!");
        minBPP = 0;
        maxBPP = 0;
        f7BPPSpinBox->setMinValue(0);
        f7BPPSpinBox->setMaxValue(0);
    }
    else {
        f7BPPSpinBox->setMinValue(minBPP);
        f7BPPSpinBox->setLineStep(minBPP > 0 ? minBPP : 1);
        f7BPPSpinBox->setMaxValue(maxBPP);
    }
    
    if(!dc1394->getF7RecBytePerPacket(id, &recBPP)) {
        logger->log("AdjustmentsForm::updateF7FrameInfoGroupBox: Reading current 'byte-per-packet' value failed!");
        recBPP = 0;
        f7RecBPPLineEdit->setText("");
    }
    else {
        f7RecBPPLineEdit->setText(QString::number(recBPP));
    }
    
    if(!dc1394->getF7BytePerPacket(id, &bpp)) {
        logger->log("AdjustmentsForm::updateF7FrameInfoGroupBox: Reading current 'byte-per-packet' value failed!");
        bpp = 0;
        f7BPPSpinBox->setValue(0);
    }
    else {
        f7BPPSpinBox->setValue(bpp);
    }
    
    if((totalBytes == 0) || (ppf == 0) || (bpp == 0)) {
        logger->log("AdjustmentsForm::updateF7FrameInfoGroupBox: Determination of current 'padding' failed!");
        f7PaddingLineEdit->setText("");
        f7FPSLineEdit->setText("");
    }
    else {
        f7PaddingLineEdit->setText(QString::number((ppf * bpp) - totalBytes) + " Bytes");
        fps = (((bpp > 4096) ? 4096 : bpp) * (unsigned int)pow(10, 6)) / (totalBytes * 125);
        f7FPSLineEdit->setText(QString::number(fps));
    }
    
    initialized = initSave;
}


void AdjustmentsForm::discardF7() {
    f7Changed = false;
    updateF7();
    
    applyPushButton->setEnabled(false);
    discardPushButton->setEnabled(false);
}


void AdjustmentsForm::updateF7BPP() {
    F7BPPChangedEvent *bppEvent = new F7BPPChangedEvent();
    
    if(!dc1394->setF7BytePerPacket(id, f7BPPSpinBox->value())) {
        logger->log("AdjustmentsForm::updateF7BPP: Error while trying to change 'byte per packet' value!");
    }
    
    QApplication::postEvent(parent, bppEvent);
}

void AdjustmentsForm::updateExtShutter() {
    unsigned int value;
    
    if(extShutterChanged) {
        if(!dc1394->setExtShutter(id, extShutterSpinBox->value())) {
            logger->log("AdjustmentsForm::updateExtShutter: Error while trying to change Ext. Shutter Time to %dus",
                   extShutterSpinBox->value());
            return;
        }
    }
    else {
        if(!dc1394->getExtShutter(id, &value)) {
            logger->log("AdjustmentsForm::updateExtShutter: Error while trying to read Ext. Shutter value");
            extShutterRangeCheckBox->setChecked(false);
            extShutterRangeCheckBox->setEnabled(false);
            extShutterSlider->setValue(0);
            extShutterSlider->setEnabled(false);
            extShutterSpinBox->setValue(0);
            extShutterSpinBox->setSuffix("");
            if(extShutterSpinBox->isEnabled() && extShutterSpinBox->hasFocus()) {
                focusdata->next()->setFocus();
            }
            extShutterSpinBox->setEnabled(false);
            return;
        }
        
        logger->log("AdjustmentsForm::updateExtShutter: Ext. Shutter Value: %dus (%dus...%dus)",
                    value, extShutterSpinBox->minValue(), (int)(pow(2, 26) - 1));
        
/*
        if(extShutterRangeCheckBox->isChecked()) {
            extShutterSlider->setEnabled(true);
            extShutterSpinBox->setEnabled(true);
        }
        else {
            extShutterSlider->setEnabled(false);
            extShutterSpinBox->setEnabled(false);
        }
*/
        
        // values will always kept up to date
        extShutterSlider->setMaxValue((int)(pow(2,26) - 1));
        extShutterSpinBox->setMaxValue((int)(pow(2, 26) - 1));
        
        initExtShutterSlider = 1;
        initExtShutterSpinBox = 1;
        
        extShutterSlider->setValue(value);
        extShutterSpinBox->setValue(value);
        extShutterSpinBox->setSuffix("us");
    }
}



void AdjustmentsForm::discardExtShutter() {
    unsigned int value;
    
    
    if(!dc1394->getExtShutter(id, &value)) {
        logger->log("AdjustmentsForm::discardExtShutter: Error while trying to read Ext. Shutter value");
        extShutterRangeCheckBox->setChecked(false);
        extShutterRangeCheckBox->setEnabled(false);
        extShutterSlider->setValue(0);
        extShutterSlider->setEnabled(false);
        extShutterSpinBox->setValue(0);
        extShutterSpinBox->setSuffix("");
        if(extShutterSpinBox->isEnabled() && extShutterSpinBox->hasFocus()) {
            focusdata->next()->setFocus();
        }
        extShutterSpinBox->setEnabled(false);
        return;
    }
    
    logger->log("AdjustmentsForm::discardExtShutter: Ext. Shutter Value reset to: %d us(%dus...%dus)", value,
           extShutterSpinBox->minValue(), (int)(pow(2, 26) - 1));
    
    // values will always kept up to date
    extShutterSlider->setMaxValue((int)(pow(2,26) - 1));
    extShutterSpinBox->setMaxValue((int)(pow(2, 26) - 1));
    
    initExtShutterSlider = 1;
    initExtShutterSpinBox = 1;
    
    extShutterSlider->setValue(value);
    extShutterSpinBox->setValue(value);
    extShutterSpinBox->setSuffix("us");
}


void AdjustmentsForm::updateTimebase() {
    CamWindow *camWindow;
    QString timebaseString = timebaseComboBox->currentText();
    unsigned int timebase;
    
    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
    
    if(QString(parent->className()) != "CamWindow") {
        logger->log("AdjustmentsForm::updateTimebase: Parent of 'AdjustmentsForm' is '%s', but 'CamWindow' is expected!",
               parent->className());
        logger->log("\tTherefore a change of the 'Timebase' is not possible!");
        QApplication::restoreOverrideCursor();
        return;
    }
    else {
        camWindow  = (CamWindow *)parent;
    }
    
    if(!featureSlider[FEATURE_SHUTTER - FEATURE_MIN]->isEnabled()) {
        logger->log("AdjustmentsForm::updateTimebase: Std.Shutter not available!");
        QApplication::restoreOverrideCursor();
        return;
    }
    
    if(!camWindow->stopCapture()) {
        logger->log("AdjustmentsForm::updateTimebase: Error while trying to stop capturing!");
        QApplication::restoreOverrideCursor();
        return;
    }
    
    timebaseString.replace("us", "");
    timebase = timebaseString.toUInt();
    
    logger->log("AdjustmentsForm::updateTimebase: Trying to set new Timebase to %d us (Selection: %s)",
           timebase, timebaseString.ascii());
    
    if(!dc1394->setTimebase(id, timebase)) {
        logger->log("AdjustmentsForm::updateTimebase: Error while trying to change timebase!");
        
        if(!dc1394->getTimebase(id, &timebase)) {
            timebaseComboBox->setEnabled(false);
            timebaseComboBox->setCurrentItem(0);
            offsetLineEdit->setText("");
            shutterFormulaLineEdit->setText("only Std. Shutter Value available");
            expTimeLineEdit->setText(QString::number(shutterSpeedSpinBox->value()) +
                                     "us");
        }
        else {
            timebaseComboBox->setEnabled(true);
            
            logger->log("AdjustmentsForm::updateTimebase: Current Timebase-ID is: %d", timebase);
            
            timebaseComboBox->setCurrentText(QString::number(timebase) + "us");
        }
    }
    else {
        logger->log("AdjustmentsForm::updateTimebase: Now trying to 're-set' Shutter Value for activating the new timebase");
        
        if(!dc1394->setFeatureValue(id, FEATURE_SHUTTER - FEATURE_MIN,
                                    (featureSet.feature[FEATURE_SHUTTER - FEATURE_MIN]).value))
            logger->log("AdjustmentsForm::updateTimebase: Error while 're-setting' new Shutter Value!");
    }
    
    camWindow->startCapture();
    QApplication::restoreOverrideCursor();
}


void AdjustmentsForm::discardTimebase() {
    unsigned int timebase;
    
    if(!dc1394->getTimebase(id, &timebase)) {
        timebaseComboBox->setEnabled(false);
        timebaseComboBox->setCurrentItem(0);
        offsetLineEdit->setText("");
        shutterFormulaLineEdit->setText("only Std. Shutter Value available");
        expTimeLineEdit->setText(QString::number(shutterSpeedSpinBox->value()) +
                                 "us");
        return;
    }
    
    timebaseComboBox->setCurrentText(QString::number(timebase) + "us");
}


void AdjustmentsForm::updateTriggerDelay() {
    bool initSave = initialized;
    unsigned int triggerDelayInfo;
//    unsigned int triggerDelayValue;
//    bool triggerDelayOn;
    bool enabled;
    unsigned int delay;
    
    
    // Get TriggerDelay informations
    if(!dc1394->getTriggerDelayInfo(id, &triggerDelayInfo)) {
        logger->log("AdjustmentsForm::updateTriggerDelay: reading 'TriggerDelay-Info' failed!");
        triggerDelayGroupBox->setEnabled(false);
        return;
    }
    logger->log("AdjustmentsForm::updateTriggerDelay: TriggerDelayInfo: 0x%x", triggerDelayInfo);
    
    // TriggerDelay currently available?
    if((triggerDelayInfo & TRIGGER_DLY_PRESENT_MASK) == 0) {
        logger->log("AdjustmentsForm::updateTriggerDelay: Std. TriggerDelay not available!");
        triggerDelayGroupBox->setEnabled(false);
        return;
    }
    
    // On/Off 'switchable' ?
    if((triggerDelayInfo & TRIGGER_DLY_ONOFF_MASK) == 0) {
        logger->log("AdjustmentsForm::updateTriggerDelay: TriggerDelay-On/Off not available!");
        triggerDelayOnCheckBox->setEnabled(false);
    }
    else {
        triggerDelayOnCheckBox->setEnabled(true);
    }
    
    // OnePush available ?
    if((triggerDelayInfo & TRIGGER_DLY_ONEPUSH_MASK) == 0) {
        logger->log("AdjustmentsForm::updateTriggerDelay: TriggerDelay-OnePush not available");
        triggerDelayOnePushPushButton->setEnabled(false);
    }
    else {
        triggerDelayOnePushPushButton->setEnabled(true);
    }
    
    // AutoMode available ?
    if((triggerDelayInfo & TRIGGER_DLY_AUTO_MODE_MASK) == 0) {
        logger->log("AdjustmentsForm::updateTriggerDelay: TriggerDelay-AutoMode not available");
        triggerDelayAutoModeCheckBox->setEnabled(false);
    }
    else {
        triggerDelayAutoModeCheckBox->setEnabled(true);
    }
    
    // Value 'readout' capable ?
    if((triggerDelayInfo & TRIGGER_DLY_READOUT_MASK) == 0) {
        logger->log("AdjustmentsForm::updateTriggerDelay: Std. TriggerDelay Value not available!");
        triggerDelaySlider->setEnabled(false);
        triggerDelaySpinBox->setEnabled(false);
    }
    else {
        logger->log("AdjustmentsForm::updateTriggerDelay: TriggerDelay-Min: %d, TriggerDelay-Max: %d",
               (int)((triggerDelayInfo >> 12) & TRIGGER_DLY_MINMAX_MASK),
               (int)(triggerDelayInfo & TRIGGER_DLY_MINMAX_MASK));
        
        // set Min-Values
        triggerDelaySlider->setMinValue((triggerDelayInfo >> 12) &
                                        TRIGGER_DLY_MINMAX_MASK);
        triggerDelaySpinBox->setMinValue((triggerDelayInfo >> 12) &
                                         TRIGGER_DLY_MINMAX_MASK);
        
        // set Max-Values
        triggerDelaySlider->setMaxValue(triggerDelayInfo & TRIGGER_DLY_MINMAX_MASK);
        triggerDelaySpinBox->setMaxValue(triggerDelayInfo & TRIGGER_DLY_MINMAX_MASK);
        
        // set Suffix (depending on absolute control)
        triggerDelaySpinBox->setSuffix(triggerDelayAbsAvailable ? (absolute ? "s" : "") : "");
        
/*
        if(!dc1394->getTriggerDelay(id, &triggerDelayValue)) {
            logger->log("reading current Std. Trigger Delay Value failed!");
            triggerDelaySlider->setEnabled(false);
            triggerDelaySpinBox->setEnabled(false);
        }
        else {
            if(!dc1394->triggerDelayEnabled(id, &triggerDelayOn)) {
                logger->log("reading Std. Trigger Delay State failed!");
                triggerDelaySlider->setEnabled(false);
                triggerDelaySpinBox->setEnabled(false);
            }
            else {
                triggerDelaySlider->setEnabled(triggerDelayOn);
                triggerDelaySlider->setValue(triggerDelayValue);
                triggerDelaySpinBox->setEnabled(triggerDelayOn);
                triggerDelaySpinBox->setValue(triggerDelayValue);
            }
        }
*/
    }
    
    if(triggerDelayChanged) {
        // UI has changed - write UI status to camera
        if(!dc1394->triggerDelayEnable(id, triggerDelayOnCheckBox->isChecked())) {
            logger->log("AdjustmentsForm::updateTriggerDelay: Error while trying to %s std. TriggerDelay",
                   triggerDelayOnCheckBox->isChecked() ? "enable" : "disable");
            return;
        }
        
        triggerDelaySlider->setEnabled(triggerDelayOnCheckBox->isChecked());
        if(!triggerDelayOnCheckBox->isChecked() &&
           triggerDelaySpinBox->isEnabled() &&
           triggerDelaySpinBox->hasFocus()) {
            
            focusdata->next()->setFocus();
        }
        triggerDelaySpinBox->setEnabled(triggerDelayOnCheckBox->isChecked());
        
        if(triggerDelayOnCheckBox->isChecked()) {
            if(!dc1394->setTriggerDelay(id, triggerDelaySpinBox->value())) {
                logger->log("AdjustmentsForm::updateTriggerDelay: Error while trying to change the std. trigger delay");
                return;
            }
        }
    }
    else {
        // update UI from camera
        if(!dc1394->triggerDelayEnabled(id, &enabled)) {
            logger->log("AdjustmentsForm::updateTriggerDelay: Error while trying to get the current Std. Trigger Delay status");
            return;
        }
        triggerDelayOnCheckBox->setChecked(enabled);
        
        if(!dc1394->getTriggerDelay(id, &delay)) {
            logger->log("AdjustmentsForm::updateTriggerDelay: Error while trying to read the current Std. Trigger Delay time");
            return;
        }
        
        initialized = false;
        triggerDelaySlider->setValue(delay);
        triggerDelaySpinBox->setValue(delay);
        initialized = initSave;
        
        triggerDelaySlider->setEnabled(enabled);
        triggerDelaySpinBox->setEnabled(enabled);
    }
}


void AdjustmentsForm::updateAdvTriggerDelay() {
    bool initSave = initialized;
    bool enabled;
    unsigned int delay;
    
    
    if(advTriggerDelayChanged) {
        // UI has changed - write UI status to camera
        if(!dc1394->advTriggerDelayEnable(id, advTriggerDelayOnCheckBox->isChecked())) {
            logger->log("AdjustmentsForm::advTriggerDelay: Error while trying to %s adv. TriggerDelay",
                   advTriggerDelayOnCheckBox->isChecked() ? "enable" : "disable");
            return;
        }
        
        advTriggerDelaySlider->setEnabled(advTriggerDelayOnCheckBox->isChecked());
        if(!advTriggerDelayOnCheckBox->isChecked() &&
           advTriggerDelaySpinBox->isEnabled() &&
           advTriggerDelaySpinBox->hasFocus()) {
            focusdata->next()->setFocus();
        }
        advTriggerDelaySpinBox->setEnabled(advTriggerDelayOnCheckBox->isChecked());
        
        if(advTriggerDelayOnCheckBox->isChecked()) {
            if(!dc1394->setAdvTriggerDelay(id, advTriggerDelaySpinBox->value())) {
                logger->log("AdjustmentsForm::advTriggerDelay: Error while trying to change the adv. trigger delay");
                return;
            }
        }
    }
    else {
        // get UI status from camera
        if(!dc1394->advTriggerDelayEnabled(id, &enabled)) {
            logger->log("AdjustmentsForm::advTriggerDelay: Error while trying to get the current Adv. Trigger Delay status");
            return;
        }
        advTriggerDelayOnCheckBox->setChecked(enabled);
        
        if(!dc1394->getAdvTriggerDelay(id, &delay)) {
            logger->log("AdjustmentsForm::advTriggerDelay: Error while trying to read the current Adv. Trigger Delay time");
            return;
        }
        
        if(!initialized) {
            advTriggerDelaySlider->setMinValue(0);
            advTriggerDelaySlider->setMaxValue((int)pow(2, 21) - 1);
            
            advTriggerDelaySpinBox->setMinValue(0);
            advTriggerDelaySpinBox->setMaxValue((int)pow(2, 21) - 1);
            advTriggerDelaySpinBox->setSuffix("us");
        }
        
        initialized = false;
        advTriggerDelaySlider->setValue(delay);
        advTriggerDelaySpinBox->setValue(delay);
        initialized = initSave;
        
        advTriggerDelaySlider->setEnabled(enabled);
        advTriggerDelaySpinBox->setEnabled(enabled);
    }
}


void AdjustmentsForm::discardTriggerDelay() {
    bool initSave = initialized;
    bool state;
    unsigned int delay;
    
    
    if(!dc1394->triggerDelayEnabled(id, &state)) {
        logger->log("AdjustmentsForm::discardTriggerDelay: Error while trying to read Std. TriggerDelay State");
        return;
    }
    
    triggerDelayOnCheckBox->setChecked(state);
    
    if(!dc1394->getTriggerDelay(id, &delay)) {
        logger->log("AdjustmentsForm::discardTriggerDelay: Errror while trying to read Adv. TriggerDelay value");
        return;
    }
    
    initialized = false;
    triggerDelaySlider->setValue(delay);
    triggerDelaySpinBox->setValue(delay);
    triggerDelaySpinBox->setSuffix(triggerDelayAbsAvailable ? (absolute ? "s" : "") : "");
    initialized = initSave;
}


void AdjustmentsForm::discardAdvTriggerDelay() {
    bool initSave = initialized;
    bool state;
    unsigned int delay;
    
    
    if(!dc1394->advTriggerDelayEnabled(id, &state)) {
        logger->log("AdjustmentsForm::discardAdvTriggerDelay: Error while trying to read Adv. TriggerDelay State");
        return;
    }
    
    advTriggerDelayOnCheckBox->setChecked(state);
    
    if(!dc1394->getAdvTriggerDelay(id, &delay)) {
        logger->log("AdjustmentsForm::discardAdvTriggerDelay: Errror while trying to read Adv. TriggerDelay value");
        return;
    }
    
    initialized = false;
    advTriggerDelaySlider->setValue(delay);
    advTriggerDelaySpinBox->setValue(delay);
    advTriggerDelaySpinBox->setSuffix("us");
    initialized = initSave;
}


// shading correction
void AdjustmentsForm::updateShadingCorrection() {
    bool showImage, initSave;
    unsigned int maxSize;
    bool on;
    
    
    logger->log("AdjustmentsForm::updateShadingCorrection: Updating Shading Correction...");
    
    // read current values from camera
    if(!dc1394->getShadingCorrectionInfos(id, &showImage, &grabCount, &maxSize, &on)) {
        logger->log("AdjustmentsForm::updateShadingCorrection: Error while trying to read Shading Correction parameters...");
        shadingCorrectionGroupBox->setEnabled(false);
        return;
    }
    
    logger->log("AdjustmentsForm::updateShadingCorrection: ShadingCorrection Parameters:\n\tshowImage is %s\n\tgrabCount = %d\n\tmaxSize = %d bytes\n\tFeature is switched %s",
           (showImage ? "on" : "off"), grabCount, maxSize, (on ? "on" : "off"));
    
    initSave = initialized;
    initialized = false;
    
    // The Load Image and Read Image Button is always available
    shadingCorrectionLoadImgPushButton->setEnabled(true);
    shadingCorrectionReadImgPushButton->setEnabled(true);
    
    // maximum Image Size will always be updated
    shadingCorrectionMaxImgSizeLineEdit->setText(QString::number(maxSize));
    maxShadingImgSize = maxSize;
    
    // GrabCount limits
    shadingCorrectionGrabCountSpinBox->setMinValue(0);
    shadingCorrectionGrabCountSpinBox->setMaxValue(255);
    
    if(shadingCorrectionChanged) {
        // update camera from UI
        if(!dc1394->shadingCorrectionEnable(id, shadingCorrectionOnCheckBox->isChecked())) {
            logger->log("AdjustmentsForm::updateShadingCorrection: Error while trying to %s Shading Correction!",
                   shadingCorrectionOnCheckBox->isChecked() ? "enable" : "disable");
            shadingCorrectionOnCheckBox->toggle();
            shadingCorrectionGroupBox->setEnabled(false);
            initialized = initSave;
            return;
        }
        
        if(!dc1394->shadingCorrectionShowImage(id,
                                               shadingCorrectionShowImageCheckBox->isChecked())) {
            logger->log("AdjustmentsForm::updateShadingCorrection: Error while trying to %s shading image!",
                   shadingCorrectionShowImageCheckBox->isChecked() ? "show" : "hide");
            shadingCorrectionShowImageCheckBox->toggle();
            initialized = initSave;
            return;
        }
        
        if(!dc1394->shadingCorrectionSetGrabCount(id,
                                                  shadingCorrectionGrabCountSpinBox->value())) {
            logger->log("AdjustmentsForm::updateShadingCorrection: Error while trying to set new GrabCount value (%d)",
                   shadingCorrectionGrabCountSpinBox->value());
        }
    }
    else {
        // update UI from camera
        shadingCorrectionOnCheckBox->setChecked(on);
        
        shadingCorrectionShowImageCheckBox->setEnabled(
                shadingCorrectionOnCheckBox->isChecked());
        shadingCorrectionShowImageCheckBox->setChecked(showImage);
        
        shadingCorrectionBuildImgPushButton->setEnabled(grabCount > 0);
        
        shadingCorrectionGrabCountSpinBox->setValue(grabCount);
    }
    
    initialized = initSave;
}


void AdjustmentsForm::discardShadingCorrection() {
    bool initSave = initialized;
    
    
    initialized = false;
    shadingCorrectionChanged = false;
    
    updateShadingCorrection();
    initialized = initSave;
}


void AdjustmentsForm::updateLUT() {
    unsigned int maxLutNo, lutNo;
    bool on;
    
    
    // get current LUT state from camera
    if(!dc1394->getLUTInfos(id, &maxLutNo, &maxLUTSize, &on)) {
        logger->log("AdjustmentsForm::updateLUT: Error while trying to get the status of the LUT feature");
        lutGroupBox->setEnabled(false);
        return;
    }
    
    logger->log("AdjustmentsForm::updateLUT: Updating LUT-controls...");
    
    // max. LUT size is updated independen on the direction of data-transfer
    lutMaxSizeLineEdit->setText(QString::number(maxLUTSize));
    
    if(lutChanged) {
        // update camera from UI
        logger->log("AdjustmentsForm::updateLUT: Updating camera-LUT-state from UI...");
        
        // update the state of the cmera
        if(!dc1394->lutEnable(id, lutOnCheckBox->isChecked()))
            logger->log("AdjustmentsForm::updateLUT: Error while trying to %s LUT",
                        on ? "enable" : "disable");
        
        // selecting LUT
        if(!dc1394->selectLUT(id, lutNumberSpinBox->value() - 1)) {
            logger->log("AdjustmentsForm::updateLUT: Error while trying to select LUT %d",
                        lutNumberSpinBox->value() - 1);
        }
    }
    else {
        // update UI from camera
        
        // OnCheckBox
        lutOnCheckBox->setChecked(on);
        
        // LUT-Selection
        if(!dc1394->getLUTSelection(id, &lutNo)) {
            logger->log("AdjustmentsForm::updateLUT: Error while trying to get current LUT selection");
            lutNumberSpinBox->setEnabled(false);
        }
        else {
            lutNumberSpinBox->setEnabled(true);
            lutNumberSpinBox->setMinValue(1);
            lutNumberSpinBox->setMaxValue(maxLutNo);
            lutNumberSpinBox->setValue(lutNo + 1);
        }
    }
}


void AdjustmentsForm::discardLUT() {
    bool initSave = initialized;
    
    
    initialized = false;
    lutChanged = false;
    
    updateLUT();
    initialized = initSave;
}


void AdjustmentsForm::updateHDR() {
    bool initSave = initialized;
    unsigned int maxKnees, knee1, knee2, knee3, activeKnees;
    bool on;
    bool valid;
    bool changeValues = false;
    
    
    // change cursor for the whole updating process
    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
    
    // update the maximum values
    // (HDR depends on shutter-speed / EXTD_SHUTTER = Max. HDR-Value)
    if(!dc1394->getExposureTime(id, &expTime, &valid)) {
        logger->log("AdjustmentsForm::updateHDR: Exposure Time not available (?) - disabling HDR feature");
        hdrGroupBox->setEnabled(false);
        QApplication::restoreOverrideCursor();
        return;
    }
    
    logger->log("AdjustmentsForm::updateHDR: Exposure Time = %d us", expTime);
    hdrKnee1Slider->setMaxValue(expTime);
    hdrKnee1SpinBox->setMaxValue(expTime);
    hdrKnee2Slider->setMaxValue(expTime);
    hdrKnee2SpinBox->setMaxValue(expTime);
    hdrKnee3Slider->setMaxValue(expTime);
    hdrKnee3SpinBox->setMaxValue(expTime);
    
    if(!dc1394->getHDRInfos(id, &maxKnees, &knee1, &knee2, &knee3, &activeKnees, &on)) {
        logger->log("AdjustmentsForm::updateHDR: Error while trying to get the status of the HDR feature");
        QApplication::restoreOverrideCursor();
        return;
    }
    
    logger->log("AdjustmentsForm::updateHDR: Updating HDR-controls...");
    logger->log("AdjustmentsForm::updateHDR: HDR-Values:\n\tmaxKnees: %d\n\tknee1: %d\n\tknee2: %d\n\tknee3: %d\n\tactive knees: %d",
                maxKnees, knee1, knee2, knee3, activeKnees);
    
    if(knee1 == 0) {
        knee1 = 5;
        changeValues = true;
    }
    if(knee2 == 0) {
        knee2 = 5;
        changeValues = true;
    }
    if(knee3 == 0) {
        knee3 = 5;
        changeValues = true;
    }
    if(changeValues) {
        if(!dc1394->hdrSetValues(id, activeKnees, knee1, knee2, knee3)) {
            logger->log("AdjustmentsForm::updateHDR: Error while trying to correct knee-values from '0' to '5'!");
            QApplication::restoreOverrideCursor();
            return;
        }
    }
    
    hdrNumKneesSpinBox->setMaxValue(maxKnees);
    
    // if HDR-On UI-control has changed
    if(hdrChanged && (hdrOnCheckBox->isChecked() != on)) {
        logger->log("AdjustmentsForm::updateHDR: On-UI-control has changed to '%s'",
                    hdrOnCheckBox->isChecked() ? "on" : "off");
        
        // change camera HDR-On state accordingly
        if(!updateHDROn(&maxKnees, &knee1, &knee2, &knee3, &activeKnees, &on))
            return;
        
        // if HDR-state 'ON/OFF' has changed no other changes will be made
        // (if switched off - all other controls are no longer valid,
        //  if switched on - all other controls will be read from the camera)
        hdrChanged = false;
    }
    else {
        // set UI-control from camera-state (initialization)
        if(hdrOnCheckBox->isChecked() != on) {
            initialized = false;
            hdrOnCheckBox->setChecked(on);
            initialized = initSave;
        }
        
        if(!initialized) {
            hdrNumKneesSpinBox->setValue((activeKnees == 0) ? 1 : activeKnees);
            hdrChanged = false;
        }
    }
    
    // if HDR is (or has just been) switched on
    if(on) {
        logger->log("AdjustmentsForm::updateHDR: HDR is switched on");
        initialized = false;
        hdrOnCheckBox->setChecked(on);
        initialized = initSave;
        
        enableHDRControls();
        
        // if UI controls have changed - store new values in camera
        if(hdrChanged) {
            if(!saveHDRToCamera())
                return;
            
            numberOfKnees = hdrNumKneesSpinBox->value();
        }
        else { // UI not changed
            setHDRUIValues(maxKnees, knee1, knee2, knee3);
        }
        
        if(!initialized)
            numberOfKnees = hdrNumKneesSpinBox->value();
        
        // enable Buttons
        enableHDRButtons(true);
    } // if(on)
    else { // HDR switched off
        logger->log("AdjustmentsForm::updateHDR: HDR is switched off - therefore disabling all controls");
        // disable Sliders and SpinBoxes
        if(hdrNumKneesSpinBox->isEnabled() && hdrNumKneesSpinBox->hasFocus()) {
            focusdata->next()->setFocus();
        }
        hdrNumKneesSpinBox->setEnabled(false);
        hdrKnee1Slider->setEnabled(false);
        if(hdrKnee1SpinBox->isEnabled() && hdrKnee1SpinBox->hasFocus()) {
            focusdata->next()->setFocus();
        }
        hdrKnee1SpinBox->setEnabled(false);
        hdrKnee2Slider->setEnabled(false);
        if(hdrKnee2SpinBox->isEnabled() && hdrKnee2SpinBox->hasFocus()) {
            focusdata->next()->setFocus();
        }
        hdrKnee2SpinBox->setEnabled(false);
        hdrKnee3Slider->setEnabled(false);
        if(hdrKnee3SpinBox->isEnabled() && hdrKnee3SpinBox->hasFocus()) {
            focusdata->next()->setFocus();
        }
        hdrKnee3SpinBox->setEnabled(false);
        
        // disableButtons
        enableHDRButtons(false);
        
        //     if((!initialized) && (!initHDRValues(knee2, knee3)))
        //  return;
        if(!initialized)
            setHDRUIValues(maxKnees, knee1, knee2, knee3);
        
    }
    
    QApplication::restoreOverrideCursor();
} // updateHDR()


bool AdjustmentsForm::updateHDROn(unsigned int *maxKnees, unsigned int *knee1,
                                  unsigned int *knee2, unsigned int *knee3,
                                  unsigned int *activeKnees, bool *on) {
    bool initSave = initialized;
    bool changed = false;
    bool error = false;
    
    // check if knees-dependencies are OK ('knee 1' > 'knee2' > 'knee3'!)
    if(*knee2 <= *knee3) {
        logger->log("AdjustmentsForm::updateHDROn: Adjusting knee2 (%d) to %d", *knee2, *knee3 + 1);
        *knee2 = *knee3 +1;
        changed = true;
    }
    
    if(*knee1 <= *knee2) {
        logger->log("AdjustmentsForm::updateHDROn: Adjusting knee1 (%d) to %d", *knee1, *knee2 + 1);
        *knee1 = *knee2 +1;
        changed = true;
    }
    
    if(*activeKnees == 0) {
        logger->log("AdjustmentsForm::updateHDROn: Changing 'activeKnee' to 1!");
        *activeKnees = 1;
        changed = true;
    }
    
    if(changed) {
        logger->log("AdjustmentsForm::updateHDROn: Now writing %d new Values (%d, %d, %d) to camera", *activeKnees,
               *knee1, *knee2, *knee3);
        
        if(!dc1394->hdrSetValues(id, *activeKnees, *knee1, *knee2, *knee3)) {
            logger->log("AdjustmentsForm::updateHDROn: Unable to correct kneepoint values!");
            return false;
        }
    }
    
    // change camera HDR-On state to match HDR-On UI-state
    if(dc1394->hdrEnable(id, hdrOnCheckBox->isChecked())) {
        // and wait for state to be changed
        logger->log("AdjustmentsForm::updateHDROn: Waiting for HDR to be switched %s",
               hdrOnCheckBox->isChecked() ? "on" : "off");
        do {
            sleep(5);
            logger->log("\tstill waiting ...");
            if(!dc1394->getHDRInfos(id, maxKnees, knee1, knee2, knee3,
                                    activeKnees, on)) {
                error = true;
                break;
            }
        } while((*on) != hdrOnCheckBox->isChecked());
        
        if(error) {
            logger->log("AdjustmentsForm::updateHDROn: Error while trying to read the changed HDR state!");
            QApplication::restoreOverrideCursor();
            return false;
        }
        
        if(on)
            logger->log("AdjustmentsForm::updateHDROn: Current HDR-values:\n\tmaxKnees: %d\n\tknee1: %d\n\tknee2: %d\n\tknee3: %d",
                        *maxKnees, *knee1, *knee2, *knee3);
        
        // set Spin Box to the max. nunmber of kneepoints to enable 'em all
        initialized = false;
        hdrNumKneesSpinBox->setValue((*activeKnees == 0) ? 1 : *activeKnees);
        initialized = initSave;
    }
    else {
        logger->log("AdjustmentsForm::updateHDROn: Error while trying to %s HDR control",
               (hdrOnCheckBox->isChecked()) ? "enable" : "disable");
        QApplication::restoreOverrideCursor();
        return false;
    }
    
    return true;
    
}


void AdjustmentsForm::enableHDRControls() {
    hdrNumKneesSpinBox->setEnabled(true);
    
    
    switch(hdrNumKneesSpinBox->value()) {
            case 1:
                hdrKnee1Slider->setEnabled(true);
                hdrKnee1SpinBox->setEnabled(true);
                hdrKnee2Slider->setEnabled(false);
                if(hdrKnee2SpinBox->isEnabled() && hdrKnee2SpinBox->hasFocus()) {
                    focusdata->next()->setFocus();
                }
                hdrKnee2SpinBox->setEnabled(false);
                hdrKnee3Slider->setEnabled(false);
                if(hdrKnee3SpinBox->isEnabled() && hdrKnee3SpinBox->hasFocus()) {
                    focusdata->next()->setFocus();
                }
                hdrKnee3SpinBox->setEnabled(false);
                break;
            case 2:
                hdrKnee1Slider->setEnabled(true);
                hdrKnee1SpinBox->setEnabled(true);
                hdrKnee2Slider->setEnabled(true);
                hdrKnee2SpinBox->setEnabled(true);
                hdrKnee3Slider->setEnabled(false);
                if(hdrKnee3SpinBox->isEnabled() && hdrKnee3SpinBox->hasFocus()) {
                    focusdata->next()->setFocus();
                }
                hdrKnee3SpinBox->setEnabled(false);
                break;
            case 3:
                hdrKnee1Slider->setEnabled(true);
                hdrKnee1SpinBox->setEnabled(true);
                hdrKnee2Slider->setEnabled(true);
                hdrKnee2SpinBox->setEnabled(true);
                hdrKnee3Slider->setEnabled(true);
                hdrKnee3SpinBox->setEnabled(true);
                break;
            default:
                logger->log("AdjustmentsForm::enableHDRControls: This message should never appear!");
                break;
            }
    
}


bool AdjustmentsForm::saveHDRToCamera() {
    logger->log("AdjustmentsForm::saveHDRToCamera: Now writing %d new Kneepoint values to camera...",
           hdrNumKneesSpinBox->value());
    
    switch(hdrNumKneesSpinBox->value()) {
                case 1:
                    if(!dc1394->hdrSetValues(id, 1, hdrKnee1SpinBox->value(), 0, 0)) {
                        logger->log("AdjustmentsForm::saveHDRToCamera: Error while trying to change the Kneepoints of the camera");
                        QApplication::restoreOverrideCursor();
                        return false;
                    }
                    sleep(5);
                    
                    break;
                case 2:
                    if(!dc1394->hdrSetValues(id, 2, hdrKnee1SpinBox->value(),
                                             hdrKnee2SpinBox->value(), 0)) {
                        logger->log("AdjustmentsForm::saveHDRToCamera: Error while trying to change the Kneepoints of the camera");
                        QApplication::restoreOverrideCursor();
                        return false;
                    }
                    sleep(5);
                    
                    break;
                case 3:
                    if(!dc1394->hdrSetValues(id, 3, hdrKnee1SpinBox->value(),
                                             hdrKnee2SpinBox->value(),
                                             hdrKnee3SpinBox->value())) {
                        logger->log("AdjustmentsForm::saveHDRToCamera: Error while trying to change the Kneepoints of the camera");
                        QApplication::restoreOverrideCursor();
                        return false;
                    }
                    sleep(5);
                    
                    break;
                default:
                    logger->log("???");
                    break;
                }
    
    return true;
}


void AdjustmentsForm::setHDRUIValues(unsigned int maxKnees, unsigned int knee1, unsigned int knee2, unsigned int knee3) {
    bool initSave = initialized;
    
    
    initialized = false;
    if((knee1 < (unsigned int)hdrKnee1SpinBox->value()) ||
       (knee2 < (unsigned int)hdrKnee2SpinBox->value()) ||
       (knee3 < (unsigned int)hdrKnee3SpinBox->value())) {
        
        hdrKnee3Slider->setValue(knee3);
        hdrKnee3SpinBox->setValue(knee3);
        hdrKnee2Slider->setValue(knee2);
        hdrKnee2SpinBox->setValue(knee2);
        hdrKnee1Slider->setValue(knee1);
        hdrKnee1SpinBox->setValue(knee1);
    }
    else {
        hdrKnee1Slider->setValue(knee1);
        hdrKnee1SpinBox->setValue(knee1);
        hdrKnee2Slider->setValue(knee2);
        hdrKnee2SpinBox->setValue(knee2);
        hdrKnee3Slider->setValue(knee3);
        hdrKnee3SpinBox->setValue(knee3);
    }
    initialized = initSave;
    
    return;
    
    logger->log("AdjustmentsForm::setHDRUIValues: Now setting new Kneepoint UI-values (knee1: %d, knee2: %d, knee3: %d)",
                knee1, knee2, knee3);
    initialized = false;
    switch(maxKnees) {
                case 1:
                    initHDRKnee1Slider = 1;
                    initHDRKnee1SpinBox = 1;
                    hdrKnee1Slider->setValue(knee1);
                    hdrKnee1SpinBox->setValue(knee1);
                    break;
                case 2:
                    initHDRKnee1Slider = 1;
                    initHDRKnee1SpinBox = 1;
                    hdrKnee1Slider->setValue(knee1);
                    hdrKnee1SpinBox->setValue(knee1);
                    initHDRKnee2Slider = 1;
                    initHDRKnee2SpinBox = 1;
                    hdrKnee2Slider->setValue(knee2);
                    hdrKnee2SpinBox->setValue(knee2);
                    break;
                case 3:
                    initHDRKnee1Slider = 1;
                    initHDRKnee1SpinBox = 1;
                    hdrKnee1Slider->setValue(knee1);
                    hdrKnee1SpinBox->setValue(knee1);
                    initHDRKnee2Slider = 1;
                    initHDRKnee2SpinBox = 1;
                    hdrKnee2Slider->setValue(knee2);
                    hdrKnee2SpinBox->setValue(knee2);
                    initHDRKnee3Slider = 1;
                    initHDRKnee3SpinBox = 1;
                    hdrKnee3Slider->setValue(knee3);
                    hdrKnee3SpinBox->setValue(knee3);
                    break;
                default:
                    break;
                }
    initialized = initSave;
}


void AdjustmentsForm::enableHDRButtons(bool enable) {
    hdrPreset1PushButton->setEnabled(enable);
    hdrPreset2PushButton->setEnabled(enable);
    hdrPreset3PushButton->setEnabled(enable);
    hdrResetPushButton->setEnabled(enable);
}


//bool AdjustmentsForm::initHDRValues(unsigned int knee1, unsigned int knee2) {
bool AdjustmentsForm::initHDR() {
    unsigned int maxKnees, knee1, knee2, knee3, activeKnees;
    unsigned int maxKneesR, knee1R, knee2R, knee3R, activeKneesR;
    bool on, onR, changeValues = false, initSave = initialized;
    
    
    if(!dc1394->getHDRInfos(id, &maxKnees, &knee1, &knee2, &knee3, &activeKnees,
                            &on)) {
        logger->log("AdjustmentsForm::initHDR: Error while trying to read current HDR-values!");
        return false;
    }
    
    hdrNumKneesSpinBox->setMinValue(1);
    hdrNumKneesSpinBox->setMaxValue(maxKnees);
    
    // if HDR is switched on, its values should be valid...
    if(!on) {
        if((activeKnees <= 0) || (activeKnees > maxKnees)) {
            activeKnees = 1;
            changeValues = true;
        }
        
        // check initial HDR-values for consistency
        if(knee1 <= 0) {
            knee1 = 2;
            changeValues = true;
        }
        
        if(knee2 <= 0) {
            knee2 = 1;
            changeValues = true;
        }
        
        if(knee3 > 0xfffd) {
            knee3 = 0xfffd;
            changeValues = true;
        }
        
        if(knee2 > 0xfffe) {
            knee2 = 0xfffe;
            changeValues = true;
        }
        
        if(knee2 < 1) {
            knee2 = 1;
            changeValues = true;
        }
        
        if(knee1 < 2) {
            knee1 = 2;
            changeValues = true;
        }
        
        // if there are values to correct
        if(changeValues) {
            // apply these to camera
            if(!dc1394->hdrSetValues(id, activeKnees, knee1, knee2, knee3)) {
                logger->log("AdjustmentsForm::initHDR: Error while trying to correct HDR values!");
                return false;
            }
            // and re-read the values
            if(!dc1394->getHDRInfos(id, &maxKneesR, &knee1R, &knee2R, &knee3R,
                                    &activeKneesR, &onR)) {
                logger->log("AdjustmentsForm::initHDR: Failed to 're'-read HDR-values after correction");
            }
            // check the new values
            if(maxKneesR != maxKnees) {
                logger->log("AdjustmentsForm::initHDR: Max. number of HDR-kneepoints has changed after value-correction!");
                return false;
            }
            if(knee1R != knee1) {
                logger->log("AdjustmentsForm::initHDR: HDR-kneepoint 1 has not been corrected!");
                return false;
            }
            if(knee2R != knee2) {
                logger->log("AdjustmentsForm::initHDR: HDR-kneepoint 2 has not been corrected!");
                return false;
            }
            if(knee3R != knee3) {
                logger->log("AdjustmentsForm::initHDR: HDR-kneepoint 3 has not been corrected!");
                return false;
            }
            if(activeKneesR != activeKnees) {
                logger->log("AdjustmentsForm::initHDR: Max. number of active HDR-kneepoints has changed after value-correction!");
                return false;
            }
            if(onR != on) {
                logger->log("AdjustmentsForm::initHDR: HDR has been switched %s after value-correction!",
                            onR ? "on" : "off");
                return false;
            }
        }
    }
    
    initialized = false;
    hdrNumKneesSpinBox->setValue(activeKnees);
    hdrKnee1Slider->setValue(knee1);
    hdrKnee1SpinBox->setValue(knee1);
    hdrKnee2Slider->setValue(knee2);
    hdrKnee2SpinBox->setValue(knee2);
    hdrKnee3Slider->setValue(knee3);
    hdrKnee3SpinBox->setValue(knee3);
    initialized = initSave;
    
    switch(activeKnees) {
    case 1:
        hdrKnee1Slider->setEnabled(true);
        hdrKnee1SpinBox->setEnabled(true);
        hdrKnee2Slider->setEnabled(false);
        hdrKnee2SpinBox->setEnabled(false);
        hdrKnee3Slider->setEnabled(false);
        hdrKnee3SpinBox->setEnabled(false);
        break;
        
    case 2:
        hdrKnee1Slider->setEnabled(true);
        hdrKnee1SpinBox->setEnabled(true);
        hdrKnee2Slider->setEnabled(true);
        hdrKnee2SpinBox->setEnabled(true);
        hdrKnee3Slider->setEnabled(false);
        hdrKnee3SpinBox->setEnabled(false);
        break;
        
    case 3:
        hdrKnee1Slider->setEnabled(true);
        hdrKnee1SpinBox->setEnabled(true);
        hdrKnee2Slider->setEnabled(true);
        hdrKnee2SpinBox->setEnabled(true);
        hdrKnee3Slider->setEnabled(true);
        hdrKnee3SpinBox->setEnabled(true);
        break;
        
    default:
        hdrKnee1Slider->setEnabled(false);
        hdrKnee1SpinBox->setEnabled(false);
        hdrKnee2Slider->setEnabled(false);
        hdrKnee2SpinBox->setEnabled(false);
        hdrKnee3Slider->setEnabled(false);
        hdrKnee3SpinBox->setEnabled(false);
        break;
    }
    
    return true;
}


void AdjustmentsForm::discardHDR() {
    unsigned int maxKnees, knee1, knee2, knee3, activeKnees;
    bool on;
    
    if(!dc1394->getHDRInfos(id, &maxKnees, &knee1, &knee2, &knee3, &activeKnees, &on)) {
        logger->log("AdjustmentsForm::discardHDR: Error while reading the current state of the HDR-Feature");
    }
    
    hdrOnCheckBox->setChecked(on);
    
    initHDRKnee1Slider = 1;
    initHDRKnee1SpinBox = 1;
    hdrKnee1Slider->setValue(knee1);
    //   hdrKnee1SpinBox->setValue(knee1);
    initHDRKnee2Slider = 1;
    initHDRKnee2SpinBox = 1;
    hdrKnee2Slider->setValue(knee2);
    //    hdrKnee2SpinBox->setValue(knee2);
    initHDRKnee3Slider = 1;
    initHDRKnee3SpinBox = 1;
    hdrKnee3Slider->setValue(knee3);
    //    hdrKnee3SpinBox->setValue(knee3);
    
    hdrNumKneesSpinBox->setValue((activeKnees == 0) ? 1 : activeKnees);
}


void AdjustmentsForm::updateMirrorImage() {
    bool state;
    
    if(!dc1394->getMirrorImageState(id, &state)) {
        logger->log("AdjustmentsForm::updateMirrorImage: Error while trying to get the current 'Mirror Image' state!");
        return;
    }
    
    mirrorImageOnCheckBox->setChecked(state);
}


void AdjustmentsForm::updateTestImage() {
    unsigned int images, currentImage;
    
    if(!dc1394->getAvailableTestImages(id, &images)) {
        logger->log("AdjustmentsForm::updateTestImage: Error while trying to get available test images!");
        return;
    }
    
    if(images == 0) {
        logger->log("AdjustmentsForm::updateTestImage: No Test Images available!");
        testImageOnCheckBox->setChecked(false);
        testImageOnCheckBox->setEnabled(false);
        testImageComboBox->setEnabled(false);
        return;
    }
    
    logger->log("AdjustmentsForm::updateTestImage: Available test-images: 0x%x", images);
    
    if(testImageComboBox->count() == 0) {
        for(unsigned int i = 0; i < 7; i++) {
            if(((images >> i) & 1)  != 0) {
                logger->log("AdjustmentsForm::updateTestImage: Test Image %d available", i + 1);
                testImageComboBox->insertItem("Test Image " + QString::number(i + 1));
            }
        }
        
    }
    
    if(!dc1394->getTestImage(id, &currentImage)) {
        logger->log("AdjustmentsForm::updateTestImage: Error while trying to read currently selected test image");
        return;
    }
    
    logger->log("AdjustmentsForm::updateTestImage: currentImage (camera-value): %d",
                currentImage);
    
    if((currentImage) == 0) {
        testImageOnCheckBox->setChecked(false);
    }
    else {
        testImageComboBox->setCurrentText("Test Image " + QString::number(currentImage));
        testImageOnCheckBox->setChecked(true);
    }
}


void AdjustmentsForm::updateColorCorr() {
    bool state;
    
    if(!dc1394->getColorCorrState(id, &state)) {
        logger->log("AdjustmentsForm::updateColorCorr: Error while trying to get the current state of 'Color corection'!");
        return;
    }
    
    colorCorrOnCheckBox->setChecked(state);
}


void AdjustmentsForm::updateIO(unsigned int ioNo, bool all) {
    bool inPolarity, inState, outPolarity, outState;
    unsigned int inMode, outMode;
    
    
    logger->log("AdjustmentsForm::updateIO...");
    
    // read current 'In'-state
    if(!dc1394->getIOInCtrl(id, ioNo, &inPolarity, &inMode, &inState)) {
        logger->log("AdjustmentsForm::updateIO: Errror while trying to get the current state of I/O-Input %d!",
                    ioNo);
        gpIO1GroupBox->setEnabled(false);
        return;
    }
    logger->log("AdjustmentsForm::updateIO: %d: In-Polarity = %s, In-Mode = 0x%x, In-State = %s",
           ioNo,
           inPolarity ? "true" : "false",
           inMode,
           inState ? "true" : "false");
    
    // read current 'Out'-state
    if(!dc1394->getIOOutCtrl(id, ioNo, &outPolarity, &outMode, &outState)) {
        logger->log("AdjustmentsForm::updateIO: Errror while trying to get the current state of I/O-Output %d!",
                    ioNo);
        gpIO1GroupBox->setEnabled(false);
        return;
    }
    logger->log("AdjustmentsForm::updateIO: %d: Out-Polarity = %s, Out-Mode = 0x%x, Out-Pin-State = %s",
           ioNo,
           outPolarity ? "high act." : "low act.",
           outMode,
           outState ? "true" : "false");
    
    // update UI-controls (depending on IO-Feature)
    switch(ioNo) {
    case 1: // I/O 1
        // input
        setIO1InUICtrl(inPolarity, inMode, inState);
        // output
        setIO1OutUICtrl(outPolarity, outMode, outState, all);
        break;
    case 2: // I/O 2
        // input
        setIO2InUICtrl(inPolarity, inMode, inState);
        // output
        setIO2OutUICtrl(outPolarity, outMode, outState, all);
        break;
    case 3: // I/O 3
        // input
        setIO3InUICtrl(inPolarity, inMode, inState);
        // output
        setIO3OutUICtrl(outPolarity, outMode, outState, all);
        break;
    default: // only I/O 1, 2 and 3 are currently supported
        logger->log("AdjustmentsForm::updateIO: Only 2 GP-I/Os are currently supported!");
        break;
    }
}


void AdjustmentsForm::setIO1InUICtrl(bool polarity, unsigned int mode, bool state) {
    // mode
    switch(mode) {
    case IO_IN_OFF:
        in1ModeComboBox->setCurrentText(IO_IN_OFF_STRING);
        break;
    case IO_IN_TRIGGER:
        in1ModeComboBox->setCurrentText(IO_IN_TRIGGER_STRING);
        break;
    case IO_IN_INC_DEC:
        in1ModeComboBox->setCurrentText(IO_IN_INC_DEC_STRING);
        break;
    case IO_IN_SPI:
        in1ModeComboBox->setCurrentText(IO_IN_SPI_STRING);
        break;
    default:
        break;
    }
    
    // polarity
    in1PolarityComboBox->setCurrentText(polarity ? "high act." : "low act.");
    
    // state
    in1StateLineEdit->setText(state ? "high" : "low");
}


void AdjustmentsForm::setIO1OutUICtrl(bool polarity, unsigned int mode, bool state, bool all) {
    // mode
    switch(mode) {
    case IO_OUT_OFF:
        out1ModeComboBox->setCurrentText(IO_OUT_OFF_STRING);
        break;
    case IO_OUT_PIN_STATE:
        out1ModeComboBox->setCurrentText(IO_OUT_PIN_STATE_STRING);
        break;
    case IO_OUT_INT_ENA:
        out1ModeComboBox->setCurrentText(IO_OUT_INT_ENA_STRING);
        break;
    case IO_OUT_INC_DEC_CMP:
        out1ModeComboBox->setCurrentText(IO_OUT_INC_DEC_CMP_STRING);
        break;
    case IO_OUT_SPI_INT:
        out1ModeComboBox->setCurrentText(IO_OUT_SPI_INT_STRING);
        break;
    case IO_OUT_SPI_EXT:
        out1ModeComboBox->setCurrentText(IO_OUT_SPI_EXT_STRING);
        break;
    case IO_OUT_FRAME_VALID:
        out1ModeComboBox->setCurrentText(IO_OUT_FRAME_VALID_STRING);
        break;
    case IO_OUT_BUSY:
        out1ModeComboBox->setCurrentText(IO_OUT_BUSY_STRING);
        break;
    case IO_OUT_CORR_IN:
        out1ModeComboBox->setCurrentText(IO_OUT_CORR_IN_STRING);
        break;
    default:
        break;
    }
    
    if(all) {
        // polarity
        out1PolarityComboBox->setCurrentText(polarity ? "high act." : "low act.");
    
        // state
        logger->log("AdjustmentsForm::setIO1OutUICtrl: Output-polarity: %s; output-state: %s",
                    (polarity ? "high act.": "low act."), (state ? "active" : "inactive"));
        out1StateComboBox->setCurrentText(state ? "active" : "inactive");
    }
    
    if(!(out1ModeComboBox->currentText() == IO_OUT_PIN_STATE_STRING) &&
       out1StateComboBox->isEnabled() &&
       out1StateComboBox->hasFocus()) {
        focusdata->next()->setFocus();
    }
    if(out1ModeComboBox->currentText() == IO_OUT_PIN_STATE_STRING) {
        out1StateComboBox->setEnabled(true);
    }
    else {
        out1StateComboBox->setEnabled(false);
        logger->log("Out-State is only available if 'Output-Mode' is selected!");
    }
}


void AdjustmentsForm::setIO2InUICtrl(bool polarity, unsigned int mode, bool state) {
    switch(mode) {
    case IO_IN_OFF:
        in2ModeComboBox->setCurrentText(IO_IN_OFF_STRING);
        break;
    case IO_IN_TRIGGER:
        in2ModeComboBox->setCurrentText(IO_IN_TRIGGER_STRING);
        break;
    case IO_IN_INC_DEC:
        in2ModeComboBox->setCurrentText(IO_IN_INC_DEC_STRING);
        break;
    case IO_IN_SPI:
        in2ModeComboBox->setCurrentText(IO_IN_SPI_STRING);
        break;
    default:
        break;
    }
    
    in2PolarityComboBox->setCurrentItem(polarity ? 1 : 0);
    
    in2StateLineEdit->setText(state ? "high" : "low");
}


void AdjustmentsForm::setIO2OutUICtrl(bool polarity, unsigned int mode, bool state, bool all) {
    switch(mode) {
    case IO_OUT_OFF:
        out2ModeComboBox->setCurrentText(IO_OUT_OFF_STRING);
        break;
    case IO_OUT_PIN_STATE:
        out2ModeComboBox->setCurrentText(IO_OUT_PIN_STATE_STRING);
        break;
    case IO_OUT_INT_ENA:
        out2ModeComboBox->setCurrentText(IO_OUT_INT_ENA_STRING);
        break;
    case IO_OUT_INC_DEC_CMP:
        out2ModeComboBox->setCurrentText(IO_OUT_INC_DEC_CMP_STRING);
        break;
    case IO_OUT_SPI_INT:
        out2ModeComboBox->setCurrentText(IO_OUT_SPI_INT_STRING);
        break;
    case IO_OUT_SPI_EXT:
        out2ModeComboBox->setCurrentText(IO_OUT_SPI_EXT_STRING);
        break;
    case IO_OUT_FRAME_VALID:
        out2ModeComboBox->setCurrentText(IO_OUT_FRAME_VALID_STRING);
        break;
    case IO_OUT_BUSY:
        out2ModeComboBox->setCurrentText(IO_OUT_BUSY_STRING);
        break;
    case IO_OUT_CORR_IN:
        out2ModeComboBox->setCurrentText(IO_OUT_CORR_IN_STRING);
        break;
    default:
        break;
    }
    
    if(all) {
        out2PolarityComboBox->setCurrentText(polarity ? "high act." : "low act.");
    
        logger->log("AdjustmentsForm::setIO2OutUICtrl: Outout-polarity: %s; output-state: %d",
                    (polarity ? "high act.": "low act."), state);
        out2StateComboBox->setCurrentText(state ? "active" : "inactive");
    }
    
    if(!(out2ModeComboBox->currentText() == IO_OUT_PIN_STATE_STRING) &&
       out2StateComboBox->isEnabled() &&
       out2StateComboBox->hasFocus()) {
        focusdata->next()->setFocus();
    }
    out2StateComboBox->setEnabled(
            out2ModeComboBox->currentText() == IO_OUT_PIN_STATE_STRING);
}


void AdjustmentsForm::setIO3InUICtrl(bool polarity, unsigned int mode, bool state) {
    switch(mode) {
    case IO_IN_OFF:
        in3ModeComboBox->setCurrentText(IO_IN_OFF_STRING);
        break;
    case IO_IN_TRIGGER:
        in3ModeComboBox->setCurrentText(IO_IN_TRIGGER_STRING);
        break;
    case IO_IN_INC_DEC:
        in3ModeComboBox->setCurrentText(IO_IN_INC_DEC_STRING);
        break;
    case IO_IN_SPI:
        in3ModeComboBox->setCurrentText(IO_IN_SPI_STRING);
        break;
    default:
        break;
    }
    
    in3PolarityComboBox->setCurrentItem(polarity ? 1 : 0);
    
    in3StateLineEdit->setText(state ? "high" : "low");
}


void AdjustmentsForm::setIO3OutUICtrl(bool polarity, unsigned int mode, bool state, bool all) {
    switch(mode) {
    case IO_OUT_OFF:
        out3ModeComboBox->setCurrentText(IO_OUT_OFF_STRING);
        break;
    case IO_OUT_PIN_STATE:
        out3ModeComboBox->setCurrentText(IO_OUT_PIN_STATE_STRING);
        break;
    case IO_OUT_INT_ENA:
        out3ModeComboBox->setCurrentText(IO_OUT_INT_ENA_STRING);
        break;
    case IO_OUT_INC_DEC_CMP:
        out3ModeComboBox->setCurrentText(IO_OUT_INC_DEC_CMP_STRING);
        break;
    case IO_OUT_SPI_INT:
        out3ModeComboBox->setCurrentText(IO_OUT_SPI_INT_STRING);
        break;
    case IO_OUT_SPI_EXT:
        out3ModeComboBox->setCurrentText(IO_OUT_SPI_EXT_STRING);
        break;
    case IO_OUT_FRAME_VALID:
        out3ModeComboBox->setCurrentText(IO_OUT_FRAME_VALID_STRING);
        break;
    case IO_OUT_BUSY:
        out3ModeComboBox->setCurrentText(IO_OUT_BUSY_STRING);
        break;
    case IO_OUT_CORR_IN:
        out3ModeComboBox->setCurrentText(IO_OUT_CORR_IN_STRING);
        break;
    default:
        break;
    }
    
    if(all) {
        out3PolarityComboBox->setCurrentText(polarity ? "high act." : "low act.");
    
        logger->log("AdjustmentsForm::setIO3OutUICtrl: Outout-polarity: %s; output-state: %d",
                    (polarity ? "high act.": "low act."), state);
        out3StateComboBox->setCurrentText(state ? "active" : "inactive");
    }
    
    if(!(out3ModeComboBox->currentText() == IO_OUT_PIN_STATE_STRING) &&
       out3StateComboBox->isEnabled() &&
       out3StateComboBox->hasFocus()) {
        focusdata->next()->setFocus();
    }
    out3StateComboBox->setEnabled(
            out3ModeComboBox->currentText() == IO_OUT_PIN_STATE_STRING);
}


void AdjustmentsForm::updateSerialIO() {
    unsigned int baud, len, par, stop, bufSize;
    
    if(!dc1394->getSerialParameters(id, &baud, &len, &par, &stop, &bufSize)) {
        logger->log("AdjustmentsForm::updateSerialIO: Error while trying to read parameters of the serial interface!");
        return;
    }
    
    logger->log("AdjustmentsForm::updateSerialIO: Serial Parameters:\n\tBaudRate: %d\n\tCharLen: %d\n\tParity: %d\n\tStopBits: %d\n\tBufferSize: %d",
                baud, len, par, stop, bufSize);
    
    serialIOBdRateComboBox->setCurrentText(QString::number(baud) + QString(" bps"));
    serialIOCharLenComboBox->setCurrentItem(len - 7);
    serialIOParityComboBox->setCurrentItem(par);
    serialIOStopbitsComboBox->setCurrentItem(stop);
    serialIOBufferSizeLineEdit->setText(QString::number(bufSize) + " byte(s)");
}


void AdjustmentsForm::updateDSNU() {
    bool dsnuOn = false, blemishOn = false;
    bool dsnuImgShown, blemishImgShown;
    unsigned int dsnuImgCnt, blemishImgCnt;
    bool dsnuAvailable = dsnuOnCheckBox->isEnabled();
    bool blemishAvailable = blemishOnCheckBox->isEnabled();
    bool initSave = initialized;
    
    
    // update UI from camera (camera is updated within the corresponding callbacks!)
    
    initialized = false;
    
    if(dsnuAvailable) {
        if(!dc1394->dsnuEnabled(id, &dsnuOn)) {
            logger->log("AdjustmentsForm::updateDSNU: Error while trying to get current DSNU-status");
        }
        else {
            dsnuOnCheckBox->setChecked(dsnuOn);
        }
    }
    
    if(blemishAvailable) {
        if(!dc1394->blemishEnabled(id, &blemishOn)) {
            logger->log("AdjustmentsForm::updateDSNU: Error while trying to get current Blemish-status");
        }
        else {
            blemishOnCheckBox->setChecked(blemishOn);
        }
    }
    
    // enable 'ShowImage' check-box
    if((dsnuOn  && dc1394->dsnuImgOn(id, &dsnuImgShown)) ||
       (blemishOn && dc1394->blemishImgOn(id, &blemishImgShown))) {
        
        dsnuShowImageCheckBox->setEnabled(true);
        dsnuShowImageCheckBox->setChecked(dsnuOn ? dsnuImgShown : blemishImgShown);
    }
    else {
        dsnuShowImageCheckBox->setEnabled(false);
        dsnuShowImageCheckBox->setChecked(false);
    }
    
    if(dsnuOn && blemishOn && (dsnuImgShown != blemishImgShown))
        logger->log("AdjustmentsForm::updateDSNU: Warning! - DSNU-'Show Image' flag differs from the Blemish one!");
    
    // enable 'image-count' spin-box
    if((dsnuOn && dc1394->getDSNUImgCnt(id, &dsnuImgCnt)) ||
       (blemishOn && dc1394->getBlemishImgCnt(id, &blemishImgCnt))) {
        
        dsnuGrabCountSpinBox->setEnabled(true);
        dsnuGrabCountSpinBox->setValue(dsnuOn ? dsnuImgCnt : blemishImgCnt);
    }
    else {
        dsnuGrabCountSpinBox->setEnabled(false);
    }
    
    if(dsnuOn && blemishOn && (dsnuImgCnt != blemishImgCnt))
        logger->log("AdjustmentsForm::updateDSNU: Warning! - DSNU-'Image Count' differs from the Blemish one!");
    
    // enable 'Reset'-button
    dsnuResetPushButton->setEnabled(dsnuOn || blemishOn);
    
    // enable 'Zero'-button
    dsnuZeroPushButton->setEnabled(dsnuOn || blemishOn);
    
    // enable 'Build'-button
    dsnuComputePushButton->setEnabled((dsnuOn || blemishOn) &&
                                      (dsnuGrabCountSpinBox->value() > 0));
    
    initialized = initSave;
}


void AdjustmentsForm::updateAutoShutter() {
    unsigned int min, max;
    unsigned int minLimit = 10; // !!! Min-Value for AutoShutter is 10us
    unsigned int maxLimit;
    bool initSave = initialized;
    unsigned int timebase;
    unsigned int stdMaxLimit = shutterSpeedSlider->maxValue();
    
    
    dc1394->getTimebase(id, &timebase);
    maxLimit = stdMaxLimit * timebase;
    
    logger->log("AdjustmentsForm::updateAutoShutter: Auto-Shutter: MinLimit = %dus, MaxLimit = %dus", minLimit, maxLimit);
    
    // update AutoShutter from UI
    if(autoShutterChanged) {
        min = autoShutterMinSpinBox->value();
        max = autoShutterMaxSpinBox->value();
        
        if(!dc1394->setAutoShutterLimits(id, min, max)) {
            logger->log("AdjustmentsForm::updateAutoShutter: Error while trying to change AutoShutter-Limits to (%d, %d)", min, max);
        }
        
        return;
    }
    
    // update UI controls from camera
    if(!dc1394->getAutoShutterLimits(id, &min, &max)) {
        logger->log("AdjustmentsForm::updateAutoShutter: Error while trying to read AutoShutter-Limits");
        autoShutterMinSlider->setEnabled(false);
        autoShutterMinSpinBox->setEnabled(false);
        autoShutterMaxSlider->setEnabled(false);
        autoShutterMaxSpinBox->setEnabled(false);
        return;
    }
    
    logger->log("AdjustmentsForm::updateAutoShutter: AutoShutter-Limits: %dus ... %dus",
                min, max);
    
    if(min >= max) {
        logger->log("AdjustmentsForm::updateAutoShutter: 'min >= max' ! - therefore adjusting these values!");
        if(min < maxLimit) {
            if(!dc1394->setAutoShutterLimits(id, min, min + 1)) {
                logger->log("AdjustmentsForm::updateAutoShutter: AutoShutter-state is inconsistent, and cannot be changed");
                logger->log("\ttherefore disabling the feature!");
                autoShutterGroupBox->setEnabled(false);
                return;
            }
        }
        else {
            if(!dc1394->setAutoShutterLimits(id, min -1, min)) {
                logger->log("AdjustmentsForm::updateAutoShutter: AutoShutter-state is inconsistent, and cannot be changed");
                logger->log("\ttherefore disabling the feature!");
                autoShutterGroupBox->setEnabled(false);
                return;
            }
        }
    }
    
    if(max > maxLimit) {
        max = maxLimit;
        if(!dc1394->setAutoShutterLimits(id, min, max)) {
            logger->log("AdjustmentsForm::updateAutoShutter: Correction of limits failed - therefore disabling the feature!");
            autoShutterGroupBox->setEnabled(false);
            return;
        }
    }
    
    if(min < minLimit) {
        logger->log("AdjustmentsForm::updateAutoShutter: Trying to bring 'Auto-Shutter' into a 'constitant' sate, because the min. Value is 10 us!");
        min = minLimit;
        max = (min < max) ? max : (min + 1);
        if(!dc1394->setAutoShutterLimits(id, min, max)) {
            logger->log("\t...failed - therefore disabling the feature!");
            autoShutterGroupBox->setEnabled(false);
            return;
        }
        logger->log("\t...succeeded...");
    }
    
    initialized = false;
    
    autoShutterMinSlider->setMinValue(minLimit);
    autoShutterMinSpinBox->setMinValue(minLimit);
    autoShutterMinSlider->setMaxValue(max - 1);
    autoShutterMinSpinBox->setMaxValue(max - 1);
    autoShutterMinSlider->setValue(min);
    autoShutterMinSpinBox->setValue(min);
    
    autoShutterMaxSlider->setMinValue(min + 1);
    autoShutterMaxSpinBox->setMinValue(min + 1);
    autoShutterMaxSlider->setMaxValue(maxLimit);
    autoShutterMaxSpinBox->setMaxValue(maxLimit);
    autoShutterMaxSlider->setValue(max);
    autoShutterMaxSpinBox->setValue(max);
    
    initialized = initSave;
}


void AdjustmentsForm::discardAutoShutter() {
    autoShutterChanged = false;
    updateAutoShutter();
}


void AdjustmentsForm::updateAutoGain() {
    unsigned int min, max;
    unsigned int minLimit = 0;
    unsigned int maxLimit = gainSpinBox->maxValue();
    bool initSave = initialized;
    
    
    // update camera from UI
    if(autoGainChanged) {
        min = autoGainMinSpinBox->value();
        max = autoGainMaxSpinBox->value();
        
        if(!dc1394->setAutoGainLimits(id, min, max)) {
            logger->log("AdjustmentsForm::updateAutoGain: Error while trying to change AutoGain-Limits to (%d, %d)", min, max);
        }
        
        return;
    }
    
    // update UI-controls from camera
    if(!dc1394->getAutoGainLimits(id, &min, &max)) {
        logger->log("AdjustmentsForm::updateAutoGain: Error while trying to read AutoGain-Limits");
        autoGainMinSlider->setEnabled(false);
        autoGainMinSpinBox->setEnabled(false);
        autoGainMaxSlider->setEnabled(false);
        autoGainMaxSpinBox->setEnabled(false);
        return;
    }
    
    logger->log("AdjustmentsForm::updateAutoGain: AutoGain-Limits: %d - %d", min, max);
    
    if(min >= max) {
        logger->log("AdjustmentsForm::updateAutoGain: 'min >= max' ! - therefore adjusting these values!");
        if(min < maxLimit) {
            if(!dc1394->setAutoGainLimits(id, min, min + 1)) {
                logger->log("AdjustmentsForm::updateAutoGain: AutoGain-state is inconsistent, and cannot be changed");
                logger->log("\ttherefore disabling the feature!");
                autoGainGroupBox->setEnabled(false);
                return;
            }
        }
        else {
            if(!dc1394->setAutoGainLimits(id, min -1, min)) {
                logger->log("AdjustmentsForm::updateAutoGain: AutoGain-state is inconsistent, and cannot be changed");
                logger->log("\ttherefore disabling the feature!");
                autoGainGroupBox->setEnabled(false);
                return;
            }
        }
    }
    
    if(max > maxLimit) {
        max = maxLimit;
        if(!dc1394->setAutoGainLimits(id, min, max)) {
            logger->log("AdjustmentsForm::updateAutoGain: Correction of limits failed - therefore disabling the feature!");
            autoGainGroupBox->setEnabled(false);
            return;
        }
    }
    
    if(min < minLimit) {
        logger->log("AdjustmentsForm::updateAutoGain: Trying to bring 'Auto-Gain' into a 'constitant' sate!");
        min = minLimit;
        max = (min < max) ? max : (min + 1);
        if(!dc1394->setAutoGainLimits(id, min, max)) {
            logger->log("\t...failed - therefore disabling the feature!");
            autoGainGroupBox->setEnabled(false);
            return;
        }
        logger->log("\t...succeeded...");
    }
    
    initialized = false;
    
    autoGainMinSlider->setMinValue(minLimit);
    autoGainMinSlider->setMaxValue(max -  1);
    autoGainMinSpinBox->setMinValue(minLimit);
    autoGainMinSpinBox->setMaxValue(max - 1);
    autoGainMinSlider->setValue(min);
    autoGainMinSpinBox->setValue(min);
    autoGainMaxSlider->setMinValue(min + 1);
    autoGainMaxSlider->setMaxValue(maxLimit);
    autoGainMaxSpinBox->setMinValue(min + 1);
    autoGainMaxSpinBox->setMaxValue(maxLimit);
    autoGainMaxSlider->setValue(max);
    autoGainMaxSpinBox->setValue(max);
    
    initialized = initSave;
}


void AdjustmentsForm::discardAutoGain() {
    autoGainChanged = false;
    updateAutoGain();
}


void AdjustmentsForm::updateAutoAOI() {
    unsigned int x, y, width, height;
    unsigned int maxXSize, maxYSize;
    unsigned int maxWidth = dc1394->getWidth(id);
    unsigned int maxSectorWidth = maxWidth;
    unsigned int maxHeight = dc1394->getHeight(id);
    unsigned int maxSectorHeight = maxHeight;
    bool autoAOIOn, showAOIEnabled;
    bool initSave;
    
    
    if(autoAOIChanged) {
        // update Camera from UI
        logger->log("AdjustmentsForm::updateAutoAOI: Update camera AutoAOI-feature from UI...");
        
        // get current status
        if(!dc1394->getAutoAOIStatus(id, &autoAOIOn, &showAOIEnabled)) {
            logger->log("AdjustmentsForm::updateAutoAOI: Unable to read the current AutoAOI-Status!");
            return;
        }
        
        // feature enabled changed
        if(autoAOIOn != autoAOIOnCheckBox->isChecked()) {
            if(autoAOISwitched) {
                logger->log("AdjustmentsForm::updateAutoAOI: Switching AutoAOI %s", autoAOIOnCheckBox->isChecked() ? "on" : "off");
                if(!dc1394->enableAutoAOI(id, autoAOIOnCheckBox->isChecked())) {
                    logger->log("AdjustmentsForm::updateAutoAOI: Error while trying to %s 'AutoAOI'",
                           autoAOIOnCheckBox->isChecked() ? "enable" : "disable");
                    autoAOIGroupBox->setEnabled(false);
                    return;
                }
            }
            else {
                logger->log("AdjustmentsForm::updateAutoAOI: AutoAOI state changed in camera ???");
            }
        }
        
        // change geometry-parameters
        logger->log("AdjustmentsForm::updateAutoAOI: Setting AutoAOI to %d x %d - %d, %d",
               autoAOIWidthSpinBox->value(),
               autoAOIHeightSpinBox->value(),
               autoAOIXSpinBox->value(),
               autoAOIYSpinBox->value());
        
        if(!dc1394->setAutoAOIPos(id, autoAOIXSpinBox->value(),
                                   autoAOIYSpinBox->value())) {
            logger->log("AdjustmentsForm::updateAutoAOI: Error while trying to change AutoAOI-Position to %d, %d",
                   autoAOIXSpinBox->value(),
                   autoAOIYSpinBox->value());
            autoAOIGroupBox->setEnabled(false);
            return;
        }
        
        if(!dc1394->setAutoAOISize(id, autoAOIWidthSpinBox->value(),
                                   autoAOIHeightSpinBox->value())) {
            logger->log("AdjustmentsForm::updateAutoAOI: Error while trying to change AutoAOI-Size to %d x %d",
                   autoAOIWidthSpinBox->value(),
                   autoAOIHeightSpinBox->value());
            autoAOIGroupBox->setEnabled(false);
            return;
        }
        
        // show area changed
        if(showAOIEnabled != autoAOIShowAreaCheckBox->isChecked()) {
            logger->log("AdjustmentsForm::updateAutoAOI: Show Area-Flag changed...");
            if(!dc1394->showAutoAOIArea(id, autoAOIShowAreaCheckBox->isChecked())) {
                logger->log("AdjustmentsForm::updateAutoAOI: Show feature seems to be currently not available");
                initSave = initialized;
                initialized = false;
                autoAOIShowAreaCheckBox->setChecked(false);
                initialized = initSave;
                autoAOIShowAreaCheckBox->setEnabled(false);
                return;
            }
        }
        
        // if feature is switched off - nothing more to do...
        if(!(autoAOIOnCheckBox->isChecked())) {
            logger->log("AdjustmentsForm::updateAutoAOI: AutoAOI is switched off - therefore disabling the other controls...");
            enableAutoAOI(false);
            return;
        }
        else {
            logger->log("AdjustmentsForm::updateAutoAOI: AutoAOI is switched on - therefore enabling the other controls...");
            enableAutoAOI(true);
        }
    }
    else {
        // update UI from Camera
        logger->log("AdjustmentsForm::updateAutoAOI: Updating AutoAOI UI-controls...");
        
        if(!dc1394->getAutoAOISize(id, &width, &height)) { 
            logger->log("AdjustmentsForm::updateAutoAOI: Error while reading AutoAOI-Size!");
            return;
        }
        
        if(!dc1394->getAutoAOIPos(id, &x, &y)) {
            logger->log("AdjustmentsForm::updateAutoAOI: Error while reading AutoAOI-Position!");
            return;
        }
        
        if(!dc1394->getAutoAOIStatus(id, &autoAOIOn, &showAOIEnabled)) {
            logger->log("AdjustmentsForm::updateAutoAOI: Error while reading AutoAOI-Status!");
            return;
        }

        logger->log("AdjustmentsForm::updateAutoAOI: AutoAOI: %d, %d - %d x %d - %s / Area: %s", x, y, width, height,
               autoAOIOn ? "enabled" : "disabled", showAOIEnabled ? "shown" : "hidden");
        logger->log("AdjustmentsForm::updateAutoAOI: AutoAOI: max. Width = %d; max. Height = %d", maxWidth, maxHeight);
        
        initSave = initialized;
        initialized = false;
        autoAOISwitched = false;
        
        enableAutoAOI(autoAOIOn);
        
        autoAOIOnCheckBox->setChecked(autoAOIOn);
        autoAOIShowAreaCheckBox->setChecked(showAOIEnabled);
        
        autoAOIWidthSlider->setMinValue(128);
        // max. AutoAOI-Area can be greater than max. Resolution
        maxWidth = (maxWidth != (maxWidth & 0xffffff80UL)) ? maxWidth + 128 : maxWidth;
        maxHeight = (maxHeight != (maxHeight & 0xffffff80UL)) ? maxHeight + 128 : maxHeight;
        autoAOIWidthSlider->setMaxValue((maxWidth & 0xffffff80UL) == 0 ? 128 :
                                        maxWidth & 0xffffff80UL);
        autoAOIWidthSlider->setLineStep(128);
        autoAOIWidthSpinBox->setMinValue(128);
        autoAOIWidthSpinBox->setMaxValue((maxWidth & 0xffffff80UL) == 0 ? 128 :
                                         maxWidth & 0xffffff80UL);
        autoAOIWidthSpinBox->setLineStep(128);
        
        autoAOIHeightSlider->setMinValue(128);
        autoAOIHeightSlider->setMaxValue((maxHeight & 0xffffff80UL) == 0 ? 128 :
                                         maxHeight & 0xffffff80UL);
        autoAOIHeightSlider->setLineStep(128);
        autoAOIHeightSpinBox->setMinValue(128);
        autoAOIHeightSpinBox->setMaxValue((maxHeight & 0xffffff80UL) == 0 ? 128 :
                                          maxHeight & 0xffffff80UL);
        autoAOIHeightSpinBox->setLineStep(128);
        
        autoAOIXSlider->setMinValue(0);
        autoAOIXSlider->setMaxValue(maxWidth - width);
        autoAOIXSlider->setLineStep(128);
        autoAOIXSpinBox->setMinValue(0);
        autoAOIXSpinBox->setMaxValue(maxWidth - width);
        autoAOIXSpinBox->setLineStep(128);
        
        autoAOIYSlider->setMinValue(0);
        autoAOIYSlider->setMaxValue(maxHeight - height);
        autoAOIYSlider->setLineStep(128);
        autoAOIYSpinBox->setMinValue(0);
        autoAOIYSpinBox->setMaxValue(maxHeight - height);
        autoAOIYSpinBox->setLineStep(128);

        if(((int)(x + width) > autoAOIWidthSlider->maxValue()) ||
           ((int)(y + height) > autoAOIHeightSlider->maxValue())) {
            logger->log("AdjustmentsForm::updateAutoAOI: Correcting size and position...");
            // correct values
            if(!dc1394->setAutoAOIPos(id, 0, 0)) {
                logger->log("AdjustmentsForm::updateAutoAOI: Error while trying to correct AutoAOI-Position!");
            }
            if(!dc1394->setAutoAOISize(id, autoAOIWidthSlider->maxValue(),
                                       autoAOIHeightSlider->maxValue())) {
                logger->log("AdjustmentsForm::updateAutoAOI: Error while trying to correct AutoAOI-Size!");
            }
            // re-read corrected values from camera
            if(!dc1394->getAutoAOIPos(id, &x, &y)) {
                logger->log("AdjustmentsForm::updateAutoAOI: Error while trying to read the corrected AutoAOI-Position");
            }
            if(!dc1394->getAutoAOISize(id, &width, &height)) {
                logger->log("AdjustmentsForm::updateAutoAOI: Error while trying to read the corrected AutoAOI-Size");
            }
            logger->log("AdjustmentsForm::updateAutoAOI: AutoAOI: %d, %d - %d x %d - %s / Area: %s", x, y, width, height,
                   autoAOIOn ? "enabled" : "disabled", showAOIEnabled ? "shown" : "hidden");
        }
        
        logger->log("AdjustmentsForm::updateAutoAOI: Setting Width-Slider to : %d", width);
        autoAOIWidthSlider->setValue(width);
        logger->log("AdjustmentsForm::updateAutoAOI: Setting Width-SpinBox to : %d", width);
        autoAOIWidthSpinBox->setValue(width);
        
        logger->log("AdjustmentsForm::updateAutoAOI: Setting Height-Slider to : %d", height);
        autoAOIHeightSlider->setValue(height);
        logger->log("AdjustmentsForm::updateAutoAOI: Setting Height-SpinBox to : %d", height);
        autoAOIHeightSpinBox->setValue(height);
        
        logger->log("AdjustmentsForm::updateAutoAOI: Setting X-Slider to : %d", x);
        autoAOIXSlider->setValue(x);
        logger->log("AdjustmentsForm::updateAutoAOI: Setting X-SpinBox to : %d", x);
        autoAOIXSpinBox->setValue(x);
        
        logger->log("AdjustmentsForm::updateAutoAOI: Setting Y-Slider to : %d", y);
        autoAOIYSlider->setValue(y);
        logger->log("AdjustmentsForm::updateAutoAOI: Setting Y-SpinBox to : %d", y);
        autoAOIYSpinBox->setValue(y);
        
        // graphical representation of the current region
        maxXSize = autoAOIImageFrame->width();
        maxYSize = autoAOIImageFrame->height();
        
        maxSectorWidth = autoAOIWidthSlider->maxValue();
        maxSectorHeight = autoAOIHeightSlider->maxValue();
        
        autoAOISectorFrame->resize((maxXSize - 6) * width / maxSectorWidth,
                                   (maxYSize - 6) * height / maxSectorHeight);
        autoAOISectorFrame->move((x * (maxXSize - 3) / maxSectorWidth) + 3,
                               (y * (maxYSize - 3) / maxSectorHeight) + 3);
        
        initialized = initSave;
    }
} // updateAutoAOI()


void AdjustmentsForm::enableAutoAOI(bool enable) {
    autoAOIWidthSlider->setEnabled(enable);
    autoAOIWidthSpinBox->setEnabled(enable);
    autoAOIHeightSlider->setEnabled(enable);
    autoAOIHeightSpinBox->setEnabled(enable);
    autoAOIXSlider->setEnabled(enable);
    autoAOIXSpinBox->setEnabled(enable);
    autoAOIYSlider->setEnabled(enable);
    autoAOIYSpinBox->setEnabled(enable);
    autoAOIMaxPushButton->setEnabled(enable);
    autoAOIShowAreaCheckBox->setEnabled(enable);
}


void AdjustmentsForm::discardAutoAOI() {
    autoAOIChanged = false;
    updateAutoAOI();
}


void AdjustmentsForm::updateDeferredImg() {
    bool status, fastCapture;
    bool initSave;
    unsigned int fifoSize, frameCount;
    
    
    if(deferredImgChanged) {
        // update camera from UI
        if(!dc1394->enableDeferredImg(id, deferredImgHoldCheckBox->isChecked())) {
            logger->log("AdjustmentsForm::updateDeferredImg: Error while trying to read current DeferredImgTransport-Status");
        }
        
        if(!dc1394->deferredImgFastCaptureEnable(id,
                                                 deferredImgFastCaptureCheckBox->isChecked())) {
            logger->log("AdjustmentsForm::updateDeferredImg: Error while trying to enable DeferredImg FastCapture");
        }
    }
    else {
        // update UI from camera
        if(!dc1394->getDeferredImgStatus(id, &status)) {
            logger->log("AdjustmentsForm::updateDeferredImg: Error while trying to read DeferredImgTransport-Status from camera");
        }
        
        if(!dc1394->deferredImgFastCaptureStatus(id, &fastCapture)) {
            logger->log("AdjustmentsForm::updateDeferredImg: Error while trying to read FastCaptureStatus from camera");
        }
        
        if(!dc1394->getDeferredImgFifoSize(id, &fifoSize)) {
            logger->log("AdjustmentsForm::updateDeferredImg: Error while trying to read FiFo-Size");
        }
        
        if(!dc1394->getDeferredImgFrameCount(id, &frameCount)) {
            logger->log("AdjustmentsForm::updateDeferredImg: Error while trying to read Frame-Count");
        }
        
        initSave = initialized;
        initialized = false;
        
        deferredImgHoldCheckBox->setChecked(status);
        deferredImgFastCaptureCheckBox->setChecked(fastCapture);
        deferredImgFifoSizeLineEdit->setText(QString::number(fifoSize));
        deferredImgFrameCountSpinBox->setMinValue(0);
        deferredImgFrameCountSpinBox->setMaxValue(fifoSize);
        deferredImgFrameCountSpinBox->setValue(frameCount);
        
        deferredImgSendPushButton->setEnabled(deferredImgHoldCheckBox->isChecked());
        
        initialized = initSave;
    }
} // updateDeferredImg()


void AdjustmentsForm::discardDeferredImg() {
    deferredImgChanged = false;
    updateDeferredImg();
}


void AdjustmentsForm::updateFrameInfo() {
    unsigned int count;
    
    
    if(!dc1394->getFrameInfoCount(id, &count))
        logger->log("AdjustmentsForm::updateFrameInfo: Error while trying to read FrameInfo counter value");
    else
        frameInfoCountLineEdit->setText(QString::number(count));
}


void AdjustmentsForm::updateDelInt() {
    bool enabled, initSave;
    unsigned int value;
    
    
    if(delIntChanged) {
        // update camera from UI
        if(!dc1394->delIntEnable(id, delIntOnCheckBox->isChecked())) {
            logger->log("AdjustmentsForm::updateDelInt: Error while trying to %s Delayed Integration",
                   delIntOnCheckBox->isChecked() ? "enable" : "disable");
        }
        
        if(!dc1394->setDelIntValue(id, delIntSpinBox->value())) {
            logger->log("AdjustmentsForm::updateDelInt: Error while trying to change Del. Int. value");
        }
    }
    else {
        // update UI from camera
        if(!dc1394->delIntStatus(id, &enabled)) {
            logger->log("AdjustmentsForm::updateDelInt: Error while trying to read Del. Int. status");
            return;
        }
        
        if(!dc1394->getDelIntValue(id, &value)) {
            logger->log("AdjustmentsForm::updateDelInt: Error while trying to read Del. Int. value");
            return;
        }
        
        initSave = initialized;
        initialized = false;
        
        delIntOnCheckBox->setChecked(enabled);
        
        delIntSpinBox->setMinValue(0);
        delIntSpinBox->setMaxValue((int)pow(2, 20));
        delIntSpinBox->setValue(value);
        
        delIntSpinBox->setEnabled(enabled);
        
        initialized = initSave;
    }
}


void AdjustmentsForm::discardDelInt() {
    delIntChanged = false;
    updateDelInt();
}


void AdjustmentsForm::updateIncDec() {
    bool enabled, initSave;
    unsigned int compare, count;
    
    
    if(incDecChanged) {
        // update camera from UI
        if(!dc1394->incDecEnable(id, incDecOnCheckBox->isChecked())) {
            logger->log("AdjustmentsForm::updateIncDec: Error while trying to %s Inc. Dec.",
                   incDecOnCheckBox->isChecked() ? "enable" : "disable");
        }
        
        if(!dc1394->setIncDecCompare(id, incDecCompareSpinBox->value())) {
            logger->log("AdjustmentsForm::updateIncDec: Error while trying to set Inc. Dec. to %d",
                   incDecCompareSpinBox->value());
        }
    }
    else {
        // update UI from camera
        if(!dc1394->incDecStatus(id, &enabled)) {
            logger->log("AdjustmentsForm::updateIncDec: Error while trying to read Inc. Dec. status");
        }
        
        if(!dc1394->getIncDecCompare(id, &compare)) {
            logger->log("AdjustmentsForm::updateIncDec: Error while trying to read Inc. Dec. compare value");
        }
        
        if(!dc1394->getIncDecCounter(id, &count)) {
            logger->log("AdjustmentsForm::updateIncDec: Error while trying to read Inc. Dec. counter value");
        }
        
        initSave = initialized;
        initialized = false;
        
        incDecOnCheckBox->setChecked(enabled);
        
        incDecCompareSpinBox->setMinValue(0);
        incDecCompareSpinBox->setMaxValue((int)pow(2, 12));
        incDecCompareSpinBox->setValue(compare);
        incDecCompareSpinBox->setEnabled(enabled);
        
        incDecCountLineEdit->setText(QString::number(count));
        
        incDecResetPushButton->setEnabled(enabled);
        
        initialized = initSave;
    }
}


void AdjustmentsForm::discardIncDec() {
    incDecChanged = false;
    updateIncDec();
}


void AdjustmentsForm::shadingCorrectionOnChecked( bool newValue ) {
    bool initSave = initialized;
    
    
    if(initialized && newValue && !shadingImgLoaded) {
        if(QMessageBox::warning(this, "Warning",
                                "No shading image has been loaded or generated since the start\nof the application. This may lead to a strange result,\n depending on the implementation.\n\nContinue?",
                                QMessageBox::Yes | QMessageBox::Default,
                                QMessageBox::No | QMessageBox::Escape,
                                QMessageBox::NoButton) == QMessageBox::No) {
            initialized = false;
            shadingCorrectionOnCheckBox->setChecked(false);
            initialized = initSave;
            return;
        }
    }
    
    if(initialized && !newValue && shadingCorrectionShowImageCheckBox->isChecked()) {
        if(QMessageBox::warning(this, "Warning",
                                "'Show Image' is still selected...\nDisabling the Feature in this state\nwmay lead to a 'black' video-image!\n\nContinue?",
                                QMessageBox::No | QMessageBox::Default | QMessageBox::Escape,
                                QMessageBox::Yes,
                                QMessageBox::NoButton) == QMessageBox::No) {
            initialized = false;
            shadingCorrectionOnCheckBox->setChecked(true);
            initialized = initSave;
            return;
        }
    }
    
    if(initialized) {
        if(!dc1394->shadingCorrectionEnable(id, newValue)) {
            logger->log("AdjustmentsForm::shadingCorrectionOnChecked: Trying to %s 'Shading-Correction' failed!", newValue ? "enable" : "disable");
            initialized = false;
            shadingCorrectionOnCheckBox->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::shadingCorrectionShowImageChecked( bool newValue ) {
    bool initSave = initialized;
    
    
    if(initialized) {
        if(!dc1394->shadingCorrectionShowImage(id, newValue)) {
            logger->log("AdjustmentsForm::shadingCorrectionShowImageChecked: %s 'Shading-Image' failed", newValue ? "Show" : "Hide");
            initialized = false;
            shadingCorrectionShowImageCheckBox->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::shadingCorrectionBuildImgPressed() {
    struct timespec req, rem;
    bool busy, error = false;
    
    
    if(initialized && shadingCorrectionOnCheckBox->isChecked() &&
       shadingCorrectionShowImageCheckBox->isChecked()) {
        if(QMessageBox::warning(this, "Warning",
                                "Building a shading-image while displaying one may lead\nto a strange result, depending on the implementation.\nOn some camera-types a new shading image is generated\nand used showing the normal video-image, while the\n'Show Image' flag is still set.\n\nContinue?",
                                QMessageBox::Yes | QMessageBox::Default,
                                QMessageBox::No | QMessageBox::Escape,
                                QMessageBox::NoButton) == QMessageBox::No) {
            return;
        }
    }
    
    req.tv_sec = 0;
    req.tv_nsec = 500000000; // 500 ms
    
    if(QMessageBox::question(this, "Attention'",
                             "This feature should be used with max. resolution only!\n\nContinue?",
                             QMessageBox::Yes | QMessageBox::Default,
                             QMessageBox::No | QMessageBox::Escape,
                             QMessageBox::NoButton)
        == QMessageBox::No) {
        
        // just exit this function, if the changes should not be rejected
        return;
    }
    
    if(!dc1394->shadingCorrectionBuildImage(id, shadingCorrectionGrabCountSpinBox->value()))
        logger->log("AdjustmentsForm::shadingCorrectionBuildImgPressed: Failure while trying to build shading image");
    else {
        // disable shading-correction controls
        shadingCorrectionGroupBox->setEnabled(false);
        // change Mouse-Cursor to 'Busy'-Indication
        QApplication::setOverrideCursor(Qt::WaitCursor);
        // wait for process to be finished
        do {
            // wait ~500ms (see 'Marlin Technical Manual')
            if(nanosleep(&req, &rem) < 0) {
                logger->log("AdjustmentsForm::shadingCorrectionBuildImgPressed: 'sleep()' interrupted!");
            }
            
            // get the current state
            if(!dc1394->shadingCorrectionBusy(id, &busy)) {
                logger->log("AdjustmentsForm::shadingCorrectionBuildImgPressed: Error while trying to read Shading-Correction busy-state");
                error = true;
                break;
            }
            // and wait until the 'busy' flag is reset
        } while(busy);
        // chage Mouse-Cursor to normal arrow
        QApplication::restoreOverrideCursor();
        // enable shading-correction controls
        shadingCorrectionGroupBox->setEnabled(true);
        // set 'loaded' flag, because a valid shading image is now present
        shadingImgLoaded = true;
    }
    
    // because there's defenately a shading image availble - enable the 'OnCheckBox'
    updateUI();
}


void AdjustmentsForm::shadingCorrectionLoadImgPressed() {
    CamWindow *camWindow;
    QString fileName;
    QFile *file;
    QByteArray fileContents;
    bool lastOnState = shadingCorrectionOnCheckBox->isChecked();
    
    
    if((dc1394->getWidth(id) * dc1394->getHeight(id)) > maxShadingImgSize) {
        QMessageBox::warning(this, "Size-Conflict",
                             "The selected resolution exceeds the\nmax. Shading-Image-Buffer-Size!",
                             QMessageBox::Ok | QMessageBox::Default | QMessageBox::Escape,
                             QMessageBox::NoButton,
                             QMessageBox::NoButton);
        return;
    }
    
    if(QMessageBox::question(this, "Attention'",
                             QString("This feature should be used with max. resolution only!\nCurrent Resolution is: ") +
                             QString::number(dc1394->getWidth(id)) + "x" +
                             QString::number(dc1394->getHeight(id)) +
                             QString("\nContinue?"),
                             QMessageBox::Yes | QMessageBox::Default,
                             QMessageBox::No | QMessageBox::Escape,
                             QMessageBox::NoButton)
        == QMessageBox::No) {
        
        // just exit this function, if the changes should not be rejected
        return;
    }
    
    fileName = QFileDialog::getOpenFileName(QString::null, "*.sci", 0, 0, "Shading Image");
    
    if(fileName.isNull()) {
        return;
    }
    
    file = new QFile(fileName);
    
    if(!file->open(IO_Raw | IO_ReadOnly)) {
        QMessageBox::warning(this, "Error",
                             QString("Error while trying to open ") + fileName,
                             QMessageBox::Ok | QMessageBox::Default,
                             QMessageBox::NoButton,
                             QMessageBox::NoButton);
        delete file;
        return;
    }
    
    // check file size
    if(file->size() > maxShadingImgSize) {
        QMessageBox::warning(this, "Size-Conflict",
                             "The file you tried to load is bigger than\n'Max. Img. Size'!",
                             QMessageBox::Ok | QMessageBox::Default,
                             QMessageBox::NoButton,
                             QMessageBox::NoButton);
        logger->log("AdjustmentsForm::shadingCorrectionLoadImgPressed: File-Size: %d, max. Img. Size: %d",
               (unsigned int)file->size(), maxShadingImgSize);
        file->close();
        delete file;
        return;
    }
    
    if(file->size() > (dc1394->getWidth(id) * dc1394->getHeight(id))) {
        if(QMessageBox::warning(this, "Size-Conflict",
                                "The file you are trying to load is bigger\n than the current resolution!\n\nContinue anyway?",
                                QMessageBox::No | QMessageBox::Default | QMessageBox::Escape,
                                QMessageBox::Yes,
                                QMessageBox::NoButton) == QMessageBox::No) {
            logger->log("AdjustmentsForm::shadingCorrectionLoadImgPressed: File-Size: %d, max. Img. Size: %d",
                   (unsigned int)file->size(), maxShadingImgSize);
            file->close();
            delete file;
            return;
        }
    }
    
    if(file->size() < (dc1394->getWidth(id) * dc1394->getHeight(id))) {
        if(QMessageBox::warning(this, "Size-Conflict",
                                "The file you are trying to load is smaller\n than the current resolution!\n\nContinue anyway?",
                                QMessageBox::No | QMessageBox::Default | QMessageBox::Escape,
                                QMessageBox::Yes,
                                QMessageBox::NoButton) == QMessageBox::No) {
            logger->log("AdjustmentsForm::shadingCorrectionLoadImgPressed: File-Size: %d, max. Img. Size: %d",
                   (unsigned int)file->size(), maxShadingImgSize);
            file->close();
            delete file;
            return;
        }
    }
    
    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
    fileContents = file->readAll();
    QApplication::restoreOverrideCursor();
    
    if(fileContents.size() == 0) {
        QMessageBox::warning(this, "File empty",
                             "The file is empty!",
                             QMessageBox::Ok | QMessageBox::Default,
                             QMessageBox::NoButton,
                             QMessageBox::NoButton);
        
        file->close();
        delete file;
        return;
    }
    
    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
    
    if(QString(parent->className()) != "CamWindow") {
        QApplication::restoreOverrideCursor();
        return;
    }
    else {
        camWindow  = (CamWindow *)parent;
    }
    
    if(!camWindow->stopCapture()) {
        logger->log("AdjustmentsForm::shadingCorrectionLoadImgPressed: Error while trying to stop capture...");
        QApplication::restoreOverrideCursor();
    }
    
    // transfer the image into the camera
    if(!dc1394->shadingCorrectionLoadImage(id, (unsigned char *)fileContents.data(),
                                           file->size())) {
        logger->log("AdjustmentsForm::shadingCorrectionLoadImgPressed: Error while trying to load shading image of size %d into the camera",
               (unsigned int)file->size());
        file->close();
        delete file;
        QApplication::restoreOverrideCursor();
        camWindow->startCapture();
        return;
    }
    
    
    file->close();
    delete file;
    
    shadingImgLoaded = true;
    
    camWindow->startCapture();
    
    if(!dc1394->shadingCorrectionEnable(id, lastOnState)) {
        logger->log("AdjustmentsForm::shadingCorrectionLoadImgPressed: Error while trying to %s shading correction",
               lastOnState ? "enable" : "disable");
    }
    
    updateUI();
    
    QApplication::restoreOverrideCursor();
}


void AdjustmentsForm::shadingCorrectionGrabCountChanged( int newValue ) {
    bool initSave = initialized;
    bool increment = (unsigned int)newValue > grabCount;
    int correctedValue = newValue;
    
    
    // adjust 'GrabCount' value to a power of 2
    for(unsigned char i = 0x80; i >= 1; i >>= 1) {
        if(((unsigned char) correctedValue & i) != 0) {
            if(increment) {
                if(correctedValue > i) {
                    correctedValue = i << 1;
                }
                else {
                    correctedValue = i;
                }
            }
            else {
                if(correctedValue < i) {
                    correctedValue = i >> 1;
                }
                else {
                    correctedValue = i;
                }
            }
            break;
        }
    }
    
    if(correctedValue > shadingCorrectionGrabCountSpinBox->maxValue()) {
        logger->log("AdjustmentsForm::shadingCorrectionGrabCountChanged: corrected Value = %d, max. Value = %d - correcting to %d",
                    correctedValue,
                    shadingCorrectionGrabCountSpinBox->maxValue(),
                    correctedValue >> 1);
        correctedValue >>= 1;
    }
    
    if(newValue != correctedValue) {
        initialized = false;
        shadingCorrectionGrabCountSpinBox->setValue(correctedValue);
        initialized = initSave;
        
        logger->log("AdjustmentsForm::shadingCorrectionGrabCountChanged: GrabCount has changed - before: %d, now: %d",
                    grabCount, correctedValue);
    }
    
    // GrabCount is a class-variable used with 'DC1394::shadingCorrectionBuildImage()'
    grabCount = correctedValue;
    
    if(initialized) {
        if(!dc1394->shadingCorrectionSetGrabCount(id, grabCount)) {
            logger->log("AdjustmentsForm::shadingCorrectionGrabCountChanged: Changing 'Shading-Correction'-GrabCount value to %d failed!", grabCount);
        }
    }
    
    updateUI();
}


void AdjustmentsForm::lutOnChecked( bool newValue ) {
    bool initSave = initialized;
    
    
    if(lutOnCanceled) {
        lutOnCanceled = false;
        return;
    }
    
    if(newValue && !lutLoaded) {
        if(QMessageBox::warning(this, "Warning",
                                "No LUT was loaded since the application was started.\nThis may lead to a strange video-image.\nOn some camera-types the default LUT is a 'Gamma-correction'-LUT.\nUsed in conjunction with the standard Gamma-correction this may\nlead to a 'double correction', depending on the camera-type.\n\nContinue?",
                                QMessageBox::Yes | QMessageBox::Default,
                                QMessageBox::No | QMessageBox::Escape,
                                QMessageBox::NoButton) == QMessageBox::No) {
            lutOnCanceled = true;
            lutOnCheckBox->setChecked(false);
            return;
        }
    }
    
    if(initialized && gammaOnCheckBox->isChecked()) {
        if(newValue) {
            if(QMessageBox::warning(this, "Gamma uses LUT",
                                    "Please Note that some camera-types use the LUT for 'Gamma-Correction'.\nIn this case changing the LUT content will 'virtually' deactivate the 'Gamma-Correction',\nwhile the Feature is still activated!\n\nContinue?",
                                    QMessageBox::Yes | QMessageBox::Default,
                                    QMessageBox::No | QMessageBox::Escape,
                                    QMessageBox::NoButton) == QMessageBox::No) {
                lutOnCanceled = true;
                lutOnCheckBox->setChecked(false);
                return;
            }
        }
        else {
            if(QMessageBox::warning(this, "Warning",
                                    "Please Note that (depending on the camera-type) disabling the LUT may\nreturn to a 'linear' LUT. Thereofore Gamma may also be (virtually) disabled,\nwhile it stays activated! Disabling Gamma afterwards may have no\neffects on the video-image.\n\nContinue?",
                                    QMessageBox::Yes | QMessageBox::Default,
                                    QMessageBox::No | QMessageBox::Escape,
                                    QMessageBox::NoButton) == QMessageBox::No) {
                lutOnCanceled = true;
                lutOnCheckBox->setChecked(true);
                return;
            }
        }
    }
    
    if(!dc1394->lutEnable(id, newValue)) {
        logger->log("AdjustmentsForm::lutOnChecked: Error while trying to %s LUT",
                    newValue ? "enable" : "disable");
        initialized = false;
        lutOnCheckBox->setChecked(!newValue);
        initialized = initSave;
    }
    
    updateUI();
}


void AdjustmentsForm::lutLoadPressed() {
    QString fileName;
    QFile *file;
    QByteArray fileContents;
    unsigned int maxLutNo, maxLutSize;
    bool lutActivated, on;
    QFileDialog *openDialog = new QFileDialog(this);
    QString filters("Lookup Tables (*.lut);;"
                    "Any Files (*)");
    
    
    fileName = QFileDialog::getOpenFileName(QString::null, "*.lut");
    
    if(fileName.isNull() || fileName.isEmpty() || (fileName == "/")) {
        logger->log("Nothing selected!");
        delete openDialog;
        return;
    }
    logger->log("%s selected", fileName.ascii());
    
    file = new QFile(fileName);
    
    if(!file->open(IO_Raw | IO_ReadOnly)) {
        QMessageBox::warning(this, "Error",
                             QString("Error while tying to open " + fileName),
                             QMessageBox::Ok | QMessageBox::Default,
                             QMessageBox::NoButton,
                             QMessageBox::NoButton);
        delete openDialog;
        delete file;
        return;
    }
    
    // check size of file-contents against the maximum size
    if(file->size() > maxLUTSize) {
        QMessageBox::warning(this, "File too big...",
                             "The file you tried to load is bigger than \n'Max.LUT Size'!",
                             QMessageBox::Ok | QMessageBox::Default,
                             QMessageBox::NoButton,
                             QMessageBox::NoButton);
        file->close();
        delete openDialog;
        delete file;
        return;
    }
    else{
        QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
        fileContents = file->readAll();
        QApplication::restoreOverrideCursor();
        
        if(fileContents.size() == 0) {
            QMessageBox::warning(this, "File empty",
                                 "The file is empty!",
                                 QMessageBox::Ok | QMessageBox::Default,
                                 QMessageBox::NoButton,
                                 QMessageBox::NoButton);
            
            file->close();
            delete file;
            delete openDialog;
            return;
        }
        
        if(fileContents.isNull()) {
            logger->log("AdjustmentsForm::lutOnChecked: File contains no data!");
            file->close();
            delete file;
            delete openDialog;
            return;
        }
        
        lutActivated = lutOnCheckBox->isChecked();
        
        // transfer the image into the camera
        if(!dc1394->loadLUT(id, (unsigned char *)fileContents.data(),
                            file->size(), lutNumberSpinBox->value() - 1)) {
            logger->log("AdjustmentsForm::lutOnChecked: Error while trying to load LUT into the camera");
            logger->log("\tLut-Size = %u", (unsigned int)file->size());
            
            file->close();
            delete file;
            delete openDialog;
            return;
        }
        
        lutLoaded = true;
        lutChanged = false;
        
        if(!dc1394->getLUTInfos(id, &maxLutNo, &maxLutSize, &on)) {
            logger->log("AdjustmentsForm::lutOnChecked: Error while trying to read current LUT status");
        }
        else {
            if(lutActivated != on) {
                lutChanged = true;
            }
        }
    }
    
    file->close();
    delete file;
    delete openDialog;
    
    if(lutActivated && !on) {
        featuresApply();
    }
    else {
        updateUI();
    }
}


void AdjustmentsForm::hdrOnChecked( bool newValue ) {
    bool initSave = initialized;
    bool error = false;
    unsigned int maxKnees, knee1, knee2, knee3, activeKnees;
    bool on;
    
    
    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
    
    if(initialized) {
        if(dc1394->hdrEnable(id, newValue)) {
            // and wait for state to be changed
            do {
                sleep(1);
                if(!dc1394->getHDRInfos(id, &maxKnees, &knee1, &knee2, &knee3,
                                        &activeKnees, &on)) {
                    error = true;
                    break;
                }
            } while(on != newValue);
            
            if(error) {
                logger->log("AdjustmentsForm::hdrOnChecked: Error while trying to read the changed HDR state!");
                QApplication::restoreOverrideCursor();
                return;
            }
        }
        else {
            logger->log("AdjustmentsForm::hdrOnChecked: Error while trying to %s HDR!",
                        newValue ? "enable" : "disable");
            initialized = false;
            hdrOnCheckBox->setChecked(!newValue);
            initialized = initSave;
        }
        
        updateUI();
    }
    
    QApplication::restoreOverrideCursor();
}


void AdjustmentsForm::hdrKnee1SliderChanged(int newValue) {
    bool initSave = initialized;
    unsigned int numKnees = hdrNumKneesSpinBox->value();
    
    
    if(initialized) {
        initialized = false;
        hdrKnee1SpinBox->setValue(newValue);
        initialized = initSave;
        
        // then set value
        if(!dc1394->hdrSetValues(id,
                                 numKnees,
                                 newValue,
                                 hdrKnee2SpinBox->value(),
                                 hdrKnee3SpinBox->value())) {
            logger->log("AdjustmentsForm::hdrKnee1SliderChanged: Error while trying to change Knee-value!");
        }
    }
}


void AdjustmentsForm::hdrKnee1SpinBoxChanged(int newValue) {
    bool initSave = initialized;
    unsigned int numKnees = hdrNumKneesSpinBox->value();
    
    
    if(initialized) {
        initialized = false;
        hdrKnee1Slider->setValue(newValue);
        initialized = initSave;
        
        // then set value
        if(!dc1394->hdrSetValues(id,
                                 numKnees,
                                 newValue,
                                 hdrKnee2SpinBox->value(),
                                 hdrKnee3SpinBox->value())) {
            logger->log("AdjustmentsForm::hdrKnee1SpinBoxChanged: Error while trying to change Knee-value!");
        }
    }
}


void AdjustmentsForm::hdrKnee2SliderChanged(int newValue) {
    bool initSave = initialized;
    unsigned int numKnees = hdrNumKneesSpinBox->value();
    
    
    if(initialized) {
        initialized = false;
        hdrKnee2SpinBox->setValue(newValue);
        initialized = initSave;
        
        // then set value
        if(!dc1394->hdrSetValues(id,
                                 numKnees,
                                 hdrKnee1SpinBox->value(),
                                 newValue,
                                 hdrKnee3SpinBox->value())) {
            logger->log("AdjustmentsForm::hdrKnee2SliderChanged: Error while trying to change Knee-value!");
        }
    }
}


void AdjustmentsForm::hdrKnee2SpinBoxChanged(int newValue) {
    bool initSave = initialized;
    unsigned int numKnees = hdrNumKneesSpinBox->value();
    
    
    if(initialized) {
        initialized = false;
        hdrKnee2Slider->setValue(newValue);
        initialized = initSave;
        
        // then set value
        if(!dc1394->hdrSetValues(id,
                                 numKnees,
                                 hdrKnee1SpinBox->value(),
                                 newValue,
                                 hdrKnee3SpinBox->value())) {
            logger->log("AdjustmentsForm::hdrKnee2SpinBoxChanged: Error while trying to change Knee-value!");
        }
    }
}


void AdjustmentsForm::hdrKnee3SliderChanged(int newValue) {
    bool initSave = initialized;
    unsigned int numKnees = hdrNumKneesSpinBox->value();
    
    
    if(initialized) {
        initialized = false;
        hdrKnee3SpinBox->setValue(newValue);
        initialized = initSave;
        
        // then set value
        if(!dc1394->hdrSetValues(id,
                                 numKnees,
                                 hdrKnee1SpinBox->value(),
                                 hdrKnee2SpinBox->value(),
                                 newValue)) {
            logger->log("AdjustmentsForm::hdrKnee3SliderChanged: Error while trying to change Knee-value!");
        }
    }
}


void AdjustmentsForm::hdrKnee3SpinBoxChanged(int newValue) {
    bool initSave = initialized;
    unsigned int numKnees = hdrNumKneesSpinBox->value();
    
    
    if(initialized) {
        initialized = false;
        hdrKnee3Slider->setValue(newValue);
        initialized = initSave;
        
        // then set value
        if(!dc1394->hdrSetValues(id,
                                 numKnees,
                                 hdrKnee1SpinBox->value(),
                                 hdrKnee2SpinBox->value(),
                                 newValue)) {
            logger->log("AdjustmentsForm::hdrKnee3SpinBoxChanged: Error while trying to change Knee-value!");
        }
    }
}


void AdjustmentsForm::hdrNumKneesSpinBoxChanged(int newValue) {
    if(initialized) {
        if(!dc1394->hdrSetValues(id, newValue,
                                 hdrKnee1SpinBox->value(),
                                 hdrKnee2SpinBox->value(),
                                 hdrKnee3SpinBox->value())) {
            logger->log("AdjustmentsForm::hdrNumKneesSpinBoxChanged: Error while trying to change number of knees!");
        }
        
        updateUI();
    }
}


void AdjustmentsForm::hdrPreset1Pressed() {
    unsigned int knee1, knee2, knee3;
    bool valid;
    
    
    if(!dc1394->getExposureTime(id, &expTime, &valid)) {
        logger->log("AdjustmentsForm::hdrPreset1Pressed: Error while trying to read Exposure-Time!");
        return;
    }
    
    knee1 = (expTime * 15) / 100;
    knee2 = (expTime * 10) / 100;
    knee3 = (expTime * 5) / 100;
    
    if(!dc1394->hdrSetValues(id,
                             hdrNumKneesSpinBox->value(),
                             knee1,
                             knee2,
                             knee3)) {
        logger->log("AdjustmentsForm::hdrPreset1Pressed: Error while trying to change values");
    }
    
    updateUI();
}


void AdjustmentsForm::hdrPreset2Pressed() {
    bool valid;
    
    
    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
    
    if(!dc1394->getExposureTime(id, &expTime, &valid)) {
        logger->log("AdjustmentsForm::hdrPreset2Pressed: Error while trying to read Exposure-Time!");
        return;
    }
    
    if(!dc1394->hdrSetValues(id,
                             hdrNumKneesSpinBox->value(),
                             (expTime * 10) / 100,
                             (expTime * 5) / 100,
                             (expTime * 25) / 1000)) {
        logger->log("AdjustmentsForm::hdrPreset2Pressed: Error while trying to change values");
    }
    
    QApplication::restoreOverrideCursor();
    
    updateUI();
}


void AdjustmentsForm::hdrPreset3Pressed() {
    bool valid;
    
    
    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
    
    if(!dc1394->getExposureTime(id, &expTime, &valid)) {
        logger->log("AdjustmentsForm::hdrPreset3Pressed: Error while trying to read Exposure-Time!");
        return;
    }
    
    if(!dc1394->hdrSetValues(id,
                             hdrNumKneesSpinBox->value(),
                             (expTime * 5) / 100,
                             (expTime * 25) / 1000,
                             (expTime * 125) / 10000)) {
        logger->log("AdjustmentsForm::hdrPreset3Pressed: Error while trying to change values");
    }
    
    QApplication::restoreOverrideCursor();
    
    updateUI();
}


void AdjustmentsForm::hdrResetPressed() {
    if(!dc1394->hdrSetValues(id,
                             hdrNumKneesSpinBox->value(),
                             hdrKnee1SpinBox->minValue(),
                             hdrKnee2SpinBox->minValue(),
                             hdrKnee3SpinBox->minValue())) {
        logger->log("AdjustmentsForm::hdrResetPressed: Error while trying to reset HDR-values!");
    }
    
    updateUI();
}


void AdjustmentsForm::timebaseChanged(const QString &newString) {
    CamWindow *camWindow;
    QString timebaseString = newString;
    unsigned int timebase;
    
    if(initialized) {
        QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
        
        if(QString(parent->className()) != "CamWindow") {
            QApplication::restoreOverrideCursor();
            return;
        }
        else {
            camWindow  = (CamWindow *)parent;
        }
        
        if(!featureSlider[FEATURE_SHUTTER - FEATURE_MIN]->isEnabled()) {
            logger->log("AdjustmentsForm::timebaseChanged: Std.Shutter not available!");
            QApplication::restoreOverrideCursor();
            return;
        }
        
        if(!camWindow->stopCapture()) {
            logger->log("AdjustmentsForm::timebaseChanged: Error while trying to stop capturing");
            QApplication::restoreOverrideCursor();
            return;
        }
        
        timebaseString.replace("us", "");
        timebase = timebaseString.toUInt();
        
        if(!dc1394->setTimebase(id, timebase)) {
            logger->log("AdjustmentsForm::timebaseChanged: Trying to change Timebase to %s failed!", newString.ascii());
        }
        
        // for getting the timebase-change take effect, the shutter-value has to be set again...
        if(!dc1394->setFeatureValue(id,
                                    FEATURE_SHUTTER - FEATURE_MIN,
                                    shutterSpeedSpinBox->value())) {
            logger->log("AdjustmentsForm::timebaseChanged: Trying to (re)set SHUTTER failed!");
        }
        
        camWindow->startCapture();
        QApplication::restoreOverrideCursor();
    
    }
    
    updateUI();
}


void AdjustmentsForm::timebaseChanged(int) {
    updateUI();
}


void AdjustmentsForm::f7AOIWidthSliderChanged(int newValue)  {
    unsigned int maxXSize = f7AOIImageFrame->width();
    unsigned int maxYSize = f7AOIImageFrame->height();
    unsigned int maxX = f7AOIWidthSlider->maxValue();
    unsigned int h = f7AOIHeightSlider->value();
    unsigned int maxY = f7AOIHeightSlider->maxValue();
    unsigned int hUnit, vUnit, hPosUnit, vPosUnit;
    unsigned int diff;
    
    if(f7CorrectWidthSliderValue) {
        logger->log("F7CorrectSliderValue == true");
        f7CorrectWidthSliderValue = false;
        return;
    }
    
    if(!dc1394->getF7Units(id, &hUnit, &vUnit, &hPosUnit, &vPosUnit)) {
        logger->log("AdjustmentsForm::f7AOIWidthSliderChanged: Unable to read Unit-Values from camera");
    }
    else {
        if((diff = newValue % hUnit) != 0) {
            logger->log("AdjustmentsForm::f7AOIWidthSliderChanged: New value (%d) does not match current 'units' (remainder: %d) - fixing to '%d'",
                   newValue, newValue % hUnit, (newValue / hUnit) * hUnit);
            newValue -= diff;
            
            f7CorrectWidthSliderValue = true;
            f7AOIWidthSlider->setValue(newValue);
        }
    }
    
    f7AOISectorFrame->resize((maxXSize - 6) * newValue / maxX, (maxYSize - 6) * h / maxY);
    f7AOIXSlider->setMaxValue(maxX - newValue);

    if(initF7AOIWidthSlider > 0) {
        f7AOIWidthSpinBox->setValue(newValue);
        
        initF7AOIWidthSlider--;
        
        return;
    }
    
    f7AOIWidthSpinBox->setValue(newValue);
    
    if(initialized) {
        // enable the apply- and discard-button
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        // set feature-changed-flag
        f7Changed = true;
    }
}


void AdjustmentsForm::f7AOIWidthSpinBoxChanged(int newValue)  {
    unsigned int maxXSize = f7AOIImageFrame->width();
    unsigned int maxYSize = f7AOIImageFrame->height();
    unsigned int maxX = f7AOIWidthSpinBox->maxValue();
    unsigned int h = f7AOIHeightSpinBox->value();
    unsigned int maxY = f7AOIHeightSpinBox->maxValue();
    unsigned int hUnit, vUnit, hPosUnit, vPosUnit;
    unsigned int diff;
    
    if(f7CorrectWidthSpinBoxValue) {
        f7CorrectWidthSpinBoxValue = false;
        return;
    }
    
    if(!dc1394->getF7Units(id, &hUnit, &vUnit, &hPosUnit, &vPosUnit)) {
        logger->log("AdjustmentsForm::f7AOIWidthSpinBoxChanged: Unable to read Unit-Values from camera");
    }
    else {
        if((diff = newValue % hUnit) != 0) {
            newValue -= diff;
            
            f7CorrectWidthSpinBoxValue = true;
            f7AOIWidthSpinBox->setValue(newValue);
        }
    }
    
    f7AOISectorFrame->resize((maxXSize - 6) * newValue / maxX, (maxYSize - 6) * h / maxY);
    f7AOIXSpinBox->setMaxValue(maxX - newValue);

    if(initF7AOIWidthSpinBox > 0) {
        f7AOIWidthSlider->setValue(newValue);
        
        initF7AOIWidthSpinBox--;
        
        return;
    }
    
    f7AOIWidthSlider->setValue(newValue);
    
    if(initialized) {
        // enable the apply- and discard-button
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        // set feature-changed-flag
        f7Changed = true;
    }
}


// vvvvvvvvvvvvvvvvvvvvvvvvvvvv
void AdjustmentsForm::f7AOIHeightSliderChanged(int newValue)  {
    unsigned int maxXSize = f7AOIImageFrame->width();
    unsigned int maxYSize = f7AOIImageFrame->height();
    unsigned int w = f7AOIWidthSlider->value();
    unsigned int maxX = f7AOIWidthSlider->maxValue();
    unsigned int maxY = f7AOIHeightSlider->maxValue();
    unsigned int hUnit, vUnit, hPosUnit, vPosUnit;
    unsigned int diff;
    
    if(f7CorrectHeightSliderValue) {
        f7CorrectHeightSliderValue = false;
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
            
            f7CorrectHeightSliderValue = true;
            f7AOIHeightSlider->setValue(newValue);
        }
    }
    
    f7AOISectorFrame->resize((maxXSize - 6) * w / maxX, (maxYSize - 6) * newValue / maxY);
    f7AOIYSlider->setMaxValue(maxY - newValue);

    if(initF7AOIHeightSlider > 0) {
        f7AOIHeightSpinBox->setValue(newValue);
        
        logger->log("F7 AOI Height Slider-Value initialized - set both slider and spin-box to %d",
               newValue);
        
        initF7AOIHeightSlider--;
        
        return;
    }
    
    logger->log("F7 AOI Height slider manually changed to %d", newValue);
    
    f7AOIHeightSpinBox->setValue(newValue);
    
    if(initialized) {
        logger->log("F7AOIHeightSlider: Enabling 'Apply' and 'Discard'...");
        // enable the apply- and discard-button
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        // set feature-changed-flag
        f7Changed = true;
    }
}


void AdjustmentsForm::f7AOIHeightSpinBoxChanged(int newValue)  {
    unsigned int maxXSize = f7AOIImageFrame->width();
    unsigned int maxYSize = f7AOIImageFrame->height();
    unsigned int w = f7AOIWidthSpinBox->value();
    unsigned int maxX = f7AOIWidthSpinBox->maxValue();
    unsigned int maxY = f7AOIHeightSpinBox->maxValue();
    unsigned int hUnit, vUnit, hPosUnit, vPosUnit;
    unsigned int diff;
    
    
    if(f7CorrectHeightSpinBoxValue) {
        f7CorrectHeightSpinBoxValue = false;
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
            
            f7CorrectHeightSpinBoxValue = true;
            f7AOIHeightSpinBox->setValue(newValue);
        }
    }
    
    f7AOISectorFrame->resize((maxXSize - 6) * w / maxX, (maxYSize - 6) * newValue / maxY);
    f7AOIYSpinBox->setMaxValue(maxY - newValue);

    if(initF7AOIHeightSpinBox > 0) {
        f7AOIHeightSlider->setValue(newValue);
        
        logger->log("F7 AOI Height SpinBox-Value initialized - set both slider and spin-box to %d",
               newValue);
        
        initF7AOIHeightSpinBox--;
        
        return;
    }
    
    logger->log("F7 AOI Height spin-box manually changed to %d", newValue);
    
    f7AOIHeightSlider->setValue(newValue);
    
    if(initialized) {
        logger->log("F7AOIHeightSpinBox: Enabling 'Apply' and 'Discard'...");
        // enable the apply- and discard-button
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        // set feature-changed-flag
        f7Changed = true;
    }
}


void AdjustmentsForm::f7AOIXSliderChanged(int newValue)  {
    unsigned int maxXSize = f7AOIImageFrame->width();
    unsigned int maxYSize = f7AOIImageFrame->height();
    unsigned int maxX = f7AOIWidthSlider->maxValue();
    unsigned int y = f7AOIYSlider->value();
    unsigned int maxY = f7AOIHeightSlider->maxValue();
    unsigned int hUnit, vUnit, hPosUnit, vPosUnit;
    unsigned int diff;
    
    
    if(f7CorrectXSliderValue) {
        f7CorrectXSliderValue = false;
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
            
            f7CorrectXSliderValue = true;
            f7AOIXSlider->setValue(newValue);
        }
    }
    
    f7AOISectorFrame->move((newValue * (maxXSize - 3) / maxX) + 3,
                           (y * (maxYSize - 3) / maxY) + 3);

    if(initF7AOIXSlider > 0) {
        f7AOIXSpinBox->setValue(newValue);
        
        logger->log("F7 AOI X Slider-Value initialized - set both slider and spin-box to %d",
               newValue);
        
        initF7AOIXSlider--;
        
        return;
    }
    
    logger->log("F7 AOI X slider manually changed to %d", newValue);
    
    f7AOIXSpinBox->setValue(newValue);
    
    if(initialized) {
        logger->log("F7AOIXSlider: Enabling 'Apply' and 'Discard'...");
        // enable the apply- and discard-button
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        // set feature-changed-flag
        f7Changed = true;
    }
}


void AdjustmentsForm::f7AOIXSpinBoxChanged(int newValue)  {
    unsigned int maxXSize = f7AOIImageFrame->width();
    unsigned int maxYSize = f7AOIImageFrame->height();
    unsigned int maxX = f7AOIWidthSpinBox->maxValue();
    unsigned int y = f7AOIYSpinBox->value();
    unsigned int maxY = f7AOIHeightSpinBox->maxValue();
    unsigned int hUnit, vUnit, hPosUnit, vPosUnit;
    unsigned int diff;
    
    
    if(f7CorrectXSpinBoxValue) {
        f7CorrectXSpinBoxValue = false;
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
            
            f7CorrectXSpinBoxValue = true;
            f7AOIXSpinBox->setValue(newValue);
        }
    }
    
    f7AOISectorFrame->move((newValue * (maxXSize - 3) / maxX) + 3,
                           (y * (maxYSize - 3) / maxY) + 3);

    if(initF7AOIXSpinBox > 0) {
        f7AOIXSlider->setValue(newValue);
        
        logger->log("F7 AOI X SpinBox-Value initialized - set both slider and spin-box to %d",
               newValue);
        
        initF7AOIXSpinBox--;
        
        return;
    }
    
    logger->log("F7 AOI X spin-box manually changed to %d", newValue);
    
    f7AOIXSlider->setValue(newValue);
    
    if(initialized) {
        logger->log("F7AOIXSpinBox: Enabling 'Apply' and 'Discard'...");
        // enable the apply- and discard-button
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        // set feature-changed-flag
        f7Changed = true;
    }
}


void AdjustmentsForm::f7AOIYSliderChanged(int newValue)  {
    unsigned int maxXSize = f7AOIImageFrame->width();
    unsigned int maxYSize = f7AOIImageFrame->height();
    unsigned int x = f7AOIXSlider->value();
    unsigned int maxX = f7AOIWidthSlider->maxValue();
    unsigned int maxY = f7AOIHeightSlider->maxValue();
    unsigned int hUnit, vUnit, hPosUnit, vPosUnit;
    unsigned int diff;
    
    
    if(f7CorrectYSliderValue) {
        f7CorrectYSliderValue = false;
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
            
            f7CorrectYSliderValue = true;
            f7AOIYSlider->setValue(newValue);
        }
    }
    
    f7AOISectorFrame->move((x * (maxXSize - 3) / maxX) + 3,
                           (newValue * (maxYSize - 3) / maxY) + 3);

    if(initF7AOIYSlider > 0) {
        f7AOIYSpinBox->setValue(newValue);
        
        logger->log("F7 AOI Y Slider-Value initialized - set both slider and spin-box to %d",
               newValue);
        
        initF7AOIYSlider--;
        
        return;
    }
    
    logger->log("F7 AOI Y slider manually changed to %d", newValue);
    
    f7AOIYSpinBox->setValue(newValue);
    
    if(initialized) {
        logger->log("F7AOIYSlider: Enabling 'Apply' and 'Discard'...");
        // enable the apply- and discard-button
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        // set feature-changed-flag
        f7Changed = true;
    }
}


void AdjustmentsForm::f7AOIYSpinBoxChanged(int newValue)  {
    unsigned int maxXSize = f7AOIImageFrame->width();
    unsigned int maxYSize = f7AOIImageFrame->height();
    unsigned int x = f7AOIXSpinBox->value();
    unsigned int maxX = f7AOIWidthSpinBox->maxValue();
    unsigned int maxY = f7AOIHeightSpinBox->maxValue();
    unsigned int hUnit, vUnit, hPosUnit, vPosUnit;
    unsigned int diff;
    
    
    if(f7CorrectYSpinBoxValue) {
        f7CorrectYSpinBoxValue = false;
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
            
            f7CorrectYSpinBoxValue = true;
            f7AOIYSpinBox->setValue(newValue);
        }
    }
    
    f7AOISectorFrame->move((x * (maxXSize - 3) / maxX) + 3,
                           (newValue * (maxYSize - 3) / maxY) + 3);

    if(initF7AOIYSpinBox > 0) {
        f7AOIYSlider->setValue(newValue);
        
        logger->log("F7 AOI Y SpinBox-Value initialized - set both slider and spin-box to %d",
               newValue);
        
        initF7AOIYSpinBox--;
        
        return;
    }
    
    logger->log("F7 AOI Y spin-box manually changed to %d", newValue);
    
    f7AOIYSlider->setValue(newValue);
    
    if(initialized) {
        logger->log("F7AOIYSpinBox: Enabling 'Apply' and 'Discard'...");
        // enable the apply- and discard-button
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        // set feature-changed-flag
        f7Changed = true;
    }
}


void AdjustmentsForm::f7AOIMaximize() {
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


void AdjustmentsForm::f7BPPSpinBoxChanged(int) {
    if(initialized) {
        f7BPPChanged = true;
        featuresApply();
    }
}


void AdjustmentsForm::f7UseRecBPP() {
    bool initSave = initialized;
    
    
    if((f7RecBPPLineEdit->text()).toUInt() != 0) {
        initialized = false;
        f7BPPSpinBox->setValue((f7RecBPPLineEdit->text()).toUInt());
        initialized = initSave;
        f7BPPChanged = true;
        featuresApply();
    }
}


void AdjustmentsForm::advTriggerDelayOnChecked(bool newValue) {
    bool initSave = initialized;
    
    
    logger->log("Adv. Trigger Delay %s", newValue ? "enabled" : "disabled");
    
    if(newValue && !triggerOnCheckBox->isChecked()) {
        switch(QMessageBox::warning(this, "Trigger", "Enabling Adv. Trigger-Delay has no further effect,\n because the Trigger is not enabled!", "OK", "Cancel", "Enable Trigger", 0, 1)) {
        case 1:
            // Cancel
            initSave = initialized;
            initialized = false;
            advTriggerDelayOnCheckBox->setChecked(false);
            initialized = initSave;
            return;
        case 2:
            // Enable Trigger
            triggerOnCheckBox->setChecked(true);
            break;
        default:
            // Ok
            break;
        }
    }            
    
    if(initialized) {
        if(!dc1394->advTriggerDelayEnable(id, newValue)) {
            logger->log("Error while trying to %s the feature", newValue ? "enable" : "disable");
            initialized = false;
            advTriggerDelayOnCheckBox->setChecked(!newValue);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::advTriggerDelaySliderChanged(int newValue)  {
    bool initSave = initialized;
    
    
    if(initialized) {
        initialized = false;
        advTriggerDelaySpinBox->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setAdvTriggerDelay(id, newValue))
            logger->log("Trying to change 'Adv. Trigger-Delay' failed!");
    }
}


void AdjustmentsForm::advTriggerDelaySpinBoxChanged(int newValue) {
    bool initSave = initialized;
    
    
    if(initialized) {
        initialized = false;
        advTriggerDelaySpinBox->setValue(newValue);
        initialized = initSave;
        
        if(!dc1394->setAdvTriggerDelay(id, newValue))
            logger->log("Trying to change 'Adv. Trigger-Delay' failed!");
    }
}


void AdjustmentsForm::extShutterRangeChanged(bool newValue) {
    logger->log("Ext. Shutter is %s", (newValue ? "enabled" : "disabled"));
    
    stdShutterGroupBox->setEnabled(!newValue);
    
    extShutterSlider->setEnabled(newValue);
    if(!newValue && extShutterSpinBox->isEnabled() && extShutterSpinBox->hasFocus()) {
        focusdata->next()->setFocus();
    }
    extShutterSpinBox->setEnabled(newValue);
    
    if(!newValue) {
        featureChanged[FEATURE_SHUTTER - FEATURE_MIN] = true;
        featuresApply();
        logger->log("ExtShutterRage: Enabling 'Apply' and 'Discard'...");
        applyPushButton->setEnabled(false);
        discardPushButton->setEnabled(false);
    }
    else {
        updateUI();
        
        initExtShutterSlider = 0;
        initExtShutterSpinBox = 0;
    }
}


// This control needs to be as is (i.e., using the 'Apply' / 'Discard' implementation)
// because the whole slider reflects 67s (i.e. 67.000.000 us), while allowing to
// change the value in 1us steps.
// Changing the behaviour to directly manipulate the camera does not makes sense,
// because every move would take an unacceptle period of time to wait for the action
// to be executed!
void AdjustmentsForm::extShutterSliderChanged(int newValue) {
    if(initExtShutterSlider > 0) {
        extShutterSpinBox->setValue(newValue);
        
        logger->log("ext. Shutter Slider-Value initialized - set both slider and spin-box to %d",
               newValue);
        
        initExtShutterSlider--;
        
        return;
    }
    
    logger->log("feature-slider manually changed to %d", newValue);
    
    extShutterSpinBox->setValue(newValue);
    
    if(initialized) {
        logger->log("ExtShutterSlider: Enabling 'Apply' and 'Discard'...");
        // enable the apply- and discard-button
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        // set feature-changed-flag
        extShutterChanged = true;
    }
}


void AdjustmentsForm::extShutterSpinBoxChanged(int newValue) {
    if(initExtShutterSpinBox > 0) {
        // initExtShutterSlider = 1;
        extShutterSlider->setValue(newValue);
        
        logger->log("ext. Shutter SpinBox-Value initialized - set both spin-box and slider to %d",
               newValue);
        
        initExtShutterSpinBox--;
        
        return;
    }
    
    logger->log("feature-spin-box manually changed to %d", newValue);
    extShutterSlider->setValue(newValue);
    
    if(initialized) {
        logger->log("ExtShutterSpinBox: Enabling 'Apply' and 'Discard'...");
        // enable the apply- and discard-button
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        // set feature-changed-flag
        extShutterChanged = true;
    }
}


void AdjustmentsForm::mirrorImageOnChecked(bool newValue) {
    bool initSave = initialized;
    
    
    if(initialized) {
        if(!dc1394->mirrorImageEnable(id, newValue)) {
            initialized = false;
            logger->log("Error while trying to %s mirroring!", (newValue ? "enable" : "disable"));
            mirrorImageOnCheckBox->setChecked(newValue ? false : true);
            initialized = initSave;
        }
    }
}


void AdjustmentsForm::testImageOnChecked(bool newValue) {
    bool initSave = initialized;
    QString testImageName;
    
    if(initialized) {
        logger->log("Test-Image switched %s", newValue ? "on" : "off");
        
        testImageName = testImageComboBox->text(testImageComboBox->currentItem());
        logger->log("Selection from ComboBox: %s", testImageName.ascii());
        testImageName.replace("Test Image", "");
        
        logger->log("Currently selected Test Image: %d", testImageName.toUInt());
        
        if(!dc1394->testImageEnable(id, newValue, testImageName.toUInt())) {
            initialized = false;
            logger->log("Error while trying to %s test-image %d!", (newValue ? "enable" : "disable"),
                        testImageName.toUInt());
            testImageOnCheckBox->setChecked(newValue ? false : true);
            initialized = initSave;
        }
    }
}


void AdjustmentsForm::testImageChanged(const QString& newImageName) {
    QString imageName;
    unsigned int selectedImage;
    
    if(testImageOnCheckBox->isChecked()) {
        logger->log("%s selected", newImageName.ascii());
        
        imageName = newImageName;
        imageName.replace("Test Image", "");
        
        logger->log("selecting test-image %d", imageName.toUInt());
        
        if(!dc1394->testImageSelect(id, imageName.toUInt())) {
            logger->log("Error while trying to select test-image %d!", imageName.toUInt());
            logger->log("trying to read current selection from camera");
            if(!dc1394->getTestImage(id, &selectedImage)) {
                logger->log("Error while trying to read selected test-image - disabling the whole feature!");
                dc1394->testImageEnable(id, false, 0);
                testImageOnCheckBox->setChecked(false);
                testImageOnCheckBox->setEnabled(false);
            }
            else {
                testImageComboBox->setCurrentText("Test Image " +
                                                  QString::number(selectedImage));
            }
        }
    }
}


void AdjustmentsForm::colorCorrOnChecked(bool newValue) {
    bool initSave = initialized;
    
    
    if(initialized) {
        if(!dc1394->colorCorrEnable(id, newValue)) {
            initialized = false;
            logger->log("Error while trying to %s color correction!",
                        (newValue ? "enable" : "disable"));
            colorCorrOnCheckBox->setChecked(!newValue);
            initialized = initSave;
        }
    }
}


void AdjustmentsForm::io1InModeChanged(const QString&) {
    int mode = determineInModeID(in1ModeComboBox->currentText());
    
    if(mode == -1) {
        logger->log("Unknown Mode");
        gpIO1GroupBox->setEnabled(false);
        return;
    }
    
    logger->log("In1:\tPolarity - %s\n\tMode: %s (%d)",
           (in1PolarityComboBox->currentText()).ascii(),
           (in1ModeComboBox->currentText()).ascii(),
           mode);
    
    if(!dc1394->setIOInCtrl(id, 1,
                            ((in1PolarityComboBox->currentText() == "low act.") ?
                             false : true),
                            mode)) {
        logger->log("Error while trying to change IO1 Input Mode");
    }
    
    updateIO(1, false);
}


void AdjustmentsForm::io1OutModeChanged(const QString&/* newText*/) {
    bool invertOut;
    bool state;
    bool camPolarity, camState;
    unsigned int camMode;
    int mode = determineOutModeID(out1ModeComboBox->currentText());
    
    
    if(mode == -1) {
        logger->log("Unknown Mode");
        return;
    }
    
    logger->log("Out1:\tPolarity - %s\n\tMode: %s (%d)\n\tState: %s",
                (out1PolarityComboBox->currentText()).ascii(),
                (out1ModeComboBox->currentText()).ascii(),
                mode,
                (out1StateComboBox->currentText()).ascii());
    
    invertOut = (out1PolarityComboBox->currentText() == "low act.");
    logger->log("%s Output", invertOut ? "Inverting" : "Non-inverting");
    
    state = (out1StateComboBox->currentText() == "active");
    
    if(!dc1394->setIOOutCtrl(id, 1,
                             invertOut ? false : true,
                             mode,
                             state)) {
        logger->log("Error while trying to change IO1 Output Parameters");
        return;
    }
    logger->log("Out-State set to %s",
           (state ? "true" : "false"));
    
    if(!dc1394->getIOOutCtrl(id, 1, &camPolarity, &camMode, &camState)) {
        logger->log("Error while trying to read current IO1-status");
        return;
    }
    
    logger->log("Output 1 controls after changes (Polarity, Mode, State): %s, %d, %s",
                camPolarity ? "high act." : "low act.",
                camMode,
                camState ? "active" : "inactive");
    
    updateIO(1, false);
}


void AdjustmentsForm::io2InModeChanged(const QString&) {
    int mode = determineInModeID(in2ModeComboBox->currentText());
    
    if(mode == -1) {
        logger->log("Unknown Mode");
        return;
    }
    
    logger->log("In2:\tPolarity - %s\n\tMode: %s (%d)",
           (in2PolarityComboBox->currentText()).ascii(),
           (in2ModeComboBox->currentText()).ascii(),
           mode);
    
    if(!dc1394->setIOInCtrl(id, 2,
                            ((in2PolarityComboBox->currentText() == "low act.") ?
                             false : true),
                            mode)) {
        logger->log("Error while trying to change IO2 Input Mode");
    }
    
    updateIO(2, false);
}


void AdjustmentsForm::io2OutModeChanged(const QString&) {
    bool invertOut;
    bool state;
    bool camPolarity, camState;
    unsigned int camMode;
    int mode = determineOutModeID(out2ModeComboBox->currentText());
    
    
    if(mode == -1) {
        logger->log("Unknown Mode");
        return;
    }
    
    logger->log("Out2:\tPolarity - %s\n\tMode: %s (%d)\n\tState: %s",
           (out2PolarityComboBox->currentText()).ascii(),
           (out2ModeComboBox->currentText()).ascii(),
           mode,
           (out2StateComboBox->currentText()).ascii());
    
    invertOut = out2PolarityComboBox->currentText() == "low act.";
    logger->log("%s Output", invertOut ? "Inverting" : "Non-inverting");
    
    state = out2StateComboBox->currentText() == "active";
    
    if(!dc1394->setIOOutCtrl(id, 2,
                             !invertOut,
                             mode,
                             state)) {
        logger->log("Error while trying to change IO2 Output Mode");
    }
    
    logger->log("Out-State set to %s",
           (state ? "true" : "false"));
    
    if(!dc1394->getIOOutCtrl(id, 2, &camPolarity, &camMode, &camState)) {
        logger->log("Error while trying to read current IO2-status");
        return;
    }
    
    updateIO(2, false);
}


void AdjustmentsForm::io3InModeChanged(const QString&) {
    int mode = determineInModeID(in3ModeComboBox->currentText());
    
    if(mode == -1) {
        logger->log("Unknown Mode");
        return;
    }
    
    logger->log("In3:\tPolarity - %s\n\tMode: %s (%d)",
           (in3PolarityComboBox->currentText()).ascii(),
           (in3ModeComboBox->currentText()).ascii(),
           mode);
    
    if(!dc1394->setIOInCtrl(id, 3,
                            ((in3PolarityComboBox->currentText() == "low act.") ?
                             false : true),
                            mode)) {
        logger->log("Error while trying to change IO2 Input Mode");
    }
    
    updateIO(3, false);
}


void AdjustmentsForm::io3OutModeChanged(const QString&) {
    bool invertOut;
    bool state;
    bool camPolarity, camState;
    unsigned int camMode;
    int mode = determineOutModeID(out3ModeComboBox->currentText());
    
    
    if(mode == -1) {
        logger->log("Unknown Mode");
        return;
    }
    
    logger->log("Out3:\tPolarity - %s\n\tMode: %s (%d)\n\tState: %s",
           (out3PolarityComboBox->currentText()).ascii(),
           (out3ModeComboBox->currentText()).ascii(),
           mode,
           (out3StateComboBox->currentText()).ascii());
    
    invertOut = out3PolarityComboBox->currentText() == "low act.";
    logger->log("%s Output", invertOut ? "Inverting" : "Non-inverting");
    
    state = out3StateComboBox->currentText() == "active";
    
    if(!dc1394->setIOOutCtrl(id, 3,
                             !invertOut,
                             mode,
                             state)) {
        logger->log("Error while trying to change IO3 Output Mode");
    }
    
    logger->log("Out-State set to %s",
           (state ? "true" : "false"));
    
    if(!dc1394->getIOOutCtrl(id, 3, &camPolarity, &camMode, &camState)) {
        logger->log("Error while trying to read current IO3-status");
        return;
    }
    
    updateIO(3, false);
}


int AdjustmentsForm::determineInModeID(QString mode) {
    logger->log("Trying to convert Input Mode '%s'...", mode.ascii());
    
    if(mode == IO_IN_OFF_STRING)
        return IO_IN_OFF;
    if(mode == IO_IN_TRIGGER_STRING)
        return IO_IN_TRIGGER;
    if(mode == IO_IN_INC_DEC_STRING)
        return IO_IN_INC_DEC;
    if(mode == IO_IN_SPI_STRING)
        return IO_IN_SPI;
    
    return -1;
}


int AdjustmentsForm::determineOutModeID(QString mode) {
    logger->log("Trying to convert Output Mode '%s'...", mode.ascii());
    
    if(mode == IO_OUT_OFF_STRING)
        return IO_OUT_OFF;
    if(mode == IO_OUT_PIN_STATE_STRING)
        return IO_OUT_PIN_STATE;
    if(mode == IO_OUT_INT_ENA_STRING)
        return IO_OUT_INT_ENA;
    if(mode == IO_OUT_INC_DEC_CMP_STRING)
        return IO_OUT_INC_DEC_CMP;
    if(mode == IO_OUT_SPI_INT_STRING)
        return IO_OUT_SPI_INT;
    if(mode == IO_OUT_SPI_EXT_STRING)
        return IO_OUT_SPI_EXT;
    if(mode == IO_OUT_FRAME_VALID_STRING)
        return IO_OUT_FRAME_VALID;
    if(mode == IO_OUT_BUSY_STRING)
        return IO_OUT_BUSY;
    if(mode == IO_OUT_CORR_IN_STRING)
        return IO_OUT_CORR_IN;
    
    return -1;
}


void AdjustmentsForm::serialBaudRateChanged(const QString& newRateName) {
    QString rateName = newRateName;
    unsigned int newRate, baud, len, par, stop, bufSize;
    
    rateName.replace(" bps", "");
    newRate = rateName.toUInt();
    
    logger->log("\t%d bps (String: %s) selected", newRate, newRateName.ascii());
    
    if(!dc1394->getSerialParameters(id, &baud, &len, &par, &stop, &bufSize)) {
        logger->log("Error while trying to read serial parameters - setting failed!");
        updateUI();
        return;
    }
    
    logger->log("\tnew parameters: baud = %d, len = %d, par = %d, stop = %d",
           newRate, len, par, stop);
    
    if(!dc1394->setSerialParameters(id, newRate, len, par, stop)) {
        logger->log("Failed to set serial parameters!");
        updateUI();
        return;
    }
    
    logger->log("\tBuadrate successfully changed");
}


void AdjustmentsForm::serialCharLenChanged(const QString& newCharLenName) {
    QString charLenName = newCharLenName;
    unsigned int newCharLen, baud, len, par, stop, bufSize;
    
    charLenName.replace(" bit", "");
    newCharLen = charLenName.toUInt();
    
    logger->log("\t%d bit characters (String: %s) selected", newCharLen, newCharLenName.ascii());
    
    if(!dc1394->getSerialParameters(id, &baud, &len, &par, &stop, &bufSize)) {
        logger->log("Error while trying to read serial parameters - setting failed!");
        updateUI();
        return;
    }
    
    logger->log("\tnew parameters: baud = %d, len = %d, par = %d, stop = %d",
           baud, newCharLen, par, stop);
    
    if(!dc1394->setSerialParameters(id, baud, newCharLen, par, stop)) {
        logger->log("Failed to set serial parameters!");
        updateUI();
        return;
    }
    
    logger->log("\tCharacter length successfully changed");
    
}


void AdjustmentsForm::serialParityChanged(const QString& newParityName) {
    unsigned int newParity, baud, len, par, stop, bufSize;
    
    if(newParityName == "None")
        newParity = SERIAL_NO_PARITY;
    else {
        if(newParityName == "Odd")
            newParity = SERIAL_ODD_PARITY;
        else {
            if(newParityName == "Even")
                newParity = SERIAL_EVEN_PARITY;
            else {
                logger->log("Unknown (and unsupported) selection (%s)!",
                       newParityName.ascii());
                updateUI();
                return;
            }
        }
    }
    
    logger->log("\tParity: '%s' (%d) selected", newParityName.ascii(), newParity);
    
    if(!dc1394->getSerialParameters(id, &baud, &len, &par, &stop, &bufSize)) {
        logger->log("Error while trying to read serial parameters - setting failed!");
        updateUI();
        return;
    }
    
    logger->log("\tnew parameters: baud = %d, len = %d, par = %d, stop = %d",
           baud, len, newParity, stop);
    
    if(!dc1394->setSerialParameters(id, baud, len, newParity, stop)) {
        logger->log("Failed to set serial parameters!");
        updateUI();
        return;
    }
    
    logger->log("\tParity successfully changed");
}


void AdjustmentsForm::serialStopbitsChanged(const QString& newStopbitsName) {
    unsigned int newStopbits, baud, len, par, stop, bufSize;
    
    if(newStopbitsName == "1")
        newStopbits = SERIAL_1_STOPBIT;
    else {
        if(newStopbitsName == "1.5")
            newStopbits = SERIAL_3_HALF_STOPBIT;
        else {
            if(newStopbitsName == "2")
                newStopbits = SERIAL_2_STOPBIT;
            else {
                logger->log("Unknown (and unsupported) selection!");
                updateUI();
                return;
            }
        }
    }
    
    logger->log("\t%s stopbits selected", newStopbitsName.ascii());
    
    if(!dc1394->getSerialParameters(id, &baud, &len, &par, &stop, &bufSize)) {
        logger->log("Error while trying to read serial parameters - setting failed!");
        updateUI();
        return;
    }
    
    logger->log("\tnew parameters: baud = %d, len = %d, par = %d, stop = %d",
           baud, len, par, newStopbits);
    
    if(!dc1394->setSerialParameters(id, baud, len, par, newStopbits)) {
        logger->log("Failed to set serial parameters!");
        updateUI();
        return;
    }
    
    logger->log("\tNumber of Stopbits successfully changed");
}


void AdjustmentsForm::sioDemo() {
/*
    SerialIOReceiveForm *receiveForm = new SerialIOReceiveForm();
    
    receiveForm->setFixedSize(receiveForm->frameSize());
    receiveForm->initForm(this, dc1394, id, logger);
    receiveForm->exec();
*/
    SIODemoForm *sioDemo = new SIODemoForm();
    
    sioDemo->setFixedSize(sioDemo->frameSize());
    sioDemo->initForm(this, dc1394, id, logger);
    sioDemo->exec();
}


/*
void AdjustmentsForm::serialTransmitButtonPressed() {
    SerialIOTransmitForm *transmitForm = new SerialIOTransmitForm();
    
    transmitForm->setFixedSize(transmitForm->frameSize());
    transmitForm->initForm(this, dc1394, id, logger);
    transmitForm->exec();
}
*/

void AdjustmentsForm::dsnuOnChecked(bool newValue) {
    bool initSave = initialized;
    
    
    if(initialized) {
        if(!dc1394->dsnuEnable(id, newValue)) {
            logger->log("Failed to enable DSNU!");
        }
        
        initialized = false;
        updateUI();
        initialized = initSave;
    }
}


void AdjustmentsForm::blemishOnChecked(bool newValue) {
    bool initSave = initialized;
    
    
    if(initialized) {
        if(!dc1394->blemishEnable(id, newValue)) {
            logger->log("Failed to enable Blemish!");
        }
        
        initialized = false;
        updateUI();
        initialized = initSave;
    }
}


void AdjustmentsForm::dsnuShowImgChecked(bool newValue) {
    bool initSave = initialized;
    bool dsnuAvailable  = dsnuOnCheckBox->isEnabled();
    bool blemishAvailable  = blemishOnCheckBox->isEnabled();
    
    
    if(initialized) {
        logger->log("\t%s DSNU", newValue ? "Enabling" : "Disabling");
        
        // if DSNU is available - try to set DSNU-'show-image'-flag
        if(dsnuAvailable) {
            if(!dc1394->dsnuShowImg(id, newValue)) {
                logger->log("Failed to %s current DSNU-Image!", newValue ? "enable" : "disable");
            }
        }
        
        // if Blemish is available - try to set Blemish-'show-image'-flag
        if(blemishAvailable) {
            if(!dc1394->blemishShowImg(id, newValue)) {
                logger->log("Failed to %s current Blemish image!", newValue ? "enable" : "disable");
            }
        }
        
        initialized = false;
        updateUI();
        initialized = initSave;
    }
}


void AdjustmentsForm::dsnuGrabCountChanged(int newValue) {
    bool initSave = initialized;
    bool dsnuAvailable = dsnuOnCheckBox->isEnabled();
    bool blemishAvailable = blemishOnCheckBox->isEnabled();
    
    
    if(initialized) {
        if(dsnuAvailable) {
            if(!dc1394->setDSNUImgCnt(id, newValue)) {
                logger->log("Changing DSNU-GrabCount value failed!");
            }
        }
        
        if(blemishAvailable) {
            if(!dc1394->setBlemishImgCnt(id, newValue)) {
                logger->log("Changing Blemish-GrabCount value failed!");
            }
        }
        
        initialized = false;
        updateUI();
        initialized = initSave;
    }
}


void AdjustmentsForm::dsnuCompute() {
    bool initSave = initialized;
    bool dsnuAvailable = dsnuOnCheckBox->isEnabled();
    bool blemishAvailable = blemishOnCheckBox->isEnabled();
    
    
    QMessageBox::information(this, "DSNU / Blemish",
                             "'Close' the lens of the camera (e.g. with the cover) in order\nto get usefull correction-images",
                             QMessageBox::Ok | QMessageBox::Default | QMessageBox::Escape,
                             QMessageBox::NoButton,
                             QMessageBox::NoButton);
                             
    if(dsnuAvailable && dsnuOnCheckBox->isChecked()) {
        if(!dc1394->dsnuComputeImg(id)) {
            logger->log("Failed to compute new DSNU image!");
            dsnuUsedImageLineEdit->setText("");
        }
        else {
            dsnuWaitBusy(5);
            logger->log("DSNU image computed");
            dsnuUsedImageLineEdit->setText("intern. computed");
        }
    }
    else {
        // !!! only one action needs to be executed !!!
        if(blemishAvailable && blemishOnCheckBox->isChecked()) {
            if(!dc1394->blemishComputeImg(id)) {
                logger->log("Failed to compute new Blemish image!");
                dsnuUsedImageLineEdit->setText("");
            }
            else {
                blemishWaitBusy(5);
                logger->log("Blemish image computed");
                dsnuUsedImageLineEdit->setText("intern. computed");
            }
        }
    }
    
    initialized = false;
    updateUI();
    initialized = initSave;
}


void AdjustmentsForm::dsnuReset() {
    bool initSave = initialized;
    bool dsnuAvailable = dsnuOnCheckBox->isEnabled();
    bool blemishAvailable = blemishOnCheckBox->isEnabled();
    
    
    if(dsnuAvailable && dsnuOnCheckBox->isChecked()) {
        if(!dc1394->dsnuLoadFactoryData(id)) {
            logger->log("Failed to reset image!");
            dsnuUsedImageLineEdit->setText("");
       }
        else {
            dsnuWaitBusy(5);
            logger->log("DSNU set to default");
            dsnuUsedImageLineEdit->setText("Factory setup");
        }
    }
    else {
        // !!! only one action needs to executed !!!
        if(blemishAvailable && blemishOnCheckBox->isChecked()) {
            if(!dc1394->blemishLoadFactoryData(id)) {
                logger->log("Failed to reset image!");
                dsnuUsedImageLineEdit->setText("");
            }
            else {
                blemishWaitBusy(5);
                logger->log("Blemish set to default");
                dsnuUsedImageLineEdit->setText("Factory setup");
            }
        }
    }
    
    initialized = false;
    updateUI();
    initialized = initSave;
}


void AdjustmentsForm::dsnuZero() {
    bool initSave = initialized;
    bool dsnuAvailable = dsnuOnCheckBox->isEnabled();
    bool blemishAvailable = blemishOnCheckBox->isEnabled();
    
    
    if(dsnuAvailable && dsnuOnCheckBox->isChecked()) {
        if(!dc1394->dsnuZeroImg(id)) {
            logger->log("Failed to set image to zero!");
            dsnuUsedImageLineEdit->setText("");
        }
        else {
            dsnuWaitBusy(5);
            logger->log("DSNU image set to zero");
            dsnuUsedImageLineEdit->setText("'Zero'");
        }
    }
    else {
        // only one action needs to be executed !!!
        if(blemishAvailable && blemishOnCheckBox->isChecked()) {
            if(!dc1394->blemishZeroImg(id)) {
                logger->log("Failed to set image to zero!");
                dsnuUsedImageLineEdit->setText("");
            }
            else {
                blemishWaitBusy(5);
                logger->log("Blemish image set to zero");
                dsnuUsedImageLineEdit->setText("'Zero'");
            }
        }
    }
    
    initialized = false;
    updateUI();
    initialized = initSave;
}


bool AdjustmentsForm::dsnuWaitBusy(unsigned int timeout) {
    unsigned int timeCount = 0;
    bool busy = true;
    
    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
    
    do{
        if(!dc1394->dsnuBusy(id, &busy)) {
            logger->log("Failed to read-out DSNU-Busy-Flag!");
        }
        
        sleep(1);
        timeCount++;
        
    }while((timeCount < timeout) && (busy));
    
    if(timeCount == timeout)
        logger->log("'dsnuWaitBusy()': %ds are maybe not enough!", timeout);
    else
        logger->log("'dsnuWaitBusy()': Waited for %ds", timeCount);
    
    QApplication::restoreOverrideCursor();
    
    return busy;
}


bool AdjustmentsForm::blemishWaitBusy(unsigned int timeout) {
    unsigned int timeCount = 0;
    bool busy = true;
    
    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
    
    do{
        if(!dc1394->blemishBusy(id, &busy)) {
            logger->log("Failed to read-out Blemish-Busy-Flag!");
        }
        
        sleep(1);
        timeCount++;
        
    }while((timeCount < timeout) && (busy));
    
    if(timeCount == timeout)
        logger->log("'blemishWaitBusy()': %ds are maybe not enough!", timeout);
    else
        logger->log("'blemish()': Waited for %ds", timeCount);
    
    QApplication::restoreOverrideCursor();
    
    return busy;
}


void AdjustmentsForm::captureModeChanged(const QString& newSelection) {
    unsigned int msFrameCount;
    
    
    //    captureModeFramesSpinBox->setValue(currentFrameCount);
    
    if(newSelection == CAPTURE_MODE_FREERUN_STRING) {
        if(currentCaptureMode != CAPTURE_MODE_FREERUN) {
            logger->log("Free-Run selected");
            captureModeStartPushButton->setEnabled(true);
            captureModeStartPushButton->setText("Start");
            captureModeFramesSpinBox->setEnabled(false);
            currentCaptureMode = CAPTURE_MODE_FREERUN;
        }
        else {
            logger->log("Free-Run currently active!");
        }
    }
    
    if(newSelection == CAPTURE_MODE_ONESHOT_STRING) {
        if(currentCaptureMode != CAPTURE_MODE_ONESHOT) {
            logger->log("One-Shot selected");
            if(!dc1394->enableFreerun(id, false)) {
                logger->log("Trying to disable 'FreeRun' failed!");
            }
            captureModeStartPushButton->setEnabled(true);
            captureModeStartPushButton->setText("Trigger");
            captureModeFramesSpinBox->setEnabled(false);
            currentCaptureMode = CAPTURE_MODE_ONESHOT;
        }
        else {
            logger->log("One-Shot currently active!");
        }
    } 
    
    if(newSelection == CAPTURE_MODE_MULTISHOT_STRING) {
        if(currentCaptureMode != CAPTURE_MODE_MULTISHOT) {
            logger->log("Multi-Shot selected");
            if(!dc1394->enableFreerun(id, false)) {
                logger->log("Trying to disable 'FreeRun' failed!");
            }
            if(!dc1394->getMultiShotFrameCount(id, &msFrameCount)) {
                logger->log("Error while trying to get current 'MultiShot-FrameCounter-Value'");
            }
            captureModeFramesSpinBox->setEnabled(true);
            captureModeFramesSpinBox->setValue(msFrameCount);
            captureModeStartPushButton->setEnabled(captureModeFramesSpinBox->value() > 0);
            captureModeStartPushButton->setText("Trigger");
            currentCaptureMode = CAPTURE_MODE_MULTISHOT;
        }
        else {
            logger->log("Multi-Shot currently active!");
        }
    }
}


void AdjustmentsForm::captureModeStart() {
    CamWindow *camWindow;
    
    if(QString(parent->className()) != "CamWindow") {
        logger->log("Parent of 'AdjustmentsForm' is '%s', but 'CamWindow' is expected!",
               parent->className());
        logger->log("Therefore a 'SW-Trigger' is not possible!");
        return;
    }
    else {
        camWindow  = (CamWindow *)parent;
    }
    
    logger->log("Starting '%s'...", (captureModeComboBox->currentText()).ascii());
    
    if(captureModeComboBox->currentText() == CAPTURE_MODE_FREERUN_STRING) {
        if(!dc1394->enableFreerun(id, true)) {
            logger->log("Error while trying to enable 'Freerun'!");
            return;
        }
        // only one 'Start'-Trigger for 'freerun-mode'
        captureModeStartPushButton->setEnabled(false);
    }
    
    if(captureModeComboBox->currentText() == CAPTURE_MODE_ONESHOT_STRING) {
        if(!dc1394->enableOneShot(id, true)) {
            logger->log("Error while trying to enable (start) 'OneShot'");
        }
    }
    
    if(captureModeComboBox->currentText() == CAPTURE_MODE_MULTISHOT_STRING) {
        if(!dc1394->enableMultiShot(id, true, captureModeFramesSpinBox->value())) {
            logger->log("Error while trying to enable (start) 'MultiShot'");
        }
    }
}


void AdjustmentsForm::captureModeFramesChanged(int newValue) {
    captureModeStartPushButton->setEnabled(newValue > 0);
}


void AdjustmentsForm::autoShutterMinSliderChanged(int newValue) {
    bool initSave = initialized;
    
    
    if(initialized) {
        if(!dc1394->setAutoShutterLimits(id, newValue, autoShutterMaxSpinBox->value())) {
            logger->log("Changing 'AutoShutter' limits failed!");
            updateUI();
            return;
        }
        
        initialized = false;
        autoShutterMinSpinBox->setValue(newValue);
        autoShutterMaxSlider->setMinValue(newValue + 1);
        autoShutterMaxSpinBox->setMinValue(newValue + 1);
        initialized = initSave;
    }
}


void AdjustmentsForm::autoShutterMinSpinBoxChanged(int newValue) {
    bool initSave = initialized;
    
    
    if(initialized) {
        if(!dc1394->setAutoShutterLimits(id, newValue, autoShutterMaxSpinBox->value())) {
            logger->log("Changing 'AutoShutter' limits failed!");
            updateUI();
            return;
        }
        
        initialized = false;
        autoShutterMinSlider->setValue(newValue);
        autoShutterMaxSlider->setMinValue(newValue + 1);
        autoShutterMaxSpinBox->setMinValue(newValue + 1);
        initialized = initSave;
    }
}


void AdjustmentsForm::autoShutterMaxSliderChanged(int newValue) {
    bool initSave = initialized;
    
    
    if(initialized) {
        if(!dc1394->setAutoShutterLimits(id, autoShutterMinSpinBox->value(), newValue)) {
            logger->log("Changing 'AutoShutter' limits failed!");
            updateUI();
            return;
        }
        
        initialized = false;
        autoShutterMaxSpinBox->setValue(newValue);
        autoShutterMinSlider->setMaxValue(newValue - 1);
        autoShutterMinSpinBox->setMaxValue(newValue - 1);
        initialized = initSave;
    }
}


void AdjustmentsForm::autoShutterMaxSpinBoxChanged(int newValue) {
    bool initSave = initialized;
    
    
    if(initialized) {
        if(!dc1394->setAutoShutterLimits(id, autoShutterMinSpinBox->value(), newValue)) {
            logger->log("Changing 'AutoShutter' limits failed!");
            updateUI();
            return;
        }
        
        initialized = false;
        autoShutterMaxSlider->setValue(newValue);
        autoShutterMinSlider->setMaxValue(newValue - 1);
        autoShutterMinSpinBox->setMaxValue(newValue - 1);
        initialized = initSave;
    }
}


void AdjustmentsForm::autoShutterReset() {
    bool initSave = initialized;
    unsigned int min = autoShutterMinSpinBox->minValue();
    unsigned int max = autoShutterMaxSpinBox->maxValue();
    
    logger->log("Min: %d, Max: %d", min, max);
    
    initialized = false;
    autoShutterMinSlider->setValue(min);
    autoShutterMinSpinBox->setValue(min);
    autoShutterMaxSlider->setMinValue(min +1);
    autoShutterMaxSpinBox->setMinValue(min +1);
    autoShutterMaxSlider->setValue(max);
    autoShutterMaxSpinBox->setValue(max);
    autoShutterMinSlider->setMaxValue(max - 1);
    autoShutterMinSpinBox->setMaxValue(max - 1);
    initialized = initSave;
    
    autoShutterChanged = true;
    featuresApply();
}


void AdjustmentsForm::autoGainMinSliderChanged(int newValue) {
    bool initSave = initialized;
    
    
    if(initialized) {
        if(!dc1394->setAutoGainLimits(id, newValue, autoGainMaxSpinBox->value())) {
            logger->log("Changing 'AutoShutter' limits failed!");
            updateUI();
            return;
        }
        
        initialized = false;
        autoGainMinSpinBox->setValue(newValue);
        autoGainMaxSlider->setMinValue(newValue - 1);
        autoGainMaxSpinBox->setMinValue(newValue - 1);
        initialized = initSave;
    }
}


void AdjustmentsForm::autoGainMinSpinBoxChanged(int newValue) {
    bool initSave = initialized;
    
    
    if(initialized) {
        if(!dc1394->setAutoGainLimits(id, newValue, autoGainMaxSpinBox->value())) {
            logger->log("Changing 'AutoShutter' limits failed!");
            updateUI();
            return;
        }
        
        initialized = false;
        autoGainMinSlider->setValue(newValue);
        autoGainMaxSlider->setMinValue(newValue - 1);
        autoGainMaxSpinBox->setMinValue(newValue - 1);
        initialized = initSave;
    }
}


void AdjustmentsForm::autoGainMaxSliderChanged(int newValue) {
    bool initSave = initialized;
    
    
    if(initialized) {
        if(!dc1394->setAutoGainLimits(id, autoGainMinSpinBox->value(), newValue)) {
            logger->log("Changing 'AutoShutter' limits failed!");
            updateUI();
            return;
        }
        
        initialized = false;
        autoGainMaxSpinBox->setValue(newValue);
        autoGainMinSlider->setMaxValue(newValue - 1);
        autoGainMinSpinBox->setMaxValue(newValue - 1);
        initialized = initSave;
    }
}


void AdjustmentsForm::autoGainMaxSpinBoxChanged(int newValue) {
    bool initSave = initialized;
    
    
    if(initialized) {
        if(!dc1394->setAutoGainLimits(id, autoGainMinSpinBox->value(), newValue)) {
            logger->log("Changing 'AutoShutter' limits failed!");
            updateUI();
            return;
        }
        
        initialized = false;
        autoGainMaxSlider->setValue(newValue);
        autoGainMinSlider->setMaxValue(newValue - 1);
        autoGainMinSpinBox->setMaxValue(newValue - 1);
        initialized = initSave;
    }
}


void AdjustmentsForm::autoGainReset() {
    bool initSave = initialized;
    unsigned int min = autoGainMinSpinBox->minValue();
    unsigned int max = autoGainMaxSpinBox->maxValue();
    
    
    logger->log("Min: %d, Max: %d", min, max);
    
    initialized = false;
    autoGainMinSlider->setMaxValue(max - 1);
    autoGainMinSpinBox->setMaxValue(max - 1);
    autoGainMinSlider->setValue(min);
    autoGainMinSpinBox->setValue(min);
    autoGainMaxSlider->setMinValue(min + 1);
    autoGainMaxSpinBox->setMinValue(min + 1);
    autoGainMaxSlider->setValue(max);
    autoGainMaxSpinBox->setValue(max);
    initialized = initSave;
    
    autoGainChanged = true;
    featuresApply();
}


void AdjustmentsForm::autoAOIOnChecked(bool on) {
    bool initSave = initialized;
    
    if(initialized) {
        if(!dc1394->enableAutoAOI(id, on)) {
            logger->log("%s 'Auto-AOI' failed!", on ? "Enabling" : "Disabling");
            initialized = false;
            autoAOIOnCheckBox->setChecked(!on);
            initialized = initSave;
        }
        else {
            autoAOISwitched = true;
            autoAOIChanged = true;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::autoAOIWidthSliderChanged(int newValue)  {
    unsigned int maxXSize = autoAOIImageFrame->width();
    unsigned int maxYSize = autoAOIImageFrame->height();
    unsigned int maxX = autoAOIWidthSlider->maxValue();
    unsigned int h = autoAOIHeightSlider->value();
    unsigned int maxY = autoAOIHeightSlider->maxValue();
    
    
    if(autoAOICorrectWidthSliderValue)
        return;
    
    logger->log("AutoAOI-WidthSliderChanged() - 'newValue' = %d", newValue);
    newValue = (newValue < 0) ? 128 : newValue;
    newValue &= 0xffffff80UL;
    if(newValue == 0)
        newValue = 128;
    
    logger->log("AutoAOI-WidthSliderChanged() - 'newValue' converted to %d", newValue);
    
    autoAOICorrectWidthSliderValue = true;
    autoAOIWidthSlider->setValue(newValue);
    autoAOICorrectWidthSliderValue = false;
            
    autoAOISectorFrame->resize((maxXSize - 6) * newValue / maxX,
                               (maxYSize - 6) * h / maxY);
    autoAOIXSlider->setMaxValue(maxX - newValue);

    logger->log("AutoAOI-Width-slider %s changed to %d", 
           initialized ? "manually" : "automatically", newValue);
    
    autoAOIWidthSpinBox->setValue(newValue);
    
    if(initialized) {
        logger->log("AutoAOIWidthSlider: Enabling 'Apply' and 'Discard'...");
        // enable the apply- and discard-button
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        // set feature-changed-flag
        autoAOIChanged = true;
    }
}


void AdjustmentsForm::autoAOIWidthSpinBoxChanged(int newValue)  {
    unsigned int maxXSize = autoAOIImageFrame->width();
    unsigned int maxYSize = autoAOIImageFrame->height();
    unsigned int maxX = autoAOIWidthSpinBox->maxValue();
    unsigned int h = autoAOIHeightSpinBox->value();
    unsigned int maxY = autoAOIHeightSpinBox->maxValue();
    
    
    if(autoAOICorrectWidthSpinBoxValue)
        return;
    
    logger->log("AutoAOI-WidthSpinBoxChanged() - 'newValue' = %d", newValue);
    newValue = (newValue < 0) ? 128 : newValue;
    newValue &= 0xffffff80UL;
    if(newValue == 0)
        newValue = 128;
    
    logger->log("AutoAOI-WidthSpinBoxChanged() - 'newValue' converted to %d", newValue);
    
    autoAOICorrectWidthSpinBoxValue = true;
    autoAOIWidthSpinBox->setValue(newValue);
    autoAOICorrectWidthSpinBoxValue = false;
            
    autoAOISectorFrame->resize((maxXSize - 6) * newValue / maxX, (maxYSize - 6) * h / maxY);
    autoAOIXSpinBox->setMaxValue(maxX - newValue);
    
    logger->log("AutoAOI-Width-spin-box %s changed to %d",
           initialized ? "manually" : "automatically", newValue);
    
    autoAOIWidthSlider->setValue(newValue);
    
    if(initialized) {
        logger->log("AutoAOIWidthSpinBox: Enabling 'Apply' and 'Discard'...");
        // enable the apply- and discard-button
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        // set feature-changed-flag
        autoAOIChanged = true;
    }
}


void AdjustmentsForm::autoAOIHeightSliderChanged(int newValue)  {
    unsigned int maxXSize = autoAOIImageFrame->width();
    unsigned int maxYSize = autoAOIImageFrame->height();
    unsigned int w = autoAOIWidthSlider->value();
    unsigned int maxX = autoAOIWidthSlider->maxValue();
    unsigned int maxY = autoAOIHeightSlider->maxValue();

    
    if(autoAOICorrectHeightSliderValue)
        return;
    
    logger->log("AutoAOI-HeightSliderChanged() - 'newValue' = %d", newValue);
    newValue = (newValue < 0) ? 128 : newValue;
    newValue &= 0xffffff80UL;
    if(newValue == 0)
        newValue = 128;
    
    logger->log("AutoAOI-HeightSliderChanged() - 'newValue' converted to %d", newValue);
    
    autoAOICorrectHeightSliderValue = true;
    autoAOIHeightSlider->setValue(newValue);
    autoAOICorrectHeightSliderValue = false;
            
    autoAOISectorFrame->resize((maxXSize - 6) * w / maxX, (maxYSize - 6) * newValue / maxY);
    autoAOIYSlider->setMaxValue(maxY - newValue);

    logger->log("AutoAOI-Height-slider %s changed to %d",
           initialized ? "manually" : "automatically", newValue);
    
    autoAOIHeightSpinBox->setValue(newValue);
    
    if(initialized) {
        logger->log("AutoAOIHeightSlider: Enabling 'Apply' and 'Discard'...");
        // enable the apply- and discard-button
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        // set feature-changed-flag
        autoAOIChanged = true;
    }
}


void AdjustmentsForm::autoAOIHeightSpinBoxChanged(int newValue)  {
    unsigned int maxXSize = autoAOIImageFrame->width();
    unsigned int maxYSize = autoAOIImageFrame->height();
    unsigned int w = autoAOIWidthSpinBox->value();
    unsigned int maxX = autoAOIWidthSpinBox->maxValue();
    unsigned int maxY = autoAOIHeightSpinBox->maxValue();
    
    
    if(autoAOICorrectHeightSpinBoxValue)
        return;
    
    logger->log("AutoAOI-HeightSpinBoxChanged() - 'newValue' = %d", newValue);
    newValue = (newValue < 0) ? 128 : newValue;
    newValue &= 0xffffff80UL;
    if(newValue == 0)
        newValue = 128;
    
    logger->log("AutoAOI-HeightSpinBoxChanged() - 'newValue' converted to %d", newValue);
    
    autoAOICorrectHeightSpinBoxValue = true;
    autoAOIHeightSpinBox->setValue(newValue);
    autoAOICorrectHeightSpinBoxValue = false;
            
    autoAOISectorFrame->resize((maxXSize - 6) * w / maxX, (maxYSize - 6) * newValue / maxY);
    autoAOIYSpinBox->setMaxValue(maxY - newValue);

    logger->log("AutoAOI-Height-spin-box %s changed to %d",
           initialized ? "manually" : "automatically", newValue);
    
    autoAOIHeightSlider->setValue(newValue);
    
    if(initialized) {
        logger->log("AutoAOIHeightSpinBox: Enabling 'Apply' and 'Discard'...");
        // enable the apply- and discard-button
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        // set feature-changed-flag
        autoAOIChanged = true;
    }
}


void AdjustmentsForm::autoAOIXSliderChanged(int newValue)  {
    unsigned int maxXSize = autoAOIImageFrame->width();
    unsigned int maxYSize = autoAOIImageFrame->height();
    unsigned int maxX = autoAOIWidthSlider->maxValue();
    unsigned int y = autoAOIYSlider->value();
    unsigned int maxY = autoAOIHeightSlider->maxValue();
    
    
    if(autoAOICorrectXSliderValue)
        return;
    
    logger->log("AutoAOI-XSliderChanged() - 'newValue' = %d (0x%x)", newValue, newValue);
    newValue = (newValue < 0) ? 0 : newValue;
    newValue &= 0xffffff80UL;
    
    logger->log("AutoAOI-XSliderChanged() - 'newValue' converted to %d (0x%x)",
           newValue, newValue);
    
    autoAOICorrectXSliderValue = true;
    autoAOIXSlider->setValue(newValue);
    autoAOICorrectXSliderValue = false;
            
    autoAOISectorFrame->move((newValue * (maxXSize - 3) / maxX) + 3,
                             (y * (maxYSize - 3) / maxY) + 3);
    
    logger->log("AutoAOI X slider %s changed to %d",
           initialized ? "manually" : "automatically", newValue);
    
    autoAOIXSpinBox->setValue(newValue);
    
    if(initialized) {
        logger->log("AutoAOIXSlider: Enabling 'Apply' and 'Discard'...");
        // enable the apply- and discard-button
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        // set feature-changed-flag
        autoAOIChanged = true;
    }
}


void AdjustmentsForm::autoAOIXSpinBoxChanged(int newValue)  {
    unsigned int maxXSize = autoAOIImageFrame->width();
    unsigned int maxYSize = autoAOIImageFrame->height();
    unsigned int maxX = autoAOIWidthSpinBox->maxValue();
    unsigned int y = autoAOIYSpinBox->value();
    unsigned int maxY = autoAOIHeightSpinBox->maxValue();
    
    
    if(autoAOICorrectXSpinBoxValue)
        return;
    
    logger->log("AutoAOI-XSpinBoxChanged() - 'newValue' = %d (0x%x)", newValue, newValue);
    newValue = (newValue < 0) ? 0 : newValue;
    newValue &= 0xffffff80UL;
    
    logger->log("AutoAOI-XSpinBoxChanged() - 'newValue' converted to %d (0x%x)",
           newValue, newValue);
    
    autoAOICorrectXSpinBoxValue = true;
    autoAOIXSpinBox->setValue(newValue);
    autoAOICorrectXSpinBoxValue = false;
            
    autoAOISectorFrame->move((newValue * (maxXSize - 3) / maxX) + 3,
                           (y * (maxYSize - 3) / maxY) + 3);

    logger->log("AutoAOI X spin-box %s changed to %d",
           initialized ? "manually" : "automatically", newValue);
    
    autoAOIXSlider->setValue(newValue);
    
    if(initialized) {
        logger->log("AutoAOIXSpinBox: Enabling 'Apply' and 'Discard'...");
        // enable the apply- and discard-button
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        // set feature-changed-flag
        autoAOIChanged = true;
    }
}


void AdjustmentsForm::autoAOIYSliderChanged(int newValue)  {
    unsigned int maxXSize = autoAOIImageFrame->width();
    unsigned int maxYSize = autoAOIImageFrame->height();
    unsigned int x = autoAOIXSlider->value();
    unsigned int maxX = autoAOIWidthSlider->maxValue();
    unsigned int maxY = autoAOIHeightSlider->maxValue();
    
    
    if(autoAOICorrectYSliderValue)
        return;
    
    logger->log("AutoAOI-YSliderChanged() - 'newValue' = %d (0x%x)", newValue, newValue);
    newValue = (newValue < 0) ? 0 : newValue;
    newValue &= 0xffffff80UL;
    
    logger->log("AutoAOI-YSliderChanged() - 'newValue' converted to %d (0x%x)",
           newValue, newValue);
    
    autoAOICorrectYSliderValue = true;
    autoAOIYSlider->setValue(newValue);
    autoAOICorrectYSliderValue = false;
            
    autoAOISectorFrame->move((x * (maxXSize - 3) / maxX) + 3,
                           (newValue * (maxYSize - 3) / maxY) + 3);

    logger->log("AutoAOI Y slider %s changed to %d",
           initialized ? "manually" : "automatically", newValue);
    
    autoAOIYSpinBox->setValue(newValue);
    
    if(initialized) {
        logger->log("AutoAOIYSlider: Enabling 'Apply' and 'Discard'...");
        // enable the apply- and discard-button
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        // set feature-changed-flag
        autoAOIChanged = true;
    }
}


void AdjustmentsForm::autoAOIYSpinBoxChanged(int newValue)  {
    unsigned int maxXSize = autoAOIImageFrame->width();
    unsigned int maxYSize = autoAOIImageFrame->height();
    unsigned int x = autoAOIXSpinBox->value();
    unsigned int maxX = autoAOIWidthSpinBox->maxValue();
    unsigned int maxY = autoAOIHeightSpinBox->maxValue();
    
    
    if(autoAOICorrectYSpinBoxValue)
        return;
    
    logger->log("AutoAOI-YSpinBoxChanged() - 'newValue' = %d (0x%x)", newValue, newValue);
    newValue = (newValue < 0) ? 0 : newValue;
    newValue &= 0xffffff80UL;
    
    logger->log("AutoAOI-YSpinBoxChanged() - 'newValue' converted to %d (0x%x)",
           newValue, newValue);
    
    autoAOICorrectYSpinBoxValue = true;
    autoAOIYSpinBox->setValue(newValue);
    autoAOICorrectYSpinBoxValue = false;
            
    autoAOISectorFrame->move((x * (maxXSize - 3) / maxX) + 3,
                             (newValue * (maxYSize - 3) / maxY) + 3);

    logger->log("AutoAOI Y spin-box %s changed to %d",
           initialized ? "manually" : "automatically", newValue);
    
    autoAOIYSlider->setValue(newValue);
    
    if(initialized) {
        logger->log("AutoAOIYSpinBox: Enabling 'Apply' and 'Discard'...");
        // enable the apply- and discard-button
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        // set feature-changed-flag
        autoAOIChanged = true;
    }
}


void AdjustmentsForm::autoAOIMaximize() {
    bool initSave = initialized;
    
    
    initialized = false;
    
    autoAOIXSlider->setValue(0);
    autoAOIXSpinBox->setValue(0);
    autoAOIYSlider->setValue(0);
    autoAOIYSpinBox->setValue(0);
    
    autoAOIWidthSlider->setValue(autoAOIWidthSlider->maxValue());
    autoAOIWidthSpinBox->setValue(autoAOIWidthSpinBox->maxValue());
    autoAOIHeightSlider->setValue(autoAOIHeightSlider->maxValue());
    autoAOIHeightSpinBox->setValue(autoAOIHeightSpinBox->maxValue());
    
    initialized = initSave;
    
    if(initialized) {
        logger->log("AutoAOIMaximize: Enabling 'Apply' and 'Discard'...");
        applyPushButton->setEnabled(true);
        discardPushButton->setEnabled(true);
        
        autoAOIChanged = true;
    }
}


void AdjustmentsForm::autoAOIShowArea(bool enabled) {
    bool initSave = initialized;
    
    if(initialized) {
        if(!dc1394->showAutoAOIArea(id, enabled)) {
            logger->log("Failed to %s 'Aoto-AOI' area", enabled ? "show" : "hide");
            initialized = false;
            autoAOIShowAreaCheckBox->setChecked(!enabled);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::deferredImgHoldImgChanged(bool enabled) {
    bool initSave = initialized;
    
    
    if(initialized) {
        if(!dc1394->enableDeferredImg(id, enabled)) {
            logger->log("Error while trying to %s 'Def.-Img.'", enabled ? "enable" : "disable");
            initialized = false;
            deferredImgHoldCheckBox->setChecked(!enabled);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::deferredImgFastCaptureChanged(bool enabled) {
    bool initSave = initialized;
    
    
    if(initialized) {
        if(!dc1394->deferredImgFastCaptureEnable(id, enabled)) {
            logger->log("Error while trying to %s 'Def.-Img.' fast-capture",
                        enabled ? "enable" : "disable");
            initialized = false;
            deferredImgFastCaptureCheckBox->setChecked(!enabled);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::deferredImgFrameCountChanged(int newValue) {
    if(initialized) {
        if(!dc1394->setDeferredImgFrameCount(id, newValue)) {
            logger->log("Error while trying to change 'Def.-Img.' frame-count");
        }
    }
    
    updateUI();
}


void AdjustmentsForm::deferredImgSend() {
    logger->log("Sending deferred images...");
    
    if(!dc1394->deferredImgSend(id, deferredImgFrameCountSpinBox->value())) {
        logger->log("Error while trying to trigger sending of the deferred images");
    }
}


void AdjustmentsForm::frameInfoReset() {
    if(!dc1394->frameInfoReset(id)) {
        logger->log("Error while trying to reset FrameInfo Counter");
    }
    else {
        frameInfoCountLineEdit->setText(QString::number(0));
    }
}


void AdjustmentsForm::frameInfoUpdate() {
    unsigned int counter;
    
    
    if(!dc1394->getFrameInfoCount(id, &counter)) {
        logger->log("Error while trying to read current FrameInfo counter value");
        counter = 0;
    }
    
    frameInfoCountLineEdit->setText(QString::number(counter));
}


void AdjustmentsForm::delIntOnChanged(bool enable) {
    bool initSave = initialized;
    logger->log("Delayed Integration switched %s", enable ? "on" : "off");
    
    if(initialized) {
        if(!dc1394->delIntEnable(id, enable)) {
            logger->log("Failed while trying to %s 'Del.-Int.'", enable ? "enable" : "disable");
            initialized = false;
            delIntOnCheckBox->setChecked(!enable);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::delIntDelayChanged(int value) {
    logger->log("Integration Delay changed to %d us", value);
    
    
    if(initialized) {
        if(!dc1394->setDelIntValue(id, value))
            logger->log("Error while trying to change 'Del.-Int.' value!");
    }
    
    updateUI();
}


void AdjustmentsForm::incDecOnChanged(bool enable) {
    bool initSave = initialized;
    
    
    if(initialized) {
        if(!dc1394->incDecEnable(id, enable)) {
            logger->log("Trying to %s 'Inc.-Dec.' failed!");
            initialized = false;
            incDecOnCheckBox->setChecked(!enable);
            initialized = initSave;
        }
    }
    
    updateUI();
}


void AdjustmentsForm::incDecCompareChanged(int newValue) {
    if(initialized) {
        if(!dc1394->setIncDecCompare(id, newValue))
            logger->log("Changing 'Inc.-Dec.' value failed!");
    }
    
    updateUI();
}



void AdjustmentsForm::incDecUpdateRequest() {
    unsigned int count;
    
    
    logger->log("Updating Inc.-Dec. Counter...");
    
    // update the Inc. Dec. Counter
    if(!dc1394->getIncDecCounter(id, &count)) {
        logger->log("Error while trying to read Inc.-Dec.-Counter value");
    }
    else {
        incDecCountLineEdit->setText(QString::number(count));
    }
}


void AdjustmentsForm::incDecReset() {
    logger->log("Resetting Inc.-Dec. Counter...");
    
    if(!dc1394->incDecReset(id)) {
        logger->log("Error while trying to reset the incremental decoder");
    }
    else {
        incDecCountLineEdit->setText(QString::number(0));
    }
}


void AdjustmentsForm::lutNumberChanged(int newValue) {
    
    if(initialized) {
        if(!dc1394->selectLUT(id, newValue)) {
            logger->log("Error while trying to select LUT %d", newValue);
        }
    }
    
    updateUI();
}


void AdjustmentsForm::shadingCorrectionReadImgClicked() {
    QString fileName;
    QFile *readFile;
    unsigned char *readBuffer;
    unsigned int readBufferSize = 0;
    CamWindow *camWindow;
    
    
    readBufferSize = dc1394->getWidth(id) * dc1394->getHeight(id);
    readBuffer = new unsigned char[readBufferSize];
    
    
    fileName = QFileDialog::getSaveFileName("./readBack.sci", "*.sci", this, "save file dialog",
                                            "Read Shading Image from camera");
    
    if(fileName.isNull()) {
        logger->log("Nothing selected!");
        delete[] readBuffer;
        return;
    }
    
    logger->log("Now reading %d bytes back from camera to file %s...",
           readBufferSize, fileName.ascii());
    
    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
    
    if(QString(parent->className()) != "CamWindow") {
        logger->log("Parent of 'AdjustmentsForm' is '%s', but 'CamWindow' is expected!",
               parent->className());
        logger->log("Therefore a change of the 'Timebase' is not possible!");
        delete[] readBuffer;
        QApplication::restoreOverrideCursor();
        return;
    }
    
    camWindow  = (CamWindow *)parent;
    
    if(!camWindow->stopCapture()) {
        logger->log("Error while trying to stop capture...");
        delete[] readBuffer;
        QApplication::restoreOverrideCursor();
        return;
    }
    
    if(!dc1394->shadingCorrectionReadImage(id, readBuffer, readBufferSize)) {
        logger->log("Error while trying to read back shading image from camera");
        delete[] readBuffer;
        QApplication::restoreOverrideCursor();
        return;
    }
    
    // create '\tmp\readBack.cc1394'
    readFile = new QFile(fileName.ascii());
    readFile->open(IO_Raw | IO_ReadWrite | IO_Truncate);
    // and write back 'readBuffer'
    readFile->writeBlock((const char*)readBuffer, readBufferSize);
    // close 'tmp-file'
    readFile->close();
    delete readFile;
    delete[] readBuffer;
    
    logger->log("\t'GPDataBuffer' successfully copied to %s", fileName.ascii());
    
    camWindow->startCapture();
    
    updateUI();
    
    QApplication::restoreOverrideCursor();
}


