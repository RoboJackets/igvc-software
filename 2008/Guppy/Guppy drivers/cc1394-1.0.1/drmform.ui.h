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

#define CAMERA_BASE 0xfffff0000000ULL
#define ADV_BASE 0xfffff1000000ULL
#define SIO_BASE 0xfffff0f02100ULL


void DRMForm::init() {
    dc1394 = NULL;
    id = -1;
    f7Mode = 0;
    
    drmHistoryTextEdit->setReadOnly(true);
    baseAddrLineEdit->setReadOnly(true);
    
    updateUIControl = true;
}


void DRMForm::setParameters(int i, DC1394 *dc, MessageLogger *l) {
    unsigned int formatInq, f7ModeInq, basicFuncInq, optFuncInq;
    
    dc1394 = dc;
    id = i;
    logger = l;
    
    // get global 'CCR-Offset'
    ccrOffset = dc1394->getCCROffset(id);
    
    // get the offset for the advanced features
    if(!dc1394->readData(id, 0x400, &basicFuncInq, false)) {
        logger->log("DRMForm::setParameters: Error while trying to read 'BASIC_FUNC_INQ' register!");
        advFeatureRadioButton->setEnabled(false);
    }
    else {
        if((basicFuncInq && 0x80000000UL) == 0) {
            logger->log("DRMForm::setParameters: 'Advanced_Feature_Inq' register not available!");
            advFeatureRadioButton->setEnabled(false);
        }
        else {
            if(!dc1394->readData(id, 0x480, &advOffset, false)) {
                logger->log("DRMForm::setParameters: Error while trying to read 'Advanced_Feature_Inq' register!");
                advFeatureRadioButton->setEnabled(false);
            }
            else {
                // 'advOffset' is a 'Quadlet-Offset' !!! 
                advOffset *= 4;
            }
        }
    }
    
    // get the offset for the SIO features
    if(!dc1394->readData(id, 0x400, &basicFuncInq, false)) {
        logger->log("DRMForm::setParameters:  Error while trying to read 'BASIC_FUNC_INQ' register!");
        sioRadioButton->setEnabled(false);
    }
    else {
        if((basicFuncInq && 0x10000000UL) == 0) {
            logger->log("DRMForm::setParameters:'Opt_Function_Inq' register not availble!");
            sioRadioButton->setEnabled(false);
        }
        else {
            if(!dc1394->readData(id, 0x40c, &optFuncInq, false)) {
                logger->log("DRMForm::setParameters: Error while trying to read 'Opt_Function_Inq' register!");
                sioRadioButton->setEnabled(false);
            }
            else {
                if((optFuncInq & 0x20000000UL) == 0) {
                    logger->log("DRMForm::setParameters:  'SIO_Control_CSR_Inq' register not available!");
                    sioRadioButton->setEnabled(false);
                }
                else {
                    if(!dc1394->readData(id, 0x488, &sioOffset, false)) {
                        logger->log("DRMForm::setParameters: Error while trying to read 'SIO_Control_CSR_Inq' register!");
                        sioRadioButton->setEnabled(false);
                    }
                    else {
                        // 'sioOffset' is a 'Quadlet-Offset' !!!
                        sioOffset *= 4;
                    }
                }
            }
        }
    }
    
    // enable / disable F7-control(s)
    if(!dc1394->readData(id, 0x100, &formatInq, false)) {
        logger->log("DRMForm::setParameters: Error while trying to read 'V_FORMAT_INQ'!");
        f7RadioButton->setEnabled(false);
        f7ModeComboBox->setEnabled(false);
    }
    else {
        if((formatInq & 0x01000000UL) == 0) {
            logger->log("DRMForm::setParameters: Format7 not available!");
            f7RadioButton->setEnabled(false);
            f7ModeComboBox->setEnabled(false);
        }
        else {
            // determine available F7-Modes
            if(!dc1394->readData(id, 0x19c, &f7ModeInq, false)) {
                logger->log("DRMForm::setParameters: Error while trying to read 'V_MODE_INQ_7'!");
                f7RadioButton->setEnabled(false);
                f7ModeComboBox->setEnabled(false);
            }
            else {
                if((f7ModeInq & 0xff000000) == 0) {
                    logger->log("DRMForm::setParameters: No F7-Modes available!");
                    f7RadioButton->setEnabled(false);
                    f7ModeComboBox->setEnabled(false);
                }
                else {
                    fillF7ModeComboBox(f7ModeInq >> 24);
                    
                    f7RadioButton->setEnabled(true);
                    f7ModeComboBox->setEnabled(f7RadioButton->isChecked());
                }
            }
        }
    }
    
    // initialize Radio-Buttons and Base-Address LineEdit
    stdFeatureRadioButton->setChecked(true);
    baseAddrLineEdit->setText(QString("0x") +
                              (QString::number(CAMERA_BASE + ccrOffset, 16)).upper());
}


void DRMForm::fillF7ModeComboBox(unsigned int f7ModeInq) {
    QString modeString;
    
    
    for(int i = 0; i < 8; i++) {
        if(((f7ModeInq >> (7 - i)) & 1) == 1) {
            f7ModeComboBox->insertItem(QString("Mode ") + QString::number(i));
        }
    }
    
    // initialize 'f7Mode'
    modeString = f7ModeComboBox->currentText();
    modeString.replace("Mode ", "");
    f7Mode = modeString.toUInt();
    
    // initial F7-Offset
    if(!dc1394->readData(id, 0x2e0 + (f7Mode * 4), &f7Offset)) {
        logger->log("DRMForm::fillF7ModeComboBox: Error while trying to get F7-Mode %d base address!", f7Mode);
        f7RadioButton->setEnabled(false);
        f7ModeComboBox->setEnabled(false);
        return;
    }
    
    f7Offset *= 4;
}


void DRMForm::drmReadData() {
    unsigned int data;
    unsigned long offset = (drmAddressLineEdit->text()).toULong(0, 16);
    unsigned long long address;
    bool sio = sioRadioButton->isChecked();
    bool f7 = f7RadioButton->isChecked();
    QString hexString, hexStringLead;
    
    
    // check if address-offset is a multiple of 4 (!)
    if((offset / 4) * 4 != offset) {
        offset = (offset / 4) * 4;
        drmAddressLineEdit->setText("0x" +
                                    (QString::number(offset, 16)).upper());
    }
    
    // prepend '0x' to address if missing
    if(!(drmAddressLineEdit->text()).contains("0x")) {
        drmAddressLineEdit->setText("0x" + drmAddressLineEdit->text());
    }
    
    // determine the complete address
    address = ((baseAddrLineEdit->text()).toULongLong(0, 16)) + offset;
    
    // read data at 'Address' and display it in the 'Data' LineEdit
    offset += (advFeatureRadioButton->isChecked() ? advOffset : 0);
    if(!dc1394->readData(id, offset,
                         &data, sio, f7, f7Mode)) {
        // Add Error-Message to History and Logger
        hexString = (QString::number(address, 16)).upper();
        hexStringLead = QString("");
        if(hexString.length() < 12) {
            for(unsigned int i = 0; (12 - hexString.length()); i++) {
                hexStringLead += "0";
            }
        }
        drmHistoryTextEdit->append("Error while trying to read data from address 0x" +
                                   hexStringLead + hexString);
        logger->log("Error while trying to read data from address 0x%X",
                    address);
        
        // Clear Data LineEdit
        drmDataLineEdit->setText("");
        
        return;
    }
    
    // Display Data
    hexString = (QString::number(data, 16)).upper();
    hexStringLead = QString("");
    if(hexString.length() < 8) {
        for(unsigned int i = 0; i < (8 - hexString.length()); i++) {
            hexStringLead += "0";
        }
    }
    drmDataLineEdit->setText("0x" + hexStringLead + hexString);
    
    // Add current command to History
    hexString = (QString::number(address, 16)).upper();
    hexStringLead = QString("");
    if(hexString.length() < 12) {
        for(unsigned int i = 0; i < (12 - hexString.length()); i++) {
            hexStringLead += "0";
        }
    }
    drmHistoryTextEdit->append("0x" +
                               hexStringLead + hexString +
                               " -> " + 
                               drmDataLineEdit->text());
    
    // Append Register-Value in binary form
    QString binNumber = QString::number((drmDataLineEdit->text()).toULong(0, 16), 2);
    QString binNumberLead = QString("");
    
    if(binNumber.length() < 32) {
        // prepend leading '0's
        for(unsigned int i = 0; i < (32 - binNumber.length()); i++) {
            binNumberLead += "0";
        }
    }
    drmHistoryTextEdit->append("(" +
                               binNumberLead +
                               binNumber +
                               ")");
}


void DRMForm::drmWriteData() {
    unsigned int data = (drmDataLineEdit->text()).toUInt(0, 16);
    unsigned long offset = (drmAddressLineEdit->text()).toULong(0, 16);
    unsigned long long address;
    bool sio = sioRadioButton->isChecked();
    bool f7 = f7RadioButton->isChecked();
    QString hexString, hexStringLead;
    
    
    if(f7) {
        if(QMessageBox::information(this, "F7-DRM",
                                    "Please be carefull with changing F7-register-contents!\n"
                                    "Because some modifications require an update of the\n"
                                    "'display-routine', this may lead to unpredictable results!\n\n"
                                    "Continue anyway?",
                                    QMessageBox::Yes,
                                    QMessageBox::No | QMessageBox::Default | QMessageBox::Escape,
                                    QMessageBox::NoButton) == QMessageBox::No)
            return;
    }
    
    // check if address-offset is a multiple of 4 (!)
    if((offset / 4) * 4 != offset) {
        offset = (offset / 4) * 4;
        drmAddressLineEdit->setText("0x" + (QString::number(offset, 16)).upper());
    }
    
    // prepend '0x' if missing to address
    if(!(drmAddressLineEdit->text()).contains("0x")) {
        drmAddressLineEdit->setText("0x" + drmAddressLineEdit->text());
    }
    
    // ... and data (incl. filling up with leading '0's)
    hexString = (QString::number((drmDataLineEdit->text()).toULong(0, 16), 16)).upper();
    hexStringLead = QString("");
    if(hexString.length() < 8) {
        for(unsigned int i = 0; i < (8 - hexString.length()); i++) {
            hexStringLead += "0";
        }
        drmDataLineEdit->setText("0x" + hexStringLead + hexString);
    }
    
    if(!(drmDataLineEdit->text()).contains("0x")) {
        drmDataLineEdit->setText("0x" + drmDataLineEdit->text());
    }
    
    // determine the complete address
    address = ((baseAddrLineEdit->text()).toULongLong(0, 16)) + offset;
    
    // write data from 'Data' to 'Address'
    offset += (advFeatureRadioButton->isChecked() ? advOffset : 0);
    if(!dc1394->writeData(id, offset, data, sio)) {
        hexString = (QString::number(address, 16)).upper();
        hexStringLead = QString("");
        if(hexString.length() < 12) {
            for(unsigned int i = 0; i < (12 - hexString.length()); i++) {
                hexStringLead += "0";
            }
        }
        drmHistoryTextEdit->append("Error while trying to change data at address 0x" +
                                   hexStringLead + hexString);
        logger->log("Error while trying to change data at address 0x%X",
                    address, 16);
        
        return;
    }
    
    hexString = (QString::number(address, 16)).upper();
    hexStringLead = QString("");
    if(hexString.length() < 12) {
        for(unsigned int i = 0; i < (12 - hexString.length()); i++) {
            hexStringLead += "0";
        }
    }
    drmHistoryTextEdit->append("0x" +
                               hexStringLead + hexString +
                               " -> " + 
                               drmDataLineEdit->text());
    
    // Append Register-Value in binary form
    QString binNumber = QString::number((drmDataLineEdit->text()).toULong(0, 16), 2);
    QString binNumberLead = QString("");
    
    if(binNumber.length() < 32) {
        // prepend leading '0's
        for(unsigned int i = 0; i < (32 - binNumber.length()); i++) {
            binNumberLead += "0";
        }
    }
    drmHistoryTextEdit->append("(" +
                               binNumberLead +
                               binNumber +
                               ")");
}


void DRMForm::f7Selected(bool newState) {
    if(!updateUIControl)
        return;
    
    if(newState) {
        QMessageBox::information(this, "F7-DRM",
                                 "Please be carefull with changing F7-register-contents!\n"
                                 "Because some modifications require an update of the\n"
                                 "'display-routine', this may lead to unpredictable results!",
                                 QMessageBox::Ok | QMessageBox::Default | QMessageBox::Escape,
                                 QMessageBox::NoButton,
                                 QMessageBox::NoButton);
    }
    
    baseAddrLineEdit->setText(QString("0x") +
                              (QString::number(CAMERA_BASE + f7Offset, 16)).upper());
    
    f7ModeComboBox->setEnabled(newState);
}


void DRMForm::sioSelected(bool newState) {
    if(newState) {
        baseAddrLineEdit->setText(QString("0x") + 
                                  (QString::number(CAMERA_BASE + sioOffset, 16)).upper());
    }
}


void DRMForm::f7ModeSelected(const QString& newMode) {
    QString modeString = newMode;
    
    
    modeString.replace("Mode ", "");
    f7Mode = modeString.toUInt();
    
    // initial f7Offset
    if(!dc1394->readData(id, 0x2e0 + (f7Mode * 4), &f7Offset)) {
        logger->log("DRMForm::fillF7ModeComboBox: Error while trying to get F7-Mode %d base address!", f7Mode);
//        f7CheckBox->setChecked(false);
        f7RadioButton->setEnabled(false);
        f7ModeComboBox->setEnabled(false);
        return;
    }
    
    f7Offset *= 4;
    
    baseAddrLineEdit->setText(QString("0x") +
                              (QString::number(CAMERA_BASE + f7Offset, 16)).upper());
    
}


void DRMForm::stdFeatureSelected( bool newState ) {
    if(newState) {
        baseAddrLineEdit->setText(QString("0x") +
                                  (QString::number(CAMERA_BASE + ccrOffset, 16)).upper());
    }
}


void DRMForm::advFeatureSelected( bool newState ) {
    if(newState) {
        baseAddrLineEdit->setText(QString("0x") +
                                  (QString::number(CAMERA_BASE + advOffset, 16)).upper());
    }
}
