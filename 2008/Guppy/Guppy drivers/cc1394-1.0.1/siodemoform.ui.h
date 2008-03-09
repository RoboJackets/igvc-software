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

void SIODemoForm::init() {
    dc1394 = NULL;
    id = 0;
    stop = false;
    receiveCount = 0;
    receivedDataTextEdit->clear();
    readCount = 0;
}


void SIODemoForm::initForm(QWidget *p, DC1394 *dc, unsigned int i, MessageLogger *ml) {
    dc1394 = dc;
    id = i;
    logger = ml;
    parent = p;
    
    receivedDataTextEdit->setReadOnly(true);
    updateUI = false;
    
    stopPushButton->setCursor(QCursor(Qt::ArrowCursor));
}


void SIODemoForm::byteNoSpinBoxChanged(int number) {
    endlessCheckBox->setEnabled(number > 0);
    timeoutSpinBox->setEnabled((!endlessCheckBox->isChecked()) && (number > 0));
    receivePushButton->setEnabled(unknownCheckBox->isChecked() || (number > 0));
}


void SIODemoForm::receive() {
    logger->log("SIODemoForm::receive: Trying to read %s bytes from SIO...",
                unknownCheckBox->isChecked() ? "'unlimited'" :
                QString::number(byteNoSpinBox->value()).ascii());
    
    // disable some UI-controls for uninterrupted handling of the read-command
    stop = false;
    disableUIControls();
    
    setCursor(QCursor(Qt::WaitCursor));
    receivedDataTextEdit->setCursor(QCursor(Qt::WaitCursor));
    // exec current GUI event-queue before continuing...
    qApp->processEvents();
    
    // prepare the serial interface (i.e. enable Rx and clear Error-Flags)
    // SERIAL_CONTROL_REG.RE = 1 (0x04 <- 0x80000000UL)
    if(!prepareSIO(true)) {
        logger->log("SerialOReceiveForm::receive: Error while trying to prepare SIO for reading!");
        enableUIControls();
        setCursor(QCursor(Qt::ArrowCursor));
        receivedDataTextEdit->setCursor(QCursor(Qt::ArrowCursor));
        qApp->processEvents();
        return;
    }
    
    // wait for bytes to come until the timeout (if not endless)
    // 'SERIAL_STATUS_REG.RDRD' == 1
    if(!waitForDataAvailability()) {
        if(!stop) {
            logger->log("SIODemoForm::receive: No data received within the requested time!");
            QMessageBox::warning(this, "Receive", "No data received within the expected time!",
                                 QMessageBox::Ok | QMessageBox::Default | QMessageBox::Escape,
                                 QMessageBox::NoButton,
                                 QMessageBox::NoButton);
        }
        else
            logger->log("SIODemoForm::receive: Waiting for data availability manually interrupted");
        enableUIControls();
        setCursor(QCursor(Qt::ArrowCursor));
        receivedDataTextEdit->setCursor(QCursor(Qt::ArrowCursor));
        qApp->processEvents();
        return;
    }
    
    do {
        qApp->processEvents();
        
        if(!waitForData() || stop) {
            if(!stop) {
                logger->log("SIODemoForm::receive: Unable to wait for expected data!");
                QMessageBox::warning(this, "Receive", "Unable to wait for expected data!",
                                     QMessageBox::Ok | QMessageBox::Default | QMessageBox::Escape,
                                     QMessageBox::NoButton,
                                     QMessageBox::NoButton);
            }
            else
                logger->log("SIODemoForm::receive: Waiting for data manually interrupted");
            enableUIControls();
            setCursor(QCursor(Qt::ArrowCursor));
            receivedDataTextEdit->setCursor(QCursor(Qt::ArrowCursor));
            qApp->processEvents();
            return;
        }
        
        if(receiveCount == 0) {
            logger->log("SIODemoForm::receive: Nothing read...");
            enableUIControls();
            setCursor(QCursor(Qt::ArrowCursor));
            receivedDataTextEdit->setCursor(QCursor(Qt::ArrowCursor));
            qApp->processEvents();
            return;
        }
        
        // read data from the serial interface
        if(!readData() || stop) {
            if(!stop)
                logger->log("SIODemoForm::receive: Error while trying to read data!");
            else {
                logger->log("SIODemoForm::receive: Waiting for data interrupted");
            }
            enableUIControls();
            setCursor(QCursor(Qt::ArrowCursor));
            receivedDataTextEdit->setCursor(QCursor(Qt::ArrowCursor));
            qApp->processEvents();
            return;
        }
    } while(unknownCheckBox->isChecked() && !stop);
    
    setCursor(QCursor(Qt::ArrowCursor));
    receivedDataTextEdit->setCursor(QCursor(Qt::ArrowCursor));
    qApp->processEvents();
    
    // enable UI-Controls if no error occured
    enableUIControls();
    responseByteSpinBox->setEnabled(responseByteRadioButton->isChecked() &&
                                    (readCount > 0));
    responseCharLineEdit->setEnabled(responseCharRadioButton->isChecked() &&
                                     (readCount > 0));
//    transmitPushButton->setEnabled(readCount > 0);
}


void SIODemoForm::saveAsSelected() {
    QString fileName;
    QFile *file;
    
    fileName = QFileDialog::getSaveFileName();
    
    if(fileName == NULL)
        return;
    
    file = new QFile(fileName);
    
    if(!file->open(IO_Raw | IO_ReadWrite)) {
        QMessageBox::warning(this, "Error",
                             QString("Error while trying to open ") + fileName,
                             QMessageBox::Ok | QMessageBox::Default,
                             QMessageBox::NoButton,
                             QMessageBox::NoButton);
        return;
    }
    
    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
    
    if(file->writeBlock((receivedDataTextEdit->text()).ascii(), receivedDataTextEdit->length()) < 0) {
        QMessageBox::warning(this, "Write Error",
                             "Error while trying to save data into file!",
                             QMessageBox::Ok | QMessageBox::Default,
                             QMessageBox::NoButton,
                             QMessageBox::NoButton);
        logger->log("SIODemoForm::saveAsSelected: Write Error while trying to save data!");
    }
    
    file->flush();
    file->close();
    
    QApplication::restoreOverrideCursor();
}


void SIODemoForm::transmit() {
    char byte;
    
    
    if(fileTransmissionRadioButton->isChecked()) {
        // transmit data from a file
        if(!dc1394->sendSerialData(id, fileContents.data(), fileContents.size())) {
            QMessageBox::warning(this, "Data-Transmission-Error",
                                 "Error while trying to transmit data via serial interface!",
                                 QMessageBox::Ok | QMessageBox::Default,
                                 QMessageBox::NoButton,
                                 QMessageBox::NoButton);
            logger->log("SerialIOTransmitForm::transmitData: Error while trying to send data!");
        }
        return;
    }
    
    byte = (responseByteRadioButton->isChecked() ?
            responseByteSpinBox->value() :
            *(responseCharLineEdit->text()).ascii());
    
    QApplication::setOverrideCursor(QCursor(Qt::WaitCursor));
    receiveGroupBox->setEnabled(false);
    transmitButtonGroup->setEnabled(false);
    
    // setup SIO
    if(!prepareSIO(false)) {
        logger->log("SerialIOTransmitForm:transmitData: Error while trying to 'setSerialControl()'");
        receiveGroupBox->setEnabled(true);
        transmitButtonGroup->setEnabled(true);
        QApplication::restoreOverrideCursor();
        return;
    }
    
    // send 'byte' via SIO
    if(!dc1394->sendSerialData(id, &byte, 1)) {
        QMessageBox::warning(this, "Data-Transmission-Error",
                             "Error while trying to transmit data via serial interface!",
                             QMessageBox::Ok | QMessageBox::Default,
                             QMessageBox::NoButton,
                             QMessageBox::NoButton);
        logger->log("SIODemoForm::sendResponse: Error while trying to respond with '%d'", byte);
    }
    
    receiveGroupBox->setEnabled(true);
    transmitButtonGroup->setEnabled(true);
    QApplication::restoreOverrideCursor();
}


void SIODemoForm::endlessChecked(bool endless) {
    timeoutSpinBox->setEnabled(!endless);
}


void SIODemoForm::unknownChecked( bool unknown ) {
    byteNoSpinBox->setEnabled(!unknown);
    endlessCheckBox->setEnabled(unknown);
    timeoutSpinBox->setEnabled(unknown && !endlessCheckBox->isChecked());
    receivePushButton->setEnabled(unknown || (byteNoSpinBox->value() > 0));
}


bool SIODemoForm::prepareSIO(bool read) {
    if(!dc1394->setSerialControl(id, read, // Rx enable
                                 !read, // Tx disable
                                 true)) { // clear 'Error-Flags'
        return false;
    }
    
    return true;
}


bool SIODemoForm::waitForDataAvailability() {
    int retries = 0;
    bool rxEnabled = false, txEnabled = false;
    bool rxReady = false, txReady = false;
    bool ovErr = false, frameErr = false, parErr = false;
    
    
    do {
        if(!dc1394->getSerialControl(id, &rxEnabled, &txEnabled,
                                     &rxReady, &txReady,
                                     &ovErr, &frameErr, &parErr)) {
            logger->log("SIODemoForm::waitForDataAvailability: Error while trying to read 'SERIAL_STATUS_CONTROL'!");
            return false;
        }
        
        if(rxReady) {
            return true;
        }
        
        logger->log("SIODemoForm::waitForDataAvailability: no data available - retrying...");
        
        qApp->processEvents(); // give the 'Stop'-Button a real chance...
        sleep(1); // wait 1s before trying to read the status again
    } while(!stop &&
            (endlessCheckBox->isChecked() ? true : (retries++ < timeoutSpinBox->value())));
    
    return rxReady;
}


bool SIODemoForm::waitForData() {
    bool rxEnabled = false, txEnabled = false;
    bool rxReady = false, txReady = false;
    bool ovErr = false, frameErr = false, parErr = false;
    
    
    // wait for the bytes to come
    while(!stop) {
        qApp->processEvents();
        
        if(!dc1394->getNoOfRxBytes(id, &receiveCount)) {
            logger->log("SerialIORecieveForm::waitForData: Error while trying to read 'RECEIVE_BUFFER_STATUS_CONTROL'!");
            return false;
        }
        
        if(receiveCount == 0) {
            // check 'SERIAL_STATUS_REG.RDRD'-Flag
            if(!dc1394->getSerialControl(id, &rxEnabled, &txEnabled,
                                         &rxReady, &txReady,
                                         &ovErr, &frameErr, &parErr)) {
                logger->log("SIODemoForm::waitForData: Error while trying to read 'SERIAL_STATUS_CONTROL'!");
                return false;
            }
            
            // no data available - RDRD == 1 ?
            if(rxReady) {
                logger->log("SIODemoForm::waitForData: Nothing read but ready to receive!");
                // wait 1 s
                sleep(1);
                // and try again
                continue;
            }
            else {
                // No (more) data available
                logger->log("SIODemoForm::waitForData: Buffer empty - no more bytes expected");
                if(unknownCheckBox->isChecked())
                    continue;
                else
                    break;
            }
        }
        
        // if No of bytes is well known but the expected No of bytes is not yet reached - wait for more
        if(!unknownCheckBox->isChecked() &&
           (receiveCount < (unsigned int)byteNoSpinBox->value())) {
            logger->log("SerialIOReceiverForm::waitForData: still bytes to come...");
            continue;
        }
        else {
            // No of expected bytes reached (if known!)
            break;
        }
    } // while(!stop)
    
    return true;
}


void SIODemoForm::accept() {
    parent->raise();
    QDialog::accept();
}


void SIODemoForm::reject() {
    parent->raise();
    QDialog::accept();
}


bool SIODemoForm::readData() {
    bool rxEnabled, txEnabled, rxBufReady, txBufReady, ovErr, frameErr, parErr;
    unsigned int expectedBytes = (receiveCount > 4) ? 4 : receiveCount;
    int n = 0;
    
    
//    while(!stop && (unknownCheckBox->isChecked() ? true : (receiveCount > 0))) {
    while(!stop && (receiveCount > 0)) {
        if((n = dc1394->readSerialData(id, &buffer[0], expectedBytes)) < 0) {
            QMessageBox::warning(this, "Read-Error",
                                 QString("Error while reading byte No. ") +
                                 QString::number(readCount) +
                                 QString(" via serial interface!"),
                                 QMessageBox::Ok | QMessageBox::Default,
                                 QMessageBox::NoButton,
                                 QMessageBox::NoButton);
            logger->log("SIODemoForm::readData: Error while trying to read bytes");
            return false;
        }
        
        // if nothing was read - continue (after waiting for 1s)
        if(n == 0) {
            logger->log("SIODemoForm::readData: Nothing read - still trying...");
            sleep(1);
            qApp->processEvents();
            continue;
        }
        
        if(n < (int)expectedBytes) {
            logger->log("SIODemoForm::readData: %d bytes expected, but only %d bytes received",
                        expectedBytes, n);
        }
        
        // display data
        for(int i = 0; i < n; i++) {
            receivedDataTextEdit->append(QString::number(readCount + i) +
                                         QString(": 0x") +
                                         QString::number((unsigned int)buffer[i], 16) +
                                         ((buffer[i] > 0x20) ? QString(QString(" (") +
                                                                       QString(QChar(buffer[i])) +
                                                                       QString(") ")) :
                                          QString("")));
        }
        
        readCount += n;
        
        // get remaining No of bytes
        if(!dc1394->getNoOfRxBytes(id, &receiveCount)) {
            logger->log("SerialIORecieveForm::readData: Error while trying to read 'RECEIVE_BUFFER_STATUS_CONTROL'!");
            return false;
        }
        
        if(receiveCount <= 0) {
            logger->log("SerialIORecieveForm::readData: No more bytes to come... checking...");
            // read RDRD-flag
            if(!dc1394->getSerialControl(id, &rxEnabled, &txEnabled,
                                         &rxBufReady, &txBufReady,
                                         &ovErr, &frameErr, &parErr)) {
                logger->log("SerialIORecieveForm::readData: Error while trying to read 'SERIAL_STATUS/CONTROL_REG'");
                return false;
            }
            
            if(ovErr || frameErr || parErr) {
                QMessageBox::warning(this, "Communication Error", "A communication-error occured!",
                                     QMessageBox::Ok | QMessageBox::Default  | QMessageBox::Escape,
                                     QMessageBox::NoButton,
                                     QMessageBox::NoButton);
                logger->log("SerialIORecieveForm::readData:\n\tOverrunError-Flag: %d\n\tFramingError-Flag: %d\n\tParityErrorFlag: %d",
                       ovErr ? 1 : 0,
                       frameErr ? 1 : 0,
                       parErr ? 1 : 0);
                return false;
            }
            // if this is (also) '0' - stop waiting for bytes
            if(!unknownCheckBox->isChecked() && (rxBufReady == 0)) {
                break;
            }
            logger->log("SerialIORecieveForm::readData: Still bytes expected...");
        }
        else {
            logger->log("SerialIORecieveForm::readData: %d bytes to come", receiveCount);
        }
        
        expectedBytes = (receiveCount > 4) ? 4 : receiveCount;
        
        qApp->processEvents(); // process pending events (e.g. Stop-Button)
    } // while(!stop)
    
    return true;
}


void SIODemoForm::stopReading() {
    stop = true;
}


void SIODemoForm::disableUIControls() {
    adjustmentsFrame->setEnabled(false); // disable all 'receive' adjustments
    receivePushButton->setEnabled(false); // disable 'Receive' while trying to receive
    receivedDataTextEdit->setEnabled(false);
    stopPushButton->setEnabled(true); // enable 'Stop' while trying to receive
    saveAsPushButton->setEnabled(false); // 'save As...' only possible when data is available!
    transmitButtonGroup->setEnabled(false); // disable all 'response' controls
    clearPushButton->setEnabled(false);
    closePushButton->setEnabled(false);
}


void SIODemoForm::enableUIControls() {
    adjustmentsFrame->setEnabled(true);
    receivePushButton->setEnabled(unknownCheckBox->isChecked() ||
                                  (byteNoSpinBox->value() > 0));
    receivedDataTextEdit->setEnabled(true);
    stopPushButton->setEnabled(false);
    saveAsPushButton->setEnabled(receivedDataTextEdit->length() > 0);
    transmitButtonGroup->setEnabled(true);
    clearPushButton->setEnabled(receivedDataTextEdit->length() > 0);
    closePushButton->setEnabled(true);
}


void SIODemoForm::clearText() {
    if(QMessageBox::question(this, "Clear Text", "Are you sure to delete the text",
                             QMessageBox::Yes | QMessageBox::Default,
                             QMessageBox::No | QMessageBox::Escape,
                             QMessageBox::NoButton) == QMessageBox::Yes) {
        receivedDataTextEdit->clear();
        clearPushButton->setEnabled(false);
        saveAsPushButton->setEnabled(false);
    }
}


void SIODemoForm::byteSelected( bool enabled ) {
    if(enabled) {
        responseByteSpinBox->setEnabled(true);
        responseCharLineEdit->setEnabled(false);
    }
}


void SIODemoForm::charSelected( bool enabled ) {
    if(enabled) {
        responseByteSpinBox->setEnabled(false);
        responseCharLineEdit->setEnabled(true);
    }
}


void SIODemoForm::loadTransmissionFile() {
    QString fileName;
    QFile *file;
    
    
    fileName = QFileDialog::getOpenFileName();
    
    if(fileName.isNull()) {
        return;
    }
    logger->log("SerialIOTransmitForm::loadData: %s selected", fileName.ascii());
    
    file = new QFile(fileName);
    
    if(!file->open(IO_Raw | IO_ReadOnly)) {
        QMessageBox::warning(this, "Error",
                             QString("Error while trying to open ") + fileName,
                             QMessageBox::Ok | QMessageBox::Default,
                             QMessageBox::NoButton,
                             QMessageBox::NoButton);
        logger->log("SerialIOTransmitForm::loadData: Error while trying to open %s", fileName.ascii());
        
        transmitPushButton->setEnabled(false);
        delete file;
        return;
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
        logger->log("SerialIOTransmitForm::loadData: File is empty!");
        
        transmitPushButton->setEnabled(false);
        file->close();
        delete file;
        return;
    }
    
    transmitPushButton->setEnabled(true);
}


void SIODemoForm::singleByteTransmission( bool enabled ) {
    if(enabled) {
        singleTransmissionButtonGroup->setEnabled(true);
        loadDataPushButton->setEnabled(false);
        
        transmitPushButton->setEnabled(true);
    }
}


void SIODemoForm::fileTransmission( bool enabled ) {
    if(enabled) {
        singleTransmissionButtonGroup->setEnabled(false);
        loadDataPushButton->setEnabled(true);
        
        transmitPushButton->setEnabled(false);
    }
}
