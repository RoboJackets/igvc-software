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

void MessageLogger::init() {
    // clear message-area
    logTextEdit->clear();
    logTextEdit->setReadOnly(true);
    
    textChanged = false;
    qdebug = false;
    saveBeforeDelete = false;
}


void MessageLogger::destroy() {
    if(saveBeforeDelete) {
        file->writeBlock((logTextEdit->text()).ascii(), logTextEdit->length());
    
        file->close();
        delete file;
    }
}


void MessageLogger::log(QString msg) {
    if(!qdebug)
        textChanged = true;
    
#ifdef MESSAGELOGGER
    qdebug ? qDebug(msg) : logTextEdit->append(msg);
#endif
}


void MessageLogger::log(const char *msg, ...) {
    QString message = QString("");
    va_list ap;
    const char *p, *sval;
    int ival;
    long long lval;
    
    
    if(!qdebug)
        textChanged = true;
    
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
        case 'X':
            lval = va_arg(ap, long long);
            message += QString::number((Q_ULLONG)lval, 16);
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
    
    
#ifdef MESSAGELOGGER
    qdebug ? qDebug(message) : logTextEdit->append(message);
#endif
}


void MessageLogger::clearMessages() {
    // really clear?
    if(QMessageBox::information(this, "Message Logger",
                                "Keep in mind, there's no UNDO...\n\nDelete all messages?",
                                QMessageBox::No | QMessageBox::Default | QMessageBox::Escape,
                                QMessageBox::Yes,
                                QMessageBox::NoButton) == QMessageBox::No) {
        return;
    }
    
    // clear logging-area
    logTextEdit->clear();
    
    textChanged = false;
}


void MessageLogger::saveClicked() {
    saveMessages(false);
}


bool MessageLogger::saved() {
#ifdef MESSAGELOGGER
    return !textChanged;
#else
    return true;
#endif
}


void MessageLogger::closed() {
    qdebug = true;
}


void MessageLogger::saveMessages(bool exitApp) {
    bool fileSelected = false;
    QString fileName;
    
    
    do{
        fileName = QFileDialog::getSaveFileName("./cc1394.log", "*.log");
        
        if(fileName.isNull())
            return;
        
        file = new QFile(fileName);
        
        if(file->exists()) {
            switch(QMessageBox::warning(this, "cc1394",
                                        "File exists - Overwrite?",
                                        QMessageBox::Yes,
                                        QMessageBox::No | QMessageBox::Default,
                                        QMessageBox::Cancel | QMessageBox::Escape)) {
            case QMessageBox::Yes:
                fileSelected = true;
                break;
            case QMessageBox::No:
                delete file;
                break;
            case QMessageBox::Cancel:
                delete file;
                return;
                break;
            default:
                break;
            }
        }
    }while(!fileSelected);
    
    // open File
    if(!file->open(IO_Raw | IO_ReadWrite | IO_Truncate)) {
        QMessageBox::warning(this, "Error",
                             QString("Error while trying to open ") + fileName,
                             QMessageBox::Ok | QMessageBox::Default,
                             QMessageBox::NoButton,
                             QMessageBox::NoButton);
        delete file;
        return;
    }
    
    if(exitApp) {
        saveBeforeDelete = true;
        return;
    }
    
    // write messages to file
    file->writeBlock((logTextEdit->text()).ascii(), logTextEdit->length());
    
    // close File (and delete object)
    file->close();
    delete file;
    
    // ask for 'cleaning' the message-area (if not exiting the application)
    if(QMessageBox::information(this, "Message Logger",
                                "Clear messages?",
                                QMessageBox::Yes | QMessageBox::Default,
                                QMessageBox::No | QMessageBox::Escape,
                                QMessageBox::NoButton) == QMessageBox::Yes)
        // clear message-area
        logTextEdit->clear();
        
    textChanged = false;
}

