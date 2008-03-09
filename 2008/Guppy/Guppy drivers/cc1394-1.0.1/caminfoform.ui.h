/****************************************************************************
** ui.h extension file, included from the uic-generated form implementation.
**
** If you wish to add, delete or rename functions or slots use
** Qt Designer which will update this file, preserving your code. Create an
** init() function in place of a constructor, and a destroy() function in
** place of a destructor.
*****************************************************************************/


void CamInfoForm::setParameters(char *vendor, char *model, u_int64_t euid, octlet_t ccrOffset,
                                quadlet_t specVersion, quadlet_t revision,
                                unsigned int uCMajor, unsigned int uCMinor,
                                unsigned int fwCamID, unsigned int fwMajor,
                                unsigned int fwMinor, unsigned int portNo,
                                unsigned int nodeNo) {
    
    // camera specific information
    
    vendorLineEdit->setText(vendor);
    
    modelLineEdit->setText(model + QString(" (ID: ") + QString::number(fwCamID) + QString(")"));
    
    uCVersionLineEdit->setText(QString::number(uCMajor) + "." +
                               QString(uCMinor < 10 ? "0" : "") +
                               QString::number(uCMinor));
    
    firmwareVersionLineEdit->setText(QString::number(fwMajor) + "v" +
                                     QString(fwMinor < 10 ? "0" : "") +
                                     QString::number(fwMinor));
    
    switch(specVersion) {
    case 0x100: // version 1.04
        specVersionLineEdit->setText(QString("1.04"));
        break;
    case 0x101: // version 1.20
        specVersionLineEdit->setText(QString("1.20"));
        break;
    case 0x102: // version 1.30
        specVersionLineEdit->setText(QString("1.30"));
        break;
    default:
        if(specVersion > 0x102)
            specVersionLineEdit->setText(">1.30");
        else
            specVersionLineEdit->setText(QString("0x") + QString::number(specVersion, 16));
        break;
    }
    
    switch(revision) {
    case 0:
        revisionLineEdit->setText("F6 not supported");
        break;
    case 1:
        revisionLineEdit->setText("EXIF V2.0 supported");
        break;
    default:
        revisionLineEdit->setText("Revision (" + QString::number(revision) + ") unknown!");
        break;
    }
    
    euidLineEdit->setText(QString::number(euid));
    
    ccrOffsetLineEdit->setText(QString("0x") + QString::number(ccrOffset, 16).upper());
    
    
    // port specific information
    
    portLineEdit->setText(QString::number(portNo));
    nodeLineEdit->setText(QString::number(nodeNo));
    
}
