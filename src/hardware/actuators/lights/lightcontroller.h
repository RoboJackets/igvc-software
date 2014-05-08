#ifndef LIGHTCONTROLLER_H
#define LIGHTCONTROLLER_H

#include <QtSerialPort/QtSerialPort>

class LightController
{
public:
    LightController();

private:
    QSerialPort port;
};

#endif // LIGHTCONTROLLER_H
