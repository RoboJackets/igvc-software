#include "lightcontroller.h"
#include <common/logger/logger.h>

LightController::LightController()
    : _port("/dev/igvc_light_arduino", 9600)
{
    _safetyIsFlashing = false;
    _underglowRedChannel = _underglowGreenChannel = _underglowBlueChannel = 0;
    _underglowBar1Sec1 = _underglowBar1Sec2 = _underglowBar1Sec3 = 0;
    _underglowBar2Sec1 = _underglowBar2Sec2 = _underglowBar2Sec3 = 0;
    if( !_port.isConnected() )
    {
        return;
    }
    this->start();
}

void LightController::run()
{
    this->sleep(2); // Gives the Arduino time to boot
    while(!this->isInterruptionRequested())
    {
        uchar msg[12] = {2,
                        _safetyIsFlashing,
                        _underglowRedChannel,
                        _underglowGreenChannel,
                        _underglowBlueChannel,
                        _underglowBar1Sec1,
                        _underglowBar1Sec2,
                        _underglowBar1Sec3,
                        _underglowBar2Sec1,
                        _underglowBar2Sec2,
                        _underglowBar2Sec3,
                        4};
        _port.write(msg, 12);
        uchar *ret = (uchar*)_port.read(4);
        if(ret[0] != 2 || ret[3] != 4)
            Logger::Log(LogLevel::Error, "[LightController]\tBad format for return packet.");
        else if(ret[1] == 21)
        {
            Logger::Log(LogLevel::Info, tr("[LightController]\tArduino gave error code %1.").arg(ret[2]).toStdString());
        }
        else
        {
            if(ret[1] != _estopStatus)
            {
                _estopStatus = ret[1];
                onEStopStatusChanged(_estopStatus);
            }
            if(ret[2] != _batteryLevel)
            {
                _batteryLevel = ret[2];
                onBatteryLevelChanged(_batteryLevel * 100 / 255);
            }
        }
        this->msleep(100);
    }
}

LightController::~LightController()
{
    this->requestInterruption();
    while(this->isRunning()) { }
    _port.close();
}
