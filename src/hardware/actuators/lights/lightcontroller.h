#ifndef LIGHTCONTROLLER_H
#define LIGHTCONTROLLER_H

#include <hardware/serial/SerialPort.h>
#include <QThread>
#include <QMutex>

class LightController : public QThread
{
    Q_OBJECT

public:
    LightController();

    ~LightController();

    void setSafetyLight(bool isFlashing)
    {
        _safetyIsFlashing = isFlashing;
    }

    void setUnderglowColor(uchar red, uchar green, uchar blue)
    {
        _underglowRedChannel = red;
        _underglowGreenChannel = green;
        _underglowBlueChannel = blue;
    }

    void setUnderglowBrightness(uchar bar, uchar s1, uchar s2, uchar s3)
    {
        switch(bar)
        {
        case 1:
            _underglowBar1Sec1 = s1;
            _underglowBar1Sec2 = s2;
            _underglowBar1Sec3 = s3;
            break;
        case 2:
            _underglowBar2Sec1 = s1;
            _underglowBar2Sec2 = s2;
            _underglowBar2Sec3 = s3;
            break;
        }
    }

    bool isSafetyLightFlashing()
    {
        return _safetyIsFlashing;
    }

    bool isRobotEnabled()
    {
        return _estopStatus;
    }

    uchar getBatteryLevel()
    {
        return _batteryLevel;
    }

    bool isConnected()
    {
        return _port.isConnected();
    }

signals:
    void onBatteryLevelChanged(int level);
    void onEStopStatusChanged(bool isEnabled);

protected:
    void run();

private:
    uchar _returnPacket[4];
    bool _returnPacketBegun;
    int _returnPacketSize;

    SerialPort _port;

    bool _safetyIsFlashing;

    uchar _underglowRedChannel;
    uchar _underglowGreenChannel;
    uchar _underglowBlueChannel;

    uchar _underglowBar1Sec1;
    uchar _underglowBar1Sec2;
    uchar _underglowBar1Sec3;

    uchar _underglowBar2Sec1;
    uchar _underglowBar2Sec2;
    uchar _underglowBar2Sec3;

    uchar _batteryLevel;
    bool _estopStatus;
};

#endif // LIGHTCONTROLLER_H
