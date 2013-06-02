#ifndef LIGHTCONTROLLER_HPP_INCLUDED
#define LIGHTCONTROLLER_HPP_INCLUDED

#include <serial/ASIOSerialPort.h>

enum SafetyLightState
{
    Solid,
    Blinking
};

class LightController
{
public:
    LightController()
        : _arduino("/dev/igvc_2013_light_arduino", 9600)
    {
    }

    void SetSafetyLightState(SafetyLightState state)
    {
        switch(state)
        {
        case Solid:
            _arduino.write("SS");
            break;
        case Blinking:
            _arduino.write("SB");
            break;
        }
    }

    ~LightController()
    {
        _arduino.close();
    }

private:
    ASIOSerialPort _arduino;
};

#endif // LIGHTCONTROLLER_HPP_INCLUDED
