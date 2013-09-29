#ifndef JOYSTICKDRIVER_HPP
#define JOYSTICKDRIVER_HPP

#include "hardware/sensors/joystick/Joystick.h"
#include "common/logger/logger.h"
#include "hardware/actuators/motors/MotorDriver.hpp"

class JoystickDriver
{
public:
    JoystickDriver(Event<JoystickState> *event) :
        _inputEvent(event),
        LOnNewJoystickEvent(this)
    {
        (*_inputEvent) += &LOnNewJoystickEvent;
        _maxVel = 1;
    }

    ~JoystickDriver()
    {
        (*_inputEvent) -= &LOnNewJoystickEvent;
    }

    Event<MotorCommand> controlEvent;

private:
    Event<JoystickState> *_inputEvent;

    double _maxVel;

    void OnNewJoystickEvent(JoystickState state)
    {
        double left = (state.axes[1]/32767)*_maxVel;
        double right = (state.axes[2]/32767)*_maxVel;
        controlEvent(MotorCommand(left,right));
    }
    LISTENER(JoystickDriver, OnNewJoystickEvent, JoystickState)
};

#endif // JOYSTICKDRIVER_HPP
