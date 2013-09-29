#ifndef JOYSTICKDRIVER_HPP
#define JOYSTICKDRIVER_HPP

#include "hardware/sensors/joystick/Joystick.h"
#include "common/logger/logger.h"

class JoystickDriver
{
public:
    JoystickDriver(Event<JoystickState> *event) :
        _event(event),
        LOnNewJoystickEvent(this)
    {
        (*_event) += &LOnNewJoystickEvent;
    }

    ~JoystickDriver()
    {
        (*_event) -= &LOnNewJoystickEvent;
    }

private:
    Event<JoystickState> *_event;

    void OnNewJoystickEvent(JoystickState state)
    {
        Logger::Log(LogLevel::Debug, state.axes[0]);
    }
    LISTENER(JoystickDriver, OnNewJoystickEvent, JoystickState)
};

#endif // JOYSTICKDRIVER_HPP
