#ifndef JOYSTICKDRIVER_HPP
#define JOYSTICKDRIVER_HPP

#include <hardware/sensors/joystick/Joystick.h>
#include <common/logger/logger.h>
#include <hardware/actuators/motors/MotorDriver.hpp>
#include <common/config/configmanager.h>

class JoystickDriver
{
public:
    JoystickDriver(Event<JoystickState> *event) :
        _inputEvent(event),
        LOnNewJoystickEvent(this)
    {
        (*_inputEvent) += &LOnNewJoystickEvent;
    }

    ~JoystickDriver()
    {
        (*_inputEvent) -= &LOnNewJoystickEvent;
    }

    Event<MotorCommand> controlEvent;

private:
    Event<JoystickState> *_inputEvent;

    void OnNewJoystickEvent(JoystickState state)
    {
        double maxVel = ConfigManager::Instance().getValue("Joystick", "MaxSpeed", 1.0);
        double left = (state.axes[1]/32767)*maxVel;
        double right = (state.axes[2]/32767)*maxVel;
        controlEvent(MotorCommand(left,right));
    }
    LISTENER(JoystickDriver, OnNewJoystickEvent, JoystickState)
};

#endif // JOYSTICKDRIVER_HPP
