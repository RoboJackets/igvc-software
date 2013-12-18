#ifndef JOYSTICKDRIVER_HPP
#define JOYSTICKDRIVER_HPP

#include <hardware/sensors/joystick/Joystick.h>
#include <common/logger/logger.h>
#include <hardware/actuators/motors/MotorDriver.hpp>
#include <common/config/configmanager.h>

/*!
 * \brief Maps joystick data to motion commands for manual drive control.
 * \author Matthew Barulic
 */
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
        int leftJoyAxis = ConfigManager::Instance().getValue("Joystick", "LeftAxis", 1);
        int rightJoyAxis = ConfigManager::Instance().getValue("Joystick", "RightAxis", 3);
        double left = (state.axes[leftJoyAxis]/32767)*maxVel;
        double right = (state.axes[rightJoyAxis]/32767)*maxVel;
        controlEvent(MotorCommand(left,right));
    }
    LISTENER(JoystickDriver, OnNewJoystickEvent, JoystickState)
};

#endif // JOYSTICKDRIVER_HPP
