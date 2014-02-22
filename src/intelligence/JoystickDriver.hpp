#ifndef JOYSTICKDRIVER_HPP
#define JOYSTICKDRIVER_HPP

#include <hardware/sensors/joystick/Joystick.h>
#include <common/logger/logger.h>
#include <hardware/actuators/motors/MotorDriver.hpp>
#include <common/config/configmanager.h>
#include <QObject>

/*!
 * \brief Maps joystick data to motion commands for manual drive control.
 * \author Matthew Barulic
 */
class JoystickDriver : public QObject
{
    Q_OBJECT
public:
    JoystickDriver(Joystick *joystick)
        : _joystick(joystick)
    {
        connect(_joystick, &Joystick::onNewData, [=](JoystickState state){
            double maxVel = ConfigManager::Instance().getValue("Joystick", "MaxSpeed", 1.0);
            int leftJoyAxis = ConfigManager::Instance().getValue("Joystick", "LeftAxis", 1);
            int rightJoyAxis = ConfigManager::Instance().getValue("Joystick", "RightAxis", 3);
            double left = (state.axes[leftJoyAxis]/32767)*maxVel;
            double right = (state.axes[rightJoyAxis]/32767)*maxVel;
            onNewMotorCommand(MotorCommand(left,right));
        });
    }

    ~JoystickDriver() { }

signals:
    void onNewMotorCommand(MotorCommand);

private:
    Joystick *_joystick;
};

#endif // JOYSTICKDRIVER_HPP
