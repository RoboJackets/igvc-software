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
    JoystickDriver(std::shared_ptr<Joystick> joystick)
        : _joystick(joystick)
    {
        connect(_joystick.get(), &Joystick::onNewData, [=](JoystickState state){

            double AbsoluteMaxVel = ConfigManager::Instance().getValue("Joystick", "AbsoluteMaxSpeed", 1.0);
            double maxVel = ConfigManager::Instance().getValue("Joystick", "MaxSpeed", 1.0);
            double maxVelIncr = ConfigManager::Instance().getValue("Joystick", "MaxSpeedIncrement", 0.1);

            if(state.buttons[1]) //Button 2
            {
                maxVel -= maxVelIncr;
            }
            else if(state.buttons[3]) //Button 4ff
            {
                maxVel += maxVelIncr;
            }
            if(maxVel > AbsoluteMaxVel)
            {
                maxVel = AbsoluteMaxVel;
            }
            if(maxVel < 0)
            {
                maxVel = 0;
            }

            ConfigManager::Instance().setValue("Joystick", "MaxSpeed", maxVel);
            std::cout << maxVel << std::endl;

            int leftJoyAxis = ConfigManager::Instance().getValue("Joystick", "LeftAxis", 1);
            int rightJoyAxis = ConfigManager::Instance().getValue("Joystick", "RightAxis", 3);
            bool leftInverted = ConfigManager::Instance().getValue("Joystick", "LeftInverted", true);
            bool rightInverted = ConfigManager::Instance().getValue("Joystick", "RightInverted", true);
            double left = (state.axes[leftJoyAxis]/32767.0)*maxVel * (leftInverted ? -1.0 : 1.0);
            double right = (state.axes[rightJoyAxis]/32767.0)*maxVel * (rightInverted ? -1.0 : 1.0);
            onNewMotorCommand(MotorCommand(left,right));
        });
    }

    ~JoystickDriver() { }

signals:
    void onNewMotorCommand(MotorCommand);

private:
    std::shared_ptr<Joystick> _joystick;

};

#endif // JOYSTICKDRIVER_HPP
