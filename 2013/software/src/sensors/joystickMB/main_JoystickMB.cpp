#include <iostream>
#include "sensors/joystickMB/JoystickMB.h"
#include "actuators/motors/OSMC_driver/OSMC_driver.hpp"

using namespace std;

class TankDriver
{
    private:
    JoystickMB *_joy;
    OSMC_driver *_driver;
    LISTENER(TankDriver, onNewJoystick, JoystickState);

    public:
    TankDriver(JoystickMB *joystick, OSMC_driver *driver)
        : LonNewJoystick(this)
    {
        _joy = joystick;
        _joy->onNewData += &LonNewJoystick;
        _driver = driver;
        _driver->stopMotors();
    }

    void onNewJoystick(JoystickState state)
    {
        int LIn = state.axes[1];
        int RIn = state.axes[2];

        double maxVal = 32767.0;

        int Lpwm = abs( ( LIn / maxVal ) * 255.0 );
        int Rpwm= abs( ( RIn / maxVal ) * 255.0 );

        int Ldir = signbit(LIn);
        int Rdir = signbit(RIn);

        // set pwm

        _driver->setRightLeftPwm(Rpwm, Rdir, Lpwm, Ldir);

        cout << "Right : " << (Ldir ? "-" : " ") << Lpwm << "    " << (Rdir ? "-" : " ") << Rpwm << endl;

        if(state.buttons[0])
        {
            cout << "exit" << endl;
            _joy->disconnect();
            exit(0);
        }
    }
};

int main()
{
    JoystickMB joystick;
    if(joystick.isOpen())
    {
        OSMC_driver driver;
        TankDriver tankdrive(&joystick, &driver);
        cout << "Press button 1 to exit." << endl;
        while(joystick.isOpen()) { }
    } else {
        cout << "Could not open joystick, exiting." << endl;
    }
}
