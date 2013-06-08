#include <iostream>
#include <sensors/joystick/Joystick.h>
//#include <actuators/motors/OSMC_driver/OSMC_driver.hpp>
#include <actuators/motors/MotorDriver/MotorEncoderDriver2013.h>
#include <sys/time.h>
#include <boost/thread.hpp>

using namespace std;

class TankDriver
{
    private:
    Joystick *_joy;
    //OSMC_driver *_driver;
    MotorDriver *_driver;
    LISTENER(TankDriver, onNewJoystick, JoystickState);
    volatile int _LPWM, _RPWM, _LDIR, _RDIR;

    public:
    TankDriver(Joystick *joystick, MotorDriver *driver)
        : LonNewJoystick(this)
    {
        _joy = joystick;
        _joy->onNewData += &LonNewJoystick;
        _driver = driver;
        _driver->stop();
        _LDIR = 0;
        _LPWM = 0;
        _RDIR = 0;
        _RPWM = 0;
    }

    void onNewJoystick(JoystickState state)
    {
        int LIn = state.axes[1];
        int RIn = state.axes[3];

        double maxVal = 32767.0;

        /*_LPWM = abs( ( LIn / maxVal ) * 150.0 );
        _RPWM= abs( ( RIn / maxVal ) * 150.0 );

        int DEADBAND = 30;
        if(_LPWM < DEADBAND)
        {
            _LPWM = 0;
        }
        if(_RPWM < DEADBAND)
        {
            _RPWM = 0;
        }

        _LDIR = !signbit(LIn);
        _RDIR = !signbit(RIn);

        cout << "Right : " << (_LDIR &&_LPWM? "-" : " ") << _LPWM << "    " << (_RDIR &&_RPWM? "-" : " ") << _RPWM << endl;
        _driver->setRightLeftPwm(_RPWM, _RDIR, _LPWM, _LDIR);*/

        double LSpeed = -(double)LIn / maxVal * 2.0;
        double RSpeed = -(double)RIn / maxVal * 2.0;

        double DEADBAND = 0.15;

        LSpeed *= 1.0;
        RSpeed *= 1.0;

        if(abs(LSpeed) < DEADBAND)
            LSpeed = 0;
        if(abs(RSpeed) < DEADBAND)
            RSpeed = 0;

        cout << LSpeed << "\t" << RSpeed << endl;

        _driver->setVelocities(LSpeed, RSpeed);


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
    Joystick joystick;
    if(joystick.isOpen())
    {
        MotorEncoderDriver2013 driver;
        TankDriver tankdrive(&joystick, &driver);
        cout << "Press button 1 to exit." << endl;
        while(joystick.isOpen()) { }
    } else {
        cout << "Could not open joystick, exiting." << endl;
    }
    return 0;
}
