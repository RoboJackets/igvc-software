#include <actuators/motors/MotorDriver/MotorEncoderDriver2013.h>
#include <iostream>

using namespace std;

int main()
{
    MotorEncoderDriver2013 driver;

    cout << "Forward" << endl;

    for(double speed = 0; speed < 1.0; speed += 0.05)
    {
        driver.setVelocities(speed, speed);
        cout << "\t" << speed << endl;
        sleep(1);
    }

    cout << "Backward" << endl;
    for(double speed = 0; speed > -1.0; speed -= 0.05)
    {
        driver.setVelocities(speed, speed);
        cout << "\t" << speed << endl;
        sleep(1);
    }

    cout << "Stop" << endl;
    driver.stop();

    cout << "Done" << endl;
}
