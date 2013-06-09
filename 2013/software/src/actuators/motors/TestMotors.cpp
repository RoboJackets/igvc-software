#include <actuators/motors/MotorDriver/MotorEncoderDriver2013.h>
#include <common/Robot.h>
#include <iostream>

using namespace std;

int main()
{
    cout << "WARNING: You have 5 seconds to depress the E-Stop!" << endl;

    for(int i = 5; i > 0; i--)
    {
        cout << i << "..." << endl;
        sleep(1);
    }

    MotorEncoderDriver2013 driver;

    double radius;

    cout << "Radius (m)? : ";
    cout.flush();

    cin >> radius;

    double speed;

    cout << "Speed (m/s)? : ";
    cout.flush();

    cin >> speed;

    int duration;
    cout << "Duration (s)?";
    cout.flush();

    cin >> duration;

    cout << "Please wait..." << endl;

    double w = speed / radius;

    double Baseline = Robot::CurrentRobot().Baseline();
    double Vr = speed + (Baseline/2.0)*w;
    double Vl = speed - (Baseline/2.0)*w;

    cout << "Computed velocities!" << endl;
    cout << "\t" << "VL=" << Vl << endl;
    cout << "\t" << "VR=" << Vr << endl;

    driver.setVelocities(Vl, Vr, duration*1000);

    cout << "Motors set, press green button to engage." << endl;
    cout << "WARNING: Ensure wireless E-Stop is functional to stop robot. Especially if using a duration of 0 seconds." << endl;

    sleep(duration);

    driver.stop();

    return 0;
}
