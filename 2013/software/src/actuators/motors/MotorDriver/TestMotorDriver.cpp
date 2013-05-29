#include <actuators/motors/MotorDriver/MotorEncoderDriver2013.h>
#include <iostream>

using namespace std;

class EncoderListener
{
public:
    MotorEncoderDriver2013 *_enc;

    EncoderListener(MotorEncoderDriver2013 *enc)
        : LonNewPose(this)
    {
        _enc = enc;
        enc->onNewPosition += &LonNewPose;
    }

    void onNewPose(EncPose pose)
    {
        cout << pose.x << "\t" << pose.y << "\t" << pose.theta << endl;
    }
    LISTENER(EncoderListener, onNewPose, EncPose);

    ~EncoderListener()
    {
        _enc->onNewPosition -= &LonNewPose;
        _enc = 0;
    }
};

int main()
{
    MotorEncoderDriver2013 driver;

    //EncoderListener list(&driver);

    cout << "initialization done" << endl;

    double v = 0.4;
    double w = -M_PI;

    for(int i = 0; i < 10; i++)
    {
        if(i % 2 == 0)
        {

        }
        else
        {

        }
    }

    double Vr = v + (0.71755/2.0)*w;
    double Vl = v - (0.71755/2.0)*w;
    driver.setVelocities(Vl, Vr);

    cout << "main sleeping..." << endl;

    usleep(0.4*1000000);
    //sleep(10);

    /*cout << "Forward" << endl;

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
    }*/

    cout << "Stop" << endl;
    driver.stop();

    cout << "Done" << endl;
}
