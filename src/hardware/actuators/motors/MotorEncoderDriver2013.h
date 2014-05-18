#ifndef MOTORENCODERDRIVER2013_H
#define MOTORENCODERDRIVER2013_H
#include <hardware/serial/SerialPort.h>
#include <hardware/actuators/motors/MotorDriver.hpp>
#include <boost/thread.hpp>
#include <QObject>

class MotorEncoderDriver2013 : public MotorDriver
{
    Q_OBJECT
public:
    MotorEncoderDriver2013();
    virtual ~MotorEncoderDriver2013();

    void setVelocities(double left, double right);
    void setLeftVelocity(double vel);
    void setRightVelocity(double vel);
    double getLeftSetVelocity();
    double getRightSetVelocity();

    double getLeftCurrentVelocity();
    double getRightCurrentVelocity();

    void stop();

    void setMotorCommand(MotorCommand);

    bool isOpen();

protected:
    void run();

private:
    SerialPort _arduino;
    double _leftVel;
    double _rightVel;
    double _maxVel;
    void writeVelocities();
    boost::mutex _portLock;

    boost::thread _thread;

    double _leftCurrVel;
    double _rightCurrVel;
};

#endif // MOTORENCODERDRIVER2013_H
