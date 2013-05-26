#ifndef MOTORENCODERDRIVER2013_H
#define MOTORENCODERDRIVER2013_H
#include <serial/ASIOSerialPort.h>
#include <actuators/motors/MotorDriver/MotorDriver.hpp>

class MotorEncoderDriver2013 : public MotorDriver
{
    public:
        MotorEncoderDriver2013();
        virtual ~MotorEncoderDriver2013();

        void setVelocities(double left, double right);
        void setLeftVelocity(double vel);
        void setRightVelocity(double vel);
        double getLeftVelocity();
        double getRightVelocity();
        void stop();

    protected:
    private:
        ASIOSerialPort _arduino;
        double _leftVel;
        double _rightVel;
        double _maxVel;
        void writeVelocities();
};

#endif // MOTORENCODERDRIVER2013_H
