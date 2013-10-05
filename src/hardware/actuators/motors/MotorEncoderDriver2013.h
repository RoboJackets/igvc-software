#ifndef MOTORENCODERDRIVER2013_H
#define MOTORENCODERDRIVER2013_H
#include <hardware/serial/ASIOSerialPort.h>
#include <hardware/actuators/motors/MotorDriver.hpp>
#include <common/events/Event.hpp>
#include <boost/thread.hpp>

struct EncPose
{
    double x;
    double y;
    double theta;
};

class MotorEncoderDriver2013 : public MotorDriver
{
    public:
        MotorEncoderDriver2013();
        virtual ~MotorEncoderDriver2013();

        void setVelocities(double left, double right, int millis = 0);
        void setLeftVelocity(double vel, int millis = 0);
        void setRightVelocity(double vel, int millis = 0);
        double getLeftVelocity();
        double getRightVelocity();
        void stop();
        void setControlEvent(Event<MotorCommand> *event);

        bool isOpen();

        Event<EncPose> onNewPosition;

    protected:
    private:
        ASIOSerialPort _arduino;
        double _leftVel;
        double _rightVel;
        int _duration;
        double _maxVel;
        void writeVelocities();
        boost::thread _encThread;
        boost::mutex _portLock;
        void encThreadRun();
        EncPose _pose;
        bool _running;
        Event<MotorCommand> *_controlEvent;
        void onControlEvent(MotorCommand cmd);
        LISTENER(MotorEncoderDriver2013, onControlEvent, MotorCommand);
};

#endif // MOTORENCODERDRIVER2013_H
