#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

#include <QObject>

struct MotorCommand
{
    double leftVel;
    double rightVel;
    int millis;
    bool timed;

    MotorCommand(double left, double right) :
        leftVel(left),
        rightVel(right),
        timed(false)
    { }

    MotorCommand(double left, double right, int ms) :
        leftVel(left),
        rightVel(right),
        millis(ms),
        timed(true)
    { }
};

class MotorDriver : public QObject
{
    Q_OBJECT
public slots:
    virtual void setMotorCommand(MotorCommand) = 0;

public:
    virtual ~MotorDriver() { }

    /*
     * Writes the given velocities out the motors.
     */
    virtual void setVelocities(double left, double right, int millis = 0) = 0;

    /*
     * Writes the given velocity to the left motors.
     */
    virtual void setLeftVelocity(double vel, int millis = 0) = 0;

    /*
     * Writes the given velocity to the right motors.
     */
    virtual void setRightVelocity(double vel, int millis = 0) = 0;

    /*
     * Returns the velocity currently being written to the left motor.
     */
    virtual double getLeftVelocity() = 0;

    /*
     * Returns the velocity currently being written to the right motor.
     */
    virtual double getRightVelocity() = 0;

    /*
     * Writes zero velocities to both motors.
     */
    virtual void stop() = 0;

    /*
     * Returns the connection status of this motor driver.
     */
    virtual bool isOpen() = 0;
};

#endif // MOTORDRIVER_H
