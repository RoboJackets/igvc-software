#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

#include <QObject>

struct MotorCommand
{
    double leftVel;
    double rightVel;

    MotorCommand() :
        leftVel(0),
        rightVel(0)
    {
    }

    MotorCommand(double left, double right) :
        leftVel(left),
        rightVel(right)
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
    virtual void setVelocities(double left, double right) = 0;

    /*
     * Writes the given velocity to the left motors.
     */
    virtual void setLeftVelocity(double vel) = 0;

    /*
     * Writes the given velocity to the right motors.
     */
    virtual void setRightVelocity(double vel) = 0;

    /*
     * Returns the velocity currently being written to the left motor.
     */
    virtual double getLeftSetVelocity() = 0;

    /*
     * Returns the velocity currently being written to the right motor.
     */
    virtual double getRightSetVelocity() = 0;

    /*
     * Writes zero velocities to both motors.
     */
    virtual void stop() = 0;

    /*
     * Returns the connection status of this motor driver.
     */
    virtual bool isOpen() = 0;

signals:
    void newCurrentVelocities(double left, double right);
};

#endif // MOTORDRIVER_H
