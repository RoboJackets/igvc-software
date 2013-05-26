#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H


class MotorDriver
{
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
        virtual double getLeftVelocity() = 0;

        /*
         * Returns the velocity currently being written to the right motor.
         */
        virtual double getRightVelocity() = 0;

        /*
         * Writes zero velocities to both motors.
         */
        virtual void stop() = 0;
};

#endif // MOTORDRIVER_H
