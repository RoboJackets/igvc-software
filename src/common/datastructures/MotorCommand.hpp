#ifndef MOTORCOMMAND_HPP
#define MOTORCOMMAND_HPP

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

#endif // MOTORCOMMAND_HPP
