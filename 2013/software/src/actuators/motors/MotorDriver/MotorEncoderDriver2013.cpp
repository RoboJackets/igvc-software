#include "MotorEncoderDriver2013.h"
#include <sstream>
#include <iostream>

MotorEncoderDriver2013::MotorEncoderDriver2013()
 : _arduino("/dev/igvc_2013_motor_arduino", 9600)
{
    _leftVel = 0;
    _rightVel = 0;
    _maxVel = 1.0;
    writeVelocities();
}

double MotorEncoderDriver2013::getLeftVelocity()
{
    return _leftVel;
}

double MotorEncoderDriver2013::getRightVelocity()
{
    return _rightVel;
}

void MotorEncoderDriver2013::stop()
{
    _leftVel = 0;
    _rightVel = 0;
    writeVelocities();
}

void MotorEncoderDriver2013::setVelocities(double left, double right)
{
    _leftVel = left;
    _rightVel = right;
    writeVelocities();
}

void MotorEncoderDriver2013::setLeftVelocity(double vel)
{
    _leftVel = vel;
    writeVelocities();
}

void MotorEncoderDriver2013::setRightVelocity(double vel)
{
    _rightVel = vel;
    writeVelocities();
}

void MotorEncoderDriver2013::writeVelocities()
{
    std::ostringstream msg;
    if(abs(_leftVel) > _maxVel)
        _leftVel = ( _leftVel > 0 ? 1 : -1) * _maxVel;
    if(abs(_rightVel) > _maxVel)
        _rightVel = ( _rightVel > 0 ? 1 : -1) * _maxVel;

    int Ldir = std::signbit( _leftVel);
    int Rdir = std::signbit(_rightVel);

    int Lpwm = ( abs( _leftVel) / _maxVel ) * 255;
    int Rpwm = ( abs(_rightVel) / _maxVel ) * 255;

    if(Ldir) Lpwm = 255 - Lpwm;
    if(Rdir) Rpwm = 255 - Rpwm;

    msg << "SW" << Rdir << " " << Rpwm << " " << Ldir << " " << Lpwm;
    _arduino.write(msg.str());
}

MotorEncoderDriver2013::~MotorEncoderDriver2013()
{
    _arduino.close();
}
