#include "MotorEncoderDriver2013.h"
#include <sstream>
#include <iostream>
#include <common/utils/StringUtils.hpp>
#include <common/logger/logger.h>

using namespace std;

MotorEncoderDriver2013::MotorEncoderDriver2013()
 : _arduino("/dev/igvc_2013_motor_arduino", 9600)
{
    _leftVel = 0;
    _rightVel = 0;
    _duration = 0;
    _maxVel = 2.0;
    writeVelocities();
    _running = true;
    _pose.x = 0;
    _pose.y = 0;
    _pose.theta = 0;
    if(_arduino.isConnected())
    {
        cout << "Waiting for motor arduino.";
        cout.flush();
        string line;
        while((line = _arduino.readln()).compare("Ready"))
        {
            //cout << line << endl;
            cout << ".";
            cout.flush();
            usleep(250);
        }
        cout << endl;
        Logger::Log(LogLevel::Info, "Motor arduino connected.");
    }
    else
    {
        Logger::Log(LogLevel::Warning, "Motor arduino not connected. Commands will be ignored.");
    }
    //TODO : Encoder access on Arduino-side code
    //_encThread = boost::thread(boost::bind(&MotorEncoderDriver2013::encThreadRun, this));
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

void MotorEncoderDriver2013::setVelocities(double left, double right, int millis)
{
    _leftVel = left;
    _rightVel = right;
    _duration = millis;
    writeVelocities();
}

void MotorEncoderDriver2013::setLeftVelocity(double vel, int millis)
{
    _leftVel = vel;
    _duration = millis;
    writeVelocities();
}

void MotorEncoderDriver2013::setRightVelocity(double vel, int millis)
{
    _rightVel = vel;
    _duration = millis;
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

    //TODO : duration not yet supported on Arduino-side code
    msg << "SW" << Rdir << " " << Rpwm << " " << Ldir << " " << Lpwm;// << " " << _duration;
    _portLock.lock();
    _arduino.write(msg.str());
    //Logger::Log(LogLevel::Info, msg.str());
    _portLock.unlock();
}

void MotorEncoderDriver2013::setMotorCommand(MotorCommand cmd)
{
    setLeftVelocity(cmd.leftVel, (cmd.timed?cmd.millis:0));
    setRightVelocity(cmd.rightVel, (cmd.timed?cmd.millis:0));
}

void MotorEncoderDriver2013::encThreadRun()
{
    while(_running)
    {
        _portLock.lock();
        _arduino.write("SR");
        string line = _arduino.readln();
        _portLock.unlock();
        while(line.size() > 6 && line[0] != 'A')
            line = line.substr(1);
        if(line.size() < 6)
            continue;
        line = line.substr(1);
        std::vector<string> values = split(line, ' ');
        if(values.size() < 3)
            continue;
        _pose.x = atof(values[0].c_str());
        _pose.y = atof(values[1].c_str());
        _pose.theta = atof(values[2].c_str());
        if(!_running)
            break;
        onNewPosition(_pose);
        usleep(1000);
    }
}

bool MotorEncoderDriver2013::isOpen()
{
    return _arduino.isConnected();
}

MotorEncoderDriver2013::~MotorEncoderDriver2013()
{
    _running = false;
    _encThread.join();
    stop();
    _arduino.close();
}
