#include "MotorEncoderDriver2013.h"
#include <sstream>
#include <iostream>
#include <common/utils/StringUtils.hpp>
#include <common/logger/logger.h>
#include <algorithm>

using namespace std;

MotorEncoderDriver2013::MotorEncoderDriver2013()
 : _arduino("/dev/igvc_motor_arduino", 9600)
{
    _leftVel = 0;
    _rightVel = 0;
    _maxVel = 2.0;
    //writeVelocities();
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
    _thread = boost::thread(boost::bind(&MotorEncoderDriver2013::run, this));
}

void MotorEncoderDriver2013::run()
{
    while(true)
    {
        try
        {
            boost::this_thread::interruption_point();
        }
        catch (boost::thread_interrupted)
        {
            return;
        }
        writeVelocities();
        //usleep(100000);
        usleep(1000000);
    }
}

double MotorEncoderDriver2013::getLeftSetVelocity()
{
    return _leftVel;
}

double MotorEncoderDriver2013::getRightSetVelocity()
{
    return _rightVel;
}

double MotorEncoderDriver2013::getLeftCurrentVelocity()
{
    return _leftCurrVel;
}

double MotorEncoderDriver2013::getRightCurrentVelocity()
{
    return _rightCurrVel;
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
    if(_arduino.isConnected())
    {
        std::ostringstream msg;

        msg << '$' << _leftVel << ',' << _rightVel;

        _portLock.lock();
        _arduino.write(msg.str());
        std::cout << "Wrote message : " << msg.str() << std::endl;
        std::string ret = _arduino.readln();
        std::cout << "Recieved line : " << ret << std::endl;
        try {
            if(!ret.empty())
            {
                size_t dollar = ret.find('$');
                size_t comma = ret.find(',');
                size_t end = ret.find('\n');
                std::string leftStr = ret.substr(dollar+1, comma-dollar-1);
                std::string rightStr = ret.substr(comma+1, end-comma-1);
                _leftCurrVel = atof(leftStr.c_str());
                _rightCurrVel = atof(rightStr.c_str());
                newCurrentVelocities(_leftCurrVel, _rightCurrVel);
            }
        } catch (std::out_of_range) { }
        _portLock.unlock();
    }
}

void MotorEncoderDriver2013::setMotorCommand(MotorCommand cmd)
{
    setVelocities(cmd.leftVel, cmd.rightVel);
}

bool MotorEncoderDriver2013::isOpen()
{
    return _arduino.isConnected();
}

MotorEncoderDriver2013::~MotorEncoderDriver2013()
{
    stop();
    _arduino.close();
}
