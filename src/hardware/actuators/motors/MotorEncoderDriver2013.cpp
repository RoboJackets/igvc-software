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
    qRegisterMetaType<MotorCommand>("MotorCommand");
    _leftVel = 0;
    _rightVel = 0;
    _maxVel = 2.0;
    if(_arduino.isConnected())
    {
        cout << "Waiting for motor arduino.";
        cout.flush();
        string line;
        while((line = _arduino.readln()).compare("Ready"))
        {
            cout << ".";
            cout.flush();
            usleep(250);
        }
        cout << endl;
        Logger::Log(LogLevel::Info, "Motor arduino connected.");
        _thread = boost::thread(boost::bind(&MotorEncoderDriver2013::run, this));
    }
    else
    {
        Logger::Log(LogLevel::Warning, "Motor arduino not connected. Commands will be ignored.");
    }
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
        usleep(10000);
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
}

void MotorEncoderDriver2013::setVelocities(double left, double right)
{
    _leftVel = left;
    _rightVel = right;
}

void MotorEncoderDriver2013::setLeftVelocity(double vel)
{
    _leftVel = vel;
}

void MotorEncoderDriver2013::setRightVelocity(double vel)
{
    _rightVel = vel;
}

void MotorEncoderDriver2013::writeVelocities()
{
    if(_arduino.isConnected())
    {
        /*
         * The newline character below must be there.
         * In the Arduino code, Serial.parseFloat is used.
         * This command will slow down significantly if
         * you do not follow the number with a non-numeric
         * character. Here, the newline does this for us.
         *
         * Checking the E-Stop status prevents the PID
         * controller from ramping up to full PWM when
         * the E-Stop is causing all PWMs to map to
         * 0 m/s.
         */
        string msg = "$" + to_string(_isEnabled ? _leftVel : 0) + "," + to_string(_isEnabled ? _rightVel : 0) + "\n";

        _portLock.lock();
        try{
            _arduino.write(msg);
        }catch(...){
            Logger::Log(LogLevel::Error, "Error writing velocities to motors.");
        }

        std::string ret = _arduino.readln();
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

void MotorEncoderDriver2013::onEStopStatusChanged(bool isEnabled)
{
    _isEnabled = isEnabled;
}
