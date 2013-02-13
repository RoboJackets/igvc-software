#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <iostream>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <linux/joystick.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <sstream>
#include <boost/thread.hpp>
#include <string>
#include <events/Event.hpp>
#include <time.h>
#define axisInvert -1 // 1 for inverted y analog axes , -1 for non-inverted
#define PI 3.14159
#define setCYCLE 200000UL
using namespace std;

class Joystick
{
    public:
    Joystick(); //Constructor
    bool initJoystick(); //Initialize Joystick
    virtual ~Joystick();
    bool readJoystick(); //Read and Set Joystick Values
    void displayJoystick(); //Display Joystick values
    void getPWM(); // to convert analog readings to byte and get direction
    void smoothPWM(); //Allows for smoother slower acceleration rate for the motor
    bool CycleCheck();
    void startEvents();
    void stopEvents();
    int LYAxis, LXAxis, RYAxis, RXAxis;
    int padYAxis, padXAxis;
    volatile int jLPWM, LPWM, sLPWM;
    volatile int jRPWM, RPWM, sRPWM;
    int LDirection, RDirection;
    timeval cycleTime;
    private:
    int joy_fd;
    int ret;
    vector<int> axis;
    vector<int> B; //Buttons
    unsigned int numAxes, numButtons;
    uint32_t currentMicro;
    uint32_t lastMicro;
    void eventThreadRun();
	bool _eventsEnabled;
	boost::thread eventThread;
};

#endif // JOYSTICK_H
