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
#define axisInvert -1 // 1 for inverted y analog axes , -1 for non-inverted
#define PI 3.14159

using namespace std;

class Joystick
{
    public:
    bool initJoystick(); //Initialize Joystick
    virtual ~Joystick();
    bool readJoystick(); //Read and Set Joystick Values
    void displayJoystick(); //Display Joystick values
    int leftYAxis, leftXAxis, rightYAxis, rightXAxis;
    int padYAxis, padXAxis;

    private:
    int joy_fd;
    int ret;
    vector<int> axis;
    vector<int> B; //Buttons
    unsigned int numAxes, numButtons;
};

#endif // JOYSTICK_H
