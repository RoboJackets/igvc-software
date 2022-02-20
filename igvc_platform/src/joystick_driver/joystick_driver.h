#ifndef JOYSTICKDRIVER_H
#define JOYSTICKDRIVER_H

#include <igvc_msgs/velocity_pair.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Joy.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <parameter_assertions/assertions.h>
#include <memory>
#include <diagnostic_updater/publisher.h>

class JoystickDriver
{
public:
    JoystickDriver();

private:

    //node handles
    ros::NodeHandle nhp;
    ros::NodeHandle nh;

    //publisher
    ros::Publisher cmd_pub;

    //subscriber
    ros::Subscriber joy_sub;

    std::unique_ptr<diagnostic_updater::Updater> updater_ptr;

    double absoluteMaxVel, maxVel, maxVelIncr;
    int leftJoyAxis, rightJoyAxis;
    bool leftInverted, rightInverted;

    void joystick_diagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat);
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);


};

#endif //JOYSTICKDRIVER_H