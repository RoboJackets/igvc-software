#include <igvc_msgs/velocity_pair.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Joy.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

ros::Publisher cmd_pub;
ros::NodeHandle* nhp;
diagnostic_updater::Updater updater;

void joystick_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Joystick Online");
    double absoluteMaxVel, maxVel, maxVelIncr;
    nhp->param(std::string("absoluteMaxVel"), absoluteMaxVel, 1.0);
    nhp->param(std::string("maxVel"), maxVel, 1.6);
    nhp->param(std::string("maxVelIncr"), maxVelIncr, 0.1);
    stat.add("absolute_max_velocity", absoluteMaxVel);
    stat.add("max_velocity", maxVel);
    stat.add("max_velocity_increment", maxVelIncr);
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    double absoluteMaxVel, maxVel, maxVelIncr;
    nhp->param(std::string("absoluteMaxVel"), absoluteMaxVel, 1.0);
    nhp->param(std::string("maxVel"), maxVel, 1.6);
    nhp->param(std::string("maxVelIncr"), maxVelIncr, 0.1);

    if (msg->buttons[1])
    maxVel -= maxVelIncr;
    else if (msg->buttons[3])
    maxVel += maxVelIncr;
    maxVel = std::min(maxVel, absoluteMaxVel);
    maxVel = std::max(maxVel, 0.0);

    nhp->setParam("maxVel", maxVel);

    int leftJoyAxis, rightJoyAxis;
    bool leftInverted, rightInverted;
    nhp->param(std::string("leftAxis"), leftJoyAxis, 1);
    nhp->param(std::string("rightAxis"), rightJoyAxis, 4);
    nhp->param(std::string("leftInverted"), leftInverted, false);
    nhp->param(std::string("rightInverted"), rightInverted, false);

    updater.update();

    igvc_msgs::velocity_pair cmd;
    cmd.left_velocity = msg->axes[leftJoyAxis] * maxVel * (leftInverted ? -1.0 : 1.0);
    cmd.right_velocity = msg->axes[rightJoyAxis] * maxVel * (rightInverted ? -1.0 : 1.0);
    cmd.header.stamp = ros::Time::now();

    cmd_pub.publish(cmd);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joystick_driver");
    ros::NodeHandle nh;
    nhp = new ros::NodeHandle("~");

    cmd_pub = nh.advertise<igvc_msgs::velocity_pair>("/motors", 1);

    ros::Subscriber joy_sub = nh.subscribe("/joy", 1, joyCallback);


    updater.setHardwareID("Joystick");
    updater.add("Joystick Diagnostic", joystick_diagnostic);

    ros::spin();
    delete nhp;
    return 0;
}
