#include <igvc_msgs/msg/velocity_pair.hpp>
//#include <ros/publisher.h>
//#include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"
//#include <ros/subscriber.h>
#include <sensor_msgs/msg/Joy.hpp>
//#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/srv/diagnostic_updater.hpp>


#include <memory>
//#include <diagnostic_updater/publisher.h>
#include <diagnostic_updater/srv/publisher.hpp>

class JoystickClass: public rclcpp::Node
{
    //ros::Publisher cmd_pub;
    auto cmd_pub = rclcpp::Node::create_publisher();
    //ros::NodeHandle* nhp;
    auto nhp = rclcpp::Node;
//    std::unique_ptr<diagnostic_updater::Updater> updater_ptr;
//
//    void joystick_diagnostic(diagnostic_updater::srv::DiagnosticStatusWrapper& stat)
//    {
//
//        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Joystick Online");
//        double absoluteMaxVel, maxVel, maxVelIncr;
//        nhp->get_parameter_or(std::string("absoluteMaxVel"), absoluteMaxVel, 1.0);
//        nhp->get_parameter_or(std::string("maxVel"), maxVel, 1.6);
//        nhp->get_parameter_or(std::string("maxVelIncr"), maxVelIncr, 0.1);
//        stat.add("absolute_max_velocity", absoluteMaxVel);
//        stat.add("max_velocity", maxVel);
//        stat.add("max_velocity_increment", maxVelIncr);
//
//    }

    void joyCallback(const sensor_msgs::msg::Joy::ConstPtr& msg)
    {

        double absoluteMaxVel, maxVel, maxVelIncr;
        nhp->get_parameter_or(std::string("absoluteMaxVel"), absoluteMaxVel, 1.0);
        nhp->get_parameter_or(std::string("maxVel"), maxVel, 1.6);
        nhp->get_parameter_or(std::string("maxVelIncr"), maxVelIncr, 0.1);

        if (msg->buttons[1])
            maxVel -= maxVelIncr;
        else if (msg->buttons[3])
            maxVel += maxVelIncr;
        maxVel = std::min(maxVel, absoluteMaxVel);
        maxVel = std::max(maxVel, 0.0);

        nhp->declare_parameter("maxVel", maxVel);

        int leftJoyAxis, rightJoyAxis;
        bool leftInverted, rightInverted;
        nhp->get_parameter_or(std::string("leftAxis"), leftJoyAxis, 1);
        nhp->get_parameter_or(std::string("rightAxis"), rightJoyAxis, 4);
        nhp->get_parameter_or(std::string("leftInverted"), leftInverted, false);
        nhp->get_parameter_or(std::string("rightInverted"), rightInverted, false);

        updater_ptr->update();

        igvc_msgs::msg::velocity_pair cmd;
        cmd.left_velocity = msg->axes[leftJoyAxis] * maxVel * (leftInverted ? -1.0 : 1.0);
        cmd.right_velocity = msg->axes[rightJoyAxis] * maxVel * (rightInverted ? -1.0 : 1.0);
        //  cmd.header.stamp = ros::Time::now();
        cmd.header.stamp = rclcpp::Node::now();

        //  cmd_pub.publish(cmd);
        cmd_pub->publish(cmd);


    }

    int main(int argc, char** argv)
    {
        //  ros::init(argc, argv, "joystick_driver");
        //  ros::NodeHandle nh;
        rclcpp::init(argc, argv);
        auto nh = rclcpp::Node::make_shared("joystick_driver");

        //  nhp = new ros::NodeHandle("~");
        nhp = rclcpp::Node::make_shared("~/joystick_driver");

        //  cmd_pub = nh.advertise<igvc_msgs::velocity_pair>("/motors", 1);
        cmd_pub = nh->create_publisher<igvc_msgs::msg::velocity_pair>("/motors", 1);

        //  ros::Subscriber joy_sub = nh.subscribe("/joy", 1, joyCallback);
        auto joy_sub = nh->create_subscription<sensor_msgs::msg::Joy>("/joy", joyCallback);

        updater_ptr = std::make_unique<diagnostic_updater::Updater>();
        updater_ptr->setHardwareID("Joystick");
        updater_ptr->add("Joystick Diagnostic", joystick_diagnostic);

        //  ros::spin();
        rclcpp::spin(nhp);
        rclcpp::spin(nh);
        delete nhp;
        return 0;
    }

};