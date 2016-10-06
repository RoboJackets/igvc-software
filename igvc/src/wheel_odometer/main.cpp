#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "Odometer.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "wheel_odom");
    ros::NodeHandle nh;

    Odometer odom(nh);

    ROS_INFO_STREAM("wheel odometry node has started");

    ros::spin();

    return 0;
}
