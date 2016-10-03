#ifndef ODOMETER_H
#define ODOMETER_H
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <igvc_msgs/velocity_pair.h>

class Odometer {
public:
    Odometer(ros::NodeHandle&);
private:
    ros::Publisher pub;
    ros::Subscriber sub;
    nav_msgs::Odometry odom;
    tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_tf;
    const float WHEEL_SEPARATION = 1.0;
    float theta;

    void enc_callback(const igvc_msgs::velocity_pair&);
};
#endif
