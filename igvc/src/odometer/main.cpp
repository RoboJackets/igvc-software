#include <ros/ros.h>
#include <ros/publisher.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <igvc_msgs/velocity_pair.h>
#include <tf/tf.h>

using namespace std;

#define WHEEL_SEPARATION 1.0

ros::Publisher pub;
nav_msgs::Odometry odom;
int seq;

/**
 * Coneverts wheel velocities to odometry message using trigonometry for calculations
 * In the ros coordinate convention x is forward, y is leftward, and z is upward relative to the robot
 *
 * This makes the assumption the robot is operating in a 2D world b/c it cannot measure
 * other DOF from the wheel odometry
*/
void enc_callback(const igvc_msgs::velocity_pair& msg) {
    float leftVelocity = msg.left_velocity;
    float rightVelocity = msg.right_velocity;
    float deltaT = msg.duration;

    float angularVelocity = (rightVelocity - leftVelocity) / WHEEL_SEPARATION;
    float deltaTheta = angularVelocity * deltaT;

    float velocity = (rightVelocity + leftVelocity) / 2;

    geometry_msgs::Vector3 linearAccelerations;
    linearAccelerations.z = 0.0;

    if (abs(rightVelocity - leftVelocity) > .01) {
        float turnRadius = velocity * deltaT / deltaTheta;

        linearAccelerations.y = turnRadius * (1 - cos(deltaTheta));
        linearAccelerations.x = turnRadius * sin(deltaTheta);
    } else {
        // limit where turn radius is infinite (ie. straight line)
        linearAccelerations.y = 0;
        linearAccelerations.x = velocity;
    }

    geometry_msgs::Vector3 angularAccelerations;
    angularAccelerations.x = 0.0;
    angularAccelerations.y = 0.0;
    angularAccelerations.z = angularVelocity;

    odom.twist.twist.linear = linearAccelerations;
    odom.twist.twist.angular = angularAccelerations;

    // Row-major representation of the 6x6 covariance matrix
    // The orientation parameters use a fixed-axis representation.
    // In order, the parameters are:
    // (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    // TODO replace with calculated values
    odom.twist.covariance = {
            .01, .01, .01, .01, .01, .01,
            .01, .01, .01, .01, .01, .01,
            .01, .01, .01, .01, .01, .01,
            .01, .01, .01, .01, .01, .01,
            .01, .01, .01, .01, .01, .01,
            .01, .01, .01, .01, .01, .01
    };

    odom.pose.pose.position.x = 0;
    odom.pose.pose.position.y = 0;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

    // this position is known very certainly b/c the reference frame is the robot base
    odom.pose.covariance = {
            1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6,
            1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6,
            1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6,
            1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6,
            1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6,
            1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6
    };

    odom.header.seq += 1;
    odom.header.stamp = ros::Time::now();

    pub.publish(odom);
}    

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/encoders", 10, enc_callback);
    pub = nh.advertise<nav_msgs::Odometry>("/odometry", 10);

    odom.header.seq = 0;
    odom.child_frame_id = "base_link";
    odom.header.frame_id = 1;

    ros::spin();
}
