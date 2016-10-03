#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <igvc_msgs/velocity_pair.h>
#include "Odometer.h"

using namespace std;

/**
 * Coneverts wheel velocities to odometry message using trigonometry for calculations
 * In the ros coordinate convention x is forward, y is leftward, and z is upward relative to the robot
 * The position is published in an absolute reference frame relative to the initial position
 * The velocities (twist) is in a reference frame relative to the robot
*/
void Odometer::enc_callback(const igvc_msgs::velocity_pair& msg) {
    float leftVelocity = msg.left_velocity;
    float rightVelocity = msg.right_velocity;
    float deltaT = msg.duration;

    float angularVelocity = (rightVelocity - leftVelocity) / WHEEL_SEPARATION;
    float deltaTheta = angularVelocity * deltaT;
    float velocity = (rightVelocity + leftVelocity) / 2;

    geometry_msgs::Vector3 linearVelocities;
    linearVelocities.z = 0;

    if (abs(rightVelocity - leftVelocity) > 1e-6) {
        float turnRadius = velocity / angularVelocity;
        linearVelocities.y = turnRadius * (1 - cos(deltaTheta));
        linearVelocities.x = turnRadius * sin(deltaTheta);
    } else {
        // limit where turn radius is infinite (ie. straight line)
        linearVelocities.y = 0;
        linearVelocities.x = velocity;
    }

    theta += deltaTheta;

    geometry_msgs::Vector3 angularVelocities;
    angularVelocities.x = 0.0;
    angularVelocities.y = 0.0;
    angularVelocities.z = angularVelocity;

    nav_msgs::Odometry odom;
    odom.twist.twist.linear = linearVelocities;
    odom.twist.twist.angular = angularVelocities;

    odom.pose.pose.position.x += linearVelocities.x * deltaT;
    odom.pose.pose.position.y += linearVelocities.y * deltaT;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

    odom.header.seq += 1;
    ros::Time currentTime = ros::Time::now(); // published message time should match tf publisher
    odom.header.stamp = currentTime;

    pub.publish(odom);

    // publish the tf using the published odom message
    odom_tf.header.stamp = ros::Time::now();
    odom_tf.transform.translation.x = odom.pose.pose.position.x;
    odom_tf.transform.translation.y = odom.pose.pose.position.y;
    odom_tf.transform.translation.z = 0;
    odom_tf.transform.rotation = odom.pose.pose.orientation;
    odom_broadcaster.sendTransform(odom_tf);
}

Odometer::Odometer(ros::NodeHandle& nh) {
    sub = nh.subscribe("/encoders", 10, &Odometer::enc_callback, this);
    pub = nh.advertise<nav_msgs::Odometry>("/wheel_odometry", 10);

    // initializing default odom message
    odom.header.seq = 0;
    odom.child_frame_id = "base_link";
    odom.header.frame_id = "wheel_odom";

    // initializing default odom tf
    odom_tf.header.frame_id = "wheel_odom";
    odom_tf.child_frame_id = "base_link";

    // initialize position - map published is relative to position at time t=0
    theta = 0;
    odom.pose.pose.position.x = 0;
    odom.pose.pose.position.y = 0;
    odom.pose.pose.position.z = 0;

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
    // the position covariance takes same form as twist covariance above
    // TODO replace with calculated values
    odom.pose.covariance = {
            1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6,
            1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6,
            1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6,
            1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6,
            1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6,
            1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6
    };
}
