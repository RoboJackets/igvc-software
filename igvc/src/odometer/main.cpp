#include <ros/ros.h>
#include <ros/publisher.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <igvc_msgs/velocity_pair.h>
#include <tf/tf.h>

using namespace std;

ros::Publisher pub;
nav_msgs::Odometry odom;
float theta;
float wheelSeparation;
int seq;

/**
 * Coneverts wheel velocities to odometry message using trigonometry for calculations
 * In the ros coordinate convention x is forward, y is leftward, and z is upward relative to the robot
*/
void enc_callback(const igvc_msgs::velocity_pair& msg) {
    float leftVelocity = msg.left_velocity;
    float rightVelocity = msg.right_velocity;
    float deltaT = msg.duration;

    float angularVelocity = (rightVelocity - leftVelocity) / wheelSeparation;
    float deltaTheta = angularVelocity * deltaT;

    float velocity = (rightVelocity + leftVelocity) / 2;

    float xVelocityPrime;
    float yVelocityPrime;
    if (abs(rightVelocity - leftVelocity) > .1) { // TODO figure out velocity difference
        float turnRadius = velocity * deltaT / deltaTheta;

        yVelocityPrime = turnRadius * (1 - cos(deltaTheta)); // direction tangential to turn circle
        xVelocityPrime = turnRadius * sin(deltaTheta); // direction along radius of turn circle
    } else {
        // limit where turn radius is infinite (ie. straight line)
        yVelocityPrime = 0;
        xVelocityPrime = velocity;
    }

    // calculating the x and y velocities in the global frame of robot accounting for various theta values
    geometry_msgs::Vector3 linearAccelerations;
    linearAccelerations.y = xVelocityPrime * cos(theta) + yVelocityPrime * sin(theta);
    linearAccelerations.x = xVelocityPrime * sin(theta) + yVelocityPrime * cos(theta);
    linearAccelerations.z = 0.0;

    theta += deltaTheta;

    geometry_msgs::Vector3 angularAccelerations;
    angularAccelerations.x = 0.0;
    angularAccelerations.y = 0.0;
    angularAccelerations.z = angularVelocity;

    odom.twist.twist.linear = linearAccelerations;
    odom.twist.twist.angular = angularAccelerations;

    odom.pose.pose.position.x += linearAccelerations.x * deltaT;
    odom.pose.pose.position.y += linearAccelerations.y * deltaT;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

    odom.header.seq += 1;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = 1;

    odom.child_frame_id = "base_link"; // TODO unsure about this

    pub.publish(odom);
}    

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/encoders", 10, enc_callback);
    pub = nh.advertise<nav_msgs::Odometry>("/odometry", 10);

    wheelSeparation = 1; // TODO replace with real wheel separation
    odom.header.seq = 0;

    ros::spin();
}
