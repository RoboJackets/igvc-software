#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

imu_pub = None
odometer_pub = None

def imu_callback(data):
    # Linear acceleration covariance
    old_lac = data.linear_acceleration_covariance
    covar_lac = (.1, 0,   0,
                 0,   .1, 0,
                 0,   0,   .1)
    data.linear_acceleration_covariance = covar_lac

    # Orientation covariance
    old_oc = data.orientation_covariance
    covar_oc = (.1, 0,  0,
                0,  .1, 0,
                0,  0,  .1)
    data.orientation_covariance = covar_oc

    # Angular velocity covariance
    old_avc = data.angular_velocity_covariance
    covar_avc = (.1, 0,  0,
                 0,  .1, 0,
                 0,  0,  .07)
    data.angular_velocity_covariance = covar_avc

    imu_pub.publish(data)

def odometer_callback(data):
    # Twist covariance
    old_tc = data.twist.covariance
    covar_tc = (.1,  0,   0,   0,   0,   0,
                0,   .1,  0,   0,   0,   0,
                0,   0,   .1,  0,   0,   0,
                0,   0,   0,   .1,  0,   0,
                0,   0,   0,   0,   .1,  0,
                0,   0,   0,   0,   0,   1.4)
    data.twist.covariance = covar_tc
    odometer_pub.publish(data)

def repub():
    rospy.init_node('repub')
    global imu_pub
    global odometer_pub

    # republish imu covariances
    imu_pub = rospy.Publisher('imu', Imu, queue_size=10)
    rospy.Subscriber('imu_old', Imu, imu_callback)

    # republish odometer covariances
    odometer_pub = rospy.Publisher('wheel_odometry', Odometry, queue_size = 10)
    rospy.Subscriber('wheel_odometry_old', Odometry, odometer_callback)

    rospy.spin()

if __name__ == "__main__":
    try:
        repub()
    except rospy.ROSInterruptException:
        pass
