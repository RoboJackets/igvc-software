#ifndef SRC_SLAMTESTS_H
#define SRC_SLAMTESTS_H

#include <sensor_msgs/Imu.h>
#include <tests/SlamTests.h>
#include <ros/ros.h>

class SlamTests {
public:
    SlamTests();

private:
    ros::NodeHandle pnh_;
    ros::Subscriber slam_sub_;
    ros::Subscriber ground_truth_sub_;
};

#endif //SRC_SLAMTESTS_H
