#include <ros/ros.h>
#include <tests/SlamTests.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "slam_test");
    SlamTests testing_node;
    ros::spin();
}