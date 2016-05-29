#include <ros/ros.h>
#include "linedetector.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "linedetector");
    
    ros::NodeHandle nh;

    LineDetector det{nh};
    
    ROS_INFO_STREAM("Line Detector started.");
    
    ros::spin();
    
    return 0;
}
