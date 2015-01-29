#include <ros/ros.h>
#include "bumblebee2.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bumblebee2");
    
    ros::NodeHandle nh;
    
    Bumblebee2 camera{};
    
    ROS_INFO_STREAM("Bumblebee2 started.");
    
    ros::spin();
    
    return 0;
}
