#include <ros/ros.h>
#include "bumblebee2.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bumblebee2");
    
    ros::NodeHandle nh;
    
    std::string path;
    ros::NodeHandle pNh("~");
    if(!pNh.hasParam("path"))
        ROS_ERROR_STREAM("no parameter path");
    pNh.getParam("path", path);
    
    Bumblebee2 camera{nh,path};

    if(!camera.isOpen())
        return 1;
    
    ROS_INFO_STREAM("Bumblebee2 started.");
    
    ros::spin();
    
    return 0;
}
