#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <iostream>
#include <fstream>

std::string fileName;

void waypoint_callback(const geometry_msgs::PointStampedConstPtr& msg)
{
    std::ofstream datacsv;
    datacsv.open (fileName, std::ios_base::app | std::ios_base::out);
    datacsv << std::to_string(msg->point.x) + ", " + std::to_string(msg->point.y) + ",\n";
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "waypoint_data_recorder");

    ros::NodeHandle nh;

    ros::NodeHandle nhParam("~");

    nhParam.getParam("file", fileName);

    ros::Subscriber waypoint_sub = nh.subscribe("/waypoint", 1, waypoint_callback);

    ros::spin();

    return 0;
}