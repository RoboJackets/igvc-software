#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "GraphSearch.hpp"
#include "igvcsearchproblem.h"

ros::Publisher disp_path_pub;

IGVCSearchProblem search_problem;

void map_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pathplanner");

    ros::NodeHandle nh;

    nh.subscribe("/map", 1, map_callback);

    disp_path_pub = nh.advertise<nav_msgs::Path>("/path_display", 1);

    ros::spin();

    return 0;
}
