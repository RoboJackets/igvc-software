#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "GraphSearch.hpp"
#include "igvcsearchproblem.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <mutex>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

ros::Publisher disp_path_pub;

IGVCSearchProblem search_problem;

mutex planning_mutex;

void map_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    lock_guard<mutex> lock(planning_mutex);
    pcl::fromROSMsg(*msg, *search_problem.Map);
}

void position_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    lock_guard<mutex> lock(planning_mutex);
    search_problem.Start.x = msg->pose.pose.position.x;
    search_problem.Start.y = msg->pose.pose.position.y;
    tf::Quaternion q{msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w};
    tf::Matrix3x3 m{q};
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    search_problem.Start.theta = yaw;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pathplanner");

    ros::NodeHandle nh;

    nh.subscribe("/map", 1, map_callback);

    nh.subscribe("robot_pose_ekf/odom_combined", 1, position_callback);

    disp_path_pub = nh.advertise<nav_msgs::Path>("/path_display", 1);

    ros::Rate rate(3);
    while(ros::ok())
    {
        ros::spinOnce();

        planning_mutex.lock();
        // TODO replan if needed.
        planning_mutex.unlock();

        rate.sleep();
    }

    return 0;
}
