#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "GraphSearch.hpp"
#include "igvcsearchproblem.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <mutex>
#include <pcl_conversions/pcl_conversions.h>
#include <algorithm>
#include <igvc_msgs/action_path.h>

using namespace std;

ros::Publisher disp_path_pub;

ros::Publisher act_path_pub;

IGVCSearchProblem search_problem;

mutex planning_mutex;

void map_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    lock_guard<mutex> lock(planning_mutex);
    pcl::fromROSMsg(*msg, *search_problem.Map);
}

void position_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    lock_guard<mutex> lock(planning_mutex);
    search_problem.Start.x = msg->pose.position.x;
    search_problem.Start.y = msg->pose.position.y;
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.orientation, q);
    search_problem.Start.theta = tf::getYaw(q);
}

void waypoint_callback(const geometry_msgs::PointStampedConstPtr& msg)
{
    lock_guard<mutex> lock(planning_mutex);
    search_problem.Goal.x = msg->point.x;
    search_problem.Goal.y = msg->point.y;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pathplanner");

    ros::NodeHandle nh;

    ros::Subscriber map_sub = nh.subscribe("/map", 1, map_callback);

    ros::Subscriber pose_sub = nh.subscribe("/robot_pose_ekf/odom_combined", 1, position_callback);

    ros::Subscriber waypoint_sub = nh.subscribe("/waypoint", 1, waypoint_callback);

    disp_path_pub = nh.advertise<nav_msgs::Path>("/path_display", 1);

    act_path_pub = nh.advertise<igvc_msgs::action_path>("/path", 1);

    double baseline = 0.7275;

    search_problem.Map = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    search_problem.GoalThreshold = 1.0;
    search_problem.Threshold = 0.36375;
    search_problem.Speed = 0.25;
    search_problem.Baseline = baseline;
    search_problem.DeltaT = 0.25;

    ros::Rate rate(3);
    while(ros::ok())
    {
        ros::spinOnce();

        /* Do not attempt to plan a path if the path length would be greater than 100ft (~30m).
         * This should only happen when we have received either a waypoint or position estimate, but not both.
         * Long paths take forever to compute, and will freeze up this node.
         */
        if(search_problem.Start.distTo(search_problem.Goal) > 30)
            continue;

        planning_mutex.lock();
        // TODO only replan if needed.
        auto path = GraphSearch::AStar(search_problem);

        if(disp_path_pub.getNumSubscribers() > 0)
        {
            nav_msgs::Path disp_path_msg;
            disp_path_msg.header.stamp = ros::Time::now();
            disp_path_msg.header.frame_id = "map";
            for(auto loc : *(path.getStates()))
            {
                geometry_msgs::PoseStamped pose;
                pose.header.stamp = disp_path_msg.header.stamp;
                pose.header.frame_id = disp_path_msg.header.frame_id;
                pose.pose.position.x = loc.x;
                pose.pose.position.y = loc.y;
                disp_path_msg.poses.push_back(pose);
            }
            disp_path_pub.publish(disp_path_msg);

            igvc_msgs::action_path act_path_msg;
            act_path_msg.header.stamp = ros::Time::now();
            act_path_msg.header.frame_id = "map";
            for(auto action : *(path.getActions()))
            {
                igvc_msgs::velocity_pair vels;
                vels.header.stamp = act_path_msg.header.stamp;
                vels.header.frame_id = act_path_msg.header.frame_id;
                double radius = action.V / action.W;
                vels.left_velocity = (radius - baseline/2.) * action.W;
                vels.right_velocity = (radius + baseline/2.) * action.W;
                vels.duration = action.DeltaT;
                act_path_msg.actions.push_back(vels);
            }
            act_path_pub.publish(act_path_msg);
        }

        planning_mutex.unlock();

        rate.sleep();
    }

    return 0;
}
