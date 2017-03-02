#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <igvc_msgs/action_path.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <tf/transform_datatypes.h>
#include <algorithm>
#include <mutex>
#include "GraphSearch.hpp"
#include "igvcsearchproblem.h"

ros::Publisher disp_path_pub;

ros::Publisher act_path_pub;

ros::Publisher expanded_pub;

IGVCSearchProblem search_problem;

std::mutex planning_mutex;

bool received_waypoint = false;

void map_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(planning_mutex);
  *search_problem.Map = *msg;
}

void position_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(planning_mutex);
  search_problem.Start.x = msg->pose.position.x;
  search_problem.Start.y = msg->pose.position.y;
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.orientation, q);
  search_problem.Start.theta = -tf::getYaw(q);
}

void waypoint_callback(const geometry_msgs::PointStampedConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(planning_mutex);
  search_problem.Goal.x = msg->point.x;
  search_problem.Goal.y = msg->point.y;
  cout << "Waypoint received. " << search_problem.Goal.x << ", " << search_problem.Goal.y << endl;
  received_waypoint = true;
}

void expanded_callback(const set<SearchLocation>& expanded)
{
  if (expanded_pub.getNumSubscribers() > 0)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.header.frame_id = "/map";
    for (auto location : expanded)
    {
      cloud.points.push_back(pcl::PointXYZ(location.x, location.y, 0));
    }
    expanded_pub.publish(cloud);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pathplanner");

  ros::NodeHandle nh;

  ros::Subscriber map_sub = nh.subscribe("/map", 1, map_callback);

  ros::Subscriber pose_sub = nh.subscribe("/odom_combined", 1, position_callback);

  ros::Subscriber waypoint_sub = nh.subscribe("/waypoint", 1, waypoint_callback);

  disp_path_pub = nh.advertise<nav_msgs::Path>("/path_display", 1);

  act_path_pub = nh.advertise<igvc_msgs::action_path>("/path", 1);

  expanded_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/expanded", 1);

  double baseline = 0.93;

  search_problem.Map = pcl::PointCloud<pcl::PointXYZ>().makeShared();
  search_problem.GoalThreshold = 1.0;
  search_problem.Threshold = 0.50;
  search_problem.Speed = 1.0;
  search_problem.Baseline = baseline;
  search_problem.DeltaT = [](double distToStart) -> double { return 0.66 * (log2(distToStart + 1) + 0.1); };
  search_problem.MinimumOmega = -0.6;
  search_problem.MaximumOmega = 0.61;
  search_problem.DeltaOmega = 0.3;  // wat
  search_problem.PointTurnsEnabled = false;
  search_problem.ReverseEnabled = false;
  search_problem.maxODeltaT = 0.1;

  ros::Rate rate(3);
  while (ros::ok())
  {
    ros::spinOnce();

    /* Do not attempt to plan a path if the path length would be greater than 100ft (~30m).
     * This should only happen when we have received either a waypoint or position estimate, but not both.
     * Long paths take forever to compute, and will freeze up this node.
     */
    auto distance_to_goal = search_problem.Start.distTo(search_problem.Goal);
    if (!received_waypoint || distance_to_goal == 0 || distance_to_goal > 60)
      continue;

    planning_mutex.lock();
    // TODO only replan if needed.
    Path<SearchLocation, SearchMove> path;
    path = GraphSearch::AStar(search_problem, expanded_callback);
    if (act_path_pub.getNumSubscribers() > 0)
    {
      nav_msgs::Path disp_path_msg;
      disp_path_msg.header.stamp = ros::Time::now();
      disp_path_msg.header.frame_id = "map";
      if (path.getStates()->empty())
        path.getStates()->push_back(search_problem.Start);
      for (auto loc : *(path.getStates()))
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
      for (auto action : *(path.getActions()))
      {
        igvc_msgs::velocity_pair vels;
        vels.header.stamp = act_path_msg.header.stamp;
        vels.header.frame_id = act_path_msg.header.frame_id;
        if (action.W != 0)
        {
          double radius = action.V / action.W;
          vels.right_velocity = (radius - baseline / 2.) * action.W;
          vels.left_velocity = (radius + baseline / 2.) * action.W;
        }
        else
        {
          vels.right_velocity = 1.0;
          vels.left_velocity = 1.0;
        }
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
