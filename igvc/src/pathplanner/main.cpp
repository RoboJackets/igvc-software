#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <igvc_msgs/action_path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/octree/octree_search.h>
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

ros::Publisher path_planner_map_pub;

IGVCSearchProblem search_problem;

std::mutex planning_mutex;

bool received_waypoint = false;

unsigned int current_index = 0;

void map_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
{
  std::lock_guard<std::mutex> planning_lock(planning_mutex);
  if (!msg->points.empty())
  {
    while (current_index < msg->size())
    {
      search_problem.Octree->addPointToCloud(
          pcl::PointXYZ(msg->points[current_index].x, msg->points[current_index].y, 0), search_problem.Map);
      current_index++;
    }
    if (path_planner_map_pub.getNumSubscribers() > 0)
    {
      path_planner_map_pub.publish(search_problem.Map);
    }
  }
}

void position_callback(const nav_msgs::OdometryConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(planning_mutex);
  search_problem.Start.x = msg->pose.pose.position.x;
  search_problem.Start.y = msg->pose.pose.position.y;
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
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
    cloud.header.frame_id = "/odom";
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

  ros::Subscriber pose_sub = nh.subscribe("/odometry/filtered", 1, position_callback);

  ros::Subscriber waypoint_sub = nh.subscribe("/waypoint", 1, waypoint_callback);

  disp_path_pub = nh.advertise<nav_msgs::Path>("/path_display", 1);

  act_path_pub = nh.advertise<igvc_msgs::action_path>("/path", 1);

  expanded_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/expanded", 1);

  path_planner_map_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/path_planner_incremental", 1);

  double baseline = 0.93;

  ros::NodeHandle pNh("~");

  search_problem.Map = pcl::PointCloud<pcl::PointXYZ>().makeShared();
  search_problem.Map->header.frame_id = "/odom";
  search_problem.Octree = boost::make_shared<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>>(0.1);
  search_problem.Octree->setInputCloud(search_problem.Map);

  if (!pNh.hasParam("goal_threshold") || !pNh.hasParam("threshold") || !pNh.hasParam("speed") ||
      !pNh.hasParam("baseline") || !pNh.hasParam("minimum_omega") || !pNh.hasParam("maximum_omega") ||
      !pNh.hasParam("delta_omega") || !pNh.hasParam("point_turns_enabled") || !pNh.hasParam("reverse_enabled") ||
      !pNh.hasParam("max_obstacle_delta_t") || !pNh.hasParam("alpha") || !pNh.hasParam("beta") ||
      !pNh.hasParam("bounding_distance"))
  {
    ROS_ERROR_STREAM("path planner does not have all required parameters");
    return 0;
  }

  pNh.getParam("goal_threshold", search_problem.GoalThreshold);
  pNh.getParam("threshold", search_problem.Threshold);
  pNh.getParam("speed", search_problem.Speed);
  pNh.getParam("baseline", search_problem.Baseline);
  search_problem.DeltaT = [](double distToStart, double distToGoal) -> double {
    return -((distToStart + distToGoal) / 7 / (pow((distToStart + distToGoal) / 2, 2)) *
             pow(distToStart - (distToStart + distToGoal) / 2, 2)) +
           (distToStart + distToGoal) / 7 + 0.3;
  };
  pNh.getParam("minimum_omega", search_problem.MinimumOmega);
  pNh.getParam("maximum_omega", search_problem.MaximumOmega);
  pNh.getParam("delta_omega", search_problem.DeltaOmega);
  pNh.getParam("point_turns_enabled", search_problem.PointTurnsEnabled);
  pNh.getParam("reverse_enabled", search_problem.ReverseEnabled);
  pNh.getParam("max_obstacle_delta_t", search_problem.MaxObstacleDeltaT);
  pNh.getParam("alpha", search_problem.Alpha);
  pNh.getParam("beta", search_problem.Beta);
  pNh.getParam("bounding_distance", search_problem.BoundingDistance);

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
    Path<SearchLocation, SearchMove> path;
    path = GraphSearch::AStar(search_problem, expanded_callback);
    if (act_path_pub.getNumSubscribers() > 0)
    {
      nav_msgs::Path disp_path_msg;
      disp_path_msg.header.stamp = ros::Time::now();
      disp_path_msg.header.frame_id = "odom";
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
      act_path_msg.header.frame_id = "odom";
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
