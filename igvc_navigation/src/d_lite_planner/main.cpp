/**
Solves for an optimal path using the D* Lite incremental search algorithm.

D* Lite implementation details can be found in DLitePlanner.h
*/

#include <geometry_msgs/PoseStamped.h>
#include <igvc_msgs/map.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>
#include <tf/transform_datatypes.h>
#include <igvc_utils/NodeUtils.hpp>
#include <mutex>

#include <tuple>
#include <vector>
#include "Graph.h"
#include "DLitePlanner.h"

ros::Publisher path_pub;

std::mutex planning_mutex;

ros::Publisher expanded_pub;
ros::Publisher expanded_size_pub;
pcl::PointCloud<pcl::PointXYZ> expanded_cloud;

igvc_msgs::mapConstPtr map; // Most up-to-date map
DLitePlanner dlite; // D* Lite path planner
std::tuple<int,int> currGoal; // Search problem goal //TODO update goal
float Resolution; // Occupancy Grid Resolution
int x_initial, y_initial; // Index for initial x and y location in search space
double maximum_distance;

bool received_waypoint = false; // true if the graph currently has a valid goal
bool initialized_map = false; // true if the grap has been initialized


/**
    Set the current map to be used by the D* Lite search problem. The initial
    map is used to perform the initial (A*) search through the occupancy grid.
    All maps thereafter are used to update edge costs for the search problem.
*/
void map_callback(const igvc_msgs::mapConstPtr& msg)
{
      std::lock_guard<std::mutex> planning_lock(planning_mutex);
      map = msg; // update current map

      if (!initialized_map)
      {
          x_initial = static_cast<int>(msg->x_initial); // initial x coord of graph search problem
          y_initial = static_cast<int>(msg->y_initial); // initial y coord of graph search problem
          dlite.graph.initializeGraph(msg);
          initialized_map = true;
      }
}

/**
    Assigns a valid goal to the graph search problem. Goal index obtained by
    converting from the /map frame goal coordinate to the graph index.
*/
void waypoint_callback(const geometry_msgs::PointStampedConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(planning_mutex);

    int goal_x, goal_y;
    goal_x = static_cast<int>(std::round(msg->point.x / dlite.graph.Resolution)) + x_initial;
    goal_y = static_cast<int>(std::round(msg->point.y / dlite.graph.Resolution)) + y_initial;
    currGoal = std::make_tuple(goal_x, goal_y);
    dlite.graph.setGoal(currGoal);

    float distance_to_goal = dlite.graph.euclidian_heuristic(currGoal);

    ROS_INFO_STREAM("Waypoint received. Search Problem Goal = " << goal_x
                    << ", " << goal_y << ". Distance: "
                    << distance_to_goal << "m.");

    if(distance_to_goal > maximum_distance)
      ROS_WARN_STREAM("Planning to waypoint more than " << maximum_distance
                      << "m. away - distance = " << distance_to_goal);

    received_waypoint = true;
}


/*
 * NOT A ROS CALLBACK
 * publishes the points that have been expanded for visualization and the size of the expanded points
 */
// void expanded_callback(const SearchLocation& location)
// {
//   expanded_cloud.points.push_back(pcl::PointXYZ((location.X - initial_x) * search_problem.Resolution,
//                                                 (location.Y - initial_y) * search_problem.Resolution, location.Theta));
//   expanded_pub.publish(expanded_cloud);
//   std_msgs::Int32 size_msg;
//   size_msg.data = expanded_cloud.size();
//   expanded_size_pub.publish(size_msg);
// }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "d_lite_planner");

  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  // subscribe to map for occupancy grid and waypoint for goal node
  ros::Subscriber map_sub = nh.subscribe("/map", 1, map_callback);
  ros::Subscriber waypoint_sub = nh.subscribe("/waypoint", 1, waypoint_callback);

  // publish path for path_follower
  path_pub = nh.advertise<nav_msgs::Path>("/path_sim", 1);

  double rateTime;
  double CSpace;
  double goal_range;

  igvc::getParam(pNh, "c_space", CSpace);
  igvc::getParam(pNh, "maximum_distance", maximum_distance);
  igvc::getParam(pNh, "rate", rateTime);
  igvc::getParam(pNh, "goal_range", goal_range);

  dlite.graph.setCSpace(static_cast<float>(CSpace));
  dlite.GOAL_DIST = static_cast<float>(goal_range);

  // expanded_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/expanded", 1);
  // expanded_size_pub = nh.advertise<std_msgs::Int32>("/expanded_size", 1);
  // expanded_cloud.header.frame_id = "/odom";

  bool initialize_search = true;
  int numNodesUpdated;
  int numNodesExpanded;

  ros::Rate rate(rateTime);

  while (ros::ok())
  {
      ros::spinOnce(); // handle subscriber callbacks

      // don't plan unless the map has been initialized
      if (!initialized_map)
          continue;
      else
          dlite.graph.updateGraph(map);

      // don't plan unless a goal node has been set
      if (!received_waypoint)
        continue;

      if (initialize_search)
          dlite.initialize();

      numNodesUpdated = dlite.updateNodesAroundUpdatedCells();
      ROS_INFO_STREAM(numNodesUpdated << " nodes updated");

      if ((numNodesUpdated > 0) || initialize_search)
      {
          numNodesExpanded = dlite.computeShortestPath();
          ROS_INFO_STREAM(numNodesExpanded << " nodes expanded");
          initialize_search = false;
      }

      dlite.constructOptimalPath();
      ROS_INFO_STREAM("Optimal Path Found");


      nav_msgs::Path path_msg;
      path_msg.header.stamp = ros::Time::now();
      path_msg.header.frame_id = "odom";

      for (std::tuple<int,int> point : dlite.path)
      {

          auto it = path_msg.poses.begin();
          geometry_msgs::PoseStamped pose;
          pose.header.stamp = path_msg.header.stamp;
          pose.header.frame_id = path_msg.header.frame_id;
          pose.pose.position.x = (std::get<0>(point) - x_initial) * dlite.graph.Resolution;
          pose.pose.position.y = (std::get<1>(point) - y_initial) * dlite.graph.Resolution;
          path_msg.poses.insert(it, pose);
      }

      path_pub.publish(path_msg);

      rate.sleep();
      // ROS_INFO_STREAM("Length: " << dlite.graph.length << " Width: " << dlite.graph.width
      //                 << " Resolution: " << dlite.graph.Resolution << " Start: "
      //                 << dlite.graph.Start.getIndex() << " Goal: " << dlite.graph.Goal.getIndex());

  }
  // while (ros::ok())
  // {
  //   ros::spinOnce();
  //
  //   // do not plan if you do not have a waypoint
  //   if (!received_waypoint) {
  //     continue;
  //   }
  //
  //   planning_mutex.lock();
  //   Path<SearchLocation, SearchMove> path;
  //   search_problem.DistanceToGoal = search_problem.Start.distTo(search_problem.Goal, search_problem.Resolution);
  //   path = GraphSearch::AStar(search_problem, expanded_callback, maxIter);
  //   nav_msgs::Path path_msg;
  //   // TODO timestamp why
  //   path_msg.header.stamp = ros::Time::now();
  //   path_msg.header.frame_id = "odom";
  //   for (auto loc : *(path.getStates()))
  //     {// publish path with theta for cool viz
  //     geometry_msgs::PoseStamped pose;
  //     pose.header.stamp = path_msg.header.stamp;
  //     pose.header.frame_id = path_msg.header.frame_id;
  //     pose.pose.position.x = (loc.X - initial_x) * search_problem.Resolution;
  //     pose.pose.position.y = (loc.Y - initial_y) * search_problem.Resolution;
  //     path_msg.poses.push_back(pose);
  //   }
  //   path_pub.publish(path_msg);
  //   expanded_cloud.clear();
  //   planning_mutex.unlock();
  //   rate.sleep();
  // }

  return 0;
}
