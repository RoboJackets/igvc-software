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
float Resolution; // Occupancy Grid Resolution
int x_initial, y_initial; // Index for initial x and y location in search space

double maximum_distance; // maximum distance to goal node before warning messages spit out
double CSpace; // configuration space
double goal_range; // distance from goal at which a node is considered the goal
double rateTime; // path planning/replanning rate

bool initialize_search = true; // set to true if the search problem must be initialized
bool initialize_graph = true; // set to true if the graph must be initialized

bool initial_goal_set = false; // true if the first goal has been set
bool goal_changed = false; // the goal node changed and the graph must be re-initialized

/**
    Set the current map to be used by the D* Lite search problem. The initial
    map is used to perform the first search through the occupancy grid (equivalent
    to A*). All maps thereafter are used to update edge costs for the search problem.
*/
void map_callback(const igvc_msgs::mapConstPtr& msg)
{
      std::lock_guard<std::mutex> planning_lock(planning_mutex);
      map = msg; // update current map

      if (initialize_graph)
      {
          x_initial = static_cast<int>(msg->x_initial); // initial x coord of graph search problem
          y_initial = static_cast<int>(msg->y_initial); // initial y coord of graph search problem
          dlite.graph.initializeGraph(msg);
          initialize_graph = false;
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

    std::tuple<int,int> newGoal = std::make_tuple(goal_x, goal_y);

    if (dlite.graph.Goal.getIndex() != newGoal)
        goal_changed = true; // re-initialize graph search problem

    dlite.graph.setGoal(newGoal);

    float distance_to_goal = dlite.graph.euclidian_heuristic(newGoal) * dlite.graph.Resolution;

    ROS_INFO_STREAM((goal_changed ? "New" : "Same") << " waypoint received. Search Problem Goal = " << goal_x
                    << ", " << goal_y << ". Distance: "
                    << distance_to_goal << "m.");

    if(distance_to_goal > maximum_distance)
      ROS_WARN_STREAM("Planning to waypoint more than " << maximum_distance
                      << "m. away - distance = " << distance_to_goal);

    initial_goal_set = true;
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
  path_pub = nh.advertise<nav_msgs::Path>("/path", 1);

  igvc::getParam(pNh, "c_space", CSpace);
  igvc::getParam(pNh, "maximum_distance", maximum_distance);
  igvc::getParam(pNh, "rate", rateTime);
  igvc::getParam(pNh, "goal_range", goal_range);

  dlite.graph.setCSpace(static_cast<float>(CSpace));
  dlite.GOAL_DIST = static_cast<float>(goal_range);

  // expanded_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/expanded", 1);
  // expanded_size_pub = nh.advertise<std_msgs::Int32>("/expanded_size", 1);
  // expanded_cloud.header.frame_id = "/odom";


  int numNodesUpdated;
  int numNodesExpanded;

  ros::Rate rate(rateTime);

  while (ros::ok())
  {
      ros::spinOnce(); // handle subscriber callbacks

      // don't plan unless the map has been initialized
      if (initialize_graph)
          continue;
      else
          dlite.graph.updateGraph(map);

      // don't plan unless a goal node has been set
      if (!initial_goal_set)
        continue;

      if (initialize_search)
          dlite.initialize();

      if (goal_changed)
      {
          ROS_INFO_STREAM("New Goal Received. Initializing Search...");
          dlite.reinitialize();
          initialize_search = true;
          goal_changed = false;
      }

      numNodesUpdated = dlite.updateNodesAroundUpdatedCells();
      ROS_INFO_STREAM(numNodesUpdated << " nodes updated");

      if ((numNodesUpdated > 0) || initialize_search)
      {
          numNodesExpanded = dlite.computeShortestPath();
          ROS_INFO_STREAM(numNodesExpanded << " nodes expanded");
          if (initialize_search) initialize_search = false;
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
