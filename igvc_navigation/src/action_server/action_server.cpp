#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include "action_server.h"

ActionServer::ActionServer()
{
  ros::NodeHandle nh;

  // wait for the action server to come up
  while (!client.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  rviz_sub = nh.subscribe("/move_base_simple/goal", 1, &ActionServer::actionCallbackPose, this);
  waypoint_sub = nh.subscribe("/waypoint", 1, &ActionServer::actionCallbackPoint, this);
}

void ActionServer::actionCallbackPose(geometry_msgs::PoseStamped pose)
{
  mbf_msgs::MoveBaseGoal goal;

  goal.target_pose = pose;

  ROS_INFO("Sending goal from rviz");
  client.sendGoal(goal);
}

void ActionServer::actionCallbackPoint(geometry_msgs::PointStamped point)
{
  mbf_msgs::MoveBaseGoal goal;

  goal.target_pose.header = point.header;
  goal.target_pose.pose.position = point.point;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  ROS_INFO("Sending goal from waypoint");
  client.sendGoalAndWait(goal);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "action_server");
  ActionServer action_server;
  ros::spin();
}
