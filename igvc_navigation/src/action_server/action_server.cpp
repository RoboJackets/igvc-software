#include <ros/ros.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include "action_server.h"

ActionServer::ActionServer() {
  ros::NodeHandle nh;

  //wait for the action server to come up
  while(!client.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ros::Subscriber rviz_sub = nh.subscribe("/move_base_simple/goal",1,&ActionServer::actionCallbackPose,this);
  ros::Subscriber waypoint_sub = nh.subscribe("/waypoint",1,&ActionServer::actionCallbackPoint, this);

  ros::spin();
}

void ActionServer::actionCallbackPose(geometry_msgs::PoseStamped pose) {
  mbf_msgs::MoveBaseGoal goal;

  goal.target_pose = pose;

  ROS_INFO("Sending goal from rviz");
  client.sendGoal(goal);
}

void ActionServer::actionCallbackPoint(geometry_msgs::PointStamped point) {
  mbf_msgs::MoveBaseGoal goal;

  goal.target_pose.header = point.header;
  goal.target_pose.pose.position = point.point;

  tf::Quaternion quat_tf;
  geometry_msgs::Quaternion quat_msg;
  quat_tf.setRPY(0,0,1);
  tf::quaternionTFToMsg(quat_tf, quat_msg);
  goal.target_pose.pose.orientation = quat_msg;

  ROS_INFO("Sending goal from waypoint");
  client.sendGoalAndWait(goal);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "action_server");
  ActionServer action_server;
}
