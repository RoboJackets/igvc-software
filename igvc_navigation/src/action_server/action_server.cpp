#include <ros/ros.h>

#include "action_server.h"

ActionServer::ActionServer() {
  ros::NodeHandle nh;

  //wait for the action server to come up
  while(!client.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ros::Subscriber rviz_sub = nh.subscribe("/move_base_simple/goal",1,&ActionServer::actionCallback,this);

  ros::spin();
}

void ActionServer::actionCallback(geometry_msgs::PoseStamped pose) {
  mbf_msgs::MoveBaseGoal goal;

  goal.target_pose = pose;

  ROS_INFO("Sending goal");
  client.sendGoal(goal);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "action_server");
  ActionServer action_server;
}
