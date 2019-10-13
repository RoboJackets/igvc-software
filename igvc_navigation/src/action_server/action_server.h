/**
 * Class for publishing move base actions for move_base_flex.
 * Currently, it takes values from Rviz's /move_base_simple/goal topic and the /waypoint topic.
 *
 * Author: Tan Gemicioglu <tangem1@hotmail.com>
 */

#ifndef ACTIONSERVER_H
#define ACTIONSERVER_H

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mbf_msgs/MoveBaseAction.h>

using MoveBaseClient = actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>;

class ActionServer
{
public:
  ActionServer();
  ros::Subscriber rviz_sub;
  ros::Subscriber waypoint_sub;

private:
  MoveBaseClient client = MoveBaseClient("move_base_flex/move_base", true);

  void actionCallbackPose(geometry_msgs::PoseStamped pose);
  void actionCallbackPoint(geometry_msgs::PointStamped point);
};

#endif  // ACTIONSERVER_H
