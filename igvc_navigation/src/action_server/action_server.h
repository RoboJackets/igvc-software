/**
 * Class for publishing move base actions for move_base_flex.
 * Currently, it only takes values from Rviz's /move_base_simple/goal topic.
 *
 * Author: Tan Gemicioglu <tangem1@hotmail.com>
 */

#ifndef ACTIONSERVER_H
#define ACTIONSERVER_H

#include <mbf_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction> MoveBaseClient;

class ActionServer {
public:
  ActionServer();

private:
  MoveBaseClient client = MoveBaseClient("move_base_flex/move_base", true);

  void actionCallback(geometry_msgs::PoseStamped pose);
};

#endif  // ACTIONSERVER_H
