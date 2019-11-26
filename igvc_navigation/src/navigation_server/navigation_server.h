#ifndef SRC_NAVIGATION_SERVER_H
#define SRC_NAVIGATION_SERVER_H

#include <ros/ros.h>

#include <actionlib/server/action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <mbf_msgs/MoveBaseAction.h>
#include <mbf_msgs/GetPathAction.h>
#include <mbf_msgs/ExePathAction.h>
#include <mbf_msgs/RecoveryAction.h>

class NavigationServer {
public:
  using ActionClientGetPath = actionlib::SimpleActionClient<mbf_msgs::GetPathAction>;
  using ActionClientExePath = actionlib::SimpleActionClient<mbf_msgs::ExePathAction>;
  using ActionClientRecovery = actionlib::SimpleActionClient<mbf_msgs::RecoveryAction>;

  NavigationServer();

private:
  enum MoveBaseActionState
  {
      NONE,
      GET_PATH,
      EXE_PATH,
      RECOVERY,
      OSCILLATING,
      SUCCEEDED,
      CANCELED,
      FAILED
  };

  ros::NodeHandle private_nh_ = ros::NodeHandle("~");
  ActionClientGetPath action_client_get_path_ = ActionClientGetPath(private_nh_, "get_path");
  ActionClientExePath action_client_exe_path_ = ActionClientExePath(private_nh_, "exe_path");
  ActionClientRecovery action_client_recovery_ = ActionClientRecovery(private_nh_, "recovery");
};


#endif //SRC_NAVIGATION_SERVER_H
