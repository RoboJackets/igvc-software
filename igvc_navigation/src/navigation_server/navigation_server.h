#ifndef SRC_NAVIGATION_SERVER_H
#define SRC_NAVIGATION_SERVER_H

#include <ros/ros.h>

#include <actionlib/server/action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <mbf_msgs/MoveBaseAction.h>


#include <mbf_msgs/MoveBaseAction.h>
#include <mbf_msgs/GetPathAction.h>
#include <mbf_msgs/ExePathAction.h>
#include <mbf_msgs/RecoveryAction.h>

class NavigationServer {
public:
  using ActionClientGetPath = actionlib::SimpleActionClient<mbf_msgs::GetPathAction>;
  using ActionClientExePath = actionlib::SimpleActionClient<mbf_msgs::ExePathAction>;
  using ActionClientRecovery = actionlib::SimpleActionClient<mbf_msgs::RecoveryAction>;

  using GoalHandle = actionlib::ActionServer<mbf_msgs::MoveBaseAction>::GoalHandle;

  NavigationServer();

private:
  enum NavigationState
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

  // params
  bool recovery_enabled_;
  bool fix_goal_poses_;

  NavigationState current_state_;
  NavigationState recovery_trigger_;
  GoalHandle current_goal_handle_;

  ros::NodeHandle private_nh_;
  ActionClientGetPath action_client_get_path_;
  ActionClientExePath action_client_exe_path_;
  ActionClientRecovery action_client_recovery_;

  std::vector<std::string> recovery_behaviors_;
  std::vector<std::string>::iterator current_recovery_behavior_;

  mbf_msgs::GetPathGoal get_path_goal_;
  std::string exe_path_controller_;

  actionlib::ActionServer<mbf_msgs::MoveBaseAction> action_server_;

  ros::Time time_of_last_get_path;
  ros::Duration time_between_get_path = ros::Duration(0.5);

  void start(GoalHandle& goal_handle);

  void cancel();

  void processLoop();

  void runGetPath();

  void actionGetPathDone(const actionlib::SimpleClientGoalState &state, const mbf_msgs::GetPathResultConstPtr &result_ptr);

  void runExePath(nav_msgs::Path path);

  void actionExePathActive();

  void actionExePathFeedback(const mbf_msgs::ExePathFeedbackConstPtr &feedback);

  void actionExePathDone(const actionlib::SimpleClientGoalState &state, const mbf_msgs::ExePathResultConstPtr &result_ptr);

  /**
  Attempts to run recovery behavior.
  @return true if recovery behavior has started
   */
  bool attemptRecovery();

  void actionRecoveryDone(const actionlib::SimpleClientGoalState &state, const mbf_msgs::RecoveryResultConstPtr &result_ptr);
};


#endif //SRC_NAVIGATION_SERVER_H
