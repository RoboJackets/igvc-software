#ifndef SRC_NAVIGATION_SERVER_H
#define SRC_NAVIGATION_SERVER_H

#include <ros/ros.h>

#include <actionlib/server/action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <mbf_msgs/GetPathAction.h>
#include <mbf_msgs/ExePathAction.h>
#include <mbf_msgs/RecoveryAction.h>

#include <igvc_msgs/NavigateWaypointAction.h>
#include <igvc_msgs/NavigateWaypointFeedback.h>

#include <mutex>

class NavigationServer
{
public:
  using ActionClientGetPath = actionlib::SimpleActionClient<mbf_msgs::GetPathAction>;
  using ActionClientExePath = actionlib::SimpleActionClient<mbf_msgs::ExePathAction>;
  using ActionClientRecovery = actionlib::SimpleActionClient<mbf_msgs::RecoveryAction>;

  using GoalHandle = actionlib::ActionServer<igvc_msgs::NavigateWaypointAction>::GoalHandle;

  NavigationServer();

private:
  enum NavigationState
  {
    NONE,
    GET_PATH,
    EXE_PATH,
    OSCILLATING,
    RECOVERY,
    SUCCEEDED,
    CANCELED,
    FAILED
  };

  // params
  bool recovery_enabled_ = true;

  NavigationState navigation_state_;
  NavigationState recovery_trigger_;
  GoalHandle goal_handle_;
  bool fix_goal_poses_ = true;

  // ros
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher current_goal_pose_publisher_;

  // action lib
  ActionClientGetPath action_client_get_path_;
  ActionClientExePath action_client_exe_path_;
  ActionClientRecovery action_client_recovery_;
  actionlib::ActionServer<igvc_msgs::NavigateWaypointAction> action_server_;

  // recovery
  std::vector<std::string> recovery_behaviors_;
  std::vector<std::string> default_recovery_behaviors_ = { "back_up_recovery" };
  std::vector<std::string>::iterator current_recovery_behavior_;

  // goals
  mbf_msgs::GetPathGoal get_path_goal_;
  std::string exe_path_controller_;

  // replanning
  ros::Rate replanning_rate_ = ros::Rate(1.0);
  std::mutex replanning_mtx_;
  int max_replanning_tries_;
  int current_replanning_tries_;

  // exe_path feedback / oscillation
  ros::Time start_time_;
  igvc_msgs::NavigateWaypointFeedback move_base_feedback_;
  geometry_msgs::PoseStamped previous_oscillation_pose_;
  ros::Time last_oscillation_reset_;
  ros::Duration oscillation_wait_time_;
  ros::Duration oscillation_timeout_;
  double oscillation_distance_ = 0.0;

  void start(GoalHandle goal_handle);

  void cancel();

  void runGetPath();

  void actionGetPathDone(const actionlib::SimpleClientGoalState &state,
                         const mbf_msgs::GetPathResultConstPtr &result_ptr);

  void actionGetPathReplanningDone(const actionlib::SimpleClientGoalState &state,
                                   const mbf_msgs::GetPathResultConstPtr &result_ptr);

  void runExePath(nav_msgs::Path path);

  void actionExePathDone(const actionlib::SimpleClientGoalState &state,
                         const mbf_msgs::ExePathResultConstPtr &result_ptr);

  static void actionExePathActive();

  void actionExePathFeedback(const mbf_msgs::ExePathFeedbackConstPtr &feedback);

  void checkForOscillation(const geometry_msgs::PoseStamped &robot_pose);

  /**
  Attempts to run recovery behavior.
  @return true if recovery behavior has started
   */
  bool attemptRecovery();

  void actionRecoveryDone(const actionlib::SimpleClientGoalState &state,
                          const mbf_msgs::RecoveryResultConstPtr &result_ptr);

  void runNextRecoveryBehavior(const igvc_msgs::NavigateWaypointResult &navigate_waypoint_result);
};

#endif  // SRC_NAVIGATION_SERVER_H
