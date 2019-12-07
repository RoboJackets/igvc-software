#include "navigation_server.h"
#include <parameter_assertions/assertions.h>

NavigationServer::NavigationServer()
  : current_state_(NONE)
  , recovery_trigger_(NONE)
  , private_nh_("~")
  , action_client_get_path_(private_nh_, "get_path")
  , action_client_exe_path_(private_nh_, "exe_path")
  , action_client_recovery_(private_nh_, "recovery")
  , action_server_(private_nh_, "move_to_waypoint", boost::bind(&NavigationServer::start, this, _1),
                   boost::bind(&NavigationServer::cancel, this, _1), false)
{
  assertions::getParam(private_nh_, "recovery_enabled", recovery_enabled_);

  action_server_.start();
}

void NavigationServer::cancel(GoalHandle goal_handle)
{
  current_state_ = CANCELED;

  if (!action_client_get_path_.getState().isDone())
  {
    action_client_get_path_.cancelGoal();
  }

  if (!action_client_exe_path_.getState().isDone())
  {
    action_client_exe_path_.cancelGoal();
  }

  if (!action_client_recovery_.getState().isDone())
  {
    action_client_recovery_.cancelGoal();
  }

  goal_handle.setCanceled();
}

void NavigationServer::start(GoalHandle goal_handle)
{
  current_goal_handle_ = goal_handle;
  current_state_ = GET_PATH;
  goal_handle.setAccepted();

  ROS_DEBUG_STREAM_NAMED("nav_server", "Started action: move_to_waypoint");

  const igvc_msgs::NavigateWaypointGoal &goal = *(goal_handle.getGoal().get());

  recovery_behaviors_ = goal.recovery_behaviors;
  current_recovery_behavior_ = recovery_behaviors_.begin();

  get_path_goal_.target_pose = goal.target_pose;
  get_path_goal_.use_start_pose = false;
  get_path_goal_.planner = goal.planner;
  exe_path_controller_ = goal.controller;
  fix_goal_poses_ = goal.fix_goal_orientation;

  igvc_msgs::NavigateWaypointResult navigate_waypoint_result;

  // wait for server connections
  ros::Duration connection_timeout(1.0);
  if (!action_client_get_path_.waitForServer(connection_timeout) ||
      !action_client_exe_path_.waitForServer(connection_timeout) ||
      !action_client_recovery_.waitForServer(connection_timeout))
  {
    ROS_ERROR_STREAM_NAMED("nav_server", "Could not connect to one or more of move_base_flex actions: 'get_path', "
                                         "'exe_path', 'recovery'!");
    navigate_waypoint_result.outcome = igvc_msgs::NavigateWaypointResult ::INTERNAL_ERROR;
    navigate_waypoint_result.message = "Could not connect to the move_base_flex actions!";
    goal_handle.setAborted(navigate_waypoint_result, navigate_waypoint_result.message);
    return;
  }

  runGetPath();
  processLoop();
}

void NavigationServer::processLoop()
{
  while (ros::ok() && current_state_ != CANCELED && current_state_ != FAILED && current_state_ != SUCCEEDED)
  {
    if (current_state_ == EXE_PATH && ros::Time::now() - time_of_last_get_path > time_between_get_path)
    {
      runGetPath();
      time_of_last_get_path = ros::Time::now();
    }
  }
}

void NavigationServer::runGetPath()
{
  ROS_DEBUG_STREAM_NAMED("nav_server", "Sent goal to get_path client.");
  current_state_ = GET_PATH;
  action_client_get_path_.sendGoal(get_path_goal_, boost::bind(&NavigationServer::actionGetPathDone, this, _1, _2));
}

void NavigationServer::actionGetPathDone(const actionlib::SimpleClientGoalState &state,
                                         const mbf_msgs::GetPathResultConstPtr &result_ptr)
{
  const mbf_msgs::GetPathResult &result = *(result_ptr.get());
  igvc_msgs::NavigateWaypointResult navigate_waypoint_result;

  switch (state.state_)
  {
    case actionlib::SimpleClientGoalState::SUCCEEDED:
    {
      ROS_DEBUG_STREAM_NAMED("nav_server", "Action 'move_to_waypoint' received path from 'get_path'");
      nav_msgs::Path path = result.path;
      if (fix_goal_poses_)
      {
        size_t size = path.poses.size();
        // change the orientation of the last pose to that of the pose before it
        path.poses[size - 1].pose.orientation = path.poses[size - 2].pose.orientation;
      }
      time_of_last_get_path = ros::Time::now();

      if (recovery_trigger_ == GET_PATH)
      {
        ROS_WARN_NAMED("nav_server", "Recovered from planner failure: restart recovery behaviors");
        current_recovery_behavior_ = recovery_behaviors_.begin();
        recovery_trigger_ = NONE;
      }

      ROS_DEBUG_STREAM_NAMED("nav_server", "Action 'move_to_waypoint' sending path to 'exe_path'");
      runExePath(path);
      break;
    }
    case actionlib::SimpleClientGoalState::ABORTED:
      ROS_DEBUG_STREAM_NAMED("nav_server", "Action 'get_path' aborted.");
      if (attemptRecovery())
      {
        recovery_trigger_ = GET_PATH;
      }
      else
      {
        // copy result from get_path action
        navigate_waypoint_result.outcome = result.outcome;
        navigate_waypoint_result.message = result.message;
        ROS_WARN_STREAM_NAMED("nav_server", "Abort the execution of the planner: " << result.message);
        current_goal_handle_.setAborted(navigate_waypoint_result, state.getText());
      }
      break;
    case actionlib::SimpleClientGoalState::PREEMPTED:
      // the get_path action has been preempted.
      // copy result from get_path action
      navigate_waypoint_result.outcome = result.outcome;
      navigate_waypoint_result.message = result.message;
      current_goal_handle_.setCanceled(navigate_waypoint_result, state.getText());
      break;
    case actionlib::SimpleClientGoalState::LOST:
      ROS_FATAL_STREAM_NAMED("nav_server", "Connection lost to the action \"get_path\"!");
      current_goal_handle_.setAborted();
      break;
    default:
      ROS_FATAL_STREAM_NAMED("nav_server", "Unreachable state reached for get_path: " << state.toString());
      break;
  }
}

void NavigationServer::runExePath(nav_msgs::Path path)
{
  current_state_ = EXE_PATH;
  mbf_msgs::ExePathGoal exe_path_goal;
  exe_path_goal.controller = exe_path_controller_;
  exe_path_goal.path = std::move(path);
  action_client_exe_path_.sendGoal(exe_path_goal, boost::bind(&NavigationServer::actionExePathDone, this, _1, _2));
}

void NavigationServer::actionExePathDone(const actionlib::SimpleClientGoalState &state,
                                         const mbf_msgs::ExePathResultConstPtr &result_ptr)
{
  const mbf_msgs::ExePathResult &result = *(result_ptr.get());
  igvc_msgs::NavigateWaypointResult navigate_waypoint_result;

  switch (state.state_)
  {
    case actionlib::SimpleClientGoalState::SUCCEEDED:
      navigate_waypoint_result.outcome = igvc_msgs::NavigateWaypointResult::SUCCESS;
      navigate_waypoint_result.message = "Action 'move_to_waypoint' succeeded!";
      ROS_INFO_STREAM_NAMED("nav_server", navigate_waypoint_result.message);
      current_goal_handle_.setSucceeded(navigate_waypoint_result, navigate_waypoint_result.message);
      current_state_ = SUCCEEDED;
      break;
    case actionlib::SimpleClientGoalState::ABORTED:
      ROS_DEBUG_STREAM_NAMED("nav_server", "Action `exe_path` aborted.");
      switch (result.outcome)
      {
        case mbf_msgs::ExePathResult::INVALID_PATH:
        case mbf_msgs::ExePathResult::TF_ERROR:
        case mbf_msgs::ExePathResult::NOT_INITIALIZED:
        case mbf_msgs::ExePathResult::INVALID_PLUGIN:
        case mbf_msgs::ExePathResult::INTERNAL_ERROR:
          // none of these errors is recoverable
          current_goal_handle_.setAborted(navigate_waypoint_result, state.getText());
          break;
        default:
          // all the rest are, so we start calling the recovery behaviors in sequence
          if (attemptRecovery())
          {
            recovery_trigger_ = EXE_PATH;
          }
          else
          {
            ROS_WARN_STREAM_NAMED("nav_server", "Abort the execution of the controller: " << result.message);
            current_goal_handle_.setAborted(navigate_waypoint_result, state.getText());
          }
          break;
      }
      break;
    default:
      ROS_FATAL_STREAM_NAMED("nav_server", "Unreachable state reached for exe_path: " << state.toString());
      break;
  }
}

bool NavigationServer::attemptRecovery()
{
  if (!recovery_enabled_)
  {
    ROS_WARN_STREAM_NAMED("nav_server", "Recovery behaviors are disabled!");
    return false;
  }

  if (current_recovery_behavior_ == recovery_behaviors_.end())
  {
    if (recovery_behaviors_.empty())
    {
      ROS_WARN_STREAM_NAMED("nav_server", "No Recovery Behaviors loaded!");
    }
    else
    {
      ROS_WARN_STREAM_NAMED("nav_server", "Executed all available recovery behaviors!");
    }
    return false;
  }

  mbf_msgs::RecoveryGoal recovery_goal;
  recovery_goal.behavior = *current_recovery_behavior_;
  action_client_recovery_.sendGoal(recovery_goal, boost::bind(&NavigationServer::actionRecoveryDone, this, _1, _2));
  current_state_ = RECOVERY;
  return true;
}

void NavigationServer::actionRecoveryDone(const actionlib::SimpleClientGoalState &state,
                                          const mbf_msgs::RecoveryResultConstPtr &result_ptr)
{
  const mbf_msgs::RecoveryResult &result = *(result_ptr.get());

  igvc_msgs::NavigateWaypointResult navigate_waypoint_result;
  navigate_waypoint_result.outcome = result.outcome;
  navigate_waypoint_result.message = result.message;

  switch (state.state_)
  {
    case actionlib::SimpleClientGoalState::SUCCEEDED:
      ROS_DEBUG_STREAM_NAMED("nav_server",
                             "Execution of the recovery behavior '" << *current_recovery_behavior_ << "' succeeded!");
      ROS_DEBUG_STREAM_NAMED("nav_server", "Try planning again and increment the current recovery behavior in the "
                                           "list.");
      current_recovery_behavior_++;
      runGetPath();
      break;
    case actionlib::SimpleClientGoalState::ABORTED:
      ROS_DEBUG_STREAM_NAMED("nav_server", "Recovery behavior aborted!");
      ROS_DEBUG_STREAM_NAMED("nav_server", "The recovery behavior '" << *current_recovery_behavior_ << "' failed. ");
      ROS_DEBUG_STREAM("Recovery behavior message: " << result.message << ", outcome: " << result.outcome);
      current_recovery_behavior_++;
      if (current_recovery_behavior_ == recovery_behaviors_.end())
      {
        ROS_DEBUG_STREAM_NAMED("nav_server", "All recovery behaviors failed. Abort recovering and abort the move_base "
                                             "action");
        current_goal_handle_.setAborted(navigate_waypoint_result, "All recovery behaviors failed.");
      }
      else
      {
        ROS_INFO_STREAM_NAMED("nav_server", "Run the next recovery behavior'" << *current_recovery_behavior_ << "'.");
        mbf_msgs::RecoveryGoal recovery_goal;
        recovery_goal.behavior = *current_recovery_behavior_;
        action_client_recovery_.sendGoal(recovery_goal,
                                         boost::bind(&NavigationServer::actionRecoveryDone, this, _1, _2));
      }
      break;
    case actionlib::SimpleClientGoalState::PREEMPTED:
      ROS_INFO_STREAM_NAMED("nav_server", "The recovery action has been preempted!");
      break;

    case actionlib::SimpleClientGoalState::RECALLED:
      ROS_INFO_STREAM_NAMED("nav_server", "The recovery action has been recalled!");
      break;

    case actionlib::SimpleClientGoalState::REJECTED:
      ROS_FATAL_STREAM_NAMED("nav_server", "The recovery action has been rejected!");
      current_goal_handle_.setRejected();
      break;
    case actionlib::SimpleClientGoalState::LOST:
      ROS_FATAL_STREAM_NAMED("nav_server", "The recovery action has lost the connection to the server!");
      current_goal_handle_.setAborted();
      break;
    default:
      ROS_FATAL_STREAM_NAMED("nav_server", "Reached unreachable case! Unknown state!");
      current_goal_handle_.setAborted();
      break;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "navigation_server");
  NavigationServer nav = NavigationServer();
  ros::spin();
}
