#include "navigation_server.h"
#include <parameter_assertions/assertions.h>
#include <igvc_utils/NodeUtils.hpp>

NavigationServer::NavigationServer()
  : current_state_(NONE)
  , recovery_trigger_(NONE)
  , nh_()
  , private_nh_("~")
  , action_client_get_path_(private_nh_, "/move_base_flex/get_path")
  , action_client_exe_path_(private_nh_, "/move_base_flex/exe_path")
  , action_client_recovery_(private_nh_, "/move_base_flex/recovery")
  , action_server_(private_nh_, "/move_to_waypoint", boost::bind(&NavigationServer::start, this, _1),
                   boost::bind(&NavigationServer::cancel, this), false)
{
  ROS_INFO_STREAM_NAMED("nav_server", "Navigation server created.");
  assertions::getParam(private_nh_, "recovery_enabled", recovery_enabled_);
  assertions::getParam(private_nh_, "oscillation_distance", oscillation_distance_);
  double timeout, wait_time;
  assertions::getParam(private_nh_, "oscillation_timeout", timeout);
  assertions::getParam(private_nh_, "oscillation_wait_time", wait_time);
  oscillation_timeout_ = ros::Duration(timeout);
  oscillation_wait_time_ = ros::Duration(wait_time);

  current_goal_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_to_waypoint/current_goal_pose", 1);

  action_server_.start();
}

void NavigationServer::cancel()
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
}

void NavigationServer::start(GoalHandle goal_handle)
{
  current_goal_handle_ = goal_handle;
  current_state_ = GET_PATH;
  current_goal_handle_.setAccepted();

  ROS_DEBUG_STREAM_NAMED("nav_server", "nav_server started action: move_to_waypoint");

  const igvc_msgs::NavigateWaypointGoal &goal = *goal_handle.getGoal();

  current_goal_pose_publisher_.publish(goal.target_pose);

  // get recovery behaviors from goal, or use default behaviors
  recovery_behaviors_ = goal.recovery_behaviors.empty() ? default_recovery_behaviors_ : goal.recovery_behaviors;
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
    ROS_ERROR_STREAM_NAMED("nav_server", "nav_server could not connect to one or more of move_base_flex actions: "
                                         "'get_path', "
                                         "'exe_path', 'recovery'!");
    navigate_waypoint_result.outcome = igvc_msgs::NavigateWaypointResult ::INTERNAL_ERROR;
    navigate_waypoint_result.message = "Could not connect to the move_base_flex actions!";
    goal_handle.setAborted(navigate_waypoint_result, navigate_waypoint_result.message);
    return;
  }

  start_time_ = ros::Time::now();
  runGetPath();
  processLoop();
}

void NavigationServer::processLoop()
{
  while (ros::ok() && current_state_ != CANCELED && current_state_ != FAILED && current_state_ != SUCCEEDED)
  {
    if (current_state_ == EXE_PATH && ros::Time::now() > time_of_last_get_path + time_between_get_path)
    {
      runGetPath();
      time_of_last_get_path = ros::Time::now();
    }
  }
}

void NavigationServer::runGetPath()
{
  ROS_DEBUG_STREAM_NAMED("nav_server", "nav_server sent goal to get_path client.");
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
      ROS_DEBUG_STREAM_NAMED("nav_server", "nav_server received path from 'get_path'");
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
        ROS_WARN_NAMED("nav_server", "nav_server recovered from planner failure.");
        current_recovery_behavior_ = recovery_behaviors_.begin();
        recovery_trigger_ = NONE;
      }

      if (recovery_trigger_ == EXE_PATH)
      {
        ROS_WARN_NAMED("nav_server", "nav_server recovered from path execution failure.");
        current_recovery_behavior_ = recovery_behaviors_.begin();
        recovery_trigger_ = NONE;
      }

      ROS_DEBUG_STREAM_NAMED("nav_server", "nav_server sending path to 'exe_path'");
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
        current_state_ = FAILED;
      }
      break;
    case actionlib::SimpleClientGoalState::PREEMPTED:
      // the get_path action has been preempted.
      // copy result from get_path action
      navigate_waypoint_result.outcome = result.outcome;
      navigate_waypoint_result.message = result.message;
      current_goal_handle_.setCanceled(navigate_waypoint_result, state.getText());
      current_state_ = CANCELED;
      break;
    case actionlib::SimpleClientGoalState::LOST:
      ROS_FATAL_STREAM_NAMED("nav_server", "Connection lost to the action \"get_path\"!");
      current_goal_handle_.setAborted();
      current_state_ = FAILED;
      break;
    default:
      ROS_FATAL_STREAM_NAMED("nav_server", "Unreachable state reached for get_path: " << state.toString());
      current_state_ = FAILED;
      break;
  }
}

void NavigationServer::runExePath(nav_msgs::Path path)
{
  if (current_state_ != RECOVERY)
  {
    current_state_ = EXE_PATH;
    mbf_msgs::ExePathGoal exe_path_goal;
    exe_path_goal.controller = exe_path_controller_;
    exe_path_goal.path = std::move(path);
    action_client_exe_path_.sendGoal(exe_path_goal, boost::bind(&NavigationServer::actionExePathDone, this, _1, _2),
                                     NavigationServer::actionExePathActive,
                                     boost::bind(&NavigationServer::actionExePathFeedback, this, _1));
  }
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
      navigate_waypoint_result.message = "nav_server: Action 'move_to_waypoint' succeeded!";
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
            ROS_DEBUG_STREAM_NAMED("nav_server", current_goal_handle_.getGoalStatus());
            current_goal_handle_.setAborted(navigate_waypoint_result, state.getText());
            current_state_ = FAILED;
          }
          break;
      }
      break;
    case actionlib::SimpleClientGoalState::PREEMPTED:
      ROS_DEBUG_STREAM_NAMED("nav_server", "Action `exe_path` preempted.");

      break;
    default:
      ROS_FATAL_STREAM_NAMED("nav_server", "Unreachable state reached for exe_path: " << state.toString());
      current_state_ = FAILED;
      break;
  }
}

void NavigationServer::actionExePathActive()
{
  ROS_DEBUG_STREAM_NAMED("nav_server", "The \"exe_path\" action is active.");
}

void NavigationServer::actionExePathFeedback(const mbf_msgs::ExePathFeedbackConstPtr &feedback)
{
  move_base_feedback_.outcome = feedback->outcome;
  move_base_feedback_.message = feedback->message;
  move_base_feedback_.angle_to_goal = feedback->angle_to_goal;
  move_base_feedback_.dist_to_goal = feedback->dist_to_goal;
  move_base_feedback_.current_pose = feedback->current_pose;
  move_base_feedback_.last_cmd_vel = feedback->last_cmd_vel;
  current_goal_handle_.publishFeedback(move_base_feedback_);

  checkForOscillation(feedback->current_pose);
}

void NavigationServer::checkForOscillation(const geometry_msgs::PoseStamped &robot_pose)
{
  // oscillation checking is disabled if oscillation_timeout_ = 0
  // wait until oscillation_wait_time has passed to start checking for oscillation
  if (!oscillation_timeout_.isZero() && ros::Time::now() > start_time_ + oscillation_wait_time_)
  {
    // check if robot has moved more than oscillation distance
    double distance = igvc::get_distance(robot_pose.pose.position, previous_oscillation_pose_.pose.position);
    ROS_DEBUG_STREAM_NAMED("nav_server", "nav_server: Distance from previous pose: " << distance);
    if (distance >= oscillation_distance_)
    {
      last_oscillation_reset_ = ros::Time::now();
      previous_oscillation_pose_ = robot_pose;

      if (recovery_trigger_ == OSCILLATING)
      {
        ROS_INFO_STREAM_NAMED("nav_server", "nav_server: Recovered from robot oscillation.");
        current_recovery_behavior_ = recovery_behaviors_.begin();
        recovery_trigger_ = NONE;
      }
    }
    // check if time has passed timeout
    else if (ros::Time::now() > last_oscillation_reset_ + oscillation_timeout_)
    {
      std::stringstream oscillation_msgs;
      oscillation_msgs << "nav_server: Robot is oscillating for "
                       << (ros::Time::now() - last_oscillation_reset_).toSec() << "s!";
      ROS_WARN_STREAM_NAMED("nav_server", oscillation_msgs.str());
      action_client_exe_path_.cancelGoal();

      if (attemptRecovery())
      {
        recovery_trigger_ = OSCILLATING;
      }
      else
      {
        igvc_msgs::NavigateWaypointResult move_base_result;
        move_base_result.outcome = igvc_msgs::NavigateWaypointResult::OSCILLATION;
        move_base_result.message = oscillation_msgs.str();
        move_base_result.final_pose = robot_pose;
        move_base_result.angle_to_goal = move_base_feedback_.angle_to_goal;
        move_base_result.dist_to_goal = move_base_feedback_.dist_to_goal;
        current_goal_handle_.setAborted(move_base_result, move_base_result.message);
        current_state_ = FAILED;
      }
    }
  }
}

bool NavigationServer::attemptRecovery()
{
  if (!recovery_enabled_)
  {
    ROS_WARN_STREAM_NAMED("nav_server", "nav_server: Recovery behaviors are disabled!");
    return false;
  }

  if (current_recovery_behavior_ == recovery_behaviors_.end())
  {
    if (recovery_behaviors_.empty())
    {
      ROS_WARN_STREAM_NAMED("nav_server", "nav_server: No Recovery Behaviors loaded!");
    }
    else
    {
      ROS_WARN_STREAM_NAMED("nav_server", "nav_server: Executed all available recovery behaviors!");
    }
    return false;
  }

  mbf_msgs::RecoveryGoal recovery_goal;
  recovery_goal.behavior = *current_recovery_behavior_;
  current_state_ = RECOVERY;
  action_client_recovery_.sendGoal(recovery_goal, boost::bind(&NavigationServer::actionRecoveryDone, this, _1, _2));
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
      ROS_INFO_STREAM_NAMED("nav_server", "nav_server: Execution of the recovery behavior '"
                                              << *current_recovery_behavior_ << "' succeeded!");
      current_recovery_behavior_++;
      runGetPath();
      break;
    case actionlib::SimpleClientGoalState::ABORTED:
      ROS_WARN_STREAM_NAMED("nav_server", "Recovery behavior aborted!");
      runNextRecoveryBehavior(navigate_waypoint_result);
      break;
    case actionlib::SimpleClientGoalState::PREEMPTED:
      ROS_INFO_STREAM_NAMED("nav_server", "The recovery action has been preempted!");
      runNextRecoveryBehavior(navigate_waypoint_result);
      break;
    case actionlib::SimpleClientGoalState::REJECTED:
      ROS_FATAL_STREAM_NAMED("nav_server", "The recovery action has been rejected!");
      runNextRecoveryBehavior(navigate_waypoint_result);
      current_state_ = FAILED;
      break;
    case actionlib::SimpleClientGoalState::RECALLED:
      ROS_INFO_STREAM_NAMED("nav_server", "The recovery action has been recalled!");
      break;
    case actionlib::SimpleClientGoalState::LOST:
      ROS_FATAL_STREAM_NAMED("nav_server", "The recovery action has lost the connection to the server!");
      current_goal_handle_.setAborted(navigate_waypoint_result);
      current_state_ = FAILED;
      break;
    default:
      ROS_FATAL_STREAM_NAMED("nav_server", "Reached unreachable case! Unknown state!");
      current_goal_handle_.setAborted(navigate_waypoint_result);
      current_state_ = FAILED;
      break;
  }
}

void NavigationServer::runNextRecoveryBehavior(const igvc_msgs::NavigateWaypointResult &navigate_waypoint_result)
{
  ROS_DEBUG_STREAM_NAMED("nav_server",
                         "nav_server: The recovery behavior '" << *current_recovery_behavior_ << "' failed. ");
  ROS_DEBUG_STREAM("Recovery behavior message: " << navigate_waypoint_result.message
                                                 << ", outcome: " << navigate_waypoint_result.outcome);
  current_recovery_behavior_++;
  if (current_recovery_behavior_ == recovery_behaviors_.end())
  {
    ROS_DEBUG_STREAM_NAMED("nav_server", "nav_server: All recovery behaviors failed. Abort recovering and abort the "
                                         "move_base "
                                         "action");
    current_goal_handle_.setAborted(navigate_waypoint_result, "All recovery behaviors failed.");
    current_state_ = FAILED;
  }
  else
  {
    ROS_INFO_STREAM_NAMED("nav_server",
                          "nav_server: Run the next recovery behavior'" << *current_recovery_behavior_ << "'.");
    mbf_msgs::RecoveryGoal recovery_goal;
    recovery_goal.behavior = *current_recovery_behavior_;
    action_client_recovery_.sendGoal(recovery_goal, boost::bind(&NavigationServer::actionRecoveryDone, this, _1, _2));
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "navigation_server");
  NavigationServer nav = NavigationServer();
  ros::spin();
}
