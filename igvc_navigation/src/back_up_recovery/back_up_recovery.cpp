#include "back_up_recovery.h"
#include <cmath>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Twist.h>
#include <algorithm>
#include <igvc_utils/NodeUtils.hpp>
#include <memory>
#include <tf2/utils.h>
#include <parameter_assertions/assertions.h>

// register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(back_up_recovery::BackUpRecovery, nav_core::RecoveryBehavior);

namespace back_up_recovery
{
BackUpRecovery::BackUpRecovery() : initialized_(false), world_model_(nullptr)
{
}

void BackUpRecovery::initialize(std::string name, tf2_ros::Buffer *, costmap_2d::Costmap2DROS *,
                                costmap_2d::Costmap2DROS *local_costmap)
{
  if (!initialized_)
  {
    local_costmap_ = std::unique_ptr<costmap_2d::Costmap2DROS>(local_costmap);

    // get some parameters from the parameter server
    ros::NodeHandle private_nh("~");

    assertions::getParam(private_nh, "/move_base_flex/back_up_recovery/velocity", velocity_);
    assertions::getParam(private_nh, "/move_base_flex/back_up_recovery/frequency", frequency_);
    assertions::getParam(private_nh, "/move_base_flex/back_up_recovery/threshold_distance",
                         threshold_distance_);  // distance robot backs up before exiting back up behavior
    assertions::getParam(private_nh, "/move_base_flex/back_up_recovery/obstacle_distance",
                         obstacle_distance_);  // closest the robot will get to an obstacle from behind

    world_model_ = std::make_unique<base_local_planner::CostmapModel>(*local_costmap_->getCostmap());

    initialized_ = true;
  }
  else
  {
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

BackUpRecovery::~BackUpRecovery()
{
}

void BackUpRecovery::runBehavior()
{
  if (!initialized_)
  {
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if (local_costmap_ == nullptr)
  {
    ROS_ERROR("The costmap passed to the BackupRecovery object cannot be NULL. Doing nothing.");
    return;
  }
  ROS_WARN("Backup recovery behavior started.");

  ros::Rate r(frequency_);
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 5);

  geometry_msgs::PoseStamped global_pose;
  local_costmap_->getRobotPose(global_pose);

  double current_yaw = tf2::getYaw(global_pose.pose.orientation);
  geometry_msgs::Point current_position = global_pose.pose.position;
  geometry_msgs::Point start_position = current_position;
  double x = current_position.x - std::cos(current_yaw) * obstacle_distance_;
  double y = current_position.y - std::sin(current_yaw) * obstacle_distance_;
  double footprint_cost = world_model_->footprintCost(x, y, current_yaw, local_costmap_->getRobotFootprint(), 0.0, 0.0);
  bool next_step_valid = footprint_cost >= 0.0;

  geometry_msgs::Twist vel_msg;
  vel_msg.linear.x = -velocity_;
  vel_pub.publish(vel_msg);

  while (n.ok() && igvc::get_distance(current_position, start_position) < threshold_distance_ && next_step_valid)
  {
    local_costmap_->getRobotPose(global_pose);
    current_position = global_pose.pose.position;
    x = current_position.x - std::cos(current_yaw) * obstacle_distance_;
    y = current_position.y - std::sin(current_yaw) * obstacle_distance_;
    footprint_cost = world_model_->footprintCost(x, y, current_yaw, local_costmap_->getRobotFootprint(), 0.0, 0.0);
    next_step_valid = footprint_cost >= 0.0;

    vel_pub.publish(vel_msg);
    r.sleep();
  }

  vel_msg.linear.x = 0;
  vel_pub.publish(vel_msg);
}
}  // namespace back_up_recovery
