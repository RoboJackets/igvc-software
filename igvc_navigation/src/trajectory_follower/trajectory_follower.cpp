#include <utility>

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

#include <igvc_utils/NodeUtils.hpp>
#include <thread>

#include <igvc_utils/robot_control.h>
#include <parameter_assertions/assertions.h>

#include "trajectory_follower.h"

TrajectoryFollower::TrajectoryFollower()
{
  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  using namespace assertions;
  Asserter asserter;
  asserter.getParam(pNh, "topics/path", path_topic_);
  asserter.param(pNh, "loop_hz", loop_hz_, 25.0);
  asserter.param(pNh, "axle_length", axle_length_, 0.48);
  asserter.param(pNh, "motor_loop_hz", motor_loop_hz_, 25.0);
  asserter.param(pNh, "min_velocity", min_velocity_, 0.2);

  ros::Subscriber path_sub = nh.subscribe(path_topic_, 1, &TrajectoryFollower::trajectoryCallback, this);
  control_pub_ = nh.advertise<igvc_msgs::velocity_pair>("/motors", 1);
  std::thread trajectory_thread(&TrajectoryFollower::trajectoryFollowLoop, this);

  ros::spin();
  ROS_INFO("Shutting down...");
  trajectory_thread.join();
}

void TrajectoryFollower::trajectoryCallback(igvc_msgs::trajectoryConstPtr trajectory)
{
  std::lock_guard<std::mutex> trajectory_lock(trajectory_mutex_);
  trajectory_ = trajectory;
  time_delta_ = ros::Time::now() - trajectory->trajectory.front().header.stamp;
}

void TrajectoryFollower::trajectoryFollowLoop()
{
  ros::Rate rate(loop_hz_);
  while (ros::ok())
  {
    if (trajectory_.get())
    {
      followTrajectory();
    }
    else
    {
      igvc_msgs::velocity_pair pair;
      pair.left_velocity = 0.0;
      pair.right_velocity = 0.0;
      control_pub_.publish(pair);
    }
    rate.sleep();
  }
}

void TrajectoryFollower::followTrajectory()
{
  RobotControl control = getControl();
  ensureAboveDeadband(control);

  control_pub_.publish(control.toMessage(ros::Time::now()));
}

RobotControl TrajectoryFollower::getControl()
{
  std::lock_guard<std::mutex> trajectory_lock(trajectory_mutex_);

  if (trajectory_->trajectory.size() < 3)
  {
    return { 0.0, 0.0 };
  }

  ros::Time current_time = ros::Time::now() - time_delta_ + ros::Duration(1 / motor_loop_hz_);
  for (size_t i = 0; i < trajectory_->trajectory.size() - 2; i++)
  {
    ros::Time last_time = trajectory_->trajectory[i].header.stamp;
    ros::Time next_time = trajectory_->trajectory[i + 1].header.stamp;
    if (last_time < current_time && current_time < next_time)
    {
      igvc_msgs::trajectory_point last = trajectory_->trajectory[i];
      igvc_msgs::trajectory_point next = trajectory_->trajectory[i + 1];

      double ratio = (current_time - last_time).toSec() / (next_time - last_time).toSec();

      double curvature = (1 - ratio) * last.curvature + ratio * next.curvature;
      double velocity = (1 - ratio) * last.velocity + ratio * next.velocity;

      //      ROS_INFO_STREAM("Interpolating from (" << last.curvature << ", " << last.velocity << ") to (" <<
      //      next.curvature << ", " << next.velocity << ")"); ROS_INFO_STREAM("\tResult: " <<
      //      RobotControl::fromKV(curvature, velocity, axle_length_));
      return RobotControl::fromKV(curvature, velocity, axle_length_);
    }
  }
  return RobotControl::fromKV(trajectory_->trajectory.back().curvature, trajectory_->trajectory.back().velocity,
                              axle_length_);
}

void TrajectoryFollower::ensureAboveDeadband(RobotControl& control)
{
  if (control.left_ != 0 || control.right_ != 0)
  {
    if (std::abs(control.left_) < min_velocity_ || std::abs(control.right_) < min_velocity_)
    {
      if (std::abs(control.left_) < min_velocity_)
      {
        control.left_ = std::copysign(min_velocity_, control.left_);
      }
      if (std::abs(control.right_) < min_velocity_)
      {
        control.right_ = std::copysign(min_velocity_, control.right_);
      }
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_follower");
  TrajectoryFollower trajectoryFollower;
  return 0;
}
