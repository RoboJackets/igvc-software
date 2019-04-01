#include <utility>

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

#include <igvc_utils/NodeUtils.hpp>
#include <thread>

#include <igvc_utils/robot_control.h>
#include "trajectory_follower.h"

TrajectoryFollower::TrajectoryFollower()
{
  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  igvc::getParam(pNh, "topics/path", path_topic_);
  igvc::param(pNh, "loop_hz", loop_hz_, 25.0);
  igvc::param(pNh, "axle_length_", axle_length_, 0.48);

  ros::Subscriber path_sub = nh.subscribe("/path", 1, &TrajectoryFollower::trajectoryCallback, this);
  control_pub_ = nh.advertise<igvc_msgs::velocity_pair>("/motors", 1);
  std::thread trajectory_thread(&TrajectoryFollower::trajectoryFollowLoop, this);

  ros::spin();
  ROS_INFO("Shutting down...");
  trajectory_thread.join();
}

void TrajectoryFollower::trajectoryCallback(igvc_msgs::trajectoryConstPtr trajectory)
{
  trajectory_ = std::move(trajectory);
}

void TrajectoryFollower::trajectoryFollowLoop()
{
  ros::Rate rate(loop_hz_);
  while (ros::ok())
  {
    followTrajectory();
    rate.sleep();
  }
}

void TrajectoryFollower::followTrajectory()
{
  RobotControl control = getControl();
  control_pub_.publish(control.toMessage(ros::Time::now()));
}

RobotControl TrajectoryFollower::getControl()
{
  ros::Time current_time = ros::Time::now();
  for (size_t i = 0; i < trajectory_->trajectory.size() - 1; i++)
  {
    ros::Time last_time = trajectory_->trajectory[i].header.stamp;
    ros::Time next_time = trajectory_->trajectory[i + 1].header.stamp;
    if (last_time < current_time && current_time < next_time)
    {
      double ratio = (current_time - last_time).toSec() / (next_time - last_time).toSec();
      double curvature = (1 - ratio) * trajectory_->trajectory[i].curvature + ratio * trajectory_->trajectory[i + 1].curvature;
      double velocity = (1 - ratio) * trajectory_->trajectory[i].velocity + ratio * trajectory_->trajectory[i + 1].velocity;
      return RobotControl::fromKV(curvature, velocity, axle_length_);
    }
  }
  return RobotControl::fromKV(trajectory_->trajectory.back().curvature, trajectory_->trajectory.back().velocity, axle_length_);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_follower");
  TrajectoryFollower trajectoryFollower;
  return 0;
}
