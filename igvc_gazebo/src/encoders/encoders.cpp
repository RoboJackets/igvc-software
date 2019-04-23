#include <igvc_msgs/velocity_pair.h>
#include <sensor_msgs/JointState.h>
#include <igvc_utils/NodeUtils.hpp>

#include "encoders.h"

Encoders::Encoders() : nh_{}, pNh_{ "~" }
{
  igvc::param(pNh_, "node/loop_rate", loop_rate_, 20.0);
  igvc::param(pNh_, "wheel_radius", wheel_radius_, 0.35);
  igvc::param(pNh_, "topics/encoders", encoder_topic_, "/encoders");
  igvc::param(pNh_, "topics/joint_state", joint_state_topic_, "/joint_states");

  igvc::param(pNh_, "left_joint", left_joint_, "left_axle");
  igvc::param(pNh_, "right_joint", right_joint_, "right_axle");

  encoder_pub_ = nh_.advertise<igvc_msgs::velocity_pair>(encoder_topic_, 1);
  joint_state_sub_ = nh_.subscribe(joint_state_topic_, 1, &Encoders::jointStateCallback, this);

  ros::Timer timer = nh_.createTimer(ros::Duration(1 / loop_rate_), &Encoders::publishValues, this);
  ros::spin();
}

void Encoders::jointStateCallback(const sensor_msgs::JointStateConstPtr &msg)
{
  auto left_it = std::find(msg->name.begin(), msg->name.end(), left_joint_);
  if (left_it != msg->name.end())
  {
    auto index = std::distance(msg->name.begin(), left_it);
    left_velocity_ = (msg->velocity[index]) * (wheel_radius_);
  }

  auto right_it = std::find(msg->name.begin(), msg->name.end(), right_joint_);
  if (right_it != msg->name.end())
  {
    auto index = std::distance(msg->name.begin(), right_it);
    right_velocity_ = (msg->velocity[index]) * (wheel_radius_);
  }
}

void Encoders::publishValues(const ros::TimerEvent &timer_event)
{
  igvc_msgs::velocity_pair velocity_pair;
  velocity_pair.left_velocity = left_velocity_;
  velocity_pair.right_velocity = right_velocity_;
  velocity_pair.duration = (timer_event.current_real - timer_event.last_real).toSec();
  velocity_pair.header.stamp = ros::Time::now();

  encoder_pub_.publish(velocity_pair);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "encoders");
  Encoders encoders;
  return 0;
}
