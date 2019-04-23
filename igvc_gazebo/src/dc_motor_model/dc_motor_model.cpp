#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <igvc_utils/NodeUtils.hpp>

#include "dc_motor_model.h"

DCMotorModel::DCMotorModel() : nh_{}, pNh_{ "~" }
{
  getParams();
  setupPublishers();
  setupSubscribers();

  ros::Timer timer = nh_.createTimer(ros::Duration(1 / loop_rate_), &DCMotorModel::updateLoop, this);
  ros::spin();
}

void DCMotorModel::updateLoop(const ros::TimerEvent &timer_event)
{
  // τ = k/R ( v - k * ω)
  double torque =
      motor_model_params_.k / motor_model_params_.internal_resistance * (voltage_ - motor_model_params_.k * velocity_);

  std_msgs::Float64 effort;
  effort.data = torque;
  effort_pub_.publish(effort);
}

void DCMotorModel::voltageCallback(const std_msgs::Float64 voltage)
{
  voltage_ = voltage.data;
}

void DCMotorModel::jointStateCallback(const sensor_msgs::JointStateConstPtr &msg)
{
  auto iter = std::find(msg->name.begin(), msg->name.end(), joint_name_);

  if (iter != msg->name.end())
  {
    auto index = std::distance(msg->name.begin(), iter);
    velocity_ = msg->velocity[index];
  }
}

void DCMotorModel::setupPublishers()
{
  effort_pub_ = nh_.advertise<std_msgs::Float64>(effort_topic_, 1);
}

void DCMotorModel::setupSubscribers()
{
  voltage_sub_ = nh_.subscribe(voltage_topic_, 1, &DCMotorModel::voltageCallback, this);
  joint_state_sub_ = nh_.subscribe(joint_state_topic_, 1, &DCMotorModel::jointStateCallback, this);
}

void DCMotorModel::getParams()
{
  igvc::param(pNh_, "node/loop_rate", loop_rate_, 100.0);

  igvc::getParam(pNh_, "topics/effort", effort_topic_);
  igvc::getParam(pNh_, "topics/voltage", voltage_topic_);
  igvc::getParam(pNh_, "topics/joint_state", joint_state_topic_);
  igvc::getParam(pNh_, "joint_name", joint_name_);

  igvc::getParam(pNh_, "motor_model/k", motor_model_params_.k);
  igvc::getParam(pNh_, "motor_model/internal_resistance", motor_model_params_.internal_resistance);

  ROS_INFO_STREAM("effort: " << effort_topic_);
  ROS_INFO_STREAM("joint_name: " << joint_name_);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dc_motor_model");
  DCMotorModel dc_motor_model;
}
