#ifndef SRC_ENCODERS_H
#define SRC_ENCODERS_H

#include <ros/ros.h>

class Encoders
{
public:
  Encoders();

private:
  void jointStateCallback(const sensor_msgs::JointStateConstPtr &msg);
  void publishValues(const ros::TimerEvent &timer_event);

  std::string encoder_topic_;
  std::string joint_state_topic_;
  std::string left_joint_;
  std::string right_joint_;

  double loop_rate_{};

  double wheel_radius_{};
  double left_velocity_{};
  double right_velocity_{};

  ros::NodeHandle nh_;
  ros::NodeHandle pNh_;

  ros::Publisher encoder_pub_;

  ros::Subscriber joint_state_sub_;
};

#endif  // SRC_ENCODERS_H
