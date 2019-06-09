#include <utility>

#include <igvc_utils/robot_control.h>

igvc_msgs::velocity_pair RobotControl::toMessage(ros::Time stamp) const
{
  igvc_msgs::velocity_pair velocity_pair;
  velocity_pair.left_velocity = left_;
  velocity_pair.right_velocity = right_;
  velocity_pair.header.stamp = stamp;
  return velocity_pair;
}

CurvatureVelocity RobotControl::toKV(double axle_length) const
{
  double linear = linearVelocity();
  double angular = angularVelocity(axle_length);
  double curvature = angular / linear;
  return { curvature, linear };
}

RobotControl RobotControl::fromKV(double curvature, double velocity, double axle_length)
{
  double w = curvature * velocity;

  double v_r = velocity + w * axle_length / 2;
  double v_l = velocity - w * axle_length / 2;

  return { v_l, v_r };
}

std::ostream &operator<<(std::ostream &out, const RobotControl &robot_control)
{
  out << "(" << robot_control.left_ << ", " << robot_control.right_ << ")";
  return out;
}

double RobotControl::linearVelocity() const
{
  return (left_ + right_) / 2;
}

double RobotControl::angularVelocity(double axle_length) const
{
  return (right_ - left_) / axle_length;
}
