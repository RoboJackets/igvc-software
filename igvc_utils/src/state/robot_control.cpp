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
  if (axle_length == -1)
  {
    axle_length = axle_length_;
  }
  return { (right_ - left_) / axle_length, (right_ + left_) / 2 };
}

RobotControl RobotControl::fromKV(double curvature, double velocity, double axle_length)
{
  double w = curvature * velocity;
  double v_r = velocity + w * axle_length / 2;
  double v_l = velocity - w * axle_length / 2;
  return { v_l, v_r, axle_length };
}

std::ostream &operator<<(std::ostream &out, const RobotControl &robot_control)
{
  out << "(" << robot_control.left_ << ", " << robot_control.right_ << ")";
  return out;
}
