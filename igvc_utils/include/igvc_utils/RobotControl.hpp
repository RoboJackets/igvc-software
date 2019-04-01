#ifndef ROBOTCONTORL_H
#define ROBOTCONTROL_H

struct CurvatureVelocity {
  double curvature;
  double velocity;
};

class RobotControl
{
 public:
  double left;
  double right;

  igvc_msgs::velocity_pair toMessage(ros::Time stamp);
  CurvatureVelocity toKV(double axle_length);

  static RobotControl fromKV(double curvature, double velocity, double axle_length);
};

igvc_msgs::velocity_pair RobotControl::toMessage(ros::Time stamp) {
  igvc_msgs::velocity_pair velocity_pair;
  velocity_pair.left_velocity = left;
  velocity_pair.right_velocity = right;
  velocity_pair.header.stamp = stamp;
  return velocity_pair;
}

RobotControl RobotControl::fromKV(double curvature, double velocity, double axle_length) {
  double w = curvature * velocity;
  double v_r = velocity + w * axle_length / 2;
  double v_l = velocity - w * axle_length / 2;
  return {v_l, v_r};
}

CurvatureVelocity RobotControl::toKV(double axle_length) {
  return {(right - left) / axle_length, (right + left)/2};
}

#endif  // ROBOTCONTROL_H
