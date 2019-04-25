#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H

#include <igvc_msgs/velocity_pair.h>

struct CurvatureVelocity
{
  double curvature;
  double velocity;
};

class RobotControl
{
public:
  double left_{};
  double right_{};
  double axle_length_{};

  friend std::ostream &operator<<(std::ostream &out, const RobotControl &robot_control);

  igvc_msgs::velocity_pair toMessage(ros::Time stamp) const;
  CurvatureVelocity toKV(const std::optional<double>& axle_length = std::nullopt) const;

  double linearVelocity() const;
  double angularVelocity(const std::optional<double>& axle_length = std::nullopt) const;

  static RobotControl fromKV(double curvature, double velocity, double axle_length);
};

#endif  // ROBOTCONTROL_H
