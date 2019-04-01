#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H

#include <igvc_msgs/velocity_pair.h>

struct CurvatureVelocity {
  double curvature;
  double velocity;
};

class RobotControl
{
 public:
  double left_{};
  double right_{};
  double axle_length_{};

  igvc_msgs::velocity_pair toMessage(ros::Time stamp) const;
  CurvatureVelocity toKV(double axle_length = -1) const;

  static RobotControl fromKV(double curvature, double velocity, double axle_length);
};

#endif  // ROBOTCONTROL_H
