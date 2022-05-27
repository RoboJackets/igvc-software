/**
 * Converts Twist messages to velocity quads for the motors of a swerve drive robot
 */

#ifndef DIFFERENTIALDRIVE_H
#define DIFFERENTIALDRIVE_H

#include <geometry_msgs/Twist.h>
#include <igvc_msgs/velocity_quad.h>

class DifferentialDrive
{
public:
  DifferentialDrive();
  ros::Subscriber mbf_twist_;

private:
  ros::Publisher vel_pub_;
  double axle_length_{};
  double max_vel_{};

  void twistToVelocity(geometry_msgs::Twist twist);
};

#endif  // DIFFERENTIALDRIVE_H
