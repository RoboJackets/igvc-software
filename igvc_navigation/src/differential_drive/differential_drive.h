/**
 * Converts Twist messages to velocity pairs for the motors of a differential drive robot
 *
 * Author: Tan Gemicioglu <tangem1@hotmail.com>
 */

#ifndef DIFFERENTIALDRIVE_H
#define DIFFERENTIALDRIVE_H

#include <geometry_msgs/Twist.h>
#include <igvc_msgs/velocity_pair.h>

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
