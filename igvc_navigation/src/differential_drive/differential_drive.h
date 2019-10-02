/**
 * Converts Twist messages to velocity pairs for the motors of a differential drive robot
 *
 * Author: Tan Gemicioglu <tangem1@hotmail.com>
 */

#ifndef DIFFERENTIALDRIVE_H
#define DIFFERENTIALDRIVE_H

#include <igvc_msgs/velocity_pair.h>
#include <geometry_msgs/Twist.h>

class DifferentialDrive {
public:
  DifferentialDrive();

private:
  ros::Publisher vel_pub_;
  double axle_length_{};

  void twistToVelocity(geometry_msgs::Twist twist);
};

#endif  // DIFFERENTIALDRIVE_H
