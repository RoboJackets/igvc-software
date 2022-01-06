/**
 * Converts Twist messages to velocity quads for the motors of a swerve drive robot
 */

#ifndef SWERVEDRIVE_H
#define SWERVEDRIVE_H

#include <geometry_msgs/Twist.h>
#include <igvc_msgs/velocity_quad.h>

class SwerveDrive
{
public:
  SwerveDrive();

private:
  ros::Subscriber mbf_twist_;
  ros::Publisher vel_pub_;
  ros::NodeHandle nh;
  ros::NodeHandle pNh;
  std::array<double, 4> radii_list;
  std::array<std::array<double, 2>, 4> positions_list;
  std::array<std::array<double, 2>, 4> wheel_info;
  double axle_length_{};
  double max_vel_{};

  void twistToVelocity(geometry_msgs::Twist twist);
  bool getParams();
};

#endif  // SWERVEDRIVE_H
