/**
 * Converts Twist messages to velocity quads for the motors of a swerve drive robot
 */

#ifndef SWERVEDRIVE_H
#define SWERVEDRIVE_H

#include <geometry_msgs/Twist.h>
#include <igvc_msgs/velocity_quad.h>
#include <ros/ros.h>

class SwerveDrive
{
public:
  SwerveDrive();

private:
  ros::NodeHandle nh;
  ros::NodeHandle pNh;

  ros::Subscriber mbf_twist_;
  ros::Publisher vel_pub_;

  std::array<std::array<double, 2>, 4> positions_list;
  std::array<std::array<double, 2>, 4> limits_list;
  std::array<std::array<double, 2>, 4> wheel_info;
  double max_vel_;

  void twistToVelocity(const geometry_msgs::TwistConstPtr& twist);
  bool getParams();
  int set_command_angle(const double& target, const int& wheel_idx);
  double theta_map(const double& theta);
  double isclose(const double& a, const double& b, const double tol = 0.00001, const double bias = 0);
};

#endif  // SWERVEDRIVE_H
