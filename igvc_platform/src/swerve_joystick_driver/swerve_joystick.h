/**
 * Converts joystick messages to velocity quads for the motors of a swerve drive robot
 */

#ifndef SWERVEJOY_H
#define SWERVEJOY_H

#include <igvc_msgs/velocity_quad.h>
#include <sensor_msgs/Joy.h>
#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

class SwerveJoy
{
public:
  SwerveJoy();

private:
  void joystick_diagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);

  ros::NodeHandle nh;
  ros::NodeHandle pNh;

  ros::Publisher cmd_pub;

  ros::Subscriber joy_sub;

  diagnostic_updater::Updater updater;

  double absoluteMaxVel, maxVel, maxVelIncr, maxAngle;
};

#endif  // SWERVEJOY_H
