#include <parameter_assertions/assertions.h>

#include "swerve_joystick.h"

void SwerveJoy::joystick_diagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Joystick Online");
  assertions::param(pNh, "absoluteMaxVel", absoluteMaxVel, 1.0);
  assertions::param(pNh, "maxVel", maxVel, 1.6);
  assertions::param(pNh, "maxVelIncr", maxVelIncr, 0.1);

  stat.add("absolute_max_velocity", absoluteMaxVel);
  stat.add("max_velocity", maxVel);
  stat.add("max_velocity_increment", maxVelIncr);
}

void SwerveJoy::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  assertions::param(pNh, "absoluteMaxVel", absoluteMaxVel, 1.0);
  assertions::param(pNh, "maxVel", maxVel, 1.6);
  assertions::param(pNh, "maxAngle", maxAngle, 1.57);
  assertions::param(pNh, "maxVelIncr", maxVelIncr, 0.1);

  if (msg->buttons[1])
    maxVel -= maxVelIncr;
  else if (msg->buttons[3])
    maxVel += maxVelIncr;
  maxVel = std::min(maxVel, absoluteMaxVel);
  maxVel = std::max(maxVel, 0.0);

  pNh.setParam("maxVel", maxVel);

  assertions::param(pNh, "stickLeft_UDAxis", stickLeft_UDAxis, 1);
  assertions::param(pNh, "stickRight_UDAxis", stickRight_UDAxis, 4);
  assertions::param(pNh, "stickLeft_LRAxis", stickLeft_LRAxis, 0);
  assertions::param(pNh, "stickRight_LRAxis", stickRight_LRAxis, 3);
  assertions::param(pNh, "leftUDInverted", leftUDInverted, false);
  assertions::param(pNh, "rightUDInverted", rightUDInverted, false);
  assertions::param(pNh, "leftLRInverted", leftLRInverted, false);
  assertions::param(pNh, "rightLRInverted", rightLRInverted, false);

  updater.update();

  igvc_msgs::velocity_quad cmd;
  cmd.fl_velocity = msg->axes[stickLeft_UDAxis] * maxVel * (leftUDInverted ? -1.0 : 1.0);
  cmd.fr_velocity = msg->axes[stickRight_UDAxis] * maxVel * (rightUDInverted ? -1.0 : 1.0);
  cmd.bl_velocity = msg->axes[stickLeft_UDAxis] * maxVel * (leftUDInverted ? -1.0 : 1.0);
  cmd.br_velocity = msg->axes[stickRight_UDAxis] * maxVel * (rightUDInverted ? -1.0 : 1.0);
  cmd.fl_angle = msg->axes[stickLeft_LRAxis] * maxAngle * (leftLRInverted ? -1.0 : 1.0);
  cmd.fr_angle = msg->axes[stickLeft_LRAxis] * maxAngle * (leftLRInverted ? -1.0 : 1.0);
  cmd.bl_angle = msg->axes[stickRight_LRAxis] * maxAngle * (rightLRInverted ? -1.0 : 1.0);
  cmd.br_angle = msg->axes[stickRight_LRAxis] * maxAngle * (rightLRInverted ? -1.0 : 1.0);
  cmd.header.stamp = ros::Time::now();

  cmd_pub.publish(cmd);
}

SwerveJoy::SwerveJoy() : pNh{ "~" }
{
  cmd_pub = nh.advertise<igvc_msgs::velocity_quad>("/motors", 1);
  joy_sub = nh.subscribe("/joy", 1, &SwerveJoy::joyCallback, this);
  updater.setHardwareID("Joystick");
  updater.add("Joystick Diagnostic", this, &SwerveJoy::joystick_diagnostic);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "swerve_joystick_driver");
  SwerveJoy swerve_joy;
  ros::spin();
}
