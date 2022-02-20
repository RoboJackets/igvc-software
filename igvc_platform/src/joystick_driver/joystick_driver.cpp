#include "joystick_driver.h"

JoystickDriver::JoystickDriver() : nhp{ "~" }
{
  cmd_pub = nh.advertise<igvc_msgs::velocity_pair>("/motors", 1);
  joy_sub = nh.subscribe("/joy", 1, &JoystickDriver::joyCallback, this);

  updater_ptr = std::make_unique<diagnostic_updater::Updater>();
  updater_ptr->setHardwareID("Joystick");
  updater_ptr->add("Joystick Diagnostic", this, &JoystickDriver::joystick_diagnostic);

  assertions::param(nhp, "absoluteMaxVel", absoluteMaxVel, 1.0);
  assertions::param(nhp, "maxVel", maxVel, 1.6);
  assertions::param(nhp, "maxVelIncr", maxVelIncr, 0.1);
  assertions::param(nhp, "leftJoyAxis", leftJoyAxis, 1);
  assertions::param(nhp, "rightJoyAxis", rightJoyAxis, 4);
  assertions::param(nhp, "leftInverted", leftInverted, false);
  assertions::param(nhp, "rightInverted", rightInverted, false);
}

void JoystickDriver::joystick_diagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Joystick Online");
  stat.add("absolute_max_velocity", absoluteMaxVel);
  stat.add("max_velocity", maxVel);
  stat.add("max_velocity_increment", maxVelIncr);
}

void JoystickDriver::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  if (msg->buttons[1])
  {
    maxVel -= maxVelIncr;
  }
  else if (msg->buttons[3])
  {
    maxVel += maxVelIncr;
  }
  maxVel = std::min(maxVel, absoluteMaxVel);
  maxVel = std::max(maxVel, 0.0);

  nhp.setParam("maxVel", maxVel);

  updater_ptr->update();

  igvc_msgs::velocity_pair cmd;
  cmd.left_velocity = msg->axes[leftJoyAxis] * maxVel * (leftInverted ? -1.0 : 1.0);
  cmd.right_velocity = msg->axes[rightJoyAxis] * maxVel * (rightInverted ? -1.0 : 1.0);
  cmd.header.stamp = ros::Time::now();

  cmd_pub.publish(cmd);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystick_driver");
  JoystickDriver joystick_driver;
  ros::spin();
}
