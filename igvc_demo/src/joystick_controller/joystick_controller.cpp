#include <igvc_msgs/velocity_pair.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Joy.h>

ros::Publisher cmd_pub;

ros::NodeHandle* nhp;

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  double absoluteMaxVel, maxVel, maxVelIncr;
  nhp->param(std::string("absoluteMaxVel"), absoluteMaxVel, 0.2);
  nhp->param(std::string("maxVel"), maxVel, 0.3);
  nhp->param(std::string("maxVelIncr"), maxVelIncr, 0.02);

  if (msg->buttons[1])
    maxVel -= maxVelIncr;
  else if (msg->buttons[3])
    maxVel += maxVelIncr;
  maxVel = std::min(maxVel, absoluteMaxVel);
  maxVel = std::max(maxVel, 0.0);

  nhp->setParam("maxVel", maxVel);

  int leftJoyAxis, rightJoyAxis;
  bool leftInverted, rightInverted;
  nhp->param(std::string("leftAxis"), leftJoyAxis, 1);
  nhp->param(std::string("rightAxis"), rightJoyAxis, 4);
  nhp->param(std::string("leftInverted"), leftInverted, false);
  nhp->param(std::string("rightInverted"), rightInverted, false);

  igvc_msgs::velocity_pair cmd;
  cmd.left_velocity = msg->axes[leftJoyAxis] * maxVel * (leftInverted ? -1.0 : 1.0);
  cmd.right_velocity = msg->axes[rightJoyAxis] * maxVel * (rightInverted ? -1.0 : 1.0);
  cmd.header.stamp = ros::Time::now();

  cmd_pub.publish(cmd);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystick_controller");
  ros::NodeHandle nh;
  nhp = new ros::NodeHandle("~");

  cmd_pub = nh.advertise<igvc_msgs::velocity_pair>("/motors", 1);

  ros::Subscriber joy_sub = nh.subscribe("/joy", 1, joyCallback);

  ros::spin();

  delete nhp;

  return 0;
}
