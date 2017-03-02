#include <igvc_msgs/velocity_pair.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <chrono>
#include <fstream>

bool enabled = false;

void enabled_callback(const std_msgs::BoolConstPtr& msg)
{
  enabled = msg->data;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pidtester");

  ros::NodeHandle nh;

  ros::Subscriber enabled_sub = nh.subscribe("/robot_enabled", 1, enabled_callback);

  ros::Publisher cmd_pub = nh.advertise<igvc_msgs::velocity_pair>("/motors", 1);

  while (!enabled)
  {
    ros::spinOnce();
  }

  usleep(500);

  auto duration = 5.0;

  igvc_msgs::velocity_pair cmd;
  cmd.duration = duration;
  cmd.left_velocity = 1.0;
  cmd.right_velocity = 1.0;

  cmd_pub.publish(cmd);

  ros::spinOnce();
  sleep((__useconds_t)(duration));

  cmd.left_velocity = 0.0;
  cmd.right_velocity = 0.0;

  cmd_pub.publish(cmd);

  ros::spin();

  return 0;
}
