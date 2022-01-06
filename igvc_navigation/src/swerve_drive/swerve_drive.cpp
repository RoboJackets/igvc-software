#include <parameter_assertions/assertions.h>
#include <ros/ros.h>

#include "swerve_drive.h"

SwerveDrive::SwerveDrive() : pNh{ "~" }
{
  if (!getParams()) {
    ROS_ERROR_STREAM("Unable to import parameters!");
    return;
  }
  assertions::param(pNh, "max_vel", max_vel_, 3.0);
  mbf_twist_ = nh.subscribe("/cmd_vel", 1, &SwerveDrive::twistToVelocity, this);
  vel_pub_ = nh.advertise<igvc_msgs::velocity_quad>("/motors", 1);
}

void SwerveDrive::twistToVelocity(geometry_msgs::Twist twist)
{
  if (!std::isfinite(twist.linear.x) || !std::isfinite(twist.linear.y) || !std::isfinite(twist.angular.z)) {
    ROS_WARN_THROTTLE(1.0, "Recieved NaN in velocity command. Ignoring.");
    return;
  }

  const double speedX = twist.linear.x;
  const double speedY = twist.linear.y;
  const double rotation = twist.angular.z;

  const size_t num_wheels = 4;
  for (size_t i = 0; i < num_wheels; ++i) {
    double wheel_vx = speedX - rotation * positions_list[i][1];
    double wheel_vy = speedY + rotation * positions_list[i][0];

    double wheel_V = sqrt(pow(wheel_vx, 2) + pow(wheel_vy, 2)) / radii_list[i];
    double wheel_dir = atan2(wheel_vy, wheel_vx);

    wheel_V = std::min(max_vel_, wheel_V);

    wheel_info[i][0] = wheel_V;
    wheel_info[i][1] = wheel_dir;
  }
  // contructing vel message
  igvc_msgs::velocity_quad vel_msg;
  vel_msg.fl_velocity = wheel_info[0][0];
  vel_msg.bl_velocity = wheel_info[1][0];
  vel_msg.br_velocity = wheel_info[2][0];
  vel_msg.fr_velocity = wheel_info[3][0];
  
  vel_msg.fl_angle = wheel_info[0][1];
  vel_msg.bl_angle = wheel_info[1][1];
  vel_msg.br_angle = wheel_info[2][1];
  vel_msg.fr_angle = wheel_info[3][1];
  vel_msg.duration = 0.02;
  vel_msg.header.stamp = ros::Time::now();
  
  vel_pub_.publish(vel_msg);
}

bool SwerveDrive::getParams()
{
  XmlRpc::XmlRpcValue xml_list;
  // radii
  if (!nh.getParam("swerve_drive/radii", xml_list)) {
    ROS_ERROR_STREAM("Unable to retrieve radii list");
    return false;
  }
  if (xml_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR_STREAM("radii list not of type array");
    return false;
  }
  const int num_wheels = 4;
  if (xml_list.size() != num_wheels) {
    ROS_ERROR_STREAM("radii list not of size 4");
    return false;
  }
  for (int i = 0; i < num_wheels; ++i) {
    if (xml_list[i].getType() != XmlRpc::XmlRpcValue::TypeDouble &&
      xml_list[i].getType() != XmlRpc::XmlRpcValue::TypeInt) {
      ROS_ERROR_STREAM("radii #" << i << " is not of type double or int");
      return false;
    }
    radii_list[i] = static_cast<double>(xml_list[i]);
  }

  // positions
  if (!nh.getParam("swerve_drive/positions", xml_list)) {
    ROS_ERROR_STREAM("Unable to retrieve position list");
    return false;
  }
  if (xml_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR_STREAM("position list not of type array");
    return false;
  }
  if (xml_list.size() != num_wheels) {
    ROS_ERROR_STREAM("position list not of size 4");
    return false;
  }
  for (int i = 0; i < num_wheels; ++i) {
    if (xml_list[i].getType() != XmlRpc::XmlRpcValue::TypeArray) {
      ROS_ERROR_STREAM("position #" << i << " is not of type array");
      return false;
    }
    if (xml_list[i].size() != 2) {
      ROS_ERROR_STREAM("position #" << i << " is not size 2");
    }
    std::array<double,2> hold;
    for (int j = 0; j < 2; ++j) {
      if (xml_list[i][j].getType() != XmlRpc::XmlRpcValue::TypeDouble &&
        xml_list[i][j].getType() != XmlRpc::XmlRpcValue::TypeInt) {
        ROS_ERROR_STREAM("position #{" << i << ", " << j << "} is not of type double or int");
        return false;
      }
      hold[j] = static_cast<double>(xml_list[i][j]);
    }
    positions_list[i] = hold;
  }
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "swerve_drive");
  SwerveDrive swerve_drive;
  ros::spin();
}
