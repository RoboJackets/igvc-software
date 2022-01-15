/**
 * Sends velocity_quad messages from the motors topic to the motors of the swervi model in simulation
 */

#ifndef SWERVECONTROL_H
#define SWERVECONTROL_H

#include <igvc_msgs/velocity_quad.h>
#include <parameter_assertions/assertions.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <algorithm>
#include <chrono>

class SwerveControl
{
public:
  SwerveControl();

private:
  void ControlLoop();
  void jointStateCallback(const sensor_msgs::JointStateConstPtr &msg);
  void speedCallback(const igvc_msgs::velocity_quad::ConstPtr &msg);

  struct controlInfo
  {
    double set_point;
    double measured;
    double last_error;
    double error_accum;
    double effort;
    double P;
    double I;
    double D;
  };

  // nodehandles
  ros::NodeHandle handle;
  ros::NodeHandle pNh;

  // Publishers
  std::vector<ros::Publisher> effortPublishers;

  ros::Publisher back_left_shock_publisher;
  ros::Publisher back_right_shock_publisher;
  ros::Publisher front_left_shock_publisher;
  ros::Publisher front_right_shock_publisher;

  ros::Publisher wheel_speed_publisher;
  ros::Publisher enabled_pub;

  ros::Subscriber speed_sub;
  ros::Subscriber state_sub;

  ros::Time prev, now;

  // Parameters for all 4 driving motors
  double speed_P_fl, speed_P_fr, speed_P_bl, speed_P_br;
  double speed_I_fl, speed_I_fr, speed_I_bl, speed_I_br;
  double speed_D_fl, speed_D_fr, speed_D_bl, speed_D_br;
  double rate_var;
  double wheel_radius;
  double max_effort;
  double alpha;

  std::vector<controlInfo> motors;

  std::vector<std::string> joint_names;
};

#endif  // SWERVECONTROL_H
