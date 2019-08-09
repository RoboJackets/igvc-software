#include <igvc_msgs/velocity_pair.h>
#include <parameter_assertions/assertions.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <algorithm>
#include <chrono>

#include <igvc_utils/NodeUtils.hpp>

double speed_set_point_left = 0.0;
double speed_set_point_right = 0.0;
double speed_measured_left = 0.0;
double speed_measured_right = 0.0;
double wheel_radius;

void speedCallback(const igvc_msgs::velocity_pair::ConstPtr &msg)
{
  if (msg->left_velocity == msg->left_velocity)
  {
    speed_set_point_left = msg->left_velocity;
  }
  if (msg->right_velocity == msg->right_velocity)
  {
    speed_set_point_right = msg->right_velocity;
  }
}

void jointStateCallback(const sensor_msgs::JointStateConstPtr &msg)
{
  auto iter = std::find(msg->name.begin(), msg->name.end(), std::string{ "left_axle" });

  if (iter != msg->name.end())
  {
    auto index = std::distance(msg->name.begin(), iter);

    speed_measured_left = (msg->velocity[index]) * (wheel_radius);
  }

  iter = std::find(msg->name.begin(), msg->name.end(), std::string{ "right_axle" });

  if (iter != msg->name.end())
  {
    auto index = std::distance(msg->name.begin(), iter);

    speed_measured_right = (msg->velocity[index]) * (wheel_radius);
  }

  // ROS_INFO_STREAM("right speed: " << speed_measured_right << " left speed: " << speed_measured_left);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jaymii_controller");

  ros::NodeHandle handle;
  ros::NodeHandle pNh("~");

  ros::Publisher left_wheel_effort_publisher =
      handle.advertise<std_msgs::Float64>("/left_wheel_effort_controller/command", 1);
  ros::Publisher right_wheel_effort_publisher =
      handle.advertise<std_msgs::Float64>("/right_wheel_effort_controller/command", 1);
  ros::Publisher wheel_speed_publisher = handle.advertise<igvc_msgs::velocity_pair>("/encoders", 1);

  ros::Publisher right_wheel_shock_publisher =
      handle.advertise<std_msgs::Float64>("right_wheel_shock_controller/command", 1, true);
  ros::Publisher left_wheel_shock_publisher =
      handle.advertise<std_msgs::Float64>("left_wheel_shock_controller/command", 1, true);

  std_msgs::Float64 shock_set_point;
  shock_set_point.data = 0.0;
  right_wheel_shock_publisher.publish(shock_set_point);
  left_wheel_shock_publisher.publish(shock_set_point);

  double speed_P_left, speed_P_right, speed_D_left, speed_D_right, speed_I_left, speed_I_right;
  double rate_var;
  double max_effort = 0.0;

  assertions::param(pNh, "speed_P_left", speed_P_left, 5.0);
  assertions::param(pNh, "speed_P_right", speed_P_right, 5.0);
  assertions::param(pNh, "speed_D_left", speed_D_left, 1.0);
  assertions::param(pNh, "speed_D_right", speed_D_right, 1.0);
  assertions::param(pNh, "speed_I_left", speed_I_left, 0.0);
  assertions::param(pNh, "speed_I_right", speed_I_right, 0.0);
  assertions::param(pNh, "wheel_radius", wheel_radius, 0.3);
  assertions::param(pNh, "max_effort", max_effort, 4.0);
  assertions::param(pNh, "rate", rate_var, 60.0);

  // Publish that the robot is enabled
  ros::Publisher enabled_pub = handle.advertise<std_msgs::Bool>("/robot_enabled", 1, /* latch = */ true);
  std_msgs::Bool enabled_msg;
  enabled_msg.data = true;
  enabled_pub.publish(enabled_msg);

  ros::Subscriber speed_sub = handle.subscribe("/motors", 1, speedCallback);

  ros::Subscriber state_sub = handle.subscribe("/joint_states", 1, jointStateCallback);

  ros::Time prev, now;
  prev = ros::Time::now();

  ros::Rate rate{ rate_var };

  double speed_last_error_left = 0.0;
  double speed_last_error_right = 0.0;
  double speed_left_error_accum = 0.0;
  double speed_right_error_accum = 0.0;
  double effort_right = 0.0;
  double effort_left = 0.0;
  ros::Time prev_time = ros::Time::now();
  while (ros::ok())
  {
    ros::spinOnce();

    ros::Time cur_time = ros::Time::now();
    double dt = cur_time.toSec() - prev_time.toSec();
    prev_time = cur_time;

    double error_left = speed_set_point_left - speed_measured_left;
    double dError_left = (error_left - speed_last_error_left) / dt;
    speed_left_error_accum += error_left;
    speed_last_error_left = error_left;

    // ROS_INFO_STREAM("error left = " << error_left);
    // ROS_INFO_STREAM("effort_left = " << effort_left);

    effort_left += speed_P_left * error_left + speed_D_left * dError_left + speed_I_left * speed_left_error_accum;

    double error_right = speed_set_point_right - speed_measured_right;
    double dError_right = (error_right - speed_last_error_right) / dt;
    speed_right_error_accum += error_right;
    speed_last_error_right = error_right;

    effort_right +=
        speed_P_right * error_right + speed_D_right * dError_right + speed_I_right * speed_right_error_accum;

    effort_right = std::min(max_effort, std::max(-max_effort, effort_right));
    effort_left = std::min(max_effort, std::max(-max_effort, effort_left));

    // ROS_INFO_STREAM("Publishing effort: " << effort_right);
    // ROS_INFO_STREAM("left: " << effort_left << " right: " << effort_right);
    std_msgs::Float64 left_wheel_message;
    std_msgs::Float64 right_wheel_message;

    left_wheel_message.data = effort_left;
    right_wheel_message.data = effort_right;
    /*
    if(speed_set_point_left == 0) {
      effort_left = 0.0;
      left_wheel_message.data = 0.0;
    } else {
      left_wheel_message.data = effort_left;
    }
    if(speed_set_point_right == 0) {
      effort_right = 0;
      right_wheel_message.data = 0.0;
    } else {
      right_wheel_message.data = effort_right;
    }
    */

    left_wheel_effort_publisher.publish(left_wheel_message);
    right_wheel_effort_publisher.publish(right_wheel_message);
    igvc_msgs::velocity_pair speed_measured;
    speed_measured.left_velocity = speed_measured_left;
    speed_measured.right_velocity = speed_measured_right;
    now = ros::Time::now();
    ros::Duration duration = now - prev;
    speed_measured.duration = duration.toSec();
    speed_measured.header.stamp = ros::Time::now();
    wheel_speed_publisher.publish(speed_measured);
    rate.sleep();
    prev = now;
  }

  return 0;
}
