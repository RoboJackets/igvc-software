#include <parameter_assertions/assertions.h>
#include <ros/ros.h>

#include "swerve_control.h"

SwerveControl::SwerveControl() : pNh{ "~" }
{
  // motor pubs
  effortPublishers = {
    handle.advertise<std_msgs::Float64>("/front_left_effort_controller/command", 1),   // front_left_effort_publisher
    handle.advertise<std_msgs::Float64>("/front_right_effort_controller/command", 1),  // front_right_effort_publisher
    handle.advertise<std_msgs::Float64>("/back_left_effort_controller/command", 1),    // back_left_effort_publisher
    handle.advertise<std_msgs::Float64>("/back_right_effort_controller/command", 1),   // back_right_effort_publisher
    handle.advertise<std_msgs::Float64>("/front_left_swivel_controller/command", 1),   // front_left_swivel_publisher
    handle.advertise<std_msgs::Float64>("/front_right_swivel_controller/command", 1),  // front_right_swivel_publisher
    handle.advertise<std_msgs::Float64>("/back_left_swivel_controller/command", 1),    // back_left_swivel_publisher
    handle.advertise<std_msgs::Float64>("/back_right_swivel_controller/command", 1)    // back_right_swivel_publisher
  };

  // shock pubs
  back_left_shock_publisher = handle.advertise<std_msgs::Float64>("/back_left_shock_controller/command", 1, true);
  back_right_shock_publisher = handle.advertise<std_msgs::Float64>("/back_right_shock_controller/command", 1, true);
  front_left_shock_publisher = handle.advertise<std_msgs::Float64>("/front_left_shock_controller/command", 1, true);
  front_right_shock_publisher = handle.advertise<std_msgs::Float64>("/front_right_shock_controller/command", 1, true);

  wheel_speed_publisher = handle.advertise<igvc_msgs::velocity_quad>("/encoders", 1);

  // Shocks for all modules
  std_msgs::Float64 shock_set_point;
  shock_set_point.data = 0.0;
  back_left_shock_publisher.publish(shock_set_point);
  back_right_shock_publisher.publish(shock_set_point);
  front_left_shock_publisher.publish(shock_set_point);
  front_right_shock_publisher.publish(shock_set_point);

  assertions::param(pNh, "speed_P_fl", speed_P_fl, 5.0);
  assertions::param(pNh, "speed_P_fr", speed_P_fr, 5.0);
  assertions::param(pNh, "speed_P_bl", speed_P_bl, 5.0);
  assertions::param(pNh, "speed_P_br", speed_P_br, 5.0);
  assertions::param(pNh, "speed_I_fl", speed_I_fl, 0.0);
  assertions::param(pNh, "speed_I_fr", speed_I_fr, 0.0);
  assertions::param(pNh, "speed_I_bl", speed_I_bl, 0.0);
  assertions::param(pNh, "speed_I_br", speed_I_br, 0.0);
  assertions::param(pNh, "speed_D_fl", speed_D_fl, 1.0);
  assertions::param(pNh, "speed_D_fr", speed_D_fr, 1.0);
  assertions::param(pNh, "speed_D_bl", speed_D_bl, 1.0);
  assertions::param(pNh, "speed_D_br", speed_D_br, 1.0);

  assertions::param(pNh, "wheel_radius", wheel_radius, 0.1375);  // found in swervi_prop urdf
  assertions::param(pNh, "max_effort", max_effort, 4.0);
  assertions::param(pNh, "rate", rate_var, 60.0);

  assertions::param(pNh, "alpha", alpha, 0.5);

  // joint names
  joint_names = { "fl_wheel_axle", "fr_wheel_axle", "bl_wheel_axle", "br_wheel_axle",
                  "fl_swivel_rev", "fr_swivel_rev", "bl_swivel_rev", "br_swivel_rev" };

  // Publish that the robot is enabled
  enabled_pub = handle.advertise<std_msgs::Bool>("/robot_enabled", 1, /*latch = */ true);
  std_msgs::Bool enabled_msg;
  enabled_msg.data = true;
  enabled_pub.publish(enabled_msg);

  // Publishers for control loops
  speed_sub = handle.subscribe("/motors", 1, &SwerveControl::speedCallback, this);
  state_sub = handle.subscribe("/joint_states", 1, &SwerveControl::jointStateCallback, this);

  motors = {
    // set_point, measured, last_error, error_accum, effort, P, I, D
    { 0.0, 0.0, 0.0, 0.0, 0.0, speed_P_fl, speed_I_fl, speed_D_fl },  // speed_fl
    { 0.0, 0.0, 0.0, 0.0, 0.0, speed_P_fr, speed_I_fr, speed_D_fr },  // speed_fr
    { 0.0, 0.0, 0.0, 0.0, 0.0, speed_P_bl, speed_I_bl, speed_D_bl },  // speed_bl
    { 0.0, 0.0, 0.0, 0.0, 0.0, speed_P_br, speed_I_br, speed_D_br },  // speed_br
    // Swivel infos
    { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },  // swivel_fl
    { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },  // swivel_fr
    { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },  // swivel_bl
    { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }   // swivel_br
  };

  SwerveControl::ControlLoop();
}

void SwerveControl::ControlLoop()
{
  // Variables for control loop
  prev = ros::Time::now();

  ros::Rate rate{ rate_var };
  ros::Time prev_time = ros::Time::now();

  while (ros::ok())
  {
    ros::spinOnce();

    ros::Time cur_time = ros::Time::now();
    double dt = cur_time.toSec() - prev_time.toSec();
    prev_time = cur_time;

    // motors
    size_t motors_end = 4;
    for (size_t i = 0; i < motors_end; ++i)
    {
      double error = motors[i].set_point - motors[i].measured;
      double filtered_error = alpha * error + (1 - alpha) * motors[i].last_error;
      double dError = (dt == 0) ? 0 : (filtered_error - motors[i].last_error) / dt;

      motors[i].error_accum += error * dt;
      motors[i].last_error = filtered_error;
      motors[i].effort = motors[i].P * error + motors[i].D * dError + motors[i].I * motors[i].error_accum;
      motors[i].effort = std::min(max_effort, std::max(-max_effort, motors[i].effort));
      std_msgs::Float64 effort_msg;
      effort_msg.data = motors[i].effort;
      effortPublishers[i].publish(effort_msg);
    }

    // swivels
    size_t swivels_end = 8;
    for (size_t i = motors_end; i < swivels_end; ++i)
    {
      std_msgs::Float64 position_msg;
      position_msg.data = motors[i].set_point;
      effortPublishers[i].publish(position_msg);
    }

    // contructing encoder message
    igvc_msgs::velocity_quad speed_measured;
    speed_measured.fl_velocity = motors[0].measured;
    speed_measured.fr_velocity = motors[1].measured;
    speed_measured.bl_velocity = motors[2].measured;
    speed_measured.br_velocity = motors[3].measured;
    speed_measured.fl_angle = motors[4].measured;
    speed_measured.fr_angle = motors[5].measured;
    speed_measured.bl_angle = motors[6].measured;
    speed_measured.br_angle = motors[7].measured;

    now = ros::Time::now();
    ros::Duration duration = now - prev;
    speed_measured.duration = duration.toSec();
    speed_measured.header.stamp = ros::Time::now();
    wheel_speed_publisher.publish(speed_measured);
    rate.sleep();
    prev = now;
  }
}

void SwerveControl::speedCallback(const igvc_msgs::velocity_quad::ConstPtr &msg)
{
  if (msg->fl_velocity == msg->fl_velocity)
  {
    motors[0].set_point = msg->fl_velocity;
  }
  if (msg->fr_velocity == msg->fr_velocity)
  {
    motors[1].set_point = msg->fr_velocity;
  }
  if (msg->bl_velocity == msg->bl_velocity)
  {
    motors[2].set_point = msg->bl_velocity;
  }
  if (msg->br_velocity == msg->br_velocity)
  {
    motors[3].set_point = msg->br_velocity;
  }
  if (msg->fl_angle == msg->fl_angle)
  {
    motors[4].set_point = msg->fl_angle;
  }
  if (msg->fr_angle == msg->fr_angle)
  {
    motors[5].set_point = msg->fr_angle;
  }
  if (msg->bl_angle == msg->bl_angle)
  {
    motors[6].set_point = msg->bl_angle;
  }
  if (msg->br_angle == msg->br_angle)
  {
    motors[7].set_point = msg->br_angle;
  }
}

void SwerveControl::jointStateCallback(const sensor_msgs::JointStateConstPtr &msg)
{
  // motors
  size_t motors_end = 4;
  for (size_t i = 0; i < motors_end; ++i)
  {
    auto iter = std::find(msg->name.begin(), msg->name.end(), joint_names[i]);

    if (iter != msg->name.end())
    {
      auto index = std::distance(msg->name.begin(), iter);
      motors[i].measured = msg->velocity[index] * wheel_radius;
    }
  }

  // swivels
  size_t swivels_end = 8;
  for (size_t i = motors_end; i < swivels_end; ++i)
  {
    auto iter = std::find(msg->name.begin(), msg->name.end(), joint_names[i]);

    if (iter != msg->name.end())
    {
      auto index = std::distance(msg->name.begin(), iter);
      motors[i].measured = msg->position[index];
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swervi_controller");
  SwerveControl swerve_control;
  ros::spin();
}