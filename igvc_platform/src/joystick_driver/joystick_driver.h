#ifndef PROJECT_JOYSTICK_DRIVER_H
#define PROJECT_JOYSTICK_DRIVER_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <igvc_msgs/velocity_pair.h>
#include <sensor_msgs/Imu.h>

enum Control_style { direction_velocity_control, smooth_control, tank_control };

struct Smooth_control_config {
  double k1;
  double k2;
  double axle_length;
  double velocity;
  double w;
  double max_w;
  double delta;
  double camera;
  double camera_move_rate;
};

struct Tank_control_config {
  double max_velocity;
  double velocity_increment;
  double velocity;
};

struct Direction_velocity_config {
  double pivot_limit;
  double max_velocity;
  double velocity_increment;
  double velocity;
};

struct Joy_map {
  int a = 0;
  int b = 1;
  int x = 2;
  int y = 3;
  int lb = 4;
  int rb = 5;
  int left_axis_x = 0;
  int left_axis_y = 1;
  int right_axis_x = 3;
  int right_axis_y = 4;
  bool left_axis_x_invert;
  bool left_axis_y_invert;
  bool right_axis_x_invert;
  bool right_axis_y_invert;
};

class joystick_Driver {
 public:
  joystick_Driver();
 private:
  ros::Publisher cmd_pub;
  double max_velocity;
  double control_loop_period;
  Smooth_control_config smooth_control_config;
  Tank_control_config tank_control_config;
  Direction_velocity_config direction_velocity_config;
  Joy_map joy_map;
  Control_style control_style;
  sensor_msgs::JoyConstPtr joystick;
  sensor_msgs::ImuConstPtr imu;
  bool a_clicked = false;
  bool rb_clicked = false;
  bool lb_clicked = false;

  igvc_msgs::velocity_pair motor_cmd;

  void joystick_callback(const sensor_msgs::JoyConstPtr &joystick);
  void imu_callback(const sensor_msgs::ImuConstPtr &imu);
  void curses_loop();
  void control_loop(const ros::TimerEvent&);
  void process_axes(const sensor_msgs::JoyConstPtr &joystick);
  void handle_tank_control(const sensor_msgs::JoyConstPtr &joystick);
  void handle_smooth_control(const sensor_msgs::JoyConstPtr &joystick);
  void handle_direction_velocity_control(const sensor_msgs::JoyConstPtr &joystick);
  double get_yaw(const sensor_msgs::ImuConstPtr& imu);
  void signal_handler();
};

#endif //PROJECT_JOYSTICK_DRIVER_H
