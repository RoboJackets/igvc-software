#ifndef PROJECT_JOYSTICK_DRIVER_H
#define PROJECT_JOYSTICK_DRIVER_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <igvc_msgs/velocity_pair.h>
#include <sensor_msgs/Imu.h>

enum ControlStyle { direction_velocity_control, smooth_control, tank_control };

struct SmoothControlConfig {
  double k1;
  double k2;
  double axle_length;
  double velocity;
  double w;
  double max_w;
  double delta;
  double camera;
  double camera_move_rate;
  bool camera_initialized;
};

struct TankControlConfig {
  double max_velocity;
  double velocity_increment;
  double velocity;
};

struct AccelerationLimits {
  double wheel_max_accel;
  double tangent_max_accel;
};

struct DirectionVelocityConfig {
  double pivot_limit;
  double max_velocity;
  double velocity_increment;
  double velocity;
};

struct Axes {
  double left_x;
  double left_y;
  double right_x;
  double right_y;
};

struct JoyMap {
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

class JoystickDriver {
 public:
  JoystickDriver();
 private:
  ros::Publisher cmd_pub_;

  double max_velocity_;
  double control_loop_period_;

  SmoothControlConfig smooth_control_config_;
  TankControlConfig tank_control_config_;
  DirectionVelocityConfig direction_velocity_config_;
  AccelerationLimits acceleration_limits_;
  JoyMap joy_map_;
  ControlStyle control_style_;

  Axes axes_;

  sensor_msgs::JoyConstPtr joystick_;
  sensor_msgs::ImuConstPtr imu_;
  bool a_clicked_ = false;
  bool rb_clicked_ = false;
  bool lb_clicked_ = false;

  igvc_msgs::velocity_pair motor_cmd_;
  igvc_msgs::velocity_pair last_motor_cmd_;

  ros::Time last_time_;

  void joystickCallback(const sensor_msgs::JoyConstPtr &joystick);
  void imuCallback(const sensor_msgs::ImuConstPtr &imu);
  void cursesLoop();
  void controlLoop(const ros::TimerEvent &);
  void processAxes(const sensor_msgs::JoyConstPtr &joystick);

  void boundAcceleration();
  void boundTangentAcceleration(double tangent_acceleration, const ros::Duration& dt);
  void boundLeftAcceleration(double left_acceleration, const ros::Duration& dt);
  void boundRightAcceleration(double right_acceleration, const ros::Duration& dt);

  void handleTankControl(const sensor_msgs::JoyConstPtr &joystick);
  void handleSmoothControl(const sensor_msgs::JoyConstPtr &joystick);
  void handleDirectionVelocityControl(const sensor_msgs::JoyConstPtr &joystick);

  double getVelocity(const igvc_msgs::velocity_pair& command) const;
  double getW(const igvc_msgs::velocity_pair& command) const;
  double getK(const igvc_msgs::velocity_pair& command) const;
  double getYaw(const sensor_msgs::ImuConstPtr &imu) const;

  void signalHandler();
};

#endif //PROJECT_JOYSTICK_DRIVER_H
