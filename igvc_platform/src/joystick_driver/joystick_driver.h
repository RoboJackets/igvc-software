#ifndef PROJECT_JOYSTICK_DRIVER_H
#define PROJECT_JOYSTICK_DRIVER_H

#include <igvc_msgs/velocity_pair.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>

enum ControlStyle
{
  direction_velocity_control,
  tank_control
};

struct TankControlConfig
{
  double max_velocity;
  double velocity_increment;
  double velocity;
};

struct AccelerationLimits
{
  double wheel_max_accel;
  double tangent_max_accel;
  bool enable;
};

struct DirectionVelocityConfig
{
  double pivot_limit;
  double max_velocity;
  double velocity_increment;
  double velocity;
};

struct Axes
{
  double left_x;
  double left_y;
  double right_x;
  double right_y;
};

struct JoyMap
{
  int a_button = 0;
  int b_button = 1;
  int x_button = 2;
  int y_button = 3;
  int lb_button = 4;
  int rb_button = 5;
  int left_axis_x = 0;
  int left_axis_y = 1;
  int right_axis_x = 3;
  int right_axis_y = 4;
  bool left_axis_x_invert;
  bool left_axis_y_invert;
  bool right_axis_x_invert;
  bool right_axis_y_invert;
};

class JoystickDriver
{
public:
  JoystickDriver();

private:
  ros::Publisher cmd_pub_;

  double max_velocity_;
  double control_loop_period_{};

  TankControlConfig tank_control_config_;
  DirectionVelocityConfig direction_velocity_config_;
  AccelerationLimits acceleration_limits_{};
  JoyMap joy_map_;
  ControlStyle control_style_;

  Axes axes_{};

  sensor_msgs::JoyConstPtr joystick_;
  bool a_clicked_ = false;

  igvc_msgs::velocity_pair motor_cmd_;
  igvc_msgs::velocity_pair last_motor_cmd_;

  ros::Time last_time_;

  void joystickCallback(const sensor_msgs::JoyConstPtr &joystick);
  void imuCallback(const sensor_msgs::ImuConstPtr &imu);
  void cursesLoop();
  void controlLoop(const ros::TimerEvent &);
  void processAxes(const sensor_msgs::JoyConstPtr &joystick);

  void boundAcceleration();
  void boundTangentAcceleration(double tangent_acceleration, const ros::Duration &dt);

  void handleTankControl(const sensor_msgs::JoyConstPtr &joystick);
  void handleDirectionVelocityControl(const sensor_msgs::JoyConstPtr &joystick);

  [[nodiscard]] double getVelocity(const igvc_msgs::velocity_pair &command) const;

  void signalHandler();
};

#endif  // PROJECT_JOYSTICK_DRIVER_H
