#include "joystick_driver.h"

#include <ncurses.h>
#include <mutex>
#include <thread>

#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>
#include <csignal>
#include <igvc_utils/NodeUtils.hpp>

volatile std::sig_atomic_t quit;

JoystickDriver::JoystickDriver()
  : max_velocity_{}
  , tank_control_config_{ 0, 0.001 }
  , direction_velocity_config_{ 0.32, 1, 0.001, 0 }
  , joy_map_{}
  , control_style_{ ControlStyle::direction_velocity_control }
  , motor_cmd_{}
{
  sigset_t sig;
  sigemptyset(&sig);
  sigaddset(&sig, SIGINT);
  sigprocmask(SIG_BLOCK, &sig, nullptr);

  std::thread GUI_thread(&JoystickDriver::cursesLoop, this);
  std::thread signal_handler(&JoystickDriver::signalHandler, this);

  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  cmd_pub_ = nh.advertise<igvc_msgs::velocity_pair>("/motors", 1);

  ros::Subscriber joy_sub = nh.subscribe("/joy", 1, &JoystickDriver::joystickCallback, this);

  igvc::param(pNh, "tank_controls/velocity", tank_control_config_.max_velocity, 1.0);
  igvc::param(pNh, "direction_velocity_control/pivot_limit", direction_velocity_config_.pivot_limit, 0.32);
  igvc::param(pNh, "limits/velocity", max_velocity_, 1.0);

  igvc::param(pNh, "limits/acceleration/wheel", acceleration_limits_.wheel_max_accel, 5.0);
  igvc::param(pNh, "limits/acceleration/tangent", acceleration_limits_.tangent_max_accel, 3.00);
  igvc::param(pNh, "limits/acceleration/enable", acceleration_limits_.enable, false);

  igvc::param(pNh, "controller/left_axis/horizontal", joy_map_.left_axis_x, 0);
  igvc::param(pNh, "controller/left_axis/horizontal_invert", joy_map_.left_axis_x_invert, true);

  igvc::param(pNh, "controller/left_axis/vertical", joy_map_.left_axis_y, 1);
  igvc::param(pNh, "controller/left_axis/vertical_invert", joy_map_.left_axis_y_invert, false);

  igvc::param(pNh, "controller/right_axis/horizontal", joy_map_.right_axis_x, 3);
  igvc::param(pNh, "controller/right_axis/horizontal_invert", joy_map_.right_axis_x_invert, true);

  igvc::param(pNh, "controller/right_axis/vertical", joy_map_.right_axis_y, 4);
  igvc::param(pNh, "controller/right_axis/vertical_invert", joy_map_.right_axis_y_invert, false);
  igvc::param(pNh, "period", control_loop_period_, 0.001);

  std::string imu_topic;
  igvc::param(pNh, "topics/imu", imu_topic, std::string{ "/imu" });

  ros::Timer control_loop = nh.createTimer(ros::Duration(control_loop_period_), &JoystickDriver::controlLoop, this);

  while (ros::ok())
  {
    ros::spinOnce();
    if (quit)
    {
      ROS_INFO("Shutting down GUI thread");
      GUI_thread.join();
      signal_handler.join();
      ros::shutdown();
    }
  }
}

void JoystickDriver::cursesLoop()
{
  std::this_thread::sleep_for(std::chrono::seconds(1));
  ROS_INFO("Starting ncurses GUI...");
  std::this_thread::sleep_for(std::chrono::seconds(1));

  initscr();
  curs_set(0);
  use_default_colors();
  start_color();
  init_pair(1, COLOR_CYAN, -1);
  init_pair(2, COLOR_GREEN, -1);
  init_pair(3, COLOR_RED, -1);
  refresh();

  while (true)
  {
    erase();
    mvprintw(1, 0, "Left: %.4f", motor_cmd_.left_velocity);
    mvprintw(1, 16, "Right: %.4f", motor_cmd_.right_velocity);
    if (control_style_ == ControlStyle::direction_velocity_control)
    {
      attron(COLOR_PAIR(1));
      mvprintw(0, 0, "Control Scheme:\t%s (Press A to change)", "Direction Velocity Control");
      attroff(COLOR_PAIR(1));
      mvprintw(2, 0, "Max velocity:\t%.4f", direction_velocity_config_.max_velocity);
      mvprintw(3, 0, "Current velocity:\t%.4f", direction_velocity_config_.velocity);
    }
    else if (control_style_ == ControlStyle::tank_control)
    {
      attron(COLOR_PAIR(2));
      mvprintw(0, 0, "Control Scheme:\t%s (Press A to change)", "Tank Control");
      attroff(COLOR_PAIR(2));
      mvprintw(2, 0, "Max velocity:\t%.4f", tank_control_config_.max_velocity);
      mvprintw(3, 0, "Current velocity:\t%.4f", tank_control_config_.velocity);
    }

    if (joystick_ != nullptr)
    {
      mvprintw(4, 0, "Joystick L_Y:\t%.4f", joystick_->axes[joy_map_.left_axis_y]);
    }
    refresh();
    if (quit)
    {
      endwin();
      break;
    }
  }
}

void JoystickDriver::controlLoop(const ros::TimerEvent &)
{
  if (joystick_ == nullptr)
  {
    return;
  }
  if (control_style_ == ControlStyle::direction_velocity_control)
  {
    handleDirectionVelocityControl(joystick_);
  }
  else if (control_style_ == ControlStyle::tank_control)
  {
    handleTankControl(joystick_);
  }

  // Toggle control mode
  if (joystick_->buttons[joy_map_.a_button])
  {
    a_clicked_ = true;
  }
  if (a_clicked_ && !joystick_->buttons[joy_map_.a_button])
  {
    if (control_style_ == ControlStyle::direction_velocity_control)
    {
      control_style_ = ControlStyle::tank_control;
    }
    else if (control_style_ == ControlStyle::tank_control)
    {
      control_style_ = ControlStyle::direction_velocity_control;
    }
    a_clicked_ = false;
  }
  if (acceleration_limits_.enable)
  {
    boundAcceleration();
  }

  last_time_ = ros::Time::now();
  last_motor_cmd_ = motor_cmd_;

  cmd_pub_.publish(motor_cmd_);
}

void JoystickDriver::boundAcceleration()
{
  ros::Duration dt = motor_cmd_.header.stamp - last_time_;
  if (dt.toSec() == 0)
  {
    dt = ros::Duration(0.001);
  }

  double dv = getVelocity(motor_cmd_) - getVelocity(last_motor_cmd_);

  double a = dv / dt.toSec();
  if (std::abs(a) > std::abs(acceleration_limits_.tangent_max_accel))
  {
    boundTangentAcceleration(a, dt);
  }
}

void JoystickDriver::joystickCallback(const sensor_msgs::JoyConstPtr &joystick)
{
  joystick_ = joystick;
}

void JoystickDriver::processAxes(const sensor_msgs::JoyConstPtr &joystick)
{
  axes_.left_y = joystick->axes[joy_map_.left_axis_y];
  axes_.left_x = joystick->axes[joy_map_.left_axis_x];
  axes_.right_y = joystick->axes[joy_map_.right_axis_y];
  axes_.right_x = joystick->axes[joy_map_.right_axis_x];

  if (joy_map_.left_axis_x_invert)
  {
    axes_.left_x *= -1;
  }
  if (joy_map_.left_axis_y_invert)
  {
    axes_.left_y *= -1;
  }
  if (joy_map_.right_axis_x_invert)
  {
    axes_.right_x *= -1;
  }
  if (joy_map_.right_axis_y_invert)
  {
    axes_.right_y *= -1;
  }
}

void JoystickDriver::handleDirectionVelocityControl(const sensor_msgs::JoyConstPtr &joystick)
{
  processAxes(joystick);
  if (joystick->buttons[joy_map_.rb_button])
  {
    direction_velocity_config_.max_velocity += direction_velocity_config_.velocity_increment;
  }
  if (joystick->buttons[joy_map_.lb_button])
  {
    direction_velocity_config_.max_velocity -= direction_velocity_config_.velocity_increment;
  }
  if (direction_velocity_config_.max_velocity > max_velocity_)
  {
    direction_velocity_config_.max_velocity = max_velocity_;
  }
  if (direction_velocity_config_.max_velocity <= 0)
  {
    tank_control_config_.max_velocity = 0;
  }

  double left;
  double right;
  if (axes_.left_y >= 0)
  {
    left = 1 + (axes_.left_x + axes_.right_x);
    right = 1 - (axes_.left_x + axes_.right_x);
  }
  else
  {
    left = 1 - (axes_.left_x + axes_.right_x);
    right = 1 + (axes_.left_x + axes_.right_x);
  }
  // Scale drive output due to throttle
  //  double exp_left_y = 0.8 * left_y * left_y * left_y + 0.2 * left_y;
  double exp_left_y = axes_.left_y;
  left = left * exp_left_y;
  right = right * exp_left_y;

  // Calculate pivot amount
  double pivot_speed = (axes_.left_x + axes_.right_x);
  double pivot_limit = direction_velocity_config_.pivot_limit;
  double pivot_scale = fabs(exp_left_y) > pivot_limit ? 0 : (1 - fabs(exp_left_y) / pivot_limit);

  // Calculate final mix of drive and pivot
  left = (1 - pivot_scale) * left + pivot_scale * pivot_speed;
  right = (1 - pivot_scale) * right + pivot_scale * -pivot_speed;

  left *= direction_velocity_config_.max_velocity;
  right *= direction_velocity_config_.max_velocity;

  direction_velocity_config_.velocity = (left + right) / 2;

  motor_cmd_ = igvc_msgs::velocity_pair{};
  motor_cmd_.header.stamp = ros::Time::now();
  motor_cmd_.left_velocity = left;
  motor_cmd_.right_velocity = right;
}

void JoystickDriver::handleTankControl(const sensor_msgs::JoyConstPtr &joystick)
{
  if (joystick->buttons[joy_map_.rb_button])
  {
    tank_control_config_.max_velocity += tank_control_config_.velocity_increment;
  }
  if (joystick->buttons[joy_map_.lb_button])
  {
    tank_control_config_.max_velocity -= tank_control_config_.velocity_increment;
  }
  if (tank_control_config_.max_velocity > max_velocity_)
  {
    tank_control_config_.max_velocity = max_velocity_;
  }
  if (tank_control_config_.max_velocity <= 0)
  {
    tank_control_config_.max_velocity = 0;
  }

  double left = joystick->axes[joy_map_.left_axis_y] * tank_control_config_.max_velocity;
  double right = joystick->axes[joy_map_.right_axis_y] * tank_control_config_.max_velocity;

  tank_control_config_.velocity = std::hypot(left, right);

  motor_cmd_ = igvc_msgs::velocity_pair{};
  motor_cmd_.header.stamp = ros::Time::now();
  motor_cmd_.left_velocity = left;
  motor_cmd_.right_velocity = right;
}

void JoystickDriver::signalHandler()
{
  sigset_t sig;
  sigemptyset(&sig);
  sigaddset(&sig, SIGINT);

  int caught_sig;
  sigwait(&sig, &caught_sig);
  quit = 1;
}

void JoystickDriver::boundTangentAcceleration(double tangent_acceleration, const ros::Duration &dt)
{
  double bound_a = copysign(acceleration_limits_.tangent_max_accel, tangent_acceleration);
  double v = getVelocity(last_motor_cmd_) + bound_a * dt.toSec();

  double v_l;
  double v_r;
  bool stopping = std::abs(motor_cmd_.left_velocity) < 0.15 && std::abs(motor_cmd_.right_velocity) < 0.15;
  bool large_a = std::abs(tangent_acceleration) > 50.0f;
  if (stopping || large_a)
  {
    v_l = v;
    v_r = v;
    motor_cmd_.left_velocity = v_l;
    motor_cmd_.right_velocity = v_r;
  }
}

double JoystickDriver::getVelocity(const igvc_msgs::velocity_pair &command) const
{
  return (command.left_velocity + command.right_velocity) / 2;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joystick_driver");
  JoystickDriver joystick_driver;
  return 0;
}
