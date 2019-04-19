#include "joystick_driver.h"

#include <ncurses.h>
#include <mutex>
#include <thread>

#include <tf/LinearMath/Matrix3x3.h>
#include <csignal>
#include <igvc_utils/NodeUtils.hpp>

volatile std::sig_atomic_t quit;

double left_x, left_y, right_x, right_y;

joystick_Driver::joystick_Driver()
  : max_velocity_{}
  , smooth_control_config_{}
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

  std::thread GUI_thread(&joystick_Driver::cursesLoop, this);
  std::thread signal_handler(&joystick_Driver::signalHandler, this);

  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  cmd_pub_ = nh.advertise<igvc_msgs::velocity_pair>("/motors", 1);

  ros::Subscriber joy_sub = nh.subscribe("/joy", 1, &joystick_Driver::joystickCallback, this);

  igvc::param(pNh, "smooth_control/k1", smooth_control_config_.k1, 1.0);
  igvc::param(pNh, "smooth_control/k2", smooth_control_config_.k2, 10.0);
  igvc::param(pNh, "smooth_control/max_w", smooth_control_config_.max_w, 2.0);
  igvc::param(pNh, "smooth_control/axle_length", smooth_control_config_.axle_length, 0.52);
  igvc::param(pNh, "smooth_control/camera_move_rate", smooth_control_config_.camera_move_rate, 1.57);

  igvc::param(pNh, "tank_controls/velocity", tank_control_config_.max_velocity, 1.0);
  igvc::param(pNh, "direction_velocity_control/pivot_limit", direction_velocity_config_.pivot_limit, 0.32);
  igvc::param(pNh, "limits/velocity", max_velocity_, 1.0);
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

  // 10 is the sentinel value for uninitialized
  smooth_control_config_.camera = 10;

  ros::Subscriber imu_sub = nh.subscribe(imu_topic, 1, &joystick_Driver::imuCallback, this);

  ros::Timer control_loop = nh.createTimer(ros::Duration(control_loop_period_), &joystick_Driver::controlLoop, this);

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

void joystick_Driver::cursesLoop()
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
    else if (control_style_ == ControlStyle::smooth_control)
    {
      attron(COLOR_PAIR(3));
      mvprintw(0, 0, "Control Scheme:\t%s (Press A to change)", "Smooth Control");
      attroff(COLOR_PAIR(3));
      mvprintw(2, 0, "w:\t%.4f", smooth_control_config_.w);
      mvprintw(3, 0, "Current velocity:\t%.4f", smooth_control_config_.velocity);

      double y = getYaw(imu_);
      mvprintw(5, 0, "yaw: %.4f", y);
      mvprintw(6, 0, "camera: %.4f", smooth_control_config_.camera);
      mvprintw(7, 0, "delta: %.4f", smooth_control_config_.delta);
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

void joystick_Driver::controlLoop(const ros::TimerEvent &)
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
  else if (control_style_ == ControlStyle::smooth_control)
  {
    handleSmoothControl(joystick_);
  }
  // Toggle control mode
  if (joystick_->buttons[joy_map_.a])
  {
    a_clicked_ = true;
  }
  if (a_clicked_ && !joystick_->buttons[joy_map_.a])
  {
    if (control_style_ == ControlStyle::direction_velocity_control)
    {
      control_style_ = ControlStyle::smooth_control;
    }
    else if (control_style_ == ControlStyle::smooth_control)
    {
      control_style_ = ControlStyle::tank_control;
    }
    else if (control_style_ == ControlStyle::tank_control)
    {
      control_style_ = ControlStyle::direction_velocity_control;
    }
    a_clicked_ = false;
  }
  cmd_pub_.publish(motor_cmd_);
}

void joystick_Driver::joystickCallback(const sensor_msgs::JoyConstPtr &joystick)
{
  joystick_ = joystick;
}

void joystick_Driver::imuCallback(const sensor_msgs::ImuConstPtr &imu)
{
  imu_ = imu;
}

void joystick_Driver::processAxes(const sensor_msgs::JoyConstPtr &joystick)
{
  left_y = joystick->axes[joy_map_.left_axis_y];
  left_x = joystick->axes[joy_map_.left_axis_x];
  right_y = joystick->axes[joy_map_.right_axis_y];
  right_x = joystick->axes[joy_map_.right_axis_x];

  if (joy_map_.left_axis_x_invert)
  {
    left_x *= -1;
  }
  if (joy_map_.left_axis_y_invert)
  {
    left_y *= -1;
  }
  if (joy_map_.right_axis_x_invert)
  {
    right_x *= -1;
  }
  if (joy_map_.right_axis_y_invert)
  {
    right_y *= -1;
  }
}

void joystick_Driver::handleDirectionVelocityControl(const sensor_msgs::JoyConstPtr &joystick)
{
  processAxes(joystick);
  if (joystick->buttons[joy_map_.rb])
  {
    direction_velocity_config_.max_velocity += direction_velocity_config_.velocity_increment;
  }
  if (joystick->buttons[joy_map_.lb])
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
  if (left_y >= 0)
  {
    left = 1 + (left_x + right_x);
    right = 1 - (left_x + right_x);
  }
  else
  {
    left = 1 - (left_x + right_x);
    right = 1 + (left_x + right_x);
  }
  // Scale drive output due to throttle
  //  double exp_left_y = 0.8 * left_y * left_y * left_y + 0.2 * left_y;
  double exp_left_y = left_y;
  left = left * exp_left_y;
  right = right * exp_left_y;

  // Calculate pivot amount
  double pivot_speed = (left_x + right_x);
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

void joystick_Driver::handleTankControl(const sensor_msgs::JoyConstPtr &joystick)
{
  if (joystick->buttons[joy_map_.rb])
  {
    tank_control_config_.max_velocity += tank_control_config_.velocity_increment;
  }
  if (joystick->buttons[joy_map_.lb])
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

void joystick_Driver::handleSmoothControl(const sensor_msgs::JoyConstPtr &joystick)
{
  processAxes(joystick);

  if (imu_ == nullptr) {
    return;
  }

  if (smooth_control_config_.camera == 10) {
    smooth_control_config_.camera = getYaw(imu_);
  }

  // 90 degree anticlockwise turn
  if (!rb_clicked_ && joystick->buttons[joy_map_.rb]) {
    rb_clicked_ = true;
    smooth_control_config_.camera -= M_PI_2;
  }
  rb_clicked_ = joystick->buttons[joy_map_.rb];
  // 90 degree clockwise turn
  if (!lb_clicked_ && joystick->buttons[joy_map_.lb]) {
    lb_clicked_ = true;
    smooth_control_config_.camera += M_PI_2;
  }
  lb_clicked_ = joystick->buttons[joy_map_.lb];

  // Center camera
  if (joystick->buttons[joy_map_.b]) {
    smooth_control_config_.camera = getYaw(imu_);
  }

  // Move camera
  double camera_turn = right_x * right_x * right_x;
  double camera_turn_amount = camera_turn * smooth_control_config_.camera_move_rate * control_loop_period_;
  smooth_control_config_.camera -= camera_turn_amount; // Left hand to right hand
  igvc::fit_to_polar(smooth_control_config_.camera);

  // Sensitivity curve ^.^
  double expo_left_x = left_x * left_x * left_x;
  double expo_left_y = left_y * left_y * left_y;
  double velocity = std::hypot(expo_left_x, expo_left_y);
  double delta = -atan2(left_y, left_x);
  delta += M_PI_2;
  delta -= smooth_control_config_.camera;

  delta += getYaw(imu_);

  igvc::fit_to_polar(delta);

  double w = -velocity * smooth_control_config_.k2 * delta + (1 + smooth_control_config_.k1) * sin(delta);
  if (velocity < 0.1)
  {
    w = 0;
  }
  w = std::min(w, smooth_control_config_.max_w);
  w = std::max(w, -smooth_control_config_.max_w);
  double v_left = (2 * velocity - w * smooth_control_config_.axle_length) / 2;
  double v_right = (2 * velocity + w * smooth_control_config_.axle_length) / 2;

  //  double max = max(fabs(v_left), fabs(v_right));

  smooth_control_config_.delta = delta;
  smooth_control_config_.w = w;
  smooth_control_config_.velocity = velocity;
  motor_cmd_ = igvc_msgs::velocity_pair{};
  motor_cmd_.header.stamp = ros::Time::now();
  motor_cmd_.left_velocity = v_left;
  motor_cmd_.right_velocity = v_right;
}

void joystick_Driver::signalHandler()
{
  sigset_t sig;
  sigemptyset(&sig);
  sigaddset(&sig, SIGINT);

  int caught_sig;
  sigwait(&sig, &caught_sig);
  quit = 1;
}

double joystick_Driver::getYaw(const sensor_msgs::ImuConstPtr &imu) {
  tf::Quaternion quat{ imu->orientation.x, imu->orientation.y, imu->orientation.z, imu->orientation.w };
  double r, p, y;
  tf::Matrix3x3(quat).getRPY(r, p, y);
  return y;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystick_driver");
  joystick_Driver joystick_driver;
  return 0;
}
