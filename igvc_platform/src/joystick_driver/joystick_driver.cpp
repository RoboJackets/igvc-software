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
  : max_velocity{}
  , smooth_control_config{}
  , tank_control_config{ 0, 0.001 }
  , direction_velocity_config{ 0.32, 1, 0.001, 0 }
  , joy_map{}
  , control_style{ Control_style::direction_velocity_control }
  , motor_cmd{}
{
  sigset_t sig;
  sigemptyset(&sig);
  sigaddset(&sig, SIGINT);
  sigprocmask(SIG_BLOCK, &sig, nullptr);

  std::thread GUI_thread(&joystick_Driver::curses_loop, this);
  std::thread signal_handler(&joystick_Driver::signal_handler, this);

  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  cmd_pub = nh.advertise<igvc_msgs::velocity_pair>("/motors", 1);

  ros::Subscriber joy_sub = nh.subscribe("/joy", 1, &joystick_Driver::joystick_callback, this);

  igvc::param(pNh, "smooth_control/k1", smooth_control_config.k1, 1.0);
  igvc::param(pNh, "smooth_control/k2", smooth_control_config.k2, 10.0);
  igvc::param(pNh, "smooth_control/max_w", smooth_control_config.max_w, 2.0);
  igvc::param(pNh, "smooth_control/axle_length", smooth_control_config.axle_length, 0.52);
  igvc::param(pNh, "smooth_control/camera_move_rate", smooth_control_config.camera_move_rate, 1.57);

  igvc::param(pNh, "tank_controls/velocity", tank_control_config.max_velocity, 1.0);
  igvc::param(pNh, "direction_velocity_control/pivot_limit", direction_velocity_config.pivot_limit, 0.32);
  igvc::param(pNh, "limits/velocity", max_velocity, 1.0);
  igvc::param(pNh, "controller/left_axis/horizontal", joy_map.left_axis_x, 0);
  igvc::param(pNh, "controller/left_axis/horizontal_invert", joy_map.left_axis_x_invert, true);

  igvc::param(pNh, "controller/left_axis/vertical", joy_map.left_axis_y, 1);
  igvc::param(pNh, "controller/left_axis/vertical_invert", joy_map.left_axis_y_invert, false);

  igvc::param(pNh, "controller/right_axis/horizontal", joy_map.right_axis_x, 3);
  igvc::param(pNh, "controller/right_axis/horizontal_invert", joy_map.right_axis_x_invert, true);

  igvc::param(pNh, "controller/right_axis/vertical", joy_map.right_axis_y, 4);
  igvc::param(pNh, "controller/right_axis/vertical_invert", joy_map.right_axis_y_invert, false);
  igvc::param(pNh, "period", control_loop_period, 0.001);

  std::string imu_topic;
  igvc::param(pNh, "topics/imu", imu_topic, std::string{ "/imu" });

  // 10 is the sentinel value for uninitialized
  smooth_control_config.camera = 10;

  ros::Subscriber imu_sub = nh.subscribe(imu_topic, 1, &joystick_Driver::imu_callback, this);

  ros::Timer control_loop = nh.createTimer(ros::Duration(control_loop_period), &joystick_Driver::control_loop, this);

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

void joystick_Driver::curses_loop()
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
    mvprintw(1, 0, "Left: %.4f", motor_cmd.left_velocity);
    mvprintw(1, 16, "Right: %.4f", motor_cmd.right_velocity);
    if (control_style == Control_style::direction_velocity_control)
    {
      attron(COLOR_PAIR(1));
      mvprintw(0, 0, "Control Scheme:\t%s (Press A to change)", "Direction Velocity Control");
      attroff(COLOR_PAIR(1));
      mvprintw(2, 0, "Max velocity:\t%.4f", direction_velocity_config.max_velocity);
      mvprintw(3, 0, "Current velocity:\t%.4f", direction_velocity_config.velocity);
    }
    else if (control_style == Control_style::tank_control)
    {
      attron(COLOR_PAIR(2));
      mvprintw(0, 0, "Control Scheme:\t%s (Press A to change)", "Tank Control");
      attroff(COLOR_PAIR(2));
      mvprintw(2, 0, "Max velocity:\t%.4f", tank_control_config.max_velocity);
      mvprintw(3, 0, "Current velocity:\t%.4f", tank_control_config.velocity);
    }
    else if (control_style == Control_style::smooth_control)
    {
      attron(COLOR_PAIR(3));
      mvprintw(0, 0, "Control Scheme:\t%s (Press A to change)", "Smooth Control");
      attroff(COLOR_PAIR(3));
      mvprintw(2, 0, "w:\t%.4f", smooth_control_config.w);
      mvprintw(3, 0, "Current velocity:\t%.4f", smooth_control_config.velocity);

      double y = get_yaw(imu);
      mvprintw(5, 0, "yaw: %.4f", y);
      mvprintw(6, 0, "camera: %.4f", smooth_control_config.camera);
      mvprintw(7, 0, "delta: %.4f", smooth_control_config.delta);
    }
    if (joystick != nullptr)
    {
      mvprintw(4, 0, "Joystick L_Y:\t%.4f", joystick->axes[joy_map.left_axis_y]);
    }
    refresh();
    if (quit)
    {
      endwin();
      break;
    }
  }
}

void joystick_Driver::control_loop(const ros::TimerEvent&)
{
  if (joystick == nullptr)
  {
    return;
  }
  if (control_style == Control_style::direction_velocity_control)
  {
    handle_direction_velocity_control(joystick);
  }
  else if (control_style == Control_style::tank_control)
  {
    handle_tank_control(joystick);
  }
  else if (control_style == Control_style::smooth_control)
  {
    handle_smooth_control(joystick);
  }
  // Toggle control mode
  if (joystick->buttons[joy_map.a])
  {
    a_clicked = true;
  }
  if (a_clicked && !joystick->buttons[joy_map.a])
  {
    if (control_style == Control_style::direction_velocity_control)
    {
      control_style = Control_style::smooth_control;
    }
    else if (control_style == Control_style::smooth_control)
    {
      control_style = Control_style::tank_control;
    }
    else if (control_style == Control_style::tank_control)
    {
      control_style = Control_style::direction_velocity_control;
    }
    a_clicked = false;
  }
  cmd_pub.publish(motor_cmd);
}

void joystick_Driver::joystick_callback(const sensor_msgs::JoyConstPtr& msg)
{
  joystick = msg;
}

void joystick_Driver::imu_callback(const sensor_msgs::ImuConstPtr& msg)
{
  imu = msg;
}

void joystick_Driver::process_axes(const sensor_msgs::JoyConstPtr& joystick)
{
  left_y = joystick->axes[joy_map.left_axis_y];
  left_x = joystick->axes[joy_map.left_axis_x];
  right_y = joystick->axes[joy_map.right_axis_y];
  right_x = joystick->axes[joy_map.right_axis_x];

  if (joy_map.left_axis_x_invert)
  {
    left_x *= -1;
  }
  if (joy_map.left_axis_y_invert)
  {
    left_y *= -1;
  }
  if (joy_map.right_axis_x_invert)
  {
    right_x *= -1;
  }
  if (joy_map.right_axis_y_invert)
  {
    right_y *= -1;
  }
}

void joystick_Driver::handle_direction_velocity_control(const sensor_msgs::JoyConstPtr& joystick)
{
  process_axes(joystick);
  if (joystick->buttons[joy_map.rb])
  {
    direction_velocity_config.max_velocity += direction_velocity_config.velocity_increment;
  }
  if (joystick->buttons[joy_map.lb])
  {
    direction_velocity_config.max_velocity -= direction_velocity_config.velocity_increment;
  }
  if (direction_velocity_config.max_velocity > max_velocity)
  {
    direction_velocity_config.max_velocity = max_velocity;
  }
  if (direction_velocity_config.max_velocity <= 0)
  {
    tank_control_config.max_velocity = 0;
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
  double pivot_limit = direction_velocity_config.pivot_limit;
  double pivot_scale = fabs(exp_left_y) > pivot_limit ? 0 : (1 - fabs(exp_left_y) / pivot_limit);

  // Calculate final mix of drive and pivot
  left = (1 - pivot_scale) * left + pivot_scale * pivot_speed;
  right = (1 - pivot_scale) * right + pivot_scale * -pivot_speed;

  left *= direction_velocity_config.max_velocity;
  right *= direction_velocity_config.max_velocity;

  direction_velocity_config.velocity = (left + right) / 2;

  motor_cmd = igvc_msgs::velocity_pair{};
  motor_cmd.header.stamp = ros::Time::now();
  motor_cmd.left_velocity = left;
  motor_cmd.right_velocity = right;
}

void joystick_Driver::handle_tank_control(const sensor_msgs::JoyConstPtr& joystick)
{
  if (joystick->buttons[joy_map.rb])
  {
    tank_control_config.max_velocity += tank_control_config.velocity_increment;
  }
  if (joystick->buttons[joy_map.lb])
  {
    tank_control_config.max_velocity -= tank_control_config.velocity_increment;
  }
  if (tank_control_config.max_velocity > max_velocity)
  {
    tank_control_config.max_velocity = max_velocity;
  }
  if (tank_control_config.max_velocity <= 0)
  {
    tank_control_config.max_velocity = 0;
  }

  double left = joystick->axes[joy_map.left_axis_y] * tank_control_config.max_velocity;
  double right = joystick->axes[joy_map.right_axis_y] * tank_control_config.max_velocity;

  tank_control_config.velocity = std::hypot(left, right);

  motor_cmd = igvc_msgs::velocity_pair{};
  motor_cmd.header.stamp = ros::Time::now();
  motor_cmd.left_velocity = left;
  motor_cmd.right_velocity = right;
}

void joystick_Driver::handle_smooth_control(const sensor_msgs::JoyConstPtr& joystick)
{
  process_axes(joystick);

  if (imu == nullptr) {
    return;
  }

  if (smooth_control_config.camera == 10) {
    smooth_control_config.camera = get_yaw(imu);
  }

  // 90 degree anticlockwise turn
  if (!rb_clicked && joystick->buttons[joy_map.rb]) {
    rb_clicked = true;
    smooth_control_config.camera -= M_PI_2;
  }
  rb_clicked = joystick->buttons[joy_map.rb];
  // 90 degree clockwise turn
  if (!lb_clicked && joystick->buttons[joy_map.lb]) {
    lb_clicked = true;
    smooth_control_config.camera += M_PI_2;
  }
  lb_clicked = joystick->buttons[joy_map.lb];

  // Center camera
  if (joystick->buttons[joy_map.b]) {
    smooth_control_config.camera = get_yaw(imu);
  }

  // Move camera
  double camera_turn = right_x * right_x * right_x;
  double camera_turn_amount = camera_turn * smooth_control_config.camera_move_rate * control_loop_period;
  smooth_control_config.camera -= camera_turn_amount; // Left hand to right hand
  igvc::fit_to_polar(smooth_control_config.camera);

  // Sensitivity curve ^.^
  double expo_left_x = left_x * left_x * left_x;
  double expo_left_y = left_y * left_y * left_y;
  double velocity = std::hypot(expo_left_x, expo_left_y);
  double delta = -atan2(left_y, left_x);
  delta += M_PI_2;
  delta -= smooth_control_config.camera;

  delta += get_yaw(imu);

  igvc::fit_to_polar(delta);

  double w = -velocity * smooth_control_config.k2 * delta + (1 + smooth_control_config.k1) * sin(delta);
  if (velocity < 0.1)
  {
    w = 0;
  }
  w = std::min(w, smooth_control_config.max_w);
  w = std::max(w, -smooth_control_config.max_w);
  double v_left = (2 * velocity - w * smooth_control_config.axle_length) / 2;
  double v_right = (2 * velocity + w * smooth_control_config.axle_length) / 2;

  //  double max = max(fabs(v_left), fabs(v_right));

  smooth_control_config.delta = delta;
  smooth_control_config.w = w;
  smooth_control_config.velocity = velocity;
  motor_cmd = igvc_msgs::velocity_pair{};
  motor_cmd.header.stamp = ros::Time::now();
  motor_cmd.left_velocity = v_left;
  motor_cmd.right_velocity = v_right;
}

void joystick_Driver::signal_handler()
{
  sigset_t sig;
  sigemptyset(&sig);
  sigaddset(&sig, SIGINT);

  int caught_sig;
  sigwait(&sig, &caught_sig);
  quit = 1;
}

double joystick_Driver::get_yaw(const sensor_msgs::ImuConstPtr& imu) {
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
