/**
 * Class for simulating controls with gazebo.
 *
 * Author: Oswin So <oswinso@gmail.com>
 */
#ifndef SRC_GAZEBO_CONTROL_H
#define SRC_GAZEBO_CONTROL_H

struct PIDCoefficients
{
  double Kp;
  double Kd;
  double Ki;
};

struct FeedforwardCoefficients
{
  double Kv;
};

class GazeboControl
{
public:
  GazeboControl();

private:
  void controlLoop(const ros::TimerEvent &timer_event);
  void motorCallback(const igvc_msgs::velocity_pair::ConstPtr &msg);
  void encoderCallback(const igvc_msgs::velocity_pair::ConstPtr &msg);
  std::pair<double, double> getControls(double dt);

  void getParams();
  void setupPublishers();
  void setupShock();
  void setupSubscribers();

  std::string voltage_left_topic_;
  std::string voltage_right_topic_;
  std::string encoder_topic_;
  std::string motor_topic_;

  double loop_rate_{};
  double max_voltage_{};

  double set_point_left_{};
  double set_point_right_{};

  double measured_left_{};
  double measured_right_{};

  double measured_left_prev_{};
  double measured_right_prev_{};

  double lp_derivative_l{};
  double lp_derivative_r{};

  double accum_left_{};
  double accum_right_{};

  double d_low_pass_alpha_{};

  double integral_clamp_;

  PIDCoefficients left_coeffs_;
  PIDCoefficients right_coeffs_;

  FeedforwardCoefficients left_feedforward_coeffs_;
  FeedforwardCoefficients right_feedforward_coeffs_;

  ros::Time prev_time_;

  ros::NodeHandle nh_;
  ros::NodeHandle pNh_;

  ros::Publisher voltage_pub_left_;
  ros::Publisher voltage_pub_right_;

  ros::Subscriber motor_sub_;
  ros::Subscriber encoder_sub_;
};

#endif  // SRC_GAZEBO_CONTROL_H
