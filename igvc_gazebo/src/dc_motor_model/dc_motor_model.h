#ifndef SRC_DC_MOTOR_MODEL_H
#define SRC_DC_MOTOR_MODEL_H

#include <sensor_msgs/JointState.h>

struct DCMotorModelParams
{
  double k;
  double internal_resistance;
};

class DCMotorModel
{
public:
  DCMotorModel();

private:
  void setupPublishers();
  void setupSubscribers();
  void getParams();
  void updateLoop(const ros::TimerEvent& timer_event);

  void voltageCallback(const std_msgs::Float64 voltage);
  void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg);

  std::string effort_topic_;
  std::string voltage_topic_;
  std::string joint_state_topic_;
  std::string joint_name_;

  double loop_rate_{};

  double voltage_{};
  double velocity_{};

  DCMotorModelParams motor_model_params_{};

  ros::NodeHandle nh_;
  ros::NodeHandle pNh_;

  ros::Publisher effort_pub_;

  ros::Subscriber voltage_sub_;
  ros::Subscriber joint_state_sub_;
};

#endif  // SRC_DC_MOTOR_MODEL_H
