#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/subscriber.h>

#include <memory>
#include <string>

#include <igvc_msgs/velocity_pair.h>
#include <igvc_utils/EthernetSocket.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

class MotorController
{
public:
  MotorController(ros::NodeHandle* nodehandle);

private:
  ros::NodeHandle nh_;
  ros::Time last_motors_message_;

  // seconds to wait before stopping if no new motors command comes in
  double watchdog_delay_;

  igvc_msgs::velocity_pair current_motor_command_;  // desired motor velocities
  double p_l_, p_r_, d_l_, d_r_, i_l_, i_r_;        // PID Values
  double kv_l_, kv_r_;

  // launch parameters
  std::string ip_addr_;         // server ip address
  int tcpport_;                 // server tcp tcp port
  double min_battery_voltage_;  // min battery voltage before warnings
  double log_period_;           // Period for logging messages
  double frequency_;            // communicate frequency_ with the mbed
  double battery_alpha_;        // alpha value for voltage exponentially weighted moving average
                                // approximate # of timesteps average taken over = 1 / (1-alpha)

  // current battery voltage
  double battery_avg_;

  std::unique_ptr<EthernetSocket> sock_;

  // subscribers
  ros::Subscriber cmd_sub_;

  // publishers
  ros::Publisher enc_pub_;
  ros::Publisher enabled_pub_;
  ros::Publisher battery_pub_;

  // diagnostics
  diagnostic_updater::Updater mc_updater_;
  diagnostic_updater::Updater battery_updater_;
  double mc_hertz_ = 0;

  /**
  updates the motor controller diagnostics

  @param[in] Diagnostics Status Wrapper
  */
  void mc_diagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat);
  /**
  updates the battery diagnostics

  @param[in] Diagnostics Status Wrapper
  */
  void battery_diagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat);
  /**
    get current motor command from the /motors topic

    @param[in] msg current message on the /motors topic
    */
  void cmdCallback(const igvc_msgs::velocity_pair::ConstPtr& msg);
  /**
  Sets PID values on the mbed
  */
  void setPID();
  /**
  Sends a request to the mbed using the current motor velocities, as read from
  the /motors topic
  */
  void sendRequest();
  /**
  Recieves a message from the mbed containing battery voltage, enabled status, and
  encoder information
  */
  void recieveResponse();
  /**
  Publishes robot status recieved from the mbed to the following topics:
    - /encoders
    - /robot_enabled
    - /battery

  @param[in] response the response message recieved from the mbed
  */
  void publishResponse(const ResponseMessage& response);
};
