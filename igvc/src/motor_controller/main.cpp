#include <igvc/SerialPort.h>
#include <igvc_msgs/velocity_pair.h>
#include <std_msgs/UInt8.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <std_msgs/Bool.h>
#include <string>
#include <vector>
#include <igvc/StringUtils.hpp>

igvc_msgs::velocity_pair current_motor_command;

bool enabled = false;

void cmdCallback(const igvc_msgs::velocity_pair::ConstPtr& msg)
{
  current_motor_command = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motor_controller");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  ros::Subscriber cmd_sub = nh.subscribe("/motors", 1, cmdCallback);

  ros::Publisher enc_pub = nh.advertise<igvc_msgs::velocity_pair>("/encoders", 1000);

  ros::Publisher enabled_pub = nh.advertise<std_msgs::Bool>("/robot_enabled", 1);

  ros::Publisher battery_pub = nh.advertise<std_msgs::UInt8>("/battery", 1);

  std::string device_path;
  nhp.param(std::string("device"), device_path, std::string("/dev/igvc_motor_arduino"));

  int baud_rate;
  nhp.param(std::string("baud_rate"), baud_rate, 9600);

  double p_l, p_r, d_l, d_r;
  nhp.param("p_l", p_l, 1.0);
  nhp.param("p_r", p_r, 1.0);
  nhp.param("d_l", d_l, 0.0);
  nhp.param("d_r", d_r, 0.0);

  ROS_INFO_STREAM("Opening port");
  SerialPort port(device_path, baud_rate);
  ROS_INFO_STREAM("Opened port");

  if (!port.isOpen())
  {
    ROS_ERROR_STREAM("Motor Board serial port failed to open.");
    return -1;
  }
  port.flush();
  ROS_INFO_STREAM("Motor Board ready.");

  ros::Rate rate(10);

  std::string pid_values = "#P" + std::to_string(p_l) + "," + std::to_string(p_r) + ","
      + std::to_string(d_l) + "," + std::to_string(d_r);

  bool valid_values = false;

  while(ros::ok() && !valid_values) {
    ros::spinOnce();
    ROS_INFO_STREAM("sending PID values to board");
    port.write(pid_values);

    std::string ret = port.readln();
    try {
      if(!ret.empty()) {
        valid_values = true;
        size_t p_loc = ret.find('P');
        size_t end = ret.find('\n');
        std::vector<std::string> vals = split(ret.substr(p_loc + 1, end), ',');
        ROS_INFO_STREAM("ret = " << ret);
        if(vals.size() == 4) {
          valid_values = (stod(vals.at(0)) == p_l) && valid_values;
          valid_values = (stod(vals.at(1)) == p_r) && valid_values;
          valid_values = (stod(vals.at(2)) == d_l) && valid_values;
          valid_values = (stod(vals.at(3)) == d_r) && valid_values;
        }
      } else {
        ROS_ERROR_STREAM("Empty return from motor arduino while sending PID values.\t" << ret);
      }
    }
    catch (std::out_of_range) {}
    rate.sleep();
  }
  ROS_INFO_STREAM("sucessfully sent PID values");

  while (ros::ok() && port.isOpen()) {
    ros::spinOnce();
    std::string msg = "$" + std::to_string(enabled ? current_motor_command.left_velocity : 0.0) + "," +
                      std::to_string(enabled ? current_motor_command.right_velocity : 0.0) + "\n";
    ROS_INFO_STREAM("write");
    port.write(msg);
    ROS_INFO_STREAM("write done");


    try {
      std::string ret = port.readln();
      size_t dollar = ret.find('$');
      size_t pound = ret.find('#');
      while (dollar != std::string::npos || pound != std::string::npos) {
        size_t end = ret.find('\n');
        std::vector<std::string> tokens;
        if(dollar != std::string::npos) {
          tokens = split(ret.substr(dollar + 1, end), ',');
        }
        if(pound != std::string::npos) {
          tokens = split(ret.substr(pound + 2, end), ',');
        }

        if(tokens.size() <= 0) {
          ROS_INFO_STREAM("invalid number of tokens from motor arduino");
        } else {
          ROS_INFO_STREAM("ret = " << ret);
          if(pound != std::string::npos) {
            if(ret.at(1) == 'I') {
              // imu message
            } else if(ret.at(1) == 'V') {
              ROS_INFO_STREAM("V " << tokens.at(0));
              std_msgs::UInt8 battery_msg;
              battery_msg.data = atoi(tokens.at(0).c_str());
              ROS_INFO_STREAM("V2 = " << battery_msg.data);
              battery_pub.publish(battery_msg);
            }
          } else if(dollar != std::string::npos) {
            igvc_msgs::velocity_pair enc_msg;
            enc_msg.left_velocity = atof(tokens.at(0).c_str());
            enc_msg.right_velocity = atof(tokens.at(1).c_str());
            enc_msg.duration = atof(tokens.at(2).c_str());
            enc_pub.publish(enc_msg);
            std_msgs::Bool enabled_msg;
            enabled_msg.data = tokens.at(3) == "1";
            enabled_pub.publish(enabled_msg);
          }
        }
        ret = port.readln();
        dollar = ret.find('$');
        pound = ret.find('#');
      }
    }
    catch (std::out_of_range) {}
    rate.sleep();
  }
}
