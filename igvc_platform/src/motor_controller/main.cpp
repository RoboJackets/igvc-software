#include <igvc_msgs/velocity_pair.h>
#include <igvc_utils/SerialPort.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <igvc_utils/StringUtils.hpp>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

igvc_msgs::velocity_pair current_motor_command;

double battery_avg = 0;
double battery_avg_num = 100;
std::list<double> battery_vals;

bool enabled = false;
int precision;

void cmdCallback(const igvc_msgs::velocity_pair::ConstPtr& msg)
{
  current_motor_command = *msg;
}

std::string toBoundedString(double input)
{
  std::stringstream stream;
  stream << std::fixed << std::setprecision(precision) << input;
  return stream.str();
}

bool validateValues(std::string ret, int loc, int end, double left, double right)
{
  std::vector<std::string> vals = split(ret.substr(loc + 1, end), ',');
  ROS_INFO_STREAM("Successfully set p values");
  bool valid_values = true;
  if (vals.size() == 2)
  {
    valid_values = (stod(vals.at(0)) == left);
    valid_values = (stod(vals.at(1)) == right) && valid_values;
  }
  return valid_values;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motor_controller");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  ros::Subscriber cmd_sub = nh.subscribe("/motors", 1, cmdCallback);

  ros::Publisher enc_pub = nh.advertise<igvc_msgs::velocity_pair>("/encoders", 1000);

  ros::Publisher enabled_pub = nh.advertise<std_msgs::Bool>("/robot_enabled", 1);

  ros::Publisher battery_pub = nh.advertise<std_msgs::Float64>("/battery", 1);

  std::string device_path;
  nhp.param(std::string("device"), device_path, std::string("/dev/igvc_motor_board"));

  int baud_rate;
  nhp.param(std::string("baud_rate"), baud_rate, 9600);

  int messages_to_read;
  nhp.param(std::string("messages_to_read"), messages_to_read, 3);

  nhp.param("precision", precision, 1);

  double p_l, p_r, d_l, d_r, i_l, i_r;
  nhp.param("p_l", p_l, 3.0);
  nhp.param("p_r", p_r, 3.0);
  nhp.param("d_l", d_l, 0.0);
  nhp.param("d_r", d_r, 0.0);
  nhp.param("i_r", i_r, 0.0);
  nhp.param("i_l", i_l, 0.0);

  SerialPort port(device_path, baud_rate);

  if (!port.isOpen())
  {
    ROS_ERROR_STREAM("Motor Board serial port failed to open.");
    return -1;
  }
  port.flush();
  ROS_INFO_STREAM("Motor Board ready.");

  ros::Rate rate(20);

  std::string p_values = "#P" + toBoundedString(p_l) + "," + toBoundedString(p_r) + "\n";
  std::string d_values = "#D" + toBoundedString(d_l) + "," + toBoundedString(d_r) + "\n";
  std::string i_values = "#I" + toBoundedString(i_l) + "," + toBoundedString(i_r) + "\n";

  bool valid_values_p = false;
  bool valid_values_d = false;
  bool valid_values_i = false;

  // Set the PID and values, sends values down and recieves them copied back to ensure success
  while (ros::ok() && (!valid_values_p || !valid_values_d || !valid_values_i))
  {
    if (!valid_values_p)
    {
      port.write(p_values);
    }
    if (!valid_values_d)
    {
      port.write(d_values);
    }
    if (!valid_values_i)
    {
      port.write(i_values);
    }
    for (int i = 0; i < 3; i++)
    {
      std::string ret = port.readln();
      if (!ret.empty() && ret.at(0) == '#' && ret.at(1) != 'E')
      {
        size_t p_loc = ret.find('P');
        size_t d_loc = ret.find('D');
        size_t i_loc = ret.find('I');
        size_t end = ret.find('\n');
        if (p_loc != std::string::npos)
        {
          valid_values_p = validateValues(ret, p_loc, end, p_l, p_r);
          ROS_INFO("Successfully set P values");
        }
        else if (d_loc != std::string::npos)
        {
          valid_values_d = validateValues(ret, d_loc, end, d_l, d_r);
          ROS_INFO("Successfully set D values");
        }
        else if (i_loc != std::string::npos)
        {
          valid_values_i = validateValues(ret, i_loc, end, i_l, i_r);
          ROS_INFO("Successfully set I values");
        }
        else if (ret.at(1) != 'V')
        {
          ROS_ERROR_STREAM("Recieved unknown string while setting PID values " << ret);
        }
      }
      else if (ret.empty())
      {
        ROS_ERROR_STREAM("Empty return from motor board while sending PID values.\t" << ret);
      }
    }
    rate.sleep();
  }
  ROS_INFO_STREAM("Sucessfully set all PID values");

  // sends down motor commands and recieves multiple responses back
  while (ros::ok() && port.isOpen())
  {
    std::string msg = "$" + (enabled ? toBoundedString(current_motor_command.left_velocity) : toBoundedString(0.0)) +
                      "," + (enabled ? toBoundedString(current_motor_command.right_velocity) : toBoundedString(0.0)) +
                      "\n";

    port.write(msg);

    std::string ret = port.readln();
    size_t dollar = ret.find('$');
    size_t pound = ret.find('#');
    int count = 0;
    while ((dollar != std::string::npos || pound != std::string::npos) && count <= messages_to_read)
    {
      count++;
      // size_t end = ret.find('\n');
      size_t end = ret.size();
      std::vector<std::string> tokens;

      if (ret.size() <= 0)
      {
        ROS_INFO_STREAM("Invalid number of tokens from motor board, ret: " << ret);
      }
      else if (pound != std::string::npos)
      {
        tokens = split(ret.substr(pound + 2, end), ',');
        switch (ret.at(1))
        {
          case 'I':
            // imu message
            break;

          case 'V':
          {
            // recieves battery message and publishes it, also checks for robot enabled
            std_msgs::Float64 battery_msg;
            double voltage = atof(tokens.at(0).c_str());
            battery_vals.push_back(voltage);
            if (battery_vals.size() > battery_avg_num)
            {
              battery_avg -= battery_vals.front() / battery_avg_num;
              battery_vals.pop_front();
            }
            battery_avg += voltage / battery_avg_num;
            battery_msg.data = battery_avg;
            battery_pub.publish(battery_msg);
            if (battery_avg < 23.5 && battery_vals.size() >= battery_avg_num)
            {
              ROS_ERROR_STREAM("Battery voltage dangerously low");
            }
            std_msgs::Bool enabled_msg;
            enabled_msg.data = tokens.at(1) == "1";
            enabled = enabled_msg.data;
            enabled_pub.publish(enabled_msg);
          }
          break;

          case 'E':
            // prints unknown error to terminal
            ROS_ERROR_STREAM("MBED error: " << ret);
            count--;
            break;

          default:
            ROS_ERROR_STREAM("unknown response: " << ret);
            count--;
            break;
        }
      }
      else if (dollar != std::string::npos)
      {
        // handles encoder feedback
        tokens = split(ret.substr(pound + 2, end), ',');
        igvc_msgs::velocity_pair enc_msg;
        enc_msg.left_velocity = atof(tokens.at(0).c_str());
        enc_msg.right_velocity = atof(tokens.at(1).c_str());
        enc_msg.duration = atof(tokens.at(2).c_str());
        enc_msg.header.stamp = ros::Time::now();
        enc_pub.publish(enc_msg);
      }
      else
      {
        // unknown error message
        ROS_ERROR_STREAM("Unknown message from motor board " << ret);
      }
      ret = port.readln();
      dollar = ret.find('$');
      pound = ret.find('#');
    }
    ros::spinOnce();
    rate.sleep();
  }
}
