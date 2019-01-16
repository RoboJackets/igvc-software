#include <igvc_msgs/velocity_pair.h>
#include <igvc_utils/EthernetSocket.h>
#include <igvc_utils/SerialPort.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <igvc_utils/StringUtils.hpp>
#include <igvc_utils/NodeUtils.hpp>

#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

igvc_msgs::velocity_pair current_motor_command;

double min_battery_voltage;
double battery_avg = 20;
double battery_alpha = 0.9; // exponentially weighted moving voltage average update value

double p_l, p_r, d_l, d_r, i_l, i_r; // PID Values

bool enabled = false;
int precision;

/**
get current motor command from the /motors topic

@param[in] msg current message on the /motors topic
*/
void cmdCallback(const igvc_msgs::velocity_pair::ConstPtr& msg)
{
  current_motor_command = *msg;
}

/**
Convert input to fixed length string whose numeric values have fixed precision

@param[in] input value to fixed length string representation of
@return string representation of input with a fized length
*/
std::string toBoundedString(double input)
{
  std::stringstream stream;
  stream << std::fixed << std::setprecision(precision) << input;
  return stream.str();
}

/**
Validate that the returned values match those values sent to the MBed
*/
bool validateValues(std::string ret, int loc, int end, double left, double right)
{
  // split string by ','. The first two substrings correspond to the values for
  // the left and right motor
  std::vector<std::string> vals = split(ret.substr(loc + 1, end), ',');
  ROS_INFO_STREAM("Checking recieved values for: " << ret.at(loc));


  return (vals.size() == 2)
         && (std::stod(vals.at(0)) == left)
         && (std::stod(vals.at(1)) == right);
}

/**
Sets PID values
*/
void setPID(EthernetSocket& sock, ros::Rate &rate)
{
  std::string p_values = "#P" + toBoundedString(p_l) + "," + toBoundedString(p_r) + "\n";
  std::string d_values = "#D" + toBoundedString(d_l) + "," + toBoundedString(d_r) + "\n";
  std::string i_values = "#I" + toBoundedString(i_l) + "," + toBoundedString(i_r) + "\n";

  bool valid_values_p = false;
  bool valid_values_d = false;
  bool valid_values_i = false;

  // Sends PID values via ethernet and recieves them back to ensure proper setting
  while (ros::ok() && (!valid_values_p || !valid_values_d || !valid_values_i))
  {
    if (!valid_values_p)
    {
      sock.sendMessage(p_values);
    }
    if (!valid_values_d)
    {
      sock.sendMessage(d_values);
    }
    if (!valid_values_i)
    {
      sock.sendMessage(i_values);
    }
    for (int i = 0; i < 3; i++)
    {
      std::string ret = sock.readMessage();
      if (!ret.empty() && ret.at(0) == '#' && ret.at(1) != 'E')
      {
        size_t p_loc = ret.find('P');
        size_t d_loc = ret.find('D');
        size_t i_loc = ret.find('I');
        size_t end = ret.find('\n');

        if (p_loc != std::string::npos && !valid_values_p)
        {
          valid_values_p = validateValues(ret, p_loc, end, p_l, p_r);
          if (valid_values_p) { ROS_INFO("Successfully set P values"); }
        }
        else if (d_loc != std::string::npos && !valid_values_d)
        {
          valid_values_d = validateValues(ret, d_loc, end, d_l, d_r);
          if (valid_values_d) { ROS_INFO("Successfully set D values"); }
        }
        else if (i_loc != std::string::npos && !valid_values_i)
        {
          valid_values_i = validateValues(ret, i_loc, end, i_l, i_r);
          if (valid_values_i) { ROS_INFO("Successfully set I values"); }
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
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motor_controller");
  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  // /motors topic contains motor velocity commands provided by path follower
  ros::Subscriber cmd_sub = nh.subscribe("/motors", 1, cmdCallback);

  ros::Publisher enc_pub = nh.advertise<igvc_msgs::velocity_pair>("/encoders", 1000);
  ros::Publisher enabled_pub = nh.advertise<std_msgs::Bool>("/robot_enabled", 1);
  ros::Publisher battery_pub = nh.advertise<std_msgs::Float64>("/battery", 1);

  // get server ip address and port number from the launch file
  std::string ip_addr;
  int tcpport;
  igvc::getParam(pNh, std::string("ip_addr"), ip_addr);
  igvc::getParam(pNh, std::string("port"), tcpport);

  ROS_INFO_STREAM("Connecting to server:"
                  << "\n\tIP: " << ip_addr
                  << "\n\tPort: " << std::to_string(tcpport));

  int messages_to_read;
  int min_battery_voltage;
  pNh.param(std::string("messages_to_read"), messages_to_read, 3);
  igvc::getParam(pNh, std::string("min_battery_voltage"), min_battery_voltage);
  pNh.param(std::string("precision"), precision, 1);

  // PID variables
  igvc::getParam(pNh, std::string("p_l"), p_l);
  igvc::getParam(pNh, std::string("p_r"), p_r);
  igvc::getParam(pNh, std::string("d_l"), d_l);
  igvc::getParam(pNh, std::string("d_r"), d_r);
  igvc::getParam(pNh, std::string("i_r"), i_r);
  igvc::getParam(pNh, std::string("i_l"), i_l);

  EthernetSocket sock(ip_addr, tcpport);

  std::cout << "Using Boost " << sock.getBoostVersion() << std::endl;

  ROS_INFO_STREAM("Successfully Connected to TCP Host:"
                  << "\n\tIP: " << sock.getIP()
                  << "\n\tPort: " << sock.getPort());

  ROS_INFO_STREAM("Motor Board ready.");

  ros::Rate rate(120);

  ROS_INFO_STREAM("Setting PID Values:"
                  << "\n\t P => L: " << p_l << " R: " << p_r
                  << "\n\t D => L: " << d_l << " R: " << d_r
                  << "\n\t I => L: " << i_l << " R: " << i_r);
  setPID(sock, rate); // Set motor's PID Values


  // sends down motor commands and recieves multiple responses back
  // while (ros::ok() && port.isOpen())
  ros::Time begin = ros::Time::now(); // begin interval
  while (ros::ok())
  {
    // construct motor command to send to motorboard (server)
    std::string msg = "$" + (enabled ? toBoundedString(current_motor_command.left_velocity) : toBoundedString(0.0)) +
                      "," + (enabled ? toBoundedString(current_motor_command.right_velocity) : toBoundedString(0.0)) +
                      "\n";
    // send motor command
    sock.sendMessage(msg);

    // read response from server
    std::string ret = sock.readMessage();

    double elapsed = (ros::Time::now() - begin).toSec();

    ROS_INFO_STREAM("Read:" << ret << ", Elapsed: " << elapsed << "s.");

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

            battery_avg = battery_alpha * battery_avg + (1 - battery_alpha) * voltage;

            battery_msg.data = battery_avg;
            battery_pub.publish(battery_msg);
            if (battery_avg < min_battery_voltage)
            {
              ROS_ERROR_STREAM("Battery voltage dangerously low:"
                               << "\n\tCurr. Voltage: " << battery_avg
                               << "\n\tMin. Voltage: " << min_battery_voltage);
            }
            else
            {
              ROS_INFO_STREAM("voltage: " << battery_avg);
            }
            std_msgs::Bool enabled_msg;
            enabled_msg.data = std::stoi(tokens.at(1)) == 1;
            enabled = enabled_msg.data;
            enabled_pub.publish(enabled_msg);
          }
          break;

          case 'E':
            // prints unknown error to terminal
            ROS_ERROR_STREAM("MBED error: " << ret);
            count++;
            break;

          default:
            ROS_ERROR_STREAM("unknown response: " << ret);
            count++;
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
      ret = sock.readMessage();
      dollar = ret.find('$');
      pound = ret.find('#');
    }
    begin = ros::Time::now();
    ros::spinOnce();
    rate.sleep();
  }
}
