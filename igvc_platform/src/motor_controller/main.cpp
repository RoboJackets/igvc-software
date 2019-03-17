#include <igvc_msgs/velocity_pair.h>
#include <igvc_utils/EthernetSocket.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <igvc_utils/NodeUtils.hpp>
#include <string>

// nanopb header files for protobuffer encoding/decoding
#include <igvc_platform/nanopb/pb_encode.h>
#include <igvc_platform/nanopb/pb_decode.h>
#include <igvc_platform/nanopb/pb_common.h>
#include "igvc_platform/nanopb/protos/igvc.pb.h" // compiled protobuf definition

igvc_msgs::velocity_pair current_motor_command; // desired motor velocities
double frequency; // communicate frequency with the mbed

// alpha value for voltage exponentially weighted moving average
// approximate # of timesteps average taken over = 1 / (1-alpha)
double battery_alpha = 0.9;
double min_battery_voltage;
double battery_avg;

double log_period_; // Period for logging messages

double p_l, p_r, d_l, d_r, i_l, i_r; // PID Values

/**
get current motor command from the /motors topic

@param[in] msg current message on the /motors topic
*/
void cmdCallback(const igvc_msgs::velocity_pair::ConstPtr& msg)
{
  current_motor_command = *msg;
}

/**
Sets PID values
*/
void setPID(EthernetSocket& sock, ros::Rate &rate)
{
  bool valid_values = false; // pid values have been set correctly

  /* This is the buffer where we will store the request message. */
  uint8_t requestbuffer[256];
  size_t message_length;
  bool status;

  /* allocate space for the request message to the server */
  RequestMessage request = RequestMessage_init_zero;

  /* Create a stream that will write to our buffer. */
  pb_ostream_t ostream = pb_ostream_from_buffer(requestbuffer, sizeof(requestbuffer));

  /* indicate that pid fields will contain values */
  request.has_p_l = true;
  request.has_p_r = true;
  request.has_i_l = true;
  request.has_i_r = true;
  request.has_d_l = true;
  request.has_d_r = true;

  /* fill in the message fields */
  request.p_l = static_cast<float>(p_l);
  request.p_r = static_cast<float>(p_r);
  request.i_l = static_cast<float>(i_l);
  request.i_r = static_cast<float>(i_r);
  request.d_l = static_cast<float>(d_l);
  request.d_r = static_cast<float>(d_r);

  /* encode the protobuffer */
  status = pb_encode(&ostream, RequestMessage_fields, &request);
  message_length = ostream.bytes_written;

  /* check for any errors.. */
  if (!status)
  {
      ROS_ERROR_STREAM("Encoding failed: " << PB_GET_ERROR(&ostream));
      ros::shutdown();
  }

  size_t n; // n is the response from socket: 0 means connection closed, otherwise n = num bytes read
  uint8_t buffer[256]; // buffer to read response into
  // unsigned char responsebuffer[256];

  /* Send PID values via ethernet and recieve response to ensure proper setting */
  while (ros::ok() && !valid_values)
  {
    sock.sendMessage(reinterpret_cast<char*>(requestbuffer), message_length);

    memset(buffer, 0, sizeof(buffer));
    n = sock.readMessage(buffer); // blocks until data is read
    // memcpy(responsebuffer, buffer, sizeof(responsebuffer));

    if (n == 0)
    {
        ROS_ERROR_STREAM("Connection closed by server");
        ros::shutdown();
    }

    /* Allocate space for the decoded message. */
    ResponseMessage response = ResponseMessage_init_zero;

    /* Create a stream that reads from the buffer. */
    pb_istream_t istream = pb_istream_from_buffer(buffer, n);

    /* decode the message. */
    status = pb_decode(&istream, ResponseMessage_fields, &response);

    /* check for any errors.. */
    if (!status)
    {
        ROS_ERROR_STREAM("Decoding Failed: " << PB_GET_ERROR(&istream));
        ros::shutdown();
    }

    ROS_INFO_STREAM("P_L: " << static_cast<double>(response.p_l) << ", P_R: " << static_cast<double>(response.p_r));

    valid_values = (static_cast<double>(response.p_l) == p_l) && \
                   (static_cast<double>(response.p_r) == p_r) && \
                   (static_cast<double>(response.i_l) == i_l) && \
                   (static_cast<double>(response.i_r) == i_r) && \
                   (static_cast<double>(response.d_l) == d_l) && \
                   (static_cast<double>(response.d_r) == d_r);

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

  int min_battery_voltage;
  igvc::getParam(pNh, std::string("min_battery_voltage"), min_battery_voltage);

  battery_avg = min_battery_voltage;

  // PID variables
  igvc::getParam(pNh, std::string("p_l"), p_l);
  igvc::getParam(pNh, std::string("p_r"), p_r);
  igvc::getParam(pNh, std::string("d_l"), d_l);
  igvc::getParam(pNh, std::string("d_r"), d_r);
  igvc::getParam(pNh, std::string("i_r"), i_r);
  igvc::getParam(pNh, std::string("i_l"), i_l);
  igvc::param(pNh, "log_period", log_period_, 5.0);

  EthernetSocket sock(ip_addr, tcpport);

  ROS_INFO_STREAM("Using Boost " << sock.getBoostVersion());

  ROS_INFO_STREAM("Successfully Connected to TCP Host:"
                  << "\n\tIP: " << sock.getIP()
                  << "\n\tPort: " << sock.getPort());

  igvc::getParam(pNh, std::string("frequency"), frequency);
  ros::Rate rate(frequency);

  ROS_INFO_STREAM("Setting PID Values:"
                  << "\n\t P => L: " << p_l << " R: " << p_r
                  << "\n\t D => L: " << d_l << " R: " << d_r
                  << "\n\t I => L: " << i_l << " R: " << i_r);

  setPID(sock, rate); // Set PID Values on mbed

  // send motor commands
  while (ros::ok())
  {
      /* This is the buffer where we will store the request message. */
      uint8_t requestbuffer[256];
      size_t message_length;
      bool status;

      /* allocate space for the request message to the server */
      RequestMessage request = RequestMessage_init_zero;

      /* Create a stream that will write to our buffer. */
      pb_ostream_t ostream = pb_ostream_from_buffer(requestbuffer, sizeof(requestbuffer));

      /* indicate that speed fields will contain values */
      request.has_speed_l = true;
      request.has_speed_r = true;

      /* fill in the message fields */
      request.speed_l = static_cast<float>(current_motor_command.left_velocity);
      request.speed_r = static_cast<float>(current_motor_command.right_velocity);

      /* encode the protobuffer */
      status = pb_encode(&ostream, RequestMessage_fields, &request);
      message_length = ostream.bytes_written;

      /* check for any errors.. */
      if (!status)
      {
          ROS_ERROR_STREAM("Encoding failed: " << PB_GET_ERROR(&ostream));
          ros::shutdown();
      }

      /* Send the message strapped to a pigeon's leg! */
      sock.sendMessage(reinterpret_cast<char*>(requestbuffer), message_length);

      /* Read response from the server */
      size_t n; // n is the response from socket: 0 means connection closed, otherwise n = num bytes read
      uint8_t buffer[256]; // buffer to read response into

      memset(buffer, 0, sizeof(buffer));
      /* read from the buffer */
      n = sock.readMessage(buffer); // blocks until data is read

      if (n == 0)
      {
          ROS_ERROR_STREAM("Connection closed by server");
          ros::shutdown();
      }

      /* Allocate space for the decoded message. */
      ResponseMessage response = ResponseMessage_init_zero;

      /* Create a stream that reads from the buffer. */
      pb_istream_t istream = pb_istream_from_buffer(buffer, n);

      /* decode the message. */
      status = pb_decode(&istream, ResponseMessage_fields, &response);

      /* check for any errors.. */
      if (!status)
      {
          ROS_ERROR_STREAM("Decoding Failed: " << PB_GET_ERROR(&istream));
          ros::shutdown();
      }

      /* update the exponentially weighted moving voltage average and publish */
      std_msgs::Float64 battery_msg;
      battery_avg = battery_alpha * battery_avg + (1 - battery_alpha) * response.voltage;
      battery_msg.data = battery_avg;
      battery_pub.publish(battery_msg);

      if (battery_avg < min_battery_voltage)
      {
        ROS_ERROR_STREAM_THROTTLE(log_period_, "Battery voltage dangerously low:"
                         << "\n\tCurr. Voltage: " << battery_avg
                         << "\n\tMin. Voltage: " << min_battery_voltage);
      }

      std_msgs::Bool enabled_msg;
      enabled_msg.data = response.estop;
      enabled_pub.publish(enabled_msg);

      /* publish encoder feedback */
      igvc_msgs::velocity_pair enc_msg;
      enc_msg.left_velocity = response.speed_l;
      enc_msg.right_velocity = response.speed_r;
      enc_msg.duration = response.dt_sec;
      enc_msg.header.stamp = ros::Time::now() - ros::Duration(response.dt_sec);
      enc_pub.publish(enc_msg);

      ROS_INFO_STREAM_THROTTLE(log_period_, "Rate: " << response.dt_sec << "s.");

      ros::spinOnce();
      rate.sleep();
  }
}
