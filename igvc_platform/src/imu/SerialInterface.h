#pragma once

#include <malloc.h>
#include <ros/ros.h>
#include <serial/serial.h>

using namespace serial;

class SerialInterface
{
private:
  using SerialPtr = std::unique_ptr<serial::Serial>;
  // baudrate
  int baud_;
  // connection related variables
  SerialPtr connection_port_;
  std::string port_;
  bool connected_ = false;
  // logger zone
  const std::string log_zone_;

public:
  SerialInterface(ros::NodeHandle &serial_nh_) : log_zone_("[ SerialInterface ] ")
  {
    serial_nh_.param<int>("BAUD_RATE", baud_, 115200);
    serial_nh_.param<std::string>("SERIAL_PORT", port_, "/dev/ttyACM0");
  }

  const int &getBaudRate()
  {
    return baud_;
  }

  /**
   * Destructor for the interface
   */
  const std::string &getSerialPort()
  {
    return port_;
  }

  /**
   * Returns true if connection is established with the serial port.
   *
   * @return true if connected to serial, false otherwise
   */
  bool isConnected()
  {
    return connected_;
  }

  /**
   * Destructor
   */
  ~SerialInterface()
  {
    if (connection_port_ != NULL)
    {
      if (connection_port_->isOpen())
      {
        ROS_INFO_STREAM(this->log_zone_ << " Closing the Serial Port");
        connection_port_->close();
          connected_ = false;
      }
    }
  }

  /**
   * Connects to the serial port.
   */
  void serialConnect()
  {
    try
    {
      connection_port_.reset(new Serial(port_, (uint32_t)baud_, Timeout::simpleTimeout(60000)));
    }
    catch (IOException &e)
    {
      std::string ioerror = e.what();
      ROS_ERROR_STREAM(this->log_zone_ << "Unable to connect port: " << port_.c_str());
      ROS_ERROR_STREAM(this->log_zone_ << "Is the serial port open? : " << ioerror.c_str());
        connected_ = false;
    }

    if (connection_port_ && connection_port_->isOpen())
    {
      ROS_INFO_STREAM(this->log_zone_ << "Connection Established with Port: " << port_.c_str()
                                      << " with baudrate: " << baud_);
        connected_ = true;
    }
  }

  /**
   * Writes a string to the serial port
   *
   * @param str The string to write to the serial port
   */
  inline void serialWriteString(const std::string &str)
  {
    this->connection_port_->write(str);
  }

  /**
   * Sets the timeout of read and write operations.
   *
   * @param timeout The desired timeout in milliseconds
   */
  void setTimeout(int timeout = 500)
  {
    Timeout t = Timeout::simpleTimeout((uint32_t)timeout);
    this->connection_port_->setTimeout(t);
  }

  /**
   * Gets the timeout of read operations.
   *
   * @return the current timeout of read operations in milliseconds
   */
  int getTimeout()
  {
    Timeout t = this->connection_port_->getTimeout();
    return t.read_timeout_constant;
  }

  /**
   * Flushes the read and write string buffers.
   */
  inline void flush()
  {
    this->connection_port_->flush();
  }

  /**
   * Flushes the read and write string buffers.
   *
   * @return the string read in from serial
   */
  inline std::string serialReadLine()
  {
    std::string str = this->connection_port_->readline();
    return str;
  }

  /**
   * Gets the number of available bytes
   *
   * @return the number of available bytes
   */
  inline size_t available()
  {
    return this->connection_port_->available();
  }
};
