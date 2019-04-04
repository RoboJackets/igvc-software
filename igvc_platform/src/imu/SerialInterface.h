#pragma once

#include <malloc.h>
#include <ros/ros.h>
#include <serial/serial.h>

using namespace serial;

class SerialInterface
{
private:
  using SerialPtr = std::unique_ptr<serial::Serial>;

public:
  SerialInterface(ros::NodeHandle& serial_nh_) : log_zone_("[ SerialInterface ] ")
  {
    serial_nh_.param<int>("BAUD_RATE", baud_, 115200);
    serial_nh_.param<std::string>("SERIAL_PORT", port_, "/dev/ttyACM0");
  }

  const int& getBaudRate()
  {
    return baud_;
  }

  const std::string& getSerialPort()
  {
    return port_;
  }

  virtual ~SerialInterface()
  {
    if (connection_port != NULL)
    {
      if (connection_port->isOpen())
      {
        ROS_INFO_STREAM(this->log_zone_ << " Closing the Serial Port");
        connection_port->close();
      }
    }
  }

  virtual void SerialConnect()
  {
    try
    {
      connection_port.reset(new Serial(port_, (uint32_t)baud_, Timeout::simpleTimeout(60000)));
    }
    catch (IOException& e)
    {
      std::string ioerror = e.what();
      ROS_ERROR_STREAM(this->log_zone_ << "Unable to connect port: " << port_.c_str());
      ROS_ERROR_STREAM(this->log_zone_ << "Is the serial port open? : " << ioerror.c_str());
    }

    if (connection_port && connection_port->isOpen())
    {
      ROS_INFO_STREAM(this->log_zone_ << "Connection Established with Port: " << port_.c_str()
                                      << " with baudrate: " << baud_);
    }
  }

  virtual inline void SerialWrite(uint8_t* buf, size_t len_)
  {
    size_t written_ = this->connection_port->write(buf, len_);
    if (written_ != len_)
      ROS_WARN_STREAM(this->log_zone_ << "Len: " << len_ << " Written: " << written_);
  }

  virtual inline void SerialWriteString(const std::string& str)
  {
    size_t written_ = this->connection_port->write(str);
  }

  virtual inline uint8_t SerialReadByte()
  {
    uint8_t buffer;
    if (this->connection_port->available() > 0)
    {
      size_t byte_ = this->connection_port->read(&buffer, 1);
      if ((byte_ != 1))
        ROS_WARN_STREAM(this->log_zone_ << "Unable to read");
    }
    return buffer;
  }

  virtual inline std::string SerialReadLine()
  {
    std::string str = this->connection_port->readline();
    return str;
  }

  virtual inline uint8_t* SerialReadBytes(size_t number_of_bytes)
  {
    uint8_t* buffer = (uint8_t*)malloc(sizeof(uint8_t) * number_of_bytes);
    if (this->connection_port->available() > 0)
    {
      size_t byte_ = this->connection_port->read(buffer, number_of_bytes);
      if ((byte_ != number_of_bytes))
        ROS_WARN_STREAM(this->log_zone_ << "Unable to read");
    }
    return buffer;
  }

  virtual inline size_t Available()
  {
    return this->connection_port->available();
  }

private:
  //! baudrate
  int baud_;
  //! connection related variables
  SerialPtr connection_port;
  std::string port_;
  // logger zone
  const std::string log_zone_;
};
