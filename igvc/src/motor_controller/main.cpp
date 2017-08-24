#include <igvc/SerialPort.h>
#include <igvc_msgs/velocity_pair.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <string>

igvc_msgs::velocity_pair current_motor_command;

bool enabled = false;

double w = 0;

double p = 3;
double d = 0;

double last_speed = 0;
double last_error = 0;
double PWM = 0;
double right_velocity = 0;
int power = 0;

void cmdCallback(const igvc_msgs::velocity_pair::ConstPtr& msg)
{
  current_motor_command = *msg;
}

void enabledCallback(const std_msgs::BoolConstPtr& msg)
{
  enabled = msg->data;
}

void imu_callback(const sensor_msgs::Imu::Ptr& msg) {
  w = msg->angular_velocity.z;
  ROS_ERROR_STREAM("callback = " << w);
  double current = 0;
  if(w > 0.1) {
    current = -w/0.472 + right_velocity;
  } else if(w < -0.1) {
    current = -w / 0.472 + right_velocity;
  }
  double error = current - current_motor_command.left_velocity;
  double dError = error - last_error;
  int dPWM = p * error + dError * d;
  PWM -= dPWM;
  if(PWM < -255) {
    PWM = -255;
  }
  if(PWM > 255) {
    PWM = 255;
  }
  if(PWM < 2 && PWM > -2) {
    PWM = 0;
  }
  power = 255 - PWM;
  last_error = error;
  last_speed = current;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motor_controller");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  ros::Subscriber cmd_sub = nh.subscribe("/motors", 1, cmdCallback);

  ros::Subscriber enabled_sub = nh.subscribe("/robot_enabled", 1, enabledCallback);

  ros::Subscriber IMU_SUB = nh.subscribe("/imu", 100, imu_callback);

  ros::Publisher enc_pub = nh.advertise<igvc_msgs::velocity_pair>("/encoders", 1000);

  std::string device_path;
  nhp.param(std::string("device"), device_path, std::string("/dev/igvc_motor_arduino"));

  int baud_rate;
  nhp.param(std::string("baud_rate"), baud_rate, 9600);

  SerialPort port(device_path, baud_rate);

  if (!port.isOpen())
  {
    ROS_ERROR_STREAM("Motor Controller serial port failed to open.");
    return -1;
  }
  port.flush();
  ROS_INFO_STREAM("Motor Controller ready.");

  ros::Rate rate(10);
  while (ros::ok() && port.isOpen())
  {
    ros::spinOnce();

    std::string msg = "$" + std::to_string(enabled ? power : 0.0) + "," +
                      std::to_string(enabled ? current_motor_command.right_velocity : 0.0) + "\n";

    //if (fabs(current_motor_command.left_velocity) > 0 || fabs(current_motor_command.right_velocity) > 0)
      ROS_ERROR_STREAM("sending " << msg);

    port.write(msg);

    std::string ret = port.readln();
    try
    {
      if (!ret.empty())
      {
        size_t dollar = ret.find('$');
        size_t comma = ret.find(',');
        size_t comma2 = ret.find_last_of(',');
        size_t end = ret.find('\n');
        std::string leftStr = ret.substr(dollar + 1, comma - dollar - 1);
        std::string rightStr = ret.substr(comma + 1, comma2 - comma - 1);
        std::string deltaT = ret.substr(comma2 + 1, end - comma2 - 1);
        igvc_msgs::velocity_pair enc_msg;
        enc_msg.right_velocity = atof(rightStr.c_str());
        right_velocity = 0.5;
        //if( w == 0) {
        //  ROS_ERROR_STREAM("w = 0");
        //} else {
        ROS_ERROR_STREAM("w = " << w); 
        if(w > 0.1) {
          enc_msg.left_velocity = -w/0.472 + right_velocity;
        } else if(w < -0.1) {
          enc_msg.left_velocity = -w / 0.472 + right_velocity;
        } else {
          enc_msg.left_velocity = enc_msg.right_velocity;
        }
        //}
        enc_msg.duration = atof(deltaT.c_str());
        enc_pub.publish(enc_msg);
      }
      else
      {
        ROS_ERROR_STREAM("Empty return from motor arduino.\t" << ret);
      }
    }
    catch (std::out_of_range)
    {
    }

    rate.sleep();
  }
}
