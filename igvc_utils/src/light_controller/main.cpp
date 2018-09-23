#include <igvc/SerialPort.h>
#include <igvc_msgs/lights.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>

igvc_msgs::lights state;

ros::Time lastCmdTime;

void lights_callback(const igvc_msgs::lightsConstPtr &msg)
{
  state = *msg;
  lastCmdTime = ros::Time::now();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "light_controller");

  ros::NodeHandle nh;

  SerialPort port{ "/dev/igvc_light_arduino", 9600 };

  if (!port.isOpen())
  {
    ROS_ERROR_STREAM("Failed to open port: " << port.devicePath());
    return -1;
  }

  ros::Subscriber lights_sub = nh.subscribe("/lights", 1, lights_callback);

  ros::Publisher enabled_pub = nh.advertise<std_msgs::Bool>("/robot_enabled", 1);

  ros::Publisher battery_pub = nh.advertise<std_msgs::UInt8>("/battery", 1);

  ROS_INFO_STREAM("Light controller ready.");

  sleep(2);  // Give the arduino time to boot.

  ros::Rate rate(10);
  while (ros::ok() && port.isOpen())
  {
    if ((ros::Time::now() - lastCmdTime).toSec() > 2.0)
      state.safety_flashing = false;
    unsigned char msg[12] = { 2,
                              state.safety_flashing,
                              state.underglow_color[0],
                              state.underglow_color[1],
                              state.underglow_color[2],
                              state.underglow_brightness[0],
                              state.underglow_brightness[1],
                              state.underglow_brightness[2],
                              state.underglow_brightness[3],
                              state.underglow_brightness[4],
                              state.underglow_brightness[5],
                              4 };

    port.write(msg, 12);

    unsigned char *ret = (unsigned char *)port.read(4);

    if (ret[0] != 2 || ret[3] != 4)
      ROS_ERROR_STREAM("Bad format for return packet.");
    else if (ret[1] == 21)
      ROS_ERROR_STREAM("Light controller gave error code " << ret[2]);
    else
    {
      std_msgs::Bool estop_msg;
      estop_msg.data = !(bool)(ret[1]);

      enabled_pub.publish(estop_msg);

      std_msgs::UInt8 battery_msg;
      battery_msg.data = ret[2];

      battery_pub.publish(battery_msg);
    }

    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
