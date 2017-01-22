#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <string>
#include <igvc_msgs/velocity_pair.h>
#include <igvc/SerialPort.h>
#include <std_msgs/Bool.h>

igvc_msgs::velocity_pair current_motor_command;

bool enabled = false;

void cmdCallback(const igvc_msgs::velocity_pair::ConstPtr& msg)
{
    current_motor_command = *msg;
}

void enabledCallback(const std_msgs::BoolConstPtr& msg)
{
    enabled = msg->data;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_controller");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");

    ros::Subscriber cmd_sub = nh.subscribe("/motors", 1, cmdCallback);

    ros::Subscriber enabled_sub = nh.subscribe("/robot_enabled", 1, enabledCallback);

    ros::Publisher enc_pub = nh.advertise<igvc_msgs::velocity_pair>("/encoders", 1000);
    
    std::string device_path;
    nhp.param(std::string("device"), device_path, std::string("/dev/igvc_motor_arduino"));
    
    int baud_rate;
    nhp.param(std::string("baud_rate"), baud_rate, 9600);
    
    SerialPort port(device_path, baud_rate);

    if(!port.isOpen())
    {
        ROS_ERROR_STREAM("Motor Controller serial port failed to open.");
        return -1;
    }
    port.flush();
    ROS_INFO_STREAM("Motor Controller ready.");
    
    ros::Rate rate(10);
    while(ros::ok() && port.isOpen())
    {
        ros::spinOnce();
        
        std::string msg = "$" + std::to_string(enabled?current_motor_command.left_velocity:0.0) + "," 
            + std::to_string(enabled?current_motor_command.right_velocity:0.0) + "\n";
        
        if(current_motor_command.left_velocity > 0 || current_motor_command.right_velocity > 0)
            ROS_INFO_STREAM(msg);
        
        port.write(msg);
        
        std::string ret = port.readln();
        try {
            if(!ret.empty())
            {
                size_t dollar = ret.find('$');
                size_t comma = ret.find(',');
                size_t comma2 = ret.find_last_of(',');
                size_t end = ret.find('\n');
                std::string leftStr = ret.substr(dollar+1, comma-dollar-1);
                std::string rightStr = ret.substr(comma+1, comma2-comma-1);
                std::string deltaT = ret.substr(comma2+1, end-comma2-1);
                igvc_msgs::velocity_pair enc_msg;
                enc_msg.left_velocity = atof(leftStr.c_str());
                enc_msg.right_velocity = atof(rightStr.c_str());
                enc_msg.duration = atof(deltaT.c_str());
                enc_pub.publish(enc_msg);
            } else {
                ROS_ERROR_STREAM("Empty return from arduino.\t" << ret);
            }
        } catch (std::out_of_range) { }
        
        rate.sleep();
    }
}
