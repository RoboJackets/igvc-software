#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <string>
#include <igvc_msgs/velocity_pair.h>
#include <igvc/SerialPort.h>

using namespace std;

igvc_msgs::velocity_pair cmd;

void cmdCallback(const igvc_msgs::velocity_pair::ConstPtr& msg)
{
    cmd = *msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_controller");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    
    std::string cmd_topic_name;
    nhp.param(string("cmd_topic"), cmd_topic_name, string("/motors"));
    ros::Subscriber cmd_sub = nh.subscribe(cmd_topic_name, 1, cmdCallback);
    
    std::string enc_topic_name;
    nhp.param(string("enc_topic"), enc_topic_name, string("/encoders"));
    ros::Publisher enc_pub = nh.advertise<igvc_msgs::velocity_pair>(enc_topic_name, 1000);
    
    std::string device_path;
    nhp.param(string("device"), device_path, string("/dev/igvc_motor_arduino"));
    
    int baud_rate;
    nhp.param(string("baud_rate"), baud_rate, 9600);
    
    SerialPort port(device_path, baud_rate);
    
    ros::Rate rate(10);
    while(ros::ok() && port.isOpen())
    {
        ros::spinOnce();
        
        string msg = "$" + to_string(cmd.left_velocity) + "," + to_string(cmd.right_velocity) + "\n";
        
        port.write(msg);
        
        string ret = port.readln();
        
        try {
            if(!ret.empty())
            {
                size_t dollar = ret.find('$');
                size_t comma = ret.find(',');
                size_t end = ret.find('\n');
                string leftStr = ret.substr(dollar+1, comma-dollar-1);
                string rightStr = ret.substr(comma+1, end-comma-1);
                igvc_msgs::velocity_pair enc_msg;
                enc_msg.left_velocity = atof(leftStr.c_str());
                enc_msg.right_velocity = atof(rightStr.c_str());
            }
        } catch (std::out_of_range) { }
        
        rate.sleep();
    }
}
