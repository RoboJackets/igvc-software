#include <igvc_msgs/velocity_pair.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <igvc_utils/StringUtils.hpp>
#include <igvc_utils/NodeUtils.hpp>
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

/**
* get current motor command from /motors topic
*
* @param[in] msg the desired velocities for the left and right motors
*/
void cmdCallback(const igvc_msgs::velocity_pair::ConstPtr& msg)
{
    current_motor_command = *msg;
}

/**
* Creates a fixed-length string of the given input
*
* @param[in] input value to convert to a fixed length string
*/
std::string toBoundedString(double input)
{
    std::stringstream stream;
    stream << std::fixed << std::setprecision(precision) << input;
    return stream.str();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_controller");
    ros::NodeHandle nh;
    ros::NodeHandle pNh("~");

    ros::Subscriber cmd_sub = nh.subscribe("/motors", 1, cmdCallback);

    ros::Publisher enc_pub = nh.advertise<igvc_msgs::velocity_pair>("/encoders", 1000);
    ros::Publisher enabled_pub = nh.advertise<std_msgs::Bool>("/robot_enabled", 1);
    ros::Publisher battery_pub = nh.advertise<std_msgs::Float64>("/battery", 1);

    std::string ip_addr;
    igvc::getParam(pNh, "ip_addr", ip_addr);
    int port;
    igvc::getParam(pNh, "port", port);

    igvc::param(pNh, "precision", precision, 1);

    double p_l, p_r, d_l, d_r, i_l, i_r;
    igvc::param(pNh, "p_l", p_l, 3.0);
    igvc::param(pNh, "p_r", p_r, 3.0);
    igvc::param(pNh, "d_l", d_l, 0.0);
    igvc::param(pNh, "d_r", d_r, 0.0);
    igvc::param(pNh, "i_r", i_r, 0.0);
    igvc::param(pNh, "i_l", i_l, 0.0);

    Ethernet socket(ip_addr, port);

}
