#include <ros/ros.h>
#include <igvc_msgs/velocity_pair.h>
#include <std_msgs/Bool.h>
#include <fstream>
#include <chrono>

using namespace std;
using namespace igvc_msgs;
using namespace ros;

bool enabled = false;

void enabled_callback(const std_msgs::BoolConstPtr &msg)
{
    enabled = msg->data;
}

int main(int argc, char** argv)
{
    init(argc, argv, "pidtester");

    NodeHandle nh;

    ros::Subscriber enabled_sub = nh.subscribe("/robot_enabled", 1, enabled_callback);

    Publisher cmd_pub = nh.advertise<velocity_pair>("/motors", 1);

    while(!enabled)
    {
        ros::spinOnce();
    }

    usleep(500);

    auto duration = 5.0;

    velocity_pair cmd;
    cmd.duration = duration;
    cmd.left_velocity = 1.0;
    cmd.right_velocity = 1.0;

    cmd_pub.publish(cmd);

    spinOnce();
    sleep((__useconds_t)(duration));

    cmd.left_velocity = 0.0;
    cmd.right_velocity = 0.0;

    cmd_pub.publish(cmd);

    spin();

    return 0;
}
