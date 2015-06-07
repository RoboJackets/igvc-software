#include <ros/ros.h>
#include <igvc_msgs/velocity_pair.h>
#include <std_msgs/Bool.h>
#include <fstream>
#include <chrono>

using namespace std;
using namespace igvc_msgs;
using namespace ros;

bool enabled = false;

ofstream file;

void encoder_callback(const velocity_pairConstPtr &msg)
{
    file << msg->left_velocity << ", " << msg->right_velocity << endl;
}

int main(int argc, char** argv)
{
    init(argc, argv, "encrecord");

    NodeHandle nh;

    ros::Subscriber encoder_sub = nh.subscribe("/encoders", 10, encoder_callback);

    file.open("/home/robojackets/Desktop/data.csv");

    spin();

    return 0;
}
