#include <ros/ros.h>
#include <ros/publisher.h>
#include <sensor_msgs/Imu.h>
#include <igvc/SerialPort.h>
#include <vector>
#include <sstream>
#include <tf/transform_datatypes.h>

using namespace std;

vector<string> split(const string &s, const char &delim) {
    vector<string> elems;
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ardupilot");
    
    ros::NodeHandle nh;
    
    ros::Publisher pub = nh.advertise<sensor_msgs::Imu>("/imu", 1000);
    
    SerialPort port("/dev/igvc_imu", 115200);
    
    while(ros::ok() && port.isOpen())
    {
        ros::spinOnce();
        
        auto line = port.readln();
        
        if(line[0] != 'A') continue;
        
        auto tokens = split(line, ' ');
        
        if(tokens.size() == 7)
        {
            sensor_msgs::Imu msg;
            
            try
            {
                auto roll = stof(tokens[1]);
                auto pitch = stof(tokens[2]);
                auto yaw = stof(tokens[3]);
                
                msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
                
                msg.linear_acceleration.x = stof(tokens[4]);
                msg.linear_acceleration.y = stof(tokens[5]);
                msg.linear_acceleration.z = stof(tokens[6]);
                
                // Indicate that no angular velocities are available
                msg.angular_velocity_covariance[0] = -1;
            } catch(const invalid_argument &e) {
                ROS_ERROR_STREAM("Exception in parsing IMU message.");
                ROS_ERROR_STREAM(e.what());
            }
            
            pub.publish(msg);
            
        } else {
            ROS_WARN_STREAM("IMU received " << tokens.size() << " tokens. Expected 7.");
            ROS_WARN_STREAM(line);
        }
    }
    

    return 0;
}
