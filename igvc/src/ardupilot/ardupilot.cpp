#include <ros/ros.h>
#include <ros/publisher.h>
#include <sensor_msgs/Imu.h>
#include <igvc/SerialPort.h>
#include <vector>
#include <sstream>
#include <tf/transform_datatypes.h>
#include <igvc/StringUtils.hpp>

#define GRAVITY_MSS 9.80665
#define DEG_TO_RAD (3.14159265/180.0)

using namespace std;

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
        
        if(tokens.size() == 10)
        {
            sensor_msgs::Imu msg;
            
            msg.header.frame_id = "imu";
            msg.header.stamp = ros::Time::now();
            
            try
            {
                auto roll = stof(tokens[1]) * DEG_TO_RAD;
                auto pitch = stof(tokens[2]) * DEG_TO_RAD;
                auto yaw = stof(tokens[3]) * DEG_TO_RAD;
                
                /* These arguments are intentionally out of order, because TF issues further upstream
                 * have not been fixed. See GitHub issue #35.
                 */
                msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(pitch, roll, -yaw);
                
                msg.linear_acceleration.x = stof(tokens[4]);
                msg.linear_acceleration.y = -stof(tokens[5]);
                msg.linear_acceleration.z = stof(tokens[6]);

                msg.angular_velocity.x = stof(tokens[7]);
                msg.angular_velocity.y = stof(tokens[8]);
                msg.angular_velocity.z = stof(tokens[9]);

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
