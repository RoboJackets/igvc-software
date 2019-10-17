#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>


geometry_msgs::PoseWithCovariance prev_odom;
bool odom_initialized = false;

void OdomCallback(geometry_msgs::PoseWithCovariance msg)
{
    // Insuring prev_odom is initialized
    if(!odom_initialized) {
        prev_odom = msg;
        odom_initialized = true;
    }



}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slam");
    ros::NodeHandle pnh {"~"};
    ros::Subscriber odom_sub = pnh.subscribe("/wheel_odom", 1, OdomCallback);
    
    ros::spin();
}


