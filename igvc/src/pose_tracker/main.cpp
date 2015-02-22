#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <memory>
#include <iostream>

using namespace tf;
using namespace std;

ros::Publisher pose_pub;
std::unique_ptr<TransformBroadcaster> tf_broadcaster;
std::unique_ptr<TransformListener> tf_listener;

sensor_msgs::Imu imu;
nav_msgs::Odometry gps;

geometry_msgs::Point origin;
bool first = true;

bool imuIsValid()
{
    return imu.orientation.w != 0 ||
           imu.orientation.x != 0 ||
           imu.orientation.y != 0 ||
           imu.orientation.z != 0;
}

void publish_pose()
{
    if(!imuIsValid())
        return;
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "/base_footprint";
    pose.pose.position.x = gps.pose.pose.position.x;
    pose.pose.position.y = gps.pose.pose.position.y;
    pose.pose.orientation = imu.orientation;
    pose_pub.publish(pose);

    Transform transform;
    transform.setOrigin(Vector3(pose.pose.position.x, pose.pose.position.y, 0.0));
    Quaternion q;
    quaternionMsgToTF(pose.pose.orientation, q);
    transform.setRotation(q);
    tf_broadcaster->sendTransform(StampedTransform(transform, ros::Time::now(), "map", "base_footprint"));

    ROS_INFO_STREAM("POSE " << pose.pose.position.x << " " << pose.pose.position.y);
}

void imu_callback(const sensor_msgs::ImuConstPtr& msg)
{
    imu = *msg;
    geometry_msgs::QuaternionStamped q;
    q.header.stamp = ros::Time(0);
    q.header.frame_id = imu.header.frame_id;
    q.quaternion = imu.orientation;
    tf_listener->waitForTransform("base_footprint", imu.header.frame_id, ros::Time(0), ros::Duration(10.0));
    geometry_msgs::QuaternionStamped transformed;
    tf_listener->transformQuaternion("base_footprint", q, transformed);
    imu.orientation = transformed.quaternion;

    publish_pose();
}

void gps_callback(const nav_msgs::OdometryConstPtr& msg)
{
    gps = *msg;
    if(first)
    {
        origin = gps.pose.pose.position;
        first = false;
    } else {
        gps.pose.pose.position.x -= origin.x;
        gps.pose.pose.position.y -= origin.y;
        gps.pose.pose.position.z -= origin.z;
    }
    publish_pose();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_tracker");

    tf_broadcaster = std::unique_ptr<TransformBroadcaster>(new TransformBroadcaster);
    
    tf_listener = std::unique_ptr<TransformListener>(new TransformListener);

    ros::NodeHandle nh;

    ros::Subscriber imu_sub = nh.subscribe("/imu", 1, imu_callback);

    ros::Subscriber gps_sub = nh.subscribe("/vo", 1, gps_callback);

    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/odom_combined", 1);

    ros::spin();

    return 0;
}
