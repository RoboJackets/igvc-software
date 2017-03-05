#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <memory>

ros::Publisher pose_pub;
ros::Publisher origin_pub;
std::unique_ptr<tf::TransformBroadcaster> tf_broadcaster;
std::unique_ptr<tf::TransformListener> tf_listener;

geometry_msgs::Quaternion orientation;
nav_msgs::Odometry gps;

geometry_msgs::Point origin;
bool first = true;

bool orientationIsValid()
{
  return orientation.w != 0 || orientation.x != 0 || orientation.y != 0 || orientation.z != 0;
}

void publish_pose()
{
  if (!orientationIsValid())
    return;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "/map";
  pose.pose.position.x = gps.pose.pose.position.x;
  pose.pose.position.y = gps.pose.pose.position.y;
  pose.pose.position.z = 0.0;
  pose.pose.orientation = orientation;
  pose_pub.publish(pose);

  tf::Transform transform;
  transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, 0.0));
  tf::Quaternion q;
  tf::quaternionMsgToTF(orientation, q);
  transform.setRotation(q);
  tf_broadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_footprint"));
}

void imu_callback(const sensor_msgs::ImuConstPtr& msg)
{
  geometry_msgs::QuaternionStamped q;
  q.header.stamp = ros::Time(0);
  q.header.frame_id = msg->header.frame_id;
  q.quaternion = msg->orientation;
  tf_listener->waitForTransform(msg->header.frame_id, "base_footprint", ros::Time(0), ros::Duration(10.0));
  geometry_msgs::QuaternionStamped transformed;
  tf_listener->transformQuaternion("base_footprint", q, transformed);
  orientation = transformed.quaternion;
}

void gps_callback(const nav_msgs::OdometryConstPtr& msg)
{
  gps = *msg;

  tf_listener->waitForTransform("base_footprint", msg->header.frame_id, ros::Time(0), ros::Duration(10.0));
  geometry_msgs::PoseStamped p;
  p.header.stamp = ros::Time(0);
  p.header.frame_id = msg->header.frame_id;
  p.pose = msg->pose.pose;
  geometry_msgs::PoseStamped transformed;
  tf_listener->transformPose("base_footprint", p, transformed);
  gps.pose.pose = transformed.pose;

  if (first)  // TODO this is nonsense, with GPS noise this will become inaccurate
  {
    origin = gps.pose.pose.position;
    first = false;
  }
  else
  {
    gps.pose.pose.position.x -= origin.x;
    gps.pose.pose.position.y -= origin.y;
    gps.pose.pose.position.z -= origin.z;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_tracker");

  tf_broadcaster = std::unique_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster);

  tf_listener = std::unique_ptr<tf::TransformListener>(new tf::TransformListener);

  ros::NodeHandle nh;

  ros::Subscriber imu_sub = nh.subscribe("/imu", 1, imu_callback);

  ros::Subscriber gps_sub = nh.subscribe("/gps_odom", 1, gps_callback);

  pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/odom_combined", 1);

  origin_pub = nh.advertise<geometry_msgs::PointStamped>("/map_origin", 1);

  // Publish initial pose
  orientation.x = 0;
  orientation.y = 0;
  orientation.z = 1;
  orientation.w = 0;
  publish_pose();

  ros::Rate rate(20);  // 20 Hz
  while (ros::ok())
  {
    ros::spinOnce();

    geometry_msgs::PointStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "/world";
    msg.point = origin;
    origin_pub.publish(msg);

    publish_pose();

    rate.sleep();
  }

  return 0;
}
