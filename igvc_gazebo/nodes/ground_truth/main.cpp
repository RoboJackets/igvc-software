#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <mutex>
#include <geometry_msgs/Vector3.h>

#include <tf/transform_datatypes.h>
#include <igvc_utils/NodeUtils.hpp>

ros::Publisher ground_truth_pub, diff_pub, angle_diff_pub;
std::mutex mutex;
nav_msgs::Odometry prev_gt;
nav_msgs::Odometry og_pose;

void state_estimate_callback(const nav_msgs::Odometry::ConstPtr& msg) {
  std::lock_guard<std::mutex> planning_lock(mutex);
  // finds and publishes diff
  nav_msgs::Odometry result;
  result.header = msg->header;
  result.pose.pose.position.x = msg->pose.pose.position.x - prev_gt.pose.pose.position.x;
  result.pose.pose.position.y = msg->pose.pose.position.y - prev_gt.pose.pose.position.y;
  result.pose.pose.position.z = msg->pose.pose.position.z - prev_gt.pose.pose.position.z;

  result.twist.twist.linear.x = msg->twist.twist.linear.x - prev_gt.twist.twist.linear.x;
  result.twist.twist.linear.y = msg->twist.twist.linear.y - prev_gt.twist.twist.linear.y;
  result.twist.twist.linear.z = msg->twist.twist.linear.z - prev_gt.twist.twist.linear.z;

  result.twist.twist.angular.x = msg->twist.twist.angular.x - prev_gt.twist.twist.angular.x;
  result.twist.twist.angular.y = msg->twist.twist.angular.y - prev_gt.twist.twist.angular.y;
  result.twist.twist.angular.z = msg->twist.twist.angular.z - prev_gt.twist.twist.angular.z;

  diff_pub.publish(result);

  //finds difference in rpy
  geometry_msgs::Vector3 angle_diff;
  double roll_gt, pitch_gt, yaw_gt, roll, pitch, yaw;
  tf::Quaternion quat_gt(prev_gt.pose.pose.orientation.x, prev_gt.pose.pose.orientation.y,
                         prev_gt.pose.pose.orientation.z, prev_gt.pose.pose.orientation.w);
  tf::Quaternion quat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

  tf::Matrix3x3(quat_gt).getRPY(roll_gt, pitch_gt, yaw_gt);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  angle_diff.x = roll - roll_gt;
  angle_diff.y = pitch - pitch_gt;
  angle_diff.z = yaw - yaw_gt;

  angle_diff_pub.publish(angle_diff);
}

void ground_truth_callback(const nav_msgs::Odometry::ConstPtr& msg) {
  std::lock_guard<std::mutex> planning_lock(mutex);
  if(msg->header.stamp.toSec() < 0.5) {
    og_pose.pose = msg->pose;
    og_pose.pose.pose.position.x = -msg->pose.pose.position.y;
    og_pose.pose.pose.position.y = msg->pose.pose.position.x;
  } else {

    nav_msgs::Odometry result;
    result.pose = msg->pose;
    // applying a transform here
    result.pose.pose.position.x = (-msg->pose.pose.position.y) - og_pose.pose.pose.position.x;
    result.pose.pose.position.y = msg->pose.pose.position.x - og_pose.pose.pose.position.y;
    result.twist = msg->twist;
    result.twist.twist.linear.x = -msg->twist.twist.linear.y;
    result.twist.twist.linear.y = msg->twist.twist.linear.x;
    result.header = msg->header;
    result.child_frame_id = "base_footprint";
    result.header.frame_id = "odom";


    tf::Quaternion quat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y);
    quat *= tf::createQuaternionFromRPY(0, 0, M_PI / 2);

    result.pose.pose.orientation.x = quat.x();
    result.pose.pose.orientation.y = quat.y();
    result.pose.pose.orientation.z = quat.z();
    result.pose.pose.orientation.w = quat.w();

    ground_truth_pub.publish(result);
    result = prev_gt;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ground_truth_republisher");

  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  std::string ground_truth_topic, estimate_topic, pub_topic, diff_topic;
  igvc::param(pNh, "ground_truth_sub_topic", ground_truth_topic, std::string("/ground_truth/state_raw"));
  igvc::param(pNh, "ground_truth_pub_topic", pub_topic, std::string("/ground_truth"));
  igvc::param(pNh, "diff_topic", diff_topic, std::string("/ground_truth/diff"));
  igvc::param(pNh, "estimate_topic", estimate_topic, std::string("/odometry/filtered"));

  ros::Subscriber ground_truth = nh.subscribe<nav_msgs::Odometry>(ground_truth_topic, 10,
                                                         ground_truth_callback);
  ros::Subscriber estimate = nh.subscribe<nav_msgs::Odometry>(estimate_topic, 10,
                                                              state_estimate_callback);

  ground_truth_pub = nh.advertise<nav_msgs::Odometry>(pub_topic, 1);
  diff_pub = nh.advertise<nav_msgs::Odometry>(diff_topic, 1);
  angle_diff_pub = nh.advertise<geometry_msgs::Vector3>(diff_topic+"/angle", 1);

  ros::spin();
}
