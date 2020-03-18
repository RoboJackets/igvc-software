#include <slam/type_conversions.h>

gtsam::Pose3 Conversion::getPose3FromOdom(const nav_msgs::Odometry &msg)
{
  return gtsam::Pose3(gtsam::Rot3(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                  msg.pose.pose.orientation.z),
                      gtsam::Point3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z));
}

nav_msgs::Odometry Conversion::getOdomFromPose3(const gtsam::Pose3 &pos)
{
  nav_msgs::Odometry msg;
  msg.child_frame_id = "/base_footprint";
  msg.header.frame_id = "/odom";
  msg.pose.pose.position.x = pos.x();
  msg.pose.pose.position.y = pos.y();
  msg.pose.pose.position.z = pos.z();
  gtsam::Vector3 r = pos.rotation().xyz();
  msg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(r.x(), r.y(), r.z());
  return msg;
}

gtsam::Point3 Conversion::getPoint3FromOdom(const nav_msgs::Odometry &msg)
{
  return gtsam::Point3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
}