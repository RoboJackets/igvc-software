#include <slam/type_conversions.h>

gtsam::Pose3 Conversion::odomMsgToGtsamPose3(const nav_msgs::Odometry &msg)
{
  return gtsam::Pose3(gtsam::Rot3(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                  msg.pose.pose.orientation.z),
                      gtsam::Point3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z));
}

geometry_msgs::Pose Conversion::gtsamPose3ToPose3Msg(const gtsam::Pose3 &pos){
  geometry_msgs::Pose gPos;
  gPos.position.x = pos.x();
  gPos.position.y = pos.y();
  gPos.position.z = pos.z();
  gtsam::Vector3 r = pos.rotation().xyz();
  gPos.orientation = tf::createQuaternionMsgFromRollPitchYaw(r.x(), r.y(), r.z());
  return gPos;
}

geometry_msgs::Vector3 Conversion::gtsamVector3ToVector3Msg(const gtsam::Vector3 &vec){
  geometry_msgs::Vector3 gVec;
  gVec.x = vec.x();
  gVec.y = vec.y();
  gVec.z = vec.z();
  return gVec;
}

gtsam::Point3 Conversion::odomMsgToGtsamPoint3(const nav_msgs::Odometry &msg)
{
  return gtsam::Point3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
}