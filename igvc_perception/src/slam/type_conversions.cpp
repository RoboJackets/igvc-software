#include <slam/type_conversions.h>

gtsam::Pose3 Conversion::odomMsgToGtsamPose3(const nav_msgs::Odometry &msg)
{
  return gtsam::Pose3(gtsam::Rot3(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                  msg.pose.pose.orientation.z),
                      gtsam::Point3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z));
}

gtsam::Pose2 Conversion::odomMsgToGtsamPose2(const nav_msgs::Odometry &msg)
{
  gtsam::Rot3 twist = gtsam::Rot3(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                                  msg.pose.pose.orientation.z);

  return gtsam::Pose2(twist.yaw(), gtsam::Point2(msg.pose.pose.position.x, msg.pose.pose.position.y));
}

geometry_msgs::PoseWithCovariance Conversion::gtsamPose3ToPose3Msg(const gtsam::Pose3 &pos, const gtsam::Matrix &cov)
{
  geometry_msgs::PoseWithCovariance gPos;
  gPos.pose.position.x = pos.x();
  gPos.pose.position.y = pos.y();
  gPos.pose.position.z = pos.z();
  gtsam::Vector3 r = pos.rotation().xyz();
  gPos.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(r.x(), r.y(), r.z());

  gPos.covariance[0] = cov(0);
  gPos.covariance[1] = cov(1);
  gPos.covariance[2] = cov(2);
  gPos.covariance[3] = cov(3);
  gPos.covariance[4] = cov(4);
  gPos.covariance[5] = cov(5);
  gPos.covariance[6] = cov(6);
  gPos.covariance[7] = cov(7);
  gPos.covariance[8] = cov(8);
  gPos.covariance[9] = cov(9);
  gPos.covariance[10] = cov(10);
  gPos.covariance[11] = cov(11);
  gPos.covariance[12] = cov(12);
  gPos.covariance[13] = cov(13);
  gPos.covariance[14] = cov(14);
  gPos.covariance[15] = cov(15);
  gPos.covariance[16] = cov(16);
  gPos.covariance[17] = cov(17);
  gPos.covariance[18] = cov(18);
  gPos.covariance[19] = cov(19);
  gPos.covariance[20] = cov(20);
  gPos.covariance[21] = cov(21);
  gPos.covariance[22] = cov(22);
  gPos.covariance[23] = cov(23);
  gPos.covariance[24] = cov(24);
  gPos.covariance[25] = cov(25);
  gPos.covariance[26] = cov(26);
  gPos.covariance[27] = cov(27);
  gPos.covariance[28] = cov(28);
  gPos.covariance[29] = cov(29);
  gPos.covariance[30] = cov(30);
  gPos.covariance[31] = cov(31);
  gPos.covariance[32] = cov(32);
  gPos.covariance[33] = cov(33);
  gPos.covariance[34] = cov(34);
  gPos.covariance[35] = cov(35);

  return gPos;
}

geometry_msgs::Vector3 Conversion::gtsamVector3ToVector3Msg(const gtsam::Vector3 &vec)
{
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