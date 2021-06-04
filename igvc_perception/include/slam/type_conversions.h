#ifndef SRC_TYPE_CONVERSIONS_H
#define SRC_TYPE_CONVERSIONS_H

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
// Pose2 == (Point3, Rot3)
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>
#include <tf/transform_datatypes.h>

class Conversion
{
public:
  static gtsam::Pose3 odomMsgToGtsamPose3(const nav_msgs::Odometry &msg);
  static gtsam::Pose2 odomMsgToGtsamPose2(const nav_msgs::Odometry &msg);
  static gtsam::Point3 odomMsgToGtsamPoint3(const nav_msgs::Odometry &msg);
  static geometry_msgs::Vector3 gtsamVector3ToVector3Msg(const gtsam::Vector3 &vec);
  static geometry_msgs::Pose gtsamPose3ToPose3Msg(const gtsam::Pose3 &pos);
};

#endif  // SRC_TYPE_CONVERSIONS_H
