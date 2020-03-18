#ifndef SRC_TYPE_CONVERSIONS_H
#define SRC_TYPE_CONVERSIONS_H

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
// Pose2 == (Point3, Rot3)
#include <gtsam/geometry/Pose3.h>
#include <tf/transform_datatypes.h>

class Conversion
{
public:
  static gtsam::Pose3 getPose3FromOdom(const nav_msgs::Odometry &msg);
  static gtsam::Point3 getPoint3FromOdom(const nav_msgs::Odometry &msg);
  static nav_msgs::Odometry getOdomFromPose3(const gtsam::Pose3 &pos);
};

#endif  // SRC_TYPE_CONVERSIONS_H
