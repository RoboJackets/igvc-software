#ifndef GROUND_TRUTH_REPUBLISHER_H
#define GROUND_TRUTH_REPUBLISHER_H

#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <robot_localization/navsat_conversions.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

class GroundTruthRepublisher
{
public:
  GroundTruthRepublisher();

private:
  ros::Publisher g_ground_truth_pub;
  nav_msgs::Odometry g_og_pose;
  ros::Time g_last_estimate;

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void groundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void utm_callback(const ros::TimerEvent& event, const tf::Transform& odom_to_utm);
};

#endif