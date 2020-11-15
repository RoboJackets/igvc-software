// GroundTruth.h
#ifndef GroundTruth_H
#define GroundTruth_H

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>


class GroundTruth 
{
public:
	ros::Publisher g_ground_truth_pub;
	nav_msgs::Odometry g_og_pose;
	ros::Time g_last_estimate;
	GroundTruth::GroundTruth();
private:
	void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void groundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg);
	void utm_callback(const ros::TimerEvent& event, const tf::Transform& odom_to_utm);

};
#endif
