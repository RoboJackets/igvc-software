#ifndef GROUND_TRUTH_REPUBLISHER
#define GROUND_TRUTH_REPUBLISHER

#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <parameter_assertions/assertions.h>
#include <robot_localization/navsat_conversions.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <mutex>

class ground_truth
{
public: 
	explicit ground_truth();
private: 
	//handlers
	ros::NodeHandle nh;
  	ros::NodeHandle pNh;

  	//PARAMETERS

  	//publishers and subscriberstf::TransformListener tf_listener
	ros::Publisher ground_truth_pub;
	ros::Subscriber ground_truth_sub;
	ros:: Subscriber estimate_sub;

	//listeners and broadcasters 
	tf::TransformListener tf_listener;
	tf::TransformBroadcaster br;
	
	//pose and estimate
  	nav_msgs::Odometry og_pose;
	ros::Time last_estimate;

	//topics 
	std::string sub_topic;
	std::string estimate_topic;
	std::string pub_topic;
	std::string diff_topic;

	//other attributes 
	double longitude;
	double latitude;
	double utm_x;
	double utm_y;
	tf::Transform utm_to_odom;
	ros::Timer utm_timer;

	bool utm_enabled;
	//disables ground truth utm_to_odom tf broadcast(utm_callback) if utm_enabled is false

	//callbacks 
	void odomCallback(const nav_msgs::Odometry::ConstPtr msg);
	void groundTruthCallback(const nav_msgs::Odometry::ConstPtr msg);
	void utm_callback(const ros::TimerEvent& event, const tf::Transform& odom_to_utm);
};
#endif
