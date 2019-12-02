#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <parameter_assertions/assertions.h>
#include <robot_localization/navsat_conversions.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <mutex>
#include <functional>
#include <random>
#include <iterator>

ros::Publisher g_ground_truth_pub;
// TODO make this a minimal object
nav_msgs::Odometry g_og_pose;
ros::Time g_last_estimate;
double noise;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  g_last_estimate = msg->header.stamp;
}

void groundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  //random noise generator
  auto dist = std::bind(std::normal_distribution<double>{0.0, noise},
                   std::mt19937(std::random_device{}()));
  // get the starting location as the origin
  if (g_og_pose.header.stamp.toSec() == 0)
  { 
    g_og_pose.pose = msg->pose;
    g_og_pose.header = msg->header;
    g_og_pose.pose.pose.position.x = msg->pose.pose.position.x + dist();
    g_og_pose.pose.pose.position.y = msg->pose.pose.position.y + dist();
    ROS_INFO_STREAM("setting g_og_pose to " << g_og_pose.pose.pose.position.x << ", "
                                            << g_og_pose.pose.pose.position.y);
  }
  else
  {
    nav_msgs::Odometry result;
    result.pose = msg->pose;

    // use the initial location as an offset (makes the starting location 0, 0)
    result.pose.pose.position.x = msg->pose.pose.position.x - g_og_pose.pose.pose.position.x + dist();
    result.pose.pose.position.y = msg->pose.pose.position.y - g_og_pose.pose.pose.position.y + dist();

    result.twist = msg->twist;

    // set up the correct header
    result.header = msg->header;
    result.child_frame_id = "/base_footprint";
    result.header.frame_id = "/odom";

    tf::Quaternion quat;
    tf::Vector3 pos;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    tf::pointMsgToTF(result.pose.pose.position, pos);

    // publish odom message
    g_ground_truth_pub.publish(result);

    // publish transform for tf if there has not been a update from the localization node in the last second
    // since it also publishes the same transform
    if (std::abs(msg->header.stamp.toSec() - g_last_estimate.toSec()) > 1.0)
    {
      static tf::TransformBroadcaster br;
      tf::Transform transform{ quat, pos };
      br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "odom", "base_footprint"));

      tf::Transform utm_to_odom;
    }
  }
}

void utm_callback(const ros::TimerEvent& event, const tf::Transform& odom_to_utm)
{
  static tf::TransformBroadcaster br;
  static tf::TransformListener tf_listener;
  tf::StampedTransform transform;
  static bool enabled = true;

  if (enabled)
  {
    bool found = true;
    try
    {
      tf_listener.lookupTransform("odom", "utm", ros::Time(0), transform);
    }
    catch (const tf::TransformException& ex)
    {
      found = false;
    }

    if (found && transform.getRotation() != odom_to_utm.getRotation() &&
        transform.getOrigin() != odom_to_utm.getOrigin())
    {
      ROS_WARN_STREAM("Anther odom -> utm tf broadcast detected. Disabling ground_truth odom -> utm tf broadcast.");
      enabled = false;
      return;
    }
    br.sendTransform(tf::StampedTransform(odom_to_utm, ros::Time(0), "odom", "utm"));
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ground_truth_republisher");

  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  nh.getParam("/noise",noise);

  std::string ground_truth_topic, estimate_topic, pub_topic, diff_topic;

  assertions::param(pNh, "ground_truth_sub_topic", ground_truth_topic, std::string("/ground_truth/state_raw"));
  assertions::param(pNh, "ground_truth_pub_topic", pub_topic, std::string("/ground_truth"));

  double longitude, latitude;
  assertions::param(pNh, "longitude", longitude, -84.405001);
  assertions::param(pNh, "latitude", latitude, 33.774497);

  ros::Subscriber ground_truth = nh.subscribe<nav_msgs::Odometry>(ground_truth_topic, 10, groundTruthCallback);

  ros::Subscriber estimate_sub = nh.subscribe<nav_msgs::Odometry>("/odometry/filtered", 1, odomCallback);
  g_ground_truth_pub = nh.advertise<nav_msgs::Odometry>(pub_topic, 1);

  double utm_x, utm_y;
  RobotLocalization::NavsatConversions::UTM(latitude, longitude, &utm_x, &utm_y);

  tf::Transform utm_to_odom;
  utm_to_odom.setOrigin(
      tf::Vector3(utm_x - g_og_pose.pose.pose.position.x, utm_y - g_og_pose.pose.pose.position.y, 0.0));
  utm_to_odom.setRotation(tf::createQuaternionFromYaw(M_PI));

  ros::Timer utm_timer = nh.createTimer(ros::Duration(1.0), boost::bind(utm_callback, _1, utm_to_odom.inverse()));
  ros::spin();
}
