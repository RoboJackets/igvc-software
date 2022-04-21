#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <parameter_assertions/assertions.h>
#include <robot_localization/navsat_conversions.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <random>


ros::Publisher g_ground_truth_pub;
// TODO make this a minimal object
nav_msgs::Odometry g_og_pose;
ros::Time g_last_estimate;
double x_noise_std_dev = 0.0;
double y_noise_std_dev = 0.0;
double z_noise_std_dev = 0.0;
double roll_noise_std_dev = 0.0;
double pitch_noise_std_dev = 0.0;
double yaw_noise_std_dev = 0.0;
std::default_random_engine engine(std::random_device{}());
std::normal_distribution<double> x_distribution;
std::normal_distribution<double> y_distribution;
std::normal_distribution<double> z_distribution;
std::normal_distribution<double> roll_distribution;
std::normal_distribution<double> pitch_distribution;
std::normal_distribution<double> yaw_distribution;
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  g_last_estimate = msg->header.stamp;
}

void groundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  // get the starting location as the origin
  if (g_og_pose.header.stamp.toSec() == 0)
  {
    g_og_pose.pose = msg->pose;
    g_og_pose.header = msg->header;
    g_og_pose.pose.pose.position.x = msg->pose.pose.position.x + x_distribution(engine);
    g_og_pose.pose.pose.position.y = msg->pose.pose.position.y + y_distribution(engine);
    g_og_pose.pose.pose.position.z = msg->pose.pose.position.z + z_distribution(engine);
    ROS_INFO_STREAM("setting g_og_pose to " << g_og_pose.pose.pose.position.x << ", "
                                            << g_og_pose.pose.pose.position.y << ", "
                                            << g_og_pose.pose.pose.position.z);
  }
  else
  {
    nav_msgs::Odometry result;
    result.pose = msg->pose;

    // use the initial location as an offset (makes the starting location 0, 0)
    result.pose.pose.position.x = msg->pose.pose.position.x - g_og_pose.pose.pose.position.x + x_distribution(engine);
    result.pose.pose.position.y = msg->pose.pose.position.y - g_og_pose.pose.pose.position.y + y_distribution(engine);
    result.pose.pose.position.z = msg->pose.pose.position.z - g_og_pose.pose.pose.position.z + z_distribution(engine);
    result.twist = msg->twist;

    // set up the correct header
    result.header = msg->header;
    result.child_frame_id = "base_footprint";
    result.header.frame_id = "odom";

    tf::Quaternion quat(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    roll = roll + roll_distribution(engine);
    pitch = pitch + pitch_distribution(engine);
    yaw = yaw + yaw_distribution(engine);
    quat = tf::createQuaternionFromRPY(roll,pitch,yaw);

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
    br.sendTransform(tf::StampedTransform(odom_to_utm, event.current_real, "odom", "utm"));
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ground_truth_republisher");

  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  assertions::param(pNh, "x_noise_std_dev", x_noise_std_dev, 0.0);
  assertions::param(pNh, "y_noise_std_dev", y_noise_std_dev, 0.0);
  assertions::param(pNh, "z_noise_std_dev", z_noise_std_dev, 0.0);
  assertions::param(pNh, "roll_noise_std_dev", roll_noise_std_dev, 0.0);
  assertions::param(pNh, "pitch_noise_std_dev", pitch_noise_std_dev, 0.0);
  assertions::param(pNh, "yaw_noise_std_dev", yaw_noise_std_dev, 0.0);

  x_distribution = std::normal_distribution<double>(0,x_noise_std_dev);
  y_distribution = std::normal_distribution<double>(0,y_noise_std_dev);
  z_distribution = std::normal_distribution<double>(0,z_noise_std_dev);
  roll_distribution = std::normal_distribution<double>(0,roll_noise_std_dev);
  pitch_distribution = std::normal_distribution<double>(0,pitch_noise_std_dev);
  yaw_distribution = std::normal_distribution<double>(0,yaw_noise_std_dev);


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
