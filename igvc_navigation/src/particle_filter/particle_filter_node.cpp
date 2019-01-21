#include <igvc_msgs/velocity_pair.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <signal.h>
#include <tf/transform_listener.h>
#include <boost/circular_buffer.hpp>
#include <igvc_utils/RobotState.hpp>
#include <pcl_ros/transforms.h>
#include "particle_filter.h"

class ParticleFilterNode
{
public:
  ParticleFilterNode(double transform_max_wait_time, double update_time_thresh, double start_x, double start_y, double start_z, bool debug)
    : m_debug(debug), m_transform_max_wait_time(transform_max_wait_time), m_update_time_thresh(update_time_thresh), m_start_x(start_x), m_start_y(start_y), m_start_z(start_z)
  {
  }

  void pc_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc);
  void pose_callback(const nav_msgs::OdometryConstPtr& pose);

  std::unique_ptr<tf::TransformListener> m_tf_listener;
  std::unique_ptr<Particle_filter> m_particle_filter;
  ros::Publisher m_particle_pcl_pub;
  std::string m_base_frame, m_lidar_frame;
  std::string m_lidar_topic, m_fused_topic, m_odom_frame;

private:
  void check_update();
  void update(int pose_idx, int pc_idx);
  boost::shared_ptr<tf::Transform> get_lidar_transform();
  bool m_debug;
  double m_transform_max_wait_time, m_update_time_thresh, m_start_x, m_start_y, m_start_z;
  boost::shared_ptr<tf::Pose> m_last_pose;
  boost::circular_buffer<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> m_pc_buf;
  boost::circular_buffer<nav_msgs::OdometryConstPtr> m_pose_buf;
  boost::shared_ptr<tf::StampedTransform> m_lidar_transform;
};

std::unique_ptr<ParticleFilterNode> pf_node;

void node_cleanup(int sig)
{
  pf_node.reset();
  ros::shutdown();
}

void ParticleFilterNode::pc_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc)
{
  m_pc_buf.push_back(pc);
  check_update();
}

void ParticleFilterNode::pose_callback(const nav_msgs::OdometryConstPtr& pose)
{
  m_pose_buf.push_back(pose);
  check_update();
}

/**
 * Compares m_pose_buf and m_pc_buf; Will update with every lidar, but needs to match the correct pose with it
 */
void ParticleFilterNode::check_update()
{
  // Needs both buffers to have at least one element
  if (m_pc_buf.empty() || m_pose_buf.empty())
  {
    return;
  }

  ros::Time pc_stamp;
  pcl_conversions::fromPCL(m_pc_buf.front()->header.stamp, pc_stamp);

  // If pose stamp > pc_stamp, then update, since the difference will only get larger,
  // and don't want to dump lidar msgs ?? Do we??
  for (int i = 0; i < m_pose_buf.size(); ++i)
  {
    if (pc_stamp - m_pose_buf[i]->header.stamp < ros::Duration(m_update_time_thresh))
    {
      update(i, 0);
      return;
    }
    else if (m_pose_buf[i]->header.stamp > pc_stamp)
    {
      update(i, 0);
      return;
    }
  }
  // Didn't find any good matches
  //
  //  tf::StampedTransform transform;
  //  if (tf_listener->waitForTransform("/odom", "/base_link", stamp, ros::Duration(m_transform_max_wait_time)))
  //  {
  //    tf_listener->lookupTransform("/odom", "/base_link", stamp, transform);
  //  } else {
  //    // TODO: How to handle if not found?
  //    ros::Time now = ros::Time::now();
  //    ROS_ERROR_STREAM("Failed lookupTransform for time " << stamp << ", using current time: " << now);
  //    try
  //    {
  //      tf_listener->lookupTransform("/odom", "/base_link", now, transform);
  //    } catch (tf::TransformException& ex)
  //    {
  //      ROS_ERROR("%s", ex.what());
  //      return;
  //    }
  //  }
  //
  //  // Pass both to particle filter to handle
  //  particle_filter->update(pc, transform);
}

/**
 * Performs one iteration of particle filter using the indices passed
 * @param pose_idx
 * @param pc_idx
 */
void ParticleFilterNode::update(int pose_idx, int pc_idx)
{
  // Get pose difference
  if (m_last_pose == nullptr)
  {
    m_last_pose = boost::make_shared<tf::Stamped<tf::Pose>>(m_start_x, m_start_y, m_start_z);
  }
  tf::Stamped<tf::Pose> cur_pose;
  tf::poseMsgToTF(m_pose_buf[pose_idx]->pose.pose, cur_pose);
  tf::Transform diff = m_last_pose->inverse() * cur_pose;

  // TODO: Maybe move this to a lidar filtering node?
  // Get static transform from lidar frame to base frame
  boost::shared_ptr<tf::Transform> lidar_to_base = get_lidar_transform();

  // Transform cloud to base frame
  pcl::PointCloud<pcl::PointXYZ> transformed_pc;
  pcl_ros::transformPointCloud(*m_pc_buf[pc_idx], transformed_pc, *m_lidar_transform);

  //TODO: Do I need to subtract covariances?
  m_particle_filter->update(diff, m_pose_buf[pose_idx]->pose.covariance, transformed_pc, *lidar_to_base);

  // Delete buffer till index
  m_pose_buf.erase_begin(pose_idx + 1);
  m_pc_buf.erase_begin(pc_idx + 1);
}

boost::shared_ptr<tf::Transform> ParticleFilterNode::get_lidar_transform()
{
  if(m_lidar_transform == nullptr)
  {
    if (m_tf_listener->waitForTransform(m_base_frame, m_lidar_frame, ros::Time::now(), ros::Duration(0)))
    {
      ROS_INFO_STREAM("Getting static transform from " << m_lidar_frame << " to " << m_base_frame);
      m_tf_listener->lookupTransform(m_base_frame, m_lidar_frame, ros::Time::now(), *m_lidar_transform);
    } else {
      ROS_ERROR_STREAM("Error getting static transform from " << m_lidar_frame << " to " << m_base_frame);
    }
  }
  return m_lidar_transform;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "particle_filter");
  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  signal(SIGINT, node_cleanup);

  double transform_max_wait_time, update_time_threshold;
  double start_x, start_y, start_z;
  bool debug;
  igvc::getParam(pNh, "transform_max_wait_time", transform_max_wait_time);
  igvc::getParam(pNh, "update_time_threshold", update_time_threshold);
  igvc::param(pNh, "debug", debug, false);

  pf_node = std::unique_ptr<ParticleFilterNode>(
      new ParticleFilterNode(transform_max_wait_time, update_time_threshold, start_x, start_y, start_z, debug));
  igvc::getParam(pNh, "lidar_topic", pf_node->m_lidar_topic);
  igvc::getParam(pNh, "fused_topic", pf_node->m_fused_topic);
  igvc::getParam(pNh, "odometry_frame", pf_node->m_odom_frame);
  igvc::getParam(pNh, "base_frame", pf_node->m_base_frame);
  igvc::getParam(pNh, "lidar_frame", pf_node->m_lidar_frame);

  pf_node->m_particle_filter = std::unique_ptr<Particle_filter>(new Particle_filter(pNh));
  pf_node->m_tf_listener = std::unique_ptr<tf::TransformListener>(new tf::TransformListener());

  ros::Subscriber pc_sub = nh.subscribe(pf_node->m_lidar_topic, 1, &ParticleFilterNode::pc_callback, pf_node.get());
  ros::Subscriber pose_sub = nh.subscribe(pf_node->m_fused_topic, 1, &ParticleFilterNode::pose_callback, pf_node.get());

  if (debug)
  {
    pf_node->m_particle_pcl_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/particles", 1);
  }
  ros::spin();

  return 0;
}
