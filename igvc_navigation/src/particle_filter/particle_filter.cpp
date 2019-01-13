#include <igvc_msgs/velocity_pair.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <boost/circular_buffer.hpp>
#include <igvc_utils/RobotState.hpp>
#include "basic_particle_filter.h"
#include "particle_filter_base.h"

class ParticleFilterNode
{
public:
  void motor_callback(igvc_msgs::velocity_pairConstPtr motor_command);
  void pc_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pointCloud);
  void map_callback(igvc_msgs::mapConstPtr map);

  double transform_max_wait_time;
  boost::circular_buffer<igvc_msgs::velocity_pairConstPtr> delta_buffer;
  boost::circular_buffer<igvc_msgs::mapConstPtr> map_buffer;
  std::unique_ptr<tf::TransformListener> tf_listener;
  std::unique_ptr<ParticleFilterBase> particle_filter;
  ros::Publisher particle_pcl_pub;

  ParticleFilterNode(int delta_buffer_size, int map_buffer_size, double transform_max_wait_time)
    : transform_max_wait_time(transform_max_wait_time), delta_buffer(delta_buffer_size), map_buffer(map_buffer_size)
  {
  }

private:
  boost::circular_buffer<igvc_msgs::velocity_pairConstPtr>::iterator find_closest(const ros::Time& stamp,
                                                               boost::circular_buffer<igvc_msgs::velocity_pairConstPtr> buffer);
  boost::circular_buffer<igvc_msgs::mapConstPtr>::iterator
  find_closest(const ros::Time& stamp, boost::circular_buffer<igvc_msgs::mapConstPtr> buffer);
  void check_synchronize();
  void publish_map(const ros::Time& stamp);
  void publish_map(uint64_t stamp);
};

std::unique_ptr<ParticleFilterNode> particle_filter_node;
bool DEBUG = false;

/**
 * Callback for motor commands. Calls `particle_filter->ProposalDistribution` to get state deltas, and adds them to
 * the delta_buffer
 * @param motor_command motor commands from encoders
 */
void ParticleFilterNode::motor_callback(igvc_msgs::velocity_pairConstPtr motor_command)
{
  // ROS_INFO("motor callback");
  tf::StampedTransform transform;
  if (tf_listener->waitForTransform("/odom", "/base_link", motor_command->header.stamp,
                                    ros::Duration(transform_max_wait_time)))
  {
    tf_listener->lookupTransform("/odom", "/base_link", motor_command->header.stamp, transform);
  }
  else
  {
    // TODO: How to handle if doesn't find it?
    // Currently will just get the latest one, shouldn't make that large of an error?
    ROS_ERROR_STREAM("Failed lookupTransform, using latest one");
    try
    {
      tf_listener->lookupTransform("/odom", "/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("%s", ex.what());
      return;
    }
  }
  RobotState state(transform, motor_command->header.stamp);
  // particle_filter_node->particle_filter->ProposalDistribution(state, motor_command);
  delta_buffer.push_back(motor_command);
  // ROS_INFO("end motor callback");
  if (DEBUG)
  {
    publish_map(motor_command->header.stamp);
  }
}

void ParticleFilterNode::map_callback(const igvc_msgs::mapConstPtr map)
{
  map_buffer.push_back(map);
}

// TODO: Add a map callback to update the maps in the particles

/**
 * Callback for lidar data. Finds corresponding delta in `delta_buffer`, consumes all deltas to propogate particles,
 * calculates weights, then resamples the particles.
 *
 * @param pointCloud pointcloud from lidar callback
 */
void ParticleFilterNode::pc_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pointcloud)
{
  if (delta_buffer.empty())
  {
    ROS_ERROR_STREAM("Couldn't find deltas");
    return;  // No deltas, rip
  }
  ros::Time stamp;
  pcl_conversions::fromPCL(pointcloud->header.stamp, stamp);
  boost::circular_buffer<igvc_msgs::velocity_pairConstPtr>::iterator delta_end_it = find_closest(stamp, delta_buffer);

  // ROS_INFO_STREAM("---------------------------------------------------------");
  // ROS_INFO_STREAM("Num points in delta_buffer: " << delta_end_it);
  // Propogate the particles using the delta
  particle_filter_node->particle_filter->propagateParticles(delta_buffer.begin(), delta_end_it);

  // Repeat above for map LOL
  boost::circular_buffer<igvc_msgs::mapConstPtr>::iterator map_end_it = find_closest(stamp, map_buffer);
  if (map_buffer.empty())
  {
    ROS_ERROR_STREAM("Couldn't find map");
    return;  // No map rip
  }
  igvc_msgs::mapConstPtr map_ptr;
  if (map_end_it == map_buffer.end())
  {
    // ROS_ERROR_STREAM("Using last value");
    map_ptr = map_buffer.back();
  }
  else
  {
    map_ptr = *map_end_it;
  }
  // ROS_INFO_STREAM("Matching pcl: " << std::setprecision(4) << stamp);
  // ROS_INFO_STREAM("With deltas : " << std::setprecision(4) << delta_buffer[delta_end_it].stamp);
  // ROS_INFO_STREAM("With map    : " << std::setprecision(4) << map_buffer[map_end_pos]->header.stamp);
  // ROS_INFO_STREAM("pcl - map   :" << std::setprecision(8) << stamp - map_ptr->header.stamp);
  igvc_msgs::velocity_pairConstPtr velpair;
  if (delta_end_it == delta_buffer.end())
  {
    velpair = delta_buffer.back();
  }
  else
  {
    velpair = *delta_end_it;
  }
  // ROS_INFO_STREAM("pcl - delta :" << std::setprecision(8) << stamp - velpair.first->header.stamp);
  // ROS_INFO_STREAM("---------------------------------------------------------");
  // Compute weights using propagated particles
  particle_filter_node->particle_filter->getWeights(pointcloud, map_ptr);
  delta_buffer.erase(delta_buffer.begin(), delta_end_it);
  // Erase from start to iterator
  map_buffer.erase(map_buffer.begin(), map_end_it);

  // Resample
  particle_filter_node->particle_filter->resample_points();

  if (DEBUG)
  {
    publish_map(pointcloud->header.stamp);
  }
}

void ParticleFilterNode::publish_map(const ros::Time& ros_stamp)
{
  uint64_t pcl_stamp;
  pcl_conversions::toPCL(ros_stamp, pcl_stamp);
  publish_map(pcl_stamp);
}
void ParticleFilterNode::publish_map(uint64_t pcl_stamp)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr particle_pcl =
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
  for (Particle particle : particle_filter_node->particle_filter->particles)
  {
    pcl::PointXYZRGB p(255, 255, 255);
    p.x = static_cast<float>(particle.state.x);
    p.y = static_cast<float>(particle.state.y);
    particle_pcl->points.emplace_back(p);
  }
  particle_pcl->header.frame_id = "/odom";
  particle_pcl->header.stamp = pcl_stamp;
  particle_pcl_pub.publish(particle_pcl);
}

struct CompareTime
{
  ros::Time asTime(const igvc_msgs::velocity_pairConstPtr vel) const
  {
    return vel->header.stamp;
  }

  ros::Time asTime(const igvc_msgs::mapConstPtr& map) const
  {
    return map->header.stamp;
  }

  ros::Time asTime(const ros::Time& stamp) const
  {
    return stamp;
  }

  template <typename T1, typename T2>
  bool operator()(T1 const& t1, T2 const& t2) const
  {
    return asTime(t1) < asTime(t2);
  }
};

// Should I be returning iterator here instead of the index? Or is it preoptimization?
/**
 * Finds the closest
 * @param stamp timestamp to search for
 * @return index of the element in `delta_buffer` that has equal or larger timestamp
 */
boost::circular_buffer<igvc_msgs::velocity_pairConstPtr>::iterator
ParticleFilterNode::find_closest(const ros::Time& stamp, boost::circular_buffer<igvc_msgs::velocity_pairConstPtr> buffer)
{
  if (buffer.empty())
  {
    ROS_DEBUG_STREAM("Empty, returning from find_closest RobotState");
    return buffer.end();
  }
  auto end_it = std::lower_bound(buffer.begin(), buffer.end(), stamp, CompareTime());
  if (end_it == buffer.end())
  {
    // ROS_ERROR_STREAM("Couldn't find Robotstate. Last value was " << std::setprecision(4)
    //                                                             << buffer.at(buffer.size() - 1).first->header.stamp);
  }
  return end_it;
}

boost::circular_buffer<igvc_msgs::mapConstPtr>::iterator
ParticleFilterNode::find_closest(const ros::Time& stamp, boost::circular_buffer<igvc_msgs::mapConstPtr> buffer)
{
  if (buffer.empty())
  {
    ROS_DEBUG_STREAM("Empty, returning from find_closest map");
    return buffer.end();
  }
  auto end_it = std::lower_bound(buffer.begin(), buffer.end(), stamp, CompareTime());
  if (end_it == buffer.end())
  {
    // ROS_ERROR_STREAM("Couldn't find map. Last value was " << std::setprecision(4)
    //                                                      << buffer.at(buffer.size() - 1)->header.stamp);
  }
  return end_it;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "particle_filter");
  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  int buffer_size, num_particles;
  double transform_max_wait_time, axle_length, motor_std_dev, initial_pos_std_dev, initial_yaw_std_dev;
  igvc::getParam(pNh, "buffer_size", buffer_size);
  igvc::getParam(pNh, "num_particles", num_particles);
  igvc::getParam(pNh, "transform_max_wait_time", transform_max_wait_time);
  igvc::getParam(pNh, "axle_length", axle_length);
  igvc::getParam(pNh, "motor_std_dev", motor_std_dev);
  igvc::getParam(pNh, "initial_pos_std_dev", initial_pos_std_dev);
  igvc::getParam(pNh, "initial_yaw_std_dev", initial_yaw_std_dev);
  igvc::param(pNh, "debug", DEBUG, false);

  particle_filter_node =
      std::unique_ptr<ParticleFilterNode>(new ParticleFilterNode(buffer_size, buffer_size, transform_max_wait_time));
  particle_filter_node->particle_filter = std::unique_ptr<ParticleFilterBase>(
      new BasicParticleFilter(motor_std_dev, num_particles, initial_pos_std_dev, initial_yaw_std_dev));
  particle_filter_node->particle_filter->axle_length = axle_length;

  particle_filter_node->tf_listener = std::unique_ptr<tf::TransformListener>(new tf::TransformListener());

  ros::Subscriber motor_sub =
      nh.subscribe("/encoders", 1, &ParticleFilterNode::motor_callback, particle_filter_node.get());
  ros::Subscriber pc_sub = nh.subscribe("/pc2", 1, &ParticleFilterNode::pc_callback, particle_filter_node.get());
  ros::Subscriber map_sub = nh.subscribe("/map", 1, &ParticleFilterNode::map_callback, particle_filter_node.get());

  if (DEBUG)
  {
    particle_filter_node->particle_pcl_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/particle_pcl", 1);
  }
  ros::spin();

  return 0;
}
