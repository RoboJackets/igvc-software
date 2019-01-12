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
  void motor_callback(const igvc_msgs::velocity_pair& motor_command);
  void pc_callback(const pcl::PointCloud<pcl::PointXYZ>& pointCloud);
  void map_callback(igvc_msgs::mapConstPtr map);

  double transform_max_wait_time;
  boost::circular_buffer<RobotState> delta_buffer;
  boost::circular_buffer<igvc_msgs::mapConstPtr> map_buffer;
  std::unique_ptr<tf::TransformListener> tf_listener;
  std::unique_ptr<ParticleFilterBase> particle_filter;
  ros::Publisher particle_pcl_pub;

  ParticleFilterNode(int delta_buffer_size, int map_buffer_size, double transform_max_wait_time)
    : transform_max_wait_time(transform_max_wait_time), delta_buffer(delta_buffer_size), map_buffer(map_buffer_size)
  {
  }

private:
  template <class T>
  int find_closest(const ros::Time& stamp, boost::circular_buffer<T> buffer);
};

std::unique_ptr<ParticleFilterNode> particle_filter_node;
bool DEBUG = false;

/**
 * Callback for motor commands. Calls `particle_filter->ProposalDistribution` to get state deltas, and adds them to
 * the delta_buffer
 * @param motor_command motor commands from encoders
 */
void ParticleFilterNode::motor_callback(const igvc_msgs::velocity_pair& motor_command)
{
  tf::StampedTransform transform;
  if (tf_listener->waitForTransform("/odom", "/base_link", motor_command.header.stamp,
                                    ros::Duration(transform_max_wait_time)))
  {
    tf_listener->lookupTransform("/odom", "/base_link", motor_command.header.stamp, transform);
  }
  else
  {
    // TODO: How to handle if doesn't find it?
    // Currently will just get the latest one, shouldn't make that large of an error?
    ROS_ERROR_STREAM("Failed lookupTransform, using latest one");
    tf_listener->lookupTransform("/odom", "/base_link", ros::Time(0), transform);
  }
  RobotState state(transform, motor_command.header.stamp);
  particle_filter_node->particle_filter->ProposalDistribution(state, motor_command);
  delta_buffer.push_back(state);
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
void ParticleFilterNode::pc_callback(const pcl::PointCloud<pcl::PointXYZ>& pointcloud)
{
  ros::Time stamp;
  pcl_conversions::fromPCL(pointcloud.header.stamp, stamp);
  int end_pos = find_closest<RobotState>(stamp, delta_buffer);

  // Accumulate all deltas into a single delta
  RobotState accumulated(delta_buffer[0]);
  for (auto i = 1; i < end_pos; i++)
  {
    accumulated.x += delta_buffer[i].x;
    accumulated.y += delta_buffer[i].y;
    accumulated.yaw += delta_buffer[i].yaw;
  }

  // Propogate the particles using the delta
  for (Particle particle : particle_filter_node->particle_filter->particles)
  {
    particle.state.x += accumulated.x;
    particle.state.y += accumulated.y;
    particle.state.yaw += accumulated.yaw;
  }
  // Erase from start to iterator
  delta_buffer.erase_begin(static_cast<unsigned long>(end_pos));

  // Repeat above for map LOL
  end_pos = find_closest<igvc_msgs::mapConstPtr>(stamp, map_buffer);

  // Compute weights using propagated particles
  particle_filter_node->particle_filter->getWeights(pointcloud, particle_filter_node->particle_filter->particles,
                                                    map_buffer[end_pos]);

  delta_buffer.erase_begin(static_cast<unsigned long>(end_pos));

  // Resample
  particle_filter_node->particle_filter->resample_points(particle_filter_node->particle_filter->particles);

  if (DEBUG)
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
    particle_pcl->header.stamp = pointcloud.header.stamp;
    particle_pcl_pub.publish(particle_pcl);
  }
}

struct CompareTime
{
  ros::Time asTime(const RobotState& state) const
  {
    return state.stamp;
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
template <class T>
int ParticleFilterNode::find_closest(const ros::Time& stamp, boost::circular_buffer<T> buffer)
{
  auto end_it = std::lower_bound(buffer.begin(), buffer.end(), stamp, CompareTime());
  return static_cast<int>(end_it - buffer.begin());
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
