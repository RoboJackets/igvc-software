#include <igvc_utils/RobotState.hpp>
#include <boost/circular_buffer.hpp>
#include <igvc_msgs/velocity_pair.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "particle_filter_base.h"
#include "basic_particle_filter.h"

class ParticleFilterNode
{
public:
    void motor_callback(const igvc_msgs::velocity_pair& motor_command);
    void pc_callback(const pcl::PointCloud<pcl::PointXYZ>& pointCloud);
    int buffer_size;
    double transform_max_wait_time;
    boost::circular_buffer <struct Point> point_buffer;
    std::unique_ptr<tf::TransformListener> tf_listener;
    std::unique_ptr<ParticleFilterBase> particle_filter;
};

void ParticleFilterNode::motor_callback(const igvc_msgs::velocity_pair& motor_command) {
  // Stamped pls? So I can TF:waitForTransform, then call a odometry model to get the actual changed positions
  tf::StampedTransform transform;
  if (tf_listener->waitForTransform("/odom", "/base_link", motor_command.header.stamp, ros::Duration(transform_max_wait_time)))
  {
    tf_listener->lookupTransform("/odom", "/base_link", motor_command.header.stamp, transform);
  } else {
    // TODO: How to handle if doesn't find it?
    // Currently will just get the latest one, shouldn't make that large of an error?
    tf_listener->lookupTransform("/odom", "/base_link", ros::Time(0), transform);
  }
  RobotState state(transform);
}

void ParticleFilterNode::pc_callback(const pcl::PointCloud<pcl::PointXYZ>& pointCloud) {
  // Search for closest one using binary search
//  pointCloud.time or something
//  int end_pos = findClosestTransform()


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "particle_filter");
  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  ParticleFilterNode particle_filter_node;
  particle_filter_node.particle_filter = std::unique_ptr<ParticleFilterBase>(new BasicParticleFilter());

  igvc::getParam(pNh, "buffer_size", particle_filter_node.buffer_size);
  igvc::getParam(pNh, "transform_max_wait_time", particle_filter_node.transform_max_wait_time);
  igvc::getParam(pNh, "axle_length", particle_filter_node.particle_filter->axle_length);
  igvc::getParam(pNh, "occupancy_grid_length", particle_filter_node.particle_filter->length_y);
  igvc::getParam(pNh, "occupancy_grid_width", particle_filter_node.particle_filter->width_x);
  igvc::getParam(pNh, "occupancy_grid_resolution", particle_filter_node.particle_filter->resolution);
  igvc::getParam(pNh, "start_X", particle_filter_node.particle_filter->cont_start_x);
  igvc::getParam(pNh, "start_Y", particle_filter_node.particle_filter->cont_start_y);

  particle_filter_node.tf_listener = std::unique_ptr<tf::TransformListener>(new tf::TransformListener());

  ros::Subscriber motor_sub = nh.subscribe("/encoders", 1, &ParticleFilterNode::motor_callback, &particle_filter_node);
  ros::Subscriber pc_sub = nh.subscribe("/pc2", 1, &ParticleFilterNode::pc_callback, &particle_filter_node);

  ros::spin();

  return 0;
}
