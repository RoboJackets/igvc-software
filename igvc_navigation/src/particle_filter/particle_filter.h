#ifndef PROJECT_PARTICLE_FILTER_H
#define PROJECT_PARTICLE_FILTER_H

#include <igvc_utils/RobotState.hpp>
#include "octomapper.h"
#include <std_msgs/Float64.h>

struct Particle
{
  RobotState state;
  pc_map_pair pair;
  float weight;
};

class Particle_filter
{
public:
  Particle_filter(ros::NodeHandle pNh) : pNh(pNh)
  {
    igvc::getParam(pNh, "particle_filter/num_particles", m_num_particles);
    particles.reserve(m_num_particles);
  }
  void update(const tf::Transform& diff, const boost::array<double, 36>& covariance, const pcl::PointCloud<pcl::PointXYZ>& pc);
  std::vector<Particle> particles;

private:
  ros::NodeHandle pNh;
  int m_num_particles;
};

#endif  // PROJECT_PARTICLE_FILTER_H
