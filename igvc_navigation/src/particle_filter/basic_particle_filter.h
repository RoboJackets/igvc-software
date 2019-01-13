#ifndef BASIC_PARTICLE_FILTER_H
#define BASIC_PARTICLE_FILTER_H

#include <cv_bridge/cv_bridge.h>
#include "particle_filter_base.h"

class BasicParticleFilter : public ParticleFilterBase
{
public:
  void ProposalDistribution(RobotState &state, const igvc_msgs::velocity_pair &motor_command) override;
  void getWeights(const pcl::PointCloud<pcl::PointXYZ> &pointCloud, igvc_msgs::mapConstPtr map_ptr) override;

  explicit BasicParticleFilter(double motor_std_dev, int num_particles, double initial_pos_std_dev, double initial_yaw_std_dev)
    : ParticleFilterBase(motor_std_dev, num_particles, initial_pos_std_dev, initial_yaw_std_dev){};

private:
  void addToParticleWeight(Particle &particle, int grid_x, int grid_y, cv_bridge::CvImageConstPtr map);
};

#endif  // BASIC_PARTICLE_FILTER_H
