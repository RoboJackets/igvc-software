#ifndef BASIC_PARTICLE_FILTER_H
#define BASIC_PARTICLE_FILTER_H

#include <cv_bridge/cv_bridge.h>
#include "particle_filter_base.h"

class BasicParticleFilter : public ParticleFilterBase
{
public:
  void ProposalDistribution(RobotState &state, const igvc_msgs::velocity_pair &motor_command) override;
  void getWeights(const pcl::PointCloud<pcl::PointXYZ> &pointCloud, std::vector<Particle> particles) override;

private:
  void addToParticleWeight(Particle &particle, double grid_x, double grid_y, cv_bridge::CvImageConstPtr map);
};

#endif  // BASIC_PARTICLE_FILTER_H
