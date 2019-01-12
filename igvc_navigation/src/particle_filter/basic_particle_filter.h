#ifndef BASIC_PARTICLE_FILTER_H
#define BASIC_PARTICLE_FILTER_H

#include "particle_filter_base.h"
#include <cv_bridge/cv_bridge.h>

class BasicParticleFilter : public ParticleFilterBase
{
public:
    void ProposalDistribution(std::vector<Particle> particles, const igvc_msgs::velocity_pair &motor_command,
                              const ros::Duration &deltaT) override;

    void getWeights(const pcl::PointCloud<pcl::PointXYZ> &pointCloud, std::vector<Particle> particles) override;

private:
    void BasicParticleFilter::addToParticleWeight(Particle &particle, double grid_x, double grid_y,
                                                  cv_bridge::CvImageConstPtr map);
};

#endif //BASIC_PARTICLE_FILTER_H
