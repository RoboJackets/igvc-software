#include "particle_filter.h"
void Particle_filter::update(const tf::Transform& diff, const boost::array<double, 36>& covariance,
                             const pcl::PointCloud<pcl::PointXYZ>& pc)
{
  // For each particle in particles
  // TODO: Add Scanmatching
  // TODO: Add CUDA or OpenMP?
  // Sampled new particle from old using pose and covariance
  // Calculate weight using sensor model
  // Update map
  // calculate Neff, if Neff < thresh then resample
}
