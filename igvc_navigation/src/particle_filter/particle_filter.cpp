#include "particle_filter.h"

void Particle_filter::update(const tf::Transform& diff, boost::array<double, 36>& covariance,
                             const pcl::PointCloud<pcl::PointXYZ>& pc)
{
  // Create noise distribution from covariance to sample from
  Eigen::Map<Eigen::Matrix<double, 6, 6>> eigen_cov(covariance.data());
  Normal_random_variable uncertainty {eigen_cov};

  // Separate pc to ground and nonground
  pcl::PointCloud<pcl::PointXYZ> ground, nonground;
  m_octomapper.filter_ground_plane(pc, ground, nonground);

  // For each particle in particles
  for (Particle& p : m_particles)
  {
    // TODO: Add Scanmatching
    // TODO: Add CUDA or OpenMP?
    // Sampled new particle from old using pose and covariance
    RobotState new_state {(p.state * diff) + uncertainty()}; // Change RobotState to a tf::Transform, overload the * operator

    // Calculate weight using sensor model


    // Update map
    // calculate Neff, if Neff < thresh then resample
  }
}

