#include <pcl_ros/transforms.h>
#include "particle_filter.h"

void Particle_filter::update(const tf::Transform& diff, const boost::array<double, 36>& covariance,
                             const pcl::PointCloud<pcl::PointXYZ>& pc, const tf::Transform& lidar_to_base)
{
  // Create noise distribution from covariance to sample from
  boost::array<double, 36> cov_copy(covariance);
  Eigen::Map<Eigen::Matrix<double, 6, 6>> eigen_cov(cov_copy.data());
  Normal_random_variable uncertainty {eigen_cov};


  // Separate pc to ground and nonground
  pcl::PointCloud<pcl::PointXYZ> ground, nonground;
  m_octomapper.filter_ground_plane(pc, ground, nonground);

  float weight_sum = 0;
  // For each particle in particles
  for (Particle& p : m_particles)
  {
    // TODO: Add Scanmatching
    // TODO: Add CUDA or OpenMP?
    // Sampled new particle from old using pose and covariance
    p.state *= diff;
    p.state += uncertainty();

    // Transform particles from base_frame to odom_frame
    pcl::PointCloud<pcl::PointXYZ> transformed_pc;
    pcl_ros::transformPointCloud(pc, transformed_pc, p.state.transform.inverse()); // TODO: Inverse?

    // Transform ground and nonground from base_frame to odom_frame
    // TODO: Is transform faster or plane detection faster? Do I move the ground filtering into the for loop?
    pcl::PointCloud<pcl::PointXYZ> transformed_ground, transformed_nonground;
    pcl_ros::transformPointCloud(ground, transformed_ground, p.state.transform.inverse()); // TODO: Inverse?
    pcl_ros::transformPointCloud(nonground, transformed_nonground, p.state.transform.inverse()); // TODO: Inverse?

    // Calculate weight using sensor model
    octomap::KeySet free, occupied;
    tf::Transform odom_to_lidar = p.state.transform * lidar_to_base.inverse();

    m_octomapper.separate_occupied(free, occupied, odom_to_lidar.getOrigin(), p.pair, transformed_ground, transformed_nonground);
    p.weight = m_octomapper.sensor_model(p.pair, free, occupied);
    weight_sum += p.weight;

    // Update map
    m_octomapper.insert_scan(p.pair, free, occupied);
  }
  // calculate Neff
  float inverse_n_eff = 0;
  float squared_weight_sum = weight_sum * weight_sum;
  for (Particle& p : m_particles)
  {
    inverse_n_eff += (p.weight*p.weight)/squared_weight_sum;
  }
  if (inverse_n_eff > m_inverse_resample_threshold)
  {
    resample_particles();
  }
  // if Neff < thresh then resample
}

void Particle_filter::resample_particles()
{
  // Create array of cumulative weights
  std::vector<double> cum_weights;
  cum_weights.reserve(m_num_particles);
  cum_weights.emplace_back(m_particles[0].weight);
  for (size_t i = 1; i < m_num_particles; i++)
  {
    cum_weights.emplace_back(cum_weights[i - 1] + m_particles[i].weight);
  }
  // cum_weights[num_particles-1] is cumulative total
  double pointer_width = cum_weights[m_num_particles - 1] / m_num_particles;
  std::uniform_real_distribution<double> unif(0, pointer_width);
  std::random_device rd;
  std::default_random_engine re{rd()};
  double starting_pointer = unif(re);

  // Resample using starting_pointer + i*pointer_width
  std::vector<struct Particle> sampled_particles;
  sampled_particles.reserve(m_num_particles);
  for (size_t i = 0; i < m_num_particles; i++)
  {
    int index = 0;
    while (cum_weights[index] < starting_pointer + i * pointer_width)
    {
      index++;
    }
    // Found cum_weights[index] >= stating_pointer + i * pointer_width, add that point to the array
    sampled_particles.emplace_back(m_particles[index]);
  }
  // Return sampled array
  m_particles.swap(sampled_particles);
}
