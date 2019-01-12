#ifndef PARTICLE_FILTER_BASE_H
#define PARTICLE_FILTER_BASE_H

#include <igvc_utils/RobotState.hpp>
#include <igvc_msgs/velocity_pair.h>
#include <igvc_msgs/map.h>
#include <pcl_ros/point_cloud.h>
#include <random>

struct Particle
{
    RobotState state;
    igvc_msgs::mapPtr map;
    double weight;
};

class ParticleFilterBase
{
public:
    double axle_length;

    /**
     * Returns a RobotState containing the deltas
     * ignore: Propagates all particles using the proposal distribution. Called during the callback of the motor command subscriber
     * @param[in/out] state The state at the time of the motor command.
     * @param[in] motor_command The motor command from encoder
     */
    virtual void ProposalDistribution(RobotState& state, const igvc_msgs::velocity_pair &motor_command) = 0;

    /**
     * Generates the weights for each particle. Defined as the proposal distribution / target distribution.
     * @param pointCloud pointcloud from the lidar callback
     * @param particles particles to calculate the weights for
     */
    virtual void getWeights(const pcl::PointCloud<pcl::PointXYZ> &pointCloud, std::vector<Particle> particles) = 0;

    void resample_points(std::vector<struct Particle>& particles);
private:
    // TODO: Use this
    double variance;
};

/**
 * Resamples points using Stochastic Universal Sampling (minimizes variance)
 * @param[in, out] particles
 * @param[in] num_particles
 */
void ParticleFilterBase::resample_points(std::vector<struct Particle>& particles)
{
  // Create array of cumulative weights
  std::vector<double> cum_weights;
  auto num_particles = particles.size();
  cum_weights.reserve(num_particles +1);
  cum_weights.emplace_back(0);
  for (int i = 0; i < num_particles; i++) {
    cum_weights.emplace_back(cum_weights[i] + particles[i].weight);
  }
  // cum_weights[num_particles-1] is cumulative total
  double pointer_width = cum_weights[num_particles - 1] / num_particles;
  std::uniform_real_distribution<double> unif(0, cum_weights[num_particles - 1]);
  std::default_random_engine re;
  double starting_pointer = unif(re);

  // Resample using starting_pointer + i*pointer_width
  std::vector<struct Particle> sampled_particles;
  sampled_particles.reserve(num_particles);
  for (int i = 0; i < num_particles; i++) {
    int index = 0;
    while (cum_weights[index] < starting_pointer + i * pointer_width) {
      index++;
    }
    // Found cum_weights[index] >= stating_pointer + i * pointer_width, add that point to the array
    sampled_particles.emplace_back(particles[i]);
  }

  // Return sampled array
  particles.swap(sampled_particles);
}

#endif //PARTICLE_FILTER_BASE_H
