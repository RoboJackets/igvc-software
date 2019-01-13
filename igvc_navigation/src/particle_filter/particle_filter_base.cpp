#include "particle_filter_base.h"

/**
 * Resamples points using Stochastic Universal Sampling (minimizes variance)
 * @param[in, out] particles
 * @param[in] num_particles
 */
void ParticleFilterBase::resample_points()
{
  // Create array of cumulative weights
  std::ostringstream out;
  std::ostringstream out1;
  std::ostringstream out2;
  std::vector<double> cum_weights;
  auto num_particles = particles.size();
  cum_weights.reserve(num_particles);
  cum_weights.emplace_back(particles[0].weight);
  for (size_t i = 1; i < num_particles; i++) {
    cum_weights.emplace_back(cum_weights[i-1] + particles[i].weight);
  }
  for (double cum : cum_weights)
  {
    out1 << cum << " ";
  }
  // cum_weights[num_particles-1] is cumulative total
  double pointer_width = cum_weights[num_particles - 1] / num_particles;
  std::uniform_real_distribution<double> unif(0, pointer_width);
  std::default_random_engine re;
  double starting_pointer = unif(re);

  // Resample using starting_pointer + i*pointer_width
  std::vector<struct Particle> sampled_particles;
  sampled_particles.reserve(num_particles);
  for (size_t i = 0; i < num_particles; i++) {
    int index = 0;
    while (cum_weights[index] < starting_pointer + i * pointer_width) {
      index++;
    }
    // Found cum_weights[index] >= stating_pointer + i * pointer_width, add that point to the array
    sampled_particles.emplace_back(particles[index]);
  }
  ROS_INFO_STREAM("Old Weights:" << out.str());
  ROS_INFO_STREAM("Cumulative:" << out1.str());
  for (Particle p : sampled_particles)
  {
    out2 << p.weight << " ";
  }
  ROS_INFO_STREAM("New Weights:" << out2.str());
  // Return sampled array
  particles.swap(sampled_particles);
}

void ParticleFilterBase::propagateParticles(const RobotState& accumulated) {
  //ROS_INFO_STREAM("Before: " << particles[0].state);
  ROS_INFO_STREAM("Accumulated: " << accumulated);
  for (Particle& particle : particles)
  {
    particle.state.x += accumulated.x;
    particle.state.y += accumulated.y;
    particle.state.yaw += accumulated.yaw;
    igvc::fit_to_polar(particle.state.yaw);
  }
  //ROS_INFO_STREAM("After: " << particles[0].state);
}
