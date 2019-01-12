#include "particle_filter_base.h"

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
  for (size_t i = 0; i < num_particles; i++) {
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
  for (size_t i = 0; i < num_particles; i++) {
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
