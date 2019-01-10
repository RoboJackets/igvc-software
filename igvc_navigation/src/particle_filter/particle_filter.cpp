#include <igvc_utils/RobotState.hpp>
#include <random>

struct point {
    RobotState robotState;
    double weight;
};

void resample_points(struct point particles[], int num_particles) {
  // Create array of cumulative weights
  double cum_weights[num_particles + 1];
  for (int i = 0; i < num_particles; i++) {
    cum_weights[i+1] = cum_weights[i] + particles[i];
  }
  // cum_weights[num_particles-1] is cumulative total
  double pointer_width = cles / cum_weights[num_particles - 1] / num_particles;
  std::uniform_real_distribution<double> unif(0, cum_weights[num_particles - 1]);
  std::default_random_engine re;
  double starting_pointer = unif(re);

  // Resample using starting_pointer + i*pointer_width
  struct point sampled_particles[];
  for (int i = 0; i < num_particles; i++) {
    int index = 0;
    while (cum_weights[index] < starting_pointer + i * pointer_width) {
      index++;
    }
    // Found cum_weights[index] >= stating_pointer + i * pointer_width, add that point to the array
    sampled_particles[i] = cum_weights[index];
  }

  // Return sampled array
  particles = sampled_particles;
}
