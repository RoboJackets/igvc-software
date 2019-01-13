#ifndef PARTICLE_FILTER_BASE_H
#define PARTICLE_FILTER_BASE_H

#include <igvc_msgs/map.h>
#include <igvc_msgs/velocity_pair.h>
#include <pcl_ros/point_cloud.h>
#include <igvc_utils/RobotState.hpp>
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
  // TODO: Use this
  double motor_std_dev, initial_pos_std_dev, initial_yaw_std_dev;
  std::vector<Particle> particles;

  /**
   * Returns a RobotState containing the deltas
   * ignore: Propagates all particles using the proposal distribution. Called during the callback of the motor command
   * subscriber
   * @param[in/out] state The state at the time of the motor command.
   * @param[in] motor_command The motor command from encoder
   */
  virtual void ProposalDistribution(RobotState& state, const igvc_msgs::velocity_pair& motor_command) = 0;

  /**
   * Generates the weights for each particle. Defined as the proposal distribution / target distribution.
   * @param pointCloud pointcloud from the lidar callback
   * @param particles particles to calculate the weights for
   */
  virtual void getWeights(const pcl::PointCloud<pcl::PointXYZ>& pointCloud, igvc_msgs::mapConstPtr map_ptr) = 0;

  void propagateParticles(const RobotState& delta);

  void resample_points();

  explicit ParticleFilterBase(double motor_std_dev, int num_particles, double initial_pos_std_dev, double initial_yaw_std_dev)
    : motor_std_dev(motor_std_dev), initial_pos_std_dev(initial_pos_std_dev), initial_yaw_std_dev(initial_yaw_std_dev) {
    particles.reserve(static_cast<unsigned long>(num_particles));

    std::random_device rd;
    std::mt19937 e2(rd());
    std::normal_distribution<> dist1(0, initial_pos_std_dev);
    std::normal_distribution<> dist2(0, initial_yaw_std_dev);

    for(int i = 0; i < num_particles; i++)
    {
      double x = dist1(e2);
      double y = dist1(e2);
      double yaw = dist2(e2);
      particles.emplace_back(Particle {RobotState(x, y, yaw), nullptr, 0});
    }
  };
};

#endif  // PARTICLE_FILTER_BASE_H
