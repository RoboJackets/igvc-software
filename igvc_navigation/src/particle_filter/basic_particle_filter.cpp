#include "basic_particle_filter.h"
#include <random>
#include "particle_filter_base.h"

/**
 * Calculates state delta using the the odometry model
 * Equations from https://robotics.stackexchange.com/questions/1653/calculate-position-of-differential-drive-robot
 * @param[in/out] state Starting state, modified to contain delta
 * @param[in] motor_command motor command from encoder
 */
void BasicParticleFilter::ProposalDistribution(RobotState &state, const igvc_msgs::velocity_pair &motor_command)
{
  std::random_device rd;
  std::mt19937 e2(rd());
  std::normal_distribution<> dist(0, motor_std_dev);
  double gaussian_noise = dist(e2);
  double left_delta = (motor_command.left_velocity + gaussian_noise) * motor_command.duration;
  double right_delta = (motor_command.right_velocity + gaussian_noise) * motor_command.duration;
  // If basically going straight
  if (fabs(left_delta - right_delta) < 1.0e-6)
  {
    state.x = state.x + left_delta * cos(state.yaw);
    state.y = state.y + right_delta * sin(state.yaw);
  }
  else
  {
    double r = axle_length * (left_delta + right_delta) / (2 * (right_delta - left_delta));
    double wd = (right_delta - left_delta) / axle_length;
    state.x = state.x + r * sin(wd + state.yaw) - r * sin(state.yaw);
    state.y = state.y - r * cos(wd + state.yaw) + r * cos(state.yaw);
    state.yaw = state.yaw + wd;
    igvc::fit_to_polar(state.yaw);
  }
  //  for (Particle particle : particles)
  //  {
  //    double left_delta = motor_command.left_velocity * motor_command.duration;
  //    double right_delta = motor_command.right_velocity * motor_command.duration;
  //    // If basically going straight
  //    if (fabs(left_delta - right_delta) < 1.0e-6)
  //    {
  //      particle.state.x = particle.state.x + left_delta * cos(particle.state.yaw);
  //      particle.state.y = particle.state.y + right_delta * sin(particle.state.yaw);
  //    }
  //    else
  //    {
  //      double r = axle_length * (left_delta + right_delta) / (2 * (right_delta - left_delta));
  //      double wd = (right_delta - left_delta) / axle_length;
  //      particle.state.x = particle.state.x + r * sin(wd + particle.state.yaw) - r * sin(particle.state.yaw);
  //      particle.state.y = particle.state.y - r * cos(wd + particle.state.yaw) + r * cos(particle.state.yaw);
  //      particle.state.yaw = particle.state.yaw + wd;
  //      igvc::fit_to_polar(particle.state.yaw);
  //    }
  //  }
}

/**
 * Generates the weights for each particle, using the sensor model.
 * @param pointCloud pointcloud from the lidar callback
 * @param particles particles to calculate the weights for
 */
void BasicParticleFilter::getWeights(const pcl::PointCloud<pcl::PointXYZ> &pointCloud, std::vector<Particle> particles,
                                     igvc_msgs::mapConstPtr map_ptr)
{
  cv_bridge::CvImageConstPtr cv_ptr;
//  cv_ptr = cv_bridge::toCvShare(particle.map->image, particle.map, "mono8");
  cv_ptr = cv_bridge::toCvShare(map_ptr->image, map_ptr, "mono8");
  for (Particle &particle : particles)
  {
    particle.weight = 0;
    pcl::PointCloud<pcl::PointXYZ>::const_iterator point_iter;
    for (point_iter = pointCloud.begin(); point_iter < pointCloud.points.end(); point_iter++)
    {
      double point_x = particle.state.x + point_iter->x;
      double point_y = particle.state.y + point_iter->y;
      int point_grid_x = static_cast<int>(std::round(point_x / particle.map->resolution + particle.map->x_initial));
      int point_grid_y = static_cast<int>(std::round(point_y / particle.map->resolution + particle.map->y_initial));

      addToParticleWeight(particle, point_grid_x, point_grid_y, cv_ptr);
    }
  }
}

/**
 * Calculates weight using gaussian from particle's location to the pointcloud, by summing the product of the gaussian
 * and the corresponding map cell. Adds weight to particle.
 * @param[in/ou] particle Particle to be used.
 * @param[in] grid_x x coordinate of pointcloud in map
 * @param[in] grid_y y coordinate of pointcloud in map
 * @param[in] map ptr of map
 */
void BasicParticleFilter::addToParticleWeight(Particle &particle, int grid_x, int grid_y,
                                              cv_bridge::CvImageConstPtr image_ptr)
{
  // Get all grid cells between particle and pointcloud point using Bressenham's line algorithm
  // TODO: Use gaussian
  particle.weight += image_ptr->image.at<uchar>(grid_x, grid_y);
}
