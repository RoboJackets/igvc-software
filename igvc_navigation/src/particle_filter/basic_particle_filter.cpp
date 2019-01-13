#include "basic_particle_filter.h"
#include <random>
#include "particle_filter_base.h"

/**
 * Calculates state delta using the the odometry model
 * Equations from https://robotics.stackexchange.com/questions/1653/calculate-position-of-differential-drive-robot
 * @param[in/out] state Starting state, modified to contain delta
 * @param[in] motor_command motor command from encoder
 */
void BasicParticleFilter::proposal_distribution(const igvc_msgs::velocity_pairConstPtr& motor_command)
{
  std::random_device rd;
  std::mt19937 e2(rd());
  std::normal_distribution<> dist(0, motor_std_dev);
  for (Particle& particle : particles)
  {
    double gaussian_noise_left = dist(e2);
    double gaussian_noise_right = dist(e2);

    // ROS_INFO_STREAM("Duration: " << motor_command.duration);
    double left_delta = (motor_command->left_velocity + gaussian_noise_left) * motor_command->duration;
    double right_delta = (motor_command->right_velocity + gaussian_noise_right) * motor_command->duration;
    // If basically going straight
    if (fabs(left_delta - right_delta) < 1.0e-6)
    {
      // state.x = state.x + left_delta * cos(state.yaw);
      // state.y = state.y + right_delta * sin(state.yaw);
      particle.state.x += left_delta * cos(particle.state.yaw);
      particle.state.y += right_delta * sin(particle.state.yaw);
    }
    else
    {
      double r = axle_length * (left_delta + right_delta) / (2 * (right_delta - left_delta));
      double wd = (right_delta - left_delta) / axle_length;
      particle.state.x += r * sin(wd + particle.state.yaw) - r * sin(particle.state.yaw);
      particle.state.y += -r * cos(wd + particle.state.yaw) + r * cos(particle.state.yaw);
      particle.state.yaw += wd;
      igvc::fit_to_polar(particle.state.yaw);
    }
  }
}

/**
 * Generates the weights for each particle, using the sensor model.
 * @param pointCloud pointcloud from the lidar callback
 * @param particles particles to calculate the weights for
 */
void BasicParticleFilter::getWeights(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pointCloud, igvc_msgs::mapConstPtr map_ptr)
{
  // ROS_INFO_STREAM("map_ptr == nullptr: " << (map_ptr == nullptr));
  cv_bridge::CvImageConstPtr cv_ptr;
  //  cv_ptr = cv_bridge::toCvShare(particle.map->image, particle.map, "mono8");
  cv_ptr = cv_bridge::toCvShare(map_ptr->image, map_ptr, "mono8");
  double sum = 0;
  for (Particle& particle : particles)
  {
    particle.weight = 0;
    pcl::PointCloud<pcl::PointXYZ>::const_iterator point_iter;
    for (point_iter = pointCloud->points.begin(); point_iter < pointCloud->points.end(); point_iter++)
    {
      double point_x = particle.state.x + point_iter->x;
      double point_y = particle.state.y + point_iter->y;
      // int point_grid_x = static_cast<int>(std::round(point_x / particle.map->resolution + particle.map->x_initial));
      // int point_grid_y = static_cast<int>(std::round(point_y / particle.map->resolution + particle.map->y_initial));
      int point_grid_x = static_cast<int>(std::round(point_x / map_ptr->resolution + map_ptr->x_initial));
      int point_grid_y = static_cast<int>(std::round(point_y / map_ptr->resolution + map_ptr->y_initial));
      addToParticleWeight(particle, point_grid_x, point_grid_y, cv_ptr);
    }
    sum += particle.weight;
  }
  for (Particle& particle : particles)
  {
    particle.weight += sum/particles.size();
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
void BasicParticleFilter::addToParticleWeight(Particle& particle, int grid_x, int grid_y,
                                              cv_bridge::CvImageConstPtr image_ptr)
{
  // Get all grid cells between particle and pointcloud point using Bressenham's line algorithm
  // TODO: Use gaussian
  if (grid_x >= image_ptr->image.cols)
  {
    // ROS_ERROR("GOTTEM 1");
  }
  if (grid_y >= image_ptr->image.rows)
  {
    // ROS_ERROR("GOTTEM 2");
  }
  particle.weight += image_ptr->image.at<uchar>(grid_x, grid_y);
}
