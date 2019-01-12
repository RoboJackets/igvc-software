#include "basic_particle_filter.h"
#include "particle_filter_base.h"

/**
 * Propagates all particles using the the odometry model
 * Equations from https://robotics.stackexchange.com/questions/1653/calculate-position-of-differential-drive-robot
 * @param particles
 * @param state
 * @param motor_command
 * @param deltaT
 */
void BasicParticleFilter::ProposalDistribution(std::vector<Particle> particles,
                                               const igvc_msgs::velocity_pair &motor_command,
                                               const ros::Duration &deltaT)
{
  for (Particle particle : particles)
  {
    double left_delta = motor_command.left_velocity * deltaT.toSec();
    double right_delta = motor_command.right_velocity * deltaT.toSec();
    // If basically going straight
    if (fabs(left_delta - right_delta) < 1.0e-6)
    {
      particle.state.x = particle.state.x + left_delta * cos(particle.state.yaw);
      particle.state.y = particle.state.y + right_delta * sin(particle.state.yaw);
    }
    else
    {
      double r = axle_length * (left_delta + right_delta) / (2 * (right_delta - left_delta));
      double wd = (right_delta - left_delta) / axle_length;
      particle.state.x = particle.state.x + r * sin(wd + particle.state.yaw) - r * sin(particle.state.yaw);
      particle.state.y = particle.state.y - r * cos(wd + particle.state.yaw) + r * cos(particle.state.yaw);
      particle.state.yaw = particle.state.yaw + wd;
      igvc::fit_to_polar(particle.state.yaw);
    }
  }
}

/**
 * Generates the weights for each particle, using the sensor model.
 * @param pointCloud pointcloud from the lidar callback
 * @param particles particles to calculate the weights for
 */
void BasicParticleFilter::getWeights(const pcl::PointCloud<pcl::PointXYZ> &pointCloud, std::vector<Particle> particles)
{
  for (Particle &particle : particles)
  {
    particle.weight = 0;
    pcl::PointCloud<pcl::PointXYZ>::const_iterator point_iter;
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvShare(particle.map->image, particle.map, "mono8");
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
void BasicParticleFilter::addToParticleWeight(Particle &particle, double grid_x, double grid_y,
                                              cv_bridge::CvImageConstPtr map)
{
  // Get all grid cells between particle and pointcloud point using Bressenham's line algorithm
  // TODO: Use gaussian
  particle.weight += map[grid_x][grid_y];
}
