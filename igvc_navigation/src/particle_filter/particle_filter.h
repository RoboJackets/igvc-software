#pragma once
#ifndef PROJECT_PARTICLE_FILTER_H
#define PROJECT_PARTICLE_FILTER_H

#include <std_msgs/Float64.h>
#include <igvc_utils/RobotState.hpp>
#include "igvc_navigation/hsluv.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <random>
#include "octomapper.h"

struct Particle
{
  RobotState state;
  pc_map_pair pair;
  float weight{0};
};

class Particle_filter
{
public:
  Particle_filter(const ros::NodeHandle& pNh);
  void initialize_particles(const tf::Transform& pose);
  void update(const tf::Transform& diff,const boost::array<double, 36>& covariance,
                               const pcl::PointCloud<pcl::PointXYZ>& pc, const tf::Transform& lidar_to_base);
  int length_x() { return m_octomapper.length_x(); }
  int width_y() { return m_octomapper.width_y(); }
  int start_x() { return m_octomapper.start_x(); }
  int start_y() { return m_octomapper.start_y(); }
  double resolution() { return m_octomapper.resolution(); }
  Particle m_best_particle;

private:
  void resample_particles();
  void map_weight_to_rgb(float weight, double *r, double *g, double *b);
  void visualize_key(const octomap::KeySet freeSet, const octomap::KeySet occSet, const pc_map_pair& pair, uint64_t stamp);

  bool m_debug;
  int m_num_particles{};
  float m_inverse_resample_threshold;
  float m_total_weight;
  double m_viz_hue_start, m_viz_hue_end;
  double m_viz_sat_start, m_viz_sat_end;
  double m_viz_light_start, m_viz_light_end;
  double m_cov_coeff;

  static constexpr double m_viz_hue_max = 360;
  static constexpr double m_viz_sat_max = 100;
  static constexpr double m_viz_light_max = 100;

  ros::NodeHandle pNh;
  ros::Publisher m_particle_pub;
  ros::Publisher m_ground_pub;
  ros::Publisher m_nonground_pub;
  Octomapper m_octomapper;
  std::vector<Particle> m_particles;
};

// From https://stackoverflow.com/questions/6142576/sample-from-multivariate-normal-gaussian-distribution-in-c
class Normal_random_variable
{
public:
  explicit Normal_random_variable(Eigen::MatrixXd const& covar) : m_mean(Eigen::VectorXd::Zero(covar.rows()))
  {
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(covar);
    m_transform = eigen_solver.eigenvectors() * eigen_solver.eigenvalues().cwiseSqrt().asDiagonal();
  }

  Eigen::VectorXd operator()()
  {
    static std::mt19937 gen{ std::random_device{}() };
    static std::normal_distribution<> dist;

    return m_mean + m_transform * Eigen::VectorXd{ m_mean.size() }.unaryExpr([&](double x) { return dist(gen); });
  }

private:
  Eigen::VectorXd m_mean;
  Eigen::MatrixXd m_transform;
};

#endif  // PROJECT_PARTICLE_FILTER_H
