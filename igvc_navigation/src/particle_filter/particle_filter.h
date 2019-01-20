#ifndef PROJECT_PARTICLE_FILTER_H
#define PROJECT_PARTICLE_FILTER_H

#include <igvc_utils/RobotState.hpp>
#include "octomapper.h"
#include <std_msgs/Float64.h>
#include <random>

struct Particle
{
  RobotState state;
  pc_map_pair pair;
  float weight;
};

class Particle_filter
{
public:
  explicit Particle_filter(const ros::NodeHandle &pNh) : pNh(pNh), m_octomapper(pNh)
  {
    igvc::getParam(pNh, "particle_filter/num_particles", m_num_particles);
    m_particles.reserve(static_cast<unsigned long>(m_num_particles));
  }
  void update(const tf::Transform& diff, const boost::array<double, 36>& covariance, const pcl::PointCloud<pcl::PointXYZ>& pc);

private:
  int m_num_particles{};
  ros::NodeHandle pNh;
  Octomapper m_octomapper;
  std::vector<Particle> m_particles;
};

// From https://stackoverflow.com/questions/6142576/sample-from-multivariate-normal-gaussian-distribution-in-c
class Normal_random_variable
{
public:
  Normal_random_variable(Eigen::MatrixXd const& covar) : m_mean(Eigen::VectorXd::Zero(covar.rows()))
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
