#ifndef SRC_IMUMEASUREMENT_H
#define SRC_IMUMEASUREMENT_H

#include <slam/g2o/g2o_robot_state.h>

namespace g2o
{
class IMUMeasurement
{
public:
  IMUMeasurement() = default;

  IMUMeasurement(double linear_acceleration, double angular_velocity, double delta_t)
    : linear_acceleration_{ linear_acceleration }, angular_velocity_{ angular_velocity }, delta_t_{ delta_t }
  {
  }

  [[nodiscard]] const double& linear_acceleration() const
  {
    return linear_acceleration_;
  }
  [[nodiscard]] double& linear_acceleration()
  {
    return linear_acceleration_;
  }
  [[nodiscard]] const double& angular_velocity() const
  {
    return angular_velocity_;
  }
  [[nodiscard]] double& angular_velocity()
  {
    return angular_velocity_;
  }
  [[nodiscard]] const double& delta_t() const
  {
    return delta_t_;
  }
  [[nodiscard]] double& delta_t()
  {
    return delta_t_;
  }

  [[nodiscard]] double delta_v() const
  {
    return linear_acceleration_ * delta_t_;
  }

  void fromVector(const Eigen::Vector3d& v)
  {
    *this = IMUMeasurement{ v[0], v[1], v[2] };
  }

  [[nodiscard]] Eigen::Vector3d toVector() const
  {
    return {linear_acceleration_, angular_velocity_, delta_t_};
  }

private:
  double linear_acceleration_;
  double angular_velocity_;
  double delta_t_;
};
}  // namespace g2o

#endif  // SRC_IMUMEASUREMENT_H
