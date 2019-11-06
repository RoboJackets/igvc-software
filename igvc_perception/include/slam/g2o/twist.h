#ifndef SRC_TWIST_H
#define SRC_TWIST_H

#include <g2o/types/slam2d/se2.h>

namespace g2o
{
class Twist
{
public:
  Twist() = default;
  Twist(double linear, double angular) : linear_{ linear }, angular_{ angular } {};

  [[nodiscard]] const double& linear() const
  {
    return linear_;
  }
  [[nodiscard]] double& linear()
  {
    return linear_;
  }

  [[nodiscard]] const double& angular() const
  {
    return angular_;
  }
  [[nodiscard]] double& angular()
  {
    return angular_;
  }

  [[nodiscard]] Twist operator+(const Twist& other) const
  {
    return { linear_ + other.linear_, angular_ + other.angular_ };
  }

  void operator+=(const Twist& other)
  {
    linear_ += other.linear_;
    angular_ += other.angular_;
  }

  [[nodiscard]] Twist operator/(double constant) const
  {
    return { linear_ / constant, angular_ / constant };
  }

  void fromVector(const Eigen::Vector2d& v)
  {
    linear_ = v(0);
    angular_ = v(1);
  }

  [[nodiscard]] Eigen::Vector2d toVector() const
  {
    return { linear_, angular_ };
  }

  [[nodiscard]] SE2 toSE2(double delta_t) const
  {
    // Avoid divide by 0
    if (angular_ == 0.0)
    {
      return { linear_ * delta_t, 0.0, 0.0 };
    }

    double radius = linear_ / angular_;

    /**
    |  ↑          |\
    |  |          | \
    |  |          |  \ R
    |  | R-Rcosθ R|   \
    |  |          |____\
    |  ↓          |
    |             <---->
    |              Rsinθ
    **/

    double delta_theta = angular_ * delta_t;
    double delta_x = radius * sin(delta_theta);
    double delta_y = radius * (1 - cos(delta_theta));

    return { delta_x, delta_y, delta_theta };
  }

private:
  double linear_;
  double angular_;
};
}  // namespace g2o

#endif  // SRC_TWIST_H
