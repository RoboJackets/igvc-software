#ifndef SRC_G2O_ROBOT_STATE_H
#define SRC_G2O_ROBOT_STATE_H

#include <g2o/types/slam2d/se2.h>
#include <slam/g2o/twist.h>

#include <utility>

namespace g2o
{
using Vector6d = Eigen::Matrix<double, 6, 1>;
class RobotState
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  RobotState() = default;
  RobotState(SE2 se2, const Twist& twist, double delta_t) : se2_{ std::move(se2) }, twist_{ twist }, delta_t_{ delta_t }
  {
  }

  [[nodiscard]] const SE2& se2() const
  {
    return se2_;
  }
  [[nodiscard]] SE2& se2()
  {
    return se2_;
  }

  [[nodiscard]] const Twist& twist() const
  {
    return twist_;
  }
  [[nodiscard]] Twist& twist()
  {
    return twist_;
  }

  [[nodiscard]] double& delta_t()
  {
    return delta_t_;
  }

  [[nodiscard]] const double& delta_t() const
  {
    return delta_t_;
  }

  [[nodiscard]] RobotState operator*(const SE2& transform) const
  {
    RobotState new_state{ *this };
    new_state.se2_ *= transform;
    return new_state;
  }

  RobotState& operator*=(const SE2& transform)
  {
    se2_ *= transform;
    return *this;
  }

  [[nodiscard]] RobotState withTwist(const Twist& twist, double delta_t) const
  {
    assert(delta_t > 0);
    RobotState ret{ *this };
    auto se2 = twist.toSE2(delta_t);
    ret *= se2;
    ret.delta_t() += delta_t;

    return ret;
  }

  void fromVector(const Vector6d& v)
  {
    se2_.fromVector(v.head<3>());
    twist_.fromVector(v.segment<2>(3));
    delta_t_ = v(5);
  }

  [[nodiscard]] Vector6d toVector() const
  {
    Vector6d ret;
    ret.head<3>() = se2_.toVector();
    ret.segment<2>(2) = twist_.toVector();
    ret(5) = delta_t_;

    return ret;
  }

  void update(const SE2& se2, const Twist& twist)
  {
    se2_ *= se2;
    twist_ += twist;
  }

  friend std::ostream& operator<<(std::ostream& os, const RobotState& robot_state);

private:
  SE2 se2_;
  Twist twist_{};
  double delta_t_{};
};

inline std::ostream& operator<<(std::ostream& os, const RobotState& robot_state)
{
  os << "(" << robot_state.se2()[0] << ", " << robot_state.se2()[1] << ", " << robot_state.se2()[2] << ") ("
     << robot_state.twist().linear() << ", " << robot_state.twist().angular() << ")";
  return os;
}

}  // namespace g2o
#endif  // SRC_G2O_ROBOT_STATE_H
