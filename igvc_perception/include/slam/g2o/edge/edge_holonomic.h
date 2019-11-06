#ifndef SRC_EDGE_HOLONOMIC_H
#define SRC_EDGE_HOLONOMIC_H

#include <ros/ros.h>
#include <g2o/core/base_binary_edge.h>
#include <slam/g2o/IMU_measurement.h>
#include <slam/g2o/vertex/vertex_robot_state.h>

namespace g2o
{
class EdgeHolonomic : public BaseBinaryEdge<3, int, VertexRobotState, VertexRobotState>
{
public:
  EdgeHolonomic() : BaseBinaryEdge<3, int, VertexRobotState, VertexRobotState>(){};

public:
  void computeError() override
  {
    const auto* v1 = static_cast<const VertexRobotState*>(_vertices[0]);
    const auto* v2 = static_cast<const VertexRobotState*>(_vertices[1]);

    const auto& v1_estimate = v1->estimate();
    const auto& v1_twist = v1_estimate.twist();
    const auto& v2_estimate = v2->estimate();
    const auto& v2_twist = v2->estimate().twist();

    Twist average_twist = (v1_twist + v2_twist) / 2.0;

    double delta_t = v2_estimate.delta_t() - v1_estimate.delta_t();

    RobotState predicted_robot_state = v1_estimate.withTwist(average_twist, delta_t);

    _error = (predicted_robot_state.se2().inverse() * v2_estimate.se2()).toVector();
  }

  bool read(std::istream& is) override
  {
    return true;
  }

  bool write(std::ostream& os) const override
  {
    os << "holonomic edge";
    return os.good();
  }
};
}  // namespace g2o

#endif  // SRC_EDGE_HOLONOMIC_H
