#ifndef SRC_VERTEX_ROBOT_STATE_H
#define SRC_VERTEX_ROBOT_STATE_H

#include <g2o/core/base_vertex.h>
#include <g2o/core/hyper_graph_action.h>
#include <slam/g2o/g2o_robot_state.h>

namespace g2o
{
class VertexRobotState : public g2o::BaseVertex<5, RobotState>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  VertexRobotState();

  void setToOriginImpl() override;
  bool read(std::istream &is) override;
  bool write(std::ostream &os) const override;
  void oplusImpl(const number_t *update) override;
};
}  // namespace g2o
#endif  // SRC_VERTEX_ROBOT_STATE_H
