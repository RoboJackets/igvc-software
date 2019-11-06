#ifndef SRC_EDGE_LANDMARK_H
#define SRC_EDGE_LANDMARK_H

#include <g2o/core/base_binary_edge.h>
#include <slam/g2o/vertex/vertex_robot_state.h>
#include <g2o/types/slam2d/vertex_point_xy.h>

namespace g2o
{
 class EdgeLandmark : public BaseBinaryEdge<2, Eigen::Vector2d, VertexRobotState, VertexPointXY>
{
 public:
  EdgeLandmark() = default;
  void computeError() override;

  bool read(std::istream& is) override;
  bool write(std::ostream& os) const override;

 private:
  double delta_v_{};
};
}  // namespace g2o

#endif //SRC_EDGE_LANDMARK_H
