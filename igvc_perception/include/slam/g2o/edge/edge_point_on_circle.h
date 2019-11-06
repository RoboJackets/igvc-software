#ifndef SRC_EDGE_POINT_ON_CIRCLE_H
#define SRC_EDGE_POINT_ON_CIRCLE_H

#include <Eigen/Core>
#include <g2o/core/base_unary_edge.h>
#include <slam/g2o/vertex/vertex_circle.h>

class EdgePointOnCircle : public g2o::BaseUnaryEdge<1, Eigen::Vector2d, VertexCircle>
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgePointOnCircle() = default;

  bool read(std::istream& is) override;
  bool write(std::ostream& os) const override;
  void computeError() override;
};

#endif //SRC_EDGE_POINT_ON_CIRCLE_H
