#ifndef SRC_VERTEX_CIRCLE_H
#define SRC_VERTEX_CIRCLE_H

#include <Eigen/Core>
#include <g2o/core/base_vertex.h>

class VertexCircle : public g2o::BaseVertex<3, Eigen::Vector3d>
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  VertexCircle() = default;

  bool read(std::istream&) override;

  bool write(std::ostream&) const override;

  void setToOriginImpl() override;

  void oplusImpl(const double* update) override;
};

#endif //SRC_VERTEX_CIRCLE_H
