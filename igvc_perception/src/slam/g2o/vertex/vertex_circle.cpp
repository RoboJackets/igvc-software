#include <slam/g2o/vertex/vertex_circle.h>

bool VertexCircle::read(std::istream&)
{
  std::cerr << __PRETTY_FUNCTION__ << " not implemented yet " << std::endl;
  return false;
}

bool VertexCircle::write(std::ostream&) const
{
  std::cerr << __PRETTY_FUNCTION__ << " not implemented yet " << std::endl;
  return false;
}

void VertexCircle::setToOriginImpl()
{
  std::cerr << __PRETTY_FUNCTION__ << " not implemented yet " << std::endl;
}

void VertexCircle::oplusImpl(const double* update)
{
  Eigen::Vector3d::ConstMapType v{ update };
  _estimate += v;
}
