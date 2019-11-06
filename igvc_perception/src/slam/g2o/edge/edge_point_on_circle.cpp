#include <slam/g2o/edge/edge_point_on_circle.h>

bool EdgePointOnCircle::read(std::istream& is)
{
  std::cerr << __PRETTY_FUNCTION__ << " not implemented yet " << std::endl;
  return false;
}

bool EdgePointOnCircle::write(std::ostream& os) const
{
  std::cerr << __PRETTY_FUNCTION__ << " not implemented yet " << std::endl;
  return false;
}

void EdgePointOnCircle::computeError()
{
  const VertexCircle* circle = static_cast<const VertexCircle*>(vertex(0));

  const Eigen::Vector2d& center = circle->estimate().head<2>();
  const double& radius = circle->estimate()(2);

  _error(0) = (measurement() - center).norm() - radius;
}
