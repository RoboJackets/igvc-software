#include <slam/g2o/vertex/vertex_robot_state.h>
#include <ros/ros.h>

namespace g2o
{
VertexRobotState::VertexRobotState() : BaseVertex<5, RobotState>()
{
}

void VertexRobotState::setToOriginImpl()
{
  _estimate = RobotState();
}

bool VertexRobotState::read(std::istream &is)
{
  Vector6d p;
  is >> p(0) >> p(1) >> p(2) >> p(3) >> p(4) >> p(5);
  _estimate.fromVector(p);

  return true;
}

bool VertexRobotState::write(std::ostream &os) const
{
  Vector6d p = estimate().toVector();

  os << p(0) << " " << p(1) << " " << p(2) << " " << p(3) << " " << p(4) << " " << p(5);

  return os.good();
}

void VertexRobotState::oplusImpl(const number_t *update)
{
  _estimate.update({update[0], update[1], update[2]}, {update[3], update[4]});
}
}  // namespace g2o
