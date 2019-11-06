#include <slam/g2o/edge/edge_landmark.h>

namespace g2o
{
void EdgeLandmark::computeError()
{
  const auto *pose = static_cast<const VertexRobotState *>(_vertices[0]);
  const auto *landmark_position = static_cast<const VertexPointXY *>(_vertices[1]);

  _error = _measurement - (pose->estimate().se2().inverse() * landmark_position->estimate());
}

bool EdgeLandmark::read(std::istream &is)
{
  is >> _measurement[0] >> _measurement[1];
  is >> information()(0, 0) >> information()(0, 1) >> information()(1, 1);
  information()(1, 0) = information()(0, 1);

  return true;
}

bool EdgeLandmark::write(std::ostream &os) const
{
  os << measurement()[0] << " " << measurement()[1] << " ";
  os << information()(0, 0) << " " << information()(0, 1) << " " << information()(1, 1);

  return os.good();
}
}  // namespace g2o
