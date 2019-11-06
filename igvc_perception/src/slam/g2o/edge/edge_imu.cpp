#include <slam/g2o/edge/edge_imu.h>

namespace g2o
{
void EdgeIMU::computeError()
{
  const auto *v1 = static_cast<const VertexRobotState *>(_vertices[0]);
  const auto *v2 = static_cast<const VertexRobotState *>(_vertices[1]);

  const auto& v1_twist = v1->estimate().twist();
  const auto& v2_twist = v2->estimate().twist();

  _error(0) = (v2_twist.linear() - v1_twist.linear()) - delta_v_;
  _error(1) = (v1_twist.angular() + v2_twist.angular()) / 2 - _measurement.angular_velocity();
}

void EdgeIMU::setMeasurement(const IMUMeasurement &m)
{
  _measurement = m;
  delta_v_ = m.delta_v();
}

bool EdgeIMU::read(std::istream &is)
{
  Eigen::Vector3d p;
  is >> p[0] >> p[1] >> p[2];
  _measurement.fromVector(p);
  delta_v_ = measurement().delta_v();

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      is >> information()(i, j);
      if (i != j)
        information()(j, i) = information()(i, j);
    }
  }

  return true;
}

bool EdgeIMU::write(std::ostream &os) const
{
  Eigen::Vector3d p = measurement().toVector();
  os << p.x() << " " << p.y() << " " << p.z();

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      os << " " << information()(i, j);
    }
  }

  return os.good();
}
}  // namespace g2o
