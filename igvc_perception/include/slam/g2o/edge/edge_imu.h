#ifndef SRC_EDGE_IMU_H
#define SRC_EDGE_IMU_H

#include <g2o/core/base_binary_edge.h>
#include <slam/g2o/vertex/vertex_robot_state.h>
#include <slam/g2o/IMU_measurement.h>

namespace g2o
{
class EdgeIMU : public BaseBinaryEdge<2, IMUMeasurement, VertexRobotState, VertexRobotState>
{
public:
  EdgeIMU() = default;
  void computeError() override;
  void setMeasurement(const IMUMeasurement& m) override;

  bool read(std::istream& is) override;
  bool write(std::ostream& os) const override;

private:
  double delta_v_{};
};
}  // namespace g2o

#endif  // SRC_EDGE_IMU_H
