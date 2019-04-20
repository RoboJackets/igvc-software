#include "differential_drive_model.h"

using namespace differential_drive_model;

std::array<Bound, ControlDims> differential_drive_model::DifferentialDriveModel::getBounds()
{
  return { { acceleration_bound_, acceleration_bound_ } };
}

RobotState differential_drive_model::DifferentialDriveModel::propogateState(RobotState state,
                                                                            const Model::Controls& controls, float dt)
{
  // TODO: Change to trapezoidal rule
  bool before = isnan(state.velocity());
  state.wheel_velocity_.left += controls[0] * dt;
  state.wheel_velocity_.right += controls[1] * dt;

  if (!before && isnan(state.velocity())) {
    ROS_ERROR_STREAM_THROTTLE(0.2, "WTF? left: " << state.wheel_velocity_.left << ", right: " << state.wheel_velocity_.right);
    ROS_ERROR_STREAM_THROTTLE(0.2, "controls: (" << controls[0] << ", " << controls[1] << "), dt: " << dt);
  }

  float left = state.wheel_velocity_.left;
  float right = state.wheel_velocity_.right;

  float k = (right - left) / axle_length_;
  float v = (right + left) / 2;

  float w = k * v;

  if (std::abs(w) > 1e-10)
  {
    // calculate instantaneous center of curvature (ICC = [ICCx, ICCy])
    float R = 1 / k;
    float ICCx = state.x() - (R * sin(state.yaw()));
    float ICCy = state.y() + (R * cos(state.yaw()));

    using namespace Eigen;
    Matrix3d T;
    double wdt = w * dt;
    T << cos(wdt), -sin(wdt), 0, sin(wdt), cos(wdt), 0, 0, 0, 1;
    Vector3d a(state.x() - ICCx, state.y() - ICCy, state.yaw());
    Vector3d b = T * a;
    Vector3d c = b + Vector3d(ICCx, ICCy, wdt);

    state.set_x(c[0]);
    state.set_y(c[1]);
    state.set_yaw(c[2]);
  }
  else
  {
    state.set_x(state.x()+ cos(state.yaw()) * v * dt);
    state.set_y(state.y() + cos(state.yaw()) * v * dt);
  }

  return state;
}

DifferentialDriveModel::DifferentialDriveModel(const DifferentialDriveOptions& options)
  : acceleration_bound_{ options.acceleration_bound }, axle_length_{ options.axle_length }
{
}
