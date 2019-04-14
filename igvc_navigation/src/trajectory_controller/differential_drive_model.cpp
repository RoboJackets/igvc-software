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
  state.wheel_velocity_.left += controls[0] * dt;
  state.wheel_velocity_.right += controls[1] * dt;

  std::cout << "  {" << state.wheel_velocity_.left << ", " << state.wheel_velocity_.right << "}  ";

  float left = state.wheel_velocity_.left;
  float right = state.wheel_velocity_.right;

  float k = (right - left) / axle_length_;
  float v = (right + left) / 2;

  float w = k * v;

  if (std::abs(w) > 1e-10)
  {
    // calculate instantaneous center of curvature (ICC = [ICCx, ICCy])
    float R = 1 / k;
    float ICCx = state.x - (R * sin(state.yaw));
    float ICCy = state.y + (R * cos(state.yaw));

    using namespace Eigen;
    Matrix3d T;
    double wdt = w * dt;
    T << cos(wdt), -sin(wdt), 0, sin(wdt), cos(wdt), 0, 0, 0, 1;
    Vector3d a(state.x - ICCx, state.y - ICCy, state.yaw);
    Vector3d b = T * a;
    Vector3d c = b + Vector3d(ICCx, ICCy, wdt);

    state.x = c[0];
    state.y = c[1];
    state.yaw = c[2];
    igvc::fit_to_polar(state.yaw);
  }
  else
  {
    std::cout << "LINEw("<<w<<")";
    state.x = state.x + cos(state.yaw) * v * dt;
    state.y = state.y + cos(state.yaw) * v * dt;
  }

  return state;
}

DifferentialDriveModel::DifferentialDriveModel(Bound acceleration_bound, float axle_length)
  : acceleration_bound_{ acceleration_bound }, axle_length_{ axle_length }
{
}
