/**
 * Templated interface for model to the Some controller.
 *
 * Author: Oswin So <oswinso@gmail.com>
 * Date Created: April 13th, 2019
 */
#ifndef SRC_MODEL_H
#define SRC_MODEL_H

#include <array>

namespace trajectory_controller
{
struct Bound
{
  float lower;
  float upper;
};

template <class State, int ControlDims, class Derived>
class Model
{
public:
  using Controls = std::array<float, ControlDims>;

  /**
   * Get the bounds for sampling for each control dimension.
   * @return array of Bounds, one for each control dimension.
   */
  std::array<Bound, ControlDims> getBounds();

  /**
   * Propogates the state forward given the current state and current controls.
   * @param state current state
   * @param controls controls to be applied
   * @param dt timstep
   * @return next state after state propogation.
   */
  State propogateState(State state, const Controls& controls, float dt);
private:
  Derived& derived();
};

  template<class State, int ControlDims, class Derived>
  std::array<Bound, ControlDims> Model<State, ControlDims, Derived>::getBounds()
  {
    static_assert(false, "getBounds must be implemented.");
    return std::array<Bound, ControlDims>();
  }

  template<class State, int ControlDims, class Derived>
  State Model<State, ControlDims, Derived>::propogateState(State state, const Controls& controls, float dt)
  {
    static_assert(false, "propogateState must be implemented.");
    return nullptr;
  }

  template<class State, int ControlDims, class Derived>
  Derived& Model<State, ControlDims, Derived>::derived()
  {
    return *static_cast<Derived*>(this);
  }

}  // namespace trajectory_controller

#endif  // SRC_MODEL_H
