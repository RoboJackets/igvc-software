/**
 * Templated interface for model to the Some controller.
 *
 * Author: Oswin So <oswinso@gmail.com>
 * Date Created: April 13th, 2019
 */
#ifndef SRC_MODEL_H
#define SRC_MODEL_H

#include <array>
#include <iostream>

namespace some_controller
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
  using StateType = State;
  constexpr static const int control_dims = ControlDims;

  using Controls = std::array<float, ControlDims>;

  /**
   * Get the bounds for sampling for each control dimension.
   * @return array of Bounds, one for each control dimension.
   */
  std::array<Bound, ControlDims> getBounds();
  std::array<Bound, ControlDims> bounds();

  /**
   * Propogates the state forward given the current state and current controls.
   * @param state current state
   * @param controls controls to be applied
   * @param dt timstep
   * @return next state after state propogation.
   */
  State propogateState(State state, const Controls& controls, float dt);
  State doPropogateState(State state, const Controls& controls, float dt);

private:
  Derived& derived();
};

template <class State, int ControlDims, class Derived>
std::array<Bound, ControlDims> Model<State, ControlDims, Derived>::getBounds()
{
  std::cout << "rip parent getBounds" << std::endl;
  //    static_assert(false, "getBounds must be implemented.");
  return std::array<Bound, ControlDims>();
}

template <class State, int ControlDims, class Derived>
std::array<Bound, ControlDims> Model<State, ControlDims, Derived>::bounds()
{
  return derived().getBounds();
}

template <class State, int ControlDims, class Derived>
State Model<State, ControlDims, Derived>::propogateState(State state, const Controls& controls, float dt)
{
  std::cout << "rip parent propogateState" << std::endl;
  //    static_assert(false, "propogateState must be implemented.");
  return nullptr;
}

template <class State, int ControlDims, class Derived>
State Model<State, ControlDims, Derived>::doPropogateState(State state, const Controls& controls, float dt)
{
  return derived().propogateState();
}

template <class State, int ControlDims, class Derived>
Derived& Model<State, ControlDims, Derived>::derived()
{
  return *static_cast<Derived*>(this);
}

}  // namespace some_controller

#endif  // SRC_MODEL_H
