/**
 * Templated interface for some cost function to the Some controller.
 *
 * Author: Oswin So <oswinso@gmail.com>
 * Date Created: April 13th, 2019
 */
#ifndef SRC_COST_FUNCTION_H
#define SRC_COST_FUNCTION_H

#include <array>
namespace some_controller {
template <class State, int ControlDims, class Derived>
class CostFunction
{
public:
  using Controls = std::array<float, ControlDims>;
  float getCost(const State& state, const Controls& controls);
  float cost(const State& state, const Controls& controls);
private:
  Derived& derived();
};

  template<class State, int ControlDims, class Derived>
  float CostFunction<State, ControlDims, Derived>::getCost(const State& state, const Controls& controls)
  {
    std::cout << "Rip parent getCost" << std::endl;
//    static_assert(false, "getCost must be implemented.");
    return 0;
  }

template<class State, int ControlDims, class Derived>
float CostFunction<State, ControlDims, Derived>::cost(const State& state, const Controls& controls)
{
  return derived().getCost();
}

  template<class State, int ControlDims, class Derived>
  Derived& CostFunction<State, ControlDims, Derived>::derived()
  {
    return *static_cast<Derived*>(this);
  }
}

#endif //SRC_COST_FUNCTION_H
