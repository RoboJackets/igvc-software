#include <cmath>

/**
 * Templated interface the Some controller.
 *
 * Author: Oswin So <oswinso@gmail.com>
 * Date Created: April 13th, 2019
 */
#ifndef SRC_SOME_CONTROLLER_H
#define SRC_SOME_CONTROLLER_H

#include <memory>
#include <vector>
#include <iostream>
#include <cmath>

#include "cost_function.h"
#include "model.h"

namespace some_controller
{
template <class Model, class CostFunction>
class SomeController
{
public:
  using Controls = typename Model::Controls;
  using State = typename Model::StateType;

  SomeController(std::shared_ptr<Model> model, std::shared_ptr<CostFunction> cost_function, float timestep,
                 float horizon, int samples);
  std::vector<Controls> optimize();

private:
  std::shared_ptr<Model> model_;
  std::shared_ptr<CostFunction> cost_function_;
  float timestep_;
  float horizon_;
  int iterations_;
  int samples_;
};

template <class Model, class CostFunction>
SomeController<Model, CostFunction>::SomeController(std::shared_ptr<Model> model,
                                                             std::shared_ptr<CostFunction> cost_function,
                                                             float timestep, float horizon, int samples)
  : model_{ model }
  , cost_function_{ cost_function }
  , timestep_{ timestep }
  , horizon_{ horizon }
  , iterations_{ static_cast<int>(std::round(horizon / timestep)) }, samples_{ samples }
{
}

template <class Model, class CostFunction>
std::vector<typename Model::Controls> SomeController<Model, CostFunction>::optimize()
{
  for (int i = 0; i < 10; i++)
  {
    std::cout << i << std::endl;
  }
  std::array<Bound, Model::control_dims> bounds = model_->getBounds();
  for (const Bound& bound : bounds)
  {
    std::cout << "lower: " << bound.lower << ", upper: " << bound.upper << std::endl;
  }
  return std::vector<Controls>();
}

}  // namespace some_controller

#endif  // SRC_SOME_CONTROLLER_H
