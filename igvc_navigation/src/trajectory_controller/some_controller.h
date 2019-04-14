#include <cmath>

/**
 * Templated interface the Some controller.
 *
 * Author: Oswin So <oswinso@gmail.com>
 * Date Created: April 13th, 2019
 */
#ifndef SRC_SOME_CONTROLLER_H
#define SRC_SOME_CONTROLLER_H

#include <cmath>
#include <iostream>
#include <memory>
#include <random>
#include <vector>

#include "cost_function.h"
#include "model.h"

namespace some_controller
{
template <class Model>
struct Particle
{
  std::vector<typename Model::Controls> controls_vec_{};
  std::vector<typename Model::StateType> state_vec_{};
  float cum_cost_{ 0.0f };

public:
  void initialize(const typename Model::StateType& initial_state);
  typename Model::StateType getState() const;
};

template <class Model>
void Particle<Model>::initialize(const typename Model::StateType& initial_state)
{
  controls_vec_.clear();
  state_vec_.clear();
  cum_cost_ = 0;
  state_vec_.emplace_back(initial_state);
}

template <class Model>
typename Model::StateType Particle<Model>::getState() const
{
  return state_vec_.back();
}

template <class Model, class CostFunction>
class SomeController
{
public:
  using Controls = typename Model::Controls;
  using State = typename Model::StateType;
  constexpr static const int control_dims = Model::control_dims;

  SomeController(std::shared_ptr<Model> model, std::shared_ptr<CostFunction> cost_function, float timestep,
                 float horizon, int num_samples);
  std::vector<Controls> optimize(const State& starting_state);

private:
  void initializeParticles(const State& starting_state);
  Controls sampleControls() const;

  std::shared_ptr<Model> model_;
  std::shared_ptr<CostFunction> cost_function_;
  float timestep_;
  float horizon_;
  int iterations_;
  int num_samples_;

  std::random_device rd_;
  mutable std::mt19937 mt_;
  std::array<std::uniform_real_distribution<float>, control_dims> distributions_;

  std::vector<Particle<Model>> particles_;
};

template <class Model, class CostFunction>
SomeController<Model, CostFunction>::SomeController(std::shared_ptr<Model> model,
                                                    std::shared_ptr<CostFunction> cost_function, float timestep,
                                                    float horizon, int num_samples)
  : model_{ model }
  , cost_function_{ cost_function }
  , timestep_{ timestep }
  , horizon_{ horizon }
  , iterations_{ static_cast<int>(std::round(horizon / timestep)) }
  , num_samples_{ num_samples }
  , rd_{}
  , mt_{ rd_() }
  , particles_(num_samples, Particle<Model>{})
{
  std::array<Bound, control_dims> bounds = model_->getBounds();
  for (int i = 0; i < control_dims; i++)
  {
    Bound bound = bounds[i];
    distributions_[i] = std::uniform_real_distribution<float>(bound.lower, bound.upper);
  }
}

template <class Model, class CostFunction>
std::vector<typename Model::Controls> SomeController<Model, CostFunction>::optimize(const State& starting_state)
{
  initializeParticles(starting_state);

  for (int i = 0; i < iterations_; i++)
  {
    for (Particle<Model>& particle : particles_)
    {
      Controls controls = sampleControls();
      State state = particle.getState();
      State new_state = model_->doPropogateState(state, controls, timestep_);
      float cost = cost_function_->cost(new_state, controls);

      particle.cum_cost_ += cost;
      particle.controls_vec_.emplace_back(controls);
      particle.state_vec_.emplace_back(new_state);
    }
  }

  Particle<Model>& optimal_particle =
      *std::min_element(particles_.begin(), particles_.end(), [](const Particle<Model>& p1, const Particle<Model>& p2) {
        return p1.cum_cost_ < p2.cum_cost_;
      });
  return optimal_particle.controls_vec_;
}

template <class Model, class CostFunction>
typename Model::Controls SomeController<Model, CostFunction>::sampleControls() const
{
  Controls controls;
  for (int i = 0; i < control_dims; i++)
  {
    std::uniform_real_distribution distribution = distributions_[i];
    controls[i] = distribution(mt_);
  }
  return controls;
}

template <class Model, class CostFunction>
void SomeController<Model, CostFunction>::initializeParticles(const State& starting_state)
{
  for (Particle<Model>& particle : particles_)
  {
    particle.initialize(starting_state);
  }
}

}  // namespace some_controller

#endif  // SRC_SOME_CONTROLLER_H
