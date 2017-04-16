#ifndef SEARCHPROBLEM_HPP_INCLUDED
#define SEARCHPROBLEM_HPP_INCLUDED

#include <algorithm>
#include <iterator>
#include <list>
#include <stdexcept>

template <class StateType, class ActionType>
class Path
{
private:
  std::list<StateType> states;
  std::list<ActionType> actions;

public:
  Path()
  {
  }
  Path(const Path<StateType, ActionType>& p) : states(p.states), actions(p.actions)
  {
  }
  void addState(StateType state)
  {
    states.push_back(state);
  }
  void addAction(ActionType action)
  {
    actions.push_back(action);
  }
  void setState(StateType state, int index)
  {
    if (index < 0 || index >= states.size())
      throw std::out_of_range("Index out of bounds!");
    states[index] = state;
  }
  void setAction(ActionType action, int index)
  {
    if (index < 0 || index >= actions.size())
      throw std::out_of_range("Index out of bounds!");
    actions[index] = action;
  }
  StateType getState(unsigned int index) const
  {
    if (index < 0 || index >= states.size())
      throw std::out_of_range("Index out of bounds!");

    auto iter = states.cbegin();
    std::advance(iter, index);
    return *iter;
  }
  ActionType getAction(unsigned int index) const
  {
    if (index < 0 || index >= actions.size())
      throw std::out_of_range("Index out of bounds!");
    auto iter = actions.cbegin();
    std::advance(iter, index);
    return *iter;
  }
  std::list<StateType>* getStates()
  {
    return &states;
  }
  std::list<ActionType>* getActions()
  {
    return &actions;
  }
  StateType getLastState() const
  {
    if (states.empty())
      throw std::out_of_range("Cannot call back() on empty list.");
    return states.back();
  }
  int getNumberOfSteps() const
  {
    return actions.size();
  }
  bool containsState(StateType state) const
  {
    return (find(states.cbegin(), states.cend(), state) != states.cend());
  }
};

template <class StateType, class ActionType>
class SearchProblem
{
public:
  virtual StateType getStartState() = 0;
  virtual std::list<ActionType> getActions(StateType state, StateType robot_position) = 0;
  virtual StateType getResult(StateType state, ActionType action) = 0;
  virtual bool isGoal(StateType tate) = 0;
  virtual double getStepCost(StateType state, ActionType action)
  {
    (void)state;
    (void)action;
    return 1;
  }
  virtual double getHeuristicCost(StateType state)
  {
    (void)state;
    return 0;
  }
  double getPathCost(Path<StateType, ActionType>* path)
  {
    double cost = 0;

    for (int i = 0; i < path->getNumberOfSteps(); i++)
    {
      cost += getStepCost(path->getState(i), path->getAction(i));
    }

    return cost;
  }
};

#endif  // SEARCHPROBLEM_HPP_INCLUDED
