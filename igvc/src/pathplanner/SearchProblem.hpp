#ifndef SEARCHPROBLEM_HPP_INCLUDED
#define SEARCHPROBLEM_HPP_INCLUDED

#include <list>
#include <algorithm>
#include <iterator>

template <class StateType, class ActionType>
class Path
{
private:
    std::list<StateType> states;
    std::list<ActionType> actions;
public:
    Path(){ }
    Path(const Path<StateType, ActionType>& p)
    {
        states = std::list<StateType>(p.states);
        actions = std::list<ActionType>(p.actions);
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
        if(index < 0 || index >= states.size())
            throw "Index out of bounds!";
        states[index] = state;
    }
    void setAction(ActionType action, int index)
    {
        if(index < 0 || index >= actions.size())
            throw "Index out of bounds!";
        actions[index] = action;
    }
    StateType getState(unsigned int index)
    {
        if(index < 0 || index >= states.size())
            throw "Index out of bounds!";

        typename std::list<StateType>::iterator iter = states.begin();
        std::advance(iter, index);
        return *iter;
    }
    ActionType getAction(unsigned int index)
    {
        if(index < 0 || index >= actions.size())
            throw "Index out of bounds!";
        typename std::list<ActionType>::iterator iter = actions.begin();
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
    StateType getLastState()
    {
        return states.back();
    }
    int getNumberOfSteps()
    {
        return actions.size();
    }
    bool containsState(StateType state)
    {
        return (find(states.begin(), states.end(), state) != states.end());
    }
};

template <class StateType, class ActionType>
class SearchProblem
{
public:
    virtual StateType getStartState() = 0;
    virtual std::list<ActionType> getActions(StateType state) = 0;
    virtual StateType getResult(StateType state, ActionType action) = 0;
    virtual bool isGoal(StateType tate) = 0;
    virtual double getStepCost(StateType state, ActionType action) { (void)state; (void)action; return 1; }
    virtual double getHeuristicCost(StateType state) { (void)state; return 0; }
    double getPathCost(Path<StateType, ActionType>* path)
    {
        double cost = 0;

        for(int i=0; i < path->getNumberOfSteps(); i++)
        {
            cost += getStepCost(path->getState(i), path->getAction(i));
        }

        return cost;
    }
};

#endif // SEARCHPROBLEM_HPP_INCLUDED
