#ifndef IGVCSEARCHPROBLEM_H
#define IGVCSEARCHPROBLEM_H

#include <cv_bridge/cv_bridge.h>
#include <math.h>
#include <pcl/octree/octree_search.h>
#include <functional>
#include <opencv2/core/core.hpp>
#include <vector>
#include "GraphSearch.hpp"
#include "searchlocation.h"
#include "searchmove.h"

class IGVCSearchProblem : public SearchProblem<SearchLocation, SearchMove>
{
public:
  // make sure these follow style
  cv_bridge::CvImageConstPtr Map;
  SearchLocation Start;
  SearchLocation Goal;
  double Resolution;
  double CSpace;
  double GoalThreshold;
  double ProbabilityThreshold;
  bool PointTurnsEnabled;
  bool ReverseEnabled;
  // TODO distance to goal from start
  double DistanceToGoal;
  double MaxJumpSize;
  double ThetaFilter;
  double MaxThetaChange;
  double ThetaChangeWindow;
  double HeuristicInflation;
  double MaximumDistance;

  SearchLocation getStartState()
  {
    return Start;
  }

  std::list<SearchMove> getActions(const SearchLocation& state);

  SearchLocation getResult(const SearchLocation& state, const SearchMove& action);

  bool isGoal(const SearchLocation& state)
  {
    // TODO consider options
    return state.distTo(Goal, Resolution) < GoalThreshold || state.distTo(Start, Resolution) > MaximumDistance;
  }

  double getStepCost(const SearchLocation& location, const SearchMove& action)
  {
    SearchLocation result;
    result.X = location.X + action.X;
    result.Y = location.Y + action.Y;
    return result.distTo(location, Resolution);
  }

  double getHeuristicCost(const SearchLocation& state)
  {
    return state.distTo(Goal, Resolution) * HeuristicInflation;
  }

  bool isActionValid(const SearchMove& move, const SearchLocation& start_state);
};

#endif  // IGVCSEARCHPROBLEM_H
