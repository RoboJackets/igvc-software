#ifndef IGVCSEARCHPROBLEM_H
#define IGVCSEARCHPROBLEM_H

#include <math.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <functional>
#include <vector>
#include "GraphSearch.hpp"
#include "searchlocation.h"
#include "searchmove.h"

class IGVCSearchProblem : public SearchProblem<SearchLocation, SearchMove>
{
public:
  pcl::PointCloud<pcl::PointXYZ>::Ptr Map;
  SearchLocation Start;
  SearchLocation Goal;
  double Threshold;
  double Speed;
  double TurningSpeed;
  std::function<double(double)> DeltaT;
  double Baseline;
  double GoalThreshold;
  bool PointTurnsEnabled;
  bool ReverseEnabled;
  double MinimumOmega;
  double MaximumOmega;
  double DeltaOmega;
  double maxODeltaT;
  double alpha = 4;
  double beta = 5;

  SearchLocation getStartState()
  {
    return Start;
  }
  std::list<SearchMove> getActions(SearchLocation state);

  SearchLocation getResult(SearchLocation state, SearchMove action);

  bool isGoal(SearchLocation state)
  {
    return state.distTo(Goal) < GoalThreshold || state.distTo(Start) > 10;
  }

  double getStepCost(SearchLocation, SearchMove action)
  {
    double pathLength = 0;
    if (action.W <= 1e-10)
    {
      pathLength = action.V * action.DeltaT;
    }
    else
    {
      double R = abs(action.V) / abs(action.W);
      double theta = abs(action.W) * action.DeltaT;
      pathLength = R * theta;
    }
    if (action.distToObs < 10)
    {
      /*cerr << action.distToObs << endl;
      cerr << alpha*exp((Threshold-action.distToObs)*beta)+1 << endl;
      cerr << "----------" << endl;*/
    }
    return pathLength * (alpha * exp((Threshold - action.distToObs) * beta) +
                         1);  // adding cost to paths close to threshold TODO: LAUNCH PARAMETERS!
  }

  double getHeuristicCost(SearchLocation state)
  {
    return state.distTo(Goal);
  }

  bool isActionValid(SearchMove& move, pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree, SearchLocation start_state);
};

#endif  // IGVCSEARCHPROBLEM_H
