#ifndef IGVCSEARCHPROBLEM_H
#define IGVCSEARCHPROBLEM_H

#include "searchlocation.h"
#include "searchmove.h"
#include "GraphSearch.hpp"
#include <vector>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>

class IGVCSearchProblem : public SearchProblem<SearchLocation, SearchMove>
{
public:

    pcl::PointCloud<pcl::PointXYZ> Map;
    SearchLocation Start;
    SearchLocation Goal;
    double Threshold;
    double Speed;
    double TurningSpeed;
    double DeltaT;
    double Baseline;
    double GoalThreshold;
    bool PointTurnsEnabled;

    SearchLocation getStartState()
    {
        return Start;
    }
    std::list<SearchMove> getActions(SearchLocation state);

    SearchLocation getResult(SearchLocation state, SearchMove action);

    bool isGoal(SearchLocation state)
    {
        return state.distTo(Goal) < GoalThreshold;
    }

    double getStepCost(SearchLocation, SearchMove action)
    {
        double R = abs(action.V) / abs(action.W);
        double theta = abs(action.W) * DeltaT;
        return R*theta;
    }

    double getHeuristicCost(SearchLocation state)
    {
        return state.distTo(Goal);
    }
};

#endif // IGVCSEARCHPROBLEM_H
