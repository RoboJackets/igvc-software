#include "igvcsearchproblem.h"
#include <math.h>

SearchLocation robot_position;

bool IGVCSearchProblem::isActionValid(SearchMove& move, SearchLocation start_state)
{
  auto deltat = move.DeltaT;
  double current = 0.0;
  if (sqrt(pow(start_state.x - robot_position.x, 2) + pow(start_state.y - robot_position.y, 2)) > BoundingDistance)
  {
    return true;
  }
  while (current < (deltat + MaxObstacleDeltaT))
  {
    current = current > deltat ? deltat : (current + MaxObstacleDeltaT);
    move.DeltaT = current;
    SearchLocation result = getResult(start_state, move);
    double offsetToCenter = 0.33;
    pcl::PointXYZ searchPoint(result.x + offsetToCenter * cos(result.theta),
                              result.y + offsetToCenter * sin(result.theta), 0);
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    int neighborsCount = Octree->nearestKSearch(searchPoint, 1, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    if (neighborsCount > 0)
    {
      float distance = sqrt(pointRadiusSquaredDistance[0]);
      if (distance < move.distToObs)
      {
        move.distToObs = distance;
      }
      if (move.distToObs <= Threshold)
      {
        return false;
      }
    }
  }
  return true;
}

std::list<SearchMove> IGVCSearchProblem::getActions(SearchLocation state, SearchLocation local_robot_position)
{
  std::list<SearchMove> acts;
  robot_position = local_robot_position;
  auto deltat = DeltaT(state.distTo(Start), state.distTo(Goal));
  double delta = DeltaOmega;
  double Wmin = MinimumOmega;
  double Wmax = MaximumOmega;
  for (double W = Wmin; W <= Wmax; W += delta)
  {
    SearchMove move(Speed, W, deltat);
    if (Octree->getTreeDepth() == 0 || isActionValid(move, state))
    {
      acts.push_back(move);
    }
  }
  if (ReverseEnabled && acts.size() == 0)
  {
    for (double W = Wmin; W <= Wmax; W += delta)
    {
      SearchMove move = SearchMove(-Speed, W, deltat);
      if (Octree->getTreeDepth() == 0 || isActionValid(move, state))
      {
        acts.push_back(move);
      }
    }
  }
  if (PointTurnsEnabled)
  {
    SearchMove move(0, TurningSpeed, deltat);
    if (Octree->getTreeDepth() == 0 || isActionValid(move, state))
    {
      acts.push_back(move);
    }
    move = SearchMove(0, -TurningSpeed, deltat);
    if (Octree->getTreeDepth() == 0 || isActionValid(move, state))
    {
      acts.push_back(move);
    }
  }
  return acts;
}

SearchLocation IGVCSearchProblem::getResult(SearchLocation state, SearchMove action)
{
  SearchLocation result;
  if (abs(action.W) > 1e-10)
  {
    double w = action.W;
    double R = action.V / action.W;
    double ICCx = state.x - (R * sin(state.theta));
    double ICCy = state.y - (R * cos(state.theta));
    using namespace Eigen;
    Matrix3d T;
    double wdt = w * action.DeltaT;
    T << cos(wdt), sin(wdt), 0, -sin(wdt), cos(wdt), 0, 0, 0, 1;
    Vector3d a(state.x - ICCx, state.y - ICCy, state.theta);
    Vector3d b = T * a;
    Vector3d c = b + Vector3d(ICCx, ICCy, wdt);
    // Vector3d b(ICCx, ICCy, wdt);
    // Vector3d c = T * a + b;
    result.x = c[0];
    result.y = c[1];
    result.theta = c[2];
    while (result.theta < 0)
      result.theta += 2 * M_PI;
    while (result.theta > 2 * M_PI)
      result.theta -= 2 * M_PI;
  }
  else
  {
    result.theta = state.theta;
    result.x = state.x + (cos(-result.theta) * action.V * action.DeltaT);
    result.y = state.y + (sin(-result.theta) * action.V * action.DeltaT);
  }
  return result;
}
