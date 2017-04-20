#include "igvcsearchproblem.h"
#include <math.h>

SearchLocation robot_position;

bool IGVCSearchProblem::isActionValid(SearchMove& move, SearchLocation start_state)
{
  /*auto deltat = move.DeltaT;
  double current = 0.0;
  if(abs(pow(start_state.x - robot_position.x, 2) + pow(start_state.y - robot_position.y, 2)) > 16) {
    return true;
  }
  while (current < (deltat + maxODeltaT))
  {
    current = current > deltat ? deltat : (current + maxODeltaT);
    move.DeltaT = current;
    SearchLocation result = getResult(start_state, move);
    double offsetToCenter = 0.33;
    pcl::PointXYZ searchPoint(result.x + offsetToCenter * cos(result.theta),
                              result.y + offsetToCenter * sin(result.theta), 0);
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    int neighborsCount = kdtree.nearestKSearch(searchPoint, 1, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    if (neighborsCount > 0)
    {
      double temp = pow(pointRadiusSquaredDistance[0], .5);
      if (temp < move.distToObs)
      {
        move.distToObs = temp;
      }
      if (move.distToObs <= Threshold)
      {
        return false;
      }
    }
  }*/
  auto deltat = move.DeltaT;
  double current = 0.0;
  int maximum_index = occupancy_grid_size / square_size;
  while (current < (deltat + maxODeltaT))
  {
    current = current > deltat ? deltat : (current + maxODeltaT);
    move.DeltaT = current;
    SearchLocation result = getResult(start_state, move);
    int x = (result.x - robot_position.x) / square_size + maximum_index / 2;
    int y = (result.y - robot_position.y) / square_size + maximum_index / 2;
    if (x < maximum_index && x > 0 && y < maximum_index && y > 0 && occupancy_grid[x][y] == 255)
    {
      return false;
    } else if(x < maximum_index && x > 0 && y < maximum_index && y > 0 && occupancy_grid[x][y] != 0 && occupancy_grid[x][y] != 255) {
      move.distToObs = occupancy_grid[x][y] / 255 * Threshold + Threshold;
    }
  }
  return true;
}

std::list<SearchMove> IGVCSearchProblem::getActions(SearchLocation state, SearchLocation local_robot_position)
{
  // pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  std::list<SearchMove> acts;
  robot_position = local_robot_position;
  auto deltat = DeltaT(state.distTo(Start), state.distTo(Goal));
  if (Map == nullptr)
    return acts;
  /*if (!Map->empty())
  {
    kdtree.setInputCloud(Map);
  }*/
  double delta = DeltaOmega;
  double Wmin = MinimumOmega;
  double Wmax = MaximumOmega;
  for (double W = Wmin; W <= Wmax; W += delta)
  {
    SearchMove move(Speed, W, deltat);
    if (Map->empty() || isActionValid(move, state))
    {
      acts.push_back(move);
    }
  }
  if (ReverseEnabled && acts.size() == 0)
  {
    for (double W = Wmin; W <= Wmax; W += delta)
    {
      SearchMove move = SearchMove(-Speed, W, deltat);
      if (Map->empty() || isActionValid(move, state))
      {
        acts.push_back(move);
      }
    }
  }
  if (PointTurnsEnabled)
  {
    SearchMove move(0, TurningSpeed, deltat);
    if (Map->empty() || isActionValid(move, state))
    {
      acts.push_back(move);
    }
    move = SearchMove(0, -TurningSpeed, deltat);
    if (isActionValid(move, state))
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
    result.x = state.x + (sin(M_PI_2 - result.theta) * action.V * action.DeltaT);
    result.y = state.y + (cos(M_PI_2 - result.theta) * action.V * action.DeltaT);
  }
  return result;
}
