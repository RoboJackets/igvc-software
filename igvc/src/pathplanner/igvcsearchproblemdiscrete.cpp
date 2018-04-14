#include "igvcsearchproblemdiscrete.h"
#include <math.h>

bool IGVCSearchProblemDiscrete::isActionValid(SearchMove& move, SearchLocation start_state)
{
  double x = start_state.X + move.X;
  double y = start_state.Y + move.Y;
  std::cout << "x = " << x << " y = " << y;
  std::cout << " width = " << Map->image.size().width << " height = " << Map->image.size().height << " val " << static_cast<int>(Map->image.at<uchar>(x, y));
  if(x < 0 || y < 0 || x >= Map->image.size().width || y >= Map->image.size().height || Map->image.at<uchar>(x, y) > (255 * CSpace)) {
    cout << " false" << std::endl;
    return false;
  }
  cout << " true" << std::endl;
  return true;
}

std::list<SearchMove> IGVCSearchProblemDiscrete::getActions(SearchLocation state, SearchLocation robot_position)
{
  std::cout << state.X << "," << state.Y << "," << state.Theta << std::endl;
  std::list<SearchMove> acts;
  double theta = state.Theta;
  double thetaThreshold = 0.1;
  if(theta == 0) {
    acts.push_back(SearchMove(1, 1));
    acts.push_back(SearchMove(1, 0));
    acts.push_back(SearchMove(1, -1));
  } else if(theta - M_PI / 4 < thetaThreshold) {
    acts.push_back(SearchMove(1, 1));
    acts.push_back(SearchMove(1, 0));
    acts.push_back(SearchMove(0, 1));
  } else if(theta - M_PI / 2 < thetaThreshold) {
    acts.push_back(SearchMove(1, 1));
    acts.push_back(SearchMove(0, 1));
    acts.push_back(SearchMove(-1, 1));
  } else if(theta - 3 * M_PI / 4 < thetaThreshold) {
    acts.push_back(SearchMove(0, 1));
    acts.push_back(SearchMove(-1, 1));
    acts.push_back(SearchMove(-1, 0));
  } else if(theta - M_PI < thetaThreshold) {
    acts.push_back(SearchMove(-1, 0));
    acts.push_back(SearchMove(-1, 1));
    acts.push_back(SearchMove(-1, -1));
  } else if(theta - 5 * M_PI / 4 < thetaThreshold) {
    acts.push_back(SearchMove(-1, -1));
    acts.push_back(SearchMove(-1, 0));
    acts.push_back(SearchMove(0, -1));
  } else if(theta - 3 * M_PI / 2 < thetaThreshold) {
    acts.push_back(SearchMove(-1, -1));
    acts.push_back(SearchMove(0, -1));
    acts.push_back(SearchMove(1, -1));
  } else if(theta - 7 * M_PI / 4 < thetaThreshold) {
    acts.push_back(SearchMove(1, 0));
    acts.push_back(SearchMove(1, -1));
    acts.push_back(SearchMove(0, -1));
  }
  acts.remove_if([this, state](SearchMove m){ return !isActionValid(m, state); });
  cout << acts.size() << endl;
  return acts;
}

SearchLocation IGVCSearchProblemDiscrete::getResult(SearchLocation state, SearchMove action)
{
  SearchLocation result;
  result.X = state.X;
  result.Y = state.Y;
  cout << "action X = " << action.X << " y = " << action.Y << std::endl;
  cout << "result before X = " << result.X << " y = " << result.Y << std::endl;
  result.X += action.X;
  result.Y += action.Y;
  if(result.X != 0)  {
    result.Theta = tan(result.Y / result.X);
  } else {
    result.Theta = result.Y == 1 ? M_PI / 4 : 3 * M_PI / 2;
  }
  cout << "result after X = " << result.X << " y = " << result.Y << std::endl;
  return result;
}
