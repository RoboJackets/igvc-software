#include "igvcsearchproblemdiscrete.h"
#include <math.h>

bool IGVCSearchProblemDiscrete::isActionValid(SearchMove& move, SearchLocation start_state)
{
  //std::cout << start_state.X + move.X << "," << start_state.Y + move.Y;
  double x = start_state.X + move.X;
  double y = start_state.Y + move.Y;
  if(x < 0 || y < 0 || x >= Map->image.size().width || y >= Map->image.size().height) {
    return false;
  }
  double min_val;
  double max_val;
  double sep = CSpace / Resolution;
  cv::Mat subsection = Map->image(cv::Range(max(x - sep, 0.0), min(static_cast<int>(x + sep) + 1, Map->image.size().height)), cv::Range(max(y - sep, 0.0), min(static_cast<int>(y + sep) + 1, Map->image.size().width)));
  cv::minMaxLoc(subsection, &min_val, &max_val);
  if(max_val > ProbabilityThreshold * 255) {
    //std::cout << " false" << std::endl;
    return false;
  }
  //std::cout << " true" << std::endl;
  return true;
}

std::list<SearchMove> IGVCSearchProblemDiscrete::getActions(SearchLocation state, SearchLocation robot_position)
{
  std::list<SearchMove> acts;
  double theta = state.Theta;
  double thetaThreshold = 0.1;
  //std::cout << "theta = " << theta << " ";
  if(abs(theta) < thetaThreshold) {
    //std::cout << "1" <<std::endl;
    acts.push_back(SearchMove(1, 1));
    acts.push_back(SearchMove(1, 0));
    acts.push_back(SearchMove(1, -1));
  } else if(abs(theta - M_PI / 4) < thetaThreshold) {
    //std::cout << "2"<<std::endl;
    acts.push_back(SearchMove(1, 1));
    acts.push_back(SearchMove(1, 0));
    acts.push_back(SearchMove(0, 1));
  } else if(abs(theta - M_PI / 2) < thetaThreshold) {
    //std::cout << "3"<<std::endl;
    acts.push_back(SearchMove(1, 1));
    acts.push_back(SearchMove(0, 1));
    acts.push_back(SearchMove(-1, 1));
  } else if(abs(theta - 3 * M_PI / 4) < thetaThreshold) {
    //std::cout << "4"<<std::endl;
    acts.push_back(SearchMove(0, 1));
    acts.push_back(SearchMove(-1, 1));
    acts.push_back(SearchMove(-1, 0));
  } else if(abs(abs(theta) - M_PI) < thetaThreshold) {
    //std::cout << "5"<<std::endl;
    acts.push_back(SearchMove(-1, 0));
    acts.push_back(SearchMove(-1, 1));
    acts.push_back(SearchMove(-1, -1));
  } else if(abs(abs(theta) - 3 * M_PI / 4) < thetaThreshold) {
    //std::cout << "6"<<std::endl;
    acts.push_back(SearchMove(-1, -1));
    acts.push_back(SearchMove(-1, 0));
    acts.push_back(SearchMove(0, -1));
  } else if(abs(abs(theta) - M_PI / 2) < thetaThreshold) {
    //std::cout << "7"<<std::endl;
    acts.push_back(SearchMove(-1, -1));
    acts.push_back(SearchMove(0, -1));
  } else if(abs(abs(theta) - M_PI / 4) < thetaThreshold) {
    //std::cout << "8"<<std::endl;
    acts.push_back(SearchMove(1, 0));
    acts.push_back(SearchMove(1, -1));
    acts.push_back(SearchMove(0, -1));
  } else {
    std::cout << "\n\n\n\n\nfail\n\n\n\n\n" << std::endl;
  }
  acts.remove_if([this, state](SearchMove m){ return !isActionValid(m, state); });
  return acts;
}

SearchLocation IGVCSearchProblemDiscrete::getResult(SearchLocation state, SearchMove action)
{
  SearchLocation result;
  result.X = state.X;
  result.Y = state.Y;
  result.X += action.X;
  result.Y += action.Y;
  if(action.X == 1 && action.Y == 0) {
    result.Theta = 0;
  } else if(action.X == 1 && action.Y == 1) {
    result.Theta = M_PI/4;
  } else if(action.X == 0 && action.Y == 1) {
    result.Theta = M_PI/2;
  } else if(action.X == -1 && action.Y == 1) {
    result.Theta = 3 * M_PI/4;
  } else if(action.X == -1 && action.Y == 0) {
    result.Theta = M_PI;
  } else if(action.X == -1 && action.Y == -1) {
    result.Theta = -3*M_PI/4;
  } else if(action.X == 0 && action.Y == -1) {
    result.Theta = -M_PI/2;
  } else if(action.X == 1 && action.Y == -1) {
    result.Theta = -M_PI/4;
  }
  //std::cout << " x,y = " << action.X << ", " << action.Y << ", " << result.Theta;
  return result;
}
